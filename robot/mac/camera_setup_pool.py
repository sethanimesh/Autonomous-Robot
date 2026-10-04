"""Camera-view model rotation with persistent cooldowns and one schema."""
import base64
import hashlib
import json
import math
import os
import re
import threading
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from robot.cloud_models import GEMINI_MODEL, GEMINI_THINKING_LEVEL
from robot.mac.camera_setup_advisor import PROMPT, SCHEMA, validate_setup_observations
from robot.mac.person_search_advisor import SEARCH_PROMPT, SEARCH_SCHEMA, validate_search_observations
from robot.mac.vision_scene_advisor import GroqAccessError


class VertexADC:
    """Use the existing Google ADC identity; keep refreshed tokens in memory."""
    def __init__(self):
        import google.auth
        from google.auth.transport.requests import Request as AuthRequest
        self.credentials, default_project = google.auth.default(
            scopes=['https://www.googleapis.com/auth/cloud-platform'])
        self.project = (os.environ.get('GOOGLE_CLOUD_PROJECT') or default_project
                        or getattr(self.credentials, 'quota_project_id', None))
        self.location = os.environ.get('GOOGLE_CLOUD_LOCATION', 'global')
        if not self.project or not re.fullmatch(r'[a-zA-Z0-9_-]+', self.project):
            raise ValueError('ADC needs a Google Cloud project')
        if not re.fullmatch(r'[a-z0-9-]+', self.location):
            raise ValueError('Invalid Vertex location')
        self.transport = AuthRequest()
        self.lock = threading.Lock()

    def authorize(self, model):
        host = 'aiplatform.googleapis.com' if self.location == 'global' else self.location + '-aiplatform.googleapis.com'
        url = 'https://{0}/v1/projects/{1}/locations/{2}/publishers/google/models/{3}:generateContent'.format(
            host, self.project, self.location, model)
        headers = {'Content-Type': 'application/json', 'User-Agent': 'echora-robot/1.0'}
        try:
            with self.lock:
                self.credentials.before_request(lambda **kw: self.transport(timeout=5, **kw),
                                                'POST', url, headers)
        except Exception:
            raise CameraCloudError(503, 30) from None
        return url, headers


def retry_seconds(value, default=30):
    try:
        delay = float(value)
    except (TypeError, ValueError):
        delay = sum(float(number) * dict(ms=.001, s=1, m=60, h=3600)[unit]
                    for number, unit in re.findall(r'([0-9.]+)(ms|s|m|h)', str(value))) or default
    return max(1, min(86400, delay)) if math.isfinite(delay) else default


class CameraCloudError(RuntimeError):
    def __init__(self, status=429, retry_after=30):
        self.status = status
        self.quota = {'retry-after': str(math.ceil(retry_after))}
        super().__init__('Camera vision models are temporarily unavailable; retry automatically')


class GeminiCameraSetupAdvisor:
    provider = 'gemini'

    def __init__(self, model=GEMINI_MODEL, adc=None, timeout=8, prompt=PROMPT, search=False):
        self.model, self.adc, self.timeout, self.prompt = model, adc or VertexADC(), timeout, prompt
        self.search = search

    def interpret(self, frames):
        schema = json.loads(json.dumps(SEARCH_SCHEMA if self.search else SCHEMA))
        observations = schema['properties']['observations']
        observations.update(minItems=len(frames), maxItems=len(frames))
        observations['items']['properties']['frame_index']['enum'] = list(range(1, len(frames) + 1))
        validator = validate_search_observations if self.search else validate_setup_observations
        result = self.interpret_structured(frames, self.prompt, schema,
            lambda value: validator(value, len(frames)))
        result['observations'] = result.pop('interpretation')
        return result

    def interpret_structured(self, frames, prompt, schema, validator, context=''):
        """Share the existing ADC transport without granting models actuator access."""
        if not 1 <= len(frames) <= 3:
            raise ValueError('Send one to three camera views')
        parts = []
        if context:
            parts.append({'text': context})
        for index, frame in enumerate(frames, 1):
            if not isinstance(frame, bytes) or not 100 <= len(frame) <= 4_000_000:
                raise ValueError('Invalid camera image')
            parts.extend([{'text': 'Image {0}.'.format(index)},
                          {'inlineData': {'mimeType': 'image/jpeg', 'data': base64.b64encode(frame).decode('ascii')}}])
        generation = dict(temperature=0, maxOutputTokens=2048,
                          responseMimeType='application/json', responseJsonSchema=schema)
        if self.model.startswith('gemini-3'):
            generation['thinkingConfig'] = {'thinkingLevel': GEMINI_THINKING_LEVEL}
        elif self.model == 'gemini-2.5-pro':
            generation['thinkingConfig'] = {'thinkingBudget': 512}
        elif self.model.startswith('gemini-2.5-flash'):
            generation['thinkingConfig'] = {'thinkingBudget': 0}
        payload = dict(systemInstruction={'parts': [{'text': prompt}]},
                       contents=[{'role': 'user', 'parts': parts}], generationConfig=generation)
        url, headers = self.adc.authorize(self.model)
        request = Request(url, data=json.dumps(payload).encode(), headers=headers)
        started = time.monotonic()
        try:
            with urlopen(request, timeout=self.timeout) as response:
                result = json.load(response)
        except HTTPError as exc:
            delay = exc.headers.get('Retry-After')
            try:
                details = json.loads(exc.read(8192)).get('error', {}).get('details', [])
                for item in details:
                    if item.get('@type', '').endswith('RetryInfo'):
                        delay = item.get('retryDelay', delay)
            except (ValueError, AttributeError, TypeError):
                pass
            status = exc.code
            exc.close()
            # Never log Google's response text, request headers, or credentials.
            raise CameraCloudError(status, retry_seconds(delay)) from None
        candidates = result.get('candidates') or []
        if len(candidates) != 1 or candidates[0].get('finishReason') != 'STOP':
            raise ValueError('Gemini did not finish a complete camera-view answer')
        output = ''.join(part.get('text', '') for part in candidates[0].get('content', {}).get('parts', [])
                         if not part.get('thought'))
        observed = validator(json.loads(output))
        return dict(provider='gemini', model=self.model, interpretation=observed,
                    frame_sha256=[hashlib.sha256(frame).hexdigest() for frame in frames],
                    auth='adc', elapsed_seconds=round(time.monotonic() - started, 3), usage=result.get('usageMetadata'),
                    quota={}, advisory_only=True)


class CameraSetupPool:
    """Rotate on success, fall through on failure, skip models until cooldown."""
    def __init__(self, clients=None, clock=time.monotonic, search=False):
        self.prefer_gemini = clients is None
        if clients is None:
            clients = []
            try:
                adc = VertexADC()
            except Exception:
                adc = None
            prompt = PROMPT
            if search:
                prompt = SEARCH_PROMPT
            if adc:
                clients.append(GeminiCameraSetupAdvisor(adc=adc, timeout=20 if search else 8,
                                                       prompt=prompt, search=search))
        self.clients, self.clock = clients, clock
        self.retry_at = [0.0] * len(clients)
        self.cursor = 0
        self.lock = threading.Lock()
        self.last_attempts = []
        self.last_status = 429

    def interpret(self, frames):
        with self.lock:
            if not self.clients:
                raise CameraCloudError(503, 60)
            started = self.clock()
            failures = []
            self.last_attempts = failures
            order = [(self.cursor + offset) % len(self.clients) for offset in range(len(self.clients))]
            if self.prefer_gemini:
                order.sort(key=lambda index: getattr(self.clients[index], 'provider', 'groq') != 'gemini')
            for index in order:
                if self.retry_at[index] > self.clock():
                    continue
                if self.clock() - started > 22:
                    break
                client = self.clients[index]
                provider = getattr(client, 'provider', 'groq')
                try:
                    result = client.interpret(frames)
                    if result.get('provider') not in ('groq', 'gemini'):
                        raise ValueError('Unexpected camera provider')
                except (GroqAccessError, CameraCloudError) as exc:
                    status = exc.status
                    delay = retry_seconds(exc.quota.get('retry-after') or
                                          exc.quota.get('x-ratelimit-reset-tokens'))
                    if status in (400, 401, 403, 404):
                        delay = max(delay, 300)
                    self.retry_at[index] = self.clock() + delay + 1
                except (ValueError, KeyError, TypeError, URLError, TimeoutError, OSError):
                    status = 'invalid_or_unavailable'
                    self.retry_at[index] = self.clock() + 15
                else:
                    self.cursor = (index + 1) % len(self.clients)
                    result['model_fallbacks'] = failures
                    result['pool_elapsed_seconds'] = round(self.clock() - started, 3)
                    return result
                self.last_status = 429 if status == 429 else 503
                failures.append(dict(provider=provider, model=client.model, status=status))
            next_ready = min(self.retry_at) - self.clock()
            # A timeout/schema failure is unavailability, not a quota rejection.
            status = (429 if all(f['status'] == 429 for f in failures) else 503) if failures else self.last_status
            self.last_status = status
            raise CameraCloudError(status, max(1, next_ready))
