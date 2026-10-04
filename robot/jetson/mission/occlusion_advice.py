"""Request a bound before/after observation; never sends a movement command."""
import base64
import hashlib
import json
from urllib.request import Request, urlopen
from urllib.parse import urlsplit, urlunsplit
from uuid import uuid4

try:
    from robot.jetson.navigation.navigation_reasoning import validate_occlusion_advice
except ImportError:
    from navigation_reasoning import validate_occlusion_advice


def request_occlusion_advice(previous_jpeg, current_jpeg, search_url, view_changed, timeout=11.):
    parts = urlsplit(search_url)
    url = urlunsplit((parts.scheme, parts.netloc, '/occlusion-advice', '', ''))
    frames = [previous_jpeg, current_jpeg]
    if previous_jpeg == current_jpeg:
        raise RuntimeError('Occlusion comparison did not receive a different image')
    request_id = uuid4().hex
    body = dict(request_id=request_id, view_changed=view_changed,
                frames_base64=[base64.b64encode(frame).decode('ascii') for frame in frames])
    request = Request(url, data=json.dumps(body).encode(), headers={'Content-Type': 'application/json'})
    try:
        with urlopen(request, timeout=timeout) as response:
            result = json.load(response)
        if (not isinstance(result, dict) or result.get('request_id') != request_id
                or result.get('frame_sha256') != [hashlib.sha256(frame).hexdigest() for frame in frames]
                or result.get('provider') != 'gemini' or result.get('advisory_only') is not True):
            raise ValueError('Unbound occlusion answer')
        validate_occlusion_advice(result.get('interpretation'))
    except Exception:
        raise RuntimeError('Occlusion interpretation unavailable or invalid') from None
    return result
