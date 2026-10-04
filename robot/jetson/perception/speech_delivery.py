"""Transient message approval, Mac speech service access, and Jetson playback."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
import os
import re
import shutil
import signal
import subprocess
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid


MAX_AUDIO_BYTES = 5 * 1024 * 1024
MAX_RECORDING_BYTES = 8 * 1024 * 1024
MAX_MESSAGE_CHARACTERS = 2000
DELIVERY_TTL_SECONDS = 15 * 60
FRESH_TARGET_SECONDS = 1.0
FRESH_ROBOT_SECONDS = 1.5
FRESH_HEAD_SECONDS = 1.5


class SpeechDeliveryError(ValueError):
    """A message cannot be safely prepared or played."""


def validate_mp3(audio):
    if not isinstance(audio, (bytes, bytearray)) or not 4 <= len(audio) <= MAX_AUDIO_BYTES:
        raise SpeechDeliveryError("The speech service returned empty or oversized audio.")
    audio = bytes(audio)
    if not (audio[:3] == b"ID3" or (audio[0] == 0xFF and audio[1] & 0xE0 == 0xE0)):
        raise SpeechDeliveryError("The speech service did not return valid MP3 audio.")
    return audio


@dataclass(frozen=True)
class PreparationTicket:
    delivery_id: str
    generation: int
    profile_id: str
    profile_revision: str
    message_id: str
    message_revision: int
    text: str
    text_sha256: str


class PreparedDeliveryStore:
    """Hold one approved, recipient-bound audio payload in memory."""

    def __init__(self, clock=None):
        self.clock = clock or time.monotonic
        self.lock = threading.RLock()
        self.generation = 0
        self.active = None

    def begin(self, profile_id, profile_revision, message_id, message_revision, text):
        if not isinstance(profile_id, str) or not profile_id or len(profile_id) > 128:
            raise SpeechDeliveryError("Select an enrolled person before approving a message.")
        if not isinstance(profile_revision, str) or not profile_revision or len(profile_revision) > 128:
            raise SpeechDeliveryError("The selected profile revision is unavailable.")
        if not isinstance(message_id, str) or not message_id or len(message_id) > 128:
            raise SpeechDeliveryError("The message draft identifier is invalid.")
        if type(message_revision) is not int or message_revision < 0:
            raise SpeechDeliveryError("The message revision is invalid.")
        if not isinstance(text, str) or not text.strip() or len(text) > MAX_MESSAGE_CHARACTERS:
            raise SpeechDeliveryError("Enter a message of 1 to 2,000 characters.")
        if len(text.encode("utf-8")) > 12000:
            raise SpeechDeliveryError("The message is too long for speech generation.")
        with self.lock:
            self.generation += 1
            ticket = PreparationTicket(
                delivery_id=uuid.uuid4().hex,
                generation=self.generation,
                profile_id=profile_id,
                profile_revision=profile_revision,
                message_id=message_id,
                message_revision=message_revision,
                text=text,
                text_sha256=hashlib.sha256(text.encode("utf-8")).hexdigest(),
            )
            self.active = {
                "ticket": ticket,
                "state": "preparing",
                "audio": None,
                "created_at": self.clock(),
            }
            return ticket

    def complete(self, ticket, audio):
        audio = validate_mp3(audio)
        with self.lock:
            current = self.active
            if (current is None or current["ticket"] != ticket
                    or ticket.generation != self.generation):
                raise SpeechDeliveryError("The message changed while speech was being prepared. Approve it again.")
            current["audio"] = audio
            current["state"] = "ready"
            current["audio_sha256"] = hashlib.sha256(audio).hexdigest()
            return self._summary_locked(current)

    def fail(self, ticket):
        with self.lock:
            if self.active and self.active["ticket"] == ticket:
                self.generation += 1
                self.active = None

    def invalidate(self, message_id=None, message_revision=None, force=False):
        with self.lock:
            current = self.active
            if current is None:
                return False
            ticket = current["ticket"]
            should_clear = force or (message_id is None and message_revision is None)
            if message_id == ticket.message_id and type(message_revision) is int:
                # Delayed invalidations from an older UI event cannot cancel a newer approval.
                should_clear = message_revision > ticket.message_revision
            if should_clear:
                self.generation += 1
                self.active = None
                return True
            return False

    def get(self, delivery_id, profile_id, profile_revision, message_id=None, message_revision=None):
        with self.lock:
            current = self.active
            if current is None:
                raise SpeechDeliveryError("Approve the message and wait for speech preparation first.")
            if self.clock() - current["created_at"] > DELIVERY_TTL_SECONDS:
                self.generation += 1
                self.active = None
                raise SpeechDeliveryError("The prepared message expired. Approve it again.")
            ticket = current["ticket"]
            if current["state"] != "ready" or current["audio"] is None:
                raise SpeechDeliveryError("Speech is still being prepared. Wait before starting.")
            if (delivery_id != ticket.delivery_id or profile_id != ticket.profile_id
                    or profile_revision != ticket.profile_revision
                    or (message_id is not None and message_id != ticket.message_id)
                    or (message_revision is not None and message_revision != ticket.message_revision)):
                raise SpeechDeliveryError("The approved message or selected person changed. Approve it again.")
            return {
                "delivery_id": ticket.delivery_id,
                "profile_id": ticket.profile_id,
                "profile_revision": ticket.profile_revision,
                "message_id": ticket.message_id,
                "message_revision": ticket.message_revision,
                "text": ticket.text,
                "text_sha256": ticket.text_sha256,
                "audio": current["audio"],
                "audio_sha256": current["audio_sha256"],
            }

    def consume(self, delivery_id):
        with self.lock:
            if self.active and self.active["ticket"].delivery_id == delivery_id:
                self.generation += 1
                self.active = None

    def snapshot(self):
        with self.lock:
            self._expire_locked()
            if not self.active:
                return {"state": "idle", "delivery_id": None}
            return self._summary_locked(self.active)

    def _expire_locked(self):
        if self.active and self.clock() - self.active["created_at"] > DELIVERY_TTL_SECONDS:
            self.generation += 1
            self.active = None

    @staticmethod
    def _summary_locked(current):
        ticket = current["ticket"]
        return {
            "state": current["state"],
            "delivery_id": ticket.delivery_id,
            "profile_id": ticket.profile_id,
            "profile_revision": ticket.profile_revision,
            "message_id": ticket.message_id,
            "message_revision": ticket.message_revision,
            "text_sha256": ticket.text_sha256,
            "audio_sha256": current.get("audio_sha256"),
        }


class MacSpeechBackend:
    """Call the Mac-only FastAPI service through its loopback SSH forward."""

    def __init__(self, base_url="http://127.0.0.1:18766", opener=None):
        parsed = urllib.parse.urlparse(base_url)
        if (parsed.scheme != "http" or parsed.hostname not in {"127.0.0.1", "localhost"}
                or not parsed.port or parsed.path not in {"", "/"}):
            raise ValueError("Speech backend URL must use the loopback SSH tunnel.")
        self.base_url = base_url.rstrip("/")
        self.opener = opener or urllib.request.urlopen

    def transcribe(self, audio, content_type="audio/webm", language="auto"):
        if not isinstance(audio, bytes) or not audio or len(audio) > MAX_RECORDING_BYTES:
            raise SpeechDeliveryError("Use a nonempty recording smaller than 8 MB.")
        if not re.fullmatch(r"[A-Za-z0-9.+/-]{1,80}", content_type or ""):
            content_type = "application/octet-stream"
        if language != "auto" and not re.fullmatch(r"[a-z]{2,3}", str(language)):
            raise SpeechDeliveryError("The selected transcription language is invalid.")
        boundary = "echora-" + uuid.uuid4().hex
        chunks = [
            ("--" + boundary + "\r\nContent-Disposition: form-data; name=\"language\"\r\n\r\n"
             + language + "\r\n").encode("ascii"),
            ("--" + boundary + "\r\nContent-Disposition: form-data; name=\"file\"; filename=\"recording.webm\"\r\n"
             + "Content-Type: " + content_type + "\r\n\r\n").encode("ascii"),
            audio,
            ("\r\n--" + boundary + "--\r\n").encode("ascii"),
        ]
        request = urllib.request.Request(
            self.base_url + "/transcribe",
            data=b"".join(chunks),
            headers={
                "Content-Type": "multipart/form-data; boundary=" + boundary,
                "X-Echora-Client": "1",
            },
            method="POST",
        )
        value = self._json_request(request, timeout=55)
        text = value.get("text")
        if not isinstance(text, str) or len(text) > MAX_MESSAGE_CHARACTERS:
            raise SpeechDeliveryError("The speech backend returned an invalid transcript.")
        return {"text": text, "metadata": value.get("metadata", {})}

    def synthesize(self, ticket):
        payload = {
            "message_id": ticket.message_id,
            "message_revision": ticket.message_revision,
            "profile_id": ticket.profile_id,
            "profile_revision": ticket.profile_revision,
            "text": ticket.text,
        }
        request = urllib.request.Request(
            self.base_url + "/synthesize",
            data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            headers={"Content-Type": "application/json", "X-Echora-Client": "1"},
            method="POST",
        )
        try:
            with self.opener(request, timeout=70) as response:
                if not response.headers.get("Content-Type", "").startswith("audio/mpeg"):
                    raise SpeechDeliveryError("The speech backend returned an unexpected audio format.")
                if (response.headers.get("X-Echora-Message-ID") != ticket.message_id
                        or response.headers.get("X-Echora-Message-Revision") != str(ticket.message_revision)
                        or response.headers.get("X-Echora-Profile-ID") != ticket.profile_id
                        or response.headers.get("X-Echora-Profile-Revision") != ticket.profile_revision):
                    raise SpeechDeliveryError("The generated audio does not match the approved message and person.")
                audio = response.read(MAX_AUDIO_BYTES + 1)
                expected_digest = response.headers.get("X-Echora-Audio-SHA256", "")
        except SpeechDeliveryError:
            raise
        except Exception as exc:
            raise SpeechDeliveryError("Mac speech service unavailable through the SSH tunnel: {0}".format(self._reason(exc))) from None
        audio = validate_mp3(audio)
        if not re.fullmatch(r"[0-9a-f]{64}", expected_digest) or hashlib.sha256(audio).hexdigest() != expected_digest:
            raise SpeechDeliveryError("The generated audio checksum did not match the Mac response.")
        return audio

    def _json_request(self, request, timeout):
        try:
            with self.opener(request, timeout=timeout) as response:
                result = json.loads(response.read(65536).decode("utf-8"))
        except urllib.error.HTTPError as exc:
            detail = ""
            try:
                value = json.loads(exc.read(16384).decode("utf-8"))
                detail = value.get("detail", "")
            except Exception:
                pass
            raise SpeechDeliveryError(detail or "Mac speech service rejected the request.") from None
        except Exception as exc:
            raise SpeechDeliveryError("Mac speech service unavailable through the SSH tunnel: {0}".format(self._reason(exc))) from None
        if not isinstance(result, dict):
            raise SpeechDeliveryError("Mac speech service returned an invalid response.")
        return result

    @staticmethod
    def _reason(exc):
        if isinstance(exc, urllib.error.URLError):
            return str(exc.reason)[:160]
        return str(exc)[:160] or type(exc).__name__


class AlsaSpeechPlayer:
    """Decode MP3 with FFmpeg and play PCM to one configured ALSA device."""

    def __init__(self, device="default", ffmpeg="ffmpeg", aplay="aplay", popen=None):
        self.device = str(device)
        self.ffmpeg = str(ffmpeg)
        self.aplay = str(aplay)
        self.popen = popen or subprocess.Popen
        self.lock = threading.RLock()
        self.processes = []
        self.active = False
        self.cancelled = False

    def status(self):
        available = bool(shutil.which(self.ffmpeg) and shutil.which(self.aplay))
        with self.lock:
            active = self.active
        return {
            "available": available,
            "active": active,
            "device": self.device,
            "message": "Audio tools ready; speaker connection is checked by a test tone." if available
                       else "Install FFmpeg and ALSA aplay on the Jetson.",
        }

    def play(self, audio, cancel_event=None):
        audio = validate_mp3(audio)
        if cancel_event is not None and cancel_event.is_set():
            raise SpeechDeliveryError("Speech playback was stopped.")
        if not self.status()["available"]:
            raise SpeechDeliveryError("Install FFmpeg and ALSA aplay on the Jetson before delivery.")
        with self.lock:
            if self.active:
                raise SpeechDeliveryError("Robot audio is already playing.")
            self.active = True
            self.cancelled = False
            self.processes = []
        player = decoder = None
        try:
            player = self.popen(
                [self.aplay, "-q", "-D", self.device, "-"],
                stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE, start_new_session=True,
            )
            self._register(player)
            if cancel_event is not None and cancel_event.is_set():
                self._terminate_all()
                raise SpeechDeliveryError("Speech playback was stopped.")
            decoder = self.popen(
                [self.ffmpeg, "-nostdin", "-hide_banner", "-loglevel", "error",
                 "-i", "pipe:0", "-f", "wav", "-acodec", "pcm_s16le", "pipe:1"],
                stdin=subprocess.PIPE, stdout=player.stdin,
                stderr=subprocess.PIPE, start_new_session=True,
            )
            player.stdin.close()
            # communicate() otherwise tries to flush this already-closed pipe.
            player.stdin = None
            self._register(decoder)
            if cancel_event is not None and cancel_event.is_set():
                self._terminate_all()
                raise SpeechDeliveryError("Speech playback was stopped.")
            _stdout, ffmpeg_error = decoder.communicate(audio, timeout=240)
            player_error = player.communicate(timeout=5)[1]
            if self.cancelled or (cancel_event is not None and cancel_event.is_set()):
                raise SpeechDeliveryError("Speech playback was stopped.")
            if decoder.returncode != 0:
                raise SpeechDeliveryError("FFmpeg could not decode the prepared message.")
            if player.returncode != 0:
                reason = (player_error or b"").decode("utf-8", "replace")[-200:].strip()
                raise SpeechDeliveryError("ALSA playback failed on {0}{1}".format(self.device, ": " + reason if reason else "."))
            return True
        except SpeechDeliveryError:
            raise
        except (OSError, subprocess.SubprocessError, BrokenPipeError) as exc:
            raise SpeechDeliveryError("Audio playback failed: {0}".format(str(exc)[:180])) from None
        finally:
            self._terminate_all()
            with self.lock:
                self.processes = []
                self.active = False

    def play_test_tone(self):
        if not self.status()["available"]:
            raise SpeechDeliveryError("Install FFmpeg and ALSA aplay on the Jetson before testing the speaker.")
        with self.lock:
            if self.active:
                raise SpeechDeliveryError("Robot audio is already playing.")
            self.active = True
            self.cancelled = False
            self.processes = []
        player = decoder = None
        try:
            player = self.popen(
                [self.aplay, "-q", "-D", self.device, "-"],
                stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE, start_new_session=True,
            )
            self._register(player)
            decoder = self.popen(
                [self.ffmpeg, "-nostdin", "-hide_banner", "-loglevel", "error",
                 "-f", "lavfi", "-i", "sine=frequency=880:duration=0.35",
                 "-f", "wav", "-acodec", "pcm_s16le", "pipe:1"],
                stdin=subprocess.DEVNULL, stdout=player.stdin,
                stderr=subprocess.PIPE, start_new_session=True,
            )
            player.stdin.close()
            player.stdin = None
            self._register(decoder)
            decoder.communicate(timeout=5)
            _stdout, player_error = player.communicate(timeout=5)
            if self.cancelled:
                raise SpeechDeliveryError("Speaker test was stopped.")
            if decoder.returncode != 0:
                raise SpeechDeliveryError("FFmpeg could not generate the speaker test tone.")
            if player.returncode != 0:
                reason = (player_error or b"").decode("utf-8", "replace")[-200:].strip()
                raise SpeechDeliveryError("ALSA playback failed on {0}{1}".format(self.device, ": " + reason if reason else "."))
            return True
        except SpeechDeliveryError:
            raise
        except (OSError, subprocess.SubprocessError, BrokenPipeError) as exc:
            raise SpeechDeliveryError("Speaker test failed: {0}".format(str(exc)[:180])) from None
        finally:
            self._terminate_all()
            with self.lock:
                self.processes = []
                self.active = False

    def stop(self):
        with self.lock:
            self.cancelled = True
        self._terminate_all()

    def _register(self, process):
        with self.lock:
            self.processes.append(process)
            cancelled = self.cancelled
        if cancelled:
            self._terminate(process)

    def _terminate_all(self):
        with self.lock:
            processes = list(self.processes)
        for process in processes:
            self._terminate(process)

    @staticmethod
    def _terminate(process):
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except (OSError, ProcessLookupError):
            try:
                process.terminate()
            except OSError:
                pass
        try:
            process.wait(timeout=1)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except (OSError, ProcessLookupError):
                process.kill()


def delivery_gate(report, profile_id, profile_revision, observation, observation_age,
                  robot_status, robot_status_age, head_status, head_status_age,
                  mission_state=None):
    """Return a reason unless the final mission and live feedback still agree."""
    if mission_state is not None and mission_state != "found":
        return "The mission did not complete successfully; the robot stayed silent."
    if not isinstance(report, dict) or report.get("outcome") != "target_found_at_standoff":
        return "Delivery needs a completed approach with a measured, validated standoff; the robot stayed silent."
    reported = report.get("target_observation")
    if not _valid_identity(reported, profile_id, profile_revision):
        return "The mission report does not confirm the selected person's identity."
    distance = reported.get("range")
    if not isinstance(distance, dict) or distance.get("validated") is not True or distance.get("feet_checked") is not True:
        return "A measured person-to-robot distance is not validated yet; the robot stayed silent."
    values = [distance.get(key) for key in ("distance_m", "lower_m", "upper_m")]
    if (any(type(value) not in (int, float) or not math.isfinite(value) for value in values)
            or not 0.5 <= values[1] <= values[0] <= values[2] <= 0.7
            or type(distance.get("consistent_samples")) is not int or distance["consistent_samples"] < 2
            or type(distance.get("age_seconds")) not in (int, float)
            or not math.isfinite(distance["age_seconds"]) or not 0 <= distance["age_seconds"] <= 3
            or distance.get("track_id") != reported.get("track_id")
            or distance.get("profile_id") != profile_id):
        return "The measured standoff is stale, inconsistent, or outside the required validated interval."
    if not _valid_identity(observation, profile_id, profile_revision):
        return "The selected person is no longer uniquely identified."
    if reported.get("track_id") and reported.get("track_id") != observation.get("track_id"):
        return "The current person track differs from the mission's final track."
    try:
        if (not isinstance(observation_age, (int, float)) or not math.isfinite(observation_age) or observation_age < 0
                or observation_age > FRESH_TARGET_SECONDS
                or not isinstance(observation.get("age_seconds"), (int, float))
                or not math.isfinite(observation["age_seconds"])
                or observation["age_seconds"] < 0
                or observation["age_seconds"] > FRESH_TARGET_SECONDS):
            return "The selected-person observation is stale."
    except (TypeError, ValueError):
        return "The selected-person observation is invalid."
    if (not isinstance(robot_status, dict) or robot_status.get("status") != "ok"
            or robot_status.get("motion_active") is not False
            or not isinstance(robot_status_age, (int, float))
            or not math.isfinite(robot_status_age)
            or robot_status_age < 0 or robot_status_age > FRESH_ROBOT_SECONDS):
        return "Fresh stopped robot feedback is unavailable."
    motors = robot_status.get("motors")
    if not isinstance(motors, dict):
        return "Stopped track feedback is unavailable."
    for role in ("left", "right"):
        motor = motors.get(role)
        state = motor.get("state") if isinstance(motor, dict) else None
        if isinstance(state, str):
            state_known = bool(state.strip())
            running = "running" in state.lower()
        elif isinstance(state, (list, tuple, set)):
            state_known = all(isinstance(item, str) for item in state)
            running = any(str(item).lower() == "running" for item in state)
        else:
            state_known = False
            running = True
        if (not isinstance(motor, dict) or not state_known
                or type(motor.get("speed")) not in (int, float)
                or not math.isfinite(motor["speed"]) or abs(motor["speed"]) > 1
                or type(motor.get("commanded_speed")) not in (int, float)
                or not math.isfinite(motor["commanded_speed"]) or motor["commanded_speed"] != 0
                or running):
            return "The tracks have not reported a stable stop."
    if (not isinstance(head_status, dict) or head_status.get("moving") is not False
            or head_status.get("homing") is not False
            or not isinstance(head_status_age, (int, float))
            or not math.isfinite(head_status_age)
            or head_status_age < 0 or head_status_age > FRESH_HEAD_SECONDS):
        return "Fresh stopped camera feedback is unavailable."
    return None


def delivery_preflight(delivery, delivery_id, profile_id, profile_revision,
                       previewed_delivery_id, audio_available):
    """Require one ready, speaker-previewed approval for the current profile."""
    if not isinstance(delivery, dict) or delivery.get("delivery_id") != delivery_id:
        return "Approve the message and wait for speech preparation first."
    if (delivery.get("profile_id") != profile_id
            or delivery.get("profile_revision") != profile_revision):
        return "The approved message is bound to a different profile revision."
    if previewed_delivery_id != delivery_id:
        return "Preview the matching approved message through the robot speaker first."
    if audio_available is not True:
        return "Install FFmpeg and ALSA aplay on the Jetson before delivery."
    return None


def _valid_identity(observation, profile_id, profile_revision):
    return bool(
        isinstance(observation, dict)
        and observation.get("identity_confirmed") is True
        and observation.get("profile_id") == profile_id
        and observation.get("target_revision", observation.get("revision")) == profile_revision
        and observation.get("identity_source") in {"face", "clothing"}
        and observation.get("track_id")
    )
