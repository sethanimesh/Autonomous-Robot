import hashlib
import io
import threading

import pytest

from robot.jetson.perception.speech_delivery import (
    AlsaSpeechPlayer,
    MacSpeechBackend,
    PreparedDeliveryStore,
    SpeechDeliveryError,
    delivery_gate,
    delivery_preflight,
)
from robot.jetson.perception import speech_delivery as player_module


MP3 = b"\xff\xfb\x90\x64"


def prepared(store=None, revision=0):
    store = store or PreparedDeliveryStore()
    ticket = store.begin("person-a", "profile-r1", "draft-a", revision, "Dinner is ready.")
    summary = store.complete(ticket, MP3)
    return store, ticket, summary


def test_store_binds_approval_to_person_and_message_revisions():
    store, ticket, summary = prepared()
    audio = store.get(ticket.delivery_id, "person-a", "profile-r1", "draft-a", 0)
    assert audio["audio"] == MP3
    assert summary["text_sha256"] == hashlib.sha256(ticket.text.encode()).hexdigest()
    with pytest.raises(SpeechDeliveryError, match="changed"):
        store.get(ticket.delivery_id, "person-b", "profile-r1")
    with pytest.raises(SpeechDeliveryError, match="changed"):
        store.get(ticket.delivery_id, "person-a", "profile-r2")
    with pytest.raises(SpeechDeliveryError, match="changed"):
        store.get(ticket.delivery_id, "person-a", "profile-r1", "draft-a", 1)


def test_stale_synthesis_and_equal_revision_clear_cannot_replace_newer_approval():
    store = PreparedDeliveryStore()
    old = store.begin("person-a", "profile-r1", "draft-a", 4, "Old text")
    new = store.begin("person-a", "profile-r1", "draft-a", 5, "Edited text")
    assert store.invalidate("draft-a", 5) is False
    with pytest.raises(SpeechDeliveryError, match="changed while"):
        store.complete(old, MP3)
    store.complete(new, MP3)
    assert store.invalidate("draft-a", 5) is False
    assert store.get(new.delivery_id, "person-a", "profile-r1")["text"] == "Edited text"
    assert store.invalidate("draft-a", 6) is True


def test_prepared_delivery_expires_in_memory():
    now = [0.0]
    store = PreparedDeliveryStore(clock=lambda: now[0])
    ticket = store.begin("person-a", "profile-r1", "draft-a", 0, "Dinner is ready.")
    store.complete(ticket, MP3)
    now[0] = 901.0
    with pytest.raises(SpeechDeliveryError, match="expired"):
        store.get(ticket.delivery_id, "person-a", "profile-r1")


@pytest.mark.parametrize(
    "changes",
    [
        {"delivery": None},
        {"delivery_id": "other"},
        {"profile_id": "person-b"},
        {"profile_revision": "profile-r2"},
        {"previewed_delivery_id": None},
        {"audio_available": False},
    ],
)
def test_delivery_start_requires_current_prepared_preview_and_audio(changes):
    values = {
        "delivery": {"delivery_id": "delivery-a", "profile_id": "person-a", "profile_revision": "profile-r1"},
        "delivery_id": "delivery-a",
        "profile_id": "person-a",
        "profile_revision": "profile-r1",
        "previewed_delivery_id": "delivery-a",
        "audio_available": True,
    }
    values.update(changes)
    reason = delivery_preflight(**values)
    assert reason is not None


def test_delivery_start_accepts_only_matching_ready_speaker_preview():
    assert delivery_preflight(
        {"delivery_id": "delivery-a", "profile_id": "person-a", "profile_revision": "profile-r1"},
        "delivery-a", "person-a", "profile-r1", "delivery-a", True,
    ) is None


def feedback():
    observation = {
        "identity_confirmed": True,
        "identity_source": "clothing",
        "profile_id": "person-a",
        "target_revision": "profile-r1",
        "track_id": "track-7",
        "age_seconds": 0.1,
        "range": {
            "validated": True, "feet_checked": True, "distance_m": 0.6,
            "lower_m": 0.55, "upper_m": 0.65, "consistent_samples": 2,
            "age_seconds": 0.2, "track_id": "track-7", "profile_id": "person-a",
        },
    }
    report = {"outcome": "target_found_at_standoff", "target_observation": dict(observation)}
    robot = {
        "status": "ok", "motion_active": False,
        "motors": {
            "left": {"speed": 0, "commanded_speed": 0, "state": []},
            "right": {"speed": 0, "commanded_speed": 0, "state": []},
        },
    }
    head = {"moving": False, "homing": False}
    return report, observation, robot, head


def test_speech_gate_accepts_unique_current_identity_and_stopped_feedback():
    report, observation, robot, head = feedback()
    assert delivery_gate(report, "person-a", "profile-r1", observation, 0.1,
                         robot, 0.1, head, 0.1, mission_state="found") is None
    assert delivery_gate(report, "person-a", "profile-r1", observation, 0.1,
                         robot, 0.1, head, 0.1, mission_state="failed")
    report["outcome"] = "target_found_at_estimated_standoff"
    assert delivery_gate(report, "person-a", "profile-r1", observation, 0.1,
                         robot, 0.1, head, 0.1)


@pytest.mark.parametrize("failure", ["outcome", "identity", "stale", "profile", "track", "range", "moving", "motor_state", "head"])
def test_speech_gate_rejects_uncertain_identity_or_motion(failure):
    report, observation, robot, head = feedback()
    if failure == "outcome":
        report["outcome"] = "target_found_not_at_standoff"
    elif failure == "identity":
        observation["identity_confirmed"] = False
    elif failure == "stale":
        observation["age_seconds"] = 1.2
    elif failure == "profile":
        observation["target_revision"] = "old-revision"
    elif failure == "track":
        observation["track_id"] = "different-track"
    elif failure == "range":
        report["target_observation"]["range"]["validated"] = False
    elif failure == "moving":
        robot["motors"]["left"].update(speed=2, state=["running"])
    elif failure == "motor_state":
        robot["motors"]["left"]["state"] = ""
    else:
        head["moving"] = True
    assert delivery_gate(report, "person-a", "profile-r1", observation, 0.1,
                         robot, 0.1, head, 0.1)


def test_speech_gate_rejects_stale_robot_and_head_feedback():
    report, observation, robot, head = feedback()
    assert delivery_gate(report, "person-a", "profile-r1", observation, 0.1,
                         robot, 1.6, head, 0.1)
    assert delivery_gate(report, "person-a", "profile-r1", observation, 0.1,
                         robot, 0.1, head, 1.6)


def test_mac_backend_is_loopback_only_and_checks_response_binding_and_digest():
    with pytest.raises(ValueError, match="loopback"):
        MacSpeechBackend("http://192.168.1.8:8766")
    _store, ticket, _summary = prepared()

    class Response:
        headers = {
            "Content-Type": "audio/mpeg",
            "X-Echora-Message-ID": ticket.message_id,
            "X-Echora-Message-Revision": str(ticket.message_revision),
            "X-Echora-Profile-ID": ticket.profile_id,
            "X-Echora-Profile-Revision": ticket.profile_revision,
            "X-Echora-Audio-SHA256": hashlib.sha256(MP3).hexdigest(),
        }

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def read(self, _limit):
            return MP3

    backend = MacSpeechBackend(opener=lambda *_args, **_kwargs: Response())
    assert backend.synthesize(ticket) == MP3
    Response.headers["X-Echora-Message-Revision"] = "99"
    with pytest.raises(SpeechDeliveryError, match="does not match"):
        backend.synthesize(ticket)


def test_operator_stop_interrupts_active_alsa_pipeline(monkeypatch):
    monkeypatch.setattr(player_module.shutil, "which", lambda _tool: "/fake/tool")
    monkeypatch.setattr(player_module.os, "killpg", lambda *_args: (_ for _ in ()).throw(ProcessLookupError()))
    decoder_started = threading.Event()
    processes = []

    class FakeProcess:
        next_pid = 1_000_000_000

        def __init__(self, command):
            self.command = command
            self.pid = FakeProcess.next_pid
            FakeProcess.next_pid += 1
            self.stdin = io.BytesIO()
            self.returncode = None
            self.stopped = threading.Event()

        def poll(self):
            return self.returncode

        def communicate(self, *_args, **_kwargs):
            if "ffmpeg" in self.command[0]:
                decoder_started.set()
                self.stopped.wait(2)
            return None, b""

        def terminate(self):
            self.returncode = -15
            self.stopped.set()

        def wait(self, timeout=None):
            return self.returncode

        def kill(self):
            self.terminate()

    def popen(command, **_kwargs):
        process = FakeProcess(command)
        processes.append(process)
        return process

    player = AlsaSpeechPlayer(popen=popen)
    errors = []

    def run():
        try:
            player.play(MP3)
        except Exception as exc:
            errors.append(exc)

    thread = threading.Thread(target=run)
    thread.start()
    assert decoder_started.wait(1)
    player.stop()
    thread.join(1)
    assert not thread.is_alive()
    assert errors and isinstance(errors[0], SpeechDeliveryError)
    assert "stopped" in str(errors[0]).lower()
    assert player.status()["active"] is False
    assert len(processes) == 2
