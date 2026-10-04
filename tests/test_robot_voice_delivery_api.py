import hashlib

from fastapi.testclient import TestClient

from communication.backend import providers
from communication.backend import fish
from robot.mac import voice_delivery as module


HEADERS = {"X-Echora-Client": "1"}
MP3 = b"\xff\xfb\x90\x64"


def test_voice_service_is_loopback_only_and_requires_client_header():
    client = TestClient(module.app)
    assert client.get("/health").status_code == 200
    assert client.post("/transcribe", headers={"host": "example.org"}).status_code == 403
    assert client.post("/transcribe", headers={"host": "127.0.0.1"}).status_code == 403


def test_transcription_uses_existing_mac_asr_and_returns_editable_text(monkeypatch):
    async def transcribe(audio, filename, content_type, language):
        assert audio == b"recording"
        assert filename == "recording.webm"
        assert content_type == "audio/webm"
        assert language == "auto"
        return "Dinner is ready.", {"model": "test-asr"}

    monkeypatch.setattr(providers, "transcribe", transcribe)
    client = TestClient(module.app)
    response = client.post(
        "/transcribe", headers=HEADERS, data={"language": "auto"},
        files={"file": ("recording.webm", b"recording", "audio/webm")},
    )
    assert response.status_code == 200
    assert response.json()["text"] == "Dinner is ready."
    assert response.json()["metadata"]["model"] == "test-asr"


def test_synthesis_reuses_fish_and_echoes_revision_binding(monkeypatch):
    monkeypatch.setattr(module, "synthesis_calls", 0)
    monkeypatch.setattr(fish, "status", lambda: {"configured": True})
    monkeypatch.setattr(fish, "configuration", lambda: ("secret", "voice-id"))

    def payload(confirmed, voice):
        assert confirmed == {"speech_text": "Dinner is ready.", "delivery": {"tone": "neutral", "rate": 1.0}}
        assert voice == "voice-id"
        return {"text": confirmed["speech_text"]}

    async def synthesize(request):
        assert request == {"text": "Dinner is ready."}
        return MP3, {"model": "fish-test"}

    monkeypatch.setattr(fish, "payload", payload)
    monkeypatch.setattr(fish, "synthesize", synthesize)
    client = TestClient(module.app)
    response = client.post("/synthesize", headers=HEADERS, json={
        "message_id": "draft-a", "message_revision": 3,
        "profile_id": "person-a", "profile_revision": "profile-r1",
        "text": "Dinner is ready.",
    })
    assert response.status_code == 200
    assert response.headers["content-type"].startswith("audio/mpeg")
    assert response.headers["x-echora-message-id"] == "draft-a"
    assert response.headers["x-echora-message-revision"] == "3"
    assert response.headers["x-echora-profile-id"] == "person-a"
    assert response.headers["x-echora-profile-revision"] == "profile-r1"
    assert response.headers["x-echora-audio-sha256"] == hashlib.sha256(MP3).hexdigest()
    assert response.content == MP3


def test_unconfigured_fish_rejects_synthesis_without_fallback(monkeypatch):
    monkeypatch.setattr(fish, "status", lambda: {"configured": False})
    client = TestClient(module.app)
    response = client.post("/synthesize", headers=HEADERS, json={
        "message_id": "draft-a", "message_revision": 0,
        "profile_id": "person-a", "profile_revision": "profile-r1", "text": "Hello.",
    })
    assert response.status_code == 503
    assert "Fish Audio is not configured" in response.json()["detail"]
