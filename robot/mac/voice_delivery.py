"""Loopback-only, narrow ASR and TTS API for robot message delivery."""

import asyncio
import hashlib
import os
import re
from pathlib import Path

from dotenv import load_dotenv
from fastapi import FastAPI, File, Form, HTTPException, Request, UploadFile
from fastapi.responses import JSONResponse, Response
from pydantic import BaseModel, Field

from communication.backend import fish, providers


ROOT = Path(__file__).resolve().parents[2]
load_dotenv(ROOT / "communication" / ".env")
load_dotenv(ROOT / ".env")
fish.load_configuration([ROOT / "communication" / ".env", ROOT / ".env"])

MAX_RECORDING_BYTES = 8 * 1024 * 1024
MAX_MESSAGE_CHARACTERS = 2000
MAX_SYNTHESIS_CALLS = 20
synthesis_calls = 0
synthesis_lock = asyncio.Lock()

app = FastAPI(title="Robot speech", docs_url=None, redoc_url=None)


class SynthesisRequest(BaseModel):
    message_id: str = Field(min_length=1, max_length=128)
    message_revision: int = Field(ge=0)
    profile_id: str = Field(min_length=1, max_length=128)
    profile_revision: str = Field(min_length=1, max_length=128)
    text: str = Field(min_length=1, max_length=MAX_MESSAGE_CHARACTERS)


@app.middleware("http")
async def loopback_tunnel_boundary(request: Request, call_next):
    host = request.headers.get("host", "").split(":", 1)[0]
    if host not in {"127.0.0.1", "localhost", "testserver"}:
        return JSONResponse({"detail": "The robot speech service is loopback-only."}, status_code=403)
    if request.method not in {"GET", "HEAD", "OPTIONS"} and request.headers.get("x-echora-client") != "1":
        return JSONResponse({"detail": "Missing application request header."}, status_code=403)
    try:
        length = int(request.headers.get("content-length", "0"))
    except ValueError:
        return JSONResponse({"detail": "Invalid request size."}, status_code=400)
    limit = MAX_RECORDING_BYTES + 65536 if request.url.path == "/transcribe" else 65536
    if length < 0 or length > limit:
        return JSONResponse({"detail": "Request exceeds the robot speech size limit."}, status_code=413)
    response = await call_next(request)
    response.headers["Cache-Control"] = "no-store"
    response.headers["X-Content-Type-Options"] = "nosniff"
    return response


@app.get("/health")
async def health():
    return {
        "ready": True,
        "transcription_model": providers.ASR_MODEL,
        "fish_configured": fish.status()["configured"],
        "synthesis_calls_remaining": max(0, MAX_SYNTHESIS_CALLS - synthesis_calls),
    }


@app.post("/transcribe")
async def transcribe(file: UploadFile = File(...), language: str = Form("auto")):
    if language != "auto" and not re.fullmatch(r"[a-z]{2,3}", language):
        raise HTTPException(400, "Choose auto, English, or a supported language code.")
    audio = await file.read(MAX_RECORDING_BYTES + 1)
    if not audio or len(audio) > MAX_RECORDING_BYTES:
        raise HTTPException(413, "Use a nonempty recording smaller than 8 MB.")
    content_type = (file.content_type or "application/octet-stream").split(";", 1)[0]
    filename = {
        "audio/webm": "recording.webm",
        "audio/mp4": "recording.mp4",
        "audio/wav": "recording.wav",
        "audio/x-wav": "recording.wav",
        "audio/ogg": "recording.ogg",
        "audio/mpeg": "recording.mp3",
    }.get(content_type, "recording.bin")
    try:
        text, metadata = await providers.transcribe(
            audio,
            filename,
            content_type,
            language,
        )
    except providers.ProviderFailure as exc:
        raise HTTPException(503, exc.message) from None
    if not text:
        raise HTTPException(422, "No words were transcribed. Record again or type the message.")
    if len(text) > MAX_MESSAGE_CHARACTERS:
        raise HTTPException(422, "The transcript is too long for robot delivery. Shorten it before approval.")
    return {"text": text, "metadata": metadata}


@app.post("/synthesize")
async def synthesize(request: SynthesisRequest):
    global synthesis_calls
    if len(request.text.encode("utf-8")) > 12000:
        raise HTTPException(413, "The message is too long for speech generation.")
    if not fish.status()["configured"]:
        raise HTTPException(503, "Fish Audio is not configured on the Mac speech service.")
    async with synthesis_lock:
        if synthesis_calls >= MAX_SYNTHESIS_CALLS:
            raise HTTPException(429, "The robot speech service reached its local synthesis limit.")
        synthesis_calls += 1
    confirmed = {
        "speech_text": request.text,
        "delivery": {"tone": "neutral", "rate": 1.0},
    }
    try:
        payload = fish.payload(confirmed, fish.configuration()[1])
        audio, metadata = await fish.synthesize(payload)
    except providers.ProviderFailure as exc:
        raise HTTPException(503, exc.message) from None
    digest = hashlib.sha256(audio).hexdigest()
    return Response(
        audio,
        media_type="audio/mpeg",
        headers={
            "X-Echora-Message-ID": request.message_id,
            "X-Echora-Message-Revision": str(request.message_revision),
            "X-Echora-Profile-ID": request.profile_id,
            "X-Echora-Profile-Revision": request.profile_revision,
            "X-Echora-Audio-SHA256": digest,
            "X-Echora-TTS-Model": metadata.get("model", fish.MODEL),
        },
    )
