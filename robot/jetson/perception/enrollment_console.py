#!/usr/bin/env python3
"""Browser console for live and uploaded target enrollment."""

import argparse
import base64
from collections import OrderedDict
import hashlib
import json
import math
import os
import shutil
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String

from camera_controls import jog_degrees
from camera_controls import validate_camera_jog
from camera_control_lease import CameraControlBusy
from camera_control_lease import camera_control_lease
from camera_control_lease import manual_camera_control_available
from face_detections import FaceDetection, select_faces
from face_inference import TensorRTMultiOutputBackend, YuNetFaceDetector
from face_observations import FaceObservationError, parse_face_observations
from face_sync import ExactPairMatcher, stamp_key
from manual_drive import ManualDriveController
from manual_drive import ManualDriveError
from manual_drive import yaw_from_quaternion
from mission_control import FindMissionController
from recognition_core import FaceEmbeddingRecognizer, TargetStore, assess_quality, classify_pose, cosine_similarity
from family_store import FamilyTargetStore
from range_capture import capture_pose, same_capture_pose
try:
    from speech_delivery import (
        AlsaSpeechPlayer,
        MacSpeechBackend,
        PreparedDeliveryStore,
        SpeechDeliveryError,
        delivery_gate,
        delivery_preflight,
        MAX_RECORDING_BYTES,
    )
except ImportError:  # pragma: no cover - package import on the development Mac
    from robot.jetson.perception.speech_delivery import (
        AlsaSpeechPlayer,
        MacSpeechBackend,
        PreparedDeliveryStore,
        SpeechDeliveryError,
        delivery_gate,
        delivery_preflight,
        MAX_RECORDING_BYTES,
    )


BEST_EFFORT = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST, reliability=QoSReliabilityPolicy.BEST_EFFORT)
REQUIRED = {"center": 3, "left": 2, "right": 2}
MINIMUM_TOTAL = 10
MAXIMUM_SAMPLES = 18
MAX_UPLOAD_BYTES = 10 * 1024 * 1024
MAX_VOICE_UPLOAD_BYTES = MAX_RECORDING_BYTES
PHOTO_DIRECTORY_PREFIX = "target_photos_"

def validate_mission_profile(request, target):
    # Older clients omit both fields. New clients bind a search to what they saw.
    if "profile_id" in request or "profile_revision" in request:
        if (not target or request.get("profile_id") != target.get("profile_id")
                or request.get("profile_revision") != target.get("revision")):
            raise ValueError("The selected profile changed. Select the person again before searching.")


PHONE_SETUP_PAGE = """<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Echora phone setup</title>
<style>body{font:16px/1.6 system-ui;background:#F2F5F2;color:#17231C;max-width:640px;margin:40px auto;padding:0 20px}a{color:#1E5B48}li{margin-bottom:12px}h1{line-height:1.2}</style>
<h1>Use your phone camera</h1><p>Complete this once on each phone, while connected to your home Wi-Fi.</p>
<ol><li><a href="/phone-ca.crt">Download the Echora local certificate</a>.</li>
<li><strong>iPhone / iPad:</strong> open Settings → General → VPN &amp; Device Management and install the downloaded certificate profile. Then open General → About → Certificate Trust Settings and enable full trust for that certificate.
<br><a href="https://support.apple.com/102390">Apple instructions</a></li>
<li><strong>Android:</strong> open Settings → Security &amp; privacy → More security settings → Encryption &amp; credentials → Install a certificate → CA certificate, then select the downloaded file. Names vary by phone.
<br><a href="https://support.google.com/pixelphone/answer/2844832">Android instructions</a></li></ol>
<p>Follow the step for your phone, then <a href="https://192.168.1.48/">open the secure Echora console</a>. Choose Settings → Profiles → Add a person → Phone camera. Allow camera access when asked.</p>
<p>Keep the Jetson on. The EV3 can stay off during enrollment. Original photos are not stored by Echora; aligned face crops are retained only when selected.</p>
<p><a href="/">Back to console</a></p></html>""".encode("utf-8")


PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#F2F5F2">
<title>Echora Control</title>
<style>
:root {
  color-scheme: light;
  --e-out: cubic-bezier(.16, 1, .3, 1);
  --e-spring: cubic-bezier(.34, 1.42, .64, 1);
  --t-press: 90ms; --t-release: 280ms; --t-state: 220ms; --t-open: 400ms;
  --sans: ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
  --mono: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;

  --void: #F2F5F2; --panel-1: #FFFFFF; --panel-2: #F8FAF8; --sunk: #F0F4F1; --well: #E7EEE9;
  --line: #DCE5DF; --line-2: #C2D0C7; --text: #17231C; --muted: #4F6055; --faint: #627269;
  --signal: #1E5B48; --sig-soft: rgba(30, 91, 72, .085); --sig-line: rgba(30, 91, 72, .38);
  --warn: #93651B; --warn-soft: rgba(147, 101, 27, .1);
  --alert: #B03A30; --alert-soft: rgba(176, 58, 48, .13);
  --alert-line: #A8352C; --alert-fill: linear-gradient(180deg, #BC4237, #A2322A); --alert-fg: #FFF6F4;
  --dgr-line: #DBB8B1; --dgr-fill: linear-gradient(180deg, #FDF2F0, #F8E6E2); --dgr-fg: #963028;
  --key-1: #FFFFFF; --key-2: #FBFCFB; --key-3: #EFF4F0; --key-line: #D3DED6; --key-line-hi: #AABBAF; --key-fg: #203128;
  --press-1: #E7EFE9; --press-2: #F1F5F2;
  --act-1: #236A52; --act-2: #195640; --act-fg: #F4F8F6; --act-label: rgba(244, 248, 246, .76);
  --bevel: rgba(255, 255, 255, .95); --seat: rgba(24, 61, 43, .09); --drop: rgba(24, 61, 43, .13);
  --glow: rgba(30, 91, 72, .2); --shade: rgba(24, 61, 43, .025);

  font-family: var(--sans);
  background: var(--void);
  color: var(--text);
}
* { box-sizing: border-box; }
html { min-height: 100%; background: var(--void); }
body { min-height: 100dvh; margin: 0; background: var(--void); }
button, input, select { font: inherit; color: inherit; }
button { cursor: pointer; -webkit-tap-highlight-color: transparent; }
button:disabled { cursor: not-allowed; }
button:focus-visible, input:focus-visible, select:focus-visible, summary:focus-visible, a:focus-visible {
  outline: 2px solid var(--signal); outline-offset: 3px; border-radius: 6px;
}
.mono { font-family: var(--mono); font-variant-numeric: tabular-nums; }
.k { font-family: var(--mono); font-size: 9px; font-weight: 500; letter-spacing: .18em; text-transform: uppercase; color: var(--faint); }
.sr-only { position: absolute; width: 1px; height: 1px; padding: 0; margin: -1px; overflow: hidden; clip: rect(0, 0, 0, 0); white-space: nowrap; border: 0; }
.skip-link { position: fixed; top: 8px; left: 8px; z-index: 100; transform: translateY(-160%); padding: 10px 14px;
  border-radius: 10px; background: var(--signal); color: var(--act-fg); font-weight: 600; text-decoration: none; }
.skip-link:focus { transform: none; }

/* ---------------- app bar ---------------- */
.bar { position: sticky; top: 0; z-index: 40; display: flex; align-items: center; justify-content: space-between; gap: 12px;
  min-height: 58px; padding: max(9px, env(safe-area-inset-top)) max(14px, env(safe-area-inset-right)) 9px max(14px, env(safe-area-inset-left));
  border-bottom: 1px solid var(--line); background: rgba(255, 255, 255, .94); backdrop-filter: blur(18px); -webkit-backdrop-filter: blur(18px); }
.brand { display: flex; align-items: center; gap: 10px; min-width: 0; }
.brand-mark { width: 30px; height: 30px; flex: 0 0 auto; display: grid; place-items: center; border-radius: 9px;
  border: 1px solid var(--line-2); background: linear-gradient(160deg, var(--key-1), var(--key-3)); color: var(--signal);
  box-shadow: inset 0 1px 0 var(--bevel); }
.brand-name { margin: 0; font-size: 14px; font-weight: 600; letter-spacing: -.01em; line-height: 1.1; }
.brand-subtitle { display: block; margin-top: 3px; font-family: var(--mono); font-size: 8.5px; font-weight: 500; letter-spacing: .2em; color: var(--faint); }
.top-actions { display: flex; align-items: center; gap: 10px; }
.service-badge { min-height: 32px; display: inline-flex; align-items: center; gap: 7px; padding: 0 11px; border: 1px solid var(--line-2);
  border-radius: 9px; background: var(--sunk); font-family: var(--mono); font-size: 9.5px; font-weight: 500; letter-spacing: .13em;
  color: var(--muted); transition: color var(--t-state) var(--e-out), border-color var(--t-state) var(--e-out); }
.service-badge[data-tone="ready"] { color: var(--text); }
.service-dot, .health-dot { width: 6px; height: 6px; flex: 0 0 auto; border-radius: 99px; background: var(--faint);
  transition: background var(--t-state) var(--e-out), box-shadow var(--t-state) var(--e-out); }
.service-badge[data-tone="ready"] .service-dot, .service-badge[data-tone="busy"] .service-dot { background: var(--signal); box-shadow: 0 0 0 3px var(--sig-soft); }
.service-badge[data-tone="busy"] .service-dot { animation: breathe 1.7s var(--e-out) infinite; }
.service-badge[data-tone="warning"] .service-dot { background: var(--warn); box-shadow: 0 0 0 3px var(--warn-soft); }
.service-badge[data-tone="offline"] .service-dot { background: var(--alert); box-shadow: 0 0 0 3px var(--alert-soft); }
.health-item[data-state="online"] .health-dot { background: var(--signal); box-shadow: 0 0 0 3px var(--sig-soft); }
.health-item[data-state="warning"] .health-dot { background: var(--warn); box-shadow: 0 0 0 3px var(--warn-soft); }
.health-item[data-state="offline"] .health-dot { background: var(--alert); box-shadow: 0 0 0 3px var(--alert-soft); }

.stop-all { min-height: 38px; display: inline-flex; align-items: center; justify-content: center; gap: 9px; padding: 0 14px;
  border: 1px solid var(--alert-line); border-radius: 10px; background: var(--alert-fill); color: var(--alert-fg);
  font-family: var(--mono); font-size: 9.5px; font-weight: 600; letter-spacing: .14em;
  box-shadow: inset 0 1px 0 rgba(255, 255, 255, .22), 0 8px 20px -14px var(--alert);
  transition: transform var(--t-release) var(--e-spring), box-shadow var(--t-release) var(--e-out); }
.stop-all:hover { transform: translateY(-1px); box-shadow: inset 0 1px 0 rgba(255, 255, 255, .22), 0 14px 28px -12px var(--alert); }
.stop-all:active { transition-duration: var(--t-press); transform: translateY(1px) scale(.99); box-shadow: inset 0 2px 6px rgba(0, 0, 0, .28); }
.stop-icon { width: 9px; height: 9px; border-radius: 2px; background: currentColor; }

/* ---------------- workspace ---------------- */
.work { display: flex; flex-direction: column; gap: 12px; padding: 12px max(12px, env(safe-area-inset-right)) max(28px, env(safe-area-inset-bottom)) max(12px, env(safe-area-inset-left)); }
.stage { display: flex; flex-direction: column; gap: 12px; min-width: 0; }
.rail > .mod { flex-shrink: 0; }

.live-panel { border: 1px solid var(--line); border-radius: 16px; overflow: hidden; background: var(--panel-2);
  box-shadow: inset 0 1px 0 var(--shade), 0 22px 46px -40px var(--drop); display: flex; flex-direction: column; }
.live-header { flex: 0 0 auto; display: flex; align-items: center; justify-content: space-between; gap: 12px;
  padding: 12px 15px; border-bottom: 1px solid var(--line); }
.live-title { display: flex; align-items: center; gap: 9px; font-family: var(--mono); font-size: 9.5px; font-weight: 500; letter-spacing: .17em; color: var(--muted); }
.live-light { width: 6px; height: 6px; border-radius: 99px; background: var(--faint); transition: background var(--t-state) var(--e-out), box-shadow var(--t-state) var(--e-out); }
.live-light[data-live="true"] { background: var(--signal); box-shadow: 0 0 0 3px var(--sig-soft); }
.live-state { font-family: var(--mono); font-size: 9.5px; letter-spacing: .1em; color: var(--faint); }
.camera-frame { position: relative; flex: 1; min-height: 190px; display: grid; place-items: center; overflow: hidden; background: #060708;
  background-image: linear-gradient(90deg, rgba(255, 255, 255, .028) 1px, transparent 1px), linear-gradient(rgba(255, 255, 255, .028) 1px, transparent 1px);
  background-size: 32px 32px; box-shadow: inset 0 2px 14px rgba(0, 0, 0, .55); }
.camera-frame::before { content: "Waiting for live camera"; position: absolute; font-family: var(--mono); font-size: 10px;
  letter-spacing: .16em; text-transform: uppercase; color: rgba(190, 190, 190, .42); }
.camera-frame img { position: relative; z-index: 1; display: block; width: 100%; height: auto; max-height: 64vh; object-fit: contain; }
.camera-frame.stream-error img { opacity: 0; }
.live-footer { flex: 0 0 auto; display: flex; justify-content: space-between; gap: 14px; padding: 11px 15px; border-top: 1px solid var(--line);
  font-family: var(--mono); font-size: 8.5px; letter-spacing: .1em; color: var(--faint); }

/* instrument strip: the readouts that must never be behind a disclosure */
.band { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); border: 1px solid var(--line); border-radius: 14px;
  overflow: hidden; background: var(--panel-1); box-shadow: inset 0 1px 0 var(--shade); }
.inst { padding: 10px 11px 12px; border-left: 1px solid var(--line); min-width: 0; }
.inst:first-child { border-left: 0; }
.inst-top { display: flex; align-items: baseline; justify-content: space-between; gap: 8px; }
.inst-top .k { min-width: 0; font-size: 7.5px; letter-spacing: .14em; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.inst-top .k + .k { flex: 0 1 auto; }
.inst-v { display: block; margin-top: 5px; font-family: var(--mono); font-size: 15px; font-weight: 500; letter-spacing: -.03em;
  font-variant-numeric: tabular-nums; color: var(--text); white-space: nowrap; overflow: hidden; text-overflow: ellipsis;
  transition: color var(--t-state) var(--e-out); }
.inst-v[data-idle="true"] { color: var(--faint); }
.inst-g { margin-top: 9px; }
.gauge { position: relative; height: 4px; border-radius: 99px; background: var(--line-2); }
.gauge::after { content: ""; position: absolute; left: 50%; top: -4px; width: 1px; height: 12px; background: var(--muted); opacity: .35; }
.gauge-marker { position: absolute; top: 50%; left: 50%; width: 11px; height: 11px; border-radius: 99px; border: 2.5px solid var(--panel-1);
  background: var(--signal); transform: translate(-50%, -50%); box-shadow: 0 0 0 1px var(--sig-line);
  transition: left 420ms var(--e-spring), opacity var(--t-state) var(--e-out); }
#headGauge::after { display: none; }
.meter { height: 4px; border-radius: 99px; background: var(--line-2); overflow: hidden; }
.meter i { display: block; width: 0; height: 100%; border-radius: inherit; background: var(--signal); transition: width 480ms var(--e-out); }

.system-note { display: flex; align-items: flex-start; gap: 10px; margin: 0; padding: 11px 14px; border: 1px solid var(--line);
  border-radius: 12px; background: var(--panel-1); color: var(--muted); font-size: 13px; line-height: 1.45;
  transition: border-color var(--t-state) var(--e-out), background var(--t-state) var(--e-out), color var(--t-state) var(--e-out); }
.system-note[data-tone="ready"] { border-color: var(--sig-line); background: var(--sig-soft); color: var(--text); }
.system-note[data-tone="warning"] { border-color: rgba(147, 101, 27, .35); background: var(--warn-soft); color: #4A3A18; }
.system-note[data-tone="offline"] { border-color: var(--dgr-line); background: var(--dgr-fill); color: var(--dgr-fg); }
.system-note::before { content: "i"; width: 18px; height: 18px; flex: 0 0 auto; display: grid; place-items: center;
  border: 1px solid currentColor; border-radius: 99px; font: 600 11px/1 var(--mono); opacity: .7; }

/* ---------------- module rail ---------------- */
.rail { display: flex; flex-direction: column; gap: 12px; min-width: 0; }
.mod { border: 1px solid var(--line); border-radius: 16px; background: var(--panel-2); overflow: hidden;
  box-shadow: 0 1px 0 var(--shade);
  transition: border-color var(--t-state) var(--e-out), background var(--t-state) var(--e-out), box-shadow var(--t-state) var(--e-out); }
.mod[open] { border-color: var(--line-2); background: linear-gradient(180deg, var(--panel-1), var(--panel-2));
  box-shadow: inset 0 1px 0 var(--shade), 0 22px 46px -38px var(--drop); }
.mod:not([open]):hover { box-shadow: 0 12px 26px -22px var(--drop); }
.mod > summary { list-style: none; display: grid; grid-template-columns: 40px minmax(0, 1fr) auto 34px; align-items: center; gap: 12px;
  padding: 15px 14px; cursor: pointer; transition: background var(--t-state) var(--e-out); }
.mod > summary::-webkit-details-marker { display: none; }
.mod > summary:hover { background: var(--shade); }
.mod-icon { width: 40px; height: 40px; display: grid; place-items: center; border-radius: 12px; border: 1px solid var(--line-2);
  background: var(--sunk); color: var(--muted);
  transition: color var(--t-state) var(--e-out), border-color var(--t-state) var(--e-out), background var(--t-state) var(--e-out); }
.mod[open] .mod-icon, .mod[data-live="true"] .mod-icon { color: var(--signal); border-color: var(--sig-line); background: var(--sig-soft); }
.mod-title { display: block; font-size: 15px; font-weight: 600; letter-spacing: -.018em; }
.mod-sub { display: block; margin-top: 3px; font-size: 12px; color: var(--faint); overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.mod-state { font-family: var(--mono); font-size: 9.5px; font-weight: 500; letter-spacing: .11em; text-transform: uppercase; color: var(--muted);
  padding: 6px 10px; border: 1px solid var(--line); border-radius: 99px; background: var(--sunk); white-space: nowrap; max-width: 40vw;
  overflow: hidden; text-overflow: ellipsis;
  transition: color var(--t-state) var(--e-out), border-color var(--t-state) var(--e-out), background var(--t-state) var(--e-out); }
.mod-state[data-tone="live"] { color: var(--act-fg); border-color: var(--act-2); background: linear-gradient(180deg, var(--act-1), var(--act-2)); }
.mod-state[data-tone="off"] { color: var(--alert); border-color: var(--dgr-line); background: var(--dgr-fill); }
.opener { width: 34px; height: 34px; display: grid; place-items: center; border-radius: 11px; border: 1px solid var(--key-line);
  background: linear-gradient(180deg, var(--key-1), var(--key-3)); color: var(--muted);
  box-shadow: inset 0 1px 0 var(--bevel), 0 4px 10px -6px var(--drop);
  transition: transform var(--t-release) var(--e-spring), box-shadow var(--t-release) var(--e-out),
              border-color var(--t-state) var(--e-out), color var(--t-state) var(--e-out), background var(--t-state) var(--e-out); }
.mod > summary:hover .opener { border-color: var(--key-line-hi); color: var(--text); transform: translateY(-1px); }
.mod > summary:active .opener { transition-duration: var(--t-press); transform: translateY(1px) scale(.97); box-shadow: inset 0 2px 6px var(--seat); }
.opener svg { transition: transform var(--t-open) var(--e-spring); }
.mod[open] .opener { background: linear-gradient(180deg, var(--act-1), var(--act-2)); border-color: var(--act-2); color: var(--act-fg); box-shadow: 0 6px 14px -8px var(--glow); }
.mod[open] .opener svg { transform: rotate(180deg); }
.mod-pad { padding: 16px 14px 18px; border-top: 1px solid var(--line); display: grid; gap: 13px; }

/* ---------------- keys ---------------- */
.key { position: relative; overflow: hidden; display: grid; place-items: center; align-content: center; gap: 7px; min-height: 48px;
  width: 100%; padding: 10px 14px; border: 1px solid var(--key-line); border-radius: 13px;
  background: linear-gradient(180deg, var(--key-1) 0%, var(--key-2) 56%, var(--key-3) 100%);
  color: var(--key-fg); font-size: 13.5px; font-weight: 500; text-align: center;
  box-shadow: 0 1px 2px var(--seat), 0 5px 12px -9px var(--drop);
  transition: transform var(--t-release) var(--e-spring), box-shadow var(--t-release) var(--e-out),
              border-color var(--t-state) var(--e-out), background var(--t-state) var(--e-out), color var(--t-state) var(--e-out);
  will-change: transform; touch-action: none; user-select: none; -webkit-user-select: none; }
.key::before { content: ""; position: absolute; inset: 0; pointer-events: none; border-radius: inherit;
  background: radial-gradient(130px circle at var(--mx, 50%) var(--my, 115%), var(--sheen, rgba(29, 92, 78, .13)), transparent 64%);
  opacity: 0; transition: opacity 260ms var(--e-out); }
.key:hover:not(:disabled)::before, .opener:hover::before, .mod > summary:hover .opener::before, .stop-all:hover::before { opacity: 1; }
.key:hover:not(:disabled) { border-color: var(--key-line-hi); transform: translateY(-1px);
  box-shadow: 0 8px 18px -10px var(--drop); }
.key:active:not(:disabled) { transition-duration: var(--t-press); transform: translateY(2px) scale(.985);
  background: linear-gradient(180deg, var(--press-1), var(--press-2)); box-shadow: inset 0 2px 7px var(--seat); }
.key:disabled { opacity: .42; transform: none; box-shadow: inset 0 1px 0 var(--shade); }
.key.is-active { border-color: var(--act-2); color: var(--act-fg); background: linear-gradient(180deg, var(--act-1), var(--act-2));
  box-shadow: 0 0 0 3px var(--sig-soft), 0 8px 18px -8px var(--glow); }
.key.is-active::after { content: ""; position: absolute; left: 15%; right: 15%; bottom: 8px; height: 2px; border-radius: 99px;
  background: linear-gradient(90deg, transparent, var(--act-fg), transparent); animation: train 1.05s var(--e-out) infinite; }
.key.action, .key.find { border-color: var(--act-2); color: var(--act-fg); background: linear-gradient(180deg, var(--act-1), var(--act-2)); }
.key.action:hover:not(:disabled), .key.find:hover:not(:disabled) { box-shadow: inset 0 1px 0 rgba(255, 255, 255, .18), 0 14px 28px -12px var(--glow); }
.key.danger { border-color: var(--dgr-line); color: var(--dgr-fg); background: var(--dgr-fill); }
.key.danger:hover:not(:disabled) { border-color: var(--alert); }
.key.action::before, .key.find::before, .key.is-active::before, .stop-all::before { --sheen: rgba(255, 255, 255, .3); }
.key.danger::before { --sheen: rgba(176, 58, 48, .15); }
.key.wake::after { content: ""; position: absolute; inset: 0; border-radius: inherit; pointer-events: none;
  background: linear-gradient(105deg, transparent 34%, rgba(255, 255, 255, .75) 50%, transparent 66%);
  animation: sweep 800ms var(--e-out) 1 both; }
.key-label { font-family: var(--mono); font-size: 9px; font-weight: 500; letter-spacing: .14em; text-transform: uppercase;
  color: var(--faint); transition: color var(--t-state) var(--e-out); }
.key.is-active .key-label, .key.action .key-label, .key.find .key-label { color: var(--act-label); }
.opener, .stop-all { position: relative; overflow: hidden; }
.opener::before, .stop-all::before { content: ""; position: absolute; inset: 0; pointer-events: none; border-radius: inherit;
  background: radial-gradient(90px circle at var(--mx, 50%) var(--my, 115%), var(--sheen, rgba(29, 92, 78, .16)), transparent 64%);
  opacity: 0; transition: opacity 260ms var(--e-out); }
.row { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
.delivery-editor { width: 100%; min-height: 92px; resize: vertical; padding: 12px 13px; border: 1px solid var(--key-line);
  border-radius: 10px; background: var(--panel-1); color: var(--text); font: 14px/1.5 var(--sans); }
.delivery-editor:focus { border-color: var(--sig-line); outline: 2px solid var(--sig-soft); }
.delivery-preview { padding: 12px; border: 1px solid var(--sig-line); border-radius: 10px; background: var(--sig-soft);
  white-space: pre-wrap; overflow-wrap: anywhere; font-size: 14px; line-height: 1.5; }
.delivery-actions { display: grid; grid-template-columns: 1fr 1fr; gap: 9px; }
.delivery-status { min-height: 20px; margin: 0; color: var(--muted); font-size: 12px; line-height: 1.5; }
.delivery-status[data-tone="warning"] { color: #6F4A0E; }
.delivery-status[data-tone="error"] { color: var(--alert); }

.recess { padding: 14px; border-radius: 20px; border: 1px solid var(--line); background: var(--well);
  box-shadow: inset 0 3px 10px var(--seat), inset 0 -1px 0 var(--bevel); }
.dpad { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); grid-template-rows: repeat(3, clamp(72px, 19vw, 88px)); gap: 9px; }
.dpad .key { min-height: 0; height: 100%; padding: 7px; border-radius: 15px; }
.dpad .drive-stop { border-color: var(--alert-line); background: var(--alert-fill); color: var(--alert-fg); }
.dpad .drive-stop .key-label { color: rgba(255, 246, 244, .74); }
.drive-help { margin: 0; font-size: 11.5px; line-height: 1.5; color: var(--faint); text-align: center; }

/* ---------------- module pieces ---------------- */
.lead { margin: 0; font-size: 15px; line-height: 1.5; letter-spacing: -.012em; color: var(--text); }
.readiness { margin: 0; font-size: 12px; line-height: 1.5; color: var(--faint); }
.supporting-copy { margin: 0; font-size: 12px; line-height: 1.55; color: var(--faint); }
.safety-check { display: flex; align-items: flex-start; gap: 11px; padding: 12px 13px; border: 1px solid var(--line); border-radius: 12px;
  background: var(--sunk); font-size: 12.5px; line-height: 1.45; color: var(--muted); cursor: pointer;
  transition: border-color var(--t-state) var(--e-out), background var(--t-state) var(--e-out), color var(--t-state) var(--e-out); }
.safety-check:hover { border-color: var(--line-2); }
.safety-check:has(input:checked) { border-color: var(--sig-line); background: var(--sig-soft); color: var(--text); }
.safety-check:has(input:disabled) { cursor: not-allowed; opacity: .5; }
.safety-check input { width: 20px; height: 20px; flex: 0 0 auto; margin: 0; accent-color: var(--signal); }

.health-strip { display: grid; grid-template-columns: 1fr 1fr; border: 1px solid var(--line); border-radius: 13px; overflow: hidden; background: var(--sunk); }
.health-item { display: flex; align-items: center; gap: 10px; min-width: 0; padding: 12px 13px; border-left: 1px solid var(--line); border-top: 1px solid var(--line); }
.health-item:nth-child(-n+2) { border-top: 0; }
.health-item:nth-child(odd) { border-left: 0; }
.health-label { min-width: 0; }
.health-label span { display: block; font-family: var(--mono); font-size: 8px; letter-spacing: .16em; text-transform: uppercase; color: var(--faint); }
.health-label strong { display: block; margin-top: 3px; font-size: 13.5px; font-weight: 500; letter-spacing: -.01em; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }

.progress-head { display: flex; align-items: flex-end; justify-content: space-between; gap: 12px; }
.guide-state { margin: 6px 0 0; font-size: 15px; font-weight: 600; letter-spacing: -.02em; }
.progress { height: 5px; border-radius: 99px; background: var(--line-2); overflow: hidden; }
.progress i { display: block; width: 0; height: 100%; border-radius: inherit; background: var(--signal); transition: width 480ms var(--e-out); }
.counts { display: grid; grid-template-columns: repeat(3, 1fr); gap: 9px; }
.count { padding: 11px 8px; border: 1px solid var(--line); border-radius: 11px; background: var(--sunk); text-align: center;
  font-family: var(--mono); font-size: 8.5px; letter-spacing: .1em; color: var(--faint); }
.count b { display: block; margin-bottom: 3px; font-size: 19px; font-weight: 500; color: var(--text); letter-spacing: -.02em; }
.enrollment-message { min-height: 20px; margin: 0; font-size: 13px; line-height: 1.45; color: var(--muted); }
.field label { display: block; margin-bottom: 7px; font-family: var(--mono); font-size: 9px; letter-spacing: .15em; text-transform: uppercase; color: var(--faint); }
.field input[type="text"], .field select, .field input[type="file"] { width: 100%; min-height: 46px; padding: 11px 13px;
  border: 1px solid var(--key-line); border-radius: 11px; background: var(--panel-1); color: var(--text); font-size: 13.5px;
  transition: border-color var(--t-state) var(--e-out); }
.field input[type="text"]:hover, .field select:hover { border-color: var(--key-line-hi); }
.field input[type="text"]:focus, .field select:focus { border-color: var(--sig-line); outline: none; }
.field input::file-selector-button { margin-right: 10px; padding: 8px 10px; border: 1px solid var(--key-line); border-radius: 8px;
  background: linear-gradient(180deg, var(--key-1), var(--key-3)); color: var(--key-fg); font: inherit; font-size: 12px; }
.danger-zone { padding-top: 13px; border-top: 1px solid var(--line); }
.privacy-note { display: flex; gap: 9px; margin: 0; font-size: 11.5px; line-height: 1.5; color: var(--faint); }
.privacy-note::before { content: "\2713"; color: var(--signal); font-weight: 700; }

[hidden] { display: none !important; }
.settings-button { min-height: 36px; padding: 8px 12px; font-size: 12px; }
#settingsDialog { width: min(620px, calc(100vw - 24px)); max-height: calc(100dvh - 32px); padding: 18px;
  border: 1px solid var(--line-2); border-radius: 18px; background: var(--void); color: var(--text); overflow-y: auto; }
#settingsDialog::backdrop { background: rgba(15, 23, 20, .56); backdrop-filter: blur(5px); }
.settings-heading { display: flex; align-items: center; justify-content: space-between; gap: 12px; margin-bottom: 14px; }
.settings-toolbar { position: sticky; top: 0; z-index: 5; background: var(--void); padding-bottom: 1px; }
.settings-heading h2 { margin: 3px 0 0; font-size: 24px; letter-spacing: -.03em; }
.settings-heading .k { margin: 0; }
.settings-heading .key { padding: 10px 16px; width: auto; flex: 0 0 auto; }
.settings-stop { width: 100%; margin-bottom: 14px; }
#settingsDialog .mod { margin-top: 12px; }
#settingsDialog .mod[open] .mod-pad > * { animation: none; }
.profile-grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 9px; margin-top: 10px; }
.profile-card { padding: 14px 10px; display: flex; flex-direction: column; align-items: flex-start; gap: 7px; text-align: left; }
.profile-card[aria-pressed="true"] { border-color: var(--signal); background: var(--sig-soft); box-shadow: inset 0 0 0 1px var(--signal); }
.profile-avatar { width: 32px; height: 32px; display: grid; place-items: center; border-radius: 50%; background: var(--well); color: var(--signal); font-size: 12px; font-weight: 600; }
.profile-name { font-weight: 600; overflow-wrap: anywhere; }
.profile-meta { font-size: 11px; color: var(--muted); }
.phone-capture { display: grid; gap: 10px; }
#phonePreview { width: 100%; max-height: 310px; aspect-ratio: 4/3; object-fit: contain; border-radius: 12px; background: var(--well); }
#phonePreview[data-facing="user"] { transform: scaleX(-1); }
#robotEnrollmentPreview { display: block; width: 100%; border-radius: 12px; }
@media (max-width: 430px) { .top-actions { gap: 6px; } .service-badge { padding: 0 7px; font-size: 8px; } #settingsDialog { padding: 12px; } }

/* module contents cascade in rather than snapping */
@keyframes unfold { from { opacity: 0; transform: translateY(-10px); } }
@keyframes sweep { from { transform: translateX(-118%); } to { transform: translateX(118%); } }
@keyframes train { 0% { transform: translateX(-52%) scaleX(.35); opacity: 0; } 45% { opacity: .9; } 100% { transform: translateX(52%) scaleX(.35); opacity: 0; } }
@keyframes breathe { 0%, 100% { opacity: 1; } 50% { opacity: .34; } }
.mod[open] .mod-pad > * { animation: unfold 500ms var(--e-out) backwards; }
.mod[open] .mod-pad > *:nth-child(1) { animation-delay: 60ms; }
.mod[open] .mod-pad > *:nth-child(2) { animation-delay: 120ms; }
.mod[open] .mod-pad > *:nth-child(3) { animation-delay: 175ms; }
.mod[open] .mod-pad > *:nth-child(4) { animation-delay: 225ms; }
.mod[open] .mod-pad > *:nth-child(5) { animation-delay: 270ms; }
.mod[open] .mod-pad > *:nth-child(6) { animation-delay: 310ms; }
.mod[open] .mod-pad > *:nth-child(7) { animation-delay: 345ms; }

/* ---------------- wide screens: camera left, rail right ---------------- */
@media (min-width: 1100px) {
  .bar { padding-left: 32px; padding-right: 32px; min-height: 66px; }
  .brand-mark { width: 32px; height: 32px; border-radius: 10px; }
  .brand-name { font-size: 14.5px; }
  .work { display: grid; grid-template-columns: minmax(0, 1fr) 470px; gap: 24px; align-items: start;
    padding: 26px 32px 40px; height: calc(100dvh - 66px); }
  .stage { gap: 18px; height: 100%; }
  .live-panel { flex: 1; min-height: 0; border-radius: 18px; }
  .camera-frame { min-height: 0; }
  .camera-frame img { max-height: 100%; height: 100%; }
  .band { flex: 0 0 auto; border-radius: 16px; }
  .inst { padding: 14px 17px 15px; }
  .inst-top .k { font-size: 8.5px; letter-spacing: .16em; }
  .inst-v { font-size: 20px; }
  .system-note { flex: 0 0 auto; }
  .rail { height: 100%; overflow-y: auto; overscroll-behavior: contain; padding-right: 2px; }
  .rail::-webkit-scrollbar { width: 6px; }
  .rail::-webkit-scrollbar-thumb { background: var(--line-2); border-radius: 99px; }
  .rail::-webkit-scrollbar-track { background: transparent; }
  .mod > summary { padding: 17px 16px; }
  .mod-pad { padding: 16px 16px 18px; gap: 12px; }
  .dpad { grid-template-rows: repeat(3, clamp(64px, 8.6vh, 84px)); }
  .recess { padding: 12px; }
  .mod-state { max-width: 160px; }
}
@media (max-width: 430px) {
  .inst-top .k + .k { display: none; }
  .inst { padding: 10px 9px 11px; }
}
@media (max-width: 420px) {
  .brand-subtitle { display: none; }
  .stop-all .stop-label { font-size: 0; }
  .stop-all .stop-label::after { content: "STOP"; font-size: 9.5px; letter-spacing: .14em; }
  .inst { padding: 9px 8px 11px; }
  .inst-v { font-size: 13.5px; }
  .mod-state { max-width: 136px; }
}
@media (prefers-reduced-motion: reduce) {
  *, *::before, *::after { animation-duration: .01ms !important; animation-iteration-count: 1 !important; transition-duration: .01ms !important; }
}
</style>
</head>
<body>
<a class="skip-link" href="#controls">Skip to controls</a>
<header class="bar">
  <div class="brand" aria-label="Echora Control">
    <span class="brand-mark" aria-hidden="true">
      <svg width="16" height="16" viewBox="0 0 20 20" fill="none">
        <circle cx="10" cy="10" r="3" stroke="currentColor" stroke-width="1.5"></circle>
        <path d="M10 1.6a8.4 8.4 0 0 1 8.4 8.4" stroke="currentColor" stroke-width="1.5" stroke-linecap="round"></path>
        <path d="M10 18.4A8.4 8.4 0 0 1 1.6 10" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" opacity="0.4"></path>
      </svg>
    </span>
    <div>
      <h1 class="brand-name">Echora Control</h1>
      <span class="brand-subtitle">SINGLE-ROOM OPERATOR</span>
    </div>
  </div>
  <div class="top-actions">
    <button class="key settings-button" id="openSettings" type="button" onclick="openSettings()" aria-haspopup="dialog">Settings</button>
    <div class="service-badge" id="serviceBadge" data-tone="connecting" role="status" aria-live="polite">
      <span class="service-dot" aria-hidden="true"></span>
      <span id="service">Connecting…</span>
    </div>
    <button class="stop-all" id="stopMission" type="button" onclick="stopMission()" aria-label="Stop all robot motion immediately">
      <span class="stop-icon" aria-hidden="true"></span>
      <span class="stop-label">STOP ALL MOTION</span>
    </button>
  </div>
</header>

<main class="work" id="controls">
  <div class="stage">
    <section class="live-panel" aria-labelledby="liveTitle">
      <div class="live-header">
        <span class="live-title" id="liveTitle"><span class="live-light" id="liveLight" aria-hidden="true"></span>LIVE VIEW</span>
        <span class="live-state" id="liveFeedState">Connecting to camera…</span>
      </div>
      <div class="camera-frame" id="cameraFrame">
        <img id="cameraStream" src="/stream" alt="Live view from Echora's front camera">
      </div>
      <div class="live-footer">
        <span>USE THIS VIEW WHENEVER THE ROBOT IS MOVING</span>
        <span>NO FULL CAMERA FRAMES ARE STORED</span>
      </div>
    </section>

    <section class="band" aria-label="Live readouts">
      <div class="inst">
        <div class="inst-top"><span class="k">Tether</span><span class="k" id="headingLimit">±—</span></div>
        <span class="inst-v" id="driveHeading" data-idle="true">—</span>
        <div class="inst-g">
          <div class="gauge" id="headingMeter" role="meter" aria-label="Tether heading" aria-valuemin="-80" aria-valuemax="80">
            <span class="gauge-marker" id="headingMarker"></span>
          </div>
        </div>
      </div>
      <div class="inst">
        <div class="inst-top"><span class="k">Head</span><span class="k" id="headRange">TILT</span></div>
        <span class="inst-v" id="headAngle" data-idle="true">—</span>
        <div class="inst-g">
          <div class="gauge" id="headGauge" role="meter" aria-label="Camera head position" aria-valuemin="-90" aria-valuemax="90">
            <span class="gauge-marker" id="headMarker"></span>
          </div>
        </div>
      </div>
      <div class="inst">
        <div class="inst-top"><span class="k">Views</span><span class="k">ENROLL</span></div>
        <span class="inst-v" id="enrollValue" data-idle="true">—</span>
        <div class="inst-g"><div class="meter"><i id="enrollBar"></i></div></div>
      </div>
      <div class="inst">
        <div class="inst-top"><span class="k">Link</span><span class="k">STATUS</span></div>
        <span class="inst-v" id="linkValue" data-idle="true">—</span>
        <div class="inst-g"><div class="meter"><i id="linkBar"></i></div></div>
      </div>
    </section>

    <p class="system-note" id="systemSummary" role="status" aria-live="polite">Checking the control systems…</p>
  </div>

  <div class="rail">
    <details class="mod" id="systemsPanel">
      <summary>
        <span class="mod-icon" aria-hidden="true">
          <svg width="19" height="19" viewBox="0 0 22 22" fill="none"><path d="M2.5 11h4l2-5 3 10 2-5h6" stroke="currentColor" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round"></path></svg>
        </span>
        <span><span class="mod-title">Systems</span><span class="mod-sub" id="systemsSub">Checking camera, EV3, odometry and head…</span></span>
        <span class="mod-state" id="systemsChip">CHECKING</span>
        <span class="opener" aria-hidden="true"><svg width="14" height="14" viewBox="0 0 20 20" fill="none"><path d="M5 8l5 5 5-5" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"></path></svg></span>
      </summary>
      <div class="mod-pad">
        <div class="health-strip">
          <div class="health-item" data-state="checking">
            <span class="health-dot" aria-hidden="true"></span>
            <div class="health-label"><span>Camera</span><strong id="cameraHealth">Checking…</strong></div>
          </div>
          <div class="health-item" data-state="checking">
            <span class="health-dot" aria-hidden="true"></span>
            <div class="health-label"><span>EV3 robot</span><strong id="robotHealth">Checking…</strong></div>
          </div>
          <div class="health-item" data-state="checking">
            <span class="health-dot" aria-hidden="true"></span>
            <div class="health-label"><span>Odometry</span><strong id="odomHealth">Checking…</strong></div>
          </div>
          <div class="health-item" data-state="checking">
            <span class="health-dot" aria-hidden="true"></span>
            <div class="health-label"><span>Camera head</span><strong id="headHealth">Checking…</strong></div>
          </div>
        </div>
        <p class="supporting-copy">Movement controls unlock only while the camera, the EV3 and fresh odometry are all reporting.</p>
      </div>
    </details>

    <details class="mod" id="drivePanel">
      <summary>
        <span class="mod-icon" aria-hidden="true">
          <svg width="19" height="19" viewBox="0 0 22 22" fill="none"><rect x="3" y="6.5" width="16" height="9" rx="3.5" stroke="currentColor" stroke-width="1.6"></rect><path d="M7.5 11h1.6M11 9.4v3.2M17 11h-1.6" stroke="currentColor" stroke-width="1.6" stroke-linecap="round"></path></svg>
        </span>
        <span><span class="mod-title">Supervised driving</span><span class="mod-sub" id="driveSub">Hold-to-move, gated on a cable check</span></span>
        <span class="mod-state" id="driveState" role="status" aria-live="polite">Driving locked</span>
        <span class="opener" aria-hidden="true"><svg width="14" height="14" viewBox="0 0 20 20" fill="none"><path d="M5 8l5 5 5-5" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"></path></svg></span>
      </summary>
      <div class="mod-pad">
        <p class="lead" id="driveMessage" role="status" aria-live="polite">Waiting for robot and odometry.</p>
        <p class="readiness" id="driveReadiness">Checking what is needed to enable manual control…</p>
        <label class="safety-check" for="driveCableClear">
          <input type="checkbox" id="driveCableClear" autocomplete="off" onchange="updateMissionControls()">
          <span>The robot is at the marked cable-neutral heading. I am supervising, and the cable and rear path are clear.</span>
        </label>
        <div class="row">
          <button class="key action" id="enableDrive" type="button" onclick="enableDrive()" aria-describedby="driveReadiness" disabled>Enable driving</button>
          <button class="key" id="disableDrive" type="button" onclick="disableDrive()" disabled>Lock controls</button>
        </div>
        <div class="recess">
          <div class="dpad" aria-label="Press and hold manual driving controls">
            <span aria-hidden="true"></span>
            <button class="key" id="driveForward" type="button" onpointerdown="beginDrive('forward',event)" onlostpointercapture="releaseDrive(event)" onkeydown="beginDrive('forward',event)" onkeyup="releaseDrive(event)" oncontextmenu="return false" aria-label="Hold to drive forward" disabled><svg width="22" height="22" viewBox="0 0 24 24" fill="none" aria-hidden="true"><path d="M12 19V6M6 12l6-6 6 6" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round"></path></svg><span class="key-label">Forward</span></button>
            <span aria-hidden="true"></span>
            <button class="key" id="driveLeft" type="button" onpointerdown="beginDrive('left',event)" onlostpointercapture="releaseDrive(event)" onkeydown="beginDrive('left',event)" onkeyup="releaseDrive(event)" oncontextmenu="return false" aria-label="Hold to turn left" disabled><svg width="22" height="22" viewBox="0 0 24 24" fill="none" aria-hidden="true"><path d="M5 12h13M12 6l-6 6 6 6" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round"></path></svg><span class="key-label">Left</span></button>
            <button class="key drive-stop" id="driveStop" type="button" onpointerdown="releaseDrive(event,true)" onkeydown="releaseDrive(event,true)" oncontextmenu="return false" aria-label="Stop manual driving" disabled><span class="stop-icon" aria-hidden="true"></span><span class="key-label">Stop</span></button>
            <button class="key" id="driveRight" type="button" onpointerdown="beginDrive('right',event)" onlostpointercapture="releaseDrive(event)" onkeydown="beginDrive('right',event)" onkeyup="releaseDrive(event)" oncontextmenu="return false" aria-label="Hold to turn right" disabled><svg width="22" height="22" viewBox="0 0 24 24" fill="none" aria-hidden="true"><path d="M19 12H6M12 6l6 6-6 6" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round"></path></svg><span class="key-label">Right</span></button>
            <span aria-hidden="true"></span>
            <button class="key" id="driveBack" type="button" onpointerdown="beginDrive('back',event)" onlostpointercapture="releaseDrive(event)" onkeydown="beginDrive('back',event)" onkeyup="releaseDrive(event)" oncontextmenu="return false" aria-label="Hold to reverse" disabled><svg width="22" height="22" viewBox="0 0 24 24" fill="none" aria-hidden="true"><path d="M12 5v13M6 12l6 6 6-6" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round"></path></svg><span class="key-label">Reverse</span></button>
            <span aria-hidden="true"></span>
          </div>
        </div>
        <p class="drive-help">Press and hold a direction. Releasing anywhere stops the tracks. The 0.25-second watchdog also stops motion if this page disconnects.</p>
      </div>
    </details>

    <details class="mod" id="missionPanel">
      <summary>
        <span class="mod-icon" aria-hidden="true">
          <svg width="19" height="19" viewBox="0 0 22 22" fill="none"><circle cx="9.8" cy="9.8" r="6.2" stroke="currentColor" stroke-width="1.6"></circle><path d="M14.4 14.4 19 19" stroke="currentColor" stroke-width="1.6" stroke-linecap="round"></path></svg>
        </span>
        <span><span class="mod-title">Find Me</span><span class="mod-sub" id="missionTarget">No target</span></span>
        <span class="mod-state" id="missionState" role="status" aria-live="polite">Checking…</span>
        <span class="opener" aria-hidden="true"><svg width="14" height="14" viewBox="0 0 20 20" fill="none"><path d="M5 8l5 5 5-5" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"></path></svg></span>
      </summary>
      <div class="mod-pad">
        <div><p class="k">Who should Echora find?</p><div id="findProfiles" class="profile-grid" role="group" aria-label="Choose a person to find"></div></div>
        <button class="key" id="addFirstProfile" type="button" onclick="openSettings();familyEnroll(false)" hidden>Add a person</button>
        <div class="field" style="margin-top:16px">
          <label for="deliveryText">Message to say</label>
          <textarea id="deliveryText" class="delivery-editor" maxlength="2000" placeholder="Type the exact message the robot should say."></textarea>
          <p class="supporting-copy">You can type a message or record it for transcription. Review and approve the exact text before speech is prepared. A recording is sent to the Mac's configured transcription provider; Echora does not retain it.</p>
        </div>
        <div class="delivery-actions">
          <button class="key" id="recordMessage" type="button" onclick="startMessageRecording()">Record message</button>
          <button class="key" id="stopMessageRecording" type="button" onclick="stopMessageRecording()" disabled>Stop recording</button>
        </div>
        <div class="delivery-actions">
          <button class="key" id="transcribeMessage" type="button" onclick="transcribeMessage()" disabled>Transcribe recording</button>
          <button class="key action" id="approveMessage" type="button" onclick="approveMessage()" disabled>Approve &amp; prepare speech</button>
        </div>
        <p class="delivery-status" id="voiceStatus" role="status" aria-live="polite">Speech uses the connected Mac service and Fish Audio.</p>
        <div class="delivery-actions">
          <button class="key" id="previewMessage" type="button" onclick="previewMessage()" disabled>Preview on robot</button>
          <button class="key find" id="findDeliver" type="button" onclick="startDeliveryMission()" disabled>Find &amp; deliver</button>
        </div>
        <p class="delivery-status" id="deliveryStatus" role="status" aria-live="polite">Test the approved message through the robot speaker. Delivery also requires a measured, validated stopping distance.</p>
        <p class="lead" id="missionMessage" role="status" aria-live="polite">Waiting for mission status.</p>
        <p class="readiness" id="missionReadiness">Checking mission requirements…</p>
        <label class="safety-check" for="cableClear">
          <input type="checkbox" id="cableClear" autocomplete="off" onchange="updateMissionControls()">
          <span>The robot is at cable-neutral and the supported cable is loose and clear of both tracks.</span>
        </label>
        <button class="key find" id="findPerson" type="button" onclick="startMission()" aria-describedby="missionReadiness" style="min-height:60px;font-size:15px" disabled>Find person</button>
        <p class="supporting-copy">Echora scans the room, verifies the floor before each short movement, and stops whenever the route or identity is uncertain.</p>
      </div>
    </details>

    <details class="mod" id="cameraPanel">
      <summary>
        <span class="mod-icon" aria-hidden="true">
          <svg width="19" height="19" viewBox="0 0 22 22" fill="none"><rect x="3" y="6" width="16" height="11" rx="3" stroke="currentColor" stroke-width="1.6"></rect><circle cx="11" cy="11.5" r="3" stroke="currentColor" stroke-width="1.6"></circle><path d="M8 6V4.6h6V6" stroke="currentColor" stroke-width="1.6" stroke-linecap="round"></path></svg>
        </span>
        <span><span class="mod-title">Camera head</span><span class="mod-sub" id="headState" role="status" aria-live="polite">Checking motor…</span></span>
        <span class="mod-state" id="headChip">TILT</span>
        <span class="opener" aria-hidden="true"><svg width="14" height="14" viewBox="0 0 20 20" fill="none"><path d="M5 8l5 5 5-5" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"></path></svg></span>
      </summary>
      <div class="mod-pad">
        <div class="row">
          <button class="key" id="tiltUp" type="button" onclick="jogCamera('up',5)" aria-describedby="headState cameraHelp" disabled><svg width="17" height="17" viewBox="0 0 24 24" fill="none" aria-hidden="true"><path d="M12 19V6M6 12l6-6 6 6" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round"></path></svg><span class="key-label">Up 5°</span></button>
          <button class="key" id="tiltDown" type="button" onclick="jogCamera('down',5)" aria-describedby="headState cameraHelp" disabled><svg width="17" height="17" viewBox="0 0 24 24" fill="none" aria-hidden="true"><path d="M12 5v13M6 12l6 6 6-6" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round"></path></svg><span class="key-label">Down 5°</span></button>
        </div>
        <div class="row">
          <button class="key" id="tiltUpCoarse" type="button" onclick="jogCamera('up',15)" aria-describedby="headState cameraHelp" disabled><span class="key-label" style="letter-spacing:.1em">Up 15°</span></button>
          <button class="key" id="tiltDownCoarse" type="button" onclick="jogCamera('down',15)" aria-describedby="headState cameraHelp" disabled><span class="key-label" style="letter-spacing:.1em">Down 15°</span></button>
        </div>
        <p class="supporting-copy" id="cameraHelp">Manual buttons work without calibration or saved angle limits. Watch the camera and stop at the view you want. Move to your lowest comfortable view and save it, then your highest. Saving does not move the camera.</p>
        <div class="row">
          <button class="key" id="saveLowerLimit" type="button" onclick="saveCameraLimit('lower')" disabled><span class="key-label" style="letter-spacing:.1em">Save lower limit</span></button>
          <button class="key" id="saveUpperLimit" type="button" onclick="saveCameraLimit('upper')" disabled><span class="key-label" style="letter-spacing:.1em">Save upper limit</span></button>
        </div>
        <div id="cameraRangeRecovery" hidden>
          <p class="supporting-copy">After a restart, use the manual buttons to return to the same lowest view you saved. Press below to confirm this lower view and reuse the previous range. Saving does not move the camera.</p>
          <button class="key" id="restoreCameraRange" type="button" onclick="restoreCameraRange()" style="margin-top:11px" disabled><span class="key-label" style="letter-spacing:.1em">Use saved range from this lower view</span></button>
        </div>
        <p class="supporting-copy" id="cameraLimitStatus" role="status" aria-live="polite">Set the lower limit first.</p>
      </div>
    </details>

  </div>
</main>
<dialog id="settingsDialog" aria-labelledby="settingsTitle">
  <div class="settings-toolbar">
  <div class="settings-heading"><div><p class="k">Settings</p><h2 id="settingsTitle">Profiles</h2></div><button class="key" type="button" onclick="closeSettings()" aria-label="Close settings">Close</button></div>
  <button class="stop-all settings-stop" type="button" onclick="stopMission()">STOP ALL MOTION</button>
  </div>
<details id="familyPanel" class="mod" open><summary style="grid-template-columns:1fr auto"><span class="mod-title">Profiles</span></summary>
<div class="mod-pad"><div class="field"><label for="familySelect">Saved profile</label><select id="familySelect" onchange="familySelect()"></select></div>
<div class="row"><button class="key" type="button" id="addProfile" onclick="familyEnroll(false)">Add a person</button></div>
<div class="row"><button class="key" type="button" data-profile-action onclick="familyRename()">Rename</button><button class="key" type="button" data-profile-action onclick="familyEnroll(true)">Update enrollment</button></div>
<div class="row"><button class="key" type="button" onclick="familyGallery()">Remembered clothes</button><button class="key danger" type="button" data-profile-action onclick="familyDelete()">Delete person</button></div>
<p id="profilesSummary" class="supporting-copy"></p><p class="supporting-copy">Clothes are remembered across restarts after a face match. Similar outfits may need another view.</p>
<p id="familyTracking" role="status" aria-live="polite"></p><div id="wardrobeGallery" style="display:flex;flex-wrap:wrap;gap:16px"></div></div></details>
    <details class="mod" id="targetPanel">
      <summary>
        <span class="mod-icon" aria-hidden="true">
          <svg width="19" height="19" viewBox="0 0 22 22" fill="none"><circle cx="11" cy="7.6" r="3.4" stroke="currentColor" stroke-width="1.6"></circle><path d="M4.2 18.4c.7-3.5 3.4-5.4 6.8-5.4s6.1 1.9 6.8 5.4" stroke="currentColor" stroke-width="1.6" stroke-linecap="round"></path></svg>
        </span>
        <span><span class="mod-title">Face enrollment</span><span class="mod-sub" id="targetSummary">No one enrolled</span></span>
        <span class="mod-state" id="targetChip">ENROLL</span>
        <span class="opener" aria-hidden="true"><svg width="14" height="14" viewBox="0 0 20 20" fill="none"><path d="M5 8l5 5 5-5" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"></path></svg></span>
      </summary>
      <div class="mod-pad">
        <div class="field">
          <label for="enrollmentSource">Camera for enrollment</label>
          <select id="enrollmentSource" onchange="updateEnrollmentControls()">
            <option value="phone">Phone camera</option><option value="robot">Robot camera</option>
          </select>
        </div>
        <section id="phoneCapture" class="phone-capture" aria-label="Phone camera enrollment">
          <video id="phonePreview" autoplay muted playsinline aria-label="Your phone camera preview" hidden></video>
          <p id="phoneMessage" class="supporting-copy" role="status" aria-live="polite">Start enrollment to open your phone camera. Hold it at eye level.</p>
          <p id="secureCameraHelp" class="supporting-copy" hidden>Live capture needs the secure console. <a href="https://192.168.1.48/">Open secure console</a> · <a href="/phone-setup">Phone setup</a>. You can also upload photos below.</p>
          <div class="row"><button class="key" id="resumePhone" type="button" onclick="resumePhoneCapture()" disabled>Open phone camera</button><button class="key" id="pausePhone" type="button" onclick="pausePhoneCapture()" disabled>Pause camera</button></div>
          <button class="key" id="switchPhone" type="button" onclick="switchPhoneCamera()" disabled>Use rear camera</button>
        </section>
        <section id="robotEnrollmentHelp" hidden><img id="robotEnrollmentPreview" alt="Robot camera for face enrollment" hidden><p class="supporting-copy">Use the live robot view. Stand 1–2 metres away; only the person being enrolled should be visible.</p></section>
        <div class="progress-head">
          <div><p class="k">Enrollment guidance</p><p class="guide-state" id="guide">Wait for camera</p></div>
          <span class="mono" id="progressLabel" style="font-size:11px;color:var(--signal)">0 / 10 views</span>
        </div>
        <div class="progress" id="enrollmentProgress" role="progressbar" aria-label="Enrollment views collected" aria-valuemin="0" aria-valuemax="10" aria-valuenow="0"><i id="bar"></i></div>
        <div class="counts" aria-label="Accepted enrollment angles">
          <div class="count"><b id="center">0</b>FRONT · NEED 3</div>
          <div class="count"><b id="left">0</b>LEFT · NEED 2</div>
          <div class="count"><b id="right">0</b>RIGHT · NEED 2</div>
        </div>
        <p class="enrollment-message" id="message" role="status" aria-live="polite"></p>
        <div class="field">
          <label for="label">Person's name</label>
          <input id="label" type="text" maxlength="40" placeholder="Example: Mom" autocomplete="off">
        </div>
        <div class="field">
          <label for="retention">Photo storage</label>
          <select id="retention">
            <option value="embeddings">Embeddings only — delete photos</option>
            <option value="face_crops">Keep aligned face crops</option>
          </select>
        </div>
        <label class="safety-check" for="consent">
          <input type="checkbox" id="consent">
          <span>I have this person's permission to create a biometric face template.</span>
        </label>
        <button class="key action" id="start" type="button" onclick="startEnrollment()" aria-describedby="message" disabled>Start enrollment</button>
        <div class="field">
          <label for="files">Add clear photographs</label>
          <input id="files" type="file" accept="image/*" multiple>
        </div>
        <div class="row">
          <button class="key" id="upload" type="button" onclick="uploadPhotos()" disabled><span class="key-label" style="letter-spacing:.1em">Add selected photos</span></button>
          <button class="key" id="cancel" type="button" onclick="enrollmentAction('/api/enrollment/cancel')" disabled><span class="key-label" style="letter-spacing:.1em">Cancel</span></button>
        </div>
        <button class="key find" id="finish" type="button" onclick="enrollmentAction('/api/enrollment/finish')" disabled>Finish enrollment</button>
        <div class="danger-zone">
          <button class="key danger" id="deleteTarget" type="button" onclick="deleteTarget()" aria-describedby="targetSummary" disabled><span class="key-label" style="letter-spacing:.1em;color:currentColor">Delete enrolled target</span></button>
        </div>
        <p class="privacy-note">No original uploads or full camera frames are stored. Aligned face crops are kept only when selected above.</p>
      </div>
    </details>
</dialog>
<noscript><p class="system-note">JavaScript is required to show status and operate Echora safely.</p></noscript>
<script>
const H={'X-Echora-Action':'1'};
async function post(url,body,type='application/json',extra={}){let h={...H,...extra.headers};if(type)h['Content-Type']=type;let r=await fetch(url,{method:'POST',headers:h,body,signal:extra.signal});let j=await r.json();if(!r.ok){let e=Error(j.error||'Request failed');e.status=r.status;throw e}return j}
async function action(url){try{await post(url,'{}');await refresh()}catch(e){alert(e.message)}}
let latestStatus=null,lastOk=0;
let deliveryDraftId=(crypto.randomUUID?crypto.randomUUID():String(Date.now())+Math.random()),deliveryRevision=0,preparedDeliveryId=null,deliveryRecording=null,deliveryRecorder=null,deliveryStream=null,deliveryRecordTimer=null,deliveryBusy=false,deliverySyncReady=false;
function deliveryStatus(text,tone=''){setText('deliveryStatus',text);let el=document.getElementById('deliveryStatus');if(el){if(tone)el.dataset.tone=tone;else el.removeAttribute('data-tone')}}
function voiceStatus(text,tone=''){setText('voiceStatus',text);let el=document.getElementById('voiceStatus');if(el){if(tone)el.dataset.tone=tone;else el.removeAttribute('data-tone')}}
function updateDeliveryControls(s=latestStatus){
  let text=document.getElementById('deliveryText'),recording=!!deliveryRecorder,busy=deliveryBusy||recording;
  let m=(s&&s.mission)||{},drive=(s&&s.manual_drive)||{},d=(s&&s.delivery)||{},audio=(s&&s.speech_output)||{};
  let active=['preparing','previewing','searching','approaching','speaking'].includes(d.state);
  let matching=!!(preparedDeliveryId&&d.prepared&&['message_ready','delivery_failed','stopped'].includes(d.state)&&d.delivery_id===preparedDeliveryId&&s&&d.profile_id===s.profile_id&&d.profile_revision===s.profile_revision&&d.message_id===deliveryDraftId&&d.message_revision===deliveryRevision);
  let controlsLocked=busy||active||!!m.running||!!drive.enabled||!!s?.enrolling;
  text.disabled=controlsLocked;
  document.getElementById('recordMessage').disabled=controlsLocked||!s||!s.profile_id;
  document.getElementById('stopMessageRecording').disabled=!recording;
  document.getElementById('transcribeMessage').disabled=controlsLocked||!deliveryRecording;
  document.getElementById('approveMessage').disabled=controlsLocked||!s||!s.profile_id||!text.value.trim();
  document.getElementById('previewMessage').disabled=controlsLocked||!matching||!audio.available;
  let cable=document.getElementById('cableClear');
  document.getElementById('findDeliver').disabled=controlsLocked||!matching||d.state!=='message_ready'||!d.previewed||!audio.available||!cable.checked||!s||!s.camera_ready||!s.robot_ready||!s.camera_head||!s.camera_head.available||s.camera_head.moving||s.camera_head.homing;
  if(d.message)setText('deliveryStatus',d.message);
  else if(d.state==='idle')setText('deliveryStatus','Test the approved message through the robot speaker before delivery.');
  if(!audio.available&&audio.message)setText('voiceStatus',audio.message);
}
function invalidateDeliveryDraft(){
  deliveryRevision++;preparedDeliveryId=null;
  let text=document.getElementById('deliveryText');if(text)text.dataset.approved='false';
  if(deliverySyncReady){post('/api/delivery/clear',JSON.stringify({message_id:deliveryDraftId,message_revision:deliveryRevision})).then(()=>refresh()).catch(()=>{})}
  updateDeliveryControls(latestStatus);
}
async function startMessageRecording(){
  if(!navigator.mediaDevices||!navigator.mediaDevices.getUserMedia||!window.MediaRecorder){voiceStatus('Recording is unavailable here. Open the secure robot app or type the message.','warning');return}
  invalidateDeliveryDraft();
  try{
    deliveryStream=await navigator.mediaDevices.getUserMedia({audio:true});let options={};
    for(let type of ['audio/webm;codecs=opus','audio/webm','audio/mp4'])if(MediaRecorder.isTypeSupported(type)){options.mimeType=type;break}
    deliveryRecorder=new MediaRecorder(deliveryStream,options);let chunks=[];
    deliveryRecorder.ondataavailable=event=>{if(event.data&&event.data.size)chunks.push(event.data)};
    deliveryRecorder.onstop=()=>{deliveryRecording=new Blob(chunks,{type:deliveryRecorder.mimeType||'audio/webm'});if(deliveryStream)deliveryStream.getTracks().forEach(track=>track.stop());deliveryStream=null;deliveryRecorder=null;clearTimeout(deliveryRecordTimer);deliveryRecordTimer=null;voiceStatus('Recording ready. Transcribe it, review the text, then approve it.');updateDeliveryControls(latestStatus)};
    deliveryRecorder.start();document.getElementById('recordMessage').disabled=true;document.getElementById('stopMessageRecording').disabled=false;voiceStatus('Recording the operator message.');
    deliveryRecordTimer=setTimeout(()=>stopMessageRecording(),60000);
  }catch(error){if(deliveryStream)deliveryStream.getTracks().forEach(track=>track.stop());deliveryStream=null;voiceStatus('Microphone unavailable: '+error.message,'error')}
}
function stopMessageRecording(){if(deliveryRecorder&&deliveryRecorder.state!=='inactive')deliveryRecorder.stop();else if(deliveryStream){deliveryStream.getTracks().forEach(track=>track.stop());deliveryStream=null}document.getElementById('stopMessageRecording').disabled=true;document.getElementById('recordMessage').disabled=false}
async function transcribeMessage(){
  if(!deliveryRecording||deliveryBusy)return;deliveryBusy=true;updateDeliveryControls(latestStatus);voiceStatus('Transcribing on the Mac speech service…');
  try{let result=await post('/api/voice/transcribe',deliveryRecording,deliveryRecording.type||'application/octet-stream');let field=document.getElementById('deliveryText');field.value=result.text;field.dispatchEvent(new Event('input',{bubbles:true}));voiceStatus(result.text?'Transcript ready. Review and edit it before approval.':'No words were transcribed. Type the message or record again.',result.text?'':'warning')}
  catch(error){voiceStatus(error.message,'error')}finally{deliveryBusy=false;updateDeliveryControls(latestStatus)}
}
async function approveMessage(){
  let text=document.getElementById('deliveryText').value,profile=latestStatus;
  if(!text.trim()||!profile||!profile.profile_id||!profile.profile_revision){voiceStatus('Select an enrolled person and enter a message first.','warning');return}
  deliveryBusy=true;preparedDeliveryId=null;let revision=deliveryRevision;updateDeliveryControls(latestStatus);voiceStatus('Preparing approved speech through the Mac and Fish Audio…');
  try{
    let result=await post('/api/delivery/prepare',JSON.stringify({profile_id:profile.profile_id,profile_revision:profile.profile_revision,message_id:deliveryDraftId,message_revision:revision,text}));
    if(revision!==deliveryRevision||profile.profile_id!==latestStatus.profile_id||profile.profile_revision!==latestStatus.profile_revision){await post('/api/delivery/clear',JSON.stringify({message_id:deliveryDraftId,message_revision:deliveryRevision}));voiceStatus('The message or recipient changed. Approve the current version again.','warning');return}
    preparedDeliveryId=result.delivery_id;document.getElementById('deliveryText').dataset.approved='true';voiceStatus('Approved text is ready. Preview it on the robot speaker before delivery.');deliveryStatus('Approved for '+profile.target_label+'. Review the robot preview, then start the supervised delivery.')
  }catch(error){voiceStatus(error.message,'error');deliveryStatus('Message was not prepared; the robot will not start.','error')}
  finally{deliveryBusy=false;await refresh()}
}
async function previewMessage(){
  if(!preparedDeliveryId||deliveryBusy)return;deliveryBusy=true;updateDeliveryControls(latestStatus);deliveryStatus('Playing the approved message through the robot speaker…');
  try{await post('/api/delivery/preview',JSON.stringify({delivery_id:preparedDeliveryId}));deliveryStatus('Speaker preview finished. This checks playback; it does not start the robot.');voiceStatus('Robot speaker preview complete.')}
  catch(error){deliveryStatus(error.message,'error');voiceStatus(error.message,'error')}
  finally{deliveryBusy=false;await refresh()}
}
async function startDeliveryMission(){
  let cable=document.getElementById('cableClear');if(!cable.checked)return alert('First place the robot at the marked cable-neutral heading and confirm the cable is clear.');if(!preparedDeliveryId)return alert('Approve and preview the message first.');
  try{document.getElementById('findDeliver').disabled=true;await post('/api/mission/deliver',JSON.stringify({delivery_id:preparedDeliveryId,cable_zero_confirmed:true,profile_id:latestStatus.profile_id,profile_revision:latestStatus.profile_revision}));cable.checked=false;deliveryStatus('Searching for the selected person. Speech requires a current unique identity, a measured validated standoff, and stopped motor feedback.');await refresh()}
  catch(error){deliveryStatus(error.message,'error');await refresh()}
}
let driveToken=sessionStorage.getItem('echoraDriveToken'),driveSequence=Number(sessionStorage.getItem('echoraDriveSequence')||'-1'),driveDirection=null,driveTimer=null,drivePulsePending=false;
const TILT_IDS=['tiltUp','tiltDown','tiltUpCoarse','tiltDownCoarse'];
const DRIVE_IDS=['driveForward','driveBack','driveLeft','driveRight','driveStop'];
function setTiltDisabled(value){for(let id of TILT_IDS){let button=document.getElementById(id);if(button)button.disabled=value}}
function clearDriveSession(){driveToken=null;driveSequence=-1;sessionStorage.removeItem('echoraDriveToken');sessionStorage.removeItem('echoraDriveSequence')}
function nextDriveSequence(){driveSequence+=1;sessionStorage.setItem('echoraDriveSequence',String(driveSequence));return driveSequence}
function setDriveDisabled(value){for(let id of DRIVE_IDS){let button=document.getElementById(id);if(button)button.disabled=value}}
function formatState(value){let text=String(value||'idle').replaceAll('_',' ');return text.replace(/^./,character=>character.toUpperCase())}
function setText(id,value){let element=document.getElementById(id),text=String(value??'');if(element&&element.textContent!==text)element.textContent=text}
function setTone(id,tone){let element=document.getElementById(id);if(!element)return;if(tone)element.dataset.tone=tone;else element.removeAttribute('data-tone')}
function setIdle(id,idle){let element=document.getElementById(id);if(element)element.dataset.idle=idle?'true':'false'}
function setService(label,tone){setText('service',label);document.getElementById('serviceBadge').dataset.tone=tone}
function setHealth(id,state,label){let value=document.getElementById(id);if(value.textContent!==label)value.textContent=label;value.closest('.health-item').dataset.state=state}
function wakeControl(id,enabled){let button=document.getElementById(id);if(!button)return;if(enabled&&button.dataset.armed==='0'){button.classList.add('wake');setTimeout(()=>button.classList.remove('wake'),820)}button.dataset.armed=enabled?'1':'0'}
function updateHeading(drive){
  let heading=drive.cable_heading_degrees,limit=Number(drive.usable_cable_limit_degrees),marker=document.getElementById('headingMarker'),meter=document.getElementById('headingMeter');
  if(heading===null||!Number.isFinite(Number(heading))||!Number.isFinite(limit)||limit<=0){
    setText('driveHeading','—');setIdle('driveHeading',true);setText('headingLimit','±—');
    marker.style.left='50%';marker.style.opacity='.3';meter.removeAttribute('aria-valuenow');return;
  }
  heading=Number(heading);let percent=Math.max(4,Math.min(96,50+(heading/(2*limit))*100));
  setText('driveHeading',(heading>0?'+':'')+heading.toFixed(1)+'°');setIdle('driveHeading',false);
  setText('headingLimit','±'+limit+'°');
  marker.style.left=percent+'%';marker.style.opacity='1';
  meter.setAttribute('aria-valuemin',String(-limit));meter.setAttribute('aria-valuemax',String(limit));meter.setAttribute('aria-valuenow',String(heading));
}
function updateHeadGauge(head){
  let value=Number(head.position),marker=document.getElementById('headMarker'),gauge=document.getElementById('headGauge');
  if(!head.available||!Number.isFinite(value)){
    setText('headAngle','—');setIdle('headAngle',true);setText('headRange','OFFLINE');
    marker.style.left='50%';marker.style.opacity='.3';gauge.removeAttribute('aria-valuenow');return;
  }
  setText('headAngle',(value>0?'+':'')+value.toFixed(1)+'°');setIdle('headAngle',false);
  let limits=head.saved_limits,low=value-45,high=value+45;
  if(limits&&Number.isFinite(Number(limits.lower))&&Number.isFinite(Number(limits.upper))){
    low=Math.min(Number(limits.lower),Number(limits.upper));high=Math.max(Number(limits.lower),Number(limits.upper));
  }
  if(!(high-low>=1)){low=value-45;high=value+45}
  setText('headRange',Math.round(low)+' … '+Math.round(high)+'°');
  marker.style.left=Math.max(4,Math.min(96,((value-low)/(high-low))*100))+'%';marker.style.opacity='1';
  gauge.setAttribute('aria-valuemin',String(Math.round(low)));gauge.setAttribute('aria-valuemax',String(Math.round(high)));gauge.setAttribute('aria-valuenow',String(value));
}
function updateLink(){
  let value=document.getElementById('linkValue'),bar=document.getElementById('linkBar');
  if(!lastOk){value.textContent='—';value.dataset.idle='true';bar.style.width='0%';return}
  let age=(Date.now()-lastOk)/1000;
  value.textContent=age.toFixed(1)+' s';value.dataset.idle=age>3?'true':'false';
  bar.style.width=Math.max(0,Math.min(100,100-(age/3)*100)).toFixed(1)+'%';
}
function updateMissionControls(s=latestStatus){
  if(!s)return;
  let m=s.mission||{},d=s.manual_drive||{},h=s.camera_head||{};
  let delivery=s.delivery||{},deliveryActive=['preparing','previewing','searching','approaching','speaking'].includes(delivery.state);
  let checked=document.getElementById('cableClear').checked,driveChecked=document.getElementById('driveCableClear').checked;
  let limitsReady=!!h.homed&&h.reference_id===h.approved_reference_id&&!h.manual_override;
  let missionReady=!!s.target_label&&s.camera_ready&&s.robot_ready&&h.available&&!h.moving&&!h.homing&&!s.enrolling&&!m.running&&!d.enabled&&!deliveryActive&&checked;
  let driveReady=s.camera_ready&&s.robot_ready&&d.odometry_ready&&!m.running&&!s.enrolling&&!d.enabled&&!deliveryActive&&driveChecked;
  let missionMissing=[];
  if(!s.target_label)missionMissing.push('enroll a target');
  if(!s.camera_ready)missionMissing.push('live camera');
  if(!s.robot_ready)missionMissing.push('EV3 connection');
  if(!h.available)missionMissing.push('camera-head connection');else if(h.moving||h.homing)missionMissing.push('camera head to stop');
  if(s.enrolling)missionMissing.push('finish enrollment');
  if(d.enabled)missionMissing.push('lock manual drive');
  if(!checked&&!m.running)missionMissing.push('confirm cable safety');
  let driveMissing=[];
  if(!s.camera_ready)driveMissing.push('live camera');
  if(!s.robot_ready)driveMissing.push('EV3 connection');
  if(!d.odometry_ready)driveMissing.push('fresh odometry');
  if(m.running)driveMissing.push('stop the mission');
  if(s.enrolling)driveMissing.push('finish enrollment');
  if(!driveChecked&&!d.enabled)driveMissing.push('confirm cable safety');
  setText('missionReadiness',m.running?'Mission in progress. Stop all motion remains available above.':missionMissing.length?'Waiting for: '+missionMissing.join(', ')+'.':limitsReady?'Ready to search the room.':'Ready. Camera setup will run automatically before searching.');
  setText('driveReadiness',d.enabled?'Controls are enabled for this page. Hold a direction below.':driveMissing.length?'Waiting for: '+driveMissing.join(', ')+'.':'Ready to enable supervised driving.');
  document.getElementById('findPerson').disabled=!missionReady||selectingProfile;
  document.getElementById('findPerson').title=missionMissing.length?'Waiting for '+missionMissing.join(', '):'Start the bounded find-person mission';
  document.getElementById('cableClear').disabled=!!m.running||!!d.enabled||deliveryActive;
  document.getElementById('driveCableClear').disabled=!!m.running||!!d.enabled||deliveryActive;
  document.getElementById('enableDrive').disabled=!driveReady;
  document.getElementById('enableDrive').title=driveMissing.length?'Waiting for '+driveMissing.join(', '):'Enable supervised manual driving';
  document.getElementById('disableDrive').disabled=!d.enabled;
  wakeControl('enableDrive',driveReady);
  wakeControl('findPerson',missionReady);
  setDriveDisabled(!d.enabled||!driveToken||!s.camera_ready||!s.robot_ready||!d.odometry_ready||!!m.running||deliveryActive);
  updateEnrollmentControls(s);
  document.getElementById('deleteTarget').disabled=!!m.running||!!d.enabled||!!s.enrolling||deliveryActive||!s.profile_id;
  updateDeliveryControls(s);
}
let updateFamilyId=null,addingFamilyMember=false;
let selectingProfile=false;
function openSettings(){let dialog=document.getElementById('settingsDialog');if(!dialog.open)dialog.showModal();document.getElementById('familyPanel').open=true}
function closeSettings(){document.getElementById('settingsDialog').close()}
function refreshFamily(s){
  setText('familyTracking',(s.family_tracking||{}).tracking_state||'Looking for a clearer view');
  let profiles=s.family_profiles||[],select=document.getElementById('familySelect');
  let signature=JSON.stringify(profiles);
  if(select.dataset.signature!==signature){select.replaceChildren();for(let p of profiles){let o=document.createElement('option');o.value=p.id;o.textContent=p.label+' · '+p.sample_count+' views';select.append(o)}select.dataset.signature=signature}
  select.value=s.profile_id||'';
  let deliveryActive=['preparing','previewing','searching','approaching','speaking'].includes((s.delivery||{}).state);
  let locked=!!(s.mission||{}).running||s.enrolling||selectingProfile||!!(s.manual_drive||{}).enabled||deliveryActive;
  select.disabled=locked||!profiles.length;
  document.getElementById('addProfile').disabled=locked;
  for(let button of document.querySelectorAll('[data-profile-action]'))button.disabled=locked||!profiles.length;
  setText('profilesSummary',profiles.length?profiles.length+' saved '+(profiles.length===1?'person':'people')+'. Choose anyone in Find Me.':'Add a person using the phone or robot camera.');
  let cards=document.getElementById('findProfiles'),cardSignature=JSON.stringify([profiles,s.profile_id]);
  if(cards.dataset.signature!==cardSignature){
    cards.replaceChildren();
    for(let p of profiles){
      let button=document.createElement('button'),avatar=document.createElement('span'),name=document.createElement('span'),meta=document.createElement('span');
      button.type='button';button.className='key profile-card';button.setAttribute('aria-pressed',String(p.id===s.profile_id));
      avatar.className='profile-avatar';avatar.setAttribute('aria-hidden','true');avatar.textContent=p.label.split(/\s+/).map(x=>x[0]||'').slice(0,2).join('').toUpperCase();
      name.className='profile-name';name.textContent=p.label;meta.className='profile-meta';meta.textContent=p.id===s.profile_id?'Selected':'Enrolled · '+p.sample_count+' views';
      button.append(avatar,name,meta);button.onclick=()=>selectProfile(p.id);cards.append(button)
    }
    cards.dataset.signature=cardSignature
  }
  for(let button of cards.querySelectorAll('button'))button.disabled=locked;
  document.getElementById('addFirstProfile').hidden=!!profiles.length;
  document.getElementById('addFirstProfile').disabled=locked;
}
async function selectProfile(id){
  if(selectingProfile)return;selectingProfile=true;document.getElementById('findPerson').disabled=true;
  invalidateDeliveryDraft();
  if(latestStatus)refreshFamily(latestStatus);
  try{await post('/api/family/action',JSON.stringify({action:'select',profile_id:id}));document.getElementById('wardrobeGallery').replaceChildren()}
  catch(e){alert(e.message)}finally{selectingProfile=false;await refresh()}
}
async function familyAction(action,extra={}){invalidateDeliveryDraft();try{await post('/api/family/action',JSON.stringify({action,profile_id:document.getElementById('familySelect').value,...extra}));await refresh()}catch(e){alert(e.message)}}
async function familySelect(){await selectProfile(document.getElementById('familySelect').value)}
async function familyRename(){let name=prompt('Name for this person');if(name)await familyAction('rename',{label:name})}
async function familyDelete(){if(confirm('Delete this person and their remembered clothes?'))await familyAction('delete')}
function familyEnroll(update){
  openSettings();
  let select=document.getElementById('familySelect');
  updateFamilyId=update?select.value:null;addingFamilyMember=!update;
  if(update&&!updateFamilyId)return;
  document.getElementById('label').value=update?((latestStatus.family_profiles||[]).find(p=>p.id===select.value)||{}).label||'':'';
  setText('targetSummary',update?document.getElementById('label').value+' · update face views':'New person');
  setText('targetChip',update?'UPDATE':'NEW');document.getElementById('deleteTarget').hidden=!update;
  document.getElementById('consent').checked=false;updateEnrollmentControls();
  let panel=document.getElementById('targetPanel');panel.open=true;panel.scrollIntoView({behavior:'smooth'});
  document.getElementById('label').focus();
  document.getElementById('message').textContent=update?'Update the face views for this person.':'Enroll a new family member below.';
}
async function familyGallery(){
  try{
    let r=await fetch('/api/family/wardrobe');if(!r.ok)throw new Error('Clothes could not be loaded.');
    let data=await r.json(),gallery=document.getElementById('wardrobeGallery');gallery.replaceChildren();
    if(!data.outfits.length){gallery.textContent='No clothes remembered for this person yet.';return}
    for(let outfit of data.outfits){
      let card=document.createElement('div'),img=document.createElement('img'),label=document.createElement('p'),button=document.createElement('button');
      card.style.maxWidth='240px';img.src='data:image/jpeg;base64,'+outfit.images[0];img.width=160;img.alt='Remembered clothing';
      label.textContent=(outfit.attributes||[]).filter(a=>a.visible).map(a=>a.detail||[a.colour,a.pattern,{upper:'top',lower:'trousers or skirt',footwear:'footwear'}[a.region]].filter(x=>x&&x!=='unknown').join(' ')).join(' · ')||'Saved outfit';
      button.className='key';button.textContent='Forget these clothes';button.onclick=async()=>{await familyAction('forget',{profile_id:outfit.profile_id,outfit_id:outfit.id});await familyGallery()};
      card.append(img,label,button);gallery.append(card)
    }
  }catch(e){alert(e.message)}
}
async function startMission(){let cable=document.getElementById('cableClear');if(!cable.checked)return alert('First place the robot at the marked cable-neutral heading and confirm the cable is clear.');try{document.getElementById('findPerson').disabled=true;await post('/api/mission/start',JSON.stringify({cable_zero_confirmed:true,profile_id:latestStatus.profile_id,profile_revision:latestStatus.profile_revision}));cable.checked=false;await refresh()}catch(e){alert(e.message);await refresh()}}
async function stopMission(){try{stopDriveLoop();clearDriveSession();await post('/api/mission/stop','{}');document.getElementById('cableClear').checked=false;document.getElementById('driveCableClear').checked=false;await refresh()}catch(e){alert(e.message);await refresh()}}
async function enableDrive(){let cable=document.getElementById('driveCableClear');if(!cable.checked)return alert('First place the robot at cable-neutral and confirm you are supervising.');try{let d=await post('/api/drive/enable',JSON.stringify({cable_zero_confirmed:true}));driveToken=d.control_token;driveSequence=-1;sessionStorage.setItem('echoraDriveToken',driveToken);sessionStorage.setItem('echoraDriveSequence','-1');cable.checked=false;await refresh()}catch(e){alert(e.message);await refresh()}}
function stopDriveLoop(){if(driveTimer)clearInterval(driveTimer);driveTimer=null;driveDirection=null}
async function sendDrive(direction,showError=true){if(!driveToken||direction!=='stop'&&drivePulsePending)return;let token=driveToken,sequence=nextDriveSequence(),pulse=direction!=='stop';if(pulse)drivePulsePending=true;try{await post('/api/drive/command',JSON.stringify({direction,control_token:token,sequence}))}catch(e){if(direction!=='stop'&&driveDirection===direction){stopDriveLoop();if(showError)alert(e.message)}await refresh()}finally{if(pulse)drivePulsePending=false}}
function isDriveKey(event){return event&&['keydown','keyup'].includes(event.type)&&(event.key===' '||event.key==='Enter')}
function beginDrive(direction,event){
  if(event&&event.type==='keydown'&&!isDriveKey(event))return;
  if(event)event.preventDefault();
  if(event&&event.repeat||event.currentTarget.disabled||driveDirection)return;
  if(event&&Number.isInteger(event.pointerId)&&event.currentTarget.setPointerCapture){try{event.currentTarget.setPointerCapture(event.pointerId)}catch(error){}}
  driveDirection=direction;sendDrive(direction);driveTimer=setInterval(()=>{if(driveDirection===direction)sendDrive(direction)},120);
}
function releaseDrive(event,force=false){if(event&&['keydown','keyup'].includes(event.type)&&!isDriveKey(event))return;if(!driveDirection&&!force)return;if(event)event.preventDefault();stopDriveLoop();sendDrive('stop',false)}
async function disableDrive(){stopDriveLoop();clearDriveSession();try{await post('/api/drive/disable','{}');document.getElementById('driveCableClear').checked=false;await refresh()}catch(e){alert(e.message);await refresh()}}
function abandonDrive(){if(!(driveToken||(latestStatus&&latestStatus.manual_drive&&latestStatus.manual_drive.enabled)))return;stopDriveLoop();clearDriveSession();fetch('/api/drive/disable',{method:'POST',headers:{...H,'Content-Type':'application/json'},body:'{}',keepalive:true})}
window.addEventListener('pointerup',e=>releaseDrive(e));window.addEventListener('pointercancel',e=>releaseDrive(e));window.addEventListener('keyup',e=>releaseDrive(e));window.addEventListener('blur',abandonDrive);window.addEventListener('pagehide',abandonDrive);document.addEventListener('visibilitychange',()=>{if(document.hidden)abandonDrive()});
async function jogCamera(direction,magnitude){try{setTiltDisabled(true);await post('/api/camera/jog',JSON.stringify({direction,magnitude}));setTimeout(refresh,350)}catch(e){alert(e.message);await refresh()}}
let savingCameraLimit=false;
function updateCameraLimits(h,s){
  let limits=h.saved_limits,matching=!!limits&&limits.reference_id===h.reference_id;
  let ready=!!h.available&&!!s.robot_ready&&!!s.camera_ready&&!h.moving&&!h.homing&&!savingCameraLimit;
  let recoverable=!!limits&&Number.isFinite(limits.lower)&&Number.isFinite(limits.upper)&&limits.lower>limits.upper&&!!limits.reference_id&&!!h.reference_id&&!matching;
  document.getElementById('saveLowerLimit').disabled=!ready;
  document.getElementById('saveUpperLimit').disabled=!ready||!matching;
  document.getElementById('cameraRangeRecovery').hidden=!recoverable;
  document.getElementById('restoreCameraRange').disabled=!ready||!recoverable;
  setText('cameraLimitStatus',savingCameraLimit?'Saving…':!h.available?'Turn on the EV3 to set camera limits.':recoverable?'Return to the saved lower view, then use the previous range. You can also save both limits again.':limits&&!matching?'Camera reference changed. Set the lower and upper limits again.':!matching?'Set the lower limit first.':limits.upper===null?'Lower limit saved. Raise the camera, then save the upper limit.':h.manual_override?'Both limits saved. Move back inside the saved range before Find Me.':'Both limits saved. Ready to search; the floor is checked before driving.');
}
async function saveCameraLimit(limit){
  savingCameraLimit=true;document.getElementById('saveLowerLimit').disabled=true;document.getElementById('saveUpperLimit').disabled=true;document.getElementById('restoreCameraRange').disabled=true;
  setText('cameraLimitStatus','Saving…');
  try{await post('/api/camera/limit',JSON.stringify({limit}))}catch(e){alert(e.message)}
  finally{savingCameraLimit=false;await refresh()}
}
async function restoreCameraRange(){
  savingCameraLimit=true;document.getElementById('saveLowerLimit').disabled=true;document.getElementById('saveUpperLimit').disabled=true;document.getElementById('restoreCameraRange').disabled=true;
  setText('cameraLimitStatus','Restoring the saved range…');
  try{await post('/api/camera/restore-range',JSON.stringify({operator_confirmed:true}))}catch(e){alert(e.message)}
  finally{savingCameraLimit=false;await refresh()}
}
// PHONE_ENROLLMENT_BEGIN
let enrollmentToken=sessionStorage.getItem('echoraEnrollmentSession'),enrollmentBusy=false;
const phone={stream:null,timer:null,controller:null,generation:0,requesting:false,inFlight:false,facing:'user',ready:false};
function rememberEnrollment(token){enrollmentToken=token;try{if(token)sessionStorage.setItem('echoraEnrollmentSession',token);else sessionStorage.removeItem('echoraEnrollmentSession')}catch(e){}}
function ownsEnrollment(s=latestStatus){return !!s&&s.enrolling&&!!enrollmentToken&&s.enrollment_session_id===enrollmentToken}
function phoneAvailable(){return window.isSecureContext&&!!navigator.mediaDevices?.getUserMedia}
function updateEnrollmentControls(s=latestStatus){
  let enrolling=!!s?.enrolling,ours=ownsEnrollment(s),source=document.getElementById('enrollmentSource');
  if(enrolling)source.value=s.enrollment_source||'robot';
  let isPhone=source.value==='phone',deliveryActive=['preparing','previewing','searching','approaching','speaking'].includes(s?.delivery?.state),locked=!s||!!s.mission?.running||!!s.manual_drive?.enabled||deliveryActive;
  source.disabled=enrolling||enrollmentBusy;
  document.getElementById('phoneCapture').hidden=!isPhone;
  document.getElementById('robotEnrollmentHelp').hidden=isPhone;
  let robotPreview=document.getElementById('robotEnrollmentPreview');
  robotPreview.hidden=isPhone||!s?.camera_ready;
  if(!isPhone&&s?.camera_ready&&document.getElementById('settingsDialog').open&&document.getElementById('targetPanel').open){if(!robotPreview.getAttribute('src'))robotPreview.src='/stream?enrollment=1'}
  else robotPreview.removeAttribute('src');
  document.getElementById('secureCameraHelp').hidden=!isPhone||phoneAvailable();
  for(let id of ['label','retention','consent'])document.getElementById(id).disabled=enrolling||enrollmentBusy;
  document.getElementById('start').disabled=locked||enrolling||enrollmentBusy||!document.getElementById('label').value.trim()||!document.getElementById('consent').checked;
  document.getElementById('finish').disabled=!ours||!s.ready||enrollmentBusy;
  document.getElementById('cancel').disabled=!enrolling||enrollmentBusy;
  document.getElementById('upload').disabled=!ours||enrollmentBusy;
  document.getElementById('files').disabled=!ours||enrollmentBusy;
  document.getElementById('resumePhone').disabled=!ours||!isPhone||!phoneAvailable()||!!phone.stream||phone.requesting||enrollmentBusy||s.ready;
  document.getElementById('pausePhone').disabled=!phone.stream&&!phone.requesting;
  document.getElementById('switchPhone').disabled=!phone.stream||phone.requesting||enrollmentBusy||!!s?.ready;
  document.getElementById('switchPhone').textContent=phone.facing==='user'?'Use rear camera':'Use selfie camera';
  if(s&&(!ours||!isPhone)&&(phone.stream||phone.requesting))pausePhoneCapture('Camera stopped because the enrollment changed.');
  if(ours&&s.ready){phone.ready=true;if(phone.timer)clearTimeout(phone.timer);phone.timer=null;setText('phoneMessage','All required views collected. Finish enrollment to save this person.')}
  if(enrolling&&!ours)setText('phoneMessage','Enrollment is open in another page. Cancel that enrollment here before starting a new one.');
  if(s&&!enrolling&&!enrollmentBusy&&enrollmentToken)rememberEnrollment(null);
}
function pausePhoneCapture(message='Camera paused. Resume to continue collecting views.'){
  phone.generation++;if(phone.timer)clearTimeout(phone.timer);phone.timer=null;
  if(phone.controller)phone.controller.abort();phone.controller=null;
  if(phone.stream)for(let track of phone.stream.getTracks())track.stop();
  phone.stream=null;phone.requesting=false;phone.inFlight=false;
  let video=document.getElementById('phonePreview');video.pause();video.srcObject=null;video.hidden=true;
  setText('phoneMessage',message);
  document.getElementById('pausePhone').disabled=true;document.getElementById('switchPhone').disabled=true;
  document.getElementById('resumePhone').disabled=!ownsEnrollment()||!!latestStatus?.ready||!phoneAvailable();
}
async function resumePhoneCapture(){
  if(!ownsEnrollment()||latestStatus.ready||phone.requesting||phone.stream)return;
  if(!phoneAvailable()){setText('phoneMessage','Open the secure console to use live capture, or add photos below.');return}
  const generation=++phone.generation,token=enrollmentToken;
  phone.requesting=true;phone.ready=false;updateEnrollmentControls();
  setText('phoneMessage','Allow camera access, then look straight at the camera.');
  try{
    let stream=await navigator.mediaDevices.getUserMedia({audio:false,video:{facingMode:{ideal:phone.facing},width:{ideal:1280},height:{ideal:960}}});
    if(generation!==phone.generation||token!==enrollmentToken||document.hidden){stream.getTracks().forEach(t=>t.stop());return}
    phone.stream=stream;let track=stream.getVideoTracks()[0];
    phone.facing=track.getSettings().facingMode||phone.facing;
    track.addEventListener('ended',()=>{if(phone.stream===stream)pausePhoneCapture('Camera access ended. Resume to try again.')});
    let video=document.getElementById('phonePreview');video.srcObject=stream;video.dataset.facing=phone.facing;video.hidden=false;
    await video.play();
    if(generation!==phone.generation)return;
    setText('phoneMessage','Look straight ahead. Clear views are collected automatically.');
    phone.timer=setTimeout(()=>capturePhoneFrame(generation,token),500);
  }catch(e){
    if(generation===phone.generation)pausePhoneCapture(e.name==='NotAllowedError'?'Camera permission was not granted. Allow it in your browser settings and resume, or upload photos.':e.name==='NotFoundError'?'No phone camera was found. You can upload photos or use the robot camera.':'Could not open the camera. Close other camera apps and resume, or upload photos.');
  }finally{if(generation===phone.generation){phone.requesting=false;updateEnrollmentControls()}}
}
async function switchPhoneCamera(){
  let next=phone.facing==='user'?'environment':'user';pausePhoneCapture();phone.facing=next;await resumePhoneCapture();
}
async function jpegBlob(source,width,height){
  if(!width||!height)throw Error('Waiting for a clear camera frame.');
  let scale=Math.min(1,1600/Math.max(width,height)),canvas=document.createElement('canvas');
  canvas.width=Math.max(1,Math.round(width*scale));canvas.height=Math.max(1,Math.round(height*scale));
  canvas.getContext('2d').drawImage(source,0,0,canvas.width,canvas.height);
  return await new Promise((resolve,reject)=>canvas.toBlob(blob=>blob?resolve(blob):reject(Error('Could not prepare this image.')),'image/jpeg',.92));
}
async function capturePhoneFrame(generation,token){
  if(generation!==phone.generation||token!==enrollmentToken||!phone.stream||phone.ready||phone.inFlight)return;
  if(document.hidden||!document.getElementById('settingsDialog').open){pausePhoneCapture();return}
  phone.inFlight=true;let timeout;
  try{
    let video=document.getElementById('phonePreview'),blob=await jpegBlob(video,video.videoWidth,video.videoHeight);
    if(generation!==phone.generation)return;
    let controller=new AbortController();phone.controller=controller;timeout=setTimeout(()=>controller.abort(),10000);
    let result=await post('/api/enrollment/frame',blob,'image/jpeg',{headers:{'X-Echora-Enrollment':token},signal:controller.signal});
    if(generation!==phone.generation)return;
    phone.ready=!!result.ready;setText('phoneMessage',result.message);setText('guide',result.guidance);
    if(result.ready)setText('phoneMessage','All required views collected. Finish enrollment to save this person.');
    await refresh();
  }catch(e){
    if(generation!==phone.generation)return;
    if(e.status===400){setText('phoneMessage',e.message)}
    else{pausePhoneCapture('Capture paused: connection interrupted. Reconnect, then resume.');return}
  }finally{
    if(timeout)clearTimeout(timeout);
    if(generation===phone.generation){phone.inFlight=false;phone.controller=null;if(!phone.ready&&phone.stream)phone.timer=setTimeout(()=>capturePhoneFrame(generation,token),1000)}
  }
}
async function startEnrollment(){
  if(enrollmentBusy)return;enrollmentBusy=true;updateEnrollmentControls();
  let source=document.getElementById('enrollmentSource').value;
  let body={profile_id:updateFamilyId,label:document.getElementById('label').value,retention:document.getElementById('retention').value,consent:document.getElementById('consent').checked,camera_source:source};
  try{
    let s=await post('/api/enrollment/start',JSON.stringify(body));rememberEnrollment(s.enrollment_session_id);latestStatus=s;updateFamilyId=null;phone.ready=false;
    enrollmentBusy=false;await refresh();if(source==='phone'){document.getElementById('phoneCapture').scrollIntoView({block:'start',behavior:'smooth'});await resumePhoneCapture()}
  }catch(e){pausePhoneCapture();alert(e.message)}finally{enrollmentBusy=false;updateEnrollmentControls()}
}
async function enrollmentAction(url){
  if(enrollmentBusy)return;
  const token=ownsEnrollment()?enrollmentToken:latestStatus?.enrollment_session_id;
  if(!token)return;
  enrollmentBusy=true;pausePhoneCapture();updateEnrollmentControls();
  try{await post(url,JSON.stringify({session_id:token}));rememberEnrollment(null);addingFamilyMember=false;updateFamilyId=null;
    if(url.endsWith('/finish')){document.getElementById('familyPanel').open=true;document.getElementById('targetPanel').open=false}
  }catch(e){alert(e.message)}finally{enrollmentBusy=false;await refresh()}
}
async function normalizedBlob(file){
  if(typeof createImageBitmap==='function'){
    let img;
    try{img=await createImageBitmap(file,{imageOrientation:'from-image'});return await jpegBlob(img,img.width,img.height)}
    catch(e){/* Older mobile decoders can fall back to an image element. */}
    finally{if(img)img.close()}
  }
  let url=URL.createObjectURL(file);
  try{let img=new Image();img.src=url;await img.decode();return await jpegBlob(img,img.naturalWidth,img.naturalHeight)}
  finally{URL.revokeObjectURL(url)}
}
async function uploadPhotos(){
  if(enrollmentBusy||!ownsEnrollment())return;
  let fs=[...document.getElementById('files').files],token=enrollmentToken;
  if(!fs.length)return alert('Choose one or more photos first.');
  pausePhoneCapture('Adding selected photos. Resume the camera afterwards if more views are needed.');enrollmentBusy=true;updateEnrollmentControls();
  try{for(let f of fs){
    if(token!==enrollmentToken)break;
    try{let b=await normalizedBlob(f);let r=await post('/api/enrollment/upload',b,'image/jpeg',{headers:{'X-Echora-Enrollment':token}});setText('phoneMessage',r.message);setText('message',r.message)}
    catch(e){alert(f.name+': '+e.message)}
  }}finally{document.getElementById('files').value='';enrollmentBusy=false;await refresh()}
}

// PHONE_ENROLLMENT_END
async function deleteTarget(){if(!confirm('Delete the enrolled target and any retained face crops?'))return;await action('/api/enrollment/delete')}
function setOfflineView(){
  pausePhoneCapture('Connection lost. Reconnect, then resume capture.');
  latestStatus=null;stopDriveLoop();clearDriveSession();document.body.dataset.connection='offline';
  for(let button of document.querySelectorAll('#findProfiles button,[data-profile-action]'))button.disabled=true;
  for(let id of ['familySelect','addProfile','addFirstProfile','files','resumePhone'])document.getElementById(id).disabled=true;
  setService('Console offline','offline');
  setHealth('cameraHealth','offline','Unavailable');setHealth('robotHealth','offline','Unavailable');setHealth('odomHealth','offline','Unavailable');setHealth('headHealth','offline','Unavailable');
  setText('systemsChip','OFFLINE');setTone('systemsChip','off');setText('systemsSub','No status from the console');
  let summary=document.getElementById('systemSummary');summary.dataset.tone='offline';setText('systemSummary','Connection lost — all stale controls are locked. The stop control remains available and reports if its command cannot be delivered.');
  document.getElementById('liveLight').dataset.live='false';setText('liveFeedState','Camera unavailable');
  setText('driveState','Driving locked');setTone('driveState','');setText('driveSub','Reconnect the console to drive');
  setText('driveMessage','The console connection is unavailable.');setText('driveReadiness','Reconnect the console before enabling manual control.');
  updateHeading({});updateHeadGauge({});
  setText('missionState','Unavailable');setTone('missionState','off');setText('missionReadiness','Reconnect the console before starting a mission.');
  setText('headState','Camera-head status unavailable.');setText('headChip','OFFLINE');setTone('headChip','off');
  updateCameraLimits({},{});
  document.getElementById('findPerson').disabled=true;document.getElementById('enableDrive').disabled=true;document.getElementById('disableDrive').disabled=true;
  for(let id of ['deliveryText','recordMessage','stopMessageRecording','transcribeMessage','approveMessage','previewMessage','findDeliver'])document.getElementById(id).disabled=true;
  document.getElementById('start').disabled=true;document.getElementById('upload').disabled=true;document.getElementById('cancel').disabled=true;document.getElementById('finish').disabled=true;document.getElementById('deleteTarget').disabled=true;
  document.getElementById('cableClear').checked=false;document.getElementById('driveCableClear').checked=false;document.getElementById('cableClear').disabled=true;document.getElementById('driveCableClear').disabled=true;
  document.getElementById('enableDrive').dataset.armed='0';document.getElementById('findPerson').dataset.armed='0';
  setDriveDisabled(true);setTiltDisabled(true);updateLink();
}
async function refresh(){
  try{
    let response=await fetch('/api/status',{cache:'no-store'});if(!response.ok)throw Error('Status unavailable');let s=await response.json();
    latestStatus=s;lastOk=Date.now();document.body.dataset.connection='connected';
    let m=s.mission||{state:'idle',message:'Ready.'},d=s.manual_drive||{enabled:false,message:'Driving unavailable.'},h=s.camera_head||{};
    if(!d.enabled&&driveToken)clearDriveSession();
    if(d.active)setService('Driving','busy');else if(m.running)setService('Finding','busy');else if(!s.robot_ready)setService('EV3 offline','warning');else if(!s.camera_ready)setService('Camera offline','warning');else if(s.enrolling)setService('Capturing','busy');else setService('Ready','ready');

    setHealth('cameraHealth',s.camera_ready?'online':'offline',s.camera_ready?'Live':'Unavailable');
    setHealth('robotHealth',s.robot_ready?'online':'offline',s.robot_ready?'Ready':'Offline');
    setHealth('odomHealth',d.odometry_ready?'online':'offline',d.odometry_ready?'Fresh':'Unavailable');
    let headReady=!!h.available&&!!h.homed&&!!h.calibrated,headLabel=!h.available?'Offline':h.homing?'Homing…':h.moving?'Moving…':!h.homed?'Not homed':!h.calibrated?'Needs calibration':'Ready';
    setHealth('headHealth',headReady?'online':h.available?'warning':'offline',headLabel);

    let faults=[];
    if(!s.camera_ready)faults.push('camera');
    if(!s.robot_ready)faults.push('EV3');
    if(!d.odometry_ready)faults.push('odometry');
    if(!headReady)faults.push('camera head');
    setText('systemsChip',faults.length?faults.length+(faults.length===1?' FAULT':' FAULTS'):'ALL CLEAR');
    setTone('systemsChip',faults.length?'off':'live');
    setText('systemsSub',faults.length?faults.join(', ')+' need attention':'Camera, EV3, odometry and head all reporting');

    let summary=document.getElementById('systemSummary');summary.dataset.tone='warning';
    if(d.active)setText('systemSummary','Manual driving is active — keep the route in view and release to stop.');
    else if(m.running)setText('systemSummary','Find-person mission is active — keep the live view visible. Stop all motion remains available.');
    else if(!s.robot_ready)setText('systemSummary',s.camera_ready?'EV3 offline — all movement controls are locked. Camera and enrollment remain available.':'EV3 and camera are offline — all movement controls are locked.');
    else if(!s.camera_ready)setText('systemSummary','Camera unavailable — movement controls are locked until a fresh view returns.');
    else if(!d.odometry_ready)setText('systemSummary','Waiting for fresh odometry — supervised driving remains locked.');
    else if(!headReady)setText('systemSummary','Supervised driving is available. Find person will calibrate the camera before searching.');
    else{summary.dataset.tone='ready';setText('systemSummary','All core systems are ready. Choose supervised driving or a bounded room search.')}

    document.getElementById('liveLight').dataset.live=String(!!s.camera_ready);setText('liveFeedState',s.camera_ready?'Camera live':'Waiting for fresh camera');
    setText('driveState',d.active?'Moving '+formatState(d.direction):d.enabled?'Driving enabled':'Driving locked');
    setTone('driveState',d.active?'live':'');
    setText('driveSub',d.active?'Release to stop the tracks':d.enabled?'Controls live for this page':'Hold-to-move, gated on a cable check');
    setText('driveMessage',d.message||'');updateHeading(d);updateHeadGauge(h);
    let activeButtons={forward:'driveForward',back:'driveBack',left:'driveLeft',right:'driveRight'};
    for(let id of DRIVE_IDS)document.getElementById(id).classList.toggle('is-active',!!d.active&&(activeButtons[d.direction]===id));

    setText('missionState',formatState(m.state));
    setTone('missionState',m.running?'live':'');
    setText('missionMessage',m.message||'');
    document.getElementById('missionTarget').textContent=s.target_label?s.target_label:'No target';
    document.getElementById('findPerson').textContent=s.target_label?'Find '+s.target_label:'Find person';
    document.getElementById('targetSummary').textContent=s.enrolling?(s.enrollment_label||'Person')+' · collecting views':addingFamilyMember?'New person':s.target_label?s.target_label+' · '+(s.target_sample_count||0)+' saved views':'No one enrolled';
    setText('targetChip',s.enrolling?'CAPTURING':addingFamilyMember?'NEW':s.target_label?'ENROLLED':'NONE');
    document.getElementById('deleteTarget').hidden=addingFamilyMember||s.enrolling;
    setTone('targetChip',s.enrolling?'live':s.target_label?'live':'');

    refreshFamily(s);
    let sampleCount=Number(s.sample_count)||0,minimumTotal=Math.max(1,Number(s.minimum_total)||10),progress=Math.min(100,100*sampleCount/minimumTotal),poses=s.pose_counts||{};
    setText('guide',s.guidance||'Wait for camera');
    for(let pose of ['center','left','right'])document.getElementById(pose).textContent=poses[pose]||0;
    document.getElementById('bar').style.width=progress+'%';document.getElementById('progressLabel').textContent=sampleCount+' / '+minimumTotal+' views';
    document.getElementById('enrollmentProgress').setAttribute('aria-valuemax',String(minimumTotal));document.getElementById('enrollmentProgress').setAttribute('aria-valuenow',String(sampleCount));
    setText('enrollValue',sampleCount+' / '+minimumTotal);setIdle('enrollValue',!sampleCount);
    document.getElementById('enrollBar').style.width=progress+'%';
    setText('message',s.message||'');
    document.getElementById('finish').disabled=!s.ready;document.getElementById('upload').disabled=!s.enrolling;document.getElementById('cancel').disabled=!s.enrolling;
    if(s.enrolling&&document.getElementById('settingsDialog').open)document.getElementById('targetPanel').open=true;

    let usable=!!h.available&&!['preparing','previewing','searching','approaching','speaking'].includes((s.delivery||{}).state);
    updateCameraLimits(h,s);
    setText('headState',h.available?(h.moving?'Moving '+h.position+'° → '+h.target_position+'°':'Manual control · position '+h.position+'°'):'Motor unavailable');
    setText('headChip',!h.available?'OFFLINE':h.moving||h.homing?'MOVING':'READY');
    setTone('headChip',!h.available?'off':h.moving||h.homing?'live':'');

    for(let id of ['tiltUp','tiltUpCoarse'])document.getElementById(id).disabled=!usable;
    for(let id of ['tiltDown','tiltDownCoarse'])document.getElementById(id).disabled=!usable;
    updateEnrollmentControls(s);
    updateMissionControls(s);
  }catch(error){setOfflineView()}
  updateLink();
}
let cameraStream=document.getElementById('cameraStream'),streamRetryTimer=null;
cameraStream.addEventListener('load',()=>{document.getElementById('cameraFrame').classList.remove('stream-error');if(streamRetryTimer)clearTimeout(streamRetryTimer);streamRetryTimer=null});
cameraStream.addEventListener('error',()=>{document.getElementById('cameraFrame').classList.add('stream-error');if(!streamRetryTimer)streamRetryTimer=setTimeout(()=>{streamRetryTimer=null;cameraStream.src='/stream?retry='+Date.now()},1800)});
const MODULES=[...document.querySelectorAll('details.mod')];
for(let panel of MODULES)panel.addEventListener('toggle',()=>{if(panel.open)for(let other of MODULES)if(other!==panel&&other.open&&other.parentElement===panel.parentElement)other.open=false});
document.addEventListener('pointermove',event=>{
  let element=event.target&&event.target.closest&&event.target.closest('.key,.opener,.stop-all');
  if(!element)return;
  let box=element.getBoundingClientRect();
  element.style.setProperty('--mx',((event.clientX-box.left)/box.width*100).toFixed(1)+'%');
  element.style.setProperty('--my',((event.clientY-box.top)/box.height*100).toFixed(1)+'%');
},{passive:true});
document.getElementById('cableClear').checked=false;document.getElementById('driveCableClear').checked=false;
document.getElementById('settingsDialog').addEventListener('close',()=>{pausePhoneCapture();document.getElementById('robotEnrollmentPreview').removeAttribute('src')});
document.getElementById('targetPanel').addEventListener('toggle',()=>{if(!document.getElementById('targetPanel').open)pausePhoneCapture();updateEnrollmentControls()});
document.addEventListener('visibilitychange',()=>{if(document.hidden)pausePhoneCapture('Camera paused while this page is in the background. Resume when ready.')});
window.addEventListener('pagehide',()=>pausePhoneCapture());
window.addEventListener('pagehide',()=>{if(['previewing','searching','approaching','speaking'].includes(latestStatus?.delivery?.state))fetch('/api/mission/stop',{method:'POST',headers:{...H,'Content-Type':'application/json'},body:'{}',keepalive:true}).catch(()=>{})});
for(let id of ['label','consent'])document.getElementById(id).addEventListener('input',()=>updateEnrollmentControls());
document.getElementById('deliveryText').addEventListener('input',()=>invalidateDeliveryDraft());
updateEnrollmentControls();setInterval(updateLink,200);setInterval(refresh,900);refresh();
</script></body></html>
""".encode("utf-8")


class Frame(object):
    __slots__ = ("image", "stamp", "frame_id")
    def __init__(self, image, stamp, frame_id):
        self.image, self.stamp, self.frame_id = image, stamp, frame_id


class EnrollmentSession(object):
    def __init__(self, label, retention, camera_source="robot"):
        self.label = label
        self.retention = retention
        self.camera_source = camera_source
        self.session_id = uuid.uuid4().hex
        self.profile_id = None
        self.samples = []
        self.crops = []
        self.message = ("Hold the phone at eye level and look at its camera."
                        if camera_source == "phone" else
                        "Stand 1–2 metres away and look straight at the camera.")
        self.last_capture = 0.0

    @property
    def pose_counts(self):
        return {pose: sum(1 for sample in self.samples if sample["pose"] == pose) for pose in REQUIRED}

    @property
    def ready(self):
        counts = self.pose_counts
        return len(self.samples) >= MINIMUM_TOTAL and all(counts[pose] >= wanted for pose, wanted in REQUIRED.items())

    @property
    def guidance(self):
        counts = self.pose_counts
        if counts["center"] < REQUIRED["center"]:
            return "Look straight at the camera"
        if counts["left"] < REQUIRED["left"] and counts["right"] < REQUIRED["right"]:
            return "Turn your face slightly to either side"
        if counts["left"] < REQUIRED["left"] or counts["right"] < REQUIRED["right"]:
            return "Now turn your face to the other side"
        if len(self.samples) < MINIMUM_TOTAL:
            return "Move naturally for a few more views"
        return "Ready to finish"

    def add(self, embedding, aligned, pose, source, quality):
        if len(self.samples) >= MAXIMUM_SAMPLES:
            self.message = "Enough samples collected. Finish enrollment."
            return False
        if self.samples:
            identity_score = max(
                cosine_similarity(embedding, sample["embedding"])
                for sample in self.samples
            )
            if identity_score < 0.25:
                self.message = "That face does not match the current enrollment."
                return False
        same_pose = [sample["embedding"] for sample in self.samples if sample["pose"] == pose]
        if same_pose and max(cosine_similarity(embedding, item) for item in same_pose) > 0.997:
            self.message = "That view is already covered—change angle slightly."
            return False
        self.samples.append({"embedding": embedding, "pose": pose, "source": source, "quality": quality})
        self.crops.append(aligned.copy())
        self.last_capture = time.monotonic()
        self.message = "Captured {0} view ({1}/{2}).".format(pose, len(self.samples), MINIMUM_TOTAL)
        return True


class EnrollmentNode(Node):
    def __init__(self, args):
        super().__init__("echora_enrollment_console")
        self.args = args
        backend = TensorRTMultiOutputBackend(args.model_path, (112, 112))
        self.recognizer = FaceEmbeddingRecognizer(backend, args.input_mean, args.input_std)
        detector_backend = TensorRTMultiOutputBackend(args.yunet_engine_path, (640, 640))
        self.upload_detector = YuNetFaceDetector(detector_backend, 0.75)
        self.store = FamilyTargetStore(TargetStore(args.store_path, args.model_name, args.model_sha256))
        self.matcher = ExactPairMatcher(24)
        self.lock = threading.RLock()
        # Keep each safety decision and its motor publication in one transaction.
        self.action_lock = threading.Lock()
        self.runtime_lock = threading.Lock()
        self.session = None
        self.family_status = {}
        self.create_subscription(String, "/perception/people_tracks", self.on_family_status, 10)
        self.latest_jpeg = None
        self.raw_frames = OrderedDict()
        self.latest_frame_time = None
        self.last_preview_encode_time = 0.0
        self.camera_head_status = None
        self.camera_head_status_time = None
        self.robot_status = None
        self.robot_status_time = None
        self.target_observation = None
        self.target_observation_time = None
        self.delivery_store = PreparedDeliveryStore()
        self.speech_backend = MacSpeechBackend(args.speech_url)
        self.speech_player = AlsaSpeechPlayer(args.speech_device)
        self.delivery_lock = threading.RLock()
        self.delivery_state = "idle"
        self.delivery_message = ""
        self.delivery_info = None
        self.delivery_previewed_id = None
        self.active_delivery_id = None
        self.active_delivery_cancel = None
        self.preview_cancel_event = None
        self.last_message = "Start an enrollment or add photographs."
        self.create_subscription(Image, args.image_topic, self.on_image, BEST_EFFORT)
        self.create_subscription(String, args.observations_topic, self.on_observations, 10)
        self.create_subscription(String, args.camera_head_status_topic, self.on_camera_head_status, 10)
        self.create_subscription(String, args.robot_status_topic, self.on_robot_status, 10)
        self.create_subscription(String, args.target_observation_topic, self.on_target_observation, 10)
        self.create_subscription(Odometry, args.odom_topic, self.on_odometry, 10)
        self.camera_head_command_publisher = self.create_publisher(String, args.camera_head_command_topic, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, args.cmd_vel_topic, 10)
        self.manual_drive = ManualDriveController(
            linear_speed=args.manual_linear_speed,
            reverse_speed=args.manual_reverse_speed,
            angular_speed=args.manual_angular_speed,
            cable_limit_degrees=args.mission_cable_limit_degrees,
            cable_margin_degrees=args.mission_cable_margin_degrees,
            turn_buffer_degrees=args.manual_turn_buffer_degrees,
            command_timeout_seconds=args.manual_command_timeout_seconds,
            session_timeout_seconds=args.manual_session_timeout_seconds,
        )
        self.mission = FindMissionController(
            args.mission_script,
            args.mission_report,
            args.route_url,
            cable_limit_degrees=args.mission_cable_limit_degrees,
            cable_margin_degrees=args.mission_cable_margin_degrees,
            on_finished=self.on_mission_finished,
        )
        self.create_timer(0.1, self.on_manual_drive_watchdog)

    def on_camera_head_status(self, message):
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not isinstance(status, dict):
            return
        with self.lock:
            self.camera_head_status = status
            self.camera_head_status_time = time.monotonic()

    def on_robot_status(self, message):
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not isinstance(status, dict):
            return
        with self.lock:
            self.robot_status = status
            self.robot_status_time = time.monotonic()

    def on_target_observation(self, message):
        try:
            observation = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not isinstance(observation, dict):
            return
        with self.lock:
            self.target_observation = observation
            self.target_observation_time = time.monotonic()

    def _set_delivery(self, state, message="", info=None):
        with self.delivery_lock:
            self.delivery_state = state
            self.delivery_message = message
            if info is not None:
                self.delivery_info = dict(info)

    def _clear_delivery(self, force=True):
        self.delivery_store.invalidate(force=force)
        with self.delivery_lock:
            self.delivery_previewed_id = None
            if self.active_delivery_id is None:
                self.delivery_state = "idle"
                self.delivery_message = ""
                self.delivery_info = None

    def _delivery_unlocked(self):
        with self.delivery_lock:
            return self.delivery_state not in {"preparing", "previewing", "searching", "approaching", "speaking"}

    def _require_delivery_unlocked(self):
        if not self._delivery_unlocked():
            raise ValueError("Stop the active message delivery before changing robot controls or profiles.")

    def _require_stationary_feedback(self):
        now = time.monotonic()
        with self.lock:
            robot = None if self.robot_status is None else dict(self.robot_status)
            robot_age = None if self.robot_status_time is None else now - self.robot_status_time
            head = None if self.camera_head_status is None else dict(self.camera_head_status)
            head_age = None if self.camera_head_status_time is None else now - self.camera_head_status_time
        if (not isinstance(robot, dict) or robot.get("status") != "ok"
                or robot.get("motion_active") is not False
                or robot_age is None or not 0 <= robot_age <= 1.5):
            raise ValueError("Fresh stopped EV3 feedback is required for speaker playback.")
        motors = robot.get("motors")
        for role in ("left", "right"):
            motor = motors.get(role) if isinstance(motors, dict) else None
            state = motor.get("state") if isinstance(motor, dict) else None
            state_known = (bool(state.strip()) if isinstance(state, str)
                           else all(isinstance(item, str) for item in state)
                           if isinstance(state, (list, tuple, set)) else False)
            running = ("running" in state.lower() if isinstance(state, str)
                       else any(str(item).lower() == "running" for item in state)
                       if isinstance(state, (list, tuple, set)) else True)
            if (not isinstance(motor, dict) or not state_known or running
                    or type(motor.get("speed")) not in (int, float)
                    or not math.isfinite(motor["speed"]) or abs(motor["speed"]) > 1
                    or type(motor.get("commanded_speed")) not in (int, float)
                    or not math.isfinite(motor["commanded_speed"]) or motor["commanded_speed"] != 0):
                raise ValueError("Both tracks must report a stopped state before speaker playback.")
        if (not isinstance(head, dict) or head.get("moving") is not False
                or head.get("homing") is not False or head_age is None or not 0 <= head_age <= 1.5):
            raise ValueError("Fresh stopped camera-head feedback is required for speaker playback.")

    def prepare_delivery(self, request):
        if hasattr(self, "delivery_lock"):
            self._require_delivery_unlocked()
        with self.lock:
            enrolling = self.session is not None
        if enrolling or self.mission.status().get("running") or self.manual_drive.status().get("enabled"):
            raise ValueError("Stop robot movement before preparing a message.")
        try:
            target = self.store.load()
        except Exception as exc:
            raise ValueError("Stored profile error: {0}".format(exc))
        if (not target or request.get("profile_id") != target.get("profile_id")
                or request.get("profile_revision") != target.get("revision")):
            raise ValueError("The selected profile changed. Select the person again before approving.")
        ticket = self.delivery_store.begin(
            target["profile_id"], target["revision"], request.get("message_id"),
            request.get("message_revision"), request.get("text"),
        )
        self._set_delivery("preparing", "Preparing approved speech through the Mac service.", {
            "profile_id": ticket.profile_id, "profile_revision": ticket.profile_revision,
            "message_id": ticket.message_id, "message_revision": ticket.message_revision,
        })
        try:
            audio = self.speech_backend.synthesize(ticket)
            current = self.store.load()
            if (not current or current.get("profile_id") != ticket.profile_id
                    or current.get("revision") != ticket.profile_revision):
                raise SpeechDeliveryError("The selected person changed while speech was being prepared. Approve again.")
            result = self.delivery_store.complete(ticket, audio)
        except Exception as exc:
            self.delivery_store.fail(ticket)
            self._set_delivery("delivery_failed", str(exc))
            if isinstance(exc, SpeechDeliveryError):
                raise ValueError(str(exc)) from None
            raise ValueError("Speech preparation failed: {0}".format(str(exc)[:180])) from None
        with self.delivery_lock:
            self.delivery_previewed_id = None
            self.delivery_state = "message_ready"
            self.delivery_message = "Approved text is ready. Preview it on the robot speaker before delivery."
            self.delivery_info = dict(result)
        return {"ok": True, "delivery_id": ticket.delivery_id, "delivery": self.delivery_status()}

    def transcribe_delivery(self, audio, content_type):
        if hasattr(self, "delivery_lock"):
            self._require_delivery_unlocked()
        if self.mission.status().get("running") or self.manual_drive.status().get("enabled"):
            raise ValueError("Stop robot movement before transcribing a message.")
        try:
            return self.speech_backend.transcribe(audio, content_type)
        except SpeechDeliveryError as exc:
            raise ValueError(str(exc)) from None

    def clear_delivery(self, request):
        cleared = self.delivery_store.invalidate(
            message_id=request.get("message_id"),
            message_revision=request.get("message_revision"),
            force=request.get("force") is True,
        )
        if not cleared:
            return {"ok": True, "cleared": False}
        with self.delivery_lock:
            active = self.active_delivery_id is not None
            self.delivery_previewed_id = None
            preview_cancel = self.preview_cancel_event
            if preview_cancel is not None:
                preview_cancel.set()
        if active:
            self.stop_find_mission()
        else:
            with self.delivery_lock:
                self.delivery_state = "idle"
                self.delivery_message = ""
                self.delivery_info = None
        return {"ok": True, "cleared": True}

    def preview_delivery(self, request):
        if hasattr(self, "delivery_lock"):
            self._require_delivery_unlocked()
        with self.lock:
            enrolling = self.session is not None
        if enrolling or self.mission.status().get("running") or self.manual_drive.status().get("enabled"):
            raise ValueError("Stop robot movement before previewing the message.")
        self._require_stationary_feedback()
        target = self.store.load()
        if not target:
            raise ValueError("Select an enrolled person before previewing the message.")
        cancel_event = threading.Event()
        with self.delivery_lock:
            if self.delivery_state in {"preparing", "previewing", "searching", "approaching", "speaking"}:
                raise ValueError("Wait for the current robot audio or mission action to finish.")
            self.delivery_previewed_id = None
            self.preview_cancel_event = cancel_event
            self.delivery_state = "previewing"
            self.delivery_message = "Playing the approved message through the robot speaker."
        try:
            delivery = self.delivery_store.get(request.get("delivery_id"), target["profile_id"], target["revision"])
            self.speech_player.play(delivery["audio"], cancel_event=cancel_event)
        except (SpeechDeliveryError, ValueError) as exc:
            with self.delivery_lock:
                if self.preview_cancel_event is cancel_event:
                    self.preview_cancel_event = None
                    self.delivery_state = "stopped" if cancel_event.is_set() else "delivery_failed"
                    self.delivery_message = "Speaker preview stopped. Preview again before delivery." if cancel_event.is_set() else str(exc)
            raise ValueError(str(exc)) from None
        with self.delivery_lock:
            self.preview_cancel_event = None
            current = self.delivery_store.snapshot()
            if current.get("delivery_id") != delivery["delivery_id"]:
                self.delivery_state = "delivery_failed"
                self.delivery_message = "The approval changed during speaker preview. Approve it again."
                raise ValueError(self.delivery_message)
            self.delivery_previewed_id = delivery["delivery_id"]
            self.delivery_state = "message_ready"
            self.delivery_message = "Speaker preview finished. This checks playback only."
        return self.delivery_status()

    def start_delivery_mission(self, request):
        with self.action_lock:
            if self.manual_drive.status().get("enabled"):
                raise ValueError("Disable manual driving before starting message delivery.")
            try:
                target = self.store.load()
            except Exception as exc:
                raise ValueError("Stored profile error: {0}".format(exc))
            if (not target or request.get("profile_id") != target.get("profile_id")
                    or request.get("profile_revision") != target.get("revision")):
                raise ValueError("The selected profile changed. Select the person again before delivery.")
            with self.lock:
                if self.session is not None:
                    raise ValueError("Finish or cancel enrollment before starting message delivery.")
            delivery_id = request.get("delivery_id")
            with self.delivery_lock:
                if self.active_delivery_id is not None:
                    raise ValueError("A message delivery is already active.")
            delivery = self.delivery_store.get(delivery_id, target["profile_id"], target["revision"])
            with self.delivery_lock:
                previewed_id = self.delivery_previewed_id
            reason = delivery_preflight(delivery, delivery_id, target["profile_id"], target["revision"],
                                        previewed_id, self.speech_player.status()["available"])
            if reason:
                raise ValueError(reason)
            camera_ready, robot_ready, robot_moving = self.runtime_safety()
            if not robot_ready or robot_moving:
                raise ValueError("Fresh stopped EV3 feedback is required before delivery.")
            self._require_stationary_feedback()
            cancel_event = threading.Event()
            with self.delivery_lock:
                self.active_delivery_id = delivery_id
                self.active_delivery_cancel = cancel_event
                self.delivery_state = "searching"
                self.delivery_message = "Searching the prepared room for the selected person."
                self.delivery_info = {key: delivery[key] for key in (
                    "delivery_id", "profile_id", "profile_revision", "message_id", "message_revision", "text_sha256"
                )}
            try:
                result = self._start_find_mission_locked(request, target, camera_ready, robot_moving)
            except Exception:
                with self.delivery_lock:
                    self.active_delivery_id = None
                    self.active_delivery_cancel = None
                    self.delivery_state = "message_ready"
                raise
            return result

    def delivery_status(self):
        prepared = self.delivery_store.snapshot()
        with self.delivery_lock:
            state = self.delivery_state
            message = self.delivery_message
            info = dict(self.delivery_info or {})
            previewed = bool(prepared.get("delivery_id") and self.delivery_previewed_id == prepared.get("delivery_id"))
        if state == "searching" and self.mission.status().get("running"):
            progress = self.mission._load_report()
            if str(progress.get("message", "")).startswith("Approaching the person"):
                state = "approaching"
                message = "Approaching the selected person in a route-checked short segment."
        if prepared.get("state") == "preparing" and state not in {"searching", "approaching", "speaking"}:
            state = "preparing"
        elif prepared.get("state") == "ready" and state == "idle":
            state = "message_ready"
        if prepared.get("delivery_id"):
            info.update({key: prepared.get(key) for key in (
                "delivery_id", "profile_id", "profile_revision", "message_id", "message_revision", "text_sha256"
            )})
        return {"state": state, "message": message, "previewed": previewed,
                "prepared": prepared.get("state") == "ready", **info}

    def on_mission_finished(self):
        self.publish_emergency_stop()
        with self.delivery_lock:
            delivery_id = self.active_delivery_id
            cancel_event = self.active_delivery_cancel
        if not delivery_id:
            return
        self._set_delivery("approaching", "Search finished. Verifying current identity and stopped feedback before speech.")
        try:
            target = self.store.load()
            if not target:
                raise SpeechDeliveryError("The selected profile is no longer available.")
            delivery = self.delivery_store.get(delivery_id, target["profile_id"], target["revision"])
            if (delivery["profile_id"] != target["profile_id"]
                    or delivery["profile_revision"] != target["revision"]):
                raise SpeechDeliveryError("The approved message is bound to a different profile revision.")
            report = self.mission._load_report()
            first_track = None
            for sample in range(3):
                if cancel_event is not None and cancel_event.is_set():
                    raise SpeechDeliveryError("Delivery was stopped before speech began.")
                now = time.monotonic()
                with self.lock:
                    observation = None if self.target_observation is None else dict(self.target_observation)
                    observation_age = None if self.target_observation_time is None else now - self.target_observation_time
                    robot = None if self.robot_status is None else dict(self.robot_status)
                    robot_age = None if self.robot_status_time is None else now - self.robot_status_time
                    head = None if self.camera_head_status is None else dict(self.camera_head_status)
                    head_age = None if self.camera_head_status_time is None else now - self.camera_head_status_time
                reason = delivery_gate(report, target["profile_id"], target["revision"], observation,
                                       observation_age, robot, robot_age, head, head_age,
                                       mission_state=self.mission.status().get("state"))
                if reason:
                    raise SpeechDeliveryError(reason)
                track_id = observation.get("track_id")
                if first_track is None:
                    first_track = track_id
                elif first_track != track_id:
                    raise SpeechDeliveryError("The uniquely identified person track changed before speech.")
                if sample < 2:
                    time.sleep(0.2)
            current = self.store.load()
            with self.delivery_lock:
                if self.active_delivery_id != delivery_id or (cancel_event and cancel_event.is_set()):
                    raise SpeechDeliveryError("Delivery was stopped before speech began.")
                if (not current or current.get("profile_id") != delivery["profile_id"]
                        or current.get("revision") != delivery["profile_revision"]):
                    raise SpeechDeliveryError("The selected profile changed before speech began.")
                self.delivery_state = "speaking"
                self.delivery_message = "Speaking the approved message through the robot speaker."
            threading.Thread(target=self._play_delivery, args=(delivery_id, delivery["audio"], cancel_event),
                             name="echora-message-playback", daemon=True).start()
        except Exception as exc:
            if cancel_event is not None and cancel_event.is_set():
                self.delivery_store.consume(delivery_id)
                return
            self._fail_delivery(delivery_id, str(exc))

    def _play_delivery(self, delivery_id, audio, cancel_event):
        try:
            self.speech_player.play(audio, cancel_event=cancel_event)
            with self.delivery_lock:
                if self.active_delivery_id != delivery_id or (cancel_event and cancel_event.is_set()):
                    self.delivery_state = "stopped"
                    self.delivery_message = "Playback stopped before completion."
                else:
                    self.delivery_state = "played"
                    self.delivery_message = "Approved message playback completed. This does not confirm the person heard it."
                self.active_delivery_id = None
                self.active_delivery_cancel = None
            self.delivery_store.consume(delivery_id)
        except Exception as exc:
            if cancel_event is not None and cancel_event.is_set():
                self._stop_delivery_state("Playback stopped by the operator.")
            else:
                self._fail_delivery(delivery_id, str(exc))

    def _fail_delivery(self, delivery_id, reason):
        self.delivery_store.consume(delivery_id)
        with self.delivery_lock:
            if self.active_delivery_id == delivery_id:
                self.active_delivery_id = None
                self.active_delivery_cancel = None
            self.delivery_state = "delivery_failed"
            self.delivery_message = str(reason)[:240]

    def _stop_delivery_state(self, message):
        with self.delivery_lock:
            self.active_delivery_id = None
            self.active_delivery_cancel = None
            self.delivery_state = "stopped"
            self.delivery_message = message


    def on_odometry(self, message):
        orientation = message.pose.pose.orientation
        self.manual_drive.update_odometry(
            yaw_from_quaternion(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
        )

    def runtime_safety(self):
        now = time.monotonic()
        with self.lock:
            camera_ready = (
                self.latest_frame_time is not None
                and now - self.latest_frame_time < 2.0
            )
            status = None if self.robot_status is None else dict(self.robot_status)
            status_age = (
                None
                if self.robot_status_time is None
                else now - self.robot_status_time
            )
        robot_ready = bool(
            status is not None
            and status_age is not None
            and status_age <= 1.5
            and status.get("status") == "ok"
        )
        robot_moving = bool(status and status.get("motion_active"))
        return camera_ready, robot_ready, robot_moving

    def on_manual_drive_watchdog(self):
        with self.action_lock:
            camera_ready, robot_ready, _robot_moving = self.runtime_safety()
            if self.manual_drive.watchdog(
                camera_ready,
                robot_ready,
                self.mission.status().get("running", False),
            ):
                self.publish_chassis_stop()

    def on_image(self, message):
        if message.encoding not in ("bgr8", "rgb8") or len(message.data) != message.height * message.width * 3:
            return
        image = numpy.frombuffer(message.data, numpy.uint8).reshape(message.height, message.width, 3)
        if message.encoding == "rgb8":
            image = image[:, :, ::-1]
        now = time.monotonic()
        key=(message.header.frame_id,message.header.stamp.sec,message.header.stamp.nanosec)
        with self.lock:
            pose = capture_pose(self.camera_head_status, self.robot_status,
                None if self.camera_head_status_time is None else now-self.camera_head_status_time,
                None if self.robot_status_time is None else now-self.robot_status_time)
            self.raw_frames[key]=(image.copy(),now,pose)
            while len(self.raw_frames)>24:self.raw_frames.popitem(last=False)
        if now - self.last_preview_encode_time >= 0.15:
            with self.lock:
                session = self.session
            self.store_preview(
                image,
                "Enrollment: {0}".format(session.guidance if session and session.camera_source == "robot" else "ready"),
            )
        pair = self.matcher.offer_left(stamp_key(message.header.stamp), Frame(image, message.header.stamp, message.header.frame_id))
        if pair is not None:
            self.process_pair(*pair)

    def on_observations(self, message):
        try:
            observations = parse_face_observations(message.data)
        except FaceObservationError:
            return
        pair = self.matcher.offer_right(observations["key"], observations)
        if pair is not None:
            self.process_pair(*pair)

    def process_pair(self, frame, observations):
        if frame.frame_id != observations["frame_id"]:
            return
        faces = observations["faces"]
        canvas = frame.image.copy()
        with self.lock:
            session = self.session
        for face in faces:
            x1, y1, x2, y2 = [int(round(v)) for v in face["box"]]
            cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 205, 255), 2)
            for point in face["landmarks"]:
                cv2.circle(canvas, tuple(int(round(v)) for v in point), 2, (60, 230, 130), -1)
        if session is not None and session.camera_source == "robot" and time.monotonic() - session.last_capture >= 0.65:
            if len(faces) != 1:
                session.message = "Keep only the target visible." if faces else "No clear face—move into view."
            else:
                accepted, reason, quality = assess_quality(frame.image, faces[0])
                if not accepted:
                    session.message = reason.capitalize() + "."
                else:
                    with self.runtime_lock:
                        embedding, aligned = self.recognizer.embedding(frame.image, faces[0]["landmarks"])
                    with self.lock:
                        if self.session is session:
                            session.add(embedding, aligned, classify_pose(faces[0]["landmarks"]), "live", quality)
        banner = "Enrollment: {0}".format(session.guidance if session and session.camera_source == "robot" else "ready")
        self.store_preview(canvas, banner)

    def store_preview(self, canvas, banner):
        canvas = canvas.copy()
        cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 30), (8, 15, 22), -1)
        cv2.putText(canvas, banner, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        ok, encoded = cv2.imencode(".jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, 78])
        if ok:
            with self.lock:
                self.latest_jpeg = encoded.tobytes()
                self.latest_frame_time = time.monotonic()
                self.last_preview_encode_time = self.latest_frame_time

    def on_family_status(self, message):
        try:
            value=json.loads(message.data)
            if isinstance(value,dict):
                with self.lock:self.family_status=value
        except ValueError: pass

    def status(self):
        try:
            target = self.store.load()
        except Exception as exc:
            target = None
            self.last_message = "Stored target error: {0}".format(exc)
        with self.lock:
            session = self.session
            camera_ready = self.latest_frame_time is not None and time.monotonic() - self.latest_frame_time < 2.0
            head_age = None if self.camera_head_status_time is None else time.monotonic() - self.camera_head_status_time
            head_status = None if self.camera_head_status is None else dict(self.camera_head_status)
            if head_status is not None:
                head_status["available"] = head_age <= 2.0
            result = {
                "camera_ready": camera_ready,
                "enrolling": session is not None,
                "enrollment_session_id": None if session is None else session.session_id,
                "enrollment_source": None if session is None else session.camera_source,
                "enrollment_label": None if session is None else session.label,
                "enrollment_profile_id": None if session is None else session.profile_id,
                "target_label": None if target is None else target["label"],
                "target_sample_count": 0 if target is None else len(target["samples"]),
                "profile_id": None if target is None else target["profile_id"],
                "profile_revision": None if target is None else target["revision"],
                "family_profiles": self.store.family.profiles(),
                "family_tracking": self.family_status,
                "sample_count": 0 if session is None else len(session.samples),
                "minimum_total": MINIMUM_TOTAL,
                "pose_counts": {pose: 0 for pose in REQUIRED} if session is None else session.pose_counts,
                "ready": False if session is None else session.ready,
                "guidance": (session.guidance if session is not None and
                             (session.camera_source == "phone" or camera_ready) else
                             "Wait for robot camera" if session is not None else "Choose a camera to get started"),
                "message": self.last_message if session is None else session.message,
                "model": self.args.model_name,
                "provider": self.recognizer.provider,
                "privacy": "No full frames stored; aligned crops retained only if selected.",
                "manual_camera_control_available": manual_camera_control_available(),
                "camera_head": {"available": False} if head_status is None else head_status,
                "mission": self.mission.status(),
            }
        result["manual_drive"] = self.manual_drive.status()
        result["robot_ready"] = self.runtime_safety()[1]
        result["delivery"] = self.delivery_status()
        result["speech_output"] = self.speech_player.status()
        return result

    def publish_chassis_velocity(self, linear=0.0, angular=0.0, repeats=1):
        if not rclpy.ok():
            return
        velocity = Twist()
        velocity.linear.x = float(linear)
        velocity.angular.z = float(angular)
        try:
            for _index in range(max(1, int(repeats))):
                self.cmd_vel_publisher.publish(velocity)
        except Exception:
            return

    def publish_chassis_stop(self, repeats=1):
        self.publish_chassis_velocity(0.0, 0.0, repeats=repeats)

    def publish_emergency_stop(self):
        """Send independent chassis and camera-head stops."""

        with self.action_lock:
            self._publish_emergency_stop_locked()

    def _publish_emergency_stop_locked(self):
        if not rclpy.ok():
            return
        command = String()
        command.data = '{"action":"stop"}'
        self.publish_chassis_stop(repeats=3)
        try:
            for _index in range(3):
                self.camera_head_command_publisher.publish(command)
        except Exception:
            return

    def enable_manual_drive(self, request):
        with self.action_lock:
            if hasattr(self, "delivery_lock"):
                self._require_delivery_unlocked()
            with self.lock:
                enrolling = self.session is not None
            camera_ready, robot_ready, robot_moving = self.runtime_safety()
            return self.manual_drive.enable(
                request.get("cable_zero_confirmed") is True,
                camera_ready,
                robot_ready,
                robot_moving,
                self.mission.status().get("running", False),
                enrollment_running=enrolling,
            )

    def command_manual_drive(self, request):
        with self.action_lock:
            return self._command_manual_drive_locked(request)

    def _command_manual_drive_locked(self, request):
        camera_ready, robot_ready, _robot_moving = self.runtime_safety()
        try:
            result = self.manual_drive.command(
                request.get("direction"),
                request.get("control_token"),
                request.get("sequence"),
                camera_ready,
                robot_ready,
                self.mission.status().get("running", False),
            )
        except ManualDriveError as exc:
            self.manual_drive.stop(message=str(exc))
            self.publish_chassis_stop(repeats=2)
            raise ValueError(str(exc))
        self.publish_chassis_velocity(result["linear"], result["angular"])
        return result

    def disable_manual_drive(self):
        with self.action_lock:
            return self._disable_manual_drive_locked()

    def _disable_manual_drive_locked(self):
        result = self.manual_drive.stop(disable=True)
        self.publish_chassis_stop(repeats=3)
        return result

    def start_find_mission(self, request):
        with self.action_lock:
            if hasattr(self, "delivery_lock"):
                self._require_delivery_unlocked()
            try:
                target = self.store.load()
            except Exception as exc:
                raise ValueError("Stored target error: {0}".format(exc))
            camera_ready, robot_ready, robot_moving = self.runtime_safety()
            if not robot_ready:
                raise ValueError("Fresh EV3 status is required before starting the mission.")
            if robot_moving:
                raise ValueError("Wait for the robot to stop before starting the mission.")
            return self._start_find_mission_locked(request, target, camera_ready, robot_moving)

    def _start_find_mission_locked(self, request, target, camera_ready=None, robot_moving=None):
        if self.manual_drive.status().get("enabled"):
            raise ValueError("Disable manual driving before starting the mission.")
        try:
            target = self.store.load()
        except Exception as exc:
            raise ValueError("Stored target error: {0}".format(exc))
        validate_mission_profile(request, target)
        current_camera_ready, robot_ready, current_robot_moving = self.runtime_safety()
        camera_ready = current_camera_ready if camera_ready is None else camera_ready
        robot_moving = current_robot_moving if robot_moving is None else robot_moving
        if not robot_ready:
            raise ValueError("Fresh EV3 status is required before starting the mission.")
        if robot_moving:
            raise ValueError("Wait for the robot to stop before starting the mission.")
        with self.lock:
            if self.session is not None:
                raise ValueError("Finish or cancel enrollment before searching.")
            head_age = (
                None
                if self.camera_head_status_time is None
                else time.monotonic() - self.camera_head_status_time
            )
            head_status = (
                None
                if self.camera_head_status is None
                else dict(self.camera_head_status)
            )
            if head_status is not None:
                head_status["available"] = head_age <= 2.0
        return self.mission.start(
            request.get("cable_zero_confirmed") is True,
            None if target is None else target.get("label"),
            camera_ready,
            head_status,
            initial_cable_heading_degrees=request.get("initial_cable_heading_degrees", 0.0),
            profile_id=None if target is None else target.get('profile_id'),
            profile_revision=None if target is None else target.get('revision'),
        )

    def stop_find_mission(self):
        with self.action_lock:
            if not hasattr(self, "delivery_lock"):
                return self._stop_find_mission_locked()
            with self.delivery_lock:
                active = self.active_delivery_id
                cancel_event = self.active_delivery_cancel
                state = self.delivery_state
                preview_cancel_event = self.preview_cancel_event
                if cancel_event is not None:
                    cancel_event.set()
                if preview_cancel_event is not None:
                    preview_cancel_event.set()
                self.active_delivery_id = None
                self.active_delivery_cancel = None
            self.speech_player.stop()
            result = self._stop_find_mission_locked()
            if active:
                self.delivery_store.consume(active)
                self._stop_delivery_state("Message delivery stopped by the operator.")
            elif state == "previewing":
                with self.delivery_lock:
                    self.delivery_previewed_id = None
                    self.delivery_state = "message_ready" if self.delivery_store.snapshot().get("state") == "ready" else "stopped"
                    self.delivery_message = "Speaker preview stopped. Preview again before delivery."
            return result

    def _stop_find_mission_locked(self):
        result = self.mission.stop()
        self.manual_drive.stop(disable=True)
        self._publish_emergency_stop_locked()
        return result

    def jog_camera(self, request):
        with self.action_lock:
            return self._jog_camera_locked(request)

    def _jog_camera_locked(self, request):
        if hasattr(self, "delivery_lock"):
            self._require_delivery_unlocked()
        degrees = jog_degrees(request.get("direction"), request.get("magnitude"))
        # A human button takes control away from mission/drive commands.
        self.mission.stop()
        self.manual_drive.stop(disable=True)
        self.publish_chassis_stop(repeats=3)
        try:
            with camera_control_lease(exclusive=False):
                with self.lock:
                    status = None if self.camera_head_status is None else dict(self.camera_head_status)
                    age = None if self.camera_head_status_time is None else time.monotonic() - self.camera_head_status_time
                target = validate_camera_jog(status, age, degrees)
                message = String()
                message.data = json.dumps(dict(action="manual_jog", target=target,
                    expected_reference_id=status.get("reference_id"), expected_position=status["position"]))
                self.camera_head_command_publisher.publish(message)
                with self.lock:
                    self.last_message = "Manual camera movement requested."
        except CameraControlBusy:
            raise ValueError("Automatic camera control is stopping; press again in a moment.")
        return self.status()

    def save_camera_limit(self, request):
        if hasattr(self, "delivery_lock"):
            self._require_delivery_unlocked()
        kind = request.get('limit')
        restore = kind == 'restore_saved_range_from_lower'
        if restore and request.get('operator_confirmed') is not True:
            raise ValueError('Confirm that the camera is back at the same saved lower view.')
        if kind not in ('lower', 'upper') and not restore:
            raise ValueError('Choose lower or upper limit.')
        with self.action_lock:
            self.mission.stop()
            self.manual_drive.stop(disable=True)
            self.publish_chassis_stop(repeats=3)
            try:
                with camera_control_lease(exclusive=False):
                    with self.lock:
                        head = dict(self.camera_head_status or {})
                        age = None if self.camera_head_status_time is None else time.monotonic() - self.camera_head_status_time
                    camera_ready, robot_ready, robot_moving = self.runtime_safety()
                    if not camera_ready or not robot_ready or age is None or not 0 <= age <= 2:
                        raise ValueError('Wait for the live camera and EV3 connection before saving.')
                    if head.get('moving') or head.get('homing') or robot_moving:
                        raise ValueError('Wait for the camera and tracks to stop before saving.')
                    if restore:
                        limits = head.get('saved_limits') or {}
                        lower, upper = limits.get('lower'), limits.get('upper')
                        if (type(lower) is not int or type(upper) is not int
                                or lower <= upper or not limits.get('reference_id')):
                            raise ValueError('Save both camera limits before reusing a range.')
                        if limits.get('reference_id') == head.get('reference_id'):
                            raise ValueError('The saved range already uses this camera reference.')
                    request_id = uuid.uuid4().hex
                    message = String()
                    command = dict(action='restore_saved_range_from_lower' if restore else 'save_manual_limit',
                        expected_reference_id=head.get('reference_id'), expected_position=head['position'],
                        request_id=request_id)
                    if restore:
                        command['operator_confirmed'] = True
                    else:
                        command['limit'] = kind
                    message.data = json.dumps(command)
                    self.camera_head_command_publisher.publish(message)
            except CameraControlBusy:
                raise ValueError('Automatic camera control is stopping; try saving again in a moment.')
        # The ROS watchdog also takes action_lock. Release it before waiting,
        # otherwise that callback blocks the single-threaded ROS executor and
        # prevents the head-status acknowledgement from being delivered.
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            with self.lock:
                result = (self.camera_head_status or {}).get('limit_save_result') or {}
            if result.get('request_id') == request_id:
                if not result.get('ok'):
                    raise ValueError(result.get('error', 'Could not save the limit.'))
                with self.lock:
                    self.last_message = ('Saved camera range restored from this lower view.' if restore
                                         else ('Lower' if kind == 'lower' else 'Upper') + ' camera limit saved.')
                return self.status()
            time.sleep(.05)
        raise ValueError('Save confirmation has not arrived. Check the limit status before trying again.')

    def restore_saved_camera_range(self, request):
        return self.save_camera_limit(dict(request, limit='restore_saved_range_from_lower'))

    def start(self, request):
        if hasattr(self, "delivery_lock"):
            self._require_delivery_unlocked()
        label = str(request.get("label", "")).strip()
        retention = request.get("retention", "embeddings")
        camera_source = request.get("camera_source", "robot")
        if not request.get("consent"):
            raise ValueError("Confirm that the target person has given permission.")
        if not label or len(label) > 40:
            raise ValueError("Enter a target label between 1 and 40 characters.")
        if retention not in ("embeddings", "face_crops"):
            raise ValueError("Invalid photo-storage choice.")
        if camera_source not in ("phone", "robot"):
            raise ValueError("Choose the phone camera or robot camera.")
        with self.action_lock:
            if self.mission.status().get("running"):
                raise ValueError("Stop the find-person mission before enrolling.")
            if self.manual_drive.status().get("enabled"):
                raise ValueError("Disable manual driving before enrolling.")
            with self.lock:
                if self.session is not None:
                    raise ValueError("Finish or cancel the current enrollment first.")
                profile_id = request.get("profile_id")
                if profile_id and not self.store.family.load(profile_id):
                    raise ValueError("Unknown family member")
                self.session = EnrollmentSession(label, retention, camera_source)
                self.session.profile_id = profile_id
                if hasattr(self, "delivery_store"):
                    self._clear_delivery()
        return self.status()

    def enrollment_session(self, session_id=None, source=None):
        """Called under self.lock; late frames/actions cannot change another session."""
        session = self.session
        if session is None:
            raise ValueError("No enrollment is in progress.")
        if ((session_id is not None or session.camera_source == "phone")
                and session_id != session.session_id):
            raise ValueError("This enrollment session has changed. Reopen Profiles to continue.")
        if source is not None and source != session.camera_source:
            raise ValueError("This session is using the robot camera.")
        return session

    def cancel(self, session_id=None):
        if hasattr(self, "delivery_lock"):
            self._require_delivery_unlocked()
        with self.action_lock, self.lock:
            self.enrollment_session(session_id)
            self.session = None
            self.last_message = "Enrollment cancelled; nothing was saved."
        return self.status()

    def upload(self, data, session_id=None, source="upload"):
        if not data or len(data) > MAX_UPLOAD_BYTES:
            raise ValueError("Each photo must be smaller than 10 MB.")
        with self.lock:
            session = self.enrollment_session(session_id, "phone" if source == "phone" else None)
        image = cv2.imdecode(numpy.frombuffer(data, numpy.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("The uploaded file is not a readable image.")
        if image.shape[0] * image.shape[1] > 20_000_000:
            raise ValueError("Resize the photo to at most 20 megapixels.")
        with self.runtime_lock:
            candidates = self.upload_detector.infer(image)
        faces = select_faces(
            [FaceDetection(x1, y1, x2, y2, score, landmarks) for x1, y1, x2, y2, score, landmarks in candidates],
            0.30,
            10,
        )
        if not faces:
            raise ValueError("No clear face was found in that photo.")
        if len(faces) != 1:
            raise ValueError("Use a photo containing exactly one clear face.")
        detected = faces[0]
        face = {
            "box": (detected.x1, detected.y1, detected.x2, detected.y2),
            "landmarks": detected.landmarks,
            "score": detected.score,
        }
        accepted, reason, quality = assess_quality(image, face, minimum_face_pixels=80, minimum_blur=35.0)
        if not accepted:
            raise ValueError("Photo rejected: {0}.".format(reason))
        with self.runtime_lock:
            embedding, aligned = self.recognizer.embedding(image, face["landmarks"])
        with self.lock:
            if self.session is not session:
                raise ValueError("Enrollment was cancelled while the photo was processing.")
            added = session.add(embedding, aligned, classify_pose(face["landmarks"]), source, quality)
            return {"ok": True, "added": added, "message": session.message,
                    "ready": session.ready, "guidance": session.guidance,
                    "sample_count": len(session.samples), "pose_counts": session.pose_counts}

    def finish(self, session_id=None):
        if hasattr(self, "delivery_lock"):
            self._require_delivery_unlocked()
        with self.action_lock, self.lock:
            session = self.enrollment_session(session_id)
            if not session.ready:
                raise ValueError("Collect at least 10 good views, including front, left, and right angles.")
            data_root = os.path.dirname(self.args.store_path)
            photo_directory = None
            if session.retention == "face_crops":
                photo_directory = PHOTO_DIRECTORY_PREFIX + "{0:x}".format(time.time_ns())
                photo_root = os.path.join(data_root, photo_directory)
                os.makedirs(photo_root, mode=0o700)
                os.chmod(photo_root, 0o700)
                for index, (sample, crop) in enumerate(zip(session.samples, session.crops), 1):
                    name = "face_{0:02d}.jpg".format(index)
                    path = os.path.join(photo_root, name)
                    if not cv2.imwrite(path, crop, [cv2.IMWRITE_JPEG_QUALITY, 94]):
                        raise RuntimeError("Could not retain an aligned face crop.")
                    os.chmod(path, 0o600)
                    sample["photo"] = photo_directory + "/" + name
            self.store.save(session.label, session.samples, session.retention, session.profile_id)
            if hasattr(self, "delivery_store"):
                self._clear_delivery()
            self.cleanup_photo_directories(keep=photo_directory)
            self.session = None
            self.last_message = "Target enrolled successfully with {0} diverse views.".format(len(session.samples))
        return self.status()

    def delete_target(self):
        if hasattr(self, "delivery_lock"):
            self._require_delivery_unlocked()
        with self.action_lock:
            if self.mission.status().get("running"):
                raise ValueError("Stop the find-person mission before deleting its target.")
            if self.manual_drive.status().get("enabled"):
                raise ValueError("Disable manual driving before deleting its target.")
            with self.lock:
                if self.session is not None:
                    raise ValueError("Finish or cancel enrollment before deleting a profile.")
            self.store.delete()
            if hasattr(self, "delivery_store"):
                self._clear_delivery()
            self.cleanup_photo_directories()
            with self.lock:
                self.session = None
                self.last_message = "Enrolled target and retained face crops deleted."
        return self.status()

    def cleanup_photo_directories(self, keep=None):
        # Each profile owns its face crops; never delete another profile's files.
        retained = set()
        for profile in self.store.family.profiles():
            for sample in self.store.family.load(profile['id'])['samples']:
                if sample.get('photo'): retained.add(sample['photo'].split('/')[0])
        legacy = self.store.legacy.load()
        for sample in (legacy or {}).get('samples', []):
            if sample.get('photo'): retained.add(sample['photo'].split('/')[0])
        if keep: retained.add(keep)
        root = os.path.abspath(os.path.dirname(self.args.store_path))
        for name in os.listdir(root):
            path = os.path.join(root, name)
            if name.startswith(PHOTO_DIRECTORY_PREFIX) and name not in retained and os.path.isdir(path):
                shutil.rmtree(path)

    def family_action(self, request):
        if hasattr(self, "delivery_lock"):
            self._require_delivery_unlocked()
        with self.action_lock:
            if self.mission.status().get('running') or self.session is not None:
                raise ValueError('Finish the active mission or enrollment first.')
            family = self.store.family
            action, pid = request.get('action'), request.get('profile_id')
            if action == 'select': family.select(pid)
            elif action == 'rename': family.rename(pid, request.get('label', ''))
            elif action == 'delete':
                family.delete(pid)
                self.cleanup_photo_directories()
            elif action == 'forget': family.forget(pid, request.get('outfit_id'))
            else: raise ValueError('Unknown family action')
            if hasattr(self, "delivery_store"):
                self._clear_delivery()
        return self.status()

    def family_range_frame(self):
        """Return an in-memory raw frame paired with this person's exact box."""
        with self.lock:
            now = time.monotonic()
            track=dict(self.family_status)
            cached=self.raw_frames.get(tuple(track.get('frame_key',[])))
            pose = capture_pose(self.camera_head_status, self.robot_status,
                None if self.camera_head_status_time is None else now-self.camera_head_status_time,
                None if self.robot_status_time is None else now-self.robot_status_time)
            if (not cached or not 0 <= now-cached[1] <= 1.2 or not track.get('body_box')
                    or not track.get('identity_confirmed') or not track.get('profile_id')
                    or not track.get('track_id') or not same_capture_pose(cached[2], pose)):
                raise ValueError('A fresh stationary view of this person is needed.')
            image=cached[0];pose=cached[2];head=pose['head']
        ok,jpeg=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,90])
        if not ok:raise ValueError('Could not capture the measurement frame.')
        import yaml
        try:
            with open(self.args.camera_intrinsics_file) as handle:
                camera_config = yaml.safe_load(handle)
            matrix = camera_config['camera_matrix']['data']
            camera = dict(image_size=[camera_config['image_width'], camera_config['image_height']],
                intrinsics=dict(fx=matrix[0], fy=matrix[4], cx=matrix[2], cy=matrix[5]),
                distortion=camera_config.get('distortion_coefficients', {}).get('data', []))
        except (OSError, ValueError, KeyError, IndexError, TypeError, yaml.YAMLError) as exc:
            raise ValueError('Camera calibration is unavailable for the range comparison') from exc
        return dict(image=base64.b64encode(jpeg).decode(),track=track,head=head,
                    capture_schema=1,pose_binding=pose,camera=camera,
                    image_sha256=hashlib.sha256(jpeg).hexdigest())

    def close(self):
        self.mission.close()
        self.manual_drive.stop(disable=True)
        self.publish_emergency_stop()
        self.recognizer.close()
        self.upload_detector.close()


class Handler(BaseHTTPRequestHandler):
    node = None
    def log_message(self, *_args):
        pass
    def json_response(self, payload, status=200):
        body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)
    def do_GET(self):
        if self.path in ("/phone-setup", "/phone-ca.crt"):
            if self.path == "/phone-ca.crt":
                try:
                    with open(self.node.args.https_ca_path, "rb") as handle:
                        body = handle.read()
                except OSError:
                    self.json_response({"error": "Phone HTTPS setup has not been installed on this Jetson yet."}, 503)
                    return
                content_type = "application/x-x509-ca-cert"
            else:
                body, content_type = PHONE_SETUP_PAGE, "text/html; charset=utf-8"
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            if self.path == "/phone-ca.crt":
                self.send_header("Content-Disposition", 'attachment; filename="echora-local-ca.crt"')
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == '/api/family/range-frame':
            try:self.json_response(self.node.family_range_frame())
            except ValueError as exc:self.json_response({'error':str(exc)},503)
            return
        if self.path == "/api/family/wardrobe":
            self.json_response({"outfits": self.node.store.family.wardrobe(self.node.store.family.selected_id(), images=True)}); return
        if self.path == "/api/status":
            self.json_response(self.node.status()); return
        if self.path == "/snapshot.jpg":
            with self.node.lock:
                jpeg = self.node.latest_jpeg
            if not jpeg:
                self.json_response({"error": "No current camera frame."}, 503); return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(jpeg)))
            self.end_headers(); self.wfile.write(jpeg); return
        if self.path.startswith("/stream"):
            self.stream(); return
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(PAGE)))
        self.end_headers(); self.wfile.write(PAGE)
    def do_POST(self):
        if self.headers.get("X-Echora-Action") != "1":
            self.json_response({"error": "Missing action header."}, 403); return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length < 0:
                raise ValueError("Invalid request size.")
            if self.path in ("/api/enrollment/upload", "/api/enrollment/frame"):
                if length > MAX_UPLOAD_BYTES:
                    raise ValueError("Each photo must be smaller than 10 MB.")
                phone = self.path == "/api/enrollment/frame"
                token = self.headers.get("X-Echora-Enrollment")
                if phone and not token:
                    raise ValueError("A phone enrollment session is required.")
                result = self.node.upload(self.rfile.read(length), token, "phone" if phone else "upload")
            elif self.path == "/api/voice/transcribe":
                if length <= 0 or length > MAX_VOICE_UPLOAD_BYTES:
                    raise ValueError("Use a nonempty recording smaller than 8 MB.")
                content_type = self.headers.get("Content-Type", "application/octet-stream").split(";", 1)[0]
                result = self.node.transcribe_delivery(self.rfile.read(length), content_type)
            else:
                if length > 65536:
                    raise ValueError("Request is too large.")
                raw = self.rfile.read(length)
                request = json.loads(raw.decode("utf-8") or "{}")
                actions = {
                    "/api/enrollment/start": lambda: self.node.start(request),
                    "/api/enrollment/cancel": lambda: self.node.cancel(request.get("session_id")),
                    "/api/enrollment/finish": lambda: self.node.finish(request.get("session_id")),
                    "/api/family/action": lambda: self.node.family_action(request),
                    "/api/enrollment/delete": self.node.delete_target,
                    "/api/camera/jog": lambda: self.node.jog_camera(request),
                    "/api/camera/limit": lambda: self.node.save_camera_limit(request),
                    "/api/camera/restore-range": lambda: self.node.restore_saved_camera_range(request),
                    "/api/mission/start": lambda: self.node.start_find_mission(request),
                    "/api/mission/deliver": lambda: self.node.start_delivery_mission(request),
                    "/api/mission/stop": self.node.stop_find_mission,
                    "/api/delivery/prepare": lambda: self.node.prepare_delivery(request),
                    "/api/delivery/clear": lambda: self.node.clear_delivery(request),
                    "/api/delivery/preview": lambda: self.node.preview_delivery(request),
                    "/api/drive/enable": lambda: self.node.enable_manual_drive(request),
                    "/api/drive/command": lambda: self.node.command_manual_drive(request),
                    "/api/drive/disable": self.node.disable_manual_drive,
                }
                if self.path not in actions:
                    self.json_response({"error": "Not found."}, 404); return
                result = actions[self.path]()
            self.json_response(result)
        except (ValueError, RuntimeError) as exc:
            self.json_response({"error": str(exc)}, 400)
    def stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            while True:
                with self.node.lock:
                    jpeg = self.node.latest_jpeg
                if jpeg:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write("Content-Length: {0}\r\n\r\n".format(len(jpeg)).encode("ascii"))
                    self.wfile.write(jpeg + b"\r\n")
                time.sleep(0.12)
        except (BrokenPipeError, ConnectionResetError):
            pass


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--image-topic", default="/camera/image_raw")
    parser.add_argument("--observations-topic", default="/perception/face_observations")
    parser.add_argument("--camera-head-status-topic", default="/camera_head/status")
    parser.add_argument("--camera-head-command-topic", default="/camera_head/command")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--robot-status-topic", default="/robot_status")
    parser.add_argument("--target-observation-topic", default="/mission/target_observation")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--model-name", default="antelopev2_glintr100")
    parser.add_argument("--model-path", required=True)
    parser.add_argument("--model-sha256", required=True)
    parser.add_argument("--input-mean", type=float, default=127.5)
    parser.add_argument("--input-std", type=float, default=127.5)
    parser.add_argument("--yunet-engine-path", required=True)
    parser.add_argument("--store-path", required=True)
    parser.add_argument("--https-ca-path", default="/home/animesh/echora/phone_https/echora-local-ca.crt")
    parser.add_argument(
        "--mission-script", default="/home/animesh/echora/autonomous_find.py"
    )
    parser.add_argument(
        "--mission-report",
        default="/home/animesh/echora/logs/ui-find-mission.json",
    )
    parser.add_argument("--route-url", default="http://127.0.0.1:18091/route")
    parser.add_argument("--speech-url", default="http://127.0.0.1:18766")
    parser.add_argument("--speech-device", default="default")
    parser.add_argument('--camera-intrinsics-file', default='/home/animesh/echora/camera_calibration.yaml')
    parser.add_argument("--mission-cable-limit-degrees", type=float, default=90.0)
    parser.add_argument("--mission-cable-margin-degrees", type=float, default=10.0)
    parser.add_argument("--manual-linear-speed", type=float, default=0.03)
    parser.add_argument("--manual-reverse-speed", type=float, default=0.02)
    parser.add_argument("--manual-angular-speed", type=float, default=0.25)
    parser.add_argument("--manual-turn-buffer-degrees", type=float, default=3.0)
    parser.add_argument(
        "--manual-command-timeout-seconds", type=float, default=0.25
    )
    parser.add_argument(
        "--manual-session-timeout-seconds", type=float, default=120.0
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    rclpy.init()
    node = EnrollmentNode(args)
    Handler.node = node
    server = ThreadingHTTPServer((args.bind, args.port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print("enrollment console ready on http://{0}:{1}/".format(args.bind, args.port), flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        server.shutdown(); server.server_close(); node.close(); node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
