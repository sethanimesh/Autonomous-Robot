#!/usr/bin/env python3
"""Run an offline BoT-SORT baseline on an Echora mission recording.

Uses the recording's person detections; never starts ROS or connects to EV3.
The optional LAP adapter uses Ultralytics' own NumPy assignment solver when the
compiled ``lap`` package is absent from the local Python environment.
"""

import argparse
import hashlib
import json
from pathlib import Path
import sys
import types

import cv2
import numpy as np


def provide_lap_adapter():
    try:
        import lap  # noqa: F401
        return "lap"
    except ImportError:
        from ultralytics.utils.ops import linear_sum_assignment

        adapter = types.ModuleType("lap")
        adapter.__version__ = "local-ultralytics-numpy-adapter"

        def lapjv(cost_matrix, extend_cost=True, cost_limit=np.inf):
            costs = np.asarray(cost_matrix, dtype=np.float64)
            rows, cols = costs.shape
            x = np.full(rows, -1, dtype=np.int64)
            y = np.full(cols, -1, dtype=np.int64)
            if not rows or not cols:
                return 0.0, x, y
            if extend_cost and np.isfinite(cost_limit):
                # Private dummy columns represent unmatched tracks. Assignments
                # above the gate are forbidden, as in lapjv(cost_limit=...).
                valid = np.where(np.isfinite(costs) & (costs <= cost_limit), costs, np.inf)
                dummy_cost = max(0.0, float(cost_limit)) + 1e-9
                extended = np.full((rows, cols + rows), np.inf)
                extended[:, :cols] = valid
                for index in range(rows):
                    extended[index, cols + index] = dummy_cost
                matched_rows, matched_cols = linear_sum_assignment(extended)
                for row, col in zip(matched_rows, matched_cols):
                    if col < cols:
                        x[row] = col
                        y[col] = row
            else:
                matched_rows, matched_cols = linear_sum_assignment(costs)
                for row, col in zip(matched_rows, matched_cols):
                    if costs[row, col] <= cost_limit:
                        x[row] = col
                        y[col] = row
            total = float(sum(costs[row, col] for row, col in enumerate(x) if col >= 0))
            return total, x, y

        adapter.lapjv = lapjv
        sys.modules["lap"] = adapter
        return adapter.__version__


class Detections:
    def __init__(self, boxes):
        values = np.asarray(boxes, dtype=np.float32).reshape((-1, 5))
        self.xywh = values[:, :4]
        self.conf = values[:, 4]
        self.cls = np.zeros(len(values), dtype=np.float32)

    @property
    def xyxy(self):
        half = self.xywh[:, 2:4] / 2
        return np.column_stack((self.xywh[:, :2] - half, self.xywh[:, :2] + half))

    def __len__(self):
        return len(self.conf)

    def __getitem__(self, key):
        values = np.column_stack((self.xywh, self.conf))[key]
        return Detections(np.asarray(values).reshape((-1, 5)))


def person_detections(message):
    detections = []
    for item in message.get("detections", []):
        if not item.get("results"):
            continue
        top = item["results"][0].get("hypothesis", {})
        if top.get("class_id") != "person":
            continue
        bbox = item["bbox"]
        position = bbox["center"].get("position", bbox["center"])
        detections.append([position["x"], position["y"], bbox["size_x"], bbox["size_y"], top["score"]])
    return detections


def box_iou(a, b):
    left, top = max(a[0], b[0]), max(a[1], b[1])
    right, bottom = min(a[2], b[2]), min(a[3], b[3])
    area = max(0, right - left) * max(0, bottom - top)
    area_a = max(0, a[2] - a[0]) * max(0, a[3] - a[1])
    area_b = max(0, b[2] - b[0]) * max(0, b[3] - b[1])
    denominator = area_a + area_b - area
    return area / denominator if denominator else 0.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--reference-report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root))
    from scripts.diagnostics.replay_mission import frame_key, load_recording

    recording_root, manifest, events = load_recording(args.recording)
    reference = json.loads(args.reference_report.read_text())
    eligible = {tuple(row["frame_key"]) for row in reference["observations"]}
    anchors = {}
    detections = {}
    for event in events:
        data = event["data"]
        if event["topic"] == "/perception/person_detections":
            detections[frame_key(data)] = data
        if event["topic"] in ("/perception/people_tracks", "/mission/target_observation"):
            if data.get("identity_confirmed") and data.get("frame_key") and data.get("body_box"):
                anchors[tuple(data["frame_key"])] = data

    solver = provide_lap_adapter()
    from ultralytics import __version__ as ultralytics_version
    from ultralytics.trackers.bot_sort import BOTSORT
    import ultralytics.trackers.bot_sort as botsort_module

    settings = dict(
        tracker="Ultralytics BoT-SORT", ultralytics_version=ultralytics_version,
        botsort_source_sha256=hashlib.sha256(Path(botsort_module.__file__).read_bytes()).hexdigest(),
        assignment_solver=solver, track_high_thresh=0.25, track_low_thresh=0.1,
        new_track_thresh=0.25, track_buffer=30, match_thresh=0.8,
        fuse_score=True, gmc_method="sparseOptFlow", proximity_thresh=0.5,
        appearance_thresh=0.8, with_reid=False, model="auto",
        appearance_model_weights="none (ReID disabled)",
        input="recorded person detections matched by exact image timestamp",
        identity_anchor="first confirmed recorded body association overlapping a tracker box",
        reference_report=str(args.reference_report),
        reference_report_sha256=hashlib.sha256(args.reference_report.read_bytes()).hexdigest(),
        recording_manifest_sha256=hashlib.sha256((recording_root / "manifest.json").read_bytes()).hexdigest(),
        recording_events_sha256=hashlib.sha256((recording_root / "events.jsonl").read_bytes()).hexdigest(),
    )
    (args.output / "settings.json").write_text(json.dumps(settings, indent=2) + "\n")
    tracker = BOTSORT(types.SimpleNamespace(**settings))
    frame_events = [event for event in events if event["kind"] == "frame"]
    video = cv2.VideoWriter(str(args.output / "annotated.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 5.0, (640, 480))
    if not video.isOpened():
        raise RuntimeError("Could not open annotated video output")
    rows = []
    target_id = None
    anchor_frame = None
    anchor_profile = None
    frames_with_target = 0
    eligible_after_anchor = 0
    missing_exact_detection = 0
    try:
        with (args.output / "tracks.jsonl").open("w") as result_file:
            for index, event in enumerate(frame_events):
                key = frame_key(event["data"])
                image = cv2.imread(str(recording_root / event["data"]["path"]))
                if image is None:
                    raise RuntimeError(f"Could not read frame {index}")
                message = detections.get(key)
                if message is None:
                    missing_exact_detection += 1
                boxes = person_detections(message or {})
                outputs = tracker.update(Detections(boxes), image)
                tracks = []
                for output in outputs:
                    tracks.append(dict(box=[float(v) for v in output[:4]], track_id=int(output[4]),
                                       score=float(output[5]), detection_index=int(output[7])))

                if target_id is None and key in anchors and tracks:
                    candidate = max(tracks, key=lambda track: box_iou(track["box"], anchors[key]["body_box"]))
                    if box_iou(candidate["box"], anchors[key]["body_box"]) >= 0.5:
                        target_id = candidate["track_id"]
                        anchor_frame = index
                        anchor_profile = anchors[key]["profile_id"]
                target_present = target_id is not None and any(track["track_id"] == target_id for track in tracks)
                if key in eligible and target_id is not None:
                    eligible_after_anchor += 1
                    frames_with_target += int(target_present)
                row = dict(frame_index=index, elapsed_seconds=event["elapsed_ns"] / 1e9,
                           frame_key=list(key), eligible_in_reference=key in eligible,
                           exact_detections=message is not None, detection_count=len(boxes),
                           tracks=tracks, anchored_target_track_id=target_id,
                           anchored_target_output=target_present)
                result_file.write(json.dumps(row, separators=(",", ":")) + "\n")
                rows.append(row)
                for track in tracks:
                    x1, y1, x2, y2 = [round(v) for v in track["box"]]
                    colour = (0, 255, 0) if track["track_id"] == target_id else (255, 150, 0)
                    cv2.rectangle(image, (x1, y1), (x2, y2), colour, 2)
                    cv2.putText(image, f"ID {track['track_id']}", (max(0, x1), max(20, y1 - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2)
                cv2.putText(image, f"frame {index} head: recorded; anchor {target_id}",
                            (8, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.47, (255, 255, 255), 2)
                video.write(image)
    finally:
        video.release()
    eligible_after_anchor_keys = {tuple(row["frame_key"]) for row in rows[anchor_frame or 0:]
                                  if row["eligible_in_reference"]} if anchor_frame is not None else set()
    reference_after_anchor = [row for row in reference["observations"]
                              if tuple(row["frame_key"]) in eligible_after_anchor_keys]
    summary = dict(status="evaluated" if target_id is not None else "no_matching_identity_anchor",
                   tracker="Ultralytics BoT-SORT", recorded_frames=len(frame_events),
                   exact_detection_frames=len(frame_events) - missing_exact_detection,
                   eligible_reference_frames=len(eligible), anchor_frame=anchor_frame,
                   anchor_profile_id=anchor_profile, anchor_track_id=target_id,
                   eligible_frames_after_anchor=eligible_after_anchor,
                   frames_with_anchored_track_output=frames_with_target,
                   distinct_track_ids=len({track["track_id"] for row in rows for track in row["tracks"]}),
                   frames_with_two_or_more_tracks=sum(len(row["tracks"]) >= 2 for row in rows),
                   reference_tracker_retained_frames=reference["frames_with_retained_identity"],
                   reference_tracker_evaluated_frames=reference["evaluated_frames"],
                   reference_tracker_retained_after_anchor=sum(any(row["tracks"].values())
                                                              for row in reference_after_anchor),
                   reference_tracker_evaluated_after_anchor=len(reference_after_anchor),
                   manually_annotated_identity_switches=None,
                   limitation="One confirmed identity anchor, no independent ground truth; track output is not verified person identity. Recording contains no completed camera transition.")
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
