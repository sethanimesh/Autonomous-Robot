"""Portable recording checks: no ROS, webcam, network, or motors required."""
import ast
import contextlib
import io
import json
from pathlib import Path
import sqlite3
import tempfile
from types import SimpleNamespace
import unittest

from scripts.diagnostics.record_mission import (
    RecordingWriter, TOPICS, message_data, parse_args, read_profiles,
)


class MissionRecorderTests(unittest.TestCase):
    def writer(self, directory, **options):
        return RecordingWriter(Path(directory) / 'run', started_at_unix=100.,
                               started_monotonic_ns=1000, source_clock_unix=99., **options)

    def test_frames_and_motion_share_ordered_receipt_clock_and_keep_source_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            writer = self.writer(directory)
            header = dict(stamp=dict(sec=98, nanosec=400), frame_id='usb_camera')
            writer.record_jpeg(b'jpeg-example', header=header, width=640, height=480,
                source_encoding='bgr8', received_at_unix=100.1, received_monotonic_ns=1001000)
            writer.record_message('/cmd_vel', dict(linear=dict(x=.02), angular=dict(z=0.)),
                received_at_unix=100.2, received_monotonic_ns=2001000)
            writer.close()
            manifest = json.loads((writer.path / 'manifest.json').read_text())
            events = [json.loads(s) for s in (writer.path / 'events.jsonl').read_text().splitlines()]
            self.assertEqual([0, 1], [e['sequence'] for e in events])
            self.assertEqual([1000000, 2000000], [e['elapsed_ns'] for e in events])
            self.assertEqual(header, events[0]['data']['header'])
            self.assertEqual(b'jpeg-example', (writer.path / events[0]['data']['path']).read_bytes())
            self.assertEqual(99., manifest['source_clock_unix'])
            self.assertEqual(1, manifest['frames'])
            self.assertFalse(manifest['commands_sent'])
            self.assertEqual('complete', manifest['status'])

    def test_duration_limit_does_not_write_late_frames(self):
        with tempfile.TemporaryDirectory() as directory:
            writer = self.writer(directory, seconds=1.)
            self.assertFalse(writer.record_jpeg(b'late', header={}, width=1, height=1,
                source_encoding='mono8', received_at_unix=102., received_monotonic_ns=2000001000))
            writer.close()
            self.assertEqual('duration_limit', writer.stop_reason)
            self.assertEqual([], list((writer.path / 'frames').iterdir()))

    def test_storage_budget_applies_to_frames_plus_events_and_final_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            writer = self.writer(directory, max_bytes=65536)
            self.assertTrue(writer.record_message('/robot_status', {'connected': True},
                received_at_unix=100., received_monotonic_ns=1000))
            self.assertFalse(writer.record_jpeg(b'x'*60000, header={}, width=640, height=480,
                source_encoding='bgr8', received_at_unix=100., received_monotonic_ns=1000))
            writer.close()
            self.assertEqual('storage_limit', writer.stop_reason)
            self.assertLessEqual(sum(p.stat().st_size for p in writer.path.rglob('*') if p.is_file()), 65536)
            self.assertEqual(0, writer.frames)

    def test_recording_does_not_overwrite_an_existing_run(self):
        with tempfile.TemporaryDirectory() as directory:
            writer = self.writer(directory)
            writer.close()
            with self.assertRaises(FileExistsError):
                self.writer(directory)

    def test_json_topics_keep_identity_evidence_but_redact_embeddings(self):
        data = message_data(SimpleNamespace(data=json.dumps(dict(profile_id='mom', track_id='t',
            tracks=[dict(identity_source='face', embedding=[1., 2.])], templates=[[4.]]))),
            'std_msgs/msg/String', lambda x: None)
        self.assertEqual(dict(profile_id='mom', track_id='t', tracks=[dict(identity_source='face')]), data)
        self.assertNotIn('/perception/family_matches', TOPICS)
        self.assertNotIn('/perception/face_observations', TOPICS)
        self.assertEqual('bad-json', message_data(SimpleNamespace(data='bad-json'), 'std_msgs/msg/String', None))

    def test_profiles_are_read_only_labels_and_revisions(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'people.sqlite3'
            with sqlite3.connect(path) as db:
                db.execute('CREATE TABLE profiles(id TEXT,label TEXT,revision TEXT,payload TEXT)')
                db.execute('INSERT INTO profiles VALUES(?,?,?,?)', ('mom', 'Mom', 'rev', '{"templates":[1,2]}'))
            original = path.read_bytes()
            self.assertEqual([dict(id='mom', label='Mom', revision='rev')], read_profiles(path))
            self.assertEqual(original, path.read_bytes())
            self.assertEqual([], read_profiles(None))

    def test_camera_recording_requires_explicit_cli_opt_in(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(['--output', '/tmp/not-created'])
        args = parse_args(['--record-camera', '--output', '/tmp/not-created'])
        self.assertEqual(5., args.fps)
        self.assertEqual(180., args.seconds)

    def test_invalid_limits_do_not_create_output(self):
        for options in (dict(seconds=float('nan')), dict(fps=0), dict(max_bytes=1), dict(jpeg_quality=1)):
            with tempfile.TemporaryDirectory() as directory:
                with self.assertRaises(ValueError):
                    self.writer(directory, **options)
                self.assertFalse((Path(directory) / 'run').exists())

    def test_replay_loader_reads_recorded_frame_and_exact_detection_binding(self):
        from scripts.diagnostics.replay_mission import load_recording, frame_key, summary
        with tempfile.TemporaryDirectory() as directory:
            writer = self.writer(directory)
            header = dict(stamp=dict(sec=99, nanosec=1234567), frame_id='camera')
            writer.record_jpeg(b'synthetic-image', header=header, width=640, height=480,
                source_encoding='bgr8', received_at_unix=100., received_monotonic_ns=1000)
            writer.record_message('/perception/person_detections', dict(header=header, detections=[]),
                received_at_unix=100.1, received_monotonic_ns=100001000)
            writer.close()
            root, manifest, events = load_recording(writer.path)
            self.assertEqual(frame_key(events[0]['data']), frame_key(events[1]['data']))
            self.assertEqual(1, summary(manifest, events)['frames'])
            self.assertFalse(summary(manifest, events)['physical_commands_sent'])

    def test_live_adapter_is_subscribe_only(self):
        source = Path('scripts/diagnostics/record_mission.py').read_text()
        tree = ast.parse(source)
        calls = {n.func.attr for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}
        self.assertIn('create_subscription', calls)
        self.assertNotIn('create_publisher', calls)
        self.assertNotIn('publish', calls)


if __name__ == '__main__':
    unittest.main()
