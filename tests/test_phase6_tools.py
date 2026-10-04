import copy
import io
import json
from pathlib import Path
import tarfile
import tempfile
import subprocess
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from scripts.phase6.phase6 import bundle, install, read_bundle, readiness, start_heading


def healthy_samples():
    robot = dict(tool_reference_id='head', motors={
        side: dict(generation=side+'-boot', position=0, speed=0, commanded_speed=0)
        for side in ('left', 'right', 'tool')})
    return dict(camera=[dict(state='streaming')],
                person=[dict(inferences=5), dict(inferences=15)],
                face=[dict(detections_published=5), dict(detections_published=15)],
                identity=[dict(matches_published=5, target_label='family'),
                          dict(matches_published=15, target_label='family')],
                observer=[dict(ok=True, confirmed=False, target_label='family')],
                robot=[robot], head=[dict(calibrated=True, homed=True, manual_override=False,
                                         reference_id='head', approved_reference_id='head')])


class Phase6ToolTests(unittest.TestCase):
    def test_navigation_bundle_loads_in_the_flat_jetson_runtime(self):
        with tempfile.TemporaryDirectory() as directory:
            archive = Path(directory) / 'navigation.tar.gz'
            runtime = Path(directory) / 'runtime'
            with patch('builtins.print'):
                bundle(archive)
                install(SimpleNamespace(archive=archive, runtime=runtime,
                    household_profile=False, faster_tracks=False, restart=False))
            for name in ('navigation_reasoning.py', 'occlusion_advice.py'):
                self.assertIn(name, read_bundle(archive))
            result = subprocess.run([sys.executable, '-I', '-c',
                'import sys; sys.path.insert(0, sys.argv[1]); '
                'import navigation_reasoning, occlusion_advice, closed_loop_detour, bounded_target_scan',
                str(runtime)], capture_output=True, text=True, check=False)
            self.assertEqual(0, result.returncode, result.stderr)

    def test_advancing_pipeline_does_not_require_person_in_view(self):
        problems, warnings = readiness(healthy_samples(), 10, True, {'observer': .2})
        self.assertEqual([], problems)
        self.assertEqual([], warnings)

    def test_face_fallback_allows_degraded_person_feed(self):
        samples = healthy_samples()
        samples['person'] = []
        problems, warnings = readiness(samples, 10, True, {'observer': .2})
        self.assertEqual([], problems)
        self.assertEqual(['person processing is not advancing'], warnings)

    def test_stalled_identity_and_frozen_camera_are_reported(self):
        samples = healthy_samples()
        samples['identity'][-1]['matches_published'] = 5
        problems, _ = readiness(samples, 10, True, {'image': 8., 'observer': .2})
        self.assertIn('identity processing is not advancing', problems)
        self.assertIn('fresh camera frames unavailable', problems)

    def test_changed_head_reference_prevents_using_old_limits(self):
        samples = healthy_samples()
        samples['robot'][0]['tool_reference_id'] = 'new-boot'
        problems, _ = readiness(samples, 10, True, {'observer': .2})
        self.assertIn('camera reference unavailable or changed', problems)

    def test_resume_recomputes_heading_from_current_same_generation_encoders(self):
        status = {role: entries[-1] for role, entries in healthy_samples().items()}
        previous = dict(status=copy.deepcopy(status), initial_heading_degrees=10.)
        geometry = dict(wheel_radius_m=.0144504, track_width_m=.182557,
                        encoder_counts_per_rev=360., left_sign=1., right_sign=1.)
        status['robot']['motors']['left']['position'] = 100
        status['robot']['motors']['right']['position'] = -100
        self.assertAlmostEqual(25.8311, start_heading(status, previous, geometry), places=3)
        status['robot']['motors']['left']['generation'] = 'reboot'
        with self.assertRaisesRegex(ValueError, 'reference changed'):
            start_heading(status, previous, geometry)

    def test_install_preserves_calibration_enrollment_and_models(self):
        with tempfile.TemporaryDirectory() as folder:
            archive = Path(folder) / 'phase6.tar.gz'
            runtime = Path(folder) / 'runtime'
            runtime.mkdir()
            originals = {'robot.yaml': 'my saved limits', 'perception.yaml': 'my model settings',
                         'data/target_person.json': 'my enrolled templates'}
            for name, content in originals.items():
                path = runtime / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content)
            (runtime / 'autonomous_find.py').write_text('# previous version')
            with patch('builtins.print'):
                bundle(archive)
                files = read_bundle(archive)
                install(SimpleNamespace(archive=archive, runtime=runtime,
                                        household_profile=False, restart=False))
            self.assertIn('recovery_policy.py', files)
            self.assertNotIn('robot.yaml', files)
            for name, content in originals.items():
                self.assertEqual(content, (runtime / name).read_text())
            backups = list((runtime / 'backups').glob('*/autonomous_find.py'))
            self.assertEqual('# previous version', backups[0].read_text())

    def test_invalid_bundle_cannot_write_outside_runtime(self):
        with tempfile.TemporaryDirectory() as folder:
            archive = Path(folder) / 'invalid.tar.gz'
            with tarfile.open(archive, 'w:gz') as tar:
                entry = tarfile.TarInfo('../outside.py')
                entry.size = 1
                tar.addfile(entry, io.BytesIO(b'1'))
            with self.assertRaisesRegex(ValueError, 'Invalid'):
                read_bundle(archive)

    def test_household_install_merges_only_recognition_tuning(self):
        # JSON is valid YAML; use it here to exercise the merge without needing
        # the Jetson's PyYAML dependency on the development Mac.
        parser = SimpleNamespace(safe_load=json.loads, safe_dump=json.dumps)
        with tempfile.TemporaryDirectory() as folder:
            archive = Path(folder) / 'phase6.tar.gz'
            runtime = Path(folder) / 'runtime'
            runtime.mkdir()
            original = {'echora_target_recognizer': {'ros__parameters': {
                'model_path': 'existing.engine', 'model_sha256': 'existing-checksum',
                'target_store_path': 'existing-enrollment', 'match_threshold': .45,
                'confirmation_required': 3}}}
            config = runtime / 'perception.yaml'
            config.write_text(json.dumps(original))
            with patch('builtins.print'), patch.dict('sys.modules', {'yaml': parser}):
                bundle(archive)
                install(SimpleNamespace(archive=archive, runtime=runtime,
                                        household_profile=True, restart=False))
            values = json.loads(config.read_text())['echora_target_recognizer']['ros__parameters']
            self.assertEqual(.40, values['match_threshold'])
            self.assertEqual(2, values['confirmation_required'])
            for key in ('model_path', 'model_sha256', 'target_store_path'):
                self.assertEqual(original['echora_target_recognizer']['ros__parameters'][key], values[key])


    def test_speed_install_retains_live_camera_limits_and_geometry(self):
        parser = SimpleNamespace(safe_load=json.loads, safe_dump=json.dumps)
        with tempfile.TemporaryDirectory() as folder:
            archive = Path(folder) / 'phase6.tar.gz'
            runtime = Path(folder) / 'runtime'
            runtime.mkdir()
            parameters = dict(max_motor_speed=120, camera_head_minimum_position=-10,
                              camera_head_maximum_position=35,
                              camera_head_approved_reference_id='live-reference',
                              wheel_radius_m=.0144504, track_width_m=.182557)
            config = runtime / 'robot.yaml'
            config.write_text(json.dumps({'ev3_bridge': {'ros__parameters': parameters}}))
            with patch('builtins.print'), patch.dict('sys.modules', {'yaml': parser}):
                bundle(archive)
                install(SimpleNamespace(archive=archive, runtime=runtime,
                                        household_profile=False, faster_tracks=True, restart=False))
            updated = json.loads(config.read_text())['ev3_bridge']['ros__parameters']
            self.assertEqual(dict(parameters, max_motor_speed=240), updated)


if __name__ == '__main__':
    unittest.main()
