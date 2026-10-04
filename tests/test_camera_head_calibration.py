import unittest

from robot.jetson.mission.camera_head_calibration import center_floor_fraction
from robot.jetson.mission.camera_head_calibration import choose_runtime_positions
from robot.jetson.mission.camera_head_calibration import dry_run_report
from robot.jetson.mission.camera_head_calibration import next_upward_position
from robot.jetson.mission.camera_head_calibration import parse_args
from robot.jetson.mission.camera_head_calibration import stable_reference_tail
from robot.jetson.mission.camera_head_calibration import track_motion_active
from robot.jetson.mission.camera_head_calibration import validate_runtime_positions
from robot.jetson.mission.camera_head_calibration import CameraCalibrationError
from robot.jetson.mission.camera_head_calibration import scene_sample, floor_reference_visible
from robot.jetson.mission.camera_head_calibration import probe_return_step, retained_floor_position, verify_view_role


class CameraHeadCalibrationTests(unittest.TestCase):
    def test_floor_beside_central_obstacle_can_verify_lower_view_without_clearing_route(self):
        sample = dict(view_usable=True, floor_fraction=.29, known_fraction=.74,
                      bottom_floor_fraction=.44, bottom_known_fraction=.45,
                      bottom_left_floor_fraction=.86, bottom_left_known_fraction=.88,
                      route_blocked=True)
        self.assertTrue(floor_reference_visible(sample))
        self.assertTrue(sample["route_blocked"])
        self.assertTrue(verify_view_role("down", dict(sample, known_fraction=.5)))
        with self.assertRaises(CameraCalibrationError):
            verify_view_role("forward", dict(sample, known_fraction=.5))
        for changes in ({"bottom_left_floor_fraction": .2}, {"bottom_left_known_fraction": .5},
                        {"floor_fraction": .01}, {"known_fraction": .2},
                        {"bottom_left_floor_fraction": float("nan")}):
            self.assertFalse(floor_reference_visible(dict(sample, **changes)))

    def test_operator_overhead_reference_can_verify_a_view_without_faking_pixel_labels(self):
        sample = dict(view_usable=True, floor_fraction=.01, ceiling_fraction=0,
                      known_fraction=.51, overhead_reference_verified=True)
        self.assertTrue(verify_view_role("up", sample))
        self.assertEqual(0, sample["ceiling_fraction"])
        with self.assertRaises(CameraCalibrationError):
            verify_view_role("up", dict(sample, overhead_reference_verified=False))
        with self.assertRaises(CameraCalibrationError):
            verify_view_role("forward", sample)

    def test_partial_floor_view_requires_strong_bottom_evidence(self):
        sample = dict(view_usable=True, floor_fraction=.19, known_fraction=.80,
                      bottom_floor_fraction=.72, bottom_known_fraction=.85)
        self.assertTrue(floor_reference_visible(sample))
        for changes in ({"bottom_floor_fraction": .2}, {"bottom_known_fraction": .5},
                        {"floor_fraction": .01}, {"bottom_floor_fraction": float("nan")}):
            self.assertFalse(floor_reference_visible(dict(sample, **changes)))

    def test_requested_pose_keeps_small_measured_settling_error_without_rezero(self):
        samples = self.scene_samples()
        for sample in samples:
            sample["target_position"] = sample["position"] - 24
            sample["position"] = sample["target_position"] + 2
        result = choose_runtime_positions(samples, floor_position=-24)
        self.assertEqual(-24, result["down_position"])
        samples[0]["position"] += 2
        with self.assertRaises(CameraCalibrationError):
            choose_runtime_positions(samples, floor_position=-24)

    def test_probe_returns_from_eight_counts_instead_of_skipping_the_return(self):
        self.assertEqual(8, probe_return_step(-8))
        self.assertEqual(15, probe_return_step(-20))
        self.assertIsNone(probe_return_step(1))

    def test_floor_pixels_do_not_normalize_a_four_count_offset_to_zero(self):
        samples = self.scene_samples()
        samples[0]["position"] = 4
        with self.assertRaises(CameraCalibrationError):
            choose_runtime_positions(samples)

    def test_negative_floor_anchor_is_preserved_in_selection_and_probe_return(self):
        samples = self.scene_samples()
        for sample in samples:
            sample["position"] -= 29
        result = choose_runtime_positions(samples, floor_position=-29)
        self.assertEqual(-29, result["down_position"])
        self.assertTrue(validate_runtime_positions(samples, result, floor_position=-29)["verified"])
        self.assertEqual(8, probe_return_step(-37, floor_position=-29))

    def test_startup_requires_retained_boundary_and_never_relabels_unknown_pose(self):
        head = dict(require_approved_reference=True, reference_id="ref", approved_reference_id="ref",
                    homed=True, position=-29, down_position=-29, maximum_target_position=-29,
                    minimum_position=-54)
        self.assertEqual(-29, retained_floor_position(head))
        for change in ({"position": -21}, {"position": 0}, {"reference_id": "reboot"},
                       {"homed": False}, {"approved_reference_id": None}, {"moving": True}):
            with self.subTest(change=change), self.assertRaises(CameraCalibrationError):
                retained_floor_position(dict(head, **change))

    def test_scene_evidence_requires_same_stopped_reference_pose_and_freshness(self):
        result = {"ok": True, "result_age_seconds": .1,
                  "camera_head": {"available": True, "homed": True, "position": 0,
                                  "reference_id": "ref", "moving": False},
                  "scene": {"floor_fraction": .8, "ceiling_fraction": .01, "known_fraction": .95}}
        self.assertEqual(.8, scene_sample(result, 0, "ref", .2)["floor_fraction"])
        self.assertEqual(0,scene_sample(result,2,'ref',.2)['scene_position'])
        with self.assertRaises(CameraCalibrationError):
            scene_sample(result,4,'ref',.2)
        for position, reference, elapsed in ((15, "ref", .2), (0, "old", .2), (0, "ref", 3), (0, "ref", float("nan"))):
            with self.subTest(position=position, reference=reference, elapsed=elapsed), self.assertRaises(CameraCalibrationError):
                scene_sample(result, position, reference, elapsed)
        result["camera_head"]["moving"] = True
        with self.assertRaises(CameraCalibrationError):
            scene_sample(result, 0, "ref", .2)

    def test_unknown_encoder_pose_can_be_inspected_without_moving_or_homing(self):
        result = {"ok": True, "result_age_seconds": .1,
                  "camera_head": {"available": True, "homed": False, "position": 185, "reference_id": "ref"},
                  "scene": {"floor_fraction": .8, "ceiling_fraction": .01, "known_fraction": .95}}
        sample = scene_sample(result, 185, "ref", .2, require_homed=False)
        self.assertTrue(floor_reference_visible(dict(sample, view_usable=True)))
        with self.assertRaises(CameraCalibrationError):
            scene_sample(result, 185, "ref", .2)
        self.assertFalse(floor_reference_visible(dict(sample, floor_fraction=.1, view_usable=True)))

    def test_reference_tail_allows_early_correction_then_requires_stability(self):
        self.assertEqual([-3, -3, -4], stable_reference_tail([-4, 2, -3, -3, -4]))
        self.assertEqual([4], stable_reference_tail([0, 4, 0, 4]))

    def test_upward_steps_are_bounded_and_end_exactly_on_limit(self):
        positions = [0]
        while True:
            value = next_upward_position(positions[-1], -52, 15)
            if value is None:
                break
            positions.append(value)
        self.assertEqual([0, -15, -30, -45, -52], positions)

    @staticmethod
    def scene_samples():
        return [dict(position=p, floor_fraction=f, known_fraction=0.98, ceiling_fraction=(.65 if p == -45 else .01),
                     view_usable=True) for p, f in ((0, .90), (-15, .61), (-30, .43), (-45, .05))]

    def test_empty_room_calibrates_without_any_person_or_face(self):
        samples = self.scene_samples()
        result = choose_runtime_positions(samples)
        self.assertEqual((0, -30, -45), (result["down_position"], result["forward_position"], result["up_position"]))
        self.assertTrue(validate_runtime_positions(samples, result)["verified"])

    def test_person_evidence_does_not_change_camera_geometry(self):
        samples = self.scene_samples()
        before = choose_runtime_positions(samples)
        samples[0].update(target_confirmed=True, target_score=.99, body_height_fraction=.9)
        self.assertEqual(before, choose_runtime_positions(samples))

    def test_bright_images_without_floor_cannot_enable_calibration(self):
        samples = self.scene_samples()
        for sample in samples:
            sample["floor_fraction"] = 0.0
        with self.assertRaisesRegex(CameraCalibrationError, "No stable floor"):
            choose_runtime_positions(samples)

    def test_unknown_and_nonfinite_segmentation_are_not_floor_evidence(self):
        for value in (.2, float("nan"), float("inf")):
            samples = self.scene_samples()
            samples[0]["known_fraction"] = value
            with self.subTest(value=value), self.assertRaises(CameraCalibrationError):
                choose_runtime_positions(samples)

    def test_occluded_floor_view_is_rejected(self):
        samples = self.scene_samples()
        samples[0]["view_usable"] = False
        with self.assertRaises(CameraCalibrationError):
            choose_runtime_positions(samples)

    def test_no_forward_floor_transition_is_rejected(self):
        samples = self.scene_samples()
        for sample in samples[1:]:
            sample["floor_fraction"] = 0.0
            sample["ceiling_fraction"] = .65
        with self.assertRaisesRegex(CameraCalibrationError, "No forward room view"):
            choose_runtime_positions(samples)

    def test_selected_poses_cannot_bypass_visual_evidence(self):
        samples = self.scene_samples()
        result = choose_runtime_positions(samples)
        result["down_position"] = -45
        with self.assertRaises(CameraCalibrationError):
            validate_runtime_positions(samples, result)

    def test_small_range_cannot_be_called_complete(self):
        samples = self.scene_samples()
        for sample in samples:
            sample["position"] //= 10
        result = choose_runtime_positions(samples)
        with self.assertRaisesRegex(CameraCalibrationError, "Three distinct"):
            validate_runtime_positions(samples, result)

    def test_route_center_floor_fraction_uses_nearest_heading(self):
        value = center_floor_fraction(
            {
                "evidence": [
                    {"heading_degrees": -20, "floor_fraction": 0.1},
                    {"heading_degrees": 5, "floor_fraction": 0.7},
                    {"heading_degrees": 0, "floor_fraction": 0.8},
                ]
            }
        )
        self.assertEqual(0.8, value)

    def test_tool_motion_never_counts_as_chassis_motion(self):
        self.assertFalse(
            track_motion_active(
                {
                    "motion_active": True,
                    "track_motion_active": False,
                    "tool_motion_active": True,
                }
            )
        )

    def test_dry_run_contains_no_chassis_commands(self):
        report = dry_run_report(parse_args([]))
        self.assertTrue(report["chassis_locked"])
        self.assertEqual(0, report["planned_positions"][0])
        self.assertEqual(-48, report["planned_positions"][-1])
        self.assertEqual(1.5, parse_args([]).boost_no_progress_timeout)


if __name__ == "__main__":
    unittest.main()
