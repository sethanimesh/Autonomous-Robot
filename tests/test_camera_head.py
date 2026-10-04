import unittest

from robot.jetson.ev3_bridge.camera_head import CameraHeadController
from robot.jetson.ev3_bridge.camera_head import CameraHeadError


class FakeClient(object):
    def __init__(self, position=0):
        self.position = position
        self.commands = []

    def status(self):
        return {
            "motors": {"tool": {"position": self.position}},
            "tool_homed": True,
            "tool_reference_id": "test-reference",
            "tool_motion_active": False,
        }

    def move_tool(self, position, speed):
        self.commands.append(("move", position, speed))
        self.position = position
        return {"applied": {"position": position, "speed": speed}}

    def zero_tool(self):
        self.commands.append(("zero",))
        self.position = 0
        return {"position": 0}

    def home_tool(self, speed=25):
        self.commands.append(("home", speed))
        return {"applied": {"direction": 1, "speed": speed}}

    def acknowledge_tool_position(self):
        self.commands.append(("acknowledge_position",))
        return {"position": self.position}

    def stop(self):
        self.commands.append(("stop",))
        return {"stopped": True}


class StalledClient(FakeClient):
    def move_tool(self, position, speed):
        self.commands.append(("move", position, speed))
        return {"applied": {"position": position, "speed": speed}}


class LiftingNeedsBoostClient(FakeClient):
    def move_tool(self, position, speed):
        self.commands.append(("move", position, speed))
        if speed >= 1500:
            self.position = position
        return {"applied": {"position": position, "speed": speed}}


class MovingClient(FakeClient):
    def status(self):
        return {
            "motors": {"tool": {"position": self.position}},
            "tool_homed": True,
            "tool_reference_id": "test-reference",
            "tool_motion_active": True,
        }


class ActiveJogClient(FakeClient):
    def __init__(self, position, target):
        super().__init__(position)
        self.target = target

    def status(self):
        return {
            "motors": {"tool": {"position": self.position}},
            "tool_homed": True,
            "tool_reference_id": "test-reference",
            "tool_motion_active": True,
            "tool_target_position": self.target,
        }

    def move_tool(self, position, speed):
        self.commands.append(("move", position, speed))
        return {"applied": {"position": position, "speed": speed}}


class BrickRetryExhaustedClient(StalledClient):
    def status(self):
        return {
            "motors": {"tool": {"position": self.position}},
            "tool_homed": True,
            "tool_reference_id": "test-reference",
            "tool_motion_active": False,
            "tool_stall_retry_count": 1,
            "tool_stall_retry_limit": 1,
        }


class FakeClock(object):
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class ScriptedMoveClient(FakeClient):
    """Timestamped motor feedback, including stopped drift or renewed motion."""

    def __init__(self, clock, samples):
        super().__init__(position=0)
        self.clock = clock
        self.samples = samples
        self.started = None

    def move_tool(self, position, speed):
        self.commands.append(("move", position, speed))
        self.started = self.clock()
        return {"applied": {"position": position, "speed": speed}}

    def status(self):
        status = super().status()
        if self.started is not None:
            elapsed = self.clock() - self.started
            applicable = [sample for sample in self.samples if sample[0] <= elapsed + 1e-9]
            if applicable:
                _, position, moving, extra = applicable[-1]
                self.position = position
                status["motors"]["tool"]["position"] = position
                status["tool_motion_active"] = moving
                status.update(extra)
        return status


class CameraHeadControllerTests(unittest.TestCase):
    def test_cloud_limits_restore_verified_room_view_after_restart(self):
        client = FakeClient(position=25)
        head = CameraHeadController(client, minimum_position=-22, maximum_position=28,
                                    forward_position=2, down_position=22, up_position=-19,
                                    lower_target_margin=6, upper_target_margin=3)
        limits = dict(version=1, reference_id='test-reference', upper=-22, lower=28,
                      source='cloud_useful_views')
        head._apply_manual_limits(limits)
        self.assertEqual((22,22,-19), (head.forward_position,head.down_position,head.up_position))
        head.use_saved_limits()
        self.assertEqual(22, head.forward_position)

    def test_move_acknowledges_only_finished_request_and_reports_failure(self):
        client = FakeClient(position=-10)
        head = CameraHeadController(client, calibrated=True, post_move_settle_seconds=0)
        published = []
        head.progress_callback = lambda status: published.append(head.describe(status)['move_result'])
        head.execute(dict(action='move_to', target=-20, request_id='new-move'))
        self.assertTrue(published)
        self.assertTrue(all(value is None for value in published))
        self.assertEqual(dict(request_id='new-move', ok=True), head.describe(client.status())['move_result'])
        with self.assertRaises(ValueError):
            head.execute(dict(action='move_to', target=999, request_id='bad-move'))
        result = head.describe(client.status())['move_result']
        self.assertEqual('bad-move', result['request_id'])
        self.assertFalse(result['ok'])

    def test_named_move_finishes_at_requested_target_after_intermediate_overshoot(self):
        class OvershootingClient(FakeClient):
            def move_tool(self, position, speed):
                self.commands.append(('move', position, speed))
                self.position = position + 2
                return {}
        client = OvershootingClient(position=8)
        head = CameraHeadController(client, minimum_position=4, maximum_position=50,
                                    forward_position=26, down_position=44, up_position=7,
                                    lower_target_margin=6, upper_target_margin=3,
                                    settle_tolerance=4, post_move_settle_seconds=0)
        head._apply_manual_limits(dict(version=1, reference_id='test-reference', upper=4, lower=50))
        result = head.execute(dict(action='move_to', target=44))
        self.assertEqual([23, 40, 44], [c[1] for c in client.commands])
        self.assertEqual(46, result['position'])
        self.assertEqual(44, result['steps'][-1]['requested'])

    def scripted_head(self, samples, **kwargs):
        clock = FakeClock()
        client = ScriptedMoveClient(clock, samples)
        head = CameraHeadController(
            client, calibrated=True, forward_position=-15, down_position=0,
            settle_tolerance=4, clock=clock, sleeper=clock.sleep, **kwargs)
        return head, client, clock

    def approved_head(self, client):
        return CameraHeadController(client, minimum_position=-54, maximum_position=-21,
                                    down_position=-29, forward_position=-29, up_position=-42,
                                    approved_reference_id="test-reference", require_approved_reference=True)

    def test_explicit_new_boot_lower_reference_keeps_encoder_and_relative_limits(self):
        client = FakeClient(0)
        head = self.approved_head(client)
        with self.assertRaises(CameraHeadError):
            head.reference_current_lower("test-reference", 0)
        with self.assertRaises(CameraHeadError):
            head.reference_current_lower("stale-reference", 0, True)
        self.assertEqual([], client.commands)
        result = head.reference_current_lower("test-reference", 0, True)
        self.assertEqual([("acknowledge_position",)], client.commands)
        self.assertEqual(0, client.position)
        self.assertEqual((-33, 0), (result["minimum_position"], result["maximum_position"]))
        self.assertFalse(head.calibrated)

    def test_measured_target_margin_is_independent_of_movement_tolerance(self):
        client = FakeClient(-28)
        head = CameraHeadController(client, minimum_position=-54, maximum_position=-21,
                                    down_position=-24, forward_position=-29, up_position=-42,
                                    approved_reference_id="test-reference", require_approved_reference=True,
                                    lower_target_margin=3)
        self.assertEqual(-24, head.maximum_target_position)
        self.assertEqual(8, head.settle_tolerance)
        with self.assertRaises(ValueError):
            head.jog_to(-23)
        self.assertEqual([], client.commands)
        head.jog_to(-24)
        self.assertEqual(-24, client.position)

    def test_lower_boundary_cannot_be_rezeroed_homed_or_acknowledged_away(self):
        client = FakeClient(-21)
        head = self.approved_head(client)
        for action in ("zero", "home", "acknowledge_position"):
            with self.subTest(action=action), self.assertRaises(CameraHeadError):
                head.execute(action)
        self.assertEqual([], client.commands)
        self.assertEqual(-21, client.position)

    def test_lower_boundary_margin_rejects_downward_targets_without_motion(self):
        client = FakeClient(-21)
        head = self.approved_head(client)
        with self.assertRaises(CameraHeadError):
            head.jog(15)
        client.position = -36
        for target in (-28, -21, -14, 0):
            with self.subTest(target=target), self.assertRaises(ValueError):
                head.jog_to(target)
        self.assertEqual([], client.commands)
        head.jog_to(-29)
        self.assertEqual(-29, client.position)

    def test_reference_change_locks_motion_even_with_homed_true(self):
        client = FakeClient(-29)
        head = self.approved_head(client)
        head.approved_reference_id = "previous-boot"
        with self.assertRaises(CameraHeadError):
            head.jog(-15)
        self.assertEqual([], client.commands)

    def test_settling_tolerance_does_not_expand_physical_lower_limit(self):
        client = FakeClient(-20)
        head = self.approved_head(client)
        with self.assertRaises(CameraHeadError):
            head.jog(-15)
        self.assertEqual([], client.commands)

    def test_out_of_range_status_invalidates_calibration_and_refuses_motion(self):
        client = FakeClient(185)
        head = CameraHeadController(client, calibrated=True, forward_position=-27,
                                    down_position=0, up_position=-42,
                                    minimum_position=-54, maximum_position=0)
        self.assertFalse(head.describe(client.status())["calibrated"])
        with self.assertRaisesRegex(CameraHeadError, "outside"):
            head.move_named("down")
        self.assertEqual([], client.commands)

    def test_reference_change_requires_new_visual_calibration(self):
        client = FakeClient(0)
        head = CameraHeadController(client, calibrated=True)
        first = client.status()
        head.describe(first)
        changed = dict(first, tool_reference_id="after-reboot")
        self.assertFalse(head.describe(changed)["calibrated"])
        self.assertFalse(head.describe(first)["calibrated"])

    def test_stale_calibration_result_cannot_enable_a_new_reference(self):
        head = CameraHeadController(FakeClient(0))
        with self.assertRaisesRegex(CameraHeadError, "different encoder reference"):
            head.set_runtime_positions(-15, 0, -45, reference_id="old-reference")
        self.assertFalse(head.calibrated)

    def test_missing_reference_or_lost_home_invalidates_calibration(self):
        for change in ({"tool_homed": False}, {"tool_reference_id": None}):
            client = FakeClient(0)
            head = CameraHeadController(client, calibrated=True)
            self.assertFalse(head.describe(dict(client.status(), **change))["calibrated"])

    def test_stop_action_stops_tracks_and_holds_camera_head(self):
        client = FakeClient(position=-42)
        head = CameraHeadController(client)

        result = head.execute('{"action":"stop"}')

        self.assertEqual({"stopped": True}, result)
        self.assertEqual([("stop",)], client.commands)

    def test_uncalibrated_head_allows_only_bounded_jog(self):
        client = FakeClient(position=-20)
        head = CameraHeadController(client)

        head.execute('{"action":"jog","degrees":-10}')

        self.assertEqual([("move", -30, 40)], client.commands)
        with self.assertRaises(CameraHeadError):
            head.execute("look_forward")
        with self.assertRaises(CameraHeadError):
            head.execute('{"action":"jog","degrees":16}')

    def test_calibrator_can_request_one_bounded_speed_boost(self):
        client = FakeClient(position=-90)
        head = CameraHeadController(client, speed=300, boost_speed=1500)

        head.execute('{"action":"jog","degrees":-15,"speed":1500}')

        self.assertEqual([("move", -105, 1500)], client.commands)
        with self.assertRaises(CameraHeadError):
            head.execute('{"action":"jog","degrees":-15,"speed":1501}')

    def test_manual_ui_jog_uses_its_validated_absolute_target(self):
        client = FakeClient(position=-20)
        head = CameraHeadController(client, speed=300)

        head.execute('{"action":"jog_to","target":-35}')

        self.assertEqual([("move", -35, 300)], client.commands)
        with self.assertRaises(CameraHeadError):
            head.execute('{"action":"jog_to","target":-51}')

    def test_active_lifting_jog_can_retry_same_absolute_target_at_boost(self):
        client = ActiveJogClient(position=-21, target=-35)
        head = CameraHeadController(client, speed=300, boost_speed=1500)

        head.execute('{"action":"retry_jog","target":-35}')

        self.assertEqual([("move", -35, 1500)], client.commands)

    def test_active_lowering_jog_retry_keeps_normal_speed(self):
        client = ActiveJogClient(position=-21, target=-10)
        head = CameraHeadController(client, speed=300, boost_speed=1500)

        head.execute('{"action":"retry_jog","target":-10}')

        self.assertEqual([("move", -10, 300)], client.commands)

    def test_jog_retry_cannot_change_or_extend_active_target(self):
        client = ActiveJogClient(position=-21, target=-35)
        head = CameraHeadController(client, speed=300, boost_speed=1500)

        with self.assertRaisesRegex(CameraHeadError, "no longer active"):
            head.execute('{"action":"retry_jog","target":-36}')

        self.assertEqual([], client.commands)

    def test_jog_is_trimmed_to_the_software_limit(self):
        client = FakeClient(position=-175)
        head = CameraHeadController(client)

        head.jog(-10)

        self.assertEqual([("move", -180, 40)], client.commands)

    def test_jog_is_rejected_once_the_head_sits_on_the_limit(self):
        client = FakeClient(position=-180)
        head = CameraHeadController(client)

        with self.assertRaises(CameraHeadError):
            head.jog(-10)

        self.assertEqual([], client.commands)

    def test_calibrated_named_positions(self):
        client = FakeClient(position=0)
        clock = FakeClock()
        head = CameraHeadController(
            client,
            calibrated=True,
            forward_position=0,
            down_position=-75,
            clock=clock,
            sleeper=clock.sleep,
        )

        head.execute("look_down")
        head.execute("look_forward")

        self.assertEqual(
            [
                ("move", -15, 40),
                ("move", -30, 40),
                ("move", -45, 40),
                ("move", -60, 40),
                ("move", -75, 40),
                ("move", -60, 40),
                ("move", -45, 40),
                ("move", -30, 40),
                ("move", -15, 40),
                ("move", 0, 40),
            ],
            client.commands,
        )

    def test_lift_to_upper_pose_is_ten_negative_fifteen_count_pulses(self):
        client = FakeClient(position=0)
        clock = FakeClock()
        head = CameraHeadController(
            client,
            calibrated=True,
            forward_position=-75,
            down_position=0,
            up_position=-150,
            speed=300,
            clock=clock,
            sleeper=clock.sleep,
        )

        result = head.execute("look_up")

        moves = [command for command in client.commands if command[0] == "move"]
        self.assertEqual(10, len(moves))
        self.assertEqual(list(range(-15, -151, -15)), [move[1] for move in moves])
        self.assertTrue(all(move[2] == 300 for move in moves))
        self.assertEqual(-150, result["position"])

    def test_loaded_lifting_gets_one_bounded_boost_without_a_larger_step(self):
        client = LiftingNeedsBoostClient(position=-75)
        clock = FakeClock()
        head = CameraHeadController(
            client,
            calibrated=True,
            forward_position=-75,
            down_position=0,
            up_position=-90,
            speed=300,
            boost_speed=1500,
            clock=clock,
            sleeper=clock.sleep,
        )

        result = head.execute("look_up")

        self.assertEqual(
            [("move", -90, 300), ("stop",), ("move", -90, 1500)],
            client.commands,
        )
        self.assertEqual(-90, result["position"])
        self.assertTrue(result["steps"][0]["boosted"])

    def test_jetson_does_not_repeat_an_exhausted_brick_retry(self):
        client = BrickRetryExhaustedClient(position=-75)
        clock = FakeClock()
        head = CameraHeadController(
            client,
            calibrated=True,
            forward_position=-75,
            down_position=0,
            up_position=-90,
            speed=300,
            boost_speed=1500,
            clock=clock,
            sleeper=clock.sleep,
        )

        with self.assertRaisesRegex(CameraHeadError, "stalled"):
            head.execute("look_up")

        self.assertEqual([("move", -90, 300), ("stop",)], client.commands)

    def test_stopped_encoder_drift_restarts_short_settlement_window(self):
        head, client, clock = self.scripted_head([
            (0., -12, False, {}), (.2, -14, False, {}),
        ])
        result = head.move_named("forward")
        self.assertEqual(-14, result["position"])
        self.assertAlmostEqual(.6, clock())
        self.assertEqual([("move", -15, 40)], client.commands)

    def test_resumed_motion_restarts_settlement_even_if_position_is_unchanged(self):
        head, client, clock = self.scripted_head([
            (0., -14, False, {}), (.2, -14, True, {}), (.4, -14, False, {}),
        ])
        head.move_named("forward")
        self.assertAlmostEqual(.8, clock())
        self.assertEqual([("move", -15, 40)], client.commands)

    def test_unsettled_head_cannot_extend_absolute_chunk_deadline(self):
        head, client, clock = self.scripted_head([
            (0., -12, False, {}), (.2, -13, False, {}), (.4, -14, False, {}),
        ], step_timeout_seconds=.65)
        with self.assertRaisesRegex(CameraHeadError, "timed out"):
            head.move_named("forward")
        self.assertAlmostEqual(.65, clock())
        self.assertEqual([("move", -15, 40), ("stop",)], client.commands)

    def test_exhausted_brick_retry_is_terminal_despite_partial_encoder_progress(self):
        for retry_limit in (0, 1):
            with self.subTest(retry_limit=retry_limit):
                head, client, clock = self.scripted_head([
                    (0., -4, False, {"tool_stall_retry_count": retry_limit,
                                      "tool_stall_retry_limit": retry_limit}),
                ])
                with self.assertRaisesRegex(CameraHeadError, "stalled"):
                    head.move_named("forward")
                self.assertEqual([("move", -15, 40), ("stop",)], client.commands)

    def test_terminal_brick_stop_is_not_hidden_by_position_tolerance(self):
        for reason in ("tool-stalled-after-retry", "tool-move-timeout", "tool-stall-retry-failure"):
            with self.subTest(reason=reason):
                head, client, clock = self.scripted_head([
                    (0., -12, False, {"last_stop_reason": reason}),
                ])
                with self.assertRaisesRegex(CameraHeadError, reason):
                    head.move_named("forward")
                self.assertEqual([("move", -15, 40), ("stop",)], client.commands)
                self.assertEqual(0., clock())

    def test_successful_brick_retry_can_settle_inside_configured_tolerance(self):
        head, client, clock = self.scripted_head([
            (0., -11, False, {"tool_stall_retry_count": 1, "tool_stall_retry_limit": 1,
                              "last_stop_reason": "tool-stall-retry"}),
        ])
        self.assertEqual(-11, head.move_named("forward")["position"])
        self.assertEqual([("move", -15, 40)], client.commands)

    def test_legacy_boost_shares_original_chunk_deadline(self):
        clock = FakeClock()
        client = LiftingNeedsBoostClient(position=0)
        head = CameraHeadController(
            client, calibrated=True, forward_position=-15, down_position=0,
            settle_tolerance=4, step_timeout_seconds=.6, clock=clock, sleeper=clock.sleep)
        with self.assertRaisesRegex(CameraHeadError, "timed out"):
            head.move_named("forward")
        self.assertAlmostEqual(.6, clock())
        self.assertEqual([("move", -15, 40), ("stop",), ("move", -15, 1500), ("stop",)], client.commands)

    def test_cancellation_before_named_movement_sends_no_motor_command(self):
        head, client, clock = self.scripted_head([], cancelled=lambda: True)
        with self.assertRaisesRegex(CameraHeadError, "cancelled"):
            head.move_named("forward")
        self.assertEqual([("stop",)], client.commands)

    def test_progress_callback_can_cancel_during_settlement_without_next_chunk(self):
        cancelled = []
        progress = []
        head, client, clock = self.scripted_head([(0., -15, False, {})])
        head.forward_position = -30
        head.cancelled = lambda: bool(cancelled)

        def on_progress(status):
            progress.append((clock(), status["motors"]["tool"]["position"]))
            if clock() >= .2:
                cancelled.append(True)

        head.progress_callback = on_progress
        with self.assertRaisesRegex(CameraHeadError, "cancelled"):
            head.move_named("forward")
        self.assertEqual([("move", -15, 40), ("stop",)], client.commands)
        self.assertEqual(3, len(progress))
        self.assertAlmostEqual(.2, clock())

    def test_zero_is_for_calibration_only(self):
        client = FakeClient(position=50)
        uncalibrated = CameraHeadController(client)
        calibrated = CameraHeadController(client, calibrated=True)

        uncalibrated.execute("zero")
        with self.assertRaises(CameraHeadError):
            calibrated.execute("zero")

    def test_home_is_available_after_reboot(self):
        client = FakeClient(position=50)
        head = CameraHeadController(client, calibrated=True)

        head.execute("home")

        self.assertEqual([("home", 300)], client.commands)

    def test_vision_calibration_can_replace_named_positions_for_this_boot(self):
        client = FakeClient(position=0)
        clock = FakeClock()
        head = CameraHeadController(
            client,
            calibrated=True,
            forward_position=-30,
            down_position=0,
            minimum_position=-150,
            maximum_position=0,
            clock=clock,
            sleeper=clock.sleep,
        )

        result = head.execute(
            {"action": "set_runtime_positions", "forward": -75, "down": 0}
        )
        head.execute("look_forward")

        self.assertEqual(-75, result["forward_position"])
        self.assertTrue(head.calibrated)
        self.assertEqual(
            [
                ("move", -15, 40),
                ("move", -30, 40),
                ("move", -45, 40),
                ("move", -60, 40),
                ("move", -75, 40),
            ],
            client.commands,
        )

        head.execute("invalidate_calibration")
        self.assertFalse(head.calibrated)
        with self.assertRaises(CameraHeadError):
            head.execute("look_forward")

    def test_floor_person_and_face_aliases_use_named_positions(self):
        client = FakeClient(position=0)
        clock = FakeClock()
        head = CameraHeadController(
            client,
            calibrated=True,
            forward_position=-30,
            down_position=0,
            up_position=-60,
            clock=clock,
            sleeper=clock.sleep,
        )

        head.execute("look_person")
        head.execute("look_face")
        head.execute("look_floor")

        self.assertEqual(0, client.position)
        self.assertIn(("move", -60, 40), client.commands)

    def test_low_camera_can_share_floor_and_person_position(self):
        client = FakeClient(position=0)
        head = CameraHeadController(
            client,
            minimum_position=-54,
            maximum_position=0,
            forward_position=0,
            down_position=0,
            up_position=-54,
        )

        result = head.set_runtime_positions(0, 0, -54)
        description = head.describe(client.status())

        self.assertEqual(0, result["forward_position"])
        self.assertTrue(head.calibrated)
        self.assertEqual("floor_person", description["named_position"])

    def test_runtime_positions_remain_within_mechanical_limits(self):
        head = CameraHeadController(
            FakeClient(),
            minimum_position=-150,
            maximum_position=0,
        )

        with self.assertRaises(CameraHeadError):
            head.set_runtime_positions(-151, 0)
        with self.assertRaises(CameraHeadError):
            head.set_runtime_positions(-40, -40, -40)

    def test_named_move_fails_immediately_when_loaded_head_does_not_progress(self):
        clock = FakeClock()
        head = CameraHeadController(
            StalledClient(position=0),
            calibrated=True,
            forward_position=-30,
            down_position=0,
            minimum_position=-150,
            maximum_position=0,
            clock=clock,
            sleeper=clock.sleep,
        )

        with self.assertRaisesRegex(CameraHeadError, "stalled"):
            head.execute("look_forward")

    def test_named_move_stops_motor_when_running_status_never_clears(self):
        clock = FakeClock()
        client = MovingClient(position=0)
        head = CameraHeadController(
            client,
            calibrated=True,
            forward_position=-30,
            down_position=0,
            minimum_position=-150,
            maximum_position=0,
            step_timeout_seconds=0.25,
            clock=clock,
            sleeper=clock.sleep,
        )

        with self.assertRaisesRegex(CameraHeadError, "timed out"):
            head.execute("look_forward")

        self.assertEqual(("stop",), client.commands[-1])

    def test_retained_position_can_be_acknowledged_after_reboot(self):
        client = FakeClient(position=63)
        head = CameraHeadController(client, calibrated=True)

        head.execute("acknowledge_position")

        self.assertEqual([("acknowledge_position",)], client.commands)

    def test_description_reports_named_position(self):
        client = FakeClient(position=-74)
        head = CameraHeadController(
            client, calibrated=True, forward_position=0, down_position=-75
        )

        description = head.describe(client.status())

        self.assertEqual("down", description["named_position"])
        self.assertEqual(-74, description["position"])

    def test_description_exposes_bounded_retry_state(self):
        client = FakeClient(position=-20)
        head = CameraHeadController(client)
        ev3_status = client.status()
        ev3_status.update(
            {
                "tool_target_position": -35,
                "tool_stall_retry_count": 1,
                "tool_stall_retry_limit": 1,
                "tool_stall_retry_ms": 1000,
            }
        )

        description = head.describe(ev3_status)

        self.assertEqual(-35, description["target_position"])
        self.assertEqual(1, description["stall_retry_count"])
        self.assertEqual(1, description["stall_retry_limit"])
        self.assertEqual(1000, description["stall_retry_ms"])


if __name__ == "__main__":
    unittest.main()
