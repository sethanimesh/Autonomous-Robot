import unittest

from robot.jetson.mission.single_room import MissionAction
from robot.jetson.mission.single_room import MissionState
from robot.jetson.mission.single_room import SingleRoomMission
from robot.jetson.mission.single_room import TargetObservation
from robot.jetson.navigation.local_planner import RouteDecision


class SingleRoomMissionTests(unittest.TestCase):
    def target(self, confirmed=True, height=0.2, age=0.1):
        return TargetObservation(confirmed, age, height)

    def ready_mission(self):
        mission = SingleRoomMission()
        command = mission.start(target_enrolled=True, camera_ready=True)
        self.assertEqual(command.action, MissionAction.LOOK_FORWARD_AND_SCAN)
        return mission

    def test_missing_target_or_camera_stops(self):
        for enrolled, camera in ((False, True), (True, False)):
            mission = SingleRoomMission()
            self.assertEqual(
                mission.start(enrolled, camera).action, MissionAction.STOP
            )
            self.assertEqual(mission.state, MissionState.STOPPED)

    def test_unconfirmed_person_continues_scan(self):
        mission = self.ready_mission()
        command = mission.observe_target(self.target(confirmed=False))
        self.assertEqual(command.action, MissionAction.LOOK_FORWARD_AND_SCAN)
        self.assertEqual(mission.state, MissionState.SCANNING)

    def test_confirmed_target_requests_floor_check(self):
        mission = self.ready_mission()
        command = mission.observe_target(self.target())
        self.assertEqual(command.action, MissionAction.LOOK_DOWN_AND_CHECK_ROUTE)
        self.assertEqual(mission.state, MissionState.ROUTE_CHECK)

    def test_close_target_finishes_without_motion(self):
        mission = self.ready_mission()
        command = mission.observe_target(self.target(height=0.30))
        self.assertEqual(command.action, MissionAction.ANNOUNCE_FOUND)
        self.assertEqual(mission.state, MissionState.FOUND)

    def test_stale_target_fails_closed(self):
        mission = self.ready_mission()
        command = mission.observe_target(self.target(age=1.0))
        self.assertEqual(command.action, MissionAction.STOP)
        self.assertEqual(mission.state, MissionState.STOPPED)

    def test_blocked_route_returns_to_scan(self):
        mission = self.ready_mission()
        mission.observe_target(self.target())
        command = mission.approve_route(RouteDecision(True, reason="blocked"))
        self.assertEqual(command.action, MissionAction.LOOK_FORWARD_AND_SCAN)
        self.assertEqual(mission.state, MissionState.SCANNING)

    def test_clear_route_emits_only_one_bounded_step(self):
        mission = self.ready_mission()
        mission.observe_target(self.target())
        command = mission.approve_route(
            RouteDecision(False, -30.0, 0.10, 0.5, "clear")
        )
        self.assertEqual(command.action, MissionAction.EXECUTE_BOUNDED_ROUTE)
        self.assertEqual(command.heading_degrees, -30.0)
        self.assertEqual(command.distance_m, 0.10)
        self.assertEqual(mission.state, MissionState.APPROACH_STEP)

    def test_step_must_reacquire_target(self):
        mission = self.ready_mission()
        mission.observe_target(self.target())
        mission.approve_route(RouteDecision(False, 0.0, 0.05, 0.5, "clear"))
        command = mission.finish_step(True)
        self.assertEqual(command.action, MissionAction.LOOK_FORWARD_AND_SCAN)
        self.assertEqual(mission.state, MissionState.VERIFY_TARGET)
        self.assertEqual(mission.approach_steps, 1)

    def test_failed_step_stops(self):
        mission = self.ready_mission()
        mission.observe_target(self.target())
        mission.approve_route(RouteDecision(False, 0.0, 0.05, 0.5, "clear"))
        self.assertEqual(mission.finish_step(False).action, MissionAction.STOP)


if __name__ == "__main__":
    unittest.main()
