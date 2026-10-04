"""Real-thread bridge command contracts, with no ROS or network connection."""

import ast
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock

from robot.jetson.ev3_bridge.camera_head import CameraHeadController, CameraHeadError
from robot.jetson.ev3_bridge.ev3_client import Ev3Client, Ev3ClientError
from tests.test_ev3_client import FakeSocket


def worker_classes():
    source = Path('robot/jetson/ev3_bridge/ros_node.py').read_text()
    tree = ast.parse(source)
    names = {'_head_action', '_HeadCommandClient', 'HeadCommandWorker', 'Ev3BridgeNode'}
    nodes = [node for node in tree.body if getattr(node, 'name', None) in names]
    scope = dict(json=json, threading=threading, time=time, Node=object,
                 CameraHeadError=CameraHeadError, Ev3ClientError=Ev3ClientError)
    exec(compile(ast.Module(body=nodes, type_ignores=[]), '<head-worker>', 'exec'), scope)
    return scope['HeadCommandWorker'], scope['Ev3BridgeNode']


class Client:
    def __init__(self):
        self.io_lock = threading.RLock()
        self.commands = []
        self.position = 0
        self.target = None
        self.moving = False
        self.retry_count = 0

    def status(self):
        with self.io_lock:
            return {'motors': {'tool': {'position': self.position, 'speed': 0}},
                    'tool_homed': True, 'tool_reference_id': 'reference',
                    'tool_motion_active': self.moving,
                    'tool_target_position': self.target,
                    'tool_stall_retry_count': self.retry_count,
                    'tool_stall_retry_limit': 1}

    def move_tool(self, position, speed):
        with self.io_lock:
            self.commands.append(('move', position, speed))
            self.position = self.target = position
            return {}

    def stop(self):
        with self.io_lock:
            self.commands.append(('stop',))
            self.moving = False
            return {}

    def drive(self, left, right):
        with self.io_lock:
            self.commands.append(('drive', left, right))
            return {}

    def close(self):
        with self.io_lock:
            self.commands.append(('close',))


class Controller:
    def __init__(self):
        self.actions = []
        self.invalidated = False
        self.handler = lambda payload: None

    def execute(self, payload):
        self.actions.append(payload)
        return self.handler(payload)

    def invalidate_calibration(self):
        self.invalidated = True


class HeadWorkerTests(unittest.TestCase):
    def test_final_status_timeout_recovers_saved_range_without_new_calibration(self):
        head = CameraHeadController(self.client, minimum_position=-20, maximum_position=30,
                                    forward_position=0, down_position=20, up_position=-10,
                                    settle_tolerance=2)
        head._apply_manual_limits(dict(version=1, reference_id='reference', upper=-20, lower=30))
        worker = self.start(controller=head, publisher=head.describe)
        original = self.client.status
        self.client.status = MagicMock(side_effect=Ev3ClientError('temporary timeout'))
        worker._publish_latest()
        self.assertFalse(head.calibrated)
        self.client.status = original
        worker._publish_latest()
        self.assertTrue(head.calibrated)
        self.assertEqual('operator_limits', head.calibration_source)

    def setUp(self):
        self.Worker, self.Bridge = worker_classes()
        self.client, self.controller = Client(), Controller()
        self.published, self.errors = [], []
        self.worker = None
        self.release = threading.Event()

    def tearDown(self):
        self.release.set()
        if self.worker is not None:
            self.assertTrue(self.worker.close(timeout=2.0))

    def start(self, controller=None, publisher=None):
        self.controller = controller or self.controller
        self.worker = self.Worker(self.client, self.controller,
                                  publisher or self.published.append, self.errors.append)
        return self.worker

    def idle(self):
        with self.worker.condition:
            self.assertTrue(self.worker.condition.wait_for(
                lambda: not self.worker.busy, timeout=2.0), 'head worker did not finish')

    def test_velocity_stream_does_not_keep_stop_worker_busy(self):
        entered = threading.Event()
        original_stop = self.client.stop
        def slow_stop():
            entered.set()
            self.release.wait(1.)
            return original_stop()
        self.client.stop = slow_stop
        self.start()
        node = self.Bridge.__new__(self.Bridge)
        node.head_worker = self.worker
        node._on_timer = MagicMock()
        self.worker.submit('stop')
        self.assertTrue(entered.wait(1.))
        for _ in range(20):
            node.on_cmd_vel(SimpleNamespace(linear=SimpleNamespace(x=0.), angular=SimpleNamespace(z=.4)))
        self.assertFalse(self.worker.stop_pending)
        self.release.set()
        self.idle()
        self.assertEqual([('stop',)], self.client.commands)
        node.on_timer()
        node._on_timer.assert_called_once_with()

    def test_actual_named_move_publishes_progress_before_worker_finishes(self):
        sleeping = threading.Event()
        clock = [0.]

        def sleep(seconds):
            sleeping.set()
            if not self.release.wait(1.0):
                raise AssertionError('test did not release the settlement poll')
            clock[0] += seconds

        head = CameraHeadController(self.client, calibrated=True,
                                    forward_position=-30, down_position=0,
                                    settle_tolerance=4, clock=lambda: clock[0], sleeper=sleep)
        self.start(head)
        self.assertTrue(self.worker.submit('look_forward'))
        self.assertTrue(sleeping.wait(1.0))
        self.assertTrue(self.worker.busy)
        self.assertEqual(-15, self.published[-1]['motors']['tool']['position'])
        self.assertEqual([('stop',), ('move', -15, 40)], self.client.commands)
        self.release.set()
        self.idle()
        self.assertEqual(-30, self.published[-1]['motors']['tool']['position'])
        self.assertFalse(self.errors)

    def test_replacement_keeps_only_latest_move_and_fences_cancelled_continuation(self):
        entered = threading.Event()

        def execute(payload):
            if payload == 'look_down':
                self.controller.client.move_tool(-15, 40)
                entered.set()
                self.release.wait(1.0)
                self.controller.client.move_tool(-30, 40)
            else:
                self.controller.client.move_tool(-45, 40)

        self.controller.handler = execute
        self.start()
        self.worker.submit('look_down')
        self.assertTrue(entered.wait(1.0))
        self.worker.submit('look_forward')
        self.worker.submit('look_up')
        self.release.set()
        self.idle()
        self.assertEqual(['look_down', 'look_up'], self.controller.actions)
        self.assertEqual([-15, -45], [command[1] for command in self.client.commands if command[0] == 'move'])
        self.assertTrue(any('cancelled' in error for error in self.errors))

    def test_stop_clears_pending_motion_and_metadata_cannot_displace_stop(self):
        entered = threading.Event()
        metadata_stops = []

        def execute(payload):
            if payload == 'look_down':
                entered.set()
                self.release.wait(1.0)
                self.controller.client.move_tool(-30, 40)
            else:
                metadata_stops.append(sum(command[0] == 'stop' for command in self.client.commands))

        self.controller.handler = execute
        self.start()
        self.worker.submit('look_down')
        self.assertTrue(entered.wait(1.0))
        before = sum(command[0] == 'stop' for command in self.client.commands)
        self.worker.submit('look_up')
        self.worker.submit('stop')
        self.worker.submit('use_saved_limits')
        self.release.set()
        self.idle()
        self.assertEqual(['look_down', 'use_saved_limits'], self.controller.actions)
        self.assertGreater(metadata_stops[0], before)
        self.assertFalse(any(command[0] == 'move' for command in self.client.commands))

    def test_cancelled_call_waiting_for_tcp_lock_cannot_send_a_motor_request(self):
        entered = threading.Event()

        def execute(payload):
            entered.set()
            self.release.wait(1.0)
            self.controller.client.move_tool(-15, 40)

        self.controller.handler = execute
        self.start()
        self.worker.submit('look_up')
        self.assertTrue(entered.wait(1.0))
        with self.client.io_lock:
            self.release.set()
            self.worker.submit('stop')
        self.idle()
        self.assertFalse(any(command[0] == 'move' for command in self.client.commands))

    def test_retry_preserves_active_target_and_brick_retry_budget(self):
        observed = []
        self.client.target, self.client.moving, self.client.retry_count = -15, True, 1
        self.controller.handler = lambda payload: observed.append(self.controller.client.status())
        self.start()
        self.worker.submit({'action': 'retry_jog', 'target': -15})
        self.idle()
        self.assertEqual([('drive', 0, 0)], self.client.commands)
        self.assertEqual(-15, observed[0]['tool_target_position'])
        self.assertTrue(observed[0]['tool_motion_active'])
        self.assertEqual(1, observed[0]['tool_stall_retry_count'])

    def test_rejected_metadata_does_not_interrupt_the_active_brick_target(self):
        self.client.moving = True

        def reject(payload):
            raise CameraHeadError('stale limit status')

        self.controller.handler = reject
        self.start()
        self.worker.submit('save_manual_limit')
        self.idle()
        self.assertEqual([('drive', 0, 0)], self.client.commands)
        self.assertTrue(self.client.moving)
        self.assertTrue(self.worker.thread.is_alive())

    def test_status_publication_exception_does_not_kill_worker(self):
        def broken_publish(status):
            raise ValueError('malformed telemetry')

        self.start(publisher=broken_publish)
        self.worker.submit('use_saved_limits')
        self.idle()
        self.assertTrue(self.worker.thread.is_alive())
        self.assertTrue(any('publication failed' in error for error in self.errors))
        self.worker.publish_status = self.published.append
        self.worker.submit('use_saved_limits')
        self.idle()
        self.assertTrue(self.published)

    def test_close_cancels_active_move_clears_pending_and_rejects_new_commands(self):
        entered = threading.Event()

        def execute(payload):
            entered.set()
            self.release.wait(1.0)
            self.controller.client.move_tool(-15, 40)

        self.controller.handler = execute
        self.start()
        self.worker.submit('look_down')
        self.assertTrue(entered.wait(1.0))
        self.worker.submit('look_up')
        self.assertFalse(self.worker.close(timeout=.01))
        self.assertFalse(self.worker.submit('look_forward'))
        self.release.set()
        self.assertTrue(self.worker.close(timeout=1.0))
        self.assertFalse(any(command[0] == 'move' for command in self.client.commands))

    def test_ros_timer_returns_without_io_while_named_head_worker_owns_controller(self):
        self.start()
        node = self.Bridge.__new__(self.Bridge)
        node.head_worker = self.worker
        node._on_timer = MagicMock()
        entered = threading.Event()

        def execute(payload):
            entered.set()
            self.release.wait(1.0)

        self.controller.handler = execute
        self.worker.submit('look_down')
        self.assertTrue(entered.wait(1.0))
        node.on_timer()
        node._on_timer.assert_not_called()
        self.release.set()
        self.idle()
        node.on_timer()
        node._on_timer.assert_called_once_with()

    def test_idle_ros_timer_publishes_status_through_actual_serialized_client(self):
        socket = FakeSocket()
        self.client = Ev3Client(socket_factory=lambda *args: socket)
        self.start()
        node = self.Bridge.__new__(self.Bridge)
        node.head_worker, node.client = self.worker, self.client
        node.camera_head = self.controller
        node.tick_count = 0
        node.last_command = node.last_command_time = None
        node.motion_active = False
        node.command_timeout_sec = .3
        node.publish_status = self.published.append
        node.get_logger = MagicMock()
        for _ in range(10):
            node.on_timer()
        self.assertEqual(10, node.tick_count)
        self.assertEqual(2, len(self.published))
        self.assertEqual(['status', 'status'], [command['command'] for command in socket.sent])
        self.assertFalse(self.worker.busy)

    def test_timeout_stop_publishes_full_feedback_not_stop_acknowledgement(self):
        node = self.Bridge.__new__(self.Bridge)
        node.client = self.client
        node.camera_head = self.controller
        node.motion_active = True
        node.publish_status = self.published.append
        node.get_logger = MagicMock()
        node.stop_for_reason('cmd_vel_timeout')
        self.assertEqual([('stop',)], self.client.commands)
        self.assertEqual(1, len(self.published))
        self.assertTrue(self.published[0]['tool_homed'])
        self.assertEqual('reference', self.published[0]['tool_reference_id'])
        self.assertEqual('cmd_vel_timeout', self.published[0]['bridge_stop_reason'])
        self.assertFalse(node.motion_active)
        self.assertFalse(self.controller.invalidated)

    def test_head_callback_queues_without_waiting_and_clears_previous_drive(self):
        self.start()
        node = self.Bridge.__new__(self.Bridge)
        node.head_worker = self.worker
        node.get_logger = MagicMock()
        node.last_command, node.last_command_time = (.1, 0.), 1.
        node.motion_active = True
        entered = threading.Event()
        self.controller.handler = lambda payload: (entered.set(), self.release.wait(1.0))
        node.on_camera_head_command(SimpleNamespace(data='look_up'))
        self.assertIsNone(node.last_command)
        self.assertIsNone(node.last_command_time)
        self.assertFalse(node.motion_active)
        self.assertTrue(entered.wait(1.0))
        node.on_cmd_vel(SimpleNamespace(linear=SimpleNamespace(x=.05), angular=SimpleNamespace(z=0.)))
        self.assertTrue(self.worker.cancelled())


if __name__ == '__main__':
    unittest.main()
