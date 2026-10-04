import json
import os
import tempfile
import unittest

from robot.ev3.server.ev3_server import Ev3JsonServer, MotorController
from tests.test_ev3_server import make_controller


class ReferenceRecoveryTests(unittest.TestCase):
    def test_only_same_boot_and_motor_can_restore_without_changing_encoder(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'reference.json')
            _, motors = make_controller()
            identity = dict(boot_id='boot-a', inode=17, device='motor0', port='outA')
            motors['tool'].reference_identity = lambda: dict(identity)
            motors['tool'].position = -22
            first = MotorController(motors, reference_path=path)
            first.acknowledge_tool_position()
            reference = first.tool_reference_id
            second = MotorController(motors, reference_path=path)
            self.assertTrue(second.tool_homed)
            self.assertEqual(reference, second.tool_reference_id)
            self.assertEqual(-22, motors['tool'].position)
            self.assertEqual([], motors['tool'].move_history)
            for key, value in [('boot_id', 'boot-b'), ('inode', 18)]:
                old = identity[key]
                identity[key] = value
                changed = MotorController(motors, reference_path=path)
                self.assertFalse(changed.tool_homed)
                self.assertNotEqual(reference, changed.tool_reference_id)
                identity[key] = old

    def test_invalidated_or_corrupt_record_does_not_enable_motion(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'reference.json')
            _, motors = make_controller()
            motors['tool'].reference_identity = lambda: {'boot_id': 'boot-a', 'inode': 1}
            first = MotorController(motors, reference_path=path)
            first.acknowledge_tool_position()
            first.tool_homed = False
            first._save_reference()
            self.assertFalse(MotorController(motors, reference_path=path).tool_homed)
            with open(path, 'w') as handle:
                handle.write('{')
            self.assertFalse(MotorController(motors, reference_path=path).tool_homed)

    def test_coordinate_reset_invalidates_saved_reference_before_motor_write(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'reference.json')
            _, motors = make_controller()
            motors['tool'].reference_identity = lambda: {'boot_id': 'boot-a', 'inode': 1}
            first = MotorController(motors, reference_path=path)
            first.acknowledge_tool_position()
            old = first.tool_reference_id
            def fail_write(position):
                with open(path) as handle:
                    saved = json.load(handle)
                self.assertFalse(saved['homed'])
                self.assertNotEqual(old, saved['reference_id'])
                raise RuntimeError('simulated motor failure')
            motors['tool'].set_position = fail_write
            with self.assertRaises(RuntimeError):
                first.zero_tool()
            self.assertFalse(MotorController(motors, reference_path=path).tool_homed)

    def test_connection_reset_and_broken_pipe_do_not_erase_reference(self):
        for failure in ('recv', 'send'):
            controller, motors = make_controller()
            controller.acknowledge_tool_position()
            original = controller.tool_reference_id
            class Connection:
                closed = False
                def settimeout(self, value):
                    pass
                def recv(self, size):
                    if failure == 'recv':
                        raise ConnectionResetError(104, 'peer reset')
                    return b'{"cmd":"status"}\n'
                def sendall(self, payload):
                    raise BrokenPipeError(32, 'peer closed')
                def close(self):
                    self.closed = True
            connection = Connection()
            Ev3JsonServer(controller)._handle_client(connection)
            self.assertTrue(connection.closed)
            self.assertTrue(controller.tool_homed)
            self.assertEqual(original, controller.tool_reference_id)
            self.assertFalse(controller.motion_active)
            self.assertEqual([], motors['tool'].move_history)
