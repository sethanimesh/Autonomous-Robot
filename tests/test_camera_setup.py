import unittest
from robot.jetson.ev3_bridge.camera_head import CameraHeadController, CameraHeadError
from tests.test_camera_head import FakeClient

class ManualCameraTests(unittest.TestCase):
    def test_automatic_upper_margin_leaves_settling_space_but_manual_still_works(self):
        c,h=self.make();c.position=-45;h.approved_reference_id="test-reference";h.upper_target_margin=3
        self.assertEqual(-51,h.minimum_target_position)
        h.jog(-15);self.assertEqual(-51,c.position)
        with self.assertRaises(ValueError):h.jog_to(-54)
        h.manual_jog("test-reference",-51,-56)
        self.assertEqual(-56,c.position)

    def make(self):
        c=FakeClient(-12)
        h=CameraHeadController(c, minimum_position=-54, maximum_position=-21,
            forward_position=-29,down_position=-24,up_position=-42,
            lower_target_margin=3,settle_tolerance=3,speed=300,
            approved_reference_id='old',require_approved_reference=True)
        return c,h

    def test_existing_button_can_move_beyond_old_limits_after_reboot(self):
        c,h=self.make();h.manual_jog('test-reference',-12,3)
        self.assertEqual([('move',3,300)],c.commands)
        self.assertIsNone(h.observe_status(c.status()))
        self.assertEqual('old',h.approved_reference_id)
        self.assertFalse(h.calibrated)
        self.assertTrue(h.describe(c.status())['manual_override'])
        with self.assertRaises(CameraHeadError):h.move_named('down')

    def test_unhomed_manual_movement_acknowledges_without_zero(self):
        c,h=self.make();original=c.status;homed=[False]
        c.status=lambda:dict(original(),tool_homed=homed[0])
        def acknowledge():homed[0]=True;c.commands.append(('acknowledge_position',))
        c.acknowledge_tool_position=acknowledge
        h.manual_jog('test-reference',-12,-7)
        self.assertEqual([('acknowledge_position',),('move',-7,300)],c.commands)

    def test_stale_button_cannot_stack_unseen_travel(self):
        c,h=self.make();h.manual_jog('test-reference',-12,-7)
        with self.assertRaises(CameraHeadError):h.manual_jog('test-reference',-12,-7)
        self.assertEqual(1,len(c.commands))

    def test_manual_control_also_overrides_matching_automatic_limits(self):
        c,h=self.make();h.approved_reference_id='test-reference'
        h.manual_jog('test-reference',-12,-7)
        self.assertEqual([('move',-7,300)],c.commands)

    def test_confirm_lower_exits_override_without_motion(self):
        c,h=self.make();h.manual_jog('test-reference',-12,-7)
        h.reference_current_lower('test-reference',-7,True)
        self.assertIsNone(h.manual_reference_id)
        self.assertEqual((-40,-7,-10),(h.minimum_position,h.maximum_position,h.down_position))
        self.assertEqual([('move',-7,300),('acknowledge_position',)],c.commands)

    def test_reference_loss_cancels_manual_override(self):
        c,h=self.make();h.manual_jog('test-reference',-12,-7)
        changed=dict(c.status(),tool_reference_id='new')
        self.assertIsNotNone(h.observe_status(changed));self.assertIsNone(h.manual_reference_id)
