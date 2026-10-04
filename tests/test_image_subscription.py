import unittest
from unittest.mock import Mock
from robot.jetson.perception.image_subscription import ReconnectingImageSubscription


class ImageSubscriptionTests(unittest.TestCase):
    def test_default_transport_preserves_an_explicit_profile(self):
        import os
        from unittest.mock import patch
        from robot.jetson.perception.image_subscription import configure_perception_transport
        with patch.dict(os.environ, {}, clear=True):
            configure_perception_transport()
            self.assertTrue(os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'].endswith('perception_transport.xml'))
        with patch.dict(os.environ, {'FASTDDS_DEFAULT_PROFILES_FILE': '/custom.xml'}, clear=True):
            configure_perception_transport()
            self.assertNotIn('FASTRTPS_DEFAULT_PROFILES_FILE', os.environ)

    def test_missing_person_feed_uses_capped_full_frame_face_detection(self):
        import ast
        from pathlib import Path
        from types import SimpleNamespace as NS
        tree = ast.parse(Path('robot/jetson/perception/face_detector.py').read_text())
        method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'on_inference_timer')
        scope = dict(time=NS(monotonic=lambda:100.), STATE_DETECTING='detecting',
                     STATE_STALE_INPUT='stale', REJECT_STALE_FRAME='stale',
                     PendingBatch=lambda frame, regions:NS(frame=frame, regions=regions),
                     frame_age_seconds=lambda *a:.1, is_frame_too_old=lambda age,limit:age>limit,
                     select_person_regions=Mock(return_value=[NS(width=640,height=480)]),
                     map_face_to_source=lambda candidate,*a:candidate, select_faces=lambda faces,*a:faces)
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<face-fallback>', 'exec'), scope)
        frame = NS(image=NS(shape=(480,640,3)), stamp=NS(sec=99,nanosec=900000000))
        node = NS(health=Mock(), config=NS(frame_timeout_sec=2.,enable_full_frame_fallback=True,
                  full_frame_fallback_rate_hz=3.,max_frame_age_sec=.5, minimum_person_roi_pixels=24,
                  nms_iou_threshold=.3,max_faces=3), slot=Mock(), matcher=Mock(),
                  person_input_is_stale=lambda now:True, latest_frame=frame,
                  _last_full_frame_fallback_at=0., set_state=lambda state:False,
                  get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_000_000_000)),
                  no_person_batches=0,full_frame_fallbacks_run=0,detector=Mock(),
                  publish_detections=Mock(),publish_annotated=Mock(),record_inference_error=Mock())
        node.health.input_is_stale.return_value=False
        node.slot.take.return_value=None
        node.detector.infer.return_value=['visible-face']
        infer=scope['on_inference_timer']
        infer(node)
        node.publish_detections.assert_called_once_with(['visible-face'],frame)
        node.detector.infer.assert_called_once_with(frame.image)
        infer(node)  # Same instant: the fallback must respect its 3 Hz cap.
        self.assertEqual(1,node.detector.infer.call_count)
        node.health.input_is_stale.return_value=True
        infer(node)
        self.assertEqual(1,node.detector.infer.call_count)
        node.slot.clear.assert_called_once_with()

    def test_face_loading_status_publishes_before_subscriptions_exist(self):
        import ast,json
        from pathlib import Path
        from types import SimpleNamespace
        from robot.jetson.perception.face_config import PARAMETER_DEFAULTS
        tree=ast.parse(Path('robot/jetson/perception/face_detector.py').read_text())
        method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='publish_status')
        scope=dict(time=SimpleNamespace(monotonic=lambda:100.),rclpy=SimpleNamespace(ok=lambda:True),
                   String=SimpleNamespace,json=json)
        exec(compile(ast.Module(body=[method],type_ignores=[]),'<face-startup-status>','exec'),scope)
        node=SimpleNamespace(config=SimpleNamespace(**PARAMETER_DEFAULTS),detector=None,
             health=Mock(recent_person_count=0),slot=SimpleNamespace(dropped=0),
             matcher=SimpleNamespace(evicted=0,left_evicted=0,right_evicted=0,left_depth=0,right_depth=0),
             status_publisher=Mock(),_last_person_message_time=None)
        for name in ('person_messages_received','matched_person_messages','person_rois_processed',
                     'no_person_batches','full_frame_fallbacks_run'):
            setattr(node,name,0)
        node.health.status.side_effect=lambda *args,**kw:dict(kw['extra'])
        scope['publish_status'](node)
        payload=json.loads(node.status_publisher.publish.call_args.args[0].data)
        self.assertEqual(0,payload['person_subscription_reconnects'])
        self.assertIsNone(payload['last_person_message_age_sec'])

    def test_face_detector_recovers_person_feed_even_when_camera_is_healthy(self):
        import ast
        from pathlib import Path
        from types import SimpleNamespace
        tree = ast.parse(Path('robot/jetson/perception/face_detector.py').read_text())
        method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'on_status_timer')
        scope = dict(time=SimpleNamespace(monotonic=lambda:100.),STATE_DETECTING='detecting')
        exec(compile(ast.Module(body=[method],type_ignores=[]),'<face-feed-recovery>','exec'),scope)
        node=SimpleNamespace(image_input=Mock(),person_input=Mock(),health=Mock(state='stale_input'),
                             _last_person_message_time=80.,matcher=Mock(),slot=Mock(),
                             get_logger=Mock(),publish_status=Mock())
        node.health.seconds_since_frame.return_value=.02
        node.image_input.refresh_if_stale.return_value=False
        node.person_input.refresh_if_stale.return_value=True
        scope['on_status_timer'](node)
        node.person_input.refresh_if_stale.assert_called_once_with(100.,20.)
        node.matcher.clear.assert_called_once_with()
        node.slot.clear.assert_called_once_with()
        node.publish_status.assert_called_once_with()

    def test_idle_subscription_reconnects_and_keeps_callback_and_qos(self):
        node = Mock()
        node.create_subscription.side_effect = ['old', 'new']
        stream = ReconnectingImageSubscription(node, 'Image', '/camera/image_raw', 'callback', 'qos')
        started = stream.last_attempt
        self.assertFalse(stream.refresh_if_stale(started + 9, None))
        self.assertTrue(stream.refresh_if_stale(started + 10, 100))
        node.destroy_subscription.assert_called_once_with('old')
        self.assertEqual('new', stream.subscription)
        self.assertEqual(node.create_subscription.call_args_list[0], node.create_subscription.call_args_list[1])
        self.assertEqual(1, stream.reconnects)
        self.assertFalse(stream.refresh_if_stale(started + 11, 101))

    def test_healthy_stream_never_recreated(self):
        node = Mock()
        stream = ReconnectingImageSubscription(node, 'Image', 'topic', 'callback', 'qos')
        self.assertFalse(stream.refresh_if_stale(stream.last_attempt + 100, .2))
        node.create_subscription.assert_called_once()
        node.destroy_subscription.assert_not_called()

    def test_failed_reconnect_retains_previous_subscription_and_waits_before_retry(self):
        node = Mock()
        node.create_subscription.side_effect = ['old', RuntimeError('unavailable'), 'new']
        stream = ReconnectingImageSubscription(node, 'Image', 'topic', 'callback', 'qos')
        started = stream.last_attempt
        self.assertFalse(stream.refresh_if_stale(started + 10, None))
        self.assertEqual('old', stream.subscription)
        self.assertEqual('unavailable', stream.last_error)
        node.destroy_subscription.assert_not_called()
        self.assertFalse(stream.refresh_if_stale(started + 11, None))
        self.assertTrue(stream.refresh_if_stale(started + 20, None))


if __name__ == '__main__':
    unittest.main()
