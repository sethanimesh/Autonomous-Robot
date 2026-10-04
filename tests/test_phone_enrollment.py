"""Offline enrollment isolation and profile binding; no ROS or motor startup."""
import ast
import copy
import json
import http.client
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import os
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
import uuid

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
from robot.jetson.perception.face_detections import FaceDetection, select_faces
from robot.jetson.perception.recognition_core import cosine_similarity, TargetStore
from robot.jetson.perception.family_store import FamilyTargetStore

SOURCE = (ROOT / 'robot/jetson/perception/enrollment_console.py').read_text()
TREE = ast.parse(SOURCE)
NODE = next(n for n in TREE.body if isinstance(n, ast.ClassDef) and n.name == 'EnrollmentNode')
METHODS = ('status', 'start', 'enrollment_session', 'cancel', 'upload', 'finish', 'process_pair')
NAMESPACE = dict(BaseHTTPRequestHandler=BaseHTTPRequestHandler, json=json, manual_camera_control_available=lambda: True, os=os, time=time, uuid=uuid, numpy=np, cv2=cv2, FaceDetection=FaceDetection,
                 select_faces=select_faces, cosine_similarity=cosine_similarity,
                 classify_pose=lambda landmarks: 'center', MAX_UPLOAD_BYTES=10*1024*1024,
                 MINIMUM_TOTAL=10, MAXIMUM_SAMPLES=18, REQUIRED={'center': 3, 'left': 2, 'right': 2})
DEFS = [copy.deepcopy(n) for n in TREE.body if
        (isinstance(n, ast.ClassDef) and n.name in ('EnrollmentSession', 'Handler')) or
        (isinstance(n, ast.FunctionDef) and n.name == 'validate_mission_profile')]
DEFS.append(ast.ClassDef(name='Harness', bases=[], keywords=[], decorator_list=[],
                        body=[copy.deepcopy(n) for n in NODE.body if isinstance(n, ast.FunctionDef) and n.name in METHODS]))
MODULE = ast.fix_missing_locations(ast.Module(body=DEFS, type_ignores=[]))
exec(compile(MODULE, '<enrollment-test>', 'exec'), NAMESPACE)
Session, Harness = NAMESPACE['EnrollmentSession'], NAMESPACE['Harness']


class PhoneEnrollmentTests(unittest.TestCase):
    def setUp(self):
        self.node = Harness()
        self.node.lock = threading.RLock()
        self.node.action_lock = threading.Lock()
        self.node.runtime_lock = threading.Lock()
        self.node.session = None
        self.node.mission = SimpleNamespace(status=lambda: {'running': False})
        self.node.manual_drive = SimpleNamespace(status=lambda: {'enabled': False})
        self.node.store = SimpleNamespace(family=SimpleNamespace(load=lambda pid: {'id': pid} if pid == 'mom' else None), save=Mock())
        self.node.status = lambda: {'session_id': self.node.session.session_id if self.node.session else None}
        self.node.store_preview = Mock()
        self.node.recognizer = SimpleNamespace(embedding=Mock(return_value=([1., 0., 0.], np.zeros((112,112,3), np.uint8))))
        self.landmarks = [(90.,90.),(150.,90.),(120.,120.),(98.,150.),(140.,150.)]
        self.node.upload_detector = SimpleNamespace(infer=Mock(return_value=[(60.,50.,185.,190.,.98,self.landmarks)]))
        NAMESPACE['assess_quality'] = Mock(return_value=(True, '', {'blur': 100.}))
        self.photo = cv2.imencode('.jpg', np.full((240,320,3), 130, np.uint8))[1].tobytes()

    def start(self, source='phone', **extra):
        self.node.start(dict(label='Mom', consent=True, camera_source=source, **extra))
        return self.node.session

    def test_phone_enrollment_needs_no_robot_or_camera_status(self):
        session = self.start(profile_id='mom')
        self.assertEqual('phone', session.camera_source)
        self.assertEqual('mom', session.profile_id)
        result = self.node.upload(self.photo, session.session_id, 'phone')
        self.assertTrue(result['added'])
        self.assertEqual('phone', session.samples[0]['source'])
        self.node.store.save.assert_not_called()

    def test_robot_frames_do_not_sample_or_change_phone_guidance(self):
        session = self.start()
        before = session.message
        frame = SimpleNamespace(frame_id='camera', image=np.zeros((240,320,3), np.uint8))
        self.node.process_pair(frame, {'frame_id':'camera','faces':[]})
        self.assertEqual([], session.samples)
        self.assertEqual(before, session.message)
        self.node.recognizer.embedding.assert_not_called()
        self.node.store_preview.assert_called_once()

    def test_legacy_start_and_upload_default_to_robot(self):
        self.node.start({'label':'Mom','consent':True})
        self.assertEqual('robot', self.node.session.camera_source)
        self.assertTrue(self.node.upload(self.photo)['added'])
        self.assertEqual('upload', self.node.session.samples[0]['source'])

    def test_phone_session_and_source_are_checked_before_inference(self):
        session = self.start()
        for token in (None, '', 'previous-session'):
            with self.subTest(token=token), self.assertRaises(ValueError):
                self.node.upload(self.photo, token, 'phone')
        self.node.upload_detector.infer.assert_not_called()
        self.node.cancel(session.session_id)
        robot = self.start('robot')
        with self.assertRaises(ValueError):
            self.node.upload(self.photo, robot.session_id, 'phone')

    def test_late_phone_frame_cannot_join_replacement_enrollment(self):
        session = self.start()
        def cancel_during_inference(_image):
            self.node.cancel(session.session_id)
            self.start()
            return [(60.,50.,185.,190.,.98,self.landmarks)]
        self.node.upload_detector.infer.side_effect = cancel_during_inference
        with self.assertRaisesRegex(ValueError, 'cancelled'):
            self.node.upload(self.photo, session.session_id, 'phone')
        self.assertEqual([], self.node.session.samples)
        self.assertNotEqual(session.session_id, self.node.session.session_id)

    def test_late_cancel_and_finish_do_not_touch_new_session(self):
        old = self.start()
        self.node.cancel(old.session_id)
        current = self.start()
        for method in (self.node.cancel, self.node.finish):
            with self.assertRaisesRegex(ValueError, 'session has changed'):
                method(old.session_id)
            self.assertIs(current, self.node.session)
        self.node.store.save.assert_not_called()

    def test_cannot_start_over_active_enrollment(self):
        current = self.start()
        with self.assertRaisesRegex(ValueError, 'current enrollment'):
            self.start('robot')
        self.assertIs(current, self.node.session)

    def test_invalid_source_profile_and_motion_are_rejected(self):
        for extra in ({'camera_source':'bad'}, {'profile_id':'missing'}):
            with self.assertRaises(ValueError):
                self.node.start(dict(label='Mom',consent=True,**extra))
            self.assertIsNone(self.node.session)
        self.node.mission.status=lambda: {'running':True}
        with self.assertRaises(ValueError): self.start()
        self.node.mission.status=lambda: {'running':False}
        self.node.manual_drive.status=lambda: {'enabled':True}
        with self.assertRaises(ValueError): self.start()

    def test_quality_multiple_faces_and_invalid_image_are_rejected(self):
        session = self.start()
        with self.assertRaises(ValueError): self.node.upload(b'not a jpeg',session.session_id,'phone')
        NAMESPACE['assess_quality'].return_value=(False, 'too blurry', {})
        with self.assertRaisesRegex(ValueError,'too blurry'): self.node.upload(self.photo,session.session_id,'phone')
        self.node.upload_detector.infer.return_value.append((200.,50.,310.,190.,.98,[(x+140,y) for x,y in self.landmarks]))
        with self.assertRaisesRegex(ValueError,'exactly one'): self.node.upload(self.photo,session.session_id,'phone')
        self.assertEqual([],session.samples)

    def test_duplicates_and_other_people_do_not_advance_progress(self):
        session = self.start()
        self.assertTrue(self.node.upload(self.photo,session.session_id,'phone')['added'])
        self.assertFalse(self.node.upload(self.photo,session.session_id,'phone')['added'])
        self.node.recognizer.embedding.return_value=([0.,1.,0.], np.zeros((112,112,3), np.uint8))
        self.assertFalse(self.node.upload(self.photo,session.session_id,'phone')['added'])
        self.assertEqual(1,len(session.samples))

    def test_finish_requires_pose_coverage_and_does_not_save_early(self):
        session = self.start()
        session.samples=[dict(pose='center') for _ in range(10)]
        with self.assertRaisesRegex(ValueError,'front, left, and right'): self.node.finish(session.session_id)
        self.node.store.save.assert_not_called()
        session.samples=[dict(pose=p) for p in ['center']*6+['left']*2+['right']*2]
        self.assertTrue(session.ready)

    def test_finishing_add_and_update_preserves_other_profiles_across_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / 'target_person.json')
            self.node.args = SimpleNamespace(store_path=path)
            self.node.store = FamilyTargetStore(TargetStore(path, 'test-model', 'a'*64))
            self.node.cleanup_photo_directories = Mock()
            samples = [dict(pose=p, embedding=[1., 0.], source='phone', quality={})
                       for p in ['center']*6+['left']*2+['right']*2]
            dad = self.node.store.save('Dad', samples, 'embeddings')
            session = self.start()
            session.samples = samples
            self.node.finish(session.session_id)
            mom = self.node.store.load()
            self.assertIsNone(self.node.session)
            self.assertEqual('Mom', mom['label'])
            self.assertNotEqual(dad['profile_id'], mom['profile_id'])
            session = self.start('robot', profile_id=mom['profile_id'])
            session.samples = [dict(x, source='live') for x in samples]
            self.node.finish(session.session_id)
            reopened = FamilyTargetStore(TargetStore(path, 'test-model', 'a'*64))
            updated = reopened.load()
            self.assertEqual(mom['profile_id'], updated['profile_id'])
            self.assertNotEqual(mom['revision'], updated['revision'])
            self.assertEqual(dad['revision'], reopened.family.load(dad['profile_id'])['revision'])
            self.assertEqual(2, len(reopened.family.profiles()))
            self.assertEqual('live', updated['samples'][0]['source'])

    def test_phone_http_contract_preserves_status_and_session_binding(self):
        # Exercise the real HTTP handler and status method with isolated inference.
        self.node.store.load = lambda: None
        self.node.store.family.profiles = lambda: []
        self.node.latest_frame_time = self.node.camera_head_status_time = None
        self.node.camera_head_status = None
        self.node.family_status = {}
        self.node.last_message = ''
        self.node.args = SimpleNamespace(model_name='test-model')
        self.node.recognizer.provider = 'test'
        self.node.runtime_safety = lambda: (False, False, False)
        del self.node.status
        # The dispatch table also references the unrelated control actions.
        for method in ('family_action', 'delete_target', 'jog_camera', 'save_camera_limit',
                       'restore_saved_camera_range', 'start_find_mission', 'stop_find_mission',
                       'enable_manual_drive', 'command_manual_drive', 'disable_manual_drive'):
            setattr(self.node, method, Mock(side_effect=AssertionError('Unexpected control call')))
        handler = type('TestHandler', (NAMESPACE['Handler'],), {'node': self.node})
        server = ThreadingHTTPServer(('127.0.0.1', 0), handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        def request(path, body, headers=None):
            conn = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=2)
            try:
                conn.request('POST', path, body, headers or {})
                response = conn.getresponse()
                return response.status, json.loads(response.read())
            finally: conn.close()
        try:
            payload = json.dumps(dict(label='Mom', consent=True, camera_source='phone'))
            code, _ = request('/api/enrollment/start', payload)
            self.assertEqual(403, code)
            headers = {'X-Echora-Action':'1','Content-Type':'application/json'}
            code, status = request('/api/enrollment/start', payload, headers)
            self.assertEqual(200, code)
            self.assertEqual('phone', status['enrollment_source'])
            self.assertEqual('Mom', status['enrollment_label'])
            self.assertFalse(status['camera_ready'])
            self.assertEqual('Look straight at the camera', status['guidance'])
            token = status['enrollment_session_id']
            code, _ = request('/api/enrollment/frame', self.photo, headers)
            self.assertEqual(400, code)
            code, result = request('/api/enrollment/frame', self.photo,
                dict(headers, **{'X-Echora-Enrollment':token,'Content-Type':'image/jpeg'}))
            self.assertEqual(200, code)
            self.assertTrue(result['added'])
            code, _ = request('/api/enrollment/finish', json.dumps({'session_id':token}), headers)
            self.assertEqual(400, code)
            code, status = request('/api/enrollment/cancel', json.dumps({'session_id':token}), headers)
            self.assertEqual(200, code)
            self.assertFalse(status['enrolling'])
            self.node.store.save.assert_not_called()
        finally:
            server.shutdown();server.server_close();thread.join(2)

    def test_mission_binds_to_both_identity_and_revision(self):
        validate=NAMESPACE['validate_mission_profile']
        target={'profile_id':'mom','revision':'r2'}
        validate({},target)
        validate({'profile_id':'mom','profile_revision':'r2'},target)
        for request in ({'profile_id':'dad','profile_revision':'r2'}, {'profile_id':'mom','profile_revision':'r1'}, {'profile_id':None,'profile_revision':None}):
            with self.assertRaisesRegex(ValueError,'selected profile changed'):validate(request,target)


if __name__ == '__main__': unittest.main()
