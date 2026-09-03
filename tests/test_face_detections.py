import unittest

from robot.jetson.perception.face_detections import FaceDetection
from robot.jetson.perception.face_detections import map_face_to_source
from robot.jetson.perception.face_detections import select_faces
from robot.jetson.perception.face_sync import PersonRegion


class FaceMappingTests(unittest.TestCase):
    def test_box_and_landmarks_map_from_crop_to_source(self):
        candidate = (64, 128, 320, 384, 0.9, ((128, 192), (256, 192)))
        region = PersonRegion(100, 50, 420, 370)

        face = map_face_to_source(candidate, region, 640, 640, 640, 480)

        self.assertEqual((132.0, 114.0, 260.0, 242.0), (face.x1, face.y1, face.x2, face.y2))
        self.assertEqual(((164.0, 146.0), (228.0, 146.0)), face.landmarks)
        self.assertEqual("face", face.label)

    def test_mapping_clips_a_box_to_the_camera_frame(self):
        face = map_face_to_source(
            (-100, -100, 900, 900, 0.8, ()),
            PersonRegion(0, 0, 640, 480),
            640,
            640,
            640,
            480,
        )

        self.assertEqual((0.0, 0.0, 640.0, 480.0), (face.x1, face.y1, face.x2, face.y2))


class GlobalFaceSelectionTests(unittest.TestCase):
    def test_overlapping_person_crops_do_not_duplicate_a_face(self):
        faces = [
            FaceDetection(10, 10, 110, 110, 0.95),
            FaceDetection(12, 12, 112, 112, 0.85),
        ]

        selected = select_faces(faces, 0.3, 20)

        self.assertEqual(1, len(selected))
        self.assertAlmostEqual(0.95, selected[0].score)

    def test_face_count_is_bounded(self):
        faces = [FaceDetection(i * 20, 0, i * 20 + 10, 10, 0.9) for i in range(10)]

        self.assertEqual(3, len(select_faces(faces, 0.3, 3)))
