import math
import os
import tempfile
import unittest

from robot.jetson.perception.recognition_core import ConfirmationWindow, RecognitionError, TargetStore, aggregate_similarity, classify_pose, cosine_similarity, normalize_vector


class RecognitionMathTests(unittest.TestCase):
    def test_normalization_and_cosine(self):
        vector = normalize_vector([3, 4])
        self.assertAlmostEqual(1.0, math.sqrt(sum(value * value for value in vector)))
        self.assertAlmostEqual(1.0, cosine_similarity(vector, vector))
        self.assertAlmostEqual(0.0, cosine_similarity([1, 0], [0, 1]))

    def test_aggregate_prefers_best_view_but_requires_consensus(self):
        score = aggregate_similarity([1, 0], [[1, 0], [0.8, 0.6], [0, 1]])
        self.assertAlmostEqual(0.88, score)

    def test_bad_vectors_are_rejected(self):
        with self.assertRaises(RecognitionError):
            normalize_vector([0, 0])
        with self.assertRaises(RecognitionError):
            cosine_similarity([1], [1, 2])

    def test_pose_uses_nose_offset_from_eye_midpoint(self):
        base = [(10, 10), (30, 10)]
        self.assertEqual("left", classify_pose(base + [(16, 20), (13, 30), (27, 30)]))
        self.assertEqual("center", classify_pose(base + [(20, 20), (13, 30), (27, 30)]))
        self.assertEqual("right", classify_pose(base + [(24, 20), (13, 30), (27, 30)]))


class ConfirmationTests(unittest.TestCase):
    def test_three_of_five_confirms_and_window_recovers(self):
        window = ConfirmationWindow(5, 3, 0.45)
        for score in (0.7, 0.1, 0.6, 0.2):
            self.assertFalse(window.add(score))
        self.assertTrue(window.add(0.5))
        self.assertFalse(window.add(0.1))


class TargetStoreTests(unittest.TestCase):
    def test_store_is_atomic_private_and_model_bound(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "private", "target.json")
            store = TargetStore(path, "model", "a" * 64)
            store.save("Mom", [{"embedding": [3, 4], "pose": "center", "source": "live", "quality": {"blur": 99}}], "embeddings")
            loaded = store.load()
            self.assertEqual("Mom", loaded["label"])
            self.assertAlmostEqual(0.6, loaded["samples"][0]["embedding"][0])
            self.assertEqual(0o600, os.stat(path).st_mode & 0o777)
            with self.assertRaises(RecognitionError):
                TargetStore(path, "different", "a" * 64).load()

    def test_delete_is_idempotent(self):
        with tempfile.TemporaryDirectory() as directory:
            store = TargetStore(os.path.join(directory, "target.json"), "model", "a" * 64)
            self.assertFalse(store.delete())
            store.save("Target", [{"embedding": [1, 0]}], "embeddings")
            self.assertTrue(store.delete())
            self.assertFalse(store.delete())
