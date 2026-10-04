"""TCP ownership across head progress, explicit stop, and connection close."""

import threading
import unittest

from robot.jetson.ev3_bridge.ev3_client import Ev3Client
from tests.test_ev3_client import FakeSocket


class GatedSocket(FakeSocket):
    def __init__(self):
        super().__init__()
        self.receiving = threading.Event()
        self.release = threading.Event()

    def recv(self, size):
        if not self.receiving.is_set():
            self.receiving.set()
            if not self.release.wait(1.0):
                raise OSError('test response was not released')
        return super().recv(size)


class Ev3ClientSerializationTests(unittest.TestCase):
    def setUp(self):
        self.socket = GatedSocket()
        self.client = Ev3Client(socket_factory=lambda *args: self.socket)
        self.threads, self.results, self.errors = [], {}, []

    def tearDown(self):
        self.socket.release.set()
        for thread in self.threads:
            thread.join(1.0)
            self.assertFalse(thread.is_alive())
        self.client.close()

    def start(self, name, operation):
        entered, finished = threading.Event(), threading.Event()

        def run():
            entered.set()
            try:
                self.results[name] = operation()
            except Exception as exc:
                self.errors.append(exc)
            finally:
                finished.set()

        thread = threading.Thread(target=run, daemon=True)
        self.threads.append(thread)
        thread.start()
        self.assertTrue(entered.wait(1.0))
        return finished

    def test_stop_waits_for_complete_status_response_and_cannot_steal_it(self):
        status_done = self.start('status', self.client.status)
        self.assertTrue(self.socket.receiving.wait(1.0))
        stop_done = self.start('stop', self.client.stop)
        self.assertFalse(stop_done.wait(.01))
        self.assertEqual(['status'], [command['command'] for command in self.socket.sent])
        self.socket.release.set()
        self.assertTrue(status_done.wait(1.0))
        self.assertTrue(stop_done.wait(1.0))
        self.assertFalse(self.errors)
        self.assertIn('motion_active', self.results['status'])
        self.assertNotIn('stopped', self.results['status'])
        self.assertTrue(self.results['stop']['stopped'])
        self.assertEqual(['status', 'stop'], [command['command'] for command in self.socket.sent])

    def test_close_waits_for_inflight_response_then_discards_connection(self):
        status_done = self.start('status', self.client.status)
        self.assertTrue(self.socket.receiving.wait(1.0))
        close_done = self.start('close', self.client.close)
        self.assertFalse(close_done.wait(.01))
        self.assertFalse(self.socket.closed)
        self.socket.release.set()
        self.assertTrue(status_done.wait(1.0))
        self.assertTrue(close_done.wait(1.0))
        self.assertFalse(self.errors)
        self.assertIn('motion_active', self.results['status'])
        self.assertTrue(self.socket.closed)
        self.assertIsNone(self.client.socket)
        self.assertEqual(b'', self.client.receive_buffer)


if __name__ == '__main__':
    unittest.main()
