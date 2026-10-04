"""Restart a stuck camera worker without restarting inference or the bridge."""
import os
import threading
import time


class CameraRestartWatchdog:
    def __init__(self, last_frame_time, timeout=12., exit_process=os._exit,
                 clock=time.monotonic):
        self.last_frame_time = last_frame_time
        self.timeout = float(timeout)
        self.exit_process = exit_process
        self.clock = clock
        self.started_at = clock()
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self.run, name='camera-recovery', daemon=True)

    def check(self):
        if self.stopped.is_set():
            return False
        last = self.last_frame_time()
        age = self.clock() - (self.started_at if last is None else last)
        if age < self.timeout:
            return False
        self.stopped.set()
        print('Camera produced no fresh frames for {:.1f}s; requesting managed restart'.format(age),
              flush=True)
        # Capture.read/release can hang, so this runs outside the ROS callback.
        # A nonzero exit asks the existing Restart=on-failure unit to recover.
        self.exit_process(75)
        return True

    def start(self):
        self.thread.start()

    def run(self):
        while not self.stopped.wait(.5):
            if self.check():
                return

    def close(self):
        self.stopped.set()
