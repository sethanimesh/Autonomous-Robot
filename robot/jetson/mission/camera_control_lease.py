"""Cross-process ownership guard for camera-head commands on the Jetson."""

from contextlib import contextmanager
import fcntl
import os


DEFAULT_CAMERA_CONTROL_LOCK = "/home/animesh/echora/data/camera-head-control.lock"
CAMERA_CONTROL_FD_ENV = "ECHORA_CAMERA_CONTROL_FD"


class CameraControlBusy(RuntimeError):
    """Raised when another process owns the camera head."""


class CameraControlLease(object):
    """Advisory shared/exclusive lock released automatically on process exit."""

    def __init__(self, exclusive, path=DEFAULT_CAMERA_CONTROL_LOCK):
        self.exclusive = bool(exclusive)
        self.path = path
        self.handle = None
        self.inherited = False

    def acquire(self):
        if self.handle is not None:
            return self
        inherited = os.environ.get(CAMERA_CONTROL_FD_ENV)
        if self.exclusive and inherited is not None:
            try:
                descriptor = int(inherited)
                actual = os.fstat(descriptor)
                expected = os.stat(self.path)
                if (actual.st_dev, actual.st_ino) != (expected.st_dev, expected.st_ino):
                    raise ValueError("inherited camera lock belongs to another file")
                fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
                self.handle = os.fdopen(os.dup(descriptor), "a+")
                self.inherited = True
                return self
            except (ValueError, OSError) as exc:
                raise CameraControlBusy("invalid inherited camera ownership: {0}".format(exc))
        descriptor = os.open(self.path, os.O_RDWR | os.O_CREAT, 0o600)
        handle = os.fdopen(descriptor, "a+")
        operation = fcntl.LOCK_EX if self.exclusive else fcntl.LOCK_SH
        try:
            fcntl.flock(handle.fileno(), operation | fcntl.LOCK_NB)
        except (BlockingIOError, OSError):
            handle.close()
            raise CameraControlBusy("camera head is owned by another controller")
        self.handle = handle
        return self

    def release(self):
        if self.handle is None:
            return
        try:
            if not self.inherited:
                fcntl.flock(self.handle.fileno(), fcntl.LOCK_UN)
        finally:
            self.handle.close()
            self.handle = None

    def __enter__(self):
        return self.acquire()

    def __exit__(self, exc_type, exc_value, traceback):
        self.release()
        return False


@contextmanager
def camera_control_lease(exclusive, path=DEFAULT_CAMERA_CONTROL_LOCK):
    lease = CameraControlLease(exclusive=exclusive, path=path)
    with lease:
        previous = os.environ.get(CAMERA_CONTROL_FD_ENV)
        try:
            if exclusive:
                os.environ[CAMERA_CONTROL_FD_ENV] = str(lease.handle.fileno())
            yield lease
        finally:
            if exclusive:
                if previous is None:
                    os.environ.pop(CAMERA_CONTROL_FD_ENV, None)
                else:
                    os.environ[CAMERA_CONTROL_FD_ENV] = previous


def subprocess_lease_options(lease=None):
    """Pass the same flock ownership through the mission process tree."""
    handle = getattr(lease, "handle", None)
    descriptor = handle.fileno() if handle is not None else os.environ.get(CAMERA_CONTROL_FD_ENV)
    if descriptor is None:
        return {}
    descriptor = int(descriptor)
    os.fstat(descriptor)
    environment = dict(os.environ)
    environment[CAMERA_CONTROL_FD_ENV] = str(descriptor)
    return {"env": environment, "pass_fds": (descriptor,)}


def manual_camera_control_available(path=DEFAULT_CAMERA_CONTROL_LOCK):
    try:
        with camera_control_lease(exclusive=False, path=path):
            return True
    except CameraControlBusy:
        return False
