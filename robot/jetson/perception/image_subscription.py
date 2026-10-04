"""Recreate an idle image subscription without reloading the inference model."""
import time


class ReconnectingImageSubscription:
    def __init__(self, node, message_type, topic, callback, qos, interval=10.0):
        self.node = node
        self.arguments = (message_type, topic, callback, qos)
        self.interval = interval
        self.last_attempt = time.monotonic()
        self.reconnects = 0
        self.last_error = None
        self.subscription = node.create_subscription(*self.arguments)

    def refresh_if_stale(self, now, frame_age):
        if frame_age is not None and frame_age < self.interval:
            return False
        if now - self.last_attempt < self.interval:
            return False
        self.last_attempt = now
        try:
            replacement = self.node.create_subscription(*self.arguments)
        except Exception as exc:
            self.last_error = str(exc)
            return False
        self.node.destroy_subscription(self.subscription)
        self.subscription = replacement
        self.reconnects += 1
        self.last_error = None
        return True


def configure_perception_transport():
    """Use the same UDP transport across isolated systemd workers by default."""
    import os
    from pathlib import Path
    if any(key in os.environ for key in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE')):
        return
    profile = Path(__file__).with_name('perception_transport.xml')
    if profile.is_file():
        os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(profile)
