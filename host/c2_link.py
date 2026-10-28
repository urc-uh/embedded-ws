import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rover_config import C2_TIMEOUT_S  # noqa: E402


class SimC2Link:
    def __init__(self):
        self.connected = False

    def connect(self):
        self.connected = True

    def disconnect(self):
        self.connected = False

    def is_connected(self):
        return self.connected
    
    def recv(self):
        frames = []
        if self.connected:
            frames.append("Connected")
        return frames

class C2Monitor:
    def __init__(self, C2_timeout_s=C2_TIMEOUT_S):
        self.C2_timeout_s = C2_timeout_s
        self.last_C2_time_s = 0
        self.have_connection = False

    def on_link_update(self, message_time):
        self.last_C2_time_s = message_time
        self.have_connection = True

    def bool_no_connection(self, now_s):
        if (not self.have_connection):
            return True
        return  (now_s - self.last_C2_time_s) >= self.C2_timeout_s

def poll_c2_lost(link, monitor, now_s):
    for msg in link.recv():
        monitor.on_link_update(now_s)
    return monitor.bool_no_connection(now_s)