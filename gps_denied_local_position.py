import math
import time
from dataclasses import dataclass

import cv2
import numpy as np
from pymavlink import mavutil


MAVLINK_CONNECTION = "/dev/ttyACM0"
MAVLINK_BAUD = 57600

DOWNWARD_CAMERA_ID = 0

# Camera intrinsics. Replace these with values from your camera calibration.
# fx/fy are focal lengths in pixels.
CAMERA_FX_PX = 615.0
CAMERA_FY_PX = 615.0

# If your downward camera image top points to the drone nose, keep this at 0.
# If rotated, use 90, 180, or 270 and verify signs during propeller-off tests.
CAMERA_YAW_DEG = 0.0

# Sign correction after mounting verification.
# If estimated motion goes backward when you move the drone forward by hand,
# change the matching sign from 1.0 to -1.0.
FORWARD_SIGN = 1.0
RIGHT_SIGN = 1.0

DEFAULT_ALTITUDE_M = 1.5
MIN_TRACKED_POINTS = 25
VISION_RATE_HZ = 20


@dataclass
class LocalState:
    north_m: float = 0.0
    east_m: float = 0.0
    down_m: float = 0.0
    yaw_rad: float = 0.0
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    vz_mps: float = 0.0


class GPSDeniedLocalPosition:
    def __init__(self):
        self.master = mavutil.mavlink_connection(
            MAVLINK_CONNECTION,
            baud=MAVLINK_BAUD,
            autoreconnect=True,
        )
        self.master.wait_heartbeat()
        print("Connected to flight controller")

        self.cap = cv2.VideoCapture(DOWNWARD_CAMERA_ID)
        if not self.cap.isOpened():
            raise RuntimeError("Could not open downward camera")

        self.state = LocalState()
        self.last_gray = None
        self.last_points = None
        self.last_time = None
        self.latest_range_m = DEFAULT_ALTITUDE_M

    def read_rangefinder_altitude(self):
        msg = self.master.recv_match(type="DISTANCE_SENSOR", blocking=False)
        if not msg:
            return self.latest_range_m

        distance_m = msg.current_distance / 100.0
        if 0.05 < distance_m < 20.0:
            self.latest_range_m = distance_m

        return self.latest_range_m

    def read_attitude_yaw(self):
        msg = self.master.recv_match(type="ATTITUDE", blocking=False)
        if msg:
            self.state.yaw_rad = msg.yaw
        return self.state.yaw_rad

    def detect_features(self, gray):
        points = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=180,
            qualityLevel=0.01,
            minDistance=8,
            blockSize=7,
        )
        return points

    def estimate_body_motion(self, gray, altitude_m, dt):
        if self.last_gray is None or self.last_points is None:
            self.last_gray = gray
            self.last_points = self.detect_features(gray)
            return 0.0, 0.0

        if self.last_points is None or len(self.last_points) < MIN_TRACKED_POINTS:
            self.last_points = self.detect_features(self.last_gray)