import cv2
import time
import math
import numpy as np
from pymavlink import mavutil

CONNECTION = "/dev/ttyACM0"
BAUD = 57600

CAMERA_ID = 0

TAKEOFF_ALT = 1.5
CORRIDOR_SPEED = 0.35
YAW_RATE = 15

QR_SCAN_TIME = 12
MISSION_TIMEOUT = 900

PAYLOAD_SERVO = 9
LOCK_PWM = 1100
RELEASE_PWM = 1900


class GPSDeniedDrone:
    def __init__(self):
        print("Connecting to flight controller...")
        self.master = mavutil.mavlink_connection(CONNECTION, baud=BAUD)
        self.master.wait_heartbeat()
        print("Heartbeat received")

        self.cap = cv2.VideoCapture(CAMERA_ID)
        self.qr = cv2.QRCodeDetector()

    def set_mode(self, mode):
        mode_id = self.master.mode_mapping()[mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        time.sleep(1)

    def arm(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        time.sleep(2)

    def disarm(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0
        )

    def takeoff(self, altitude):
        print("Taking off")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0,
            0, 0,
            altitude
        )
        time.sleep(6)

    def land(self):
        print("Landing")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0, 0, 0, 0
        )

    def send_body_velocity(self, vx, vy, vz, yaw_rate=0):
        """
        vx: forward m/s
        vy: right m/s
        vz: down m/s
        yaw_rate: deg/s
        """
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000011111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0,
            math.radians(yaw_rate)
        )

    def stop(self):
        self.send_body_velocity(0, 0, 0, 0)

    def scan_qr(self, timeout=QR_SCAN_TIME):
        print("Scanning QR...")
        start = time.time()

        while time.time() - start < timeout:
            ok, frame = self.cap.read()
            if not ok:
                continue

            text, points, _ = self.qr.detectAndDecode(frame)

            if text:
                print("QR found:", text)
                return text.strip()

            cv2.imshow("QR Scan", frame)
            cv2.waitKey(1)

        print("QR not found")
        return None

    def detect_green_banner_offset(self):
        ok, frame = self.cap.read()
        if not ok:
            return None

        h, w, _ = frame.shape
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_green = np.array([35, 60, 60])
        upper_green = np.array([85, 255, 255])

        mask = cv2.inRange(hsv, lower_green, upper_green)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < 1000:
            return None

        x, y, bw, bh = cv2.boundingRect(largest)
        cx = x + bw // 2

        offset = (cx - w // 2) / (w // 2)
        return offset

    def align_to_green_banner(self, timeout=10):
        print("Aligning to green corridor banner")
        start = time.time()

        while time.time() - start < timeout:
            offset = self.detect_green_banner_offset()

            if offset is None:
                self.send_body_velocity(0, 0, 0, YAW_RATE)
                continue

            if abs(offset) < 0.12:
                self.stop()
                print("Aligned")
                return True

            yaw = -offset * YAW_RATE
            self.send_body_velocity(0, 0, 0, yaw)
            time.sleep(0.1)

        self.stop()
        print("Banner alignment failed")
        return False

    def detect_red_zone(self):
        ok, frame = self.cap.read()
        if not ok:
            return False

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 80, 80])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 80, 80])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        red_area = cv2.countNonZero(mask)
        return red_area > 5000

    def fly_corridor(self, duration=18):
        print("Flying corridor")
        start = time.time()

        while time.time() - start < duration:
            if self.detect_red_zone():
                print("Red zone detected, shifting right")
                self.send_body_velocity(0.1, 0.25, 0)
            else:
                self.send_body_velocity(CORRIDOR_SPEED, 0, 0)

            time.sleep(0.1)

        self.stop()

    def release_payload(self):
        print("Releasing payload")

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            PAYLOAD_SERVO,
            RELEASE_PWM,
            0, 0, 0, 0, 0
        )

        time.sleep(2)

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            PAYLOAD_SERVO,
            LOCK_PWM,
            0, 0, 0, 0, 0
        )

    def mission(self):
        mission_start = time.time()

        self.set_mode("GUIDED")
        self.arm()
        self.takeoff(TAKEOFF_ALT)

        delivery_code = self.scan_qr()

        if not delivery_code:
            self.land()
            return

        aligned = self.align_to_green_banner()

        if not aligned:
            self.land()
            return

        self.fly_corridor(duration=18)

        print("Searching delivery QR")
        target_code = self.scan_qr(timeout=20)

        if target_code == delivery_code:
            print("Correct delivery target found")
            self.release_payload()
        else:
            print("Target mismatch or not found")

        print("Returning approximately by reverse corridor path")
        self.send_body_velocity(-CORRIDOR_SPEED, 0, 0)
        time.sleep(18)
        self.stop()

        self.land()

        while time.time() - mission_start < MISSION_TIMEOUT:
            time.sleep(1)
            break

        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    drone = GPSDeniedDrone()
    drone.mission()