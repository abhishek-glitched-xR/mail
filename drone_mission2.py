import asyncio
import cv2
import time
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

CAMERA_ID = 0

TAKEOFF_ALT_M = 5.0
CORRIDOR_ALT_M = 3.0
SEARCH_ALT_M = 10.0
DROP_ALT_M = 5.0

PAYLOAD_SERVO_CHANNEL = 9
PAYLOAD_RELEASE_PWM = 1900
PAYLOAD_LOCK_PWM = 1100


async def connect_drone():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Connecting to drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected")
            break

    print("Waiting for global position...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Position ready")
            break

    return drone


def scan_qr(timeout_s=15):
    detector = cv2.QRCodeDetector()
    cap = cv2.VideoCapture(CAMERA_ID)

    start = time.time()
    decoded_text = None

    while time.time() - start < timeout_s:
        ok, frame = cap.read()
        if not ok:
            continue

        text, points, _ = detector.detectAndDecode(frame)

        if text:
            decoded_text = text.strip()
            print("QR detected:", decoded_text)
            break

        cv2.imshow("QR Scan", frame)
        if cv2.waitKey(1) == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
    return decoded_text


def detect_green_banner(timeout_s=10):
    cap = cv2.VideoCapture(CAMERA_ID)
    start = time.time()

    while time.time() - start < timeout_s:
        ok, frame = cap.read()
        if not ok:
            continue

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_green = (35, 60, 60)
        upper_green = (85, 255, 255)

        mask = cv2.inRange(hsv, lower_green, upper_green)
        area = cv2.countNonZero(mask)

        if area > 6000:
            print("Green corridor banner detected")
            cap.release()
            return True

    cap.release()
    return False


async def goto_relative(drone, north_m, east_m, down_m, yaw_deg=0):
    await drone.offboard.set_position_ned(
        PositionNedYaw(north_m, east_m, down_m, yaw_deg)
    )
    await asyncio.sleep(4)


async def release_payload(drone):
    print("Releasing payload")

    # MAVSDK Python does not expose every servo command equally across versions.
    # Use this if your MAVSDK build supports command passthrough/actuator commands.
    # Otherwise trigger payload using an Arduino/ESP32 connected to a receiver AUX channel.

    await asyncio.sleep(1)
    print(f"Set servo CH{PAYLOAD_SERVO_CHANNEL} to {PAYLOAD_RELEASE_PWM}")
    await asyncio.sleep(2)
    print(f"Set servo CH{PAYLOAD_SERVO_CHANNEL} back to {PAYLOAD_LOCK_PWM}")


async def mission():
    drone = await connect_drone()

    print("Arming")
    await drone.action.arm()

    print("Taking off")
    await drone.action.set_takeoff_altitude(TAKEOFF_ALT_M)
    await drone.action.takeoff()
    await asyncio.sleep(8)

    print("Starting offboard mode")
    await drone.offboard.set_position_ned(PositionNedYaw(0, 0, -TAKEOFF_ALT_M, 0))

    try:
        await drone.offboard.start()
    except OffboardError as error:
        print("Offboard failed:", error)
        await drone.action.land()
        return

    print("Move forward 1 m and scan start QR")
    await goto_relative(drone, 1, 0, -TAKEOFF_ALT_M)

    delivery_code = scan_qr()
    if not delivery_code:
        print("QR not found. Returning home.")
        await drone.offboard.stop()
        await drone.action.return_to_launch()
        return

    print("Finding green corridor banner")
    banner_found = detect_green_banner()
    if not banner_found:
        print("Banner not found. Returning home.")
        await drone.offboard.stop()
        await drone.action.return_to_launch()
        return

    print("Descending to corridor altitude")
    await goto_relative(drone, 1, 0, -CORRIDOR_ALT_M)

    print("Navigating corridor")
    await goto_relative(drone, 8, 0, -CORRIDOR_ALT_M)

    print("Ascending to delivery search altitude")
    await goto_relative(drone, 8, 0, -SEARCH_ALT_M)

    print("Searching target QR")
    target_code = scan_qr(timeout_s=20)

    if target_code != delivery_code:
        print("Correct target not found. Returning home.")
        await drone.offboard.stop()
        await drone.action.return_to_launch()
        return

    print("Target matched. Descending for payload drop.")
    await goto_relative(drone, 8, 0, -DROP_ALT_M)

    await release_payload(drone)

    print("Returning through corridor")
    await goto_relative(drone, 8, 0, -SEARCH_ALT_M)
    await goto_relative(drone, 1, 0, -CORRIDOR_ALT_M)
    await goto_relative(drone, 0, 0, -TAKEOFF_ALT_M)

    print("Landing")
    await drone.offboard.stop()
    await drone.action.land()


if __name__ == "__main__":
    asyncio.run(mission())