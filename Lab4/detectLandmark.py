import time
import numpy as np
from robot_systems.robot import HamBot

# 감지할 색상
COLOR_LIST = ["orange", "green", "blue", "pink"]
TARGET_COLORS = {
    "orange": (255, 153, 51),
    "green": (0, 255, 0),
    "blue": (153, 255, 255),
    "pink": (204, 0, 152)
}
TOLERANCE = 50
FIXED_SPEED = 1.0  # 제자리 회전 속도
SLEEP_TIME = 0.05  # 루프 딜레이

def detect_color(pixel_rgb, target_color, tolerance):
    """단일 픽셀로 색상 존재 여부 확인"""
    diff = np.abs(np.array(pixel_rgb, dtype=np.int16) - np.array(target_color, dtype=np.int16))
    return np.all(diff <= tolerance)

def get_forward_distance(bot):
    """전방 최소 거리 계산"""
    scan = bot.get_range_image()
    if scan is not None and len(scan) > 0:
        forward_distance = np.min(scan[175:185])
        if np.isnan(forward_distance) or np.isinf(forward_distance) or forward_distance < 0:
            forward_distance = 9999.9999
        return forward_distance / 600
    return 9999.9999

def turn_to_detect(bot):
    """제자리 회전하며 색상 감지"""
    start_heading = bot.get_heading()
    if start_heading is None:
        print("[DEBUG] Warning: start heading is None")
        return

    detected_list = [None] * 4  # 색상별 [distance, delta_angle]
    detected_flags = [False] * 4

    print("Starting 360° color detection...")

    while True:
        current_heading = bot.get_heading()
        if current_heading is None:
            continue

        delta_angle = (current_heading - start_heading + 360) % 360
        if delta_angle > 180:
            delta_angle = 360 - delta_angle

        # 360° 완료
        if abs(delta_angle - 360) < 3:
            bot.set_left_motor_speed(0)
            bot.set_right_motor_speed(0)
            print("360° rotation completed.")
            break

        # 제자리 회전
        bot.set_left_motor_speed(-FIXED_SPEED)
        bot.set_right_motor_speed(FIXED_SPEED)

        # 카메라 프레임 가져오기
        frame = bot.camera.get_frame()
        if frame is None:
            continue

        H, W = frame.shape[:2]
        pixel_bgr = frame[H//2, W//2]           # 중앙 픽셀 (B,G,R)
        pixel_rgb = pixel_bgr[::-1]             # BGR -> RGB

        forward_distance = get_forward_distance(bot)

        for idx, color_name in enumerate(COLOR_LIST):
            if detected_flags[idx]:
                continue

            found = detect_color(pixel_rgb, TARGET_COLORS[color_name], TOLERANCE)
            print(f"[DEBUG] {color_name} | Found: {found} | Pixel RGB: {pixel_rgb} | Forward distance: {forward_distance:.3f}")

            if found:
                detected_flags[idx] = True
                detected_list[idx] = [forward_distance, delta_angle]
                print(f"[DETECTED] {color_name} | Distance: {forward_distance:.3f} m, Delta angle: {delta_angle:.2f}°")

        # 아직 감지 안 된 색상 디버그
        for idx, color_name in enumerate(COLOR_LIST):
            if not detected_flags[idx]:
                print(f"[CHECKING] {color_name} | Forward: {forward_distance:.3f} m")

        time.sleep(SLEEP_TIME)

    print("Final detected list:", detected_list)
    return detected_list

def main():
    try:
        bot = HamBot(lidar_enabled=True, camera_enabled=True)
        time.sleep(1)
        detected_list = turn_to_detect(bot)
    except KeyboardInterrupt:
        print("Detection stopped by user.")
    finally:
        bot.set_left_motor_speed(0)
        bot.set_right_motor_speed(0)
        bot.camera.stop()
        print("Camera disabled.")

if __name__ == "__main__":
    main()
