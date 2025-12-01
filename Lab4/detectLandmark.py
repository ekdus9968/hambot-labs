import time
import numpy as np
from robot_systems.robot import HamBot

# 감지할 색상
COLOR_LIST = ["orange", "green", "blue", "pink"]
TARGET_COLORS = {
    "orange": (255, 140, 50),
    "green": (50, 200, 50),
    "blue": (100, 180, 255),
    "pink": (255, 220, 0)
}
TOLERANCE = 50
FIXED_SPEED = 1.0  # 회전 속도
SLEEP_TIME = 0.05   # 루프 딜레이

def detect_color_in_frame(frame, target_color, tolerance):
    """이미지 전체에서 특정 색상이 있는지 확인"""
    diff = np.abs(frame.astype(int) - np.array(target_color))
    mask = np.all(diff <= tolerance, axis=2)
    return np.any(mask)

def get_forward_distance(bot):
    """전방 최소 거리 계산"""
    scan = bot.get_range_image()
    if scan is not None and len(scan) > 0:
        # 전방 10도 영역에서 최소값
        forward_distance = np.min(scan[175:185])
        if np.isnan(forward_distance) or np.isinf(forward_distance):
            forward_distance = 9999.9999
        return forward_distance / 600  # 기존 코드처럼 스케일 조정
    return None

def turn_to_detect(bot):
    """제자리 회전하면서 색상 감지"""
    start_heading = bot.get_heading()
    if start_heading is None:
        print("[DEBUG] Warning: start heading is None")
        return

    detected_list = [None] * 4  # 4개 색상, 각 [distance, delta_angle]
    detected_flags = [False] * 4

    print("Starting 360° color detection...")

    while True:
        current_heading = bot.get_heading()
        if current_heading is None:
            continue

        # 360° 완료 여부
        delta_angle = (current_heading - start_heading + 360) % 360
        if delta_angle > 180:
            delta_angle = 360 - delta_angle  # 항상 최소 각도

        if abs(delta_angle - 360) < 3:
            # 회전 완료
            bot.set_left_motor_speed(0)
            bot.set_right_motor_speed(0)
            print("360° rotation completed.")
            break

        # 제자리 회전
        bot.set_left_motor_speed(-FIXED_SPEED)
        bot.set_right_motor_speed(FIXED_SPEED)

        # 프레임 색상 확인
        frame = bot.camera.get_frame()
        if frame is None:
            continue

        forward_distance = get_forward_distance(bot)
        if forward_distance is None:
            forward_distance = 9999.9999

        for idx, color_name in enumerate(COLOR_LIST):
            if detected_flags[idx]:
                continue  # 이미 감지했으면 패스

            found = detect_color_in_frame(frame, TARGET_COLORS[color_name], TOLERANCE)
            H, W = frame.shape[:2]
            center_pixel = frame[H//2, W//2]  # (R,G,B)

            print(f"[DEBUG] {color_name} | Found: {found} | Center pixel RGB: {center_pixel} | Forward distance: {forward_distance:.3f}")
            
            if found:
                detected_flags[idx] = True
                detected_list[idx] = [forward_distance, delta_angle]
                print(f"[DETECTED] {color_name} | Distance: {forward_distance:.3f} m, Delta angle: {delta_angle:.2f}°")

        # 디버그 출력 (감지 안 된 색상도 표시)
        for idx, color_name in enumerate(COLOR_LIST):
            if not detected_flags[idx]:
                print(f"[CHECKING] {color_name} | Forward: {forward_distance:.3f} m")

        time.sleep(SLEEP_TIME)

    print("Final detected list:", detected_list)
    return detected_list

def main():
    try:
        bot = HamBot(lidar_enabled=True, camera_enabled=True)
        time.sleep(1)  # 카메라 초기화 대기
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
