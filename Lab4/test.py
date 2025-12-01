"""
필요한 function:
360 돌면서 4개의 landmark detect 하고 4개중 하나의 색상이 발견되면 그 위치와 시작의 각도에서 얼마나 돌았는지
저장하고 계속 회전
color_list= [orange, green, blue, pink]
detected_list = [4][2] // 4 colors with ecah distance and delta
def turn_to_detect():
    current_heading = self.bot.get_heading()
    if current_heading is None:
        print("[DEBUG] Warning: current_heading is None, skipping turn")
        return False
    error = abs((current_heading - target_angle + 360) % 360 - 180)
        if error > 180:
            error = 360 - error  # 항상 최소 각도
     print(f"[DEBUG] Turning left only - Current: {current_heading:.2f}, Target: {target_angle:.2f}, Error: {error:.2f}")
    if error < 3:  # ±3° 안이면 멈춤
            self.bot.set_left_motor_speed(0.0)
            self.bot.set_right_motor_speed(0.0)
            print("[DEBUG] Reached target heading, motors stopped")
            return True
        else:
            # HamBot 모터 범위 내 고정 속도
            fixed_speed = 4.0
            self.bot.set_left_motor_speed(-fixed_speed)   # 왼쪽 모터 뒤
            self.bot.set_right_motor_speed(fixed_speed)   # 오른쪽 모터 앞으로
            frame = bot.camera.get_frame() # get_image() → get_frame() 변경 if frame is None:
                continue H, W = frame.shape[:2] # 지정 좌표 픽셀 값 확인 
                x = min(max(SAMPLE_X, 0), W-1) 
                y = min(max(SAMPLE_Y, 0), H-1) 
                pixel_color = frame[y, x] # (R, G, B) 
                r_diff = abs(int(pixel_color[0]) - TARGET_COLOR[0]) 
                g_diff = abs(int(pixel_color[1]) - TARGET_COLOR[1]) 
                b_diff = abs(int(pixel_color[2]) - TARGET_COLOR[2]) 
            print()
            for checking the list of color i want to detect :
                if find:
                    print index of the list 
                    
                    store the distance using lidar and delta how move
                    append a stored list
        
                    within_tolerance = r_diff <= TOLERANCE and g_diff <= TOLERANCE and b_diff <= TOLERANCE 
                    print(f"Forward: {forward_distance:.3f}") print(f"Pixel ({x},{y}) RGB: {pixel_color}, Diff: (R:{r_diff}, G:{g_diff}, B:{b_diff}), Match: {within_tolerance}")
                    
                    continue
                else:
                    print(f"Forward: {forward_distance:.3f}") print(f"Pixel ({x},{y}) RGB: {pixel_color}")
                    conitue
            return False 

"""
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

def get_forward_distance(bot):
    """전방 최소 거리 계산"""
    scan = bot.get_range_image()
    if scan is not None and len(scan) > 0:
        forward_distance = np.min(scan[175:185])
        if np.isnan(forward_distance) or np.isinf(forward_distance) or forward_distance < 0:
            forward_distance = 9999.9999
        return forward_distance / 600
    return 9999.9999

def detect_landmark_color(bot, target_color, tolerance):
    """
    랜드마크를 찾고, 해당 위치 픽셀 RGB가 target_color 범위 내이면 True 반환
    """
    landmarks = bot.camera.find_landmarks()
    if not landmarks:
        return False

    # 첫 번째 Landmark 사용
    landmark = landmarks[0]
    x, y = landmark.x, landmark.y

    frame = bot.camera.get_frame()
    if frame is None:
        return False

    H, W = frame.shape[:2]
    x = min(max(0, int(x)), W-1)
    y = min(max(0, int(y)), H-1)

    pixel_bgr = frame[y, x]         # BGR
    pixel_rgb = pixel_bgr[::-1]     # BGR -> RGB

    r_diff = abs(int(pixel_rgb[0]) - target_color[0])
    g_diff = abs(int(pixel_rgb[1]) - target_color[1])
    b_diff = abs(int(pixel_rgb[2]) - target_color[2])

    within_tolerance = r_diff <= tolerance and g_diff <= tolerance and b_diff <= tolerance
    return within_tolerance, pixel_rgb

def turn_to_detect(bot):
    """제자리 회전하며 랜드마크 색상 감지"""
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

        # 360° 완료 여부
        if abs(delta_angle - 360) < 3:
            bot.set_left_motor_speed(0)
            bot.set_right_motor_speed(0)
            print("360° rotation completed.")
            break

        # 제자리 회전
        bot.set_left_motor_speed(-FIXED_SPEED)
        bot.set_right_motor_speed(FIXED_SPEED)

        forward_distance = get_forward_distance(bot)

        for idx, color_name in enumerate(COLOR_LIST):
            if detected_flags[idx]:
                continue

            found, pixel_rgb = detect_landmark_color(bot, TARGET_COLORS[color_name], TOLERANCE)
            if found:
                detected_flags[idx] = True
                detected_list[idx] = [forward_distance, delta_angle]
                print(f"[DETECTED] {color_name} | Pixel RGB: {pixel_rgb} | Distance: {forward_distance:.3f} m, Delta angle: {delta_angle:.2f}°")
            else:
                print(f"[DEBUG] {color_name} | Pixel RGB: {pixel_rgb} | Forward distance: {forward_distance:.3f}")

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
