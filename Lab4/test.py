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

# import time
# import numpy as np
# from robot_systems.robot import HamBot

# # 감지할 색상
# COLOR_LIST = ["orange", "green", "blue", "pink"]
# TARGET_COLORS = {
#     "orange": (255, 140, 50),
#     "green": (50, 200, 50),
#     "blue": (50, 150, 200),
#     "pink": (125, 100, 100)
# }
# TOLERANCE = 50
# FIXED_SPEED = 0.0  # 회전 속도
# SLEEP_TIME = 0.05   # 루프 딜레이

# def detect_color_in_frame(frame, target_color, tolerance):
#     """이미지 전체에서 특정 색상이 있는지 확인"""
#     diff = np.abs(frame.astype(int) - np.array(target_color))
#     mask = np.all(diff <= tolerance, axis=2)
#     return np.any(mask)

# def get_forward_distance(bot):
#     """전방 최소 거리 계산"""
#     scan = bot.get_range_image()
#     if scan is not None and len(scan) > 0:
#         # 전방 10도 영역에서 최소값
#         forward_distance = np.min(scan[175:185])
#         if np.isnan(forward_distance) or np.isinf(forward_distance):
#             forward_distance = 9999.9999
#         return forward_distance / 600  # 기존 코드처럼 스케일 조정
#     return None

# def turn_to_detect(bot):
#     """제자리 회전하면서 색상 감지"""
#     start_heading = bot.get_heading()
#     if start_heading is None:
#         print("[DEBUG] Warning: start heading is None")
#         return

#     detected_list = [None] * 4  # 4개 색상, 각 [distance, delta_angle]
#     detected_flags = [False] * 4

#     print("Starting 360° color detection...")

#     while True:
#         current_heading = bot.get_heading()
#         if current_heading is None:
#             continue

#         # 360° 완료 여부
#         delta_angle = (current_heading - start_heading + 360) % 360
#         if delta_angle > 180:
#             delta_angle = 360 - delta_angle  # 항상 최소 각도

#         if abs(delta_angle - 360) < 3:
#             # 회전 완료
#             bot.set_left_motor_speed(0)
#             bot.set_right_motor_speed(0)
#             print("360° rotation completed.")
#             break

#         # 제자리 회전
#         bot.set_left_motor_speed(-FIXED_SPEED)
#         bot.set_right_motor_speed(FIXED_SPEED)

#         # 프레임 색상 확인
#         frame = bot.camera.get_frame()
#         if frame is None:
#             continue

#         forward_distance = get_forward_distance(bot)
#         if forward_distance is None:
#             forward_distance = 9999.9999

#         for idx, color_name in enumerate(COLOR_LIST):
#             if detected_flags[idx]:
#                 continue  # 이미 감지했으면 패스

#             found = detect_color_in_frame(frame, TARGET_COLORS[color_name], TOLERANCE)
#             H, W = frame.shape[:2]
#             center_pixel = frame[H//2, W//2]  # (R,G,B)

#             print(f"[DEBUG] {color_name} | Found: {found} | Center pixel RGB: {center_pixel} | Forward distance: {forward_distance:.3f}")
            
#             if found:
#                 detected_flags[idx] = True
#                 detected_list[idx] = [forward_distance, delta_angle]
#                 print(f"[DETECTED] {color_name} | Distance: {forward_distance:.3f} m, Delta angle: {delta_angle:.2f}°")

#         # 디버그 출력 (감지 안 된 색상도 표시)
#         for idx, color_name in enumerate(COLOR_LIST):
#             if not detected_flags[idx]:
#                 print(f"[CHECKING] {color_name} | Forward: {forward_distance:.3f} m")

#         time.sleep(SLEEP_TIME)

#     print("Final detected list:", detected_list)
#     return detected_list

# def main():
#     try:
#         bot = HamBot(lidar_enabled=True, camera_enabled=True)
#         time.sleep(1)  # 카메라 초기화 대기
#         detected_list = turn_to_detect(bot)
#     except KeyboardInterrupt:
#         print("Detection stopped by user.")
#     finally:
#         bot.set_left_motor_speed(0)
#         bot.set_right_motor_speed(0)
#         bot.camera.stop()
#         print("Camera disabled.")

# if __name__ == "__main__":
#     main()

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
TOLERANCE = 50  # RGB 허용 오차
FIXED_SPEED = 0.0  # 제자리 회전 속도
SLEEP_TIME = 0.05   # 루프 딜레이

ROI_SIZE = 20  # 중앙 영역 크기 (20x20 픽셀)

def detect_color_in_frame(frame, target_color, tolerance):
    """중앙 영역 평균으로 특정 색상 존재 여부 확인"""
    H, W = frame.shape[:2]
    y1 = max(H//2 - ROI_SIZE//2, 0)
    y2 = min(H//2 + ROI_SIZE//2, H)
    x1 = max(W//2 - ROI_SIZE//2, 0)
    x2 = min(W//2 + ROI_SIZE//2, W)

    roi = frame[y1:y2, x1:x2]
    avg_rgb = np.mean(roi.reshape(-1,3), axis=0)[::-1]  # BGR -> RGB

    diff = np.abs(avg_rgb - np.array(target_color))
    found = np.all(diff <= tolerance)
    return found, avg_rgb.astype(int)

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

        # 현재 delta angle 계산
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

        # 프레임 가져오기
        frame = bot.camera.get_frame()
        if frame is None:
            continue

        forward_distance = get_forward_distance(bot)

        for idx, color_name in enumerate(COLOR_LIST):
            if detected_flags[idx]:
                continue

            found, avg_rgb = detect_color_in_frame(frame, TARGET_COLORS[color_name], TOLERANCE)

            print(f"[DEBUG] {color_name} | Found: {found} | Avg RGB: {avg_rgb} | Forward distance: {forward_distance:.3f}")

            if found:
                detected_flags[idx] = True
                detected_list[idx] = [forward_distance, delta_angle]
                print(f"[DETECTED] {color_name} | Distance: {forward_distance:.3f} m, Delta angle: {delta_angle:.2f}°")

        # 아직 감지되지 않은 색상 디버그
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
