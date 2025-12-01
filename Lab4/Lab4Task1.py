import time
import numpy as np
from robot_systems.robot import HamBot

# -------------------------------
# 감지할 색상 설정
# -------------------------------
COLOR_LIST = ["orange", "green", "blue", "pink"]
TARGET_COLORS = {
    "orange": (255, 150, 30),
    "green": (50, 200, 130),
    "blue": (50, 80, 80),
    "pink": (150, 30, 30)
}
TOLERANCE = 50
FIXED_SPEED = 4.0  # 제자리 회전 속도
SLEEP_TIME = 0.05  # 루프 딜레이

# -------------------------------
# Landmark positions (x, y) mm
# -------------------------------
landmark_positions = {
    "orange": (-350, 350),
    "green": (-350,-350),
    "blue": (350,-350),
    "pink": (350,350)
}

# -------------------------------
# 전방 최소 거리 계산
# -------------------------------
def get_forward_distance(bot):
    scan = bot.get_range_image()
    if scan is not None and len(scan) > 0:
        forward_distance = np.min(scan[175:185])
        if np.isnan(forward_distance) or np.isinf(forward_distance) or forward_distance < 0:
            forward_distance = 9999.9999
        return forward_distance
    return 9999.9999

# -------------------------------
# 360° 회전하며 색상 감지
# -------------------------------
def turn_360_detect(bot):
    start_heading = bot.get_heading()
    if start_heading is None:
        print("[DEBUG] Warning: start heading is None")
        return

    detected_flags = [False] * 4
    detected_list = [None] * 4

    print("Starting 360° color detection...")

    while True:
        current_heading = bot.get_heading()
        if current_heading is None:
            continue

        # delta angle 계산
        delta_angle_total = (current_heading - start_heading + 360) % 360

        # 360° 완료 여부 (±3° 허용)
        if delta_angle_total >= 357:
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
        center_pixel = frame[H // 2, W // 2]  # 중앙 픽셀
        r, g, b = center_pixel

        forward_distance = get_forward_distance(bot)

        # 4개 색상 확인
        for idx, color_name in enumerate(COLOR_LIST):
            if detected_flags[idx]:
                # 이미 감지된 색상
                print(f"[DEBUG] {color_name} already detected | Pixel RGB: R:{r} G:{g} B:{b} | Forward:{forward_distance:.3f}")
                continue

            target_color = TARGET_COLORS[color_name]
            r_diff = abs(r - target_color[0])
            g_diff = abs(g - target_color[1])
            b_diff = abs(b - target_color[2])
            within_tolerance = r_diff <= TOLERANCE and g_diff <= TOLERANCE and b_diff <= TOLERANCE

            # 실시간 디버그 출력
            print(f"[DEBUG] {color_name} | Pixel RGB: R:{r} G:{g} B:{b} | R_diff:{r_diff} G_diff:{g_diff} B_diff:{b_diff} | Forward:{forward_distance:.3f} | Delta:{delta_angle_total:.2f}")

            if within_tolerance:
                detected_flags[idx] = True
                detected_list[idx] = [forward_distance, delta_angle_total]
                print(f"[DETECTED] {color_name} | Forward:{forward_distance:.3f}, Delta angle:{delta_angle_total:.2f}")

        time.sleep(SLEEP_TIME)

    print("Final detected list:", detected_list)
    return detected_list

# -------------------------------
# Trilateration (2D)
# -------------------------------
def trilateration(detected_list):
    # 최소 3개 유효 landmark 필요
    points = []
    distances = []
    for idx, item in enumerate(detected_list):
        if item is not None:
            distance, _ = item
            if distance >= 9999.0:
                continue  # 유효하지 않은 거리 제외
            color_name = COLOR_LIST[idx]
            points.append(landmark_positions[color_name])
            distances.append(distance)

    if len(points) < 3:
        print("[TRILATERATION] Not enough landmarks detected for trilateration.")
        return None, None

    # 단순 평균법으로 근사
    xs, ys = zip(*points)
    # 비율로 보정: landmark 중심과 거리 비율
    estimated_x = np.mean([x - distances[i] for i, x in enumerate(xs)])
    estimated_y = np.mean([y - distances[i] for i, y in enumerate(ys)])

    return estimated_x, estimated_y

# -------------------------------
# Grid Cell Index 매핑
# -------------------------------
def get_cell_index(x, y, grid_size=5, world_min=-400, world_max=400):
    cell_width = (world_max - world_min) / grid_size
    col = int((x - world_min) // cell_width)
    row = int((world_max - y) // cell_width)  # y는 위쪽이 최대
    col = max(0, min(grid_size-1, col))
    row = max(0, min(grid_size-1, row))
    return row * grid_size + col + 1

# -------------------------------
# 메인 실행
# -------------------------------
def main():
    try:
        bot = HamBot(lidar_enabled=True, camera_enabled=True)
        time.sleep(1)  # 카메라 초기화 대기

        # 360° 색상 감지
        detected_list = turn_360_detect(bot)

        # Trilateration
        x, y = trilateration(detected_list)
        if x is not None and y is not None:
            print(f"[TRILATERATION] Estimated robot position: x={x:.2f}, y={y:.2f}")

            # Grid cell index
            cell_index = get_cell_index(x, y)
            print(f"[GRID] Estimated starting cell index: {cell_index}")
        else:
            print("[TRILATERATION] Could not estimate position.")

    except KeyboardInterrupt:
        print("Detection stopped by user.")
    finally:
        bot.set_left_motor_speed(0)
        bot.set_right_motor_speed(0)
        bot.camera.stop()
        print("Camera disabled.")

if __name__ == "__main__":
    main()
