import time
import numpy as np
from robot_systems.robot import HamBot
import math

# -------------------------------
# 감지할 색상 설정
# -------------------------------
COLOR_LIST = ["orange", "green", "blue", "pink"]
TARGET_COLORS = {
    "orange": (255, 150, 30),
    "green": (50, 200, 130),
    "blue": (50, 245, 245),
    "pink": (225, 30, 165)
}
TOLERANCE = 50
FIXED_SPEED = 4.0  # 제자리 회전 속도
SLEEP_TIME = 0.05  # 루프 딜레이

# -------------------------------
# Landmark positions (x, y) mm
# -------------------------------
landmark_positions = {
    "orange": (-2400, 2400),
    "green": (-2400,-2400),
    "blue": (2400,-2400),
    "pink": (2400,2400)
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
# 색상 감지 (ROI)
# -------------------------------
def turn_360_detect_roi(bot, roi_size=40):
    start_heading = bot.get_heading()
    if start_heading is None:
        print("[DEBUG] Warning: start heading is None")
        return

    detected_flags = [False] * 4
    detected_list = [None] * 4

    print("Starting 360° color detection (ROI)...")

    while True:
        current_heading = bot.get_heading()
        if current_heading is None:
            continue

        delta_angle_total = (current_heading - start_heading + 360) % 360

        if delta_angle_total >= 357:
            bot.set_left_motor_speed(0)
            bot.set_right_motor_speed(0)
            print("360° rotation completed.")
            break

        bot.set_left_motor_speed(-FIXED_SPEED)
        bot.set_right_motor_speed(FIXED_SPEED)

        frame = bot.camera.get_frame()
        if frame is None:
            continue

        H, W = frame.shape[:2]
        cx, cy = W // 2, H // 6  # 상단 영역
        x_start = max(cx - roi_size//2, 0)
        x_end = min(cx + roi_size//2, W)
        y_start = max(cy - roi_size//2, 0)
        y_end = min(cy + roi_size//2, H)
        roi = frame[y_start:y_end, x_start:x_end]

        forward_distance = get_forward_distance(bot)  # mm

        for idx, color_name in enumerate(COLOR_LIST):
            if detected_flags[idx]:
                print(f"[DEBUG] {color_name} already detected | Forward:{forward_distance:.3f} | Delta:{delta_angle_total:.2f}")
                continue

            target_color = np.array(TARGET_COLORS[color_name])
            diff = np.abs(roi.astype(int) - target_color)
            mask = np.all(diff <= TOLERANCE, axis=2)
            found = np.any(mask)

            roi_avg = roi.mean(axis=(0,1)).astype(int)
            print(f"[DEBUG] {color_name} | ROI avg RGB: R:{roi_avg[0]} G:{roi_avg[1]} B:{roi_avg[2]} | Forward:{forward_distance:.3f} | Delta:{delta_angle_total:.2f} | Found: {found}")

            if found:
                detected_flags[idx] = True
                detected_list[idx] = [forward_distance, delta_angle_total]
                print(f"[DETECTED] {color_name} | Forward:{forward_distance:.3f}, Delta angle:{delta_angle_total:.2f}")

        time.sleep(SLEEP_TIME)

    print("Final detected list:", detected_list)
    return detected_list

# -------------------------------
# Trilateration (3개 이상)
# -------------------------------
def trilateration(detected_list):
    xs, ys = [], []

    for idx, item in enumerate(detected_list):
        if item is None:
            continue
        distance, delta_angle = item
        if distance >= 9999.0:
            continue
        color_name = COLOR_LIST[idx]
        lx, ly = landmark_positions[color_name]

        angle_rad = math.radians(delta_angle)
        # landmark 기준 좌표에서 로봇 위치 계산
        rx = lx - distance * math.cos(angle_rad)
        ry = ly - distance * math.sin(angle_rad)
        xs.append(rx)
        ys.append(ry)

    if len(xs) == 0:
        return -300, -300

    estimated_x = np.mean(xs)
    estimated_y = np.mean(ys)
    return estimated_x, estimated_y

# -------------------------------
# Trilateration (2개 교차점)
# -------------------------------
def trilateration_two_points(detected_list):
    valid_items = [(COLOR_LIST[i], detected_list[i]) for i in range(4) if detected_list[i] is not None]
    if len(valid_items) != 2:
        return -300, -300

    (c1, (d1, a1)), (c2, (d2, a2)) = valid_items
    x1, y1 = landmark_positions[c1]
    x2, y2 = landmark_positions[c2]

    dx = x2 - x1
    dy = y2 - y1
    D = math.hypot(dx, dy)

    if D > d1 + d2 or D < abs(d1 - d2):
        return (x1+x2)/2, (y1+y2)/2

    a = (d1**2 - d2**2 + D**2)/(2*D)
    h = math.sqrt(max(d1**2 - a**2, 0))
    xm = x1 + a*dx/D
    ym = y1 + a*dy/D
    rx = -dy * h/D
    ry = dx * h/D
    p1 = (xm+rx, ym+ry)
    p2 = (xm-rx, ym-ry)
    print(f"[2-POINT SOLUTIONS] Intersection points: {p1}, {p2}")
    return ((p1[0]+p2[0])/2, (p1[1]+p2[1])/2)

# -------------------------------
# Grid cell 매핑 (0~15)
# -------------------------------
def get_cell_index(x, y, grid_size=4, world_min=-2400, world_max=2400):
    cell_width = (world_max - world_min)/grid_size
    col = int((x - world_min)//cell_width)
    row = int((world_max - y)//cell_width)
    col = max(0, min(grid_size-1, col))
    row = max(0, min(grid_size-1, row))
    return row*grid_size + col  # 0~15

# -------------------------------
# Main
# -------------------------------
def main():
    try:
        bot = HamBot(lidar_enabled=True, camera_enabled=True)
        time.sleep(1)

        detected_list = turn_360_detect_roi(bot)

        num_detected = sum([1 for d in detected_list if d is not None and d[0]<9999.0])

        if num_detected >= 3:
            x, y = trilateration(detected_list)
            print(f"[TRILATERATION] Estimated robot position: x={x:.2f}, y={y:.2f}")
        elif num_detected == 2:
            x, y = trilateration_two_points(detected_list)
            print(f"[TRILATERATION-2] Estimated robot position: x={x:.2f}, y={y:.2f}")
        else:
            x, y = -300, -300
            print("[TRILATERATION] Not enough landmarks detected. Using default position.")

        cell_index = get_cell_index(x, y)
        print(f"[GRID] Estimated starting cell index (0~15): {cell_index}")

    except KeyboardInterrupt:
        print("Detection stopped by user.")
    finally:
        bot.set_left_motor_speed(0)
        bot.set_right_motor_speed(0)
        bot.camera.stop()
        print("Camera disabled.")

if __name__ == "__main__":
    main()

