import time
import numpy as np
from robot_systems.robot import HamBot

import math

def trilateration_two_points(detected_list):
    """
    detected_list에서 2개만 존재할 때,
    두 landmark와 거리로 두 개의 교차점을 계산.
    """

    # 감지된 두 개 추출
    valid_items = [(COLOR_LIST[i], detected_list[i]) for i in range(4) if detected_list[i] is not None]
    if len(valid_items) != 2:
        print("[ERROR] trilateration_two_points called but detected_list does not contain exactly 2 entries.")
        return None, None

    # Landmark positions & distances
    (color1, (d1, _)), (color2, (d2, _)) = valid_items
    (x1, y1) = landmark_positions[color1]
    (x2, y2) = landmark_positions[color2]

    # Landmark 간 거리
    dx = x2 - x1
    dy = y2 - y1
    D = math.sqrt(dx*dx + dy*dy)

    print(f"[DEBUG-2PT] Using landmarks {color1} and {color2}")
    print(f"[DEBUG-2PT] Landmark1=({x1},{y1}), d1={d1}")
    print(f"[DEBUG-2PT] Landmark2=({x2},{y2}), d2={d2}")
    print(f"[DEBUG-2PT] Landmark distance D={D}")

    # 원이 서로 닿지 않거나 한 원이 다른 원을 포함하는 경우
    if D > d1 + d2 or D < abs(d1 - d2):
        print("[WARNING] Circles do not intersect. No valid solution.")
        return None, None

    # 두 원 교차점 공식
    a = (d1*d1 - d2*d2 + D*D) / (2*D)
    h = math.sqrt(max(d1*d1 - a*a, 0))

    xm = x1 + a * dx / D
    ym = y1 + a * dy / D

    # 교차점 2개
    rx = -dy * (h / D)
    ry = dx * (h / D)

    p1 = (xm + rx, ym + ry)
    p2 = (xm - rx, ym - ry)

    print(f"[2-POINT SOLUTIONS] Intersection point 1: x={p1[0]:.2f}, y={p1[1]:.2f}")
    print(f"[2-POINT SOLUTIONS] Intersection point 2: x={p2[0]:.2f}, y={p2[1]:.2f}")

    # Return both in case user wants to choose
    # For now, pick the midpoint (or one of them)
    px = (p1[0] + p2[0]) / 2
    py = (p1[1] + p2[1]) / 2

    return px, py


# -------------------------------
# 감지할 색상 설정
# -------------------------------
COLOR_LIST = ["orange", "green", "blue", "pink"]
TARGET_COLORS = {
    "orange": (255, 150, 30),
    "green": (50, 225, 130),
    "blue": (110, 220, 225),
    "pink": (225, 30, 165)
}
TOLERANCE = 45
FIXED_SPEED = 2.0  # 제자리 회전 속도
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
# 360° 회전하며 색상 감지
# -------------------------------
# # -------------------------------
# 360° 회전하며 색상 감지 (ROI 기준)
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
        cx, cy = W // 2, H // 2

        # ROI 범위 설정
        x_start = max(cx - roi_size//2, 0)
        x_end = min(cx + roi_size//2, W)
        y_start = max(cy - roi_size//2, 0)
        y_end = min(cy + roi_size//2, H)

        roi = frame[y_start:y_end, x_start:x_end]

        forward_distance = get_forward_distance(bot)

        # 4개 색상 확인
        for idx, color_name in enumerate(COLOR_LIST):
            if detected_flags[idx]:
                # 이미 감지된 색상
                print(f"[DEBUG] {color_name} already detected | Forward:{forward_distance:.3f} | Delta:{delta_angle_total:.2f}")
                continue

            target_color = np.array(TARGET_COLORS[color_name])
            diff = np.abs(roi.astype(int) - target_color)
            mask = np.all(diff <= TOLERANCE, axis=2)
            found = np.any(mask)  # ROI 안에 한 픽셀이라도 존재하면 True

            # 디버그용: ROI 평균 RGB (중앙 픽셀 대신 참고용)
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
def get_cell_index(x, y, grid_size=4, world_min=-2400, world_max=2400):
    cell_width = (world_max - world_min) / grid_size  # 각 셀 크기
    col = int((x - world_min) // cell_width)
    row = int((world_max - y) // cell_width)  # y는 위쪽이 최대
    col = max(0, min(grid_size-1, col))
    row = max(0, min(grid_size-1, row))
    return row * grid_size + col + 1  # 1~16

def get_mode_color(roi):
    # roi: HxWx3
    # reshape into N x 3
    pixels = roi.reshape(-1, 3)

    # unique colors with counts
    unique, counts = np.unique(pixels, axis=0, return_counts=True)

    # pick the most frequent color
    mode_color = unique[counts.argmax()]
    r, g, b = int(mode_color[0]), int(mode_color[1]), int(mode_color[2])
    return r, g, b


# -------------------------------
# 메인 실행
# -------------------------------
def main():
    try:
        bot = HamBot(lidar_enabled=True, camera_enabled=True)
        time.sleep(1)  # 카메라 초기화 대기

        # 360° 색상 감지
        detected_list = turn_360_detect_roi(bot)

        # 감지된 랜드마크 수 확인
        num_detected = sum([1 for d in detected_list if d is not None and d[0] < 9999.0])

        if num_detected >= 3:
            # Trilateration 계산
            x, y = trilateration(detected_list)
            print(f"[TRILATERATION] Estimated robot position: x={x:.2f}, y={y:.2f}")
        elif num_detected ==2:
            x, y = trilateration_two_points(detected_list)
            print(f"[TRILATERATION-2] Final chosen estimate: x={x:.2f}, y={y:.2f}")

            
        else:
            # 2개 이하 감지 시 기본 위치 사용
            # **************
            x, y = -300, -300
            print("[TRILATERATION] Not enough landmarks detected (<=2). Using default position: x=-300, y=-300")

        # Grid cell index 계산 (4x4 기준)
        print("HERE IS OUTPUT")
        print("HERE IS OUTPUT")
        print("HERE IS OUTPUT")
        print("HERE IS OUTPUT")
        print("HERE IS OUTPUT")
        print("HERE IS OUTPUT")
        print("HERE IS OUTPUT")
        print("HERE IS OUTPUT")
        print("HERE IS OUTPUT")
        print("HERE IS OUTPUT")
        print("HERE IS OUTPUT")
        print("HERE IS OUTPUT")
        print("HERE IS OUTPUT")
        print("HERE IS OUTPUT")
        print("HERE IS OUTPUT")
        print("HERE IS OUTPUT")
        print("HERE IS OUTPUT")
        print("HERE IS OUTPUT")
        cell_index = get_cell_index(x, y, grid_size=4)
        print(f"[GRID] Estimated starting cell index: {cell_index}")

    except KeyboardInterrupt:
        print("Detection stopped by user.")
    finally:
        bot.set_left_motor_speed(0)
        bot.set_right_motor_speed(0)
        bot.camera.stop()
        print("Camera disabled.")

if __name__ == "__main__":
    main()
