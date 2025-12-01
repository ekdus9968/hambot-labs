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
# 랜드마크 절대 좌표 (mm)
# -------------------------------
landmark_positions = {
    "orange": (-350, 350),
    "green": (-350, -350),
    "blue": (350, -350),
    "pink": (350, 350)
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

        delta_angle_total = (current_heading - start_heading + 360) % 360

        # 360° 완료 여부
        if delta_angle_total >= 357:
            bot.set_left_motor_speed(0)
            bot.set_right_motor_speed(0)
            print("360° rotation completed.")
            break

        # 제자리 회전
        bot.set_left_motor_speed(-FIXED_SPEED)
        bot.set_right_motor_speed(FIXED_SPEED)

        frame = bot.camera.get_frame()
        if frame is None:
            continue

        H, W = frame.shape[:2]
        r, g, b = frame[H // 2, W // 2]  # 중앙 픽셀

        forward_distance = get_forward_distance(bot)

        for idx, color_name in enumerate(COLOR_LIST):
            if detected_flags[idx]:
                print(f"[DEBUG] {color_name} already detected | Pixel RGB: R:{r} G:{g} B:{b} | Forward:{forward_distance:.3f}")
                continue

            target_color = TARGET_COLORS[color_name]
            r_diff = abs(r - target_color[0])
            g_diff = abs(g - target_color[1])
            b_diff = abs(b - target_color[2])
            within_tolerance = r_diff <= TOLERANCE and g_diff <= TOLERANCE and b_diff <= TOLERANCE

            print(f"[DEBUG] {color_name} | Pixel RGB: R:{r} G:{g} B:{b} | R_diff:{r_diff} G_diff:{g_diff} B_diff:{b_diff} | Forward:{forward_distance:.3f} | Delta:{delta_angle_total:.2f}")

            if within_tolerance:
                detected_flags[idx] = True
                detected_list[idx] = [forward_distance, delta_angle_total]
                print(f"[DETECTED] {color_name} | Forward:{forward_distance:.3f}, Delta angle:{delta_angle_total:.2f}")

        time.sleep(SLEEP_TIME)

    print("Final detected list:", detected_list)
    return detected_list

# -------------------------------
# 단순 Trilateration (NumPy만 사용)
# -------------------------------
def trilateration(detected_list):
    points = []
    distances = []
    for idx, item in enumerate(detected_list):
        if item is not None:
            distance, _ = item
            color_name = COLOR_LIST[idx]
            points.append(landmark_positions[color_name])
            distances.append(distance)

    points = np.array(points)
    distances = np.array(distances)

    if len(points) < 2:
        print("[TRILATERATION ERROR] 최소 2개 이상의 landmark 필요")
        return None

    # 단순 평균 기반 추정: 모든 랜드마크에서의 원과 중심점 추정
    xs = []
    ys = []
    for i in range(len(points)):
        for j in range(i+1, len(points)):
            x1, y1 = points[i]
            x2, y2 = points[j]
            r1 = distances[i]
            r2 = distances[j]

            # 두 원의 교차점 근사 계산 (x-axis 기준)
            dx = x2 - x1
            dy = y2 - y1
            d = np.hypot(dx, dy)
            if d > (r1 + r2):
                continue  # 교차하지 않음
            # 단순히 두 점 중간에 가까운 위치로 대략 계산
            t = (r1**2 - r2**2 + d**2) / (2*d**2)
            x_mid = x1 + t*dx
            y_mid = y1 + t*dy
            xs.append(x_mid)
            ys.append(y_mid)

    if len(xs) == 0:
        print("[TRILATERATION ERROR] 유효한 교차점 없음")
        return None

    x_est = np.mean(xs)
    y_est = np.mean(ys)
    print(f"[TRILATERATION] Estimated robot position: x={x_est:.3f}, y={y_est:.3f}")
    return x_est, y_est

# -------------------------------
# 메인 실행
# -------------------------------
def main():
    try:
        bot = HamBot(lidar_enabled=True, camera_enabled=True)
        time.sleep(1)

        detected_list = turn_360_detect(bot)
        trilateration(detected_list)

    except KeyboardInterrupt:
        print("Detection stopped by user.")
    finally:
        bot.set_left_motor_speed(0)
        bot.set_right_motor_speed(0)
        bot.camera.stop()
        print("Camera disabled.")

if __name__ == "__main__":
    main()
