import time
import numpy as np
from robot_systems.robot import HamBot

# =========================================================
# LANDMARKS POSITIONS (mm)
# =========================================================
LANDMARKS = np.array([
    [-2400,  2400],   # Pink
    [ 2400,  2400],   # Baby Blue
    [-2400, -2400],   # Orange
    [ 2400, -2400],   # Green
])

# =========================================================
# COLOR RANGES
# =========================================================
PINK       = (255, 20, 200)
BABY_BLUE  = (100, 180, 255)
ORANGE     = (255, 140, 50)
GREEN      = (50, 200, 50)
TOL = 80

# =========================================================
# CAMERA / LIDAR UTILITIES
# =========================================================
CAMERA_FOV_DEG = 60.0

def get_landmark_bbox(bot, idx):
    boxes = bot.camera.find_landmarks(index=idx)
    if len(boxes) == 0:
        return None
    return max(boxes, key=lambda b: (b[2]-b[0])*(b[3]-b[1]))

def bbox_center_x(bbox):
    x1, y1, x2, y2 = bbox
    return (x1 + x2) / 2.0

def pixel_to_angle(px, frame_width):
    center = frame_width / 2.0
    norm = (px - center) / center
    return norm * (CAMERA_FOV_DEG / 2.0)

def get_distance_for_angle(bot, angle_deg):
    lidar = bot.lidar.get_scan()   # 360-array
    idx = int(angle_deg % 360)
    return lidar[idx]

# =========================================================
# TRILATERATION LEAST-SQUARES
# =========================================================
def trilaterate(landmarks, distances):
    x1, y1 = landmarks[0]
    d1 = distances[0]
    A = []
    b = []
    for i in range(1, len(landmarks)):
        xi, yi = landmarks[i]
        di = distances[i]
        A.append([2*(xi-x1), 2*(yi-y1)])
        b.append(d1**2 - di**2 - x1**2 + xi**2 - y1**2 + yi**2)
    A = np.array(A)
    b = np.array(b)
    p, *_ = np.linalg.lstsq(A, b, rcond=None)
    return p[0], p[1]

# =========================================================
# POINT → GRID CELL
# =========================================================
CELL_SIZE = 600
GRID_MIN = -2400
GRID_MAX =  2400
GRID_SIZE = 5

def point_to_cell(x, y):
    col = int((x - GRID_MIN) // CELL_SIZE)
    row = int((GRID_MAX - y) // CELL_SIZE)
    col = max(0, min(GRID_SIZE-1, col))
    row = max(0, min(GRID_SIZE-1, row))
    return row * GRID_SIZE + col + 1

# =========================================================
# FULL 360 ROTATION TRILATERATION
# =========================================================
def run_trilateration_full_rotation(bot, rotation_step_deg=30, step_time=0.5):
    """
    Rotate robot 360 degrees in steps, detect landmarks, then perform trilateration.
    """
    bot.camera.set_target_colors([PINK, BABY_BLUE, ORANGE, GREEN], tolerance=TOL)
    all_distances = [None]*4
    detected = [False]*4

    steps = int(360 / rotation_step_deg)
    for _ in range(steps):
        frame = bot.camera.raw_frame()
        h, w, _ = frame.shape

        # 각 랜드마크 검사
        for idx in range(4):
            if detected[idx]:
                continue
            bbox = get_landmark_bbox(bot, idx)
            if bbox is None:
                continue
            px = bbox_center_x(bbox)
            angle = pixel_to_angle(px, w)
            dist = get_distance_for_angle(bot, angle)
            all_distances[idx] = dist
            detected[idx] = True
            print(f"[INFO] Landmark {idx} detected at distance {dist:.1f} mm")

        # 회전
        fixed_speed = 4.0
        bot.set_left_motor_speed(-fixed_speed)
        bot.set_right_motor_speed(fixed_speed)
        time.sleep(step_time)  # 회전 시간
        bot.set_left_motor_speed(0.0)
        bot.set_right_motor_speed(0.0)

        if all(detected):
            break

    # 감지되지 않은 랜드마크는 큰 값으로 설정
    for i in range(4):
        if all_distances[i] is None:
            all_distances[i] = 9999
            print(f"[WARN] Landmark {i} not detected, setting distance to 9999")

    # Trilateration
    x, y = trilaterate(LANDMARKS, all_distances)
    cell = point_to_cell(x, y)

    print("\n=== Trilateration Result After Full Rotation ===")
    print(f"Distances: {all_distances}")
    print(f"Estimated Position: ({x:.1f}, {y:.1f}) mm")
    print(f"Grid Cell Index: {cell}")
    print("==============================================\n")

    return x, y, cell

# =========================================================
# MAIN ENTRY
# =========================================================
def main():
    try:
        bot = HamBot(lidar_enabled=True, camera_enabled=True)
        x, y, cell = run_trilateration_full_rotation(bot)
    except Exception as e:
        print("[DEBUG] Error:", e)
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
