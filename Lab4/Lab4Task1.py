import time
import numpy as np
import math
from robot_systems.robot import HamBot

# =========================================================
#  LANDMARK POSITIONS  (mm)
# =========================================================
LANDMARKS = np.array([
    [-2400,  2400],   # Pink
    [ 2400,  2400],   # Baby Blue
    [-2400, -2400],   # Orange
    [ 2400, -2400],   # Green
])


# =========================================================
#  COLOR RANGES FOR LANDMARK DETECTION
# =========================================================
PINK       = (255, 20, 200)
BABY_BLUE  = (100, 180, 255)
ORANGE     = (255, 140, 50)
GREEN      = (50, 200, 50)

TOL = 80 


# =========================================================
#  LANDMARK BBOX
# =========================================================
def get_landmark_bbox(bot, idx):
    boxes = bot.camera.find_landmarks(index=idx)
    if len(boxes) == 0:
        return None
    return max(boxes, key=lambda b: (b[2]-b[0])*(b[3]-b[1]))


# =========================================================
#  CAMERA ANGLE CALCULATION
# =========================================================
CAMERA_FOV_DEG = 60.0   

def bbox_center_x(bbox):
    (x1, y1, x2, y2) = bbox
    return (x1 + x2) / 2.0

def pixel_to_angle(px, frame_width):
    center = frame_width / 2.0
    norm = (px - center) / center
    angle = norm * (CAMERA_FOV_DEG / 2.0)
    return angle


# =========================================================
#  LIDAR DISTANCE QUERY
# =========================================================
def get_distance_for_angle(bot, angle_deg):
    lidar = bot.lidar.get_scan()   
    idx = int((angle_deg % 360))
    return lidar[idx]


# =========================================================
#  LEAST-SQUARES TRILATERATION
# =========================================================
def trilaterate(landmarks, distances):
    x1, y1 = landmarks[0]
    d1 = distances[0]

    A = []
    b = []

    for i in range(1, len(landmarks)):
        xi, yi = landmarks[i]
        di = distances[i]

        A.append([2 * (xi - x1), 2 * (yi - y1)])
        b.append(d1**2 - di**2 - x1**2 + xi**2 - y1**2 + yi**2)

    A = np.array(A)
    b = np.array(b)

    p, *_ = np.linalg.lstsq(A, b, rcond=None)
    return p[0], p[1]


# =========================================================
#  POINT → GRID CELL
# =========================================================
CELL_SIZE = 600
GRID_MIN = -2400
GRID_MAX =  2400
GRID_SIZE = 5   

def point_to_cell(x, y):
    col = int((x - GRID_MIN) // CELL_SIZE)
    row = int((GRID_MAX - y) // CELL_SIZE)
    col = max(0, min(GRID_SIZE - 1, col))
    row = max(0, min(GRID_SIZE - 1, row))
    return row * GRID_SIZE + col + 1


# =========================================================
#  TRILATERATION MAIN FUNCTION
# =========================================================
def run_trilateration_once(bot):
    frame = bot.camera.raw_frame()
    h, w, _ = frame.shape

    distances = []

    # 1) 각 랜드마크 bbox로 거리 측정
    for idx in range(4):
        bbox = get_landmark_bbox(bot, idx)

        if bbox is None:
            print(f"[WARN] Landmark {idx} not found")
            distances.append(9999)
            continue

        px = bbox_center_x(bbox)
        angle = pixel_to_angle(px, w)
        dist = get_distance_for_angle(bot, angle)
        distances.append(dist)

    # 2) Trilateration 계산
    x, y = trilaterate(LANDMARKS, distances)

    # 3) Cell 표시
    cell = point_to_cell(x, y)

    print("\n================ Trilateration Result ================")
    print(f"Distances (mm): {distances}")
    print(f"Estimated Position: ({x:.1f}, {y:.1f}) mm")
    print(f"Grid Cell Index: {cell}")
    print("======================================================\n")

    return x, y, cell


# =========================================================
#  Program entry
# =========================================================
def main():
    try:
        bot = HamBot(lidar_enabled=True, camera_enabled=True)

        # landmark 색깔 등록은 bot 생성 이후 실행해야 함
        bot.camera.set_target_colors([PINK, BABY_BLUE, ORANGE, GREEN],
                                     tolerance=TOL)

        x, y, cell = run_trilateration_once(bot)

    except Exception as e:
        print("[DEBUG] Error:", e)
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
