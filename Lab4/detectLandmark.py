# -----------------------------------------------
#  Detect 4 color landmarks separately
# -----------------------------------------------
def detect_all_landmarks(bot):
    """
    4개 랜드마크(PINK, BLUE, ORANGE, GREEN)를 각각 탐지하고
    각 랜드마크의 bbox 중심 (cx, cy)를 반환한다.

    return:
        positions = [
            (found, cx, cy),   # Pink
            (found, cx, cy),   # Blue
            (found, cx, cy),   # Orange
            (found, cx, cy)    # Green
        ]
    """
    results = []

    for idx in range(4):
        boxes = bot.camera.find_landmarks(index=idx)

        if not boxes:
            results.append((False, None, None))
            print(f"[DEBUG] Landmark {idx} NOT FOUND")
            continue

        # 가장 큰 landmark bbox 선택
        bbox = max(boxes, key=lambda b: (b[2] - b[0]) * (b[3] - b[1]))

        x1, y1, x2, y2 = bbox
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        print(f"[DEBUG] Landmark {idx} FOUND at center ({cx},{cy}) bbox={bbox}")
        results.append((True, cx, cy))

    return results
def main():
    try:
        bot = HamBot(lidar_enabled=True, camera_enabled=True)

        # 4개 색 등록 (index=0,1,2,3 순서가 중요)
        COLORS = [
            (255, 20, 200),   # Pink
            (100, 180, 255),  # Baby Blue
            (255, 140, 50),   # Orange
            (50, 200, 50)     # Green
        ]
        TOL = 80
        bot.camera.set_target_colors(COLORS, tolerance=TOL)

        # --- 4개 랜드마크 탐지 테스트 ---
        print("=== Detecting All Landmarks ===")
        results = detect_all_landmarks(bot)

        # 확인 출력
        for i, (found, cx, cy) in enumerate(results):
            if found:
                print(f"Landmark {i} detected at ({cx},{cy})")
            else:
                print(f"Landmark {i} not detected")

    except Exception as e:
        print(f"[DEBUG] Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
