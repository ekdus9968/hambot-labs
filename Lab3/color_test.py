import time
import numpy as np
from robot_systems.robot import HamBot

# -------------------------------
# 색상 테스트용
# -------------------------------

TARGET_COLOR = (255, 0, 200)   # 기존 목표 color
TOLERANCE = 80                 # ± 허용 오차
SAMPLE_X = 320                 # 테스트할 X 좌표 (프레임 중앙 등)
SAMPLE_Y = 240                 # 테스트할 Y 좌표

def detect_color_post(bot):
    posts = bot.camera.find_landmarks(color=TARGET_COLOR)
    if posts:
        print("FINDFIND")
        return True
    else:
        print("NONENONE")
        return False


def main():
    try:
        bot = HamBot(lidar_enabled=True, camera_enabled=True)
        bot.camera.set_target_colors([TARGET_COLOR], tolerance=TOLERANCE)
        
        time.sleep(1)  # 카메라 초기화 대기

        print("Camera color test started. Press Ctrl+C to stop.")
        
        while True:
            while True:
                scan = bot.get_range_image()
                if scan is not None and len(scan) > 0:
                    center_idx = len(scan) // 2
                    print(f"Front distance: {scan[center_idx]:.3f} m")
                else:
                    print("No LiDAR data received")
                
                forward_distance = np.min(scan[175:185])
                
                if np.isnan(forward_distance) or np.isinf(forward_distance):
                    forward_distance = 9999.999999
                
                frame = bot.camera.get_frame()  # get_image() → get_frame() 변경
                if frame is None:
                    continue

                H, W = frame.shape[:2]

                # 지정 좌표 픽셀 값 확인
                x = min(max(SAMPLE_X, 0), W-1)
                y = min(max(SAMPLE_Y, 0), H-1)
                pixel_color = frame[y, x]  # (R, G, B)
                
                r_diff = abs(int(pixel_color[0]) - TARGET_COLOR[0])
                g_diff = abs(int(pixel_color[1]) - TARGET_COLOR[1])
                b_diff = abs(int(pixel_color[2]) - TARGET_COLOR[2])
                
                within_tolerance = r_diff <= TOLERANCE and g_diff <= TOLERANCE and b_diff <= TOLERANCE
                
                print(f"Forward: {forward_distance:.3f}, Pixel ({x},{y}) RGB: {pixel_color}, Diff: (R:{r_diff}, G:{g_diff}, B:{b_diff}), Match: {within_tolerance}")
                detect_color_post(bot)
                time.sleep(0.2)


    except KeyboardInterrupt:
        print("Test stopped by user.")
    finally:
        bot.camera.stop()
        print("Camera disabled.")

if __name__ == "__main__":
    main()

