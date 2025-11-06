import time
import numpy as np
import math
from robot_systems.robot import HamBot
class BUG0:
    def __init__(self, bot: HamBot):
        self.bot = bot
        self.state = 'start'
        self.COLOR = (255,0,200)
        self.TOLERANCE = 80
        self.bot.camera.set_target_colors([self.COLOR], tolerance=self.TOLERANCE)

    # -------------------------------
# Detect landmarks with target color
# -------------------------------
    def detect_landmark(self, target_color=(255,0,200), tolerance=80):
        """
        랜드마크를 찾고, 그 위치의 색상이 target_color 범위 내이면 True 반환.
        """
        try:
            # find_landmarks()는 Landmark 객체 리스트를 반환
            landmarks = self.bot.camera.find_landmarks()
            if not landmarks:
                print("[DEBUG] No landmarks found")
                return False

            # 첫 번째 Landmark 객체 사용
            landmark = landmarks[0]
            x, y = landmark.x, landmark.y  # Landmark 속성 사용

            # 카메라 프레임 가져오기
            frame = self.bot.camera.get_frame()
            if frame is None:
                print("[DEBUG] Camera frame not available")
                return False

            H, W = frame.shape[:2]
            x = min(max(0, int(x)), W-1)
            y = min(max(0, int(y)), H-1)

            pixel_color = frame[y, x]  # (R,G,B)
            r_diff = abs(int(pixel_color[0]) - target_color[0])
            g_diff = abs(int(pixel_color[1]) - target_color[1])
            b_diff = abs(int(pixel_color[2]) - target_color[2])

            within_tolerance = r_diff <= tolerance and g_diff <= tolerance and b_diff <= tolerance

            if within_tolerance:
                print(f"[DEBUG] Landmark with target color found at ({x},{y}) RGB: {pixel_color}")
                return True
            else:
                print(f"[DEBUG] Landmark found but color does not match at ({x},{y}) RGB: {pixel_color}")
                return False

        except Exception as e:
            print(f"[DEBUG] Error in detect_landmark_color: {e}")
            return False

    def run_state(self):
        self.state = 'start'

        while True:
            print(f"[DEBUG] Current state: {self.state}")

            if self.state == 'start':
                print("[DEBUG] Start state -> switch to find")
                self.state = 'find'

            # 상태 머신 내
            elif self.state == 'find':
                if self.detect_landmark(target_color=self.COLOR, tolerance=self.TOLERANCE):
                    print("[DEBUG] Target landmark detected!")
                    self.change_state('move_to_goal')
                else:
                    print("[DEBUG] Keep searching...")

                        
           


# -------------------------------
# Main
# -------------------------------
def main():
    try:
        bot = HamBot(lidar_enabled=True, camera_enabled=True)
        TARGET_COLOR = (255, 0, 200)
        TOLERANCE = 80
        bot.camera.set_target_colors([TARGET_COLOR], tolerance=TOLERANCE)
        
        follower = BUG0(bot)
        follower.run_state()

    except Exception as e:
        print(f"[DEBUG] Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
#back up