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

    def detect_landmark(self, target_color=(255,0,200), tolerance=80):
        try:
            landmarks = self.bot.camera.find_landmarks()
            if not landmarks:
                print("[DEBUG] No landmarks found")
                return False

            x, y = landmarks[0][:2]

            frame = self.bot.camera.get_frame()
            if frame is None:
                print("[DEBUG] Camera frame not available")
                return False

            H, W = frame.shape[:2]
            x = min(max(0,int(x)), W-1)
            y = min(max(0,int(y)), H-1)

            pixel_color = frame[y,x]
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

            elif self.state == 'find':
                found = self.detect_landmark()
                if found:
                    print("[DEBUG] Target landmark color detected!")
                    break  # 종료 예시
                else:
                    print("[DEBUG] Keep searching...")
            


    
    # -------------------------------
    # Main state machine
    # -------------------------------
    def run_state(self):
        
        self.change_state('start')

        # goal angle 한 번만 계산
        self.goal_angle = self.calculate_goal_angle()

        while self.state != 'end':
            
            self.read_lidar()

            print(f"[DEBUG] Current state: {self.state}")

            if self.state == 'start':
                self.update_position_and_distance()
                self.change_state('turn_to_goal')

            elif self.state == 'turn_to_goal':
                if self.turn_to_goal(self.goal_angle):
                    self.change_state('move_to_goal')
#********************************그리고 애초에 계속 찾아가야함+ 찾으러 가면서도 오왼오왼 맞추기 
            elif self.state == 'move_to_goal':
                self.update_position_and_distance()
                if self.front_dist > 600 :
                    self.bot.set_left_motor_speed(3)
                    self.bot.set_right_motor_speed(3)
                    print("[DEBUG] Moving forward")
                else:
                    self.stop_motors()
                    self.change_state('wall_following')

                if self.dist_to_goal < 50:
                    self.change_state('end')


            time.sleep(0.05)

        self.stop_motors()
        print("[DEBUG] Reached goal, stopping.")

# -------------------------------
# Main
# -------------------------------
def main():
    try:
        bot = HamBot(lidar_enabled=True, camera_enabled=True)
        bot.camera.set_target_colors([(255, 0, 200)], tolerance=80)

        follower = BUG0(bot)
        follower.run_state()
        

    except Exception as e:
        print(f"[DEBUG] Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
#back up