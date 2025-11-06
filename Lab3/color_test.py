import time
import numpy as np
import math
from robot_systems.robot import HamBot

class BUG0:
    def __init__(self, bot: HamBot):
        self.bot = bot
        self.state = 'start'
        self.visited_points = []

        # GOAL and START POINT (mm)
        self.goal_x = 2400
        self.goal_y = 2400
        self.start_x = -750
        self.start_y = -750
        self.dist_to_goal = 0.0
        self.goal_angle = 0.0

        # ROBOT PARAMETERS
        self.wheel_r = 45.0    # mm
        self.wheel_d = 92.0    # mm
        
        # DISTANCE
        self.front_dist = 6.66666
        self.front_dist_right = 6.66666
        self.front_dist_left = 6.66666
        self.right_dist = 6.66666
        self.left_dist = 6.66666
        self.max_front = 0.0

        # ROBOT POSITION
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # degree

        # ENCODER
        left, right = self.bot.get_encoder_readings()
        self.prev_left_enc = left
        self.prev_right_enc = right

        # CAMERA
        self.COLOR = (255, 0, 200)
        self.TOLERANCE = 80
        self.frame = None
        self.bot.camera.set_target_colors([self.COLOR], tolerance=self.TOLERANCE)
        self.landmarks = None
        self.bot.camera.find_landmarks()

        # IMU initial heading
        self.initial_heading = self.bot.get_heading()
        print(f"[DEBUG] Initial heading: {self.initial_heading:.2f}°")

    # -------------------------------
    # Motor control
    # -------------------------------
    def stop_motors(self):
        try:
            self.bot.set_left_motor_speed(0)
            self.bot.set_right_motor_speed(0)
            print("[DEBUG] Motors stopped")
        except AttributeError:
            print("[DEBUG] Motor stop failed")

    # -------------------------------
    # Encoder + IMU based dead reckoning
    # -------------------------------
    def get_current_position(self):
        left_enc, right_enc = self.bot.get_encoder_readings()
        delta_left = left_enc - self.prev_left_enc
        delta_right = right_enc - self.prev_right_enc

        # distance traveled in mm
        d = (delta_left + delta_right) / 2 * self.wheel_r

        # heading from IMU
        theta = math.radians(self.bot.get_heading())

        # update position
        self.x += d * math.cos(theta)
        self.y += d * math.sin(theta)

        self.prev_left_enc = left_enc
        self.prev_right_enc = right_enc

        print(f"[DEBUG] Position updated: x={self.x:.1f}, y={self.y:.1f}, heading={self.bot.get_heading():.2f}°")
        return self.x, self.y
 
    # -------------------------------
    # FIND the Obj
    # -------------------------------
    # -------------------------------
    # Detect landmarks
    # -------------------------------
    def detect_landmark(self, target_color=(255,0,200), tolerance=80):
        """
        랜드마크를 찾고, 그 위치의 색상이 target_color 범위 내이면 True 반환.
        """
        try:
            landmarks = self.camera.find_landmarks()
            if not landmarks:
                print("[DEBUG] No landmarks found")
                return False

            # 첫 번째 랜드마크 위치 사용 (x, y 픽셀 좌표)
            x, y = landmarks[0][:2]  # find_landmarks()가 (x, y, ...) 반환한다고 가정

            frame = self.camera.get_frame()
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
        
        self.change_state('start')

        # goal angle 한 번만 계산
        self.goal_angle = self.calculate_goal_angle()

        while self.state != 'end':
            
            self.read_lidar()

            print(f"[DEBUG] Current state: {self.state}")

            if self.state == 'start':
                print('start')
                self.state = 'find'
                

            elif self.state == 'find':
                self. detect_landmark()


    


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