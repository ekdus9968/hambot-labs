import time
import numpy as np
import math
from robot_systems.robot import HamBot
#check
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
        self.front_dist = 666.0
        self.front_dist_right = 666.0
        self.front_dist_left = 666.0
        self.right_dist = 666.0
        self.left_dist = 666.0
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
    # Update distance to goal
    # -------------------------------
    def update_position_and_distance(self):
        self.get_current_position()
        self.dist_to_goal = self.calculate_distance_to_goal()
        print(f"[DEBUG] Distance to goal: {self.dist_to_goal:.1f} mm")

    # -------------------------------
    # (DONE)Change state
    # -------------------------------
    def change_state(self, new_state):
        self.update_position_and_distance()
        self.visited_points.append((new_state, self.x, self.y, self.dist_to_goal))
        self.state = new_state
        print(f"[DEBUG] State changed to: {new_state}")

    # -------------------------------
    # (DONE)Goal calculations
    # -------------------------------
    def calculate_goal_angle(self):
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        goal_angle = math.degrees(math.atan2(dy, dx)) % 360
        print(f"[DEBUG] Goal angle: {goal_angle:.2f}°")
        return goal_angle

    def calculate_distance_to_goal(self):
        return math.sqrt((self.goal_x - self.x)**2 + (self.goal_y - self.y)**2)

    # -------------------------------
    # (DONE)LIDAR
    # -------------------------------
    def read_lidar(self):
        self.lidar = self.bot.get_range_image()
        if self.lidar is None or len(self.lidar) < 360:
            print("[DEBUG] No LiDAR data received")
            return

        # Front and side distances
        self.front_dist = np.nanmin(self.lidar[175:185])
        self.front_dist_right = np.nanmin(self.lidar[185:195])
        self.front_dist_left = np.nanmin(self.lidar[165:175])
        self.right_dist = np.nanmin(self.lidar[265:285])
        self.left_dist = np.nanmin(self.lidar[85:95])

        # Handle invalid readings
        for name in ["front_dist", "front_dist_right", "front_dist_left", "right_dist", "left_dist"]:
            val = getattr(self, name)
            if np.isinf(val) or np.isnan(val) or val < 0.05:
                setattr(self, name, 666.666)

        self.max_front = max(self.front_dist, self.front_dist_left, self.front_dist_right)
        print(f"[DEBUG] LiDAR - Front: {self.front_dist:.1f}, Left: {self.left_dist:.1f}, Right: {self.right_dist:.1f}")

    # -------------------------------
    # (DONE)Turn to goal - 왼쪽(반시계) 고정 속도 회전
    # -------------------------------
    def turn_to_goal(self, target_angle):
        current_heading = self.bot.get_heading()

        # 왼쪽 회전 전용 error 계산 (0~180°)
        error = abs((current_heading - target_angle + 360) % 360 - 180)
        if error > 180:
            error = 360 - error  # 항상 최소 각도

        print(f"[DEBUG] Turning left only - Current: {current_heading:.2f}, Target: {target_angle:.2f}, Error: {error:.2f}")

        if error < 3:  # ±3° 안이면 멈춤
            self.bot.set_left_motor_speed(0)
            self.bot.set_right_motor_speed(0)
            print("[DEBUG] Reached target heading, motors stopped")
            return True
        else:
            # HamBot 모터 범위 내 고정 속도
            fixed_speed = 4
            self.bot.set_left_motor_speed(-fixed_speed)   # 왼쪽 모터 뒤
            self.bot.set_right_motor_speed(fixed_speed)   # 오른쪽 모터 앞으로
            return False

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

            elif self.state == 'wall_following':
                self.update_position_and_distance()
                if self.right_dist > 200:
                    self.bot.set_left_motor_speed(0.5)
                    self.bot.set_right_motor_speed(0.25)
                else:
                    self.bot.set_left_motor_speed(0.25)
                    self.bot.set_right_motor_speed(0.5)

                if self.dist_to_goal < 50:
                    self.update_position_and_distance()
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