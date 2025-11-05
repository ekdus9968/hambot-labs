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

    # -------------------------------
    # Motor control
    # -------------------------------
    def stop_motors(self):
        try:
            self.bot.set_left_motor_speed(0)
            self.bot.set_right_motor_speed(0)
        except AttributeError:
            print("Motor stop failed")

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

        return self.x, self.y

    # -------------------------------
    # Update distance to goal
    # -------------------------------
    def update_position_and_distance(self):
        self.get_current_position()
        self.dist_to_goal = self.calculate_distance_to_goal()

    # -------------------------------
    # Change state
    # -------------------------------
    def change_state(self, new_state):
        self.update_position_and_distance()
        self.visited_points.append((new_state, self.x, self.y, self.dist_to_goal))
        self.state = new_state
        print(f"State changed to: {new_state}, Position: ({self.x:.1f}, {self.y:.1f}), Distance to goal: {self.dist_to_goal:.1f}")

    # -------------------------------
    # Goal calculations
    # -------------------------------
    def calculate_goal_angle(self):
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        goal_angle = math.degrees(math.atan2(dy, dx)) % 360
        return goal_angle

    def calculate_distance_to_goal(self):
        return math.sqrt((self.goal_x - self.x)**2 + (self.goal_y - self.y)**2)

    # -------------------------------
    # LIDAR
    # -------------------------------
    def read_lidar(self):
        self.lidar = self.bot.get_range_image()
        if self.lidar is None or len(self.lidar) < 360:
            print("No LiDAR data received")
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

    # -------------------------------
    # Turn to goal with proportional control
    # -------------------------------
    def turn_to_goal(self, target_angle):
        current_heading = self.bot.get_heading()
        error = (target_angle - current_heading + 540) % 360 - 180

        if abs(error) < 5:
            self.stop_motors()
            return True
        else:
            # Simple proportional control
            speed = 1.0
            error > 0
            self.bot.set_left_motor_speed(-speed)
            self.bot.set_right_motor_speed(speed)
            return False

    # -------------------------------
    # Camera: detect landmarks
    # -------------------------------
    def detect_landmark(self):
        self.frame = self.bot.camera.get_frame()
        if self.frame is None:
            return False

        landmarks = self.bot.camera.find_landmarks()
        if landmarks is not None and len(landmarks) > 0:
            print("Landmark detected!")
            return True
        return False

    # -------------------------------
    # Main state machine
    # -------------------------------
    def run_state(self):
        self.update_position_and_distance()
        self.change_state('start')
        while self.state != 'end':
            self.get_current_position()
            self.read_lidar()

            if self.state == 'start':
                self.change_state('turn_to_goal')

            elif self.state == 'turn_to_goal':
                goal_angle = self.calculate_goal_angle()
                # adjust with initial heading
                target_angle = (goal_angle - self.initial_heading) % 360
                if self.turn_to_goal(target_angle):
                    self.change_state('move_to_goal')

            elif self.state == 'move_to_goal':
                # move forward if no obstacle
                if self.front_dist > 200:
                    self.bot.set_left_motor_speed(1.0)
                    self.bot.set_right_motor_speed(1.0)
                else:
                    self.stop_motors()
                    self.change_state('wall_following')

                if self.dist_to_goal < 50:
                    self.change_state('end')

            elif self.state == 'wall_following':
                # simple right-wall following
                if self.right_dist > 200:
                    self.bot.set_left_motor_speed(1.0)
                    self.bot.set_right_motor_speed(0.5)
                else:
                    self.bot.set_left_motor_speed(0.5)
                    self.bot.set_right_motor_speed(1.0)

                if self.dist_to_goal < 50:
                    self.change_state('end')

            time.sleep(0.05)

        self.stop_motors()
        print("Reached goal, stopping.")


def main():
    try:
        bot = HamBot(lidar_enabled=True, camera_enabled=True)
        bot.camera.set_target_colors([(255, 0, 200)], tolerance=80)

        follower = BUG0(bot)
        follower.run_state()

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
