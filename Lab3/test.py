import time
import numpy as np
import math
from robot_systems.robot import HamBot



class BUG0:
    def __init__(self, bot: HamBot):
        self.bot = bot
        self.state = 'start'
        self.visited_points = []
        self.step_back_counter = 0

        # GOAL and START POINT (mm)
        self.goal_x = 2400
        self.goal_y = 2400
        self.start_x = -750
        self.start_y = -750
        self.dist_to_goal = 0.0

        # ROBOT PARAMETER
        self.wheel_r = 0.045
        self.wheel_d = 0.092

        # DISTANCE
        self.front_dist = 0.0
        self.front_dist_right = 0.0
        self.front_dist_left = 0.0
        self.right_dist = 0.0
        self.left_dist = 0.0
        self.max_front = 0.0

        # ROBOT POS
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # degree
        self.error_theta = 0.0

        # ENCODER
        left, right = self.bot.get_encoder_readings()
        self.prev_left_enc = left
        self.prev_right_enc = right
        
        # CAMERA
        self.cam = None
        self.COLOR = (180, 150, 170)   # attempting to make yellow
        self.COLOR_TOLERANCE  = 50
        self.bot.camera.set_target_colors([self.COLOR], tolerance=self.COLOR_TOLERANCE)

        
        self.lidar = None

    def stop_motors(self):
        self.bot.set_left_motor_velocity(0)
        self.bot.set_right_motor_velocity(0)
    # -------------------------------
    # GPS : get curr pos
    # -------------------------------
    # No need, hambot has lidar 
    # 로봇 위치 구하기 self.x self.y self. theta
    #if state = 'turn: theta 만 구하기
    # else : 간 거리 구하기 
    def get_current_position(self):
        if self.state == 'turn_to_goal':
            return self.x, self.y
        left_enc, right_enc = self.bot.get_encoder_readings()
        delta_left = left_enc - self.prev_left_enc
        delta_right = right_enc - self.prev_right_enc
        
        # 거리 이동 계산 (mm)
        d = (delta_left + delta_right) / 2 * self.wheel_r

        # 현재 heading
        theta = math.radians(self.bot.get_heading())

        # 위치 업데이트
        self.x += d * math.cos(theta)
        self.y += d * math.sin(theta)

        # 이전 엔코더 업데이트
        self.prev_left_enc = left_enc
        self.prev_right_enc = right_enc
        
        return self.x, self.y

    def update_position_and_distance(self):
        print("update_position_and_distance()")
        # HamBot에서 x, y 좌표 얻기 (예: dead-reckoning)
        self.x, self.y = self.get_current_position()  # 사용자 정의 함수 필요
        self.dist_to_goal = self.calculate_distance_to_goal()
        
    def change_state(self, new_state):
        print("change_state()")
        # 상태 바뀔 때 현재 위치 기록
        self.update_position_and_distance()
        self.visited_points.append((new_state, self.x, self.y, self.dist_to_goal))
        print(self.visited_points)
    
    # -------------------------------
    # goal angle 
    # -------------------------------
    def calculate_goal_angle(self):
        print("calculate_goal_angle()")
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        return math.degrees(math.atan2(dy, dx)) % 360



    # -------------------------------
    # distance to goal
    # -------------------------------
    def calculate_distance_to_goal(self):
        print("calculate_distance_to_goal()")
        return math.sqrt((self.goal_x - self.x)**2 + (self.goal_y - self.y)**2)


    # -------------------------------
    # lidar
    # -------------------------------
    def read_lidar(self):
        print("read_lidar()")
        
        if self.lidar is None or len(self.lidar) < 360:
            print("LiDAR data missing")
            return

        if self.lidar is None or len(self.lidar) < 360:
            print("LiDAR data missing")
            return

        # distance from each anlge
        self.front_dist = np.nanmin(self.lidar[175:185])  
        self.front_dist_right = np.nanmin(self.lidar[185:195])
        self.front_dist_left = np.nanmin(self.lidar[165:175])
        self.right_dist = np.nanmin(self.lidar[265:285])
        self.left_dist = np.nanmin(self.lidar[85:95])
        self.left_dist_front = np.nanmin(self.lidar[95:105])
        self.left_dist_back = np.nanmin(self.lidar[75:85])

        # NaN
        for name in ["front_dist", "front_dist_right", "front_dist_left", "right_dist", "left_dist"]:
            val = getattr(self, name)
            if np.isinf(val) or np.isnan(val) or val < 0.05:
                setattr(self, name, 666.666)
        self.max_front = max(self.front_dist, self.front_dist_left, self.front_dist_right)



    # -------------------------------
    # Turn to goal (LEFT)
    # -------------------------------
    def turn_to_goal(self, target_angle):
        print("turn_to_goal()")
        # based on IMU 
        current_heading = self.bot.get_heading()  # 0~360° 북 기준
        #target_angle = self.calculate_goal_angle()
        
        error = (target_angle - current_heading + 540) % 360 - 180

        if abs(error) < 5 :
            self.bot.stop_motors()
            return True
        elif self.front_dist_left < 500 :
            self.state = 'wall_following'
            self.bot.stop_motors()
            return True
            
        else:
            # 왼쪽으로만 회전
            self.bot.set_left_motor_velocity(-1)
            self.bot.set_right_motor_velocity(1)
            return False
        
    def turn_to_str(self, speed_left, speed_right):
        while self.front_dist < self.front_dist_left and self.front_dist < self.front_dist_right:
            self.bot.set_left_motor_velocity(speed_left)
            self.bot.set_right_motor_velocity(speed_right)
            break
        return 
        
        
    # -------------------------------
    # Find the goal
    # -------------------------------
    #if cqmera see pink
    def detect_color_post(self):
        posts = self.bot.camera.find_landmarks([self.COLOR])
        return bool(posts)

    
    def check_in_goal(self):
        print("check_in_goal()")
        dist = self.front_dist 
        if dist > 25:
            self.change_state('check in goal to go close')
            self.state = 'go_close'
            return 
        elif dist < 25 :
            self.change_state('check in goal to go fat')
            self.state = 'go_far'
            return 
        else :
            self.change_state('check in goal to end')
            self.state = 'end'
            return
        
    # -------------------------------
    # STATE
    # -------------------------------
    def run_state(self):
        print("run_state()")
        self.get_current_position()
        self.change_state('start0')
        while self.state != 'end':
            print(":::::::::START STATE:::::::::")
            
            if self.state == 'start':
                self.read_lidar()
                self.get_current_position()
                print(":::::::::START STATE:::::::::")
                print(":::::::::START STATE:::::::::")
                print(":::::::::START STATE:::::::::")
                print(":::::::::START STATE:::::::::")
                print(":::::::::START STATE:::::::::")
                self.change_state('start to turn to goal')
                self.state = 'turn_to_goal'
                self.change_state('start to turn to goal')
                
            elif self.state == 'turn_to_goal':
                self.read_lidar()
                self.get_current_position()
                print(":::::::::TURN TO GOAL:::::::::")
                print(":::::::::TURN TO GOAL:::::::::")
                print(":::::::::TURN TO GOAL:::::::::")
                print(":::::::::TURN TO GOAL:::::::::")
                print(":::::::::TURN TO GOAL:::::::::")
                target_angle = self.calculate_goal_angle()
                if self.turn_to_goal(target_angle):
                    self.change_state('turn to move to goal')
                    self.state = 'move_to_goal'
                
            elif self.state == 'move_to_goal':
                self.read_lidar()
                self.get_current_position()
                print(":::::::::MOVE TO GOAL:::::::::")
                print(":::::::::MOVE TO GOAL:::::::::")
                print(":::::::::MOVE TO GOAL:::::::::")
                print(":::::::::MOVE TO GOAL:::::::::")
                print(":::::::::MOVE TO GOAL:::::::::")
                if self.max_front > 600 :
                    self.bot.set_left_motor_velocity(4)
                    self.bot.set_right_motor_velocity(4)
                else: 
                    post = self.detect_color_post()
                    if post:
                        self.change_state('move to goal to go close')
                        self.state = 'go_close'
                    else :
                        self.turn_to_goal(90)
                        self.change_state('move to goal to wall following')
                        self.state = 'wall_following'
                
            # elif self.state == 'wall_following':
            #     self.read_lidar()
            #     self.get_current_position()
            #     print("/////////WALL FOLLOWING/////////")
            #     print("/////////WALL FOLLOWING/////////")
            #     print("/////////WALL FOLLOWING/////////")
            #     print("/////////WALL FOLLOWING/////////")
            #     print("/////////WALL FOLLOWING/////////")
            #     if self.left_dist < 600:
            #         self.bot.set_left_motor_velocity(4)
            #         self.bot.set_right_motor_velocity(4)
            #         if self.front_dist_left < self.front_dist_right:
            #             self.stop_motors()
            #             self.turn_to_str(1, -1)
            #         elif self.front_dist_left < self.front_dist_right:
            #             self.stop_motors()
            #             self.turn_to_str(-1, 1)
                        
            #     else :
            #         self.change_state('wall following to turn to goal')
            #         self.state = 'turn_to_goal'
                
            # elif self.state == 'go_close':
            #     self.get_current_position()
            #     self.read_lidar()
            #     print("~~~~~~~~~GO CLOSER~~~~~~~~~~")
            #     print("~~~~~~~~~GO CLOSER~~~~~~~~~~")
            #     print("~~~~~~~~~GO CLOSER~~~~~~~~~~")
            #     print("~~~~~~~~~GO CLOSER~~~~~~~~~~")
            #     print("~~~~~~~~~GO CLOSER~~~~~~~~~~")
            #     if self.front_dist> 250:
            #         self.bot.set_left_motor_velocity(2)
            #         self.bot.set_right_motor_velocity(2)
            #     else:
            #         self.change_state('go close to check in goal')
            #         self.state = 'check_in_goal'
                
            # elif self.state == 'go_far':
            #     self.get_current_position()
            #     self.read_lidar()
            #     print("~~~~~~~~~GO FAR~~~~~~~~~~")
            #     print("~~~~~~~~~GO FAR~~~~~~~~~~")
            #     print("~~~~~~~~~GO FAR~~~~~~~~~~")
            #     print("~~~~~~~~~GO FAR~~~~~~~~~~")
            #     print("~~~~~~~~~GO FAR~~~~~~~~~~")
            #     if self.front_dist  < 250:
            #         self.bot.set_left_motor_velocity(-2)
            #         self.bot.set_right_motor_velocity(-2)
            #     else:
            #         self.change_state('go far to check in goal')
            #         self.state = 'check_in_goal'
                
            # elif self.state == 'check_in_goal':
            #     self.get_current_position()
            #     self.read_lidar()
            #     print("-------CHECK IN GOAL---------")
            #     print("-------CHECK IN GOAL---------")
            #     print("-------CHECK IN GOAL---------")
            #     print("-------CHECK IN GOAL---------")
            #     print("-------CHECK IN GOAL---------")
            #     self.check_in_goal()
                
            # elif self.state == 'end':
            #     self.get_current_position()
            #     print("----------END---------")
            #     self.bot.set_left_motor_velocity(0)
            #     self.bot.set_right_motor_velocity(0)
            #     self.stop_motors()
            #     break
                

def main():
    """Main entry point"""
    try:
        bot = HamBot(lidar_enabled=True, camera_enabled=True)
        print("HamBot initialized and ready for wall following.")
        # Create wall follower
        follower = BUG0(bot)
        
        # Follow wall for 20 seconds
        follower.run_state()
        
        # Cleanup
        follower.stop_motors()
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()