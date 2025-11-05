# 1m = 1000mm, 100cm = 1m
# 60 cm = 600 mm 
# get_image() -> np.ndarray | None 최신 RGB 프레임(모양 (H, W, 3)) 을 반환하거나 None아직 사용할 수 없는 경우 반환합니다.
# Landmark(center=(x, y), width=w, height=h)
# set_target_colors(colors, tolerance=0.05) 감지할 대상 색상을 하나 이상 설정합니다.
# colors: (R, G, B)튜플 또는 튜플 목록(채널당 0~255)
# tolerance: float은 [0, 1]각 채널 주변의 대칭적 ±허용 오차로 해석됩니다.
#CHECK
# stop_camera()
import time
import numpy as np
import math
from robot_systems.robot import HamBot

YELLOW = (255, 220, 0)   # attempting to make yellow
color_tolerance  = 45  

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
        self.bot.camera.enable(32)  # 32 ms timestep
        self.bot.camera.set_target_colors([(255, 255, 0)], tolerance=45) #color
        self.cam = self.bot.camera.get_image()
        self.frame_height = 0.0
        self.frame_width = 0.0

        

    def stop_motors(self):
        self.bot.set_left_motor_velocity(0)
        self.bot.set_right_motor_velocity(0)
    # -------------------------------
    # GPS
    # -------------------------------
    # No need, hambot has lidar 



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
        lidar = self.bot.get_lidar_range_image()

        if lidar is None or len(lidar) < 360:
            print("LiDAR data missing")
            return

        # distance from each anlge
        self.front_dist = np.nanmin(lidar[175:185])  
        self.front_dist_right = np.nanmin(lidar[185:195])
        self.front_dist_left = np.nanmin(lidar[165:175])
        self.right_dist = np.nanmin(lidar[265:285])
        self.left_dist = np.nanmin(lidar[85:95])
        self.left_dist_front = np.nanmin(lidar[95:105])
        self.left_dist_back = np.nanmin(lidar[75:85])

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
            self.bot.stop()
            return True
        elif self.front_dist_left < 500 :
            self.state = 'wall_following'
            self.bot.stop()
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
    #get frame size from the camera
    def get_frame_size(self):
        self.cam = self.bot.camera.get_image()  # (H, W, 3) 또는 None
        if self.cam is None:
            return None  # 아직 프레임 준비 안 됨

        # 전체 프레임 크기
        H, W = self.cam.shape[:2]  # height, width
        self.frame_height = H
        self.frame_width = W

        # 필요하면 프레임 첫 행도 따로 저장 가능
        self.first_row = self.cam[0]  # shape = (W, 3)

        # 반환값은 전체 프레임
        return self.cam

    
    #if cqmera see yellow 
    def detect_yellow_post(self):
        print("detect_yellow_post()")
        posts = self.bot.camera.find_landmarks(YELLOW, tolerance=color_tolerance)
        if not posts:
            return None
        # post_fields func
        def post_fields(p):
            x = p.get("x", 0)
            y = p.get("y", 0)
            w = p.get("w", 0)
            h = p.get("h", 0)
            return int(x), int(y), int(w), int(h)

        yposts = [post_fields(p) for p in posts]
        x, y, w, h = max(yposts, key=lambda t: t[2]*t[3])  # 가장 큰 노란 영역 선택
        return (x, y, w, h)
    
    def check_in_goal(self):
        print("check_in_goal()")
        dist = self.front_dist - 300 
        if dist > 25:
            self.state = 'go_close'
            return 
        elif dist < 25 :
            self.state = 'go_far'
            return 
        else :
            self.state = 'end'
            return
        
    def run_state(self):
        print("run_state()")
        while self.state != 'end':
            print(":::::::::START STATE:::::::::")
            
            if self.state == 'start':
                self.read_lidar()
                print(":::::::::START STATE:::::::::")
                print(":::::::::START STATE:::::::::")
                print(":::::::::START STATE:::::::::")
                print(":::::::::START STATE:::::::::")
                print(":::::::::START STATE:::::::::")
                self.dist_to_goal = self.calculate_distance_to_goal()
                self.state = 'turn_to_goal'
                
            elif self.state == 'turn_to_goal':
                self.read_lidar()
                print(":::::::::TURN TO GOAL:::::::::")
                print(":::::::::TURN TO GOAL:::::::::")
                print(":::::::::TURN TO GOAL:::::::::")
                print(":::::::::TURN TO GOAL:::::::::")
                print(":::::::::TURN TO GOAL:::::::::")
                target_angle = self.calculate_goal_angle()
                if self.turn_to_goal(target_angle):
                    self.state = 'move_to_goal'
                
            elif self.state == 'move_to_goal':
                self.read_lidar()
                print(":::::::::MOVE TO GOAL:::::::::")
                print(":::::::::MOVE TO GOAL:::::::::")
                print(":::::::::MOVE TO GOAL:::::::::")
                print(":::::::::MOVE TO GOAL:::::::::")
                print(":::::::::MOVE TO GOAL:::::::::")
                if self.max_front > 600 :
                    self.bot.set_left_motor_velocity(4)
                    self.bot.set_right_motor_velocity(4)
                else: 
                    post = self.detect_yellow_post()
                    if post:
                        self.state = 'go_close'
                    else :
                        self.turn_to_goal(90)
                        self.state = 'wall_following'
                
            elif self.state == 'wall_following':
                self.read_lidar()
                print("/////////WALL FOLLOWING/////////")
                print("/////////WALL FOLLOWING/////////")
                print("/////////WALL FOLLOWING/////////")
                print("/////////WALL FOLLOWING/////////")
                print("/////////WALL FOLLOWING/////////")
                if self.left_dist < 600:
                    self.bot.set_left_motor_velocity(4)
                    self.bot.set_right_motor_velocity(4)
                    if self.front_dist_left < self.front_dist_right:
                        self.stop_motors()
                        self.turn_to_str(1, -1)
                    elif self.front_dist_left < self.front_dist_right:
                        self.stop_motors()
                        self.turn_to_str(-1, 1)
                        
                else :
                    self.state = 'turn_to_goal'
                
            elif self.state == 'go_close':
                self.read_lidar()
                print("~~~~~~~~~GO CLOSER~~~~~~~~~~")
                print("~~~~~~~~~GO CLOSER~~~~~~~~~~")
                print("~~~~~~~~~GO CLOSER~~~~~~~~~~")
                print("~~~~~~~~~GO CLOSER~~~~~~~~~~")
                print("~~~~~~~~~GO CLOSER~~~~~~~~~~")
                target_dist = 300
                if abs(self.front_dist - target_dist) > 250:
                    self.bot.set_left_motor_velocity(2)
                    self.bot.set_right_motor_velocity(2)
                else:
                    self.state = 'check_in_goal'
                
            elif self.state == 'go_far':
                self.read_lidar()
                print("~~~~~~~~~GO FAR~~~~~~~~~~")
                print("~~~~~~~~~GO FAR~~~~~~~~~~")
                print("~~~~~~~~~GO FAR~~~~~~~~~~")
                print("~~~~~~~~~GO FAR~~~~~~~~~~")
                print("~~~~~~~~~GO FAR~~~~~~~~~~")
                target_dist = 300
                if abs(self.front_dist - target_dist) < 250:
                    self.bot.set_left_motor_velocity(-2)
                    self.bot.set_right_motor_velocity(-2)
                else:
                    self.state = 'check_in_goal'
                
            elif self.state == 'check_in_goal':
                self.read_lidar()
                print("-------CHECK IN GOAL---------")
                print("-------CHECK IN GOAL---------")
                print("-------CHECK IN GOAL---------")
                print("-------CHECK IN GOAL---------")
                print("-------CHECK IN GOAL---------")
                self.check_in_goal()
                
            elif self.state == 'end':
                print("----------END---------")
                self.bot.set_left_motor_velocity(0)
                self.bot.set_right_motor_velocity(0)
                self.stop_motors()
                break
                

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
        follower.stop()
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()