"""
HamBot Wall Following PID Controller (Real Robot Version)
Author: Seyoung Kan
Date: 2025-10-10
"""

import time
import math
import numpy as np
from robot_systems.robot import HamBot  # HamBot 실물용 라이브러리
# ========================
# ROBOT Parameters
# ========================
axel_length = 0.184
wheel_radius = 0.090
dt = 0.032 

# ========================
# PID gains
# ========================
Kp = 5
Ki = 0.0
Kd = 5

# Target distances (meters)
target_D_f = 0.5
target_D_r = 0.4


# PID state variables
I_f = 0.0
E_prev_f = 0.0
I_r = 0.0
E_prev_r = 0.0
E_f = 0.0
E_r = 0.0

# ========================
# Robot setup
# ========================
bot = HamBot(lidar_enabled=True, camera_enabled=False)
print("HamBot initialized and ready for wall following.")


# ========================
# REST PID
# ========================
def resetPID(bot):
    global prev_error, integral, prev_angle_error, angleIntegral
    print("restPID")
    prev_error = 0.0
    integral = 0.0
    prev_angle_error = 0.0
    angleIntegral = 0.0

# ========================
# move_forward
# ========================
def move_forward(bot, speed=20, duration=2.0):
    """Move straight forward for 'duration' seconds"""
    print("Moving forward...")
    bot.set_left_motor_speed(speed)
    bot.set_right_motor_speed(speed)
    time.sleep(duration)
    bot.stop_motors()
    #withWall(bot)

# def safe_distance(value, max_range=9.5):
#     """
#     LiDAR 값에서 NaN, Inf를 처리하고 최대 거리 제한.
#     """
#     if math.isinf(value) or math.isnan(value) or value < 0.05:
#         return max_range
#     return min(value, max_range)

    
    
# ========================
# Rotation
# ========================

def move_arc(bot, R, theta, direction="CCW", max_v=50):
    # cal r/l turn
    if direction.upper() == "CCW":
        d_left = (R - axel_length/2) * theta
        d_right = (R + axel_length/2) * theta
    else:
        d_left = (R + axel_length/2) * theta
        d_right = (R - axel_length/2) * theta
    
    bot.reset_encoders()  
    init_l = bot.get_left_encoder_reading()
    init_r = bot.get_right_encoder_reading()
    
    v_ratio_l = d_left / max(abs(d_left), abs(d_right))
    v_ratio_r = d_right / max(abs(d_left), abs(d_right))
    
    bot.set_left_motor_speed(max_v * v_ratio_l)
    bot.set_right_motor_speed(max_v * v_ratio_r)
    
    lidar = bot.get_range_image()

        # 기본 예외 처리 (라이다 데이터 존재 확인)
    if lidar is None or len(lidar) < 360:
        center_idx = len(lidar) // 2
        print(f"Front distance: {lidar[center_idx]:.3f} m")
    else:
        print("No LiDAR data received")

    D_r = np.nanmin(lidar[265:285])  / 600 # right
    if np.isinf(D_r) or np.isnan(D_r) or D_r < 0.05:
        D_r = 1.0
    
    while True:
        l_delta = bot.get_left_encoder_reading() - init_l
        r_delta = bot.get_right_encoder_reading() - init_r
        
        d_l = wheel_radius * l_delta
        d_r = wheel_radius * r_delta
        
        #each loop, cal rest of D ratio in live 
        #when i only use ratio var, it stops faster
        l_remain = d_left - d_l
        r_remain = d_right - d_r
        
        v_ratio_l = l_remain / max(abs(l_remain), abs(r_remain))
        v_ratio_r = r_remain / max(abs(l_remain), abs(r_remain))
        
        bot.set_left_motor_speed(max_v * v_ratio_l)
        bot.set_right_motor_speed(max_v * v_ratio_r)
        
        print("ARC:: L: ", d_l)
        print("ARC:: Target L: ", d_left )
        print("ARC:: Target R: ", d_right )
        
        if (abs(d_l) >= abs(d_left) and abs(d_r) >= abs(d_right)) or D_r < 0.6:
            bot.set_left_motor_speed(0)
            bot.set_right_motor_speed(0)
            bot.stop_motors()
            break
#         time.sleep(0.01)    
# def rotate(bot, radianAngle):
#     """
#     Rotate the robot by a given angle (in radians).
#     Positive angle -> left (CCW), Negative angle -> right (CW)
#     """
#     print("Rotate start...")
#     resetPID(bot)
#     base_speed = 5 # 회전 속도 (너무 빠르면 overshoot)

    

#     # HamBot의 heading은 'degrees from East'
#     initial_yaw = bot.get_heading()  # degrees
#     target_yaw = (initial_yaw + math.degrees(radianAngle)) % 360
#     while True:
#         current_yaw = bot.get_heading()  # degrees

#         # 차이 계산 (−180~180 범위로 정규화)
#         delta = (target_yaw - current_yaw + 540) % 360 - 180
#         error = target_yaw - delta

#         print(f"Rotate:: current={current_yaw:.2f}, target={target_yaw:.2f}, delta={delta:.2f}, errer={error:.2f}")
#         # 목표 각도에 거의 도달하면 정지
#         if abs(delta) < 20:  # ±2 허용 오차
#             bot.stop_motors()
#             print("Rotation complete.")
#             break
        
#         print(f"Dr:: Dr={D_r:.2f}")
#         if D_r < 0.6:
#             bot.stop_motors()
#             print("Right wall too close, stopping rotation.")
#             break

#         # 회전 속도 적용
#         rotation_speed = base_speed if delta > 0 else -base_speed
#         bot.set_left_motor_speed(rotation_speed)
#         bot.set_right_motor_speed(-rotation_speed)
#         time.sleep(dt)
    
#     resetPID(bot)
#     move_forward(bot)


# ========================
# PID withWall
# ========================
def withWall(bot):
    
    """Right wall following using PID."""
    global I_f, E_prev_f, I_r, E_prev_r, E_r, E_f

    print(" Starting wall following mode...")

    while True:
        # lidar = bot.get_range_image()

        # # 기본 예외 처리 (라이다 데이터 존재 확인)
        # if lidar is None or len(lidar) < 360:
        #     center_idx = len(lidar) // 2
        #     print(f"Front distance: {lidar[center_idx]:.3f} m")
        # else:
        #     print("No LiDAR data received")


        # # 센서 데이터 (degrees 기준)
        # D_f = np.nanmin(lidar[175:185])  / 600 # front
        
        # D_r = np.nanmin(lidar[265:285])  / 600 # right
        # D_l = np.nanmin(lidar[75:105])   / 600 # left
        
        # lidar = bot.get_range_image()  # HamBot LiDAR 360° 배열

        # if lidar is None or len(lidar) < 360:
        #     print("No LiDAR data received, skipping this step")
        #     time.sleep(dt)
        #     continue

        # # front 구간 평균 거리 계산 (safe_distance 적용)
        # D_f = sum([safe_distance(v) for v in lidar[175:185]]) / 60.0
        # D_r = sum([safe_distance(v) for v in lidar[265:285]]) / 600.0
        # D_l = sum([safe_distance(v) for v in lidar[75:105]]) / 600.0

        # print(f"[SUMMMM]Front={D_f:.2f}, Right={D_r:.2f}, Left={D_l:.2f}")

        # # 결측치 처리
        # if np.isinf(D_f) or np.isnan(D_f) or D_f < 0.05:
        #     D_f = 1.0
        # if np.isinf(D_r) or np.isnan(D_r) or D_r < 0.05:
        #     D_r = 1.0
        # if np.isinf(D_l) or np.isnan(D_l):
        #     D_l = 1.0

        # # 에러 계산
        # E_f = D_f - target_D_f
        # E_r = D_r - target_D_r

        # # PID 계산 (오른쪽 벽 기준)
        # P = Kp * E_r
        # I_r += E_r * dt
        # D_term = (E_r - E_prev_r) / dt
        # E_prev_r = E_r

        # control = P + Ki * I_r + Kd * D_term
        # if np.isnan(control) or np.isinf(control):
        #     control = 0.0
        # control = np.clip(control, -1, 1)
        # base_speed = 10
        # left_speed  = base_speed + control
        # right_speed = base_speed - control
        # bot.set_left_motor_speed(left_speed)
        # bot.set_right_motor_speed(right_speed)


        # print(f"[WallFollow] D_f={D_f:.4f}, E_f={E_f:.4f}, D_r={D_r:.4f}, E_r={E_r:.4f}, control={control:.4f}, D_l={D_l:.4f}")

        # if D_f < 1.5 :
        #     print(f"[BEFORETURN] D_f={D_f:.4f}, E_f={E_f:.4f}, D_r={D_r:.4f}, E_r={E_r:.4f}, control={control:.4f}, D_l={D_l:.4f}")
        #     print(f"[BEFORETURN] D_f={D_f:.4f}, E_f={E_f:.4f}, D_r={D_r:.4f}, E_r={E_r:.4f}, control={control:.4f}, D_l={D_l:.4f}")
        #     print(f"[BEFORETURN] D_f={D_f:.4f}, E_f={E_f:.4f}, D_r={D_r:.4f}, E_r={E_r:.4f}, control={control:.4f}, D_l={D_l:.4f}")
        #     print(f"[BEFORETURN] D_f={D_f:.4f}, E_f={E_f:.4f}, D_r={D_r:.4f}, E_r={E_r:.4f}, control={control:.4f}, D_l={D_l:.4f}")
        #     print(f"[BEFORETURN] D_f={D_f:.4f}, E_f={E_f:.4f}, D_r={D_r:.4f}, E_r={E_r:.4f}, control={control:.4f}, D_l={D_l:.4f}")
        #     bot.stop_motors()
        #     bot.set_left_motor_speed(0)
        #     bot.set_right_motor_speed(0)
        #     if D_r < D_l:
        #         print("LEFT:::STOPSTOPSTOPSTOPSTOPSTOPSTOSPTOSPTOPSTOPSTOSPTOPOSP")
        #         rotate(bot, -math.pi / 2)
        #         move_forward(bot)
        #         break
        #     elif D_r > D_l:
        #         print("Right:::STOPSTOPSTOPSTOPSTOPSTOPSTOSPTOSPTOPSTOPSTOSPTOPOSP")
        #         rotate(bot, math.pi / 2)
        #         move_forward(bot)
        #         break
        # elif D_r > 1.2:
        #     bot.stop_motors()
        #     bot.set_left_motor_speed(0)
        #     bot.set_right_motor_speed(0)
        #     print(f"[BEFORETURN] D_f={D_f:.4f}, E_f={E_f:.4f}, D_r={D_r:.4f}, E_r={E_r:.4f}, control={control:.4f}, D_l={D_l:.4f}")
        #     print("Right wall is far from at least 1.2")
        #     rotate(bot, math.pi / 2)
        #     move_forward(bot)
            
        # time.sleep(dt)
        
        
        """Right wall following using PID."""
        global I_f, E_prev_f, I_r, E_prev_r, E_r, E_f

        print(" Starting wall following mode...")

        while True:
            lidar = bot.get_range_image()

            # 기본 예외 처리 (라이다 데이터 존재 확인)
            if lidar is None or len(lidar) < 360:
                center_idx = len(lidar) // 2
                print(f"Front distance: {lidar[center_idx]:.3f} m")
            else:
                print("No LiDAR data received")


            # 센서 데이터 (degrees 기준)
            D_f = np.nanmin(lidar[179:181])  / 600 # front
            D_r = np.nanmax(lidar[265:285])  / 600 # right
            D_l = np.nanmin(lidar[75:105])   / 600 # left

            # 결측치 처리
            if np.isinf(D_f) or np.isnan(D_f) or D_f < 0.05:
                D_f = 1.0
            if np.isinf(D_r) or np.isnan(D_r) or D_r < 0.05:
                D_r = 0.5
            if np.isinf(D_l) or np.isnan(D_l):
                D_l = 1.0

            # 에러 계산
            E_f = D_f - target_D_f
            E_r = D_r - target_D_r

            # PID 계산 (오른쪽 벽 기준)
            P = Kp * E_r
            I_r += E_r * dt
            D_term = (E_r - E_prev_r) / dt
            E_prev_r = E_r

            control = P + Ki * I_r + Kd * D_term
            if np.isnan(control) or np.isinf(control):
                control = 0.0
            control = np.clip(control, -1, 1)
            base_speed = 10
            left_speed  = base_speed + control
            right_speed = base_speed - control
            bot.set_left_motor_speed(left_speed)
            bot.set_right_motor_speed(right_speed)


            print(f"[WallFollow] D_f={D_f:.4f}, E_f={E_f:.4f}, D_r={D_r:.4f}, E_r={E_r:.4f}, control={control:.4f}, D_l={D_l:.4f}")

            if D_f < 0.4 :
                print(f"[BEFORETURN] D_f={D_f:.4f}, E_f={E_f:.4f}, D_r={D_r:.4f}, E_r={E_r:.4f}, control={control:.4f}, D_l={D_l:.4f}")
                print(f"[BEFORETURN] D_f={D_f:.4f}, E_f={E_f:.4f}, D_r={D_r:.4f}, E_r={E_r:.4f}, control={control:.4f}, D_l={D_l:.4f}")
                print(f"[BEFORETURN] D_f={D_f:.4f}, E_f={E_f:.4f}, D_r={D_r:.4f}, E_r={E_r:.4f}, control={control:.4f}, D_l={D_l:.4f}")
                print(f"[BEFORETURN] D_f={D_f:.4f}, E_f={E_f:.4f}, D_r={D_r:.4f}, E_r={E_r:.4f}, control={control:.4f}, D_l={D_l:.4f}")
                print(f"[BEFORETURN] D_f={D_f:.4f}, E_f={E_f:.4f}, D_r={D_r:.4f}, E_r={E_r:.4f}, control={control:.4f}, D_l={D_l:.4f}")
                bot.stop_motors()
                bot.set_left_motor_speed(0)
                bot.set_right_motor_speed(0)
                if D_r < D_l:
                    print("LEFT:::STOPSTOPSTOPSTOPSTOPSTOPSTOSPTOSPTOPSTOPSTOSPTOPOSP")
                    #rotate(bot, -math.pi / 2)
                    move_arc(bot, R = 0.1, theta = np.pi / 2, direction="CCW", max_v=50)
                    move_forward(bot)
                    break
                elif D_r > D_l:
                    print("Right:::STOPSTOPSTOPSTOPSTOPSTOPSTOSPTOSPTOPSTOPSTOSPTOPOSP")
                    #rotate(bot, math.pi / 2)
                    move_arc(bot, R = 0.1, theta = np.pi / 2, direction="CW", max_v=50)
                    move_forward(bot)
                    break
            elif D_r > 0.6:
                bot.stop_motors()
                bot.set_left_motor_speed(0)
                bot.set_right_motor_speed(0)
                print(f"[BEFORETURN] D_f={D_f:.4f}, E_f={E_f:.4f}, D_r={D_r:.4f}, E_r={E_r:.4f}, control={control:.4f}, D_l={D_l:.4f}")
                print("Right wall is far from at least 1.2")
                move_arc(bot, R = 0.1, theta = np.pi / 2, direction="CW", max_v=50)
                move_forward(bot)
                
            time.sleep(dt)
        #Turing 
        # if E_f < 0.5 and (D_r > D_l):
        #     print("RIGHT:::STOPSTOPSTOPSTOPSTOPSTOPSTOSPTOSPTOPSTOPSTOSPTOPOSP")
        #     bot.stop_motors()
        #     bot.set_left_motor_speed(0)
        #     bot.set_right_motor_speed(0)
        #     rotate(bot, -math.pi)
        #     move_forward(bot)
        # elif D_f < 0.3 and (D_r < D_l):
        #     print("LEFT:::STOPSTOPSTOPSTOPSTOPSTOPSTOSPTOSPTOPSTOPSTOSPTOPOSP")
        #     bot.stop_motors()
        #     bot.set_left_motor_speed(0)
        #     bot.set_right_motor_speed(0)
        #     bot.stop_motors()
        #     rotate(bot, math.pi)
        #     move_forward(bot)
        # time.sleep(dt)


# ========================
# Main loop
# ========================
if __name__ == "__main__":
    print("HamBot Wall Following PID Controller Started.")
    withWall(bot)
