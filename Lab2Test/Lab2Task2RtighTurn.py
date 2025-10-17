"""
HamBot Wall Following PID Controller (Real Robot Version)
Author: Seyoung Kan
Date: 2025-10-10
"""

##Test
import time
import math
import numpy as np
from robot_systems.robot import HamBot  # HamBot 실물용 라이브러리
#change

axel_length = 0.184
wheel_radius = 0.090

# ========================
# PID gains
# ========================
Kp = 0.45
Ki = 0.00019
Kd = 1

# Target distances (meters)
target_D_f = 0.3
target_D_r = 0.3
target_D_l = 1.0
dt = 0.032  # 제어 주기 (초)

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
def move_forward(bot, speed=10, duration=2.0):
    """Move straight forward for 'duration' seconds"""
    print("Moving forward...")
    bot.set_left_motor_speed(speed)
    bot.set_right_motor_speed(speed)
    time.sleep(duration)
    bot.stop_motors()


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
        time.sleep(0.01)       

# ========================
# Main loop
# ========================
if __name__ == "__main__":
    print("HamBot Wall Following PID Controller Started.")
    move_arc(bot, R=0.3, theta = -math.pi, direction="CW", max_v=10)
    move_forward(bot)
