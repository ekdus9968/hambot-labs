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


def rotate(bot, radianAngle):
    """
    Rotate the robot by a given angle (in radians).
    Positive angle -> left (CCW), Negative angle -> right (CW)
    """
    print("Rotate start...")
    resetPID(bot)
    base_speed = 1.0  # 회전 속도 (너무 빠르면 overshoot)

    # radianAngle (+ left / - right)
    left_direction = 1 if radianAngle > 0 else -1
    right_direction = -left_direction

    # HamBot의 heading은 'degrees from East'
    initial_yaw = bot.get_heading()  # degrees
    target_yaw = (initial_yaw + math.degrees(radianAngle)) % 360

    while True:
        current_yaw = bot.get_heading()  # degrees

        # 차이 계산 (−180~180 범위로 정규화)
        delta = (target_yaw - current_yaw + 540) % 360 - 180

        print(f"Rotate:: current={current_yaw:.2f}, target={target_yaw:.2f}, delta={delta:.2f}")

        # 목표 각도에 거의 도달하면 정지
        if abs(delta) < 2.0:  # ±2 허용 오차
            bot.stop_motors()
            print("Rotation complete.")
            break

        # 회전 속도 적용
        bot.set_left_motor_speed(left_direction * base_speed)
        bot.set_right_motor_speed(right_direction * base_speed)

        time.sleep(dt)
    
    resetPID(bot)

# ========================
# Main loop
# ========================
if __name__ == "__main__":
    print("HamBot Wall Following PID Controller Started.")
    rotate(bot, math.pi)
    move_forward(bot)
