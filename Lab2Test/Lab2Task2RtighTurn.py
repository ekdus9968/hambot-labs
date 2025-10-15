"""
HamBot Wall Following PID Controller (Real Robot Version)
Author: Seyoung Kan
Date: 2025-10-10
"""

import time
import numpy as np
from robot_systems.robot import HamBot  # HamBot 실물용 라이브러리

# ========================
# PID gains
# ========================
Kp = 1.0
Ki = 0.01
Kd = 0.5

# Target distances (meters)
target_D_f = 0.5
target_D_r = 0.15
target_D_l = 0.5
dt = 0.05  # 제어 주기 (초)

# PID state variables
I_f = 0.0
E_prev_f = 0.0
I_r = 0.0
E_prev_r = 0.0

# ========================
# Robot setup
# ========================
bot = HamBot(lidar_enabled=True, camera_enabled=False)
print("amBot initialized and ready for wall following.")

# ========================
# Utility functions
# ========================


def move_forward(bot, speed=50, duration=2.0):
    """Move straight forward for 'duration' seconds"""
    print("Moving forward...")
    bot.set_left_motor_speed(-speed)
    bot.set_right_motor_speed(speed)
    time.sleep(duration)
    bot.stop_motors()

def turn_right(bot, target_angle=90):
    """Turn right by using IMU heading."""
    print("Turning right...")
    start_angle = bot.get_heading()
    target = (start_angle - target_angle) % 360

    bot.set_left_motor_speed(40)
    bot.set_right_motor_speed(-40)

    while True:
        current = bot.get_heading()
        diff = (target - current + 180) % 360 - 180
        if abs(diff) < 2:  # 2도 이내 도달 시 정지
            break
        time.sleep(0.05)

    bot.stop_motors()
    print("Right turn complete.")
    move_forward(bot)



# ========================
# Wall following control
# ========================

def withWall(bot):
    """Right wall following using PID."""
    global I_f, E_prev_f, I_r, E_prev_r

    print(" Starting wall following mode...")

    while True:
        lidar = bot.get_range_image()

        # 기본 예외 처리 (라이다 데이터 존재 확인)
        if lidar is None or len(lidar) < 360:
            print(" LiDAR data invalid, skipping cycle.")
            time.sleep(dt)
            continue

        # 센서 데이터 (degrees 기준)
        D_f = np.nanmin(lidar[175:195])  # front
        D_r = np.nanmin(lidar[265:285])  # right
        D_l = np.nanmin(lidar[75:105])   # left

        # 결측치 처리
        if np.isinf(D_f) or np.isnan(D_f) or D_f < 0.05:
            D_f = 6.0
        if np.isinf(D_r) or np.isnan(D_r) or D_r < 0.05:
            D_r = 0.01
        if np.isinf(D_l) or np.isnan(D_l):
            D_l = 0.01

        # 에러 계산
        E_f = D_f - target_D_f
        E_r = D_r - target_D_r

        # PID 계산 (오른쪽 벽 기준)
        P = Kp * E_r
        I_r += E_r * dt
        D_term = (E_r - E_prev_r) / dt
        E_prev_r = E_r

        control = P + Ki * I_r + Kd * D_term
        control = np.clip(control, -100, 100)

        left_speed = 60 + control
        right_speed = 60 - control

        bot.set_left_motor_speed(-left_speed)
        bot.set_right_motor_speed(right_speed)

        print(f"[WallFollow] D_f={D_f:.2f}, D_r={D_r:.2f}, control={control:.2f}")

        # 장애물 또는 벽 조건 처리
        if D_f < 0.6 and D_r > 0.5:
            bot.stop_motors()
            turn_right(bot)
        

        time.sleep(dt)


# ========================
# Main loop
# ========================
if __name__ == "__main__":
    print("🤖 HamBot Wall Following PID Controller Started.")
    move_forward(bot)
    withWall(bot)
