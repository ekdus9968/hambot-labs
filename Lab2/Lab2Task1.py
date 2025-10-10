"""HamBot PID Wall Following Example"""

import time
import numpy as np
from robot_systems.robot import HamBot

# PID gains
Kp = 1.0
Ki = 0.01
Kd = 0.5

# T_Distance
target_distance = 1
dt = 0.032  # 32ms

# PID 
integral = 0.0
prev_error = 0.0
error = 0.0

# HamBot setting
bot = HamBot(lidar_enabled=True, camera_enabled=False)

try:
    while True:
        # LiDAR
        lidar = bot.get_range_image()  # 360개 값, 각도 0~359
        # 로봇 앞쪽 거리 (approx 180° 전방)
        forward_distance = min(lidar[175:195])/10000.00
        if np.isinf(forward_distance) or np.isnan(forward_distance):
            forward_distance = 2.4  # 기본값

        # PID 계산
        error = forward_distance - target_distance
        proportional = Kp * error
        integral += error * dt
        derivative = (error - prev_error) / dt
        prev_error = error
        u = proportional + Ki * integral + Kd * derivative

        if np.isnan(u) or np.isinf(u):
            u = 0.0
        #
        u = max(min(u, 25), -25)  # HamBot 모터 범위 -100~100

        # 모터 속도 적용
        bot.set_left_motor_speed(u)
        bot.set_right_motor_speed(u)

        # 디버깅 출력
        print(f"Forward: {forward_distance:.3f}, Error: {error:.3f}, PID u: {u:.2f}")

        if abs(error) < 0.005 and abs(u) < 0.005:
            print("Reached target distance. Stopping robot.")
            bot.stop_motors()
            break
        # dt만큼 대기
        time.sleep(dt)

except KeyboardInterrupt:
    # 안전 정지
    bot.stop_motors()
    print("Stopped.")
