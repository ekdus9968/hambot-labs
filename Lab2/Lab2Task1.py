"""HamBot PID Wall Following Example"""

import time
import numpy as np
from robot_systems.robot import HamBot

# PID gains
Kp = 10.0
Ki = 0.5
Kd = 1.0

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
        scan = bot.get_range_image()
        if scan is not None and len(scan) > 0:
            center_idx = len(scan) // 2
            print(f"Front distance: {scan[center_idx]:.3f} m")
        else:
            print("No LiDAR data received")
        
        forward_distance = np.min(scan[175:185]) /600
        
        if np.isnan(forward_distance) or np.isinf(forward_distance):
            forward_distance = 6.0
        
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
        u = max(min(u, 40), -40)  # HamBot 모터 범위 -100~100

        # 모터 속도 적용
        bot.set_left_motor_speed(u)
        bot.set_right_motor_speed(u)

        # 디버깅 출력
        print(f"Forward: {forward_distance:.3f}, Error: {error:.3f}, PID u: {u:.2f}")

        if abs(error) < 0.01 and abs(u) < 0.5:
            print("Reached target distance. Stopping robot.")
            bot.stop_motors()
            break
        # dt만큼 대기
        time.sleep(dt)

except KeyboardInterrupt:
    # 안전 정지
    bot.stop_motors()
    print("Stopped.")
