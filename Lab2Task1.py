"""HamBot PID Wall Following Example"""

import time
import numpy as np
from robot_systems.robot import HamBot

# PID gains
Kp = 1.0
Ki = 0.01
Kd = 0.5

# 목표 거리 (미터)
target_distance = 1.0
dt = 0.032  # 32ms

# PID 상태 변수
integral = 0.0
prev_error = 0.0
error = 0.0

# HamBot 초기화
bot = HamBot(lidar_enabled=True, camera_enabled=False)

try:
    while True:
        # LiDAR 읽기
        lidar = bot.get_range_image()  # 360개 값, 각도 0~359
        # 로봇 앞쪽 거리 (approx 180° 전방)
        forward_distance = min(lidar[175:195])
        if np.isinf(forward_distance) or np.isnan(forward_distance):
            forward_distance = 6.0  # 기본값

        # PID 계산
        error = forward_distance - target_distance
        proportional = Kp * error
        integral += error * dt
        derivative = (error - prev_error) / dt
        prev_error = error
        u = proportional + Ki * integral + Kd * derivative

        # 속도 제한
        u = max(min(u, 50), -50)  # HamBot 모터 범위 -100~100

        # 모터 속도 적용
        bot.set_left_motor_speed(u)
        bot.set_right_motor_speed(u)

        # 디버깅 출력
        print(f"Forward: {forward_distance:.3f}, Error: {error:.3f}, PID u: {u:.2f}")

        # dt만큼 대기
        time.sleep(dt)

except KeyboardInterrupt:
    # 안전 정지
    bot.stop_motors()
    print("Stopped.")
