
import time
import numpy as np
import random
from collections import defaultdict
from robot_systems.robot import HamBot

def drive_forward(bot, D, speed=5):
    wheel_radius = 90
    bot.reset_encoders()
    bot.set_left_motor_speed(speed)
    bot.set_right_motor_speed(speed)
    
    initial_l = bot.get_left_encoder_reading()
    initial_r = bot.get_right_encoder_reading()
    
    while True:
        # 현재 이동 거리 계산
        l_delta = bot.get_left_encoder_reading() - initial_l
        r_delta = bot.get_right_encoder_reading() - initial_r
        distance_traveled = wheel_radius * (l_delta + r_delta) / 2

        # 디버그 출력
        print(f"[DEBUG] Encoders -> Left: {bot.get_left_encoder_reading():.2f}, Right: {bot.get_right_encoder_reading():.2f}")
        print(f"[DEBUG] Distance traveled: {distance_traveled:.2f} m / Target: {D} m\n")
        
        # 목표 거리 도달 시 멈춤
        if distance_traveled >= D:
            bot.set_left_motor_speed(0)
            bot.set_right_motor_speed(0)
            bot.stop_motors()
            print(f"[DEBUG] Target distance reached: {distance_traveled:.2f} m")
            break

        time.sleep(0.01)
def turn_left(bot, deg, speed=8):
    """
    Turn left by `deg` degrees using only wheel encoders.
    """
    wheel_radius=90
    axel_length=184
    # 목표 바퀴 이동 거리 계산
    delta_theta_rad = np.deg2rad(deg)
    l_dist = -delta_theta_rad * (axel_length / 2)
    r_dist = delta_theta_rad * (axel_length / 2)

    # 초기 엔코더 값 저장
    init_l = bot.get_left_encoder_reading()
    init_r = bot.get_right_encoder_reading()

    # 속도 비율 설정
    max_dist = max(abs(l_dist), abs(r_dist))
    v_l = speed * l_dist / max_dist
    v_r = speed * r_dist / max_dist

    # 모터 속도 설정
    bot.set_left_motor_speed(v_l)
    bot.set_right_motor_speed(v_r)

    while True:
        l_delta = bot.get_left_encoder_reading() - init_l
        r_delta = bot.get_right_encoder_reading() - init_r

        l_d = wheel_radius * l_delta
        r_d = wheel_radius * r_delta

        if abs(l_d) >= abs(l_dist) or abs(r_d) >= abs(r_dist):
            bot.set_left_motor_speed(0)
            bot.set_right_motor_speed(0)
            bot.stop_motors()
            break
        time.sleep(0.01)


def turn_right(bot, deg, speed=8):
    """
    Turn right by `deg` degrees using only wheel encoders.
    """
    wheel_radius=90
    axel_length=184
    # 목표 바퀴 이동 거리 계산
    delta_theta_rad = np.deg2rad(deg)
    l_dist = delta_theta_rad * (axel_length / 2)
    r_dist = -delta_theta_rad * (axel_length / 2)

    # 초기 엔코더 값 저장
    init_l = bot.get_left_encoder_reading()
    init_r = bot.get_right_encoder_reading()

    # 속도 비율 설정
    max_dist = max(abs(l_dist), abs(r_dist))
    v_l = speed * l_dist / max_dist
    v_r = speed * r_dist / max_dist

    # 모터 속도 설정
    bot.set_left_motor_speed(v_l)
    bot.set_right_motor_speed(v_r)

    while True:
        l_delta = bot.get_left_encoder_reading() - init_l
        r_delta = bot.get_right_encoder_reading() - init_r

        l_d = wheel_radius * l_delta
        r_d = wheel_radius * r_delta

        if abs(l_d) >= abs(l_dist) or abs(r_d) >= abs(r_dist):
            bot.set_left_motor_speed(0)
            bot.set_right_motor_speed(0)
            bot.stop_motors()
            break
        time.sleep(0.01)

def main():
    bot = HamBot()
    print("turn right")
    turn_right(bot, 150)
    #turn_left(bot, 90)

    print("Finished.")

if __name__ == "__main__":
    main()