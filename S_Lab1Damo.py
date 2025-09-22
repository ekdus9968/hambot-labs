import numpy as np
import time
from robot_systems.robot import HamBot

# HamBot
bot = HamBot(lidar_enabled=False, camera_enabled=False)

#

MAX_W = 50
wheel_radius = 0.090
axel_length = 0.184

# ----Move Funv----
def move_str(bot, D, max_v = 50):
    bot.set_left_motor_speed(max_v)
    bot.set_right_motor_speed(max_v)
    
    initial_encoder = bot.get_left_motor_encoder_reading()
    
    while True:
        # 현재 이동 거리 계산
        distance_traveled = wheel_radius * (bot.get_left_motor_encoder_reading() - initial_encoder)
        
        # 디버깅 출력
        print("Motor Encoder Readings: ", bot.get_encoder_readings())
        print("Left Motor Encoder Reading: ", bot.get_left_motor_encoder_reading())
        print("Distance traveled: ", distance_traveled, '\n')
        
        # 목표 거리 도달 시 멈춤
        if distance_traveled >= D:
            bot.stop_motors()
            break
        
        time.sleep(0.01)  # 루프 너무 빠르게 도는 것 방지
        

def move_rot(bot, curr_yaw, new_yaw, max_v=50):
    """
    bot: HamBot 객체
    curr_yaw: 현재 회전 각도 (rad)
    new_yaw: 목표 회전 각도 (rad)
    max_v: 회전 속도 (-100 ~ 100)
    """
    # 회전 방향 계산
    delta_theta = (new_yaw - curr_yaw + np.pi) % (2 * np.pi) - np.pi
    
    if delta_theta > 0:
        l_speed = -max_v
        r_speed = max_v
    else:
        l_speed = max_v
        r_speed = -max_v
    
    # 모터 속도 설정
    bot.set_left_motor_speed(l_speed)
    bot.set_right_motor_speed(r_speed)
    
    # 단순 루프: 목표 각도에 도달하면 정지
    while True:
        # HamBot IMU가 있는 경우 현재 yaw 읽기
        try:
            curr_yaw = bot.imu.get_roll_pitch_yaw()[2]
        except AttributeError:
            # IMU 없으면 임시로 루프 시간을 두고 회전만 시뮬레이션
            time.sleep(0.01)
            break  # 실제 모터 테스트 시 IMU 값 필요
        
        turn_wheel = (new_yaw - curr_yaw + np.pi) % (2 * np.pi) - np.pi
        if abs(turn_wheel) < 0.01:
            bot.stop_motors()
            break
        
        time.sleep(0.01)


def move_arc(bot, R, theta, direction="CCW", max_v=50):
    """
    bot: HamBot 객체
    R: 회전 반지름 (m)
    theta: 회전 각도 (rad)
    direction: "CCW" 또는 "CW"
    max_v: 최대 속도 (-100 ~ 100)
    """
    # 왼쪽/오른쪽 이동 거리 계산
    if direction.upper() == "CCW":
        d_left = (R - axel_length/2) * theta
        d_right = (R + axel_length/2) * theta
    elif direction.upper() == "CW":
        d_left = (R + axel_length/2) * theta
        d_right = (R - axel_length/2) * theta
    else:
        raise ValueError("direction must be 'CCW' or 'CW'")
    
    # 이동 시간 계산 (단순 속도 기반)
    t_total = max(abs(d_left), abs(d_right)) / max_v
    
    # 모터 속도 설정
    omega_left = (d_left / t_total) / bot.wheel_radius
    omega_right = (d_right / t_total) / bot.wheel_radius
    
    bot.set_left_motor_speed(omega_left)
    bot.set_right_motor_speed(omega_right)
    
    # 이동 루프
    t0 = time.time()
    while True:
        if time.time() - t0 >= t_total:
            bot.stop_motors()
            break
        time.sleep(0.01)


# ---- Path ----
P = [(2.0, -2.0, np.pi),
     (-1.5, -2.0, np.pi),
     (-2.0, -1.5, np.pi/2),
     (-2.0, -0.5, np.pi/2),
     (-1.0, -0.5, 3*np.pi/2),
     (-0.5, -1.0, 7*np.pi/4)]

# ---- 자동 주행 루프 ----
# for i in range(len(P)-1):
#     start = P[i][:2]
#     end = P[i+1][:2]
    
#     # 이동 벡터
#     vec = np.array(end) - np.array(start)
#     distance = np.linalg.norm(vec)
    
#     # 목표 각도
#     angle_target = np.arctan2(vec[1], vec[0])
#     angle_current = P[i][2]
#     delta_angle = (angle_target - angle_current + np.pi) % (2*np.pi) - np.pi
    
#     # 회전 후 직선 이동
#     move_rot(bot, angle_current, angle_target)
#     move_str(bot, distance, max_v=50)


D_01 = np.linalg.norm(np.array(P[1][:2]) - np.array(P[0][:2]))
move_str(bot, D=D_01, max_v=50)  # 50 ~100 단위로 속도 조정

# # P1 -> P2
# move_arc(bot, R=0.5, theta=np.pi/2, direction="CW", max_v=50)

# # P2 -> P3
# D_23 = np.linalg.norm(np.array(P[3][:2]) - np.array(P[2][:2]))
# move_str(bot, D=D_23, max_v=50)

# # P3 -> P4
# move_arc(bot, R=0.5, theta=np.pi, direction="CW", max_v=50)

# # P4 -> P5
# move_rot(bot, curr_yaw=np.pi/2, new_yaw=7*np.pi/4, max_v=50)

# # P5 -> P6
# D_45 = np.linalg.norm(np.array(P[5][:2]) - np.array(P[4][:2]))
# move_str(bot, D=D_45, max_v=50)


# print("Path 완료!")
# bot.stop_motors()

