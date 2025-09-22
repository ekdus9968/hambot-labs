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
#DONE!
def move_str(bot, D, max_v = 50):
    
    bot.reset_encoders()
    bot.set_left_motor_speed(max_v)
    bot.set_right_motor_speed(max_v)
    
    initial_l = bot.get_left_encoder_reading()
    initial_r = bot.get_right_encoder_reading()
    
    while True:
        # 현재 이동 거리 계산
        l_delta = bot.get_left_encoder_reading() - initial_l
        r_delta = bot.get_right_encoder_reading() - initial_r
        distance_traveled = wheel_radius * (l_delta + r_delta) / 2
        
        # 디버깅 출력
        print("Motor Encoder Readings: ", bot.get_encoder_readings())
        print("Distance traveled: ", distance_traveled, '\n')
        
        # 목표 거리 도달 시 멈춤
        if distance_traveled >= D:
            bot.set_left_motor_speed(0)
            bot.set_right_motor_speed(0)
            bot.stop_motors()
            break
        
        time.sleep(0.01)  # 루프 너무 빠르게 도는 것 방지
        
#DONE!
def move_rot(bot, delta_theta, max_v=50):
    """
    bot: HamBot 객체
    curr_yaw: 현재 회전 각도 (rad)
    new_yaw: 목표 회전 각도 (rad)
    max_v: 회전 속도 (-100 ~ 100)
    """
    # 회전 방향 계산
    l_dist = -delta_theta * (axel_length / 2)
    r_dist = delta_theta * (axel_length / 2)
    
    init_l = bot.get_left_encoder_reading()
    init_r = bot.get_right_encoder_reading()
    
    max_dist = max(abs(l_dist), abs(r_dist))
    v_l = max_v * l_dist / max_dist
    v_r = max_v * r_dist / max_dist
    
    # 모터 속도 설정
    bot.set_left_motor_speed(v_l)
    bot.set_right_motor_speed(v_r)
    
    # 단순 루프: 목표 각도에 도달하면 정지
    while True:
        
        l_delta = bot.get_left_encoder_reading() - init_l
        r_delta = bot.get_right_encoder_reading() - init_r
        
        l_d = wheel_radius * l_delta
        r_d = wheel_radius * r_delta
        
        if abs(l_d) >= abs(l_dist) or abs(r_d) >= abs(r_dist):
            bot.stop_motors()
            break        
        time.sleep(0.01)

"Turning R/W"
def move_arc(bot, R, theta, direction="CCW", max_v=50):
    # 왼쪽/오른쪽 이동 거리 계산
    if direction.upper() == "CCW":
        d_left = (R - axel_length/2) * theta
        d_right = (R + axel_length/2) * theta
    else:
        d_left = (R + axel_length/2) * theta
        d_right = (R - axel_length/2) * theta
    
    bot.reset_encoders()  
    init_l = bot.get_left_encoder_reading()
    init_r = bot.get_right_encoder_reading()
    
    v_ratio_l = l_remain / max(abs(d_left), abs(d_right))
    v_ratio_r = r_remain / max(abs(d_left), abs(d_right))
    
    bot.set_left_motor_speed(max_v * v_ratio_l)
    bot.set_right_motor_speed(max_v * v_ratio_r)
    
    while True:
        l_delta = bot.get_left_encoder_reading() - init_l
        r_delta = bot.get_right_encoder_reading() - init_r
        
        d_l = wheel_radius * l_delta
        d_r = wheel_radius * r_delta
        
        #each loop. cal rest of D ratio in live 
        #when i only use ratio var, it stops faster
        l_remain = d_left - d_l
        r_remain = d_right - d_r
        
        v_ratio_l = l_remain / max(abs(l_remain), abs(r_remain))
        v_ratio_r = r_remain / max(abs(l_remain), abs(r_remain))
        
        bot.set_left_motor_speed(max_v * v_ratio_l)
        bot.set_right_motor_speed(max_v * v_ratio_r)
        
        print("L: ", d_l)
        print("Target L: ", d_left )
        print("Target R: ", d_right )
        
        if abs(d_l) >= abs(d_left) and abs(d_r) >= abs(d_right):
            bot.set_left_motor_speed(0)
            bot.set_right_motor_speed(0)
            bot.stop_motors()
            break
        time.sleep(0.01)       


# ---- Path ----
P = [(1.0, -2.0, np.pi),
     (1.5, -2.0, np.pi),
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


#D_01 = np.linalg.norm(np.array(P[1][:2]) - np.array(P[0][:2]))
#move_str(bot, D=0.3, max_v=50)  # 50 ~100 단위로 속도 조정

# # P1 -> P2
move_arc(bot, R=2.0, theta=np.pi, direction="CW", max_v=50)

# # P2 -> P3
# D_23 = np.linalg.norm(np.array(P[3][:2]) - np.array(P[2][:2]))
# move_str(bot, D=D_23, max_v=50)

# # P3 -> P4
# move_arc(bot, R=0.5, theta=np.pi, direction="CW", max_v=50)

# # P4 -> P5
#move_rot(bot, np.pi, max_v=50)

# # P5 -> P6
# D_45 = np.linalg.norm(np.array(P[5][:2]) - np.array(P[4][:2]))
# move_str(bot, D=D_45, max_v=50)


# print("Path 완료!")
# bot.stop_motors()

