#Test 1: Connecting Ras and Mac
#Commit 1
import sys
sys.path.append('/home/hambot/HamBot/src')
from robot_systems.robot import HamBot


import numpy as np
import time


# HamBot
bot = HamBot(lidar_enabled=False, camera_enabled=False)

# Setting 
wheel_radius = 0.090
axel_length = 0.184

# ----Move Func----
def move_str(D, max_v = 50):
    bot.set_left_motor_speed(max_v)
    bot.set_right_motor_speed(max_v)
    
    init_encoder = bot.get_left_motor_encoder_reading()
    
    while True:
        # Current D
        distance_traveled = wheel_radius * (bot.get_left_motor_encoder_reading() - init_encoder)
        
        # Dubugging
        print("Motor Encoder Readings: ", bot.get_encoder_readings())
        print("Left Motor Encoder Reading: ", bot.get_left_motor_encoder_reading())
        print("Distance traveled: ", distance_traveled, '\n')
        
        # Stop when curr >= D
        if distance_traveled >= D:
            bot.stop_motors()
            break
        
        time.sleep(0.01)  # prevent loop so fast
        

def move_rot(bot, curr_yaw, new_yaw, max_v=50):
    """
    bot: HamBot object
    curr_yaw: curr angular V (rad)
    new_yaw: new AV (rad)
    max_v: V (-100 ~ 100)
    """
    # calcuate orientation
    delta_theta = (new_yaw - curr_yaw + np.pi) % (2 * np.pi) - np.pi
    
    if delta_theta > 0:
        l_speed = -max_v
        r_speed = max_v
    else:
        l_speed = max_v
        r_speed = -max_v
    
    # set motor speed
    bot.set_left_motor_speed(l_speed)
    bot.set_right_motor_speed(r_speed)
    
    start_time = time.time()
    
    # when new orientation, stop
    while True:
        # read HamBot IMU
        try:
            curr_yaw = bot.imu.get_roll_pitch_yaw()[2]
        except AttributeError:
            # if dont have IMU, simulation 
            print("IMU not detected.")
            time.sleep(0.01)
            break  
        
        turn_wheel = (new_yaw - curr_yaw + np.pi) % (2 * np.pi) - np.pi
        if abs(turn_wheel) < 0.01:
            bot.stop_motors()
            break
        
        time.sleep(0.01)
    bot.stop_motor()


def rot_in_place(bot, R, D, theta, max_v=50):
    """
    bot: HamBot object
    R: (m)
    theta: (rad)
    direction: "CCW" or "CW"
    max_v: V (-100 ~ 100)
    """
    start_O = bot.get_heading()
    # calcualate right/left
    if theta > 0 :
        bot.set_left_motor_speed(-max_v)
        bot.set_right_motor_speed(max_v)
        
    else:
        bot.set_left_motor_speed(max_v)
        bot.set_right_motor_speed(-max_v)
    
    
    rotated = 0.0
    prev_O = start_O
    while abs(rotated) < abs(theta):
        curr_O = bot.get_heading()
         ,.         
        delta = (curr_O - prev_O + np.pi) %(2 * np.pi) - np.pi
        rotated += delta
        prev_O = curr_O
    
    bot.stop_motors()
        
      
    # # total time
    # t_total = max(abs(d_left), abs(d_right)) / max_v
    
    # # 이동 루프
    # t0 = time.time()
    # while True:
    #     if time.time() - t0 >= t_total:
    #         bot.stop_motors()
    #         break
    #     time.sleep(0.01)
    
    # initial_encoder = bot.get_left_motor_encoder_reading()  
    # while True:
    #     # Current D
    #     distance_traveled = wheel_radius * (bot.get_left_motor_encoder_reading() - initial_encoder)
        
    #     # Dubugging
    #     print("Motor Encoder Readings: ", bot.get_encoder_readings())
    #     print("Left Motor Encoder Reading: ", bot.get_left_motor_encoder_reading())
    #     print("Distance traveled: ", distance_traveled, '\n')
        
    #     # Stop when curr >= D
    #     if distance_traveled >= D:
    #         bot.stop_motors()
    #         break
        
    #     time.sleep(0.01)  # prevent loop so fast


# ---- Path ----
P = [(2.0, -2.0, np.pi),
     (-1.5, -2.0, np.pi),
     (-2.0, -1.5, np.pi/2),
     (-2.0, -0.5, np.pi/2),
     (-1.0, -0.5, 3*np.pi/2),
     (-0.5, -1.0, 7*np.pi/4)]

# # ---- auto loop ----
# for i in range(len(P)-1):
#     start = P[i][:2]
#     end = P[i+1][:2]
    
#     # move vector
#     vec = np.array(end) - np.array(start)
#     distance = np.linalg.norm(vec)
    
#     # new angle
#     angle_target = np.arctan2(vec[1], vec[0])
#     angle_current = P[i][2]
#     delta_angle = (angle_target - angle_current + np.pi) % (2*np.pi) - np.pi
    
#     # rot->str
#     move_rot(bot, angle_current, angle_target)
#     move_str(bot, distance, max_v=50)


# P0 -> P1
# D_01 = np.linalg.norm(np.array(P[1][:2]) - np.array(P[0][:2]))
# move_str(bot, D=D_01, max_v=50)  # 50 ~100 단위로 속도 조정

# P1 -> P2
rot_in_place(bot, R=1.0, theta=np.pi/2, max_v=50)

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


print("Path Success!")
bot.stop_motors()

