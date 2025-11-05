#Lab 3 PID + target detection 
import time
import math
import numpy as np
from enum import Enum
from robot_systems.robot import HamBot 

# ========================
# HamBot Setup
# ========================
bot = HamBot(lidar_enabled=True, camera_enabled=True)
dt = 0.032

# PID state variables
prev_error = 0
integral = 0
prev_angle_error = 0
angleIntegral = 0

# ========================
# Utility functions
# ========================
def resetPID():
    global prev_error, integral, prev_angle_error, angleIntegral
    print("restPID")
    prev_error = 0
    integral = 0
    prev_angle_error = 0
    angleIntegral = 0

def safe_distance(value, max_range=9.5):
    print("safe_distance")
    if math.isinf(value) or math.isnan(value):
        return max_range
    return min(value, max_range)

def saturation(speed, max_speed=20):
    print("saturation")
    return max(min(speed, max_speed), -max_speed)

# ========================
# PID Controllers
# ========================
def forwardPID(target_distance=0.4):
    global prev_error, integral
    print("ForwardPID")
    lidar = bot.get_range_image()
    actual_distance = np.mean([safe_distance(v) for v in lidar[175:185]]) / 600
    error = actual_distance - target_distance

    Kp = 3.0
    Ki = 0
    Kd = 0.15

    P = Kp * error
    I = Ki * integral
    D = Kd * (error - prev_error) / dt
    prev_error = error
    integral += error * dt

    v = P + I + D
    return saturation(v)

def sidePID(wall="left"):
    print("SidePID")
    global prev_angle_error, angleIntegral
    lidar = bot.get_range_image()
    side_distance = 0.4
    Kp = 0.45
    Ki = 0.00019
    Kd = 1

    actual_left = safe_distance(np.min(lidar[90:115]))/ 600
    actual_right = safe_distance(np.min(lidar[265:290]))/600

    if wall == "left" and actual_left > 2.5:
        return 0.0
    if wall == "right" and actual_right > 2.5:
        return 0.0

    error = (actual_right - side_distance) if wall == "right" else (actual_left - side_distance)
    P = Kp * error
    angleIntegral += error * dt
    angleIntegral = np.clip(angleIntegral, -1.0, 1.0)
    I = Ki * angleIntegral
    D = Kd * (error - prev_angle_error) / dt
    prev_angle_error = error

    angular_velocity = P + I + D
    return saturation(angular_velocity)

# ========================
# Rotation for corners
# ========================
def rotate(radianAngle):
    print("Rotate")
    resetPID()
    base_speed = 1.0
    left_direction = 1 if radianAngle > 0 else -1
    right_direction = -left_direction

    initial_yaw = bot.get_heading()   # radians

    while True:
        current_yaw = bot.get_heading()
        delta = (current_yaw - initial_yaw + math.pi) % (2*math.pi) - math.pi
        if abs(delta) >= abs(radianAngle):
            bot.set_left_motor_speed(0)
            bot.set_right_motor_speed(0)
            break
        bot.set_left_motor_speed(left_direction * base_speed)
        bot.set_right_motor_speed(right_direction * base_speed)
        time.sleep(dt)
    resetPID()

# ========================
# Wall following (left & right)
# ========================
def wall_follow(wall="left"):
    print("Wall_Following")
    lidar = bot.get_range_image()
    left_distance = safe_distance(np.min(lidar[90:115]))/600
    print("LEFT D: ", left_distance)
    right_distance = safe_distance(np.min(lidar[265:290]))/600
    print("Right D: ", right_distance)
    target = 0.4

    linear_velocity = forwardPID(target_distance=target)
    angular_velocity = sidePID(wall)

    rightv = leftv = linear_velocity

    search_sign = +1 if wall == "right" else -1
    side_distance = right_distance if wall == "right" else left_distance

    if side_distance >= 2.5:
        # No wall detected then arc
        base = 0.6 * linear_velocity
        bias = 0.4
        rightv = base - search_sign * bias
        leftv = base + search_sign * bias
    else:
        # Wall-following PID adjustment
        if wall == "right":
            if right_distance < target:
                rightv = linear_velocity + abs(angular_velocity)
                leftv = linear_velocity - abs(angular_velocity)
            elif right_distance < 2.0:
                rightv = linear_velocity - abs(angular_velocity)
                leftv = linear_velocity + abs(angular_velocity)
        else:
            if left_distance < target:
                rightv = linear_velocity - abs(angular_velocity)
                leftv = linear_velocity + abs(angular_velocity)
            elif left_distance > target and left_distance < 2.0:
                rightv = linear_velocity + abs(angular_velocity)
                leftv = linear_velocity - abs(angular_velocity)

    return saturation(rightv), saturation(leftv)

#camera setup

YELLOW = (255, 220, 0)   # attempting to make yellow
color_tolerance  = 45              # per-channel tolerance (tune)

# Distance proxy calibration: distance_k = D0 (m) * bbox_height_px at that distance
distance_k = 120.0 #used to estimate distance

last_detection = {"seen": False, "ypost": None, "frame_size": None}

#get frame size from the camera
def _get_frame_size():
    image = bot.camera.get_image()  # most recent frame (threaded)
    if image is None:
        return None
    #infer shape and height of tqrget
    H, W = image.shape[0], image.shape[1]
    return (W, H)

#if cqmera see yellow 
def _detect_yellow_post():
    posts = bot.camera.find_landmarks(YELLOW, tolerance=color_tolerance)
    if not posts:
        return None

    def post_fields(p): #used for dictionary return values
            x = p.get("x", 0); y = p.get("y", 0); w = p.get("w", 0); h = p.get("h", 0)
            return int(x), int(y), int(w), int(h)

    #using postts to detect yellow post region 
    yposts = [post_fields(p) for p in posts]
    x, y, w, h = max(yposts, key=lambda t: t[2]*t[3]) #pick the largest yellow region
    return (x, y, w, h) #goes towards largest yyellow area 

#if target in sight returh seen
def target_in_sight():
    ypost = _detect_yellow_post() #landmark detection
    fs = _get_frame_size() 
    last_detection["seen"] = ypost is not None
    last_detection["ypost"] = ypost
    last_detection["frame_size"] = fs
    return last_detection["seen"]

def target_bearing():
    if not last_detection["seen"] or not last_detection["ypost"] or not last_detection["frame_size"]:
        return 0.0 #if nothing return 0
    x, y, w, h = last_detection["ypost"] #using yellow post
    W, H = last_detection["frame_size"]
    cx = x + w/2.0
    x_err = (cx - (W/2.0)) / (W/2.0)  # normalize -1 left, +1 right
    half_fov = 1.0  
    return float(np.clip(x_err * half_fov, -half_fov, half_fov))

def target_distance():
    if not last_detection["seen"] or not last_detection["ypost"]:
        return 9.9
    _, _, _, h = last_detection["ypost"] #height of post
    if h <= 1:
        return 9.9
    d = distance_k / float(h)
    return float(np.clip(d, 0.05, 9.9)) #bound restriction

#nodes for state machine (search, drive, wall following)
class Mode(Enum):
    SEARCH = 0
    DRIVE = 1
    WALL_FOLLOW = 2

mode = Mode.SEARCH
wall = "left"      #follow left 
GOAL_RADIUS = 0.20   #desired stop distance 
TOO_CLOSE_WALL = 0.45     

def front_distance_m(lidar):
    if lidar is None or len(lidar) < 200:
        return 9.9 #maybe adjust this if hambot is weird
    win = lidar[175:185] if len(lidar) >= 360 else lidar[len(lidar)//2 - 5 : len(lidar)//2 + 5]
    return float(np.mean([safe_distance(v) for v in win])) / 600.0

def too_close_to_wall(lidar):
    return front_distance_m(lidar) < TOO_CLOSE_WALL

def close_enough_to_target():
    return target_distance() <= GOAL_RADIUS

def do_search():
    #search for target by turning in a cicle
    print("Spin modeeee")
    bot.set_left_motor_speed(+0.6)
    bot.set_right_motor_speed(-0.6)

def do_drive():
    print("going to target")
    Kp_aim = 2.0 #steering gain
    beep = target_bearing()  # 0 if centered
    ang_cmd = np.clip(Kp_aim * beep, -1.0, 1.0) #capped angular 
    v = forwardPID(target_distance=0.40)
    left = v - ang_cmd
    right = v + ang_cmd
    bot.set_left_motor_speed(saturation(left))
    bot.set_right_motor_speed(saturation(right))

#follows the wall while continously checking for the yellow marker
#switches wall follow depending on where target is (allegedly)
def do_wall_follow():
    print("following wall")
    r, l = wall_follow(wall)
    bot.set_left_motor_speed(l)
    bot.set_right_motor_speed(r)

def target_on_opposite_side_of_wall():
    b = target_bearing()
    if wall == "left":
        return b > 0  # target on right
    else:
        return b < 0  # target on left

#state machine (while loop) lol
while True:
    lidar = bot.get_range_image()

    #search mode
    if mode == Mode.SEARCH:
        do_search()
        # target found go to drive mode
        if target_in_sight():
            resetPID()
            mode = Mode.DRIVE

    #drive mode
    elif mode == Mode.DRIVE:
        #call drive function while target in sight
        if target_in_sight():
            do_drive()
        else:
            # if target lost keep driving using pid
            print("target lost now using forward pid ")
            v = forwardPID(target_distance=0.40)
            bot.set_left_motor_speed(v)
            bot.set_right_motor_speed(v)

        # close stop
        if close_enough_to_target():
            print("reached target")
            bot.set_left_motor_speed(0)
            bot.set_right_motor_speed(0)
            break
        
        #to close to wall use turn function then change to wall follow mode
        if too_close_to_wall(lidar):
            print("too close must turn now")
            turn = (math.pi/2) if wall == "left" else -(math.pi/2)
            rotate(turn)
            resetPID()
            mode = Mode.WALL_FOLLOW

    #follow state
    elif mode == Mode.WALL_FOLLOW:
        #continuous wall follow if target not in sight 
        do_wall_follow()

        # if target is in sight drive towards it 
        if target_in_sight() and target_on_opposite_side_of_wall():
            print("Target is in free space")
            resetPID()
            mode = Mode.DRIVE

        # final stop if close
        if close_enough_to_target():
            print("reached target")
            bot.set_left_motor_speed(0)
            bot.set_right_motor_speed(0)
            break

    time.sleep(dt)
