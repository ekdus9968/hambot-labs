"""new_controller controller."""
from robot_systems.robot import HamBot
import time
import math

robot = HamBot(lidar_enabled=False, camera_enabled=False)

WHEEL_RADIUS = 0.045  
WHEEL_DISTANCE = 0.092

def go_straight(x1, y1, x2, y2):
    # finding distance
    if x1 != x2:
        calc_distance = abs(x2 - x1)
    else:
        calc_distance = abs(y2 - y1)

    # record starting encoder values
    start_left, start_right = robot.get_encoder_readings()

    while True:
        left, right = robot.get_encoder_readings()
        # average distance traveled by both wheels
        distance = ((left - start_left) + (right - start_right)) / 2.0 * WHEEL_RADIUS

        if distance >= calc_distance - 0.02:
            robot.stop_motors()
            return
        else:
            robot.set_left_motor_speed(15)
            robot.set_right_motor_speed(15)
        time.sleep(0.01)

def turn_to(target_heading):
    """
    Turn robot in place to absolute heading (0=East, 90=North, 180=West, 270=South).
    Uses compass + wheels. Angles in degrees.
    """
    # read current heading (degrees from East, CCW)
    current = robot.get_heading()

    # compute shortest turn error (-180, 180]
    error = target_heading - current
    if error > 180:
        error -= 360
    elif error <= -180:
        error += 360

    # decide direction
    direction = 1 if error > 0 else -1   # + = CCW (left), - = CW (right)

    # turn until aligned within a small tolerance
    while True:
        current = robot.get_heading()
        err = target_heading - current
        if err > 180: err -= 360
        elif err <= -180: err += 360

        # stop if within tolerance
        if abs(err) < 2:   # 2 degrees tolerance
            robot.stop_motors()
            return

        # turn in place: left wheel opposite of right wheel
        speed = 2.0 * direction   # rad/s, tune this
        robot.set_left_motor_speed(-speed)
        robot.set_right_motor_speed(speed)

        time.sleep(0.01)

    #calculate time

#point 0 to point 1 
go_straight(2.0,-2.0,-1.5,-2.0)


#point 1 to point 2
t0 = time.time()

while True:
    t = time.time() - t0  # elapsed just for the arc

    if t >= 1.378:
        robot.stop_motors()
        robot.set_left_motor_speed(0)
        robot.set_right_motor_speed(0)
        break

    robot.set_left_motor_speed(15)
    robot.set_right_motor_speed(10.338)

    time.sleep(0.01)

turn_to(90)

#point 2 to 3
go_straight(-2.0,-1.5,-2.0,-0.5)

#point 3 4
t0 = time.time() 
while True:
    t = time.time()  - t0  # elapsed just for the arc

    if t >= 2.755:
        robot.stop_motors()
        robot.set_left_motor_speed(0)
        robot.set_right_motor_speed(0)
        break

    robot.set_left_motor_speed(15)
    robot.set_right_motor_speed(10.338)

    time.sleep(0.01)

#point 4 to 5
turn_to(315) #275+45
go_straight(-1.0,-0.5,-0.5,-1.0)

#point 5 to 6 
turn_to(0)
go_straight(-0.5,-1.0,2.0,-1.0)

#point 6 to 7
turn_to(90)
go_straight(2.0,-1.0,2.0,0.0)

#point 7 to 8 
turn_to(180)
go_straight(2.0,0.0,0.0,0.0)

#point 8 to 9
turn_to(90)
go_straight(0.0,0.0,0.0,1.0)

#point 9 to 10
turn_to(180)
go_straight(0.0, 1.0,-2.0,1.0)

#point 10 to 11
# --- P10 -> P11 (two-arc, no spins) ---
# Arc 1 (clockwise)
t0 = time.time()
while True:
    t =  time.time() - t0
    if t >= 4.649:
        robot.stop_motors()
        robot.set_left_motor_speed(0.0)
        robot.set_right_motor_speed(0.0)
        break
    robot.set_left_motor_speed(15.000)    # rad/s 15
    robot.set_right_motor_speed(14)   # rad/s 11.456

    time.sleep(0.01)


#point 11 to 12
turn_to(0)
go_straight(-1.0,2.0,1.5,2.0)
go_straight(1.5,2.0,2.5,2.0)

#point 12 to 13

t0 = time.time() - t0
while True:
    t = time.time() - t0
    if t >= 2.5:
        robot.set_left_motor_velocity(0.0)
        robot.set_right_motor_velocity(0.0)
        break
    robot.set_left_motor_velocity(15.000)    # rad/s
    robot.set_right_motor_velocity(9.0)   # rad/s

    time.sleep(0.01)

