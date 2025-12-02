import time
import numpy as np
import random
from collections import defaultdict
from robot_systems.robot import HamBot


def turn_left(bot, deg):
    """
    Turn left by `deg` degrees using heading only.
    Stops when error < 3°.
    """
    start = bot.get_heading()
    if start is None:
        print("[DEBUG] No heading, skipping turn")
        return

    goal = (start - deg) % 360
    fixed_speed = 4.0

    while True:
        current = bot.get_heading()
        if current is None:
            print("[DEBUG] No heading during turn")
            break
        # 왼쪽 회전 기준 최소 각도 error
        error = abs((current - goal + 360) % 360 - 180)
        if error > 180:
            error = 360 - error

        print(f"[DEBUG] Turning left - Current: {current:.2f}, Goal: {goal:.2f}, Error: {error:.2f}")

        if error < 3:
            bot.set_left_motor_speed(0)
            bot.set_right_motor_speed(0)
            print("[DEBUG] Left turn complete")
            break
        else:
            bot.set_left_motor_speed(-fixed_speed)
            bot.set_right_motor_speed(fixed_speed)
        time.sleep(0.01)


def turn_right(bot, deg):
    """
    Turn right by `deg` degrees using heading only.
    Stops when error < 3°.
    """
    start = bot.get_heading()
    if start is None:
        print("[DEBUG] No heading, skipping turn")
        return

    goal = (start + deg) % 360
    fixed_speed = 4.0

    while True:
        current = bot.get_heading()
        if current is None:
            print("[DEBUG] No heading during turn")
            break
        # 오른쪽 회전 기준 최소 각도 error
        error = abs((goal - current + 360) % 360 - 180)
        if error > 180:
            error = 360 - error

        print(f"[DEBUG] Turning right - Current: {current:.2f}, Goal: {goal:.2f}, Error: {error:.2f}")

        if error < 3:
            bot.set_left_motor_speed(0)
            bot.set_right_motor_speed(0)
            print("[DEBUG] Right turn complete")
            break
        else:
            bot.set_left_motor_speed(fixed_speed)
            bot.set_right_motor_speed(-fixed_speed)
        time.sleep(0.01)

def main():
    bot = HamBot()
    turn_left(bot,90)
    turn_right(bot,90)

    print("Finished.")

if __name__ == "__main__":
    main()