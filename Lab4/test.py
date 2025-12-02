import time
import numpy as np
import random
from collections import defaultdict
from robot_systems.robot import HamBot

def turn_left(bot, deg):
    start = bot.get_heading()
    goal = (start - deg) % 360  # 왼쪽 회전 목표
    while True:
        h = bot.get_heading()
        # -180 ~ +180 범위 error
        error = (goal - h + 180) % 360 - 180
        if abs(error) < 2:  # 2도 오차 허용
            break
        # 왼쪽 회전이면 error < 0
        if error < 0:
            bot.set_left_motor_speed(4)
            bot.set_right_motor_speed(-4)
        else:
            bot.set_left_motor_speed(-4)
            bot.set_right_motor_speed(4)
        time.sleep(0.01)
    bot.set_left_motor_speed(0)
    bot.set_right_motor_speed(0)


def turn_right(bot, deg):
    start = bot.get_heading()
    goal = (start + deg) % 360  # 오른쪽 회전 목표
    while True:
        h = bot.get_heading()
        error = (goal - h + 180) % 360 - 180
        if abs(error) < 2:
            break
        # 오른쪽 회전이면 error > 0
        if error > 0:
            bot.set_left_motor_speed(4)
            bot.set_right_motor_speed(-4)
        else:
            bot.set_left_motor_speed(-4)
            bot.set_right_motor_speed(4)
        time.sleep(0.01)
    bot.set_left_motor_speed(0)
    bot.set_right_motor_speed(0)


def main():
    bot = HamBot()
    turn_left(bot,90)
    turn_right(bot,90)

    print("Finished.")

if __name__ == "__main__":
    main()