import time
import numpy as np
import random
from collections import defaultdict
from robot_systems.robot import HamBot

# ============================================
# CONFIG
# ============================================
N_PARTICLES = 160
ORIENTATIONS = ["N", "E", "S", "W"]  # 북 동 남 서
GRID_SIZE = 16   # 4x4 grid
SUCCESS_RATIO = 0.80

# ============================================
# MAZE WALL INFO  (N,E,S,W)
# ============================================
maze_map = {
    0: (1,0,0,1), 1:(1,0,0,0), 2:(1,0,0,0), 3:(1,1,0,0),
    4: (0,0,0,1), 5:(1,0,1,1), 6:(1,0,1,0), 7:(0,1,1,0),
    8: (0,0,0,1), 9:(1,0,1,0),10:(1,1,1,0),11:(1,1,0,1),
    12:(0,0,1,1),13:(1,0,1,0),14:(1,0,1,0),15:(0,1,1,0)
}

# ============================================
# PARTICLE CLASS
# ============================================
class Particle:
    def __init__(self, cell, orientation):
        self.cell = cell
        self.orientation = orientation
        self.weight = 1.0 / N_PARTICLES

# ============================================
# PARTICLE FILTER INITIALIZATION
# ============================================
particles = []
for i in range(N_PARTICLES):
    cell = random.randint(0, GRID_SIZE-1)
    orientation = ORIENTATIONS[i % 4]   # 균등하게 배분
    particles.append(Particle(cell, orientation))


# ============================================
# MOTION UPDATE (perfect motion model)
# ============================================
def move_particle(p, motion):
    walls = maze_map[p.cell]
    o = p.orientation

    if motion == "forward":
        if o == "N" and walls[0] == 0 and p.cell >= 4:
            p.cell -= 4
        elif o == "E" and walls[1] == 0 and p.cell % 4 != 3:
            p.cell += 1
        elif o == "S" and walls[2] == 0 and p.cell < 12:
            p.cell += 4
        elif o == "W" and walls[3] == 0 and p.cell % 4 != 0:
            p.cell -= 1

    elif motion == "left_turn":
        p.orientation = ORIENTATIONS[(ORIENTATIONS.index(o)-1) % 4]

    elif motion == "right_turn":
        p.orientation = ORIENTATIONS[(ORIENTATIONS.index(o)+1) % 4]

# ============================================
# SENSOR MODEL
# ============================================
sensor_probs = {
    0: {0:0.6, 1:0.4},   # true wall = 0
    1: {0:0.2, 1:0.8}    # true wall = 1
}

def compute_weight(p, observation):
    true_sig = maze_map[p.cell]
    w = 1.0
    for obs, s_true in zip(observation, true_sig):
        w *= sensor_probs[s_true][obs]
    p.weight = w

# ============================================
# RESAMPLING
# ============================================
def resample_particles(particles):
    weights = [p.weight for p in particles]
    total = sum(weights)

    if total == 0:
        # reset
        return [
            Particle(random.randint(0,GRID_SIZE-1),
                     random.choice(ORIENTATIONS))
            for _ in range(N_PARTICLES)
        ]

    normalized = [w/total for w in weights]
    new_ps = random.choices(particles, weights=normalized, k=N_PARTICLES)

    # deepcopy + random orientation
    return [
        Particle(p.cell, random.choice(ORIENTATIONS))
        for p in new_ps
    ]

# ============================================
# POSITION ESTIMATION
# ============================================
def estimate_position(particles):
    counter = defaultdict(int)
    for p in particles:
        counter[p.cell] += 1

    mode_cell = max(counter, key=counter.get)
    count = counter[mode_cell]
    return mode_cell, count

# ============================================
# SENSOR FROM REAL ROBOT
# ============================================
def get_observation(bot):
    # 거리값 측정 (mm)
    dN, dE, dS, dW = bot.get_front_distance(), bot.get_right_distance(), bot.get_back_distance(), bot.get_left_distance()

    # threshold 정의 (벽인지 아닌지)
    # 벽 가까이 있으면 1, 아니면 0으로 단순화
    TH = 400  # 30cm 이내면 벽
    obs = (
        1 if dN < TH else 0,
        1 if dE < TH else 0,
        1 if dS < TH else 0,
        1 if dW < TH else 0
    )
    return obs

def drive_forward(bot, D, speed=5):
    wheel_radius = 0.090
    bot.reset_encoders()
    bot.set_left_motor_speed(speed)
    bot.set_right_motor_speed(speed)
    
    initial_l = bot.get_left_encoder_reading()
    initial_r = bot.get_right_encoder_reading()
    
    while True:
        # Curr D
        l_delta = bot.get_left_encoder_reading() - initial_l
        r_delta = bot.get_right_encoder_reading() - initial_r
        distance_traveled = wheel_radius * (l_delta + r_delta) / 2
        
        # print
        print("STR:: Motor Encoder Readings: ", bot.get_encoder_readings())
        print("STR:: Distance traveled: ", distance_traveled, '\n')
        
        # stop 
        if distance_traveled >= D:
            bot.set_left_motor_speed(0)
            bot.set_right_motor_speed(0)
            bot.stop_motors()
            break
        
        time.sleep(0.01)  # prevent loop susper fast
        
def turn_left(bot, target_deg):
    start = bot.get_heading()
    goal = (start - target_deg) % 360

    while True:
        h = bot.get_heading()
        error = (h - goal + 360) % 360
        if error < 3:
            break
        bot.set_left_motor_speed(-4)
        bot.set_right_motor_speed(4)

    bot.set_left_motor_speed(0)
    bot.set_right_motor_speed(0)

def turn_right(bot, target_deg):
    start = bot.get_heading()
    goal = (start + target_deg) % 360

    while True:
        h = bot.get_heading()
        error = (goal - h + 360) % 360
        if error < 3:
            break
        bot.set_left_motor_speed(4)
        bot.set_right_motor_speed(-4)

    bot.set_left_motor_speed(0)
    bot.set_right_motor_speed(0)


# ============================================
# PARTICLE FILTER MAIN LOOP
# ============================================
# motions = ["forward", "right_turn", "forward", "forward", "forward", "right_turn", "forward", "forward", "left_turn", "left_turn", "forward", "forward", "forward", "right_turn", "forward", "forward", "forward", "right_turn", "forward", "forward", "forward", "right_turn", "forward", "right_turn", "forward", "forward"]
def main():
    bot = HamBot()

    #motions = ["forward", "right_turn", "forward", "forward", "forward", "right_turn", "forward", "forward", "left_turn", "left_turn", "forward", "forward", "forward", "right_turn", "forward", "forward", "forward", "right_turn", "forward", "forward", "forward", "right_turn", "forward", "right_turn", "forward", "forward"]
    motions = ["left_turn", "forward", "left_turn", "forward", "forward", "forward", "right_turn", "forward", "forward", "left_turn"]

    for step, command in enumerate(motions):
        print(f"\n=== STEP {step+1}: {command} ===")

        # --- 실제 로봇 움직임 ---
        if command == "forward":
            drive_forward(bot, 600, speed=5)
        elif command == "left_turn":
            turn_left(bot, 90)
        elif command == "right_turn":
            turn_right(bot, 90)

        time.sleep(1)

        # --- PF prediction ---
        for p in particles:
            move_particle(p, command)

        # --- PF sensor update ---
        obs = get_observation(bot)
        print("Observation:", obs)
        for p in particles:
            compute_weight(p, obs)

        # --- PF resample ---
        particles[:] = resample_particles(particles)

        # --- PF estimate ---
        cell, count = estimate_position(particles)
        print(f"Estimated cell = {cell}, count = {count}/{N_PARTICLES}")

        if count >= N_PARTICLES * SUCCESS_RATIO:
            print("Localization success!")
            break


if __name__ == "__main__":
    main()
