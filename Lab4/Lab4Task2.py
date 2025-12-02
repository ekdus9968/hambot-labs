import time
import numpy as np
import random
from collections import defaultdict
from robot_systems.robot import HamBot

# ============================================
# CONFIG
# ============================================
N_PARTICLES = 160
ORIENTATIONS = ["N", "E", "S", "W"]  # 북, 동, 남, 서
GRID_SIZE = 16  # 4x4 grid
SUCCESS_RATIO = 0.80
STEP_DISTANCE = 600  # mm

# ============================================
# MAZE WALL INFO (N,E,S,W)
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
particles = [Particle(cell=i % GRID_SIZE, orientation=ORIENTATIONS[i % 4]) for i in range(N_PARTICLES)]

# ============================================
# MOTION UPDATE
# ============================================
def move_particle(p, motion):
    walls = maze_map[p.cell]
    o = p.orientation
    new_cell = p.cell

    if motion == "forward":
        if o == "N" and walls[0] == 0 and p.cell >= 4:
            new_cell = p.cell - 4
        elif o == "E" and walls[1] == 0 and p.cell % 4 != 3:
            new_cell = p.cell + 1
        elif o == "S" and walls[2] == 0 and p.cell < 12:
            new_cell = p.cell + 4
        elif o == "W" and walls[3] == 0 and p.cell % 4 != 0:
            new_cell = p.cell - 1
        p.cell = max(0, min(new_cell, GRID_SIZE-1))
    elif motion == "left_turn":
        p.orientation = ORIENTATIONS[(ORIENTATIONS.index(o)-1) % 4]
    elif motion == "right_turn":
        p.orientation = ORIENTATIONS[(ORIENTATIONS.index(o)+1) % 4]

# ============================================
# SENSOR MODEL
# ============================================
sensor_probs = {
    0: {0:0.6, 1:0.4},
    1: {0:0.2, 1:0.8}
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
        return [Particle(random.randint(0, GRID_SIZE-1),
                         random.choice(ORIENTATIONS)) for _ in range(N_PARTICLES)]

    normalized = [w/total for w in weights]
    new_ps = random.choices(particles, weights=normalized, k=N_PARTICLES)
    return [Particle(p.cell, random.choice(ORIENTATIONS)) for p in new_ps]

# ============================================
# ESTIMATION
# ============================================
def estimate_position(particles):
    counter = defaultdict(int)
    for p in particles:
        counter[p.cell] += 1
    mode_cell = max(counter, key=counter.get)
    count = counter[mode_cell]
    return mode_cell, count

# ============================================
# PARTICLE DEBUG
# ============================================
def debug_particles(particles):
    counts = [0]*GRID_SIZE
    for p in particles:
        counts[p.cell] += 1
    print("Particles per cell:")
    for i in range(0, GRID_SIZE, 4):
        print(counts[i:i+4])
    weights = [round(p.weight,3) for p in particles[:10]]
    print("Sample weights:", weights, "...")

# ============================================
# SENSOR FROM ROBOT
# ============================================
def get_observation(bot):
    dN, dE, dS, dW = bot.get_front_distance(), bot.get_right_distance(), bot.get_back_distance(), bot.get_left_distance()
    TH = 400
    return (1 if dN < TH else 0,
            1 if dE < TH else 0,
            1 if dS < TH else 0,
            1 if dW < TH else 0)

# ============================================
# ROBOT CONTROL
# ============================================
def drive_forward(bot, D, speed=5):
    wheel_radius = 0.090
    bot.reset_encoders()
    bot.set_left_motor_speed(speed)
    bot.set_right_motor_speed(speed)
    initial_l = bot.get_left_encoder_reading()
    initial_r = bot.get_right_encoder_reading()
    
    while True:
        l_delta = bot.get_left_encoder_reading() - initial_l
        r_delta = bot.get_right_encoder_reading() - initial_r
        distance_traveled = wheel_radius * (l_delta + r_delta) / 2
        if distance_traveled >= D:
            bot.set_left_motor_speed(0)
            bot.set_right_motor_speed(0)
            bot.stop_motors()
            break
        time.sleep(0.01)

def turn_left(bot, deg):
    start = bot.get_heading()
    goal = (start - deg) % 360
    while True:
        h = bot.get_heading()
        error = (h - goal + 360) % 360
        if error < 3:
            break
        bot.set_left_motor_speed(-4)
        bot.set_right_motor_speed(4)
    bot.set_left_motor_speed(0)
    bot.set_right_motor_speed(0)

def turn_right(bot, deg):
    start = bot.get_heading()
    goal = (start + deg) % 360
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
# MAIN LOOP
# ============================================
def main():
    bot = HamBot()
    
    
    motions = []

    for step, command in enumerate(motions):
        print(f"\n=== STEP {step+1}: {command} ===")
        

        # --- PARTICLE PREDICTION ---
        for p in particles:
            move_particle(p, command)

        # --- SENSOR UPDATE ---
        obs = get_observation(bot)
        print("Observation:", obs)
        for p in particles:
            compute_weight(p, obs)

        # --- RESAMPLING ---
        particles[:] = resample_particles(particles)

        # --- DEBUG ---
        print("Debug particle")
        debug_particles(particles)
        print("debug estimation")
        # --- ESTIMATION ---
        cell, count = estimate_position(particles)
        print(f"Estimated cell = {cell}, count = {count}/{N_PARTICLES}")
        
        # --- ROBOT ACTION ---
        if command == "forward":
            drive_forward(bot, STEP_DISTANCE)
        elif command == "left_turn":
            turn_left(bot, 90)
        elif command == "right_turn":
            turn_right(bot, 90)

        time.sleep(0.3)

        if count / N_PARTICLES >= SUCCESS_RATIO:
            print("Localization success! >80% particles converged.")
            break

    print("Finished.")

if __name__ == "__main__":
    main()


    #motions = ["forward", "right_turn", "forward", "forward", "forward", "right_turn", "forward", "forward", "left_turn", "left_turn", "forward", "forward", "forward", "right_turn", "forward", "forward", "forward", "right_turn", "forward", "forward", "forward", "right_turn", "forward", "right_turn", "forward", "forward"]
    motions = ["left_turn", "forward", "left_turn", "forward", "forward", "forward", "right_turn", "forward", "forward", "left_turn"]
