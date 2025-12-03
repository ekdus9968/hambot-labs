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
STEP_DISTANCE = 1200  # m

# ============================================
# MAZE WALL INFO (N,E,S,W)
# ============================================
maze_map = {
    12: (1,0,0,1), 1:(1,0,0,0), 2:(1,0,0,0), 3:(1,1,0,0),
    4: (0,1,0,1), 5:(1,0,1,1), 6:(1,0,1,0), 7:(0,1,1,0),
    8: (0,0,0,1), 9:(1,0,1,0),10:(1,1,1,0),11:(1,1,0,1),
    0:(0,0,1,1),13:(1,0,1,0),14:(1,0,1,0),15:(0,0,1,1)
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
particles = [Particle(cell=i % GRID_SIZE, orientation="S") for i in range(N_PARTICLES)]

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
    return [Particle(p.cell, p.orientation) for p in new_ps]

# ============================================
# ESTIMATION
# ============================================
# def estimate_position(particles):
#     counter = defaultdict(int)
#     for p in particles:
#         counter[p.cell] += 1
#     mode_cell = max(counter, key=counter.get)
#     count = counter[mode_cell]
#     return mode_cell, count

# ============================================
# WEIGHT-BASED ESTIMATION
# ============================================
def estimate_position_weighted(particles):
    """
    Return the cell with the highest total particle weight
    """
    weight_sum = defaultdict(float)
    for p in particles:
        weight_sum[p.cell] += p.weight

    max_cell = max(weight_sum, key=weight_sum.get)
    max_weight = weight_sum[max_cell]
    return max_cell, max_weight


# ============================================
# DEBUG PARTICLES (Weight 기반)
# ============================================
def debug_particles_weighted(bot, particles):
    """
    Print detailed debug info for particles using weight fractions.
    """
    weight_sum_per_cell = [0.0]*GRID_SIZE
    for p in particles:
        weight_sum_per_cell[p.cell] += p.weight

    total_weight = sum(weight_sum_per_cell)
    fractions = [w / total_weight for w in weight_sum_per_cell]
    cumulative = np.cumsum(fractions)
    max_frac = max(fractions)
    max_cell = fractions.index(max_frac)

    print("\n--- Particle Weight Distribution ---")
    for row in range(4):
        row_weights = weight_sum_per_cell[row*4:(row+1)*4]
        row_fracs = [round(fractions[i+row*4],3) for i in range(4)]
        print(f"Row {row}: Weights {row_weights}, Fractions {row_fracs}")
    print(f"Cumulative fractions: {[round(c,3) for c in cumulative]}")
    print(f"Max weight cell: {max_cell}, Localization confidence: {round(max_frac*100,1)}%")

    # sample weights for first 10 particles
    sample_weights = [round(p.weight,3) for p in particles[:10]]
    print(f"Sample particle weights: {sample_weights} ...\n")

    if max_frac > 0.80:
        bot.stop_motors()
        bot.set_left_motor_speed(0)
        bot.set_right_motor_speed(0)
        print("Localization confidence > 80%, stopping robot")
        return 1

    return 0

# ============================================
# PARTICLE DEBUG
# ============================================
# def debug_particles(particles):
#     counts = [0]*GRID_SIZE
#     for p in particles:
#         counts[p.cell] += 1
#     print("Particles per cell:")
#     for i in range(0, GRID_SIZE, 4):
#         print(counts[i:i+4])
#     weights = [round(p.weight,3) for p in particles[:10]]
#     print("Sample weights:", weights, "...")
def debug_particles(bot, particles):
    """
    Print detailed debug info for particles.
    - Count per cell
    - Fraction per cell
    - Cumulative fraction
    - Max cell fraction (localization confidence)
    """
    counts = [0]*GRID_SIZE
    for p in particles:
        counts[p.cell] += 1

    total_particles = len(particles)
    fractions = [c/total_particles for c in counts]
    cumulative = np.cumsum(fractions)
    max_frac = max(fractions) 
    max_cell = counts.index(max(counts))

    print("\n--- Particle Distribution ---")
    for row in range(4):
        row_counts = counts[row*4:(row+1)*4]
        row_fracs = [round(fractions[i+row*4],3) for i in range(4)]
        print(f"Row {row}: Counts {row_counts}, Fractions {row_fracs}")
    print(f"Cumulative fractions: {[round(c,3) for c in cumulative]}")
    print(f"Max cell: {max_cell}, Localization confidence: {round(max_frac*100,1)}%")

    # sample weights for first 10 particles
    sample_weights = [round(p.weight,3) for p in particles[:10]]
    print(f"Sample particle weights: {sample_weights} ...\n")
    if max_frac > 0.80:
        bot.stop_motors()
        bot.set_left_motor_speed(0)
        bot.set_right_motor_speed(0)
        print("REUTNE over 80% RATIO")
        return 1
        
    return 0

# ============================================
# SENSOR FROM ROBOT
# ============================================
def read_lidar(bot):
    lidar = bot.get_range_image()
    if lidar is None or len(lidar) < 360:
        print("[DEBUG] No LiDAR data received")
        return

    # Front and side distances
    front_dist = np.nanmin(lidar[175:185])
    right_dist = np.nanmin(lidar[265:275])
    left_dist = np.nanmin(lidar[85:95])
    back_dist = np.nanmin(lidar[355:360] + lidar[0:5])
    
    # Handle invalid readings
    for i, val in enumerate([front_dist, right_dist, back_dist, left_dist]):
        if np.isinf(val) or np.isnan(val) or val < 0.05:
            if i == 0: front_dist = 666.666
            elif i == 1: right_dist = 222.222
            elif i == 2: back_dist = 222.222
            elif i == 3: left_dist = 222.222

    
    print(f"[DEBUG] LiDAR - Front: {front_dist:.1f}, Left: {left_dist:.1f}, Right: {right_dist:.1f}, Back:  {back_dist:.1f}")
    return front_dist, left_dist, back_dist, right_dist

def get_observation(bot):
    dN, dE, dS, dW = read_lidar(bot)
    if dN is None: 
        return 0,0,0,0
    TH = 300
    return (1 if dN < TH else 0,
            1 if dE < TH else 0,
            1 if dS < TH else 0,
            1 if dW < TH else 0)

# ============================================
# ROBOT CONTROL
# ============================================
def drive_forward(bot, D, speed=8):
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
        #print(f"[DEBUG] Encoders -> Left: {bot.get_left_encoder_reading():.2f}, Right: {bot.get_right_encoder_reading():.2f}")
        #print(f"[DEBUG] Distance traveled: {distance_traveled:.2f} m / Target: {D} m\n")
        
        # 목표 거리 도달 시 멈춤
        if distance_traveled >= D:
            bot.set_left_motor_speed(0)
            bot.set_right_motor_speed(0)
            bot.stop_motors()
            #print(f"[DEBUG] Target distance reached: {distance_traveled:.2f} m")
            break

        time.sleep(0.01)

# ============================================
# TURN USING IMU
# ============================================
def turn_left(bot, deg):
    """
    Turn left by `deg` degrees using heading only.
    Stops when error < 3°.
    """
    start = bot.get_heading()
    if start is None:
        print("[DEBUG] No heading, skipping turn")
        

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

        #print(f"[DEBUG] Turning left - Current: {current:.2f}, Goal: {goal:.2f}, Error: {error:.2f}")

        if error < 1:
            bot.set_left_motor_speed(0)
            bot.set_right_motor_speed(0)
            print("[DEBUG] Left turn complete")
            break
        else:
            bot.set_left_motor_speed(-fixed_speed)
            bot.set_right_motor_speed(fixed_speed)
        time.sleep(0.01)
        
    for p in particles:
        idx = ORIENTATIONS.index(p.orientation)
        p.orientation = ORIENTATIONS[(idx - deg // 90) % 4]

    # --- 로봇 heading 동기화 (Particle 초기화) ---
    heading = bot.get_heading()
    if heading is not None:
        current_ori = heading_to_orientation(heading)
        for p in particles:
            p.orientation = current_ori



def turn_right(bot, deg):
    """
    Turn right by `deg` degrees using heading only.
    Stops when error < 3°.
    """
    start = bot.get_heading()
    if start is None:
        print("[DEBUG] No heading, skipping turn")
    

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

        # print(f"[DEBUG] Turning right - Current: {current:.2f}, Goal: {goal:.2f}, Error: {error:.2f}")

        if error < 1:
            bot.set_left_motor_speed(0)
            bot.set_right_motor_speed(0)
            print("[DEBUG] Right turn complete")
            break
        else:
            bot.set_left_motor_speed(fixed_speed)
            bot.set_right_motor_speed(-fixed_speed)
        time.sleep(0.01)
        
        
    for p in particles:
        idx = ORIENTATIONS.index(p.orientation)
        p.orientation = ORIENTATIONS[(idx + deg // 90) % 4]

    # --- 로봇 heading 동기화 (Particle 초기화) ---
    heading = bot.get_heading()
    if heading is not None:
        current_ori = heading_to_orientation(heading)
        for p in particles:
            p.orientation = current_ori


def heading_to_orientation(heading):
    """
    0°~360° heading을 N/E/S/W orientation으로 변환
    """
    if 45 <= heading < 135:
        return "W"
    elif 135 <= heading < 225:
        return "S"
    elif 225 <= heading < 315:
        return "E"
    else:
        return "N"

# ============================================
# TURN USING ENCODERS
# ============================================
# def turn_left(bot, deg= 90, speed=8):
#     """
#     Turn left by `deg` degrees using only wheel encoders.
#     """
#     wheel_radius=90
#     axel_length=184
#     # 목표 바퀴 이동 거리 계산
#     delta_theta_rad = np.deg2rad(deg)
#     l_dist = -delta_theta_rad * (axel_length / 2)
#     r_dist = delta_theta_rad * (axel_length / 2)

#     # 초기 엔코더 값 저장
#     init_l = bot.get_left_encoder_reading()
#     init_r = bot.get_right_encoder_reading()

#     # 속도 비율 설정
#     max_dist = max(abs(l_dist), abs(r_dist))
#     v_l = speed * l_dist / max_dist
#     v_r = speed * r_dist / max_dist

#     # 모터 속도 설정
#     bot.set_left_motor_speed(v_l)
#     bot.set_right_motor_speed(v_r)

#     while True:
#         l_delta = bot.get_left_encoder_reading() - init_l
#         r_delta = bot.get_right_encoder_reading() - init_r

#         l_d = wheel_radius * l_delta
#         r_d = wheel_radius * r_delta

#         if abs(l_d) >= abs(l_dist) or abs(r_d) >= abs(r_dist):
#             bot.set_left_motor_speed(0)
#             bot.set_right_motor_speed(0)
#             bot.stop_motors()
#             break
#         time.sleep(0.01)


# def turn_right(bot, deg = 150, speed=8):
#     """
#     Turn right by `deg` degrees using only wheel encoders.
#     """
#     wheel_radius=90
#     axel_length=184
#     # 목표 바퀴 이동 거리 계산
#     delta_theta_rad = np.deg2rad(deg)
#     l_dist = delta_theta_rad * (axel_length / 2)
#     r_dist = -delta_theta_rad * (axel_length / 2)

#     # 초기 엔코더 값 저장
#     init_l = bot.get_left_encoder_reading()
#     init_r = bot.get_right_encoder_reading()

#     # 속도 비율 설정
#     max_dist = max(abs(l_dist), abs(r_dist))
#     v_l = speed * l_dist / max_dist
#     v_r = speed * r_dist / max_dist

#     # 모터 속도 설정
#     bot.set_left_motor_speed(v_l)
#     bot.set_right_motor_speed(v_r)

#     while True:
#         l_delta = bot.get_left_encoder_reading() - init_l
#         r_delta = bot.get_right_encoder_reading() - init_r

#         l_d = wheel_radius * l_delta
#         r_d = wheel_radius * r_delta

#         if abs(l_d) >= abs(l_dist) or abs(r_d) >= abs(r_dist):
#             bot.set_left_motor_speed(0)
#             bot.set_right_motor_speed(0)
#             bot.stop_motors()
#             break
#         time.sleep(0.01)

# ============================================
# MAIN LOOP
# ============================================
def main():
    bot = HamBot()
    
    
    motions = ["forward","right_turn", "forward", "forward", "forward", "right_turn", "forward", "forward", "forward", "right_turn", "forward"]
    #motions = ["forward", "forward", "right_turn", "forward", "forward", "right_turn", "forward", "forward", "forward", "right_turn", "forward", "forward", "forward", "right_turn", "forward", "right_turn", "forward", "forward"]
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
        curr_ratio = debug_particles_weighted(bot, particles)

        if curr_ratio != 0:
            print("Localization success! >80% particles converged.")
            bot.set_left_motor_speed(0)
            bot.set_right_motor_speed(0)
            bot.stop_motors()
            break
            
        print("debug estimation")
        # --- ESTIMATION ---
        # cell, count = estimate_position(particles)
        # print(f"Estimated cell = {cell}, count = {count}/{N_PARTICLES}")
        
        # --- ROBOT ACTION ---
        if command == "forward":
            drive_forward(bot, STEP_DISTANCE)
            bot.set_left_motor_speed(0)
            bot.set_right_motor_speed(0)
            
        elif command == "left_turn":
            turn_left(bot, 90)
            bot.set_left_motor_speed(0)
            bot.set_right_motor_speed(0)
        elif command == "right_turn":
            turn_right(bot, 90)
            bot.set_left_motor_speed(0)
            bot.set_right_motor_speed(0)

        time.sleep(0.3)

    print("Finished.")

if __name__ == "__main__":
    main()


    #motions = ["forward", "right_turn", "forward", "forward", "forward", "right_turn", "forward", "forward", "left_turn", "left_turn", "forward", "forward", "forward", "right_turn", "forward", "forward", "forward", "right_turn", "forward", "forward", "forward", "right_turn", "forward", "right_turn", "forward", "forward"]
    #motions = ["left_turn", "forward", "left_turn", "forward", "forward", "forward", "right_turn", "forward", "forward", "left_turn"]
