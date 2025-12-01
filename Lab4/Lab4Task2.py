# 1m = 1000mm, 100cm = 1m
# 60 cm = 600 mm 
import time
import numpy as np
import math
from robot_systems.robot import HamBot
import random
from collections import defaultdict

# -----------------------------
# HamBot 기반 초기화
# -----------------------------
# from hambot import HamBot  # 실제 로봇 구동 시
# bot = HamBot()

# -----------------------------
# Maze Map 정의 (예: 4x4)
# 각 cell[id] = (N, E, S, W) 1=wall, 0=no wall
# 실제 미로는 사용자가 가지고 있는 maze8.xml 기반으로 정의
# -----------------------------
maze_map = {
    0: (1, 0, 0, 1), 1: (1, 0, 0, 0), 2: (1, 0, 0, 0), 3: (1, 1, 0, 0),
    4: (0, 0, 0, 1), 5: (0, 0, 0, 0), 6: (0, 0, 0, 0), 7: (0, 1, 0, 0),
    8: (0, 0, 0, 1), 9: (0, 0, 0, 0), 10: (0, 0, 0, 0), 11: (0, 1, 0, 0),
    12: (0, 0, 1, 1), 13: (0, 0, 1, 0), 14: (0, 0, 1, 0), 15: (0, 1, 1, 0)
}

N_PARTICLES = 160
GRID_SIZE = 16

# -----------------------------
# Particle Class
# -----------------------------
class Particle:
    def __init__(self, cell_id):
        self.cell = cell_id
        self.weight = 1.0 / N_PARTICLES

# -----------------------------
# Particle Filter 초기화
# -----------------------------
particles = [Particle(cell_id=i % GRID_SIZE) for i in range(N_PARTICLES)]

# -----------------------------
# Motion 모델 (Perfect)
# -----------------------------
def move_particle(particle, command):
    cell_id = particle.cell
    n, e, s, w = maze_map[cell_id]
    
    # 방향은 항상 'up' 기준
    if command == "forward":
        if n == 0:
            new_cell = cell_id - 4 if cell_id >= 4 else cell_id
        else:
            new_cell = cell_id
    elif command == "backward":
        if s == 0:
            new_cell = cell_id + 4 if cell_id < 12 else cell_id
        else:
            new_cell = cell_id
    elif command == "left":
        if w == 0:
            new_cell = cell_id - 1 if cell_id % 4 != 0 else cell_id
        else:
            new_cell = cell_id
    elif command == "right":
        if e == 0:
            new_cell = cell_id + 1 if cell_id % 4 != 3 else cell_id
        else:
            new_cell = cell_id
    particle.cell = new_cell

# -----------------------------
# Sensor 모델 (Noisy)
# -----------------------------
sensor_probs = {
    0: {0:0.6, 1:0.4},  # s=0
    1: {0:0.2, 1:0.8}   # s=1
}

def compute_weight(particle, observation):
    """observation: (N, E, S, W) 0=no wall, 1=wall"""
    cell_signature = maze_map[particle.cell]
    weight = 1.0
    for s_obs, s_true in zip(observation, cell_signature):
        weight *= sensor_probs[s_true][s_obs]
    particle.weight = weight

# -----------------------------
# Resampling
# -----------------------------
def resample_particles(particles):
    weights = [p.weight for p in particles]
    total = sum(weights)
    if total == 0:
        # 모든 weight 0이면 균등 재배치
        new_particles = [Particle(random.randint(0, GRID_SIZE-1)) for _ in range(N_PARTICLES)]
        return new_particles
    normalized_weights = [w/total for w in weights]
    new_particles = random.choices(particles, weights=normalized_weights, k=N_PARTICLES)
    # 새 객체로 복제
    new_particles = [Particle(p.cell) for p in new_particles]
    return new_particles

# -----------------------------
# 추정치 계산
# -----------------------------
def estimate_position(particles):
    count = defaultdict(int)
    for p in particles:
        count[p.cell] += 1
    mode_cell = max(count, key=count.get)
    max_particles = count[mode_cell]
    return mode_cell, max_particles

# -----------------------------
# 출력
# -----------------------------
def print_particles(particles):
    count = [0]*GRID_SIZE
    for p in particles:
        count[p.cell] += 1
    print("Particle count per cell:")
    for i in range(4):
        print(count[i*4:i*4+4])

# -----------------------------
# PF 실행 예시
# -----------------------------
# 랜덤 초기 위치
for p in particles:
    p.cell = random.randint(0, GRID_SIZE-1)

# 시뮬레이션 반복
motions = ["forward", "forward", "right", "forward"]  # 예시 이동
for step, command in enumerate(motions):
    print(f"\nStep {step+1}: Motion = {command}")
    
    # 1. Prediction
    for p in particles:
        move_particle(p, command)
    
    # 2. 센서 관측 (예시, 실제 로봇에서는 bot.get_sensors())
    # 여기서는 랜덤 예시 관측값 (0=no wall, 1=wall)
    observation = tuple(random.randint(0,1) for _ in range(4))
    for p in particles:
        compute_weight(p, observation)
    
    # 3. Resampling
    particles = resample_particles(particles)
    
    # 4. 출력
    print_particles(particles)
    mode_cell, max_particles = estimate_position(particles)
    print(f"Estimated cell (mode) = {mode_cell}, max particles = {max_particles}")
    if max_particles / N_PARTICLES >= 0.8:
        print("Localization successful!")
        break
