# 2026-10-13 — Project start

## What happened
- Ordered the parts for my own Sesame build: ESP32-S3 SuperMini, PCA9685 servo driver, BNO055 IMU, OLED, 10× MG90S, 2S 14500 battery. The power regulator is a Hobbywing UBEC 5A I already had.
- Surveyed the existing Sesame RL work:
  - `lukehollis/sesame-ml` has a MuJoCo model and a solid sim-to-real design (servo lag model, domain randomization), but its environment code is missing from the repo.
  - `Karabibik/sesameRL` trains a walking policy with GPU simulation, but uses observations the real robot can't measure.
  - None of the projects I found has a learned policy walking on a real robot yet.
- Set up this repo: uv environment (Python 3.11, MuJoCo, Gymnasium, Stable-Baselines3, CPU-only torch).

## Why RL instead of the stock gait?
The stock firmware plays back hand-made keyframes: a fixed list of servo angles with fixed delays. It never checks what actually happens. If the robot slips or tilts, the gait just continues. An RL policy is a small neural network that picks the next servo targets every 20 ms based on what it observes. In simulation, it can learn a gait that copes with slipping, varying friction and servo lag, provided the simulation contains those effects.

## Decisions
See [decisions.md](../decisions.md): monorepo, CPU MuJoCo + SB3, laptop/home-server split, own environment, blind vs. IMU observations, uv, English.

## Next
Phase 1: open the MuJoCo model and go through it piece by piece (bodies, joints, actuators, collision shapes), then replay the stock firmware gait in simulation as a baseline.
