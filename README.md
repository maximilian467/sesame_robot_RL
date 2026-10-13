# Sesame Robot RL

Teaching a small, 3D-printed quadruped — the [Sesame Robot](https://github.com/dorianborian/sesame-robot) — to walk with reinforcement learning, from MuJoCo simulation to the real robot.

> **Status:** project setup. The parts are on their way; training starts in simulation.

## Why this repo

Most Sesame RL projects stop in simulation. This repo documents the whole sim-to-real journey in the open: the simulation model, the training code, the firmware changes, the hardware deviations from the original build, measurements of the real servos, and an honest log of what worked and what didn't.

It is also a learning project — every major decision is written down with its reasoning in [`docs/decisions.md`](docs/decisions.md), and the [`docs/journal/`](docs/journal/) tracks progress step by step.

## Roadmap

- [x] Phase 0 — Repository and Python environment
- [ ] Phase 1 — Understand and verify the MuJoCo model; replay the stock firmware gait as a baseline
- [ ] Phase 2 — Own Gymnasium environment `SesameWalk-v0`
- [ ] Phase 3 — First PPO training runs (laptop), progress tracked in ControlBench
- [ ] Phase 4 — Long runs on a home server (24/7)
- [ ] Later — Build the robot, port the firmware, measure the servos, deploy the policy on the ESP32-S3

## Hardware (my build)

Differs from the stock build: servos are driven by a PCA9685 board instead of ESP32 GPIO pins, and a BNO055 IMU is added so the policy can sense tilt.

| Part | Notes |
|---|---|
| ESP32-S3 SuperMini | Microcontroller |
| PCA9685 16-channel servo driver | I2C, 0x40 |
| BNO055 9-axis IMU | I2C, 0x28, on-chip sensor fusion |
| 0.96" SSD1306 OLED | I2C, 0x3C, the face |
| 8× MG90S servos (180°) | 2 DOF per leg |
| 2S 14500 Li-ion pack, 7.4 V 800 mAh | Main battery |
| Hobbywing UBEC 5A | Steps the battery down to 5–6 V |

## Quickstart

Requires [uv](https://docs.astral.sh/uv/).

```bash
uv sync
uv run python -c "import mujoco, gymnasium, stable_baselines3"
```

## Training setup

- **Simulator:** [MuJoCo](https://mujoco.org/) on CPU
- **RL library:** [Stable-Baselines3](https://stable-baselines3.readthedocs.io/)
- **Experiment tracking:** [ControlBench](https://github.com/maximilian467/ControlBench), my own benchmark dashboard

## Upstream projects

- [dorianborian/sesame-robot](https://github.com/dorianborian/sesame-robot) — the robot itself (Apache-2.0)
- [lukehollis/sesame-ml](https://github.com/lukehollis/sesame-ml) — MuJoCo model this project starts from (Apache-2.0)

See [`NOTICE`](NOTICE) for attribution. For local reference, both are cloned into the git-ignored `external/` folder:

```bash
git clone --depth 1 https://github.com/dorianborian/sesame-robot.git external/sesame-robot
git clone --depth 1 https://github.com/lukehollis/sesame-ml.git external/sesame-ml
```

## License

[Apache-2.0](LICENSE)
