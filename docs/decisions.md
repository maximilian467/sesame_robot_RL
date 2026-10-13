# Decision log

Every real choice in this project: the question, the options, what was picked, and why.

## 2026-10-13 — One public monorepo

- **Question:** Separate repos for code / firmware / docs, or one repo? Public from day one?
- **Options:** public monorepo · private first · local only
- **Choice:** One public repo for simulation, training, firmware, hardware notes and the journal.
- **Why:** A robotics project is more than code. The value lies in connecting the pieces: hardware deviations, calibration, measured servo behaviour, sim vs. real comparisons.

## 2026-10-13 — CPU training: MuJoCo + Stable-Baselines3

- **Question:** Which simulator and RL stack?
- **Options:** Isaac Lab / MJX / mjlab (GPU, thousands of parallel envs) · MuJoCo + Stable-Baselines3 (CPU)
- **Choice:** MuJoCo + Stable-Baselines3 on CPU.
- **Why:** Neither machine has an NVIDIA GPU. An 8-DOF robot is small enough to train on CPU in hours, and the stack is the one I already used in earlier projects (Ant, Reacher, ball balancer).

## 2026-10-13 — Develop on the laptop, long runs on the home server

- **Options:** laptop only · home server only · both
- **Choice:** Iterate on the laptop (i7-13700H, 20 threads), run long seed sweeps 24/7 on the Mini-PC (i5-8500T, 6 cores).
- **Why:** Early on, the bottleneck is how fast I can change something and see the effect, not raw compute. The Mini-PC is about 3–4× slower but can run around the clock.

## 2026-10-13 — Own environment, sesame-ml's model

- **Question:** Use `lukehollis/sesame-ml` as-is?
- **Choice:** Reuse its MJCF model and meshes, write my own Gymnasium environment.
- **Why:** The package currently doesn't run: `envs/core.py` was excluded by a `.gitignore` pattern (upstream issue #1). Also, writing the environment myself is the point of the learning project.

## 2026-10-13 — Switchable observations: blind vs. IMU

- **Choice:** The environment supports two observation sets: `blind` (only what the robot commands itself) and `imu` (plus gravity vector and angular velocity, as the BNO055 provides).
- **Why:** The stock robot has no sensors; mine has a BNO055. Training both shows how much the IMU actually helps.

## 2026-10-13 — ControlBench on the home server

- **Choice:** ControlBench moves to the Mini-PC as the single instance, reachable only inside my Tailscale network.
- **Why:** Training runs on two machines should report to one place that is always on.

## 2026-10-13 — D0: uv for the Python environment

- **Options:** `venv` + `requirements.txt` · `uv` + `pyproject.toml` + `uv.lock`
- **Choice:** uv.
- **Why:** uv pins the Python version itself (3.11) and locks every package, including transitive dependencies, so the laptop (Windows) and the Mini-PC (Ubuntu) run identical software. `torch` comes from PyTorch's CPU-only index, which avoids several GB of CUDA libraries on Linux.

## 2026-10-13 — D0b: English everywhere

- **Options:** English README + German notes · all English · all German
- **Choice:** All English: README, journal, concept notes, code.
- **Why:** The repo is public, and the sim-to-real story is most useful to the international Sesame community.
