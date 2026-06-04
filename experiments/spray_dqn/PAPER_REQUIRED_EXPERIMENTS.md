# Paper-Required Experiments

This note tracks the additional experiments needed before making paper-grade claims.

## Required Runs

Run the full suite:

```bash
.venv/bin/python experiments/spray_dqn/run_paper_experiments.py --force
```

For WSL from PowerShell:

```powershell
wsl --cd /mnt/c/Users/zyy/Documents/GitHub/aerial-autonomy-stack bash -lc ".venv/bin/python experiments/spray_dqn/run_paper_experiments.py --force"
```

The default suite writes to:

```text
experiments/spray_dqn/outputs/paper_required_experiments/
```

It includes:

- `hierarchical_5seeds`: DQN, DRQN, Dueling DQN, Rainbow DQN lite over seeds `7,11,19,23,31`.
- `ablation_always_spray_no_safety`: no auto-spray, no safety controller.
- `ablation_autospray_no_safety`: auto-spray only.
- `ablation_always_spray_safety`: safety controller only.
- `ablation_autospray_safety`: auto-spray plus safety controller.
- `ablation_explicit_spray_no_safety`: 8-action learned spray control, no safety controller.
- `ablation_explicit_spray_safety`: 8-action learned spray control with safety controller.

Each run stores:

- `summary/multiseed_summary.md`
- `summary/multiseed_summary.csv`
- `summary/multiseed_per_seed.csv`
- `summary/learning_curves.png`
- `summary/learning_curve_summary.csv`

## Learning Curves

Learning curves are periodic deterministic evaluations during training, not rolling training episode logs. Each point reports:

- evaluation return
- coverage
- demand satisfaction
- goal progress
- path length
- collisions
- dynamic collisions
- minimum safety value

The default evaluation interval is `5000` training timesteps. Change it with:

```bash
.venv/bin/python experiments/spray_dqn/run_paper_experiments.py --eval-freq 2500
```

## Gazebo Replay Verification

Generate one replay artifact from a successful saved RL summary:

```bash
.venv/bin/python experiments/spray_dqn/prepare_gazebo_replay.py \
  --summary experiments/spray_dqn/outputs/paper_required_experiments/hierarchical_5seeds/seed_7/metrics/drqn_summary.json \
  --mission-out aircraft/aircraft_resources/missions/spray_paper_replay.yaml \
  --report-out experiments/spray_dqn/outputs/gazebo_replay/gazebo_replay_report.md
```

Then launch AAS/Gazebo and execute `spray_paper_replay.yaml` using the commands written in:

```text
experiments/spray_dqn/outputs/gazebo_replay/gazebo_replay_report.md
```

For the paper, include at least one screenshot or log excerpt showing:

- Gazebo `apple_orchard` world launched.
- UAV takeoff succeeded.
- Reposition waypoint sequence executed.
- Mission completed or landed.
- No visible crash during replay.

## Statistical Reporting

Use these definitions consistently:

- Success: `goal_progress >= goal_coverage` and planning-layer collisions equal `0`.
- Static coverage experiments: `goal_metric=coverage`, usually `goal_coverage=1.0`.
- Enhanced intelligent irrigation experiments: `goal_metric=demand`, default `goal_coverage=0.90`.
- Reported values: mean +/- sample standard deviation over completed random seeds.
- Seeds: report the exact seed list, not only the count.
- Path length: grid path length converted by `cell_size_m`.
- Collisions: planning-layer grid collisions; Gazebo replay crash status is reported separately.
- Dynamic collisions: moving-obstacle collisions in the Python planning environment.
- `S_v`: dynamic obstacle safety value, where `1.0` is safest and `0.0` means collision-distance proximity.

Minimum table columns for the paper:

- algorithm
- seeds
- success rate
- goal progress
- coverage
- demand satisfaction when intelligent irrigation is enabled
- path length
- repeat spray ratio
- collisions
- dynamic collisions
- minimum `S_v`
- episode max steps
- training timesteps

## Hardware and Software Record

Fill this in before final submission:

```text
OS:
CPU:
GPU:
RAM:
Python:
PyTorch:
Stable-Baselines3:
CUDA:
Docker:
NVIDIA driver:
ROS 2:
Gazebo:
AAS commit:
Training device:
```

The repository already records the algorithm hyperparameters in each `*_summary.json` and the active run settings in `multiseed_summary.json`.
