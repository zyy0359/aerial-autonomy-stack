# Hierarchical RL Orchard Spraying Report Draft

## 1. Task and Method Summary

This project studies autonomous orchard spraying in AAS using a grid-level reinforcement-learning planner. The agent navigates an orchard map, selects movement actions, and satisfies spraying demand under safety constraints. The enhanced setting uses intelligent irrigation demand, automatic spray activation, dynamic obstacles, and a safety controller.

Action spaces:

- 4-action navigation: move in cardinal grid directions; spraying is handled by the environment policy.
- 8-action navigation plus spray control: movement direction is combined with learned spray on/off decisions.

Reward design:

- Positive reward for newly covered target cells and demand satisfaction.
- Penalty for repeated spraying, inefficient paths, collisions, and unsafe proximity.
- Success is evaluated by goal progress, using demand satisfaction in intelligent-irrigation runs.

Algorithm set:

- DQN baseline.
- DRQN for partial observability and temporal context.
- Dueling DQN for value/advantage decomposition.
- Rainbow DQN lite with lightweight improvements over vanilla DQN.

## 2. Statistical Protocol

- Seeds: `7,11,19,23,31`.
- Main hierarchical enhanced experiment: 15,000 training steps.
- Ablations: 10,000 training steps.
- Episode max steps: 500.
- Success threshold: goal progress >= 90% and zero planning collisions.
- Goal metric: demand satisfaction for intelligent-irrigation experiments.
- Reported values: mean +/- sample standard deviation over 5 seeds.
- Learning curves: reward, coverage, and demand satisfaction are logged during training and summarized as mean/std curves.

Hardware/software fields to fill before paper submission:

- CPU:
- GPU:
- RAM:
- OS:
- Python:
- PyTorch:
- Gymnasium:
- Stable-Baselines3:
- AAS/Gazebo version or commit:

## 3. Main 5-Seed Results

Enhanced setting: intelligent irrigation, auto-spray, safety controller, 2 dynamic obstacles in corridor mode.

| Rank | Algorithm | Success rate | Goal progress | Coverage | Demand satisfaction | Planning collisions |
|---:|---|---:|---:|---:|---:|---:|
| 1 | DQN | 80.0% | 78.7 +/- 29.3% | 82.9 +/- 30.4% | 78.7 +/- 29.3% | 0.0 +/- 0.0 |
| 2 | DRQN | 60.0% | 80.9 +/- 14.7% | 83.8 +/- 15.6% | 80.9 +/- 14.7% | 0.0 +/- 0.0 |
| 3 | Dueling DQN | 60.0% | 55.0 +/- 50.2% | 57.1 +/- 52.2% | 55.0 +/- 50.2% | 0.0 +/- 0.0 |
| 4 | Rainbow DQN lite | 20.0% | 60.2 +/- 24.4% | 63.8 +/- 23.7% | 60.2 +/- 24.4% | 0.0 +/- 0.0 |

Interpretation:

- DQN has the highest success rate in the current 5-seed enhanced setting.
- DRQN has slightly higher mean goal progress and lower variance than DQN, but lower success count.
- All enhanced runs report zero planning and dynamic collisions, supporting the safety-controller result.
- Dueling DQN and Rainbow lite are not yet competitive in this short-training configuration.

## 4. Ablation Results

| Comparison setting | Best algorithm | Success rate | Goal progress | Coverage | Collisions |
|---|---|---:|---:|---:|---:|
| 4 actions, always spray, no safety | DRQN | 0.0% | 48.2 +/- 31.5% | 52.4 +/- 33.8% | 0.0 +/- 0.0 |
| 4 actions, auto-spray, no safety | DRQN | 40.0% | 86.0 +/- 13.7% | 88.6 +/- 14.9% | 0.6 +/- 0.5 |
| 4 actions, always spray, safety | DQN | 40.0% | 59.8 +/- 40.7% | 64.8 +/- 42.4% | 0.0 +/- 0.0 |
| 4 actions, auto-spray, safety | DQN | 60.0% | 72.0 +/- 29.4% | 75.2 +/- 29.8% | 0.0 +/- 0.0 |
| 8 actions, learned spray, no safety | DRQN | 20.0% | 88.1 +/- 9.1% | 91.4 +/- 11.4% | 0.8 +/- 0.4 |
| 8 actions, learned spray, safety | DRQN | 60.0% | 72.1 +/- 27.6% | 75.2 +/- 27.8% | 0.0 +/- 0.0 |

Key ablation conclusions:

- Auto-spray is the most important functional enhancement. In the 4-action setting, enabling auto-spray raises the best success rate from 0% to 40% without safety, and from 40% to 60% with safety.
- The safety controller reliably removes collisions. The auto-spray no-safety setting has nonzero collision counts, while the corresponding safety setting has zero collisions.
- DRQN is stronger than DQN in settings with explicit spray control or no safety. This suggests recurrent state helps when the action/state coupling is harder.
- 8-action learned spray can reach high goal progress with DRQN, but it is less stable and collision-prone without safety. The 4-action auto-spray design is simpler and currently more robust.

## 5. Learning Curves

Learning-curve logging was added for:

- Episode reward.
- Coverage percentage.
- Demand satisfaction percentage.

The ablation suite has full 5-seed learning curves. The main enhanced 5-seed result currently has learning curves for the newly added seeds 23 and 31 only, because seeds 7, 11, and 19 were existing runs and were skipped by the resume logic.

For the paper, either rerun the main enhanced setting with `--force --learning-curves`, or state that the main table is 5-seed while the learning-curve figure is reported for the newly logged subset.

## 6. Gazebo Replay Verification

A mission YAML has been generated from a successful RL path:

- Algorithm: DRQN.
- Seed: 7.
- Coverage: 95.2%.
- Demand satisfaction: 90.9%.
- Path length: 410.0 m.
- Planning collisions: 0.
- Dynamic collisions: 0.
- Mission waypoints: 83.

Current status:

- Mission artifact is ready.
- Replay command and verification checklist are ready.
- Actual AAS/Gazebo execution evidence is still pending and should be collected before final paper submission.

Required replay evidence:

- Screenshot or log showing the apple-orchard world launched.
- Mission log showing takeoff and waypoint execution.
- ROS 2 position topic or trajectory evidence.
- Completion/landing log.
- Confirmation that no visible crash occurred during replay.

## 7. Paper Readiness Assessment

The project is close to a writeable paper draft. The core ingredients are now present: method definition, multi-seed comparison, ablation study, learning-curve logging, and a Gazebo replay artifact.

Before submission-quality writing, the remaining high-priority items are:

- Complete the actual Gazebo replay and save visual/log evidence.
- Rerun or backfill main enhanced learning curves for all 5 seeds.
- Add hardware/software environment details.
- Decide the final claim carefully: current data supports "auto-spray plus safety improves robust orchard spraying"; it does not yet support a strong claim that one RL algorithm dominates universally.

