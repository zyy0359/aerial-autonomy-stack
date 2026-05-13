# System Status

This is now a DQN high-level spray path planning prototype on the existing AAS apple orchard world.

## What It Does

- Reuses `simulation/simulation_resources/simulation_worlds/apple_orchard.sdf`.
- Parses active SDF includes and referenced model files to find current tree targets and static obstacles.
- Trains a DQN policy over a compact planning state derived from the original SDF world.
- Evaluates DQN, PPO, A2C, SAC, TD3, Double DQN, Dueling DQN, Rainbow DQN lite, orchard-row traversal, and nearest-target greedy planning in the planning layer.
- Optionally enables moving grid obstacles, corridor-mode dynamic obstacle placement, a demand-aware irrigation map, spray on/off control, `S_v` dynamic safety scoring, and DRQN recurrent training for paper experiments.
- Generates comparison charts for coverage, path length, repeat spray ratio, collision count, coverage over waypoints, and path overlays.
- Converts the DQN path into `aircraft/aircraft_resources/missions/spray_dqn.yaml`.
- Converts the best current Rainbow DQN lite path into `aircraft/aircraft_resources/missions/spray_rainbow_dqn.yaml`.
- Lets the existing AAS mission node execute the generated waypoints in Gazebo.
- Supports live flight observation through Gazebo Sim, QGroundControl, and ROS 2 position topics.

## Current Verified Result

With the current `apple_orchard.sdf` active entities:

- targets: 21
- static obstacles: 8
- DQN coverage: 100.00%
- DQN path length: 410 m
- DQN planning collisions: 0
- generated mission reposition waypoints: 83
- best current RL coverage: 100.00% (`rainbow-dqn-lite`)
- best current RL path length: 360 m
- best current RL planning collisions: 0
- best current RL mission reposition waypoints: 73

## Main Outputs

- `aircraft/aircraft_resources/missions/spray_dqn.yaml`
- `aircraft/aircraft_resources/missions/spray_rainbow_dqn.yaml`
- `experiments/spray_dqn/outputs/metrics/evaluation.json`
- `experiments/spray_dqn/outputs/plots/dqn_coverage.png`
- `experiments/spray_dqn/outputs/comparison/comparison_metrics.png`
- `experiments/spray_dqn/outputs/comparison/coverage_over_waypoints.png`
- `experiments/spray_dqn/outputs/comparison/path_overlays.png`
- `experiments/spray_dqn/outputs/paper_comparison/rl_benchmark_table.csv`
- `experiments/spray_dqn/outputs/paper_comparison/rl_benchmark_metrics.png`
- `experiments/spray_dqn/outputs/paper_comparison/rl_coverage_over_waypoints.png`
- `experiments/spray_dqn/outputs/paper_comparison/rl_path_overlays.png`

## Current Limits

- Obstacle avoidance is high-level static avoidance based on the SDF map, not real-time perception.
- Spray is represented as coverage accounting in Python, not liquid physics in Gazebo.
- The currently active orchard targets are from `birch_row_1`; the `apple_grid_1/2/3` includes in `apple_orchard.sdf` are commented out.
- Full 3D validation still depends on launching AAS/Gazebo and running the generated mission.
- Current RL ranking is single-seed. For a paper-grade claim, run multiple seeds and report mean/std.
- Dynamic obstacles, intelligent irrigation demand, and `S_v` are implemented in the Python planning environment; they are not yet physical moving actors or liquid simulation inside Gazebo.
- A 97% threshold is supported as an experiment success target (`--goal-coverage 0.97`), but it must be demonstrated by trained results before being claimed in the paper.
- `rainbow-dqn-lite` does not include C51 distributional learning or NoisyNet exploration.
- `Maskable PPO` requires `sb3-contrib`; it is currently skipped in this environment.
- `SAC` and `TD3` use a continuous-action wrapper mapped back to grid moves, so they are less directly matched to this discrete planning task.
