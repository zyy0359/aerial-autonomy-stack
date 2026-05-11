# Spray DQN on the Existing Apple Orchard

This experiment reuses the existing AAS `apple_orchard` Gazebo world instead of creating a separate orchard map.

The scripts parse:

- `simulation/simulation_resources/simulation_worlds/apple_orchard.sdf`
- referenced models such as `birch_row`, `apple_grid`, `apple`, and `birch`

Only active SDF `<include>` entries are used. In the current world, `birch_row_1` is active and `apple_grid_1/2/3` are commented out, so generated missions currently cover the active birch row. If the apple grids are enabled in the SDF later, the same parser will include those trees too.

## Commands

## Local Planning Pipeline

Run the whole local planning pipeline. This trains DQN, evaluates it against baseline planners, generates the AAS mission YAML, and writes plots:

```bash
.venv/bin/python experiments/spray_dqn/run_pipeline.py --timesteps 20000
```

If a trained model already exists, skip training:

```bash
.venv/bin/python experiments/spray_dqn/run_pipeline.py --skip-train --model experiments/spray_dqn/outputs/models/dqn_apple_orchard.zip
```

Evaluate the path derived from the current orchard world:

```bash
.venv/bin/python experiments/spray_dqn/evaluate_dqn.py --model experiments/spray_dqn/outputs/models/dqn_apple_orchard.zip
```

Compare planners and generate chart outputs:

```bash
.venv/bin/python experiments/spray_dqn/compare_algorithms.py --model experiments/spray_dqn/outputs/models/dqn_apple_orchard.zip
```

Train additional RL baselines for paper comparison:

```bash
.venv/bin/python experiments/spray_dqn/train_sb3_algorithms.py --algorithms ppo,a2c,sac,td3,maskable-ppo --timesteps 20000
.venv/bin/python experiments/spray_dqn/train_dqn_variants.py --algorithms double-dqn,dueling-dqn,rainbow-dqn-lite --timesteps 20000
```

Build the paper-ready RL comparison table and plots:

```bash
.venv/bin/python experiments/spray_dqn/compare_rl_results.py
```

The paper comparison script writes:

- `experiments/spray_dqn/outputs/paper_comparison/rl_benchmark.json`
- `experiments/spray_dqn/outputs/paper_comparison/rl_benchmark_table.csv`
- `experiments/spray_dqn/outputs/paper_comparison/rl_benchmark_table.md`
- `experiments/spray_dqn/outputs/paper_comparison/rl_benchmark_metrics.png`
- `experiments/spray_dqn/outputs/paper_comparison/rl_coverage_over_waypoints.png`
- `experiments/spray_dqn/outputs/paper_comparison/rl_path_overlays.png`

Current single-seed 20k-step ranking on the active orchard world:

| Rank | Algorithm | Coverage | Path length | Repeat spray | Collisions |
|---:|---|---:|---:|---:|---:|
| 1 | Rainbow DQN lite | 100.0% | 360 m | 54.3% | 0 |
| 2 | PPO | 100.0% | 380 m | 70.4% | 0 |
| 3 | DQN | 100.0% | 410 m | 43.2% | 0 |
| 4 | Dueling DQN | 100.0% | 410 m | 47.5% | 0 |
| 5 | Double DQN | 100.0% | 410 m | 53.3% | 0 |

`rainbow-dqn-lite` is a Rainbow-style implementation with Double DQN, Dueling network, and prioritized replay. It does not include C51 distributional learning or NoisyNet exploration, so describe it as "Rainbow-style" or "Rainbow DQN lite" in the paper unless those components are added later. `SAC` and `TD3` are run through a continuous-action wrapper that maps 2D continuous directions back to grid moves, so they are useful secondary baselines but not as directly matched to this discrete path-planning task as DQN/PPO/A2C.

The comparison script writes:

- `experiments/spray_dqn/outputs/comparison/comparison.json`
- `experiments/spray_dqn/outputs/comparison/comparison_metrics.png`
- `experiments/spray_dqn/outputs/comparison/coverage_over_waypoints.png`
- `experiments/spray_dqn/outputs/comparison/path_overlays.png`

Generate an AAS mission YAML:

```bash
.venv/bin/python experiments/spray_dqn/generate_spray_mission.py --policy dqn --model experiments/spray_dqn/outputs/models/dqn_apple_orchard.zip
```

The DQN mission generator now uses a 100% coverage target by default and allows up to 120 reposition waypoints before compression. The current trained DQN path writes 83 reposition waypoints.

You can also generate missions for baseline planners:

```bash
.venv/bin/python experiments/spray_dqn/generate_spray_mission.py --policy orchard-row --output aircraft/aircraft_resources/missions/spray_orchard_row.yaml
.venv/bin/python experiments/spray_dqn/generate_spray_mission.py --policy nearest-target --output aircraft/aircraft_resources/missions/spray_nearest_target.yaml
```

Generate a Gazebo mission from any saved RL result:

```bash
.venv/bin/python experiments/spray_dqn/generate_mission_from_result.py --summary experiments/spray_dqn/outputs/metrics/rainbow-dqn-lite_summary.json --output aircraft/aircraft_resources/missions/spray_rainbow_dqn.yaml
```

## Full AAS/Gazebo Startup

Start the orchard simulation in dev mode so the generated mission on the host is mounted into the aircraft container:

```bash
DEV=true AUTOPILOT=px4 NUM_QUADS=1 NUM_VTOLS=0 WORLD=apple_orchard HEADLESS=false CAMERA=false LIDAR=false GND_CONTAINER=false RTF=1.0 ./scripts/sim_run.sh
```

The actual flight view appears in the Gazebo Sim window launched from the `Simulation` xterm. If `GND_CONTAINER=false`, QGroundControl is launched inside the simulation container too; otherwise QGroundControl appears from the ground container.

Run the generated mission in the aircraft container:

```bash
docker exec -d aircraft-container-inst0_1 bash -c "source /opt/ros/humble/setup.bash && source /aas/github_ws/install/setup.bash && source /aas/aircraft_ws/install/setup.bash && ros2 run mission mission --conops spray_dqn.yaml --ros-args -r __ns:=/Drone1 -p use_sim_time:=true"
```

Watch live position topics from another terminal:

```bash
docker exec -it aircraft-container-inst0_1 bash -c "source /opt/ros/humble/setup.bash && source /aas/github_ws/install/setup.bash && source /aas/aircraft_ws/install/setup.bash && ros2 topic echo /Drone1/fmu/out/vehicle_local_position"
```

For PX4, useful live topics include:

- `/Drone1/fmu/out/vehicle_local_position`
- `/Drone1/fmu/out/vehicle_global_position`
- `/Drone1/fmu/out/vehicle_odometry`

## What You Can See

- Gazebo Sim: actual 3D aircraft motion in the orchard world.
- QGroundControl: live vehicle state, telemetry, and map/route view.
- ROS 2 topics: numeric live trajectory/position streams.
- Offline comparison plots: coverage, path length, repeat spray ratio, collision count, coverage-over-waypoints, and path overlays.
