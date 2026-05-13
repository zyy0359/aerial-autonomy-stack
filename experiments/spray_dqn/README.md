# Spray DQN on the Existing Apple Orchard

This experiment reuses the existing AAS `apple_orchard` Gazebo world instead of creating a separate orchard map.

The scripts parse:

- `simulation/simulation_resources/simulation_worlds/apple_orchard.sdf`
- referenced models such as `birch_row`, `apple_grid`, `apple`, and `birch`

Only active SDF `<include>` entries are used. In the current world, `birch_row_1` is active and `apple_grid_1/2/3` are commented out, so generated missions currently cover the active birch row. If the apple grids are enabled in the SDF later, the same parser will include those trees too.

## Implemented System

The current code implements a high-level orchard UAV spray planning system with:

- SDF parsing from the original AAS `apple_orchard.sdf`; no separate hand-built orchard map is required.
- A grid-based UAV planning environment with static obstacle avoidance and target-tree spray coverage accounting.
- DQN, Double DQN, Dueling DQN, Rainbow DQN lite, PPO, DRQN, and selected SB3 baselines.
- Optional intelligent irrigation / variable spray demand maps.
- Optional spray on/off control through `--spray-control`, expanding actions to movement plus spray decisions.
- Optional moving grid obstacles, including `--dynamic-obstacle-mode corridor` to place obstacles near orchard corridors.
- Dynamic safety scoring with `S_v`, plus dynamic collision and safety-violation metrics.
- Multi-seed experiment aggregation with mean/std tables and summary plots.
- Offline path visualizations for coverage, demand heatmaps, dynamic obstacle routes, and UAV paths.
- Mission YAML generation for replaying selected paths in AAS/Gazebo.

## Recommended Experiment Structure

Use three progressive experiment levels in the paper:

1. Static orchard coverage: static targets and static obstacles only. This is the clean baseline and currently supports choosing Dueling DQN as the main static planner.
2. Intelligent irrigation demand: enable variable demand and spray control, but keep dynamic obstacles off. This isolates whether the algorithm can match spray dose instead of only reaching trees.
3. Full enhanced task: enable dynamic obstacles, `S_v`, intelligent irrigation, spray control, and DRQN/Rainbow DQN lite. This tests system extensibility and exposes current stability limits.

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

Run the enhanced paper setting with dynamic obstacles, demand-aware intelligent irrigation, the `S_v` safety value, and a 97% goal threshold:

```bash
.venv/bin/python experiments/spray_dqn/run_top5_multiseed.py \
  --algorithms dqn,dueling-dqn,rainbow-dqn-lite,drqn \
  --seeds 7,11,19,23,31 \
  --timesteps 30000 \
  --goal-coverage 0.97 \
  --goal-metric demand \
  --dynamic-obstacles 2 \
  --intelligent-irrigation \
  --output-dir experiments/spray_dqn/outputs/enhanced_dynamic_irrigation_5seeds
```

From PowerShell, run the same command through WSL:

```powershell
wsl --cd /mnt/c/Users/zyy/Documents/GitHub/aerial-autonomy-stack bash -lc ".venv/bin/python experiments/spray_dqn/run_top5_multiseed.py --algorithms dqn,dueling-dqn,rainbow-dqn-lite,drqn --seeds 7,11,19,23,31 --timesteps 30000 --goal-coverage 0.97 --goal-metric demand --dynamic-obstacles 2 --intelligent-irrigation --output-dir experiments/spray_dqn/outputs/enhanced_dynamic_irrigation_5seeds"
```

The enhanced table adds goal progress, demand satisfaction, dynamic collisions, minimum `S_v`, dose RMSE, and over-spray metrics. `train_drqn.py` is the recurrent DQN baseline; it uses a GRU memory layer so the policy can infer obstacle motion from recent observations. The current 5-seed enhanced run writes:

- `experiments/spray_dqn/outputs/enhanced_dynamic_irrigation_5seeds/summary/multiseed_summary.md`
- `experiments/spray_dqn/outputs/enhanced_dynamic_irrigation_5seeds/summary/multiseed_summary.png`
- `experiments/spray_dqn/outputs/enhanced_dynamic_irrigation_5seeds/summary/rainbow_seed31_path.png`
- `experiments/spray_dqn/outputs/enhanced_dynamic_irrigation_5seeds/summary/drqn_seed31_path.png`

To regenerate an enhanced result visualization from any saved algorithm summary:

```bash
.venv/bin/python experiments/spray_dqn/plot_enhanced_result.py \
  --summary experiments/spray_dqn/outputs/enhanced_dynamic_irrigation_5seeds/seed_31/metrics/rainbow-dqn-lite_summary.json \
  --output experiments/spray_dqn/outputs/enhanced_dynamic_irrigation_5seeds/summary/rainbow_seed31_path.png
```

If your WSL session has a display server, add `--show` to open a live matplotlib window. Otherwise, use the saved PNG. During offline training, the terminal shows training logs rather than a live moving UAV; the 3D UAV flight view is available in Gazebo after converting a saved path to an AAS mission YAML.

Recommended next optimized enhanced run:

```powershell
wsl --cd /mnt/c/Users/zyy/Documents/GitHub/aerial-autonomy-stack bash -lc ".venv/bin/python experiments/spray_dqn/run_top5_multiseed.py --algorithms rainbow-dqn-lite,drqn --seeds 7,11,19,23,31 --timesteps 100000 --goal-coverage 0.97 --goal-metric demand --dynamic-obstacles 2 --dynamic-obstacle-mode corridor --spray-control --intelligent-irrigation --output-dir experiments/spray_dqn/outputs/enhanced_optimized_5seeds"
```

This optimized setting keeps the same 97% demand goal but gives the policy an explicit spray on/off choice and places moving obstacles near orchard corridors, making both over-spray control and `S_v` evaluation more meaningful. It is a better paper experiment than simply making the old 30k-step run longer.

Current optimized 5-seed result:

| Algorithm | Success rate | Demand satisfaction | Coverage | Dynamic collisions | Min `S_v` |
|---|---:|---:|---:|---:|---:|
| DRQN | 0.0% | 41.7 +/- 35.4% | 44.8 +/- 37.1% | 0.0 +/- 0.0 | 0.47 +/- 0.30 |
| Rainbow DQN lite | 0.0% | 10.1 +/- 14.9% | 11.4 +/- 15.6% | 0.0 +/- 0.0 | 0.60 +/- 0.37 |

This result is a useful failure diagnosis: simply increasing training steps to 100k while adding spray on/off control makes exploration harder. For the paper, report this as a limitation and use staged experiments rather than claiming the full enhanced task is solved.

Run only the middle intelligent-irrigation demand experiment:

```powershell
wsl --cd /mnt/c/Users/zyy/Documents/GitHub/aerial-autonomy-stack bash -lc ".venv/bin/python experiments/spray_dqn/run_top5_multiseed.py --algorithms dqn,dueling-dqn,rainbow-dqn-lite,drqn --seeds 7,11,19,23,31 --timesteps 60000 --goal-coverage 0.97 --goal-metric demand --dynamic-obstacles 0 --spray-control --intelligent-irrigation --output-dir experiments/spray_dqn/outputs/irrigation_demand_5seeds"
```

Key outputs for each multi-seed run are:

- `<output-dir>/summary/multiseed_summary.md`
- `<output-dir>/summary/multiseed_summary.csv`
- `<output-dir>/summary/multiseed_summary.json`
- `<output-dir>/summary/multiseed_summary.png`

The terminal log is useful for training progress, but the paper-ready numbers come from the summary directory.

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
