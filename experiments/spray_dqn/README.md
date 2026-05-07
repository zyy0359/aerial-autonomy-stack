# Spray DQN on the Existing Apple Orchard

This experiment reuses the existing AAS `apple_orchard` Gazebo world instead of creating a separate orchard map.

The scripts parse:

- `simulation/simulation_resources/simulation_worlds/apple_orchard.sdf`
- referenced models such as `birch_row`, `apple_grid`, `apple`, and `birch`

Only active SDF `<include>` entries are used. In the current world, `birch_row_1` is active and `apple_grid_1/2/3` are commented out, so generated missions currently cover the active birch row. If the apple grids are enabled in the SDF later, the same parser will include those trees too.

## Commands

Run the whole local planning pipeline:

```bash
.venv/bin/python experiments/spray_dqn/run_pipeline.py --timesteps 20000
```

Evaluate the path derived from the current orchard world:

```bash
.venv/bin/python experiments/spray_dqn/evaluate_dqn.py --model experiments/spray_dqn/outputs/models/dqn_apple_orchard.zip
```

Generate an AAS mission YAML:

```bash
.venv/bin/python experiments/spray_dqn/generate_spray_mission.py --policy dqn --model experiments/spray_dqn/outputs/models/dqn_apple_orchard.zip
```

Start the orchard simulation in dev mode so the generated mission on the host is mounted into the aircraft container:

```bash
DEV=true AUTOPILOT=px4 NUM_QUADS=1 NUM_VTOLS=0 WORLD=apple_orchard HEADLESS=false CAMERA=false LIDAR=false GND_CONTAINER=false RTF=1.0 ./scripts/sim_run.sh
```

The actual flight view appears in the Gazebo Sim window launched from the `Simulation` xterm. If `GND_CONTAINER=false`, QGroundControl is launched inside the simulation container too; otherwise QGroundControl appears from the ground container.

Run the generated mission in the aircraft container:

```bash
docker exec -d aircraft-container-inst0_1 bash -c "source /opt/ros/humble/setup.bash && source /aas/github_ws/install/setup.bash && source /aas/aircraft_ws/install/setup.bash && ros2 run mission mission --conops spray_dqn.yaml --ros-args -r __ns:=/Drone1 -p use_sim_time:=true"
```
