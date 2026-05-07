# System Status

This is now a DQN high-level spray path planning prototype on the existing AAS apple orchard world.

## What It Does

- Reuses `simulation/simulation_resources/simulation_worlds/apple_orchard.sdf`.
- Parses active SDF includes and referenced model files to find current tree targets and static obstacles.
- Trains a DQN policy over a compact planning state derived from the original SDF world.
- Evaluates coverage, repeat spray ratio, path length, and collision count in the planning layer.
- Converts the DQN path into `aircraft/aircraft_resources/missions/spray_dqn.yaml`.
- Lets the existing AAS mission node execute the generated waypoints in Gazebo.

## Current Verified Result

With the current `apple_orchard.sdf` active entities:

- targets: 21
- static obstacles: 8
- DQN coverage: 95.24%
- DQN path length: 400 m
- DQN planning collisions: 0
- generated mission reposition waypoints: 80

## Current Limits

- Obstacle avoidance is high-level static avoidance based on the SDF map, not real-time perception.
- Spray is represented as coverage accounting in Python, not liquid physics in Gazebo.
- The currently active orchard targets are from `birch_row_1`; the `apple_grid_1/2/3` includes in `apple_orchard.sdf` are commented out.
- Full 3D validation still depends on launching AAS/Gazebo and running the generated mission.

