# TODOs

## Roadmap

### Feature: LiDAR-inertial Odometry and SLAM

> Advanced localization and mapping baselines and capabilities

- [x] [KISS-ICP](https://github.com/PRBonn/kiss-icp) LiDAR odometry (LO) baseline
- [ ] Add config files (and shell) for the resolution/rate/extrinsic of the camera and lidar sensors/pipelines/topics
- [ ] Add visual-inertial (VIO) baseline (e.g., [open_vins](https://github.com/rpng/open_vins))
- [ ] Add  LiDAR-intertial (LIO) baseline (e.g., [SPARK-FAST-LIO](https://github.com/MIT-SPARK/spark-fast-lio))
- [ ] Add  LIVO baselines (e.g., [SuperOdom](https://github.com/superxslam/SuperOdom), [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2))
- [ ] Create a 3D world.sdf for LIVO-based navigation and mapping (i.e., textured and obstacle rich)
- [ ] Compare localization performance for a pre-defined navigation task at varying speeds
- [ ] Develop proposed approach (e.g., [DOOR-SLAM](https://github.com/MISTLab/DOOR-SLAM), [Swarm-SLAM](https://github.com/MISTLab/Swarm-SLAM))
- [ ] Use LIVO for PX4/ArduPilot SITL control (e.g., [PX4-Multiagent-Simulation](https://github.com/TannerGilbert/PX4-Multiagent-Simulation), [Ardupilot_Multiagent_Simulation](https://github.com/aau-cns/Ardupilot_Multiagent_Simulation))

### Feature: Gymnasium RL Environment and Examples

> SITL and perception-enabled reinforcement learning for real-world deployment

- [x] Wrap FTRT, headless, steppable simulation in `aas-gym`
- [ ] Optimize the environment `.reset()` time
- [ ] Conditional/AP mode startup to replace `GYM_INIT_DURATION`
- [ ] `offboard_control` references from external topics bridged by ZeroMQ
- [ ] ...

<!-- 

### Feature: Betaflight SITL

> Implement a C++ gz-transport/UDP bridge between Gazebo Sim and Betaflight SITL

- https://www.betaflight.com/docs/development/SITL
- https://github.com/Aeroloop/betaloop
- https://github.com/utiasDSL/gym-pybullet-drones/blob/a8c238c21c7586ee1735bafb358a4d5637402f14/gym_pybullet_drones/envs/BetaAviary.py#L111C1-L172C56

### More Out-there Ideas

> Potential for technical spikes/long-term, nice-to-have features

- Use ArduPilot ROS2 DDS interface instead of or alongside MAVROS
    - https://github.com/ArduPilot/ardupilot/tree/master/Tools/ros2#readme
    - https://ardupilot.org/dev/docs/ros2-sitl.html
- Add support for tail-sitter models
    - https://github.com/ArduPilot/ardupilot_gazebo/tree/main/models/zephyr
    - https://github.com/PX4/PX4-gazebo-models/tree/main/models/quadtailsitter
- Integrate a GIS world generator (e.g., Cesium)
    - https://github.com/CesiumGS/cesium-native
- Integrate a photorealistic simulator (e.g., IsaacSim)
    - https://github.com/PegasusSimulator/PegasusSimulator
- Integrate more realistic flight dynamics (e.g., JSBSim)
    - https://github.com/JSBSim-Team/jsbsim
- Integrate a VLA model bridging the `yolo_py` and `mission` packages
- Re-instate Gazebo Sim support for Pixhawk HITL simulation using MAVLink HIL_ interface
    - https://mavlink.io/en/messages/common.html
    - https://github.com/tiiuae/px4-gzsim-plugins/
    - https://docs.px4.io/main/en/simulation/hitl
    - https://ardupilot.org/dev/docs/hitl-simulators.html

-->

### Maintenance: Dependency Management

> PRs to update dependencies to their latest stable release are always welcome
>
> Also remember to update: [`check_requirements.sh`](/scripts/check_requirements.sh)

- [x] Host OS: [Ubuntu 22.04/24.04 (LTS, ESM 4/2034)](https://ubuntu.com/about/release-cycle)
- [x] Jetson OS: [L4T 36 (Ubuntu 22-based)/JetPack 6 (latest major release for Orin as of 2/2026)](https://developer.nvidia.com/embedded/jetpack-archive)
- [ ] [`nvidia-driver-580`](https://developer.nvidia.com/datacenter-driver-archive) -> **TEST ON 590**
- [x] [Docker Engine v29](https://docs.docker.com/engine/release-notes/)
- [x] [NVIDIA Container Toolkit 1.18](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/index.html)
- [ ] `amd64` base image: [`cuda:12.8.1-cudnn-runtime-ubuntu22.04`](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/cuda/tags) -> **UPDATE TO `cuda:13.1.1-cudnn-runtime-ubuntu22.04`**
- [x] `arm64`/Jetson base image: [`l4t-jetpack:r36.4.0`](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-jetpack/tags)
- [x] [ROS2 Humble (LTS, EOL 5/2027)](https://docs.ros.org/en/rolling/Releases.html)
- [x] [Gazebo Sim Harmonic (LTS, EOL 9/2028)](https://gazebosim.org/docs/latest/releases/)
- [x] [PX4 1.16.1](https://github.com/PX4/PX4-Autopilot/releases)
- [x] [ArduPilot 4.6.3](https://github.com/ArduPilot/ardupilot/releases)
- [x] [YOLO26](https://github.com/ultralytics/ultralytics/releases)
- [x] [ONNX Runtime 1.23.2](https://github.com/microsoft/onnxruntime/releases) (updating to 1.24 from wheel will require switching to Python 3.11)
