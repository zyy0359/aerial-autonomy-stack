# Rationale

This project aims to speed up the ideation-development-simulation-deployment cycle of PX4/ArduPilot-based applications.

## The Many Facets of the Sim2real Gap

The *sim2real gap* is an euphemism for robotic projects that work well on a developer's laptop but not so much in the field.
Aerial sim2real research often focuses on modeling and simulation of complex aerodynamics effects.

Nonetheless, at deployment time, an equally important component of *sim2real gap* arises from [system design](https://arxiv.org/abs/2510.20808)—in particular, software tooling and engineering, where the dynamic range between ["average and the best is 50-to-1, maybe 100-to-1"](https://www.youtube.com/watch?v=wTgQ2PBiz-g&t=35s).

This is the challenge—and good sport—of **vertical/full-stack integration** among:

- the **many frameworks** that go into drone autonomy (a physics engine to simulate drone dynamics, a rendering engine to generate realistic imagery, a GPU-accelerated machine learning runtime for perception, one or more inter-process and inter-thread communication middleware, the interface to the microcontroller and autopilot software performing state-estimation and low-level control, the SDKs of the deployed embedded systems, etc.)
- emulated **inter-robot communication** (in aerial systems, this is heavily affected by the actual flight plans and available RF hardware)

## Presentations

- [Aerial Robotics Meeting - January 22nd 2026 [slides]](https://docs.google.com/presentation/d/1Sz0d7WPWNwgCM3Q49Nnq1mvkSwPyK_CCQ7viHxO_Ucs)

## Related Work

A summary of existing multi-drone flight stacks can be found in [Table II of this paper](https://arxiv.org/pdf/2303.18237). Notable ones are:

- *Universidad Politécnica de Madrid (UPM)*'s [`aerostack2`](https://github.com/aerostack2/aerostack2) (multicopter-only)
- *Czech Technical University in Prague (CTU)*'s [`mrs_uav_system`](https://github.com/ctu-mrs/mrs_uav_system) (multicopter-only)
- *Technische Universität (TU) Berlin*'s [`crazyswarm2`](https://github.com/IMRCLab/crazyswarm2) (indoor, crazyflie-only)
- *Peking University*'s [`XTDrone`](https://github.com/robin-shaun/XTDrone) (PX4-only)

A summary of aerial robotics simulators can be found in [Table IV of this paper](https://arxiv.org/pdf/2311.02296), these include:

- *Norwegian University of Science and Technology (NTNU)*'s [`aerial_gym_simulator`](https://github.com/ntnu-arl/aerial_gym_simulator) (high-performance simulator for RL)
- *University of Pennsylvania (UPenn)*'s [`RotorPy`](https://github.com/spencerfolk/rotorpy) (high-fidelity simulator for control)
- *University of Toronto (UofT)*'s [`gym-pybullet-drones`](https://github.com/utiasDSL/gym-pybullet-drones) (simple simulator for education, control, and RL)
- *UZH*'s [`flightmare`](https://github.com/uzh-rpg/flightmare), *ETH*'s [`RotorS`](https://github.com/ethz-asl/rotors_simulator), *NYU*'s [`RotorTM`](https://github.com/arplaboratory/RotorTM), *Microsoft*'s [`AirSim`](https://github.com/microsoft/AirSim), etc.

For even more resources, check out [`aerial_robotic_landscape`](https://github.com/ROS-Aerial/aerial_robotic_landscape).

## Design Manifesto

- **Simplicity** (["simple is better than complex"](https://peps.python.org/pep-0020/), ["worse is better"](https://www.dreamsongs.com/RiseOfWorseIsBetter.html), and ["no fat software"](https://people.inf.ethz.ch/wirth/Articles/LeanSoftware.pdf) are the 3 slogans of the AAS)
- [おまかせ](https://dhh.dk/2012/rails-is-omakase.html) **end-to-end**ness (from camera frames, to autopilot uORB/MAVLink commands)
- **Recentness** (break and fix, rather than carrying technical [debt](https://c2.com/doc/oopsla92.html))
- **Deployment** focus
    - Clear, Dockerized split between aircraft, ground, and simulation software
    - ROS2 intra-companion board messaging
    - XRCE-DDS (PX4), MAVROS (ArduPilot) autopilot-to-companion board ROS2 bridge
    - GStreamer camera-to-companion board acquisition pipelines
    - Zenoh inter-vehicle ROS2 bridge, with networking over LAN (HITL) or emulated by `docker network` (SITL)
    - Dual network—in both SITL and HITL—to separate synthetic sensor data from inter-vehicle communication

## Roadmap

### Feature: LiDAR-inertial Odometry and SLAM

> Advanced localization and mapping baselines and capabilities

- [x] [KISS-ICP](https://github.com/PRBonn/kiss-icp) LiDAR odometry baseline
- [ ] Add  LIO baselines (e.g., [SuperOdom](https://github.com/superxslam/SuperOdom), [SPARK-FAST-LIO](https://github.com/MIT-SPARK/spark-fast-lio))
- [ ] (optional) visual-inertial baseline (e.g., [open_vins](https://github.com/rpng/open_vins), [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono))
- [ ] Create an indoor (maze-like) 3D world.sdf for LIO-based navigation and mapping
- [ ] Create an outdoor 3D world.sdf for LIO-based navigation and mapping
- [ ] Refine the sensor model and placement of `sensor_lidar`
- [ ] Compare localization performance for a pre-defined navigation task at varying speeds
- [ ] Develop proposed approach
- [ ] Fuse offboard state estimation into PX4/ArduPilot onboard state estimation
- [ ] Add GPSless SITL simulation option

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

- [x] Host OS: [Ubuntu 22.04/24.04 (LTS, ESM 4/2034)](https://ubuntu.com/about/release-cycle)
- [x] Jetson OS: [L4T 36 (Ubuntu 22-based)/JetPack 6 (latest major release for Orin as of 2/2026)](https://developer.nvidia.com/embedded/jetpack-archive)
- [ ] [`nvidia-driver-580`](https://developer.nvidia.com/datacenter-driver-archive) -> **TEST ON 590**
- [x] [Docker Engine v29](https://docs.docker.com/engine/release-notes/)
- [x] [NVIDIA Container Toolkit 1.18](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/index.html)
- [ ] `amd64` base image: [`cuda:12.8.1-cudnn-runtime-ubuntu22.04`](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/cuda/tags) -> **UPDATE TO `cuda:13.1.1-cudnn-runtime-ubuntu22.04`**
- [x] `arm64`/Jetson base image: [`l4t-jetpack:r36.4.0`](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-jetpack/tags)
- [x] [ROS2 Humble (LTS, EOL 5/2027)](https://docs.ros.org/en/rolling/Releases.html)
- [x] [Gazebo Sim Harmonic (LTS, EOL 9/2028)](https://gazebosim.org/docs/latest/releases/)
- [ ] [PX4 1.16.0](https://github.com/PX4/PX4-Autopilot/releases) -> **UPDATE TO 1.16.1**
- [ ] [ArduPilot 4.6.2](https://github.com/ArduPilot/ardupilot/releases) -> **UPDATE TO 4.6.3**
- [x] [YOLOv8](https://github.com/ultralytics/ultralytics/releases)
- [ ] [ONNX Runtime 1.22.1](https://onnxruntime.ai/getting-started) -> **UPDATE TO 1.23.2**
