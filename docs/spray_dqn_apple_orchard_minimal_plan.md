# 基于 AAS 现有果园环境的撒农药无人机 DQN 精简方案

## 目标

尽量复用 `aerial-autonomy-stack` 已有的 `apple_orchard` 果园世界、任务执行链路和高层飞行接口，避免从零搭建复杂仿真。

整体思路是：DQN 负责二维果园抽象地图中的高层路径规划，AAS/Gazebo 负责三维果园飞行验证。

## 核心方案

第一阶段，先建立二维果园网格环境，将果树、障碍物、河流和危险区域抽象成地图约束。DQN 的动作空间为上、下、左、右，每移动到一个格子时，会对当前位置及上下左右邻近格子进行十字形喷洒。

第二阶段，训练完成后，将 DQN 生成的路径转换成 AAS mission YAML 中的 `reposition` 航点任务。这样可以复用 AAS 原本的飞行控制、任务执行和 `apple_orchard` Gazebo 场景。

最后，用覆盖率、重复喷洒率、路径长度、危险区进入次数和碰撞率评估方法效果，并与简单的蛇形扫描路径进行比较，验证 DQN 是否能在保证覆盖的同时减少危险区域进入和无效路径。

## 为什么这条路线工作量更小

- 直接复用 `simulation/simulation_resources/simulation_worlds/apple_orchard.sdf` 作为三维果园场景。
- 直接复用 `scripts/sim_run.sh` 启动仿真，只需把 `WORLD` 设成 `apple_orchard`。
- 直接复用 mission YAML 和 `mission_node.py` 的 `takeoff`、`reposition`、`speed`、`land` 流程。
- 喷洒逻辑先放在 Python 覆盖图里，不做真实液滴物理仿真。
- DQN 只做高层规划，无人机具体飞行仍由 AAS 原本的控制链完成。

## 建议复用的现有模块

- `simulation/simulation_resources/simulation_worlds/apple_orchard.sdf`
  用作果园 Gazebo 场景。
- `scripts/sim_run.sh`
  用 `WORLD=apple_orchard` 启动现有仿真。
- `aircraft/aircraft_resources/missions/*.yaml`
  复用现有任务 YAML 结构。
- `aircraft/aircraft_ws/src/mission/mission/mission_node.py`
  复用任务执行节点，不重写飞控逻辑。
- `/Drone1/set_reposition`
  把 DQN 生成的网格路径转换成航点后直接执行。

## 建议新增文件

- `experiments/spray_dqn/spray_grid_env.py`
  二维果园网格环境，包含喷洒、障碍、危险区、河流和覆盖率统计。
- `experiments/spray_dqn/train_dqn.py`
  DQN 训练入口。
- `experiments/spray_dqn/evaluate_dqn.py`
  评估脚本，输出覆盖率、碰撞率、危险区进入次数等指标。
- `experiments/spray_dqn/generate_spray_mission.py`
  把 DQN 路径转换成 AAS mission YAML。
- `experiments/spray_dqn/plot_coverage.py`
  绘制路径、十字喷洒覆盖图、危险区和河流区。
- `experiments/spray_dqn/configs/apple_orchard_grid.yaml`
  果园二维抽象地图配置。
- `aircraft/aircraft_resources/missions/spray_dqn.yaml`
  自动生成的喷洒航点任务。

## 关键命令

启动果园仿真：

```bash
AUTOPILOT=px4 NUM_QUADS=1 NUM_VTOLS=0 WORLD=apple_orchard HEADLESS=false CAMERA=false LIDAR=false GND_CONTAINER=false RTF=1.0 ./sim_run.sh
```

训练 DQN：

```bash
python experiments/spray_dqn/train_dqn.py
```

生成任务 YAML：

```bash
python experiments/spray_dqn/generate_spray_mission.py
```

执行喷洒任务：

```bash
docker exec -d aircraft-container-inst0_1 bash -c "source /opt/ros/humble/setup.bash && source /aas/github_ws/install/setup.bash && source /aas/aircraft_ws/install/setup.bash && ros2 run mission mission --conops spray_dqn.yaml --ros-args -r __ns:=/Drone1 -p use_sim_time:=true"
```

## 最小改动范围

- 优先不改飞控、不改 PX4、不改 Gazebo 插件。
- 优先新增 `experiments/spray_dqn/` 下的训练和可视化代码。
- `gym_run.py` 和 `aas_env.py` 最多只做小改，让 `world` 可配置，不再写死。
- 喷洒动作第一版只在 Python 覆盖图中体现，不接入 ROS2 真实喷洒话题。

## 最终成果

- 基于 `apple_orchard` 的撒药无人机路径规划实验。
- DQN 训练曲线和评价指标：覆盖率、路径长度、危险区进入次数、重复喷洒率。
- 喷洒覆盖图和避障路径图。
- 可在 AAS/Gazebo 果园场景中执行的 `spray_dqn.yaml` 航点任务。
- 一份说明文档，清楚描述“DQN 高层规划 + AAS 三维验证”的实验流程。
