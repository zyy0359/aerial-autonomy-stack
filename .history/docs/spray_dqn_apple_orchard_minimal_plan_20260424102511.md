# 基于 AAS 现有果园环境的撒农药无人机 DQN 精简方案

## 目标

尽量复用 `aerial-autonomy-stack` 已有的 `apple_orchard` 果园世界、任务执行链路和高层飞行接口，避免从零搭建复杂仿真环境。

整体思路是：DQN 负责二维果园抽象地图中的高层路径规划，AAS/Gazebo 负责三维果园场景中的飞行执行与可视化验证。

## 核心方案

### 1. 总体设计思路

本方案不直接在三维 Gazebo 环境中从零训练无人机控制策略，而是采用“二维抽象规划 + 三维场景验证”的两阶段结构，以减少开发工作量并提高实验稳定性。

第一阶段，在 Python 中建立一个二维果园网格环境，将现实中的果树、障碍物、河流和危险区域抽象成地图约束。无人机在该网格中按照离散动作移动，动作空间定义为 `上、下、左、右`。每当无人机从一个格子移动到下一个格子后，会自动对当前位置以及其上、下、左、右四个相邻格子执行一次十字形喷洒，从而模拟喷头在实际飞行中的横向覆盖效果。

第二阶段，在 DQN 完成训练后，不直接把网络输出作为低层飞控命令，而是将其生成的离散路径转换成 AAS 中 mission YAML 文件里的 `reposition` 航点任务。这样可以复用 AAS 原有的飞行控制、任务执行、PX4/ArduPilot 接口以及 `apple_orchard` Gazebo 场景，只需要新增路径生成和覆盖率统计模块，而不必改动底层飞控与 Gazebo 插件。

最后，通过覆盖率、重复喷洒率、路径长度、危险区进入次数和碰撞率等指标评估方法效果，并与简单的蛇形扫描路径进行比较，验证 DQN 是否能够在保证喷洒覆盖的同时减少危险区域进入和无效路径。

### 2. 第一阶段：二维果园网格环境构建

第一阶段的重点是建立一个适合强化学习训练的轻量级环境。该环境不追求真实气动与液滴扩散物理，而是突出“喷洒任务规划”本身，即在有限时间和有限喷洒资源下，尽量覆盖更多目标区域，同时避免进入危险区和障碍区。

建议将果园抽象为一个二维矩阵，例如 `20 x 20` 或 `30 x 30` 的网格。每个格子代表现实果园中的一个固定面积区域，例如 `5m x 5m`。这样一来，DQN 输出的路径既能在二维中训练，又能较方便地映射到 AAS 中的 `east/north` 航点坐标。

在网格环境中，建议至少定义以下几类区域：

- 可喷洒区域：表示果树所在区域或需要覆盖的目标区域。
- 已喷洒区域：表示已经完成喷洒覆盖的格子。
- 障碍区域：表示果树干、围栏、设施、不可通过的固定障碍物。
- 河流区域：表示严格禁止进入的区域，一旦进入可视为碰撞或严重违规。
- 危险区域：表示允许经过但代价很高的区域，例如居民区、水源保护区、禁喷区。
- 起飞点或任务起点：表示无人机开始执行任务的位置。
- 可选返航点：用于后续扩展任务完成后回到指定位置。

### 3. 状态、动作与喷洒规则设计

在二维环境中，DQN 的动作空间设为四个离散动作：

- `0`: 上
- `1`: 下
- `2`: 左
- `3`: 右

每执行一次动作后，无人机从当前格子移动到新格子，然后立即触发一次十字形喷洒。十字形喷洒的覆盖范围建议定义为：

- 当前格子
- 当前格子的上邻格
- 当前格子的下邻格
- 当前格子的左邻格
- 当前格子的右邻格

如果这些格子中存在未喷洒的目标区域，则将其标记为“已喷洒”，并为策略提供正奖励；如果这些格子早已喷洒过，则视为重复喷洒，可给予较小惩罚。

状态表示可以采用两种方式：

- 方式一：全局网格矩阵作为输入，编码当前无人机位置、已喷洒区域、障碍、河流和危险区域。
- 方式二：在全局矩阵基础上，额外加入当前剩余未覆盖比例、累计步数、喷洒次数等辅助变量。

第一版建议先采用方式一，以降低实现复杂度。后续若效果不理想，再加入局部视野或辅助变量。

### 4. 奖励函数设计

奖励函数是整个实验是否有效的关键。建议将奖励函数拆解成多个可解释的组成部分，而不是只使用一个简单总分。一个较合理的设计如下：

- 新覆盖奖励：每喷到一个新的目标格子时，给予正奖励。
- 重复喷洒惩罚：对已经喷过的格子重复喷洒时，给予小幅负奖励。
- 步长惩罚：每走一步给予一个很小的负奖励，鼓励更短路径。
- 障碍碰撞惩罚：撞到障碍区时给予较大的负奖励，并可终止当前回合。
- 河流进入惩罚：进入河流区时给予更大的负奖励，并终止当前回合。
- 危险区进入惩罚：进入危险区时给予中到大的负奖励，但不一定立即终止回合。
- 完成奖励：当覆盖率达到预设阈值，例如 `90%` 或 `95%` 时，给予额外奖励。
- 可选返航奖励：若任务完成后回到返航点，再给予一部分奖励。

这样设计的目的是让 DQN 学会以下行为：

- 优先覆盖未喷洒区域。
- 尽量减少来回绕圈。
- 主动避开河流与障碍。
- 尽可能少进入高代价危险区。

### 5. 训练流程设计

训练时建议先从简单地图开始，再逐步提高难度。一个推荐流程如下：

1. 先使用一个较小的固定地图，例如 `10 x 10`，只有少量障碍物，不含危险区和河流。
2. 在该地图上验证环境逻辑、奖励函数和十字喷洒逻辑是否正确。
3. 然后引入河流区域，让策略学习绕行。
4. 再引入危险区，让策略学习“可通过但代价高”的避障行为。
5. 最后将地图扩大到更接近真实果园布局的 `20 x 20` 或 `30 x 30`，并加入更复杂的约束。

训练脚本中建议记录以下信息：

- 每个 episode 的总奖励
- 每个 episode 的覆盖率
- 每个 episode 的危险区进入次数
- 每个 episode 的碰撞次数
- 每个 episode 的路径长度

这些数据应保存为日志文件，并在训练后绘制曲线图，例如：

- reward 随 episode 变化曲线
- 覆盖率随 episode 变化曲线
- 碰撞率随 episode 变化曲线

这样可以直观看到 DQN 是否真正学到了有效路径。

### 6. 第二阶段：路径转换为 AAS mission YAML

第二阶段的重点不是重新训练一个三维飞行策略，而是将第一阶段 DQN 学到的二维路径转成 AAS 可执行的高层任务。

AAS 现有任务系统已经支持 mission YAML，并由 `mission_node.py` 依次执行 `takeoff`、`reposition`、`speed`、`land` 等动作。因此可以沿用这套结构，将 DQN 输出的网格路径转换为一串 `reposition` 指令。

建议转换步骤如下：

1. 设定二维网格与 Gazebo 世界坐标之间的映射关系。
   例如每个网格对应现实中的 `5m x 5m`，并选择果园某个角点作为 `(east=0, north=0)` 参考点。
2. 将 DQN 输出的网格序列，例如 `[(0,0), (0,1), (1,1), ...]`，转换成对应的 `(east, north, altitude)` 航点序列。
3. 在航点序列前加入 `takeoff` 动作，在航点序列后加入 `land` 动作。
4. 将所有航点写入 `aircraft/aircraft_resources/missions/spray_dqn.yaml`。
5. 使用现有 mission 节点运行该任务，让无人机在 `apple_orchard` 场景中飞行。

这样做的最大优点是：

- 不用自己写 PX4 控制器。
- 不用自己处理姿态、速度和轨迹跟踪。
- 可以直接看到无人机在果园三维场景里沿规划路径飞行。

### 7. 三维验证步骤

在 AAS/Gazebo 中进行验证时，建议按如下顺序操作：

1. 启动果园场景：

```bash
AUTOPILOT=px4 NUM_QUADS=1 NUM_VTOLS=0 WORLD=apple_orchard HEADLESS=false CAMERA=false LIDAR=false GND_CONTAINER=false RTF=1.0 ./sim_run.sh
```

2. 在 Python 中加载已训练的 DQN 模型，生成喷洒路径。
3. 调用路径转换脚本，把路径写入 `spray_dqn.yaml`。
4. 使用 mission 节点执行该 YAML：

```bash
docker exec -d aircraft-container-inst0_1 bash -c "source /opt/ros/humble/setup.bash && source /aas/github_ws/install/setup.bash && source /aas/aircraft_ws/install/setup.bash && ros2 run mission mission --conops spray_dqn.yaml --ros-args -r __ns:=/Drone1 -p use_sim_time:=true"
```

5. 在 Gazebo 场景中观察无人机是否沿预定路径经过果园区域。
6. 将飞行路径与二维覆盖图对应起来，验证“训练中的规划路径”与“三维场景中的执行路径”是否一致。

需要注意的是，第一版验证重点是“规划路径可执行”，而不是“真实药液喷洒建模”。因此覆盖统计仍建议在 Python 中基于路径回放完成。

### 8. 对比实验设计

为了证明 DQN 的有效性，建议不要只展示 DQN 单独结果，而是设计一个基准方法进行对比。最简单可行的基线就是蛇形扫描路径。

蛇形路径可定义为：

- 第 1 行从左到右扫描
- 第 2 行从右到左扫描
- 第 3 行再从左到右扫描
- 依此类推

在有障碍、河流和危险区域的情况下，蛇形路径通常会：

- 出现大量无效绕行
- 更容易进入危险区
- 覆盖效率下降
- 重复喷洒增加

因此建议在同一张地图上，分别运行：

- DQN 规划路径
- 蛇形扫描路径
- 可选随机路径

并比较以下指标：

- 覆盖率：最终喷洒覆盖的目标区域比例
- 重复喷洒率：已喷洒区域再次被喷的比例
- 路径长度：总移动步数或总航迹长度
- 危险区进入次数：进入危险区的累计次数
- 碰撞率：进入障碍或河流区域的次数

如果 DQN 能在接近或超过蛇形扫描覆盖率的情况下，显著降低危险区进入次数和无效路径长度，就说明其规划策略是有效的。

### 9. 建议的实验输出

为了便于后续写报告、论文或答辩展示，建议每次实验至少保留以下结果：

- 训练曲线图：
  - reward 曲线
  - 覆盖率曲线
  - 碰撞率曲线
- 路径图：
  - DQN 路径
  - 蛇形路径
- 覆盖图：
  - 已喷洒区域分布
  - 重复喷洒热力图
- 指标表：
  - 覆盖率
  - 重复喷洒率
  - 路径长度
  - 危险区进入次数
  - 碰撞率
- Gazebo 验证截图或视频：
  - 无人机在 `apple_orchard` 中执行 `spray_dqn.yaml` 的飞行过程

这些输出可以帮助你把实验分为“算法结果”和“仿真验证结果”两部分，结构会比较完整。

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

## 14 天精简计划

| 天数 | 每天 2 小时具体做什么 | 当天要做出的内容 |
|---|---|---|
| Day 1 | 启动 `apple_orchard` 世界，确认能看到果园和无人机。 | 截图一张，记录启动命令。 |
| Day 2 | 新建 `experiments/spray_dqn/`，写果园二维网格配置。 | `apple_orchard_grid.yaml`。 |
| Day 3 | 实现十字形喷洒规则和覆盖率统计。 | 能画喷洒覆盖图。 |
| Day 4 | 加入障碍区、危险区、河流区，用矩形或多边形手动定义。 | 地图图像：树行、河流、危险区、可喷区域。 |
| Day 5 | 写 DQN 环境，动作先用上、下、左、右。 | `SprayGridEnv` 可运行。 |
| Day 6 | 写随机策略 baseline。 | 随机路径图和覆盖率。 |
| Day 7 | 写 DQN 训练脚本。 | 能训练并保存模型。 |
| Day 8 | 训练简单果园地图。 | reward 曲线、覆盖率曲线。 |
| Day 9 | 调 reward，让策略少撞危险区、少重复喷。 | 比随机策略更好的结果。 |
| Day 10 | 把 DQN 路径转成 AAS `reposition` YAML。 | `spray_dqn.yaml`。 |
| Day 11 | 在 `apple_orchard` 中运行生成的航点任务。 | 无人机按路径飞的视频或截图。 |
| Day 12 | 画最终喷洒覆盖图和避障路径图。 | `coverage_final.png`、`path_final.png`。 |
| Day 13 | 整理实验指标：覆盖率、危险区进入次数、重复喷洒率、路径长度。 | 结果表格和对比图。 |
| Day 14 | 写 README 或报告，说明现有 AAS 果园场景复用方式和 DQN 结果。 | 最终说明文档。 |

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
