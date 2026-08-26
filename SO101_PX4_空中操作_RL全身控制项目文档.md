# SO101 简化刚性臂 + 欠驱动四旋翼空中操作 RL 全身控制项目文档

> 用途：给后续 Codex / Claude Code / 自己实施项目时作为“项目总规范”。  
> 当前日期：2026-07-23  
> 当前主硬件：PX4 四旋翼 + Jetson Xavier NX + Ubuntu 20.04 + ROS Noetic + MAVROS + D435I + 简化 SO101 刚性臂。  
> 主参考论文：`thesis_wholebody_control_of_aerial_manipulator_with_RL_Shlok_5928516.pdf`。  
> 当前研究取向：暂不以柔性/连续体臂作为主线，转向“无人机 + 简化刚性臂 + 学习型全身协调控制”。

---

## 0. 一句话结论

本项目不要从“端到端 RL 直接控制 PX4 电机和 SO101 舵机”开始，而应采用：

```text
RL policy 作为外环/协调器
  -> 输出无人机高层 setpoint + 简化臂关节目标/增量
  -> PX4/MAVROS 跟踪无人机 setpoint
  -> SO101 舵机内部 PID 或 ROS/Python PID 跟踪关节目标
  -> safety supervisor 对所有命令限幅、限速、急停
```

这与硕士论文的核心思路一致：论文中的 PPO policy 并不直接输出电机 PWM，而是输出四旋翼加速度、body rate、yaw reference 和机械臂关节目标；底层由 acceleration controller、INDI 和 arm PID 完成执行。

对你的硬件架构，应改成：

```text
PPO / SAC / imitation policy
  -> drone velocity / position offset / attitude-rate setpoint
  -> arm joint target / joint delta

PX4
  -> 姿态、速度、位置稳定

SO101 bridge
  -> active joints 映射
  -> locked joints 保持固定
  -> joint limits / velocity limits / smooth filter
```

---

## 1. 研究目标与边界

### 1.1 目标

建立一套可复现、可逐步真机迁移的空中操作系统：

```text
欠驱动四旋翼无人机
  + 简化 SO101 3-4DOF 刚性臂
  + D435I 视觉/深度感知
  + Jetson Xavier NX 上位机
  + PX4/MAVROS 传统飞控
  + SO101 传统关节控制
  + RL 全身协调策略
```

第一阶段只追求：

```text
无人机稳定悬停或仿真悬停
简化臂末端到达目标点/目标姿态附近
不要求抓取、不要求接触、不要求图像端到端
```

中后期再扩展到：

```text
抓取
推压
路径跟踪
接触力控制
视觉目标跟踪
电力巡检/电缆附近操作
```

### 1.2 当前必须坚持的边界

```text
1. SO101 首先是机器人学习/算法平台，不默认是最终可飞硬件。
2. PX4 负责飞行安全，RL 不直接接管飞控底层。
3. SO101 使用传统关节控制保底，RL 只输出关节目标或增量。
4. D435I 第一版只做目标位姿估计，不直接把图像喂给 RL。
5. 真机无人机不做在线 RL，只部署离线训练后的 policy。
6. 没有推重比、重心、供电、限位、安全绳测试前，不挂臂飞行。
```

---

## 2. 主参考论文精读结论

### 2.1 论文信息

本地文件：

```text
E:/Desktop/刚柔耦合连续体机械臂/无人机/资料/thesis_wholebody_control_of_aerial_manipulator_with_RL_Shlok_5928516.pdf
```

题目：

```text
Whole-body Control of an Aerial Manipulator with Reinforcement Learning
```

作者与学校：

```text
Shlok Deshmukh
TU Delft
Master Thesis
2025
```

平台：

```text
Osprey aerial manipulator
欠驱动四旋翼 + 2DOF 刚性机械臂 + gripper
```

算法：

```text
PPO
Actor-Critic
MLP [512, 256, 128]
ELU activation
4096 parallel environments
Isaac Lab training
Gazebo/Agilicious deployment simulator
real-world validation
```

### 2.2 论文最重要的工程结论

论文真正值得复现的不是某个网络结构，而是下面这套工程思想：

```text
1. policy 是外环，不是底层电机控制器。
2. action 采用高层控制抽象，而不是 raw motor command。
3. observation 要显式包含关键状态，否则问题会变成 POMDP。
4. reward 必须包含动作平滑项，否则真机容易振荡。
5. domain randomization 必须覆盖负载、摩擦、控制器/执行器差异。
6. 训练仿真和部署仿真要分开验证。
7. 真机部署前要对齐控制器频率、PID、延迟、摩擦和 actuator response。
```

### 2.3 论文 action space

论文 action 是 9 维：

```text
a_t = [
  W a_b,      # 3D quadrotor linear acceleration in world frame
  B Omega_b, # 3D body angular velocity in body frame
  W psi_r,   # 1D yaw reference in world frame
  theta_r    # 2D arm joint angle references
]
```

也就是：

```text
无人机：加速度 + body rate + yaw reference
机械臂：2 个关节目标角
```

对你的系统，第一版不建议照搬到 PX4 真机。建议从更安全接口开始：

```text
Level 1:
  action = [
    drone position offset xyz,
    arm active joint delta,
    gripper command
  ]

Level 2:
  action = [
    drone velocity setpoint xyz,
    yaw rate or yaw setpoint,
    arm joint target / delta,
    gripper command
  ]

Level 3:
  action = [
    drone acceleration setpoint xyz,
    body rate xyz,
    yaw reference,
    arm joint targets
  ]
```

建议顺序：

```text
先 Level 1/2
后 Level 3
不要第一版 raw motor RL
```

### 2.4 论文 observation space

论文 observation 是 29 维：

```text
o_t = [
  B v_b,                 # 3D body-frame linear velocity
  B Omega_b,             # 3D body-frame angular velocity
  W R_b,                 # 9D rotation matrix
  theta,                 # 2D arm joint angles
  B x_goal,              # 3D goal position in drone body frame
  EE x_goal,             # 3D goal position in end-effector frame
  EE R_goal[:, 0:2]      # 6D continuous rotation representation
]
```

论文设计很聪明：目标不直接使用世界坐标，而是转换到无人机机体系和末端坐标系。这样 policy 更容易泛化。

你的第一版 observation 建议：

```text
drone:
  position error to hover point, 3
  velocity, 3
  attitude representation, 4 or 6 or 9
  angular velocity, 3

arm:
  active joint positions, n
  active joint velocities, n
  locked joint values, optional
  gripper state, 1

task:
  end-effector pose relative to drone body
  target pose relative to drone body
  target pose relative to end-effector

history:
  previous action
```

最小可训练版：

```text
obs = [
  drone_pos_error_body,
  drone_vel_body,
  drone_attitude,
  drone_angular_vel_body,
  active_q,
  active_qdot,
  ee_pos_body,
  target_pos_body,
  ee_pos_error_body,
  last_action
]
```

### 2.5 论文 reward

论文 reward 由五类组成：

```text
R_pos   # 末端位置奖励
R_ori   # 末端姿态奖励
R_ds    # 无人机动作平滑奖励
R_js    # 机械臂动作平滑奖励
R_dmag  # 无人机动作幅值奖励
```

形式上使用负指数：

```text
R = w * exp(-alpha * error)
```

你的第一版 reward：

```text
reward =
  + end_effector_distance_reward
  + progress_reward
  + success_bonus
  - drone_position_error_penalty
  - drone_attitude_penalty
  - arm_joint_limit_penalty
  - arm_velocity_penalty
  - action_magnitude_penalty
  - action_delta_penalty
  - collision_or_failsafe_penalty
```

推荐从下面的简单形式开始：

```python
reward = - w_ee * norm(ee_pos - target_pos)
reward += w_progress * (prev_err - err)
reward -= w_drone * norm(drone_pos - hover_pos)
reward -= w_att * attitude_tilt_penalty
reward -= w_act * norm(action)
reward -= w_smooth * norm(action - last_action)

if err < success_threshold and stable_for_N_steps:
    reward += success_bonus

if unsafe:
    reward -= fail_penalty
    done = True
```

### 2.6 论文 domain randomization

论文随机化：

```text
end-effector payload mass
joint friction
joint stiffness / actuator behavior
```

对你的系统应随机化：

```text
无人机：
  mass
  inertia
  thrust scale
  motor delay
  battery voltage / thrust degradation
  wind disturbance
  state estimation noise
  action delay

机械臂：
  joint friction
  joint backlash
  servo delay
  joint stiffness
  payload mass
  active joint limits
  locked joint small compliance

挂载：
  arm base transform
  center-of-mass offset
  cable/线束扰动
```

### 2.7 论文实验结果

论文结果可作为目标上限，而不是第一版指标：

```text
真实末端 6D pose control:
  平均位置误差约 5.3 cm
  平均姿态误差约 8.8 deg
  推理时间约 0.18 ms

真实负载：
  50 g 和 140 g payload 均验证

仿真：
  250 个随机目标位姿，成功率 100%
  平均位置误差约 5.9 cm
  平均姿态误差约 7.5 deg
```

对你的第一版验收目标应更保守：

```text
仿真中：
  末端位置误差 < 10 cm
  hover 位置误差不发散
  action 不高频抖动
  关节不越界

真机低空中：
  只做慢速小范围动作
  先不要求抓取
  先不要求姿态 6D 精确控制
```

### 2.8 论文暴露出的坑

论文附录中最值得你记住的坑：

```text
1. Isaac Lab 和 Gazebo 的 PID D 项实现不同，导致 arm 高频振荡。
2. 仿真与真实摩擦模型不同，导致 arm 低频振荡。
3. training sim 与 deployment sim 的 rotor thrust update 顺序不同，导致 quadrotor roll 振荡。
4. policy 使用接近完美的状态估计，真实场景中 VIO/SLAM/GPS/视觉会有噪声、延迟、漂移。
```

所以你的复现流程必须包含：

```text
训练仿真
  -> 部署仿真
  -> PX4 SITL
  -> 桌面机械臂
  -> 假臂飞行
  -> 真臂台架
  -> 低空慢速真机
```

不能直接：

```text
Isaac Lab 训练
  -> 真机挂臂飞
```

---

## 3. 与用户给定论文/链接的关系

### 3.1 TU Delft 硕士论文

用途：

```text
本项目主复现对象。
重点复现其控制层级、action/observation/reward、domain randomization、部署流程。
```

可迁移：

```text
PPO 外环
低层飞控/关节 PID 内环
动作平滑 reward
仿真到真机的二级验证流程
```

不可直接照搬：

```text
INDI 控制器细节
2DOF differential arm 机构
VICON 状态估计条件
Osprey 平台质量/惯量/推力参数
```

### 3.2 arXiv:2512.21085

题目：

```text
Global End-Effector Pose Control of an Underactuated Aerial Manipulator via Reinforcement Learning
```

用途：

```text
硕士论文的论文版/精炼版。
可作为后续写文献综述和方法对比时的主引用。
```

重点：

```text
PPO policy 输出 quadrotor acceleration、body rates 和 joint angle targets。
底层由 INDI 和 joint PID 跟踪。
```

链接：

```text
https://arxiv.org/abs/2512.21085
```

### 3.3 arXiv:2605.14805

题目：

```text
Learning Cross-Coupled and Regime Dependent Dynamics for Aerial Manipulation
```

用途：

```text
后续升级方向：学习 residual dynamics，用于补偿无人机-机械臂耦合、负载变化、机械臂重构带来的时变动力学。
```

适合你的第二阶段研究：

```text
传统控制/RL policy 之外增加 learned residual dynamics model
  -> MPC 或 tracking controller
  -> 对 payload / arm configuration / transient dynamics 做在线适应
```

链接：

```text
https://arxiv.org/abs/2605.14805
```

### 3.4 arXiv:2606.16621

题目：

```text
Reinforcement Learning with Inner-loop Dynamics Estimator for Aerial Manipulation under Uncertainty
```

用途：

```text
后续升级方向：RL 外环 + dynamics estimator 内环。
这与“PX4/MAVROS + SO101 PID + RL 外环”的思路很接近。
```

可借鉴：

```text
RL 外环负责将 6D end-effector target 映射为 whole-body command。
内环 dynamics estimator 负责补偿惯性变化、payload、未知扰动。
```

链接：

```text
https://arxiv.org/abs/2606.16621
```

### 3.5 arXiv:2407.00889

题目：

```text
Non-Prehensile Aerial Manipulation using Model-Based Deep Reinforcement Learning
```

用途：

```text
后期接触/推物体任务参考。
第一阶段不必复现。
```

适合扩展：

```text
无人机推动目标
未知摩擦下物体移动
连续接触保持
Dreamer/world-model 类方法
```

链接：

```text
https://arxiv.org/abs/2407.00889
```

### 3.6 IEEE Xplore 11267184

链接：

```text
https://ieeexplore.ieee.org/document/11267184
```

当前状态：

```text
网页端需要 JavaScript / 人机验证，当前无法可靠提取题名、摘要和方法。
后续请上传 PDF 或 BibTeX，再补入本文件。
```

执行要求：

```text
在未读到 PDF 前，不要在论文综述中引用它的具体结论。
```

---

## 4. 开源资料清单

### 4.1 SO101 / LeRobot

#### TheRobotStudio/SO-ARM100

链接：

```text
https://github.com/TheRobotStudio/SO-ARM100
```

用途：

```text
SO-100 / SO-101 开源硬件资料主仓库。
包含 STL、STEP、Simulation、BOM、装配指南、可选相机架和夹爪改件。
```

对本项目的作用：

```text
1. 获取 SO101 机械结构尺寸。
2. 建立简化 URDF/MJCF/USD。
3. 判断哪些自由度可锁定。
4. 设计轻量化改装或自研 2-3DOF 版本。
```

注意：

```text
SO101 原始定位是低成本桌面机器人学习机械臂，不是空中机械臂。
挂载到四旋翼前必须重做重量、重心、惯量、供电和强度验证。
```

#### LeRobot

链接：

```text
https://github.com/huggingface/lerobot
https://huggingface.co/docs/lerobot/index
```

用途：

```text
机器人学习工具箱。
覆盖 teleoperation、dataset recording、imitation learning、HIL-SERL、policy training、policy deployment。
```

关键命令类别：

```text
lerobot-find-port
lerobot-setup-motors
lerobot-calibrate
lerobot-teleoperate
lerobot-record
lerobot-train
lerobot-eval
lerobot-replay
```

SO101 典型 teleoperation 命令形式：

```bash
lerobot-teleoperate \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyUSB0 \
  --robot.id=my_follower_arm \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyUSB1 \
  --teleop.id=my_leader_arm
```

Python 控制思路：

```python
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

robot = SO101Follower(
    SO101FollowerConfig(
        port="/dev/ttyUSB0",
        id="aerial_so101_follower",
    )
)

robot.connect()
obs = robot.get_observation()
robot.send_action(action)
```

实际代码必须以当前 LeRobot 版本为准。若 import path 改动，应先用：

```bash
python -c "import lerobot, pkgutil; print(lerobot.__file__)"
python -m pip show lerobot
```

检查当前 API。

#### LeRobot HIL-SERL

链接：

```text
https://github.com/huggingface/lerobot/blob/main/docs/source/hilserl.mdx
```

用途：

```text
人类在环强化学习。
适合桌面 SO101 任务，不适合第一版无人机真机在线 RL。
```

建议使用顺序：

```text
SO101 桌面 reach/push/pick
  -> teleop 数据采集
  -> imitation policy
  -> HIL-SERL 微调
  -> 再考虑接入无人机移动基座
```

#### SO101 ROS2 / MoveIt2 参考栈

链接：

```text
https://github.com/legalaspro/so101-ros-physical-ai
https://so101-ros2.readthedocs.io/latest/
https://github.com/ycheng517/lerobot-ros
```

用途：

```text
参考 SO101 与 ROS2、MoveIt2、LeRobot policy inference、episode recording、URDF/mesh/robot_state_publisher 的连接方式。
```

对你的项目的定位：

```text
当前 Jetson 计划是 Ubuntu 20.04 + ROS Noetic + MAVROS。
因此这些 ROS2 项目不直接作为第一版运行依赖，而是作为接口设计、URDF、关节状态发布、policy runner 的参考。
```

建议迁移方式：

```text
先写 ROS1 so101_ros1_bridge，保持与 MAVROS 同一 ROS master。
后续如果整体迁移 ROS2，再参考 so101-ros-physical-ai / lerobot-ros 做 ROS2 bridge、MoveIt2 和远程 policy server。
```

### 4.2 PX4 / MAVROS

#### PX4 Offboard Mode

链接：

```text
https://docs.px4.io/v1.14/en/flight_modes/offboard
```

核心要求：

```text
PX4 Offboard 需要外部控制器持续发送 proof-of-life / setpoint。
官方最低要求 > 2 Hz。
工程上建议 20 Hz 或更高，并监控 timeout。
```

可接收命令类型：

```text
position
velocity
acceleration
attitude
attitude rates
thrust / torque
```

本项目第一版建议：

```text
position setpoint
velocity setpoint
position + velocity feedforward
```

后期再考虑：

```text
acceleration setpoint
attitude setpoint
body rate + thrust
```

#### MAVROS

链接：

```text
https://docs.px4.io/v1.14/en/ros/mavros_installation
https://docs.px4.io/v1.14/en/ros/ros1
```

用途：

```text
ROS1 与 PX4/MAVLink 桥接。
适配你的 Jetson Xavier NX + Ubuntu 20.04 + ROS Noetic。
```

注意：

```text
PX4 官方长期推荐迁移 ROS2。
但你的当前硬件与已有系统是 Ubuntu20.04 + ROS Noetic，因此短期使用 ROS1/MAVROS 是合理的工程选择。
```

### 4.3 D435I / RealSense

#### RealSense ROS

链接：

```text
https://github.com/realsenseai/realsense-ros
```

注意：

```text
ROS1 wrapper 使用 ros1-legacy branch。
D435I 可同时输出 RGB、depth、IMU。
```

本项目第一版用法：

```text
D435I -> AprilTag / ArUco / object pose / depth point
  -> 输出低维目标位姿
  -> policy / visual servo / state machine 使用目标位姿
```

不要第一版：

```text
RGBD image -> CNN/Transformer -> UAV+arm raw action
```

### 4.4 仿真与 RL

#### Isaac Lab

链接：

```text
https://isaac-sim.github.io/IsaacLab/
https://github.com/isaac-sim/IsaacLab
```

用途：

```text
大规模并行 RL、domain randomization、机器人学习。
适合后期复现论文的 whole-body training。
```

对本项目：

```text
中后期用 Isaac Lab 建立 quadrotor + simplified arm。
第一阶段不建议直接上完整 Isaac Lab 耦合系统。
```

#### MuJoCo

链接：

```text
https://mujoco.org/
https://github.com/google-deepmind/mujoco_menagerie
```

用途：

```text
机械臂刚体动力学、接触、摩擦、低维任务快速验证。
```

本项目建议：

```text
先用 MuJoCo 做 SO101 简化臂 reach / push / gripper / payload。
再把无人机抽象成移动基座或简化动力学。
```

#### gym-pybullet-drones

链接：

```text
https://github.com/utiasDSL/gym-pybullet-drones
```

用途：

```text
无人机 RL 入门、hover、trajectory、single/multi-agent drone。
```

定位：

```text
学习无人机 RL 接口和 reward 设计。
不作为最终 PX4 真实控制替代。
```

#### Agilicious

链接：

```text
https://github.com/uzh-rpg/agilicious
```

用途：

```text
论文部署仿真/真机框架参考。
本项目可以借鉴其“外环 policy + 内环飞控”的架构思想，不必强依赖其代码。
```

---

## 5. 硬件架构建议

### 5.1 当前硬件

```text
飞控：
  Pixhawk / PX4，后续可能更换

上位机：
  Jetson Xavier NX
  Ubuntu 20.04
  ROS Noetic
  MAVROS

视觉：
  Intel RealSense D435I

机械臂：
  SO101 简化版
  去除底座 yaw 自由度
  锁定部分关节
  保留 3-4DOF

低层控制：
  无人机：PX4 position/velocity/attitude control
  机械臂：舵机内部 PID 或 ROS/Python 外层 PID
```

### 5.2 SO101 是否适合直接上天

结论：

```text
SO101 适合作为桌面学习平台和算法验证平台。
不应默认作为第一版真实飞行机械臂。
```

需要先测：

```text
1. 简化后机械臂重量。
2. 机械臂伸出时重心偏移。
3. 舵机峰值电流。
4. 供电噪声对 PX4/Jetson 的影响。
5. 关节间隙/回差。
6. 末端负载能力。
7. 桨叶安全距离。
8. 无人机推重比。
```

推重比最低工程要求：

```text
总最大拉力 / 起飞重量 >= 2.0
```

更稳妥：

```text
>= 2.5
```

如果达不到：

```text
不要挂 SO101 真飞。
改用自研轻量 2DOF/3DOF 刚性臂或更大平台。
```

### 5.3 推荐机械臂配置

#### 配置 A：完整 SO101 桌面学习平台

用途：

```text
LeRobot teleop
dataset recording
BC / ACT / Diffusion Policy
HIL-SERL
```

不挂无人机。

#### 配置 B：简化 4DOF 空中操作平台

建议保留：

```text
shoulder pitch
elbow pitch
wrist pitch or wrist roll
gripper
```

锁定：

```text
base yaw
非必要 wrist yaw / wrist roll
```

用途：

```text
近距离触碰、轻抓取、末端局部调整。
```

#### 配置 C：极简 3DOF 安全平台

建议保留：

```text
shoulder pitch
elbow pitch
gripper
```

可选：

```text
wrist pitch
```

用途：

```text
第一版低空真机悬停 + 小动作。
```

### 5.4 供电

推荐：

```text
PX4:
  独立电源模块

Jetson:
  独立稳压供电，留足峰值电流

SO101 servos:
  独立 BEC / DC-DC
  与 PX4/Jetson 共地
  加电流监测和保险
```

禁止：

```text
舵机电源直接从飞控或 Jetson 取电。
```

### 5.5 安全

必须具备：

```text
遥控器接管
PX4 failsafe
Offboard timeout
机械臂急停
机械臂 freeze/retract
地面电源测试
无桨测试
保护绳/防护网
低空测试
日志记录
```

---

## 6. SO101 控制方案

### 6.1 不要先改 LeRobot 源码

锁定部分关节时，不建议一开始深改 LeRobot 内部。

推荐做一个 action mapping layer：

```text
reduced_action: R^3 or R^4
  -> full_action_mapper
  -> full_so101_action: R^N
  -> LeRobot SO101Follower.send_action()
```

示例：

```python
def map_reduced_to_full(
    reduced_action,
    home_action,
    active_joint_indices,
    locked_joint_indices,
    locked_joint_values,
):
    full_action = home_action.copy()
    full_action[active_joint_indices] = reduced_action
    full_action[locked_joint_indices] = locked_joint_values
    return full_action
```

好处：

```text
保留 LeRobot 校准、通信、数据格式。
后期可恢复全自由度。
```

### 6.2 ROS1 Bridge 设计

创建 ROS1 包：

```text
so101_ros1_bridge
```

节点：

```text
so101_driver_node.py
```

职责：

```text
1. 调用 LeRobot SO101Follower。
2. 发布 /so101/joint_states。
3. 订阅 /so101/command_joint_positions。
4. 订阅 /so101/command_joint_deltas。
5. 订阅 /so101/gripper_command。
6. 实现 locked joints 映射。
7. 实现 joint limit / velocity limit。
8. 实现 emergency stop。
9. 实现 home / freeze / relax。
```

Topic 建议：

```text
/so101/joint_states              sensor_msgs/JointState
/so101/command_joint_positions   trajectory_msgs/JointTrajectory or custom
/so101/command_joint_deltas      custom message
/so101/gripper_command           std_msgs/Float32 or custom
/so101/status                    custom status
/so101/estop                     std_msgs/Bool
/so101/home                      std_msgs/Empty
/so101/freeze                    std_msgs/Empty
```

参数文件：

```yaml
so101:
  port: "/dev/ttyUSB0"
  id: "aerial_so101"
  active_joints:
    - shoulder_pitch
    - elbow_pitch
    - wrist_pitch
    - gripper
  locked_joints:
    base_yaw: 0.0
    wrist_roll: 0.0
  limits:
    shoulder_pitch: [-1.2, 1.2]
    elbow_pitch: [-1.5, 1.5]
    wrist_pitch: [-1.0, 1.0]
  max_velocity:
    shoulder_pitch: 0.4
    elbow_pitch: 0.5
    wrist_pitch: 0.5
  command_rate_hz: 50
```

### 6.3 SO101 桌面验收

必须先完成：

```text
1. lerobot-find-port 能找到端口。
2. lerobot-setup-motors 完成电机配置。
3. lerobot-calibrate 完成校准。
4. lerobot-teleoperate 能正常遥操作。
5. Python API 能 read observation / send action。
6. ROS1 bridge 能发布 JointState。
7. locked joints 能保持固定。
8. 关节越界命令会被拦截。
```

验收命令：

```bash
rostopic echo /so101/joint_states
rostopic pub /so101/home std_msgs/Empty "{}"
rostopic pub /so101/freeze std_msgs/Empty "{}"
```

---

## 7. 系统软件架构

### 7.1 总体 ROS 图

```text
D435I / RealSense
  -> /camera/color/image_raw
  -> /camera/depth/image_rect_raw
  -> /camera/imu
  -> target pose estimator

PX4 + MAVROS
  -> /mavros/state
  -> /mavros/local_position/pose
  -> /mavros/local_position/velocity_local
  -> /mavros/imu/data
  <- /mavros/setpoint_position/local
  <- /mavros/setpoint_velocity/cmd_vel_unstamped
  <- /mavros/setpoint_raw/local

SO101 bridge
  -> /so101/joint_states
  -> /so101/status
  <- /so101/command_joint_positions
  <- /so101/gripper_command

wholebody_policy_runner
  subscribes:
    drone state
    arm state
    target pose
    previous action
  publishes:
    px4 setpoint
    so101 command
    policy diagnostics

safety_supervisor
  monitors:
    offboard heartbeat
    geofence
    battery
    joint limits
    policy output
    RC override
  publishes:
    emergency hover / freeze / land
```

### 7.2 坐标系

必须建立清晰 TF：

```text
map / world
  -> odom / local_origin
  -> base_link / drone_body
  -> arm_base
  -> arm_link_1
  -> arm_link_2
  -> wrist
  -> end_effector
  -> camera_color_optical_frame
```

注意：

```text
ROS 常用 ENU / FLU。
PX4/MAVLink 常涉及 NED / FRD。
D435I optical frame 有自己的相机坐标定义。
```

必须写清：

```text
T_drone_body_arm_base
T_drone_body_camera
T_camera_target
T_arm_base_ee
```

### 7.3 数据记录

每次实验记录 rosbag：

```text
/mavros/state
/mavros/local_position/pose
/mavros/local_position/velocity_local
/mavros/imu/data
/so101/joint_states
/so101/command_joint_positions
/wholebody_policy/observation
/wholebody_policy/action_raw
/wholebody_policy/action_filtered
/wholebody_policy/reward_terms
/safety/status
/target/pose
/tf
```

同时保存 PX4 ULog。

---

## 8. 传统控制基线

### 8.1 无人机基线

第一版：

```text
PX4 Position Mode
PX4 Altitude Mode
MAVROS Offboard position setpoint
MAVROS Offboard velocity setpoint
```

必须先完成：

```text
1. SITL 起飞/悬停/降落。
2. SITL waypoint tracking。
3. 真机手动 Stabilized / Altitude / Position 熟悉。
4. 真机不挂臂 Offboard hover。
5. 假载荷 hover。
```

### 8.2 机械臂基线

第一版：

```text
joint position control
joint delta control
servo internal PID
```

需要实现：

```text
home
freeze
locked joints
joint limit
velocity limit
acceleration smoothing
gripper open/close
```

### 8.3 末端运动学基线

需要建立：

```text
so101_simplified_3dof.urdf
so101_simplified_4dof.urdf
```

推荐工具：

```text
Pinocchio
KDL
MoveIt, optional
MuJoCo MJCF, for dynamics/contact
```

实现：

```text
FK: q -> T_arm_base_ee
IK: T_arm_base_ee_goal -> q_target
Jacobian: qdot -> ee_velocity
```

---

## 9. RL 全身控制设计

### 9.1 任务 1：无人机悬停 + 末端到点

第一版任务定义：

```text
给定 target position in drone body frame，
policy 输出 drone setpoint correction + arm joint command，
让 end-effector 到达 target。
```

不包含：

```text
真实接触
抓取
复杂目标识别
图像端到端
在线 RL
```

### 9.2 Observation v1

```text
drone_pos_error_body      3
drone_vel_body            3
drone_attitude_6d         6
drone_angular_vel_body    3
active_q                  n
active_qdot               n
ee_pos_body               3
target_pos_body           3
ee_error_body             3
last_action               m
```

如果 4DOF arm + 7D action，则大约：

```text
obs_dim = 3 + 3 + 6 + 3 + 4 + 4 + 3 + 3 + 3 + 7 = 39
```

### 9.3 Action v1

推荐：

```text
action = [
  drone_vx_body,
  drone_vy_body,
  drone_vz_body,
  drone_yaw_rate,
  arm_joint_delta_1,
  arm_joint_delta_2,
  arm_joint_delta_3,
  optional_arm_joint_delta_4,
  gripper_cmd
]
```

简化：

```text
3DOF arm:
  action_dim = 3 drone velocity + 1 yaw rate + 3 arm joints + 1 gripper = 8

4DOF arm:
  action_dim = 3 drone velocity + 1 yaw rate + 4 arm joints + 1 gripper = 9
```

如果不训练 gripper：

```text
action_dim = 7 or 8
```

### 9.4 Action scaling

```python
drone_v_cmd = clip(action[0:3] * v_max, -v_max, v_max)
yaw_rate_cmd = clip(action[3] * yaw_rate_max, -yaw_rate_max, yaw_rate_max)
q_delta = action[4:4+n] * q_delta_max
q_target = clip(q_current + q_delta, q_min, q_max)
```

建议初值：

```text
v_max = 0.2-0.5 m/s in sim
real_v_max = 0.05-0.15 m/s first flight
yaw_rate_max = 10-20 deg/s in sim
real_yaw_rate_max = 5-10 deg/s first flight
q_delta_max = 2-5 deg per step
policy_rate = 20-50 Hz
```

### 9.5 Reward v1

```text
R = R_ee
  + R_progress
  + R_success
  - R_drone_position
  - R_attitude
  - R_arm_limit
  - R_action
  - R_smooth
  - R_collision
  - R_failsafe
```

建议：

```python
R_ee = -1.0 * norm(ee_pos_body - target_pos_body)
R_progress = 0.5 * (prev_err - err)
R_drone_position = -0.2 * norm(drone_pos_world - hover_pos_world)
R_attitude = -0.1 * (roll**2 + pitch**2)
R_action = -0.01 * norm(action)**2
R_smooth = -0.05 * norm(action - last_action)**2
```

成功：

```python
if err < 0.05 and drone_stable and hold_steps > 20:
    reward += 5.0
    done = True
```

失败：

```python
if abs(roll) > 30deg or abs(pitch) > 30deg:
    reward -= 10
    done = True

if altitude < min_alt or altitude > max_alt:
    reward -= 10
    done = True

if joint_limit_violation:
    reward -= 10
    done = True
```

### 9.6 Algorithm

第一版：

```text
PPO
```

理由：

```text
1. 参考论文使用 PPO。
2. 机器人全身控制常用。
3. 并行仿真下稳定。
4. 调参资料多。
```

第二版对比：

```text
SAC
TD3
```

后期升级：

```text
PPO + privileged critic
teacher-student
RL + inner-loop dynamics estimator
RL + residual dynamics model
RL + MPC
```

### 9.7 Policy 网络

起点：

```text
Actor MLP: [256, 256]
Critic MLP: [256, 256]
activation: ELU or ReLU
action distribution: Gaussian
observation normalization: RunningMeanStd / VecNormalize
```

论文配置：

```text
[512, 256, 128]
ELU
Gaussian policy
RunningStandardScaler
```

建议：

```text
先 [256, 256] 跑通，再上 [512, 256, 128]。
```

---

## 10. 仿真路线

### 10.1 不同仿真器职责

```text
Gymnasium + SB3:
  RL 入门、简单 reach 环境、reward/action 快速验证。

MuJoCo:
  简化 SO101 机械臂动力学、接触、摩擦、payload。

Isaac Lab:
  大规模并行 RL、whole-body UAV + arm、domain randomization。

PX4 SITL + Gazebo:
  MAVROS / Offboard / 飞控接口验证。

RViz:
  TF、URDF、状态可视化、目标点可视化。
```

### 10.2 推荐顺序

```text
Phase A: SO101 桌面控制
  LeRobot -> teleop -> record -> replay

Phase B: SO101 简化臂仿真
  URDF/MJCF -> FK/IK -> joint control

Phase C: Arm-only RL
  reach task -> PPO/SAC -> action/reward 调通

Phase D: Drone-only baseline
  PX4 SITL -> MAVROS Offboard -> hover/waypoint

Phase E: Coupled sim
  simplified drone dynamics + simplified arm
  or Isaac Lab quadrotor + arm

Phase F: Deployment sim
  PX4 SITL + arm mock

Phase G: Real staged test
  no prop -> desk arm -> fake payload -> low hover -> real arm slow motion
```

### 10.3 不建议

```text
不建议第一版直接在 Gazebo 中训练 RL。
不建议第一版把 D435I 图像直接输入 policy。
不建议第一版让 RL 输出 motor thrust/PWM。
不建议没有假载荷测试就挂 SO101 飞。
```

---

## 11. 推荐项目结构

```text
aerial_so101_rl/
  README.md
  docs/
    literature_notes.md
    system_architecture.md
    hardware_checklist.md
    safety_protocol.md
    experiment_log_template.md
  configs/
    so101_simplified_3dof.yaml
    so101_simplified_4dof.yaml
    px4_offboard.yaml
    reward_wholebody_v1.yaml
  models/
    urdf/
      so101_simplified_3dof.urdf
      so101_simplified_4dof.urdf
      quadrotor_so101.urdf
    mujoco/
      so101_simplified.xml
      quadrotor_so101.xml
    isaac/
      usd/
  envs/
    arm_reach_env.py
    quad_hover_env.py
    quad_arm_reach_env.py
  controllers/
    action_mapper.py
    arm_pid_controller.py
    px4_setpoint_controller.py
    safety_filter.py
  policies/
    ppo_policy.py
    export_onnx.py
  ros1_ws/
    src/
      so101_ros1_bridge/
      px4_arm_offboard/
      wholebody_policy_runner/
      aerial_manipulation_msgs/
      target_pose_estimator/
  scripts/
    test_so101_lerobot.py
    train_ppo.py
    eval_policy.py
    compare_baselines.py
    export_policy_onnx.py
    replay_rosbag.py
```

---

## 12. Codex / Claude Code 任务拆分

### Task 0：仓库审计

给 Codex 的指令：

```text
请先审计当前工程，不要假设文件名。
查找已有的 ROS workspace、LeRobot 脚本、SO101/机械臂模型、PX4/MAVROS 代码、RL 训练脚本。
输出：
1. 当前已有文件树摘要
2. 可复用代码
3. 缺失模块
4. 不要创建重复包
```

### Task 1：SO101 控制验证

交付：

```text
scripts/test_so101_lerobot.py
```

功能：

```text
connect
read observation
send home
send small joint move
send reduced action through mapper
lock joints
emergency stop
```

验收：

```text
能读取关节状态
能执行小动作
locked joints 不动
越界命令被拒绝
```

### Task 2：SO101 ROS1 Bridge

交付：

```text
ros1_ws/src/so101_ros1_bridge
```

功能：

```text
/so101/joint_states publisher
/so101/command_joint_positions subscriber
/so101/command_joint_deltas subscriber
/so101/gripper_command subscriber
/so101/status publisher
/so101/estop subscriber
locked joints action mapper
joint limit filter
```

### Task 3：PX4 Offboard Manager

交付：

```text
ros1_ws/src/px4_arm_offboard
```

功能：

```text
takeoff
hover
land
position setpoint
velocity setpoint
offboard heartbeat
failsafe monitor
RC override detection
```

验收：

```text
SITL 中能起飞、悬停、降落。
Offboard 不因 setpoint 中断掉线。
```

### Task 4：Arm Kinematics

交付：

```text
controllers/arm_kinematics.py
models/urdf/so101_simplified_3dof.urdf
models/urdf/so101_simplified_4dof.urdf
```

功能：

```text
FK
Jacobian
simple IK
joint limit check
ee pose publisher
TF tree
```

### Task 5：Whole-body Policy Runner

交付：

```text
ros1_ws/src/wholebody_policy_runner
```

功能：

```text
observation builder
policy inference
action scaling
safety filter
MAVROS setpoint publisher
SO101 command publisher
diagnostics publisher
```

### Task 6：RL 仿真环境

交付：

```text
envs/quad_arm_reach_env.py
scripts/train_ppo.py
scripts/eval_policy.py
```

功能：

```text
reset
step
observation
action mapping
reward terms
termination
domain randomization
baseline comparison
```

### Task 7：实验评估

交付：

```text
scripts/compare_baselines.py
docs/experiment_report_template.md
```

必须对比：

```text
1. PX4 hover + fixed arm
2. PX4 hover + traditional arm IK
3. RL arm-only
4. RL whole-body
5. ablation without action smoothing
6. ablation without randomization
```

指标：

```text
end-effector final error
end-effector RMSE
drone position RMSE
roll/pitch/yaw max
joint limit violation count
action smoothness
success rate
failsafe count
```

---

## 13. 最小可行 MVP

### 13.1 MVP 定义

```text
在仿真中，简化四旋翼保持悬停，简化 SO101 末端跟踪目标点。
policy 输出无人机速度修正和机械臂关节目标。
```

不包括：

```text
真实抓取
真实接触
图像端到端
复杂避障
在线学习
```

### 13.2 MVP 输入输出

输入：

```text
drone state
arm joint state
target position in drone body frame
previous action
```

输出：

```text
drone velocity setpoint
arm joint targets
```

### 13.3 MVP 成功标准

仿真：

```text
success rate > 80%
mean ee error < 10 cm
drone hover error 不发散
roll/pitch 不长期超过安全阈值
action 无明显高频振荡
```

SITL：

```text
MAVROS Offboard 连续稳定
arm mock command 正常
policy runner 不阻塞 setpoint
```

桌面 SO101：

```text
active joints 能跟踪命令
locked joints 保持
急停有效
```

真机低空：

```text
先只挂假载荷
再挂断电机械臂
最后挂通电机械臂并做慢速小动作
```

---

## 14. 训练与部署路线

### 14.1 第 1-2 周：SO101/LeRobot 桌面闭环

任务：

```text
安装 LeRobot
完成 SO101 端口、电机、校准
完成 teleop
记录一个简单 dataset
训练一个 BC/ACT 或最小 policy
```

输出：

```text
SO101 控制视频
dataset
训练日志
README
```

### 14.2 第 3-4 周：ROS1 Bridge + PX4 SITL

任务：

```text
so101_ros1_bridge
PX4 SITL
MAVROS Offboard hover
rosbag 记录
```

输出：

```text
ROS graph
SITL hover video
JointState + Offboard logs
```

### 14.3 第 5-8 周：arm-only / simplified whole-body RL

任务：

```text
Gymnasium/MuJoCo arm reaching
quad-arm simplified environment
PPO training
baseline comparison
```

输出：

```text
training curve
success rate
ablation table
policy checkpoint
```

### 14.4 第 9-12 周：Isaac Lab / deployment sim

任务：

```text
建立 quadrotor + simplified arm
加入 domain randomization
导出 ONNX/TorchScript
接 ROS policy runner
```

输出：

```text
Isaac Lab demo
policy export
SITL integration
```

### 14.5 第 13 周以后：真机分级测试

任务：

```text
无桨联调
假载荷 hover
假臂 hover
真实简化臂台架
低空慢速动作
```

输出：

```text
ULog
rosbag
安全测试表
对比曲线
实验视频
```

---

## 15. 研究问题设计

### 15.1 论文级问题

可以把课题组织成：

```text
如何在欠驱动四旋翼 + 轻量简化刚性臂系统上，
通过传统低层控制与学习型外环策略结合，
实现安全、可迁移的空中末端操作？
```

### 15.2 可发表/可写论文的贡献点

保守但可落地：

```text
1. 基于 SO101 简化臂的低成本空中操作实验平台。
2. 面向 PX4/MAVROS 的 hierarchical whole-body RL 框架。
3. 面向轻量机械臂挂载的 action/observation/reward 设计。
4. sim-to-real 分级验证：arm-only -> drone-only -> coupled -> real staged tests。
5. 对比 traditional decoupled control 与 RL whole-body coordination。
```

更前沿但风险更高：

```text
1. RL + inner-loop dynamics estimator。
2. residual dynamics learning + MPC。
3. privileged critic / teacher-student sim-to-real。
4. D435I 视觉目标输入 + whole-body policy。
5. 接触/推压任务的 model-based DRL。
```

### 15.3 不建议作为第一篇主贡献

```text
端到端图像到电机控制
真实无人机在线 RL
复杂抓取 + 接触 + 视觉 + 全身控制一次性完成
直接复现论文 Osprey 全部控制器细节
强行把 SO101 原版完整挂飞
```

---

## 16. 关键实验设计

### 16.1 Baseline 1：解耦传统控制

```text
无人机保持 hover
机械臂使用 IK/PID 到点
```

目的：

```text
证明没有 whole-body coordination 时，机械臂动作会扰动无人机或末端误差较大。
```

### 16.2 Baseline 2：arm-only policy

```text
无人机固定或理想 hover
RL 只控制 arm joints
```

目的：

```text
证明机器人学习流程可用。
```

### 16.3 Proposed：whole-body policy

```text
RL 同时输出 drone correction + arm command
```

目的：

```text
证明 policy 学到了无人机-机械臂协同。
```

### 16.4 Ablation

必须做：

```text
without action smoothing
without payload randomization
without arm joint states
without drone velocity
without drone correction action
```

预期：

```text
无动作平滑 -> 抖动大
无随机化 -> 扰动下性能下降
无关节状态 -> arm pose 控制变差
无无人机状态 -> ee position 控制变差
无 drone correction -> 末端误差/hover 误差变大
```

---

## 17. 视觉与 D435I 使用方式

### 17.1 第一版

```text
D435I + AprilTag
  -> target pose
  -> low-dimensional target state
  -> policy / visual servo
```

### 17.2 第二版

```text
D435I depth
  -> target 3D point
  -> obstacle / workspace check
  -> safety filter
```

### 17.3 第三版

```text
RGBD / segmentation / object detector
  -> object pose / affordance
  -> task planner
  -> policy target command
```

### 17.4 不建议第一版

```text
RGB image
  -> CNN/Transformer
  -> direct drone+arm action
```

原因：

```text
数据量不足
训练不稳定
难以调试
真机风险高
```

---

## 18. 安全 supervisor

必须独立于 policy 实现。

### 18.1 输入

```text
PX4 state
MAVROS local pose
MAVROS velocity
PX4 battery
RC override
SO101 joint states
SO101 current/temperature if available
policy raw action
filtered action
target pose
```

### 18.2 检查项

```text
roll/pitch max
yaw rate max
altitude min/max
local position geofence
velocity max
arm joint limit
arm velocity max
arm self collision
end-effector no-fly zone around propellers
Offboard heartbeat
policy inference timeout
camera/target timeout
```

### 18.3 输出

```text
emergency hover
arm freeze
arm retract
land
disarm request, only under controlled conditions
```

---

## 19. 后续需要 Codex 补全的资料

### 19.1 本地论文待继续精读

```text
Learning-Based_Methods_for_Aerial_Manipulation_A_Focused_Review.pdf
AION_Aerial_Indoor_Object_Goal_Navigation_Using_Dual_Policy_RL_arXiv_2601.15614.pdf
```

### 19.2 外部链接待补

```text
IEEE Xplore 11267184
```

需要用户提供：

```text
PDF
BibTeX
题名/摘要截图
```

---

## 20. 给 Codex 的完整执行提示词

可直接复制给 Codex / Claude Code：

```text
你正在处理一个“欠驱动四旋翼 + 简化 SO101 机械臂 + PX4/MAVROS + Jetson + D435I”的空中操作项目。

请先阅读 docs/ 或本项目文档，不要假设已有文件名。你的第一步必须是审计当前工程结构，查找已有 ROS workspace、SO101/LeRobot 控制脚本、PX4/MAVROS 代码、URDF/MJCF/Isaac 模型、RL 训练脚本。

项目原则：
1. RL 作为外环 whole-body coordination policy。
2. PX4/MAVROS 负责无人机底层控制。
3. SO101 使用舵机内部 PID 或 ROS/Python PID 跟踪关节目标。
4. 简化 SO101 采用 reduced action -> full action mapping，不要直接深改 LeRobot 源码。
5. D435I 第一版只输出目标位姿，不做图像端到端控制。
6. 真机无人机不做在线 RL，所有 policy 必须先经过仿真和 safety filter。
7. 不要让 RL 输出 raw motor PWM / raw rotor thrust 作为第一版实现。

请分阶段完成：

Phase 0: 审计当前代码和文档，输出可复用模块与缺失模块。
Phase 1: 建立 SO101 Python 控制验证脚本，完成 connect/read/send/home/estop/locked joints。
Phase 2: 建立 ROS1 so101_ros1_bridge，发布 JointState，订阅关节命令，实现限位和急停。
Phase 3: 建立 PX4/MAVROS Offboard manager，完成 SITL takeoff/hover/land/velocity setpoint。
Phase 4: 建立简化 SO101 URDF/MJCF 和 FK/IK/Jacobian。
Phase 5: 建立 arm-only reach RL 环境，先用 PPO 跑通。
Phase 6: 建立 quadrotor + arm simplified whole-body 环境，训练 PPO policy。
Phase 7: 建立 wholebody_policy_runner，把 policy 输出映射为 MAVROS setpoint + SO101 command。
Phase 8: 加入 safety_supervisor 和 rosbag/ULog logging。

每一阶段都要给出：
- 修改了哪些文件
- 如何运行
- 如何验证
- 当前未验证/不支持什么
- 不要声称真机验证，除非真的有日志或视频证据
```

---

## 21. 最重要的工程原则

```text
1. 先 SO101 桌面，后无人机。
2. 先传统控制，后 RL。
3. 先低维状态，后视觉端到端。
4. 先仿真，后 SITL，再假载荷，最后真臂。
5. RL 只做高层策略，PX4/PID 保底。
6. SO101 是学习平台，不默认是飞行最终臂。
7. 每次真机测试都必须有日志、急停和人工接管。
```

这七条是项目保命线，也是保证课题能毕业、能写论文、能做作品集的主线。
