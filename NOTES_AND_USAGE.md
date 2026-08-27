# SO-101 MuJoCo 轨迹跟踪修改记录与使用说明

本文档用于记录本项目的本地修改、常用命令、第一阶段实验流程，以及
IK / PID / MPC / RL 在机械臂运动控制中的关系。

## 修改记录

### 2026-08-27

- 新增汇报用 MP4 导出能力。
  - 新增 `scripts/video_utils.py`，统一使用 `imageio` 写出 H.264 MP4。
  - `mujoco_planar_control_sim.py`、`compare_planar_controllers.py`、`eval_tracking.py`、
    `eval_pick_lift.py`、`train_sac.py`、`train_pick_lift_sac.py` 均支持
    `--record-video --video-fps`。
  - `ik_baseline.py` 和 `ik_random_tracking.py` 支持直接录制传统 IK rollout，
    不再依赖交互 viewer。
  - `view_so101.py --record-video` 可生成 5 秒默认或指定 `--duration` 的场景展示片段。
- 修正 `SO101TrackingEnv.render()` 重复创建 renderer 的问题，并增加 `close()` 释放资源。

### 2026-08-04

- 新增实时 Cartesian servo 通路，面向后续无人机/RL 联动。
  - 新增 `so101_ros1_bridge/servo.py`：ROS 无关的
    `PlanarCartesianServo`、`RuckigJointLimiter`、`SimpleJointLimiter`。
  - 新增 `so101_servo_node.py`：100Hz ROS 节点，订阅
    `/so101/cartesian_servo_target` 和 `/so101/ee_velocity_cmd`，发布
    `/so101/command_joint_servo`。
  - `so101_driver_node.py` 新增 `/so101/command_joint_servo` 订阅；driver 仍只做
    真机安全层、限位、限速、超时保护和 Feetech 写入。
  - `JointSafetyFilter.set_servo_target()` 只更新连续 servo 目标，不重启
    minimum-jerk 轨迹，避免高频目标导致反复 ease-in/ease-out。
  - `mujoco_planar_control_sim.py` 新增 `--controller servo`，本机仿真可复用
    与真机一致的 servo 控制律。
  - `tests/test_servo.py` 覆盖 Jacobian、DLS velocity resolve、Ruckig/simple limiter
    和 servo-mode safety filter。
- 修正状态 topic 命名。
  - `/so101/servo_status` 保留给 driver 的低层舵机/后端遥测。
  - `/so101/cartesian_servo_status` 用于新的 Cartesian servo 节点状态，避免两个
    不同 JSON schema 混在同一 topic。
- Python/Jetson 兼容性结论。
  - `ros1_ws/src/so101_ros1_bridge` 已按 Python 3.8 语法检查通过，适合
    Ubuntu 20.04 / ROS Noetic 路径。
  - Ruckig 是可选依赖；Jetson 上安装失败时会自动使用 `SimpleJointLimiter`。
  - 桌面训练模块 `so101_tracking/` 和部分 standalone 脚本包含 Python 3.10+
    类型标注，建议继续留在本机训练环境，不直接作为 Noetic 运行路径。

### 2026-08-03

- 新增本机无 ROS 的 MuJoCo 平面 3DOF+夹爪控制仿真脚本。
  - 新增 `scripts/mujoco_planar_control_sim.py`。
  - 默认保持 `shoulder_pan=0`、`wrist_roll=0`，只控制
    `shoulder_lift`、`elbow_flex`、`wrist_flex` 和 `gripper`。
  - 支持 `--execution-mode joint_trajectory`：先离线用阻尼最小二乘 IK 求整条
    XZ 末端轨迹，再用 cubic 多点关节轨迹执行。
  - 支持 `--execution-mode cartesian_stream`：模拟在线连续发送单点目标，用于对比
    单点命令反复重启插值造成的滞后/抖动。
  - 输出 CSV 和 XZ 轨迹图到 `outputs/mujoco_planar_control_sim/`。
- 修正平面 8 字轨迹可视化用法。
  - `mujoco_planar_control_sim.py` 新增 `--cycles`，不传 `--duration` 时按
    `cycles / frequency` 自动运行完整周期。
  - 默认可视化参数改为 `x/z` 幅度 `0.03 m`、`0.05 Hz`、`1` 个完整 cycle，
    避免之前 `duration=20`、`frequency=0.02` 只画 `0.4` 圈。
  - MuJoCo viewer 新增红色目标点和绿色末端点，便于确认机械臂与目标是否同步运动。
- 新增三套本地控制器对比。
  - `--controller joint_trajectory`：当前预规划 DLS IK + cubic timed trajectory。
  - `--controller moveit_like`：同一 IK 路径上做 MoveIt 风格的关节速度限幅重定时。
  - `--controller argo_like`：在线 DLS IK 直接写 MuJoCo 关节位置目标，作为 Argo-Robot/controls 风格基线。
  - 新增 `scripts/compare_planar_controllers.py`，自动输出三者 CSV、summary 和综合轨迹/误差图。
- 接入两个开源项目用于更真实的对比。
  - MoveIt 源码快照放在 `third_party/moveit/`，本机 Ubuntu 24.04 无 ROS 时不直接运行
    MoveIt 节点；当前用 Python `ruckig` 实现 `--controller moveit_ruckig`，
    对应 MoveIt 轨迹平滑/时间参数化中常用的 jerk-limited retiming 思路。
  - Argo-Robot/controls raw 源码快照放在 `third_party/Argo-Robot-controls-raw/`，
    `--controller argo_external` 会调用其 `URDF_Kinematics` 对 SO101 做 IK。
  - Argo 的 `gripper_link` 与 MuJoCo 的 body `gripper` 对齐，因此公平对比时使用
    `--ee-frame body_gripper`，不要用默认的 `site_gripperframe`。
- 新增后仰和大轨迹测试支持。
  - `scripts/scan_planar_workspace.py` 可扫描平面 XZ 可达工作空间。
  - `mujoco_planar_control_sim.py` 新增 `--limit-profile mujoco`，用于仿真里使用
    MuJoCo XML 原始关节范围；默认 `config` 仍使用保守硬件安全限位。
  - 新增 `--start-at-first-target`，用于后仰轨迹测试时直接从第一目标点 IK 解起步，
    避免从前伸 `reach` 姿态切入后仰区域造成虚假的最大误差。
- 增强 `ros1_ws/src/so101_ros1_bridge/src/so101_ros1_bridge/control.py`。
  - `JointSafetyFilter` 新增可选 `clock` 参数；ROS 节点不传时行为不变，
    本机 MuJoCo 仿真可用仿真时间快速执行。
- 新增 `so101_planar_3dof_gripper.yaml`，明确区分“3 个平面机械臂关节 + 夹爪”
  与旧文件名中的 4DOF 表述。

### 2026-07-21

- 新增 SO-101 MuJoCo pick-and-lift cube 第一版抓取任务。
  - 新增 `assets/so101/scene_pick_lift.xml`，包含地面、freejoint 方块和抬升目标点。
  - 新增 `so101_tracking/pick_lift_env.py`，实现 state-based `SO101PickLiftEnv`。
  - 新增 `scripts/smoke_pick_lift_env.py`，用于不训练地验证 reach-close-lift 流程。
  - 新增 `scripts/train_pick_lift_sac.py` 和 `scripts/eval_pick_lift.py`。
  - 默认动作是 4 维 `ee_delta + gripper`，内部用 DLS IK 转为 arm 关节位置目标。
  - 默认启用 `virtual_grasp=True`，用于先稳定跑通抓取训练闭环；后续可用
    `--no-virtual-grasp` 做纯接触实验。
- 增强 SO-101 pick-and-lift 训练流程。
  - `SO101PickLiftEnv` 新增 `scripted_action()`，用于生成成功抓取示范。
  - `scripts/train_pick_lift_sac.py` 新增 `--demo-prefill-episodes`，训练前把示范
    transitions 写入 SAC replay buffer。
  - `scripts/train_pick_lift_sac.py` 新增 `--bc-pretrain-steps`，先用示范做 actor
    行为克隆 warm start，再进行 SAC 在线训练。
  - 默认 `--ent-coef auto_0.01`、`--gamma 0.97`、`--gradient-steps 2`。
- 新增 `PICK_LIFT_AND_ACT_NOTES.md`。
  - 记录 LeRobot ACT `httpx.ConnectError: [Errno 101] Network is unreachable`
    的处理方法。
  - 记录 pick-and-lift 环境设计、训练命令、评估命令和 TensorBoard 指标。
- 增强 `scripts/view_so101.py`。
  - 新增 `--model pick_lift`，可直接打开含方块和目标点的 MuJoCo 场景。
- 增强 `scripts/train_sac.py` 的 MuJoCo 可视化训练能力。
  - 新增参数：`--viewer-real-time`、`--viewer-speed`、`--frame-skip`。
  - `--viewer` 默认仍可用于快速训练时的视觉检查；加
    `--viewer-real-time` 后会按仿真时间节流，便于观察机械臂运动。
- 修正 `scripts/eval_tracking.py` 的默认模型选择逻辑。
  - 默认按模型文件修改时间选择最新 `best_model.zip`，避免误选旧 run。
  - 新增 `--frame-skip`，用于保持训练、评估、IK 基线时间设置一致。
- 新增 `scripts/ik_random_tracking.py`。
  - 单独运行阻尼最小二乘 IK，跟踪随机生成的平滑三维末端轨迹。
  - 不涉及 SAC、不涉及训练，只用于观察传统 IK 基线效果。
  - 支持 `--viewer` 在 MuJoCo 中直接观察运动。
- 新增本文档，用于保存修改记录、使用说明和阶段性实验方法。
- 增强 `so101_tracking/env.py`、`scripts/train_sac.py`、`scripts/eval_tracking.py`。
  - 新增 `--trajectory-mode {lissajous,random}`。
  - 新增 `--random-segments`、`--random-center`、`--random-half-range`。
  - 默认仍使用原固定 Lissajous 轨迹；显式设置 `random` 才切换到随机平滑轨迹。

## 常用命令

进入环境：

```bash
cd /home/bot/research
source .venvs/lerobot/bin/activate
```

基础检查：

```bash
python so101_mujoco_tracking/scripts/smoke_model.py
python so101_mujoco_tracking/scripts/smoke_env.py
python so101_mujoco_tracking/scripts/smoke_pick_lift_env.py
```

打开 SO-101 MuJoCo 模型：

```bash
python so101_mujoco_tracking/scripts/view_so101.py
```

打开含方块和抬升目标点的 pick-and-lift 场景：

```bash
python so101_mujoco_tracking/scripts/view_so101.py --model pick_lift
```

本机无 ROS 时运行平面 3DOF+夹爪控制仿真，默认画完整一套 8 字：

```bash
python so101_mujoco_tracking/scripts/mujoco_planar_control_sim.py \
  --controller joint_trajectory \
  --cycles 1 \
  --frequency 0.05 \
  --x-amplitude 0.03 \
  --z-amplitude 0.03
```

导出对应 MP4 汇报视频：

```bash
python so101_mujoco_tracking/scripts/mujoco_planar_control_sim.py \
  --controller joint_trajectory \
  --cycles 1 \
  --frequency 0.05 \
  --x-amplitude 0.03 \
  --z-amplitude 0.03 \
  --record-video
```

边看 MuJoCo，边实时看目标轨迹、末端轨迹和误差曲线。MuJoCo 里红点是当前
目标，绿点是末端：

```bash
python so101_mujoco_tracking/scripts/mujoco_planar_control_sim.py \
  --controller joint_trajectory \
  --viewer \
  --viewer-real-time \
  --live-plot \
  --cycles 1 \
  --frequency 0.05 \
  --x-amplitude 0.03 \
  --z-amplitude 0.03
```

对比三套控制指令：当前预规划 DLS+cubic、MoveIt-like 限速重定时、
Argo-like 在线 IK 直接写关节目标：

```bash
python so101_mujoco_tracking/scripts/compare_planar_controllers.py \
  --controllers joint_trajectory moveit_like argo_like \
  --cycles 1 \
  --frequency 0.05 \
  --x-amplitude 0.03 \
  --z-amplitude 0.03
```

对比当前控制、MoveIt/Ruckig、Argo 外部 IK 三套控制器。这个命令用的是
Argo 能对齐的 `body_gripper` 末端框架：

```bash
python so101_mujoco_tracking/scripts/compare_planar_controllers.py \
  --controllers joint_trajectory moveit_ruckig argo_external \
  --ee-frame body_gripper \
  --cycles 1 \
  --frequency 0.05 \
  --x-amplitude 0.03 \
  --z-amplitude 0.03
```

扫描平面 3DOF 机械臂工作空间：

```bash
python so101_mujoco_tracking/scripts/scan_planar_workspace.py \
  --limit-profile config \
  --ee-frame body_gripper

python so101_mujoco_tracking/scripts/scan_planar_workspace.py \
  --limit-profile mujoco \
  --ee-frame body_gripper
```

复现后仰大轨迹测试。注意这里明确使用 MuJoCo 原始限位，而不是硬件安全限位：

```bash
python so101_mujoco_tracking/scripts/compare_planar_controllers.py \
  --controllers joint_trajectory moveit_ruckig argo_external \
  --ee-frame body_gripper \
  --limit-profile mujoco \
  --center -0.04 0 0.32 \
  --x-amplitude 0.06 \
  --z-amplitude 0.02 \
  --frequency 0.05 \
  --cycles 1 \
  --start-at-first-target \
  --plan-multistart-every-target \
  --output-dir /tmp/so101_backbend_compare_true_external
```

本次已验证结果：

```text
joint_trajectory mean=0.000101 m max=0.000339 m final=0.000101 m
moveit_ruckig    mean=0.007196 m max=0.026951 m final=0.000101 m
argo_external    mean=0.002696 m max=0.039753 m final=0.000325 m
```

解释：`joint_trajectory` 在这条慢速、已知、无避障轨迹上误差最低，因为它提前
规划完整 IK 路径并按固定时间执行；`moveit_ruckig` 会按速度/加速度/jerk 限制
拉长执行时间，所以中途误差按原始目标时间比较会变大；`argo_external` 是在线
单步 IK，视觉上平滑，但遇到 IK 分支或局部误差时可能出现更大的瞬时误差。

对比在线单点命令流：

```bash
python so101_mujoco_tracking/scripts/mujoco_planar_control_sim.py \
  --controller cartesian_stream \
  --cycles 1 \
  --frequency 0.05 \
  --x-amplitude 0.03 \
  --z-amplitude 0.03
```

运行固定轨迹 IK 基线：

```bash
python so101_mujoco_tracking/scripts/ik_baseline.py
```

运行随机三维轨迹 IK 基线，不训练：

```bash
python so101_mujoco_tracking/scripts/ik_random_tracking.py \
  --segments 8 \
  --steps-per-segment 80 \
  --seed 0
```

边看 MuJoCo 边运行随机三维轨迹 IK 基线：

```bash
python so101_mujoco_tracking/scripts/ik_random_tracking.py \
  --viewer \
  --real-time \
  --segments 8 \
  --steps-per-segment 80 \
  --seed 0
```

训练 residual SAC，即在 IK 控制器基础上学习残差：

```bash
python so101_mujoco_tracking/scripts/train_sac.py \
  --total-timesteps 200000 \
  --control-mode ik_residual \
  --run-name residual_tracking
```

边看 MuJoCo 边训练 residual SAC：

```bash
python so101_mujoco_tracking/scripts/train_sac.py \
  --total-timesteps 200000 \
  --control-mode ik_residual \
  --viewer \
  --viewer-real-time \
  --run-name residual_tracking_viewer
```

评估最新 SAC 模型：

```bash
python so101_mujoco_tracking/scripts/eval_tracking.py
```

评估最新 SAC 模型在随机轨迹上的泛化能力：

```bash
python so101_mujoco_tracking/scripts/eval_tracking.py \
  --episodes 10 \
  --trajectory-mode random \
  --output-dir so101_mujoco_tracking/outputs/eval_random_latest
```

在随机轨迹上重新训练 residual SAC：

```bash
python so101_mujoco_tracking/scripts/train_sac.py \
  --total-timesteps 200000 \
  --control-mode ik_residual \
  --trajectory-mode random \
  --run-name residual_random_tracking
```

训练 SO-101 pick-and-lift cube，使用示范预填充 + BC warm start + SAC：

```bash
python so101_mujoco_tracking/scripts/train_pick_lift_sac.py \
  --total-timesteps 300000 \
  --control-mode ee_delta \
  --demo-prefill-episodes 50 \
  --bc-pretrain-steps 2000 \
  --demo-noise 0.03 \
  --run-name pick_lift_demo_bc_sac_v1
```

边看 MuJoCo 边训练 pick-and-lift：

```bash
python so101_mujoco_tracking/scripts/train_pick_lift_sac.py \
  --total-timesteps 300000 \
  --control-mode ee_delta \
  --demo-prefill-episodes 50 \
  --bc-pretrain-steps 2000 \
  --viewer \
  --viewer-real-time \
  --run-name pick_lift_demo_bc_sac_viewer
```

评估 pick-and-lift SAC：

```bash
python so101_mujoco_tracking/scripts/eval_pick_lift.py \
  --episodes 20 \
  --output-dir so101_mujoco_tracking/outputs/eval_pick_lift_v1
```

查看 TensorBoard：

```bash
tensorboard --logdir so101_mujoco_tracking/outputs/sac
```

如果系统找不到 `tensorboard`，使用虚拟环境里的命令：

```bash
.venvs/lerobot/bin/tensorboard \
  --logdir so101_mujoco_tracking/outputs/sac \
  --host 127.0.0.1 \
  --port 6006
```

然后在浏览器打开：

```text
http://127.0.0.1:6006/
```

TensorBoard 命令所在终端要保持打开，关闭终端后网页会断开。

## 如何看 TensorBoard 曲线

进入 TensorBoard 后选择 `Scalars` 页面，左侧 `Runs` 勾选要比较的训练：

- `residual_tracking_viewer`：固定 Lissajous 轨迹 residual SAC。
- `residual_random_tracking`：随机平滑轨迹 residual SAC。
- `smoke_test`、`residual_smoke`：短测试 run，通常只用于确认代码能跑。

建议重点看这些曲线：

- `tracking/error_m`：最重要，末端跟踪误差，应该整体下降并稳定到毫米级。
- `eval/mean_reward`：评估回报，应该上升并趋于平台；固定 300 步任务中接近
  295 说明已接近饱和。
- `rollout/ep_rew_mean`：训练采样过程中的平均回报，比 `eval/mean_reward`
  噪声更大。
- `train/critic_loss`：Q 网络损失，前期下降、后期稳定即可；不要求单调下降。
- `train/ent_coef`：SAC 熵系数，下降表示策略从探索转向利用；过早贴近 0 可能
  表示探索不足。
- `time/fps`：训练速度；开 MuJoCo viewer 或 `--viewer-real-time` 时会明显降低。

不建议过度解读：

- `train/actor_loss`：数值可能越来越负，不等价于“效果变差”。
- 单个 step 的尖峰：机械臂从 home pose 进入第一目标点时可能有瞬时误差峰值，
  更应看趋势、评估均值和最终误差。

判断一次训练是否有效：

```text
tracking/error_m: 下降并稳定
eval/mean_reward: 上升并平台化
critic_loss: 不发散
ent_coef: 合理下降
eval 曲线和 MuJoCo 可视化: 轨迹没有明显抖动、发散或撞限位
```

## 第一阶段建议流程

1. 先跑 `ik_baseline.py` 或 `ik_random_tracking.py`，得到不训练的传统基线。
2. 再跑 `train_sac.py --control-mode ik_residual`，让策略学习 IK 基线之外的
   小残差。
3. 用 `eval_tracking.py` 评估 SAC 模型。
4. 对比 IK 与 residual SAC 的平均误差、最大误差、终点误差和轨迹图。
5. 只有 residual SAC 稳定后，再考虑直接训练 `joint_delta` 或更难的抓取任务。

## 阶段 1.5：固定轨迹跑通后的下一步

你当前固定轨迹训练结果已经达到亚毫米级，说明原任务已接近饱和。后续不要继续
只在同一条轨迹上训练，应转向下面三类实验：

1. 泛化评估：用 `eval_tracking.py --trajectory-mode random --episodes 10`
   测不同随机轨迹。
2. 消融对比：比较 IK-only、residual SAC、direct `joint_delta` SAC。
3. 难度提升：扩大随机工作空间、减少 IK 迭代、降低执行器增益或增加扰动，
   检查 residual SAC 是否比 IK-only 更稳。

建议保存一张阶段 1.5 结果表：

```text
方法                    轨迹类型      mean error   max error   备注
DLS IK-only             lissajous     ...          ...         无训练
DLS IK-only             random        ...          ...         无训练
SAC + IK residual       lissajous     ...          ...         已训练
SAC + IK residual       random        ...          ...         泛化评估
SAC direct joint_delta  lissajous     ...          ...         难基线
```

做公平对比时，应保持以下参数一致：

- `episode_steps`
- `frame_skip`
- `ik_iters`
- `ik_gain`
- `ik_damping`
- `ik_max_dq`

## 控制与强化学习关系图

一个实用的机械臂控制栈通常是：

```text
任务目标 / 末端轨迹
  -> 轨迹插值 / 路径规划 / IK
  -> 关节位置、速度或力矩目标
  -> PID / 阻抗控制 / MPC / 电机控制器
  -> 机器人动力学
  -> 传感器观测与反馈
```

不同方法的位置：

- 阻尼最小二乘 IK：运动学层方法，把末端位置误差通过 Jacobian 转成关节增量。
- PID：底层反馈控制器，让关节、电机或执行器跟踪目标。
- MPC：优化控制器，基于模型反复求解短时域最优控制问题。
- MoveIt2：ROS2 生态中的路径规划、碰撞检测、轨迹生成和执行框架。
- RL：通过与环境交互和奖励信号学习策略，可以放在高层、中层或低层。

RL 可以有多种用法：

- 高层 RL：选择子目标、抓取时机、动作 primitive。
- residual RL：保留传统控制器，学习修正量。
- 端到端 RL：直接从观测输出关节目标、速度或力矩。
- model-based RL / MPC+RL：学习动力学模型、代价函数或 MPC 的补偿策略。

当前第一阶段采用 residual RL：DLS IK 先给出主要关节更新，SAC 学习一个小的
残差关节更新。这比从零学习关节运动更稳，也更适合作为 SO-101 仿真训练的
第一步。
