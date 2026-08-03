# SO-101 Pick-and-Lift 与 LeRobot ACT 网络问题说明

本文记录第二阶段第一版抓取任务，以及官方 LeRobot ACT 训练时
`httpx.ConnectError: [Errno 101] Network is unreachable` 的处理方法。

## ACT 网络报错

这个错误不是 ACT 模型本身的问题，而是 Python 进程无法访问 Hugging Face Hub。
LeRobot 在以下场景会访问 Hub：

- `dataset.repo_id` 指向远程数据集，且本地没有完整 `meta/`、`data/`、`videos/`。
- `policy.path` 指向远程模型仓库，例如 `lerobot/...`。
- `resume` 或 `config_path` 指向远程 checkpoint。

先确认网络：

```bash
cd /home/bot/research
source .venvs/lerobot/bin/activate

curl -I https://huggingface.co
```

如果机器需要代理：

```bash
export HTTPS_PROXY=http://127.0.0.1:7890
export HTTP_PROXY=http://127.0.0.1:7890
```

如果使用镜像站：

```bash
export HF_ENDPOINT=https://hf-mirror.com
```

如果离线训练，必须提前把数据集和模型下载到本地，然后把官方命令里的远程路径
换成本地路径：

```bash
huggingface-cli download <dataset_repo_id> \
  --repo-type dataset \
  --local-dir /home/bot/research/datasets/<dataset_name>

huggingface-cli download <policy_repo_id> \
  --local-dir /home/bot/research/models/<policy_name>
```

训练时使用：

```bash
lerobot-train \
  --dataset.repo_id=<dataset_repo_id> \
  --dataset.root=/home/bot/research/datasets/<dataset_name> \
  --policy.path=/home/bot/research/models/<policy_name>
```

注意：`dataset.root` 必须是包含 `meta/`、`data/`、`videos/` 的数据集根目录。
如果只下载了部分文件或目录层级不对，LeRobot 会再次尝试联网下载。

## 第一版抓取任务

新增文件：

```text
assets/so101/scene_pick_lift.xml
so101_tracking/pick_lift_env.py
scripts/smoke_pick_lift_env.py
scripts/train_pick_lift_sac.py
scripts/eval_pick_lift.py
```

任务定义：

- 任务：SO-101 抓取桌面方块并抬升。
- 方块：MuJoCo freejoint box，受重力、地面接触和摩擦影响。
- 观测：state-based，不使用图像。
- 默认动作：`[dx, dy, dz, gripper]`。
- 控制：RL 输出末端 delta，环境内部用 DLS IK 转为 5 个 arm 关节位置目标。
- 夹爪：第 6 个位置执行器。
- 默认使用 `virtual_grasp=True`，用于稳定第一版 reach-close-lift 流程。

## 官方抓取方法与本项目复用方式

LeRobot 官方抓取相关流程主要有两条：

1. 模仿学习：采集 teleoperation 演示数据，再用 ACT / Diffusion / VQ-BeT
   等策略做行为克隆。ACT 不是强化学习，它学习从观测到一段 action chunk
   的映射。
2. HIL-SERL：先有少量演示和/或 reward classifier，再用 actor / learner
   架构做 SAC，训练中可以通过人工介入修正策略；仿真教程用 `gym_hil`
   的 MuJoCo Franka Panda `PandaPickCube*` 环境验证这套流程。

不能直接把官方 Franka Panda pick cube checkpoint 用到 SO-101：

- 机器人自由度、URDF/MJCF、夹爪结构、工作空间不同。
- observation/action 维度不同。
- 官方 `gym_hil` 环境是 Franka Panda，不是 SO-101。

可以直接复用的是方法和代码结构：

- 使用末端 delta action + IK 映射到底层关节控制。
- 使用少量成功演示初始化策略，而不是纯随机探索。
- 使用 SAC 做在线微调。
- 使用 replay buffer 中的 offline demos + online transitions 混合训练。
- 后续如果要接 LeRobot 官方 ACT，需要把 SO-101 仿真 rollout 保存成
  LeRobotDataset 格式，再用 `lerobot-train --policy.type=act` 训练。

本项目当前已实现官方思路的最小版本：

- `SO101PickLiftEnv.scripted_action()` 生成成功示范轨迹。
- `train_pick_lift_sac.py --demo-prefill-episodes` 把示范 transitions 写入
  SAC replay buffer。
- `train_pick_lift_sac.py --bc-pretrain-steps` 用示范先做 actor 行为克隆
  warm start，再进入 SAC 在线训练。
- 默认 `--ent-coef auto_0.01`、`--gamma 0.97`、`--gradient-steps 2`，更接近
  官方 HIL-SERL 配置习惯。

为什么默认使用 virtual grasp：

- 当前 SO-101 CAD/MJCF 是学习运动控制的轻量模型，不是高保真双指接触抓取模型。
- 单纯依赖 mesh 接触很容易把训练难度变成“接触建模调参”，而不是学习抓取流程。
- 第一版目标是先跑通状态观测、奖励、训练、评估和可视化。
- 后续可以用 `--no-virtual-grasp` 切到纯接触实验。

## 常用命令

基础检查：

```bash
cd /home/bot/research
source .venvs/lerobot/bin/activate

python so101_mujoco_tracking/scripts/smoke_pick_lift_env.py
```

打开含方块的 MuJoCo 场景：

```bash
python so101_mujoco_tracking/scripts/view_so101.py --model pick_lift
```

短训练检查：

```bash
python so101_mujoco_tracking/scripts/train_pick_lift_sac.py \
  --total-timesteps 1000 \
  --demo-prefill-episodes 5 \
  --bc-pretrain-steps 200 \
  --eval-freq 500 \
  --save-freq 500 \
  --run-name demo_bc_smoke \
  --device cpu
```

推荐训练：

```bash
python so101_mujoco_tracking/scripts/train_pick_lift_sac.py \
  --total-timesteps 300000 \
  --control-mode ee_delta \
  --demo-prefill-episodes 50 \
  --bc-pretrain-steps 2000 \
  --demo-noise 0.03 \
  --run-name pick_lift_demo_bc_sac_v1
```

边看 MuJoCo 边训练：

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

评估最新模型：

```bash
python so101_mujoco_tracking/scripts/eval_pick_lift.py \
  --episodes 20 \
  --output-dir so101_mujoco_tracking/outputs/eval_pick_lift_v1
```

边看 MuJoCo 边评估：

```bash
python so101_mujoco_tracking/scripts/eval_pick_lift.py \
  --episodes 3 \
  --viewer \
  --real-time
```

TensorBoard：

```bash
tensorboard --logdir so101_mujoco_tracking/outputs/pick_lift_sac
```

重点看：

- `eval/mean_reward`：评估回报，整体应上升。
- `pick_lift/success`：是否成功抬升。
- `pick_lift/is_grasped`：是否学会闭合抓取。
- `pick_lift/cube_height`：方块高度。
- `pick_lift/reach_dist`：末端到方块距离，应下降。
- `pick_lift/hover_dist`：末端到方块上方预抓取点距离，应下降。
- `pick_lift/grasp_dist`：末端到闭合抓取点距离，应下降。
- `pick_lift/goal_dist`：方块到抬升目标距离，应下降。
- `pick_lift/is_closed_cmd`：策略是否在合适阶段闭合夹爪。

## 推荐训练顺序

1. 先跑 `smoke_pick_lift_env.py`，确认任务可完成。
2. 跑 `train_pick_lift_sac.py` 的 demo+BC smoke，确认预填充和训练管线正常。
3. 跑 `ee_delta + virtual_grasp + demo prefill + BC warm start` 的 300k 训练。
4. 评估 `success_rate`、平均 episode length、动作是否平滑；不要只看 reward。
5. 缩小 `cube_xy_range` 做容易任务；成功率稳定后再扩大随机范围。
6. 加 `--no-virtual-grasp` 做纯接触版本，只作为后续高保真方向，不作为第一版起点。

## 已验证结果

本地 smoke 验证：

```text
obs_shape=(34,)
action_space=Box(-1.0, 1.0, (4,), float32)
final_cube_height=0.1327 m
is_grasped=True
success=True
```

新增 demo+BC 验证：

```text
BC actor warm start complete: steps=1500, final_mse=0.006905
Demo prefill complete: episodes=30, transitions=4748, successes=30
bc_only_success_rate=1.0
```

这说明示范策略和 BC warm start 本身可用。后续 SAC 的目标是让策略从示范轨迹
泛化到更大随机范围、更低辅助、更真实接触，而不是从零学会抓取。
