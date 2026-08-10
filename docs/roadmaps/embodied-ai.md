---
title: 具身智能算法工程师学习路线
description: 面向 RTX 5090 单机环境的 10 个月具身智能学习与项目计划。
---

# 具身智能算法工程师学习路线

> 目标：用 10 个月建立“感知、规划、控制、学习、部署”完整能力，并完成 3 个可以面试演示的项目。

## 1. 先明确目标

具身智能不是一个单独算法，而是一条完整链路：

```text
传感器输入
  → 视觉/点云感知
  → 状态估计与世界模型
  → 任务规划与运动规划
  → 控制器执行
  → 数据回流
  → 模仿学习/强化学习/VLA 更新策略
```

学习顺序不能反过来。没有机器人学、视觉和控制基础，直接运行 VLA 只能得到一个演示程序，很难定位失败原因。

### 10 个月后的验收目标

- 能用 C++/Python 编写 ROS2 节点并排查通信、坐标系和时序问题。
- 能完成 RGB-D/点云处理、目标检测、位姿估计和多传感器融合。
- 能使用 MoveIt2 完成机械臂规划，并理解 IK、轨迹和控制器接口。
- 能在 Isaac Lab/MuJoCo 中训练强化学习策略并分析 reward 和 sim-to-real 风险。
- 能使用 LeRobot/Diffusion Policy 完成数据采集、模仿学习训练和闭环评估。
- 能理解 VLM/VLA 的模型结构，并在 5090 上完成 LoRA/QLoRA 微调或推理部署。
- 能使用 TensorRT/ONNX/CUDA 工具分析并优化推理性能。
- GitHub 上至少有 3 个带视频、指标、复现实验和技术文档的项目。

## 2. 5090 能做什么

RTX 5090 的价值主要在以下任务：

- 多相机视觉、3D 检测、分割和位姿估计训练。
- Isaac Lab 等 GPU 并行仿真。
- 3B-8B 视觉语言模型的量化推理和 LoRA/QLoRA 微调。
- Diffusion Policy、ACT 等模仿学习策略训练。
- CUDA、TensorRT、ONNX Runtime 性能优化。

不要把目标设为“从零训练一个机器人基础模型”。单卡更合理的路线是：使用公开预训练模型，建立自己的数据集、微调方法、评估流程和机器人闭环。

## 3. 环境配置

### 推荐系统

机器人开发优先使用原生 Ubuntu；如果当前主机必须保留 Windows，可以先使用 WSL2 做深度学习，但真实 USB、相机、CAN 和实时控制建议在原生 Ubuntu 上完成。

推荐组合：

| 用途 | 推荐方案 |
| --- | --- |
| 深度学习 | Ubuntu 24.04 / WSL2 + 最新稳定 NVIDIA 驱动 |
| ROS2 | Jazzy（Ubuntu 24.04）或 Humble（Ubuntu 22.04） |
| Python 环境 | Miniforge/Conda 或 uv |
| 容器 | Docker + NVIDIA Container Toolkit |
| 仿真 | Isaac Sim/Isaac Lab、MuJoCo、Gazebo |
| 训练框架 | PyTorch、Transformers、LeRobot |

5090 属于较新的 GPU，不要照搬旧教程固定安装老 CUDA。先根据 PyTorch/Isaac 官方兼容矩阵选择驱动和 CUDA 版本。

### 环境自检

```bash
nvidia-smi
python -c "import torch; print(torch.__version__, torch.cuda.is_available()); print(torch.cuda.get_device_name())"
```

建立独立环境：

```bash
conda create -n embodied python=3.11 -y
conda activate embodied
pip install torch torchvision torchaudio
```

验收：矩阵乘法能够在 GPU 上执行，训练时显存占用和功耗能够在 `nvidia-smi` 中观察到。

## 4. 总体时间表

假设每周投入 15-20 小时：

| 阶段 | 时间 | 核心成果 |
| --- | ---: | --- |
| 0. 数学与工程基础 | 4 周 | PyTorch + ROS2 + 机器人学最小闭环 |
| 1. 3D 视觉与感知 | 6 周 | RGB-D 位姿估计项目 |
| 2. SLAM、导航与融合 | 6 周 | 可量化评估的导航系统 |
| 3. 机械臂规划与控制 | 6 周 | 仿真抓取与 MoveIt2 项目 |
| 4. 强化学习与 Sim2Real | 8 周 | Isaac Lab 控制策略 |
| 5. 模仿学习与 VLA | 8 周 | 数据采集、策略训练与 VLA 微调 |
| 6. CUDA 与部署 | 4 周 | TensorRT 部署与性能报告 |
| 7. 综合项目与求职 | 4 周 | 作品集、演示视频、技术文章 |

## 5. 阶段 0：数学与工程基础（第 1-4 周）

### 必须掌握

- 线性代数：矩阵、特征值、SVD、旋转矩阵、四元数、SE(3)。
- 概率统计：高斯分布、贝叶斯、最大似然、交叉熵、KL 散度。
- 优化：梯度下降、Adam、约束优化、最小二乘。
- 机器人学：正运动学、逆运动学、雅可比、PID、轨迹插值。
- 工程：Linux、Git、CMake、Docker、Python、现代 C++。

### 每周任务

#### 第 1 周：PyTorch

- 手写线性回归和两层 MLP。
- 理解 Dataset、DataLoader、训练/验证集、checkpoint。
- 学会 TensorBoard 或 Weights & Biases 记录实验。

验收：同一实验固定随机种子后结果可复现，能够解释过拟合和欠拟合。

#### 第 2 周：ROS2

- 节点、topic、service、action、parameter、launch。
- TF2 坐标变换和时间戳。
- rosbag2 录制、回放与调试。

验收：发布模拟里程计和激光数据，在 RViz2 中正确显示 TF 树。

#### 第 3 周：机器人学

- 用 NumPy 实现二维/三维坐标变换。
- 实现二自由度机械臂正逆运动学。
- 实现 PID 并分析超调、稳态误差和噪声影响。

#### 第 4 周：工程化

- 为一个 ROS2 包配置格式化、静态检查和单元测试。
- 使用 Docker 固化依赖。
- 写 README：安装、运行、输入输出、已知问题。

## 6. 阶段 1：3D 视觉、点云与位姿估计（第 5-10 周）

### 学习内容

- 相机模型、内外参、畸变、双目和 RGB-D。
- OpenCV：特征、PnP、标定、图像几何。
- 点云：滤波、法线、ICP、聚类、RANSAC。
- 深度模型：检测、分割、深度估计、6D Pose。
- 指标：Precision/Recall、mAP、ADD/ADD-S、旋转和平移误差。

### 工具

- OpenCV、Open3D、PCL。
- PyTorch、Ultralytics/Detectron2（任选一个）。
- MMDetection3D 或同类 3D 感知框架。

### 项目 A：RGB-D 目标定位

```text
RGB-D 输入 → 目标检测/分割 → 深度过滤 → 点云生成
→ 坐标系转换 → 物体 3D 位置/姿态 → ROS2 发布
```

必须提交：

- 相机标定结果和误差。
- 录制数据集与数据说明。
- 检测精度、位置误差、推理延迟。
- RViz2 可视化和演示视频。
- 错误案例：反光、遮挡、深度孔洞、运动模糊。

验收建议：固定 5-10 个测试位置，统计位置误差的均值、P95 和最大值，而不是只展示一次成功结果。

## 7. 阶段 2：SLAM、导航与感知融合（第 11-16 周）

### 学习内容

- EKF/UKF、IMU 预积分、轮速里程计。
- 前端匹配、后端优化、回环检测。
- 栅格地图、代价地图、A*、Dijkstra、Hybrid A*。
- 局部规划、轨迹跟踪、碰撞检测。
- Nav2 生命周期、行为树、恢复行为。

### 工具

- ROS2 Nav2。
- robot_localization。
- SLAM Toolbox、Cartographer、RTAB-Map 中任选一个深入。
- GTSAM/Ceres 了解基本使用。

### 项目 B：可评估的移动机器人导航

- 在 Gazebo/Isaac Sim 中搭建移动机器人。
- 融合 IMU、轮速和激光里程计。
- 自动建图、定位、全局规划和局部避障。
- 编写自动测试脚本，随机生成起点终点。

指标：

- 20 次导航成功率。
- 平均路径长度与规划时间。
- 定位漂移和回环前后误差。
- 动态障碍下的最小安全距离。

## 8. 阶段 3：机械臂运动规划与控制（第 17-22 周）

### 学习内容

- DH 参数、URDF、正逆运动学、雅可比。
- 关节空间和笛卡尔空间轨迹。
- 碰撞检测、采样规划、轨迹优化。
- PID、前馈、阻抗控制的基本思想。
- MoveIt2、ros2_control、控制器接口。

### 项目 C：机械臂视觉抓取

```text
RGB-D 感知 → 目标位姿 → TF 变换 → 抓取候选
→ MoveIt2 规划 → 轨迹执行 → 抓取结果评估
```

先在仿真完成，再接真实机械臂。没有真实机械臂时也要保留硬件接口抽象和 rosbag 回放模式。

验收：至少 5 类物体、每类 20 次尝试，给出抓取成功率和主要失败原因。

## 9. 阶段 4：强化学习、模仿学习与 Sim2Real（第 23-30 周）

### 强化学习顺序

1. 多臂老虎机和 Bellman 方程。
2. DQN：理解 replay buffer 和 target network。
3. PPO：理解 advantage、clip、on-policy。
4. SAC：理解 entropy 和 off-policy。
5. 连续控制与并行环境。

不要一开始写算法框架。先使用 Stable-Baselines3 验证概念，再阅读 CleanRL 的单文件实现，最后进入 Isaac Lab。

### Isaac Lab 项目

- 训练机械臂 reaching 或抓取策略，或移动机器人跟踪策略。
- 逐步设计 observation、action、reward、termination。
- 记录 reward 曲线、成功率和训练吞吐。
- 做消融实验：去掉一项 reward 或观测，比较结果。

### Sim2Real 必须理解

- Domain randomization：质量、摩擦、延迟、噪声、纹理、光照。
- System identification：根据真实数据估计动力学参数。
- Action delay、观测延迟、控制频率不一致。
- 安全约束、动作限幅和异常停止。

### 模仿学习

- Behavior Cloning。
- DAgger。
- ACT。
- Diffusion Policy。

使用 LeRobot 或 Diffusion Policy 官方实现完成：数据采集、数据检查、训练、离线评估、闭环评估。

## 10. 阶段 5：多模态模型、VLA 与机器人基础模型（第 31-38 周）

### 理论顺序

- Transformer、attention、位置编码。
- ViT、CLIP、视觉编码器。
- LLM tokenization、指令微调、LoRA/QLoRA。
- VLM：图像和文本如何对齐。
- VLA：视觉、语言、机器人状态如何映射为动作。

### 推荐实践层次

#### 层次 1：理解输入输出

- 使用一个开源 VLM 对机器人场景做描述、目标识别和任务分解。
- 分析它在空间关系、左右方向、遮挡和数量判断上的错误。

#### 层次 2：小规模微调

- 使用 LoRA 微调视觉语言模型。
- 训练数据必须包含训练/验证/测试划分。
- 对比零样本、提示词、LoRA 三种效果。

#### 层次 3：VLA/策略模型

- 使用 LeRobot、OpenVLA、Octo 或同类公开实现中的一个。
- 先跑通公开数据集和预训练 checkpoint。
- 再使用自己的遥操作数据进行微调。
- 明确 action 表示：关节位置、速度、末端位姿或 action chunk。

### 5090 训练策略

- 默认使用 BF16；显存不足时使用 8-bit/4-bit 量化和 LoRA。
- 开启 gradient checkpointing、gradient accumulation。
- 先用 1%-5% 数据跑通完整流程，再扩大数据。
- 每次只改变一个变量，记录配置、显存、吞吐和指标。

### VLA 项目验收

- 至少 3 类语言指令。
- 训练集未出现的物体位置或背景。
- 统计任务成功率而不是只展示最佳视频。
- 比较纯 BC、Diffusion Policy/VLA 的差异。
- 记录失败分类：感知、语言理解、规划、控制、数据覆盖。

## 11. 阶段 6：CUDA、TensorRT 与部署（第 39-42 周）

### 学习顺序

- GPU 执行模型、warp、memory hierarchy。
- CUDA kernel、内存合并访问、shared memory。
- PyTorch Profiler、Nsight Systems、Nsight Compute。
- ONNX 导出和算子兼容。
- TensorRT FP16/INT8、动态 shape、engine 构建。

### 部署项目

选择项目 A 的感知模型：

1. PyTorch eager 基线。
2. `torch.compile` 基线。
3. ONNX Runtime CUDA。
4. TensorRT FP16。
5. 有条件再做 INT8 校准。

报告必须包含：

| 指标 | PyTorch | ONNX Runtime | TensorRT |
| --- | ---: | ---: | ---: |
| 平均延迟 |  |  |  |
| P95 延迟 |  |  |  |
| 显存 |  |  |  |
| 功耗 |  |  |  |
| 精度变化 |  |  |  |

优化目标不是“用了 TensorRT”，而是在精度可接受的前提下证明延迟和吞吐改善。

## 12. 每周执行模板

### 时间分配

| 任务 | 每周时间 |
| --- | ---: |
| 理论学习 | 4 小时 |
| 编程与实验 | 8 小时 |
| 阅读代码/论文 | 2 小时 |
| 文档、复盘和博客 | 2 小时 |

### 周一

- 写下本周唯一主要目标。
- 定义验收指标。
- 建立 issue 和实验分支。

### 周二至周四

- 每天完成一个可运行的小增量。
- 记录命令、环境和失败原因。
- 不在未记录 baseline 前开始“优化”。

### 周五

- 跑固定测试集。
- 保存日志、图表、模型和配置。
- 把失败样本单独分类。

### 周末

- 更新 README 和技术博客。
- 录制 1-3 分钟演示视频。
- 列出下周只解决的 1-2 个问题。

## 13. 前 90 天具体计划

### 第 1 个月

- 第 1 周：PyTorch 训练模板、实验记录。
- 第 2 周：ROS2 topic/service/action/TF/rosbag。
- 第 3 周：SE(3)、四元数、正逆运动学、PID。
- 第 4 周：Docker、CMake、测试、项目模板。

输出：`robotics-foundation` 仓库和一篇环境搭建文章。

### 第 2 个月

- 相机标定、OpenCV PnP。
- Open3D/PCL 点云处理。
- 训练一个检测或分割模型。
- 将视觉结果通过 ROS2 发布并在 RViz2 中显示。

输出：RGB-D 目标定位项目第一版。

### 第 3 个月

- 完成数据集划分和误差评估。
- 加入 TF、时间同步和 rosbag 回放。
- TensorRT FP16 初步部署。
- 写项目文档、失败分析和演示视频。

输出：第一个可用于简历的完整项目。

## 14. 作品集标准

每个项目仓库必须包含：

```text
README.md
docs/architecture.md
configs/
scripts/
src/
tests/
assets/demo.mp4
results/metrics.csv
Dockerfile 或 environment.yml
```

README 必须回答：

1. 解决了什么问题？
2. 系统输入和输出是什么？
3. 如何一条命令复现？
4. 使用什么数据和指标？
5. baseline 是什么？
6. 当前结果和失败原因是什么？
7. 下一步如何改进？

## 15. 学习资源优先级

优先阅读官方资料，版本最可靠：

- ROS2、Nav2、MoveIt2、ros2_control 官方教程。
- PyTorch、Hugging Face Transformers、PEFT 官方文档。
- NVIDIA CUDA、TensorRT、Isaac Sim、Isaac Lab 官方文档。
- OpenCV、Open3D、PCL 官方教程。
- LeRobot、Diffusion Policy、OpenVLA、Octo 的论文和官方仓库。
- 《Probabilistic Robotics》。
- 《Modern Robotics》。
- Sutton & Barto《Reinforcement Learning》。

论文阅读顺序：先读摘要、问题定义、方法图、实验指标和失败案例；确认与当前项目直接相关后再逐公式精读。

## 16. 常见误区

- 同时学习人形控制、VLA、SLAM、CUDA，结果没有一个完整项目。
- 只跑官方 demo，不更换数据、不做指标、不分析失败。
- 把“会使用 ROS2”当作算法能力。
- 只看 reward 曲线，不做独立测试和消融实验。
- 训练规模越大越好，却没有固定 baseline 和数据质量检查。
- 简历写“熟悉”十几个框架，但 GitHub 没有可复现代码。

## 17. 最终主线选择

完成前 6 个月后，根据兴趣选择一个主线，其他方向保持能协作的程度：

### 主线 A：VLA/模仿学习

重点：Transformers、VLM、机器人数据、ACT、Diffusion Policy、OpenVLA、LoRA。

### 主线 B：运动规划与控制

重点：机器人学、MoveIt2、优化控制、强化学习、Isaac Lab、Sim2Real。

### 主线 C：3D 感知与导航

重点：点云、位姿估计、多传感器融合、SLAM、Nav2、TensorRT。

对于本科阶段，建议先以 **3D 感知与导航** 或 **机械臂规划与控制** 建立扎实工程闭环，再进入 VLA。这样更容易获得实习，也能在 VLA 项目失败时准确定位问题。

## 18. 今日开始

今天只做以下四件事：

- [ ] 确认 Ubuntu/WSL2、驱动、PyTorch 和 5090 工作正常。
- [ ] 创建 `embodied-ai-learning` GitHub 仓库。
- [ ] 完成一个 PyTorch GPU 训练模板并记录显存、速度。
- [ ] 建立第 1 周 issue：ROS2 + PyTorch + 实验记录。

不要先下载十个大模型。先建立可复现的训练环境和实验纪律，这是后续所有高薪方向的共同基础。
