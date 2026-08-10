---
title: 术语表与毕业清单
description: 具身智能常用术语、能力检查和综合项目交付标准。
---

# 术语表与毕业清单

## 数学与估计

- **SO(3)**：三维旋转矩阵构成的特殊正交群。
- **SE(3)**：三维刚体旋转和平移构成的特殊欧氏群。
- **Jacobian**：输入微小变化到输出微小变化的一阶线性映射。
- **Covariance**：随机变量不确定性和相关性的矩阵表示。
- **Information Matrix**：协方差逆，图优化中表示约束强度。
- **Innovation**：测量与预测测量的差。
- **Observability**：能否从输入输出唯一推断内部状态。
- **RMSE**：均方根误差，对较大误差更敏感。
- **P95**：95% 样本不超过的值，用于描述尾延迟或误差。

## 机器人系统

- **URDF**：机器人连杆、关节、几何、惯量描述。
- **SRDF**：MoveIt 语义模型，如规划组和允许碰撞。
- **TF2**：ROS2 中带时间的坐标变换系统。
- **Odometry**：由运动积分得到的局部连续位姿，通常会漂移。
- **SLAM**：同时定位与建图。
- **Loop Closure**：识别历史地点并增加全局约束。
- **Costmap**：导航中表达障碍和通行代价的栅格。
- **Footprint**：机器人在平面上的碰撞轮廓。
- **IK**：根据末端目标求关节状态。
- **Impedance Control**：控制末端力与位移关系，使机器人表现为弹簧阻尼系统。

## 学习算法

- **MDP**：状态、动作、转移、奖励和折扣构成的决策模型。
- **On-policy**：主要使用当前策略采集的数据更新。
- **Off-policy**：能够复用其他或历史策略数据。
- **Advantage**：一个动作相对当前状态平均行为的价值。
- **Replay Buffer**：保存交互数据供重复采样。
- **Behavior Cloning**：从专家状态动作对进行监督学习。
- **Covariate Shift**：执行状态分布偏离训练数据导致误差累积。
- **Action Chunk**：一次预测未来多个动作。
- **Domain Randomization**：随机仿真参数提升现实适应性。
- **System Identification**：用真实数据估计系统参数。

## 多模态与 VLA

- **VLM**：视觉语言模型，联合处理图像和文本。
- **VLA**：视觉语言动作模型，从视觉、指令和状态生成动作。
- **LoRA**：用低秩参数更新适配冻结模型。
- **QLoRA**：量化基座模型后进行 LoRA 微调。
- **Token**：模型处理的离散或连续表示单元。
- **Embodiment**：机器人的形态、传感器和动作空间。
- **Action Tokenization**：把连续动作量化或编码成 token。
- **Zero-shot**：不针对目标任务训练直接推理。
- **Fine-tuning**：在目标数据上继续训练预训练模型。

## 部署

- **Latency**：单次请求耗时。
- **Throughput**：单位时间处理量。
- **Warmup**：正式计时前运行以完成初始化。
- **ONNX**：模型交换图表示。
- **TensorRT**：NVIDIA 推理优化和执行引擎。
- **FP16/BF16**：低精度浮点格式。
- **INT8 PTQ**：训练后使用校准数据做 8 位量化。
- **QAT**：训练中模拟量化误差。
- **Kernel**：GPU 上执行的函数。
- **Warp**：NVIDIA GPU 同步执行的一组线程。
- **Occupancy**：SM 上活跃 warp 与硬件容量的关系。

## 毕业能力自检

### 数学

- [ ] 能推导并实现 SE(3) 变换和逆变换。
- [ ] 能解释协方差、雅可比和最小二乘。
- [ ] 能用有限差分检查解析雅可比。

### ROS2

- [ ] 能设计 topic/service/action 接口。
- [ ] 能解释 QoS 和时间戳问题。
- [ ] 能从 TF、频率、日志和 rosbag 定位故障。

### 感知

- [ ] 能完成相机标定和误差分析。
- [ ] 能把 RGB-D 转点云并处理单位/坐标系。
- [ ] 能完成检测、位姿和 ROS2 发布闭环。

### 导航

- [ ] 能解释 map/odom/base_link。
- [ ] 能实现 KF 和 A* 基线。
- [ ] 能配置 Nav2 并做 20 次自动评估。

### 操作

- [ ] 能实现 FK、IK 和雅可比检查。
- [ ] 能用 MoveIt2 规划并通过 ros2_control 执行。
- [ ] 能分层统计抓取失败。

### 学习

- [ ] 能解释 PPO、SAC 和 BC 的数据差异。
- [ ] 能设计 reward 并做消融。
- [ ] 能在 Isaac Lab 训练并进行 domain randomization。
- [ ] 能训练 ACT/Diffusion Policy 或微调 VLA。

### 部署

- [ ] 能正确测量 CUDA 延迟。
- [ ] 能使用 profiler 找瓶颈。
- [ ] 能导出 ONNX、构建 TensorRT 并验证精度。
- [ ] 能让 ROS2 推理节点处理超时和断流。

## 毕业项目仓库

```text
embodied-capstone/
  README.md
  LICENSE
  docker/
  configs/
  ros2_ws/src/
  perception/
  navigation/
  manipulation/
  policies/
  deployment/
  tests/
  scripts/
  datasets/README.md
  results/
  docs/
    architecture.md
    experiments.md
    safety.md
    failure-analysis.md
```

## 面试讲解顺序

1. 用一句话说明任务和约束。
2. 画数据流和坐标系。
3. 给出 baseline。
4. 说明你负责的模块和关键决策。
5. 展示量化结果，不只播放视频。
6. 展示一个失败案例和定位过程。
7. 说明性能、安全和可复现设计。
8. 说明下一步改进及其代价。

面试官更关心你如何证明系统有效、如何处理失败，而不是背了多少模型名称。
