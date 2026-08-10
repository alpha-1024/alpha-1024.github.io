---
title: 第一册实验、习题与答案
description: 数学、PyTorch、Linux、C++ 与 ROS2 的动手实验和自测答案。
---

# 第一册实验、习题与答案

## 实验 1：坐标变换库

### 目标

实现并测试旋转矩阵、齐次变换、逆变换和点变换。

### 要求

```python
import numpy as np

def is_rotation_matrix(R):
    """检查形状、正交性和行列式。"""
    pass

def make_transform(R, t):
    pass

def inverse_transform(T):
    pass

def transform_points(T, points):
    """points 形状为 [N, 3]。"""
    pass
```

必须测试：单位变换、90 度旋转、组合变换、逆变换、非法旋转矩阵和空点云。

### 参考实现

```python
def is_rotation_matrix(R, atol=1e-6):
    return (
        R.shape == (3, 3)
        and np.allclose(R.T @ R, np.eye(3), atol=atol)
        and np.isclose(np.linalg.det(R), 1.0, atol=atol)
    )

def make_transform(R, t):
    if not is_rotation_matrix(R):
        raise ValueError('R is not a valid rotation matrix')
    t = np.asarray(t, dtype=float).reshape(3)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

def inverse_transform(T):
    R = T[:3, :3]
    t = T[:3, 3]
    result = np.eye(4)
    result[:3, :3] = R.T
    result[:3, 3] = -R.T @ t
    return result

def transform_points(T, points):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('points must have shape [N, 3]')
    return points @ T[:3, :3].T + T[:3, 3]
```

### 思考

为什么逆变换平移不是简单的 $-t$？因为平移向量也必须转换到逆变换对应的坐标系，所以是 $-R^Tt$。

## 实验 2：可复现的 PyTorch 训练模板

### 目标

训练一个小型分类网络，并保证配置、日志和 checkpoint 可追溯。

### 目录

```text
classification-baseline/
  configs/base.yaml
  src/data.py
  src/model.py
  src/train.py
  src/evaluate.py
  tests/test_model.py
  outputs/
  requirements.txt
  README.md
```

### 必做功能

- 配置文件控制 batch size、学习率、epoch、seed。
- 自动选择 CUDA。
- 保存 best 和 last checkpoint。
- 每个 epoch 输出训练/验证损失与准确率。
- 训练结束后在独立测试集评估。
- 保存混淆矩阵。

### 验收问题

1. 把训练标签随机打乱，模型还能达到高准确率吗？如果能，数据管线可能泄漏。
2. 只用 20 个样本，模型能否过拟合到接近 100%？如果不能，训练实现可能有错误。
3. 重复运行三次，结果方差多大？
4. 把 `model.eval()` 删除，验证结果如何变化？

## 实验 3：ROS2 发布订阅与 QoS

创建发布端和订阅端。发布端以 30Hz 发布带序号的消息，订阅端统计：收到数量、丢失数量、平均间隔和最大间隔。

分别测试：

- Reliable → Reliable。
- Best Effort → Best Effort。
- 发布 Best Effort、订阅 Reliable。
- 人为让订阅回调休眠 100ms。

记录每种组合能否建立连接、是否丢消息以及队列深度的影响。

结论不应写成“Reliable 一定更好”。图像高频流如果处理速度跟不上，可靠传输可能积压旧数据，使机器人基于过时图像决策。

## 实验 4：TF2 坐标树

建立：

```text
map → odom → base_link → camera_link → optical_frame
```

要求：

- `base_link → camera_link` 使用静态变换。
- `odom → base_link` 随时间变化。
- 在 `optical_frame` 创建一点，转换到 `map`。
- 故意查询未来时间并观察异常。
- 使用 `view_frames` 导出坐标树。

验收：能够解释 `map → odom` 和 `odom → base_link` 为什么通常分开，而不是直接发布 `map → base_link`。

## 实验 5：rosbag 回归测试

录制 60 秒传感器数据。编写脚本回放 bag 并运行处理节点，将输出保存为 CSV。

每次修改算法后自动比较：

- 输出数量。
- 平均处理延迟。
- 数值均值、标准差、最大值。
- 是否出现 NaN/Inf。

这就是最小的机器人算法回归测试。真实硬件不在身边时，仍能验证代码没有破坏已有行为。

## 理论检查题

### 题 1

$T_{AB}$ 和 $T_{BC}$ 已知，如何得到 $T_{AC}$？

**答案**：$T_{AC}=T_{AB}T_{BC}$。变换从右向左作用，点先从 C 转到 B，再从 B 转到 A。

### 题 2

为什么不能逐元素平均两个旋转矩阵？

**答案**：逐元素平均通常不再满足正交性和行列式为 1，因此不属于 SO(3)。旋转平均应使用四元数、李群方法或先平均后投影回 SO(3)。

### 题 3

协方差矩阵对角线很小意味着什么？

**答案**：算法声明对应维度的不确定性很小，但不保证估计真的准确。错误的噪声模型可能产生“自信但错误”的结果。

### 题 4

训练损失下降而验证损失上升是什么现象？

**答案**：典型过拟合。应检查数据划分、增强、正则化、模型容量和训练时长，而不是只看训练指标。

### 题 5

为什么验证阶段要调用 `model.eval()`？

**答案**：它让 Dropout、BatchNorm 等模块切换到推理行为。它不等于关闭梯度，因此通常还要使用 `torch.inference_mode()`。

### 题 6

ROS2 节点存在，但 `ros2 topic echo` 没有数据，应先查什么？

**答案**：节点状态、Topic 名称和类型、发布频率、QoS 兼容性，再检查回调和算法。

### 题 7

为什么传感器消息不能都在接收时填写当前时间？

**答案**：接收时间包含传输和排队延迟，不能代表采集时刻，会破坏多传感器同步和 TF 查询。

### 题 8

Git 仓库中误提交私钥后，删除文件并再次提交是否足够？

**答案**：不够。私钥仍存在于历史提交且可能已被复制。必须撤销/更换密钥，并根据需要清理仓库历史。

## 阶段项目评分表

| 项目 | 分值 | 验收标准 |
| --- | ---: | --- |
| 功能闭环 | 20 | 数据发布、处理、TF 和记录全部工作 |
| 正确性 | 20 | 有单元测试，坐标和时间处理正确 |
| 稳定性 | 15 | 连续运行 5 分钟，无崩溃和持续积压 |
| 可复现 | 15 | 新环境按 README 可以运行 |
| 可观测 | 10 | 有日志、频率、延迟和异常统计 |
| 文档 | 10 | 架构图、Topic 表、参数和已知问题完整 |
| 实验纪律 | 10 | 配置、Git commit、bag 和结果对应 |

达到 80 分再进入第二册。低于 80 分时，继续堆新算法只会把基础问题带到更复杂系统中。

## 学习日志模板

```md
# YYYY-MM-DD 学习记录

## 今日目标

## 环境与版本

## 完成内容

## 实验配置

## 结果与指标

## 失败现象

## 原因分析

## 明日唯一目标
```

坚持记录失败原因，比只保存成功截图更能形成工程能力。
