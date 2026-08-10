---
title: 第一册：数学与工程基础
description: 具身智能所需的数学、PyTorch、Linux、C++ 与 ROS2 基础。
---

# 第一册：数学与工程基础

## 第 1 章：什么是具身智能系统

具身智能强调智能体通过身体与环境交互。它与只处理文本或图片的模型不同：机器人每一次输出都会改变世界，新的世界状态又会影响下一次输入。一个错误分类可能只是标签错了，而一个错误动作可能损坏设备或伤害人员。因此，具身智能算法必须同时考虑感知误差、动作约束、时间延迟和安全边界。

一个典型系统包括六层：

1. **传感器层**：相机、激光雷达、IMU、编码器、力传感器。
2. **感知层**：目标检测、分割、深度、位姿、场景理解。
3. **状态估计层**：机器人位姿、速度、地图和物体状态。
4. **规划层**：任务规划、路径规划、抓取规划和轨迹生成。
5. **控制层**：把轨迹变成电机能够执行的命令。
6. **学习层**：从数据或交互中改进感知、规划和策略。

工程中最重要的不是某个模型的榜单成绩，而是接口是否定义清楚。例如感知节点输出“物体位置”时，必须说明坐标系、单位、时间戳、置信度和协方差。若只输出三个数字，下游无法判断它们属于相机坐标系还是世界坐标系，也无法判断数据是否已经过时。

### 闭环与开环

开环系统生成一次动作后不再观察结果；闭环系统持续测量误差并修正动作。机器人通常需要闭环：机械臂抓取时，目标可能移动；移动机器人导航时，轮胎会打滑；相机估计也存在噪声。

设目标状态为 $x^*$，实际状态为 $x_t$，误差为：

$$e_t = x^* - x_t$$

最简单的比例控制器为：

$$u_t = K_p e_t$$

这里的核心不是背公式，而是理解：控制输入由当前误差决定。$K_p$ 太小，响应慢；太大，系统可能振荡。后续学习强化学习和 VLA 时也要保留这个视角：策略输出动作，但环境反馈决定下一步。

### 本章检查

- 为什么具身智能比离线图像分类更强调安全？
- 一个 ROS2 位姿消息至少应包含哪些信息？
- 开环抓取在什么情况下会失败？

## 第 2 章：向量、矩阵与坐标变换

### 2.1 向量不是一组无意义的数字

向量必须属于某个坐标系。点 $p=[1,0,0]^T$ 在相机坐标系表示相机前方或右方，取决于坐标轴约定；同样数字在 ROS 的 `base_link` 中可能表示机器人前方一米。任何时候看到向量，都要问三个问题：

1. 它表示点、方向、速度还是力？
2. 它属于哪个坐标系？
3. 单位是什么？

向量点积：

$$a \cdot b = \|a\|\|b\|\cos\theta$$

可用于求夹角和投影。叉积 $a\times b$ 得到垂直于两向量的方向，在计算平面法线、角速度和力矩时频繁出现。

### 2.2 旋转矩阵

三维旋转矩阵 $R$ 满足：

$$R^TR=I,\quad \det(R)=1$$

$R^{-1}=R^T$。如果数值计算后 $R^TR$ 明显不等于单位阵，说明旋转矩阵已经受到误差污染。绕 $z$ 轴旋转角度 $\theta$：

$$
R_z(\theta)=
\begin{bmatrix}
\cos\theta & -\sin\theta & 0\\
\sin\theta & \cos\theta & 0\\
0 & 0 & 1
\end{bmatrix}
$$

注意矩阵乘法不可交换。先旋转再平移和先平移再旋转通常得到不同结果。

### 2.3 齐次变换与 SE(3)

刚体变换写成：

$$
T=
\begin{bmatrix}
R&t\\
0&1
\end{bmatrix}
$$

其中 $R$ 是旋转，$t$ 是平移。把坐标系 B 中的点转换到 A：

$$p_A=T_{AB}p_B$$

下标阅读法非常重要：$T_{AB}$ 表示“B 到 A”的变换。两个变换组合：

$$T_{AC}=T_{AB}T_{BC}$$

中间下标 B 被消去，这是一种快速检查乘法顺序的方法。

### 2.4 四元数

四元数避免欧拉角的万向节锁，但它不是“旋转的四个随意参数”。单位四元数必须满足：

$$q_w^2+q_x^2+q_y^2+q_z^2=1$$

$q$ 与 $-q$ 表示同一旋转，所以比较四元数时不能只做逐元素相减。插值通常使用 SLERP。ROS 消息顺序常为 `x,y,z,w`，某些数学库使用 `w,x,y,z`，接口转换时必须确认。

### 2.5 NumPy 实现

```python
import numpy as np

def transform(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

theta = np.deg2rad(90.0)
R_ab = np.array([
    [np.cos(theta), -np.sin(theta), 0.0],
    [np.sin(theta),  np.cos(theta), 0.0],
    [0.0,            0.0,           1.0],
])
t_ab = np.array([1.0, 0.0, 0.0])
T_ab = transform(R_ab, t_ab)

p_b = np.array([1.0, 0.0, 0.0, 1.0])
p_a = T_ab @ p_b
print(p_a)
print(np.allclose(R_ab.T @ R_ab, np.eye(3)))
```

练习时不要只打印结果。先手算，再用程序验证，并加入非法矩阵测试。

## 第 3 章：概率、估计与优化

### 3.1 为什么机器人离不开概率

传感器都有噪声。同一物体静止不动，相机测出的深度仍会波动；IMU 即使静止也会有零偏。确定性地相信单次测量会让机器人抖动或错误决策。

随机变量的均值描述中心趋势，方差描述不确定性：

$$\mu=E[x],\quad \sigma^2=E[(x-\mu)^2]$$

多维变量使用协方差矩阵 $\Sigma$。对角线是各维方差，非对角线表示维度相关性。协方差不是越小越好；虚假地报告很小协方差会让融合算法过度相信错误传感器。

### 3.2 贝叶斯公式

$$P(x|z)=\frac{P(z|x)P(x)}{P(z)}$$

- $P(x)$：看到测量前的先验。
- $P(z|x)$：状态为 $x$ 时出现测量 $z$ 的可能性。
- $P(x|z)$：结合测量后的后验。

定位问题中，运动模型产生先验，传感器测量更新后验。贝叶斯思想贯穿卡尔曼滤波、粒子滤波、SLAM 和许多学习算法。

### 3.3 最小二乘

给定残差 $r_i(\theta)$，寻找参数：

$$\theta^*=\arg\min_\theta \sum_i \|r_i(\theta)\|^2$$

相机标定、PnP、ICP、图优化都可以写成残差最小化。若存在离群点，平方损失会放大其影响，因此常用 Huber、Cauchy 等鲁棒核。

### 3.4 梯度下降

$$\theta_{k+1}=\theta_k-\eta\nabla_\theta L(\theta_k)$$

学习率 $\eta$ 太大可能发散，太小收敛缓慢。优化失败时不要第一反应换模型，应先检查：数据是否归一化、损失是否有限、梯度是否为零或爆炸、标签是否正确。

### 3.5 一个可视化例子

```python
import torch

x = torch.linspace(-2, 2, 200).unsqueeze(1)
y = 3.0 * x + 0.5 + 0.1 * torch.randn_like(x)

model = torch.nn.Linear(1, 1)
optimizer = torch.optim.Adam(model.parameters(), lr=1e-2)

for step in range(1000):
    pred = model(x)
    loss = torch.mean((pred - y) ** 2)
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()
    if step % 100 == 0:
        print(step, float(loss))

print(model.weight.item(), model.bias.item())
```

把噪声扩大、减少样本、加入离群点，观察参数变化。实验的价值来自比较，而不是成功运行一次。

## 第 4 章：PyTorch 与可复现实验

### 4.1 张量与设备

Tensor 同时包含数据、形状、类型和设备。常见错误包括：模型在 GPU、数据在 CPU；标签是整数而损失要求浮点；图像维度写成 HWC 而模型需要 CHW。

```python
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
x = torch.randn(32, 3, 224, 224, device=device)
print(x.shape, x.dtype, x.device)
```

5090 支持高效的低精度计算，训练通常优先尝试 BF16。低精度不是无条件更好，损失异常时要回退 FP32 定位问题。

### 4.2 Dataset 与 DataLoader

数据管线应独立于模型。Dataset 负责读取一个样本，DataLoader 负责批处理、打乱和并行加载。训练集可使用随机增强，验证集必须确定性处理。

```python
from torch.utils.data import Dataset, DataLoader

class PairDataset(Dataset):
    def __init__(self, xs, ys):
        self.xs, self.ys = xs, ys
    def __len__(self):
        return len(self.xs)
    def __getitem__(self, index):
        return self.xs[index], self.ys[index]

loader = DataLoader(PairDataset(x, y), batch_size=32,
                    shuffle=True, num_workers=0)
```

Windows/WSL 初学阶段先用 `num_workers=0` 排除多进程问题，再逐步增加。

### 4.3 标准训练循环

```python
def train_one_epoch(model, loader, optimizer, loss_fn, device):
    model.train()
    total_loss = 0.0
    for inputs, targets in loader:
        inputs = inputs.to(device)
        targets = targets.to(device)
        optimizer.zero_grad(set_to_none=True)
        outputs = model(inputs)
        loss = loss_fn(outputs, targets)
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
        optimizer.step()
        total_loss += loss.item() * inputs.size(0)
    return total_loss / len(loader.dataset)
```

验证时使用 `model.eval()` 和 `torch.inference_mode()`。忘记切换模式会导致 Dropout 和 BatchNorm 行为错误。

### 4.4 可复现性

```python
import random
import numpy as np
import torch

def seed_everything(seed=42):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
```

固定随机种子不保证所有 GPU 算子绝对确定，但它能减少无意义变化。每次实验还应保存：Git commit、依赖版本、配置、随机种子、硬件、数据版本和指标。

### 4.5 显存与性能

显存主要由参数、梯度、优化器状态和激活组成。显存不足时按以下顺序处理：

1. 减小 batch size。
2. 使用 BF16/FP16。
3. 梯度累积。
4. gradient checkpointing。
5. LoRA/QLoRA。
6. 最后才考虑降低输入分辨率或模型规模，因为它们可能影响任务定义。

## 第 5 章：Linux、Git 与项目纪律

### 5.1 文件、进程和权限

必须熟悉：

```bash
pwd
ls -lah
find . -name "*.yaml"
ps aux
top
free -h
df -h
chmod +x script.sh
journalctl -f
```

机器人系统常由多个进程组成。定位故障时先确认进程是否存在、端口或设备是否占用、日志是否持续输出、CPU/GPU/内存是否异常。

### 5.2 Git 不是云盘

一次提交应该表达一个完整意图，例如“加入点云滤波”或“修复时间戳错误”。不要把模型文件、数据集、密钥和构建目录提交到 Git。

推荐流程：

```bash
git switch -c feature/rgbd-localization
git status
git add src tests configs
git commit -m "Add RGB-D localization baseline"
git push -u origin feature/rgbd-localization
```

任何 API 密钥、SSH 私钥、数据访问令牌一旦进入公开仓库，都应视为已经泄露：删除当前文件不够，还要撤销密钥并清理历史。

### 5.3 项目配置

不要把超参数散落在代码中：

```yaml
seed: 42
device: cuda
model:
  hidden_dim: 256
training:
  batch_size: 64
  learning_rate: 0.0003
  epochs: 100
```

程序启动时把完整配置复制到输出目录，使每个结果都能追溯。

## 第 6 章：现代 C++ 与 CMake

机器人实时路径和 ROS2 核心节点常使用 C++。学习重点不是语法数量，而是资源管理、类型安全和可测试接口。

### 6.1 RAII 与智能指针

对象构造时获取资源，析构时释放资源。优先使用值语义和 `std::unique_ptr`；只有明确共享所有权时才用 `std::shared_ptr`。

```cpp
#include <memory>
#include <vector>

class PointFilter {
public:
  explicit PointFilter(double threshold) : threshold_(threshold) {}
  std::vector<double> run(const std::vector<double>& values) const;
private:
  double threshold_;
};
```

使用 `const` 表达不修改，使用引用避免复制。不要在不清楚所有权时保存裸指针。

### 6.2 CMake 最小结构

```cmake
cmake_minimum_required(VERSION 3.16)
project(robot_math LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(robot_math src/transform.cpp)
target_include_directories(robot_math PUBLIC include)

add_executable(demo src/main.cpp)
target_link_libraries(demo PRIVATE robot_math)
```

库和可执行程序分离后更容易测试。编译选项、依赖和包含目录应绑定到具体 target，而不是全局污染。

## 第 7 章：ROS2 通信模型

### 7.1 节点与接口

ROS2 节点应职责单一。相机驱动负责发布图像，不应顺便完成所有感知和规划。接口按语义选择：

- Topic：连续数据流，例如图像、点云、里程计。
- Service：短时间请求响应，例如清空地图。
- Action：可取消、可反馈的长任务，例如导航到目标点。
- Parameter：运行配置，而不是高频数据。

### 7.2 QoS

QoS 决定可靠性、历史深度和数据生命周期。相机高频数据通常允许丢帧，控制命令则需要更谨慎。发布端和订阅端 QoS 不兼容时，节点都在运行却收不到数据。

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)
```

### 7.3 时间戳

传感器融合依赖正确时间。收到消息时临时填写当前时间会掩盖采集延迟。时间戳应尽量表示数据被采集的时间，而不是被下游处理的时间。

### 7.4 最小 Python 节点

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        self.publisher = self.create_publisher(Float32, 'temperature', 10)
        self.timer = self.create_timer(0.1, self.tick)

    def tick(self):
        msg = Float32()
        msg.data = 25.0
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = TemperaturePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

真实项目应加入参数、输入校验、日志级别和异常处理。

## 第 8 章：TF2、rosbag2 与系统调试

### 8.1 TF2 是带时间的坐标关系

TF2 不只是保存矩阵，它还保存坐标系树和时间。常见错误：

- 父子坐标系写反。
- 同一个 child 被多个节点发布。
- 查询时间超出缓存。
- 静态变换误用动态发布。
- 坐标系形成环。

检查工具：

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link camera_link
```

### 8.2 rosbag2

```bash
ros2 bag record /camera/image_raw /camera/depth /tf /tf_static
ros2 bag info <bag_directory>
ros2 bag play <bag_directory> --clock
```

录包使硬件问题和算法问题解耦。固定数据回放还能用于回归测试：新算法必须在相同输入上比较指标。

### 8.3 调试顺序

遇到“没有结果”时按顺序检查：

1. 节点是否运行，是否崩溃。
2. Topic 名称和消息类型是否一致。
3. QoS 是否兼容。
4. 发布频率和时间戳是否合理。
5. TF 是否存在且时间有效。
6. 数据值和单位是否正确。
7. 最后再怀疑算法本身。

这种顺序可以避免在模型代码中浪费数小时，最后发现只是 Topic 拼写错误。

## 第一册阶段项目

实现一个“模拟移动机器人状态系统”：

- Python 节点发布轮速和 IMU 模拟数据。
- C++ 节点订阅并计算简化里程计。
- 发布 `odom → base_link` TF。
- 使用参数配置噪声和发布频率。
- rosbag2 录制并回放。
- 单元测试坐标变换和积分计算。
- README 给出架构图、运行命令、Topic 表和已知误差。

验收标准：运行 5 分钟不崩溃；回放相同 bag 得到一致结果；缺少输入时给出明确日志；改变频率后积分仍使用真实时间差而不是固定步长。

完成后进入 [第一册实验与答案](/textbook/volume-1-labs)。
