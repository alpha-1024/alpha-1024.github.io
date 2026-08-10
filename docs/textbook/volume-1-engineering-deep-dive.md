---
title: 第一册扩展：PyTorch 工程与 ROS2 系统
description: 可复现训练、数据管线、C++ 工程、ROS2 执行模型和机器人软件测试。
---

# 第一册扩展：PyTorch 工程与 ROS2 系统

## 第 1 章：从 Notebook 到工程项目

Notebook 适合探索，不适合作为长期系统入口。可维护项目应把数据、模型、训练、评估和配置分开：

```text
project/
  pyproject.toml
  configs/
  src/project/
    data/
    models/
    training/
    evaluation/
  scripts/
  tests/
  outputs/
  README.md
```

核心原则：同一份代码通过配置运行不同实验，而不是复制 `train_v2_final_really_final.py`。

### 配置快照

运行时创建唯一输出目录，保存解析后的完整配置、Git commit、环境和命令：

```python
from datetime import datetime
from pathlib import Path
import json, subprocess

run_dir = Path('outputs') / datetime.now().strftime('%Y%m%d-%H%M%S')
run_dir.mkdir(parents=True)
commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()
(run_dir / 'meta.json').write_text(json.dumps({'git': commit, 'config': cfg}, indent=2))
```

若工作区有未提交修改，应记录 diff 或直接拒绝正式实验，否则同一 commit 无法复现。

## 第 2 章：数据集设计

### 样本单位

先定义一个样本是什么。图像分类是一张图和标签；机器人策略可能是一段历史图像、关节状态、语言和未来动作块。定义不清会产生时间错位。

### 划分原则

- 不按视频帧随机划分。
- 不让同一物体实例同时出现在训练和测试（若目标是实例泛化）。
- 测试集在训练前固定。
- 归一化统计只用训练集。
- 删除重复样本后再划分。

### 数据版本

数据集版本至少记录：来源、采集时间、设备、过滤规则、标注版本、样本数量、哈希。大文件可用对象存储或 DVC，Git 只保存元数据和小样例。

### 数据检查

```python
def validate_sample(sample):
    assert sample['image'].ndim == 3
    assert sample['image'].shape[2] == 3
    assert np.isfinite(sample['state']).all()
    assert np.isfinite(sample['action']).all()
    assert sample['timestamp'] >= 0
```

检查器应输出错误列表，而不是遇到第一条坏数据就停止。

## 第 3 章：训练循环的正确性

训练前做三项测试：

1. **前向测试**：shape、dtype、device、输出范围。
2. **小数据过拟合**：几十样本训练到接近零损失。
3. **随机标签测试**：验证泛化应接近随机水平。

若小数据不能过拟合，优先查实现；若随机标签验证也很高，优先查数据泄漏。

### 梯度诊断

```python
def grad_stats(model):
    values = []
    for p in model.parameters():
        if p.grad is not None:
            values.append(p.grad.detach().norm())
    return torch.stack(values).mean().item() if values else 0.0
```

记录梯度范数、参数范数和学习率。梯度为零可能来自 detach、饱和激活或掩码；爆炸可能来自损失尺度、序列长度或错误标签。

### Checkpoint

完整 checkpoint 包含模型、优化器、scheduler、epoch、global step、随机状态和配置。只保存模型权重无法无缝续训。

## 第 4 章：评估与统计

测试集只用于最终或阶段性评估，不能反复根据测试结果调参。需要频繁调参时使用验证集。

报告均值还不够。随机种子、数据采样和环境随机性会造成方差。至少重复多次并报告标准差或置信区间。

分类指标要结合混淆矩阵；回归应报告 MAE、RMSE、P95 和最大误差；机器人闭环应报告成功率和失败类型。

### Bootstrap 置信区间

从实验结果有放回重采样，多次计算统计量，用分位数得到区间。样本少时区间很宽，这是事实，不应隐藏。

## 第 5 章：混合精度与 5090

BF16 指数范围接近 FP32，通常比 FP16 更不易溢出。训练示例：

```python
with torch.autocast(device_type='cuda', dtype=torch.bfloat16):
    output = model(inputs)
    loss = loss_fn(output, targets)
```

部分损失、归一化和几何计算可能需要 FP32。若出现 NaN：定位第一个非有限 Tensor，检查输入、loss 和梯度，不要只加入 `nan_to_num` 掩盖错误。

显存优化顺序：batch、混合精度、梯度累积、checkpointing、参数高效微调。每次修改重新测量吞吐，因为省显存措施可能增加计算。

## 第 6 章：Python 并发与机器人数据流

Python GIL 限制 CPU 密集型线程并行，但 I/O 和底层释放 GIL 的库仍可并发。多进程有序列化和复制成本；大图像跨进程传输可能成为瓶颈。

生产者快于消费者时必须定义背压：阻塞生产者、丢弃旧帧、采样或扩大缓冲。实时感知常选择最新帧，离线记录则优先不丢数据。

队列不能无限增长。监控队列长度、消息年龄和处理时间。

## 第 7 章：现代 C++ 所有权

### 值语义

能作为值保存的对象优先作为值。编译器可通过移动语义避免大复制。接口应清楚表达是否拥有资源。

- `T`：获得一个值。
- `const T&`：只读借用。
- `T&`：可修改借用。
- `std::unique_ptr<T>`：转移独占所有权。
- `std::shared_ptr<T>`：共享所有权。

共享指针不是默认答案。循环引用会泄漏，所有权不清会让对象生命周期难以推理。

### 异常与错误

构造阶段无法建立有效对象时可抛异常；实时控制循环通常避免异常路径。跨线程和跨节点接口应显式传播状态码和诊断信息。

### 数据竞争

两个线程并发访问同一内存且至少一个写入，没有同步就是数据竞争。互斥锁、原子和无锁结构各有代价。先保证正确，再根据 profiler 优化。

## 第 8 章：CMake 与依赖边界

每个 target 声明自己的 include、compile features 和依赖：

```cmake
add_library(localization src/ekf.cpp)
target_compile_features(localization PUBLIC cxx_std_17)
target_include_directories(localization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(localization PUBLIC Eigen3::Eigen)
```

`PUBLIC` 表示依赖会传播给使用者，`PRIVATE` 只用于实现。滥用全局 `include_directories` 会让模块边界失效。

启用警告和 sanitizer：

```cmake
target_compile_options(localization PRIVATE -Wall -Wextra -Wpedantic)
```

开发阶段使用 AddressSanitizer/UndefinedBehaviorSanitizer，注意它们会影响实时性能，不用于最终基准。

## 第 9 章：ROS2 图与发现

ROS2 基于 DDS。节点通过发现机制找到彼此，Domain ID 可隔离网络。多机器人或实验室网络中，错误 Domain ID 会导致看不到节点或串入其他机器人数据。

命名应使用 namespace：

```text
/robot_1/camera/image
/robot_1/odom
/robot_2/camera/image
```

不要把机器人编号硬编码在节点源代码；通过 namespace 和 remapping 配置。

## 第 10 章：QoS 深入

主要策略：

- Reliability：Reliable / Best Effort。
- Durability：Volatile / Transient Local。
- History：Keep Last / Keep All。
- Depth：保留样本数。
- Deadline：期望更新周期。
- Lifespan：消息有效时间。
- Liveliness：发布者存活检测。

静态地图或配置可能需要 Transient Local，让后加入订阅者获得最近数据；高频传感器常用 Best Effort 和小 depth。

QoS 是系统语义，不是网络调优按钮。要求每条都可靠可能产生积压，全部 Best Effort 又可能丢失关键命令。

## 第 11 章：Executor 与 Callback

Executor 决定 callback 何时在哪个线程执行。SingleThreadedExecutor 易推理但长回调阻塞所有任务；MultiThreadedExecutor 提高并发，但引入数据竞争。

Callback Group：

- Mutually Exclusive：同组回调不并行。
- Reentrant：允许并行或递归执行。

将推理、控制和服务分组。控制回调不能被耗时图像处理长期阻塞。

### 回调设计

回调中避免：大文件 I/O、无界等待、长时间网络请求。把重任务放 worker，并用有界队列传递数据。

## 第 12 章：生命周期与组件化

Lifecycle Node 状态：unconfigured、inactive、active、finalized。配置阶段加载参数和资源，activate 后开始发布/执行，deactivate 停止输出但保留资源。

这使系统能按顺序启动：驱动 ready → 定位 active → 导航 active。启动失败可回滚而不是所有节点无序运行。

Composable Node 将多个节点装入同一进程，减少序列化和拷贝，但故障隔离变弱。是否组件化应由性能和可靠性需求决定。

## 第 13 章：时间、同步与仿真时钟

ROS time 可来自系统时钟或 `/clock`。仿真和 rosbag 回放需设置 `use_sim_time=true`。混用系统时间和仿真时间会导致 TF 查询超时。

消息同步：ExactTime 要求时间戳完全一致；ApproximateTime 允许容差。容差不是越大越好，移动场景中过大容差造成空间误差。

测量端到端延迟时区分：采集时间、发布、接收、处理完成和输出时间。

## 第 14 章：参数、Launch 与配置

参数必须有默认值、类型、范围和描述。启动时校验：负频率、空模型路径、非法 frame 名应立即报错。

Launch 负责组合节点、namespace、remap 和参数文件，不应包含复杂业务逻辑。

为开发、仿真和真实硬件建立分层配置：共享算法参数，覆盖设备路径和频率。

## 第 15 章：日志与诊断

日志级别：DEBUG、INFO、WARN、ERROR、FATAL。高频回调不能每帧 INFO，否则影响性能和可读性。使用 throttle 日志。

诊断应发布结构化健康状态：频率、延迟、温度、丢帧、错误码。机器人故障时只留下“node started”没有价值。

## 第 16 章：测试体系

### 单元测试

测试纯函数和算法：坐标变换、滤波更新、边界检查。快速、确定、不依赖 ROS 图。

### 集成测试

启动多个节点，验证 Topic、Service、TF 和生命周期。使用固定 rosbag 作为输入。

### 系统测试

在仿真或硬件执行任务，统计成功率、延迟和资源。系统测试失败时回到分层日志定位。

### 回归测试

每个修复都加入可重复测试。否则同类错误会再次出现。

## 第 17 章：安全与密钥

代码仓库不得提交：SSH 私钥、API token、数据库密码、个人敏感数据。使用 `.gitignore`、环境变量和 secret manager。

密钥一旦进入公开历史，应立即撤销重建。Git 删除当前文件不等于互联网副本消失。

机器人动作还需要软件安全：命令超时归零、限幅、急停、watchdog、通信丢失处理和启动状态检查。

## 第 18 章：综合工程实验

构建 ROS2 感知服务：相机节点发布图像，推理 worker 处理最新帧，结果节点转换 TF 并发布，diagnostic 节点报告频率和延迟。

要求：

- QoS 有设计说明。
- 队列有界。
- 支持 lifecycle。
- `use_sim_time` 可配置。
- 相机断流可检测。
- 用 rosbag 做回归。
- C++ 和 Python 均有单元测试。
- 运行 30 分钟资源稳定。

## 工程章末考试

1. 为什么同一训练脚本需要保存 Git commit？
2. 小数据不能过拟合说明什么？
3. 随机帧划分视频数据为何会泄漏？
4. `shared_ptr` 为什么不应默认使用？
5. Reliable 传感器流为什么可能增加控制延迟？
6. MultiThreadedExecutor 带来什么风险？
7. rosbag 回放为何常需 `use_sim_time`？
8. Composable Node 的收益和代价是什么？
9. 回调中为什么不应做无界等待？
10. 为什么删除私钥提交仍不安全？

### 答案要点

1. 让结果对应到确切代码状态。
2. 实现、标签、损失或优化可能有错误。
3. 相邻帧高度相似，测试集不独立。
4. 共享所有权复杂、循环引用和生命周期不清。
5. 丢失时重传/积压旧消息，消息年龄增加。
6. 数据竞争、顺序不确定和锁竞争。
7. 节点和 TF 必须使用录制时钟一致推进。
8. 减少拷贝和进程开销，但故障隔离降低。
9. 阻塞 Executor，影响其他回调和控制周期。
10. 历史和外部副本仍含密钥，必须撤销轮换。
