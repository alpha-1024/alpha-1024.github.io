---
title: 第六册：CUDA、TensorRT 与工程部署
description: GPU 性能分析、ONNX、TensorRT、ROS2 推理节点和系统验收。
---

# 第六册：CUDA、TensorRT 与工程部署

## 第 1 章：先定义性能问题

“模型很慢”不是可执行描述。需要区分：

- Latency：单个请求完成时间。
- Throughput：单位时间处理数量。
- P50/P95/P99：延迟分位数。
- GPU utilization：GPU 忙碌比例。
- Memory：峰值显存和常驻显存。
- Power：功耗和温度。
- End-to-end latency：采集到动作发布的总延迟。

提高 batch 通常提升吞吐，但可能增加单请求等待。机器人闭环更关注 P95 延迟和抖动，而离线数据处理更关注吞吐。

测试必须包含 warmup，GPU 第一次运行会初始化上下文、分配内存和选择 kernel。

```python
import time
import torch

@torch.inference_mode()
def benchmark(model, x, warmup=50, runs=200):
    model.eval()
    for _ in range(warmup):
        model(x)
    torch.cuda.synchronize()
    times = []
    for _ in range(runs):
        start = time.perf_counter()
        model(x)
        torch.cuda.synchronize()
        times.append((time.perf_counter() - start) * 1000)
    t = torch.tensor(times)
    return {'mean': t.mean().item(), 'p95': torch.quantile(t, .95).item()}
```

没有 `torch.cuda.synchronize()` 的 CPU 计时通常只测到异步 kernel 提交时间。

## 第 2 章：GPU 执行模型

GPU 由大量线程并行执行。线程组成 warp，warp 在 SM 上调度。性能常受以下因素限制：

- 计算吞吐。
- 显存带宽。
- 访存不合并。
- 分支发散。
- occupancy 不足。
- CPU/GPU 同步和数据拷贝。

### 内存层次

寄存器最快且每线程私有；shared memory 位于 SM、线程块共享；global memory 容量大但延迟高。优化 kernel 常围绕减少 global memory 访问和提高数据复用。

### Arithmetic Intensity

单位数据搬运执行的计算量。低算术强度算子更可能受带宽限制；高算术强度更可能受计算限制。不要在没有 profiler 证据时猜瓶颈。

## 第 3 章：PyTorch 性能基础

优化顺序：

1. 正确性基线。
2. 去掉不必要的数据转换和同步。
3. `torch.inference_mode()`。
4. BF16/FP16 autocast。
5. 合理 batch 和输入尺寸。
6. `torch.compile`。
7. ONNX/TensorRT。
8. 最后才写自定义 CUDA kernel。

常见隐式同步：频繁 `.item()`、把 GPU Tensor 转 NumPy、每步打印 GPU 结果。

数据加载可使用 pinned memory 和异步拷贝：

```python
x = x.pin_memory().to('cuda', non_blocking=True)
```

只有数据管线和计算能重叠时才真正获益。

## 第 4 章：Profiler

PyTorch Profiler 查看 CPU、CUDA 时间、shape 和内存：

```python
from torch.profiler import profile, ProfilerActivity

with profile(
    activities=[ProfilerActivity.CPU, ProfilerActivity.CUDA],
    record_shapes=True,
    profile_memory=True,
) as prof:
    for _ in range(20):
        model(x)

print(prof.key_averages().table(sort_by='cuda_time_total'))
```

Nsight Systems 观察跨 CPU/GPU 时间线和空闲间隙；Nsight Compute 深入单个 kernel 的访存、occupancy 和指令。先用 Systems 找大瓶颈，再用 Compute 分析目标 kernel。

## 第 5 章：ONNX

ONNX 是模型交换表示，不保证任意 PyTorch 代码都可导出。控制流、自定义算子、动态 shape 和版本兼容是主要问题。

导出后必须验证数值：

```python
torch.onnx.export(
    model,
    example,
    'model.onnx',
    input_names=['image'],
    output_names=['output'],
    dynamic_axes={'image': {0: 'batch'}, 'output': {0: 'batch'}},
)
```

使用多组输入比较 PyTorch 和 ONNX Runtime，报告最大绝对误差、平均误差和任务指标。导出成功不代表语义一致。

## 第 6 章：TensorRT

TensorRT 优化图、融合算子、选择 kernel，并支持 FP32、FP16、BF16、INT8 等精度。

### Engine 与 Profile

动态输入需要定义 min/opt/max shape。范围太宽会增加 tactic 选择复杂度并可能影响性能；范围太窄则真实输入无法运行。

### FP16/BF16

通常显著提高吞吐并降低显存，但数值敏感算子可能需要高精度。必须比较最终任务指标。

### INT8

PTQ 需要代表性校准集。校准数据只覆盖晴天白背景，真实暗光场景可能严重掉点。QAT 在训练中模拟量化，成本更高但精度通常更稳。

## 第 7 章：端到端 ROS2 推理节点

推理链路：

```text
Sensor callback
→ 时间同步
→ 预处理
→ Host-to-Device
→ Inference
→ 后处理
→ TF
→ Publish
```

只报告 inference kernel 时间会掩盖图像解码、resize、拷贝和后处理。每个阶段都应打时间戳。

### 队列策略

处理速度低于相机频率时，不应无限排队旧帧。实时控制常选择小队列和丢弃旧数据，优先处理最新帧。

### 线程模型

回调中直接长时间推理会阻塞其他回调。可使用专用 worker、callback group 和多线程 executor，但必须明确数据所有权和并发安全。

## 第 8 章：模型服务与进程边界

同进程调用延迟低，但模型崩溃会影响整个节点；独立服务易隔离和扩展，但增加序列化和网络开销。机器人本机部署通常优先简单可靠，只有多消费者或多 GPU 调度时再引入复杂服务。

输入输出接口应版本化，包含：模型版本、预处理版本、坐标系、时间戳、单位、置信度和健康状态。

## 第 9 章：鲁棒性与降级

生产系统必须处理：

- 模型文件缺失或校验失败。
- GPU OOM。
- 输入尺寸或类型错误。
- 相机断流。
- 推理超时。
- 输出 NaN/Inf。
- GPU 温度或功耗异常。

降级策略：降低频率、切换轻量模型、回退传统算法、停车或进入安全姿态。异常不能只打印日志后继续发送无效动作。

## 第 10 章：性能实验方法

固定以下条件：

- GPU 型号、驱动、功耗模式。
- 软件版本和 Git commit。
- 输入 shape 和 batch。
- warmup 与运行次数。
- 精度模式。
- 是否包含预处理和后处理。

报告模板：

| Backend | Precision | Mean ms | P95 ms | FPS | VRAM | Task metric |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| PyTorch | FP32 |  |  |  |  |  |
| PyTorch | BF16 |  |  |  |  |  |
| ONNX Runtime | FP16 |  |  |  |  |  |
| TensorRT | FP16 |  |  |  |  |  |
| TensorRT | INT8 |  |  |  |  |  |

每种配置至少重复多轮，报告均值和离散程度。

## 第 11 章：综合部署项目

把第二册 RGB-D 感知模型部署到 ROS2：

1. 建立 PyTorch FP32 正确性基线。
2. 加 BF16/FP16。
3. 导出 ONNX 并验证。
4. 构建 TensorRT engine。
5. 封装 ROS2 推理节点。
6. 加入最新帧队列、超时和健康状态。
7. 用 rosbag 固定输入进行回归。
8. 输出性能和精度报告。

验收：P95 端到端延迟满足控制周期；连续运行一小时无内存持续增长；输入断流可检测；GPU OOM 后系统进入安全状态；模型输出与 PyTorch 基线在容差内。

## 第六册检查题

1. 为什么 GPU 计时需要同步？
2. 吞吐提高为什么可能让机器人更慢？
3. ONNX 导出成功后为什么仍要数值验证？
4. INT8 校准集为什么必须有代表性？
5. 为什么实时视觉节点通常不应积压所有帧？
6. 推理节点为什么要报告 P95 而不只报告平均值？
7. 什么情况下才值得写自定义 CUDA kernel？

### 答案摘要

1. CUDA 异步执行，不同步只测到提交时间。
2. 大 batch 等待聚合会增加单请求延迟。
3. 算子转换和数值精度可能造成输出偏差。
4. 量化范围来自校准数据，分布不匹配会导致饱和和精度损失。
5. 旧帧对闭环控制价值低，积压会增加感知延迟。
6. 抖动和尾延迟可能造成控制周期错过，平均值会掩盖它。
7. 已经 profiler 证明特定算子是主要瓶颈，且现有融合/库无法解决时。

## 全书毕业项目

构建一个语言指令驱动的仿真机器人系统：

- 视觉/点云感知目标和障碍。
- SLAM 或已知地图定位与导航。
- 机械臂规划和抓取。
- 模仿学习或 VLA 生成操作策略。
- TensorRT 部署感知模型。
- ROS2 编排、日志、rosbag 和自动评估。

最终交付：架构图、代码、锁定环境、数据说明、训练配置、模型、20+ 次实验统计、失败分类、性能报告、安全设计和演示视频。任何不能被别人按 README 复现的成果，都不算完成。
