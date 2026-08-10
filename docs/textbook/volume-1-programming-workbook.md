---
title: 第一册编程综合实验：从张量到 ROS2 节点
description: 一套可运行、可测试、可复现的 PyTorch、现代 C++、CMake 与 ROS2 综合训练。
---

# 第一册编程综合实验：从张量到 ROS2 节点

本实验册把第一册的数学与工程基础落实为一个完整的小系统：读取机器人状态数据，训练一个速度预测器，把模型输出封装成带安全检查的控制组件，再通过 ROS2 节点发布结果。重点不在模型规模，而在建立可以迁移到大型机器人项目的工作方法。

每个实验都遵循同一条验收规则：先写输入、输出和不变量，再实现正常路径，随后主动构造失败样本，最后保存能够复现实验的命令与结果。只看到一次正确输出不算完成；能够解释错误、重复运行并通过测试，才算完成。

## 第 1 章：建立统一实验项目

### 1.1 项目目标与数据约定

假设移动机器人每个时刻有六维观测：平面位置 `x, y`、航向角 `yaw`、当前线速度 `v`、当前角速度 `w` 和目标距离 `distance`。学习器预测下一时刻的两个控制量 `v_cmd, w_cmd`。

一个样本写成：

```text
timestamp,x,y,yaw,v,w,distance,v_cmd,w_cmd,episode
0.00,0.00,0.00,0.00,0.00,0.00,2.00,0.20,0.00,1
0.05,0.01,0.00,0.00,0.20,0.00,1.99,0.22,0.01,1
```

这里有三个容易被忽略的约束。

第一，`timestamp` 在单个 episode 内必须严格递增。第二，训练集和验证集按 episode 划分，不能随机打散所有行后划分，否则相邻帧泄漏会使验证指标虚高。第三，角度的单位必须固定为弧度。工程中最危险的错误常常不会抛异常，只会产生“看起来还能训练”的错误结果。

建立如下目录：

```text
robot_learning_lab/
  pyproject.toml
  configs/default.json
  data/raw/
  data/processed/
  outputs/
  src/robot_lab/
    __init__.py
    dataset.py
    model.py
    train.py
    evaluate.py
  tests/
    test_dataset.py
    test_model.py
  cpp/
    CMakeLists.txt
    include/robot_lab/safety_filter.hpp
    src/safety_filter.cpp
    tests/test_safety_filter.cpp
```

`pyproject.toml` 的最小版本如下：

```toml
[build-system]
requires = ["setuptools>=68"]
build-backend = "setuptools.build_meta"

[project]
name = "robot-learning-lab"
version = "0.1.0"
requires-python = ">=3.10"
dependencies = ["numpy>=1.26", "torch>=2.2"]

[project.optional-dependencies]
dev = ["pytest>=8", "ruff>=0.5"]

[tool.pytest.ini_options]
testpaths = ["tests"]
```

安装时执行 `python -m pip install -e ".[dev]"`。可编辑安装保证测试和脚本导入的是当前源码，而不是某个旧目录下的副本。运行 `python -c "import robot_lab; print(robot_lab.__file__)"` 可以确认实际导入位置。

### 1.2 配置不是散落的常量

把实验参数保存到 `configs/default.json`：

```json
{
  "seed": 42,
  "batch_size": 64,
  "learning_rate": 0.001,
  "epochs": 40,
  "hidden_dim": 128,
  "train_episodes": [1, 2, 3, 4, 5, 6, 7, 8],
  "val_episodes": [9],
  "test_episodes": [10]
}
```

配置加载后立即检查，不要等到训练半小时后才发现参数无效：

```python
def validate_config(cfg: dict) -> None:
    required = {"seed", "batch_size", "learning_rate", "epochs"}
    missing = required - cfg.keys()
    if missing:
        raise ValueError(f"missing config keys: {sorted(missing)}")
    if cfg["batch_size"] <= 0:
        raise ValueError("batch_size must be positive")
    if not 0.0 < cfg["learning_rate"] < 1.0:
        raise ValueError("learning_rate must be in (0, 1)")
    groups = [set(cfg[k]) for k in
              ("train_episodes", "val_episodes", "test_episodes")]
    if groups[0] & groups[1] or groups[0] & groups[2] or groups[1] & groups[2]:
        raise ValueError("episode split leakage detected")
```

实验记录至少包含：完整配置、Git 提交号、Python 与 PyTorch 版本、CUDA 是否可用、随机种子、训练日志和最终权重。没有这些信息，指标只是一个无法验证的数字。

## 第 2 章：数据管线与单元测试

### 2.1 先验证，再转换

下面的检查函数对每个 episode 验证时间、数值范围和有限性：

```python
from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class ValidationIssue:
    row: int
    field: str
    message: str

def validate_array(data: np.ndarray) -> list[ValidationIssue]:
    issues: list[ValidationIssue] = []
    if data.ndim != 2 or data.shape[1] != 10:
        return [ValidationIssue(-1, "shape", f"expected [N,10], got {data.shape}")]

    for row, col in zip(*np.where(~np.isfinite(data))):
        issues.append(ValidationIssue(int(row), str(col), "non-finite value"))

    episode_ids = np.unique(data[:, 9].astype(np.int64))
    for episode_id in episode_ids:
        rows = np.where(data[:, 9] == episode_id)[0]
        times = data[rows, 0]
        bad = np.where(np.diff(times) <= 0)[0]
        for index in bad:
            issues.append(ValidationIssue(
                int(rows[index + 1]), "timestamp", "not strictly increasing"
            ))

    for row in np.where(np.abs(data[:, 3]) > np.pi + 1e-6)[0]:
        issues.append(ValidationIssue(int(row), "yaw", "outside [-pi, pi]"))
    return issues
```

返回所有问题比遇到第一个错误就退出更适合数据治理，因为采集人员可以一次看到错误分布。若数据量很大，可以分块统计，但错误报告仍应包含文件、episode、行号和字段。

### 2.2 归一化只能拟合训练集

```python
@dataclass
class Standardizer:
    mean: np.ndarray
    std: np.ndarray

    @classmethod
    def fit(cls, x: np.ndarray) -> "Standardizer":
        mean = x.mean(axis=0)
        std = x.std(axis=0)
        std = np.where(std < 1e-8, 1.0, std)
        return cls(mean=mean, std=std)

    def transform(self, x: np.ndarray) -> np.ndarray:
        return (x - self.mean) / self.std

    def inverse(self, x: np.ndarray) -> np.ndarray:
        return x * self.std + self.mean
```

为什么小方差维度要把标准差替换为 `1.0`？若某个维度恒定，除以接近零的数会放大浮点误差。替换后该维度归一化结果为零，既保持有限，也明确告诉模型该特征不提供区分信息。

测试不是只检查形状：

```python
def test_standardizer_round_trip():
    rng = np.random.default_rng(7)
    x = rng.normal(size=(100, 6))
    scaler = Standardizer.fit(x)
    restored = scaler.inverse(scaler.transform(x))
    np.testing.assert_allclose(restored, x, rtol=1e-6, atol=1e-6)

def test_constant_column_stays_finite():
    x = np.array([[1.0, 5.0], [2.0, 5.0], [3.0, 5.0]])
    z = Standardizer.fit(x).transform(x)
    assert np.isfinite(z).all()
    np.testing.assert_allclose(z[:, 1], 0.0)
```

### 2.3 Dataset 的完整实现

```python
from pathlib import Path
import torch
from torch.utils.data import Dataset

FEATURE_COLUMNS = [1, 2, 3, 4, 5, 6]
TARGET_COLUMNS = [7, 8]

class ControlDataset(Dataset):
    def __init__(self, path: str | Path, episodes: set[int],
                 x_scaler: Standardizer | None = None,
                 y_scaler: Standardizer | None = None):
        raw = np.loadtxt(path, delimiter=",", skiprows=1, dtype=np.float64)
        issues = validate_array(raw)
        if issues:
            preview = "; ".join(str(issue) for issue in issues[:10])
            raise ValueError(f"invalid dataset ({len(issues)} issues): {preview}")

        mask = np.isin(raw[:, 9].astype(np.int64), list(episodes))
        selected = raw[mask]
        if len(selected) == 0:
            raise ValueError(f"no rows for episodes {sorted(episodes)}")

        x = selected[:, FEATURE_COLUMNS]
        y = selected[:, TARGET_COLUMNS]
        self.x_scaler = x_scaler or Standardizer.fit(x)
        self.y_scaler = y_scaler or Standardizer.fit(y)
        self.x = torch.from_numpy(
            self.x_scaler.transform(x).astype(np.float32)
        )
        self.y = torch.from_numpy(
            self.y_scaler.transform(y).astype(np.float32)
        )

    def __len__(self) -> int:
        return self.x.shape[0]

    def __getitem__(self, index: int):
        return self.x[index], self.y[index]
```

创建验证集时必须传入训练集的两个 scaler：

```python
train_set = ControlDataset(csv_path, set(cfg["train_episodes"]))
val_set = ControlDataset(
    csv_path,
    set(cfg["val_episodes"]),
    x_scaler=train_set.x_scaler,
    y_scaler=train_set.y_scaler,
)
```

若验证集重新拟合归一化参数，相当于利用了验证集整体分布，部署时单个在线样本又不具备这种信息。这个错误有时会让指标更好，却让真实部署更差。

## 第 3 章：写出可信的 PyTorch 训练程序

### 3.1 模型与前向不变量

```python
import torch
from torch import nn

class VelocityMLP(nn.Module):
    def __init__(self, input_dim: int = 6, hidden_dim: int = 128):
        super().__init__()
        self.network = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.SiLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.SiLU(),
            nn.Linear(hidden_dim, 2),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        if x.ndim != 2 or x.shape[1] != 6:
            raise ValueError(f"expected [B,6], got {tuple(x.shape)}")
        return self.network(x)
```

这里不用最后一层 `tanh`，因为目标已经标准化；部署时先逆归一化，再由独立安全层限幅。把物理安全完全寄托在神经网络激活函数上不可靠：模型转换、归一化错误或后处理遗漏都可能绕开它。

### 3.2 确定性与设备选择

```python
import random

def seed_everything(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)

def choose_device() -> torch.device:
    return torch.device("cuda" if torch.cuda.is_available() else "cpu")
```

“设了随机种子”不等于所有 GPU 算子都严格确定。需要逐位复现时可启用 `torch.use_deterministic_algorithms(True)`，但它可能降低速度或让不支持确定性实现的算子报错。研究阶段应记录随机种子并报告多次运行的均值和方差；排错阶段才优先追求严格确定性。

### 3.3 完整训练与验证循环

```python
from contextlib import nullcontext
from torch.utils.data import DataLoader

def run_epoch(model, loader, loss_fn, device, optimizer=None, scaler=None):
    training = optimizer is not None
    model.train(training)
    total_loss = 0.0
    total_samples = 0

    for x, y in loader:
        x = x.to(device, non_blocking=True)
        y = y.to(device, non_blocking=True)
        if training:
            optimizer.zero_grad(set_to_none=True)

        amp_context = (
            torch.autocast(device_type="cuda", dtype=torch.float16)
            if device.type == "cuda" else nullcontext()
        )
        grad_context = torch.enable_grad() if training else torch.no_grad()
        with grad_context, amp_context:
            prediction = model(x)
            loss = loss_fn(prediction, y)

        if not torch.isfinite(loss):
            raise FloatingPointError(f"non-finite loss: {loss.item()}")

        if training:
            if scaler is None:
                loss.backward()
                torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=5.0)
                optimizer.step()
            else:
                scaler.scale(loss).backward()
                scaler.unscale_(optimizer)
                torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=5.0)
                scaler.step(optimizer)
                scaler.update()

        batch_size = x.shape[0]
        total_loss += loss.item() * batch_size
        total_samples += batch_size

    if total_samples == 0:
        raise RuntimeError("empty dataloader")
    return total_loss / total_samples
```

损失乘以 batch 大小后再除总样本数，是为了正确处理最后一个不足整批的 batch。直接平均每个 batch 的均值，会让小 batch 获得与大 batch 相同的权重。

训练入口：

```python
def train_model(train_set, val_set, cfg, output_dir):
    device = choose_device()
    pin = device.type == "cuda"
    generator = torch.Generator().manual_seed(cfg["seed"])
    train_loader = DataLoader(
        train_set, batch_size=cfg["batch_size"], shuffle=True,
        num_workers=4, pin_memory=pin, persistent_workers=True,
        generator=generator,
    )
    val_loader = DataLoader(
        val_set, batch_size=cfg["batch_size"], shuffle=False,
        num_workers=4, pin_memory=pin, persistent_workers=True,
    )

    model = VelocityMLP(hidden_dim=cfg["hidden_dim"]).to(device)
    optimizer = torch.optim.AdamW(
        model.parameters(), lr=cfg["learning_rate"], weight_decay=1e-4
    )
    loss_fn = nn.SmoothL1Loss(beta=0.5)
    amp_scaler = torch.amp.GradScaler("cuda", enabled=device.type == "cuda")
    best_val = float("inf")

    for epoch in range(1, cfg["epochs"] + 1):
        train_loss = run_epoch(
            model, train_loader, loss_fn, device, optimizer, amp_scaler
        )
        val_loss = run_epoch(model, val_loader, loss_fn, device)
        print(f"epoch={epoch:03d} train={train_loss:.6f} val={val_loss:.6f}")
        if val_loss < best_val:
            best_val = val_loss
            checkpoint = {
                "model": model.state_dict(),
                "config": cfg,
                "x_mean": train_set.x_scaler.mean,
                "x_std": train_set.x_scaler.std,
                "y_mean": train_set.y_scaler.mean,
                "y_std": train_set.y_scaler.std,
                "best_val": best_val,
                "epoch": epoch,
            }
            torch.save(checkpoint, output_dir / "best.pt")
    return best_val
```

### 3.4 三个训练前测试

正式训练前必须通过以下检查。

**检查一：形状测试。** 随机生成 `[8, 6]` 输入，确认输出是 `[8, 2]`，所有值有限，反向传播后每个可训练参数都有梯度。

**检查二：单批过拟合。** 固定一个 32 样本的 batch，训练 500 步。如果损失不能下降两个数量级，应先检查标签、归一化、模型连接和优化器，不能立刻换更大的模型。

**检查三：标签打乱。** 随机打乱目标后，验证损失不应仍然很好。若指标几乎不变，常见原因是数据泄漏、指标实现错误，或模型根本没有使用输入。

```python
def test_backward_has_finite_gradients():
    model = VelocityMLP()
    x = torch.randn(8, 6)
    y = torch.randn(8, 2)
    loss = nn.MSELoss()(model(x), y)
    loss.backward()
    for name, parameter in model.named_parameters():
        assert parameter.grad is not None, name
        assert torch.isfinite(parameter.grad).all(), name
```

## 第 4 章：评估不是只报一个 MSE

归一化空间的损失用于优化，但最终报告应回到物理单位。对线速度和角速度分别计算 MAE、RMSE、95% 分位绝对误差，并按场景切片，例如近目标、远目标、急转弯和低速区。

```python
def regression_metrics(y_true: np.ndarray, y_pred: np.ndarray) -> dict:
    error = y_pred - y_true
    return {
        "mae_v": float(np.mean(np.abs(error[:, 0]))),
        "mae_w": float(np.mean(np.abs(error[:, 1]))),
        "rmse_v": float(np.sqrt(np.mean(error[:, 0] ** 2))),
        "rmse_w": float(np.sqrt(np.mean(error[:, 1] ** 2))),
        "p95_v": float(np.quantile(np.abs(error[:, 0]), 0.95)),
        "p95_w": float(np.quantile(np.abs(error[:, 1]), 0.95)),
    }
```

平均误差会掩盖尾部风险。机器人多数时刻在直行，少量急转弯样本可能决定是否碰撞；因此除了整体均值，还要看困难子集和最差 episode。部署门槛应写成多个条件，例如 `mae_v < 0.05 m/s`、`p95_w < 0.20 rad/s`，并要求所有输出有限且不超过安全层允许的硬范围。

### 4.1 误差分析表

对误差最大的 50 个样本保存以下字段：episode、时间、原始输入、真实值、预测值、误差、数据文件和可视化链接。逐个观察后给失败归因：标签噪声、时间错位、分布外状态、传感器异常、模型欠拟合或控制策略本身不连续。

一次规范的结论不是“模型在拐弯处不好”，而是：“测试集角速度绝对值大于 `0.6 rad/s` 的 312 个样本中，角速度 MAE 为整体的 2.8 倍；其中 71% 来自 episode 10 的湿滑地面段。排除时间同步错误后，下一轮应增加低附着场景并将路面状态纳入观测。”这种结论才直接导向下一项实验。

## 第 5 章：现代 C++ 安全过滤器

学习模型输出不能直接发送给执行器。下面实现独立的限幅和变化率限制器。即使模型输出 `NaN`，组件也应进入确定的安全状态。

### 5.1 头文件

```cpp
// include/robot_lab/safety_filter.hpp
#pragma once

#include <optional>

namespace robot_lab {

struct Command {
  double linear;
  double angular;
};

struct Limits {
  double max_linear{1.0};
  double max_angular{1.5};
  double max_linear_accel{0.8};
  double max_angular_accel{2.0};
  double max_dt{0.2};
};

class SafetyFilter {
 public:
  explicit SafetyFilter(Limits limits);
  std::optional<Command> update(Command requested, double dt);
  void reset(Command command = {0.0, 0.0});

 private:
  Limits limits_;
  Command previous_{};
};

}  // namespace robot_lab
```

### 5.2 实现

```cpp
// src/safety_filter.cpp
#include "robot_lab/safety_filter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot_lab {

SafetyFilter::SafetyFilter(Limits limits) : limits_(limits) {
  if (limits.max_linear <= 0.0 || limits.max_angular <= 0.0 ||
      limits.max_linear_accel <= 0.0 || limits.max_angular_accel <= 0.0 ||
      limits.max_dt <= 0.0) {
    throw std::invalid_argument("all limits must be positive");
  }
}

std::optional<Command> SafetyFilter::update(Command requested, double dt) {
  if (!std::isfinite(requested.linear) || !std::isfinite(requested.angular) ||
      !std::isfinite(dt) || dt <= 0.0 || dt > limits_.max_dt) {
    previous_ = {0.0, 0.0};
    return std::nullopt;
  }

  requested.linear = std::clamp(
      requested.linear, -limits_.max_linear, limits_.max_linear);
  requested.angular = std::clamp(
      requested.angular, -limits_.max_angular, limits_.max_angular);

  const double dv = limits_.max_linear_accel * dt;
  const double dw = limits_.max_angular_accel * dt;
  Command filtered{
      std::clamp(requested.linear, previous_.linear - dv, previous_.linear + dv),
      std::clamp(requested.angular,
                 previous_.angular - dw, previous_.angular + dw)};
  previous_ = filtered;
  return filtered;
}

void SafetyFilter::reset(Command command) { previous_ = command; }

}  // namespace robot_lab
```

返回 `std::optional` 而不是悄悄把非法输入变成零，是为了让调用者明确记录故障并决定是否触发急停。过滤器内部仍把历史状态归零，防止故障恢复后的第一帧沿用过时速度。

### 5.3 无第三方框架的最小测试

```cpp
#include "robot_lab/safety_filter.hpp"
#include <cassert>
#include <cmath>
#include <limits>

int main() {
  robot_lab::SafetyFilter filter({1.0, 1.5, 0.8, 2.0, 0.2});

  auto first = filter.update({10.0, -10.0}, 0.1);
  assert(first.has_value());
  assert(std::abs(first->linear - 0.08) < 1e-12);
  assert(std::abs(first->angular + 0.20) < 1e-12);

  auto invalid = filter.update(
      {std::numeric_limits<double>::quiet_NaN(), 0.0}, 0.1);
  assert(!invalid.has_value());

  auto recovered = filter.update({0.5, 0.5}, 0.1);
  assert(recovered.has_value());
  assert(std::abs(recovered->linear - 0.08) < 1e-12);
  return 0;
}
```

## 第 6 章：CMake、编译器警告与动态分析

```cmake
cmake_minimum_required(VERSION 3.20)
project(robot_lab LANGUAGES CXX)

add_library(robot_safety src/safety_filter.cpp)
target_include_directories(robot_safety PUBLIC include)
target_compile_features(robot_safety PUBLIC cxx_std_20)

if(MSVC)
  target_compile_options(robot_safety PRIVATE /W4 /permissive-)
else()
  target_compile_options(robot_safety PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(safety_filter_test tests/test_safety_filter.cpp)
  target_link_libraries(safety_filter_test PRIVATE robot_safety)
  add_test(NAME safety_filter_test COMMAND safety_filter_test)
endif()
```

标准构建流程是：

```bash
cmake -S cpp -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

`RelWithDebInfo` 同时保留优化和调试符号，适合性能接近发布版本的定位。Linux 下再建立一个启用 AddressSanitizer 和 UndefinedBehaviorSanitizer 的调试构建：

```bash
cmake -S cpp -B build-asan -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer"
cmake --build build-asan --parallel
ctest --test-dir build-asan --output-on-failure
```

编译器警告不能代替测试，测试也不能代替动态分析。三者分别擅长发现可疑语法与类型问题、行为回归、内存和未定义行为问题。

## 第 7 章：封装 ROS2 控制节点

节点订阅模型给出的 `geometry_msgs/msg/Twist`，通过 `SafetyFilter` 后发布安全命令。真实项目还应订阅急停、机器人状态和模型健康状态；本实验先聚焦时序、参数和故障语义。

### 7.1 节点骨架

```cpp
#include <chrono>
#include <memory>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_lab/safety_filter.hpp"

class CommandGuardNode : public rclcpp::Node {
 public:
  CommandGuardNode() : Node("command_guard"), filter_(load_limits()) {
    using std::placeholders::_1;
    publisher_ = create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel_safe", rclcpp::QoS(1).reliable());
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_raw", rclcpp::QoS(1).reliable(),
        std::bind(&CommandGuardNode::on_command, this, _1));
    last_time_ = now();
  }

 private:
  robot_lab::Limits load_limits() {
    return {
      declare_parameter("max_linear", 1.0),
      declare_parameter("max_angular", 1.5),
      declare_parameter("max_linear_accel", 0.8),
      declare_parameter("max_angular_accel", 2.0),
      declare_parameter("max_dt", 0.2)
    };
  }

  void on_command(const geometry_msgs::msg::Twist::SharedPtr msg) {
    const auto current = now();
    const double dt = (current - last_time_).seconds();
    last_time_ = current;
    const auto result = filter_.update(
        {msg->linear.x, msg->angular.z}, dt);
    if (!result) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000, "unsafe command or time step");
      publish_stop();
      return;
    }
    geometry_msgs::msg::Twist output;
    output.linear.x = result->linear;
    output.angular.z = result->angular;
    publisher_->publish(output);
  }

  void publish_stop() {
    publisher_->publish(geometry_msgs::msg::Twist{});
  }

  robot_lab::SafetyFilter filter_;
  rclcpp::Time last_time_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};
```

### 7.2 为什么还需要看门狗

上面的节点只在收到消息时执行。如果上游模型节点崩溃，它不会自动发布停止命令，底层控制器可能保持最后速度。因此生产系统必须有超时看门狗：定时器以固定频率检查 `now - last_received_time`，超过阈值就发布停止并报告诊断状态。更底层的电机控制器也应有独立超时，不能只依赖 ROS2 用户态进程。

看门狗测试至少包括：正常 20 Hz 输入不误触发；停止输入后在规定时间内输出零；模拟时间回退时进入安全状态；系统恢复后必须重新收到新命令，不能自动恢复旧命令。

### 7.3 QoS 的推理过程

控制命令关注最新值，不需要积压历史，因此深度通常设为 1。可靠传输可以减少局域网偶发丢包，但它不能代替超时机制；若无线网络很差，可靠传输的重发还可能带来陈旧消息。传感器高频数据常使用 best effort，控制与状态命令需依据链路、频率和安全要求实测。QoS 必须在发布端和订阅端兼容，否则节点图里能看到话题却收不到消息。

### 7.4 ROS2 集成验收命令

```bash
ros2 node list
ros2 node info /command_guard
ros2 topic info /cmd_vel_raw --verbose
ros2 topic hz /cmd_vel_safe
ros2 topic echo /cmd_vel_safe
ros2 param dump /command_guard
```

不要一次打开所有工具。先验证节点存在，再核对话题类型和 QoS，然后看频率，最后采样内容。这样可以快速区分“节点没启动”“通信不匹配”“频率异常”和“数值错误”。

## 第 8 章：四次完整故障排查演练

### 演练一：训练损失第一步就是 NaN

先保存出错 batch，不要反复盲跑。依次检查输入、标签、模型输出和损失是否有限；打印各列最小值、最大值、均值和标准差；禁用混合精度重试；把学习率降低十倍；在前向和反向处注册检查。

如果输入中存在无穷值，修复数据并让验证器拒绝该文件。如果仅在混合精度出现，检查是否有指数、除法或大范围平方运算，应将敏感计算强制为 float32。如果全精度也在若干步后出现，记录梯度范数，定位首次变为非有限的参数。梯度裁剪可以缓解爆炸，但不能掩盖错误标签或不稳定公式。

最终报告应写明首次异常位置、最小复现脚本、根因、修复和防回归测试。例如：“第 417 行 `distance` 为空字符串，被预处理替换为 `inf`；改为拒绝缺失值并新增 `test_non_finite_distance_is_rejected`。”

### 演练二：GPU 利用率只有 20%

先测每个 batch 的数据等待时间、主机到设备复制时间、前向、反向和优化器时间。若数据等待占主导，逐步试验 `num_workers`、`pin_memory`、`persistent_workers` 和缓存；若计算 kernel 很碎，提高 batch 或使用编译优化；若输入尺寸变化频繁，检查动态 shape 是否导致重复编译。

不要以 GPU 利用率作为唯一目标。小模型即使完全满足实时吞吐，也可能无法让 5090 长时间满载。真正指标是每秒样本数、单步延迟、显存占用和结果正确性。优化前后使用同一数据、预热步骤和计时同步；CUDA 操作异步，计时前后要用 CUDA Event 或适当同步。

### 演练三：ROS2 能看到话题但收不到数据

先执行 `ros2 topic info --verbose` 比较两端类型和 QoS。确认 `ROS_DOMAIN_ID`、RMW 实现、网络接口与容器网络设置一致。若同机进程可通信而跨机不行，检查组播、防火墙和 DDS 发现配置。若只有 rosbag 回放失败，检查是否使用模拟时间和回放 QoS。

最小化时，用官方命令行发布器替换上游，用命令行订阅替换下游。若发布器到命令行正常，问题在订阅节点；若命令行到订阅正常，问题在原发布端；两者都失败再查中间件环境。二分替换比同时修改多个节点更快。

### 演练四：离线指标优秀，上车振荡

先确认离线特征与在线特征的顺序、单位、坐标系、归一化版本完全一致。检查从传感器时间戳到执行器生效的总延迟，而不只是模型推理时间。对录制数据按在线代码路径重放，逐层比较中间张量。随后在仿真中注入实际延迟、噪声和控制频率抖动。

若接口一致但仍振荡，检查训练分布是否覆盖闭环产生的偏离状态。行为克隆只学习专家轨迹附近的动作，微小误差会把系统带到未见状态。可通过数据聚合、扰动采集、历史观测、动作平滑或模型预测控制改善。安全上应先降低速度与加速度上限，在隔离场地逐级恢复，不允许直接用实机搜索危险参数。

## 第 9 章：阶段考试

建议限时 180 分钟，满分 100 分。先独立作答，再看答案。代码题不仅评价语法，还评价边界条件、错误语义和测试设计。

### 一、基础判断与解释题，共 20 分

1. 将所有 CSV 行随机打乱后按 8:1:1 划分，是否适用于连续采集的机器人轨迹？解释原因。（4 分）
2. 设置随机种子后，两次 GPU 训练是否必然逐位相同？还需要考虑什么？（4 分）
3. 验证循环忘记 `model.eval()` 会对含 Dropout 或 BatchNorm 的模型产生什么影响？（4 分）
4. ROS2 话题深度设为 100 是否一定比深度 1 更安全？（4 分）
5. 为什么模型输出限幅和执行器急停必须是两个独立安全层？（4 分）

### 二、程序阅读题，共 20 分

观察以下代码：

```python
model.eval()
losses = []
for x, y in loader:
    prediction = model(x.cuda())
    losses.append(loss_fn(prediction, y.cuda()).item())
print(sum(losses) / len(losses))
```

指出至少四个工程问题，并给出修改方向。（12 分）

观察以下 C++：

```cpp
double limit(double value, double maximum) {
  if (value > maximum) return maximum;
  return value;
}
```

说明它对负数、`NaN`、非法 `maximum` 的行为，并写出你期望的接口契约。（8 分）

### 三、实现题，共 35 分

1. 实现 `split_by_episode(rows, train_ids, val_ids, test_ids)`，要求检查集合互斥、未知 episode、空划分，并保持每个划分内部原顺序。（15 分）
2. 为 `SafetyFilter` 设计至少六个测试，覆盖正常限幅、变化率限制、非法时间步、非有限输入、故障恢复和构造参数错误。（10 分）
3. 为控制节点写出看门狗伪代码，要求时钟回退和上游断流都进入停止状态，并限制重复日志频率。（10 分）

### 四、综合分析题，共 25 分

某实验报告只给出：“测试 MSE 为 0.002，GPU 推理 1.3 ms，模型可以部署。”请说明为什么证据不足，并设计从离线验证到实机低速测试的分阶段验收表。至少覆盖数据、指标、时序、资源、安全与回滚。（25 分）

## 第 10 章：考试参考答案与评分要点

### 一、基础题答案

1. 不适用。相邻帧高度相关，随机划分会把同一轨迹甚至近乎相同的帧放进训练和测试，造成泄漏。应按 episode、场景、日期或实体划分，并让划分规则匹配泛化目标。

2. 不必然。还受非确定性 CUDA 算子、线程调度、数据加载顺序、库版本和硬件影响。应记录环境与 seed，需要严格复现时启用确定性算法并接受性能代价。

3. Dropout 会继续随机丢弃激活，BatchNorm 会使用当前 batch 统计并更新运行统计，导致验证结果波动甚至污染模型状态。应在验证前 `model.eval()`，并使用 `torch.no_grad()` 或 `torch.inference_mode()`。

4. 不一定。控制只需要最新命令，较深队列可能在拥塞后依次执行陈旧命令，增加延迟。深度应与消息语义、发布频率和消费能力匹配，并配合时间戳与超时机制。

5. 软件限幅只能处理已知数值范围，不能覆盖进程崩溃、通信中断、错误接线或执行器故障。独立急停提供不同失效机制下的最后保护，避免单点故障。

### 二、程序阅读题答案

Python 片段的主要问题包括：没有 `torch.no_grad()`，会建立无用计算图；每批调用 `.cuda()`，没有统一设备抽象；没有检查空 loader；简单平均 batch loss 会错误加权最后的小 batch；没有检查非有限输入和损失；没有保存逐样本预测以计算物理单位指标；若数据加载启用 pinned memory，也没有使用非阻塞复制。每指出一项并给合理修改得 3 分，最多 12 分。

C++ 函数只限制正上界，不限制负下界；`NaN > maximum` 为假，会原样返回 `NaN`；零或负 `maximum` 没有被拒绝；接口也没说明无穷值。合理契约可以是：`maximum` 必须为有限正数，否则抛 `invalid_argument`；`value` 必须有限，否则返回错误状态；合法值被限制到 `[-maximum, maximum]`。也可采用显式 `expected` 或 `optional`，只要错误语义一致且可测试。

### 三、实现题参考

```python
def split_by_episode(rows, train_ids, val_ids, test_ids):
    groups = [set(train_ids), set(val_ids), set(test_ids)]
    if any(len(group) == 0 for group in groups):
        raise ValueError("every split needs at least one episode")
    if groups[0] & groups[1] or groups[0] & groups[2] or groups[1] & groups[2]:
        raise ValueError("episode sets must be disjoint")

    available = {int(row["episode"]) for row in rows}
    requested = set().union(*groups)
    unknown = requested - available
    if unknown:
        raise ValueError(f"unknown episodes: {sorted(unknown)}")

    outputs = []
    for group in groups:
        selected = [row for row in rows if int(row["episode"]) in group]
        if not selected:
            raise ValueError("empty split after filtering")
        outputs.append(selected)
    return tuple(outputs)
```

`SafetyFilter` 六类测试的关键断言分别是：大命令先满足速度硬上限；相邻帧差值不超过加速度乘时间；`dt <= 0` 和 `dt > max_dt` 返回空；`NaN` 与无穷输入返回空；故障后历史速度归零，下一帧重新受变化率约束；任一非正限制使构造函数抛异常。额外测试正负方向对称、恰好边界值和连续多步逼近上限可获加分。

看门狗伪代码：

```text
on_message(message):
  if message.timestamp older than last_message_timestamp:
    latch_fault("time moved backward")
    publish_stop()
    return
  last_message_timestamp = message.timestamp
  last_receive_clock = monotonic_now()
  if finite(message) and not fault_latched:
    publish(filtered(message))
  else:
    publish_stop()

on_timer():
  age = monotonic_now() - last_receive_clock
  if age < 0 or age > timeout:
    publish_stop()
    log_throttled("command timeout", period=2 seconds)
```

时间间隔优先使用单调时钟，消息时间戳用于检测数据新旧；故障锁存是否自动恢复必须由系统安全要求决定。答案明确区分两种时钟、持续发布零命令且有限频日志即可得满分。

### 四、综合题参考

单个 MSE 没有物理单位、数据划分、基线、方差和尾部误差；1.3 ms 没说明硬件、batch、预热、同步和端到端延迟。它们都不能证明闭环稳定与安全。

合格验收分为五级。第一级是数据门禁：固定版本、按 episode 隔离、校验单位和时间戳、统计分布与缺失值。第二级是离线模型：与零输出、上一帧动作和传统控制器比较，报告物理单位 MAE、RMSE、P95、最差 episode 与多 seed 方差。第三级是软件回放：用部署代码读取 rosbag，逐层比对预处理、模型输出和安全过滤结果，验证非有限输入、断流和时钟异常。

第四级是仿真与硬件在环：注入网络延迟、丢包、传感器噪声和执行器饱和，检查闭环稳定、端到端 P95/P99 延迟、CPU/GPU/显存和温度，连续运行规定时长。第五级才是隔离场地实机：先架空或低功率，再低速、空场、保护人员在位，逐级扩大速度和场景；每一级都有明确通过阈值、停止条件、日志和可一键切回的旧控制器。任何安全层失效、时序超限或分布外比例超过阈值都阻止升级，而不是用平均 MSE 覆盖风险。

## 第 11 章：完成标准与复盘模板

完成本实验册时，应能交付一个干净环境可重复执行的仓库，而不是几张终端截图。最低交付物包括：数据格式说明和校验报告；训练、验证、测试严格隔离的配置；单批过拟合与标签打乱结果；至少三次 seed 的物理单位指标；最佳和最差 episode 的误差分析；C++ 安全层测试；ROS2 看门狗验证；一份已知限制和实机前置条件清单。

每次实验用下面六句话复盘：本次只改变了什么；保持不变的对照条件是什么；预期现象是什么；实际证据是什么；哪个反例最能挑战当前结论；下一次最小实验是什么。若无法回答第一和第二句，说明一次改动混入了太多变量；若无法回答第五句，说明结论可能只是对结果的事后解释。

第一册的真正目标不是记住所有 API，而是形成稳定的工程闭环：用数学定义问题，用类型和断言守住接口，用测试覆盖边界，用指标验证假设，用日志保存证据，再用分层安全机制约束未知风险。后续无论学习视觉、SLAM、机械臂、强化学习还是 VLA，这套闭环都会反复出现。
