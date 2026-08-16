---
title: 第三册第五篇：局部规划、轨迹跟踪与 Nav2
description: 从 Pure Pursuit、DWA/DWB、TEB/MPC 思想到动态障碍、行为树、恢复策略和自动化导航评测。
---

# 第三册第五篇：局部规划、轨迹跟踪与 Nav2

全局规划器给出从起点到目标的大致路线，机器人仍要在每个控制周期根据当前位姿、速度和局部障碍生成可执行命令。局部导航是感知、规划和控制真正闭环的地方：定位抖动会变成转向振荡，代价地图延迟会变成碰撞风险，速度死区会让“数学上可行”的轨迹无法执行。

本篇从路径几何和闭环跟踪开始，逐步讲解 Pure Pursuit、DWA/DWB、TEB 与 MPC 的建模思想，再把它们放进 Nav2 的 lifecycle、plugin、behavior tree 和 costmap 架构中。最终目标不是某次演示成功，而是建立可批量复现、能区分定位/规划/控制/执行故障的导航评测系统。

## 第 1 章：路径、轨迹和控制命令

路径是一串具有几何顺序的位姿，通常没有明确到达时间；轨迹同时给出时间、速度和加速度；控制命令是当前周期发送给底盘的线速度和角速度。把全局路径点直接当成控制目标，会忽略时间和动力学约束。

局部控制器输入至少包括：当前位姿与速度、全局路径局部段、local costmap、机器人 footprint、速度/加速度限制、目标容差和控制周期。输出不仅是 `cmd_vel`，还应有选中轨迹、碰撞预测、评分分解和失败状态。

## 第 2 章：路径预处理

### 2.1 最近点与单调进度

每周期寻找路径最近点时，如果路径自交或机器人偏离，最近索引可能跳回历史段。维护单调进度窗口，只在上一索引附近向前搜索，并允许受控回退。全局重规划后路径 ID 改变，必须重置进度。

### 2.2 重采样

路径点间距不均会让控制器行为依赖规划器输出密度。按弧长重采样到固定间隔，并保留曲率和终点。插值后的每段必须重新碰撞检查。

```python
import numpy as np

def cumulative_arc_length(points):
    points = np.asarray(points, np.float64)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) == 0:
        raise ValueError("points must be non-empty [N,2]")
    if len(points) == 1:
        return np.array([0.0])
    segment = np.linalg.norm(np.diff(points, axis=0), axis=1)
    return np.concatenate(([0.0], np.cumsum(segment)))

def resample_path(points, spacing):
    if spacing <= 0:
        raise ValueError("spacing must be positive")
    points = np.asarray(points, np.float64)
    arc = cumulative_arc_length(points)
    if arc[-1] == 0:
        return points[:1].copy()
    samples = np.arange(0.0, arc[-1], spacing)
    samples = np.append(samples, arc[-1])
    x = np.interp(samples, arc, points[:, 0])
    y = np.interp(samples, arc, points[:, 1])
    return np.column_stack((x, y))
```

重复路径点会产生零长度段，`np.interp` 对重复横坐标的行为不应被盲目依赖；正式实现先删除相邻重复点，并测试单点路径和极短路径。

## 第 3 章：Pure Pursuit 几何

### 3.1 前视点

在路径前方选择与机器人距离约为 $L_d$ 的点。将该点变换到机器人坐标 $(x_t,y_t)$。对差速或自行车运动学，连接圆的曲率

$$
\kappa=\frac{2y_t}{L_d^2}.
$$

若线速度为 $v$，差速角速度命令

$$
\omega=v\kappa.
$$

前视距离小，跟踪紧但对定位噪声和离散路径敏感；前视距离大，平滑但切弯。常用速度自适应：

$$
L_d=\operatorname{clip}(L_0+k_v|v|,L_{min},L_{max}).
$$

### 3.2 可测试实现

```python
from dataclasses import dataclass

@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float

def to_robot_frame(point_world, pose):
    dx = point_world[0] - pose.x
    dy = point_world[1] - pose.y
    c, s = np.cos(pose.yaw), np.sin(pose.yaw)
    return np.array([c * dx + s * dy, -s * dx + c * dy])

def choose_lookahead(path, pose, start_index, lookahead):
    if lookahead <= 0 or not 0 <= start_index < len(path):
        raise ValueError("invalid lookahead or start index")
    for index in range(start_index, len(path)):
        local = to_robot_frame(path[index], pose)
        if local[0] >= 0 and np.linalg.norm(local) >= lookahead:
            return index, local
    return len(path) - 1, to_robot_frame(path[-1], pose)

def pure_pursuit_command(path, pose, start_index, linear_speed,
                         lookahead, max_angular_speed):
    index, target = choose_lookahead(path, pose, start_index, lookahead)
    distance_squared = float(target @ target)
    if distance_squared < 1e-12:
        return index, 0.0, 0.0
    curvature = 2.0 * target[1] / distance_squared
    angular = np.clip(linear_speed * curvature,
                      -max_angular_speed, max_angular_speed)
    return index, linear_speed, float(angular)
```

测试直线路径输出零角速度，左弯为正/右弯为负，路径终点、机器人偏离、反向目标、重复点和角速度限幅。符号取决于 ROS REP-103：正 yaw 通常逆时针。

### 3.3 速度调度

高速急弯会超过横向加速度：

$$
a_y=v^2|\kappa|.
$$

给定最大横向加速度，速度上界

$$
v\le\sqrt{a_{y,max}/\max(|\kappa|,\epsilon)}.
$$

还应根据障碍距离、目标距离和定位质量降速。速度调度必须遵守加减速度限制，不能一帧从最大速度降为零而期待底盘瞬时执行。

## 第 4 章：路径跟踪误差

横向误差表示机器人到参考路径的有符号垂距，航向误差是机器人 yaw 与路径切向差。Pure Pursuit 间接修正两者；Stanley 等控制器显式组合航向与横向误差。

评价跟踪要报告横向误差 P50/P95/max、航向误差、曲率、命令和实测速度、控制延迟和饱和比例。只看是否到终点会掩盖途中贴墙与振荡。

## 第 5 章：DWA 动态窗口

### 5.1 可达速度窗口

当前速度 $(v,omega)$、最大加速度 $(a_v,a_\omega)$、周期 $\Delta t$ 决定下一周期可达范围：

$$
v'\in[v-a_v\Delta t,v+a_v\Delta t],
$$

$$
\omega'\in[\omega-a_\omega\Delta t,
\omega+a_\omega\Delta t],
$$

再与全局速度上下限取交集。在窗口内采样速度，按运动模型前向模拟一段时间，拒绝碰撞轨迹并评分。

### 5.2 差速前向模拟

```python
def simulate_unicycle(state, linear, angular, horizon, dt):
    if horizon <= 0 or dt <= 0:
        raise ValueError("horizon and dt must be positive")
    x, y, yaw = map(float, state)
    trajectory = [(x, y, yaw)]
    steps = int(np.ceil(horizon / dt))
    for _ in range(steps):
        if abs(angular) < 1e-8:
            x += linear * np.cos(yaw) * dt
            y += linear * np.sin(yaw) * dt
        else:
            next_yaw = yaw + angular * dt
            radius = linear / angular
            x += radius * (np.sin(next_yaw) - np.sin(yaw))
            y -= radius * (np.cos(next_yaw) - np.cos(yaw))
            yaw = next_yaw
        trajectory.append((x, y, yaw))
    return np.asarray(trajectory)
```

实际最后一步可能超过 horizon，可调整步长；教学实现便于理解，生产控制器要统一模拟周期与真实控制频率。

### 5.3 评分

典型代价：终点方向、路径距离、目标距离、障碍距离、速度偏好、旋转和振荡。各项单位不同，必须归一化或理解尺度。总分最低不代表绝对安全，碰撞、制动距离和动力学约束应作为硬条件先过滤。

## 第 6 章：DWB 的 critic 思想

Nav2 DWB 把轨迹生成和多个 critic 插件组合。常见 critic 评价路径对齐、目标对齐、障碍、振荡、旋转到目标等。调参前逐项可视化评分；若某一 critic 数值量级远大于其他项，它会支配总行为。

调参顺序：先保证速度/加速度与底盘一致；再验证 footprint 和碰撞；再让路径可跟踪；最后平衡净空、速度和终点行为。直接随机改所有 scale 很难复现原因。

## 第 7 章：制动安全

机器人必须在碰撞前停下。忽略控制和感知延迟的一维制动距离

$$
d_b=\frac{v^2}{2a_{brake}}.
$$

加入总延迟 $\tau$：

$$
d_{safe}\ge v\tau+\frac{v^2}{2a_{brake}}+m.
$$

局部规划的仿真 horizon 和障碍范围必须覆盖足够制动距离。加速度参数应来自实测，轮胎/地面变化会影响制动。软件发送零速度不是急停，底层驱动仍需超时和硬件安全链。

## 第 8 章：TEB 思想

Timed Elastic Band 把轨迹表示为带时间间隔的位姿序列，优化时间、障碍距离、速度/加速度和运动学等目标。轨迹像弹性带一样在障碍间调整，并可处理不同拓扑路径候选。

非线性优化依赖初值和权重，狭窄动态场景可能局部最优。时间间隔过小增加变量，过大降低分辨率。TEB 适合需要时间参数化和非完整约束的场景，但配置复杂；使用前仍要保证 costmap 与 footprint 正确。

## 第 9 章：MPC 思想

MPC 在有限预测时域内优化控制序列：

$$
\min_{u_{0:N-1}}\sum_{k=0}^{N-1}
\|x_k-x_k^{ref}\|_Q^2+\|u_k\|_R^2
+\|\Delta u_k\|_S^2+\Phi_{obstacle}(x_k),
$$

满足动力学、输入、状态和碰撞约束。每周期只执行第一步，再滚动优化。

MPC 能显式处理约束和动态，但依赖模型、求解时间和可行初值。求解超时/不可行必须有备用命令，不能继续使用旧控制序列而不检查障碍变化。对普通低速差速机器人，调好的简单控制器可能更可靠；选择以任务证据为准。

## 第 10 章：动态障碍

普通 costmap 只表示当前占据，不含障碍速度。对缓慢移动人群，频繁更新和保守膨胀可能足够；高速交互需要跟踪、预测和时空碰撞检查。

预测有不确定性，时间越远区域应越宽。不能假设行人恒速直线必然成立。安全策略包括降速、让行、保留逃逸空间和超时停止。动态障碍消失后清除地图必须有射线或跟踪证据，不能永久留下幽灵。

## 第 11 章：目标到达逻辑

位置容差、角度容差、速度阈值和保持时间共同定义到达。只进入位置圆不代表停稳；角度要求过严会让机器人在目标附近反复旋转。路径末端方向应与任务相关：到充电桩需要精确朝向，普通巡检点可能不需要。

goal checker 和 progress checker 是不同概念。progress checker 判断一段时间是否有足够移动，防止卡死；低速精确对接时阈值过大可能误判无进展。

## 第 12 章：Nav2 架构

主要组件：`planner_server`、`controller_server`、`smoother_server`、`behavior_server`、`bt_navigator`、`waypoint_follower`、global/local costmap、map server、AMCL 和 lifecycle manager。它们通过 action、topic、service 和 TF 协作。

Nav2 是插件化系统。全局 planner、controller、goal/progress checker、costmap layer 和 behavior 都可替换。插件 ID、类型名和参数层级必须一致，YAML 缩进错误可能让参数落在错误节点而使用默认值。

## 第 13 章：Lifecycle

节点从 unconfigured → inactive → active。configure 加载参数和资源，activate 才开始实际工作。节点存在于 `ros2 node list` 不代表已 active。启动排查检查 lifecycle 状态、transition 错误、bond 和依赖服务。

配置失败时应停在安全状态，不发送速度。重新激活前检查 TF、地图和传感器是否准备好。lifecycle manager 的 autostart 方便演示，但生产系统应有明确启动依赖和超时。

## 第 14 章：Nav2 坐标与 TF

典型链：

```text
map -> odom -> base_link -> sensors
```

AMCL/SLAM 发布 `map -> odom`，里程计发布 `odom -> base_link`，传感器外参静态发布。控制器通常在 odom/local costmap 连续坐标中工作，全局目标在 map 中。

TF tolerance 不是越大越好。容忍过期变换可能降低报错但增加空间错位。统计传感器消息年龄、TF 可用时间和系统延迟，修复时钟和队列根因。

## 第 15 章：Global 与 Local Costmap

global costmap 常在 map frame、范围较大，包含静态层和较慢更新；local costmap 常在 odom frame、rolling window、高频融合实时障碍。两个 costmap 的 footprint、分辨率和 inflation 可以不同，但差异必须有目的。

局部地图太小，高速时看不到制动距离；太大计算昂贵。更新频率、发布频率和传感器频率不同，重点是内部更新能跟上运动。障碍 mark/clear 的 range、raytrace 和高度范围要与传感器安装匹配。

## 第 16 章：行为树

行为树节点返回 Success、Failure 或 Running。Sequence 顺序执行，Fallback 在前项失败后尝试后项，Decorator 控制重试或速率。Nav2 用行为树组合规划、跟踪、重规划、恢复和任务取消。

行为树解决任务流程，不修复底层算法。若 footprint 错，反复清图重试只会再次碰撞；若定位丢失，原地旋转可能让问题更糟。恢复行为必须基于失败分类。

## 第 17 章：恢复策略

常见恢复：清除局部/全局 costmap、原地旋转、后退、等待、重新规划、重新定位。每个动作有前置安全条件、最大次数和超时。

设计层级：先局部重规划；确认临时障碍后等待；地图残留才清理；局部困住且后方安全才后退；感知覆盖允许才旋转；定位异常先停止并重定位。无限 retry 会掩盖系统故障和耗尽任务时间。

## 第 18 章：参数调试顺序

1. 验证底盘真实速度、加速度、制动和死区。
2. 验证 TF、时间戳与 odom 连续性。
3. 验证静态地图、AMCL 和激光叠加。
4. 验证 footprint、障碍层、清除和 inflation。
5. 在无障碍宽场调路径跟踪。
6. 加入静态障碍平衡净空和效率。
7. 调终点行为和 progress checker。
8. 加入动态障碍与恢复行为。

每次只改变一组参数，用固定起终点和 rosbag 比较。YAML 保存 commit，不使用 `final_v7_really.yaml`。

## 第 19 章：自动化导航评测

### 19.1 场景矩阵

至少包含宽走廊、窄门、直角弯、U 型障碍、动态横穿、弱定位区、未知边界和目标附近障碍。每个场景设置多组起终点和随机种子，正向/反向都测试。

### 19.2 指标

- 成功率和失败状态；
- 规划时间、控制时间、总到达时间；
- 实际路径/参考最短路径比；
- 横向误差和最终位姿误差；
- 最小障碍净空；
- 速度、加速度、jerk 和饱和比例；
- 重规划/恢复次数；
- 定位质量与消息年龄；
- 碰撞、急停和错误成功。

### 19.3 结果记录

```python
from dataclasses import asdict, dataclass

@dataclass
class NavigationTrial:
    trial_id: str
    map_id: str
    start: tuple[float, float, float]
    goal: tuple[float, float, float]
    success: bool
    failure_reason: str
    duration_s: float
    path_length_m: float
    minimum_clearance_m: float
    recoveries: int
    final_position_error_m: float
    final_yaw_error_rad: float
```

每次失败保存最近一段 rosbag、参数、地图、BT 日志、costmap 快照和轨迹。只保存成功录像无法提高系统。

## 第 20 章：失败分类器

导航 action 返回 failure 后，依据证据分类：定位丢失、全局不可达、局部无安全轨迹、控制无响应、进度超时、目标无效、传感器/TF 过期、碰撞或系统异常。分类器可先基于规则，保留 unknown 类，不能强行把每个失败塞进已有标签。

失败分类准确率也要抽样人工审核。若大量失败被错分，自动报表会把调参方向带偏。

## 第 21 章：常见故障

### 路径正确但机器人蛇形

检查定位 yaw 抖动、控制周期、前视距离、最小角速度、底盘死区、实测速度反馈和角速度饱和。先在无障碍直线做阶跃/跟踪实验，避免代价地图干扰。

### 终点附近持续旋转

检查目标 yaw、角度容差、rotate-to-heading、定位角度噪声、最小可执行角速度和 goal checker。若底盘低于某角速度不动，控制器会反复发送无效小命令。

### costmap 障碍不消失

检查 clearing 是否开启、raytrace range、传感器最大距离、TF、消息高度、Inf 处理和 rolling window。清全图能暂时恢复但不应成为常态。

### 动态行人横穿时急停过晚

测量从采样到制动的总延迟、local costmap 频率、预测 horizon、制动能力和传感器视场。只增加障碍 critic 权重不能弥补看见太晚。

## 第 22 章：综合实验一——Pure Pursuit

在仿真中生成直线、圆、S 弯、直角和平滑随机路径。实现路径重采样、最近进度、速度自适应前视和曲率限速。加入位姿噪声、控制延迟、角速度饱和和底盘死区。

比较固定/速度自适应前视，报告横向 P95、最大误差、到达时间、角速度变化和饱和比例。解释一组“小前视误差低但振荡大”和“大前视平滑但切弯”的反例。

## 第 23 章：综合实验二——Nav2 基准

构建仿真差速机器人，固定地图和 30 组起终点。先使用默认基线参数，再只改变一组：footprint/inflation、速度加速度、controller critic、goal/progress checker、恢复 BT。

每组至少重复三次，报告成功率置信区间和失败分类。挑选三个最差案例，用 rosbag 回放复现并提出最小修复。最终演示必须包含失败拒绝与恢复，不只是成功路线。

## 第 24 章：阶段考试

建议限时 180 分钟，满分 100 分。

### 一、理论题，共 35 分

1. 区分路径、轨迹和控制命令。（5 分）
2. 推导 Pure Pursuit 曲率并说明前视距离权衡。（5 分）
3. DWA 动态窗口由哪些约束形成？（5 分）
4. 推导包含延迟的制动距离安全条件。（5 分）
5. 比较 DWB、TEB 和 MPC 的特点。（5 分）
6. Nav2 global/local costmap 为什么常用不同 frame？（5 分）
7. 行为树恢复为什么不能替代底层修复？（5 分）

### 二、代码题，共 30 分

1. 为 Pure Pursuit 写十个边界测试。（10 分）
2. 实现 DWA 速度采样、轨迹模拟与硬碰撞过滤。（10 分）
3. 设计自动导航 action 测试器的数据和超时状态机。（10 分）

### 三、综合题，共 35 分

1. 机器人在窄门前左右振荡，设计分层排查。（15 分）
2. 默认 Nav2 成功率 92%，调参后 97%，为什么仍不能直接断言更好？设计公平实验。（20 分）

## 第 25 章：参考答案

### 一、理论题

1. 路径定义空间顺序，轨迹增加时间和速度/加速度，控制命令是当前周期执行输入。路径几何可行不保证动力学可执行。

2. 机器人到前视点的圆满足几何关系，曲率 $2y_t/L_d^2$。小前视跟踪紧但敏感、易振荡；大前视平滑但切弯。应随速度、曲率和障碍自适应。

3. 全局速度上下限与当前速度在一个控制周期内按最大加速度可达的区间取交集，再考虑制动可行性和运动学。

4. 感知/计算/执行总延迟 $\tau$ 内先走 $v\tau$，随后以减速度 $a$ 停止需 $v^2/(2a)$，再加模型和安全余量。

5. DWB 离散采样速度、易解释插件化；TEB 优化带时间轨迹和拓扑，约束丰富但非线性调参复杂；MPC 显式模型和约束、滚动优化，计算和模型要求高。

6. global costmap 在 map 中保持全局一致，local costmap 在连续 odom 中滚动，避免全局定位修正让局部控制坐标跳变。

7. BT 只编排重试、清图、旋转等动作。错误 footprint、定位、时序或制动不会因重试消失，反复恢复还可能制造风险。

### 二、代码题

Pure Pursuit 测试包括直线、左右弯、起点/终点、单点、重复点、机器人偏离、目标在后方、yaw 跨界、角速度限幅、零速度和非法前视。DWA 必须验证动态窗口、采样边界、直线/圆弧、footprint 碰撞、制动距离和无轨迹状态。

自动测试器状态可为 reset → wait_localization → send_goal → navigating → success/failure/timeout → save_artifacts。保存 action 结果、轨迹、costmap、速度、碰撞、恢复和配置；任何阶段超时都有独立原因，下一 trial 前确认机器人/仿真已复位。

### 三、综合题

窄门振荡先确认门宽与 footprint/inflation 是否物理可通；检查定位横向/yaw 抖动；查看 local costmap 左右障碍是否交替；检查路径是否在门中心、controller critic 分解、前视/预测 horizon、速度死区和控制延迟；低速、固定真值定位和静态 costmap 分别做对照，逐层隔离。

成功率提升可能来自随机场景差异、超时放宽、速度降低、路径更长或碰撞风险增加。公平实验固定地图、起终点、随机种子和硬件负载，多次交叉运行；同时报告成功率置信区间、时间、长度、净空、碰撞、恢复、定位质量和失败类型；预先定义通过门槛，在未参与调参的测试集评价。

## 本篇完成标准

完成本篇后，应能把全局路径转换为满足速度、加速度和制动要求的闭环命令；能实现和测试 Pure Pursuit 与 DWA 基础；能解释 DWB/TEB/MPC 适用范围；能正确配置 Nav2 TF、costmap、lifecycle、plugin 和 behavior tree；能批量运行导航任务、保存失败证据并用多指标而非单次成功验收。

下一阶段将完成第三册结业项目和全册考试：从传感器融合、SLAM 建图、AMCL、全局/局部规划到自动化 Nav2 评测，最终把第三册推过十万字符并形成完整交付物。
