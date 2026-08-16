---
title: 第四册第二篇：轨迹生成、碰撞检测与运动规划
description: 从多项式轨迹和时间参数化，到配置空间、连续碰撞检测、RRT/PRM、轨迹优化和 PlanningScene。
---

# 第四册第二篇：轨迹生成、碰撞检测与运动规划

逆运动学只回答“目标位姿附近是否存在一组关节角”。机械臂执行还要回答：从当前状态怎样到达该解；沿途是否碰撞；每个关节的速度、加速度和 jerk 是否满足限制；轨迹时间戳是否能被控制器正确执行；环境变化后原轨迹是否仍安全。

本篇从一维时间函数开始，构建多关节同步轨迹；随后进入配置空间和碰撞检测，推导采样规划、路线图与轨迹优化，最后建立与 MoveIt PlanningScene 连接的工程验收方法。

## 第 1 章：路径与轨迹

关节路径是连续曲线 $q(s),s\in[0,1]$，描述几何顺序。时间参数化 $s(t)$ 生成轨迹

$$
q(t)=q(s(t)).
$$

速度与加速度为

$$
\dot q=q'(s)\dot s,
$$

$$
\ddot q=q''(s)\dot s^2+q'(s)\ddot s.
$$

同一条无碰撞路径可以慢速安全执行，也可能因时间太短而超速度、加速度或力矩。路径规划和时间参数化应分别验证。

## 第 2 章：三次多项式

$$
q(t)=a_0+a_1t+a_2t^2+a_3t^3.
$$

给定 $q(0)=q_0,\dot q(0)=v_0,q(T)=q_f,\dot q(T)=v_f$，可解四个系数。矩阵法通用但对极小/极大 T 可能条件差；归一化时间 $\tau=t/T$ 更稳定。

零起终速度时：

$$
q(t)=q_0+(q_f-q_0)(3\tau^2-2\tau^3).
$$

其加速度在起终点一般不为零，轨迹拼接时会产生加速度跳变和无限理论 jerk。

## 第 3 章：五次多项式

五次多项式可满足起终位置、速度、加速度六个约束。零速度零加速度的归一化曲线：

$$
h(\tau)=10\tau^3-15\tau^4+6\tau^5.
$$

$$
q(t)=q_0+(q_f-q_0)h(t/T).
$$

```python
from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class TrajectorySample:
    time: float
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    jerk: np.ndarray

def quintic_rest_to_rest(q0, q1, duration, sample_times):
    q0 = np.asarray(q0, np.float64)
    q1 = np.asarray(q1, np.float64)
    times = np.asarray(sample_times, np.float64)
    if q0.shape != q1.shape or q0.ndim != 1:
        raise ValueError("q0 and q1 must be equal one-dimensional arrays")
    if duration <= 0 or np.any(times < 0) or np.any(times > duration):
        raise ValueError("invalid duration or sample time")
    delta = q1 - q0
    samples = []
    for t in times:
        tau = t / duration
        h = 10*tau**3 - 15*tau**4 + 6*tau**5
        dh = (30*tau**2 - 60*tau**3 + 30*tau**4) / duration
        ddh = (60*tau - 180*tau**2 + 120*tau**3) / duration**2
        dddh = (60 - 360*tau + 360*tau**2) / duration**3
        samples.append(TrajectorySample(
            float(t), q0 + delta*h, delta*dh, delta*ddh, delta*dddh
        ))
    return samples
```

测试端点位置/速度/加速度、维度、T 缩放规律、单调时间和随机有限差分。duration 翻倍时速度约减半、加速度减为四分之一、jerk 减为八分之一。

## 第 4 章：梯形速度与 S 曲线

梯形速度包含加速、匀速、减速。距离短时达不到最大速度，退化为三角速度。它易实现，但加速度瞬时跳变导致 jerk 无穷。

S 曲线限制 jerk，把加速度也平滑变化，适合振动敏感和高速机构。参数包括最大速度、加速度、jerk；短距离时多个阶段消失，分段求解需覆盖所有组合。实际控制器可能已做内部滤波，仍不能向其发送不满足接口的离散跳变。

## 第 5 章：多关节同步

每个关节按自身距离和限制计算最短时间，整体 duration 取最大值，其他关节降速同步到达。若只让每个关节独立最快到达，末端路径和碰撞过程会改变。

同步后重新计算全部关节速度/加速度/jerk，不能只检查最慢关节。关节限制来自 URDF、驱动和负载能力，使用最保守有效值；软件中 rad/s 与 degree/s 混用是高风险错误。

## 第 6 章：路径时间参数化

给定离散路径点，时间参数化分配严格递增时间，使关节速度/加速度满足限制。常见算法有 Iterative Parabolic Time Parameterization、Iterative Spline、Time-Optimal Trajectory Generation 等。

时间最优不等于任务最优：更快会增加振动、力矩、制动风险和感知模糊。可在限制内使用速度缩放，并以实机跟踪误差和周期抖动验收。

### 6.1 离散验证

对轨迹点 $(q_k,t_k)$：

$$
v_k\approx\frac{q_{k+1}-q_k}{t_{k+1}-t_k}.
$$

速度差估计加速度。离散采样可能漏掉多项式内部峰值，最好使用解析极值或足够密集采样并留余量。

```python
def validate_discrete_trajectory(times, positions, velocity_limits,
                                 acceleration_limits, tolerance=1e-9):
    times = np.asarray(times, np.float64)
    q = np.asarray(positions, np.float64)
    if q.ndim != 2 or len(times) != len(q) or len(times) < 2:
        raise ValueError("need times [N] and positions [N,J]")
    dt = np.diff(times)
    if np.any(dt <= 0):
        return False, ["timestamps_not_strictly_increasing"]
    velocity = np.diff(q, axis=0) / dt[:, None]
    reasons = []
    if np.any(np.abs(velocity) > np.asarray(velocity_limits) + tolerance):
        reasons.append("velocity_limit")
    if len(velocity) >= 2:
        midpoint_dt = 0.5 * (dt[:-1] + dt[1:])
        acceleration = np.diff(velocity, axis=0) / midpoint_dt[:, None]
        if np.any(np.abs(acceleration) > np.asarray(acceleration_limits) + tolerance):
            reasons.append("acceleration_limit")
    return not reasons, reasons
```

## 第 7 章：笛卡尔插值

直线 TCP 路径常用于接近、插入和焊接。位置可线性插值，姿态用 SLERP 或 Lie 群插值，不能逐元素插旋转矩阵。每个笛卡尔点需 IK，并选择连续分支。

即使所有离散 IK 点有解，点间关节插值仍可能碰撞或跨奇异。笛卡尔 fraction < 1 表示只完成部分，不应把部分轨迹当成完整成功执行。

## 第 8 章：配置空间

n 自由度机械臂配置空间 $\mathcal C$ 中一个点是完整 q。碰撞配置集合为 $\mathcal C_{obs}$，自由空间 $\mathcal C_{free}=\mathcal C\setminus\mathcal C_{obs}$。工作空间简单障碍在配置空间可能形成复杂高维区域。

规划问题：找到连续路径 $q:[0,1]\to\mathcal C_{free}$，满足起终配置、限位和可能的约束。IK 多解意味着目标是多个配置候选，不应预先只选一个质量差的解。

## 第 9 章：碰撞模型

visual mesh 用于显示，collision geometry 用于快速保守碰撞。过细三角网格降低速度并包含不影响安全的细节；过粗模型可能误报或漏碰。使用凸包、凸分解、盒/柱/球组合，并与实物外廓对照。

网格单位和 scale 必须验证。毫米网格按米加载会变大千倍。检查 AABB 尺寸、质量和惯量数量级。

### 9.1 自碰撞矩阵

相邻永远接触的 link 可禁用检测，永不可能碰撞的对也可预计算禁用。错误允许碰撞矩阵会漏掉真实自碰撞。生成后对随机配置主动搜索每个被禁用 pair，审计原因。

## 第 10 章：离散与连续碰撞检测

只检查路径节点会漏掉节点间穿障碍。最简单的边检测按关节距离细分：

```python
def interpolate_segment(q0, q1, maximum_joint_step):
    q0 = np.asarray(q0, np.float64)
    q1 = np.asarray(q1, np.float64)
    if q0.shape != q1.shape or maximum_joint_step <= 0:
        raise ValueError("invalid segment")
    steps = max(1, int(np.ceil(np.max(np.abs(q1 - q0)) / maximum_joint_step)))
    return np.array([q0 + (q1 - q0) * (index / steps)
                     for index in range(steps + 1)])

def segment_is_valid(q0, q1, state_is_valid, maximum_joint_step):
    for q in interpolate_segment(q0, q1, maximum_joint_step):
        if not state_is_valid(q):
            return False
    return True
```

最大 joint step 与 link 长度共同决定工作空间扫过距离。统一 0.1 rad 对长臂可能太粗。连续碰撞检测计算扫掠体或保守时间，可靠但实现复杂；生产使用成熟碰撞库。

## 第 11 章：距离场和安全余量

布尔碰撞只说碰/不碰，距离查询提供最小净空，可用于轨迹优化和验收。安全余量应覆盖碰撞模型误差、关节/结构柔性、环境感知误差和控制跟踪误差。

规划路径恰好贴碰撞边界，即使数学无碰撞，实机也不稳健。报告整条轨迹最小距离和低分位，而不只开始/结束。

## 第 12 章：RRT

RRT 循环：随机采样 $q_{rand}$；找树中最近节点 $q_{near}$；向样本扩展一步 $q_{new}$；验证边；加入树。一定概率直接采目标提高速度。

距离度量应按关节范围和对末端影响加权。转动连续关节要用环形角距离。欧氏 q 距离近不代表工作空间相似。

```python
from dataclasses import dataclass

@dataclass
class TreeNode:
    q: np.ndarray
    parent: int | None

def weighted_joint_distance(a, b, weights):
    delta = np.asarray(a) - np.asarray(b)
    return float(np.sqrt(np.sum(np.asarray(weights) * delta * delta)))

def steer(q_from, q_to, step_size, weights):
    delta = np.asarray(q_to, np.float64) - np.asarray(q_from, np.float64)
    distance = weighted_joint_distance(q_to, q_from, weights)
    if distance <= step_size:
        return np.asarray(q_to, np.float64).copy()
    return np.asarray(q_from) + delta * (step_size / distance)
```

## 第 13 章：RRT-Connect

两棵树从起点和目标生长。一棵向随机样本扩展，另一棵反复朝新节点 connect，然后交换。它常能快速找到机械臂可行路径，但路径锯齿、离障碍近，需 shortcut 和时间参数化。

连接成功必须最后验证两树节点间的整条边。重建路径时一棵树方向相反，索引处理错误会产生不连续轨迹。

## 第 14 章：概率完备性和非最优性

在一定条件下，RRT 随采样增加找到存在路径的概率趋近 1，称概率完备；它不保证固定时间找到，也不保证最短。RRT* 通过 rewiring 渐近最优，但初次解可能更慢。

一次规划失败不能证明不可达，可能是超时、采样、IK seed 或窄通道。要区分 invalid goal、无 IK、碰撞、timeout 和真正不可行未知。

## 第 15 章：PRM

PRM 离线采样自由配置，连接邻近可通边形成 roadmap；查询时连接起终点并图搜索。适合静态环境多次查询。环境变化后受影响边需重新验证。

窄通道占配置空间体积小，均匀采样难覆盖。可用障碍边界采样、bridge test 或任务先验提高。roadmap 节点和边的有效性要绑定机器人模型、碰撞环境和 joint limits 版本。

## 第 16 章：约束规划

末端保持水平、工具沿直线、闭链或接触形成低维约束流形。普通随机采样几乎不会精确落在等式约束上。方法包括投影采样、Atlas、任务空间规划或在优化中加入约束。

投影失败和多分支必须处理。约束容差太紧导致无解，太松破坏任务。容差用插入/抓取物理要求设置。

## 第 17 章：路径 shortcut 和平滑

随机选路径上两点，若直接连接无碰撞且更短，用直连替换中间段。重复可显著缩短 RRT 路径。shortcut 仍需连续碰撞，并可能降低障碍净空。

平滑 B-spline/样条要检查关节限位、碰撞和曲率。平滑后的曲线不是原折线的安全子集，必须作为新路径完整验证。

## 第 18 章：轨迹优化

将离散轨迹 $q_0,\ldots,q_N$ 作为变量，目标包含平滑、碰撞、路径长度、限位和终端：

$$
J=w_s\sum_k\|q_{k+1}-2q_k+q_{k-1}\|^2
+w_c\sum_k\phi(d(q_k))
+w_l\sum_k\|q_{k+1}-q_k\|^2.
$$

CHOMP 使用函数梯度，STOMP 用随机扰动，TrajOpt 常用序列凸优化。都依赖初值，可能陷入障碍错误侧。用 RRT 提供可行初值再优化是常见组合。

碰撞代价的梯度要求距离场和 link Jacobian。离散点优化也可能漏掉段间碰撞，仍需连续验证。

## 第 19 章：PlanningScene

MoveIt PlanningScene 包含机器人当前状态、世界碰撞物、附着物、允许碰撞矩阵和变换。规划前必须使用最新且一致的 scene snapshot。

环境物体有唯一 ID、frame 和 pose。更新同 ID 可移动，remove 显式删除。异步 PlanningScene 更新还未应用时立即规划，可能使用旧环境；等待确认或使用同步接口。

### 19.1 Attached Collision Object

抓住物体后，从 world 移除并附着到末端 link，允许与夹爪触碰但仍与环境碰撞。放置后 detach 并以正确世界位姿加入。忘记 attach 会让搬运中物体穿障碍；错误 touch links 会忽略过多碰撞。

## 第 20 章：规划请求契约

请求包括 group、起始状态、目标、workspace、规划时间、尝试次数、planner ID、速度/加速度缩放、路径约束。起始状态应来自新鲜关节状态，不能默认全零。

目标 pose frame 和时间必须可变换到 planning frame。目标容差影响 IK 和规划。多次 attempts 使用不同随机 seed，保存 seed 才能复现。

## 第 21 章：规划响应验收

规划成功后独立检查：轨迹非空；关节名称和顺序；时间严格递增；起点与当前状态一致；终点满足目标；位置/速度/加速度限制；离散与连续碰撞；最小净空；奇异值；执行 duration 合理；环境 scene version 未变化。

规划器 success code 不能替代调用者门禁。场景在规划后改变时需重新验证或取消。

## 第 22 章：规划基准

建立目标集合：宽空间、窄通道、绕障碍、多个 IK 解、近限位、近奇异、不可达和目标碰撞。每个 planner/参数多 seed 运行。

指标：成功率、首解时间、总优化时间、路径长度、平滑度、最小净空、执行时间、限位余量、轨迹点数和失败分类。只比较平均规划时间会忽略超时长尾和安全质量。

## 第 23 章：常见故障

### 起终状态无碰撞，执行中撞障碍

检查边/连续碰撞分辨率、轨迹平滑后是否重验、环境更新时间和实机跟踪误差。降低采样步长只是一种诊断，最终依据 link 扫过距离。

### 规划总绕远

采样规划器首先找可行，不保证短；检查 joint distance 权重、目标 IK 分支、规划时间和 shortcut。最近 IK 可能把肘部放在障碍错误侧。

### 时间参数化后控制器拒绝

检查 joint names、时间递增、首点、速度/加速度字段、控制器容差和 clock。浮点舍入产生相同纳秒时间也会违规。

### 仿真可行实机跟踪超差

检查动力学能力、负载、摩擦、加速度/jerk、控制周期和驱动内部限制。降低速度若改善说明规划约束与硬件能力不匹配。

## 第 24 章：综合实验一——RRT-Connect

在二维二连杆或三关节平面臂建立矩形障碍。实现状态采样、加权距离、steer、边碰撞、双树连接和路径重建。固定 seed 做单元测试，再在 100 个 seed 上报告成功率和时间。

故意只检查节点展示穿障碍反例，再加入边检查。比较 step size、goal bias 和边分辨率对成功、时间和漏碰的影响。

## 第 25 章：综合实验二——MoveIt PlanningScene

为六轴臂加载桌面、架子和三个可移动物体。自动生成 50 个可达目标和 20 个不可达/碰撞目标。比较 RRTConnect、PRM 与优化规划器，多 seed 记录指标。

模拟抓取 attach、搬运、detach，验证物体全程碰撞。规划完成后移动环境物体，确认 scene version 门禁取消旧轨迹。

## 第 26 章：阶段考试

建议限时 180 分钟，满分 100 分。

### 一、理论题，共 35 分

1. 路径和轨迹如何通过 $s(t)$ 联系？（5 分）
2. 三次与五次多项式边界连续性区别。（5 分）
3. 多关节同步为什么要重验限制？（5 分）
4. 离散节点无碰撞为何不保证路径安全？（5 分）
5. RRT-Connect 与 PRM 适用差异。（5 分）
6. 概率完备不意味着什么？（5 分）
7. PlanningScene attached object 语义。（5 分）

### 二、代码题，共 30 分

1. 为五次轨迹写十项测试。（10 分）
2. 实现 segment validity 并设计漏碰反例。（10 分）
3. 实现 RRT-Connect 路径重建和连续性测试。（10 分）

### 三、综合题，共 35 分

1. MoveIt 报 SUCCESS，但实机控制器拒绝轨迹，分层排查。（15 分）
2. 新 planner 成功率提高但路径更贴障碍，怎样判断是否更好？（20 分）

## 第 27 章：参考答案

路径 $q(s)$ 配时间标度 $s(t)$ 得轨迹，速度/加速度同时依赖路径导数和时间导数。三次可满足位置速度，五次还能满足加速度，因而起终更平滑。多关节同步改变 duration，使各关节导数缩放，必须全部重算。

碰撞集合可能位于两个采样节点之间，机械臂 link 扫过工作空间很大，所以要边插值或连续碰撞。RRT-Connect 单次查询快，PRM 预建图适合静态多查询。概率完备只表示时间趋无限找到存在路径的概率趋 1，不保证有限时间、最优或数值模型正确。

attached object 随末端运动，从 world 中移除，允许指定 touch links，但仍与其他环境碰撞。detach 后以正确世界位姿放回。

五次轨迹测试端点 q/v/a、T 缩放、有限差分、单关节/多关节、零位移、非法 duration、时间边界、非单调输入和 jerk。segment 测试构造两个安全关节状态但中点 link 穿矩形障碍，粗检查应失败用例，细分应发现。

控制器拒绝先看 error code、joint names/order、时间严格递增、首状态容差、缺失/超限速度加速度；再查 controller active、接口、clock 和 scene。不要重新规划掩盖格式错误。planner 比较需同时报告成功、P95 时间、路径长度、平滑、最小净空、连续碰撞、执行跟踪和任务成功；净空低于误差余量则不能发布，即使成功率高。

## 本篇完成标准

完成本篇后，应能生成并验证多关节平滑轨迹；能解释路径与时间参数化；能构造保守碰撞模型和连续边检查；能实现 RRT-Connect 教学版本并理解 PRM/轨迹优化边界；能正确维护 PlanningScene 和 attached object；能对规划结果做独立限制、碰撞、净空和 scene-version 门禁。

下一阶段将进入机械臂动力学与控制：质量矩阵、Coriolis、重力、递归 Newton-Euler、PID/前馈、计算力矩、阻抗/导纳和实时 ros2_control。
