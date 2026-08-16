---
title: 第四册第一篇：机械臂建模、运动学与数值 IK
description: 系统讲解关节与坐标系、DH/POE、正运动学、Jacobian、奇异性、解析/数值逆运动学、冗余与标定。
---

# 第四册第一篇：机械臂建模、运动学与数值 IK

机械臂规划和控制的第一前提是模型正确。一个关节轴方向、零位偏移或网格单位错误，可能让 RViz 看起来接近真实，却让末端误差随姿态变化；IK 求解器仍可能返回数值解，但执行后偏离目标甚至碰撞。

本篇建立从物理关节到可验证数学模型的完整链路：定义 frame 和关节正方向；用 DH 或 POE 计算正运动学；推导几何 Jacobian；分析奇异和可操作度；实现解析及阻尼数值 IK；处理关节限位、冗余和多解；最后通过真实测量、URDF 和自动测试验证模型。

## 第 1 章：机械臂的状态与坐标系

### 1.1 关节类型

转动关节以角度 $q_i$ 为变量，移动关节以位移 $q_i$ 为变量。固定关节不增加自由度，mimic 关节由其他关节按比例驱动。连续转动关节没有位置上下限，但软件仍需处理电缆和机构限制。

配置向量

$$
q=[q_1,\ldots,q_n]^T
$$

必须绑定关节名称顺序。只传一个裸数组，在 URDF、控制器和模型关节顺序不同时会产生静默错误。工程接口使用 `{joint_name: value}` 或携带 names，并在边界转换为固定顺序数组。

### 1.2 常用 frame

- `world`：工作站或全局参考。
- `base_link`：机械臂安装基座。
- link frame：每个刚性连杆参考。
- flange：机械臂法兰。
- tool0/TCP：工具中心点。
- camera optical frame：视觉传感器。

法兰到 TCP 的工具变换会随夹爪、焊枪或吸盘改变。规划到 flange 还是 TCP 必须明确。手眼标定得到的是 camera 与 flange/base 的关系，不应通过改 joint origin 补偿。

### 1.3 右手系和单位

ROS 使用米、弧度和右手系。CAD/网格常以毫米保存。旋转关节正方向按右手规则绕 joint axis。每个关节做小幅正向 jog，观察编码器、TF 和实机方向是否一致。

## 第 2 章：刚体变换复习

齐次变换

$$
{}^aT_b=
\begin{bmatrix}
{}^aR_b&{}^at_b\\0&1
\end{bmatrix}
$$

把 b frame 点转换到 a frame。链式关系

$$
{}^0T_n={}^0T_1{}^1T_2\cdots{}^{n-1}T_n.
$$

逆变换

$$
T^{-1}=\begin{bmatrix}R^T&-R^Tt\\0&1\end{bmatrix}.
$$

代码中 `T_base_tool` 表示把 tool 坐标转换到 base，不能同时用它表示 tool 在 base 的“方向相反”语义。用已知 TCP 原点变换验证方向。

## 第 3 章：标准 DH 建模

标准 DH 用四个参数描述从 frame $i-1$ 到 $i$：绕 $z_{i-1}$ 旋转 $\theta_i$，沿 $z_{i-1}$ 平移 $d_i$，沿 $x_i$ 平移 $a_i$，绕 $x_i$ 旋转 $\alpha_i$。

$$
{}^{i-1}T_i=
R_z(\theta_i)T_z(d_i)T_x(a_i)R_x(\alpha_i).
$$

矩阵为

$$
\begin{bmatrix}
c_\theta&-s_\theta c_\alpha&s_\theta s_\alpha&a c_\theta\\
s_\theta&c_\theta c_\alpha&-c_\theta s_\alpha&a s_\theta\\
0&s_\alpha&c_\alpha&d\\
0&0&0&1
\end{bmatrix}.
$$

修改 DH 的变换顺序不同，参数表不能混用。DH frame 不一定与 URDF link frame 相同，需要固定变换连接。

```python
import numpy as np

def dh_transform(a, alpha, d, theta):
    cth, sth = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [cth, -sth * ca,  sth * sa, a * cth],
        [sth,  cth * ca, -cth * sa, a * sth],
        [0.0,       sa,        ca,       d],
        [0.0,      0.0,       0.0,     1.0],
    ])
```

### 3.1 参数偏置

转动关节常为 $\theta_i=q_i+\theta_{0i}$，移动关节为 $d_i=q_i+d_{0i}$。零位偏置来自装配和编码器标定。把偏置散落在驱动与运动学两处会重复补偿，必须定义唯一来源。

## 第 4 章：通用 DH 正运动学

```python
from dataclasses import dataclass

@dataclass(frozen=True)
class DHJoint:
    a: float
    alpha: float
    d: float
    theta_offset: float
    joint_type: str = "revolute"

def forward_kinematics_dh(joints, q):
    q = np.asarray(q, np.float64).reshape(-1)
    if len(joints) != len(q) or not np.isfinite(q).all():
        raise ValueError("joint model and finite q must have equal length")
    transform = np.eye(4)
    transforms = [transform.copy()]
    for joint, value in zip(joints, q):
        if joint.joint_type == "revolute":
            theta = joint.theta_offset + value
            d = joint.d
        elif joint.joint_type == "prismatic":
            theta = joint.theta_offset
            d = joint.d + value
        else:
            raise ValueError(f"unsupported joint type {joint.joint_type}")
        transform = transform @ dh_transform(joint.a, joint.alpha, d, theta)
        transforms.append(transform.copy())
    return transforms
```

返回每个中间 frame 便于 Jacobian 和可视化。测试每个旋转块 $R^TR=I$、$\det R=1$，最后一行为 `[0,0,0,1]`，并与手算零位和 URDF TF 对照。

## 第 5 章：POE 指数积方法

DH 依赖相邻轴放置 frame，复杂机构容易混乱。POE 使用空间螺旋轴 $S_i$ 和零位末端 $M$：

$$
T(q)=e^{[S_1]q_1}e^{[S_2]q_2}\cdots e^{[S_n]q_n}M.
$$

转动关节螺旋轴

$$
S=\begin{bmatrix}\omega\\v\end{bmatrix},
\qquad v=-\omega\times q_0,
$$

$q_0$ 是轴上一点。移动关节 $\omega=0$，$v$ 为移动方向。

POE 与 Lie 群优化自然连接，轴可以统一表达在 space 或 body frame。两种形式不能混用。学习项目可分别实现 DH 和 POE，同一随机关节配置的 FK 应一致，这是一项强验证。

## 第 6 章：二维二连杆解析模型

连杆长度 $l_1,l_2$：

$$
x=l_1\cos q_1+l_2\cos(q_1+q_2),
$$

$$
y=l_1\sin q_1+l_2\sin(q_1+q_2),
$$

$$
\phi=q_1+q_2.
$$

位置 Jacobian

$$
J_p=
\begin{bmatrix}
-l_1\sin q_1-l_2\sin(q_1+q_2)&-l_2\sin(q_1+q_2)\\
l_1\cos q_1+l_2\cos(q_1+q_2)&l_2\cos(q_1+q_2)
\end{bmatrix}.
$$

它是所有 IK、奇异和控制概念的最小可视化平台。

## 第 7 章：几何 Jacobian

Jacobian 把关节速度映射到末端 twist：

$$
\begin{bmatrix}v_e\\\omega_e\end{bmatrix}=J(q)\dot q.
$$

对 base frame 中的转动关节轴 $z_i$、轴原点 $p_i$、末端原点 $p_e$：

$$
J_{v,i}=z_i\times(p_e-p_i),\qquad J_{\omega,i}=z_i.
$$

移动关节：

$$
J_{v,i}=z_i,\qquad J_{\omega,i}=0.
$$

```python
def geometric_jacobian_dh(joints, q):
    transforms = forward_kinematics_dh(joints, q)
    end_position = transforms[-1][:3, 3]
    J = np.zeros((6, len(joints)))
    for index, joint in enumerate(joints):
        axis = transforms[index][:3, 2]
        origin = transforms[index][:3, 3]
        if joint.joint_type == "revolute":
            J[:3, index] = np.cross(axis, end_position - origin)
            J[3:, index] = axis
        else:
            J[:3, index] = axis
    return J
```

上述公式假设标准 DH 的关节轴是变换前 frame 的 z 轴。修改 DH 或 URDF 任意 axis 时需按真实轴表达。

## 第 8 章：Jacobian 验证

位置部分可以中心差分 FK。姿态不能直接对旋转矩阵元素差分后塞成角速度，应计算相对旋转的 Log：

$$
\omega\approx\frac{\operatorname{Log}(R(q+\epsilon e_i)R(q-\epsilon e_i)^T)}{2\epsilon}.
$$

根据空间/身体角速度约定，相对矩阵顺序不同。对多个随机 q 扫描 $\epsilon=10^{-3}\ldots10^{-8}$，寻找截断和浮点误差之间稳定区。靠关节限位时使用单边差分并标记精度降低。

验证还包括力矩对偶：随机末端 wrench $F$，计算 $\tau=J^TF$，检查虚功

$$
F^TV=\tau^T\dot q.
$$

若不相等，常见原因是 Jacobian frame 或 wrench 排列不一致。

## 第 9 章：奇异性

当 $J$ 降秩时，某些任务空间方向无法由关节速度产生。二连杆伸直或完全折叠时 $\det J_p=0$。接近奇异时，小任务速度需要巨大关节速度。

用 SVD

$$
J=U\Sigma V^T
$$

分析奇异值。最小奇异值接近零表示弱方向；条件数 $\sigma_{max}/\sigma_{min}$ 衡量各向异性。矩阵非方时不要只看 determinant。

### 9.1 可操作度

Yoshikawa 可操作度

$$
w(q)=\sqrt{\det(JJ^T)}.
$$

它随单位、选取的任务维度和尺度变化。把线速度米/秒和角速度弧度/秒放在同一矩阵求 determinant 需要定义特征长度。更可靠做法是报告归一化 Jacobian 的奇异值和具体弱方向。

## 第 10 章：解析逆运动学

二连杆目标 $(x,y)$，余弦定理：

$$
c_2=\frac{x^2+y^2-l_1^2-l_2^2}{2l_1l_2}.
$$

若 $|c_2|>1$，目标不可达。数值误差可能得到 $1+10^{-12}$，在容差内 clip；明显越界必须拒绝。

$$
q_2=\operatorname{atan2}(\pm\sqrt{1-c_2^2},c_2),
$$

$$
q_1=\operatorname{atan2}(y,x)-
\operatorname{atan2}(l_2\sin q_2,l_1+l_2\cos q_2).
$$

正负对应肘上/肘下两解。再按关节限位、碰撞、与当前状态距离和可操作度排序。

```python
def ik_2link(position, lengths, tolerance=1e-10):
    x, y = map(float, position)
    l1, l2 = map(float, lengths)
    if l1 <= 0 or l2 <= 0:
        raise ValueError("link lengths must be positive")
    c2 = (x*x + y*y - l1*l1 - l2*l2) / (2*l1*l2)
    if c2 < -1 - tolerance or c2 > 1 + tolerance:
        return []
    c2 = np.clip(c2, -1.0, 1.0)
    solutions = []
    for sign in (1.0, -1.0):
        s2 = sign * np.sqrt(max(0.0, 1.0 - c2*c2))
        q2 = np.arctan2(s2, c2)
        q1 = np.arctan2(y, x) - np.arctan2(l2*s2, l1 + l2*c2)
        solution = np.array([wrap_angle(q1), wrap_angle(q2)])
        if not any(np.allclose(solution, old, atol=1e-9) for old in solutions):
            solutions.append(solution)
    return solutions
```

## 第 11 章：任务空间位姿误差

平移误差

$$
e_p=p_d-p(q).
$$

旋转误差不能用欧拉角直接相减。空间 frame 误差可用

$$
e_R=\operatorname{Log}(R_dR(q)^T)^\vee.
$$

或 body frame 使用 $\operatorname{Log}(R(q)^TR_d)$。误差与 Jacobian 必须在同一 frame。靠近 180° 的 Log 有轴歧义，数值 IK 需要谨慎初值和步长。

组合误差时米和弧度需要权重：

$$
e=\begin{bmatrix}w_pe_p\\w_Re_R\end{bmatrix}.
$$

权重来自任务容差，不是让曲线好看的任意系数。

## 第 12 章：伪逆 IK

局部线性化

$$
e\approx J\Delta q.
$$

最小二乘增量

$$
\Delta q=J^+e.
$$

SVD 伪逆将非零奇异值取倒数。小奇异值会被放大，需阈值截断，但硬截断在阈值处不连续。

## 第 13 章：阻尼最小二乘

DLS 解

$$
\Delta q=J^T(JJ^T+\lambda^2I)^{-1}e
$$

或解 $(J^TJ+\lambda^2I)\Delta q=J^Te$。阻尼限制奇异附近关节增量，但引入任务误差。可根据最小奇异值自适应增大 $\lambda$。

```python
from dataclasses import dataclass

@dataclass(frozen=True)
class IKResult:
    success: bool
    reason: str
    q: np.ndarray
    iterations: int
    error_norm: float
    minimum_singular_value: float

def damped_step(J, error, damping):
    task_dim = J.shape[0]
    system = J @ J.T + damping * damping * np.eye(task_dim)
    return J.T @ np.linalg.solve(system, error)

def solve_position_ik(fk_position, jacobian_position, target, seed,
                      lower, upper, tolerance=1e-5, max_iterations=100,
                      damping=1e-3, maximum_step=0.2):
    q = np.clip(np.asarray(seed, np.float64), lower, upper)
    target = np.asarray(target, np.float64)
    minimum_sv = 0.0
    for iteration in range(max_iterations):
        error = target - fk_position(q)
        error_norm = float(np.linalg.norm(error))
        if error_norm < tolerance:
            return IKResult(True, "converged", q, iteration,
                            error_norm, minimum_sv)
        J = jacobian_position(q)
        singular_values = np.linalg.svd(J, compute_uv=False)
        minimum_sv = float(singular_values[-1])
        adaptive = max(damping, 0.02 * max(0.0, 0.05 - minimum_sv))
        delta = damped_step(J, error, adaptive)
        norm = np.linalg.norm(delta)
        if norm > maximum_step:
            delta *= maximum_step / norm
        candidate = np.clip(q + delta, lower, upper)
        if np.linalg.norm(candidate - q) < 1e-10:
            return IKResult(False, "stalled_at_limits", q, iteration,
                            error_norm, minimum_sv)
        q = candidate
    final_error = float(np.linalg.norm(target - fk_position(q)))
    return IKResult(False, "maximum_iterations", q, max_iterations,
                    final_error, minimum_sv)
```

这是位置 IK 教学实现。完整位姿、连续关节、碰撞和更复杂限位应使用成熟求解器，但门禁和测试原则相同。

## 第 14 章：步长和线搜索

直接应用 $\Delta q$ 只在局部线性近似有效。最大步长避免大跳，回溯线搜索确保误差下降：从 $\alpha=1$ 开始，若新误差未下降则乘 0.5。若多次失败，报告局部模型无效或更换 seed。

误差下降不代表目标全局可达；求解器可能停在关节限位或局部极小。必须返回 reason、最终误差、最小奇异值和限位状态，而不是一个 bool。

## 第 15 章：冗余和零空间

当关节自由度多于任务维度，通解可写

$$
\dot q=J^+\dot x+(I-J^+J)z.
$$

第二项位于 Jacobian 零空间，理论上不改变一阶末端运动。选择

$$
z=-k\nabla H(q)
$$

可优化关节限位、可操作度或姿态偏好。

关节中心代价

$$
H(q)=\sum_i\left(
\frac{q_i-q_{mid,i}}{q_{max,i}-q_{min,i}}
\right)^2.
$$

接近奇异和使用阻尼伪逆时，$I-J^+J$ 不再是精确投影，零空间任务可能影响主任务。应设优先级、步长并验证末端误差。

## 第 16 章：关节限位

仅在每步 `clip` 会让多个关节卡在边界，IK 停滞。更好的策略包括限位势函数、active-set、约束优化或为近限位关节加权。连续关节需要选择与 seed 最近的等价角，避免从 $179°$ 跳到 $-179°$ 造成规划大转。

位置、速度、加速度和 jerk 都有限制。IK 只检查位置，后续轨迹仍可能因速度或加速度不可执行。

## 第 17 章：多初值与解排序

非线性 IK 对 seed 敏感。使用当前关节状态、命名姿态、随机低差异样本和历史解多初值求解。每个候选检查 FK 残差、限位、自碰撞、环境碰撞和数值质量。

排序代价可包含：与当前状态的加权距离、关节限位余量、可操作度、肘部姿态、预测轨迹碰撞和任务偏好。最接近的关节解不一定最易规划。

## 第 18 章：工作空间

解析二连杆可达半径

$$
|l_1-l_2|\le\sqrt{x^2+y^2}\le l_1+l_2.
$$

三维机械臂工作空间受关节限位、自碰撞、工具、环境和姿态约束。位置可达不代表任意方向可达。用大量关节采样建立 workspace 可视化时，采样密度只提供近似，不能把没采到直接判不可达。

## 第 19 章：URDF 与数学模型对照

URDF joint 给 parent/child、origin、axis、limit。FK 应从 URDF 树按 joint origin 和 axis 构造，非串联机构还要处理固定分支。与 DH 对照时，比较具体 link frame 变换，不比较参数表外观。

自动生成随机 q，分别用自研 FK、KDL/Pinocchio/MoveIt RobotState 计算各 link 位姿，平移和旋转误差应在容差内。发现不一致从第一个错误 link 开始排查。

## 第 20 章：运动学标定

实际参数 $\pi$ 包括零位偏置、连杆长度和安装变换。测得多个关节配置 $q_k$ 的末端位姿 $T_k^{meas}$，最小化

$$
\min_\pi\sum_k\|\operatorname{Log}((T_k^{meas})^{-1}T(q_k;\pi))\|_W^2.
$$

数据需充分激励各关节和工作空间。某些参数组合不可辨识，不能同时自由优化所有 DH 参数。先做可观性/Jacobian 秩分析，固定规范。

标定集与验证集分开。训练残差下降但验证不改善，说明过拟合测量噪声或参数耦合。标定不应补偿柔性、回差和负载变形而假装刚性参数变化。

## 第 21 章：TCP 和手眼误差分离

末端偏差可能来自关节模型、TCP、base 安装和 camera 外参。用外部跟踪测 flange 可隔离关节运动学；固定 flange 更换工具可标定 TCP；视觉目标在不同机械臂姿态下测量可诊断手眼。

若所有姿态偏移近似固定，优先检查 base/TCP；若误差随关节角周期变化，检查零位、长度和编码器；随负载和方向变化，可能是回差/柔性。

## 第 22 章：速度与力映射

$$
V=J\dot q,
\qquad \tau=J^TF.
$$

第二式来自虚功，不是把速度公式随意转置。$F$ 和 $V$ 的 frame、分量排列必须一致。靠奇异位形，某些末端力可能对应很小关节力矩，另一些方向需要极大力矩/无法控制。

静力映射忽略动力学和重力，不能直接作为实际力矩控制器。它用于理解 Jacobian 对偶，后续控制篇加入质量矩阵、Coriolis 和重力。

## 第 23 章：故障案例

### 零位正确，其他姿态末端偏差增大

检查连杆长度、关节轴和 joint origin，而不是只加 TCP 偏移。采集误差对关节角曲线，定位第一个出现偏差的 link。

### IK 成功但实机朝向相反

检查目标变换方向、空间/body 旋转误差、四元数顺序和 optical frame。用一个非对称工具轴画三根坐标轴验证。

### 关节在奇异附近突然高速

检查最小奇异值、阻尼、步长和速度限幅。任务层改变路径避开奇异，比无限增大阻尼更有效。

### 随机 seed 偶尔给出碰撞解

IK 不负责路径碰撞时这是预期风险。每个候选做自碰撞/环境检查，再由规划器连接；保存 seed 和候选排序以复现。

## 第 24 章：综合实验一——二连杆工作台

实现 FK、解析 Jacobian、解析 IK、伪逆和 DLS IK。扫描整个关节空间，画末端工作空间、最小奇异值和两解分支。对不可达、边界、奇异和关节限位目标建立测试。

比较解析/数值解成功率、误差、迭代、解分支和奇异附近增量。故意把 $l_2$ 改错 5%，观察末端误差怎样随姿态变化。

## 第 25 章：综合实验二——六轴模型验证

从 URDF 加载六轴机械臂，用 Pinocchio/KDL/MoveIt 作为参考。随机生成至少 10,000 组限位内 q，比各 link FK 和 Jacobian。统计 P50/P95/max，保存最差配置。

对随机可达目标运行多初值 IK，报告成功、限位、奇异、碰撞和超时。目标由已知 q 的 FK 生成，因此至少存在一个运动学解；求解失败能暴露数值或 seed 覆盖问题。

## 第 26 章：阶段考试

建议限时 180 分钟，满分 100 分。

### 一、理论题，共 35 分

1. 标准 DH 四步变换及与修改 DH 的区别。（5 分）
2. 推导转动/移动关节几何 Jacobian 列。（5 分）
3. 说明空间与 body 位姿误差的区别。（5 分）
4. SVD 怎样揭示奇异性？（5 分）
5. 推导二连杆解析 IK 和多解。（5 分）
6. 阻尼最小二乘解决什么、牺牲什么？（5 分）
7. 冗余零空间怎样用于限位回避？（5 分）

### 二、代码题，共 30 分

1. 为 DH FK 写十个边界/性质测试。（10 分）
2. 用旋转 Log 做数值 Jacobian 验证。（10 分）
3. 为 DLS IK 设计状态码、限位、奇异和不可达测试。（10 分）

### 三、综合题，共 35 分

1. RViz 与实机零位一致，但末端误差随 q2 增大，设计排查。（15 分）
2. IK 成功率 99%，抓取规划仍频繁失败，说明缺少哪些评价并设计实验。（20 分）

## 第 27 章：参考答案

标准 DH 顺序为 $R_z(\theta)T_z(d)T_x(a)R_x(\alpha)$，修改 DH 顺序和 frame 定义不同，参数不能混用。转动轴对末端线速度贡献 $z\times(p_e-p)$、角速度 $z$；移动轴贡献线速度 $z$、角速度零。

空间误差/Jacobian 表达在基座等空间 frame，body 版本表达在末端 frame，相对旋转顺序不同。SVD 的小奇异值对应弱任务方向，右奇异向量给关节方向，左奇异向量给任务方向。

二连杆由余弦定理得到 $c_2$，$s_2$ 正负产生肘上/肘下，再解 $q_1$。DLS 在小奇异值上限制放大，换取任务偏差。冗余解把限位代价负梯度投影到 $(I-J^+J)$，但阻尼下要验证主任务耦合。

FK 测试包括单位/零位、单关节、正交旋转、det=1、逆变换、链组合、转动周期、移动线性、非法类型、非有限输入和参考库随机对照。数值 Jacobian 对位置用中心差分，对旋转用相对 $SO(3)$ Log，并扫描 epsilon。

DLS 状态区分 converged、maximum_iterations、stalled_at_limits、singular、invalid_target 和 collision_rejected。测试可达已知目标、不可达目标、奇异边界、多 seed、多解、限位卡住、步长限幅和误差单调。

误差随 q2 增大提示 q2 后链路参数：轴、origin、连杆长度、零偏或工具。固定 q1 扫 q2，用外部测量各 link，找首个偏差；对比 URDF/reference FK；不要先改 TCP。IK 99% 只说明数值求解，需要候选质量、限位余量、奇异值、自碰撞、环境碰撞、从当前状态的规划成功、轨迹限制和实机误差。按目标位置/姿态/seed 切片，逐层报告。

## 本篇完成标准

完成本篇后，应能从物理关节建立 DH 或 POE 模型；能与 URDF/参考库逐 link 验证 FK；能推导并数值检查 Jacobian；能解释奇异和弱方向；能实现解析二连杆和通用 DLS IK；能处理多解、限位、冗余和状态码；能通过真实测量分离关节模型、TCP 和手眼误差。

下一阶段将学习关节/笛卡尔轨迹、速度/加速度/jerk 约束、时间参数化、RRT/PRM/轨迹优化、碰撞检测与 PlanningScene。
