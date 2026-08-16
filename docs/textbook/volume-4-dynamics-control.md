---
title: 第四册第三篇：机械臂动力学、接触控制与 ros2_control
description: 从 Lagrange/Newton-Euler 动力学到 PID、前馈、计算力矩、阻抗/导纳、力传感和实时控制工程。
---

# 第四册第三篇：机械臂动力学、接触控制与 ros2_control

运动学描述“关节怎样决定末端几何”，动力学进一步描述“产生这些运动需要多少力矩”。在低速轻载演示中，位置控制器可能掩盖动力学模型缺陷；速度提高、负载变化或发生接触后，惯性、重力、摩擦、结构柔性、通信延迟和采样抖动会决定系统能否稳定。

本篇先建立机械臂标准动力学方程，讲解 Lagrange 与递归 Newton-Euler；随后从单关节 PID 和重力前馈扩展到计算力矩与任务空间阻抗；最后讨论力传感、碰撞、离散稳定性、实时线程和 `ros2_control` 硬件/控制器接口。

## 第 1 章：动力学方程

n 自由度刚性机械臂常写为

$$
M(q)\ddot q+C(q,\dot q)\dot q+g(q)+\tau_f(\dot q)=
\tau+J(q)^TF_{ext}.
$$

- $M(q)$：质量矩阵，对称正定。
- $C(q,\dot q)\dot q$：Coriolis 和离心项。
- $g(q)$：重力力矩。
- $\tau_f$：摩擦、回差等非理想项。
- $\tau$：执行器关节力矩。
- $J^TF_{ext}$：外部末端 wrench 映射到关节。

不同资料对外力符号、$C$ 的具体矩阵表示和 frame 有不同约定。物理组合 $C\dot q$ 更具唯一意义；代码需通过能量和仿真验证，而不是比较单个 C 元素。

## 第 2 章：质量矩阵性质

系统动能

$$
K=\frac12\dot q^TM(q)\dot q.
$$

对任意非零 $\dot q$，$K>0$，因此物理可行模型的 $M$ 对称正定。数值测试随机 q，检查

```python
def validate_mass_matrix(M, symmetry_tolerance=1e-9,
                         eigenvalue_tolerance=1e-9):
    M = np.asarray(M, np.float64)
    if M.ndim != 2 or M.shape[0] != M.shape[1]:
        return False, "shape"
    if not np.isfinite(M).all():
        return False, "non_finite"
    if not np.allclose(M, M.T, atol=symmetry_tolerance):
        return False, "not_symmetric"
    if np.linalg.eigvalsh(M).min() <= eigenvalue_tolerance:
        return False, "not_positive_definite"
    return True, "ok"
```

惯量单位是 kg·m²，CAD 以毫米导出但按米解释会造成百万倍惯量误差。URDF inertia 还必须关于 link inertial frame 表达，质心 origin 错也会改变动力学。

## 第 3 章：Lagrange 方法

势能 $U(q)$，Lagrangian $L=K-U$。每个广义坐标满足

$$
\frac{d}{dt}\frac{\partial L}{\partial\dot q_i}
-\frac{\partial L}{\partial q_i}=\tau_i.
$$

展开得到 $M,C,g$。Lagrange 适合推导低自由度模型和理解能量结构，但符号表达随自由度快速膨胀。可用 SymPy 推导二连杆，再与 Pinocchio/RBDL 等库的数值结果对照。

重力项与势能关系

$$
g(q)=\frac{\partial U}{\partial q}
$$

的符号取决于方程与势能定义。将机械臂静止在多个姿态，计算保持力矩并与仿真/实测比较，是直接验证。

## 第 4 章：二连杆动力学

平面二连杆质量 $m_1,m_2$，质心距离 $l_{c1},l_{c2}$，惯量 $I_1,I_2$，可写

$$
M_{11}=I_1+I_2+m_1l_{c1}^2+m_2(l_1^2+l_{c2}^2
+2l_1l_{c2}\cos q_2),
$$

$$
M_{12}=M_{21}=I_2+m_2(l_{c2}^2+l_1l_{c2}\cos q_2),
$$

$$
M_{22}=I_2+m_2l_{c2}^2.
$$

令 $h=m_2l_1l_{c2}\sin q_2$，Coriolis/离心组合的一种写法

$$
C\dot q=
\begin{bmatrix}
-h(2\dot q_1\dot q_2+\dot q_2^2)\\
h\dot q_1^2
\end{bmatrix}.
$$

重力方向选定后可推导 $g(q)$。用这一最小模型进行前向仿真、能量和控制实验。

## 第 5 章：递归 Newton-Euler

Newton-Euler 先从基座向末端递推各 link 角速度、角加速度和线加速度，再从末端向基座递推力和力矩，复杂度 $O(n)$。它适合逆动力学：给定 $q,\dot q,\ddot q$ 求 $\tau$。

Composite Rigid Body Algorithm 高效计算 $M(q)$，Articulated Body Algorithm 高效做前向动力学。生产系统使用经过验证的刚体动力学库；学习时实现二连杆和小链，理解 frame、质心和惯量转换。

## 第 6 章：前向与逆动力学

逆动力学：

$$
\tau=M(q)\ddot q_d+C(q,\dot q)\dot q+g(q)+\tau_f.
$$

前向动力学：

$$
\ddot q=M(q)^{-1}
[\tau-C\dot q-g-\tau_f+J^TF_{ext}].
$$

代码中不要显式求 $M^{-1}$，用线性方程求解。每步积分前检查 M 条件数和输入有限性。

## 第 7 章：数值积分与能量测试

显式 Euler 简单但对刚性控制系统可能不稳定：

$$
\dot q_{k+1}=\dot q_k+\ddot q_k\Delta t,
\qquad q_{k+1}=q_k+\dot q_k\Delta t.
$$

半隐式 Euler 先更新速度再用新速度更新位置，通常能量行为更好；RK4 精度高但每步多次动力学计算。仿真步长必须远小于最快系统动态。

无重力、无摩擦、无外力、无控制时，总能量应近似守恒。能量持续增长可能是积分步长、Coriolis 符号或模型错误。带粘性阻尼时机械能应下降。

## 第 8 章：摩擦与回差

常见摩擦模型

$$
\tau_f=b\dot q+\tau_c\operatorname{sign}(\dot q),
$$

包含粘性和 Coulomb 摩擦。零速附近 sign 不连续，可用平滑近似或更复杂 Stribeck 模型。齿轮回差和柔性有记忆，不能由速度单变量完全描述。

摩擦辨识可在不同恒速下测稳态力矩，先补偿重力。模型过拟合某负载/温度会在另一工况产生错误前馈，因此保留反馈和限幅。

## 第 9 章：执行器与传动

电机力矩 $\tau_m=K_ti$，经减速比 N 和效率映射到关节。反射到电机/关节侧的惯量随 $N^2$ 缩放。电流限制、速度-力矩曲线、热限制和母线电压共同决定能力。

“URDF effort limit=100”不证明执行器所有速度下都能输出 100 Nm。轨迹验收需结合电机曲线和持续/峰值时间。高减速器提高力矩，也增加摩擦、回差和不可回驱性。

## 第 10 章：单关节 PID

位置误差 $e=q_d-q$：

$$
u=K_pe+K_i\int e,dt+K_d(\dot q_d-\dot q).
$$

实现时注意采样 dt、导数噪声、输出饱和和积分 anti-windup。

```python
from dataclasses import dataclass

@dataclass
class PIDState:
    integral: float = 0.0
    previous_measurement: float | None = None

def pid_step(target, measurement, target_velocity, measured_velocity,
             dt, kp, ki, kd, limit, state, integral_limit):
    if dt <= 0 or limit <= 0:
        raise ValueError("dt and output limit must be positive")
    error = target - measurement
    derivative_error = target_velocity - measured_velocity
    candidate_integral = np.clip(
        state.integral + error * dt, -integral_limit, integral_limit
    )
    unsaturated = kp*error + ki*candidate_integral + kd*derivative_error
    output = float(np.clip(unsaturated, -limit, limit))
    if output == unsaturated or np.sign(error) != np.sign(unsaturated):
        state.integral = candidate_integral
    state.previous_measurement = measurement
    return output
```

对测量而不是 error 直接差分可避免设定值跳变引起 derivative kick；若已有可靠速度测量可直接使用。

## 第 11 章：PID 调试

先确认控制符号、单位和安全限幅。低 Kp 单关节小幅阶跃，确认负反馈；逐步增加 P 到响应足够；加入 D 抑制振荡；只有明确稳态偏差再加入 I。

报告上升时间、超调、稳定时间、稳态误差、RMS、最大输出和饱和比例。空载调参不能直接用于最大负载。高频噪声下 D 项需滤波，但滤波增加相位延迟。

## 第 12 章：重力与速度前馈

$$
\tau=g(q)+K_p(q_d-q)+K_d(\dot q_d-\dot q).
$$

重力前馈承担静态负载，反馈只修模型误差，可降低 Kp 和稳态误差。模型质量或 payload 错时前馈本身造成偏差。先在多个静态姿态低增益测试，验证力矩方向和大小。

速度/加速度前馈来自期望轨迹和惯性模型，必须使用与当前负载一致的参数。轨迹数值微分噪声大，应从时间参数化器直接获得导数。

## 第 13 章：计算力矩控制

选择虚拟加速度

$$
v=\ddot q_d+K_d(\dot q_d-\dot q)+K_p(q_d-q),
$$

控制

$$
\tau=M(q)v+C(q,\dot q)\dot q+g(q).
$$

模型精确时闭环近似解耦线性二阶系统。实际质量、摩擦和负载误差会破坏抵消，所以需要鲁棒余量和力矩限幅。奇异与碰撞由规划/安全层管理，计算力矩不自动保证。

## 第 14 章：任务空间控制

末端 twist $V=J\dot q$，加速度

$$
\dot V=J\ddot q+\dot J\dot q.
$$

operational-space inertia

$$
\Lambda=(JM^{-1}J^T)^{-1}.
$$

任务 wrench 可映射 $\tau=J^TF$，再加重力和零空间。靠奇异时 $JM^{-1}J^T$ 病态，需要阻尼、任务降维或避开奇异。

## 第 15 章：阻抗控制

期望机器人在接触下表现为质量-弹簧-阻尼：

$$
M_d\ddot e+D_d\dot e+K_de=F_{ext}.
$$

简化笛卡尔阻抗输出期望 wrench

$$
F_c=K(x_d-x)+D(\dot x_d-\dot x)+F_{ff}.
$$

再用 $J^T$ 转关节力矩。位置和旋转误差必须在同一 frame，刚度单位分别 N/m 与 Nm/rad。

高刚度在有限采样、延迟和硬环境中可能不稳定。先低刚度、低速度和软环境逐级增加，并设置位移、速度、力和能量限幅。

## 第 16 章：导纳控制

位置控制型机器人不能直接命令力矩时，导纳把测得外力通过虚拟动力学生成位置修正：

$$
M_d\ddot x_c+D_d\dot x_c+K_dx_c=F_{ext}.
$$

外层积分产生位置目标，内层位置控制跟踪。导纳适合刚性位置接口，但外力噪声、漂移和内环延迟会影响。目标修正仍需 IK、限位和碰撞保护。

## 第 17 章：力/力矩传感器

六轴 F/T 传感器输出 wrench，需零偏、温漂、重力和工具载荷补偿。换工具后质量、质心和传感器到 TCP 变换改变。wrench 从 frame A 变换到 B 需使用伴随对偶，不只是旋转力向量；平移会让力产生附加力矩。

静止多姿态采集可估计工具质量、质心和偏置。验证补偿后不同姿态自由空间 wrench 接近零，并报告残余误差。

## 第 18 章：接触检测与碰撞

接触检测可用 F/T、关节力矩残差、驱动电流或观测器。阈值随姿态、速度和负载变化。只用固定电流阈值会在加速时误报，在慢速夹持时漏报。

碰撞响应状态机：正常 → suspicious → contact_confirmed → controlled_stop/retract → human_reset。接触确认前可降速，确认后停止或沿安全方向退让。退让方向必须避免二次夹伤。

## 第 19 章：离散时间稳定性

连续理论稳定不保证数字控制稳定。采样、零阶保持、通信延迟和滤波引入相位。刚度越高，要求控制频率越高、延迟越低。控制周期 P99 和 deadline miss 比平均频率重要。

接触环境刚度未知时，passivity/energy tank 等方法用于保证能量行为。至少监控控制器注入功率 $P=F^TV$ 和累计能量，异常时降低刚度或停止。

## 第 20 章：ros2_control 架构

Hardware Component 暴露 state/command interfaces；Controller Manager 周期性 read → update controllers → write；controller plugin 实现控制律；broadcaster 发布状态。

硬件可以是 system、sensor 或 actuator。多关节机械臂通常是 system，F/T 是 sensor。接口名称如 position/velocity/effort 必须与 controller 配置匹配。

## 第 21 章：硬件接口生命周期

`on_init` 解析硬件参数，`on_configure` 建立资源，`on_activate` 清状态并允许命令，`read` 获取传感器，`write` 输出命令。activate 前不得发送旧命令；deactivate/error 应进入安全状态。

通信断开、编码器非有限、周期超时和驱动 fault 必须返回错误并触发 controller manager 行为。不要在错误后继续写最后一次非零力矩。

## 第 22 章：实时循环规则

控制 update 中避免：动态内存分配、锁等待、文件/终端日志、网络阻塞、参数服务器访问、模型加载。非实时线程接收目标，通过 realtime buffer 交给实时线程；实时状态通过无锁/预分配结构给发布线程。

预分配 Eigen 矩阵，避免隐式 resize；日志限频并移出实时路径。用 cyclictest/trace 和 controller statistics 记录周期 P50/P95/P99/max、deadline miss 和 read/update/write 分段。

## 第 23 章：控制器切换

从 position trajectory 切到 effort impedance 时，先确认目标和当前状态对齐，实现 bumpless transfer。两个 controller 不能同时 claim 同一 command interface，除非设计了链式控制。

切换失败保持原安全 controller 或停止；不能出现无人 claim 后驱动维持旧力矩。切换测试覆盖运行中、通信抖动、目标未准备和急停。

## 第 24 章：JointTrajectoryController

输入 trajectory joint names、points、positions/velocities/accelerations 和 `time_from_start`。时间严格递增，起点与当前状态容差，goal/trajectory tolerance 配置合理。控制器插值方式和 command interface 决定跟踪。

action success 表示控制器达到配置容差，不代表末端任务完成。外部系统还检查 TCP、夹爪和物体状态。

## 第 25 章：watchdog 与安全限幅

硬件层对命令时间戳设 watchdog，超时进入零速度、制动或安全力矩。每周期检查位置、速度、力矩、温度和通信。软限位在接近硬限位前减速，硬限位/机械挡块是最后保护。

限幅后 anti-windup，且诊断记录哪个关节何种限制。持续饱和说明轨迹或控制器超出能力，不能只让 limiter 长期承担。

## 第 26 章：模型验证实验

### 26.1 重力

在多个静态姿态低速保持，记录关节力矩/电流，与 $g(q)$ 比较。正反接近同姿态区分摩擦和回差。换 payload 重复。

### 26.2 惯性

在安全范围执行不同加速度轨迹，扣除重力/摩擦，比较 $M\ddot q+C\dot q$。不要用控制器命令力矩当作无误差真值。

### 26.3 能量

仿真无耗散自由运动检查能量，加入粘性摩擦应下降。数值积分步长扫描，确认结论不依赖单一步长。

## 第 27 章：控制器基准

对 P、PD+gravity、PID、computed torque 使用相同轨迹、负载和限制。报告关节/TCP RMS、P95/max、超调、相位滞后、力矩 RMS/peak、饱和、能耗和周期。

训练/调参轨迹与测试轨迹隔离。正弦、多项式、快速方向反转和 payload 变化分别测试。

## 第 28 章：接触综合实验

任务：末端从自由空间接近固定平面，检测接触后沿法向保持目标力、沿切向移动。阶段：自由空间位置控制 → 低速接近 → 接触确认 → 力/阻抗控制 → 退出。

指标：峰值冲击、稳态力误差、接触建立时间、切向跟踪、能量、超限和急停。改变环境刚度、速度、控制频率和延迟，画稳定区域。

先仿真软环境，再低刚度实机并设物理缓冲。人员不把手作为接触面。

## 第 29 章：常见故障

### 重力补偿后手臂向上加速

符号、重力方向、关节轴或力矩接口语义错误。低力矩限幅单关节验证，不加反馈硬压。

### 空载稳定，带载振荡

payload 质量/质心错误改变惯性和重力，增益裕量下降。更新模型并降低带宽，测周期和饱和。

### 力信号随姿态变化

工具重力/质心或 wrench frame 补偿错误。多姿态静止标定，检查对偶伴随和平移力矩项。

### 实时循环偶发 20 ms

分段 trace read/update/write，检查日志、内存分配、DDS 回调、驱动阻塞和 CPU 调度。平均 1 ms 不能掩盖 P99 deadline miss。

## 第 30 章：阶段考试

建议限时 180 分钟，满分 100 分。

### 一、理论题，共 35 分

1. 解释 $M,C,g,J^TF$ 各项和 frame。（5 分）
2. 为什么 M 必须对称正定？（5 分）
3. Lagrange 与 Newton-Euler 差异。（5 分）
4. PID anti-windup 为什么必要？（5 分）
5. 计算力矩控制的假设与风险。（5 分）
6. 阻抗和导纳适用接口差异。（5 分）
7. 连续稳定为何不保证离散稳定？（5 分）

### 二、代码题，共 30 分

1. 为质量矩阵和前向动力学写性质测试。（10 分）
2. 实现带 anti-windup 的 PID 并测试饱和恢复。（10 分）
3. 设计实时 buffer、watchdog 和 controller switch 测试。（10 分）

### 三、综合题，共 35 分

1. 位置跟踪仿真优秀但实机带载误差大，分层诊断。（15 分）
2. 提高阻抗刚度后接触振荡，设计安全实验找稳定边界。（20 分）

## 第 31 章：参考答案

$M$ 表示惯性耦合，$C\dot q$ 速度相关，$g$ 重力，$J^TF$ 把同一 frame/排列的外部 wrench 映射关节。动能对任意非零速度为正，所以 M 对称正定。Lagrange 易理解能量和符号推导，Newton-Euler 递归 O(n) 适合数值逆动力学。

输出饱和时积分继续积累，解除后产生大过冲，anti-windup 条件积分或反算。计算力矩依赖准确 M/C/g 和状态，模型/负载错误与延迟会破坏解耦。阻抗通常力矩接口输出力，导纳把力转位置，适合刚性位置内环。

离散采样、延迟、滤波和数值积分引入相位/能量，高刚度可能越过稳定裕量。测试 M 对称正定、能量、零加速度静止力矩、逆/前向往返、随机与参考库对照。PID 测正常、饱和、误差反号释放、dt 变化、限幅和噪声。

实机带载先查 payload/质心、力矩/单位、饱和和轨迹导数；再查摩擦回差、周期延迟、结构柔性和模型；同轨迹不同速度/负载对照。接触振荡从低刚度/速度和软环境开始，逐级扫描 K/D、频率、延迟和环境刚度，记录峰值力/能量，设硬停止门槛，不能直接在实机高刚度试错。

## 本篇完成标准

完成本篇后，应能解释并验证机械臂动力学各项；能用能量、M 正定和重力静止测试发现模型错误；能实现带限幅/anti-windup 的 PID；能比较重力前馈与计算力矩；能区分阻抗和导纳并安全设计接触实验；能实现 `ros2_control` 生命周期、实时数据交换、切换与 watchdog，并用 P99 周期和任务指标验收。

下一阶段将进入 MoveIt2 与抓取系统：URDF/SRDF、规划组、碰撞矩阵、IK 插件、Planning Pipeline、任务构造器、抓取候选、夹爪和感知到执行闭环。
