---
title: 第三册第一篇：状态估计与多传感器融合
description: 从 Bayes 滤波、KF/EKF/UKF 推导到 IMU、轮速、异步融合、可观性、一致性检验和 ROS2 工程实践。
---

# 第三册第一篇：状态估计与多传感器融合

移动机器人不会直接“知道”自己的位置、速度和姿态。编码器给出轮子转动，IMU 给出角速度和比力，相机或激光给出环境相对约束，GNSS 或地图匹配偶尔提供全局信息。状态估计的任务是把这些不同频率、不同坐标系、不同噪声和不同延迟的观测组合成带不确定性的机器人状态。

本篇的目标不是让轨迹看起来平滑，而是建立一个可检验的概率系统：状态定义明确；运动和测量模型有单位；时间顺序正确；协方差与真实误差匹配；异常观测能够拒绝；不可观方向不会被假装成高精度。后续 SLAM 前端、图优化和导航控制都依赖这些原则。

## 第 1 章：把估计问题写完整

### 1.1 状态不是传感器消息的拼接

状态 $x_t$ 是为了预测未来和解释观测而选择的最小变量集合。二维差速机器人可以使用

$$
x=[p_x,p_y,\theta,v,\omega]^T,
$$

三维惯性导航常使用位置 $p$、速度 $v$、姿态 $R$、陀螺偏置 $b_g$ 和加速度偏置 $b_a$。若把无法从传感器观测的变量全部加入状态，滤波器会病态；若遗漏重要偏置，误差会被错误归因给其他状态。

每个状态必须写明：物理含义、表达坐标系、单位、参考时间和参数化。例如速度是 world frame 还是 body frame，角度是弧度还是度，姿态是 ${}^wR_b$ 还是 ${}^bR_w`。代码名 `velocity` 和 `pose` 远远不够。

### 1.2 控制、过程与测量

离散状态空间模型写为

$$
x_k=f(x_{k-1},u_k,\Delta t)+w_k,
$$

$$
z_k=h(x_k)+v_k.
$$

$u_k$ 可以是控制命令，也可以是用于传播的 IMU 或轮速；$w_k$ 表示模型未覆盖的随机过程；$z_k$ 是传感器测量；$v_k$ 是测量噪声。控制命令不等于真实运动，尤其在打滑、碰撞和执行器饱和时。

### 1.3 Markov 假设与充分状态

Bayes 滤波假设给定当前状态后，下一状态与更早历史条件独立，当前测量也只依赖当前状态。若状态遗漏了传感器偏置、执行器延迟或接触模式，历史仍会影响未来，Markov 近似变差。扩充状态是一种方法，但也增加维度和可观性要求。

## 第 2 章：Bayes 滤波递推

### 2.1 预测

已知上一时刻后验 $p(x_{k-1}|z_{1:k-1})$，通过运动模型得到先验：

$$
p(x_k|z_{1:k-1})=
\int p(x_k|x_{k-1},u_k)
p(x_{k-1}|z_{1:k-1})dx_{k-1}.
$$

预测把上一状态的不确定性向前传播，并加入过程噪声。没有新测量时，合理系统的协方差通常增长，而不是保持不变。

### 2.2 更新

收到测量 $z_k$ 后：

$$
p(x_k|z_{1:k})=\eta
p(z_k|x_k)p(x_k|z_{1:k-1}),
$$

$\eta$ 是归一化常数。似然描述某个状态产生当前测量的合理程度。预测不是“旧值”，更新也不是“用传感器覆盖状态”，而是两个概率信息源的组合。

### 2.3 三类近似

卡尔曼滤波用高斯分布和线性模型得到闭式解；EKF 对非线性模型局部线性化；UKF 用确定性 sigma 点传播均值和协方差；粒子滤波用带权样本表达多峰分布。算法选择取决于非线性、多峰性、维度和实时预算，不存在统一最优。

## 第 3 章：线性卡尔曼滤波推导

线性系统为

$$
x_k=F_kx_{k-1}+B_ku_k+w_k,
\qquad w_k\sim\mathcal N(0,Q_k),
$$

$$
z_k=H_kx_k+v_k,
\qquad v_k\sim\mathcal N(0,R_k).
$$

### 3.1 预测均值和协方差

若上一后验为 $\mathcal N(\hat x_{k-1},P_{k-1})$，则

$$
\hat x_k^-=F_k\hat x_{k-1}+B_ku_k,
$$

$$
P_k^-=F_kP_{k-1}F_k^T+Q_k.
$$

上标 $-$ 表示加入本次测量前的先验。$Q$ 不只是“让轨迹更灵活的参数”，它应描述单位时间内未建模加速度、打滑等过程造成的状态协方差。

### 3.2 创新与创新协方差

预测测量为 $H_k\hat x_k^-$，创新

$$
y_k=z_k-H_k\hat x_k^-.
$$

创新协方差

$$
S_k=H_kP_k^-H_k^T+R_k.
$$

$S$ 同时包含状态预测和测量的不确定性。只看残差绝对值无法判断异常：误差 1 m 对高精度室内定位很大，对初次全球定位可能合理。

### 3.3 卡尔曼增益与更新

$$
K_k=P_k^-H_k^TS_k^{-1},
$$

$$
\hat x_k=\hat x_k^-+K_ky_k.
$$

不要在代码中显式求 $S^{-1}$，应解线性方程，数值更稳定。协方差推荐使用 Joseph 形式：

$$
P_k=(I-KH)P_k^-(I-KH)^T+KRK^T.
$$

理论上它与简式 $(I-KH)P^-$ 等价，有限精度下更容易保持对称半正定。

### 3.4 可复用实现

```python
import numpy as np

def symmetrize_covariance(P: np.ndarray) -> np.ndarray:
    return 0.5 * (P + P.T)

class LinearKalmanFilter:
    def __init__(self, state, covariance):
        self.x = np.asarray(state, np.float64).reshape(-1, 1)
        self.P = np.asarray(covariance, np.float64)
        if self.P.shape != (len(self.x), len(self.x)):
            raise ValueError("covariance shape mismatch")
        self._validate()

    def _validate(self):
        if not np.isfinite(self.x).all() or not np.isfinite(self.P).all():
            raise ValueError("state and covariance must be finite")
        if not np.allclose(self.P, self.P.T, atol=1e-9):
            raise ValueError("covariance must be symmetric")
        if np.linalg.eigvalsh(self.P).min() < -1e-10:
            raise ValueError("covariance must be positive semidefinite")

    def predict(self, F, Q, B=None, control=None):
        F = np.asarray(F, np.float64)
        Q = np.asarray(Q, np.float64)
        self.x = F @ self.x
        if B is not None and control is not None:
            self.x += np.asarray(B) @ np.asarray(control).reshape(-1, 1)
        self.P = symmetrize_covariance(F @ self.P @ F.T + Q)
        self._validate()

    def update(self, measurement, H, R):
        z = np.asarray(measurement, np.float64).reshape(-1, 1)
        H = np.asarray(H, np.float64)
        R = np.asarray(R, np.float64)
        innovation = z - H @ self.x
        S = symmetrize_covariance(H @ self.P @ H.T + R)
        gain = np.linalg.solve(S, H @ self.P).T
        self.x = self.x + gain @ innovation
        identity = np.eye(len(self.x))
        residual_map = identity - gain @ H
        self.P = symmetrize_covariance(
            residual_map @ self.P @ residual_map.T + gain @ R @ gain.T
        )
        self._validate()
        return innovation, S
```

测试包括维度错误、非有限输入、零噪声极限、极大 $R$、多次预测协方差增长、更新后协方差下降和长时间数值对称性。

## 第 4 章：匀速模型中的 Q 不是随手对角阵

设一维状态 $x=[p,v]^T$，采样间隔 $\Delta t$，未建模加速度为白噪声 $a\sim\mathcal N(0,\sigma_a^2)$。状态增量由加速度产生：

$$
G=\begin{bmatrix}\frac12\Delta t^2\\\Delta t\end{bmatrix},
$$

所以

$$
Q=G\sigma_a^2G^T=sigma_a^2
\begin{bmatrix}
\frac14\Delta t^4&\frac12\Delta t^3\\
\frac12\Delta t^3&\Delta t^2
\end{bmatrix}.
$$

位置与速度过程噪声相关，不能仅凭习惯写两个独立对角值。不同连续时间噪声定义会得到不同离散化形式，必须说明假设和单位。

采样间隔变化时必须每步重算 $F,Q$。用固定 `dt=0.01` 处理实际抖动的 80～120 Hz 数据，会形成系统时间误差。

## 第 5 章：扩展卡尔曼滤波

非线性系统：

$$
x_k=f(x_{k-1},u_k)+w_k,
\qquad z_k=h(x_k)+v_k.
$$

EKF 用雅可比

$$
F_k=\left.\frac{\partial f}{\partial x}\right|_{\hat x_{k-1}},
\qquad
H_k=\left.\frac{\partial h}{\partial x}\right|_{\hat x_k^-}
$$

传播协方差。均值使用原非线性函数，协方差使用一阶近似。

### 5.1 差速平面模型

状态 $[x,y,\theta]^T$，输入线速度 $v$ 和角速度 $\omega$：

$$
f(x,u)=
\begin{bmatrix}
x+v\cos\theta\Delta t\\
y+v\sin\theta\Delta t\\
\theta+\omega\Delta t
\end{bmatrix}.
$$

状态雅可比

$$
F=
\begin{bmatrix}
1&0&-v\sin\theta\Delta t\\
0&1&v\cos\theta\Delta t\\
0&0&1
\end{bmatrix}.
$$

输入噪声雅可比

$$
L=
\begin{bmatrix}
\cos\theta\Delta t&0\\
\sin\theta\Delta t&0\\
0&\Delta t
\end{bmatrix}.
$$

若轮速的 $[v,\omega]$ 协方差为 $M$，状态过程噪声可以写为 $Q=LML^T$。

### 5.2 角度创新必须归一化

预测航向 $179^\circ$，测量 $-179^\circ$，物理误差为 $2^\circ$，直接相减为 $-358^\circ$。使用

```python
def wrap_angle(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi
```

在状态传播后和角度创新中归一化。对三维旋转不能逐个欧拉角简单 wrap，应在 $SO(3)$ 切空间定义误差。

### 5.3 Jacobian 测试

手推雅可比必须与有限差分或自动微分比较：

```python
def numerical_jacobian(function, x, epsilon=1e-6):
    x = np.asarray(x, np.float64)
    baseline = np.asarray(function(x), np.float64)
    J = np.empty((baseline.size, x.size))
    for column in range(x.size):
        delta = np.zeros_like(x)
        delta[column] = epsilon
        J[:, column] = (
            np.asarray(function(x + delta))
            - np.asarray(function(x - delta))
        ) / (2.0 * epsilon)
    return J
```

在多个随机状态和角度边界附近测试。有限差分步长过大有截断误差，过小有浮点消减，应扫描多个数量级。

## 第 6 章：UKF 的适用位置

UKF 不显式求雅可比，而是从均值和协方差构造 sigma 点，经非线性函数传播，再加权恢复均值与协方差。它对某些强非线性变换比一阶 EKF 更准确，也便于复杂模型实现。

但 UKF 不是自动稳定器：状态维度增加时 sigma 点数增加；旋转和角度均值需要流形处理；噪声、时间戳和坐标错误仍会失败；参数 $\alpha,\beta,\kappa$ 会影响权重。对于模型近线性、雅可比清晰的系统，EKF 通常更直接高效。

比较 EKF 与 UKF 要使用同一数据、相同噪声假设和一致初始化，报告真实误差、NIS/NEES、延迟和算力，而不是只看轨迹平滑。

## 第 7 章：IMU 测量模型

### 7.1 陀螺仪

$$
\omega_m=\omega+b_g+n_g,
$$

$b_g$ 是缓慢变化偏置，$n_g$ 是白噪声。偏置积分后直接形成姿态漂移。可将偏置建模为随机游走：

$$
\dot b_g=n_{wg}.
$$

### 7.2 加速度计测的是比力

加速度计模型常写为

$$
a_m=R_{wb}^T(a_w-g_w)+b_a+n_a.
$$

静止放在桌上时，传感器不会输出零，而会感受到与重力相反的支撑比力，具体符号取决于坐标和驱动约定。直接把读数二次积分而不估计姿态、重力和偏置，会快速漂移。

### 7.3 安装外参

IMU frame 与 `base_link` 不一致时，角速度和加速度必须旋转。平移杠杆臂在机器人旋转时还会产生切向和向心加速度：

$$
a_{imu}=a_{base}+\dot\omega\times r+\omega\times(\omega\times r).
$$

低速小机器人可能忽略，高动态平台和远离旋转中心安装时不可忽略。

### 7.4 静止检测

静止窗口内，陀螺均值可估计初始偏置，加速度方向可估计 roll/pitch，但不能由重力确定 yaw。静止检测可检查角速度范数和加速度范数/方差，但车辆匀速直行也可能低动态，不能随意施加零速度更新。

## 第 8 章：IMU 噪声标定

### 8.1 静态统计

固定 IMU 数小时，记录温度和原始数据。短窗口标准差估计白噪声，长时间均值漂移显示偏置和温度相关性。数据手册噪声密度与每采样标准差单位不同，转换通常与采样频率平方根相关，必须依据厂商定义。

### 8.2 Allan 方差

Allan deviation 随聚合时间变化，可识别白噪声、偏置不稳定和随机游走等区间。其斜率和系数解释依赖单位和定义。学习者应使用成熟工具计算，同时用合成噪声验证工具配置，避免从漂亮曲线读出错误参数。

### 8.3 温度

冷启动到热稳态偏置可能明显变化。实验应记录开机时间和温度，比较冷态/热态。可建立温度补偿模型，但必须保留残余不确定性；补偿后的协方差不能虚假缩小到零。

## 第 9 章：差速轮速里程计

### 9.1 从编码器到轮速

每轮一周期 tick 变化 $\Delta n_l,\Delta n_r$，每转 tick 数 $N$，半径 $r_l,r_r$：

$$
\Delta s_l=2\pi r_l\frac{\Delta n_l}{N},
\qquad
\Delta s_r=2\pi r_r\frac{\Delta n_r}{N}.
$$

机器人中心弧长与航向变化：

$$
\Delta s=\frac{\Delta s_r+\Delta s_l}{2},
\qquad
\Delta\theta=\frac{\Delta s_r-\Delta s_l}{b},
$$

$b$ 为有效轮距。轮胎变形和接触点使有效参数可能不同于卡尺尺寸。

### 9.2 精确圆弧积分

$|\Delta\theta|$ 足够大时：

$$
x_{k+1}=x_k+\frac{\Delta s}{\Delta\theta}
[\sin(\theta+\Delta\theta)-\sin\theta],
$$

$$
y_{k+1}=y_k-\frac{\Delta s}{\Delta\theta}
[\cos(\theta+\Delta\theta)-\cos\theta].
$$

小角度时使用中点近似，避免除以接近零：

$$
x_{k+1}\approx x_k+\Delta s\cos(\theta+\Delta\theta/2),
$$

$$
y_{k+1}\approx y_k+\Delta s\sin(\theta+\Delta\theta/2).
$$

```python
from dataclasses import dataclass

@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float

def integrate_differential_drive(pose, left_distance, right_distance,
                                 wheel_base, epsilon=1e-8):
    if wheel_base <= 0 or not np.isfinite(
        [pose.x, pose.y, pose.yaw, left_distance, right_distance, wheel_base]
    ).all():
        raise ValueError("finite inputs and positive wheel base required")
    distance = 0.5 * (right_distance + left_distance)
    delta_yaw = (right_distance - left_distance) / wheel_base
    if abs(delta_yaw) < epsilon:
        heading = pose.yaw + 0.5 * delta_yaw
        dx = distance * np.cos(heading)
        dy = distance * np.sin(heading)
    else:
        radius = distance / delta_yaw
        dx = radius * (np.sin(pose.yaw + delta_yaw) - np.sin(pose.yaw))
        dy = -radius * (np.cos(pose.yaw + delta_yaw) - np.cos(pose.yaw))
    return Pose2D(pose.x + dx, pose.y + dy, wrap_angle(pose.yaw + delta_yaw))
```

### 9.3 参数标定

直行已知距离主要约束平均轮径尺度；原地旋转多圈主要约束有效轮距和左右差异。按实验逐个标定，不要一次同时优化所有参数而缺少可辨识数据。正向、反向、左右旋转都要测试，以发现回差和非对称。

### 9.4 打滑不是高斯小噪声

地毯边缘、湿滑地面和碰撞会产生突发大误差。只增大固定 $Q$ 会让正常路段也变差。可依据 IMU/轮速不一致、驱动电流或地面分类动态调过程噪声，并对严重滑移进入故障模式。

## 第 10 章：多传感器异步融合

### 10.1 以测量时间为准

IMU 200 Hz、轮速 50 Hz、视觉 20 Hz、GNSS 5 Hz，不应先粗暴重采样到同一频率。按时间戳排序事件：从当前滤波时刻传播到测量时刻，再执行对应更新。

若迟到测量的时间早于当前状态，有三种策略：丢弃；保存历史状态并回滚重放；使用固定延迟平滑器。直接把旧测量当成当前测量会引入速度相关偏差。

### 10.2 事件队列骨架

```python
from dataclasses import dataclass
from heapq import heappush, heappop

@dataclass(order=True)
class SensorEvent:
    timestamp_ns: int
    sequence: int
    sensor: str
    measurement: object

class FusionEventQueue:
    def __init__(self):
        self._heap = []
        self._sequence = 0

    def push(self, timestamp_ns, sensor, measurement):
        if timestamp_ns <= 0:
            raise ValueError("timestamp must be positive")
        event = SensorEvent(timestamp_ns, self._sequence, sensor, measurement)
        self._sequence += 1
        heappush(self._heap, event)

    def pop(self):
        if not self._heap:
            return None
        return heappop(self._heap)
```

相同时间戳使用 sequence 保持确定顺序。正式系统还要定义队列等待窗口、最大迟到、重复消息和时钟回退处理。

### 10.3 相关测量不能当作独立

视觉惯性里程计已经使用 IMU，若再把同一 IMU 和 VIO 输出作为独立测量放入另一个 EKF，会重复计入信息，协方差过小。轮速参与激光里程计初值后，输出也存在相关性。理想做法是融合原始独立约束或显式建模交叉协方差；工程中无法建模时至少避免明显重复，并进行一致性检验。

## 第 11 章：创新门限与异常测量

### 11.1 NIS

创新归一化平方

$$
\operatorname{NIS}=y^TS^{-1}y
$$

若模型和噪声正确，测量维数为 $m$ 时，NIS 近似服从 $\chi^2_m$。可用卡方分位数做门限。不能给所有传感器统一“残差小于 1”，因为单位和维度不同。

```python
def normalized_innovation_squared(innovation, covariance):
    y = np.asarray(innovation, np.float64).reshape(-1, 1)
    S = np.asarray(covariance, np.float64)
    if S.shape != (len(y), len(y)):
        raise ValueError("innovation covariance shape mismatch")
    return float(y.T @ np.linalg.solve(S, y))
```

单次超门限可拒绝测量，连续偏高更可能说明模型、外参、时延或噪声低估。门限过严会在快速运动中拒绝所有真正有用的修正，导致滤波器仅靠预测漂移。

### 11.2 鲁棒更新

除了硬拒绝，可根据残差使用 Huber 等权重增大有效测量协方差。但鲁棒核不能修复坐标轴反转、单位错误和系统性延迟。先保证接口正确，再处理统计外点。

### 11.3 故障隔离

多个传感器冲突时，仅凭创新无法确定谁错。结合传感器自检、物理范围、时间连续性和第三方参照。维护每个传感器的健康状态和连续异常计数，避免单帧噪声频繁开关融合。

## 第 12 章：滤波一致性

### 12.1 NEES

有真值 $x^*$ 时，估计误差归一化平方

$$
\operatorname{NEES}=(\hat x-x^*)^TP^{-1}(\hat x-x^*).
$$

若估计维数为 $n$ 且假设正确，NEES 近似服从 $\chi^2_n$。长期高于区间说明过度自信、模型或噪声错误；长期过低说明协方差过于保守。

### 12.2 轨迹平滑的反例

把 $R$ 设得极大，滤波器几乎忽略测量，输出会非常平滑，但转弯滞后且长期漂移；把错误 GNSS 协方差设为极小，轨迹会平滑地跟随错误位置。视觉观感不能代替真值和一致性统计。

### 12.3 Monte Carlo 验证

用相同模型随机生成多条真值和噪声序列，运行滤波并统计 NEES/NIS 落入置信区间比例。单条轨迹可能偶然好看，多次模拟能验证协方差传播是否与生成假设一致。

## 第 13 章：可观性

线性系统的可观矩阵

$$
\mathcal O=
\begin{bmatrix}
H\\HF\\HF^2\\\vdots\\HF^{n-1}
\end{bmatrix}.
$$

若秩为状态维数，系统在理论上可观。非线性系统通常在轨迹附近分析局部可观性，机器人运动本身会影响可观程度。

### 13.1 常见不可观方向

- 纯 IMU 积分没有全局位置和 yaw 约束。
- 单目视觉缺少绝对尺度。
- 只有单平面点到平面配准时，沿平面平移和绕法线旋转弱约束。
- 差速轮速无法可靠观察横向滑移。
- 静止加速度只能给重力方向，不能给 yaw。

### 13.2 激励运动

标定 IMU-相机外参、时间偏移和偏置需要足够旋转与加速度激励。一直匀速直线运动可能让若干参数耦合。采集标定数据应刻意覆盖各轴运动，同时遵守设备安全范围。

### 13.3 不可观不等于数值不输出

滤波器仍可能为不可观变量输出一个数字，甚至协方差因错误模型逐渐缩小。必须通过理论分析、Hessian/可观矩阵和受控实验识别，而不是相信消息字段存在就代表可估计。

## 第 14 章：二维轮速、IMU 与绝对位置 EKF

### 14.1 状态与模型

教学系统定义

$$
x=[p_x,p_y,\theta,v,b_g]^T.
$$

轮速给 $v_m$，陀螺给 $\omega_m=\omega+b_g+n_g$，传播：

$$
p_x'=p_x+v_m\cos\theta\Delta t,
$$

$$
p_y'=p_y+v_m\sin\theta\Delta t,
$$

$$
\theta'=\theta+(\omega_m-b_g)\Delta t,
$$

$$v'=v_m,\qquad b_g'=b_g.$$

绝对位置传感器测量 $z=[p_x,p_y]^T$。若长期没有绝对航向且运动缺乏转弯，yaw 与陀螺偏置可能弱可观。

### 14.2 传播 Jacobian

$$
F=
\begin{bmatrix}
1&0&-v_m\sin\theta\Delta t&0&0\\
0&1&v_m\cos\theta\Delta t&0&0\\
0&0&1&0&-\Delta t\\
0&0&0&0&0\\
0&0&0&0&1
\end{bmatrix}.
$$

这里把 $v'$ 直接设为当前轮速，所以对应行对旧状态导数为零。若使用一阶速度动态模型，F 会不同。模型、代码和 Jacobian 必须来自同一假设。

### 14.3 更新与门禁

绝对位置更新前计算 NIS；超过卡方门限时拒绝并累积健康计数。若连续拒绝，不应无限沿用原门限，可报告传感器故障、重新初始化候选或进入降级模式。自动重置状态是重大行为，必须由上层状态机决定。

## 第 15 章：ROS2 坐标系与融合架构

### 15.1 map、odom、base_link

`odom -> base_link` 应连续、短期准确但允许长期漂移，供控制器使用；`map -> odom` 由地图定位或 SLAM 提供全局修正，允许缓慢或离散变化；组合得到 `map -> base_link`。

若全局 GNSS 跳变直接进入 odom，控制器会看到位置和速度不连续。若局部里程计错误发布 map frame，会破坏坐标树语义。

### 15.2 传感器 frame

IMU、轮速、相机、激光都必须有到 `base_link` 的外参。只修改 `frame_id` 不会旋转数值。IMU 的 orientation、angular_velocity、linear_acceleration 可能有不同协方差和有效性，不能因为一个字段存在就默认可信。

### 15.3 robot_localization 配置方法

先制作状态-传感器矩阵：每行一个传感器，每列一个状态，标记直接测量还是由同一来源推导。再确定差分/相对模式、frame、频率、超时和协方差。避免同时融合同一轮速里程计中的位姿、速度和由它导出的航向，造成重复信息。

调试按以下顺序：单独验证消息单位与轴；静止偏置；TF；单传感器输出；逐个加入新传感器并查看创新/轨迹；最后才调整协方差。一次启用全部来源后盲调参数很难定位问题。

## 第 16 章：时间同步与延迟标定

### 16.1 三种时间

采样时间是物理测量对应的时刻；接收时间是消息到达主机的时刻；处理完成时间是输出生成时刻。状态估计必须基于采样时间，性能监控使用接收和完成时间。

设备时钟、ROS clock 和系统时钟可能不同。跨机器系统需要 PTP、硬件触发或经过验证的同步机制。仅让主机 NTP 大致同步不一定满足高速融合。

### 16.2 延迟的空间后果

速度 $v$ 下时间偏移 $\delta t$ 近似产生位置误差 $v\delta t$，角速度 $\omega$ 下产生角误差 $\omega\delta t$。把机器人以多个正负速度运动，误差随速度符号翻转，是时间偏移的重要证据；固定外参偏差通常不会以同样方式变化。

### 16.3 迟到测量重放

保存有限历史状态和输入。迟到测量落在窗口内时，恢复测量前状态、更新，再按原输入顺序传播到当前。所有随机过程和事件顺序必须可重复。窗口外测量丢弃并计数。图优化和平滑器天然更适合处理历史约束，但计算结构不同。

## 第 17 章：故障注入与系统化诊断

### 故障一：IMU yaw 轴反号

直行时可能不明显，旋转时轮速航向与 IMU 相反，创新快速增长。检查 REP-103 轴、右手规则、安装旋转和驱动符号。不要靠把协方差调大掩盖。

### 故障二：轮速 rpm 当 rad/s

速度超过物理上限，里程尺度错误。入口做单位转换和范围检查；用一圈轮子 tick 与实际周长做最小测试。

### 故障三：协方差全零

滤波器把测量当近乎完美，其他传感器难以修正，数值可能奇异。未知协方差应填经过标定的值或明确禁用字段，不能用零表达“没有信息”。ROS 消息中某些特殊约定要按消息定义处理。

### 故障四：视觉里程计延迟 200 ms

静态正常，运动时更新把状态拉回过去，出现锯齿和速度相关偏差。使用采样时间、延迟补偿或历史重放，记录消息年龄。

### 故障五：滤波器逐渐过度自信

真实误差增长而 P 缩小，NEES 持续过高。检查过程噪声、重复融合相关信息、线性化、一致性和未建模偏置。单纯把所有 Q 放大可以缓解数值，却不一定修复根因。

## 第 18 章：综合实验一——KF 一致性

模拟二维匀速状态，随机加速度为已知白噪声，位置测量为已知高斯噪声。运行至少 500 次 Monte Carlo，每次 200 步。比较三组滤波器：正确 Q/R、Q 缩小 100 倍、R 放大 100 倍。

报告位置/速度 RMSE、平均协方差、NIS 和 NEES 置信区间覆盖率。预期 Q 过小时过度自信且机动响应差，R 过大时过分依赖模型。必须展示某条“看起来平滑但 NEES 失败”的反例。

## 第 19 章：综合实验二——轮速与 IMU 融合

采集包含静止、直线、左右圆弧、原地旋转和不同地面的 rosbag。先单独标定轮径/轮距和 IMU 静止噪声，再逐个融合。使用外部定位或闭环终点测量作为真值。

依次注入轮速尺度 +5%、陀螺偏置、200 ms 延迟、轴反转和打滑段。对每种故障保存轨迹、创新、NIS、协方差、拒绝计数和传感器健康状态。写出自动检测规则与已知漏检。

### 验收表

| 场景 | 位置 P95 | 航向 P95 | NIS 覆盖 | 拒绝率 | 结论 |
| --- | ---: | ---: | ---: | ---: | --- |
| 正常地面 |  |  |  |  |  |
| 地毯边缘 |  |  |  |  |  |
| 快速旋转 |  |  |  |  |  |
| 视觉延迟 |  |  |  |  |  |
| 轮速滑移 |  |  |  |  |  |

## 第 20 章：阶段考试

建议限时 180 分钟，满分 100 分。

### 一、理论题，共 35 分

1. 写出 Bayes 滤波预测和更新，并解释各条件独立假设。（5 分）
2. 推导线性 KF 的创新、增益和 Joseph 协方差更新。（5 分）
3. 从白加速度噪声推导 $[p,v]$ 状态的离散 Q。（5 分）
4. 推导差速平面运动模型的状态 Jacobian。（5 分）
5. 加速度计静止时为什么不输出零？（5 分）
6. 比较 EKF、UKF 和粒子滤波的适用条件。（5 分）
7. 解释可观性和“输出一个数”的区别。（5 分）

### 二、代码题，共 30 分

1. 为 `LinearKalmanFilter` 设计十个测试，包括数值和统计测试。（10 分）
2. 实现差速圆弧积分并验证直行极限连续。（10 分）
3. 设计异步事件队列的迟到、重复和时钟回退策略。（10 分）

### 三、诊断题，共 35 分

1. EKF 输出平滑但真值误差越来越大，给出分层排查树。（15 分）
2. 静态融合准确，快速转弯时轨迹锯齿，设计区分时延、外参和轮距错误的实验。（10 分）
3. VIO 已融合 IMU，又把 VIO 和原 IMU 输入 EKF，有什么风险？如何验证？（10 分）

## 第 21 章：参考答案

### 一、理论题

1. 预测对上一后验与转移概率积分，更新用测量似然乘先验并归一化。依赖状态满足 Markov 性、测量给定当前状态后与历史条件独立；遗漏偏置和延迟会破坏近似。

2. 创新 $y=z-H\hat x^-$，协方差 $S=HP^-H^T+R$，增益 $K=P^-H^TS^{-1}$，状态更新 $\hat x=\hat x^-+Ky$。Joseph 形式为 $(I-KH)P^-(I-KH)^T+KRK^T$，数值上更好保持半正定。

3. 加速度对位置和速度的离散影响为 $G=[\Delta t^2/2,\Delta t]^T$，故 $Q=G\sigma_a^2G^T$，包含 $\Delta t^4/4,\Delta t^3/2,\Delta t^2$ 项和非零交叉协方差。

4. 对 $x'=x+v\cos\theta\Delta t$、$y'=y+v\sin\theta\Delta t$、$\theta'=\theta+\omega\Delta t$ 求状态偏导，关键项为 $\partial x'/\partial\theta=-v\sin\theta\Delta t$、$\partial y'/\partial\theta=v\cos\theta\Delta t$。

5. 加速度计测比力，静止时地面对传感器的支撑抵消重力产生非零输出，符号取决于 frame 和驱动定义。模型中还含偏置和噪声。

6. EKF 适合近似高斯、非线性温和且 Jacobian 可得；UKF 用 sigma 点处理较强非线性和复杂函数但成本更高；粒子滤波能表达多峰和非高斯，维度高时样本成本迅速增加。

7. 可观性说明有限观测能否区分状态。程序可以为不可观变量保留初始值或由数值耦合更新，并输出一个看似精确的数，但这不创造信息；需从模型、轨迹和信息矩阵分析。

### 二、代码题

KF 测试应包括：维度错误；非对称/非半正定 P；无测量时预测均值；Q=0 的确定传播；Q>0 协方差增长；R 很小时更新靠近测量；R 很大时接近先验；Joseph 更新保持对称半正定；与解析一维结果比较；长序列无 NaN；Monte Carlo NIS/NEES 覆盖。仅普通轨迹截图不能替代断言。

差速积分测试包括相等轮距直行、等值反向原地转、已知圆弧、负向运动、极小 $\Delta\theta$ 与零转角结果连续、无效轮距拒绝。可对一系列趋近零的角度比较圆弧公式和中点极限。

事件队列按采样时间和序号排序；为每传感器维护最后序列/时间检测重复；允许配置乱序等待窗口；窗口内迟到可历史重放，窗口外丢弃并计数；时钟明显回退触发新 epoch 或停止融合，不能与旧时间线混合。策略与诊断都要测试。

### 三、诊断题

平滑但漂移先查真值和坐标；再查单位、轴、时间戳、外参；单独运行每个传感器；查看创新、NIS/NEES、P 特征值；核对 Q/R 单位和离散化；检查重复融合相关信息、偏置状态和 Jacobian；最后才调参。若 P 缩小而误差增大，是过度自信强信号。

快速转弯锯齿实验：改变速度和角速度，若误差随角速度及消息年龄增长并在调整时间偏移后改善，支持时延；外参旋转错误在静态不同姿态或传感器轴对比也产生系统偏差；轮距错误使左右旋转累计角度按方向呈规律尺度误差，且单独轮速积分已可复现。使用硬件同步或离线时间偏移扫描进一步区分。

VIO 输出与原 IMU 高度相关，把二者当独立测量会重复计入 IMU 信息，使协方差过小、NIS/NEES 不一致，并可能在 VIO 故障时两个输入同时受影响。对比只融合 VIO、VIO+IMU 的真值误差与 NEES，检查后者是否没有真实精度提升却显著缩小 P。优先融合独立原始约束或使用系统提供的完整协方差/紧耦合接口。

## 本篇完成标准

完成本篇时，应能从状态和测量语义推导滤波模型，而不是只填写 YAML；能实现数值稳定的 KF 并用 Monte Carlo 验证一致性；能解释 IMU 比力、偏置、轮速参数和打滑；能按采样时间处理异步数据；能用 NIS/NEES 判断“平滑但错误”；能识别不可观与相关信息；能在 ROS2 中维护正确的 `map -> odom -> base_link` 语义。

下一篇将在这些状态估计基础上进入 SLAM 前端：二维激光扫描匹配、视觉/激光里程计、关键帧、局部地图、运动畸变、退化检测和前端质量评价。
