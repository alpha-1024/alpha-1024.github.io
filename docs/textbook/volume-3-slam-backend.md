---
title: 第三册第三篇：SLAM 后端、回环与图优化
description: 系统讲解位姿图、因子图、SE(2)/SE(3) 残差、稀疏优化、鲁棒核、回环验证与全局地图一致性。
---

# 第三册第三篇：SLAM 后端、回环与图优化

前端把连续传感器转换成局部相对运动约束，但每条约束都有噪声，误差会沿轨迹累积。后端把历史位姿、地图点、IMU 偏置和回环等变量放入同一个优化问题，寻找最符合全部约束的全局状态。回环让机器人知道“这里以前来过”，也让一条错误约束有能力扭曲整张地图。

本篇围绕一个核心问题展开：怎样把约束的物理语义、坐标方向和不确定性正确转换成可求解的图，并在错误回环、规范自由度、稀疏数值问题和实时增量更新中保持系统可诊断。

## 第 1 章：从滤波到平滑

### 1.1 滤波只维护当前状态

滤波计算 $p(x_k|z_{1:k})$，历史状态通常被边缘化。它实时、内存固定，但旧状态无法在新回环到来时任意调整。平滑估计一段或全部轨迹：

$$
p(x_{0:k}|z_{1:k}).
$$

回环能够同时修正过去和现在，因此 SLAM 后端常使用非线性最小二乘、因子图或位姿图。

### 1.2 位姿图和因子图

位姿图的节点通常是关键帧位姿，边是相邻里程计、回环或绝对位姿约束。因子图更一般：变量可以包括位姿、速度、IMU 偏置、路标和外参；每个因子连接它依赖的变量。

图不是数据结构装饰，而是条件独立关系。一个因子连接哪些变量，应由测量模型决定。把同一测量拆成多个独立因子可能重复计入信息。

## 第 2 章：最大后验与非线性最小二乘

给定变量集合 $X$ 和条件独立测量 $z_k$：

$$
p(X|Z)\propto p(X)\prod_kp(z_k|X_k).
$$

若测量噪声为高斯，残差 $e_k(X)$ 协方差为 $\Sigma_k$，最大化后验等价于最小化负对数：

$$
X^*=\arg\min_X\sum_k e_k(X)^T\Omega_ke_k(X),
\qquad \Omega_k=\Sigma_k^{-1}.
$$

这里忽略了与 $X$ 无关的常数。若噪声不是高斯或存在外点，需要鲁棒损失或显式混合模型。

## 第 3 章：SE(2) 位姿图

### 3.1 二维位姿

位姿 $x_i=(p_{xi},p_{yi},\theta_i)$ 对应

$$
T_i=
\begin{bmatrix}
\cos\theta_i&-\sin\theta_i&p_{xi}\\
\sin\theta_i&\cos\theta_i&p_{yi}\\
0&0&1
\end{bmatrix}.
$$

边测量 $Z_{ij}$ 表示从节点 $i$ 到节点 $j$ 的相对变换。根据当前估计，预测相对变换为 $T_i^{-1}T_j$。群上的误差可写为

$$
e_{ij}=\operatorname{Log}\left(Z_{ij}^{-1}T_i^{-1}T_j\right).
$$

误差位于 $se(2)$ 切空间，是三维向量。若边方向定义相反，公式也要相应改变。

### 3.2 简化坐标残差

教学实现可在二维坐标中写预测：

$$
\hat p_{ij}=R_i^T(p_j-p_i),
\qquad
\hat\theta_{ij}=\operatorname{wrap}(\theta_j-\theta_i),
$$

残差为预测减测量，角度项必须 wrap。小角度附近它与 Lie 群残差接近，但大误差和三维扩展应使用正规群运算。

```python
import numpy as np

def wrap_angle(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi

def relative_pose_2d(a, b):
    ax, ay, atheta = a
    bx, by, btheta = b
    c, s = np.cos(atheta), np.sin(atheta)
    delta = np.array([bx - ax, by - ay])
    local = np.array([c * delta[0] + s * delta[1],
                      -s * delta[0] + c * delta[1]])
    return np.array([local[0], local[1], wrap_angle(btheta - atheta)])

def edge_residual_2d(pose_i, pose_j, measurement):
    residual = relative_pose_2d(pose_i, pose_j) - measurement
    residual[2] = wrap_angle(residual[2])
    return residual
```

### 3.3 残差测试

单位位姿与单位边残差应为零；两个姿态同时左乘相同全局变换后残差不变；交换节点并使用逆测量后结果一致；航向跨越 $\pm\pi$ 时残差保持小角度。全局变换不改变相对残差，正是规范自由度的来源。

## 第 4 章：线性化与 Gauss-Newton

在当前估计 $X$ 附近用增量 $\delta$：

$$
e_k(X\boxplus\delta)\approx e_k(X)+J_k\delta.
$$

目标二次近似形成正规方程：

$$
H\delta=-b,
$$

$$
H=\sum_kJ_k^T\Omega_kJ_k,
\qquad
b=\sum_kJ_k^T\Omega_ke_k.
$$

求解增量并在流形上更新位姿，重复到代价或增量足够小。不能把 SE(3) 位姿矩阵逐元素相加；旋转更新应通过指数映射或受约束参数化。

### 4.1 Levenberg-Marquardt

Gauss-Newton 初值差时可能代价上升。LM 在 Hessian 上加入阻尼：

$$
(H+\lambda D)\delta=-b.
$$

步长成功则减小 $\lambda$，失败则增大。$D$ 可取单位阵或 Hessian 对角。LM 提高收敛稳定性，但无法把错误回环变成正确约束。

### 4.2 数值 Jacobian 是单元测试工具

解析 Jacobian 性能好，但符号、左右扰动和坐标方向容易出错。对随机姿态，用中心差分计算每个变量的数值 Jacobian，与解析结果比较。测试覆盖角度边界和不同尺度；正式优化仍使用解析或自动微分。

## 第 5 章：规范自由度

只含相对约束的位姿图整体平移和旋转不会改变任何残差，因此解不唯一，Hessian 奇异。二维有三个规范自由度，单目三维还可能有整体尺度。

最常用做法是固定第一个位姿，或给它添加高置信先验。这个先验只是选择世界坐标，不应被解释成传感器精确测得原点。如果图中有真实 GNSS/地图先验，应使用其实际协方差。

直接给 Hessian 对角加一个很小数可以让求解器不报奇异，但会隐藏规范问题并影响协方差。应显式固定规范，再用阻尼处理非线性。

## 第 6 章：稀疏结构与求解

每条相邻边只连接两个位姿，Jacobian 大部分为零，Hessian 呈块稀疏。若使用稠密矩阵，变量增长后内存和计算迅速爆炸。成熟库使用稀疏 Cholesky、QR、PCG 或 Bayes tree 等结构。

位姿与路标共同优化时，Schur 补可先消去大量路标变量，得到只含相机的约化系统。消元顺序会影响填充和性能。生产 SLAM 应使用 Ceres、GTSAM、g2o 等库，教学实现用于验证残差和图结构。

## 第 7 章：一个小型 SE(2) 优化器骨架

```python
from dataclasses import dataclass

@dataclass(frozen=True)
class PoseEdge2D:
    source: int
    target: int
    measurement: np.ndarray
    information: np.ndarray
    kind: str = "odometry"

def numerical_edge_jacobians(pose_i, pose_j, measurement, epsilon=1e-6):
    base_i = np.asarray(pose_i, np.float64)
    base_j = np.asarray(pose_j, np.float64)
    A = np.empty((3, 3))
    B = np.empty((3, 3))
    for axis in range(3):
        step = np.zeros(3)
        step[axis] = epsilon
        plus = edge_residual_2d(base_i + step, base_j, measurement)
        minus = edge_residual_2d(base_i - step, base_j, measurement)
        difference = plus - minus
        difference[2] = wrap_angle(difference[2])
        A[:, axis] = difference / (2 * epsilon)

        plus = edge_residual_2d(base_i, base_j + step, measurement)
        minus = edge_residual_2d(base_i, base_j - step, measurement)
        difference = plus - minus
        difference[2] = wrap_angle(difference[2])
        B[:, axis] = difference / (2 * epsilon)
    return A, B

def assemble_system(poses, edges, robust_weight=lambda error: 1.0):
    count = len(poses)
    H = np.zeros((3 * count, 3 * count))
    b = np.zeros(3 * count)
    total_cost = 0.0
    for edge in edges:
        i, j = edge.source, edge.target
        error = edge_residual_2d(poses[i], poses[j], edge.measurement)
        A, B = numerical_edge_jacobians(
            poses[i], poses[j], edge.measurement
        )
        squared = float(error.T @ edge.information @ error)
        weight = robust_weight(squared) if edge.kind == "loop" else 1.0
        Omega = weight * edge.information
        ii = slice(3 * i, 3 * i + 3)
        jj = slice(3 * j, 3 * j + 3)
        H[ii, ii] += A.T @ Omega @ A
        H[ii, jj] += A.T @ Omega @ B
        H[jj, ii] += B.T @ Omega @ A
        H[jj, jj] += B.T @ Omega @ B
        b[ii] += A.T @ Omega @ error
        b[jj] += B.T @ Omega @ error
        total_cost += squared * weight
    return H, b, total_cost
```

固定第一个位姿可以删除其三行三列，或对其块加入强先验。教学代码使用数值 Jacobian，规模大时必须换解析/自动微分和稀疏矩阵。

### 7.1 终止条件

同时检查最大增量、相对代价下降、梯度和迭代上限。求解器返回“成功”只说明满足数值终止条件，不证明图约束正确。每轮保存总成本、里程计成本、回环成本、阻尼和最大更新。

## 第 8 章：信息矩阵的构造

### 8.1 单位问题

SE(2) 残差同时包含米和弧度。信息矩阵决定它们相对权重。简单单位阵隐含“1 m 与 1 rad 同等标准差”，通常没有物理依据。

前端可从匹配 Jacobian 和测量噪声近似协方差，但模型误差、数据关联和退化常被低估。应通过带真值重复实验校准，按场景或质量动态调整，并设置合理上下界。

### 8.2 相关性

相邻边共享传感器帧和局部地图，严格上并非独立；回环匹配也可能复用特征。多数位姿图忽略部分相关性，协方差会偏乐观。至少避免把同一前端输出拆成多个“独立”边重复添加。

### 8.3 信息矩阵验证

必须对称正定或半正定。检查特征值、条件数和物理尺度。若前端退化方向明显，应降低对应信息，而不是只降低整体标量。

## 第 9 章：鲁棒核

普通二次损失让大残差平方增长，错误回环可支配优化。Huber 损失小残差二次、大残差线性；Cauchy 对大残差抑制更强。

以平方 Mahalanobis 残差 $s=e^T\Omega e$ 为输入，Huber 权重可近似：

```python
def huber_weight(squared_error, delta=3.0):
    if squared_error < 0 or delta <= 0:
        raise ValueError("invalid robust loss input")
    norm = np.sqrt(squared_error)
    return 1.0 if norm <= delta else delta / max(norm, 1e-12)
```

鲁棒核阈值作用于归一化残差。如果信息矩阵错误，阈值也失去统计意义。鲁棒核只能降低少量外点影响，无法在多数回环都错误时恢复真相。

## 第 10 章：可切换约束和动态协方差缩放

Switchable Constraints 为可疑回环引入开关变量 $s_{ij}$，优化可把错误边权重降到零，同时用先验鼓励正确边保持开启。Dynamic Covariance Scaling 根据残差动态缩放信息。

这类方法为后端提供第二道防线，但不能替代前端几何验证。若错误回环初始残差小或形成互相一致的错误集，仍可能被接受。

## 第 11 章：回环候选检索

### 11.1 外观检索

视觉可使用词袋、全局描述子或学习检索；激光可使用 Scan Context 等全局几何描述。检索输出只是候选，不是约束。相似走廊、重复货架和季节/光照变化会产生假阳性与假阴性。

### 11.2 排除近邻

时间上很近的关键帧天然相似，不构成有价值回环。设置最小关键帧间隔或拓扑距离。机器人停留原地时更要避免连续重复帧淹没检索库。

### 11.3 候选多样性

保留多个候选并做非极大抑制，避免 Top-K 全部来自同一局部区域。候选分数需在验证集校准，不能跨环境使用固定阈值而不评估。

## 第 12 章：回环几何验证

视觉回环：描述子匹配 → 比率与双向过滤 → 本质/PnP/Sim(3) RANSAC → 内点覆盖、视差、重投影、正深度 → 局部优化。激光回环：全局描述候选 → yaw 初值 → scan-to-submap → 重叠、残差、Hessian 和初值扰动检查。

### 12.1 多帧一致性

单帧候选通过后，检查相邻关键帧是否也支持相近回环位置。真实回环通常形成连续一致的候选簇；单帧重复纹理更容易孤立。但长期重复结构也可能形成错误簇，所以仍需几何和拓扑检查。

### 12.2 回环闭合误差

沿现有里程计从 $i$ 到 $j$ 的组合与候选回环比较，差异大并不自动代表回环错误，因为累计漂移正是回环要修正；但差异应与轨迹长度、估计漂移和不确定性相容。一步要求地图折叠数十米的高信息回环必须更严格验证。

### 12.3 延迟激活

可把新回环作为候选边，先低权重加入或等待多帧确认，再提高权重。保存激活历史和证据，便于错误回环发生后定位。

## 第 13 章：SE(3) 位姿图

三维误差通常写为

$$
e_{ij}=\operatorname{Log}\left(Z_{ij}^{-1}T_i^{-1}T_j\right)\in\mathbb R^6.
$$

六维向量包含旋转和平移扰动。左扰动与右扰动会得到不同 Jacobian，代码、库接口和推导必须一致。不能把四元数四个分量直接当作独立最小变量；常用局部三维旋转增量更新单位四元数。

三维图的规范自由度有整体 3 平移和 3 旋转；纯单目还存在尺度，形成 Sim(3) 问题。回环纠正单目尺度漂移时常估计 Sim(3) 相似变换，而不是强行 SE(3)。

## 第 14 章：边缘化与滑动窗口

视觉惯性系统常只优化最近若干帧，旧变量通过 Schur 补边缘化成先验。边缘化先验与线性化点相关；状态大幅改变后旧线性化可能不一致。First-Estimate Jacobian 等策略用于改善一致性。

边缘化不是简单删除节点。删除变量前必须把其信息传递到保留变量，否则历史观测丢失。边缘化会产生更稠密的先验因子，需要控制窗口和消元顺序。

## 第 15 章：增量优化

每来一个关键帧都从头优化全部图会越来越慢。iSAM2 等增量方法只重线性化受影响变量并更新 Bayes tree，在实时 SLAM 中常用。参数包括重线性化阈值和频率，太保守精度下降，太频繁成本升高。

实时系统应分离前端高频 odom 与后端低频全局修正。`odom -> base_link` 保持连续，优化后的全局变化通过 `map -> odom` 表达，避免控制器看到跳变。

## 第 16 章：地图一致性评价

### 16.1 轨迹指标

绝对轨迹误差 ATE 在对齐后比较估计与真值全局位姿；相对位姿误差 RPE 比较固定时间/距离间隔的局部漂移。单目需说明 SE(3) 还是 Sim(3) 对齐。只报 ATE 可能掩盖局部抖动，只报 RPE 又看不到长期全局变形。

### 16.2 地图指标

墙面厚度、重复结构重影、闭环接缝、占据一致性、点云到真值表面距离都可量化。轨迹 ATE 小不保证地图适合导航，时间不同步和外参误差可能让地图模糊。

### 16.3 回环指标

报告候选检索 precision/recall、几何验证通过率、错误接受率、正确回环延迟和后端优化时间。错误接受应单独列出，不能被大量容易的负样本稀释。

## 第 17 章：错误回环故障实验

生成一条方形轨迹的 SE(2) 图：每边 25 个节点，相邻里程计加入小噪声，终点到起点添加正确回环。先优化并记录 ATE。然后分别注入：

1. 错误连接到相邻重复位置。
2. 平移正确但旋转错 180°。
3. 正确回环但信息矩阵放大 1000 倍。
4. 五条相互一致的错误回环。

比较普通二次损失、Huber、可切换约束和前端拒绝。预期单个大残差外点可被鲁棒核缓解，相互一致且初始残差不大的错误集更危险，必须依靠几何与语义验证。

## 第 18 章：综合实验——二维位姿图优化器

### 18.1 功能要求

- 读取 g2o 风格的顶点和边。
- 检查 ID、变换方向和信息矩阵。
- 固定首节点。
- 数值 Jacobian 与解析 Jacobian对照。
- Gauss-Newton 和 LM 两种模式。
- Huber 回环权重。
- 输出每轮成本、增量、条件数和边类型成本。
- 导出优化前后轨迹与残差分布。

### 18.2 测试

三节点直线无噪声应保持不变；加入已知噪声后回环使轨迹接近真值；整体变换初始轨迹不改变相对成本；不固定规范时检测奇异；错误边在无鲁棒核时破坏轨迹；信息矩阵非对称或非正定时拒绝。

### 18.3 与成熟库比较

使用同一图与 GTSAM/g2o/Ceres 结果比较。差异较大时检查残差方向、角度 wrap、信息矩阵上三角读取、扰动约定和固定节点。教学实现通过后，实际系统使用成熟库。

## 第 19 章：后端故障排查

### 优化第一步代价爆炸

检查 Jacobian 符号、左右扰动、边方向、角度单位、更新方式和初值。用单边两节点图做最小复现，数值 Jacobian 对照解析结果。

### 地图优化后镜像或整体翻转

检查坐标手性、旋转矩阵行列式、四元数顺序和固定规范。整体刚体变化不影响相对地图，但镜像不是合法旋转。

### 加回环后地图突然折叠

隔离该回环，查看原始匹配、几何内点、信息矩阵和鲁棒权重。若关闭边后恢复，先修回环验证；不要通过降低所有边权重掩盖。

### 优化越来越慢

检查关键帧增长、边密度、错误全连接、稠密矩阵、重线性化频率和边缘化。用 profiler 分解残差、线性化、因子分解和更新耗时。

## 第 20 章：阶段考试

建议限时 180 分钟，满分 100 分。

### 一、理论题，共 35 分

1. 从高斯因子推导位姿图最小二乘目标。（5 分）
2. 写出 SE(2) 相对位姿残差并解释边方向。（5 分）
3. 为什么只含相对约束的图必须固定规范？（5 分）
4. Gauss-Newton、LM 和鲁棒核分别解决什么问题？（5 分）
5. 信息矩阵中的米和弧度如何权衡？（5 分）
6. 为什么回环候选不能直接成为图边？（5 分）
7. 比较 ATE、RPE 和地图一致性指标。（5 分）

### 二、代码题，共 30 分

1. 为 SE(2) 残差和 Jacobian 设计十项测试。（10 分）
2. 实现固定首节点的高斯牛顿优化循环。（10 分）
3. 设计错误回环的分级处理状态机。（10 分）

### 三、综合题，共 35 分

1. 加入一条回环后 ATE 下降但地图墙面变厚，分析原因和实验。（15 分）
2. 一个仓库数据集回环 precision 为 99.5%，仍偶尔破坏地图。解释为什么，并设计安全评价。（20 分）

## 第 21 章：参考答案

### 一、理论题

1. 条件独立高斯测量概率相乘，取负对数后乘积变求和，每项为残差 Mahalanobis 平方，信息矩阵是协方差逆，加上先验得到 MAP 最小二乘。

2. 若测量是 $i$ 到 $j$，误差可写 $\Log(Z_{ij}^{-1}T_i^{-1}T_j)$。交换边方向必须同时求测量逆和调整信息表达，不能只交换 ID。

3. 整体平移/旋转不改变相对残差，存在无限等价解，Hessian 有零空间。固定一个位姿或添加先验选择坐标规范。

4. Gauss-Newton 用局部二次近似求非线性最小二乘；LM 用阻尼改善差初值下的步长；鲁棒核降低外点大残差影响。三者都不能替代正确测量模型和回环验证。

5. 用测量协方差设定平移与旋转标准差及相关项，信息矩阵进行统计归一化。单位阵隐含任意等价尺度，应由前端真值实验校准并检查退化方向。

6. 外观相似会在重复结构中产生假候选，必须经过描述子/语义、几何 RANSAC、内点覆盖、重叠、Hessian、多帧和拓扑一致性验证。

7. ATE 衡量对齐后的全局轨迹，RPE 衡量局部时间/距离间隔漂移，地图指标衡量表面重影和占据一致性。三者互补。

### 二、代码题

残差/Jacobian 测试包括单位边、已知平移、已知旋转、角度跨界、测量逆、共同全局变换不变、随机数值 Jacobian、近平角、错误维度和非有限输入。优化循环应组装块 Hessian 和梯度，删除或固定首块，解增量，在 SE(2) 更新并检查代价下降/增量/迭代上限，同时保留诊断。

回环状态机可为 candidate → geometrically_verified → temporally_confirmed → low_weight_active → active；任一阶段失败进入 rejected 并保存原因。优化后残差或地图一致性异常可降级/quarantine，不能静默删除证据。

### 三、综合题

ATE 下降只说明轨迹点更接近对齐真值，墙变厚可能来自时间同步、外参、地图点未随优化重建、局部姿态抖动、回环信息过强或轨迹评价采样不足。分别用优化后位姿重新建图、检查 RPE、传感器时序、墙面残差和信息权重，比较关闭回环及真值位姿建图。

99.5% precision 在百万候选中仍可能有大量错误，且一次错误回环损失远大于漏掉一次正确回环；平均 precision 还会被简单场景主导。应报告每公里/小时错误接受、最坏序列、重复结构切片、错误边对地图和导航的后果、几何验证与后端鲁棒层的独立失效率，并进行故障注入和长时间压力测试。

## 本篇完成标准

完成本篇后，应能从测量概率写出位姿图残差和信息矩阵；能解释规范自由度、稀疏性和流形更新；能实现并验证小型 SE(2) 优化器；能区分数值收敛与约束正确；能设计多级回环验证和后端鲁棒防线；能用 ATE、RPE、地图与回环风险共同评价系统。

下一阶段将进入地图表示、定位与全局规划：占据栅格、inverse sensor model、log-odds、代价地图、AMCL、A*/Dijkstra/Hybrid A* 和地图质量评价。
