---
title: 第一册：机器人数学例题与阶段考试
description: 坐标变换、概率估计、最小二乘和优化的完整例题、推导与答案。
---

# 第一册：机器人数学例题与阶段考试

## 例题 1：坐标系链式变换

机器人基座坐标系为 $B$，相机坐标系为 $C$，目标坐标系为 $O$。已知相机安装位姿：

$$
T_{BC}=
\begin{bmatrix}
0&-1&0&0.2\\
1&0&0&0\\
0&0&1&0.5\\
0&0&0&1
\end{bmatrix}
$$

视觉算法输出目标在相机坐标系中的位姿：

$$
T_{CO}=
\begin{bmatrix}
1&0&0&0.4\\
0&1&0&0.1\\
0&0&1&1.0\\
0&0&0&1
\end{bmatrix}
$$

求目标在基座中的位姿。

### 解答

坐标链为 $O\rightarrow C\rightarrow B$：

$$T_{BO}=T_{BC}T_{CO}$$

旋转：

$$R_{BO}=R_{BC}R_{CO}=R_{BC}$$

平移：

$$t_{BO}=R_{BC}t_{CO}+t_{BC}$$

$R_{BC}[0.4,0.1,1]^T=[-0.1,0.4,1]^T$，加安装平移得：

$$t_{BO}=[0.1,0.4,1.5]^T$$

工程检查：如果直接把两个平移相加，会得到 `[0.6,0.1,1.5]`，因为忽略了相机坐标中的平移需要先旋转到基座。

```python
import numpy as np

T_bc = np.array([[0,-1,0,.2],[1,0,0,0],[0,0,1,.5],[0,0,0,1.]], float)
T_co = np.array([[1,0,0,.4],[0,1,0,.1],[0,0,1,1.],[0,0,0,1.]], float)
T_bo = T_bc @ T_co
assert np.allclose(T_bo[:3, 3], [0.1, 0.4, 1.5])
```

## 例题 2：逆变换

已知 $p_A=T_{AB}p_B$，证明：

$$
T_{AB}^{-1}=
\begin{bmatrix}
R^T&-R^Tt\\0&1
\end{bmatrix}
$$

### 推导

由 $p_A=Rp_B+t$：

$$p_B=R^{-1}(p_A-t)=R^Tp_A-R^Tt$$

因此逆变换平移是 $-R^Tt$，不是简单 $-t$。只有 $R=I$ 时两者相同。

验证应检查：

$$T_{AB}T_{AB}^{-1}=I$$

浮点计算使用 `np.allclose`，不要使用逐元素严格相等。

## 例题 3：旋转矩阵修复

数值积分得到近似矩阵：

$$
\tilde R=
\begin{bmatrix}
1&-0.01&0\\0.0102&0.9998&0\\0&0&1.0001
\end{bmatrix}
$$

它不严格属于 SO(3)。使用 SVD：

$$\tilde R=U\Sigma V^T,\quad R=UV^T$$

若 $\det(UV^T)<0$，翻转一个奇异向量以避免反射。

```python
def project_to_so3(M):
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R
```

投影适合修复小数值漂移，不能把任意错误矩阵“洗成正确旋转”。

## 例题 4：四元数符号

两个相邻姿态四元数 $q_1$ 和 $q_2$ 点积为负。由于 $q$ 与 $-q$ 同旋转，插值前令：

$$q_2\leftarrow-q_2$$

使点积为正，SLERP 走较短路径。否则可视化可能出现无意义大旋转。

## 例题 5：点到平面距离

平面由单位法向 $n$ 和平面上一点 $p_0$ 定义。点 $p$ 到平面的有符号距离：

$$d=n^T(p-p_0)$$

若 $n$ 未归一化，几何距离为：

$$d=\frac{n^T(p-p_0)}{\|n\|}$$

ICP 点到平面残差使用这一形式。法向方向翻转会改变残差符号，但平方损失不变；涉及接触方向时符号很重要。

## 例题 6：最小二乘直线拟合

模型 $y=ax+b$。构造：

$$
A=\begin{bmatrix}x_1&1\\\vdots&\vdots\\x_n&1\end{bmatrix},
\quad \theta=\begin{bmatrix}a\\b\end{bmatrix}
$$

求：

$$\theta^*=\arg\min_\theta\|A\theta-y\|^2$$

不要使用显式逆：

```python
theta, residuals, rank, singular = np.linalg.lstsq(A, y, rcond=None)
```

若所有 $x_i$ 相同，$A$ 不满秩，无法分别估计斜率和截距。程序应检查 `rank`。

## 例题 7：加权融合两个测量

两个独立一维高斯测量：

$$z_1\sim\mathcal N(x,\sigma_1^2),\quad z_2\sim\mathcal N(x,\sigma_2^2)$$

最大似然估计：

$$\hat x=\frac{z_1/\sigma_1^2+z_2/\sigma_2^2}{1/\sigma_1^2+1/\sigma_2^2}$$

融合方差：

$$\sigma^2=\frac1{1/\sigma_1^2+1/\sigma_2^2}$$

方差小的测量权重大。若两个测量相关，直接使用此公式会重复计算信息。

## 例题 8：协方差传播

极坐标到直角坐标：

$$x=r\cos\theta,\quad y=r\sin\theta$$

Jacobian：

$$
J=\begin{bmatrix}
\cos\theta&-r\sin\theta\\
\sin\theta&r\cos\theta
\end{bmatrix}
$$

$$\Sigma_{xy}\approx J\Sigma_{r\theta}J^T$$

距离越远，角度噪声产生的位置横向误差越大。这个结论解释了远距离激光/视觉角度误差为何显著。

## 例题 9：Mahalanobis 门限

预测测量均值 $\mu$、协方差 $S$，实际测量 $z$：

$$d^2=(z-\mu)^TS^{-1}(z-\mu)$$

在二维高斯假设下，可用卡方分布阈值做数据关联门控。使用普通欧氏距离会忽略不同方向的不确定性。

计算时不要显式 `inv(S)`：

```python
delta = z - mu
d2 = delta.T @ np.linalg.solve(S, delta)
```

## 例题 10：Gauss-Newton 拟合指数曲线

模型：

$$\hat y_i=ae^{bx_i}+c$$

残差：

$$r_i=ae^{bx_i}+c-y_i$$

Jacobian 行：

$$J_i=[e^{bx_i},\ ax_ie^{bx_i},\ 1]$$

更新：

$$\Delta\theta=-(J^TJ)^{-1}J^Tr$$

实现使用 `solve`，并加入停止条件：参数步长、损失变化、最大迭代。指数溢出时检查输入范围和初值。

## 例题 11：Huber 鲁棒拟合

Huber 损失：

$$
\rho(r)=
\begin{cases}
\frac12r^2,&|r|\le\delta\\
\delta(|r|-\frac12\delta),&|r|>\delta
\end{cases}
$$

迭代重加权最小二乘中，大残差权重下降。$\delta$ 应结合预期噪声尺度，数据未归一化时统一阈值没有意义。

## 例题 12：二连杆雅可比

$$
J=\begin{bmatrix}
-l_1\sin q_1-l_2\sin(q_1+q_2)&-l_2\sin(q_1+q_2)\\
l_1\cos q_1+l_2\cos(q_1+q_2)&l_2\cos(q_1+q_2)
\end{bmatrix}
$$

当两连杆完全伸直，Jacobian 降秩，末端某个方向瞬时不可达。使用数值差分验证解析式。

## 例题 13：阻尼伪逆

$$J^+_\lambda=J^T(JJ^T+\lambda^2I)^{-1}$$

$\lambda$ 越大，关节速度更小、更稳定，但末端跟踪误差增大。可根据最小奇异值自适应调整阻尼。

## 例题 14：梯度下降步长

函数 $f(x)=\frac12ax^2$，梯度 $ax$，更新：

$$x_{k+1}=(1-\eta a)x_k$$

收敛要求：

$$|1-\eta a|<1\Rightarrow0<\eta<\frac2a$$

这说明曲率大方向允许学习率更小。多维问题条件数大时，不同方向收敛速度差，归一化和二阶方法有价值。

## 例题 15：softmax 数值稳定

直接计算 $e^{1000}$ 溢出。利用平移不变性：

$$\operatorname{softmax}(x_i)=\frac{e^{x_i-m}}{\sum_je^{x_j-m}},\quad m=\max_jx_j$$

不要自己重复实现生产级交叉熵，优先使用框架稳定算子。

## 阶段考试 A：基础计算

1. 计算两个给定齐次变换的组合和逆。
2. 判断矩阵是否属于 SO(3)。
3. 求向量到平面的投影和距离。
4. 计算线性变换后的均值和协方差。
5. 写出二维极坐标转换 Jacobian。

## 阶段考试 B：建模

1. 用最小二乘建模轮径标定。
2. 为相机和激光位置测量设计加权融合。
3. 写出点到平面 ICP 残差。
4. 说明 PnP 点近共线时为什么不稳定。
5. 设计检测离群数据的 Mahalanobis 门限流程。

## 阶段考试 C：代码

实现以下函数并配 `pytest`：

```python
def compose(T_ab, T_bc): ...
def inverse(T): ...
def so3_exp(phi): ...
def numerical_jacobian(fn, x): ...
def weighted_least_squares(A, b, covariance): ...
def huber_weights(residuals, delta): ...
```

测试零输入、随机输入、退化输入、非法 shape、NaN 和极端尺度。

## 考试答案与评分

- 组合顺序正确、坐标系解释完整：20%。
- 推导过程和维度正确：20%。
- 不显式求逆、处理病态和异常：20%。
- 代码测试覆盖边界：20%。
- 能说明工程失败现象：20%。

关键答案：旋转判定检查 $R^TR=I$ 与 $\det R=1$；协方差传播为 $J\Sigma J^T$；加权最小二乘权重是协方差逆；病态问题应检查奇异值、点分布和单位，而不是仅加一个很大正则项。

得分不足 80 时，应重新完成对应例题并修改代码，不能只阅读答案。
