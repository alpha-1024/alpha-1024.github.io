---
title: 第二册第三篇：深度相机、点云处理与三维配准
description: 系统讲解深度测量误差、点云结构、邻域与法线、滤波分割、ICP/NDT、ROS2 数据接口及完整实验。
---

# 第二册第三篇：深度相机、点云处理与三维配准

二维图像告诉机器人“哪里看起来像目标”，深度和点云进一步告诉机器人“目标在空间中的哪里、表面朝向如何、能否通过或抓取”。三维数据并不天然可靠：每种深度传感器都有特定失效材料、距离范围和噪声结构，点云算法又强烈依赖尺度、密度、重叠和初值。

本篇从传感原理开始，建立一条可验证的工程链路：读取深度并确认单位，依据相机模型反投影，保存坐标系和时间戳，通过体素和邻域方法估计几何属性，使用鲁棒模型分割结构，再用 ICP 或 NDT 对齐多帧数据。最后不只看库函数返回的 `fitness`，而是用已知变换、重叠区域、残差分布和任务误差判断结果是否可信。

## 第 1 章：深度传感器究竟测量什么

### 1.1 双目深度

已校正的平行双目通过左右图水平视差 $d$ 计算深度：

$$
Z=\frac{f_xB}{d},
$$

其中 $f_x$ 是像素焦距，$B$ 是基线。被动双目依赖场景纹理；白墙、重复栅栏、镜面和弱光区域难以匹配。主动双目投射红外纹理，使无纹理表面出现可匹配图案，但多台设备互相干扰或强太阳光会降低效果。

视差量化误差导致深度方差随距离快速增长：

$$
\sigma_Z\approx\frac{Z^2}{f_xB}\sigma_d.
$$

因此“标称最远 10 米”不等于 10 米处仍满足定位精度。选型时必须把工作距离、焦距、基线和任务容许误差放在同一张表中估算。

### 1.2 结构光

结构光向场景投射已知图案，通过图案变形三角化深度。近距离室内精度通常较好，但强环境光会淹没投影，透明、黑色吸光和镜面材料也会造成孔洞。投影器与相机之间仍存在基线，所以物体边缘会出现只被一侧看到的遮挡区域。

### 1.3 飞行时间

ToF 通过光的往返时间或调制相位估计距离。相位式 ToF 可简化为

$$
d=\frac{c}{4\pi f_m}\phi,
$$

其中 $f_m$ 是调制频率，$\phi$ 是相位差。相位具有周期性，超过无模糊距离会产生绕回，需要多频解模糊。多径反射使光经过墙角、地面或半透明材料后再返回，测得距离不是直接路径。飞点常出现在物体边缘，因为一个像素混合前景和背景回波。

### 1.4 激光雷达

旋转式或固态激光雷达输出每束激光的距离、角度、强度和时间。点云不是严格的同一时刻快照：一帧扫描期间机器人持续运动，若不使用 IMU/里程计去畸变，墙面会弯曲，配准会把运动畸变误认为环境结构。

激光雷达的距离通常是沿射线的 range，而 RGB-D 深度图常保存相机 $z$ 方向的 $Z$。两者转换时要区别：对单位方向 $r=(r_x,r_y,r_z)$，若量到 range $\rho$，三维点为 $P=\rho r$，而其轴向深度为 $Z=\rho r_z$。

### 1.5 传感器选型不是排名

选择深度传感器至少比较：工作距离、视场、最小距离、分辨率、帧率、曝光和同步、室外能力、材料适应性、尺寸功耗、SDK、时间戳质量和实际误差。机械臂桌面抓取、室内导航和室外高速移动的最优选择不会相同。

## 第 2 章：深度数据的单位、无效值与时间

### 2.1 无效值编码

深度图可能用 `uint16` 毫米保存，零表示无效；也可能用 `float32` 米保存，`NaN` 表示无效。读取后第一件事不是可视化，而是检查 dtype、scale、最小有效距离、最大有效距离以及无效值比例。

```python
from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class DepthStats:
    total: int
    valid: int
    valid_ratio: float
    minimum_m: float | None
    median_m: float | None
    maximum_m: float | None

def normalize_depth(depth: np.ndarray, unit_scale: float,
                    minimum_m: float, maximum_m: float):
    if depth.ndim != 2:
        raise ValueError(f"depth must be HxW, got {depth.shape}")
    if unit_scale <= 0 or minimum_m <= 0 or maximum_m <= minimum_m:
        raise ValueError("invalid scale or range")
    depth_m = depth.astype(np.float32) * unit_scale
    valid = (
        np.isfinite(depth_m)
        & (depth_m >= minimum_m)
        & (depth_m <= maximum_m)
    )
    values = depth_m[valid]
    stats = DepthStats(
        total=depth_m.size,
        valid=int(valid.sum()),
        valid_ratio=float(valid.mean()),
        minimum_m=float(values.min()) if values.size else None,
        median_m=float(np.median(values)) if values.size else None,
        maximum_m=float(values.max()) if values.size else None,
    )
    return depth_m, valid, stats
```

对 `uint16` 毫米图，`unit_scale=0.001`；对已经以米保存的浮点图，使用 `1.0`。不要根据数值大小自动猜单位，因为近距离毫米值和远距离米值可能重叠，猜测会让错误悄悄进入系统。

### 2.2 深度和彩色对齐

RGB 相机与深度相机通常有不同光心、内参和视场。所谓 aligned depth 是把深度点变换到 RGB 相机坐标，再投影到 RGB 像素网格。这个过程可能产生孔洞、多个深度竞争同一像素和插值误差。

读取相机 SDK 输出时要确认：深度是否已经对齐；`CameraInfo` 属于哪一个光学坐标系；对齐后应使用哪组内参；图像是否裁剪或缩放。若对齐图使用了原深度相机内参，中心附近可能看似正确，边缘误差会明显增大。

### 2.3 时间同步和运动错位

RGB 与深度相差 30 ms，在静态桌面上不明显；机械臂末端以 1 m/s 运动时，忽略旋转和投影也已产生约 3 cm 位移。近似同步容差必须根据最大相对速度和允许空间误差倒推：

$$
\Delta t_{max}\lesssim\frac{e_{max}}{v_{max}}.
$$

这只是平移的一阶上界，旋转、视差和处理延迟还会增加误差。日志应同时保存传感器时间戳、主机接收时间和算法输出时间，以区分采集不同步与计算过慢。

## 第 3 章：深度图反投影为有组织点云

### 3.1 向量化反投影

对轴向深度 $Z$：

$$
X=\frac{u-c_x}{f_x}Z,\qquad
Y=\frac{v-c_y}{f_y}Z.
$$

```python
def depth_to_organized_points(depth_m: np.ndarray, valid: np.ndarray,
                              K: np.ndarray) -> np.ndarray:
    if depth_m.shape != valid.shape:
        raise ValueError("depth and mask shape mismatch")
    K = np.asarray(K, dtype=np.float64)
    if K.shape != (3, 3) or K[0, 0] <= 0 or K[1, 1] <= 0:
        raise ValueError("invalid camera matrix")
    height, width = depth_m.shape
    v, u = np.indices((height, width), dtype=np.float32)
    z = depth_m.astype(np.float32)
    x = (u - K[0, 2]) * z / K[0, 0]
    y = (v - K[1, 2]) * z / K[1, 1]
    points = np.stack((x, y, z), axis=-1)
    points[~valid] = np.nan
    return points
```

输出形状为 `[H,W,3]`，保留了像素邻接关系，称为有组织点云。它适合快速邻域、边缘和法线估计。删除无效点后得到 `[N,3]` 的无组织点云，存储紧凑，适合 KD-tree 和通用几何算法。

### 3.2 同时附加颜色

若 RGB 与深度已对齐，可以用同一个像素索引给三维点赋色。OpenCV 默认 BGR，而 Open3D 颜色通常按 RGB 且范围为 `[0,1]`：

```python
def flatten_valid_cloud(points_hw3, bgr_image, valid):
    if bgr_image.shape[:2] != valid.shape:
        raise ValueError("color and depth are not aligned")
    xyz = points_hw3[valid].astype(np.float64)
    rgb = bgr_image[..., ::-1][valid].astype(np.float64) / 255.0
    return xyz, rgb
```

点的坐标系必须随数据保存。例如同一个点云文件旁保存 `frame_id=camera_depth_optical_frame`、时间戳、单位和外参版本。仅保存 `cloud.ply` 会在数周后失去关键语义。

### 3.3 反投影的测试

至少验证：主点像素反投影到光轴；图像右侧点具有正 $X$；图像下方点在 OpenCV 光学系中具有正 $Y$；深度翻倍使三维坐标整体翻倍；无效像素变为 `NaN`；米和毫米输入经正确 scale 后结果一致。

## 第 4 章：点云预处理的尺度逻辑

### 4.1 范围裁剪

先按任务工作空间裁剪，可以减少后续计算并避免无关结构支配配准。机械臂桌面任务可在 `base_link` 中定义盒状 ROI；相机坐标裁剪随相机运动而变化，未必等价。

范围必须使用明确坐标系。若在 optical frame 中把 `z` 当成高度，会错误删除远近点；REP-103 的 base frame 才通常用 `z` 表示向上。

### 4.2 体素降采样

体素栅格把空间分成边长 $v$ 的立方体，每个非空体素用质心或代表点替代。它降低密度并让不同距离的点权重更均衡。体素太小几乎不减点，太大会抹掉物体边缘和薄结构。

参数应与任务最小结构和传感噪声相关。若要检测直径 15 mm 的杆，使用 30 mm 体素已从信息层面破坏目标；若深度噪声本身约 10 mm，使用 0.1 mm 体素只增加计算。

### 4.3 统计离群点

对每个点计算 $k$ 个邻居的平均距离，超过全局均值若干标准差的点被视为离群。它对孤立噪声有效，但会误删稀疏边界和小物体，因为这些区域真实邻居也少。

### 4.4 半径离群点

统计固定半径内邻居数，少于阈值则删除。固定半径假设点密度大致一致；透视深度相机和激光雷达的点间距随距离增长，远处正常点更容易被误删。可以按距离设置半径，或先体素化到较均匀密度。

### 4.5 顺序不是任意的

常见流程是有限性和范围检查 → 粗 ROI → 体素降采样 → 离群点处理。若先在千万点原云上做昂贵邻域搜索，浪费计算；若先激进滤波，可能删掉后续所需结构。每一步都要记录输入点数、输出点数和空间范围，出现“结果空了”时才能定位是哪一步导致。

## 第 5 章：邻域搜索与 KD-tree

### 5.1 三类邻域

固定 $k$ 近邻保证每个点都有相同数量邻居，但稀疏区域会跨越很大空间；固定半径保证物理尺度一致，但密度低时可能没有足够点；混合搜索同时限制最大半径和最多邻居，常用于 Open3D 法线和特征计算。

邻域尺度决定你在描述哪种几何。半径 5 mm 可能看到表面粗糙度，50 mm 看到物体局部曲面，500 mm 看到墙面与房间结构。没有脱离尺度的“点云法线”。

### 5.2 KD-tree 的适用边界

KD-tree 递归划分空间，适合三维中低维最近邻。构建约为 $O(N\log N)$，查询平均较快。高维描述子可能受维数灾难影响，通常使用近似最近邻库。点云持续变化时反复重建树也有成本，在线系统应评估增量结构或局部地图策略。

## 第 6 章：法线、曲率与方向一致性

### 6.1 PCA 法线估计

对点 $p$ 的邻域 $q_i$，计算质心 $\bar q$ 和协方差

$$
C=\frac{1}{N}\sum_i(q_i-\bar q)(q_i-\bar q)^T.
$$

设特征值 $\lambda_0\le\lambda_1\le\lambda_2$，最小特征值对应的特征向量是局部法线，因为沿表面法向变化最小。局部表面变化可近似为

$$
\kappa=\frac{\lambda_0}{\lambda_0+\lambda_1+\lambda_2}.
$$

### 6.2 退化邻域

邻居几乎共线时，两个小特征值接近，法线方向不稳定；邻居太少或重复点也会使协方差退化。实现应输出质量指标，而不是无条件返回单位向量。

```python
def estimate_normal_pca(neighbors: np.ndarray, eps=1e-12):
    points = np.asarray(neighbors, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3:
        raise ValueError("at least three 3D neighbors are required")
    if not np.isfinite(points).all():
        raise ValueError("neighbors must be finite")
    centered = points - points.mean(axis=0)
    covariance = centered.T @ centered / len(points)
    values, vectors = np.linalg.eigh(covariance)
    if values.sum() < eps or values[1] < eps:
        return None, 1.0
    normal = vectors[:, 0]
    curvature = float(values[0] / values.sum())
    return normal, curvature
```

### 6.3 法线有符号歧义

$n$ 和 $-n$ 描述同一几何平面。为了显示、点到平面 ICP 或抓取，需要统一朝向。单视角深度云可让法线朝向相机：若 $n\cdot(v-p)<0$ 就翻转，其中 $v$ 是相机位置。多视图融合后应沿邻接图传播方向，或依据物体内外先验确定。

## 第 7 章：RANSAC 平面与几何分割

### 7.1 三点定义平面

三个不共线点 $p_1,p_2,p_3$ 定义法线

$$
n=\frac{(p_2-p_1)\times(p_3-p_1)}
{\|(p_2-p_1)\times(p_3-p_1)\|},
$$

平面写成 $n^Tp+d=0$，其中 $d=-n^Tp_1$。点到归一化平面的距离为 $|n^Tp+d|$。

共线或距离过近的样本必须拒绝。深度噪声随距离增加时，固定距离阈值会对近处过宽、远处过严，可使用距离相关阈值或先将残差除以预测标准差。

### 7.2 最大平面未必是桌面

室内点云中最大平面可能是墙或地面。要找桌面，还应利用法线方向、高度范围、面积、边界和机器人工作区。算法输出“最大一致平面”只是几何事实，语义需要额外约束。

### 7.3 平面移除与聚类

桌面抓取常先分割支撑平面，再对剩余点做欧氏聚类。聚类半径应匹配降采样后的点距；太小会把一个物体碎裂，太大会合并相邻物体。物体接触或遮挡时，纯几何聚类很难分离，需要颜色、法线、凹凸边界或学习分割。

## 第 8 章：刚体配准问题

给定源点 $p_i$ 和目标对应点 $q_i$，求旋转 $R\in SO(3)$ 与平移 $t$：

$$
\min_{R,t}\sum_i\|q_i-(Rp_i+t)\|^2.
$$

若对应关系已知，可通过 SVD 得到闭式解。令两组质心为 $\bar p,\bar q$，中心化后构造

$$
H=\sum_i(p_i-\bar p)(q_i-\bar q)^T.
$$

对 $H=U\Sigma V^T$，旋转为

$$
R=V\operatorname{diag}(1,1,\det(VU^T))U^T,
$$

平移为 $t=\bar q-R\bar p$。行列式修正确保 $R$ 是旋转而不是反射。

```python
def rigid_transform_svd(source: np.ndarray, target: np.ndarray):
    source = np.asarray(source, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError("source and target must both be [N,3]")
    if len(source) < 3 or not np.isfinite(source).all() or not np.isfinite(target).all():
        raise ValueError("need at least three finite pairs")
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    source_zero = source - source_center
    target_zero = target - target_center
    H = source_zero.T @ target_zero
    U, singular_values, Vt = np.linalg.svd(H)
    if singular_values[1] < 1e-10:
        raise ValueError("degenerate correspondence geometry")
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(Vt.T @ U.T)
    R = Vt.T @ correction @ U.T
    t = target_center - R @ source_center
    return R, t
```

用随机三维点和已知变换生成无噪声对应，恢复结果应接近机器精度；再测试共线点、重复点、反射数据和含外点数据。闭式解不具备外点鲁棒性，错误对应需要 RANSAC 或鲁棒损失。

## 第 9 章：ICP 的完整逻辑

### 9.1 交替优化

ICP 在对应关系和变换之间交替：用当前变换把源云移到目标附近；为每个源点寻找目标最近邻；拒绝距离过大或不可靠对应；求解增量刚体变换；更新并检查收敛。

这是局部优化。初值差时，最近邻对应本身就是错的，后续每一步都可能稳定地收敛到错误局部极小。ICP 的“converged”通常只表示更新变小，并不表示到达真实位姿。

### 9.2 点到点 ICP

点到点目标就是上一章的欧氏距离。每轮可用 SVD 求解。它不需要目标法线，适合通用点集，但沿平滑表面的收敛较慢。

### 9.3 点到平面 ICP

若目标点 $q_i$ 有法线 $n_i$，目标函数为

$$
\min_{R,t}\sum_i[n_i^T(Rp_i+t-q_i)]^2.
$$

它只惩罚法向距离，对表面切向滑动不敏感，却能更快对齐局部平面。法线错误、场景只有单一平面或几何结构不足时，某些自由度不可观。

用小旋转 $R\approx I+[\omega]_\times$ 线性化，残差

$$
r_i\approx n_i^T(p_i+t-q_i)-n_i^T[p_i]_\times\omega.
$$

因此对增量 $\xi=(\omega,t)$ 的雅可比可写为

$$
J_i=\begin{bmatrix}(p_i\times n_i)^T&n_i^T\end{bmatrix}.
$$

正规矩阵 $J^TJ$ 的小特征值揭示不可观方向。例如只观察一面无限平墙时，沿墙平移和绕墙法线旋转难以由点到平面误差约束。

### 9.4 对应点拒绝

常用策略包括最大距离、法线夹角、边界剔除、互为最近邻、裁剪最高残差比例和鲁棒核。最大距离应随金字塔层逐渐减小；一开始过小可能没有对应，一直过大则易吸收错误结构。

### 9.5 多尺度 ICP

先使用大体素、较宽对应距离估计粗变换，再在细体素和窄阈值下精化。每一层应重新计算目标法线，法线半径通常为体素尺寸的若干倍。

```python
import copy
import open3d as o3d

def multiscale_icp(source, target, initial, voxel_sizes=(0.08, 0.04, 0.02)):
    transform = np.asarray(initial, dtype=np.float64).copy()
    diagnostics = []
    for voxel in voxel_sizes:
        src = source.voxel_down_sample(voxel)
        tgt = target.voxel_down_sample(voxel)
        radius = voxel * 3.0
        tgt.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=40)
        )
        maximum_distance = voxel * 2.0
        result = o3d.pipelines.registration.registration_icp(
            src, tgt, maximum_distance, transform,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50),
        )
        transform = result.transformation
        diagnostics.append({
            "voxel": voxel,
            "source_points": len(src.points),
            "target_points": len(tgt.points),
            "fitness": float(result.fitness),
            "inlier_rmse": float(result.inlier_rmse),
        })
    return transform, diagnostics
```

函数没有自动证明初值可用。调用前仍需全局特征、里程计或机械结构提供粗对齐；调用后要做独立验证。

## 第 10 章：NDT 配准

### 10.1 从点对应到概率分布

NDT 把目标空间划分为体素，在每个含足够点的体素中估计均值 $\mu_k$ 和协方差 $\Sigma_k$。变换后的源点 $x'=Rx+t$ 落入某体素时，根据

$$
(x'-\mu_k)^T\Sigma_k^{-1}(x'-\mu_k)
$$

评价匹配。它不显式建立逐点最近邻，对分辨率合适的激光地图配准常有较大捕获范围。

### 10.2 分辨率的影响

体素太小，每格点数不足，协方差不稳定；太大则不同结构混在一个高斯分布中，目标函数失去细节。协方差还需要正则化，避免平面点造成奇异矩阵。

NDT 同样依赖初值，也可能在重复走廊和稀疏环境中得到错误峰值。不能把“没有显式最近邻”理解为“全局优化”。

### 10.3 ICP 与 NDT 的选择

ICP 直接、易解释，点到平面版本在局部高质量表面上精度高；NDT 对密度变化和一定初始偏差可能更平滑，常用于车辆激光定位。实践中可以 NDT 粗配准、ICP 精化，也可以用特征全局配准后 ICP。选择应通过初值扰动实验比较成功域，而不是只在真值附近比较最终 RMSE。

## 第 11 章：全局配准与局部配准

没有可靠初值时，需要建立更具辨识力的三维特征对应，例如 FPFH，再通过 RANSAC 或快速全局配准估计粗姿态。一般流程为：体素降采样；估计法线；计算特征；匹配特征；鲁棒估计粗变换；多尺度 ICP 精化。

全局特征也会在对称物体、重复结构和低重叠中失败。圆柱绕轴旋转不可由几何唯一确定，完全相同的货架单元会产生多解。此时应加入颜色、语义、重力方向、机械关节先验或时间连续性。

### 11.1 对称性必须进入评价

若物体有对称群 $\mathcal S$，预测姿态与真值的误差应考虑所有等价变换：

$$
e=\min_{S\in\mathcal S}d(T_{pred},T_{gt}S).
$$

否则算法预测了物理等价姿态，却被指标判为错误；反过来，控制任务可能仍需特定抓取朝向，所以评价还要结合任务约束。

## 第 12 章：配准指标和不确定性

### 12.1 Fitness 与 RMSE 的陷阱

Open3D `fitness` 常表示在距离阈值内的源点比例，`inlier_rmse` 只统计这些内点。增大阈值通常提高 fitness，也可能增加 RMSE；裁剪源云只保留重叠区域也会显著提高 fitness。不同阈值和预处理下的数值不能直接横向比较。

必须额外报告：真实重叠率、对应点数量、平移与旋转误差、成功率、初值扰动范围、运行时间、失败模式和变换物理合理性。有真值时，旋转误差可写为

$$
e_R=\arccos\left(\frac{\operatorname{trace}(R_{gt}^TR_{est})-1}{2}\right),
$$

平移误差为 $e_t=\|t_{est}-t_{gt}\|$。

### 12.2 Hessian 与可观性

在线性化附近，信息矩阵近似为 $H=J^TWJ$。小特征值对应弱约束方向，可用于发现走廊、单平面和重复结构退化。直接把 $H^{-1}$ 当作真实协方差需要谨慎，因为对应关系、外点和模型误差并不满足理想高斯假设，但它仍能提供相对质量信号。

### 12.3 一致性检查

将 A 配准到 B 得 $T_{BA}$，再把 B 配准回 A 得 $T_{AB}$，理想情况下 $T_{AB}T_{BA}\approx I$。多帧还可检查环路组合是否接近单位变换。双向一致不能证明绝对正确，但能发现部分方向性错误和局部极小。

## 第 13 章：ROS2 PointCloud2 工程接口

### 13.1 消息语义

`sensor_msgs/msg/PointCloud2` 通过字段描述二进制布局。常见字段有 `x,y,z,intensity,rgb,ring,time`。不能假设所有云都是连续的 `[float x, float y, float z]`；字段偏移、类型、步长和字节序都由消息元数据决定。

`header.frame_id` 表示点坐标所属坐标系，`header.stamp` 表示采样时间。处理后发布点云时，若已经把点变换到 `base_link`，必须同步修改 `frame_id`；只改 frame 名而不变换数值是严重错误。

### 13.2 TF 查询时间

应查询点云采样时刻的变换，而不是一律使用“最新变换”。机器人运动时，最新姿态与采样姿态不同会扭曲地图。若 TF 缓存中没有对应时间，应记录缺失并根据任务选择丢帧或等待，不能静默替换为当前时间。

### 13.3 节点处理预算

假设传感器 30 Hz，每帧处理时间必须稳定低于约 33 ms 才能持续在线；平均 20 ms 但 P99 为 80 ms 仍可能积压。订阅队列过深会让系统处理旧点云。对实时感知通常使用小队列，过载时丢旧帧并暴露诊断指标。

日志至少包括输入频率、处理 P50/P95/P99、点数、过滤比例、TF 失败、空云、配准成功率和输出时间戳延迟。

## 第 14 章：四个完整失败案例

### 案例一：桌面平面法线突然翻转

现象是桌面位置稳定，但发布姿态绕某轴跳变 180 度。先检查法线估计是否存在 $n/-n$ 未统一，再确认 PCA 邻域是否跨越桌边，最后检查姿态构造时切向轴如何确定。只让法线朝向相机仍不能确定绕法线旋转，需要用桌面边缘、机器人轴或上一帧方向补充约束。

### 案例二：ICP 显示收敛，点云却错位到相邻货架

重复货架提供多个相似局部极小，ICP 更新变小所以报告收敛。诊断要比较初值扰动、多起点结果、语义标志、闭环一致性和位姿先验。解决方案可能是扩大局部地图的唯一结构、加入全局描述、使用里程计约束，并在结果偏离预测过大时拒绝，而不是简单增加迭代次数。

### 案例三：彩色点云出现物体彩边

静态场景正常，机械臂运动时目标边缘出现红蓝错位。先核对 RGB 和深度时间戳，计算运动速度乘时间差的误差上界；再检查外参、对齐方向和滚动快门。若减慢机器人后彩边显著减小，时间同步比静态外参更可能是主因。

### 案例四：远处墙面被统计滤波删除

透视点云随距离变稀，固定邻域参数把远处正常点判作离群。按距离切片统计邻居距离，确认阈值是否随距离增长。可先体素化、采用距离自适应半径，或对不同传感区域使用不同噪声模型。不能只调宽全局阈值，否则近处真实离群点又会保留。

## 第 15 章：综合实验一——RGB-D 桌面分割

### 15.1 任务

输入对齐的 RGB、深度和相机内参，输出桌面平面、桌上物体簇、每个簇的轴对齐包围盒和质心，并发布到 `base_link`。至少采集正常光照、弱光、黑色物体、反光物体、遮挡和桌边六种场景。

### 15.2 推荐流程

1. 验证 RGB/深度尺寸、时间差和内参版本。
2. 深度单位转换并统计无效率。
3. 反投影并通过采样时刻 TF 转换到 `base_link`。
4. 使用机器人工作空间 ROI 裁剪。
5. 体素降采样并保留原点索引映射。
6. 使用方向、高度和面积约束选择桌面，而非只取最大平面。
7. 移除平面后做欧氏聚类。
8. 把结果映射回高分辨率原云，计算几何和颜色统计。
9. 发布结果、可视化标记和诊断信息。

### 15.3 指标

平面高度误差、法线角度误差、物体召回率、过分割率、欠分割率、质心误差、P95 延迟和无效帧比例。每个场景至少重复 20 次。透明物体若超出传感器能力，应作为已知限制并触发降级，而不是把没有深度的区域凭空补成高置信点。

## 第 16 章：综合实验二——配准成功域

从同一场景选择一对有真值的点云，构造平移和旋转初值网格。例如平移每轴从 -0.5 m 到 0.5 m，旋转从 -30 度到 30 度。分别运行单尺度点到点、多尺度点到平面、NDT 或全局特征加 ICP。

每个初值记录是否达到平移和旋转误差门槛，绘制成功率热图。再改变重叠率、体素、噪声和外点比例。这个实验得到的是算法捕获范围，比只用真值初值运行一次更能指导系统设计。

公平比较必须固定输入云和真值，分别调参但公开参数；计时包含降采样、法线、特征和配准全过程；进行预热并报告 P50/P95；失败也计入平均任务成本。

## 第 17 章：阶段考试

建议限时 180 分钟，满分 100 分。

### 一、理论题，共 30 分

1. 比较双目、结构光、ToF 和激光雷达的主要误差与典型失效场景。（6 分）
2. 解释轴向深度与沿射线 range 的区别，并给出转换关系。（6 分）
3. 推导 PCA 法线估计，说明共线邻域为何退化。（6 分）
4. 比较点到点与点到平面 ICP 的目标和可观性。（6 分）
5. 为什么 ICP 返回 converged 不能证明配准正确？（6 分）

### 二、代码题，共 30 分

1. 为 `normalize_depth` 设计六个边界测试。（10 分）
2. 为 `rigid_transform_svd` 设计合成数据验证，包括反射和退化样本。（10 分）
3. 设计点云处理节点的诊断字段和过载策略。（10 分）

### 三、综合题，共 40 分

1. 深度相机安装在移动机械臂末端，彩色点云在快速运动时错位。设计逐层实验区分内参、外参、同步、滚动快门和运动补偿问题。（20 分）
2. 一份报告只写“ICP fitness=0.96，RMSE=0.008，因此精度很好”。指出证据缺口，并设计完整配准评估。（20 分）

## 第 18 章：参考答案

### 一、理论题答案

1. 双目依赖纹理，深度误差随距离平方增长；结构光怕强环境光、反光和透明材料；ToF 有多径、相位绕回和混合像素；激光雷达有稀疏采样、材料反射和扫描运动畸变。不同设备还都受同步、标定和遮挡影响。

2. 轴向深度 $Z$ 是点在相机光轴方向的坐标，range $\rho$ 是光心到点的欧氏距离。对单位射线 $r$，$P=\rho r$ 且 $Z=\rho r_z$；已知归一化非单位方向 $(x,y,1)$ 时，要先归一化再乘 range。

3. 邻域中心化后构造协方差，最小特征值方向是变化最小的局部法线。共线点只有一个主要变化方向，垂直于直线的二维子空间都可作为“最小方向”，法线不唯一。

4. 点到点最小化三维欧氏距离，不依赖法线但局部表面收敛较慢；点到平面只最小化目标法向残差，正确法线下收敛更快。单一平面不能约束沿平面平移和绕法线旋转，Hessian 会出现弱方向。

5. ICP 的收敛判据通常是变换或损失变化很小。错误最近邻也能形成稳定局部极小，重复结构、低重叠、对称和差初值都会让算法收敛到错误姿态。

### 二、代码题答案

`normalize_depth` 测试应覆盖：毫米整数正确转米；米浮点保持不变；零和 `NaN` 被判无效；小于最小距离和大于最大距离被拒绝；全无效图返回空统计而不崩溃；错误维度、非正 scale 或反向范围抛异常。还可验证输入数组未被修改。

SVD 测试先随机生成非共面点，施加已知 $R,t$，验证恢复和重投误差；加入小高斯噪声观察误差连续增长；构造镜像目标，确认行列式修正后 `det(R)=1` 且无法零误差拟合反射；使用共线和重复点应触发退化；加入一个大外点应展示普通最小二乘不鲁棒，为 RANSAC 提供反例。

节点诊断至少包括输入/输出频率、消息年龄、处理 P50/P95/P99、原始与各阶段点数、无效深度比例、TF 查询失败、空云、配准拒绝原因、fitness、RMSE、位姿跳变量和队列积压。过载时使用小队列优先保留最新帧，主动丢弃陈旧数据并计数报警；不能无限排队造成数秒延迟。

### 三、综合题答案

彩色点云排查先做静态多距离标定板实验：若静态边缘随位置系统偏移，检查内参、对齐内参和 RGB-D 外参。随后保持相机静止只让物体运动，再固定物体让机械臂以不同速度运动，画误差对速度和时间差的关系。若误差随速度近似线性且减速后消失，优先检查硬件触发、时间戳语义和 TF 查询时刻。改变运动方向可区分固定外参偏差和时延偏差；快速旋转下残差随图像行变化提示滚动快门。最后用高频关节/IMU 对扫描或两传感器时刻做运动补偿，比较补偿前后独立数据。

`fitness=0.96` 依赖对应距离阈值、源云裁剪和重叠定义，`RMSE=0.008` 只统计内点且单位未说明；没有真值、初值、场景、参数、运行时间和失败率。完整评估应使用带真值的多场景数据，按重叠、噪声、动态比例和对称性分层；对初始平移与旋转做网格扰动；报告平移/旋转误差、成功率、fitness、残差分布、P50/P95 延迟和资源；固定预处理与阈值；保存失败可视化；验证双向和环路一致性。只有任务误差与成功率达到门槛才能宣称可用。

## 本篇完成标准

完成本篇时，你应能明确说出每个深度值的单位、测量方向、坐标系和时间；能从深度图生成经过验证的彩色点云；能依据物理尺度选择体素和邻域；能检测法线与刚体估计退化；能解释 ICP/NDT 的捕获范围和局部极小；能构建包含成功与失败样本的配准基准；能在 ROS2 中正确处理 `PointCloud2`、TF 时间和实时过载。

下一阶段将进入物体检测、语义与实例分割、单目深度、关键点和 6D 位姿估计，并把二维网络输出与本篇的三维几何、不确定性和机器人任务指标连接起来。
