---
title: 第二册第二篇：图像特征、光流与多视图重建
description: 从图像形成、采样和梯度出发，系统讲解角点、描述子、匹配、光流、RANSAC、模型选择与稀疏重建。
---

# 第二册第二篇：图像特征、光流与多视图重建

上一学习阶段建立了相机的几何模型。本篇继续回答一个更具体的问题：当机器人从两帧或多帧图像中寻找同一个空间点时，怎样得到可靠的对应关系，并利用这些对应关系估计相机运动和三维结构？

完整链路不是“调用 ORB，再调用 RANSAC”这么简单。图像的曝光、模糊和采样决定可用信息；检测器决定在哪里测量；描述子和光流决定如何建立候选对应；鲁棒估计决定如何容忍错误匹配；场景结构和相机运动决定几何模型是否可观；最后的非线性优化才把多个局部估计整合为一致地图。本篇沿这条因果链展开。

## 第 1 章：相机记录的不是物体本身

### 1.1 辐照度、反射与曝光

像素亮度同时受光源、物体反射率、表面方向、遮挡、镜头通光量、曝光时间、模拟增益和相机响应曲线影响。同一面白墙在阴影里可能比黑色物体在强光下更暗，因此灰度值不是稳定的材料属性。

常见简化是假设短时间、局部区域内满足亮度恒常：

$$
I(x,y,t)=I(x+\Delta x,y+\Delta y,t+\Delta t).
$$

光流和直接法依赖这个假设。但自动曝光、镜面反射、阴影移动、显示屏闪烁和运动模糊都会破坏它。工程实现应把亮度恒常当作可检验的近似，而不是自然定律。

曝光时间越长，弱光信噪比通常越好，但运动物体会沿曝光时间内的轨迹积分，形成运动模糊。增益提高可以让图像更亮，也会放大读出噪声。机器人快速运动时，清晰边缘往往比“看起来明亮”的画面更重要，应优先控制曝光上限，再补充照明或使用更高感光能力的传感器。

### 1.2 Gamma 与线性强度

相机输出的 8 位图像通常经过非线性响应和 gamma 编码。数值 200 并不表示光子数量是数值 100 的两倍。用于显示、传统特征或训练预训练网络时可以遵循对应接口；用于光度标定、HDR 合成或精确直接法时，应估计响应曲线并在线性辐照度域计算。

混合使用 RGB、BGR、YUV、全范围和有限范围视频也是常见静默错误。数据接口应保存颜色空间、位深、通道顺序和归一化规则。随机抽一张图“肉眼看着差不多”不能证明数值管线一致。

### 1.3 噪声不是固定高斯白噪声

光子到达近似服从泊松统计，光子噪声随信号强度变化；电子读出还引入近似与信号无关的噪声。暗场、热像素、压缩和去噪算法会产生更多结构化误差。实际建模可先使用

$$
\operatorname{Var}(I)\approx aI+b
$$

的异方差模型，再用不同曝光的静态场景估计 $a,b$。若算法在仿真中只注入固定标准差高斯噪声，可能无法覆盖真实弱光表现。

## 第 2 章：采样、混叠与图像金字塔

### 2.1 为什么缩小图像前必须低通

连续图像经过像素阵列采样成为离散网格。若信号包含超过新采样频率一半的高频成分，降采样后会发生混叠：细条纹变成错误的粗条纹，轮廓产生锯齿，远处栅栏随相机移动出现闪烁。

把图像宽高直接每隔两个像素取一个点不是可靠降采样。应先用低通滤波抑制无法被低分辨率表达的频率，再采样。OpenCV 的 `pyrDown` 会做高斯平滑和二倍降采样；深度学习框架中的 resize 是否抗混叠取决于算子和参数，必须查明并通过合成条纹测试。

### 2.2 卷积与边界条件

二维离散卷积写为

$$
(I*k)[u,v]=\sum_i\sum_j I[u-i,v-j]k[i,j].
$$

许多深度学习库实际实现互相关，即不翻转卷积核。对于学习出来的卷积核差异不重要，因为权重会适应；对于手写 Sobel 核，翻转可能改变梯度符号。

图像边缘缺少邻域，需要选择零填充、复制、反射或只输出有效区域。零填充会在边缘制造人造黑色轮廓；反射通常适合自然图像平滑，但也不是物理观测。测试滤波器时必须包含边界像素。

### 2.3 高斯滤波与尺度空间

二维各向同性高斯函数为

$$
G(x,y;\sigma)=\frac{1}{2\pi\sigma^2}
\exp\left(-\frac{x^2+y^2}{2\sigma^2}\right).
$$

$\sigma$ 决定平滑尺度。小尺度保留细节但对噪声敏感，大尺度抑制噪声也会消除小结构。尺度空间通过不同 $\sigma$ 描述同一图像，使特征能够在不同观察距离下被检测。

高斯核可分离为两个一维核，先横向再纵向，计算量从每像素 $O(k^2)$ 降为 $O(2k)$。在 GPU 上实际性能还取决于内存访问和库实现，不能只按算术次数推断。

### 2.4 构建可检查的金字塔

```python
import cv2
import numpy as np

def build_gaussian_pyramid(image: np.ndarray, levels: int) -> list[np.ndarray]:
    if image.ndim != 2:
        raise ValueError("expected a grayscale image")
    if image.dtype != np.uint8:
        raise ValueError("expected uint8 input")
    if levels < 1:
        raise ValueError("levels must be positive")
    pyramid = [image]
    for _ in range(1, levels):
        previous = pyramid[-1]
        if min(previous.shape) < 4:
            raise ValueError("too many levels for image size")
        pyramid.append(cv2.pyrDown(previous))
    return pyramid
```

测试应确认每层尺寸、输入不被原地修改、常量图像仍近似常量，以及高频棋盘在下采样前后没有产生明显伪低频结构。尺寸为奇数时，下一层的取整行为也要写入契约。

## 第 3 章：梯度、边缘与局部结构

### 3.1 图像梯度

连续图像的一阶变化由

$$
\nabla I=\begin{bmatrix}I_x&I_y\end{bmatrix}^T
$$

描述。离散图像常用 Sobel 核同时完成差分和轻微平滑。梯度幅值 $\sqrt{I_x^2+I_y^2}$ 表示变化强度，方向 $\operatorname{atan2}(I_y,I_x)$ 表示最陡上升方向。

直接在 `uint8` 上相减会发生截断或回绕。计算梯度时应输出浮点或有符号类型：

```python
def image_gradients(gray: np.ndarray):
    if gray.ndim != 2:
        raise ValueError("gray image required")
    gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    magnitude = cv2.magnitude(gx, gy)
    orientation = cv2.phase(gx, gy, angleInDegrees=False)
    return gx, gy, magnitude, orientation
```

### 3.2 Canny 边缘不是简单阈值

Canny 流程包括平滑、梯度、非极大值抑制和双阈值滞后连接。非极大值抑制让宽梯度带变成细边缘；双阈值让强边缘带动相连弱边缘保留，同时抑制孤立噪声。

阈值不能跨相机和曝光条件盲目复用。可以根据灰度中位数自适应初始化，但最终要依据任务验证。抓取透明物体时边缘可能比纹理可靠；在树叶和栅栏场景中过多边缘反而增加匹配歧义。

### 3.3 结构张量

考虑小窗口发生位移 $(\Delta x,\Delta y)$ 后的平方差：

$$
E(\Delta x,\Delta y)=\sum_{(x,y)\in W}
[I(x+\Delta x,y+\Delta y)-I(x,y)]^2.
$$

一阶展开后

$$
E\approx
\begin{bmatrix}\Delta x&\Delta y\end{bmatrix}
M
\begin{bmatrix}\Delta x\\\Delta y\end{bmatrix},
$$

其中结构张量

$$
M=\sum_W w(x,y)
\begin{bmatrix}
I_x^2&I_xI_y\\
I_xI_y&I_y^2
\end{bmatrix}.
$$

若两个特征值都小，窗口近似平坦；一个大一个小，说明是边缘；两个都大，说明各方向移动都会显著改变外观，是适合定位的角点。这个分析同时解释了角点检测和光流的可观性。

## 第 4 章：角点与关键点检测器

### 4.1 Harris 角点

Harris 响应为

$$
R=\det(M)-k\operatorname{trace}(M)^2.
$$

角点处 $R$ 较大，边缘处通常为负，平坦区接近零。阈值后还要做非极大值抑制，否则同一角点附近会输出一团响应。

Harris 对图像旋转有一定稳定性，但不天然具有尺度不变性。图像缩放后，固定窗口覆盖的物理区域改变。可在尺度空间检测，或使用带尺度选择的算法。

### 4.2 FAST 与 ORB

FAST 比较候选像素周围圆环上的亮暗关系，通过连续像素判定角点，速度很快。它本身不提供方向和描述子。ORB 使用 FAST 关键点、方向估计和旋转后的 BRIEF 二进制描述子，并通过多层金字塔提高尺度适应性。

ORB 描述子用 Hamming 距离匹配，不能用 L2；SIFT 这类浮点描述子通常用 L2。距离类型错了有时仍能运行，却会破坏匹配排序。

### 4.3 SIFT 的尺度与方向

SIFT 在高斯差分尺度空间寻找极值，为关键点分配主方向，再统计局部梯度方向直方图形成描述子。它对旋转和一定尺度变化稳定，通常比 ORB 更鲁棒，但计算和存储成本更高。

“SIFT 一定优于 ORB”也不成立。移动机器人实时定位可能更看重延迟和足够多的稳定点；低功耗平台适合 ORB。大视角变化、离线重建或纹理复杂场景可能更适合 SIFT。应在目标数据上比较有效内点数、空间覆盖、运动估计成功率和耗时，而不只是原始匹配数量。

### 4.4 网格化特征分配

检测器常把大量点集中在海报、树叶或高纹理角落，导致几何估计受局部区域支配。可把图像划成网格，每格保留响应最强的若干点，确保全局覆盖。特征总数相同的情况下，均匀覆盖往往比局部密集更利于估计相机运动。

```python
def select_by_grid(keypoints, width, height, rows=6, cols=8, per_cell=20):
    cells = [[[] for _ in range(cols)] for _ in range(rows)]
    for kp in keypoints:
        col = min(cols - 1, int(kp.pt[0] * cols / width))
        row = min(rows - 1, int(kp.pt[1] * rows / height))
        cells[row][col].append(kp)
    selected = []
    for row in cells:
        for cell in row:
            cell.sort(key=lambda kp: kp.response, reverse=True)
            selected.extend(cell[:per_cell])
    return selected
```

## 第 5 章：描述子匹配不是最终对应关系

### 5.1 最近邻与比率测试

对每个查询描述子寻找最近和次近邻。Lowe 比率

$$
\frac{d_1}{d_2}<\tau
$$

衡量最佳匹配是否明显优于第二候选。若两者距离接近，局部纹理可能有歧义。阈值越小越保守，精度提高但召回下降。

```python
def match_sift(desc1, desc2, ratio=0.75):
    if desc1 is None or desc2 is None:
        return []
    if len(desc1) < 2 or len(desc2) < 2:
        return []
    matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
    pairs = matcher.knnMatch(desc1, desc2, k=2)
    return [best for best, second in pairs
            if best.distance < ratio * second.distance]
```

交叉验证要求 A 的最佳匹配是 B，且 B 的最佳匹配也是 A。它与比率测试解决的问题不同，可以组合使用，但会降低召回。运动连续的视频还可以加入搜索半径、极线和预测位姿约束。

### 5.2 描述子距离不能跨图直接解释置信度

距离 30 是否可靠取决于描述子类型、光照、图像纹理和候选集合。同一绝对距离在重复纹理中可能很危险，在唯一纹理中可能可靠。更好的置信度来自距离比、双向一致性、几何残差、轨迹长度和空间覆盖的组合，并在标注数据上做校准。

### 5.3 匹配可视化的必要信息

可视化不应只画前 50 个“最好看的”匹配。应分别保存原始候选、比率测试后、几何验证后和最终内点；同时标出内点覆盖网格、残差和被拒绝原因。这样才能区分描述子失败、几何模型不适用和阈值错误。

## 第 6 章：Lucas-Kanade 光流

### 6.1 光流约束方程

由亮度恒常对时间一阶展开：

$$
I_xu+I_yv+I_t=0,
$$

其中 $(u,v)$ 是像素速度。一个像素只有一个方程、两个未知量，无法唯一求解，这就是孔径问题。Lucas-Kanade 假设小窗口内所有像素共享同一运动，得到超定方程：

$$
A
\begin{bmatrix}u\\v\end{bmatrix}=b,
$$

$$
A=
\begin{bmatrix}
I_x(q_1)&I_y(q_1)\\
\vdots&\vdots\\
I_x(q_n)&I_y(q_n)
\end{bmatrix},
\qquad
b=-\begin{bmatrix}I_t(q_1)&\cdots&I_t(q_n)\end{bmatrix}^T.
$$

正规方程中的 $A^TA$ 正是结构张量。边缘窗口的一个特征值很小，沿边缘方向运动不可观；角点两个特征值都大，流更稳定。

### 6.2 金字塔解决大位移

一阶线性化只适合小位移。金字塔 LK 先在低分辨率估计大致运动，再逐层放大并细化。层数越多可处理更大位移，但小物体和细节可能在高层消失。窗口太小缺少纹理，太大则违反窗口内运动一致假设。

### 6.3 前后向一致性检查

先从帧 1 跟踪到帧 2 得到 $p_2$，再从帧 2 反向跟踪到帧 1 得到 $p'_1$。若

$$
\|p'_1-p_1\|<\tau
$$

则轨迹较可信。这能过滤遮挡、出界和局部优化错误，但在重复纹理中前后两次可能一致地跟错，因此仍需几何验证。

```python
def track_forward_backward(previous, current, points, threshold=1.0):
    if points is None or len(points) == 0:
        return np.empty((0, 2), np.float32), np.empty(0, dtype=bool)
    source = np.asarray(points, np.float32).reshape(-1, 1, 2)
    params = dict(
        winSize=(21, 21), maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
    )
    target, status_forward, _ = cv2.calcOpticalFlowPyrLK(
        previous, current, source, None, **params
    )
    restored, status_backward, _ = cv2.calcOpticalFlowPyrLK(
        current, previous, target, None, **params
    )
    error = np.linalg.norm(restored - source, axis=2).reshape(-1)
    height, width = current.shape[:2]
    x, y = target.reshape(-1, 2).T
    inside = (x >= 0) & (x < width) & (y >= 0) & (y < height)
    valid = (
        status_forward.reshape(-1).astype(bool)
        & status_backward.reshape(-1).astype(bool)
        & np.isfinite(error) & (error < threshold) & inside
    )
    return target.reshape(-1, 2), valid
```

## 第 7 章：RANSAC 的概率与实现

### 7.1 为什么最小二乘会被外点破坏

最小二乘对大残差平方惩罚，单个错误匹配可能比许多正确匹配贡献更大。RANSAC 重复抽取最小样本拟合模型，用阈值判断全部数据中的一致集，再选择支持度高的模型。

若内点率为 $w$，最小样本数为 $s$，一次抽样全为内点的概率是 $w^s$。为了以概率 $p$ 至少成功一次，迭代次数满足

$$
N\geq\frac{\log(1-p)}{\log(1-w^s)}.
$$

例如 $w=0.5$、五点法 $s=5$、$p=0.99$ 时约需 145 次；若内点率降到 0.2，则需要约 14,389 次。先用描述子和运动约束提高候选质量，会直接降低鲁棒估计成本。

### 7.2 一个可读的直线 RANSAC

```python
from dataclasses import dataclass

@dataclass
class LineModel:
    normal: np.ndarray
    offset: float

def line_from_two_points(a, b, eps=1e-12):
    direction = b - a
    norm = np.linalg.norm(direction)
    if norm < eps:
        return None
    normal = np.array([direction[1], -direction[0]]) / norm
    return LineModel(normal, -float(normal @ a))

def ransac_line(points, threshold, iterations=1000, seed=0):
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2:
        raise ValueError("points must have shape [N,2], N >= 2")
    if threshold <= 0 or iterations <= 0:
        raise ValueError("threshold and iterations must be positive")
    rng = np.random.default_rng(seed)
    best_model, best_mask, best_score = None, None, (-1, np.inf)
    for _ in range(iterations):
        indices = rng.choice(len(points), size=2, replace=False)
        model = line_from_two_points(points[indices[0]], points[indices[1]])
        if model is None:
            continue
        residuals = np.abs(points @ model.normal + model.offset)
        mask = residuals < threshold
        count = int(mask.sum())
        mean_residual = float(residuals[mask].mean()) if count else np.inf
        score = (count, -mean_residual)
        if score > best_score:
            best_model, best_mask, best_score = model, mask, score
    if best_model is None:
        raise RuntimeError("no valid model")
    return best_model, best_mask
```

真实实现还应在最佳内点上重新拟合模型、动态更新迭代次数、拒绝退化最小样本，并使用适合几何模型的残差。随机种子便于复现实验，但正式评估应测试多个 seed。

### 7.3 阈值必须连接测量噪声

阈值太小会把有噪声的真匹配拒绝，太大会把错误模型周围的外点接纳。像素阈值应考虑关键点定位精度、图像缩放和运动模糊。使用归一化坐标时，要把像素阈值除以焦距尺度。不同金字塔层的关键点也可能需要尺度相关阈值。

内点数最多不一定是最佳模型。若一个模型以宽阈值获得大量高残差内点，另一个模型以窄阈值获得略少但一致的内点，应结合截断残差、模型复杂度和下游任务选择。MLESAC、MSAC 等方法就是对简单计数评分的改进。

## 第 8 章：单应矩阵、基础矩阵还是本质矩阵

### 8.1 单应矩阵的适用范围

当场景点位于同一平面，或相机只发生纯旋转时，两幅图像满足

$$
s p_2=Hp_1.
$$

单应矩阵有 8 个自由度，至少需要 4 对非共线点。文档扫描、平面标记、图像拼接和桌面目标定位经常使用它。若把非平面场景强行拟合为单应，远近点会产生不同残差。

### 8.2 两视图模型竞争

视觉里程计不能简单规定“总是估计本质矩阵”。纯旋转或远景时，本质矩阵平移不可观，单应可能更稳定；一般三维场景有平移和足够视差时，本质矩阵更合适；近似平面场景两者可能都得到高内点率。

可以同时估计 $H$ 与 $E/F$，比较经过相同噪声尺度归一化的评分、空间覆盖、视差和三角化质量。模型复杂度不同，不能只比较原始内点数量。最终系统还需要一种“当前不可初始化”的状态，而不是强迫每对图像输出三维运动。

### 8.3 一个反例

相机正对一面有丰富纹理的墙缓慢横移。几乎所有点共面，单应矩阵能很好解释观测，本质矩阵也可能返回一个看似合理的姿态。如果只看本质矩阵内点率，系统会误以为获得了可靠三维点；实际上墙外结构没有约束，深度对噪声很敏感。反过来，普通三维场景在相机纯旋转时，单应占优并不说明场景是平面。

因此模型判断必须同时考虑运动和场景，不能从单个矩阵的成功返回值推出唯一解释。

## 第 9 章：两视图估计完整管线

```python
from dataclasses import dataclass

@dataclass
class TwoViewResult:
    success: bool
    reason: str
    rotation: np.ndarray | None = None
    translation_direction: np.ndarray | None = None
    inlier_mask: np.ndarray | None = None
    median_parallax_deg: float | None = None

def median_parallax(points1, points2, R):
    rays1 = np.column_stack((points1, np.ones(len(points1))))
    rays2 = np.column_stack((points2, np.ones(len(points2))))
    rays1 /= np.linalg.norm(rays1, axis=1, keepdims=True)
    rays2 /= np.linalg.norm(rays2, axis=1, keepdims=True)
    rays2_in_1 = (R.T @ rays2.T).T
    cosine = np.sum(rays1 * rays2_in_1, axis=1)
    angles = np.arccos(np.clip(cosine, -1.0, 1.0))
    return float(np.degrees(np.median(angles)))

def validate_two_view(points1_px, points2_px, K, dist):
    p1 = np.asarray(points1_px, np.float64).reshape(-1, 1, 2)
    p2 = np.asarray(points2_px, np.float64).reshape(-1, 1, 2)
    if len(p1) < 30:
        return TwoViewResult(False, "too few candidate correspondences")
    n1 = cv2.undistortPoints(p1, K, dist).reshape(-1, 2)
    n2 = cv2.undistortPoints(p2, K, dist).reshape(-1, 2)
    threshold = 1.5 / ((K[0, 0] + K[1, 1]) * 0.5)
    E, mask = cv2.findEssentialMat(
        n1, n2, np.eye(3), cv2.RANSAC, 0.999, threshold
    )
    if E is None or mask is None:
        return TwoViewResult(False, "essential matrix estimation failed")
    _, R, t, pose_mask = cv2.recoverPose(E, n1, n2, np.eye(3), mask=mask)
    inliers = pose_mask.reshape(-1).astype(bool)
    if inliers.sum() < 25:
        return TwoViewResult(False, "too few geometrically valid inliers")
    parallax = median_parallax(n1[inliers], n2[inliers], R)
    if parallax < 1.0:
        return TwoViewResult(False, "insufficient parallax", median_parallax_deg=parallax)
    grid_x = np.unique((p1.reshape(-1, 2)[inliers, 0] / 80).astype(int)).size
    grid_y = np.unique((p1.reshape(-1, 2)[inliers, 1] / 80).astype(int)).size
    if grid_x < 3 or grid_y < 3:
        return TwoViewResult(False, "inliers are spatially concentrated")
    return TwoViewResult(
        True, "ok", R, t.reshape(3), inliers, parallax
    )
```

这段代码仍是教学骨架：真实系统还要处理 `findEssentialMat` 可能返回多个候选矩阵、比较单应模型、检查重投影误差和三角化深度，并根据时间连续性验证运动。它的重点是展示“失败是合法输出”。

## 第 10 章：从两视图到多视图重建

### 10.1 增量式 SfM

典型增量式结构恢复流程如下：

1. 提取各图像特征并建立候选匹配。
2. 选择视差充足、内点覆盖良好的初始图像对。
3. 从本质矩阵恢复相对位姿并三角化初始地图点。
4. 选择与已有地图有足够 2D-3D 对应的新图像。
5. 使用 PnP + RANSAC 估计新相机位姿。
6. 在新相机和已有相机之间三角化新点。
7. 删除重投影误差大、视角过小或观测不一致的点。
8. 周期性执行局部或全局束调整。

初始化对不能只选匹配最多的一对。相邻帧匹配很多但基线很小，三角化深度不稳定；间隔很远的帧视差大但匹配可能太少。合适初始对需要在重叠和视差之间平衡。

### 10.2 观测轨迹与并查集陷阱

多幅图像中的两两匹配需要合并成同一三维点的观测轨迹。例如图 A 的点 3 匹配图 B 的点 8，图 B 的点 8 匹配图 C 的点 5，则三者可能属于同一轨迹。但若图 A 的两个不同点通过错误匹配合并到同一轨迹，就产生冲突：同一图像中的两个特征不能观察同一个普通点。

构建轨迹时应在合并前检测同图冲突，并优先使用几何验证后的边。简单无条件并查集合并会让少量错误匹配污染大量视图。

### 10.3 PnP 接入新相机

新图像只要与已有三维地图建立足够 2D-3D 对应，就可以通过 PnP 定位。对应点应在图像和空间中分布良好。三维点接近平面、图像点集中一角或错误对应过多都会使姿态不稳定。

PnP 后必须用内点重新优化，并检查位姿相对上一帧是否合理。运动先验不能替代几何证据，但能拒绝明显跳变。若内点不足，系统应保留图像等待后续匹配，不能用低质量位姿继续三角化污染地图。

## 第 11 章：束调整

### 11.1 优化目标

束调整同时优化相机参数 $\theta_i$ 和三维点 $P_j$，最小化所有可见观测的重投影误差：

$$
\min_{\{\theta_i\},\{P_j\}}
\sum_{(i,j)\in\mathcal O}
\rho\left(
\left\|p_{ij}-\pi(\theta_i,P_j)\right\|_{\Sigma_{ij}}^2
\right).
$$

$\rho$ 是 Huber、Cauchy 等鲁棒核，$\Sigma_{ij}$ 描述测量不确定性。并非所有角点定位精度相同：不同金字塔层、运动模糊和边缘方向会产生不同噪声。

### 11.2 规范自由度

纯视觉重建的整体坐标系可以任意旋转和平移，单目还可以整体缩放而不改变重投影。若不固定规范，自由度会使 Hessian 奇异。常见做法是固定第一相机位姿，并固定第二相机平移尺度或引入已知尺度约束。

### 11.3 稀疏结构为什么重要

每个观测只连接一个相机和一个三维点，因此雅可比非常稀疏。利用 Schur 补先消去点变量，可以显著降低求解规模。实际项目应使用 Ceres、g2o、GTSAM 等成熟优化库，而不是为生产系统手写通用稀疏求解器；学习阶段可以对小问题用 SciPy 验证残差和雅可比。

### 11.4 鲁棒核不是数据清洗替代品

鲁棒核降低大残差的影响，但大量结构化错误仍会把优化拉向错误解。优化前应通过描述子、几何和轨迹一致性过滤；优化后再根据标准化残差剔除异常观测并重新优化。一次优化“成功收敛”只表示数值求解停止，不表示几何正确。

## 第 12 章：动态场景、重复纹理与其他反例

### 12.1 动态物体占据多数画面

若相机静止而一辆大车经过，RANSAC 可能把车上的一致运动当成相机运动，因为它贡献最多内点。最大一致集不等于背景。可以结合语义掩码、深度、长期静态地图和运动聚类，识别多个运动模型。

### 12.2 重复纹理

仓库货架、楼窗、地砖和栅栏会产生大量相似描述子。比率测试可能全部失败，也可能在周期结构中稳定地匹配到错误位置。时间连续性、预测搜索窗、立体顺序约束和多帧轨迹比单帧描述子更有效。

### 12.3 低纹理与强反光

白墙缺少梯度，局部位置不可观；玻璃和金属的外观随视角改变，亮度恒常不成立。可以增加主动纹理、选择轮廓或线特征、融合 IMU/激光，或明确将这些区域标记为低置信度。

### 12.4 滚动快门

滚动快门逐行曝光，快速运动时同一图像的不同扫描行对应不同相机姿态。全局快门模型不能用单个位姿解释所有像素，表现为直线倾斜和几何残差随图像行变化。提高读出速度、缩短曝光、使用全局快门，或采用滚动快门运动模型才能从根本上处理。

## 第 13 章：综合实验一——特征管线基准

### 13.1 数据集设计

自行采集六组短序列：正常室内、弱光、快速转动、前向低视差、重复货架、含大面积动态物体。固定原始图像，比较 ORB、SIFT 和稀疏 LK。每组至少选取不同帧间隔，观察位移增大带来的变化。

### 13.2 指标

每种方法记录：检测点数、网格覆盖率、候选匹配数、几何内点率、有效内点数、单应与本质模型评分、视差中位数、旋转误差、平移方向误差、每阶段耗时和失败原因。

若没有高精度真值，可在固定标定板、转台角度或仿真中建立部分真值，也可检查闭环一致性和重投影误差。但没有真值时不能把代理指标写成绝对精度结论。

### 13.3 消融实验

至少完成以下消融：关闭网格分配；改变比率阈值；关闭前后向光流检查；改变 RANSAC 阈值；只比较内点数而不检查覆盖；禁用图像金字塔。每次只改变一个因素，保存配置和随机种子。

预期不是某个方法永远最好，而是得到适用边界。例如 ORB 在正常室内达到更高帧率，SIFT 在大尺度变化中保留更多有效内点，LK 在小帧间运动中轨迹最连续但快速转动时失败。报告应说明证据和反例。

## 第 14 章：综合实验二——小型稀疏重建器

### 14.1 最低功能

实现一个 20 至 100 张图像的小型重建器：读取标定参数；提取并匹配特征；选择初始化对；估计相对位姿；三角化；通过 PnP 添加新视图；局部束调整；导出相机轨迹和 PLY 点云。

工程目录建议：

```text
sparse_reconstruction/
  configs/
  data/
  src/
    camera.py
    features.py
    matching.py
    tracks.py
    two_view.py
    triangulation.py
    pnp.py
    bundle_adjustment.py
    export.py
  tests/
  outputs/
```

### 14.2 单元测试

用合成相机和三维点测试几何模块。已知 $K,R,t,P$ 后生成无噪声像素，检查本质约束、三角化和 PnP 是否恢复真值；再逐步加入高斯噪声和外点，画误差曲线。合成测试能隔离算法错误，但不能替代真实图像，因为它通常不包含错误匹配、模糊和曝光变化。

### 14.3 验收标准

重建器应能拒绝纯旋转初始化；低视差时输出明确原因；相机轨迹没有突然翻转；绝大多数地图点在观测相机前方；重投影误差有统计报告；输出结果可由固定命令复现；失败数据仍被保留并可视化。

## 第 15 章：阶段考试

建议限时 180 分钟，满分 100 分。

### 一、理论题，共 30 分

1. 为什么图像降采样前需要低通滤波？给出一个机器人场景中的混叠例子。（5 分）
2. 用结构张量特征值解释平坦区、边缘和角点对二维运动估计的可观性。（5 分）
3. 推导 LK 光流约束，并说明它依赖的三个主要假设。（5 分）
4. 推导 RANSAC 迭代次数公式，计算内点率 0.4、样本数 5、成功率 0.99 时的次数。（5 分）
5. 单应矩阵内点率高能否证明场景是平面？给出反例。（5 分）
6. 为什么单目束调整需要固定规范自由度？（5 分）

### 二、代码题，共 30 分

1. 为前后向 LK 实现设计至少六个边界测试。（10 分）
2. 某程序对 ORB 描述子使用 `NORM_L2`，随后只保留距离最小的 100 个匹配。分析风险并修正。（10 分）
3. 写出两视图初始化的拒绝条件，至少覆盖点数、覆盖、视差、正深度和模型竞争。（10 分）

### 三、综合题，共 40 分

1. 仓库机器人在重复货架区域定位跳变，但 RANSAC 内点率达到 85%。设计逐层诊断实验，说明为什么高内点率不能排除错误。（20 分）
2. 设计一份 ORB、SIFT、LK 的公平基准，必须说明数据划分、参数预算、指标、计时和失败统计。（20 分）

## 第 16 章：参考答案

### 一、理论题

1. 新采样网格无法表示超过奈奎斯特频率的成分，它们会折叠为错误低频。远处栅栏、细密货架或地砖在缩小图像后出现假条纹，可能制造虚假角点并导致跟踪抖动。

2. 两个特征值都小时，各方向移动都几乎不改变窗口，运动不可观；一个大一个小时，只能确定垂直边缘方向的运动，沿边缘方向存在孔径问题；两个都大时，两个方向都受约束，角点适合二维定位。

3. 对亮度恒常做一阶泰勒展开得到 $I_xu+I_yv+I_t=0$。主要假设是短时间亮度近似不变、位移足够小以适用线性化、局部窗口内运动一致。金字塔放宽小位移限制，但不会修复遮挡和非朗伯反射。

4. 一次全内点概率为 $w^s$，连续 $N$ 次都失败概率为 $(1-w^s)^N$。要求其不超过 $1-p$，得 $N\ge\log(1-p)/\log(1-w^s)$。代入 $w=0.4,s=5,p=0.99$ 得约 448 次，应向上取整，并根据实际内点率动态更新。

5. 不能。相机纯旋转时，任意深度场景也可由旋转诱导的单应解释；远景在平移相对深度很小时也近似满足单应。反过来，场景大部分为平面不代表所有点都能用同一平面重建。

6. 整体旋转、平移不会改变投影，单目系统整体缩放也不改变投影，目标函数存在等价解和奇异方向。固定第一相机位姿并固定尺度，或加入外部先验，才能得到数值确定的解。

### 二、代码题

前后向 LK 的测试至少包括：空点集返回空结果；静态相同图像得到近零位移；已知平移合成图恢复正确方向；出界点被拒绝；遮挡点前后误差变大并被拒绝；常量低纹理图不产生可信轨迹；错误图像类型或尺寸被拒绝；含 `NaN` 坐标被拒绝。每项需写具体断言。

ORB 是二进制描述子，应使用 Hamming 距离。只取绝对距离最小的 100 个不能处理重复纹理，也可能使点集中在一个区域。正确流程可用 KNN Hamming 匹配、比率测试、双向一致性、网格覆盖和几何验证，最终依据几何内点及残差，而不是固定截断原始匹配。

两视图初始化至少要求：候选和几何内点数量达到门槛；内点覆盖多个图像网格；中位视差或射线夹角充足；三角化后两视图正深度比例高；重投影误差受控；$H$ 与 $E$ 的竞争没有显示平面或纯旋转退化；恢复运动与时间先验不冲突。任一关键条件失败都应延迟初始化。

### 三、综合题

仓库案例中，重复货架可能让大量点以同一个周期偏移，错误对应之间仍满足一个一致的几何模型，所以 RANSAC 会给出高内点率。诊断顺序应保存原始、描述子过滤后和几何内点图；检查内点是否集中在同一排货架；查看匹配位移是否呈周期峰；比较预测运动和恢复运动；用更短帧间隔限制搜索窗；加入唯一标志区域或深度；检查多帧轨迹闭环一致性。还应在已知轨迹或人工标注子集上统计真实正确率，而不是把 RANSAC 标签当真值。

公平基准应使用完全相同的原始序列和相机标定，按场景划分调参与测试数据；为三种方法分别调参但限制同等计算预算或同时报告不同预算的精度速度曲线；计时包含检测、描述、匹配或跟踪、几何验证，并进行预热和多次重复；指标包含有效内点、覆盖、姿态误差、成功率、P50/P95 延迟和内存；按弱光、模糊、尺度变化、重复纹理、动态物体切片。LK 需要共同初始点，其重检测成本也必须计入，不能只计跟踪函数。

## 本篇完成标准

完成本篇不是“看懂了 ORB 和 RANSAC”，而是能够从原始图像建立经过验证的几何对应：解释曝光与采样如何影响特征；根据任务选择角点、描述子或光流；用结构张量判断可观性；根据噪声设置鲁棒阈值；识别平面、纯旋转、低视差和动态主体退化；实现一个能拒绝坏初始化的小型稀疏重建器；用分场景指标而不是单一内点率评价系统。

下一阶段将从二维对应进入真实三维数据处理，系统学习深度传感原理、深度图误差、点云组织、邻域搜索、法线、体素、RANSAC 几何分割、ICP/NDT 配准和不确定性传播。
