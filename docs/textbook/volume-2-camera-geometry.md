---
title: 第二册第一篇：相机模型、标定与多视图几何
description: 从坐标约定和针孔投影开始，完整推导镜头畸变、相机标定、对极几何、三角化，并给出可运行实验与答案。
---

# 第二册第一篇：相机模型、标定与多视图几何

视觉系统首先是一个测量系统，然后才是一个识别系统。相机把光线转换成像素，算法再从像素反推空间结构。若不了解成像模型、参数可观性和误差来源，即使检测网络精度很高，也可能把错误的三维位置交给机器人。本篇目标不是背诵 OpenCV 接口，而是能够回答四个工程问题：一个像素是怎样产生的；哪些三维信息在投影中丢失了；如何用实验估计相机参数；估计结果是否足以支持后续定位任务。

完成本篇后，你应当能够独立实现投影和反投影，完成单目与双目标定，解释本质矩阵和基础矩阵，使用三角化恢复空间点，并通过重投影、独立测量和失败样例判断结果是否可信。

## 第 1 章：建立不含糊的坐标约定

### 1.1 点、坐标和坐标系不是同一概念

空间中的几何点不会因为换坐标系而移动，但描述它的三个数会变化。记世界中的同一个点为 $P$，它在世界坐标系中的坐标为 ${}^w p$，在相机坐标系中的坐标为 ${}^c p$。若 ${}^cT_w$ 表示把世界坐标转换到相机坐标的变换，则

$$
{}^c\tilde p = {}^cT_w {}^w\tilde p,
\qquad
{}^cT_w =
\begin{bmatrix}
{}^cR_w & {}^ct_w\\
0 & 1
\end{bmatrix}.
$$

左上标说明“结果在哪个坐标系中表达”，右下标说明“输入来自哪个坐标系”。采用这种记号以后，连乘顺序可以像单位一样检查：

$$
{}^cT_b {}^bT_w = {}^cT_w.
$$

中间的 $b$ 相消。若代码中写成相反顺序，矩阵维度可能仍然合法，但物理含义已经错误。

### 1.2 常见坐标轴约定

OpenCV 常用的光学坐标系是 $x$ 向图像右侧、$y$ 向图像下方、$z$ 指向镜头前方。ROS REP-103 的机体坐标通常是 $x$ 向前、$y$ 向左、$z$ 向上。ROS 中的 optical frame 仍采用 $z$ 向前、$x$ 向右、$y$ 向下，因此 `camera_link` 和 `camera_optical_frame` 不应被当成同一个坐标系。

建议在每个项目 README 中保存一张坐标系表：

| 坐标系 | 原点 | x 轴 | y 轴 | z 轴 | 单位 |
| --- | --- | --- | --- | --- | --- |
| `base_link` | 机器人底盘中心 | 前 | 左 | 上 | m |
| `camera_link` | 相机安装基准 | 前 | 左 | 上 | m |
| `camera_optical_frame` | 光心 | 右 | 下 | 前 | m |
| 图像像素 | 左上角附近 | 右 | 下 | 无 | px |

像素坐标本身还需要约定原点位于像素中心还是像素角。多数视觉公式把整数坐标视为像素中心，某些图形学接口则使用半像素偏移。单个任务内只要一致就能工作，但跨库转换时必须通过已知点实验确认。

### 1.3 主动变换与被动变换

主动变换描述点在固定坐标系中真的旋转或移动；被动变换描述同一个点改用另一坐标系表达。两者可能使用转置关系，因此“旋转矩阵看起来方向反了”经常源于语义混淆。

工程上不要只命名 `R`、`T`、`pose`。优先使用 `T_camera_world`、`point_in_camera` 这类带方向的名称，并在接口注释中写明：输入坐标系、输出坐标系、长度单位、旋转参数化和时间戳。

## 第 2 章：针孔相机模型的完整推导

### 2.1 从相似三角形到归一化平面

设相机坐标点为 $P_c=(X,Y,Z)^T$，且 $Z>0$。在焦距为 $f$ 的理想针孔模型中，投影点满足相似三角形关系：

$$
x'=f\frac{X}{Z},\qquad y'=f\frac{Y}{Z}.
$$

先除以深度得到归一化坐标

$$
x=\frac{X}{Z},\qquad y=\frac{Y}{Z}.
$$

归一化平面可看成位于 $Z=1$ 的虚拟成像平面。它把三维射线压缩成二维点，因此 $(X,Y,Z)$ 与 $(\lambda X,\lambda Y,\lambda Z)$ 对任意正数 $\lambda$ 都有相同的归一化坐标。单目投影丢失绝对尺度不是算法不够强，而是观测模型本身存在尺度不确定性。

### 2.2 从物理尺寸到像素坐标

传感器上的物理距离还要转换成像素。若水平方向每米对应 $s_x$ 个像素，垂直方向每米对应 $s_y$ 个像素，则 $f_x=s_xf$、$f_y=s_yf$。再加上主点 $(c_x,c_y)$：

$$
u=f_xx+c_x,\qquad v=f_yy+c_y.
$$

相机内参矩阵写为

$$
K=
\begin{bmatrix}
f_x & s & c_x\\
0 & f_y & c_y\\
0 & 0 & 1
\end{bmatrix}.
$$

$s$ 是像素轴不正交产生的 skew。现代相机通常令它为零。主点接近图像中心，但不应强制等于中心；镜头装配、裁剪和缩放都会改变它。

齐次形式为

$$
\lambda
\begin{bmatrix}u\\v\\1\end{bmatrix}
=K
\begin{bmatrix}X\\Y\\Z\end{bmatrix},
$$

其中 $\lambda=Z$。若三维点先由世界坐标变换到相机坐标，则

$$
\lambda\tilde p=K[R\mid t]\tilde P_w.
$$

矩阵 $P=K[R\mid t]$ 称为投影矩阵。它只有 3 行 4 列，不能被当作刚体变换使用。

### 2.3 投影的可运行实现

```python
import numpy as np

def project_points(points_camera: np.ndarray, K: np.ndarray) -> np.ndarray:
    points = np.asarray(points_camera, dtype=np.float64)
    K = np.asarray(K, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"points must have shape [N,3], got {points.shape}")
    if K.shape != (3, 3):
        raise ValueError(f"K must have shape [3,3], got {K.shape}")
    if not np.isfinite(points).all() or not np.isfinite(K).all():
        raise ValueError("inputs must be finite")
    if np.any(points[:, 2] <= 0.0):
        raise ValueError("all points must lie in front of the camera")

    normalized = points[:, :2] / points[:, 2:3]
    pixels = np.empty_like(normalized)
    pixels[:, 0] = K[0, 0] * normalized[:, 0] + K[0, 2]
    pixels[:, 1] = K[1, 1] * normalized[:, 1] + K[1, 2]
    return pixels
```

最小测试应包含光轴上的点、四个象限、不同深度但同一射线的点，以及 $Z=0$、负深度、`NaN` 和错误形状。特别要验证尺度不变性：`project_points(P, K)` 应与 `project_points(3 * P, K)` 相等。

```python
def test_projection_is_invariant_along_ray():
    K = np.array([[600., 0., 320.], [0., 600., 240.], [0., 0., 1.]])
    point = np.array([[0.2, -0.1, 2.0]])
    np.testing.assert_allclose(
        project_points(point, K),
        project_points(3.0 * point, K),
        atol=1e-12,
    )
```

### 2.4 像素反投影是一条射线

给定无畸变像素 $\tilde p=(u,v,1)^T$，可计算相机坐标中的方向

$$
d=K^{-1}\tilde p.
$$

但它不是唯一三维点。所有 $P_c=Zd$ 都产生同一像素，只有得到深度 $Z$ 后才能恢复坐标：

$$
X=(u-c_x)Z/f_x,\qquad
Y=(v-c_y)Z/f_y.
$$

```python
def unproject_pixels(pixels: np.ndarray, depth_m: np.ndarray,
                     K: np.ndarray) -> np.ndarray:
    pixels = np.asarray(pixels, dtype=np.float64)
    depth = np.asarray(depth_m, dtype=np.float64).reshape(-1)
    if pixels.shape != (depth.size, 2):
        raise ValueError("pixels must be [N,2] and depth must be [N]")
    if np.any(depth <= 0.0) or not np.isfinite(depth).all():
        raise ValueError("depth must be finite and positive")
    x = (pixels[:, 0] - K[0, 2]) / K[0, 0]
    y = (pixels[:, 1] - K[1, 2]) / K[1, 1]
    return np.column_stack((x * depth, y * depth, depth))
```

深度值必须说明是沿相机 $z$ 轴的深度，还是从光心到点的欧氏距离。常见 RGB-D 图像保存的是 $Z$；某些激光或射线渲染接口返回的是 range。把 range 当作 $Z$ 时，图像边缘误差更大。

### 2.5 分辨率变化对内参的影响

如果图像从 $(W,H)$ 等比例缩放到 $(aW,aH)$，则 $f_x,f_y,c_x,c_y$ 同时乘以 $a$。若左右裁剪 $d_x$、上下裁剪 $d_y$，主点变为 $(c_x-d_x,c_y-d_y)$，焦距不变。先裁剪再缩放时必须按实际顺序更新。

不能把 1920×1080 标定得到的内参直接用于 640×480 图像，因为后者可能不是简单等比例缩放，而是改变传感器读出区域或宽高比。可靠方法是在部署分辨率和对焦状态下标定，或通过相机驱动提供的 `CameraInfo` 获取匹配参数。

### 2.6 视场角与空间分辨率

忽略畸变时，水平视场角近似为

$$
\operatorname{FOV}_x=2\arctan\frac{W}{2f_x}.
$$

焦距越小，视场越宽，但同一物体占据的像素更少。若目标宽度为 $L$，距离约为 $Z$，它在图像上的宽度近似为 $f_xL/Z$。这条公式可以用于采型前的可行性估算：如果最远工作距离上目标只有 6 个像素宽，再复杂的识别算法也很难稳定输出精确边界。

## 第 3 章：镜头畸变与校正

### 3.1 径向畸变

理想针孔模型假设所有光线在一个点相交，真实镜头会使直线在图像边缘弯曲。对归一化坐标 $(x,y)$，令 $r^2=x^2+y^2$。OpenCV 常用的径向模型为

$$
x_r=x(1+k_1r^2+k_2r^4+k_3r^6),
$$

$$
y_r=y(1+k_1r^2+k_2r^4+k_3r^6).
$$

桶形畸变通常使边缘向外鼓，枕形畸变使边缘向内收。系数正负与具体公式约定有关，不要仅凭符号判断镜头类型，应直接绘制畸变前后的网格。

### 3.2 切向畸变

镜头与传感器平面不完全平行会产生切向畸变：

$$
x_t=2p_1xy+p_2(r^2+2x^2),
$$

$$
y_t=p_1(r^2+2y^2)+2p_2xy.
$$

最终畸变坐标为 $x_d=x_r+x_t$、$y_d=y_r+y_t$，再经内参转换成像素。部分 OpenCV 模型还包含 $k_4,k_5,k_6$ 的有理函数、薄棱镜和倾斜传感器参数。参数越多不一定越好：如果标定图像不能约束高阶项，多余自由度会拟合噪声，并在图像边缘产生异常外推。

### 3.3 普通模型与鱼眼模型不能混用

普通透视镜头常使用 `cv2.calibrateCamera` 和 Brown-Conrady 模型；超广角、鱼眼镜头更适合 `cv2.fisheye.calibrate` 的角度模型。鱼眼视场接近或超过 180 度时，针孔平面无法稳定表示所有射线。不能因为普通接口返回了一个较低 RMS 就认为模型正确，要观察边缘直线、每张图误差和实际定位误差。

### 3.4 去畸变的两种策略

第一种是在整幅图像上建立映射表，然后每帧 `remap`。适合需要输出矫正图像的检测和显示。第二种只对稀疏关键点调用 `undistortPoints`，避免对所有像素插值，适合特征匹配与几何估计。

```python
import cv2

def build_undistort_maps(K, dist, image_size, alpha=0.0):
    width, height = image_size
    new_K, valid_roi = cv2.getOptimalNewCameraMatrix(
        K, dist, (width, height), alpha, (width, height)
    )
    map_x, map_y = cv2.initUndistortRectifyMap(
        K, dist, None, new_K, (width, height), cv2.CV_32FC1
    )
    return map_x, map_y, new_K, valid_roi
```

`alpha=0` 倾向于裁掉无效黑边并保留全有效区域，`alpha=1` 倾向于保留更多原始视场但会出现黑边。改变 `new_K` 后，下游必须使用新的内参。常见错误是图像已经去畸变，却仍向 PnP 传入原始畸变系数，导致二次校正。

## 第 4 章：平面标定为什么可行

### 4.1 从平面单应性约束内参

令标定板坐标系位于 $Z=0$ 平面，则空间点为 $(X,Y,0,1)^T$。投影方程化简为

$$
s\tilde p=K[r_1\ r_2\ t]
\begin{bmatrix}X\\Y\\1\end{bmatrix}
=H\tilde P.
$$

每张标定图像都提供一个平面单应矩阵 $H$。因为旋转矩阵的前两列满足

$$
r_1^Tr_2=0,\qquad \|r_1\|=\|r_2\|,
$$

可以对 $K^{-T}K^{-1}$ 建立约束。多张不同姿态的图像共同约束内参，随后再恢复每张图的外参，并通过非线性最小二乘联合优化内参、畸变和外参。这就是 Zhang 平面标定法的核心逻辑。

如果所有图像都正对标定板、距离和位置相近，提供的约束高度相似，参数会病态。不同倾角不是为了“照片看起来丰富”，而是为了提升观测方程的独立性。

### 4.2 棋盘格规格的含义

OpenCV 的 `pattern_size=(columns, rows)` 指内部角点数，不是黑白方格数量。一个 10×7 方格的棋盘通常有 9×6 内角点。`square_size` 可以用毫米或米，只要统一；内参不受单位影响，但每张图外参的平移会使用相同单位。

标定板必须平整。普通打印纸贴在软纸板上可能弯曲，模型却仍假设所有点共面。对于精确机械臂定位，建议使用尺寸经过测量的硬质标定板，并记录温度、对焦和镜头固定状态。

### 4.3 数据采集协议

一组高质量单目标定数据应满足：

1. 标定板覆盖画面中心、四边和四角。
2. 包含近、中、远距离，但角点仍清晰可见。
3. 绕水平轴和垂直轴有充分倾斜，而不只是平面内旋转。
4. 不使用明显运动模糊、过曝、反光或角点被遮挡的图像。
5. 固定焦距、分辨率和对焦；自动对焦相机应锁定对焦后采集。
6. 相邻帧差异足够大，避免从同一静止视频截取几十张近重复帧。

“图片越多越好”并不准确。20 张覆盖充分且清晰的图片，通常比 200 张几乎相同的图片更有价值。采集程序可计算角点中心、覆盖面积和板面姿态，对重复视角发出提示。

## 第 5 章：完整单目标定程序

### 5.1 角点检测与数据整理

```python
from dataclasses import dataclass
from pathlib import Path
import cv2
import numpy as np

@dataclass
class CalibrationView:
    path: Path
    object_points: np.ndarray
    image_points: np.ndarray

def make_board_points(pattern_size, square_size_m):
    columns, rows = pattern_size
    points = np.zeros((columns * rows, 3), dtype=np.float32)
    points[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    points[:, :2] *= square_size_m
    return points

def collect_views(paths, pattern_size=(9, 6), square_size_m=0.024):
    board = make_board_points(pattern_size, square_size_m)
    views = []
    image_size = None
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        50,
        1e-4,
    )

    for path in map(Path, paths):
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            print(f"skip unreadable image: {path}")
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        current_size = (gray.shape[1], gray.shape[0])
        if image_size is None:
            image_size = current_size
        elif current_size != image_size:
            raise ValueError(f"mixed image sizes: {image_size} and {current_size}")

        found, corners = cv2.findChessboardCorners(
            gray,
            pattern_size,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if not found:
            print(f"corners not found: {path}")
            continue
        refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        views.append(CalibrationView(path, board.copy(), refined))

    if image_size is None or len(views) < 10:
        raise RuntimeError(f"too few valid calibration views: {len(views)}")
    return views, image_size
```

角点精细化依赖局部灰度梯度。图像严重模糊时，算法可能仍返回角点，但位置不稳定。应保存带角点覆盖的预览图，人工抽查边缘和高倾角样本。

### 5.2 求解、逐图误差与结果保存

```python
import json

def calibrate(views, image_size):
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        [v.object_points for v in views],
        [v.image_points for v in views],
        image_size,
        None,
        None,
    )
    per_view = []
    all_squared_error = 0.0
    all_point_count = 0
    for view, rvec, tvec in zip(views, rvecs, tvecs):
        projected, _ = cv2.projectPoints(
            view.object_points, rvec, tvec, K, dist
        )
        residual = projected.reshape(-1, 2) - view.image_points.reshape(-1, 2)
        point_error = np.linalg.norm(residual, axis=1)
        per_view.append({
            "path": str(view.path),
            "rmse_px": float(np.sqrt(np.mean(point_error ** 2))),
            "mean_px": float(np.mean(point_error)),
            "max_px": float(np.max(point_error)),
        })
        all_squared_error += float(np.sum(point_error ** 2))
        all_point_count += point_error.size

    global_rmse = np.sqrt(all_squared_error / all_point_count)
    return {
        "opencv_rms": float(rms),
        "global_rmse_px": float(global_rmse),
        "K": K,
        "dist": dist,
        "rvecs": rvecs,
        "tvecs": tvecs,
        "per_view": per_view,
    }

def save_intrinsics(result, image_size, destination):
    payload = {
        "image_width": image_size[0],
        "image_height": image_size[1],
        "camera_matrix": result["K"].tolist(),
        "distortion_coefficients": result["dist"].reshape(-1).tolist(),
        "global_rmse_px": result["global_rmse_px"],
        "per_view": result["per_view"],
    }
    Path(destination).write_text(
        json.dumps(payload, indent=2), encoding="utf-8"
    )
```

### 5.3 不要机械删除误差最大的图片

误差最大的图片可能确实模糊或检测错误，也可能恰好是唯一覆盖图像边缘的高倾角视图。直接删除会降低平均误差，却削弱参数约束。正确流程是查看残差向量：若某张图所有角点呈一致偏移，可能是图像或板面问题；若全体图像在边缘呈系统性方向误差，可能是畸变模型不足；若少量单点异常，可能是角点检测错误。

剔除样本必须记录原因，并在独立验证集上比较。训练误差降低而验证误差升高，说明模型或筛选流程在过拟合。

## 第 6 章：怎样判断标定结果可信

### 6.1 重投影误差只是必要条件

重投影残差为

$$
e_{ij}=p_{ij}-\pi(K,d,R_i,t_i,P_j),
$$

其中 $i$ 表示图像，$j$ 表示角点。优化目标通常最小化 $\sum\|e_{ij}\|^2$。较小误差说明参数能够解释这些标定图像，但不保证物理参数唯一，也不保证在未覆盖区域外推正确。

至少进行以下检查：

- `fx`、`fy` 为正，数量级与分辨率和视场匹配。
- 主点在图像附近，不应远离画面。
- 畸变后再去畸变的点能够近似恢复。
- 每张图误差没有明显长尾，残差方向没有空间结构。
- 独立拍摄的直线经去畸变后接近直线。
- 用已知尺寸或已知距离完成外部验证。

“RMS 小于 0.5 像素就一定合格”不是通用规则。对远距离粗检测可能足够，对高精度手眼标定可能远远不够。门槛必须从最终三维误差倒推。

### 6.2 像素误差怎样传播到空间

由 $X=(u-c_x)Z/f_x$ 可得

$$
\frac{\partial X}{\partial u}=\frac{Z}{f_x},\qquad
\frac{\partial X}{\partial Z}=\frac{u-c_x}{f_x}.
$$

在 $f_x=600$ px、$Z=2$ m 时，1 px 水平误差约对应 $3.3$ mm 的横向误差；在 10 m 时约为 $16.7$ mm。距离增加后，同样的像素误差造成更大的空间误差。深度误差还会与离主点距离耦合，因此应在完整工作空间内评估，而不是只测图像中心和近距离。

### 6.3 交叉验证稳定性

把标定图像分成若干组，反复用其中大部分标定，用剩余图像评估。记录每次的 $f_x,f_y,c_x,c_y$ 和畸变系数。如果不同子集产生的参数变化很大，而各自训练误差都很小，说明数据覆盖不足或参数不可观。

还可逐步增加图像数量画稳定性曲线。当新增不同姿态后参数逐渐收敛，说明数据有信息；如果新增近重复帧只让 RMS 轻微变小，却不改善独立误差，则继续采相同数据没有价值。

## 第 7 章：双目成像与对极约束

### 7.1 为什么第二个视角能恢复深度

单个像素对应一条空间射线。两个相机观察同一点时，理论上两条射线的交点给出三维位置。实际测量有噪声，两条射线往往不严格相交，三角化寻找最符合两幅图像观测的空间点。

设点在第一相机归一化平面为 $x_1$，第二相机为 $x_2$，两相机关系为

$$
P_2=RP_1+t.
$$

第二视图的点、平移向量和旋转后的第一条射线共面，因此

$$
x_2^T[t]_\times R x_1=0.
$$

定义本质矩阵

$$
E=[t]_\times R,
$$

就得到 $x_2^TEx_1=0$。本质矩阵只作用于已经去畸变并消除内参的归一化坐标。

### 7.2 基础矩阵

像素坐标满足 $x_1=K_1^{-1}p_1$、$x_2=K_2^{-1}p_2$。代入可得

$$
p_2^TFp_1=0,
\qquad
F=K_2^{-T}EK_1^{-1}.
$$

给定第一幅图的点 $p_1$，第二幅图中的对应点必须位于对极线 $l_2=Fp_1$ 上。对极约束把二维搜索降低为沿一条线搜索，但不能单独确定对应点；重复纹理、无纹理和遮挡仍会失败。

### 7.3 本质矩阵的自由度与退化

相对旋转有 3 个自由度，平移方向有 2 个自由度，本质矩阵整体尺度不可观，因此共有 5 个自由度。由 $E$ 分解出的平移只有方向，没有绝对长度。要得到米制深度，需要已知双目基线、物体尺寸、IMU/轮速或其他尺度来源。

纯旋转时 $t=0$，本质矩阵退化，不能三角化深度。点全部位于同一平面时，单应矩阵也能很好解释匹配，恢复位姿可能出现多解。低视差、点集中在小区域或多数点位于无穷远都会降低平移估计稳定性。

## 第 8 章：从匹配点估计相对位姿

### 8.1 完整 OpenCV 流程

```python
def estimate_relative_pose(points1, points2, K, dist=None,
                           ransac_threshold_px=1.0):
    points1 = np.asarray(points1, dtype=np.float64).reshape(-1, 1, 2)
    points2 = np.asarray(points2, dtype=np.float64).reshape(-1, 1, 2)
    if len(points1) != len(points2) or len(points1) < 8:
        raise ValueError("need at least eight paired points")

    if dist is None:
        dist = np.zeros(5)
    normalized1 = cv2.undistortPoints(points1, K, dist)
    normalized2 = cv2.undistortPoints(points2, K, dist)
    focal_scale = 0.5 * (K[0, 0] + K[1, 1])
    normalized_threshold = ransac_threshold_px / focal_scale

    E, ransac_mask = cv2.findEssentialMat(
        normalized1,
        normalized2,
        focal=1.0,
        pp=(0.0, 0.0),
        method=cv2.RANSAC,
        prob=0.999,
        threshold=normalized_threshold,
    )
    if E is None:
        raise RuntimeError("essential matrix estimation failed")

    inlier_count, R, t, pose_mask = cv2.recoverPose(
        E, normalized1, normalized2, mask=ransac_mask
    )
    final_mask = (pose_mask.reshape(-1) != 0)
    return R, t.reshape(3), final_mask, int(inlier_count)
```

代码先去畸变得到归一化点，所以 `findEssentialMat` 使用单位焦距和零主点。RANSAC 阈值也从像素转换到归一化尺度。混用像素点和归一化阈值是常见错误。

`recoverPose` 在本质矩阵的四种候选分解中，通过正深度条件选择使最多点同时位于两台相机前方的解。返回的 $t$ 通常被归一化，表示方向而非米制位移。

### 8.2 不能只看内点数量

内点多可能只是匹配都落在同一平面，或阈值过宽。还应检查：内点在图像中的覆盖范围、对极残差分布、三角化后的正深度比例、视差角、恢复旋转与运动先验是否一致，以及交换两帧后变换是否近似互逆。

Sampson 距离是对几何重投影误差的一阶近似：

$$
d_S=\frac{(p_2^TFp_1)^2}
{(Fp_1)_1^2+(Fp_1)_2^2+(F^Tp_2)_1^2+(F^Tp_2)_2^2}.
$$

它比直接使用代数误差 $p_2^TFp_1$ 更有几何意义，可用于排序和诊断，但最终高精度系统仍应联合优化相机位姿与三维点。

## 第 9 章：三角化与不确定性

### 9.1 线性三角化

对投影矩阵 $P_i$ 和观测 $x_i=(u_i,v_i,1)^T$，有 $x_i\times P_iX=0$。每个视图提供两条独立线性约束，两个视图组成齐次方程 $AX=0$，通过 SVD 取最小奇异值对应的向量作为齐次三维点。

OpenCV 实现：

```python
def triangulate_normalized(points1, points2, R, t):
    p1 = np.asarray(points1, dtype=np.float64).reshape(-1, 2).T
    p2 = np.asarray(points2, dtype=np.float64).reshape(-1, 2).T
    P1 = np.column_stack((np.eye(3), np.zeros(3)))
    P2 = np.column_stack((R, np.asarray(t).reshape(3)))
    homogeneous = cv2.triangulatePoints(P1, P2, p1, p2)
    valid_w = np.abs(homogeneous[3]) > 1e-12
    points3d = np.full((homogeneous.shape[1], 3), np.nan)
    points3d[valid_w] = (
        homogeneous[:3, valid_w] / homogeneous[3:4, valid_w]
    ).T
    depth1 = points3d[:, 2]
    points_in_2 = (R @ points3d.T + np.asarray(t).reshape(3, 1)).T
    valid = valid_w & (depth1 > 0.0) & (points_in_2[:, 2] > 0.0)
    return points3d, valid
```

输入必须是归一化坐标，因为这里的 $P_1=[I|0]$、$P_2=[R|t]$ 不含内参。如果输入原始像素，则投影矩阵必须左乘对应的 $K$。

### 9.2 视差决定深度精度

对已经极线校正的平行双目，基线为 $B$，左右像素水平差为 $d=u_L-u_R$，深度为

$$
Z=\frac{f_xB}{d}.
$$

其对视差的导数为

$$
\frac{\partial Z}{\partial d}=-\frac{f_xB}{d^2}
=-\frac{Z^2}{f_xB}.
$$

因此深度误差随距离平方增长。假设 $f_x=700$ px、$B=0.12$ m、视差误差标准差 $\sigma_d=0.5$ px，在 $Z=2$ m 时，近似深度标准差为

$$
\sigma_Z\approx\frac{Z^2}{f_xB}\sigma_d
=\frac{4}{84}\times0.5\approx0.024\text{ m}.
$$

在 6 m 时则约为 0.214 m。这个推导说明，仅靠调网络不一定能解决远距离深度问题；增加基线、提高有效焦距、改善亚像素匹配或融合其他传感器更直接。

### 9.3 三角化后的验收

每个三维点至少检查：齐次分母不接近零；在两相机中均为正深度；两条观测射线夹角超过最小阈值；重投影误差低于门槛；深度位于任务工作范围。对低视差点宁可标记为高不确定性，也不要输出一个看似精确的巨大深度。

## 第 10 章：单目与双目标定综合实验

### 10.1 实验设备与固定条件

准备硬质棋盘格或 ChArUco 板、卷尺、固定支架和目标相机。记录相机型号、序列号、分辨率、曝光、增益、焦距、对焦状态和驱动版本。双目实验还要记录两相机基线和硬件同步方式。

不要在实验中途改变分辨率和对焦。自动曝光可以在采集前调整，但应避免一组图像中亮度剧烈变化。标定数据、验证数据和最终应用数据分目录保存，原始文件只读保留。

### 10.2 实验步骤

1. 打印或购买标定板，用量具测量多个方格，记录均值和测量不确定性。
2. 采集约 25 张覆盖不同位置、距离和倾角的清晰图像。
3. 检测并保存角点可视化，拒绝模糊、遮挡和错误排序样本。
4. 用全部有效图像标定，输出内参、畸变、每图误差和残差图。
5. 做五折子集实验，统计内参参数波动。
6. 用独立直线场景和已知尺寸物体验证去畸变及空间测量。
7. 双目系统分别完成单目标定，再估计两相机相对外参并进行极线校正。
8. 在 0.5、1、2、4 米距离放置目标，比较三角化距离与卷尺测量。

### 10.3 必须保存的结果表

| 项目 | 指标 | 通过标准示例 |
| --- | --- | --- |
| 角点覆盖 | 图像网格覆盖率 | 四角均有有效样本 |
| 单目标定 | 独立集重投影 P95 | 根据任务设定 |
| 参数稳定性 | 五折 `fx` 变异系数 | 小于项目门槛 |
| 去畸变 | 独立直线最大偏差 | 无系统性边缘弯曲 |
| 双目校正 | 对应点纵向误差 P95 | 例如小于 1 px |
| 三角化 | 各距离绝对误差 P95 | 满足抓取或导航需求 |
| 鲁棒性 | 低光和边缘区域失败率 | 有明确降级策略 |

表中的门槛需要从任务反推。桌面抓取可能要求毫米到厘米级误差，室外障碍物检测更关注分米级误差和漏检率。不要复制别人的阈值而不说明工作距离和硬件。

### 10.4 失败案例记录模板

每个失败样本保存：原始图像、角点或匹配覆盖图、相机参数版本、输入时间戳、算法输出、期望结果、重投影残差、是否位于训练或标定覆盖范围、初步根因和下一步最小实验。

示例结论：“4 m 距离的深度 P95 从 8 cm 上升到 31 cm；纵向极线误差仍小于 0.7 px，但水平视差仅约 18 px。误差符合 $Z^2/(fB)$ 增长趋势，优先测试亚像素匹配与更大基线，而不是重新拟合更多畸变参数。”

## 第 11 章：常见错误与系统化排查

### 错误一：去畸变后图像反而更弯

检查标定分辨率与当前图像是否一致；检查普通模型和鱼眼模型是否混用；确认畸变系数顺序；确认 `new_K` 是否与输出图像一起传递；检查图像是否已经被相机驱动去畸变。用人工绘制的规则网格验证代码路径，而不是只看自然场景。

### 错误二：PnP 或三角化位置差 1000 倍

检查标定板尺寸使用毫米还是米，深度图 scale，TF 平移单位，以及模型输出单位。旋转矩阵没有长度单位，平移有，因此尺度错误经常只出现在位置而不出现在朝向。

### 错误三：双目极线校正后仍有纵向偏差

检查两相机是否真正同步；标定板在左右图是否是同一时刻；角点顺序是否一致；镜头是否在标定后移动；图像是否被单独缩放或裁剪。动态场景中的纵向偏差可能来自时间不同步，而不是几何参数。

### 错误四：重投影误差很好，实际距离仍不准

可能原因包括标定板尺寸错误、板面弯曲、数据姿态退化、双目基线测量错误、低视差和错误外参方向。使用独立尺寸目标能区分“解释标定数据很好”和“物理尺度正确”。

### 错误五：相对位姿方向偶尔翻转

检查内点是否集中、视差是否过低、场景是否近似平面或纯旋转。正深度检查只能在有足够视差和正确匹配时可靠。应输出退化评分并拒绝低质量结果，而不是强制每帧都给位姿。

## 第 12 章：阶段考试

建议限时 150 分钟，满分 100 分。先独立完成，再阅读答案。

### 一、概念与推导，共 30 分

1. 证明同一射线上的点具有相同像素坐标，并解释这对单目绝对尺度的影响。（6 分）
2. 图像等比例缩小一半后，$f_x,f_y,c_x,c_y$ 如何变化？若只是从左侧裁掉 100 像素呢？（6 分）
3. 为什么平面标定需要多个倾斜姿态，而不是只拍摄正对棋盘的图像？（6 分）
4. 说明本质矩阵与基础矩阵的输入坐标、关系和尺度性质。（6 分）
5. 从 $Z=fB/d$ 推导视差噪声对深度误差的影响。（6 分）

### 二、代码与接口，共 30 分

1. 为 `project_points` 设计六个边界测试，并说明每个测试防止什么错误。（10 分）
2. 一段程序先调用 `cv2.undistort`，随后把输出关键点和原始 `dist` 一起传给 `solvePnP`。指出问题并给出两种正确方案。（10 分）
3. 实现一个函数，根据原内参、裁剪区域和缩放比例计算新内参，并写最小测试。（10 分）

### 三、实验分析，共 40 分

某次标定得到 RMS 0.19 px，`fx=2850`、`fy=910`，图像大小 1280×720，主点为 `(1880, -240)`；独立图像边缘仍有明显弯曲。实验者认为“RMS 已经很低，所以参数没问题”。分析至少五个风险，给出排查顺序和重新采集方案。（20 分）

某双目系统基线 6 cm，焦距 600 px，在 5 m 处视差约 7.2 px，匹配误差约 0.5 px。估算深度误差，判断是否适合要求 10 cm P95 精度的任务，并提出三种改进方向。（20 分）

## 第 13 章：参考答案与评分要点

### 一、概念题答案

1. 对点 $(X,Y,Z)$，像素取决于 $X/Z$ 和 $Y/Z$。缩放为 $(\lambda X,\lambda Y,\lambda Z)$ 后两个比例不变，因此像素不变。单幅图像只能确定射线，缺少绝对深度或尺度来源。

2. 等比例缩小一半时四个参数都乘 0.5。仅从左裁掉 100 px 时焦距不变，$c_x$ 减 100，$c_y$ 不变。若随后再缩放，应在裁剪结果上按比例缩放。

3. 正对视图产生的单应约束相似，焦距、距离和部分畸变容易耦合。不同绕水平轴和垂直轴的倾角使约束方向更独立，提高参数可观性。

4. $E$ 作用于去除内参后的归一化坐标，$E=[t]_\times R$；$F$ 作用于像素坐标，$F=K_2^{-T}EK_1^{-1}$。两者都只确定到非零尺度，分解 $E$ 得到的平移也只有方向。

5. 对 $Z=fB/d$ 求导得 $\partial Z/\partial d=-fB/d^2=-Z^2/(fB)$，小噪声近似满足 $\sigma_Z\approx Z^2\sigma_d/(fB)$，因此距离翻倍时深度误差约增至四倍。

### 二、代码题答案

`project_points` 的六类测试可包括：光轴点落在主点；已知正负横纵坐标投到正确象限；同射线尺度不变；零深度被拒绝；负深度被拒绝；`NaN`、错误形状或错误内参被拒绝。只要测试给出具体输入、断言和对应风险即可得分。

去畸变图像上的关键点已经符合新针孔模型，再传原畸变会重复校正。方案一是在原始畸变图像上检测点，向 `solvePnP` 传原始 `K` 和 `dist`。方案二是在去畸变图像上检测点，传生成该图像的 `new_K`，畸变设为空或全零。图像与参数必须成对。

```python
def adjusted_intrinsics(K, crop_x, crop_y, scale_x, scale_y=None):
    if scale_y is None:
        scale_y = scale_x
    if scale_x <= 0 or scale_y <= 0:
        raise ValueError("scales must be positive")
    result = np.asarray(K, dtype=np.float64).copy()
    result[0, 2] -= crop_x
    result[1, 2] -= crop_y
    result[0, :] *= scale_x
    result[1, :] *= scale_y
    result[2, :] = [0.0, 0.0, 1.0]
    return result
```

测试可令 `K=[[600,0,320],[0,500,240],[0,0,1]]`，左裁 20、上裁 40，再水平缩放 0.5、垂直缩放 0.25，期望 `fx=300`、`fy=125`、`cx=150`、`cy=50`。

### 三、实验题答案

第一题中，`fx/fy` 比例严重异常，主点甚至位于图像外很远，参数与一般相机物理结构不符；低 RMS 可能来自姿态退化、错误角点规格、高阶畸变过拟合、近重复图像或只覆盖中心。边缘直线弯曲已经提供独立反证，不能用训练重投影误差否定它。

排查顺序应为：确认图像尺寸和棋盘内角点行列；核对方格尺寸与板面平整；绘制每张角点和残差；查看位置与倾角覆盖；检查是否固定了不合理参数或使用错误畸变模型；用子集交叉验证观察参数波动。重新采集时锁定分辨率、焦距和对焦，让棋盘覆盖四角、四边、中心和多个距离，并加入绕两个轴的倾斜，保留独立验证图像。

第二题使用 $\sigma_Z\approx Z^2\sigma_d/(fB)$：

$$
\sigma_Z\approx\frac{25\times0.5}{600\times0.06}
=0.347\text{ m}.
$$

即使把它视为标准差而非 P95，也已经远大于 10 cm，不能满足要求。改进方向包括增加物理基线、提高有效焦距或输入分辨率、改善标定与亚像素匹配；也可限制工作距离、融合主动深度或激光雷达。单纯增加 RANSAC 次数不会改变低视差导致的基本精度上限。

## 本篇完成标准

只有同时满足以下条件，才算真正完成本篇：能够不查资料写出坐标变换和针孔投影；投影、反投影和内参调整代码通过边界测试；完成一组具有充分姿态覆盖的真实标定；保存每图残差和交叉验证结果；用独立场景验证几何参数；完成双目三角化误差曲线；能够解释一个低重投影误差但实际失败的反例。

下一篇将在这些几何约束上继续学习图像形成、滤波、角点、描述子、光流、匹配、RANSAC 和多视图重建。几何模型不会被深度学习淘汰：它们仍是数据生成、结果校验、位姿恢复和失败诊断的基本工具。
