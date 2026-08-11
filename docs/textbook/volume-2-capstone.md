---
title: 第二册结业实训：RGB-D 感知系统与全册考试
description: 将相机、特征、点云、检测分割和 6D 位姿整合为可复现工程，包含代码模板、故障树、实验、考试和答案。
---

# 第二册结业实训：RGB-D 感知系统与全册考试

本实训把第二册前四篇知识整合成一个可运行项目：输入同步的彩色图、深度图和相机参数，检测并分割已知类别物体，估计相机坐标中的三维位置，经过几何质量门禁后转换到机器人基座坐标，发布带协方差、状态和诊断信息的结果。

项目故意不把“换一个更大的模型”当作默认答案。一个可信的机器人感知系统必须同时处理数据契约、几何标定、学习推理、时间同步、坐标变换、异常拒绝、性能预算和版本复现。任何一环错误，网络 mAP 再高也无法保证机械臂拿到正确目标。

## 第 1 章：结业项目的边界与验收目标

### 1.1 任务定义

工作台上放置 3 至 5 种刚性物体。固定 RGB-D 相机或腕部相机以 15 Hz 以上输入彩色和深度，系统输出每个实例：

- 类别、实例 ID 和二维掩码质量；
- 采样时间戳与结果消息年龄；
- `camera_optical_frame` 中的三维中心或 6D 位姿；
- `base_link` 中的三维中心或 6D 位姿；
- 平移协方差、质量等级和标准化拒绝原因；
- 模型、相机标定和外参版本。

第一版只要求通过实例掩码与 RGB-D 得到可靠三维中心；进阶版为有 CAD 模型的物体加入关键点 PnP 或点云配准，输出完整 6D 位姿。分层实现能把“掩码失败”和“姿态算法失败”分开诊断。

### 1.2 示例门槛

以下数值只是桌面项目起点，应依据硬件和抓取容差重新确定：

| 指标 | 基础级 | 进阶级 |
| --- | ---: | ---: |
| 实例检测召回 | 95% | 98% |
| 连续漏检 | 不超过 5 帧 | 不超过 2 帧 |
| 相机坐标位置 P95 | 20 mm | 10 mm |
| base 坐标位置 P95 | 30 mm | 15 mm |
| 姿态角误差 P95 | 不要求 | 5° |
| 端到端延迟 P95 | 100 ms | 60 ms |
| 错误接受率 | 小于 1% | 小于 0.2% |
| 连续运行 | 30 min | 4 h |

错误接受指质量门禁判断有效但实际误差超过安全阈值。机器人任务中它通常比主动拒绝更危险，因此必须单独统计。

### 1.3 非目标

第一版不解决透明物体精密深度、可变形物体 6D 位姿、开放词汇识别和高速动态抓取。把非目标明确写出不是逃避，而是防止项目在没有验证基本链路前无限扩张。

## 第 2 章：仓库结构和数据契约

```text
rgbd_pose_system/
  pyproject.toml
  README.md
  configs/
    base.yaml
    camera_front.yaml
    thresholds.yaml
  assets/
    class_names.json
    models/
  data/
    raw/
    annotations/
    splits/
  src/rgbd_pose/
    __init__.py
    contracts.py
    calibration.py
    depth.py
    geometry.py
    postprocess.py
    quality.py
    metrics.py
    pipeline.py
  tests/
    test_contracts.py
    test_depth.py
    test_geometry.py
    test_quality.py
    test_pipeline_synthetic.py
  ros2_ws/src/rgbd_pose_ros/
  scripts/
    validate_dataset.py
    evaluate_offline.py
    benchmark.py
  outputs/
```

### 2.1 不可变输入对象

```python
# src/rgbd_pose/contracts.py
from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class CameraModel:
    width: int
    height: int
    matrix: np.ndarray
    distortion: np.ndarray
    frame_id: str
    calibration_id: str

    def validate(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("camera dimensions must be positive")
        if self.matrix.shape != (3, 3):
            raise ValueError("camera matrix must be 3x3")
        if not np.isfinite(self.matrix).all():
            raise ValueError("camera matrix must be finite")
        if self.matrix[0, 0] <= 0 or self.matrix[1, 1] <= 0:
            raise ValueError("focal lengths must be positive")
        if not self.frame_id or not self.calibration_id:
            raise ValueError("frame and calibration identifiers are required")

@dataclass(frozen=True)
class FrameBundle:
    rgb: np.ndarray
    depth_m: np.ndarray
    timestamp_ns: int
    camera: CameraModel

    def validate(self) -> None:
        self.camera.validate()
        expected = (self.camera.height, self.camera.width)
        if self.rgb.shape != (*expected, 3):
            raise ValueError(f"RGB shape mismatch: {self.rgb.shape} vs {expected}")
        if self.depth_m.shape != expected:
            raise ValueError("depth shape does not match camera model")
        if self.rgb.dtype != np.uint8 or self.depth_m.dtype != np.float32:
            raise ValueError("expected uint8 RGB and float32 depth in meters")
        if self.timestamp_ns <= 0:
            raise ValueError("timestamp must be positive")

@dataclass(frozen=True)
class InstancePrediction:
    class_id: int
    score: float
    mask: np.ndarray

    def validate(self, shape) -> None:
        if self.class_id < 0 or not 0.0 <= self.score <= 1.0:
            raise ValueError("invalid class or score")
        if self.mask.shape != shape or self.mask.dtype != np.bool_:
            raise ValueError("mask must be bool and match image shape")
```

进入核心管线后，所有深度统一为 `float32` 米，颜色统一为 RGB `uint8`，时间统一为纳秒整数。设备驱动的毫米、BGR 或其他格式只能在边界适配器中出现。这样避免每个函数都猜测数据语义。

### 2.2 输出状态不是一个布尔值

```python
from enum import Enum

class RejectReason(str, Enum):
    NONE = "none"
    INVALID_INPUT = "invalid_input"
    LOW_DETECTION_SCORE = "low_detection_score"
    SMALL_MASK = "small_mask"
    INSUFFICIENT_DEPTH = "insufficient_depth"
    HIGH_DEPTH_DISPERSION = "high_depth_dispersion"
    GEOMETRY_DEGENERATE = "geometry_degenerate"
    OUTSIDE_WORKSPACE = "outside_workspace"
    STALE_TRANSFORM = "stale_transform"
    EXCESSIVE_LATENCY = "excessive_latency"
    TEMPORAL_JUMP = "temporal_jump"

@dataclass(frozen=True)
class PositionResult:
    valid: bool
    reason: RejectReason
    position_camera_m: np.ndarray | None
    covariance_camera: np.ndarray | None
    diagnostics: dict[str, float | int | str]
```

标准化拒绝原因使离线评估、ROS 诊断和线上监控使用同一语义。只有 `valid=True` 才允许消费者执行动作；无效结果不能通过全零位姿伪装成原点目标。

## 第 3 章：配置与复现

```yaml
# configs/base.yaml
seed: 42
depth:
  minimum_m: 0.20
  maximum_m: 2.50
  minimum_valid_pixels: 50
  minimum_valid_ratio: 0.35
  mad_multiplier: 3.5
  maximum_mad_m: 0.04
mask:
  minimum_area_px: 120
  erosion_radius_small: 1
  erosion_radius_large: 2
  large_area_px: 2500
detection:
  minimum_score: 0.55
workspace_camera:
  x: [-1.2, 1.2]
  y: [-0.8, 0.8]
  z: [0.2, 2.5]
runtime:
  maximum_latency_ms: 100
  maximum_rgb_depth_delta_ms: 20
  maximum_temporal_jump_m: 0.15
```

配置加载后转换成带类型的数据类并立即校验。输出目录保存解析后的完整配置，而不是只保存命令行覆盖项。正式实验元数据至少包含：Git commit、工作区是否干净、Python/PyTorch/CUDA/cuDNN 版本、GPU 名称、模型 SHA-256、相机标定 ID、外参 ID、数据集版本、随机种子和启动命令。

同一权重配不同类别表或预处理就不再是同一模型。部署包应把权重、类别表、输入尺寸、颜色顺序、归一化和后处理阈值作为一个版本化整体。

## 第 4 章：从实例掩码得到三维观测

### 4.1 为什么不用框中心深度

框中心可能落在背景、孔洞、物体把手空隙或遮挡者上。掩码内所有深度也不能直接平均，因为边界混合像素和背景大值会显著拉偏。稳健流程是：腐蚀边缘、过滤范围、用中位数和 MAD 识别主深度簇、再反投影有效像素。

### 4.2 核心实现

```python
# src/rgbd_pose/depth.py
import cv2
import numpy as np

def erode_instance_mask(mask: np.ndarray, radius: int) -> np.ndarray:
    if mask.dtype != np.bool_ or mask.ndim != 2:
        raise ValueError("mask must be a 2D bool array")
    if radius < 0:
        raise ValueError("radius must be non-negative")
    if radius == 0:
        return mask.copy()
    kernel = np.ones((2 * radius + 1, 2 * radius + 1), np.uint8)
    return cv2.erode(mask.astype(np.uint8), kernel).astype(bool)

def select_depth_inliers(depth_m, mask, minimum_m, maximum_m,
                         mad_multiplier=3.5):
    values = depth_m[mask]
    finite = values[
        np.isfinite(values) & (values >= minimum_m) & (values <= maximum_m)
    ]
    if finite.size == 0:
        return np.empty(0, np.float32), np.nan, np.nan
    median = float(np.median(finite))
    mad = float(np.median(np.abs(finite - median)))
    robust_sigma = max(1.4826 * mad, 1e-4)
    inliers = finite[np.abs(finite - median) <= mad_multiplier * robust_sigma]
    return inliers, median, mad

def masked_points(depth_m, mask, K, minimum_m, maximum_m,
                  mad_multiplier=3.5):
    valid_range = (
        mask & np.isfinite(depth_m)
        & (depth_m >= minimum_m) & (depth_m <= maximum_m)
    )
    values = depth_m[valid_range]
    if values.size == 0:
        return np.empty((0, 3), np.float64), valid_range, np.nan
    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median)))
    robust_sigma = max(1.4826 * mad, 1e-4)
    inlier_mask = valid_range & (
        np.abs(depth_m - median) <= mad_multiplier * robust_sigma
    )
    v, u = np.nonzero(inlier_mask)
    z = depth_m[v, u].astype(np.float64)
    x = (u - K[0, 2]) * z / K[0, 0]
    y = (v - K[1, 2]) * z / K[1, 1]
    return np.column_stack((x, y, z)), inlier_mask, mad
```

这段实现假设目标在深度上形成主要簇。若物体沿视线方向很长，或掩码包含两个分离表面，MAD 截断可能误删真实点。应在目标形状上验证，必要时使用深度连通分量、聚类或 CAD 模型。

### 4.3 中心与协方差

```python
def robust_centroid(points: np.ndarray, trim_quantile=0.9):
    points = np.asarray(points, np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3:
        raise ValueError("at least three 3D points are required")
    center0 = np.median(points, axis=0)
    distance = np.linalg.norm(points - center0, axis=1)
    cutoff = np.quantile(distance, trim_quantile)
    selected = points[distance <= cutoff]
    center = selected.mean(axis=0)
    covariance_points = np.cov(selected.T, ddof=1)
    covariance_mean = covariance_points / len(selected)
    return center, covariance_mean, selected
```

`covariance_mean` 只反映选中点的随机离散对均值的影响，不包含相机内参、深度系统偏差、RGB-D 外参和掩码偏差。输出可以把它作为下界，并通过重复测量建立经验误差模型。

### 4.4 掩码面积与有效深度率

小掩码的像素量化误差大，深度有效点也少。有效率定义应使用腐蚀后掩码面积作分母。对透明物体，只有背景深度也可能让“有效率”很高，所以还要检查深度分布、已知尺寸和表面一致性。

## 第 5 章：几何质量门禁

```python
# src/rgbd_pose/quality.py
from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class QualityThresholds:
    minimum_score: float
    minimum_mask_area: int
    minimum_depth_points: int
    minimum_depth_ratio: float
    maximum_depth_mad: float
    workspace_min: np.ndarray
    workspace_max: np.ndarray

def check_quality(score, mask_area, depth_count, depth_ratio,
                  depth_mad, position, thresholds):
    if not np.isfinite(score) or position.shape != (3,) or not np.isfinite(position).all():
        return RejectReason.INVALID_INPUT
    if score < thresholds.minimum_score:
        return RejectReason.LOW_DETECTION_SCORE
    if mask_area < thresholds.minimum_mask_area:
        return RejectReason.SMALL_MASK
    if depth_count < thresholds.minimum_depth_points or depth_ratio < thresholds.minimum_depth_ratio:
        return RejectReason.INSUFFICIENT_DEPTH
    if not np.isfinite(depth_mad) or depth_mad > thresholds.maximum_depth_mad:
        return RejectReason.HIGH_DEPTH_DISPERSION
    if np.any(position < thresholds.workspace_min) or np.any(position > thresholds.workspace_max):
        return RejectReason.OUTSIDE_WORKSPACE
    return RejectReason.NONE
```

门禁顺序应从便宜检查到昂贵检查，拒绝时保留已计算诊断。阈值来自验证集上的风险权衡：画出错误接受率与主动拒绝率曲线，根据机器人安全代价选择，不应凭感觉填一个整数。

### 5.1 门禁的反例测试

必须主动构造“高检测分数但错误深度”“大掩码但全是背景”“低 MAD 但位置越界”“位置正常但时间戳过期”等反例。每个反例只破坏一个条件，验证返回准确拒绝码。若多个条件同时失败，文档要规定主拒绝码优先级，同时在 diagnostics 中保存全部失败项。

## 第 6 章：端到端纯函数管线

```python
# src/rgbd_pose/pipeline.py
import time

class RgbdPositionPipeline:
    def __init__(self, predictor, config, thresholds):
        self.predictor = predictor
        self.config = config
        self.thresholds = thresholds

    def process(self, bundle: FrameBundle) -> list[PositionResult]:
        bundle.validate()
        predictions = self.predictor(bundle.rgb)
        results = []
        for prediction in predictions:
            prediction.validate(bundle.depth_m.shape)
            results.append(self._process_instance(bundle, prediction))
        return results

    def _process_instance(self, bundle, prediction):
        mask_cfg = self.config["mask"]
        area = int(prediction.mask.sum())
        radius = (
            mask_cfg["erosion_radius_small"]
            if area < mask_cfg["large_area_px"]
            else mask_cfg["erosion_radius_large"]
        )
        inner = erode_instance_mask(prediction.mask, radius)
        depth_cfg = self.config["depth"]
        points, inlier_mask, mad = masked_points(
            bundle.depth_m, inner, bundle.camera.matrix,
            depth_cfg["minimum_m"], depth_cfg["maximum_m"],
            depth_cfg["mad_multiplier"],
        )
        inner_area = int(inner.sum())
        ratio = len(points) / max(inner_area, 1)
        if len(points) >= 3:
            position, covariance, selected = robust_centroid(points)
        else:
            position = np.full(3, np.nan)
            covariance = None
            selected = points
        reason = check_quality(
            prediction.score, area, len(selected), ratio, mad,
            position, self.thresholds,
        )
        diagnostics = {
            "class_id": prediction.class_id,
            "score": prediction.score,
            "mask_area": area,
            "inner_area": inner_area,
            "depth_points": len(points),
            "selected_points": len(selected),
            "depth_ratio": ratio,
            "depth_mad_m": float(mad),
            "calibration_id": bundle.camera.calibration_id,
        }
        return PositionResult(
            valid=reason == RejectReason.NONE,
            reason=reason,
            position_camera_m=position if reason == RejectReason.NONE else None,
            covariance_camera=covariance if reason == RejectReason.NONE else None,
            diagnostics=diagnostics,
        )
```

把核心实现为不依赖 ROS 的纯函数/普通类，可以用保存的数组快速测试。ROS 节点只负责消息转换、同步、TF 和发布，不应把几何逻辑散落在回调函数中。

## 第 7 章：合成测试建立几何真值

### 7.1 生成一个理想球形目标

```python
def synthetic_bundle(width=640, height=480, center=(0.1, -0.05, 1.2),
                     radius_px=35):
    K = np.array([[600., 0., width / 2],
                  [0., 600., height / 2],
                  [0., 0., 1.]])
    camera = CameraModel(width, height, K, np.zeros(5),
                         "camera_optical_frame", "synthetic-v1")
    rgb = np.zeros((height, width, 3), np.uint8)
    depth = np.full((height, width), np.nan, np.float32)
    x, y, z = center
    u0 = int(round(K[0, 0] * x / z + K[0, 2]))
    v0 = int(round(K[1, 1] * y / z + K[1, 2]))
    yy, xx = np.indices((height, width))
    mask = (xx - u0) ** 2 + (yy - v0) ** 2 <= radius_px ** 2
    depth[mask] = z
    rgb[mask] = [220, 30, 30]
    return FrameBundle(rgb, depth, 1_000_000_000, camera), mask
```

理想平面深度的反投影点均位于 $Z=z$，其均值 $X,Y$ 应接近设定中心。因为圆形像素掩码、取整和透视，不能要求逐位相同，应根据像素角分辨率推导合理容差。

### 7.2 完整测试例

```python
class FixedPredictor:
    def __init__(self, mask, score=0.99):
        self.mask = mask
        self.score = score

    def __call__(self, image):
        return [InstancePrediction(0, self.score, self.mask.copy())]

def test_pipeline_recovers_synthetic_center(config, thresholds):
    bundle, mask = synthetic_bundle()
    pipeline = RgbdPositionPipeline(FixedPredictor(mask), config, thresholds)
    result = pipeline.process(bundle)[0]
    assert result.valid, result
    np.testing.assert_allclose(
        result.position_camera_m,
        np.array([0.1, -0.05, 1.2]),
        atol=0.005,
    )

def test_background_contamination_is_trimmed(config, thresholds):
    bundle, mask = synthetic_bundle()
    corrupted = bundle.depth_m.copy()
    boundary = mask & ~erode_instance_mask(mask, 3)
    corrupted[boundary] = 2.4
    noisy_bundle = FrameBundle(
        bundle.rgb, corrupted, bundle.timestamp_ns, bundle.camera
    )
    result = RgbdPositionPipeline(
        FixedPredictor(mask), config, thresholds
    ).process(noisy_bundle)[0]
    assert result.valid
    assert abs(result.position_camera_m[2] - 1.2) < 0.01

def test_depth_hole_is_rejected(config, thresholds):
    bundle, mask = synthetic_bundle()
    empty_depth = np.full_like(bundle.depth_m, np.nan)
    invalid_bundle = FrameBundle(
        bundle.rgb, empty_depth, bundle.timestamp_ns, bundle.camera
    )
    result = RgbdPositionPipeline(
        FixedPredictor(mask), config, thresholds
    ).process(invalid_bundle)[0]
    assert not result.valid
    assert result.reason == RejectReason.INSUFFICIENT_DEPTH
```

### 7.3 属性测试思路

随机生成合法深度和内参，验证以下不变量：所有有效点有限且 $Z>0$；投影反投影近似互逆；深度整体乘 $a$ 时三维位置整体乘 $a$；图像与主点同时平移时反投影射线保持；刚体变换后两点距离不变；协方差矩阵对称半正定。

属性测试比手写少量例子更容易发现符号、广播和边界错误，但生成器必须限制在物理合法域。

## 第 8 章：真实数据集设计

### 8.1 六个变化轴

每个对象至少覆盖：距离 0.3/0.6/1.0/1.5 m；画面中心、四边和四角；正面、侧面、俯视等视角；正常、弱光、逆光；无遮挡、25%、50% 遮挡；单物体、相邻同类和杂乱背景。

数据量不是简单相乘后机械拍满，而是确保测试集中包含关键组合。训练集可广覆盖，验证集用于阈值，测试集保留未见背景、日期和物体实例。每个失败类别还应建立挑战集，但挑战集指标不能替代自然分布指标。

### 8.2 真值建立

相机坐标位置可用标定板固定目标、精密移动台或已知机械结构建立。base 坐标真值还包含手眼外参和机器人状态误差。对 6D 位姿，使用高精度标记系统时要测量 tag-to-object 变换，并把它的不确定性计入真值质量。

不能用同一个 PnP 算法产生标签再评价 PnP，否则共享偏差会让结果虚高。真值链必须尽量独立。

### 8.3 数据版本

清单记录每个文件哈希、采集设备、序列、时间、场景、对象实例、标注版本、相机参数和划分。修改标注后产生新版本，不覆盖旧版本。Git 保存清单和小样例，大图像通过对象存储或 DVC 管理。

## 第 9 章：训练、离线推理与阈值选择

### 9.1 三阶段训练诊断

第一阶段只取 5 至 20 个样本，关闭强增强并过拟合。若无法接近完美，检查类别映射、掩码插值、损失和模型输出。第二阶段用完整训练集观察训练/验证曲线，确认过拟合和欠拟合。第三阶段才调增强、预训练、分辨率和模型规模。

训练中每个 epoch 保存按类别 IoU/AP、按目标尺寸指标、学习率、各损失、梯度范数和吞吐。至少运行三个随机种子，报告均值和标准差。单次最佳结果不能说明稳定性。

### 9.2 阈值联合选择

检测分数、掩码面积、有效深度率、MAD 和空间范围共同影响输出。逐个独立调阈值可能得到次优组合。可先用物理边界固定明显无效条件，再在验证集上网格或贝叶斯搜索剩余阈值，以错误接受成本、主动拒绝成本和延迟组成目标。

阈值确定后冻结，在测试集只运行一次正式报告。看到测试失败后再调阈值必须记为新开发轮次，并重新建立保留集。

### 9.3 离线回放接口

在线与离线必须调用同一个 `RgbdPositionPipeline`。离线脚本逐帧读取原始 bundle，保存每个实例的结果和 diagnostics 为 JSONL。这样可以比较两个 commit 在完全相同输入上的逐帧差异，而不是只对比汇总指标。

## 第 10 章：指标实现与置信区间

### 10.1 位置误差切片

```python
def position_metrics(predicted, target):
    predicted = np.asarray(predicted, np.float64)
    target = np.asarray(target, np.float64)
    if predicted.shape != target.shape or predicted.ndim != 2 or predicted.shape[1] != 3:
        raise ValueError("expected matching [N,3] arrays")
    error_vector = predicted - target
    distance = np.linalg.norm(error_vector, axis=1)
    return {
        "count": len(distance),
        "mean_m": float(distance.mean()),
        "median_m": float(np.median(distance)),
        "p95_m": float(np.quantile(distance, 0.95)),
        "maximum_m": float(distance.max()),
        "bias_xyz_m": error_vector.mean(axis=0).tolist(),
        "rmse_xyz_m": np.sqrt(np.mean(error_vector ** 2, axis=0)).tolist(),
    }
```

三维距离给总体大小，XYZ 偏差帮助发现固定外参或轴方向问题。按距离、画面位置、物体、遮挡和光照切片后再运行同一函数。

### 10.2 Bootstrap 置信区间

相邻视频帧不独立，不能逐帧 bootstrap。应以采集序列或 episode 为重采样单位，重复抽取序列并计算指标分布。测试集只有一条长视频时，百万帧也不能代表丰富场景，置信区间会产生虚假精确。

### 10.3 拒绝系统的评价

把样本按真实误差是否安全、门禁是否接受形成四格表：正确接受、错误接受、正确拒绝、错误拒绝。随质量阈值变化画 risk-coverage 曲线：coverage 是接受比例，risk 是被接受样本中的错误率。理想门禁在覆盖下降时显著降低风险。

## 第 11 章：ROS2 适配层设计

### 11.1 节点职责

ROS2 节点负责参数、生命周期、订阅、同步、消息转换、TF、发布和诊断。模型和几何核心不依赖 `rclpy`/`rclcpp`。建议组件：

```text
rgbd_pose_node
  subscriptions:
    /camera/color/image_raw
    /camera/aligned_depth_to_color/image_raw
    /camera/color/camera_info
  publications:
    /perception/instances
    /perception/poses
    /perception/markers
    /diagnostics
  tf:
    camera_optical_frame -> base_link at sample timestamp
```

### 11.2 同步验证

近似同步回调仍应计算三条消息时间戳最大值减最小值，超过配置就拒绝。CameraInfo 如果低频或 latched，要确认其分辨率与图像一致，并检查 calibration ID。深度编码 `16UC1` 与 `32FC1` 使用不同转换路径，进入核心前统一为米。

### 11.3 变换位置与协方差

相机到基座变换 ${}^bT_c=[R,t]$ 作用于位置：

$$
{}^bp=R{}^cp+t.
$$

忽略外参自身不确定性时，位置协方差变换为

$$
{}^b\Sigma_p=R{}^c\Sigma_pR^T.
$$

若外参误差不可忽略，需要使用位姿扰动雅可比联合传播，而不是只旋转 3×3 协方差。

### 11.4 QoS 与队列

高频图像通常使用 sensor-data QoS/best effort，深度小队列避免积压；但所有同步话题 QoS 必须兼容。推理速度低于输入时保留最新帧，记录丢弃数。绝不能因为“不能丢数据”使用无限队列，最终让机器人消费数秒前位置。

### 11.5 生命周期

`configure` 加载并校验权重、类别、相机配置；`activate` 才启动订阅和发布；异常进入 error 或 inactive 并发布诊断。模型预热后再宣告 ready，避免第一帧包含图编译和 CUDA 初始化导致超时。

## 第 12 章：性能基准与 RTX 5090 使用

### 12.1 分段计时

使用统一时钟记录：消息同步等待、解码、resize/normalize、主机到 GPU、模型、后处理、掩码深度、TF 查询、序列化和总延迟。CUDA 异步执行，精确 kernel 计时使用 CUDA Event；端到端计时则从消息采样时间到发布时间。

### 12.2 预热和统计

前 20 至 100 次推理可能触发内存分配、算法选择和编译，不计入稳定统计但要单独报告冷启动。至少测数千帧，报告 P50/P95/P99 和最大值；平均值不能揭示偶发卡顿。

### 12.3 吞吐与延迟不是一回事

增大 batch 能提高每秒图像数，却会等待凑批并增加单帧延迟。实时机器人通常 batch=1。离线数据标注或回放才适合大 batch。5090 算力富余时优先提高输入分辨率、模型鲁棒性或保留延迟余量，不必为了满载而人为增加工作。

### 12.4 显存和稳定性

连续记录分配显存、保留显存和系统显存。若每帧增长，检查是否保存带计算图张量、日志列表是否无界、可视化是否缓存 GPU 数据。进行 4 小时压力测试，同时记录温度、功耗、频率和错误帧。

## 第 13 章：跨模块故障树

### 顶层事件 A：位置整体偏移但很稳定

优先怀疑系统偏差：相机内参分辨率不匹配；RGB-D 外参错误；深度 scale 错；camera-to-base 手眼外参错误；TF 方向或时间错误；目标真值坐标系定义错误。

二分实验：先在 camera frame 与独立相机坐标真值比较。若相机坐标已偏，问题在成像、深度和局部定位；若相机坐标准而 base 偏，问题在 TF/手眼/机器人状态。不要一开始重训模型。

### 顶层事件 B：静态准确，运动时偏移

优先怀疑 RGB-depth 不同步、TF 查询时刻、滚动快门、扫描去畸变和队列延迟。画误差对机器人速度、角速度和消息年龄的曲线。误差随速度变大而静态消失，是时序问题的强证据。

### 顶层事件 C：只在画面边缘偏移

检查畸变模型、去畸变是否重复、对齐深度使用的内参、resize/crop 后主点，以及标定数据是否覆盖边缘。让固定标定板遍历九宫格，画残差矢量场；系统性径向模式通常不是随机网络误差。

### 顶层事件 D：只在物体边缘或遮挡时跳动

检查掩码边界、混合深度、腐蚀尺度、深度连通分量和遮挡标注。分别用真值掩码与预测掩码运行同一几何后处理：真值正常说明学习分割是主因，两者都失败说明深度或几何策略不适用。

### 顶层事件 E：离线正常，ROS2 在线失败

固定同一帧保存在线输入数组，与离线逐元素比较；检查颜色顺序、编码、stride、深度 scale、CameraInfo、时间戳和模型运行模式。用 rosbag 走完整在线节点路径，若复现则逐层记录中间哈希。在线/离线共用核心函数能显著缩小差异范围。

### 顶层事件 F：偶发发布极端位置

检查 `NaN/Inf`、空掩码除零、未初始化内存、错误实例关联、TF 外推和旧结果复用。所有输出进入门禁前做有限性和工作空间检查；异常帧保存最小证据包，不允许用平滑把极端值掩盖后继续发布有效状态。

## 第 14 章：故障注入实验

### 14.1 数据级注入

对同一测试集分别注入：随机深度孔洞；物体边缘背景深度；深度整体乘 1000；RGB 平移若干像素；曝光和运动模糊；掩码侵蚀/膨胀；时间戳偏移。每次只注入一种故障并控制强度，画质量门禁、位置误差和拒绝率曲线。

### 14.2 几何级注入

扰动 $f_x,f_y,c_x,c_y$、畸变、RGB-D 外参和 camera-to-base 外参。观察误差空间模式：焦距错误随离光轴角度增长；主点错误产生方向性偏差；外参旋转误差随距离放大；平移误差近似形成固定偏置。

### 14.3 系统级注入

降低推理速度、随机阻塞 TF、让图像短时断流、模拟时钟回退、替换错误模型版本、让 GPU 内存不足。系统应进入已定义状态，持续发布诊断，并禁止陈旧或未经验证的位姿驱动机器人。

### 14.4 通过标准

故障注入不是要求系统在所有错误下仍给正确位置，而是要求它在无法保证正确时可靠拒绝、给出可诊断原因并安全恢复。恢复策略要测试：故障消失后是否需要连续若干有效帧才重新激活，是否清除旧跟踪状态，是否发生瞬间跳变。

## 第 15 章：从离线到实机的七级验收

### 第 0 级：单元与合成几何

所有数据契约、投影反投影、掩码深度、质心、协方差、坐标变换和门禁测试通过。合成真值覆盖正常、边界和退化。

### 第 1 级：独立静态数据集

冻结模型和阈值，在保留测试集报告二维、三维、切片、拒绝与性能指标。不能只展示成功图片。

### 第 2 级：rosbag 完整回放

使用与在线相同节点、参数和 TF 回放，输出与离线核心逐帧一致。检查消息年龄、QoS、队列和可复现性。

### 第 3 级：固定相机、固定物体

连续运行并测抖动、偏差和温漂。改变物体在工作空间位置，验证九宫格和多距离误差。

### 第 4 级：固定相机、移动物体或机械臂

逐级增加速度，验证同步和延迟。此阶段机器人末端与物体保持安全距离。

### 第 5 级：高处指向和空抓

机械臂移动到目标上方安全高度，比较末端位置与独立测量；执行不闭合夹爪的空抓轨迹，检查路径和碰撞。

### 第 6 级：低速真实抓取

保护人员和急停到位，限制速度、力和工作空间。按物体、位置、遮挡重复统计成功率。失败立即保存完整证据包。

任一级失败都回到最近通过级，不跨级用实机试错。模型新版本必须重新完成受影响的等级。

## 第 16 章：实验记录模板

每次实验建立唯一目录：

```text
outputs/20260811-153000-exp042/
  resolved_config.yaml
  environment.json
  git.diff
  model.sha256
  calibration.json
  metrics.json
  predictions.jsonl
  latency.csv
  failures/
  plots/
  report.md
```

`report.md` 回答：本次假设；唯一主动改变；控制变量；数据版本；通过门槛；主要指标；最差五个序列；反例；是否接受假设；下一次最小实验。没有记录控制变量的对比不能支持因果结论。

### 16.1 失败证据包

每个失败保存原始 RGB/深度、相机参数、预测掩码、有效深度图、三维点、TF、输入/输出时间戳、中间统计、模型版本和拒绝码。若只保存最终截图，许多单位、时序和坐标问题无法复盘。

## 第 17 章：代码审查清单

### 数据和接口

- 所有数组 shape、dtype、单位、颜色和坐标系是否明确？
- 输入是否检查有限性、尺寸和时间戳？
- 原始数据是否保持不可变？
- 模型类别表和预处理是否随权重版本化？

### 几何

- 变换命名是否包含目标/源坐标系？
- 点使用左乘还是右乘，是否全项目一致？
- 去畸变图像是否使用匹配的新内参？
- 深度是 $Z$ 还是 range，scale 是否明确？
- 旋转矩阵是否验证正交和行列式？

### 学习推理

- `eval()` 与 inference mode 是否启用？
- resize、letterbox 和掩码回映射是否经过合成测试？
- NMS 是否按类别处理？
- 空检测、空掩码和未知类别是否有定义？

### 实时与安全

- 使用采样时间还是处理完成时间查询 TF？
- 队列积压时策略是什么？
- 过期、越界和跳变能否被拒绝？
- 无效结果是否可能被消费者误当成零位姿？
- 日志是否限频且关键计数持续可见？

## 第 18 章：全册结业考试

建议闭卷理论 180 分钟，加上两天编程实践。总分 150 分，理论与实践均达到 70% 才通过。

### 一、相机与标定，共 20 分

1. 从针孔模型推导三维点到像素的过程，并解释单目尺度不确定性。（4 分）
2. 图像先从左裁 80 px、上裁 40 px，再水平缩放 0.5、垂直缩放 0.75，内参如何更新？（4 分）
3. 为什么只使用正对棋盘且集中在中心的标定图会退化？（4 分）
4. 重投影 RMS 很低为什么不能证明标定适合三维测量？（4 分）
5. 设计一个区分内参错误和手眼外参错误的实验。（4 分）

### 二、特征与多视图，共 20 分

1. 用结构张量解释角点比边缘适合 LK 光流。（4 分）
2. ORB 与 SIFT 描述子分别使用什么距离？为何不能混用？（4 分）
3. 写出 RANSAC 迭代次数公式并说明内点率的影响。（4 分）
4. 单应矩阵内点更多时为什么不能直接断言场景为平面？（4 分）
5. 两视图初始化需要检查哪些退化条件？（4 分）

### 三、深度与点云，共 20 分

1. 比较轴向深度和沿射线 range。（4 分）
2. 体素和法线半径如何依据任务尺度选择？（4 分）
3. 推导 SVD 刚体配准并说明反射修正。（4 分）
4. 点到平面 ICP 在单一墙面中哪些方向弱可观？（4 分）
5. `fitness=0.95` 为什么不能单独证明配准正确？（4 分）

### 四、检测、分割与位姿，共 20 分

1. 解释视频帧随机划分的数据泄漏。（4 分）
2. 检测 mAP 与机器人抓取成功率之间缺少哪些环节？（4 分）
3. 比较关键点 PnP、稠密对象坐标和 RGB-D 配准路线。（4 分）
4. 对称物体的损失和评价为什么要考虑等价姿态？（4 分）
5. 设计包含错误接受率的质量门禁评价。（4 分）

### 五、系统诊断，共 20 分

1. 静态准确、运动偏移时的排查顺序是什么？（5 分）
2. 只在图像边缘偏差时应优先检查什么？（5 分）
3. 推理 10 ms、端到端 150 ms 时如何分段定位？（5 分）
4. 说明为什么滤波不能修复外参偏差和消息延迟。（5 分）

### 六、编程实践，共 50 分

1. 实现并测试从深度图到点云、刚体变换和投影回图像的往返管线。（15 分）
2. 实现本章 RGB-D 三维中心管线，覆盖空掩码、孔洞、背景污染、越界和过期输入。（15 分）
3. 在一组真实或公开 RGB-D 数据上完成检测/分割、三维定位和指标报告。（10 分）
4. 提交可复现环境、配置、测试、失败案例、性能基准和 5 分钟演示视频。（10 分）

## 第 19 章：理论考试参考答案

### 一、相机与标定

1. 相机点 $(X,Y,Z)$ 先除以 $Z$ 得归一化坐标，再乘焦距并加主点得到 $(u,v)$。$(X,Y,Z)$ 与任意正比例缩放的点具有相同 $X/Z,Y/Z$，所以单目只确定射线和相对结构，绝对尺度需要外部信息。

2. 裁剪后 $c_x'=c_x-80,c_y'=c_y-40$，焦距不变；再缩放得 $f_x''=0.5f_x,f_y''=0.75f_y,c_x''=0.5(c_x-80),c_y''=0.75(c_y-40)$。skew 若存在也应按对应行列变换。

3. 近似相同姿态提供高度相关的单应约束，焦距、距离和畸变可能耦合；中心区域也无法约束边缘畸变。需要不同距离、位置以及绕两个轴的倾角。

4. RMS 只衡量参数对标定图像角点的解释，可被错误板尺寸、弯曲、退化姿态或过复杂畸变模型掩盖。需要独立图像、参数稳定性、直线和已知三维尺寸验证。

5. 先在 camera frame 用独立真值比较。如果相机坐标位置正确而转换到 base 后错误，手眼/TF 是主因；如果 camera frame 已错，检查内参、深度和局部算法。固定相机移动目标九宫格可观察内参空间模式，固定目标改变机器人姿态可观察外参一致性。

### 二、特征与多视图

1. LK 正规矩阵是结构张量。角点两个特征值都大，二维位移都受约束；边缘只有法向变化明显，沿边缘运动不可观，即孔径问题。

2. ORB 是二进制描述子，用 Hamming；SIFT 是浮点梯度直方图，用 L2。错误距离破坏描述子空间的相似性排序，即使接口仍可运行。

3. $N\ge\log(1-p)/\log(1-w^s)$。内点率 $w$ 下降或最小样本数 $s$ 增大时，全内点抽样概率 $w^s$ 快速下降，所需迭代暴增。

4. 纯旋转对任意深度场景也可由单应解释，远景和低视差也近似单应；模型评分还受阈值和复杂度影响。必须结合视差、三角化和运动判断。

5. 检查内点数量与空间覆盖、视差/射线夹角、正深度率、重投影误差、单应与本质模型竞争、点是否近共面、运动是否近纯旋转和结果是否符合时序先验。

### 三、深度与点云

1. $Z$ 是光轴方向坐标，range 是光心到点的欧氏距离。单位射线 $r$ 下 $P=\rho r$、$Z=\rho r_z$。二者在光轴上相等，图像边缘差异增大。

2. 体素必须小于任务要保留的最小结构，同时不必远小于传感噪声。法线半径通常是体素的数倍并确保足够邻居；应通过不同距离和边缘区域验证，不存在通用固定值。

3. 两组对应点去质心后构造 $H=\sum p_iq_i^T$，SVD 为 $U\Sigma V^T$，$R=V\operatorname{diag}(1,1,\det(VU^T))U^T$，$t=\bar q-R\bar p$。行列式项阻止得到镜像反射。

4. 单一无限平面主要约束法向平移和绕平面内轴的旋转；沿平面两个方向平移以及绕法线旋转弱可观或不可观，具体程度受有限边界和采样影响。

5. fitness 依赖对应阈值、裁剪和重叠，错误重复结构也能产生大量近邻。需要真值姿态误差、成功率、初值扰动、残差分布、重叠、物理先验和失败案例。

### 四、检测、分割与位姿

1. 相邻帧共享几乎相同的目标、背景和噪声，随机划分把近重复帧放入训练与测试，指标高估新序列泛化。应按序列、场景或实例隔离。

2. 中间还包含掩码边界、深度有效性、相机内参、RGB-D 对齐、PnP/配准、手眼外参、时间同步、质量拒绝、规划和控制。mAP 只评价二维检测的一部分。

3. 关键点 PnP 几何清晰但依赖稳定语义点；稠密对象坐标提供大量对应、抗局部遮挡但标签和对称性复杂；RGB-D 粗到精提供米制表面约束，但受深度孔洞、掩码污染和初值影响。

4. 多个旋转可能产生相同外观或任务状态，单一任意标签形成冲突监督，普通误差会惩罚正确等价解。应对对称群取最小误差、预测规范不变量或输出多假设。

5. 用独立真值把样本分为安全/不安全，再与门禁接受/拒绝组成四格表。报告 coverage、被接受样本 risk、错误接受和错误拒绝，随阈值画 risk-coverage 曲线，并按困难场景切片。

### 五、系统诊断

1. 先量化误差对速度和消息年龄关系；核对 RGB-depth 时间差、采样时间戳、TF 查询时刻、队列积压；再检查滚动快门和扫描运动畸变。静态正常使固定内参偏差可能性降低，但仍应保留对照。

2. 优先检查畸变模型和边缘标定覆盖、图像是否重复去畸变、resize/crop 后主点、aligned depth 使用的内参。用九宫格残差矢量场判断系统模式。

3. 从采样到发布分段记录同步等待、解码、预处理、传输、模型、NMS、点云、TF 和队列。CUDA kernel 用 Event 并同步，端到端用消息时间戳。比较 P50/P95 找到积压或长尾，而不是只看平均网络时间。

4. 滤波只对随机抖动有效；固定外参偏差经平滑仍是偏差，消息延迟经平滑通常更滞后。应先校准坐标和修复时序，再依据控制带宽选择滤波。

## 第 20 章：实践题评分标准

### 20.1 几何管线 15 分

输入契约和非法值 3 分；投影/反投影实现与单位 4 分；刚体变换方向和 $SE(3)$ 检查 4 分；合成往返测试、误差曲线和退化测试 4 分。只有正常样例截图最多得 7 分。

### 20.2 RGB-D 管线 15 分

掩码与深度稳健处理 4 分；质量门禁和标准化拒绝原因 4 分；坐标/时间语义 3 分；五类指定失败测试 4 分。用框中心单像素深度且无拒绝机制不超过 6 分。

### 20.3 真实数据实验 10 分

数据划分和真值 2 分；二维与三维组合指标 3 分；距离/位置/遮挡切片 2 分；失败分析和反例 2 分；明确限制 1 分。

### 20.4 工程交付 10 分

一键环境和锁定依赖 2 分；配置/版本/命令可复现 2 分；自动测试 2 分；端到端性能和压力测试 2 分；演示同时展示正常结果和安全拒绝 2 分。

## 第 21 章：第二册毕业答辩问题

答辩时不展示幻灯片堆砌，直接打开代码、测试、失败样本和指标。应能现场回答：

1. 当前深度值是 $Z$ 还是 range，为什么？
2. 把图像缩放后哪几个内参需要改变？
3. 最差的三个序列是什么，根因证据是什么？
4. 为什么门禁阈值选在当前位置，而不是更高或更低？
5. 相机坐标误差和 base 坐标误差各有多少？差异说明什么？
6. RGB 与深度不同步 20 ms 在最大速度下产生多大误差？
7. 当前模型遇到透明物体怎样失败，系统如何降级？
8. 对称物体姿态输出的物理含义是什么？
9. 5090 上网络时间和端到端 P95 分别是多少？
10. 随机删除一半有效深度后，系统何时拒绝？
11. 更换相机分辨率需要重新验证哪些模块？
12. 如何证明线上代码和离线评估走的是同一路径？

能够用具体配置、曲线、代码和反例回答，才说明掌握了系统；只背概念不能通过。

## 第 22 章：第二册完成清单

### 理论

- 能推导针孔投影、反投影、对极约束和双目深度误差。
- 能解释畸变、标定可观性、结构张量、光流和 RANSAC。
- 能推导 PCA 法线、SVD 刚体配准和点到平面 ICP 线性化。
- 能解释检测、分割、单目深度、PnP、对称性和位姿指标。

### 代码

- 投影反投影、内参调整和变换方向有自动测试。
- 特征/光流/几何验证可在真实序列复现。
- 深度到点云、法线、分割和配准有合成真值测试。
- RGB-D 实例定位具有数据契约、质量门禁和拒绝原因。

### 实验

- 完成真实相机标定与独立验证。
- 完成传统特征在六类场景的基准。
- 完成点云配准成功域和初值扰动实验。
- 完成 RGB-D 位姿离线、rosbag 和低速实机分级验收。

### 工程

- 数据、模型、相机和外参均有版本。
- 训练和推理能够由固定命令复现。
- 报告包含均值、P95、最大值、切片和失败案例。
- 系统对过期、越界、无深度和退化结果主动拒绝。
- 能在不依赖原作者口头说明的情况下由另一人复现实验。

当这些项目全部有证据支持时，第二册才算完成。学习者此时不只是“会使用 OpenCV 和检测模型”，而是能够建立一个从光学测量到机器人坐标、从网络输出到安全动作之间闭环可验证的三维感知系统。
