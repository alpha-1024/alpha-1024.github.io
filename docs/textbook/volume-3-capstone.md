---
title: 第三册结业实训：SLAM、Nav2 与自动化导航验收
description: 将传感器融合、SLAM、地图、AMCL、规划控制和行为树整合为可复现系统，包含故障树、考试和答案。
---

# 第三册结业实训：SLAM、Nav2 与自动化导航验收

本实训构建一个完整移动机器人系统：轮速和 IMU 形成连续里程计；激光 SLAM 在建图模式产生全局一致地图；导航模式加载版本化地图并由 AMCL 定位；Nav2 规划和跟踪路径；自动测试器批量发送目标、测量安全与效率，并在失败时保存可回放证据。

项目的完成标准不是“RViz 中点一下能走”。必须证明坐标和时间正确、里程计一致、地图可量化、定位可恢复、路径安全、控制可执行、失败可归因、参数和数据可复现。单次成功是调试起点，不是验收结论。

## 第 1 章：系统目标和模式边界

### 1.1 建图模式

数据流：

```text
encoders + IMU -> local fusion -> odom -> base_link
laser + odom -> SLAM frontend/backend -> map -> odom
optimized poses + scans -> occupancy map -> map artifact
```

输出地图图像/YAML、轨迹、位姿图、参数、传感器外参、rosbag 和质量报告。建图期间 Nav2 可用于遥控辅助或探索，但最终地图必须在固定版本上验收。

### 1.2 导航模式

```text
map artifact -> map_server
laser + odom + map -> AMCL -> map -> odom
goal + map/costmaps -> planner/controller -> cmd_vel
cmd_vel -> base driver -> encoders -> feedback
```

SLAM 与 AMCL 不应同时发布同一个 `map -> odom`。切换模式时通过 launch 配置明确唯一发布者，启动测试自动检查 TF authority。

### 1.3 示例验收指标

| 层级 | 指标 | 目标示例 |
| --- | --- | ---: |
| 里程计 | 10 m 直线尺度误差 | < 2% |
| 里程计 | 原地 360° 航向误差 | < 3° |
| 地图 | 已知墙距误差 P95 | < 5 cm |
| 定位 | 静态位置误差 P95 | < 5 cm |
| 定位 | 绑架后恢复 P95 | < 15 s |
| 导航 | 50 组任务成功率 | >= 95% |
| 安全 | 最小净空 | >= 项目门槛 |
| 控制 | 横向误差 P95 | < 10 cm |
| 时序 | 目标到 cmd_vel P95 | < 100 ms |
| 稳定 | 连续运行 | >= 4 h |

数值需按机器人尺寸、传感器和场地调整，但必须在实验前冻结。

## 第 2 章：工程仓库

```text
mobile_robot_nav/
  README.md
  docker/
  maps/
    warehouse_v1/
      map.yaml
      map.pgm
      metadata.json
  params/
    sensors.yaml
    ekf.yaml
    slam.yaml
    amcl.yaml
    nav2.yaml
  src/
    robot_bringup/
    robot_description/
    wheel_imu_fusion/
    navigation_benchmark/
    navigation_diagnostics/
  test/
    unit/
    launch/
    bags/
  scripts/
    validate_tf.py
    inspect_bag.py
    evaluate_map.py
    summarize_trials.py
  outputs/
```

地图是版本化制品，不用含糊的 `map_final2.pgm`。`metadata.json` 保存地图 ID、commit、建图算法与参数哈希、分辨率、采集 bag、相机/激光外参 ID、创建时间、测试区域和已知限制。

## 第 3 章：TF 合同

### 3.1 唯一发布者表

| Transform | 建图模式 | 导航模式 | 性质 |
| --- | --- | --- | --- |
| `map -> odom` | SLAM | AMCL | 全局修正，可跳变 |
| `odom -> base_link` | EKF/里程计 | EKF/里程计 | 连续，可漂移 |
| `base_link -> laser` | static TF | static TF | 标定外参 |
| `base_link -> imu` | static TF | static TF | 标定外参 |

同一 transform 有两个发布者时，TF 会来回跳。静态外参必须由一个来源发布，URDF 和独立 static publisher 不重复。

### 3.2 自动检查

启动测试等待所有节点 active 后，查询 TF 链和 authority，检查：图无环；必要 frame 可达；时间戳新鲜；静态边数值与标定一致；平移单位为米；旋转四元数归一；`map -> odom` 只有一个来源。

### 3.3 已知运动测试

机器人向前 1 m，`base_link` 的 x 应在 odom 中增加；逆时针旋转 90°，yaw 应为正。这个最小物理测试能发现编码器符号、左右轮交换和坐标手性。

## 第 4 章：传感器启动门禁

系统进入 active 前验证：消息频率在范围；时间戳单调；消息年龄低于门槛；LaserScan 角度/量程合法；IMU 单位和轴符合 REP-103；轮速不超物理范围；协方差有限且非零；所有 sensor frame 可变换到 base。

```python
from dataclasses import dataclass

@dataclass(frozen=True)
class StreamHealth:
    name: str
    frequency_hz: float
    age_ms: float
    timestamp_regressions: int
    invalid_ratio: float

def stream_is_ready(health, minimum_hz, maximum_age_ms,
                    maximum_invalid_ratio):
    reasons = []
    if health.frequency_hz < minimum_hz:
        reasons.append("frequency_low")
    if health.age_ms > maximum_age_ms:
        reasons.append("message_stale")
    if health.timestamp_regressions:
        reasons.append("timestamp_regression")
    if health.invalid_ratio > maximum_invalid_ratio:
        reasons.append("too_many_invalid_samples")
    return len(reasons) == 0, reasons
```

门禁失败保持底盘停止并发布 diagnostics。不能为了启动成功而把超时无限放宽。

## 第 5 章：轮速与 IMU 融合配置

配置前制作“状态来源矩阵”，避免重复融合。示例：轮速只提供 body x 速度和 yaw rate；IMU 提供 yaw rate 和 roll/pitch；SLAM/AMCL 不进入本地 odom EKF，而发布 map 修正。

```yaml
frequency: 50.0
sensor_timeout: 0.1
two_d_mode: true
publish_tf: true
world_frame: odom
odom_frame: odom
base_link_frame: base_link

odom0: /wheel/odometry
odom0_config: [false, false, false,
               false, false, false,
               true,  false, false,
               false, false, true,
               false, false, false]

imu0: /imu/data
imu0_config: [false, false, false,
              true,  true,  false,
              false, false, false,
              false, false, true,
              false, false, false]
imu0_remove_gravitational_acceleration: true
```

配置数组字段顺序必须对照当前 `robot_localization` 文档确认。本例表达方法，不应不经检查直接部署。若 IMU orientation 由内部滤波且已使用角速度，再同时融合它们可能相关。

### 5.1 单传感器到联合融合

先只运行轮速，验证直线/旋转尺度和协方差；再只检查 IMU 静止、轴和偏置；最后联合。每增加一个来源保存同一 bag 的轨迹、创新代理、终点误差和消息年龄。

## 第 6 章：建图数据采集协议

采集路线覆盖外墙、内部通道、独特结构和多个闭环。速度保持在传感器去畸变与前端捕获范围内，避免长时间只沿单方向。开始与结束在可测标记处静止，便于漂移评估。

保存 `/scan`、`/imu`、轮速、`/tf`、`/tf_static`、时钟和诊断。录包前验证磁盘吞吐；丢消息会形成不连续约束。记录场地动态人员、玻璃、门状态和临时物体。

### 6.1 采集矩阵

- 顺时针/逆时针各一圈；
- 正向和反向通过窄通道；
- 低速与目标运行速度；
- 正常人流和静态场景；
- 冷启动与热稳态 IMU；
- 至少三个空间独立闭环。

不要用建图参数调试所用的同一 bag 作为最终测试 bag。

## 第 7 章：SLAM 参数实验

每轮只改变一组：扫描匹配搜索窗/阈值、关键帧距离角度、子图大小、回环阈值、占据分辨率。输出 ATE/RPE（有真值时）、闭环误差、墙厚、已知距离、CPU、内存和回环接受/拒绝。

### 7.1 回环审计

保存每个候选的源/目标关键帧、描述分数、几何内点、残差、信息矩阵、激活时间和优化前后影响。错误回环必须能从日志回到原始 scan/image。

### 7.2 地图选择

最漂亮的地图不一定最好。用冻结指标比较候选：拓扑是否正确、通道宽度、障碍重影、动态残留、AMCL 定位成功和 Nav2 任务结果。选定后生成不可变地图 ID。

## 第 8 章：地图验收工具

### 8.1 已知点测距

人工或自动标记地图中若干墙角/标志，转换到世界坐标，与卷尺或 CAD 距离比较。地图缩放错误常来自轮速尺度、激光单位或优化规范。

### 8.2 墙厚

对多个截面统计占据带宽。墙厚随机器人速度增长提示运动畸变/延迟；随离激光距离增长可能来自角分辨率和噪声；闭环区域双层墙提示轨迹不一致。

### 8.3 导航可行性

用实际 footprint 和安全 inflation 在地图上运行连通性检查。若关键区域被错误封闭或出现穿墙捷径，地图不通过。人工修补必须记录 diff 和物理依据。

## 第 9 章：导航模式启动序列

1. 启动底盘、传感器、静态 TF。
2. 启动本地里程计，等待健康。
3. 启动 map server，验证地图 ID。
4. 启动 AMCL，设置/恢复初始位姿。
5. 检查激光与地图叠加及定位质量。
6. 配置并激活 costmap、planner、controller。
7. 激活 BT navigator。
8. 安全管理器允许非零 `cmd_vel`。

任何步骤失败不继续。lifecycle manager 可自动化转换，但 readiness 条件仍需显式诊断。

## 第 10 章：AMCL 验收

### 10.1 初始位姿网格

在地图多个位置分别注入小误差、中等误差、错误朝向和全局未知。测收敛时间、最终误差、错误收敛和粒子数量。重复结构区域单独切片。

### 10.2 动态测试

低速/高速转弯、短时遮挡激光、轮速打滑、传感器延迟。观察粒子分布、激光似然、`map -> odom` 跳变和恢复。定位跳变期间控制层应降速或停止。

### 10.3 绑架

仿真瞬移或人工搬运机器人，确保安全。检测依据可以是持续低似然、外部运动不一致和激光地图错位。触发全局重定位，连续多帧通过几何一致性后才恢复任务。

## 第 11 章：Nav2 参数分层

### 11.1 几何层

footprint、robot radius、传感器高度、obstacle/raytrace range、inflation radius。先用静态场景验证，不进入 controller 权重。

### 11.2 运动层

最大/最小线角速度、加速度、减速度、速度死区、controller frequency。命令限制必须与底盘真实能力一致。

### 11.3 行为层

planner/controller plugin、critic 权重、progress/goal checker、BT 超时和恢复次数。前两层错误时不调行为层。

### 11.4 安全层

碰撞监控、速度区、消息超时、急停和工作空间。它独立于局部规划器，避免单点故障。

## 第 12 章：自动任务执行器

```python
from dataclasses import dataclass, asdict
from enum import Enum
import json
from pathlib import Path

class TrialState(str, Enum):
    RESETTING = "resetting"
    WAITING_LOCALIZATION = "waiting_localization"
    SENDING_GOAL = "sending_goal"
    NAVIGATING = "navigating"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    TIMED_OUT = "timed_out"

@dataclass
class TrialRecord:
    trial_id: str
    map_id: str
    config_id: str
    start: tuple[float, float, float]
    goal: tuple[float, float, float]
    state: str
    failure_reason: str
    duration_s: float
    path_length_m: float
    minimum_clearance_m: float
    recoveries: int
    replans: int
    final_position_error_m: float
    final_yaw_error_rad: float
    bag_path: str

def append_jsonl(path, record):
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(asdict(record), ensure_ascii=False) + "\n")
```

状态机每步有独立超时。reset 后等待 TF、传感器、定位和 costmap 达到门禁再发目标。成功也要检查最终误差和速度是否停稳，不能只信 action result。

## 第 13 章：指标采集

从 odom/TF 积分实际路径长度；从局部代价地图或传感器计算最小净空；记录 `cmd_vel` 和实际速度计算饱和、加速度和 jerk；统计 planner/controller action、recovery 和 costmap clear；记录定位协方差/粒子质量和消息年龄。

### 13.1 路径效率

$$
\eta_L=\frac{L_{executed}}{L_{reference}}.
$$

参考可以是同地图同 footprint 的全局最短安全路径。比值接近 1 不保证安全，必须同时报告净空和时间。

### 13.2 成功率区间

50 次成功 48 次不是精确“96% 能力”。使用 Wilson 或 bootstrap 给置信区间，并按场景分层。每类只有两次测试时，整体高成功率无法证明窄门性能。

## 第 14 章：失败证据包

失败发生时保存：前后 30 秒 rosbag、map/config/model ID、TF 快照、global/local costmap、全局路径、局部轨迹、cmd/actual velocity、AMCL 状态、BT 日志、diagnostics 和机器资源。文件名使用 trial ID，自动写 manifest 哈希。

环形 buffer 持续缓存最近数据，失败触发后补录一段后续数据再落盘。磁盘写入不得阻塞控制回调；使用独立线程并监控丢包。

## 第 15 章：跨模块故障树

### 顶层事件 A：机器人不动

先看是否有导航目标、BT 状态和 controller 输出；有 `cmd_vel` 再查安全 mux、急停、驱动和底盘反馈；无命令则查 lifecycle、路径、局部轨迹和 progress checker。不要一开始调加速度。

### 顶层事件 B：机器人位置在 RViz 跳动

区分 `odom -> base_link` 是否连续和 `map -> odom` 是否跳。前者异常查轮速/融合/时间，后者查 AMCL/SLAM 与地图匹配。检查同一 TF 多发布者。

### 顶层事件 C：路径穿墙

验证地图版本和 frame；world-grid 转换；障碍阈值和 unknown；footprint 碰撞回放；对角角切；地图更新竞态。若路径本身安全但机器人撞墙，转向定位/控制层。

### 顶层事件 D：路径安全但实机擦墙

比较实测轨迹与路径，测横向误差；检查 footprint、inflation、定位偏差、控制延迟、速度和底盘打滑。用真值定位或低速对照隔离定位和控制。

### 顶层事件 E：窄门前振荡

检查门是否在安全几何上可通；定位/局部 costmap 是否左右跳；全局路径是否居中；controller critic 和前视；速度死区；反复重规划是否切换左右同价路径。不可通应明确失败，而不是无限振荡。

### 顶层事件 F：动态障碍消失后仍堵塞

检查 ray clearing、最大量程、Inf 语义、TF 和观察源持久时间。比较原始 scan 与 costmap 更新；清图行为只能验证症状，不是永久修复。

### 顶层事件 G：终点附近旋转不停

检查目标 yaw、goal checker 容差、定位 yaw 抖动、最小角速度、底盘死区和 rotate-to-goal critic。记录命令与实测角速度，区分控制器持续要求和底盘不响应。

## 第 16 章：故障注入

### 16.1 传感器

注入 scan 丢帧、距离噪声、IMU 偏置、轮速尺度、时间延迟、TF 外参误差和时钟回退。系统应检测消息健康、扩大不确定性、降速或停止，并给出具体状态。

### 16.2 SLAM/定位

注入错误回环、重复走廊、地图偏移、错误初始位姿和绑架。评估错误接受、地图破坏、定位恢复和任务安全。

### 16.3 规划控制

注入封路、动态横穿、底盘速度死区、制动能力下降、controller 超时和 costmap 更新停滞。验证制动距离、安全监控、BT 恢复次数和最终失败语义。

### 16.4 系统

杀死关键节点、GPU/CPU 满载、磁盘写满、网络抖动和 rosbag 录制失败。导航不能在失去关键诊断/感知后继续高速运行。

## 第 17 章：七级验收

### 0 级：纯函数和单元测试

坐标、角度、差速积分、滤波、栅格、A*、Pure Pursuit 和指标代码通过正常/边界/退化。

### 1 级：组件 bag 回放

轮速/IMU、SLAM、AMCL、planner、controller 分别用固定 bag 或仿真真值评估。

### 2 级：完整离线仿真

固定地图运行至少 50 任务，故障注入可复现，成功率和安全指标达到门槛。

### 3 级：实机架空/低功率

验证轮子方向、速度、超时、急停和 TF，不允许自主移动。

### 4 级：空场低速

直线、旋转、圆弧和停止，验证里程计与控制跟踪。

### 5 级：静态障碍导航

逐级加入宽通道、转角和窄门；保护人员与急停到位。

### 6 级：动态与长时间

行人横穿、临时封路、绑架恢复和 4 小时压力；任何安全事件阻止发布。

新地图、外参、底盘固件或关键参数改变后，重跑受影响等级。

## 第 18 章：公平对比实验

比较两套 controller 或参数时固定：地图 ID、起终点集合、场景动态脚本、机器人模型、传感器噪声、最大速度和超时。交错运行 A/B，避免温度、电量和场地时间偏差。调参集和最终测试集隔离。

同时报告成功率、时间、长度、净空、跟踪误差、jerk、恢复、碰撞和资源。若新方案成功率提高但净空下降到危险范围，不是总体改进。

## 第 19 章：持续集成

### 19.1 每次提交

运行 Python/C++ 单元测试、参数 schema、URDF/TF 静态检查、launch smoke test 和小地图规划测试。构建失败或 TF 多发布者阻止合并。

### 19.2 每夜

运行固定仿真场景与 rosbag 回放，比较基线指标。设回归门槛：成功率下降、P95 延迟上升、净空下降、错误回环或错误接受都报警。

### 19.3 发布前

完整 50+ 任务、故障注入和压力测试，生成带 commit/map/config ID 的报告。发布包只包含通过验证的组合，不能任意混合参数。

## 第 20 章：运行手册

值班人员应能按文档完成：开机自检；切换建图/导航；选择地图；设置初始位姿；开始任务；识别 localization degraded；执行安全停止；保存失败包；恢复到上一版本。手册命令必须在干净机器验证。

任何自动恢复都有次数和时间上限。超过后保持停止并请求人工，而不是循环清图/旋转。

## 第 21 章：结业项目交付物

- ROS2 workspace 与锁定依赖；
- URDF、传感器外参和 TF 图；
- 轮速/IMU 标定报告；
- 原始与测试 rosbag 清单；
- 至少一个版本化地图及质量报告；
- SLAM 回环审计；
- AMCL 初始/绑架基准；
- Nav2 参数与行为树；
- 50+ 自动任务 JSONL/CSV；
- 三类失败证据包及复盘；
- 故障注入报告；
- 连续运行报告和演示视频；
- 一键启动、测试和回滚命令。

## 第 22 章：全册结业考试

理论 180 分钟，编程与系统实践两天，总分 150 分。理论和实践都达到 70% 才通过。

### 一、状态估计，共 20 分

1. 推导 KF 预测/更新和 Joseph 形式。（4 分）
2. 从白加速度噪声推导 $[p,v]$ 的 Q。（4 分）
3. NIS 与 NEES 分别需要什么信息，检验什么？（4 分）
4. 为什么 VIO 与其使用过的 IMU 不宜当独立测量重复融合？（4 分）
5. 静止 IMU 能观察哪些姿态，不能观察什么？（4 分）

### 二、SLAM 前后端，共 20 分

1. 点到线扫描匹配的残差和退化方向。（4 分）
2. 关键帧应由哪些条件触发？（4 分）
3. 位姿图为何有规范自由度？（4 分）
4. 鲁棒核为何不能替代回环验证？（4 分）
5. ATE、RPE 和地图指标如何互补？（4 分）

### 三、地图定位规划，共 20 分

1. 推导 log-odds 更新。（4 分）
2. AMCL 为什么能多峰，如何处理绑架？（4 分）
3. footprint 与 inflation 如何由误差确定？（4 分）
4. A* 可采纳启发式的意义。（4 分）
5. Hybrid A* 比二维 A* 增加什么？（4 分）

### 四、局部导航 Nav2，共 20 分

1. 推导 Pure Pursuit 曲率。（4 分）
2. DWA 动态窗口和硬约束是什么？（4 分）
3. 制动距离如何包含延迟？（4 分）
4. map/odom/base_link 如何分工？（4 分）
5. 行为树恢复的安全边界是什么？（4 分）

### 五、诊断设计，共 20 分

1. RViz 位姿跳动怎样区分本地 odom 和全局定位问题？（5 分）
2. 地图墙变厚怎样区分运动畸变、外参和轨迹误差？（5 分）
3. 路径安全但实机碰墙如何分层排查？（5 分）
4. 如何证明一次参数优化不是过拟合测试路线？（5 分）

### 六、实践，共 50 分

1. 轮速/IMU 融合与一致性验证。（10 分）
2. 建图、回环审计和地图质量报告。（10 分）
3. AMCL 初始误差与绑架恢复实验。（10 分）
4. Nav2 30+ 自动任务和失败分类。（10 分）
5. 故障注入、压力测试、复现和答辩。（10 分）

## 第 23 章：理论答案

### 状态估计

KF 预测均值 $F\hat x+Bu$、协方差 $FPF^T+Q$；创新 $y=z-H\hat x^-$、$S=HP^-H^T+R$、$K=P^-H^TS^{-1}$；Joseph 更新 $(I-KH)P^-(I-KH)^T+KRK^T$ 更能保持数值半正定。白加速度经 $G=[\Delta t^2/2,\Delta t]^T$ 进入状态，$Q=G\sigma_a^2G^T$。

NIS 只需创新和其协方差，检查测量预测一致性；NEES 需要真值，检查状态误差与 P 是否一致。VIO 和原 IMU 相关，重复融合会协方差过小。静止加速度给重力方向即 roll/pitch，不能给全局 yaw；陀螺静止均值可估偏置。

### SLAM

点到线残差 $n^T(Tp-q)$，长直墙沿墙平移弱约束。关键帧结合位姿变化、视差、跟踪点、共视、新区域和后端负载。相对边对整体刚体变换不敏感，Hessian 有规范零空间，需固定节点。

鲁棒核只能降低大残差外点；相互一致或初始残差小的错误回环仍可能被接受，必须前端几何/时序验证。ATE 看全局轨迹，RPE 看局部漂移，地图指标看表面和占据结果。

### 地图规划

独立栅格近似下 posterior odds 乘当前 inverse sensor odds 并除先验 odds，取对数变成加法并截断。AMCL 粒子可处于多个模式，绑架后通过低似然检测、随机注入/全局粒子并多帧确认恢复。

安全膨胀至少覆盖机器人外廓、定位误差、控制误差和额外余量。A* 的可采纳启发式不高估剩余代价，保证最优性。Hybrid A* 把朝向加入状态并用运动学 primitive 和曲率约束。

### 局部导航

前视点在 robot frame 为 $(x_t,y_t)$，连接圆曲率 $2y_t/L_d^2$。DWA 取全局速度限制与当前速度在周期内按加速度可达窗口的交集，轨迹必须通过碰撞和制动硬约束。

安全距离包含延迟运动 $v\tau$、制动 $v^2/(2a)$ 和余量。odom 连续供控制，map 全局但可修正，base_link 是机体。行为树只能在前置安全条件下有限重试；底层几何、定位或制动错误必须修复而非恢复循环。

### 诊断

位姿跳先分别查看 `odom -> base_link` 与 `map -> odom`；前者查轮速/融合/时间，后者查 AMCL/SLAM/地图。墙厚对速度敏感支持运动畸变，对位置方向固定支持外参，闭环/轨迹段变化支持轨迹误差，可用真值位姿建图对照。

路径安全但碰墙要依次检查地图/路径 footprint 回放、定位误差、local costmap、控制跟踪、底盘执行和延迟。参数对比需冻结调参集，使用未见路线/场景、多次重复、交错 A/B 和成功、安全、效率多指标。

## 第 24 章：实践评分

### 融合 10 分

单位/TF/时间 2 分，轮速和 IMU 单独标定 2 分，融合配置与协方差 2 分，真值误差和 NIS/一致性 2 分，故障注入 2 分。

### 建图 10 分

数据协议 2 分，前端/回环证据 2 分，地图尺度/墙厚/拓扑 3 分，版本复现 2 分，已知限制 1 分。

### 定位 10 分

初始误差矩阵 2 分，多峰/重复区域 2 分，动态运行 2 分，绑架检测恢复 2 分，错误收敛和安全状态 2 分。

### 导航 10 分

任务自动化 2 分，30+ 试验 2 分，多指标 2 分，失败分类/证据 2 分，实机安全 2 分。

### 工程 10 分

测试/CI 2 分，版本化制品 2 分，故障注入 2 分，压力/延迟 2 分，另一人复现和答辩 2 分。

## 第 25 章：毕业答辩

现场应能回答并用证据展示：

1. 谁发布 `map -> odom`，当前模式为何唯一？
2. 轮径和有效轮距怎样标定？
3. IMU 静止输出和协方差来自哪里？
4. 最差回环候选为何被接受/拒绝？
5. 地图墙厚 P95 和已知尺度误差是多少？
6. AMCL 在重复走廊有何错误模式？
7. 绑架后如何检测和恢复，期间机器人做什么？
8. footprint 与 inflation 的物理依据是什么？
9. 最高速度下制动安全距离是多少？
10. 成功率置信区间和最差场景是什么？
11. 推理/规划/控制 P95 延迟如何分解？
12. 杀死定位或激光节点后系统怎样安全停止？
13. 新参数为何不是对测试路线过拟合？
14. 如何一条命令复现某次失败？

## 第 26 章：第三册完成清单

### 理论

- 能推导 KF/EKF、NIS/NEES 和可观性。
- 能解释扫描匹配、视觉里程计、运动畸变和退化。
- 能写出位姿图残差、规范、鲁棒核和回环验证。
- 能推导占据地图、AMCL、A*、Pure Pursuit 和制动距离。

### 工程

- TF 唯一、单位统一、时间健康可自动检查。
- 建图和导航模式启动边界明确。
- 地图、参数、bag 和软件版本可追踪。
- Nav2 lifecycle、costmap、plugin 和 BT 有测试。

### 实验

- 融合、建图、定位和导航都有独立真值/代理指标。
- 至少 50 个导航任务和场景切片。
- 至少五类故障注入和三个真实失败复盘。
- 完成低速实机与长时间压力测试。

### 安全

- 定位退化、传感器过期、无安全轨迹和节点死亡均停止。
- 速度、加速度、制动、footprint 和净空来自实测。
- 自动恢复有前置条件、次数、超时和人工接管。
- 错误成功和错误接受单独统计。

这些证据齐全后，第三册才算完成。学习者应能把“机器人走不动”拆成传感器、TF、融合、定位、地图、规划、控制、底盘和行为树中的可验证假设，并用日志、回放和自动测试定位，而不是依赖 RViz 观感反复盲调参数。

## 第 27 章：参数 Schema 与启动前审计

大型 Nav2 YAML 最危险的问题不是语法错误，而是拼写或层级错误后插件悄悄使用默认值。项目应为关键参数建立 schema，并在启动前执行跨参数约束。

```python
from dataclasses import dataclass

@dataclass(frozen=True)
class MotionLimits:
    minimum_linear: float
    maximum_linear: float
    maximum_angular: float
    acceleration_linear: float
    deceleration_linear: float
    controller_frequency: float

    def validate(self):
        errors = []
        if not 0 <= self.minimum_linear <= self.maximum_linear:
            errors.append("linear speed bounds are inconsistent")
        if self.maximum_angular <= 0:
            errors.append("maximum angular speed must be positive")
        if self.acceleration_linear <= 0 or self.deceleration_linear <= 0:
            errors.append("acceleration magnitudes must be positive")
        if self.controller_frequency < 5:
            errors.append("controller frequency is below project minimum")
        if errors:
            raise ValueError("; ".join(errors))

@dataclass(frozen=True)
class SafetyGeometry:
    footprint_radius_m: float
    localization_p95_m: float
    tracking_p95_m: float
    extra_margin_m: float
    inflation_radius_m: float

    def required_inflation(self):
        return (self.footprint_radius_m + self.localization_p95_m
                + self.tracking_p95_m + self.extra_margin_m)

    def validate(self):
        values = vars(self).values()
        if any(value < 0 for value in values):
            raise ValueError("safety geometry cannot contain negative values")
        required = self.required_inflation()
        if self.inflation_radius_m + 1e-9 < required:
            raise ValueError(
                f"inflation {self.inflation_radius_m:.3f} m is below "
                f"evidence-based requirement {required:.3f} m"
            )
```

真实 footprint 为多边形时，上例的 radius 只是保守包络。审计还要检查：local costmap 边长大于最高速度下的两倍制动观察距离；transform tolerance 不大于项目允许消息年龄；controller 模拟周期与频率相容；goal tolerance 大于定位噪声下界；所有 plugin ID 都有对应类型；observation source 的 topic 和 frame 存在。

审计输出解析后的完整参数、来源文件和每个派生量。例如以 `v=1.0 m/s`、总延迟 `0.15 s`、实测减速度 `0.8 m/s²`、余量 `0.2 m`，最低前向观察距离为

$$
1.0\times0.15+\frac{1.0^2}{2\times0.8}+0.2=0.975\text{ m}.
$$

若 local costmap 前方只有 0.8 m 可见范围，调 controller 权重不能满足制动安全。

## 第 28 章：任务结果汇总程序

自动测试产生 JSONL 后，汇总器必须把失败保留在分母中，按场景分组，并防止缺失值被当作零。

```python
from collections import defaultdict
import json
import numpy as np

def read_trials(path):
    records = []
    with open(path, "r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError as error:
                raise ValueError(f"invalid JSON at line {line_number}") from error
            required = {"trial_id", "scenario", "success", "duration_s",
                        "minimum_clearance_m", "failure_reason"}
            missing = required - record.keys()
            if missing:
                raise ValueError(f"line {line_number} missing {sorted(missing)}")
            records.append(record)
    if not records:
        raise ValueError("no trial records")
    return records

def summarize_group(records):
    count = len(records)
    successes = [record for record in records if record["success"]]
    clearances = [record["minimum_clearance_m"] for record in records
                  if record["minimum_clearance_m"] is not None]
    durations = [record["duration_s"] for record in successes
                 if record["duration_s"] is not None]
    failures = defaultdict(int)
    for record in records:
        if not record["success"]:
            failures[record["failure_reason"] or "unknown"] += 1
    return {
        "trials": count,
        "successes": len(successes),
        "success_rate": len(successes) / count,
        "duration_p50_s": float(np.median(durations)) if durations else None,
        "duration_p95_s": float(np.quantile(durations, 0.95)) if durations else None,
        "clearance_min_m": float(np.min(clearances)) if clearances else None,
        "clearance_p05_m": float(np.quantile(clearances, 0.05)) if clearances else None,
        "failure_counts": dict(failures),
    }

def summarize_by_scenario(records):
    groups = defaultdict(list)
    for record in records:
        groups[record["scenario"]].append(record)
    return {name: summarize_group(group) for name, group in sorted(groups.items())}
```

成功任务的 duration 与失败超时不能直接混成一个“平均到达时间”，应分别报告。净空的低分位比平均值更能反映风险。场景结果还要与总体结果一起展示，避免大量宽走廊掩盖窄门零成功。

### 28.1 基线回归

每个软件版本和基线版本对相同 trial ID 配对比较。对连续指标计算逐任务差值，对成功/失败统计“基线成功新版本失败”和相反情况。未配对的随机运行受场景难度干扰。

若新版本总体成功率从 94% 到 96%，但五条基线成功任务变失败、七条失败变成功，需要逐条分析，不能只看净增加两条。安全净空下降或碰撞增加时直接阻止发布。

## 第 29 章：十类标准场景验收

### 29.1 空旷直线

目标：隔离底盘和跟踪。要求直线横向误差、速度稳态误差、停止距离和 yaw 漂移达标。失败优先查轮速尺度、左右轮差、控制死区和定位噪声，不查障碍 critic。

### 29.2 连续 S 弯

目标：观察曲率变化、角速度饱和和前视策略。分别低速和最高工作速度运行，记录横向 P95、角速度/加速度和切弯净空。若低速好高速差，检查曲率限速和延迟。

### 29.3 直角转弯

比较全局路径平滑、controller horizon 和 footprint 角部。禁止仅以圆形半径替代长方形底盘。反向运行验证左右行为对称。

### 29.4 窄门

门宽分为安全余量 +20 cm、+10 cm、临界和不可通四档。规划器应在前三档表现符合门槛，并对不可通明确失败。记录最小净空和定位横向误差，不以“挤过去”作为成功。

### 29.5 U 型障碍

检验局部极小、全局重规划和恢复次数。行为树应在有限重试后换策略或失败，不允许无限旋转。保存每次 global path 和 local trajectory 变化。

### 29.6 动态横穿

障碍以多个速度和出现距离横穿。测首次可见到速度下降、完全停止距离、障碍离开后的恢复时间。动态脚本固定随机种子，保护人员不充当测试障碍。

### 29.7 临时封路

原全局路径被封锁后，系统应在时限内重新规划替代路线；没有替代路线时安全等待/失败。清图不得删除仍被传感器观察的真实障碍。

### 29.8 重复走廊

检验 AMCL 多峰和错误收敛。设置不同初始误差与绑架，记录粒子模式、激光似然、恢复和错误高置信定位。定位不确定时控制降速。

### 29.9 感知盲区

在不造成碰撞的装置中放置低矮/悬空/玻璃等挑战目标，验证传感器能力边界。无法可靠观测的类别写入运行限制，必要时增加传感器或禁行区，不能靠调 inflation 推断不存在的数据。

### 29.10 长时间巡航

至少 4 小时重复多路线，记录内存、CPU、温度、消息频率、TF 失败、规划长尾、地图残留和失败证据保存。资源缓慢增长是失败，即使导航任务尚未超时。

## 第 30 章：五次值班故障演练

### 演练一：启动后没有地图

观察 map server lifecycle、地图文件路径、YAML 权限和 QoS；检查 `ros2 topic echo /map --once`；确认 BT/AMCL 是否等待 map。演练目标是在不重启整机的情况下定位到资源加载还是 lifecycle 转换，并保持底盘停止。

### 演练二：突然出现 TF extrapolation

记录请求时刻、缓存最早/最晚时刻、消息 stamp 和系统 clock。检查某设备时钟跳变、仿真 `/clock`、bag loop 和队列延迟。禁止把 tolerance 调到数秒作为修复；恢复前清除跨 epoch 的旧队列。

### 演练三：AMCL 错误收敛

识别激光与地图错位、粒子过度集中和低似然；取消当前任务，触发全局重定位或要求人工初始位姿；连续多帧通过后恢复。保留错误粒子和原始 scan，不用重新点初始位姿掩盖案例。

### 演练四：局部 costmap 全黑

检查机器人是否在地图/窗口内、unknown 组合方式、footprint 自碰撞、传感器 marking、map 更新和坐标。依次关闭非必要 layer 找到来源，不能永久关闭障碍层换取运动。

### 演练五：底盘继续执行旧速度

断开 controller 或网络，验证 driver watchdog 在规定时间停止；检查 mux 优先级和安全节点；硬件急停独立生效。该演练在架空/低功率环境进行，任何超时失败阻止实机导航。

每次演练记录发现时间、停止时间、根因时间、恢复时间和错误操作。运行手册根据演练结果更新，并由另一人复现。

## 第 31 章：十二周执行计划

### 第 1 周：传感器与 TF

完成频率、单位、轴、时间和外参表。通过直行/旋转物理测试。交付 TF 唯一发布者测试和传感器健康报告。

### 第 2 周：轮速里程计

实现差速积分，标定轮径和有效轮距，完成正反直线与左右旋转。交付尺度和闭环误差。

### 第 3 周：IMU 与融合

采集静态数据、估计偏置噪声，逐个融合轮速/IMU，注入轴反转和延迟。交付真值误差与一致性分析。

### 第 4 周：SLAM 前端

采集/仿真走廊、房间、动态和快速旋转。分析扫描匹配残差、条件数、运动畸变和跟踪失败。

### 第 5 周：后端与回环

实现小型 SE(2) 图优化，验证 Jacobian、规范和错误回环。审计真实 SLAM 回环候选。

### 第 6 周：地图制品

生成候选地图，测尺度、墙厚、拓扑、未知和动态残留，冻结 `map_id` 与元数据。

### 第 7 周：AMCL

完成初始误差网格、重复走廊、动态运动和绑架恢复。设置定位质量门禁。

### 第 8 周：全局规划和 costmap

验证 footprint、inflation、unknown 和 A* 最优性。完成窄门四档实验和路径 footprint 回放。

### 第 9 周：局部控制

在空场完成直线、圆、S 弯和直角；测曲率限速、延迟、饱和、死区和制动距离。

### 第 10 周：行为树与动态障碍

实现有限恢复、封路重规划和动态横穿安全。验证所有失败有状态码且不会无限 retry。

### 第 11 周：自动化基准

执行至少 50 任务，生成场景切片、置信区间、失败证据和基线对比。修复仅使用训练/调参场景。

### 第 12 周：保留测试与答辩

冻结版本，在未见路线运行最终测试、故障演练和 4 小时压力。整理一键复现、演示视频和答辩证据。

若某周验收未通过，不按日历强行进入下一级。计划的价值在于依赖顺序和证据门槛，而不是十二周这个数字。

## 第 32 章：最终报告目录

```text
# 第三册结业报告

## 1. 任务、边界与安全门槛
## 2. 硬件、软件和版本
## 3. 坐标系、时间与 TF authority
## 4. 轮速和 IMU 标定
## 5. 状态融合模型与一致性
## 6. SLAM 前端、退化与运动畸变
## 7. 后端、回环和地图优化
## 8. 地图质量与版本制品
## 9. AMCL 定位与绑架恢复
## 10. Costmap、Footprint 与安全余量
## 11. 全局/局部规划和控制
## 12. Nav2 行为树与恢复
## 13. 自动化任务结果与置信区间
## 14. 故障注入和压力测试
## 15. 最差案例及根因证据
## 16. 已知限制、回滚和下一步
## 17. 完整复现命令
```

报告中的每个结论链接到配置、原始记录和生成脚本。图表不手工修改数值，全部由版本化数据生成。最终交付的真正价值不是“我成功跑过 Nav2”，而是另一名工程师能重放同一失败、复核同一指标，并在改变一个参数后知道系统的安全、精度和效率发生了什么。

## 第 33 章：实机安全评审

### 33.1 能量和速度边界

在允许自主导航前，记录机器人质量、最大速度、最大动能、实测制动减速度和最坏通信延迟。平地测试不能代表坡面、湿滑地面和低电量。按不同载荷和地面测制动距离，取保守分位数设置速度区。

软件速度限制必须在底层驱动再次执行。即使 Nav2 参数错误或话题被其他节点发布，驱动也拒绝超过物理门槛的命令。速度 mux 明确 teleop、navigation、safety 和 emergency 的优先级，并对每个来源设置 watchdog。

### 33.2 急停链

硬件急停不依赖 ROS、网络和主机进程，触发后切断或安全禁用执行器。软件急停用于自动故障，但不能替代硬件。测试包括：导航正常时按下；高 CPU 时按下；网络断开；controller 崩溃；急停释放。释放急停不能自动恢复旧速度，必须重新经过 readiness 和人工确认。

### 33.3 场地和人员

标定隔离区、测试区、观察区和紧急撤离路线。指定一人负责急停，不同时操作调参。首次运行去除无关人员和易碎物，速度/加速度从最低档逐级提高。动态障碍使用可控设备或仿真人员轨迹，不让人员承担碰撞测试。

### 33.4 上车前签字表

| 检查项 | 证据 | 负责人 | 结果 |
| --- | --- | --- | --- |
| 轮子方向和编码器符号 | 架空录像/日志 |  |  |
| 硬件急停独立有效 | 测试记录 |  |  |
| driver watchdog | 断流测试 |  |  |
| footprint 覆盖外廓 | 尺寸图 |  |  |
| 制动距离覆盖最高速度 | 曲线 |  |  |
| TF 唯一且时间健康 | 自动报告 |  |  |
| 定位退化触发停止 | 故障注入 |  |  |
| 无安全轨迹触发停止 | 仿真/低速 |  |  |
| 地图和配置版本冻结 | 哈希 |  |  |
| 回滚版本可启动 | 演练记录 |  |  |

任一关键项没有证据即不进入实机自主阶段。“以前测过”不等于当前硬件、地图和版本组合通过。

## 第 34 章：发布与回滚演练

### 34.1 发布包

发布版本是一个不可拆分组合：容器/依赖、ROS workspace、URDF、固件要求、地图 ID、传感器外参、融合参数、Nav2 参数、行为树、安全门槛和基准报告。为整个 manifest 计算哈希，运行时 diagnostics 持续发布版本。

### 34.2 灰度验证

新版本先在仿真和 bag replay 运行，再在一台测试机器人低速运行，最后扩大到目标设备。每级使用同一自动任务集合和发布门槛。发现回归时停止扩大，保留现场证据。

### 34.3 回滚触发

以下任一项立即回滚：碰撞或急停异常；错误高置信定位；路径穿越致命区；成功率低于门槛；最小净空下降；P95 延迟超限；资源持续增长；失败证据无法保存。普通任务超时是否回滚取决于预定义数量和场景，不能事后更改规则。

### 34.4 回滚步骤

1. 停止当前导航并确认底盘零速度。
2. 保存当前 trial、bag、参数和 diagnostics。
3. 停用 lifecycle 节点，不让两版本同时发布 TF 或速度。
4. 加载上一完整 manifest，而不是只替换一个 YAML。
5. 重新执行传感器、TF、定位和安全 readiness。
6. 运行固定 smoke goals，确认通过后恢复任务。

回滚也要定期演练。没有验证过的旧版本可能因地图、固件或依赖变化已经无法启动。

### 34.5 变更影响矩阵

| 变更 | 最低重跑内容 |
| --- | --- |
| 轮径/轮距 | 里程计、融合、建图、定位、导航 |
| 激光外参 | 扫描匹配、地图、AMCL、costmap |
| 地图 | AMCL、全局路径、全部任务 |
| Footprint/inflation | 碰撞、窄门、全部安全场景 |
| Controller | 跟踪、制动、动态障碍、任务基准 |
| 行为树 | 故障注入、恢复次数、超时 |
| 底盘固件 | 命令、反馈、watchdog、全级安全 |

影响矩阵防止“只改一个小参数所以不用测试”的错误假设。变更越靠近传感器、坐标和底盘基础层，向下游传播的验证范围越大。

## 第 35 章：结业结论写作标准

合格结论包含条件、样本、指标和限制。例如：“在 warehouse_v1 地图、0.6 m/s 上限、50 组保留任务和三种动态脚本中，成功率 96%，Wilson 95% 区间为某范围；最小净空不低于门槛；两次失败分别为定位恢复超时和封路不可达，均安全停止。”

不合格结论是“Nav2 已调好”“地图很准”“大多数时候能走”。它没有环境、版本、速度、样本、指标和反例，无法支持部署决定。

最终答辩应主动展示最差案例。能够说明系统在哪里失败、怎样检测、如何停止和下一步需要什么证据，比隐藏失败更能证明工程能力。第三册的核心不是记住每个插件，而是建立从概率估计到安全动作的可验证责任链。
