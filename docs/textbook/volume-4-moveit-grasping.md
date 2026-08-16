---
title: 第四册第四篇：MoveIt2 与机器人抓取系统
description: 从 URDF/SRDF、规划流水线和 PlanningScene，到 MTC、抓取候选、夹爪、附着物及感知执行闭环。
---

# 第四册第四篇：MoveIt2 与机器人抓取系统

机械臂抓取不是“给 MoveIt 一个 Pose”。感知结果要有坐标、时间和不确定性；机器人模型要正确；场景要包含桌面、目标和夹爪；抓取任务要分成打开夹爪、预抓取、直线接近、闭合、附着、抬升、搬运和放置；每一阶段都可能因 IK、碰撞、控制或接触失败。

本篇以 MoveIt2 为工程框架，讲解 URDF/SRDF、规划组、IK 插件、Planning Pipeline、PlanningScene、MoveIt Task Constructor 和夹爪控制，并建立一条可复现的 RGB-D 抓取状态机。

## 第 1 章：MoveIt2 的职责边界

MoveIt2 提供机器人模型、运动学插件、碰撞环境、运动规划、时间参数化和执行接口。它不自动保证：视觉目标正确、TF 新鲜、碰撞网格与实物一致、底层控制稳定、夹爪真正抓住物体、环境在执行期间静止。

系统分层：

```text
任务/状态机
  -> 感知与目标管理
  -> 抓取候选与 MoveIt Task Constructor
  -> Planning Pipeline / PlanningScene
  -> trajectory execution manager
  -> ros2_control controllers
  -> hardware + safety
```

每层有独立状态码和日志，禁止把所有失败统一成 `planning_failed`。

## 第 2 章：URDF 审计

URDF 描述 link、joint、visual、collision、inertial、transmission/ros2_control。审计包括：

- link/joint 树唯一且无断链；
- joint origin、axis、type、limit 正确；
- visual/collision 网格单位和姿态正确；
- inertial origin 位于合理质心，惯量正定；
- flange、tool0、camera frame 明确；
- mimic 夹爪关系正确；
- `ros2_control` state/command interface 与硬件一致。

随机 q 比较 URDF 模型 FK 与实机/参考库。碰撞几何单独显示，不能只看 visual。

## 第 3 章：SRDF

SRDF 定义 planning group、group state、end effector、virtual joint、passive joint 和 disabled collision pairs。planning group 可以是 chain 或 joints 集合；tip link 和 base link 错会让 IK frame 不符合预期。

自碰撞矩阵通常通过随机采样生成。`never` 碰撞对可禁用以加速，但采样没覆盖到不等于物理不可能。对靠近关节限位和夹爪区域人工审计。

命名状态如 `home`、`ready` 应在关节限位内、无自碰撞，并由版本化配置生成。实机启动时当前状态不等于 home，不能把命名状态当作实际起点。

## 第 4 章：运动学插件

MoveIt 可使用 KDL、TRAC-IK、IKFast、bio_ik 或自定义插件。比较维度：成功率、求解时间、限位、冗余、多解、近奇异、position-only 和一致性限制。

解析 IKFast 快但模型改变要重新生成；KDL 数值通用、对 seed 敏感；TRAC-IK 通过多策略改善限位附近表现。选择基于目标集合基准，不仅是单个姿态。

### 4.1 IK 基准

从已知 q 的 FK 生成目标，因此至少存在一个解。按工作空间中心/边缘、姿态、限位和奇异切片，运行多个 seed。输出成功率、P50/P95、残差、限位余量、最小奇异值和碰撞率。

IK plugin 返回解后仍由 PlanningScene 检查碰撞。`setFromIK` 的 timeout、attempts 和 consistency limits 应记录。

## 第 5 章：Planning Pipeline

请求适配器、planner 和响应适配器组成流水线。常见预处理修正起始状态轻微越界、解析约束，后处理时间参数化。适配器可以改善输入，但不能静默修复严重越界或碰撞。

OMPL 提供 RRTConnect、RRT*、PRM 等；Pilz 适合工业 PTP/LIN/CIRC；CHOMP/STOMP/TrajOpt 类优化器提供平滑/优化。planner ID 必须确认实际加载，未知 ID 可能回退默认。

## 第 6 章：起始状态

规划前从 joint state monitor 获取最新完整状态。检查时间年龄、所有 group joint 存在、有限、限位内，并与控制器实际反馈一致。

规划到执行之间机器人可能被移动。执行前比较轨迹首点与当前状态，超过容差取消并重规划。将 start state 留空让 MoveIt 用 current state 时也要确认 monitor 已更新。

## 第 7 章：目标约束

关节目标直接、没有 IK 歧义。Pose 目标转换为 position/orientation constraints，并通过 IK 找配置。容差越紧越难规划；容差应来自任务物理要求。

轴对称物体的抓取可能只约束夹爪 z 轴，不需固定绕轴旋转。用完整四元数硬约束会丢失大量可行解。位置/方向约束 frame 和 link 必须明确。

## 第 8 章：PlanningScene Monitor

PlanningScene Monitor 融合 robot state、TF、collision objects 和 octomap。规划使用某一时刻 scene snapshot。异步更新后立即规划可能看不到新障碍。

为 scene 维护应用确认：发布目标 object 后等待其 ID、pose 和版本出现在 monitored scene。物体 pose frame 转 planning frame 时使用对应采样时间，不使用任意最新 TF。

## 第 9 章：Collision Object 生命周期

对象操作有 ADD、MOVE、REMOVE、APPEND 等语义。每个对象 ID 唯一，几何和 pose 一致。更新环境障碍不应通过不断创建新随机 ID，否则场景积累。

```python
from dataclasses import dataclass

@dataclass(frozen=True)
class SceneObjectRecord:
    object_id: str
    frame_id: str
    timestamp_ns: int
    geometry_version: str
    pose_version: int

def validate_scene_object(record, now_ns, maximum_age_ms):
    reasons = []
    if not record.object_id or not record.frame_id:
        reasons.append("missing_identity")
    age_ms = (now_ns - record.timestamp_ns) / 1e6
    if age_ms < 0:
        reasons.append("future_timestamp")
    if age_ms > maximum_age_ms:
        reasons.append("stale_pose")
    if not record.geometry_version:
        reasons.append("unknown_geometry")
    return not reasons, reasons
```

## 第 10 章：Octomap 与深度环境

深度点云可更新 octomap，表示未知障碍。传感器噪声、机器人自身点和动态物体会污染。使用 self-filter、范围裁剪、占据/清除参数和时间管理。

Octomap 分辨率太细计算大，太粗封闭窄空间。抓取桌面上目标时，目标点云可能同时作为障碍阻止夹爪接近；将目标分割为独立 collision object，并在抓取阶段允许预定义接触，而不是清空所有障碍。

## 第 11 章：抓取姿态定义

一个 grasp 候选包含：夹爪在物体坐标的 pose、pre-grasp approach 方向/距离、post-grasp retreat、pre-grasp/grasp 夹爪状态、允许 touch object 和质量分数。

感知给 ${}^bT_o$，候选模板给 ${}^oT_g$，夹爪目标

$$
{}^bT_g={}^bT_o{}^oT_g.
$$

若规划 link 是 TCP 而模板定义在手掌 frame，还需固定变换。通过在物体坐标绘制候选轴验证方向。

## 第 12 章：候选生成

几何方法可从物体点云/包围盒生成顶抓、侧抓和法线对齐候选；学习方法预测抓取位姿/质量。无论来源，都先做：夹爪宽度、接近空间、IK、限位、碰撞、可操作度、与当前状态路径可达和不确定性鲁棒性。

候选不应只按网络分数排序。综合代价：

$$
J=w_qq_{quality}-w_dd_{motion}-w_cc_{risk}
-w_uu_{uncertainty}+w_mm_{margin}.
$$

各项归一化并通过验证集选择。保留失败原因统计，可以知道瓶颈是宽度、IK 还是碰撞。

## 第 13 章：位姿不确定性

感知输出均值位姿和协方差/多假设。简单方法在 $\pm$ 若干标准差扰动目标，候选只有在大多数样本仍 IK/无碰撞才接受。更严格使用 chance constraint 或 belief-space planning。

高不确定性可触发主动感知：移动相机到更佳视角重新估计，而不是盲目加大抓取容差。对称物体输出等价姿态集合，规划器选择易达安全候选。

## 第 14 章：MoveIt Task Constructor

MTC 将任务拆成 stage 并传播解。典型抓取：

1. CurrentState。
2. MoveTo 打开夹爪。
3. Connect 到预抓取区域。
4. GenerateGraspPose。
5. ComputeIK。
6. MoveRelative 沿接近方向。
7. ModifyPlanningScene 允许接触。
8. MoveTo 闭合夹爪。
9. AttachObject。
10. MoveRelative 抬升。
11. Connect 搬运。
12. 放置、打开、Detach。

每个 stage 可生成/传播多个 solution，成本累积。阶段化能明确失败位置，比单次 pose planning 更适合抓取。

## 第 15 章：预抓取与直线接近

从自由空间到预抓取可用全局规划；最后若任务要求直线接近，使用笛卡尔 MoveRelative/Pilz LIN，并设置最小/最大距离。接近方向 frame 常出错：物体 z、夹爪 z 或世界 z 意义不同。

直线段每一点 IK/碰撞都要验证。只规划终点会让夹爪侧向扫过物体。接近速度低于自由空间速度，并依据感知不确定性增加安全距离。

## 第 16 章：夹爪控制

夹爪可以是 position trajectory、GripperCommand action、力/电流控制或真空 I/O。命令包含目标宽度与最大 effort。成功条件不能只看 action SUCCEEDED：检查最终宽度、力/电流、物体传感器和时间。

空抓时夹爪通常闭到最小宽度；抓到物体时停在非零宽度或达到力阈值。软物体和滑移需要不同判据。最大 effort 从物体和夹爪安全范围确定。

## 第 17 章：Attach 与 Detach

确认夹持后，场景中目标从 world object 变为 attached object，父 link 通常是 tool/夹爪。允许与 finger touch links 接触，但不允许与手腕/环境任意碰撞。

附着前物体仍可能移动，使用最终确认的相对 pose。attach 不是物理抓住，只改变碰撞语义；夹爪实际失败却 attach 会让规划器相信物体随动。

放置时先到 pre-place、直线下降、接触/位置确认、打开、detach，再把物体以估计世界 pose 加回。操作顺序错误会瞬时产生碰撞或物体消失。

## 第 18 章：抓取状态机

```text
IDLE
 -> ACQUIRE_TARGET
 -> VALIDATE_TARGET
 -> BUILD_SCENE
 -> GENERATE_CANDIDATES
 -> PLAN_PICK
 -> EXECUTE_PREGRASP
 -> APPROACH
 -> CLOSE_GRIPPER
 -> VERIFY_GRASP
 -> ATTACH
 -> LIFT
 -> TRANSPORT
 -> PLACE
 -> VERIFY_PLACE
 -> COMPLETE
```

任何状态可进入 SAFE_STOP 或 RETRY。重试必须分类：感知失败换视角；IK 失败换候选；路径失败换 planner/候选；夹持失败重新感知；控制 fault 不自动重试。

## 第 19 章：目标数据契约

目标包含 object ID/class、pose、frame、采样 stamp、几何尺寸/CAD ID、对称性、协方差、可见比例和质量。规划前验证 age、frame、工作空间、有限性和 geometry version。

机器人/目标运动时，规划执行时间内 pose 会过期。固定桌面可设置较长 age，传送带需要预测和在线 servo，不使用同一阈值。

## 第 20 章：MoveIt Servo

Servo 接收 twist 或 joint jog，进行高速增量控制、限位和碰撞减速，适合视觉伺服和人工遥操作。输入必须定期更新，有 command timeout；靠碰撞和奇异时缩放/停止。

视觉伺服误差例如图像特征或末端 pose，控制频率与视觉延迟决定稳定性。Servo 不是绕开全局规划的捷径：长距离仍需规划，局部精调才用 servo。

## 第 21 章：场景变化与执行监控

执行期间持续监测关节跟踪、碰撞传感、目标/环境更新和安全。新障碍进入轨迹时取消、停止、更新 scene 并重规划。取消响应时间和制动距离必须实测。

目标 pose 更新不应每帧立即重规划，可能造成抖动。使用稳定窗口、变化阈值和任务阶段：接近前允许更新，接触后通常锁定或切换力控制。

## 第 22 章：失败分层

### 感知失败

无目标、错误类别、pose 越界、深度不足、过期、对称多解。行动：重新观察或拒绝。

### 模型/场景失败

对象未进入 scene、frame 错、桌面尺寸错、目标与环境重叠。行动：修 scene，不盲目重规划。

### IK/规划失败

无 IK、限位、奇异、自碰撞、环境碰撞、timeout。行动：换候选/seed/planner，次数有限。

### 执行失败

起点偏差、controller rejected、跟踪超差、驱动 fault、安全停止。行动：保存证据，控制类故障不自动继续。

### 抓持失败

空抓、滑落、碰撞、物体未抬起。行动：detach/更新 scene，退回安全位并重新感知。

## 第 23 章：抓取评测

分层概率：

$$
P(task)=P(perception)P(plan|perception)
P(execute|plan)P(grasp|execute)P(place|grasp).
$$

实际阶段相关，但分层计数能定位瓶颈。报告每阶段分母，不能只报“规划成功率 95%”而排除无 IK 目标。

指标包括：目标检测/pose、候选数、IK/规划、轨迹净空、执行跟踪、夹持、抬升、放置、总成功、时延 P95、重试和安全停止。

## 第 24 章：数据矩阵

至少 5 类物体，每类 20+ 次，覆盖工作空间中心/边缘、目标朝向、遮挡、背景、光照、相邻障碍、空/满载、抓取方向。训练/调参与保留测试分开。

对感知 pose 注入已知平移/旋转噪声，画成功率、碰撞拒绝和错误抓取随噪声曲线，确定门禁。

## 第 25 章：故障注入

- 相机到 base 外参偏移；
- 目标时间戳延迟；
- CAD 尺度错误；
- PlanningScene 更新延迟；
- 夹爪虚假成功；
- 执行中障碍加入；
- 控制器断开；
- 物体滑落。

系统应安全拒绝/停止并给出具体阶段，而不是继续执行后续 attach/transport。

## 第 26 章：综合项目

实现固定桌面 RGB-D pick-and-place：三种物体、两个放置区。使用第二册感知接口，生成至少顶/侧两类候选；MTC 规划；ros2_control 执行；夹爪验证；自动运行至少 60 次。

交付 scene 可视化、MTC stage 统计、分层成功率、P95 时延、三类失败 rosbag、噪声曲线和完整版本信息。仿真先通过，再低速实机分级。

## 第 27 章：常见故障

### 目标在 RViz 正确但 IK 无解

检查 tip link/TCP、姿态约束、planning group、目标 frame、关节限位和 seed。显示的是物体 pose，不代表夹爪候选 pose 可达。

### 规划夹爪穿过目标

目标未在 scene、更新未应用、允许碰撞过宽或只加了 visual。检查 object ID、collision geometry 和 ACM。

### 闭合后抬升规划失败

确认 attach、touch links、物体几何和相对 pose；物体可能与桌面仍深度相交。接触允许只针对夹爪，不应允许目标-桌面穿透。

### 规划每次结果不同

采样规划器随机正常。记录 seed，增加目标候选/规划时间，按质量门禁；不要只重复到偶然成功而不统计。

## 第 28 章：阶段考试

建议限时 180 分钟，满分 100 分。

### 一、理论题，共 35 分

1. URDF 与 SRDF 职责区别。（5 分）
2. IK success 为什么不等于可规划？（5 分）
3. PlanningScene snapshot 和版本为何重要？（5 分）
4. MTC 分阶段比单 Pose 规划的优势。（5 分）
5. attached object 与物理抓持区别。（5 分）
6. 目标 pose 不确定性怎样进入候选筛选？（5 分）
7. MoveIt Servo 的使用边界。（5 分）

### 二、代码题，共 30 分

1. 设计 PlanningScene 对象生命周期测试。（10 分）
2. 实现抓取候选门禁与排序。（10 分）
3. 设计抓取状态机的 retry/safe-stop 规则。（10 分）

### 三、综合题，共 35 分

1. MoveIt 规划成功但夹爪在目标旁闭合，分层诊断。（15 分）
2. 总抓取成功率从 70% 提到 82%，如何判断改进来自哪层且可泛化？（20 分）

## 第 29 章：参考答案

URDF 是物理/运动/碰撞模型，SRDF 是语义组、末端、命名状态和允许碰撞。IK 只给目标配置，可能碰撞、限位余量差、奇异或无法从当前状态连接。Scene snapshot 保证规划使用一致环境；执行前环境变化需重验。

MTC 暴露预抓取、接近、闭合、附着、抬升等阶段并保留多解，能定位失败。attach 只改变规划场景中物体随 link 运动和碰撞语义，不证明物理夹住。对 pose 分布扰动，候选在多数样本下保持 IK/无碰撞并有余量才接受。

Servo 用于局部高频增量/视觉精调，有 timeout、碰撞和奇异保护；不替代长距离全局规划。对象生命周期测试 ADD/MOVE/REMOVE、重复 ID、frame、异步确认、attach/detach、touch links、scene version 和过期 pose。

旁边闭合先检查相机外参、采样时间和 object/TCP frame；比较规划目标与执行时目标；检查 tool transform、夹爪控制和跟踪误差；用固定标定物隔离感知与机械臂。成功率改进按感知、候选、IK、规划、执行、夹持、放置分层计数，在未见物体/位置保留集多次运行，同时比较安全、时延和重试，不能只看总数。

## 本篇完成标准

完成本篇后，应能审计 URDF/SRDF 与 IK 插件；能维护有版本和时间语义的 PlanningScene；能把目标不确定性转成抓取候选门禁；能使用 MTC 建立预抓取到放置任务；能正确控制夹爪和 attach/detach；能监控执行中环境变化；能按感知、规划、执行、夹持和放置分层评价，并安全处理每类失败。

下一阶段将补充视觉伺服、手眼标定、移动目标抓取和机械臂综合工程验收，随后完成第四册结业项目与全册考试。
