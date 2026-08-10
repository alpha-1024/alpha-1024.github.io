---
title: 第一册扩展：ROS2 实战案例
description: 用完整节点、接口、时间、TF、QoS 和故障诊断案例掌握 ROS2。
---

# 第一册扩展：ROS2 实战案例

## 案例 1：设计一个传感器驱动接口

假设要接入串口 IMU。不要一开始把串口解析、滤波、TF 和可视化写在一个节点中。分层：

```text
serial_driver → raw_imu → imu_calibration → /imu/data
                                  ↓
                            diagnostics
```

驱动层只负责协议和时间戳；标定层处理轴向、比例和偏置；融合由后续节点完成。

### 消息选择

优先使用 `sensor_msgs/Imu`，其中包括 orientation、angular_velocity、linear_acceleration 和三组协方差。若设备不提供姿态，不要填单位四元数并声称可信，应按消息约定标记不可用或提供合理协方差。

### 参数

```yaml
imu_driver:
  ros__parameters:
    port: /dev/ttyUSB0
    baud_rate: 921600
    frame_id: imu_link
    expected_rate: 200.0
    timestamp_source: device
```

`timestamp_source` 必须明确。设备时钟需要与主机时钟建立偏移或同步，否则多传感器融合会出现固定延迟。

### 诊断

每秒统计：实际频率、校验失败、丢包、时间戳倒退、串口重连次数。一次校验失败可警告，持续失败应进入 ERROR。

## 案例 2：Publisher 与 Subscriber

发布端不应在构造函数里做无限阻塞读取。可使用定时器轮询非阻塞接口，或单独 I/O 线程把完整帧放入有界队列。

```python
class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.pub = self.create_publisher(Imu, 'imu/data_raw', sensor_qos)
        self.timer = self.create_timer(0.005, self.publish_latest)

    def publish_latest(self):
        packet = self.device.try_read_latest()
        if packet is None:
            return
        msg = convert(packet)
        self.pub.publish(msg)
```

如果设备一次返回多帧，要决定全部发布还是只发布最新帧。用于离线记录时保留全部；用于实时控制时过旧数据可能没有价值。

## 案例 3：QoS 不兼容

现象：节点列表正常、Topic 名存在、发布者计数为 1，但订阅回调从不触发。

排查：

```bash
ros2 topic info /imu/data_raw --verbose
```

查看两端 Reliability、Durability。发布端 Best Effort、订阅端强制 Reliable 时可能不兼容。

修复不等于统一改 Reliable。传感器推荐 sensor data QoS，算法节点应匹配接口设计。对“最后一份静态配置”使用 Transient Local，对高频图像不要使用 Keep All。

## 案例 4：Service 与 Action

“重置 IMU 偏置”是短操作，可用 Service；“导航到目标并持续反馈”是长任务，可取消，使用 Action。

Action 包含 goal、feedback、result。服务端必须处理：拒绝非法目标、取消请求、执行超时、并发目标策略和节点关闭。

取消不是收到请求就立即报告成功。应先让底层控制器停止或进入安全状态，再返回 canceled。

## 案例 5：TF 发布方向错误

目标：相机安装在机器人前方 0.2m、高 0.5m。需要发布 `base_link → camera_link`。

常见错误：把测量“相机在 base 中的位姿”误解为 “base 在相机中的位姿”并求逆两次。

验证方法：

1. RViz 显示坐标轴。
2. 在 camera 前方 1m 创建点。
3. 转到 base 后应位于机器人前方约 1.2m、相应高度。
4. 手算一个简单点验证。

不要只看模型“似乎在正确位置”，镜像或轴向错误可能在静态图中不明显。

## 案例 6：TF 时间问题

现象：`Lookup would require extrapolation into the future/past`。

可能原因：

- 消息时间戳使用设备时间但未同步。
- TF 发布频率低。
- 算法处理延迟超过 TF 缓存。
- rosbag 回放未使用仿真时间。
- 查询使用 `now()` 而不是测量时间。

正确策略通常是在消息采集时间查询对应 TF。如果必须用最新变换，应明确这会引入运动误差。

设机器人角速度 $1rad/s$，时间错位 100ms，姿态误差约 0.1rad（5.7°），对远距离点会产生明显位置误差。

## 案例 7：消息同步

RGB 30Hz、深度 30Hz，但硬件触发不同。ExactTime 可能永远匹配不到；ApproximateTime 可以设 20ms 容差。

实验应记录实际时间差分布。若 P95 为 35ms，设 20ms 会丢大量对；设 100ms 又会匹配错误时刻。最终方案可能需要硬件同步而不是继续调软件容差。

## 案例 8：Callback 阻塞

现象：相机 30Hz，推理 100ms，结果延迟越来越大。

原因：每帧进入队列，消费速度 10Hz，消息年龄持续增长。解决：QoS depth=1、最新帧缓冲，或降低采集频率。不能用更多线程无上限并发推理，否则显存和调度恶化。

设计一个 latest-frame buffer：写入替换旧帧；worker 取走当前最新帧；统计被覆盖数量和消息年龄。

## 案例 9：Lifecycle 启动顺序

系统依赖：相机 → 感知 → 规划 → 控制。若控制先 active，可能在感知未就绪时接收空目标。

Lifecycle Manager 按顺序 configure 和 activate。每个节点 configure 时加载资源并验证参数，activate 时才开始输出。

模型加载失败应使 configure 失败，而不是 active 后每帧报错。

## 案例 10：参数更新

部分参数可运行时更新，例如置信度阈值；模型路径和网络结构通常不适合无保护热改。

参数回调先验证全部参数，再原子应用。若一组参数相互依赖，不应更新一半成功、一半失败。

```python
def on_parameters(params):
    candidate = dict(current)
    for p in params:
        candidate[p.name] = p.value
    if not 0.0 <= candidate['threshold'] <= 1.0:
        return SetParametersResult(successful=False, reason='threshold out of range')
    apply(candidate)
    return SetParametersResult(successful=True)
```

## 案例 11：rosbag 数据产品

录制不是随手执行命令。定义 bag manifest：机器人、传感器序列号、标定版本、软件 commit、Topic、场景、起止时间和备注。

包损坏或磁盘写满必须监控。高带宽图像可评估压缩和存储速度，但压缩会增加 CPU 并改变数据质量。

回放测试：

- 固定播放速率。
- 使用 `/clock`。
- 清理上次输出。
- 运行算法。
- 收集结构化指标。
- 与允许阈值比较。

## 案例 12：Launch 分层

```text
bringup.launch.py
  ├─ sensors.launch.py
  ├─ perception.launch.py
  ├─ localization.launch.py
  └─ control.launch.py
```

顶层负责组合，子 launch 可独立启动测试。使用 Launch Argument 选择 `simulation`、`robot_name`、`params_file`，不要复制多份 launch。

## 案例 13：多机器人 namespace

每台机器人使用 namespace 和独立 TF 前缀策略。全局地图和调度节点可能位于根 namespace。

Topic 做 namespace 不会自动解决 TF frame 冲突，因为 frame_id 是消息字符串。需要明确 `robot_1/base_link` 或独立 DDS domain/网络架构。

## 案例 14：控制命令超时

底盘收到最后一条 `cmd_vel` 后，如果上游崩溃仍保持速度，会造成危险。底盘控制层实现 watchdog：超过 200ms 未收到新命令，速度逐渐或立即归零。

watchdog 必须位于接近执行器的层，而不是只放在高层导航节点。

## 案例 15：日志风暴

错误循环每毫秒打印一次，导致磁盘和 CPU 异常，关键日志反而被淹没。使用 throttle、状态变化日志和计数器。

良好日志包含上下文：节点、设备、错误码、期望/实际值、下一动作。`something wrong` 无法诊断。

## 案例 16：测试一个 ROS2 节点

把算法核心写成不依赖 ROS 的库，单元测试纯逻辑。ROS adapter 负责消息转换和参数。

集成测试启动节点，发布输入并等待输出。设置超时，测试正常、边界和错误输入。测试结束确保进程退出，否则 CI 会挂起。

## 完整项目：可靠的 RGB-D 定位节点

### 接口

输入：RGB、Depth、CameraInfo、TF。输出：目标 `PoseWithCovarianceStamped`、诊断、可视化。

### 数据流

```text
同步 → 校验 → 预处理 → 推理 → 深度统计
→ 相机坐标位姿 → TF → 协方差传播 → 发布
```

### 失败状态

- 输入超时。
- 时间差超限。
- CameraInfo 不匹配。
- 无检测。
- 有检测但无有效深度。
- TF 不可用。
- 推理超时。
- 输出非有限。

每种状态有明确诊断，不发布伪造零位姿。

### 指标

- 输入/输出频率。
- 消息年龄。
- 各阶段平均/P95 延迟。
- 检测和定位成功率。
- TF 失败数。
- 深度有效比例。
- 丢弃旧帧数。

### 测试

1. 纯函数：深度统计、坐标转换、协方差。
2. 固定图片：模型输出回归。
3. rosbag：端到端结果回归。
4. 故障注入：断流、错时间、无深度、无 TF。
5. 稳定性：连续运行 30 分钟。

## 实战检查题

1. IMU 不提供姿态时能否填单位四元数？
2. 为什么 sensor QoS 常用 Best Effort？
3. TF 查询为什么应使用测量时间？
4. 推理慢于相机时为何要丢旧帧？
5. Lifecycle configure 阶段适合做什么？
6. 多机器人 namespace 为什么不自动隔离 frame_id？
7. watchdog 为什么要靠近执行层？
8. 什么情况下应拒绝发布位姿而不是发布零值？

### 答案

1. 不应把单位四元数当有效测量，应按消息约定标记未知。
2. 新鲜数据比补发旧数据更重要，可降低积压延迟。
3. 坐标变换随机器人运动，当前变换不代表采集时刻。
4. 旧帧会增加闭环延迟，且队列持续增长。
5. 声明/验证参数、加载资源、建立连接，但不开始业务输出。
6. frame_id 是字符串内容，不一定受 ROS namespace remap。
7. 上游崩溃或网络断开时，执行层仍能独立停止。
8. 输入超时、深度无效、TF 缺失、输出 NaN 等无法保证语义时。
