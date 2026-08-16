---
title: 第三册第四篇：地图、定位与全局规划
description: 从占据栅格、inverse sensor model、AMCL 和代价地图，推导到 Dijkstra、A*、Theta* 与 Hybrid A*。
---

# 第三册第四篇：地图、定位与全局规划

SLAM 前后端给出机器人轨迹和环境约束，导航还需要把它们转换成可查询的地图、在已有地图中定位，并为具有真实尺寸和运动约束的机器人规划可执行路径。本篇从占据概率开始，说明一束激光怎样改变栅格；随后建立 AMCL 粒子定位、静态/动态代价地图和多类全局搜索。

地图不是一张供人观看的灰度图片。每个像素有分辨率、原点、坐标轴、未知状态和概率语义；规划器也不是只在二维数组上找最短线，它必须考虑机器人 footprint、定位误差、障碍膨胀、运动学和未知区域风险。

## 第 1 章：地图表示的选择

### 1.1 占据栅格

二维平面被分成固定分辨率的格子，每格保存占用概率 $p(m_i)$ 或离散状态：空闲、占用、未知。它适合室内移动机器人、射线更新、碰撞检查和图搜索，但高度信息被压缩，桌面下方、悬空障碍和坡道可能产生歧义。

### 1.2 代价地图

代价地图不仅表示障碍，还融合膨胀距离、动态传感器、禁行区、语义区域和未知空间风险。它是为规划决策服务的派生表示，不应覆盖原始静态地图。

### 1.3 点云、体素和语义地图

三维点云保留表面几何但查询和内存成本高；体素占据/TSDF/ESDF 提供三维空间结构和距离；语义地图附加类别、实例或可通行属性。选择取决于机器人自由度、传感器和任务。二维底盘也可能需要三维障碍投影，以识别桌面边缘或低矮障碍。

## 第 2 章：世界坐标与栅格坐标

设地图分辨率为 $r$ 米/格，地图原点在世界坐标 $(o_x,o_y)$，世界点 $(x,y)$ 对应

$$
i=\left\lfloor\frac{x-o_x}{r}\right\rfloor,
\qquad
j=\left\lfloor\frac{y-o_y}{r}\right\rfloor.
$$

这里 $i$ 是列还是行必须在代码中固定。ROS OccupancyGrid 数据按行展开，原点 `origin` 还包含姿态；很多简单实现假设原点旋转为零。如果地图原点有旋转，需要先把世界点变换到 map grid frame。

```python
from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class GridGeometry:
    width: int
    height: int
    resolution: float
    origin_xy: np.ndarray

    def validate(self):
        if self.width <= 0 or self.height <= 0 or self.resolution <= 0:
            raise ValueError("invalid grid dimensions or resolution")
        if np.asarray(self.origin_xy).shape != (2,):
            raise ValueError("origin must be [x,y]")

    def world_to_cell(self, xy):
        self.validate()
        xy = np.asarray(xy, np.float64)
        cell = np.floor((xy - self.origin_xy) / self.resolution).astype(int)
        col, row = int(cell[0]), int(cell[1])
        if not (0 <= col < self.width and 0 <= row < self.height):
            return None
        return row, col

    def cell_center(self, row, col):
        if not (0 <= col < self.width and 0 <= row < self.height):
            raise IndexError("cell outside grid")
        return self.origin_xy + self.resolution * np.array([col + 0.5, row + 0.5])
```

测试应覆盖原点、负坐标、恰好边界、地图外、cell center 往返和非整数分辨率。世界转 cell 再转中心不会恢复原点，只保证原点落在该 cell 范围内。

## 第 3 章：占据概率与 log-odds

### 3.1 二元占据变量

每个格子 $m_i\in\{0,1\}$。给定观测历史 $z_{1:t}$ 和机器人位姿 $x_{1:t}$，希望估计

$$
p(m_i=1|z_{1:t},x_{1:t}).
$$

直接反复做概率乘除容易数值不稳定。定义 log-odds：

$$
l_{t,i}=\log\frac{p(m_i=1|z_{1:t})}{1-p(m_i=1|z_{1:t})}.
$$

在常见独立栅格近似下：

$$
l_{t,i}=l_{t-1,i}+l(z_t|x_t,m_i)-l_0.
$$

$l_0$ 是先验 log-odds。概率恢复为

$$
p=\frac{1}{1+e^{-l}}.
$$

### 3.2 截断

反复观测同一墙面会让 log-odds 无限增长，环境变化后难以恢复。把 $l$ 限制在 $[l_{min},l_{max}]$。截断值决定需要多少次反向证据才能改变状态。

```python
def probability_to_log_odds(probability):
    p = np.asarray(probability, np.float64)
    if np.any((p <= 0) | (p >= 1)):
        raise ValueError("probability must lie strictly inside (0,1)")
    return np.log(p / (1.0 - p))

def log_odds_to_probability(log_odds):
    value = np.asarray(log_odds, np.float64)
    positive = value >= 0
    result = np.empty_like(value)
    result[positive] = 1.0 / (1.0 + np.exp(-value[positive]))
    exp_value = np.exp(value[~positive])
    result[~positive] = exp_value / (1.0 + exp_value)
    return result
```

分支实现避免很大负数上的指数溢出。

## 第 4 章：Inverse Sensor Model

### 4.1 一束激光的地图更新

从传感器原点到命中点之间的格子获得空闲证据，命中附近获得占用证据，射线之后保持未知。若测量为 `Inf` 或超过最大量程，它可能表示没有命中：更新射线内空闲，但不能在最大距离制造障碍。

### 4.2 Bresenham 栅格射线

```python
def bresenham_cells(start, end):
    row0, col0 = start
    row1, col1 = end
    x0, y0, x1, y1 = col0, row0, col1, row1
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    cells = []
    while True:
        cells.append((y0, x0))
        if x0 == x1 and y0 == y1:
            break
        twice = 2 * error
        if twice >= dy:
            error += dy
            x0 += sx
        if twice <= dx:
            error += dx
            y0 += sy
    return cells

def update_ray(log_odds_grid, cells, free_increment, occupied_increment,
               minimum, maximum, hit=True):
    if not cells:
        return
    free_cells = cells[:-1] if hit else cells
    for row, col in free_cells:
        log_odds_grid[row, col] = np.clip(
            log_odds_grid[row, col] + free_increment, minimum, maximum
        )
    if hit:
        row, col = cells[-1]
        log_odds_grid[row, col] = np.clip(
            log_odds_grid[row, col] + occupied_increment, minimum, maximum
        )
```

起点是否更新为空闲需要明确；通常机器人所在格应为空闲，但传感器附近的盲区和机体自反射要过滤。多个射线在同一帧重复更新同格会让证据与角分辨率相关，可按帧去重或接受该模型近似并校准。

### 4.3 位姿不确定性

上述更新假设机器人位姿已知。定位误差会把同一墙面写成厚带。高质量建图应联合优化位姿，或在更新中考虑位姿分布。若地图墙体很厚，不能只调占据阈值，应先检查里程计、时序、外参和回环。

## 第 5 章：地图分辨率和质量

分辨率从 5 cm 降到 1 cm，二维格数约增加 25 倍，更新、规划和内存成本随之上升。高分辨率也不会突破传感器、标定和定位误差。应让分辨率小于需要表示的最窄通道，但不远小于系统实际空间精度。

地图验收包括：墙厚分布、闭环重影、已知长度误差、自由/占据冲突、未知比例、动态残留、门宽和可通行拓扑。地图看起来漂亮不等于导航可用，尤其是人工修图可能删掉真实障碍或填充未知区域。

## 第 6 章：地图保存与 ROS 语义

ROS 地图通常包含图像和 YAML：`resolution`、`origin`、`occupied_thresh`、`free_thresh`、`negate`。不同阈值把灰度概率转成占用/空闲/未知。PNG 的图像 y 轴向下，而 map 世界 y 轴约定可能相反，加载器负责转换；自行处理时必须用已知点测试。

地图版本要绑定建图算法、参数、传感器外参和采集日期。修改地图后产生新 ID。定位节点、任务目标和禁行区域必须声明使用哪个地图版本。

## 第 7 章：已知地图中的 Bayes 定位

定位后验为

$$
p(x_t|z_{1:t},u_{1:t},m).
$$

运动模型根据里程计传播，观测模型比较当前激光与地图。单峰 EKF 难以表达机器人可能位于多个相似走廊的情况，粒子滤波用样本表达多峰。

## 第 8 章：粒子滤波与 AMCL

### 8.1 粒子表示

$$
p(x_t|\cdot)\approx\sum_{i=1}^Nw_t^{[i]}\delta(x_t-x_t^{[i]}).
$$

每个粒子按运动模型传播，再根据激光似然赋权并归一化。有效样本数

$$
N_{eff}=\frac{1}{\sum_i(w_i)^2}
$$

低于阈值时重采样。每步都重采样会加剧粒子贫化。

### 8.2 系统重采样

```python
def systematic_resample(weights, rng):
    weights = np.asarray(weights, np.float64)
    if weights.ndim != 1 or len(weights) == 0:
        raise ValueError("non-empty weight vector required")
    total = weights.sum()
    if not np.isfinite(total) or total <= 0:
        raise ValueError("weights must have positive finite sum")
    normalized = weights / total
    positions = (rng.random() + np.arange(len(weights))) / len(weights)
    cumulative = np.cumsum(normalized)
    cumulative[-1] = 1.0
    return np.searchsorted(cumulative, positions)
```

测试均匀权重、单一权重、未归一化权重和多次抽样频率。重采样复制高权粒子，不增加新信息；运动噪声和随机粒子注入保持多样性。

### 8.3 似然场模型

预先计算每个栅格到最近障碍的距离场。激光端点离地图障碍越近，似然越高：

$$
p(z|x,m)\propto\prod_k
\left(z_{hit}\exp(-d_k^2/(2\sigma^2))+z_{rand}c\right).
$$

连乘易下溢，使用 log-likelihood 或归一化。激光束不是严格独立，使用全部高密度束会过度自信，常做 beam skipping 或降采样。

### 8.4 KLD 自适应采样

AMCL 可根据粒子分布占据的 bin 数动态调整粒子数量：分布多峰/广泛时更多粒子，收敛后减少。最小/最大粒子数、误差和置信参数需要结合地图复杂度与实时性测试。

## 第 9 章：AMCL 初始化和绑架恢复

已知初始位姿时在局部高斯分布采样；全局定位时在所有自由区域采样，需要更多计算和独特环境结构。机器人被搬到新位置后，原粒子全部错误，似然也可能在错误局部模式中保持。随机粒子注入、全局重定位或上层人工重置用于恢复。

评价不只看粒子云是否集中：错误位置也可能高度集中。检查激光与地图叠加、真值误差、N_eff、恢复时间、错误收敛率和初始位姿覆盖。

## 第 10 章：AMCL 运动模型

差速或全向运动模型把里程计增量分解为旋转和平移，并依据移动量采样噪声。噪声参数反映轮速里程计误差随运动增长，不是随意的“抖动程度”。

用真实数据按直行、旋转、圆弧统计里程计误差，拟合参数。地面变化和打滑会让固定模型失效，可按场景使用更保守噪声。若粒子云太窄且真值常在外部，模型过度自信。

## 第 11 章：代价地图分层

典型 global/local costmap 包含：静态地图层、障碍层、体素层、膨胀层，以及禁行区/速度区等过滤器。各层按组合规则生成最终代价。

global costmap 用于较大范围路径，更新可较慢；local costmap 随机器人滚动，融合实时障碍并高频更新。动态障碍不应永久写入静态地图，但传感器清除失败会留下“幽灵障碍”。

## 第 12 章：Footprint 和机器人几何

圆形机器人可用半径近似，长方形、叉车或带机械臂底盘应使用多边形 footprint。footprint 必须覆盖实际外廓、线缆和定位/控制误差。仅使用内切圆会让角部碰撞。

碰撞检查可将 footprint 在候选姿态下变换并查询覆盖格子，也可把障碍按机器人形状做 Minkowski 膨胀，把机器人简化成点。非圆 footprint 的膨胀依赖朝向，普通二维各向同性 inflation 是保守近似。

## 第 13 章：膨胀层

距离障碍越近，代价越高。常见指数衰减：

$$
c(d)=c_{max}\exp[-\alpha(d-r_{inscribed})],
$$

其中 $r_{inscribed}$ 是内切半径。致命障碍、内切碰撞区、膨胀区和自由区有不同语义。

inflation radius 太小，路径贴墙且定位误差导致碰撞；太大，窄门被封死。参数应由实际 footprint、定位 P95、控制跟踪 P95 和安全余量共同确定：

$$
m_{safe}\ge r_{robot}+e_{localization}+e_{tracking}+m_{extra}.
$$

如果物理门宽小于双侧安全余量，调小膨胀只是取消保护，不能创造安全通道。

## 第 14 章：图搜索基础

栅格转成图：可通行格为节点，四邻域或八邻域为边，边代价包含距离与地图代价。Dijkstra 维护起点到节点的最小累计代价 $g(n)$；A* 使用

$$
f(n)=g(n)+h(n)
$$

优先扩展预计总代价小的节点。

若启发式不高估真实剩余代价，则可采纳，A* 保持最优。启发式还应尽量一致：$h(n)\le c(n,n')+h(n')$，避免节点反复打开。

## 第 15 章：完整 A* 实现

```python
from heapq import heappush, heappop
from math import sqrt

NEIGHBORS_8 = [
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, sqrt(2)), (-1, 1, sqrt(2)),
    (1, -1, sqrt(2)), (1, 1, sqrt(2)),
]

def octile_distance(a, b):
    dy, dx = abs(a[0] - b[0]), abs(a[1] - b[1])
    return max(dx, dy) + (sqrt(2) - 1.0) * min(dx, dy)

def astar(cost_grid, start, goal, lethal=253.0):
    grid = np.asarray(cost_grid, np.float64)
    rows, cols = grid.shape
    for node in (start, goal):
        if not (0 <= node[0] < rows and 0 <= node[1] < cols):
            raise ValueError("start or goal outside grid")
        if grid[node] >= lethal:
            return None, {"reason": "start_or_goal_blocked", "expanded": 0}
    open_heap = []
    counter = 0
    heappush(open_heap, (octile_distance(start, goal), counter, start))
    came_from = {}
    best_g = {start: 0.0}
    closed = set()
    while open_heap:
        _, _, current = heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path, {"reason": "ok", "expanded": len(closed),
                          "cost": best_g[goal]}
        closed.add(current)
        for dr, dc, distance in NEIGHBORS_8:
            neighbor = (current[0] + dr, current[1] + dc)
            if not (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols):
                continue
            if grid[neighbor] >= lethal or neighbor in closed:
                continue
            if dr != 0 and dc != 0:
                if grid[current[0] + dr, current[1]] >= lethal:
                    continue
                if grid[current[0], current[1] + dc] >= lethal:
                    continue
            traversal = distance * (1.0 + grid[neighbor] / 252.0)
            tentative = best_g[current] + traversal
            if tentative < best_g.get(neighbor, float("inf")):
                best_g[neighbor] = tentative
                came_from[neighbor] = current
                counter += 1
                priority = tentative + octile_distance(neighbor, goal)
                heappush(open_heap, (priority, counter, neighbor))
    return None, {"reason": "unreachable", "expanded": len(closed)}
```

对角移动时禁止穿过两个障碍角之间，否则点机器人路径在栅格上合法，真实 footprint 会碰撞。代价缩放会影响启发式的可采纳性；这里最低边代价仍为几何距离，所以 octile 不高估。

## 第 16 章：A* 测试与反例

测试空地图最短路、起点等于终点、起终点阻塞、不可达、单通道、对角角切、非方形地图、代价绕行和 Dijkstra 对照。将启发式设为零应得到与 Dijkstra 相同最优成本，但扩展节点通常更多。

曼哈顿距离用于四邻域可采纳；八邻域对角代价 $\sqrt2$ 时，曼哈顿会高估，可能失去最优性。欧氏和 octile 可采纳，octile 对八邻域更贴近真实代价。

## 第 17 章：Theta* 与路径平滑

A* 路径受栅格方向限制，产生锯齿。Theta* 在扩展时检查祖先到新节点是否有直线视线，得到任意角路径。视线检查必须覆盖 footprint/膨胀代价，而不是只看障碍中心。

后处理平滑可减少折线，但必须重新碰撞检查并保留足够安全距离。将路径点用样条强行拟合可能切入障碍或产生不可执行曲率。

## 第 18 章：Hybrid A*

汽车式机器人不能原地横移，状态需包含 $(x,y,\theta)$。Hybrid A* 使用符合运动学的 motion primitive 扩展，并可用解析 Reeds-Shepp/Dubins 连接目标。栅格离散用于 closed set，但节点位姿保持连续。

代价包含路径长度、倒车、转向、转向变化、障碍和启发式。启发式可组合忽略障碍的运动学距离与二维障碍搜索距离。维度增加使计算更重，分辨率、角度 bin 和 primitive 长度决定质量与成本。

对差速机器人，全局路径可不严格满足曲率，但局部控制仍有限速；对 Ackermann 底盘必须在全局层考虑最小转弯半径，否则窄区路径无法跟踪。

## 第 19 章：未知空间与探索风险

未知不是空闲。允许穿越未知适合探索任务，不适合已有地图上的安全运输。规划配置应明确 `allow_unknown`，并在可视化中区分未知、空闲和占用。

地图边界附近的未知可能只是未扫描，也可能真实有障碍。探索规划使用 frontier（已知空闲与未知边界）选目标，同时考虑信息增益、路径成本和返回安全。

## 第 20 章：全局规划工程接口

输入不仅是起终点，还有地图版本、时间、机器人模型和规划参数。输出应包括路径 frame、时间、总成本、长度、最小净空、扩展节点、状态码和地图版本。起点或终点被占用、TF 过期、地图为空和超时都需要独立状态。

若地图更新中途发生，规划器可锁定一个地图快照完成，随后检查路径是否仍有效；或者支持增量重规划。不能把来自不同版本代价地图的 g 值混在一个搜索中。

## 第 21 章：地图与规划综合实验

### 21.1 建图

用仿真或真实二维激光数据实现 log-odds 栅格更新。分别改变分辨率、占用/空闲增量和截断，测墙厚、门宽、未知比例和更新时间。注入位姿噪声与时间延迟，观察地图重影。

### 21.2 定位

在固定地图实现简化粒子滤波或使用 AMCL，测试正确初值、错误初值、全局定位和绑架。报告收敛时间、错误收敛率、粒子数、N_eff 和 CPU。

### 21.3 规划

实现 Dijkstra、A* 和 Theta*，在同一组 100 个起终点比较成功率、最优成本、扩展节点、时间、路径转折和最小净空。再改变 inflation 和 footprint，记录窄门何时安全/不可通行。

## 第 22 章：故障排查

### 地图上下颠倒

检查图像行方向、OccupancyGrid 展平方式、origin 姿态和显示工具。用三个不对称已知标记验证，不要依赖肉眼对称房间。

### AMCL 粒子集中但位置错误

重复走廊导致多峰错误收敛，粒子贫化后很难恢复。检查激光-地图叠加、初始分布、随机粒子、束模型和地图独特结构；集中不等于正确。

### 全局路径穿墙

检查 world/cell 转换、障碍阈值、地图版本、对角角切、footprint 和 unknown 设置。将路径每个姿态用 footprint 做独立碰撞回放。

### 窄门始终不可达

测实际门宽、机器人最大外廓、定位和控制误差，计算安全余量。若几何上确实不足，应报告不可达；只有确认模型过保守后才调整膨胀。

## 第 23 章：阶段考试

建议限时 180 分钟，满分 100 分。

### 一、理论题，共 35 分

1. 推导占据 log-odds 递推并解释截断。（5 分）
2. 激光 `Inf`、最大量程命中和无效测量如何区别更新？（5 分）
3. AMCL 为什么能表示多峰，何时粒子贫化？（5 分）
4. footprint、内切半径、外接半径和 inflation 的关系是什么？（5 分）
5. A* 启发式可采纳和一致分别意味着什么？（5 分）
6. 为什么八邻域用 Manhattan 可能非最优？（5 分）
7. Hybrid A* 相比栅格 A* 增加了什么约束？（5 分）

### 二、代码题，共 30 分

1. 为 world/cell 转换与 Bresenham 写十个边界测试。（10 分）
2. 完成 A* 并证明与 Dijkstra 的最优成本一致。（10 分）
3. 设计 AMCL 绑架检测和恢复状态机。（10 分）

### 三、综合题，共 35 分

1. 地图视觉清晰但 Nav2 经常擦墙，设计分层实验。（15 分）
2. 调小 inflation 后机器人能通过窄门，但实机偶尔碰撞。分析为何这不是成功调参，并提出验收方案。（20 分）

## 第 24 章：参考答案

### 一、理论题

1. 在独立栅格和 inverse sensor model 近似下，后验 odds 等于上一 odds 乘当前测量 odds/先验 odds，取对数变为加法。截断防止长期观测后概率饱和到无法适应环境变化，并限制数值范围。

2. 有限且在量程内的命中：沿途空闲、端点占用；明确“无返回”的最大量程：沿有效射线为空闲但端点不设障碍；NaN、非法负数等无效测量通常不更新。具体还要遵循驱动编码。

3. 粒子集合可同时分布在多个假设区域。反复重采样、噪声过小或观测过强会复制少数粒子并丢失其他模式，形成贫化。只剩错误模式后需要随机注入或全局重置。

4. footprint 是真实多边形，内切半径用于保证某距离内必碰，外接半径覆盖所有角点。inflation 将障碍代价向外传播，为机器人尺寸与误差留余量；圆形近似对非圆机器人可能过保守或不安全。

5. 可采纳表示 $h$ 不高估真实剩余代价，保证树搜索最优；一致表示对每条边满足三角不等式，使 f 值沿路径不下降，节点无需反复打开。

6. 八邻域允许以 $\sqrt2$ 代价同时减少横纵差，Manhattan 把它算成 2，会高估真实代价，破坏可采纳性。使用 octile 或欧氏距离。

7. Hybrid A* 将朝向加入状态，使用满足转向/曲率和前后运动的 primitive，在连续位姿中扩展，并加入运动学启发式和解析连接。

### 二、代码题

转换/Bresenham 测试覆盖地图原点、负原点、边界内外、cell center 往返、水平/垂直/对角线、反向射线、单点射线、陡斜率和八象限。A* 在随机小地图上用零启发式作为 Dijkstra 基准，比较可达性和成本；还需测试角切与代价绕行。

绑架状态机可由 tracking → suspicious → global_relocalizing → recovered/failed。连续低似然、激光地图错位、外部运动不一致触发 suspicious；避免单帧误触发。重定位时扩大/全局粒子并降低机器人速度，连续多帧几何一致后恢复；超时报告失败而不发布高置信错误位姿。

### 三、综合题

擦墙排查先确认实际 footprint、传感器和 base TF；比较定位真值/P95、局部控制跟踪误差；检查地图墙厚和外参；检查 global/local costmap 的障碍、清除和 inflation；回放全局路径每个姿态 footprint；再观察局部控制是否切角。地图清晰只说明视觉效果，不代表定位、代价或控制安全。

调小 inflation 只是允许搜索更靠近障碍，降低了安全余量。实机碰撞已经证明门槛不满足。应测门宽、机器人外廓、定位/控制误差分布和动态摆动，计算所需净空；在多方向、多速度和多次重复中报告最小距离/P95；几何不足则改变路线、降低外廓或增加传感/控制精度，而非继续缩小保护区。

## 本篇完成标准

完成本篇后，应能从带时间和位姿的激光构建 log-odds 地图；能解释未知、空闲和占用；能用粒子滤波表达多峰定位并处理绑架；能根据真实 footprint 和误差设计代价地图；能实现、测试和比较 Dijkstra/A*/Theta*；能判断何时需要 Hybrid A*；能用地图、定位、路径和安全净空指标而非截图验收系统。

下一阶段将进入局部规划、轨迹跟踪与 Nav2：Pure Pursuit、DWA/DWB、TEB/MPC 思想、动态障碍、行为树、恢复策略和自动化导航评测。
