# Lattice路径跟踪系统 - 使用指南

## 🎯 系统概述

基于VRPN动捕系统和Lattice Planner的实时路径跟踪系统，实现差分驱动小车的半圆轨迹跟踪和实时路径修正。

## 📋 系统架构

```
VRPN动捕系统 (/vrpn_mocap/rm_0_Test/pose)
     ↓
   位姿信息 (x, y, θ)
     ↓
目标点 (/target_point) → 生成圆形参考轨迹
     ↓
Lattice Planner (实时规划)
     ↓
差分控制器 (/rm_0/cmd_vel)
     ↓
   小车运动
```

## 🚀 快速开始

### 1. 编译功能包

```bash
cd /home/kuko/humble_ws
colcon build --packages-select ros_action
source install/setup.bash
```

### 2. 启动系统

#### 方法A：使用启动文件（推荐）

```bash
ros2 launch ros_action lattice_follower.launch.py
```

#### 方法B：直接运行节点

```bash
ros2 run ros_action lattice_follower
```

#### 方法C：自定义参数

```bash
ros2 run ros_action lattice_follower --ros-args \
    -p target_speed:=0.4 \
    -p control_frequency:=20.0 \
    -p deviation_threshold:=0.3 \
    -p max_angular_speed:=1.5 \
    -p lookahead_distance:=0.3
```

### 3. 发送目标点

在另一个终端中：

```bash
# 方法1：使用send_target节点（交互式）
ros2 run ros_action send_target

# 方法2：直接发布消息
ros2 topic pub --once /target_point geometry_msgs/msg/Point "{x: 2.0, y: 2.0, z: 0.0}"
```

### 4. 可视化（RViz）

```bash
rviz2
```

在RViz中添加以下显示：
- **Path** (Topic: `/reference_path`) - 参考轨迹（灰色虚线）
- **Path** (Topic: `/planned_path`) - 规划路径（蓝色实线）
- **PoseStamped** (Topic: `/vrpn_mocap/rm_0_Test/pose`) - 小车位置

## 📊 话题说明

### 订阅的话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/target_point` | geometry_msgs/Point | 目标点坐标 |
| `/vrpn_mocap/rm_0_Test/pose` | geometry_msgs/PoseStamped | VRPN动捕位姿（BEST_EFFORT QoS） |

### 发布的话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/rm_0/cmd_vel` | geometry_msgs/Twist | 差分驱动控制指令 |
| `/planned_path` | nav_msgs/Path | 规划的路径（可视化） |
| `/reference_path` | nav_msgs/Path | 参考轨迹（可视化） |

## ⚙️ 参数配置

### 节点参数

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `target_speed` | 0.3 | 目标速度 (m/s) |
| `control_frequency` | 20.0 | 控制频率 (Hz) |
| `deviation_threshold` | 0.3 | 偏离阈值 (m)，超过时重新规划 |
| `max_angular_speed` | 1.5 | 最大角速度 (rad/s) |
| `lookahead_distance` | 0.3 | 前视距离 (m)，用于纯跟踪控制 |

### 规划器参数

在 `simple_lattice_planner.py` 的 `PlannerConfig` 类中修改：

```python
class PlannerConfig:
    MAX_SPEED = 1.0         # 最大速度 (m/s)
    MAX_ACCEL = 2.0         # 最大加速度 (m/s²)
    MAX_CURVATURE = 1.0     # 最大曲率 (1/m)
    ROAD_WIDTH = 0.5        # 道路宽度 (m)
    
    # 代价函数权重
    K_JERK = 0.1            # 急动度权重（平滑性）
    K_TIME = 0.1            # 时间权重（效率）
    K_V_DIFF = 1.0          # 速度差权重（速度跟踪）
    K_OFFSET = 2.0          # 偏移权重（保持车道中心）
    K_COLLISION = 500       # 碰撞权重（安全性）
```

## 🔄 工作流程

### 1. 初始化阶段

```
启动节点 → 等待VRPN位姿 → 等待目标点
```

节点日志：
```
[INFO] Lattice路径跟踪节点已启动
[INFO] 等待目标点和位姿数据...
```

### 2. 接收目标点

```
接收 /target_point → 记录起点位置 → 生成圆形参考轨迹 → 初始化规划器
```

节点日志：
```
[INFO] 接收目标点: (2.00, 2.00)
[INFO] 起点: (0.00, 0.00)
[INFO] 生成圆形轨迹: 中心=(1.00, 1.00), 半径=1.41m
[INFO] 参考轨迹生成完成，开始跟踪
[INFO] 参考路径已发布
```

### 3. 实时跟踪

```
while 未到达目标:
    1. 更新位姿（来自VRPN）
    2. 转换到Frenet坐标 (s, l)
    3. Lattice Planner规划路径
    4. 计算差分控制指令
    5. 发布到 /rm_0/cmd_vel
    6. 检查偏离情况
```

控制频率：20 Hz（每50ms一次）

### 4. 偏离检测与修正

```
每1秒检查一次偏离：
if 横向偏离 > 阈值 (0.3m):
    重新规划路径（最少间隔2秒）
```

节点日志：
```
[INFO] 检测到偏离 0.35m，重新规划路径
```

### 5. 到达目标

```
if 距离目标点 < 0.15m:
    停止运动
    完成任务
```

节点日志：
```
[INFO] 已到达目标点！
```

## 🎮 控制算法

### 差分驱动控制

**线速度计算：**
```python
base_speed = 规划速度
speed_factor = 1.0 - min(|航向误差| / π, 0.7)
linear_x = base_speed * max(speed_factor, 0.3)
```

**角速度计算：**
```python
yaw_error = 目标航向 - 当前航向
angular_z = kp_angular * yaw_error
angular_z = clip(angular_z, -max_angular_speed, max_angular_speed)
```

**参数：**
- `kp_angular = 2.5` - 角速度比例增益
- 自动减速机制：航向误差大时减速，保持至少30%速度

## 📈 性能调优

### 速度调整

- **提高速度**：增加 `target_speed` 参数
- **更平滑**：增加 `K_JERK` 权重，降低 `control_frequency`
- **更激进转向**：增加 `max_angular_speed`

### 跟踪精度

- **更精确跟踪**：降低 `lookahead_distance`，增加 `K_OFFSET`
- **更快响应**：降低 `deviation_threshold`
- **减少抖动**：增加 `lookahead_distance`

### 代价权重调整

```python
# 更平滑的轨迹
K_JERK = 0.5      # 增加

# 更快到达目标
K_TIME = 0.05     # 降低

# 更好的速度跟踪
K_V_DIFF = 2.0    # 增加

# 更接近参考线
K_OFFSET = 3.0    # 增加
```

## 🐛 故障排除

### 问题1：节点启动后没有反应

**可能原因：**
- VRPN系统未启动
- 小车名称不匹配

**解决方案：**
```bash
# 检查VRPN话题
ros2 topic list | grep vrpn

# 检查位姿数据
ros2 topic echo /vrpn_mocap/rm_0_Test/pose --once
```

### 问题2：小车不动

**可能原因：**
- 未发送目标点
- 目标点与起点重合

**解决方案：**
```bash
# 检查目标点
ros2 topic echo /target_point

# 发送明确的目标点
ros2 topic pub --once /target_point geometry_msgs/msg/Point "{x: 2.0, y: 2.0, z: 0.0}"
```

### 问题3：规划失败

**节点日志：**
```
[WARN] 路径规划失败
```

**可能原因：**
- 参数设置不合理（如ROAD_WIDTH太小）
- 目标点太近

**解决方案：**
```python
# 调整PlannerConfig
ROAD_WIDTH = 0.8  # 增加
MAX_SPEED = 1.5   # 增加
```

### 问题4：小车偏离过大

**可能原因：**
- 速度过快
- 控制频率太低
- 前视距离不合适

**解决方案：**
```bash
# 降低速度
-p target_speed:=0.2

# 提高控制频率
-p control_frequency:=30.0

# 调整前视距离
-p lookahead_distance:=0.2
```

### 问题5：小车震荡

**可能原因：**
- 角速度增益太大
- 前视距离太小

**解决方案：**
```python
# 在lattice_path_follower.py中调整
kp_angular = 1.5  # 降低（原值2.5）

# 或增加前视距离
-p lookahead_distance:=0.5
```

## 📝 日志监控

### 关键日志信息

```bash
# 启动成功
[INFO] Lattice路径跟踪节点已启动
[INFO] 等待目标点和位姿数据...

# 接收目标
[INFO] 接收目标点: (x, y)
[INFO] 起点: (x, y)
[INFO] 生成圆形轨迹: 中心=(x, y), 半径=r m

# 跟踪中
[INFO] 参考轨迹生成完成，开始跟踪

# 偏离修正
[INFO] 检测到偏离 d m，重新规划路径

# 完成
[INFO] 已到达目标点！

# 警告
[WARN] 路径规划失败
[WARN] 规划路径太短
[WARN] 尚未接收到位姿数据，等待中...
```

### 实时监控命令

```bash
# 监控控制指令
ros2 topic echo /rm_0/cmd_vel

# 监控位姿
ros2 topic echo /vrpn_mocap/rm_0_Test/pose

# 监控节点日志
ros2 node list
ros2 node info /lattice_path_follower
```

## 🔬 测试场景

### 场景1：直线测试

```bash
# 起点: (0, 0)
# 目标: (2, 0)
ros2 topic pub --once /target_point geometry_msgs/msg/Point "{x: 2.0, y: 0.0, z: 0.0}"
```

预期：半圆轨迹，中心(1, 0)，半径1m

### 场景2：对角线测试

```bash
# 起点: (0, 0)
# 目标: (2, 2)
ros2 topic pub --once /target_point geometry_msgs/msg/Point "{x: 2.0, y: 2.0, z: 0.0}"
```

预期：半圆轨迹，中心(1, 1)，半径√2 m

### 场景3：短距离测试

```bash
# 起点: (0, 0)
# 目标: (0.5, 0.5)
ros2 topic pub --once /target_point geometry_msgs/msg/Point "{x: 0.5, y: 0.5, z: 0.0}"
```

预期：半圆轨迹，中心(0.25, 0.25)，半径0.35m

## 🎓 扩展建议

1. **添加障碍物避让**
   ```python
   config.obstacles = [(x, y, radius), ...]
   ```

2. **实现速度规划**
   - 根据曲率自动调整速度
   - 加速/减速段规划

3. **多目标点规划**
   - 连续接收目标点
   - 实现路径拼接

4. **性能监控**
   - 记录跟踪误差
   - 统计到达时间
   - 分析控制平滑度

5. **添加其他轨迹类型**
   - 直线轨迹
   - 三次样条轨迹
   - 贝塞尔曲线

## 📞 技术支持

如有问题，请检查：
1. 所有依赖是否安装
2. 参数配置是否合理
3. 话题连接是否正常
4. 日志信息中的警告和错误

---

**更新日期**: 2026-01-21  
**版本**: 1.0  
**适用于**: ROS 2 Humble + VRPN动捕系统
