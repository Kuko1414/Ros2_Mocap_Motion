# Lattice Planner 路径跟踪项目

## 📁 项目结构

```
ros_action/
├── curve_generators/          # 曲线生成器模块
│   ├── __init__.py
│   └── circle_trajectory.py   # 圆形轨迹生成器
├── simple_lattice_planner.py  # 简化版Lattice规划器
├── lattice_path_follower.py   # ROS 2路径跟踪节点
└── ...
```

## 🎯 功能说明

### 1. CurveGenerator（曲线生成器）

**circle_trajectory.py** - 圆形轨迹生成器
- 生成指定圆心和半径的圆形参考轨迹
- 提供Frenet坐标查询接口
- 可扩展为其他曲线类型（直线、椭圆、样条等）

**主要方法：**
- `calc_position(s)` - 根据弧长查询位置
- `calc_yaw(s)` - 根据弧长查询航向角
- `calc_curvature(s)` - 根据弧长查询曲率
- `get_trajectory()` - 获取完整轨迹数据

### 2. SimpleLattice Planner（简化规划器）

**simple_lattice_planner.py** - 基于采样的局部路径规划器
- 使用Frenet坐标系进行路径规划
- 采样多条候选路径并选择最优路径
- 综合考虑舒适性、效率、安全性

**核心特性：**
- ✅ 横向和纵向解耦规划
- ✅ 多项式轨迹生成（四次/五次多项式）
- ✅ 多目标代价函数优化
- ✅ 车辆约束验证（速度、加速度、曲率）
- ✅ 碰撞检测（可添加障碍物）

### 3. ROS 2 节点（实时路径跟踪）

**lattice_path_follower.py** - 边走边修正的路径跟踪节点
- 订阅机器人里程计信息
- 实时规划并发布路径
- 发布速度控制指令

## 🚀 使用方法

### 方法1：独立测试规划器

```bash
cd /home/kuko/humble_ws/src/ros_action/ros_action
python3 simple_lattice_planner.py
```

这将运行测试程序并生成可视化图像。

### 方法2：ROS 2节点运行

1. **编译功能包**
```bash
cd /home/kuko/humble_ws
colcon build --packages-select ros_action
source install/setup.bash
```

2. **运行节点**
```bash
ros2 run ros_action lattice_follower
```

或使用启动文件：
```bash
ros2 launch ros_action lattice_follower.launch.py
```

3. **修改参数**
```bash
ros2 run ros_action lattice_follower --ros-args \
    -p circle_center_x:=2.0 \
    -p circle_center_y:=2.0 \
    -p circle_radius:=3.0 \
    -p target_speed:=0.8 \
    -p control_frequency:=20.0
```

### 方法3：在RViz中可视化

```bash
# 终端1：运行节点
ros2 run ros_action lattice_follower

# 终端2：启动RViz
rviz2
```

在RViz中添加：
- **Path** - Topic: `/reference_path` (参考轨迹，灰色虚线)
- **Path** - Topic: `/planned_path` (规划路径，蓝色实线)
- **Odometry** - Topic: `/odom` (机器人位置)

## 📊 话题说明

### 订阅的话题
- `/odom` (nav_msgs/Odometry) - 机器人里程计信息

### 发布的话题
- `/cmd_vel` (geometry_msgs/Twist) - 速度控制指令
- `/planned_path` (nav_msgs/Path) - 规划的路径
- `/reference_path` (nav_msgs/Path) - 参考轨迹

## ⚙️ 参数配置

### 节点参数
- `circle_center_x` (默认: 1.0) - 圆心x坐标
- `circle_center_y` (默认: 1.0) - 圆心y坐标
- `circle_radius` (默认: 2.0) - 圆半径
- `target_speed` (默认: 0.5) - 目标速度 m/s
- `control_frequency` (默认: 10.0) - 控制频率 Hz

### 规划器参数（PlannerConfig类）
在 `simple_lattice_planner.py` 中修改：
```python
class PlannerConfig:
    MAX_SPEED = 1.0         # 最大速度
    MAX_ACCEL = 2.0         # 最大加速度
    MAX_CURVATURE = 1.0     # 最大曲率
    ROAD_WIDTH = 0.5        # 道路宽度
    TARGET_SPEED = 0.5      # 目标速度
    K_JERK = 0.1            # 急动度权重
    K_TIME = 0.1            # 时间权重
    K_V_DIFF = 1.0          # 速度差权重
    K_OFFSET = 2.0          # 偏移权重
    K_COLLISION = 500       # 碰撞权重
```

## 🔧 扩展其他曲线

### 添加直线轨迹生成器

在 `curve_generators/` 文件夹中创建 `line_trajectory.py`:

```python
class LineTrajectory:
    def __init__(self, start_x, start_y, end_x, end_y):
        # 实现直线轨迹生成
        pass
```

### 添加样条曲线生成器

可以参考原始代码中的 `cubic_spline.py`，创建更复杂的轨迹。

## 🎨 算法原理

### Lattice Planner工作流程

1. **状态输入** - 接收当前位置、速度、加速度（Frenet坐标）
2. **采样生成** - 生成多条候选路径
   - 纵向：四次多项式（约束速度和加速度）
   - 横向：五次多项式（约束位置、速度、加速度）
3. **坐标转换** - Frenet → 笛卡尔坐标
4. **代价评估** - 计算每条路径的代价
   - 平滑性（急动度）
   - 时间效率
   - 速度跟踪
   - 车道保持
   - 碰撞避免
5. **约束验证** - 检查速度、加速度、曲率限制
6. **最优选择** - 选择代价最小且满足约束的路径
7. **执行控制** - 执行路径的第一步，更新状态

### Frenet坐标系

- **s** - 沿参考线的纵向距离（弧长）
- **l** - 垂直于参考线的横向距离
- **优势**：解耦纵向和横向运动，简化规划

## 📝 调试技巧

### 1. 查看规划器输出
```bash
ros2 topic echo /planned_path
```

### 2. 检查速度指令
```bash
ros2 topic echo /cmd_vel
```

### 3. 调整代价权重
如果路径不理想，可以调整代价函数权重：
- 增加 `K_OFFSET` → 更接近车道中心
- 增加 `K_JERK` → 更平滑的轨迹
- 增加 `K_V_DIFF` → 更好的速度跟踪

### 4. 添加障碍物测试
在代码中设置障碍物：
```python
config = PlannerConfig()
config.obstacles = [
    (2.0, 2.0, 0.3),  # (x, y, radius)
    (3.0, 1.0, 0.2),
]
```

## 📚 参考

- 原始Lattice Planner: `/src_learn/MotionPlanning-master/LatticePlanner/`
- Frenet坐标系详解: [Optimal Trajectory Generation for Dynamic Street Scenarios](https://arxiv.org/abs/1003.1990)
- 多项式轨迹生成: 五次多项式可以完全约束位置、速度、加速度

## 🐛 常见问题

**Q: 规划失败怎么办？**
A: 检查参数设置，特别是 `ROAD_WIDTH` 和 `TARGET_SPEED`，确保它们适合你的应用场景。

**Q: 小车转弯太急？**
A: 降低 `MAX_CURVATURE` 或增加 `K_JERK` 权重。

**Q: 速度不稳定？**
A: 增加 `K_V_DIFF` 权重，或降低 `control_frequency`。

**Q: 如何添加其他轨迹？**
A: 在 `curve_generators/` 中创建新的轨迹类，实现相同的接口方法。

## ✨ 下一步改进

- [ ] 添加动态障碍物支持
- [ ] 实现更复杂的轨迹生成器（样条、贝塞尔曲线）
- [ ] 添加路径预测和前瞻控制
- [ ] 集成MPC控制器
- [ ] 添加参数动态配置（rqt_reconfigure）
- [ ] 性能优化（C++重写核心算法）

---

**作者**: Kuko  
**日期**: 2026-01-21  
**基于**: MotionPlanning-master/LatticePlanner
