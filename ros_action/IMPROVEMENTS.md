# 路径跟踪系统改进说明

## 🔧 主要改进

### 问题分析
原系统出现：
- ❌ 偏离越来越大（0.39m → 1.76m）
- ❌ 频繁重新规划但无效
- ❌ 实时Lattice规划计算量大
- ❌ 控制不稳定

### 解决方案

已将系统从**实时Lattice规划**改为**Waypoints跟踪 + Lattice修正**，参考了ros_action.py的成熟实现。

## ✨ 新特性

### 1. **Waypoints跟踪**
```python
# 初始生成半圆waypoints
waypoints = deque([60个点])

# 实时跟踪
- 找前视点
- 移除已过waypoints
- 纯跟踪控制
```

### 2. **智能偏离修正**
```python
# 只在偏离>阈值且间隔>3秒时重新规划
if deviation > 0.5m and time_gap > 3s:
    使用Quintic Polynomial回到圆上
```

### 3. **改进的控制算法**
```python
# 结合横向偏差的修正
heading_error += lateral_error_correction

# 自适应速度
speed = target_speed * (1 / (1 + error))

# 大转角自动减速
if heading_error > 1.0:
    speed *= max(0.4, cos(heading_error))
```

### 4. **参数优化**
| 参数 | 旧值 | 新值 | 说明 |
|------|------|------|------|
| `target_speed` | 0.3 | 0.4 | 提高速度 |
| `deviation_threshold` | 0.3 | 0.5 | 更宽容的偏离阈值 |
| `max_angular_speed` | 1.5 | 1.8 | 允许更快转向 |
| `lookahead_distance` | 0.3 | 0.4 | 更远的前视距离 |
| 重规划间隔 | 2s | 3s | 减少频繁重规划 |

## 📊 性能对比

### 旧系统（实时Lattice）
- ⚠️ 每20ms重新规划一次（计算密集）
- ⚠️ 偏离持续增大
- ⚠️ 频繁重规划无效
- ⚠️ 控制抖动

### 新系统（Waypoints + Lattice）
- ✅ 跟踪预生成waypoints（轻量级）
- ✅ 仅在偏离大时使用Lattice修正
- ✅ 稳定的纯跟踪控制
- ✅ 平滑的速度调整

## 🎯 使用方法

### 重新启动测试

```bash
# 终端1：启动节点
cd /home/kuko/humble_ws
source install/setup.bash
ros2 run ros_action lattice_follower
```

```bash
# 终端2：发送目标点
ros2 topic pub --once /target_point geometry_msgs/msg/Point "{x: 2.0, y: 2.0, z: 0.0}"
```

### 预期行为

1. **接收目标点**
   ```
   [INFO] 接收目标点: (2.00, 2.00)
   [INFO] 起点: (x, y)
   [INFO] 生成圆形轨迹: 中心=(x, y), 半径=r m
   [INFO] 初始路径: 60 个waypoints
   ```

2. **稳定跟踪**
   - 小车应该平滑沿半圆前进
   - 偏离应该保持在0.5m以内
   - 重规划次数大幅减少

3. **到达目标**
   ```
   [INFO] 已到达目标点！
   ```

## 🔍 监控命令

### 查看控制指令
```bash
ros2 topic echo /rm_0/cmd_vel
```
应该看到平滑的速度变化，不应该有剧烈抖动。

### 查看路径
```bash
# 参考路径（灰色虚线）
ros2 topic echo /reference_path

# 当前跟踪路径（蓝色实线）
ros2 topic echo /planned_path
```

### 查看位姿
```bash
ros2 topic echo /vrpn_mocap/rm_0_Test/pose
```

## ⚙️ 调试参数

如果还有问题，可以调整参数：

### 1. 降低速度（更稳定）
```bash
ros2 run ros_action lattice_follower --ros-args -p target_speed:=0.3
```

### 2. 增加偏离容忍度（减少重规划）
```bash
ros2 run ros_action lattice_follower --ros-args -p deviation_threshold:=0.8
```

### 3. 调整前视距离
```bash
# 更小 = 更精确但可能抖动
# 更大 = 更平滑但可能偏离
ros2 run ros_action lattice_follower --ros-args -p lookahead_distance:=0.5
```

### 4. 限制角速度（防止过度转向）
```bash
ros2 run ros_action lattice_follower --ros-args -p max_angular_speed:=1.2
```

## 🐛 如果还有问题

### 症状1：小车原地转圈
**可能原因：** 前视距离太小
**解决：**
```bash
-p lookahead_distance:=0.6
```

### 症状2：小车偏离太远
**可能原因：** 控制增益太小
**解决：** 在代码中增加 `kp` 值
```python
kp = 3.0  # 增大（原值2.0）
```

### 症状3：速度太慢/太快
**解决：**
```bash
-p target_speed:=0.5  # 调整速度
```

### 症状4：仍然频繁重规划
**可能原因：** 偏离阈值太小
**解决：**
```bash
-p deviation_threshold:=1.0  # 增大阈值
```

## 📚 关键代码位置

如需进一步调整，编辑这些位置：

### 控制增益
[lattice_path_follower.py:230](lattice_path_follower.py#L230)
```python
kp = 2.0  # 航向角比例增益
self.lateral_gain = 2.0  # 横向偏差增益
```

### 速度调整
[lattice_path_follower.py:240](lattice_path_follower.py#L240)
```python
speed_factor = 1.0 / (1.0 + error_magnitude)
```

### 重规划逻辑
[lattice_path_follower.py:250](lattice_path_follower.py#L250)
```python
if deviation > self.deviation_threshold and time_gap > 3.0:
```

## 📈 预期改进效果

- ✅ **偏离稳定**：保持在0.3-0.5m范围内
- ✅ **重规划减少**：从每2秒到每10-20秒（或更少）
- ✅ **控制平滑**：无剧烈抖动
- ✅ **成功到达**：稳定完成半圆轨迹

## 🎓 核心改进思想

原系统问题：**实时规划 + 不稳定控制 = 累积误差**

新系统策略：**稳定跟踪 + 偶尔修正 = 收敛误差**

参考了ros_action.py中的：
- ✅ Waypoints管理
- ✅ 纯跟踪控制
- ✅ 横向偏差修正
- ✅ 自适应速度

---

**更新时间**: 2026-01-21  
**测试状态**: 已编译，待测试
