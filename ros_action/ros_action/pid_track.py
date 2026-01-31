#!/usr/bin/env python3
"""
PID路径跟踪节点：订阅路径，使用PID算法跟踪圆形路径
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float64
import math


class PIDTracker(Node):
    def __init__(self):
        super().__init__('pid_tracker')
        
        # 参数设置
        self.target_speed = 0.33  # 目标速度 m/s
        self.radius_error_threshold = 0.05  # 半径偏差阈值 (米)
        
        # PID参数 - 用于半径误差修正
        self.kp_radius = 2.5  # 半径误差比例系数
        self.ki_radius = 1.0  # 积分系数
        self.kd_radius = 0.5  # 微分系数
        self.integral_error = 0.1  # 积分误差
        self.last_error = 0.0  # 上一次误差
        
        # 角度PID参数 - 用于朝向修正
        self.kp_angle = 2.0  # 角度误差比例系数
        
        # 状态变量
        self.current_pose = None  # 当前位置
        self.circle_path = []  # 圆形路径点列表
        self.circle_center = None  # 圆心
        self.circle_radius = None  # 圆的半径
        self.path_ready = False  # 路径是否准备好
        
        # 设置QoS配置以匹配VRPN mocap
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅当前位置（使用BEST_EFFORT QoS）
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/rm_0_Test/pose',
            self.pose_callback,
            qos_profile
        )
        
        # 订阅路径
        self.path_sub = self.create_subscription(
            Path,
            '/circle_path',
            self.path_callback,
            10
        )
        
        # 订阅圆心
        self.center_sub = self.create_subscription(
            Point,
            '/circle_center',
            self.center_callback,
            10
        )
        
        # 订阅半径
        self.radius_sub = self.create_subscription(
            Float64,
            '/circle_radius',
            self.radius_callback,
            10
        )
        
        # 发布速度命令
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/rm_0/cmd_vel',
            10
        )
        
        # 发布可视化Marker
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/pid_tracker_markers',
            10
        )
        
        # 定时器：控制循环 (50Hz)
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        # 定时器：可视化更新 (10Hz)
        self.viz_timer = self.create_timer(0.1, self.publish_visualization)
        
        self.get_logger().info('PIDTracker节点已启动')
        self.get_logger().info('等待路径 (/circle_path)...')

    def pose_callback(self, msg: PoseStamped):
        """接收当前位置"""
        self.current_pose = msg.pose

    def path_callback(self, msg: Path):
        """接收路径（只接收第一次）"""
        if self.path_ready:
            return  # 已有路径，忽略
        
        if len(msg.poses) == 0:
            return
        
        # 存储路径点
        self.circle_path = []
        for pose in msg.poses:
            self.circle_path.append({
                'x': pose.pose.position.x,
                'y': pose.pose.position.y
            })
        
        self.get_logger().info(f'收到路径，共 {len(self.circle_path)} 个点')
        
        # 检查是否所有数据都已就绪
        self.check_ready()

    def center_callback(self, msg: Point):
        """接收圆心"""
        if self.path_ready:
            return
        
        self.circle_center = {
            'x': msg.x,
            'y': msg.y
        }
        self.get_logger().info(f'收到圆心: ({msg.x:.3f}, {msg.y:.3f})')
        self.check_ready()

    def radius_callback(self, msg: Float64):
        """接收半径"""
        if self.path_ready:
            return
        
        self.circle_radius = msg.data
        self.get_logger().info(f'收到半径: {msg.data:.3f}')
        self.check_ready()

    def check_ready(self):
        """检查是否所有数据都已就绪"""
        if len(self.circle_path) > 0 and self.circle_center is not None and self.circle_radius is not None:
            self.path_ready = True
            self.get_logger().info('路径数据已完整，开始跟踪！')

    def find_nearest_path_index(self):
        """找到当前位置最近的路径点索引"""
        if self.current_pose is None or len(self.circle_path) == 0:
            return 0
        
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        min_dist = float('inf')
        nearest_idx = 0
        
        for i, point in enumerate(self.circle_path):
            dist = math.sqrt((current_x - point['x'])**2 + (current_y - point['y'])**2)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        
        return nearest_idx

    def get_yaw_from_quaternion(self, orientation):
        """从四元数获取yaw角"""
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw

    def normalize_angle(self, angle):
        """将角度归一化到 [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def control_loop(self):
        """控制循环：沿圆形路径行驶，带PID修正"""
        if not self.path_ready:
            return
        
        if self.current_pose is None:
            return
        
        # 获取当前位置和朝向
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        
        # 计算当前位置相对于圆心的角度
        dx = current_x - self.circle_center['x']
        dy = current_y - self.circle_center['y']
        angle_from_center = math.atan2(dy, dx)
        
        # 计算当前位置到圆心的距离
        distance_to_center = math.sqrt(dx**2 + dy**2)
        
        # 计算半径误差（正值表示在圆外，负值表示在圆内）
        radius_error = distance_to_center - self.circle_radius
        
        # ====== 关键修正：计算圆上的投影点，然后从投影点计算前瞻目标 ======
        # 圆上最近的点（小车在圆上的投影）
        proj_x = self.circle_center['x'] + self.circle_radius * math.cos(angle_from_center)
        proj_y = self.circle_center['y'] + self.circle_radius * math.sin(angle_from_center)
        
        # 前瞻距离（沿圆弧的距离，单位：米）
        lookahead_dist = 0.01  # 1cm前瞻
        # 转换为角度：angle = arc_length / radius
        lookahead_angle = lookahead_dist / self.circle_radius
        
        # 目标点在圆上，从投影点向前（逆时针方向）
        target_angle_on_circle = angle_from_center + lookahead_angle
        target_x = self.circle_center['x'] + self.circle_radius * math.cos(target_angle_on_circle)
        target_y = self.circle_center['y'] + self.circle_radius * math.sin(target_angle_on_circle)
        
        # ====== 加入横向误差修正：把目标点稍微向圆心/外侧偏移 ======
        # 如果小车在圆外(radius_error > 0)，目标点应该向圆心方向偏移，让小车向内切
        # 如果小车在圆内(radius_error < 0)，目标点应该向外偏移，让小车向外切
        correction_gain = 2.0  # 修正增益
        correction_dist = -radius_error * correction_gain  # 注意负号：在圆外时向内修正
        
        # 修正后的目标点（在目标点的基础上沿半径方向偏移）
        target_radius = self.circle_radius + correction_dist
        target_x = self.circle_center['x'] + target_radius * math.cos(target_angle_on_circle)
        target_y = self.circle_center['y'] + target_radius * math.sin(target_angle_on_circle)
        
        # 计算从当前位置到目标点的方向
        dx_to_target = target_x - current_x
        dy_to_target = target_y - current_y
        angle_to_target = math.atan2(dy_to_target, dx_to_target)
        
        # 计算当前朝向与目标方向的角度误差
        angle_error = self.normalize_angle(angle_to_target - current_yaw)
        
        # 基础角速度（沿圆运动：ω = v / r）
        base_angular_speed = self.target_speed / self.circle_radius
        
        # 角度误差修正
        angle_correction = self.kp_angle * angle_error
        
        # 创建速度命令
        cmd_vel = Twist()
        
        # 线速度：当角度误差太大时减速
        if abs(angle_error) > math.pi / 2:  # 90度
            cmd_vel.linear.x = 0.0
        elif abs(angle_error) > math.pi / 3:  # 60度
            cmd_vel.linear.x = self.target_speed * 0.3
        elif abs(angle_error) > math.pi / 6:  # 30度
            cmd_vel.linear.x = self.target_speed * 0.6
        else:
            cmd_vel.linear.x = self.target_speed
        
        # 角速度 = 角度修正 + 基础角速度（仅当角度对齐时）
        if abs(angle_error) > math.pi / 6:
            cmd_vel.angular.z = angle_correction
        else:
            cmd_vel.angular.z = angle_correction + base_angular_speed
        
        # 限制角速度
        max_angular_speed = 2.0
        cmd_vel.angular.z = max(-max_angular_speed, min(max_angular_speed, cmd_vel.angular.z))
        
        # 发布速度命令
        self.cmd_vel_pub.publish(cmd_vel)
        
        # 保存误差供下次使用
        self.last_error = radius_error

    def publish_visualization(self):
        """发布可视化Marker到rviz2"""
        if not self.path_ready or self.current_pose is None:
            return
        
        marker_array = MarkerArray()
        
        # 1. 蓝色小车方块
        car_marker = Marker()
        car_marker.header.frame_id = "world"
        car_marker.header.stamp = self.get_clock().now().to_msg()
        car_marker.ns = "car"
        car_marker.id = 0
        car_marker.type = Marker.CUBE
        car_marker.action = Marker.ADD
        car_marker.pose.position.x = self.current_pose.position.x
        car_marker.pose.position.y = self.current_pose.position.y
        car_marker.pose.position.z = 0.05
        car_marker.pose.orientation = self.current_pose.orientation
        car_marker.scale.x = 0.15
        car_marker.scale.y = 0.10
        car_marker.scale.z = 0.05
        car_marker.color.r = 0.0
        car_marker.color.g = 0.0
        car_marker.color.b = 1.0
        car_marker.color.a = 1.0
        
        marker_array.markers.append(car_marker)
        
        # 2. 浅蓝色未来路径
        future_marker = Marker()
        future_marker.header.frame_id = "world"
        future_marker.header.stamp = self.get_clock().now().to_msg()
        future_marker.ns = "future_path"
        future_marker.id = 1
        future_marker.type = Marker.LINE_STRIP
        future_marker.action = Marker.ADD
        future_marker.scale.x = 0.03
        future_marker.color.r = 0.0
        future_marker.color.g = 0.5
        future_marker.color.b = 1.0
        future_marker.color.a = 0.8
        future_marker.pose.orientation.w = 1.0
        
        nearest_idx = self.find_nearest_path_index()
        num_future_points = min(100, len(self.circle_path))
        
        for i in range(num_future_points):
            idx = (nearest_idx + i) % len(self.circle_path)
            p = Point()
            p.x = self.circle_path[idx]['x']
            p.y = self.circle_path[idx]['y']
            p.z = 0.01
            future_marker.points.append(p)
        
        marker_array.markers.append(future_marker)
        
        # 3. 半径误差指示线
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        distance_to_center = math.sqrt(
            (current_x - self.circle_center['x'])**2 + 
            (current_y - self.circle_center['y'])**2
        )
        radius_error = abs(distance_to_center - self.circle_radius)
        
        radius_marker = Marker()
        radius_marker.header.frame_id = "world"
        radius_marker.header.stamp = self.get_clock().now().to_msg()
        radius_marker.ns = "radius_error"
        radius_marker.id = 2
        radius_marker.type = Marker.LINE_STRIP
        radius_marker.action = Marker.ADD
        radius_marker.scale.x = 0.01
        radius_marker.pose.orientation.w = 1.0
        
        # 根据误差大小显示不同颜色
        if radius_error < self.radius_error_threshold:
            radius_marker.color.r = 0.0
            radius_marker.color.g = 1.0
            radius_marker.color.b = 0.0  # 绿色
        elif radius_error < self.radius_error_threshold * 2:
            radius_marker.color.r = 1.0
            radius_marker.color.g = 1.0
            radius_marker.color.b = 0.0  # 黄色
        else:
            radius_marker.color.r = 1.0
            radius_marker.color.g = 0.0
            radius_marker.color.b = 0.0  # 红色
        radius_marker.color.a = 1.0
        
        p1 = Point()
        p1.x = self.circle_center['x']
        p1.y = self.circle_center['y']
        p1.z = 0.0
        radius_marker.points.append(p1)
        
        p2 = Point()
        p2.x = current_x
        p2.y = current_y
        p2.z = 0.0
        radius_marker.points.append(p2)
        
        marker_array.markers.append(radius_marker)
        
        # 4. 文字显示半径误差
        text_marker = Marker()
        text_marker.header.frame_id = "world"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "error_text"
        text_marker.id = 3
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = self.circle_center['x']
        text_marker.pose.position.y = self.circle_center['y']
        text_marker.pose.position.z = 0.3
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.1
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = f"Error: {radius_error:.3f}m"
        
        marker_array.markers.append(text_marker)
        
        # 5. 绿色箭头显示目标点方向
        dx = current_x - self.circle_center['x']
        dy = current_y - self.circle_center['y']
        angle_from_center = math.atan2(dy, dx)
        
        # 计算前瞻目标点
        lookahead_angle = 0.2
        target_angle = angle_from_center + lookahead_angle
        target_x = self.circle_center['x'] + self.circle_radius * math.cos(target_angle)
        target_y = self.circle_center['y'] + self.circle_radius * math.sin(target_angle)
        
        # 计算到目标点的方向
        dx_to_target = target_x - current_x
        dy_to_target = target_y - current_y
        angle_to_target = math.atan2(dy_to_target, dx_to_target)
        
        tangent_marker = Marker()
        tangent_marker.header.frame_id = "world"
        tangent_marker.header.stamp = self.get_clock().now().to_msg()
        tangent_marker.ns = "tangent"
        tangent_marker.id = 4
        tangent_marker.type = Marker.ARROW
        tangent_marker.action = Marker.ADD
        tangent_marker.scale.x = 0.2  # 箭头长度
        tangent_marker.scale.y = 0.03  # 箭头宽度
        tangent_marker.scale.z = 0.03
        tangent_marker.color.r = 0.0
        tangent_marker.color.g = 1.0
        tangent_marker.color.b = 0.0
        tangent_marker.color.a = 1.0
        tangent_marker.pose.position.x = current_x
        tangent_marker.pose.position.y = current_y
        tangent_marker.pose.position.z = 0.1
        # 设置箭头朝向目标点方向
        tangent_marker.pose.orientation.z = math.sin(angle_to_target / 2.0)
        tangent_marker.pose.orientation.w = math.cos(angle_to_target / 2.0)
        
        marker_array.markers.append(tangent_marker)
        
        # 6. 黄色球显示目标点位置
        target_marker = Marker()
        target_marker.header.frame_id = "world"
        target_marker.header.stamp = self.get_clock().now().to_msg()
        target_marker.ns = "target_point"
        target_marker.id = 6
        target_marker.type = Marker.SPHERE
        target_marker.action = Marker.ADD
        target_marker.pose.position.x = target_x
        target_marker.pose.position.y = target_y
        target_marker.pose.position.z = 0.05
        target_marker.pose.orientation.w = 1.0
        target_marker.scale.x = 0.05
        target_marker.scale.y = 0.05
        target_marker.scale.z = 0.05
        target_marker.color.r = 1.0
        target_marker.color.g = 1.0
        target_marker.color.b = 0.0
        target_marker.color.a = 1.0
        
        marker_array.markers.append(target_marker)
        
        # 6. 红色箭头显示当前小车朝向
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        
        heading_marker = Marker()
        heading_marker.header.frame_id = "world"
        heading_marker.header.stamp = self.get_clock().now().to_msg()
        heading_marker.ns = "heading"
        heading_marker.id = 5
        heading_marker.type = Marker.ARROW
        heading_marker.action = Marker.ADD
        heading_marker.scale.x = 0.15  # 箭头长度
        heading_marker.scale.y = 0.025
        heading_marker.scale.z = 0.025
        heading_marker.color.r = 1.0
        heading_marker.color.g = 0.0
        heading_marker.color.b = 0.0
        heading_marker.color.a = 1.0
        heading_marker.pose.position.x = current_x
        heading_marker.pose.position.y = current_y
        heading_marker.pose.position.z = 0.12
        heading_marker.pose.orientation.z = math.sin(current_yaw / 2.0)
        heading_marker.pose.orientation.w = math.cos(current_yaw / 2.0)
        
        marker_array.markers.append(heading_marker)
        
        self.marker_pub.publish(marker_array)

    def stop_robot(self):
        """停止机器人"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info('机器人已停止')


def main(args=None):
    rclpy.init(args=args)
    
    node = PIDTracker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到键盘中断，正在停止...')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
