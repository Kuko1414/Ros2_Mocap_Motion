#!/usr/bin/env python3
"""
路径生成节点：接收目标点，生成圆形路径并发布
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float64
import math


class PathGeneration(Node):
    def __init__(self):
        super().__init__('path_generation')
        
        # 状态变量
        self.current_pose = None  # 当前位置（持续更新）
        self.start_position = None  # 起始位置（收到target point时记录，不变）
        self.target_point = None  # 目标点
        self.circle_center = None  # 圆心
        self.circle_radius = None  # 圆的半径
        self.circle_path = []  # 圆形路径点列表
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
        
        # 发布路径（供pid_track订阅）
        self.path_pub = self.create_publisher(
            Path,
            '/circle_path',
            10
        )
        
        # 发布圆的半径（供pid_track使用）
        self.radius_pub = self.create_publisher(
            Float64,
            '/circle_radius',
            10
        )
        
        # 发布圆心位置（供pid_track使用）
        self.center_pub = self.create_publisher(
            Point,
            '/circle_center',
            10
        )
        
        # 发布可视化Marker（用于rviz2）
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/circle_path_markers',
            10
        )
        
        # 定时器：持续发布路径 (10Hz)
        self.publish_timer = self.create_timer(0.1, self.publish_path_and_viz)
        
        self.get_logger().info('PathGeneration节点已启动')
        self.get_logger().info('等待当前位置 (/vrpn_mocap/rm_0_Test/pose)...')
        self.get_logger().info('输入目标点格式: x y (用空格分隔)')
        self.get_logger().info('输入 "q" 退出')

    def pose_callback(self, msg: PoseStamped):
        """接收当前位置"""
        self.current_pose = msg.pose

    def set_target_point(self, x: float, y: float):
        """设置目标点并生成路径（只接受第一次）"""
        if self.path_ready:
            self.get_logger().info('路径已生成，忽略新的目标点')
            return False
        
        if self.current_pose is None:
            self.get_logger().warn('尚未收到当前位置，无法生成路径')
            return False
        
        # 记录目标点
        self.target_point = Point()
        self.target_point.x = x
        self.target_point.y = y
        self.target_point.z = 0.0
        self.get_logger().info(f'收到目标点: ({x:.3f}, {y:.3f})')
        
        # 记录起始位置
        self.start_position = {
            'x': self.current_pose.position.x,
            'y': self.current_pose.position.y
        }
        self.get_logger().info(f'记录起始位置: ({self.start_position["x"]:.3f}, {self.start_position["y"]:.3f})')
        
        # 计算圆形路径
        self.calculate_circle_path()
        return True

    def calculate_circle_path(self):
        """计算圆形路径"""
        if self.start_position is None or self.target_point is None:
            return
        
        # 获取起始位置坐标
        p1_x = self.start_position['x']
        p1_y = self.start_position['y']
        
        # 获取目标点坐标
        p2_x = self.target_point.x
        p2_y = self.target_point.y
        
        # 计算两点之间的距离（直径）
        distance = math.sqrt((p2_x - p1_x)**2 + (p2_y - p1_y)**2)
        
        # 半径 = 直径 / 2
        self.circle_radius = distance / 2.0
        
        # 圆心 = 两点中点
        self.circle_center = {
            'x': (p1_x + p2_x) / 2.0,
            'y': (p1_y + p2_y) / 2.0
        }
        
        self.get_logger().info(f'起始位置: ({p1_x:.3f}, {p1_y:.3f})')
        self.get_logger().info(f'目标点: ({p2_x:.3f}, {p2_y:.3f})')
        self.get_logger().info(f'圆心: ({self.circle_center["x"]:.3f}, {self.circle_center["y"]:.3f})')
        self.get_logger().info(f'直径: {distance:.3f}, 半径: {self.circle_radius:.3f}')
        
        # 生成圆形路径点
        self.generate_circle_path()
        
        self.path_ready = True
        self.get_logger().info(f'圆形路径已生成，共 {len(self.circle_path)} 个点')

    def generate_circle_path(self, num_points=200):
        """生成圆形路径点"""
        self.circle_path = []
        
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = self.circle_center['x'] + self.circle_radius * math.cos(angle)
            y = self.circle_center['y'] + self.circle_radius * math.sin(angle)
            self.circle_path.append({'x': x, 'y': y, 'angle': angle})
        
        self.get_logger().info('圆形路径点已生成')

    def publish_path_and_viz(self):
        """发布路径和可视化"""
        if not self.path_ready:
            return
        
        # 发布Path消息
        path_msg = Path()
        path_msg.header.frame_id = "world"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for point in self.circle_path:
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = point['x']
            pose.pose.position.y = point['y']
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
        
        # 发布半径
        radius_msg = Float64()
        radius_msg.data = self.circle_radius
        self.radius_pub.publish(radius_msg)
        
        # 发布圆心
        center_msg = Point()
        center_msg.x = self.circle_center['x']
        center_msg.y = self.circle_center['y']
        center_msg.z = 0.0
        self.center_pub.publish(center_msg)
        
        # 发布可视化
        self.publish_visualization()

    def publish_visualization(self):
        """发布可视化Marker到rviz2"""
        marker_array = MarkerArray()
        
        # 1. 黑色圆形路径
        path_marker = Marker()
        path_marker.header.frame_id = "world"
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = "circle_path"
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.02
        path_marker.color.r = 0.0
        path_marker.color.g = 0.0
        path_marker.color.b = 0.0
        path_marker.color.a = 1.0
        path_marker.pose.orientation.w = 1.0
        
        for point in self.circle_path:
            p = Point()
            p.x = point['x']
            p.y = point['y']
            p.z = 0.0
            path_marker.points.append(p)
        # 闭合圆
        p = Point()
        p.x = self.circle_path[0]['x']
        p.y = self.circle_path[0]['y']
        p.z = 0.0
        path_marker.points.append(p)
        
        marker_array.markers.append(path_marker)
        
        # 2. 红色圆心标记
        center_marker = Marker()
        center_marker.header.frame_id = "world"
        center_marker.header.stamp = self.get_clock().now().to_msg()
        center_marker.ns = "center"
        center_marker.id = 1
        center_marker.type = Marker.SPHERE
        center_marker.action = Marker.ADD
        center_marker.pose.position.x = self.circle_center['x']
        center_marker.pose.position.y = self.circle_center['y']
        center_marker.pose.position.z = 0.0
        center_marker.pose.orientation.w = 1.0
        center_marker.scale.x = 0.08
        center_marker.scale.y = 0.08
        center_marker.scale.z = 0.08
        center_marker.color.r = 1.0
        center_marker.color.g = 0.0
        center_marker.color.b = 0.0
        center_marker.color.a = 1.0
        
        marker_array.markers.append(center_marker)
        
        # 3. 绿色起始点标记
        start_marker = Marker()
        start_marker.header.frame_id = "world"
        start_marker.header.stamp = self.get_clock().now().to_msg()
        start_marker.ns = "start"
        start_marker.id = 2
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.position.x = self.start_position['x']
        start_marker.pose.position.y = self.start_position['y']
        start_marker.pose.position.z = 0.0
        start_marker.pose.orientation.w = 1.0
        start_marker.scale.x = 0.06
        start_marker.scale.y = 0.06
        start_marker.scale.z = 0.06
        start_marker.color.r = 0.0
        start_marker.color.g = 1.0
        start_marker.color.b = 0.0
        start_marker.color.a = 1.0
        
        marker_array.markers.append(start_marker)
        
        # 4. 蓝色目标点标记
        target_marker = Marker()
        target_marker.header.frame_id = "world"
        target_marker.header.stamp = self.get_clock().now().to_msg()
        target_marker.ns = "target"
        target_marker.id = 3
        target_marker.type = Marker.SPHERE
        target_marker.action = Marker.ADD
        target_marker.pose.position.x = self.target_point.x
        target_marker.pose.position.y = self.target_point.y
        target_marker.pose.position.z = 0.0
        target_marker.pose.orientation.w = 1.0
        target_marker.scale.x = 0.06
        target_marker.scale.y = 0.06
        target_marker.scale.z = 0.06
        target_marker.color.r = 0.0
        target_marker.color.g = 0.0
        target_marker.color.b = 1.0
        target_marker.color.a = 1.0
        
        marker_array.markers.append(target_marker)
        
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    
    node = PathGeneration()
    
    # 创建一个线程来处理用户输入
    import threading
    
    def input_thread():
        print('\n' + '='*50)
        print('路径生成节点 - 输入目标点')
        print('='*50)
        print('输入格式: x y (用空格分隔)')
        print('输入 "q" 退出')
        print('='*50 + '\n')
        
        while rclpy.ok():
            try:
                user_input = input('> ').strip()
                if user_input.lower() == 'q':
                    rclpy.shutdown()
                    break
                try:
                    x, y = map(float, user_input.split())
                    if node.set_target_point(x, y):
                        print('路径已生成！节点将持续发布路径。')
                        print('可以启动 pid_track 节点来跟踪路径。')
                except ValueError:
                    print('无效输入。请输入 "x y" 格式 (例如: 1.0 2.5)')
            except EOFError:
                break
    
    input_thread_handle = threading.Thread(target=input_thread, daemon=True)
    input_thread_handle.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到键盘中断，正在停止...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
