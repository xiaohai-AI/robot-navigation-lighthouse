#!/usr/bin/env python3
"""
odom_tf_node.py —— 把 gz 的 /odom 转成 TF: odom -> base_footprint

为什么需要它：
  gz-sim 的 DiffDrive 插件能发布 /odom 话题，但它的 publish_odom_tf 走的是 gz 自己的
  TF 通道，桥接出来很别扭。所以我们在模型里把 publish_odom_tf 关掉，
  统一用这个节点把 /odom 转成 ROS 的 tf 广播，链路清晰、方便调试。

TF 链（全部到齐 slam_toolbox 才能工作）：
    map        -> odom            slam_toolbox 发布
    odom       -> base_footprint  本节点
    base_footprint -> base_link -> base_scan / imu_link / camera_*
                                  robot_state_publisher 发布（读官方 waffle URDF）

注意：不要在这里写 sys.path.insert 之类的手工路径 hack —— 那会把 rclpy 指向不匹配的
目录，直接报 ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'。
"""
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTf(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')
        # 注意：rclpy 会自动声明 use_sim_time，这里再 declare 会抛
        # ParameterAlreadyDeclaredException，所以不要动它。
        #
        # odom_topic 可切换，用来做 A/B 对比：
        #   /odom                 → 仿真真值里程计（A 阶段，gz OdometryPublisher）
        #   /odometry/filtered    → EKF 融合结果（C 阶段，robot_localization）
        self.odom_topic = self.declare_parameter(
            'odom_topic', '/odom').get_parameter_value().string_value
        self.br = TransformBroadcaster(self)
        self.last = None
        self.create_subscription(Odometry, self.odom_topic, self.cb, 10)
        # gz 的 /odom 偶有丢帧，用定时器按固定频率重发最后一次，TF 不会断流
        self.create_timer(1.0 / 30.0, self.tick)
        self.get_logger().info(
            f'odom -> base_footprint TF 广播已启动（输入话题：{self.odom_topic}）')

    def cb(self, msg: Odometry):
        self.last = msg

    def tick(self):
        if self.last is None:
            return
        msg = self.last
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        # base_footprint 语义上贴地，z 强制 0（gz 里模型原点有 1 mm 抬升）
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
