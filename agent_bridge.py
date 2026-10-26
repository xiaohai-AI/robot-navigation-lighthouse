#!/usr/bin/python3
"""ROS 侧的 agent 桥接节点（用系统 python，不是 venv）。

为什么需要它：
    Nav2 的 NavigateToPose 是 action，rosbridge 的 ROS2 action 协议不好用。
    这里用标准 rclpy ActionClient 把 action 包装成两个普通话题：

        /agent/nav_goal    geometry_msgs/PoseStamped   ← agent 发目标（覆盖旧目标）
        /agent/nav_status  std_msgs/String (JSON)      → 状态/进度/剩余距离/恢复次数
        /agent/pose        geometry_msgs/PoseStamped   → 当前位姿 5 Hz
        /agent/cancel      std_msgs/Empty              ← 取消当前目标

状态机（status.state）：
    idle / sending / active / succeeded / failed / canceled / stalled / rejected

看门狗：目标处于 active 时，若 12 s 内 distance_remaining 没有下降 0.15 m，
        判定 stalled → 取消目标（这就是探索阶段踩过的"目标在飞但车不动"）。

跑法（必须在已 source 过 ROS 的终端里）：
    cd ~/Project/hermes_proj/robot-navigation-lighthouse
    source /opt/ros/humble/setup.bash
    /usr/bin/python3 agent_bridge.py
"""
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Empty, String

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:  # pragma: no cover
    raise SystemExit('缺 nav2_msgs，先 source /opt/ros/humble/setup.bash')

import tf2_ros

STALL_SECS = 12.0
STALL_DROP_M = 0.15


class AgentBridge(Node):
    def __init__(self):
        super().__init__('agent_bridge')

        # 状态用 TRANSIENT_LOCAL：agent 晚连上来也能立刻拿到当前状态
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.pub_status = self.create_publisher(String, '/agent/nav_status', latched)
        self.pub_pose = self.create_publisher(PoseStamped, '/agent/pose', 10)

        self.create_subscription(PoseStamped, '/agent/nav_goal', self.on_goal, 10)
        self.create_subscription(Empty, '/agent/cancel', self.on_cancel, 10)

        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.state = 'idle'
        self.goal = None
        self.goal_seq = 0          # 目标序号：新目标 +1，异步回调带自家序号回来，旧的回调直接丢弃
        self.goal_xy = None
        self.goal_handle = None
        self.busy = False
        self.distance = None
        self.recoveries = 0
        self.nav_time = 0.0
        self.detail = ''
        self.started_at = None
        self.best_distance = None
        self.best_at = None
        self.pose = None

        self.create_timer(0.2, self.publish_pose)
        self.create_timer(0.5, self.publish_status)
        self.create_timer(1.0, self.watchdog)
        self.get_logger().info(
            f'agent_bridge 就绪：/agent/nav_goal 发目标，/agent/nav_status 看状态，'
            f'看门狗 {STALL_SECS:.0f}s/{STALL_DROP_M:.2f}m')

    # ---------------- 位姿 ----------------
    def publish_pose(self):
        try:
            tr = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return
        t = tr.transform.translation
        q = tr.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (t.x, t.y, yaw)
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = t.x
        msg.pose.position.y = t.y
        msg.pose.orientation = q
        self.pub_pose.publish(msg)

    # ---------------- 状态 ----------------
    def publish_status(self, extra=None):
        d = {
            'state': self.state,
            'goal': self.goal_xy,
            'distance_remaining': None if self.distance is None else round(self.distance, 3),
            'recoveries': self.recoveries,
            'nav_time': round(self.nav_time, 1),
            'elapsed': None if self.started_at is None else round(time.time() - self.started_at, 1),
            'pose': None if self.pose is None else [round(v, 3) for v in self.pose],
            'detail': self.detail,
        }
        if extra:
            d.update(extra)
        msg = String()
        msg.data = json.dumps(d, ensure_ascii=False)
        self.pub_status.publish(msg)

    # ---------------- 目标 ----------------
    def on_goal(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.get_logger().info(f'收到目标 ({x:+.2f}, {y:+.2f}) yaw={yaw:+.2f}')

        # 目标序号：每个新目标 +1，所有异步回调都带着自己的序号回来。
        # 没有这个序号，旧目标被取消后它的 result 回调会把「失败」写进新目标的状态：
        # 实测抢占时新目标 0.1 s 就报 failed（Nav2 结束码 5），同时 goal_handle 被清成
        # None —— 之后 /agent/cancel 和看门狗都够不着真正在跑的那个目标。
        # 必须在取消旧 handle 之前加，否则旧回调可能先用旧序号返回。
        self.goal_seq += 1
        seq = self.goal_seq

        if self.busy and self.goal_handle is not None:      # 新目标覆盖旧目标
            self.get_logger().warn(f'已有目标在飞，先取消旧的（旧 #{self.goal_seq - 1} → 新 #{seq}）')
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass

        if not self.nav.wait_for_server(timeout_sec=5.0):
            self.state = 'rejected'
            self.detail = 'navigate_to_pose action server 不在（Nav2 没起来？）'
            self.get_logger().error(self.detail)
            self.publish_status()
            return

        goal = NavigateToPose.Goal()
        goal.pose = msg
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        self.state = 'sending'
        self.detail = ''
        self.busy = True
        self.goal_xy = [round(x, 3), round(y, 3)]
        self.distance = None
        self.recoveries = 0
        self.nav_time = 0.0
        self.started_at = time.time()
        self.best_distance = None
        self.best_at = time.time()
        self.publish_status()

        fut = self.nav.send_goal_async(
            goal, feedback_callback=lambda fb, s=seq: self.on_feedback(fb, s))
        fut.add_done_callback(lambda f, s=seq: self.on_goal_response(f, s))

    def on_goal_response(self, fut, seq: int):
        if seq != self.goal_seq:                 # 已被更新的目标取代，它的结局与当前目标无关
            return
        try:
            handle = fut.result()
        except Exception as e:
            self.state, self.detail, self.busy = 'failed', f'发送目标异常: {e}', False
            self.publish_status()
            return
        if not handle.accepted:
            self.state, self.detail, self.busy = 'rejected', 'Nav2 拒绝了目标', False
            self.publish_status()
            return
        self.goal_handle = handle
        self.state = 'active'
        # 基线必须在这里（目标被接受的那一刻）重置，而不是在 on_goal：
        # 实测「先去厨房，然后到沙发旁边」第二个目标把第一个目标的残留距离 0.22 m
        # 当成了基线 → 真实距离 11.66 m 永远比不过它 → 12 s 后误判 stalled。
        self.best_distance = None
        self.best_at = time.time()
        self.publish_status()
        handle.get_result_async().add_done_callback(lambda f, s=seq: self.on_result(f, s))

    def on_feedback(self, fb, seq: int):
        # 只接受"当前目标已被接受"之后的 feedback：旧目标的 feedback 可能在新目标
        # 注册之后才到（NavigateToPose 的 feedback 没有目标 id，无法从内容区分）。
        if seq != self.goal_seq or self.state != 'active':
            return
        f = fb.feedback
        self.distance = float(f.distance_remaining)
        self.recoveries = int(f.number_of_recoveries)
        self.nav_time = float(f.navigation_time.sec) + float(f.navigation_time.nanosec) * 1e-9

    def on_result(self, fut, seq: int):
        if seq != self.goal_seq:
            # 被取代的目标：它的 succeeded/failed/canceled 都不属于当前目标。
            # 这一行就是"抢占后新目标 0.1 s 报 failed"的修复点。
            self.get_logger().info(f'忽略已被取代的目标 #{seq} 的结果（当前 #{self.goal_seq}）')
            return
        try:
            res = fut.result()
            status = res.status
        except Exception as e:
            self.state, self.detail = 'failed', f'结果回调异常: {e}'
            self.busy = False
            self.publish_status()
            return
        self.busy = False
        self.goal_handle = None
        if self.state == 'stalled':
            self.publish_status()
            return
        if status == 4:                  # SUCCEEDED
            self.state, self.detail = 'succeeded', '到达目标'
        elif status == 5:                # CANCELED：到这里说明取消的是当前目标本身
            self.state, self.detail = 'canceled', '已取消'
        else:
            self.state, self.detail = 'failed', f'Nav2 结束码 {status}'
        self.get_logger().info(f'目标结束：#{seq} {self.state} {self.detail}')
        self.publish_status()

    def on_cancel(self, _msg: Empty):
        if self.goal_handle is not None:
            self.state, self.detail = 'canceled', 'agent 主动取消'
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.publish_status()

    # ---------------- 看门狗 ----------------
    def watchdog(self):
        if self.state != 'active' or self.distance is None:
            return
        now = time.time()
        best = self.best_distance
        # 自愈：真实目标不会"离自己更远了"。距离突然比基线大 1 m 以上，
        # 说明基线被残留采样污染了 → 以当前采样重新起算，而不是死等 12 s 误杀。
        if best is not None and self.distance > best + 1.0:
            self.get_logger().warn(
                f'看门狗基线自愈：{best:.2f} → {self.distance:.2f} m（疑似残留采样）')
            best = None
        if best is None or self.distance < best - STALL_DROP_M:
            self.best_distance = self.distance
            self.best_at = now
            return
        if now - self.best_at > STALL_SECS:
            self.state = 'stalled'
            self.detail = (f'{STALL_SECS:.0f}s 内剩余距离未下降 {STALL_DROP_M:.2f} m'
                           f'（当前 {self.distance:.2f} m）→ 取消')
            self.get_logger().warn(self.detail)
            try:
                if self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.publish_status()


def main():
    rclpy.init()
    node = AgentBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
