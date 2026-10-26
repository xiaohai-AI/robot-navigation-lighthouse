#!/usr/bin/env python3
"""
tour_node.py —— 自动巡游建图（配合 slam_toolbox）

干什么
------
按一条固定路线把整栋房子（走廊 + 7 个房间 + 厨房↔书房内门回环）走一遍，
边走边让 slam_toolbox 建图，跑完你去存图。

为什么这么设计
--------------
1) 用 /odom 闭环，不是开环定时 —— 定时走会因打滑/加减速误差累积而走偏。
   每个航点算「距离误差 + 朝向误差」：偏得多就先原地转正，再直着开过去。
2) 房间靠「进门后原地转 360°」扫全 —— 雷达在 0.132 m 高、量程 12 m，
   门内 1.8 m 处转一圈就能把 4.7 m 深的房间扫干净；比在满是家具的房间里
   绕圈安全得多。
3) 路线里特意穿一次「厨房 <-> 书房」内门，在地图上形成真实回环，
   顺便验证 slam_toolbox 的回环闭合。
4) 前向急停保护：航点都按家具位置留了 >= 0.35 m 余量，
   所以只要前方 0.22 m 内出现东西就是异常 —— 立刻停车报错，不盲冲。

用法
----
    # 最省事：和 headless 仿真 + SLAM 一起起（无 3D 窗口，只看 RViz）
    ros2 launch ./dan_house.launch.py headless:=true slam:=true tour:=true rviz:=true

    # 或者单独跑（仿真 + slam 已经在跑的情况下）
    /usr/bin/python3 tour_node.py --ros-args -p use_sim_time:=true
    /usr/bin/python3 tour_node.py --speed 0.15     # 更慢，建图更稳
    /usr/bin/python3 tour_node.py --dry-run        # 只打印路线，不发指令

    # 跑完存图
    ros2 run nav2_map_server map_saver_cli -f config/maps/dan_house
"""
import argparse
import json
import math
import os
import subprocess
import sys
import threading

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

# ============================================================================
# 路线表：(x, y, 动作, 说明)
#   x, y —— odom 坐标系（机器人出生在 (0,0) 朝 +x，所以 odom ≈ 世界坐标）
#   动作 —— 'drive' 开过去即可；'spin' 开过去之后原地转 360°
# 坐标都按家具实际位置留了 >= 0.35 m 余量（车半径 0.21 m）
# ============================================================================
ROUTE = [
    # ---- 走廊东段 ----
    (3.00,  0.00, 'drive', '走廊东行'),
    (6.40,  0.00, 'spin',  '走廊东端：扫东墙 + 南北两侧门'),

    # ---- 储藏间（南侧门 x=5.6）----
    (5.60,  0.20, 'drive', '储藏间门口'),
    (5.60, -1.80, 'spin',  '进储藏间：扫货架和箱子'),
    (5.60,  0.20, 'drive', '退回走廊'),

    # ---- 书房（北侧门 x=4.6）----
    (4.60,  0.20, 'drive', '书房门口'),
    (4.60,  1.90, 'spin',  '进书房（南半边）'),
    (3.40,  3.70, 'drive', '书房北侧'),
    (1.90,  3.50, 'spin',  '书房西侧，准备穿内门'),

    # ---- 穿「厨房<->书房」内门（x=1.4，洞口 y 3.0~4.0），形成回环 ----
    (0.55,  3.05, 'drive', '穿过内门进入厨房'),
    (-0.50, 2.20, 'drive', '厨房中部（绕过餐桌）'),
    (-0.70, 0.20, 'drive', '从厨房门退回走廊'),

    # ---- 卫生间（南侧门 x=1.9）----
    (1.90,  0.20, 'drive', '卫生间门口'),
    (1.90, -1.90, 'spin',  '进卫生间'),
    (1.90,  0.20, 'drive', '退回走廊'),

    # ---- 卧室 2（南侧门 x=-1.6）----
    (-1.60,  0.20, 'drive', '卧室2 门口'),
    (-1.60, -2.00, 'spin',  '进卧室2'),
    (-1.60,  0.20, 'drive', '退回走廊'),

    # ---- 客厅（北侧门 x=-5.4）----
    (-5.40, 0.20, 'drive', '客厅门口'),
    (-5.40, 2.20, 'spin',  '进客厅：扫沙发/茶几/电视'),
    (-5.40, 0.20, 'drive', '退回走廊'),

    # ---- 卧室 1（南侧门 x=-5.6）----
    (-5.60, -0.20, 'drive', '卧室1 门口'),
    (-5.60, -2.00, 'spin',  '进卧室1'),
    (-5.60,  0.20, 'drive', '退回走廊'),

    # ---- 收尾 ----
    (-7.20, 0.00, 'spin',  '走廊西端：扫西墙，补齐走廊'),
    (0.00,  0.00, 'spin',  '回到起点，最后扫一圈'),
]

# ---- 路线以 build_world.py 生成的 config/route.json 为准 ----
# （上面这份内置路线只是兜底：改了房子形状后，route.json 会自动跟着变，
#   不用再手动同步两个文件）
def _load_route(default):
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config', 'route.json')
    try:
        with open(path) as f:
            wps = json.load(f)['waypoints']
        if wps:
            return [tuple(w) for w in wps]
    except Exception as e:
        print(f'[tour] 读取 {path} 失败（{e}），改用内置路线')
    return default


ROUTE = _load_route(ROUTE)

ARRIVE_TOL = 0.10      # 到位容差 m
HEAD_TOL = 0.05        # 朝向容差 rad（≈3°；原来 0.12/7° 太大，穿 2 m 门洞会偏 24 cm 刮门框）
FRONT_HALF = 15.0      # 前向保护扇区半角（度）
STALL_TIME = 1.5       # 命令前进但 odom 不动超过这么久 -> 判定卡住
STALL_SPEED = 0.04     # 低于这个速度算"没动" m/s
MAP_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config', 'maps')
CHECKPOINT_EVERY = 8   # 每完成 8 个航点在后台存一张地图快照（防止跑完前挂掉白跑）


def save_map(path_base, timeout=40):
    """调 nav2_map_server 存图。返回 (成功?, 输出)。绝不抛异常、绝不阻塞控制循环。"""
    cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', path_base,
           '--ros-args', '-p', 'use_sim_time:=true', '-p', 'save_map_timeout:=15.0']
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        ok = os.path.exists(path_base + '.pgm') and os.path.exists(path_base + '.yaml')
        return ok, ((r.stdout or '') + (r.stderr or '')).strip()
    except Exception as e:
        return False, str(e)


def truth_pose():
    """查 gz 里的真实位姿（只在调试时用；独立线程里跑，绝不阻塞控制循环）"""
    try:
        out = subprocess.run(['ign', 'model', '-m', 'tb3', '-p'],
                             capture_output=True, text=True, timeout=5).stdout
        lines = [l for l in out.splitlines() if l.strip().startswith('[')]
        if len(lines) >= 2:
            xyz = [float(v) for v in lines[0].replace('[', ' ').replace(']', ' ').split()[:3]]
            rpy = [float(v) for v in lines[1].replace('[', ' ').replace(']', ' ').split()[:3]]
            return xyz[0], xyz[1], rpy[2]
    except Exception:
        pass
    return None


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TourNode(Node):
    def __init__(self, v_max, spin_rate, front_guard, debug=False):
        super().__init__('tour_driver')
        self.v_max = v_max
        self.debug = debug
        self.truth = None          # (x, y, yaw) 真实位姿，调试用
        self.t_debug = 0.0
        self.w_max = 1.0
        self.spin_rate = spin_rate
        self.front_guard = front_guard
        self.dt = 0.05

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

        # 位姿优先取 map 坐标系（slam_toolbox 扫描匹配纠正过），拿不到才退回 /odom。
        # 原因：odom 是纯轮式推算，轮子一打滑（刮门框、卡住）位置和朝向同时被污染，
        #       实测能偏到 83°；而 map 帧的位姿会被扫描匹配拉回正确位置。
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._tf_warned = False
        self._map_ok = 0

        self.pose = None            # (x, y, yaw)，优先来自 map 坐标系
        self.odom_pose = None       # /odom 兜底
        self.front_min = float('inf')
        self.side_min = float('inf')
        self.side_hit_since = None
        self.i = 0                  # 当前航点序号
        self.phase = 'rotate'       # rotate | drive | spin
        self.spin_acc = 0.0         # 已转过的角度（用 odom 实测累加）
        self.spin_last_yaw = None
        self.t_waypoint = None
        self.t_drive_start = None
        self.t_settle = 0.0
        self.prev_pos = None
        self.stall_since = None
        self.scan_sig = None        # 雷达画面快照（用于判断"车到底动没动"）
        self.spin_sig_ref = None
        self.t_spin_check = 0.0
        self.spin_stuck = 0.0
        self.t_start = self.now()
        self.done = False

        self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            f'巡游开始：{len(ROUTE)} 个航点 | 速度 {v_max} m/s | '
            f'旋转 {spin_rate} rad/s | 前向保护 {front_guard} m')
        if self.debug:
            threading.Thread(target=self._truth_loop, daemon=True).start()
            self.get_logger().info('调试模式：每 4 s 打印一次 odom 与真实位姿的偏差')

    # ---------------- 调试：后台查真实位姿 ----------------
    def _truth_loop(self):
        while not self.done:
            p = truth_pose()
            if p is not None:
                self.truth = p
            threading.Event().wait(4.0)

    # ---------------- 回调 ----------------
    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom_pose = (p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))

    def on_scan(self, msg):
        """
        取正前方 ±FRONT_HALF° 的最小距离。

        注意雷达索引：本车雷达 min_angle=-π、max_angle=+π、samples=360，
        所以「0 弧度（车头方向）」在数组**中间**（约 index 180），
        不在数组两端（两端都是车尾方向，±π）——这是很容易搞反的地方。
        """
        n = len(msg.ranges)
        if n == 0:
            return
        a0, a1 = msg.angle_min, msg.angle_max
        # 0 弧度对应的浮点索引
        mid = (0.0 - a0) / (a1 - a0) * (n - 1)
        half = (FRONT_HALF * math.pi / 180.0) / (a1 - a0) * (n - 1)
        idx = {int(round(mid + k)) % n
               for k in range(-int(half), int(half) + 1)}
        vals = [msg.ranges[j] for j in idx
                if math.isfinite(msg.ranges[j]) and 0.0 < msg.ranges[j] < msg.range_max]
        self.front_min = min(vals) if vals else float('inf')

        # 左右两侧最近的墙/门框距离（±25°，取左右较小值）—— 用来提前发现"在刮门框"
        def _sector_min(center_deg, half_deg):
            c = (math.radians(center_deg) - a0) / (a1 - a0) * (n - 1)
            hw = math.radians(half_deg) / (a1 - a0) * (n - 1)
            ids = {int(round(c + k)) % n for k in range(-int(hw), int(hw) + 1)}
            vs = [msg.ranges[j] for j in ids
                  if math.isfinite(msg.ranges[j]) and 0.0 < msg.ranges[j] < msg.range_max]
            return min(vs) if vs else float('inf')
        self.side_min = min(_sector_min(90.0, 25.0), _sector_min(-90.0, 25.0))

        # 存一份降采样的"雷达画面快照"，用来判断车体是否真的在动
        self.scan_sig = [min(r, 50.0) if math.isfinite(r) else 50.0
                         for r in msg.ranges[::9]]

    # ---------------- 主循环 ----------------
    def refresh_pose(self):
        """优先用 map 坐标系的位姿（被 slam_toolbox 扫描匹配纠正过），
        拿不到再退回 /odom（纯轮式推算，打滑后会错）。"""
        try:
            tr = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
            t, q = tr.transform.translation, tr.transform.rotation
            self.pose = (t.x, t.y, yaw_from_quat(q))
            self._map_ok += 1
            return
        except Exception:
            if not self._tf_warned and self._map_ok == 0:
                self._tf_warned = True
                self.get_logger().warn('map 坐标系尚不可用，先用 /odom（SLAM 起来后会自动切换）')
        # 退回 odom
        if self.odom_pose is not None:
            self.pose = self.odom_pose

    def tick(self):
        self.refresh_pose()
        if self.done or self.pose is None:
            return
        if self.i >= len(ROUTE):
            self.finish('全部航点完成')
            return

        tx, ty, kind, label = ROUTE[self.i]
        x, y, yaw = self.pose
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        prev = self.prev_pos            # 上一拍的位姿（卡住检测用）
        self.prev_pos = (x, y)

        if self.t_waypoint is None:
            self.t_waypoint = self.now()
        if self.now() - self.t_waypoint > 120.0:
            self.abort(f'航点 {self.i + 1} 超时（{label}）')
            return

        # 调试：每 2 s 打一次 odom vs 真实 的偏差
        if self.debug and self.truth is not None and self.now() - self.t_debug > 2.0:
            self.t_debug = self.now()
            err = math.hypot(self.truth[0] - x, self.truth[1] - y)
            self.get_logger().info(
                f'[dbg] 航点{self.i + 1}/{len(ROUTE)} {self.phase:6s} '
                f'odom=({x:+.2f},{y:+.2f}) 真实=({self.truth[0]:+.2f},{self.truth[1]:+.2f}) '
                f'偏差={err * 100:.0f}cm')

        if self.phase == 'rotate':
            err = 0.0 if dist < ARRIVE_TOL else wrap(math.atan2(dy, dx) - yaw)
            if abs(err) < HEAD_TOL:
                self.phase = 'drive'
                self.t_drive_start = self.now()
                self.front_min = float('inf')   # 丢掉转向前留下的陈旧雷达读数
                self.stall_since = None
            else:
                self.send(0.0, self.clamp(1.8 * err, self.w_max))
                return

        if self.phase == 'drive':
            if dist < ARRIVE_TOL:
                self.arrive(label, kind)
                return
            err = wrap(math.atan2(dy, dx) - yaw)
            if abs(err) > 0.7:
                self.phase = 'rotate'
                self.send(0.0, self.clamp(1.8 * err, self.w_max))
                return
            # 前向急停保护：起步 0.4 s 内不判（雷达可能还是转向时的旧值），
            # 并且只在朝向已经正对目标时才生效（斜着过门洞时门框会擦边，属正常）
            armed = (self.t_drive_start is not None
                     and self.now() - self.t_drive_start > 0.4
                     and abs(err) < 0.20)
            if armed and self.front_min < self.front_guard:
                self.abort(f'前方仅 {self.front_min:.2f} m 有障碍（{label}）')
                return
            # 侧面刮擦检测：一侧紧贴门框/墙时继续开，会让单侧轮子打滑，
            # odom 的位置和朝向会被污染（实测能偏 83°），必须在发生前停。
            if self.side_min < 0.20 and self.t_drive_start is not None \
                    and self.now() - self.t_drive_start > 0.4:
                if self.side_hit_since is None:
                    self.side_hit_since = self.now()
                elif self.now() - self.side_hit_since > 1.0:
                    self.abort(f'侧面仅 {self.side_min:.2f} m，在刮门框/墙（{label}）')
                    return
            else:
                self.side_hit_since = None
            v = min(self.v_max, 1.2 * dist)
            # 转弯时降速，建图更稳
            v *= max(0.35, 1.0 - abs(err) / 0.7 * 0.65)
            # 卡住检测：命令在前进但 odom 位置几乎不动 -> 被什么挡住了，
            # 此时若继续发速度命令，odom 会疯狂累加（轮子空转），越漂越远。
            if prev is not None:
                moved = math.hypot(x - prev[0], y - prev[1])
                if moved < STALL_SPEED * self.dt:
                    if self.stall_since is None:
                        self.stall_since = self.now()
                    elif self.now() - self.stall_since > STALL_TIME:
                        self.abort(f'卡住不动：命令 {v:.2f} m/s（{label}）')
                        return
                else:
                    self.stall_since = None
            self.send(v, self.clamp(1.8 * err, self.w_max))
            return

        if self.phase == 'settle':
            self.send(0.0, 0.0)
            if self.now() - self.t_settle >= 0.6:
                self.phase = 'spin'
                self.spin_acc = 0.0
                self.spin_last_yaw = None
                self.spin_sig_ref = None
                self.t_spin_check = self.now()
                self.spin_stuck = 0.0
            return

        if self.phase == 'spin':
            # 闭环旋转：用 odom 实测偏航增量累加，不用命令值积分。
            # 开环积分会因为实际转速略低于命令值而少转，偏几度累积下来就会在门洞里撞框。
            yaw_now = self.pose[2]
            if self.spin_last_yaw is None:
                self.spin_last_yaw = yaw_now
            d = abs(wrap(yaw_now - self.spin_last_yaw))
            self.spin_last_yaw = yaw_now
            self.spin_acc += d

            # 旋转卡死检测：转的时候雷达画面必须一直在变。
            # odom 是从轮子推的，车被墙卡住时轮子照样转、odom 照样涨，只有雷达能看出"没在转"。
            tn = self.now()
            if self.scan_sig is not None and tn - self.t_spin_check > 0.6:
                if self.spin_sig_ref is not None:
                    diff = sum(abs(a - b)
                               for a, b in zip(self.scan_sig, self.spin_sig_ref)) / len(self.scan_sig)
                    if diff < 0.02:
                        self.spin_stuck += tn - self.t_spin_check
                        if self.spin_stuck > 1.5:
                            self.abort('原地旋转被卡住：雷达画面 1.5 s 没变化')
                            return
                    else:
                        self.spin_stuck = 0.0
                self.spin_sig_ref = self.scan_sig
                self.t_spin_check = tn

            if self.spin_acc >= 2.0 * math.pi - 0.05:
                self.arrive(label, kind)
            else:
                self.send(0.0, self.spin_rate)

    # ---------------- 状态切换 ----------------
    def arrive(self, label, kind):
        if kind == 'spin' and self.phase not in ('spin', 'settle'):
            # 先停稳 0.6 s 再转：行驶中车体有微小俯仰/振动，边开边转会让点云拖影
            self.phase = 'settle'
            self.t_settle = self.now()
            self.send(0.0, 0.0)
            self.get_logger().info(f'✓ 到达「{label}」，停稳后原地转 360°')
            return
        self.send(0.0, 0.0)
        self.get_logger().info(f'✓ 航点 {self.i + 1}/{len(ROUTE)}：{label}')
        self.i += 1
        self.phase = 'rotate'
        self.t_waypoint = None
        self.t_drive_start = None
        # 每 8 个航点在后台存一张快照：万一跑完前 slam 卡死/机器过载，也不至于白跑
        if self.i % CHECKPOINT_EVERY == 0:
            threading.Thread(target=self._checkpoint, args=(self.i,), daemon=True).start()

    def _checkpoint(self, n):
        path = os.path.join(MAP_DIR, f'checkpoint_{n:02d}')
        ok, out = save_map(path)
        if ok:
            self.get_logger().info(f'💾 检查点地图已存：config/maps/checkpoint_{n:02d}.pgm（航点 {n}）')
        else:
            self.get_logger().warn(f'检查点存图失败（不影响巡游）：{out[-160:]}')

    def finish(self, why):
        self.done = True
        for _ in range(20):
            self.send(0.0, 0.0)
        el = self.now() - self.t_start
        self.get_logger().info('=' * 62)
        self.get_logger().info(f'🏁 巡游结束：{why}')
        self.get_logger().info(f'   耗时 {el:.0f} s（仿真时间）')
        # 结束时立刻自动存图（不等你手动敲命令）
        self.get_logger().info('   💾 正在自动保存地图 ...')
        ok, out = save_map(os.path.join(MAP_DIR, 'dan_house'))
        if ok:
            self.get_logger().info('   ✅ 地图已保存：config/maps/dan_house.pgm + .yaml')
        else:
            self.get_logger().error(f'   ❌ 自动存图失败：{out[-200:]}')
            self.get_logger().error('   手动重试：ros2 run nav2_map_server map_saver_cli -f config/maps/dan_house')
        self.get_logger().info('=' * 62)

    def abort(self, why):
        self.done = True
        for _ in range(20):
            self.send(0.0, 0.0)
        self.get_logger().error(f'❌ 巡游中止：{why}')
        self.get_logger().error('   已停车。看一眼 RViz/雷达，或手动开出来再重跑。')

    # ---------------- 工具 ----------------
    def send(self, v, w):
        m = Twist()
        m.linear.x = float(v)
        m.angular.z = float(w)
        self.pub.publish(m)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def clamp(v, lim):
        return max(-lim, min(lim, v))


def main():
    ap = argparse.ArgumentParser(description='自动巡游建图')
    ap.add_argument('--speed', type=float, default=0.20, help='最大线速度 m/s（默认 0.20）')
    ap.add_argument('--spin-rate', type=float, default=0.5, help='原地旋转 rad/s（默认 0.5）')
    ap.add_argument('--front-guard', type=float, default=0.16, help='前方急停距离 m（默认 0.16）')
    ap.add_argument('--debug', action='store_true',
                    help='每 2 s 打印 odom 与 gz 真实位姿的偏差（定位漂移用）')
    ap.add_argument('--dry-run', action='store_true', help='只打印路线，不发指令')
    args, _ = ap.parse_known_args()

    if args.dry_run:
        total = sum(math.hypot(ROUTE[i + 1][0] - ROUTE[i][0],
                               ROUTE[i + 1][1] - ROUTE[i][1])
                    for i in range(len(ROUTE) - 1))
        spins = sum(1 for r in ROUTE if r[2] == 'spin')
        walk = total / args.speed
        rot = spins * 2 * math.pi / args.spin_rate
        print(f'路线 {len(ROUTE)} 个航点，直线总长 {total:.1f} m，{spins} 处 360° 扫描')
        print(f'预计 ≈ {walk:.0f} s 行走 + {rot:.0f} s 旋转 = '
              f'{(walk + rot) / 60:.1f} 分钟（仿真时间）')
        for i, (x, y, kind, label) in enumerate(ROUTE, 1):
            print(f'  {i:2d}. ({x:+.2f}, {y:+.2f}) {kind:5s} {label}')
        return 0

    rclpy.init(args=None)
    node = TourNode(args.speed, args.spin_rate, args.front_guard, debug=args.debug)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        for _ in range(20):
            node.send(0.0, 0.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
