#!/usr/bin/env python3
"""
frontier_explore.py —— 前沿探索：放到地上，自己把"能到的地方"探一遍

与 tour_node.py 完全独立、互不影响：
    tour_node.py     固定路线巡游（路线写死，换房子要改路线表）
    frontier_explore 看实时地图自己决定去哪（换任何房子都能跑，直到没有未知边界）

算法（经典 frontier exploration）
--------------------------------
  1. 订阅 /map（slam_toolbox 在线建图，5 cm 栅格，TRANSIENT_LOCAL）
  2. frontier 格子 = 自由格，且 8 邻域里至少有一个未知格
  3. 8 连通聚类；丢掉太小的簇（噪声）、丢掉紧贴机器人脚下的簇
  4. 打分 score = 簇大小 / (距离 + 1.0)  → 偏好"又大又近"的边界
  5. 目标点取簇里离质心最近的可通行格，发 NavigateToPose（map 系）
  6. 成功/失败后回到 2；失败的目标进黑名单，永不重复
  7. 没有可用 frontier / 达到时间或目标数上限 -> 结束，自动存图
     （存到 dan_house_explore.*，绝不覆盖巡游的 dan_house.*）

用法
----
  # 推荐：一条命令起全套（slam + nav2 实况导航 + 探索）
  ros2 launch ./dan_house.launch.py explore:=true rviz:=true

  # 或者单独跑（仿真 + slam + nav2 实况已在跑的情况下）
  /usr/bin/python3 frontier_explore.py --ros-args -p use_sim_time:=true

  # 参数
  --max-goals 40       最多发多少个导航目标
  --max-minutes 30     最长探索时间（仿真时间）
  --min-cluster 12     一个 frontier 簇至少多少格才算有效
  --revisit-gap 0.8    已放弃目标的最小重试距离 m
"""
import argparse
import math
import os
import subprocess
import sys
import threading
import time
import traceback
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

HERE = os.path.dirname(os.path.abspath(__file__))
MAP_DIR = os.path.join(HERE, 'config', 'maps')


def save_map(path_base, timeout=60):
    """存图（后台/同步都行）。绝不抛异常。"""
    cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', path_base,
           '--ros-args', '-p', 'use_sim_time:=true', '-p', 'save_map_timeout:=20.0']
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        ok = os.path.exists(path_base + '.pgm') and os.path.exists(path_base + '.yaml')
        return ok, ((r.stdout or '') + (r.stderr or '')).strip()
    except Exception as e:
        return False, str(e)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _dilate_sep(a, r):
    """方形膨胀（半径 r 格），可分离实现：先沿列、再沿行。

    等价于 (2r+1)^2 的方形核，但只需 4r 次数组移位而不是 (2r+1)^2 次。
    用于"目标净空"判断：把占据格向外撑开 r 格，撑开后仍为空闲的地方
    才允许做导航目标（否则机器人站不下）。
    """
    out = a.copy()
    for s in range(1, r + 1):
        out[s:, :] |= a[:-s, :]
        out[:-s, :] |= a[s:, :]
    mid = out.copy()          # 必须复制！否则列方向会在"已膨胀"的结果上再膨胀一次
    for s in range(1, r + 1):
        out[:, s:] |= mid[:, :-s]
        out[:, :-s] |= mid[:, s:]
    return out


class FrontierExplore(Node):
    def __init__(self, max_goals, max_minutes, min_cluster, revisit_gap,
                 goal_clearance=0.40, stall_secs=12.0, stall_meters=0.25,
                 min_goal_dist=0.60, map_prefix='dan_house_explore', min_gain=40,
                 plan_check=True, plan_timeout=1.5):
        super().__init__('frontier_explore')
        self.max_goals = max_goals
        self.max_time = max_minutes * 60.0
        self.min_cluster = min_cluster
        self.revisit_gap = revisit_gap
        # 假边界判据：到达目标后延迟几秒看"已知格数"涨了多少，涨不到 min_gain 就拉黑。
        # 踩过的坑（2026-10-24）：blacklist 原来只在**失败**时追加，成功到达不加，
        # 于是两块"走到了但那里根本没有新未知区"的假边界被无限重选 ——
        # 实测一轮 80 个目标里 76 个是同一对坐标来回跳（(+6.78,-2.45) ×38、
        # (+5.13,-2.90) ×38），地图逐格没变，整轮 22 分钟零产出。
        self.min_gain = min_gain
        self.known_n = 0            # 已知格数（空闲+占据），在 find_frontiers 里更新
        self.known_at_goal = 0      # 发目标那一刻的已知格数
        self.pending_gain = None    # (时刻, 发目标时已知格数, 目标坐标) 待核对
        # ---- 路径预检（2026-10-25 新增）----
        # 踩过的坑：Nav2 在探索日志里留下 50 次 "GridBased failed to generate a valid path"。
        # 那些目标在 /map 上明明是空闲格、与车同连通域，但规划器（看的是带膨胀的
        # costmap）当场算不出路径 → BT 反复重规划/恢复动作 → 车一步不动 →
        # 12 s 看门狗判定"卡死"并**永久拉黑这块 frontier**（其实边界没问题，只是那一点不可规划）。
        # 修法：发导航目标之前，先向 compute_path_to_pose 要一条路径；
        # 拿不到就临时跳过该点（30 s 后可重试），不发车、不拉黑。
        self.plan_check = plan_check
        self.plan_timeout = plan_timeout
        self.plan_cand = None       # 正在预检的候选 (tx, ty, dist, size)
        self.plan_fut = None        # 当前阶段等待的 future
        self.plan_phase = ''        # 'send' | 'result'
        self.plan_deadline = 0.0
        self.soft_black = []        # [(x, y, 过期时刻)] 路径预检失败的临时黑名单
        self.n_plan_reject = 0
        self.n_plan_ok = 0
        self.n_plan_detour = 0
        self.n_plan_timeout = 0
        self.plan_disabled = False  # 连续超时后整体关掉预检（宁可回到旧行为，也不能不发目标）
        # 目标净空：目标格周围这一圈内不能有占据格。
        # 机器人半径 0.23 m + 代价地图膨胀 0.45 m，frontier 又天生贴着未知区边界，
        # 常常就在墙角边上 —— 直接把这种格子当目标，Nav2 会"规划得出来但走不到"，
        # 表现为原地打转 + number_of_recoveries 一直涨，探索就此停住（实测耗掉 20 分钟）。
        self.goal_clearance = goal_clearance
        # 看门狗：目标被接受后 stall_secs 秒内位移不足 stall_meters 视为卡死
        self.stall_secs = stall_secs
        self.stall_meters = stall_meters
        # 护栏 ③：最终目标离车必须 ≥ 这个距离。
        # 踩过的坑：snap_clear 在窄通道里会把目标拉回到"车正站着的开阔处"，
        # 于是目标落在 Nav2 的 xy_goal_tolerance(0.25 m) 之内 → Nav2 立刻回 SUCCEEDED，
        # 车一步不动却消耗一个目标。实测 18 秒烧掉 18 个目标、地图完全没变。
        self.min_goal_dist = min_goal_dist
        # 存图前缀：检查点与最终图都用它，方便一次跑一套名字、不覆盖历史地图
        self.map_prefix = map_prefix

        # /map 是 latched（TRANSIENT_LOCAL），这样订阅时能立刻拿到当前地图
        # 回调群分离 + 多线程执行器：on_map 里算 frontier 要 0.5~2 s（地图越大越久），
        # 如果和 action 回调同在一个互斥组里，目标响应握手会被阻塞，Nav2 侧报
        # "Failed to send goal response (timeout)"，客户端永远收不到“已接受”。
        self.cb_map = MutuallyExclusiveCallbackGroup()
        self.cb_tick = MutuallyExclusiveCallbackGroup()
        self.cb_act = ReentrantCallbackGroup()

        map_qos = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos,
                                 callback_group=self.cb_map)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                               callback_group=self.cb_act)
        # 路径预检用的规划器客户端（只用来"问有没有路"，不执行）
        self.planner = ActionClient(self, ComputePathToPose, 'compute_path_to_pose',
                                    callback_group=self.cb_act)
        self.goal_handle = None
        self.goal_xy = None
        self.goal_sent_at = None
        self.busy = False

        self.grid = None          # OccupancyGrid
        self.clusters = []        # 缓存的 frontier 簇
        self.meta = None
        self.clear_m = None       # numpy bool[h][w]：该格周围 goal_clearance 内无占据 → 可当目标
        self.n_free = 0           # 已知自由格数（判断"地图还太小"用）
        self.pause_until = 0.0    # Nav2 未就绪时的退避截止时刻（秒，sim time）
        self.blacklist = []       # [(x, y)] 失败过的目标
        self.stall_ref = None     # 上次"确实有进展"时的车位置
        self.stall_t = None       # 那次的时间戳
        self.n_goals = 0
        self.n_fail = 0
        self.n_zero = 0           # 瞬时完成/瞬时失败（无进展）的目标数，不计入 max_goals
        self.goal_start_pos = None
        # 起点时间必须等 /clock 到了再定：手动在"已经跑了很久的仿真"里启动节点时，
        # __init__ 里还拿不到 use_sim_time 的时钟（now()==0），一旦写成 t_start=0，
        # 时钟同步后 elapsed 会直接等于仿真的绝对时间（实测 112 分钟）→ 立刻"超时收工"。
        self.t_start = None
        self.done = False

        self.create_timer(1.0, self.tick, callback_group=self.cb_tick)
        self.get_logger().info(
            f'前沿探索启动：最多 {max_goals} 个目标 / {max_minutes} 分钟 / '
            f'最小簇 {min_cluster} 格 / 目标净空 {goal_clearance:.2f} m / '
            f'最小目标距离 {min_goal_dist:.2f} m / '
            f'看门狗 {stall_secs:.0f} s·{stall_meters:.2f} m / '
            f'信息增量阈值 {min_gain} 格 / '
            f'路径预检 {"开" if plan_check else "关"}（{plan_timeout:.1f} s）')

    # ---------- 数据 ----------
    def on_map(self, msg):
        """地图每 2 s 更新一次；在这里算好 frontier 簇并缓存，
        控制循环里只做“挑一个、发目标”，避免每秒扫 10 万格。"""
        self.grid = msg
        try:
            r = self.find_frontiers()
            if r:
                self.clusters, self.meta, self.n_free = r
        except Exception:
            # 单帧地图算错不能让整个节点死掉（巡游/导航还在跑，死了就全停了）
            self.get_logger().error('frontier 计算异常（已跳过这一帧）：\n'
                                    + traceback.format_exc())

    def robot_pose(self):
        try:
            tr = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
            t = tr.transform.translation
            return t.x, t.y, yaw_from_quat(tr.transform.rotation)
        except Exception:
            return None

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ---------- frontier 检测 ----------
    def find_frontiers(self):
        g = self.grid
        if g is None:
            return []
        w, h = g.info.width, g.info.height
        res = g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        data = g.data

        # 用 numpy 向量化分类 + frontier 检测（原来纯 Python 双循环要 1~2 s，
        # 会把 Nav2 的 20 Hz 控制环饿到 1.5 Hz，表现为 "Failed to make progress"）
        try:
            import numpy as np
            arr = np.asarray(data, dtype=np.int16).reshape(h, w)
            free_m = (arr == 0)
            unk_m = (arr < 0)
            nb = np.zeros_like(unk_m)
            nb[1:, :] |= unk_m[:-1, :]
            nb[:-1, :] |= unk_m[1:, :]
            nb[:, 1:] |= unk_m[:, :-1]
            nb[:, :-1] |= unk_m[:, 1:]
            nb[1:, 1:] |= unk_m[:-1, :-1]
            nb[:-1, :-1] |= unk_m[1:, 1:]
            nb[1:, :-1] |= unk_m[:-1, 1:]
            nb[:-1, 1:] |= unk_m[1:, :-1]
            front_m = free_m & nb
            nf = int(free_m.sum())
            # ---- 目标净空掩码 ----
            # 以"占据格"为源做半径 r 格的方形膨胀，取反后与空闲区相交：
            #   该格周围 r 内没有墙 → 机器人能站得下。
            # 用可分离膨胀（先列后行，每次对原始数组移位）代替圆形核，
            # 13x13 的核从 169 次操作降到 24 次，50 ms 级 → 1 ms 级。
            occ_m = (arr >= 65)
            self.known_n = int(free_m.sum()) + int(occ_m.sum())
            r_clr = max(1, int(round(self.goal_clearance / res)))
            self.clear_m = free_m & ~_dilate_sep(occ_m, r_clr)
            ys, xs = np.nonzero(front_m)
            front_cells = [(int(y) * w + int(x)) for y, x in zip(ys, xs)]
        except Exception:
            # 没有 numpy 时退回纯 Python（慢，但能用）
            self.clear_m = None      # 无净空掩码 → snap_clear 直接放行
            free = bytearray(w * h)
            unk = bytearray(w * h)
            for i in range(w * h):
                v = data[i]
                if v == 0:
                    free[i] = 1
                elif v < 0:
                    unk[i] = 1
            nf = sum(free)
            front_cells = []
            for y in range(1, h - 1):
                row = y * w
                for x in range(1, w - 1):
                    i = row + x
                    if not free[i]:
                        continue
                    hit = any(unk[i + dy * w + dx] for dy in (-1, 0, 1)
                              for dx in (-1, 0, 1) if (dx or dy))
                    if hit:
                        front_cells.append(i)

        front = bytearray(w * h)
        for i in front_cells:
            front[i] = 1

        # 8 连通聚类（只遍历 frontier 格，不再扫全图）
        seen = bytearray(w * h)
        clusters = []
        for i in front_cells:
            if seen[i]:
                continue
            q = deque([i])
            seen[i] = 1
            cells = []
            while q:
                j = q.popleft()
                cells.append(j)
                cy, cx = divmod(j, w)
                for dy in (-1, 0, 1):
                    ny = cy + dy
                    if ny < 0 or ny >= h:
                        continue
                    for dx in (-1, 0, 1):
                        nx = cx + dx
                        if nx < 0 or nx >= w:
                            continue
                        k = ny * w + nx
                        if front[k] and not seen[k]:
                            seen[k] = 1
                            q.append(k)
            if len(cells) >= self.min_cluster:
                mx = sum(c % w for c in cells) / len(cells)
                my = sum(c // w for c in cells) / len(cells)
                # 采样保存簇内格子坐标（最多 200 个），用于"最近距离/最近目标点"判断。
                # 关键：不能用簇质心算距离 —— 小地图时 frontier 是一条围着机器人的闭环，
                # 质心正好落在机器人身上，会被误判成"就在脚下"而跳过。
                step = max(1, len(cells) // 200)
                pts = [(ox + (c % w + 0.5) * res, oy + (c // w + 0.5) * res)
                       for c in cells[::step]]
                clusters.append({
                    'size': len(cells), 'pts': pts,
                    'cx': ox + (mx + 0.5) * res, 'cy': oy + (my + 0.5) * res,
                })
        return clusters, (w, h, res, ox, oy), nf

    def is_blacklisted(self, x, y):
        if any(math.hypot(x - bx, y - by) < self.revisit_gap for bx, by in self.blacklist):
            return True
        # 临时黑名单：路径预检失败的点只挡一段时间，过期后允许重试
        # （永久拉黑会白白丢掉一块其实没问题的边界）
        if self.soft_black:
            t = self.now()
            self.soft_black = [e for e in self.soft_black if e[2] > t]
            return any(math.hypot(x - bx, y - by) < self.revisit_gap
                       for bx, by, _ in self.soft_black)
        return False

    # ---------- 路径预检（发目标前先问题规划器要一条路径）----------
    def soft_blacklist(self, x, y, ttl=30.0):
        self.soft_black.append((x, y, self.now() + ttl))

    @staticmethod
    def _path_length(path):
        tot = 0.0
        ps = path.poses
        for a, b in zip(ps, ps[1:]):
            tot += math.hypot(b.pose.position.x - a.pose.position.x,
                              b.pose.position.y - a.pose.position.y)
        return tot

    def start_plan_check(self, cand):
        """发导航目标前先问规划器要一条路径。

        为什么：探索日志里 50 次 \"GridBased failed to generate a valid path\"。
        那些目标在 /map 上是空闲格、与车同连通域，但规划器看的是**带膨胀的 costmap**，
        当场算不出路径 → BT 反复重规划 + 恢复动作 → 车一步不动 → 12 s 看门狗
        判\"卡死\"并把这块边界**永久拉黑**（边界其实没问题，只是那一点不可规划）。
        """
        tx, ty, dist, size = cand
        if (not self.plan_check) or self.plan_disabled:
            self.send_goal(tx, ty, dist, size, len(self.clusters))
            return
        if not self.planner.server_is_ready():
            self.get_logger().warn('规划器未就绪 → 跳过路径预检，按旧行为直接发目标',
                                   throttle_duration_sec=30.0)
            self.send_goal(tx, ty, dist, size, len(self.clusters))
            return
        g = ComputePathToPose.Goal()
        g.goal.header.frame_id = 'map'
        g.goal.header.stamp = self.get_clock().now().to_msg()
        g.goal.pose.position.x = tx
        g.goal.pose.position.y = ty
        g.goal.pose.orientation.w = 1.0
        g.use_start = False              # 让规划器自己取当前位姿
        self.plan_cand = cand
        self.plan_fut = self.planner.send_goal_async(g)
        self.plan_phase = 'send'
        self.plan_deadline = self.now() + self.plan_timeout

    def poll_plan_check(self):
        """每拍调用：确认预检结果。成功 → 发导航目标；失败 → 临时跳过该点。"""
        if self.plan_phase == '':
            return
        if not self.plan_fut.done():
            if self.now() > self.plan_deadline:
                self.n_plan_timeout += 1
                cand = self.plan_cand
                self.plan_phase = ''
                if cand:
                    self.soft_blacklist(cand[0], cand[1], ttl=15.0)
                self.get_logger().warn(
                    f'路径预检超时（{self.plan_timeout:.1f} s，累计 {self.n_plan_timeout} 次）'
                    f' → 先跳过该点', throttle_duration_sec=10.0)
                if self.n_plan_timeout >= 3:
                    self.plan_disabled = True
                    self.get_logger().warn('路径预检连续超时 → 本轮关闭它，回到旧行为')
            return
        cand, phase = self.plan_cand, self.plan_phase
        self.plan_phase = ''
        if cand is None:
            return
        if phase == 'send':
            try:
                gh = self.plan_fut.result()
                if gh is None or not gh.accepted:
                    raise RuntimeError('规划器拒绝了请求')
                self.plan_fut = gh.get_result_async()
                self.plan_cand = cand
                self.plan_phase = 'result'      # 下一拍再看结果
            except Exception as exc:
                self.n_plan_reject += 1
                self.soft_blacklist(cand[0], cand[1], ttl=30.0)
                self.get_logger().warn(f'规划器拒绝预检请求（{exc}）→ 跳过该目标点')
            return
        # phase == 'result'
        plen, ok = 0.0, False
        try:
            res = self.plan_fut.result()
            ok = res is not None and res.status == 4 and len(res.result.path.poses) >= 2
            if ok:
                plen = self._path_length(res.result.path)
        except Exception:
            ok = False
        tx, ty, dist, size = cand
        if ok:
            # 这条规则来自实测（2026-10-25，25 个目标的路径数据）：
            #   到达组平均"路径/直线" = 1.12；卡死组 = 2.42；
            #   最极端一个是"直线 1.5 m、路径 10.6 m"——目标在另一个封闭区域，
            #   规划器只能从门绕过去，车开到一半在窄处被 DWB 卡住。
            # 判据取保守形式（直线很近但要绕很远），实测零误杀：
            detour = (dist < 3.0 and plen > 3.0 * dist + 1.5)
            if detour:
                self.n_plan_detour += 1
                self.soft_blacklist(tx, ty, ttl=60.0)
                self.get_logger().warn(
                    f'目标 ({tx:+.2f}, {ty:+.2f}) 直线只有 {dist:.1f} m，但要绕 {plen:.1f} m '
                    f'→ 判定在另一个封闭区域，临时跳过（累计 {self.n_plan_detour} 次）')
                return
            self.n_plan_ok += 1
            self.get_logger().info(f'   路径预检通过：路径 {plen:.1f} m（直线 {dist:.1f} m）')
            self.send_goal(tx, ty, dist, size, len(self.clusters))
        else:
            self.n_plan_reject += 1
            self.soft_blacklist(tx, ty, ttl=30.0)
            self.get_logger().warn(
                f'目标 ({tx:+.2f}, {ty:+.2f}) 规划器算不出路径 → 临时跳过'
                f'（累计 {self.n_plan_reject} 次；不发车，也不永久拉黑这块边界）')

    def snap_clear(self, x, y, max_r=1.2):
        """把目标点挪到最近一个"周围有净空"的空闲格。

        返回 (x, y, 挪动距离)；找不到就返回 None（该簇本轮跳过）。
        为什么要挪：frontier 天生贴着未知区边界，边界又常常就在墙角/门框边上。
        目标离墙 0.17 m 时 Nav2 能规划出路径却永远走不到 —— 只能原地打转+
        反复恢复动作（实测在一个目标上耗掉 20 分钟，地图从 16:05 起完全冻结）。
        """
        if self.clear_m is None or self.meta is None:
            return x, y, 0.0
        w, h, res, ox, oy = self.meta
        cx = int((x - ox) / res)
        cy = int((y - oy) / res)
        if not (0 <= cx < w and 0 <= cy < h):
            return None
        if self.clear_m[cy, cx]:
            return x, y, 0.0
        R = max(1, int(round(max_r / res)))
        best, bd = None, None
        for dy in range(-R, R + 1):
            yy = cy + dy
            if yy < 0 or yy >= h:
                continue
            for dx in range(-R, R + 1):
                xx = cx + dx
                if xx < 0 or xx >= w:
                    continue
                if not self.clear_m[yy, xx]:
                    continue
                d2 = dx * dx + dy * dy
                if bd is None or d2 < bd:
                    bd, best = d2, (xx, yy)
        if best is None or bd is None:
            return None
        return (ox + (best[0] + 0.5) * res,
                oy + (best[1] + 0.5) * res,
                math.sqrt(bd) * res)

    def pick_target(self, clusters, meta, pose):
        """逐格筛选 frontier 目标（不是逐簇！）

        教训：用"整簇最近格"作判据时，只要簇里有一个格子离车很近（<0.6 m），
        整簇就被丢掉 —— 而开局地图很小时 frontier 正好是围着机器人的一环，
        环上必然有近格，于是永远发不出目标，形成死锁。
        正确做法：只排除太近的"格子"，再从剩余格子里挑目标。
        """
        rx, ry, ryaw = pose
        best, best_score = None, 0.0
        for c in clusters:
            # 单簇出错（数据异常）只跳过这一簇，不能让整个 tick 抛异常
            try:
                picked = self._score_cluster(c, rx, ry, ryaw)
            except Exception:
                # 静默吞错会让"节点活着但不动"变成谜案（踩过：一次 24 分钟空转），
                # 所以这里必须留下痕迹
                self.get_logger().warn('单簇打分异常（已跳过该簇）：'
                                       + traceback.format_exc(limit=2),
                                       throttle_duration_sec=10.0)
                continue
            if picked is None:
                continue
            score, cand = picked
            if score > best_score:
                best_score, best = score, cand
        return best

    def _score_cluster(self, c, rx, ry, ryaw):
        """给单个 frontier 簇打分并给出目标点；不可用则返回 None"""
        # 候选：距车 >= 0.8 m 的格子；若簇里有 >= 1.5 m 的格子，优先只在其中挑，
        # 这样每一步都有实质进展，不会在脚下反复挪
        far = [(px, py) for (px, py) in c['pts']
               if math.hypot(px - rx, py - ry) >= 1.5]
        near = [(px, py) for (px, py) in c['pts']
                if math.hypot(px - rx, py - ry) >= 0.8]
        cands = far or near
        if not cands:
            return None
        # 候选按"离车由近到远"排序，逐个试：某个候选回退+挪净空之后离车太近，
        # 就换下一个候选，而不是整簇放弃。
        cands.sort(key=lambda p: math.hypot(p[0] - rx, p[1] - ry))
        for (px, py) in cands[:12]:
            if self.is_blacklisted(px, py):
                continue
            tx, ty = px, py
            # 目标从边界往"已知区"回退（默认 0.4 m）：frontier 格常常紧贴墙角或窄缝，
            # 直接把车开过去容易被 Nav2 判为不可达（status=6 aborted）。
            # 关键：回退量不能让目标落进 min_goal_dist 之内，否则会被下面的护栏 ③
            # 全部拒掉 → 小地图启动时"找不到目标"死锁（实测踩过）。
            dd0 = math.hypot(tx - rx, ty - ry)
            back = min(0.4, max(0.0, dd0 - self.min_goal_dist))
            if dd0 > 1e-6 and back > 0.0:
                k = (dd0 - back) / dd0
                tx = rx + (tx - rx) * k
                ty = ry + (ty - ry) * k
            # 净空检查（护栏 ①）：目标附近必须有 ≥ goal_clearance 的净空，
            # 否则挪到最近合格格；1.2 m 内都找不到就换下一个候选。
            snapped = self.snap_clear(tx, ty)
            if snapped is None:
                continue
            tx, ty, shift = snapped
            # 护栏 ③：最终目标离车必须够远。否则它落在 Nav2 的
            # xy_goal_tolerance 里 → 立刻 SUCCEEDED、车不动、白烧一个目标
            # （实测 18 s 烧掉 18 个目标，地图一个字节都没变）。
            d_fin = math.hypot(tx - rx, ty - ry)
            if d_fin < self.min_goal_dist:
                continue
            # 黑名单要按"挪动之后"的最终坐标判：候选格和挪动后的目标可能差 0.35 m，
            # 只判候选会让同一个失败目标被无限重挑（实测 4 次连发同一点）。
            if self.is_blacklisted(tx, ty):
                continue
            if shift > 0.01:
                self.get_logger().info(
                    f'目标贴墙/贴角，已挪动 {shift:.2f} m 到净空 '
                    f'{self.goal_clearance:.2f} m 的格', throttle_duration_sec=15.0)
            # 朝向加成（只当 tie-breaker）：尽量不原地掉头 180°。
            # 万向球在原地转时会打滑，每次掉头都会给偏航累积一点误差，
            # 走廊里 scan matching 又救不回来 —— 少掉头 = 少漂移。
            dyaw = math.atan2(ty - ry, tx - rx) - ryaw
            dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
            fwd = 1.0 + 0.35 * max(0.0, math.cos(dyaw))   # 正前方 1.35x，正后方 1.0x
            score = fwd * c['size'] / (d_fin + 1.0)       # 又大又近又顺路优先
            return score, (tx, ty, d_fin, c['size'])
        return None

    # ---------- 主循环 ----------
    def tick(self):
        """定时器回调：任何异常都只记录、绝不让节点退出
        （节点一死，车就永远停在原地 —— 你刚才遇到的正是这个）"""
        try:
            self._tick()
        except Exception:
            self.get_logger().error('控制循环异常（本拍跳过，继续运行）：\n'
                                    + traceback.format_exc())

    def _tick(self):
        if self.done:
            return
        t_now = self.now()
        if self.t_start is None:
            # 等仿真时钟就绪（手动接续启动时，节点可能比 /clock 先到）
            if t_now <= 0.0:
                return
            self.t_start = t_now
            self.get_logger().info(f'计时起点设定：仿真时刻 {t_now:.1f} s')
        if t_now - self.t_start > self.max_time:
            self.finish('到达时间上限')
            return
        if self.n_goals >= self.max_goals:
            self.finish(f'达到目标数上限（{self.max_goals}）')
            return

        # 延迟核对上一次成功目标的"信息增量"：已知格涨不到 min_gain → 假边界，拉黑。
        # 必须先等 4 s —— 结果回调一到就比对的话，SLAM 还没把车在那儿看到的新格子写进
        # /map，任何目标都会被误判成"零增量"，那就变成把所有边界都拉黑了。
        if self.pending_gain is not None:
            t_chk, known0, xy = self.pending_gain
            if t_now - t_chk >= 4.0:
                self.pending_gain = None
                gain = self.known_n - known0
                if gain < self.min_gain:
                    self.blacklist.append(xy)
                    self.get_logger().warn(
                        f'目标 ({xy[0]:+.2f}, {xy[1]:+.2f}) 已到达，但已知格只涨 {gain} '
                        f'< {self.min_gain} → 判定假边界，拉黑')
                else:
                    self.get_logger().info(f'   信息增量 +{gain} 格（目标 {xy[0]:+.2f},{xy[1]:+.2f}）')

        # 路径预检结果：每拍都要推进，否则预检卡住就再也不发目标了
        self.poll_plan_check()

        pose = self.robot_pose()
        if pose is None or self.grid is None:
            return

        if self.busy:
            # 目标发出后 15 s 还没拿到“已接受”握手 → 说明 Nav2 的 action server 被 DDS
            # 发现了但节点还在 inactive（带 RViz 时激活更慢），目标不会被处理。
            # 关键：这种情况**不能拉黑目标** —— 目标本身是好的；而小地图时 frontier 只是
            # 一圈 1 m 半径的环，一次 0.8 m 拉黑就能把整圈盖住，永久死锁。
            if self.goal_handle is None and self.goal_sent_at \
                    and self.now() - self.goal_sent_at > 15.0:
                self.get_logger().warn(
                    '目标未被接受（Nav2 可能还没就绪）→ 稍后重发同一目标，不拉黑',
                    throttle_duration_sec=5.0)
                self.busy = False
                self.goal_handle = None
                self.goal_xy = None
                self.goal_sent_at = None
                self.n_goals = max(0, self.n_goals - 1)   # 这次不算数
                return
            # 进度看门狗（护栏 ②）：目标已被接受，但 stall_secs 内位移不足
            # stall_meters → Nav2 陷在"原地恢复"死循环里（实测：5 秒转 111°、
            # 反复清代价地图、9 次恢复动作，却始终离目标 5.4 m）。
            # 不干预的话它会把剩下所有时间耗在这一个目标上。
            if self.goal_handle is not None:
                if self.stall_t is None or self.stall_ref is None:
                    self.stall_ref = (pose[0], pose[1])
                    self.stall_t = self.now()
                ref = self.stall_ref
                moved = math.hypot(pose[0] - ref[0], pose[1] - ref[1])
                if moved >= self.stall_meters:
                    self.stall_ref = (pose[0], pose[1])
                    self.stall_t = self.now()
                elif self.now() - self.stall_t > self.stall_secs:
                    gx, gy = self.goal_xy if self.goal_xy else (float('nan'),) * 2
                    # 打印"实际经过的秒数"而不是阈值 —— 之前写的是 self.stall_secs，
                    # 日志里看着像"被接受后 25 s"，其实可能只过了 7 s（BT 重试会重置计时），
                    # 排查时被这个数字误导过。
                    self.get_logger().warn(
                        f'目标 ({gx:+.2f}, {gy:+.2f}) 距上次进展已 '
                        f'{self.now() - self.stall_t:.0f} s（阈值 {self.stall_secs:.0f} s）、'
                        f'只挪了 {moved:.2f} m → 判定卡死，取消并拉黑该目标')
                    try:
                        self.goal_handle.cancel_goal_async()
                    except Exception:
                        pass
                    self.fail_current()
                    return
            if self.goal_sent_at and self.now() - self.goal_sent_at > 240.0:
                self.get_logger().warn('当前目标超时，放弃并拉黑')
                try:
                    if self.goal_handle is not None:
                        self.goal_handle.cancel_goal_async()
                except Exception:
                    pass
                self.fail_current()
            return

        # Nav2 未就绪的退避期：别狂发目标（否则 max_goals 会被瞬间烧光）
        if self.now() < self.pause_until:
            return

        if self.meta is None:
            return
        if self.plan_phase != '':        # 预检结果还没回来，这拍先不发新目标
            return
        target = self.pick_target(self.clusters, self.meta, pose)
        if target is None:
            # 没有可用目标时，要先排除"地图还太小"这种假性完成：
            # 开局已知区域只有几个平方，frontier 是围着机器人的小环，
            # 很容易被误判成"都在脚下"。此时应该等图长起来（机器一动图就长）。
            if self.n_free < 2500:
                self.get_logger().info(
                    f'暂无合适边界，但地图还很小（已知 {self.n_free} 格），继续等待',
                    throttle_duration_sec=10.0)
                return
            self.finish(f'没有可用边界了（已知 {self.n_free} 格），判定探索完成')
            return

        tx, ty, dist, size = target
        # 先问规划器有没有路：算不出路径的目标当场跳过，不发车、不永久拉黑边界
        self.start_plan_check(target)

    def send_goal(self, x, y, dist, size, n_clusters):
        # 记下发目标这一刻的已知格数，到达后用它核对"这一趟到底有没有带来新信息"
        self.known_at_goal = self.known_n
        # 等 2 s：Nav2 刚起时 action server 可能还没被发现
        if not self.ac.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('等不到 NavigateToPose 服务器（Nav2 还没起来？）',
                                   throttle_duration_sec=10.0)
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        # 目标朝向 = 前进方向，**不要写死 yaw=0**。
        # 写死 0 的后果：车到了目标点还要原地转到 +x 方向（DWB 的 RotateToGoal 准则），
        # 万向球原地旋转会打滑 → 转不到位 → 触发恢复动作 → 一个目标能耗掉 3 分钟
        # （实测：25 分钟只走了 8 个目标），同时给偏航累积误差。
        cur = self.robot_pose()
        gyaw = math.atan2(y - cur[1], x - cur[0]) if cur else 0.0
        goal.pose.pose.orientation.z = math.sin(gyaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(gyaw / 2.0)

        self.goal_xy = (x, y)
        self.goal_sent_at = self.now()
        self.busy = True
        self.goal_start_pos = None
        self.stall_ref = None     # 新目标 → 看门狗重新起算
        self.stall_t = None
        self.n_goals += 1
        # 每 8 个目标后台存一张检查点：节点万一死了，也不至于丢掉整轮探索
        if self.n_goals % 8 == 0:
            threading.Thread(target=self._checkpoint, args=(self.n_goals,), daemon=True).start()
        self.get_logger().info(
            f'→ 目标 #{self.n_goals}: ({x:+.2f}, {y:+.2f})  '
            f'距离 {dist:.1f} m  frontier 大小 {size} 格  （候选 {n_clusters} 处）')
        fut = self.ac.send_goal_async(goal)
        fut.add_done_callback(self.on_goal_accepted)

    def on_goal_accepted(self, fut):
        # action 回调里的异常必须自己吞掉：rclpy 中 done-callback 抛异常会直接终止节点
        try:
            gh = fut.result()
        except Exception:
            self.get_logger().error('goal 提交回调异常：\n' + traceback.format_exc())
            self.fail_current()
            return
        if gh is None or not gh.accepted:
            # 目标被拒 ≠ 目标不好。最常见原因是 Nav2 还没激活完
            # （lifecycle_manager 卡在 Configuring，节点停在 inactive），
            # 此时**绝不能拉黑目标** —— 否则几十个点瞬间被拉黑，
            # 探索就"1 分钟烧光 40 个目标"直接结束（实测踩过）。
            self.get_logger().warn(
                '目标被拒（Nav2 可能还没激活完，看 ros2 lifecycle get /bt_navigator）'
                '→ 10 s 后重试，不拉黑', throttle_duration_sec=10.0)
            self.busy = False
            self.goal_handle = None
            self.goal_xy = None
            self.goal_sent_at = None
            self.n_goals = max(0, self.n_goals - 1)   # 这次不算数
            self.pause_until = self.now() + 10.0
            return
        self.goal_handle = gh
        self.goal_start_pos = self.robot_pose()
        try:
            gh.get_result_async().add_done_callback(self.on_goal_result)
        except Exception:
            self.get_logger().error('注册结果回调失败：\n' + traceback.format_exc())
            self.fail_current()

    def on_goal_result(self, fut):
        try:
            status = fut.result().status
        except Exception:
            self.get_logger().warn('取 goal 结果失败（服务器消失？）→ 拉黑该点')
            self.n_fail += 1
            self.fail_current()
            return
        # 零进展判定：目标 2.5 s 内就结束、而车挪动 < 0.15 m
        #   status=4 → 目标落在 Nav2 容差内，瞬间 SUCCEEDED（白烧一个目标）
        #   status=6 → 从当前位置出发立刻失败（同样没有信息量）
        # 两种都不该计入 max_goals，否则一轮探索会在十几秒内"用光配额"。
        el = self.now() - (self.goal_sent_at or self.now())
        cur = self.robot_pose()
        if cur and self.goal_start_pos:
            moved = math.hypot(cur[0] - self.goal_start_pos[0],
                               cur[1] - self.goal_start_pos[1])
        else:
            moved = 99.0
        zero = (el < 2.5 and moved < 0.15)
        # 4 = STATUS_SUCCEEDED
        if status == 4 and zero:
            self.get_logger().warn(
                f'目标 {self.goal_xy} 在 {el:.1f} s 内"完成"，但车只挪了 {moved:.2f} m '
                f'→ 判定落在容差内，拉黑且不计入目标数')
            self.n_zero += 1
            self.n_goals = max(0, self.n_goals - 1)
            if self.goal_xy:
                self.blacklist.append(self.goal_xy)
        elif status == 4:
            self.get_logger().info(f'✓ 到达目标 {self.goal_xy}，继续找下一个边界')
            # 延迟 4 s 核对信息增量（SLAM 要几帧才把新看到的格子写进 /map）
            if self.goal_xy:
                self.pending_gain = (self.now(), self.known_at_goal, self.goal_xy)
        elif status == 5 and self.done:
            # 收尾时 finish() 主动取消的在飞目标，不算失败（否则总结里的"失败 N 个"会误导）
            self.get_logger().info('在飞目标已在收尾时取消（正常）')
        else:
            if zero:
                self.get_logger().warn(
                    f'✗ 目标 {self.goal_xy} 在 {el:.1f} s 内失败（status={status}）、'
                    f'车只挪了 {moved:.2f} m → 瞬时失败，拉黑且不计入目标数')
                self.n_zero += 1
                self.n_goals = max(0, self.n_goals - 1)
            else:
                self.get_logger().warn(f'✗ 目标 {self.goal_xy} 失败（status={status}）→ 拉黑')
                self.n_fail += 1
            if self.goal_xy:
                self.blacklist.append(self.goal_xy)
        self.busy = False
        self.goal_handle = None
        self.goal_xy = None

    def fail_current(self):
        if self.goal_xy:
            self.blacklist.append(self.goal_xy)
        self.busy = False
        self.goal_handle = None
        self.goal_xy = None
        self.stall_ref = None
        self.stall_t = None

    def _checkpoint(self, n):
        name = f'{self.map_prefix}_checkpoint_{n:02d}'
        path = os.path.join(MAP_DIR, name)
        ok, out = save_map(path)
        if ok:
            self.get_logger().info(f'💾 探索检查点已存：config/maps/{name}.pgm')
        else:
            self.get_logger().warn(f'检查点存图失败（不影响探索）：{out[-160:]}')

    def finish(self, why):
        self.done = True
        # 先把在飞的目标取消掉，让车就地停住（否则 Nav2 会继续把它开到旧目标）
        try:
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
        except Exception:
            pass
        el = self.now() - (self.t_start if self.t_start is not None else self.now())
        self.get_logger().info('=' * 62)
        self.get_logger().info(f'🏁 探索结束：{why}')
        self.get_logger().info(f'   用时 {el/60:.1f} 分钟（仿真时间）｜'
                               f'发出目标 {self.n_goals} 个｜失败 {self.n_fail} 个｜'
                               f'无进展 {self.n_zero} 个（不计配额）')
        if self.plan_check and not self.plan_disabled:
            self.get_logger().info(
                f'   路径预检：通过 {self.n_plan_ok} 次｜拦下不可规划 {self.n_plan_reject} 个｜'
                f'绕路过大 {self.n_plan_detour} 个｜超时 {self.n_plan_timeout} 次'
                f'（被跳过的点都不发车、不永久拉黑边界）')
        elif self.plan_disabled:
            self.get_logger().info('   路径预检：本轮被自动关闭（连续超时）')
        self.get_logger().info(f'   💾 正在保存探索地图到 config/maps/{self.map_prefix}.*')
        ok, out = save_map(os.path.join(MAP_DIR, self.map_prefix))
        if ok:
            self.get_logger().info(f'   ✅ 已保存 config/maps/{self.map_prefix}.pgm（巡游图未受影响）')
        else:
            self.get_logger().error(f'   ❌ 存图失败：{out[-200:]}')
        self.get_logger().info('=' * 62)


def main():
    ap = argparse.ArgumentParser(description='前沿探索（自主把可达区域探完）')
    ap.add_argument('--max-goals', type=int, default=120)
    ap.add_argument('--max-minutes', type=float, default=60.0)
    ap.add_argument('--min-cluster', type=int, default=12)
    ap.add_argument('--revisit-gap', type=float, default=0.4,
                    help='拉黑半径 m（别超过 0.5：小地图时 frontier 只是一圈 1 m 的环，'
                         '半径太大会把整圈盖住导致死锁）')
    ap.add_argument('--goal-clearance', type=float, default=0.40,
                    help='目标格周围必须有的净空 m（机器人半径 0.23 + 余量）。'
                         '太小 → 目标贴墙，Nav2 规划得出走不到，恢复动作死循环；'
                         '太大 → 窄门口/小房间会被过度排除')
    ap.add_argument('--stall-secs', type=float, default=12.0,
                    help='看门狗：目标接受后这么多秒内位移不足 --stall-meters 判为卡死')
    ap.add_argument('--stall-meters', type=float, default=0.25,
                    help='看门狗位移阈值 m')
    ap.add_argument('--min-goal-dist', type=float, default=0.60,
                    help='最终目标离车的最小距离 m（Nav2 xy_goal_tolerance=0.20，'
                         '留 3 倍余量；太小会瞬间 SUCCEEDED、白烧一个目标）')
    ap.add_argument('--min-gain', type=int, default=40,
                    help='到达目标后已知格数至少要涨这么多，否则判为假边界并拉黑。'
                         '太小会误杀真边界，太大则假边界仍会反复重选')
    ap.add_argument('--plan-check', dest='plan_check', action='store_true', default=True,
                    help='发导航目标前先向规划器要一条路径（默认开）。'
                         '规划不出来就临时跳过该点，不发车、不永久拉黑边界')
    ap.add_argument('--no-plan-check', dest='plan_check', action='store_false',
                    help='关闭路径预检（回到旧行为：直接发目标，卡了再拉黑）')
    ap.add_argument('--plan-timeout', type=float, default=1.5,
                    help='路径预检的等待上限（仿真秒）；连续 3 次超时会自动关掉预检')
    ap.add_argument('--map-prefix', default='dan_house_explore',
                    help='存图文件名前缀（检查点与最终图都用它）。'
                         '换雷达高度/换车体后请用新前缀，别覆盖历史地图')
    args, _ = ap.parse_known_args()

    rclpy.init(args=None)
    node = FrontierExplore(args.max_goals, args.max_minutes,
                           args.min_cluster, args.revisit_gap,
                           goal_clearance=args.goal_clearance,
                           stall_secs=args.stall_secs,
                           stall_meters=args.stall_meters,
                           min_goal_dist=args.min_goal_dist,
                           map_prefix=args.map_prefix,
                           min_gain=args.min_gain,
                           plan_check=args.plan_check,
                           plan_timeout=args.plan_timeout)
    # 多线程执行器：建图计算（重）与 action 握手（需及时）不能互相阻塞
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
