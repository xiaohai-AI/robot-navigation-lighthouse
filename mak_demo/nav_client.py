"""agent 侧导航封装：只发话题、读状态，不碰 rclpy。

ROS 侧由 agent_bridge.py 把 NavigateToPose 的 action 变成：
    /agent/nav_goal    PoseStamped   ← 我们发
    /agent/nav_status  String(JSON)  → 我们读
"""
import json
import math
import threading
import time

POSE_T = 'geometry_msgs/PoseStamped'
STR_T = 'std_msgs/String'
DONE = ('succeeded', 'failed', 'canceled', 'stalled', 'rejected')

# 是否逐 2 s 打印"剩余距离"。交互模式下必须关掉：
# 打印发生在工作线程，而 input() 正在等键盘 —— 两个线程同时写终端会把用户敲的字搅在一起
# （实测「停下」「停下」「冰箱在哪」三行被合并成一句指令「停下停下冰箱在哪」）。
VERBOSE = True

# 在飞的客户端登记表：抢占时要能让它们立刻收手。
# 被取消的 asyncio 任务是撤不掉工作线程的，只能靠这个开关让线程自己退出轮询。
_ACTIVE: set = set()


def stop_all() -> int:
    """叫停所有在飞的导航客户端（抢占用）。返回被叫停的个数。"""
    n = 0
    for c in list(_ACTIVE):
        c.stop()
        n += 1
    return n


class NavClient:
    def __init__(self, link):
        self.link = link
        self.status = {}
        self._stop = False
        self._cv = threading.Condition()
        _ACTIVE.add(self)
        link.subscribe('/agent/nav_status', STR_T, self._on_status)

    def _on_status(self, msg):
        try:
            data = json.loads(msg.get('data', '{}'))
        except Exception:
            return
        with self._cv:
            self.status = data
            self.status_ts = time.time()      # 记录这条状态的到达时刻，用于识别"残留终态"
            self._cv.notify_all()

    def wait_ready(self, timeout=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.status:
                return True
            time.sleep(0.1)
        return False

    def pose(self):
        return self.status.get('pose')

    def go_to(self, x, y, yaw=0.0, timeout=180.0, verbose=None):
        if verbose is None:
            verbose = VERBOSE
        msg = {
            'header': {'frame_id': 'map'},
            'pose': {
                'position': {'x': float(x), 'y': float(y), 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0,
                                'z': math.sin(yaw / 2.0),
                                'w': math.cos(yaw / 2.0)},
            },
        }
        self.link.publish('/agent/nav_goal', POSE_T, msg)
        t_pub = time.time()

        # 必须等"发布之后才收到的新状态"并且 goal 匹配，才算进入本次任务。
        # 只看坐标不行：重复去同一个地方时，上一次目标的终态坐标完全一样
        # （实测第二次跑同一目标，_await_goal 立刻通过，随后读到残留的 succeeded）。
        if not self._await_goal(x, y, t_pub, timeout=8.0):
            if self._stop:                      # 抢占发生在"等 bridge 接受"这段窗口里
                return 'canceled', dict(self.status, detail='agent 抢占取消')
            return 'rejected', {'detail': '状态机没有响应新目标（agent_bridge 挂了？）'}

        t0 = time.time()
        next_print = 0.0
        while time.time() - t0 < timeout:
            if self._stop:                 # 被抢占：立刻收手，不再打印残留状态
                self.cancel()
                return 'canceled', dict(self.status, detail='agent 抢占取消')
            with self._cv:
                st = dict(self.status)
            state = st.get('state')
            el = time.time() - t0
            if verbose and not self._stop and el >= next_print:
                d = st.get('distance_remaining')
                print(f'  [{el:5.1f}s] {state:9s} 剩余 '
                      f'{d if d is None else round(d, 2)} m'
                      f'  恢复 {st.get("recoveries")}')
                next_print = el + 2.0
            # 终态只认"本次发布之后的新状态"，防止被残留终态提前判定成功/失败
            with self._cv:
                ts = getattr(self, 'status_ts', 0.0)
            if ts > t_pub and state in DONE:
                return state, st
            time.sleep(0.2)
        self.cancel()
        return 'timeout', self.status

    def cancel(self):
        self.link.publish('/agent/cancel', 'std_msgs/Empty', {})

    def stop(self):
        """被抢占时由 agent 侧调用：让正在等待的 go_to 立刻返回 canceled。"""
        self._stop = True

    def release(self):
        """任务结束时注销，别让登记表越攒越多。"""
        _ACTIVE.discard(self)

    def _await_goal(self, x, y, t_pub, timeout=8.0, tol=0.05):
        """等一条"发布之后收到的"、goal 匹配的新状态。

        两个条件缺一不可：
          1. 这条状态是在 publish 之后才收到的（status_ts > t_pub）
             —— TRANSIENT_LOCAL 会把最后一条状态补发给新订阅者，那可能是上一个目标的终态；
          2. goal 坐标匹配 —— 证明 bridge 已经换成本次目标。
        """
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self._stop:
                return False
            with self._cv:
                st, ts = dict(self.status), getattr(self, 'status_ts', 0.0)
            g = st.get('goal')
            if ts > t_pub and g and len(g) == 2 \
                    and abs(g[0] - x) <= tol and abs(g[1] - y) <= tol:
                return True
            time.sleep(0.1)
        return False
