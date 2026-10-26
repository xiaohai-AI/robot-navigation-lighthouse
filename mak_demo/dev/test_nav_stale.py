"""nav_client 的时序单元测试（不需要 ROS、不需要车）。

抓的是这次真机踩到的 bug：
    上一个目标残留 succeeded → go_to 立刻返回成功 → 车一步没动。

跑法：.venv/bin/python dev/test_nav_stale.py
"""
import json
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from nav_client import NavClient  # noqa: E402


class FakeLink:
    """假 rosbridge：记下 publish，按脚本回放 status。"""

    def __init__(self):
        self.published = []
        self.subs = {}
        self.script = []

    def subscribe(self, topic, msg_type, cb):
        self.subs[topic] = cb

    def publish(self, topic, msg_type, msg):
        self.published.append((topic, msg))
        if topic == '/agent/nav_goal':
            x = msg['pose']['position']['x']
            y = msg['pose']['position']['y']
            threading.Thread(target=self._play, args=(x, y), daemon=True).start()

    def emit(self, payload: dict):
        cb = self.subs.get('/agent/nav_status')
        if cb:
            cb({'data': json.dumps(payload)})

    def _play(self, x, y):
        """模拟 agent_bridge：先 sending，再 active 两拍，再 succeeded。"""
        for state, d in (('sending', None), ('active', 4.0), ('active', 2.0), ('succeeded', 0.0)):
            time.sleep(0.15)
            self.emit({'state': state, 'goal': [round(x, 3), round(y, 3)],
                       'distance_remaining': d, 'recoveries': 0, 'pose': [0, 0, 0]})


fails = 0


# ---------- 用例 1：状态里残留上一个目标的 succeeded，且坐标与新目标完全相同 ----------
# 这是真机上踩到的原样场景：重复去同一个地方，只靠坐标匹配无法区分新旧状态。
link = FakeLink()
nav = NavClient(link)
link.emit({'state': 'succeeded', 'goal': [-1.79, 4.84], 'distance_remaining': 0.208,
           'recoveries': 0, 'pose': [2.8, 0.0, 0.0]})     # 上一轮残留的终态（同坐标）
time.sleep(0.1)

t0 = time.time()
state, st = nav.go_to(-1.79, 4.84, 2.75, timeout=10, verbose=False)
el = time.time() - t0

print(f'① 残留终态场景：返回 {state}，耗时 {el:.2f}s')
ok1 = state == 'succeeded' and el > 0.3         # 必须真的等了新目标，不能 0.0s 返回
print('   ' + ('✅ 没有误读上一个目标' if ok1 else '❌ 又读到了残留终态'))
fails += (not ok1)

# 新目标必须真的发出去了
sent = [m for t, m in link.published if t == '/agent/nav_goal']
ok1b = len(sent) == 1 and abs(sent[0]['pose']['position']['x'] + 1.79) < 1e-6
print(f'   目标消息 {len(sent)} 条，x={sent[0]["pose"]["position"]["x"] if sent else "-"} '
      + ('✅' if ok1b else '❌'))
fails += (not ok1b)


# ---------- 用例 2：状态机根本不响应（agent_bridge 挂了）----------
class DeadLink(FakeLink):
    def _play(self, x, y):
        return                                   # 什么都不回放


link2 = DeadLink()
nav2 = NavClient(link2)
t0 = time.time()
state2, st2 = nav2.go_to(1.0, 1.0, timeout=30, verbose=False)
el2 = time.time() - t0
print(f'② 状态机不响应场景：返回 {state2}，耗时 {el2:.2f}s')
ok2 = state2 == 'rejected' and el2 < 12.0
print('   ' + ('✅ 8 秒内判 rejected，不会傻等 30 秒' if ok2 else '❌ 没按预期拒绝'))
fails += (not ok2)


print()
print('=' * 46)
print('全部通过 ✅' if fails == 0 else f'失败 {fails} 项 ❌')
sys.exit(1 if fails else 0)
