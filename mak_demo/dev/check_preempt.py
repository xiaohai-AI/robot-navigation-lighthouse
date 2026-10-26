#!/usr/bin/python3
"""抢占（中途改主意）的离线验收 —— 不需要 ROS、不需要仿真、不需要模型。

为什么必须离线验：
    这个功能的三个失效方式都只在"车正在走"的那个窗口里出现，
    而那个窗口在真仿真里要靠人肉抓时机。用假连接把窗口固定下来：
        · 判定错了（该改道却停车 / 该停车却继续走）
        · 取消了旧目标，但新目标发不出去（用户等到死）
        · 取消之后谎报"已到达"（上一轮遗留的终态被读成新目标的结果）

四组用例：
    ① 判定表          纯函数，6 条边界
    ② 真句子的解析     SemanticMap + 关键词，确认"别去厨房了，去冰箱"判成改道
    ③ cancel 的机制    假连接：go_to 卡住时发 cancel，必须在 2 s 内返回 canceled
    ④ 事件循环是否活着  to_thread vs 同步直调，心跳计数对比（这正是旧实现的病根）

跑法：.venv/bin/python dev/check_preempt.py
"""
from __future__ import annotations

import asyncio
import json
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from agent_nav import classify_intent, plan_action            # noqa: E402
from nav_client import NavClient                             # noqa: E402
from preempt import describe, preempt_decision, wants_cancel  # noqa: E402
from semantic_map import SemanticMap                         # noqa: E402

OK = '\033[32mPASS\033[0m'
NG = '\033[31mFAIL\033[0m'
fails = []


def check(name, cond, detail=''):
    print(f'  {"✅" if cond else "❌"} {name}{("　" + detail) if detail else ""}')
    if not cond:
        fails.append(name)


# ---------------- ① 判定表 ----------------
def t_decision():
    print('\n① 判定表（running=车在路上, cancel=句中有叫停词, plan=有可解析的新目的地, ask=问句）')
    cases = [
        # (说明, running, cancel, is_act, has_plan, is_ask, 期望)
        ('空闲时说"去冰箱"',                 False, False, True,  True,  False, 'run'),
        ('车在路上时说"去冰箱"',             True,  False, True,  True,  False, 'preempt'),
        ('车在路上时说"别去厨房了，去冰箱"',  True,  True,  True,  True,  False, 'preempt'),
        ('车在路上时说"停下"',               True,  True,  False, False, False, 'cancel'),
        ('空闲时说"算了"',                   False, True,  False, False, False, 'cancel'),
        ('车在路上时问"冰箱在哪"',            True,  False, False, False, True,  'parallel'),
        # 终端被搅乱时的拼接句（实测踩到）：问句优先于叫停词，否则问题被吞掉
        ('拼接句"停下停下冰箱在哪"',          True,  True,  False, False, True,  'parallel'),
        ('空闲时"算了，冰箱在哪"',            False, True,  False, False, True,  'run'),
    ]
    for label, run_, cancel, act, plan, ask, want in cases:
        got = preempt_decision(running=run_, cancel=cancel, is_act=act,
                               has_plan=plan, is_ask=ask)
        check(f'{label:<28s} → {want:<8s}', got == want, f'实际 {got}')


# ---------------- ② 真句子 ----------------
def t_phrases():
    print('\n② 真实句子的解析（SemanticMap，纯离线）')
    sm = SemanticMap()
    rows = [
        ('别去厨房了，去冰箱', True,  True,  '改道：有新目的地'),
        ('不用去了',           True,  False, '叫停：无目的地'),
        ('停下',               True,  False, '叫停'),
        ('带我去冰箱',         False, True,  '正常发车'),
    ]
    for instr, want_cancel, want_plan, note in rows:
        c = wants_cancel(instr)
        p = plan_action(sm, instr)
        check(f'「{instr}」 叫停={c} 目的地={"有" if p else "无"}　{note}',
              c == want_cancel and (p is not None) == want_plan)

    # 关键边界：既有新目的地又叫停时，判定必须落在 preempt（用户最终想去新地方）
    dec = preempt_decision(running=True, cancel=wants_cancel('别去厨房了，去冰箱'),
                           is_act=classify_intent('别去厨房了，去冰箱') == 'ACT',
                           has_plan=plan_action(sm, '别去厨房了，去冰箱') is not None)
    check('「别去厨房了，去冰箱」车在路上 → preempt', dec == 'preempt', f'实际 {dec}')

    # 分句：叫停段不能被拿去解析目的地（实测「别去了，去客厅」解析出"别了"这个房间）
    from preempt import filter_cancel_segments
    seg_cases = [
        (['别去了', '去客厅'],              ['去客厅']),
        (['别去厨房了', '去卧室'],          ['去卧室']),
        (['停下'],                          ['停下']),          # 纯叫停要留给路线 F
        (['去厨房', '到冰箱旁边'],          ['去厨房', '到冰箱旁边']),
    ]
    for segs, want in seg_cases:
        got = filter_cancel_segments(segs)
        check(f'分句 {segs} → {want}', got == want, f'实际 {got}')


# ---------------- ③ 假连接 + cancel 机制 ----------------
class FakeLink:
    """假 rosbridge：记录 publish，并按"bridge 的行为"回放状态。

    只有两条规则，正好覆盖 cancel 的语义：
        /agent/nav_goal  → 立刻回一条 active（带 goal 坐标）
        /agent/cancel    → 回一条 canceled（终态）
    """

    def __init__(self):
        self.published = []
        self.subs = {}
        self.last_goal = None

    def subscribe(self, topic, typ, cb):
        self.subs[topic] = cb

    def publish(self, topic, typ, msg):
        self.published.append((topic, msg))
        if topic == '/agent/nav_goal':
            self.last_goal = [msg['pose']['position']['x'], msg['pose']['position']['y']]
            self._emit({'state': 'active', 'goal': self.last_goal,
                        'distance_remaining': 3.0, 'recoveries': 0, 'pose': [0.0, 0.0]})
        elif topic == '/agent/cancel':
            self._emit({'state': 'canceled', 'goal': self.last_goal,
                        'distance_remaining': 2.0, 'recoveries': 0, 'pose': [0.0, 0.0]})

    def _emit(self, d):
        cb = self.subs.get('/agent/nav_status')
        if cb:
            cb({'data': json.dumps(d)})

    def close(self):
        pass


def t_cancel_mechanism():
    print('\n③ 取消机制：go_to 卡住时发 cancel，必须让等待立刻结束')
    link = FakeLink()
    nav = NavClient(link)
    nav.wait_ready(timeout=1.0)
    res = {}

    th = threading.Thread(target=lambda: res.update(r=nav.go_to(1.0, 2.0, timeout=8.0,
                                                               verbose=False)))
    th.start()
    time.sleep(0.6)
    still_running = th.is_alive()
    nav.cancel()
    th.join(3.0)

    check('车在"路上"时 go_to 确实还没返回', still_running)
    got = res.get('r')
    state = got[0] if isinstance(got, tuple) else got      # go_to 返回 (state, status)
    check('发出 cancel 后 go_to 返回 canceled', state == 'canceled', f'实际 {state}')
    goals = [m for t, m in link.published if t == '/agent/nav_goal']
    cancels = [m for t, m in link.published if t == '/agent/cancel']
    check('只发了 1 个新目标', len(goals) == 1, f'实际 {len(goals)}')
    check('只发了 1 次取消', len(cancels) == 1, f'实际 {len(cancels)}')
    nav.release()


# ---------------- ④ 事件循环是否活着 ----------------
async def _measure(use_thread: bool) -> int:
    ticks = 0

    async def heartbeat():
        nonlocal ticks
        for _ in range(40):
            ticks += 1
            await asyncio.sleep(0.02)

    async def drive():
        if use_thread:
            await asyncio.to_thread(time.sleep, 0.5)
        else:
            time.sleep(0.5)          # 旧写法：同步阻塞

    hb = asyncio.create_task(heartbeat())
    await drive()
    hb.cancel()
    return ticks


def t_loop_alive():
    print('\n④ 事件循环的活性（这就是旧实现收不到键盘输入的原因）')
    blocking = asyncio.run(_measure(False))
    threaded = asyncio.run(_measure(True))
    check(f'同步直调工具：心跳 {blocking} 次（循环被堵死）', blocking <= 2)
    check(f'to_thread 调用：心跳 {threaded} 次（循环活着）', threaded >= 10)
    check('两者差异显著（抢占的前提成立）', threaded > blocking + 5)

    # 源码级护栏：别再退回同步调用
    src = (Path(__file__).resolve().parent.parent / 'agent_nav.py').read_text()
    direct = [ln.strip() for ln in src.splitlines()
              if "by_name['go_to'](key)" in ln or "by_name['look_at'](key)" in ln]
    check('agent_nav.py 里没有同步的工具直调', not direct,
          f'{len(direct)} 处' if direct else '')
    n_thread = src.count('asyncio.to_thread(')
    check('三条发车路线都走了 to_thread', n_thread >= 3, f'实际 {n_thread} 处')
    check('路线 A 用 fn 间接调用也走了 to_thread',
          'await asyncio.to_thread(fn, key)' in src)


def t_stop_all():
    print('\n⑤ 抢占的实际收手动作：stop_all() 必须让在飞的 go_to 立刻结束')
    import nav_client as navc
    link = FakeLink()
    nav = NavClient(link)
    nav.wait_ready(timeout=1.0)
    res = {}
    th = threading.Thread(target=lambda: res.update(r=nav.go_to(3.0, 4.0, timeout=8.0,
                                                               verbose=False)))
    th.start()
    time.sleep(0.4)
    n = navc.stop_all()
    th.join(2.0)
    got = res.get('r')
    state = got[0] if isinstance(got, tuple) else got
    check('stop_all 找到在飞的客户端', n >= 1, f'实际 {n} 个')
    check('go_to 立刻返回 canceled（不再空转到 180 s 超时）', state == 'canceled',
          f'实际 {state}')
    nav.release()

    # 源码护栏：桥接必须把目标序号传给三个异步回调，否则旧目标的结局会覆盖新目标
    bridge = (Path(__file__).resolve().parent.parent.parent / 'agent_bridge.py').read_text()
    for frag in ('self.goal_seq += 1', 'self.on_result(f, s)',
                 'self.on_goal_response(f, s)', 'self.on_feedback(fb, s)'):
        check(f'agent_bridge.py 含「{frag}」', frag in bridge)

    # 源码护栏：交互模式默认关掉逐行进度（否则和输入抢终端）
    main_src = (Path(__file__).resolve().parent.parent / 'main.py').read_text()
    check('main.py 用 navc.VERBOSE = progress 控制进度打印', 'navc.VERBOSE = progress' in main_src)
    check('main.py 抢占时先 stop_all 再 cancel 任务',
          'navc.stop_all()' in main_src and 'old_task.cancel()' in main_src)


def main():
    print('=' * 70)
    print('抢占（中途改主意）离线验收')
    print('=' * 70)
    t_decision()
    t_phrases()
    t_cancel_mechanism()
    t_loop_alive()
    t_stop_all()
    print()
    if fails:
        print(f'❌ 失败 {len(fails)} 项：{fails}')
        print(f'判定说明表：{[describe(d) for d in ("preempt", "cancel", "parallel", "run")]}')
        return 1
    print('抢占验收 通过 ✅')
    return 0


if __name__ == '__main__':
    sys.exit(main())
