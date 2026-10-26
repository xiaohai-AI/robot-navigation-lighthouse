"""阶段 2A 步骤 5：统一入口（复合指令 + 交互模式）。

用法：
    .venv/bin/python main.py "带我去冰箱看看"                     # 单目标（等同 agent_nav.py）
    .venv/bin/python main.py "去厨房，然后到冰箱旁边"              # 复合：串行执行
    .venv/bin/python main.py --interactive                       # 交互模式，一条条下指令

三条路线（与 agent_nav.py 完全一致，不重复实现）：
    单目标 / 问句 / 目标不存在  → 交给 agent_nav.run_instruction（A/B/C 三条路线）
    多目标                    → 本文件串行执行：每个子目标仍由代码解析 + 发车，
                                最后由无工具的 reporter 一起汇报

关于"中途插话改主意"（抢占模式，2026-10-25 已实现）：
    交互模式里，车在路上时再说一句新指令会立刻生效：
        带我去厨房 → （车在路上）→ 别去厨房了，去卧室
        → 已取消「去厨房」，改去「卧室」
    判定在 preempt.py 里，纯代码、不调模型：
        有新目的地 → 抢占（取消旧的 + 发新的）
        只是叫停   → 取消（不发新目标），由 agent_nav 路线 F 处理
        问句       → 并行回答，不打断行驶
    两个前提条件（缺一就退化回"一条跑完再下一条"）：
        ① 工具调用必须 asyncio.to_thread —— 同步调用会把事件循环堵死，
           车在路上的几分钟里 input() 根本收不到；
        ② 每条指令是一个后台 Task，而不是 await 到底。
    离线验证：dev/check_preempt.py（假连接 + 脚本化状态流，不需要 ROS/仿真）。
"""
from __future__ import annotations

import argparse
import asyncio
import sys

from agent_framework import Agent
from agent_framework.openai import OpenAIChatClient

from agent_nav import (API_KEY, BASE_URL, DEFAULT_MODEL, REPORTER_MODEL,
                       classify_intent, plan_action, print_checks,
                       run_instruction, strip_action)
from nav_client import NavClient
from planner import describe_plan, split_segments, summarize
import nav_client as navc
from preempt import (describe, filter_cancel_segments, preempt_decision,
                     wants_cancel)
from rosbridge_link import RosbridgeLink
from semantic_map import SemanticMap
from tools import build_tools

REPORT_SEQ = """你负责把机器人这一串动作如实转述给家里的人。

规则：
1. 只用中文，最多三句；按顺序说每个目标到了没有。
2. 不许添加工具结果里没有的情节（不要编"遇到障碍物""绕了一圈"）。
3. 不许出现工具名、参数名、括号、object_id、坐标数字。
4. 全部成功就说完成了；有失败的必须点出来，不能一句"都好了"带过。

范例：
   "1. 厨房 ✅ 45.2s  2. 冰箱 ✅ 86.1s" → 先去厨房、再到冰箱，两个地方都到了，一共用了两分多。
   "1. 厨房 ✅ 45.2s  2. 地下室 ❌ 没有这个房间" → 厨房到了，但家里没有地下室，那个目标没执行。
"""


async def _run_sequence(plans: list, model: str, temperature: float,
                        reporter_model: str) -> dict:
    """串行执行多个已解析的目标，最后统一汇报。"""
    link = RosbridgeLink()
    nav = None
    try:
        nav = NavClient(link)
        if not nav.wait_ready(timeout=6.0):
            raise RuntimeError('收不到 /agent/nav_status —— agent_bridge.py 起了吗？')

        sm = SemanticMap()
        fns, log = build_tools(sm, nav)
        by_name = {f.__name__: f for f in fns}
        reporter = Agent(
            client=OpenAIChatClient(model=reporter_model, api_key=API_KEY,
                                    base_url=BASE_URL),
            instructions=REPORT_SEQ,
            default_options={'temperature': temperature})

        entries = []
        for i, (seg, plan) in enumerate(plans, 1):
            print(f'\n--- 目标 {i}/{len(plans)}：「{seg}」---')
            if plan is None:
                target = strip_action(seg) or seg
                refuse = f'家里没有叫「{target}」的地方，这一步跳过'
                print(f'  ❌ {refuse}')
                entries.append((seg, None, None, refuse))
                continue

            kind, key, desc = plan
            print(f'  执行：{desc}')
            # to_thread：串行执行也要让事件循环活着，否则连 Ctrl+C 都要等这一步走完
            fn = by_name['go_to'] if kind == 'object' else by_name['go_to_room']
            res = await asyncio.to_thread(fn, key)
            # 一步失败不否决整串：用户是一次说完的意图，剩下的目标仍然有效
            entries.append((seg, plan, res, None))

        summary = summarize(entries)
        res = await reporter.run(
            f'机器人依次执行了 {len(entries)} 个目标，工具返回如下：\n{summary}\n'
            f'请用中文如实汇报。')
        text = getattr(res, 'text', None) or str(res)
        if not text.strip():
            text = '已执行完毕，但汇报失败；请查看上面的工具轨迹。'

        return {'text': text, 'log': log, 'entries': entries, 'mode': '多目标串行'}
    finally:
        if nav is not None:
            nav.release()
        link.close()


async def run_any(instr: str, model: str, temperature: float,
                  reporter_model: str, base_url: str | None = None,
                  api_key: str | None = None) -> dict:
    """单目标 / 问句 → 交给 agent_nav；只有"多目标且至少一个能解析"才串行执行。"""
    sm = SemanticMap()                      # 只用于拆解与校验，不连 ROS
    segs = split_segments(instr)
    # 「别去了，去客厅」这类：叫停段不参与目的地解析（否则会解析成"别了"这个不存在的房间）
    segs = filter_cancel_segments(segs)
    plans = [(seg, plan_action(sm, seg)) for seg in segs]
    multi = len(plans) > 1 and any(p is not None for _s, p in plans)

    # 只把显式给过的端点参数往下传，避免覆盖 agent_nav 里的默认值
    extra = {}
    if base_url:
        extra['base_url'] = base_url
    if api_key:
        extra['api_key'] = api_key

    if classify_intent(instr) == 'ASK' or not multi:
        r = await run_instruction(instr, model, temperature, reporter_model, **extra)
        r['plans'] = plans
        return r

    r = await _run_sequence(plans, model, temperature, reporter_model)
    r.update({'intent': 'ACT', 'plans': plans, 'action': None,
              'elapsed': 0.0, 'sm': sm})
    return r


def _print_result(r: dict, plans: list) -> None:
    if len(plans) > 1:
        print('拆解结果：')
        print(describe_plan(plans))

    print()
    print(f'--- 工具调用轨迹（{len(r["log"])} 次）---')
    for i, c in enumerate(r['log'], 1):
        ret = c['ret'].replace('\n', ' ⏎ ')
        print(f"  {i}. {c['tool']}({c['args']})")
        print(f'     → {ret[:200]}{" …" if len(ret) > 200 else ""}')

    print()
    print('--- 机器人回答 ---')
    print(r['text'])
    print()
    print_checks(r)


def _send_cancel() -> None:
    """尽力取消在飞目标（退出或 /cancel 时用）。连不上就静默放弃。"""
    try:
        link = RosbridgeLink()
        try:
            nc = NavClient(link)
            try:
                nc.cancel()
            finally:
                nc.release()
        finally:
            link.close()
    except Exception as e:                                   # 退出路径不该再抛异常
        print(f'（发送取消失败：{e}）')


async def _shutdown(drive) -> None:
    """退出前尽量让车停下：发一次取消，再撤掉在跑的任务。"""
    await asyncio.to_thread(_send_cancel)
    if drive is not None and not drive.done():
        drive.cancel()
        await asyncio.sleep(0)


async def repl_async(model: str, temperature: float, reporter_model: str,
                     base_url: str | None = None,
                     api_key: str | None = None,
                     progress: bool = False) -> int:
    """交互模式：整个会话只用一个事件循环（+ 抢占）。

    为什么不能每条指令 asyncio.run() 一次：MAF 内部的 httpx2 异步客户端没有 close 接口，
    它的清理任务排在 asyncio.run 关循环之后，于是每条指令都会吐
        RuntimeError: Event loop is closed  (Task exception was never retrieved)
    功能虽然不受影响，但连接不会被回收，多轮之后会泄漏。一个循环跑到底即可。

    抢占的关键不是判定表，而是这里的两件事：
      · input() 走 to_thread —— 否则车在路上时键盘根本读不到；
      · 每条指令是独立 Task —— 否则 await 到底，没有"打断"的入口。
    """
    print('=' * 70)
    print('交互模式：直接输入中文指令    q / exit 退出    /cancel 叫停')
    print('车在路上时也可以说话：说新目的地＝改道，说「停下」＝停车，问问题＝边开边答')
    print('=' * 70)

    # 交互模式下默认关掉逐 2 s 的"剩余距离"打印：它发生在工作线程，会和 input() 抢终端，
    # 实测把用户敲的三行搅成一句「停下停下冰箱在哪」。要看进度加 --progress。
    navc.VERBOSE = progress

    sm = SemanticMap()                      # 只用来判断"这句话有没有新目的地"
    drive = None                            # 当前在跑的"发车"任务
    drive_label = ''
    bg: set = set()

    async def _settle(task, label: str, kind: str) -> None:
        """等一条指令结束并打印结果；被取消时明确说"已取消"，绝不谎报到达。"""
        try:
            r = await task
        except asyncio.CancelledError:
            print(f'\n⏹ 「{label}」已取消（{kind}）')
            return
        except RuntimeError as e:
            print(f'❌ {e}')
            return
        print(f'\n──── 结果：「{label}」 ────')
        _print_result(r, r.get('plans') or [])

    while True:
        try:
            line = (await asyncio.to_thread(input, '\n指令> ')).strip()
        except (EOFError, KeyboardInterrupt):
            print()
            await _shutdown(drive)
            return 0
        if not line:
            continue
        if line.lower() in ('q', 'quit', 'exit'):
            await _shutdown(drive)
            return 0
        if line == '/cancel':
            await asyncio.to_thread(_send_cancel)
            if drive is not None and not drive.done():
                drive.cancel()
                print(f'⏹ 已叫停「{drive_label}」')
            else:
                print('已发送取消（当前没有在跑的任务，清掉可能在飞的目标）')
            continue

        running = drive is not None and not drive.done()
        intent = classify_intent(line)
        has_plan = intent == 'ACT' and plan_action(sm, line) is not None
        dec = preempt_decision(running=running, cancel=wants_cancel(line),
                               is_act=intent == 'ACT', has_plan=has_plan,
                               is_ask=intent == 'ASK')
        print(f'判定：{describe(dec)}')

        if dec == 'preempt' and running and drive is not None:
            # 顺序很重要：先让 bridge 真的停下，再发新目标。
            # 反过来的话，新目标可能刚被接受就被这条取消打掉。
            old, old_task = drive_label, drive
            await asyncio.to_thread(_send_cancel)
            # 光取消 asyncio 任务是撤不掉工作线程的：它还会带着已经冻结的状态继续打印到
            # 超时（实测一行 5.28 m 卡着刷了两分钟）。用登记表让它自己收手。
            navc.stop_all()
            old_task.cancel()
            print(f'⏹ 已取消「{old}」，改去「{line}」')

        task = asyncio.create_task(
            _settle(run_any(line, model, temperature, reporter_model,
                            base_url, api_key),
                    line, '发车' if has_plan else '问答'))
        if has_plan:
            drive, drive_label = task, line
        else:
            bg.add(task)
            task.add_done_callback(bg.discard)


def main():
    ap = argparse.ArgumentParser(description='中文指令 → 机器人（支持复合指令）')
    ap.add_argument('instruction', nargs='*', help='中文指令，例如：去厨房，然后到冰箱旁边')
    ap.add_argument('--interactive', action='store_true', help='交互模式，一条条下指令')
    ap.add_argument('--model', default=DEFAULT_MODEL)
    ap.add_argument('--reporter-model', default=REPORTER_MODEL)
    ap.add_argument('--temp', type=float, default=0.0)
    ap.add_argument('--progress', action='store_true',
                    help='交互模式下逐 2 s 打印剩余距离（会与键盘输入抢终端，默认关闭）')
    ap.add_argument('--base-url', default=None,
                    help='OpenAI 兼容端点（默认本地 ollama；换 cloud 填这里）')
    ap.add_argument('--api-key', default=None,
                    help='端点密钥（也可用环境变量 ROBOT_LLM_API_KEY）')
    a = ap.parse_args()

    if a.interactive:
        try:
            return asyncio.run(repl_async(a.model, a.temp, a.reporter_model,
                                          a.base_url, a.api_key, a.progress))
        except KeyboardInterrupt:
            # 用户按 Ctrl+C 退出：车可能还在跑，尽力发一次取消再走
            print('\n收到 Ctrl+C，正在取消在飞目标…')
            _send_cancel()
            return 0

    instr = ' '.join(a.instruction).strip()
    if not instr:
        print(__doc__)
        return 1

    print('=' * 70)
    print(f'指令: 「{instr}」  （理解/查证 {a.model}，汇报 {a.reporter_model}）')
    print('=' * 70)
    try:
        r = asyncio.run(run_any(instr, a.model, a.temp, a.reporter_model,
                                a.base_url, a.api_key))
    except RuntimeError as e:
        print(f'❌ {e}')
        return 2
    _print_result(r, r.get('plans') or [])
    return 0


if __name__ == '__main__':
    sys.exit(main())
