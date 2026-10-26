"""语言层基准：把踩过的中文坏例子固化成可复跑的用例，用来横向比模型。

只管"说话"这一段，不连 ROS、不开车、不碰相机：
    A 组  问句查证（B 模式）：模型要调 find_place，然后用中文答，不许泄漏工具名/坐标/编数据
    B 组  结果汇报（reporter）：喂它一段真实工具返回值，看它说成人话的质量

为什么需要它：
    "换更强的大模型有用吗"这个问题，只有量出来才能答。
    另外它也是防退化的闸门 —— 以后改提示词/换模型，先跑这个。

跑法：
    .venv/bin/python dev/bench_language.py                       # 默认 qwen3-vl:8b + qwen2.5:7b
    .venv/bin/python dev/bench_language.py --model qwen3:latest
    # 换 cloud（OpenAI 兼容端点）：
    .venv/bin/python dev/bench_language.py \
        --base-url https://api.deepseek.com/v1 --api-key $DEEPSEEK_API_KEY --model deepseek-chat
"""
from __future__ import annotations

import argparse
import asyncio
import os
import re
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

from agent_framework import Agent                          # noqa: E402
from agent_framework.openai import OpenAIChatClient        # noqa: E402

from agent_nav import (INSTRUCTIONS, REPORT_INSTRUCTIONS,  # noqa: E402
                       TOOL_NAMES, garble_check, tool_leak_check,
                       collect_allowed_names)
from dev.check_tools import FakeNav                        # noqa: E402
from semantic_map import SemanticMap                       # noqa: E402
from tools import build_tools                              # noqa: E402

# ---------------- A 组：问句查证 ----------------
# (指令, 期望回答里出现的同义词组（每组任一命中即可）, 期望不出现的词)
ASK_CASES = [
    ('洗手间在哪',      [['洗手间', '卫生间']], []),
    ('家里有哪些房间',  [['客厅'], ['厨房'], ['储藏间']], ['储藏室']),   # 数据里是"储藏间"
    ('冰箱在哪',        [['厨房']], []),
    ('沙发在哪个房间',  [['客厅']], []),
    ('床在哪',          [['卧室']], []),
    ('储藏间在哪',      [['储藏间'], ['走廊', '南']], []),
]

# ---------------- B 组：结果汇报 ----------------
# (工具返回值, 期望出现的词, 期望不出现的词)
REPORT_CASES = [
    ('✅ 去 冰箱（fridge_1），耗时 86.6s，状态 succeeded，恢复 0 次；'
     '终点 (-1.61, +5.04)，距物体中心 1.07 m（规划站姿 0.94 m）',
     ['冰箱'], ['fridge_1', '-1.61', '0.94', '站姿']),
    ('❌ 去 沙发（sofa_1） 未到达，耗时 13.1s，状态 stalled，恢复 0 次；'
     '终点 (-0.56, +2.25)，距物体中心 8.10 m（规划站姿 1.80 m）',
     ['沙发'], ['sofa_1', 'stalled', '8.10']),
    ('✅ 已到 冰箱（fridge_1）的观察站位，距中心 1.79 m，耗时 54.7s；'
     '视觉确认一致 ✅（VLM 认为主体是「冰箱」），依据：白色双开门带长把手；'
     'VLM 11.1s，帧 logs/look_220537.png',
     ['冰箱'], ['fridge_1', 'look_220537', '1.79']),
    ('✅ 去 客厅（living_room）中心，耗时 88.0s，状态 succeeded，剩余 0.228 m',
     ['客厅'], ['living_room', '0.228']),
]


def check(text: str, must: list, banned: list, sm) -> list:
    """返回问题列表（空 = 通过）。must 是"同义词组"列表：每组任一命中即可。"""
    problems = []
    for group in must:
        words = group if isinstance(group, (list, tuple)) else [group]
        if not any(w in text for w in words):
            problems.append(f'缺少「{"/".join(words)}」')
    for w in banned:
        if w in text:
            problems.append(f'出现禁止词「{w}」')
    leaked = tool_leak_check(text)
    if leaked:
        problems.append(f'泄漏工具名 {leaked}')
    bad = garble_check(text, collect_allowed_names(sm))
    if bad:
        problems.append(f'乱码碎片 {bad}')
    if not text.strip():
        problems.append('空回答')
    if len(text) > 120:
        problems.append(f'太长（{len(text)} 字，用户要的是 1~2 句）')
    if re.search(r'[（(]\s*-?\d+\.\d+', text):
        problems.append('回答里带了括号坐标（用户不需要）')
    return problems


async def run(base_url, api_key, model, reporter_model, timeout):
    sm = SemanticMap()
    fns, _log = build_tools(sm, FakeNav())

    client = OpenAIChatClient(model=model, api_key=api_key, base_url=base_url)
    agent = Agent(client=client, instructions=INSTRUCTIONS, tools=fns,
                  default_options={'temperature': 0.0})
    rclient = OpenAIChatClient(model=reporter_model, api_key=api_key, base_url=base_url)
    reporter = Agent(client=rclient, instructions=REPORT_INSTRUCTIONS,
                     default_options={'temperature': 0.0})

    print('=' * 78)
    print(f'模型 {model} / 汇报 {reporter_model}')
    print(f'端点 {base_url}')
    print('=' * 78)

    total = passed = 0
    t_all = time.time()

    print('\n---- A 组：问句查证（LLM 带工具）----')
    a_answers = []
    for instr, must, banned in ASK_CASES:
        t0 = time.time()
        try:
            res = await asyncio.wait_for(agent.run(instr), timeout=timeout)
            text = (getattr(res, 'text', None) or str(res)).strip()
        except Exception as e:                     # 超时/断网都要记成失败而不是崩掉
            text = f'<异常 {type(e).__name__}: {e}>'
        el = time.time() - t0
        problems = check(text, must, banned, sm)
        total += 1
        ok = not problems
        passed += ok
        a_answers.append((instr, text))
        print(f'\n{"✅" if ok else "❌"} 「{instr}」 {el:5.1f}s')
        print(f'   {text[:160]}')
        for p in problems:
            print(f'   ⚠️ {p}')

    # 跨用例重复检查：不同问题给出同一句话 ⇒ 模型没真读坐标，只套了模板（实测踩过）
    seen = {}
    for instr, text in a_answers:
        seen.setdefault(re.sub(r'\s+', '', text), []).append(instr)
    dups = {k: v for k, v in seen.items() if len(v) > 1}
    if dups:
        print('\n⚠️ 重复回答（模板化）：不同问题给出同一句，说明没真用坐标')
        for k, v in dups.items():
            print(f'   「{k[:50]}」 ← 用于 {v}')
        passed -= len(dups) * 2                    # 每条重复算两处失败

    print('\n---- B 组：结果汇报（reporter，无工具）----')
    for ret, must, banned in REPORT_CASES:
        t0 = time.time()
        try:
            res = await asyncio.wait_for(
                reporter.run(f'机器人刚执行完，工具返回如下：\n{ret}\n请用一句中文如实汇报给用户。'),
                timeout=timeout)
            text = (getattr(res, 'text', None) or str(res)).strip()
        except Exception as e:
            text = f'<异常 {type(e).__name__}: {e}>'
        el = time.time() - t0
        problems = check(text, must, banned, sm)
        total += 1
        ok = not problems
        passed += ok
        print(f'\n{"✅" if ok else "❌"} 汇报 {el:5.1f}s')
        print(f'   {text[:160]}')
        for p in problems:
            print(f'   ⚠️ {p}')

    print()
    print('=' * 78)
    print(f'得分 {passed}/{total}　总耗时 {time.time() - t_all:.1f}s')
    print('=' * 78)
    return 0 if passed == total else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--base-url', default='http://127.0.0.1:11434/v1')
    ap.add_argument('--api-key', default=os.environ.get('OPENAI_API_KEY') or 'ollama')
    ap.add_argument('--model', default='qwen3-vl:8b')
    ap.add_argument('--reporter-model', default='qwen2.5:7b')
    ap.add_argument('--timeout', type=float, default=120.0)
    a = ap.parse_args()
    return asyncio.run(run(a.base_url, a.api_key, a.model,
                           a.reporter_model, a.timeout))


if __name__ == '__main__':
    sys.exit(main())
