"""单独测 reporter 的延迟：/no_think 有没有用、冷加载占多少。

不连 ROS、不发车，只做 LLM 调用。
跑法：.venv/bin/python dev/check_report.py
"""
import asyncio
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

from agent_framework import Agent                                    # noqa: E402
from agent_framework.openai import OpenAIChatClient                 # noqa: E402

from agent_nav import API_KEY, BASE_URL, REPORT_INSTRUCTIONS        # noqa: E402

ACTION = ('✅ 去 冰箱（fridge_1），耗时 86.6s，状态 succeeded，恢复 0 次；'
          '终点 (-1.61, +5.04)，距物体中心 1.07 m（规划站姿 0.94 m）')
ASK = '请用一句中文如实汇报给用户。'


async def one(reporter, tag, prompt):
    t0 = time.time()
    res = await reporter.run(prompt)
    el = time.time() - t0
    text = getattr(res, 'text', None) or str(res)
    print(f'  {tag:22s} {el:6.1f}s  → {text.strip()[:60]}')
    return el


async def main():
    model = sys.argv[1] if len(sys.argv) > 1 else 'qwen3-vl:8b'
    client = OpenAIChatClient(model=model, api_key=API_KEY, base_url=BASE_URL)
    reporter = Agent(client=client, instructions=REPORT_INSTRUCTIONS,
                     default_options={'temperature': 0.0})
    prompt = f'机器人刚执行完，工具返回如下：\n{ACTION}\n{ASK}'

    print('=' * 70)
    print(f'reporter 延迟测试（模型 {model}）')
    print('=' * 70)
    t1 = await one(reporter, '① 冷加载 + thinking', prompt)
    t2 = await one(reporter, '② 热 + thinking', prompt)
    t3 = await one(reporter, '③ 热 + /no_think', prompt + '/no_think')
    t4 = await one(reporter, '④ 热 + /no_think（复测）', prompt + '/no_think')

    print()
    print(f'冷加载代价 ≈ {t1 - t2:.1f}s')
    print(f'thinking 代价 ≈ {t2 - min(t3, t4):.1f}s（② 减 ③④ 较小者）')
    print(f'/no_think 后稳定在 {min(t3, t4):.1f}s')


if __name__ == '__main__':
    asyncio.run(main())
