"""MAF 全链路冒烟测试：Agent + 本地 ollama + 一个 function tool。

目的（只验这一件事）：venv 里的 MAF 能真的驱动一个 LLM 并调用我们自己的工具。
不接 ROS、不接地图，纯框架验证。

跑法：
    cd mak_demo && .venv/bin/python _smoke_agent.py
"""
import asyncio

from agent_framework import Agent
from agent_framework.openai import OpenAIChatClient

# ---- 一个假的语义地图工具（真实现以后从 config/semantics.json 读） ----
FAKE_PLACES = {
    '厨房': (0.8, 5.2),
    '客厅': (-6.0, 5.0),
    '书房': (5.2, 5.0),
    '卫生间': (2.3, -4.0),
}


def find_place(name: str) -> str:
    """按名字查一个地点，返回 "名字 -> (x, y)" 或"没找到"。"""
    for k, (x, y) in FAKE_PLACES.items():
        if k in name or name in k:
            return f'{k} -> ({x:.1f}, {y:.1f})'
    return f'没有叫「{name}」的地点，现有：{", ".join(FAKE_PLACES)}'


async def main():
    client = OpenAIChatClient(
        model='qwen2.5:7b',
        api_key='ollama',                       # ollama 不校验，但 SDK 要求非空
        base_url='http://127.0.0.1:11434/v1',   # OpenAI 兼容端点
    )
    agent = Agent(
        client=client,
        instructions=(
            '你是室内机器人导航助手。用户说要去的房间名可能不完整，'
            '你必须先调用 find_place 工具查坐标，再回答。'
            '回答一行：地点 + 坐标。不要编造坐标。'
        ),
        tools=[find_place],
    )

    result = await agent.run('带我去厨房')
    print('--- LLM 回答 ---')
    print(result.text if hasattr(result, 'text') else result)

    # 顺带验证工具调用痕迹（MAF 会把 tool call/result 放进 messages）
    try:
        for m in result.messages:
            for c in getattr(m, 'contents', []) or []:
                kind = type(c).__name__
                if 'Function' in kind:
                    print(f'--- 工具轨迹: {kind} ---')
                    print(c)
    except Exception as e:                                     # 只为观测，不影响结论
        print(f'(工具轨迹读不到: {e})')


if __name__ == '__main__':
    asyncio.run(main())
