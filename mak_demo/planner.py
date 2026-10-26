"""复合指令拆解：一句话 → 多个目标 → 串行执行（阶段 2A 步骤 5·第一种模式）。

设计要点（和 agent_nav.py 同一套原则）：
  * 拆句子由**代码**做（分隔词 + 语义解析器），不交给 LLM —— 拆错了会开错房间；
  * 每个子目标仍然走「语义解析 → 代码发车」，LLM 只在最后把整段结果说成人话；
  * 某个子目标解析不到就如实拒绝它，**不影响后面的**，并把它写进汇报里。

为什么要"继续执行后面的"：复合指令是用户一次说完的意图，
中间一个地点不存在（"去厨房，然后去地下室"），不该把前面已经完成的动作也否掉。
"""
from __future__ import annotations

import re

# 分隔词：长词在前，避免"然后"被单字"后"之类拆坏
_SEP = re.compile(r'然后|接着|紧接着|之后再|之后|最后|并且|再去|，再|,再|；|;|。|，|,\s*')


def split_segments(instr: str) -> list:
    """把复合指令拆成子句列表。'去厨房，然后到冰箱旁边' → ['去厨房', '到冰箱旁边']。"""
    parts = [p.strip(' ，。；;,、') for p in _SEP.split(instr or '')]
    return [p for p in parts if p]


def plan_sequence(sm, instr: str) -> list:
    """返回 [(子句, 计划或None)]，计划来自 semantic_map 的确定性解析。"""
    from agent_nav import plan_action          # 复用同一套解析，避免两处逻辑漂移

    out = []
    for seg in split_segments(instr):
        out.append((seg, plan_action(sm, seg)))
    return out


def describe_plan(plans: list) -> str:
    """人类可读的计划表（打印用）。"""
    if not plans:
        return '（空指令）'
    lines = []
    for i, (seg, plan) in enumerate(plans, 1):
        what = plan[2] if plan else '❌ 解析不到目标'
        lines.append(f'  {i}. 「{seg}」→ {what}')
    return '\n'.join(lines)


def summarize(entries: list) -> str:
    """把逐步执行结果拼成一段结构化文本，交给 reporter 改写成人话。

    entries: [(子句, 计划, 工具结果或None, 拒绝原因或None)]
    """
    lines = []
    for i, (seg, plan, result, refuse) in enumerate(entries, 1):
        if refuse:
            lines.append(f'{i}. 「{seg}」：{refuse}')
        else:
            lines.append(f'{i}. 「{seg}」：{result}')
    return '\n'.join(lines)
