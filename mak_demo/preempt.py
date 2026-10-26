"""抢占 / 中途改主意：纯代码判定，不交给模型。

产品行为（用户视角）：

    用户：带我去厨房
    机器人：好的（开始走）
    用户：别去厨房了，去卧室        ← 车还在路上，这句话现在能生效
    机器人：已取消去厨房，改去卧室

为什么判定必须是代码而不是模型：
    · 判错的代价不对称 —— 车该停没停会撞、该走没走走，用户站在旁边干等；
    · 这类句子极短，信息全在关键词里（"别去了" / "停下" / "算了"），
      LLM 既没有更多信息可补，又会引入延迟（实测 reporter 1.9 s、planner 30 s+）；
    · 与 classify_intent 同一个理由：动作由代码保证，模型只负责说人话。

判定优先级（顺序有意义，别改）：

    1. 能解析出新目的地  → preempt（"别去厨房了，去卧室" 同时含取消和新目标，
                                    新目的地优先 —— 用户最终想去的是卧室）
    2. 只是叫停          → cancel
    3. 正在开车但问的是问题 → parallel（问题不打断行驶：问"冰箱在哪"不该让车停下）
    4. 其它              → run（正常执行）

词汇表刻意短：只收真正的停止/反悔说法。"停车" 这类会误伤的词不收进 CANCEL_KWS，
靠第 1 条优先级兜住（能解析出目的地就不当取消）。
"""
from __future__ import annotations

# 只是叫停，不带新目的地
CANCEL_KWS = (
    '别去了', '别去', '不用去了', '不用了', '不去了', '不要去了',
    '算了', '取消', '停下', '停下来', '停一下', '先停', '停',
    '回来', '回原地', '原地等', '先别动', '别动', '等一下', '等等',
)


def wants_cancel(instr: str) -> bool:
    """句子里有没有"叫停/反悔"的意思（不含新目的地时的语义）。"""
    s = (instr or '').strip()
    return any(k in s for k in CANCEL_KWS)


def preempt_decision(*, running: bool, cancel: bool,
                     is_act: bool, has_plan: bool, is_ask: bool = False) -> str:
    """决定这条新输入怎么处理。返回值：

        'preempt'  干掉当前任务，立刻执行新目的地
        'cancel'   只叫停，不执行新的
        'parallel' 当前任务继续，这条并行处理（问句）
        'run'      当前空闲，正常执行
    """
    if is_act and has_plan:
        return 'preempt' if running else 'run'
    # 问句优先于叫停词：一个问题不可能是"停下"的命令。
    # 实测终端被打搅时会拼出「停下停下冰箱在哪」，若按叫停处理，用户的问题就被吞掉了
    # （车停了，但没人回答）。顺序放在 cancel 之前是有意的。
    if is_ask:
        return 'parallel' if running else 'run'
    if cancel:
        return 'cancel'
    if running:
        return 'parallel'
    return 'run'


def describe(dec: str) -> str:
    """给日志/轨迹用的一行说明。"""
    return {
        'preempt': '抢占：取消当前任务并改去新目标',
        'cancel': '取消：只叫停，不发新目标',
        'parallel': '并行：当前任务继续，这条问题同时回答',
        'run': '正常执行',
    }.get(dec, dec)


# 以"取消某个目的地"开头的分句：整段丢掉，不要拿去解析目的地。
# 「别去了，去客厅」按逗号拆成两段后，第一段若不丢，会被当成目的地去解析，
# 于是用户看到「家里没有叫「别了」的地方，这一步跳过」—— 实测踩到。
CANCEL_PREFIX = ('别去', '别到', '别往', '别走', '不用去', '不要去', '不去', '别过来')


def is_cancel_segment(seg: str) -> bool:
    """这一段是"叫停"而不是"前往"吗？"""
    t = (seg or '').strip()
    if not t:
        return True
    if any(t.startswith(p) for p in CANCEL_PREFIX):
        return True
    # 短句里的叫停词（停下 / 算了 / 不用了 / 取消）也算，长句不算：
    # "厨房现在能不能过去" 含"能不能"不含叫停词，不会被误伤。
    return wants_cancel(t) and len(t) <= 6


def filter_cancel_segments(segs: list[str]) -> list[str]:
    """从复合指令的分段里剔掉叫停段。

    全被剔掉时原样返回：一条纯叫停（"停下"）要留给 agent_nav 路线 F 去发取消，
    而不是变成"没有目标"的空执行。
    """
    kept = [s for s in segs if not is_cancel_segment(s)]
    return kept or list(segs)
