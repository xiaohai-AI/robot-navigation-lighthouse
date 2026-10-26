"""阶段 2A 步骤 4：中文指令 → 机器人导航（MAF + 本地 ollama）。

架构（确定性主导，LLM 只占语言边缘）：

    指令
     ├─ classify_intent()          代码：祈使句(ACT) / 问句(ASK)
     ├─ ACT 且有明确目标 → 代码直接发车（go_to / go_to_room）
     │                     → reporter（无工具权限的 LLM）把结果说成一句人话
     ├─ ACT + "看看/确认" + 物体 → 模式 D：look_at（退到观察站位 + VLM 核对）
     │                     相机离地 0.70 m、离物体 1.40 m 时才看得清；
     │                     贴到 0.55 m 时只拍到面板，VLM 会把冰箱认成门（实测）
     ├─ ACT 但目标解析不到 → 代码直接拒绝：列出家里实际有的房间（不调 LLM，也不发车）
     └─ ASK              → 交给 LLM：调 find_place / list_rooms，再用人话回答

为什么"要不要动手"不由 LLM 决定：
    实测 qwen3-vl:8b 收到「带我去沙发看看」时，只调了 find_place 就答
    "沙发在客厅正中间"收工，车一步没动；加提示词、加 tool_choice="required"
    都没能改掉（ollama 兼容层忽略该字段）。而"该动没动"的代价是用户站在旁边干等。
    结论：动作由代码保证，LLM 负责它真正擅长的 —— 理解口语歧义、把结果说成人话。

跑法：
    .venv/bin/python agent_nav.py "带我去冰箱看看"
    .venv/bin/python agent_nav.py "洗手间在哪"
    .venv/bin/python agent_nav.py "带我去地下室"            # 应老实说没找到
    .venv/bin/python agent_nav.py --model qwen3:latest "洗手间在哪"

模型选型（实测 2026-10-22，本机 ollama）：
    qwen3-vl:8b    completion+vision+tools+thinking   中文准、参数抄得对 → 默认
    qwen2.5:7b     completion+tools                   会输出碎片乱码（"pora房间信息…"）→ 不用
    llama3.1:8b    completion+tools                   会把「洗手间」抄成「汉测式」→ 不用
    gemma3:12b     completion+vision                  没有 tools，无法 function calling → 不用
"""
from __future__ import annotations

import argparse
import asyncio
import os
import re
import sys
import time

from agent_framework import Agent
from agent_framework.openai import OpenAIChatClient

from nav_client import NavClient
from preempt import wants_cancel
from rosbridge_link import RosbridgeLink
from semantic_map import SemanticMap
from tools import build_tools

BASE_URL = 'http://127.0.0.1:11434/v1'      # 本地 ollama 的 OpenAI 兼容端点
API_KEY = 'ollama'                          # ollama 不校验，但 SDK 要求非空
DEFAULT_MODEL = 'qwen3-vl:8b'
# 汇报专用模型：只做"把结果改写成一句人话"，不需要推理。
# 实测（dev/check_report.py）：qwen2.5:7b 热态 1.9 s，qwen3-vl:8b 热态 31.7 s —— 差 15 倍。
REPORTER_MODEL = 'qwen2.5:7b'

# 带工具的 Agent：负责"问句"和"目标解析不到时的查证"
INSTRUCTIONS = """你是这户人家的室内移动机器人助手。你能调用的工具已经接好了真实的机器人硬件。

必须遵守的规则：
1. 用户问"某物在哪 / 有哪些房间"时，必须先调用工具（find_place 或 list_rooms）再回答。
   答案里的房间名、房间数量只能来自工具返回值，不许凭记忆列举。
2. 工具返回没找到，你就如实说没找到，并说出家里实际有哪些房间（来自工具返回值）。
3. 只能报告工具实际返回的结果。工具没说遇到障碍、没说你改变了计划，你就不能那么说。

4. 说方向和距离时，只能引用工具返回值里"从你现在的位置看：<方位>方向 约 X 米"那一段。
   不许自己估、不许说"走廊南边""十米左右"这种没出处的描述。
   工具没给方位就别提方位，只说它在哪个房间。数字照抄阿拉伯数字（"约 5.8 米"），
   不要写成"五点八米"或"大概四点八米"。

5. 你是在跟家里的人说话，不是跟程序员说话。回答里绝对不能出现：
   工具名（find_place / go_to / go_to_room / list_rooms / where_am_i）、参数名、代码、
   括号、object_id、坐标数字。不要说"请调用某工具"或"如需前往请…"。

6. 回答范例，照这个风格说（只用中文，一到两句话）：
   问"洗手间在哪"   → 洗手间在走廊南边，从这边走过去大概十米。
   问"带我去地下室" → 家里没有叫地下室的地方，现有的是客厅、厨房、书房、
                      两间卧室、卫生间和储藏间。
"""

# 不带工具的 Agent：只负责把机器人的执行结果说成人话，物理上不可能再"乱动"
REPORT_INSTRUCTIONS = """你负责把刚发生的机器人动作如实转述给家里的人。

规则：
1. 只用中文，一到两句话。
2. 结果里有"✅"＝已经到了，只说两件事：到了哪儿、花了多久。
   结果里有"❌"＝没到，只说没到以及为什么。
3. 严禁添加工具结果里没有的内容：不许说"准备做什么""接下来要…"，
   不许说距离多少米，不许编"遇到障碍物""绕开家具"之类的情节。
4. 不许出现工具名、参数名、括号、object_id、坐标数字。

范例：
   工具返回"✅ 去 冰箱（fridge_1），耗时 86.6s，状态 succeeded" → 已经到冰箱旁边了，路上很顺，用了一分半。
   工具返回"❌ 去 沙发（sofa_1） 未到达，状态 stalled"        → 没能开到沙发那边，车在途中停住了。

4. 如果结果里有"视觉确认一致"：要说出机器人看到了什么（如"能看到白色双开门和长把手"），
   不要只说"到了"。如果有"视觉与语义地图不符"：如实说"我看到的东西和地图上标的不是一个"。
   如果写"视觉无结论"：就说看得不太清楚，别硬猜。

5. 如果结果是"可通行性核对"：把雷达量到的距离和视觉看到的东西都说出来（例如
   "雷达量到正前方一米二才有东西，画面里也没看到障碍，可以过去"），
   然后给一句结论。两者不一致时如实说"雷达和视觉说法不一样"，不要自己挑一个当定论。
"""


# ---------------- 文本自检（回答质量） ----------------
def collect_allowed_names(sm) -> set:
    """回答里允许出现的拉丁 token：房间名、物体 id、单位。"""
    names = {r['name'].lower() for r in sm.rooms}
    names |= {o['id'].lower() for o in sm.objects}
    names |= {'ok', 'm', 'cm', 's'}
    return names


# 内部工具名：出现在给用户的回答里就是泄漏（用户不该知道这些）
TOOL_NAMES = {'go_to', 'go_to_room', 'look_at', 'find_place', 'list_rooms',
              'where_am_i', 'room', 'object_id'}


def latin_tokens(text: str) -> list:
    return re.findall(r'[A-Za-z_][A-Za-z0-9_]{2,}', text)


def garble_check(text: str, allowed: set) -> list:
    """找出疑似模型退化的拉丁字母碎片（如 'pora'、'sidelineda'）。"""
    skip = allowed | TOOL_NAMES
    return [t for t in latin_tokens(text) if t.lower() not in skip]


def tool_leak_check(text: str) -> list:
    """回答里有没有把内部工具名/参数名当场解释给用户。"""
    return [t for t in latin_tokens(text) if t.lower() in TOOL_NAMES]


# ---------------- 意图与计划（纯代码） ----------------
def classify_intent(instr: str) -> str:
    """祈使句 → ACT，问句 → ASK。判错的代价极不对称，所以宁可从严判 ACT。

    顺序很关键：先判"问位置"，再判"要动手/要看"。
    否则 "冰箱还在吗" 这类既含"问"又要求动作的句子会掉进 ASK，
    于是车不动、只回一句"在厨房"—— 而用户问的是"还在吗"，是要它去看一眼。
    """
    s = instr.strip()
    # 问位置/清点 → ASK（"洗手间在哪"、"家里有哪些房间"）
    if any(k in s for k in ('在哪', '哪里', '有哪些', '什么房间', '有多少')):
        return 'ASK'
    # 要动手开车、要去看一眼、要判断能不能过去 → ACT
    if any(k in s for k in ('带我去', '带我到', '带他去', '去', '前往', '过去',
                            '走到', '导航到', '拿给我', '帮我拿',
                            '看看', '看一下', '看一眼', '瞧瞧',
                            '还在吗', '还在不在', '确认')
            + PASS_KWS):
        return 'ACT'
    return 'ASK'


def plan_action(sm, instr: str):
    """祈使句 → ('object', id, 说明) / ('room', name, 说明) / None。

    房间限定必须先算：'到卧室2的床边' 里的「卧室2」要把物体解析锁到 bedroom_2，
    否则 resolve_object 会在两个房间里挑分高的那个 —— 实测会解析成 bed_1（卧室1的床）。
    """
    r = sm.resolve_room(instr)
    objs = sm.resolve_object(instr, room=r['name'] if r else None)
    if objs:
        o, _score, why = objs[0]
        return ('object', o['id'], f'{sm.class_zh(o["class"])}（{o["id"]}，{why}）')
    if r:
        return ('room', r['name'], f'{r["label_zh"]}（{r["name"]}）')
    return None


def strip_action(s: str) -> str:
    """从指令里剥掉动作词，留下目标短语（'带我去地下室' → '地下室'）。"""
    for w in ('带我去', '带我到', '带他去', '帮我拿', '拿给我', '导航到', '走到',
              '前往', '过去', '看看', '看一下', '瞧瞧', '一下', '去'):
        s = s.replace(w, '')
    return s.strip(' ，。！？!?、的')


# "看"类指令走视觉确认（模式 D）：退到观察站位 + VLM 核对，
# 而不是贴到 0.55 m —— 贴太近时相机只拍到面板，VLM 会把冰箱认成门（实测）。
LOOK_KWS = ('看看', '看一下', '看一眼', '瞧瞧', '还在吗', '还在不在', '确认一下', '确认')

# "能不能过去"类指令走模式 E：雷达扇区量测 + 相机视觉双源核对。
# 这类问题不该开车，也不该由 LLM 凭地图回答（地图不知道此刻有没有东西挡路）。
PASS_KWS = ('能过去', '能过吗', '通不通', '通得过', '挡住', '挡着', '有障碍',
            '堵住', '堵了', '门开着', '前面能', '前面通', '前方能', '前方通')


def wants_look(instr: str) -> bool:
    return any(k in instr for k in LOOK_KWS)


def wants_pass(instr: str) -> bool:
    return any(k in instr for k in PASS_KWS)


def room_list_text(sm) -> str:
    """家里实际有哪些房间（给用户的拒绝回答用，不含走廊）。"""
    return '、'.join(r['label_zh'] for r in sm.rooms if r['name'] != 'corridor')


async def run_instruction(instr: str, model: str, temperature: float = 0.0,
                          reporter_model: str = REPORTER_MODEL,
                          base_url: str = BASE_URL, api_key: str = API_KEY) -> dict:
    """跑一条中文指令，返回结果字典。

    base_url / api_key 可换：任何 OpenAI 兼容端点都能接（本地 ollama、DeepSeek、
    Moonshot、OpenAI 官方…）。换模型不需要改代码，见 main.py 的 --base-url / --api-key。
    """
    link = RosbridgeLink()
    nav = None
    try:
        nav = NavClient(link)
        if not nav.wait_ready(timeout=6.0):
            raise RuntimeError(
                '收不到 /agent/nav_status —— agent_bridge.py 起了吗？'
                '（/usr/bin/python3 agent_bridge.py）')

        sm = SemanticMap()
        fns, log = build_tools(sm, nav)
        by_name = {f.__name__: f for f in fns}

        client = OpenAIChatClient(model=model, api_key=api_key, base_url=base_url)
        agent = Agent(client=client, instructions=INSTRUCTIONS, tools=fns,
                      default_options={'temperature': temperature})
        # 汇报用另一个（更快的）模型，且不绑任何工具
        rclient = OpenAIChatClient(model=reporter_model, api_key=api_key,
                                   base_url=base_url)
        reporter = Agent(client=rclient, instructions=REPORT_INSTRUCTIONS,
                         default_options={'temperature': temperature})

        t0 = time.time()
        intent = classify_intent(instr)
        plan = plan_action(sm, instr) if intent == 'ACT' else None
        action = None
        mode = ''

        # ---- 路线 F：叫停 / 反悔，且句子里没有新目的地 ----
        # 有目的地的新指令走"抢占"（在 main.py 的交互模式里判定），到这里只剩纯叫停。
        # 纯叫停不该调任何模型：一句话、零延迟、零幻觉空间。
        if wants_cancel(instr) and not plan:
            nav.cancel()
            action = '⏹ 已发送取消（/agent/cancel），没有发新的移动目标'
            mode = 'F 取消（确定性，不调 LLM）'
            intent = 'CANCEL'
            text = '好，停下了。'

        # ---- 路线 E：问"能不能过去" → 雷达 + 视觉双源核对（不开车）----
        # 放在 A/D 之前：这类指令既没有导航目标（会被误判成 C 拒绝），
        # 也不该由 LLM 凭地图回答 —— 地图是静态的，此刻有没有东西挡路只有传感器知道。
        elif intent == 'ACT' and wants_pass(instr):
            action = await asyncio.to_thread(by_name['check_path'])
            mode = 'E 可通行性核对（雷达 + 视觉双源）'
            res = await reporter.run(
                f'机器人刚做完一次"前方能否通过"的核对，工具返回如下：\n{action}\n'
                '请用一到两句中文如实汇报：雷达量到多近有障碍、视觉看到什么、结论能不能过。')

            text = getattr(res, 'text', None) or str(res)

        # ---- 路线 A/D：祈使句 + 目标明确 → 代码直接发车 ----
        elif plan:
            kind, key, desc = plan
            if kind == 'object' and wants_look(instr):
                # D：用户要"看一眼"，退到能看清的观察站位再让 VLM 核对
                # to_thread 是抢占的前提：同步调用会把事件循环堵死，
                # 车在路上那几分钟交互模式连输入都读不到。
                action = await asyncio.to_thread(by_name['look_at'], key)
                mode = 'D 视觉确认（观察站位 + VLM 核对）'
                res = await reporter.run(
                    f'机器人刚执行完"去看看"的任务，工具返回如下：\n{action}\n'
                    '请用一到两句中文汇报：到没到、看到了什么、和预期是否一致。')
                text = getattr(res, 'text', None) or str(res)
            else:
                fn = by_name['go_to'] if kind == 'object' else by_name['go_to_room']
                action = await asyncio.to_thread(fn, key)
                mode = 'A 确定性执行（代码发车，LLM 只汇报）'
                res = await reporter.run(
                    f'机器人刚执行完，工具返回如下：\n{action}\n'
                    '请用一句中文如实汇报给用户。')
                text = getattr(res, 'text', None) or str(res)

        # ---- 路线 B：问句 → LLM 用工具查证后回答 ----
        elif intent == 'ASK':
            mode = 'B LLM 主导（查证/消歧）'
            res = await agent.run(instr)
            text = getattr(res, 'text', None) or str(res)

        # ---- 路线 C：祈使句但目标解析不到 → 确定性拒绝，不调 LLM ----
        # 实测走 LLM 时它思考 53.8 s 却返回空正文（用户拿到一片空白）。
        # 而"家里有哪些房间"是我们自己的真值，直接回答既准确又即时。
        else:
            mode = 'C 确定性拒绝（目标不存在，不调 LLM）'
            target = strip_action(instr) or instr
            text = f'家里没有叫「{target}」的地方。现有的房间是：{room_list_text(sm)}。'

        # 兜底：任何路线返回空字符串都不能让用户拿到空白
        if not text or not text.strip():
            text = ('没拿到模型输出。' if not action
                    else '动作已执行，但汇报失败；请查看上面的工具轨迹。')
            mode += '（空回答已兜底）'

        el = time.time() - t0
        return {'text': text, 'log': log, 'elapsed': el, 'sm': sm,
                'intent': intent, 'plan': plan, 'action': action, 'mode': mode}
    finally:
        if nav is not None:
            nav.release()          # 从"在飞"登记表里注销，抢占时不会被误叫停
        link.close()


def print_checks(r: dict) -> None:
    """硬核查：把"看起来成功"和"真的成功"分开。两个入口（agent_nav / main）共用。"""
    went = [c for c in r['log'] if c['tool'] in ('go_to', 'go_to_room', 'look_at')]
    moved = [c for c in went if c['ret'].startswith('✅')]
    if went:
        print(f'核查①：发出 {len(went)} 次移动指令，成功 {len(moved)} 次')
    else:
        print('核查①：本次没有发出移动指令（取消 / 纯问答）')

    bad = garble_check(r['text'], collect_allowed_names(r['sm']))
    leaked = tool_leak_check(r['text'])
    if leaked:
        print(f'核查②：⚠️ 回答里泄漏了内部工具名 {leaked} —— 用户看不懂')
    elif bad:
        print(f'核查②：⚠️ 回答里有疑似退化碎片 {bad} —— 换模型试试 --model qwen3:latest')
    else:
        print('核查②：回答文本干净（无英文碎片、无工具名泄漏）')

    if r.get('intent') == 'CANCEL':
        print('核查③：已发送取消，没有发新的移动目标 ✅')
    elif r.get('intent') == 'ACT':
        plans = r.get('plans') or []
        paths = [c for c in r['log'] if c['tool'] == 'check_path']
        if paths:
            # 可通行性核对：结论是"能不能过"，不是"到没到"
            ret = paths[-1]['ret']
            concl = [ln.strip() for ln in ret.splitlines() if ln.strip().startswith('结论')]
            print('核查③：本次是可通行性核对（未发车）'
                  + (f'　{concl[-1]}' if concl else ''))
            return
        if len(plans) > 1:
            # 复合指令：每段都有自己的结局，看轨迹比给一个总结论更准
            print(f'核查③：复合指令 {len(plans)} 段，逐段结果见上面的轨迹')
        elif moved:
            print('核查③：祈使句已真实执行并到达 ✅')
        elif r.get('plan'):
            print('核查③：❌ 有明确目标却没有成功到达 —— 任务未完成')
        else:
            print('核查③：目标在语义地图里不存在，没发车 —— 正确拒绝 ✅')

    # 视觉确认单独一条：它回答的是"地图不知道的事"（物体还在不在）
    looks = [c for c in r['log'] if c['tool'] == 'look_at']
    if looks:
        ret = looks[-1]['ret']
        if '视觉确认一致' in ret:
            print('核查④：视觉确认与语义地图一致 ✅')
        elif '视觉与语义地图不符' in ret:
            print('核查④：⚠️ 视觉与地图不一致 —— 物体被挪走 / 标注错了 / 相机角度不对')
        elif '视觉确认失败' in ret:
            print('核查④：⚠️ 视觉环节失败（相机或 ollama 不可用）')
        else:
            print('核查④：视觉确认没执行（没到观察站位）')


def main():
    ap = argparse.ArgumentParser(description='中文指令 → 机器人导航（MAF + ollama）')
    ap.add_argument('instruction', nargs='+', help='中文指令，例如：带我去冰箱看看')
    ap.add_argument('--model', default=DEFAULT_MODEL,
                    help=f'ollama 模型名（默认 {DEFAULT_MODEL}）')
    ap.add_argument('--temp', type=float, default=0.0,
                    help='温度，默认 0（导航任务别引入随机性）')
    ap.add_argument('--reporter-model', default=REPORTER_MODEL,
                    help=f'汇报专用模型（默认 {REPORTER_MODEL}，只做改写，要快）')
    ap.add_argument('--base-url', default=os.environ.get('ROBOT_LLM_BASE_URL', BASE_URL),
                    help=f'OpenAI 兼容端点（默认 {BASE_URL}；换 cloud 用这个）')
    ap.add_argument('--api-key', default=os.environ.get('ROBOT_LLM_API_KEY', API_KEY),
                    help='端点密钥（默认取环境变量 ROBOT_LLM_API_KEY，否则 ollama 占位值）')
    a = ap.parse_args()

    instr = ' '.join(a.instruction).strip()

    print('=' * 70)
    print(f'指令: 「{instr}」  （理解/查证 {a.model}，汇报 {a.reporter_model}，'
          f'temperature {a.temp}）')
    print(f'端点: {a.base_url}')
    print('=' * 70)

    try:
        r = asyncio.run(run_instruction(instr, a.model, a.temp, a.reporter_model,
                                        a.base_url, a.api_key))
    except RuntimeError as e:
        print(f'❌ {e}')
        return 2

    print(f'意图判定：{r["intent"]}'
          f'（{"祈使句，必须有移动动作" if r["intent"] == "ACT" else "问句，只回答不开车"}）')
    if r['plan']:
        print(f'执行计划：{r["plan"][2]}')
    print(f'执行模式：{r["mode"]}')

    print()
    print(f'--- 工具调用轨迹（{len(r["log"])} 次）---')
    for i, c in enumerate(r['log'], 1):
        ret = c['ret'].replace('\n', ' ⏎ ')
        print(f"  {i}. {c['tool']}({c['args']})  @{c['t']}")
        print(f'     → {ret[:300]}{" …" if len(ret) > 300 else ""}')

    print()
    print('--- 机器人回答 ---')
    print(r['text'])
    print()
    print(f'总耗时 {r["elapsed"]:.1f}s')

    print_checks(r)
    return 0


if __name__ == '__main__':
    sys.exit(main())
