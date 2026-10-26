"""离线验收 print_checks：四种结局各打一遍，确认核查行说得准。

跑法：.venv/bin/python dev/check_checks.py
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

from agent_nav import print_checks          # noqa: E402
from semantic_map import SemanticMap        # noqa: E402

sm = SemanticMap()

CASES = [
    ('模式 D 视觉一致', {
        'intent': 'ACT', 'plan': ('object', 'fridge_1', '冰箱'),
        'text': '已经到了冰箱旁边，看到白色双开门带长把手，和地图上标注的一致。',
        'log': [{'tool': 'look_at', 'args': {}, 't': '22:57:00',
                 'ret': '✅ 已到 冰箱（fridge_1）的观察站位，距中心 1.79 m，耗时 54.7s；'
                        '视觉确认一致 ✅（VLM 认为主体是「冰箱」），依据：白色双开门带长把手；'
                        'VLM 11.7s，帧 logs/look_225700.png'}],
    }),
    ('视觉与地图不符', {
        'intent': 'ACT', 'plan': ('object', 'fridge_1', '冰箱'),
        'text': '我看到的东西和地图上标的不是一个。',
        'log': [{'tool': 'look_at', 'args': {}, 't': '22:58:00',
                 'ret': '✅ 已到 冰箱（fridge_1）的观察站位，距中心 1.79 m，耗时 54.7s；'
                        '⚠️ 视觉与语义地图不符：地图说是「冰箱」，VLM 说是「门」；'
                        '依据：竖直把手；VLM 12.0s，帧 logs/x.png'}],
    }),
    ('目标不存在（C 模式拒绝）', {
        'intent': 'ACT', 'plan': None,
        'text': '家里没有叫「地下室」的地方。现有的房间是：客厅、厨房、书房。',
        'log': [],
    }),
    ('复合指令两段', {
        'intent': 'ACT', 'plan': None,
        'plans': [('先去厨房', ('room', 'kitchen', '厨房')),
                  ('到沙发旁边', ('object', 'sofa_1', '沙发'))],
        'text': '厨房到了，也到了沙发旁边。',
        'log': [{'tool': 'go_to_room', 'args': {}, 't': '1', 'ret': '✅ 去 厨房（kitchen）中心'},
                {'tool': 'go_to', 'args': {}, 't': '2', 'ret': '✅ 去 沙发（sofa_1）'}],
    }),
    ('工具名泄漏', {
        'intent': 'ASK', 'plan': None,
        'text': '如需前往请调用 go_to_room(room="bathroom")',
        'log': [],
    }),
    ('可通行性核对（未发车）', {
        'intent': 'ACT', 'plan': None,
        'text': '雷达量到正前方一米二才有东西，画面里也没看到障碍，可以过去。',
        'log': [{'tool': 'check_path', 'args': {}, 't': '23:40',
                 'ret': '可通行性核对（正前方）\n'
                        '  雷达：正前方 ±25° 内最近障碍 1.42 m（方位 -3°），'
                        '车还能前进 1.21 m → 判定：能走一段，但前方不是畅通通道\n'
                        '  视觉：答案「畅通」，依据「画面是地面和墙面」，2.3s\n'
                        '  结论：⚠️ 能走一小段，但前方不是通畅通道 —— 雷达量到还能前进 1.21 m 就会碰到东西'}],
    }),
]

for name, r in CASES:
    r['sm'] = sm
    print('=' * 66)
    print(f'【{name}】')
    print_checks(r)
