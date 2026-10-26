"""离线验收"看看 X"的路由判定（不连 ROS、不开车、不调 LLM）。

要证明的是三件事：
    ① "看看冰箱" 会被判成视觉确认（模式 D），而不是普通 go_to（模式 A）
    ② "去冰箱旁边" 仍然走 A（不该被误伤）
    ③ 目标解析不到时，"看看地下室" 仍走 C 拒绝，不发车

跑法：.venv/bin/python dev/check_look_route.py
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

from agent_nav import classify_intent, plan_action, wants_look      # noqa: E402
from semantic_map import SemanticMap                                # noqa: E402

CASES = [
    # (指令, 期望意图, 期望走视觉?, 期望解析出的目标关键词)
    ('看看冰箱',          'ACT', True,  'fridge'),
    ('冰箱还在吗',        'ACT', True,  'fridge'),
    ('带我去冰箱看看',    'ACT', True,  'fridge'),
    ('确认一下沙发',      'ACT', True,  'sofa'),
    ('去冰箱旁边',        'ACT', False, 'fridge'),
    ('带我到沙发旁边',    'ACT', False, 'sofa'),
    ('我要看看卧室2的床', 'ACT', True,  'bed_2'),
    ('洗手间在哪',        'ASK', False, None),
    ('看看地下室',        'ACT', True,  None),      # 目标不存在 → 走 C
]


def main():
    sm = SemanticMap()
    ok = 0
    print('=' * 74)
    for instr, want_intent, want_look, want_key in CASES:
        intent = classify_intent(instr)
        plan = plan_action(sm, instr) if intent == 'ACT' else None
        look = wants_look(instr) if plan else False
        key = plan[1] if plan else None

        route = ('D 视觉确认' if (plan and look) else
                 'A 普通发车' if plan else 'C 拒绝')
        checks = [intent == want_intent]
        if want_key is None:
            checks.append(plan is None)
        else:
            checks.append(key is not None and want_key in key)
        checks.append(look == want_look if plan else True)

        passed = all(checks)
        ok += passed
        print(f'{"✅" if passed else "❌"} {instr:20s} 意图 {intent:3s} '
              f'→ {route:10s} 目标 {key}')
        if not passed:
            print(f'     期望：意图 {want_intent}，视觉 {want_look}，目标含 {want_key}')

    print('=' * 74)
    print(f'{ok}/{len(CASES)} 通过')
    return 0 if ok == len(CASES) else 1


if __name__ == '__main__':
    sys.exit(main())
