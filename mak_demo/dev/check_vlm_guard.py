"""离线验收 VLM 空回答的防护（不连 ollama：猴子补丁替换 _post）。

三个场景：
  ① 第一次被截断（done=length + content 空）→ 自动翻倍重试 → 第二次拿到答案
  ② 两次都被截断 → 从 thinking 末尾把"答案="捞回来
  ③ 正常返回 → 不该触发任何重试

跑法：.venv/bin/python dev/check_vlm_guard.py
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import vision                                  # noqa: E402

PNG = b'\x89PNGfake'
calls = []


def fake_post(png, prompt, model, num_predict):
    calls.append(num_predict)
    scenario = fake_post.scenario
    if scenario == 'retry':
        if len(calls) == 1:
            return {'content': '', 'thinking': '想了很多但没说完', 'done_reason': 'length',
                    'eval_count': num_predict, 'elapsed': 0.1}
        return {'content': '答案=畅通；依据=前方空旷', 'thinking': '',
                'done_reason': 'stop', 'eval_count': 20, 'elapsed': 0.2}
    if scenario == 'salvage':
        return {'content': '', 'thinking': '反复纠结……最后我决定 答案=被挡住；依据=有柱子',
                'done_reason': 'length', 'eval_count': num_predict, 'elapsed': 0.1}
    return {'content': '答案=畅通；依据=前方空旷', 'thinking': '',
            'done_reason': 'stop', 'eval_count': 20, 'elapsed': 0.3}


vision._post = fake_post

ok = 0
print('=' * 70)
for scenario, want_ans, want_calls in (('normal', '畅通', 1),
                                       ('retry', '畅通', 2),
                                       ('salvage', '被挡住', 2)):
    fake_post.scenario = scenario
    calls.clear()
    r = vision.ask_vlm_detailed(PNG, 'x')
    ans, why = vision.parse_answer(r['content'])
    good = (ans == want_ans and len(calls) == want_calls)
    ok += good
    print(f'{"✅" if good else "❌"} {scenario:8s} 答案={ans!r} 调用次数={len(calls)} '
          f'(num_predict={calls}) salvaged={r.get("salvaged", False)} '
          f'truncated={r.get("truncated")}')
    if not good:
        print(f'     期望 答案={want_ans!r}，调用 {want_calls} 次')

print('=' * 70)
print(f'{ok}/3 通过')
sys.exit(0 if ok == 3 else 1)
