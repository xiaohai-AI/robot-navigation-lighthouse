"""找出"思考模型把 token 吃光导致空回答"的正解（同一帧、同一个问题）。

候选：
  ① 加大 num_predict（让它想完再答）
  ② /no_think 后缀（原生 /api/chat 才认得的老办法）
  ③ 换不带 thinking 的视觉模型 gemma3:12b

评估：能不能解析出 答案=…；依据=…，以及耗时。

跑法：.venv/bin/python dev/probe_vlm_fix.py
"""
import base64
import json
import re
import sys
import time
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import vision                                    # noqa: E402

OLLAMA = 'http://127.0.0.1:11434/api/chat'
png = (HERE / 'logs' / 'look_143601.png').read_bytes()
b64 = base64.b64encode(png).decode()
BASE = vision.build_pass_prompt()


def ask(model, prompt, num_predict=200, extra=None):
    body = {'model': model, 'stream': False,
            'options': {'temperature': 0, 'num_predict': num_predict},
            'messages': [{'role': 'user', 'content': prompt, 'images': [b64]}]}
    if extra:
        body.update(extra)
    req = urllib.request.Request(OLLAMA, data=json.dumps(body).encode(),
                                 headers={'Content-Type': 'application/json'})
    t0 = time.time()
    try:
        with urllib.request.urlopen(req, timeout=240) as r:
            d = json.loads(r.read())
    except Exception as e:
        return {'err': f'{type(e).__name__}: {e}'}
    el = time.time() - t0
    msg = d.get('message', {})
    content = msg.get('content') or ''
    thinking = msg.get('thinking') or ''
    ans, why = vision.parse_answer(content)
    # 兜底：content 空时，从 thinking 末尾找 "答案="
    if not ans and thinking:
        tail = thinking[-200:]
        mm = re.search(r'答案\s*[=＝]\s*([^\s；;，,]+)', tail)
        if mm:
            ans = f'{mm.group(1)}（从 thinking 末尾捞到）'
    return {'elapsed': round(el, 1), 'ans': ans, 'why': why,
            'content_len': len(content), 'thinking_len': len(thinking),
            'done': d.get('done_reason'), 'eval': d.get('eval_count')}


CASES = [
    ('① num_predict=600', 'qwen3-vl:8b', BASE, 600, None),
    ('② /no_think + 200', 'qwen3-vl:8b', BASE + '\n/no_think', 200, None),
    ('③ gemma3:12b + 200', 'gemma3:12b', BASE, 200, None),
    ('④ gemma3:12b + 80', 'gemma3:12b', BASE, 80, None),
]

for name, model, prompt, npd, extra in CASES:
    r = ask(model, prompt, npd, extra)
    print(f'\n【{name}】')
    if r.get('err'):
        print(f'  ❌ {r["err"]}')
        continue
    print(f'  {r["elapsed"]}s  done={r["done"]}  eval={r["eval"]}  '
          f'content={r["content_len"]}字  thinking={r["thinking_len"]}字')
    print(f'  答案={r["ans"]!r}  依据={r["why"]!r}')
