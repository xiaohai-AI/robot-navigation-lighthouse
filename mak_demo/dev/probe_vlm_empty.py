"""诊断 VLM 空回答：直接调 ollama 的 /api/chat，把完整 JSON 的字段打出来。

怀疑：qwen3-vl 是 thinking 模型，num_predict 太小（80）时 token 全用在思考上，
message.content 为空，message.thinking 里却有内容。

跑法：.venv/bin/python dev/probe_vlm_empty.py
"""
import base64
import json
import sys
import time
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import vision                                    # noqa: E402

OLLAMA = 'http://127.0.0.1:11434/api/chat'
png = HERE / 'logs' / 'look_143601.png'
if not png.exists():
    print(f'❌ 缺少测试帧 {png}，先跑 dev/probe_scan_raw.py')
    sys.exit(1)
b64 = base64.b64encode(png.read_bytes()).decode()
prompt = vision.build_pass_prompt()


def ask(**extra):
    body = {'model': 'qwen3-vl:8b', 'stream': False,
            'options': {'temperature': 0, 'num_predict': extra.pop('num_predict', 80)},
            'messages': [{'role': 'user', 'content': prompt, 'images': [b64]}],
            **extra}
    req = urllib.request.Request(OLLAMA, data=json.dumps(body).encode(),
                                 headers={'Content-Type': 'application/json'})
    t0 = time.time()
    with urllib.request.urlopen(req, timeout=180) as r:
        d = json.loads(r.read())
    el = time.time() - t0
    msg = d.get('message', {})
    return {
        'elapsed': round(el, 1),
        'content': msg.get('content', ''),
        'thinking_len': len(msg.get('thinking') or ''),
        'thinking_head': (msg.get('thinking') or '')[:120],
        'done_reason': d.get('done_reason'),
        'eval_count': d.get('eval_count'),
        'prompt_eval_count': d.get('prompt_eval_count'),
    }


CASES = [
    ('num_predict=80（复现）', {'num_predict': 80}),
    ('num_predict=200', {'num_predict': 200}),
    ('num_predict=80 + think=False', {'num_predict': 80, 'think': False}),
    ('num_predict=200 + think=False', {'num_predict': 200, 'think': False}),
]

for name, kw in CASES:
    try:
        r = ask(**kw)
    except Exception as e:
        print(f'\n【{name}】❌ 异常 {type(e).__name__}: {e}')
        continue
    print(f'\n【{name}】{r["elapsed"]}s')
    print(f'  done_reason={r["done_reason"]}  eval_count={r["eval_count"]}  '
          f'prompt_tokens={r["prompt_eval_count"]}')
    print(f'  thinking 长度 {r["thinking_len"]}：{r["thinking_head"]!r}')
    print(f'  content={r["content"]!r}')
