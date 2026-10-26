"""对已有的 PNG 帧提问（不连 ROS、不开车）—— 用来对比"开放式"与"封闭式"提问的差别。

为什么需要：同一帧画面，开放式提问（"描述画面里有哪些物体"）会让 8B 模型编造场景
（实测把厨房地砖说成床单、把冰箱说成房门）；封闭式提问（只许输出 是/否 + 20 字依据）
通常能压住。这个脚本就是验证这一点。

跑法：
    .venv/bin/python dev/ask_png.py logs/vlm_211214.png "画面主体是冰箱吗？只答 是/否/看不出"
    .venv/bin/python dev/ask_png.py <png> "<问题>" qwen3-vl:8b
"""
from __future__ import annotations

import base64
import json
import sys
import time
import urllib.request
from pathlib import Path

OLLAMA = 'http://127.0.0.1:11434/api/chat'


def ask(png: bytes, question: str, model: str, num_predict: int = 120):
    body = json.dumps({
        'model': model,
        'stream': False,
        'options': {'temperature': 0, 'num_predict': num_predict},
        'messages': [{'role': 'user', 'content': question,
                      'images': [base64.b64encode(png).decode()]}],
    }).encode()
    req = urllib.request.Request(OLLAMA, data=body,
                                 headers={'Content-Type': 'application/json'})
    t0 = time.time()
    with urllib.request.urlopen(req, timeout=180) as r:
        d = json.loads(r.read())
    return d.get('message', {}).get('content', ''), time.time() - t0


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    p = Path(sys.argv[1])
    if not p.is_absolute():
        p = Path('/home/dan/HelloAgents/hello-agents/code/chapter6/AutoGenDemo/robot/mak_demo') / p
    q = sys.argv[2]
    model = sys.argv[3] if len(sys.argv) > 3 else 'qwen3-vl:8b'
    png = p.read_bytes()

    print('=' * 70)
    print(f'帧：{p.name}  {len(png) / 1024:.1f} KB　模型：{model}')
    print(f'问题：{q}')
    print('=' * 70)
    ans, el = ask(png, q, model)
    print(ans.strip())
    print(f'\n耗时 {el:.1f}s')


if __name__ == '__main__':
    main()
