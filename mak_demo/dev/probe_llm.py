"""摸清两件事：本地 ollama 有哪些模型、MAF 的 OpenAIChatClient 接受哪些参数。

跑法：.venv/bin/python dev/probe_llm.py
"""
import inspect
import json
import urllib.request

print('=' * 70)
print('① 本地 ollama 模型列表')
print('=' * 70)
try:
    with urllib.request.urlopen('http://127.0.0.1:11434/api/tags', timeout=5) as r:
        data = json.loads(r.read())
    for m in data.get('models', []):
        size_gb = m.get('size', 0) / 1e9
        print(f"  {m['name']:28s} {size_gb:6.1f} GB  "
              f"family={m.get('details', {}).get('family', '?')}  "
              f"params={m.get('details', {}).get('parameter_size', '?')}")
except Exception as e:
    print(f'  ❌ 读不到 ollama 模型列表: {e}')

print()
print('=' * 70)
print('② MAF OpenAIChatClient.__init__ 签名')
print('=' * 70)
from agent_framework.openai import OpenAIChatClient   # noqa: E402

sig = inspect.signature(OpenAIChatClient.__init__)
for name, p in sig.parameters.items():
    if name == 'self':
        continue
    print(f'  {name:24s} default={p.default!r}')

print()
print('③ 有没有"默认请求参数"的口子（找 temperature / model_settings 之类）')
for name, p in sig.parameters.items():
    if any(k in name.lower() for k in ('option', 'setting', 'temper', 'param', 'default')):
        print(f'  → {name}: {p}')

print()
print('④ Agent.__init__ 签名')
from agent_framework import Agent   # noqa: E402

for name, p in inspect.signature(Agent.__init__).parameters.items():
    if name == 'self':
        continue
    d = '' if p.default is inspect.Parameter.empty else f' default={p.default!r}'
    print(f'  {name:24s}{d}')
