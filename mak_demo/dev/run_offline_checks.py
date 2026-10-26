#!/usr/bin/python3
"""跑 mak_demo 的离线验收套件，汇总通过/失败。不需要 ROS、不启仿真。

搬到仓库内（原先只是一个 scratch 脚本）之后，DEMO 由 __file__ 推导，
整个项目换目录也不会失效。

跑法：
    mak_demo/.venv/bin/python mak_demo/dev/run_offline_checks.py
    mak_demo/.venv/bin/python mak_demo/dev/run_offline_checks.py --fast
"""
import subprocess
import sys
from pathlib import Path

DEMO = Path(__file__).resolve().parent.parent          # mak_demo/
PY = DEMO / '.venv/bin/python'
FLAGS = ['--fast'] if '--fast' in sys.argv else []

OFFLINE = ['check_checks', 'check_intent', 'check_tools', 'check_refusal',
           'check_planner', 'check_passability', 'check_vlm_guard',
           'check_vision_options', 'check_look_route', 'check_standoffs',
           'check_preempt']

rows = []
for name in OFFLINE:
    f = DEMO / 'dev' / f'{name}.py'
    if not f.exists():
        rows.append((name, 'MISSING', ''))
        continue
    try:
        r = subprocess.run([str(PY), str(f)] + FLAGS, cwd=str(DEMO),
                           capture_output=True, text=True, timeout=180)
        out = (r.stdout + r.stderr).strip().splitlines()
        tail = next((ln for ln in reversed(out) if ln.strip()), '')
        rows.append((name, 'PASS' if r.returncode == 0 else f'FAIL({r.returncode})', tail[:88]))
    except subprocess.TimeoutExpired:
        rows.append((name, 'TIMEOUT', '180 s 未结束（可能需要在线 ROS）'))

w = max(len(r[0]) for r in rows)
print(f'{"检查项":<{w}}  {"结果":<12s} 末行')
print('-' * (w + 100))
for n, st, tail in rows:
    print(f'{n:<{w}}  {st:<12s} {tail}')
bad = [r for r in rows if r[1] != 'PASS']
print(f'\n合计：{len(rows) - len(bad)}/{len(rows)} 通过')
sys.exit(0 if not bad else 1)
