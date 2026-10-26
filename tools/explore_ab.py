#!/usr/bin/python3
"""探索参数的可重复 A/B/C 实验驱动（3 臂 × N 次，自动串行 + 自动统计）。

为什么要有它：2026-10-25 的实测发现，同一个配置两次 40 目标跑出来能差 9~18 m²，
单次结果不可信（据此下过两个错误结论）。要判断哪个参数真的更好，必须**每臂多次取中位数**。

做的事（每一轮）：
  1. 选一个没被占用的 ROS_DOMAIN_ID / IGN_PARTITION（互不干扰，也不会碰你手动的仿真）
  2. 后台起 eval 探针（订阅 DWB 的 /evaluation，统计"发呆/原地转/行驶"占比）
  3. 起探索（独立地图前缀，绝不覆盖任何现役图）
  4. 轮询日志直到出现"探索结束"或超时，然后优雅收尾（不 pkill 用户进程）
  5. 自己算指标：到达/卡死/仿真用时/内部未知 m²/三类控制占比
  6. 追加一行到 CSV；全部跑完打印中位数汇总表
  7. 可断点续跑：CSV 里已有的 (臂, 第几次) 自动跳过

用法：
    # 完整实验：净空 0.40 / 0.50 / 0.55，各 3 次，每次 40 目标（约 3 小时）
    /usr/bin/python3 tools/explore_ab.py

    # 冒烟测试（先验证管道通不通，约 5 分钟）
    /usr/bin/python3 tools/explore_ab.py --arms 0.40 --reps 1 --goals 4 --minutes 4 --tag smoke

    # 只看已有结果
    /usr/bin/python3 tools/explore_ab.py --report-only
"""
from __future__ import annotations

import argparse
import csv
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
MAPS = ROOT / 'config' / 'maps'
OUT = ROOT / 'experiments'
ROS_SETUP = '/opt/ros/humble/setup.bash'
ROS_PY = '/usr/bin/python3'
PROBE = Path.home() / '.hermes/cache/scratch/probe_eval.py'

# 内部覆盖率口径（与 probe_interior_coverage.py 一致）
X0, X1, Y0, Y1 = -7.9, 7.9, -5.9, 5.9

RE_SIM = re.compile(r'用时 ([\d.]+) 分钟（仿真时间）')
RE_STALL = re.compile(r'判定卡死')
RE_ARRIVE = re.compile(r'✓ 到达目标')
RE_GOAL = re.compile(r'→ 目标 #(\d+)')

FIELDS = ['tag', 'arm', 'rep', 'status', 'goals_issued', 'arrived', 'stalls',
          'sim_min', 'wall_min', 'unknown_m2', 'free_pct', 'wall_pct',
          'stationary_pct', 'rotate_pct', 'moving_pct', 'map', 'log']


# ---------------------------------------------------------------- 地图统计
# PGM 解码与"占据/空闲/未知"的判定统一走 tools/map_quality.py ——
# 技能里记过这个坑：把 unknown 当 free（img > 200）会得出"探索完成"的反结论。
import sys as _sys
_sys.path.insert(0, str(Path(__file__).resolve().parent))
try:
    import map_quality as _mq
except Exception:                                  # 没装 numpy 的老环境兜底
    _mq = None


def interior_stats(name: str) -> tuple[float, float, float]:
    """返回 (内部未知 m², 内部空闲 %, 内部墙 %)

    口径与技能一致：只统计房屋footprint 内（x∈[-7.9,7.9], y∈[-5.9,5.9]），
    外圈永远打不到的区域不计入覆盖损失。
    """
    if _mq is None:
        raise RuntimeError('需要 tools/map_quality.py（同目录）来解码 PGM')
    img, w, h = _mq.read_pgm(MAPS / f'{name}.pgm')
    free, occ = _mq.occupancy(img)
    meta = yaml.safe_load((MAPS / f'{name}.yaml').read_text())
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])

    xs = ox + np.arange(w) * res
    ys = oy + (h - 1 - np.arange(h)) * res          # pgm 第 0 行 = 最大 y
    box = ((ys >= Y0) & (ys <= Y1))[:, None] & ((xs >= X0) & (xs <= X1))[None, :]
    free_in = int((free & box).sum())
    occ_in = int((occ & box).sum())
    unk_in = int((~free & ~occ & box).sum())
    total = max(free_in + occ_in + unk_in, 1)
    return unk_in * res * res, 100.0 * free_in / total, 100.0 * occ_in / total


# ---------------------------------------------------------------- 控制质量
def eval_shares(path: Path) -> tuple[float, float, float]:
    """从 eval jsonl 算 (发呆%, 原地转%, 行驶%)"""
    import json
    n = dict(s=0, r=0, m=0)
    if not path.exists():
        return float('nan'), float('nan'), float('nan')
    for line in path.read_text(errors='ignore').splitlines():
        try:
            rec = json.loads(line)
            vx, wz = abs(rec['best_vx']), abs(rec['best_wz'])
        except Exception:
            continue
        if vx >= 0.02:
            n['m'] += 1
        elif wz >= 0.10:
            n['r'] += 1
        else:
            n['s'] += 1
    tot = (n['s'] + n['r'] + n['m']) or 1
    return 100.0 * n['s'] / tot, 100.0 * n['r'] / tot, 100.0 * n['m'] / tot


# ---------------------------------------------------------------- 进程管理
STOP = False          # Ctrl+C 时置位：本轮收尾（杀子进程、清残留）后再退出


def _on_sigint(sig, frm):
    global STOP
    if not STOP:
        print('\n  收到 Ctrl+C：本轮收尾后退出（再按一次强杀）')
    STOP = True
    signal.signal(signal.SIGINT, signal.SIG_DFL)   # 第二次 Ctrl+C 立刻强退


def domain_in_use(domain: int) -> bool:
    """该 domain 上是否已有 ROS 进程（避免撞车）"""
    try:
        out = subprocess.run(['pgrep', '-af', 'ros2 launch|gazebo|controller_server|slam_toolbox'],
                             capture_output=True, text=True, timeout=10).stdout
    except Exception:
        return False
    for line in out.splitlines():
        try:
            pid = int(line.split()[0])
        except Exception:
            continue
        try:
            env = Path(f'/proc/{pid}/environ').read_bytes().split(b'\0')
        except Exception:
            continue
        for kv in env:
            if kv.startswith(b'ROS_DOMAIN_ID='):
                if kv.split(b'=')[1].strip() == str(domain).encode():
                    return True
    return False


def kill_stragglers(domain: int, partition: str) -> int:
    """只杀本实验自己那套（按 domain + partition 双重确认），不碰用户进程"""
    killed = 0
    try:
        out = subprocess.run(['pgrep', '-af', 'gazebo|controller_server|planner_server|bt_navigator'
                              '|slam_toolbox|frontier_explore|ros2 launch|pointcloud_to_laserscan'],
                             capture_output=True, text=True, timeout=10).stdout
    except Exception:
        return 0
    for line in out.splitlines():
        try:
            pid = int(line.split()[0])
        except Exception:
            continue
        if pid == os.getpid():
            continue
        try:
            env = Path(f'/proc/{pid}/environ').read_bytes().split(b'\0')
        except Exception:
            continue
        dom = part = None
        for kv in env:
            if kv.startswith(b'ROS_DOMAIN_ID='):
                dom = kv.split(b'=')[1].decode(errors='ignore')
            elif kv.startswith(b'IGN_PARTITION='):
                part = kv.split(b'=')[1].decode(errors='ignore')
        if dom == str(domain) and part == partition:
            try:
                os.kill(pid, signal.SIGTERM)
                killed += 1
            except Exception:
                pass
    if killed:
        time.sleep(4)
    return killed


# ---------------------------------------------------------------- 单次运行
def run_one(tag: str, arm: str, rep: int, domain: int, goals: int, minutes: int,
            stall: float) -> dict:
    partition = f'ab{domain}'
    prefix = f'dan_house_lighthouse3d_ab_{tag}_{arm}_{rep}'
    OUT.mkdir(parents=True, exist_ok=True)
    # 日志名必须带 domain：同一个 (tag, 臂, 次数) 的重试会覆盖同名文件，
    # 一次失败尝试的证据（为什么 0 目标）就这样消失了 —— 实测踩过一次。
    log = OUT / f'{tag}_{arm}_{rep}_d{domain}.log'
    eval_jsonl = OUT / f'{tag}_{arm}_{rep}_d{domain}_eval.jsonl'

    if domain_in_use(domain):
        print(f'  ! domain {domain} 已被占用，跳过（换一个 domain 再跑）')
        return dict(tag=tag, arm=arm, rep=rep, status='domain_busy')

    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(domain), IGN_PARTITION=partition,
               GZ_PARTITION=partition, RCUTILS_LOGGING_BUFFERED_STREAM='0')

    launch_cmd = (
        f'source {ROS_SETUP} && exec ros2 launch ./dan_house.launch.py '
        f'lidar:=3d explore:=true headless:=true '
        f'explore_max_goals:={goals} explore_max_minutes:={minutes} '
        f'explore_stall_secs:={stall} explore_goal_clearance:={arm} '
        f'explore_map_prefix:={prefix}'
    )
    t0 = time.time()
    with open(log, 'w') as lf:
        proc = subprocess.Popen(['bash', '-c', launch_cmd], cwd=str(ROOT), env=env,
                                stdout=lf, stderr=subprocess.STDOUT, start_new_session=True)
    # 探针（延后 85 s，等 Nav2 起来；跑满整轮时长 + 余量）
    probe = subprocess.Popen(
        ['bash', '-c',
         f'sleep 85 && source {ROS_SETUP} && exec {ROS_PY} {PROBE} '
         f'--secs {minutes * 60 + 240} --out {eval_jsonl}'],
        cwd=str(ROOT), env=env, stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL, start_new_session=True)

    # 起点前的"开机自检"：Nav2 生命周期在这种满负载启动时偶尔会超时
    # （behavior_server/get_state 超时 → Failed to bring up all requested nodes），
    # 这时规划器完全不可用，跑下去就是 0 目标白烧时间。必须当场发现、当场放弃重跑。
    guard_deadline = time.time() + 210
    ready = False
    while time.time() < guard_deadline:
        time.sleep(5)
        if STOP:
            break
        if proc.poll() is not None:
            break
        txt = log.read_text(errors='ignore')
        if ('Failed to bring up all requested nodes' in txt
                or 'Failed to change state for node' in txt):
            break
        if '前沿探索启动' in txt:
            ready = True
            break

    if not ready:
        for p in (proc, probe):
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
        kill_stragglers(domain, partition)
        if STOP:
            print('  Ctrl+C：本轮已收尾，退出')
            return dict(tag=tag, arm=arm, rep=rep, status='interrupted')
        why = ('nav_bringup_failed'
               if 'Failed to bring up all requested nodes' in log.read_text(errors='ignore')
               else 'no_start_marker')
        print(f'  ! 启动自检失败（{why}），本轮作废，稍后重试')
        return dict(tag=tag, arm=arm, rep=rep, status=why)

    # 轮询：等到"探索结束"或超时（墙钟上限 = 仿真分钟 × 2.2 + 8 分钟余量）
    limit = minutes * 60 * 2.2 + 480
    status, sim_min = 'timeout', float('nan')
    while time.time() - t0 < limit:
        time.sleep(10)
        if STOP:
            status = 'interrupted'
            break
        if proc.poll() is not None:
            status = 'exited'
            break
        txt = log.read_text(errors='ignore')
        m = RE_SIM.search(txt)
        if m:
            sim_min = float(m.group(1))
            status = 'done'
            break

    # 收尾：先礼后兵（SIGTERM 整个会话组），再按 domain+partition 清残留
    for p in (proc, probe):
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        except Exception:
            pass
    time.sleep(6)
    for p in (proc, probe):
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except Exception:
            pass
    killed = kill_stragglers(domain, partition)

    txt = log.read_text(errors='ignore')
    if sim_min != sim_min:
        m = RE_SIM.search(txt)
        sim_min = float(m.group(1)) if m else float('nan')
    arrived = len(RE_ARRIVE.findall(txt))
    stalls = len(RE_STALL.findall(txt))
    issued = max([int(x) for x in RE_GOAL.findall(txt)] or [0])

    # 等地图落盘（探索结束后节点会存图，最多等 30 s）
    pgm = MAPS / f'{prefix}.pgm'
    for _ in range(15):
        if pgm.exists() and time.time() - pgm.stat().st_mtime < 600:
            break
        time.sleep(2)
    if pgm.exists():
        unknown, free_pct, wall_pct = interior_stats(prefix)
    else:
        unknown = free_pct = wall_pct = float('nan')
    st, rt, mv = eval_shares(eval_jsonl)

    row = dict(tag=tag, arm=arm, rep=rep, status=status, goals_issued=issued,
               arrived=arrived, stalls=stalls, sim_min=round(sim_min, 1),
               wall_min=round((time.time() - t0) / 60, 1),
               unknown_m2=round(unknown, 1), free_pct=round(free_pct, 1),
               wall_pct=round(wall_pct, 1), stationary_pct=round(st, 1),
               rotate_pct=round(rt, 1), moving_pct=round(mv, 1),
               map=prefix, log=str(log))
    # 有效性判据：一个目标都没发出去 → 这轮没有信息量，标成 no_goals（不进汇总）
    if issued == 0 and status == 'done':
        row['status'] = 'no_goals'
    print(f'  → {status}：到达 {arrived}/{issued}｜卡死 {stalls}｜仿真 {row["sim_min"]} min'
          f'｜未知 {row["unknown_m2"]} m²｜发呆 {row["stationary_pct"]}%'
          f'｜清理残留 {killed} 个')
    return row


# ---------------------------------------------------------------- CSV / 报告
def load_rows(csv_path: Path) -> list[dict]:
    if not csv_path.exists():
        return []
    with open(csv_path) as f:
        return list(csv.DictReader(f))


def append_row(csv_path: Path, row: dict):
    new = not csv_path.exists()
    with open(csv_path, 'a', newline='') as f:
        wr = csv.DictWriter(f, fieldnames=FIELDS)
        if new:
            wr.writeheader()
        wr.writerow({k: row.get(k, '') for k in FIELDS})


def report(csv_path: Path, tag: str | None = None):
    rows = [r for r in load_rows(csv_path) if not tag or r['tag'] == tag]
    # 只统计有效样本：正常结束且真的发过目标
    rows = [r for r in rows
            if r.get('status') in ('done', 'exited', 'timeout')
            and float(r.get('arrived') or 0) > 0]
    if not rows:
        print('还没有结果')
        return
    arms = sorted({r['arm'] for r in rows})
    print(f'\n{"臂(净空)":<10s} {"样本":>4s} {"未知 m² 中位":>13s} {"到达 中位":>10s} '
          f'{"卡死 中位":>10s} {"仿真 min 中位":>15s} {"发呆% 中位":>12s}')
    for a in arms:
        rs = [r for r in rows if r['arm'] == a]
        med = lambda k: _median([float(r[k]) for r in rs if r[k] not in ('', 'nan')])
        print(f'{a:<10s} {len(rs):4d} {med("unknown_m2"):13.1f} {med("arrived"):10.0f} '
              f'{med("stalls"):10.0f} {med("sim_min"):15.1f} {med("stationary_pct"):12.1f}')
    print('\n（中位数：样本少于 2 个时只是单次结果，别当结论）')


def _median(xs):
    if not xs:
        return float('nan')
    xs = sorted(xs)
    n = len(xs)
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--arms', default='0.40,0.50,0.55', help='净空取值，逗号分隔')
    ap.add_argument('--reps', type=int, default=3, help='每个臂跑几次')
    ap.add_argument('--goals', type=int, default=40)
    ap.add_argument('--minutes', type=int, default=25)
    ap.add_argument('--stall', type=float, default=25.0)
    ap.add_argument('--domain-base', type=int, default=120, help='起始 ROS_DOMAIN_ID')
    ap.add_argument('--tag', default=None, help='实验标签（默认按时间生成）')
    ap.add_argument('--retries', type=int, default=2,
                    help='每轮允许尝试几次（启动自检失败/0 目标会作废重跑）')
    ap.add_argument('--report-only', action='store_true')
    a = ap.parse_args()

    csv_path = OUT / 'ab_explore.csv'
    tag = a.tag or datetime.now().strftime('%m%d_%H%M')
    signal.signal(signal.SIGINT, _on_sigint)
    # 行缓冲：接了 `| tee` 之后 stdout 不是终端，print 默认块缓冲（4~8 KB 才吐一次），
    # 表现就是"敲了命令没有任何反应"。三小时的实验必须能实时看到进度。
    # getattr 而不是直连 .reconfigure：sys.stdout 被重定向成普通 TextIO 时没有这个方法。
    _reconf = getattr(sys.stdout, 'reconfigure', None)
    if _reconf is not None:
        try:
            _reconf(line_buffering=True)
        except Exception:
            pass

    if a.report_only:
        report(csv_path, a.tag if a.tag else None)
        return

    arms = [x.strip() for x in a.arms.split(',') if x.strip()]
    # 断点续跑只认"有效样本"：失败尝试也写在 CSV 里（审计用），
    # 但它不能把这一轮标记成已完成。
    done = {(r['tag'], r['arm'], r['rep']) for r in load_rows(csv_path)
            if r.get('status') == 'done' and float(r.get('arrived') or 0) > 0}
    print(f'实验 {tag}：净空 {arms} × {a.reps} 次，每次 {a.goals} 目标 / {a.minutes} 分钟')
    print(f'结果写入 {csv_path}\n')

    idx = 0
    ok = ('done', 'exited', 'timeout')
    for arm in arms:
        for rep in range(1, a.reps + 1):
            if (tag, arm, str(rep)) in done:
                print(f'[{arm} #{rep}] 已有结果，跳过')
                continue
            for attempt in range(1, a.retries + 1):
                domain = a.domain_base + idx
                idx += 1
                print(f'[{arm} #{rep}] domain {domain} 第 {attempt}/{a.retries} 次尝试 '
                      f'({datetime.now().strftime("%H:%M:%S")})')
                row = run_one(tag, arm, rep, domain, a.goals, a.minutes, a.stall)
                st = row.get('status')
                if st == 'interrupted':
                    print('\n已中断。已完成的结果都在 CSV 里，重跑同一条命令会接着来。')
                    report(csv_path, tag)
                    return 0
                if st in ok:
                    append_row(csv_path, row)
                    break
                # 失败尝试也留一行（status 非 done），复盘时能看到这类浪费有多少；
                # 汇总统计与断点续跑都只认 status=done 的行，不会被它污染。
                append_row(csv_path, row)
                if attempt < a.retries:
                    print(f'  第 {attempt} 次无效（{st}），换 domain 重试')
                    time.sleep(10)
                else:
                    print(f'  {a.retries} 次都无效，记录下来留待复盘')
                # 轮间冷却，避免上一个仿真的 gz 还没退干净就起下一个
            time.sleep(8)

    report(csv_path, tag)


if __name__ == '__main__':
    sys.exit(main())
