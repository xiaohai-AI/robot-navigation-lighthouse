#!/usr/bin/python3
"""前沿探索的进度条 —— 读 launch 日志 + 检查点地图，不需要连 ROS。

用法：
    # 看一次
    python3 tools/explore_progress.py --prefix dan_house_lighthouse3d_explore2

    # 每 10 秒刷新一次（相当于进度条）
    python3 tools/explore_progress.py --watch --prefix dan_house_lighthouse3d_explore2 \
        --log ~/explore.log

进度口径（三条都独立显示，不合并成一个假数字）：
    · 目标进度   = 已发目标数 / --max-goals（探索撞上限就停，所以这条最准）
    · 覆盖率     = 最新检查点地图的空闲格占比（真实建图进度）
    · 时间       = 墙钟已用 + 按目标数外推的 ETA（仿真时间按 --rtf 折算）
覆盖率复用 tools/map_quality.py 的函数，保证与验收报告是同一份实现。
"""
import argparse
import glob
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MAPS = os.path.join(ROOT, 'config', 'maps')
sys.path.insert(0, HERE)
import map_quality as mq            # noqa: E402  (复用同一份地图统计实现)

TS = re.compile(r'\[(\d{10}\.\d+)\]\s*\[frontier_explore\]')
GOAL_N = re.compile(r'目标 #(\d+)')
END_SIM = re.compile(r'用时\s*([\d.]+)\s*分钟')
END_CNT = re.compile(r'发出目标\s*(\d+)\s*个.*?失败\s*(\d+)\s*个')


def newest_map(prefix):
    """优先取最新的检查点，其次最终图。返回 (path, mtime) 或 (None, 0)。"""
    cands = glob.glob(os.path.join(MAPS, f'{prefix}_checkpoint_*.pgm'))
    final = os.path.join(MAPS, f'{prefix}.pgm')
    if os.path.exists(final):
        cands.append(final)
    if not cands:
        return None, 0.0
    cands.sort(key=os.path.getmtime)
    return cands[-1], os.path.getmtime(cands[-1])


def ckpt_goals(prefix):
    ns = [int(re.search(r'_checkpoint_(\d+)\.pgm$', p).group(1))
          for p in glob.glob(os.path.join(MAPS, f'{prefix}_checkpoint_*.pgm'))
          if re.search(r'_checkpoint_(\d+)\.pgm$', p)]
    return max(ns) if ns else 0


def parse_log(path):
    """→ dict(goals, arrived, failed, t_first, t_last, done, sim_min, sent, failed_end)"""
    info = dict(goals=0, arrived=0, failed=0,
                t_first=0.0, t_last=0.0,
                done=False, end_text='', sim_min=0.0, sent=0, failed_end=0)
    if not path or not os.path.exists(path):
        return info
    try:
        with open(path, 'r', errors='ignore') as f:
            for line in f:
                m = TS.search(line)
                if m:
                    t = float(m.group(1))
                    if info['t_first'] == 0.0:
                        info['t_first'] = t
                    info['t_last'] = t
                g = GOAL_N.search(line)
                if g:
                    info['goals'] = max(int(info['goals']), int(g.group(1)))
                if '✓ 到达目标' in line:
                    info['arrived'] = int(info['arrived']) + 1
                if '✗' in line:
                    info['failed'] = int(info['failed']) + 1
                if '🏁 探索结束' in line:
                    info['done'] = True
                    info['end_text'] = line.split('探索结束：')[-1].strip()
                s = END_SIM.search(line)
                if s:
                    info['sim_min'] = float(s.group(1))
                c = END_CNT.search(line)
                if c:
                    info['sent'] = int(c.group(1))
                    info['failed_end'] = int(c.group(2))
    except OSError:
        pass
    return info


def bar(frac, width=26):
    frac = max(0.0, min(1.0, frac))
    k = int(round(frac * width))
    return '[' + '█' * k + '░' * (width - k) + f'] {frac*100:3.0f}%'


def snapshot(a):
    lg = parse_log(a.log)
    goals = max(int(lg['goals']), ckpt_goals(a.prefix))
    done = bool(lg['done'])
    t_first = float(lg['t_first'])
    t_last = float(lg['t_last'])
    failed = int(lg['failed'])

    mp, mt = newest_map(a.prefix)
    cov_note = ''
    if mp is None and a.fallback_prefix:
        mp, mt = newest_map(a.fallback_prefix)
        if mp is not None:
            cov_note = '（本轮尚未存检查点 → 显示上一轮）'
    cov_name, cov_stats = '', None
    if mp:
        try:
            img, w, h = mq.read_pgm(mp)
            free, occ = mq.occupancy(img)
            cov_name, cov_stats = os.path.basename(mp), mq.size_stats(free, occ, w, h)
        except Exception as exc:                       # 地图正在写 → 读失败不算错
            cov_name = f'{os.path.basename(mp)}（读取中: {exc}）'

    lines = []
    frac = 1.0 if done else goals / max(a.max_goals, 1)
    head = f'{bar(frac)}  目标 {goals}/{a.max_goals}'
    if done:
        head += '   已完成'
    lines.append(head)

    if cov_stats:
        age = (time.time() - mt) / 60.0
        lines.append(f'覆盖率   {cov_stats["free_pct"]:5.1f}% 空闲 · '
                     f'{cov_stats["wall_pct"]:4.1f}% 墙 · '
                     f'{cov_stats["unk_pct"]:5.1f}% 未知     '
                     f'最新图 {cov_name}{cov_note}（{age:.1f} min 前）')
    elif cov_name:
        lines.append(f'覆盖率   地图读取中：{cov_name}')

    if t_first and t_last:
        wall = (t_last - t_first) / 60.0
        per_goal = (t_last - t_first) / max(goals, 1)
        eta = (a.max_goals - goals) * per_goal / 60.0
        lines.append(f'用时     {wall:5.1f} min 墙钟 ≈ {wall*a.rtf:5.1f} min 仿真'
                     f'（RTF 按 {a.rtf:.2f} 折算）   {per_goal:4.1f} s/目标   失败 {failed}')
        if not done:
            sim_left = a.max_minutes - wall * a.rtf
            lines.append(f'预计     还需 ≈ {eta:4.1f} min（按目标数） · '
                         f'仿真时间上限还剩 ≈ {max(sim_left, 0):4.1f} min '
                         f'（上限 {a.max_minutes:.0f} min，谁先到谁停）')
    elif goals:
        lines.append('用时     没有日志时间戳（--log 未给或路径不对），只能看目标数')

    if done:
        lines.append('结束原因  ' + str(lg['end_text'] or '?'))
        sim_min = float(lg['sim_min'])
        if sim_min:
            lines.append(f'最终      用时 {sim_min:.1f} min 仿真 · 发出 {int(lg["sent"])} 个 · '
                         f'失败 {int(lg["failed_end"])} 个')
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--log', default='', help='launch 日志路径（ros2 launch ... | tee 的那个文件）')
    ap.add_argument('--prefix', default='dan_house_lighthouse3d_explore',
                    help='探索存图前缀（与 --map-prefix 一致）')
    ap.add_argument('--fallback-prefix', default='',
                    help='本轮还没存检查点时，用这个前缀的地图显示覆盖率（一般填上一轮）')
    ap.add_argument('--max-goals', type=int, default=120, help='与 explore_max_goals 一致')
    ap.add_argument('--max-minutes', type=float, default=60.0, help='与 explore_max_minutes 一致')
    ap.add_argument('--rtf', type=float, default=0.9, help='实测 RTF，用于墙钟→仿真换算')
    ap.add_argument('--watch', action='store_true', help='循环刷新（进度条模式）')
    ap.add_argument('--interval', type=float, default=10.0, help='刷新间隔秒')
    a = ap.parse_args()

    if not a.watch:
        print(snapshot(a))
        return 0

    try:
        while True:
            sys.stdout.write('\033[2J\033[H')      # 清屏 + 光标回左上
            print(time.strftime('  探索进度  %H:%M:%S\n'))
            print(snapshot(a))
            print('\n  Ctrl-C 退出（不影响探索本身）')
            sys.stdout.flush()
            time.sleep(a.interval)
    except KeyboardInterrupt:
        print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
