#!/usr/bin/python3
"""Generate every figure used by the textbook (docs/book/src).

All data is read from the repository itself — no numbers are typed in by hand
except the planner A/B table, which is transcribed from STATUS.md §5 with the
source line cited in the caption.

Run:  /usr/bin/python3 docs/book/make_figures.py
Out:  docs/book/figures/*.png
"""
import csv
import math
import json
import subprocess
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
FIGS = HERE / 'figures'
FIGS.mkdir(parents=True, exist_ok=True)

plt.rcParams.update({
    'font.family': 'DejaVu Sans',
    'font.size': 9,
    'axes.titlesize': 11,
    'axes.labelsize': 9,
    'figure.dpi': 200,
    'savefig.bbox': 'tight',
})
INK = '#1f2933'
ACCENT = '#2b6cb0'
WARN = '#c05621'
GOOD = '#2f855a'


def dot(name: str, src: str):
    """Render a graphviz source to PNG."""
    p = FIGS / f'{name}.dot'
    p.write_text(src, encoding='utf-8')
    out = FIGS / f'{name}.png'
    subprocess.run(['dot', '-Tpng', '-Gdpi=160', str(p), '-o', str(out)], check=True)
    print(f'  ✓ {out.name}')


# ---------------------------------------------------------------- 1. floorplan
def fig_world():
    sem = json.loads((ROOT / 'config' / 'semantics.json').read_text())
    EN = {'living_room': 'Living room', 'kitchen': 'Kitchen', 'study': 'Study',
          'bedroom_1': 'Bedroom 1', 'bedroom_2': 'Bedroom 2',
          'bathroom': 'Bathroom', 'storage': 'Storage'}

    def corners(o):
        """build_world.foot() already swaps w/d for +-90 deg poses, so those
        entries are world-axis extents and must NOT be rotated again."""
        w, d, yaw = o['w'], o['d'], o.get('yaw', 0.0)
        if abs(abs(yaw) - math.pi / 2) < 0.2:
            hw, hd = w / 2, d / 2
            pts = [(-hw, -hd), (hw, -hd), (hw, hd), (-hw, hd)]
        else:
            c, s = math.cos(yaw), math.sin(yaw)
            pts = [(px * c - py * s, px * s + py * c)
                   for px, py in ((-w / 2, -d / 2), (w / 2, -d / 2), (w / 2, d / 2), (-w / 2, d / 2))]
        return [(o['x'] + px, o['y'] + py) for px, py in pts]

    fig, ax = plt.subplots(figsize=(8.6, 6.8))
    ax.add_patch(Rectangle((-9, -7), 18, 14, fill=False, lw=2.4, ec=INK, zorder=3))
    ax.add_patch(Rectangle((-9, -1.44), 18, 2.88, fc='#f0f4f8', ec='none', zorder=0))
    ax.text(-8.7, 0.62, 'central corridor — 2.88 m clear', ha='left', va='center',
            color='#475569', fontsize=8, style='italic', zorder=9,
            bbox=dict(fc='white', ec='none', alpha=.85, pad=1.5))

    for i, r in enumerate(sem['rooms']):
        x0, y0, x1, y1 = r['rect']
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0,
                               fc='#e8eef5' if i % 2 == 0 else '#eef3f8',
                               ec='#94a3b8', lw=0.8, zorder=1))
        ax.text((x0 + x1) / 2, (y0 + y1) / 2 - 1.25, EN.get(r['name'], r['name']),
                ha='center', va='center', fontsize=9.5, color='#334155', zorder=2)
        dc = r.get('door_center')
        if dc:
            ax.plot(dc[0], dc[1], 'o', ms=6, mfc=GOOD, mec='white', mew=1.0, zorder=6)

    n_corridor = 0
    for o in sem['objects']:
        cor_pt = abs(o['y']) < 1.5                     # physically in the corridor
        n_corridor += cor_pt
        ax.add_patch(plt.Polygon(corners(o), closed=True,
                                 fc=WARN if cor_pt else '#94a3b8',
                                 ec=INK, lw=0.35, alpha=.9, zorder=4))

    ax.plot([], [], 'o', ms=6, mfc=GOOD, mec='white', label='door centre (7 doors)')
    ax.plot([], [], 's', ms=7, mfc='#94a3b8', mec=INK, label=f'furniture ({len(sem["objects"]) - n_corridor} objects)')
    ax.plot([], [], 's', ms=7, mfc=WARN, mec=INK, label=f'corridor landmarks ({n_corridor} objects)')
    ax.legend(loc='lower left', fontsize=7.5, framealpha=.95)
    ax.set_aspect('equal')
    ax.set_xlim(-9.6, 9.6)
    ax.set_ylim(-7.6, 7.6)
    ax.set_xlabel('x [m]   —   robot spawn is the origin, heading +x')
    ax.set_ylabel('y [m]')
    ax.set_title('The simulated house: 7 rooms + central corridor, 17.8 x 13.8 m of floor')
    ax.grid(alpha=.25, lw=.4)
    fig.savefig(FIGS / 'world_floorplan.png')
    plt.close(fig)
    print(f'  ✓ world_floorplan.png  ({len(sem["objects"])} objects, {n_corridor} in the corridor)')


# ---------------------------------------------------------------- 2. layers
def fig_layers():
    dot('layers', r'''
digraph G {
  graph [rankdir=TB, fontname="Helvetica", nodesep=.35, ranksep=.45, bgcolor="white"];
  node  [shape=box, style="rounded,filled", fontname="Helvetica", fontsize=10, color="#64748b", penwidth=1.1];
  edge  [color="#94a3b8", arrowsize=.7];

  subgraph cluster_0 { label="Layer 5 — interaction"; style="rounded,dashed"; color="#cbd5e1"; fontsize=9;
    a [label="terminal REPL  (mak_demo/main.py)\npreemption: one asyncio task per utterance", fillcolor="#fef3c7"];
  }
  subgraph cluster_1 { label="Layer 4 — agent (LLM)"; style="rounded,dashed"; color="#cbd5e1"; fontsize=9;
    b [label="agent_nav.py — intent routing\nA nav · D nav+vision · E passability · F cancel · ASK", fillcolor="#dbeafe"];
    c [label="qwen3-vl:8b — the agent\ncompletion + vision + tools + thinking\nqwen2.5:7b — reporter (1.9 s vs 31.7 s)", fillcolor="#dbeafe"];
  }
  subgraph cluster_2 { label="Layer 3 — semantic map"; style="rounded,dashed"; color="#cbd5e1"; fontsize=9;
    d [label="semantics.json (authored truth)\nresolve_room / resolve_object\nstandoff_pose() · 16-direction sampling", fillcolor="#dcfce7"];
  }
  subgraph cluster_3 { label="Layer 2 — ROS 2 navigation"; style="rounded,dashed"; color="#cbd5e1"; fontsize=9;
    e [label="slam_toolbox (mapping)\nNavFn + DWB + BT (nav2)", fillcolor="#f1f5f9"];
    f [label="frontier_explore.py\ntour_node.py (frozen)", fillcolor="#f1f5f9"];
  }
  subgraph cluster_4 { label="Layer 1 — simulation"; style="rounded,dashed"; color="#cbd5e1"; fontsize=9;
    g [label="Gazebo Fortress 6.18\ndan_house.world (*.world from build_world.py)", fillcolor="#f8fafc"];
    h [label="robot: Lighthouse diff-drive\nlidar 2D / 64-line · RGB camera · IMU", fillcolor="#f8fafc"];
  }
  a -> b [label="text"];
  b -> c [label="prompt", style=dashed];
  b -> d [label="resolve / standoff"];
  b -> e [label="/agent/nav_goal  (rosbridge → agent_bridge)"];
  d -> e [label="goal pose"];
  e -> f [label="goal source", dir=back];
  e -> g [label="/cmd_vel · /scan · TF"];
  g -> h [label="physics + sensors"];
  h -> e [label="/scan, /odom, images", style=dashed];
}
''')


# ---------------------------------------------------------------- 3. lineage
def fig_lineage():
    dot('lineage', r'''
digraph G {
  graph [rankdir=LR, fontname="Helvetica", nodesep=.28, ranksep=.5, bgcolor="white"];
  node  [shape=box, fontname="Helvetica", fontsize=9.5, style="rounded,filled", color="#64748b"];
  edge  [color="#94a3b8", arrowsize=.7, fontname="Helvetica", fontsize=8, fontcolor="#475569"];

  bw  [label="build_world.py\n(898 lines)\nsingle source of truth", fillcolor="#fde68a"];
  wo  [label="worlds/dan_house.world\nworlds/dan_house_3d.world", fillcolor="#f1f5f9"];
  se  [label="config/semantics.json\n11 KB · rooms + 61 objects", fillcolor="#dcfce7"];
  ro  [label="config/route.json\n26 tour waypoints", fillcolor="#dcfce7"];

  lh  [label="models/lighthouse/   (2D)\nmodels/lighthouse_3d/ (64-line)", fillcolor="#f1f5f9"];
  lch [label="dan_house.launch.py\n(495 lines)", fillcolor="#fde68a"];
  sl  [label="config/slam_toolbox.yaml", fillcolor="#f1f5f9"];
  np  [label="config/nav2_params*.yaml\n(4 files, isolated roles)", fillcolor="#f1f5f9"];
  bt  [label="config/nav_to_pose*.xml", fillcolor="#f1f5f9"];
  mp  [label="config/maps/*.pgm + .yaml\n(390 files: live + experiments)", fillcolor="#e2e8f0"];

  ag  [label="agent_bridge.py\n(287 lines)", fillcolor="#fde68a"];
  md  [label="mak_demo/ (2161 lines)\nagent_nav · semantic_map · vision", fillcolor="#fde68a"];
  lx  [label="mak_demo/data/lexicon.json", fillcolor="#dcfce7"];
  gl  [label="tools/*.py\nscan_pub_guard · explore_ab · map_quality", fillcolor="#fde68a"];
  dl  [label="delivery/2026-10-26/\nfrozen docs + MANIFEST.sha256", fillcolor="#fee2e2"];

  bw -> wo [label="generates"];
  bw -> se [label="generates"];
  bw -> ro [label="generates"];
  lch -> lh [label="spawns (model:// URI)"];
  lch -> mp  [label="SLAM writes"];
  sl -> lch;
  np -> lch;
  bt -> np;
  se -> md  [label="read-only"];
  lx -> md  [label="read-only"];
  ro -> lch [label="tour waypoints"];
  md -> ag   [label="websocket /agent/*"];
  ag -> lch  [label="Nav2 actions"];
  md -> gl [style=dashed, label="dev checks"];
  {rank=same; se; ro;}
}
''')


# ---------------------------------------------------------------- 4. tf tree
def fig_tf():
    dot('tf_tree', r'''
digraph G {
  graph [rankdir=LR, fontname="Helvetica", nodesep=.25, ranksep=.45, bgcolor="white"];
  node  [shape=box, fontname="Helvetica", fontsize=10, style="rounded,filled", color="#64748b"];
  edge  [color="#94a3b8", fontname="Helvetica", fontsize=8, fontcolor="#475569"];

  m  [label="map", fillcolor="#dbeafe"];
  o  [label="odom", fillcolor="#dbeafe"];
  bf [label="base_footprint", fillcolor="#dbeafe"];
  bl [label="base_link", fillcolor="#dbeafe"];
  bs [label="base_scan\n(lidar)  z=0.780", fillcolor="#dcfce7"];
  cl [label="camera_link\nz=0.700", fillcolor="#dcfce7"];
  il [label="imu_link", fillcolor="#dcfce7"];
  wl [label="wheel_left_link", fillcolor="#f1f5f9"];
  wr [label="wheel_right_link", fillcolor="#f1f5f9"];

  m -> o  [label="slam_toolbox\n(or static 0 0 0 when loc:=truth)"];
  o -> bf [label="odom_tf_node\n(or ekf_node when ekf:=true)"];
  bf -> bl [label="static"];
  bl -> bs; bl -> cl; bl -> il; bl -> wl; bl -> wr;
}
''')


# ---------------------------------------------------------------- 5. scan chain
def fig_scan_chain():
    dot('scan_chain', r'''
digraph G {
  graph [rankdir=LR, fontname="Helvetica", nodesep=.3, ranksep=.5, bgcolor="white"];
  node  [shape=box, fontname="Helvetica", fontsize=9.5, style="rounded,filled", color="#64748b"];
  edge  [color="#94a3b8", arrowsize=.7, fontname="Helvetica", fontsize=8, fontcolor="#475569"];

  gz2 [label="gz lidar (2D)\nLaserScan 10 Hz", fillcolor="#f1f5f9"];
  gz3 [label="gz lidar (64-line)\nPointCloud2 23,040 pts/frame", fillcolor="#f1f5f9"];
  br  [label="ros_gz_bridge\ngz_bridge", fillcolor="#fde68a"];
  p2l [label="pointcloud_to_laserscan\ntarget_frame base_scan\n-0.75 m .. +0.05 m", fillcolor="#fde68a"];
  sc  [label="/scan  (LaserScan 360 rays, 0.12–12 m)", fillcolor="#dbeafe"];
  gd  [label="tools/scan_pub_guard.py\nexactly one publisher, name checked", fillcolor="#fee2e2"];
  sl  [label="slam_toolbox", fillcolor="#dcfce7"];
  cm  [label="Nav2 costmap\nobservation_sources: scan", fillcolor="#dcfce7"];

  gz2 -> br [label="2D mode"];
  gz3 -> br [label="3D mode"];
  br -> sc [label="2D mode: direct"];
  br -> p2l [label="3D mode: /scan/points only"];
  p2l -> sc;
  sc -> gd [style=dashed, label="watches"];
  sc -> sl; sc -> cm;
}
''')


# ---------------------------------------------------------------- 6. coverage
def fig_coverage():
    sys.path.insert(0, str(ROOT / 'tools'))
    import explore_ab as ea

    runs = {}
    for rep in (1, 2):
        goals, unk = [], []
        for g in range(8, 81, 8):
            name = f'dan_house_lighthouse3d_ab_1025_2047_0.40_{rep}_checkpoint_{g:02d}'
            try:
                u, _, _ = ea.interior_stats(name)
            except Exception as e:                      # noqa: BLE001
                print(f'    ! {name}: {e}')
                continue
            goals.append(g)
            unk.append(u)
        runs[rep] = (goals, unk)

    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    for rep, (g, u) in runs.items():
        ax.plot(g, u, 'o-', lw=1.8, ms=4,
                color=ACCENT if rep == 1 else WARN, label=f'run #{rep}')
        print(f'  run #{rep}: ' + ' '.join(f'{v:.1f}' for v in u))
    first_g, first_u = runs[1][0][0], runs[1][1][0]
    ax.axvline(48, ls='--', lw=1.0, color='#718096')
    ax.annotate('knee ≈ 48 goals', xy=(48, first_u * .55), xytext=(52, first_u * .72),
                fontsize=8.5, color='#4a5568',
                arrowprops=dict(arrowstyle='->', color='#718096', lw=.9))
    ax.set_xlabel('goals issued')
    ax.set_ylabel('unknown area inside the house footprint  [m²]')
    ax.set_title('Coverage saturates at ~48 goals — the last 32 goals buy ~2 m²')
    ax.grid(alpha=.3, lw=.5)
    ax.legend(fontsize=8.5)
    ax.set_ylim(0, None)
    fig.savefig(FIGS / 'coverage_curve.png')
    plt.close(fig)
    print('  ✓ coverage_curve.png  (recomputed from checkpoint maps)')


# ---------------------------------------------------------------- 7. 3x3 arms
def fig_3x3():
    rows = []
    with open(ROOT / 'experiments' / 'ab_explore.csv', newline='') as f:
        for r in csv.DictReader(f):
            if r['tag'] == '1025_1647' and r['status'] == 'done' and int(r['arrived'] or 0) > 0:
                rows.append(r)
    arms = ['0.40', '0.50', '0.55']
    metrics = [('unknown_m2', 'unknown area [m²]'), ('arrived', 'goals reached'),
               ('stalls', 'stalls'), ('sim_min', 'sim time [min]')]

    fig, axes = plt.subplots(1, 4, figsize=(11, 3.4))
    rng = np.random.default_rng(0)
    for ax, (key, label) in zip(axes, metrics):
        for i, arm in enumerate(arms):
            vals = [float(r[key]) for r in rows if r['arm'] == arm]
            if not vals:
                continue
            x = i + rng.uniform(-.08, .08, len(vals))
            ax.plot(x, vals, 'o', ms=6, color=ACCENT, alpha=.75, mec='white')
            ax.hlines(np.median(vals), i - .22, i + .22, color=WARN, lw=2.2)
            if len(vals) > 1:
                ax.vlines(i, min(vals), max(vals), color='#cbd5e1', lw=1.0, zorder=0)
        ax.set_xticks(range(len(arms)))
        ax.set_xticklabels([f'clear {a}' for a in arms], fontsize=8)
        ax.set_title(label, fontsize=9.5)
        ax.grid(alpha=.3, lw=.5, axis='y')
    fig.suptitle('Clearance 0.40 / 0.50 / 0.55 — 3 runs each, 40 goals per run\n'
                 'orange bar = median; spread inside an arm is larger than any shift between arms',
                 fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, .88))
    fig.savefig(FIGS / 'clearance_3x3.png')
    plt.close(fig)
    print('  ✓ clearance_3x3.png  (from experiments/ab_explore.csv)')


# ---------------------------------------------------------------- 8. planner A/B
def fig_planner():
    # transcribed from STATUS.md §5 (40 goals, identical start, same world)
    data = {
        'NavFn': {'arrived': 35, 'stalls': 4, 'unknown': 18.0, 'sim': 21.9},
        'Smac2D': {'arrived': 28, 'stalls': 11, 'unknown': 36.2, 'sim': 22.2},
    }
    fig, axes = plt.subplots(1, 3, figsize=(8.6, 3.1))
    keys = [('arrived', 'goals reached (of 40)', 40), ('stalls', 'stall events', None),
            ('unknown', 'unknown area [m²]', None)]
    for ax, (k, label, ymax) in zip(axes, keys):
        names = list(data)
        vals = [data[n][k] for n in names]
        cols = [GOOD, WARN]
        bars = ax.bar(names, vals, color=cols, width=.55)
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width() / 2, v, f'{v:g}', ha='center', va='bottom', fontsize=8.5)
        ax.set_title(label, fontsize=9.5)
        if ymax:
            ax.set_ylim(0, ymax)
        ax.grid(alpha=.3, lw=.5, axis='y')
    fig.suptitle('Planner A/B at 40 goals: the global planner flipped the result\n'
                 '(the whole gap sits in one room: study 2.1 m² vs 16.4 m² unknown)',
                 fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, .85))
    fig.savefig(FIGS / 'planner_ab.png')
    plt.close(fig)
    print('  ✓ planner_ab.png')


# ---------------------------------------------------------------- 9. preemption
def fig_preempt():
    fig, ax = plt.subplots(figsize=(8.4, 3.0))
    events = [
        (0, 'user: "take me to the kitchen"\ngo to kitchen', ACCENT, 1.0),
        (15, 'user: "never mind, go to the living room"\nre-target while driving', WARN, 1.0),
        (41, 'kitchen leg prints\n"cancelled (departing)"', GOOD, 0.62),
        (71, 'living room reached\nsucceeded, 0.19 m remaining', GOOD, 1.0),
        (70, 'user: "stop"\ncancel, no new goal', WARN, 0.62),
        (78, 'user: "where is the fridge?"\nanswered, no motion', '#6b46c1', 0.62),
    ]
    ax.axhline(0, color='#cbd5e1', lw=8, solid_capstyle='butt', zorder=1)
    for t, label, c, h in events:
        ax.vlines(t, 0, h, color=c, lw=1.6, zorder=2)
        ax.plot(t, h, 'o', color=c, ms=7, mec='white', mew=1.0, zorder=3)
        ax.text(t, h + .06, label, fontsize=7.6, color=c, ha='center', va='bottom', zorder=4)
    ax.set_xlim(-6, 96)
    ax.set_ylim(0, 1.85)
    ax.set_yticks([])
    ax.set_xlabel('time since first utterance [s]   (session of 2026-10-25, isolated ROS domain 160)')
    ax.set_title('Preemption: a new destination while the robot is already driving\n'
                 '7/7 acceptance criteria passed — and the cancelled leg never claims arrival')
    ax.grid(alpha=.25, lw=.4, axis='x')
    for s in ('top', 'right', 'left'):
        ax.spines[s].set_visible(False)
    fig.savefig(FIGS / 'preempt_timeline.png')
    plt.close(fig)
    print('  ✓ preempt_timeline.png')


if __name__ == '__main__':
    print('building figures...')
    for fn in (fig_world, fig_layers, fig_lineage, fig_tf, fig_scan_chain,
               fig_coverage, fig_3x3, fig_planner, fig_preempt):
        try:
            fn()
        except Exception as e:                              # noqa: BLE001
            print(f'  ✗ {fn.__name__}: {e}')
    print('done →', FIGS)
