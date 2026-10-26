"""从生成的 SDF 里算出每个物体的真实 z 跨度，判断 0.132 m / 0.75 m 两个扫描平面各能看到谁。

为什么不用 semantics.json：它只有平面尺寸（w/d），没有高度。
为什么不用 build_world.py 的函数默认值：很多调用处传了覆盖参数（如 shoe_cabinet(h=0.70)），
只有 world 文件里 XML 的实际几何才是真值。

跑法：.venv/bin/python dev/probe_slice.py
"""
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
WORLD = ROOT / 'worlds' / 'dan_house.world'
PLANES = (0.132, 0.78)

tree = ET.parse(WORLD)
root = tree.getroot()
world = root.find('world')
models = world.findall('model')
print(f'{WORLD.name}: {len(models)} 个 model，平面 = {PLANES}')


def _geom_z(g):
    """从一个 <geometry> 的第一个子元素取 z 方向尺寸；取不到返回 None。"""
    if g is None or len(g) == 0:
        return None
    shape = g[0]
    size = shape.find('size')
    if size is not None and size.text:
        parts = size.text.split()
        if len(parts) >= 3:
            return float(parts[2])
        return None
    cyl = shape.find('length')
    if cyl is not None and cyl.text:
        return float(cyl.text)
    rad = shape.find('radius')
    if rad is not None and rad.text:          # sphere
        return 2 * float(rad.text)
    return None


def zspan(model):
    zs = []
    for tag in ('collision', 'visual'):
        for node in model.findall(f'./{tag}') + model.findall(f'./link/{tag}'):
            hz = _geom_z(node.find('geometry'))
            if hz is None:
                continue
            p = node.find('pose')
            pz = float(p.text.split()[2]) if p is not None and p.text else 0.0
            zs.append((pz - hz / 2, pz + hz / 2))
    if not zs:
        return None
    return (min(z[0] for z in zs), max(z[1] for z in zs))


LANDS = ('stool', 'umbrella_stand', 'potted_plant', 'shoe_cabinet',
         'console_table', 'landmark_bookshelf')

rows = []
for m in models:
    name = m.get('name') or '?'
    s = zspan(m)
    if s is None:
        continue
    rows.append((name, s, s[0] <= PLANES[0] <= s[1], s[0] <= PLANES[1] <= s[1]))

vis_lo = [r for r in rows if r[2]]
vis_hi = [r for r in rows if r[3]]
lost = [r for r in rows if r[2] and not r[3]]
gained = [r for r in rows if r[3] and not r[2]]

print(f'\n0.132 m 平面可见：{len(vis_lo)} / {len(rows)}')
print(f'{PLANES[1]:.2f} m 平面可见：{len(vis_hi)} / {len(rows)}')
print(f'抬到 {PLANES[1]} 会消失：{len(lost)} 个')
for n, s, _, _ in sorted(lost, key=lambda r: -r[1][1]):
    print(f'  ✗ {n:<26} z = {s[0]:.2f} ~ {s[1]:.2f}')
if gained:
    print(f'\n0.75 才出现（0.132 看不到）：{len(gained)} 个')
    for n, s, _, _ in gained:
        print(f'  + {n:<26} z = {s[0]:.2f} ~ {s[1]:.2f}')

print('\n走廊地标在 0.75 m 平面上的存活情况：')
for n, s, lo, hi in rows:
    if any(n.startswith(k) or k in n for k in LANDS):
        print(f'  {"✅" if hi else "✗ "} {n:<26} z = {s[0]:.2f} ~ {s[1]:.2f}'
              f'{"   （差 %.2f m 才够）" % (PLANES[1] - s[1]) if not hi else ""}')
