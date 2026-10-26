#!/usr/bin/python3
"""Build the presentation (English) that accompanies the textbook.

Same content spine as docs/book/src, condensed to 18 slides:
story first, numbers second, and the same honesty about what is *not* done.

Run:  /usr/bin/python3 docs/book/build_pptx.py
Out:  docs/slides/lighthouse-navigation.pptx
"""
from pathlib import Path

from pptx import Presentation
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE

HERE = Path(__file__).resolve().parent          # docs/book
ROOT = HERE.parent.parent                        # robot/
FIGS = HERE / 'figures'
OUT = ROOT / 'docs' / 'slides' / 'lighthouse-navigation.pptx'
OUT.parent.mkdir(parents=True, exist_ok=True)

INK = RGBColor(0x1F, 0x29, 0x33)
NAVY = RGBColor(0x10, 0x2A, 0x43)
ACCENT = RGBColor(0x2B, 0x6C, 0xB0)
MUTED = RGBColor(0x5A, 0x6B, 0x7B)
GOOD = RGBColor(0x2F, 0x85, 0x5A)
WARN = RGBColor(0xC0, 0x56, 0x21)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)
LIGHT = RGBColor(0xF2, 0xF5, 0xF9)

FONT = 'Arial'
MONO = 'Consolas'

prs = Presentation()
prs.slide_width = Inches(13.333)
prs.slide_height = Inches(7.5)
BLANK = prs.slide_layouts[6]


def txbox(slide, x, y, w, h, align=PP_ALIGN.LEFT, anchor=MSO_ANCHOR.TOP):
    tb = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = tb.text_frame
    tf.word_wrap = True
    tf.vertical_anchor = anchor
    tf.paragraphs[0].alignment = align
    return tf


def para(tf, text, size=20, bold=False, color=INK, space_before=6, font=FONT,
         first=False, level=0, italic=False):
    p = tf.paragraphs[0] if first else tf.add_paragraph()
    p.alignment = tf.paragraphs[0].alignment
    p.level = level
    p.space_before = Pt(space_before)
    r = p.add_run()
    r.text = text
    r.font.size = Pt(size)
    r.font.bold = bold
    r.font.italic = italic
    r.font.color.rgb = color
    r.font.name = font
    return p


def header(slide, kicker, title):
    bar = slide.shapes.add_shape(1, 0, 0, prs.slide_width, Inches(1.18))
    bar.fill.solid(); bar.fill.fore_color.rgb = NAVY; bar.line.fill.background()
    tf = txbox(slide, 0.55, 0.10, 12.2, 0.42)
    para(tf, kicker.upper(), size=12, bold=True, color=RGBColor(0x9F, 0xC5, 0xE8), first=True,
         space_before=0)
    tf2 = txbox(slide, 0.5, 0.38, 12.3, 0.72)
    para(tf2, title, size=27, bold=True, color=WHITE, first=True, space_before=0)


def footer(slide, text):
    tf = txbox(slide, 0.55, 7.02, 12.2, 0.34)
    para(tf, text, size=10.5, color=MUTED, first=True, space_before=0)


def bullets_slide(kicker, title, items, note=None, size=19):
    s = prs.slides.add_slide(BLANK)
    header(s, kicker, title)
    tf = txbox(s, 0.62, 1.5, 12.1, 5.3)
    for i, it in enumerate(items):
        if isinstance(it, tuple):
            text, kind = it
        else:
            text, kind = it, 'b'
        marker = '•  ' if kind == 'b' else ('→  ' if kind == 'arrow' else '')
        color = INK if kind != 'warn' else WARN
        bold = kind in ('h',)
        para(tf, (marker if kind != 'h' else '') + text, size=(size + 3) if kind == 'h' else size,
             bold=bold, color=color, first=(i == 0), space_before=(16 if kind == 'h' else 9),
             font=FONT)
    if note:
        n = txbox(s, 0.62, 6.55, 12.1, 0.5)
        para(n, note, size=12.5, italic=True, color=MUTED, first=True, space_before=0)
    return s


def figure_slide(kicker, title, image, caption, bullets=None, img_w=7.6, img_left=0.55,
                 img_top=1.5):
    s = prs.slides.add_slide(BLANK)
    header(s, kicker, title)
    s.shapes.add_picture(str(FIGS / image), Inches(img_left), Inches(img_top),
                         width=Inches(img_w))
    cap = txbox(s, img_left, 1.5 + img_w * 0.62, img_w, 0.8)
    para(cap, caption, size=11.5, italic=True, color=MUTED, first=True, space_before=0)
    if bullets:
        tf = txbox(s, img_left + img_w + 0.35, 1.55, 13.0 - (img_left + img_w + 0.4), 4.9)
        for i, b in enumerate(bullets):
            para(tf, '•  ' + b, size=15, color=INK, first=(i == 0), space_before=10)
    return s


def table_slide(kicker, title, headers, rows, col_w=None, font_size=13, note=None):
    s = prs.slides.add_slide(BLANK)
    header(s, kicker, title)
    n_rows, n_cols = len(rows) + 1, len(headers)
    top = Inches(1.6)
    height = Inches(0.42 * n_rows)
    width = Inches(12.2)
    shape = s.shapes.add_table(n_rows, n_cols, Inches(0.55), top, width, height)
    table = shape.table
    if col_w:
        total = sum(col_w)
        for i, w in enumerate(col_w):
            table.columns[i].width = Emu(int(width * w / total))
    for c, h in enumerate(headers):
        cell = table.cell(0, c)
        cell.text = h
        p = cell.text_frame.paragraphs[0]
        p.font.size = Pt(font_size); p.font.bold = True; p.font.name = FONT
        p.font.color.rgb = WHITE
        cell.fill.solid(); cell.fill.fore_color.rgb = NAVY
        cell.margin_top = cell.margin_bottom = Pt(3)
    for r, row in enumerate(rows, start=1):
        for c, val in enumerate(row):
            cell = table.cell(r, c)
            cell.text = str(val)
            p = cell.text_frame.paragraphs[0]
            p.font.size = Pt(font_size); p.font.name = FONT; p.font.color.rgb = INK
            cell.fill.solid()
            cell.fill.fore_color.rgb = LIGHT if r % 2 else WHITE
            cell.margin_top = cell.margin_bottom = Pt(3)
    if note:
        n = txbox(s, 0.55, 1.6 + 0.42 * n_rows + 0.15, 12.2, 0.5)
        para(n, note, size=12, italic=True, color=MUTED, first=True, space_before=0)
    return s


def layers_slide(kicker, title, layers, note=None):
    """Five-layer stack drawn with native shapes: readable when projected."""
    s = prs.slides.add_slide(BLANK)
    header(s, kicker, title)
    top = 1.55
    h = 0.92
    gap = 0.20
    for i, (name, owns) in enumerate(layers):
        y = top + i * (h + gap)
        box = s.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(0.75), Inches(y),
                                 Inches(8.1), Inches(h))
        box.fill.solid()
        box.fill.fore_color.rgb = [NAVY, RGBColor(0x1B, 0x4B, 0x7A), ACCENT,
                                   RGBColor(0x3E, 0x8E, 0x8A), RGBColor(0x6B, 0x7C, 0x8C)][i]
        box.line.fill.background()
        tf = box.text_frame
        tf.word_wrap = True
        tf.margin_left = Inches(0.16)
        tf.vertical_anchor = MSO_ANCHOR.MIDDLE
        para(tf, name, size=17, bold=True, color=WHITE, first=True, space_before=0)
        tf2 = txbox(s, 9.05, y + 0.06, 3.85, h)
        tf2.vertical_anchor = MSO_ANCHOR.MIDDLE
        para(tf2, owns, size=15.5, color=INK, first=True, space_before=0)
        if i < len(layers) - 1:
            ar = s.shapes.add_shape(MSO_SHAPE.DOWN_ARROW, Inches(1.32), Inches(y + h + 0.02),
                                    Inches(0.30), Inches(gap - 0.05))
            ar.fill.solid(); ar.fill.fore_color.rgb = MUTED; ar.line.fill.background()
    if note:
        n = txbox(s, 0.75, 6.72, 12.0, 0.4)
        para(n, note, size=12.5, italic=True, color=MUTED, first=True, space_before=0)
    return s


def stat_slide(kicker, title, stats, bullets, note=None):
    s = prs.slides.add_slide(BLANK)
    header(s, kicker, title)
    x = 0.62
    for value, label in stats:
        tf = txbox(s, x, 1.55, 3.0, 1.5)
        para(tf, value, size=40, bold=True, color=ACCENT, first=True, space_before=0)
        para(tf, label, size=13, color=MUTED, space_before=2)
        x += 3.05
    tf = txbox(s, 0.62, 3.35, 12.1, 3.0)
    for i, b in enumerate(bullets):
        para(tf, '•  ' + b, size=17, color=INK, first=(i == 0), space_before=11)
    if note:
        n = txbox(s, 0.62, 6.6, 12.1, 0.5)
        para(n, note, size=12.5, italic=True, color=MUTED, first=True, space_before=0)
    return s


# ------------------------------------------------------------------ 1 title
s = prs.slides.add_slide(BLANK)
bg = s.shapes.add_shape(1, 0, 0, prs.slide_width, prs.slide_height)
bg.fill.solid(); bg.fill.fore_color.rgb = NAVY; bg.line.fill.background()
acc = s.shapes.add_shape(1, Inches(0), Inches(4.02), prs.slide_width, Inches(0.06))
acc.fill.solid(); acc.fill.fore_color.rgb = ACCENT; acc.line.fill.background()
tf = txbox(s, 0.9, 2.15, 11.6, 1.0)
para(tf, 'Lighthouse', size=54, bold=True, color=WHITE, first=True, space_before=0)
tf = txbox(s, 0.95, 3.2, 11.6, 0.8)
para(tf, 'Building an agent-guided mobile robot from an empty Gazebo world',
     size=22, color=RGBColor(0xC9, 0xD6, 0xE2), first=True, space_before=0)
tf = txbox(s, 0.95, 4.45, 11.6, 1.6)
para(tf, 'SLAM  ·  Nav2  ·  frontier exploration  ·  natural-language control  ·  preemption',
     size=15, color=RGBColor(0x9F, 0xC5, 0xE8), first=True, space_before=0)
para(tf, 'Dan · 2026-10-26', size=14, color=RGBColor(0x8A, 0x9B, 0xAE), space_before=14)

# ------------------------------------------------------------------ 2 the task
bullets_slide('The goal', 'One sentence, and everything else serves it', [
    ('「带我去厨房看看冰箱」  —  "take me to the kitchen and look at the fridge"', 'h'),
    ('Resolve the words to a place and an object', 'b'),
    ('Drive there without hitting furniture', 'b'),
    ('Stop at a pose from which the camera can actually see the object', 'b'),
    ('Answer what it saw — or refuse honestly if the place does not exist', 'b'),
    ('Abandon the trip cleanly if the operator changes their mind', 'b'),
], note='Design driver: every failure must be diagnosable, not merely observed.')

# ------------------------------------------------------------------ 3 architecture
layers_slide('Architecture', 'Five layers, one authority rule', [
    ('5 · Interaction', 'owns concurrency: one task per utterance'),
    ('4 · Agent', 'owns intent: which goal, in words'),
    ('3 · Semantic map', 'owns naming: a database, read-only'),
    ('2 · ROS 2 navigation', 'owns motion: goals in, velocities out'),
    ('1 · Simulation', 'owns physics and sensors'),
], note='Decisions flow down as goals; measurements flow up as observations. '
        'The model never touches /cmd_vel.')

# ------------------------------------------------------------------ 4 world
figure_slide('The world', 'A generated house, not a hand-drawn one', 'world_floorplan.png',
             'Fig. 7 rooms + central corridor, 17.8 × 13.8 m. 61 objects, 16 of them '
             'deliberately standing in the corridor.',
             bullets=['build_world.py (898 lines) is the only source of world truth',
                      'It also generates semantics.json and the 32-waypoint route',
                      '2D world raises low furniture so a single-line lidar can see it',
                      '3D world keeps real heights',
                      'Hand-editing a .world file is forbidden by rule'],
             img_w=7.0)

# ------------------------------------------------------------------ 5 sensors
table_slide('Sensors', 'Two lidar heads, one camera without depth',
            ['Sensor', 'Rate', 'Good for', 'Must not be used for'],
            [['/scan (LaserScan)', '8.4–8.5 Hz', 'SLAM, costmaps, passability, frontiers',
              'seeing above the slice'],
             ['/scan/points (64-line)', '8.4 Hz', 'future 3D costmap, offline analysis',
              'anything today — nothing subscribes'],
             ['/camera/image_raw', '7.1 Hz, 320×240', 'VLM identity check, passability veto',
              'geometry, scale, depth'],
             ['/imu', '100 Hz', 'EKF experiments', 'accepted navigation'],
             ['/odom', '50 Hz', 'truth-pose navigation, TF', 'real-robot localisation']],
            col_w=[3.2, 2.2, 4.6, 3.4], font_size=13,
            note='The camera is a semantic instrument. It has no depth channel at all.')

# ------------------------------------------------------------------ 6 scan contract
figure_slide('The contract', 'One message type is the spine of the stack', 'scan_chain.png',
             'Fig. In 3D mode the gz LaserScan is never bridged — only the point cloud, '
             'projected to a single 2D slice.',
             bullets=['360 rays, 0.12–12.0 m, frame base_scan',
                      'A guard asserts exactly one publisher AND the right node name',
                      '≥ 2 publishers fails immediately: never transient',
                      'A rogue publisher started later is caught within 30 s',
                      'Cost of the contract: the costmap cannot see overhangs'],
             img_w=7.6)

# ------------------------------------------------------------------ 7 SLAM
table_slide('SLAM', 'The map everything else trusts',
            ['Property', 'Value', 'How it is known'],
            [['size', '359 × 278 cells @ 5 cm', 'PGM header'],
             ['extent', '17.95 × 13.90 m', 'cells × resolution'],
             ['tilt', '−0.1°', 'fitted in the single map decoder'],
             ['interior unknown', '13.5 m²', 'inside the house footprint only'],
             ['live map (3D)', 'dan_house_lighthouse3d.yaml → _explore4.pgm', 'config/maps'],
             ['localisation', 'loc:=truth for every acceptance run', 'the honest gap: loc:=amcl unverified']],
            col_w=[3.0, 5.4, 4.0], font_size=14,
            note='One decoder (tools/map_quality.py) defines free / occupied / unknown. '
                 'A second decoder once reported "done" with 180 m² never seen.')

# ------------------------------------------------------------------ 8 Nav2
bullets_slide('Navigation', 'The door was innocent; the goal point was guilty', [
    ('A 1.30 m door with inflation 0.45 m leaves 0.02 m per side — feasible', 'b'),
    ('Widest-path probe: every door is 0.50–0.55 m half-width → cost 0', 'b'),
    ('Failed goals measured 0.35–0.40 m clearance — inside the inflation band', 'warn'),
    ('Inside that band the cost is 132–177 per cell; DWB must cross all of it', 'b'),
    ('It often prefers to rotate in place instead of entering', 'b'),
    ('Stall diagnosis: 419/419 trajectories valid, vx=0.000 scored 63.40 vs 72.70', 'h'),
    ('The robot was not stuck — standing still was simply cheaper', 'arrow'),
], note='"No valid trajectories = 0" is not a health metric.')

# ------------------------------------------------------------------ 9 two maps
table_slide('Mapping', 'Two ways to build a map, kept for two reasons',
            ['', 'Tour (frozen)', 'Frontier exploration'],
            [['Driver', 'tour_node.py, 521 lines, plain Twist', 'frontier_explore.py, 943 lines'],
             ['Input', '32 waypoints from route.json', 'live SLAM map, no input'],
             ['Time', '~7 min of simulation', '20–45 min, budget-capped'],
             ['Best result', '359 × 278 map, −0.1° tilt', '15.8 m² interior unknown'],
             ['Role', 'phase-1 acceptance artefact', 'the experiment platform']],
            col_w=[2.2, 4.8, 4.8], font_size=14,
            note='The exploration node is valuable for its cancellation machinery, '
                 'not its frontier scoring.')

# ------------------------------------------------------------------ 10 methodology
bullets_slide('Method', 'One run per arm is not an experiment', [
    ('15 goals: SmacPlanner2D wins by 21 m²', 'b'),
    ('40 goals, same code: NavFn wins by 18 m²', 'warn'),
    ('Short runs measure variance, not effect', 'h'),
    ('≥ 3 repetitions, ≥ 40 goals, compare medians', 'b'),
    ('Always print the within-arm range next to the between-arm shift', 'b'),
    ('Isolation: fresh DDS domain + Gazebo partition + map prefix per run', 'b'),
    ('Budget by the knee of the curve (~48 goals), not by round numbers', 'b'),
], note='A driver that cannot invalidate its own runs will happily produce confident nonsense.')

# ------------------------------------------------------------------ 11 coverage
figure_slide('Results', 'More budget does not buy more map', 'coverage_curve.png',
             'Fig. Recomputed from the checkpoint maps of both 80-goal runs. Knee at ~48 goals; '
             'the last 32 goals buy 2.2–2.9 m² in 41 minutes of simulation.',
             bullets=['80 goals: 63/80 and 57/80 reached, both capped by goal count',
                      'Stalls triple towards the cap',
                      'Residue is shadow pockets behind furniture',
                      'No configuration beat the 13.5 m² test map',
                      'Recorded as a ceiling, not a target'],
             img_w=7.2)

# ------------------------------------------------------------------ 12 clearance
figure_slide('Results', 'A parameter that turned out not to matter', 'clearance_3x3.png',
             'Fig. 3 runs per arm, 40 goals each. Between-arm median shift 2.9 m² vs '
             'within-arm range 13.8 m².',
             bullets=['0.40 / 0.50 / 0.55 clearance',
                      'Reached 33 / 36 / 33, stalls 6 / 3 / 6',
                      'Every between-arm difference < the noise inside an arm',
                      'Verdict: clearance is a safety floor, not a coverage knob',
                      'One invalid run was retried on a new domain — and recorded'],
             img_w=7.4)

# ------------------------------------------------------------------ 13 agent
bullets_slide('The agent', 'Language in, motion out — with authority kept apart', [
    ('Four processes: launch · rosbridge · agent_bridge · venv agent', 'b'),
    ('Iron rule: anything importing rclpy runs on the system interpreter', 'warn'),
    ('Status topic is latched, or a late subscriber waits forever', 'b'),
    ('Every goal carries a sequence number; stale results are dropped', 'b'),
    ('Agent model qwen3-vl:8b · reporter model qwen2.5:7b (1.9 s vs 31.7 s)', 'b'),
    ('Seven tools: list_rooms, find_place, where_am_i, go_to, go_to_room, look_at, check_path', 'arrow'),
], note='Asked to 「带我去沙发看看」, the model called find_place, answered, and never moved. '
        'tool_choice="required" was ignored by the compat layer. Hence: code moves the robot.')

# ------------------------------------------------------------------ 14 semantic map
stat_slide('Semantics', 'Authored truth, not perception', [
    ('8', 'rooms (7 + corridor)'),
    ('61', 'objects in the map'),
    ('33', 'object classes'),
    ('0', 'lines of perception code'),
], bullets=['Monocular camera: no depth, no scale → no map write-back',
            'Millimetre coordinates, because it comes from the same table as physics',
            'Standoff: 16 directions, clearance = Nav2 inflation radius (0.45 m)',
            'Observation pose: object radius + 1.40 m, camera height 0.70 m',
            'At 0.55 m the VLM calls a fridge "a door" — a geometric fix, not a model fix'],
   note='The camera confirms identity at query time. It never builds the map.')

# ------------------------------------------------------------------ 15 preemption
figure_slide('Interaction', 'Changing its mind mid-drive', 'preempt_timeline.png',
             'Fig. Isolated domain 160. The cancelled leg prints "cancelled (departing)" and '
             'never claims arrival. 7/7 criteria passed.',
             bullets=['Judgment order: new destination > question > cancel',
                      'Three pieces: to_thread · one task per utterance · code-side table',
                      'Heartbeat 0 vs 25 per 0.5 s proves why to_thread is required',
                      'Four bugs only a live run could find (stale seq, stuck thread, '
                      'merged input, cancel parsed as a place)',
                      'Guardrails: source-level assertions in the offline suite'],
             img_w=7.5)

# ------------------------------------------------------------------ 16 gates
table_slide('Acceptance', 'Eleven measurements, not eleven opinions',
            ['Gate', 'What it proves', 'Result (2026-10-26)'],
            [['G1 / G1b', 'link + localisation', '24 pose frames · 0.00 m / 0.0°'],
             ['G2', 'hard-coded goal', 'succeeded 13.3 s · 0.19 m · 0 recoveries'],
             ['G3', 'object → standoff → Nav2', 'succeeded 63.0 s · 0.15 m off plan'],
             ['G4', 'mode D vision', '1.79 m · VLM 13.1 s · 4/4 checks'],
             ['G5 / G6', 'passability / question', 'laser 1.12 m + vision agree · 41.1 s'],
             ['G7 / G8', 'refusal / compound', '0.0 s · two segments succeeded'],
             ['offline', '11 checks + 6 guard cases', '11/11 · 6/6']],
            col_w=[2.0, 4.6, 5.6], font_size=13,
            note='A cancelled leg may never report arrival — that single sentence summarises '
                 'half of the preemption work.')

# ------------------------------------------------------------------ 17 open items
table_slide('Honest gaps', 'What this project deliberately does not do',
            ['Gap', 'Status', 'Why it matters'],
            [['loc:=amcl', 'never re-verified', 'every number here assumes truth poses'],
             ['EKF fusion drift', '7.1 m / −37°, unsolved', 'blocks wheel-odometry-only running'],
             ['3D obstacles in costmap', 'not implemented', 'overhangs and table tops are invisible'],
             ['semantic map from perception', 'not attempted', 'monocular camera has no depth'],
             ['coverage > 13.5 m²', 'measured ceiling', 'frontier exploration plateaus'],
             ['voice / wake word', 'not implemented', 'the REPL is the only input'],
             ['real hardware', 'never run', 'this is a simulation project throughout']],
            col_w=[4.0, 4.0, 5.0], font_size=13,
            note='Each row is a decision with a reason, not an oversight.')

# ------------------------------------------------------------------ 18 warehouse
table_slide('Outlook', 'From a house to a warehouse: 4–6 days, three real changes',
            ['Change', 'Nature', 'Work'],
            [['scale 18×14 m → 60×40 m', 're-measure', 'range, loop closure, costmap size, budget'],
             ['3D obstacles', 'architectural', 'voxel layer from /scan/points; overhangs become visible'],
             ['slot-address semantics', 'architectural', 'A-12-03-2 → pose; new resolver, reuse standoff'],
             ['world + parameters', 'data', 'new generator, lexicon, route, maps'],
             ['everything else', 'unchanged', '~3,900 lines: robot, launch, agent, tools, gates']],
            col_w=[3.4, 2.6, 6.2], font_size=13,
            note='Recommendation: add a world module in this repo (world:= switch, new branch), '
                 'not a fork. Postponed on 2026-10-26 by decision.')

# ------------------------------------------------------------------ 19 closing
s = prs.slides.add_slide(BLANK)
bg = s.shapes.add_shape(1, 0, 0, prs.slide_width, prs.slide_height)
bg.fill.solid(); bg.fill.fore_color.rgb = NAVY; bg.line.fill.background()
tf = txbox(s, 0.9, 1.5, 11.6, 0.8)
para(tf, 'Three things worth carrying away', size=30, bold=True, color=WHITE, first=True,
     space_before=0)
items = [
    ('Keep authority apart.', 'Code moves the robot and refuses what cannot exist; the model '
     'resolves ambiguity and talks.'),
    ('Measure, then decide — and write down the retraction.',
     'Clearance was a superstition until 3×3 said so. One published conclusion here was withdrawn.'),
    ('Make failures loud at the edges.',
     'A second publisher on /scan, a stale goal result, a cancelled leg claiming arrival — '
     'each is silent by nature, so each got a mechanism.'),
]
tf = txbox(s, 0.95, 2.6, 11.4, 4.2)
for i, (h, b) in enumerate(items):
    para(tf, f'{i + 1}.  {h}', size=21, bold=True, color=RGBColor(0x9F, 0xC5, 0xE8),
         first=(i == 0), space_before=22)
    para(tf, b, size=16, color=RGBColor(0xD5, 0xDF, 0xE8), space_before=6)

prs.save(OUT)
print(f'saved {OUT}  ({len(prs.slides.__iter__.__self__._sldIdLst)} slides)')
