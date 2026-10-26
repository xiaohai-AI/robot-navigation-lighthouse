# The house: world generation

## 3.1 Why the world is code

The simulated house is not a hand-drawn `.world` file. It is emitted by `build_world.py`
(898 lines) from a small table of rooms, a corridor width and a set of furniture placements.
Three reasons, all of which cost real debugging time before the rule was adopted:

1. **Consistency.** The tour route, the semantic map and the physics geometry must agree.
   Generating all three from one table makes disagreement impossible.
2. **Regeneration.** Changing a door width means editing one constant, not hunting for
   occurrences in XML. The finished world has 136 `<model>` elements; no human should
   maintain them by hand.
3. **Diffability.** A generated file is 165 lines / 676 KB of XML, and it never needs a
   code review. The *table* is what gets reviewed.

    $ python3 build_world.py              # 2D world  → worlds/dan_house.world
    $ python3 build_world.py --lidar 3d   # 3D world  → worlds/dan_house_3d.world

## 3.2 Geometry in one table

```python
XW, XE = -9.0, 9.0      # outer wall centre lines  → 18 m
YS, YN = -7.0, 7.0      # outer wall centre lines  → 14 m
COR     = 1.5           # corridor walls at y = ±1.5  → 2.88 m clear
DOOR_W  = 1.3           # standard door width
DOOR_W_BATH = 1.2       # bathroom door

ROOMS = [               # name     x0      x1      y0      y1    floor material
    ('living',  -8.89, -3.20,  1.56,  6.89, WOOD),
    ('kitchen', -3.08,  1.60,  1.56,  6.89, TILE),
    ('study',    1.72,  8.89,  1.56,  6.89, WOODD),
    ('bed1',    -8.89, -4.00, -6.89, -1.56, CARP),
    ('bed2',    -3.88,  0.90, -6.89, -1.56, CARP),
    ('bath',     1.02,  3.60, -6.89, -1.56, TILE),
    ('store',    3.72,  8.89, -6.89, -1.56, WOODD),
]
```

The floor therefore spans 17.8 × 13.8 m of room area; older notes round it to "16 × 12 m",
which is a historical approximation, not a measurement. Door centres sit on the corridor walls
at x = −6.0 (living), −0.8 (kitchen), +5.2 (study), −6.3 (bed 1), −1.5 (bed 2), +2.3 (bath),
+6.2 (storage); kitchen and study are additionally joined by an internal door at x = 1.66,
y = 3.8.

![The generated house. Grey boxes are furniture; orange boxes are the sixteen corridor
landmarks discussed in §3.4.](figures/world_floorplan.png){width=92%}

Why these dimensions? They are a compromise between three constraints that all bite at once:
the robot is 0.42 m wide, so the 2.88 m corridor leaves 2.23 m of slack; a 1.3 m door lets the
0.46 m costmap footprint through with room to spare; and a 5 cm map resolution at this size
gives a 359 × 278 cell map — small enough that SLAM stays fast, large enough that five rooms
are more than a corner.

## 3.3 Furniture, and the two-height trick

61 objects in 33 classes are placed by the same script. Each placement calls one helper:

```python
def foot(cx, cy, w, d, rz, name, cls=None):
    if abs(abs(rz) - math.pi / 2) < 0.2:   # at ±90° swap width and depth
        w, d = d, w
    FOOT.append((cx, cy, w/2.0, d/2.0, name))
    SEM.append({'id': ..., 'class': cls or name, 'room': room_of(cx, cy),
                'x': cx, 'y': cy, 'yaw': rz, 'w': w, 'd': d})
```

That three-line swap has bitten every downstream consumer at least once: because `w`/`d` are
already exchanged for ±90° poses, a visualiser that *also* rotates the box by `yaw` draws it
rotated twice and appears to push furniture through the wall. Chapter 14's figure-generation
script documents the correct rule.

The two world variants differ in one respect:

| | 2D world (`dan_house.world`) | 3D world (`dan_house_3d.world`) |
|---|---|---|
| Lidar | one scan line at 0.78 m | 64 lines, −20° … +15° |
| Low furniture | **raised** so its top is ≥ 0.85 m (`RAISE_LOW`) | real dimensions |
| Purpose | honest SLAM baseline for a single-line sensor | see what a real depth sensor sees |

Raising furniture in the 2D world is not cheating: a single-line lidar genuinely cannot see a
0.4 m coffee table, and if the map is expected to look like the house, the geometry has to be
visible to the sensor. The 3D variant is where the *real* heights live — and where the
costmap's blindness to overhanging geometry becomes visible (Chapter 14).

## 3.4 The corridor landmarks, and why the label does not match the position

Sixteen of the 61 objects have `|y| < 1.5`, i.e. they physically stand in the corridor:
consoles, a cart, stools, umbrella stands, a fire extinguisher. `room_of()` applies a 0.3 m
tolerance, so each of them receives a *room* label — `shoe_cabinet_1` is labelled
`living_room` while standing at y = 1.29 in the corridor.

This is **intentional**: the semantic layer records *ownership* ("this shoe cabinet belongs to
the living room"), not a polygon test. It also produces eight "not in room" warnings during
semantic self-checks, which are correct warnings about a wrong assumption, not bugs. Removing
them by editing the JSON would delete a deliberate design decision in order to silence a
diagnostic.

## 3.5 Self-checks that run with the generator

Generation validates itself before anything is ever launched:

```
① route geometry   min clearance between 32 waypoints: 42 cm ✅
                   7 door passages traversable ✅
② SDF validity     gz sdf -k worlds/dan_house_3d.world → Valid ✅
③ see-ability      12/12 landmarks detected from the tour poses ✅
                   potted_plant south wall: theory 1.09 m, measured 1.10 m
                   shoe_cabinet north wall: theory 2.60 m, measured 2.65 m
```

Step ③ is the interesting one: it closes the loop between "the world says there is a plant
there" and "the laser can actually return a point for it". A landmark that exists but is
invisible to the sensor is indistinguishable from a landmark that does not exist, and the
semantic layer would be quietly wrong.

> **Takeaways.** The world is generated, never edited. One table drives physics, semantics and
> the tour route. Two world variants exist because a single-line lidar and a 64-line lidar do
> not see the same house. Object labels record ownership, not physical containment.
