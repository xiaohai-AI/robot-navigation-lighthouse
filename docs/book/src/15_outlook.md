# From a house to a warehouse

A natural next question — and the one that decides whether this architecture generalises — is
what it would take to replace the seven-room house with an Amazon-style fulfilment centre. This
chapter answers it with the code as the unit of accounting, and it is deliberately written before
any warehouse code exists, so the estimate can be checked later.

## 15.1 What is world-specific today

Only five things in the repository know anything about *this* house:

| Artefact | Size | Warehouse equivalent |
|---|---|---|
| `build_world.py` — room table, corridor constant, door widths, route table | 898 lines, hardcoded table | a new generator (racks, aisles, docks, charging bays) |
| `worlds/*.world` | 165 lines, 136 models (generated) | regenerated |
| `config/semantics.json` | 61 objects, 33 classes (generated) | racks, pallets, totes, conveyor, dock doors |
| `config/route.json` | 32 waypoints (generated) | inspection / pick route |
| `mak_demo/data/lexicon.json` | 35 alias entries | warehouse vocabulary + slot addresses |
| `config/nav2_params*.yaml` | tuned for 16 m rooms | retuned for metre-wide aisles at 60 m scale |

Everything else — 3900 lines of robot model, launch wiring, agent layer, tools, experiments —
is world-agnostic as written.

## 15.2 What is hardcoded that would have to change

Two structural couplings, both small and both worth naming precisely:

**(a) The launch file cannot choose a world.** `dan_house.launch.py` holds `WORLD_2D` and
`WORLD_3D` as constants and selects between them only by `lidar`. A warehouse needs a
`world:=home|warehouse` argument resolving to a bundle
`{world_file, semantics, lexicon, map_prefix, params}` — roughly 20 lines.

**(b) The agent resolves paths, not places.** `semantic_map.py` reads
`config/semantics.json`, `mak_demo/data/lexicon.json` and one map path as module constants. A
warehouse needs a configuration root parameter (~30 lines), plus the test vocabulary, which is
spread across ~100 occurrences of home-specific words in `mak_demo/dev/check_*.py`.

## 15.3 Three changes that are architectural, not configurational

This is where the real work is. They are listed in order of how deeply they cut.

**1. Scale.** The house is 17.8 × 13.8 m with a 60 m loop. A fulfilment centre is 60 × 40 m or
larger with kilometre-scale driving. Consequences: laser range (12 m) truncates in open dock
areas; SLAM loop closure over long aisle loops is untested here (the recorded drift experiment
only covers the house); the global costmap grows to ~1200 × 800 cells (still tractable, but
update rates must be re-measured); and the exploration budget rule must be re-derived — the
coverage knee at 48 goals is a property of *this* world's frontier density, not a law.

**2. 3D obstacles become mandatory.** This is the change that cannot be deferred. A warehouse is
full of geometry that a single horizontal slice at one height cannot see: rack beams, pallets at
varying heights, forklift tines, conveyor overhangs, and the gap *under* a shelf that a 0.85 m
tall robot could enter but never leave. Today all four Nav2 parameter files declare
`observation_sources: scan` with `data_type: "LaserScan"`. The 64-line point cloud is already
published and bridged but **nothing subscribes to it**. The costmap must be fed from
`/scan/points` (voxel layer with a real 3D source, or several horizontal slices with explicit
heights), and the passability logic must stop assuming a plane.

**3. Semantics change shape, not just vocabulary.** Today a place is
`room + object(x, y, yaw, w, d)`. A warehouse's central concept is a **slot address** —
aisle-rack-bay-level, e.g. `A-12-03-2` — which is a coordinate system, not an object list. The
resolver therefore needs a new module (`slot_map.py`, ~200–300 lines) that parses an address,
expands rack geometry into accessible slots, and reuses `standoff_pose()` for the final pose
(because "drive to a rack face, not into it" is exactly the geometry already solved here).

Two secondary consequences: the lexicon must map operator language to slot addresses
(*"A12 货架第三个托盘"*), and mode D's visual confirmation becomes *more* valuable, not less —
it is the only mechanism that can check "am I in front of the rack the address claims".

## 15.4 Effort, in self-contained steps

| Step | Deliverable | Estimate | Acceptance |
|---|---|---|---|
| W1 | warehouse world generator + first map | 1–2 days | SDF valid, tour/exploration produces a map, no collisions |
| W2 | parameter adaptation (Nav2, exploration, route) + large-scale localisation | 0.5–1 day | gates G1/G1b/G2 pass in the new world |
| W3 | slot semantics + vocabulary + agent integration | 1–2 days | "go to A-12-03-2" drives and arrives; mode D confirms the rack |
| W4 | 3D obstacles in the costmap | 1–2 days | a pallet above the slice is avoided, not driven under |

Total 4–6 days, each step independently acceptable with the existing gates — which is the real
argument for the architecture: **the acceptance machinery transfers unchanged**, so the estimate
is bounded by world-building and parameter work, not by redesign.

## 15.5 The recommendation recorded here

Add a world module inside this repository rather than forking the project:

* ~3900 lines are world-independent; a fork would mean fixing every future bug twice (the four
  preemption bugs of Chapter 12, and the scan guard, would each have two homes);
* the phase-1 baseline is frozen as a read-only delivery snapshot, so new work must be
  *additive* by construction;
* isolation is already available per world (separate semantics/route/maps directories, separate
  `world:=` value, separate branch).

Shape of the change:

```
build_world_warehouse.py         new generator (the existing 898 lines untouched)
worlds/warehouse_{2d,3d}.world   generated
config/warehouse/{semantics.json, route.json, nav2_params*.yaml, maps/}
mak_demo/data/lexicon_warehouse.json
dan_house.launch.py              + world:=home|warehouse  (~20 lines)
branch: feature/warehouse-world
```

## 15.6 Decision

As of 2026-10-26 this work is **deliberately postponed**. The current project's objectives are
met and frozen; a warehouse would add two days of world-building before the interesting question
(3D obstacles, slot semantics) can even be posed. Recording the analysis now means the decision
can be re-opened cheaply later, with the estimate already on the table.

> **Takeaways.** The world is data; the stack is mechanism. Two small couplings (world selection,
> configuration root) stand between the current code and a second environment. Three changes are
> architectural: scale, 3D obstacles, slot-address semantics. The acceptance machinery is the
> part that transfers for free — and the part that makes a 4–6 day estimate credible.
