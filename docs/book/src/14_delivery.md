# Delivery: what is frozen, what is open

## 14.1 The lineage of every artefact

![Generated, consumed, frozen. Yellow boxes are code; green boxes are generated data; the red
box is the read-only delivery snapshot.](figures/lineage.png){width=100%}

Reading the figure left to right is the fastest way to understand the repository:

| Layer | Artefacts | Authority |
|---|---|---|
| Generator | `build_world.py` (898 lines) | the only place world truth is written |
| Generated data | `worlds/*.world`, `config/semantics.json`, `config/route.json` | never edited by hand |
| Models | `models/lighthouse/`, `models/lighthouse_3d/` | spawned via `model://` URIs |
| Runtime | `dan_house.launch.py` (495 lines) + `config/*.yaml`, `config/nav_to_pose*.xml` | wiring order and parameters |
| Evidence | `config/maps/*` (390 files), `experiments/ab_explore.csv`, logs | append-only in spirit |
| Agent | `mak_demo/` (2161 lines), `agent_bridge.py` (287) | language, semantics, concurrency |
| Tools | `tools/*.py` | measurement, assertions, drivers |
| Frozen | `delivery/2026-10-26/` | read-only, hash-manifested |

## 14.2 The frozen snapshot

Five documents were frozen on 2026-10-26 as the deliverable set:

```
delivery/2026-10-26/
  README.md                       1281 lines   full project manual (Chinese)
  STATUS.md                        644 lines   acceptance state + chapters ①–⑮ of experiments (Chinese)
  docs/phase2A_results.md          372 lines   agent-guided navigation results (Chinese)
  docs/phase2B_vision.md           705 lines   camera / VLM / passability results (Chinese)
  docs/phase3_lidar3d.md           437 lines   64-line lidar conversion (Chinese)
  MANIFEST.sha256                              SHA-256 of all five
  交付说明.md                                   index: scope, acceptance, gaps, how to verify
```

Properties that make it a freeze rather than a copy:

* byte-identical to the working files at the moment of freezing (`cmp` verified);
* file mode 444 — an accidental edit announces itself;
* `sha256sum -c MANIFEST.sha256` verifies all five;
* the working documents carry a pointer to the snapshot, so a reader who edits them knows a
  frozen version exists;
* the snapshot records the git state (`main`, HEAD `45dd84e`, 1570 uncommitted paths) and states
  explicitly that it freezes **working-tree content, not a commit**.

That last point is the snapshot's main weakness and is stated in the index: to reproduce from
source control, these files must be committed and the resulting hash recorded.

## 14.3 Reproducing the system from zero

```bash
# 1. the world (regenerates world + semantics + route)
python3 build_world.py --lidar 3d

# 2. mapping the classical way (frozen tour)
ros2 launch ./dan_house.launch.py lidar:=3d slam:=true tour:=true headless:=true

# 3. save the map (during or after the tour)
ros2 run nav2_map_server map_saver_cli -f config/maps/my_map

# 4. navigate + agent
ros2 launch ./dan_house.launch.py lidar:=3d nav:=true loc:=truth headless:=true \
  map:=config/maps/dan_house_lighthouse3d.yaml
#   terminal 2: ros2 launch rosbridge_server rosbridge_websocket_launch.xml
#   terminal 3: /usr/bin/python3 agent_bridge.py
#   terminal 4: cd mak_demo && .venv/bin/python main.py --interactive

# 5. exploration instead of a tour
ros2 launch ./dan_house.launch.py lidar:=3d slam:=true explore:=true nav:=true headless:=true
```

Two operational rules that cost hours when ignored: stop the simulation with **Ctrl+C in its
terminal** (not `pkill`), and run `bash preflight.sh` afterwards until it reports zero gz
processes. An orphaned physics engine is invisible in `ros2 topic list` and expensive in CPU.

## 14.4 What is deliberately not done

This list is the most important part of the book for a future maintainer, because every item is
a decision, not an oversight.

| Gap | Status | Why it matters |
|---|---|---|
| `loc:=amcl` (real-robot localisation) | **not re-verified** since truth poses were adopted | every acceptance number in this book assumes truth poses; hardware has none |
| EKF fusion drift | measured 7.1 m / −37° over a long run, unsolved | blocks any "wheel odometry only" configuration |
| 3D obstacles in the costmap | not implemented; all four param files read `LaserScan` | overhangs, table tops and shelves are invisible to navigation |
| Semantic map from perception | not attempted | monocular camera without depth; the map is authored truth instead |
| Coverage beyond 13.5 m² interior unknown | measured ceiling | frontier exploration plateaus; the residue is shadow pockets |
| Voice input / wake word | not implemented | the REPL is the only input channel |
| Warehouse-scale behaviour | analysed, not built | Chapter 15 |
| Real hardware | never run | this is a simulation project throughout |

## 14.5 Reading order for a newcomer

1. This book's Chapter 1 (ten minutes) — what the system does.
2. `README.md` §0–§5 — the hands-on path to a map and a goal.
3. `STATUS.md` "验收状态" table — what is claimed, with evidence links.
4. `docs/phase2A_results.md` §3 — why authority moved out of the LLM.
5. Chapter 9 of this book — how experiments are run here, before proposing a change.

> **Takeaways.** One generator writes all world truth. Evidence is append-only and traceable.
> The delivery snapshot is verifiable and honest about being a working-tree freeze. Chapter 14.4
> is the project's conscience: eight things it does not do, each with a reason.
