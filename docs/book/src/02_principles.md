# Eight design principles

This chapter is the *constitution* of the project. Every later chapter can be read as an
application of one of these rules, and every rule below carries a concrete instance where
it was either followed or violated and then repaired.

## 2.1 Simulation is a measuring instrument, not a toy

A simulator that runs at 0.4× real time silently changes every experiment: the laser publishes
at 4 Hz instead of 10, the controller's progress checker times out, and you spend a day
debugging a "navigation bug" that is really a frame-rate bug.

Therefore the project treats real-time factor (RTF) as a first-class quantity and provides
four levers: run headless (no GUI), keep the physics step at 1 ms, avoid textures you do not
need, and offload rendering to the GPU when the GUI *is* needed. Measured RTF in the finished
system: **0.99** (2D sensors) and **0.98** (64-line lidar). Earlier, a GUI session with a
heavy world produced `/scan` at 5 Hz and the whole stack looked broken.

## 2.2 One source of truth for the world

`worlds/*.world` and `config/semantics.json` and `config/route.json` are **generated**.
Their single source is `build_world.py` (898 lines). Hand-editing a `.world` file is forbidden
because the next regeneration silently discards the edit — and because three consumers
(the physics world, the SLAM ground truth of what exists, and the tour route) would then drift
apart without anyone noticing.

    build_world.py  ──generates──▶ worlds/dan_house.world
                    ──generates──▶ worlds/dan_house_3d.world
                    ──generates──▶ config/semantics.json      (7 rooms + 61 objects)
                    ──generates──▶ config/route.json          (32 tour waypoints)

The payoff appears in Chapter 11: because the semantic map is generated from the same table
that places the furniture, object coordinates are exact to the millimetre — a deliberate
choice with a matching cost, also documented there.

## 2.3 The model decides *what*, code decides *whether* and *how*

The single most important structural decision of phase 2. An early version let the LLM decide
everything, including whether a request was safe to execute. It produced plausible navigation
to places that did not exist and, worse, could not be debugged: the failure was inside a
sampled token.

The final design moves authority down:

| Question | Decided by |
|---|---|
| Is this a navigation request, a question, a cancellation or a refusal? | a deterministic table (`preempt.py`, `planner.py`) |
| Which room or object does the phrase mean? | lexicon + scoring (`semantic_map.resolve_*`) |
| Where should the robot stand to see the object? | geometry, 16-direction sampling (`standoff_pose`) |
| What is the overall plan for a multi-clause sentence? | LLM (`qwen3-vl:8b`) with tools — but *code* decides the route |
| Is the camera actually looking at a fridge? | VLM (`qwen3-vl:8b`) with closed options |

The LLM never emits a velocity. It emits *intent plus arguments*; code validates the arguments
against the semantic map before anything moves.

## 2.4 One 2D contract in the middle

Every consumer above layer 2 — SLAM, both costmaps, the exploration node, the agent's
passability check — reads the same message: `sensor_msgs/LaserScan` on `/scan`, 360 rays,
0.12–12.0 m, `frame_id=base_scan`.

That is why the 64-line upgrade did not ripple through the system: the 23,040-point cloud is
projected into a single 2D slice (`pointcloud_to_laserscan`, −0.75 m … +0.05 m) and everything
downstream is unchanged. The cost of this contract is stated plainly in Chapter 14: the
costmap cannot see what sits above the slice.

## 2.5 Freeze what passed; add next to it

Acceptance artefacts are immobilised rather than trusted to discipline:

* `tour_node.py` and the 2D world are untouched since their acceptance run;
* historical maps live under `config/maps/preserved/`;
* the five deliverable documents are snapshotted read-only with a SHA-256 manifest
  (`delivery/2026-10-26/`);
* exploration experiments may only modify `config/nav2_params_explore*.yaml` and
  `config/nav_to_pose_explore.xml`, never the acceptance parameters.

## 2.6 Measure with one decoder, and never trust a single map statistic

Map quality is computed by exactly one module, `tools/map_quality.py`, used by every probe and
by the experiment driver. The reason is concrete: a second decoder that treated "unknown" as
"free" reported *exploration finished* while 180 m² of the house had never been seen. One
decoder, one definition of free/occupied/unknown, no contradiction between tools.

## 2.7 One run per arm is not an experiment

The project learned this the expensive way. A single 40-goal run said planner A was better;
three runs per arm said the same thing more strongly but also revealed that the *spread inside
an arm* exceeded every difference *between* arms for the clearance parameter. The rule that
followed: for any A/B claim, ≥ 3 repetitions, ≥ 40 goals, compare medians, and report the
within-arm range next to the between-arm difference. Chapter 9 §9.6 shows both the successes
and the retracted claim.

## 2.8 Fail loudly at startup, report honestly at the end

Two mechanisms embody this rule.

**Startup.** `tools/scan_pub_guard.py` asserts that `/scan` has exactly one publisher and that
the publisher is the expected node. A second publisher is never transient (it means a bridge
line was added twice), so the guard fails immediately; zero publishers is tolerated only inside
a startup grace window.

**Shutdown.** A cancelled navigation leg must never report arrival. This was a real defect:
after a preemption, the cancelled leg printed `failed` and, worse, an unrelated stale result
could mark a *new* goal as failed. The fix (per-goal sequence numbers, Chapter 12) is now
enforced by a grep-level assertion inside the offline test suite, so it cannot silently
regress.

> **Takeaways.** Instrument the simulator; generate the world from one source; keep authority
> for *safety* in code and *intent* in the model; one 2D contract; freeze accepted artefacts;
> one decoder; three runs where two would look sufficient; make failures loud and lies
> impossible.
