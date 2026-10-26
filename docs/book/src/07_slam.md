# SLAM: building the map that everything else trusts

## 7.1 Configuration

Mapping is `slam_toolbox` in asynchronous mapping mode. The parameter file is short and every
line has a reason:

```yaml
slam_toolbox:
  odom_frame: odom
  scan_topic: /scan
  mode: mapping                  # mapping | localization
  resolution: 0.05               # 5 cm — fine enough for a 1.3 m door, cheap enough to be fast
  map_update_interval: 2.0       # 5.0 → 2.0: the live map is also exploration's planning input
  min_laser_range: 0.12          # identical to the sensor and to the projection
  max_laser_range: 12.0          # identical to the sensor and to the projection
  transform_publish_period: 0.02 # 50 Hz: Nav2 must never plan from a stale map→odom
  minimum_travel_distance: 0.3   # keep the default: smaller values make the solver unstable
  minimum_travel_heading: 0.3
  use_scan_matching: true
```

Two of these are not defaults and both were chosen after measurement:
`map_update_interval: 2.0` because exploration plans against the *live* map (a 5 s-stale map
means the frontier list is stale too), and `transform_publish_period: 0.02` because at 10 Hz
the controller occasionally read a transform 100 ms old and planned from the wrong pose.

## 7.2 What the map looks like, quantitatively

| Metric | Value | How it is measured |
|---|---|---|
| Size | 359 × 278 cells | PGM header |
| Extent | 17.95 × 13.90 m | cells × 0.05 m |
| Tilt | −0.1° | Hough-style fit in `tools/map_quality.py` |
| Interior unknown | **13.5 m²** | unknown cells inside x∈[−7.9,7.9], y∈[−5.9,5.9] × 0.0025 |
| Live 3D map | `config/maps/dan_house_lighthouse3d.yaml` → `..._explore4.pgm` | |

The "interior unknown" definition matters and is used everywhere in this book: it counts
unknown cells **only inside the house footprint**, so the permanently-unreachable ring outside
the walls is not counted as coverage loss. Without that restriction every run reports a similar
large number and improvements become invisible.

## 7.3 The ghosting investigation, and one retracted conclusion

Early maps had double walls in the corridors and a visible rotation between the north and south
halves. The investigation proceeded in the only order that works:

```
① is /odom trustworthy?      compare /odom against the simulator's ground-truth pose
② how big is SLAM's patch?   print map→odom translation and rotation over time
③ score the map objectively  tilt, extent, unknown area — not "it looks bad in RViz"
```

Step ① found `/odom` matching ground truth to below 1 mm. Step ② found `map→odom` carrying a
1.25 m / 124.5° "correction" in one session — which led to a diagnosis of an AMCL defect, a
switch to truth poses for simulation, and a paragraph in three documents.

**That conclusion was later withdrawn.** The jump was caused by a manual misconfiguration of
the simulator during that session, not by AMCL or by the stack. The *method* survives (compare
`/odom` with `map→base_link` before reading Nav2 logs — Nav2 will dutifully report "剩余 7.2 m"
for a 4.6 m trip and look like a planner bug), and `loc:=truth` remains the recommended
simulation setting, because in simulation `/odom` *is* truth and AMCL can only add error.

The documents record this as a correction with a date rather than an edit, because a future
reader who finds the old numbers must be able to see that they were measured, and why they were
misinterpreted.

## 7.4 Localisation: three modes, one unfinished branch

| Mode | `map→odom` source | Used for | Status |
|---|---|---|---|
| `loc:=truth` | static identity transform | all acceptance runs, all exploration experiments | **verified** |
| `loc:=amcl` | AMCL on the saved map | the real-robot configuration | **not re-verified** since the truth shortcut was adopted |
| `ekf:=true` | `robot_localization` fusing wheels + IMU | "closer to reality" experiments | measured drift 7.1 m / −37° over a long run — unsolved |

That middle row is the honest gap in this project (Chapter 14 §14.4). Everything you can
measure in this book was measured with truth poses. A real robot has no truth pose, so the
first task on hardware is not the agent layer — it is making `loc:=amcl` hold
`map→base_link` within 0.3 m over a long drive.

## 7.5 Map hygiene: checkpoints, preservation, naming

A map is evidence, so it is never overwritten by an experiment.

```
config/maps/
  dan_house_lighthouse3d.*                     live map for phase 2/3 acceptance
  dan_house_lighthouse.*                       2D baseline (frozen)
  ..._explore_checkpoint_08/16/24/32.*         tour checkpoints (every 8 waypoints)
  ..._ab_1025_1647_*_checkpoint_NN.*           3×3 clearance campaign, 10 runs
  ..._ab_1025_2047_0.40_{1,2}_checkpoint_NN.*  80-goal coverage campaign
  preserved/                                   four historical maps, untouched
```

Naming carries the experiment tag, the arm and the repetition, because the alternative — one
file named `map` overwritten by whichever run finished last — is how an experiment loses its
evidence. The same rule appears in the driver (Chapter 9 §9.5): a retry writes to a new
domain-suffixed filename instead of overwriting the failed attempt.

## 7.6 Known SLAM weaknesses

* **Radial spray.** Rays that never return are still painted as free space at their far end, so
  the map has thin free streaks in unexplored directions. It affects map aesthetics and
  marginally the frontier detector; it does not affect navigation.
* **Long-haul drift is unmeasured.** The house allows a loop of ~60 m; a warehouse aisle loop is
  several hundred metres. Nothing in this project has measured loop-closure quality beyond the
  house.
* **AMCL on a ghosted map is unreliable** — which is why phase 2 uses truth poses, and why the
  real-robot path must begin with a clean map and a localisation acceptance test.

> **Takeaways.** SLAM parameters are chosen to serve exploration's freshness needs. Map quality
> is a number, computed by one decoder. One published conclusion in this project was later
> withdrawn — and the withdrawal is part of the record. The localisation branch that a real
> robot needs is the branch that is *not* verified here.
