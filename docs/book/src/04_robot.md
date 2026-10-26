# The robot: Lighthouse

## 4.1 One robot, two sensor heads

Lighthouse is a two-wheel differential drive platform with two caster wheels, a sensor tower,
and a single RGB camera. It exists in two variants that share the chassis but differ in the
lidar:

| | `models/lighthouse/` | `models/lighthouse_3d/` |
|---|---|---|
| Lidar | single-line `LaserScan`, topic `/scan` | 64-line `gpu_lidar`, topic `/scan/points` via bridge |
| Vertical field | one plane at z ≈ 0.78 m | −0.349066 … +0.261799 rad (−20° … +15°) |
| Points per frame | 360 | 360 × 64 = 23,040 (≈ 720 KB, ≈ 7 MB/s at 10 Hz) |
| Status | **frozen** (phase 1 baseline) | active for phases 2–3 experiments |

The 3D head is mounted on a mast whose top sits at 0.833 m; the scan joint is at 0.770 m. The
camera sits lower, at 0.70 m, on purpose — that height is an *empirical* choice explained in
Chapter 11 §11.4.

## 4.2 Chassis numbers that the rest of the stack depends on

```
body             0.306 × 0.281 m       mass 1.3729096 kg
wheel radius     0.033 m              wheel separation 0.288 m
costmap radius   0.23 m               (circumscribed footprint ≈ 0.21 m + margin)
height           0.85 m (tower top)   camera 0.70 m   lidar (3D) 0.833 m
```

Three of these are load-bearing for later chapters: `robot_radius = 0.23` decides whether the
robot fits through a 1.3 m door with a 0.45 m inflation radius (it does, with 0.02 m to spare
per side); `0.288 m` is what the wheel odometry uses; and the 0.033 m wheel radius is the
scale factor that made the "wheels sinking into the floor" bug so confusing (§4.4).

## 4.3 The TF tree is a contract

![TF tree. Everything above `odom` is a decision (SLAM, or a static zero when truth poses are
used); everything below `base_footprint` is mechanical.](figures/tf_tree.png){width=92%}

```python
# dan_house.launch.py — the two switches that change this tree
'loc' : 'amcl'  | 'truth'    # who owns map -> odom
'ekf' : 'false' | 'true'     # who owns odom -> base_footprint
```

* `map→odom` is published by `slam_toolbox` in mapping mode, or by a static identity transform
  when `loc:=truth` (the simulator's `/odom` *is* ground truth), or by AMCL in the real-robot
  configuration.
* `odom→base_footprint` is published by `odom_tf_node.py` (a 40-line node that integrates the
  ground-truth pose) or by `robot_localization`'s `ekf_node` when `ekf:=true`.
* Everything below `base_link` is static: `base_scan`, `camera_link`, `imu_link`, the two
  wheels, the casters.

The rule that follows: **no consumer may assume a transform is fresh**. `slam_toolbox`
publishes `map→odom` at 50 Hz (`transform_publish_period: 0.02`) precisely because a stale
`map→base_link` makes the controller plan from a pose the robot no longer occupies.

## 4.4 Four hardware bugs worth remembering

**(a) Wheels sinking into the floor.** The chassis was placed with its origin at the axle
height, but the wheel collision geometry extended below the ground plane; the physics engine
then pushed the whole body up by a few millimetres every step. Symptom: odometry drifted even
when standing still. Fix: correct the link origins so the wheel's lowest point is exactly at
z = 0 in the spawn pose.

**(b) The robot lifting its nose.** Asymmetric caster placement meant the front casters carried
more load; during hard acceleration the body pitched. Measured `pitch ≈ 0.0004 rad` at rest and
a visible lift under acceleration. Fix: symmetric caster geometry plus inertia values matching
the real mass distribution.

**(c) `/scan` at 5 Hz.** Not a sensor bug at all — RTF was 0.5 because the GUI was rendering a
textured world. Fix: headless server plus the RTF levers of Chapter 2 §2.1.

**(d) The lidar seeing its own tower.** The mast stood 0.089 m in front of the scan plane, and
the laser returned 10 echoes at 0.124–0.138 m, angles −24° … −31°, which in the body frame is
`(0.054, −0.055)` — i.e. the mast, not the chassis. Because the passability threshold was
`MIN_R = 0.12`, those echoes passed the filter, and an open corridor was reported as
"blocked right at the nose". Fix: move the sensor out of the housing, then add a minimum-range
guard that is *smaller* than the true obstacle distance rather than tuned to the artefact.

Lesson: two of these four looked like software bugs for hours. A simulator will happily let you
debug a geometry error as if it were a control error.

## 4.5 Sensors the agent layer can use

| Sensor | Rate | What it is good for | What it must not be used for |
|---|---|---|---|
| `/scan` (LaserScan) | 8.4–8.5 Hz measured | SLAM, costmaps, passability, frontier detection | seeing above the slice height |
| `/scan/points` (3D) | 8.4 Hz measured | future 3D costmap, offline analysis | direct consumer today (nothing subscribes) |
| `/camera/image_raw` | 7.1 Hz, 320×240 rgb8, HFOV 1.204 rad | VLM confirmation, passability second opinion | geometry, scale, depth (monocular, no depth topic) |
| `/imu` | 100 Hz | EKF experiments (mode C) | accepted navigation (unused there) |
| `/odom` | 50 Hz | truth-pose navigation, TF | real-robot localisation (that is AMCL's job) |

The camera row matters for two later chapters: the camera has **no depth channel at all**
(Chapter 11), and it is deliberately *not* part of the mapping pipeline (Chapter 3 of the
project's original design notes) — it is a semantic instrument, not a geometric one.

> **Takeaways.** Two sensor heads, one chassis. The TF tree encodes who owns which decision.
> Two of the four hardware bugs presented as software bugs. The camera has no depth, which
> determines what the agent layer can and cannot do with it.
