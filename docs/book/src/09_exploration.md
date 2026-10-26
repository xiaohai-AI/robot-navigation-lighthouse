# Two ways to map, and how to measure them

## 9.1 Tour versus exploration

The project can produce a map two ways. Both are kept, because they answer different questions.

**Tour (frozen).** `tour_node.py` drives 32 waypoints from `config/route.json` (each entry is
`[x, y, 'drive'|'spin', note]`), publishing plain `Twist` commands — it does not use Nav2 at all.
It takes roughly seven minutes of simulated time, produces a clean 359 × 278 map, and is the
fastest way to a usable map for a *known* floor plan. It is also the phase-1 acceptance
artefact: unchanged since 2026-10-20 17:21.

**Frontier exploration.** `frontier_explore.py` starts with no map, detects the boundary between
free and unknown space on the live SLAM map, and asks Nav2 to drive to selected frontiers until
none remain (or a goal/time budget is exhausted). It is the interesting one, and the rest of
this chapter is about it — including the fact that, in this house, it never beat the tour map's
coverage.

## 9.2 The exploration node, in the order it makes decisions

```
① detect      free cells adjacent to unknown → connected clusters (min_cluster = 12 cells)
              cluster centroid → candidate goal
② filter      reject goals closer than explore_min_goal_dist (0.60 m)
              reject goals whose clearance < explore_goal_clearance (0.40 m)
              reject goals in the "fake frontier" blacklist (see below)
③ pick        score by gain / distance, with a revisiting penalty (explore_revisit_gap 0.4)
④ pre-check   ask ComputePathToPose whether a path exists (timeout 1.5 s)
              no path → soft-blacklist for 30 s
              path exists but absurd → detour rule: dist < 3.0 m and path > 3·dist + 1.5 m
              → soft-blacklist for 60 s
⑤ drive       send NavigateToPose; watchdog: no displacement for 12 s → abandon + blacklist
⑥ checkpoint  save map every 8 goals (this is what makes the coverage curve in §9.4 possible)
```

Two mechanisms deserve their own paragraph.

**Fake frontiers.** A frontier can persist for many goals because the planner keeps failing to
reach it, which makes the node waste its entire budget on the same unreachable edge. The fix is
a *pending gain* rule: a frontier whose expected gain is below `min_gain = 40` cells and which
has been attempted (or pending) for more than 4 s is blacklisted, not retried. Measured effect:
the "frontier ping-pong" pattern disappeared from the logs (Chapter 3 of the phase-3 notes).

**Pre-checking with an action, not an assumption.** Asking the planner for a path before sending
the goal costs one action call (a few hundred milliseconds) and converts "drive there, get stuck,
wait 12 s, blacklist" into "blacklist immediately". On a 40-goal run that is the difference
between ~4 and ~11 stalls, depending on configuration.

## 9.3 What was actually compared

Three campaigns were run, each in its own isolated DDS domain and Gazebo partition, each writing
to its own map prefix and CSV row. The driver is `tools/explore_ab.py` (§9.5).

**(a) Planner A/B, 40 goals, identical start.** The global planner was the variable.

![Planner A/B at 40 goals. The entire difference is concentrated in the study: 2.1 m² unknown
with NavFn versus 16.4 m² with SmacPlanner2D.](figures/planner_ab.png){width=92%}

| Arm | reached | stalls | unknown | idle | sim time |
|---|---|---|---|---|---|
| NavFn | **35/40** | **4** | **18.0 m²** | 8.3 % | 21.9 min |
| SmacPlanner2D | 28/40 | 11 | 36.2 m² | 25.3 % | 22.2 min |

Verdict: NavFn stays. Note the *reason* the project believes this: it is not "NavFn is better"
in general, it is that the difference is localised in one room, and that room's failures are
driven by goal clearance (§8.2), which interacts with the planner's tolerance settings.

**(b) Goal clearance 0.40 / 0.50 / 0.55, three runs each.**

![Three runs per arm, 40 goals each. Each dot is a run; the orange bar is the median. The spread
inside an arm is larger than any shift between arms.](figures/clearance_3x3.png){width=96%}

| Arm | unknown (median [range]) | reached | stalls | sim time | idle |
|---|---|---|---|---|---|
| 0.40 | 20.2 [17.9–31.1] | 33 [30–34] | 6 [5–8] | 18.9 [16.9–21.3] | 11.3 % |
| 0.50 | 21.2 [20.3–22.2] | 36 [32–36] | 3 [2–7] | 16.8 [16.2–17.1] | 9.6 % |
| 0.55 | 23.1 [19.3–33.1] | 33 [30–35] | 6 [4–8] | 16.6 [15.6–18.8] | 10.6 % |

Between-arm median differences versus within-arm ranges: unknown **2.9 vs 13.8 m²**, reached
**3.0 vs 5.0**, stalls **3.0 vs 5.0**. Every between-arm difference is smaller than the noise
inside an arm. The honest conclusion is therefore *not* "0.50 is best" — it is
**"clearance is a safety floor, not a coverage knob; keep it ≥ `inflation_radius` and stop
tuning it."**

One of the nine runs was invalid (0 goals issued, 25 minutes, 182.8 m² unknown — a Nav2
lifecycle bring-up race, §9.5) and was replaced by a retry on a new domain, whose result landed
inside the 0.50 arm's range. That replacement is recorded, not hidden: replacing a failed run
with a retry is legitimate, silently deleting it would not be.

**(c) 80 goals: does more budget help?**

![Coverage versus goals issued, recomputed from the checkpoint maps of both 80-goal runs.](figures/coverage_curve.png){width=88%}

| goals | 8 | 16 | 24 | 32 | 40 | 48 | 56 | 64 | 72 | 80 |
|---|---|---|---|---|---|---|---|---|---|---|
| run 1 | 107.2 | 70.0 | 57.3 | 31.5 | 18.7 | 18.2 | 18.0 | 17.0 | 16.6 | 15.8 |
| run 2 | 111.3 | 81.9 | 72.1 | 32.5 | 22.5 | 19.4 | 19.1 | 16.7 | 16.5 | 16.2 |

Both runs (63/80 and 57/80 reached, 40.9 and 40.7 minutes of simulated time) ended by hitting
the goal cap, not by exhausting frontiers. The curve flattens around **48 goals**: the last 32
goals buy 2.2–2.9 m², at roughly 0.1 m² per goal, while stalls triple. Hence the project's
budget rule: **choose the goal count at the knee of the curve (40–48), not a round number.**

## 9.4 Why no configuration beat the test map

The live acceptance map has 13.5 m² interior unknown. No exploration configuration ever beat it:
the best 40-goal run left 15.8 m², the best 80-goal run 15.8 m². Two explanations, both partial:

* the 13.5 m² figure came from a single run of a different configuration and may itself be a
  lucky sample (it has never been reproduced under the current driver);
* the residue is dominated by shadow pockets behind furniture and unreachable slivers, which a
  frontier-based explorer cannot enter by construction.

The project records this as a **known ceiling**, not as a target: the remaining unknown area is
not proportional to effort, and stating that plainly prevents a future maintainer from burning
weeks on a metric that has already been measured to plateau.

## 9.5 The driver: how to run an experiment instead of a demo

`tools/explore_ab.py` (~420 lines) turns "run it a few times" into a procedure. Per arm and
repetition it:

1. allocates a fresh `ROS_DOMAIN_ID` / `IGN_PARTITION` and a unique map prefix;
2. launches the stack and waits for the *exploration started* marker, with a **210 s boot
   self-check** that also fails on any `Failed to bring up` line from Nav2;
3. starts an evaluation probe that records velocity shares (idle / rotating / moving);
4. polls until "exploration finished — no frontiers" or the goal/time cap;
5. computes eight metrics itself (reusing `map_quality.py` for map decoding), appends a CSV row;
6. cleans up the domain and prints a median table.

Three validity guards exist because each was earned:

| Guard | Failure it prevents | Evidence |
|---|---|---|
| 210 s boot self-check + `nav_bringup_failed` | a run that issues 0 goals and burns 25 minutes producing a 182.8 m² "result" | the 0.50 #3 run |
| `no_goals` is never counted as complete | the same, in the aggregation stage | `explore_ab.py` filter |
| retry on a new domain, log name carries the domain | a retry overwriting the failed run's log — i.e. losing the only evidence of the failure | observed before the fix |

Resume behaviour follows from the same principle: a run is "done" only if `status == done` **and**
`arrived > 0`. Anything else is re-attempted.

## 9.6 Methodology: what this chapter cost

Three lessons, all recorded because they were learned expensively:

1. **One run per arm is not an experiment.** A 15-goal A/B showed SmacPlanner2D winning by
   21 m²; at 40 goals the same comparison reversed to an 18 m² *loss*. Short runs measure
   variance.
2. **Report ranges next to medians.** The 3×3 table is the reason the clearance question is
   closed: not because a winner was found, but because the noise floor was measured and every
   candidate effect fell below it.
3. **Budget by the knee, not by round numbers.** 80 goals is 40 minutes of simulation to buy
   2 m² — an obviously bad trade once the curve exists, and a plausible plan before it did.

> **Takeaways.** Tour and exploration coexist because they answer different questions. The
> exploration node's value is in its *cancellation* machinery (fake frontiers, path pre-checks,
> stall watchdog), not in its frontier scoring. Three campaigns produced one adopted change
> (NavFn), one closed question (clearance) and one budget rule (≈48 goals). Everything else was
> noise, and proving that it was noise was the work.
