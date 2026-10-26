---
title: "Lighthouse"
subtitle: "Building an agent-guided mobile robot from an empty Gazebo world — a design textbook"
author: "Dan · with Hermes"
date: "2026-10-26"
lang: en
toc-title: "Contents"
---

# Preface {.unnumbered}

This book documents one project end to end: a differential-drive robot called **Lighthouse**
that maps a seven-room house in the Gazebo Fortress simulator, navigates it with Nav2, explores
it on its own, and accepts instructions in natural language from a local LLM agent — including
the ability to change its mind mid-drive.

It is written for someone who has to *take over* this project: a new student, a colleague, or
you in six months. That reader does not need a tour of the code. They need to know **why each
piece exists, which piece owns which decision, where the truth lives, and what has actually been
measured versus what is merely believed.** Those four things are the structure of every chapter.

## What "verified" means in this book

Every number quoted here was produced by running something and reading its output; nothing is
"should work". Where a claim is a design intention rather than a measurement, it is marked
as such. Where the project deliberately does *not* do something, the omission is documented
in Chapter 14 rather than hidden — the honest gaps are: localisation with AMCL on a live map
has never been re-accepted since the truth-pose shortcut was adopted, the EKF drift over long
runs is measured but unsolved, and the 2D costmap cannot see obstacles above the laser slice.

## How the book is organised

| Part | Chapters | Question it answers |
|---|---|---|
| I — Orientation | 1–2 | What does the system do, and by which rules is it built? |
| II — The world | 3–5 | Where does the simulated house, the robot model and the runtime come from? |
| III — The classical stack | 6–9 | How do laser data, SLAM, Nav2 and exploration actually work here? |
| IV — The agent | 10–12 | How does a language model drive this robot without being allowed to lie? |
| V — Engineering | 13–15 | How do we know it works, what is frozen, what is still open? |

Appendix A lists every topic and node, Appendix B every tuning parameter, Appendix C a
Chinese↔English glossary (the robot's utterances are Chinese), Appendix D a file index and
Appendix E the experiment log.

## Three paths through the book

* **Operator** (just wants to run it): Chapter 1, then Chapter 13 §13.6, then Appendix A.
* **Engineer** (will modify it): Chapters 3–9 in order, then Chapter 13.
* **Reviewer** (must judge it): Chapter 1, Chapter 2, Chapter 13, Chapter 14, Chapter 15.

\newpage

# The system at a glance

## 1.1 The task

A robot stands at the origin of a 17.8 × 13.8 m simulated house, in the central corridor,
facing +x. Its job, in the finished system, is to accept a sentence like
“带我去厨房看看冰箱” ("take me to the kitchen and look at the fridge"), decide what that means,
drive there without hitting furniture, stop at a pose from which a camera can actually see the
object, answer a question about what it saw, and — if the operator changes their mind halfway —
abandon the trip cleanly.

Everything in this project exists to make that sentence work, and to make its failures
*diagnosable*.

## 1.2 Three phases, three additions

The project grew in three phases. Later phases add capability; they never replace earlier
behaviour, which is why the code has explicit switches rather than one growing pipeline.

| Phase | Added | Evidence of completion |
|---|---|---|
| **1 — classical stack** | Gazebo world, robot model, TF tree, SLAM built by driving a fixed tour, Nav2 navigation on the saved map | tour map 359×278 cells at 5 cm (−0.1° tilt), Nav2 drives to arbitrary goals |
| **2 — agent layer** | semantic map, local LLM (tools + vision), natural-language control, vision confirmation, passability question answering | 7 online + 6 offline acceptance items (`docs/phase2A_results.md`, `docs/phase2B_vision.md`) |
| **3 — sensors and behaviour** | 64-line lidar behind a `lidar:=3d` switch, autonomous frontier exploration, mid-drive preemption, a startup assertion for the laser topic | RTF 0.98, `/scan` 8.50 Hz, exploration campaigns, 7/7 preemption criteria (`docs/phase3_lidar3d.md`) |

The important structural fact: **phase 1 is frozen**. The tour script, the two-dimensional
world and the maps built by the tour are byte-identical to the version that passed acceptance.
Exploration, the 3D sensor and the agent layer are *additions next to* that baseline, never
edits of it. Chapter 14 explains how that freeze is enforced (a read-only delivery snapshot).

## 1.3 Five layers, and the one rule that keeps them apart

![The five layers. Decisions flow downwards as goals; measurements flow upwards as
observations. The LLM sits in layer 4 and never touches `/cmd_vel`.](figures/layers.png){width=88%}

The rule that makes the system debuggable is a strict separation of *authority*:

* **Layer 1 (simulation)** owns physics and sensors. Nothing above it is allowed to pretend
  a sensor is more accurate than it is.
* **Layer 2 (ROS 2 navigation)** owns motion. `Nav2` executes goals; it does not decide them.
* **Layer 3 (semantic map)** owns naming. It answers "which coordinates does the word
  *kitchen* mean", and it is a *database*, not a perception system (Chapter 11).
* **Layer 4 (agent)** owns intent. It may choose a goal, but a deterministic routing table
  decides whether the utterance is a navigation request, a question, a cancellation, or a
  rejection.
* **Layer 5 (interaction)** owns concurrency: one task per utterance, so a new sentence can
  preempt an old one without corrupting state.

A bug in this project is almost always a bug *between* layers: a stale goal result written
into a fresh goal's state (Chapter 12), a semantic label that disagrees with the physical
world (Chapter 11), a second publisher on `/scan` that silently feeds SLAM the wrong data
(Chapter 6).

## 1.4 What the robot can do today

```
$ cd mak_demo && .venv/bin/python main.py --interactive

指令> 带我去厨房                          → drives to the kitchen
指令> 别去了，去客厅                       → cancels the kitchen leg, re-targets living room
指令> 停下                                → cancels, sends no new goal
指令> 冰箱在哪                            → answers positionally, no motion
指令> 前面能过去吗                         → laser + vision passability verdict
指令> 带我去冰箱看看                       → drives to an observation pose, then VLM confirms
指令> 带我去地下室                         → refuses in 0.0 s, no motion
```

Each line is an acceptance item with a measured latency in Chapters 11–12. The refusal is
not politeness: the lexicon has no mapping for *basement*, so the router returns before any
model call (Chapter 10 §10.5).

## 1.5 Acceptance at a glance

The project uses eight runtime gates plus two offline suites. The last full run of every gate
(2026-10-26, isolated ROS domain 161) is the reference result:

| Gate | What it proves | Result |
|---|---|---|
| G1 link | rosbridge, `/agent/pose`, TF all alive | 24 pose frames, status idle |
| G1b localisation | `/odom` agrees with `map→base_link` | 0.00 m / 0.0° |
| G2 drive | one hard-coded goal is executed | `succeeded` 13.3 s, 0.19 m error, 0 recoveries |
| G3 semantic | object → standoff pose → Nav2 | `succeeded` 63.0 s, 0.15 m off plan |
| G4 vision (mode D) | drive, stop at 1.79 m, VLM names the object | observed 1.79 m, VLM 13.1 s, 4/4 checks |
| G5 passability (mode E) | laser and vision agree about a blocked path | laser 1.12 m, vision "blocked", agree |
| G6 question (mode B) | answer without moving | 41.1 s, "south, about 9.2 m" |
| G7 refusal | unknown place rejected, no motion | 0.0 s |
| G8 compound | two-segment command still works | both segments `succeeded` (55.6 s + 58.2 s) |
| Offline suite | 11 self-checks run without ROS | 11/11 |
| Scan guard | laser topic has exactly one publisher | 6/6 unit cases + 4 live scenarios |

> **Takeaways.** Three phases, five layers, one authority rule. Phase 1 is frozen; phases 2–3
> are additive. Eleven acceptance results — not opinions — define "working".
