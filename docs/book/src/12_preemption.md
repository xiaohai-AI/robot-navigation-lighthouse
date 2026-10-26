# Changing its mind: preemption

## 12.1 The interaction contract

The robot is expected to accept speech **while driving**. Three behaviours are required:

| Utterance while driving | Expected behaviour |
|---|---|
| a different destination | abandon the current goal, drive to the new one, never claim the abandoned one arrived |
| 「停下」 / "stop" | cancel, send no new goal, stop cleanly |
| a question | answer *while continuing*, and do not swallow the question |

The third row is not decorative: it is the row that broke.

## 12.2 Judgment order is a code decision

`mak_demo/preempt.py` (101 lines) is a pure, testable table with a strict priority:

```
1. new destination present?  → PREEMPT   ("别去厨房了，去卧室" is a redirect, not a cancel)
2. question present?         → PARALLEL  ("停下停下冰箱在哪" must still answer the question)
3. cancel word/prefix only?  → CANCEL    ("停下" alone)
4. otherwise                 → NONE
```

Priority 2 above priority 3 is subtle and was learned the hard way: with the naive order, a
pending "stop" in an utterance that also asked a question caused the question to be dropped
entirely (Chapter 12 §12.4, bug 3). Priority 1 above 3 matters because "别去X了，去Y" contains a
cancel-looking clause and a destination; treating it as a pure cancel strands the robot.

Segment filtering handles the compound case: `filter_cancel_segments()` drops comma-separated
clauses that begin with 别去 / 不用去 / 不去 / 别往 …, and returns the original list unchanged if
everything was dropped (so a pure 「停下」 still reaches route F).

## 12.3 Three pieces, all required

Preemption is not a parsing problem. It is three concurrency changes, and removing any one
degrades the system to "one command at a time":

| Piece | Without it |
|---|---|
| every tool call runs in `asyncio.to_thread` | the event loop blocks inside `go_to`, so `input()` never returns and nobody can speak at all |
| one `asyncio.Task` per utterance | a new utterance waits for the previous task to finish — the definition of not preempting |
| code-side judgment table | the model decides, and re-decides inconsistently under prompt changes |

The first row was measured, not assumed. With a synchronous tool call, a heartbeat coroutine
fired **0** times in a 0.5 s window; wrapped in `to_thread` it fired **25** times.

## 12.4 Four bugs that only a live run could find

All four passed the offline suite. All four appeared within minutes of a real session.

**Bug 1 — the new goal reports failure in 0.1 s.**
*Symptom:* after a preemption, the new goal printed `failed` after 0.1 s while the robot was
still driving, and the agent answered "could not reach the living room, maybe an obstacle".
*Root cause:* `agent_bridge.py` had no goal identity. Nav2's terminal result for the cancelled
goal (status 5) arrived after the new goal had started, was written into the new goal's state,
and cleared `goal_handle` — so the cancel path and the watchdog could no longer reach the real
goal.
*Fix:* a monotonic `seq` per goal, incremented **before** the old handle is released, carried
through response/feedback/result callbacks, with stale callbacks logged and dropped.
*Guardrail:* the offline suite greps the source for `self.goal_seq += 1` before the release and
for all three callbacks taking a sequence argument.

**Bug 2 — cancellation stops nothing.**
*Symptom:* after 「停下」 the terminal kept printing `剩余 5.28 m` every two seconds for two
minutes until the 180 s timeout.
*Root cause:* cancelling an `asyncio` task cannot interrupt a thread already inside
`to_thread`. The worker kept polling a frozen link.
*Fix:* a registry of in-flight goals in `nav_client.py` with `stop()` / `stop_all()`; the polling
loop checks the flag and returns `('canceled', {'detail': 'agent preemption'})` immediately.
`canceled` is deliberately distinct from `rejected`.
*Guardrail:* a unit test that starts a fake `go_to` and asserts it returns `canceled` within 2 s.

**Bug 3 — three utterances become one.**
*Symptom:* typing three lines produced the single string `停下停下冰箱在哪`, and the question was
swallowed as a cancel.
*Root cause:* the progress printer ran in a worker thread and wrote to the same terminal as
`input()`, interleaving prompts and swallowing newlines.
*Fix:* progress printing is off by default in interactive mode (`--progress` re-enables it);
plus the judgment order of §12.2.
*Guardrail:* a test asserting that a concatenated stop+question string routes to PARALLEL.

**Bug 4 — the cancel clause becomes a place name.**
*Symptom:* 「别去了，去客厅」 produced `❌ 家里没有叫「别了」的地方，这一步跳过`. The behaviour
was eventually right, but the user saw a nonsensical rejection.
*Root cause:* the segmenter split on the comma and fed the first clause to the resolver.
*Fix:* `CANCEL_PREFIX` + `filter_cancel_segments()`.
*Guardrail:* four segment-level test cases.

The pattern across all four: **the offline tests verified the logic I wrote, and the live run
verified the logic I did not write** — lifecycle ordering, thread ownership, terminal
interleaving, and segmentation.

## 12.5 Acceptance evidence

![The accepted session, isolated ROS domain 160. The cancelled leg prints "cancelled
(departing)" and never claims arrival.](figures/preempt_timeline.png){width=94%}

```
t = 0 s   「带我去厨房」              → drives
t = 15 s  「别去了，去客厅」           → PREEMPT; bridge logs "旧 #1 → 新 #2"
t = 41 s  cancelled leg prints       ⏹ 「带我去厨房」已取消（发车）
t = 71 s  living room                ✅ succeeded, 26.9 s, 0.19 m remaining
t = 70 s  「停下」                    → CANCEL, no new goal
t = 78 s  「冰箱在哪」                 → PARALLEL, "fridge is in the kitchen, east, ~3.5 m"
```

Seven criteria, all measured: preemption announced; cancelled leg says "cancelled" and not
"arrived"; new goal `succeeded` (not `failed`); no spurious `failed`; no residual progress
spam after the stop; the cancel clause is not resolved as a place; no false arrival claim.

Because a manual session cannot reliably hit "the robot is mid-corridor at t = 15 s", the run is
driven by a script (`preempt_live.py`) that feeds utterances at fixed times through the
interactive REPL. That is the same reproducibility principle as Chapter 9, applied to a
conversation.

## 12.6 What remains fragile

* **One input stream.** The REPL is the only input; there is no voice, no wake word, no
  barge-in. A second input source would need the same task-per-utterance discipline.
* **Cancellation is cooperative.** It works because every blocking call checks a flag. A new
  blocking call added later must do the same, or bug 2 returns in a new costume.
* **Status is a JSON string.** Adequate, but a typed message plus a sequence field would remove
  the possibility of parsing drift between the two languages (Python and rclpy) that share it.

> **Takeaways.** Preemption is concurrency, not parsing: `to_thread`, one task per utterance, and
> a code-side priority table — all three or nothing. Every goal needs an identity, or stale
> results rewrite live state. The four real bugs were all invisible to offline tests, which is
> why the acceptance procedure includes a scripted live session.
