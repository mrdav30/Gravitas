# Workload And Tuning

Size a Gravitas world against the work it performs during a tick. Registered
bodies determine the population the context manages. Awake bodies, their local
contact density, partition coverage, and scheduled host work determine how much
of that population becomes expensive in a given frame.

## Set The Host Budget

The fixed interval in milliseconds is `1000 / tick rate`. At 60 Hz it is
16.67 ms. If the host reserves 5 ms for work outside the measured scenario,
the scenario has 11.67 ms available. Choose the reserve and the tolerated
miss rate from the application’s own latency requirements.

Measure two windows on the target host:

1. **Physics step.** Time `Simulate()` and `LateSimulate()` together, including
   the hooks and coroutines that execute inside them.
2. **Scenario frame.** Also include ordered command application, actor
   registration and retirement, and scheduled queries. Reserve time only for
   host work outside this window.

Keep wall-clock measurements outside authoritative decisions. Preallocate
sample buffers and record the frame and stage for each miss. During clean
timing runs, defer replay hashes, metric aggregation, and output until after
the timed window. Use a separate pass for expensive observation or profiling
and compare its replay results with the timing control. [Host lifecycle](HOST_INTEGRATION.md#fixed-loop)
and [diagnostics](DIAGNOSTICS.md) describe the available boundaries.

## Describe And Replay The Workload

Record settings, runtime and package versions, hardware, initial state, and the
ordered commands. Include quiet, sparse, and crowded frames, first activation,
and repeated wake, spawn, retirement, and recovery. Keep local spacing and
encounter density explicit when scaling the actor count.

| Record | Why it matters |
| --- | --- |
| Registered and peak awake bodies by mode and mobility | A mostly sleeping population can have very different cost from an equally sized active one. |
| Local density, collider types and extents, covered partitions | Large shapes or tighter cells can increase coverage even with steady actor count. |
| Candidate pairs, touching pairs, connected contacts, and islands | Broad-phase work, retained contacts, and solver work can scale differently. |
| Query type, cadence, and candidate counts, plus CCD bodies and time-of-impact work | A small number of scheduled or fast-moving actors can create their own bursts. |
| First-use and warmed median, p95, p99, maximum, and misses by stage | A pooled percentile can hide an overloaded stage or a synchronized wake burst. |

Verify replay and expected outcomes for each configuration. For example, check
floor support, projectile stops, connected encounters, and ordered query hits.
Changing a setting can legitimately change the trace. Check reproducibility
within that setting and compare behavior against the intended physical bounds.

## Compare Partition Cell Sizes

Start with cells that suit the collider extents and local spacing. Hold the
scene bounds, commands, materials, and solver settings fixed. For a 3D scene
with a floor, compare horizontal X/Z sizes while keeping Y fixed to separate
horizontal coverage and candidate effects. Smaller cells can raise per-body
coverage and lookup costs. Larger cells can increase candidate and contact
work even when partition refresh gets cheaper. Measure both before choosing.
[Collision Broad Phase](COLLISION_BROAD_PHASE.md)

One 60 Hz synthetic strategy scene used pure 2D circles and boxes or pure 3D
spheres and boxes, manual grounding, six discrete solver iterations, and CCD
on the scheduled projectiles. The capture used a Windows x64 Core i7-9700K
and .NET 10 Release.
Two fresh processes per configuration each contributed 10,240 frames after
four complete warmup cycles. The active scene used up to 32 awake actors per
256 registered actors. These measurements provide a reference for similar
scenes on that host.

| Configuration | Warmed scenario p99 | Warmed misses of 16.67 ms |
| --- | ---: | ---: |
| 2D, 256 actors | 1.08 ms | 0 / 20,480 |
| 2D, 1,024 actors | 3.81 ms | 0 / 20,480 |
| 3D, 256 actors | 3.75 ms | 0 / 20,480 |
| 3D, 1,024 actors, X/Z 8 and Y 4 | 15.65 ms | 9 / 20,480 |
| 3D, 1,024 actors, X/Z 16 and Y 4 | 12.28 ms | 3 / 20,480 |

In the 1,024-actor 3D scene, 16-unit cells improved **crowded** warmed p99
from 15.96 to 12.66 ms and retained about 2.9 MiB more managed heap. The first
synchronized wake at 16 units missed 60 Hz in all six fresh-context comparisons,
while the warmed wake bursts fit at both sizes. Four-unit cells missed every
crowded frame in the short comparison. Keep 8 as the reference for this scene.
Compare 16 if sustained crowd cost matters and first-use latency and memory fit
your host budget. At the example 11.67 ms budget, both 1,024-actor 3D settings
exceed the crowded p99 target.

## Tune One Policy At A Time

- **Awake population and density.** Track the work created by simultaneous
  wakes and connected contacts. If the game permits staggered activation, replay
  the new command schedule and check response and gameplay timing. Keep sleeping
  and unregistered populations distinct in the capture.
- **CCD.** Choose continuous modes for bodies whose motion needs a swept
  collision check. Verify supported shapes and motion, then compare
  fast-projectile or thin-obstacle cases against the intended stop positions
  and record time-of-impact work and iteration limits.
  [Continuous Collision Detection](CONTINUOUS_COLLISION_DETECTION.md)
- **Grounding and solver iterations.** Keep the baseline policy and iteration
  count fixed during capacity comparisons. To change either, pair frame costs
  with measurements of support, penetration, settling, drift, or joint error
  on the affected fixtures. Accept a setting only when it meets the host's
  physical error bounds and tick budget. [Collision Response](COLLISION_RESPONSE.md)
- **Queries and shapes.** Measure the actual cadence, filters, and geometry.
  Reuse caller-owned hit buffers and verify ordered results. Curved feature
  evaluation can cost time even when another feature wins or a candidate
  produces no contact. Count dispatch and expensive-path work before using
  observed final contacts to explain cost. [Query Services](QUERY_SERVICES.md)

## Check Long-Session Memory

Run repeated equal-envelope churn in the same context. Record startup,
warmup, and steady allocation separately from spawn and wake bursts.
Track GC activity, registered objects, and active, retained, and pooled
partitions. Compare retained managed memory after equal cooldown boundaries,
after retirement, and after disposal in a separate pass with full collections
outside timed frames. Record process working set separately because it includes
runtime and native memory. A plateau applies to the measured envelope and
duration. Repeat the comparison when actor policy, geometry, target hardware,
or package versions change materially.
