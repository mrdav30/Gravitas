# Evolving Game Capacity And Soak Plan

**Date:** 2026-10-06  
**Status:** Phase 1 verified. Phase 2 baseline captures and initial 1,024-actor 3D attribution are complete.  
**Owner:** Gravitas benchmark workloads; lower-stack changes only when profiles
justify them.

## Goal And Boundaries

Measure useful physics capacity for a deterministic strategy scene, then use the
same evolving scenes for frame latency and long-session memory evidence. Extend
to an MMO-zone policy only after the first scene exposes a meaningful difference
worth measuring. These are declared synthetic workloads, not proof of game-wide
player counts or observed production contact frequency.

This plan consumes experimental
[GRV-Benchmark-024](benchmark-signal-hardening-backlog.md#grv-benchmark-024--curved-contact-workload-frequency-and-step-cost),
including expensive candidates that lose or reject. It does not reopen closed
query-cost signals. Artifact publishing infrastructure remains owned by the
[existing publishing plan](2026-06-21-benchmark-publishing-and-ccd-diagnostics-plan.md);
an external service or performance CI gate is not a prerequisite for local
evidence.

## Reuse And Measurement Contract

Start with existing
[`BenchmarkPhysicsScene`](../../tests/Gravitas.Benchmarks/Support/BenchmarkPhysicsScene.cs),
[`CircleContactSimulationBenchmarks`](../../tests/Gravitas.Benchmarks/Physics2D/CircleContactSimulationBenchmarks.cs),
[`SimulationAllocationBenchmarks`](../../tests/Gravitas.Benchmarks/Core/SimulationAllocationBenchmarks.cs),
constraint benchmarks and replay fixtures. Add only the support needed for
ordered evolving commands; do not build a general scene framework, navigation
system, networking layer or new public instrumentation API.

Authoritative inputs, actor policies and event schedules use fixed-point values,
fixed frames and owned deterministic seeds. Stopwatch timings and memory probes
stay in benchmark support and never feed commands or simulation decisions.

Report two windows: `Simulate + LateSimulate` including their hooks/coroutines,
and the scenario frame including command application, scheduled queries,
spawn/despawn and host event work. Hashing, metric reduction and artifact output
are identified separately. Preallocate measurement buffers; validate observer
effects with a control that disables measurement while preserving commands.

## Phase 1: One Evolving Strategy Workload

- [x] Declare and verify the 256-actor pure 2D baseline, including static
      obstacles, motion/sleep/grounding policy, command trace, cell size, frame
      rate and solver/CCD settings.
- [x] Run the 1,024-actor tier after the 256-actor layout and work counts
      validate. Record sampled budget overruns and execute all scheduled work.
- [x] Run three stages within the 256-actor scene: mostly sleeping/inactive
      population, sparse commanded movement, and a crowded encounter with
      connected contact islands. Distinguish sleeping registered bodies from
      unregistered actors.
- [x] Include bounded deterministic overlap/raycast queries, projectiles with
      opt-in CCD, scheduled spawn/despawn and wake bursts. Preflight that these
      features actually occur; report query/pair counts and actor roles.
- [x] Add a comparable 256-actor 3D footprint/population/command schedule. Declare
      gravity, ground contacts, shapes and mobility differences; require valid
      physical behavior, not identical hashes or identical contact counts across
      dimensions.
- [x] Verify the same 3D workload at 1,024 actors with four translated regions
      and region-local floor support, CCD, queries and contact chains.
- [x] Verify repeated full-loop replay and command/fixture identity for both
      tiers in both dimensions.

Exit: one reproducible evolving workload, explicit workload assumptions, real
phase transitions and verified work counts in 2D and comparable 3D.

### 256-Actor 2D Checkpoint

Run the [opt-in verifier](../../tests/Gravitas.Benchmarks/README.md#opt-in-evolving-2d-strategy-workload).
[`strategy-2d-256-v1`](../../tests/Gravitas.Benchmarks/Physics2D/Strategy2DWorkload.cs)
uses an explicit fixed-point layout and a frame-indexed command sequence with
no random inputs. The scenario runs at 60 Hz with 8-unit rectangular cells,
unit-mass radius-one actors, manual grounding with no ground support, zero
gravity, sleep enabled, six discrete solver iterations and a four-iteration
TOI budget. Two static boxes supply a projectile barrier and ray targets.
Only the two radius-half projectiles opt into CCD.

- Frames 1–32 keep all 256 registered actors asleep. Every eighth frame across
  the run checks an all-hit ray through the two obstacles and a fixed-area
  overlap that must include the designated scout.
- Frame 33 wakes eight scouts with an impulse. Two actor slots retire on
  frame 40, receive two fresh projectiles on frame 44, and retire again on
  frame 57. Each projectile's free-flight endpoint crosses the barrier. Each
  takes one TOI iteration and stops at the near face with the radius accounted for.
- Frame 65 refills both empty slots and wakes the 24 encounter actors with
  inward impulses. The first complete connected contact chain is on frame 67.

| Stage | Registered actors | Peak awake | Candidate pairs dispatched across stage | Peak adjacent encounter contacts | TOI iterations | Query calls / hits |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Quiet | 256 | 0 | 0 | 0 | 0 | 8 / 12 |
| Sparse | 254–256 | 10 | 26 | 0 | 2 | 8 / 12 |
| Crowded | 256 | 32 | 689 | 23 | 0 | 8 / 12 |

On Windows x64 (Core i7-9700K, SDK 10.0.302, runtime 10.0.10), two Release
processes and one ReleaseLean process each ran two fresh contexts. All 96
authoritative frame hashes, ordered query observations and per-frame work counts
matched within each process. All three processes printed observation digest
`ed0ba077af1d3a4952fbecac69fbcf7c`. The runs used the Phase 1 working changes
on Gravitas `13f73bdc` with released Chronicler 1.0.0, FixedMathSharp 8.0.0,
SwiftCollections 8.0.0 and GridForge 10.0.0 dependencies.

Smoke sampling covered only the complete physics step in the second context
of each process. The Release runs averaged 1.26–1.32 ms across 24 quiet samples,
1.63–1.67 ms across 32 sparse samples and 3.57–3.85 ms across 32 crowded
samples. Current-thread allocation totaled 0, 4,992 and 36,800 bytes in those
windows, respectively, in both profiles. The active stages include first-use
work in the fresh context. Commands, scheduled queries, hashing and observation
reduction were outside the samples. Frame tails, retention and host tick
headroom require the later capture phases.

Package-backed base suites passed 8,348 Release and 8,277 ReleaseLean tests.
Four BenchmarkDotNet dry cases also passed for the shared planar-grid setup
across circle contact, cold registration and partition maintenance.

### 1,024-Actor 2D Checkpoint

`verify-strategy-2d 1024` runs `strategy-2d-1024-v1`. Four copies of the
256-actor region share one world, with origins at (0, 0), (192, 0), (0, 192)
and (192, 192) in X/Z. One grid covers the four regions with the original
8-unit cells. Each region retains the original actor spacing, two static
boxes, command and query schedule, and physics settings. The four encounter
chains preserve local contact density and all complete on frame 67.

| Stage | Registered actors | Peak awake | Candidate pairs dispatched across stage | Peak adjacent encounter contacts | TOI iterations | Query calls / hits |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Quiet | 1,024 | 0 | 0 | 0 | 0 | 32 / 48 |
| Sparse | 1,016–1,024 | 40 | 104 | 0 | 8 | 32 / 48 |
| Crowded | 1,024 | 128 | 2,756 | 92 | 0 | 32 / 48 |

The verifier checked grid membership for every registered actor and
confirmed eight projectile stops and all region-local query targets. All
96 frames matched between fresh contexts. Two Release processes and one
ReleaseLean process printed the same trace `7bece61d4c37790c1c14a89a2fa3db51`
on the Windows x64 host and released package graph used for the baseline.
The 256-actor command still produced its recorded trace in both profiles.
These runs used the scaling changes on Gravitas `9b905b79`.

Across the three 1,024-actor smoke runs, the quiet, sparse and crowded
stage means ranged from 0.95–1.00, 1.40–2.32 and 5.81–7.98 ms per physics
step, respectively. Current-thread managed allocations totaled 0, 20,256
and 147,056 bytes in those windows on each run. One sampled sparse step in
Release and one crowded step in ReleaseLean exceeded 16.67 ms. These samples
include first-use work and omit commands, queries and observation. Use the
later frame and soak captures to assess tick headroom and sustained behavior.

### 256-Actor 3D Checkpoint

[`verify-strategy-3d`](../../tests/Gravitas.Benchmarks/README.md#opt-in-evolving-3d-strategy-workload)
runs `strategy-3d-256-v1`. It preserves the X/Z positions, stage boundaries,
churn, and query cadence of the 256-actor 2D workload. The 3D grid has 8-unit
X/Z cells and 4-unit Y cells. Unit-mass radius-one spheres begin at Y=1 over
a static cuboid with its top at Y=0. Gravity is 9.8 units/s². All translation
axes are free, and the frictionless, zero-restitution contacts support the
actors with ground probes disabled. Linear drag is zero and the speed cap
is 1,024 units/s, so a projectile launched at 900 units/s can cross the barrier
in one step. Only the radius-half projectiles use CCD.

- Every eighth frame casts a spatial ray at Y=4 and runs a projected X/Z
  circle overlap on the designated scout. The query mask excludes the floor.
- At frame 44 both elevated projectiles consume a TOI iteration, reach the
  near-face X with their radius accounted for, and lose their wall-normal
  velocity. Their Y positions decrease under gravity and stay above the floor.
- Every ordinary actor remains within 1/16 unit of its supported center height.
  All eight scouts advance over one unit and have floor contact on frame 64.
  The 24-actor encounter first has all 23 adjacent contacts on frame 67.

| Stage | Registered actors | Peak awake | Touching pairs summed over steps | Peak adjacent encounter contacts | TOI iterations | Query calls / hits |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Quiet | 256 | 0 | 0 | 0 | 0 | 8 / 12 |
| Sparse | 254–256 | 10 | 986 | 0 | 2 | 8 / 12 |
| Crowded | 256 | 32 | 2,430 | 23 | 0 | 8 / 12 |

The 3D pair metric counts each owned pair once per step when it was dispatched
that frame and retains a contact manifold. This includes cached contacts and
can include sleeping actors whose partitions contain awake actors. The 2D
metric counts dispatched candidates, so use each metric to characterize its
own path.

Both package-backed profiles matched all 96 authoritative frame hashes,
ordered query observations and work counts across fresh contexts. Two Release
processes and one ReleaseLean process printed digest
`5956caee8275d7a486c9f551821bcf01` on the same Windows x64 host and released
package graph as the 2D runs, with the 3D changes on Gravitas `fbeea789`.
The two 2D digests also matched their recorded baselines in both profiles after
extracting shared replay and measurement support.

Across the three verified 3D runs, quiet, sparse and crowded smoke means
ranged from 1.64–1.87, 3.23–3.95 and 5.95–8.59 ms per physics step,
respectively. Current-thread allocation totaled 0, 720,024 and 689,584 bytes
in those windows on each run. None of the sampled physics steps exceeded
16.67 ms. The active windows include first-use work and exclude commands,
queries and observation. Allocation attribution and host tick headroom
require later captures.

### 1,024-Actor 3D Checkpoint

`verify-strategy-3d 1024` runs `strategy-3d-1024-v1`. It uses the same
four X/Z region origins as the 1,024-actor 2D tier and one shared grid with
the 256-actor 3D cell metrics. Each region retains its own floor cuboid,
two obstacles, elevated projectile lanes, and original actor spacing and
commands. The four encounter chains remain independent and complete together
on frame 67.

| Stage | Registered actors | Peak awake | Touching pairs summed over steps | Peak adjacent encounter contacts | TOI iterations | Query calls / hits |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Quiet | 1,024 | 0 | 0 | 0 | 0 | 32 / 48 |
| Sparse | 1,016–1,024 | 40 | 3,944 | 0 | 8 | 32 / 48 |
| Crowded | 1,024 | 128 | 9,720 | 92 | 0 | 32 / 48 |

Each active stage dispatched four times the 256-actor touching-pair total.
The verifier confirmed all 32 scouts moved along their own floors, all eight
projectiles reached their walls and lost wall-normal velocity, and every
scheduled query hit its region-local targets. Both package-backed profiles
matched all 96 frames between fresh contexts. Two Release processes and one
ReleaseLean process printed digest `d5e9d21184fbb529b3b3b83e86f2b6b1` on the
same Windows x64 host and released package graph as the smaller tier, with
the 3D working changes on Gravitas `fbeea789`. The three existing tier
digests remained unchanged after sharing the region layout and ordered
command/query dispatch.

The quiet, sparse and crowded smoke means ranged from 3.84–4.25,
7.16–7.53 and 16.21–16.74 ms per physics step. Current-thread managed
allocations totaled 0, 2,859,352 and 2,882,160 bytes across those windows on
each run. The crowded windows exceeded the 16.67 ms interval on 10 and 21
of 32 samples in Release and 15 of 32 in ReleaseLean. Quiet and sparse
samples stayed within the interval, and every scheduled frame ran. The
active windows include first-use work and exclude commands, queries and
observation. These local overruns leave 60 Hz headroom unverified.

## Phase 2: Capacity And Curved-Feature Attribution

- [x] Capture baseline physics-step and scenario-frame distributions for all
      four workloads on a documented host. Profile sparse and crowded stages
      separately for the 1,024-actor 3D tier.
- [ ] Count active/awake population, broad/narrow-phase pairs, solver islands,
      contacts, query candidates and CCD work using available owners. Separate
      dispatched curved queries, root-path entries, admitted candidates and
      final winners when reliable attribution is available.
- [ ] Reuse #024 pose sweeps for geometric envelopes and declare their synthetic
      nature. Include tilted/tumbling cylinder/cone/triangle and capsule/slab
      candidates without implying they dominate an ordinary upright scene.
- [ ] If current counters cannot identify internal root work, justify narrowly
      scoped measurement support and verify unchanged answers. Identify
      instrumented builds separately from clean timing binaries.
- [ ] Vary one meaningful setting at a time around the baseline: cell size,
      actor/collider policy, grounding, solver iterations and CCD. Require
      quality checks alongside throughput; faster incorrect behavior is not a
      viable preset.
- [ ] Establish explicit host tick budgets and physics headroom before labeling
      an actor tier accepted. Publish measured cost curves when no budget is
      chosen; isolated query or batch means do not establish host capacity.

Exit: a capacity characterization and a decision for #024: a documented small
contribution, a measured optimization opportunity, or an explicit remaining
evidence gap. Runtime changes require a separate test-backed measured proposal.

### Baseline Frame Captures

The four verified Phase 1 workloads ran with released Chronicler 1.0.0,
FixedMathSharp 8.0.0, SwiftCollections 8.0.0 and GridForge 10.0.0. The timing
binary used Release `net10.0` on Windows x64 (build 26300, Core i7-9700K,
SDK 10.0.302, runtime 10.0.10) with the Phase 1 and Phase 2 benchmark changes
on Gravitas `fbeea789`. Each tier ran in five fresh processes. Run order
alternated forward and reverse over 2D-256, 2D-1024, 3D-256 and 3D-1024.
All four observation digests matched their Phase 1 baselines.

Build the benchmark project with
`dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net10.0 -p:UseLocalLsfStack=false`.
Set `$env:GRAVITAS_STRATEGY_FRAME_CSV = '1'` in PowerShell and run the four
[verifier commands](../../tests/Gravitas.Benchmarks/README.md#opt-in-evolving-2d-strategy-workload).
Keep separate stdout logs per process. Local raw captures and binary hashes
are under ignored `artifacts/strategy-phase2-clean`.

Two untimed contexts compare all 96 observations. The measured context
checks every query and the terminal observation, leaving the pair and state
scan out of the inter-frame path. Its eight initial quiet frames are omitted.
The timed physics window is `Simulate + LateSimulate`. The scenario window
also includes commands and their inline guards, spawn/despawn, and all
scheduled queries. It contains two allocation-counter reads around the
physics window. Frame-level query validation, hashing, work-count reduction
and output happen afterward. CSV work counters come from the matched
untimed replay. Active stages include first-use work in the fresh context.

Values below are pooled by tier and stage, in milliseconds. Each cell gives
median / p95 / p99 / maximum, with nearest-rank p95 and p99. The quiet stage
has 120 samples and each active stage has 160. The budget is the full 60 Hz
interval of 16.67 ms, before reserving headroom for other host systems.

| Tier | Stage | Physics step | Scenario frame | Over budget, physics / scenario |
| --- | --- | --- | --- | ---: |
| 2D 256 | Quiet | 0.79 / 1.87 / 4.73 / 7.35 | 0.82 / 1.93 / 4.73 / 7.36 | 0 / 0 |
| 2D 256 | Sparse | 0.89 / 2.81 / 31.81 / 42.45 | 0.98 / 4.00 / 31.81 / 42.45 | 2 / 2 |
| 2D 256 | Crowded | 2.47 / 4.01 / 16.22 / 36.87 | 2.53 / 4.73 / 16.22 / 36.87 | 1 / 1 |
| 2D 1,024 | Quiet | 1.30 / 2.38 / 4.23 / 4.56 | 1.42 / 2.61 / 4.23 / 4.56 | 0 / 0 |
| 2D 1,024 | Sparse | 1.60 / 3.35 / 9.35 / 10.14 | 1.63 / 3.86 / 9.42 / 10.21 | 0 / 0 |
| 2D 1,024 | Crowded | 5.08 / 7.17 / 11.62 / 12.19 | 5.12 / 7.17 / 11.62 / 12.19 | 0 / 0 |
| 3D 256 | Quiet | 0.98 / 1.90 / 3.40 / 3.60 | 0.98 / 1.90 / 3.46 / 3.60 | 0 / 0 |
| 3D 256 | Sparse | 2.54 / 4.20 / 12.55 / 13.32 | 2.56 / 4.25 / 12.60 / 13.37 | 0 / 0 |
| 3D 256 | Crowded | 5.80 / 7.25 / 7.78 / 7.95 | 5.84 / 7.36 / 7.78 / 7.96 | 0 / 0 |
| 3D 1,024 | Quiet | 4.19 / 5.68 / 6.08 / 6.15 | 4.20 / 5.68 / 6.08 / 6.15 | 0 / 0 |
| 3D 1,024 | Sparse | 7.62 / 10.04 / 11.18 / 14.43 | 7.66 / 10.05 / 11.18 / 14.43 | 0 / 0 |
| 3D 1,024 | Crowded | 17.13 / 21.98 / 28.61 / 35.59 | 17.18 / 21.98 / 28.61 / 36.16 | 108 / 110 |

The 3D 1,024 crowded scenario missed the interval on 25, 25, 29, 31 and 0
of 32 frames across the five processes. Of its 110 full-frame misses,
108 already exceeded the interval within physics and 94 had no scheduled
query. The paired non-physics contribution had p95 0.33 ms and maximum
0.75 ms. The worst frame was the frame-65 wake burst in run four. It
allocated 2,017,656 of the crowded stage's 2,882,160 physics bytes, while
misses also occurred on frames allocating as little as 48 bytes. All three
2D-256 misses came from one process and are retained as unclassified outliers.
These are local distributions for a fixed schedule, with no accepted host
capacity tier assigned.

### 1,024-Actor 3D Overrun Attribution

Two separate package-backed Release profiles used `dotnet-trace` 10.0.750501
with `collect --profile dotnet-sampled-thread-time --format Speedscope`.
In a disposable copy of the benchmark project, named non-inlined wrappers
around `Simulate + LateSimulate` mark only measured sparse and crowded
physics steps. Filter the Speedscope stacks to each wrapper and sum
`CPU_TIME` weights. Both profiled runs retained the verified replay digest.
The disposable wrapper source, logs and traces are under ignored
`artifacts/strategy-phase2-profile`. These samples are separate from the
baseline timing runs.

| Inclusive path | Sparse CPU sample weight | Crowded CPU sample weight |
| --- | ---: | ---: |
| `PrepareCollisionPartitions` | 34.0–34.3% | 29.6–30.5% |
| `PublishCollisionPositionStates` | 8.9–11.1% | 20.1–21.1% |
| `PartitionVoxel`, nested under those refresh paths | 16.6–18.2% | 32.7–36.2% |
| `CheckAndDistributeCollisions` | 20.0–21.8% | 15.9–16.3% |
| `SolveDiscreteResponsePairs` | 2.9–3.3% | 16.7–18.9% |
| `PrepareContinuousCollisionFrame` | 26.2–28.2% | 11.1–11.8% |

The sparse wrappers carried 225 and 251 ms of sampled CPU weight. The crowded
wrappers carried 515 and 552 ms. The two partition-refresh phases together
accounted for roughly half of crowded sample weight. They refresh moved
colliders before discrete collision and publish corrected poses afterward.
`PartitionVoxel` calls the released GridForge voxel's locked typed partition
lookup for each covered voxel. The profile places the monitor-entry samples
under those refresh paths. The response solver also grows from roughly 3%
of sparse weight to 17–19% in the crowded stage. The crowded frames consumed
zero projectile TOI iterations. The sphere/box fixture does not exercise
the experimental #024 curved-feature cases.

Two additional counter-only runs and one sampled confirmation read process
`Monitor.LockContentionCount` and `GC.CollectionCount(0/1/2)` immediately
around each measured physics step. All three matched the replay digest and
recorded zero new monitor contentions, one Gen0 collection in sparse, and
zero collections in crowded. In the sampled confirmation, monitor entry
still accounted for 40.3% of the crowded CPU sample weight. The counter runs
had zero crowded interval misses. They show that the monitor stack can be
prominent without recorded contention. They do not attribute an individual
baseline tail to lock waits or GC.

Matched configuration and quality comparisons remain open. A cell-size
comparison can test how refresh and pair distribution trade off while
checking floor support, CCD stops, and connected-chain behavior. No runtime
change was made from the sampled attribution.

## Phase 3: Long-Session Latency And Memory

- [ ] Use the same scene and ordered commands for sustained movement/churn,
      collision bursts, wake cascades and quieter recovery periods. Initial
      local capture: at least 10,000 measured frames after explicit warmup; add
      a longer soak when retained-state trends remain unclear. Keep ordinary CI
      smoke short.
- [ ] Report per-frame median, p95, p99, maximum, sample count and burst
      location; distinguish physics-step from scenario-frame times.
      BenchmarkDotNet batch statistics are not substitutes for gameplay-frame
      tails.
- [ ] Record startup allocation/peak memory, warmed steady-state allocations,
      churn allocations, GC activity and live runtime/pool/partition counts.
      Separate expected spawn/capacity growth from unexpected steady-state work.
- [ ] Compare retained managed memory after repeated equal-envelope churn and
      sufficient retirement/cooldown, then after context disposal. Track process
      working set separately; it is not a managed leak oracle. Any forced-GC
      retained-heap probes run outside timed frames in a separately labeled
      pass.
- [ ] Explain bounded high-water retention versus continuing growth; test
      lifecycle ownership if a repeatable leak is found. Do not require an
      artificial zero-allocation startup or OS working-set return to baseline.

Exit: repeatable frame tails and memory plateaus or a reproduced, independently
tracked defect. A clean warmed allocation counter alone cannot close this phase.

## Completion And Product Guidance

- [ ] Publish scenario/command identities, exact settings,
      hardware/runtime/source metadata, timings with uncertainty, memory trends
      and instrumentation limits.
- [ ] Capture an MMO-zone variant only when its actor/activity/query policy adds
      distinct evidence. Measure physics scope without claiming networking,
      persistence, AI or total server capacity.
- [ ] Preserve deterministic results, allocation guarantees and exact 100%
      reachable line/branch/method coverage in Release/Lean for any changed
      library. Use inherited and MSBuild `UseLocalLsfStack=true` for unreleased
      sibling validation; released packages remain a release gate.
- [ ] Feed measured setting tradeoffs into evergreen user guidance, with
      hardware and workload boundaries. Promote confirmed bugs/performance
      concerns into the appropriate tracker; do not add speculative regressions.
- [ ] Archive completed evidence and this plan in `done/`, updating #024 and the
      overview only for the scopes actually measured.
