# Evolving Game Capacity And Soak Plan

**Date:** 2026-10-06  
**Status:** Completed 2026-11-09. Phases 1–3 verified and host guidance added.  
**Owner:** Gravitas benchmark workloads; lower-stack changes only when profiles
justify them.

## Goal And Boundaries

Measure useful physics capacity for a deterministic strategy scene, then use the
same evolving scenes for frame latency and long-session memory evidence. Extend
to an MMO-zone policy only after the first scene exposes a meaningful difference
worth measuring. These are declared synthetic workloads, not proof of game-wide
player counts or observed production contact frequency.

This plan consumes experimental
[GRV-Benchmark-024](../benchmark-signal-hardening-backlog.md#grv-benchmark-024--curved-contact-workload-frequency-and-step-cost),
including expensive candidates that lose or reject. It does not reopen closed
query-cost signals. Artifact publishing infrastructure remains owned by the
[existing publishing plan](../2026-06-21-benchmark-publishing-and-ccd-diagnostics-plan.md);
an external service or performance CI gate is not a prerequisite for local
evidence.

## Reuse And Measurement Contract

Start with existing
[`BenchmarkPhysicsScene`](../../../tests/Gravitas.Benchmarks/Support/BenchmarkPhysicsScene.cs),
[`CircleContactSimulationBenchmarks`](../../../tests/Gravitas.Benchmarks/Physics2D/CircleContactSimulationBenchmarks.cs),
[`SimulationAllocationBenchmarks`](../../../tests/Gravitas.Benchmarks/Core/SimulationAllocationBenchmarks.cs),
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

Run the [opt-in verifier](../../../tests/Gravitas.Benchmarks/README.md#opt-in-evolving-2d-strategy-workload).
[`strategy-2d-256-v1`](../../../tests/Gravitas.Benchmarks/Physics2D/Strategy2DWorkload.cs)
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

[`verify-strategy-3d`](../../../tests/Gravitas.Benchmarks/README.md#opt-in-evolving-3d-strategy-workload)
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
- [x] Count active/awake population, broad/narrow-phase pairs, solver islands,
      contacts, query candidates and CCD work using available owners. Separate
      dispatched curved queries, root-path entries, admitted candidates and
      final winners when reliable attribution is available.
- [x] Reuse #024 pose sweeps for geometric envelopes and declare their synthetic
      nature. Include tilted/tumbling cylinder/cone/triangle and capsule/slab
      candidates without implying they dominate an ordinary upright scene.
- [x] If current counters cannot identify internal root work, justify narrowly
      scoped measurement support and verify unchanged answers. Identify
      instrumented builds separately from clean timing binaries.
- [x] Compare X/Z cell sizes with Y fixed and verify replay, floor support,
      projectile stops, and connected chains alongside timings.
- [x] Use the measured workload and quality evidence to select any further
      one-setting comparisons in actor/collider policy, grounding, solver
      iterations or CCD.
- [x] Publish measured cost curves and time remaining in the 60 Hz interval.
      Require a host-specific reserve before accepting an actor tier.

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
[verifier commands](../../../tests/Gravitas.Benchmarks/README.md#opt-in-evolving-2d-strategy-workload).
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

### Matched 1,024-Actor 3D X/Z Cell Sizes

On the baseline Windows x64 host, build package-backed Release `net10.0`
with the benchmark selector change on Gravitas `91382799`. Run
`verify-strategy-3d 1024 4`, `verify-strategy-3d 1024 8` and
`verify-strategy-3d 1024 16` with `GRAVITAS_STRATEGY_FRAME_CSV=1`. The
rectangle bounds, Y edge of 4, region origins, commands and solver settings
are identical. Run six blocks with cell sizes ordered (4, 8, 16),
(16, 8, 4), (8, 16, 4), (4, 16, 8), (16, 4, 8), and (8, 4, 16).
Each launch uses a fresh process and the same timing windows and warmup as
the baseline. Raw CSV output, build hashes and host metadata are in ignored
`artifacts/strategy-phase2-xz-cells`. Use the contemporaneous 8-unit control
for the comparison.

Each cell size verified all 96 replay frames across two fresh contexts in
every process. Each configuration kept the ordinary spheres in their floor
support band and verified all 32 scout floor contacts at frame 64. The eight
projectiles reached their wall stops with zero wall-normal velocity. All four
24-actor encounter chains completed at frame 67 with 92 simultaneous adjacent
contacts. Ordered query observations also passed. Digests identify the
configuration and include its observed work counts. ReleaseLean reproduced
all three digests, and all four original verifier commands kept their
recorded digests.

| X/Z edge | Observation digest | Sparse / crowded touching pairs per launch |
| ---: | --- | ---: |
| 4 | `9f1d6eae83995bd4a3c730f379a2eadb` | 2,408 / 8,184 |
| 8 | `d5e9d21184fbb529b3b3b83e86f2b6b1` | 3,944 / 9,720 |
| 16 | `ef91be67d1a2dad2aa00ffb755e62ab4` | 6,126 / 12,856 |

Values are pooled by cell size and stage, in milliseconds. Each timing
cell gives median / p95 / p99 / maximum. There are 144 samples per quiet
row and 192 per active row. Overruns use the full 16.67 ms interval.

| X/Z edge | Stage | Physics step | Scenario frame | Over budget, physics / scenario |
| ---: | --- | --- | --- | ---: |
| 4 | Quiet | 5.11 / 5.76 / 6.30 / 6.48 | 5.11 / 5.76 / 6.30 / 6.48 | 0 / 0 |
| 4 | Sparse | 12.71 / 14.27 / 14.64 / 14.82 | 12.73 / 14.41 / 15.66 / 15.83 | 0 / 0 |
| 4 | Crowded | 34.76 / 35.52 / 36.30 / 36.52 | 34.78 / 35.58 / 36.30 / 36.53 | 192 / 192 |
| 8 | Quiet | 3.24 / 3.72 / 3.83 / 3.92 | 3.25 / 3.73 / 3.83 / 3.92 | 0 / 0 |
| 8 | Sparse | 6.40 / 7.57 / 7.91 / 8.01 | 6.47 / 7.73 / 8.15 / 8.33 | 0 / 0 |
| 8 | Crowded | 15.38 / 16.17 / 16.73 / 18.63 | 15.44 / 16.17 / 16.73 / 18.63 | 2 / 2 |
| 16 | Quiet | 2.83 / 3.62 / 4.46 / 4.61 | 2.84 / 3.63 / 4.46 / 4.69 | 0 / 0 |
| 16 | Sparse | 5.67 / 7.07 / 8.53 / 8.97 | 5.69 / 7.07 / 8.53 / 9.02 | 0 / 0 |
| 16 | Crowded | 12.22 / 13.62 / 34.34 / 53.71 | 12.27 / 13.63 / 34.68 / 54.04 | 7 / 7 |

The 16-unit cells lowered the crowded scenario median by 2.63–3.38 ms in
all six matched blocks. The frame-65 wake burst missed the interval in
every 16-unit launch. It used 21.23–34.68 ms per scenario frame and allocated
about 5.48 MB in physics, compared with 10.82–11.68 ms and 2.02 MB at 8.
The additional 16-unit miss was frame 72 of the first process, which used
54.04 ms with 48 physics bytes allocated. The 8-unit misses were frames
73 and 81 in one launch. All observations are retained. The earlier
8-unit baseline missed more often, which limits inference about absolute
headroom from the new series.

Two separate profile launches per size used the same EventPipe sampling
method as the initial attribution, with distinct wrappers for frame 65 and
frames 66–96. All four matched their configuration digest. Over frames
66–96, partition preparation plus corrected-position publication fell from
239–253 ms of inclusive sampled CPU weight at 8 units to 113–130 ms at 16.
Collision distribution rose from 68–74 ms to 98–101 ms, alongside the higher
touching-pair count. These are sampled CPU weights from 31 steps per launch.
Each frame-65 wrapper contained only five sample intervals, which leaves the
cause of its allocation and timing spike unresolved. The profile source and
traces are under ignored `artifacts/strategy-phase2-xz-cells/profile`.

Retain 8 as the reference for capacity comparisons. The 16-unit configuration
is a useful contrasting case for the later churn soak because typical
crowded frames are faster and the repeated wake burst is slower. The captures
do not establish an additional setting comparison in actor policy, grounding,
solver iterations or CCD. Keep those settings fixed for the soak and use
physical-quality evidence to select any later tuning comparisons.

### Reference 3D Work Counts

Two untimed replays of the 1,024-actor, 8-unit reference matched the
`d5e9d21184fbb529b3b3b83e86f2b6b1` digest and produced identical counts.
Counts cover all 32 frames per stage, including the quiet frames omitted
from timing. The broad count is the first partition visit of a pair in a
frame, counted after the partition-version deduplication guard in
`PhysicsPartition.ProcessPair`. The narrow count comes from
`CollisionDetection.DoCollisionCheck(CollisionPair)`. Touching pairs include
cached contacts. Island solves count entries into the island response path.
Query candidates sum the service's reported candidate count after each
scheduled ray or overlap.

| Stage | Registered actors | Peak awake | Broad pair visits | Narrow dispatches | Touching pairs | Island solves | Queries / candidates | CCD TOI iterations |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Quiet | 1,024 | 0 | 0 | 0 | 0 | 0 | 32 / 16 | 0 |
| Sparse | 1,016–1,024 | 40 | 27,680 | 1,216 | 3,944 | 1,128 | 32 / 16 | 8 |
| Crowded | 1,024 | 128 | 54,340 | 6,904 | 9,720 | 1,336 | 32 / 16 | 0 |

All stages had zero single-row fast-path solves and zero entries into the
instrumented #024 curved paths. Crowded narrow dispatches grew 5.7× from
sparse while query candidates stayed constant and projectile TOI work ended.
The separate CPU profiles above show where crowded physics spent its time.

### Synthetic Curved-Contact Step Envelopes

Run the opt-in
[`verify-curved-workload`](../../../tests/Gravitas.Benchmarks/README.md#opt-in-curved-contact-step-envelopes)
on the same Windows x64 host and released dependency versions as the baseline.
The clean timing runs used Release `net10.0` on Gravitas `91382799` with
the benchmark changes in this plan. Each of the 16 cases reuses one checked
contact fixture with one dynamic capsule, cylinder or cone and one static
target. The pure 3D cases use a triangle mesh or cylinder. Mixed cases use a
circle or stadium slab. A 16-frame pose cycle uses nine distinct translations
up to 1/4 unit from the authored pose. Tilted cases also sweep up to 4 degrees.
The oblique stadium gap shifts the authored pose by `(0, 1/2, -1/2)` and
slides along `(0, 1, -1)`. Each frame resets pose and motion. This controls
the geometry presented to narrow phase across the sweep.

Each launch uses two untimed 144-frame contexts to compare every state hash,
diagnostic contact count and mixed-candidate count. The measured context
checks terminal state and measures 128 steps after a 16-step warmup with
diagnostics off. The physics window is `Simulate + LateSimulate`. The
scenario window also includes the pose command and allocation measurement
setup. Timing and validation run on the calling thread.

#### Work Counts From A Separate Build

The public diagnostics report emitted contacts. A disposable build counted
earlier work in Gravitas at `91382799` and FixedMathSharp at release tag
`v8.0.0` (`763efc57cea399cde375fe0ef4aa296c1eaa3cba`). Increment
dispatch counters at the 3D pair dispatcher and mixed discrete manifold
dispatcher. Count entries, isolation calls, returned roots, and roots
passing feature filters in FixedMathSharp's `TriangleCylinderEdgeContacts`,
`SegmentConeSurfaceCandidates.Rim`, and the `CylinderCapsuleEllipse` and
`CapsuleSlabRim` parts of `WideConvexPrismRelations`. The stadium reciprocal
isolation counts one batch across both charts and counts roots from each.
For cone face work, separately count calls into
`ConePlaneRayEvents.AccumulateCircles`, actual quadratic parameter solves,
returned branches including conjugate reuse, and branches admitted by
the circle feature. Record owner-level selections at result publication or
rank reduction, as specified below. Reset counters after warmup and compare
the two untimed replay totals. Use clean binaries for timing.

The instrumented build reproduced all 16 clean trace digests and repeated
each counter row exactly. Each row below has 128 narrow dispatches per replay.
The selection columns count roots selected by the geometry owner before
later contact manifold reduction.

| Envelope | Root path entries | Isolation calls | Returned roots | Feature admitted | Selected contact / separation | Contact events |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Cylinder oblique | 128 | 1,920 | 1,144 | 632 | 120 / 0 | 128 |
| Cylinder interior | 128 | 1,920 | 1,032 | 512 | 120 / 0 | 128 |
| Cone oblique, boundary quartic | 128 | 512 | 512 | 128 | See cone ranks below | 128 |
| Cone interior, boundary quartic | 72 | 288 | 576 | 0 | See cone ranks below | 72 |
| Capsule/cylinder oblique | 128 | 128 | 128 | 128 | 128 / 0 | 128 |
| Capsule/circle oblique | 128 | 128 | 128 | 128 | 128 / 0 | 128 |
| Capsule/stadium oblique | 128 | 384 | 512 | 256 | 128 / 0 | 128 |
| Capsule/stadium oblique gap | 128 | 384 | 512 | 256 | 0 / 128 | 0 |

Cylinder cap, cone base, capsule side and straight gap controls had zero
isolation calls and zero cone circle quadratic solves. Both cylinder rim
envelopes had eight frames that isolated roots and emitted contacts without
selecting a root contact. The oblique stadium gap isolated roots on all 128
frames and emitted zero contacts.

Cone oblique and interior each entered circle accumulation 512 times.
Oblique made 4,368 quadratic parameter solves, returned 6,352 branches
and admitted 1,248. Interior made 2,672 solves, returned 3,152 branches and
admitted 552. `MeshConeFaceRegions.Samples` retained 120 and 72 curved
circle-descriptor rows respectively. `MeshConeSurfaceBoundary` retained 32
and zero co-minimal rim candidates at boundary ranking, before final
manifold reduction. The interior case admitted no quartic roots even on
the frames with a contact. Contact-event counts alone miss this work.

#### Clean Frame Costs

All six fresh Release processes and the ReleaseLean check matched the 16
trace digests. The Release timing ranges below give the minimum and maximum
of each launch's mean / p95 / p99 / maximum in milliseconds, using 128
samples per case per launch. Overruns sum all six launches, using the
16.67 ms interval. The clean Release and Lean runs each measured zero
current-thread physics bytes in every case. Raw logs, binary hashes and
instrumented source snapshots are under ignored `artifacts/strategy-phase2-curved`.

| Envelope | Physics step | Scenario frame | Overruns, physics / scenario |
| --- | --- | --- | ---: |
| Cylinder cap | 0.555–0.842 / 0.625–1.603 / 0.851–6.279 / 0.895–6.460 | 0.617–1.054 / 0.701–3.756 / 0.910–6.606 / 1.008–7.877 | 0 / 0 |
| Cylinder oblique | 1.330–3.337 / 1.571–4.434 / 2.199–7.567 / 2.632–9.251 | 1.374–3.415 / 1.609–4.515 / 2.245–7.621 / 2.676–9.329 | 0 / 0 |
| Cylinder interior | 1.185–1.375 / 1.330–2.099 / 1.445–2.607 / 1.459–5.228 | 1.220–1.418 / 1.370–2.139 / 1.486–2.632 / 1.495–5.268 | 0 / 0 |
| Cylinder gap | 0.043–0.059 / 0.050–0.071 / 0.060–0.126 / 0.061–0.791 | 0.055–0.079 / 0.062–0.093 / 0.079–0.166 / 0.082–0.811 | 0 / 0 |
| Cone base | 0.213–0.355 / 0.263–0.583 / 0.340–1.818 / 0.341–1.983 | 0.237–0.413 / 0.308–0.655 / 0.401–1.854 / 0.461–2.019 | 0 / 0 |
| Cone oblique | 1.980–4.731 / 3.694–17.954 / 3.837–20.264 / 3.888–27.986 | 2.036–4.843 / 3.751–18.024 / 3.897–20.337 / 3.948–28.057 | 8 / 8 |
| Cone interior | 0.912–1.293 / 1.737–3.145 / 1.989–8.038 / 2.023–11.066 | 0.970–1.454 / 1.794–3.718 / 2.054–9.811 / 2.090–11.139 | 0 / 0 |
| Cone gap | 0.028–0.068 / 0.029–0.186 / 0.035–0.713 / 0.036–0.842 | 0.046–0.093 / 0.047–0.208 / 0.052–0.733 / 0.054–0.864 | 0 / 0 |
| Capsule/cylinder side | 0.036–0.182 / 0.042–0.090 / 0.046–6.446 / 0.060–11.586 | 0.047–0.195 / 0.055–0.103 / 0.062–6.457 / 0.073–11.598 | 0 / 0 |
| Capsule/cylinder oblique | 0.809–1.128 / 0.902–1.713 / 1.024–4.790 / 1.529–24.073 | 0.834–1.162 / 0.925–1.739 / 1.045–4.819 / 1.553–24.099 | 1 / 1 |
| Capsule/circle side | 0.061–0.063 / 0.066–0.077 / 0.072–0.128 / 0.074–0.164 | 0.077–0.081 / 0.084–0.113 / 0.089–0.155 / 0.102–0.181 | 0 / 0 |
| Capsule/circle oblique | 0.707–1.792 / 0.781–7.197 / 1.163–10.568 / 1.525–47.469 | 0.741–1.854 / 0.848–7.234 / 1.195–10.606 / 1.558–47.511 | 1 / 1 |
| Capsule/stadium side | 0.058–0.119 / 0.064–0.202 / 0.086–2.167 / 0.090–2.345 | 0.073–0.142 / 0.081–0.296 / 0.105–2.186 / 0.106–2.363 | 0 / 0 |
| Capsule/stadium oblique | 0.630–1.207 / 0.699–4.072 / 0.757–6.793 / 0.768–9.769 | 0.665–1.256 / 0.744–4.107 / 0.793–6.832 / 0.846–9.805 | 0 / 0 |
| Capsule/stadium oblique gap | 0.528–0.561 / 0.611–0.718 / 0.727–1.193 / 0.733–1.910 | 0.576–0.613 / 0.685–0.804 / 0.821–1.260 / 0.832–1.956 | 0 / 0 |
| Capsule/stadium gap | 0.072–0.122 / 0.085–0.223 / 0.087–0.956 / 0.088–4.929 | 0.095–0.160 / 0.110–0.340 / 0.114–0.986 / 0.121–4.954 | 0 / 0 |

All eight cone oblique overruns came from one process. Its scenario p95 was
18.024 ms. The other five ranged from 3.751 to 4.408 ms. The two oblique
capsule misses came from different processes and each occurred once. Keep
these tails in the record. The launches identify expensive poses and
reproducible work counts, while the cause of the run-to-run tails remains
unresolved.

### Observed Tick Headroom And Phase 2 Exit

Use the larger of the sparse and crowded scenario percentiles and subtract
it from 16.67 ms. Each percentile can come from a different stage. Negative
values show the amount by which the observed percentile exceeded the
interval. The table leaves any reserve for application systems to the host.

| Capture | Scenario p95 / p99 (ms) | Time remaining at p95 / p99 (ms) | Active scenario overruns |
| --- | ---: | ---: | ---: |
| 2D 256 baseline | 4.73 / 31.81 | +11.94 / −15.14 | 3 / 320 |
| 2D 1,024 baseline | 7.17 / 11.62 | +9.50 / +5.05 | 0 / 320 |
| 3D 256 baseline | 7.36 / 12.60 | +9.31 / +4.07 | 0 / 320 |
| 3D 1,024 baseline, X/Z 8 | 21.98 / 28.61 | −5.31 / −11.94 | 110 / 320 |
| 3D 1,024 matched, X/Z 4 | 35.58 / 36.30 | −18.91 / −19.63 | 192 / 384 |
| 3D 1,024 matched, X/Z 8 | 16.17 / 16.73 | +0.50 / −0.06 | 2 / 384 |
| 3D 1,024 matched, X/Z 16 | 13.63 / 34.68 | +3.04 / −18.01 | 7 / 384 |

The 1,024-actor 3D reference remains a stress tier. Its p99 exhausted the
interval in both series, and the lower p95 at 16 units came with a larger
wake-burst tail. Host acceptance still requires an application reserve and
the longer churn capture.

#024 remains experimental. The ordinary scene's overruns occur with zero
work in the instrumented curved paths. The isolated sweeps establish that
oblique contacts and rejections can consume meaningful step time when
present. A representative scene or host trace must establish dispatch
frequency, local density and contribution to its budget before choosing a
geometry optimization. Carry the reference and the 16-unit contrast into
Phase 3 for sustained tails and retained memory.

## Phase 3: Long-Session Latency And Memory

- [x] Use the same scene and ordered commands for sustained movement/churn,
      collision bursts, wake cascades and quieter recovery periods. Initial
      local capture: at least 10,000 measured frames after explicit warmup; add
      a longer soak when retained-state trends remain unclear. Keep ordinary CI
      smoke short.
- [x] Report per-frame median, p95, p99, maximum, sample count and burst
      location; distinguish physics-step from scenario-frame times.
      BenchmarkDotNet batch statistics are not substitutes for gameplay-frame
      tails.
- [x] Record startup allocation/peak memory, warmed steady-state allocations,
      churn allocations, GC activity and live runtime/pool/partition counts.
      Separate expected spawn/capacity growth from unexpected steady-state work.
- [x] Compare retained managed memory after repeated equal-envelope churn and
      sufficient retirement/cooldown, then after context disposal. Track process
      working set separately; it is not a managed leak oracle. Any forced-GC
      retained-heap probes run outside timed frames in a separately labeled
      pass.
- [x] Explain bounded high-water retention versus continuing growth; test
      lifecycle ownership if a repeatable leak is found. Do not require an
      artificial zero-allocation startup or OS working-set return to baseline.

Exit: repeatable frame tails and memory plateaus or a reproduced, independently
tracked defect. A clean warmed allocation counter alone cannot close this phase.

### Soak Protocol

Use the package-backed Release build and the same host as Phase 2. The
[`soak-strategy-2d` and `soak-strategy-3d` commands](../../../tests/Gravitas.Benchmarks/README.md#opt-in-evolving-strategy-soak)
reuse the original strategy scene and commands in one owned context.
Run 2D and 3D at both actor tiers and compare 8-unit and 16-unit X/Z cells
at 1,024 actors in 3D with Y fixed at 4. The ordered command cycle is
`strategy-{dimension}-{actors}[-xz16]-soak-v1`. Frames 1–96 repeat the
original quiet, sparse and crowded sequence. On frame 97 the 24 encounter
actors and eight scouts in each region reset to their authored poses and
sleep. Frames 98–128 give a quiet recovery. The query cadence remains one
ordered ray and overlap per region every eight frames. The existing
projectile retirement, fire and replacement schedule runs every cycle.
Use four complete warmup cycles (512 frames) and 80 measured cycles
(10,240 frames) per fresh process.

Use two timing runs in forward and reverse configuration order. Set
`GRAVITAS_STRATEGY_FRAME_CSV=1` and keep stdout from each fresh process
separately. The physics window covers `Simulate + LateSimulate`. The
scenario window also includes commands, rearm, scheduled queries and the
physics allocation reads. Clock guards, query checks and hashes, the
terminal state hash, metric reduction and output are outside timed
windows. Per-frame allocations cover the stepping thread. Process-wide
collection deltas cover only the measured interval.

Use two separate fresh processes per configuration with
`GRAVITAS_STRATEGY_SOAK_HEAP=1`. In each cycle validate registration and
floor support at the selected milestones, the sparse wake and movement,
the CCD stops, and all encounter chains on phase frame 67. Compare
ordered query and terminal state digests across timing and heap runs.
Compare full state and work checkpoint digests between the heap runs.
Hashing every frame would add an all-pairs registry scan. The soak hashes
full state at selected checkpoints in the untimed run and once at the end
of each timed run.

Force full GC only in the heap probes. Sample before context creation,
after warmup and every 20 measured cycles at the same post-recovery phase.
After the last cycle clear the probe's replay snapshot and caller query
buffers, retire all dynamic bodies, and step another 1,024 quiet frames.
That exceeds the default 320-frame empty partition lifetime and leaves
time for the retirement sweep. The static scene stays registered until
disposal. Sample after the context and actor arrays leave scope. Report
active and retained partition payloads separately from the detached
partition pool. Retained payloads can include occupied partitions.
Working set and process peak working set are recorded independently of
managed heap. Raw logs, host metadata and binary hashes are under ignored
`artifacts/strategy-phase3`.

### Warmed Frame Tails

The Phase 3 benchmark changes on Gravitas `c820b8ee` used Windows x64
build 26300 on a Core i7-9700K, SDK 10.0.302 and runtime 10.0.10.
The build resolved Chronicler 1.0.0, FixedMathSharp 8.0.0,
SwiftCollections 8.0.0 and GridForge 10.0.0. Values below pool the
two timing launches per setting, 20,480 measured frames each, with
median / nearest-rank p95 / p99 / maximum in milliseconds.

| Configuration | Physics step | Scenario frame | 60 Hz misses, physics / scenario |
| --- | --- | --- | ---: |
| 2D 256 | 0.197 / 0.761 / 1.060 / 4.038 | 0.211 / 0.858 / 1.084 / 4.085 | 0 / 0 |
| 2D 1,024 | 0.877 / 3.258 / 3.653 / 7.456 | 0.958 / 3.396 / 3.806 / 8.709 | 0 / 0 |
| 3D 256 | 1.370 / 3.590 / 3.728 / 7.230 | 1.415 / 3.600 / 3.745 / 7.230 | 0 / 0 |
| 3D 1,024, X/Z 8 | 6.229 / 15.316 / 15.587 / 24.105 | 6.282 / 15.380 / 15.646 / 24.105 | 8 / 9 |
| 3D 1,024, X/Z 16 | 5.293 / 11.858 / 12.215 / 28.483 | 5.338 / 11.919 / 12.280 / 28.484 | 3 / 3 |

Compare the crowded stage independently of the recovery and quiet samples.
Each row below has 5,120 crowded samples and 160 phase-65 wake bursts.
Six and three crowded scenario misses occurred at 8 units in the two
launches. Two occurred at 16 units in launch one and zero in launch two,
which had one sparse miss.

| X/Z edge | Crowded scenario frame | Phase 65 scenario frame | Worst scenario frame (launch / cycle / phase) |
| ---: | --- | --- | --- |
| 8 | 15.182 / 15.609 / 15.958 / 24.105 | 12.929 / 13.478 / 15.103 / 15.572 | 1 / 48 / 74 |
| 16 | 11.700 / 12.218 / 12.664 / 28.484 | 9.440 / 10.056 / 10.566 / 11.041 | 1 / 24 / 83 |

The 16-unit setting improved crowded p99 in both warmed launches, with
per-launch p99s of 12.71 and 12.66 ms versus 15.98 and 15.96 ms at 8 units.
Every measured wake burst fit the interval in both settings. Phase 2's
fresh-context wake bursts at 16 units still exceeded it in all six runs.
The soak establishes a warmed setting tradeoff. Keep 8 as the reference
and carry both startup and warmed costs into guidance for this scene.

### Allocation And Collection Activity

Startup allocation is the stepping-thread delta from before context setup to
initial scene registration, excluding the preallocated frame sample buffer.
It includes small measurement and output overhead. Process peak working
set includes the runtime and process setup. These ranges are from the two
clean timing launches.

| Configuration | Startup / warmup allocated (MiB) | Peak working set at startup / measured end (MiB) | Measured GC counts, gen0 / gen1 / gen2 per launch |
| --- | ---: | ---: | ---: |
| 2D 256 | 2.48 / 0.41 | 35.79–35.98 / 54.21–54.45 | 0 / 0 / 0 |
| 2D 1,024 | 8.65 / 1.59 | 44.29–44.33 / 67.21–67.45 | 2 / 1 / 0 |
| 3D 256 | 11.24 / 1.93 | 47.56–47.79 / 68.55–68.89 | 1 / 0 / 0 |
| 3D 1,024, X/Z 8 | 41.47 / 7.84 | 85.68–86.41 / 107.36–107.57 | 7 / 2 / 1 |
| 3D 1,024, X/Z 16 | 34.88 / 17.17 | 69.14–69.24 / 110.96–111.14 | 8 / 2 / 1 |

The projectile spawn on phase 44 and normal replacement on phase 65 account
for most scenario allocation. At 1,024 in 3D, those commands and steps cost
about 0.49 MiB per cycle together at either cell size. The warmed physics
step on phase 65 had median 0 bytes at 8 units and 2,400 bytes at 16, down
from about 2.02 MB and 5.48 MB respectively in the fresh-context cell-size
comparison. Quiet and recovery allocation and the smaller residual crowded
allocation are separately identifiable in the CSV. A collection count for
the whole interval cannot identify an individual frame's cause.

### Retained Managed Memory

The forced-collection probes sample the same recovery boundary after each
20-cycle block. Heap sizes below are process managed totals in MiB. The
partition figures are active / retained / pooled. The retired rows keep
two planar obstacles or three spatial statics per region registered. All
dynamic bodies and active dynamic partitions are gone.

| Configuration | Managed heap at cycle 80 / retired / disposed (MiB) | Partitions at cycle 80 | Partitions after retirement |
| --- | ---: | ---: | ---: |
| 2D 256 | 2.96 / 2.96 / 0.36 | 240 / 246 / 0 | 0 / 6 / 240 |
| 2D 1,024 | 9.85 / 9.85 / 0.36 | 960 / 984 / 0 | 0 / 24 / 960 |
| 3D 256 | 12.17 / 6.18 / 0.36 | 975 / 1,896 / 0 | 0 / 1,893 / 3 |
| 3D 1,024, X/Z 8 | 45.13 / 19.87 / 0.36 | 3,900 / 7,287 / 0 | 0 / 7,275 / 12 |
| 3D 1,024, X/Z 16 | 48.02 / 25.14 / 0.36 | 1,248 / 1,935 / 0 | 0 / 1,923 / 12 |

All five envelopes plateaued. In each of the ten heap runs the difference
between the highest and lowest checkpoint from cycles 20–80 was at most
192 bytes, with identical partition counts. The 16-unit 3D setting uses
fewer payloads and retained about 2.9 MiB more managed heap
in the active scene than 8 units. The 3D heap fell after dynamic retirement.
In 2D, the dynamic partitions moved into the pool and the heap stayed
nearly level. A one-actor diagnostic isolated another owner of that
retention. The grounding snapshot held the retired body, collider and
host agent after 1,024 empty steps. Clearing that scratch buffer released
all three in three fresh processes. The no-populated-step control released
all three without intervention. The finding is separately tracked as
[GRV-Issue-149](../issue-tracker.md#grv-issue-149). The probe establishes an
actor reference root and leaves the byte share of each heap owner open.

Disposal brought managed memory near the pre-context process baseline.
Working set remained above its startup level. These results characterize
bounded high-water retention over this workload and duration. A longer
run is warranted if a different envelope exhibits continuing growth.

### Phase 3 Replay Evidence

Replay digests cover 10,752 frames including warmup. Heap passes also
compare 588 per-launch checkpoints, taken at phase frames 1, 33, 44, 64,
65, 67 and 128. Those checks exercise the stage changes, floor support,
projectile stops and connected chains. Each timing and heap run verifies
the ordered query results throughout the schedule.

| Configuration | Ordered queries | Terminal state | Heap checkpoint digest |
| --- | --- | --- | --- |
| 2D 256 | `0122df9fc4f2fc943f27ba040296a566` | `a8f43241ca42775712e00d8edeec02f4` | `fc34516a812f91a966959f9d604baf47` |
| 2D 1,024 | `9d180902d02eac4b0f62aefae135de96` | `e9259fb482b81b1acf3e30b2dd6e6516` | `d354a57e7ef19ce3f3ba20070960977e` |
| 3D 256 | `752138d308f91e8f2108bda3c3b632a8` | `ddc7d29b0a0075f893ad019053347100` | `642f0c900722d4643aacbfb0b8e821e0` |
| 3D 1,024, X/Z 8 | `d2462dee7488e6bfe2cb4502a852b5dd` | `389b5f9598e1614058e35c7540ece669` | `c519249fd878115c76e8e0240d230fd5` |
| 3D 1,024, X/Z 16 | `80159a87b1d3c32eb8fda881e0c9e642` | `5dc7ec311c06b1c2735cf77b2df088cb` | `7c20409df39dc9eedb63fd65d0e90c55` |

The package-backed Release and ReleaseLean builds passed. One measured
cycle after warmup reproduced the Release smoke replay and checkpoint
digests in Lean for all five settings. The original four strategy traces,
the 16-unit comparison trace and all 16 curved envelopes kept their
recorded digests. Default BenchmarkDotNet discovery excludes the soak.

## Completion And Product Guidance

- [x] Record scenario/command identities, exact settings,
      hardware/runtime/source metadata, timings with uncertainty, memory trends
      and instrumentation limits.
- [x] Defer the MMO-zone variant until a distinct actor/activity/query policy
      warrants another capture. The current physics measurements cover the
      declared strategy scene.
- [x] Verify released-package replay and warm allocations for the measured
      workloads and the separately tracked lifetime fixes.
- [x] Publish the measured setting tradeoffs and host acceptance method in
      [Workload And Tuning](../../api/guides/WORKLOAD_TUNING.md).
- [x] Archive the plan and preserve the open benchmark follow-ups in their
      owning backlog.

The 2D and 3D lifetime findings were resolved as
[#149](../issue-tracker.md#grv-issue-149) and [#150](../issue-tracker.md#grv-issue-150).
Their package-backed Release and ReleaseLean validation preserved the
strategy traces and warm allocation baselines. The retained-memory and timing
tables above describe the measured revisions, including the pre-fix heap
probes. The guidance closeout changes documentation only.

Keep 8-unit X/Z cells as the 1,024-actor 3D reference. Evaluate 16 units
against the host's first-use, warmed, and memory budgets. The profile evidence
and cold wake spike continue under
[#033](../benchmark-signal-hardening-backlog.md#grv-benchmark-033).
Representative curved-contact frequency continues under
[#024](../benchmark-signal-hardening-backlog.md#grv-benchmark-024--curved-contact-workload-frequency-and-step-cost).
The unclassified 2D-256 outliers came from one process and did not recur in
the warmed soak. The [benchmark watch list](../benchmark-signal-hardening-backlog.md#watch-items)
retains the matched fresh-context recurrence check.

Host capacity still depends on the application's tick reserve and workload.
Solver iteration, grounding, and frame-rate quality curves belong to the
[Long-Duration Physics Quality And Tuning plan](2026-10-06-long-duration-physics-quality-and-tuning-plan.md).
