# Evolving Game Capacity And Soak Plan

**Date:** 2026-10-06  
**Status:** Phase 1 in progress. The opt-in 256-actor 2D workload is verified.  
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
- [ ] Run the 1,024-actor tier after the 256-actor layout and work counts
      validate. Record budget overruns rather than silently dropping physics work.
- [x] Run three stages within the 256-actor scene: mostly sleeping/inactive
      population, sparse commanded movement, and a crowded encounter with
      connected contact islands. Distinguish sleeping registered bodies from
      unregistered actors.
- [x] Include bounded deterministic overlap/raycast queries, projectiles with
      opt-in CCD, scheduled spawn/despawn and wake bursts. Preflight that these
      features actually occur; report query/pair counts and actor roles.
- [ ] Add a comparable 3D footprint/population/command schedule. Declare
      gravity, ground contacts, shapes and mobility differences; require valid
      physical behavior, not identical hashes or identical contact counts across
      dimensions.
- [x] Verify repeated full-loop replay and command/fixture identity for the
      256-actor 2D workload.

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

## Phase 2: Capacity And Curved-Feature Attribution

- [ ] Capture clean complete-frame distributions and matched configuration
      comparisons on a documented host; profile stages separately.
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
