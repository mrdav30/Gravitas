# Evolving Game Capacity And Soak Plan

**Date:** 2026-10-06  
**Status:** Proposed; queued after complete replay stepping and shared trace format.  
**Owner:** Gravitas benchmark workloads; lower-stack changes only when profiles justify them.

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
an external service or performance CI gate is not a prerequisite for local evidence.

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

- [ ] Declare a baseline of pure 2D circle actors and static obstacles, actor
      motion/sleep/grounding policy, command trace, cell size, frame rate and
      solver/CCD settings. Use 256 and 1,024 actors initially; extend scale only
      after the layout and work counts validate. Record budget overruns rather
      than silently dropping physics work.
- [ ] Run three stages within the scene: mostly sleeping/inactive population,
      sparse commanded movement, and a crowded encounter with connected contact
      islands. Distinguish sleeping registered bodies from unregistered actors.
- [ ] Include bounded deterministic overlap/raycast queries, projectiles with
      opt-in CCD, scheduled spawn/despawn and wake bursts. Preflight that these
      features actually occur; report query/pair counts and actor roles.
- [ ] Add a comparable 3D footprint/population/command schedule. Declare gravity,
      ground contacts, shapes and mobility differences; require valid physical
      behavior, not identical hashes or identical contact counts across dimensions.
- [ ] Verify repeated full-loop replay and command/fixture identities before
      using the timing results to justify a refactor.

Exit: one reproducible evolving workload, explicit workload assumptions, real
phase transitions and verified work counts in 2D and comparable 3D.

## Phase 2: Capacity And Curved-Feature Attribution

- [ ] Capture clean complete-frame distributions and matched configuration
      comparisons on a documented host; profile stages separately.
- [ ] Count active/awake population, broad/narrow-phase pairs, solver islands,
      contacts, query candidates and CCD work using available owners. Separate
      dispatched curved queries, root-path entries, admitted candidates and final
      winners when reliable attribution is available.
- [ ] Reuse #024 pose sweeps for geometric envelopes and declare their synthetic
      nature. Include tilted/tumbling cylinder/cone/triangle and capsule/slab
      candidates without implying they dominate an ordinary upright scene.
- [ ] If current counters cannot identify internal root work, justify narrowly
      scoped measurement support and verify unchanged answers. Identify
      instrumented builds separately from clean timing binaries.
- [ ] Vary one meaningful setting at a time around the baseline: cell size,
      actor/collider policy, grounding, solver iterations and CCD. Require quality
      checks alongside throughput; faster incorrect behavior is not a viable preset.
- [ ] Establish explicit host tick budgets and physics headroom before labeling
      an actor tier accepted. Publish measured cost curves when no budget is
      chosen; isolated query or batch means do not establish host capacity.

Exit: a capacity characterization and a decision for #024: a documented small
contribution, a measured optimization opportunity, or an explicit remaining
evidence gap. Runtime changes require a separate test-backed measured proposal.

## Phase 3: Long-Session Latency And Memory

- [ ] Use the same scene and ordered commands for sustained movement/churn,
      collision bursts, wake cascades and quieter recovery periods. Initial local
      capture: at least 10,000 measured frames after explicit warmup; add a longer
      soak when retained-state trends remain unclear. Keep ordinary CI smoke short.
- [ ] Report per-frame median, p95, p99, maximum, sample count and burst location;
      distinguish physics-step from scenario-frame times. BenchmarkDotNet batch
      statistics are not substitutes for gameplay-frame tails.
- [ ] Record startup allocation/peak memory, warmed steady-state allocations,
      churn allocations, GC activity and live runtime/pool/partition counts.
      Separate expected spawn/capacity growth from unexpected steady-state work.
- [ ] Compare retained managed memory after repeated equal-envelope churn and
      sufficient retirement/cooldown, then after context disposal. Track process
      working set separately; it is not a managed leak oracle. Any forced-GC
      retained-heap probes run outside timed frames in a separately labeled pass.
- [ ] Explain bounded high-water retention versus continuing growth; test
      lifecycle ownership if a repeatable leak is found. Do not require an
      artificial zero-allocation startup or OS working-set return to baseline.

Exit: repeatable frame tails and memory plateaus or a reproduced, independently
tracked defect. A clean warmed allocation counter alone cannot close this phase.

## Completion And Product Guidance

- [ ] Publish scenario/command identities, exact settings, hardware/runtime/source
      metadata, timings with uncertainty, memory trends and instrumentation limits.
- [ ] Capture an MMO-zone variant only when its actor/activity/query policy adds
      distinct evidence. Measure physics scope without claiming networking,
      persistence, AI or total server capacity.
- [ ] Preserve deterministic results, allocation guarantees and exact 100%
      reachable line/branch/method coverage in Release/Lean for any changed
      library. Use inherited and MSBuild `UseLocalLsfStack=true` for unreleased
      sibling validation; released packages remain a release gate.
- [ ] Feed measured setting tradeoffs into evergreen user guidance, with hardware
      and workload boundaries. Promote confirmed bugs/performance concerns into
      the appropriate tracker; do not add speculative regressions.
- [ ] Archive completed evidence and this plan in `done/`, updating #024 and the
      overview only for the scopes actually measured.
