# Long-Duration Physics Quality And Tuning Plan

**Date:** 2026-10-06  
**Status:** Proposed; queued after initial evolving-workload characterization.  
**Owner:** Gravitas contact/constraint quality and configuration guidance.

## Goal

Publish how existing settings trade physical quality for cost over meaningful
simulation durations. Reuse current deterministic contact/island/constraint
owners; add solver behavior or public tuning only when evidence establishes a
specific limitation. This is characterization and hardening, not a presumption
that the current solver needs replacement.

## Existing Foundation

The completed
[discrete-response plan](done/2026-06-21-discrete-response-and-contact-quality-hardening-plan.md)
and [3D stress plan](done/2026-07-02-3d-constraint-solver-stress-and-tuning-hardening-plan.md)
cover warm starts, islands, resting contacts, chain/ragdoll stress and deterministic
joint metrics. Reuse
[`Constraint3DStressTests`](../../tests/Gravitas.Tests/Constraints/Constraint3DStressTests.cs),
the dimensional constraint tests, sleep tests and
[`Constraint3DBenchmarks`](../../tests/Gravitas.Benchmarks/Core/Constraint3DBenchmarks.cs).
The [capacity/soak plan](2026-10-06-evolving-game-capacity-and-soak-plan.md) owns
frame-time/memory collection; share its measurement contract rather than add a
second runner or generic physics-quality framework.

## Phase 1: Physical Fixtures And Metrics

- [ ] Add or extend freely simulated stacks without hidden axis locks or joints
      that manufacture stability. Include ordinary primitives and an explicit
      tilted/curved-feature stress case. Extend long chains and resting/contact
      ragdolls from existing fixtures instead of rebuilding them.
- [ ] Cover equal and varied mass ratios, initially 1:1, 1:10 and 1:100, with
      fixed documented lengths/masses and an admitted fixed-point range. Cover
      pure 2D and 3D equivalents and focused mixed-response parity where the
      constrained model applies; do not demand 3D behavior from planar bodies.
- [ ] Measure maximum/residual penetration, positional/angular drift, joint
      anchor/limit error, contact identity churn and sleep/wake transitions over
      time. Report units and normalize geometric errors to declared shape scale.
- [ ] Track kinetic/potential energy and work from gravity, impulses and motors
      with the fixture's friction, restitution and damping policy. Conservation
      is a gate only in an appropriate conservative fixture; resting friction,
      damping and stabilization need their own explainable behavior bounds.
- [ ] Keep physical comparisons deterministic and overflow-safe. Reuse neutral
      upstream math when needed; do not move rigid-body policy into FixedMathSharp.
      Wall-clock measurements and chart formatting remain outside physics logic.

Each fixture declares its expected physical behavior and numeric acceptance
bounds before an optimization is accepted. Baseline observations help choose
scale-aware bounds; an unstable baseline does not become correct by definition.

## Phase 2: Duration And Continuation

- [ ] Measure settling, sustained rest and repeated controlled disturbances over
      equal simulated durations at different frame rates. Begin with 32 and 60
      Hz; use a shorter regression plus an extended local characterization, not
      a huge soak in every unit test.
- [ ] Exercise repeated sleeping/waking, connected wake propagation, runtime
      mass changes between complete steps and deterministic spawn/despawn.
      Verify the scene still contains the intended dynamic bodies and contacts.
- [ ] Compare repeated command traces, then supported save/populate continuation
      into host-created matching shells and a matching clock timeline.
- [ ] Respect the [serialization contract](../wiki/SERIALIZATION.md): body records
      do not restore a world, arbitrary host/coroutine state or solver caches.
      Compare exact traces only where continuation state is equivalent. If
      population invalidates caches, use an equivalently cold-cache control and
      separately measure the recovery transient against uninterrupted simulation.
      Do not disguise a genuine supported-contract divergence as a quality tolerance.

Exit: long-duration quality curves, repeatable disturbances and supported
continuation evidence. Record failures as focused issues with reproducing tests;
repair their owning layer before calling an affected configuration acceptable.

## Phase 3: Quality Versus Cost

- [ ] Compare current default solver settings with a lower/higher iteration
      setting, initially 2/6/12 iterations where supported. Add tuning dimensions
      only when a fixture identifies a meaningful sensitivity.
- [ ] Report quality and measured complete-step time together at each setting,
      including awake population, contact/island counts and relevant CCD work.
      Changing frame rate requires equal simulated duration, not equal frame count.
- [ ] Separate contact and joint metrics; distinguish physical effects from
      partition/query cost and presentation interpolation. Keep engine adapters
      outside the experiment.
- [ ] Retain a solver change only with a reproduced physical improvement,
      deterministic ordering/rounding, bounded resource behavior, benchmark
      comparison and no unexplained regression in other dimensions/features.

Exit: a small set of measured, usable configurations or a scoped proposal for
the demonstrated solver limitation. Do not add named public presets, compliance
APIs, clamping or alternate penetration kernels solely to fill a tuning table.

## Completion And Publication

- [ ] Publish reproducible settings and quality/time curves, limitations and
      recommended usage boundaries. Feed lasting guidance into the relevant wiki
      pages without plan links, capture paths or dated implementation history.
- [ ] Preserve current public contracts unless an intentional redesign is
      justified. Follow normal ownership and dependency release order if an
      upstream change is required.
- [ ] Run focused quality/replay/allocation/resource checks, full Release/Lean
      suites, both target-framework builds and exact 100% reachable line/branch/
      method coverage in every changed library. Validate unreleased siblings with
      MSBuild and inherited `UseLocalLsfStack=true`; validate released packages
      before release.
- [ ] Archive completed evidence in `done/`; retain only concrete unresolved
      issues or explicitly evidence-gated follow-ups in coordination documents.

Offline mass-property tooling and richer diagnostic/publication platforms retain
their existing separate plans. This work does not activate those deferred
features without a measured need.
