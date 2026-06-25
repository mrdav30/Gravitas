# Feature Work Overview

**Status:** Living release-scope guide

## Release Quality Bar

Gravitas' first public release is not an MVP target. It should feel like a
first-class deterministic physics engine: coherent APIs, deterministic ordering,
physically explainable solver behavior, low-allocation hot paths, and credible
2D, 3D, mixed, CCD, query, and response quality.

Breaking changes are acceptable before the first public release when they
strengthen the engine or end-user development experience. Weak compatibility
layers, vague wiki caveats, and hidden "future work" notes should be promoted
into explicit plans or closed with evidence.

## Release Closure Trackers

These evergreen closure trackers should stay visible before release and after
release. Keep them empty when possible, and promote broad work into dated plans
instead of burying it in notes.

1. [`Benchmark Signal Hardening Backlog`](benchmark-signal-hardening-backlog.md)
   - Measured allocation or runtime-cost signals must be reproduced, resolved,
     or closed with a no-change decision before release.
2. [`Feature Work Issue Tracker`](issue-tracker.md)
   - Bugs, correctness risks, documentation defects, and feature-work-discovered
     issues should be triaged, tested, and committed independently from feature
     design plans.

## Recently Completed

- [`CCD Service-Level Island Solver`](done/2026-06-21-ccd-service-level-island-solver-plan.md)
  - Completed 2026-06-23. Pure 3D, pure 2D, and mixed dynamic CCD use
    service-owned processed-body handoff queues for chained TOI contacts,
    cross-service velocity transfer, bounded continuation, cap diagnostics, and
    active kinematic-source velocity handoff.
- [`CCD Exact TOI And Shape Reducers`](done/2026-06-21-ccd-exact-toi-and-shape-reducers-plan.md)
  - Completed 2026-06-23. Body-owned CCD refines static-style 3D
    non-sphere targets with supported convex-source reducers, bracketed
    rotational CCD with fixed-iteration exact narrow-phase bisection, and pure
    2D/3D dynamic relative proxy candidates with exact mover-shape validation
    where supported. Mixed dynamic CCD uses the service-level handoff queues
    added by the completed island-solver plan.
- [`CCD Active Swept Sources`](done/2026-06-21-ccd-active-swept-sources-plan.md)
  - Completed 2026-06-23. Host-driven kinematic 2D/3D translation and rotation
    run as active CCD sources; static-style blockers clip the source,
    dynamic pure/mixed targets receive deterministic velocity handoff through
    the completed service-level queue, and benchmark/docs coverage was added under
    `kinematic-active-ccd-scaling`.
- [`Mixed Sphere Against 2D Slab Reducer Completion`](done/2026-06-23-mixed-sphere-2d-slab-reducer-completion-plan.md)
  - Completed 2026-06-23. `SweepSphereAgainst2D` uses exact finite-slab
    reducers for 2D circle, AABB, convex polygon, and supported compound
    targets; static mixed CCD shares that policy, diagnostics label the path as
    exact, and dense/false-positive benchmark rows cover the source direction.
- [`Mixed Query Finite-Slab Reducer Completion`](done/2026-06-22-mixed-query-finite-slab-reducer-completion-plan.md)
  - Completed 2026-06-23. Rotated capsule/cylinder, mesh, and compound target
    reducers for `SweepCircleAgainst3D` are exact; convex mesh source scaling is
    accelerated by deterministic support-tree pruning; mixed query diagnostics,
    docs, and benchmark signal were refreshed.
- [`Query And Mixed Swept Shape Hardening`](done/2026-06-21-query-and-mixed-swept-shape-hardening-plan.md)
  - Completed 2026-06-22. Public 2D area-query parity, mixed primitive
    finite-slab reducers, convex/compound source sweeps, explicit
    concave-source rejection, query diagnostics, deterministic ordering, and
    benchmark/docs coverage are in place.
- [`Discrete Response And Contact Quality Hardening`](done/2026-06-21-discrete-response-and-contact-quality-hardening-plan.md)
  - Completed 2026-06-22. Resting friction, 3D warm-start application,
    deterministic discrete islands, cylinder/mesh contact quality, and mixed
    response islands are covered by tests, docs, and benchmark signal.

## Post-Release / Evidence-Gated

These are valuable, but they do not define the core physics quality bar for the
first public release.

- [`Mesh Tooling Simplification And Decomposition`](2026-06-17-mesh-tooling-simplification-and-decomposition-plan.md)
  - Offline decomposition, simplification, and richer mesh tooling can mature
    after the runtime collision boundary is solid.
- [`Mass Inertia Tooling And Diagnostics Follow-Up`](2026-06-19-mass-inertia-tooling-and-diagnostics-follow-up-plan.md)
  - Principal-axis tooling, COM markers, and richer mass-property payloads are
    useful when demand appears, but the runtime mass/inertia model is already
    release-capable.
- [`Benchmark Publishing And CCD Diagnostics`](2026-06-21-benchmark-publishing-and-ccd-diagnostics-plan.md)
  - Publishing, baseline comparison, CI integration, and host-visible diagnostic
    polish can follow once the release-critical physics behavior is nailed down.

## Recommended Execution Order

No active release-blocking feature plans remain in this overview. Keep the
benchmark backlog and issue tracker as intake buckets; promote new measured
risks into dated plans only when they are broader than a focused patch.
