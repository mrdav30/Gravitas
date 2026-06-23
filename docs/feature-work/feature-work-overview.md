# Feature Work Overview

**Status:** Living release-scope guide
**Owner:** Gravitas alpha hardening

## Alpha Bar

Gravitas alpha is not an MVP target. The alpha release should already feel like
a first-class deterministic physics engine: coherent APIs, deterministic
ordering, physically explainable solver behavior, low-allocation hot paths, and
credible 2D, 3D, mixed, CCD, query, and response quality.

Breaking changes are acceptable before alpha when they strengthen the engine or
end-user development experience. Weak compatibility layers, vague wiki caveats,
and hidden "future work" notes should be promoted into explicit plans or closed
with evidence.

## Pre-Alpha Release Blockers

These plans and evergreen closure trackers should be completed, or explicitly
closed with evidence, before the alpha release.

1. [`CCD Service-Level Island Solver`](2026-06-21-ccd-service-level-island-solver-plan.md)
   - Dense, chained, same-TOI, and mixed continuous contacts need a deterministic
     service-level model where body-owned substeps are insufficient.
2. [`Benchmark Signal Hardening Backlog`](benchmark-signal-hardening-backlog.md)
   - Measured allocation or runtime-cost signals must be reproduced, resolved,
     or closed with a no-change decision before alpha.
3. [`Feature Work Issue Tracker`](issue-tracker.md)
   - Bugs, correctness risks, documentation defects, and feature-work-discovered
     issues should be triaged, tested, and committed independently from feature
     design plans.

## Recently Completed

- [`CCD Exact TOI And Shape Reducers`](done/2026-06-21-ccd-exact-toi-and-shape-reducers-plan.md)
  - Completed 2026-06-23. Body-owned CCD now refines static-style 3D
    non-sphere targets with supported convex-source reducers, bracketed
    rotational CCD with fixed-iteration exact narrow-phase bisection, and pure
    2D/3D dynamic relative proxy candidates with exact mover-shape validation
    where supported. Mixed dynamic CCD remains in the service-level island
    plan for cross-dimension advancement and velocity handoff.
- [`CCD Active Swept Sources`](done/2026-06-21-ccd-active-swept-sources-plan.md)
  - Completed 2026-06-23. Host-driven kinematic 2D/3D translation and rotation
    now run as active CCD sources; static-style blockers clip the source,
    dynamic pure/mixed targets receive deterministic wake plus positional
    correction, and benchmark/docs coverage was added under
    `kinematic-active-ccd-scaling`.
- [`Mixed Sphere Against 2D Slab Reducer Completion`](done/2026-06-23-mixed-sphere-2d-slab-reducer-completion-plan.md)
  - Completed 2026-06-23. `SweepSphereAgainst2D` now uses exact finite-slab
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
    benchmark/docs coverage are now in place.
- [`Discrete Response And Contact Quality Hardening`](done/2026-06-21-discrete-response-and-contact-quality-hardening-plan.md)
  - Completed 2026-06-22. Resting friction, 3D warm-start application,
    deterministic discrete islands, cylinder/mesh contact quality, and mixed
    response islands are now covered by tests, docs, and benchmark signal.

## Post-Alpha / Evidence-Gated

These are valuable, but they do not define the core physics quality bar for the
first alpha release.

- [`Mesh Tooling Simplification And Decomposition`](2026-06-17-mesh-tooling-simplification-and-decomposition-plan.md)
  - Offline decomposition, simplification, and richer mesh tooling can mature
    after the runtime collision boundary is solid.
- [`Mass Inertia Tooling And Diagnostics Follow-Up`](2026-06-19-mass-inertia-tooling-and-diagnostics-follow-up-plan.md)
  - Principal-axis tooling, COM markers, and richer mass-property payloads are
    useful when demand appears, but the runtime mass/inertia model is already
    alpha-capable.
- [`Benchmark Publishing And CCD Diagnostics`](2026-06-21-benchmark-publishing-and-ccd-diagnostics-plan.md)
  - Publishing, baseline comparison, CI integration, and host-visible diagnostic
    polish can follow once the pre-alpha physics behavior is nailed down.

## Recommended Execution Order

1. CCD service-level island solver.
2. Benchmark signal closure pass.

This order now focuses on island-level CCD behavior after public mixed query
fallback directions, active kinematic source coverage, and exact body-owned CCD
reducers were closed. Benchmark signals remain a closure pass so measured
hot-path risks do not survive into alpha.
