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

1. [`Query And Mixed Swept Shape Hardening`](2026-06-21-query-and-mixed-swept-shape-hardening-plan.md)
   - Public query truth and mixed finite-slab swept behavior need exact support
     or explicit fallback policy before users build gameplay on them.
2. [`CCD Active Swept Sources`](2026-06-21-ccd-active-swept-sources-plan.md)
   - Host-driven kinematic movement and rotation should be first-class swept
     sources rather than passive targets only.
3. [`CCD Exact TOI And Shape Reducers`](2026-06-21-ccd-exact-toi-and-shape-reducers-plan.md)
   - Remaining conservative reductions should become exact where the correctness
     gain justifies the cost, with deterministic fallback policy for unsupported
     shape families.
4. [`CCD Service-Level Island Solver`](2026-06-21-ccd-service-level-island-solver-plan.md)
   - Dense, chained, same-TOI, and mixed continuous contacts need a deterministic
     service-level model where body-owned substeps are insufficient.
5. [`Benchmark Signal Hardening Backlog`](benchmark-signal-hardening-backlog.md)
   - Measured allocation or runtime-cost signals must be reproduced, resolved,
     or closed with a no-change decision before alpha.
6. [`Feature Work Issue Tracker`](issue-tracker.md)
   - Bugs, correctness risks, documentation defects, and feature-work-discovered
     issues should be triaged, tested, and committed independently from feature
     design plans.

## Recently Completed

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

1. Query and mixed swept shape hardening.
2. CCD active swept sources.
3. CCD exact TOI and shape reducers.
4. CCD service-level island solver.
5. Benchmark signal closure pass.

This order now starts with the query/sweep surface users build on, then
finishes CCD source coverage, precision, and island-level behavior. Benchmark
signals remain a closure pass so measured hot-path risks do not survive into
alpha.
