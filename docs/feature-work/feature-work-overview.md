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

## Active Release-Scope

- [`Physics Material Model`](2026-06-26-physics-material-model-plan.md)
  - Planned 2026-06-26. Replaces ad hoc body-owned friction/restitution
    coefficients with deterministic collider-surface materials, static/dynamic
    friction, restitution combine policies, compound part material ownership,
    response/CCD integration, serialization, docs, and benchmarks.
- [`Body Axis Freeze Constraints`](2026-06-26-body-axis-freeze-constraints-plan.md)
  - Planned 2026-06-26. Replaces coarse mutable `Immovable` and
    `PreventAngularForces` toggles with explicit per-axis freeze constraints
    across 3D, pure 2D, mixed response, CCD, sleep, partitioning,
    serialization, and docs.
- [`Constraint And Ragdoll Foundation`](2026-06-26-constraint-and-ragdoll-foundation-plan.md)
  - Planned 2026-06-26. Adds deterministic 3D articulated-body physics through
    context-owned joints, warm-started solver constraints, ragdoll authoring
    definitions, collider-hierarchy-backed self-filtering, kinematic/animation
    handoff boundaries, CCD, serialization, diagnostics, and benchmarks.
- [`Pure 2D Grounding And Support`](2026-06-26-pure-2d-grounding-and-support-plan.md)
  - Planned 2026-06-26. Adds first-class `SolidBody2D` grounded-state support
    through planar contacts, deterministic in-plane probes, manual and disabled
    ownership modes, serialization, diagnostics, and docs while preserving the
    pure 2D X/Z coordinate contract.
- [`Collider Local Collision Filtering`](2026-06-26-collider-local-collision-filtering-plan.md)
  - Planned 2026-06-26. Adds collider-owned ignored physical layer masks for
    3D, pure 2D, mixed, CCD, and grounding/support paths while preserving
    caller-owned public query include-mask behavior.
- [`Batched Query APIs`](2026-06-26-batched-query-apis-plan.md)
  - Planned 2026-06-26. Adds deterministic high-throughput batched query
    request/result surfaces for 3D, pure 2D, and mixed services using
    caller-owned buffers, stable per-request ranges, shared scratch, diagnostics,
    allocation guardrails, and benchmark-backed broad-phase reuse.
- [`Pure 2D Capsule And Convenience Shapes`](2026-06-26-pure-2d-capsule-and-convenience-shapes-plan.md)
  - Planned 2026-06-26. Adds `LSCapsuleCollider2D` as a first-class analytic
    primitive across collision, query, CCD, mixed mode, serialization, and
    docs, while keeping triangles as convex-polygon authoring convenience.
- [`Cone Collider And Query Support`](2026-06-26-cone-collider-and-query-support-plan.md)
  - Planned 2026-06-26. Adds an analytic 3D cone primitive and cone-volume
    query support, with explicit research gates for mass properties, discrete
    contact quality, swept reducers, mixed mode, diagnostics, and benchmarks.

## Recently Completed

- [`Restitution Gravity And Grounded State Hardening`](done/2026-06-26-restitution-gravity-grounded-state-hardening-plan.md)
  - Completed 2026-06-26. Moves restitution cutoff policy into
    `PhysicsSettings`, routes discrete and CCD response through the context
    setting, adds `GravityScale` for 3D and pure 2D bodies, and records
    previous-step 3D grounded state for deterministic transition handling.
- [`Deterministic Replay Hash Conformance Harness`](done/2026-06-26-deterministic-replay-hash-conformance-harness-plan.md)
  - Completed 2026-06-26. Adds a deterministic authoritative-state hash,
    optional solver-cache hash mode, host-facing frame hash API, replay
    conformance fixtures, allocation guardrails, docs, and benchmark signal
    across 3D, pure 2D, mixed, CCD, query-cache, and serialization paths.
- **SolidBody Naming Cleanup**
  - Completed 2026-06-26. The public body API, source files, tests,
    benchmarks, and docs use `SolidBody` and `SolidBody2D` directly, with no
    compatibility aliases for the old pre-release terminology.
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
- **Scene / Fixture Authoring Definitions**
  - Hold until engine-specific adapter packages and sample projects clarify the
    real public authoring needs. Gravitas can already be configured directly
    through contexts, bodies, colliders, shape definitions, materials, and
    ragdoll definitions; a friendlier scene/fixture DTO layer should come from
    observed host workflows rather than speculation.

## Recommended Execution Order

1. [`Physics Material Model`](2026-06-26-physics-material-model-plan.md)
   - Clean up friction/restitution ownership before deeper solver constraints,
     ragdoll links, and new collider families multiply the old body-coefficient
     model.
2. [`Pure 2D Grounding And Support`](2026-06-26-pure-2d-grounding-and-support-plan.md)
   - Close the remaining pure 2D body-state parity gap before broader new
     feature work. This is release-scope because platformer-style 2D hosts
     reasonably expect grounded-state behavior, but it must stay planar rather
     than becoming hidden 3D height logic.
3. [`Collider Local Collision Filtering`](2026-06-26-collider-local-collision-filtering-plan.md)
   - Low conceptual risk but cross-cutting. It should land before new collider
     families so future shape work inherits one physical filtering rule.
4. [`Body Axis Freeze Constraints`](2026-06-26-body-axis-freeze-constraints-plan.md)
   - Deeper solver work that should land before adding more collider families,
     because new primitives should inherit the final mobility/constraint model
     instead of `Immovable` and angular-force cleanup churn.
5. [`Constraint And Ragdoll Foundation`](2026-06-26-constraint-and-ragdoll-foundation-plan.md)
   - Build deterministic articulation after the body constraint model is final.
     Ragdoll links can reuse collider hierarchy identity and local filtering,
     while the solver work can integrate with the final body freeze semantics.
6. [`Batched Query APIs`](2026-06-26-batched-query-apis-plan.md)
   - Add the high-throughput LSF query surface after local filtering is final
     and before new shape query families make the public surface broader.
7. [`Pure 2D Capsule And Convenience Shapes`](2026-06-26-pure-2d-capsule-and-convenience-shapes-plan.md)
   - Pairs naturally with 2D grounding and improves character-style 2D
     ergonomics without adding runtime triangle complexity.
8. [`Cone Collider And Query Support`](2026-06-26-cone-collider-and-query-support-plan.md)
   - Highest geometry/reducer risk of the new plans. Execute after the smaller
     collider API hardening items so cone work can focus on analytic geometry,
     contact quality, and query evidence.
9. Keep the benchmark backlog and issue tracker as intake buckets; promote new
   measured risks into dated plans only when they are broader than a focused
   patch.
