# Feature Work Overview

## Purpose

This document is a living overview of Gravitas feature work. It tracks the
current release-scope, recently completed work, and future / evidence-gated
plans. It is not a backlog of all possible work, but a curated view of the most
important work for the first public release and beyond.

## Release Closure Trackers

These evergreen closure trackers should stay visible before release and after
release. Keep them empty when possible, and promote broad work into dated plans
instead of burying it in notes.

1. [`Benchmark Signal Hardening Backlog`](benchmark-signal-hardening-backlog.md)
   - Measured allocation or runtime-cost signals must be reproduced, resolved,
     or closed with a no-change decision before release.
2. [`Issue Tracker`](issue-tracker.md)
   - Bugs, correctness risks, documentation defects, and feature-work-discovered
     issues should be triaged, tested, and committed independently from feature
     design plans.

## Active Release-Scope

- [`Cross-Stack Issue Resolution`](issue-tracker.md)
  - Resolve release-blocking issues in dependency order: `FixedMathSharp`,
    `SwiftCollections`, `GridForge`, then Gravitas. Use the `develop` worktrees
    under `F:/gamedevrepos` and temporary local project references throughout
    the consumer chain while validating lower-stack changes. Retain those links
    until the complete cross-stack issue queue and final ownership pass are
    closed, then release each library sequentially, restore package references
    as releases become available, and revalidate every downstream consumer.
    Treat local links as temporary validation scaffolding, not release
    dependency changes.
- [`Benchmark Signal Hardening`](benchmark-signal-hardening-backlog.md)
  - Reproduce and close confirmed release-relevant signals alongside the owning
    library change. Do not broaden this into speculative optimization work.

## Recently Completed

- [`Full-Domain SolidBody Point Transforms`](done/2026-07-30-full-domain-solid-body-point-transform-plan.md)
  - Completed 2026-07-30. FixedMathSharp now owns one exact component-scaled
    3D transform kernel and the missing inverse operation; Gravitas exposes
    allocation-free throwing/`Try*` point conversion pairs over authoritative
    body pose and strict host hierarchy scale. Representable cancellation is
    retained until one final round-half-to-even coordinate conversion, while
    singular scale, unavailable hierarchy scale, and true final overflow fail
    explicitly and atomically.
  - FixedMathSharp passes 2,595 `Release` and 2,574 `ReleaseLean` tests at
    49,865/49,865 lines, 8,426/8,426 branches, and 3,304/3,304 methods.
    Gravitas passes 3,866 `Release` and 3,811 `ReleaseLean` tests at
    43,664/43,664 lines, 12,775/12,775 branches, and 4,503/4,503 methods.
    Standard and Lean multi-target package builds are warning-free. Ordinary
    and full-domain body round trips remain allocation-free in ShortRun.
- [`Full-Domain Friction Response`](done/2026-07-29-full-domain-friction-response-plan.md)
  - Completed 2026-07-30. Gravitas now retains exact 3D cached Coulomb-disk
    accumulation, pure-2D Coulomb-line response, and mixed two-axis response
    whenever compact arithmetic cannot prove the complete operation safe.
    True final overflow rejects atomically; ordinary contacts keep the compact
    path, and warmed exact fallbacks allocate zero managed bytes.
  - Gravitas passes 3,861 Release and 3,806 ReleaseLean tests at
    43,653/43,653 lines, 12,775/12,775 branches, and 4,501/4,501 methods.
    Forty-two representative 3D, 2D, and mixed response benchmark rows report
    zero managed allocation without a gross compact-path regression.
- [`FixedMathSharp / Gravitas Ownership Boundary`](done/2026-07-28-fixedmathsharp-gravitas-ownership-boundary-plan.md)
  - Completed 2026-07-29. FixedMathSharp now owns reusable exact math,
    semantic geometry, and internal wide mechanics; Gravitas is its sole
    intentional non-test friend and owns rigid-body levers, mass semantics,
    normal/friction policy, and exact response. The pass removed intermediate
    v7 physics APIs, centralized proven wide duplication, and reorganized
    geometry by coherent owners without exposing raw wide types.
  - FixedMathSharp passes 2,590 Release and 2,569 ReleaseLean tests plus eight
    Chronicler tests in each mode at 49,416/49,416 authored lines,
    8,340/8,340 branches, and 3,238/3,238 methods. Gravitas passes 3,818
    Release and 3,763 ReleaseLean tests at 42,180/42,180 lines,
    12,627/12,627 branches, and 4,474/4,474 methods. Standard and Lean
    package-only validation is warning-free across both target frameworks;
    38 focused anchor/response benchmark rows report zero managed allocation.
    The published-package relink and revalidation remain in the sequential
    release gates below.
- [`Exact Contact Lever And Mass Response`](done/2026-07-27-exact-contact-lever-response-plan.md)
  - Completed 2026-07-28. FixedMathSharp retains semantic 2D/3D point anchors,
    contact relations, and exact anchor-distance ordering without exposing raw
    wide arithmetic. Gravitas owns contact levers, mass points, positive
    weights, compact representable paths, and allocation-free exact fallback
    across 2D, 3D, mixed response, rotational CCD, compound mass properties,
    and embedded mixed boundaries. Runtime collider hierarchies are closed;
    host adapters author built-in geometry through
    `ColliderShapeDefinition*.CreateCollider()`.
    FixedMathSharp passes 2,638 Release and 2,617 ReleaseLean tests at
    47,462/47,462 lines, 8,732/8,732 branches, and 3,500/3,500 methods.
    Gravitas passes 3,776 Release and 3,721 ReleaseLean tests at
    40,072/40,072 lines, 12,365/12,365 branches, and 4,368/4,368 methods;
    replay, allocation, package, documentation, and independent-review gates
    are closed.
- [`Canonical Collider Geometry And Exact Scale Admission`](issue-tracker.md#finite-axis-collider-geometry-uses-canonical-rigid-frames)
  - Completed 2026-07-27. FixedMathSharp owns strict transform composition,
    fused scaled dimensions, centered finite-axis relations, local convex
    boundaries, and `FixedOrientedBox`; Gravitas publishes collider geometry
    transactionally and consumes canonical rigid-frame anchors across 2D, 3D,
    mixed, mesh, query, CCD, replay, response, and diagnostics. FixedMathSharp
    passes 2,575 Release and 2,554 ReleaseLean tests at exact 100% coverage.
    Gravitas passes 3,669 Release and 3,614 ReleaseLean tests at 37,548/37,548
    lines, 11,865/11,865 branches, and 4,246/4,246 methods; repeated replay and
    allocation gates are green. The correctness work is closed, while the
    exact-winner optimization reduced the measured ordinary-domain
    `FixedOrientedBox` cost by 37-46%. The remaining throughput gap stays a
    high-priority benchmark signal rather than being hidden behind downstream
    approximation.
- [`Full-Domain Conic Query And Triangle Arithmetic`](issue-tracker.md#cone-triangle-face-interiors-are-reduced-without-edge-crossings)
  - Completed 2026-07-22. FixedMathSharp now owns exact allocation-free
    finite-cone segment intervals for apex-authored and centered cones, while
    Gravitas consumes distance intervals for cone-collider raycasts and
    high-resolution point intervals for concave-mesh edge reduction.
    `FixedTriangle` separately owns exact projected containment plus
    face-interior cone hits even when no edge or cone axis crosses the triangle.
    Extreme
    coordinates, clipped opposite lobes, roots near half-even boundaries,
    non-cardinal authored endpoint contact, long-edge spatial witnesses,
    triangle feature ties, and tiny nonzero segments retain full-domain
    behavior. The authoritative Gravitas Release artifact passes 3,238 tests at
    100% line, branch, and method coverage; ReleaseLean passes 3,183 tests.
    FixedMathSharp passes 1,714 Release and 1,693 ReleaseLean tests plus eight
    Chronicler tests in each configuration at exact 100% coverage. The final
    warmed cone raycast, concave-mesh overlap, mesh/cone collision, and oblique
    long/narrow bounds rows remain allocation-free.
- [`Body Motion Type And Solver Mobility Hardening`](done/2026-07-20-body-motion-type-and-solver-mobility-plan.md)
  - Completed 2026-07-20. Adds explicit Dynamic, Kinematic, and Static body
    roles while keeping translation and rotation freeze constraints
    independent across 2D, 3D, mixed response, constraints, partitions, CCD,
    serialization, replay, and host presentation. Atomic transition and pose
    contracts preserve runtime identity and reject invalid hierarchy-composed
    transforms before observable mutation. The authoritative Release artifact
    passes 3,237 tests at 100% line, branch, and method coverage; warmed 3D/2D
    role transitions remain allocation-free.
- [`Finite-Axis Full-Domain Projection Closure`](issue-tracker.md#finite-axis-capsule-cylinder-and-mesh-edge-projections-can-saturate-before-solving)
  - Completed 2026-07-19. FixedMathSharp now owns exact bounded-ray and
    authored-segment physical-distance capsule/cylinder intervals, and Gravitas
    consumes them across 2D, 3D, mixed, raycast, sweep, and mesh-edge reducers.
    The authoritative Gravitas Release artifact passes 3,103 tests at 100%
    line, branch, and method coverage; FixedMathSharp also retains exact 100%
    coverage. Final finite-axis benchmark rows remain allocation-free, while
    radial distance precision and the remaining conservative rim/support
    models stay explicitly queued as separate work.
- [`Rotational Moving-Pair CCD Hardening`](done/2026-07-18-rotational-moving-pair-ccd-plan.md)
  - Completed 2026-07-19. Pure 2D, pure 3D, and mixed rotational CCD now owns
    moving dynamic and kinematic targets through order-independent piecewise
    trajectories, stable normalized-time arbitration, contact-point angular
    response, and bounded atomic handoffs. The authoritative 3,056-test
    artifact reports 100% line, branch, and method coverage; the focused
    1/8/32-pair benchmark remains approximately linear with only the separately
    tracked mixed broad-phase capacity-growth allocation signal.

- [`FixedMathSharp Foundation Hardening`](../../../FixedMathSharp/docs/feature-work/done/2026-07-14-fixedmathsharp-foundation-hardening-plan.md)
  - Completed 2026-07-17. FixedMathSharp now owns the shared full-domain
    arithmetic, vector/quaternion, segment/triangle, and transform contracts;
    Gravitas consumes those contracts without duplicate math. The final artifact
    reports 100% line, branch, and method coverage, with 1,406 standard and
    1,385 Lean tests passing. Sequential package releases remain tracked by the
    issue tracker.
- [`Coverage Hardening`](done/coverage-hardening-plan.md)
  - Completed 2026-07-13. The final unexcluded artifact reports 100% line,
    branch, and method coverage across 27,477 lines, 10,411 branches, and 3,838
    methods. The `Release` suite passes 2,556 tests, `ReleaseLean` passes 2,518
    tests, both Lean targets build without warnings, and independent final
    review found no actionable issues.
- [`Pure 2D Constraint And Ragdoll Foundation`](done/2026-07-02-pure-2d-constraint-and-ragdoll-foundation-plan.md)
  - Completed 2026-07-03. Adds a native pure 2D constraint service, distance,
    pin/revolute, weld/fixed, and prismatic/slider joint rows, contact-
    integrated 2D islands, linked-collider filtering, 2D ragdoll authoring,
    serialization, replay hashing, diagnostics, docs, and benchmark evidence.
    Independent review fixes tightened ragdoll registration atomicity, shared
    joint payload validation, zero-error solver row emission, and enabled-joint
    fast-path gating in both 2D and 3D constraint services.
- [`3D Constraint Solver Stress And Tuning Hardening`](done/2026-07-02-3d-constraint-solver-stress-and-tuning-hardening-plan.md)
  - Completed 2026-07-03. Adds long-chain, alternating hinge, humanoid-ish,
    contact-heavy, and motor-driven 3D articulation stress coverage; exposes
    deterministic joint solve metrics through `Joint3D`, replay hashing, and
    diagnostics; hardens angular-error math and warmed body hit buffers; and
    closes the public tuning decision with benchmark/test evidence instead of
    speculative stiffness/compliance knobs.
- [`Rotated Cone Projection And Query Bounds Hardening`](done/2026-07-02-rotated-cone-projection-and-query-bounds-hardening-plan.md)
  - Completed 2026-07-02. Replaces rotated finite-cone mixed query conservative
    projection with exact support-mapped circle-slab source sweeps, tightens
    physical cone and cone-volume query bounds through shared deterministic cone
    geometry, and records mixed/cone benchmark evidence.
- [`Chronicler Replay Hash Migration`](done/2026-07-02-chronicler-replay-hash-migration-plan.md)
  - Completed 2026-07-02. Replaces Gravitas-local replay hash value and writer
    infrastructure with Chronicler `ChronicleHash`/`ChronicleHashWriter` and
    FixedMathSharp.Chronicler math writers while preserving Gravitas-owned
    replay inclusion policy, deterministic ordering, and allocation guardrails.
- [`FixedMathSharp v6 Geometry Adoption`](done/2026-07-02-fixedmathsharp-v6-geometry-adoption-plan.md)
  - Completed 2026-07-02. Replaces duplicate local mesh triangle structs with a
    `FixedTriangle`-backed `CollisionTriangle`, routes unsafe 3D edge
    closest-point work through `FixedSegment`, centralizes tolerant 2D segment
    projection, preserves cached mesh normals and SwiftCollections query bounds,
    and records benchmark evidence for mesh/mixed/query-sensitive paths.
- [`Cone Collider And Query Support`](done/2026-06-26-cone-collider-and-query-support-plan.md)
  - Completed 2026-06-28. Adds `LSConeCollider` as a first-class analytic 3D
    primitive across shape definitions, mass properties, collision, CCD, mixed
    mode, cone-volume queries, source sweeps, diagnostics, serialization, docs,
    and benchmark signal.
- [`Pure 2D Capsule And Convenience Shapes`](done/2026-06-26-pure-2d-capsule-and-convenience-shapes-plan.md)
  - Completed 2026-06-27. Adds `LSCapsuleCollider2D` as a first-class analytic
    primitive across shape definitions, mass properties, collision manifolds,
    pure 2D query/CCD/grounding, mixed slabs, diagnostics, serialization, docs,
    and benchmark signal while keeping triangles as convex-polygon authoring
    convenience.
- [`Batched Query APIs`](done/2026-06-26-batched-query-apis-plan.md)
  - Completed 2026-06-27. Adds typed closest/all-hit batch APIs for current 3D,
    pure 2D, and mixed query families with caller-owned request/output buffers,
    stable per-request hit ranges, public batch summary counters, allocation
    guardrails, docs, and benchmark smoke coverage.
- [`Constraint And Ragdoll Foundation`](done/2026-06-26-constraint-and-ragdoll-foundation-plan.md)
  - Completed 2026-06-27. Adds context-owned deterministic 3D joints,
    contact-integrated constraint islands, ragdoll authoring/runtime activation,
    linked-collider self-filtering, service-level motor target handoff,
    Chronicler state recording, replay hashing, diagnostics, debug draw capture,
    tests, and benchmark signal.
- [`Collider Local Collision Filtering`](done/2026-06-26-collider-local-collision-filtering-plan.md)
  - Completed 2026-06-27. Adds collider-owned ignored physical layer masks for
    3D, pure 2D, mixed, CCD, and grounding/support paths while preserving
    caller-owned public query include-mask behavior.
- [`Pure 2D Grounding And Support`](done/2026-06-26-pure-2d-grounding-and-support-plan.md)
  - Completed 2026-06-27. Adds first-class `SolidBody2D` grounded-state support
    through planar contacts, deterministic in-plane ray/swept-circle probes,
    automatic/manual ownership modes, serialization, replay hashing,
    diagnostics, and docs while preserving the pure 2D X/Z coordinate contract.
- [`Body Axis Freeze Constraints`](done/2026-06-26-body-axis-freeze-constraints-plan.md)
  - Completed 2026-06-27. Replaces coarse mutable body mobility toggles with
    explicit 3D and pure 2D freeze axes across motion, constrained solver mass,
    mixed response, CCD, partition mobility, serialization, docs, tests, and
    benchmarks.
- [`Physics Material Model`](done/2026-06-26-physics-material-model-plan.md)
  - Completed 2026-06-26. Replaces ad hoc body-owned friction/restitution
    coefficients with deterministic collider-surface materials, static/dynamic
    friction, restitution combine policies, compound part material ownership,
    3D/pure 2D/mixed response integration, serialization, docs, and benchmark
    signal.
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
  - Completed 2026-06-26. The public body API, source files, tests, benchmarks,
    and docs use `SolidBody` and `SolidBody2D` directly, with no compatibility
    aliases for the old pre-release terminology.
- [`CCD Service-Level Island Solver`](done/2026-06-21-ccd-service-level-island-solver-plan.md)
  - Completed 2026-06-23. Pure 3D, pure 2D, and mixed dynamic CCD use
    service-owned processed-body handoff queues for chained TOI contacts,
    cross-service velocity transfer, bounded continuation, cap diagnostics, and
    active kinematic-source velocity handoff.
- [`CCD Exact TOI And Shape Reducers`](done/2026-06-21-ccd-exact-toi-and-shape-reducers-plan.md)
  - Completed 2026-06-23. Body-owned CCD refines static-style 3D non-sphere
    targets with supported convex-source reducers, bracketed rotational CCD with
    fixed-iteration exact narrow-phase bisection, and pure 2D/3D dynamic
    relative proxy candidates with exact mover-shape validation where supported.
    Mixed dynamic CCD uses the service-level handoff queues added by the
    completed island-solver plan.
- [`CCD Active Swept Sources`](done/2026-06-21-ccd-active-swept-sources-plan.md)
  - Completed 2026-06-23. Host-driven kinematic 2D/3D translation and rotation
    run as active CCD sources; static-style blockers clip the source, dynamic
    pure/mixed targets receive deterministic velocity handoff through the
    completed service-level queue, and benchmark/docs coverage was added under
    `kinematic-active-ccd-scaling`.
- [`Mixed Sphere Against 2D Slab Reducer Completion`](done/2026-06-23-mixed-sphere-2d-slab-reducer-completion-plan.md)
  - Completed 2026-06-23. `SweepSphereAgainst2D` uses exact finite-slab reducers
    for current supported 2D slab targets; static mixed CCD shares that policy,
    diagnostics label the path as exact, and dense/false-positive benchmark rows
    cover the source direction.
- [`Mixed Query Finite-Slab Reducer Completion`](done/2026-06-22-mixed-query-finite-slab-reducer-completion-plan.md)
  - Completed 2026-06-23. Rotated capsule/cylinder, mesh, and compound target
    reducers for `SweepCircleAgainst3D` are exact; convex mesh source scaling is
    accelerated by deterministic support-tree pruning; mixed query diagnostics,
    docs, and benchmark signal were refreshed.
- [`Query And Mixed Swept Shape Hardening`](done/2026-06-21-query-and-mixed-swept-shape-hardening-plan.md)
  - Completed 2026-06-22. Public 2D area-query parity, mixed primitive
    finite-slab reducers, convex/compound source sweeps, explicit concave-source
    rejection, query diagnostics, deterministic ordering, and benchmark/docs
    coverage are in place.
- [`Discrete Response And Contact Quality Hardening`](done/2026-06-21-discrete-response-and-contact-quality-hardening-plan.md)
  - Completed 2026-06-22. Resting friction, 3D warm-start application,
    deterministic discrete islands, cylinder/mesh contact quality, and mixed
    response islands are covered by tests, docs, and benchmark signal.

## Future / Evidence-Gated

The following work is not currently in the active release-scope, but it is
evidence-gated and may be promoted into dated plans once measured risks or a
host-facing need appears.

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
- **Support-mapped convex penetration kernel**
  - Evaluate deterministic EPA or MPR only if measured contact-quality gaps
    appear in generic convex fallback pairs such as
    cone/cylinder/capsule/convex-mesh overlaps. Analytic, SAT, and
    shape-specific manifold paths remain primary; any EPA/MPR work must be
    bounded, allocation-free, fixed-point deterministic, benchmarked, and
    adopted only where it improves contact quality without replacing stronger
    existing solvers.

## Recommended Execution Order

1. Keep the benchmark backlog and issue tracker as intake buckets; promote new
   measured risks into dated plans only when they are broader than a focused
   patch.
2. Resolve the remaining cross-stack issues, including Gravitas-owned
   lifecycle, admission, geometry, and numeric-range blocks, while retaining
   the temporary local project references.
3. Release FixedMathSharp from the reviewed hardening tree.
4. Update SwiftCollections to the released FixedMathSharp package, run its full
   validation, and release SwiftCollections before advancing the chain.
5. Update GridForge to the released lower-stack packages, validate its resolved
   runtime-identity hardening and downstream consumers, and release GridForge.
6. Update Gravitas to the released package versions, remove every temporary
   local link, and rerun `Release`, `ReleaseLean`, coverage, replay, allocation,
   and relevant benchmark gates against package-only dependencies before its
   first public release.
