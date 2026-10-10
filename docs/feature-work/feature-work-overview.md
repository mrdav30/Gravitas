# Feature Work Overview

## Purpose

This document is an evergreen overview of Gravitas feature work. It tracks the
active scope, recently completed work, and deferred or evidence-gated plans. It
is a curated view rather than a backlog of every possible feature.
Update it for significant feature-work milestones; keep individual issue and
benchmark status in their coordination trackers.

Keep active and deferred plans alongside the coordination trackers in this
directory. Archive completed plans and finished investigation reports under
`done/`, mark their completion status, and update incoming and relative links.
Archived phase assessments describe the evidence at that time.

## Coordination Trackers

Keep these trackers empty when possible, and promote broad work into dated plans
instead of burying it in notes.

1. [`Benchmark Signal Hardening Backlog`](benchmark-signal-hardening-backlog.md)
   - Measured allocation or runtime-cost signals must be reproduced, resolved,
     or closed with a documented no-change decision.
2. [`Issue Tracker`](issue-tracker.md)
   - Bugs, correctness risks, documentation defects, and feature-work-discovered
     issues should be triaged, tested, and committed independently from feature
     design plans.

## Active Coordination

- **Lockstep Conformance And Workload Guidance**
  - Approved follow-up sequence after the confirmed benchmark signals close:
    strengthen complete-loop replay evidence, characterize an evolving strategy
    scene, then publish long-duration physical quality versus configuration cost.
    Full-lifecycle replay conformance is complete across the eight native
    platform/profile lanes, and upstream develop CI alignment is complete.
    The workload and quality plans follow.
  - [`Evolving Game Capacity And Soak`](2026-10-06-evolving-game-capacity-and-soak-plan.md)
    follows complete stepping and a shared trace format. One strategy scene
    supplies activity stages, a comparable 3D case, per-frame tails and churn
    memory evidence; an MMO-zone variant follows only when it adds distinct
    actor-policy evidence. This is the execution scope for experimental #024.
  - [`Long-Duration Physics Quality And Tuning`](2026-10-06-long-duration-physics-quality-and-tuning-plan.md)
    follows initial workload evidence. Reuse existing stack/chain/ragdoll tests
    and telemetry to measure drift, penetration, energy behavior, sleep/wake and
    supported restore continuation alongside solver cost. Lasting configuration
    guidance is a deliverable shared with the capacity plan.
- [`Cross-Stack Issue Resolution`](issue-tracker.md)
  - Resolve cross-stack issues in dependency order: `FixedMathSharp`,
    `SwiftCollections`, `GridForge`, then Gravitas. Package references are the
    default; use `UseLocalLsfStack=true` only for coordinated validation of
    unreleased sibling changes, then revalidate against released packages.
- [`Benchmark Signal Hardening`](benchmark-signal-hardening-backlog.md)
  - Reproduce and close confirmed release-relevant signals alongside the owning
    library change. Do not broaden this into speculative optimization work.

## Recently Completed

- [`Surface Contact Manifolds`](done/2026-10-08-surface-contact-manifold-plan.md)
  - Independent 3D surface groups preserve finite mesh/cone contacts through
    deterministic sequential response, resolving #095/#099. Exact admission and
    provenance precede bounded anchor sampling; four points per group replace
    the pair-wide ceiling. Canonical inspection order and retained capacity are
    documented in the collision guides.

- **Upstream Develop CI Alignment**
  - Completed 2026-10-07. FixedMathSharp, SwiftCollections and GridForge retain
    Windows/Linux Release/Lean matrices, with pinned sibling sources on develop
    and released packages on main or PRs targeting main. Serial source builds
    and separate dependency-mode caches preserve the selected graph. Hosted
    [FixedMathSharp run 37644534163](https://github.com/mrdav30/FixedMathSharp/actions/runs/37644534163),
    [SwiftCollections run 37654294859](https://github.com/mrdav30/SwiftCollections/actions/runs/37654294859)
    and [GridForge run 37655373739](https://github.com/mrdav30/GridForge/actions/runs/37655373739)
    pass. Chronicler already passed and needed no workflow change. The collection
    RCA and finalizer coverage follow-up are owned by the
    [SwiftCollections tracker](https://github.com/mrdav30/SwiftCollections/blob/develop/docs/feature-work/issue-tracker.md).

- [`Full-Lifecycle Replay Conformance`](done/2026-10-06-full-lifecycle-replay-conformance-plan.md)
  - Completed 2026-10-07. Full host-loop stepping, reviewed command/result
    fixtures and native Windows/Linux x64/ARM64 Release/Lean CI now agree across
    five fixtures and 36 frames. [Run 37640701967](https://github.com/mrdav30/Gravitas/actions/runs/37640701967)
    passed every lane and direct comparison; downloaded artifacts independently
    reproduce the result. Exact production coverage remains 100%. Develop
    validates the source stack; main promotion still requires released packages.

- [`Runtime Mass Mutation`](issue-tracker.md#grv-issue-089---runtime-mass-changes-leave-inertia-or-awake-membership-stale)
  - GRV-Issue-089 resolved 2026-10-04. Both dimensional body owners derive
    inertia atomically, invalidate contact/joint/CCD caches, and synchronize
    pure/mixed awake membership while preserving roles and accepted motion.
  - Same-value assignments preserve sleep; changed runtime assignments are
    allowed between complete steps. Recorded-state loads retain saved sleep.
    Release/Lean coverage, replay, and warmed allocation gates pass; the 3D
    field-to-property migration is documented. Gravitas phase committed in `942d80f`.

- [`Circle Workload Partition And Grounding Scaling`](benchmark-signal-hardening-backlog.md#grv-benchmark-023--circle-workload-partition-and-grounding-scaling)
  - GRV-Benchmark-023 closed 2026-10-04 under the predictable-scaling and
    published-cost criterion. Gravitas changes are committed through `1c2ce4a`,
    with coordinated FixedMathSharp `5447b34` and GridForge `3f34f8b` phases.
  - Closure verification passed 4473/4414 Release/Lean tests with 100% line/branch/method coverage;
    both target frameworks build without warnings/errors. Final scaling,
    contract/allocation smoke and all three DocFX gates pass. Repeated controls
    do not establish a stable full-step gain from the latest bounds rejection.
  - At 1024 diagonal pairs, 16-unit cells cost 30.0562 +/- 0.5735 ms and unit
    cells 201.9929 +/- 4.3606 ms, both 0 B/op. Costs are not uniformly linear;
    closure does not certify a fixed-Hz host budget. Detailed counts, rejected
    experiments and history remain in the backlog. Released-package validation
    remains a future release gate.

- [`Exact 2D Circle Contacts`](issue-tracker.md#grv-issue-083---2d-circle-contacts-compare-saturated-squared-distances)
  - Completed 2026-09-30. Circle contacts retain wide classification, correctly
    rounded depth, conceptual clamping and authored radial anchors through the
    existing FixedMathSharp zero-core capsule owner. No second Gravitas solver
    or public API was added.
  - Shared point-core, root, normalization and residual work removes 66.6-83.2%
    of the first complete-query cost; sphere and nonzero-capsule controls also
    improve. Both repositories retain 100% reachable line/branch/method coverage
    in Release and Lean in that capture.
  - The 2026-10-02 GRV-Benchmark-022 follow-up removes redundant shared vector
    rescaling, preserves raw results and improves diagonal/rotated direct queries
    by 7.2%/3.7%. A sustained 64/1024-pair workload keeps the remaining exact
    premium visible without claiming a full-step speedup or accepting a frame
    budget. Its larger partition/grounding costs are documented in closed
    GRV-Benchmark-023.
    Fresh Release/ReleaseLean builds and suites pass in both repositories with
    exact 100% reachable line/branch/method coverage and no review findings.

- [`Complete Capsule/Slab Contact`](done/2026-09-29-complete-capsule-slab-contact-plan.md)
  - Completed 2026-09-30; GRV-Issue-088 is resolved. Both repositories retain
    100% measured line/branch/method coverage in Release/ReleaseLean; focused
    Debug/resource checks and DocFX pass. Ordinary cap/side/end fixtures improve
    37-41%. The user accepted the remaining complete-query cost for now.
  - GRV-Benchmark-021 closed 2026-10-05; Gravitas `989c7f9` and coordinated
    FixedMathSharp `bd8f6a2` are committed. GRV-Benchmark-020 also closes on
    2026-10-05 after its separate investigation. Their shared
    analytic materializer and distinct oblique root paths have coordinated
    profiles. [Retained-root refinement](done/2026-10-04-capsule-slab-cost-refinement.md)
    improves four oblique stadium fixtures by 7.1-24.2% with 0 B/op. The
    [certified-sign refinement](done/2026-10-05-capsule-slab-sign-refinement.md)
    subsequently brings the analytic-winner fixture to about 1.39 ms with
    0 B/op (49.9-51.3% mean reduction across repeated committed baselines),
    reusing retained-cell and endpoint certificates with a measured
    point-evaluation budget. The subsequent
    [root-isolation refinement](done/2026-10-05-capsule-slab-root-isolation-refinement.md)
    shares subdivision counts and reciprocal-chart chains and certifies negative
    global minima. Its four fixtures improve another 11.5-26.4% against fresh
    committed captures, to 0.53-1.05 ms/query, all 0 B/op. Closure uses verified
    gains and published remaining cost, not a fixed-rate workload budget.
    No single common CPU hotspot is proved for #020 and #021. Exact classification, rounding, chart
    coverage and bounded scratch remain unchanged. Fresh 2026-10-05
    standard/Lean coverage remains exactly 100% line/branch/method in both
    repositories; focused Debug/resource checks and both DocFX gates pass.

- [`Complete Capsule/Circle-Slab Contacts`](issue-tracker.md#grv-issue-085---mixed-capsulecircle-slab-contact-bypasses-the-complete-upstream-query)
  - Completed 2026-09-29. Mixed capsule contacts reuse FixedMathSharp's complete
    cylinder/capsule geometry, including zero-core planar capsule slabs. The
    incomplete direction helpers are removed, full slab thickness is retained,
    and exact rim classification, minimum depth and canonical anchors agree
    with the shared 3D contract. Nonzero-core capsule slabs were subsequently
    repaired as GRV-Issue-088. The measured complete-query cost, including
    equivalent 3D controls, is retained as GRV-Benchmark-020. Its first
    [exact output refinement](done/2026-10-05-capsule-circle-cost-refinement.md)
    improves the representative oblique query by 37.2% mixed / 35.6% 3D,
    to 0.72-0.75 ms at 0 B/op with exact 100% standard/Lean coverage across
    both repositories. Subsequent certified point signs, invariant normal products
    and factored radical bounds bring endpoint rim output to 37–39 µs, ordinary
    output to roughly 10–11.5 µs and the oblique query to about 0.70 ms. Shared
    stadium ordinary contacts improve another 36–54% and straight-rim output
    improves 23%; oblique and triangle controls have small shifts with overlapping
    intervals. GRV-Benchmark-020 closes 2026-10-05 with 0 B/op throughout and
    fresh exact 100% reachable line/branch/method coverage in both configurations,
    Debug resource/allocation checks, both target frameworks and DocFX/local-link
    gates. The rejected cell-certificate experiment is removed. Remaining exact
    oblique root cost is published under the same judgment-based closure bar as
    #021; these query costs promise no host rate or universal multiplayer scale.

- [`Canonical-Frame Cone-Volume Queries`](issue-tracker.md#grv-issue-087---cone-volume-queries-reject-an-intersecting-mesh-when-the-apex-cannot-enter-its-scalar-frame)
  - Completed 2026-09-29. Closest/all-hit and batch cone queries preserve the
    authored mesh frame through FixedMathSharp's shared minimum-axial query.
    Unrepresentable intermediate coordinates no longer reject genuine hits;
    deterministic candidate order and allocation-free warmed queries remain.

- [`Complete Triangle/Cylinder Cost Refinement`](done/2026-10-06-triangle-cylinder-cost-refinement.md)
  - GRV-Benchmark-018 closes 2026-10-06 after exact reciprocal-chart and
    opposite-direction support reuse. Complete feature admission, canonical ties,
    correctly rounded depth and paired witnesses remain shared by 3D cylinder
    and mixed circle-slab contacts.
  - Rim-heavy fixtures improve 9-20%; 64-pair mesh batches improve 6.5-8.1%,
    all at 0 B/op. Positive-core triangle/slab controls preserve results and
    allocations. Winning-root retention is reverted after its preliminary gain
    fails the fresh controlled comparison.
  - Fresh Release/Lean coverage, Debug/resource tests, both target frameworks
    and API documentation gates are recorded in the report. Remaining exact
    costs and scene capacity belong to GRV-Benchmark-024.

- [`Complete Triangle/Cone Contact`](done/2026-09-28-complete-triangle-cone-contact-plan.md)
  - Completed 2026-09-29. FixedMathSharp now selects complete triangle/cone
    contacts in authored frames; Gravitas consumes coherent normal/depth/anchors
    and removes incomplete sampling and scalar-frame rejection. Positive gaps
    reject without changing closed-convex fallback or CCD policy.
  - Both repositories retain 100% reachable line, branch and method coverage in
    Release and ReleaseLean. Exact face certificates and shared depth reduction
    cut ordinary face costs by 67–89% from the first complete solver; measured
    remaining costs are published in closed GRV-Benchmark-019. Fresh shared-owner
    controls detected no material regression; frequency and complete-step capacity
    remain GRV-Benchmark-024. The separate cone-volume query bug was resolved as
    GRV-Issue-087.
  - [Final benchmark refinement](done/2026-10-06-triangle-cone-final-refinement.md)
    closes GRV-Benchmark-019 after four measured passes through existing exact
    owners. The last retained root-sign change lowers interior-rim cost by 4.1%
    against its fresh baseline, with zero allocations and 100% coverage. Ordinary
    faces cost about 23-29 us; remaining exact rim and gap costs are published.

- [`Mixed Discrete Broad-Phase Signal Closure`](benchmark-signal-hardening-backlog.md#grv-benchmark-012--mixed-discrete-broad-phase-allocation-at-32-pairs)
  - Completed 2026-08-04. The original run-dependent 32-pair allocation no
    longer reproduces: two rotational confirmations and corrected sparse, dense,
    and churn broad-phase rows report `0 B/op` through 1,024 colliders.
  - The audit repaired a stale benchmark that stopped before `LateSimulate`,
    stabilized its workload with trigger pairs, isolated per-row setup, and
    removed an unrepresentative 4,096-collider monolithic dense-grid case that
    exceeded `2.6 GB` before measurement. No production preallocation or second
    runtime path was added, and no new release-relevant signal remains.
  - Release and `ReleaseLean` pass 3,930 and 3,875 tests. Fresh coverage remains
    at 100% across 55,869 lines, 15,833 branches, and 5,321 ReportGenerator
    methods; both package configurations build without warnings.

- [`Mesh Scale Rebuild Throughput Hardening`](done/2026-08-03-mesh-scale-rebuild-throughput-plan.md)
  - Completed 2026-08-03. Convex meshes now build one immutable support-vertex
    topology and refit only transactional node bounds after scale changes. The
    second support-index buffer, repeated per-node sorting, and retained
    construction comparer were deleted without changing exact support results or
    authored-order ties.
  - Subdivision 8 and 16 scale-rebuild rows drop from `4,032 B/op` and
    `16,320 B/op` to `0 B/op` while improving by `7.9%` and `7.8%` against the
    refreshed baseline. Gravitas passes 3,928 Release and 3,873 ReleaseLean
    tests at 100% reachable line, branch, and method coverage.

- [`Exact 3D Contact Response Throughput Hardening`](done/2026-08-03-exact-contact-response-throughput-plan.md)
  - Completed 2026-08-03. FixedMathSharp now proves compact identical-frame and
    identity-frame point-anchor differences before entering the general exact
    two-frame reducer. It adds no public API, physics policy, cache, or second
    answer path; failed compact proofs retain the full-domain fallback.
  - Direct same-frame identity and rotated rows improve by `95.9%` and `61.0%`.
    The unchanged 24-row Gravitas matrix improves by `46.4%` median versus the
    exact-response baseline and `36.0%` versus the older compact implementation;
    a second optimized matrix is within `0.7%` median. Every row remains at
    `0 B`, both repositories retain 100% reachable coverage, and no residual
    solver experiment was required.

- [`Exact Canonical OBB Throughput Hardening`](done/2026-08-02-exact-canonical-obb-throughput-plan.md)
  - Completed 2026-08-03. Box, triangle/hull, and finite-capsule contacts now
    use single exact relative-frame kernels with deterministic feature
    ownership, canonical anchors, zero allocation, and complete raw-domain
    behavior.
  - Matched-command phase comparisons improve direct FixedMathSharp rows by
    `35.3-64.0%` and matching Gravitas rows by `30.9-55.7%`; two full
    `DefaultJob` confirmations establish stable final timings at `0 B`. Both
    repositories retain 100% reachable coverage, and residual experiments below
    the `5%` family gate were reverted completely.

- [`Experimental Exact Triangle-Pair Throughput Pass`](done/2026-08-02-experimental-triangle-pair-throughput-plan.md)
  - Completed 2026-08-02 with no production change retained. Exact signed
    two-limb multiplication improved the direct triangle row only `0.6%`, and
    invocation-local rigid-frame preparation left affected Gravitas rows flat to
    `1.04%` slower. Both experiments were reverted exactly.
  - Evidence now favors reducing complete exact SAT work per BVH-admitted
    triangle pair. Dense dynamic concave mesh/mesh collision is now experimental
    capacity guidance; competitive release authoring should favor primitives,
    convex meshes, decomposed compounds, or partitioned static concave surfaces.

- [`Exact Triangle-Pair Throughput Optimization`](done/2026-08-02-exact-triangle-pair-throughput-plan.md)
  - Completed 2026-08-02. A sampled dense-row profile isolated generic
    wide-multiply dispatch inside exact triangle projection. FixedMathSharp now
    owns an exact `Signed576`-by-`long` specialization, and the unchanged
    Gravitas concave rows improved by a repeatable `13.7-15.4%` without a new
    public API, answer path, or warmed allocation.
  - FixedMathSharp passes 2,653 Release and 2,632 ReleaseLean tests at
    47,137/47,137 lines, 8,706/8,706 branches, and 3,421/3,421 methods. Gravitas
    passes 3,925 Release and 3,870 ReleaseLean tests at 43,911/43,911 lines,
    12,845/12,845 branches, and 4,510/4,510 methods. Standard and Lean packages
    build warning-free for both target frameworks. The focused optimization is
    complete. The final bounded follow-up above found no additional local change
    worth retaining and moved the broader signal to experimental capacity
    guidance.

- [`Scaled Mesh Query Normal`](done/2026-08-01-scaled-mesh-query-normal-plan.md)
  - Completed 2026-08-01. Mesh raycasts and swept-sphere queries now consume the
    face normal committed with their non-uniformly scaled triangle vertices. The
    unreferenced authored-normal cache and its per-mesh array allocation were
    deleted after a cross-stack caller audit.
  - Gravitas passes 3,925 Release and 3,870 ReleaseLean tests at 55,839/55,839
    lines, 15,829/15,829 branches, and 5,320/5,320 methods. Both corrected query
    paths remain at zero managed bytes after warmup, standard and Lean packages
    build warning-free for both targets, and the existing dense-mesh sweep
    benchmark remains comparable to baseline. The correctness queue is empty.

- [`Full-Domain Triangle-Pair Contact`](done/2026-07-31-full-domain-triangle-pair-contact-plan.md)
  - Completed 2026-08-01. FixedMathSharp owns one full-domain rigid-triangle
    contact relation; Gravitas retains deterministic BVH traversal and uses it
    as the sole concave mesh-pair authority. The narrowed scalar SAT, duplicate
    projection/ranking helpers, cached collision-triangle wrapper, synthetic
    witness fallback, and hollow wrapper tests were removed.
  - FixedMathSharp passes 2,649 Release and 2,628 ReleaseLean tests at
    47,095/47,095 lines, 8,698/8,698 branches, and 3,419/3,419 methods. Gravitas
    passes 3,923 Release and 3,868 ReleaseLean tests at 55,850/55,850 lines,
    15,833/15,833 branches, and 5,322/5,322 methods. Standard and Lean packages
    build warning-free for both target frameworks; all 72 named Gravitas
    allocation guards plus the direct FixedMathSharp relation guard pass. The
    dense concave throughput gap remains an explicit benchmark signal; the
    2026-08-02 follow-up above narrowed it after another repeatable improvement.
    The parity audit also captured the separate scaled-mesh query-normal issue.

- [`Exact Radial Segment Distance`](done/2026-07-31-radial-segment-distance-plan.md)
  - Completed 2026-07-31. FixedMathSharp segment APIs now solve circle and
    sphere intersections directly in authored physical distance while retaining
    the separate ray-parameter contract. Gravitas query and CCD families keep
    exact source/target trajectories through ordering and only then materialize
    normalized time; mixed proxy admission is geometric, ceiling-safe, and
    separate from contact-normal closing policy. Dead normalized-time wrappers
    and `RadialSweepAdmission` were removed.
  - FixedMathSharp passes 2,628 Release and 2,607 ReleaseLean tests at
    46,756/46,756 lines, 8,648/8,648 branches, and 3,413/3,413 methods. Gravitas
    passes 3,919 Release and 3,864 ReleaseLean tests at 56,012/56,012 lines,
    15,889/15,889 branches, and 5,348/5,348 methods. All 56 warmed allocation
    guards pass, radial and relative-CCD benchmark rows retain zero managed
    allocation, and independent closure review reported no findings.

- [`Full-Domain 3D Surface Projection`](done/2026-07-31-full-domain-3d-surface-projection-plan.md)
  - Completed 2026-07-31. FixedMathSharp now owns reusable exact planar
    relations and semantic surface anchors; Gravitas classifies complete X/Z
    collider projections independently from optional point materialization.
    Closest, directional, all-hit, and batch queries share one exact reducer and
    one partition-synchronized planar index with deterministic authored-order
    ties, grid-removal repair, Y-independent discovery, and zero warmed
    allocation.
  - FixedMathSharp passes 2,625 Release and 2,604 ReleaseLean tests at
    46,549/46,549 lines, 8,640/8,640 branches, and 3,400/3,400 methods. Gravitas
    passes 3,894 Release and 3,839 ReleaseLean tests at 43,971/43,971 lines,
    12,865/12,865 branches, and 4,531/4,531 methods. Dense and sparse
    projected-circle rows remain approximately 5.4 us from 8 through 1,024
    vertical cells with zero allocation. Translation-only updates of the unique
    widest candidate and the following query remain flat from 64 through 16,384
    entries when ordering is retained; independent closure review found no
    remaining critical or important issue.
- [`Full-Domain SolidBody Point Transform`](done/2026-07-30-full-domain-solid-body-point-transform-plan.md)
  - Completed 2026-07-30. `FixedTransform` now owns strict current-snapshot 3D
    and explicit X/Z point conversion. `SolidBody` and `SolidBody2D` expose
    dimensionally symmetric authoritative `GetWorldPoint` / `GetLocalPoint` APIs
    over committed collider owner scale, so mutable presentation state cannot
    enter deterministic simulation queries.
  - FixedMathSharp passes 2,603 Release and 2,582 ReleaseLean tests at
    44,334/44,334 lines, 8,393/8,393 branches, and 3,320/3,320 methods. Gravitas
    passes 3,870 Release and 3,815 ReleaseLean tests at 43,028/43,028 lines,
    12,779/12,779 branches, and 4,486/4,486 methods. Standard and Lean package
    builds are warning-free, all five focused ShortRun rows allocate zero
    managed bytes, and independent review reported no findings.
- [`Full-Domain Friction Response`](done/2026-07-29-full-domain-friction-response-plan.md)
  - Completed 2026-07-30. Gravitas now retains exact 3D cached Coulomb-disk
    accumulation, pure-2D Coulomb-line response, and mixed two-axis response
    whenever compact arithmetic cannot prove the complete operation safe. True
    final overflow rejects atomically; ordinary contacts keep the compact path,
    and warmed exact fallbacks allocate zero managed bytes.
  - Gravitas passes 3,861 Release and 3,806 ReleaseLean tests at 43,653/43,653
    lines, 12,775/12,775 branches, and 4,501/4,501 methods. Forty-two
    representative 3D, 2D, and mixed response benchmark rows report zero managed
    allocation without a gross compact-path regression.
- [`FixedMathSharp / Gravitas Ownership Boundary`](done/2026-07-28-fixedmathsharp-gravitas-ownership-boundary-plan.md)
  - Completed 2026-07-29. FixedMathSharp now owns reusable exact math, semantic
    geometry, and internal wide mechanics; Gravitas is its sole intentional
    non-test friend and owns rigid-body levers, mass semantics, normal/friction
    policy, and exact response. The pass removed intermediate v7 physics APIs,
    centralized proven wide duplication, and reorganized geometry by coherent
    owners without exposing raw wide types.
  - FixedMathSharp passes 2,590 Release and 2,569 ReleaseLean tests plus eight
    Chronicler tests in each mode at 49,416/49,416 authored lines, 8,340/8,340
    branches, and 3,238/3,238 methods. Gravitas passes 3,818 Release and 3,763
    ReleaseLean tests at 42,180/42,180 lines, 12,627/12,627 branches, and
    4,474/4,474 methods. Standard and Lean package-only validation is
    warning-free across both target frameworks; 38 focused anchor/response
    benchmark rows report zero managed allocation. The published-package relink
    and revalidation remain in the sequential release gates below.
- [`Exact Contact Lever And Mass Response`](done/2026-07-27-exact-contact-lever-response-plan.md)
  - Completed 2026-07-28. FixedMathSharp retains semantic 2D/3D point anchors,
    contact relations, and exact anchor-distance ordering without exposing raw
    wide arithmetic. Gravitas owns contact levers, mass points, positive
    weights, compact representable paths, and allocation-free exact fallback
    across 2D, 3D, mixed response, rotational CCD, compound mass properties, and
    embedded mixed boundaries. Runtime collider hierarchies are closed; host
    adapters author built-in geometry through
    `ColliderShapeDefinition*.CreateCollider()`. FixedMathSharp passes 2,638
    Release and 2,617 ReleaseLean tests at 47,462/47,462 lines, 8,732/8,732
    branches, and 3,500/3,500 methods. Gravitas passes 3,776 Release and 3,721
    ReleaseLean tests at 40,072/40,072 lines, 12,365/12,365 branches, and
    4,368/4,368 methods; replay, allocation, package, documentation, and
    independent-review gates are closed.
- [`Canonical Collider Geometry And Exact Scale Admission`](issue-tracker.md#grv-issue-061--finite-axis-collider-geometry-uses-canonical-rigid-frames)
  - Completed 2026-07-27. FixedMathSharp owns strict transform composition,
    fused scaled dimensions, centered finite-axis relations, local convex
    boundaries, and `FixedOrientedBox`; Gravitas publishes collider geometry
    transactionally and consumes canonical rigid-frame anchors across 2D, 3D,
    mixed, mesh, query, CCD, replay, response, and diagnostics. FixedMathSharp
    passes 2,575 Release and 2,554 ReleaseLean tests at exact 100% coverage.
    Gravitas passes 3,669 Release and 3,614 ReleaseLean tests at 37,548/37,548
    lines, 11,865/11,865 branches, and 4,246/4,246 methods; repeated replay and
    allocation gates are green. The later exact relative-frame throughput pass
    closes the ordinary-domain signal with repeatable `30.9-64.0%` improvements
    across the affected direct and Gravitas families.
- [`Full-Domain Conic Query And Triangle Arithmetic`](issue-tracker.md#grv-issue-054--cone-triangle-face-interiors-are-reduced-without-edge-crossings)
  - Completed 2026-07-22. FixedMathSharp now owns exact allocation-free
    finite-cone segment intervals for apex-authored and centered cones, while
    Gravitas consumes distance intervals for cone-collider raycasts and
    high-resolution point intervals for concave-mesh edge reduction.
    `FixedTriangle` separately owns exact projected containment plus
    face-interior cone hits even when no edge or cone axis crosses the triangle.
    Extreme coordinates, clipped opposite lobes, roots near half-even
    boundaries, non-cardinal authored endpoint contact, long-edge spatial
    witnesses, triangle feature ties, and tiny nonzero segments retain
    full-domain behavior. The authoritative Gravitas Release artifact passes
    3,238 tests at 100% line, branch, and method coverage; ReleaseLean passes
    3,183 tests. FixedMathSharp passes 1,714 Release and 1,693 ReleaseLean tests
    plus eight Chronicler tests in each configuration at exact 100% coverage.
    The final warmed cone raycast, concave-mesh overlap, mesh/cone collision,
    and oblique long/narrow bounds rows remain allocation-free.
- [`Body Motion Type And Solver Mobility Hardening`](done/2026-07-20-body-motion-type-and-solver-mobility-plan.md)
  - Completed 2026-07-20. Adds explicit Dynamic, Kinematic, and Static body
    roles while keeping translation and rotation freeze constraints independent
    across 2D, 3D, mixed response, constraints, partitions, CCD, serialization,
    replay, and host presentation. Atomic transition and pose contracts preserve
    runtime identity and reject invalid hierarchy-composed transforms before
    observable mutation. The authoritative Release artifact passes 3,237 tests
    at 100% line, branch, and method coverage; warmed 3D/2D role transitions
    remain allocation-free.
- [`Finite-Axis Full-Domain Projection Closure`](issue-tracker.md#grv-issue-048--finite-axis-capsule-cylinder-and-mesh-edge-projections-can-saturate-before-solving)
  - Completed 2026-07-19. FixedMathSharp now owns exact bounded-ray and
    authored-segment physical-distance capsule/cylinder intervals, and Gravitas
    consumes them across 2D, 3D, mixed, raycast, sweep, and mesh-edge reducers.
    The authoritative Gravitas Release artifact passes 3,103 tests at 100% line,
    branch, and method coverage; FixedMathSharp also retains exact 100%
    coverage. Final finite-axis benchmark rows remain allocation-free, while
    radial distance precision and the remaining conservative rim/support models
    stay explicitly queued as separate work.
- [`Rotational Moving-Pair CCD Hardening`](done/2026-07-18-rotational-moving-pair-ccd-plan.md)
  - Completed 2026-07-19. Pure 2D, pure 3D, and mixed rotational CCD now owns
    moving dynamic and kinematic targets through order-independent piecewise
    trajectories, stable normalized-time arbitration, contact-point angular
    response, and bounded atomic handoffs. The authoritative 3,056-test artifact
    reports 100% line, branch, and method coverage; the focused 1/8/32-pair
    benchmark remains approximately linear with only the separately tracked
    mixed broad-phase capacity-growth allocation signal.

- [`FixedMathSharp Foundation Hardening`](https://github.com/mrdav30/FixedMathSharp/blob/main/docs/feature-work/done/2026-07-14-fixedmathsharp-foundation-hardening-plan.md)
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

## Deferred / Evidence-Gated

The following work may be promoted into dated plans when measured risks or a
host-facing need appears.

- [`Mesh Tooling Simplification And Decomposition`](2026-06-17-mesh-tooling-simplification-and-decomposition-plan.md)
  - Offline decomposition, simplification, and richer mesh tooling can mature
    after the runtime collision boundary is solid.
- [`Mass Inertia Tooling And Diagnostics Follow-Up`](2026-06-19-mass-inertia-tooling-and-diagnostics-follow-up-plan.md)
  - Principal-axis tooling, COM markers, and richer mass-property payloads are
    useful when demand appears; the runtime already covers the core mass and
    inertia contract.
- [`Benchmark Publishing And CCD Diagnostics`](2026-06-21-benchmark-publishing-and-ccd-diagnostics-plan.md)
  - Publishing, baseline comparison, CI integration, and host-visible diagnostic
    polish remain evidence-gated until an explicit publication need exists.
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

1. Build one evolving strategy workload using the completed replay contract.
   Full-lifecycle replay and upstream develop CI alignment are verified. Measure
   comparable 2D/3D capacity, then use the same scenes for latency and memory
   soaks; evaluate #024 from observed work, not hypothetical query counts.
2. Extend physical-quality fixtures over meaningful durations and publish
   quality/time configuration guidance.
3. Keep trackers as intake buckets for reproduced defects and measured risks.
   Resolve defects in their owning repository and validate coordinated consumers
   with `UseLocalLsfStack=true`; preserve 100% reachable coverage.
4. Release dependencies before consumers, then run Gravitas `Release`,
   `ReleaseLean`, coverage, replay, allocation and relevant benchmark gates
   against the released package chain.
