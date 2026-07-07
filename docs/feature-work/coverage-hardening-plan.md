# Coverage Hardening Plan

**Date:** 2026-07-06  
**Status:** Active - branch coverage to 90%  
**Owner:** Gravitas coverage, test-quality, zombie-code, and branch-quality
hardening

---

> **For agentic workers:** Treat this as a living context guide until Gravitas
> reaches 100% line, branch, and method coverage. Steps use checkbox (`- [ ]`)
> syntax for tracking. Do not inflate coverage with shallow tests,
> generated-code chasing, stale compatibility paths, or tests for code that
> should be deleted.

## Goal

Long term, Gravitas should reach 100% line, branch, and method coverage with a
test suite that proves useful deterministic physics behavior.

The current release-hardening gate is narrower and more urgent:

- Keep line and method coverage at or above 90%.
- Raise branch coverage from 79.7% to at least 90%.
- Remove zombie code instead of testing it.
- Condense or delete duplicate tests while adding only high-signal branch tests.
- Record suspected bugs, parity gaps, stale behavior, and deeper RCA items in
  [`issue-tracker.md`](issue-tracker.md) instead of burying them in plan notes.
- After the 90% branch gate is met, refresh the remaining 100% roadmap from a
  fresh coverage and CRAP report.

## Measurement Source Of Truth

Use the repository coverage configuration, not ad hoc inline collector filters:

```powershell
dotnet test tests\Gravitas.Tests\Gravitas.Tests.csproj `
    --configuration Release `
    --collect:"XPlat Code Coverage" `
    --settings tests\Gravitas.Tests\coverlet.runsettings `
    --results-directory TestResults\coverage-branch-hardening
```

Generate a local report from the resulting Cobertura file:

```powershell
reportgenerator `
    "-reports:TestResults\coverage-branch-hardening\**\coverage.cobertura.xml" `
    "-targetdir:TestResults\coverage-branch-hardening\reports" `
    "-reporttypes:Html;TextSummary;MarkdownSummaryGithub;JsonSummary" `
    "-title:Coverage Branch Hardening"
```

Generate CRAP and method-gap data when prioritizing work:

```powershell
& C:\Users\david\.codex\skills\coverage-analysis\scripts\Compute-CrapScores.ps1 `
    -CoberturaPath TestResults\coverage-branch-hardening\**\coverage.cobertura.xml `
    -CrapThreshold 25 `
    -TopN 100

& C:\Users\david\.codex\skills\coverage-analysis\scripts\Extract-MethodCoverage.ps1 `
    -CoberturaPath TestResults\coverage-branch-hardening\**\coverage.cobertura.xml `
    -CoverageThreshold 90 `
    -BranchThreshold 90 `
    -Filter below-threshold `
    > TestResults\coverage-branch-hardening\method-gaps-under-90.json
```

If `TestResults\coverage-branch-hardening` contains prior run directories,
either delete the old run directories before collection or pass the newest
`coverage.cobertura.xml` file explicitly to the CRAP/method-gap scripts. Those
scripts expect one Cobertura document, while ReportGenerator can merge multiple
files.

The runsettings file excludes generated sources:

- `**/bin/**`
- `**/obj/**`
- `**/*.g.cs`
- `**/MemoryPack.Generator/**/*.cs`
- `GeneratedCodeAttribute`
- `CompilerGeneratedAttribute`
- `ExcludeFromCodeCoverageAttribute`

The CI workflow should use the same runsettings and ReportGenerator file
filters. If local and CI numbers disagree, fix the coverage command or
configuration before making source changes.

## Current Standing

Fresh checkpoint after Workstream 19 kinematic CCD and dynamic response residue
pass:

| Metric | Baseline | Current | Short-Term Gate | Long-Term Target |
| --- | ---: | ---: | ---: | ---: |
| Line coverage | 87.3% | 94.8% | 90% | 100% |
| Branch coverage | 74.1% | 83.2% | 90% | 100% |
| Method coverage | 86.5% | 94.6% | 90% | 100% |
| Tests | 974 passed | 1272 passed | green | green |

At the current denominator, the 90% branch gate requires at least 10,023 covered
branches. The latest run covered 9,268 of 11,137 branches, leaving roughly 755
net branch outcomes to cover or delete. Treat this as a focused branch
campaign, not a single gate-check workstream.

Current evidence:

- Coverage report:
  `TestResults/coverage-branch-hardening-ws19-final/reports/Summary.txt`
- Coverage collection:
  `dotnet test tests\Gravitas.Tests\Gravitas.Tests.csproj --configuration Release --collect:"XPlat Code Coverage" --settings tests\Gravitas.Tests\coverlet.runsettings`
  passed with 1272 tests.
- Branch shortlist:
  `TestResults/coverage-branch-hardening-ws19-final/branch-gap-shortlist.csv`
- Latest summary reports 199 uncovered methods.

## Historical Summary

This plan began after trigger collider hardening with 87.3% line, 74.1% branch,
and 86.5% method coverage. The campaign so far raised the suite to 94.7% line,
82.8% branch, and 94.6% method coverage while also finding real defects and
removing stale runtime branches.

| Phase | Coverage Result | Tests | Main Outcome |
| --- | --- | ---: | --- |
| Baseline | 87.3% line / 74.1% branch / 86.5% method | 974 | Starting point after trigger collider hardening. |
| First coverage campaign | 90.0% line / 76.0% branch / 91.0% method | 1025 | Line and method gates reached; branch campaign stayed active. |
| Roadmaps A-E | 92.0% line / 77.9% branch / 92.2% method | 1094 | Replay hash, diagnostics, dispatch, geometry, lifecycle, and collider authoring coverage. |
| Workstreams 1-6 | 93.3% line / 79.7% branch / 93.6% method | 1146 | Zombie-code sweep, query/collision/serialization/lifecycle/diagnostic branch hardening. |
| Workstreams 7-12 | 94.1% line / 81.3% branch / 94.4% method | 1214 | CCD handoff, mixed response, query reducer, replay, lifecycle, and shape-cast residue. |
| Workstreams 13-18 | 94.7% line / 82.8% branch / 94.6% method | 1261 | Contact geometry, hierarchy, convex support, CCD eligibility, mixed pair retention, and joint-island cleanup. |

High-value work completed:

- Query and collision coverage across 2D, 3D, and mixed dimensions, including
  capsule sweeps, compound shapes, mixed finite slabs/prisms, cone queries,
  convex sweeps, mesh contacts, 2D manifolds, and public batch-query contracts.
- Runtime lifecycle coverage for partition reset, retained membership cleanup,
  context reset, coroutine lifecycle, clock hooks, trigger-driven deferred
  refresh, inactive collider loading, stale collider-ID reuse safety, and
  hierarchy reparent/rejection behavior.
- Serialization and replay coverage for shape-definition equality/hash
  semantics, compound authored payloads, polygon serialization, invalid trigger
  load rejection, pending CCD handoff hash state, and authoritative-vs-cache
  hash distinctions.
- Constraint, ragdoll, grounding, diagnostics, trigger, material, logger,
  convex-support, mixed island root-key, and shape-exact CCD coverage where the
  behavior protects deterministic runtime edge cases.
- Real bug fixes found by coverage review: rotated cuboid raycasts now clip
  local slabs, 2D active-state changes clear mixed partitions, inactive 3D
  direct-collider loads clear stale partition state, and mixed CCD handoff
  queues now drain through one context-owned budget before partition/discrete
  completion.

Cleanup completed:

- Removed unused transient-state scaffolding, stale cuboid face-projection
  helpers, unused joint awake helpers, dead acceleration flags, stale
  immovable-contact state, duplicate mesh-cylinder fallback logic, unused mesh
  edge-normal storage, and unreachable CCD fallback helpers.
- Centralized duplicated voxel ordering, CCD contact-point selection, rotational
  hit tie-breaks, mixed bounds comparisons, and mesh embedded-volume reuse.
- Tightened cuboid SAT helper contracts, dynamic body dessimilation invariants,
  unbound collider transform writes, and joint-island lookup assumptions.
- Classified low-value diagnostic DTO getter/constructor noise as out of scope
  unless a future adapter invariant makes it meaningful.

## Hardening Rules

- Do not chase generated MemoryPack formatter coverage.
- Do not add tests just to touch lines. Tests must prove behavior, invariants,
  deterministic ordering, edge cases, replay safety, or regression risk.
- Prefer deleting truly dead code over testing it. If a path cannot be reached
  by a valid public/internal flow, remove it or record why it must stay.
- Prefer condensing duplicate tests over adding more cases. If multiple tests
  assert the same branch with different names, merge them into a clearer theory
  or delete the weaker duplicate.
- Do not preserve weak or legacy behavior only because a stale test asserts it.
  Update or delete stale tests when the stronger model is clear.
- Keep branch coverage honest. Avoid exclusions for hand-authored runtime code
  unless the code is platform-guarded, generated, or explicit diagnostic
  boilerplate with no meaningful branch behavior.
- When a suspected bug, parity issue, stale behavior, or deeper RCA item appears,
  add it to [`issue-tracker.md`](issue-tracker.md). If it is fixed immediately,
  add it under `Resolved Issues` with RCA and verification evidence.
- Performance signals belong in
  [`benchmark-signal-hardening-backlog.md`](benchmark-signal-hardening-backlog.md)
  unless they become confirmed runtime defects.
- Keep performance-sensitive tests allocation-aware when they touch hot paths.
  Use existing allocation assertion patterns for query, partition, CCD,
  diagnostics, and response loops when practical.
- Run focused tests first, then full `Release`, and rerun coverage at the end of
  each workstream. Use `ReleaseLean` when serialization, MemoryPack shims,
  public packages, or conditional compilation are touched.

## Zombie-Code Triage Protocol

Every branch-coverage pass should classify uncovered branch families before
adding tests:

| Classification | Action |
| --- | --- |
| Real behavior | Add focused tests through public APIs when readable, or small internal-policy tests when public setup hides the invariant. |
| Zombie code | Delete it, then run focused and full validation. |
| Duplicate code | Centralize or collapse it, then update tests around the shared path. |
| Defensive impossible branch | Replace with clearer validation or remove the branch if invariants already prevent it. |
| Suspected bug/parity gap | Record in `issue-tracker.md`, write a failing regression, fix root cause, then move/record as resolved. |
| Performance signal | Record in `benchmark-signal-hardening-backlog.md` unless it is already a confirmed correctness defect. |
| Low-value DTO/view getter noise | Leave it unless construction, visitor dispatch, serialization, or host-adapter behavior is meaningful. |

Do not let an issue hide inside a result bullet. The 3D inactive direct-collider
load parity bug is the model: it should be visible in the issue tracker with
RCA and verification, even if the fix is small.

## Active Branch-90 Roadmap

This roadmap replaces the old pattern of appending one new workstream after
each coverage attempt. Work through the workstreams below from the current
`ws19-final` shortlist. After each completed workstream, update checkboxes and
the coverage checkpoint table; do not add a new workstream unless fresh
evidence invalidates the next two planned areas or the 90% gate passes.

Run focused tests during each workstream. Run full `Release`, `ReleaseLean`
when relevant, and coverage at the end of each workstream or after a tightly
coupled pair of workstreams if the code changes share the same validation
surface. Use the fresh coverage result to choose which tasks inside the next
predefined workstream matter most, not to rewrite the whole plan.

### Workstream 19: Kinematic CCD And Dynamic Response Residue

**Purpose**

Collapse the largest current branch family around 2D/3D/mixed kinematic CCD
handoff, dynamic CCD pushes, rotational CCD, and continuous-response fallbacks.
These branches are release-critical because they decide fast-mover behavior.

**Candidate Areas**

- `src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Kinematic.cs`
- `src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.Kinematic.cs`
- `src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Dynamic.cs`
- `src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.Dynamic.cs`
- `src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Rotational.cs`
- `src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.Rotational.cs`
- `src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Hits.cs`
- `tests/Gravitas.Tests/CollisionHandling`
- `tests/Gravitas.Tests/Physics2D`
- `tests/Gravitas.Tests/MixedDimensions`

**Tasks**

- [x] Classify `ApplyKinematicContinuousCollisionHandoff` rows across 2D and
      3D as real behavior, obsolete fallback, or impossible guard.
- [x] Cover valid kinematic handoff branches where source/target state changes
      are observable through position, velocity, ignored-target, trigger, or
      sibling/hierarchy behavior.
- [x] Cover or delete `TryApplyKinematicDynamic3DContinuousCollisionPushes`,
      `TryApplyKinematicDynamic2DContinuousCollisionPushes`, and mixed
      kinematic push residue through public fast-mover scenarios.
- [x] Review rotational CCD residue in both dimensions; add behavior tests only
      for valid angular sweeps, no-hit/miss ordering, and replacement-hit
      semantics.
- [x] Review `ResolveSweptSphereContinuousNormal`,
      `TryApplyContinuousCollisionDynamicResponse`,
      `TryApplyContinuousCollisionMixed*Response`,
      `TryGetExactDynamicRelativeContinuousCollisionHit`, and
      `TryRefineShapeExactContinuousCollisionHit` for real behavior versus stale
      fallback code.
- [x] Run focused CCD tests, full `Release`, coverage collection, and
      `ReleaseLean` if conditional serialization or package behavior changes.

**Completion Notes**

Workstream 19 added public CCD coverage for kinematic handoff into
per-axis-frozen dynamic targets across 3D, pure 2D, and mixed 3D/2D paths. It
also added a dynamic sphere-source exact CCD theory for cuboid, cylinder, cone,
and convex mesh targets so swept-sphere target-normal branches are exercised
through real fast-mover behavior.

The cleanup removed private CCD branches proven impossible by caller/runtime
invariants: positive source-length rechecks, `sourceLength > Epsilon` ternaries
after positive-length callers, nonpositive impulse checks after positive
inverse mass plus closing velocity plus nonnegative restitution, and exact-hit
source-length/displacement guards where callers already prove motion. A
subagent review caught that positive frame-rate validation alone did not prove
`Context.DeltaTime > Epsilon`, so this workstream added
`PhysicsSettings.MaxResolvableFrameRate` and shared frame-rate validation before keeping
the removed CCD delta-time guards out of hot paths. Remaining rotational CCD
residue is valid behavior surface, not stale fallback.

### Workstream 20: Query Reducers And Shape-Cast Geometry Residue

**Purpose**

Tackle public query and shape-cast branches that still carry meaningful
geometry risk: mixed point-in-space reducers, finite slab side/cap reducers,
convex sweep hit normals, 2D convex/capsule sweep branches, and 3D swept cone
query residue.

**Candidate Areas**

- `src/Gravitas/Queries/Mixed/GravitasQueryMixedService.SphereAgainst2DReducers.cs`
- `src/Gravitas/Queries/Mixed/GravitasQueryMixedService.Support.cs`
- `src/Gravitas/Queries/Mixed/FiniteSlabProjectionSweep.cs`
- `src/Gravitas/Queries/3D/Sweeps/ConvexSweepQueryWorker.cs`
- `src/Gravitas/Queries/3D/Sweeps/SweptSphereQueryWorker.cs`
- `src/Gravitas/Queries/3D/RaycastSegmentWorker.cs`
- `src/Gravitas/Queries/2D/QueryDetection2D.cs`
- `src/Gravitas/Queries/3D/GravitasQuery3DService.Circle.cs`
- `src/Gravitas/Queries/3D/GravitasQuery3DService.Cone.cs`
- `tests/Gravitas.Tests/Queries`
- `tests/Gravitas.Tests/MixedDimensions`

**Tasks**

- [ ] Cover or simplify `TrySweepPointInSpace`, mixed target eligibility, and
      finite slab reducer rows through public mixed query scenarios.
- [ ] Cover mixed capsule/cylinder/cone/cuboid/triangle prism reducer residue
      where exact collision or query behavior is user-visible.
- [ ] Review `ConvexSweepQueryWorker.ResolveHitNormal`,
      `SweptSphereQueryWorker.TrySweepCone`, and 3D sweep-normal rows for
      supported shape-cast combinations.
- [ ] Cover 2D `TryConvexConvex`, `ClipSegment`,
      `TrySweepConvexMoverAgainstConvex`,
      `TrySweepCapsuleSegmentAgainstConvexEdges`, and `TryRaycastCircle`
      residue only through public 2D query or CCD flows when readable.
- [ ] Review `RaycastSegmentWorker.CheckOBBoxOverlaps` and remaining overlap
      sphere hit-construction rows; delete or classify any view-construction
      residue that does not protect runtime behavior.
- [ ] Run focused query tests, allocation-sensitive query checks where relevant,
      full `Release`, and coverage collection.

### Workstream 21: Mixed Pair, Contact Notification, And Response Lifecycle

**Purpose**

Clean up the service-level lifecycle rows that remain after the CCD and query
passes: mixed candidate processing, untouched-pair retention, 2D existing-pair
reuse, contact notification branches, and response/event parity.

**Candidate Areas**

- `src/Gravitas/Core/Mixed/GravitasMixedCollisionService.Pairs.cs`
- `src/Gravitas/Core/2D/GravitasPhysics2DService.Pairs.cs`
- `src/Gravitas/Core/2D/GravitasPhysics2DService.Grounding.cs`
- `src/Gravitas/Colliders/3D/LSCollider.cs`
- `src/Gravitas/Colliders/2D/LSCollider2D.cs`
- `src/Gravitas/CollisionHandling/Response/Mixed/CollisionResponseMixed.cs`
- `tests/Gravitas.Tests/MixedDimensions`
- `tests/Gravitas.Tests/Physics2D`
- `tests/Gravitas.Tests/CollisionHandling`

**Tasks**

- [ ] Revisit `ProcessCandidate`, `TryKeepUntouchedPair`, and
      `TryAddExistingResponsePair` after Workstreams 19-20; cover only real
      trigger, sleeping, inactive, bounds-miss, retained-pair, or island flows.
- [ ] Cover 3D/2D `NotifyContact` branches through symmetric enter/stay/exit
      contact and trigger scenarios where parity is expected.
- [ ] Review mixed response impulse branch residue after query reducer cleanup;
      add tests only for physical behavior such as frozen axes, bodyless
      participants, friction/restitution, or constrained 2D vertical response.
- [ ] Review `RefreshGroundingFromDiscreteResponse` residue against current 2D
      grounding/support semantics; avoid duplicating existing support tests.
- [ ] Record any event-ordering or pair-retention parity bug in
      `issue-tracker.md` before fixing it.
- [ ] Run focused mixed/2D response tests, full `Release`, and coverage
      collection.

### Workstream 22: Collision Geometry, Convex Support, And Contact Residue

**Purpose**

Address geometry helpers that still rank high enough to matter but are easy to
over-test poorly: convex support triangles, cuboid normals, 2D collision
clipping, contact normal fallback, and mixed exact prism branches.

**Candidate Areas**

- `src/Gravitas/CollisionHandling/Detection/3D/ConvexColliderSupport.cs`
- `src/Gravitas/CollisionHandling/Detection/2D/CollisionDetection2D.cs`
- `src/Gravitas/CollisionHandling/Detection/Mixed`
- `src/Gravitas/CollisionHandling/Response/3D/CollisionResponse.cs`
- `src/Gravitas/CollisionHandling/Response/2D/CollisionResponse2D.cs`
- `src/Gravitas/Colliders/3D/LSCuboidCollider.cs`
- `tests/Gravitas.Tests/CollisionHandling`
- `tests/Gravitas.Tests/MixedDimensions`

**Tasks**

- [ ] Review `ConvexColliderSupport.UpdateTriangle` for meaningful simplex
      behavior versus unreachable/defensive state.
- [ ] Cover supported `LSCuboidCollider.GetNormalAtPoint` edge/face/corner
      semantics if they affect collision/query behavior; otherwise simplify the
      helper contract.
- [ ] Cover 2D `TryConvexConvex` and `ClipSegment` branches through manifold or
      query scenarios that prove physical contact behavior.
- [ ] Revisit mixed prism exact branches not already handled in Workstream 20
      if they still rank highly.
- [ ] Review contact-normal fallback rows in 2D/3D response; delete impossible
      branches or add regression tests for physically valid zero-normal/contact
      edge cases.
- [ ] Run focused collision geometry/response tests, full `Release`, and
      coverage collection.

### Workstream 23: Serialization, Replay, Authoring, And Partition Lifecycle

**Purpose**

Clean branch residue around state loading, replay hash variation, retained
partition cleanup, dynamic partition membership, hierarchy replay ordinals, and
shape authoring helpers.

**Candidate Areas**

- `src/Gravitas/Colliders/3D/LSCollider.cs`
- `src/Gravitas/Colliders/2D/LSCollider2D.cs`
- `src/Gravitas/Colliders/Hierarchy/ColliderHierarchyState.cs`
- `src/Gravitas/Core/2D/GravitasPhysics2DService.ReplayHash.cs`
- `src/Gravitas/Core/Mixed/GravitasMixedCollisionService.ReplayHash.cs`
- `src/Gravitas/Partitions/3D/PhysicsPartition.cs`
- `src/Gravitas/Partitions/2D/PhysicsPartition2D.cs`
- `src/Gravitas/Core/3D/GravitasCollisionService.cs`
- `src/Gravitas/Core/2D/GravitasCollision2DService.cs`
- `src/Gravitas/Core/Mixed/GravitasMixedCollisionService.Partitioning.cs`
- `tests/Gravitas.Tests/Serialization`
- `tests/Gravitas.Tests/Replay`
- `tests/Gravitas.Tests/CollisionHandling`
- `tests/Gravitas.Tests/Physics2D`

**Tasks**

- [ ] Cover or simplify 2D/3D `ApplyLoadedState` residue through load/default
      semantics that matter for deterministic continuation.
- [ ] Review replay hierarchy ordinal branches and `RemoveChild` residue for
      valid hierarchy mutation behavior.
- [ ] Cover replay hash service rows only where different authoritative state
      should produce different hashes; avoid duplicate hash-equality padding.
- [ ] Review partition `Distribute`, `RemoveDynamicObject`, and retained
      partition detach/retire branches for real lifecycle behavior and
      allocation-sensitive cleanup.
- [ ] Capture any load/replay parity bug in `issue-tracker.md` before fixing.
- [ ] Run focused serialization/replay/partition tests, full `Release`,
      `ReleaseLean`, and coverage collection.

### Workstream 24: Constraint Solver And Ragdoll Residue

**Purpose**

Handle remaining constraint branch families after the core runtime passes:
2D/3D joint row creation, hinge/limit/motor branches, solver metrics, and joint
serialization edge cases.

**Candidate Areas**

- `src/Gravitas/Constraints/2D/JointSolver2D.cs`
- `src/Gravitas/Constraints/3D/JointSolver3D.cs`
- `src/Gravitas/Constraints/2D/Joint2D.cs`
- `src/Gravitas/Constraints/3D/Joint3D.cs`
- `src/Gravitas/Core/2D/GravitasPhysics2DService.Response.cs`
- `src/Gravitas/Core/3D/GravitasPhysicsService.Response.cs`
- `tests/Gravitas.Tests/Constraints`

**Tasks**

- [ ] Cover `JointSolver2D.AddDistanceRow` and `JointSolver3D.AddHingeLimitRow`
      branches through valid pin/distance/hinge/limit scenarios.
- [ ] Review joint `RecordData` residue for version-tolerant load semantics and
      invalid payload handling.
- [ ] Cover disabled, body-frozen, static-only, and no-solver-participant joint
      branches only through public service registration/removal/simulation
      flows.
- [ ] Extend stress tests only if they reveal measurable solver stability or
      allocation value; do not add slow long-chain cases for branch count alone.
- [ ] Run focused constraint tests, allocation-sensitive constraint checks,
      full `Release`, `ReleaseLean`, and coverage collection.

### Workstream 25: Low-Value Surface Audit And Branch-90 Gate

**Purpose**

Make the final push to the 90% branch gate after the behavior-heavy workstreams
above. This is the place to audit remaining DTO/view getter noise, diagnostic
draw command branches, small property wrappers, and any residual low-risk
surface before deciding whether to test, delete, or explicitly leave it.

**Candidate Areas**

- `src/Gravitas/Diagnostics`
- `src/Gravitas/Support`
- remaining low-count rows from the newest `branch-gap-shortlist.csv`
- `docs/feature-work/issue-tracker.md`
- `docs/feature-work/benchmark-signal-hardening-backlog.md`

**Tasks**

- [ ] Generate fresh coverage, CRAP, and method-gap artifacts after Workstreams
      19-24 or earlier if branch coverage crosses 90%.
- [ ] Classify all remaining top branch rows as behavior, zombie, duplicate,
      impossible guard, low-value DTO/view noise, or feature-worthy defect.
- [ ] Cover only diagnostic visitor/adapter dispatch, draw-command payload, or
      lifecycle behavior that matters to host integration.
- [ ] Delete or simplify stale wrappers and impossible guards discovered in the
      final audit.
- [ ] Confirm line and method coverage remain above 90%.
- [ ] Run full `Release`, full `ReleaseLean`, coverage collection, CRAP/method
      gap scripts, `git diff --check`, and a final independent review.
- [ ] If branch coverage is at least 90%, refresh this document into the 100%
      coverage roadmap. If not, add one evidence-based follow-up workstream
      from the newest shortlist instead of expanding the document piecemeal.

## Coverage Checkpoints

Update this compact table after each coverage collection. Keep only meaningful
campaign checkpoints; do not add a row for every focused test filter.

| Date | Line | Branch | Method | Tests | Notes |
| --- | ---: | ---: | ---: | --- | --- |
| 2026-07-05 | 87.3% | 74.1% | 86.5% | 974 passed | Baseline captured with `tests/Gravitas.Tests/coverlet.runsettings` after trigger collider hardening. |
| 2026-07-05 | 90.0% | 76.0% | 91.0% | 1025 passed | First campaign completed; line and method gates met. |
| 2026-07-05 | 92.0% | 77.9% | 92.2% | 1094 passed | Roadmaps A-E completed across mixed query support, replay hash, diagnostics, collision dispatch, geometry, lifecycle, and collider authoring. |
| 2026-07-06 | 93.3% | 79.7% | 93.6% | 1146 passed | Workstreams 1-6 completed; zombie-code sweep plus query, collision, serialization, lifecycle, and diagnostics hardening. |
| 2026-07-07 | 94.1% | 81.3% | 94.4% | 1214 passed | Workstreams 7-12 completed; CCD handoff, mixed response, query reducer, replay, lifecycle, and shape-cast residue. |
| 2026-07-07 | 94.7% | 82.8% | 94.6% | 1261 passed | Workstreams 13-18 completed; geometry, hierarchy, convex support, CCD eligibility, mixed pair retention, and joint-island cleanup. |
| 2026-07-07 | 94.8% | 83.2% | 94.6% | 1272 passed | Workstream 19 completed; kinematic CCD frozen-axis coverage plus fixed-step frame-rate invariant. Branches covered: 9268/11137. |
