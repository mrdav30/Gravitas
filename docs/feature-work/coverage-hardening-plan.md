# Coverage Hardening Plan

**Date:** 2026-07-06  
**Status:** Active - branch coverage hardening phase  
**Owner:** Gravitas coverage, test-quality, zombie-code, and branch-quality
hardening

---

> **For agentic workers:** Treat this as a living context guide until Gravitas
> reaches 100% line, branch, and method coverage. Update the current standing
> and progress log after each focused coverage pass. Do not inflate coverage
> with shallow tests, generated-code chasing, stale compatibility paths, or
> tests for code that should be deleted.

## Goal

Long term, Gravitas should reach 100% line, branch, and method coverage with a
test suite that proves useful deterministic physics behavior.

The current release-hardening gate is narrower and more urgent:

- Keep line and method coverage at or above 90%.
- Raise branch coverage from 77.9% to at least 90%.
- Remove zombie code instead of testing it.
- Condense or delete duplicate tests while adding only high-signal branch tests.
- Record suspected bugs, parity gaps, stale behavior, and deeper RCA items in
  [`issue-tracker.md`](issue-tracker.md) instead of burying them in plan notes.

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

Fresh checkpoint after Workstream 5 service lifecycle, partition, and runtime
hardening:

| Metric | Baseline | Current | Short-Term Gate | Long-Term Target |
| --- | ---: | ---: | ---: | ---: |
| Line coverage | 87.3% | 93.2% | 90% | 100% |
| Branch coverage | 74.1% | 79.6% | 90% | 100% |
| Method coverage | 86.5% | 93.4% | 90% | 100% |
| Tests | 974 passed | 1137 passed | green | green |

Current evidence:

- Coverage report:
  `TestResults/coverage-branch-hardening-ws5/reports/Summary.txt`
- Coverage collection:
  `dotnet test tests\Gravitas.Tests\Gravitas.Tests.csproj --configuration Release --collect:"XPlat Code Coverage" --settings tests\Gravitas.Tests\coverlet.runsettings`
  passed with 1137 tests.
- Branch shortlist:
  `TestResults/coverage-branch-hardening-ws5/branch-gap-shortlist.txt`
- Latest CRAP extraction reported 34 flagged methods and 248 uncovered
  methods.

## Completed Coverage Summary

The first coverage campaign, branch inventory sweep, and collision geometry
hardening moved the project from 87.3% line, 74.1% branch, and 86.5% method
coverage to 93.0% line, 79.2% branch, and 93.0% method coverage.

High-value work completed:

- Query and 2D collision edge coverage for capsule sweeps, compound 2D
  collision, mixed compound boundaries, and query request/range contracts.
- Partition retained-membership cleanup coverage across 3D, pure 2D, and mixed
  partitions.
- Deterministic ordering coverage for 2D grounding candidates and continuous
  collision hit replacement.
- 3D raycast, mesh, convex, and collision dispatch branch coverage.
- 2D compound geometry behavior and compound-vs-compound manifold coverage.
- Mixed compound part-selection coverage and 3D cone-convex reversed/negative
  dispatch coverage.
- Stale SAT/mesh fallback cleanup for unused polygon projection and duplicate
  mesh-cylinder candidate logic.
- Batch query API coverage for range/output contracts without duplicating every
  scalar query.
- Diagnostic sink, debug draw, typed event, joint, ragdoll, and mixed diagnostic
  branch coverage.
- Constraint and replay hash branch coverage for 2D/3D joint limits, motor
  branches, authoritative-vs-cache distinctions, hierarchy replay ordinal
  normalization, and mixed pair payloads.
- Service lifecycle and collider authoring coverage for hierarchy detach,
  cross-dimensional parent cleanup, compound 2D authoring, ragdoll pose-target
  helpers, settings saver branches, and inactive 2D/3D collider load cleanup.
- Rotated cuboid raycast regression coverage after the inventory sweep exposed
  that oriented-box raycasts clipped the enclosing AABB instead of cuboid-local
  slabs.
- Query reducer and shape-cast branch coverage for convex source sweeps against
  compound 3D targets, concave triangle cone-axis hits, mixed conservative
  fallback reducers, pure 2D convex mover/capsule CCD, and direct raycast worker
  point/rotated-box geometry.
- Serialization, replay, and authoring coverage for 2D/3D shape-definition
  equality/hash semantics, compound-part replay hash payloads, 2D polygon shape
  serialization, and inactive mixed-partition cleanup.
- Service lifecycle, partition, and runtime coverage for retained partition
  reset idempotency, context reset collider/partition cleanup, coroutine
  reset/deactivate disposal, clock hook ordering, and 2D deferred partition
  refresh from distribution-time trigger callbacks.

Cleanup completed:

- Removed unused `ITransient`, `TransientAttribute`, and
  `TransientStateUtility` scaffolding.
- Removed stale cuboid face-projection helpers including the allocating
  `GetFace(...)` helper.
- Removed unused `Joint2D.HasAwakeParticipant()` and
  `Joint3D.HasAwakeParticipant()`.
- Fixed 3D direct-collider inactive load so loaded inactive state clears stale
  primary and mixed partition state just like 2D.
- Removed stale 3D immovable-contact-direction state from contact manifolds,
  collision pairs, and replay hashing after confirming response no longer reads
  it.
- Removed the unused `LSCollider.HeightPos` shortcut; body-owned `HeightPos`
  remains the authoritative 3D height state.
- Removed dead `SolidBody.UpdateAcceleration()` state and the never-read
  `_decelerating` / `_isVelocityConstant` serialization and replay fields.
- Removed unused `PhysicsMesh.EdgeNormals` storage and the allocating
  `GetTriangleAtIndex(...)` wrapper, keeping the allocation-free
  `GetTriangleVertices(..., out ...)` path.
- Fixed `LSCollider2D.IsActive` so mixed-mode bodyless 2D colliders refresh and
  clear mixed partition membership along with pure 2D partition membership.
- Centralized 2D/3D/mixed `WorldVoxelIndex` ordering and removed duplicated
  impossible-null partition comparer branches.

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

## Active Branch Coverage Campaign

### Workstream 1: Fresh Branch Inventory And Zombie-Code Sweep

**Status:** Complete - 2026-07-06

**Purpose**

Build the next branch-coverage target list from current evidence, then remove
or track zombie code before writing more tests.

**Files To Inspect**

- `TestResults/coverage-branch-hardening/reports/Summary.txt`
- `TestResults/coverage-branch-hardening/method-gaps-under-90.json`
- `TestResults/coverage-branch-hardening/branch-gap-shortlist.txt`
- `src/Gravitas`
- `tests/Gravitas.Tests`
- `docs/feature-work/issue-tracker.md`
- `docs/feature-work/benchmark-signal-hardening-backlog.md`

**Tasks**

- [x] Rerun coverage into `TestResults\coverage-branch-hardening`.
- [x] Regenerate ReportGenerator, CRAP, and method-gap artifacts.
- [x] Sort the remaining branch gaps by CRAP score, uncovered branch count,
      runtime risk, and hot-path relevance.
- [x] For each top candidate, classify it using the zombie-code triage
      protocol before writing tests.
- [x] Delete or simplify obvious zombie code immediately when it has no valid
      runtime path.
- [x] Record suspected bugs or parity smells in `issue-tracker.md`; if fixed in
      the same pass, record them under `Resolved Issues`.
- [x] Update this plan with the new prioritized branch campaign list.

**Exit Criteria**

- A current branch-gap shortlist exists.
- Any discovered correctness risk is visible in the issue tracker.
- Obvious zombie code is deleted instead of carried forward.

**Outcome**

- Coverage after cleanup: 92.1% line, 77.9% branch, 92.4% method.
- Tests after cleanup: coverage collection passed with 1095 tests.
- CRAP extraction after cleanup: 41 flagged methods, down from 42.
- Method-gap extraction after cleanup: 285 uncovered methods, down from 292.
- The inventory sweep found and fixed the rotated cuboid raycast local-slab
  bug, and recorded the mesh-cuboid fallback SAT completeness concern in
  `issue-tracker.md` for focused RCA.

## Current Branch Campaign Shortlist

Use this list as the next-pass ordering, then re-rank after each workstream:

1. **CCD Handoff And Shape-Exact Refinement Families**
   - 3D/2D kinematic-dynamic handoff helpers.
   - `TryResolveRotationalContinuousCollision(...)` in 3D and 2D.
   - `TryRefineShapeExactContinuousCollisionHit(...)`.
   - `ProcessQueuedContinuousCollisionHandoffs(...)`.
   - Classification: real runtime behavior. High risk; cover with focused
     deterministic CCD scenarios rather than private-helper branch steering.
2. **Mixed Prism And Response Families**
   - `TryTestCuboidPrism(...)`: missed 8 / 28 branches.
   - `TryTestCapsulePrism(...)`, `TryTestCylinderPrism(...)`, and
     `TryTestConePrism(...)`: each missed 8 / 20 branches.
   - `CollisionResponseMixed.Resolve(...)`, mixed candidate processing, and
     mixed trigger/contact event branches.
   - Classification: real mixed behavior. Cover with physical pair scenarios
     and avoid duplicating tests that only repeat shape families.
3. **3D Query And Shape-Cast Families**
   - `RaycastSegmentWorker.CheckOBBoxOverlaps(...)`: missed 18 / 24 branches
     after the local-slab bug fix; remaining coverage should target meaningful
     local slab edge cases rather than restoring the old AABB behavior.
   - Cone hit construction and concave-mesh cone paths.
   - `ConvexSweepQueryWorker` compound target, hit point/normal, and reducer
     ordering paths.
   - Zero-length sphere ray behavior in `RaycastSegmentWorker` remains a
     defensive worker branch because public query APIs reject zero-length rays.
   - Classification: real query behavior. Cover through public query APIs and
     keep scalar/batch tests non-duplicative.
4. **Collision Geometry Families**
   - Mesh-cuboid, mesh-cylinder, and mesh-cone fallback/contact branches.
   - 2D convex/compound collision and manifold replacement branches.
   - `ConvexColliderSupport` simplex update branches.
   - Classification: real geometry behavior until proven otherwise. Delete only
     after demonstrating no valid collider setup can reach the fallback.
5. **Service Lifecycle And Partition Families**
   - Deferred 2D partition refresh, retained mixed membership reset, partition
     order comparers, and collider load-state branches.
   - Classification: real lifecycle/pooling behavior. Add tests only for
     observable ownership and idempotency guarantees.
6. **Serialization, Replay, And Authoring Residue**
   - Authoring equality/hash hot spots were covered in Workstream 4.
   - Remaining replay branches are mostly body/cache mode distinctions and
     hierarchy ordinal failure paths; cover only through valid replay or load
     semantics, not reflection.
   - Hierarchy serialization remains host-owned unless a future feature plan
     deliberately changes that boundary.
7. **Low-Value Diagnostic/View Getter Noise**
   - Immutable diagnostic view getters and simple constructor branches remain
     lower priority unless they affect visitor dispatch, disabled-path behavior,
     or host adapter data.

### Workstream 2: Collision And Shape Geometry Branches

**Status:** Done

**Purpose**

Close high-risk collision/geometry branch gaps that affect physical behavior,
while deleting stale fallback paths that no valid collider shape can reach.

**Likely Candidate Areas**

- `src/Gravitas/CollisionHandling/Detection/3D`
- `src/Gravitas/CollisionHandling/Detection/2D/CollisionDetection2D.cs`
- `src/Gravitas/CollisionHandling/Detection/Mixed/CollisionDetectionMixed*.cs`
- `src/Gravitas/CollisionHandling/Detection/3D/ConvexColliderSupport.cs`
- `tests/Gravitas.Tests/CollisionHandling`

**Tasks**

- [x] Cover or delete remaining mixed prism branch gaps for cuboid, capsule,
      cylinder, cone, and triangle/mesh slab paths.
- [x] Review 2D convex/compound collision branches for duplicate tests and
      unreachable defensive branches.
- [x] Review 3D mesh/cuboid/cylinder/capsule branches for measured behavior
      gaps rather than private-helper branch steering.
- [x] If GJK/simplex branches remain high-risk, refactor only the simplex policy
      needed for direct deterministic tests; avoid brittle geometry setups that
      exist only for coverage.
- [x] Run focused collision tests, then full `Release`, then coverage.

**Outcome**

- Removed the unused `AxisProjectionHelper.ProjectPolygonOntoAxis(...)`
  overload for `SwiftList<Vector3d>`.
- Removed duplicate mesh-cylinder fallback candidate logic from
  `CollisionDetection.Mesh`; `MeshTriangleContactGenerator` already owns the
  same candidate/contact predicate and returns no weaker result.
- Added focused behavior coverage for `LSCompoundCollider2D.ContainsPoint`,
  `LSCompoundCollider2D.GetSupportPoint`, 2D compound-vs-compound manifold
  contacts, mixed 2D compound shallowest-part selection, and reversed/negative
  cone-convex dispatch.
- Reviewed remaining mixed prism SAT branch misses. The remaining high CRAP
  entries are mostly private defensive separating-axis exits where the public
  broad-phase or earlier SAT axes reject invalid setups before those exact
  branches can be observed. They remain classified as geometry guardrails, not
  current release behavior gaps.
- Focused collision slice passed with 106 tests. Full Release coverage passed
  with 1105 tests and reported 92.5% line, 78.4% branch, and 92.5% method
  coverage.

### Workstream 3: Query Reducer And Shape-Cast Branches

**Status:** Complete - 2026-07-06

**Purpose**

Raise branch coverage around query result ordering, reducer classification,
shape-cast exactness, and cone/convex sweep paths without adding duplicate
scalar-vs-batch tests.

**Likely Candidate Areas**

- `src/Gravitas/Queries/3D/GravitasQuery3DService.Cone.cs`
- `src/Gravitas/Queries/3D/Sweeps/ConvexSweepQueryWorker.cs`
- `src/Gravitas/Queries/3D/RaycastSegmentWorker.cs`
- `src/Gravitas/Queries/Mixed`
- `src/Gravitas/Queries/2D/QueryDetection2D.cs`
- `tests/Gravitas.Tests/Queries`
- `tests/Gravitas.Tests/MixedDimensions`

**Tasks**

- [x] Cover cone query branches only where gameplay-facing behavior changes:
      axis hits, cap/rim hits, triangle intersections, rejected candidates, and
      conservative reducer markers.
- [x] Review `ConvexSweepQueryWorker` uncovered branches for true exact
      shape-cast behavior versus stale compound fallback.
- [x] Cover mixed sphere/2D reducer branches that still decide exact vs
      conservative hit reporting.
- [x] Remove duplicate batch tests that only repeat scalar query correctness.
- [x] Run focused query tests, allocation-sensitive query checks where
      relevant, full `Release`, then coverage.

**Result Notes**

- Added behavior coverage for convex mesh source sweeps against compound 3D
  targets, keeping target owner identity while selecting the nearest part
  geometry. `ConvexSweepQueryWorker.TrySweepTargetCompound(...)` moved from
  uncovered to fully line-covered with 83.3% branch coverage.
- Added cone overlap coverage for a concave mesh triangle crossing the cone
  axis. This covers the axis/triangle intersection path without adding private
  helper tests.
- Added mixed swept-sphere coverage for unsupported 2D collider fallback
  reducers and capsule slab starting overlap. This documents exact-vs-
  conservative reducer reporting through the public mixed query surface.
- Added pure 2D mover-shape sweep coverage for polygon movers against capsule
  targets. `QueryDetection2D.TrySweepConvexMoverAgainstCapsule(...)` now has
  100% line and branch coverage through the live CCD helper path.
- Added direct `RaycastSegmentWorker` coverage for point-inside sphere,
  point-inside finite cylinder, and rotated cuboid point checks. This raised
  raycast-worker coverage while keeping the tests focused on reusable query
  geometry rather than batch plumbing.
- Extracted reusable unsupported 2D/3D test colliders into
  `tests/Gravitas.Tests/Support/UnsupportedTestColliders.cs` so mixed query
  fallback tests do not duplicate private fake collider implementations. Final
  review tightened the unsupported 2D helper to clamp fallback closest points to
  its authored bounds and assert the resulting mixed contact points.
- Avoided adding a conservative fallback miss test after validation showed the
  public broad phase rejects that setup before the reducer runs. Keeping it
  would have asserted a candidate-gathering behavior, not the reducer branch.
- Focused query/reducer slice passed with 150 tests. Full Release coverage
  passed with 1118 tests and reported 92.8% line, 78.7% branch, and 92.7%
  method coverage.

### Workstream 4: Serialization, Replay, And Authoring Branches

**Status:** Complete - 2026-07-06

**Purpose**

Close branch gaps where serialized state, replay hashes, shape definitions, and
authoring objects decide deterministic continuation behavior.

**Likely Candidate Areas**

- `src/Gravitas/Colliders/Definitions`
- `src/Gravitas/Colliders/2D/*ReplayHash.cs`
- `src/Gravitas/Colliders/3D/*ReplayHash.cs`
- `src/Gravitas/CollisionHandling/Pairs`
- `src/Gravitas/Materials`
- `tests/Gravitas.Tests/Serialization`
- `tests/Gravitas.Tests/Determinism`
- `tests/Gravitas.Tests/Colliders`

**Tasks**

- [x] Review `ColliderShapeDefinition` equality/hash branches for meaningful
      authored-shape behavior; delete weak comparison paths if the API no
      longer needs them.
- [x] Cover replay hash branches that distinguish authoritative state from
      solver-cache or diagnostic state.
- [x] Audit inactive/active load, trigger, material, ignored-layer, hierarchy,
      and compound-part serialization parity between 2D and 3D.
- [x] Record any parity bug in `issue-tracker.md` before fixing it.
- [x] Run focused serialization/determinism tests, `ReleaseLean`, full
      `Release`, then coverage.

**Result Notes**

- Added compact 3D and 2D shape-definition equality/hash tests that prove
  authored kind, dimensions, material presence/value, mesh inertia policy,
  vertex count/order, triangle count/order, and polygon vertex order affect
  value semantics.
- Added context-level replay hash coverage for compound 2D and 3D authored
  shape payloads. The 2D case now covers circle, capsule, AABB, convex polygon,
  part-level material, and shape-level material contribution through
  `ComputeReplayHash()`.
- Added 2D polygon serialization coverage that loads into a target polygon with
  a different vertex count, then verifies rebuilt vertices, bounds, mass
  properties, and next-frame force replay.
- Strengthened inactive collider serialization tests so 3D and 2D targets in a
  mixed runtime clear mixed partition state as well as primary partition state.
- Tightened the final review gaps by asserting old mixed partitions drop the
  collider ID, reactivated bodyless 2D colliders are visible in mixed partition
  membership again, equivalent independently rebuilt compound payloads keep the
  same replay hash, and compound part offset/rotation/scale payload changes are
  encoded.
- Found and fixed a real parity bug: `LSCollider2D.IsActive` cleared/refreshed
  pure 2D partitions but left mixed partition membership stale. The red
  regression and RCA are recorded in `issue-tracker.md`.
- Focused serialization/replay/authoring slice passed with 57 tests. Full
  Release coverage passed with 1125 tests and reported 93.0% line, 79.2%
  branch (9033 / 11401 covered branches), and 93.0% method coverage.

### Workstream 5: Service Lifecycle, Partition, And Runtime Branches

**Status:** Complete - 2026-07-06

**Purpose**

Close branch gaps in runtime ownership, deferred refresh, lifecycle hooks,
partition cleanup, and coroutine/wait behavior.

**Likely Candidate Areas**

- `src/Gravitas/Core/2D`
- `src/Gravitas/Core/3D`
- `src/Gravitas/Core/Mixed`
- `src/Gravitas/Runtime/GravitasWorldContext.cs`
- `src/Gravitas/Partitions`
- `src/Gravitas/Support`
- `tests/Gravitas.Tests/Core`
- `tests/Gravitas.Tests/Runtime`
- `tests/Gravitas.Tests/Partitions`

**Tasks**

- [x] Review deferred partition refresh branches in 2D/3D/mixed services for
      valid lifecycle paths, duplicate work, and stale cleanup logic.
- [x] Cover service reset/deactivate/idempotency branches that protect pooling
      and context reuse.
- [x] Cover queued CCD handoff branches only where they affect deterministic
      runtime behavior.
- [x] Delete lifecycle branches that only preserve obsolete initialization
      orders.
- [x] Run focused runtime/partition tests, full `Release`, then coverage.

**Result Notes**

- Added retained partition reset and empty-copy idempotency coverage for 3D,
  pure 2D, and mixed partitions without asserting implementation-only pool
  internals.
- Added context reset parity coverage showing 3D and 2D collider registries
  clear IDs, partition flags, and partition coordinates when the owning context
  resets. Direct collision-service reset remains a service-cache boundary; the
  physics registries own collider state cleanup.
- Added coroutine lifecycle coverage for rejecting foreign handles, ignoring
  repeated stops, and disposing active iterators during `Reset`, `Initialize`,
  and `Deactivate`.
- Added deterministic clock and hook coverage for frame-rate conversion,
  reset-hook ordering, hook disposal idempotency, and frame-rate changed hooks.
- Added a distribution-time 2D trigger callback test that mutates and rebuilds
  a third collider, proving deferred shape refresh is flushed before the next
  query boundary.
- Centralized duplicated 2D/3D/mixed partition voxel ordering into
  `WorldVoxelIndexOrdering`, preserving 2D planar X/Z/Y order and 3D/mixed
  X/Y/Z order while deleting impossible null comparer branches.
- Reviewed context-level queued CCD handoff coverage. Existing 2D and 3D
  dynamic-chain tests cover service queue budget and limit behavior; no
  additional private/reflection test was added because a meaningful
  cross-service surviving-queue setup belongs with the CCD-focused branch
  campaign, not lifecycle padding.
- Focused lifecycle/partition/coroutine slice passed with 36 tests. Full
  `Release` passed with 1137 tests, `ReleaseLean` passed with 1116 tests, and
  coverage reported 93.2% line, 79.6% branch, and 93.4% method coverage.

### Workstream 6: Diagnostics And Low-Value Surface Audit

**Status:** Pending

**Purpose**

Make sure diagnostics branches are meaningful for host adapters without chasing
property getter noise.

**Likely Candidate Areas**

- `src/Gravitas/Diagnostics`
- `tests/Gravitas.Tests/Diagnostics`

**Tasks**

- [ ] Review remaining diagnostic coverage gaps and classify them as visitor
      behavior, construction behavior, disabled-path behavior, or getter noise.
- [ ] Add tests for host-adapter-visible visitor and disabled-path behavior.
- [ ] Leave mirrored immutable view getter noise alone unless it hides a real
      branch or construction invariant.
- [ ] Condense duplicate diagnostic tests that differ only by payload name.
- [ ] Run focused diagnostics tests, full `Release`, then coverage.

### Workstream 7: Branch 90 Gate

**Status:** Pending

**Purpose**

Close the release-hardening branch gate cleanly before expanding the plan toward
100% across the board.

**Tasks**

- [ ] Rerun coverage with `tests/Gravitas.Tests/coverlet.runsettings`.
- [ ] Confirm line coverage remains at least 90%.
- [ ] Confirm branch coverage is at least 90%.
- [ ] Confirm method coverage remains at least 90%.
- [ ] Run `dotnet test Gravitas.slnx --configuration Release`.
- [ ] Run `dotnet test Gravitas.slnx --configuration ReleaseLean`.
- [ ] Run `git diff --check`.
- [ ] Record final 90% gate evidence in this plan.
- [ ] Refresh the remaining 100% coverage roadmap based on measured gaps after
      the branch gate is met.

## Progress Log

| Date | Line | Branch | Method | Tests | Notes |
| --- | ---: | ---: | ---: | --- | --- |
| 2026-07-05 | 87.3% | 74.1% | 86.5% | 974 passed | Baseline captured with `tests/Gravitas.Tests/coverlet.runsettings` after trigger collider hardening. |
| 2026-07-05 | 90.0% | 76.0% | 91.0% | 1025 passed | First coverage campaign completed. Line and method gates met; branch gap remained active. Removed unused transient-state scaffolding. |
| 2026-07-05 | 92.0% | 77.9% | 92.2% | 1094 passed | Roadmaps A-E completed. Added focused branch coverage across mixed query support, replay hash, diagnostics, collision dispatch, geometry, service lifecycle, and collider authoring. Removed stale cuboid and joint helper code. Fixed 3D inactive direct-collider load partition cleanup parity. |
| 2026-07-06 | 92.1% | 77.9% | 92.4% | 1095 passed | Workstream 1 branch inventory completed. Removed stale immovable-contact direction, collider height shortcut, dead acceleration flags, mesh edge-normal storage, and an allocating triangle wrapper. Fixed rotated cuboid raycasts to clip local slabs. Recorded mesh-cuboid fallback SAT completeness for RCA. |
| 2026-07-06 | 92.5% | 78.4% | 92.5% | 1105 passed | Workstream 2 collision geometry hardening completed. Removed unused SAT projection overload and duplicate mesh-cylinder fallback logic. Added 2D compound geometry/manifold, mixed compound selection, and cone-convex reversed/negative dispatch coverage. |
| 2026-07-06 | 92.8% | 78.7% | 92.7% | 1118 passed | Workstream 3 query reducer and shape-cast hardening completed. Added focused coverage for convex-vs-compound sweeps, cone-axis concave triangle hits, mixed conservative fallback reporting, 2D convex mover/capsule sweeps, and raycast worker point/rotated-box geometry. |
| 2026-07-06 | 93.0% | 79.2% | 93.0% | 1125 passed | Workstream 4 serialization, replay, and authoring hardening completed. Added shape-definition value semantics, compound authored replay payload, polygon serialization, and mixed inactive-load coverage. Tightened final review assertions for mixed partition membership and compound replay hash stability. Fixed 2D active-state mixed partition cleanup parity. |
| 2026-07-06 | 93.2% | 79.6% | 93.4% | 1137 passed | Workstream 5 service lifecycle, partition, and runtime hardening completed. Added retained partition idempotency, context reset parity, coroutine lifecycle, clock hook, and 2D deferred refresh coverage. Centralized partition voxel ordering and removed duplicated null-comparer branch debt. |
