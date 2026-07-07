# Coverage Hardening Plan

**Date:** 2026-07-06  
**Status:** Active - branch coverage to 90%  
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

Fresh checkpoint after Workstream 12 CCD, query, and mixed reducer residue
pass:

| Metric | Baseline | Current | Short-Term Gate | Long-Term Target |
| --- | ---: | ---: | ---: | ---: |
| Line coverage | 87.3% | 94.1% | 90% | 100% |
| Branch coverage | 74.1% | 81.3% | 90% | 100% |
| Method coverage | 86.5% | 94.4% | 90% | 100% |
| Tests | 974 passed | 1214 passed | green | green |

At the current denominator, the 90% branch gate requires at least 10,234 covered
branches. The latest run covered 9,251 of 11,371 branches, leaving roughly
983 net branch outcomes to cover or delete. Treat this as a focused branch
campaign, not a single gate-check workstream.

Current evidence:

- Coverage report:
  `TestResults/coverage-branch-hardening-ws12-final2/reports/Summary.txt`
- Coverage collection:
  `dotnet test tests\Gravitas.Tests\Gravitas.Tests.csproj --configuration Release --collect:"XPlat Code Coverage" --settings tests\Gravitas.Tests\coverlet.runsettings`
  passed with 1214 tests.
- Branch shortlist:
  `TestResults/coverage-branch-hardening-ws12-final2/branch-gap-shortlist.csv`
- Latest method-gap extraction reported 210 uncovered methods.

## Completed Coverage Summary

The completed coverage campaign moved Gravitas from 87.3% line, 74.1% branch,
and 86.5% method coverage to 94.1% line, 81.3% branch, and 94.4% method
coverage.

High-value work completed:

- Query and collision coverage across 2D, 3D, and mixed dimensions, including
  capsule sweeps, compound shapes, mixed compound boundaries, cone queries,
  convex-vs-compound sweeps, public batch-query contracts, mixed sphere-slab
  fallback normals, shape-cast reducer reporting, 2D reversed capsule manifold
  paths, mesh-cone support/fallback contacts, cuboid-capsule SAT misses, and
  curved/mesh frontal-area geometry used by drag.
- Runtime lifecycle coverage for partition reset, retained membership cleanup,
  context reset, coroutine lifecycle, clock hooks, trigger-driven deferred
  refresh, inactive collider loading, mixed partition state cleanup, static and
  kinematic partition removals, stale collider-ID reuse safety, and hierarchy
  reparent/rejection behavior.
- Serialization and replay coverage for shape-definition equality/hash
  semantics, shape-definition guardrails, compound authored replay payloads,
  polygon serialization, invalid trigger load rejection, pending CCD handoff
  hash state, and authoritative-vs-cache hash distinctions.
- Constraint, ragdoll, grounding, diagnostics, trigger, material, and logger
  coverage where behavior is host-visible or deterministic-state-relevant.
- Real bug fixes found by coverage review: rotated cuboid raycasts now clip
  local slabs, 2D active-state changes clear mixed partitions, inactive 3D
  direct-collider loads clear stale partition state, and mixed CCD handoff
  queues now drain through one context-owned budget before partition/discrete
  completion.

Cleanup completed:

- Removed unused transient-state scaffolding, stale cuboid face-projection
  helpers, unused joint awake helpers, dead acceleration flags, stale
  immovable-contact state, `LSCollider.HeightPos`, unused mesh edge-normal
  storage, and an allocating mesh triangle wrapper.
- Removed duplicate mesh-cylinder fallback candidate logic and centralized
  2D/3D/mixed `WorldVoxelIndex` ordering.
- Tightened cuboid SAT helper contracts so successful geometry helpers return
  concrete contact state instead of nullable success payloads.
- Removed unreachable kinematic push-axis helpers, removed stale static CCD
  exact-source refinement now covered by the exact source sweep path, and
  dropped an unreachable cone swept-sphere fallback normal helper.
- Classified remaining low-value diagnostic view getter and simple constructor
  noise as intentionally out of scope unless a future adapter invariant makes
  it meaningful.

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

## Active Branch-90 Campaign

Workstreams 1-12 are now historical context. The next phase starts at
Workstream 13 and should be reranked after each coverage run. The ordering below
comes from
`TestResults/coverage-branch-hardening-ws12-final2/branch-gap-shortlist.csv`
plus
runtime risk, hot-path relevance, and duplicate/zombie-code likelihood.

| Order | Branch Family | Why It Comes Next |
| ---: | --- | --- |
| 1 | Collision geometry fallback, response-contact, and transform residue | The top zero-branch rows after Workstream 12 are now `ResolveDynamicContactPoint`, mixed/mesh fallback-normal helpers, 2D solver contact velocity, and simple body transform accessors. Classify these before adding tests; delete only proven zombie helpers. |
| 2 | Collider hierarchy, pair-state, and constraint capacity residue | Several one-covered-branch rows sit in hierarchy mutation, pair registration, and joint capacity helpers. These are deterministic lifecycle invariants and good candidates for focused tests if the public setup stays readable. |
| 3 | Query reducer and convex-support CRAP hotspots | `TryBuildConeHitForCollider`, `TrySweepPointInSpace`, convex sweep hit-point/normal resolution, and mixed prism reducers remain high-CRAP but behavior-bearing. Tackle them with scenario tests or small helper cleanup, not private steering. |
| 4 | Diagnostic/debug-draw DTO surface audit | Several diagnostic view and draw-command constructors remain low coverage. Cover only visitor/adapter dispatch invariants; leave simple immutable getter noise alone. |
| 5 | Branch 90 gate and 100% roadmap refresh | Only run as a true gate after focused campaigns make 90% realistic. |

### Workstream 7: CCD Handoff And Shape-Exact Branch Families

**Status:** Complete - 2026-07-06

**Purpose**

Close the largest and riskiest branch gaps around continuous collision handoff,
hit ordering, rotational CCD, kinematic-dynamic pushes, and shape-exact
refinement. These are release-critical because they decide deterministic
fast-mover behavior, not just coverage percentages.

**Likely Candidate Areas**

- `src/Gravitas/Core/3D/SolidBody.ContinuousCollision*.cs`
- `src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision*.cs`
- `src/Gravitas/Runtime/GravitasWorldContext.cs`
- `src/Gravitas/CollisionHandling/Continuous/ContinuousCollisionCandidateOrdering.cs`
- `tests/Gravitas.Tests/CollisionHandling`
- `tests/Gravitas.Tests/Physics2D`
- `tests/Gravitas.Tests/MixedDimensions`

**Tasks**

- [x] Re-open the latest branch shortlist and classify each CCD entry as real
      behavior, duplicate code, defensive impossible branch, or bug/parity
      smell before adding tests.
- [x] Cover 3D and 2D kinematic-dynamic push eligibility, ignored-target, and
      no-target branches through public simulation scenarios.
- [x] Cover mixed 2D/3D CCD push/handoff branches where they affect observable
      movement, contact, trigger, or replay state.
- [x] Review rotational CCD branches in 3D and 2D with deterministic angular
      sweeps, including no-hit, replacement-hit, and exact-hit paths.
- [x] Review shape-exact refinement and swept-normal fallback branches through
      valid collider configurations rather than private-helper steering.
- [x] Review candidate-ordering duplicate entries and centralize/delete any
      redundant branch logic before writing more tests around it.
- [x] Record any CCD semantic bug or parity gap in `issue-tracker.md`; if fixed
      immediately, move it to resolved with RCA and verification evidence.
- [x] Run focused CCD tests, full `Release`, coverage collection, and
      `ReleaseLean` if serialization or conditional compilation changes.

**Result**

- Added deterministic candidate-ordering coverage for 3D and 2D hit ordering,
  plus ignored-target self/body/hierarchy cases. The
  `ContinuousCollisionCandidateOrdering` class now reports 100% line coverage
  in the WS7 report.
- Removed a mixed CCD test-only service helper and routed the 2D-to-3D
  kinematic handoff test through `GravitasWorldContext.LateSimulate`.
- Fixed a real lifecycle ownership issue: context-driven late simulation now
  integrates 3D and 2D bodies first, drains the shared CCD handoff queue once at
  the context level, then completes partition/discrete response phases. Direct
  service `LateSimulate()` calls remain self-contained.
- Added mixed 3D-to-2D and 2D-to-3D handoff-chain tests plus an independent
  same-frame 3D/2D queued-handoff regression that proves
  `ContinuousCollisionMaxToiIterations` is honored as a single context-owned
  queue budget in mixed frames.
- Existing rotational and shape-exact public tests already cover the meaningful
  no-hit/hit/allocation scenarios. The remaining high-complexity CCD rows stay
  visible in the WS7 shortlist for later branch campaigns rather than being
  padded with private-helper tests.

**Exit Criteria**

- CCD-related branch gaps are measurably reduced or explicitly reclassified.
- No new private-only tests assert implementation detail without behavior.
- Any discovered CCD bug is fixed or tracked in `issue-tracker.md`.
- The progress log records the new coverage numbers and test count.

### Workstream 8: Mixed Prism, Response, Trigger, And Contact Branches

**Status:** Complete - 2026-07-06

**Purpose**

Close mixed collision and response branch gaps that affect first-class mixed
simulation behavior, especially prism reducers, impulse application, trigger
notifications, pair retention, and candidate processing.

**Likely Candidate Areas**

- `src/Gravitas/CollisionHandling/Detection/Mixed`
- `src/Gravitas/CollisionHandling/Response/Mixed/CollisionResponseMixed.cs`
- `src/Gravitas/Core/Mixed`
- `src/Gravitas/Colliders/3D/LSCollider.cs`
- `src/Gravitas/Colliders/2D/LSCollider2D.cs`
- `tests/Gravitas.Tests/MixedDimensions`
- `tests/Gravitas.Tests/CollisionHandling`

**Tasks**

- [x] Cover meaningful cuboid, capsule, cylinder, cone, and triangle prism
      branches with physical mixed pair scenarios.
- [x] Cover mixed response branches for dynamic/static, dynamic/kinematic,
      trigger, sensor-like bodyless, frozen-axis, friction, and restitution
      behavior where those branches are valid.
- [x] Review mixed trigger/contact notification branches for duplicate 2D/3D
      behavior and centralize shared assertions where useful.
- [x] Review mixed candidate processing and pair-retention branches for stale
      lifecycle behavior before adding tests.
- [x] Avoid one-test-per-shape padding when one shared test can prove the branch
      family.
- [x] Run focused mixed tests, full `Release`, and coverage collection.

**Result**

- Added focused mixed response coverage for bodyless 3D and bodyless 2D
  physical participants, separating velocities with correction disabled,
  frictionless contacts, high-static-friction contacts, and fallback/contrary
  normal resolution. This raised `CollisionResponseMixed` from 94.2% to 94.7%
  line coverage in the WS8 report and improved branch coverage in `Resolve`,
  `ApplyFrictionImpulse`, and normal resolution without changing runtime logic.
- Tightened mixed contact event parity by asserting 2D stay and exit callbacks
  alongside the existing 3D contact notifications.
- Added an oblique cuboid-vs-convex-polygon slab regression where broad bounds
  overlap but the exact prism SAT path rejects the pair. This covers a useful
  mixed prism false-positive guard instead of a far-apart bounds-only case.
- Reviewed the remaining mixed candidate and pair-retention gaps. Existing
  sleep, filter, trigger, stale-pair, and retained-partition tests already
  cover the meaningful lifecycle outcomes; the remaining branch residue is
  best handled by later reducer/lifecycle passes if it still ranks highly.
- `CollisionDetectionMixed.ResolveFallbackNormal(...)` remains fully uncovered.
  Current 2D shape semantics include boundaries in `ContainsPoint`, so valid
  zero-distance sphere/slab cases route through the inside-slab path before that
  fallback. Treat it as a defensive fallback to revisit during a later
  collision-geometry zombie-code sweep rather than adding a synthetic test.

**Exit Criteria**

- Mixed prism and response gaps are reduced through behavior-driven tests or
  cleanup.
- Trigger/contact behavior stays symmetric where intended and explicit where it
  is dimension-specific.
- Any mixed parity issue is fixed or visible in `issue-tracker.md`.

### Workstream 9: Query Reducer And Shape-Cast Residue

**Status:** Complete - 2026-07-06

**Purpose**

Cover remaining public query and shape-cast branch families without duplicating
scalar and batch tests. The target is correctness around ordering, exactness,
fallback classification, and rotated/curved shape geometry.

**Likely Candidate Areas**

- `src/Gravitas/Queries/3D`
- `src/Gravitas/Queries/2D/QueryDetection2D.cs`
- `src/Gravitas/Queries/Mixed`
- `tests/Gravitas.Tests/Queries`
- `tests/Gravitas.Tests/MixedDimensions`

**Tasks**

- [x] Cover public overlap hit-building, directional query residue,
      cone sweep/contact construction, and concave-mesh cone query branches
      that are public-query-visible.
- [x] Cover mixed sphere-against-2D reducer branches that still decide exact
      vs conservative reporting.
- [x] Cover 2D query clipping/sweep branches through public 2D query or CCD
      flows when possible.
- [x] Review zero-length or invalid request branches and prefer public
      validation tests over private worker calls unless the worker is itself a
      reusable geometry policy.
- [x] Keep batch-query coverage focused on range/output contracts, not repeated
      scalar geometry.
- [x] Run focused query tests, allocation-sensitive query checks where
      relevant, full `Release`, and coverage collection.

**Result**

- Added public 2D AABB batch coverage for closest-hit order, all-hit ranges,
  and no-allocation batch reuse without duplicating scalar AABB geometry tests.
- Added 3D circle query closest-hit and zero-direction directional-query
  coverage, proving stable closest selection and no false hit when direction is
  intentionally degenerate.
- Added mixed `SweepSphereAgainst2D` coverage for circle-slab side, cap, and
  boundary-overlap normals, plus unsupported-shape conservative fallback
  normal selection through the public mixed query API.
- Added deterministic reducer ordering coverage for equal-distance 3D compound
  target parts and equal-distance concave mesh triangles in convex mesh sweeps.
- Added concave mesh cone-query tie coverage and direct cone ray worker side,
  base, and point-inside geometry tests. The pass intentionally did not force
  invalid degenerate mesh triangles because `PhysicsMesh` rejects them during
  construction.
- Remaining query residue is now mostly lower-value fallback geometry such as
  `ConvexSweepQueryWorker.ResolveHitPoint/ResolveHitNormal`,
  `RaycastSegmentWorker.CheckConeSide`, and `SweptSphereQueryWorker` cone
  fallback normals. Keep these visible for later geometry/zombie-code sweeps
  rather than padding WS9 with invalid setups.

**Exit Criteria**

- Remaining query branch tests prove user-visible query behavior.
- Exact, conservative, miss, and ordering branches are covered where meaningful.
- No duplicate scalar/batch coverage padding is added.

### Workstream 10: Collision Geometry And Convex Support Residue

**Status:** Complete - 2026-07-07

**Purpose**

Close geometry branches that can hide real collision-quality defects, including
GJK/simplex transitions, mesh/cone/cuboid fallbacks, 2D convex clipping, and
manifold replacement behavior.

**Likely Candidate Areas**

- `src/Gravitas/CollisionHandling/Detection/3D`
- `src/Gravitas/CollisionHandling/Detection/2D/CollisionDetection2D.cs`
- `src/Gravitas/CollisionHandling/Contacts`
- `tests/Gravitas.Tests/CollisionHandling`

**Tasks**

- [x] Review `ConvexColliderSupport` simplex branches for valid geometry cases
      versus unreachable defensive transitions.
- [x] Cover mesh-cone, mesh-cuboid, and axis-aligned cuboid manifold branches
      only through valid collider setups.
- [x] Cover 2D convex clipping, manifold replacement, and compound collision
      branches that still affect contact quality.
- [x] Delete fallback code if no valid collider setup can reach it after the
      stronger geometry paths added in prior hardening work.
- [x] Add or update regression tests for any false-positive or false-negative
      geometry issue discovered during the sweep.
- [x] Run focused collision tests, full `Release`, and coverage collection.

**Results**

- Added valid-geometry coverage for axis-aligned cuboid Z-manifold contacts,
  cuboid-capsule SAT misses, mesh-cone support contacts, mesh-cone convex
  fallback contacts, 2D reversed capsule side contacts, reversed capsule-convex
  fallback contacts, 2D rotated clipping, manifold replacement ordering, and
  material-preserving `SetContact`.
- Added internal support-policy coverage for supported/unsupported convex
  shapes, zero-axis fallback projection, same-center spheres, separated convex
  shapes, rotated overlaps, and cone-volume support hits/misses.
- Added drag-facing frontal-area coverage for cone, cylinder, and mesh
  geometry, including zero-direction, axial, radial, front-facing,
  back-facing, and transformed mesh normals.
- Removed the unused cuboid-capsule SAT counter and tightened cuboid SAT helper
  contracts so successful helpers return concrete `AxisPenetration` or
  `CollisionResult` values.
- Classified remaining `ConvexColliderSupport.UpdateTriangle` and
  `Perpendicular` residue as defensive simplex fallback logic that is not worth
  brittle private branch padding without a real geometry defect.
- Left mixed/mesh/swept fallback-normal residue visible for future sweeps:
  `CollisionDetectionMixed.ResolveFallbackNormal`,
  `MeshTriangleContactGenerator.ResolveNormal`, and similar geometry
  guardrails should be exercised only through user-visible collision/query cases
  if a measured gap or bug appears.

**Exit Criteria**

- Remaining geometry branch gaps are either covered, deleted, or explicitly
  classified as defensive guardrails.
- No geometry test relies on magic coordinates without documenting the contact
  or separation invariant it proves.

### Workstream 11: Lifecycle, Serialization, Replay, And Authoring Residue

**Status:** Complete - 2026-07-07

**Purpose**

Sweep lower-risk branch families after the physics-heavy passes: service
lifecycle residue, partition edge cases, replay hash distinctions, authored
shape state, and record/load behavior.

**Likely Candidate Areas**

- `src/Gravitas/Core`
- `src/Gravitas/Runtime`
- `src/Gravitas/Partitions`
- `src/Gravitas/Colliders`
- `src/Gravitas/Constraints`
- `tests/Gravitas.Tests/Core`
- `tests/Gravitas.Tests/Runtime`
- `tests/Gravitas.Tests/Serialization`
- `tests/Gravitas.Tests/Determinism`

**Tasks**

- [x] Re-check retained partition, direct load, active-state, and deferred
      refresh branches for 2D/3D/mixed parity.
- [x] Cover replay and serialization branches only through valid save/load,
      replay hash, or `IRecordable` flows.
- [x] Cover constraint record/load or solver branch residue only where it proves
      meaningful deterministic continuation or solver state.
- [x] Delete stale authoring or lifecycle compatibility branches when the new
      public API makes them impossible.
- [x] Run focused serialization/determinism/runtime tests, full `Release`,
      `ReleaseLean`, and coverage collection.

**Results**

- Deleted unused 3D `LateInitialize` lifecycle scaffolding from
  `GravitasPhysicsService` and `LSCollider`; no call sites or 2D counterpart
  existed.
- Removed explicit collider parameters from 2D/3D ragdoll link definitions so
  authored links derive their collider from the owning body. This prevents
  mismatched body/collider authoring and keeps benchmarks/tests on the cleaner
  API.
- Added serialization guard coverage proving loaded trigger state is rejected
  when applied to a body-owned collider in both 2D and 3D.
- Added replay hash coverage proving pending CCD handoff state contributes to
  both authoritative and solver-cache hash modes in 2D and 3D.
- Added authoring guard coverage for undefined/wrong-family shape definitions,
  invalid 2D AABB size, shape snapshot equality, capsule mutation/no-op
  branches, zero-mass capsule inertia, hierarchy reparent/rejection behavior,
  2D suppress-all ragdoll filtering parity, static/kinematic partition removal,
  stale collider-ID removal safety, and mixed thickness override idempotence.
- Coverage moved to 94.1% line, 81.3% branch, and 94.3% method with 1212
  Release tests passing. `ReleaseLean` passed with 1189 tests.

**Exit Criteria**

- Remaining lifecycle and serialization branch gaps are low-risk, tracked, or
  covered by meaningful behavior tests.
- `ReleaseLean` remains green when serialization or conditional compile paths
  are touched.

### Workstream 12: CCD, Query, And Mixed Reducer Residue

**Status:** Complete - 2026-07-07

**Purpose**

Continue the branch-90 campaign where the latest evidence points: CCD exact-hit
refinement, kinematic push-axis helpers, cone/query reducers, mixed prism
reducers, and fallback-normal guardrails. This should be a classification-first
pass: cover public behavior, delete duplication, and leave defensive guardrails
alone unless a real collision/query gap appears.

**Likely Candidate Areas**

- `src/Gravitas/Core/3D/SolidBody.ContinuousCollision*.cs`
- `src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision*.cs`
- `src/Gravitas/Queries/3D`
- `src/Gravitas/Queries/Mixed`
- `src/Gravitas/CollisionHandling/Detection`
- `tests/Gravitas.Tests/CollisionHandling`
- `tests/Gravitas.Tests/Physics2D`
- `tests/Gravitas.Tests/MixedDimensions`

**Tasks**

- [x] Classify `ResolveKinematicPushAxis`,
      `ResolveDynamicContactPoint`, CCD exact-hit refinement, and swept-normal
      residue as public behavior, defensive guardrail, duplicate helper, or bug.
- [x] Cover or delete only the branches that represent valid user-visible
      collision/query behavior.
- [x] Review cone overlap/query reducers and mixed prism reducers for shared
      helper opportunities before adding more tests.
- [x] Keep fallback-normal coverage tied to observable collision/query cases,
      not private-helper branch steering.
- [x] Record deeper CCD/query parity or performance issues in
      `issue-tracker.md` or `benchmark-signal-hardening-backlog.md`.
- [x] Run focused tests, full `Release`, and coverage collection.

**Results**

- Removed unreachable `ResolveKinematicPushAxis` helper overloads in 3D and 2D.
  Every call passed the same vector as candidate and fallback, so the helper's
  fallback branch space was unreachable by construction.
- Removed stale static CCD exact-source refinement from
  `TryRefineShapeExactContinuousCollisionHit`. Exact-capable non-sphere sources
  already enter `SweepExactSourceAgainstStaticAll`; the remaining refinement
  job is reverse-swept sphere rejection for unsupported non-sphere sources
  against sphere targets.
- Removed the unreachable `SweptSphereQueryWorker.ResolveFallbackConeNormal`
  helper. Positive cone separation implies a non-zero closest-surface delta for
  valid non-negative swept radii.
- Added focused query tests for cone broad-phase false positives rejected by the
  exact cone volume and mixed swept-circle sphere center-overlap surface normal
  selection.
- Strengthened the direct swept-sphere cone worker test so the cone marcher
  cleanup is anchored to stable impact-distance and impact-center assertions.
- Classified `CollisionDetectionMixed.ResolveFallbackNormal` as a defensive
  guardrail that is not cleanly reachable through valid built-in 2D colliders;
  do not steer it through an artificial inconsistent collider just for branch
  coverage.
- Coverage moved to 94.1% line, 81.3% branch, and 94.4% method with 1214
  Release coverage tests passing. Latest CRAP extraction reported 32 methods at
  or above threshold 25.

**Evidence**

- Focused tests:
  `dotnet test tests\Gravitas.Tests\Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~GravitasQuery3DServiceConeTests|FullyQualifiedName~GravitasQuery3DServiceSweepTests|FullyQualifiedName~MixedQueryCcdTests"`
  passed with 115 tests.
- CCD-focused tests:
  `dotnet test tests\Gravitas.Tests\Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~ContinuousCollisionDetectionTests|FullyQualifiedName~ColliderLocalCollisionFilteringTests|FullyQualifiedName~GravitasQuery3DServiceSweepTests"`
  passed with 100 tests.
- Coverage collection:
  `TestResults/coverage-branch-hardening-ws12-final2/reports/Summary.txt`
- Branch shortlist:
  `TestResults/coverage-branch-hardening-ws12-final2/branch-gap-shortlist.csv`
- CRAP shortlist:
  `TestResults/coverage-branch-hardening-ws12-final2/crap-scores-top100.txt`

### Workstream 13: Collision Geometry, Response, And Transform Residue

**Status:** Pending

**Purpose**

Continue the branch-90 campaign from the fresh Workstream 12 shortlist. This is
not the 90% gate yet: branch coverage remains at 81.3%, so the next useful pass
should classify and harden remaining geometry/contact/transform residue before
another gate attempt.

**Likely Candidate Areas**

- `src/Gravitas/Core/3D/SolidBody*.cs`
- `src/Gravitas/Core/2D/SolidBody2D*.cs`
- `src/Gravitas/CollisionHandling/Detection`
- `src/Gravitas/CollisionHandling/Response`
- `src/Gravitas/CollisionHandling/Contacts`
- `tests/Gravitas.Tests/CollisionHandling`
- `tests/Gravitas.Tests/Core`
- `tests/Gravitas.Tests/Physics2D`

**Tasks**

- [ ] Classify remaining zero-branch geometry/contact rows:
      `SolidBody2D.ResolveDynamicContactPoint`,
      `CollisionDetectionMixed.ResolveFallbackNormal`,
      `MeshTriangleContactGenerator.ResolveNormal`, and
      `SolverContactBuffer2D.GetNormalVelocity`.
- [ ] Cover public behavior for contact-point fallback, response velocity, and
      body transform helpers where host-visible behavior is meaningful.
- [ ] Delete or simplify fallback helpers only when valid collider/query flows
      prove the state is unreachable.
- [ ] Review one-covered-branch rows around `ContinuousCollisionHitComesBefore`,
      contact ordering, and collision normal fallback before adding tests.
- [ ] Run focused tests, full `Release`, and coverage collection.

### Workstream 14: Branch 90 Gate And 100% Roadmap Refresh

**Purpose**

Confirm the short-term 90% branch gate only after the preceding workstreams make
it realistically reachable, then pivot the living plan toward the long-term
100% target.

**Tasks**

- [ ] Rerun coverage with `tests/Gravitas.Tests/coverlet.runsettings`.
- [ ] Confirm line coverage remains at least 90%.
- [ ] Confirm branch coverage is at least 90%.
- [ ] Confirm method coverage remains at least 90%.
- [ ] Run `dotnet test Gravitas.slnx --configuration Release`.
- [ ] Run `dotnet test Gravitas.slnx --configuration ReleaseLean`.
- [ ] Run `git diff --check`.
- [ ] Record final 90% gate evidence in this plan.
- [ ] If branch coverage is still below 90%, rerank the active campaign instead
      of marking the gate complete.
- [ ] Once the branch gate passes, refresh this plan into a 100% roadmap using
      the newest coverage, CRAP, and method-gap artifacts.

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
| 2026-07-06 | 93.3% | 79.7% | 93.6% | 1146 passed | Workstream 6 diagnostics and low-value surface audit completed. Added logger facade, visitor null guard, disabled capture, zero-ray, and non-prismatic 2D joint draw coverage while leaving immutable view getter noise classified as low-value. |
| 2026-07-06 | 93.3% | 79.9% | 93.6% | 1153 passed | Workstream 7 CCD handoff and shape-exact branch pass completed. Added 3D/2D candidate-ordering and ignored-target coverage, removed a mixed CCD test-only handoff helper, and fixed context-driven mixed CCD handoff ownership so the shared TOI budget drains before partition/discrete completion. |
| 2026-07-06 | 93.4% | 80.1% | 93.6% | 1162 passed | Workstream 8 mixed prism, response, trigger, and contact branch pass completed. Added bodyless mixed response, friction, response normal-fallback, 2D contact parity, and oblique prism SAT rejection coverage. Reviewed mixed candidate/pair-retention residue and classified `ResolveFallbackNormal` as defensive collision-geometry zombie-code sweep material. |
| 2026-07-06 | 93.6% | 80.3% | 93.7% | 1174 passed | Workstream 9 query reducer and shape-cast residue completed. Added 2D AABB batch contracts, 3D circle closest/zero-direction coverage, mixed sphere-against-2D circle/fallback normals, convex sweep equal-distance ordering, concave mesh cone query tie coverage, and cone ray worker geometry tests. |
| 2026-07-07 | 93.9% | 80.7% | 93.9% | 1192 passed | Workstream 10 collision geometry and convex support residue completed. Added mesh-cone support/fallback, cuboid-capsule SAT miss, axis-aligned cuboid Z-contact, 2D reversed capsule/clipping, manifold replacement/material, convex-support policy, and cone/cylinder/mesh frontal-area coverage. Tightened cuboid SAT helper contracts and classified remaining simplex/fallback-normal residue as defensive guardrails unless a public bug appears. |
| 2026-07-07 | 94.1% | 81.3% | 94.3% | 1212 passed | Workstream 11 lifecycle, serialization, replay, and authoring residue completed. Deleted unused `LateInitialize` scaffolding, removed weak explicit ragdoll-link collider constructors, and added focused coverage for shape authoring guards, trigger load rejection, pending CCD handoff replay hash state, hierarchy rejection/reparenting, partition cleanup, stale collider-ID removal safety, and mixed thickness override idempotence. |
| 2026-07-07 | 94.1% | 81.3% | 94.4% | 1214 passed | Workstream 12 CCD, query, and mixed reducer residue completed. Removed unreachable kinematic push-axis helpers, stale static CCD exact-source refinement, and unreachable cone swept-sphere fallback-normal helper. Added cone exact-miss and mixed swept-circle center-overlap normal coverage, and strengthened direct swept-sphere cone worker assertions. Fresh branch shortlist reranks the next campaign around geometry/contact/transform residue before any 90% gate attempt. |
