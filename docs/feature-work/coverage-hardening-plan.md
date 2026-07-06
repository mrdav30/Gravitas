# Coverage Hardening Plan

**Date:** 2026-07-05  
**Status:** Active - Roadmap D complete; 100% campaign remains  
**Owner:** Gravitas coverage, test-quality, and dead-code hardening

---

> **For agentic workers:** Treat this as a living context guide until Gravitas
> reaches 100% line, branch, and method coverage. Update progress after each
> focused coverage pass. Do not inflate coverage with shallow tests, generated
> code chasing, or compatibility-preserving dead paths.

**Goal:** Raise Gravitas coverage to at least 90% line, branch, and method
coverage in the short term, then continue toward 100% with high-signal tests,
dead-code removal, duplicate-code cleanup, and no test-suite bloat.

**Architecture:** Coverage hardening should strengthen the runtime, not merely
touch lines. Each pass should inspect uncovered behavior, remove truly dead or
weak code when it no longer belongs, consolidate duplicate production/test
helpers where useful, and add focused tests only for meaningful public or
internal invariants.

**Tech Stack:** .NET 8, xUnit v3, FluentAssertions, Coverlet
`tests/Gravitas.Tests/coverlet.runsettings`, ReportGenerator, FixedMathSharp,
SwiftCollections, GridForge, Chronicler, and Gravitas deterministic replay,
query, collision, partition, diagnostics, and serialization systems.

## Measurement Source Of Truth

Use the repository coverage configuration, not ad hoc inline collector filters:

```powershell
dotnet test tests\Gravitas.Tests\Gravitas.Tests.csproj `
    --configuration Release `
    --collect:"XPlat Code Coverage" `
    --settings tests\Gravitas.Tests\coverlet.runsettings `
    --results-directory TestResults\coverage-hardening-baseline
```

Generate a local report from the resulting Cobertura file:

```powershell
reportgenerator `
    "-reports:TestResults\coverage-hardening-baseline\**\coverage.cobertura.xml" `
    "-targetdir:TestResults\coverage-hardening-baseline\reports" `
    "-reporttypes:Html;TextSummary;MarkdownSummaryGithub;JsonSummary" `
    "-title:Coverage Hardening Baseline"
```

The runsettings file excludes generated sources:

- `**/bin/**`
- `**/obj/**`
- `**/*.g.cs`
- `**/MemoryPack.Generator/**/*.cs`
- `GeneratedCodeAttribute`
- `CompilerGeneratedAttribute`
- `ExcludeFromCodeCoverageAttribute`

The CI workflow uses the same runsettings and ReportGenerator file filters.
If local and CI numbers disagree, fix the coverage command/configuration before
making source changes.

## Current Baseline

Fresh baseline captured on 2026-07-05 after trigger collider hardening:

| Metric | Current | Short-Term Target | Long-Term Target |
| --- | ---: | ---: | ---: |
| Line coverage | 87.3% | 90% | 100% |
| Branch coverage | 74.1% | 90% | 100% |
| Method coverage | 86.5% | 90% | 100% |
| Tests | 974 passed | green | green |

Top CRAP hotspots from the runsettings-based baseline:

| Rank | Method | File | Coverage | CRAP |
| ---: | --- | --- | ---: | ---: |
| 1 | `TrySweepCapsuleSegmentAgainstConvexEdges` | `Queries/2D/QueryDetection2D.cs` | 0.0% | 342.00 |
| 2 | `ResetRetainedMembership` | `Partitions/Mixed/PhysicsMixedPartition.cs` | 0.0% | 342.00 |
| 3 | `CompareGroundContactCandidate` | `Core/2D/SolidBody2D.Grounding.cs` | 0.0% | 272.00 |
| 4 | `CheckPointInsideBox` | `Queries/3D/RaycastSegmentWorker.cs` | 0.0% | 210.00 |
| 5 | `ShouldReplaceMixedHit` | `CollisionHandling/Continuous/ContinuousCollisionCandidateOrdering.cs` | 22.2% | 170.44 |
| 6 | `TryCompoundOther` | `CollisionHandling/Detection/2D/CollisionDetection2D.cs` | 0.0% | 156.00 |
| 7 | `TryCompoundCompound` | `CollisionHandling/Detection/2D/CollisionDetection2D.cs` | 0.0% | 156.00 |
| 8 | `ResetRetainedMembership` | `Partitions/2D/PhysicsPartition2D.cs` | 0.0% | 110.00 |
| 9 | `ResetRetainedMembership` | `Partitions/3D/PhysicsPartition.cs` | 0.0% | 110.00 |
| 10 | `TryGetCompoundBoundary` | `CollisionHandling/Detection/Mixed/MixedEmbedded2DGeometry.cs` | 0.0% | 110.00 |

Largest uncovered file areas from the same baseline:

| Area | Line Coverage | Branch Coverage | Uncovered Lines |
| --- | ---: | ---: | ---: |
| `Queries/3D/GravitasQuery3DService.Batch.cs` | 60.3% | 62.5% | 192 |
| `Queries/2D/QueryDetection2D.cs` | 79.8% | 69.7% | 154 |
| `CollisionHandling/Detection/2D/CollisionDetection2D.cs` | 77.0% | 58.4% | 135 |
| `Diagnostics/GravitasDiagnosticSink.Draw.cs` | 73.7% | 67.5% | 102 |
| `CollisionHandling/Detection/3D/CollisionDetection.Mesh.cs` | 59.3% | 52.2% | 79 |
| `Constraints/2D/JointSolver2D.cs` | 78.4% | 71.7% | 74 |
| `Queries/3D/RaycastSegmentWorker.cs` | 79.7% | 58.9% | 63 |

## Current Checkpoint

First hardening pass completed on 2026-07-05:

| Metric | Baseline | Current | Delta | Short-Term Gate |
| --- | ---: | ---: | ---: | --- |
| Line coverage | 87.3% | 90.0% | +2.7% | Met |
| Branch coverage | 74.1% | 76.0% | +1.9% | Not met |
| Method coverage | 86.5% | 91.0% | +4.5% | Met |
| Tests | 974 passed | 1025 passed | +51 | Green |

Evidence:

- Coverage report:
  `TestResults/coverage-hardening-final/reports/Summary.txt`
- ReportGenerator output:
  `TestResults/coverage-hardening-final/reports/index.html`
- Full validation:
  `dotnet test Gravitas.slnx --configuration Release` passed with 1025 tests.
- Lean validation:
  `dotnet test Gravitas.slnx --configuration ReleaseLean` passed with 1007
  tests.
- The branch gap is real runtime branch surface, not generated MemoryPack
  formatter noise. Reaching 90% branch coverage should be treated as a focused
  branch-quality campaign, not a DTO/property coverage sweep.

Cleanup completed:

- Removed unused `ITransient`, `TransientAttribute`, and
  `TransientStateUtility` scaffolding. It was a self-contained, unused
  reflection/expression-compiled transient-state path that no Gravitas runtime
  code referenced. Chronicler `IRecordable` remains the explicit state-transfer
  model.

Remaining top CRAP hotspots after the first pass:

| Rank | Method | File | Coverage | CRAP |
| ---: | --- | --- | ---: | ---: |
| 1 | `IsInsideBox` | `Queries/Mixed/GravitasQueryMixedService.Support.cs` | 0.0% | 110.00 |
| 2 | `ClipSegmentAxis` | `Queries/Mixed/GravitasQueryMixedService.Support.cs` | 0.0% | 110.00 |
| 3 | `ClassifySweepCircleAgainst3DReducer` | `Queries/Mixed/GravitasQueryMixedService.CircleAgainst3DReducers.cs` | 42.9% | 78.45 |
| 4 | `RemoveChild` | `Colliders/Hierarchy/ColliderHierarchyState.cs` | 0.0% | 72.00 |
| 5 | `SetImmovableDirection` | `CollisionHandling/Pairs/3D/CollisionPair.cs` | 0.0% | 72.00 |
| 6 | `TrySweepBox` | `Queries/Mixed/GravitasQueryMixedService.Support.cs` | 0.0% | 72.00 |
| 7 | `ContributeReplayHash` | `Constraints/2D/Joint2D.cs` | 0.0% | 72.00 |
| 8 | `TryCollide` | `CollisionHandling/Detection/2D/CollisionDetection2D.cs` | 50.0% | 64.12 |
| 9 | `TryIntersectSegments` | `CollisionHandling/Detection/2D/CollisionDetection2D.cs` | 33.3% | 54.67 |
| 10 | `TryRefineShapeExactContinuousCollisionHit` | `Core/3D/SolidBody.ContinuousCollision.Hits.cs` | 35.3% | 51.01 |

## Hardening Rules

- Do not chase generated MemoryPack formatter coverage. Keep generated-code
  exclusions aligned between `coverlet.runsettings`, local commands, and CI.
- Do not add tests just to touch lines. Tests must prove behavior, invariants,
  deterministic ordering, edge cases, replay safety, or regression risk.
- Prefer deleting truly dead code over testing it. If a path cannot be reached
  by a valid public/internal flow, remove it or record an issue explaining why
  it must stay.
- Prefer condensing duplicate tests over adding more cases. If multiple tests
  assert the same branch with different names, merge them into a clearer theory
  or delete the weaker duplicate.
- Prefer reusable focused test builders when setup noise hides the assertion,
  but do not create broad magic helpers that make physics behavior implicit.
- Do not preserve weak or legacy behavior only because a stale test asserts it.
  Update or delete the stale test when the stronger model is clear.
- Keep branch coverage honest. Avoid exclusions for hand-authored runtime code
  unless the code is platform-guarded, generated, or explicitly diagnostic-only
  boilerplate with no meaningful branch behavior.
- Keep performance-sensitive tests allocation-aware when they touch hot paths.
  Use existing allocation assertion patterns for query, partition, CCD,
  diagnostics, and response loops when practical.
- Run focused tests first, then full `Release`, and rerun coverage at the end of
  each workstream. Use `ReleaseLean` when serialization, MemoryPack shims,
  public packages, or conditional compilation are touched.

## Workstream 1: Query And 2D Collision Hotspots

**Status:** Completed

**Purpose**

Close the highest-value source hotspots before broadening into low-risk DTO or
diagnostic boilerplate. This pass should strengthen gameplay-facing query and
collision behavior while pruning any unreachable or duplicate test/code paths.

**Files To Inspect**

- `src/Gravitas/Queries/2D/QueryDetection2D.cs`
- `src/Gravitas/CollisionHandling/Detection/2D/CollisionDetection2D.cs`
- `src/Gravitas/CollisionHandling/Detection/Mixed/MixedEmbedded2DGeometry.cs`
- `tests/Gravitas.Tests/Queries`
- `tests/Gravitas.Tests/Physics2D`
- `tests/Gravitas.Tests/CollisionHandling`
- `tests/Gravitas.Tests/MixedDimensions`

**Tasks**

- [x] Review existing 2D query and collision tests for duplicate scenario
      setup, stale assertions, and redundant cases.
- [x] Cover `QueryDetection2D.TrySweepCapsuleSegmentAgainstConvexEdges(...)`
      with deterministic hit, miss, endpoint, tangent, and initial-overlap
      cases where the public API reaches this helper.
- [x] Cover `QueryDetection2D.TryRaycastCompound(...)` through public 2D query
      APIs using compound colliders with at least two parts and stable hit
      ordering.
- [x] Cover `CollisionDetection2D.TryCompoundOther(...)` with compound-vs-
      primitive pairs that exercise both normal orientation paths.
- [x] Cover `CollisionDetection2D.TryCompoundCompound(...)` with compound-vs-
      compound contact selection and deterministic tie ordering.
- [x] Cover `MixedEmbedded2DGeometry.TryGetCompoundBoundary(...)` through mixed
      collision or mixed query paths that use compound 2D slabs.
- [x] Delete or simplify any private helper branch that proves unreachable
      through valid collider/query shapes.
- [x] Run focused tests for `QueryDetection2D`, `CollisionDetection2D`, and
      mixed compound slab behavior.
- [x] Run full `dotnet test Gravitas.slnx --configuration Release`.
- [x] Rerun coverage with `coverlet.runsettings` and update this plan's
      progress log with line/branch/method deltas.

**Expected Evidence**

- Top query/collision CRAP scores should drop materially.
- No new allocation regressions on public query paths.
- Test count should not increase more than the number of genuinely distinct
  behaviors covered; duplicate tests should be removed or consolidated.

## Workstream 2: Partition Retained-Membership Cleanup

**Status:** Completed

**Purpose**

Cover and simplify retained partition membership cleanup across 3D, pure 2D,
and mixed. These methods are small but high-branch because they encode cleanup
order and duplicate-removal invariants.

**Files To Inspect**

- `src/Gravitas/Partitions/3D/PhysicsPartition.cs`
- `src/Gravitas/Partitions/2D/PhysicsPartition2D.cs`
- `src/Gravitas/Partitions/Mixed/PhysicsMixedPartition.cs`
- `tests/Gravitas.Tests/Partitions`
- `tests/Gravitas.Tests/MixedDimensions`

**Tasks**

- [x] Review partition tests for overlap between 3D, 2D, and mixed retained
      membership scenarios.
- [x] Add focused tests that build retained static, kinematic, dynamic, and
      sleeping memberships, then verify reset clears only the intended retained
      state without corrupting active membership.
- [x] Check whether retained reset logic can share a small helper or clearer
      collection pattern without adding abstraction overhead.
- [x] Verify no managed allocations are introduced in partition cleanup hot
      paths.
- [x] Run focused partition tests.
- [x] Run full `dotnet test Gravitas.slnx --configuration Release`.
- [x] Rerun coverage and update this plan's progress log.

## Workstream 3: Grounding And Candidate Ordering Branches

**Status:** Completed

**Purpose**

Close high-risk branch gaps in deterministic ordering helpers. These branches
are small, but they decide tie-breaking and deterministic contact/TOI
selection.

**Files To Inspect**

- `src/Gravitas/Core/2D/SolidBody2D.Grounding.cs`
- `src/Gravitas/CollisionHandling/Continuous/ContinuousCollisionCandidateOrdering.cs`
- `tests/Gravitas.Tests/Physics2D`
- `tests/Gravitas.Tests/CollisionHandling`

**Tasks**

- [x] Cover `CompareGroundContactCandidate(...)` with distance, normal, collider
      ID, and contact key tie-break cases through public grounding probes where
      practical.
- [x] If public grounding setup becomes too indirect, extract a tiny internal
      comparison policy only if it improves clarity and testability without
      adding runtime overhead.
- [x] Cover `ShouldReplaceMixedHit(...)` with earlier TOI, equal TOI,
      exact-vs-conservative reducer, trigger/filter, and collider ordering
      cases.
- [x] Remove any duplicate CCD ordering tests that assert the same replacement
      branch through a slower integration scenario.
- [x] Run focused grounding and CCD ordering tests.
- [x] Run full `dotnet test Gravitas.slnx --configuration Release`.
- [x] Rerun coverage and update this plan's progress log.

## Workstream 4: 3D Raycast Segment And Mesh Branches

**Status:** Completed

**Purpose**

Improve coverage around narrow 3D geometric edge paths that are easy to get
wrong and hard to debug in gameplay.

**Files To Inspect**

- `src/Gravitas/Queries/3D/RaycastSegmentWorker.cs`
- `src/Gravitas/CollisionHandling/Detection/3D/CollisionDetection.Mesh.cs`
- `src/Gravitas/CollisionHandling/Detection/3D/ConvexColliderSupport.cs`
- `tests/Gravitas.Tests/Queries`
- `tests/Gravitas.Tests/CollisionHandling`

**Tasks**

- [x] Cover `RaycastSegmentWorker.CheckPointInsideBox(...)` through public
      segment/raycast query cases hitting inside, outside, face, edge, and
      corner conditions.
- [x] Review mesh/capsule and mesh/cylinder uncovered branches for true
      behavior gaps versus unreachable defensive branches.
- [x] Add focused mesh collision tests only for meaningful shape pairs and
      deterministic contact ordering.
- [x] Remove unreachable defensive branches if valid mesh/collider invariants
      make them impossible.
- [x] Run focused 3D query and mesh collision tests.
- [x] Run full `dotnet test Gravitas.slnx --configuration Release`.
- [x] Rerun coverage and update this plan's progress log.

## Workstream 5: Batch Query API Coverage And Test Condensing

**Status:** Completed

**Purpose**

Batch query APIs have the largest uncovered-line block. The goal is to cover
the batch range/output contract without duplicating every single scalar query
test.

**Files To Inspect**

- `src/Gravitas/Queries/3D/GravitasQuery3DService.Batch.cs`
- `src/Gravitas/Queries/2D/GravitasQuery2DService.Batch.cs`
- `src/Gravitas/Queries/Mixed/GravitasQueryMixedService.Batch.cs`
- `tests/Gravitas.Tests/Queries`

**Tasks**

- [x] Inventory existing scalar and batch query tests and remove redundant cases
      that only repeat scalar behavior.
- [x] Add compact batch tests for closest-hit and all-hit range contracts.
- [x] Cover empty request spans, undersized output spans, mixed hit ranges,
      stable per-request ordering, and no-allocation caller-owned buffers.
- [x] Keep shape-specific correctness in scalar query tests; batch tests should
      verify batching mechanics and routing.
- [x] Run focused query batch tests.
- [x] Run full `dotnet test Gravitas.slnx --configuration Release`.
- [x] Rerun coverage and update this plan's progress log.

## Workstream 6: Diagnostics, Debug Draw, And Low-Value Surface Review

**Status:** Completed

**Purpose**

Diagnostics and debug draw contain many view/visitor branches. Cover behavior
that matters to host adapters, and remove or exclude only surfaces that are
genuinely boilerplate or unreachable.

**Files To Inspect**

- `src/Gravitas/Diagnostics`
- `tests/Gravitas.Tests/Diagnostics`

**Tasks**

- [x] Review diagnostic view and debug draw visitor tests for duplicate
      assertions.
- [x] Cover disabled-sink allocation-free paths for diagnostic families not
      currently exercised.
- [x] Cover visitor dispatch for every public debug draw and diagnostic event
      kind that host adapters are expected to consume.
- [x] Identify trivial immutable view types where coverage is low only because
      property getters are not read; add tests only when construction or visitor
      behavior is meaningful.
- [x] Remove duplicate diagnostic tests that assert the same sink enqueue path
      with different event names.
- [x] Run focused diagnostics tests.
- [x] Run full `dotnet test Gravitas.slnx --configuration Release`.
- [x] Rerun coverage and update this plan's progress log.

## Workstream 7: Constraint And Replay Hash Coverage

**Status:** Completed

**Purpose**

Constraint solver and replay hash branches are important deterministic
correctness areas. This pass should avoid broad solver churn and focus on
explicit untested branch families.

**Files To Inspect**

- `src/Gravitas/Constraints/2D`
- `src/Gravitas/Constraints/3D`
- `src/Gravitas/CollisionHandling/Pairs/*/*ReplayHash.cs`
- `tests/Gravitas.Tests/Constraints`
- `tests/Gravitas.Tests/Determinism`

**Tasks**

- [x] Cover `JointSolver2D` angular limit, motor, and prismatic branch gaps
      with deterministic constraint tests.
- [x] Cover `JointSolver3D.AddConeTwistRows(...)` with cone/twist limit cases
      that prove the hardened quaternion angular-error behavior.
- [x] Cover replay hash contribution paths for 2D joints and mixed/contact
      pairs in both authoritative and solver-cache modes.
- [x] Remove duplicate high-level ragdoll tests if lower-level joint tests make
      them redundant.
- [x] Run focused constraint and replay hash tests.
- [x] Run full `dotnet test Gravitas.slnx --configuration Release`.
- [x] Rerun coverage and update this plan's progress log.

## Workstream 8: Final 90% Gate And 100% Roadmap

**Status:** Active

**Purpose**

Close the first milestone cleanly, then keep this plan alive as the 100%
coverage tracker.

**Tasks**

- [x] Rerun coverage with `tests/Gravitas.Tests/coverlet.runsettings`.
- [ ] Confirm line, branch, and method coverage are all at least 90%.
- [x] Run `dotnet test Gravitas.slnx --configuration Release`.
- [x] Run `dotnet test Gravitas.slnx --configuration ReleaseLean`.
- [x] Record the 90% milestone evidence in this plan.
- [x] Re-sort remaining coverage gaps by CRAP score and uncovered branch count.
- [x] Add the next 100% milestone workstreams to this same document.
- [x] Move any larger discovered correctness, performance, or API issues into
      `issue-tracker.md`, `benchmark-signal-hardening-backlog.md`, or a focused
      dated feature-work plan.

**Current Result**

Line and method coverage reached the short-term gate. Branch coverage improved
to 76.0%, but the remaining gap is too large to close honestly in one pass
without targeted branch campaigns across mixed query support, replay hash,
diagnostics, collision dispatch, and service lifecycle paths.

No new correctness or performance issue was split into `issue-tracker.md` or
`benchmark-signal-hardening-backlog.md` during this pass; the remaining work is
coverage quality and is tracked below.

## 100% Roadmap Workstreams

### Roadmap A: Mixed Query Support Branches

**Status:** Completed

Target `GravitasQueryMixedService.Support.cs` and mixed finite-slab reducer
classification branches. Add focused tests for inside-box clipping, segment
axis clipping, swept-box hit/miss/parallel cases, and reducer classification
for all 2D/3D supported shape families. Prefer public mixed query APIs when
setup is readable; extract tiny internal policies only if it reduces brittle
integration setup without changing runtime cost.

**Result**

- Extracted mixed swept-box clipping into `MixedSweepBoxUtility` and covered
  start-inside, parallel-inside, parallel-outside, interval-reject, and
  negative-direction clipping cases.
- Extracted mixed reducer classification into `MixedQueryReducerClassifier`,
  covered exact 2D and 3D shape families, and kept unknown collider subclasses
  classified as conservative fallback.
- Removed unreachable compound-part fallback classification branches. Current
  2D and 3D compound colliders materialize only supported exact parts, so the
  classifier now reflects the actual authored compound model instead of
  carrying future-shape defensive logic.
- Coverage report:
  `TestResults/coverage-roadmap-a/reports/Summary.txt`

### Roadmap B: Replay Hash Branch Families

**Status:** Completed

Target 2D joint replay hashing, mixed collision pair replay hashing, and
manifold/contact contribution branches. Cover authoritative and solver-cache
modes, inactive/empty state, hierarchy ordinal resolution, and stable ordering.
Do not add tests that merely hash default objects; each case should prove a
deterministic state distinction.

**Result**

- Added a focused replay hash branch suite for 2D joint authoritative state,
  solver-cache-only mutation, removed 2D constraint slots, 2D ragdoll activation
  metadata, 2D/3D pair manifold material payloads, warm-start cache payloads,
  mixed trigger/contact material payloads, and mixed 2D/3D hierarchy ordinal
  resolution.
- Kept this pass test-only. The existing replay hash writers already separated
  authoritative continuation state from solver-cache RCA state correctly.
- Verified hierarchy replay keys are normalized through replay ordinals rather
  than raw collider IDs by comparing equivalent mixed hierarchies with deleted
  collider ID churn.
- Coverage report:
  `TestResults/coverage-roadmap-b/reports/Summary.txt`

### Roadmap C: Diagnostics Draw And Event Branches

**Status:** Completed

Target diagnostic sink emit/draw helpers with disabled-sink, enabled-sink,
capacity, mixed polygon, compound-part, joint, ragdoll, and query event
branches. Keep disabled paths allocation-free and do not chase view property
getters unless the view construction or visitor dispatch has host-adapter value.

**Result**

- Added focused diagnostic sink coverage for 3D compound capsule/cylinder/mesh
  part draw expansion, mixed 2D polygon/capsule/compound slab draw expansion,
  3D circle-query event payloads, 2D/3D joint-limit payloads, ragdoll
  activation payloads, and dimensional joint debug-draw commands.
- Added a compact typed diagnostic-event rejection test so wrong-kind `TryAs*`
  calls fail cleanly across the event visitor surface.
- Left low-value typed debug-draw view getter noise alone. Existing dispatch
  tests already assert every host-adapter draw payload; the remaining uncovered
  lines are mirrored metadata/property accessors rather than branchy behavior.
- Coverage report:
  `TestResults/coverage-roadmap-c/reports/Summary.txt`

### Roadmap D: Collision Dispatch And Geometry Branches

**Status:** Completed

Target 2D dispatch branches, segment intersection branches, 3D mesh/cuboid/
cylinder/capsule contact branches, convex support simplex updates, and compound
compound collision branches. Prefer edge-case tests that prove physical
behavior: parallel/separated, tangent, contained, degenerate-but-valid, and
deterministic tie-ordering cases.

**Result**

- Added focused 2D single-contact dispatch coverage for required circle,
  AABox, convex polygon, and capsule pair permutations.
- Added crossed 2D capsule coverage for the segment-intersection contact path.
- Added 3D compound-vs-compound owner-order coverage using the narrow-phase
  detection path directly so response mutation does not hide symmetry defects.
- Added convex mesh-vs-capsule fallback and reversed-dispatch coverage.
- Added mixed prism rejection coverage for cuboid, capsule, cylinder, and cone
  families against AABox, capsule, and convex polygon slabs.
- Removed the unused `LSCuboidCollider` face-projection helper cluster,
  including the allocating `GetFace(...)` public helper. The active oriented
  cuboid closest-point path no longer uses those methods, so deleting them was
  stronger than preserving stale geometry code for coverage.
- Left `ConvexColliderSupport` simplex internals as future branch work. They
  are reachable only through cone collision/query and convex sweep paths, and
  steering private simplex topology with brittle geometry cases would not be a
  good coverage trade. A later pass should refactor GJK simplex policy into a
  directly testable helper if those branches remain high-risk.
- Coverage report:
  `TestResults/coverage-roadmap-d/reports/Summary.txt`

### Roadmap E: Service Lifecycle And Collider Authoring Branches

**Status:** Pending

Target collider hierarchy removal, compound collider mutation/lifecycle edges,
constraint service ragdoll pose-target helpers, physics settings saver branches,
and service lifecycle branch families that remain below 90%. Delete unreachable
or stale support paths rather than preserving them for coverage.

## Progress Log

| Date | Line | Branch | Method | Tests | Notes |
| --- | ---: | ---: | ---: | --- | --- |
| 2026-07-05 | 87.3% | 74.1% | 86.5% | 974 passed | Baseline captured with `tests/Gravitas.Tests/coverlet.runsettings` after trigger collider hardening. |
| 2026-07-05 | 87.8% | 74.7% | 86.7% | 983 passed | Workstream 1 covered 2D capsule sweep, compound 2D collision, and mixed compound boundary behavior. |
| 2026-07-05 | 87.9% | 75.0% | 86.8% | 986 passed | Workstream 2 covered retained partition cleanup across 3D, 2D, and mixed partitions. |
| 2026-07-05 | 88.0% | 75.2% | 86.8% | 988 passed | Workstream 3 covered 2D support candidate tie-ordering and mixed CCD reducer-kind ordering. |
| 2026-07-05 | 88.0% | 75.3% | 86.9% | 1000 passed | Workstream 4 covered segment raycast point/mesh edge cases. |
| 2026-07-05 | 88.9% | 75.6% | 87.3% | 1002 passed | Workstream 5 covered 3D batch routing for registered source sweeps and directional overlap order. |
| 2026-07-05 | 89.1% | 75.6% | 89.4% | 1006 passed | Workstream 6 covered diagnostic debug-draw and event visitor families. |
| 2026-07-05 | 89.4% | 75.8% | 89.5% | 1012 passed | Workstream 7 covered additional 2D/3D constraint limit and motor branches. |
| 2026-07-05 | 90.0% | 76.0% | 91.0% | 1025 passed | Final first-pass checkpoint. Removed unused transient-state scaffolding and covered query request/range, coroutine wait, layer, and compound-part contracts. Branch 90% remains active roadmap work. |
| 2026-07-05 | 90.1% | 76.5% | 91.1% | 1034 passed | Roadmap A completed. Mixed swept-box clipping and reducer classification are split into focused internal helpers with direct branch coverage. |
| 2026-07-05 | 90.7% | 76.8% | 91.5% | 1042 passed | Roadmap B completed. Replay hash branch coverage now covers 2D joint cache/authoritative distinctions, pair manifold/warm-start payloads, mixed trigger/contact materials, and hierarchy replay ordinal normalization. |
| 2026-07-05 | 91.4% | 77.1% | 91.7% | 1048 passed | Roadmap C completed. Diagnostic sink coverage now exercises compound/mixed slab draw expansion, joint draw branches, circle-query events, 2D/3D joint-limit events, ragdoll activation events, and typed event rejection branches. |
| 2026-07-05 | 91.7% | 77.4% | 91.8% | 1076 passed | Roadmap D completed. Collision dispatch coverage now includes 2D single-contact pair routing, crossed capsule segment contacts, 3D compound-compound owner-order symmetry, convex mesh/capsule fallback dispatch, mixed prism rejection families, and stale cuboid face-projection helper removal. |
