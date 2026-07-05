# Coverage Hardening Plan

**Date:** 2026-07-05  
**Status:** Active  
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

**Status:** Pending

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

- [ ] Review existing 2D query and collision tests for duplicate scenario
      setup, stale assertions, and redundant cases.
- [ ] Cover `QueryDetection2D.TrySweepCapsuleSegmentAgainstConvexEdges(...)`
      with deterministic hit, miss, endpoint, tangent, and initial-overlap
      cases where the public API reaches this helper.
- [ ] Cover `QueryDetection2D.TryRaycastCompound(...)` through public 2D query
      APIs using compound colliders with at least two parts and stable hit
      ordering.
- [ ] Cover `CollisionDetection2D.TryCompoundOther(...)` with compound-vs-
      primitive pairs that exercise both normal orientation paths.
- [ ] Cover `CollisionDetection2D.TryCompoundCompound(...)` with compound-vs-
      compound contact selection and deterministic tie ordering.
- [ ] Cover `MixedEmbedded2DGeometry.TryGetCompoundBoundary(...)` through mixed
      collision or mixed query paths that use compound 2D slabs.
- [ ] Delete or simplify any private helper branch that proves unreachable
      through valid collider/query shapes.
- [ ] Run focused tests for `QueryDetection2D`, `CollisionDetection2D`, and
      mixed compound slab behavior.
- [ ] Run full `dotnet test Gravitas.slnx --configuration Release`.
- [ ] Rerun coverage with `coverlet.runsettings` and update this plan's
      progress log with line/branch/method deltas.

**Expected Evidence**

- Top query/collision CRAP scores should drop materially.
- No new allocation regressions on public query paths.
- Test count should not increase more than the number of genuinely distinct
  behaviors covered; duplicate tests should be removed or consolidated.

## Workstream 2: Partition Retained-Membership Cleanup

**Status:** Pending

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

- [ ] Review partition tests for overlap between 3D, 2D, and mixed retained
      membership scenarios.
- [ ] Add focused tests that build retained static, kinematic, dynamic, and
      sleeping memberships, then verify reset clears only the intended retained
      state without corrupting active membership.
- [ ] Check whether retained reset logic can share a small helper or clearer
      collection pattern without adding abstraction overhead.
- [ ] Verify no managed allocations are introduced in partition cleanup hot
      paths.
- [ ] Run focused partition tests.
- [ ] Run full `dotnet test Gravitas.slnx --configuration Release`.
- [ ] Rerun coverage and update this plan's progress log.

## Workstream 3: Grounding And Candidate Ordering Branches

**Status:** Pending

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

- [ ] Cover `CompareGroundContactCandidate(...)` with distance, normal, collider
      ID, and contact key tie-break cases through public grounding probes where
      practical.
- [ ] If public grounding setup becomes too indirect, extract a tiny internal
      comparison policy only if it improves clarity and testability without
      adding runtime overhead.
- [ ] Cover `ShouldReplaceMixedHit(...)` with earlier TOI, equal TOI,
      exact-vs-conservative reducer, trigger/filter, and collider ordering
      cases.
- [ ] Remove any duplicate CCD ordering tests that assert the same replacement
      branch through a slower integration scenario.
- [ ] Run focused grounding and CCD ordering tests.
- [ ] Run full `dotnet test Gravitas.slnx --configuration Release`.
- [ ] Rerun coverage and update this plan's progress log.

## Workstream 4: 3D Raycast Segment And Mesh Branches

**Status:** Pending

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

- [ ] Cover `RaycastSegmentWorker.CheckPointInsideBox(...)` through public
      segment/raycast query cases hitting inside, outside, face, edge, and
      corner conditions.
- [ ] Review mesh/capsule and mesh/cylinder uncovered branches for true
      behavior gaps versus unreachable defensive branches.
- [ ] Add focused mesh collision tests only for meaningful shape pairs and
      deterministic contact ordering.
- [ ] Remove unreachable defensive branches if valid mesh/collider invariants
      make them impossible.
- [ ] Run focused 3D query and mesh collision tests.
- [ ] Run full `dotnet test Gravitas.slnx --configuration Release`.
- [ ] Rerun coverage and update this plan's progress log.

## Workstream 5: Batch Query API Coverage And Test Condensing

**Status:** Pending

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

- [ ] Inventory existing scalar and batch query tests and remove redundant cases
      that only repeat scalar behavior.
- [ ] Add compact batch tests for closest-hit and all-hit range contracts.
- [ ] Cover empty request spans, undersized output spans, mixed hit ranges,
      stable per-request ordering, and no-allocation caller-owned buffers.
- [ ] Keep shape-specific correctness in scalar query tests; batch tests should
      verify batching mechanics and routing.
- [ ] Run focused query batch tests.
- [ ] Run full `dotnet test Gravitas.slnx --configuration Release`.
- [ ] Rerun coverage and update this plan's progress log.

## Workstream 6: Diagnostics, Debug Draw, And Low-Value Surface Review

**Status:** Pending

**Purpose**

Diagnostics and debug draw contain many view/visitor branches. Cover behavior
that matters to host adapters, and remove or exclude only surfaces that are
genuinely boilerplate or unreachable.

**Files To Inspect**

- `src/Gravitas/Diagnostics`
- `tests/Gravitas.Tests/Diagnostics`

**Tasks**

- [ ] Review diagnostic view and debug draw visitor tests for duplicate
      assertions.
- [ ] Cover disabled-sink allocation-free paths for diagnostic families not
      currently exercised.
- [ ] Cover visitor dispatch for every public debug draw and diagnostic event
      kind that host adapters are expected to consume.
- [ ] Identify trivial immutable view types where coverage is low only because
      property getters are not read; add tests only when construction or visitor
      behavior is meaningful.
- [ ] Remove duplicate diagnostic tests that assert the same sink enqueue path
      with different event names.
- [ ] Run focused diagnostics tests.
- [ ] Run full `dotnet test Gravitas.slnx --configuration Release`.
- [ ] Rerun coverage and update this plan's progress log.

## Workstream 7: Constraint And Replay Hash Coverage

**Status:** Pending

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

- [ ] Cover `JointSolver2D` angular limit, motor, and prismatic branch gaps
      with deterministic constraint tests.
- [ ] Cover `JointSolver3D.AddConeTwistRows(...)` with cone/twist limit cases
      that prove the hardened quaternion angular-error behavior.
- [ ] Cover replay hash contribution paths for 2D joints and mixed/contact
      pairs in both authoritative and solver-cache modes.
- [ ] Remove duplicate high-level ragdoll tests if lower-level joint tests make
      them redundant.
- [ ] Run focused constraint and replay hash tests.
- [ ] Run full `dotnet test Gravitas.slnx --configuration Release`.
- [ ] Rerun coverage and update this plan's progress log.

## Workstream 8: Final 90% Gate And 100% Roadmap

**Status:** Pending

**Purpose**

Close the first milestone cleanly, then keep this plan alive as the 100%
coverage tracker.

**Tasks**

- [ ] Rerun coverage with `tests/Gravitas.Tests/coverlet.runsettings`.
- [ ] Confirm line, branch, and method coverage are all at least 90%.
- [ ] Run `dotnet test Gravitas.slnx --configuration Release`.
- [ ] Run `dotnet test Gravitas.slnx --configuration ReleaseLean`.
- [ ] Record the 90% milestone evidence in this plan.
- [ ] Re-sort remaining coverage gaps by CRAP score and uncovered branch count.
- [ ] Add the next 100% milestone workstreams to this same document.
- [ ] Move any larger discovered correctness, performance, or API issues into
      `issue-tracker.md`, `benchmark-signal-hardening-backlog.md`, or a focused
      dated feature-work plan.

## Progress Log

| Date | Line | Branch | Method | Tests | Notes |
| --- | ---: | ---: | ---: | --- | --- |
| 2026-07-05 | 87.3% | 74.1% | 86.5% | 974 passed | Baseline captured with `tests/Gravitas.Tests/coverlet.runsettings` after trigger collider hardening. |

