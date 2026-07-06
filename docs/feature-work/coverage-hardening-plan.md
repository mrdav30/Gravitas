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

Fresh checkpoint after Roadmap E and the 3D inactive-collider load parity fix:

| Metric | Baseline | Current | Short-Term Gate | Long-Term Target |
| --- | ---: | ---: | ---: | ---: |
| Line coverage | 87.3% | 92.0% | 90% | 100% |
| Branch coverage | 74.1% | 77.9% | 90% | 100% |
| Method coverage | 86.5% | 92.2% | 90% | 100% |
| Tests | 974 passed | 1094 passed | green | green |

Current evidence:

- Coverage report:
  `TestResults/coverage-roadmap-e/reports/Summary.txt`
- Coverage collection:
  `dotnet test tests\Gravitas.Tests\Gravitas.Tests.csproj --configuration Release --collect:"XPlat Code Coverage" --settings tests\Gravitas.Tests\coverlet.runsettings`
  passed with 1094 tests.
- Full validation:
  `dotnet test Gravitas.slnx --configuration Release` passed with 1094 tests.
- Lean validation:
  `dotnet test Gravitas.slnx --configuration ReleaseLean` passed with 1074
  tests.
- Latest CRAP extraction reported 42 flagged methods and 292 uncovered
  methods.

## Completed Coverage Summary

The first coverage campaign moved the project from 87.3% line, 74.1% branch,
and 86.5% method coverage to 92.0% line, 77.9% branch, and 92.2% method
coverage.

High-value work completed:

- Query and 2D collision edge coverage for capsule sweeps, compound 2D
  collision, mixed compound boundaries, and query request/range contracts.
- Partition retained-membership cleanup coverage across 3D, pure 2D, and mixed
  partitions.
- Deterministic ordering coverage for 2D grounding candidates and continuous
  collision hit replacement.
- 3D raycast, mesh, convex, and collision dispatch branch coverage.
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

Cleanup completed:

- Removed unused `ITransient`, `TransientAttribute`, and
  `TransientStateUtility` scaffolding.
- Removed stale cuboid face-projection helpers including the allocating
  `GetFace(...)` helper.
- Removed unused `Joint2D.HasAwakeParticipant()` and
  `Joint3D.HasAwakeParticipant()`.
- Fixed 3D direct-collider inactive load so loaded inactive state clears stale
  primary and mixed partition state just like 2D.

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

**Status:** Ready

**Purpose**

Build the next branch-coverage target list from current evidence, then remove
or track zombie code before writing more tests.

**Files To Inspect**

- `TestResults/coverage-roadmap-e/reports/Summary.txt`
- `TestResults/coverage-roadmap-e/method-gaps-under-90.json`
- `src/Gravitas`
- `tests/Gravitas.Tests`
- `docs/feature-work/issue-tracker.md`
- `docs/feature-work/benchmark-signal-hardening-backlog.md`

**Tasks**

- [ ] Rerun coverage into `TestResults\coverage-branch-hardening`.
- [ ] Regenerate ReportGenerator, CRAP, and method-gap artifacts.
- [ ] Sort the remaining branch gaps by CRAP score, uncovered branch count,
      runtime risk, and hot-path relevance.
- [ ] For each top candidate, classify it using the zombie-code triage
      protocol before writing tests.
- [ ] Delete or simplify obvious zombie code immediately when it has no valid
      runtime path.
- [ ] Record suspected bugs or parity smells in `issue-tracker.md`; if fixed in
      the same pass, record them under `Resolved Issues`.
- [ ] Update this plan with the new prioritized branch campaign list.

**Exit Criteria**

- A current branch-gap shortlist exists.
- Any discovered correctness risk is visible in the issue tracker.
- Obvious zombie code is deleted instead of carried forward.

### Workstream 2: Collision And Shape Geometry Branches

**Status:** Pending

**Purpose**

Close high-risk collision/geometry branch gaps that affect physical behavior,
while deleting stale fallback paths that no valid collider shape can reach.

**Likely Candidate Areas**

- `src/Gravitas/CollisionHandling/Pairs/3D/CollisionPair.cs`
- `src/Gravitas/CollisionHandling/Detection/3D`
- `src/Gravitas/CollisionHandling/Detection/2D/CollisionDetection2D.cs`
- `src/Gravitas/CollisionHandling/Detection/Mixed/CollisionDetectionMixed*.cs`
- `src/Gravitas/CollisionHandling/Detection/3D/ConvexColliderSupport.cs`
- `tests/Gravitas.Tests/CollisionHandling`

**Tasks**

- [ ] Review `SetImmovableDirection(...)` and decide whether it is real runtime
      behavior, stale API, or an issue-tracker candidate.
- [ ] Cover or delete remaining mixed prism branch gaps for cuboid, capsule,
      cylinder, cone, and triangle/mesh slab paths.
- [ ] Review 2D convex/compound collision branches for duplicate tests and
      unreachable defensive branches.
- [ ] Review 3D mesh/cuboid/cylinder/capsule branches for measured behavior
      gaps rather than private-helper branch steering.
- [ ] If GJK/simplex branches remain high-risk, refactor only the simplex policy
      needed for direct deterministic tests; avoid brittle geometry setups that
      exist only for coverage.
- [ ] Run focused collision tests, then full `Release`, then coverage.

### Workstream 3: Query Reducer And Shape-Cast Branches

**Status:** Pending

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

- [ ] Cover cone query branches only where gameplay-facing behavior changes:
      axis hits, cap/rim hits, triangle intersections, rejected candidates, and
      conservative reducer markers.
- [ ] Review `ConvexSweepQueryWorker` uncovered branches for true exact
      shape-cast behavior versus stale compound fallback.
- [ ] Cover mixed sphere/2D reducer branches that still decide exact vs
      conservative hit reporting.
- [ ] Remove duplicate batch tests that only repeat scalar query correctness.
- [ ] Run focused query tests, allocation-sensitive query checks where
      relevant, full `Release`, then coverage.

### Workstream 4: Serialization, Replay, And Authoring Branches

**Status:** Pending

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

- [ ] Review `ColliderShapeDefinition` equality/hash branches for meaningful
      authored-shape behavior; delete weak comparison paths if the API no
      longer needs them.
- [ ] Cover replay hash branches that distinguish authoritative state from
      solver-cache or diagnostic state.
- [ ] Audit inactive/active load, trigger, material, ignored-layer, hierarchy,
      and compound-part serialization parity between 2D and 3D.
- [ ] Record any parity bug in `issue-tracker.md` before fixing it.
- [ ] Run focused serialization/determinism tests, `ReleaseLean`, full
      `Release`, then coverage.

### Workstream 5: Service Lifecycle, Partition, And Runtime Branches

**Status:** Pending

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

- [ ] Review deferred partition refresh branches in 2D/3D/mixed services for
      valid lifecycle paths, duplicate work, and stale cleanup logic.
- [ ] Cover service reset/deactivate/idempotency branches that protect pooling
      and context reuse.
- [ ] Cover queued CCD handoff branches only where they affect deterministic
      runtime behavior.
- [ ] Delete lifecycle branches that only preserve obsolete initialization
      orders.
- [ ] Run focused runtime/partition tests, full `Release`, then coverage.

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
