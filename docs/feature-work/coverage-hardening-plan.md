# Coverage Hardening Plan

**Date:** 2026-07-06  
**Status:** Active - branch coverage to 90%  
**Owner:** Gravitas coverage, test-quality, zombie-code, and branch-quality
hardening

---

> **For agentic workers:** Treat this as a living context guide until Gravitas
> reaches 100% line, branch, and method coverage. Do not inflate coverage with
> shallow tests, generated-code chasing, stale compatibility paths, or tests for
> code that should be deleted.

## Goal

Long term, Gravitas should reach 100% line, branch, and method coverage with a
test suite that proves useful deterministic physics behavior.

The current release-hardening gate is narrower and more urgent:

- Keep line and method coverage at or above 90%.
- Raise branch coverage to at least 90%.
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
$cobertura = Get-ChildItem TestResults\coverage-branch-hardening `
    -Filter coverage.cobertura.xml `
    -Recurse |
    Sort-Object LastWriteTime -Descending |
    Select-Object -First 1 -ExpandProperty FullName

& C:\Users\david\.codex\skills\coverage-analysis\scripts\Compute-CrapScores.ps1 `
    -CoberturaPath $cobertura `
    -CrapThreshold 25 `
    -TopN 100

& C:\Users\david\.codex\skills\coverage-analysis\scripts\Extract-MethodCoverage.ps1 `
    -CoberturaPath $cobertura `
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

Fresh checkpoint after Workstream 26 CCD handoff and rotational sweep coverage:

| Metric | Baseline | Current | Short-Term Gate | Long-Term Target |
| --- | ---: | ---: | ---: | ---: |
| Line coverage | 87.3% | 95.3% | 90% | 100% |
| Branch coverage | 74.1% | 84.9% | 90% | 100% |
| Method coverage | 86.5% | 94.7% | 90% | 100% |
| Tests | 974 passed | 1411 passed | green | green |

At the current denominator, the 90% branch gate requires at least 10,059 covered
branches. The latest run covered 9,499 of 11,176 branches, leaving roughly 560
net branch outcomes to cover or delete. The remaining gap is too large for one
low-value audit pass, so the next phase should attack broad branch families
instead of adding one-off residue tests.

Current evidence:

- Coverage report:
  `TestResults/coverage-branch-hardening-ws26-final/reports/Summary.txt`
- Branch shortlist:
  `TestResults/coverage-branch-hardening-ws26-final/branch-gap-shortlist.csv`
- Latest uncovered method count: 195.

Top branch-gap groups by source area:

| Area | Missing Branch Outcomes |
| --- | ---: |
| `Core/3D` | 263 |
| `CollisionHandling/Detection` | 258 |
| `Core/2D` | 250 |
| `Queries/3D` | 169 |
| `Queries/Mixed` | 137 |
| `Queries/2D` | 91 |
| `Core/Mixed` | 88 |
| `Constraints` | 84 |
| `Colliders/3D` | 63 |
| `Colliders/2D` | 60 |
| `Partitions` | 52 |
| `CollisionHandling/Response` | 42 |

## Historical Summary

This plan began after trigger collider hardening with 87.3% line, 74.1% branch,
and 86.5% method coverage. The campaign so far raised the suite to 95.3% line,
84.9% branch, and 94.7% method coverage while also finding real defects and
removing stale runtime branches.

| Phase | Coverage Result | Tests | Main Outcome |
| --- | --- | ---: | --- |
| Baseline | 87.3% line / 74.1% branch / 86.5% method | 974 | Starting point after trigger collider hardening. |
| First coverage campaign | 90.0% line / 76.0% branch / 91.0% method | 1025 | Line and method gates reached; branch campaign stayed active. |
| Roadmaps A-E | 92.0% line / 77.9% branch / 92.2% method | 1094 | Replay hash, diagnostics, dispatch, geometry, lifecycle, and collider authoring coverage. |
| Workstreams 1-6 | 93.3% line / 79.7% branch / 93.6% method | 1146 | Zombie-code sweep plus query, collision, serialization, lifecycle, and diagnostics hardening. |
| Workstreams 7-12 | 94.1% line / 81.3% branch / 94.4% method | 1214 | CCD handoff, mixed response, query reducer, replay, lifecycle, and shape-cast residue. |
| Workstreams 13-18 | 94.7% line / 82.8% branch / 94.6% method | 1261 | Contact geometry, hierarchy, convex support, CCD eligibility, mixed pair retention, and joint-island cleanup. |
| Workstreams 19-24 | 95.2% line / 84.5% branch / 94.7% method | 1346 | Kinematic CCD, query reducers, mixed pair lifecycle, collision geometry, serialization, partition lifecycle, constraints, and ragdoll residue. |
| Workstream 25 | 95.2% line / 84.6% branch / 94.7% method | 1391 | Low-value audit, compact lifecycle/diagnostic/support tests, 2D collision-type parity fix, and stale branch removal. |
| Workstream 26 | 95.3% line / 84.9% branch / 94.7% method | 1411 | CCD handoff lifecycle, rotational Auto/miss paths, same-velocity dynamic TOI, kinematic shape-exact misses, `Both` versus `Mixed` CCD gating, and redundant CCD invariant guard removal. |

High-value work completed:

- Query and collision coverage across 2D, 3D, and mixed dimensions, including
  capsule sweeps, compound shapes, mixed finite slabs/prisms, cone queries,
  convex sweeps, mesh contacts, 2D manifolds, and public batch-query contracts.
- CCD coverage for kinematic handoff lifecycle, rotational Auto skips and
  kinematic near-misses, same-velocity dynamic relative motion, shape-exact
  kinematic proxy misses, and explicit `Both` versus `Mixed` runtime-mode
  handoff behavior.
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
  direct-collider loads clear stale partition state, mixed CCD handoff queues
  drain through one context-owned budget before partition/discrete completion,
  3D joint replay loads keep enabled-joint counts coherent, 3D joint/ragdoll
  validation rejects incompatible angular limit payloads atomically, 2D distance
  replay-load semantics canonicalize unrestricted payloads to explicit target
  distances, `ColliderType2D.None` now resolves to `CollisionType2D.None` for
  all 2D collision matrix rows, and 2D public motor mutation rejects linear
  motors on non-prismatic joints.

Cleanup completed:

- Removed unused transient-state scaffolding, stale cuboid face-projection
  helpers, unused joint awake helpers, dead acceleration flags, stale
  immovable-contact state, duplicate mesh-cylinder fallback logic, unused mesh
  edge-normal storage, and unreachable CCD fallback helpers.
- Centralized duplicated voxel ordering, CCD contact-point selection,
  rotational hit tie-breaks, mixed bounds comparisons, and mesh embedded-volume
  reuse.
- Removed unreachable rotational CCD substep fallbacks and redundant
  non-positive-delta CCD frame-velocity branches after verifying the frame-rate
  contract keeps `DeltaTime` above `Fixed64.Epsilon`.
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

The W25 audit confirmed that branch coverage will not reach 90% by chasing
small diagnostic DTO/view branches. The next revision should close the gate by
attacking broad, high-value branch families from the latest shortlist. These
workstreams are intentionally larger than the recent residue passes so each
coverage collection can move the denominator meaningfully.

### Workstream 26: CCD Handoff, Rotational Sweep, And Dynamic TOI Branches

**Purpose**

Close the largest branch family in `Core/3D` and `Core/2D` by covering or
simplifying the remaining kinematic handoff, rotational CCD, dynamic exact TOI,
mixed CCD, and source/target eligibility branches.

**Candidate Areas**

- `src/Gravitas/Core/3D/SolidBody.ContinuousCollision.*.cs`
- `src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.*.cs`
- `src/Gravitas/Core/3D/GravitasPhysicsService.ContinuousCollision.cs`
- `src/Gravitas/Core/2D/GravitasPhysics2DService.ContinuousCollision.cs`
- `tests/Gravitas.Tests/CollisionHandling`
- `tests/Gravitas.Tests/Physics2D`
- `tests/Gravitas.Tests/MixedDimensions`

**Tasks**

- [x] Cover remaining 2D/3D rotational CCD hit, miss, replacement, and
      kinematic-source rows through public fast-mover scenarios.
- [x] Cover 2D/3D `ApplyKinematicContinuousCollisionHandoff` rows that change
      target position, source position, ignored target, or response velocity.
- [x] Cover `TryConsumeContinuousCollisionHandoff`, dynamic relative exact TOI,
      mixed target selection, and shape-exact fallback rows where public
      behavior is meaningful.
- [x] Delete impossible positive-motion, positive-delta-time, or positive-mass
      guards only when caller invariants prove them.
- [x] Run focused CCD tests, full `Release`, coverage collection, and
      `ReleaseLean` if touched code affects replay or serialization.

**Result**

- Final W26 report: 95.3% line / 84.9% branch / 94.7% method coverage, 1,411
  `Release` tests passing under coverage.
- Focused CCD tests passed for 3D, 2D, mixed, and frame-rate guard scenarios.
- Full non-instrumented `Release` passed with 1,411 tests.
- `ReleaseLean` passed with 1,384 tests.

### Workstream 27: Mixed Prism, Finite-Slab, And Query Reducer Branches

**Purpose**

Attack the mixed 2D/3D reducer and prism branch family that remains near the top
of the shortlist: cuboid/capsule/cylinder/cone/triangle prisms, finite-slab
projection, swept sphere against 2D slabs, and mixed pair candidate processing.

**Candidate Areas**

- `src/Gravitas/CollisionHandling/Detection/Mixed`
- `src/Gravitas/Queries/Mixed`
- `src/Gravitas/Core/Mixed`
- `src/Gravitas/Partitions/Mixed`
- `tests/Gravitas.Tests/MixedDimensions`
- `tests/Gravitas.Tests/Queries`

**Tasks**

- [ ] Cover or simplify mixed prism branches for cuboid, capsule, cylinder,
      cone, triangle, mesh, and compound targets.
- [ ] Cover finite-slab projection rows for side, cap, disk-boundary, rotated,
      and no-hit cases through public mixed query calls.
- [ ] Cover mixed source/target eligibility, pair retention, sleeping,
      inactive, trigger, and partition-retention rows where event or contact
      behavior changes.
- [ ] Review `PhysicsMixedPartition.Distribute`, `IsEmpty`, and retained
      membership branches for duplicate state or stale defensive guards.
- [ ] Run focused mixed detection/query/partition tests and coverage
      collection.

### Workstream 28: Collision Geometry, Convex Support, And 2D Query Branches

**Purpose**

Close the remaining geometry-heavy branches in collision detection and queries
without adding brittle private-helper tests. Prefer public collision/query
scenarios, and remove private branches that valid authored shapes cannot reach.

**Candidate Areas**

- `src/Gravitas/CollisionHandling/Detection/3D`
- `src/Gravitas/CollisionHandling/Detection/2D`
- `src/Gravitas/Queries/3D`
- `src/Gravitas/Queries/2D`
- `src/Gravitas/CollisionHandling/Response`
- `tests/Gravitas.Tests/CollisionHandling`
- `tests/Gravitas.Tests/Queries`

**Tasks**

- [ ] Cover convex support simplex branches only through public convex/mesh
      collision or sweep behavior unless the branch is a proven private policy.
- [ ] Cover 2D query residue around raycast circle, sweep circle edge,
      capsule-segment versus convex, and convex mover versus convex target.
- [ ] Cover 3D query residue around overlap sphere hit construction, swept cone,
      sweep normals, OBB raycasts, and triangle closest-point behavior.
- [ ] Review contact notification and response CRAP hotspots for branch
      consolidation before adding more event permutation tests.
- [ ] Run focused collision/query tests, allocation-sensitive query checks, and
      coverage collection.

### Workstream 29: Lifecycle, Serialization, Replay, Authoring, And Low-Value Residue

**Purpose**

Sweep the remaining non-geometry branch residue after the three broad runtime
passes: collider load/apply semantics, replay hash hierarchy ordinals,
partition lifecycle, constraints, diagnostics, logger support, and small public
authoring helpers.

**Candidate Areas**

- `src/Gravitas/Colliders`
- `src/Gravitas/Core`
- `src/Gravitas/Constraints`
- `src/Gravitas/Diagnostics`
- `src/Gravitas/Partitions`
- `src/Gravitas/Support`
- `tests/Gravitas.Tests/Serialization`
- `tests/Gravitas.Tests/Replay`
- `tests/Gravitas.Tests/Constraints`
- `tests/Gravitas.Tests/Diagnostics`

**Tasks**

- [ ] Cover only meaningful load/replay/authoring branches that affect
      deterministic continuation or public API invariants.
- [ ] Audit low-value diagnostic draw/view/logger branches one more time and
      leave construction noise alone unless visitor dispatch or adapter behavior
      is meaningful.
- [ ] Review 2D/3D parity in lifecycle, partition, and authoring behavior while
      touching these areas.
- [ ] Delete stale wrappers, duplicate tests, and defensive branches that are
      unreachable under validated public APIs.
- [ ] Run full `Release`, full `ReleaseLean`, coverage, CRAP, and method-gap
      scripts.

### Workstream 30: Branch-90 Gate And 100% Roadmap Refresh

**Purpose**

Use the fresh evidence after Workstreams 26-29 to close the 90% branch gate. If
90% is reached, rewrite this document into the remaining 100% plan. If not,
identify the minimum next branch family needed to cross the gate.

**Tasks**

- [ ] Generate fresh coverage, CRAP, method-gap, and branch-shortlist artifacts.
- [ ] Confirm line and method coverage remain above 90%.
- [ ] Confirm branch coverage is at least 90% or calculate the exact remaining
      branch delta.
- [ ] Run `git diff --check` and a final independent review.
- [ ] If branch coverage is at least 90%, replace this active roadmap with a
      100% coverage roadmap focused on the remaining uncovered methods and
      meaningful branch families.
- [ ] If branch coverage is still below 90%, add one evidence-based follow-up
      workstream from the newest shortlist and explain why the prior four did
      not close the gap.

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
| 2026-07-07 | 94.9% | 83.7% | 94.7% | 1295 passed | Workstreams 20-21 completed; query reducer/shape-cast geometry plus mixed pair, contact notification, trigger policy, and constrained mixed response lifecycle. Branches covered: 9315/11129. |
| 2026-07-07 | 95.0% | 83.9% | 94.7% | 1317 passed | Workstream 22 completed; cuboid normal bug fix, 2D convex/mixed prism/contact fallback branch hardening, and stale clipping guard removal. Branches covered: 9339/11119. |
| 2026-07-07 | 95.1% | 84.2% | 94.7% | 1330 passed | Workstream 23 completed; active collider load partition refresh/stale bucket cleanup, replay hierarchy mutation, mixed replay pair-state hash, 2D/3D partition lifecycle parity, and stale distribution guard removal. Branches covered: 9360/11115. |
| 2026-07-08 | 95.2% | 84.5% | 94.7% | 1346 passed | Workstream 24 completed; 3D joint load-count fix, 3D ragdoll atomic validation parity, 2D distance load canonicalization, 2D motor mutation validation, and constraint solver edge coverage. Branches covered: 9398/11121. |
| 2026-07-08 | 95.2% | 84.6% | 94.7% | 1391 passed | Workstream 25 completed; low-value surface audit, 2D collision matrix parity fix, support/lifecycle/diagnostic branch coverage, and stale branch removal. Branches covered: 9388/11087. |
