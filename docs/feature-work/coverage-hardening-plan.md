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

Fresh checkpoint after Workstream 30 mixed exact-prism and swept-cone worker
coverage:

| Metric | Baseline | Current | Short-Term Gate | Long-Term Target |
| --- | ---: | ---: | ---: | ---: |
| Line coverage | 87.3% | 95.4% | 90% | 100% |
| Branch coverage | 74.1% | 85.5% | 90% | 100% |
| Method coverage | 86.5% | 94.7% | 90% | 100% |
| Tests | 974 passed | 1461 passed | green | green |

At the current denominator, the 90% branch gate requires at least 10,037 covered
branches. The latest run covered 9,537 of 11,152 branches, leaving roughly 500
net branch outcomes to cover or delete. The remaining gap is too large for one
low-value audit pass, so the next phase should attack broad branch families
instead of adding one-off residue tests.

Current evidence:

- Coverage report:
  `TestResults/coverage-branch-hardening-ws30-final/reports/Summary.txt`
- Branch shortlist:
  `TestResults/coverage-branch-hardening-ws30-final/branch-gap-shortlist.csv`
- CRAP hotspots:
  `TestResults/coverage-branch-hardening-ws30-final/crap-scores.txt`
- Method gaps:
  `TestResults/coverage-branch-hardening-ws30-final/method-gaps-under-90.json`
- Latest uncovered method count: 195.

Top branch-gap groups by source area:

| Area | Missing Branch Outcomes |
| --- | ---: |
| `Core/3D` | 262 |
| `Core/2D` | 248 |
| `CollisionHandling/Detection` | 247 |
| `Queries/3D` | 158 |
| `Queries/Mixed` | 124 |
| `Core/Mixed` | 88 |
| `Queries/2D` | 82 |
| `Colliders/3D` | 60 |
| `Colliders/2D` | 57 |
| `Constraints/3D` | 43 |
| `CollisionHandling/Response` | 42 |
| `Constraints/2D` | 41 |

## Historical Summary

This plan began after trigger collider hardening with 87.3% line, 74.1% branch,
and 86.5% method coverage. The campaign so far raised the suite to 95.4% line,
85.3% branch, and 94.7% method coverage while also finding real defects and
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
| Workstream 27 | 95.3% line / 85.1% branch / 94.7% method | 1415 | Mixed partition bucket invariants, finite-slab reducer miss exits, projected capsule/cylinder helper cleanup, and unreachable mixed query guard removal. |
| Workstream 28 | 95.4% line / 85.3% branch / 94.7% method | 1429 | Public 2D query validation, focused 2D geometry-worker ray/sweep edge cases, 3D overlap-sphere filter branches, swept-sphere worker guards, convex support unsupported dispatch parity, and dead static-sphere dynamic traversal removal. |
| Workstream 29 | 95.4% line / 85.3% branch / 94.7% method | 1438 | Pure-runtime 2D/3D collider load partition refresh/cleanup, 2D grounding load canonicalization, replay hierarchy null-check cleanup, and low-value lifecycle/diagnostic residue classification. |
| Workstream 30 | 95.4% line / 85.5% branch / 94.7% method | 1461 | Mixed primitive-prism exact axis misses, swept-sphere cone iterative miss exits, and branch-90 roadmap refresh. |

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

- [x] Cover or simplify mixed prism branches for cuboid, capsule, cylinder,
      cone, triangle, mesh, and compound targets.
- [x] Cover finite-slab projection rows for side, cap, disk-boundary, rotated,
      and no-hit cases through public mixed query calls where visible, with
      focused reducer-policy tests for exits hidden behind candidate
      collection.
- [x] Cover mixed source/target eligibility, pair retention, sleeping,
      inactive, trigger, and partition-retention rows where event or contact
      behavior changes.
- [x] Review `PhysicsMixedPartition.Distribute`, `IsEmpty`, and retained
      membership branches for duplicate state or stale defensive guards.
- [x] Run focused mixed detection/query/partition tests and coverage
      collection.

**Result**

- Final W27 report: 95.3% line / 85.1% branch / 94.7% method coverage, 1,415
  `Release` tests passing under coverage.
- Focused mixed detection/query/partition test slice passed with 173 tests.
- Removed unreachable mixed reducer guards around projected finite slabs,
  zero-thickness slab duplicates, and validated 2D convex slab vertex counts.
- Split projected capsule/cylinder finite-slab handling away from an impossible
  fallback branch and added compact reducer miss-exit coverage for no
  projection, moving-away, and out-of-range hits.
- Added direct mixed-partition bucket tests for duplicate adds, missing removes,
  awake tracking, sorted copy helpers, and dynamic deactivation.

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

- [x] Triage convex support simplex branches through public convex/mesh
      collision or sweep behavior unless the branch is a proven private policy.
- [x] Cover 2D query residue around raycast circle, sweep circle edge,
      capsule-segment versus convex, and convex mover versus convex target.
- [x] Cover and triage 3D query residue around overlap sphere hit construction,
      swept cone/sphere guards, sweep normals, OBB raycasts, and triangle
      closest-point behavior.
- [x] Review contact notification and response CRAP hotspots for branch
      consolidation before adding more event permutation tests.
- [x] Run focused collision/query tests, allocation-sensitive query checks, and
      coverage collection.

**Result**

- Final W28 report: 95.4% line / 85.3% branch / 94.7% method coverage, 1,429
  `Release` tests passing under coverage.
- Focused allocation-sensitive query checks passed for 2D ray/area/capsule
  queries, 3D ray/cone queries, and 2D/3D query batch paths.
- Added public 2D query-service tests for non-positive AABB validation,
  clockwise convex polygon validation, and zero-length circle sweeps, plus
  focused 2D geometry-worker tests for circle tangent/out-of-range raycasts,
  bounds misses, zero-displacement mover sweeps, and separated convex mover
  sweeps.
- Added 3D overlap-sphere query coverage for linked-constraint suppression,
  trigger inclusion, and non-positive radius exits, plus swept-sphere worker
  coverage for zero-length and unsupported-collider dispatch.
- Removed dead dynamic partition traversal from the static-only overlap-sphere
  query path instead of preserving an unreachable branch.
- Remaining cone-iteration and convex-simplex private-policy branches stay in
  the branch shortlist for future targeted work. OBB slab and triangle
  closest-point residue also remain in the shortlist where existing public tests
  already cover the main behavior, but no brittle private-helper tests were
  added.

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

- [x] Cover only meaningful load/replay/authoring branches that affect
      deterministic continuation or public API invariants.
- [x] Audit low-value diagnostic draw/view/logger branches one more time and
      leave construction noise alone unless visitor dispatch or adapter behavior
      is meaningful.
- [x] Review 2D/3D parity in lifecycle, partition, and authoring behavior while
      touching these areas.
- [x] Delete stale wrappers, duplicate tests, and defensive branches that are
      unreachable under validated public APIs.
- [x] Run full `Release`, full `ReleaseLean`, coverage, CRAP, and method-gap
      scripts.

**Result**

- Final W29 report: 95.4% line / 85.3% branch / 94.7% method coverage, 1,438
  `Release` tests passing under coverage.
- Full non-instrumented `Release` passed with 1,438 tests.
- `ReleaseLean` passed with 1,407 tests.
- Added pure-runtime 2D and 3D collider load tests proving inactive loads clear
  primary partition membership and active shape loads refresh stale primary
  partitions without touching mixed membership.
- Added a 2D load canonicalization test proving invalid loaded grounding values
  fall back to the default support-up direction, clamp negative probe radius to
  zero, and normalize loaded support normals before simulation resumes.
- Removed impossible replay hierarchy null-success checks after
  `ColliderRegistry.TryGetById`; the registry contract only reports success for
  live stored colliders.
- Classified retained-partition callback fallback, diagnostic view
  construction, and logger facade residue as low-value or defensive behavior to
  leave alone for now rather than weakening reset resilience or bloating tests.

### Workstream 30: Branch-90 Gate And 100% Roadmap Refresh

**Purpose**

Use the fresh evidence after Workstream 29 to continue closing the 90% branch
gate and refresh the roadmap from a new full coverage run.

**Tasks**

- [x] Cover or simplify the remaining 2D/3D kinematic CCD handoff branches,
      especially ignored-target, source/target push, and mixed handoff rows.
- [x] Attack the mixed capsule/cylinder/cone/cuboid/triangle prism branch
      family through public collision scenarios or compact policy tests where
      candidate setup hides the invariant.
- [x] Triage convex support simplex update branches and 3D swept-cone/OBB query
      residue without adding brittle private tests unless the policy is truly
      internal-only.
- [x] Review 2D grounding refresh, 2D pair replay/hash, mixed pair candidate,
      and retained-partition defensive branches for real behavior versus
      stale guard code.
- [x] Generate fresh coverage, CRAP, method-gap, and branch-shortlist artifacts.
- [x] Confirm line and method coverage remain above 90%, then calculate the
      remaining branch delta against the 90% gate.
- [x] Run `git diff --check` and a final review.
- [x] Evaluate whether branch coverage is at least 90%; it is not yet, so the
      remaining 100% roadmap stays deferred until the branch gate is met.
- [x] If branch coverage is still below 90%, add one evidence-based follow-up
      workstream from the newest shortlist and explain why the prior four did
      not close the gap.

**Result**

- Final W30 report: 95.4% line / 85.5% branch / 94.7% method coverage, 1,461
  `Release` tests passing under coverage.
- Full non-instrumented `Release` passed with 1,461 tests.
- `ReleaseLean` passed with 1,430 tests.
- Added compact public mixed narrow-phase cases where broad bounds overlap but
  exact primitive-prism SAT rejects on real cuboid, capsule, cylinder, and cone
  axes. This moved the mixed prism family without introducing private helper
  tests.
- Added direct `SweptSphereQueryWorker` cone cases for broad-phase-visible
  iterative misses: moving away from the cone surface and a short sweep whose
  computed step exceeds remaining travel.
- Triage result: `ConvexColliderSupport.UpdateTriangle` and
  `UpdateTetrahedron` remain meaningful internal GJK simplex-policy gaps. Public
  collision setup already covers the main behavior but does not expose enough
  simplex-region control. Prefer extracting a small internal simplex policy
  helper with direct tests in the next geometry workstream over reflection-based
  private tests.
- Branch coverage did not reach 90%. The denominator stayed at 11,152 branches;
  the suite covers 9,537, leaving roughly 500 net branch outcomes to cover or
  delete.

### Workstream 31: GJK Simplex, Convex Geometry, And Narrow-Phase Policy

**Purpose**

Close the largest remaining `CollisionHandling/Detection` family without
forcing brittle public coordinates. The fresh CRAP/method-gap data points at
GJK simplex updates, mixed triangle/prism SAT, compound part contact ordering,
mesh/cone geometry, and 2D segment/convex dispatch.

**Candidate Areas**

- `src/Gravitas/CollisionHandling/Detection/3D/ConvexColliderSupport.cs`
- `src/Gravitas/CollisionHandling/Detection/3D`
- `src/Gravitas/CollisionHandling/Detection/2D`
- `src/Gravitas/CollisionHandling/Detection/Mixed`
- `tests/Gravitas.Tests/CollisionHandling`
- `tests/Gravitas.Tests/MixedDimensions`

**Tasks**

- [ ] Extract or expose a focused internal GJK simplex policy helper only if it
      makes `UpdateLine`, `UpdateTriangle`, and `UpdateTetrahedron` directly
      testable without reflection or public-coordinate hunting.
- [ ] Cover GJK simplex regions through deterministic simplex-policy tests and
      keep public `ConvexColliderSupport.Intersects` tests for end-to-end
      behavior.
- [ ] Continue mixed prism/triangle SAT coverage for capsule-2D edge axes,
      cone planar/edge axes, and triangle-prism misses where public fixtures
      stay stable.
- [ ] Audit compound and mesh contact helper branches for stale reduced-SAT or
      duplicate ordering logic before adding tests.
- [ ] Run focused collision/mixed tests, full `Release`, coverage, and CRAP.

### Workstream 32: CCD Handoff, Dynamic Response, And Rotational Residue

**Purpose**

Attack the still-dominant `Core/3D` and `Core/2D` branch families around
kinematic handoff, dynamic exact TOI, rotational CCD, target eligibility,
grounding support selection, and response handoff queues.

**Candidate Areas**

- `src/Gravitas/Core/3D/SolidBody.ContinuousCollision.*.cs`
- `src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.*.cs`
- `src/Gravitas/Core/3D/GravitasPhysicsService.ContinuousCollision.cs`
- `src/Gravitas/Core/2D/GravitasPhysics2DService.Grounding.cs`
- `tests/Gravitas.Tests/CollisionHandling`
- `tests/Gravitas.Tests/Physics2D`
- `tests/Gravitas.Tests/MixedDimensions`

**Tasks**

- [ ] Cover 2D and 3D kinematic handoff branches that still distinguish ignored
      targets, constrained axes, mixed target projection, and non-closing
      response velocity.
- [ ] Cover dynamic exact TOI branches for equal/near-equal velocities,
      unsupported exact convex source paths, ignored target ordering, and mixed
      target eligibility.
- [ ] Review rotational CCD branch residue for duplicated 2D/3D policy and
      remove stale fallback branches where frame-rate or mobility invariants
      already prove the condition.
- [ ] Cover 2D support/grounding refresh and contact-candidate tie-break
      branches where they affect deterministic support state.
- [ ] Run focused CCD/grounding tests, full `Release`, `ReleaseLean`, and
      coverage.

### Workstream 33: Query Reducer, Shape-Cast, And Ray Geometry Residue

**Purpose**

Close remaining query branches in 3D, 2D, and mixed shape-cast reducers while
avoiding low-value DTO/view coverage. The fresh shortlist still shows OBB
raycasts, cone ray side checks, convex sweep normals, mixed finite-slab
reducers, and 2D sweep geometry.

**Candidate Areas**

- `src/Gravitas/Queries/3D`
- `src/Gravitas/Queries/2D`
- `src/Gravitas/Queries/Mixed`
- `tests/Gravitas.Tests/Queries`
- `tests/Gravitas.Tests/MixedDimensions`

**Tasks**

- [ ] Cover 3D OBB/cone ray and sweep-normal branches through public query
      worker/service scenarios where possible.
- [ ] Cover `ConvexSweepQueryWorker` distance/normal residue with direct worker
      tests only when public query setup cannot isolate the geometry.
- [ ] Cover mixed sphere/circle finite-slab reducer residue for point-in-space,
      capsule side, convex slab side faces, and projection-distance exits.
- [ ] Cover 2D query sweep/raycast edge cases that represent real public
      behavior, especially capsule-segment and convex mover residue.
- [ ] Run focused query allocation checks where hot paths are touched, then
      full coverage.

### Workstream 34: Lifecycle, Replay, Pair, And Low-Value Surface Gate

**Purpose**

Use the next fresh shortlist to separate remaining real runtime lifecycle
branches from low-value construction/view noise. This is the cleanup pass after
the geometry, CCD, and query families move.

**Candidate Areas**

- `src/Gravitas/Core/2D`
- `src/Gravitas/Core/3D`
- `src/Gravitas/Core/Mixed`
- `src/Gravitas/Colliders`
- `src/Gravitas/Diagnostics`
- `src/Gravitas/Partitions`
- `tests/Gravitas.Tests/Serialization`
- `tests/Gravitas.Tests/Replay`
- `tests/Gravitas.Tests/Diagnostics`

**Tasks**

- [ ] Cover remaining pair candidate, replay hash, retained partition,
      active-state, trigger/contact notification, and load/apply branches that
      affect deterministic continuation.
- [ ] Re-audit diagnostic view constructors and low-value getter branches; keep
      them out of scope unless visitor dispatch, adapter behavior, or
      serialization makes them meaningful.
- [ ] Delete stale lifecycle guards that valid registries, frame-rate
      invariants, or shape validation already make impossible.
- [ ] Generate fresh coverage, CRAP, method-gap, and branch-shortlist artifacts.
- [ ] If branch coverage reaches 90%, rewrite this plan as the remaining 100%
      roadmap. If not, create one more evidence-based gate workstream from the
      newest shortlist.

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
| 2026-07-08 | 95.3% | 84.9% | 94.7% | 1411 passed | Workstream 26 completed; CCD handoff lifecycle, rotational Auto/miss paths, same-velocity dynamic TOI, kinematic shape-exact misses, and redundant CCD guard removal. Branches covered: 9447/11125. |
| 2026-07-08 | 95.3% | 85.1% | 94.7% | 1415 passed | Workstream 27 completed; mixed partition bucket invariants, finite-slab reducer miss exits, projected capsule/cylinder cleanup, and unreachable mixed query guard removal. Branches covered: 9476/11130. |
| 2026-07-08 | 95.4% | 85.3% | 94.7% | 1429 passed | Workstream 28 completed; public 2D query validation, 2D geometry-worker edge cases, 3D overlap/swept-sphere guard coverage, convex support dispatch parity, and dead static-sphere traversal removal. Branches covered: 9523/11160. |
| 2026-07-08 | 95.4% | 85.3% | 94.7% | 1438 passed | Workstream 29 completed; pure-runtime collider load partition refresh/cleanup, 2D grounding load canonicalization, replay hierarchy null-check cleanup, and low-value lifecycle residue classification. Branches covered: 9523/11152. |
| 2026-07-08 | 95.4% | 85.5% | 94.7% | 1461 passed | Workstream 30 completed; mixed exact primitive-prism axis misses, swept-sphere cone iterative miss exits, and branch-90 roadmap refresh. Branches covered: 9537/11152. |
