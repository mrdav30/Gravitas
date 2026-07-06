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

Fresh checkpoint after Workstream 6 diagnostics and low-value surface audit:

| Metric | Baseline | Current | Short-Term Gate | Long-Term Target |
| --- | ---: | ---: | ---: | ---: |
| Line coverage | 87.3% | 93.3% | 90% | 100% |
| Branch coverage | 74.1% | 79.7% | 90% | 100% |
| Method coverage | 86.5% | 93.6% | 90% | 100% |
| Tests | 974 passed | 1146 passed | green | green |

At the current denominator, the 90% branch gate requires 10,236 covered
branches. The latest run covered 9,066 of 11,373 branches, leaving roughly
1,170 net branch outcomes to cover or delete. Treat this as a focused branch
campaign, not a single gate-check workstream.

Current evidence:

- Coverage report:
  `TestResults/coverage-branch-hardening-ws6/reports/Summary.txt`
- Coverage collection:
  `dotnet test tests\Gravitas.Tests\Gravitas.Tests.csproj --configuration Release --collect:"XPlat Code Coverage" --settings tests\Gravitas.Tests\coverlet.runsettings`
  passed with 1146 tests.
- Branch shortlist:
  `TestResults/coverage-branch-hardening-ws6/branch-gap-shortlist.txt`
- Latest CRAP extraction reported 34 flagged methods and 237 uncovered
  methods.

## Completed Coverage Summary

The completed coverage campaign moved Gravitas from 87.3% line, 74.1% branch,
and 86.5% method coverage to 93.3% line, 79.7% branch, and 93.6% method
coverage.

High-value work completed:

- Query and collision coverage across 2D, 3D, and mixed dimensions, including
  capsule sweeps, compound shapes, mixed compound boundaries, cone queries,
  convex-vs-compound sweeps, and shape-cast reducer reporting.
- Runtime lifecycle coverage for partition reset, retained membership cleanup,
  context reset, coroutine lifecycle, clock hooks, trigger-driven deferred
  refresh, inactive collider loading, and mixed partition state cleanup.
- Serialization and replay coverage for shape-definition equality/hash
  semantics, compound authored replay payloads, polygon serialization, and
  authoritative-vs-cache hash distinctions.
- Constraint, ragdoll, grounding, diagnostics, trigger, material, and logger
  coverage where behavior is host-visible or deterministic-state-relevant.
- Real bug fixes found by coverage review: rotated cuboid raycasts now clip
  local slabs, 2D active-state changes clear mixed partitions, and inactive 3D
  direct-collider loads clear stale partition state.

Cleanup completed:

- Removed unused transient-state scaffolding, stale cuboid face-projection
  helpers, unused joint awake helpers, dead acceleration flags, stale
  immovable-contact state, `LSCollider.HeightPos`, unused mesh edge-normal
  storage, and an allocating mesh triangle wrapper.
- Removed duplicate mesh-cylinder fallback candidate logic and centralized
  2D/3D/mixed `WorldVoxelIndex` ordering.
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

Workstreams 1-6 are now historical context. The next phase starts at Workstream
7 and should be reranked after each coverage run. The ordering below comes from
`TestResults/coverage-branch-hardening-ws6/branch-gap-shortlist.txt` plus
runtime risk, hot-path relevance, and duplicate/zombie-code likelihood.

| Order | Branch Family | Why It Comes Next |
| ---: | --- | --- |
| 1 | CCD handoff and shape-exact branches | Highest-risk uncovered runtime behavior, many high-complexity branches, and direct impact on deterministic fast-mover correctness. |
| 2 | Mixed prism, response, trigger, and contact branches | Large remaining branch clusters in first-class mixed collision/response behavior. |
| 3 | Query reducer and shape-cast residue | Public query correctness and exact-vs-conservative reporting still have meaningful branch gaps. |
| 4 | Collision geometry and convex support residue | Remaining GJK/simplex, mesh, convex, and 2D clipping branches can hide real geometric false positives or false negatives. |
| 5 | Lifecycle, serialization, replay, and authoring residue | Lower-risk after prior passes, but still worth sweeping for parity bugs and zombie code before the gate. |
| 6 | Branch 90 gate and 100% roadmap refresh | Only run as a true gate after focused campaigns make 90% realistic. |

### Workstream 7: CCD Handoff And Shape-Exact Branch Families

**Status:** Pending

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
- `tests/Gravitas.Tests/ContinuousCollision`
- `tests/Gravitas.Tests/Core`

**Tasks**

- [ ] Re-open the latest branch shortlist and classify each CCD entry as real
      behavior, duplicate code, defensive impossible branch, or bug/parity
      smell before adding tests.
- [ ] Cover 3D and 2D kinematic-dynamic push eligibility, ignored-target, and
      no-target branches through public simulation scenarios.
- [ ] Cover mixed 2D/3D CCD push/handoff branches where they affect observable
      movement, contact, trigger, or replay state.
- [ ] Cover rotational CCD branches in 3D and 2D with deterministic angular
      sweeps, including no-hit, replacement-hit, and exact-hit paths.
- [ ] Cover shape-exact refinement and swept-normal fallback branches through
      valid collider configurations rather than private-helper steering.
- [ ] Review candidate-ordering duplicate entries and centralize/delete any
      redundant branch logic before writing more tests around it.
- [ ] Record any CCD semantic bug or parity gap in `issue-tracker.md`; if fixed
      immediately, move it to resolved with RCA and verification evidence.
- [ ] Run focused CCD tests, full `Release`, coverage collection, and
      `ReleaseLean` if serialization or conditional compilation changes.

**Exit Criteria**

- CCD-related branch gaps are measurably reduced or explicitly reclassified.
- No new private-only tests assert implementation detail without behavior.
- Any discovered CCD bug is fixed or tracked in `issue-tracker.md`.
- The progress log records the new coverage numbers and test count.

### Workstream 8: Mixed Prism, Response, Trigger, And Contact Branches

**Status:** Pending

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

- [ ] Cover meaningful cuboid, capsule, cylinder, cone, and triangle prism
      branches with physical mixed pair scenarios.
- [ ] Cover mixed response branches for dynamic/static, dynamic/kinematic,
      trigger, sensor-like bodyless, frozen-axis, friction, and restitution
      behavior where those branches are valid.
- [ ] Review mixed trigger/contact notification branches for duplicate 2D/3D
      behavior and centralize shared assertions where useful.
- [ ] Review mixed candidate processing and pair-retention branches for stale
      lifecycle behavior before adding tests.
- [ ] Avoid one-test-per-shape padding when one shared test can prove the branch
      family.
- [ ] Run focused mixed tests, full `Release`, and coverage collection.

**Exit Criteria**

- Mixed prism and response gaps are reduced through behavior-driven tests or
  cleanup.
- Trigger/contact behavior stays symmetric where intended and explicit where it
  is dimension-specific.
- Any mixed parity issue is fixed or visible in `issue-tracker.md`.

### Workstream 9: Query Reducer And Shape-Cast Residue

**Status:** Pending

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

- [ ] Cover `TryBuildOverlapSphereHit(...)`, rotated cuboid raycast slab
      residue, cone sweep/contact construction, and concave-mesh cone query
      branches that are public-query-visible.
- [ ] Cover mixed sphere-against-2D reducer branches that still decide exact
      vs conservative reporting.
- [ ] Cover 2D query clipping/sweep branches through public 2D query or CCD
      flows when possible.
- [ ] Review zero-length or invalid request branches and prefer public
      validation tests over private worker calls unless the worker is itself a
      reusable geometry policy.
- [ ] Keep batch-query coverage focused on range/output contracts, not repeated
      scalar geometry.
- [ ] Run focused query tests, allocation-sensitive query checks where
      relevant, full `Release`, and coverage collection.

**Exit Criteria**

- Remaining query branch tests prove user-visible query behavior.
- Exact, conservative, miss, and ordering branches are covered where meaningful.
- No duplicate scalar/batch coverage padding is added.

### Workstream 10: Collision Geometry And Convex Support Residue

**Status:** Pending

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

- [ ] Review `ConvexColliderSupport` simplex branches for valid geometry cases
      versus unreachable defensive transitions.
- [ ] Cover mesh-cone, mesh-cuboid, and axis-aligned cuboid manifold branches
      only through valid collider setups.
- [ ] Cover 2D convex clipping, manifold replacement, and compound collision
      branches that still affect contact quality.
- [ ] Delete fallback code if no valid collider setup can reach it after the
      stronger geometry paths added in prior hardening work.
- [ ] Add or update regression tests for any false-positive or false-negative
      geometry issue discovered during the sweep.
- [ ] Run focused collision tests, full `Release`, and coverage collection.

**Exit Criteria**

- Remaining geometry branch gaps are either covered, deleted, or explicitly
  classified as defensive guardrails.
- No geometry test relies on magic coordinates without documenting the contact
  or separation invariant it proves.

### Workstream 11: Lifecycle, Serialization, Replay, And Authoring Residue

**Status:** Pending

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

- [ ] Re-check retained partition, direct load, active-state, and deferred
      refresh branches for 2D/3D/mixed parity.
- [ ] Cover replay and serialization branches only through valid save/load,
      replay hash, or `IRecordable` flows.
- [ ] Cover constraint record/load or solver branch residue only where it proves
      meaningful deterministic continuation or solver state.
- [ ] Delete stale authoring or lifecycle compatibility branches when the new
      public API makes them impossible.
- [ ] Run focused serialization/determinism/runtime tests, full `Release`,
      `ReleaseLean`, and coverage collection.

**Exit Criteria**

- Remaining lifecycle and serialization branch gaps are low-risk, tracked, or
  covered by meaningful behavior tests.
- `ReleaseLean` remains green when serialization or conditional compile paths
  are touched.

### Workstream 12: Branch 90 Gate And 100% Roadmap Refresh

**Status:** Pending

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
