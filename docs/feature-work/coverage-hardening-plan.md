# Coverage Hardening Plan

**Date:** 2026-07-06  
**Status:** Active - 95% branch roadmap; branch-90 gate met  
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

The current short-term goal is the next practical chunk:

- Keep line, branch, and method coverage at or above the 90% release floor.
- Raise branch coverage to at least 95%.
- Remove zombie code instead of testing it.
- Condense or delete duplicate tests while adding only high-signal branch tests.
- Record suspected bugs, parity gaps, stale behavior, and deeper RCA items in
  [`issue-tracker.md`](issue-tracker.md) instead of burying them in plan notes.
- After the 95% branch gate is met, refresh the remaining 100% roadmap from a
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
    -CoverageThreshold 95 `
    -BranchThreshold 95 `
    -Filter below-threshold `
    > TestResults\coverage-branch-hardening\method-gaps-under-95.json
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

Fresh checkpoint after the branch-90 gate pass:

| Metric          |   Baseline |     Current | Release Floor | Short-Term Target | Long-Term Target |
| --------------- | ---------: | ----------: | ------------: | ----------------: | ---------------: |
| Line coverage   |      87.3% |      96.99% |           90% |    keep above 90% |             100% |
| Branch coverage |      74.1% |      90.03% |           90% |               95% |             100% |
| Method coverage |      86.5% |      96.66% |           90% |    keep above 90% |             100% |
| Tests           | 974 passed | 1756 passed |         green |             green |            green |

The branch-90 gate is met, but the buffer is intentionally treated as fragile.
The latest run covered 9,573 of 10,632 branch outcomes. The 90% threshold
requires 9,569 covered outcomes, leaving a +4 branch buffer.

At the current denominator, 95% branch coverage requires 10,101 covered branch
outcomes. That means roughly +528 net branch outcomes must be covered or deleted
to hit the next gate. This is a directional budget, not a test count; new code
and branch removal can move the denominator.

Current evidence:

- Coverage artifact:
  `TestResults/coverage-branch-hardening-live33/1b2cbff6-fc6c-457f-9033-123dd6908878/coverage.cobertura.xml`
- Covered branches: 9,573 / 10,632.
- Branch-90 threshold: 9,569 covered branches.
- Branch-95 threshold at the current denominator: 10,101 covered branches.
- Method-gap artifact:
  `TestResults/coverage-branch-hardening-live33/method-gaps-under-95.json`

## Historical Context

This plan began after trigger collider hardening with 87.3% line, 74.1% branch,
and 86.5% method coverage. The first target was 90% across the board. That gate
is now met; the next target is 95% branch coverage.

| Phase                   | Coverage Result                             | Tests | Main Outcome                                                                                                                                                                                                                                                          |
| ----------------------- | ------------------------------------------- | ----: | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Baseline                | 87.3% line / 74.1% branch / 86.5% method    |   974 | Starting point after trigger collider hardening.                                                                                                                                                                                                                      |
| First coverage campaign | 90.0% line / 76.0% branch / 91.0% method    |  1025 | Line and method gates reached; branch campaign stayed active.                                                                                                                                                                                                         |
| Roadmaps A-E            | 92.0% line / 77.9% branch / 92.2% method    |  1094 | Replay hash, diagnostics, dispatch, geometry, lifecycle, and collider authoring coverage.                                                                                                                                                                             |
| Workstreams 1-6         | 93.3% line / 79.7% branch / 93.6% method    |  1146 | Zombie-code sweep plus query, collision, serialization, lifecycle, and diagnostics hardening.                                                                                                                                                                         |
| Workstreams 7-18        | 94.7% line / 82.8% branch / 94.6% method    |  1261 | CCD handoff, mixed response, query reducers, contact geometry, hierarchy, convex support, mixed pair retention, and joint-island cleanup.                                                                                                                             |
| Workstreams 19-30       | 95.4% line / 85.5% branch / 94.7% method    |  1461 | Kinematic CCD, query reducers, mixed pair lifecycle, collision geometry, serialization, partition lifecycle, constraints, ragdolls, and branch-90 roadmap refresh.                                                                                                    |
| Workstreams 31-34       | 95.6% line / 85.8% branch / 94.7% method    |  1493 | GJK simplex policy extraction, 2D/raycast query geometry, convex sweep contracts, mixed trigger lifecycle, cone-prism hit coverage, and duplicate candidate-gate cleanup.                                                                                             |
| Branch-90 gate pass     | 96.99% line / 90.03% branch / 96.66% method |  1756 | CCD/query/mixed/lifecycle residue, stale reducer cleanup, 3D/2D active-state parity, collider registry and partition lifecycle hardening, contact/trigger lifecycle coverage, constraint validation parity, settings matrix coverage, and shape authoring guardrails. |

High-value outcomes from the 90% campaign:

- Query and collision coverage now spans 2D, 3D, and mixed dimensions, including
  capsule sweeps, compound shapes, mixed finite slabs/prisms, cone queries,
  convex sweeps, mesh contacts, 2D manifolds, and public batch-query contracts.
- CCD coverage now includes kinematic handoff lifecycle, rotational Auto skips
  and kinematic near-misses, same-velocity dynamic relative motion, shape-exact
  kinematic proxy misses, and explicit `Both` versus `Mixed` runtime-mode
  behavior.
- Runtime lifecycle coverage now protects partition reset, retained membership
  cleanup, context reset, coroutine lifecycle, clock hooks, trigger-driven
  deferred refresh, inactive collider loading, stale collider-ID reuse safety,
  and hierarchy reparent/rejection behavior.
- Serialization and replay coverage now protects shape-definition equality/hash
  semantics, compound authored payloads, polygon serialization, invalid trigger
  load rejection, pending CCD handoff hash state, and authoritative-vs-cache
  hash distinctions.
- Real bug fixes found by coverage review include rotated cuboid raycast slab
  clipping, 2D active-state mixed partition cleanup, inactive direct-collider
  load cleanup, mixed CCD handoff queue draining, joint replay load counts,
  ragdoll validation atomicity, 2D distance replay canonicalization,
  `ColliderType2D.None` collision matrix parity, and 2D motor mutation
  validation.
- Cleanup removed unused transient-state scaffolding, stale cuboid face
  projection helpers, unused joint awake helpers, dead acceleration flags, stale
  immovable-contact state, duplicate mesh-cylinder fallback logic, unused mesh
  edge-normal storage, unreachable CCD fallback helpers, and duplicate
  voxel/contact/rotational/mixed-bounds policies.

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
- When a suspected bug, parity issue, stale behavior, or deeper RCA item
  appears, add it to [`issue-tracker.md`](issue-tracker.md). If it is fixed
  immediately, add it under `Resolved Issues` with RCA and verification
  evidence.
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

| Classification                  | Action                                                                                                                     |
| ------------------------------- | -------------------------------------------------------------------------------------------------------------------------- |
| Real behavior                   | Add focused tests through public APIs when readable, or small internal-policy tests when public setup hides the invariant. |
| Zombie code                     | Delete it, then run focused and full validation.                                                                           |
| Duplicate code                  | Centralize or collapse it, then update tests around the shared path.                                                       |
| Defensive impossible branch     | Replace with clearer validation or remove the branch if invariants already prevent it.                                     |
| Suspected bug/parity gap        | Record in `issue-tracker.md`, write a failing regression, fix root cause, then move/record as resolved.                    |
| Performance signal              | Record in `benchmark-signal-hardening-backlog.md` unless it is already a confirmed correctness defect.                     |
| Low-value DTO/view getter noise | Leave it unless construction, visitor dispatch, serialization, or host-adapter behavior is meaningful.                     |

Do not let an issue hide inside a result bullet. The 3D inactive direct-collider
load parity bug is the model: it should be visible in the issue tracker with RCA
and verification, even if the fix is small.

## Active 95% Branch Roadmap

The next phase should begin from fresh coverage and CRAP data, then work broad
families instead of single-branch residue. Getting from 90.03% to 95% is roughly
+528 net branch outcomes at the current denominator, so each workstream should
either move a meaningful family or delete code that no longer earns its keep.

### Workstream A: Fresh 95% Inventory

**Purpose**

Rebuild the branch shortlist from the current source so the next implementation
passes are evidence-led.

**Candidate Areas**

- `TestResults/coverage-branch-hardening-live33/**/coverage.cobertura.xml`
- `TestResults/coverage-branch-hardening-live33/method-gaps-under-95.json`
- `src/Gravitas/Core`
- `src/Gravitas/CollisionHandling`
- `src/Gravitas/Queries`
- `src/Gravitas/Constraints`
- `src/Gravitas/Colliders`
- `src/Gravitas/Partitions`

**Tasks**

- [ ] Run fresh `Release` coverage, ReportGenerator summary, CRAP scores, and
      95-threshold method-gap extraction.
- [ ] Build a branch shortlist grouped by source area and missing branch count.
- [ ] Classify each top branch family as real behavior, zombie code, duplicate
      code, defensive invariant, suspected bug, performance signal, or low-value
      surface.
- [ ] Record deeper bugs in [`issue-tracker.md`](issue-tracker.md) before
      starting implementation work.
- [ ] Update the coverage checkpoint table with the fresh baseline.

### Workstream B: Stale, Duplicate, And Defensive Branch Sweep

**Purpose**

Avoid paying for old complexity. Delete or centralize obvious residue before
adding more tests.

**Candidate Areas**

- Repeated CCD proxy/contact helpers in `src/Gravitas/Core/3D` and
  `src/Gravitas/Core/2D`
- Duplicate mixed reducer fallbacks in `src/Gravitas/Queries/Mixed`
- Low-value wrappers and thin overloads in `src/Gravitas/Queries`
- Repeated contact/trigger policy branches in `src/Gravitas/Colliders`
- Defensive branches already guaranteed by collider-shape validation, registry
  contracts, or fixed-step frame-rate invariants

**Tasks**

- [ ] Remove unreachable guards proven by frame-rate, registry, collider-shape,
      body-lifecycle, or validated-authored-shape invariants.
- [ ] Collapse duplicate policy code where 2D, 3D, and mixed paths have drifted
      into near-identical helpers.
- [ ] Delete stale tests that only preserve behavior the runtime no longer
      supports.
- [ ] Run focused tests for every touched subsystem, then full `Release`.
- [ ] Rerun coverage and record whether the denominator moved down.

### Workstream C: CCD And Body Runtime Branch Families

**Purpose**

Close high-value body and service branches that affect deterministic motion,
handoff, partition refresh, and simulation ordering.

**Candidate Areas**

- `src/Gravitas/Core/3D/SolidBody.ContinuousCollision.*.cs`
- `src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.*.cs`
- `src/Gravitas/Core/3D/SolidBody*.cs`
- `src/Gravitas/Core/2D/SolidBody2D*.cs`
- `src/Gravitas/Core/3D/GravitasPhysicsService*.cs`
- `src/Gravitas/Core/2D/GravitasPhysics2DService*.cs`
- `src/Gravitas/Runtime/GravitasWorldContext.cs`

**Tasks**

- [ ] Cover dynamic and mixed TOI selection branches that change chosen impact
      time, response velocity, or handoff ownership.
- [ ] Cover kinematic handoff budget and ignored-target rows where public
      behavior differs.
- [ ] Cover rotational CCD tie-break, miss, replacement, and frozen-axis skip
      rows through valid fast-mover scenarios.
- [ ] Review body lifecycle parity while touching 2D and 3D paths; fix small
      asymmetries immediately and record larger ones in the issue tracker.
- [ ] Keep allocation-sensitive CCD and partition paths measured when touched.

### Workstream D: Mixed Collision, Prism, And Finite-Slab Reducers

**Purpose**

Attack one of the largest remaining branch families: mixed 2D/3D contact,
finite-slab, prism, candidate, and response behavior.

**Candidate Areas**

- `src/Gravitas/CollisionHandling/Detection/Mixed`
- `src/Gravitas/Queries/Mixed`
- `src/Gravitas/Core/Mixed`
- `src/Gravitas/Partitions/Mixed`
- `src/Gravitas/CollisionHandling/Response/Mixed`

**Tasks**

- [ ] Cover prism and slab SAT misses for cuboid, capsule, cylinder, cone,
      triangle, mesh, and compound paths where authored shapes can reach them.
- [ ] Cover clipped capsule/cylinder/cone projection rows that change exact hit
      distance, normal, or reducer kind.
- [ ] Cover mixed island wake, candidate deduplication, trigger/contact,
      sleeping, inactive, and retained-partition behavior where runtime events
      or response differ.
- [ ] Delete private reducer permutations that valid authored shapes cannot
      reach and that do not protect a public invariant.
- [ ] Run focused mixed detection, mixed query, mixed partition, and mixed
      response tests before full coverage.

### Workstream E: Pure Query Geometry And Shape Casts

**Purpose**

Harden pure 2D and 3D query geometry branches that still carry real runtime
meaning.

**Candidate Areas**

- `src/Gravitas/Queries/3D`
- `src/Gravitas/Queries/3D/Sweeps`
- `src/Gravitas/Queries/2D`
- `src/Gravitas/CollisionHandling/Detection/2D`
- `src/Gravitas/CollisionHandling/Detection/3D`

**Tasks**

- [ ] Cover cone, mesh, convex, and compound query reducers where hit ordering,
      normal selection, or closest/all-hit behavior changes.
- [ ] Cover conservative-advancement exits, edge/corner tie-breaks, and
      closest-hit replacement rows.
- [ ] Prefer public query-service tests for reducer, normal, ordering, and
      all-hit contracts.
- [ ] Extract small internal policy helpers only when public setup hides a
      stable physics invariant and reflection/private tests would be brittle.
- [ ] Leave thin overload wrappers and already-covered unsupported dispatch
      cases alone unless they hide a real API inconsistency.

### Workstream F: Collision Response, Constraints, And Ragdoll Branches

**Purpose**

Cover physically meaningful solver branches without duplicating existing stress
tests.

**Candidate Areas**

- `src/Gravitas/CollisionHandling/Response`
- `src/Gravitas/CollisionHandling/Pairs`
- `src/Gravitas/Constraints/3D`
- `src/Gravitas/Constraints/2D`
- `src/Gravitas/CollisionHandling/Contacts`

**Tasks**

- [ ] Cover friction clamp, restitution cutoff, warm-start, mixed correction,
      and contact lifecycle rows where response differs.
- [ ] Cover joint solver row/no-row transitions, cached impulse cleanup, limit
      rows, motor rows, and diagnostic emission only when those behaviors are
      observable.
- [ ] Condense duplicate constraint stress tests that assert the same solver
      branch with less diagnostic value.
- [ ] Record solver-stability or performance concerns in the benchmark backlog
      when coverage work exposes a measured signal.

### Workstream G: Collider Load, Replay, Authoring, Partitions, And 95 Gate

**Purpose**

Sweep deterministic continuation and lifecycle residue, then validate whether
the 95% branch gate is met.

**Candidate Areas**

- `src/Gravitas/Colliders/3D`
- `src/Gravitas/Colliders/2D`
- `src/Gravitas/Colliders/Definitions`
- `src/Gravitas/Colliders/Mesh`
- `src/Gravitas/Partitions/3D`
- `src/Gravitas/Partitions/2D`
- `src/Gravitas/Serialization`
- `src/Gravitas/Diagnostics`
- `src/Gravitas/Support`

**Tasks**

- [ ] Cover `ApplyLoadedState`, hierarchy ordinal replay, active/inactive
      partition refresh, mesh topology validation, trigger/contact lifecycle,
      and authoring guardrails only where deterministic continuation changes.
- [ ] Keep diagnostic visitor dispatch, disabled-path allocation behavior, and
      adapter-visible payload correctness; leave DTO getter/constructor noise
      for the long-term 100% pass.
- [ ] Run full `Release`, full `ReleaseLean`, coverage, CRAP, method-gap, and
      `git diff --check`.
- [ ] If branch coverage is at least 95%, update this plan to a 100% roadmap.
- [ ] If branch coverage is still below 95%, add the next alpha-lettered
      evidence-based workstream from the newest shortlist.

## Coverage Checkpoints

Update this compact table after each coverage collection. Keep only meaningful
campaign checkpoints; do not add a row for every focused test filter.

| Date       |   Line | Branch | Method | Tests       | Notes                                                                                                                                                                                                                                                                                |
| ---------- | -----: | -----: | -----: | ----------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 2026-07-05 |  87.3% |  74.1% |  86.5% | 974 passed  | Baseline captured with `tests/Gravitas.Tests/coverlet.runsettings` after trigger collider hardening.                                                                                                                                                                                 |
| 2026-07-05 |  90.0% |  76.0% |  91.0% | 1025 passed | First campaign completed; line and method gates met.                                                                                                                                                                                                                                 |
| 2026-07-05 |  92.0% |  77.9% |  92.2% | 1094 passed | Roadmaps A-E completed across mixed query support, replay hash, diagnostics, collision dispatch, geometry, lifecycle, and collider authoring.                                                                                                                                        |
| 2026-07-06 |  93.3% |  79.7% |  93.6% | 1146 passed | Workstreams 1-6 completed; zombie-code sweep plus query, collision, serialization, lifecycle, and diagnostics hardening.                                                                                                                                                             |
| 2026-07-07 |  94.7% |  82.8% |  94.6% | 1261 passed | Workstreams 13-18 completed; geometry, hierarchy, convex support, CCD eligibility, mixed pair retention, and joint-island cleanup.                                                                                                                                                   |
| 2026-07-08 |  95.4% |  85.5% |  94.7% | 1461 passed | Workstream 30 completed; mixed exact primitive-prism axis misses, swept-sphere cone iterative miss exits, and branch-90 roadmap refresh. Branches covered: 9537/11152.                                                                                                               |
| 2026-07-08 |  95.6% |  85.8% |  94.7% | 1493 passed | Workstreams 31-34 completed; GJK simplex policy extraction, 2D/raycast query geometry, convex sweep worker contracts, mixed trigger lifecycle, cone-prism hit coverage, duplicate 2D/mixed candidate-gate cleanup, and next branch-gate phase refresh. Branches covered: 9561/11140. |
| 2026-07-09 | 96.99% | 90.03% | 96.66% | 1756 passed | Branch-90 gate met; CCD/query/mixed/lifecycle residue, parity cleanup, contact/trigger lifecycle coverage, constraint validation parity, settings matrix coverage, and shape authoring guardrails. Branches covered: 9573/10632.                                                     |
