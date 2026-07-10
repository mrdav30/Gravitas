# Coverage Hardening Plan

**Date:** 2026-07-10  
**Status:** Active - 95% branch gate met; 100% roadmap refreshed  
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
- Preserve the verified 95% branch gate while working toward 100% line, branch,
  and method coverage.
- Remove zombie code instead of testing it.
- Condense or delete duplicate tests while adding only high-signal branch tests.
- Record suspected bugs, parity gaps, stale behavior, and deeper RCA items in
  [`issue-tracker.md`](issue-tracker.md) instead of burying them in plan notes.
- Drive the remaining work from the fresh 95% coverage, CRAP, and method-gap
  artifacts recorded below.

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

Fresh checkpoint after the branch-95 gate pass:

| Metric          |   Baseline |     Current | Release Floor | Short-Term Target | Long-Term Target |
| --------------- | ---------: | ----------: | ------------: | ----------------: | ---------------: |
| Line coverage   |      87.3% |       98.2% |           90% |    keep above 90% |             100% |
| Branch coverage |      74.1% |       95.0% |           90% | preserve at least 95% |             100% |
| Method coverage |      86.5% |       96.6% |           90% |    keep above 90% |             100% |
| Tests           | 974 passed | 2014 passed |         green |             green |            green |

The branch-95 gate is met exactly: the authoritative run covered 9,698 of 10,208
branch outcomes. Treat the gate as fragile until later work creates a buffer;
new hand-authored branches can move the denominator below 95% even when existing
coverage does not regress.

Current evidence:

- Coverage artifact:
  `TestResults/coverage-branch-hardening-95-final2/05bb0b97-6100-424b-9d1e-8ae22eb73d4d/coverage.cobertura.xml`
- ReportGenerator summary:
  `TestResults/coverage-branch-hardening-95-final2/reports/Summary.txt`
- CRAP analysis:
  `TestResults/coverage-branch-hardening-95-final2/crap-scores.txt`
- Methods below the 95% line-or-branch threshold:
  `TestResults/coverage-branch-hardening-95-final2/method-gaps-under-95.json`
- Covered branches: 9,698 / 10,208.

## Historical Context

This plan began after trigger collider hardening with 87.3% line, 74.1% branch,
and 86.5% method coverage. The 90% release floor and 95% branch gate are now
met; the remaining target is 100% across the board without weakening test
quality.

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
| Branch-93 review pass   |  97.7% line /  93.0% branch /  96.6% method |  1906 | Workstreams A-G plus follow-on branch-residue passes; added high-signal coverage for authored shape factories, collider geometry fallback normals/frontal areas, 2D batch sweep filtering, 2D partition stale-ID/idempotence guards, manifold replacement tie-breaks, constraint solver-body filters, and compact runtime-state helpers. Stopping here for review before the 95% push. |
| Branch-95 gate pass     |  98.2% line /  95.0% branch /  96.6% method |  2014 | Focused 3D/2D/mixed CCD, mixed narrow phase and partitioning, query geometry, collision response, pair lifecycle, and `SolidBody2D` lifecycle hardening. Removed dead pair-culling serialization state and duplicate runtime guards; independently reviewed each concentrated block. |

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

## 95% Branch Campaign Planning Record

The following workstreams are retained as the planning record for the campaign
that moved branch coverage from 93% to 95%. The implementation followed these
families in concentrated blocks, with focused tests and independent reviews,
rather than treating every checklist item as an obligation to preserve stale
code.

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
- [x] Run full `Release`, full `ReleaseLean`, coverage, CRAP, method-gap, and
      `git diff --check`.
- [x] Branch coverage reached at least 95%; update this plan to a 100% roadmap.
- The below-95 fallback was not needed.

## Active 100% Coverage Roadmap

The authoritative 95% run leaves 454 uncovered lines, 510 uncovered branch
outcomes, and 117 uncovered methods. The 95-threshold extraction identifies 478
methods whose line or branch coverage remains below 95%; many are tiny wrappers
or DTO construction paths, so method count alone is not a priority signal.

### Workstream H: Residual Runtime Branch Families

- Preserve a 95% branch buffer before broadening production code.
- Start with branch-dense behavior that still changes runtime outcomes:
  rotational CCD in 2D and 3D, queued 3D CCD handoffs, 2D segment clipping,
  cylinder/capsule/cone separating axes, and 3D overlap hit reducers.
- Continue the zombie/duplicate/defensive classification before adding tests.
- Keep each pass scoped to one related block and obtain an independent review
  before moving to the next block.

### Workstream I: Method And Line Residue

- Review the 117 uncovered methods by public reachability and behavior value.
- Cover meaningful host-facing constructors, query overloads, replay paths,
  partition state transitions, and diagnostic payloads through real workflows.
- Delete unreachable or duplicate wrappers instead of creating invocation-only
  tests. Leave generated MemoryPack code excluded.

### Workstream J: Complexity And CRAP Reduction

- The fresh CRAP report flags five methods above 30. Four are fully line-covered
  and remain flagged because their complexity alone is 32-81; treat those as
  refactor candidates, not coverage failures.
- Close or simplify the only under-covered flagged method,
  `ConvexSweepQueryWorker.ClosestPointOnTriangleToOrigin` (93.9% line coverage,
  CRAP 30.2), when working the related convex-sweep block.
- Reassess high-complexity collision dispatch and contact notification only when
  a focused refactor makes determinism and reviewability clearer.

### Workstream K: Final 100% Validation

- Run focused tests, full `Release`, and full `ReleaseLean` for every completed
  block.
- Regenerate coverage, ReportGenerator, CRAP, and method-gap artifacts from one
  explicit Cobertura source of truth.
- Require independent review of correctness, determinism, allocation behavior,
  and test signal before declaring 100% complete.

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
| 2026-07-10 |  97.7% |  93.0% |  96.6% | 1906 passed | Branch-93 review checkpoint met; pass43 covered 9600/10322 branches. Added targeted coverage for shape factories, cone/cylinder/capsule geometry edges, 2D batch sweep filtering, partition stale-ID/idempotence guards, manifold replacement, and constraint solver-body filters. |
| 2026-07-10 |  98.2% |  95.0% |  96.6% | 2014 passed | Branch-95 gate met exactly at 9698/10208 branches. Concentrated passes hardened CCD, mixed detection/partitioning, query geometry, collision response, pair lifecycle, and `SolidBody2D`; final Release and ReleaseLean suites passed. |
