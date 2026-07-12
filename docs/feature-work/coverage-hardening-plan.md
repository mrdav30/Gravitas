# Coverage Hardening Plan

**Date:** 2026-07-12  
**Status:** Active - closing the final gap to 100% coverage  
**Owner:** Gravitas coverage, test-quality, zombie-code, and branch-quality
hardening

> **For agentic workers:** Own one cohesive source block until its tests,
> focused coverage, full-suite verification, and independent review are
> complete. Do not switch subsystems to harvest easier branches.

## Mission

Reach 100% line, branch, and method coverage for hand-authored Gravitas code
without weakening the suite or preserving code that should be deleted.

Coverage is evidence, not the product. Every retained test must prove a real
deterministic behavior, invariant, lifecycle contract, failure mode, ordering
rule, or public API. Invocation-only tests and denominator manipulation do not
count.

## Current Checkpoint

The authoritative artifact is:
`TestResults/coverage-task20-authoritative-reviewed-full-rerun/95aa4cda-427e-4e65-8d74-514ce8d81674/coverage.cobertura.xml`.

| Metric | Current | Covered / Total | Remaining | Target |
| ------ | ------: | --------------: | --------: | -----: |
| Lines | 99.53% | 26,170 / 26,293 | 123 | 100% |
| Branches | 98.31% | 10,167 / 10,342 | 175 | 100% |
| Methods | 99.02% | 3,435 / 3,469 | 34 | 100% |

The full coverage-enabled `Release` suite passes 2,325/2,325 tests, and
`ReleaseLean` builds both targets without warnings. Branch coverage is now the
primary constraint, but the remaining line and method gaps must close from the
same final artifact.

### Immediate Next Block

Finish `src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.Dynamic.cs`
before changing target. The current artifact reports three uncovered lines and
five uncovered branch outcomes. Treat dynamic target eligibility, constrained
relative motion, time-of-impact response, remaining-step handoff, and temporary
state restoration as one CCD contract. Extend the already-established 2D CCD
public workflows instead of creating a private reducer seam.

## Rules Of Engagement

1. **Delete before testing.** Remove zombie code, duplicate policy, and guards
   already guaranteed by validated invariants.
2. **Test behavior, not reachability.** Assert results, state transitions,
   ordering, replay continuity, ownership, or deterministic failure behavior.
3. **Preserve determinism.** Keep fixed-point math, stable ordering, explicit
   state, context ownership, and fixed-step phase boundaries.
4. **Keep hot paths lean.** Do not add LINQ, reflection, unstable hash-order
   dependencies, avoidable allocations, or speculative abstractions.
5. **Do not widen exclusions.** Existing generated and compiler-generated
   exclusions may remain; hand-authored runtime code stays accountable.
6. **Record discoveries in the right place.** Correctness and parity defects
   belong in [`issue-tracker.md`](issue-tracker.md). Measured performance risks
   belong in
   [`benchmark-signal-hardening-backlog.md`](benchmark-signal-hardening-backlog.md).

Classify every uncovered outcome before editing:

| Classification | Required action |
| -------------- | --------------- |
| Real behavior | Add the smallest focused behavioral test. |
| Zombie code | Delete it after verifying callers and contracts. |
| Duplicate policy | Collapse it into the existing shared path. |
| Impossible defensive branch | Prove the invariant, then remove or simplify it. |
| Bug or parity gap | Add a failing regression, fix the root cause, and record the RCA. |
| Thin wrapper or DTO noise | Retain and cover only when its contract is meaningful. |
| Generated code | Leave it under the existing generated-code exclusion. |

## Block Completion Protocol

For each source block:

1. Parse its exact missing lines, branch outcomes, and methods from the latest
   full Cobertura artifact.
2. Trace all callers, dimensional counterparts, state owners, and lifecycle
   phases before deciding whether each gap is real or stale.
3. Add a failing behavioral regression for retained behavior or delete the
   proven-dead path.
4. Run the narrowest affected test classes.
5. Collect focused coverage and confirm the intended source block reaches 100%
   line, branch, and method coverage.
6. Run the full coverage-enabled `Release` suite.
7. Run `ReleaseLean` when serialization, MemoryPack shims, conditional
   compilation, or package shape is touched.
8. Obtain an independent correctness and test-quality review and resolve every
   actionable finding.
9. Generate a fresh full artifact, update this checkpoint, and rerank the
   remaining gaps before selecting the next block.

If a block is genuinely blocked, record the blocker and move to another whole
block. Do not leave a partially investigated family without a handoff note.

## Active Battle Plan

The queue below comes from the current full artifact. Its order is provisional:
finish the active block, regenerate coverage, then rerank. Never switch targets
mid-block merely because another branch looks easier.

### Priority Queue

| Order | Source block | Lines | Branches | Methods | Focus |
| ----: | ------------ | ----: | -------: | ------: | ----- |
| 1 | `Core/2D/SolidBody2D.ContinuousCollision.Dynamic.cs` | 3 | 5 | 0 | Dynamic 2D CCD handoff and substep state. |
| 2 | `Core/3D/SolidBody.ContinuousCollision.Hits.cs` | 3 | 5 | 0 | 3D hit admission and deterministic reduction. |
| 3 | `Partitions/RetainedPartitionLifecycle.cs` | 3 | 5 | 0 | Retained partition teardown, owner validation, and aggregate failure ordering. |
| 4 | `CollisionHandling/Response/2D/CollisionResponse2D.cs` | 2 | 5 | 0 | 2D response admission and anchored-body outcomes. |
| 5 | `Queries/Mixed/GravitasQueryMixedService.CircleAgainst3DReducers.cs` | 4 | 4 | 0 | Mixed circle/capsule reducers and conservative fallback admission. |
| 6 | `Core/3D/GravitasPhysicsService.SupportTypes.cs` | 3 | 3 | 0 | Stable joint keys and ownerless endpoint ordering. |
| 7 | `Queries/3D/GravitasQuery3DService.Batch.cs` | 3 | 3 | 0 | Batch validation, empty work, and stable aggregate ordering. |
| 8 | `Core/2D/GravitasPhysics2DService.ContinuousCollision.cs` | 2 | 3 | 0 | CCD queue ownership and stale candidate cleanup. |
| 9 | `Diagnostics/GravitasDiagnosticSink.Draw.cs` | 2 | 3 | 0 | Draw-buffer limits and deterministic disabled behavior. |
| 10 | `Materials/PhysicsMaterial.cs` | 2 | 3 | 0 | Combine policy ties, load defaults, and deterministic validation. |

### Phase 1: Core Runtime And Service Ownership

- [x] Complete and independently review
      `Constraints/3D/GravitasConstraint3DService.cs`.
- [x] Complete and independently review
      `Core/3D/GravitasPhysicsService.cs`.
- [x] Complete `Core/2D/GravitasPhysics2DService.cs` and verify intentional
      2D/3D lifecycle parity.
- [x] Close and independently review the 2D pair and response lifecycle,
      including callback mutation, shell reuse, trigger ordering, pooling, and
      dense cleanup capacity.
- [x] Close and independently review 3D response island admission, sleeping
      suppression, sparse joint traversal, anchored participants, rootless
      contacts, and single-contact dispatch.
- [x] Close and independently review the residual 3D response block by fixing
      the vacuous coincident-center setup and asserting zero-normal rejection
      through unchanged position, velocity, and warm-start state.
- [x] Close and independently review the 3D pair lifecycle, including callback
      mutation, per-side admission, exception retry, stale queue snapshots,
      exact exit order, and pooled-lifetime reuse.
- [x] Close and independently review the mixed pair/response family, including
      stale queued candidates, pooled lifetimes, nested removal snapshots,
      sleeping/rootless response admission, per-side callbacks, and rebound
      suppression.
- [x] Close residual world-context outcomes through real owned/attached
      lifetime, phase-routing, reset, and disposal workflows.
- [x] Close and independently review residual 3D body-motion outcomes through
      initialize, simulate, late-simulate, reset, deactivate, shell reuse,
      grounded friction, anisotropic gyro, and queued CCD workflows.

Exit condition: the selected service family reports 100% line, branch, and
method coverage from focused tests; the full artifact confirms the gains; no
registration, partition, pair, constraint, or host binding survives teardown
incorrectly.

### Phase 2: Collision And Query Geometry

- [x] Complete `CollisionDetection.Mesh.cs` through public collision workflows
      for meaningful dispatch, topology, separation, and contact-reduction
      outcomes.
- [x] Complete `RaycastSegmentWorker.cs` with exact hit, miss, filtering,
      stale-candidate, tie, and deterministic ordering assertions.
- [x] Close `LSPolygonCollider2D.cs` through authored/load/degenerate-scale
      workflows and translation-stable centroid/inertia math for both windings,
      compound offsets/scales, and arbitrary reference points.
- [x] Close cuboid detection through rotated face separation, exact AABB
      manifolds, zombie-guard removal, and insertion-ordered OBB/capsule SAT
      ties after independent review exposed pooled hash-order dependence.
- [x] Close and independently review pure 2D detection, including first-source
      separation, capsule end-cap fallback, ultra-thin axes, winding-independent
      features, containment-aware MTV depth, and signed off-origin axes.
- [x] Close and independently review 3D convex support and cone-volume GJK,
      including initial-origin and post-simplex epsilon contacts, bounded
      fixed-point cycles, nearby separating controls, and conservative
      no-witness exhaustion.
- [x] Close and independently review mixed embedded-volume boundary geometry,
      including unsupported-shape fallback, round coincident directions,
      authored-first ties, exact polygon edges, valid authoring invariants, and
      the `Fixed64.MaxValue` first-candidate sentinel regression.
- [ ] Reassess the mixed circle-against-3D reducers,
      `AxisProjectionHelper`, and remaining geometry after each fresh artifact.
- [ ] Delete reducer permutations or fallback branches that valid authored
      shapes and validated callers cannot reach.

Exit condition: retained geometry branches change an asserted hit, distance,
normal, penetration, material, or ordering result. Private seams are allowed
only for isolated pure reducers that cannot be expressed reliably through a
public workflow.

### Phase 3: Motion, CCD, Grounding, And Constraints

- [x] Complete `SolidBody.Motion.cs` with fixed-step state-transition tests.
- [x] Complete `SolidBody2D.ContinuousCollision.Helpers.cs` and tighten its
      successful-hit caller contracts.
- [x] Complete and independently review 2D CCD hit admission and reduction,
      including static/mixed precedence, stale and newly filtered candidates,
      deterministic replacement, exact non-closing rejection, and temporary
      transform restoration.
- [ ] Complete the remaining 2D CCD dynamic-response paths as the adjacent
      behavior family.
- [x] Close and independently review residual 2D grounding through automatic,
      manual, cached-support, callback-reentrancy, shell-reuse, query, and
      deterministic contact-candidate workflows.
- [ ] Verify dimensional parity only where the physical models are intended to
      match; keep 2D, 3D, and mixed behavior explicit elsewhere.

Exit condition: every retained branch protects a real motion, time-of-impact,
support, constraint, budget, or mobility invariant, and every deleted branch
has a caller-proven impossibility argument.

### Phase 4: Method And Public-Surface Closure

- [ ] Complete the four remaining `ContactManifold` methods through meaningful
      construction, mutation, and reduction contracts or delete unused surface.
- [ ] Regenerate the uncovered-method inventory from the latest full artifact
      and classify all 34 current method gaps.
- [ ] Delete unused wrappers, aliases, constructors, and helpers instead of
      invoking them solely for coverage.
- [ ] Cover retained query overloads, diagnostic payloads, authored-shape
      validation, and replay/load entry points through real host workflows.
- [ ] Keep runtime IDs, delegates, caches, partitions, query stamps, and host
      bindings out of serialized identity.

Exit condition: the fresh artifact reports zero uncovered hand-authored methods
and no production method was introduced merely to make testing easier.

### Phase 5: Final Convergence

- [ ] Rebuild the complete line, branch, method, and CRAP inventory from one
      explicit full-suite Cobertura artifact.
- [ ] Resolve every residual item with a behavioral test, deletion,
      consolidation, or an already-established generated-code exclusion.
- [ ] Condense duplicate tests exposed by the final sweep while preserving
      distinct behavior and regression signal.
- [ ] Run full `Release`, full `ReleaseLean`, coverage, CRAP, method-gap, and
      `git diff --check` verification.
- [ ] Obtain an independent final review of correctness, determinism, hot-path
      allocations, lifecycle ownership, serialization, and test signal.

## Coverage Accounting

- Use one newly created results directory for each authoritative full run.
- Do not merge stale result directories or compare totals from different
  artifacts as if they were one checkpoint.
- Do not add hand-authored source exclusions, `ExcludeFromCodeCoverage`, or
  coverage-only conditional compilation.
- Do not inflate the branch denominator with production branches whose only
  purpose is to be covered by tests.
- A source block is complete only when line, branch, and method coverage all
  reach 100% in focused evidence and remain closed in the next full artifact.
- Complexity alone is not a reason to refactor an already covered method.
  Simplify high-CRAP methods only when the change reduces real correctness,
  review, or maintenance risk.

## Completion Gate

This plan is complete only when all of the following are true:

- 100% line coverage for hand-authored Gravitas code.
- 100% branch coverage for hand-authored Gravitas code.
- 100% method coverage for hand-authored Gravitas code.
- Full `Release` tests pass under coverage collection.
- `ReleaseLean` builds both targets without warnings.
- No generated-source or hand-authored-runtime exclusion was added to inflate
  the result.
- Every retained test proves behavior and every discovered zombie path is
  removed or consolidated.
- Final CRAP and method-gap reports come from the same authoritative coverage
  run.
- Independent final review has no unresolved actionable findings.

## Measurement

Use the repository runsettings and one explicit result directory:

```powershell
dotnet test tests\Gravitas.Tests\Gravitas.Tests.csproj `
    --configuration Release `
    --collect:"XPlat Code Coverage" `
    --settings tests\Gravitas.Tests\coverlet.runsettings `
    --results-directory TestResults\coverage-branch-hardening-100
```

Pass the resulting `coverage.cobertura.xml` explicitly to ReportGenerator and
the coverage-analysis scripts. Never allow a report tool to select an artifact
implicitly from a directory containing older runs.

## Historical Context

The detailed completed-workstream logs have been retired from the active plan.
Their tests, source changes, reviews, and coverage artifacts remain the evidence
of record.

| Checkpoint | Line | Branch | Method | Tests | Outcome |
| ---------- | ---: | -----: | -----: | ----: | ------- |
| Baseline | 87.3% | 74.1% | 86.5% | 974 | Trigger-collider hardening baseline. |
| 90% gate | 97.0% | 90.0% | 96.7% | 1,756 | Release floor cleared across runtime, collision, lifecycle, constraints, and authoring. |
| 93% review | 97.7% | 93.0% | 96.6% | 1,906 | Broad residual pass completed before the 95% push. |
| 95% gate | 98.2% | 95.0% | 96.6% | 2,014 | CCD, mixed collision, queries, response, pair lifecycle, and `SolidBody2D` hardening completed. |
| Last-mile checkpoint | 99.2% | 96.6% | 98.8% | 2,138 | Partition services, teardown/load ownership slices, compound collision, and mixed-query geometry completed and independently reviewed. |
| 3D physics service | 99.2% | 96.7% | 98.9% | 2,142 | Service wrappers and impossible guards removed; disabled phases, sparse visualization, refresh ownership, diagnostics, and cross-context cleanup independently reviewed. |
| 3D constraint service | 99.2% | 96.7% | 98.9% | 2,145 | Replay holes and ragdoll metadata pinned exactly; suppression cleanup covered; duplicate collider and resolver validation removed. |
| 2D physics service | 99.2% | 96.8% | 98.9% | 2,150 | Disabled/direct phases, non-dynamic and stale teardown, pooling-off collision, and refresh ownership completed with 2D/3D parity review. |
| 3D mesh detection | 99.3% | 96.9% | 98.9% | 2,156 | Convex sphere and SAT outcomes covered, callerless wrapper removed, fallback signal strengthened, and disconnected-convex false-positive tracked. |
| 2D CCD helpers | 99.3% | 97.0% | 98.9% | 2,163 | Inheritance, closing, static/kinematic, and mixed policies covered; impossible geometry and duplicate hit guards removed. |
| World context | 99.3% | 97.1% | 98.9% | 2,165 | Ownership validation and disposal serialized; disabled phases preserve pending CCD handoffs; strong disposal-scoped registry contract documented and independently reviewed. |
| 3D raycast segment worker | 99.3% | 97.2% | 98.9% | 2,172 | Fixed-point tangent false negative corrected; finite cone, OBB, mesh-plane, disabled-output, duplicate, and root-bound outcomes closed with mutation-sensitive review. |
| 2D pair and response lifecycle | 99.3% | 97.4% | 98.9% | 2,194 | Callback-safe snapshots, ordered enter/exit delivery, lifetime-version shell reuse, trigger eligibility, busy-pair pooling exclusion, dense cleanup capacity, and caller-proven solver guards independently reviewed. |
| 2D grounding lifecycle | 99.3% | 97.5% | 98.9% | 2,215 | Manual ownership, cached support, query/contact lifetime validation, callback replacement, nested body/pair snapshots, pooled pair generations, and automatic invalidation independently reviewed. |
| 3D body motion | 99.4% | 97.6% | 98.9% | 2,226 | Reset/reuse stores, angular friction, analytic gyroscopic precession, total-step acceleration, queued CCD handoff, sleep/wake, and zombie correction state independently reviewed. |
| 3D response islands | 99.4% | 97.7% | 98.9% | 2,229 | Sleeping and single-contact islands, sparse and anchored joints, live rootless contacts, and the caller-impossible joint-root guard independently reviewed. |
| 3D and mixed pair lifetime hardening | 99.3% | 97.6% | 98.9% | 2,255 | 3D pair coverage closed; callback-safe snapshots, admitted-side retry, exception-safe teardown, mixed lifetime tokens, deferred nested exits, and stale-generation suppression independently reviewed. |
| Mixed pair and response closure | 99.4% | 97.8% | 98.9% | 2,279 | Mixed pair, response, and notification paths reached 100%; zombie getters and impossible admission branches removed; response admission assertions strengthened under independent review. |
| 2D polygon geometry | 99.4% | 97.9% | 98.9% | 2,286 | Polygon coverage reached 100%; empty/same-count loads and degenerate scale covered; centroid/area/inertia made translation-stable after fixed-point cancellation and overflow regressions. |
| 3D cuboid detection | 99.45% | 98.01% | 98.99% | 2,290 | Cuboid coverage reached 100%; rotated and axis-aligned separation/manifold outcomes covered; redundant guards removed; pooled hash-order normal selection replaced with authored insertion order and independently reviewed. |
| 2D CCD hit reduction | 99.47% | 98.10% | 98.99% | 2,295 | Hit reduction reached 100%; mixed/static arbitration, stale and filtered candidates, deterministic replacement, exact non-closing rejection, rotational restoration, and a caller-impossible relative-motion guard were independently reviewed. |
| 2D narrow-phase detection | 99.49% | 98.16% | 99.02% | 2,307 | Detection reached 100%; clockwise false negatives and containment depth were corrected; signed interval exits decoupled normals from off-origin host transforms; exact owner manifolds and clip invariants were independently reviewed. |
| 3D convex support GJK | 99.50% | 98.21% | 99.02% | 2,315 | Convex support reached 100%; exact touching cycles and epsilon simplex outcomes were covered; bounded exhaustion now preserves contact without conservative leakage beyond the fixed-point tolerance band. |
| Mixed embedded 2D geometry | 99.52% | 98.26% | 99.02% | 2,325 | Embedded boundary geometry reached 100%; authored-first ties, exact boundary distances, authoring invariants, unsupported fallback, and maximum-distance compound candidates were covered and independently reviewed. |
| 3D degenerate response | 99.53% | 98.31% | 99.02% | 2,325 | Response reached 100% after a vacuous coincident-center regression was corrected; zero-normal/no-fallback contacts now prove no position, velocity, or warm-start mutation under independent review. |

Completed campaigns established broad 2D, 3D, mixed, CCD, query, partition,
lifecycle, replay, serialization, diagnostics, and authored-shape coverage.
They also removed stale transient state, duplicate reducers and wrappers,
unreachable fallbacks, dead serialization paths, and incorrect ownership
branches. That history now informs the rules above without competing with the
remaining plan of attack.
