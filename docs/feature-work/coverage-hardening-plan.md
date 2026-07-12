# Coverage Hardening Plan

**Date:** 2026-07-11  
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
`TestResults/coverage-task12-authoritative-reviewed-full-rerun/56f638df-6d66-4c69-a5e7-cfa6ba170f93/coverage.cobertura.xml`.

| Metric | Current | Covered / Total | Remaining | Target |
| ------ | ------: | --------------: | --------: | -----: |
| Lines | 99.3% | 26,113 / 26,278 | 165 | 100% |
| Branches | 97.6% | 10,107 / 10,354 | 247 | 100% |
| Methods | 98.9% | 3,433 / 3,470 | 37 | 100% |

The full coverage-enabled `Release` suite passes 2,255/2,255 tests, and
`ReleaseLean` builds both targets without warnings. Branch coverage is now the
primary constraint, but the remaining line and method gaps must close from the
same final artifact.

### Immediate Next Block

Finish the mixed pair lifecycle family before changing target:
`Core/Mixed/GravitasMixedCollisionService.Pairs.cs`,
`CollisionHandling/Pairs/Mixed/CollisionPairMixed.cs`,
`Core/Mixed/GravitasMixedCollisionService.Response.cs`, and the mixed
notification paths in `LSCollider.Events.cs` and `LSCollider2D.cs`. Together
they account for 16 uncovered lines, 29 uncovered branch outcomes, and two
uncovered methods. Close pooling modes, stale lifetime tokens, nested callback
mutation, trigger admission, resting pairs, response admission, and exact
enter/stay/exit order as one contract. The newly completed 3D pair safeguards
are a reference, not a reason to force dimensional parity where mixed ownership
differs.

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
| 1 | Mixed pair lifecycle and response family | 16 | 29 | 2 | Mixed callbacks, pooled lifetime, response admission, ordering, and cleanup. |
| 2 | `Colliders/2D/LSPolygonCollider2D.cs` | 4 | 7 | 0 | Authored polygon validation and geometry. |
| 3 | `CollisionHandling/Detection/3D/CollisionDetection.Cuboid.cs` | 3 | 7 | 0 | Cuboid feature selection and separation. |
| 4 | `Core/2D/SolidBody2D.ContinuousCollision.Hits.cs` | 2 | 7 | 0 | 2D hit admission and deterministic reduction. |
| 5 | `CollisionHandling/Detection/2D/CollisionDetection2D.cs` | 4 | 6 | 0 | Pure 2D dispatch and degenerate-shape rejection. |
| 6 | `Core/2D/SolidBody2D.ContinuousCollision.Dynamic.cs` | 4 | 6 | 0 | Dynamic 2D CCD handoff and substep state. |
| 7 | `CollisionHandling/Detection/3D/ConvexColliderSupport.cs` | 5 | 5 | 0 | Convex volume tests and degenerate geometry. |
| 8 | `CollisionHandling/Response/3D/CollisionResponse.cs` | 3 | 5 | 0 | Reclose response outcomes no longer reached after stale-pair response removal. |

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
- [x] Close and independently review the 3D pair lifecycle, including callback
      mutation, per-side admission, exception retry, stale queue snapshots,
      exact exit order, and pooled-lifetime reuse.
- [ ] Close the mixed pair/response files as one cohesive dimensional family,
      including their callback, response-admission, and pooled-lifetime
      counterparts.
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
- [ ] Reassess the mixed circle-against-3D reducers, cuboid detection, 2D
      detection, and polygon geometry after each fresh artifact.
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
- [ ] Complete the remaining 2D CCD hit, dynamic-response, and reduction paths
      as one behavior family.
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
      and classify all 37 current method gaps.
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

Completed campaigns established broad 2D, 3D, mixed, CCD, query, partition,
lifecycle, replay, serialization, diagnostics, and authored-shape coverage.
They also removed stale transient state, duplicate reducers and wrappers,
unreachable fallbacks, dead serialization paths, and incorrect ownership
branches. That history now informs the rules above without competing with the
remaining plan of attack.
