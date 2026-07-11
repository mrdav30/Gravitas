# Coverage Hardening Plan

**Date:** 2026-07-10  
**Status:** Active - closing the 95% to 100% coverage gap  
**Owner:** Gravitas coverage, test-quality, zombie-code, and branch-quality
hardening

> **For agentic workers:** Work one related source block at a time. Do not jump
> to a new subsystem until the current block has focused tests, fresh coverage,
> and an independent review.

## Mission

Move Gravitas from the verified 95% branch checkpoint to 100% line, branch, and
method coverage without weakening the suite or preserving code that should be
deleted.

Coverage is evidence, not the product. Every change must improve or protect a
real deterministic behavior, invariant, lifecycle contract, failure mode, or
public API. Generated-code chasing and invocation-only tests do not count.

## Current Gap

Authoritative checkpoint:
`TestResults/coverage-branch-hardening-95-final2/05bb0b97-6100-424b-9d1e-8ae22eb73d4d/coverage.cobertura.xml`.

| Metric   | Current | Covered / Total | Remaining | Target |
| -------- | ------: | --------------: | --------: | -----: |
| Lines    |   98.2% | 25,438 / 25,892 |       454 |   100% |
| Branches |   95.0% |  9,698 / 10,208 |       510 |   100% |
| Methods  |   96.6% |    3,364 / 3,481 |       117 |   100% |

The branch gate has no useful buffer: 9,698 is the minimum whole-outcome count
that reports at least 95% for the current denominator. New production branches
must arrive with coverage.

Supporting evidence:

- Report: `TestResults/coverage-branch-hardening-95-final2/reports/Summary.txt`
- CRAP analysis:
  `TestResults/coverage-branch-hardening-95-final2/crap-scores.txt`
- Methods below 95% line or branch coverage:
  `TestResults/coverage-branch-hardening-95-final2/method-gaps-under-95.json`
- Residual summary:
  `TestResults/coverage-branch-hardening-95-final2/coverage-analysis.md`

The method-gap inventory contains 478 methods below either the 95% line or 95%
branch threshold, including the 117 fully uncovered methods. That list is an
inventory, not a mandate to test every wrapper.

## Rules Of Engagement

1. **Delete before testing.** Remove zombie code, duplicate policy, and guards
   already guaranteed by validated invariants.
2. **Test behavior, not reachability.** Assertions must prove results, state
   transitions, ordering, replay continuity, or lifecycle ownership.
3. **Keep determinism first.** Preserve fixed-point math, stable ordering,
   explicit state, context ownership, and fixed-step phase boundaries.
4. **Keep hot paths lean.** Do not add LINQ, reflection, unstable hash-order
   dependencies, avoidable allocations, or speculative abstractions.
5. **Do not widen exclusions.** Generated and compiler-generated sources remain
   excluded through `tests/Gravitas.Tests/coverlet.runsettings`; hand-authored
   runtime code stays accountable.
6. **Record real discoveries.** Correctness and parity defects go to
   [`issue-tracker.md`](issue-tracker.md). Measured performance concerns go to
   [`benchmark-signal-hardening-backlog.md`](benchmark-signal-hardening-backlog.md).

Classify every uncovered family before editing:

| Classification | Required action |
| -------------- | --------------- |
| Real behavior | Add the smallest focused behavioral test. |
| Zombie code | Delete it and verify callers. |
| Duplicate policy | Collapse it into the existing shared path. |
| Impossible defensive branch | Prove the invariant, then remove or simplify it. |
| Bug or parity gap | Add a failing regression, fix the root cause, record the RCA. |
| Thin wrapper or DTO noise | Cover only when its public contract is meaningful. |
| Generated code | Leave excluded. |

## Block Discipline

Each pass owns one cohesive source block. Before implementation, record its
target files and exact missing outcomes. Finish these steps before switching
blocks:

1. Trace all callers and classify the gaps.
2. Write focused failing tests for retained behavior.
3. Delete or condense stale branches before adding new code.
4. Run the narrowest affected test classes.
5. Collect focused coverage and confirm the intended outcomes moved.
6. Run full `Release`; run `ReleaseLean` when serialization, MemoryPack shims,
   conditional compilation, or package shape is touched.
7. Obtain an independent correctness and test-quality review.
8. Update the checkpoint only after the block is complete.

If a block is genuinely blocked, record why and move to another whole block.
Do not harvest unrelated single branches to manufacture progress.

## Battle Plan

### Workstream 1: Build A Branch Buffer In Core Runtime Paths

Start with the largest branch-dense, behavior-bearing methods. These protect
motion and lifecycle correctness while creating room for later cleanup.

Priority blocks:

- 2D kinematic rotational CCD: five missing outcomes in
  `src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.Rotational.cs`.
- 3D kinematic rotational CCD: five missing outcomes in
  `src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Rotational.cs`.
- Queued 3D CCD handoffs: four missing outcomes in
  `src/Gravitas/Core/3D/GravitasPhysicsService.ContinuousCollision.cs`.
- 3D rotational CCD: four missing outcomes in
  `src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Rotational.cs`.
- Dynamic 3D CCD and response subpaths: four missing outcomes in
  `src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Dynamic.cs`, plus related
  three-outcome response helpers.

Tasks:

- [ ] Cover meaningful miss, replacement, frozen-axis, handoff-budget,
      stale-target, and relative-motion outcomes.
- [ ] Remove caller-proven or duplicate eligibility gates.
- [ ] Verify 2D/3D parity where the physical model is intentionally equivalent.
- [ ] Finish and review each CCD block separately.

### Workstream 2: Collision And Query Geometry

Close geometry branches where the result changes hit selection, distance,
normal, penetration, or contact ordering.

Priority blocks:

- 2D segment clipping: four missing outcomes in
  `src/Gravitas/CollisionHandling/Detection/2D/CollisionDetection2D.cs`.
- Cuboid/cylinder and cylinder/capsule separating axes: three missing outcomes
  per method in `src/Gravitas/CollisionHandling/Detection/3D`.
- Cone/convex and mesh/cone reducers: three-outcome families in
  `src/Gravitas/CollisionHandling/Detection/3D`.
- 3D overlap hit reducers: three missing outcomes per leading method in
  `src/Gravitas/Queries/3D/GravitasQuery3DService.Circle.cs`.
- `ConvexSweepQueryWorker.ClosestPointOnTriangleToOrigin`, the only
  under-covered CRAP hotspot: 93.9% lines, complexity 30, CRAP 30.2.

Tasks:

- [ ] Prefer public query and collision workflows over private-method tests.
- [ ] Assert deterministic closest/all-hit ordering and exact normal/distance
      behavior at edge, corner, parallel, degenerate, and tie cases.
- [ ] Delete reducer permutations that valid authored shapes cannot reach.
- [ ] Keep 2D, 3D, and mixed semantics explicit; do not gain coverage through
      accidental projection behavior.

### Workstream 3: Lifecycle, Replay, Partition, And Public Surface Residue

Classify the 117 uncovered methods and remaining low-line-coverage paths by
public reachability and deterministic value.

Priority areas:

- Collider activation, hierarchy ordinals, partition refresh, and stale-ID
  protection.
- Replay hashing, `RecordData`, load/populate behavior, and host-created shell
  continuation.
- Query overloads and all-hit buffer APIs that carry distinct public contracts.
- Diagnostic payload construction and visitor dispatch used by host adapters.
- Mesh topology validation and authored shape failure paths.

Tasks:

- [ ] Cover meaningful constructors and overloads through real host workflows.
- [ ] Delete unused wrappers and private helpers rather than invoking them for
      coverage.
- [ ] Keep runtime IDs, delegates, caches, partitions, and host bindings out of
      serialized identity.
- [ ] Run `ReleaseLean` for every serialization or conditional-compilation
      block.

### Workstream 4: Complexity And Zombie-Code Sweep

CRAP currently flags five methods above 30. Four already have 100% line
coverage and are flagged by complexity alone:

- `ColliderSettings.GetCollisionType` - complexity 81.
- `LSCollider.NotifyContact` - complexity 38.
- `LSCollider2D.NotifyContact` - complexity 38.
- `PhysicsMixedPartition.Distribute` - complexity 32.

Tasks:

- [ ] Refactor these only when a smaller explicit policy reduces real review or
      correctness risk; coverage alone is not a reason to rewrite them.
- [ ] Continue removing stale serialized fields, redundant guards, duplicate
      reducers, and unreachable fallbacks found during gap analysis.
- [ ] Mutation-check tests that justify deleting a guard or claiming a lifecycle
      invariant.

### Workstream 5: Last-Mile Closure

After the high-value families are complete, regenerate the inventory and work
the remaining gaps by source block until every hand-authored outcome is covered
or deleted.

Tasks:

- [ ] Rebuild the line, branch, method, and CRAP inventory from one explicit
      Cobertura artifact.
- [ ] Resolve every remaining uncovered item with a behavioral test, deletion,
      consolidation, or documented generated-code exclusion.
- [ ] Condense duplicate tests exposed by the final sweep.
- [ ] Run full `Release`, full `ReleaseLean`, coverage, CRAP, method-gap, and
      `git diff --check`.
- [ ] Obtain an independent final review of correctness, determinism,
      allocations, serialization, and test signal.

## Completion Gate

This plan is complete only when all of the following are true:

- 100% line coverage for hand-authored Gravitas code.
- 100% branch coverage for hand-authored Gravitas code.
- 100% method coverage for hand-authored Gravitas code.
- Full `Release` and `ReleaseLean` suites pass.
- No generated-source or hand-authored-runtime exclusions were added to inflate
  the result.
- All retained tests prove behavior and all discovered zombie code is removed.
- Final CRAP and method-gap reports are generated from the same coverage run.
- Independent final review has no unresolved findings.

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
the coverage-analysis scripts. Do not merge stale result directories.

## Condensed History

| Checkpoint | Line | Branch | Method | Tests | Outcome |
| ---------- | ---: | -----: | -----: | ----: | ------- |
| Baseline | 87.3% | 74.1% | 86.5% | 974 | Trigger-collider hardening baseline. |
| 90% gate | 97.0% | 90.0% | 96.7% | 1,756 | Release floor cleared across runtime, collision, lifecycle, constraints, and authoring. |
| 93% review | 97.7% | 93.0% | 96.6% | 1,906 | Broad residual pass completed before the final 95% push. |
| 95% gate | 98.2% | 95.0% | 96.6% | 2,014 | CCD, mixed collision, queries, response, pair lifecycle, and `SolidBody2D` hardening completed and independently reviewed. |

The completed campaigns established broad 2D, 3D, mixed, replay, query,
partition, lifecycle, serialization, and diagnostics coverage. They also fixed
real correctness defects and removed stale transient state, duplicate reducers,
unreachable CCD fallbacks, and dead serialization branches. Detailed historical
checkpoints remain in the prior coverage artifacts; they are intentionally not
duplicated here.
