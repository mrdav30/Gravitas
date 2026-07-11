# Coverage Hardening Plan

**Date:** 2026-07-11  
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
`TestResults/coverage-circle-geometry-full/e0f5020c-824b-4d66-ac8d-0035474368d3/coverage.cobertura.xml`.

| Metric   | Current | Covered / Total | Remaining | Target |
| -------- | ------: | --------------: | --------: | -----: |
| Lines    |   99.2% | 25,586 / 25,801 |       215 |   100% |
| Branches |   96.6% |  9,799 / 10,142 |       343 |   100% |
| Methods  |   98.8% |    3,396 / 3,437 |        41 |   100% |

The completed blocks continue to reduce both uncovered outcomes and stale
production surface while keeping every new production method covered.

Supporting evidence:

- Previous 95% gate:
  `TestResults/coverage-branch-hardening-95-final2/05bb0b97-6100-424b-9d1e-8ae22eb73d4d/coverage.cobertura.xml`
- Previous report: `TestResults/coverage-branch-hardening-95-final2/reports/Summary.txt`
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

- [x] 2D kinematic rotational CCD: five missing outcomes closed with five
  behavioral tests; focused method coverage is 100% line/branch, the affected
  class passes 76/76, full `Release` passes 2,019/2,019, and independent review
  is resolved. Source:
  `src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.Rotational.cs`.
- [x] 3D kinematic rotational CCD: five missing outcomes closed with five
  behavioral tests; the target decisions report 8/8 and 2/2 focused branch
  outcomes, the affected class passes 90/90, full `Release` passes 2,024/2,024,
  and independent review approved. Source:
  `src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Rotational.cs`.
- [x] Queued CCD handoffs: the original four 3D outcomes and matching 2D
  lifecycle paths are closed. Queue ownership now preserves body identity,
  exhausted budgets and resets discard pending body state, affected methods
  report 100% focused line/branch coverage, both CCD classes pass 178/178,
  full `Release` passes 2,036/2,036, and independent review approved. Source:
  `src/Gravitas/Core/3D/GravitasPhysicsService.ContinuousCollision.cs` and its
  2D counterpart.
- [x] Dynamic 3D rotational CCD: four missing outcomes closed with epsilon
  angular-distance, epsilon-proxy-with-offset-inertia, sub-epsilon arc, and
  no-static-candidate tests. Target decisions report 2/2, 8/8, and 2/2
  focused outcomes, the complete CCD class passes 100/100, full `Release`
  passes 2,040/2,040, and independent review is resolved. Source:
  `src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Rotational.cs`.
- [x] Dynamic 3D TOI loop: the four proxy, quantized-tail, frame-end, and
  zero-time outcomes are closed and every branch in the leading resolver is
  covered. Review also corrected finite-heavy-body response across 2D, 3D,
  mixed, dynamic, and kinematic paths, added max-mass saturation and explicit
  near-singular mobility policy coverage, and retained bounded zero-time
  stagnation handling for unsupported mobility. The CCD class passes 108/108,
  full `Release` passes 2,052/2,052, and independent review approved. Source:
  `src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Dynamic.cs` and
  dimensional counterparts.
- [x] CCD response/helper residue: caller-proven normal, closing, and aggregate-
  mass guards were removed; finite/max-mass and near-singular mobility behavior
  is explicit and covered across pure and mixed paths. The complete 3D dynamic
  CCD file reports 100% line/branch coverage, focused pure 3D and mixed classes
  pass 108/108 and 147/147, full `Release` passes 2,052/2,052, and independent
  review approved.
- [x] Kinematic translation CCD residue: seven deterministic workflows cover
  tiny positive proxies, live candidates invalidated by same-frame filter
  changes, broad-corner proxy misses, and near-singular partially frozen target
  mobility across pure 3D and mixed 2D handoffs. Successful relative/exact
  sweeps already prove a closing normal, so four unreachable normal-flip and
  non-closing branches plus one duplicate inverse-mass guard were deleted. The
  complete 3D kinematic CCD file reports 100% line/branch/method coverage,
  focused suites pass 262/262, full coverage-enabled `Release` passes
  2,116/2,116, `ReleaseLean` builds both targets, and independent review
  approved.

Tasks:

- [x] Cover meaningful miss, replacement, frozen-axis, handoff-budget,
      stale-target, and relative-motion outcomes.
- [x] Remove caller-proven or duplicate eligibility gates.
- [x] Verify 2D/3D parity where the physical model is intentionally equivalent.
- [x] Finish and review each CCD block separately.

### Workstream 2: Collision And Query Geometry

Close geometry branches where the result changes hit selection, distance,
normal, penetration, or contact ordering.

Priority blocks:

- [x] 2D segment clipping: removed four unreachable single-point reclip
  outcomes by proving each private two-point clip returns only 0 or 2 points.
  Public collision/manifold suites pass 78/78, `ClipSegment` reports 100%
  focused line/branch coverage, full `Release` passes 2,052/2,052, and
  independent review approved. Source:
  `src/Gravitas/CollisionHandling/Detection/2D/CollisionDetection2D.cs`.
- [x] Cuboid/cylinder and cylinder/capsule separating axes: six public shape-
  pair fixtures isolate cylinder, capsule, cross, edge-cross, and closest-
  feature exits. Both SAT methods report 100% focused line/branch coverage,
  the shape-pair class passes 56/56, full `Release` passes 2,058/2,058, and
  independent review approved. Source:
  `src/Gravitas/CollisionHandling/Detection/3D/CollisionDetection.Cylinder.cs`.
- [x] Cone/convex and mesh/cone reducers: removed impossible dispatcher,
  post-normal-resolution, and validated-mesh degeneracy branches; covered
  concave rejection, convex miss, both pair orientations, triangle-reducer
  outcomes, and deterministic coincident-center fallback. Independent review
  exposed epsilon-tolerant GJK contacts whose signed support depth is one raw
  unit negative, so both zero-depth clamps were retained and pinned with exact
  public-workflow regressions. All four target methods report 100% focused
  line/branch coverage, the shape-pair class passes 61/61, full `Release`
  passes 2,063/2,063, and re-review approved. Source:
  `src/Gravitas/CollisionHandling/Detection/3D/CollisionDetection.Cone.cs`.
- [x] 3D overlap hit reducers: public fixtures cover broad-sphere and exact-
  surface misses, empty all-hit diagnostics, stale partition IDs in both
  planar-circle and static-sphere resolution, and deterministic rejection of
  farther closest/directional candidates. The complete source file reports
  100% focused line/branch coverage, the focused class passes 15/15, the full
  coverage-enabled `Release` suite passes 2,066/2,066, and independent review
  approved after diagnostic and fixture-invariant assertions were strengthened.
  Source: `src/Gravitas/Queries/3D/GravitasQuery3DService.Circle.cs`.
- [x] `ConvexSweepQueryWorker.ClosestPointOnTriangleToOrigin`: exact fixed-
  point fixtures now validate all three vertex Voronoi regions and reconstruct
  the selected endpoint from barycentric weights. The standard GJK arithmetic
  and tetrahedron face order remain unchanged; a narrow internal seam avoids
  reflection and brittle end-to-end arrangements without expanding public API.
  The target method reports 100% focused line/branch coverage, the combined
  sweep/reducer suites pass 90/90, full `Release` passes 2,069/2,069, and
  independent review approved. Source:
  `src/Gravitas/Queries/3D/Sweeps/ConvexSweepQueryWorker.cs`.
- [x] Mesh triangle manifolds: five public collision fixtures cover the
  coplanar sphere-normal fallback, capsule segment/edge closest-feature
  reduction, cuboid and cylinder plane separation with overlapping AABBs, and
  both mesh face-normal exit orders. Proven-impossible negative-overlap clamps
  and the caller-impossible reverse contact-order branch were deleted. The
  complete generator reports 100% line/branch coverage, the focused shape-pair
  class passes 66/66, full `Release` passes 2,096/2,096, and independent review
  approved. Source:
  `src/Gravitas/CollisionHandling/Detection/3D/Mesh/MeshTriangleContactGenerator.cs`.

Tasks:

- [x] Prefer public query and collision workflows over private-method tests;
      use a narrow internal seam only for the isolated pure triangle reducer.
- [x] Assert deterministic closest/all-hit ordering and exact normal/distance
      behavior at edge, corner, parallel, degenerate, and tie cases.
- [x] Delete reducer permutations that valid authored shapes cannot reach.
- [x] Keep 2D, 3D, and mixed semantics explicit; do not gain coverage through
      accidental projection behavior.

### Workstream 3: Lifecycle, Replay, Partition, And Public Surface Residue

Classify the 77 uncovered methods and remaining low-line-coverage paths by
public reachability and deterministic value.

Completed blocks:

- [x] Diagnostic event construction: deleted the unused pre-joint internal
  constructor, leaving the joint-aware constructor as the sole source of truth.
  This removed 25 uncovered lines and one uncovered method; diagnostic suites
  pass 29/29, full `Release` passes 2,069/2,069, and independent review found no
  serialization, reflection, source-generation, or caller dependency.
- [x] 2D layer-matrix fallback stability: an explicit pair-eligibility test now
  covers both out-of-range short-circuit positions and normal in-range lookup,
  preventing full-run order from deciding whether the method reports 3/4 or
  4/4 branch outcomes. The method reports 100% focused line/branch coverage,
  full `Release` passes 2,070/2,070, and independent review approved.
- [x] 3D `SolidBody` surface and lifecycle: deleted 15 unused methods plus dead
  spawned-position and time-scaled-acceleration state, folded one-time setup
  into the validated constructor, and covered the retained acceleration,
  orientation, visualization-buffer, interaction-speed, sleep-threshold, and
  idempotent-deactivation contracts. `SolidBody.cs` reports 100% line/branch
  coverage, serialization/replay tests pass 31/31, focused lifecycle tests pass
  22/22, full coverage-enabled `Release` passes 2,073/2,073, `ReleaseLean`
  builds both targets, and independent deletion and contract reviews approved.
- [x] 3D raycast and registered-source sweep surface: direct behavioral tests
  cover capsule, cylinder, cone, and compound all-hit entry points, equal-point
  batch sweeps, stale partition IDs and mobility, out-of-order intersection
  reduction, malformed extensible colliders, and context ownership. Duplicate
  endpoint visits, dead closest-distance plumbing, and impossible closest-path
  static-only guards were removed. Rollover testing and review exposed that
  reused query versions could suppress live colliders after `uint` wrap or a
  standalone public reset; both cache families now invalidate the compact live
  registry before version reuse. The complete raycast source reports 100%
  line/branch coverage, focused query suites pass 158/158, full `Release`
  passes 2,085/2,085, and independent review approved after the reset defect
  was fixed and re-reviewed.
- [x] 3D collider surface and active/load lifecycle: deleted nine unused
  registry, partition-cache, inertia, and interface wrappers; internalized
  query stamps; retained and covered the public subclass radius/initialization
  hooks, unbound access failures, hierarchy state, and reverse mixed filtering.
  Review exposed stale partition/query visibility from flag-only activation,
  missing repartition after inactive-to-active loads, invalid ID `-1`
  repartition of fully deactivated shells, and false errors on repeated inactive
  loads. `IsActive` now owns primary/mixed partition transitions and load paths
  distinguish unbound, unregistered, inactive, and registered-active shells.
  `LSCollider.cs` reports 100% line/branch coverage, full `Release` passes
  2,091/2,091, `ReleaseLean` builds both targets, and independent review
  approved after all findings were resolved.
- [x] Compound authored-part surface: retained and behaviorally covered the 13
  previously untouched 2D/3D constructor, material, transform, scale, and mesh
  policy overloads. Deleted all three duplicate `CompoundColliderPart2D.AABox`
  aliases plus the matching zero-caller `ColliderShapeDefinition2D.AABox`
  alias, migrating existing tests to canonical `AABBox`. Both compound-part
  files report 100% line/branch/method coverage, focused suites pass 19/19,
  full `Release` passes 2,096/2,096, `ReleaseLean` builds both targets, and
  independent review approved after non-identity rotation and mesh-policy
  assertions were strengthened.
- [x] 2D collider surface, teardown, load, and query-cache parity: deleted the
  unused registry/inertia wrappers and dead compound-owner chain, internalized
  query stamps, simplified caller-proven ownership guards, and covered retained
  hierarchy/default-shape behavior. Red regressions exposed unbound compound
  rebuild-before-context failure, inactive registered-collider leaks, broken
  direct teardown of body-owned colliders, and ray/overlap false negatives
  after version wrap or public reset. The collider and both query service files
  report 100% line/branch/method coverage, focused suites pass, full `Release`
  passes 2,104/2,104, `ReleaseLean` builds both targets, and independent review
  approved after the teardown findings were resolved.
- [x] Inactive `SolidBody2D` load ownership: JSON and MemoryPack regressions
  proved inactive snapshots left registered body/collider IDs behind and later
  active snapshots could invent activity on the resulting unregistered shell.
  Teardown is now registration-aware, inactive load reconciles runtime ownership
  while bindings remain valid, and active snapshot state is accepted only for
  already registered shells; explicit `Initialize()` owns re-registration.
  Both body source files report 100% line/branch/method coverage, full `Release`
  passes 2,106/2,106, `ReleaseLean` builds both targets, and independent review
  approved.
- [x] Physics mesh authored/topology surface: deleted three zero-caller public
  wrappers, including the mutable `FaceAreas` array exposure, and removed the
  impossible zero-total-area fallback guaranteed by constructor validation.
  Public tests cover empty topology, singular authored transforms, invalid
  inertia policy, and negative-Y deterministic support-tree traversal. The
  complete `PhysicsMesh` class reports 100% line/branch/method coverage, the
  focused suite passes 28/28, full `Release` passes 2,108/2,108, `ReleaseLean`
  builds both targets, and independent review approved.
- [x] Cuboid geometry and authored surface: a public regression proved the
  frontal-area selector returned the wrong face for every principal axis and
  only one face for diagonal motion. It now computes the exact deterministic
  orthographic projection in fixed-point world space, with the established
  zero-direction fallback. Deleted the unused centroid/cache tables, duplicate
  cuboid-state enum, stale edge helpers and overridable build hooks, and
  external mutable-array exposure; live geometry remains internal to collision
  and query consumers. `LSCuboidCollider.cs` reports 100%
  line/branch/method coverage, full `Release` passes 2,109/2,109,
  `ReleaseLean` builds both targets, and independent review approved.
- [x] 3D grounding lifecycle and probe policy: deterministic workflows now
  cover skip-window expiry plus the stationary-probe throttle boundary,
  no-clear/no-immediate mode transitions, inactive ownership changes,
  transition callbacks, platform and last-grounded-position state, explicit
  swept-sphere radius, cone ray fallback, and the sub-threshold compound-radius
  policy. Deleted the zero-caller `HitPlatform` setter and two nullable paths
  that validated query hits cannot produce. `SolidBody.Grounding.cs` reports
  100% line/branch/method coverage, focused suites pass 22/22, full
  coverage-enabled `Release` passes 2,120/2,120, `ReleaseLean` builds both
  targets, and independent review approved.
- [x] Dimensional teardown, binding reuse, and inactive 3D body loads: physics
  services now solely own constraints, pairs/hierarchy, primary/mixed
  partitions, refresh registration, and collider registry removal. Public
  body-owned collider teardown delegates atomically to its body; partition
  clears normalize state and are idempotent without false errors. JSON and
  MemoryPack regressions reconcile inactive registered 3D shells and prevent
  active payloads from inventing registration. Three independent P1 review
  rounds additionally closed stale body/host bindings, stale-body teardown of
  rebound colliders, and registered or post-reset foreign-binding theft during
  reinitialization in both dimensions. The touched 2D/3D body, 3D collider, and
  3D body serialization files report 100% line/branch/method coverage, focused
  lifecycle/reset/serialization suites pass 222/222, full coverage-enabled
  `Release` passes 2,123/2,123, `ReleaseLean` builds both targets, and final
  review approved.
- [x] 3D partition service and bodyless binding reuse: repeated public
  bodyless initialization could overwrite collider IDs and orphan the prior
  registry/partition entry, so both dimensions now reject registered or
  foreign-bound shells before mutation while permitting same-agent reuse after
  context reset. The 3D service also internalizes partition coordinates,
  removes its duplicate voxel hash pass, dead reset alias, and caller-proven
  guards, and handles removed grids, replaced grid slots, missing voxels, and
  detached partitions without error-log spam. `GravitasCollisionService.cs`
  reports 100% line/branch/method coverage, full `Release` passes 2,129/2,129,
  `ReleaseLean` builds both targets without warnings, and independent review
  approved after its findings were resolved.
- [x] 2D partition service parity: deleted the matching dead alias, duplicate
  voxel hash pass, unreachable attach rollback, zero-caller mobility wrapper,
  caller-proven ID checks, and redundant static-style query revalidation.
  Behavioral coverage retains stale registry-ID and registered-inactive query
  filtering, four directional bounds misses, deferred inactive/deactivated
  refresh, primary partition pool reuse, and removed/replaced/missing GridForge
  state without error-log spam. `GravitasCollision2DService.cs` reports 100%
  line/branch/method coverage, focused tests pass 37/37, full `Release` passes
  2,132/2,132, `ReleaseLean` builds without warnings, and independent review
  approved with no findings.
- [x] 3D compound collision detection: public workflows now cover all-parts-
  separated compound/primitive and compound/compound pairs, broad-bounds-only
  corner misses, and higher-priority primitive swaps with owner-ordered points,
  normals, and part materials. Deleted duplicate manifold predicates and an
  impossible owner-reference path already guaranteed by compound-first pair
  ordering. `CollisionDetection.Compound.cs` reports 100%
  line/branch/method coverage, focused tests pass 10/10, full `Release` passes
  2,136/2,136, and independent review approved with no actionable findings.
- [x] Mixed projected-circle geometry: a rotated multi-triangle mesh regression
  proves conservative local triangle candidates wholly below the finite slab
  are clipped before hit reduction. One compound workflow covers an out-of-
  slab cuboid projection plus vertical cylinders on both interval-separation
  sides. Deleted an impossible clip-closing duplicate guard, a caller-proven
  convex-hull count guard, and a zero-caller 3D segment helper.
  `GravitasQueryMixedService.CircleGeometry.cs` reports 100%
  line/branch/method coverage, focused mixed query tests pass 152/152, full
  `Release` passes 2,138/2,138, `ReleaseLean` builds both targets without
  warnings, and independent review approved with no actionable findings.

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
