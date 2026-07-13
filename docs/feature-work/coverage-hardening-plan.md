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
`TestResults/coverage-ccd-helpers-task52-authoritative-root-comparable/04c47dfc-f6b6-409e-8869-f56b269e1c28/coverage.cobertura.xml`.

| Metric | Current | Covered / Total | Remaining | Target |
| ------ | ------: | --------------: | --------: | -----: |
| Lines | 99.77% | 26,916 / 26,978 | 62 | 100% |
| Branches | 99.45% | 10,343 / 10,400 | 57 | 100% |
| Methods | 99.38% | 3,515 / 3,537 | 22 | 100% |

The authoritative full coverage-enabled `Release` suite passes 2,490/2,490 tests, and
`ReleaseLean` builds both targets without warnings. Branch coverage is now the
primary constraint, but the remaining line and method gaps must close from the
same final artifact.

Task 52's authoritative artifact reports 100% line, branch, and method coverage
for the 3D body CCD helper and shared target policy. Final 3D target admission
now delegates to the canonical collision-pair gate, closing a real linked-joint
suppression defect while preserving collider lifecycle, hierarchy, layer, and
authored-filter rules.

### Immediate Completed Block

The 3D CCD helper boundary is resolved and independently reviewed. Context
`Inherit` fallback, zero-scale bounds proxies, moving-away overlap rejection,
and dynamic plus kinematic linked-joint suppression are mutation-sensitive.
Duplicate proxy thresholding and normalized-normal checks were removed after
caller proof, and the shared policy now receives one canonical physical-pair
decision instead of a drifting subset of collision rules.

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
   Rebuild restored production source first after every temporary mutation and
   reject artifacts whose module hash or IL offsets do not match that build.
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
| 1 | `CollisionHandling/Detection/3D/Sat/AxisProjectionHelper.cs` | 5 | 2 | 1 | Capsule/cuboid axes, sphere projection, and degenerate normals. |
| 2 | `Constraints/3D/RagdollRuntime3D.cs` | 3 | 2 | 1 | Replay load shape, link ownership, and stable reconstruction. |
| 3 | `Core/2D/GravitasPhysics2DService.SupportTypes.cs` | 3 | 2 | 0 | Stable support ordering and lifecycle-proven ownership. |
| 4 | Remaining two-branch collision, constraint, query, and CCD blocks | 0-2 | 2 each | 0 | Re-rank from the next authoritative artifact; finish one cohesive source block at a time. |

### Phase 1: Core Runtime And Service Ownership

- [x] Complete and independently review
      `Constraints/3D/GravitasConstraint3DService.cs`.
- [x] Complete and independently review
      `Core/3D/GravitasPhysicsService.cs`.
- [x] Complete `Core/2D/GravitasPhysics2DService.cs` and verify intentional
      2D/3D lifecycle parity.
- [x] Close and independently review residual 2D constraint-service ownership
      through larger-ID suppression cleanup; remove impossible collider-null
      checks and duplicate post-validation ragdoll materialization.
- [x] Close and independently review `Joint2D` explicit and legacy distance
      loads through suppression, enabled-count, frame, and solver-cache state;
      remove impossible nullable collider replay IDs.
- [x] Close and independently review the 2D pair and response lifecycle,
      including callback mutation, shell reuse, trigger ordering, pooling, and
      dense cleanup capacity.
- [x] Close and independently review residual 2D solver outcomes through both
      trigger orderings, frozen contact-axis mobility, frictionless contacts,
      and near-zero tangential mobility; replace redundant non-negative
      friction-limit predicates with direct deterministic products.
- [x] Close and independently review 3D response island admission, sleeping
      suppression, sparse joint traversal, anchored participants, rootless
      contacts, and single-contact dispatch.
- [x] Close and independently review the residual 3D response block by fixing
      the vacuous coincident-center setup and asserting zero-normal rejection
      through unchanged position, velocity, and warm-start state.
- [x] Close and independently review 3D response support ordering through
      canonical endpoint keys, contact-before-joint ordering, ascending
      duplicate-joint IDs, and non-null deferred-pair ownership; remove
      redundant nullable and dynamic-ID predicates after lifecycle proof.
- [x] Close and independently review the 3D pair lifecycle, including callback
      mutation, per-side admission, exception retry, stale queue snapshots,
      exact exit order, and pooled-lifetime reuse.
- [x] Close and independently review the mixed pair/response family, including
      stale queued candidates, pooled lifetimes, nested removal snapshots,
      sleeping/rootless response admission, per-side callbacks, and rebound
      suppression.
- [x] Close residual world-context outcomes through real owned/attached
      lifetime, phase-routing, reset, and disposal workflows.
- [x] Close and independently review retained partition retirement through
      stale indices, foreign and occupied partitions, missing attachments, and
      partial removal-callback failure recovery; remove the caller-impossible
      concurrent `TryRemovePartition` failure branch.
- [x] Close and independently review diagnostic sink ownership through a
      successful context-owned ground probe; remove redundant summary
      enablement, mixed-contact hit, and implicit 2D dimension branches proven
      by synchronous callers and constructible metadata invariants.
- [x] Close and independently review 2D partition spurious-removal diagnostics,
      awake-membership no-op state, and retained lifecycle invariants; delete
      the obsolete GridForge `OnChange` method.
- [x] Close and independently review the matching 3D partition contract through
      diagnostic spurious removals, unattached awake no-op state, and unchanged
      retained ownership; delete the obsolete GridForge `OnChange` method.
- [x] Close and independently review residual 3D body-motion outcomes through
      initialize, simulate, late-simulate, reset, deactivate, shell reuse,
      grounded friction, anisotropic gyro, and queued CCD workflows.
- [x] Close and independently review the context-owned coroutine subsystem:
      phase-entry snapshots, sparse-slot reuse, reset/deactivate/dispose,
      callback-safe cancellation, dual-failure cleanup, instruction ownership,
      reference/high-water release, pure clock waits, and frame wrap.

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
- [x] Close and independently review convex sweep termination through a real
      saturated fixed-point budget exit and successful worker reuse; remove
      caller-impossible triangle source bounds and offset behavior.
- [x] Close `LSPolygonCollider2D.cs` through authored/load/degenerate-scale
      workflows and translation-stable centroid/inertia math for both windings,
      compound offsets/scales, and arbitrary reference points.
- [x] Close and independently review the 2D compound-collider family through
      authored-first geometry ties, conservative radius selection, valid and
      invalid AABB loads, allocation-free convex vertex ownership, and
      fixed-point zero-area center-of-mass and inertia limits.
- [x] Close and independently review the 3D capsule contract through exact
      rotation-aware frontal area, fixed-point projection overshoot, solid
      hemisphere inertia, sphere/thin-rod limits, cap-normal underflow, and
      deletion of the invariant-impossible closest-point guard.
- [x] Close and independently review the 3D compound contract through solid
      volume/shell mass measures, exact residual assignment, owner-local COM
      and tensor transforms, owner-offset geometry, conservative query radius,
      authored-first ties, and all-zero fixed-point fallback.
- [x] Repair and independently review the mesh scale/shell inertia boundary exposed by the compound
      review, keep immutable topology/BVH ownership intact, and independently
      review the resulting collision, query, normal, area, COM, and tensor
      behavior before resuming unrelated gaps.
- [x] Close and independently review 2D overlap queries through bounds-admitted
      diagonal exact misses, closest/all circle and AABB paths, default-overload
      delegation, and caller-buffer clearing.
- [x] Close and independently review 3D batch queries through zero-length
      all-ray ranges, exact source/displacement preparation reuse, changed-
      displacement recomputation, and overlap miss/default behavior.
- [x] Close and independently review 2D collider replay hierarchy hashing
      through live 2D/3D parents, clear-to-baseline behavior, canonical dimension
      tags, and deletion of impossible registry/dimension guards; fix first-call
      mixed hierarchy hash drift by preparing both registries before any
      subsystem contribution.
- [x] Close and independently review the symmetric 3D collider replay hierarchy
      contract through live 3D/2D parents, clear-to-baseline behavior, equal-
      ordinal dimension tags, and churned raw-ID independence.
- [x] Close and independently review 3D joint row admission and correct the
      cone-twist model: preserve allowed swing, handle antiparallel swing with
      deterministic fallback axes, use true signed swing-twist decomposition,
      and canonicalize quaternion signs on a half-open angle interval.
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
- [x] Close and independently review mixed circle-against-3D reducers through
      compound vertical separation, end-cap planar misses, nearest mesh
      distance, and both BVH-authored tie outcomes.
- [x] Close and independently review mesh collider area, frontal area,
      disconnected-BVH fallbacks, and all-zero-scale mass properties; remove
      the impossible constructor-owned null mesh arm.
- [ ] Reassess `AxisProjectionHelper` and remaining geometry after each fresh
      artifact.
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
- [x] Complete and independently review the 3D CCD hit counterpart through
      epsilon-scale fallback normals, exact non-closing rejection, complete
      transform restoration, newly filtered mixed targets, and deletion of
      duplicate sphere dispatch and impossible relative-length guards.
- [x] Complete and independently review the remaining 2D CCD dynamic-response
      paths through near-singular source/target mobility, positive sub-epsilon
      mass sums, exact mixed target non-handoff state, and duplicate zero-sum
      guard deletion aligned with the 3D solver.
- [x] Close and independently review 2D CCD service candidate and handoff
      ownership through duplicate queue updates, latest-state consumption, and
      shared mixed-buffer clearing; remove the lifecycle-impossible inactive
      body guard without involving GridForge.
- [x] Close and independently review 2D rotational CCD through epsilon proxies,
      sub-epsilon arcs, and invalid physical candidates; count accepted dynamic
      rotational impacts in both dimensions and normalize kinematic 2D yaw to
      the deterministic shortest arc across the signed boundary.
- [x] Close and independently review 2D motion through zero and sub-resolution
      effective angular inputs plus immediate zero-threshold sleep; align sleep
      disabling and threshold validation with 3D lifecycle semantics and track
      the separate frame-rate-dependent 3D impulse-units decision.
- [x] Close and independently review residual 2D grounding through automatic,
      manual, cached-support, callback-reentrancy, shell-reuse, query, and
      deterministic contact-candidate workflows.
- [x] Close and independently review the 3D CCD helper and target policy:
      context-default inheritance, exact bounds proxies, closing-hit admission,
      and canonical dynamic/static/kinematic pair filtering including linked-
      joint suppression.
- [ ] Verify dimensional parity only where the physical models are intended to
      match; keep 2D, 3D, and mixed behavior explicit elsewhere.

Exit condition: every retained branch protects a real motion, time-of-impact,
support, constraint, budget, or mobility invariant, and every deleted branch
has a caller-proven impossibility argument.

### Phase 4: Method And Public-Surface Closure

- [ ] Complete the four remaining `ContactManifold` methods through meaningful
      construction, mutation, and reduction contracts or delete unused surface.
- [ ] Regenerate the uncovered-method inventory from the latest full artifact
      and classify all 30 current method gaps.
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
| 2D dynamic CCD mobility | 99.54% | 98.36% | 99.02% | 2,328 | Dynamic 2D/mixed response reached 100%; near-singular constrained mobility is rejected before division, positive sub-epsilon sums remain valid, and duplicate zero-sum guards were removed after mutation-sensitive review. |
| 3D CCD hit reduction | 99.55% | 98.40% | 99.02% | 2,331 | 3D hit reduction reached 100%; epsilon fallback normals, non-closing exact hits, restored transforms, and post-index filtering were covered; duplicate sphere dispatch and an impossible relative-length guard were removed. |
| Retained partition lifecycle | 99.56% | 98.45% | 99.02% | 2,334 | Retirement reached 100%; stale indices, foreign and occupied partitions, missing attachments, and partial removal-callback failures were covered; impossible concurrent removal bookkeeping was simplified after independent review. |
| 2D response closure | 99.57% | 98.49% | 99.02% | 2,337 | Response reached 100%; both trigger orderings, frozen contact-axis rejection, frictionless preservation, and near-zero tangent mobility were covered; algebraically redundant friction-limit branches were removed after mutation-sensitive review. |
| Diagnostic sink closure | 99.57% | 98.54% | 99.02% | 2,338 | Diagnostics reached 100%; successful ground-probe identity and geometry were covered; redundant summary enablement, mixed-contact hit, and implicit 2D inference branches were removed after complete call-graph review. |
| Mixed circle reducer closure | 99.59% | 98.58% | 99.02% | 2,340 | Reducers reached 100%; vertical and planar capsule misses, nearest mesh selection, and both BVH-authored tie outcomes were covered. A stale mutation-built artifact was rejected and replaced with clean focused and full evidence. |
| 3D response support ordering | 99.60% | 98.62% | 99.02% | 2,341 | Support types reached 100%; endpoint, kind, and joint-ID sorting now have mutation-sensitive diagnostic order; deferred-pair nullability and duplicate dynamic-ID guards were removed after lifecycle review. |
| Mesh collider closure | 99.60% | 98.66% | 99.05% | 2,343 | Mesh collider reached 100%; public area/frontal area, disconnected-neighborhood bounds fallbacks, and all zero-scale inertia axes were covered; the impossible null mesh arm was removed. |
| 2D constraint service closure | 99.61% | 98.69% | 99.05% | 2,344 | Constraint service reached 100%; larger-ID suppression cleanup was covered; impossible collider-null checks and duplicate post-validation ragdoll resolution/allocation were removed. |
| 2D joint load closure | 99.61% | 98.73% | 99.05% | 2,346 | Joint2D reached 100%; explicit and legacy distance loads now prove suppression, enabled-count, frame, and cache synchronization; impossible nullable collider replay IDs were removed. |
| 2D partition closure | 99.61% | 98.78% | 99.08% | 2,347 | PhysicsPartition2D reached 100%; diagnostic spurious removals and awake no-op lifecycle were covered; obsolete GridForge `OnChange` surface was removed. |
| 3D partition closure | 99.62% | 98.82% | 99.11% | 2,348 | PhysicsPartition reached 100%; diagnostic spurious removals and unattached awake no-op lifecycle were covered; obsolete GridForge `OnChange` surface was removed without involving the pooled spawn-token defect. |
| 2D overlap query closure | 99.63% | 98.85% | 99.13% | 2,349 | The overlap service reached 100%; one bounds-admitted diagonal exact miss covers closest/all circle and AABB rejection, default-overload delegation, and caller-buffer clearing with four killed mutations. |
| 3D convex sweep closure | 99.64% | 98.88% | 99.13% | 2,350 | ConvexSweepQueryWorker reached 100%; saturated extreme inputs prove deterministic budget termination and clean reuse, while source-only call-graph proof removed impossible triangle bounds and offset paths. |
| 3D batch query closure | 99.65% | 98.91% | 99.13% | 2,350 | Batch queries reached 100%; zero-ray ranges, exact preparation reuse, changed-displacement recomputation, overlap miss/default behavior, and redundant result assignment were independently reviewed. |
| 2D replay hierarchy closure | 99.66% | 98.94% | 99.13% | 2,352 | The 2D collider hash reached 100%; live parent dimensions and clear-to-baseline behavior are exact, impossible guards were removed, and a P1 first-call mixed-hierarchy replay drift was fixed by preparing both dimensional registries up front. |
| 3D replay hierarchy closure | 99.67% | 98.97% | 99.13% | 2,353 | The 3D collider hash reached 100%; equal-ordinal parent dimensions, clear-to-baseline state, and compact/churned raw-ID independence close the symmetric hierarchy contract after impossible guards were removed. |
| 3D joint solver closure | 99.68% | 99.00% | 99.14% | 2,362 | JointSolver3D reached 100%; unrestricted rows are exact, cone-twist no longer over-constrains allowed swing, antiparallel swing is deterministic, and signed swing-twist limits are invariant across quaternion signs including exact pi. |
| 2D CCD service closure | 99.68% | 99.02% | 99.14% | 2,364 | The 2D CCD service reached 100%; duplicate handoffs queue once and consume the latest state, shared planar/mixed candidate storage clears across disabled mixed queries, and the lifecycle-impossible inactive registry guard was removed after independent review. |
| 2D rotational CCD closure | 99.69% | 99.05% | 99.14% | 2,369 | The 2D rotational block reached 100%; real epsilon-proxy, sub-epsilon-arc, and post-broad-phase filter outcomes are covered; dynamic impacts now count in both dimensions; kinematic signed-boundary yaw uses the shortest arc; the endpoint-only sampling limitation is tracked with an exact reproducer. |
| 2D motion closure | 99.69% | 99.08% | 99.14% | 2,376 | The motion block reached 100%; zero and quantized-away angular inputs preserve sleep, zero-frame thresholds sleep immediately, 2D sleep disable/validation matches 3D, serialization/replay remain stable, and the separate 3D impulse-unit defect is tracked. |
| Coroutine lifecycle closure | 99.70% | 99.12% | 99.14% | 2,409 | The full coroutine subsystem reached 100%; sparse-slot scheduling, callback-safe cancellation, reset/dispose ownership, exception aggregation, instruction identity/context, snapshot and bucket cleanup, clock-observing waits, and frame wrap are deterministic and independently approved. |
| 2D compound collider closure | 99.71% | 99.16% | 99.25% | 2,417 | The touched 2D collider family reached 100%; fake vertices were removed, authored-first geometry selection stayed stable, valid/invalid AABB loads synchronize exactly, and quantized-zero compound parts retain deterministic center-of-mass and parallel-axis inertia. The analogous 3D capsule zero-volume defect is carried directly into the next block. |
| 3D capsule closure | 99.72% | 99.21% | 99.28% | 2,422 | The capsule reached 100%; drag area now follows the exact rotation-aware silhouette, solid hemisphere centroids contribute the missing transverse inertia term, quantized zero volume uses a shifted thin-rod limit, fixed-point cap-normal fallbacks remain covered, and one impossible closest-point guard was removed. |
| 3D compound closure | 99.73% | 99.24% | 99.34% | 2,433 | The compound family reached 100%; mass now uses explicit solid volume or shell policy, fixed-point residual mass is exact, part COM/tensors and owner offsets share one frame, remote parts remain query-visible, false aggregate `ScaledSize` was removed, and the discovered mesh-scale and point-transform boundaries are tracked explicitly. |
| Mesh transform and mass closure | 99.73% | 99.25% | 99.35% | 2,461 | Mesh scale now reaches bounds, normals, areas, queries, collision, closed-volume covariance, and physical thin-shell inertia through checked atomic lifecycle commits. Immutable topology/BVH caches survive transforms, closed mass is cached per committed scale, SAT edge separation is exact, and the touched mesh/SAT files reached 100%. |
| Diagnostic draw closure | 99.74% | 99.28% | 99.35% | 2,463 | Diagnostic draw reached 100%; unsupported custom shapes preserve sequence ownership, zero authored joint rotations emit unit axes through normalized identity fallback, and redundant zero-axis and zero-vertex guards were removed after independent review. |
| Physics material closure | 99.75% | 99.31% | 99.35% | 2,472 | PhysicsMaterial reached 100%; duplicate validation was collapsed without changing parameter contracts, average is overflow-safe and ties-to-even, geometric mean preserves positive identity and extreme coefficients, and the relevant default-material response benchmark remains allocation-free without a credible regression. |
| Authored shape-definition closure | 99.75% | 99.36% | 99.35% | 2,474 | The 3D and 2D authored definition families reached 100%; dispatch fallbacks now own invalid/default errors, impossible private payload-null guards were deleted, mesh index and planar size boundaries are exact, and omitted versus explicit default material semantics remain distinct. |
| Hierarchy ownership closure | 99.76% | 99.38% | 99.38% | 2,476 | Shared hierarchy state reached 100%; the unused ParentId duplicate was removed, empty cleanup is idempotent, and reparent/clear ownership now releases the retained exact top parent instead of risking stale or reused-ID registry aliases. |
| Shared segment-box clipping closure | 99.77% | 99.42% | 99.38% | 2,485 | Mixed fallback, ray AABB/OBB, and swept-sphere cuboid clipping reached 100%; exact-zero parallel policy preserves one-raw endpoint contacts, true ray exits remain exact, and three drifting clip implementations collapsed into SweepBoundsUtility with 77 net production lines deleted. |
| 3D CCD helper and pair-policy closure | 99.77% | 99.45% | 99.38% | 2,490 | The 3D helper and shared target policy reached 100%; context `Inherit` falls back deterministically, duplicate proxy/normal guards were removed, and dynamic plus kinematic CCD now honor the canonical pair gate including linked-joint suppression. Independent review found no issues. |

Completed campaigns established broad 2D, 3D, mixed, CCD, query, partition,
lifecycle, replay, serialization, diagnostics, and authored-shape coverage.
They also removed stale transient state, duplicate reducers and wrappers,
unreachable fallbacks, dead serialization paths, and incorrect ownership
branches. That history now informs the rules above without competing with the
remaining plan of attack.
