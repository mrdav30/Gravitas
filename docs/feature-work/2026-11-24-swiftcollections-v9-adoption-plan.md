# SwiftCollections v9 Adoption Plan

**Status:** Proposed; source audit complete, adoption evidence pending.  
**Owner:** Gravitas runtime collections and planar candidate indexing.  
**Scope:** Potential use of `SwiftDenseTicketMap<T>`, `SwiftTicketStore<T>`,
`SwiftFixedBVH2D<TKey>`, `SwiftFixedSpatialHash2D<TKey>` and
`SwiftFixedQuadtree<TKey>`. This is an implementation and verification plan,
not an instruction to migrate every owner.

## Decision And Boundaries

Start with a measured replacement experiment for the existing planar candidate
index. Separately test whether sparse body registration history costs enough to
justify a dense ticket issuer. Retain current owners when a candidate cannot
preserve ordering, lifecycle or full-workload cost. No speedup or host frame
budget is established by this audit.

The audit used Gravitas `9c372a74274022d9a737cb1747a014c44b4b9732`
and SwiftCollections source at `7c2fa696f10d88194f5a895aaafd7debd8e27574`.
Gravitas was clean before this documentation change. Its project already names
SwiftCollections and its FixedMathSharp companion version `9.0.0` in standard
and Lean package modes. Package availability, binary contents and performance
were not verified here. No runtime, tests, benchmark, configuration, dependency
pin or package changes belong to this planning change; no executions establish
new validation results.

Use the existing context/service owners and consume the provider directly.
Delete replaced storage, sort/update logic and dead helpers when a replacement
passes. Do not retain parallel permanent indexes, introduce a backend-selection
framework, or add forwarding wrappers without a concrete runtime consumer.
Temporary comparison paths are experiment scaffolding and must be removed.
Feature design stays here; do not assign feature work bug-tracker IDs.

For coordination, the sibling GridForge checkout records its completed evaluation
in `docs/feature-work/done/2026-11-24-swiftcollections-v9-adoption-plan.md`.
Its compatible dense-occupancy and projected-index prototypes were rejected and
removed after measured regressions. Use that record's ordering, lifecycle and
total-cost evidence as related context, not as a Gravitas adoption result.

## Source Audit And Candidate Decisions

All links below are repository source, tests or maintained guides. Provider
contracts are in the sibling SwiftCollections checkout under
`src/SwiftCollections/Collection/{DenseTicketMap,TicketStore,Support}` and
`src/SwiftCollections.FixedMathSharp/Query/{Bounds,BVH,SpatialHash,Quadtree}`.
Read those actual sources at the selected revision before implementation.

| Candidate and current owner | Decision | Reason and required proof |
| --- | --- | --- |
| Planar CCD base and dirty indexes in [`GravitasPhysics2DService.ContinuousCollision`](../../src/Gravitas/Core/2D/Services/GravitasPhysics2DService.ContinuousCollision.cs), backed by [`DynamicCcdCandidateIndex2D`](../../src/Gravitas/CollisionHandling/Continuous/Candidates/DynamicCcdCandidateIndex2D.cs) | First bounded experiment: native fixed-point BVH2D; compare spatial hash only for a demonstrated uniform/high-churn workload | Current storage is already native planar four-scalar bounds with a min-X sorted list, optional key-to-position update map, extent metadata and binary-search scan. It is not padded 3D storage. A BVH may reject clustered-X candidates better, but adds internal nodes, key lookup, update/rebuild cost, traversal scratch and order restoration. Include preparation, dirty override, handoff, queries and complete host frames. Retain the current sweep if the full workload does not improve. |
| Persistent projected-X/Z 3D circle index in [`GravitasCollisionService`](../../src/Gravitas/Core/3D/Services/GravitasCollisionService.cs), consumed by [`GravitasQuery3DService.Circle`](../../src/Gravitas/Queries/3D/GravitasQuery3DService.Circle.cs) | Second concrete consumer of the same planar-owner decision | `_planarQueryCandidates` also uses `DynamicCcdCandidateIndex2D`; registration, coverage refresh, removal and world-version rebuild update it. Its persistent updates differ from per-frame CCD rebuilding. Validate each consumer independently; replace both only if both pass. Partial adoption requires measured benefit that justifies the remaining code and maintenance cost. Avoid leaving a generic abstraction or two competing planar implementations merely to force consolidation. |
| `_dynamicBodies` in [`GravitasPhysicsService`](../../src/Gravitas/Core/3D/Services/GravitasPhysicsService.cs) and [`GravitasPhysics2DService`](../../src/Gravitas/Core/2D/Services/GravitasPhysics2DService.cs) | Conditional `SwiftDenseTicketMap` proof after the index slice | Slot maps are the real issuers of integer `DynamicId`. Full/CCD preparation and 3D late/visualization paths scan slot history; churn may make packed iteration useful. IDs also index CCD lifetime arrays and sleep/island work and enter joint replay hashes. Dense swap-back order changes processing order. Measure the existing sparse-history case and reject adoption if preserving current order requires an unprofitable second ordered owner. |
| [`ColliderRegistry<TCollider>`](../../src/Gravitas/Core/ColliderRegistry.cs) shared by both dimensional services | Retain by default; reconsider only with measured registry-specific savings | Already owns slot lookup plus dense live iteration with swap-back service-index repair, and a separate registration-order chain/replay snapshot. A dense ticket map duplicates much of the solved problem, changes integer-ID lookup and does not supply stable registration order. [`ColliderRegistryTests`](../../tests/Gravitas.Tests/Core/ColliderRegistryTests.cs) explicitly checks dense service indices, identity rejection and replay ordinals after removal. |
| Awake/dynamic/kinematic/static memberships in [`PhysicsPartition`](../../src/Gravitas/Partitions/3D/PhysicsPartition.cs), [`PhysicsPartition2D`](../../src/Gravitas/Partitions/2D/PhysicsPartition2D.cs) and [`PhysicsMixedPartition`](../../src/Gravitas/Partitions/Mixed/PhysicsMixedPartition.cs) | Retain current role collections; defer `SwiftTicketStore` | These store externally owned integer collider IDs, not full `SwiftTicket` values. Pure-dimensional role lists and mixed pooled hash memberships have deliberate ordering/copy rules and local-membership scaling. There is no matching Swift ticket issuer in the audited Gravitas path. Creating `SwiftTicket(id, syntheticGeneration)` from IDs or lifetime counters is invalid. |
| Optional body/collider side records, such as CCD lifetime entries, if a real issuer migration passes | Conditional future `SwiftTicketStore` consumer, not a new phase requirement | Name one existing array/map to remove and carry the issuer's full ticket through attach/lookup/teardown. A store verifies its local index/generation only; it cannot validate issuer liveness or make several stores atomic. Do not split existing body state into component stores for this feature. |
| Retained/empty partition ownership in [`RetainedPartitionLifecycle`](../../src/Gravitas/Partitions/RetainedPartitionLifecycle.cs) and dimensional/mixed collision services | Retain dense lists and intrusive indices | RetainedIndex/EmptyIndex already give swap-back removal, deterministic reuse and pre-reserved empty eligibility. Tickets introduce no required stale-handle contract here. Keep the single shared lifecycle rather than another dense registry. |
| Public pure-2D query gathering in [`GravitasCollision2DService.CollectBoundsCandidates`](../../src/Gravitas/Core/2D/Services/GravitasCollision2DService.cs), consumed by [`GravitasQuery2DService`](../../src/Gravitas/Queries/2D/GravitasQuery2DService.cs) | Defer broad replacement; require a separate measured deletion opportunity | Queries refresh physical GridForge membership, collect role-aware candidates, deduplicate and sort collider IDs before reduction. Adding a global query-only spatial hash/BVH while retaining all partition work is not presumed to save cost. Any future replacement must remove superseded gathering/update work and preserve voxel presence and committed-pose semantics. |
| `SwiftFixedSpatialHash2D` | Workload-specific comparison, not default replacement | Useful hypothesis for roughly uniform movers and small sparse windows. Measure cell visits, large swept rectangles, negative coordinates, extreme/clamped cell indices, migration between cells, membership allocation and duplicate suppression. Entry `EnsureCapacity` is not a no-allocation guarantee for cells. |
| `SwiftFixedQuadtree` | Defer unless a concrete fixed-world regional-query caller wins | Immutable root bounds require complete containment. Gravitas supports world changes and large positions; inventing a global root, silent rejection, clamping or overflow fallback would alter admission. Prove root policy, split/merge/straddler cost and all callers before adopting. |
| GridForge world topology, 3D CCD indexes, mixed 3D slabs and 3D broad phase | Retain domain owners | Planar indexes cannot replace full `WorldVoxelIndex`, world/grid lifetime or Y-sensitive slabs. No flattening of 3D/mixed physics is authorized. |

## Contracts The Replacement Must Preserve

**Retain bounded pure-2D support discovery.** The separate
[`GravitasQuery2DService.BoundedSupport`](../../src/Gravitas/Queries/2D/GravitasQuery2DService.BoundedSupport.cs)
path uses `PhysicsQueryScratch.Discover`, role-membership work charging,
`RememberCandidate` and `CheckView`; it does not use ordinary
`CollectBoundsCandidates`. Keep its exact budget, scratch, view-validation and
status semantics. BVH `TryQuery` admission based on worst-case leaf Count is not
a substitute for this public work meter. Reuse
[`Physics2DSupportQueryTests`](../../tests/Gravitas.Tests/Physics2D/Physics2DSupportQueryTests.cs)
and [`PhysicsQueryScratchTests`](../../tests/Gravitas.Tests/Queries/PhysicsQueryScratchTests.cs)
as unchanged controls, including one-below/exact budgets and capacities,
incomplete support versus a completed miss, failure mutation and view changes.

### Identity, iteration and replay

`SwiftDenseTicketMap` owns slot generations, rejects default tickets and retires
maximum-generation slots instead of wrapping. It retains dense values plus slot
history and free order; `Clear` invalidates live tickets while retaining history.
`SwiftTicketStore` accepts generation zero, rejects any occupied-index attachment,
does not advance generations, and sizes sparse lookup by highest index. Neither
type gives canonical/insertion order, stable borrowed storage, owner identity or
timeline identity. Numeric tickets from distinct owners or restored timelines
can alias. Keep context/dimension/issuer checks explicit.

Before changing a body issuer, trace `DynamicId` through both services'
`Response` and `ContinuousCollision` partials,
[`GravitasWorldContext.Sleep`](../../src/Gravitas/Runtime/GravitasWorldContext.Sleep.cs),
[`Joint2D`](../../src/Gravitas/Constraints/2D/Joints/Joint2D.cs),
[`Joint3D`](../../src/Gravitas/Constraints/3D/Joints/Joint3D.cs), and replay
partials. Slot identity and dense position must remain separate. Current ordered
body processing is not replaceable with swap-back enumeration just because both
are deterministic for their own mutation history. Preserve solver, CCD handoff,
sleep-island, notification and pair tie order. If that blocks a profitable
replacement, retain the slot map unless the user separately approves a concrete
contract change. Do not silently rewrite expected hashes.

Per the [serialization guide](../api/guides/SERIALIZATION.md), context IDs,
service indices, partitions, pairs and query caches are derived runtime state;
recorded bodies/settings and explicit declaration links remain authoritative.
Do not serialize a newly introduced ticket merely because its collection offers
a State codec. Rebuild derived state in canonical registration order, preserve
populate-existing continuation and cross-context separation, and test old held
work across reset, object reuse and restored timelines. Existing
[`ColliderLifetimeToken`](../../src/Gravitas/Colliders/State/ColliderLifetimeToken.cs)
and [`ColliderLifetimeToken2D`](../../src/Gravitas/Colliders/State/ColliderLifetimeToken2D.cs)
also protect object registration lifetimes; a body slot ticket does not
automatically replace collider, pair or component-instance revisions.

### Planar geometry, ordering and capacity

Swift native 2D indexes use XY `Vector2d`; Gravitas pure 2D values represent
world X/Z. Map `(world.X, world.Z)` explicitly to `(index.X, index.Y)` at the
owning boundary. `DynamicCcdPlanarBounds` already stores X/Z endpoints. Use
`FixedBoundVolume2D` endpoint construction and inclusive intersection, including
touching and zero-size rectangles; do not read checked `Size`/`Area` to choose
an otherwise valid full-domain path. Preserve the current unrepresentable-X
extent fallback and no-false-negative behavior. No new geometry admission limit
is implied by choosing an index.

The pure 2D GridForge Y=0 plane is storage identity. Mixed colliders have physical
host-Y placement and finite `Mixed2DHalfThickness`/overrides. Preserve
[`DIMENSIONS`](../api/guides/DIMENSIONS.md) and dedicated mixed CCD bounds,
pair keys and reducers. Projected 3D circle queries are intentionally planar
candidate queries followed by their existing physical reducers; that does not
authorize flattening other 3D queries.

Each current sweep orders by `(MinX, MinZ, MaxX, MaxZ, DynamicId)`. The effective
CCD sequence is the sorted base results with dirty bodies removed, followed by
the sorted dirty results, with lifetime validation in both parts. It is not a
globally sorted merged sequence; preserve this distinction when restoring order.
Provider BVH, hash and quadtree traversal orders differ and depend on
topology/mutation history.
Restore the current effective ordering before CCD or query workers can observe
candidates; include bounds lookup, copies and sorting in costs. Preserve base
versus dirty candidate override, stale-lifetime rejection, exact-distance/closing
speed/ID arbitration, all-hit sorting, duplicate safety and append/failure
contracts. Public 2D gathering already canonicalizes collider IDs. Do not infer
that order is irrelevant from a passing unordered set comparison.

Provider `Query` appends without clearing and may partially append on failure;
the current candidate index clears its result list. Keep the current consumer
contract explicitly. BVH `Query` scratch can grow. Its `TryQuery` requires a
plain SwiftList with spare capacity and disjoint traversal scratch of at least
entry Count; insufficient capacity returns false before mutating either buffer.
Reserve correctly outside hot work and preserve that behavior where used. Do
not implement a silent growing fallback that breaks an allocation gate.

Count BVH internal nodes (up to `2N-1` live nodes), lookup metadata, capacity
rounding and retained peak storage. Compare per-frame `Clear`/reinsert against
the current list's `FastClear`; provider insertion/update and hash-cell/tree
creation are not blanket warmed-allocation guarantees. Prove first use, changed
bounds, removals, shrink/regrow and reset behavior. Keep query admission/work
budgets, early exits and caller-owned result buffers intact; a new backend
cannot promise equal bounded work without evidence.

### Lifecycle and mutation

Preserve single context ownership, role transitions, sleep/wake and mass-change
synchronization, partition delta refresh, pooling and retained capacity rules.
Callbacks may remove/reinitialize a collider or reuse its integer ID during
contact/query work; retained candidates must validate the captured lifetime
again. Do not hold dense positions, refs or spans across structural mutation or
callbacks. Preserve current query reentrancy guards and settings/geometry
transactional admission; provider indexes must not be mutated or reentered from
key/result callbacks. Reset/dispose must release ended graphs and support reuse
without retaining departed bodies through scratch or snapshots.

## Phase 0: Baseline And Smallest Decisive Proof

- [ ] Freeze exact consumer/provider revisions, dirty patches, dependency graph,
      SDK/runtime, configuration, host and fixture inputs. Use the existing
      benchmark runner and maintained support; do not add tests of launchers.
- [ ] Capture the committed baseline with the smallest complete pure-2D host
      case: one continuous moving circle and one eligible dynamic target, using
      `Simulate` and `LateSimulate`, scheduled overlap/raycast and normal
      lifecycle teardown. Check the accepted pose, finite contact/witness,
      velocity, event sequence, ordered hits and replay continuation against an
      independent simple geometry oracle before timing. Verify that the dynamic
      target is discovered through the planar candidate index; a static obstacle
      alone does not prove this path. Keep a static-obstacle control and include
      a kinematic-to-dynamic CCD handoff case before expanding.
- [ ] Add the discriminating adversary: several candidates share X intervals
      but are separated in Z; include touching/zero-size bounds, negative and
      large coordinates, a widest candidate whose span is unrepresentable,
      equal-TOI candidates and dirty trajectory changes. Verify exact ordered
      candidates and no false negatives against a straightforward endpoint
      intersection oracle. Run the real CCD/query callers as well as the index.
- [ ] Exercise removal and same-ID reinitialization from a contact callback,
      changed role/sleep and populate/reset/reuse. The new registration must not
      inherit the old candidate or pending handoff. Include the 3D and explicit
      mixed controls, with coincident planar footprints at different physical
      Y/heights/thicknesses, preserving the eligible result.
- [ ] Measure baseline and native-BVH candidate with equivalent physical output.
      Include prepare/clear/rebuild, update/remove, traversal, filtering, order
      restoration and query publication, then complete host step and scenario
      frame. Compare clustered-X and ordinary uniform scenes; inspect scratch,
      dense/slot/node/cell capacity and allocation under churn.
- [ ] Review the first actual shared runtime slice independently before more
      consumers. Record accepted design, rejected alternatives and the measured
      reason for continuing here. Stop and retain the baseline if correctness,
      ordering, allocations or full-workload cost cannot pass.

Use existing evidence owners, extending only the necessary product cases:

| Proof | Existing starting points |
| --- | --- |
| CCD bounds/update/tie/lifetime | `tests/Gravitas.Tests/CollisionHandling/ContinuousCollisionBoundsTests.cs`, `ContinuousCollisionCandidateOrderingTests.cs`, `tests/Gravitas.Tests/Physics2D/ContinuousCollision2DTests.RotationalCandidateLifetime.cs` and other trajectory/handoff partials |
| Physical query/world visibility | `tests/Gravitas.Tests/Queries/QueryPartitionReadinessTests.cs`, `tests/Gravitas.Tests/Physics2D/Physics2DPartitionCandidateTransitionTests.cs`, `PreparedSegmentQuery2DTests.cs` and mixed query/CCD tests |
| Identity/density/reset | `tests/Gravitas.Tests/Core/ColliderRegistryTests.cs`, `tests/Gravitas.Tests/Runtime/ContextResetLifetimeTests.cs`, `tests/Gravitas.Tests/Partitions/RetainedPartitionLifecycleTests.cs`, `PartitionMembershipScalingTests.cs` and sleep/role transition tests |
| Kernels plus full workload | `tests/Gravitas.Benchmarks/Core/DynamicCandidateIndexUpdateScalingBenchmarks.cs`, `DynamicCcdScalingBenchmarks.cs`, `KinematicActiveCcdScalingBenchmarks.cs`, `tests/Gravitas.Benchmarks/Queries/QueryProjectionScalingBenchmarks.cs`, `PostResponseQueryBenchmarks.cs`, `tests/Gravitas.Benchmarks/Physics2D/CircleContactSimulationBenchmarks.cs` and `Strategy2DWorkload.cs` |
| Replay/state | `tests/Gravitas.Tests/Determinism/SharedReplayFixtureTests.cs`, `GravitasReplayConformanceTests.cs`, `tests/Gravitas.Tests/Serialization` and the existing duration/continuation verifier |

Provider regressions belong in SwiftCollections tests if a reusable defect is
discovered. Existing `SwiftDenseTicketMap.*`, `SwiftTicketStore.*` and native
fixed 2D query tests are API-contract references, not substitutes for consumer
proof. No defect was established by this audit.

## Phase 1: Complete A Winning Planar Replacement

- [ ] Choose one backend from Phase 0 evidence. Consume it in the existing
      planar CCD owner, retaining separate physical mixed indexing. Preserve
      base/dirty override and all lifetime/order rules.
- [ ] Verify the persistent projected-3D-circle consumer with its own update and
      world-version rebuild schedule. Include sustained spatial rearrangement:
      start with coherent clusters, then repeatedly move alternating members
      between clusters while keeping live population and local query size fixed.
      BVH bounds updates refit ancestors without relocating leaves or rebuilding
      topology. Measure query cost after sustained movement and include any
      chosen rebuild cost; a freshly built tree is insufficient evidence.
      Adopt the same proven owner where its complete cost passes; otherwise
      document why retaining the current sweep is preferable and reassess whether
      a partial replacement is worth its net runtime growth and maintenance cost.
- [ ] Remove superseded heapsort, entry/update maps, extent bookkeeping and
      `DynamicCcdPlanarBounds` helpers only when all remaining production callers
      permit deletion. Retain policy-bearing code at its owner. Report source
      growth, deleted mechanisms and actual production callers separately from
      tests and docs.
- [ ] Close this phase with full consumer verification and measured workloads.
      Do not widen to global pure-2D partition/query replacement merely because
      the local candidate-index experiment wins.

## Phase 2: Conditional Dense Issuer And External Store Decision

- [ ] Profile real sparse registration history in both dimensional body owners:
      same live population at different historical peaks, sleeping versus active
      actors, burst spawn/despawn, motion-type changes, CCD and constraints.
      Include retained generations, retirement/free-list metadata and sleep/CCD
      arrays. Add a dense issuer only when this cost is material.
- [ ] Prove the smallest full body lifecycle using actual tickets, stale/default
      rejection, slot reuse, terminal retirement, context separation and reset.
      Preserve public `int DynamicId`, its `-1` unregistered/static sentinel,
      existing numeric identity/reuse behavior and body/joint replay references.
      Keep a full issuer ticket private if needed; replacing public IDs with
      `SwiftTicket` requires a separately justified and approved API change.
      Preserve effective body/solver order, joint replay identity and all callback
      mutation semantics. Do not create a second permanent authoritative ordered
      body registry to disguise a failed replacement.
- [ ] For any proposed store, name its actual issuer, exact removed side record,
      full-ticket propagation, ordered attach/detach transaction and abandoned
      timeline policy. Validate issuer liveness separately and remove attached
      records before issuer reuse. Otherwise explicitly close store adoption as
      deferred: integer membership alone is insufficient justification.
- [ ] Keep collider and retained-partition registries unchanged unless separate
      complete-workload evidence demonstrates net deletion and useful savings.
      Finish with a documented adopt/retain decision for each attempted owner.

## Verification And Measurement Acceptance

One coordinator owns mutable build/test/coverage/timing outputs. Read reviewers
can work concurrently; run acceptance measurements without competing work.

The full applicable checklist below closes an adopted implementation. If an
experiment is rejected and no source changes remain, close it with the decisive
correctness/performance evidence, confirmation that the prototype was removed,
and the retention rationale and reopening criteria. Record unreached adoption
gates as not applicable to that outcome, not as passed; do not run fresh
whole-library coverage or the full native matrix solely to reject a removed
prototype. Any retained test, benchmark or tooling changes still require their
own applicable verification.

- [ ] Build both runtime target frameworks (`netstandard2.1`, `net10.0`) in
      `Release` and `ReleaseLean`; run the net10 test suite in both. Preserve
      standard/Lean package identities and source-generated serializer behavior.
      Collect fresh coverage using `tests/Gravitas.Tests/coverlet.runsettings`
      and run `.assets/scripts/assert-full-coverage.ps1` per configuration:
      100% reported lines, branches and executable methods by integer counts,
      with no new exclusions or uncovered guards.
- [ ] Verify package mode and, for coordinated unreleased changes, the intended
      source graph via `UseLocalLsfStack=true` plus global
      `DisableTransitiveProjectReferences=true`, serial `-m:1` builds and the
      existing local assembly identities. Carry source selection to generated
      benchmark children; inspect loaded binaries, child exit/results and all
      expected cases. Follow [CONTRIBUTING](../../CONTRIBUTING.md).
- [ ] Run focused high-risk contracts before full suites: serialization and
      populate-existing continuation, reset/pool graph release, role transitions,
      wake/grounding, ID reuse/reentrancy, reordered removals, world/grid
      replacement and mixed slab controls. Preserve replay fixture hashes unless
      an explicitly reviewed semantic change requires otherwise; run existing
      native conformance captures/comparisons for supported Windows/Linux
      x64/ARM64 lanes rather than inferring portability from repeated local runs.
- [ ] Preserve existing benchmark README allocation smoke targets and relevant
      simulation, query, partition, mixed, diagnostics and replay gates. Report
      startup separately from warm cost; measure complete
      `Simulate + LateSimulate` and scenario frames including scheduled queries,
      commands and churn. Include retained-memory plateaus, allocation spikes,
      practical frame latency and constrained/connected-island adversaries.
- [ ] Compare balanced independent baseline/candidate runs with equivalent
      inputs, order and physical outputs. Retain adverse/inconclusive results;
      kernel gains or `0 B/op` alone do not establish full-step improvement.
      Do not invent speedup thresholds or a fixed-Hz budget. A measured
      regression introduced here must be resolved here; only the user can
      accept a tradeoff. If no repeatable useful gain remains after publication,
      ordering and lifecycle costs, remove the candidate and record retention.
- [ ] Update affected XML/conceptual guidance and run repository documentation
      checks when implementation changes public/runtime contracts. Preserve
      complexity rationales without using them to waive coverage.

If reusable provider changes become necessary, implement and verify them in
SwiftCollections first, then verify Gravitas against that exact source graph.
After those upstream changes land, remind the coordinator to refresh the actual
Gravitas CI build/test sibling pin. This reminder is conditional; this plan
requires no dependency-pin edit and has no distribution, publication or release
completion gate.

## Completion Record

Record phase decisions and concise evidence in this plan: revisions and patches,
fixture/settings/commands, independent oracle and exact outputs, test/coverage
counts, dependency modes, allocation/capacity observations, timing units and
dispersion, rejected candidates and reopening criteria. Keep essential evidence
self-contained rather than relying on ignored benchmark output. Leave changes
unstaged and uncommitted for review. Completion is implementation plus required
verification of the selected owners, or a verified no-adoption decision; the
existence of these provider APIs alone is not an adoption requirement.
