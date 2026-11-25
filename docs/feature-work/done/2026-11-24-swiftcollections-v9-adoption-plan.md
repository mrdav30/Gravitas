# SwiftCollections v9 Adoption Plan

**Status:** Phase 0 retained the sweep on 2026-11-24. The bounded 2026-11-25
follow-up selects the revised native BVH under the user's relaxed tradeoff
criterion; see the follow-up record for evidence and validation scope.  
**Owner:** Gravitas runtime collections and planar candidate indexing.  
**Scope:** Potential use of `SwiftDenseTicketMap<T>`, `SwiftTicketStore<T>`,
`SwiftFixedBVH2D<TKey>`, `SwiftFixedSpatialHash2D<TKey>` and
`SwiftFixedQuadtree<TKey>`. This is an implementation and verification plan, not
an instruction to migrate every owner.

## Decision And Boundaries

The original proposal and Phase 0 record below are historical. The bounded
follow-up at the end governs the current implementation and acceptance decision;
issuer/store work remains deferred.

Start with a measured replacement experiment for the existing planar candidate
index. Separately test whether sparse body registration history costs enough to
justify a dense ticket issuer. Retain current owners when a candidate cannot
preserve ordering, lifecycle or full-workload cost. No speedup or host frame
budget is established by this audit.

The initial source-only audit used Gravitas
`9c372a74274022d9a737cb1747a014c44b4b9732` and SwiftCollections source at
`7c2fa696f10d88194f5a895aaafd7debd8e27574`. Gravitas was clean before that
documentation change. Its project already named SwiftCollections and its
FixedMathSharp companion version `9.0.0` in standard and Lean package modes.
That initial audit did not verify package availability, binary contents or
performance, change runtime/tests/benchmarks/configuration/ dependency pins, or
establish execution results. The completed Phase 0 record below supersedes its
pending-evidence status and records the retained benchmark and measured
no-adoption decision. The original design and unselected adoption checklists
remain as context.

Use the existing context/service owners and consume the provider directly.
Delete replaced storage, sort/update logic and dead helpers when a replacement
passes. Do not retain parallel permanent indexes, introduce a backend-selection
framework, or add forwarding wrappers without a concrete runtime consumer.
Temporary comparison paths are experiment scaffolding and must be removed.
Feature design stays here; do not assign feature work bug-tracker IDs.

For coordination, the sibling GridForge checkout records its completed
evaluation in
`docs/feature-work/done/2026-11-24-swiftcollections-v9-adoption-plan.md`. Its
compatible dense-occupancy and projected-index prototypes were rejected and
removed after measured regressions. Use that record's ordering, lifecycle and
total-cost evidence as related context, not as a Gravitas adoption result.

## Source Audit And Candidate Decisions

All links below are repository source, tests or maintained guides. Provider
contracts are in the sibling SwiftCollections checkout under
`src/SwiftCollections/Collection/{DenseTicketMap,TicketStore,Support}` and
`src/SwiftCollections.FixedMathSharp/Query/{Bounds,BVH,SpatialHash,Quadtree}`.
Read those actual sources at the selected revision before implementation.

| Candidate and current owner                                                                                                                                                                                                                                                                                          | Decision                                                                                                                     | Reason and required proof                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Planar CCD base and dirty indexes in [`GravitasPhysics2DService.ContinuousCollision`](../../../src/Gravitas/Core/2D/Services/GravitasPhysics2DService.ContinuousCollision.cs), backed by [`DynamicCcdCandidateIndex2D`](../../../src/Gravitas/CollisionHandling/Continuous/Candidates/DynamicCcdCandidateIndex2D.cs) | First bounded experiment: native fixed-point BVH2D; compare spatial hash only for a demonstrated uniform/high-churn workload | Current storage is already native planar four-scalar bounds with a min-X sorted list, optional key-to-position update map, extent metadata and binary-search scan. It is not padded 3D storage. A BVH may reject clustered-X candidates better, but adds internal nodes, key lookup, update/rebuild cost, traversal scratch and order restoration. Include preparation, dirty override, handoff, queries and complete host frames. Retain the current sweep if the full workload does not improve. |
| Persistent projected-X/Z 3D circle index in [`GravitasCollisionService`](../../../src/Gravitas/Core/3D/Services/GravitasCollisionService.cs), consumed by [`GravitasQuery3DService.Circle`](../../../src/Gravitas/Queries/3D/GravitasQuery3DService.Circle.cs)                                                       | Second concrete consumer of the same planar-owner decision                                                                   | `_planarQueryCandidates` also uses `DynamicCcdCandidateIndex2D`; registration, coverage refresh, removal and world-version rebuild update it. Its persistent updates differ from per-frame CCD rebuilding. Validate each consumer independently; replace both only if both pass. Partial adoption requires measured benefit that justifies the remaining code and maintenance cost. Avoid leaving a generic abstraction or two competing planar implementations merely to force consolidation.     |
| `_dynamicBodies` in [`GravitasPhysicsService`](../../../src/Gravitas/Core/3D/Services/GravitasPhysicsService.cs) and [`GravitasPhysics2DService`](../../../src/Gravitas/Core/2D/Services/GravitasPhysics2DService.cs)                                                                                                | Conditional `SwiftDenseTicketMap` proof after the index slice                                                                | Slot maps are the real issuers of integer `DynamicId`. Full/CCD preparation and 3D late/visualization paths scan slot history; churn may make packed iteration useful. IDs also index CCD lifetime arrays and sleep/island work and enter joint replay hashes. Dense swap-back order changes processing order. Measure the existing sparse-history case and reject adoption if preserving current order requires an unprofitable second ordered owner.                                             |
| [`ColliderRegistry<TCollider>`](../../../src/Gravitas/Core/ColliderRegistry.cs) shared by both dimensional services                                                                                                                                                                                                  | Retain by default; reconsider only with measured registry-specific savings                                                   | Already owns slot lookup plus dense live iteration with swap-back service-index repair, and a separate registration-order chain/replay snapshot. A dense ticket map duplicates much of the solved problem, changes integer-ID lookup and does not supply stable registration order. [`ColliderRegistryTests`](../../../tests/Gravitas.Tests/Core/ColliderRegistryTests.cs) explicitly checks dense service indices, identity rejection and replay ordinals after removal.                          |
| Awake/dynamic/kinematic/static memberships in [`PhysicsPartition`](../../../src/Gravitas/Partitions/3D/PhysicsPartition.cs), [`PhysicsPartition2D`](../../../src/Gravitas/Partitions/2D/PhysicsPartition2D.cs) and [`PhysicsMixedPartition`](../../../src/Gravitas/Partitions/Mixed/PhysicsMixedPartition.cs)        | Retain current role collections; defer `SwiftTicketStore`                                                                    | These store externally owned integer collider IDs, not full `SwiftTicket` values. Pure-dimensional role lists and mixed pooled hash memberships have deliberate ordering/copy rules and local-membership scaling. There is no matching Swift ticket issuer in the audited Gravitas path. Creating `SwiftTicket(id, syntheticGeneration)` from IDs or lifetime counters is invalid.                                                                                                                 |
| Optional body/collider side records, such as CCD lifetime entries, if a real issuer migration passes                                                                                                                                                                                                                 | Conditional future `SwiftTicketStore` consumer, not a new phase requirement                                                  | Name one existing array/map to remove and carry the issuer's full ticket through attach/lookup/teardown. A store verifies its local index/generation only; it cannot validate issuer liveness or make several stores atomic. Do not split existing body state into component stores for this feature.                                                                                                                                                                                              |
| Retained/empty partition ownership in [`RetainedPartitionLifecycle`](../../../src/Gravitas/Partitions/RetainedPartitionLifecycle.cs) and dimensional/mixed collision services                                                                                                                                        | Retain dense lists and intrusive indices                                                                                     | RetainedIndex/EmptyIndex already give swap-back removal, deterministic reuse and pre-reserved empty eligibility. Tickets introduce no required stale-handle contract here. Keep the single shared lifecycle rather than another dense registry.                                                                                                                                                                                                                                                    |
| Public pure-2D query gathering in [`GravitasCollision2DService.CollectBoundsCandidates`](../../../src/Gravitas/Core/2D/Services/GravitasCollision2DService.cs), consumed by [`GravitasQuery2DService`](../../../src/Gravitas/Queries/2D/GravitasQuery2DService.cs)                                                   | Defer broad replacement; require a separate measured deletion opportunity                                                    | Queries refresh physical GridForge membership, collect role-aware candidates, deduplicate and sort collider IDs before reduction. Adding a global query-only spatial hash/BVH while retaining all partition work is not presumed to save cost. Any future replacement must remove superseded gathering/update work and preserve voxel presence and committed-pose semantics.                                                                                                                       |
| `SwiftFixedSpatialHash2D`                                                                                                                                                                                                                                                                                            | Workload-specific comparison, not default replacement                                                                        | Useful hypothesis for roughly uniform movers and small sparse windows. Measure cell visits, large swept rectangles, negative coordinates, extreme/clamped cell indices, migration between cells, membership allocation and duplicate suppression. Entry `EnsureCapacity` is not a no-allocation guarantee for cells.                                                                                                                                                                               |
| `SwiftFixedQuadtree`                                                                                                                                                                                                                                                                                                 | Defer unless a concrete fixed-world regional-query caller wins                                                               | Immutable root bounds require complete containment. Gravitas supports world changes and large positions; inventing a global root, silent rejection, clamping or overflow fallback would alter admission. Prove root policy, split/merge/straddler cost and all callers before adopting.                                                                                                                                                                                                            |
| GridForge world topology, 3D CCD indexes, mixed 3D slabs and 3D broad phase                                                                                                                                                                                                                                          | Retain domain owners                                                                                                         | Planar indexes cannot replace full `WorldVoxelIndex`, world/grid lifetime or Y-sensitive slabs. No flattening of 3D/mixed physics is authorized.                                                                                                                                                                                                                                                                                                                                                   |

## Contracts The Replacement Must Preserve

**Retain bounded pure-2D support discovery.** The separate
[`GravitasQuery2DService.BoundedSupport`](../../../src/Gravitas/Queries/2D/GravitasQuery2DService.BoundedSupport.cs)
path uses `PhysicsQueryScratch.Discover`, role-membership work charging,
`RememberCandidate` and `CheckView`; it does not use ordinary
`CollectBoundsCandidates`. Keep its exact budget, scratch, view-validation and
status semantics. BVH `TryQuery` admission based on worst-case leaf Count is not
a substitute for this public work meter. Reuse
[`Physics2DSupportQueryTests`](../../../tests/Gravitas.Tests/Physics2D/Physics2DSupportQueryTests.cs)
and
[`PhysicsQueryScratchTests`](../../../tests/Gravitas.Tests/Queries/PhysicsQueryScratchTests.cs)
as unchanged controls, including one-below/exact budgets and capacities,
incomplete support versus a completed miss, failure mutation and view changes.

### Identity, iteration and replay

`SwiftDenseTicketMap` owns slot generations, rejects default tickets and retires
maximum-generation slots instead of wrapping. It retains dense values plus slot
history and free order; `Clear` invalidates live tickets while retaining
history. `SwiftTicketStore` accepts generation zero, rejects any occupied-index
attachment, does not advance generations, and sizes sparse lookup by highest
index. Neither type gives canonical/insertion order, stable borrowed storage,
owner identity or timeline identity. Numeric tickets from distinct owners or
restored timelines can alias. Keep context/dimension/issuer checks explicit.

Before changing a body issuer, trace `DynamicId` through both services'
`Response` and `ContinuousCollision` partials,
[`GravitasWorldContext.Sleep`](../../../src/Gravitas/Runtime/GravitasWorldContext.Sleep.cs),
[`Joint2D`](../../../src/Gravitas/Constraints/2D/Joints/Joint2D.cs),
[`Joint3D`](../../../src/Gravitas/Constraints/3D/Joints/Joint3D.cs), and replay
partials. Slot identity and dense position must remain separate. Current ordered
body processing is not replaceable with swap-back enumeration just because both
are deterministic for their own mutation history. Preserve solver, CCD handoff,
sleep-island, notification and pair tie order. If that blocks a profitable
replacement, retain the slot map unless the user separately approves a concrete
contract change. Do not silently rewrite expected hashes.

Per the [serialization guide](../../api/guides/SERIALIZATION.md), context IDs,
service indices, partitions, pairs and query caches are derived runtime state;
recorded bodies/settings and explicit declaration links remain authoritative. Do
not serialize a newly introduced ticket merely because its collection offers a
State codec. Rebuild derived state in canonical registration order, preserve
populate-existing continuation and cross-context separation, and test old held
work across reset, object reuse and restored timelines. Existing
[`ColliderLifetimeToken`](../../../src/Gravitas/Colliders/State/ColliderLifetimeToken.cs)
and
[`ColliderLifetimeToken2D`](../../../src/Gravitas/Colliders/State/ColliderLifetimeToken2D.cs)
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

The pure 2D GridForge Y=0 plane is storage identity. Mixed colliders have
physical host-Y placement and finite `Mixed2DHalfThickness`/overrides. Preserve
[`DIMENSIONS`](../../api/guides/DIMENSIONS.md) and dedicated mixed CCD bounds,
pair keys and reducers. Projected 3D circle queries are intentionally planar
candidate queries followed by their existing physical reducers; that does not
authorize flattening other 3D queries.

Each current sweep orders by `(MinX, MinZ, MaxX, MaxZ, DynamicId)`. The
effective CCD sequence is the sorted base results with dirty bodies removed,
followed by the sorted dirty results, with lifetime validation in both parts. It
is not a globally sorted merged sequence; preserve this distinction when
restoring order. Provider BVH, hash and quadtree traversal orders differ and
depend on topology/mutation history. Restore the current effective ordering
before CCD or query workers can observe candidates; include bounds lookup,
copies and sorting in costs. Preserve base versus dirty candidate override,
stale-lifetime rejection, exact-distance/closing speed/ID arbitration, all-hit
sorting, duplicate safety and append/failure contracts. Public 2D gathering
already canonicalizes collider IDs. Do not infer that order is irrelevant from a
passing unordered set comparison.

Provider `Query` appends without clearing and may partially append on failure;
the current candidate index clears its result list. Keep the current consumer
contract explicitly. BVH `Query` scratch can grow. Its `TryQuery` requires a
plain SwiftList with spare capacity and disjoint traversal scratch of at least
entry Count; insufficient capacity returns false before mutating either buffer.
Reserve correctly outside hot work and preserve that behavior where used. Do not
implement a silent growing fallback that breaks an allocation gate.

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

- [x] Freeze exact consumer/provider revisions, dirty patches, dependency graph,
      SDK/runtime, configuration, host and fixture inputs. Use the existing
      benchmark runner and maintained support; do not add tests of launchers.
- [x] Capture the committed baseline with the smallest complete pure-2D host
      case: one continuous moving circle and one eligible dynamic target, using
      `Simulate` and `LateSimulate`, scheduled overlap/raycast and normal
      lifecycle teardown. Check the accepted pose, finite contact/witness,
      velocity, event sequence, ordered hits and replay continuation against an
      independent simple geometry oracle before timing. Verify that the dynamic
      target is discovered through the planar candidate index; a static obstacle
      alone does not prove this path. Keep a static-obstacle control and include
      a kinematic-to-dynamic CCD handoff case before expanding.
- [x] Add the discriminating adversary: several candidates share X intervals but
      are separated in Z; include touching/zero-size bounds, negative and large
      coordinates, a widest candidate whose span is unrepresentable, equal-TOI
      candidates and dirty trajectory changes. Verify exact ordered candidates
      and no false negatives against a straightforward endpoint intersection
      oracle. Run the real CCD/query callers as well as the index.
- [x] Exercise removal and same-ID reinitialization from a contact callback,
      changed role/sleep and populate/reset/reuse. The new registration must not
      inherit the old candidate or pending handoff. Include the 3D and explicit
      mixed controls, with coincident planar footprints at different physical
      Y/heights/thicknesses, preserving the eligible result.
- [x] Measure baseline and native-BVH candidate with equivalent physical output.
      Include prepare/clear/rebuild, update/remove, traversal, filtering, order
      restoration and query publication, then complete host step and scenario
      frame. Compare clustered-X and ordinary uniform scenes; inspect scratch,
      dense/slot/node/cell capacity and allocation under churn.
- [x] Review the first actual shared runtime slice independently before more
      consumers. Record accepted design, rejected alternatives and the measured
      reason for continuing here. Stop and retain the baseline if correctness,
      ordering, allocations or full-workload cost cannot pass.

Use existing evidence owners, extending only the necessary product cases:

| Proof                           | Existing starting points                                                                                                                                                                                                                                                                                                                                                        |
| ------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| CCD bounds/update/tie/lifetime  | `tests/Gravitas.Tests/CollisionHandling/ContinuousCollisionBoundsTests.cs`, `ContinuousCollisionCandidateOrderingTests.cs`, `tests/Gravitas.Tests/Physics2D/ContinuousCollision2DTests.RotationalCandidateLifetime.cs` and other trajectory/handoff partials                                                                                                                    |
| Physical query/world visibility | `tests/Gravitas.Tests/Queries/QueryPartitionReadinessTests.cs`, `tests/Gravitas.Tests/Physics2D/Physics2DPartitionCandidateTransitionTests.cs`, `PreparedSegmentQuery2DTests.cs` and mixed query/CCD tests                                                                                                                                                                      |
| Identity/density/reset          | `tests/Gravitas.Tests/Core/ColliderRegistryTests.cs`, `tests/Gravitas.Tests/Runtime/ContextResetLifetimeTests.cs`, `tests/Gravitas.Tests/Partitions/RetainedPartitionLifecycleTests.cs`, `PartitionMembershipScalingTests.cs` and sleep/role transition tests                                                                                                                   |
| Kernels plus full workload      | `tests/Gravitas.Benchmarks/Core/DynamicCandidateIndexUpdateScalingBenchmarks.cs`, `DynamicCcdScalingBenchmarks.cs`, `KinematicActiveCcdScalingBenchmarks.cs`, `tests/Gravitas.Benchmarks/Queries/QueryProjectionScalingBenchmarks.cs`, `PostResponseQueryBenchmarks.cs`, `tests/Gravitas.Benchmarks/Physics2D/CircleContactSimulationBenchmarks.cs` and `Strategy2DWorkload.cs` |
| Replay/state                    | `tests/Gravitas.Tests/Determinism/SharedReplayFixtureTests.cs`, `GravitasReplayConformanceTests.cs`, `tests/Gravitas.Tests/Serialization` and the existing duration/continuation verifier                                                                                                                                                                                       |

Provider regressions belong in SwiftCollections tests if a reusable defect is
discovered. Existing `SwiftDenseTicketMap.*`, `SwiftTicketStore.*` and native
fixed 2D query tests are API-contract references, not substitutes for consumer
proof. No defect was established by this audit.

## Phase 1: Complete A Winning Planar Replacement

Not selected after the Phase 0 retain decision. Unchecked items are historical
adoption criteria, not pending work in this completed evaluation.

- [ ] Choose one backend from Phase 0 evidence. Consume it in the existing
      planar CCD owner, retaining separate physical mixed indexing. Preserve
      base/dirty override and all lifetime/order rules.
- [ ] Verify the persistent projected-3D-circle consumer with its own update and
      world-version rebuild schedule. Include sustained spatial rearrangement:
      start with coherent clusters, then repeatedly move alternating members
      between clusters while keeping live population and local query size fixed.
      BVH bounds updates refit ancestors without relocating leaves or rebuilding
      topology. Measure query cost after sustained movement and include any
      chosen rebuild cost; a freshly built tree is insufficient evidence. Adopt
      the same proven owner where its complete cost passes; otherwise document
      why retaining the current sweep is preferable and reassess whether a
      partial replacement is worth its net runtime growth and maintenance cost.
- [ ] Remove superseded heapsort, entry/update maps, extent bookkeeping and
      `DynamicCcdPlanarBounds` helpers only when all remaining production
      callers permit deletion. Retain policy-bearing code at its owner. Report
      source growth, deleted mechanisms and actual production callers separately
      from tests and docs.
- [ ] Close this phase with full consumer verification and measured workloads.
      Do not widen to global pure-2D partition/query replacement merely because
      the local candidate-index experiment wins.

## Phase 2: Conditional Dense Issuer And External Store Decision

Deferred: no sparse-registration-history profile was selected in Phase 0.
Unchecked items require a separately justified reopening of issuer/store work.

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
      Preserve effective body/solver order, joint replay identity and all
      callback mutation semantics. Do not create a second permanent
      authoritative ordered body registry to disguise a failed replacement.
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
      and run `.assets/scripts/assert-full-coverage.ps1` per configuration: 100%
      reported lines, branches and executable methods by integer counts, with no
      new exclusions or uncovered guards.
- [ ] Verify package mode and, for coordinated unreleased changes, the intended
      source graph via `UseLocalLsfStack=true` plus global
      `DisableTransitiveProjectReferences=true`, serial `-m:1` builds and the
      existing local assembly identities. Carry source selection to generated
      benchmark children; inspect loaded binaries, child exit/results and all
      expected cases. Follow [CONTRIBUTING](../../../CONTRIBUTING.md).
- [ ] Run focused high-risk contracts before full suites: serialization and
      populate-existing continuation, reset/pool graph release, role
      transitions, wake/grounding, ID reuse/reentrancy, reordered removals,
      world/grid replacement and mixed slab controls. Preserve replay fixture
      hashes unless an explicitly reviewed semantic change requires otherwise;
      run existing native conformance captures/comparisons for supported
      Windows/Linux x64/ARM64 lanes rather than inferring portability from
      repeated local runs.
- [ ] Preserve existing benchmark README allocation smoke targets and relevant
      simulation, query, partition, mixed, diagnostics and replay gates. Report
      startup separately from warm cost; measure complete
      `Simulate + LateSimulate` and scenario frames including scheduled queries,
      commands and churn. Include retained-memory plateaus, allocation spikes,
      practical frame latency and constrained/connected-island adversaries.
- [ ] Compare balanced independent baseline/candidate runs with equivalent
      inputs, order and physical outputs. Retain adverse/inconclusive results;
      kernel gains or `0 B/op` alone do not establish full-step improvement. Do
      not invent speedup thresholds or a fixed-Hz budget. A measured regression
      introduced here must be resolved here; only the user can accept a
      tradeoff. If no repeatable useful gain remains after publication, ordering
      and lifecycle costs, remove the candidate and record retention.
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

### Phase 0 Decision: Retain The Planar Sweep

The bounded production experiment passed its physical and ordering checks but
failed the no-regression gate. The BVH improved larger repeated-collision
frames, but consistently slowed the one-pair frame, added a first-query
allocation in the evolving strategy workload, and retained more backing storage.
These are tradeoffs, not a universally beneficial replacement. No tradeoff was
accepted. The original `DynamicCcdCandidateIndex2D` source was restored
byte-for-byte.

Phase 0 is closed with a retain decision. Phase 1 is not selected. Phase 2's
dense issuer/store work remains deferred pending its own sparse-history profile;
this experiment establishes no body-registration benefit. Spatial hash and
quadtree alternatives remain deferred without a measured workload justification.
No provider, dependency, CI, public API, serializer or maintained unit-test
change is retained. The useful addition is the six-case
[`PlanarCcdFrameBenchmarks`](../../../tests/Gravitas.Benchmarks/Physics2D/PlanarCcdFrameBenchmarks.cs)
and its benchmark README guidance.

#### Frozen Inputs And Removed Prototype

The execution baseline was clean Gravitas
`6311213b133fd74d93cac1ed5ba2f8bb9c8bc537`. All siblings were clean:

| Repository       | Source revision                            |
| ---------------- | ------------------------------------------ |
| FixedMathSharp   | `8fdf889322f1bb82501c2afec63d6bb2f304ace2` |
| SwiftCollections | `7c2fa696f10d88194f5a895aaafd7debd8e27574` |
| GridForge        | `b513d37f52abbb75b228baf521ef6144c6545810` |
| Chronicler       | `0281469ecc40e65a05867f473bc10f030e7f69f8` |

Runs used Windows x64, Intel Core i7-9700K (eight cores), SDK `10.0.302`,
runtime `10.0.10`, Release and explicit local source identities. Builds used
`-p:UseLocalLsfStack=true -p:DisableTransitiveProjectReferences=true -m:1`. No
package availability or released-package compatibility result is implied.

Only the existing planar owner was prototyped. Reconstruct it from the baseline
by replacing its entry list, update map, extent metadata and heap sort with
`SwiftFixedBVH2D<int>`:

- Reserve `2 * capacity` node slots for an explicit positive capacity, otherwise
  start with one node slot. Keep the existing update-mode flag and bounds
  helpers.
- `Clear` calls provider `Clear`; unique registration IDs and endpoint bounds go
  directly through `Insert`. `AddOrUpdate` keeps its update-mode guard, and
  `Remove` remains unavailable when updates are disabled.
- Map `(MinX, MinZ)`/`(MaxX, MaxZ)` to native 2D endpoints. Never read `Size` or
  `Area`. The old preparation `Sort()` is temporarily a no-op.
- `Query` clears the caller's output, invokes provider `Query` with its retained
  traversal scratch, then uses a struct comparer and `SortInPlace` to restore
  `(MinX, MinZ, MaxX, MaxZ, DynamicId)`. The comparer reads bounds through
  `NodePool[FindEntry(id)]` only during mutation-free sorting.
- Leave service base/dirty concatenation, lifetimes, mixed indexing, physical
  reducers and query publication unchanged. No backend selector or parallel
  permanent index was added.

The removed prototype reduced this owner from 315 to 97 lines, introduced no
runtime file, and served both existing consumers. Its source-file SHA-256 was
`0E2C44BE55B1082839DD917168D88C4425A0C9F0C92DAD86E0827EE886540477`. Frozen
Release `Gravitas.dll` hashes were:

- Baseline: `411497AA4672D752CC5ED8E889628CDCEF9045C844223D7A47343BE10DB47283`.
- Candidate: `6F7F54E5C0FEDE0989EB9325CFE505A10912F9F0113233590639D84228A855C3`.

The shared provider binaries also matched across hosts: SwiftCollections
`2FF7A7D889E9C0E7064936EDD6F7431228574293A35C4A7EB3D1B4FAE906DF9E` and its
FixedMathSharp companion
`CD4BF5F653B5EDCD0D97C43BDD6D0F86B09A3AC3F84E356F5B05B2A7505F2DCD`. Separate
copied hosts and a disposable binary-reference driver verified loaded paths and
hashes; no generated benchmark child rebuilt the acceptance payload.

#### Physical And Ordering Evidence

The maintained frame fixture uses equal-mass, radius-`1/2` circles initially at
`center +/- 2`, opposing speeds `4`, zero restitution/friction/gravity/drag, and
a one-second step. Independent pairs have spacing `16`. The analytic contact
time is `3/8` second; centers stop at `center +/- 1/2` with zero velocity. The
designated ray from `(-4, 0)` to `(4, 0)` hits at distances `3` and `4`, with
points `(-1, 0)` and `(0, 0)` in collider order; overlap returns both bodies.
All six cases passed 24 repeated physical checks in setup and checks around
timing. Each invocation resets motion, applies impulses, runs both host phases,
queries and consumes results. One-pair layouts share actor geometry but have
different grid extents; they are compared only within the same parameter case.
Query count stays fixed as population grows. This is a commanded re-collision
scenario, not an evolving-world or persistent-index throughput claim.

A disposable endpoint oracle used seed `102491`, 64 unique IDs, clustered-X and
negative coordinates, touching/zero-size bounds, one full-domain X span and both
extreme endpoints, four remove/reinsert cycles, clear and reuse. It checked 516
ordered queries per implementation against inclusive endpoint intersection and
an independent full bounds-tuple/ID sort, including output clearing. All passed.
The six raw frame checksums also matched between implementations.

Candidate focused Release runs passed 301 tests covering the direct 2D/3D index,
2D CCD/handoff/dirty trajectory/callback reuse, reset graph release, world/grid
query refresh and replay/restore. A further 592 mixed-query/CCD, bounded planar
support and scratch controls passed, followed by eight exact-distance/closing-
speed/ID arbitration tests against the frozen candidate binary. Existing
reviewed shared replay fixtures kept their hashes and ordered observations.
Contact events and lifecycle behavior are evidenced by these product tests, not
by the frame fixture's scalar checksum. Independent subagent reviews found no
correctness blocker in the first shared slice or the analytic fixture.

#### Timing, Allocation And Storage

Baseline sampling reached the actual index query, sort and update call chain;
sorting was visible in the commanded collision workload. Trace collection was
separate from acceptance timings. Initial unpinned runs were noisy, including a
32-pair uniform candidate process mean of `2.408 ms` versus baseline process
means `0.860-1.102 ms`. They were retained as inconclusive evidence, not
silently treated as a zero-cost result.

The controlled repeat used CPU affinity mask `4`, process priority High,
`DOTNET_TieredCompilation=0` and `DOTNET_gcConcurrent=0` for both variants. Run
order was baseline/candidate, candidate/baseline, baseline/candidate: three
independent processes per implementation. Each case warmed 128 invocations,
calibrated a power-of-two batch to at least 200 ms, then collected seven
batches. GC ran before each measured batch. Reported ranges below are the
minimum and maximum of the three process means, in microseconds per complete
scenario frame; percentages compare the mean of those process means. All
samples, including adverse ones, were included. These are fixture costs, not
game-frame percentiles or a recommended fixed-step budget.

| Dynamic pairs / layout | Sweep us/frame      | BVH us/frame        | Mean change |
| ---------------------- | ------------------- | ------------------- | ----------- |
| 1 / Uniform            | 39.270-39.426       | 40.151-40.570       | +2.6%       |
| 1 / ClusteredX         | 39.418-39.560       | 40.111-40.389       | +2.0%       |
| 32 / Uniform           | 825.798-857.639     | 760.211-765.290     | -9.3%       |
| 32 / ClusteredX        | 864.948-882.964     | 750.941-756.411     | -13.7%      |
| 128 / Uniform          | 6,467.476-6,823.301 | 3,794.708-3,887.415 | -41.6%      |
| 128 / ClusteredX       | 6,727.863-7,000.153 | 3,943.122-4,029.161 | -41.6%      |

All controlled repeated-collision batches reported zero current-thread managed
allocation. Existing `TranslateUniqueWidest2D`/`ThenQuery2D` controls also
remained allocation-free but slowed: at 64 entries, update means were `30-33 ns`
versus `74-89 ns`; at 16,384 entries, update-plus-query means were `62-70 ns`
versus `276-371 ns`. These preliminary kernel results do not establish a
full-step loss on their own.

Three balanced runs of each frozen host's `verify-strategy-2d`, with
`GRAVITAS_STRATEGY_FRAME_CSV=1`, matched all 96 frames and the same combined
state, query and work trace: `ed0ba077af1d3a4952fbecac69fbcf7c`. Connected
chains first completed on frame 67. Stage timings overlapped and were
inconclusive; no general strategy speedup or slowdown is claimed. Allocation was
repeatable: quiet steps allocated zero in both, sparse-stage physics totals were
`5,728` versus `7,936 B`, and crowded totals were `35,456 B` in both. The sole
allocation difference was projectile frame 44: `5,672` versus `7,880 B`, an
introduced `2,208 B` burst.

A separate first-query control prefilled 256 base-index entries with bounds
`(8*i, 0)` to `(8*i+1, 1)` and preallocated one-hit output. It warmed generic
machinery on another index before measuring the fresh index's first query. The
sweep allocated `0 B`; the BVH allocated `2,248 B`; the immediate repeat
allocated zero in both. The provider lazily creates per-index/thread traversal
scratch and requests 512 integers for this tree. This supports lazy first-query
scratch as the strategy burst mechanism; the probe's extra 40 bytes mean it is
not an exact allocation-stack attribution of that frame.

Reflection counted reachable backing-array element bytes, excluding object/array
headers and other object overhead. For that 256-entry base index, the sweep held
`10,240 B`; BVH held `40,992 B` before first query and `43,056 B` after it. A
64-entry update-capable churn/reset control held `6,656` versus `10,792 B`,
retaining the same storage after clear/reuse. These are bounded storage
snapshots, not full retained-heap or long-session plateau claims.

#### Closeout, Limits And Reopening

The prototype was removed after the no-go decision, before expanding the
persistent projected-circle experiment, long-session memory investigation or
dense issuer work. The first-query burst and one-pair regression are resolved in
this change by retaining the original owner, not by deferring an adopted
regression. No physical defect or provider bug was established.

Builds/tests/smoke checks and their final counts are recorded below. Fresh
whole- library coverage, the eight native replay lanes, package mode, sustained
persistent-BVH permutations and further issuer/store validation are not passing
results from this experiment. Adoption-only gates were not reached after
rejection. No upstream edit occurred, so there is no CI dependency-pin change.

Essential inputs, prototype recipe, source/binary identities, outputs, timing
protocol and adverse results are recorded here. Supplemental logs, copied hosts,
trace, disposable oracle/driver and removed patch are under
`artifacts/swiftcollections-v9-phase0`; this ignored directory is not required
to understand the decision. Reproduce the maintained scenario with the benchmark
README command; the temporary driver is not maintained test infrastructure.

Reopen when a profiled intended workload makes the dense dynamic-contact saving
material, and a candidate preserves the small-scene baseline, avoids the added
first-query burst and justifies retained storage without permanent competing
indexes. Persistent projected-circle adoption still needs its own update,
world-change and sustained spatial-permutation evidence. Body issuer/store
adoption needs independent sparse-history profiling and its full identity/order
proof; larger CCD wins do not justify it.

#### Final Verification

| Check                                   | Result                                                                                                                |
| --------------------------------------- | --------------------------------------------------------------------------------------------------------------------- |
| Baseline Release suite before prototype | 8,572 passed; zero failed/skipped                                                                                     |
| Candidate focused controls              | 301 + 592 + 8 passed; zero failed/skipped                                                                             |
| Restored Release library                | Both `netstandard2.1` and `net10.0`; zero build warnings/errors                                                       |
| Restored Release suite                  | 8,572 passed; zero failed/skipped                                                                                     |
| ReleaseLean library                     | Both target frameworks; zero build warnings/errors                                                                    |
| ReleaseLean suite                       | 8,495 passed; zero failed/skipped                                                                                     |
| Retained frame benchmark                | Both profiles built; all six in-process Dry cases executed in each, with statistics and measurement records inspected |
| Documentation/style                     | Archive links and whitespace checked; new C# fixture passes the targeted formatter check                              |

Build/test commands used the explicit source flags above. Both final test runs
used the existing unit-test project and
`RunConfiguration.TreatNoTestsAsError=true`. The benchmark smoke command was
`planar-ccd-frame --filter '*' -j Dry -i --exporters json`, with separate
artifact paths per profile. Dry results verify execution, not throughput or warm
allocation; the controlled process comparison provides that evidence.

Restoring the source snapshot preserved its older timestamp. An initial
incremental Release closeout build therefore reused the candidate DLL; its hash
exposed the mismatch. That smoke was not accepted as restored-source validation.
A forced rebuild and fresh smoke/test runs corrected it. The final Release DLL
again matches the frozen baseline hash above. The restored source has no diff;
all siblings and dependency pins are unchanged. Fresh coverage, package-mode and
full native-matrix gates were not run for this rejected prototype. Final changes
are the retained benchmark and documentation, unstaged and uncommitted.

### 2026-11-25 Bounded Single-Index Follow-Up

The user authorized a bounded attempt to remove first-query allocation and
small-scene overhead without retaining two index implementations, and allowed
modest measured costs when justified by heavier workloads. This supersedes the
original blanket no-regression decision for this follow-up only.

**Decision:** keep one native BVH backend for the existing planar CCD and
projected-circle owners, with provider-owned reserved traversal storage. The
first-query allocation is removed. A small-scene cost remains; it is explicitly
accepted here under the revised criterion, alongside the persistent-query and
memory costs below. Larger CCD gains remain material. This is local source
validation, not a package-release or cross-platform completion claim.

#### Smallest Change And Causal Proof

SwiftCollections' `SwiftBVH<TKey,TVolume>` now owns one `int[]` traversal
buffer, reserved in the constructor and whenever node capacity grows. A valid
binary tree with N leaves owns `2N-1` live nodes, and pending traversal subtrees
number at most N. Thus node capacity C needs `(C+1)/2` integers. The traversal
core, visit order and `TryQuery` admission contract are unchanged. Owner-held
storage also works after a synchronized transfer to another thread. Result
callbacks must not reenter the index; XML now makes that existing restriction
explicit. The obsolete `QueryTraversalScratch` and its helper-specific test were
removed. No new public API, warming query, service reservation layer or
threshold backend was introduced. BVHs that are never queried now pay eager
traversal-array cost.

Gravitas replaces the sweep storage and its maps/extent bookkeeping with the
provider directly. It still restores the full bounds/ID order and preserves
base-then-dirty lifetime filtering. The empty preparation `Sort` shim and its
pure-2D caller/test calls were removed; mixed/3D preparation remains intact. The
planar owner is 91 lines rather than 315. There is no sweep fallback or parallel
comparison index in retained runtime code.

Before the fix, all four new Gravitas first-use allocation cases failed at 672
bytes, and all six provider boundary cases failed, including 2,208 bytes in
growth/thread-handoff cases. After the fix they pass. They warm generic code on
a different owner, then measure the fresh owner's first normal query with
preallocated sparse/all-hit output, including growth and clear/reuse. Provider
tests also cover a synchronized new-thread query. Removing the obsolete helper
test exposed the existing stack `Clear` method's lost indirect coverage; a
direct storage-retention/push/pop behavior test restored it without exclusions.

One additional bounded trial removed `AllocateNode`'s redundant reset, after
independent review of constructor/growth/free-list/clear invariants. In a
two-process-per-variant ABBA control, one-pair uniform means were
`40.209-40.577 us` with the reset versus `40.036-40.811 us` without it;
clustered means were `40.510-41.149` versus `39.939-40.817 us`. Small-scene
improvement was not consistent, so that trial was reverted. No further hot-path
tuning was selected.

#### Balanced Results And Remaining Costs

The six maintained frame cases and original source revisions/settings were
unchanged. Fresh baseline/candidate processes used the prior controlled
protocol: CPU mask 4, High priority, tiering/background GC disabled, 128 warm
calls, power-of-two batches of at least 200 ms, seven samples per case, and
three independent processes per implementation in ABBAAB order. Ranges are
process means in microseconds; percentages compare their balanced means.
Preliminary trial results remain separate from the selected-source evidence.

| Complete scenario frame | Sweep us            | Revised BVH us      | Mean change |
| ----------------------- | ------------------- | ------------------- | ----------- |
| 1 pair / Uniform        | 39.369-39.632       | 39.477-40.975       | +1.7%       |
| 1 pair / ClusteredX     | 39.851-40.082       | 39.945-40.657       | +0.7%       |
| 32 pairs / Uniform      | 833.302-870.511     | 747.966-783.003     | -10.2%      |
| 32 pairs / ClusteredX   | 844.890-848.911     | 742.363-756.859     | -11.7%      |
| 128 pairs / Uniform     | 6,231.806-6,381.961 | 3,735.444-3,845.892 | -39.9%      |
| 128 pairs / ClusteredX  | 6,572.051-6,669.100 | 3,908.543-3,959.471 | -40.5%      |

All repeated-collision allocation samples remain zero. Small-case process ranges
overlap: the mean deltas do not establish a repeatable small-scene regression,
zero overhead or a precise improvement over the earlier BVH's 2-3% result from a
different run.

The persistent projected consumer was separately exercised through actual
`Query3D.OverlapCircleAll` calls against 32/128 dynamic half-unit spheres in
fixed lanes at Z=`8*lane`, with physical Y cycling through -1/0/1. Four fixed
query lanes use X=2, Y=`Fixed64.MaxValue`, radius 2 and one-hit output storage.
Stationary cases preserve identities. Permuted cases alternate identity and
multiplier-5 affine lane assignments, with a rotating offset modulo population,
using `ResetPosition` commands before queries. After 256 evolving warm frames,
every query still verifies the inverse-permutation collider, distance `3/2` and
finite sphere witness. Timings include commands, refresh, queries and
assertions; they are validation-inclusive query scenarios, not host-step
throughput.

| Projected scenario          | Sweep us            | Revised BVH us      | Mean change |
| --------------------------- | ------------------- | ------------------- | ----------- |
| 32 / stationary             | 28.409-28.962       | 28.996-29.406       | +1.7%       |
| 32 / sustained permutation  | 467.287-478.986     | 474.108-481.444     | +0.9%       |
| 128 / stationary            | 29.398-30.579       | 32.139-32.832       | +8.8%       |
| 128 / sustained permutation | 2,040.744-2,119.651 | 2,084.667-2,268.658 | +4.3%       |

The stationary 128-body cost is roughly 2-3 us per four-query batch; sustained
relocation costs more in absolute time. These regressions and eager/retained
memory remain part of the selected tradeoff. The shared owner is justified here
by the measured dynamic-contact workload, not a universal query improvement.
Allocation vectors matched between implementations in every paired projected
run. The 128-body relocation case has the same tapering pre-existing allocation
in both variants; it is not claimed allocation-free or a long-session plateau.

The fresh 256-entry first-query control now reports `0 B`, including its repeat.
Backing-array element bytes are `42,016` both before and after query, versus
`10,240` for the sweep and the old BVH's `40,992` before / `43,056` after query.
The array reserves about 1 KiB earlier while reducing warmed scratch versus the
old BVH. It still costs about 31 KiB more than the sweep for this base index.
The 64-entry churn/clear control holds `10,528` bytes, versus the sweep's
`6,656`. These exclude object headers and are bounded snapshots, not heap-budget
claims.

Three balanced runs of each frozen host's evolving 256-actor strategy matched
all 96 frames and trace `ed0ba077af1d3a4952fbecac69fbcf7c`. Every sampled
frame's physics allocation count now matches the sweep, including projectile
frame 44: `5,672 B` in both, removing the previous extra `2,208 B`. Stage totals
remain zero quiet, `5,728 B` sparse and `35,456 B` crowded. This demonstrates
removal of the added burst in the real lifecycle, not merely a synthetic
warm-query result. The independent endpoint oracle again passed 516 ordered
queries and all six analytic frame proofs matched. Native cross-platform
conformance remains unrun.

#### Selected Source And Verification

The original clean baseline and four sibling revisions above are unchanged; the
follow-up adds the coordinated SwiftCollections patch. The measured selected
hosts have these SHA-256 values:

| Assembly                        | Measured candidate                                                 |
| ------------------------------- | ------------------------------------------------------------------ |
| Gravitas                        | `30502FFE79465F56711ADB618E3D53E943C9395467FE2FEC56C5B51728404256` |
| SwiftCollections                | `C4DAF26A523FDD9141DDD73633F90F3F812695F894B7323F3E5D18FB400337A8` |
| SwiftCollections.FixedMathSharp | `07D9EE63489D9774C2A22A32E22A6D70F0A3930461AF1BF8980F3BD0EEE4CBD2` |

Only the array-reservation variant is retained; the reset-deletion trial was
restored before final builds. Timing hosts were copied before coverage runs, and
measured code did not change during comparisons. Independent reviews checked the
scratch bound, synchronization, order preservation, regression tests, deletion
of replaced machinery and physical/measurement limits. Supplemental captures and
disposable probes are in this repository's
`artifacts/swiftcollections-v9-followup`; provider results are under
SwiftCollections `artifacts/bvh-query-scratch-followup-final`.

All final local gates passed, with no new coverage exclusions:

| Assembly / profile                            | Tests passed | Lines covered / reported | Branches covered / reported | Executable methods covered / reported |
| --------------------------------------------- | ------------ | ------------------------ | --------------------------- | ------------------------------------- |
| SwiftCollections / Release                    | 2,432        | 6,627 / 6,627            | 3,128 / 3,128               | 1,473 / 1,473                         |
| SwiftCollections / ReleaseLean                | 2,277        | 6,400 / 6,400            | 3,028 / 3,028               | 1,423 / 1,423                         |
| SwiftCollections.FixedMathSharp / Release     | 100          | 205 / 205                | 86 / 86                     | 67 / 67                               |
| SwiftCollections.FixedMathSharp / ReleaseLean | 100          | 205 / 205                | 86 / 86                     | 67 / 67                               |
| Gravitas / Release                            | 8,576        | 54,684 / 54,684          | 22,767 / 22,767             | 5,778 / 5,778                         |
| Gravitas / ReleaseLean                        | 8,499        | 54,682 / 54,682          | 22,767 / 22,767             | 5,777 / 5,777                         |

Both runtime target frameworks built in both profiles. Source builds retained
the explicit local-stack flags, serial execution and existing identities.
Coverage used each project's existing runsettings, fresh per-profile results and
both repositories' strict assertion scripts. No tests failed or skipped in these
final runs. The six retained frame cases also executed in each profile's
in-process Dry smoke, with all result records inspected; smoke timing is not
acceptance throughput. Targeted C# formatting, changed Markdown links and
whitespace checks passed.

Provider solution builds retain pre-existing analyzer warnings: xUnit2013 at
`SwiftSlotMap.Tests.cs:138` in both profiles and xUnit1026 at
`SwiftObservableDictionary.Tests.cs:279` in Lean. Those files were unchanged
from provider HEAD. Gravitas runtime builds had no warnings/errors. Independent
review found no remaining source correctness blocker; its dead-code and dangling
link findings were resolved. GridForge acquired documentation-only edits from
other work during execution; they were not edited or staged here.

Gravitas now depends on the coordinated SwiftCollections scratch patch even
though public signatures are unchanged. After that change lands, refresh the
actual Gravitas build/test sibling pin to its immutable commit. No pin, version
or CI edits belong to this working change. Package-mode validation and the full
native replay matrix are not results of this bounded follow-up; do not infer
release readiness from the local evidence. All changes remain unstaged and
uncommitted for review in their owning repositories.

### Final Coordinated Review

Three fresh subagents reviewed the complete Gravitas/SwiftCollections changes,
including staged changes and new files. The reviews covered scratch capacity and
lifetime, connected CCD/projected-query consumers, and the actual benchmark,
binary and coverage evidence. No runtime correctness defect was found.

The stale-scratch concern prompted one concrete test/clarity improvement.
`Query` starts a local count at zero, writes the current root and only reads the
active `[0,count)` prefix. Every push overwrites a slot before it becomes live.
Old integer indices may remain outside that prefix after completion or failure,
but cannot resume in the next query and retain no managed objects. The existing
external synchronization/no-reentry contract still applies. Clearing the array
would add work without fixing a stale-read defect, so no clearing was added.

An invariant comment documents this at the traversal loop. The new regression
`Query_AfterResultCallbackThrows_DiscardsPendingTraversalAcrossReuse` uses four
overlapping leaves and throws on the second result while nodes remain pending.
It checks the partial first result, exact normal retry, clear/repopulate with a
distant miss/hit, and remove/reinsert with a new key. The independent scratch
reviewer checked the actual test and found the review gap resolved.

Fresh provider Release/Lean suites and strict coverage pass with the updated
2,432/2,277 core counts above and 100 companion cases each. Fresh Gravitas
consumer controls pass 905 Release and 901 Lean cases against the final
provider. Gravitas runtime source is unchanged; the preceding full 8,576/8,499
suites and coverage remain applicable. No executable runtime code changed during
this review, so throughput comparisons were not repeated. Supplemental review
logs are under Gravitas `artifacts/swiftcollections-v9-final-review` and
SwiftCollections `artifacts/bvh-final-review`.

The comment changes the provider DLL/PDB identity despite unchanged runtime
logic. The measured hashes above identify the earlier timing payloads; the final
Release SwiftCollections DLL is
`9595C062341E3C06F14691414907EBFD0C4A5FF49D607C613728BA6DD0EB4F4C`. Gravitas and
the fixed-point companion retain their recorded candidate hashes. The
SwiftCollections files staged before review were returned to unstaged as
requested. Both repositories' staging areas are empty, and nothing was
committed.
