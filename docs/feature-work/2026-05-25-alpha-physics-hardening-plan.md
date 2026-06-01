# Alpha Physics Hardening Action Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the post-backlog prototype into an alpha-ready deterministic physics foundation with stronger simulation contracts, solver behavior, mesh/query policy, dimensional modeling, and replay confidence.

**Architecture:** Work from simulation invariants outward. First lock down frame ordering, replay baselines, and measurement gates; then simplify ownership seams; then harden contact generation, response, CCD, mesh policy, queries, dimensions, serialization, and diagnostics. Each phase should leave source comments clean, `docs/wiki/` current, focused tests in place, and benchmark evidence for any hot-path or algorithmic change.

**Tech Stack:** C# 11, `FixedMathSharp`, `SwiftCollections`, `SwiftCollections.FixedMathSharp`, `GridForge`, `Chronicler.Core`, xUnit v3, BenchmarkDotNet.

---

## Source Context Reviewed

- `docs/feature-work/done/2026-05-23-physics-hardening-backlog-plan.md`
- `docs/wiki/OVERVIEW.md`
- `docs/wiki/RUNTIME_ARCHITECTURE.md`
- `docs/wiki/COLLISION_PIPELINE.md`
- `docs/wiki/QUERY_SERVICES.md`
- `docs/wiki/HOST_INTEGRATION.md`
- `docs/wiki/DIAGNOSTICS.md`
- `docs/wiki/DIMENSIONS.md`
- `src/Gravitas/Runtime/GravitasWorldContext.cs`
- `src/Gravitas/Core/GravitasPhysicsService.cs`
- `src/Gravitas/Core/StiffBody.cs`
- `src/Gravitas/Colliders/LSCollider.cs`
- `src/Gravitas/Colliders/Primitives`
- `src/Gravitas/CollisionHandling`
- `src/Gravitas/Queries`
- `src/Gravitas/Partitions/PhysicsPartition.cs`
- `tests/Gravitas.Tests`
- `tests/Gravitas.Benchmarks`

## Completed Plan Audit

The prior hardening backlog has no orphaned phase tasks. The only unchecked
items are the reusable verification-gate checklist at the end of the document,
not phase-specific work. The plan has been marked done and moved under
`docs/feature-work/done/`.

Meaningful deferred work captured from that plan and the wiki:

| Source | Deferred work | Destination |
| --- | --- | --- |
| `docs/wiki/COLLISION_PIPELINE.md` | Contact manifolds, friction impulses, continuous collision detection, warm starting, island solving, mixed 2D/3D exchange rules. | Phases 3-6, 9-10 |
| `docs/wiki/COLLISION_PIPELINE.md` | Dynamic mesh behavior, arbitrary mesh contact manifolds, swept mesh queries. | Phases 3, 6, 7 |
| `docs/wiki/QUERY_SERVICES.md` | Swept mesh support policy, shape-specific query tests, query reentrancy/job state. | Phases 6 and 8 |
| `docs/wiki/OVERVIEW.md` | First-class pure 2D support and mixed 2D/3D interaction needed explicit contracts instead of accidental flattened-3D behavior. | Phases 9-10 |
| `docs/wiki/DIAGNOSTICS.md` | Generic diagnostic payloads may need richer typed events; host adapters remain outside core. | Phase 12 |
| `docs/wiki/HOST_INTEGRATION.md` | Serialization is populate-existing-shell behavior and remains experimental. | Phase 11 |
| Prior plan recommendations | `LSCollider` owns too many responsibilities. | Phase 2 |
| Prior plan recommendations | Consume the next fixed GridForge package and revisit partition allocation without local project-link scaffolding. | Phase 0 and Phase 8 |
| Prior plan recommendations | Prefer FixedMathSharp geometry and SwiftCollections fixed-query structures before adding local spatial math. | All algorithm phases |
| Phase 7B implementation review | Host/offline decomposed convex pieces and Gravitas-owned convex decomposition are optional mesh-policy backlog items, not Phase 7B or Phase 7C blockers. The local-BVH triangle path is the alpha runtime baseline unless benchmarks or solver-quality work prove a need to revisit. | Mesh Decomposition Backlog |

## Downstream Library And Package Notes

- Keep the temporary local GridForge project references only while hardening
  needs them. Once the GridForge partition-provider retention fix is packaged,
  validate against the package and remove local-link scaffolding from Gravitas
  project files.
- Review `../FixedMathSharp/src/FixedMathSharp/Geometry` before adding local
  ray, plane, frustum, bounds, or matrix helpers. `FixedRay`, `FixedPlane`,
  typed containment/intersection APIs, `BoundingFrustum`, and expanded
  `Fixed4x4` should be the default comparison point.
- Review `../SwiftCollections/src/SwiftCollections.FixedMathSharp` before
  writing new broad-phase/query structures. `SwiftFixedBVH<T>`,
  `SwiftFixedOctree<T>`, `SwiftFixedSpatialHash<T>`, and `FixedBoundVolume`
  are preferred starting points.
- `SwiftSparseSet` is the current partition membership structure for dynamic
  and static collider IDs. Revisit only if partition iteration/removal
  benchmarks show a better deterministic membership layout.

## Phase 0: Baseline, Package Hygiene, And Risk Register

**Purpose:** Establish the next hardening baseline before changing behavior.

**Files:**

- Modify: `docs/feature-work/2026-05-25-alpha-physics-hardening-plan.md`
- Modify: `tests/Gravitas.Benchmarks/README.md`
- Potentially modify: `src/Gravitas/Gravitas.csproj`
- Potentially modify: `tests/Gravitas.Tests/Gravitas.Tests.csproj`
- Potentially modify: `tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj`

**Tasks:**

- [x] Run the full `Release` and `ReleaseLean` build/test gate on the current baseline.
- [x] Run short smoke benchmarks for `simulation-allocation`, `query-service`, `collision-detection`, `collision-response`, `partition-culling`, and `diagnostics`.
- [x] Record which benchmark selections report managed allocation and which are expected to stay at `0 B/op`.
- [x] Inspect whether the temporary local GridForge project references are still needed. If the fixed GridForge package is available, validate against the package and remove local-link scaffolding.
- [x] Create a compact risk register in this plan with any newly observed failing tests, benchmark regressions, stale docs, or package-version blockers.
- [x] Keep `docs/wiki/` unchanged unless the baseline reveals stale claims.

**Phase 0 Status - 2026-05-26**

- Package hygiene validated against `FixedMathSharp` `4.0.0`, `SwiftCollections`
  `4.1.0`, `SwiftCollections.FixedMathSharp` `4.1.0`, and `GridForge`
  `6.0.5` package references. No temporary local GridForge project references
  remain in the active project files.
- Replaced partition membership from `SwiftSparseMap<byte>` to
  `SwiftSparseSet` now that SwiftCollections exposes the dedicated sparse-set
  primitive. This removes the dummy value payload while preserving dense ID
  iteration for partitions and query services.
- Added assembly-level serial execution for the xUnit project. Allocation and
  replay guardrails use thread-local allocation counters and shared pool warmup
  behavior, so they should not run concurrently with unrelated tests.
- `Release` gate: `dotnet build Gravitas.slnx --configuration Release` and
  `dotnet test Gravitas.slnx --configuration Release --no-build` passed with
  134 tests.
- `ReleaseLean` gate: `dotnet build Gravitas.slnx --configuration ReleaseLean`
  and `dotnet test Gravitas.slnx --configuration ReleaseLean --no-build`
  passed with 134 tests.
- Focused sparse-set validation:
  `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~PhysicsPartitionPerformanceShapeTests|FullyQualifiedName~QueryService|FullyQualifiedName~Raycast|FullyQualifiedName~Circle"`
  passed with 21 tests.
- Benchmark smoke command:
  `dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -- simulation-allocation query-service collision-detection collision-response partition-culling diagnostics --filter "*" -j Short -i --exporters json`
  completed 24 benchmarks. BenchmarkDotNet could not raise process priority in
  this sandbox, so treat timings as local smoke evidence only.

**Phase 0 Allocation Notes**

| Selection | Current smoke allocation note |
| --- | --- |
| `simulation-allocation` | Most measured methods reported no managed allocation; `GroundingSweptSphereProbeOnly` reported `1 B/op` in the short job and should be watched with explicit allocation tests if it becomes repeatable. |
| `query-service` | Most query paths reported no managed allocation; `RaycastAcrossTwoOverlappingContexts` reported `1 B/op` in the short job and should be watched with explicit allocation tests if it becomes repeatable. |
| `collision-detection` | Most methods reported no managed allocation; `CheckCuboidCuboidSatPairs` reported `1 B/op` in the short job while focused SAT allocation tests remain the stronger guardrail. |
| `collision-response` | `CalculateImpulseForPreparedPairs` reported `1.27 KB/op`; keep this as a Phase 4 solver allocation target. |
| `partition-culling` | Direct membership churn and culled-pair recheck reported no managed allocation; teleported repartition reported `1 B/op` in the short job and should be rechecked after broader partition scalability work. |
| `diagnostics` | Disabled and enabled event/debug-draw paths reported no managed allocation. |

**Phase 0 Risk Register**

- `dotnet clean` against the `.slnx` and project files returned a failure exit
  code without MSBuild errors in this environment. Sequential `dotnet build`
  commands produced clean artifacts; prefer build/test gates over relying on
  `dotnet clean` until this is understood.
- Do not run `Release` and `ReleaseLean` builds/tests concurrently in the same
  workspace. Parallel configuration builds can cross-contaminate generated
  `obj` state and surface bogus MemoryPack shim errors.
- `CollisionResponse` still has a measurable allocation baseline in the short
  benchmark. Track this in the solver hardening phase instead of hiding it with
  a weak benchmark.

## Phase 1: Simulation Phase Order And Replay Contract

**Purpose:** Decide and test the authoritative order of command application, partition refresh, collision distribution, response, integration, grounding, and visualization before the solver grows more complex.

**Files:**

- Modify: `src/Gravitas/Runtime/GravitasWorldContext.cs`
- Modify: `src/Gravitas/Core/GravitasPhysicsService.cs`
- Modify: `src/Gravitas/Core/StiffBody.cs`
- Modify: `tests/Gravitas.Tests/Runtime`
- Modify: `tests/Gravitas.Tests/Core`
- Modify: `docs/wiki/RUNTIME_ARCHITECTURE.md`
- Modify: `docs/wiki/HOST_INTEGRATION.md`

**Tasks:**

- [x] Add deterministic replay tests that run the same context setup, command sequence, and frame count twice and compare body/collider state.
- [x] Add tests that pin when host transform teleports, force commands, kinematic reads, collision distribution, response, body integration, grounding, and visualization state are allowed to mutate authoritative data.
- [x] Re-evaluate the current collide-in-`Simulate`, integrate-in-`LateSimulate` order against desired lockstep semantics. Preserve it only if tests and docs make the behavior intentional.
- [x] If the order changes, split the work into a focused migration phase before touching the response solver. No migration phase was needed because the current order was preserved and pinned.
- [x] Document the final phase order, replay expectations, and non-authoritative visualization boundary.

**Phase 1 Status - 2026-05-26**

- Added `GravitasSimulationPhaseOrderTests` to pin the current lockstep phase
  contract without changing production order.
- Preserved the current alpha order intentionally: host commands before
  `Simulate()`, dynamic collider refresh and collision distribution in
  `Simulate()`, active-pair processing plus body force integration/grounding in
  `LateSimulate()`, and presentation-only visualization afterward.
- Confirmed pre-`Simulate()` teleports can create same-frame contacts, while
  pre-`Simulate()` forces do not move bodies until `LateSimulate()`.
- Confirmed hooks observe built-in work after each phase and visualization does
  not mutate authoritative body position or velocity.
- Updated `docs/wiki/RUNTIME_ARCHITECTURE.md` and
  `docs/wiki/HOST_INTEGRATION.md` with the replay contract and visualization
  boundary.

## Phase 2: Collider, Body, And Hierarchy Ownership Cleanup

**Purpose:** Reduce `LSCollider` and `StiffBody` responsibility density before manifold and solver work depends on those seams.

**Files:**

- Modify: `src/Gravitas/Colliders/LSCollider.cs`
- Modify: `src/Gravitas/Core/StiffBody.cs`
- Already present: `src/Gravitas/Colliders/Support/ColliderRuntimeShapeState.cs`
- Create: `src/Gravitas/Colliders/Support/ColliderPartitionState.cs`
- Create: `src/Gravitas/Colliders/Support/ColliderQueryState.cs`
- Create: `src/Gravitas/Colliders/Support/ColliderPairState.cs`
- Create: `src/Gravitas/Colliders/Support/ColliderHierarchyState.cs`
- Modify: `tests/Gravitas.Tests/Colliders`
- Modify: `tests/Gravitas.Benchmarks/Colliders`
- Modify: `docs/wiki/RUNTIME_ARCHITECTURE.md`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`

**Tasks:**

- [x] Map `LSCollider` responsibilities into identity, host binding, runtime shape, partition state, query versioning, hierarchy filtering, and pair references.
- [x] Add regression tests around parent/child collision exclusion, collider deactivation cleanup, pair-holder cleanup, partition refresh, and query-version reset before moving state.
- [x] Extract only the state groups that reduce real complexity without creating indirection-heavy API bloat.
- [x] Preserve disabled/allocation hot paths for collider simulation and partition refresh.
- [x] Revisit parent/child metadata and document the engine-agnostic hierarchy rule that replaced Unity transform traversal.

**Phase 2 Status - 2026-05-26**

- Split `LSCollider` dense mutable state into focused internal helpers:
  `ColliderPartitionState`, `ColliderQueryState`, `ColliderPairState`, and
  `ColliderHierarchyState`, while keeping identity, host binding, shape API, and
  events on the collider itself.
- Kept pair containers lazy so colliders that never collide avoid upfront pair
  dictionary/holder set allocation. Existing partition and query hot paths still
  use caller-owned/context-owned state.
- Replaced implicit/unusable parent traversal with explicit
  `SetParent(LSCollider parent)` and `ClearParent()` binding. Parent-child and
  sibling filtering now depends on stored top-parent collider IDs, not host
  transform traversal.
- Added `ColliderOwnershipStateTests` for explicit hierarchy filtering,
  owned-pair cleanup, holder-side cleanup, static partition refresh, and query
  version reset.
- Fixed a holder-side deactivation bug found by the new tests: holder cleanup
  now enumerates the `SwiftHashSet<int>` instead of indexing it as though the
  hash-set key were a dense ordinal.
- Fixed parent deactivation cleanup so children drop stale parent IDs before the
  parent collider ID can be reused by an unrelated collider.
- Removed duplicate collider position/rotation dirty flags. Runtime-shape
  snapshot commits now drive broad-phase versioning, and collision pairs use
  broad-phase version changes as their authoritative movement/shape
  invalidation signal.
- Updated `docs/wiki/RUNTIME_ARCHITECTURE.md`,
  `docs/wiki/COLLISION_PIPELINE.md`, and `docs/wiki/HOST_INTEGRATION.md` with
  the new collider state split and explicit hierarchy contract.
- Verification: focused collider/partition/physics-service tests passed, full
  `Release` and `ReleaseLean` build/test gates passed with 145 tests, and short
  `partition-culling` plus `simulation-allocation` benchmark smoke completed.
  The only allocation reported in those selections was the existing
  `GroundingSweptSphereProbeOnly` `1 B/op` short-job artifact.

## Phase 3: Contact Manifold Data Model

**Purpose:** Replace single-contact response assumptions with deterministic manifold data that can support stacking, friction, warm starting, and mesh contacts.

**Files:**

- Remove: `src/Gravitas/CollisionHandling/Support/ContactPoint.cs`
- Create: `src/Gravitas/CollisionHandling/Contacts/ContactManifold.cs`
- Create: `src/Gravitas/CollisionHandling/Contacts/ManifoldContact.cs`
- Modify: `src/Gravitas/CollisionHandling/Pairs/CollisionPair.cs`
- Modify: `src/Gravitas/CollisionHandling/Detection/CollisionDetection.cs`
- Modify: `tests/Gravitas.Tests/CollisionHandling`
- Modify: `tests/Gravitas.Benchmarks/CollisionHandling`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`

**Tasks:**

- [x] Define deterministic manifold identity, contact ordering, maximum contact count, point lifetime, and reduction policy.
- [x] Add tests for zero-depth touching contacts, stacked contacts, edge/face contacts, reversed pair ordering, and stable contact order across repeated runs.
- [x] Start with primitive pairs where manifold candidates are easiest to reason about: sphere/sphere, cuboid/sphere, cuboid/cuboid, capsule/capsule, and cylinder/cylinder.
- [x] Keep legacy single-contact behavior as an internal compatibility path only while tests transition, then remove it if the manifold path fully replaces it.
- [x] Add benchmarks that compare single-contact and manifold generation cost by shape family.
- [x] Document manifold limits and known unsupported mesh manifold behavior.

**Phase 3 Status - 2026-05-26**

- Replaced `ContactPoint` with `ContactManifold` and `ManifoldContact`.
  Collision pairs now own fixed-capacity manifold state directly; the stale
  single-contact API was removed instead of left as a compatibility bridge.
- Manifolds store up to four contacts, sort exposed contacts by stable contact
  identity, retain the deepest four candidates, and expose `PrimaryContact` for
  the current alpha response solver.
- Axis-aligned cuboid/cuboid detection now generates deterministic face, edge,
  and stacked/touching manifolds, including zero-depth contacts. The pair
  broad-phase gate now uses inclusive AABB overlap so runtime pair updates do
  not drop touching contacts before narrow phase.
- Sphere, capsule, cylinder, oriented cuboid SAT, and mesh paths now write one
  manifold contact. Full mesh contact manifolds remain deferred to Phase 7 mesh
  policy work.
- Collision response and diagnostics consume the manifold primary contact. Full
  multi-contact solving, friction, and warm-start use remain Phase 4/5 work.
- Added manifold-focused tests and collision-detection benchmark methods for
  primitive manifold generation and cuboid face-manifold generation.

## Phase 4: Response Solver, Friction, Restitution, And Stabilization

**Purpose:** Turn the prototype response into a deterministic solver with physically explainable units, friction impulses, stable restitution, and stacking behavior.

**Files:**

- Modify: `src/Gravitas/CollisionHandling/Response/CollisionResponse.cs`
- Potentially create: `src/Gravitas/CollisionHandling/Solver`
- Modify: `src/Gravitas/Core/StiffBody.cs`
- Modify: `src/Gravitas/Settings/PhysicsSettings.cs`
- Modify: `tests/Gravitas.Tests/CollisionHandling`
- Modify: `tests/Gravitas.Benchmarks/CollisionHandling`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`

**Tasks:**

- [x] Add solver tests for immovable bodies, kinematic bodies, equal mass exchange, different mass exchange, restitution thresholding, resting contact, slopes, and stacks.
- [x] Define normal and tangential impulse equations in fixed-point terms, including units and clamping rules.
- [x] Implement deterministic friction impulses after manifold contact data is available.
- [x] Add positional stabilization that does not hide narrow-phase penetration-depth bugs.
- [x] Benchmark solver cost by contact count and pair count.
- [x] Document solver invariants, remaining divergences from real-world physics, and any deliberate simplifications.

**Phase 4 Status - 2026-05-26**

- Replaced the primary-contact response path with a fixed-capacity manifold
  response pass. The solver builds up to four contacts from `ContactManifold`
  and no longer depends on `PrimaryContact` for physical response.
- Positional correction remains a solver-side slop/stabilization rule, but the
  correction is now divided across active contacts so a four-contact face
  manifold does not over-correct relative to the narrow-phase depth.
- Normal impulses are computed for all contacts before application. This keeps
  centered face manifolds from injecting angular velocity because of a single
  arbitrary corner ordering.
- Added deterministic Coulomb friction impulses after normal response.
  `StiffBody.FrictionCoefficient` is now a public validated coefficient shared
  by contact response and grounded body friction. Pair friction uses the
  geometric mean and clamps tangent impulse by `normalImpulse * coefficient`.
- Expanded response tests for different masses, tangential friction, sloped
  contact normals, and centered stacked face manifolds, alongside the existing
  immovable, kinematic, equal-mass, restitution-threshold, no-contact, trigger,
  zero-restitution, off-center angular, and deterministic replay coverage. Added
  a prepared-contact allocation guard for single-contact and face-manifold
  response after warmup.
- Updated the collision-response benchmark to compare single-contact and
  face-manifold solver cases across pair counts.
- Documented response equations, units, correction sharing, friction clamping,
  and remaining alpha simplifications in `docs/wiki/COLLISION_PIPELINE.md`.
  Static resting friction with cached normal forces remains intentionally
  deferred to Phase 5 island/warm-start work.

## Phase 5: Island Solving, Sleep, And Warm Starting

**Purpose:** Improve stability and cost for resting scenes while preserving deterministic ordering.

**Files:**

- Modify: `src/Gravitas/Core/GravitasPhysicsService.cs`
- Modify: `src/Gravitas/Core/GravitasCollisionService.cs`
- Modify: `src/Gravitas/CollisionHandling/Pairs/CollisionPair.cs`
- Modify: `src/Gravitas/Partitions/PhysicsPartition.cs`
- Potentially create: `src/Gravitas/CollisionHandling/Solver/PhysicsIsland.cs`
- Potentially create: `src/Gravitas/CollisionHandling/Solver/IslandBuilder.cs`
- Modify: `src/Gravitas/Core/StiffBody.cs`
- Modify: `tests/Gravitas.Tests/CollisionHandling`
- Modify: `tests/Gravitas.Tests/Partitions`
- Modify: `tests/Gravitas.Benchmarks/Core`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`

**Tasks:**

- [x] Define deterministic island membership, island ordering, pair ordering inside an island, and body wake/sleep rules.
- [x] Add tests for stacked bodies, body wake-up after impulse, sleeping body ignored by solver until disturbed, and stable island ordering across repeated runs.
- [x] Add per-partition awake dynamic membership or awake-count state so `PhysicsPartition.Distribute()` can skip pair generation when a voxel contains no awake dynamic bodies.
- [x] Keep sleeping bodies in normal dynamic partition membership so raycasts, overlap queries, wake propagation, contact exits, and future island rebuilds can still find them.
- [x] Define deterministic wake stimuli: force, impulse, collision with an awake body, kinematic motion, host transform teleport, shape/bounds mutation, and explicit host wake.
- [x] Add warm-start storage keyed by stable pair/manifold contact identity.
- [x] Ensure sleeping and warm starting never skip collision events or trigger/contact notifications incorrectly.
- [x] Benchmark large resting scenes before and after island/sleep changes, including partitions with only sleeping dynamic bodies.

**Design Notes:**

- Adapt the old acceleration-structure sleeping-object note to Gravitas'
  current GridForge voxel broad phase first: `PhysicsPartition` is flat voxel
  membership, so Phase 5 should introduce partition-local awake gating rather
  than recursive tree propagation.
- Sleeping should be a deterministic solver/broad-phase optimization, not a
  visibility rule. Sleeping bodies remain queryable and partitioned; only
  collision pair generation/solver work can be skipped when no awake dynamic
  body can disturb the partition.
- Island sleep should only occur when every dynamic body in the island remains
  under explicit linear/angular thresholds for the configured frame window. A
  wake event should wake the body and deterministically wake affected island or
  contact-connected neighbors.

**Phase 5 Status - 2026-05-26**

- Added deterministic body sleep state to `StiffBody`: configurable sleep
  enablement, frame window, linear/angular speed thresholds, explicit
  `Sleep()`, and `Wake()`. Sleep clears accumulated motion
  state while leaving the collider partitioned.
- Defined deterministic wake stimuli in docs and tests without exposing unused
  wake-reason API: explicit host wake, force, linear impulse, angular impulse,
  collision, kinematic motion, transform teleport, and shape mutation.
  Force/impulse/teleport/shape/collision paths now wake sleeping bodies before
  mutation or response.
- Follow-up review removed the unused `StiffBodyWakeReason` enum and simplified
  `Wake()` so Phase 5 does not leave speculative diagnostics/island-propagation
  scaffolding in the public API.
- Added `PhysicsPartition.ContainedAwakeDynamicObjects` and awake-count helpers.
  Partitions still keep all sleeping bodies in `ContainedDynamicObjects`, but
  `Distribute()` now returns early when a voxel has no awake dynamic IDs and
  distributes awake dynamic IDs against dynamic/static membership otherwise.
- Added collision-service awake-state refresh so body wake/sleep transitions
  update every partition currently occupied by the collider.
- Preserved sleeping resting contacts in the active-pair queue so a fully
  sleeping contact does not age out and emit a false contact exit while its
  partition legitimately skips pair generation.
- Added fixed-size pair-local warm-start storage keyed by stable manifold
  contact identity. The current solver records normal and tangent impulse
  scalars; applying cached impulses as a true iterative warm start remains a
  later solver hardening task.
- Defined the alpha "island" boundary as the current flat GridForge partition
  candidate set plus awake dynamic membership. No explicit graph island builder
  was introduced in this phase because the current solver still resolves pairs
  immediately during partition distribution. A future explicit island solver
  should build from the same body wake rules and pair/contact identities.
- Added focused tests for sleep window behavior, wake stimuli, shape mutation
  wake, stacked resting body sleep, sleeping-only partition skip, awake-vs-
  sleeping pair processing, contact-enter preservation, sleeping resting
  contact retention, repeated contact ordering, and warm-start storage/reset.
- Added `DistributeSleepingOnlyDynamicPartition` to the partition-culling
  benchmarks to watch the no-awake-dynamic branch. Short local smoke reported
  about `6.24 ns` mean and no managed allocation for 64 sleeping dynamic IDs;
  BenchmarkDotNet could not raise process priority in this sandbox, so treat
  timing as smoke evidence only.
- Updated `docs/wiki/COLLISION_PIPELINE.md`,
  `docs/wiki/RUNTIME_ARCHITECTURE.md`, and `docs/wiki/OVERVIEW.md` with the
  sleep/awake partition and warm-start storage behavior.
- Verification passed:
  `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~CollisionHandlingTests|FullyQualifiedName~PhysicsPartition|FullyQualifiedName~StiffBody"`
  passed with 86 tests; `dotnet test Gravitas.slnx --configuration Release --no-restore`
  passed with 169 tests; `dotnet test Gravitas.slnx --configuration ReleaseLean --no-restore`
  passed with 169 tests. The `ReleaseLean` test command emitted the existing
  MemoryPack shim `CS0436` warnings while still completing successfully.

## Phase 6: Continuous Collision Detection And Swept Mesh Policy

**Purpose:** Move from query-only swept sphere support toward collision-time continuous collision detection for fast movers.

**Files:**

- Modify: `src/Gravitas/Queries/SweptSphereQueryWorker.cs`
- Potentially create: `src/Gravitas/CollisionHandling/Continuous`
- Potentially create: `src/Gravitas/CollisionHandling/Continuous/ContinuousCollisionMode.cs`
- Modify: `src/Gravitas/CollisionHandling/Detection/CollisionDetection.cs`
- Modify: `src/Gravitas/Core/StiffBody.cs`
- Potentially modify: `src/Gravitas/Settings/PhysicsSettings.cs`
- Modify: `tests/Gravitas.Tests/Queries`
- Modify: `tests/Gravitas.Tests/CollisionHandling`
- Modify: `tests/Gravitas.Benchmarks/Queries`
- Modify: `tests/Gravitas.Benchmarks/Core`
- Modify: `docs/wiki/QUERY_SERVICES.md`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`

**Tasks:**

- [x] Decide whether CCD lives in collision detection, body integration, or a dedicated continuous-collision service.
- [x] Define the CCD activation policy. Current recommendation: do not run CCD for every body by default; use an explicit per-body or per-collider policy with an optional `Auto` mode and a context default. This keeps deterministic response ordering and hot-path cost visible to hosts while still making fast-projectile setup ergonomic.
- [x] Add tunneling tests for fast sphere/capsule/cuboid/cylinder bodies against thin static geometry.
- [x] Add tests proving discrete bodies still use the existing integration path while CCD-enabled bodies sweep from current position to proposed position.
- [x] Hook CCD after velocity/acceleration integration computes the intended frame displacement and before `StiffBody` commits authoritative position. The candidate sweep should use `startPosition -> startPosition + velocity * DeltaTime`, then clamp/adjust movement at the earliest deterministic time of impact.
- [x] Define time-of-impact ordering and response handoff for multiple hits in one frame.
- [x] Start with fast dynamic primitives against static, bodyless, immovable, or kinematic targets. Dynamic-vs-dynamic CCD requires relative velocity and pairwise TOI ordering; include it only if the phase can cover tests and benchmarks without weakening the alpha contract.
- [x] Decide the alpha policy for swept mesh targets: unsupported, triangle-sweep query path, convex-decomposed mesh path, or dedicated CCD mesh path.
- [x] If mesh sweep is implemented, require triangle candidate acceleration tests and benchmarks before enabling it broadly. Mesh sweep was not implemented in Phase 6, so this requirement carries into Phase 7 if mesh CCD becomes part of the mesh alpha policy.
- [x] Keep visual mesh-normal smoothing separate from physics. Physics CCD should use deterministic triangle planes/face normals/contact geometry; chunk seam smoothing or vertex-normal transfer belongs in mesh preprocessing or renderer-facing data, not collision truth.
- [x] Document the CCD contract and any excluded shape pairs.

**Phase 6 Result:**

- CCD now lives in `StiffBody` movement commit as an explicit body/context
  policy. `PhysicsSettings.DefaultContinuousCollisionMode` defaults to
  `Discrete`, while `StiffBody.ContinuousCollisionMode` defaults to `Inherit`.
  `Inherit` resolves through the precomputed top-parent body policy before
  falling back to the context default.
- `Continuous` always sweeps when displacement and proxy radius are valid.
  `Auto` sweeps when intended frame displacement exceeds the collider proxy
  radius.
- The alpha path uses a swept-sphere proxy for sphere, capsule, cuboid, and
  cylinder movers against non-trigger bodyless, immovable, or kinematic targets.
  It clamps to the earliest deterministic TOI and removes only closing normal
  velocity.
- Dynamic-vs-dynamic CCD and swept mesh targets remain intentionally deferred.
  They belong with relative-velocity ordering, deterministic TOI tie-breakers,
  and Phase 7 mesh policy work.

**Design Notes:**

- CCD exists to prevent fast bodies from tunneling between fixed ticks; it
  should not replace ordinary discrete collision for every body. Always-on CCD
  would add broad-phase sweeps, time-of-impact sorting, and response-order
  branches to normal bodies that do not need it.
- Prefer a policy shape such as `Inherit`, `Discrete`, `Continuous`, and
  possibly `Auto`. `Inherit` can read a context default, `Discrete` preserves the
  current path, `Continuous` forces CCD, and `Auto` can later enable CCD when
  the intended displacement is large relative to collider thickness, radius,
  voxel size, or a configured threshold. Do not add every mode unless tests and
  benchmarks justify it.
- The old "draw a line from current position to new position" note maps to the
  intended center sweep for the current frame. For extended shapes, use the
  appropriate swept volume or conservative support sweep rather than a naked
  center ray where that would miss edge/corner tunneling.
- The old "relative velocities" note is valid for dynamic-vs-dynamic CCD. For
  alpha, static/kinematic targets are the safer first boundary; relative-motion
  CCD should be added only with deterministic TOI tie-breakers and replay tests.
- The old mesh-normal note appears to describe visual smoothing across chunked
  mesh seams. It is relevant to Phase 7 mesh policy only if Gravitas starts
  storing or deriving mesh normals for contact generation. It should not drive
  Phase 6 CCD except as a reminder that physics normals must be deterministic
  geometry data, not renderer smoothing data.

## Phase 7A: Mesh Collider Alpha Policy And Local-Space BVH

**Purpose:** Keep mesh support useful, make convex/concave intent explicit, and
remove the current rigid-mesh movement cost before hardening more mesh behavior.
This phase may add small policy/scaffold seams for compound support, but it
should not implement `LSCompoundCollider`.

**Files:**

- Modify: `src/Gravitas/Colliders/Support/PhysicsMesh/PhysicsMesh.cs`
- Modify: `src/Gravitas/Colliders/Primitives/LSMeshCollider.cs`
- Modify: `src/Gravitas/CollisionHandling/Detection/CollisionDetection.cs`
- Modify: `src/Gravitas/Queries/RaycastSegmentWorker.cs`
- Potentially create: `src/Gravitas/Colliders/Support/PhysicsMesh/ConvexMeshPolicy.cs`
- Potentially create: `src/Gravitas/Colliders/Support/PhysicsMesh/MeshColliderPolicy.cs`
- Potentially create: `src/Gravitas/Colliders/Support/PhysicsMesh/MeshColliderMode.cs`
- Modify: `tests/Gravitas.Tests/Colliders`
- Modify: `tests/Gravitas.Tests/CollisionHandling`
- Modify: `tests/Gravitas.Tests/Queries`
- Modify: `tests/Gravitas.Benchmarks/Colliders`
- Reference only: `F:\gamedevrepos\SoulsClone\Library\PackageCache\com.whinarn.unitymeshsimplifier@d741912bfe`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`

**Tasks:**

- [x] Define alpha shape policy terminology. Mesh modes should use `Convex` and
  `Concave`; `Compound` should mean a separate `LSCompoundCollider` strategy,
  not a mesh mode. Keep `non-convex` as internal validation language for the
  wider set of topology problems such as concavity, open meshes, disconnected
  islands, self-intersection, non-manifold edges, holes, and winding issues.
- [x] Add an explicit mesh-collider mode or policy. Do not silently infer convex
  versus concave behavior except for cheap validation that proves a declared mode
  is invalid. Developer intent should drive runtime policy.
- [x] Define the alpha boundary: static/kinematic concave triangle meshes are
  acceptable collision data under the temporary 7A gate, but full static,
  kinematic, and dynamic concave support remains Phase 7B work. Convex meshes
  are the only mesh mode that 7A treats as dynamic-ready.
- [x] Decide that 7A does not claim true concave support. Deterministic
  Gravitas-owned convex decomposition and host/offline decomposed convex pieces
  belong to the dedicated concave mesh work in Phase 7B, not to the compound
  collider API in Phase 7C.
- [x] If 7A needs compound-related seams, keep them neutral and minimal:
  declared mesh modes and policy names only. Do not route concave support
  through compound colliders; Phase 7B owns concave narrow-phase behavior and any
  decomposition internals.
- [x] Refactor rigid mesh movement toward local-space acceleration ownership:
  build local vertices, local triangle normals, local bounds, and local triangle
  BVH once; update only transform, inverse transform, and conservative world
  bounds when the mesh moves; rebuild the BVH only when local vertex or triangle
  topology changes.
- [x] Update mesh query and narrow-phase callers to transform world-space query
  shapes into mesh-local space, query the local triangle BVH, and transform final
  contact points/normals back to world space. Rotation-only normal transforms are
  enough for the current scale model; any future non-uniform mesh scale needs an
  inverse-transpose normal policy.
- [x] Benchmark dynamic mesh movement before and after the local-space BVH
  refactor. The expected win is avoiding O(triangle count) BVH rebuilds on rigid
  translation/rotation.
- [x] Add tests for invalid mesh input, declared mesh-mode validation, rotated
  mesh bounds, dynamic mesh movement without triangle BVH rebuild, mesh/collider
  contact order, and concave/non-convex policy enforcement.
- [x] Add temporary policy tests proving raw dynamic concave meshes are rejected
  in 7A instead of silently behaving like convex meshes. Phase 7B will replace
  this temporary guard with real dynamic concave support.
- [x] Review `PhysicsMesh.CalculateInertiaTensor(...)` and decide the alpha contract: keep the current triangle-area-weighted approximation with documentation, or replace it with a deterministic thin-triangle/shell or tetrahedral volume approximation. Any replacement needs fixed-value tests on known simple meshes and benchmark coverage.
- [x] Decide whether the mesh edge cache remains required after manifold work. Remove it only if tests and benchmarks prove face normals/on-demand edges cover all callers.
- [x] Review the old `UnityMeshSimplifier` package and the prototype
  `MeshSimplificationHelper` for design context without vendoring Unity-specific
  runtime code into Gravitas. If temporary scratch notes are created under
  `docs/feature-work/prototype`, remove them before closing the task unless the
  user explicitly asks to keep or force-add them.
- [x] Decide mesh simplification/LOD policy. Prefer host/offline simplification
  for alpha; any Gravitas-owned runtime simplifier must be deterministic,
  fixed-point, bounded, benchmarked, and must not change collision truth during a
  simulation frame.
- [x] Decide dynamic mesh update boundaries: rigid transformed meshes should not
  rebuild local BVH/topology data, while deformable/breakable topology or vertex
  updates require explicit invalidation, deterministic rebuild ordering, and
  separate tests before support is claimed.
- [x] Add benchmark coverage for mesh construction, BVH build, triangle query windows, dynamic mesh repartitioning, and mesh contact generation.
- [x] Document the mesh policy in host-facing terms so engine adapters know what data to provide.

**Phase 7A Status - 2026-05-27**

- Added explicit `MeshColliderMode` values for `Convex` and `Concave`, with
  `MeshColliderPolicy` centralizing a temporary 7A guard that prevented movable
  dynamic concave meshes from silently behaving like convex meshes. Phase 7B
  removed this guard after adding triangle-level concave narrow phase coverage.
- Kept automatic deterministic convex decomposition out of 7A because no
  algorithm, determinism contract, pathological-shape tests, or benchmark budget
  has been proven yet. Phase 7B owns the next pass: true concave narrow phase,
  dynamic concave support, Gravitas-owned deterministic decomposition where
  feasible, and host/offline decomposed convex pieces as a fallback.
- Refactored `PhysicsMesh` to keep vertices, triangle normals, areas, bounds,
  and triangle BVH in local mesh space. Rigid movement now updates transform,
  inverse transform, lazy world vertices, and conservative world bounds without
  rebuilding the triangle BVH.
- Updated mesh triangle queries, mesh ray overlap, SAT mesh object data,
  mesh-cylinder contact generation, closest-surface lookup, support-point lookup,
  diagnostics, and inertia tensor behavior around the local-space mesh model.
- Kept mesh edge caches for now. They are local-space topology data and should
  only be removed in a later pass if tests and benchmarks prove on-demand edge
  handling covers all callers.
- Added focused tests for explicit mesh modes, the temporary dynamic concave
  rejection, kinematic concave acceptance, local-space BVH rebuild stability,
  moved-mesh world queries, moved-mesh closest-surface lookup, rotated mesh
  bounds, ray intersections, collision shape pairs, diagnostics, and
  local-geometry inertia tensors. The temporary rejection test was replaced by
  Phase 7B dynamic concave acceptance coverage.
- Added `MoveMeshRuntimeShapeStateAndQueryTriangles` benchmark coverage to
  compare rigid mesh movement/query cost after the local-space BVH refactor.
  The short smoke selected the benchmark successfully and measured the new path,
  but still reported `216 B/op`. Treat that as Phase 8 query
  scratch/data-structure follow-up rather than hiding it inside mesh policy work.
- Updated `docs/wiki/COLLISION_PIPELINE.md` with explicit mesh modes,
  local-space BVH ownership, concave dynamic-body policy, and simplification /
  decomposition boundaries.

**Design Notes:**

- Public mesh terminology should prefer `Convex` and `Concave`. `Compound` is a
  collider composition strategy, not a mesh classification. Concave meshes are
  non-convex, but non-convex is broader than concavity and includes invalid or
  unsupported topology. Use `non-convex` in validation notes only when the wider
  meaning is intentional.
- Concave mesh support should remain part of the long-term design, similar to
  the behavior users expect from engines such as Unity. The alpha boundary is
  not Unity's restriction: Gravitas should support concave meshes against every
  supported collider class, including dynamic concave bodies, once Phase 7B
  proves the behavior with tests and benchmarks.
- Decomposing a concave mesh into convex pieces is concave mesh implementation
  detail, not compound collider behavior. Host/offline decomposed convex pieces
  are an acceptable fallback input for concave mesh acceleration, while
  Gravitas-owned automatic decomposition must be deterministic, bounded,
  benchmarked, and tested before it is claimed.
- Ear clipping is useful for triangulating or partitioning 2D polygons, but it
  is not sufficient as a general 3D non-convex mesh decomposition strategy. If
  Gravitas eventually owns automatic 3D convex decomposition, evaluate a
  deterministic VHACD-style or exact convex partition approach against tests and
  benchmark budgets.
- The scratch note about mesh overlap being only bounds-based appears stale:
  current mesh ray overlap is triangle-backed, and mesh/cuboid, mesh/cylinder,
  and mesh/mesh collision paths already use triangle candidates/SAT-style
  checks. 7A should harden and classify those paths rather than assume no mesh
  collision exists.
- Visual mesh normals, smoothing, simplification, and render LOD are not
  automatically physics data. Physics mesh normals and simplified collision
  geometry must be deterministic inputs or deterministic preprocessing outputs.
- Before 7A, `PhysicsMesh.UpdatePosition(...)` transformed every vertex,
  invalidated normals, rebuilt the triangle BVH, and updated bounds for rigid
  movement. The 7A target is a local-space BVH with query/collision inputs
  transformed into mesh-local coordinates and final outputs transformed back to
  world space.
- The old Unity Mesh Simplifier package is useful context because it uses a
  quadric-error-style simplification pipeline with smart vertex linking and
  preservation flags. It should stay reference material for now: the package is
  Unity/float/double oriented, so a Gravitas simplifier would need a deliberate
  fixed-point port with deterministic tie-breakers and benchmark gates.

## Phase 7B: Concave Mesh Collision Contract And Decomposition

**Purpose:** Turn `MeshColliderMode.Concave` from declaration into a real
deterministic collision contract. Concave mesh colliders should collide with
every supported collider class, including other concave meshes and dynamic
concave bodies, when tests and benchmarks prove the behavior.

**Execution Order:** Run this phase before Phase 7C. Compound colliders should
not become the escape hatch for concave mesh support.

**Files:**

- Modify: `src/Gravitas/Colliders/Primitives/LSMeshCollider.cs`
- Modify: `src/Gravitas/Colliders/Support/PhysicsMesh/PhysicsMesh.cs`
- Delete: `src/Gravitas/Colliders/Support/PhysicsMesh/MeshColliderPolicy.cs`
- Potentially create: `src/Gravitas/Colliders/Support/PhysicsMesh/MeshConcavityAnalyzer.cs`
- Potentially create: `src/Gravitas/Colliders/Support/PhysicsMesh/MeshConvexDecomposition.cs`
- Create: `src/Gravitas/CollisionHandling/Detection/Mesh/MeshTriangleContactGenerator.cs`
- Potentially create: `src/Gravitas/CollisionHandling/Detection/Context/MeshTriangleContactContext.cs`
- Modify: `src/Gravitas/CollisionHandling/Detection/CollisionDetection.cs`
- Modify: `src/Gravitas/CollisionHandling/Detection/Context/MeshObjectInfo.cs`
- Modify: `src/Gravitas/Queries/RaycastSegmentWorker.cs` only if concave ray
  behavior exposes stale convex assumptions.
- Modify: `tests/Gravitas.Tests/Colliders/PhysicsMeshTests.cs`
- Create or modify: `tests/Gravitas.Tests/Colliders/MeshColliderModeTests.cs`
- Create or modify: `tests/Gravitas.Tests/CollisionHandling/ConcaveMeshCollisionTests.cs`
- Modify: `tests/Gravitas.Tests/CollisionHandling/CollisionDetectionShapePairTests.cs`
- Modify: `tests/Gravitas.Benchmarks/Colliders/ColliderShapeBenchmarks.cs`
- Modify: `tests/Gravitas.Benchmarks/CollisionHandling/CollisionDetectionBenchmarks.cs`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`

**Tasks:**

- [x] Define the mesh-mode runtime contract in code and docs. `Convex` mesh can
  use whole-shape convex assumptions where valid; `Concave` mesh must use
  triangle-set or decomposed-convex logic and must not project the whole mesh as
  one convex polytope.
- [x] Replace the temporary dynamic-concave rejection with a real policy.
  `MeshColliderMode.Concave` must be allowed for bodyless, immovable,
  kinematic, and dynamic bodies once the narrow phase below is implemented.
- [x] Add deterministic mesh fixtures under the test project rather than relying
  on imported engine meshes. Include at minimum:
  - a closed convex tetrahedron or cube mesh for convex-control tests.
  - an open concave inside-corner mesh made from three perpendicular quads so
    sphere/capsule/cylinder/cuboid contacts can be checked against floor and
    wall triangles.
  - a concave notch or U-channel mesh with reentrant geometry so tests prove the
    collider is not being treated as one convex hull.
  - two small concave meshes whose triangle sets overlap at a deterministic
    edge or face so concave-vs-concave ordering can be pinned.
- [x] Add failing tests before implementation for declared `Concave` meshes:
  static/bodyless, immovable, kinematic, and dynamic initialization should all be
  legal, and the dynamic body should still repartition through the local-BVH
  movement path without rebuilding triangle topology.
- [x] Add concave-vs-primitive narrow-phase tests for sphere, capsule, cuboid,
  and cylinder. Each shape pair needs at least one hit against an interior
  concave feature, one hit against an exterior face, one edge-touch case, and
  one separated case.
- [x] Add concave-vs-mesh tests for concave-vs-convex, convex-vs-concave, and
  concave-vs-concave. These tests must verify stable contact normals, stable
  contact IDs or deterministic point ordering where applicable, and reversed
  dispatch behavior.
- [x] Add simulation-level tests for dynamic concave bodies. At minimum, move a
  dynamic concave mesh into a primitive collider and move a primitive collider
  into a dynamic concave mesh; both should produce deterministic contacts and
  physically coherent response direction.
- [x] Split mesh narrow phase where necessary. Whole-mesh SAT is acceptable only
  for convex mesh paths. Concave paths should gather candidate triangles from
  the local BVH, run deterministic triangle-vs-shape or triangle-vs-triangle
  checks, then reduce contacts using stable depth, distance, triangle index,
  vertex index, and collider ID tie-breakers.
- [x] Keep all-hit/candidate buffers caller-owned or context-owned. Concave
  triangle gathering and contact reduction must avoid per-frame allocations
  after warmup.
- [x] Add or update allocation guard tests for concave mesh pair checks after
  warmup. Allocation tests should cover at least concave-vs-cuboid,
  concave-vs-cylinder, and concave-vs-concave.
- [x] Add benchmark coverage for concave candidate gathering, concave-vs-primitive
  narrow phase, concave-vs-convex mesh, concave-vs-concave mesh, and dynamic
  concave movement/repartitioning. Decomposition preprocessing remains
  unclaimed until a real decomposition data model lands.
- [x] Decide decomposition follow-up policy. The local-BVH triangle-set concave
  path is the alpha runtime baseline. Host/offline decomposed convex pieces and
  Gravitas-owned deterministic convex decomposition are not Phase 7B or Phase
  7C blockers; keep them in the mesh decomposition backlog and revisit only
  when benchmarks, mass/inertia work, or contact-manifold quality prove they
  would outperform or materially improve the triangle path.
- [x] Update `docs/wiki/COLLISION_PIPELINE.md` after code lands so it no longer
  describes dynamic concave meshes as routed through compound colliders.

**Phase 7B Implementation Notes:**

- Added triangle-level concave narrow phase through
  `MeshTriangleContactGenerator`, using local-BVH candidate gathering and
  context-owned buffers rather than whole-mesh convex projection.
- Removed the temporary `MeshColliderPolicy` dynamic-concave rejection.
  `MeshColliderMode.Concave` is now legal for bodyless, immovable, kinematic,
  and dynamic meshes.
- Added deterministic fixtures for a convex cube, an inside-corner concave mesh,
  a U-channel concave mesh, and overlapping mesh cases.
- Added unit coverage for concave-vs-sphere, capsule, cuboid, cylinder,
  convex mesh, concave mesh, dynamic movement, local-BVH stability, and warm
  allocation behavior.
- Added benchmark coverage for concave mesh/cuboid, mesh/cylinder, mesh/mesh,
  and dynamic concave mesh movement/query paths.
- Short BenchmarkDotNet smoke executed the new `*Concave*` benchmarks. Treat the
  timings as non-canonical. Explicit allocation guard tests report zero
  allocations for warmed concave collision checks; the short in-process
  benchmark still reports small non-zero allocation noise on concave mesh/cuboid
  and mesh/mesh plus the existing 216 B/op dynamic mesh movement/query signal.
- Did not add a half-wired public decomposition API in this phase. Runtime
  concave support is currently the raw triangle-set path. Host/offline
  decomposed convex-piece data and Gravitas-owned volumetric convex
  decomposition remain explicit follow-up work; they should land only with a
  real data model, tests, and benchmarks so the API surface does not get muddy.

**Mesh Decomposition Backlog:**

These are retained for context, but they are not Phase 7B completion work and
should not be treated as required before Phase 7C. The current local-BVH
triangle-set implementation is the alpha baseline until evidence says
otherwise.

- [ ] Evaluate host/offline decomposed convex-piece support as an optional
  `LSMeshCollider` data path only if benchmarks show the raw triangle path is
  too expensive for representative dense or contact-heavy concave meshes, or if
  closed-volume mass/inertia/manifold work needs convex chunks. The owning mesh
  must still present one collider ID, one body binding, one event surface, and
  one broad-phase identity. This must not be implemented as `LSCompoundCollider`
  or as internal collider identity leakage.
- [ ] Evaluate Gravitas-owned deterministic convex decomposition as R&D only if
  Gravitas needs an engine-agnostic asset-prep path. Any implementation must be
  explicit preprocessing, not implicit runtime mutation; it needs deterministic
  ordering, deterministic tie-breakers, bounded failure/result codes,
  pathological mesh tests, and benchmarks against the raw local-BVH triangle
  path before it can be claimed useful.
- [ ] Before implementing either backlog item, create comparison fixtures for
  raw triangle-BVH concave collision versus decomposed convex pieces across
  dense concave meshes, dynamic concave bodies, contact-heavy corners/channels,
  and closed-volume inertia/mass scenarios.

**Acceptance Bar:**

- Do not claim concave support until the unit tests prove every supported
  collider pair involving `MeshColliderMode.Concave`.
- Do not claim dynamic concave support until at least one simulation-level test
  proves deterministic movement, pair generation, contact normal direction, and
  response behavior for a dynamic concave body.
- Do not claim deterministic convex decomposition until decomposition outputs
  are stable across repeated runs and benchmarked against raw triangle narrow
  phase on the same fixtures.
- Do not use compound colliders to satisfy concave mesh requirements. Compound
  colliders remain a separate composition feature.

## Phase 7C: Compound Collider Policy And Scaffold

**Purpose:** Define `LSCompoundCollider` as a special-case collider composition
strategy. Phase 7C should execute after Phase 7B so compound behavior does 
not absorb unresolved concave mesh responsibilities.

**Files:**

- Potentially create: `src/Gravitas/Colliders/Primitives/LSCompoundCollider.cs`
- Potentially create: `src/Gravitas/Colliders/Support/Compound`
- Potentially modify: `src/Gravitas/Colliders/LSCollider.cs`
- Potentially modify: `src/Gravitas/CollisionHandling/Detection/CollisionDetection.cs`
- Potentially modify: `src/Gravitas/Diagnostics`
- Modify: `tests/Gravitas.Tests/Colliders`
- Modify: `tests/Gravitas.Tests/CollisionHandling`
- Modify: `tests/Gravitas.Benchmarks/Colliders`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`

**Tasks:**

- [x] Implement an initial runtime type and try to keep it narrow: one public collider
  identity/body/layer, stable part IDs, deterministic part ordering, aggregate
  bounds, and part-local transforms.
- [x] Define compound parts as internal collider-like shape parts, not
  independent host colliders or parent/child objects. They may reuse collider
  shape logic, but the compound owns registration, broad-phase identity, event
  surface, body binding, and lifecycle.
- [x] Define the supported alpha part set as primitives plus declared `Convex`
  mesh parts only. `Concave` mesh parts are not allowed inside
  `LSCompoundCollider`; concave mesh decomposition belongs to `LSMeshCollider`
  internals from Phase 7B.
- [x] Define compound collision rules before exposing an API:
  aggregate mass/inertia policy, part-level broad-phase behavior, pair identity,
  contact manifold reduction, trigger/contact event surface, parent/child
  filtering behavior, CCD proxy behavior, and debug draw representation.
- [x] Define deterministic contact reduction for overlapping compound parts.
  Internal part overlap is acceptable, but a single opposing collider should not
  receive arbitrary duplicate contacts. Prefer stable part ordering plus a
  physically meaningful best-contact/manifold reduction over "first contact
  found" behavior.
- [x] Add tests for part ordering, aggregate bounds, overlapping internal parts,
  rejection of concave mesh parts, single external event emission, parent/child
  hierarchy separation, debug draw output, and deterministic contact selection.
- [x] Add benchmarks for compound-vs-primitive pair checks and broad-phase
  repartitioning when a compound's aggregate bounds span many voxels.
- [x] Document the difference between compound colliders and parent/child
  collider hierarchy in `docs/wiki/COLLISION_PIPELINE.md`.

**Implementation Notes:**

- Added `LSCompoundCollider` and `CompoundColliderPart`. The owning compound is
  the only registered collider identity; parts are context-bound geometry only.
- Part order is constructor order, and part IDs are stable zero-based indices.
- Supported alpha parts are primitives and `LSMeshCollider` with
  `MeshColliderMode.Convex`. Nested compounds and concave mesh parts are
  rejected.
- Narrow phase dispatches compound pairs through existing part-vs-shape checks
  with context-owned scratch manifold state. The owning pair manifold performs
  stable deepest-contact reduction and duplicate suppression through contact
  identity ordering.
- Aggregate bounds drive broad-phase partitioning. Compound CCD uses the
  aggregate bounds' smallest half extent as the swept-sphere proxy, matching the
  conservative cuboid proxy style.
- Debug draw emits part geometry with the owning compound collider ID and
  `ColliderType.Compound`, leaving host renderers engine-agnostic.
- Aggregate inertia is an alpha approximation: part tensors are area-weighted
  and offset through a diagonal parallel-axis term. Revisit this when mass,
  center-of-mass, and closed-volume policy are hardened.

**Design Notes:**

- A compound collider behaves like one collider to the host and one body to the
  solver, but internally contains deterministic primitive and declared convex
  mesh parts. Concave mesh parts are excluded so compound behavior stays a
  composition feature instead of becoming the concave-mesh decomposition system.
  This is similar in spirit to parent/child collider composition, but should
  have one collider ID, one broad-phase owner, stable part ordering, and
  explicit aggregate mass/inertia rules.
- A compound collider is not the same as a parent/child collider hierarchy. A
  parent/child hierarchy can represent separate host objects, such as a warrior
  and a held sword. A compound collider represents one physical collider whose
  shape is approximated by multiple internal parts.
- Compound colliders can approximate complex shapes with multiple primitives or
  convex meshes, but they are not the concave mesh fallback. Raw concave triangle
  meshes and decomposed concave-mesh internals are owned by `LSMeshCollider` and
  the Phase 7B concave contract.

## Phase 8: Broad-Phase And Query State Scalability

**Purpose:** Tighten partition/query data structures and decide whether current context-owned query services are enough for alpha.

**Files:**

- Modify: `src/Gravitas/Core/GravitasCollisionService.cs`
- Modify: `src/Gravitas/Partitions/PhysicsPartition.cs`
- Modify: `src/Gravitas/Queries/GravitasQuery3DService.cs`
- Modify: `src/Gravitas/Queries/GravitasQuery3DService.cs`
- Potentially create: `src/Gravitas/Queries/QueryState`
- Modify: `tests/Gravitas.Tests/Partitions`
- Modify: `tests/Gravitas.Tests/Queries`
- Modify: `tests/Gravitas.Benchmarks/Core`
- Modify: `tests/Gravitas.Benchmarks/Queries`
- Modify: `docs/wiki/QUERY_SERVICES.md`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`

**Tasks:**

- [x] Validate packaged GridForge partition-provider behavior once the retention fix is released; remove temporary local project references when possible.
- [x] Compare `SwiftSparseMap<T>` membership against a future `SwiftSparseSet` or another deterministic sparse membership structure when available. Phase 0 moved partition membership to `SwiftSparseSet`; continue benchmarking the layout under broader partition/query scale tests.
- [x] Revisit active partition ordering and pair candidate ordering for determinism under high churn.
- [x] If Phase 8 introduces a hierarchical broad phase, spatial tree, or nested
  acceleration structure above GridForge voxels, revisit awake-state propagation
  from child nodes to parent nodes. A branch with no awake dynamic descendants
  should be skippable during pair generation, but only if tests prove sleeping
  bodies remain visible to queries, wake propagation, and contact lifecycle.
- [x] Add stress tests for moving many colliders across grids, repeatedly emptying/refilling partitions, and querying colliders spanning many voxels.
- [x] Decide whether query services should remain context-owned mutable services or expose explicit caller-owned/rented query state for reentrancy.
- [x] Compare custom ray/segment workers against `FixedRay` for any first-hit or non-allocation query paths where the downstream primitive now fits.
- [x] Add deterministic frame-based time-to-kill for retained empty `PhysicsPartition` instances so long-running simulations do not keep every touched voxel partition forever.
- [x] Investigate the small managed allocation reported by the Phase 7A
  `MoveMeshRuntimeShapeStateAndQueryTriangles` benchmark (`216 B/op` in the
  2026-05-27 short smoke). Confirm whether it comes from
  `SwiftFixedBVH<T>.Query(...)` scratch ownership, interface dispatch, or
  Gravitas call-site state, then either fix the hot path or capture the
  downstream SwiftCollections change needed.
- [x] Keep all-hit query paths caller-buffered and benchmarked.

**Phase 8 Implementation Notes:**

- Validated package references against `GridForge` `6.0.5`,
  `SwiftCollections` `4.1.0`, and `SwiftCollections.FixedMathSharp` `4.1.0`
  with no temporary local project references in the active project files.
- Kept empty `PhysicsPartition` instances attached to their GridForge voxels
  after the last collider leaves. The partition becomes inactive and
  query-invisible, but future collider churn within the retained window reuses
  the same partition instead of paying the successful
  `Voxel.TryAddPartition(...)` path again.
- Added `PhysicsSettings.RetainedPartitionTimeToKillFrames` and
  `PhysicsSettings.RetainedPartitionRetirementSweepBudget`. Empty retained
  partitions now expire through a deterministic frame-based, bounded sweep and
  return to the context-local partition pool.
- Added retained-partition reset cleanup so `GravitasWorldContext.Reset()`
  clears collider ID membership and activation state from voxel-retained
  partitions before physics IDs can be reused.
- Made active partition traversal deterministic by copying active partitions to
  a context-owned buffer, sorting by `WorldVoxelIndex`, and distributing pairs
  from sorted collider-ID buffers. This removes sparse-set dense-order churn
  from contact ordering.
- Reorganized `src/Gravitas/CollisionHandling` into source-layout subdomains:
  `Detection`, `Detection/Context`, `Detection/Mesh`, `Detection/Sat`,
  `Pairs`, `Contacts`, `Response`, and `Continuous`. Namespaces were left
  unchanged.
- Reviewed `SwiftSortedList` for distribution scratch buffers. Even with
  `AddRange`, it copies source items into a temporary array and merges into the
  sorted list, so the current reusable `SwiftList` bulk-copy plus sort path is
  the better non-allocation default unless benchmarks show persistent sorted
  membership is worth the mutation cost.
- Added tests for different partition churn orders, retained empty partitions,
  frame-based partition retirement, reset cleanup, and duplicate suppression
  when ray/circle queries touch a collider spanning many voxels.
- Investigated the `216 B/op` mesh movement/query benchmark allocation. The
  source was Gravitas call-site state: mesh bounds created a new
  `BoundingBox` on each rigid movement and repartitioning detached/re-added
  empty voxel partitions. `PhysicsMesh` now mutates warmed bounds in place, and
  retained voxel partitions remove the repartition add cost. The focused
  allocation guard reports zero bytes after warmup.
- The first short benchmark rerun exposed an `AccessViolationException` in the
  mesh bounds transform helper under BenchmarkDotNet's unrolled hot loop. The
  helper now transforms each corner in the caller frame and only passes the
  transformed point into the min/max accumulator. A rerun completed with
  `MoveMeshRuntimeShapeStateAndQueryTriangles` at about `1.957 us` and no
  managed allocation reported.
- No hierarchical broad phase was introduced in this phase, so awake-state
  propagation beyond flat GridForge voxel partitions remains deferred.
- Query services remain context-owned mutable services for alpha. They are
  single-threaded by contract; concurrent query job/state objects remain a
  future redesign only if host requirements demand them.
- `FixedRay` was reviewed as the downstream primitive comparison point. The
  current custom segment worker remains the better alpha fit because Gravitas
  needs bounded segment queries, all-hit intersection buffers, starting-inside
  behavior, mesh triangle candidate handling, and deterministic caller-buffered
  result ordering.
- Split `CollisionDetection` into shape-focused partial files so future
  narrow-phase work can target sphere, capsule, cuboid, cylinder, compound, and
  mesh paths without navigating a 1200-line monolith.

## Phase 9: First-Class 2D Physics Foundation

**Purpose:** Stop treating 2D as accidental 3D flattening and build a working
pure-2D simulation slice with first-class 2D body, shape, collision, response,
query, and replay contracts. Mixed 2D/3D interaction is intentionally Phase 10
work.

**Completion Target:** By the end of Phase 9, Gravitas should support a pure 2D
simulation path for circle, axis-aligned box, and convex polygon colliders with
deterministic integration, broad-phase bounds, narrow-phase contacts, response,
queries, replay tests, and benchmark coverage. The implementation must not
layer 2D as reused 3D primitive code with one axis ignored or make 2D pay
unnecessary 3D runtime costs. After the Phase 9C/9D review, Phase 9 remains
open until the 2D runtime contract cleanup and GridForge-backed broad-phase
hardening below are complete.

**Reference Context:**

- Old deterministic prototype reference:
  `F:\gamedevrepos\LockstepFramework-develop\Core\Simulation\Physics\Core`,
  especially `LSBodyOverlapCheck.cs`, `LSBody.cs`, and `CollisionPair.cs`.
  Use this for shape vocabulary and test inspiration, not as code to port.
- Current downstream primitives to review before custom math:
  `FixedMathSharp.Geometry.BoundingArea`, `FixedMathSharp.FixedRay`,
  `SwiftCollections.FixedMathSharp.Query.FixedBoundVolume`, and
  `SwiftFixedSpatialHash<T>`.

**Files:**

- Potentially create: `src/Gravitas/Dimensions`
- Potentially create: `docs/wiki/DIMENSIONS.md`
- Potentially modify: `src/Gravitas/Core/StiffBody.cs`
- Potentially modify: `src/Gravitas/Colliders/LSCollider.cs`
- Potentially create: `src/Gravitas/Colliders/Primitives2D`
- Potentially create or modify: `src/Gravitas/CollisionHandling/Detection`
- Potentially create or modify: `src/Gravitas/Queries`
- Modify: `tests/Gravitas.Tests`
- Modify: `tests/Gravitas.Benchmarks`
- Modify: `docs/wiki/OVERVIEW.md`
- Modify: `docs/wiki/RUNTIME_ARCHITECTURE.md`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`

**Tasks:**

- [x] **Phase 9A - Dimension contract:** Write `docs/wiki/DIMENSIONS.md`
  defining 2D axes, units, coordinate embedding, rotation model, gravity model,
  body state, collider shape set, query behavior, serialization expectations,
  and the explicit boundary that mixed 2D/3D is Phase 10.
- [x] **Phase 9A - Architecture decision:** Decide and document the shared
  engine model: common identity/lifecycle where useful, but dimension-specific
  body motion/state, shape data, broad-phase bounds, narrow-phase detection, and
  solver paths. Avoid separate engines that recreate the Unity Box2D/PhysX
  split, and avoid mode flags that bloat current 3D hot paths.
- [x] **Phase 9B - Body and collider responsibility split:** Audit
  `StiffBody` and `LSCollider` for baked 3D/y-up/XZ-ground assumptions. Create
  seams for physical body state versus visual presentation state and for
  collider identity versus dimension-specific shape logic. If `LSCollider`
  remains the public base, primitive geometry should move toward focused shape
  support types rather than more abstract-class bloat.
- [x] **Phase 9B - Bounds and broad phase:** Define how pure 2D bounds map to
  broad-phase storage. Prefer `BoundingArea`/`FixedBoundVolume` and
  SwiftCollections fixed query structures where they fit; document any custom
  path if 2D needs a leaner dedicated bound representation.
- [x] **Phase 9C - 2D shape foundation:** Add first-class shape support for
  circle, axis-aligned box, and convex polygon. Treat polygon concavity as an
  explicit unsupported/validation case for the first slice rather than silently
  accepting ambiguous shape truth.
- [x] **Phase 9C - 2D narrow phase and queries:** Add deterministic
  circle/circle, circle/AABB, AABB/AABB, circle/convex-polygon,
  AABB/convex-polygon, and convex-polygon/convex-polygon detection. Add
  2D-specific ray/segment or overlap query behavior without routing through
  unnecessary 3D workers.
- [x] **Phase 9D - Pure 2D integration and response:** Add tests and
  implementation for pure 2D deterministic integration, contact manifolds,
  response, trigger/contact events, sleep/wake behavior, and replay. Do not
  claim mixed 2D/3D collision support in this phase.
- [x] **Phase 9D - Verification and benchmarks:** Add focused 2D unit tests,
  deterministic replay tests, and 2D benchmark selections for integration,
  broad-phase membership, narrow-phase shape pairs, response, and queries.
  Benchmark 2D separately from 3D so regressions and unnecessary 3D costs are
  visible.
- [x] Update `docs/wiki/OVERVIEW.md`, `RUNTIME_ARCHITECTURE.md`,
  `COLLISION_PIPELINE.md`, and `QUERY_SERVICES.md` to describe the pure 2D
  model and explicitly defer mixed 2D/3D interaction to Phase 10.
- [x] **Phase 9E - 2D runtime contract cleanup:** Align pure 2D host binding,
  runtime toggles, type-system boundaries, transform projection, folder layout,
  and parity tests before broad-phase scale work.
- [x] **Phase 9F - GridForge-backed 2D broad phase:** Replace the alpha
  sweep-and-prune runtime path with a partitioned 2D broad phase that shares
  GridForge world/voxel identity, keeps queries caller-buffered, and proves
  scale behavior with tests and benchmarks. Keep mixed 2D/3D collision itself
  deferred to Phase 10.
- [x] **Phase 9G - 2D dense pair hardening:** Remove avoidable dense-scene
  overhead from the GridForge-backed 2D broad phase before Phase 10. In
  particular, partition traversal must not be invalidated by response-time
  repartitioning, hot duplicate/pair state must retain warmed capacity, and
  the `physics-2d` benchmarks should show the dense-pair path moving toward the
  sweep baseline without sacrificing partition scale behavior.
- [x] **Phase 9H - 2D host polish and query parity:** Add pure 2D visualization
  transform publishing and a caller-buffered deterministic `Raycast2D` path
  backed by the same 2D partition/query rules. Keep mixed 2D/3D collision
  deferred to Phase 10.
- [x] **Phase 9I - Query domain consolidation:** Move 2D and 3D raycast,
  swept-sphere, circle-overlap, hit sorting, and query-detection ownership into
  a single query domain with explicit `Query2D` and `Query3D` context services.
  Remove stale query service surfaces rather than preserving compatibility
  bridges.

**Phase 9A-9B Status - 2026-05-28**

- Added `PhysicsDimension` as the shared body/collider dimensionality contract.
  At the 9A/9B checkpoint, production colliders remained explicitly `ThreeD`;
  the follow-up 2D collider family needed to own 2D shape data instead of
  reusing 3D primitive shape caches with ignored axes.
- Added `StiffBody.Dimension` with supported-value validation, post-initialize
  immutability, Chronicler state recording, and body/collider dimension
  mismatch rejection before body initialization mutates runtime state.
- Added `Physics2DBounds` as the alpha broad-phase bridge for pure 2D X/Z
  bounds into fixed `FixedBoundVolume` storage. Later Phase 9E/9F refinement
  should remove that abstraction unless a tiny private transient helper proves
  useful; it must not become a public 2D physical-thickness or storage-slab
  contract.
- Added focused tests for dimension defaults, unsupported dimension values,
  body/collider mismatch rejection, and deterministic 2D bounds projection.
- Added `docs/wiki/DIMENSIONS.md` and updated overview, runtime architecture,
  collision pipeline, and query service docs with the Phase 9A/9B boundary.
  At that point, Phase 9C/9D were planned to add real 2D shapes, 2D narrow
  phase, 2D response, 2D queries, replay, and benchmarks.

**Phase 9C-9D Status - 2026-05-28**

- Added the pure 2D runtime slice under `src/Gravitas/Physics2D` and
  `src/Gravitas/Colliders/Primitives2D`: `StiffBody2D`,
  `GravitasPhysics2DService`, `LSCollider2D`, `LSCircleCollider2D`,
  `LSAABBoxCollider2D`, `LSPolygonCollider2D`, `CollisionDetection2D`,
  `CollisionPair2D`, `Contact2D`, and `Physics2DHit`.
- Wired `GravitasWorldContext.Physics2D` into `Simulate`, `LateSimulate`, and
  `Reset` while keeping mixed 2D/3D interaction out of Phase 9.
- Implemented a lean pure 2D sweep-and-prune broad phase sorted by `MinX`, with
  Y-bounds rejection before deterministic 2D narrow-phase dispatch. This avoids
  making pure 2D scenes pay for the 3D GridForge partition path until benchmarks
  justify a different 2D broad-phase structure.
- Added deterministic 2D collision checks for circle/circle, circle/AABB,
  AABB/AABB, circle/convex-polygon, AABB/convex-polygon, and
  convex-polygon/convex-polygon. Concave or collinear polygon input is rejected
  up front.
- Added simple pure 2D response and contact lifecycle: position correction to
  penetration slop, closing-velocity normal impulse, sleep/wake propagation,
  and contact enter/stay/exit events. Rich 2D manifolds, angular impulses,
  friction impulses, and mixed-dimension impulse exchange remain future solver
  work.
- Added `OverlapCircleAll` as the first explicit pure 2D query API. Results are
  written into caller-owned `SwiftList<Physics2DHit>` buffers and sorted by
  surface distance plus collider ID.
- Added focused tests for 2D shape bounds, convexity validation, rotated
  polygon vertices, required shape-pair narrow phase, pure 2D overlap query
  ordering, integration, response, contact events, sleep/wake, and replay.
- Added `Physics2DBenchmarks` with the `physics-2d`/`2d` benchmark selection
  covering 2D integration, overlapping-pair response, required shape-pair
  detection, and `OverlapCircleAll`. The short in-process smoke exposed a
  broad-phase sort allocation, so the 2D service now uses an allocation-free
  deterministic in-place sort; the repeat smoke reported no managed allocation
  for all four 2D benchmark methods.
- Pre-review hardening tightened the pure 2D lifecycle and filtering contract:
  collider IDs are monotonic within a context, deactivation swap-removes the
  collider and immediately separates owned pairs, simulation respects the
  context collision matrix, `OverlapCircleAll` has a layer-mask overload, trigger
  enter/exit events mirror the 3D event surface, and solid sleeping/static pairs
  no longer wake or resolve unless an awake movable body participates.
- Updated `docs/wiki/DIMENSIONS.md`, `OVERVIEW.md`,
  `RUNTIME_ARCHITECTURE.md`, `COLLISION_PIPELINE.md`, and
  `QUERY_SERVICES.md` with the pure 2D alpha model and Phase 10 mixed-dimension
  boundary.

## Phase 9E: 2D Runtime Contract Cleanup

**Purpose:** Make the pure 2D runtime feel like the same engine-facing API as
the 3D runtime, while removing dimension scaffolding that no longer earns its
keep. This phase should keep mixed 2D/3D disabled and focus on host contracts,
source layout, lifecycle parity, and tests.

**Files:**

- Modify: `src/Gravitas/Runtime/GravitasWorldContext.cs`
- Modify: `src/Gravitas/Settings/PhysicsSettings.cs`
- Potentially create: `src/Gravitas/Settings/PhysicsRuntimeMode.cs`
- Modify: `src/Gravitas/Core/StiffBody2D.cs` after moving it from the current
  `src/Gravitas/Physics2D` folder.
- Modify: `src/Gravitas/Colliders/Primitives2D/LSCollider2D.cs`
- Delete: `src/Gravitas/Dimensions/PhysicsDimension.cs`
- Delete: `src/Gravitas/Dimensions/PhysicsDimensionRules.cs`
- Delete: `src/Gravitas/Dimensions/Physics2DBounds.cs` unless Phase 9F proves
  a tiny private transient helper is needed. Do not keep it as a public or
  long-lived configuration abstraction.
- Move: `src/Gravitas/Physics2D/GravitasPhysics2DService.cs` to
  `src/Gravitas/Core/GravitasPhysics2DService.cs`
- Move: `src/Gravitas/Physics2D/CollisionDetection2D.cs` to
  `src/Gravitas/CollisionHandling/Detection/CollisionDetection2D.cs`
- Move: `src/Gravitas/Physics2D/CollisionPair2D.cs` to
  `src/Gravitas/CollisionHandling/Pairs/CollisionPair2D.cs`
- Move: `src/Gravitas/Physics2D/Contact2D.cs` to
  `src/Gravitas/CollisionHandling/Contacts/Contact2D.cs`
- Move: `src/Gravitas/Physics2D/CollisionResponse2D.cs` to
  `src/Gravitas/CollisionHandling/Response/CollisionResponse2D.cs`
- Move: `src/Gravitas/Physics2D/Physics2DHit.cs` and
  `src/Gravitas/Physics2D/Physics2DHitSorter.cs` to the query domain.
- Modify: `tests/Gravitas.Tests/Physics2D`
- Modify: `tests/Gravitas.Tests/Core`
- Modify: `tests/Gravitas.Benchmarks/Physics2D`
- Modify: `docs/wiki/DIMENSIONS.md`
- Modify: `docs/wiki/RUNTIME_ARCHITECTURE.md`
- Modify: `docs/wiki/HOST_INTEGRATION.md`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`
- Modify: `docs/wiki/QUERY_SERVICES.md`

**Tasks:**

- [x] Add a context-level runtime mode with explicit `TwoD` and `ThreeD`
  options only. Do not add `Mixed` until Phase 10 implements the embedding and
  impulse-exchange policy.
- [x] Update `GravitasWorldContext.Simulate()`, `LateSimulate()`,
  `Visualize()`, and `LateVisualize()` so the disabled dimensional runtime path
  is skipped entirely. Pure 2D should not pay 3D service cost, and pure 3D
  should not pay 2D service cost.
- [x] Add runtime-mode tests proving `TwoD` skips 3D simulation/visualization
  work, `ThreeD` skips 2D simulation work, and the active mode still advances
  the shared deterministic clock and hooks correctly.
- [x] Change `StiffBody2D` construction to use `IMatterAgent` as the required
  host bridge, matching the 3D body contract. The body should derive its
  `GravitasWorldContext` from `agent.Context`, keep the agent reference, and
  use the agent's `FixedTransform` for host-facing kinematic and visual
  synchronization.
- [x] Add `LSCollider2D.InitializeWithNoBody(IMatterAgent)` parity for bodyless
  static/trigger 2D colliders. Bodyless 2D colliders must still bind to the
  host agent, context, layer, trigger/contact surface, and broad-phase service.
- [x] Define and document pure 2D transform projection explicitly. Gravitas'
  stack convention is X/Z planar physics: `Vector2d.x` maps from
  `Vector3d.x`, `Vector2d.y` maps from `Vector3d.z`, and `Vector3d.y` remains
  vertical height or future embedding metadata. `Vector3d.ToVector2d()` is
  correct for this convention and should be preferred when converting 3D world
  positions to pure 2D planar positions.
- [x] Add 2D kinematic tests proving host transform movement updates the 2D
  body deterministically, refreshes collider bounds/query visibility, and does
  not mutate disabled runtime paths.
- [x] Add bodyless static 2D collider tests for collision, trigger events,
  layer filtering, query visibility, deactivation cleanup, and reset cleanup.
- [x] Add same-agent or explicit hierarchy exclusion tests for 2D colliders.
  The final rule should match the engine-agnostic 3D host-bound hierarchy
  contract without walking host transform trees at simulation time.
- [x] Remove `PhysicsDimension` and `PhysicsDimensionRules`. The runtime should
  use concrete body/collider types to distinguish 2D and 3D. Update tests and
  docs so mixed behavior is described as a future policy between concrete types,
  not as a third dimension enum value.
- [x] Reorganize the current `src/Gravitas/Physics2D` files into the same
  subdomain layout as their 3D counterparts. Avoid namespace churn unless it
  reduces real API ambiguity.
- [x] Remove `Physics2DBounds` unless the Phase 9F implementation proves a
  tiny private transient helper is needed. Prefer direct collider-bounds to
  GridForge planar coverage flow over another configuration-shaped abstraction.
- [x] Update `docs/wiki` pages and benchmark docs to reflect the final 2D host
  contract, runtime mode behavior, transform projection, and source layout.
- [x] Run focused 2D/core tests, full `Release` build/test, full `ReleaseLean`
  build/test, and a short `physics-2d` benchmark smoke before closing this
  phase.

**Acceptance Bar:**

- The public 2D body/collider creation flow should feel like the 3D flow:
  host code supplies an `IMatterAgent`, not a raw context.
- `PhysicsDimension` must be gone from production, tests, and docs unless a
  concrete mixed-dimension design reintroduces a better concept later.
- Runtime mode tests must prove disabled dimensional paths do not run.
- 2D kinematic, bodyless, trigger, layer, hierarchy, query, and replay behavior
  must remain deterministic after the source-layout cleanup.

**Phase 9E Status - 2026-05-29**

- Added `PhysicsRuntimeMode.TwoD` and `ThreeD` on `PhysicsSettings`; context
  simulation and visualization phases now advance only the enabled dimensional
  service while keeping the shared clock, coroutines, and hooks active.
- Changed `StiffBody2D` construction to require `IMatterAgent`, derive context
  from `agent.Context`, and project kinematic host transforms through the X/Z
  convention.
- Added bodyless 2D collider binding through
  `LSCollider2D.InitializeWithNoBody(IMatterAgent)`, including query visibility,
  static collision response, and same-agent collision suppression.
- Removed `PhysicsDimension`, `PhysicsDimensionRules`, and `Physics2DBounds`.
  Pure 2D collider bounds now use `FixedMathSharp.BoundingArea` directly.
- Moved 2D runtime files beside their 3D counterparts:
  `StiffBody2D` and `GravitasPhysics2DService` moved to `Core`, 2D detection,
  pairs, contacts, and response moved under `CollisionHandling`, and
  `Physics2DHit` moved under `Queries`.
- Updated tests, benchmarks, and `docs/wiki` for runtime mode, host-agent 2D
  setup, X/Z projection, source layout, and the removal of dimension scaffolding.

## Phase 9F: GridForge-Backed 2D Broad Phase And Scale Gate

**Purpose:** Replace the current pure 2D sweep-and-prune runtime path with a
partitioned broad phase that can scale beyond small correctness scenes and can
later participate in mixed 2D/3D interaction through shared GridForge
world/voxel identity.

**Design Direction:**

- Start by defining the Gravitas-side 2D broad-phase contract cleanly: X/Z
  planar coverage, stable collider IDs, stable voxel/partition identity,
  deterministic
  active partition ordering, awake-dynamic gating, retained empty partition
  cleanup, duplicate suppression, and caller-buffered query candidate gathering.
- Use GridForge as the single voxel world model. Do not add a separate 2D grid
  type, 2D GridForge engine, or Gravitas-only grid stand-in. Gravitas should
  use the host-provided `GridWorld` configured for the simulation shape, then
  ask GridForge for deterministic X/Z planar voxel coverage.
- Avoid introducing `Physics2DBounds` as another configuration-shaped
  abstraction. Collider bounds should flow directly into GridForge planar
  coverage helpers or, if absolutely necessary, a private transient calculation.
- If the Gravitas implementation starts reimplementing GridForge traversal by
  hand or paying unnecessary 3D grid costs, local-link `../GridForge` and add
  the smallest reusable primitive there.
- GridForge helper work should improve the existing world model for planar and
  volume use. It should not fork GridForge into separate 2D and 3D concepts.

**Files:**

- Modify: `src/Gravitas/Core/GravitasPhysics2DService.cs`
- Potentially create: `src/Gravitas/Core/GravitasCollision2DService.cs`
- Potentially create: `src/Gravitas/Partitions/PhysicsPartition2D.cs`
- Potentially create: `src/Gravitas/Colliders/Support/ColliderPartitionState2D.cs`
- Delete or avoid: `src/Gravitas/Dimensions/Physics2DBounds.cs` unless a tiny
  private transient helper survives profiling and code review.
- Modify: `src/Gravitas/Settings/PhysicsSettings.cs` if 2D partition retention,
  sweep budget, or cell sizing needs settings.
- Modify: `tests/Gravitas.Tests/Physics2D`
- Modify: `tests/Gravitas.Tests/Partitions`
- Modify: `tests/Gravitas.Benchmarks/Physics2D`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`
- Modify: `docs/wiki/QUERY_SERVICES.md`
- Modify: `docs/wiki/RUNTIME_ARCHITECTURE.md`
- Potentially local-link and modify: `../GridForge/src/GridForge`
- Do not commit temporary local project references unless the user explicitly
  asks to keep them.

**Tasks:**

- [x] Add benchmark baselines for the current sweep-and-prune path using
  representative 2D scene shapes: sparse thousands, dense clusters, moving
  churn, sleeping-heavy scenes, query-heavy scenes, and broad colliders that
  span many cells.
- [x] Add correctness tests that pin current pure 2D pair ordering, query
  ordering, layer filtering, deactivation cleanup, sleeping/static behavior,
  and replay before changing the broad phase.
- [x] Design `PhysicsPartition2D` around collider IDs, not collider references.
  It should mirror the useful 3D partition lessons: dynamic/static membership,
  awake-dynamic membership, deterministic sorted distribution, retained empty
  partition cleanup, and reset safety before IDs can be reused.
- [x] Implement 2D collider partition state so movement, rotation, local shape
  edits, activation, deactivation, and reset update partition membership without
  scanning every collider every frame.
- [x] Replace pair generation in `GravitasPhysics2DService.Simulate()` with
  partition-driven candidate distribution. The sweep-and-prune path may remain
  only as a benchmark baseline during the phase; remove it from runtime once the
  partition path is proven correct and faster or complexity-safer.
- [x] Keep 2D query APIs caller-buffered. `OverlapCircleAll` should gather
  candidates from relevant 2D partitions, suppress duplicates for colliders
  spanning multiple voxels/cells, filter by layer mask, run exact shape checks,
  and sort by deterministic hit ordering.
- [x] Add tests for large 2D colliders spanning many cells, duplicate
  suppression, retained empty 2D partitions, partition time-to-kill, reset
  cleanup, sleeping partition skips, and wake propagation.
- [x] Benchmark the partition path against the sweep-and-prune baseline for
  sparse, dense, moving, sleeping-heavy, and query-heavy scenes. Keep timings as
  evidence, not truth, unless the scenario represents a real alpha workload.
- [x] If current GridForge helpers make 2D broad-phase implementation awkward,
  local-link `../GridForge` and evaluate adding one or more narrowly scoped
  helpers:
  - deterministic X/Z planar bounds scan over `GridWorld`/`VoxelGrid`;
  - plane-aware or axis-projection voxel coverage helpers over the existing
    GridForge grid model;
  - covered voxel/partition enumeration that avoids Gravitas duplicating
    GridForge traversal;
  - clearer planar-use docs or names where GridForge already supports the use
    case but the API wording implies volume-only behavior.
- [x] If GridForge changes are needed, validate GridForge directly with its
  focused tests plus `Release`/`ReleaseLean` build/test gates before validating
  Gravitas against the local link.
- [x] After package-local validation, remove any temporary project references
  from Gravitas unless the user asks to keep them for the hardening window.
- [x] Update `docs/wiki` to describe the final 2D broad-phase path, including
  which pieces are storage projection versus physical 2D semantics.

**Acceptance Bar:**

- Pure 2D collision distribution must no longer scan every active collider
  against every other active collider for normal runtime simulation.
- Tests must prove deterministic pair ordering, query ordering, duplicate
  suppression, retained partition cleanup, and replay under high collider
  counts and movement churn.
- Benchmarks must compare the previous sweep-and-prune baseline to the
  GridForge-backed path. If sweep-and-prune remains faster for small scenes,
  document the crossover and keep the runtime choice explicit.
- No fake 3D thickness, storage slab, or `Physics2DBounds` abstraction may leak
  into public pure 2D APIs. Pure 2D semantics are X/Z planar until Phase 10
  defines physical mixed 2D/3D interaction.
- If GridForge changes are required, they must be minimal, reusable outside
  Gravitas, deterministic, and validated in GridForge before Gravitas relies on
  them.

**Phase 9F Status - 2026-05-29**

- Added `GravitasCollision2DService` and `PhysicsPartition2D` as the pure 2D
  GridForge-backed broad phase. Runtime 2D simulation no longer sorts and scans
  every active collider; it distributes candidates from active 2D voxel
  partitions.
- `PhysicsPartition2D` stores collider IDs in static, dynamic, and awake
  dynamic sparse sets. Bodyless 2D colliders and immovable 2D bodies are static
  members; movable 2D bodies are dynamic members.
- Added `ColliderPartitionState2D` to track 2D partition coordinates and snapped
  X/Z grid bounds. Body movement, kinematic host motion, bodyless collider
  `Simulate()`, shape edits, activation, deactivation, and reset now refresh
  partition membership without a service-wide collider scan.
- `OverlapCircleAll` remains caller-buffered but gathers candidates from
  relevant `PhysicsPartition2D` payloads, suppresses duplicate collider IDs,
  filters layers and bounds, runs exact 2D shape checks, and sorts hits by the
  existing deterministic hit order.
- Pure 2D partition storage uses the internal GridForge Y=0 plane only as broad
  phase identity. No public `Physics2DBounds`, fake physical thickness, or slab
  configuration was introduced.
- GridForge package APIs were sufficient for this phase; no local GridForge
  project link or upstream GridForge changes were needed.
- Added focused partition-broad-phase tests for sparse scenes, duplicate query
  suppression, movement churn, retained partition TTK, sleeping skips, wake
  propagation, replay, and preserving existing resting solid contacts when a
  partition becomes sleep-gated.
- Added physics-2D benchmark sweep baselines next to the GridForge-backed
  runtime methods so future runs can compare the old sweep shape against the
  partition path.
- Short `physics-2d` benchmark smoke completed. The partition-backed query path
  is already competitive on larger query scenes, but the dense overlapping-pair
  benchmark showed avoidable response-time partition refresh and duplicate
  candidate overhead. That became the Phase 9G target before making broad
  performance claims about dense 2D collision response.
- Focused 2D tests, full `Release` tests, full `ReleaseLean` tests, sequential
  `Release` build, and focused `physics-2d` benchmark smoke completed locally.

## Phase 9G: 2D Dense Pair Hardening

**Purpose:** Make the partition-backed pure 2D collision path scale without
paying avoidable dense-scene overhead before mixed 2D/3D adds more moving parts.

**Files:**

- Modify: `src/Gravitas/Core/GravitasCollision2DService.cs`
- Modify: `src/Gravitas/Core/GravitasPhysics2DService.cs`
- Modify: `src/Gravitas/CollisionHandling/Pairs/CollisionPair2D.cs`
- Modify: `src/Gravitas/Core/StiffBody2D.cs`
- Modify: `src/Gravitas/Colliders/Primitives2D/LSCollider2D.cs`
- Modify: `tests/Gravitas.Tests/Physics2D`
- Modify: `tests/Gravitas.Benchmarks/Physics2D/Physics2DBenchmarks.cs`

**Tasks:**

- [x] Add or update tests proving 2D collision response does not mutate
  GridForge partition membership while `PhysicsPartition2D` is distributing
  candidates.
- [x] Defer partition refreshes caused by 2D response until after the active
  partition distribution pass completes, while still refreshing query-visible
  state before control returns to the host.
- [x] Avoid redundant awake-state refreshes when `StiffBody2D.Wake()` is called
  on an already-awake body.
- [x] Keep warmed duplicate/pair/query buffers sized for dense scenes instead
  of allocating on every simulate.
- [x] Rerun the focused dense-pair benchmark before and after the change. If
  partition-backed dense response remains slower than sweep in tiny/dense
  scenes, document whether the remaining cost is real partition fan-out,
  collision response cost, or benchmark setup churn.

**Acceptance Bar:**

- Dense 2D pair simulation should not repartition colliders during partition
  distribution.
- Focused tests must cover response-time movement, duplicate suppression, and
  deterministic replay.
- `physics-2d` dense-pair benchmark results must improve or clearly identify
  the next real bottleneck with evidence.

**Phase 9G Status - 2026-05-29**

- Added a steady-state allocation regression test proving dense 2D response
  does not allocate after warmup.
- Deferred response-time 2D partition refreshes while
  `PhysicsPartition2D.Distribute(...)` is walking active partitions. Deferred
  membership is refreshed before the next simulation distribution or before
  any 2D query gathers candidates, preserving query-visible state.
- Kept partition candidate, query, duplicate, and pair buffers warmed instead
  of rebuilding capacity every frame.
- Avoided redundant awake-partition refresh when `StiffBody2D.Wake()` is called
  for an already-awake body.
- Moved cheap same-agent, layer-mask, and 2D bounds rejection ahead of the
  frame duplicate-pair set so dense partition fan-out does not hash impossible
  pairs.
- Added deterministic first-shared-partition gating for pairs emitted from
  multiple covered voxels. The frame duplicate set remains as a cross-grid
  safety net.
- Short `physics-2d` dense-pair benchmark evidence:
  - Before Phase 9G: 64-body partition response was about `503.6 us` with
    `34,049 B/op`; 1024-body partition response was about `21.9 ms` with
    `585,088 B/op`.
  - After deferred refresh, non-allocating sorts, cheap prefilters, and
    first-shared-partition gating: 64-body partition response was about
    `129.4 us` with no reported managed allocation; 1024-body partition
    response was about `4.97 ms` with `8 B/op` short-job noise.
  - The 1024-body detection-only sweep baseline in the same short job was about
    `2.53 ms` with `4 B/op` short-job noise. That baseline does not own pair
    lifecycle or response, so the
    remaining gap should be treated as solver/pair-service cost rather than a
    broad-phase allocation failure.

## Phase 9H: 2D Host Polish And Query Parity

**Purpose:** Round out the pure 2D host contract before Phase 10 by publishing
2D body state back to host transforms during visualization and by adding a
first-class 2D raycast query.

**Files:**

- Modify: `src/Gravitas/Runtime/GravitasWorldContext.cs`
- Modify: `src/Gravitas/Core/GravitasPhysics2DService.cs`
- Modify: `src/Gravitas/Core/StiffBody2D.cs`
- Modify: `src/Gravitas/CollisionHandling/Detection/CollisionDetection2D.cs`
- Modify: `src/Gravitas/Queries/Physics2DHit.cs`
- Modify: `tests/Gravitas.Tests/Physics2D`
- Modify: `docs/wiki/DIMENSIONS.md`
- Modify: `docs/wiki/HOST_INTEGRATION.md`
- Modify: `docs/wiki/QUERY_SERVICES.md`

**Tasks:**

- [x] Add `GravitasPhysics2DService.Visualize()` and hook it from
  `GravitasWorldContext.Visualize()` when `PhysicsRuntimeMode.TwoD` is active.
- [x] Add `StiffBody2D.OnVisualize()` so dynamic 2D body position/rotation is
  projected back into `IMatterAgent.Transform` using the X/Z planar convention
  while preserving host vertical height.
- [x] Add tests proving pure 2D visualize updates dynamic body transforms and
  does not run when the context is in 3D mode.
- [x] Add `Raycast2D` and `Raycast2DAll` style APIs on the pure 2D service,
  with caller-owned result buffers, layer masks, duplicate suppression for
  multi-voxel colliders, stable hit ordering, and deterministic start-inside
  behavior.
- [x] Add tests for circle, AABB, polygon, layer filtering, hit ordering,
  zero-length segments, and collider-spanning-many-voxels duplicate
  suppression.
- [x] Update wiki/query docs with the final pure 2D query surface.

**Acceptance Bar:**

- Host adapters can call `Visualize()` in pure 2D mode and receive updated
  `FixedTransform` position/rotation without touching authoritative simulation
  state.
- 2D raycasts must use pure 2D shape math and the GridForge-backed 2D
  partition path, not the 3D raycast service.
- Query results must be caller-buffered and deterministically ordered.

**Phase 9H Status - 2026-05-29**

- `GravitasWorldContext.Visualize()` and `LateVisualize()` now call the pure 2D
  service only when `PhysicsRuntimeMode.TwoD` is active.
- `StiffBody2D.OnVisualize()` projects authoritative 2D X/Z position and yaw
  rotation back into the host `FixedTransform` while preserving host vertical
  height.
- Added pure 2D segment raycast APIs on `GravitasPhysics2DService`:
  `Raycast(start, end, out hit)`, `Raycast(start, end, layerMask, out hit)`,
  `RaycastAll(start, end, results)`, and
  `RaycastAll(start, end, layerMask, results)`.
- 2D raycasts gather candidates through `GravitasCollision2DService`, use pure
  2D shape math for circles, AABBs, and convex polygons, handle starting inside
  colliders deterministically, skip zero-length segments, and sort all-hit
  results by distance plus collider ID.
- Added tests for transform publishing, runtime-mode gating, hit ordering,
  start-inside behavior, layer filtering, zero-length segments, and duplicate
  suppression for multi-voxel 2D colliders.
- Added `RaycastAll` and `RaycastAll_SweepBaseline` coverage to
  `Physics2DBenchmarks`.
- Hardened the shared 2D GridForge coverage scanner so query paths skip
  out-of-bounds grid positions before calling `VoxelGrid.TryGetVoxel(...)` and
  no longer allocate captured visitor delegates per query.
- Short `physics-2d` raycast benchmark evidence: 64-body `RaycastAll` reported
  about `29.0 us` with no managed allocation; 1024-body partition-backed
  `RaycastAll` reported about `29.4 us` with no managed allocation versus the
  detection-only sweep baseline at about `520.8 us`.

## Phase 9I: Query Domain Consolidation

**Purpose:** Make query ownership discoverable before mixed 2D/3D work starts.
The current query surface is split by implementation history: 3D raycasts and
circle overlaps lived in separate services under the old raycasting domain,
while pure 2D overlap/raycast methods live on `GravitasPhysics2DService` with
some query math inside `CollisionDetection2D`. This phase should consolidate
query-facing API and implementation under a query domain without changing
collision semantics.

**Files:**

- Rename/move: old `src/Gravitas/Raycasting` files into the broader
  `src/Gravitas/Queries` domain.
- Modify: `src/Gravitas/Runtime/GravitasWorldContext.cs`
- Modify: `src/Gravitas/Core/GravitasPhysics2DService.cs`
- Modify: `src/Gravitas/CollisionHandling/Detection/CollisionDetection2D.cs`
- Potentially create: `src/Gravitas/Queries/GravitasQuery2DService.cs`
- Potentially create: `src/Gravitas/Queries/GravitasQuery3DService*.cs`
- Potentially create: `src/Gravitas/Queries/QueryDetection2D.cs`
- Modify: `src/Gravitas/Colliders`
- Modify: `src/Gravitas/Diagnostics`
- Modify: `tests/Gravitas.Tests`
- Modify: `tests/Gravitas.Benchmarks`
- Modify: `README.md`, `AGENTS.md`, and `docs/wiki`

**Tasks:**

- [x] Add tests or update existing tests to exercise the intended context API:
  `context.Query2D.Raycast*`, `context.Query2D.OverlapCircleAll*`,
  `context.Query3D.Raycast*`, `context.Query3D.SweepSphere*`, and
  `context.Query3D.OverlapCircle*`.
- [x] Rename the `Raycasting` source folder to a query-domain folder and update
  namespaces/imports so hit types, hit sorters, workers, and query services are
  co-located.
- [x] Replace `GravitasWorldContext.Raycasts` and
  `GravitasWorldContext.CircleQueries` with one `Query3D` service. Do not leave
  public compatibility bridges.
- [x] Move pure 2D query methods and query buffers from
  `GravitasPhysics2DService` into `Query2D`, leaving `Physics2D` focused on
  registration, simulation, pair lifecycle, response, and visualization.
- [x] Move 2D query-specific shape math out of `CollisionDetection2D` into the
  query domain so collision detection owns contacts and query code owns hits.
- [x] Keep query results caller-buffered, layer-filtered, duplicate-suppressed,
  deterministically ordered, and context-owned.
- [x] Review 3D query traversal while the code is open. Remove avoidable
  duplication where it is clearly safe, but avoid changing ray/sweep/circle
  semantics in the same pass.
- [x] Update benchmarks and docs to describe the consolidated query API and run
  the focused query/2D benchmark smoke after the `cap_sys_nice` fix.
- [x] Run focused query tests, full `Release`, full `ReleaseLean`, and
  `git diff --check` before closing the phase.

**Acceptance Bar:**

- Public query access is discoverable from `GravitasWorldContext` as
  `Query2D` and `Query3D`; old `Raycasts` and `CircleQueries` surfaces are
  gone.
- 2D query methods no longer live on `GravitasPhysics2DService`, and 2D hit
  math no longer lives in the collision-contact detector.
- Existing 2D and 3D query behavior remains covered by tests, including layer
  filtering, duplicate suppression, stable ordering, and zero-length/no-hit
  behavior.
- Query benchmarks compile and run with the consolidated API.

**Phase 9I Status - 2026-05-29:**

- Consolidated query-facing APIs under `GravitasWorldContext.Query2D` and
  `GravitasWorldContext.Query3D`.
- Moved old `src/Gravitas/Raycasting` files into `src/Gravitas/Queries`, with
  `GravitasQuery3DService` split into ray/sweep and circle partials.
- Moved pure 2D overlap-circle/raycast query buffers and methods out of
  `GravitasPhysics2DService` into `GravitasQuery2DService`.
- Moved 2D query hit math out of `CollisionDetection2D` into
  `QueryDetection2D`.
- Renamed public 3D query hit types from `LSRaycastHit` to `Physics3DHit` and
  the sorter from `RaycastHitSorter` to `Physics3DHitSorter`.
- Updated tests, benchmarks, README, AGENTS, and wiki docs to use the
  consolidated query domain.
- Verified with focused query/regression tests, full `Release` tests, full
  `ReleaseLean` tests, `git diff --check`, and a short `physics-2d` plus
  `query-service` benchmark smoke. BenchmarkDotNet still reports the
  environment's priority-elevation warning even after the attempted
  `cap_sys_nice` fix, but all 21 benchmark cases executed and exported.

## Phase 9J: 2D Collider State And Dispatch Parity

**Purpose:** Bring pure 2D collider state management, pair ownership, hierarchy
exclusion, and collision dispatch up to the same architectural standard as the
hardened 3D path before mixed 2D/3D work starts. This phase should avoid
copying 3D state types into parallel 2D-only versions when the existing state is
dimension-agnostic or can be generalized cleanly.

**Reuse Guidance:**

- Reuse `ColliderQueryState` directly if it still only carries query stamps and
  no spatial dimensionality.
- Reuse or generalize `ColliderHierarchyState` instead of duplicating it. It has
  no inherent 3D data, but its current type references may need a narrow shared
  abstraction, small generic wrapper, or focused overloads.
- Reuse the dirty/version semantics from `ColliderRuntimeShapeState`. If the
  3D `ColliderShapeSnapshot` payload makes direct reuse awkward, prefer a small
  shared snapshot-comparer pattern plus a focused 2D snapshot over a copy/paste
  class with drift-prone logic.
- Reuse or generalize `ColliderPairState` if doing so keeps pair tracking clear.
  A focused 2D pair state is acceptable only if the 3D `CollisionPair` typing
  makes a shared implementation more complex than the behavior it saves.
- Keep `ColliderPartitionState2D` 2D-specific unless the 3D and 2D partition
  coordinate state can share behavior without hiding X/Z projection rules.

**Files:**

- Modify: `src/Gravitas/Colliders/Primitives2D/LSCollider2D.cs`
- Modify: `src/Gravitas/Colliders/Support/ColliderQueryState.cs`
- Potentially modify/generalize:
  `src/Gravitas/Colliders/Support/ColliderHierarchyState.cs`
- Potentially modify/generalize:
  `src/Gravitas/Colliders/Support/ColliderPairState.cs`
- Potentially modify/generalize:
  `src/Gravitas/Colliders/Support/ColliderRuntimeShapeState.cs`
- Potentially create:
  `src/Gravitas/Colliders/Support/ColliderShapeSnapshot2D.cs`
- Potentially create:
  `src/Gravitas/Colliders/Support/ColliderSettings2D.cs`
- Potentially create:
  `src/Gravitas/CollisionHandling/Detection/CollisionType2D.cs`
- Modify: `src/Gravitas/CollisionHandling/Pairs/CollisionPair2D.cs`
- Modify: `src/Gravitas/CollisionHandling/Detection/CollisionDetection2D.cs`
- Modify: `src/Gravitas/Core/GravitasPhysics2DService.cs`
- Modify: `tests/Gravitas.Tests/Physics2D`
- Modify: `tests/Gravitas.Benchmarks/Physics2D/Physics2DBenchmarks.cs`
- Modify: `docs/wiki`

**Tasks:**

- [x] Add focused tests first for 2D collider state behavior: unchanged
  colliders should skip shape rebuild/partition refresh work, position or
  rotation changes should dirty the runtime shape, shape mutations should dirty
  bounds, and query stamps should suppress duplicate candidates deterministically.
- [x] Reuse `ColliderQueryState` in `LSCollider2D` and generalize/reuse
  `ColliderHierarchyState`, `ColliderPairState`, and runtime-shape dirty state
  only where that keeps the code smaller and clearer than a 2D-specific copy.
- [x] Add `LSCollider2D.Priority` and a compact 2D collider-settings dispatch
  table so shape ordering and pair ordering are explicit instead of inferred
  from type-check conditionals.
- [x] Add `CollisionType2D` and route `CollisionDetection2D.TryCollide(...)`
  through a resolved work item or collision type, matching the 3D dispatch
  pattern closely enough that new 2D shape pairs do not grow conditional soup.
- [x] Refactor `CollisionPair2D` to store deterministic collider order,
  resolved collision type, and any pair-state links needed for cheap removal
  from involved colliders.
- [x] Add 2D hierarchy/self-exclusion parity for same-agent, parent/child, and
  sibling relationships using shared hierarchy state where practical.
- [x] Use 2D pair state to avoid broad O(total-pairs) removal scans when a
  collider leaves the simulation or changes identity.
- [x] Keep disabled paths allocation-conscious and avoid LINQ/iterator
  allocations in per-frame, per-pair, or per-collider paths.
- [x] Update 2D benchmarks to cover collider churn, pair cleanup, and dense
  collision dispatch after the state refactor.
- [x] Update `docs/wiki` if the 2D collider lifecycle, hierarchy exclusions, or
  collision dispatch model changes.
- [x] Run focused 2D tests, full `Release`, full `ReleaseLean`, relevant
  benchmarks, and `git diff --check`.

**Acceptance Bar:**

- 2D collider state uses shared dimension-free helpers where practical and only
  introduces 2D-specific state where the payload is genuinely dimensional.
- Unchanged 2D colliders do not rebuild runtime shape state or refresh
  partition coverage every frame.
- 2D pair cleanup scales with pairs owned by the affected colliders where pair
  state is available, not with every active 2D pair.
- 2D collision dispatch is driven by a resolved collision type/work item rather
  than public type-check conditionals.
- Same-agent, parent/child, and sibling exclusion behavior is covered for 2D.
- Tests and benchmarks demonstrate the refactor did not regress deterministic
  2D contact ordering or broad-phase behavior.

**Phase 9J Status - 2026-05-29:**

- Reused shared dimension-free collider helpers in the 2D path:
  `ColliderQueryState`, generic `ColliderHierarchyState<TCollider>`, generic
  `ColliderPairState<TPair>`, and generic runtime-shape dirty/version state.
  Only the shape snapshot and partition-coordinate payloads remain 2D-specific.
- Added `ColliderSettings2D`, `CollisionType2D`, and `CollisionWorkItem2D`.
  Pure 2D narrow phase now dispatches from a resolved collision type instead of
  growing public type-check conditionals.
- `LSCollider2D` now owns runtime-shape versions, query stamps, hierarchy
  relationships, pair ownership/holder references, and deterministic priority
  parity with the hardened 3D collider path.
- `CollisionPair2D` now stores deterministic collider order and resolved
  collision type. Pair removal during collider deactivation uses the affected
  collider's owned/holder state instead of scanning all active pairs.
- Query duplicate suppression now stamps candidate colliders with per-query
  versions instead of clearing a context hash set per query.
- Added focused 2D parity tests for unchanged-shape rebuild skipping,
  shape/position dirtying, query stamp reset, hierarchy exclusions, pair cleanup,
  and allocation-free warmed pair-owner deactivation.
- Added `SimulateUnchangedColliders` and dense pair-owner deactivation coverage
  to the 2D benchmark suite.
- During allocation hardening, bulk active-partition removal exposed a
  SwiftCollections issue: `SwiftBucket<T>` could still grow its internal
  free-index stack after caller preallocation because `SwiftIntStack` grew when
  requested capacity equaled current capacity. The fix lives in
  `../SwiftCollections` and Gravitas is temporarily local-linked to validate
  against that source until a package version carries the fix.
- Verification completed with focused `Collider2DStateParityTests`, focused
  pure `Physics2D` tests, full Gravitas unit tests in `Release` and
  `ReleaseLean`, Gravitas library builds in `Release` and `ReleaseLean`,
  SwiftCollections `Release` and `ReleaseLean` test gates, two short
  `physics-2d` benchmark smokes, and `git diff --check`. Because the temporary
  local SwiftCollections project reference can trigger the known `.slnx`
  transitive configuration issue, the final Gravitas gates were run at the
  project level so SwiftCollections built in the matching configuration.

## Phase 9K: 2D Continuous Collision

**Purpose:** Add pure 2D continuous collision detection through the existing
`ContinuousCollisionMode` concept so fast 2D bodies do not tunnel through
static or kinematic 2D colliders before mixed 2D/3D integration begins.

**Files:**

- Modify: `src/Gravitas/Core/StiffBody2D.cs`
- Modify: `src/Gravitas/Queries/GravitasQuery2DService.cs`
- Potentially modify/create:
  `src/Gravitas/Queries/QueryDetection2D.cs`
- Modify: `src/Gravitas/Settings/PhysicsSettings.cs` only if the existing
  context default cannot be reused cleanly.
- Potentially create:
  `tests/Gravitas.Tests/Physics2D/ContinuousCollision2DTests.cs`
- Modify: `tests/Gravitas.Benchmarks/Physics2D/Physics2DBenchmarks.cs`
- Modify: `docs/wiki/QUERY_SERVICES.md`
- Modify: `docs/wiki/RUNTIME_ARCHITECTURE.md`

**Tasks:**

- [x] Add `ContinuousCollisionMode` support to `StiffBody2D`, defaulting to
  `Inherit`, and include it in Chronicler state recording if body state
  recording already covers the equivalent 3D property.
- [x] Resolve the effective 2D CCD mode from the body, then applicable hierarchy
  state when available from Phase 9J, then the context default. Final unresolved
  `Inherit` should behave as `Discrete`.
- [x] Add caller-buffered `Query2D.SweepCircle(...)` and
  `Query2D.SweepCircleAll(...)` APIs with deterministic ordering, layer
  filtering, trigger filtering, self/hierarchy exclusion, and duplicate
  suppression.
- [x] Use a conservative deterministic proxy radius per shape: circle radius,
  AABB half-diagonal, and polygon maximum local/world vertex distance from the
  collider center.
- [x] In `StiffBody2D.LateSimulate`, compute the proposed movement, resolve CCD
  mode before expensive query setup, sweep when required, and commit the clipped
  position before publishing visualization state.
- [x] Implement `Auto` mode so it only sweeps when displacement squared exceeds
  the deterministic proxy-radius threshold; keep `Discrete` behavior unchanged.
- [x] Start with static/kinematic target coverage, mirroring the current 3D CCD
  maturity, and document dynamic-vs-dynamic CCD as a later hardening target if
  not solved in this phase.
- [x] Add tests for fast circle-vs-circle, circle-vs-AABB, and circle-vs-polygon
  tunneling prevention, plus no-hit, zero-displacement, layer filtering,
  trigger filtering, self-exclusion, and deterministic hit ordering.
- [x] Add benchmark coverage for 2D CCD cost in no-hit, sparse-hit, and
  dense-hit scenes.
- [x] Update query/runtime wiki docs for the 2D sweep contract and CCD limits.
- [x] Run focused 2D CCD tests, full `Release`, full `ReleaseLean`, relevant
  benchmarks, and `git diff --check`.

**Acceptance Bar:**

- `StiffBody2D` exposes and records effective CCD behavior consistently with
  the 3D body path.
- Fast 2D movers do not tunnel through covered static or kinematic 2D targets
  under `Continuous` mode.
- `Auto` mode avoids sweep work below the proxy threshold and performs sweep
  work above it.
- `Discrete` mode remains unchanged and covered by tests.
- 2D sweep queries are caller-buffered, deterministic, and discoverable through
  `GravitasWorldContext.Query2D`.

**Phase 9K Implementation Notes:**

- `StiffBody2D.ContinuousCollisionMode` now mirrors the 3D body contract and is
  recorded through Chronicler state. Effective mode resolves from the body, then
  the cached collider hierarchy top parent when present, then
  `PhysicsSettings.DefaultContinuousCollisionMode`, with unresolved `Inherit`
  falling back to `Discrete`.
- `Query2D.SweepCircle` and `SweepCircleAll` now provide caller-buffered pure 2D
  swept-circle queries over circles, AABBs, and convex polygons. Query results
  use deterministic distance/collider-ID ordering and support layer, trigger,
  self, same-agent, and hierarchy exclusion filters.
- `StiffBody2D.LateSimulate` computes proposed movement, exits before query work
  when mode/proxy/displacement rules do not require CCD, and clips movement
  before committing authoritative position. The initial scope matches 3D CCD
  maturity: dynamic movers sweep against static, bodyless, immovable, or
  kinematic 2D targets; dynamic-vs-dynamic TOI remains a later hardening item.
- Added focused unit coverage for circle, AABB, polygon, default inheritance,
  `Auto`, `Discrete`, filtering, deterministic result ordering, and warm
  allocation behavior. Added benchmark coverage for no-hit, sparse-hit, and
  dense-hit swept-circle query scenes.

## Phase 10: Mixed 2D/3D Interaction Model

**Purpose:** Implement the first deterministic mixed-dimension runtime so 2D
and 3D bodies can coexist, collide, wake, and exchange constrained impulses in
one `GravitasWorldContext`. This is an alpha feature goal, not a post-alpha
placeholder.

**Completion Target:** By the end of Phase 10, `PhysicsRuntimeMode.Both`
should run the existing pure 2D and pure 3D runtimes in one context without
cross-dimensional contacts, while `PhysicsRuntimeMode.Mixed` should run both
dimensional runtimes plus a dedicated mixed collision path. Pure `TwoD` and
`ThreeD` modes must keep their current costs and behavior. Mixed collision must
be explicit, deterministic, tested, benchmarked, and documented.

**Runtime Mode Contract:** Convert `PhysicsRuntimeMode` into a validated
bitmask instead of adding a second runtime-capability enum:

```csharp
[Flags]
public enum PhysicsRuntimeMode : byte
{
    None = 0,
    TwoD = 1 << 0,
    ThreeD = 1 << 1,
    Both = TwoD | ThreeD,
    Mixed = Both | (1 << 2)
}
```

`None` and arbitrary bit patterns are invalid settings values. The only valid
public settings choices are `TwoD`, `ThreeD`, `Both`, and `Mixed`. Runtime
phase checks should use small direct bitwise helpers such as `Runs2D`,
`Runs3D`, and `RunsMixedContacts`; avoid `Enum.HasFlag` and avoid reintroducing
a separate `PhysicsDimension` or runtime-rule abstraction.

**Design Decision:** Mixed 2D colliders are embedded into 3D as finite,
plane-constrained solids for mixed contacts only. The 2D shape still owns its
authoritative `Vector2d` X/Z state and 2D yaw. For mixed broad phase and
contacts, each 2D collider gets a deterministic Y-centered slab using the host
`FixedTransform.Position.y` as the slab center and a positive mixed half
thickness. Circles become finite vertical cylinders, AABBs become finite
rectangular prisms, and convex polygons become finite convex prisms. Pure 2D
collision and query behavior continues to use the existing X/Z 2D path and must
not pay mixed-mode cost.

**Impulse Rule:** 2D bodies are constrained to their plane. Mixed response
decomposes contact impulses into planar X/Z and vertical Y components:

- planar impulse and correction can affect both movable 3D and movable 2D
  bodies according to mass and kinematic/immovable state.
- vertical impulse and correction affect the 3D body against the 2D slab; the
  2D body has infinite constrained mass along Y and never gains height.
- yaw/angular effects on 2D bodies are allowed only from planar contact
  tangents once the 2D angular model is explicitly implemented. Until then,
  mixed response should not invent hidden 2D angular state.

**Files:**

- Modify: `src/Gravitas/Settings/PhysicsRuntimeMode.cs`
- Modify: `src/Gravitas/Settings/PhysicsSettings.cs`
- Modify: `src/Gravitas/Runtime/GravitasWorldContext.cs`
- Modify: `src/Gravitas/Core/GravitasPhysicsService.cs`
- Modify: `src/Gravitas/Core/GravitasPhysics2DService.cs`
- Modify: `src/Gravitas/Core/StiffBody.cs`
- Modify: `src/Gravitas/Core/StiffBody2D.cs`
- Modify: `src/Gravitas/Colliders/Primitives/LSCollider.cs`
- Modify: `src/Gravitas/Colliders/Primitives2D/LSCollider2D.cs`
- Potentially create: `src/Gravitas/Core/GravitasMixedCollisionService.cs`
- Potentially create: `src/Gravitas/Partitions/PhysicsMixedPartition.cs`
- Potentially create: `src/Gravitas/CollisionHandling/Pairs/CollisionPairMixed.cs`
- Potentially create: `src/Gravitas/CollisionHandling/Contacts/MixedContact.cs`
- Potentially create: `src/Gravitas/CollisionHandling/Detection/CollisionDetectionMixed.cs`
- Potentially create:
  `src/Gravitas/CollisionHandling/Response/CollisionResponseMixed.cs`
- Potentially create: `src/Gravitas/CollisionHandling/Mixed/MixedColliderKey.cs`
- Potentially create:
  `src/Gravitas/CollisionHandling/Mixed/MixedDimensionContactContext.cs`
- Modify: `src/Gravitas/Queries`
- Modify: `tests/Gravitas.Tests/Runtime`
- Create: `tests/Gravitas.Tests/MixedDimensions`
- Modify: `tests/Gravitas.Benchmarks`
- Modify: `docs/wiki/OVERVIEW.md`
- Modify: `docs/wiki/DIMENSIONS.md`
- Modify: `docs/wiki/RUNTIME_ARCHITECTURE.md`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`
- Modify: `docs/wiki/QUERY_SERVICES.md`

**Implementation Phases:**

- [x] **Phase 10A - Bitmask runtime contract:** Convert
  `PhysicsRuntimeMode` to a validated `[Flags]` bitmask, add `Both` and
  `Mixed`, update settings validation, replace `if ThreeD else TwoD` lifecycle
  branching with direct bitwise helpers, and test all four valid runtime modes.
  `Both` must advance 3D simulation, pure 2D simulation, visualization, hooks,
  reset, and context-local state without mixed collision distribution or mixed
  response. `Mixed` must do the same plus the dedicated mixed collision path.
  `TwoD` and `ThreeD` behavior and cost boundaries must remain unchanged.
- [x] **Phase 10A - Embedding contract:** Add the minimal mixed embedding data
  needed by `LSCollider2D`: a positive half-thickness resolved from collider
  override or context default, a cached slab center Y from the host transform,
  and deterministic mixed 3D bounds. Do not reintroduce `PhysicsDimension` or
  public `Physics2DBounds`.
- [x] **Phase 10B - Mixed broad phase:** Add a mixed broad-phase service backed
  by the existing `GridWorld` and stable voxel identities. Store separate 3D
  collider IDs and 2D collider IDs so ID spaces never alias. Use awake-dynamic
  gating, same-agent/hierarchy/layer filtering, deterministic partition
  ordering, deterministic pair keys, and retained empty partition cleanup. Avoid
  O(3D * 2D) scans except in tiny test-only helpers.
- [x] **Phase 10B - Broad-phase tests and benchmarks:** Add sparse, dense,
  large-world, sleeping, trigger, layer, hierarchy, and churn tests. Add
  benchmarks for mixed candidate gathering, pair distribution, and retained
  partition cleanup at 64, 1024, and larger representative collider counts.
- [ ] **Phase 10C - Mixed narrow phase primitives:** Add mixed shape tests and
  detection for 3D sphere, cuboid, capsule, and finite cylinder against 2D
  circle, AABB, and convex polygon slabs. Cover separated Y slabs, touching slab
  faces, planar side contacts, edge/corner contacts, starting overlap, rotated
  3D shapes, rotated 2D convex polygons, and deterministic contact ordering.
- [ ] **Phase 10C - Mixed narrow phase complex shapes:** Extend mixed detection
  to 3D compound colliders and mesh colliders only through explicit tested
  policies. Compound colliders should scan stable part order and emit one mixed
  pair surface. Mesh support should use local BVH candidate gathering and
  triangle-level tests; if the triangle policy is not robust enough, record the
  exact unsupported mesh combinations in this phase's follow-up notes rather
  than silently falling through.
- [ ] **Phase 10D - Mixed pair and response model:** Add `CollisionPairMixed`
  and mixed contacts with stable pair identity, one-sided pair ownership, pooled
  contacts, wake propagation, resting-pair retention, enter/stay/exit events,
  and deterministic response order. Keep 2D vertical constraint explicit:
  planar impulse can move a 2D body, vertical impulse cannot.
- [ ] **Phase 10D - Response tests:** Add tests for dynamic 3D vs static 2D
  platform, dynamic 2D pushed by dynamic 3D planar contact, immovable and
  kinematic participants, bodyless mixed triggers, sleeping wake propagation,
  same-agent/hierarchy exclusion, layer matrix filtering, and repeated replay
  determinism.
- [ ] **Phase 10E - Mixed query and CCD policy:** Decide and implement the
  smallest discoverable query surface needed for mixed mode. Recommended start:
  keep `Query2D` and `Query3D` pure by default, then add explicit mixed query
  APIs or opt-in flags only where tests prove hosts need them. Add 3D swept
  sphere vs embedded 2D slabs and 2D swept circle vs 3D primitive coverage for
  CCD after discrete mixed response is stable.
- [ ] **Phase 10E - Diagnostics and debug draw:** Extend diagnostics so mixed
  pairs report both collider IDs with dimension tags, mixed contact points,
  normals, penetration/correction, and constrained impulse components. Debug
  draw should show the 2D slab/prism used for mixed collision, not just the pure
  2D outline.
- [ ] **Phase 10F - Documentation and hardening:** Update wiki pages with the
  mixed runtime mode, embedding rule, constrained impulse model, supported shape
  matrix, unsupported combinations, query/CCD contract, and host integration
  guidance. Add benchmark notes and follow-up actions for any shape pair or
  solver behavior that remains too expensive or physically weak for alpha.
- [ ] **Phase 10F - Verification:** Run focused mixed-dimension tests, focused
  2D and 3D regression tests, full `Release`, full `ReleaseLean`, mixed
  benchmark smoke, and `git diff --check`.

**Acceptance Bar:**

- `PhysicsRuntimeMode` is a validated bitmask with exact public settings values
  for `TwoD`, `ThreeD`, `Both`, and `Mixed`; invalid bit combinations are
  rejected.
- `PhysicsRuntimeMode.Both` is an implemented runtime path that advances pure
  2D and pure 3D services without mixed broad phase, mixed queries, mixed
  response, or cross-dimensional contacts.
- `PhysicsRuntimeMode.Mixed` is an implemented runtime path, not a dormant enum
  value.
- Pure `TwoD` and `ThreeD` modes keep their current behavior and do not run
  mixed broad phase, mixed queries, or mixed response.
- Mixed broad phase uses GridForge-backed spatial identity with deterministic
  ordering and avoids whole-world cross products.
- 2D colliders expose an explicit finite mixed slab/prism for mixed contacts
  while preserving pure 2D collision semantics.
- Mixed pair identity cannot alias 2D and 3D collider IDs.
- Mixed contacts are deterministic, physically explainable, and preserve the 2D
  body's plane constraint.
- Layer, trigger, bodyless, kinematic, immovable, sleeping, same-agent, and
  hierarchy behavior is covered for mixed pairs.
- Benchmarks exist for sparse and dense mixed scenes before making performance
  claims.
- Unsupported combinations are explicit, tested, and documented.

**Implementation Notes:**

- Do not register `LSCollider2D` as an `LSCollider` or copy it into the 3D
  collider table. Mixed services should resolve through the owning 2D and 3D
  services using dimension-tagged keys.
- Keep runtime-mode checks cheap and explicit. Use direct bitwise tests against
  the `[Flags]` mode value rather than `Enum.HasFlag`, LINQ, or a second
  capability enum. `Both` means pure 2D plus pure 3D only; only `Mixed` enables
  mixed contacts.
- Do not treat the pure 2D Y=0 partition plane as physical thickness. Mixed
  embedding uses the host transform Y and mixed half-thickness.
- Prefer one dedicated mixed service over hidden conditionals inside every pure
  2D and pure 3D hot path. Shared generic helpers are welcome where they reduce
  duplication without obscuring dimensional rules.
- If mixed response ordering between 3D/3D, 2D/2D, and 2D/3D pairs changes body
  outcomes, make the ordering explicit and test replay stability. A later
  unified island solver can replace the phase order only with tests and
  benchmarks.
- Treat mixed CCD as follow-on work inside Phase 10 after discrete mixed
  collision is stable. It should reuse the same embedding and pair filtering
  rules rather than inventing a second mixed-shape interpretation.

**Phase 10B Result:**

- Added `GravitasMixedCollisionService` as a real mixed broad-phase owner with
  GridForge-backed `PhysicsMixedPartition` payloads, separate 3D and 2D
  collider ID spaces, stable `MixedColliderKey` output, duplicate suppression,
  awake-dynamic gating, layer filtering, same-agent exclusion, and retained
  empty-partition cleanup.
- Added separate mixed partition state on `LSCollider` and `LSCollider2D` so
  mixed membership does not overload the pure 3D or pure 2D partition paths.
- Added focused tests for sparse, dense, large-world, sleeping, trigger, layer,
  same-agent, and churn/retained-cleanup behavior. Cross-dimensional hierarchy
  is currently represented by same-agent exclusion because existing collider
  hierarchy state is dimension-local; do not claim a public cross-dimensional
  hierarchy API until a host-level relationship contract exists.
- Added `MixedBroadPhaseBenchmarks` with sparse gathering, dense gathering, and
  retained cleanup/churn coverage at 64, 1024, and 4096 colliders.

## Phase 11: Serialization, Snapshots, And Deterministic Replay

**Purpose:** Make Chronicler state transfer and deterministic replay confidence strong enough for lockstep debugging and rollback-style validation.

**Files:**

- Modify: `src/Gravitas/Core/StiffBody.cs`
- Modify: `src/Gravitas/Colliders/LSCollider.cs`
- Modify: `src/Gravitas/Settings`
- Potentially create: `tests/Gravitas.Tests/Serialization`
- Potentially create: `tests/Gravitas.Tests/Replay`
- Modify: `docs/wiki/HOST_INTEGRATION.md`
- Modify: `docs/wiki/RUNTIME_ARCHITECTURE.md`

**Tasks:**

- [ ] Inventory all authoritative runtime state that must survive populate-existing-instance serialization.
- [ ] Add round-trip tests for body state, collider state, settings, layer matrices, ground probe settings, and collision/query-relevant state.
- [ ] Add replay tests that serialize at frame N, populate a fresh host-created shell, continue simulation, and compare against uninterrupted simulation.
- [ ] Verify `ReleaseLean` when touching MemoryPack-adjacent or conditional serialization code.
- [ ] Document what Gravitas serializes, what the host must bind externally, and what is intentionally presentation-only.

## Phase 12: Diagnostics Payloads, Host Adapters, And Tooling Samples

**Purpose:** Keep diagnostics engine-agnostic while making them easier for real hosts to consume.

**Files:**

- Modify: `src/Gravitas/Diagnostics`
- Potentially create: `docs/wiki/DIAGNOSTIC_ADAPTERS.md`
- Modify: `tests/Gravitas.Tests/Diagnostics`
- Modify: `tests/Gravitas.Benchmarks/Diagnostics`
- Modify: `docs/wiki/DIAGNOSTICS.md`

**Tasks:**

- [ ] Review whether generic `ScalarA`, `ScalarB`, `DataA`, and `DataB` fields are sufficient or whether typed event payload helpers would reduce adapter mistakes.
- [ ] Add tests for diagnostic frame boundaries, `Clear()`/`Disable()` semantics, capacity preservation, and high-volume mesh capture.
- [ ] Create renderer-neutral sample adapter docs for Unity-style debug draw, server logs, and replay timeline capture without adding engine references to `src/Gravitas`.
- [ ] If `SwiftCollections.Observable` is used for tooling projection, keep it outside authoritative runtime paths and benchmark notification cost.
- [ ] Keep disabled diagnostics at zero managed allocation in benchmark smoke.

## Verification Gate For Every Phase

- [ ] Run focused tests for the changed subsystem.
- [ ] Run `dotnet build Gravitas.slnx --configuration Release`.
- [ ] Run `dotnet test Gravitas.slnx --configuration Release --no-build`.
- [ ] Run `dotnet build Gravitas.slnx --configuration ReleaseLean` when settings, serialization, package references, or MemoryPack-adjacent code changes.
- [ ] Run `dotnet test Gravitas.slnx --configuration ReleaseLean` when the Lean build is touched.
- [ ] Run the relevant benchmark aliases for hot-path, data-structure, query, collision, partition, diagnostics, or solver changes.
- [ ] Update `docs/wiki/`, `README.md`, benchmark docs, and this plan status when behavior, architecture, or workflow changes.
