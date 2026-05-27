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
- `src/Gravitas/Runtime/GravitasWorldContext.cs`
- `src/Gravitas/Core/GravitasPhysicsService.cs`
- `src/Gravitas/Core/StiffBody.cs`
- `src/Gravitas/Colliders/LSCollider.cs`
- `src/Gravitas/Colliders/Primitives`
- `src/Gravitas/CollisionHandling`
- `src/Gravitas/Raycasting`
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
| `docs/wiki/OVERVIEW.md` | First-class 2D and mixed 2D/3D are goals, not current guarantees. | Phases 9-10 |
| `docs/wiki/DIAGNOSTICS.md` | Generic diagnostic payloads may need richer typed events; host adapters remain outside core. | Phase 12 |
| `docs/wiki/HOST_INTEGRATION.md` | Serialization is populate-existing-shell behavior and remains experimental. | Phase 11 |
| Prior plan recommendations | `LSCollider` owns too many responsibilities. | Phase 2 |
| Prior plan recommendations | Consume the next fixed GridForge package and revisit partition allocation without local project-link scaffolding. | Phase 0 and Phase 8 |
| Prior plan recommendations | Prefer FixedMathSharp geometry and SwiftCollections fixed-query structures before adding local spatial math. | All algorithm phases |

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
- Create: `src/Gravitas/CollisionHandling/Support/ContactManifold.cs`
- Create: `src/Gravitas/CollisionHandling/Support/ManifoldContact.cs`
- Modify: `src/Gravitas/CollisionHandling/CollisionPair.cs`
- Modify: `src/Gravitas/CollisionHandling/CollisionDetection.cs`
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

- Modify: `src/Gravitas/CollisionHandling/CollisionResponse.cs`
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
- Modify: `src/Gravitas/CollisionHandling/CollisionPair.cs`
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

- Modify: `src/Gravitas/Raycasting/SweptSphereQueryWorker.cs`
- Potentially create: `src/Gravitas/CollisionHandling/Continuous`
- Potentially create: `src/Gravitas/CollisionHandling/Continuous/ContinuousCollisionMode.cs`
- Modify: `src/Gravitas/CollisionHandling/CollisionDetection.cs`
- Modify: `src/Gravitas/Core/StiffBody.cs`
- Potentially modify: `src/Gravitas/Settings/PhysicsSettings.cs`
- Modify: `tests/Gravitas.Tests/Raycasting`
- Modify: `tests/Gravitas.Tests/CollisionHandling`
- Modify: `tests/Gravitas.Benchmarks/Raycasting`
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
- Modify: `src/Gravitas/CollisionHandling/CollisionDetection.cs`
- Modify: `src/Gravitas/Raycasting/RaycastSegmentWorker.cs`
- Potentially create: `src/Gravitas/Colliders/Support/PhysicsMesh/ConvexMeshPolicy.cs`
- Potentially create: `src/Gravitas/Colliders/Support/PhysicsMesh/MeshColliderPolicy.cs`
- Potentially create: `src/Gravitas/Colliders/Support/PhysicsMesh/MeshColliderMode.cs`
- Modify: `tests/Gravitas.Tests/Colliders`
- Modify: `tests/Gravitas.Tests/CollisionHandling`
- Modify: `tests/Gravitas.Tests/Raycasting`
- Modify: `tests/Gravitas.Benchmarks/Colliders`
- Reference only: `F:\gamedevrepos\SoulsClone\Library\PackageCache\com.whinarn.unitymeshsimplifier@d741912bfe`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`

**Tasks:**

- [ ] Define alpha shape policy terminology. Mesh modes should use `Convex` and
  `Concave`; `Compound` should mean a separate `LSCompoundCollider` strategy,
  not a mesh mode. Keep `non-convex` as internal validation language for the
  wider set of topology problems such as concavity, open meshes, disconnected
  islands, self-intersection, non-manifold edges, holes, and winding issues.
- [ ] Add an explicit mesh-collider mode or policy. Do not silently infer convex
  versus concave behavior except for cheap validation that proves a declared mode
  is invalid. Developer intent should drive runtime policy.
- [ ] Define the alpha boundary: static/kinematic concave triangle meshes are
  acceptable collision data, convex meshes are the preferred dynamic mesh shape,
  and dynamic concave behavior should route through compound/decomposed-convex
  data unless a raw dynamic concave triangle-mesh policy is explicitly proven.
- [ ] Decide whether alpha accepts only host/offline decomposed convex pieces for
  concave meshes, or whether Gravitas should own deterministic convex
  decomposition later. Do not implement automatic decomposition in 7A unless the
  algorithm, determinism contract, tests, and benchmarks are all explicit.
- [ ] If 7A needs compound-related seams, keep them neutral and minimal:
  declared mesh modes, decomposed-convex policy names, and tests that prove raw
  dynamic concave meshes route to a future compound/decomposition path instead of
  silently behaving like convex meshes.
- [ ] Refactor rigid mesh movement toward local-space acceleration ownership:
  build local vertices, local triangle normals, local bounds, and local triangle
  BVH once; update only transform, inverse transform, and conservative world
  bounds when the mesh moves; rebuild the BVH only when local vertex or triangle
  topology changes.
- [ ] Update mesh query and narrow-phase callers to transform world-space query
  shapes into mesh-local space, query the local triangle BVH, and transform final
  contact points/normals back to world space. Rotation-only normal transforms are
  enough for the current scale model; any future non-uniform mesh scale needs an
  inverse-transpose normal policy.
- [ ] Benchmark dynamic mesh movement before and after the local-space BVH
  refactor. The expected win is avoiding O(triangle count) BVH rebuilds on rigid
  translation/rotation.
- [ ] Add tests for invalid mesh input, declared mesh-mode validation, rotated
  mesh bounds, dynamic mesh movement without triangle BVH rebuild, mesh/collider
  contact order, and concave/non-convex policy enforcement.
- [ ] Add policy tests proving raw dynamic concave meshes are either rejected,
  treated as static-only, or converted into declared convex/compound sub-shapes
  by initialization-time data. Runtime per-frame decomposition is out of scope
  for alpha.
- [ ] Review `PhysicsMesh.CalculateInertiaTensor(...)` and decide the alpha contract: keep the current triangle-area-weighted approximation with documentation, or replace it with a deterministic thin-triangle/shell or tetrahedral volume approximation. Any replacement needs fixed-value tests on known simple meshes and benchmark coverage.
- [ ] Decide whether the mesh edge cache remains required after manifold work. Remove it only if tests and benchmarks prove face normals/on-demand edges cover all callers.
- [ ] Review the old `UnityMeshSimplifier` package and the prototype
  `MeshSimplificationHelper` for design context without vendoring Unity-specific
  runtime code into Gravitas. If temporary scratch notes are created under
  `docs/feature-work/prototype`, remove them before closing the task unless the
  user explicitly asks to keep or force-add them.
- [ ] Decide mesh simplification/LOD policy. Prefer host/offline simplification
  for alpha; any Gravitas-owned runtime simplifier must be deterministic,
  fixed-point, bounded, benchmarked, and must not change collision truth during a
  simulation frame.
- [ ] Decide dynamic mesh update boundaries: rigid transformed meshes should not
  rebuild local BVH/topology data, while deformable/breakable topology or vertex
  updates require explicit invalidation, deterministic rebuild ordering, and
  separate tests before support is claimed.
- [ ] Add benchmark coverage for mesh construction, BVH build, triangle query windows, dynamic mesh repartitioning, and mesh contact generation.
- [ ] Document the mesh policy in host-facing terms so engine adapters know what data to provide.

**Design Notes:**

- Public mesh terminology should prefer `Convex` and `Concave`. `Compound` is a
  collider composition strategy, not a mesh classification. Concave meshes are
  non-convex, but non-convex is broader than concavity and includes invalid or
  unsupported topology. Use `non-convex` in validation notes only when the wider
  meaning is intentional.
- Concave mesh support should remain part of the long-term design, similar to
  the behavior users expect from engines such as Unity. The alpha boundary is
  raw movable concave triangle soup, not concave colliders as a concept.
- Decomposing a concave mesh into convex pieces can feed a later compound
  collider model. For 7A, host/offline decomposition is safer than runtime
  automatic decomposition unless Gravitas owns a deterministic, bounded,
  benchmarked algorithm.
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
- Current `PhysicsMesh.UpdatePosition(...)` transforms every vertex, invalidates
  normals, rebuilds the triangle BVH, and updates bounds for rigid movement.
  That is acceptable prototype behavior, but it is the wrong target for dynamic
  meshes. The preferred design is a local-space BVH with query/collision inputs
  transformed into mesh-local coordinates and final outputs transformed back to
  world space.
- The old Unity Mesh Simplifier package is useful context because it uses a
  quadric-error-style simplification pipeline with smart vertex linking and
  preservation flags. It should stay reference material for now: the package is
  Unity/float/double oriented, so a Gravitas simplifier would need a deliberate
  fixed-point port with deterministic tie-breakers and benchmark gates.

## Phase 7B: Compound Collider Policy And Scaffold

**Purpose:** Define `LSCompoundCollider` as a special-case collider composition
strategy without bloating mesh-policy work. Implement only the minimal scaffold
needed for alpha if the design remains contained; otherwise produce a follow-up
implementation plan before Phase 8.

**Files:**

- Potentially create: `src/Gravitas/Colliders/Primitives/LSCompoundCollider.cs`
- Potentially create: `src/Gravitas/Colliders/Support/Compound`
- Potentially modify: `src/Gravitas/Colliders/LSCollider.cs`
- Potentially modify: `src/Gravitas/CollisionHandling/CollisionDetection.cs`
- Potentially modify: `src/Gravitas/Diagnostics`
- Modify: `tests/Gravitas.Tests/Colliders`
- Modify: `tests/Gravitas.Tests/CollisionHandling`
- Modify: `tests/Gravitas.Benchmarks/Colliders`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`

**Tasks:**

- [ ] Decide whether 7B implements an initial runtime type or only writes the
  policy and follow-up plan. If implemented, keep it narrow: one public collider
  identity/body/layer, stable part IDs, deterministic part ordering, aggregate
  bounds, and part-local transforms.
- [ ] Define compound parts as internal collider-like shape parts, not
  independent host colliders or parent/child objects. They may reuse collider
  shape logic, but the compound owns registration, broad-phase identity, event
  surface, body binding, and lifecycle.
- [ ] Define the supported alpha part set. Prefer primitives and convex meshes
  for dynamic bodies. Concave mesh parts, if allowed, should follow the same
  static/kinematic restrictions as standalone concave meshes.
- [ ] Define compound/decomposed-convex collision rules before exposing an API:
  aggregate mass/inertia policy, part-level broad-phase behavior, pair identity,
  contact manifold reduction, trigger/contact event surface, parent/child
  filtering behavior, CCD proxy behavior, and debug draw representation.
- [ ] Define deterministic contact reduction for overlapping compound parts.
  Internal part overlap is acceptable, but a single opposing collider should not
  receive arbitrary duplicate contacts. Prefer stable part ordering plus a
  physically meaningful best-contact/manifold reduction over "first contact
  found" behavior.
- [ ] Add tests for part ordering, aggregate bounds, overlapping internal parts,
  single external event emission, parent/child hierarchy separation, debug draw
  output, and deterministic contact selection.
- [ ] Add benchmarks for compound-vs-primitive pair checks and broad-phase
  repartitioning when a compound's aggregate bounds span many voxels.
- [ ] Document the difference between compound colliders and parent/child
  collider hierarchy in `docs/wiki/COLLISION_PIPELINE.md`.

**Design Notes:**

- A compound collider is the likely bridge: it behaves like one collider to the
  host and one body to the solver, but internally contains deterministic
  primitive and convex-mesh parts, with possible concave-mesh parts only under a
  declared policy. This is similar in spirit to parent/child collider
  composition, but should have one collider ID, one broad-phase owner, stable
  part ordering, and explicit aggregate mass/inertia rules.
- A compound collider is not the same as a parent/child collider hierarchy. A
  parent/child hierarchy can represent separate host objects, such as a warrior
  and a held sword. A compound collider represents one physical collider whose
  shape is approximated by multiple internal parts.
- A compound collider can provide practical support for concave/non-convex
  meshes when the concave shape is represented as multiple deterministic parts.
  It does not automatically make arbitrary raw concave triangle meshes dynamic-
  safe; each part still needs a declared policy and stable collision ordering.

## Phase 8: Broad-Phase And Query State Scalability

**Purpose:** Tighten partition/query data structures and decide whether current context-owned query services are enough for alpha.

**Files:**

- Modify: `src/Gravitas/Core/GravitasCollisionService.cs`
- Modify: `src/Gravitas/Partitions/PhysicsPartition.cs`
- Modify: `src/Gravitas/Raycasting/GravitasRaycastService.cs`
- Modify: `src/Gravitas/Raycasting/GravitasCircleQueryService.cs`
- Potentially create: `src/Gravitas/Raycasting/QueryState`
- Modify: `tests/Gravitas.Tests/Partitions`
- Modify: `tests/Gravitas.Tests/Raycasting`
- Modify: `tests/Gravitas.Benchmarks/Core`
- Modify: `tests/Gravitas.Benchmarks/Raycasting`
- Modify: `docs/wiki/QUERY_SERVICES.md`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`

**Tasks:**

- [ ] Validate packaged GridForge partition-provider behavior once the retention fix is released; remove temporary local project references when possible.
- [x] Compare `SwiftSparseMap<T>` membership against a future `SwiftSparseSet` or another deterministic sparse membership structure when available. Phase 0 moved partition membership to `SwiftSparseSet`; continue benchmarking the layout under broader partition/query scale tests.
- [ ] Revisit active partition ordering and pair candidate ordering for determinism under high churn.
- [ ] If Phase 8 introduces a hierarchical broad phase, spatial tree, or nested
  acceleration structure above GridForge voxels, revisit awake-state propagation
  from child nodes to parent nodes. A branch with no awake dynamic descendants
  should be skippable during pair generation, but only if tests prove sleeping
  bodies remain visible to queries, wake propagation, and contact lifecycle.
- [ ] Add stress tests for moving many colliders across grids, repeatedly emptying/refilling partitions, and querying colliders spanning many voxels.
- [ ] Decide whether query services should remain context-owned mutable services or expose explicit caller-owned/rented query state for reentrancy.
- [ ] Compare custom ray/segment workers against `FixedRay` for any first-hit or non-allocation query paths where the downstream primitive now fits.
- [ ] Keep all-hit query paths caller-buffered and benchmarked.

## Phase 9: First-Class 2D Physics Foundation

**Purpose:** Stop treating 2D as accidental 3D flattening and define the first explicit 2D runtime model.

**Files:**

- Potentially create: `src/Gravitas/Dimensions`
- Potentially modify: `src/Gravitas/Core/StiffBody.cs`
- Potentially modify: `src/Gravitas/Colliders`
- Modify: `tests/Gravitas.Tests`
- Modify: `tests/Gravitas.Benchmarks`
- Modify: `docs/wiki/OVERVIEW.md`
- Modify: `docs/wiki/RUNTIME_ARCHITECTURE.md`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`

**Tasks:**

- [ ] Write a design note that defines 2D axes, units, body state, collider shape set, rotation model, gravity model, and solver differences.
- [ ] Decide whether 2D uses separate body/collider types, dimension-specific strategies, or shared types with explicit dimension modes.
- [ ] Add tests for pure 2D deterministic integration, collision detection, response, queries, and replay before exposing public APIs.
- [ ] Avoid leaking X/Z-ground-plane assumptions into 2D naming.
- [ ] Benchmark 2D paths separately from 3D paths so 2D does not inherit unnecessary 3D costs.

## Phase 10: Mixed 2D/3D Interaction Model

**Purpose:** Define how 2D and 3D bodies coexist before any mixed collision API ships.

**Files:**

- Modify: `docs/wiki/OVERVIEW.md`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`
- Potentially create: `docs/wiki/DIMENSIONS.md`
- Potentially create: `src/Gravitas/Dimensions/MixedDimensionPolicy.cs`
- Modify: `tests/Gravitas.Tests`
- Modify: `tests/Gravitas.Benchmarks`

**Tasks:**

- [ ] Define the mixed-dimension embedding rule: plane, thickness, projection volume, contact manifold shape, and impulse exchange.
- [ ] Add design tests for 3D sphere/cuboid/capsule/cylinder bodies interacting with 2D circles/boxes/polygons under the chosen rule.
- [ ] Decide whether mixed 2D/3D is an alpha feature, experimental feature flag, or documented post-alpha target.
- [ ] Document unsupported combinations explicitly rather than letting them fall through to accidental 3D behavior.

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
