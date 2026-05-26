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
- Potentially create: `src/Gravitas/Colliders/ColliderRuntimeShapeState.cs`
- Potentially create: `src/Gravitas/Colliders/ColliderPartitionState.cs`
- Potentially create: `src/Gravitas/Colliders/ColliderPairState.cs`
- Modify: `tests/Gravitas.Tests/Colliders`
- Modify: `tests/Gravitas.Benchmarks/Colliders`
- Modify: `docs/wiki/RUNTIME_ARCHITECTURE.md`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`

**Tasks:**

- [ ] Map `LSCollider` responsibilities into identity, host binding, runtime shape, partition state, query versioning, hierarchy filtering, and pair references.
- [ ] Add regression tests around parent/child collision exclusion, collider deactivation cleanup, pair-holder cleanup, partition refresh, and query-version reset before moving state.
- [ ] Extract only the state groups that reduce real complexity without creating indirection-heavy API bloat.
- [ ] Preserve disabled/allocation hot paths for collider simulation and partition refresh.
- [ ] Revisit parent/child metadata and document the engine-agnostic hierarchy rule that replaced Unity transform traversal.

## Phase 3: Contact Manifold Data Model

**Purpose:** Replace single-contact response assumptions with deterministic manifold data that can support stacking, friction, warm starting, and mesh contacts.

**Files:**

- Modify: `src/Gravitas/CollisionHandling/Support/ContactPoint.cs`
- Potentially create: `src/Gravitas/CollisionHandling/Support/ContactManifold.cs`
- Potentially create: `src/Gravitas/CollisionHandling/Support/ManifoldContact.cs`
- Modify: `src/Gravitas/CollisionHandling/CollisionPair.cs`
- Modify: `src/Gravitas/CollisionHandling/CollisionDetection.cs`
- Modify: `tests/Gravitas.Tests/CollisionHandling`
- Modify: `tests/Gravitas.Benchmarks/CollisionHandling`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`

**Tasks:**

- [ ] Define deterministic manifold identity, contact ordering, maximum contact count, point lifetime, and reduction policy.
- [ ] Add tests for zero-depth touching contacts, stacked contacts, edge/face contacts, reversed pair ordering, and stable contact order across repeated runs.
- [ ] Start with primitive pairs where manifold candidates are easiest to reason about: sphere/sphere, cuboid/sphere, cuboid/cuboid, capsule/capsule, and cylinder/cylinder.
- [ ] Keep legacy single-contact behavior as an internal compatibility path only while tests transition, then remove it if the manifold path fully replaces it.
- [ ] Add benchmarks that compare single-contact and manifold generation cost by shape family.
- [ ] Document manifold limits and known unsupported mesh manifold behavior.

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

- [ ] Add solver tests for immovable bodies, kinematic bodies, equal mass exchange, different mass exchange, restitution thresholding, resting contact, slopes, and stacks.
- [ ] Define normal and tangential impulse equations in fixed-point terms, including units and clamping rules.
- [ ] Implement deterministic friction impulses after manifold contact data is available.
- [ ] Add positional stabilization that does not hide narrow-phase penetration-depth bugs.
- [ ] Benchmark solver cost by contact count and pair count.
- [ ] Document solver invariants, remaining divergences from real-world physics, and any deliberate simplifications.

## Phase 5: Island Solving, Sleep, And Warm Starting

**Purpose:** Improve stability and cost for resting scenes while preserving deterministic ordering.

**Files:**

- Modify: `src/Gravitas/Core/GravitasPhysicsService.cs`
- Modify: `src/Gravitas/CollisionHandling/CollisionPair.cs`
- Potentially create: `src/Gravitas/CollisionHandling/Solver/PhysicsIsland.cs`
- Potentially create: `src/Gravitas/CollisionHandling/Solver/IslandBuilder.cs`
- Modify: `src/Gravitas/Core/StiffBody.cs`
- Modify: `tests/Gravitas.Tests/CollisionHandling`
- Modify: `tests/Gravitas.Benchmarks/Core`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`

**Tasks:**

- [ ] Define deterministic island membership, island ordering, pair ordering inside an island, and body wake/sleep rules.
- [ ] Add tests for stacked bodies, body wake-up after impulse, sleeping body ignored by solver until disturbed, and stable island ordering across repeated runs.
- [ ] Add warm-start storage keyed by stable pair/manifold contact identity.
- [ ] Ensure sleeping and warm starting never skip collision events or trigger/contact notifications incorrectly.
- [ ] Benchmark large resting scenes before and after island/sleep changes.

## Phase 6: Continuous Collision Detection And Swept Mesh Policy

**Purpose:** Move from query-only swept sphere support toward collision-time continuous collision detection for fast movers.

**Files:**

- Modify: `src/Gravitas/Raycasting/SweptSphereQueryWorker.cs`
- Potentially create: `src/Gravitas/CollisionHandling/Continuous`
- Modify: `src/Gravitas/CollisionHandling/CollisionDetection.cs`
- Modify: `src/Gravitas/Core/StiffBody.cs`
- Modify: `tests/Gravitas.Tests/Raycasting`
- Modify: `tests/Gravitas.Tests/CollisionHandling`
- Modify: `tests/Gravitas.Benchmarks/Raycasting`
- Modify: `docs/wiki/QUERY_SERVICES.md`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`

**Tasks:**

- [ ] Decide whether CCD lives in collision detection, body integration, or a dedicated continuous-collision service.
- [ ] Add tunneling tests for fast sphere/capsule/cuboid/cylinder bodies against thin static geometry.
- [ ] Define time-of-impact ordering and response handoff for multiple hits in one frame.
- [ ] Decide the alpha policy for swept mesh targets: unsupported, triangle-sweep query path, convex-decomposed mesh path, or dedicated CCD mesh path.
- [ ] If mesh sweep is implemented, require triangle candidate acceleration tests and benchmarks before enabling it broadly.
- [ ] Document the CCD contract and any excluded shape pairs.

## Phase 7: Mesh Collider Alpha Policy And Dynamic Mesh Boundaries

**Purpose:** Keep mesh support useful without letting arbitrary dynamic non-convex mesh behavior become an unbounded alpha blocker.

**Files:**

- Modify: `src/Gravitas/Colliders/Support/PhysicsMesh/PhysicsMesh.cs`
- Modify: `src/Gravitas/Colliders/Primitives/LSMeshCollider.cs`
- Modify: `src/Gravitas/CollisionHandling/CollisionDetection.cs`
- Potentially create: `src/Gravitas/Colliders/Support/PhysicsMesh/ConvexMeshPolicy.cs`
- Modify: `tests/Gravitas.Tests/Colliders`
- Modify: `tests/Gravitas.Tests/CollisionHandling`
- Modify: `tests/Gravitas.Benchmarks/Colliders`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`

**Tasks:**

- [ ] Define alpha mesh categories: static triangle mesh, convex mesh, decomposed convex mesh, and unsupported dynamic non-convex mesh.
- [ ] Add tests for invalid mesh input, rotated mesh bounds, dynamic mesh movement, mesh/collider contact order, and non-convex policy enforcement.
- [ ] Decide whether the mesh edge cache remains required after manifold work. Remove it only if tests and benchmarks prove face normals/on-demand edges cover all callers.
- [ ] Add benchmark coverage for mesh construction, BVH build, triangle query windows, dynamic mesh repartitioning, and mesh contact generation.
- [ ] Document the mesh policy in host-facing terms so engine adapters know what data to provide.

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
