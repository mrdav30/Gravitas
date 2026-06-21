# Discrete Response And Contact Quality Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Upgrade Gravitas discrete contact response from alpha-solid behavior to a first-class deterministic solver for resting stacks, cached impulse application, contact islands, cylinder contacts, mesh clipping, and mixed-dimension response.

**Architecture:** Keep the current narrow-phase/contact-pair ownership model, then harden the solver around explicit contact identity, stable island ordering, and measurable response quality. CCD-specific island time stepping remains in the CCD service-level island plan; this plan owns ordinary discrete/resting response after contacts exist.

**Tech Stack:** .NET 8, xUnit v3, BenchmarkDotNet, FixedMathSharp, SwiftCollections, Gravitas collision detection/response services.

---

**Date:** 2026-06-21
**Status:** Pre-alpha release blocker
**Owner:** Gravitas response and contact hardening

## Purpose

The current response layer is much stronger than the early prototype: 3D and
pure 2D contacts have deterministic manifolds, friction impulses, material
coefficients, and pair-local warm-start storage. Pure 2D now applies cached
normal and tangent impulses before the fresh solve. Workstream 2 brought 3D
single-pair warm-starting up to the same standard with normal-compatible cache
reuse and a two-axis tangent basis. Resting-stack quality still needs explicit
discrete islands and bounded multi-iteration solving, cylinder edge cases and
mesh contact clipping still need hardening, and mixed response is intentionally
constrained.

Those are not acceptable as loose wiki caveats for a first-class deterministic
physics engine. They should be driven by tests, benchmark signal, and explicit
solver invariants.

## Relationship To Existing Plans

- [`2026-06-21-ccd-service-level-island-solver-plan.md`](2026-06-21-ccd-service-level-island-solver-plan.md)
  owns continuous-collision TOI islands and same-frame advancement.
- This plan owns ordinary discrete contact islands, resting response,
  warm-start application, and island-wide sleep/wake after CCD has produced or
  handed off contact state.
- [`2026-06-21-query-and-mixed-swept-shape-hardening-plan.md`](2026-06-21-query-and-mixed-swept-shape-hardening-plan.md)
  owns public query and sweep exactness. This plan consumes the resulting
  contacts and improves response quality.

## Current Baseline

- 3D contacts store pair-local cached normal and two-axis tangent impulses by
  contact identity and the contact normal used by the previous solve.
- 3D response applies compatible cached impulses before the fresh solve,
  accumulates/clamps normal impulses, and solves friction over a deterministic
  two-axis tangent basis. Cached entries whose normals are no longer compatible
  are ignored and overwritten by the fresh solve.
- Pure 2D response applies pair-local cached impulses and supports two-contact
  manifolds. Workstream 3 follow-up moved pure 2D discrete response to the same
  post-integration service phase as 3D and added deterministic 2D island
  solving.
- Mixed response now refreshes and processes mixed contacts during
  `LateSimulate` after pure 2D and 3D body integration, rather than before body
  motion in `Simulate`.
- Cylinder collision/query support exists, but docs still call out edge-case
  hardening for the finite-cylinder model.
- Mesh narrow phase supports triangle-level contacts, but richer mesh contact
  clipping remains future work.
- Mixed response applies planar X/Z impulse and angular yaw impulse to 2D
  bodies while constraining vertical response to the 3D participant.

## Guiding Rules

- Contact ordering, island ordering, warm-start application, and sleep/wake
  propagation must be deterministic and testable.
- The solver should fail visibly through diagnostics or iteration caps rather
  than hiding instability behind silent clamps.
- Static and dynamic friction should remain physically explainable Coulomb-style
  behavior with explicit thresholds.
- 3D warm-starting must not inject energy when cached contacts are stale or
  reordered.
- Cylinder and mesh contact improvements must preserve finite-cylinder and
  triangle identity semantics.
- No recurring hot-path allocation after warmup.

## Workstream 1: Solver Invariant Inventory And Evidence

**Tasks**

- [x] Inventory current 3D, 2D, and mixed response paths, including cached
  impulse storage, tangent basis selection, contact IDs, pair ordering, and
  sleep/wake hooks.
- [x] Add or update tests that expose the current limitations: resting-stack
  drift, stale 3D cached impulses, same-island wake propagation, cylinder
  edge-touching, and mesh clipping ambiguity.
- [x] Add benchmark rows only where runtime cost is expected to change:
  dense resting contacts, cylinder-heavy scenes, mesh-contact scenes, and mixed
  response scenes.
- [x] Document the baseline before changing solver behavior.

**Progress 2026-06-21:** Workstream 1 added a current-baseline evidence slice
without changing runtime solver behavior. The new
`DiscreteResponseCurrentBaselineTests` file records that:

- stored 3D warm-start impulses did not affect the fresh single-pair solve
  before Workstream 2, confirming the original storage-only behavior. That
  stale baseline test was removed when 3D warm-start application landed.
- a short 3D cuboid stack under gravity remains awake and drifts, preserving
  evidence for the resting-friction/warm-start workstream.
- cylinder rim contact currently reduces to one near-zero-depth contact.
- mesh/cuboid face contact through a triangle mesh currently reduces to one
  triangle-level contact instead of a clipped contact surface.

Same-island wake propagation was not locked in as a current-baseline unit test:
the flat partition path can wake direct contacts, and a meaningful island test
should assert the desired connected-body behavior in Workstream 3 before the
island builder is introduced.

Current response inventory:

| Path | Current Behavior |
| --- | --- |
| 3D response | `CollisionPair` owns one four-contact `ContactManifold`, pair-local `ContactWarmStartCache`, and priority/speed/candidate-order collider ordering. Workstream 2 updated `CollisionResponse` to apply normal-compatible cached impulses before solving, accumulate/clamp normal impulses, solve friction over a deterministic two-axis tangent basis, then store normal/tangent impulse scalars and the solved normal by contact identity. Sleeping body wake happens before response when the opposite participant is awake. |
| Pure 2D response | `CollisionPair2D` owns a two-contact `ContactManifold2D` and `ContactWarmStartCache2D`. `CollisionResponse2D` reads cached normal/tangent impulses, applies them before the fresh solve, accumulates/clamps normal impulses, clamps tangent impulses to the current Coulomb bound, and stores the refreshed cache. Pair ordering uses priority, speed, then collider ID. Wake happens through pair handling after solid response. |
| Mixed response | `CollisionPairMixed` owns stable 3D/2D identity and a single `MixedContact` input. It has no pair-local warm-start cache or manifold reduction. `CollisionResponseMixed` applies constrained 3D/2D positional correction, normal impulse, and friction: planar X/Z impulse can move and yaw the 2D body, while vertical Y response is constrained out of the 2D participant. Wake happens before mixed response. |

Benchmark evidence rows were expanded before solver changes:

- `collision-response` now covers `SingleContact`, `FaceManifold`,
  `RestingFaceManifold`, `CylinderContact`, and `MeshContact` prepared 3D pairs
  at `16` and `64` pairs.
- `mixed-collision-response` covers prepared mixed sphere/circle response at
  `16` and `64` pairs.

The short in-process benchmark smoke executed successfully for the new rows,
but emitted BenchmarkDotNet minimum-iteration-time warnings. Treat that run as
setup validation, not canonical performance evidence.

## Workstream 2: 3D Warm-Start And Resting Friction

**Problem**

3D response stores pair-local cached impulses, but the cached values are not yet
applied as the first step of an iterative solve. Resting friction is currently
dynamic-friction oriented and does not fully stabilize resting stacks.

**Tasks**

- [x] Add tests where persistent 3D resting contacts avoid tangential jitter
  only when cached impulses are applied safely. Connected multi-body stack
  settling remains Workstream 3 because flat pair iteration should not be
  misrepresented as an island solver.
- [x] Add tests where stale cached impulses unwind instead of injecting energy
  after contact normals or IDs change.
- [x] Apply cached normal and tangent impulses before the fresh 3D solve,
  following the pure 2D accumulated-impulse rules where they fit 3D.
- [x] Introduce explicit static-friction behavior for near-resting tangential
  motion, with deterministic thresholds and material combination rules.
- [x] Re-run existing restitution, friction, and replay tests after each solver
  change.

**Progress 2026-06-21:** Workstream 2 updated 3D response to consume the
pair-local warm-start cache. Solver contacts now derive a deterministic 3D
tangent frame from the contact normal and carry cached normal, primary tangent,
and secondary tangent impulses. `ContactWarmStartImpulse` stores the previous
normal so 3D cache entries are reused only when the current normal remains
compatible (`63/64` fixed dot-product threshold); changed normals or changed
contact IDs fall back to a fresh solve.

Normal impulses now follow accumulated-impulse behavior: compatible cached
normal impulses are applied before the solve, positive fresh normal deltas are
distributed across manifold contacts, and negative stale-cache deltas can unwind
the full per-contact cached contribution without being reduced by face-manifold
contact share. Friction solves over the two tangent axes as a Coulomb disk. The
existing single `FrictionCoefficient` remains the material coefficient and is
combined by geometric mean; it acts as both the static sticking bound for
near-resting tangential motion and the sliding clamp when the requested tangent
impulse exceeds that bound.

Tests now cover persistent resting-load friction, stale cached impulse unwind,
normal-incompatible cache rejection, warm-start storage/reset, and the existing
restitution/friction/response invariants. The Workstream 1 storage-only
baseline was removed because it described behavior that no longer exists.

## Workstream 3: Deterministic Discrete Island Model

**Problem**

Flat pair iteration is predictable, but it does not model connected contact
islands, island-wide wake, or multi-iteration stabilization.

**Tasks**

- [x] Add tests for connected bodies where solving pairs independently produces
  order-sensitive drift or incomplete wake propagation.
- [x] Introduce an island builder keyed by stable body/collider IDs and pair
  keys, with explicit ordering for bodies, contacts, and constraints.
- [x] Add a bounded multi-iteration solve path for islands that need
  stabilization.
- [x] Connect island-wide wake/sleep to existing wake reasons without changing
  pure query or trigger-only behavior.
- [x] Keep single-pair scenes on a low-overhead path when island solving is not
  needed.

**Progress 2026-06-21:** Workstream 3 moved the 3D discrete response pass to
post-integration `LateSimulate`: dynamic bodies integrate first, service-owned
dynamic collider partitions refresh once, active partitions distribute
candidates, queued solid pairs are solved, active-pair maintenance runs, and
sleep state updates after response. Direct `StiffBody.LateSimulate()` remains
self-contained for callers outside the service path.

`GravitasCollisionService` now owns deterministic discrete island assembly.
Queued response pairs are ordered by stable collider ID pair. Movable island
nodes are keyed by `StiffBody.DynamicId`, union roots pick the lower stable
body key, and constraints are solved in root-key then pair-key order.
Single-pair scenes bypass island construction and keep the direct response path.
Multi-constraint islands run the bounded
`PhysicsSettings.DiscreteSolverIterations` count; cached warm-start impulses
and positional correction apply on the first iteration, with later iterations
refining velocity response without repeating the positional correction.

`PhysicsPartition.Distribute()` now emits all local dynamic-dynamic links and
all dynamic/static-style links when at least one dynamic collider in the
partition is awake. Sleeping bodies stay query-visible and can now participate
in a connected island when an awake body activates that partition. Island wake
uses the existing deterministic collision wake path so connected sleeping
bodies wake without changing trigger-only or query behavior. Fully sleeping
islands emitted only because another awake body activated the same partition are
not solved; they keep contact state visible without mutating sleeping body
positions.

**2D/Mixed parity follow-up 2026-06-21:** Pure 2D now follows the same
service-owned post-integration shape as 3D. `GravitasPhysics2DService.Simulate`
only clears frame counters; `LateSimulate` prepares CCD, integrates
`StiffBody2D` bodies without per-body collider refresh, refreshes dynamic 2D
colliders once, distributes 2D partition candidates, solves deterministic
discrete response islands, preserves connected resting pairs, and updates sleep
state after response. Existing resting pair-owned contacts adjacent to an active
2D response body are pulled into the temporary island graph so wake propagation
does not stop at an awake-partition boundary.

`GravitasMixedCollisionService` now mirrors the fixed-step phase boundary:
`Simulate` remains counter-only, while `LateSimulate` refreshes mixed partitions
after pure 2D and 3D integration and then processes mixed contacts. This keeps
mixed contacts from seeing stale pre-integration collider positions. Full mixed
island/warm-start quality remains Workstream 5 scope.

The full-step validation exposed a hot-path allocation RCA: the newly exercised
3D late collision pass still used comparer-based `Array.Sort` through
`SwiftList.Sort` on partition and island buffers. Those sorts were replaced
with allocation-free in-place sorting in the collision service and per-partition
collider-ID copies, restoring the CCD no-allocation assertions under the real
full fixed-step path.

## Workstream 4: Cylinder And Mesh Contact Geometry

**Problem**

Finite-cylinder support exists, but cylinder edge cases and mesh clipping still
need hardening so contacts are stable, physically plausible, and deterministic
across edge/cap/side transitions.

**Tasks**

- [ ] Add cylinder edge-case tests for sphere, capsule, cuboid, cylinder, mesh,
  and mixed embedded-2D slab contacts.
- [ ] Add mesh contact clipping tests where a triangle-level hit should produce
  a stable contact surface rather than a noisy point or ambiguous normal.
- [ ] Harden finite-cylinder cap, side, and rim contact selection without
  treating cylinders as capsules.
- [ ] Preserve mesh triangle candidate ordering and authored collider identity
  when multiple clipped contacts reduce to an external contact surface.
- [ ] Add benchmarks for cylinder-heavy and mesh-contact-heavy scenes before
  expanding clipping complexity.

## Workstream 5: Mixed-Dimension Response Quality

**Problem**

Mixed response correctly constrains vertical 2D motion, but richer mixed solver
behavior remains future hardening.

**Tasks**

- [ ] Add tests for mixed resting contacts where planar impulse, yaw torque,
  vertical constraint, and 3D participant motion interact.
- [ ] Decide whether mixed response should participate in discrete islands with
  pure 3D contacts, pure 2D contacts, or a dedicated mixed island type.
- [ ] Preserve `PhysicsRuntimeMode.Both` isolation from mixed response work.
- [ ] Add diagnostics for mixed response iteration count and cap hits if mixed
  islands become iterative.
- [ ] Document unsupported mixed solver behavior explicitly instead of leaving
  it as an implementation accident.

## Done Criteria

- 3D cached impulses are applied safely with normal-compatible reuse and stale
  cache unwind tests.
- Resting friction behavior has correctness tests, replay coverage, and clear
  thresholds.
- Discrete islands have stable ordering, bounded iteration, and wake/sleep
  tests.
- Cylinder and mesh contact edge cases have regression coverage and benchmark
  signal where hot-path cost changes.
- Mixed response limitations are resolved or documented as explicit policy.
