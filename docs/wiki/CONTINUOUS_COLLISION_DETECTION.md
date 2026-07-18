# Continuous Collision Detection

Continuous collision detection (CCD) prevents fast motion from skipping relevant
contacts. Gravitas keeps CCD deterministic by using fixed-point sweep reducers,
stable candidate ordering, bounded time-of-impact work, and service-owned
handoff queues.

## Quick Read

- CCD resolves from body, hierarchy, then context defaults.
- Static/kinematic targets use static-style sweeps.
- Dynamic targets use relative-motion candidate indexing.
- Kinematic bodies can act as active swept sources from frame-start pose to host
  target pose.
- Mixed CCD runs only in `PhysicsRuntimeMode.Mixed`.
- Service-level handoff queues handle dense same-frame contact chains.
- Public query APIs remain query APIs; CCD uses internal target filters where
  needed.

## Public Control Surface

| Concern                 | API                                                                                                                                                       |
| ----------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Context default         | `PhysicsSettings.DefaultContinuousCollisionMode`                                                                                                          |
| 3D body override        | `SolidBody.ContinuousCollisionMode`                                                                                                                       |
| 2D body override        | `SolidBody2D.ContinuousCollisionMode`                                                                                                                     |
| Body-owned TOI counters | `LastContinuousCollisionToiIterationCount`, `LastContinuousCollisionToiIterationLimitReached`                                                             |
| 3D service counters     | `GravitasPhysicsService.LastContinuousCollisionIslandCount`, `LastContinuousCollisionIslandIterationCount`, `LastContinuousCollisionIslandLimitReached`   |
| 2D service counters     | `GravitasPhysics2DService.LastContinuousCollisionIslandCount`, `LastContinuousCollisionIslandIterationCount`, `LastContinuousCollisionIslandLimitReached` |

Both body types expose body-owned TOI counters, and both physics services expose
service-level island counters for the last late step.

`Inherit`, `Discrete`, `Continuous`, and `Auto` are the only valid
`ContinuousCollisionMode` values. The context default deliberately accepts
`Inherit`; if no body or hierarchy override supplies a concrete mode, that
context value resolves to `Discrete`. Public settings/body assignment and
settings or replay population reject undefined byte-cast values with
`ArgumentOutOfRangeException` before publishing them. Invalid authored or
serialized state therefore cannot silently change tunneling policy.

## CCD Paths

### `SolidBody`

| Path                    | Target set                                                           | Reducer policy                                                                                                                            |
| ----------------------- | -------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| Static/kinematic 3D     | bodyless, position-frozen, kinematic 3D colliders                    | Source collider proxy sphere, then shape-exact validation where supported.                                                                |
| Dynamic 3D              | movable dynamic 3D bodies                                            | Relative proxy spheres, exact validation for supported source families, conservative proxy behavior where no exact source reducer exists. |
| Mixed static 2D         | bodyless, position-frozen, kinematic 2D slabs                        | Same reducer policy as public `QueryMixed.SweepSphereAgainst2D`.                                                                          |
| Mixed dynamic 2D        | movable dynamic 2D bodies                                            | Conservative mixed proxy candidate, then bounded handoff.                                                                                 |
| Kinematic active source | static-style blockers and dynamic 3D/2D targets before first blocker | Frame-start pose to host target pose, using the underlying dimension-local or mixed reducer.                                              |

### `SolidBody2D`

| Path                    | Target set                                                           | Reducer policy                                                                                                              |
| ----------------------- | -------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------- |
| Static/kinematic 2D     | bodyless, position-frozen, kinematic 2D colliders                    | Source circle sweep, refined by mover shape where needed.                                                                   |
| Dynamic 2D              | movable dynamic 2D bodies                                            | Relative proxy circles, then exact mover-shape validation for circle, capsule, AABB, convex polygon, and compound families. |
| Mixed static 3D         | bodyless, position-frozen, kinematic 3D colliders                    | Same reducer policy as public `QueryMixed.SweepCircleAgainst3D`.                                                            |
| Mixed dynamic 3D        | movable dynamic 3D bodies                                            | Conservative mixed proxy candidate, then bounded handoff.                                                                   |
| Kinematic active source | static-style blockers and dynamic 2D/3D targets before first blocker | Frame-start pose to host target pose, using the underlying dimension-local or mixed reducer.                                |

## Static And Kinematic Targets

Static-style CCD targets include bodyless colliders, position-frozen bodies, and
kinematic bodies. Public sweep queries can report movable dynamic, kinematic,
position-frozen, and bodyless targets according to normal query filters; body
CCD uses internal static-style collectors so movable dynamic targets are handled
by the dynamic relative-motion path. Final CCD target admission uses the owning
collision service's physical-pair gate, including collider lifecycle, authored
filters, hierarchy rules, and linked-joint collision suppression.

For kinematic active-source CCD, hosts write deterministic target transforms
before `context.LateSimulate()`. Gravitas captures the frame-start pose, reads
the host transform as the requested target pose, sweeps between those poses, and
clips the first static-style blocker. Dynamic targets crossed before the first
blocker are woken and position-corrected through service-owned handoff queues.

## Dynamic Candidate Ordering

Dynamic-vs-dynamic CCD uses candidate indices and stable ordering:

- time of impact.
- closing speed where applicable.
- collider ID or dimension-tagged mixed key.
- bounded iteration counts.

The ordering is deterministic across repeated runs. Service-level queues own
same-frame handoff processing so dense contact chains do not depend on traversal
side effects. Queue admission deduplicates a body only while that body owns an
unread entry. Dequeue releases that ownership before consumption, allowing a
later same-frame relay to append the body again under the same deterministic
iteration budget. Requeued work that exceeds the budget is explicitly
discarded rather than left as stale continuation state. A later terminal
handoff update (no remaining time or no resulting motion) likewise cancels any
older pending continuation for that body; latest-state-wins includes the
absence of further work.

## Mixed CCD

Mixed CCD uses explicit mixed query reducers only in `PhysicsRuntimeMode.Mixed`.
`PhysicsRuntimeMode.Both` advances 2D and 3D services side by side without
cross-dimensional CCD.

3D swept-sphere mixed CCD routes through `QueryMixed.SweepSphereAgainst2D`.
Accepted circle, capsule, AABB, convex polygon, and supported compound slab hits
use exact finite-slab reducers.

2D swept-circle mixed CCD routes through `QueryMixed.SweepCircleAgainst3D`.
Supported primitive, mesh, and compound target families use exact reducers for
the public mixed query contract. Rotated finite cones use the same
support-mapped convex advancement kernel as 3D swept source queries with a
query-owned circle-slab source.

`PhysicsMixedHit.ReducerKind` labels exact hits separately from conservative
proxy candidates used by dynamic CCD paths.

## Rotational CCD

Same-dimensional rotational CCD is bounded and deterministic. It traverses
normalized-time intervals earliest-first for each candidate and samples exact
narrow phase at each midpoint. Shape-specific closest-feature separation, with
an AABB fallback, certifies an interval only when its gap exceeds an
outward-rounded bound on translation and pivot-centered angular travel. The
bound also scales fixed-point pose uncertainty by pivot radius. An unresolved
interval is subdivided until a fixed depth or per-candidate work budget;
exhaustion clamps at the interval's lower time. It never borrows a normal from
another candidate; when the same target already has an exact later contact
witness, that upper-bound normal may remove closing linear velocity at the
earlier conservative clamp. Without a same-target witness, only rotational
motion is stopped. Candidate results are then ordered by time and collider ID.
This permits an early conservative stop but cannot silently advance through an
unresolved contact window.

Rotational broad-phase radii are measured from the body's actual rotation pivot,
not merely from the collider center, so local offsets and remote compound parts
remain inside the candidate volume. Unsupported collision pairs are skipped
explicitly. If the required pivot radius exceeds the scalar domain, candidate
admission scans the bounded context registry instead of issuing an effectively
unbounded GridForge query. Dynamic-target handoff and mixed-dimensional
rotational sweeps are a separate active hardening contract; the current
rotational interval path covers same-dimensional static and kinematic targets.

## Diagnostics And Replay

Body-owned bounded TOI counters are deterministic frame-local state for tuning,
tests, and host diagnostics. Service counters describe CCD island/handoff
behavior without adding event-buffer traffic to the hot path.

Active cross-frame CCD handoff state is included in the authoritative replay
hash because it can affect the next fixed step. Rebuildable per-frame CCD
snapshots are excluded from ordinary authoritative hashes and are available only
through solver-cache hash mode when useful for drift RCA.

## Rules That Matter

- Keep CCD target filters separate from public query filters.
- Keep exact reducers and conservative proxy behavior explicit.
- Bound TOI iterations and handoff processing.
- Preserve stable candidate ordering.
- Use mixed CCD only through the mixed runtime path.
- Add replay tests for any CCD state that affects continuation.
- Add benchmarks for dense dynamic, mixed, rotational, or kinematic-source CCD
  changes.

## Source Map

| Area                 | Source                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| -------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 3D body CCD          | [`src/Gravitas/Core/3D/SolidBody.ContinuousCollision.cs`](../../src/Gravitas/Core/3D/SolidBody.ContinuousCollision.cs), [`src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Dynamic.cs`](../../src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Dynamic.cs), [`src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Kinematic.cs`](../../src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Kinematic.cs)                                 |
| 2D body CCD          | [`src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.cs`](../../src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.cs), [`src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.Dynamic.cs`](../../src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.Dynamic.cs), [`src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.Kinematic.cs`](../../src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.Kinematic.cs)                     |
| 3D service CCD       | [`src/Gravitas/Core/3D/GravitasPhysicsService.ContinuousCollision.cs`](../../src/Gravitas/Core/3D/GravitasPhysicsService.ContinuousCollision.cs)                                                                                                                                                                                                                                                                                           |
| 2D service CCD       | [`src/Gravitas/Core/2D/GravitasPhysics2DService.ContinuousCollision.cs`](../../src/Gravitas/Core/2D/GravitasPhysics2DService.ContinuousCollision.cs)                                                                                                                                                                                                                                                                                       |
| CCD common helpers   | [`src/Gravitas/CollisionHandling/Continuous`](../../src/Gravitas/CollisionHandling/Continuous)                                                                                                                                                                                                                                                                                                                                             |
| Mixed query reducers | [`src/Gravitas/Queries/Mixed`](../../src/Gravitas/Queries/Mixed)                                                                                                                                                                                                                                                                                                                                                                           |
| CCD tests            | [`tests/Gravitas.Tests/CollisionHandling/ContinuousCollisionDetectionTests.cs`](../../tests/Gravitas.Tests/CollisionHandling/ContinuousCollisionDetectionTests.cs), [`tests/Gravitas.Tests/Physics2D/ContinuousCollision2DTests.cs`](../../tests/Gravitas.Tests/Physics2D/ContinuousCollision2DTests.cs), [`tests/Gravitas.Tests/MixedDimensions/MixedQueryCcdTests.cs`](../../tests/Gravitas.Tests/MixedDimensions/MixedQueryCcdTests.cs) |
