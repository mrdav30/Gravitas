# Continuous Collision Detection

Continuous collision detection (CCD) prevents fast motion from skipping relevant
contacts. Gravitas keeps CCD deterministic by using fixed-point sweep reducers,
stable candidate ordering, bounded time-of-impact work, and service-owned
handoff queues.

## Quick Read

- CCD resolves from body, hierarchy, then context defaults.
- Bodyless and position-frozen targets use static-style sweeps.
- Moving dynamic and kinematic targets use frame-prepared candidate indexing.
- Kinematic bodies can act as active swept sources from frame-start pose to host
  target pose.
- Translation and rotation compete in one normalized-time arbiter whenever
  either participant has rotational motion.
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
| Stationary 3D           | bodyless, position-frozen, and stationary kinematic 3D colliders     | Source collider proxy sphere, then shape-exact validation where supported.                                                                |
| Moving 3D               | movable dynamic and moving kinematic 3D bodies                       | Prepared pair trajectories, exact validation for supported source families, conservative proxy behavior where no exact source reducer exists. |
| Mixed stationary 2D     | bodyless, position-frozen, and stationary kinematic 2D slabs         | Same reducer policy as public `QueryMixed.SweepSphereAgainst2D`.                                                                          |
| Mixed moving 2D         | movable dynamic and moving kinematic 2D bodies                       | Prepared pair trajectories, conservative mixed proxy candidate, then bounded handoff.                                                     |
| Kinematic active source | static-style blockers and dynamic 3D/2D targets before first blocker | Frame-start pose to host target pose, using the underlying dimension-local or mixed reducer.                                              |

### `SolidBody2D`

| Path                    | Target set                                                           | Reducer policy                                                                                                              |
| ----------------------- | -------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------- |
| Stationary 2D           | bodyless, position-frozen, and stationary kinematic 2D colliders     | Source circle sweep, refined by mover shape where needed.                                                                   |
| Moving 2D               | movable dynamic and moving kinematic 2D bodies                       | Prepared pair trajectories, then exact mover-shape validation for circle, capsule, AABB, convex polygon, and compound families. |
| Mixed stationary 3D     | bodyless, position-frozen, and stationary kinematic 3D colliders     | Same reducer policy as public `QueryMixed.SweepCircleAgainst3D`.                                                            |
| Mixed moving 3D         | movable dynamic and moving kinematic 3D bodies                       | Prepared pair trajectories, conservative mixed proxy candidate, then bounded handoff.                                      |
| Kinematic active source | static-style blockers and dynamic 2D/3D targets before first blocker | Frame-start pose to host target pose, using the underlying dimension-local or mixed reducer.                                |

## Stationary And Kinematic Targets

Static-style CCD targets include bodyless colliders and position-frozen bodies.
Moving kinematic targets are captured in the same frame-prepared candidate
index and piecewise trajectory model as moving dynamic targets, so their sampled
pose does not depend on body registration or service order. Public sweep queries
can report dynamic, kinematic, position-frozen, and bodyless targets according
to normal query filters. Final CCD target admission uses the owning collision
service's physical-pair gate, including collider lifecycle, authored filters,
hierarchy rules, and linked-joint collision suppression.

For kinematic active-source CCD, hosts write deterministic target transforms
before `context.LateSimulate()`. Gravitas captures the frame-start pose, reads
the host transform as the requested target pose, sweeps between those poses, and
clips the first static-style blocker. Dynamic targets crossed before the first
blocker are woken and position-corrected through service-owned handoff queues.
Discrete contact response samples the same prepared kinematic end velocity, so
authored same-frame linear and angular motion contributes consistently even when
the kinematic body was processed earlier in service order.

## Dynamic Candidate Ordering

Moving-pair CCD uses immutable frame-start candidate indices plus bounded dirty
overlays for bodies whose same-frame handoff changes their remaining swept
bounds. A dirty body shadows its immutable entry; stale prepared bounds are not
unioned back into admission. Each body exposes a canonical piecewise position,
rotation, and velocity trajectory. Later sources therefore sample the exact
pre-impact and post-impact history instead of whichever pose happened to be
published most recently. Translational moving-pair reducers traverse only the
target segments that overlap the source's remaining interval, clip the source
sweep to each segment, and map segment-local hits back to the source's global
time of impact. Handoff boundaries are right-continuous: when a target reverses
exactly at a shared boundary, the successor segment owns that instant. Because
canonical segments are chronological and non-overlapping, the first admitted
non-boundary hit is the global earliest hit. This contract is shared by dynamic
and kinematic 2D, 3D, and mixed CCD.

Candidate results use stable ordering:

- time of impact.
- target dimension, with 2D before 3D for an exact-time tie.
- stable collider ID or dimension-tagged mixed key.
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
Capsule boundary intervals use full-domain finite-segment arithmetic, but the
current horizontal-rim decomposition conservatively overexpands the true
rounded boundary. Circle slabs likewise use full-domain arithmetic for a
conservative sharp-rim expanded-cylinder proxy. Both current `Exact` labels are
tracked for correction. AABB, convex polygon, and supported compound slab hits
use their exact finite-slab reducers.

2D swept-circle mixed CCD routes through `QueryMixed.SweepCircleAgainst3D`.
Supported primitive, mesh, and compound target families use exact reducers for
the public mixed query contract. Rotated finite cones use the same
support-mapped convex advancement kernel as 3D swept source queries with a
query-owned circle-slab source.

`PhysicsMixedHit.ReducerKind` labels exact hits separately from conservative
proxy candidates used by dynamic CCD paths.

## Rotational CCD

Rotational CCD is bounded and deterministic across same-dimensional and mixed
pairs. When either participant has rotational motion, source translation,
source rotation, target translation, and target rotation compete in one
normalized-time arbiter. It traverses intervals earliest-first for each
candidate and samples both prepared poses at each midpoint. Shape-specific
closest-feature separation, with a conservative AABB fallback where no tighter
proof exists, certifies an interval only when its gap exceeds an
outward-rounded bound on both participants' linear and pivot-centered angular
travel. The bound also scales fixed-point pose uncertainty by pivot radius.

An unresolved interval is subdivided until a fixed depth or per-candidate work
budget. A witnessed contact can apply the contact-point response and bounded
handoffs atomically. If the search cannot prove separation or witness contact,
it clamps at the unresolved interval's lower time without inventing an impulse.
Only the immediate prior pair is excluded from continuation, so a deterministic
`A -> B -> A` same-frame chain remains admissible. Candidate results are ordered
by normalized time, target dimension, and stable collider identity.

Trajectory mutation and dirty-overlay admission share a deterministic frame
budget. If either participant cannot reserve all state required for an atomic
pair update, Gravitas conservatively clamps before mutating either body and
reports the CCD iteration limit. Linear-only handoffs preserve unrelated
angular acceleration, and angular-only handoffs preserve unrelated linear
acceleration.

Rotational broad-phase radii are measured from each body's actual rotation pivot,
not merely from the collider center, so local offsets and remote compound parts
remain inside the candidate volume. Unsupported collision pairs are skipped
explicitly. If the required pivot radius exceeds the scalar domain, candidate
admission scans the bounded context registry instead of issuing an effectively
unbounded GridForge query. Dynamic and kinematic moving targets use their
prepared piecewise trajectories; mixed candidates are admitted only in
`PhysicsRuntimeMode.Mixed`, never in `Both`.

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
| 3D body CCD          | [`src/Gravitas/Core/3D/SolidBody.ContinuousCollision.cs`](../../src/Gravitas/Core/3D/SolidBody.ContinuousCollision.cs), [`src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Dynamic.cs`](../../src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Dynamic.cs), [`src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Kinematic.cs`](../../src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Kinematic.cs), and the focused `SolidBody.ContinuousCollision.Rotational*.cs` partials.             |
| 2D body CCD          | [`src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.cs`](../../src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.cs), [`src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.Dynamic.cs`](../../src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.Dynamic.cs), [`src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.Kinematic.cs`](../../src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.Kinematic.cs), and the focused `SolidBody2D.ContinuousCollision.Rotational*.cs` partials. |
| 3D service CCD       | [`src/Gravitas/Core/3D/GravitasPhysicsService.ContinuousCollision.cs`](../../src/Gravitas/Core/3D/GravitasPhysicsService.ContinuousCollision.cs)                                                                                                                                                                                                                                                                                           |
| 2D service CCD       | [`src/Gravitas/Core/2D/GravitasPhysics2DService.ContinuousCollision.cs`](../../src/Gravitas/Core/2D/GravitasPhysics2DService.ContinuousCollision.cs)                                                                                                                                                                                                                                                                                       |
| CCD common helpers   | [`src/Gravitas/CollisionHandling/Continuous`](../../src/Gravitas/CollisionHandling/Continuous)                                                                                                                                                                                                                                                                                                                                             |
| Mixed query reducers | [`src/Gravitas/Queries/Mixed`](../../src/Gravitas/Queries/Mixed)                                                                                                                                                                                                                                                                                                                                                                           |
| CCD tests            | [`tests/Gravitas.Tests/CollisionHandling/ContinuousCollisionDetectionTests.cs`](../../tests/Gravitas.Tests/CollisionHandling/ContinuousCollisionDetectionTests.cs), [`tests/Gravitas.Tests/Physics2D/ContinuousCollision2DTests.cs`](../../tests/Gravitas.Tests/Physics2D/ContinuousCollision2DTests.cs), [`tests/Gravitas.Tests/MixedDimensions/MixedQueryCcdTests.cs`](../../tests/Gravitas.Tests/MixedDimensions/MixedQueryCcdTests.cs) |
