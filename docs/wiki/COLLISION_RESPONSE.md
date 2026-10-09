# Collision Response

Response converts contact manifolds into deterministic body motion, trigger and
contact events, warm-start caches, sleep/wake state, and cleanup. 3D, 2D, and
mixed response share ordering principles while keeping dimensional math
explicit.

## Quick Read

- Narrow phase writes pair-owned contact manifolds.
- Bodyless trigger volumes skip physical response and emit trigger events for
  valid trigger/body pairs.
- Non-trigger pairs are solved with enabled joints in deterministic body
  islands.
- 3D response uses 3D mass, inertia tensors, contact arms, and tangent frames.
- 2D response uses planar mass, scalar moment, and scalar yaw.
- Mixed response constrains 2D participants to planar X/Z motion and scalar yaw.
- Warm-start caches are pair-local and keyed by stable contact identity.
- Sleep updates after response.

## Contact Manifolds

The 3D narrow phase writes a `ContactManifold` owned by the `CollisionPair`.
`ManifoldContact` stores:

- stable contact identity derived from unordered canonical local anchor features
  and compound namespaces; world pose is excluded.
- point on collider A.
- point on collider B.
- penetration depth.
- normal oriented from collider A toward collider B.

3D manifolds retain independent geometric groups, with up to
`ContactManifold.MaxContactsPerGroup` (four) point samples per group. Within a
group, the reducer keeps the deepest samples, prefers conceptually clamped
depths on equal rounded depth, then breaks ties by lower contact identity.
Groups use full structural provenance: A/B compound namespaces, A/B surface
ordinals and region identity. Groups and their point identities have canonical
order; a group or triangle count is never a pressure weight. Primitive contacts
use the default group, and compound part contacts retain independent namespaces.
Mesh contact producers determine which regions they admit. Mesh/cone face
regions and real boundary features retain independent constraints. Their finite
samples cover the admitted geometry; coincident anchors can still carry distinct
normals at exact touch. Rotational CCD selects a closing impact row using the
sampled bodies' contact-point velocities, rather than assuming the deepest row
is the impact witness.

`Count`, the indexer and enumeration expose flattened group/point order;
`GroupCount`, `GetGroupStartIndex` and `GetGroupContactCount` support inspection.
The former pair-wide `MaxContactCount` constant is replaced by
`MaxContactsPerGroup`. The common single group is inline; overflow storage grows
on demand and remains with the pair after reset or pooling. First-ever capacity
growth can allocate; regeneration at a retained high-water mark does not.
Warm starts follow the same grouped ownership and are pruned after narrow-phase
regeneration, including when response is skipped.

2D narrow phase writes `ContactManifold2D` into `CollisionPair2D`. The 2D
manifold is fixed at two contacts because supported convex 2D face contacts need
at most the incident edge endpoints. Circle/circle and circle/convex contacts
normally produce one contact; convex/convex face overlap can produce two.

Contact depth is narrow-phase world distance. Solver slop belongs to response,
not contact data.

## Response Islands

After active partitions distribute candidates, the owning physics service sorts
queued response pairs by stable pair key and combines them with enabled joints:

| Domain | Contact rows                            | Joint rows | Island key                 |
| ------ | --------------------------------------- | ---------- | -------------------------- |
| 3D     | `CollisionPair` / `ContactManifold`     | `Joint3D`  | `SolidBody.DynamicId`      |
| 2D     | `CollisionPair2D` / `ContactManifold2D` | `Joint2D`  | `SolidBody2D.DynamicId`    |
| Mixed  | `CollisionPairMixed` / `MixedContact`   | none       | dimension-tagged body keys |

Fully sleeping islands are skipped. If an island contains an awake participant,
connected sleeping dynamic bodies are woken in deterministic order. Contact-only
single-row 3D scenes stay on a direct response path when no active joints exist.
A lone 3D pair with multiple points uses the configured iteration budget, as does
the public `CollisionResponse.CalculateImpulse` entry point. The 2D owner keeps
its own dimensional scheduling policy.

Multi-constraint islands run a bounded number of iterations from
`PhysicsSettings.DiscreteSolverIterations`. Cached warm-start impulses and
positional correction are applied on the first island iteration; later
iterations refine velocity response.

## 3D Response

Non-trigger 3D response:

1. captures incoming motion and COM-relative lever frames before any island
   warm start or joint solve.
2. derives linear and angular mobility independently from mass, inertia,
   role and frozen axes.
3. corrects position once per identical admitted normal direction, using its
   deepest depth above `PenetrationSlop` and the existing correction fraction.
   Other directions do not dilute correction; separate lever samples remain
   independent velocity constraints. This translation policy is not an exact
   multi-contact depenetration solve.
4. applies compatible grouped warm starts in canonical order.
5. solves and immediately applies each accumulated normal delta against current
   linear/angular point velocity, clamping accumulated normal impulse at zero.
6. fixes restitution to incoming motion before warm starts, so later iterations
   refine the same target instead of retracting bounce.
7. resolves per-contact materials and solves the two-axis Coulomb disk against
   that row's actual accumulated normal load.
8. stores each row's normal/tangent impulses independently. A failed
   representability preflight clears only that row's cache; other rows continue.

Admitted nonzero normals are authoritative. Collider-center direction is a
fallback only for a genuinely zero legacy normal. Contained sphere contacts with
finite cones/cylinders retain the solid's outward escape normal and sphere's
inward support. Exact kernels preserve frozen lever frames and incoming motion
through completed impulse/body-delta rounding.

Sequential multipoint response has a finite-iteration residual. More
`DiscreteSolverIterations` improve convergence at additional cost, including for
symmetric face contacts. Choose the budget using representative quality and
frame-time measurements.

3D response torque arms are measured from `SolidBody.WorldCenterOfMass`.
Collider centers remain collision-geometry references for narrow phase, culling,
and normal fallback; they are not implicit body COM.

Ordinary-domain 3D friction stays on a checked compact path. If point velocity,
effective mass, cache accumulation, disk clamping, or final velocity
materialization cannot be proven representable there, response falls through
once to the exact two-axis Coulomb-disk kernel. The exact path keeps both
tangent accumulators in exact rational/radical form through static retention,
dynamic radial projection, and cache removal, then rounds only the final body
deltas and representable cache projections. Friction application is atomic
across both bodies; a true final friction overflow does not undo the normal
response that was already applied earlier in the solver phase.

## 2D Response

2D response uses 2D-specific solver data:

- `ResponseBody2D`
- `SolverContact2D`
- `SolverContactBuffer2D`
- `SolidBody2D.EffectiveInverseMass`
- `SolidBody2D.EffectiveInverseMomentOfInertia`
- `SolidBody2D.WorldCenterOfMass`

Translation-frozen dynamic bodies contribute zero constrained inverse mass but
retain scalar angular inertia when yaw remains free. Yaw-frozen bodies retain
their available linear response. Static, kinematic, inactive, and
non-positive-mass states contribute neither applicable solver value while raw
mass and scalar moment stay inspectable. 2D contact response applies planar
linear velocity deltas and scalar angular velocity deltas from COM-relative
normal and tangent friction impulses.

Ordinary 2D friction uses the compact scalar tangent solve only while point
velocity, effective mass, friction limits, cache accumulation/removal, and final
body deltas remain representable without losing nonzero terms. Failed proofs
route once to the exact Coulomb-line owner, which keeps the cached tangent
impulse and solved delta exact until final round-to-even materialization.
Static-limit classification uses the signed interval directly, so
`Fixed64.MinValue` is not mistaken for a smaller saturated magnitude.

## Mixed Response

Mixed contacts are solved inside `GravitasMixedCollisionService` after both
dimension-local services have integrated bodies and refreshed their colliders.

Mixed response applies:

- X/Z penetration correction to movable 2D participants.
- planar normal impulse and friction impulse to 2D linear velocity.
- scalar yaw angular velocity deltas from planar COM-relative impulse arms.
- vertical Y correction/impulse only to the 3D participant.

The 2D body is treated as having infinite constrained mass along world Y.
`PhysicsRuntimeMode.Both` never creates mixed contacts.

Mixed friction maps 2D linear velocity and contact arms explicitly into world
X/Z before constructing point velocity. The compact path proves tangent
projection and normalization, every linear/angular effective-mass term,
friction-limit multiplication, and final impulse materialization. Any failed
proof routes once to the existing uncached exact Coulomb disk; no parallel mixed
wide solver or warm-start cache is introduced. Exact fallback preserves the same
planar constraint: world Y response remains exclusive to the 3D participant.

## Materials

`PhysicsMaterial` is collider surface data. `LSCollider`, `LSCollider2D`,
authored shape definitions, and compound parts can carry a material. Compound
parts without an explicit material inherit the owning compound collider material
when private part colliders are materialized.

Response rules:

- restitution is clamped to `[0, 1]`.
- default restitution combine policy is `Minimum`.
- materials can choose `Minimum`, `Maximum`, `Average`, `Multiply`, or
  `GeometricMean`.
- differing policies resolve deterministically in ascending precedence:
  `Average < Minimum < GeometricMean < Multiply < Maximum`.
- closing speeds at or below `PhysicsSettings.RestitutionVelocityThreshold` use
  zero restitution.
- static and dynamic friction are non-negative Coulomb coefficients.
- dynamic friction must not exceed static friction.
- values above one are allowed for intentional high-friction surfaces.

Friction impulses oppose tangential contact motion and are clamped by normal
impulse and resolved material coefficients. Static friction can stick within the
static bound; sliding clamps to the dynamic bound.

## Joint Metrics

3D joint rows write `JointSolveMetrics3D` to the owning `Joint3D`. 2D joint rows
write `JointSolveMetrics2D` to the owning `Joint2D`.

Metrics include prepared row count, pre-solve anchor error, limit error, motor
error, cached impulse magnitude, incremental impulse magnitude, motor impulse,
and clamped row count. These are deterministic diagnostic/stress signals, not
separate tuning knobs.

## Sleep And Wake

`SolidBody` and `SolidBody2D` own deterministic sleep state. A dynamic
non-kinematic body can sleep after linear and angular speed remain at or below
explicit thresholds for `SleepFrameThreshold` fixed frames.

Sleeping clears accumulated force, velocity, torque, acceleration, and pending
position-correction state, but does not remove the collider from GridForge
partitions.

Deterministic wake stimuli include:

- explicit host wake through `Wake()`.
- non-zero force.
- non-zero linear impulse.
- non-zero angular impulse or torque.
- collision with an awake body.
- kinematic host motion.
- host transform teleport.
- collider shape mutation.

Waking refreshes the collider's awake membership across current partitions.
Discrete response expands wake across connected dynamic contacts in
deterministic body-ID order.

## Notifications

Collision pairs are queued into the physics service active-pair queue the first
time they update. During late simulation, active pair maintenance:

- deactivates pairs that have not collided for the inactive-frame threshold.
- emits ongoing contact notifications when a pair is active and not culled.
- keeps active pairs queued for later maintenance.

Sleeping contact pairs are preserved while their manifold is known to be
colliding. This prevents resting sleeping contacts from aging out and emitting a
false contact exit simply because their partition skipped pair generation.

`LSCollider.NotifyContact(...)` and `LSCollider2D.NotifyContact(...)` emit:

- `OnTriggerEnter`, `OnTriggerStay`, and `OnTriggerExit` when exactly one
  collider is a trigger volume and the non-trigger collider is body-owned. Both
  colliders in the pair receive the trigger callback.
- `OnContactEnter`, `OnContact`, and `OnContactExit` for body contacts.

Mixed pairs follow the same rule with `OnMixedTriggerEnter`,
`OnMixedTriggerStay`, and `OnMixedTriggerExit`. Trigger pairs never emit contact
callbacks and do not participate in physical response.

## Diagnostics

When diagnostics are enabled, response emits events in deterministic processing
order:

1. `Contact`
2. `ResponseImpulse` for fresh normal-solve deltas
3. body velocity-delta events produced by warm-start, normal, and friction
   response

Diagnostics are observational only. They do not change pair ordering, contact
data, response behavior, or replay state.

## Source Map

| Area            | Source                                                                                                 |
| --------------- | ------------------------------------------------------------------------------------------------------ |
| 3D contact data | [`src/Gravitas/CollisionHandling/Contacts/3D`](../../src/Gravitas/CollisionHandling/Contacts/3D)       |
| 2D contact data | [`src/Gravitas/CollisionHandling/Contacts/2D`](../../src/Gravitas/CollisionHandling/Contacts/2D)       |
| Mixed contacts  | [`src/Gravitas/CollisionHandling/Contacts/Mixed`](../../src/Gravitas/CollisionHandling/Contacts/Mixed) |
| 3D response     | [`src/Gravitas/CollisionHandling/Response/3D`](../../src/Gravitas/CollisionHandling/Response/3D)       |
| 2D response     | [`src/Gravitas/CollisionHandling/Response/2D`](../../src/Gravitas/CollisionHandling/Response/2D)       |
| Mixed response  | [`src/Gravitas/CollisionHandling/Response/Mixed`](../../src/Gravitas/CollisionHandling/Response/Mixed) |
| Materials       | [`src/Gravitas/Materials`](../../src/Gravitas/Materials)                                               |
| 3D constraints  | [`src/Gravitas/Constraints/3D`](../../src/Gravitas/Constraints/3D)                                     |
| 2D constraints  | [`src/Gravitas/Constraints/2D`](../../src/Gravitas/Constraints/2D)                                     |
