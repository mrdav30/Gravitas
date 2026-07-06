# Collision Response

Response converts contact manifolds into deterministic body motion, trigger and
contact events, warm-start caches, sleep/wake state, and cleanup. 3D, 2D,
and mixed response share ordering principles while keeping dimensional math
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

- stable contact identity derived from the unordered pair of world-space contact
  points.
- point on collider A.
- point on collider B.
- penetration depth.
- normal oriented from collider A toward collider B.

3D manifolds store up to four contacts. When more candidates are offered, the
manifold keeps the deepest four and breaks depth ties by lower stable contact
identity. Exposed contact order is stable ascending contact identity.

2D narrow phase writes `ContactManifold2D` into `CollisionPair2D`. The 2D
manifold is fixed at two contacts because supported convex 2D face contacts need
at most the incident edge endpoints. Circle/circle and circle/convex contacts
normally produce one contact; convex/convex face overlap can produce two.

Contact depth is narrow-phase world distance. Solver slop belongs to response,
not contact data.

## Response Islands

After active partitions distribute candidates, the owning physics service sorts
queued response pairs by stable pair key and combines them with enabled joints:

| Domain | Contact rows | Joint rows | Island key |
| --- | --- | --- | --- |
| 3D | `CollisionPair` / `ContactManifold` | `Joint3D` | `SolidBody.DynamicId` |
| 2D | `CollisionPair2D` / `ContactManifold2D` | `Joint2D` | `SolidBody2D.DynamicId` |
| Mixed | `CollisionPairMixed` / `MixedContact` | none | dimension-tagged body keys |

Fully sleeping islands are skipped. If an island contains an awake participant,
connected sleeping dynamic bodies are woken in deterministic order. Contact-only
single-pair scenes stay on a low-overhead direct response path when no active
joints exist.

Multi-constraint islands run a bounded number of iterations from
`PhysicsSettings.DiscreteSolverIterations`. Cached warm-start impulses and
positional correction are applied on the first island iteration; later
iterations refine velocity response.

## 3D Response

Non-trigger 3D response:

1. builds solver contacts from the pair manifold, collider bodies, contact
   points, relative COM arms, penetration depth, and pair-oriented normal.
2. treats fully position-frozen and kinematic bodies as infinite mass.
3. applies positional correction only for depth above
   `CollisionResponse.PenetrationSlop`.
4. shares correction across active manifold contacts.
5. computes normal contact velocity from linear velocity plus angular velocity
   at each relative contact arm.
6. applies compatible cached normal/tangent impulses before the fresh solve.
7. solves normal impulse deltas and clamps accumulated normal impulse at zero.
8. resolves collider surface materials and combine policies.
9. solves friction over a deterministic tangent frame derived from the contact
   normal.
10. stores solved normal/tangent impulses and contact normal in a fixed-size
    warm-start cache.

3D response torque arms are measured from `SolidBody.WorldCenterOfMass`.
Collider centers remain collision-geometry references for narrow phase, culling,
and normal fallback; they are not implicit body COM.

## 2D Response

2D response uses 2D-specific solver data:

- `ResponseBody2D`
- `SolverContact2D`
- `SolverContactBuffer2D`
- `SolidBody2D.EffectiveInverseMass`
- `SolidBody2D.EffectiveInverseMomentOfInertia`
- `SolidBody2D.WorldCenterOfMass`

Position-frozen, kinematic, inactive, non-positive-mass, and yaw-frozen states
remain infinite mass/inertia to the solver while raw mass and scalar moment stay
inspectable. 2D contact response applies planar linear velocity deltas and
scalar angular velocity deltas from COM-relative normal and tangent friction
impulses.

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
- closing speeds at or below
  `PhysicsSettings.RestitutionVelocityThreshold` use zero restitution.
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

Sleeping clears accumulated force, impulse, velocity, torque, acceleration, and
pending position-correction state, but does not remove the collider from
GridForge partitions.

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

| Area | Source |
| --- | --- |
| 3D contact data | [`src/Gravitas/CollisionHandling/Contacts/3D`](../../src/Gravitas/CollisionHandling/Contacts/3D) |
| 2D contact data | [`src/Gravitas/CollisionHandling/Contacts/2D`](../../src/Gravitas/CollisionHandling/Contacts/2D) |
| Mixed contacts | [`src/Gravitas/CollisionHandling/Contacts/Mixed`](../../src/Gravitas/CollisionHandling/Contacts/Mixed) |
| 3D response | [`src/Gravitas/CollisionHandling/Response/3D`](../../src/Gravitas/CollisionHandling/Response/3D) |
| 2D response | [`src/Gravitas/CollisionHandling/Response/2D`](../../src/Gravitas/CollisionHandling/Response/2D) |
| Mixed response | [`src/Gravitas/CollisionHandling/Response/Mixed`](../../src/Gravitas/CollisionHandling/Response/Mixed) |
| Materials | [`src/Gravitas/Materials`](../../src/Gravitas/Materials) |
| 3D constraints | [`src/Gravitas/Constraints/3D`](../../src/Gravitas/Constraints/3D) |
| 2D constraints | [`src/Gravitas/Constraints/2D`](../../src/Gravitas/Constraints/2D) |
