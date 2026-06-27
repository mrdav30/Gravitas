# Collision Pipeline

Gravitas collision work is split into GridForge-backed broad phase,
context-local pair management, shape-pair narrow phase, deterministic contact
manifolds, manifold response, and late contact notification.

## 2D And 3D Boundary

The 3D collision handling path routes `SolidBody` and
`LSCollider` route through `GravitasPhysicsService`; `SolidBody2D` and
`LSCollider2D` route through `GravitasPhysics2DService`. The active path is
selected by `PhysicsSettings.RuntimeMode`, so pure 2D scenes do not advance 3D
pair distribution or visualization work.

Pure 2D uses X/Z host projection: world `Vector3d.x` maps to `Vector2d.x` and
world `Vector3d.z` maps to `Vector2d.y`. World `Vector3d.y` is height or future
mixed-dimension embedding metadata, not a pure 2D collision axis.

## Pure 2D Collision Path

`GravitasPhysics2DService` owns the pure 2D path for `SolidBody2D` and
`LSCollider2D`. It keeps 2D collider IDs, 2D body registration, reusable pair
state, visualization publishing, and caller-buffered overlap/raycast query
output local to one
`GravitasWorldContext`.

The 2D broad phase is GridForge-backed:

1. rebuild a 2D collider's `FixedBoundArea` when body motion, kinematic host
   motion, explicit bodyless collider refresh, or shape input edits change it.
2. project the collider's X/Z bounds into private GridForge storage on the Y=0
   plane.
3. scan covered GridForge spatial cells and voxels.
4. attach or reuse a `PhysicsPartition2D` payload on each covered voxel.
5. store collider IDs in static, dynamic, and awake-dynamic sparse sets.
6. distribute candidate pairs from active partitions in deterministic
   voxel/order and collider-ID order.
7. suppress duplicate pair work when broad colliders share several voxels by
   routing each pair through its deterministic first shared partition before
   the frame duplicate-pair set.
8. run same-agent, explicit hierarchy, layer, and bounds filtering before exact
   2D narrow-phase dispatch.

The Y=0 storage plane is not physical thickness and does not claim mixed
2D/3D interaction. It is a deterministic broad-phase identity that lets pure
2D and 3D use the same host-owned `GridWorld` model. `PhysicsRuntimeMode.Both`
keeps those paths side by side without cross-dimensional contacts; only
`PhysicsRuntimeMode.Mixed` enables the mixed lifecycle path. The mixed broad
phase uses `PhysicsMixedPartition` payloads attached to GridForge voxels and emits
stable 3D/2D candidate keys after awake-dynamic, layer, same-agent, explicit
hierarchy, duplicate, and bounds filtering. The mixed embedding state on
`LSCollider2D` is a finite 3D `FixedBoundBox` built from pure 2D X/Z bounds plus a
positive Y half-thickness centered on the host transform's Y position.
`CollisionDetectionMixed` supports 3D sphere, cuboid, capsule, finite
cylinder, compound, and mesh contacts against embedded 2D circle, AABB, convex
polygon, and compound slabs. Compound mixed contacts scan owned parts in stable
order and return one external contact surface on either side. Mesh mixed
contacts gather local-BVH triangle candidates and test triangles against the
embedded 2D slab volume.
`CollisionPairMixed` owns stable 3D/2D pair identity, resting-pair retention,
wake propagation, mixed contact enter/stay/exit events, trigger-only mixed
trigger events, and pooled pair reuse. `CollisionResponseMixed` applies the
constrained mixed response: planar X/Z correction and impulses can move a
2D body, planar normal and friction impulse components can spin it around its
scalar yaw axis from the 2D COM-relative contact arm, and vertical Y correction
and impulses treat the 2D body as having infinite constrained mass.

Mixed contacts are processed during `LateSimulate` after both pure 2D and 3D
services have integrated bodies and refreshed their own collider partitions, so
mixed response observes post-integration collider positions. Active mixed
partitions emit deterministic cross-dimension candidate links for dynamic
members once any local dynamic participant is awake; brand-new sleeping/sleeping
links still exit before mixed narrow phase, while retained resting links can
bridge a connected mixed island. Non-trigger mixed pairs are collected into a
dedicated mixed response graph keyed by stable dimension-tagged body IDs. Mixed
islands are solved inside `GravitasMixedCollisionService` in deterministic
pair-key order for `PhysicsSettings.DiscreteSolverIterations`; they do not
merge into the pure 3D or pure 2D discrete island solvers.
`PhysicsRuntimeMode.Both` remains isolated and never creates mixed contacts.

Mixed diagnostics, explicit mixed queries, and mixed CCD hooks are implemented.
Mixed query hits expose `PhysicsMixedHit.ReducerKind` so hosts can distinguish
exact finite-slab reducers from safe conservative fallbacks. 2D swept-circle
mixed CCD routes through the same mixed query reducers as public
`SweepCircleAgainst3D`: sphere, cuboid, capsule, and finite-cylinder targets
use finite-slab reducers. Mesh targets clip candidate triangles to the finite
slab before X/Z projection, and compound targets reduce exact supported parts in
authored order. 3D swept-sphere mixed CCD routes through the same mixed query
reducers as public `SweepSphereAgainst2D`: circle slabs, AABB slabs, convex
polygon slabs, and supported compound 2D slabs are exact.
When diagnostics are enabled, mixed queries also emit `QuerySummary` events with
eligible top-level exact attempt, accepted hit, fallback hit, and rejected
fallback counts.

`CollisionDetection2D` supports:

- circle/circle.
- circle/axis-aligned box.
- axis-aligned box/axis-aligned box.
- circle/convex polygon.
- axis-aligned box/convex polygon.
- convex polygon/convex polygon.
- compound/primitive or compound/compound, resolved by scanning owned parts in
  stable declaration order and returning the owner collider identity.

Circles use center/radius tests and support points. Boxes and polygons use 2D
separating-axis tests over deterministic vertex order. `LSPolygonCollider2D`
rejects concave and collinear input up front; concave 2D decomposition is not
claimed yet. `CollisionPair2D` resolves collider priority up front and
`CollisionDetection2D` dispatches through `CollisionType2D`, so adding a new
2D shape pair should extend the settings/type table instead of growing public
type-check conditionals.

The separating-axis invariant is the broad rule behind every convex SAT path:
if a candidate axis projects two convex shapes into non-overlapping intervals,
the shapes cannot be colliding. A collision is possible only when every tested
required axis overlaps. Candidate axes must be generated and tested in stable
order, and contact normals must be oriented by the pair convention rather than
by ad hoc shape-pair sign fixes.

`CollisionPair2D` owns pure 2D pair lifecycle: stable collider priority,
pair-owned `ContactManifold2D` state, pair-local warm-start cache, wake
propagation from awake movable bodies, trigger enter/exit events, and contact
enter/stay/exit events. Solid response is delegated to `CollisionResponse2D`,
which builds `ResponseBody2D`, `SolverContact2D`, and `SolverContactBuffer2D`
state from the current manifold, shares positional correction across active
contacts, applies cached normal/tangent impulses when contact IDs persist, then
solves one stable normal pass and one stable tangent-friction pass. It reads
`SolidBody2D.EffectiveInverseMass`,
`SolidBody2D.EffectiveInverseMomentOfInertia`, and
`SolidBody2D.WorldCenterOfMass` so position-frozen, kinematic, inactive,
non-positive-mass, and angular-disabled bodies remain infinite mass/inertia to
the solver while raw mass and scalar moment values stay inspectable. If a solid
pair has no awake movable participant, the existing pair is kept alive as
resting state without applying response or waking a sleeping body.

## Broad Phase: Voxel Partitions

When a collider initializes, moves, rotates, changes scale, or changes local
shape inputs, `LSCollider` rebuilds its runtime shape data and asks
`GravitasCollisionService` to repartition it. Shape inputs are tracked by an
internal snapshot so several local edits made before a simulation call collapse
into one bounds/shape rebuild.

`GravitasCollisionService.PartitionObject(...)`:

1. validates that the collider belongs to the service context.
2. asks GridForge `GridTracer.GetCoveredVoxels(...)` for topology-aware voxel coverage.
3. uses GridForge `GridTraversalState` and topology metrics as conservative
   voxel-position padding.
4. suppresses duplicate voxel visits with GridForge traversal helpers and
   context-local sets.
5. checks that the voxel position falls within the collider bounds.
6. rents or reuses a `PhysicsPartition` on the voxel.
7. stores the collider's `WorldVoxelIndex`.
8. adds the collider ID to the partition dynamic or static membership set.

When a collider leaves a voxel, Gravitas removes the collider ID from the
partition but keeps the empty `PhysicsPartition` attached to that voxel. Empty
partitions are inactive and query-invisible, while future movement back into
the same voxel can reuse the existing partition without going through
GridForge's partition add path again. Empty retained partitions are not kept
forever: `PhysicsSettings.RetainedPartitionTimeToKillFrames` controls the
deterministic frame window, and
`PhysicsSettings.RetainedPartitionRetirementSweepBudget` bounds how many
retained partitions the collision service checks per distribution step. Expired
empty partitions are removed from their voxels and returned to the
context-local partition pool. `GravitasWorldContext.Reset()` is a stronger
session boundary: it detaches retained Gravitas partition payloads from
GridForge voxels, clears retained tracking, and clears partition pools before
collider IDs are reused.

`PhysicsPartition` is a GridForge voxel partition payload. It stores
context-local collider IDs, not collider references. Dynamic and static
membership uses `SwiftSparseSet`: lookups and removals are keyed by collider ID,
while dense-key storage keeps partition iteration compact. Do not treat dense-key
order as a semantic ordering rule; deterministic pair ordering must be explicit
at the pair or service layer. The owner service is required before the partition
is added to a voxel. The partition returns to that owner service pool when
GridForge removes the voxel partition itself, either through retained-partition
retirement or world/grid cleanup.

Current mobility membership is explicit: bodyless colliders and colliders whose
body has all translation axes frozen are added to `ContainedStaticObjects`;
bodies with `IsKinematic == true` are added to `ContainedKinematicObjects`;
movable non-kinematic bodies are added to `ContainedDynamicObjects`. Dynamic
partitions also keep `ContainedAwakeDynamicObjects`, a second sparse set for
dynamic collider IDs whose bodies are currently awake for collision work. Only
dynamic membership activates solver partition work. Sleeping bodies stay in
normal dynamic membership so queries, wake propagation, pair cleanup, and
contact lifecycle can still find them.

`PhysicsPartition2D` mirrors the same ID-first lessons for pure 2D. Bodyless
2D colliders and fully position-frozen 2D bodies are static members, kinematic
2D bodies are kinematic members, and movable 2D bodies are dynamic members.
Partial planar freezes remain dynamic. Only awake dynamic IDs activate pair
distribution. Sleeping 2D bodies remain query-visible in dynamic membership,
but partitions with no awake dynamic IDs skip solver work. Empty 2D partitions
are retained, retired by the same deterministic TTK settings, and returned to
the 2D collision service's partition pool through GridForge voxel removal.

Large-object-count optimization stays inside deterministic broad-phase and
solver ownership:

- spatial partitioning is GridForge-backed voxel partitioning with retained
  `PhysicsPartition` and `PhysicsPartition2D` payloads.
- broad phase uses collider bounds, mobility buckets, awake-dynamic sets,
  duplicate-pair suppression, hierarchy filtering, layer filtering, and explicit
  local collider filtering where configured.
- narrow phase runs only after broad-phase candidate reduction and then uses the
  exact shape-pair path for the current collider families.
- temporal coherence is captured through retained collision pairs,
  pair-owned manifolds, warm-start impulses, retained partitions, sleep state,
  and CCD frame caches.
- authoritative collision detection is not asynchronous. Hosts can run several
  independent contexts on separate threads, but one context's collision and
  response phases must keep a stable observable order.
- object-importance throttling is not a runtime collision policy. Pair culling
  may delay checks only for non-colliding retained pairs according to stable
  distance, velocity, age, and size scores; it must not skip active contacts or
  nearby candidate pairs because a host considers an object less important.
- collision LOD is authored data, not camera-distance runtime mutation. A host
  or offline tool may choose simpler fixed collision shapes for a simulation,
  but Gravitas should not change authoritative collision geometry during a run
  based on renderer distance or presentation priority.

## Collider Runtime Shape State

`LSCollider` and `LSCollider2D` separate collider identity and host binding from
the dense mutable state used by shape rebuilds, partition ownership, query
duplicate suppression, pair cleanup, and hierarchy filtering. The 3D
runtime-shape snapshot watches:

- world-space center.
- rotation.
- local scale.
- local offset.
- unscaled local size.
- unscaled local radius.

Mutating `LocalOffset`, `Radius`, or `Size` marks the runtime shape dirty.
Changing host/body scale, position, or rotation is detected from the snapshot on
the next `Simulate()` call. If the snapshot has not changed, the collider skips
the rebuild and keeps its existing partition state.
Shape mutation wakes a sleeping bound body before the broad phase refreshes, so
changed bounds cannot remain hidden behind a sleeping partition entry.

Pure 2D colliders use the same state-helper pattern where the payload is
dimension-free. `ColliderQueryState` is shared directly. Runtime-shape dirtying
reuses the same version/commit helper with a 2D snapshot payload, while pair
state uses the same generic helper over 2D collision pairs. Hierarchy state is
shared across 2D and 3D through dimension-tagged `ColliderHierarchyKey` values,
so cross-dimensional parent/child and sibling filters do not alias plain
collider IDs. Only partition coordinates remain 2D-specific because they store
X/Z planar coverage. A standalone 2D collider whose center, rotation, local
offset, or shape version has not changed skips `FixedBoundArea` rebuilds and
partition refreshes. A 2D compound part also includes inherited local scale in
its private runtime-shape snapshot so authored part scale changes rebuild the
owning aggregate without adding host-scale lookup cost to ordinary standalone
2D colliders.

Partition state tracks grid coordinates, previous broad-phase coverage bounds,
last mobility kind, partition-change flags, and broad-phase versioning together.
Mobility changes such as dynamic -> kinematic -> position-frozen refresh partition
state even when bounds do not change, and clears remove collider IDs from the
bucket they were previously inserted into. Query state tracks the raycast and
circle-query versions used by context-owned query services to suppress duplicate
collider hits. Pair state owns the one-sided collision-pair dictionary and the
opposite-side holder set; both are allocated lazily so colliders that never form
pairs do not pay for pair containers up front. Broad-phase versioning advances
from committed runtime-shape changes, so collision pairs do not need separate
collider position/rotation dirty flags. If the runtime snapshot changes,
bounds/partition state refreshes and pairs observe the broad-phase version
change.

Hierarchy state is explicit. Hosts call `child.SetParent(parent)` after collider
initialization when two colliders belong to the same engine object or aggregate
body and should not collide with each other. Gravitas stores the top parent as a
dimension-tagged collider key for filtering; it does not walk host transform
trees at simulation time. When a parent collider deactivates, its child bindings
are cleared before the parent collider ID returns to the reusable ID pool,
preventing stale hierarchy keys from suppressing collisions against future
unrelated colliders. Mixed 2D/3D hierarchy filtering uses the same state; body
policy inheritance remains dimension-local by design. Mixed CCD and mixed
response use explicit cross-service pair and handoff contracts rather than
implicitly inheriting parent body policy across dimensions.

`LSCompoundCollider` is different from hierarchy binding. A parent/child
relationship links independently registered colliders that may represent
separate host objects. A compound collider owns internal geometry parts under
one collider ID, one body binding, one broad-phase entry, and one contact/event
surface. Compound parts are not registered with `GravitasPhysicsService`, cannot
be parented independently, and are scanned in stable part order by the owning
compound collider. Public `CompoundColliderPart` values are data-first authored
descriptors: each part stores a `ColliderShapeDefinition`, local offset, local
rotation, and local scale. `LSCompoundCollider` may materialize private runtime
part colliders internally to reuse the existing narrow phase, query, diagnostics,
and inertia code, but those internal colliders are not the authored asset format
and are not independent runtime identities.

`LSCompoundCollider2D` applies the same authored-data rule to pure 2D. Public
`CompoundColliderPart2D` values store `ColliderShapeDefinition2D`, local offset,
local rotation, and local scale. The owner materializes private `LSCollider2D`
part colliders in stable declaration order, keeps one 2D collider ID, one body
binding, one broad-phase entry, and one event/query/diagnostic identity, and
rejects lifecycle operations on the private parts.

Capsules rebuild their hemisphere centers, cylinder height, area, and segment
endpoints together. Short capsules collapse to a sphere-like segment and use a
sphere inertia fallback instead of producing a zero diagonal for the capsule's
main axis.

Cylinders rebuild their finite flat-capped axis segment, cap centers, height,
area, and inertia inputs together. Cylinder support intentionally treats the
shape as a real finite cylinder rather than a capsule with ignored hemispheres,
so cap separation and side separation are tested independently.

Cuboids keep face, edge, normal, and centroid arrays on the collider. Rotated
closest-surface queries walk the cached face index data directly rather than
materializing temporary face vertex arrays, because those queries are used by
SAT contact seeding.

`FixedMathSharp.FixedTransform.LossyScale` uses basis-vector scale extraction
rather than raw matrix diagonals. This matters for rotated colliders because
diagonal extraction can report near-zero scale for 90-degree rotations and
collapse derived shape state.

Mesh colliders validate vertices and triangle indices at construction time.
`MeshColliderMode` declares whether the mesh is intended as `Convex` or
`Concave`. Convex meshes may use whole-shape convex assumptions where valid.
Concave meshes are explicit triangle collision data and are legal for
bodyless, position-frozen, kinematic, and dynamic bodies.

`PhysicsMesh` owns source vertices, triangle normals, triangle areas, local
bounds, and the triangle BVH in local mesh space. Rigid movement updates the
mesh transform, inverse transform, and conservative world bounds without
rebuilding the local BVH or allocating new bounds after warmup. Mesh queries and
narrow-phase callers transform their world-space query bounds or points into
local space, query the local BVH, then transform final contact points and
normals back to world space. The full world-vertex array is retained only as an
on-demand compatibility view.

Mesh policy is explicit rather than engine-compatible by default: concave
meshes collide as triangle sets instead of being treated as one convex hull.
The concave narrow phase gathers local-BVH triangle candidates, runs
triangle-vs-shape or triangle-vs-triangle checks, and reduces contacts through
the pair-owned `ContactManifold`. Dynamic concave meshes keep topology and the
local BVH stable while rigid movement updates transform-derived state only.

For mesh-mesh pairs involving a concave mesh, Gravitas keeps the raw local-BVH
triangle-gather path rather than a direct BVH-vs-BVH traversal. This policy
preserves exact triangle candidate truth, stable same-pair contact IDs, and
zero steady-state allocations after warmup. With the current BVH node API, a
paired traversal has to transform conservative internal bounds and can expand
too many node pairs to beat the simpler gather path. The retained optimization
keeps candidate generation unchanged and reduces triangle-triangle SAT work by
testing raw axes first, normalizing only axes that can improve the stored
penetration depth and contact normal.

Mesh policy work should keep these boundaries explicit:

- Concave triangle meshes are supported for static, kinematic, position-frozen, and
  dynamic bodies, but they should be chosen deliberately because candidate
  count scales with local triangle density. Simple authored physics meshes are
  the intended raw-triangle use case; dense rendered meshes should be
  simplified, decomposed, or represented as authored convex collision assets.
- Dynamic mesh bodies that can rotate require closed-volume inertia by default.
  `MeshInertiaPolicy.RequireClosedVolume` validates one connected, consistently
  wound triangle shell where every undirected edge has exactly two incident
  triangles. Boundary, duplicate-triangle, non-manifold, inconsistent-winding,
  disconnected-shell, and zero-volume meshes are rejected with explicit
  `MeshVolumeValidationResult` values. Reversed whole-mesh winding is accepted
  deterministically.
- Open or surface-only dynamic meshes must opt in with
  `MeshInertiaPolicy.SurfaceApproximation` when angular dynamics are enabled.
  Bodyless, fully position-frozen, kinematic, and fully rotation-frozen mesh
  bodies do not consume mesh inertia and remain legal collision surfaces.
- Closed-volume mesh inertia is integrated with fixed-point signed tetrahedra
  and cached on the immutable mesh topology. `MeshMassProperties.CenterOfMass`
  is the homogeneous COM, and `MeshMassProperties.UnitMassInertiaTensor`
  preserves products of inertia about the cached reference point. Reusable
  barycentric product algebra comes from FixedMathSharp; Gravitas owns the
  physics-specific closed-volume validation and inertia integration. Mesh
  inertia shifts between the reference point, COM, and requested body-local
  point with the full parallel-axis tensor. Runtime principal-axis
  diagonalization is not part of the current solver; if needed, it should land
  in FixedMathSharp or an offline/tooling path with deterministic tie rules and
  benchmark evidence.
- Convex mesh paths remain free to use whole-shape convex tests where valid.
- Compound colliders present one collider identity to hosts and one body to the
  solver, while internally ordering primitive or convex-mesh parts by stable
  part index. They aggregate part bounds, approximate mass/inertia from the
  parts, emit one event surface, and draw part geometry through the owning
  collider ID. Public authored parts use `ColliderShapeDefinition` data rather
  than pre-instantiated child `LSCollider` objects. Concave mesh parts are not a
  compound authoring surface; concave behavior belongs to `LSMeshCollider`.
  Authored/offline decomposed collision assets should use `LSCompoundCollider`
  unless a future asset pipeline proves that mesh-owned pieces need different
  public semantics.
- Mesh colliders are supported as collision shapes and as raycast and
  swept-sphere query targets. Capsule, cuboid, finite-cylinder, convex mesh,
  and authored compound sources also have explicit 3D swept query APIs.
  `SweptSphereQueryWorker.TrySweep(LSCollider collider, ...)` is the inverse
  relationship: it sweeps a prepared sphere source against a target collider.
  The intentionally unsupported high-risk case is exact concave
  mesh-as-source sweeping or automatic runtime decomposition, which can hide
  unbounded source-triangle expansion behind a simple query call.
- Host/offline convex decomposition should feed explicit primitive or convex
  mesh `ColliderShapeDefinition` parts into the owning compound collider without
  changing the owning collider identity. Runtime automatic decomposition remains
  out of scope.
  Automatic convex decomposition is not claimed unless the chosen algorithm is
  deterministic, bounded, tested on pathological input, and benchmarked. Ear
  clipping is a 2D polygon triangulation/partitioning tool, not a complete 3D
  convex decomposition strategy.
- Mesh simplification and collision LOD should be host/offline data.
  Runtime simplification must not alter authoritative collision geometry during
  a simulation frame.
- Rigid dynamic meshes should keep local topology and BVH stable while updating
  only transform-derived state. Deformable or breakable topology changes require
  a separate invalidation/rebuild contract before support is claimed.
- `PhysicsMesh.CalculateInertiaTensor(...)` defaults to closed-volume mass
  properties. Callers that knowingly want the surface-area-weighted approximation
  must pass `MeshInertiaPolicy.SurfaceApproximation`.
- `PhysicsMesh` does not inspect body mobility, kinematic state, or angular
  force policy. `SolidBody` decides whether angular inertia is needed before it
  asks the collider/mesh for geometry-derived mass properties.

## Continuous Collision Detection

CCD is body-owned and opt-in. `PhysicsSettings.DefaultContinuousCollisionMode`
defaults to `Discrete`; `SolidBody.ContinuousCollisionMode` and
`SolidBody2D.ContinuousCollisionMode` default to `Inherit`, so existing bodies
keep the discrete integration path unless the host sets a context default,
enables a body explicitly, or assigns a top-level parent body with an explicit
CCD mode. `ColliderHierarchyState` caches the top parent when parent
relationships are bound, so `Inherit` can check the parent policy in constant
time before falling back to the context default.

The CCD path runs during body position integration, after velocity and
acceleration have produced an intended frame displacement and before the
authoritative position is committed. Kinematic bodies use the same fixed-step
window: their frame-start pose is captured before the host transform is read,
then the host-requested target pose is treated as the swept source target for
that late-simulate step. The path first uses a swept proxy derived from the
moving collider:

- 3D sphere uses its exact scaled radius.
- 3D capsule, cuboid, finite cylinder, mesh, and compound movers use the
  world-bounds sphere radius (`Bounds.Scope.Magnitude`). This is intentionally
  conservative for elongated or sparse shapes: it can stop early, but it avoids
  the false-negative tunneling risk of using the smallest bounds axis while the
  shape's wider portion passes through a target away from the center path.
- 2D circle uses its scaled radius.
- 2D AABB and convex polygon use a conservative bounds radius.
- 2D compound uses a conservative aggregate radius over its private parts.

For static-style targets, supported movers then run an exact reduction pass
before the proxy hit can be accepted:

- pure 2D circles reuse swept-circle tests, convex/AABB movers use deterministic
  swept SAT against convex targets, convex movers against circles reuse the
  circle sweep in reverse, and 2D compounds reduce through private parts while
  keeping the compound target identity.
- pure 3D sphere targets are reduced by sweeping the target sphere backward
  against the moving source collider with `SweptSphereQueryWorker`. This covers
  3D cuboid, capsule, cylinder, mesh, and compound movers for sphere-target
  false-positive rejection without duplicating 3D shape math.
- supported 3D convex movers, convex meshes, and compounds made from supported
  convex parts use the same support-mapped source sweeps as public `Query3D`
  collider-source sweeps. Concave mesh targets reduce triangle candidates back
  to the owning mesh collider; concave mesh sources remain unsupported and
  should be authored as stable convex compound parts.

Unsupported source families and mixed dynamic CCD paths without exact reducers
continue to use conservative proxy results. Those paths prefer false-positive
early stops over false-negative tunneling, but accepted dynamic hits use the
same service-level handoff queue as exact pure-dimension hits so target velocity
and remaining-time advancement stay deterministic.

`Continuous` always sweeps when the proxy radius and displacement are non-zero.
`Auto` sweeps only when the intended displacement is larger than the proxy
radius. When a hit is accepted, the body advances to the earliest swept center
time of impact, removes only the closing component of linear velocity, and then
continues through the remaining frame time with the updated tangential velocity.
`PhysicsSettings.ContinuousCollisionMaxToiIterations` bounds this same-frame TOI
consumption; the default is `PhysicsSettings.DefaultContinuousCollisionMaxToiIterations`.
`SolidBody.LastContinuousCollisionToiIterationCount`,
`SolidBody.LastContinuousCollisionToiIterationLimitReached`,
`SolidBody2D.LastContinuousCollisionToiIterationCount`, and
`SolidBody2D.LastContinuousCollisionToiIterationLimitReached` expose the most recent
step's bounded-solver status for deterministic diagnostics.

Rotational CCD is layered onto the same body-owned opt-in contract for dynamic
2D and 3D bodies. When a body has angular displacement for the frame, Gravitas
builds a conservative angular candidate radius, gathers static-style targets,
and samples a bounded deterministic sequence of intermediate poses. Each sample
refreshes runtime shape state and uses the ordinary exact narrow-phase before a
rotational contact is bracketed. The sample count is derived from angular
displacement, capped by `ContinuousCollisionMath.MaxRotationalSubsteps`, and
uses a fixed angular step target so replay does not depend on platform timing or
collection order. The first hit bracket is refined with a fixed-iteration
bisection over exact narrow-phase checks before the accepted rotational hit
clamps the body, stops angular motion for the frame, and removes only the linear
closing velocity component along the accepted contact normal.

The current rotational path covers dynamic angular sources against static-style
targets and host-driven kinematic rotation as an active swept source against
static-style targets. Translational dynamic hits are handed off through the
service-level CCD queue; rotational CCD remains body-owned and static-style
target focused.

Static and kinematic CCD targets are non-trigger bodyless colliders, position-frozen
bodies, and kinematic bodies whose layers are allowed by the context collision
matrix and whose hierarchy is not excluded. Static or kinematic mesh and
compound targets are covered by the query workers, so 3D swept-sphere CCD keeps
triangle and stable part-order target behavior. Mesh sweep normals are oriented
against the sweep direction when authored triangle winding would otherwise point
with the moving source, so closing velocity removal is two-sided and
deterministic. Pure 2D and 3D CCD use internal static-style query collectors for
this leg: public sweep queries still report movable dynamic, kinematic,
position-frozen, and bodyless targets, while CCD's static leg copies only
kinematic/static partition IDs and skips movable dynamics because the
relative-motion path below owns those candidates.

Dynamic-vs-dynamic CCD uses a frame-start relative-motion model. During
`LateSimulate`, the physics services cache each movable body's start position
and predicted displacement under a context-local late-simulation token before
any individual body commits movement. A moving source compares the
static/kinematic query hit with dynamic relative TOI candidates and chooses the
earliest distance, then higher closing speed, then stable collider ID order.
This prevents opposing fast bodies from depending on dynamic body iteration
order. When bounded TOI iterations continue after an earlier hit, dynamic target
prediction is sampled from the same frame-start displacement at the elapsed
frame fraction, then swept only through the remaining frame fraction.

Kinematic active-source CCD uses the same target ordering. Static-style hits
clip the kinematic body to the earliest safe pose and write the clipped pose
back to the bound transform, because bodyless, position-frozen, and kinematic targets
cannot receive solver correction. Dynamic candidates at or before that first
static blocker receive deterministic velocity handoff at the accepted TOI and
are advanced through the remaining frame time by the owning 2D or 3D service. If
the target has already run in the current service pass, it is queued for bounded
same-frame continuation; if it has not, the pending handoff is consumed when its
own service reaches it. Handoff continuation ignores the initiating kinematic
source for that segment so the target does not immediately collide with the same
source treated as a final-pose static obstacle.

Dynamic relative CCD keeps proxy sweeps as the broad candidate stage, then
validates supported pure-dimension candidates with exact mover-shape reducers:

- pure 3D dynamic CCD supports exact source-sphere sweeps, target-sphere reverse
  sweeps, convex primitive movers, convex mesh movers, and compounds made from
  supported convex parts against dynamic 3D targets. Concave mesh targets reduce
  through bounded triangle candidates; concave mesh sources remain unsupported.
- pure 2D dynamic CCD uses exact relative mover-shape sweeps for circle, AABB,
  convex polygon, and compound movers and targets.
- mixed dynamic CCD keeps the conservative relative proxy path when no
  shape-exact mixed reducer exists. Mixed dynamic targets use the larger of
  planar radius and mixed half-thickness as their 3D proxy radius, then accepted
  hits exchange planar/3D velocity through the same bounded service-level
  handoff queues as pure dynamic CCD.

## Active Partitions

A partition becomes active when its dynamic membership transitions from empty to
non-empty. Active partitions are stored in
`GravitasCollisionService._activePartitions`.

During the pure 3D `context.LateSimulate()` path,
`GravitasPhysicsService.LateSimulate()` first integrates registered dynamic
bodies, then refreshes their collider bounds and partition membership once
before calling `GravitasCollisionService.CheckAndDistributeCollisions()`. Pure
2D mirrors that ownership in `GravitasPhysics2DService.LateSimulate()`: 2D
bodies integrate first, dynamic 2D colliders refresh once, then
`GravitasCollision2DService.CheckAndDistributeCollisions()` emits candidates
for the 2D island solver. These post-integration passes catch host command
teleports, direct body moves, forces, and accelerations made before
`Simulate()` in the same fixed step.

The collision services increment their distribution versions, copy active
partitions into reusable buffers, sort by `WorldVoxelIndex` with
allocation-free in-place sorts, and ask each active partition to distribute
candidate pairs.

`PhysicsPartition.Distribute()` checks:

- every local dynamic-dynamic unordered pair when the partition contains at
  least one awake dynamic.
- every local dynamic ID against the static-style IDs in that partition, where
  static-style includes bodyless, position-frozen, and kinematic colliders.

The dynamic, awake-dynamic, and static sparse-set keys are copied into
context-owned buffers and sorted by collider ID before pair generation with an
allocation-free `SwiftList<T>.SortInPlace(...)` or
`SwiftSparseSet.CopySortedKeysTo(...)` path from SwiftCollections. This keeps
pair/contact ordering stable even when movement churn changes sparse-set dense
storage order. `SwiftSortedList` remains a persistent sorted-membership
collection; transient per-frame scratch ordering stays in reusable `SwiftList`
buffers so the services do not add another membership structure.

If a partition contains no awake dynamic IDs, distribution returns before pair
generation. Static/static pairs are not distributed. Sleeping dynamic bodies
remain in dynamic membership, and when an awake body activates a partition the
partition emits sleeping-connected dynamic links too. The discrete island
builders then decide wake propagation and response order across the connected
contact graph. Pure 2D also pulls already-owned resting contacts adjacent to an
active 2D response body into the temporary island graph before cleanup, so a
connected sleeping edge is not lost merely because it lives in a neighboring
partition that had no awake dynamic at distribution time. A fully sleeping
island that was emitted only because another body activated the same broad-phase
voxel is not solved, so sleeping body positions do not drift from unrelated
awake bodies.

## Pair Creation And Filtering

For each candidate ID pair, the partition asks
`owner.Context.Physics.GetCollisionPair(id1, id2)`.

The physics service resolves both IDs through the owning context and filters
with `RequireCollisionPair(...)`. A pair is required only when:

- both colliders are active.
- both colliders have real shapes.
- at least one collider has a body.
- the context collision matrix allows the two layers to collide.
- the colliders are not explicitly bound as parent-child or siblings in the
  host hierarchy.

If the pair already exists, it is reused. Otherwise, the service rents or
creates a `CollisionPair`, stores it on the lower-ordered collider, and stores a
holder entry on the other collider. This gives one owning side for cleanup while
still allowing either collider to remove its relationship during deactivation.

Inside the pair, `ColliderA` and `ColliderB` are ordered for narrow-phase
dispatch and contact data, not for ownership. Shape priority wins first because
some detection paths expect the higher-priority shape in `ColliderA`. When
shape priority ties and both colliders have bodies, the higher linear speed wins
so same-shape dynamic pairs produce contact normals from the more active body
toward the other body. Equal priority and equal speed keep the original
candidate order as the stable tie-breaker.

## Duplicate Suppression

The same two colliders can share several voxels. `CollisionPair.PartitionVersion`
prevents duplicate narrow-phase work in the same distribution pass.

Each `CheckAndDistributeCollisions()` call increments the collision service
`Version`. A partition processes a pair only if the pair's partition version is
not equal to the current service version, then stamps the pair with that version.

## Pair Update And Culling

`CollisionPair.UpdateCollision()` performs pair-local checks:

1. reject inactive pairs or inactive colliders.
2. queue the pair for active-pair maintenance if this is the first update after
   activation.
3. invalidate culling if either collider changed partition state or
   broad-phase version since the last pair check.
4. either run collision work immediately or decrement the cull counter.

Fast rejection happens before narrow phase:

- squared distance between collider centers is checked against combined bounds
  scope.
- collider AABB bounds must inclusively overlap so zero-depth touching contacts
  can reach narrow phase.

If a pair was colliding on the previous check, it can reuse that state only
while both colliders keep the same broad-phase version. Shape or bounds changes
must re-run narrow phase even when object transforms did not move.

If the pair is not colliding, `CalculateCullScore()` combines distance,
relative velocity, and time-since-last-collision into a frame countdown.
Distance and age increase the delay; relative velocity reduces the delay so
fast-moving pairs are checked more conservatively. Disabled or zero culling
thresholds disable that score contribution rather than dividing by zero. Large or
explicitly protected colliders can prevent culling.

## Narrow Phase

`CollisionDetection.DoCollisionCheck(pair)` dispatches by
`ColliderSettings.GetCollisionType(...)`.

Current shape support:

| Pair | Detection path |
| --- | --- |
| Sphere/Sphere | center distance against combined radius. |
| Capsule/Sphere | closest capsule surface point to sphere center. |
| Capsule/Capsule | closest points between capsule line segments plus radii, with a deterministic fallback for degenerate capsule segments. |
| Cuboid/Sphere | closest cuboid surface point to sphere center. |
| AABox/Capsule | closest capsule line point to box center, then box surface point. |
| OBBox/Capsule | separating axes from cuboid/capsule support. |
| Cuboid/Cuboid | axis-aligned face/edge/stack manifolds for AABoxes, SAT primary contacts for oriented boxes. |
| Cylinder/Sphere | finite cylinder closest surface against sphere radius. |
| Cylinder/Capsule | finite-cylinder projection axes against capsule segment/radius projection. |
| Cylinder/Cylinder | finite-cylinder projection axes, preserving flat cap separation; parallel cap overlap emits a four-contact cap manifold. |
| Cuboid/Cylinder | cuboid vertex projection against finite-cylinder projection; cuboid face/cylinder cap overlap emits a four-contact manifold. |
| Mesh/Sphere | convex mesh uses closest surface point; concave mesh gathers triangle candidates against the sphere bounds. |
| Mesh/Capsule | convex mesh uses closest surface from the capsule line seed; concave mesh uses segment-vs-triangle closest points. |
| Mesh/Cuboid | triangle-BVH candidate scan runs per-triangle SAT against the cuboid and clips cuboid support-face contacts to authored triangles. |
| Mesh/Cylinder | triangle-BVH candidate scan tests finite cylinder volume and clips cap contacts to authored triangles; side/rim cases keep representative finite-cylinder contacts. |
| Mesh/Mesh | convex mesh uses nearby-triangle SAT; concave-involved pairs run triangle-vs-triangle candidate checks. |
| Compound/* | stable part-order scan, existing part-vs-shape narrow phase, and pair-owned manifold reduction. |

Current shape-pair matrix:

| A / B | Sphere | Capsule | Cuboid | Cylinder | Mesh | Compound |
| --- | --- | --- | --- | --- | --- | --- |
| Sphere | Supported | Supported | Supported | Supported | Supported | Supported |
| Capsule | Supported | Supported | Supported | Supported | Supported | Supported |
| Cuboid | Supported | Supported | Supported | Supported | Supported | Supported |
| Cylinder | Supported | Supported | Supported | Supported | Supported | Supported |
| Mesh | Supported | Supported | Supported | Supported | Supported | Supported |
| Compound | Supported | Supported | Supported | Supported | Supported | Supported |

`Cuboid` covers both `AABox` and `OBBox` dispatch. `Cylinder/Mesh` is
normalized to `Mesh/Cylinder` by pair priority so contact data is written in the
mesh-to-cylinder direction.

SAT and mesh candidate paths use context-owned scratch state through
`GravitasWorldContext`. `CollisionSatScratch` owns the reusable
`CollisionContext`, cuboid object-info buffers, mesh object-info buffers,
mesh/cylinder triangle buffer, concave triangle candidate buffers, and SAT axis
sets for one world context. This is
intentionally not static: concurrent worlds keep isolated scratch, while repeated
checks in the same world avoid per-check object-info construction and pool
rent/release churn. Short `collision-detection` benchmark smoke currently
reports on aggregate primitive checks, single-contact primitive manifold
generation, cylinder cap-manifold generation, axis-aligned cuboid
face-manifold generation, cuboid/cuboid SAT, mesh/cylinder and mesh/cuboid
manifold generation, mesh/mesh, compound/primitive, and concave mesh paths after
warmup. The `physics-2d` benchmark selection covers pure 2D shape-pair
manifold checks, convex/convex two-contact manifold detection, direct
single-contact angular response, and direct two-contact manifold response.

## Contact Data

The narrow phase writes a `ContactManifold` owned by the `CollisionPair`.
Manifold contacts are value types (`ManifoldContact`) with:

- stable contact identity derived from the unordered pair of world-space contact
  points.
- point on collider A.
- point on collider B.
- penetration depth.
- normal oriented from collider A toward collider B.
- optional immovable collision direction.

Manifolds currently store up to four contacts. When more candidates are offered,
the manifold keeps the deepest four and breaks depth ties by lower stable contact
identity. Exposed contact order is stable ascending contact identity.
`PrimaryContact` remains a convenience for diagnostics, tests, and callers that
need one representative contact; the response solver iterates the full
manifold. Duplicate contact identities update only when the new candidate is
deeper.

Contact data stores the narrow phase's detected depth without adding a solver
margin. Touching contacts can therefore have zero depth, and any stabilization
margin belongs to the response solver rather than hidden inside contact data.
`ContactManifold.HasContact` distinguishes unset contact data from legitimate
zero-valued fields, such as touching contacts with zero depth or contact points
at the origin. The response solver ignores pairs whose manifold has not been
written by narrow phase.

Axis-aligned cuboid/cuboid detection generates up to four contacts for
face-overlap and stacked/touching faces. Edge contact reduction naturally drops
duplicate corners and can produce two contacts; corner contact can produce one.
Parallel cylinder/cylinder cap overlap and cuboid/cylinder face-cap overlap
also emit four deterministic cap contacts. Mesh/cuboid and mesh/cylinder paths
preserve authored triangle candidate order and can write clipped multi-contact
surfaces for face/cap overlaps. Sphere, capsule, cylinder side/rim, and
oriented cuboid SAT paths still write representative single contacts when the
geometry does not describe a stable support face.

Pure 2D narrow phase writes `ContactManifold2D` into the owning
`CollisionPair2D`. The 2D manifold is fixed at two contacts because current
convex 2D face contacts need at most the incident edge endpoints. Circle/circle
and circle/convex contacts normally produce one contact; convex/convex face
overlap can produce two; compound 2D contacts scan owned parts in stable
declaration order and reduce to the deepest two owner-level contacts. The
single-contact `Contact2D` value remains an internal candidate helper for
primitive subchecks, not a public response or authoring surface.

## Response

`CollisionPair.ProcessCollision()` queues a solid response pair when detection
reports a collision and the pair should perform physics response. Pairs with
either collider marked as a trigger skip physical response; they can still flow
through contact notification.

After all active partitions distribute candidates, `GravitasPhysicsService`
sorts queued 3D response pairs by stable collider ID pair, builds
deterministic body islands keyed by `SolidBody.DynamicId`, skips fully sleeping
islands, wakes connected sleeping bodies when an island contains an awake
participant, and then solves constraints in stable island/pair order.
`GravitasCollisionService` owns broad-phase partition distribution and retained
partition cleanup, not the response island solver. Single-pair scenes stay on a
low-overhead direct response path. Multi-constraint islands run a bounded number
of response iterations from
`PhysicsSettings.DiscreteSolverIterations`; cached warm-start impulses and
positional correction are applied on the first island iteration, then subsequent
iterations refine velocity response without applying the same correction
repeatedly.

If every dynamic body in the partition is sleeping, pair generation is skipped
until a deterministic wake reason changes one of those bodies or its shape
state.

Current non-trigger response behavior is a deterministic fixed-capacity
manifold solver:

1. build up to four explicit solver contacts from the pair manifold, collider
   bodies, contact points, relative contact arms, detected depth, and normals
   oriented from collider A to collider B.
2. treat fully position-frozen and `IsKinematic` bodies as infinite mass for
   response.
3. apply immediate positional correction only for depth above
   `CollisionResponse.PenetrationSlop`; the correction is distributed by
   inverse mass, scaled by `PenetrationCorrectionPercent`, and divided across
   the active manifold contacts so a four-contact face does not correct four
   times as far as the detected penetration.
4. compute normal contact velocity from linear velocity plus angular velocity at
   each relative contact arm.
5. apply compatible cached normal and tangent impulses from the pair-local
   warm-start cache before the fresh solve. A 3D cache entry stores the previous
   solve normal and is reused only while that normal remains compatible with the
   current contact normal.
6. compute normal impulse deltas for all contacts before applying them. Positive
   fresh deltas are shared across manifold contacts so symmetric face manifolds
   do not over-respond, while negative stale-cache deltas can remove the full
   per-contact cached contribution.
7. accumulate and clamp normal impulses at zero, then apply only the delta from
   the cached value. This lets stale normal impulses unwind instead of injecting
   separating energy.
8. resolve the collider surface materials for the contact pair. Static and
   dynamic friction use the materials' friction combine policy; restitution
   uses the materials' restitution combine policy.
9. solve friction over a deterministic two-axis tangent frame derived from the
   contact normal. Tangent impulses that fit within
   `normalImpulse * staticFriction` behave as static sticking. Requests above
   that bound are clamped by `normalImpulse * dynamicFriction` so sliding uses
   the dynamic coefficient instead of the static coefficient.
10. store the solved normal, primary tangent, secondary tangent, and contact
    normal in a fixed-size pair-local warm-start cache keyed by stable manifold
    contact identity.

When diagnostics are enabled, the pair emits contact and response events in the
same deterministic order as collision processing: `Contact`, one
`ResponseImpulse` event for each fresh normal-solve delta, then body
velocity-delta events produced by cached warm-start, normal, and friction
response. The diagnostics stream is observational only; it does not change pair
ordering, contact data, or response behavior.

Response units and invariants:

- mass is body mass in the same unit model used by `SolidBody`.
- `SolidBody.InverseMass` is the raw reciprocal of body mass. Collision
  response reads constrained inverse mass along each contact axis so frozen
  translation axes contribute zero while unfrozen axes remain dynamic. Fully
  position-frozen, kinematic, inactive, and non-positive-mass bodies expose
  zero solver mass while leaving raw mass values inspectable.
- `SolidBody.FreezeAxes`, `SolidBody.CanTranslate`, `SolidBody.CanRotate`, and
  the constrained inverse-inertia helpers are the 3D response mobility contract
  used by both ordinary 3D response and mixed 2D/3D response.
- `SolidBody2D.CanTranslate`, `SolidBody2D.CanRotate`,
  `SolidBody2D.EffectiveInverseMass`, and
  `SolidBody2D.EffectiveInverseMomentOfInertia` are the pure 2D body-side
  mobility contract. `BodyFreezeAxes2D.PositionX` and `PositionY` constrain
  planar translation per axis; `Rotation` constrains yaw. Pure 2D body
  integration consumes scalar moment for host-applied torque and angular
  impulses. Pure 2D contact response consumes the same scalar moment surface
  for COM-relative normal and friction angular velocity deltas. Mixed response
  consumes constrained planar mass and scalar moment for the embedded 2D
  participant, but only from planar X/Z impulse components; vertical Y impulse
  remains constrained out of the 2D body model.
- 3D response torque arms are measured from `SolidBody.WorldCenterOfMass`.
  Collider centers remain collision-geometry references for narrow phase,
  culling, and normal fallback; they are not the implicit body COM.
- linear velocity is world units per second.
- angular velocity is radians per second around each local/world axis.
- pure 2D scalar rotation is radians around the yaw axis, and
  `SolidBody2D.WorldCenterOfMass` rotates the body-local COM offset in the X/Z
  simulation plane.
- inertia tensors are fixed-point `Fixed3x3` values supplied by the collider
  shape for the requested `SolidBody.LocalCenterOfMassOffset` and transformed by
  `SolidBody`. Primitive and aligned tensors keep a diagonal inversion fast
  path; mesh and compound tensors can preserve products of inertia and use full
  deterministic inversion. `SolidBody` keeps local and world-space tensor state
  separate so orientation refreshes are idempotent. Mesh colliders use cached
  closed-volume mass properties by default when angular dynamics are enabled and
  keep explicit surface approximation opt-in for open meshes.
- `PhysicsMaterial` is collider surface data. `LSCollider`, `LSCollider2D`,
  authored `ColliderShapeDefinition` values, and compound parts can carry a
  material. Compound parts without an explicit material inherit the owning
  compound collider material when private part colliders are materialized.
- restitution is clamped to `[0, 1]`. The default combine policy is `Minimum`
  so a low-bounce participant can dampen the pair; materials can explicitly
  choose `Minimum`, `Maximum`, `Average`, `Multiply`, or `GeometricMean`.
- closing speeds at or below `PhysicsSettings.RestitutionVelocityThreshold` use
  zero restitution to avoid resting-contact bounce.
- static and dynamic friction are non-negative Coulomb coefficients on
  `PhysicsMaterial`. Dynamic friction must not exceed static friction. Values
  above one are allowed for intentional high-friction surfaces.
- friction impulses oppose tangential contact motion and are clamped by the
  normal impulse and the resolved material coefficients. 3D pair-local
  warm-start storage records solved normal, primary tangent, secondary tangent,
  and contact normal values by contact identity; cached entries are applied
  before the fresh solve only when the current normal remains compatible with
  the stored normal. Pure 2D response applies cached normal and tangent impulses
  before the fresh solve, accumulates and clamps normal impulses at zero, and
  clamps tangent impulses to the current Coulomb bound so stale cache entries
  can unwind.
- penetration depth is a world distance from narrow phase; response slop is a
  solver invariant, not contact data.
- drag and angular damping remain integration/body behavior; contact friction is
  handled by the response solver.

Contact impulses follow the oriented normal written by narrow phase. Response
applies equal and opposite impulses to the two participants according to that
normal, effective mass, inertia, and body mobility. Sphere/sphere,
sphere/capsule, capsule/capsule, and other shape pairs should not need
shape-specific "add versus subtract" impulse rules. If a pair appears to need a
sign exception, the first thing to audit is normal orientation and pair
priority, not response impulse polarity.

Dense same-frame CCD contact chains are handled by bounded service-level
handoff queues, including cross-service mixed velocity transfer. Additional
response work should be evidence-driven: measured contact-quality regressions,
new shape families, or reducer gaps should enter the feature-work trackers
rather than live as vague wiki caveats.

## Body Sleep And Wake

`SolidBody` owns deterministic sleep state. A dynamic non-kinematic body can
sleep after its linear and angular speeds remain at or below explicit thresholds
for `SleepFrameThreshold` fixed frames. Sleeping clears accumulated force,
impulse, velocity, torque, acceleration, and pending position-correction state,
but does not remove the collider from GridForge partitions.

Current deterministic wake stimuli are:

- explicit host wake through `Wake()`.
- non-zero force.
- non-zero linear impulse.
- non-zero angular impulse or torque.
- collision with an awake body.
- kinematic host motion.
- host transform teleport.
- collider shape mutation.

Waking refreshes the collider's awake membership across its current partitions.
During 3D and pure 2D discrete response, the island builders expand collision
wake across connected dynamic contacts in deterministic body-ID order so
resting connected bodies do not remain asleep behind an awake island
participant.

## Contact Notifications

Collision pairs are queued into the physics service active-pair queue the first
time they update. During `context.LateSimulate()`,
`GravitasPhysicsService.ProcessActiveCollisionPairs()`:

- deactivates pairs that have not collided for the inactive-frame threshold.
- emits ongoing contact notifications when the pair is still active and not
  culled.
- keeps active pairs queued for future maintenance.

Sleeping contact pairs are preserved while their manifold is still known to be
colliding. This prevents a resting sleeping contact from aging out and emitting
a false contact exit simply because its partition skipped pair generation.

`LSCollider.NotifyContact(...)` emits:

- `OnTriggerEnter` and `OnTriggerExit` for trigger colliders.
- `OnContactEnter`, `OnContact`, and `OnContactExit` for body contacts.

## Deactivation Cleanup

`LSCollider.Deactivate()`:

1. clears partition membership through the owning collision service.
2. removes owned collision-pair references.
3. removes holder references from the opposite colliders.
4. clears explicit parent binding.
5. deactivates and pools pairs when pooling is enabled.
6. returns the collider ID to the context-local physics service.
7. marks the collider inactive.

Partition cleanup must flow through the owning service. Do not manually return
the same partition through a second path; that risks double-release and stale
activation state.

## Determinism And Performance Notes

- Collider IDs are context-local; never resolve an ID through another context.
- Pair ordering, partition traversal, culling, and notification timing are
  deterministic behavior and should be tested when changed.
- Avoid LINQ and iterator allocations in collision hot paths.
- Use `SwiftCollections` collections and pools where ownership is clear.
- Add focused tests for every shape-pair change, including separated,
  edge-touching, overlapping, degenerate, and rotated cases.
- Add benchmarks when changing partitioning, pair distribution, narrow phase, or
  response loops.
