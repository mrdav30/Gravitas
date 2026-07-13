# Issue Tracker

## Tracker Rules

- Add new items when feature work uncovers a suspected bug, stale doc, test
  smell, performance anomaly, or correctness risk.
- Keep each item scoped tightly enough to fix and verify independently.
- Record the date on the item, not in this filename.
- Move an item to `Resolved Issues` only after the fix has tests or documented
  verification evidence.
- Do not use this tracker as a substitute for tests, benchmarks, or release
  notes.
- Performance issues should stay in
  [`benchmark-signal-hardening-backlog.md`](benchmark-signal-hardening-backlog.md)
  unless they become a confirmed runtime defect. Do not add performance issues
  here until they have been investigated and confirmed as runtime defects.

## Active Issues

### 3D Angular Impulse Scales Immediate Velocity By Frame Delta

**Discovered:** 2026-07-12  
**Source:** 95%-to-100% coverage hardening, 2D/3D motion parity review  
**Affected area:** `SolidBody.AddAngularImpulse(...)` units and frame-rate
invariance

The 3D angular-impulse API multiplies the supplied impulse by both inverse
inertia and `GravitasWorldContext.DeltaTime` before changing angular velocity.
Consequently, otherwise identical bodies receive different immediate angular
velocity changes from the same impulse when their context frame rates differ.
The 2D angular-impulse API and the usual physical impulse contract apply
inverse inertia without a time-step factor. The adjacent 3D linear-impulse API
uses the same time-step pattern and should be audited as part of the semantic
decision.

Resolve this as an explicit breaking API/units decision rather than removing
the factor incidentally. Add cross-frame-rate regressions for immediate linear
and angular impulse response, update XML/wiki unit documentation, and verify
collision and constraint callers that may already compensate for the current
scaling.

### Rotational CCD Can Miss Contacts Between Bounded Pose Samples

**Discovered:** 2026-07-12  
**Source:** 95%-to-100% coverage hardening, rotational CCD review  
**Affected area:** 2D rotational CCD sampling and the shared 2D/3D rotational
substep policy

Rotational CCD performs exact narrow-phase checks only at bounded substep
endpoints and refines time of impact only after an endpoint overlaps. A
confirmed 2D counterexample rotates a thin polygon from -5 to +5 degrees past a
circle at the blade tip: both endpoints are separated, the shapes overlap near
zero degrees, and the current one-step path commits the full rotation with zero
TOI iterations. The 3D rotational path uses the same endpoint sampling policy
and needs parity validation when this is resolved.

Adding one midpoint is not a complete fix because a narrower contact window can
fall between the new samples. Resolve this with a shape-aware continuous
rotational sweep or a conservative interval-bracketing policy that cannot skip
an intervening contact, while preserving fixed work bounds and deterministic
target ordering. Add shifted narrow-window regressions plus 2D/3D parity and
allocation checks.

### Extreme Convex Sweeps Can Normalize To Non-Unit Directions

**Discovered:** 2026-07-12  
**Source:** 95%-to-100% coverage hardening, convex sweep termination review  
**Affected area:** `ConvexSweepQueryWorker` displacement normalization and
fixed-budget conservative advancement

`ConvexSweepQueryWorker` derives sweep length from
`displacement.Magnitude`. At extreme representable inputs, fixed-point squared
magnitude saturates before the square root, so dividing the original
displacement by that shortened length produces a non-unit sweep direction.
The 32-iteration budget still guarantees deterministic termination and the
worker remains reusable, but hit geometry at that scale is not physically
trustworthy. Resolve this with an explicit large-vector normalization policy
shared with FixedMathSharp or with validated sweep-range limits, then add
boundary tests that preserve ordinary sweep results and deterministic failure
outside the supported range.

### GridForge Reuses Grid Spawn Tokens Across Pooled Generations

**Discovered:** 2026-07-11  
**Source:** 95%-to-100% coverage hardening, 3D partition teardown review  
**Affected area:** GridForge pooled `VoxelGrid` identity and stale
`WorldVoxelIndex` rejection

`VoxelGrid` assigns `SpawnToken = GetHashCode()` during initialization and
resets the token when returned to its pool. Reusing the same pooled instance in
the same `GridWorld` slot therefore recreates the same grid spawn token. A stale
`WorldVoxelIndex` can pass the intended generation check and resolve against a
replacement grid when its old local voxel index also exists there.

Gravitas now handles removed grids, missing voxel addresses, and detached
physics partitions without error-log spam, but it cannot distinguish a
same-slot, same-shaped replacement whose lower-stack generation token is
reused. Fix this in GridForge with a world-owned generation token that changes
on every grid allocation, then add a remove/re-add regression proving old
`WorldVoxelIndex` values cannot resolve into the replacement grid.

### SolidBody Point Transforms Use Collider Dimensions As Transform Scale

**Discovered:** 2026-07-12  
**Source:** 95%-to-100% coverage hardening, 3D compound `ScaledSize` review  
**Affected area:** `SolidBody.TransformPoint(...)`,
`SolidBody.InverseTransformPoint(...)`, and collider size semantics

The generic point-transform helpers multiply and divide by
`Collider.ScaledSize`. For primitive colliders that value includes authored
shape dimensions as well as host scale, so a local point is incorrectly scaled
by the collider's geometry. The old compound override made the mismatch worse
by returning a world-axis-aligned bounds size and then rotating it again.

The compound override has been removed rather than preserving a false local
size contract. Resolve the generic helpers against the host transform's actual
scale, define zero-scale inverse behavior, and add primitive/compound round-trip
tests across non-unit authored sizes, nonuniform host scale, and rotation.

### Convex Mesh Mode Accepts Disconnected Topology And Can Collide In Empty Bounds Space

**Discovered:** 2026-07-11  
**Source:** 95%-to-100% coverage hardening, mesh/sphere fallback review  
**Affected area:** `PhysicsMesh` validation, `MeshColliderMode.Convex`, and
convex mesh/sphere closest-surface fallback

`PhysicsMesh.ValidateInput(...)` validates counts, indices, and nondegenerate
triangles but does not validate the topology promised by
`MeshColliderMode.Convex`. Disconnected triangles can therefore be accepted as
a convex mesh. Near an empty corner of their combined AABB, the local triangle
query can return no candidates and the convex mesh/sphere path falls back to
the AABB surface, producing a contact where no authored triangle exists.

Open convex surfaces are supported intentionally, so requiring every convex
mesh to be a closed volume would reject valid floors and other planar assets.
Resolve this with an explicit semantic decision: either validate a documented
connected/convex open-or-closed topology contract, or change the empty-query
fallback so invalid topology cannot create an empty-space contact. Add a
regression that preserves valid open convex surfaces while rejecting or safely
handling disconnected input without the AABB false-positive.

## Resolved Issues

### Mesh Scale And Surface-Shell Mass Did Not Match Authored Geometry

**Resolved:** 2026-07-12  
**Source:** 95%-to-100% coverage hardening, mesh transform/mass follow-up  
**Affected area:** `LSMeshCollider`, `PhysicsMesh`, compound mesh parts, and
`MeshInertiaPolicy.SurfaceApproximation`

RCA: `PhysicsMesh.UpdatePosition(...)` preserved only translation and rotation,
so host and compound-part scale did not reach runtime mesh vertices, bounds,
normals, area, queries, collision, closed-volume properties, or inertia. The
legacy surface approximation also averaged triangle-row matrices rather than a
physical thin-shell tensor.

Fix: mesh points now use the explicit affine contract
`origin + R * (S * source)`, with a normalized rigid rotation and strictly
positive representably invertible diagonal scale. Scale and rotation are
prevalidated before standalone registration, before compound part rebuilds,
and before runtime cache mutation. Scaled bounds, face normals/areas, projected
frontal area, closed-volume covariance, COM, and inertia now match authored
geometry. The surface policy uses a stable two-pass uniform thin-shell
integration relative to scaled bounds, and checked fixed-point arithmetic
rejects collapsed, saturated, or otherwise nonrepresentable geometry and mass
properties without publishing partial state.

Immutable source vertices, triangle indices, convex SAT edge topology, support
topology, and the local triangle BVH survive pose/scale changes. SAT edge
classification now uses authored coplanarity rather than an angle threshold,
which remains valid under positive nonsingular diagonal scale. Public topology
and normal views are read-only; the unused public mesh tensor was removed.

Verification:

- Added fixed-value scaled bounds, normals, area, projected area, closed-volume
  COM/tensor, physical shell tensor, triangulation, large-translation, query,
  collision, BVH reuse, support-tree, standalone/compound lifecycle, and
  checked underflow/saturation regressions.
- Added a combined off-center compound regression with nonuniform owner and
  part scale, part rotation, owner-local COM, and arbitrary-reference inertia.
- Authoritative coverage artifact
  `TestResults/coverage-mesh-task46-authoritative-final2/b5fa3c62-a27b-4416-a20c-5454bf41b21c/coverage.cobertura.xml`
  reports 100% line and branch coverage for the new checked mesh files and the
  touched mesh collision/SAT files.
- Full `Release` tests passed 2,461/2,461; `ReleaseLean` built both target
  frameworks without warnings.
- Scale-only and scale-plus-shell benchmarks remain allocation-free. At
  subdivision 16, the post-check implementation measured about 1.122 ms and
  3.609 ms respectively under the BenchmarkDotNet short job. The final
  scale-keyed closed-volume cache measured 64.33 ns with no allocation at the
  same subdivision; its reviewed artifact is
  `artifacts/benchmarks/2026-07-12-task46-mesh-scale-cache-fix2`.

### 3D Compound Mass And Geometry Used Incompatible Frames And Measures

**Discovered:** 2026-07-12  
**Resolved:** 2026-07-12  
**Source:** 95%-to-100% coverage hardening, `LSCompoundCollider` block review  
**Affected area:** 3D compound mass distribution, inertia, owner offsets,
conservative query radius, and private part transforms

RCA: `LSCompoundCollider` distributed mass by `Area`, whose 3D meaning varied
between projected area, surface area, volume, and mesh triangle area. It also
added anisotropic part tensors without rotating them into owner-local space,
returned raw part COM offsets without applying authored local rotation, and
positioned private parts from owner `Position` rather than owner `Center`.
Consequently, owner offsets did not move part geometry and rotated compound
inertia was physically wrong. `ScaledRadius` measured only the aggregate AABB
half-size about the AABB center, allowing remote parts to fall outside query
proxy checks. The compound `ScaledSize` override exposed a world-AABB size as
though it were owner-local scale.

Fix: every 3D collider now implements an explicit compound mass-property
measure. Solid primitives and validated closed meshes use volume; explicitly
selected surface-approximation meshes use their scaled physical shell-area
measure. All-zero measures use equal authored-order center weights and nominal
mass shares. Fixed-point division assigns the exact residual mass to the last
positive-weight part when one exists, otherwise to the last authored part.
Part COM points include owner offset and authored local rotation. Center tensors
are rotated by `R*I*R^T`, clamped near zero, and then shifted through the
parallel-axis theorem. Private parts inherit owner `Center`; radius encloses the
farthest aggregate-bounds corner about that center; the false `ScaledSize`
override was removed.

Verification: regressions cover analytic sphere volume weighting and tensor,
rotated anisotropic cuboid inertia, rotated owner/part offsets, off-center closed
mesh COM, every primitive and mesh mass measure, explicit shell policy, invalid
closed-volume topology, exact residual assignment including trailing zero-weight
parts, all-zero fixed-point fallback, remote-part public query visibility,
authored-first geometry ties, capsule frontal aggregation, and degenerate cone
projection. All touched executable production types report 100% line, branch,
and method coverage; full coverage-enabled `Release` passes 2,433/2,433,
`ReleaseLean` passes 2,396/2,396, both library targets build without warnings,
and independent review approved.

### 3D Motion State Could Leak Across Reuse And Apply Incorrect Rotational Dynamics

**Discovered:** 2026-07-12  
**Resolved:** 2026-07-12  
**Source:** 95%-to-100% coverage hardening, `SolidBody.Motion` review  
**Affected area:** 3D body reset/reuse, grounded angular friction, and gyroscopic precession

RCA: `Initialize(...)` and `ResetPosition(...)` cleared visible velocities but
left queued force/torque and cached angular acceleration state intact. Grounded
angular friction wrote to the linear acceleration store after linear
integration, so the next frame overwrote it without slowing rotation.
Gyroscopic precession also added the Euler correction instead of subtracting
`I^-1(w x Iw)` and changed angular velocity after speed, direction, and
acceleration had already been cached. The first cache fix measured only the
precession delta, omitting torque acceleration applied earlier in the same
fixed step. Queued CCD handoff consumption then applied another full-frame
gyroscopic correction after the normal angular step even though the handoff
changed only linear state.

Fix: both reset paths now use the shared complete motion clear. Angular
friction accumulates in the angular store. Gyroscopic precession applies the
negative Euler term and refreshes angular motion state from the fixed-step
starting velocity after the correction. Non-torque impulse paths use their own
pre-gyro velocity as that refresh baseline.
Linear-only queued handoffs no longer rebuild unchanged inertia orientation or
run gyroscopic integration a second time.
The unused planar `AddPositionCorrection(Vector3d)` API and its serialized and
replay-hashed load-only state were deleted; collision response already uses the
full 3D immediate correction path.

Verification: RED regressions reproduced deferred motion after shell reuse and
reset, unchanged grounded angular speed, and the wrong-sign off-principal
anisotropic rotation. GREEN coverage proves exact reset poses, fresh/reused
body replay-hash equality, repeat-run gyro determinism, correct correction
sign against a final-orientation world-tensor reconstruction, and coherent
angular velocity, speed, and total torque-plus-gyro acceleration. JSON
snapshots also exclude the removed stale correction state. A service-phase CCD
regression proves queued linear handoff processing preserves angular velocity,
speed, acceleration, and rotation exactly after the normal body step.

### Synchronous 2D Contact Callbacks Could Corrupt Pair Teardown And Reuse

**Discovered:** 2026-07-12  
**Resolved:** 2026-07-12  
**Source:** 95%-to-100% coverage hardening, 2D pair/response lifecycle review  
**Affected area:** `GravitasPhysics2DService` response expansion, pair cleanup,
deactivation, and pooling

RCA: 2D contact enter/exit callbacks run synchronously while the service is
walking pair registries. A callback that deactivated a collider could mutate
the active `SwiftDictionary` enumerator and throw. An enter callback could also
remove and recycle the current pair before `ProcessCandidate(...)` appended
its local reference, allowing that object to be reused by a later collision
and solved twice.

Fix: existing response edges are snapshotted into a pre-sized service buffer
before callbacks. Pair cleanup snapshots stable keys, and direct teardown
stages nested separation ranges, removes registry ownership before notifying,
and recycles each still-current pair once. Response append paths revalidate
registered pair identity and physical eligibility after notification.

Verification: deterministic regressions cover current and later snapshotted
pair removal during expansion, stale queued bodies and rootless response rows,
cleanup removal of a later key, nested multi-pair deactivation with exact exit
counts, distinct pooled replacements, and pooled/unpooled position equality
after enter-callback removal.

### Fixed-Point Sphere Tangency Could Be Rejected By Normalization Residue

**Discovered:** 2026-07-12  
**Resolved:** 2026-07-12  
**Source:** 95%-to-100% coverage hardening, 3D raycast segment review  
**Affected area:** `RaycastSegmentWorker.CheckSphereOverlaps(...)`

RCA: the closest-point test could prove a segment touched a sphere while the
subsequent quadratic discriminant evaluated to one negative raw fixed-point
unit because the normalized segment direction was fractionally longer than
one. The quadratic guard then rejected the exact tangent.

Fix: after the closest-point overlap and outside-origin checks establish a
non-negative discriminant geometrically, the worker clamps fixed-point residue
to zero before calculating the tangent root. The same proof makes negative
sphere-root distances unreachable, so their redundant lower-bound guard was
removed.

Verification: a deterministic regression casts `(0,0,0)->(3,4,0)` against a
sphere centered at `(1/5,3/10,0)` with radius `1/50`; it failed before the fix
and now returns exactly one hit at `(27/125,36/125,0)`.

### Context Disposal Ordering Could Admit Inactive Worlds And Invalidate Disabled CCD Handoffs

**Discovered:** 2026-07-11  
**Resolved:** 2026-07-11  
**Source:** 95%-to-100% coverage hardening, world-context lifecycle review  
**Affected area:** `GravitasWorldContext` world registration/disposal and
disabled-service late-simulate CCD state

RCA: `Attach(...)` validated `GridWorld.IsActive` before taking the ownership
lock, while owned-context disposal removed its registry entry before
`GridWorld.Dispose()`. A waiting or reset-handler attach could therefore bind
an inactive or disposal-in-progress world. Separately, context late simulation
advanced the CCD frame token even when both enabled dimensional physics
services were disabled, making their untouched pending handoffs stale.

Fix: world activity validation, registration, owned-world disposal, and entry
release are now serialized under the ownership lock, with the entry retained
through world disposal. The context advances its CCD token only when at least
one dimensional physics service runs.

Verification: a public owned-world reset regression proves reentrant attach is
rejected until disposal completes. A disabled `Both`-mode regression seeds
pending 3D and 2D handoffs, advances the public context clock and hook phase,
and proves both body states remain unchanged and both handoffs remain
consumable afterward.

### Partition Teardown Logged Errors After Host Grid Removal

**Discovered:** 2026-07-11  
**Resolved:** 2026-07-11  
**Source:** 95%-to-100% coverage hardening, dimensional partition-service review  
**Affected area:** 2D/3D partition clear and awake-state refresh after host grid
lifecycle changes

RCA: both collision services resolved every stored coordinate through
`GridWorld.TryGetVoxel(...)` even after the host removed its grid. GridForge
correctly reported each unallocated grid index as an error, so one collider
could emit an error for every stale voxel during otherwise valid teardown or
awake-state refresh.

Fix: both services skip unallocated grid slots before resolution and treat
missing/replaced voxel addresses or detached physics partitions as stale
lifecycle state. Caller-owned GridForge trace scratch already guarantees unique
generated coordinates, so the duplicate hash pass was removed at the same
boundary.

Verification: removed-grid, same-slot replacement, missing-voxel, and detached-
partition regressions cover clear and awake refresh in both dimensions and
assert no error logs. Both collision service files report 100%
line/branch/method coverage; full `Release` passes 2,132/2,132; independent
review approved.

### Repeated Bodyless Initialization Could Orphan Collider Registrations

**Discovered:** 2026-07-11  
**Resolved:** 2026-07-11  
**Source:** 95%-to-100% coverage hardening, 3D partition service review  
**Affected area:** 2D/3D bodyless collider initialization, registry identity,
partition membership, and reset reuse

RCA: `InitializeWithNoBody(...)` accepted an already registered collider. A
second call assimilated the same object again, overwrote its current ID and
service indices, and left the old registry and partition membership orphaned.
After context reset, the same unregistered shell could also be rebound to a
different host agent without an explicit teardown.

Fix: both dimensional entry points reject registered colliders and foreign
host bindings before mutating collider state. A context-reset shell may be
explicitly reinitialized only through the same agent binding; full deactivation
clears the binding for general reuse.

Verification:

- Added 2D/3D regressions for registered duplicate initialization, post-reset
  foreign binding rejection, and same-agent reset reuse.
- Same-agent reuse proves restored primary partitions plus 3D raycast and 2D
  overlap-query visibility, not merely registry counts.
- Full coverage-enabled `Release` passes 2,129/2,129; affected collider files
  remain at 100% line/branch/method coverage; independent review approved.

### Deactivation Duplicated Teardown And Allowed Stale Collider Ownership

**Discovered:** 2026-07-11  
**Resolved:** 2026-07-11  
**Source:** 95%-to-100% coverage hardening, 3D grounding lifecycle review  
**Affected area:** 2D/3D body and collider deactivation, physics-service
dessimilation, partition ownership, reusable bindings, and 3D body loading

RCA: ordinary 3D body deactivation emitted
`Attempted to clear partitions for a non-partitioned collider` even though the
collider was validly registered and partitioned before teardown. Collider and
physics-service layers both removed pairs and primary/mixed partition state.
Directly deactivating a body-owned 3D collider also left its `SolidBody` active
in the dynamic registry. Separately, deactivation preserved stale 3D body,
agent, and context bindings; old bodies could later tear down or reinitialize a
collider rebound to another host, including after `GravitasWorldContext.Reset()`
cleared its ID. The 3D body load path mirrored the earlier 2D defect by allowing
inactive snapshots to retain registry ownership and active snapshots to invent
activity on an unregistered shell.

Fix: physics services are now the sole owners of registered collider teardown.
Collision services normalize partition flags and coordinates; repeated clears
return false without error. Body-owned collider deactivation delegates to the
body, while registration teardown clears reusable host bindings. Stale bodies
verify current collider ownership before teardown, and both dimensional
`Initialize()` paths reject registered or foreign-bound colliders before any
mutation. Inactive 3D body loads immediately reconcile registration; active
payloads remain inactive until explicit initialization when applied to an
unregistered shell.

Verification:

- Added body-owned, bodyless, inactive, repeated, registered-rebind, and
  post-reset foreign-binding regressions across 2D and 3D, including captured
  logger assertions and cross-context transform/partition identity.
- Added JSON and MemoryPack inactive-body transitions covering immediate
  teardown, idempotency, rejected activity invention, and explicit shell reuse.
- Updated direct partition-clear tests to require normalized state and
  idempotent false on a second clear.
- Focused lifecycle/reset/serialization suites pass 222/222; full
  coverage-enabled `Release` passes 2,123/2,123; `ReleaseLean` builds both
  targets; independent final review approved after three P1 findings were
  resolved.

### Cuboid Frontal Area Selected The Wrong Face And Ignored Diagonal Projection

**Discovered:** 2026-07-11  
**Resolved:** 2026-07-11  
**Source:** 95%-to-100% coverage hardening, `LSCuboidCollider` surface review  
**Affected area:** 3D cuboid linear/angular drag area and authored collider
geometry surface

RCA: `LSCuboidCollider.GetFrontalArea(...)` compared absolute axis dot products
as though the least-aligned axis were the most aligned, then returned one face
area. A `2 x 4 x 6` cuboid moving along world X therefore reported the X/Y face
area `8` instead of the correct Y/Z projection `24`. Non-axis-aligned motion
also discarded the other two projected face contributions.

Fix: the cuboid now computes the exact orthographic box projection as the sum
of each face area multiplied by the absolute dot product between its local axis
and the normalized world direction. The epsilon zero-direction fallback
matches the existing cylinder/cone contract. The same block removed unused
centroid and copied topology caches, duplicate cuboid-state policy, dead public
edge helpers and build hooks, and public mutable-array exposure; collision and
query consumers retain internal access to live geometry.

Verification:

- Added a public initialized-collider regression covering zero, all three
  principal axes, a three-axis diagonal, and a 90-degree rotated cuboid.
- `LSCuboidCollider.cs` reports 100% line/branch/method coverage.
- Full coverage-enabled `Release` passes 2,109/2,109, `ReleaseLean` builds both
  targets, and independent review approved with no findings.

### Capsule Drag And Inertia Ignored Direction And Hemisphere Centroids

**Discovered:** 2026-07-12  
**Resolved:** 2026-07-12  
**Source:** 95%-to-100% coverage hardening, `LSCapsuleCollider` block review  
**Affected area:** 3D capsule linear/angular drag area and solid mass properties

RCA: `LSCapsuleCollider.GetFrontalArea(...)` ignored its world direction and
always returned the perpendicular capsule silhouette. The solid inertia model
treated the two hemispheres as a sphere translated only from the cap centers,
omitting the transverse cross term from each hemisphere centroid lying `3r/8`
outward from its flat face. Positive-height capsules whose scaled radius and
both component volumes quantized to zero also divided by zero while assigning
component mass.

Fix: frontal area now uses the exact rotation-aware orthographic capsule
projection, with fixed-point overshoot clamped before its radial square root.
The cap tensor includes the combined `3*mCaps*d*r/4` transverse term. A
quantized zero-volume capsule uses the zero-radius thin-rod tensor before the
normal parallel-axis shift. The invariant-impossible post-inside-test distance
guard was removed, while cap-normal fallbacks reachable through fixed-point
magnitude underflow were retained.

Verification: RED regressions independently reproduced the direction-insensitive
drag result, missing centroid inertia, and initialization-time divide-by-zero.
GREEN tests cover zero, axial, perpendicular, diagonal, rotated, and
over-normalized fixed-point directions; exact sphere, ordinary capsule, and
shifted thin-rod inertia; and both sub-magnitude cap-normal fallbacks. The
canonical coverage artifact reports `LSCapsuleCollider.cs` at 100%
line/branch/method coverage; full coverage-enabled `Release` passes
2,422/2,422, `ReleaseLean` builds both targets without warnings, and independent
review approved.

### Inactive SolidBody2D Loads Could Preserve Or Invent Runtime Activity

**Discovered:** 2026-07-11  
**Resolved:** 2026-07-11  
**Source:** 95%-to-100% coverage hardening, `LSCollider2D` lifecycle review  
**Affected area:** `SolidBody2D.RecordData(...)`, inactive body snapshot loads,
body/collider registration teardown, and reusable shells

RCA: loading an inactive body snapshot into an initialized target assigned
`Active=false` but left the body and collider in their runtime registries.
`SolidBody2D.Deactivate()` then returned solely from the flag and could not
clean up the live IDs. The reverse transition was also invalid: applying an
active payload to the now-unregistered shell set `Active=true` without
reconstructing non-serialized dynamic/static ownership, leaving a zombie body
that could neither simulate correctly nor be explicitly initialized.

Fix: body teardown now returns early only when both activity is false and the
collider has no live registry ID. Inactive loads reconcile runtime ownership
after shape/transform restoration while bindings are valid. Snapshot activity
is accepted only for an already registered shell; an unregistered shell remains
inactive until its host explicitly calls `Initialize()`.

Verification:

- Added JSON and MemoryPack transitions covering registered active to inactive,
  immediate registry/partition cleanup, repeated teardown, attempted active
  load into the unregistered shell, and explicit reinitialization.
- Verified registered active snapshot loads retain their existing continuation
  contract.
- `SolidBody2D.cs` and `SolidBody2D.Serialization.cs` report 100%
  line/branch/method coverage, full `Release` passes 2,106/2,106,
  `ReleaseLean` builds both targets, and independent review approved.

### 2D Collider Teardown And Load Paths Could Preserve Invalid Runtime Ownership

**Discovered:** 2026-07-11  
**Resolved:** 2026-07-11  
**Source:** 95%-to-100% coverage hardening, `LSCollider2D` parity review  
**Affected area:** 2D collider activation, body-owned teardown, unbound compound
loads, and primary/mixed partition normalization

RCA: `LSCollider2D.Deactivate()` treated `IsActive=false` as equivalent to full
runtime teardown. A registered bodyless collider first made inactive therefore
kept its registry identity forever. Directly deactivating a body-owned collider
created the opposite split: it removed and unbound the collider while leaving
its owning `SolidBody2D` live in the dynamic-body registry, causing later body
teardown to throw. Separately, unbound compound collider loading rebuilt shape
state before checking for a context, and inactive loads attempted partition
cleanup even when the corresponding ownership flag was already clear.

Fix: full collider teardown no longer returns merely because collision
participation is inactive; body-owned teardown delegates to the owning body;
unbound loads leave shape state dirty for initialization; registered load paths
guard primary/mixed cleanup by actual ownership and preserve the ID gate before
repartitioning.

Verification:

- Added regressions for inactive-then-deactivate bodyless colliders and direct
  body-owned collider teardown followed by idempotent body teardown.
- Added unbound compound loading, repeated inactive load, active restoration,
  and active-payload-into-deactivated-shell coverage for JSON and MemoryPack.
- `LSCollider2D.cs` reports 100% line/branch/method coverage, full `Release`
  passes 2,104/2,104, `ReleaseLean` builds both targets, and independent review
  approved after both teardown defects were resolved.

### 2D Query Version Reuse Could Suppress Live Colliders

**Discovered:** 2026-07-11  
**Resolved:** 2026-07-11  
**Source:** 95%-to-100% coverage hardening, 2D query-stamp parity audit  
**Affected area:** `GravitasQuery2DService` raycast, sweep, and overlap-query
deduplication state

RCA: pure 2D queries wrapped their raycast and overlap counters from
`uint.MaxValue` to one and public reset rewound both counters to zero, but live
colliders retained the prior version stamps. Reusing version one could reject a
valid collider as already visited and return a false negative.

Fix: rollover clears the matching stamp family across the compact live 2D
collider registry before reusing version one. Public reset clears both stamp
families before rewinding counters. The scan is allocation-free and rollover
cost remains once per full 32-bit cycle.

Verification:

- Added red regressions for raycast and overlap counter wrap with colliders
  pre-stamped at version one.
- Added a public-reset regression proving both query families still find the
  same live collider after counter reuse.
- The query service and support files report 100% line/branch/method coverage;
  full `Release` passes 2,104/2,104 and independent review approved.

### 3D Collider Active-State Transitions Could Leave Invalid Partition Ownership

**Discovered:** 2026-07-11  
**Resolved:** 2026-07-11  
**Source:** 95%-to-100% coverage hardening, `LSCollider` lifecycle review  
**Affected area:** 3D collider activation, primary/mixed partition ownership,
query visibility, and collider state loading

RCA: the 3D `SetStatus(...)` method changed only the active flag, unlike the 2D
active-state lifecycle. Deactivated colliders could therefore remain in
primary and mixed partitions and continue appearing in spatial queries. The
load path had related ownership holes: an inactive payload cleared membership,
but a later active payload skipped primary repartitioning; an unbound shell
attempted shape rebuild before its context guard; and an active payload applied
to a fully deactivated shell attempted to partition the unregistered ID `-1`.
Repeated inactive loads also called primary partition cleanup after membership
was already gone, emitting a false invariant error.

Fix: 3D collider activation is now an explicit `IsActive` lifecycle property.
Registered deactivation clears primary and mixed membership, while reactivation
rebuilds primary membership and refreshes mixed membership when enabled.
Loading now defers unbound rebuilds, skips partition ownership for unregistered
shells, restores primary membership for registered inactive-to-active loads,
and guards idempotent primary/mixed cleanup by the corresponding partition
flags.

Verification:

- Added pure and mixed 3D active-state workflows proving partition removal,
  query exclusion, and deterministic reactivation.
- Extended JSON and MemoryPack workflows to load inactive state twice and then
  restore active primary/mixed membership on the same registered collider.
- Added unbound-load-then-initialize and active-load-into-deactivated-shell
  regressions; the latter originally failed while inserting ID `-1` into a
  `SwiftSparseSet`.
- `LSCollider.cs` reports 100% line/branch coverage, full `Release` passes
  2,091/2,091, `ReleaseLean` builds both targets, and independent review
  approved after three lifecycle findings were resolved.

### 3D Query Version Reuse Could Suppress Live Colliders

**Discovered:** 2026-07-11  
**Resolved:** 2026-07-11  
**Source:** 95%-to-100% coverage hardening, 3D query surface review  
**Affected area:** `GravitasQuery3DService` raycast, sweep, and circle-query
deduplication state

RCA: 3D queries stamp each visited collider with the current raycast or circle
version. The service reserved zero as the reset sentinel and wrapped
`uint.MaxValue` back to one, but it did not invalidate collider stamps from the
previous version-one query. The public `GravitasQuery3DService.Reset()` had the
same practical defect: it rewound service versions to zero while live
colliders retained their old stamps. The next query could therefore reuse a
collider's cached version and reject it as already visited, producing a false
negative.

Fix: raycast and circle version invalidation now scan the context's compact
live-collider registry. Public reset clears both cache families before
rewinding the service counters, while each rollover path clears only its own
cache family before advancing to version one. The scan is allocation-free and
the rollover cost occurs only once per full 32-bit version cycle.

Verification:

- Added failing raycast and circle rollover regressions with both the service
  and live collider seeded at the colliding version-one stamp.
- Added a failing standalone public-reset regression proving both ray and
  circle queries still find the same live collider after reset.
- Focused 3D query suites pass 158/158, the complete raycast source reports
  100% line/branch coverage, full `Release` passes 2,085/2,085, and independent
  review approved the registry scan and reset lifecycle.

### CCD Rejected Finite Heavy-Body Response As Zero Inverse Mass

**Discovered:** 2026-07-10  
**Resolved:** 2026-07-10  
**Source:** 95%-to-100% coverage hardening, dynamic TOI loop review  
**Affected area:** 2D, 3D, and mixed dynamic/kinematic CCD impulse response

RCA: CCD response treated a positive combined inverse mass less than or equal
to `Fixed64.Epsilon` as immovable. For supported finite masses whose inverse
mass is still representable, this rejected the pair impulse and fell back to
removing only the source body's closing velocity. A target-driven zero-time
hit could therefore freeze at impact with unresolved relative motion, while a
stagnation guard merely prevented the same hit from consuming the full TOI
budget. Simply accepting the smaller inverse mass also exposed a second fixed-
point hazard: computing the shared impulse scalar before multiplying by each
body's inverse mass could saturate even when both final velocity deltas were
representable.

Fix: equivalent 2D, 3D, mixed, and kinematic CCD response paths now reject only
nonpositive combined inverse mass and calculate per-body velocity deltas from
inverse-mass ratios before applying response speed, avoiding a saturated shared
impulse intermediate. CCD rejects near-singular constrained-axis mobility when
the constrained-to-raw inverse-mass ratio is at or below epsilon, which keeps
the ratio calculation within fixed-point resolution without rejecting fully
mobile heavy bodies. Per-body deltas scale the normal by the bounded inverse-
mass ratio before response speed so oblique components remain representable.
The 2D and 3D stagnation guards remain because zero-planar mixed hits and
near-singular fallback can legitimately leave source velocity unchanged.

Verification:

- Added a target-driven zero-time 3D pair regression using the real context
  lifecycle and exact restitution-aware final positions and velocities.
- Updated pure 2D and mixed 2D-to-3D heavy-body regressions to prove positive
  representable inverse masses resolve instead of freezing at TOI.
- Added exact huge-mass kinematic transfer coverage and a `Fixed64.MaxValue`
  equal-mass collision proving the ratio-first response avoids saturation.
- Added a near-singular constraint-policy regression proving unsupported
  mobility is rejected while epsilon inverse mass with full mobility remains
  resolvable.
- Added a high-response oblique-component regression proving normal scaling
  occurs before response speed when the opposite grouping would saturate.
- The complete leading 3D dynamic TOI resolver reports 100% focused branch
  coverage and full `Release` passes 2,052/2,052; independent review approved.

### Exhausted CCD Budget Left Pending Body Handoffs Alive

**Discovered:** 2026-07-10  
**Resolved:** 2026-07-10  
**Source:** 95%-to-100% coverage hardening, queued CCD handoff audit  
**Affected area:** `GravitasPhysicsService`, `GravitasPhysics2DService`,
`SolidBody`, and `SolidBody2D` continuous-collision handoff state

RCA: when the shared TOI budget was exhausted, each physics service cleared its
handoff queue and deduplication set but did not clear the corresponding pending
state stored on queued bodies. This affected both a zero budget and a positive
budget exhausted partway through a queue. Queue ownership also used recyclable
dynamic IDs, so deactivating one body and registering another before the drain
could let the replacement consume the stale entry. Deactivation had the same
split-ownership defect: it removed a queued body from the service without
clearing the body-local handoff. These paths left handoffs consumable during the
current late-simulate token and otherwise stale in runtime-full replay state.

Fix: the queue and its frame-local processed/deduplication sets now preserve
body instance identity instead of treating a recyclable dynamic ID as ownership.
Budget-exhaustion cleanup discards pending handoff state on every queued body
before clearing service ownership, including entries left after a partial
drain. The 2D and 3D body deactivation paths also discard pending handoffs
before deregistration, so both services use the same explicit lifecycle.

Verification:

- Added failing 2D and 3D regressions that prepare a real service frame, queue
  body handoffs, exhaust zero and positive budgets, and prove no unprocessed
  handoff remains consumable.
- Added failing 2D and 3D regressions that recycle a dynamic ID before service
  drain and prove the stale queue entry cannot consume the replacement body's
  state.
- Added failing 2D and 3D deactivation regressions that queue a handoff, remove
  the body before service drain, and prove no pending body state survives.
- Added direct-preconsumption parity coverage proving neither service counts an
  island after the body has already consumed its queued handoff.
- Focused handoff methods report 100% line and branch coverage, both complete
  CCD test classes pass 178/178, full `Release` passes 2,036/2,036, and
  independent review approved the final lifecycle and reset ordering.

### Context-Driven Mixed CCD Handoffs Could Drain Per Service Before The Shared Budget

**Discovered:** 2026-07-06  
**Resolved:** 2026-07-06  
**Source:** Coverage Workstream 7 CCD handoff branch audit  
**Affected area:** `GravitasWorldContext.LateSimulate`,
`GravitasPhysicsService`, `GravitasPhysics2DService`, mixed 2D/3D continuous
collision handoff chains

RCA: direct `GravitasPhysicsService.LateSimulate()` and
`GravitasPhysics2DService.LateSimulate()` correctly owned their local handoff
drain for standalone service calls. The context-driven mixed runtime reused the
same service method and then ran an additional context-level handoff relay
afterward. That split ownership meant `ContinuousCollisionMaxToiIterations`
could be consumed independently by each pure service before the context-level
mixed relay, making the mixed-frame budget less explicit than the public
setting implied.

Fix: the pure physics services now expose an internal begin/complete late-step
split. Direct service calls remain self-contained, while
`GravitasWorldContext.LateSimulate()` integrates 3D and 2D bodies first,
drains the shared queued CCD handoff budget once at the context level, then
completes partitioning, discrete response, active-pair processing, and sleep
updates for the services that actually ran.

Verification:

- Added mixed 3D-to-2D and 2D-to-3D handoff-chain regressions, plus an
  independent same-frame 3D/2D queued-handoff regression, with
  `ContinuousCollisionMaxToiIterations = 1`.
- Converted the 2D-to-3D kinematic mixed handoff test from a manual service
  helper to the real `GravitasWorldContext.LateSimulate()` path.
- Ran focused pure 3D, pure 2D, mixed handoff, and direct-service CCD tests.
- Ran full coverage collection: 1153 tests passed, branch coverage reached
  79.9%.

### 2D Active-State Toggle Preserved Mixed Partition Membership

**Discovered:** 2026-07-06  
**Resolved:** 2026-07-06  
**Source:** Coverage Workstream 4 serialization/replay/authoring branch audit  
**Affected area:** `LSCollider2D.IsActive`, mixed 2D/3D static collider
partition membership

RCA: pure 2D bodyless colliders can be toggled through `IsActive` without
detaching from their host binding. The setter refreshed or cleared the pure 2D
partition only. In a mixed context, a collider that already had mixed partition
membership could be deactivated while still reporting stale mixed partition
state, leaving primary and mixed ownership semantics out of parity.

Fix: `LSCollider2D.IsActive` now refreshes mixed partition membership when a
collider is reactivated in `PhysicsRuntimeMode.Mixed`, and clears mixed
partition membership when the collider is deactivated.

Verification:

- Added a red regression for a mixed-mode bodyless 2D collider whose mixed
  membership was seeded before toggling `IsActive`.
- Verified the regression failed before the setter fix and passed after it.
- Included the regression in the Workstream 4 focused serialization/replay/
  authoring test slice and full coverage run.

### Reduced SAT Helper Could False-Positive Rotated Cuboid And Convex Mesh-Mesh Paths

**Discovered:** 2026-07-06  
**Resolved:** 2026-07-06  
**Source:** Mesh-cuboid fallback SAT RCA  
**Affected area:** `CollisionDetection.Cuboid`, `CollisionDetection.Mesh`,
rotated cuboid vs cuboid and convex mesh vs convex mesh fallback contact
generation

RCA: the legacy `CollisionContext` SAT model prepared axes from face normals
only. That was insufficient for non-axis-aligned cuboid/cuboid and convex
mesh/mesh public paths because both oriented box SAT and convex polyhedron SAT
require edge-cross axes to reject certain separated configurations. Public-path
counterexamples confirmed false positives for both rotated cuboid/cuboid and
convex mesh/mesh.

Fix: rotated cuboid/cuboid now uses explicit full OBB SAT over three
representative face axes from each cuboid plus the nine representative
edge-cross axes. Convex mesh/mesh now uses full convex SAT over both meshes'
face normals plus cross products of cached SAT mesh edges. `PhysicsMesh`
builds the convex SAT edge cache once at construction time, skipping coplanar
triangulation diagonals and omitting the cache for concave meshes. The obsolete
`CollisionContext`, `CollisionObjectInfo`, `CuboidObjectInfo`, and
`MeshObjectInfo` reduced SAT path was removed.

Verification:

- Added public-path red regressions for rotated cuboid/cuboid and convex
  mesh/mesh configurations separated by edge-cross axes.
- Verified the cuboid/cuboid regression failed before the OBB SAT fix and the
  mesh/mesh regression failed before the convex mesh SAT fix.
- Ran focused shape-pair and mesh/collider suites.

### Mesh-Cuboid Fallback SAT Could False-Positive Without Edge-Cross Axes

**Discovered:** 2026-07-06  
**Resolved:** 2026-07-06  
**Source:** Coverage Workstream 1 zombie-code sweep and subagent geometry review  
**Affected area:** `CollisionDetection.Mesh`, convex mesh vs cuboid fallback
contact generation

RCA: the common mesh-cuboid triangle manifold path already checks cuboid face
normals, triangle normals, and triangle-edge x cuboid-edge axes. The fallback
path is still reachable for closed-convex cases such as a cuboid contained
inside a convex mesh, but it prepared SAT from nearby mesh triangle face normals
plus cuboid face normals only. Rotated convex mesh/cuboid pairs could therefore
overlap on all sampled face axes while separating on an edge-cross axis,
producing a false positive.

Fix: the convex fallback now performs full convex mesh vs cuboid SAT over mesh
face normals, representative cuboid face axes, and mesh-edge x representative
cuboid-edge axes using full convex mesh vertices. The obsolete nearby-triangle
mesh-cuboid scratch preparation path was removed.

Verification:

- Added a regression for a rotated cuboid separated from a convex cube mesh by
  an edge-cross axis; verified it failed before the fix and passes after.
- Added a containment guard proving the convex fallback remains reachable for a
  cuboid fully inside a closed convex mesh.
- Added a steady-state allocation guard for the convex fallback.
- Ran the focused `CollisionDetectionShapePairTests` suite.

### Rotated Cuboid Raycast Clipped The Enclosing AABB Instead Of Local Slabs

**Discovered:** 2026-07-06  
**Resolved:** 2026-07-06  
**Source:** Coverage Workstream 1 branch inventory and subagent query review  
**Affected area:** `RaycastSegmentWorker.CheckOBBoxOverlaps(...)`,
3D raycast queries against rotated `LSCuboidCollider`

RCA: rotated cuboid raycasts first clipped the ray segment against the
collider's enclosing world-space AABB, then rotated those world-space
intersection points around the cuboid. That could report hit points that no
longer lay on the original ray and could accept candidates based on the broad
box rather than the cuboid's local slabs.

Fix: `CheckOBBoxOverlaps(...)` now transforms the prepared ray segment into the
cuboid's local space, clips against local half-extents, and transforms accepted
intersection points back to world space.

Verification:

- Added `Raycast_ShouldClipRotatedCuboidInLocalSpace`.
- Verified the regression failed before the fix and passed after the fix.
- Ran the focused 3D raycast test suite.

### 3D Direct Collider Inactive Load Preserved Stale Partition State

**Discovered:** 2026-07-06  
**Resolved:** 2026-07-06  
**Source:** Coverage Roadmap E review and 2D/3D serialization parity audit  
**Affected area:** `LSCollider.RecordData(...)`, 3D bodyless collider
serialization, primary and mixed partition state cleanup

RCA: 3D direct-collider serialization correctly wrote and loaded
`Active=false`, but the inactive load branch only removed the collider from the
partition services. It did not mark the collider's own primary/mixed partition
state unpartitioned or clear cached coordinates. The matching 2D path already
cleared service membership and collider-local partition state, so 3D could
remain inactive while still reporting stale partition membership.

Fix: `LSCollider.ApplyLoadedState()` now clears collider-local primary and
mixed partition state after loading inactive collider state.

Verification:

- Added a 3D parity regression for inactive bodyless collider population.
- Verified the new regression failed before the fix and passed after the fix.
- Ran focused 2D/3D serialization tests.
- Ran full `Release`, full `ReleaseLean`, coverage collection, and
  `git diff --check`.

### Mixed Discrete Response Can Reverse Restitution-Heavy Kinematic CCD Handoff Velocity

**Discovered:** 2026-06-23  
**Resolved:** 2026-06-25  
**Source:** CCD service-level island solver validation  
**Affected area:** `CollisionResponseMixed`, mixed CCD handoff tests,
`GravitasMixedCollisionService` full-frame response ordering

RCA: the isolated pure-service CCD handoff was correct, but the later full-frame
mixed discrete response read kinematic participants through stored dynamic
`LinearVelocity`. Kinematic bodies keep that velocity at zero and expose their
deterministic host movement through the current continuous-collision frame
displacement instead. With restitution enabled, the same-frame mixed response
therefore compared a fast handed-off 3D target against a seemingly stationary 2D
source and could apply a backward bounce.

Fix: `CollisionResponseMixed` now resolves kinematic participants through their
current frame displacement velocity while still treating them as infinite-mass
participants for impulse application.

Verification:

- Added full-frame mixed regression coverage for a kinematic 2D source crossing
  a dynamic 3D target with restitution enabled.
- Verified existing symmetric kinematic mixed-source cases.
- Ran the mixed-dimension test suite.
