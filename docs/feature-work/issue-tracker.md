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

No active issues currently tracked.

## Resolved Issues

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
