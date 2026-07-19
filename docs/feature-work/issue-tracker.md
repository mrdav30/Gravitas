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

The numbered queue below is the authoritative execution order. Detailed issue
records follow with their original discovery context.

### Release Workflow

- Use the `develop` worktrees under `F:/gamedevrepos` and temporary local
  project references through the dependency chain while lower-stack changes are
  under validation. Apply explicit links to library, test, and benchmark
  projects when transitive resolution is insufficient.
- Treat local links as unstaged validation scaffolding. Do not publish or
  release with them in place.
- FixedMathSharp foundation hardening is complete. The current locally linked
  radial extension also reports 100% line, branch, and method coverage, with
  1,460 standard and 1,439 Lean tests plus 8 Chronicler tests in each
  configuration passing. Its radial interval implementation is complete;
  retain the local link while the remaining Gravitas queue and the explicitly
  deferred finite-axis/sphere-construction items are triaged.
- SwiftCollections has no library-specific active issue at this checkpoint; its
  place in the sequence is a full downstream compatibility and release gate.
- GridForge's runtime-identity defect is resolved. Keep the lower stack locally
  linked while the remaining Gravitas queue is hardened so another downstream
  discovery does not force a partial release cycle.
- After the Gravitas queue closes, release the lower stack in dependency order,
  replace local links with released packages at each layer, and rerun Gravitas
  `Release`, `ReleaseLean`, coverage, replay, and relevant benchmark gates.

### Ordered Queue

1. **Gravitas:**
   [3D CCD Handoff Callback Failure Can Abandon Queue Cleanup](#3d-ccd-handoff-callback-failure-can-abandon-queue-cleanup).
2. **Gravitas:**
   [Rotational CCD Omits Dynamic And Mixed Targets](#rotational-ccd-omits-dynamic-and-mixed-targets).
3. **FixedMathSharp then Gravitas:**
   [Finite-Axis Capsule, Cylinder, And Mesh-Edge Projections Can Saturate Before Solving](#finite-axis-capsule-cylinder-and-mesh-edge-projections-can-saturate-before-solving).
4. **FixedMathSharp:**
   [Sphere Construction And Merge Paths Are Not Full-Domain](#sphere-construction-and-merge-paths-are-not-full-domain).
5. **Gravitas:**
   [Conic Query Quadratics Can Saturate Before Solving](#conic-query-quadratics-can-saturate-before-solving).

### 3D CCD Handoff Callback Failure Can Abandon Queue Cleanup

**Discovered:** 2026-07-18  
**Source:** same-frame CCD handoff dedupe final lifecycle review  
**Affected area:** queued 3D handoff consumption, `SolidBody.OnMoved`, mixed
handoffs into 3D bodies, budget counters, and replay continuity

`SolidBody.TryConsumeContinuousCollisionHandoff(...)` invokes the public
`OnMoved` delegate after consuming body-local handoff state. If that delegate
throws while the service drains its queue, control leaves
`ProcessQueuedContinuousCollisionHandoffs(...)` before the service updates its
counters or clears/discards unread entries. Starting another frame clears queue
ownership without discarding those other bodies' pending handoffs, leaving
old-token state visible to authoritative replay hashing. Pure 2D consumption
does not invoke an equivalent delegate, but mixed routing can enqueue the
affected 3D path.

Resolve this against the callback-failure paradigm established for contact
notifications: preserve the original host exception while deterministically
closing service ownership and discarding work that cannot finish in the aborted
step. Add a real multi-entry queue regression where the first consumed body's
`OnMoved` throws and prove unread and requeued states are neither directly
consumable nor replay-visible afterward. Keep the successful drain
allocation-free and define counter behavior for the partially completed batch.

### Rotational CCD Omits Dynamic And Mixed Targets

**Discovered:** 2026-07-18  
**Source:** between-sample rotational CCD final parity review  
**Affected area:** 2D/3D rotational candidate gathering, mixed collision mode,
dynamic target response, and same-frame CCD handoffs

The same-dimensional rotational path gathers only through the static-target
query surface. Pure rotation also leaves the translational active-source path
before dynamic candidate indexing or mixed routing can run. A rotating body can
therefore be protected against static geometry while still rotating through a
dynamic same-dimensional target or an embedded mixed-dimensional slab. A
kinematic rotation also cannot wake or push a dynamic target through the
existing handoff mechanism.

Resolve this as a first-class moving-pair contract rather than treating dynamic
targets as static proxies. The interval proof must bound both bodies' pose
motion, mixed mode must use dimension-tagged deterministic candidate identity,
and dynamic hits must enter the existing bounded handoff/response lifecycle.
`PhysicsRuntimeMode.Both` must continue to exclude cross-dimensional contacts.
Add pure 2D, pure 3D, both mixed source directions, dynamic handoff, replay, and
allocation regressions.

### Finite-Axis Capsule, Cylinder, And Mesh-Edge Projections Can Saturate Before Solving

**Discovered:** 2026-07-18  
**Source:** full-domain radial interval consumer audit  
**Affected area:** FixedMathSharp finite-axis geometry ownership; Gravitas 3D
raycast capsules/cylinders, 3D swept-sphere capsule/cylinder and mesh-edge
reducers, mixed swept-sphere segment-capsule reducers, and mixed circle-slab
height/cap clipping

The exact circle/sphere interval API cannot repair a finite-axis projection
after endpoint subtraction, axis normalization, perpendicular reduction, or
quadratic coefficients have already saturated in `Fixed64`. These consumers
also need both roots because axial clipping can reject the entry root while
accepting the exit root. Feeding narrowed coefficients into another exact root
solver would only hide the earlier information loss.

Design one allocation-free FixedMathSharp primitive that owns the endpoint
differences, perpendicular projection, exact radial solve, and finite axial
clipping before public narrowing. Then migrate all 2D/3D/mixed consumers as one
contract, preserving authored endpoints, deterministic feature order, and
degenerate-segment reduction. Include the existing 3D raycast-capsule defect in
this work: endpoint hemispheres are currently evaluated only when the cylinder
misses, and short-circuit endpoint checks can select an internal seam or the far
hemisphere instead of the nearest outer feature. This is mirrored as
`FMS-Issue-014` in the FixedMathSharp tracker.

The mixed circle-slab radial side and sphere cross-section are exact, but its
vertical contract remains in this work: combined half-height, local Y offsets,
and cap reconstruction can still narrow before finite-axis clipping.

### Sphere Construction And Merge Paths Are Not Full-Domain

**Discovered:** 2026-07-18  
**Source:** exact radial predicate migration  
**Affected area:** FixedMathSharp `FixedBoundSphere.CreateFromPoints`,
`CreateFromFrustum`, `CreateMerged`, and radius expansion/update paths

Exact containment exposed that several sphere construction and merge paths
still narrow endpoint differences, squared-distance ordering, or radius updates
before the result is known. The radial predicate work corrected ordinary
tilted-frustum admission but deliberately did not claim full-domain sphere
construction. Resolve this in FixedMathSharp before its next release with
focused construction/merge regressions, 2D parity where applicable, and
allocation benchmarks. This is the downstream release-tracking mirror of
`FMS-Issue-015`.

### Conic Query Quadratics Can Saturate Before Solving

**Discovered:** 2026-07-18  
**Source:** radial consumer audit  
**Affected area:** `RaycastSegmentWorker` cone intersections and
`GravitasQuery3DService` cone sweep/query reducers

Cone intersections are conic rather than radial. Their local reductions and
quadratic coefficients can saturate before discriminant/root evaluation, so
the new circle/sphere interval primitive is not an honest replacement. Design
a dedicated exact conic reducer with explicit finite-height clipping and
feature ordering; cover extreme crossings, tangency/near-miss, first-root
rejection with second-root admission, starts inside, and authored endpoint
contact. Keep it allocation-free and benchmark the query hot path.

## Resolved Issues

### Full-Domain Radial Bounds And Query Intervals Were Incomplete

**Resolved:** 2026-07-18  
**Source:** relative CCD exact-root migration and mixed-query parity review  
**Affected area:** FixedMathSharp radial predicates, bounded ray intervals, and
cross-sections; Gravitas 3D sphere-segment raycasts and mixed circle-slab/sphere
cross-section reducers

RCA: several circle/sphere predicates compared saturated squared values, and
interval consumers recomputed two-root quadratics after narrowing. Gravitas's
raw sphere-segment overload also accepted a pre-squared radius, preventing the
lower layer from preserving the authored radius across the full domain. Mixed
sphere/circle reduction separately narrowed a difference of squares.

FixedMathSharp now owns exact 2D/3D radial predicates, strict containment,
bounded entry/exit intervals with separate radius expansion, and exact sphere
cross-section radii. Misleading public squared-radius properties were removed.
Gravitas now retains actual radii through explicit `FixedBoundSphere` ownership,
parameterizes sphere-segment raycasts by the authored segment over `[0, 1]`,
preserves authored endpoints, uses exact mixed circle-slab entry/exit intervals,
and consumes an exact full-domain sphere-vs-slab cross-section helper. The old
raw `RaycastSegmentWorker.CheckSphereOverlaps(Vector3d, Fixed64, ...)` overload
was replaced by a compiler-visible `FixedBoundSphere` overload so squared-radius
callers cannot silently compile with changed semantics.

Verification includes 100% FixedMathSharp line/branch/method coverage
(9,408/9,408 lines, 3,064/3,064 branches, and 1,528/1,528 methods), full
standard and Lean suites, full Gravitas Release validation, focused ordinary,
extreme-scale, sub-raw-segment, authored-endpoint, and mixed-slab regressions,
and zero-allocation benchmarks. Gravitas ShortRun medians were 1.425/1.669 us
for sphere segments and 3.326/4.217 us for mixed circle slabs at scales 1 and
100,000 respectively, with 0 B allocated. Finite-axis
capsule/cylinder/mesh-edge projection, sphere construction/merge, and conic
quadratics remain explicitly separate active issues rather than being masked by
the radial result.

### Relative CCD Quadratic Saturation Could Miss Extreme-Range Crossings

**Resolved:** 2026-07-18  
**Source:** 95%-to-100% coverage hardening, shared relative-sweep review  
**Affected area:** FixedMathSharp radial rays; Gravitas 2D, 3D, and mixed radial
sweeps and relative continuous collision

RCA: relative sphere/circle sweeps formed their quadratic directly in Q32.32.
At large separations or displacements, squared terms saturated before the
discriminant and root were evaluated. Query reducers duplicated variants of the
same arithmetic. A crossing could collapse to a wrong endpoint candidate,
produce the wrong normal, and be rejected despite broad-phase admission.

Fix: FixedMathSharp now owns an allocation-free exact first-root solver. It
retains 65-bit endpoint differences, `Signed192` coefficients, a `Signed320`
discriminant, exact bounded-root ordering, and nearest-even conversion only at
the public `Fixed64` boundary. `FixedRay` and `FixedRay2d` expose bounded radial
intersection overloads with exact nonnegative radius expansion. Full-domain
vector direction and endpoint-distance helpers support downstream
reconstruction without saturated subtraction or squared magnitude.

Gravitas now centralizes radial admission in `RadialSweepAdmission`. Relative
CCD uses exact normalized-frame roots, validates both world endpoints, resolves
impact normals from full-domain interpolation, and admits the closed frame end
only when the separately authored endpoints round to contact. Public query
workers retain normalized directions and spatial-distance parameters, avoiding
amplification of one-raw-unit direction rounding. Safe first-root circle/sphere
reducers in pure and mixed 2D/3D paths share the same contract. Consumers that
require both roots remain explicitly queued under the separate full-domain
radial-interval issue.

Verification:

- FixedMathSharp `Release` passed 1,432 tests and `ReleaseLean` passed 1,411,
  plus eight Chronicler tests in each configuration.
- Fresh merged coverage is 9,017/9,017 lines, 2,986/2,986 branches, and
  1,504/1,504 ReportGenerator methods. All 1,498 CRAP identities are fully
  covered; the five scores above 30 remain registered complexity floors.
- Gravitas `Release` passes all 2,783 tests, including symmetric extreme
  crossings, exact frame-end contacts, unrepresentable endpoint separation,
  query-distance compatibility, and mixed routing regressions.
- Short-run continuous-pipeline means were `6.520 us` discrete, `19.900 us`
  against a thin wall, `37.264 us` for opposing dynamic spheres, and `37.159 us`
  against a position-frozen mesh, with zero managed allocation on every row.
  FixedMathSharp radial-ray means were `196.7 ns` in 2D and `119.2 ns` in 3D,
  also with zero managed allocation.

### Convex Mesh Mode Accepted Invalid Topology And Could Collide In Empty Bounds Space

**Resolved:** 2026-07-18  
**Source:** 95%-to-100% coverage hardening, mesh/sphere fallback review  
**Affected area:** mesh topology admission, closed-surface identity, exact
surface queries, collision dispatch, and full-domain fixed-point predicates

RCA: `MeshColliderMode.Convex` was only a label. Disconnected, concave, folded,
or otherwise invalid triangle sets could enter convex collision paths, while an
empty local BVH query let mesh/sphere contacts substitute the mesh AABB for
authored geometry. Saturating cross, triple-product, and squared-distance
operations also could not prove topology or nearest-surface ordering over the
full Q32.32 domain.

Fix: `Convex` now admits exactly one connected closed convex two-manifold shell
or one connected open coplanar triangulation that fills a single convex polygon.
Construction uses a deterministic exact-position-welded topology view while
preserving authored vertices and triangle order. It rejects unused vertices,
duplicate faces, disconnected components or vertex links, edge
non-manifoldness, inconsistent winding, reflex closed edges, and open folds,
holes, or overlaps. `Concave` remains the explicit arbitrary open, closed, or
disconnected surface mode. Both modes expose cached `IsClosedSurface` topology,
including exact seam handling and pinched-vertex rejection.

Closest-surface queries now seed an authored triangle, prove a conservative BVH
search cube from that exact upper bound, and fall back to a stable full scan only
when the bound is unrepresentable. Exact FixedMathSharp orientation,
triple-product, and squared-distance predicates prevent saturation from changing
validation or selection. Equal-distance winners use authored triangle index,
including exact shared-feature hits with different normals. Mesh/sphere and
circle queries no longer use AABB geometry, and the misleading public mesh
point-named support helper was removed.

Verification:

- Added adversarial regressions for disconnected, duplicate, non-manifold,
  pinched, reflex, folded, overlapping, holed, seam-welded, full-domain, global
  nearest, and authored-tie cases across construction, collision, queries,
  replay, scaling, and allocation contracts.
- The final independent review reproduced and then approved the authored-index
  exact-hit correction with no remaining findings.
- The full Release suite passed all 2,771 tests. Instrumented focused coverage
  reports 100% line and branch coverage for the new topology validator, mesh
  core, mass properties, and mesh collision dispatch; uninstrumented allocation
  regressions remain green.
- Closed-mesh construction remains deterministic and construction-only. The
  final `BuildAndValidateClosedVolume` means were `18.74 us`, `1.795 ms`, and
  `7.246 ms` for subdivision levels `1`, `8`, and `16`, allocating `10.25 KB`,
  `557.9 KB`, and `2,222.99 KB` respectively.

### Rotational CCD Could Miss Contacts Between Bounded Pose Samples

**Resolved:** 2026-07-18  
**Source:** 95%-to-100% coverage hardening, rotational CCD review  
**Affected area:** 2D/3D rotational CCD, pivot-centered candidate proxies,
deterministic interval traversal, and conservative fixed-point separation

RCA: both dimensional paths sampled only bounded substep endpoints and entered
time-of-impact refinement only when an endpoint overlapped. A thin blade could
therefore enter and leave a circle or sphere between samples. The Boolean
overlap bisection also assumed monotonic overlap even though rotational motion
can enter, exit, and re-enter within one interval. Separately, proxy radii were
shape-centered while their broad queries originated at the body pivot, so an
offset shape could rotate outside its admitted candidate volume.

Fix: rotational CCD now resolves each candidate independently through a
normalized-time interval traversal, then selects the earliest result with
collider-ID tie ordering. Each midpoint uses an exact narrow-phase test and
shape-specific closest-feature or AABB separation against an outward-rounded
bound on translational and pivot-centered angular motion. That bound includes a
pivot-radius-scaled fixed-point pose uncertainty instead of relying on a fixed
absolute tolerance. A real midpoint contact narrows the search to the earlier
half. Any interval still unresolved at the fixed depth or per-candidate node
budget clamps at its lower bound, so bounded work can produce an early
conservative stop but not a skipped contact. Candidate-local fallbacks cannot
borrow another collider's contact normal; an exact later witness from the same
target remains the upper-bound response normal if an earlier interval exhausts
the budget. Unsupported collision pairs are ignored, the traversal uses only
stack storage, and proxy radii now enclose every source point around the actual
body pivot in both dimensions. An unrepresentable pivot radius falls back to a
bounded service-registry scan rather than a full-domain spatial query.

Verification:

- Added 2D and 3D regressions for contact windows between endpoints and
  midpoints, plus offset circle/sphere regressions for pivot-centered candidate
  admission and scale-propagated evaluated-pose error coverage.
- The focused rotational, near-miss, and angular-tunneling surface passes all
  51 tests, including existing unsupported-pair and allocation contracts.
- Dense unresolved-candidate benchmarks cover 2D and 3D aggregate interval
  costs at `1`, `8`, and `32` admitted targets. The final short-run means were
  `1.80`/`4.90`/`7.23` ms in 3D and `0.73`/`1.33`/`1.98` ms in 2D.
- Benchmark and full-suite release evidence are recorded in the resolving
  commit.

### 3D Angular Impulse Scaled Immediate Velocity By Frame Delta

**Resolved:** 2026-07-18  
**Source:** 95%-to-100% coverage hardening, 2D/3D motion parity review  
**Affected area:** public force/impulse units, immediate body motion, and 2D/3D
motion API parity

RCA: both public 3D impulse methods multiplied their inverse-mass response by
`DeltaTime`, treating instantaneous momentum transfer as a continuous force.
`AddLinearImpulse(...)` additionally routed through the fixed-step velocity and
pose update, so a host command could apply gravity and move, ground, or sweep a
body before the next simulation phase. Collision, mixed-response, and
constraint solvers already applied velocity deltas directly and did not
compensate for this behavior.

Fix: 3D linear and angular impulse now apply the physical frame-rate-invariant
contracts `deltaVelocity = impulse * EffectiveInverseMass` and
`deltaAngularVelocity = impulse * EffectiveInverseInertiaTensor`. They wake and
refresh body motion immediately without advancing pose. Continuous force and
torque remain queued acceleration inputs integrated during the next fixed step.
The dead 3D pending-impulse store was removed from runtime, replay hashing, and
serialization. Pure 2D gained the matching public `AddLinearImpulse(...)`
contract, and force/torque admission in both dimensions now uses effective
mobility so kinematic or otherwise immovable bodies do not retain stale inputs.

Verification:

- Added cross-frame-rate 2D/3D regressions for immediate linear and angular
  impulse response plus a fixed-step boundary regression proving pose advances
  exactly once during `LateSimulate()`.
- Audited collision, CCD, constraint, diagnostics, and benchmark callers;
  fixtures that encoded the old frame-scaled inputs now use explicit velocity
  targets and true impulses, and CCD helpers advance through the lifecycle.
- `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj -c Release
  --no-restore` passed all 2,734 tests.
- `dotnet build src/Gravitas/Gravitas.csproj -c Release --no-restore` passed for
  `net8.0` and `netstandard2.1`; the benchmark project also built cleanly.
- `ReleaseLean` remains deferred to the package-reference release gate because
  the intentionally retained local GridForge project link exposes MemoryPack
  interfaces without its package assembly in that configuration.

### SolidBody Point Transforms Used Collider Dimensions As Transform Scale

**Resolved:** 2026-07-18  
**Source:** 95%-to-100% coverage hardening, 3D compound `ScaledSize` review  
**Affected area:** `SolidBody.TransformPoint(...)`,
`SolidBody.InverseTransformPoint(...)`, host transform scale, and collider size
semantics

RCA: both public point helpers used `Collider.ScaledSize`, which combines a
primitive's authored shape dimensions with host scale. A local point was
therefore enlarged by the collision geometry before rotation and translation.
The inverse compounded the mismatch and relied on component-wise vector
division, whose zero-divisor contract silently returns zero for that component
even though a zero-scale transform has no inverse.

Fix: both helpers retain the body's authoritative position and rotation but now
read scale exclusively from `Agent.Transform.LossyScale`. The inverse rejects
any zero world-scale component with `InvalidOperationException` before applying
the inverse rotation and component division. This keeps the implementation
allocation-free and avoids constructing or inverting a matrix per call. A 2D
parity audit found no `SolidBody2D` point-transform API or equivalent
shape-dimension conversion path, so no speculative 2D surface was added.

Verification: RED tests reproduced primitive geometry leaking into a rotated,
nonuniformly scaled point and the silent singular inverse. Primitive and
compound round trips now use only host scale, while the singular inverse fails
explicitly. Focused regressions pass `3/3`, the complete `SolidBodyIntegrationTests`
surface passes `23/23`, and full locally linked suites pass `2731/2731` in
`Release` and `2692/2692` in `ReleaseLean`. Both configurations build the
`net8.0` and `netstandard2.1` package targets with zero warnings. Both modified
methods report 100% line and branch coverage.

### CCD Handoff Dedupe Could Strand A Same-Frame Requeued Body

**Resolved:** 2026-07-18  
**Source:** 95%-to-100% coverage hardening, dimensional CCD service admission
review  
**Affected area:** 2D/3D continuous-collision handoff queues, same-frame relay
cycles, mixed CCD routing, iteration-budget ownership, and replay continuity

RCA: each service's dedupe set represented every body seen anywhere in the
current drain instead of only bodies that still owned an unread queue entry. If
body A was consumed and a later same-frame relay returned work to A,
`ApplyContinuousCollisionHandoff(...)` published new body-local pending state
but queue admission rejected A. End-of-drain cleanup then removed service
ownership without consuming or discarding that authoritative, replay-hashed
state.

Fix: both dimensional services now remove a body from the dedupe set immediately
after dequeue and before consumption. Reentrant or later same-frame relays can
therefore append A at the queue tail in stable FIFO order, while repeated
updates before dequeue retain the existing latest-state-wins dedupe. Requeued
work remains governed by the same shared iteration budget; exhaustion reaches
the existing explicit discard path. The added `SwiftHashSet.Remove(...)` is
expected O(1), allocation-free, and does not expose hash iteration order.
Mixed responses already route target handoffs through these dimensional queues,
so no parallel mixed implementation was required. Final review also exposed
the complementary latest-state edge: a terminal update could clear ignored
collider references while leaving an older pending continuation intact. The 2D
and 3D body paths now discard that superseded continuation atomically when the
new update has no remaining time or resulting motion.

Verification: symmetric 2D and 3D RED regressions use a perfectly elastic
`A -> B <- C` relay cycle that returns a handoff to A after its first dequeue.
With four iterations A is consumed again; with a three-iteration cap the new
entry exposes the limit and is explicitly discarded. Both variants prove no
directly consumable state remains and that a subsequent explicit discard cannot
change the authoritative replay hash. Existing pre-dequeue dedupe behavior is
also exercised because B receives multiple updates before its single entry is
consumed. Separate terminal-update regressions prove a queued continuation is
cancelled when a newer handoff has no time or velocity, without consuming a
false iteration or changing replay state during later cleanup. The focused
regressions pass `6/6`; the complete 2D, 3D, and mixed CCD surface passes
`417/417`. Full locally linked suites pass `2729/2729` in `Release` and
`2690/2690` in `ReleaseLean`; both configurations build the `net8.0` and
`netstandard2.1` package targets with zero warnings. Both modified queue methods
report 100% line and branch coverage.

### 3D Exit Callback Failure Duplicated Reentrant Separation Notifications

**Resolved:** 2026-07-18  
**Source:** 95%-to-100% coverage hardening, 3D collision-pair lifecycle review  
**Affected area:** `CollisionPair.NotifyCollidersOfContact()`,
`CollisionPair2D.NotifyColliders()`, `CollisionPairMixed.MarkColliding()`, and
callback exception/reentrancy teardown

RCA: 3D separation flags remained admitted until after both user delegates
returned. If collider A's exit callback reentrantly deactivated the pair and
then threw, the outer flag clears were skipped and deferred cleanup saw A as
still admitted. `EndNotification()` also cleared `_notificationInProgress`
before invoking deferred exits, allowing a captured pair to recurse directly
into the same separation. A repeated A failure could mask the original while B
was skipped and notification state remained live.

Parity review found the same first-failure short-circuit and deferred cleanup
masking in 2D. Mixed pairs already consumed their exit flags before delegates,
but still skipped the 2D side after a 3D failure and released the notification
guard before deferred exit dispatch.

Fix: each 2D, 3D, and mixed pair dispatcher now consumes exit admission before
invoking user code. Both still-current sides are attempted in stable pair order
even when the first throws. One failure is rethrown with its original stack;
multiple failures use `AggregateException` in callback order. Deferred exits
retain the notification guard through dispatch, and final cleanup always clears
pending state before the pair becomes reentrant again. Exception storage uses
`SwiftList` and is allocated only after a delegate throws; successful
notification paths remain allocation-free.

Verification: RED regressions reproduced the duplicate A exit after B was
rebound, direct captured-pair reentry from a deferred exit, skipped B dispatch,
and first-callback short-circuiting. GREEN assertions prove at-most-once
delivery, untouched rebound lifetime state, complete owner/holder cleanup,
stable exception order, and no notifying-shell pool reuse. Matching 2D and
mixed RED regressions reproduced first-callback short-circuiting and early
deferred-guard release; GREEN assertions prove stable A/B and 3D/2D exception
order with both guards retained through exit dispatch. The combined lifecycle
surface passes `125/125`; full locally linked suites pass `2723/2723` in
`Release` and `2684/2684` in `ReleaseLean`. Both configurations build
`net8.0` and `netstandard2.1` package targets with zero warnings.

### Continuous-Collision Modes Accepted Undefined Enum Values

**Resolved:** 2026-07-17  
**Source:** 95%-to-100% coverage hardening, 3D CCD helper review  
**Affected area:** `PhysicsSettings.DefaultContinuousCollisionMode`,
`SolidBody.ContinuousCollisionMode`, `SolidBody2D.ContinuousCollisionMode`, and
replay/settings population

RCA: the context setting and both body properties assigned the byte-backed enum
without validating that the value was declared. Chronicler also populated the
body backing fields directly. Undefined values could therefore survive public
assignment or serialization, then resolve as neither continuous nor automatic
and silently use discrete movement.

Fix: one enum-adjacent validation policy now admits only `Inherit`, `Discrete`,
`Continuous`, and `Auto`. Context settings and both body properties reject
undefined values with `ArgumentOutOfRangeException` before changing their
stored mode. Body replay loads through a validated local value, so a rejected
payload does not publish the corrupt mode, while `PhysicsSettingsSaver`
materialization rejects invalid authored or deserialized values before
replacing context settings. Context-level `Inherit` remains valid and
deliberately resolves to `Discrete` when no concrete body or hierarchy override
exists.

Verification: first-invalid (`4`) and byte-maximum (`255`) regressions cover
3D and 2D body assignment, context settings, settings application, and both
body replay paths, including preservation of the previously valid value after
rejection. Existing 2D and 3D context-`Inherit` behavior remains covered. The
focused suite passes `14/14`; the full locally linked suites pass `2716/2716`
in `Release` and `2677/2677` in `ReleaseLean`. Both configurations build the
`net8.0` and `netstandard2.1` package targets with zero warnings.

### Non-Unit Quaternion Admission Can Collapse Runtime Shape Axes

**Resolved:** 2026-07-17  
**Source:** 95%-to-100% coverage hardening, cone-bounds fallback review  
**Affected area:** `SolidBody` rotation admission, compound-part local
rotations, collider shape state, and replay/load population

RCA: `FixedTransform` already scale-safely normalized host rotations, but
`SolidBody.Initialize(...)`, public body rotation mutators, and Chronicler load
wrote raw quaternions into authoritative body state. `CompoundColliderPart`
likewise retained its raw local rotation. Operator-based shape transforms could
therefore consume a collapsed axis while normalized basis helpers observed a
different orientation, making bounds, normals, mass properties, queries, and
diagnostics disagree.

Fix: every public 3D body rotation admission and replay-load path now
scale-safely normalizes before publishing authoritative or visual state.
`CompoundColliderPart` normalizes its local rotation once at construction, so
runtime parts and replay hashing share that exact stored orientation. Zero maps
to identity, scaled and saturated inputs use FixedMathSharp's full-domain
normalization, and already near-unit values retain their deterministic
representation. Direct `PhysicsMesh` transform APIs remain intentionally
strict and still reject non-normalized rotations; compound mesh parts reach
that validation only after descriptor normalization.

Verification: focused regressions cover zero, scaled, saturated, near-unit, and
axis-collapsing quaternions across body initialization, public mutation, visual
state, compound primitives, compound meshes, cone bounds, replay population,
and direct mesh rejection. The full locally linked suites pass `2704/2704` in
`Release` and `2665/2665` in `ReleaseLean`; Lean builds both `net8.0` and
`netstandard2.1` and produces both packages with zero warnings.

### Registered Joints Can Outlive Their Body And Collider Lifetimes

**Resolved:** 2026-07-17  
**Source:** 95%-to-100% coverage hardening, 3D joint replay-hash lifecycle
review  
**Affected area:** 2D/3D joint ownership, body/collider deactivation and reuse,
linked-collision suppression, replay identity, and ragdoll lifecycle

RCA: joint services owned registrations independently from endpoint body and
collider lifetimes. Deactivating an endpoint released its reusable collider ID
and broadly deleted matching suppression keys, but left the joint active and
enabled. Same-shell reuse could therefore resume a stale joint without its
filter policy, while rebinding the collider to another body exposed the old
joint to a new registry identity.

Fix: 2D and 3D constraint services now index joint IDs by endpoint body and
remove affected registrations before collider identity release. Removal
reconciles exact ref-counted suppressions and never scans the context's peak
joint range or the whole suppression table. Intrusive endpoint links make each
unlink O(1), preserve reverse-registration teardown order, and keep total body
teardown O(attached joints). Ragdolls are atomic registrations: a body can
belong to one registered ragdoll, link teardown or
`RemoveRagdoll(...)` removes the runtime and all owned joints, independent
owned-joint removal is rejected, and stale joint/ragdoll handles cannot mutate
or serialize later simulation lifetimes. Their intrusive registration-order
chain gives O(1) removal without making replay hashes depend on removal history.
Context reset and disposal invalidate all handles, and mutating constraint APIs
reject a disposed context.

Verification: symmetric tests cover both endpoint positions, multiple attached
joints, same-shell reinitialization, different-body collider rebinding, solver
admission, counts, exact suppression removal, automatic-versus-explicit replay
hash equivalence, zero-joint ragdolls, out-of-order ragdoll removal, overlapping
and duplicate link admission, stable replay hashes across ragdoll removal
orders, reset/disposal invalidation, stable endpoint teardown order, and
stale-handle mutation and serialization. The focused constraint, context, and
replay-writer suite passes `168/168` in `Release` with the locally linked lower
stack; the full suites pass `2692/2692` in `Release` and `2653/2653` in
`ReleaseLean`. Every source method changed by this resolution has full line and
branch coverage. Project-wide coverage is `99.9%` line and `99.8%` branch; the
remaining six lines and eleven branches are confined to the separately queued
continuous-collision handoff paths.

### GridForge Reuses Grid Spawn Tokens Across Pooled Generations

**Resolved:** 2026-07-17  
**Source:** 95%-to-100% coverage hardening, 3D partition teardown review  
**Affected area:** GridForge pooled `VoxelGrid` identity, exact traversal, and
Gravitas 2D/3D/mixed partition and query consumers

RCA: GridForge derived world and grid allocation tokens from `GetHashCode()`
values. Removing and re-adding an identical pooled grid could reuse both the
world-local slot and token, allowing a stale `WorldVoxelIndex` to resolve
replacement state. Hash-derived voxel tokens also made hash collisions capable
of suppressing distinct traversal results.

Fix: GridForge commit `0c5420f` added process-unique 64-bit world identity and a
nonrepeating grid generation owned by each world, preserved across
non-deactivating reset. `WorldVoxelIndex` now validates the world token, the
recyclable `GridIndex`, the grid generation, and the voxel coordinate. GridForge
commit `cc2c451` changed unique traversal to
`SwiftHashSet<WorldVoxelIndex>` and removed hash-derived voxel and scan-cell
identity. Gravitas commit `598c2de` consumes the exact key in 3D query
deduplication and adds same-configuration replacement regressions for pure 2D,
3D, and mixed partitions.

Verification: with Gravitas locally linked to the corrected GridForge, each
2D/3D/mixed regression proves the stale coordinate fails, the replacement grid
receives a different generation, and its live partition resolves normally. The
focused Gravitas identity/query/order suite passed `159/159`; GridForge's exact
traversal regressions cover duplicate suppression, synthetic hash collisions,
and `0 B` warm reusable-set traversal. The local project links remain temporary
uncommitted release-validation scaffolding.

### Extreme Convex Sweeps Can Normalize To Non-Unit Directions

**Resolved:** 2026-07-14  
**Source:** 95%-to-100% coverage hardening, convex sweep termination review  
**Affected area:** FixedMathSharp vector magnitude, normalization, comparison,
and averaging; Gravitas 2D, 3D, and mixed query/CCD sweep admission, GJK,
conservative advancement, and concave-mesh hit geometry

**Follow-up status:** Closed by
[`FixedMathSharp Foundation Hardening`](../../../FixedMathSharp/docs/feature-work/done/2026-07-14-fixedmathsharp-foundation-hardening-plan.md).
FixedMathSharp now owns the shared full-domain arithmetic and Gravitas consumes
it without local overflow helpers. The odd-raw GJK expansion boundary is fixed
and committed. The separately tracked relative-CCD quadratic issue remains
active.

RCA: fixed-point squared magnitude saturated before the square root. Dividing an
extreme vector by the shortened result produced a non-unit direction, while
saturating endpoint subtraction could publish a different displacement than the
caller requested. Fixed-coordinate GJK tolerances, support projection, same-sign
triangle-centroid sums, and whole-mesh normal rediscovery introduced additional
range and feature-identity failures after the initial direction was formed.

Fix: FixedMathSharp now owns exact raw squared-magnitude representability and
comparison across 2D/3D/4D vectors, scale-safe magnitude and normalization
fallbacks, explicit `TryGetMagnitude(...)` APIs, and an overflow-safe
three-value `FixedMath.Average(...)`. Gravitas rejects unrepresentable endpoint
or relative-motion construction, uses adaptive GJK working coordinates, exact
support ordering and conservative lower-bound projection, preserves same-pose
intersection witnesses, and resolves normals from the actual winning feature.
Concave triangle shapes retain their mesh owner and triangle ordinal, so their
transformed face normal is O(1) and cannot be replaced by BVH query order.

Verification:

- Red regressions reproduced non-unit tiny/extreme normalization, saturated
  endpoint and relative displacement, false GJK/support outcomes, a false
  distance-zero near-maximum triangle hit, and adjacent-face normal drift.
- Final FixedMathSharp verification passed `1,398` Release and `1,377`
  ReleaseLean tests, plus `8` Chronicler tests in each configuration. Its merged
  artifact reports `8,679/8,679` lines, `2,924/2,924` branches, and
  `1,469/1,469` methods. Focused vector magnitude and normalization benchmarks
  remained allocation-free.
- SwiftCollections passed `1,091` Release and `1,063` ReleaseLean tests;
  GridForge passed `431` in each configuration through explicit source links.
- Gravitas passed `2,659` Release and `2,620` ReleaseLean tests after consuming
  the final FixedMathSharp contracts and removing release-only assertions.
- Convex sphere-target sweeps remained allocation-free. Removing the whole-mesh
  normal rescan improved dense concave sweeps at 8/16/32 subdivisions from
  `117.22 us` / `436.02 us` / `1.6676 ms` to `77.58 us` / `277.52 us` /
  `1.0848 ms`, also with zero allocations.
- Independent task reviews and the final FixedMathSharp coverage review reported
  no remaining findings after arithmetic ownership and the odd-raw expansion
  boundary were corrected.
- The separate relative-CCD quadratic saturation issue remains active; this work
  validates its inputs but does not replace its scale-sensitive quadratic.
- Local project links remain unstaged and must be removed before package release
  validation.

### Extreme Collider Bounds Underestimated CCD Proxy Radius

**Resolved:** 2026-07-13  
**Source:** 95%-to-100% coverage hardening, 3D CCD helper review  
**Affected area:** FixedMathSharp vector magnitude/distance and Gravitas 2D/3D
continuous-collision proxy radius, candidate admission, and `Auto` gating

RCA: fixed-point vector magnitude and distance squared every component before
taking the square root. Once the square sum saturated at `Fixed64.MaxValue`, a
larger but still representable length collapsed to approximately `46,340.95`.
Gravitas also compared saturated squared distances directly in 2D convex and
compound proxy loops and in `Auto` threshold checks.

Fix: FixedMathSharp retains its direct square/root path for ordinary values and
uses max-component scaling only when the square sum saturates. Gravitas keeps
its squared fast paths and falls back to robust distances only on saturation; an
unrepresentable `MaxValue` displacement conservatively enables CCD.

Verification:

- Red regressions reproduced underestimated 2D/3D/4D magnitudes, convex and
  compound proxy radii, and incorrect `Auto` threshold decisions.
- Near-unit distance regressions preserve the original raw fixed-point result;
  signed extreme endpoint tests cover each vector dimension.
- FixedMathSharp passed `1,149` Release and `1,128` ReleaseLean tests;
  SwiftCollections passed `1,091` and `1,063`; GridForge passed `431` in each
  mode; Gravitas passed `2,563` and `2,525`.
- Final magnitude and dynamic CCD benchmarks remained allocation-neutral and
  within the established baseline variance.
- Independent review found and verified the near-unit distance correction, then
  reported no remaining Critical or Important issues.
- Local project links remain unstaged and must be removed before release.

### FixedMathSharp Rays Now Treat Only Exact-Zero Slab Directions As Parallel

**Resolved:** 2026-07-13  
**Source:** 95%-to-100% coverage hardening, shared segment-box clipping review  
**Affected area:** FixedMathSharp `FixedRay` and `FixedRay2d` slab intersection

RCA: both ray slab helpers classified direction components at or below
`Fixed64.Epsilon` as parallel. `Fixed64.FromRaw(1)` is representable motion, so
a ray beginning one raw unit outside a slab could reach its boundary at the
endpoint but incorrectly report no intersection.

Fix: FixedMathSharp slab clipping now treats only exact zero as parallel. The
separate 3D near-zero policy remains unchanged for plane and frustum
classification, and the unused 2D tolerance helper was removed.

Verification:

- Positive and negative one-raw endpoint regressions failed with `null` before
  the fix and now return `Fixed64.One` in both 2D and 3D.
- Exact-zero outside-slab controls continue to return no intersection.
- Full `Release` and `ReleaseLean` suites passed through FixedMathSharp,
  SwiftCollections, GridForge, and Gravitas using explicit local project links.
- Focused BenchmarkDotNet runs remained allocation-free and statistically
  neutral for 2D area and 3D box ray intersections.
- Independent review found no correctness, scope, API, determinism, or
  performance issues.

### FixedMathSharp Vector Midpoints Saturated Before Halving

**Resolved:** 2026-07-13  
**Source:** 95%-to-100% coverage hardening, physics-material average review  
**Affected area:** FixedMathSharp scalar and vector midpoint helpers plus
Gravitas `PhysicsMaterialCombine.Average`

RCA: `Vector3d.Midpoint(...)` and `Vector4d.Midpoint(...)` computed each
component as `(left + right) * Fixed64.Half`. Saturating addition therefore
discarded half the magnitude before halving equal extreme endpoints.

Fix: FixedMathSharp now owns a branchless, overflow-safe
`FixedMath.Midpoint(...)` primitive with nearest-even raw rounding. Both vector
helpers delegate per component, and Gravitas delegates material averaging to the
shared primitive instead of retaining a duplicate raw algorithm.

Verification:

- Regressions cover equal maximum and minimum values, opposite extremes,
  positive and negative odd-raw ties, operand symmetry, and distinct vector
  components.
- Independent review checked 2,048,697 operand pairs with no arithmetic or
  symmetry mismatch.
- Full `Release` and `ReleaseLean` suites passed through FixedMathSharp,
  SwiftCollections, GridForge, and Gravitas using explicit local project links.
- Focused BenchmarkDotNet runs remained allocation-free and reduced the
  1,024-pair vector midpoint jobs from `20.568 us` to `1.017 us` for `Vector3d`
  and from `29.024 us` to `1.418 us` for `Vector4d`.
- Local project links remain unstaged and must be removed before release;
  Gravitas will transition to the published package after FixedMathSharp ships.

### Overlong Settings Collision Matrix Rows Were Silently Truncated

**Resolved:** 2026-07-13  
**Source:** 95%-to-100% coverage hardening, final settings branch review  
**Affected area:** `PhysicsSettingsSaver.CreateCollisionMatrix()`

RCA: settings load validation required each collision-matrix row to contain at
least the outer row count, then copied only that many entries. A longer row was
therefore accepted and silently truncated even though the public failure message
and matrix contract require square data. A missing row was guarded in production
but lacked a regression proving deterministic failure instead of null
dereference.

Fix: row validation now requires exact length. Separate regressions cover short,
overlong, and missing rows; all malformed shapes throw the explicit
square-matrix `InvalidOperationException`.

Verification:

- The overlong-row regression failed before the fix because no exception was
  thrown, then passed with exact-length validation.
- Removing the null-row guard changes the declared settings error into a
  `NullReferenceException` and fails the missing-row regression.
- Focused settings coverage passes 7/7 with `PhysicsSettingsSaver` at 100% line,
  branch, and method coverage.
- Authoritative artifact
  `TestResults/coverage-settings-square-validation-task83-final-authoritative-root-comparable/a12df29a-6fdf-4bdb-a3ac-8c0c11751a0d/coverage.cobertura.xml`
  passes 2,555/2,555 full `Release` tests and reports 10,407/10,407 branches.

### Pending CCD Replay Hashes Depended On Deleted Collider ID History

**Resolved:** 2026-07-13  
**Source:** 95%-to-100% coverage hardening, dimensional body replay review  
**Affected area:** `SolidBody.ContributeReplayHash(...)` and
`SolidBody2D.ContributeReplayHash(...)`

RCA: pending 2D/3D CCD handoffs hashed ignored collider references using
context-local registry IDs. Equivalent contexts with the same live registration
order but different deleted-ID/free-list history could therefore produce
different authoritative hashes, contradicting the documented dense
replay-ordinal contract. Solver-cache subsections also wrote both ignored IDs a
second time even though non-null references imply a pending handoff and were
already encoded authoritatively.

Fix: authoritative ignored references now hash their prepared `ReplayOrdinal`.
The four duplicate solver-cache writes were removed. Both dimensional
authoritative-CCD and solver-cache subsection versions were incremented from 1
to 2.

Verification:

- Symmetric mixed handoff tests batch-create and delete six colliders, then
  register the same live anchor and ignored collider. Compact and churned
  contexts retain identical replay order while allocator IDs differ; hashes now
  match in both 2D-to-3D and 3D-to-2D directions and fail under the old ID
  policy.
- Focused replay suites pass 48/48 and both body replay-hash files report 100%
  line, branch, and method coverage.
- Authoritative artifact
  `TestResults/coverage-body-replay-task76-final-authoritative-root-comparable/89757f3d-f55c-41d5-998b-e1d4f97f8d20/coverage.cobertura.xml`
  passes 2,549/2,549 full `Release` tests.

### Mesh-Cone Triangle Containment Used Contact-Oriented Normals

**Resolved:** 2026-07-13  
**Source:** 95%-to-100% coverage hardening, mesh-cone branch review  
**Affected area:** `CollisionDetection.TryFindMeshConeTriangleContact(...)`

RCA: mesh-cone triangle detection oriented each face normal toward the cone
before passing it to `MeshUtils.ClosestPointOnTriangle(...)` and
`IsPointInTrianglePlane(...)`. Those helpers classify edge half-spaces against
the triangle's authored winding. Flipping the normal for a cone approaching the
back face reversed every containment test and could reject a real crossing. The
same contact normal was oriented from the collider's mesh-bounds center, so
disconnected or strongly offset geometry could also face a valid contact in the
wrong direction and publish the wrong support point and depth.

Fix: retain the world winding normal for every triangle projection and
containment operation. Derive a separate mesh-to-cone contact normal from the
cone center and the candidate point on that triangle, then use only that normal
for support half-space distance and manifold state.

Verification:

- A mirrored concave triangle now detects a back-face cone crossing and kills
  any reuse of the oriented normal for winding containment.
- A disconnected mesh moves the collider center away from the contacted
  triangle; reverting orientation to the whole mesh center flips the pinned
  normal, support point, and depth.
- An oblique concave near miss kills removal of the retained plane-separation
  guard. A separate exact positive-`Epsilon` signed gap kills `>` to `>=` while
  confirming fixed-point distance quantizes its published depth to zero.
- Authoritative artifact
  `TestResults/coverage-cone-task72-final2-authoritative-root-comparable/5a77e663-470f-4ad0-89c8-df09249a72f0/coverage.cobertura.xml`
  reports 100% line, branch, and method coverage for
  `CollisionDetection.Cone.cs`; full `Release` passes 2,547/2,547 tests.

### Small CCD Proxy Radii Could Turn Tangency Into A Closing Hit

**Resolved:** 2026-07-13  
**Source:** 95%-to-100% coverage hardening, shared relative-sweep review  
**Affected area:** `ContinuousCollisionMath` relative sphere and circle sweeps

RCA: impact-normal selection compared the squared impact separation with the
linear `Fixed64.Epsilon` threshold. Small, valid proxy radii could therefore
produce a nonzero tangential impact delta whose square quantized below the
threshold. The sweep selected the negated motion direction instead of the
geometric normal, computed a positive closing speed, and reported a false CCD
hit. Testing only `MagnitudeSquared > 0` was also insufficient because the
square itself can quantize to zero.

Fix: every exact-nonzero 2D/3D impact delta is scaled by its largest absolute
component before normalization. The scaled magnitude remains representable,
while an exact-zero delta alone retains the motion-direction fallback.

Verification:

- An exact fixed-point witness produces closing speed equal to `Fixed64.Epsilon`
  and proves the retained `<=` rejection boundary.
- Symmetric 2D/3D tangency regressions use radii above the admission epsilon
  whose combined-radius square is exactly zero. Restoring the former fallback
  independently fails each dimensional assertion.
- Authoritative artifact
  `TestResults/coverage-continuous-math-task71-final-authoritative-root-comparable/74e3f071-bbf8-493e-9c2e-a2284311cf13/coverage.cobertura.xml`
  reports 100% line, branch, and method coverage for `ContinuousCollisionMath`;
  full `Release` passes 2,543/2,543 tests.

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
prevalidated before standalone registration, before compound part rebuilds, and
before runtime cache mutation. Scaled bounds, face normals/areas, projected
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
  collision, BVH reuse, support-tree, standalone/compound lifecycle, and checked
  underflow/saturation regressions.
- Added a combined off-center compound regression with nonuniform owner and part
  scale, part rotation, owner-local COM, and arbitrary-reference inertia.
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
positive-weight part when one exists, otherwise to the last authored part. Part
COM points include owner offset and authored local rotation. Center tensors are
rotated by `R*I*R^T`, clamped near zero, and then shifted through the
parallel-axis theorem. Private parts inherit owner `Center`; radius encloses the
farthest aggregate-bounds corner about that center; the false `ScaledSize`
override was removed.

Verification: regressions cover analytic sphere volume weighting and tensor,
rotated anisotropic cuboid inertia, rotated owner/part offsets, off-center
closed mesh COM, every primitive and mesh mass measure, explicit shell policy,
invalid closed-volume topology, exact residual assignment including trailing
zero-weight parts, all-zero fixed-point fallback, remote-part public query
visibility, authored-first geometry ties, capsule frontal aggregation, and
degenerate cone projection. All touched executable production types report 100%
line, branch, and method coverage; full coverage-enabled `Release` passes
2,433/2,433, `ReleaseLean` passes 2,396/2,396, both library targets build
without warnings, and independent review approved.

### 3D Motion State Could Leak Across Reuse And Apply Incorrect Rotational Dynamics

**Discovered:** 2026-07-12  
**Resolved:** 2026-07-12  
**Source:** 95%-to-100% coverage hardening, `SolidBody.Motion` review  
**Affected area:** 3D body reset/reuse, grounded angular friction, and
gyroscopic precession

RCA: `Initialize(...)` and `ResetPosition(...)` cleared visible velocities but
left queued force/torque and cached angular acceleration state intact. Grounded
angular friction wrote to the linear acceleration store after linear
integration, so the next frame overwrote it without slowing rotation. Gyroscopic
precession also added the Euler correction instead of subtracting `I^-1(w x Iw)`
and changed angular velocity after speed, direction, and acceleration had
already been cached. The first cache fix measured only the precession delta,
omitting torque acceleration applied earlier in the same fixed step. Queued CCD
handoff consumption then applied another full-frame gyroscopic correction after
the normal angular step even though the handoff changed only linear state.

Fix: both reset paths now use the shared complete motion clear. Angular friction
accumulates in the angular store. Gyroscopic precession applies the negative
Euler term and refreshes angular motion state from the fixed-step starting
velocity after the correction. Non-torque impulse paths use their own pre-gyro
velocity as that refresh baseline. Linear-only queued handoffs no longer rebuild
unchanged inertia orientation or run gyroscopic integration a second time. The
unused planar `AddPositionCorrection(Vector3d)` API and its serialized and
replay-hashed load-only state were deleted; collision response already uses the
full 3D immediate correction path.

Verification: RED regressions reproduced deferred motion after shell reuse and
reset, unchanged grounded angular speed, and the wrong-sign off-principal
anisotropic rotation. GREEN coverage proves exact reset poses, fresh/reused body
replay-hash equality, repeat-run gyro determinism, correct correction sign
against a final-orientation world-tensor reconstruction, and coherent angular
velocity, speed, and total torque-plus-gyro acceleration. JSON snapshots also
exclude the removed stale correction state. A service-phase CCD regression
proves queued linear handoff processing preserves angular velocity, speed,
acceleration, and rotation exactly after the normal body step.

### Synchronous 2D Contact Callbacks Could Corrupt Pair Teardown And Reuse

**Discovered:** 2026-07-12  
**Resolved:** 2026-07-12  
**Source:** 95%-to-100% coverage hardening, 2D pair/response lifecycle review  
**Affected area:** `GravitasPhysics2DService` response expansion, pair cleanup,
deactivation, and pooling

RCA: 2D contact enter/exit callbacks run synchronously while the service is
walking pair registries. A callback that deactivated a collider could mutate the
active `SwiftDictionary` enumerator and throw. An enter callback could also
remove and recycle the current pair before `ProcessCandidate(...)` appended its
local reference, allowing that object to be reused by a later collision and
solved twice.

Fix: existing response edges are snapshotted into a pre-sized service buffer
before callbacks. Pair cleanup snapshots stable keys, and direct teardown stages
nested separation ranges, removes registry ownership before notifying, and
recycles each still-current pair once. Response append paths revalidate
registered pair identity and physical eligibility after notification.

Verification: deterministic regressions cover current and later snapshotted pair
removal during expansion, stale queued bodies and rootless response rows,
cleanup removal of a later key, nested multi-pair deactivation with exact exit
counts, distinct pooled replacements, and pooled/unpooled position equality
after enter-callback removal.

### Fixed-Point Sphere Tangency Could Be Rejected By Normalization Residue

**Discovered:** 2026-07-12  
**Resolved:** 2026-07-12  
**Source:** 95%-to-100% coverage hardening, 3D raycast segment review  
**Affected area:** `RaycastSegmentWorker.CheckSphereOverlaps(...)`

Historical RCA: the former closest-point and normalized-direction quadratic
disagreed by one raw unit for a near-tangent fixture, so the discriminant was
clamped to zero. Full-domain radial hardening later proved that the stored
Q32.32 values in that fixture are an exact one-raw-unit miss, not a tangent;
the clamp therefore invented a contact.

Superseding fix: sphere segments now use the authored segment over `[0, 1]`
and FixedMathSharp's exact bounded interval solver. Exact stored-value tangency
still returns one hit, while the historical `(0,0,0)->(3,4,0)` fixture against
center `(1/5,3/10,0)` and radius `1/50` is retained as an explicit near-miss
regression. No epsilon or discriminant clamp remains.

### Context Disposal Ordering Could Admit Inactive Worlds And Invalidate Disabled CCD Handoffs

**Discovered:** 2026-07-11  
**Resolved:** 2026-07-11  
**Source:** 95%-to-100% coverage hardening, world-context lifecycle review  
**Affected area:** `GravitasWorldContext` world registration/disposal and
disabled-service late-simulate CCD state

RCA: `Attach(...)` validated `GridWorld.IsActive` before taking the ownership
lock, while owned-context disposal removed its registry entry before
`GridWorld.Dispose()`. A waiting or reset-handler attach could therefore bind an
inactive or disposal-in-progress world. Separately, context late simulation
advanced the CCD frame token even when both enabled dimensional physics services
were disabled, making their untouched pending handoffs stale.

Fix: world activity validation, registration, owned-world disposal, and entry
release are now serialized under the ownership lock, with the entry retained
through world disposal. The context advances its CCD token only when at least
one dimensional physics service runs.

Verification: a public owned-world reset regression proves reentrant attach is
rejected until disposal completes. A disabled `Both`-mode regression seeds
pending 3D and 2D handoffs, advances the public context clock and hook phase,
and proves both body states remain unchanged and both handoffs remain consumable
afterward.

### Partition Teardown Logged Errors After Host Grid Removal

**Discovered:** 2026-07-11  
**Resolved:** 2026-07-11  
**Source:** 95%-to-100% coverage hardening, dimensional partition-service
review  
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

Fix: both dimensional entry points reject registered colliders and foreign host
bindings before mutating collider state. A context-reset shell may be explicitly
reinitialized only through the same agent binding; full deactivation clears the
binding for general reuse.

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

Fix: the cuboid now computes the exact orthographic box projection as the sum of
each face area multiplied by the absolute dot product between its local axis and
the normalized world direction. The epsilon zero-direction fallback matches the
existing cylinder/cone contract. The same block removed unused centroid and
copied topology caches, duplicate cuboid-state policy, dead public edge helpers
and build hooks, and public mutable-array exposure; collision and query
consumers retain internal access to live geometry.

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
line/branch/method coverage; full coverage-enabled `Release` passes 2,422/2,422,
`ReleaseLean` builds both targets without warnings, and independent review
approved.

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
after shape/transform restoration while bindings are valid. Snapshot activity is
accepted only for an already registered shell; an unregistered shell remains
inactive until its host explicitly calls `Initialize()`.

Verification:

- Added JSON and MemoryPack transitions covering registered active to inactive,
  immediate registry/partition cleanup, repeated teardown, attempted active load
  into the unregistered shell, and explicit reinitialization.
- Verified registered active snapshot loads retain their existing continuation
  contract.
- `SolidBody2D.cs` and `SolidBody2D.Serialization.cs` report 100%
  line/branch/method coverage, full `Release` passes 2,106/2,106, `ReleaseLean`
  builds both targets, and independent review approved.

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
active-state lifecycle. Deactivated colliders could therefore remain in primary
and mixed partitions and continue appearing in spatial queries. The load path
had related ownership holes: an inactive payload cleared membership, but a later
active payload skipped primary repartitioning; an unbound shell attempted shape
rebuild before its context guard; and an active payload applied to a fully
deactivated shell attempted to partition the unregistered ID `-1`. Repeated
inactive loads also called primary partition cleanup after membership was
already gone, emitting a false invariant error.

Fix: 3D collider activation is now an explicit `IsActive` lifecycle property.
Registered deactivation clears primary and mixed membership, while reactivation
rebuilds primary membership and refreshes mixed membership when enabled. Loading
now defers unbound rebuilds, skips partition ownership for unregistered shells,
restores primary membership for registered inactive-to-active loads, and guards
idempotent primary/mixed cleanup by the corresponding partition flags.

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
same practical defect: it rewound service versions to zero while live colliders
retained their old stamps. The next query could therefore reuse a collider's
cached version and reject it as already visited, producing a false negative.

Fix: raycast and circle version invalidation now scan the context's compact
live-collider registry. Public reset clears both cache families before rewinding
the service counters, while each rollover path clears only its own cache family
before advancing to version one. The scan is allocation-free and the rollover
cost occurs only once per full 32-bit version cycle.

Verification:

- Added failing raycast and circle rollover regressions with both the service
  and live collider seeded at the colliding version-one stamp.
- Added a failing standalone public-reset regression proving both ray and circle
  queries still find the same live collider after reset.
- Focused 3D query suites pass 158/158, the complete raycast source reports 100%
  line/branch coverage, full `Release` passes 2,085/2,085, and independent
  review approved the registry scan and reset lifecycle.

### CCD Rejected Finite Heavy-Body Response As Zero Inverse Mass

**Discovered:** 2026-07-10  
**Resolved:** 2026-07-10  
**Source:** 95%-to-100% coverage hardening, dynamic TOI loop review  
**Affected area:** 2D, 3D, and mixed dynamic/kinematic CCD impulse response

RCA: CCD response treated a positive combined inverse mass less than or equal to
`Fixed64.Epsilon` as immovable. For supported finite masses whose inverse mass
is still representable, this rejected the pair impulse and fell back to removing
only the source body's closing velocity. A target-driven zero-time hit could
therefore freeze at impact with unresolved relative motion, while a stagnation
guard merely prevented the same hit from consuming the full TOI budget. Simply
accepting the smaller inverse mass also exposed a second fixed- point hazard:
computing the shared impulse scalar before multiplying by each body's inverse
mass could saturate even when both final velocity deltas were representable.

Fix: equivalent 2D, 3D, mixed, and kinematic CCD response paths now reject only
nonpositive combined inverse mass and calculate per-body velocity deltas from
inverse-mass ratios before applying response speed, avoiding a saturated shared
impulse intermediate. CCD rejects near-singular constrained-axis mobility when
the constrained-to-raw inverse-mass ratio is at or below epsilon, which keeps
the ratio calculation within fixed-point resolution without rejecting fully
mobile heavy bodies. Per-body deltas scale the normal by the bounded inverse-
mass ratio before response speed so oblique components remain representable. The
2D and 3D stagnation guards remain because zero-planar mixed hits and
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
before clearing service ownership, including entries left after a partial drain.
The 2D and 3D body deactivation paths also discard pending handoffs before
deregistration, so both services use the same explicit lifecycle.

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
mixed relay, making the mixed-frame budget less explicit than the public setting
implied.

Fix: the pure physics services now expose an internal begin/complete late-step
split. Direct service calls remain self-contained, while
`GravitasWorldContext.LateSimulate()` integrates 3D and 2D bodies first, drains
the shared queued CCD handoff budget once at the context level, then completes
partitioning, discrete response, active-pair processing, and sleep updates for
the services that actually ran.

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
face normals plus cross products of cached SAT mesh edges. `PhysicsMesh` builds
the convex SAT edge cache once at construction time, skipping coplanar
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
**Source:** Coverage Workstream 1 zombie-code sweep and subagent geometry
review  
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
**Affected area:** `RaycastSegmentWorker.CheckOBBoxOverlaps(...)`, 3D raycast
queries against rotated `LSCuboidCollider`

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

RCA: 3D direct-collider serialization correctly wrote and loaded `Active=false`,
but the inactive load branch only removed the collider from the partition
services. It did not mark the collider's own primary/mixed partition state
unpartitioned or clear cached coordinates. The matching 2D path already cleared
service membership and collider-local partition state, so 3D could remain
inactive while still reporting stale partition membership.

Fix: `LSCollider.ApplyLoadedState()` now clears collider-local primary and mixed
partition state after loading inactive collider state.

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
