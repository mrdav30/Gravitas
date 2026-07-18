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
- FixedMathSharp foundation hardening is complete and committed. Its final
  artifact reports 100% line, branch, and method coverage, with 1,406 standard
  and 1,385 Lean tests passing. Release FixedMathSharp, then restore its package
  references and validate/release SwiftCollections.
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
   [CCD Handoff Dedupe Can Strand A Same-Frame Requeued Body](#ccd-handoff-dedupe-can-strand-a-same-frame-requeued-body).
2. **Gravitas:**
   [SolidBody Point Transforms Use Collider Dimensions As Transform Scale](#solidbody-point-transforms-use-collider-dimensions-as-transform-scale).
3. **Gravitas:**
   [3D Angular Impulse Scales Immediate Velocity By Frame Delta](#3d-angular-impulse-scales-immediate-velocity-by-frame-delta).
4. **Gravitas:**
   [Rotational CCD Can Miss Contacts Between Bounded Pose Samples](#rotational-ccd-can-miss-contacts-between-bounded-pose-samples).
5. **Gravitas:**
   [Convex Mesh Mode Accepts Disconnected Topology And Can Collide In Empty Bounds Space](#convex-mesh-mode-accepts-disconnected-topology-and-can-collide-in-empty-bounds-space).
6. **Gravitas:**
   [Relative CCD Quadratic Saturation Can Miss Extreme-Range Crossings](#relative-ccd-quadratic-saturation-can-miss-extreme-range-crossings).
   Reuse the magnitude and normalization policy established by the resolved
   extreme-convex-sweep work when adding the separate scale-safe quadratic
   implementation.

### Relative CCD Quadratic Saturation Can Miss Extreme-Range Crossings

**Discovered:** 2026-07-13  
**Source:** 95%-to-100% coverage hardening, shared relative-sweep review  
**Affected area:** 2D, 3D, and mixed relative continuous-collision sweeps

The relative sphere/circle sweep evaluates its quadratic directly in Q32.32. At
separations or per-frame displacements above roughly `46,340.95`, squared terms
can saturate before the discriminant and root are formed. A crossing can then
collapse to an endpoint candidate with the wrong impact normal and be rejected
even though the swept broad phase admitted it.

Resolve this with a scale-safe quadratic formulation or an explicit validated
world/displacement range contract. Add symmetric 2D/3D regressions for large
crossings, endpoint-adjacent hits, misses, and mixed CCD routing at the chosen
boundary. This is distinct from conservative proxy-radius saturation and does
not block coverage convergence because ordinary supported ranges and the current
uncovered closing-speed boundary remain independently testable.

### 3D Angular Impulse Scales Immediate Velocity By Frame Delta

**Discovered:** 2026-07-12  
**Source:** 95%-to-100% coverage hardening, 2D/3D motion parity review  
**Affected area:** `SolidBody.AddAngularImpulse(...)` units and frame-rate
invariance

The 3D angular-impulse API multiplies the supplied impulse by both inverse
inertia and `GravitasWorldContext.DeltaTime` before changing angular velocity.
Consequently, otherwise identical bodies receive different immediate angular
velocity changes from the same impulse when their context frame rates differ.
The 2D angular-impulse API and the usual physical impulse contract apply inverse
inertia without a time-step factor. The adjacent 3D linear-impulse API uses the
same time-step pattern and should be audited as part of the semantic decision.

Resolve this as an explicit breaking API/units decision rather than removing the
factor incidentally. Add cross-frame-rate regressions for immediate linear and
angular impulse response, update XML/wiki unit documentation, and verify
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

### SolidBody Point Transforms Use Collider Dimensions As Transform Scale

**Discovered:** 2026-07-12  
**Source:** 95%-to-100% coverage hardening, 3D compound `ScaledSize` review  
**Affected area:** `SolidBody.TransformPoint(...)`,
`SolidBody.InverseTransformPoint(...)`, and collider size semantics

The generic point-transform helpers multiply and divide by
`Collider.ScaledSize`. For primitive colliders that value includes authored
shape dimensions as well as host scale, so a local point is incorrectly scaled
by the collider's geometry. The old compound override made the mismatch worse by
returning a world-axis-aligned bounds size and then rotating it again.

The compound override has been removed rather than preserving a false local size
contract. Resolve the generic helpers against the host transform's actual scale,
define zero-scale inverse behavior, and add primitive/compound round-trip tests
across non-unit authored sizes, nonuniform host scale, and rotation.

### Convex Mesh Mode Accepts Disconnected Topology And Can Collide In Empty Bounds Space

**Discovered:** 2026-07-11  
**Source:** 95%-to-100% coverage hardening, mesh/sphere fallback review  
**Affected area:** `PhysicsMesh` validation, `MeshColliderMode.Convex`, and
convex mesh/sphere closest-surface fallback

`PhysicsMesh.ValidateInput(...)` validates counts, indices, and nondegenerate
triangles but does not validate the topology promised by
`MeshColliderMode.Convex`. Disconnected triangles can therefore be accepted as a
convex mesh. Near an empty corner of their combined AABB, the local triangle
query can return no candidates and the convex mesh/sphere path falls back to the
AABB surface, producing a contact where no authored triangle exists.

Open convex surfaces are supported intentionally, so requiring every convex mesh
to be a closed volume would reject valid floors and other planar assets. Resolve
this with an explicit semantic decision: either validate a documented
connected/convex open-or-closed topology contract, or change the empty-query
fallback so invalid topology cannot create an empty-space contact. Add a
regression that preserves valid open convex surfaces while rejecting or safely
handling disconnected input without the AABB false-positive.

### CCD Handoff Dedupe Can Strand A Same-Frame Requeued Body

**Discovered:** 2026-07-13  
**Source:** 95%-to-100% coverage hardening, dimensional CCD service admission
review  
**Affected area:** 2D/3D continuous-collision handoff queues, same-frame relay
cycles, mixed CCD routing, iteration-budget ownership, and replay continuity

The handoff drain leaves each dequeued body in
`_queuedContinuousCollisionHandoffBodies` until the entire queue is cleared. If
body A is consumed and a later same-frame relay applies another handoff to A,
`ApplyContinuousCollisionHandoff` marks A pending but queue admission rejects
the re-enqueue because A is still present in the dedupe set. End-of-drain queue
cleanup then erases service ownership without discarding A's new pending state;
the next late-simulate token makes that state permanently stale. The same queue
shape exists in 2D and 3D and can participate in pure or mixed relay cycles.

Resolve this as a focused queue-ownership change. Remove a body from the dedupe
set immediately before consuming its queue entry so a later same-frame relay can
re-enqueue it under the existing deterministic iteration budget, while retaining
dedupe for repeated updates before dequeue. Add symmetric 2D/3D state-machine
regressions (and a mixed relay witness if practical) where A is queued,
consumed, receives a second handoff before drain completion, and is either
consumed again within budget or explicitly discarded at the cap. Assert no
directly consumable or replay-visible pending handoff remains after drain. This
does not block coverage convergence and is independent of the redundant
active/dynamic-ID admission predicates removed in Task 67.

## Resolved Issues

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

RCA: the closest-point test could prove a segment touched a sphere while the
subsequent quadratic discriminant evaluated to one negative raw fixed-point unit
because the normalized segment direction was fractionally longer than one. The
quadratic guard then rejected the exact tangent.

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
