# Static Surface Continuity And Heightfield Integration

**Date:** 2026-10-15  
**Status:** Phases 1–11 complete, 2026-10-28
#133/#139 remain coordinated. Heightfield and alternative-index work are deferred;
no public continuity API is enabled. Internal 3D discrete response covers rounded,
cone, extrusion and native mesh families, pure 2D joined-solid response,
embedded 2D movers against joined 3D meshes, and native 3D boxes, closed convex
hulls, spheres, capsules, cylinders and cones against joined 2D solids.
Remaining reverse mixed families and continuation work remain in the phases
below.  
**Order:** #132 is resolved. Resolve #133 and its confirmed pure 2D counterpart,
#139, through the generic-continuity phases below before terrain-specific work.
Keep one implementation plan rather than duplicate the prerequisite design.  
**Goal:** Let hosts describe a continuous static environment using bounded,
spatially indexed pieces, while evaluating reuse of existing deterministic
height data for physics and navigation.

## Agreed Direction

- Physics owns geometry, contact boundaries and response; hosts decide whether
  a surface represents terrain, roads, floors or another environment feature.
- Distinguish a logical continuous surface from its storage/streaming pieces.
  Explicit membership expresses intended continuity; geometric validation must
  establish which borders actually join. Membership alone cannot erase edges.
- Members retain bounded geometry, collider identity and spatial membership.
  They do not become one rigid body or compound with a common mass, inertia,
  center of mass or enormous attachment/proxy radius.
- Evaluate GridForge-backed candidate gathering and a member-level BVH against
  the same workloads, reusing current per-mesh BVHs and topology/contact logic.
  Choose index ownership from evidence; avoid redundant authoritative indexes
  and work that scans every member per frame.
- Heightfields are a separate, optional representation. Establish correct mesh
  continuity before deciding whether a specialized backend improves our needs.

## Current Evidence And Ownership

[#132](issue-tracker.md#grv-issue-132---bodyless-3d-static-geometry-does-not-receive-discrete-blocking-response)
resolved missing bodyless 3D discrete response using the existing solver and
zero-mobility endpoints. Bodyless and explicit-static support now agree.
[#133](issue-tracker.md#grv-issue-133---adjacent-mesh-collider-rims-alter-frictionless-seam-crossing-response)
owns the artificial-rim response difference across separate mesh colliders.
The [supported-geometry closeout](done/2026-10-12-supported-geometry-range-plan.md#phase-4-compatibility-performance-and-096-closeout)
retains the continuous/chunked comparison, repeated original/final traces and
bounded cost controls. It does not establish physically equivalent chunking.

Local Trailblazer source inspected for this proposal:

- `src/Trailblazer/Heightmaps/HeightmapSurface.cs`: immutable short-backed X/Z
  samples, reference-height/step decompression and bilinear ground-Y sampling.
- `HeightmapCompression.cs`: explicit quantization; `FromHeights` clamps to the
  chosen short range. A physics import must account for this authored loss.
- `TrailblazerHeightmapService.cs`: context-owned layer registration and stable
  height-layer selection, not rigid-body collision generation.
- `Navigation/Navigator/Navigator.HeightmapGrounding.cs`: opt-in ground contact
  using `Vector3d.Up`, with optional position projection without velocity change.
  The helper assumes the caller has already established solid-ground traversal.

A physics heightfield is actual collision geometry backed by height samples.
It needs finite-shape intersection, contact witnesses/normals, response, queries
and CCD. Point-height sampling alone does not provide those semantics.
Trailblazer's existing data/quantization is an optional reuse candidate, not an
existing Gravitas collider. Keep both core libraries independent.

`src/Trailblazer.Gravitas` already provides the optional integration boundary.
Its `GravitasNavigator3D` prepares requested movement, commits the pose accepted
by Gravitas, and translates physical support into navigation ground state.
Inspect/reuse that adapter before adding integration behavior. Additional
physics-backed ground sampling or alignment policies belong there when needed,
not in core Gravitas or core Trailblazer. Prefer host adapters/shared authored
input before moving types or introducing a shared package. GridForge owns
spatial organization, FixedMathSharp neutral math, Gravitas physics, and
Trailblazer navigation policy.

## Independent Usage Profiles

These are documentation-level host choices, not a shared runtime mode setting:

- **Height-driven navigation:** Trailblazer samples ground height for agents
  that do not require physical contacts with that surface. No Gravitas terrain
  collider or specialized physics heightfield is required for this use.
- **Physics-backed environment:** Gravitas mesh surfaces provide collision and
  response. Neither Trailblazer heightmaps nor a new heightfield backend are
  prerequisites. `Trailblazer.Gravitas` can connect navigation to accepted
  physical movement/support when a host needs both libraries.
- **Optional mixed population:** A host may use height-driven agents alongside
  physics-driven vehicles, props or projectiles. Define consistent ground data
  only where actors need agreement; do not apply competing height projection
  and physics pose resolution to the same degrees of freedom.

## Surface Agreement When Profiles Share Ground

The same sample grid can describe different interior geometry. Trailblazer uses
bilinear interpolation; a triangle-cell backend describes a piecewise planar
surface. For unit-cell heights `h00=h10=h01=0`, `h11=1`, the center height is
`1/4` bilinearly, but `0` or `1/2` with the two possible triangle diagonals.

Only when both representations must describe the same ground, choose a
canonical physical surface and define navigation agreement: exact shared
evaluation or an intentional, measured approximation. Independent usage does
not require cross-library sampling agreement or new integration machinery.
Decide cell diagonals, normal semantics, hole/layer selection, quantization,
coordinates and rounding. Preserve existing sampling behavior unless an
intentional compatibility change is justified. A single height per X/Z location
cannot represent overhangs; arbitrary meshes and layered environments remain
separate supported cases. Avoid promising heightfield/mesh equivalence without
matching actual surface definitions.

## Candidate Member Lookup Designs

Each `PhysicsMesh` already uses `SwiftFixedBVH<int>` for local triangle lookup.
A BVH over member mesh bounds is a valid candidate for the logical surface:
query/body bounds -> relevant mesh members -> existing local triangle BVHs.
Reuse SwiftCollections' fixed-point BVH operations before writing index mechanics.

Compare the existing grid-backed route with this two-level hierarchy for sparse,
dense and irregular layouts, large member counts, and streaming/edit churn.
Decide whether a member BVH augments or replaces the relevant static-candidate
route; retaining two indexes that do the same job is not a requirement. Include
lookup time, update/refit/rebuild cost, memory, candidate counts and tail latency.
Keep stable member/feature keys and deterministic final processing order rather
than relying on tree traversal order. Audit large-world internal-node bounds
without treating an index node as a rigid collider or computing a world-sized
body radius.

A BVH answers spatial questions; overlapping or neighboring bounds do not prove
that two edges form a continuous surface. Validated border adjacency and surface
membership still supply that meaning to contact generation/reduction. Store
that information alongside the selected index/owners without presupposing a
second topology tree or a new broad-phase framework.

## Generic Continuity Contract Proposed For #133/#139

The prerequisite is substantial: contact generation currently sees each member's
finite boundary independently. Removing sideways normals in response would
leave artificial face samples and torque, and would not fix mixed CCD. Advance
this generic portion of the plan rather than implement a temporary seam filter
and replace it during terrain work. The user approved resolving #133/#139
together through this plan; Phase 1 settled the API naming, contact/recovery
contract and bounded arithmetic feasibility recorded below.

- **Explicit membership:** Prefer a context-owned static-surface handle with
  creation, atomic member replacement and disposal. Separate typed 3D mesh and
  2D surface membership. Mixed contacts need both directions checked: 2D movers
  against 3D surfaces and 3D movers against embedded 2D pieces. Host Y and slab
  half-thickness must preserve real steps rather than assume planar continuity
  proves identical embedded geometry. Use existing dimensional physics-service
  ownership rather than add a second world registry.
  Use `context.Physics.CreateStaticSurface(ReadOnlySpan<LSMeshCollider>)`
  returning `StaticSurface`, and
  `context.Physics2D.CreateStaticSurface(ReadOnlySpan<LSCollider2D>)`
  returning `StaticSurface2D`. Each handle has typed `ReplaceMembers(...)` and
  `Dispose()` operations; no additional registry service or common body wrapper
  is needed. These are planned public API names; exposure follows the complete supported contract.
  One membership per collider initially; empty membership may retain a handle
  while its host streams pieces out. Disposal ends that handle's lifetime.
  Membership permits geometric matching within that set; it does not assert
  that all pieces touch. Disconnected members retain their genuine boundaries.
- **Eligible pieces:** Bodyless and explicit-static colliders share the contract.
  Independent kinematic/dynamic bodies remain independent. Start 3D admission
  with concave mesh surface geometry and 2D with AABB/convex polygon pieces;
  3D pieces form finite sheets; 2D pieces form a filled planar union whose
  interior shared edges cease being perimeter. Do not reinterpret closed convex
  volumes as thin sheets or invent a 2D mesh type. Audit compound leaves before
  deciding whether they can join: do not silently merge a compound's own internal
  boundaries as a side effect.
- **Exact joins:** Validate actual committed boundary spans, including unequal
  subdivision and partial edge coverage. Only a nonzero geometrically covered
  span can cease being a boundary. Point-only touches, gaps, overlapping fills
  and ambiguous/nonmanifold joins do not authorize erasing rims. Preserve real
  creases at folded joins rather than smoothing normals. No proximity welding,
  implicit snapping or tolerance based on rounded world vertices. Exactness is
  relative to committed scaled-local geometry and stored rigid transforms,
  before world-point materialization; visually equal rounded endpoints can
  still describe a real gap under that metric.
- **Local frames:** Retain each member's origin, rotation, admitted local data,
  partitions and existing triangle BVH. Support independently transformed pieces
  without a common-origin/rotation restriction. Do not build a giant combined
  `PhysicsMesh`, narrow absolute-world differences or round neighboring vertices
  into one chart before proving a join. A logical surface has no aggregate body
  reach, mass or inertia. Its per-query work must follow local candidates and
  admitted connectivity, not scan every member of a world-sized set.
- **Physical domain:** Join finite-shape-admitted regions across validated
  neighbors before selecting face samples and true-perimeter constraints.
  Connectivity alone does not certify a whole-shape exit. Coverage must respect
  the moving participant's filters, trigger policy and live neighbor state.
  An excluded or unloaded piece cannot hide an eligible member's rim.
- **Contact and recovery contract:** Generate independent genuine-boundary
  normal/feature groups. Local penetration stabilization is not a promise of a
  one-step whole-union exit or global minimum translation. The existing
  two-contact planar manifold limit is not a union-wide reduction limit; retain
  distinct groups through pair/solver ownership without losing independent
  normals, finite endpoints or materials to depth/ID ranking.
  A recovery candidate is certified only after checking its final representable
  pose against all eligible geometry in that pose's complete spatial query,
  including members absent from the original overlap set. This finite query
  bounds the certificate's domain; neither a fixed iteration budget nor a
  subset of initial overlaps proves clearance. Endpoint clearance alone does
  not promise an unobstructed path to it.
  Failure to certify means only **no certified exit**, not no collision or
  proof that no exit exists. Preserve geometric overlap, events, and valid
  contact groups; initially overlapping sweeps remain distance zero. Do not
  fall back to artificial internal-edge constraints.
  If a response-required movable, nontrigger overlap has neither genuine
  boundary rows nor a certified recovery, its dimensional collision/physics
  owner must raise a deterministic `InvalidOperationException` identifying
  the unsupported deep placement. A disabled diagnostic logger is insufficient.
  Trigger, query-only and static/static observations retain their geometric
  results without requiring a response manifold. A failed fixed step is not a
  completed step; no rollback or continuation guarantee is implied. Hosts can
  restore/reinitialize with valid placement before stepping again; correcting
  one pose alone does not repair an already partially executed step.
- **Identity and materials:** Keep each collider, pair, query hit and geometric
  overlap event distinct. Deduplicate physical constraints where appropriate
  without manufacturing contact exits for an overlapping member. Every retained
  witness keeps its supplying member's material and feature provenance; material
  discontinuities alone are not holes. Define deterministic ownership at an
  exactly shared feature before publishing reduction logic.
- **Transactions and lifetime:** Change membership only between full fixed
  steps. Rejected replacement preserves the previous usable state. Bind members
  to collider lifetimes; deactivation removes membership, reactivation needs an
  explicit declaration, and context reset/disposal invalidates its handles.
  Geometry edits publish the affected join changes coherently: moving a member
  can open a real gap. Invalidate affected neighbors' contact, support, warm-start
  and CCD dependencies even when their own geometry did not change. Rebuild
  locally rather than rebake every member after an edit.
- **Replay and restore:** Membership is authoritative; adjacency is derived.
  Include membership in replay hashes using stable identities, not addresses or
  recycled collider IDs. Define explicit recording/reconstruction for host-created
  handles and member lifetimes, following existing Chronicler patterns. Test
  populate-existing and continuation; a disposable handle is not itself a saved
  authoring identity. Keep derived caches out of the authored representation.

## Generic Continuity Implementation Sequence

Every runtime phase must preserve raw reachable 100% line/branch/method coverage
in Release/Lean, relevant replay behavior, zero added warmed allocations and the
ordinary warm baseline, using `UseLocalLsfStack=true`. Refine introduced costs
before handoff and obtain independent ponytail review. Phase 14 consolidates the
evidence; it does not postpone these requirements. Keep experimental membership
internal until response, query/CCD and restore semantics are coherent.

### Investigation - Complete, 2026-10-16

On the unchanged committed stack, settle both explicit-static and bodyless
floors for 32 frames, then repeat the original 64-frame sphere/capsule crossing.
All four split controls first lose speed at frame 30: maximum X-speed loss
`0.024248116184026003`, final X `1.9498989908024669` and velocity
`0.975751883815974`. Continuous controls finish at X `2`, velocity `1`.
Each case repeats byte-identically. This updates the earlier bodyless probe,
which did not settle, to exercise #132's corrected support path equally.

The pure 2D counterpart also reproduces: circle/capsule against either adjacent
AABBs or convex polygons loses `0.02416824991814792` speed at frame 30 and
finishes at X `1.9501529859844595`. Continuous controls again finish at X `2`,
velocity `1`; all eight cases repeat identically. Record this as #139, with
shared design review and dimensional implementation rather than a new mesh API.
Mixed runtime probes now confirm the same defect in both directions. Planar
circle/capsule movers crossing a split vertical 3D sheet first lose speed at
frame 30 and stall or cross through the sheet. 3D sphere/capsule movers crossing
adjoining embedded 2D AABB slabs stop at X=-0.5 at frame 23. Continuous controls
finish at X=2 with unit X velocity. Bodyless and explicit-static floors agree.
All 16 cases repeat identically; an independent rerun matches the complete
`mixed-baseline/{results,root-rerun}.json` payload byte-for-byte against the same
eight frozen modules. Reported contact rows are fresh post-step narrow-phase
inspections, not retained solver-manifold captures. Mixed CCD remains a later
implementation gate.

Independent ponytail and dimensional reviews traced face sampling, true-rim
generation, event ownership, filters and neighbor invalidation. They reject a
response-only normal filter. Existing exact rigid projections, anchored points,
coplanar topology and finite-section machinery are the reuse starting point;
their existence does not prove multi-frame reduction needs no math extension.

Ignored `artifacts/grv-issue-133/original-settled-terrain.json` and
`parity/{original,binding,source-hashes}.json` retain fixtures and loaded-module
bindings. The eight-module baseline uses Gravitas `8A7E5C4D` and production inputs
matching committed `fa7da28`; it is the retained tested #138 runtime, not a claim
that binaries were rebuilt at this documentation commit. The frozen baseline and
six terrain/five ordinary warm controls are under `artifacts/grv-issue-133/perf`.
The six terrain baseline guards pass eight repeats each with zero warmed
allocations. No candidate timings, new runtime behavior or coverage result is
claimed here.

### Phase 1 - Prove Bounded Cross-Member Geometry And Ownership - Complete, 2026-10-16

- [x] Review the combined #133/#139 scope: the user approved resolving both
  through this plan, with heightfields and new indexes deferred.
- [x] Supply exact capsule math prerequisites: member-scale depth ordering and
  original-edge rational clipping with reconstructible events. Keep the joined
  domain and final witness-ownership proof as the separate gates below.
- [x] Demonstrate exact boundary coincidence, coplanarity, opposite-side fill,
  partial coverage and shared-corner handling across independent member frames.
  Extend existing `PhysicsMesh` topology and `MeshSurfaceConnectivity` access
  only where needed; retain per-member feature identities. Use actual committed
  collider centering/transforms in the runtime fixtures: an arithmetic proof
  over synthetic frames does not establish chunk-authoring equivalence after
  independently rounded poses or centering.
- [x] Prove face-region reduction across those frames before materialization,
  with explicit width/rounding bounds and a correct material/sample owner.
  Reuse FixedMathSharp's wide rigid projections and anchored geometry. If a
  missing operation is policy-neutral math, add the narrow operation upstream;
  keep connectivity, physics selection and failure policy in Gravitas.
  The complete capsule-region proof and cone/slab clipped-boundary/native-witness
  proofs establish this arithmetic feasibility gate; complete joined family face
  pools are integrated by the owning shape/dimensional phases.
- [x] Settle filled-2D deep-containment recovery before claiming the dimensional
  proof complete. Ordinary contacts retain independent true-boundary normal
  groups; nearest-corner projection alone does not certify a whole-union exit.
- [x] Measure the bounded capsule two-member seam/interior proof against the
  frozen ordinary control and matched complete single-member gathering.
  Refine introduced costs in this phase. Stop and revise the design if exact
  multi-frame reduction requires a large parallel geometry solver, a global
  mesh rebake or a resolved ordinary-path warm regression.

Record a concise phase result here; keep #133/#139 open until their full runtime
and lifecycle gates pass.

**Progress, 2026-10-16:** The maintained bounded proof now retains original
member edges, exact rational clips and member/surface identities. It counts
filled half-neighborhoods, rather than treating pairwise coincidence as trusted
topology. Exactly one filled side per eligible member can join; point contacts,
duplicate fill and folded/nonmanifold incidents retain boundaries. Compatible
intervals coalesce across irrelevant triangulation cuts. Actual committed
collider fixtures cover independent poses, centering, world limits, unequal
subdivision, partial spans, gaps and opposite physical normal conventions.

Pure 2D uses its actual scalar Sin/Cos geometry. Embedded mixed targets use
the actual quaternion prism frames; equal host Y and half-thickness permit a
full-height planar certificate, while differing slabs retain their steps.
A mixed diamond-overlap regression verifies that its internal fan diagonal
does not manufacture separate ambiguous boundary intervals. No 2D mesh API,
combined world mesh or additional spatial index has been introduced.

The joined capsule proof reuses existing finite connectivity, retained algebraic
banks and the plane sample reducer. Input triangles normalize to unique surface
owners and meshes. Finite admission joins local regions through certified
spans; static adjacency alone cannot connect them. Intrinsic plane events are
produced once per joined region and admitted against each member in its native
frame. Event reconstruction retains the producer frame separately from the
supplying owner, so anchors and materials cannot be inherited from the wrong
member. Equal depth/point ties use stable supplying collider/owner identity;
within that owner the existing fixed producer order suffices. Preparation reuse
and opposite-normal fixtures check every sample, including nonseed suppliers.
Representative exits touch the complete supplied geometry and separate one raw
unit farther; anchor distance alone is not the exit assertion.

Only the last supplied local member set retains static boundary preparation,
keyed by collider reference, surface owner, lifetime and committed shape version.
Any member change rebuilds that complete local preparation. Finite admission,
frames, connectivity and samples are recomputed every call; this is not retained
contact-result reuse. Mutation tests compare reused and fresh preparation after
pose, scale, replacement, lifetime, owner, filtering and input-order changes.

An endpoint fan follows only incident positive-length certified joins. An exact
cut inside an original edge uses its interior sector, not an original endpoint's
corner. Point-only neighbors remain separate; ambiguous incidents decline the
certificate. Inward rays are combined in the source chart without rounding cuts.
The contained-section shortcut checks the complete retained perimeter and one
trusted region before using the existing strictly curved support. Full canonical
shared edges with one finite region per side use exact segment admission rather
than generating unrelated ray extrema. Other cases retain the general path.

**Arithmetic and rounding bounds:** Physical plane-ray depth is
`t * sqrt(N·N) / (2D)`; raw member-native depths are not comparable across `D`.
Capsule, cone and finite-slab comparisons retain exact frame scales. Slab compact
parameters are converted with their existing rational/quadratic owners, including
either quadratic branch, repeated roots and conditioning differences. Eliminated
slab quadratics can require fewer than 2,133 coefficient bits; the adaptive
comparison is tested beyond 4,096 discriminant bits. These operations do not
replace the ordinary single-frame comparison path.

Normalized framed point coordinates need fewer than 200 bits, triangle clip
bounds fewer than 268 and sided cross products fewer than 403. Doubled capsule
coordinates/differences stay below 198 bits; original-edge rational bounds below
319 bits keep clipped endpoint numerators below 518, within Signed576. Rational
endpoint incidence needs fewer than 589 bits within Signed832, with clipped-bound
products compared at full width. Common-chart fan rays fit Signed320. Cross-frame
admission and retained-point normalization add at most 132 denominator bits;
three extra limbs preserve the fields. Point deduplication, connectivity, depth
ordering and supplying provenance precede materialization. Coverage is ranked
using exact plane metrics over once-rounded Q32.32 capsule-chart points; each
coordinate has at most half a raw unit of nearest-even rounding. Supplier-relative
anchors retain their own final coordinate rounding, without narrowing absolute
world positions. Exact normals below 262 bits keep projected metrics in Signed832.

**Resolved prerequisite:** FMS-Issue-032 was reproduced on the frozen pre-work
stack: two anchor projection forwarders dropped the retained sub-raw local term.
Both now preserve it through cached-frame projection and final interpolation.
Nine mirrored/world-limit and independently rotated real-support cases include
an exact test-only rounding oracle. FixedMathSharp Release/Lean pass 5,985/5,964
core cases and 49 recording cases each, with raw reachable 100% line, branch and
method coverage and unchanged frozen source hashes. The existing complexity
register records current arithmetic rationale; no separate report is maintained.
Captures are under `artifacts/grv-issue-133/phase1/joined-preliminary`.

**Capsule-checkpoint performance evidence:** The earlier ordinary capsule `+1.967 us` hold was
resolved through one bounded nine-block calibration: byte-identical accepted
A0/A1 graphs reproduce `+1.964 us` with a process-level 95% interval of
`-2.311..+6.238 us`. Candidate versus the within-block accepted mean is
`-3.269 us`, interval `-6.733..+0.194 us`. All 432 actual observations allocate
zero bytes, collect no generations and retain identical physical signatures.
This supports baseline preservation at that resolution, without claiming an
exact zero cost or a speedup. Earlier native BenchmarkDotNet observations remain
unchanged; separate attribution evidence is under
`perf/boundary-domain/calibration`. Prior arithmetic and boundary-preparation
gates remain in `perf/{selection,boundary-domain}/acceptance-final.json`.

New joined budgets distinguish the original pre-admitted-triangle face path
from matched complete owner gathering: their different admission work is not
pure joining overhead. Removing duplicated support/intrinsic work and retaining
bounded static preparation reduced the initial sphere/axial/angled pilots from
about `479/550/2,965 us` to `104/128/1,509 us`. The final native run retains all
144 actual observations across nine cases and two launches per case:

| Geometry | Pre-admitted single sheet | Gathered single sheet | Joined two members |
| --- | ---: | ---: | ---: |
| Sphere | 25.36 us | 62.03 us | 90.70 us |
| Axial capsule | 37.26 us | 79.47 us | 128.52 us |
| Angled capsule | 1,032.70 us | 1,358.98 us | 1,421.56 us |

All warmed cases allocate zero bytes. These are bounded internal-kernel budgets
over the same retained local member set, not frame-capacity claims or precise
scaling guarantees. A fresh face owner over already committed meshes allocates
49,280 bytes for sphere/axial and 68,520 for angled storage; that is distinct from
warmed calls and whole-world startup. Preparation remains bounded by supplied
local candidates, rather than scanning a world-sized membership set. Native
observations, first-owner guards and source/module bindings remain under
`perf/joined-face/final-feature-{budget,all-observations,guard}.json` and the
corresponding native command/log records.

A separate final-source pilot alternates two equivalent two-member sets on one
reused face owner, forcing static preparation to rebuild. Sphere/axial/angled
means are `192.01/212.56/1,580.47 us`, across two fresh processes and 36 retained
phases. All phases and 128-call warmed guards allocate zero bytes and collect no
generations; both sets retain matching physical signatures and current-set
suppliers. These fixed-loop pilot budgets are separate from the native same-set
measurements. `final-churn-{budget,all-observations}.json` retains observations
and exact module bindings. Last-set reuse therefore does not establish arbitrary
world-level contact reuse. Runtime integration must measure changing candidate
neighborhoods and reuse committed static preparation through its membership
lifecycle, without building a world-sized surface or treating cold-owner timings
as a warmed-allocation result.

That checkpoint's ordinary native ABBA gate uses identical ordinary-only host sources
and timed-method IL, with live generated-child bindings to all eight runtime
modules. All 128 actual measurements are retained. Mean changes are capsule
`+0.48%`, cone `+0.71%`, mixed circle `+0.06%` and pure 2D capsule/polygon
`+0.90%`; all warmed allocations are zero and physical outputs match exactly.
Capsule, mixed and 2D paired contrasts reverse sign; cone's two are positive.
All process-paired 95% intervals include both signs. Two paired launch blocks
give wide uncertainty, so this passes the no-resolved-regression gate at the
observed resolution, not a proof of zero overhead. No iteration samples were
treated as independent process replications. Detailed observations, failed
zero-benchmark startups and bindings remain in
`perf/joined-face/final-ordinary-{assessment,all-observations}.json`, each
`final-ordinary-bound-*-command.json` and the corresponding native logs.

**Capsule-checkpoint validation:** Frozen source passes 6,778 Gravitas Release and 6,710 Lean
cases, with raw reachable 100% line, branch and method coverage in both profiles.
All six native replay fixtures agree between Windows x64 Release/Lean and their
shared expected captures. Independent ponytail review of both repositories,
connected owners and final regression additions has no outstanding findings.
`phase1/{coverage-joined-closeout-summary,gaps-joined-closeout}.json` and
`phase1/replay-joined-closeout-comparison.log` retain the local results. Linux,
ARM and released-package validation were not executed here. This validation
does not establish the remaining dimensional contracts below.

**Filled-2D contract proof:** Actual committed AABB and polygon L-unions
agree: a radius-0.5 circle at `(-1,-1)` pushed by nearest-corner distance plus
radius ends near `(0.353553390596,0.353553390596)` and still penetrates both arms
by about `0.146446609404`. Center `(0.5,0.5)` touches both; one raw unit farther
on each axis separates. Ordinary exterior manifolds retain two circle, three
capsule and four polygon rows across two independent real-boundary normal
groups, each at depth 0.25. Preserve both supplier/normal groups; lowest-ID ties
only deduplicate equivalent constraints. Polygon clipping must regenerate the
missing endpoint `(0,0)` between an internal row `(-0.25,0)` and an exposed row
`(0.75,0)`, rather than merely delete the covered row. The repeated frozen
proof and eight-module bindings are under `artifacts/grv-issue-133/l-union-proof`.
A separate pre-existing single/query witness defect was captured as #140; this
proof uses the correct finite manifold path. The subsequent #140 correction
retains the already-computed first finite pair in single-contact consumers too.

The decision is local true-boundary contact groups with separately certified
recovery, following the explicit failure contract above. Global minimum exit
is a different nonconvex configuration-space problem: the exterior can lie
arbitrarily far beyond otherwise identical local tiled neighborhoods. Removing
chunk rims does not require that new global solver, and local response must not
claim its guarantee. Fourteen maintained manifold/query cases now cover the
L-union constraints, exact tangency/separation, a new blocker encountered only
at the proposed final pose, and completely joined interior tiles that still
report overlap and distance-zero sweeps.

The exposed endpoint operation now recovers `(0,0)` from the original edge's
retained rational interval, including supplier-native frames and subraw cuts.
It interpolates before nearest-even materialization, reusing the existing wide
coordinate interpolation. The numerator requires fewer than 386 bits within
Signed576. Final local coordinate error is at most half a raw unit; a pure
rotated component has error at most
`0.5 * (abs(cRaw) + abs(sRaw)) / 2^32` raw units, plus half a raw unit if the
world point is materialized. Original intervals remain authoritative for
coverage; rounded endpoints never replace them. Mixed callers receive native
local endpoints and use their actual quaternion frames. Complete polygon
reclipping and dimensional-owner failure handling remain Phase 6 consumers,
not new public APIs in this proof.

**Cone and mixed-slab boundary proof:** Existing family owners now consume
original-edge rational clips. Cone visitation creates the new cut endpoints,
filters every event cohort and replays with the same interval; final selection
retains the exact native point and exit fields rather than replaying a clipped
descriptor in an unclipped pool. Slab extrema constrain the original segment's
`lambda + mu` parameter before solving the existing finite support problem.
Exact cross-frame comparison precedes materialization; each anchor retains its
actual supplying frame. Collider IDs resolve equal-depth cross-member ties;
canonical span/event order resolves ties within a member.

Cone cut numerators remain below 518 bits, inside the existing 526-bit endpoint
and 326-bit denominator bounds; discriminants stay below 2,485 bits in the
40-word root bank. Slab clip coefficients remain below 319 bits, within the
existing 334-bit linear-row bound. Two additional domain rows require no new
solver: three active rows bound LP/conic determinants below 1,005/1,065 bits,
eliminated quadratics below 2,133 bits and retained final fields below 3,400
bits in 4,096-bit storage. Original intervals remain exact; rounding occurs
only when producing final anchors.

The bounded family fixtures include independently conditioned and opposite
normal frames, material/supplier ties, reversed member blocks, and complete
cone-cap exits. Circle, stadium and polygon slab fixtures touch every supplied
triangle at the selected exit and separate one raw unit farther. Partial-cut
cases establish boundary witnesses, not complete face pools or whole-union
clearance. Cone events retain the existing initial-section admission contract;
this proof does not turn them into arbitrary translated-solid recovery rays.
Nonaxial slab integration must retain the existing full-owner
physical-support search: a source outside the initial section can enter the
slab during translation. These helpers reuse existing family owners without
new production files, geometry frameworks or public APIs. Complete joined face
pools and dispatch remain Phase 3/7 integration work.

This is a coordinated internal-ABI change: commit/push FixedMathSharp first,
then pin Gravitas's develop workflow to that revision before pushing Gravitas.
Released-package validation follows the upstream release. Full-loop crossing
regressions belong to the runtime integration phase below; they cannot pass
before membership and response are wired. Keep the existing baseline fixtures.

**Phase 1 proof-to-runtime ownership:** At closeout, the caller audit confirmed
that joined capsule gathering, endpoint fans and planar boundary preparation were exercised
through tests/benchmarks rather than public collision dispatch. They are bounded
geometry proofs, not an enabled continuity feature. Their next consumers are:

| Retained owner | Required integration consumer |
| --- | --- |
| `MeshSurfaceBoundaryDomain` / `MeshSurfaceBoundaryDomain2D` | Membership-owned committed adjacency and dimensional exposed-feature generation |
| `MeshCapsuleFaceRegions` joined gathering | Existing mesh capsule/sphere face generation after membership/filter admission |
| `MeshSurfaceBoundaryFan` | Existing genuine-boundary generators at certified join endpoints |
| Cone/slab interval events and depth comparisons | Existing cone face/boundary and mixed slab support owners |
| Planar exposed endpoint finalization | Existing planar incident-feature reclipping before final anchors |

Do not duplicate these owners behind a new reducer framework. The proof's local
member set does not bound its triangle scans; the two-triangle rectangle budgets
cannot qualify a dense runtime mesh. Runtime promotion must use existing BVH and
topology access and membership-owned preparation, including reset/disposal of
retained references. Entry points superseded during that integration are deleted.


**Closeout validation, 2026-10-16:** FixedMathSharp passes 6,009 Release and
5,988 Lean core cases plus 49 recording cases per profile. Gravitas passes
6,811 Release and 6,743 Lean cases. Both libraries retain raw reachable 100%
line, branch and method coverage in both profiles, using `UseLocalLsfStack=true`.
All six native Windows x64 replay fixtures match across Release/Lean and shared
expectations. Source hashes remain frozen; no production files or public APIs
were added in this closeout. Independent geometry and ponytail reviews have no
outstanding findings, including the proof-only caller audit above.

The final ordinary ABBA compares this closeout with committed Gravitas `3e57caf`
and FixedMathSharp `9145a7a`, rather than reusing an earlier feature baseline.
All 128 actual observations are retained, allocate 0 B, collect no generations
and preserve physical signatures. Identical harness hot-method IL and all eight
live runtime modules are verified, with identical generated runtime settings.
Nominal changes are capsule `+0.52%`, pure 2D `+0.62%`, cone `-0.37%` and mixed
`+0.34%`. Capsule and pure 2D have small positive paired contrasts in both blocks;
cone and mixed reverse signs. Every process-paired 95% interval includes zero.
With only two independent paired blocks, uncertainty remains wide: this passes
the ordinary no-resolved-regression gate at the observed resolution, without
claiming equivalence, zero overhead or a speedup. No additional timing run is
justified by these results. Coverage/replay captures use `phase1-complete-v2`;
all timing observations and bindings remain under `perf/completion/`.
Standalone FixedMathSharp and Gravitas's pinned local graph have identical raw
executable bodies, layouts, constants and non-version configuration in both
profiles; only declared version metadata differs. The retained proof is
`perf/completion/fms-graph-proof/full-gate-to-gravitas-tested-proof.json`.

Phases 2–7 consume the bounded proofs through existing owners and delete
superseded entry points. It must establish complete joined face pools, actual
response, eligibility, membership lifetime and unsupported-placement handling;
these are not enabled by a geometry-only proof. Keep #133/#139 open through the
remaining runtime and continuation gates. Linux/ARM and released packages were
not validated locally.

### Phase 2 - Complete 3D Rounded Static-Surface Response

**Rewrite checkpoint, 2026-10-17:** The first implementation passed correctness,
coverage and replay gates but was rejected for size and native runtime cost:
one cube against two wall pieces cost about 9 ms in 3D and 39 ms mixed. Preserve
the prior correctness commits and Phase 1 foundation. The rejected implementation
and regression fixtures are retained in Gravitas stash `84d84b7` and
FixedMathSharp stash `434e6f9`, with an additional ignored checkpoint archive.
Bring back only code consumed by the rewrite; do not retain a second pipeline.

First establish correct, practical native convex contact generation using the
existing exact SAT and finite-feature operations. Then extend existing face and
perimeter consumers with the classified neighbor spans from Phase 1. Cross a join
before reducing face samples; retain real edges, partial coverage, creases,
supplier anchors and materials. Prove the small workload before expanding shape
coverage. Keep 100% reachable coverage and warmed allocation/performance gates.

**Committed native rewrite checkpoint:** That rewrite added 164 net runtime source
lines in Gravitas and 343 in FixedMathSharp, across nine existing files with no
new runtime files. The rejected parallel surface-generation pipeline remains
stashed. The existing convex SAT owner supplies one solid exit; its finite
support witnesses feed the existing plane sample reducer. Trusted convex mesh
patches and actual mixed polygon prisms share that producer. Nonconvex and
untrusted owners retain their finite triangle fallback. Mixed pairs reuse
unchanged manifolds through their existing publication and per-step solve reset.
Independent ponytail review found no unused production helpers or avoidable
runtime framework in this rewrite.

The maintained native fixture contains one unit convex cube and two finite wall
pieces, with frictionless forced response and frozen rotation at 16 Hz. It does
not declare continuity yet. Two BenchmarkDotNet launches, six warmups and eight
500 ms measurements per launch give:

| Full-step fixture | 3D | Mixed | Warm allocation |
| --- | ---: | ---: | ---: |
| Forced response, unchanged committed contact geometry | 49.60 us | 77.85 us | 0 B/op |
| Host moves the hull each step, requiring fresh geometry | 464.98 us | 583.09 us | 0 B/op |

All eight timed child launches loaded the benchmark and eight runtime modules
byte-identical to the validated Release payload. The moving case deliberately
exposes regeneration cost; cached timings are not a moving-contact capacity
claim. Captures and bindings are retained under ignored
`artifacts/grv-issue-133/phase2/rewrite/native-cache-bound-basis-v2*`.
This bound capture bypasses the stock generated-host build defect recorded as
#111; it does not resolve that independent tooling issue.

Full Release/ReleaseLean validation passes: Gravitas 6,864/6,796 tests,
FixedMathSharp 6,032/6,011 core tests plus 49 Chronicler tests per profile, and
raw 100% reachable line, branch and method coverage in both libraries. All six
shared replay fixtures agree across the two Windows x64 profiles. Physical
states, events and queries remain unchanged; only the eight mixed fixture hashes
change with `pair.mixed` schema 9 and normalized geometry-cache freshness.
Full coverage captures are `rewrite/gravitas-native-basis-final-v4` and
`rewrite/fms-native-basis-final-v3`. Linux, ARM and released-package validation are
not established by these local runs.

Nine ordinary-path ABBA controls preserve exact physical outputs and 0 B/op,
with all 36 timed child launches bound to their intended runtime payloads.
Paired means range from -1.74% to +1.62%. The rim's +1.62% (+1.89% in a focused
process repeat) reverses under interleaved load-order controls: -1.73% with the
baseline loaded first, +1.62% with the candidate first, -0.07% balanced. No
changed math helper executes that fixture; these captures do not establish a
repeatable rim regression. Public scalar hull controls are +0.66%/-0.36%, with
identical results and 0 B allocated. Retained evidence: `perf/rewrite-native-ordinary-v3*`,
`perf/rewrite-native-rim-v1*`, `rewrite/rim-paired-probe/*-*.json` and
`rewrite/public-sat-bound-v3*`, beneath the same ignored Phase 2 artifact root.
Independent review found every new helper has a production caller. Joined
membership and neighbor-aware response were still unimplemented at that
checkpoint; it did not close #133/#139.

The new math internals are a coordinated compatibility change. Commit and push
FixedMathSharp first, then advance the Gravitas develop CI pin to that fetchable
revision. Released-package validation still follows the upstream release.

The original Phase 2 mixed multiple dimensional implementations and shape
families. Its checklist is split into explicit phases below; completing this
phase means completing the 3D rounded-shape runtime contract, not completing
another exploratory checkpoint. Public exposure still waits for continuation
and the supported dimensional contract.

- [x] Wire 3D membership and rounded face/boundary response through existing
  owners. Cover spheres, zero/nonzero-core capsules and rounded compound leaves,
  bodyless/static members, free/frozen rotation and per-member events/materials.
- [x] Cover invalid ownership/roles, atomic membership replacement, removal,
  disposal/reset, ID/lifetime reuse, live eligibility and sleeping invalidation.
- [x] Consume exact partial exposed spans and their endpoint fans in the existing
  rounded boundary generator. Preserve real rims, holes, T-junctions, gaps,
  point touches, folds/ambiguity and reversed winding/member order. Remove the
  complete-edge restriction through actual support, including geometry mutation
  from a complete join to a partial join without a deferred preparation failure.
- [x] Qualify local gathering with changing neighborhoods and increasing triangle
  density/member count. Reuse existing BVH/topology and membership preparation;
  refine introduced time/allocation costs here. Do not qualify scaling from a
  repeated two-triangle rectangle alone.
- [x] Run final Release/Lean raw 100% coverage, allocation, replay and bound
  joined/interior/ordinary performance controls; obtain independent geometry and
  ponytail review. Record results here and close this phase when all tasks pass.

**Committed complete-edge response checkpoint, 2026-10-17:** Existing rounded
face/perimeter generators consume Phase 1 domains through context-owned internal
`StaticSurface` membership. Connected samples reduce before routing to their
supplying pairs; per-member events, materials and geometric overlap remain.
Registration, replacement, mutation, sleep and lifetime invalidation are wired
through existing owners. This checkpoint added 350 net Gravitas runtime lines
and one membership file, with no math change; tests/coverage and independent
review passed. The two-member sphere/capsule crossing cost 112.85/146.49 us/frame
with 0 B. Captures: ignored `artifacts/grv-issue-133/phase2/rewrite/` under
`joined-{release,lean}-final-source`, `joined-final-perf`, `joined-final-replay`
and `joined-controls`. Its partial-edge restriction and missing scaling evidence
are superseded by the completion below.

**Phase 2 complete, 2026-10-17:** Rounded response now handles exact partial spans,
cut endpoints and complete-to-partial mutation. Production regressions preserve
real rims/holes, gaps/touches, folds/ambiguity, reversed frames/winding at world
limits, sphere/zero-core equivalence, parallel axis reversal and disconnected
sections. Existing closest-feature projection, finite-ray materialization, fans
and local BVHs remain the owners; no second solver/index was added. Parallel
families retain stable interior witnesses instead of denominator-weighted jumps.

Final serialized local-stack validation has no new coverage exclusions:

| Core/profile | Tests passed | Covered sequence points | Covered branches | Covered methods |
| --- | ---: | ---: | ---: | ---: |
| FixedMathSharp Release | 6,044 | 60,353 / 60,353 | 17,530 / 17,530 | 4,541 / 4,541 |
| FixedMathSharp Lean | 6,023 | 60,345 / 60,345 | 17,530 / 17,530 | 4,537 / 4,537 |
| Gravitas Release | 6,940 | 48,888 / 48,888 | 16,956 / 16,956 | 5,154 / 5,154 |
| Gravitas Lean | 6,872 | 48,886 / 48,886 | 16,956 / 16,956 | 5,153 / 5,153 |

Raw line/branch/method counts and the FluentAssertions companion are complete.
Both 49-test Chronicler runs and both target-framework builds pass without
warnings. Six shared fixtures match expectations and Windows x64 Release/Lean
captures. Linux, ARM and released packages were not run locally.

The one-body frictionless 16 Hz crossing checks tangential velocity throughout
changing neighborhoods. Final Release BDN means use six warmups, eight iterations
and 32 invocations of 64 frames each:

| Joined workload | Sphere us/frame | Capsule us/frame | Warm allocation |
| --- | ---: | ---: | ---: |
| Two full-edge members | 109.27 | 138.70 | 0 B/frame |
| Partial join | 115.34 | 151.74 | 0 B/frame |
| 32 strips/member (128 triangles) | 185.38 | 239.12 | 0 B/frame |
| Sixteen two-triangle members | 207.32 | 263.29 | 0 B/frame |

Strict counters also retain 0 B. The short singleton BDN capture is diagnostic;
the larger batch supersedes its allocation deltas/first-case warmup distortion.
Frozen identical-fixture ABBA means are unchanged or lower (-0.66% to -6.81%);
partial joins had no supported old baseline. Nine ordinary controls include
reversed order; adverse short sphere/mixed rim means reverse in longer windows
(-1.68%/-1.98%). No persistent regression, equivalence or zero overhead is proven.
Cold declaration costs roughly 0.12–0.15 ms for two members and 3.24 ms for sixteen;
preparation still visits member pairs. These are bounded setup measurements,
not large-world capacity or frame-tail qualification.

Growth is **59 net Gravitas runtime lines and 164 FixedMathSharp lines**, with
one new math partial. Independent ponytail review found a lowest-ID real-corner
supplier lost behind joined edges; retaining their endpoints fixes its regression.
Final review has no outstanding correctness/bloat finding. Membership stays
internal; remaining families/dimensions/continuation have phases below.
#133/#139 remain open until that contract is complete.

Ignored captures: `artifacts/grv-issue-133/phase2/rounded-completion/`, including
`{fms,g}-final-{Release,ReleaseLean}`, `replay-comparison.log`, `bdn-warm-final`,
short/adverse controls and source/eight-module hashes. Commit/push FixedMathSharp
first, then advance Gravitas's develop CI pin; release it before package validation.

### Phase 3 - Complete Cone Continuity - Complete, 2026-10-18

The former all-family Phase 3 is split along existing contact owners. Cones
need native-frame critical events and clipped continuous families; cuboids and
cylinders can share the existing finite extrusion owner, while arbitrary hulls
use finite SAT witnesses. Keep each production integration bounded and run all
acceptance gates before expanding. This changes phase boundaries, not scope.

- [x] Integrate cone joined face pools and genuine clipped-boundary events in the
  existing cone owners. Boundary-witness proofs alone do not qualify the face
  reducer or complete-shape response; retire that superseded proof-only path.
- [x] Verify complete-cone clearance, finite anchors/materials, genuine ends,
  holes, ambiguous/folded joins, free rotation and order reversal. Include an
  actual cone-part compound through existing leaf dispatch.
- [x] Prove small and changing/dense/member workloads through the complete host
  step; preserve ordinary fast paths, 0 B warm allocation and warm controls.
- [x] Complete affected Release/Lean coverage, replay, allocation/performance and
  independent review gates before closing this phase.

Cone certification keeps an ambiguous canonical owner independent on both
sides of a prospective join. Its face events and rim solve retain the original
perimeter; consecutive retained classification spans are one edge domain. This
avoids manufacturing corners at arbitrary crossing cuts while preserving
certified partial joins between trusted owners. Real certified cut corners have
an authored canonical vertex supplier, selected in eligible collider/surface
order among owners that actually supply that vertex.

**Closeout:** Cone contacts now use the native face-event pool, exact regional
winner bank and existing final sample reducer across eligible members. Supplier
anchors/materials remain native; clipping intersects continuous families before
sampling. Capsule preparation/connectivity is extracted and consumed by both
families; the superseded cone boundary-proof API is deleted. Net runtime growth
is **384 Gravitas lines, two new files, and 219 FixedMathSharp lines, no new
runtime files**. Independent geometry and ponytail reviews have no outstanding
source findings. Their orientation, duplicate-sample, clipped-singleton and
ambiguous-cut findings have production regressions.

Final local-stack validation:

| Library/profile | Passed tests | Raw line points | Raw branches | Raw methods |
| --- | ---: | ---: | ---: | ---: |
| FixedMathSharp Release | 6,070 | 60,477 / 60,477 | 17,642 / 17,642 | 4,552 / 4,552 |
| FixedMathSharp Lean | 6,049 | 60,469 / 60,469 | 17,642 / 17,642 | 4,548 / 4,548 |
| Gravitas Release | 6,977 | 49,096 / 49,096 | 17,194 / 17,194 | 5,183 / 5,183 |
| Gravitas Lean | 6,909 | 49,094 / 49,094 | 17,194 / 17,194 | 5,182 / 5,182 |

FluentAssertions also has complete raw coverage. Both 49-test Chronicler adapter
runs and both target-framework builds pass without warnings. Six shared replay
fixtures match expectations and Windows x64 Release/Lean captures; the new
96-frame cone trace also repeats across declaration order. Whole-shape exits,
independent/opposed frames, finite ends, holes, remote joins, filtering,
ambiguity, free rotation and actual cone-part compound dispatch are covered.

Final Release BDN means on the i7-9700K/.NET 8.0.29 host use four warmups, six
iterations and 32 invocations of 64 complete frames at a configured 16 Hz:

| One-body cone workload | us/frame | Warm allocation |
| --- | ---: | ---: |
| Unsplit small floor | 189.34 | 0 B/frame |
| Two joined full-edge members | 240.70 | 0 B/frame |
| Partial join | 330.13 | 0 B/frame |
| 32 strips/member (128 triangles) | 348.27 | 0 B/frame |
| Sixteen two-triangle members | 310.97 | 0 B/frame |

All 24 sphere/capsule/cone cases and strict counters retain 0 B. Frozen ABBA
ordinary and rounded controls retain adverse runs: early native 3D, interior
cone and tilted-cone increases do not persist in longer reversed-order windows;
the small joined capsule's +1.68% becomes +0.17%. This does not establish exact
equivalence or zero overhead. These are bounded full-step fixtures, not game
capacity or frame-tail qualification. No introduced persistent regression was
established.

Ignored evidence is under `artifacts/grv-issue-133/phase3/`: raw
`{fms-final,g-accepted}-{Release,ReleaseLean}`, `replay-comparison.log`,
`bdn-warm-final`, paired/reversed controls, source and eight-module hashes.
Linux, ARM and released packages were not run locally. Commit/push FixedMathSharp
first, then advance Gravitas's develop CI pin. Membership remains internal;
#133/#139 stay open for the remaining checklist in Phases 4–14.

### Phase 4 - Complete 3D Extrusion Continuity And Response Ownership - Complete, 2026-10-18

- [x] Establish cuboid, cylinder and their compound-leaf behavior against
  declared surfaces. Reuse the finite extrusion support and boundary owners;
  retire superseded local-only branches where the shared owner covers both.
- [x] Keep exact initial overlap and per-member events separate from response
  supply. Gather projected supports outside the mover's initial AABB and carry
  complete component face/perimeter rows through an actually overlapping member,
  preserving finite supplier anchors, materials, provenance and warm-start
  invalidation without changing the pair or solver model.
- [x] Verify complete-shape clearance, finite anchors/materials, genuine ends,
  holes, ambiguous/folded joins, free rotation and order reversal. Maintain
  ordinary fast paths, 0 B warm allocation and practical moving-contact costs.
- [x] Complete affected Release/Lean coverage, replay, allocation/performance and
  independent review gates before closing this phase.

**Closeout:** Cuboids, cylinders and actual compound leaves now reuse the native
finite extrusion support, sample reducer and exact clipped-boundary owners.
A support outside the mover's initial AABB keeps its finite supplying member;
one genuinely overlapping member carries the coplanar component's face and
perimeter rows. Events retain actual member overlap. Keeping those rows in one
solver pass also prevents duplicate positional correction. Supplier identity is
separate from compound leaf identity, and supplier changes clear carrier warm
starts. Odd raw cylinder heights retain their exact caps before materialization.

Profiling led to bounded scalar determinant scratch, exact zero/constant pruning
and delayed radial scratch initialization in the existing math owner. An exact
filled axial-section certificate reuses its intrinsic support producer; holes,
partial rims and oblique cases retain the finite path. No solver or index layer
was added. Avoiding whole warm-start group copies during key/count lookups
resolved the introduced native-control increase without a new cache.

Final local-stack validation:

| Library/profile | Passed tests | Raw line points | Raw branches | Raw methods |
| --- | ---: | ---: | ---: | ---: |
| FixedMathSharp Release | 6,130 | 60,611 / 60,611 | 17,752 / 17,752 | 4,556 / 4,556 |
| FixedMathSharp Lean | 6,109 | 60,603 / 60,603 | 17,752 / 17,752 | 4,552 / 4,552 |
| Gravitas Release | 7,028 | 49,351 / 49,351 | 17,446 / 17,446 | 5,208 / 5,208 |
| Gravitas Lean | 6,960 | 49,349 / 49,349 | 17,446 / 17,446 | 5,207 / 5,207 |

FluentAssertions also retains complete raw coverage. Both target-framework builds
and both 49-case Chronicler adapter runs pass without warnings. Six shared native
replay fixtures agree across Windows x64 Release/Lean and reviewed expectations.
Manifold/warm-start hash sections advance to 9/3 for supplier references; only
expected hashes change in five fixtures, with identical body, event and query
data. A single-supplier test normalizes registry references; reconstructed
multi-supplier solver ordering remains Phase 13's continuation contract.

The final moving one-body batch uses 64 complete frames per invocation at a
configured 16 Hz, 24 warmed windows and four invocations/window on the
Windows/i7-9700K/.NET 8.0.29 host:

| Workload | Cuboid us/frame | Cylinder us/frame |
| --- | ---: | ---: |
| Unsplit small floor | 95.38 | 138.29 |
| Two joined full-edge members | 401.67 | 505.54 |
| Partial join | 1,079.20 | 1,032.72 |
| 32 strips/member (128 triangles) | 665.53 | 854.53 |
| Sixteen two-triangle members | 752.37 | 1,017.90 |

All ten cases retain 0 B/frame and exact frictionless tangent velocity. These are
bounded moving-contact budgets, not game-capacity or frame-tail qualification.
Thirteen ordinary controls use frozen graphs, exact inspected physical outputs,
actual eight-module bindings and two reversed-load processes. All warmed
windows allocate zero bytes; mixed manifold means improve 14–56%. Initial native
increases persist in longer windows, then disappear after the lookup refinement:
long native 3D contrasts are +0.25%/-3.46%, mixed -2.13%/-4.20%. A later capsule
increase changes sign in longer windows (+1.17%/-1.06%); pure 2D is lower in both.
No persistent ordinary regression remains at this resolution; this is not exact
equivalence or zero-overhead proof. All adverse and short runs are retained.

Net runtime growth is **453 Gravitas lines, one new runtime file, and 229
FixedMathSharp lines, no new runtime files**. Independent geometry and ponytail
reviews have no outstanding findings. The separate rotated-mesh recentering
defect is captured as [#141](issue-tracker.md#grv-issue-141---recentered-rotated-mesh-pieces-lose-exact-authored-shared-vertices);
shared authored vertices need preservation, rather than tolerance welding.

Ignored evidence is under `artifacts/grv-issue-133/phase4/`, including
`{fms-final,g-accepted-ref}-{Release,ReleaseLean}`, `replay-ref-comparison.log`,
`ordinary-ref-{ab,ba,assessment}.json`, `ref-*`, `final-long-*`,
`extrusion-ref-final.json`, source hashes and frozen payloads. Linux, ARM and
released packages were not run locally. Commit/push FixedMathSharp first, then
advance Gravitas's develop CI pin. Membership stays internal; #133/#139 remain
open for the original remaining family/dimensional/continuation checklist below.

### Phase 5 - Complete Native Mesh Continuity

The original remaining-3D checklist is split at the existing geometry owners.
Extrusions use the finite slab support owner; native hull and triangle pairs
need exact finite-source admission during axis selection. Complete Phase 4's
small-workload performance and correctness gates before expanding these owners.

- [x] Establish convex/concave mesh and mesh compound-leaf behavior against
  declared surfaces. Preserve closed convex volume semantics and open/concave
  sheet semantics; reuse the native hull and triangle-pair producers rather
  than convexifying members or creating another union solver.
  The committed Phase 4 two-member floor probe reproduced an artificial +X exit
  for a closed convex cube at `(1/4,1/4,0)`, captured in the ignored
  `artifacts/grv-issue-133/phase4/repro.log`. Its cuboid counterpart belongs to
  Phase 4. Cylinder and concave cube pass that initial probe; that result does
  not establish complete mesh continuity.
- [x] Admit the exact finite source support before ranking response exits,
  including candidates generated by the moving hull's own face axes. Assess
  both signed directions independently and preserve every separation check;
  dropping a covered winner afterward must not discard another valid exit.
- [x] Clip original finite support intervals before witness rounding. A partly
  exposed edge is insufficient when all matched witnesses lie on its joined
  portion. Extend the existing rational witness and finite-feature owners only
  as needed for this implemented consumer; do not filter rounded anchors.
- [x] Preserve the native coplanar triangle-pair axes, including plane-normal
  cross own-edge directions. Keep holes, genuine corners and ambiguous joins
  finite; a degenerate hull substitution must not remove those separation axes.
- [x] Reuse component carriers, exact supplier anchors/materials, sample
  reduction and lifetime invalidation. Retire superseded local-only branches
  where the shared owner completely covers their contracts.
- [x] Verify complete-shape clearance, genuine ends, holes, folded/ambiguous
  joins, free rotation, argument/declaration reversal and compound provenance.
  Preserve ordinary fast paths and practical full-loop costs; complete the
  same Release/Lean coverage, replay, zero warmed allocation, performance and
  independent review gates before closing this phase.

**Phase 5 complete, 2026-10-18:** Native hull/triangle producers now admit exact
retained finite features before signed exit ranking, preserving native coplanar
axes. Canonical convex moving patches avoid artificial diagonal dependence;
genuinely nonconvex sheets retain their finite native triangles. Authored
collinear subedges map to canonical retained intervals before witness rounding.
Existing component carriers, support graphs and reduction retain supplier
anchors/materials, holes and disconnected regions. No new solver or index was
introduced. Runtime growth is 641 net lines/one new file in Gravitas and 174 net
lines/no new files in FixedMathSharp; duplicated carrier logic and an unused
anchor wrapper were removed.

Review corrected two diagnostics that inferred whole-union recovery from local
rows. Those expectations exceeded the local stabilization contract and did not
establish a runtime defect, so the interval-solver proposal was withdrawn.
Maintained tests retain finite signed witnesses, actual missing areas
and explicitly chosen clear poses. Original assertions remain in ignored
`phase5/rejected-recovery-oracles.cs`; no recovery guarantee was added.

The full-host cube probe measures 64-step batches (`Simulate` + `LateSimulate`),
with exact tangential-speed guards and zero warmed allocation. Repeated balanced,
reversed-load-order captures retain the earlier results; final frozen-payload
medians follow. Aligned warm/timed body position, rotation and velocities agree
exactly across the supplier-culling refinement.

| Joined workload | Before supplier cull, us/frame | Refined, us/frame |
| --- | ---: | ---: |
| Convex cube, two members | 367 | 248 |
| Concave cube, two members | 551 | 423 |
| Convex cube, sixteen members | 1,053 | 595 |
| Concave cube, sixteen members | 1,301 | 832 |

The earlier first concave implementation cost 1.87 ms/frame. Unsplit controls
measure approximately 119 us convex/603 us concave; corrected joined work is
not equivalent to those paths. Dense and partial-layout checks also retain
zero allocation. Captures and actual module hashes are under ignored
`artifacts/grv-issue-133/phase5`, including `culled-*.json`, `state-*.json` and
`final-*.json`.

Thirteen ordinary controls preserve exact initial outputs/state and zero
allocation. An initial mixed-circle interior result near +3% did not persist
in four longer independent reversed-order processes (-1.06% to +0.34%); adverse
captures remain retained. Five previously completed joined families preserve
raw body state and zero allocation against the committed stack; the initial
cuboid +0.4% changes sign in a reversed-order control (-0.52%). Matching new
ordinary mesh fixtures against the old runtime gives approximately 129→117 us
convex and 602→604 us concave, with sign-changing concave contrasts. Those
captures record actual loaded assembly paths/hashes, not only input filenames.
These controls establish no resolved regression at the measured resolution,
not universal zero overhead.

Final Release/Lean suites pass 7,124/7,056 Gravitas and 6,193/6,172 FixedMathSharp
cases, with raw 100% line, branch and method coverage in all four runs.
Both target frameworks build in both profiles; 49 math recording-adapter tests
pass per profile. All six shared replay fixtures agree across local Windows x64
Release/Lean and their expectations. `g-final-*`, `fms-mapping-*` and
`replay-comparison.log` retain the reports. Fresh independent ponytail review
has no outstanding findings. Native tangential witness tests retain the
reviewed Q32 barycentric rounding bound (11 raw units for their maximum
10-unit edges); admission, normal and depth checks are not relaxed.

Commit/push FixedMathSharp first, then advance Gravitas's develop CI dependency
pin to that fetchable commit before pushing Gravitas; an uncommitted internal
ABI has no usable pin. Released-package, Linux and ARM validation were not run
locally. #133/#139 remain open for the following dimensional/continuation phases.

### Phase 6 - Complete Pure 2D Static Continuity

- [x] Implement typed 2D membership through existing services/lifecycle owners;
  retain the same ownership, transactional replacement and lifetime guarantees.
- [x] Implement #139's dimensional boundary reduction without projecting 3D
  sheets into 2D volumes. Test circles, zero/nonzero-core capsules and polygonal
  bodies against connected convex pieces and genuine concave corners.
- [x] Reclip finite incident features against exact exposed spans before final
  endpoint materialization. Preserve independent normal groups, supplier
  materials and overlap truth separately from response rows; no global two-row cap.
- [x] Certify full recovery against the final pose's complete eligible spatial
  candidates and wire the dimensional deep-placement failure contract. Triggers,
  query-only and immovable observations must not require physical recovery.
- [x] Verify full-loop baseline crossings, free rotation, real boundaries, order,
  membership/filter changes and density/scaling; complete coverage, replay,
  allocation/performance and independent review gates before closing this phase.

**Completion, 2026-10-19:** Internal `StaticSurface2D` uses the existing spatial
candidate collector and Phase 1 boundary-domain owner. Registered static AABB
and convex-polygon members have transactional replacement, dependency
invalidation and context/reset/disposal cleanup. Circles, zero/nonzero-core
capsules and polygons retain exact finite face/vertex constraints, independent
normal groups and supplying-member materials. Borrowed planes require connected
solid spans; genuine rims, concave corners and gaps remain physical features.
No new spatial index or public membership API was introduced.

The existing 2D manifold, warm-start cache and solver now retain two samples per
independent structural group. Ordinary compounds keep their existing default
sample pool. Genuine member overlap/events remain separate from response rows;
after callbacks, the final response boundary revalidates registry identity,
eligibility and changed dependencies before selecting the current carrier.
Recovery proposals are conservative and must clear the whole moving shape
against fresh complete final-pose candidates, including newly encountered
blockers and target compounds. Unsupported deep placement fails explicitly;
observations and immovable pairs require no physical recovery. Existing cold
invalidation clears full reference-array capacity, including stale tails beyond
`Count`, so retained handles cannot retain ended collider lifetimes.

Small full-host crossings, free rotation, rounded/polygonal shapes, bodyless and
explicit-static members, reversed order, material/filter/geometry changes,
callback-time carrier removal, recovery failures and lifetime reuse pass.
Gravitas Release/Lean pass **7,441/7,372** tests; FixedMathSharp passes
**6,624/6,603**, plus **49 recording-adapter tests per profile**. Raw reachable
line, branch and method coverage is exactly **100% in all four library runs**,
without new exclusions. Both target frameworks build in both profiles without
warnings/errors. All six shared replay fixtures match their expectations and
local Windows x64 Release/Lean captures. Only the two 2D-bearing fixtures'
authoritative hashes change for the explicit manifold/warm-start group schema;
physical, event and query expectations remain unchanged.

Warm Release measurements use the maintained joined-surface fixtures through
the verified timing host: one moving actor, 16 Hz, frictionless driven motion,
64 complete `Simulate`/`LateSimulate` frames per invocation, Windows x64
.NET 8.0.29, `UseLocalLsfStack=true`. Loaded module hashes accompany captures.
Values below are **microseconds per complete frame**, all **0 B/frame**:

| Moving shape | Continuous floor | 2 joined pieces | 16 joined pieces | 64 joined pieces |
| --- | ---: | ---: | ---: | ---: |
| Circle | 21.48 | 46.14 | 108.26 | 452.50 |
| Capsule | 26.72 | 62.38 | 160.47 | 777.40 |
| Polygon | 40.45 | 108.47 | 271.00 | 1,023.04 |

All layouts cover the same 16-unit floor. Profiling dense capsule response
identified repeated exact comparisons of disjoint finite features. Inclusive
outward-bound rejection now precedes those comparisons, reusing existing edge
bounds. Dense circle/capsule/polygon cost fell from approximately
1,387/5,870/2,657 us without changing the mixed quaternion chart. Increasing
remote loaded population to 64 raises both joined and continuous controls;
their difference remains approximately 23/36/69 us, consistent with local
membership gathering rather than a warm scan of every declared member. Broken
independent-seam response is a diagnostic, never the acceptance denominator.
The unchanged no-membership population fixture also reproduces 21.57 -> 77.09
us/frame on the frozen pre-phase stack (one versus 64 loaded statics), with
matching physical witnesses and zero warm allocation. This separate existing
refresh-cost signal is recorded as GRV-Benchmark-030; no unrelated optimization
was added to this phase.

Balanced unaffected controls put the ordinary 64-pair 2D full step at
**923.1 -> 941.0 us (+17.9 us, +1.94%)**, with **0 B/op**. The user explicitly
accepted this measured correctness cost on 2026-10-19. Other controls range
from -1.73% to +0.85%; these observations do not establish universal zero
overhead or game-capacity/tail-latency guarantees. Earlier adverse runs and a
slower attempted solver-row simplification remain in the capture set.

Independent ponytail review has no outstanding findings after callback-carrier
and retained-reference regressions were resolved. Net runtime growth is
**1,117 lines / four new files in Gravitas** and **542 lines / three focused
partials in FixedMathSharp**, excluding tests/docs. The existing boundary owner
has its production consumer; obsolete endpoint materialization, unused source
metadata and six redundant warm-cache forwarders were removed. Reports, raw
coverage, module hashes, profiles and repeated timings remain under ignored
`artifacts/static-surface/phase6/` rather than a separate phase report.

Commit/push FixedMathSharp first, then advance Gravitas's develop CI pin from
`8302f055fe28560109780b57579e97d8b44a8899` to the new fetchable commit before
pushing Gravitas. Released-package, Linux and ARM validation were not run
locally. #133/#139 remain open; mixed continuity, queries/CCD, membership
recording/reconstruction and public-contract closeout remain Phases 7–14.

### Phase 7 - Mixed Response Against Joined 3D Meshes

- [x] Apply the same ownership/eligibility contract to embedded 2D movers against
  declared static 3D meshes. Complete joined face pools through the existing mixed slab support
  and clipped-boundary owners, including a vertical 3D sheet with a planar
  response normal. Retire the Phase 1 slab boundary-witness proof once the
  production path covers its nonzero-core embedded cases.
- [x] Preserve per-member events/query identity, deterministic solver ordering,
  materials and lifetime invalidation; cover actual full-loop crossings and
  complete-shape response before the same coverage/performance/review closeout.

**Scope split, 2026-10-25:** The original Phase 7 combined two distinct physical
owners. A moving embedded slab can reuse the joined mesh-face producer. Static
2D members instead describe filled solid prisms; their ordinary contact owners
select a whole-volume minimum and use native scalar yaw for primitive contacts.
Removing a covered winning contact afterward cannot regenerate a valid exposed
exit, and copied quaternion-oriented mesh proxies would not preserve the exact
prism chart. Separate that work into Phase 8 below rather than adding another
mesh/BVH representation or expanding this integration into several arithmetic
families at once. Both directions remain required before issue closeout.

**Completion, 2026-10-25:** Existing surface/slab owners now build mixed
discrete manifolds for circles, zero/nonzero-core capsules, AABBs, convex
polygons and compound leaves. Actual member overlap/events remain independent
of supplier rows, materials and provenance. Final callback validation rejects
retired carriers and rebuilds changed dependencies. Retirement clears queued
aliases before pooling, including the confirmed pure 2D counterpart.

Exact exit comparison retains one peak per member; support dimension follows
the selected orientation. Existing arithmetic scratch is shared across active
vertices with reentrant visitors covered. Exact-zero-normal bounds rejection
skips unreachable spans/members; tiny nonzero tilts and suppliers outside the
initial section retain the finite path. No new index, proxy or solver was added;
the slab proof-only helper/tests were retired. Net runtime growth is **90
lines/one new file in Gravitas** and **41 lines/no new files in FixedMathSharp**,
excluding tests/docs. Independent geometry/ponytail review has no open findings.

Gravitas Release/Lean pass **7,488/7,419** tests; FixedMathSharp passes
**6,861/6,840**, plus **49 recording-adapter tests per profile**. All four
captures have exact raw **100% line, branch and method coverage**, without new
exclusions. Both target frameworks build in both profiles with zero warnings
or errors. All six shared replay fixtures match their expectations and local
Windows x64 Release/Lean captures; no expectation was changed.

Warm Windows x64 .NET 8 measurements use one driven planar body crossing the
same 16-unit vertical wall at 16 Hz, with 64 complete `Simulate`/`LateSimulate`
frames per invocation. Values are **microseconds per frame**, all **0 B/frame**:

| Moving shape | Continuous wall | 2 joined pieces | 16 joined pieces | 64 joined pieces |
| --- | ---: | ---: | ---: | ---: |
| Circle | 1,464.96 | 641.61 | 1,077.58 | 2,905.88 |
| Capsule | 2,382.52 | 721.21 | 1,229.64 | 3,443.58 |
| Polygon | 756.77 | 590.06 | 1,361.85 | 3,680.42 |

Initial two-piece costs were approximately 2,923/5,483/1,446 us; initial
64-piece costs were 6,158/10,651/7,295 us. Profiling led to the shared bounds
proof rejecting unreachable members before support construction, preserving
topology and identical raw body state through 512 additional frozen-payload
frames per dense shape. Fragmentation remains costly; these are lifecycle
fixtures, not game-capacity/frame-tail guarantees. Broken independent seams
are not an acceptance denominator.

Seven matched ordinary/previously joined controls retain exact physical
witnesses and zero warm allocation. The existing joined 3D cylinder full step
improves approximately 8.7%. Small ordinary contrasts change sign across fresh
normal/reversed-load processes. An adverse 64-pair 2D contrast of +3.41% does
not persist in four focused ABBA repeats (-1.06%, -0.28%, -0.17%, +2.04%);
native mixed contrasts range -2.96% to +3.57% across six processes. These runs
do not resolve an ordinary regression or prove universal zero overhead. The
pre-phase payload already includes the explicitly accepted Phase 6 cost.

Coverage, module hashes, state comparisons and adverse/refined timings remain
under ignored `artifacts/static-surface/phase7/`, including
`coverage-closeout-*`, `math-final-coverage-*`, `member-cull-state.json`,
`controls-accepted-*`, `controls-focused-*` and `mixed-accepted-timing.json`.
Commit/push FixedMathSharp first, then advance the develop CI pin to that
fetchable commit before pushing Gravitas. Released-package, Linux and ARM
validation were not run locally. #133/#139 remain open for Phases 8–14;
per-collider query/CCD behavior remains unchanged for Phase 13.

### Phase 8 - Native Mixed Response Against Joined 2D Solids

**Scope split, 2026-10-25:** Native boxes/hulls share exact SAT and finite
polyhedral support. Curved shapes use different finite support solvers. Complete
the native production path and its gates here before integrating those families
in Phases 9–12; do not add unused curved-shape infrastructure to this phase.

- [x] Reuse typed 2D membership and exact exposed boundary spans for static
  embedded AABB/polygon members against moving 3D boxes and closed convex hulls. Preserve the actual
  primitive and mesh charts; prove the small native crossing before broadening
  families. Admit finite exposed features before contact selection, preserving
  filled-solid containment and genuine cap/side/rim constraints.
- [x] Verify matching/differing host Y and half-thickness. Planar adjacency alone
  must not erase a real step or finite rim; retain rational cuts through final
  witness materialization and check whole-shape clearance.
- [x] Preserve member observations, supplier materials, independent solver
  groups, mutations and lifetime invalidation. Complete full-host shape/order,
  raw coverage, replay, warm allocation/performance and independent review gates.

**Completion, 2026-10-26:** Native boxes and closed convex hulls now use the
existing SAT, finite visitors, boundary spans and sample reducer against joined
2D prisms. Actual scalar-yaw and authored mesh charts survive selection. Side
bands follow physical Y overlap; exact retained intervals/fans keep real steps,
rims, holes and disconnected supports. Each connected support region retains its
own samples, finite supplier/material and real overlap carrier. Adaptive query
reach includes deeper projected witnesses without a fixed expansion budget.

All 109 native regressions pass, including full host crossings/order checks,
affine/large-world charts, whole-shape exits, compounds, callback invalidation,
mutation and retained lifetimes. Full solutions build both target frameworks in
Release/Lean without warnings. Raw reachable coverage is complete, with no new exclusions:

| Library/profile | Passing tests | Lines | Branches | Methods |
| --- | ---: | ---: | ---: | ---: |
| Gravitas Release | 7,612 | 51,473/51,473 | 19,614/19,614 | 5,440/5,440 |
| Gravitas ReleaseLean | 7,543 | 51,471/51,471 | 19,614/19,614 | 5,439/5,439 |
| FixedMathSharp Release | 6,911 | 66,232/66,232 | 21,134/21,134 | 5,147/5,147 |
| FixedMathSharp ReleaseLean | 6,890 | 66,325/66,325 | 21,134/21,134 | 5,143/5,143 |

All 49 math recording-adapter tests pass in each profile. Six shared replay
fixtures match unchanged expectations and direct Windows x64 Release/Lean
captures. New continuity trajectories also repeat exactly; full membership
recording/hash/CCD conformance remains Phase 13.

Warm Windows x64 .NET 8 measurements drive one 3D body along the same 16-unit
static 2D wall at 16 Hz, with 64 full host frames per invocation. Values are
**microseconds per frame**, all **0 B/frame**:

| Moving shape | Continuous wall | 2 joined pieces | 16 joined pieces | 64 joined pieces |
| --- | ---: | ---: | ---: | ---: |
| Box | 111.93 | 438.01 | 905.47 | 2,661.88 |
| Hull | 178.77 | 417.84 | 880.33 | 2,716.24 |

Two-piece costs initially measured approximately 647/605 us. Retaining the
original rigid visitors and culling unrelated zero-yaw footprint bounds keeps
that native path small; the latter reduces the dense hull fixture from 5,037
to 2,716 us (46.1%). Frozen-payload comparisons retain identical raw body states
through 512 additional frames per layout. Fragmentation has a real cost; these
fixtures establish corrected-work costs, not game-capacity/frame-tail budgets.

Seven ordinary/previously joined controls preserve exact physical witnesses
and zero allocation. Final normal/reversed contrasts range -1.95% to +2.38%;
adverse contrasts change sign, including native mixed +2.38%/-1.95% and 2D
64-pair +0.23%/-0.90%. Earlier controls and adverse runs are retained. Direct
#143 sphere/hull sweep repeats also preserve hit witnesses and zero allocation;
four focused runs range -0.18% to +0.89% for spheres and -2.77% to +0.30% for
hulls. These measurements do not establish an ordinary regression or prove
universal zero overhead; no new baseline tradeoff was accepted.

Independent geometry and ponytail reviews are clear. Net source growth is
+1,146 Gravitas geometry lines (two owner partials) and +440 FixedMathSharp lines
(one native-yaw frame file plus shared existing kernels). All new internal
helpers have production callers; no proxy collider, index or solver was added.
The strict-convex topology invariant also removes the cap third-vertex search.

**#143 query-lifetime closeout, 2026-10-26:** The initial 3D source/exclusion
cleanup is committed in `93ae61f` (+66 runtime lines, six duplicate wrappers
removed). Dimensional follow-up also releases 2D/mixed candidates, all three
services' internal batch hits after each caller copy, and completed 3D voxel
trace references. Hit-output replacement uses `Clear()` at the existing admission
boundary: a smaller/empty query cannot hide older collider/body references, and
current caller-owned hits remain valid. Capacity, counters and registered-source
preparation within a batch are preserved; numeric/geometry scratch is unchanged.
The parity revision adds 248 runtime lines across existing owners, two private
cleanup methods, and no runtime file/type. Independent ponytail review is clear.

All **48 lifetime cases** pass (33 buffer/output cases and the original 15 source
cases). Full Gravitas Release/Lean suites pass **7,645/7,576** tests with exact
raw line coverage **51,575/51,575** and **51,573/51,573**, branch coverage
**19,612/19,612** in both, and methods **5,442/5,442** / **5,441/5,441**.
Both target frameworks build without warnings. Six unchanged shared replay
fixtures agree across local Windows x64 profiles; upstream source was not changed.

Twelve frozen-payload query/frame controls preserve identical physical witnesses
and **0 B/op**, including 16-request 3D/2D batches, 48-query area work, both mixed
directions, 64-agent support/full-step work and the existing 64-pair 2D control.
Normal/reversed broad contrasts range -7.09% to +5.38%; the adverse 2D contact
frame +5.38% reverses to -1.84%, and the support frame +5.23% to -1.27%.
Four fresh focused ABBA processes span **-3.06% to +2.37%**, without a persistent
adverse contrast. This establishes no repeatable warm regression, not universal
zero overhead; no new baseline tradeoff was accepted. Ignored
`artifacts/grv-issue-143/` retains red/green lifetime evidence, exact coverage,
replay, before/after payloads with module hashes, broad `timing-*` and focused
`focused-*` captures. Pooled voxel survival was not used as a GC leak oracle:
GridForge intentionally pools reset voxels; trace cleanup was audited and
exercised through existing ray, cone, sphere and registered-source tests.

Ignored `artifacts/static-surface/phase8/` retains `coverage-final-*`, math raw
coverage, `native-closeout-*`, `controls-closeout-*`, earlier/adverse controls,
`query-focused-*`, `replay/` and exact source/module hashes. Everything used
`UseLocalLsfStack=true`; Linux/ARM and released-package lanes were not run locally.
Commit/push FixedMathSharp first, then advance the develop pin to that fetchable
revision before pushing Gravitas. #133/#139 and public membership remain open
for Phases 9–14; this phase closes the native box/hull portion only.

### Phase 9 - Spherical Movers Against Joined 2D Solids

**Scope split, 2026-10-27:** The previous remaining-movers checklist crossed
three different exact feature owners. Complete the smallest rounded production
path here: spheres and geometrically equivalent zero-core capsules. Nonzero
capsules require retained projected intervals, while cylinders/cones require
curved support selection and open meshes require patch regions. Those are
explicit subsequent phases below, not infrastructure prerequisites here.

- [x] Extend the existing exposed prism domain to spheres and zero-core capsules.
  Preserve complete solid admission; admit exact finite supports before depth
  ranking, including both oriented exits, cap/side/rim features, native scalar
  yaw and containment. Keep public per-member queries and overlap events distinct.
- [x] Prove near-seam and whole-shape exits, equivalent representations, full
  host crossings, suppliers/materials, height steps, mutation and lifetime.
- [x] Complete raw coverage, replay, zero warm allocation/performance controls
  and independent geometry/ponytail review before closeout.

**Completion, 2026-10-27:** Spheres and zero-core capsules
reuse existing native prism charts, bands, joins, fans, carriers and adaptive
gathering. Complete solid separation remains independent from exact finite-point
admission before exit selection. Concave side/cap peaks use the existing
point-ray owner; exact identity removes duplicate height-band/cap-vertex samples.
A sufficient half-plane certificate cached with prepared geometry skips impossible
peaks on convex components. The local stabilization and separately certified
whole-union recovery contract is unchanged.

Profiling identified radical depth comparison as the largest arithmetic cost.
Equal radial coefficients and axis norms now cancel before a widened integer
comparison; unequal terms retain the existing comparer. Every new helper has a
production consumer. Runtime growth is 322 net lines/one new file in Gravitas and
277 net lines/two new files in FixedMathSharp. Independent geometry and ponytail
reviews have no remaining findings.

The 63 runtime and 31 math cases cover native yaw, whole-shape clearance checks,
containment, concave cuts/steps, simultaneous global/finite contacts, suppliers,
materials, reversed host crossings, mutation and lifetimes. Duplicate regressions
failed before their fixes. Nonzero capsules, cylinders/cones and open meshes
retain their ordinary paths pending their explicit phases.

Both solution configurations build `net10.0`/C#14 and `netstandard2.1`/C#11 with
zero warnings/errors, using `UseLocalLsfStack=true`. Final raw coverage:

| Library/profile | Passing core tests | Lines | Branches | Methods |
| --- | ---: | ---: | ---: | ---: |
| Gravitas Release | 7,708 | 51,742/51,742 | 19,784/19,784 | 5,452/5,452 |
| Gravitas ReleaseLean | 7,639 | 51,740/51,740 | 19,784/19,784 | 5,451/5,451 |
| FixedMathSharp Release | 6,942 | 66,392/66,392 | 21,216/21,216 | 5,156/5,156 |
| FixedMathSharp ReleaseLean | 6,921 | 66,485/66,485 | 21,216/21,216 | 5,152/5,152 |

All 49 math recording-adapter cases pass in each profile. All six shared replay
fixtures retain their expectations and agree between Windows x64 Release/Lean.
The baseline context-reset weak-reference probe exposed pre-existing bounded
retention on `3f0b759`, recorded separately as #144. Declaration disposal and
semantic reset/reuse checks pass.

Warm Release measurements on Windows x64/i7-9700K, .NET 10/SDK 10.0.302, use
one radius-0.5 mover, a 16-unit wall, 16 Hz full host steps, tangential motion
between X=-2/+2 and repeated inward impulses. Each measured invocation contains
64 `Simulate`/`LateSimulate` steps. Final mean time per frame, all **0 B/frame**:

| Mover | Continuous wall | 2 pieces | 16 pieces | 64 pieces |
| --- | ---: | ---: | ---: | ---: |
| Sphere | 0.202 ms | 0.463 ms | 0.671 ms | 2.025 ms |
| Zero-core capsule | 0.207 ms | 0.478 ms | 0.674 ms | 2.024 ms |

The initial correct spherical implementation cost about 0.623/0.861/2.046 ms
for 2/16/64 pieces. Profile-guided refinement preserved exact flat-wall state
through 512 additional frames per layout. Paired, reversed-order ordinary
sphere controls improve 33.6–34.5%; zero-core capsules improve 33.5–33.8%.
Existing nonzero-core capsule/prism containment and off-center crossing improve
32.2–32.4% and 10.6–11.4%, with identical witnesses. Eighteen ordinary query,
support, contact and simulation controls use identical fixtures/drivers and 24
balanced windows per process, all 0 B/op. Initially positive native box/hull
contrasts changed sign in focused repeats; unaffected controls did not establish
a regression. These scene costs do not establish game-capacity or frame-tail
budgets.

`artifacts/static-surface/phase9/` retains final `*-final-*-e` raw coverage/logs,
`replay/`, baseline/candidate binaries, source/module hashes, `controls-*`,
`focused-native-*`, `spherical-*`, the profile and earlier/adverse captures.
`identical-native-*` calibration is rejected because its driver copy differs;
`closeout-audit.json` records that exclusion and verifies final payload bindings.
Everything used `UseLocalLsfStack=true`. Linux/ARM, Unity runtime and released
packages were not executed. Push FixedMathSharp first, then advance Gravitas's
develop pin to that fetchable commit. Public continuity and #133/#139 remain
open for Phases 10–14.

### Phase 10 - Nonzero-Core Capsules Against Joined 2D Solids

- [x] Reuse rounded selection with exact retained projected core intervals and
  clipped closest-edge families. A covered midpoint must not discard a valid
  retained interval; preserve the complete capsule and its actual quaternion.
- [x] Complete whole-shape/full-host, mutations, suppliers, coverage, replay,
  zero warm allocation/performance and independent review gates.

**Completion, 2026-10-27:** Nonzero-core capsules reuse native prism charts,
height bands, incident fans, adaptive gathering and response carriers. Exact
projected intervals are retained before selection; original edges reuse finite
closest-point matching, and concave lines/points reuse capsule events. Complete
solid admission remains independent of exposure. Physical constraints precede
recovery, which runs only for components without physical rows.

Selected projection/clipping is prepared once. Physical free-core intervals
retain the primary point and admitted extrema; fixed tips/recovery retain one
point. Every sample refreshes its own incident fan immediately before emission.
The existing four-point reducer preserves the deepest, lowest-lever physical
point using the actual body COM, then ranks coverage; canonical contact IDs
still determine solver order. Local connectivity, materials, suppliers, compound
namespaces and member events remain explicit. Independent geometry/ponytail
review has no remaining findings. Runtime growth: **515 net lines/two new files
in Gravitas; 883 net lines/two new files in FixedMathSharp**. Every new internal
helper has a production consumer.

The 46 runtime and 69 focused math cases cover retained core intervals, native
yaw/general quaternions, actual finite witnesses, oblique reselection, true rims,
concave/ambiguous/disconnected regions, height cuts, remote recovery, scalar
limits, callback invalidation and bodyless/off-center compound ownership.
Repeated full-host crossings cover both orientations, frozen/free rotation and
initial spin. Finite PGS iterations can leave angular residuals: free-angular
whole-shape checks include incoming motion and actual rotation-induced reach,
then require 80% correction with four raw rounding units. Frozen rotation keeps
its stricter steady-state bound; zero angular equilibrium is not claimed.

Both solution profiles build net10.0/C#14 and netstandard2.1/C#11 with zero
warnings/errors, using `UseLocalLsfStack=true`. Final raw coverage:

| Library/profile | Passing core tests | Lines | Branches | Methods |
| --- | ---: | ---: | ---: | ---: |
| Gravitas Release | 7,753 | 52,028/52,028 | 20,094/20,094 | 5,475/5,475 |
| Gravitas ReleaseLean | 7,684 | 52,026/52,026 | 20,094/20,094 | 5,474/5,474 |
| FixedMathSharp Release | 7,011 | 66,877/66,877 | 21,570/21,570 | 5,197/5,197 |
| FixedMathSharp ReleaseLean | 6,990 | 66,970/66,970 | 21,570/21,570 | 5,193/5,193 |

All 49 math recording-adapter cases pass in each profile. Six shared replay
fixtures retain their expectations and agree between Windows x64 Release/Lean.

Warm Release measurements use Windows x64/i7-9700K, .NET 10/SDK 10.0.302, one
radius-0.5/core-length-2 capsule and a 16-unit wall divided into 1/2/16/64 pieces.
Each invocation performs 64 full host steps at 16 Hz, with frictionless crossings
and inward impulses. Mean time/frame over 24 windows, all **0 B/frame**:

| Mover | Continuous wall | 2 pieces | 16 pieces | 64 pieces |
| --- | ---: | ---: | ---: | ---: |
| Upright capsule | 0.161 ms | 0.430 ms | 0.612 ms | 1.609 ms |
| Horizontal capsule | 0.199 ms | 0.684 ms | 1.410 ms | 4.639 ms |

Fresh-pose builds against two members cost **1.375 ms for the concave L** and
**0.884 ms for the oblique height step**, both **0 B/build**. These include pose
refresh and both manifold requests, rather than a full frame or cached contact.

Refinement removes repeated clipping/unnecessary recovery, rejects distant edges
before expensive admission, cancels equal radial coefficients, and accelerates
the existing convexity certificate with a conservative AABB corner; clipped
bounds retain exact vertex proof. The initial geometry prototype cost 4.503 ms
per fresh L build and 6.297/8.828 ms per dense upright/horizontal frame. The
slower dense direct-fan experiment is retained. Angular samples increase the
small prototype's 0.419/0.612 ms costs to the table values; that prototype was
not the accepted ordinary baseline.

Final paired/reversed-order controls use identical drivers, exact witnesses and
24 balanced windows/process, all **0 B/op**. Ordinary upright/horizontal capsule
full steps improve **24.9–32.3%**; capsule/prism controls improve about **43%**,
and joined spherical controls about **28%**. The earlier 20-control matrix is
retained. Box/hull controls are +0.22%/-0.11%; the box paired delta is +0.89 us
against 3.44 us window SD, while the hull contrast changes sign. This establishes
neither a regression nor exact zero overhead; no game-capacity/frame-tail claim
is made.

All six reviewed joined layouts match their continuous-wall controls in pose
and velocities at eight 64-frame checkpoints over 512 additional frames. An
extra prototype comparison differs in horizontal Joined16 Z by -10,485,760 raw
units, with matching velocity/rotation. The reviewed state matches the continuous
control; exact preservation of every prototype state is not claimed. Changed
carrier merging is a plausible explanation, not a confirmed prototype RCA.

A public post-step ray reproduces stale 3D geometry on baseline and candidate;
**#145** records the self-contained pre-existing defect. One intermediate
profiler launch exited 0xC0000005 with a broken trace and is excluded. The same
command then passed, a named profile passed, and three unprofiled 12-second
repeats of that frozen payload passed. No cause or fix is attributed.

`artifacts/static-surface/phase10/` retains Gravitas `*-final-*-e` and math
`math-*-final-*-f` logs/raw coverage, replay, frozen binaries, paired/density
captures, `closeout-audit.json`, representation checks and adverse captures.
Linux/ARM, Unity runtime and released packages were not executed. Push the
FixedMathSharp commit before advancing Gravitas's develop dependency pin.
#133/#139 remain open for Phases 11–14; heightfields/index experiments stay deferred.

### Phase 11 - Cylinders And Cones Against Joined 2D Solids

- [x] Extend the existing exact curved-prism finite owners with exposure before
  analytic/rim selection. Keep full solid admission and ensure any minimum
  certificate accounts for the retained physical domain, not the original prism.
- [x] Complete whole-shape/full-host, mutations, suppliers, coverage, replay,
  zero warm allocation/performance and independent review gates.

**Completion, 2026-10-28:** Physical response requires actual original-
source overlap; a different eligible member may supply the finite witness.
Complete exposed supports certify native minimum candidates. An own containing
parent window suppresses its redundant subset line; a borrowed coplanar proof
remains a native bound, not that line's parent. Each source compares its whole-
source native exit exactly with the maximum of its complete independent finite
cut set, retaining the native alternative on a tie. Published cut normals/depths
also undergo a source-local translation-compatibility check; incompatible rows
prefer the existing native/far alternative. Existing finite-slab active sets and
cone ray events own line maxima and witness materialization. Geometry admission
and depth ranking use exact source charts rather than rounded anchors.

Face windows, lines and supplier certificates remain eagerly available. Exact
edge/fan inventories prepare only for visited response sources and reuse the
existing topology lifetime; pose admission and local fan transforms refresh
each visit. The lazy owner clears interrupted inventories before retrying.

Real-edge strata distinguish closed endpoints from open interiors and use exact
quaternion-transpose incident fans. Finite shared-stratum admission connects
coplanar physical patches before the existing bounded sample reducer; ambiguous
and disconnected patches stay separate. Regression tests cover canonical full-
interval denominators and clipping signed collinear bounds before nonnegative
ratio comparison, plus suppliers, mutation, representations, bodyless/compound
movers and source-family selection. Local/far constraints do not certify global
escape from arbitrary nonconvex unions. Open meshes remain Phase 12;
continuation/queries/CCD remain Phase 13; publication and #133/#139 closeout remain
Phase 14.

**Adverse deep-placement comparison:** All six prism centers have Y=0. Their X/Z
footprints are `[-1,0] x [-2,-1/4]` and `[0,1] x [-2,-1/4]` with half-height
`1/4`; `[-1,1] x [-1/4,2]`, `[-3,-1] x [-2,-1/4]` and
`[1,3] x [-2,-1/4]` with half-height `2`; and `[-1,0] x [-4,-2]` with
half-height `1/8`. A unit-mass cylinder of full height `6`, radius `1/2`, starts
at `(0,0,0)` with local Up rotated to world -X by a +90-degree Z quaternion.
Rotation is frozen, translation is free, materials are frictionless, sleep is
disabled, and gravity/air/damping/minimum speed are zero. Run 32 complete
`Simulate`/`LateSimulate` frames at 16 Hz with no forces. The retained
`CylinderDeepOpposedMemberPlacement` test verifies finite actual-supplier anchors,
positive opposed Z constraints from distinct suppliers/regions and repeated
state/event traces. The residual overlap below is a captured observation, not
an assertion that would require later implementations to preserve the stall.

The frozen pre-phase stack (`Gravitas 9d51836`, `FixedMathSharp f975035`) ends at
raw Q32.32 position `(0,0,10694468568)`, with maximum ordinary penetration
`42949672` raw. The phase's `current-d` capture ends at `(0,0,-929887622)`, with
maximum penetration `2291337850` raw; both velocities are zero. This is an
observable behavior change and a limit of local stabilization. The legacy escape
mechanism has not been established; neither result proves a globally preferred
exit. The isolated wide control removes the side walls, uses lower slabs
`[-4,0]`/`[0,4] x [-2,-1/4]` and front `[-4,4] x [-1/4,2]`, and clears every
ordinary penetration to `PenetrationSlop + 8 raw` after 32 frames. Independent
review accepts this source-local scope boundary. The separately reproduced
pre-existing capsule failure remains [#146](issue-tracker.md#grv-issue-146---joined-mixed-capsule-contacts-can-eject-a-body-along-an-unrelated-axis);
the cylinder comparison is not classified as pre-existing.

Both solution profiles build net10.0/C#14 and netstandard2.1/C#11 with zero
warnings/errors using `UseLocalLsfStack=true`. Final raw coverage:

| Library/profile | Passing core tests | Sequence points | Branches | Methods |
| --- | ---: | ---: | ---: | ---: |
| Gravitas Release | 7,836 | 52,367/52,367 | 20,446/20,446 | 5,508/5,508 |
| Gravitas ReleaseLean | 7,767 | 52,365/52,365 | 20,446/20,446 | 5,507/5,507 |
| FixedMathSharp Release | 7,146 | 67,805/67,805 | 22,250/22,250 | 5,279/5,279 |
| FixedMathSharp ReleaseLean | 7,125 | 67,898/67,898 | 22,250/22,250 | 5,275/5,275 |

All 49 math recording-adapter cases pass in each profile. Six existing shared
replay fixtures retain their expectations and agree between Windows x64
Release/Lean. Independent geometry and ponytail review has no remaining
findings. Runtime growth is **695 net lines in Gravitas** (three new files and
the existing 214-line boundary owner's zero-net rename), and **1,685 net lines
in FixedMathSharp** (nine new files). Counts include comments and exclude tests/docs.
New internal entry points have production consumers; native curved minimum,
finite-slab active sets, cone events, boundary preparation and sample reduction
retain their existing owners. Obsolete cap-edge certificate options and the
redundant far-depth scan were removed.

Warm Release measurements use Windows x64/i7-9700K, SDK 10.0.302/runtime 10.0.10,
one radius-0.5/full-height-2 mover, and a 16-unit wall split into 1/2/16/64
members. At 16 Hz, frictionless inward impulses accompany alternating unit
tangential velocity, with Y translation and rotation frozen. Each invocation
performs 64 complete host steps. Mean milliseconds/frame, all **0 B/frame**:

| Mover | Continuous wall | 2 pieces | 16 pieces | 64 pieces |
| --- | ---: | ---: | ---: | ---: |
| Cylinder | 0.483 | 0.467 | 1.014 | 3.561 |
| Cone | 0.593 | 0.478 | 1.053 | 3.711 |

Continuous/two-piece measurements use 24 windows of four invocations; dense
measurements use 12 windows of one invocation. Fresh pose/contact builds against
both members cost **2.020/1.460 ms for the concave L** and **1.842/1.062 ms for
the oblique height step**, cylinder/cone respectively, all **0 B/build** across
24 windows of 16 invocations. These build measurements include pose refresh and
both manifold requests, rather than a solver frame or stationary cached result.
The committed `JoinedStaticSolidMixedSimulationBenchmarks` and
`JoinedStaticRoundedMixedContactBenchmarks` define the fixtures.

The initial validated 64-piece costs were **8.887/9.145 ms/frame**. A sampled
cylinder full-step profile attributed about 67% to eager edge/fan preparation
when the nearby roster changed. Preparing only visited source inventories
reduces the final dense costs by about **60%**, preserving all supplier faces
and certificates. Native line LP fields also retain their proven 64-word banks.
First activation can allocate cold buffers; warmed motion and roster changes
were measured at zero allocation. These are single-mover mean costs, not
many-body capacity or gameplay-frame tail-latency budgets.

Eighty refinement comparisons retain complete body states or every finite
contact's identity, normal, depth and both anchors. Forty-eight additional
checkpoints match all 2/16/64-piece layouts to their continuous-wall controls
over 512 further frames per layout. The matched deep-placement comparison also
preserves the previously captured cylinder and capsule outcomes; the test does
not require later implementations to preserve residual overlap.

Seven ordinary query/contact/full-step controls use identical drivers,
authoritative states before/after timing, 24 balanced windows and both load
orders, all **0 B/op**. Final contrasts range from -2.13% to +2.61%; the largest
positive sphere contrast changes sign in the opposite order. A bounded focused
repeat measures that sphere at -0.34%/-0.70%, while identical-payload controls
vary +0.78%/-1.19%. The earlier seven-control matrix is retained as well.
No persistent ordinary regression is established at this resolution; exact
zero overhead is not claimed.

`artifacts/static-surface/phase11/` retains math `math-*-final-*-f`, Gravitas
`*-final-*-g`, raw coverage, replay, baseline/source/module hashes, final `*-h`
timings and state checks, the dense profile and adverse captures. Baselines are
Gravitas `9d51836` and FixedMathSharp `f975035`; frozen matched-driver payloads
verify executed dependencies. Reproduction uses the named benchmark fixtures
and both core test projects in Release/ReleaseLean with
`-p:UseLocalLsfStack=true`. Linux/ARM, Unity runtime and released packages were
not executed. Push FixedMathSharp before advancing Gravitas's develop pin; the
user's existing pin edit is preserved. Resolve #146 before Phase 12; #144/#145
remain independent. #133/#139 remain coordinated through Phases 12–14.

### Phase 12 - Open Mesh Movers Against Joined 2D Solids

- [ ] Cover moving open/concave mesh patches through the existing native patch
  owners. Preserve independent disconnected regions and actual finite suppliers.
- [ ] Complete whole-shape/full-host, mutations, suppliers, coverage, replay,
  zero warm allocation/performance and independent review gates.

### Phase 13 - Continuation, Queries And Continuous Collision

- [ ] Extend existing dependency invalidation to affected neighbors for geometry,
  filtering, trigger state, removal and membership changes. Test sleeping contact
  and query-grounded bodies, including expose-a-rim transitions without a local
  pose change. Verify the current step uses one coherent committed topology.
- [ ] Audit 3D sweep/initial-overlap and mixed query/CCD certification, and 2D CCD
  against the same physical domain. Cover fast seam crossing, rotation, retained
  tokens and true-edge impact; keep per-collider query results distinct.
  A horizontal 3D floor cannot prove planar mixed sliding parity.
- [ ] Complete authoritative membership hashing, explicit recording/restore and
  host reconstruction, including unloaded/replaced members and context reuse.
  Settle supplier identity and solver-group ordering across reconstructed registry
  slots; normalized hash references alone do not establish equivalent ordering.
  Extend shared native replay conformance with reviewed expectations; do not
  weaken existing unrelated fixtures to accommodate a seam correction.

### Phase 14 - Close #133/#139 And Publish The Supported Contract

- [ ] Validate Release/ReleaseLean, raw reachable 100% line/branch/method coverage,
  relevant replay lanes and warmed allocation guards using `UseLocalLsfStack=true`.
  Coordinate any upstream math change and preserve its same coverage guarantees.
- [ ] Run repeated, bound seam/interior and unaffected 3D/2D/mixed/query controls.
  Require zero added warmed allocations and no ordinary-path regression beyond
  measured uncertainty against the accepted baseline, including the explicitly
  accepted Phase 6 correctness cost. Broken seam response is not a speedup denominator;
  document the corrected-work cost and refine introduced overhead before closeout.
- [ ] Obtain independent geometry and ponytail reviews of the complete changes,
  including deletion/reuse opportunities and every coordinated sibling change.
- [ ] Update evergreen host/collision/dimensional/serialization guidance with the
  implemented membership contract, exact joining requirements and finite edges.
  Close each issue only after its dimensional behavior is covered. Leave
  heightfields and optional index experiments deferred for a separate decision.

## Later Terrain And Heightfield Gates

1. **Reuse the completed foundation.** Close the generic continuity issues and
   reuse their membership, geometry, lifecycle and conformance contracts. Do not
   redesign border identity, query/event ownership or persistence for terrain.
   Select member-level BVH experiments only if existing spatial lookup leaves a
   measured workload gap, using the comparison requirements above.
2. **Optional heightfield decision.** Height-driven navigation and mesh physics
   remain independently supported host choices. If a physics heightfield or
   shared-data workflow is needed, evaluate the existing `Trailblazer.Gravitas`
   adapter and a narrow authored-sample-to-mesh path before a specialized cell
   backend. Measure preparation, memory, rebuild/streaming and warmed contact
   costs with identical physical surfaces. No backend is selected in advance;
   no core-library coupling, shared mode switch or duplicate terrain math is
   required by the independent profiles.
3. **Representation conformance.** Reuse Phase 14's correctness, determinism,
   coverage and cost gates for any selected representation. Add explicit sample
   quantization/diagonal agreement, streaming/edit churn and adapter validation
   only where that representation introduces them. Publish its supported
   contract after implementation; released packages remain the release gate.

## Boundaries

No engine-specific terrain API, renderer/LOD system, terrain editor, automatic
global welding or new spatial-index framework is authorized by this proposal.
Do not silently smooth real features or introduce a height-only substitute for
rigid-body contacts. Record experiments and phase results here; keep one living
design/plan rather than separate phase reports. Record reviewed contract changes
and implementation results here as the generic prerequisites are resolved.

## Reference Context

Other engines are evidence about representations and known failure modes, not
the specification for Gravitas:

- [Unity TerrainCollider](https://docs.unity.com/en-us/engine/6000.3/manual/physics-section/physics-overview/collision-section/collider-shapes/terrain-colliders/introduction)
  derives physics collision geometry from height data.
- [Godot HeightMapShape3D](https://docs.godotengine.org/en/stable/classes/class_heightmapshape3d.html)
  is a collision shape with two triangles per grid cell.
- [Godot/Jolt edge handling](https://docs.godotengine.org/en/stable/tutorials/physics/using_jolt_physics.html#ghost-collisions)
  documents internal-edge mitigation and its boundary across separate bodies.
