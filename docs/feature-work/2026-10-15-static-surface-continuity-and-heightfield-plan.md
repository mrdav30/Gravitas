# Static Surface Continuity And Heightfield Integration

**Date:** 2026-10-15  
**Status:** Phases 1–2 complete; Phase 3 (other 3D shape families) next, 2026-10-17
#133/#139 remain coordinated. Heightfield and alternative-index work are deferred;
no public continuity API is enabled. Internal 3D rounded response is complete;
other shape families, dimensions and continuation remain in the phases below.  
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
before handoff and obtain independent ponytail review. Phase 7 consolidates the
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
A separate pre-existing single/query witness defect is captured as #140; this
proof uses the correct finite manifold path.

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
reclipping and dimensional-owner failure handling remain Phase 4 consumers,
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
pools and dispatch remain Phase 3/5 integration work.

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

Phases 2–5 consume the bounded proofs through existing owners and delete
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

### Phase 3 - Complete Other 3D Shape Families

- [ ] Integrate cone joined face pools and genuine clipped-boundary events in the
  existing cone owners. Boundary-witness proofs alone do not qualify the face
  reducer or complete-shape response.
- [ ] Establish cuboid, cylinder, convex/concave mesh and non-rounded compound
  behavior against declared surfaces. Reuse the native convex SAT/finite-feature
  producer and existing family reducers; retire superseded local-only branches
  where the shared owner can cover both cases.
- [ ] Verify complete-shape clearance, finite anchors/materials, genuine ends,
  holes, ambiguous/folded joins, free rotation and order reversal. Maintain
  ordinary fast paths, 0 B warm allocation and practical moving-contact costs.
- [ ] Complete affected Release/Lean coverage, replay, allocation/performance and
  independent review gates before closing this phase.

### Phase 4 - Complete Pure 2D Static Continuity

- [ ] Implement typed 2D membership through existing services/lifecycle owners;
  retain the same ownership, transactional replacement and lifetime guarantees.
- [ ] Implement #139's dimensional boundary reduction without projecting 3D
  sheets into 2D volumes. Test circles, zero/nonzero-core capsules and polygonal
  bodies against connected convex pieces and genuine concave corners.
- [ ] Reclip finite incident features against exact exposed spans before final
  endpoint materialization. Preserve independent normal groups, supplier
  materials and overlap truth separately from response rows; no global two-row cap.
- [ ] Certify full recovery against the final pose's complete eligible spatial
  candidates and wire the dimensional deep-placement failure contract. Triggers,
  query-only and immovable observations must not require physical recovery.
- [ ] Verify full-loop baseline crossings, free rotation, real boundaries, order,
  membership/filter changes and density/scaling; complete coverage, replay,
  allocation/performance and independent review gates before closing this phase.

### Phase 5 - Complete Mixed Static Continuity

- [ ] Apply the same ownership/eligibility contract to mixed finite slabs in both
  directions. Complete joined face pools through the existing mixed slab support
  and clipped-boundary owners, including a vertical 3D sheet with a planar
  response normal.
- [ ] Verify connected 2D pieces as embedded targets with matching/differing host
  Y and thickness. Planar adjacency must not erase a real step or finite rim.
- [ ] Preserve per-member events/query identity, deterministic solver ordering,
  materials and lifetime invalidation; cover actual full-loop crossings and
  complete-shape response before the same coverage/performance/review closeout.

### Phase 6 - Continuation, Queries And Continuous Collision

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
  Extend shared native replay conformance with reviewed expectations; do not
  weaken existing unrelated fixtures to accommodate a seam correction.

### Phase 7 - Close #133/#139 And Publish The Supported Contract

- [ ] Validate Release/ReleaseLean, raw reachable 100% line/branch/method coverage,
  relevant replay lanes and warmed allocation guards using `UseLocalLsfStack=true`.
  Coordinate any upstream math change and preserve its same coverage guarantees.
- [ ] Run repeated, bound seam/interior and unaffected 3D/2D/mixed/query controls.
  Require zero added warmed allocations and no ordinary-path regression beyond
  measured uncertainty. Broken seam response is not a speedup denominator;
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
3. **Representation conformance.** Reuse Phase 7's correctness, determinism,
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
