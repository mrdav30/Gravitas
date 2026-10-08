# Surface Contact Manifold Design

**Status:** Approved architecture; runtime implementation has not started.
The [implementation plan](2026-10-08-surface-contact-manifold-plan.md) maps its
dependencies and validation gates. **Date:** 2026-10-08. **Owning issues:**
[#095](issue-tracker.md#grv-issue-095---discrete-mesh-cone-contacts-can-choose-an-artificial-triangulation-seam-exit),
#099; #097 and #098 are related follow-ups.

## Intended Outcome

Generate deterministic rigid-body contacts against the actual finite mesh
surface. Covered triangle seams must not become constraints, genuine boundaries
and independent surfaces must survive, and admitted anchors must remain accurate
before final fixed-point rounding. Preserve low allocation costs and practical
scaling for single-player, strategy and lockstep server workloads.

The owner has delegated the contact-model decision to engineering judgment:
choose correctness, determinism and physical coherence before preserving the
smallest implementation. Maintain exact reachable line, branch and method
coverage in both build profiles, including coordinated upstream changes.

## Recommendation

Use topology-aware surface manifolds for runtime response. Retain minimum-exit
geometry as a distinct operation; do not require one global escape vector to
represent every impulse and torque constraint of a concave surface union.

The committed exact convex solver and face certificates remain useful geometric
owners. Their result can be reused when it satisfies the surface-contact
contract. Convexity alone must not silently select different physical semantics
from an otherwise equivalent surface with a small concave tab.

This is a larger change than adding another sufficient face proof. It affects
contact generation, storage, warm starts, response and observable manifold data.
Its implementation needs a reviewed plan before changing those contracts.

## Evidence And Alternatives

The square-plus-tab reproduction in #095 still selects a covered seam. Exact
convex-subset certificates could resolve this particular fixture, but they are
sufficient proofs, not complete nonconvex contact generation.

A local-stack Release probe also places a unit-height, radius-1/2 cone at
`(3/10, 0, 3/10)` against two finite orthogonal walls. With both walls in one
concave mesh, detection retains one wall normal and full-loop response leaves
the other incoming velocity component unchanged. Separate static wall colliders
stop both components. Both windings are exercised. The probe uses explicit
static bodies for response; bodyless colliders are not response participants.
The ignored source and log are `artifacts/grv-issue-095/MeshConeSurfacePolicyAudit.cs`
and `surface-policy-audit.log`; the temporary test is removed after execution.

A complete global minimum exit would instead solve the boundary of a union of
configuration obstacles. The closest exposed point can occur where component
boundaries intersect, rather than at any independently minimized triangle or
perimeter feature. A complete solution would need joint feature events, exposure
tests, exact ranking and new width/root-degree proofs. That is a useful possible
depenetration query, but it does not retain the independent locations and normals
needed by rigid-body response. Do not build that arrangement solver merely to
preserve the current cone reducer.

Dropping final seam winners is also insufficient: the discarded triangle can
still have a genuine face or perimeter intersection. Its replacement must be
generated from admitted geometry, rather than fabricated by changing the normal.

The distinction is consistent with primary references: PhysX separates minimum
translation queries from contact generation, and Catto explains why adjacent
feature context is needed to eliminate ghost contacts. Their algorithms are
reference material, not implementations to copy. In particular, Box2D's
one-sided ghost-vertex rules cannot establish our two-sided surface contract.

- [PhysX geometry queries](https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/docs/GeometryQueries.html)
- [Box2D ghost collisions](https://box2d.org/posts/2020/06/ghost-collisions/)

## Geometry Contract

1. A committed coplanar patch represents its exact filled authored domain,
   including exposed perimeter, holes and notches. Reuse `PhysicsMesh` welding,
   seam classification and atomic scale preparation. Ambiguous topology does
   not authorize deleting a feature.
2. Contact admission operates on exact geometry. Rounded coordinates and
   normals cannot be fed back into predicates to establish membership or reject
   a candidate. Materialize anchors, normal and directional depth only after
   selecting the admitted feature.
3. A face witness must lie on the actual finite domain. Global cone support
   projections are fast cases, not a complete face generator. A long tilted cone
   can intersect wholly inside a patch while its projected apex and base extrema
   are outside the patch and no perimeter edge intersects the cone.
4. The proposed general face path clips the triangle union against the cone's plane
   section and derives matched cone-boundary rays through admitted face points.
   Reuse the exact finite-intersection and axial-extremum machinery in
   `WideTriangleConeIntersection`; retain its unrounded admission information.
   Face sampling must include the finite side, base and rim regions. Reduce
   over the geometric contact region, not over authored triangle count. This
   is an algorithm to establish, not an existing complete sampler: prove its
   finite admission, sample coverage and complexity before calling it complete.
5. Exposed edges require independent finite-feature admission: exact segment
   parameters, true boundary ownership and matched cone support-region terms.
   Existing generator slices and stationary rim charts provide arithmetic to
   reuse. A support gap alone does not prove that its affine witness lies on
   the finite edge.
6. Corner ownership follows the actual local domain. Strict convex corner fans
   are not valid at reflex vertices. Preserve the real incident edge/face
   constraints of notches and holes without inventing a convex ear.
7. Normals remain two-sided and geometrically selected, independent of motion
   direction and authored winding. Each emitted contact retains its own paired
   canonical anchors, normal, directional depth and clamp state.
8. Contact geometry and minimum-exit geometry have distinct tests. A shorter
   possible future escape through an edge is not, by itself, proof of a current
   edge constraint. Existing minimum-exit regressions must stay meaningful in
   the math owner; runtime behavior changes require explicit physical tests.

Choose one side for a continuous coplanar face-contact region before reducing
its samples. For each orientation of the canonical plane normal, compare the
maximum cone-boundary ray distance over that exact admitted region; choose the
orientation with the smaller maximum. Compare unrounded values, and resolve an
exact tie with a documented canonical plane-normal convention. Do not choose
the shorter ray independently at each point: that can produce opposing positive-
depth corrections on the same crossing region. These are local directional
depths, not a certificate of a global union exit. The implementation plan must
prove the extrema construction and include a tilted crossing with varying chord
midpoints. Exact symmetric ties follow the stated convention; do not promise
equivariance under every rigid symmetry when geometry supplies no unique side.

Exact clipping and finite witness admission belong in FixedMathSharp. Surface
ownership, grouping, reduction and response policy belong in Gravitas. Preserve
the existing one-way internal friendship boundary and coordinated release order;
do not expose wide types publicly or add a downstream wide-arithmetic facade.

## Contact Storage And Reduction

Represent independent geometric surfaces/contact regions separately from the
small point set that samples one surface. Reuse the existing four-point storage
where it remains appropriate for an individual group. Four total points for an
entire pair are not a sufficient contract for arbitrary simultaneously contacted
mesh surfaces.

- Retain independent normals and contact regions across the pair. Reduce
  redundant samples within their owning group; do not allow four deep duplicate
  samples to evict a shallower independent wall constraint.
- Derive groups from actual admitted regions and surface/part provenance,
  including material boundaries. Disconnected same-normal regions need their
  own spatial coverage. Connectivity or a tiny connecting tab must not change
  response weighting solely by changing the number of groups. Define grouping
  and weighting together; neither triangle count nor group count is pressure.
- Within a group, use deterministic geometric coverage to preserve spatial
  extent and angular leverage. A bounded point set is an explicit solver
  approximation; it does not claim exact preservation of a continuous pressure
  distribution or every point of an arbitrary intersection curve.
- Order groups and points by canonical geometric provenance and admitted local
  features. Preserve deterministic ties. Authored triangle indices can identify
  preparation work, but must not make equivalent triangulation produce different
  physical constraints or weighting.
- Keep pair-owned contacts and warm starts retained and pooled. Reserve scratch
  and contact capacity from registered/prepared geometry; avoid a new collection
  allocation per feature or frame. Measure registration cost and retained memory
  as well as warmed bytes per operation.
- Migrate `SolverContactBuffer`, warm-start storage and fixed-width response
  failure masks together. Allowing a larger manifold while those owners still
  overwrite the fourth slot or use a byte bitmask is not a valid intermediate
  implementation.
- Preserve group provenance, material overrides and reversed anchor/normal
  ownership through compound scratch manifolds and contact transfer. Retire or
  rebuild grouped caches on reconfiguration, scale preparation, mass mutation,
  population, pair pooling and context reset. Enter/stay/exit notifications
  remain once per collider pair.

`ContactManifold.MaxContactCount`, count semantics and enumeration are currently
public. A four-points-per-surface contract must use explicit naming; do not
silently reinterpret the existing total-capacity constant. Document necessary
public changes and their release implications before implementation. Do not add
a legacy mode that keeps known missing constraints merely for compatibility.

## Response Contract

Independent surfaces are independent constraints. Their impulses and correction
must not shrink automatically because another surface was added to the pair.
Equivalent triangulation and duplicate samples must not multiply response either.

Keep solving order explicit. Evaluate each independent constraint against the
current authoritative velocity, apply its admitted impulse, then continue in
stable order; preserve accumulated nonnegative normal impulses and the existing
Coulomb bound. Audit warm-start application and position correction separately.
Blindly removing `contactShare` from the present batch calculation would permit
duplicate samples to over-apply impulses, so geometry reduction and response
scheduling must change together.

Audit both one-pair shortcuts in `GravitasPhysicsService.Response`: the queued
pair path and the single-contact island path currently solve only once. A pair
with multiple independent surface groups must receive the intended solver
iterations; preserve the genuine one-surface fast path. The probe demonstrates
an omitted constraint, not a standalone defect in `contactShare`.

Use one explained correction policy for equivalent same-normal surface samples,
while preserving correction against independent normals. Keep contact slop,
mobility constraints, mass/inertia ownership, representability preflights and
failure handling explicit. Do not use a position tweak to hide omitted contacts.

Grounding consumes actual upward support normals and admitted witnesses. CCD
retains its closing-contact and earliest-impact contract; minimum depenetration
and initial-contact classification must not be conflated with time of impact.
Normal generation must remain geometry-based when velocities change sign.
Audit singular consumers such as rotational CCD's `PrimaryContact` selection
against their own impact/closing-contact purpose rather than passing an arbitrary
deepest group witness into that calculation.

Manifold and warm-start data participate in the mandatory authoritative replay
hash, not only its optional cache mode. Version the affected hash sections and
include group ownership, ordering and authoritative weighting state. Body saves
continue excluding runtime contact caches; populate-existing tests must show
stale impulses clear for every group without implying cache restoration.

## Implementation Boundaries

The detailed implementation plan follows review of this design. Its dependency
order is:

1. Establish the contact-group/storage and response contract with the two-wall
   regression and redundant-sample controls. Cover all fixed-capacity consumers
   plus compound transfer, replay schemas, cache lifecycle and both solver
   dispatch shortcuts; preserve the existing one-surface hot path.
2. Add exact finite cone/patch witness generation, starting with the tab blocker,
   holes and the long tilted interior. Keep face certificates as proven fast
   paths; complete clipping supplies cases where they decline.
3. Integrate cone mesh surface groups, update runtime contact semantics and
   validate full-loop support, sliding and torque. Close #095 and #099 only after
   their geometry and response boundaries pass.
4. Reuse proven topology/admission principles for #097 sphere/capsule contacts
   and #098 mixed curved slabs. Preserve explicit pure-2D and mixed response
   constraints; do not turn either into accidental 3D projection.

Pure 2D keeps its existing API unless shared scheduling changes require a
deliberate migration; verify that boundary. Mixed currently stores one
`MixedContact` and has separate response and replay owners. Its follow-up needs
an explicit storage/response migration, not an implicit reuse of the 3D buffer.
Preserve planar 2D impulses, 3D-only vertical response, zero planar coupling for
vertical normals, slab thickness, part materials and pair-level events.

The following are outside this design: a general exact nonconvex minimum-
translation API, extreme-size support decisions in #096, a replacement broad
phase, new engine adapters, and unrelated global solver/quality refactors.

## Acceptance And Evidence

- Square plus tab, ring/hole, U-shaped notch, disconnected contact regions and
  connected orthogonal walls; each must preserve real geometry and reject seams.
- Small convex patches, exposed edges/corners, long tilted cones with support
  extrema outside the finite patch, and cap/rim/generator tangencies.
- Both windings, alternate diagonals, collinear subdivisions, welded duplicate
  vertices, rigid transforms, raw-neighbor touch/gap and scale preparation.
- Frictionless seam sliding, independent-wall approach, separated support torque,
  sleep/wake, grounding, warm starts and applicable CCD direction changes.
- Equivalent same-normal samples must not change linear response merely by
  changing their count. Independent normals must remain effective as count grows
  beyond the old four-point pair limit. Test the documented angular reduction
  approximation rather than claiming arbitrary exact constraint preservation.
- Repeated full-loop replay traces and existing cross-platform conformance.
  Update reviewed expectations only for an explained intended behavior change.
- `UseLocalLsfStack=true` for coordinated development, Release/ReleaseLean full
  suites and exact reachable line/branch/method coverage in every changed owner.
  Release upstream first and validate released packages before promotion.
- Benchmark one-surface interiors, true boundaries, multiple surface groups,
  subdivision scaling and the nine original triangle-contact controls. Retain
  warmed zero-allocation gates and measure startup/retained-memory growth.

Keep validation notes at the end of the corresponding implementation phase in
its eventual plan; do not create separate phase-report documents. Wiki and
complexity records describe lasting contracts without referring to this design
or its artifacts. Keep changes unstaged and uncommitted for owner review.
