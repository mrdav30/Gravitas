# Surface Contact Manifold Implementation Plan

> **For agentic workers:** Use `superpowers:executing-plans` for implementation
> in this checkout, with independent subagent reviews at the phase boundaries.
> Include a dedicated ponytail reviewer of the complete unstaged change in each
> handoff, covering new files and coordinated sibling changes.
> Steps use checkboxes. Leave every change unstaged and uncommitted for owner
> review; do not create another checkout or commit a phase automatically.

**Goal:** Close #095 and #099 by admitting actual finite mesh/cone surface
contacts and retaining independent constraints through deterministic response.

**Architecture:** Evolve the existing pair-owned manifold into bounded point
groups with retained overflow. Stream sequential response through the existing
exact kernels. FixedMathSharp supplies unrounded finite-feature certificates;
Gravitas owns prepared topology, contact-region grouping and physical policy.

**Tech stack:** C# 11, `netstandard2.1`/`net8.0`, FixedMathSharp,
SwiftCollections, GridForge, Chronicler.Hashing, xUnit v3, BenchmarkDotNet.

**Design authority:** The contracts and rationale below are part of this plan;
implementation decisions and validation stay in this single document.

**Status:** Phase 1 is complete with full validation and independent review.
Phase 2 is complete with full validation and independent review. Phase 3 integration is in progress; phase 4 remains planned. #095/#099 stay active; #097/#098 remain separate
follow-ups.

## Design Contracts And Rationale

### Evidence And Alternatives

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

### Geometry Contract

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
   Reuse the exact finite-intersection predicates and relative-frame owners.
   `WideTriangleConeIntersection`'s existing rounded axial minimum is a separate
   query contract; it cannot establish a complete plane-ray maximum.
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

For each actual exposed edge or corner, its supported cone branches are
alternative exits of that one feature. After exact incident-fan and selected-face
hemisphere admission, retain the minimum unrounded feature depth and all exact
ties, including continuous families. Reduce only that co-minimal finite sample
pool. Different exposed features and face regions remain independent constraints;
do not minimize across the mesh. Promoting every alternate exit would turn a
generator tangency into a deep correction toward the distant apex and would
remain discontinuous at a one-raw inward neighbor. Phase 3 physical regressions
cover both cases and stationary full-loop touch.

Exact clipping and finite witness admission belong in FixedMathSharp. Surface
ownership, grouping, reduction and response policy belong in Gravitas. Preserve
the existing one-way internal friendship boundary and coordinated release order;
do not expose wide types publicly or add a downstream wide-arithmetic facade.

### Contact Storage And Reduction

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

`ContactManifold.MaxContactCount` formerly described pair-wide capacity.
Phase 1 replaces it with
`MaxContactsPerGroup`; preserve explicit naming and the documented group/point
inspection contract. Do not add
a legacy mode that keeps known missing constraints merely for compatibility.

### Response Contract

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

Both one-pair shortcuts in `GravitasPhysicsService.Response` reuse the
budget-aware public response wrapper established in phase 1. A pair
with multiple independent surface groups must receive the intended solver
iterations. A single contact row keeps the one-sweep fast path; even one
geometric group can contain coupled angular constraints that need iteration. The probe demonstrates
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

### Implementation Boundaries

Implementation proceeds in this dependency order:

1. Establish the contact-group/storage and response contract with the two-wall
   regression and redundant-sample controls. Cover all fixed-capacity consumers
   plus compound transfer, replay schemas, cache lifecycle and both solver
   dispatch shortcuts; retain the isolated-pair dispatch shortcut while applying
   the configured budget to every multi-point manifold.
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


## Global Constraints

- Deterministic fixed-point runtime math; no floating-point geometry predicates.
- Four points per geometric group are a documented angular/pressure
  approximation. Neither triangle count nor group count is pressure weight.
- Exact admission and comparison precede final rounding. No rounded witness
  may establish finite-feature membership or geometric connectivity.
- Preserve the one-way FixedMathSharp internal friendship boundary and reuse
  existing arithmetic/root owners; no downstream wide facade or copied limbs.
- Use `UseLocalLsfStack=true`, including child builds. Validate `Release` and
  `ReleaseLean`, with exact 100% reachable line, branch and fully covered methods
  in every changed repository. Release upstream before package validation.
- Retain warmed zero-allocation gates. Measure cold pair creation, new capacity
  high-water marks and retained memory separately; the existing pair pool does
  not guarantee allocation-free first-ever contact.
- Preserve pure-2D and mixed owners. Test their boundaries; do not expand their
  contact storage merely for symmetry.
- Explain complex invariants in source comments. Main partial files retain
  their `<content>` XML documentation. Wiki content stays evergreen.
- Serialize .NET/benchmark/DocFX jobs, use two .NET processors and serial
  MSBuild. Put phase summaries here, not in separate reporting documents.

## Review Focus

1. A remote tab changes a mesh centroid: an admitted local normal must not flip
   in response. Covered in phase 1 response and phase 3 tab tests.
2. A hole or notch splits a cone intersection into disconnected regions: choose
   sides per actual region, with exact edge/vertex tangency connectivity.
   Covered in phase 2 topology and phase 3 integration.
3. A sample beyond the eighth flattened contact fails representability: reject
   its response and clear its impulse without corrupting other constraints.
   Covered in phase 1 response and warm-start tests.
4. Compound reversal or a material boundary shares similar anchors: preserve
   group provenance, materials and pair-level notifications. Covered in phase 1.
5. A long tilted cone crosses inside a face while global support projections
   are outside: complete clipping and side/base branch switches must supply
   admitted witnesses. Covered in phase 2 exact geometry and phase 3 physics.

## Baseline And Execution

Work in the current checkout. Implement sequentially; independent reviewers
inspect each completed phase without running competing jobs or editing files.
Capture baseline benchmark results before each affected hot path changes. The
previous minimum-exit benchmark results are context, not measurements of the
new physical contact contract.

```powershell
$env:UseLocalLsfStack = 'true'
$env:DOTNET_PROCESSOR_COUNT = '2'
dotnet restore Gravitas.slnx -p:UseLocalLsfStack=true -p:BuildInParallel=false -m:1
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj -c Release -p:UseLocalLsfStack=true -p:BuildInParallel=false -m:1
```

### Preparation Summary — 2026-10-08

- [x] Confirmed clean committed starting points: Gravitas `bf80a82`,
  FixedMathSharp `517553a`.
- [x] Audited storage, response, topology and exact-geometry owners, including
  independent read-only reviews. The four-slot solver buffer and byte failure
  mask can be removed with sequential response instead of generalized.
- [x] Ran 85 maintained manifold, warm-start, response, cone, compound and
  replay-hash tests with the local stack in Release; all passed. Log:
  ignored `artifacts/grv-issue-095/surface-plan-baseline.log`.
- [x] Retained the earlier six-case two-wall probe as diagnostic evidence:
  four combined-mesh failures and two separate-static-wall controls. No
  experimental source remains in the maintained test project.

## Phase 1 — Group Storage And Independent Response

### Task 1: Migrate manifold, cache, compound transfer and replay atomically

**Owners:**
`src/Gravitas/CollisionHandling/Contacts/3D/ContactManifold.cs`,
`ManifoldContact.cs`, `ContactWarmStartCache.cs` in that directory;
`src/Gravitas/CollisionHandling/Pairs/3D/CollisionPair.cs` and
`CollisionPair.ReplayHash.cs`;
`src/Gravitas/CollisionHandling/Detection/3D/CollisionDetection.Compound.cs`,
`CollisionDetection.Cylinder.cs`, and `Context/CollisionSatScratch.cs`.
Create focused `ContactGroup.cs` and `ContactGroupKey.cs` beside the existing
3D contact owners only where the inline value storage/key requires them.

**Interfaces:** Replace public `MaxContactCount` with
`MaxContactsPerGroup = 4`; preserve flattened `Count`, indexer, enumeration,
`PrimaryContact`, `BeginUpdate(long)` and `Reset()`. Add public
`int GroupCount`, `int GetGroupStartIndex(int groupIndex)` and
`int GetGroupContactCount(int groupIndex)` for inspection. Internal
`ContactGroupKey` compares the full tuple of A/B compound namespace, A/B
prepared surface ordinal and admitted-region ordinal; zero is the default
primitive group. Add `AddContact(in ContactGroupKey group, in ManifoldContact
contact)` and `ReserveGroups(int capacity)` internally. Existing contact
overloads populate the default group. Match caches using full group provenance
plus existing contact features, not a new public hash-only surface ID.

- [x] Add failing production-behavior regressions in focused `ContactGroup*Tests.cs`:
  nine independent groups survive; deep same-group samples cannot evict a
  shallower independent group; insertion permutations expose equal groups and
  points; group/flattened indexing and reset/reuse agree.
- [x] Run the focused tests and record the expected capacity failure before
  implementation. Tests needing the new API must compile before their red run;
  do not count a missing-member compiler error as the reproduction.
- [x] Move the existing four slots into an inline first group and retain a
  `SwiftList<ContactGroup>` only for overflow. Enumerate group-major, then
  canonical local features, without repeated linear flattened lookups. Keep
  reduction bounded within each group and collision-free group equality.
- [x] Retain the existing four-entry warm-start value within each cache group;
  extend `StoreWarmStartImpulse`, `TryGetWarmStartImpulse` and
  `RemoveWarmStartImpulse` with `in ContactGroupKey`. Prune absent points/groups
  after regeneration. Clear logical caches on reset, pooling, mass mutation,
  population and reconfiguration while retaining reusable capacity.
- [x] Transfer groups through compound scratch, remapping A/B namespaces and
  reversing provenance with anchors/normals. Keep material regions distinct and
  notifications once per collider pair. Rename cylinder's local four-sample
  bound to `MaxContactsPerGroup`.
- [x] Version `manifold.3d` from 7 to 8 and `warm-start.3d` from 1 to 2; hash
  canonical group structure and point/cache order in the existing mandatory
  authoritative contributor. Update the pair section only if its direct
  serialized layout changes. Keep body saves free of contact caches.
- [x] Extend `CollisionWarmStartTests.cs`, `CompoundColliderCollisionTests.cs`
  and replay-hash tests for nine groups, disappearance, normal compatibility,
  reversed compounds/materials, lifecycle invalidation and all cached impulses.
  Run their focused suites, then full Release/Lean and exact coverage gates.

### Task 2: Stream sequential response and fix multi-group scheduling

**Owners:** `src/Gravitas/CollisionHandling/Response/3D/CollisionResponse.cs`,
`SolverContactBuffer.cs`; `src/Gravitas/Core/3D/GravitasPhysicsService.Response.cs`.
Reuse `SolverContact`, `ContactNormalImpulse3D` and exact response kernels.

**Interfaces:** Preserve `CalculateImpulse(CollisionPair, bool
applyCachedImpulse, bool applyPositionCorrection)` and the public wrapper.
Consume task 1 groups and caches. `TryCreateContact(...)` consumes a group/local
point rather than repeatedly scanning flattened indices. Remove the solver
buffer and index-mask failure bookkeeping once all consumers stream rows.

- [x] Add red tests in focused `ContactGroup*Tests.cs` and existing response/island
  suites:
  orthogonal admitted normals stop both incoming components; duplicate
  same-normal samples with frozen rotation do not multiply linear response;
  disconnected supports retain torque; failure beyond contact index seven
  clears only that row; supplied normals survive misleading collider centers.
- [x] Apply compatible warm starts in a separate deterministic pass before
  solving; a row must not read a cache already overwritten by another row.
  Preserve response-center/position snapshots for exact lever ownership.
- [x] Solve and apply each accumulated normal delta against current velocity,
  using share one, then Coulomb friction against that row's accumulated normal
  impulse. Preserve normal nonnegativity, restitution threshold and exact
  representability preflights. A failed row clears its cache and skips its
  remaining response; other rows continue without a bitmask.
- [x] Correct position once per identical canonical correction direction,
  choosing its deepest admitted depth with deterministic ties. Preserve slop
  and the existing correction fraction; do not divide by unrelated point/group
  count. Keep separate same-normal lever arms for velocity solving. Test
  near-parallel/opposing directions and rotated inertia; this is a documented
  translation correction policy, not exact multi-contact depenetration.
- [x] Preserve admitted nonzero normals in `ResolveContactNormal`; collider
  centers are fallback only for genuinely zero legacy normals. Audit other
  producers before changing the shared consumer; correct inverted producer
  contracts at their source rather than retaining a concave-normal flip.
- [x] Make the public response wrapper apply configured
  `DiscreteSolverIterations` to multi-point manifolds. Both queued-one-pair and
  single-contact-island shortcuts reuse that wrapper, retaining isolated-pair
  dispatch without bypassing the iteration budget. Keep the one-row shortcut
  inside the wrapper. Verify iteration effects through physics state, not
  test-only counters. Delete unused `SolverContactBuffer` and mask helpers.
- [x] Run response, exact-lever, warm-start, joint/island and existing 2D/mixed
  parity controls, full suites/coverage and one-surface response benchmarks.

**Phase 1 summary - 2026-10-08:** Group storage and independent response are
implemented; #095/#099 remain active until finite-surface admission and
full-loop integration pass later phases.

- Regressions retain nine groups/36 points, canonical insertion order, compound
  reversal/materials, pair-level events, independent walls, disconnected torque,
  rotated inertia and failed rows beyond index seven. Regeneration prunes vanished
  caches even when response is skipped. Review caught and verified fixes for this
  pruning boundary and the clamped-depth reduction tie; no actionable findings remain.
- Frozen incoming restitution targets survive warm starts and later iterations
  through compact/exact-lever paths and shared-body islands. Contained cone/sphere
  and cylinder/sphere producers now supply outward normals.
- Replay versions are `manifold.3d` 8 and `warm-start.3d` 2; `pair.3d` remains 3.
  Four fixtures change hashes only; recorded body states, events and queries stay
  unchanged, as do pure 2D expectations. Evidence is local Windows x64
  Release/Lean; native CI lanes remain the cross-platform gate.

| Configuration | Tests passed | Sequence points | Branch points | Fully covered methods |
| --- | ---: | ---: | ---: | ---: |
| Release | 4,916 | 45,174 / 45,174 | 13,716 / 13,716 | 4,664 / 4,664 |
| ReleaseLean | 4,851 | 45,172 / 45,172 | 13,716 / 13,716 | 4,663 / 4,663 |

Every warmed response and retained-storage benchmark reports 0 B/op. Matching
Short runs (one launch, three warmups, five measurements) at 16 default-material
pairs measure single contact 0.701 -> 0.696 ms, moving face 1.573 -> 1.638 ms and
resting face 1.189 -> 2.297 ms, each at one sweep. Sequential corner impulses
create intermediate angular motion and real friction work that the old batched
resting case canceled before friction; first-sweep contact reconstruction also
adds work. Later sweeps skip warm/correction traversal. An equivalent arithmetic
reuse experiment measured about 5-8% slower and was removed.

The invocation-count-one Short timings include JIT startup effects. A separate
stable-JIT run disables tiered compilation (two launches, six warmups, ten
measurements); these numbers are separate runtime conditions, not a direct
comparison against the original Short baseline. At 64 default-material pairs:

| Prepared response | One sweep | Configured six sweeps |
| --- | ---: | ---: |
| Single contact | 0.414 ms | 0.417 ms |
| Moving four-point face | 1.015 ms | 6.013 ms |
| Resting four-point face | 1.508 ms | 9.687 ms |

Even one surface has coupled angular rows. The centered unit-cube/default-material
fixture leaves angular component-sum residual 0.623 rad/s after one sweep,
0.0186 after six and zero after 32. Every multi-point manifold therefore uses the
configured budget; only a single row uses one sweep. Both service shortcuts reuse
the budget-aware public wrapper without building an island for an isolated pair.
The extra iterations are an explicit quality/cost tradeoff, not a speedup claim.

Cold pair/group/cache storage allocates 2,672 B for one four-point group and
18,864 B for nine. Retained reset/regeneration is 0 B/op at 0.637 / 8.861 us.
A 256-pair probe with forced compacting GC estimates about 2.6 / 18.4 KiB live
per pair before and after reset, including retained capacity; these are host heap
deltas, not portable layout guarantees. Logs/results are under ignored
`artifacts/grv-issue-095/phase1-*`, including `phase1-final-release-gate`,
`phase1-final-lean-gate`, `phase1-response-baseline`, `phase1-response-after-valid`,
`phase1-response-optimized` and `phase1-memory.log`.
Release/Lean solution builds pass with zero warnings/errors, including both
library target frameworks and benchmarks. DocFX passes warnings-as-errors;
195 API pages, branding/resources, changed Markdown paths, wiki link rewriting
and `git diff --check` pass. All changes remain unstaged/uncommitted.
The producer audit captured separate CCD normal inversion as **GRV-Issue-100**;
its swept-contact policy remains follow-up work.

## Phase 2 — Exact Finite Geometry And Admitted Connectivity

### Task 3: Prepare canonical surface provenance and exact region connectivity

**Owners:** `src/Gravitas/Colliders/Mesh/PhysicsMesh.CoplanarPatches.cs`,
`PhysicsMesh.Scale.cs`, and existing convex-topology welding;
`src/Gravitas/CollisionHandling/Detection/3D/Context/CollisionSatScratch.cs`.

**Interfaces:** Retain `GetCoplanarPatchId(int triangleIndex)` as a preparation
lookup only. Add internal `int GetCanonicalSurfaceOrdinal(int triangleIndex)`
and `ReadOnlySpan<int> GetCoplanarTriangleNeighbors(int triangleIndex)` backed
by compact committed spans. Retain welded-vertex incidence for vertex-touch
connectivity. Scratch owns admitted triangles, component parents and geometric
region ordering, never public or body-serialized state.

Manifold consumers use `GetManifoldSurfaceOwner` and
`GetCanonicalSurfaceBoundaryVertexPairs` together; canonical segments remove
monotone collinear subdivisions. A legacy patch can have a
twice-wound or overlapping planar cover despite balanced seams. The new owner
therefore requires an embedded triangle complex: distinct faces intersect only
in their shared welded simplex. A certified single strict convex ring supplies
the existing unit-winding proof; other patches retain directed boundary cycles,
collapse monotone collinear subdivisions and require simple disjoint cycles
with nested winding zero or one. This certifies the oriented triangle chain
without comparing every pair of triangles. Declined owners retain individual
triangle domains.
Canonical provenance, trusted neighbors, exposed boundaries and query grouping
consume this same certificate, published atomically with scaled geometry.
This preserves the separate legacy minimum-exit patch contract.

- [x] Add red mesh/topology tests for alternate diagonals, winding, welded
  duplicates, collinear subdivisions, rejected ambiguous topology and failed
  scale preparation. Compare canonical geometric surface provenance rather
  than authored triangle or welded-array indices.
- [x] Extend existing welded preparation to retain adjacency/incidence and
  canonical surface ordinals from actual local plane/domain geometry, ignoring
  removable collinear subdivisions. Preserve atomic candidate-scale publication
  and leave current geometry untouched on failure.
- [x] Derive query components only through exact admitted shared-edge or
  shared-vertex intersections supplied by task 4. Include zero-depth tangency
  explicitly. Each clipped triangle section is convex, but one patch can have
  multiple admitted components; do not equate patch IDs with region IDs.
- [x] Order components by exact geometric region extrema/features, independent
  of triangle discovery order. Test hole/notch intersections, point-touching
  sections and disconnected regions that share the same plane normal.

### Task 4: Establish exact plane-section ray candidates in FixedMathSharp

**Owners:** sibling `src/FixedMathSharp/Geometry/Wide/FiniteAxis/`
`ConePlaneRayEvents`, `ConePlaneRayFrame`, `ConePlaneRaySelection`,
`ConePlaneRayPoint`, `ConePlaneRayPointExits` and
`ConePlaneRayPointMaterialization`, reusing `ContactQuadratic` and existing
finite-cone arithmetic. Gravitas `MeshConeFaceRegions` owns component reduction.

**Interfaces:** Construct one internal `ConePlaneRayFrame` per geometric
surface, using a canonical exact plane normal and the authoritative relative
rigid frame. `ConePlaneRayEvents.GetIntrinsicEvents(frame, events)` supplies shared-plane
descriptors; `GetBoundaryEvents` supplies finite-segment descriptors.
`TryEvaluateEvent` consumes a `ConePlaneRayEventSource` (`Plane` or `Segment`)
and caller-owned point, root and two directional selection resources. Gravitas
tests each exact point against filled triangle walls before assigning its
component, retaining two unrounded winning certificates per region. Exact
shared-edge and vertex admission uses the same finite-cone predicates. Preserve
`TryGetMinimumAxialPoint` and existing minimum-exit contracts separately.

For an apex-centered cone, write `F(p) = H²(px²+pz²)-R²py²`, with
`0 <= py <= H`, and a plane `N·p=c`. A ray uses `q=p+tN`. On the upper
rim, `q=(R cos(theta), H, R sin(theta))` and `t=(N·q-c)/(N·N)`.
Factoring `F(q-tN)` using `F(q)=0` reduces lower-side admission to a linear
circle predicate for nonzero `t`; zero depth is handled separately. Triangle
walls perpendicular to `N` can test `q` directly because their projection is
unchanged along the ray. On a lower-side generator `g`, `p=c*g/(N·g)`;
the nonzero upper-side root is fractional-linear in the circle parameter.
These charts keep regular face events quadratic. Candidate coverage must also
include finite edge stationary points, cap switches, apex/base sections and
degenerate intervals; the circle argument alone is not a completeness proof.

The reviewed full-domain face ledger bounds compressed event coefficients below
1,248 bits and discriminants below 2,498 bits; rational axis/generator point rays
have coefficients below 1,302 bits and discriminants below 2,485 bits. Forty
words retain the root, and 64-word fields retain evaluated point/depth values.
Admission and paired-anchor products use bounded larger transient fields.
Retained descriptors reconstruct only requested events; they preserve exact
geometry even when final coordinate rounding reports overflow. The shared-plane
cohort has at most 102 possible constructions and each finite boundary segment
has 14; these bounds count descriptors before admission, not admitted points.
For an exact nonzero axial plane normal, the intrinsic cohort retains just 12
descriptors in the original order: eight seams, three axis events and the apex.
The other 90 constructions cannot be admitted: non-seam circle restrictions
reduce to `C*(1+u*u)` (no real root, or an already-rejected zero polynomial),
`Ny*FullHeight*(d*d+n*n)` cannot vanish for a generator, and zero radial normal
rejects base stationarity. This holds for either axial direction and zero radius;
degenerate zero-normal planes retain the general cohort. Explicit descriptor
replay and all finite-boundary constructions remain available.

- [x] Establish a candidate-coverage proof in this phase's notes and source
  invariants before choosing fixed-width carriers or claiming completeness.
  Cover admitted projected global support; stationary extrema on finite clipped
  edges; lateral section extrema; base-section intervals; side/base branch
  switches; rim/apex tangencies and degenerate generators. Axial endpoints
  alone cannot maximize the minimum of side and cap ray exits.
- [x] Derive root degrees, sign/rounding rules and intermediate width bounds;
  reuse existing exact comparisons and root resources where their proven
  contracts fit. If those contracts do not fit, establish focused neutral math
  changes upstream before integrating a caller; never substitute rounded tests.
- [x] Add red FMS tests in a focused `FixedTriangle.FiniteCone.PlaneSection.Tests.cs`
  beside the existing finite-cone tests. Include H=1000/R=1, rotation
  `(0,0,3/5,4/5)`, Y=0 quad `[-2,2]`: the section is interior while global
  projected apex/base support lies outside. Include side/base switch extrema,
  exact touch and one-raw-unit gaps, rigid frames and final nearest-even rounding.
- [x] Implement unrounded finite admission and maximum ray distance in each
  canonical plane-normal orientation. Reduce across each actual component,
  choose the smaller maximum before point sampling, and resolve exact symmetric
  ties by canonical plane orientation. Test varying chord midpoints never emit
  opposite positive-depth face corrections within one continuous region.
- [x] Retain admitted candidate geometry for bounded spatial sampling and final
  paired anchors. Do not re-test membership using a rounded intermediate or
  reuse a complete minimum-exit gap proof after masking features.
- [x] Run FMS Release/Lean suites and exact reachable coverage, then local-stack
  Gravitas controls and allocation gates. Review arithmetic proofs independently.

### Task 5: Admit exposed edges/reflex vertices and reduce geometric samples

**Owners:** task 4 FMS geometry owners, prepared perimeter data from task 3,
and existing Gravitas mesh contact generation. No new general union-MTD API.

**Interfaces:** Add finite-feature candidate accumulation beside the plane-ray
operation: internal `bool AccumulateSegmentConeSurfaceCandidates(
FixedSegment segment, Vector3d origin, FixedQuaternion rotation,
Vector3d coneCenter, FixedQuaternion coneRotation, Fixed64 height,
Fixed64 radius, scoped ref ConeSurfaceSelection selection)`. Selection borrows
caller-owned exact resources; retain finite parameter/provenance and materialize
`FixedContactAnchors` only after admission. Gravitas consumes true exposed
boundary ownership; no physics policy or prepared-mesh IDs enter the math API.
For continuous families, `SegmentConeSurfaceCandidate.AccumulateFamilyEvents`
accepts authored halfspaces, point location and a caller-owned
`ConeSurfaceFamilySelection`. Gravitas derives the incident-fan constraints;
FMS clips the exact family and retains event descriptors for admitted
representatives before materialization.

- [x] Add red edge/generator/rim tests where a support gap's affine foot lies
  outside the segment, plus genuine edge/vertex touches and reflex notch/hole
  corners. A convex ear with `requiredMask` must not authorize reflex contact.
- [x] Reuse existing generator slices and stationary rim arithmetic with
  independent exact finite-parameter admission. Preserve real incident face/
  boundary constraints and use internal seams solely for clipping/connectivity.
- [x] Select at most four samples per face region from its finite admitted
  selected-orientation pool: exact maximum directional depth first, then widest
  projected span, largest triangle area and largest distance from the filled
  selected triangle. Evaluate these coverage metrics exactly on once-rounded
  Q32.32 anchors; resolve ties by exact geometric point order and collapse
  exact-coincident points. This does not establish continuous spatial extrema.
  Test equivalent triangulation, mirrored winding and connectivity changes.
- [x] Preserve standalone minimum-exit and certified-fast-path regressions in
  FMS while verifying scoped surface constructors independently. Convexity
  alone cannot authorize old MTD shortcut semantics in a surface sampler;
  approve runtime reuse only with witness/region coverage during phase 3.
- [x] Run focused geometry/mesh tests, full owner suites, coverage and the nine
  original triangle-contact performance/allocation controls.

**Phase 2 summary — 2026-10-09:** Canonical prepared surfaces,
exact clipped connectivity and per-component face orientation are implemented.
The face path evaluates shared-plane and canonical finite-boundary events,
admits exact points through filled triangle walls, and compares both unrounded
regional maxima, using canonical positive orientation on exact equality.
Compact once-rounded mesh anchors support finite-pool reduction; exact winner
storage and per-direction source descriptors retain depth and geometric ties.
All eligible selected-direction exits receive an exact world-range preflight;
only the chosen four outputs incur exit/depth rounding. Nearest-even rounding
is monotone, so a representable exact regional maximum bounds every eligible
pool depth. Isolated
boundary certificates and continuous families obey the actual same-owner
incident fan, including reflex/hole corners. Family halfspaces and admitted
event descriptors support materializable representatives; internal seams
produce no boundary contacts. All Task 5 geometry, sampling, performance and
coverage gates pass. Runtime integration remains in phase 3.

**Foundation measurements — 2026-10-08:** Canonical concave preparation
measures 0.233 / 0.939 / 4.075 ms for 64 / 256 / 1,024 faces, versus
0.557 / 6.448 / 83.713 ms for the original triangle-pair certificate. Warmed
connectivity takes 7.15 / 11.98 / 31.50 us, versus 267.56 / 1,062 / 4,217 us,
at 0 B/op. Surviving-corner certification remains quadratic; collinear
subdivision is linear. Each prepared/committed metadata bank uses
`52*T + 8*V + 8*B + 12` bytes before headers/capacity (`B` counts canonical
boundary segments). Connectivity extrema add 809 bytes per observed
multi-region high-water slot; single connected regions skip them. The current
full validation below supersedes the earlier coverage snapshots.

Independent review found that fully unrounded four-sample coverage can combine
five quadratic fields, reaching degree 32. Depth events also do not supply all
extrema of a later spatial metric over a continuous section. The accepted
refinement keeps admission, connectivity, depth and provenance exact, then
ranks a finite admitted sample pool using final Q32.32 anchors with geometric
ties. For once-rounded representable coordinates, let `u = 2^-32` world units:
each anchor has Euclidean error at most `sqrt(3)*u/2`, and a two-anchor span has
error at most `e = sqrt(3)*u`; exact orthogonal plane projection cannot
increase it. Let `d`, `a` and `b` be the exact pre-rounding projected edge
vectors. Squared length differs by at most `2*|d|*e + e^2`; triangle area differs
by at most `((|a|+|b|)*e + e^2)/2`. The metrics must be evaluated exactly on the
rounded anchors: additional arithmetic rounding or overflow is outside these
bounds. These bounds concern finite sample ranking, not complete coverage of
a continuous section or contact admission. Owner review accepted this finite-pool contract. Admission, connectivity,
depth and provenance remain exact; only coverage ranking uses the final
once-rounded Q32.32 anchors, with exact projected metrics. The finite pool contains the exact regional maximum and available
paired-ray certificates in its selected orientation, using intrinsic cone/plane
features and canonical exposed boundary segments. It excludes internal
triangulation sampling events; a source with only the opposite exit certificate
is outside this selected-orientation pool. Exact-coincident points collapse;
distinct exact points that round to the same coordinate retain geometric ties. This does not establish continuous spatial
extrema. Phase 2 implementation and validation are complete; #095/#099 remain
active until runtime integration and physical validation.


Phase 2 closeout includes refinement of costs introduced by this phase.
Profiling identified repeated intrinsic construction per triangle,
full synthetic-triangle construction per exposed edge, and exit reconstruction
for point membership/ties. The refined path evaluates one shared-plane cohort
and true finite-segment cohorts, assigns exact points through filled triangle
walls, and retains only two winning wide certificates per region. Compact
once-rounded mesh anchors and per-direction source/range records avoid rounding
the entire candidate pool. Exact exit preflight preserves whole-pool failures;
only the selected outputs replay their certificate for final exit/depth rounding.
Unavailable certificates and output-range failures remain distinct for each direction.
The two exact winner banks retain 13,024 bytes of coefficients/signs per
allocated region slot, plus descriptors and array overhead; candidate-pool
entries retain compact mesh anchors and source/range records, not wide banks.
Superseded internal triangle accumulation/emitter APIs are retired with their
consumer. These introduced performance costs are optimized and validated here,
rather than deferred to runtime integration. The comparable refined capture completed every fixture at 0 B/op:

| Face sampling fixture | Initial Phase 2 | Refined Phase 2 | Reduction |
| --- | ---: | ---: | ---: |
| Interior | 4.020 ms | 0.721 ms | 82.1% |
| Boundary | 16.951 ms | 2.644 ms | 84.4% |
| Tilted interior | 3.244 ms | 1.020 ms | 68.6% |
| Subdivided tilted | 44.813 ms | 1.029 ms | 97.7% |

The added 32-triangle subdivided boundary fixture takes 2.623 ms, versus
2.644 ms for its two-triangle counterpart; canonical boundaries retain four
segments in both. Exact membership can reuse a strict convex corner fan only
for one region, when that fan is smaller and the grouped triangles are closed
under every committed owner neighbor. Prepared patch unions record those same
neighbors, so this nonempty closed subset is the whole connected owner. One
`O(T)` check replaces repeated authored-triangle membership with `O(K*(B-2))`
fan membership, using the existing exact closed-triangle predicate. No new
math predicate, persistent metadata or cache is needed. Partial, nonconvex and
multiple-region owners retain the original exact assignment. Full → subset →
full regressions under both windings verify that the shortcut cannot widen
admission or retain stale results. This last refinement cuts subdivided boundary
from 3.865 to 2.623 ms (32.1%) and subdivided tilted from 1.384 to 1.029 ms
(25.6%); two-triangle controls remain steady within their confidence intervals.
Exposed-boundary admission takes 0.957 ms for the boundary
fixture and 14.17–15.69 us for the interior fixtures, all at 0 B/op. These are
prepared geometry operations, not complete gameplay frames. Face sampling
includes connectivity, exact orientation and four materialized outputs;
boundary sampling measures admitted descriptors. Both compared captures use
one launch, three warmups and five measured iterations on this Windows x64
host. An earlier boundary benchmark process exited with native code
`0xC0000005` without an attributable managed frame. A fresh pre-refinement
capture, the subsequent refined ten-fixture captures and the 256-iteration warmed
allocation regression complete successfully; the failed capture remains under
`artifacts/grv-issue-095/phase2-wrap-face-sampling-benchmarks`. This is unreproduced evidence,
not an identified or claimed-fixed source defect. Final face-sampling evidence is under
`artifacts/grv-issue-095/phase2-fan-face-benchmarks`; the comparable initial
Phase 2 capture is `phase2-wrap-face-sampling-repeat`. These timings compare
the new multi-sample geometry operation before and after refinement, not the
existing single-contact runtime path.

Final validation after the sampling refinements follows. All captures use
`UseLocalLsfStack=true`:

| Owner/configuration | Tests passed | Sequence points | Branches | Fully covered methods |
| --- | ---: | ---: | ---: | ---: |
| Gravitas Release | 5,018 | 45,915/45,915 | 14,180/14,180 | 4,747/4,747 |
| Gravitas ReleaseLean | 4,953 | 45,913/45,913 | 14,180/14,180 | 4,746/4,746 |
| FixedMathSharp core/helper Release | 4,821 | 55,855/55,855 | 13,942/13,942 | 4,198/4,198 |
| FixedMathSharp core/helper ReleaseLean | 4,800 | 55,948/55,948 | 13,942/13,942 | 4,194/4,194 |
| FixedMathSharp.Chronicler Release | 49 | 85/85 | 12/12 | 18/18 |
| FixedMathSharp.Chronicler ReleaseLean | 49 | 85/85 | 12/12 | 18/18 |

Both solutions build Release/ReleaseLean for `netstandard2.1`/`net8.0` with
zero warnings/errors; both API sites pass DocFX warnings-as-errors. Independent
mathematical and ponytail reviews found no remaining blocker. Exact overflow
regressions include opposite-direction overflow, coincident complementary
certificates, admitted mesh-anchor overflow, and an unchosen eligible boundary
exit overflow that fails the entire selected-orientation pool. Warmed sampling
remains allocation-free over 256 iterations. The nine original runtime contact controls remain at 0 B/op; short-run means
vary -1.1% to +3.8%, with overlapping 99.9% confidence intervals in each fixture.
Those unchanged minimum-exit consumers do not call the plane-ray exit
materializer. Its exact-zero shortcut adds about 7–8% improvement to the three
interior sampling fixtures after the main scoped-event refinement, with no
material boundary regression in these short captures. The additional exact
axial pruning reduces the interior fixture from 1.023 to 0.718 ms (29.8%) and
the boundary fixture from 3.069 to 2.643 ms (13.9%); every omitted descriptor
is checked against explicit full-inventory replay. Both point and exit
range decisions still use the same parity-aware nearest-even thresholds;
invalid denominators are rejected before the zero shortcut. The final upstream plane-section controls take 0.645 / 0.998 / 1.707 ms for
interior / side-base switch / tilted 1000:1, versus the initial 1.185 / 1.051 /
2.563 ms, all at 0 B/op. Full upstream Release/Lean coverage is complete;
Gravitas whole-owner refinement validation is complete. Final coverage artifacts
are `phase2-axial-final-FixedMathSharp-*` and `phase2-fan-final-Gravitas-*` under
`artifacts/grv-issue-095`; upstream plane controls are
`phase2-axial-final-plane-controls`. These Windows x64 gates do not replace cross-platform replay
conformance or released-package validation after the coordinated FixedMathSharp
release.

**Recommended phase commits:** FixedMathSharp: `feat: add clipped cone families and scoped plane-ray geometry`;
Gravitas: `feat: complete exact mesh-cone surface sampling`. All changes remain
unstaged and uncommitted for owner review. No phase 3 runtime integration is
included in this closeout.

## Phase 3 — Mesh/Cone Integration And Physical Regressions

**Owners:** `src/Gravitas/CollisionHandling/Detection/3D/CollisionDetection.Cone.cs`,
`Context/CollisionSatScratch.cs`, existing contact group and mesh owners;
`src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Rotational.cs` for its
singular witness audit.

**Interfaces:** Replace `TryFindMeshConeTriangleContact` and its single
`SetContact` reduction with `BuildMeshConeSurfaceContacts(LSMeshCollider mesh,
LSConeCollider cone, CollisionSatScratch scratch, ContactManifold manifold)`
returning `bool`. Consume exact admitted components and geometric group keys;
retain separately justified closed-convex containment behavior.

- [x] Promote the diagnostic into maintained `MeshConeSurfaceManifoldTests.cs`
  using `PhysicsScenarioBuilder`. Unit cone at `(3/10,0,3/10)`, joined X/Z walls
  spanning Y `[-4,4]` and positive tangential coordinates `[0,4]`: require both
  `Right`/`Forward` contacts and full-loop blocking of impulse `(-1,0,-1)`.
  Use explicit static wall bodies and frictionless materials; retain separate
  wall controls and both windings. Record the actual red failures.
- [x] Add tab, ring/hole, U-notch, disconnected supports, long tilted interior,
  exposed edge and cap/rim/generator cases. Include seam sliding, support torque,
  centroid changes, raw-neighbor gaps, rigid transforms and scale refresh.
- [x] Generate all admitted groups through tasks 3–5, reserve required overflow
  before publishing the manifold, and preserve existing BVH candidate gathering.
  Use checked geometry budgets as safety bounds, not a reason to allocate a
  full mesh–mesh product per pair. Reserve context scratch from prepared geometry;
  retain pair capacity at its observed high-water mark and measure growth.
- [ ] Admit existing certified fast paths only when their actual finite
  witnesses and region coverage satisfy this surface contract. Preserve cheap
  ordinary contacts where that proof holds; convexity alone does not establish
  compatible surface semantics. Measure each accepted shortcut against the
  complete sampler and the original runtime controls.
- [x] Retire the replaced runtime path in the same integration change:
  `TryFindMeshConeTriangleContact`, `PrepareMeshConePatchContacts`, and
  `CollisionSatScratch.MeshConePatchContacts`. After tracing all remaining
  callers, remove FMS `TriangleConeContact.Patch.cs` and
  `TriangleConeContact.ConvexPatch.cs`, their exclusive polygon helpers, and
  unused legacy mesh patch IDs/corner banks. Keep the public triangle MTD and
  sweep contracts, shared perimeter data and certificates still consumed by
  surface trust. Add context-owned mesh/cone scratch only with its production
  consumer; retain continuous-family descriptors only when admission uses them.
- [x] Update runtime tests/benchmark assertions that currently equate physical
  contact with global minimum exit; replace them with actual surface witness,
  sliding/support/torque assertions. Keep standalone math-MTD expectations.
- [x] Audit rotational CCD `PrimaryContact` selection for the relevant closing
  impact witness. Test direction reversal and earliest impact; do not replace
  CCD with deepest-group depenetration. Verify grounding, sleep/wake and events.
- [x] Run integrated pure-3D, compound, full-loop replay, CCD, grounding and
  lifecycle suites plus pure-2D/mixed parity controls. #097/#098 migrations stay
  in their own tracker entries. #096 practical geometry-size policy stays
  separate; supported relative-anchor compatibility is repaired in this phase.

**Phase 3 progress:** The maintained joined-wall regression reproduced four
failures while both separate-wall controls passed. Integration now emits
independent face, edge and corner groups, preserves local identities through
compound staging, and selects rotational CCD response by exact closing point
velocity. Physical regressions cover tabs, holes, disconnected supports, sliding,
torque, rigid transforms, scale publication, raw-neighbor gaps, earliest impact,
grounding, sleep/wake, events and full-loop replay. A generator-touch defect in
the first boundary integration was fixed here: alternative exits use the exact
per-feature minimum and its ties, preventing false depenetration at touch.

The replaced patch runtime, its exclusive upstream geometry helpers/tests and
unused mesh patch IDs are retired. This phase adds no production files. Refinement
reuses primitive plane normals, bounded exact rounding, active-width complete
products, scoped identity materialization, a synchronous ordered event cohort,
closed-boundary admission reuse and conservative whole-cone range certificates.
The same exact frame is reused across connectivity, faces and boundaries. A
scoped trace certificate cancels the inverse identity basis, and collapsed apex
sections reuse the axis event instead of duplicating cardinal events. Rational
comparisons use two signed cross products; scaling skips mathematically zero
coefficient banks. Selected-sample replay omits the unused opposite rational or
side exit, while admission and streaming retain both directions.
Fourteen upstream fixtures independently compare streamed descriptors, points,
both directional certificates and final materialization with targeted reconstruction.
For axial plane cardinals, the existing dispatcher proves a primitive normal
of `(0, ±1, 0)`. The producer checks the closed cone-height interval before
construction; its exact point then satisfies both the plane and cone side
polynomial by construction. Reusing that certificate avoids redundant generic
admission and exit work. Eight upstream cases cover both normal signs,
out-of-height rejection, collapsed axis/radius events, requested-direction replay
and odd-origin nearest-even endpoint rounding. Targeted replay still reconstructs
collapsed descriptors omitted from the streaming inventory.
Global identity-basis normalization was rejected after stalling an existing
cylinder rounding test and was removed. Common whole-zero limb cancellation was
narrowed to the immutable-span ratio entry after typed controls exposed shared
core overhead; inputs remain unchanged and typed division mechanics remain
shared. Global nearest-even parity is explicitly carried into translated
rounding. A separate common whole-limb cancellation experiment in homogeneous
points was also removed: its 20-fixture capture showed mostly marginal or mixed
changes, insufficient to justify scanning and mutating every admitted point.
Two further experiments were removed after complete captures: extending the
rounded-coordinate ordering shortcut to vertical planes, and reducing the
shared affine frame's denominator, basis and translation by their joint gcd.
Both preserved exact geometry but showed no useful overall gain. The retained
identity-cone X-order certificate skips reconstruction only when distinct rounded
X values already prove the exact order; ties retain the original exact comparison.
A direct fixed-width replacement for the quadratic normal products was also
removed: small rim gains did not offset mixed small-contact and group regressions.

The final Windows x64 capture covers 20 warmed runtime fixtures, all at 0 B/op:

| Control | Committed single-contact baseline (µs) | First integration (µs) | Current (µs) |
| --- | ---: | ---: | ---: |
| Apex face | 22.86 | 851.95 | 104.57 |
| Base face | 21.89 | 872.51 | 111.21 |
| Side face | 27.85 | 1,969.38 | 574.56 |
| Interior rim | 905.84 | 4,299.23 | 1,308.13 |
| Oblique rim | 668.97 | 3,051.91 | 1,493.91 |
| Subdivided tilted surface | 244.90 | 3,704 | 1,282.56 |

Joined walls, disconnected supports, ring and thin-tab controls take
2.056/2.606/2.430/2.896 ms. Against the previous committed refinement,
apex/base/side improve 43%/32%/21%, and the subdivided tilted control improves
16%. The axial-cardinal certificate adds 10–12% improvement to the apex/base
and three small axial controls against the preceding accepted refinement.
Both final captures retain 0 B/op in all 20 fixtures. The richer constraints change the work
performed, but ordinary controls remain roughly 5–21 times their original
single-contact costs. **Performance acceptance remains open; introduced costs
stay in phase 3.** Profile remaining admission/materialization work before
choosing the next refinement. The thin-tab capture has greater variation; do not
treat every small difference between short captures as an established gain.
Fresh EventPipe profiles of apex, side and interior-rim controls identify the
remaining general circle-parameter path at 46.7% and 42.3% inclusive managed CPU
samples in the side and interior-rim fixtures; these overlapping call-tree
percentages are not additive. Generic point admission accounts for 15.2% and
10.3% respectively, including callers that already carry a side certificate;
only part of that cost is potentially redundant. The next experiment should reuse exact construction
certificates within those existing owners, measuring the complete controls
before retaining a shortcut. Lower-circle and zero-orientation upper-rim events
already skip cone-polynomial admission; only a proved projected upper-rim
side-equality stratum could remove that check. Finite-segment and other projected
rim strata retain general admission. No proposed certificate is implemented or
claimed as a gain in this capture.

With `UseLocalLsfStack=true`, full **unfiltered** FixedMathSharp suites pass
4,909 Release and 4,888 ReleaseLean cases; Gravitas passes 5,077 Release and
5,012 ReleaseLean cases. Raw sequence/line, branch and fully-covered-method
totals are exact 100% in all four reports:

| Repository / configuration | Lines | Branches | Fully covered methods |
| --- | ---: | ---: | ---: |
| FixedMathSharp Release | 56,010/56,010 | 14,218/14,218 | 4,209/4,209 |
| FixedMathSharp ReleaseLean | 56,103/56,103 | 14,218/14,218 | 4,205/4,205 |
| Gravitas Release | 46,043/46,043 | 14,348/14,348 | 4,759/4,759 |
| Gravitas ReleaseLean | 46,041/46,041 | 14,348/14,348 | 4,758/4,758 |

Both multi-target solution builds have zero warnings or errors. Independent
ponytail/math/physics source review found no additional blocker or actionable
bloat cut. The 49-case FixedMathSharp.Chronicler adapter suites also pass in
both configurations with exact 85/85 lines, 12/12 branches and 18/18 fully
covered methods. Both API sites pass DocFX warnings-as-errors and local-link
validation. The five upstream typed ratio controls take
40.89/50.66/45.73/27.81/52.69 ns; three plane-section controls take
292.1/667.7/1,221.3 us, all at 0 B/op. Small differences in these short captures
are not universal speedup or regression claims. Evidence is retained under ignored
`artifacts/grv-issue-095/phase3-axial-final-*`; this host evidence does not
replace native cross-platform replay gates.

**Compatibility repair complete:** Radius-1, height-4 cones near both scalar
limits now retain valid relative contacts even when conceptual endpoints cross
the absolute coordinate limit. Sampling reuses existing anchor/materialization
owners in one common translated frame, preserves global nearest-even parity and
publishes the original collider origins without narrowing large cancelling
offsets. Tests cover both limits, odd-origin half ties, exact additive anchor
terms, contact identities and response parity; the original four-case
cone/cylinder theory is restored to the unfiltered suites.

The phase-2 absolute-world restriction was an implementation assumption. Its
isolated sampler tests enforced that assumption while public-anchor regressions
still exercised the legacy producer. Integration exposed the mismatch; the
producer now preserves the existing relative-anchor contract. This does not
expand supported geometry sizes or alter final body-position policy. Practical
radii/extents remain #096; source-observed saturation in existing 3D/2D collision
position correction is separately captured as #102.

Performance acceptance remains open in this phase; additional constraints alone
do not justify an unusable ordinary-contact cost. Phase 3 and #095/#099 remain
open. The separately confirmed conservative rotational CCD frontier for concave
aggregate bounds remains #101.

## Phase 4 — Coverage, Performance, Documentation And Closeout

- [ ] Extend `MeshConeSurfaceContactBenchmarks` with tab/hole, connected walls
  and subdivision cases. Add direct grouped-response and cold/retained-memory
  measurements alongside existing response benchmarks; measure one-surface,
  redundant samples and multiple independent groups under the same settings.
- [ ] Compare before/after on the same host, stack, build and fixture. Require
  warmed zero allocation and explain every material ordinary-contact regression;
  investigate regressions before accepting them as the cost of correctness.
  Do not silently loosen allocation/performance gates.
- [ ] Run full Release and ReleaseLean coverage in changed repositories,
  verifying raw OpenCover sequence/branch points and fully covered methods,
  rather than rounded percentage summaries. Re-run only when changes/failures
  justify it. Gravitas collection command for each configuration:

  ```powershell
  dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj -c Release -p:UseLocalLsfStack=true -p:BuildInParallel=false -m:1 --collect:'XPlat Code Coverage' --settings tests/Gravitas.Tests/coverlet.runsettings --results-directory artifacts/surface-manifold/Release
  ```

- [ ] Version authoritative replay expectations only for reviewed semantic/schema
  changes. Run repeated full host traces and existing cross-platform conformance;
  distinguish local evidence from CI platforms not executed on this host.
- [ ] Update evergreen `COLLISION_PIPELINE.md`, `COLLISION_RESPONSE.md` and API
  XML docs with group limits, inspection order, local surface contacts, reduction
  approximation and response semantics. Document the intentional public constant/
  ordering compatibility change; do not add a legacy missing-constraint mode.
- [ ] Build both API sites with DocFX warnings as errors when their owners
  change, validate local links, and run `git diff --check`. Independent final
  review covers mathematical admission, solver behavior and unnecessary code.
- [ ] Close #095 and #099 in `issue-tracker.md` with concise dated evidence after
  all gates pass. Keep #097/#098 and confirmed unrelated findings separately
  tracked. Preserve coordinated upstream release/package-validation obligations.

**Phase 4 summary:** Append final validation, measured costs/limitations and
recommended commit messages here. Leave source, tests and docs unstaged and
uncommitted for owner review.
