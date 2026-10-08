# Surface Contact Manifold Implementation Plan

> **For agentic workers:** Use `superpowers:executing-plans` for implementation
> in this checkout, with independent subagent reviews at the phase boundaries.
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

**Spec:** [Approved surface-contact design](2026-10-08-surface-contact-manifold-design.md).

**Status:** Implementation plan ready for review; production changes have not
started. #097/#098 remain separate follow-ups, not closeout prerequisites.

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

- [ ] Add failing production-behavior regressions in `ContactManifoldTests.cs`:
  nine independent groups survive; deep same-group samples cannot evict a
  shallower independent group; insertion permutations expose equal groups and
  points; group/flattened indexing and reset/reuse agree.
- [ ] Run the focused tests and record the expected capacity failure before
  implementation. Tests needing the new API must compile before their red run;
  do not count a missing-member compiler error as the reproduction.
- [ ] Move the existing four slots into an inline first group and retain a
  `SwiftList<ContactGroup>` only for overflow. Enumerate group-major, then
  canonical local features, without repeated linear flattened lookups. Keep
  reduction bounded within each group and collision-free group equality.
- [ ] Retain the existing four-entry warm-start value within each cache group;
  extend `StoreWarmStartImpulse`, `TryGetWarmStartImpulse` and
  `RemoveWarmStartImpulse` with `in ContactGroupKey`. Prune absent points/groups
  after regeneration. Clear logical caches on reset, pooling, mass mutation,
  population and reconfiguration while retaining reusable capacity.
- [ ] Transfer groups through compound scratch, remapping A/B namespaces and
  reversing provenance with anchors/normals. Keep material regions distinct and
  notifications once per collider pair. Rename cylinder's local four-sample
  bound to `MaxContactsPerGroup`.
- [ ] Version `manifold.3d` from 7 to 8 and `warm-start.3d` from 1 to 2; hash
  canonical group structure and point/cache order in the existing mandatory
  authoritative contributor. Update the pair section only if its direct
  serialized layout changes. Keep body saves free of contact caches.
- [ ] Extend `CollisionWarmStartTests.cs`, `CompoundColliderCollisionTests.cs`
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

- [ ] Add red tests in `CollisionResponseInvariantTests.cs`,
  `CollisionResponseExactLeverTests.cs` and `DiscreteIslandSolverTests.cs`:
  orthogonal admitted normals stop both incoming components; duplicate
  same-normal samples with frozen rotation do not multiply linear response;
  disconnected supports retain torque; failure beyond contact index seven
  clears only that row; supplied normals survive misleading collider centers.
- [ ] Apply compatible warm starts in a separate deterministic pass before
  solving; a row must not read a cache already overwritten by another row.
  Preserve response-center/position snapshots for exact lever ownership.
- [ ] Solve and apply each accumulated normal delta against current velocity,
  using share one, then Coulomb friction against that row's accumulated normal
  impulse. Preserve normal nonnegativity, restitution threshold and exact
  representability preflights. A failed row clears its cache and skips its
  remaining response; other rows continue without a bitmask.
- [ ] Correct position once per identical canonical correction direction,
  choosing its deepest admitted depth with deterministic ties. Preserve slop
  and the existing correction fraction; do not divide by unrelated point/group
  count. Keep separate same-normal lever arms for velocity solving. Test
  near-parallel/opposing directions and rotated inertia; this is a documented
  translation correction policy, not exact multi-contact depenetration.
- [ ] Preserve admitted nonzero normals in `ResolveContactNormal`; collider
  centers are fallback only for genuinely zero legacy normals. Audit other
  producers before changing the shared consumer; correct inverted producer
  contracts at their source rather than retaining a concave-normal flip.
- [ ] Change both queued-one-pair and single-contact-island shortcuts so
  multi-group pairs receive configured `DiscreteSolverIterations`. Keep the
  genuine one-surface shortcut. Verify iteration effects through physics state,
  not test-only counters. Delete unused `SolverContactBuffer` and mask helpers.
- [ ] Run response, exact-lever, warm-start, joint/island and existing 2D/mixed
  parity controls, full suites/coverage and one-surface response benchmarks.

**Phase 1 summary:** Append actual test, coverage, allocation and performance
results here after implementation; #095/#099 remain active until integration.

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

- [ ] Add red mesh/topology tests for alternate diagonals, winding, welded
  duplicates, collinear subdivisions, rejected ambiguous topology and failed
  scale preparation. Compare canonical geometric surface provenance rather
  than authored triangle or welded-array indices.
- [ ] Extend existing welded preparation to retain adjacency/incidence and
  canonical surface ordinals from actual local plane/domain geometry, ignoring
  removable collinear subdivisions. Preserve atomic candidate-scale publication
  and leave current geometry untouched on failure.
- [ ] Derive query components only through exact admitted shared-edge or
  shared-vertex intersections supplied by task 4. Include zero-depth tangency
  explicitly. Each clipped triangle section is convex, but one patch can have
  multiple admitted components; do not equate patch IDs with region IDs.
- [ ] Order components by exact geometric region extrema/features, independent
  of triangle discovery order. Test hole/notch intersections, point-touching
  sections and disconnected regions that share the same plane normal.

### Task 4: Establish exact plane-section ray candidates in FixedMathSharp

**Owners:** sibling
`src/FixedMathSharp/Geometry/Wide/FiniteAxis/WideTriangleConeIntersection.cs`,
existing `TriangleConeWitnesses`, `TriangleConeGeneratorFeatures`,
`TriangleConeRimContacts`, `ContactQuadratic` and `FiniteAxisValueRoot` owners.
Add focused plane-ray selection/certificate files under that geometry area.

**Interfaces:** Add internal `AccumulatePlaneSectionRayCandidates(
FixedTriangle triangle, Vector3d origin, FixedQuaternion rotation,
Vector3d coneCenter, FixedQuaternion coneRotation, Fixed64 height,
Fixed64 radius, scoped ref ConePlaneRaySelection positive,
scoped ref ConePlaneRaySelection negative)` returning whether the finite
section is nonempty. Selection borrows caller-owned resources and retains
unrounded candidates/admission until component reduction. Exact shared-edge
and vertex admission uses the same finite-cone polynomial predicates. Preserve
`TryGetMinimumAxialPoint` and existing minimum-exit contracts separately.

- [ ] Establish a candidate-coverage proof in this phase's notes and source
  invariants before choosing fixed-width carriers or claiming completeness.
  Cover admitted projected global support; stationary extrema on finite clipped
  edges; lateral section extrema; base-section intervals; side/base branch
  switches; rim/apex tangencies and degenerate generators. Axial endpoints
  alone cannot maximize the minimum of side and cap ray exits.
- [ ] Derive root degrees, sign/rounding rules and intermediate width bounds;
  reuse existing exact comparisons and root resources where their proven
  contracts fit. If those contracts do not fit, establish focused neutral math
  changes upstream before integrating a caller; never substitute rounded tests.
- [ ] Add red FMS tests in a focused `FixedTriangle.FiniteCone.PlaneSection.Tests.cs`
  beside the existing finite-cone tests. Include H=1000/R=1, rotation
  `(0,0,3/5,4/5)`, Y=0 quad `[-2,2]`: the section is interior while global
  projected apex/base support lies outside. Include side/base switch extrema,
  exact touch and one-raw-unit gaps, rigid frames and final nearest-even rounding.
- [ ] Implement unrounded finite admission and maximum ray distance in each
  canonical plane-normal orientation. Reduce across each actual component,
  choose the smaller maximum before point sampling, and resolve exact symmetric
  ties by canonical plane orientation. Test varying chord midpoints never emit
  opposite positive-depth face corrections within one continuous region.
- [ ] Retain admitted candidate geometry for bounded spatial sampling and final
  paired anchors. Do not re-test membership using a rounded intermediate or
  reuse a complete minimum-exit gap proof after masking features.
- [ ] Run FMS Release/Lean suites and exact reachable coverage, then local-stack
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

- [ ] Add red edge/generator/rim tests where a support gap's affine foot lies
  outside the segment, plus genuine edge/vertex touches and reflex notch/hole
  corners. A convex ear with `requiredMask` must not authorize reflex contact.
- [ ] Reuse existing generator slices and stationary rim arithmetic with
  independent exact finite-parameter admission. Preserve real incident face/
  boundary constraints and use internal seams solely for clipping/connectivity.
- [ ] Select at most four samples per region by exact geometric coverage:
  maximum directional depth, then widest tangential span, largest remaining
  covered triangle area, then largest uncovered planar distance; resolve every
  tie by canonical local feature order. Collapse coincident admitted samples.
  Document angular reduction as approximation and test equivalent triangulation,
  mirrored winding and connectivity changes rather than pressure exactness.
- [ ] Verify valid certified fast paths against this surface contract. Reuse
  them only when their witnesses/region coverage satisfy it; convexity alone
  must not choose incompatible runtime semantics. Retain old geometric
  minimum-exit tests in FMS rather than deleting useful regression evidence.
- [ ] Run focused geometry/mesh tests, full owner suites, coverage and the nine
  original triangle-contact performance/allocation controls.

**Phase 2 summary:** Append the proofs, measured resource bounds and actual
validation here. The ray-candidate construction is proposed until these gates
pass; do not describe the existing axial helper as a complete clipped sampler.

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

- [ ] Promote the diagnostic into maintained `MeshConeSurfaceManifoldTests.cs`
  using `PhysicsScenarioBuilder`. Unit cone at `(3/10,0,3/10)`, joined X/Z walls
  spanning Y `[-4,4]` and positive tangential coordinates `[0,4]`: require both
  `Right`/`Forward` contacts and full-loop blocking of impulse `(-1,0,-1)`.
  Use explicit static wall bodies and frictionless materials; retain separate
  wall controls and both windings. Record the actual red failures.
- [ ] Add tab, ring/hole, U-notch, disconnected supports, long tilted interior,
  exposed edge and cap/rim/generator cases. Include seam sliding, support torque,
  centroid changes, raw-neighbor gaps, rigid transforms and scale refresh.
- [ ] Generate all admitted groups through tasks 3–5, reserve required overflow
  before publishing the manifold, and preserve existing BVH candidate gathering.
  Use checked geometry budgets as safety bounds, not a reason to allocate a
  full mesh–mesh product per pair. Reserve context scratch from prepared geometry;
  retain pair capacity at its observed high-water mark and measure growth.
- [ ] Update runtime tests/benchmark assertions that currently equate physical
  contact with global minimum exit; replace them with actual surface witness,
  sliding/support/torque assertions. Keep standalone math-MTD expectations.
- [ ] Audit rotational CCD `PrimaryContact` selection for the relevant closing
  impact witness. Test direction reversal and earliest impact; do not replace
  CCD with deepest-group depenetration. Verify grounding, sleep/wake and events.
- [ ] Run integrated pure-3D, compound, full-loop replay, CCD, grounding and
  lifecycle suites plus pure-2D/mixed parity controls. #097/#098 migrations stay
  in their own tracker entries; #096 extreme-range work stays deferred.

**Phase 3 summary:** Append actual #095 geometry and #099 full-loop evidence
here. Neither issue closes merely because the old probe passes in isolation.

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
