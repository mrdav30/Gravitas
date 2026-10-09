# Issue Tracker

## Tracker Rules

- Issue IDs use `GRV-Issue-NNN`. The next available ID is `GRV-Issue-101`.
- Assign an ID when an issue enters this tracker, keep it through resolution,
  and never reuse an ID even if an entry is later removed. Check this file's Git
  history before advancing or repairing the counter.
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

### Validation Workflow

- Use package references for normal development and release validation.
- Use `UseLocalLsfStack=true` only when an unreleased lower-stack change must be
  validated across sibling repositories.
- Resolve defects in the repository that owns the behavior, then release and
  validate packages in dependency order.
- Before release, run Gravitas `Release`, `ReleaseLean`, coverage, replay,
  allocation, and relevant benchmark gates against package dependencies.
- Keep volatile test and coverage counts in generated reports or dated
  verification records rather than this active section.

### Ordered Queue

### GRV-Issue-096 - Extreme-scale capsule initial mesh sweep can miss a genuine overlap

- **Status:** Supported geometry range decision; extreme-radius algorithm work
  deferred. The intended workloads do not require billion-unit colliders.
- **Confirmed:** 2026-10-07 in the isolated local-stack Release
  `CapsuleInitialContact_WithUnavailableTriangleCharts_ShouldRetainGeometricFallback`
  experiment. An ordinary-radius source control passes; the extreme
  radius version returns false before initial-contact normal resolution.
- **Reproduction:** Let `r = Fixed64.FromRaw(long.MaxValue / 2)` and `m` be
  `Fixed64.MaxValue`. Initialize a capsule of radius `r`, full height `m`,
  centered at `(m - 4, m - 4, 0)`. Initialize a concave triangle with local
  vertices `(m, 0, -1)`, `(m, 0, 1)`, `(m, 5, 0)`, origin
  `(-r - 3, m - 4, 0)` and identity rotation. Prepare a unit rightward source
  sweep through `ConvexSweepQueryWorker`; `TrySweepPreparedSource` returns
  false. Its plane is only `r - 1` from the capsule center, and all triangle
  vertices lie strictly within the radial volume, proving initial overlap.
- **Boundary:** Both collider centers and the proposed source center remain
  representable. The capsule's core length is one raw unit. Its radius is
  extreme, while the plane overlap is only one world unit; this is distinct
  from #094's source-side and reduction defect. The generic candidate/GJK path
  before that resolution is unchanged by #094.
- **Follow-up:** Define a practical supported geometry range and consider hard
  admission limits consistently across radii, extents, authored scale and
  offsets. Do not infer a geometry-size promise from the full Fixed64 position
  domain: ordinary-sized shapes at large coordinates remain a distinct useful
  contract. If extreme geometry remains supported, attribute the miss to
  candidate admission or bounded GJK scaling before changing contact policy.
- **Evidence:** Ignored `artifacts/grv-issue-094/rotational-boundary-third.log`
  retains the isolated failure. The experimental fixture was removed from the
  maintained suite after recording this independent boundary.

### GRV-Issue-095 - Discrete mesh-cone contacts can choose an artificial triangulation-seam exit

- **Status:** Active; convex geometry refinement implemented. The surface-manifold
  architecture is approved; its [implementation plan](2026-10-08-surface-contact-manifold-plan.md)
  specifies the remaining runtime work and joint #095/#099 validation gates.
- **Confirmed:** 2026-10-07 during #094 geometry review, in four isolated
  local-stack Release cases: convex/concave flat quad targets and both windings.
- **Reproduction:** A default cone centered at `(0, -1/4, 0)` overlaps a flat
  quad at Y zero spanning `[-2, 2]` in X/Z. Its base is at Y `-3/4` and apex at
  Y `1/4`. `CollisionDetection.DoCollisionCheck` selects an oblique normal near
  `(-0.63246, -0.44721, +/-0.63246)` instead of the whole surface's downward
  exit. Any translation shorter than `1/4` retains a plane crossing strictly
  inside the quad; downward translation of `1/4` removes it. The diagonal is an
  internal triangulation seam, not an exposed mesh edge.
- **Root cause:** The existing discrete mesh-cone reducer compares individual
  two-sided triangle contacts. A triangle's minimum exit need not leave the
  union of adjacent faces, and its seam normal gives horizontal movement a
  spurious normal component. This discrete path is unchanged by #094's sweep
  contact fix.
- **Follow-up:** Resolve whole-surface contact selection without suppressing
  genuine exposed-edge contacts or treating concave meshes as convex solids.
  Audit other curved primitive mesh reducers for the same union boundary.
- **Refinement:** Prepared scaled geometry retains exact welded patches and
  complete exposed boundaries. Sufficient minimum-face proofs cover ordinary
  and tilted interiors. A trusted filled convex patch also uses its complete
  face, perimeter-edge and corner normal fan, retaining exact cross-feature
  ranking and paired witnesses. Small-square regressions distinguish a real
  perimeter exit from both a covered diagonal and an incorrect face override.
- **Remaining reproduction:** On 2026-10-08, an isolated local-stack Release
  diagnostic adds a thin coplanar tab to the X/Z square `[-1/5, 1/5]`.
  The tab spans X `[1/5, 1/4]` and Z `[-1/20, 1/20]`. The original square is
  a subset and proves that no translation shorter than `1/4` clears the union;
  the downward `1/4` exit still clears all Y-zero faces. The patch is nonconvex,
  both sufficient proofs decline, and the triangle reducer still selects an
  artificial seam normal approximately `(-0.31405, -0.44721, -0.83748)`.
  This is the same unresolved union boundary, not a new issue.
- **Decision refinement (2026-10-08):** After the owner delegated the choice to
  deterministic, accurate physical behavior, source and independent math/solver
  review favor topology-aware surface manifolds. One global minimum escape
  vector is a depenetration result and cannot preserve independent impulse and
  torque constraints. The confirmed #099 wall control demonstrates the runtime
  consequence. The [surface-contact plan](2026-10-08-surface-contact-manifold-plan.md#design-contracts-and-rationale)
  specifies exact finite-domain admission, grouped contact storage, response and
  compatibility boundaries. The existing convex minimum-exit helpers remain
  valid geometric owners; their global-minimum witness proofs cannot simply be
  reused after masking features. Finite clipping is needed even when neither
  global face-support projection belongs to the patch and no perimeter touches
  the cone. Do not apply convex corner charts to reentrant vertices or replace
  the tab failure with an unconditional infinite-face override. Keep #095 active
  through the corresponding geometry and full-loop implementation gates.
- **Verified refinement (2026-10-08):** Local-stack Gravitas Release passes
  4,852 full-suite tests plus the subsequently added exposed-nonconvex-edge
  regression; ReleaseLean passes all 4,788 tests. Combined Release OpenCover
  evidence covers 56,874/56,874 lines, 16,608/16,608 branches and 5,454/5,454
  fully covered methods; ReleaseLean covers 56,872/56,872, 16,608/16,608 and
  5,453/5,453 respectively. Raw sequence, branch and method points are all
  covered. FixedMathSharp also retains exact 100% in both configurations:
  Release combines its full suite and focused final rim-gap regression;
  ReleaseLean passes 4,491 core and 49 Chronicler bridge tests. Focused cases
  exercise both windings, triangulation changes, exact touch/one-raw gaps,
  small real edges/corners, rigid poses, full-loop frictionless seam motion,
  failed scale preparation and repeated scale quantization. Existing scale
  allocation guards and warmed dispatch pass at zero managed bytes. Topology
  preparation reuses retained SwiftCollections sorting buffers and publishes
  candidate metadata atomically with geometry.
- **Measured refinement (2026-10-08):** Windows x64, i7-9700K, .NET 8 Release,
  local stack; BenchmarkDotNet short job with three warmups and five measured
  iterations. The old Gravitas reducer at `f5ed8e4` and the current reducer both
  consume the same current FixedMathSharp source. Surface means in microseconds:

  | Fixture | Old triangle reduction | Current patch reduction |
  | --- | ---: | ---: |
  | Quad interior | 333.6 | 30.85 |
  | Subdivided interior | 1,261.2 | 84.75 |
  | Subdivided tilted | 6,266.1 | 243.46 |
  | Quad exposed edge | 142.1 | 501.45 |
  | Small interior | 345.1 | 549.00 |
  | Small exposed edge | 345.0 | 676.54 |
  | Single triangle control | 162.8 | 157.67 |

  All rows allocate zero managed bytes. The old interior/small-patch outputs
  select covered seams, so their lower times do not establish equivalent
  correctness. The real quad-edge control does expose a cost increase. Complete
  convex fallback includes an intersecting-seed query and full perimeter feature
  ranking; reuse or defer that seed work only with exact witness admission and
  cross-feature ranking intact. Keep this refinement inside #095 while its
  surface-contact contract is being specified. The nine original triangle-contact
  controls remain within their preceding measured range, approximately 21–882
  microseconds, with zero bytes allocated.
  Evidence is retained under ignored `artifacts/grv-issue-095/`:
  `baseline-common-stack/results`, `after-final/results`,
  `grv-release-verified-opencover-report` and `grv-lean-final-gates.log`.
  Reproduce by building the benchmark project in local-stack Release, then
  running its DLL with `--filter '*MeshConeContactBenchmarks*'
  '*MeshConeSurfaceContactBenchmarks*' --job short --warmupCount 3
  --iterationCount 5`; the baseline capture needs only the surface filter.
  Both repositories' API documentation builds with warnings as errors; local
  links pass in 195 Gravitas and 116 FixedMathSharp generated API pages and all
  edited Markdown documents. Independent topology and exact-contact reviews
  found no remaining convex correctness or retained-buffer ownership defects.
- **Evidence:** Ignored `artifacts/grv-issue-094/DiscreteConeSeamAudit.cs` and
  `audit-scalar.log` retain the four failures. The temporary diagnostic was
  removed from the maintained suite.
  Current ignored `artifacts/grv-issue-095/ConcavePatchTabAudit.cs` and
  `concave-tab-audit.log` retain the nonconvex closeout blocker.

### GRV-Issue-097 - Sphere and capsule mesh manifolds retain internal-seam constraints

- **Confirmed:** 2026-10-08 against the unchanged #095 baseline, with both
  windings of a concave Y-zero quad spanning `[-2, 2]` in X/Z and diagonal
  `X+Z=0`. A radius-`1/2` sphere or vertical capsule centered at
  `(3/10, -1/4, 3/10)` overlaps both triangles strictly inside the surface.
- **Reproduction:** Both manifolds retain a face contact and a second contact
  against the covered diagonal. The sphere's second normal is approximately
  `(0.60921, -0.50767, 0.60921)`; the capsule's is approximately
  `(0.70711, 0, 0.70711)`. Inspecting only the primary contact hides this defect.
- **Root cause:** Per-triangle curved contacts are individually valid but the
  manifold also retains a triangle-only escape into its neighbor. The solver
  processes every retained contact, so a covered seam becomes a lateral
  constraint. The tested finite-cylinder controls retained only face normals;
  that result does not establish cylinder conformance for other geometry.
- **Follow-up:** Reduce curved contacts over the actual surface, preserving
  exposed edges, holes and noncoplanar faces. Verify every manifold contact and
  frictionless full-loop movement, including winding and triangulation parity.
- **Evidence:** Ignored `artifacts/grv-issue-095/CurvedMeshSeamParityAudit.cs`
  and `parity-baseline.log` retain the four failures and cylinder controls.

### GRV-Issue-098 - Mixed curved slab mesh contacts can select a covered seam

- **Confirmed:** 2026-10-08 against the unchanged #095 baseline, with both
  windings of the same concave quad. A radius-`1/2` circle slab or positive-core
  stadium slab centered at `(3/10, -1/4, 3/10)`, with mixed half-thickness
  `1/2`, intersects the surface strictly inside its perimeter.
- **Reproduction:** `CollisionDetectionMixed.TryCollide` selects the planar
  normal approximately `(0.70711, 0, 0.70711)` and depths approximately
  `0.07574` (circle) or `0.16412` (stadium with total height `5/4`). The complete
  flat surface requires a downward `1/4` exit; its covered diagonal cannot
  provide a planar escape.
- **Root cause:** The mixed mesh reducer selects the shallowest individual
  triangle/slab exit without proving that it leaves the adjacent surface.
- **Follow-up:** Establish mixed surface reduction for curved slabs with genuine
  boundary controls, then audit embedded AABB/polygon prisms separately. Keep
  the plane-constrained 2D response and 3D-to-2D normal convention explicit.
- **Evidence:** The same ignored audit and log retain all four mixed failures.

### GRV-Issue-099 - Cone mesh reduction omits independent wall constraints

- **Confirmed:** 2026-10-08 against the committed #095 refinement, in four
  failing local-stack Release diagnostic cases and two passing separate-wall
  controls. Both windings are exercised.
- **Reproduction:** Two finite quads meet at an ordinary concave crease, X=0
  with Z in `[0,4]`, and Z=0 with X in `[0,4]`; both span Y `[-4,4]`. A
  default height-1/radius-1/2 cone centered at `(3/10,0,3/10)` intersects both.
  Combined in one concave mesh, detection retains only the Forward normal at
  depth approximately `1/5`, omitting the Right normal. With explicit static
  wall bodies, zero gravity/air density, frictionless materials, frozen cone
  rotation, manual grounding and incoming velocity `(-1,0,-1)`, one complete
  host step leaves velocity `(-1,0,0)`: approach into the X wall continues.
  The same surfaces as separate static colliders pass both approach checks.
- **Root cause:** `TryFindMeshConeTriangleContact` reduces all triangles and
  patches to one shallowest contact; `DoMeshConeCheck` replaces the manifold
  with that result. The single-contact reduction already existed before #095's
  convex refinement, as verified in `f5ed8e4`. This independent noncoplanar
  contact-loss boundary is not caused by the new coplanar topology.
- **Follow-up:** Preserve independent geometric surface constraints and stable
  finite witnesses. Address with the
  [surface-contact plan](2026-10-08-surface-contact-manifold-plan.md#design-contracts-and-rationale),
  including contact capacity, warm starts and response weighting. Do not count
  arbitrary extra triangle samples as a solution.
- **Evidence:** Ignored `artifacts/grv-issue-095/MeshConeSurfacePolicyAudit.cs`
  and `surface-policy-audit.log`; the temporary diagnostic was removed from the
  maintained suite. The two passing controls use the same geometry, materials,
  motion and host loop, changing only whether the walls share a mesh collider.

### GRV-Issue-100 - Contained cylinder and cone sphere sweeps invert outward surface normals

- **Confirmed:** 2026-10-08 during the surface-manifold phase 1 producer audit.
  Source and a maintained cylinder expectation establish the inward initial-
  overlap normal; a dedicated full-loop regression remains follow-up work.
- **Reproduction:** Resolve `ContinuousCollisionContactPolicy` for a sphere
  center strictly inside an ordinary finite cylinder or cone. Both direct
  primitive branches negate the nearest outward surface normal when signed
  distance is negative. For a height-1/radius-1/2 cylinder and center `(1/10,0,0)`,
  the emitted normal is Left instead of the nearest surface's Right. A cone of
  the same dimensions with center `(1/100,-1/10,0)` similarly emits the inward
  lateral normal. The sweep worker admits contained starts at distance zero.
- **Root cause:** `ContinuousCollisionContactPolicy.TryResolveSweptSphereContact`
  treats the signed-distance containment flag as a normal reversal. Cylinder
  and cone outward normals remain escape directions for contained points.
  Capsule and compound primitive contact paths retain outward normals instead.
  Closing classification uses the hit normal, so the direct primitive policy
  can classify escape motion as closing while permitting motion farther inward.
- **Follow-up:** Align direct primitive initial-overlap contact orientation
  with the solid-volume contract. Preserve admitted target anchors, including
  conceptual surfaces outside the scalar domain. Add ordinary-size full-loop
  entering/leaving tests, direct/compound parity and swept-query normal checks;
  do not conflate solid containment with two-sided mesh surface contacts.
- **Evidence:** Maintained
  `FiniteAxisProjectionWorkerTests.SweptSphereCylinderContact_WithUnrepresentableCap_ShouldRetainTargetAnchor`
  currently expects Down for a contained point whose nearest conceptual cap
  has outward Up normal. The anchor-retention assertion remains useful; its
  normal expectation codifies the inversion. The existing
  `SweptSphereWorker_WithConeStartingOverlap_ShouldReturnZeroDistance` checks
  distance and center only, leaving normal orientation unverified. Phase 1
  fixes the discrete cone/sphere and cylinder/sphere producers; it does not
  change this separate swept-contact policy.

Remaining measured performance costs are tracked in the benchmark backlog.

## Resolved Issues

### GRV-Issue-094 - Non-sphere convex CCD can block separation from the back of a mesh face

- **Resolved:** 2026-10-07. Generic convex mesh sweeps retain geometric initial
  contact normals independently of travel and winding. CCD admits closing
  features before triangle, source-part and target-part reduction, allowing
  separation without hiding a blocking sibling.
- **Original reproduction:** A unit dynamic cuboid centered at `(0, -1/2, 0)`
  touches the underside of an upward-wound static concave quad at Y zero.
  With a one-second step, zero gravity/air density, frictionless materials,
  manual grounding and continuous CCD, downward unit velocity formerly left
  its center at `-1/2`; the complete host loop now reaches the expected `-3/2`.
- **Root cause and fix:** The initial generic query normal retained authored
  winding, and CCD re-queried an on-surface witness that had lost the source
  side. The prepared worker now resolves the selected leaf's complete triangle
  or hull contact, retains exact edge/vertex separation, and prefers a
  supporting face when its projection certifies one. An uncertain GJK
  intersection requires complete contact geometry or an exact support
  certificate; it cannot fabricate a fallback normal. CCD consumes this
  geometric normal directly. Closed hulls retain solid-volume containment;
  curved hull sources rank face exits without claiming a global edge-axis MTV.
- **Rotational boundary:** The existing interval search can exclude a
  tangential/separating contact only when a stationary target's full support
  plane and literal cardinal quaternion/angular components certify the entire
  interval. Full mesh and compound target support includes every relevant
  vertex/leaf. General rotations, moving targets, compound sources and curved
  target leaves retain conservative search. The centered-sphere primitive
  rule remains intact.
- **Upstream ownership:** FixedMathSharp owns exact anchor projection ranking
  and inclusive triangle projection containment. Its existing closest-point
  frame conversion now preserves independent translation and exact residual
  metadata. Gravitas composes these internal math owners without copying limb
  arithmetic or moving physics policy upstream.
- **Validation:** Local-stack Release **4,794** and ReleaseLean **4,729**
  Gravitas tests pass. Exact line/branch/fully covered method counts are
  **56,591/56,591; 16,476/16,476; 5,436/5,436** and
  **56,589/56,589; 16,476/16,476; 5,435/5,435**; raw OpenCover checks pass.
  FixedMathSharp Release **4,464** and Lean **4,443** core tests plus **49**
  Chronicler bridge tests per configuration pass, also with exact 100% line,
  branch and fully covered method coverage. Regressions cover both windings,
  contact sides, primitive/mesh/compound source and target leaves, relative
  motion, rotational crossing/tangency, thin-cylinder rounded GJK admission,
  one-raw feature gaps, unmaterializable witnesses and warmed allocation gates.
- **Performance and follow-ups:** Ordinary impacts and dense concave sweep
  controls remain near baseline with **0 B/op**. Complete initial-contact
  resolution has a measured correctness cost tracked as **GRV-Benchmark-026**.
  The unchanged discrete cone seam reducer is **#095**; the extreme-radius
  admission boundary is **#096**, a supported-range decision. Ignored evidence
  remains under `artifacts/grv-issue-094`. Release FixedMathSharp first, then
  revalidate Gravitas against released packages before the Gravitas release.

### GRV-Issue-091 - Initially overlapping mesh sphere sweeps can classify an overhead surface as support

- **Resolved:** 2026-10-07. Mesh sphere sweeps retain zero-distance initial
  hits and target surface anchors, with a geometric anchor-to-sphere normal
  independent of travel. Grounding rejects the original body-backed overhead
  quad and can select a farther genuine floor.
- **Original reproduction:** A radius-half sphere at the origin probed down
  from Y `1/2` while already overlapping a downward-wound concave quad at Y
  `3/4`. The old hit normal was upward despite its overhead surface witness.
- **Root cause and fix:**
  [`ContinuousCollisionContactPolicy`](../../src/Gravitas/CollisionHandling/Continuous/ContinuousCollisionContactPolicy.cs)
  flipped mesh normals against travel. Reuse FixedMathSharp's exact anchor
  direction operation instead: faces, edges, vertices, one-raw gaps and
  full-domain offsets retain the sphere's geometric side. Only exact
  coincidence uses the selected authored face normal. Both mesh modes remain
  two-sided triangle-surface sweeps, including the interior side of a closed
  convex mesh; solid-volume containment is not inferred.
- **Compound ownership and ordering:** Reuse the existing nearest-surface
  reducer and retain its selected part's exact anchor and outward primitive
  normal. This avoids duplicate searches, re-selection at shared boundaries,
  and rejection merely because a valid part witness cannot enter the parent
  scalar frame. Mesh parts use the same sphere-side rule. Triangle-index,
  authored-part and collider-ID ordering remain deterministic.
- **Validation:** Local-stack Release **4,687** and ReleaseLean **4,622** tests
  pass. Exact line/branch/fully covered method counts are respectively
  **56,385/56,385; 16,370/16,370; 5,427/5,427** and
  **56,383/56,383; 16,370/16,370; 5,426/5,426**; raw OpenCover checks also pass.
  Regressions cover overhead rejection, floor selection, windings, travel,
  tiny/coincident starts, edge/vertex normals, compound ties, far-domain
  witnesses, allocation-free queries and complete-loop sphere CCD admission.
  The public tolerance-boundary rejection guard remains covered by a coherent
  finite-axis fixture. No upstream source changes were required.
- **Benchmark evidence:** Five measured Release iterations on this host with
  local dependencies and a two-triangle target: direct ordinary sweeps
  **45.18 → 47.11 μs**, direct initial overlaps **16.39 → 19.87 μs**,
  compound ordinary sweeps **48.69 → 47.32 μs**, and compound initial overlaps
  **19.96 → 19.78 μs**. All remain **0 B/op** after warmup. Exact feature-side
  resolution adds about 3.5 μs to the tiny direct initial-overlap fixture;
  compound witness reuse keeps its costs near baseline. Captures are under
  ignored `artifacts/grv-issue-091/{baseline,final-benchmark}`.
- **Review boundary:** Sphere CCD already consumes these corrected witnesses
  directly. Generic convex CCD retains its separate admission path; its
  pre-existing mesh-side gap is recorded as #094.

### GRV-Issue-093 - 2D initial-overlap ray fallback normals can classify a wall as sloped support

- **Resolved:** 2026-10-07. Contained-start 2D rays retain distance zero and the
  start-point witness, with an outward geometric separation normal independent
  of travel direction. Automatic grounding now rejects the wall and selects the
  farther eligible floor at both zero and default support thresholds.
- **Original reproduction:** A radius-half circle at `(0, 1/2)` casts downward
  for length 3. A static AABB centered at `(-1/2, 0)`, size `(1, 4)`, contains
  the start on its right face. Its old normal raw components were
  `(3037000500, 3037000500)` instead of the geometric normal `(1, 0)`, admitting
  the wall as sloped support. This query defect predated the body-policy hardening.
- **Root cause:**
  [`QueryDetection2D.TryRaycastPrepared`](../../src/Gravitas/Queries/2D/QueryDetection2D.cs)
  used a center-radial fallback before reaching convex intersection geometry.
  The defect belonged to Gravitas; existing upstream point-contact geometry
  supplied correct normals.
- **Fix and ordering:** Reuse circle radial and centered-capsule normal owners.
  Convex leaves use a focused FixedMathSharp internal point-normal operation
  that shares exact axis/depth ranking without materializing depth or anchors.
  Validated nondegenerate points need only face axes; general contacts retain
  vertex axes and their degenerate-boundary behavior. Equal convex features
  retain geometric order, circle centers use world +X, and capsule centerlines
  use rotated local +X. Compound ties retain the first authored part and public
  collider ordering remains distance then ID. Unsupported custom geometry
  cannot fabricate a contained-start normal.
- **Scaled-boundary admission:** Review reproduced positive scaling that rounds
  a valid thin polygon into a line, including compound parts and count-changing
  recorded loads. The polygon owner now validates changed scaled vertices
  before publication and retains committed geometry when replacement admission
  fails. Successful count changes restore matching spare buffers; ordinary
  pose changes reuse validation and remain allocation-free.
- **Verification:** Initial query/grounding regressions failed 18 of 20 cases;
  three scaled-boundary cases and the loaded replacement case also reproduced
  their defects before the corresponding fixes. Final local-stack Release and
  ReleaseLean full suites pass 4,648/4,583 Gravitas tests and 4,481/4,460
  FixedMathSharp solution tests, with exact 100% reachable line, branch and
  fully covered method coverage in both repositories. Combined raw OpenCover
  counts independently confirm complete sequence, branch and method coverage.
  Regressions cover boundary/interior starts, scalar limits, rotated geometry,
  winding, feature/compound ties, rejected geometry and recovery, grounding and
  warmed allocations. Shared replay expectations and schemas are unchanged;
  both core target frameworks build in both profiles.
- **Measured refinement:** Ten Release query cases on Windows x64 all retain
  warmed 0 B/op. Against the first correct full-contact implementation,
  contained box/polygon/compound means fall from 11.8–12.5 to 5.5–5.7 microseconds;
  capsule/circle means fall from 4.592/2.757 to 1.120/1.056 microseconds. Outside
  controls remain comparable across captures. The original roughly-one-
  microsecond radial shortcut did less geometry work and returned incorrect
  normals. These five-iteration samples assess the isolated path, not complete
  gameplay-frame capacity.
- **Documentation and review:** Both repositories' DocFX builds pass with zero
  warnings/errors. Generated API local links and 127 wiki/tracker local links
  pass validation. Independent final review has no actionable findings.
- **Compatibility and evidence:** Public query signatures and hit layout remain
  unchanged. The new FixedMathSharp internal consumer is a coordinated ABI
  event: release FixedMathSharp first, then revalidate Gravitas against released
  packages before release. Final coverage, benchmark and documentation logs are
  under ignored `artifacts/grv-issue-093/`; the original diagnostic remains in
  `artifacts/grv-issue-092/ray-normal.log`. GRV-Issue-091 remains independent.

### GRV-Issue-092 - 2D ground-normal policy admits non-support normals and retains stale probe eligibility

- **Resolved:** 2026-10-07 as the focused 2D parity follow-up to GRV-Issue-090.
  The existing body owner shares one support-normal predicate across query
  probes, discrete contacts, motion projection and callback revalidation.
- **Fix:** `GroundMinNormalDot` defaults to one half and accepts `[0, 1]`.
  Support requires a strictly positive normalized dot with resolved planar up;
  its slope boundary is inclusive. Threshold or effective-up changes invalidate
  automatic probe timing and pending contact candidates and wake the body.
  Equal assignments, unused fallback changes and same-direction gravity changes
  preserve sleep. Manual support remains host-owned. Callback revalidation
  prevents newly ineligible support from surviving publication, and motion
  preparation ignores normals rejected by the current automatic policy.
- **Recorded state:** Validate the threshold before publishing loaded body
  state, restore gravity and policy backing fields without waking saved bodies,
  and retain the existing transient support/ownership rebuild. Missing sparse
  values restore one half. Recorded fields and replay-hash layout are unchanged;
  no fixture hashes or schema versions changed.
- **Reproduction and verification:** The initial regression run failed 14 of
  15 cases before the fix, reproducing invalid threshold admission, non-support
  normals at zero, stale motion projection and sleeping membership. Three
  callback cases also failed before implementation. Final full local-stack
  Release/ReleaseLean suites pass 4,609/4,544 tests with exact 100% reachable
  line, branch and fully covered method coverage, including replay fixtures and
  the warmed allocation regression. Cases cover ray/swept-circle selection,
  contacts, inclusive boundaries, awake/sleeping changes, callbacks changing
  the current or later body's policy, manual support and both record transports.
  Raw OpenCover counts independently confirm complete sequence, branch and
  method coverage. Logs and fresh reports are under ignored
  `artifacts/grv-issue-092/`.
- **Allocation and documentation gates:** Existing nearest-accepted-hit and
  complete automatic-probe benchmarks pass their support/witness checks at
  64 and 1,024 pairs, with warmed 0 B/op in all four cases. This short sample
  verifies allocation and behavior, not a before/after speed claim. Both core
  target frameworks build in Release and ReleaseLean; DocFX completes with
  zero warnings/errors, and changed wiki pages have valid local links.
- **Witness follow-ups:** GRV-Issue-091 remains active for the separate 3D
  overhead mesh witness. GRV-Issue-093 subsequently resolved the separately
  reproduced 2D contained-start ray defect in the query owner.

### GRV-Issue-090 - 3D automatic swept-ground probes accept vertical-wall contacts

- **Resolved:** 2026-10-07. The shared 3D body ground-hit validator now requires
  an upward candidate normal for both ray and swept-sphere probes.
- **Fix:** `GroundMinNormalDot` defaults to one half and accepts `[0, 1]`.
  The normalized up-dot boundary is inclusive; horizontal, downward-facing and
  zero normals are always rejected. Actual automatic policy changes invalidate
  the cached probe and wake the body; equal assignments and host-owned manual
  support are preserved. Populate validates before publishing state and restores
  the backing policy without changing saved sleep or probe timing. Omitted
  record values use one half. Raw query hits and diagnostics remain geometry
  witnesses, and the sorted scan continues to a farther eligible support.
- **Verification:** Local-stack Release and ReleaseLean full suites pass with
  exact 100% reachable line, branch and method coverage. Regression tests first
  reproduced the height jump at both translated heights, then passed without
  changing CCD's `7/16` stop position. Tests cover automatic/explicit swept
  shapes, inside-support ray/sweep witnesses, farther-floor selection, inclusive
  slope thresholds, invalid values, awake/sleeping runtime changes, manual
  support, both record transports and replay-hash inclusion. All shared replay
  raw state, events and query expectations remain unchanged. The existing
  64-body 3D ground-probe benchmark passed all eight ray/sweep, target-count and
  supported/miss cases with warmed 0 B/op. Its short timing sample is an
  allocation/behavior check, not a before/after speed claim. API documentation
  builds with zero warnings and errors.
- **Compatibility:** Additive body setting and record key; `body.3d` replay-hash
  section advances from 6 to 7. Shared 3D-containing fixture hashes were refreshed
  only after checking every non-hash field; the pure 2D fixture is unchanged.
- **Parity audit:** 2D already filters support normals against its resolved up
  direction and records planar support without the 3D height snap. Existing
  filtering, manual support and full-loop tests pass in both profiles. The
  subsequent threshold and runtime-cache parity gaps are resolved in
  [GRV-Issue-092](#grv-issue-092---2d-ground-normal-policy-admits-non-support-normals-and-retains-stale-probe-eligibility). The
  separate shared mesh initial-overlap normal defect is tracked as
  [GRV-Issue-091](#grv-issue-091---initially-overlapping-mesh-sphere-sweeps-can-classify-an-overhead-surface-as-support).
- **Confirmed:** 2026-10-06 during full-lifecycle replay fixture review. A
  focused local-stack Release diagnostic reproduced the same unwanted height
  change at initial heights 0 and 4; both diagnostic cases passed assertions
  describing the defect. No runtime fix is included in the replay work.
- **Reproduction:** At 8 Hz with gravity, air density, damping and minimum speed
  zero, create a mass-one radius-half dynamic sphere at `(-2, h, 4)` and a
  static cuboid at `(1, h, 4)` with size `(1/8, 4, 4)`. Set maximum speed/fall
  speed to 64, enable continuous collision on the sphere, apply impulse
  `(32, 0, 0)`, and call `Simulate()` then `LateSimulate()`. CCD correctly stops
  X at `7/16`. Automatic grounding then marks the sphere grounded against the
  wall and raises its body height from `h` to `h + 1/2`, despite zero vertical
  velocity.
- **Exact witness:** For `h = 4`, the radius-half downward ground sweep starts
  at `(7/16, 9/2, 4)` and ends at `(7/16, 4, 4)`. It accepts wall collider ID 1
  at distance zero with point `(15/16, 9/2, 4)` and normal `(-1, 0, 0)`. The
  final body pose is `(7/16, 9/2, 4)`. For `h = 0`, the same relative witness is
  `(15/16, 1/2, 4)` and the final pose is `(7/16, 1/2, 4)`. The probe is tangent
  to the vertical wall at its start; this is not upward support or an implicit
  world-floor contract.
- **Source chain at confirmation:**
  [`ResolveGroundProbeMode`](../../src/Gravitas/Core/3D/SolidBody.Grounding.cs)
  selects a swept sphere for sphere bodies. `TryFindGroundHitWithSweptSphere`
  accepted the first eligible hit; `IsValidGroundHit` checked self identity,
  physical filtering and static/kinematic mobility, but did not reject a
  horizontal support normal. `ApplyGroundedHeightOrReset` then publishes the
  accepted witness Y as body height. The accepted query witness is valid wall
  geometry; the incorrect support classification and height publication are
  owned by 3D grounding.
- **Evidence:** The temporary diagnostic source is retained under ignored
  `artifacts/replay-conformance-phase2/grounding-witness/GroundingWitnessScratchTests.cs`;
  exact observed console excerpts and the invocation are in `diagnostic.log`
  beside it. The diagnostic invocation used only the console logger and produced
  no TRX file; unrelated replay TRX captures are not evidence for this defect.
  The tracked temporary test was removed after the two-case run. Shared replay
  fixtures use explicit manual grounding to isolate their CCD expectations from
  this separately tracked runtime behavior.
- **Resolution evidence:** Serial owner gates and raw OpenCover/Cobertura
  captures are retained under ignored `artifacts/grv-issue-090/`. `red.log`
  records the two original failures; `cache-red.log` records stale awake and
  sleeping support before cache invalidation was added. Temporary diagnostics
  were removed from the tracked test suite.

### GRV-Issue-089 - Runtime mass changes leave inertia or awake membership stale

- **Resolved:** 2026-10-04. Gravitas body owners now enforce one runtime mass
  mutation contract for pure 2D, 3D, and mixed membership. No upstream change is
  required; changes remain unstaged for review.
- **Confirmed:** 2026-10-02 during `GRV-Benchmark-023`. A registered 3D sphere
  changed from mass 1 to 2 retained its old inverse inertia and doubled the
  expected angular impulse response. Changing it to zero left rotation enabled.
  Pure 2D refreshed inertia but retained stale awake membership across zero.
  Original executable evidence remains in the ignored
  `artifacts/grv-benchmark-023/mass-mutation-probe.ps1` and `.json`.
- **Fix:** Actual active changes calculate replacement inertia before publishing
  mass, clear contact and connected-joint impulse caches and CCD state, wake,
  and synchronize pure/mixed awake membership. Non-positive mass disables
  applicable solver motion without changing role, freeze axes, registration,
  coordinates, pose, velocities, or already accepted accelerations. Equal values
  are no-ops. Inactive mass is configuration derived at initialization. Changed
  runtime values reject during fixed-step transactions/callbacks or after a
  registration reset. Recorded loads bypass the waking setter, refresh derived
  state, and clear caches while retaining saved sleep and pair/joint identity.
- **Compatibility:** `SolidBody.Mass` changes from a field to a property.
  Ordinary assignment/object-initializer source use is preserved; rebuild
  consumers and migrate ref/reflection access. The `"Mass"` record key and body
  schemas are unchanged. See
  [the migration guide](../MIGRATION.md#runtime-mass-changes) and
  [host contract](../wiki/HOST_INTEGRATION.md#runtime-mass-changes).
- **Verification:** Local-stack Release and ReleaseLean suites pass with 100%
  reachable line, branch, and method coverage in raw and rendered reports.
  Focused cases cover angular scaling, non-positive transitions, sleep/no-op,
  role/freeze preservation, custom math failure, stale registrations, callbacks,
  contact/joint cache ownership, transport restores, replay, pure/mixed
  candidate routing, prepared CCD trajectories/indexes/handoffs, and
  zero-allocation warmed mutations. The impulse-kernel boundary fixture now uses
  its existing explicit tensor helper instead of depending on stale sphere
  inertia after a tiny-mass assignment. Dated logs, coverage, and mutation-cycle
  measurements remain under ignored `artifacts/grv-issue-089`.
- **Measured mutation cost:** On this i7-9700K host, the complete registered
  `0 -> 2 -> 1` kilogram cycle costs `13.373 +/- 0.1671 us` for the 3D sphere
  fixture and `4.897 +/- 0.0438 us` for the 2D circle fixture, both `0 B/op`.
  Default-job measurements use two launches, five warmups, fifteen measured 250
  ms iterations, and CPU affinity `3`; these are three-assignment fixture costs,
  not a simulation-step budget. The API build and local-link checks pass.

### GRV-Issue-083 - 2D circle contacts compare saturated squared distances

- **Resolved:** 2026-09-30. The correctness repair and focused shared-owner
  optimization are complete; remaining exact-contact cost is tracked separately.
- **Confirmed:** 2026-09-25 at `e85cda8f1e42acd0669211a8fda21e99c1251d80`, with
  FixedMathSharp `6368582`, during the FMS-Issue-024 neighboring-solver audit.
  This is a discrete-contact defect, not an arbitrary 3D circle-distance defect
  or a recurrence of the repaired radial-query issue GRV-Issue-045.
- **Reproduction:** In `CollisionDetection2DTests`, use `Create2DContext` and
  `CreateBody` to initialize static `LSCircleCollider2D` bodies with radius
  `25000`, at `(0,0)` and `(40000,40000)`, zero rotation and unit scale.
  `CollisionDetection2D.BoundsOverlap` returns true: the AABBs overlap by
  `10000` on both axes. Exact squared distance is `3200000000`, greater than
  squared radius sum `2500000000`, so the circles are disjoint. Nevertheless,
  `CollisionDetection2D.TryCollide(first, second, out contact)` returns true
  with depth `3659.049988158513` and normal
  `(0.8631674575153738,0.8631674575153738)`. The equivalent
  `FixedBoundCircle.Intersects` correctly returns false.
- **Cause:** `CollisionDetection2D.TryCircleCircle` forms the center difference,
  radius sum and both squares in scalar `Fixed64`. Both squared values in this
  fixture saturate to `Fixed64.MaxValue` before comparison; subsequent square
  root, normal and depth calculations use the saturated distance. The normal is
  not unit length either. Bounds admission does not make these intermediate
  quantities representable.
- **Initial reproduction:** Built the then-current Release local-stack test
  project with zero warnings/errors, then executed its existing initialization
  helpers and actual narrow-phase dispatcher through a PowerShell 7.6.5
  reflection probe. No collider fields were fabricated. This was a direct
  contact reproduction, not a full simulation-step test or a completed
  regression-test matrix. A direct call to the existing FixedMathSharp zero-axis
  capsule contact query also correctly rejected the same separated geometry.
- **Implementation:** `TryCircleCircle` now reuses
  `FixedSegment2d.TryGetCenteredCapsulesContact` with zero core lengths, just as
  the 3D sphere path uses its dimensional counterpart. The explicit-axis
  overload avoids rotating irrelevant zero cores while preserving authored
  anchor frames, inclusive tangency and the coincident world-+X fallback.
  Complete upstream anchors and the conceptual-depth clamp flag are forwarded
  through the existing manifold/compound path. FixedMathSharp also optimizes
  this shared owner for both circle and sphere point cores, reuses narrower
  exact square roots and shares allocation-free anchor residual arithmetic. The
  public API and exact contact contract are unchanged upstream.
- **Regression evidence:** The original enabled .NET 8 regression fails against
  unchanged source; the expanded first matrix exposes 12 failures in 17 cases,
  and a further four-case raw matrix exposes three more failures. These cover
  scalar-square saturation/underflow, radius and center-span overflow, raw
  neighbors, rounded depth, conceptual clamping and rotated anchor ownership.
  The rotation fixture uses cardinal contact and exact quarter turns: diagonal
  normals are rounded Fixed64 values, not infinitely precise rational witnesses.
  Logs and TRX results are under `artifacts/grv083`.
- **Upstream verification, 2026-09-30:** FixedMathSharp passes 4,038 / 4,017
  core tests in Release / ReleaseLean, plus 49 Chronicler integration tests in
  each. The configured core/FluentAssertions Cobertura report is exactly 100%:
  Release has 52,626/52,626 lines, 12,088/12,088 branches and 3,934 covered
  methods; Lean has 52,719/52,719 lines, 12,088/12,088 branches and 3,930
  covered methods. No exclusions changed. Tests pin full-domain radius/center
  spans, pre-rounding clamping, raw neighbors, 2D/3D parity, product-root width
  boundaries, nonzero-core endpoint rounding and retained residuals after both
  overflow cancellation and axial double rounding. An obsolete residual-bound
  constant and its redundant range assertions were removed; exact-value
  regressions replace them. The focused Debug arithmetic/anchor run also passes
  all 125 cases.
- **Final validation, 2026-09-30:** Full Windows local-stack suites, rebuilt
  against the optimized FixedMathSharp source, pass: Release 4,363 tests and
  ReleaseLean 4,304 tests, with no failures or skips. Both configurations retain
  100% reachable line, branch and method coverage without exclusion changes. Raw
  Cobertura counts are 44,363/44,363 lines, 13,218/13,218 branches and 4,561
  covered methods in Release; Lean has 44,361/44,361 lines, 13,218/13,218
  branches and 4,560 covered methods. Both solution builds validate
  `netstandard2.1` and `net8.0` with zero warnings or errors; DocFX passes with
  warnings as errors. The new 21-case regressions include zero-allocation direct
  and manifold checks. Logs, TRX, coverage and benchmark artifacts are under
  `artifacts/grv083`; the final full-suite reports use `final-coverage-release`
  and `final-coverage-releaselean` in both repositories. This is source-mode
  Windows evidence, not released-package or Linux validation.
- **Performance disposition:** Shared-owner optimization removes 66.6-83.2% of
  the initial complete-query time in the six matched Gravitas fixtures, with
  zero allocation. Axis contact is 5.4363 -> 0.9125 microseconds; rotated
  contact is 5.8454 -> 1.6113 microseconds. Upstream sphere and nonzero-capsule
  controls also improve. The exact path still costs 8.6-24.0x the old incomplete
  scalar path; this is not scalar-cost parity or an accepted whole-frame budget.
  The direct-query investigation is now closed in
  [GRV-Benchmark-022](benchmark-signal-hardening-backlog.md#grv-benchmark-022--exact-pure-2d-circle-contact-cost)
  after further shared normalization improvement and a documented decision to
  retain the exact solver. Larger-scene partition/grounding scaling remains open
  in GRV-Benchmark-023; whole-frame cost has not been accepted. Independent
  correctness, numerical/evidence and Ponytail reviews have no outstanding
  findings.

### GRV-Issue-088 - Mixed capsule/nonzero-core capsule-slab contact retains incomplete directions

- **Confirmed:** 2026-09-29 while validating GRV-Issue-085 against Gravitas
  `04805b8` plus its local fix and FixedMathSharp `9833123` plus the shared
  penetration extraction. This is the unchanged nonzero-core slab path, not a
  regression introduced by the circle-slab repair.
- **Reproduction:** Reuse GRV-Issue-085's 3D capsule: radius 1, total height 22,
  center `(83/4,7/4,0)`, rotation `(0,0,-q,q)` where
  `q=Fixed64.FromRaw(3037000500)`. At the origin use an unrotated
  `LSCapsuleCollider2D(radius:10,height:22)` with half-thickness 1. Its planar
  core has length 2 along Z. At Z=0 the nearest cap-rim point is `(10,1,0)`; the
  closest 3D core endpoint is `(43/4,7/4,0)`. The squared gap is `9/8`, strictly
  greater than the 3D capsule's radius squared 1. Extending the slab core along
  Z cannot reduce this X/Y gap.
- **Observed:** The public mixed query returns true, depth
  `0.13923784578219056`, normal approximately
  `(-0.9870072698686272,-0.16067560226656497,0)`. The equivalent zero-core slab
  now correctly rejects through GRV-Issue-085's complete circle-slab route.
  Reproduced through the existing `MixedNarrowPhaseTests` initialization helpers
  using PowerShell reflection against the fresh Release local-stack assembly; no
  private geometry was fabricated.
- **Cause:** The removed `TryTestCapsuleCapsuleSlab` selected an incomplete
  direction set. Exact projections on those directions did not establish the
  whole shape's minimum penetration; a nonzero-core slab is not a single
  cylinder or a rounded 3D capsule.
- **Resolved 2026-09-30:** FixedMathSharp now owns a complete signed
  capsule/stadium-slab support query. Gravitas delegates to it, retains
  canonical anchors/materials, and removes capsule-only direction helpers. The
  enabled separated-rim regression, raw neighbors, whole-shape containment, tiny
  cores, oblique roots, wide/clamped inputs, runtime response and zero
  allocation pass. Both repositories retain 100% measured line/branch/method
  coverage in Release and ReleaseLean; Debug resource checks and both DocFX
  sites also pass.
- **Coordination:** The completed
  [Complete Capsule/Slab Contact plan](done/2026-09-29-complete-capsule-slab-contact-plan.md)
  retains the proof, matched baseline, full verification and review evidence.
  The correctness defect no longer reproduces. The user accepted the remaining
  curved-query cost for now on 2026-09-30. The subsequent
  [root-isolation refinement](done/2026-10-05-capsule-slab-root-isolation-refinement.md)
  closes GRV-Benchmark-021 on 2026-10-05 with verified gains and published
  remaining cost. The subsequent
  [circle/slab output refinement](done/2026-10-05-capsule-circle-cost-refinement.md)
  also closes GRV-Benchmark-020 on 2026-10-05. Acceptance of this correctness
  repair did not itself close either performance signal.

### GRV-Issue-085 - Mixed capsule/circle-slab contact bypasses the complete upstream query

- **Confirmed:** 2026-09-25 at `e85cda8f1e42acd0669211a8fda21e99c1251d80` with
  FixedMathSharp `6368582`; reproduced again on 2026-09-29 before repair.
- **Reproduction:** A 3D capsule of radius 1 and total height 22 (core 20),
  center `(83/4,7/4,0)`, rotation `(0,0,-q,q)` with
  `q=Fixed64.FromRaw(3037000500)`, faces a radius-10 circle slab at the origin
  with half-thickness 1. The closest core endpoint `(43/4,7/4,0)` has cap-rim
  squared gap `9/8 > 1`. Bounds overlap, but the shapes do not.
- **Old behavior / cause:** Mixed contact returned true with depth
  `0.19961997726932168`; the complete upstream query returned false. The mixed
  implementation selected only world up, capsule axis, their cross product and a
  closest-centerline direction. The equivalent zero-core planar capsule also
  misclassified the gap. Doubling slab half-thickness in `Fixed64` additionally
  shortened valid very tall circle slabs and could reject overlap.
- **Resolved:** 2026-09-29. Both circle slabs and zero-core planar capsule slabs
  now share FixedMathSharp's complete cylinder/capsule penetration owner. That
  owner retains full cylinder length wider than `Fixed64`, using the existing
  proved arithmetic widths. Its public contact API, exact feature selection, tie
  ordering and support anchors remain unchanged. Gravitas reverses the
  cylinder-first normal and keeps its canonical support/material/response path.
  The two obsolete capsule/circle direction helpers were removed.
- **Regression evidence:** `MixedNarrowPhaseTests.CapsuleCircle.cs` covers
  separated rims, exact tangency and one-raw penetration, oblique interior rims,
  admitted maximum half-thickness, conceptual-depth clamping and allocation-free
  warmed contacts. Before the repair, 14 of the 19 focused mixed cases failed;
  afterward all 237 mixed narrow-phase cases passed. Upstream wide-length and
  existing cylinder/capsule and cylinder-pair controls passed 243 cases. A
  subsequent two-raw-unit positive-core regression verifies that only an exactly
  zero core takes the circle route: the stadium's extra endpoint extent remains
  visible in its exact contact depth.
- **Verification:** FixedMathSharp passes 3,932 / 3,911 core tests in Release /
  ReleaseLean, plus 49 Chronicler integration tests in each configuration.
  Gravitas passes 4,335 / 4,276 tests. Both repositories retain exact 100%
  reachable line, branch and method coverage in both configurations, without new
  exclusions. Both full solutions build for netstandard2.1 and net8.0 with zero
  warnings/errors. The 15 focused wide-length Debug regressions also pass, and
  both DocFX sites build with warnings treated as errors. Independent
  correctness, arithmetic and Ponytail reviews have no outstanding findings.
  Coverage evidence is under `artifacts/grv085/coverage-release`,
  `coverage-lean`, `report-release` and `report-lean` in FixedMathSharp;
  Gravitas's final reports use `coverage-release-final`, `coverage-lean`,
  `report-release-final` and `report-lean` under the same artifact root.
- **Performance:** The repeated mixed benchmark remains allocation-free but is
  not an overall speedup: ordinary cap/side/zero-core costs rise, endpoint-rim
  contact costs about 108 microseconds and oblique interior-rim contact about
  1.19 milliseconds. Equivalent complete 3D controls have the same expensive
  curved-feature behavior. Before/after fixtures, repeated timings, limitations
  and the next shared-owner profiling step are captured in
  [GRV-Benchmark-020](benchmark-signal-hardening-backlog.md#grv-benchmark-020--complete-capsulecircle-slab-contact-cost).
- **Scope:** Nonzero-core planar capsule slabs remain a separate shape and were
  subsequently repaired as GRV-Issue-088. Package consumers require the matching
  FixedMathSharp release before Gravitas release validation; source-stack builds
  do not replace that gate.

### GRV-Issue-087 - Cone-volume queries reject an intersecting mesh when the apex cannot enter its scalar frame

**Resolved:** 2026-09-29.

**Fix:** The cone-volume reducer now passes the world-space cone and the mesh's
canonical frame to FixedMathSharp's minimum-axial triangle query. It no longer
uses failed scalar-frame conversion as a separation proof or rounds the inverse
rotation before classification. Scalar and rigid queries share edge/face
selection and the existing bounded conic reducer; no penetration solver or
second Gravitas geometry implementation was added.

**Regressions:** Closest/all-hit and both batch entry points return the expected
earliest axial witness for the original full-domain fixture, while the genuinely
separated mesh still rejects. Rotated/full-domain faces, exact apex contact,
finite caps, reversed winding, quaternion-sign equivalence, root-rounding ties
and allocation-free warmed queries are covered upstream. Independent review also
caught a near-unit-axis edge/face ranking mismatch; its failing exact witness
regression now passes using the existing wide-ratio conversion.

**Verification:** FixedMathSharp passes 3,917 / 3,896 core tests in Release /
ReleaseLean plus 49 integration tests in each configuration; core coverage is
100% reachable line, branch and method. The focused Debug arithmetic run passes
186 cases. Gravitas passes 4,318 / 4,259 tests with 100% reachable line, branch
and method coverage in Release / ReleaseLean. No exclusions were added. Both
full solutions build for netstandard2.1 and net8.0 in both configurations; both
DocFX sites pass warnings-as-errors. Independent correctness, arithmetic and
Ponytail reviews have no outstanding findings. Source-stack validation uses
`UseLocalLsfStack=true`; it is not published-package validation. Coverage/build
evidence is under `artifacts/grv087/final-*` in each repository.

**Performance:** The existing `OverlapConeAllAcrossConcaveMeshTargets` row
(`ColliderCount=64`) measures **3.440 ms before / 3.158 ms after**, about **8.2%
lower mean time**, with **0 B/op**. Both use Release, .NET 8.0.29, two launches,
five warmups and fifteen measured iterations on the same two-core-affinity host.
The first frame-preserving implementation measured 3.796 ms; retaining the
proven exact-translation path and reusing its scalar projection removed that
avoidable cost. An earlier optimized run measured 3.166 ms. These are this
ordinary query scenario's results, not a claim about every geometry workload or
the cost of newly admitted extreme-coordinate hits. Baseline source was Gravitas
`4d69d81` / FixedMathSharp `0323efb`; logs and BenchmarkDotNet reports are under
`artifacts/grv087/baseline*`, `current*`, `optimized*` and `final-benchmark*` in
Gravitas.

**Original evidence:**

- **Confirmed:** 2026-09-28 during GRV-Issue-086 caller review, from Gravitas
  `3471085` / FixedMathSharp `3f7a606`. This query path is independent of
  mesh/cone collision contact generation.
- **Reproduction:** In the existing `PhysicsScenarioBuilder` grid, initialize
  one bodyless concave `SurfaceApproximation` mesh with local vertices
  `(-3,-1,0)`, `(3,-1,0)`, `(0,1,0)`, identity rotation and origin `(3,0,0)`.
  Call `context.Query3D.OverlapCone` with apex
  `(Fixed64.MinValue + Fixed64.Two, 1/2, 0)`, direction `Vector3d.Down`, length
  `1`, end radius `Fixed64.MaxValue` and `PhysicsLayerMask.FromLayer(0)`. It
  returns false after admitting one collider and one mesh triangle. This cone is
  the same geometric volume as the original GRV-Issue-086 fixture;
  `(1,-1/2+2^-32,0)` is strictly inside the triangle and cone.
- **Cause:** `TryBuildConeHitForConcaveMesh` rejects when the query apex cannot
  be materialized in the mesh's local scalar frame. The geometric witness and
  its axial distance are representable; failure to represent an intermediate is
  not a separation proof. Both closest-hit and all-hit entry points share this
  reducer.
- **Original probe:** Direct public closest-hit query using existing Release
  local-stack assemblies in PowerShell/.NET `10.0.11`; internal counters were
  read only to establish broad-phase admission. No private geometry was
  fabricated. This was not a new build or the .NET 8 regression suite; the
  all-hit entry point had not yet been exercised at discovery.

### GRV-Issue-086 - Mesh/cone contact uses incomplete triangle sampling and scalar-frame rejection

**Resolved:** 2026-09-29. Full evidence and measured performance controls are in
[Complete Triangle/Cone Contact](done/2026-09-28-complete-triangle-cone-contact-plan.md).

**Fix:** FixedMathSharp owns the complete canonical-frame cone/triangle
relation. Gravitas now consumes one feature's normal, depth, clamp flag and both
anchors; the incomplete sampling and scalar-frame rejection branches are
removed. Touching is inclusive, genuine positive gaps reject, and the existing
closed-convex fallback and CCD policies are unchanged.

**Validation:** Both full source-stack solutions build for `netstandard2.1` and
`net8.0` in Release and ReleaseLean. FixedMathSharp passes 3,886 / 3,865 core
tests plus 49 integration tests per configuration; Gravitas passes 4,317 /
4,258. All retain 100% reachable line, branch and method coverage with no new
exclusions. The 47 focused upstream cone cases include raw-neighbor boundaries,
paired witnesses, full-domain and 1 MiB worker-stack regressions. All nine cone
and eighteen shared cylinder/capsule-slab benchmark rows are allocation-free;
fresh old-source controls detected no material shared-path regression. Both
DocFX sites pass warnings-as-errors; independent correctness and Ponytail
reviews have no outstanding findings. Source-mode validation is not published
package availability evidence.

**Performance:** Proof-based reductions cut base/side/apex costs by 78% / 89% /
67% versus the first complete solver. Some previously correct ordinary cases
remain slower than the incomplete predecessor, and curved/full-domain cases
remain expensive. Follow-up is **GRV-Benchmark-019**; the distinct minimum-axial
cone-volume query defect was separately resolved as **GRV-Issue-087**.

**Original evidence:**

- **Confirmed:** 2026-09-27 while verifying GRV-Issue-082, from Gravitas
  `0a6f480` with the local FixedMathSharp source stack. This is an existing
  cone-path defect, not introduced by the cylinder repair.
- **Reproduction:** Create a concave `SurfaceApproximation` mesh with local
  vertices `(-3,-1,0)`, `(3,-1,0)`, `(0,1,0)` at world `(3,0,0)` and identity
  rotation. Create an identity `LSConeCollider` with `Size=Vector3d.One`,
  `Radius=Fixed64.MaxValue`, and center
  `(Fixed64.MinValue + Fixed64.Two, 0, 0)`. Bounds overlap, but public
  `CollisionDetection.DoCollisionCheck(new CollisionPair(mesh, cone))` returns
  false.
- **Independent geometric proof:** Write `epsilon=2^-32` and `M=2^31`. The point
  `(1,-1/2+epsilon,0)` is strictly inside the triangle: its left and right edges
  at that height are `3/4+3*epsilon/2` and `21/4-3*epsilon/2`. Its distance from
  the cone axis is `M-1`, while the cone radius at that height is
  `(M-epsilon)*(1-epsilon)`, exceeding that distance by `1/2-epsilon+epsilon^2`.
  The miss is not a tangency convention.
- **Cause:** `CollisionDetection.Cone.cs` skips the admitted triangle when
  `coneCenterAnchor.TryGetLocalPointIn(...)` cannot materialize the center in
  the mesh's scalar coordinate range. The concave path has no fallback. A
  representability check is being used as geometric rejection.
- **Evidence:** The original cone row of
  `FiniteAxisMeshContact_WithOverlappingClippedBoundsAndUnrepresentableFrameOffset_ShouldReject`
  passed its incorrect false expectation in
  `artifacts/grv082-checkpoints/contacts-release.trx`. Independent review
  confirmed the proof above. The shared separated control now moves the local
  apex to `(6,1,0)`; that is a genuine gap, not the reproduction here. The
  original cylinder case is retained separately as a positive regression.
- **Expanded investigation (2026-09-28):** The same owner also fails at ordinary
  coordinates; this is not only a full-domain conversion defect. For an identity
  cone at zero with radius `1` and height `2`, use an identity concave triangle
  at zero with vertices `(4/5,0,0)`, `(4/5,-2,0)`, `(2,-2,0)`. Bounds overlap,
  but the dispatcher returns false. The point `(9/10,-19/20,0)` is strictly
  inside both: the triangle spans X from `4/5` to `137/100` at that height,
  while the cone's cross-section radius is `39/40`. The nearest triangle point
  to the cone center is outside the cone; the single cone support projected onto
  the triangle plane is outside the triangle. Neither failed sample proves
  separation.
- **Inconsistent contact evidence:** With the same radius-1, height-2 cone,
  triangle `(0,-2,-2)`, `(0,2,-2)`, `(0,0,2)` returns depth
  `0.4472135955002159`, normal `(1,0,0)`, mesh anchor `(0,0,0)` and cone anchor
  approximately `(0.4,0.2,0)`. The selected nearest-surface depth is not the
  penetration along the returned triangle-face normal; the witness displacement
  is not parallel to that normal and has the wrong signed orientation. The
  closest-surface routine supplies its own normal, but this consumer discards it
  and retains the separately chosen triangle normal.
- **Initial verification boundary:** Read-only inspection at Gravitas `3471085`
  / FixedMathSharp `3f7a606`, followed by direct probes of the existing Release
  local-stack assemblies using the real scenario initializer and collision
  dispatcher in PowerShell/.NET `10.0.11`. The original full-domain miss and
  both ordinary cases above were reproduced; exact upstream cone containment
  confirmed the interior witnesses. These are diagnostic probes, not a new
  build, a .NET 8 regression suite, or throughput measurements.

### GRV-Issue-082 - Cylinder contact can miss a triangle crossing below its cap

**Resolved:** 2026-09-28.

**Reproduction and cause:** A radius-5, height-10 cylinder at zero intersects
triangle `(0,7,-5)`, `(10,1,-5)`, `(5,4,5)`: `(4,23/5,0)` lies strictly inside
both. The former center-nearest fallback chose triangle Y=`175/34 > 5` and
missed the hit. It could also pair a surface depth with an unrelated face
normal. The mixed circle-slab path admitted the separated oblique-rim fixture
now retained in the FixedMathSharp contact tests.

**Fix:** FixedMathSharp owns complete finite triangle/cylinder contact
selection, sharing cylinder support algebra, stationary quartics and exact root
signs with its existing box/cylinder owner. Paired witnesses and normal/depth
are selected together before final rounding; full-width circle-slab thickness
and zero-core capsule slabs share this owner. Gravitas removed the
center-nearest fallback and approximate cap-alignment test. Only exact cap/face
winners may be enriched. Enrichment preserves the selected depth and clamp flag,
including odd raw heights, and geometric success no longer depends on manifold
count growth after deduplication or capacity reduction.

**Regression evidence:** Enabled tests cover the original intrusion, oblique rim
normals and paired anchors, one-raw gap/touch boundaries, full-domain center
offsets, cap-only enrichment, duplicate/reversed faces, mixed circle slabs and
zero allocations. The final cap-depth review additionally reproduced both
nearest-even errors (one raw instead of zero/two) with a two-triangle mesh that
keeps its canonical origin fixed; both pass after preserving primary metadata.
Upstream tests cover analytic and genuine-root witnesses, half-raw ties,
zero-radius cores, clamp boundaries, independent intrusion classification and
bounded worker-stack use. No coverage exclusion or skipped test was added.

**Validation:** Windows, `UseLocalLsfStack=true`, serial workloads on two cores:

- Gravitas full solution: Release **4,304/4,304**, ReleaseLean **4,245/4,245**.
- Gravitas Release: **56,541/56,541 lines**, **16,306/16,306 branches**,
  **5,385/5,385 methods**; Lean: **56,539/56,539**, **16,306/16,306**,
  **5,384/5,384**, respectively.
- FixedMathSharp full solution: Release **3,762 core + 49 integration** tests,
  Lean **3,743 core + 49 integration**, followed by three public boundary tests
  in each configuration after those test cases were added. Runtime sources were
  unchanged between the full and supplemental runs.
- Combined full/supplemental upstream coverage: Release **51,926/51,926 lines**,
  **11,382/11,382 branches**, **3,845/3,845 methods**; Lean **52,019/52,019**,
  **11,382/11,382**, **3,841/3,841**, respectively.
- Both libraries built for `netstandard2.1` and `net8.0` in both configurations.
  Independent geometry and Ponytail reviews completed, including the final
  cap-depth correction.
- Both generated API sites built with DocFX warnings-as-errors: zero warnings
  and zero errors. All twelve focused cylinder/circle-slab benchmark rows
  completed their preflights and measured zero managed allocation.

Reports and TRX captures are under `artifacts/grv082-final-*-coverage`,
`artifacts/grv082-final-*-supplement` and `artifacts/grv082-final-*-report` in
the owning repository. The source-mode results do not establish published
package availability. Performance evidence is recorded separately in
[GRV-Benchmark-018](benchmark-signal-hardening-backlog.md#grv-benchmark-018--complete-trianglecylinder-contact-cost).

The separately reproduced positive-core capsule-slab defect, **FMS-Issue-027**,
was resolved upstream on 2026-09-28 with complete stadium-prism contacts.
`MixedNarrowPhaseTests.CapsuleTriangle.cs` verifies separated rims, exact
interior-rim depth/normal/anchors and middle-cap contacts through the actual
mixed dispatcher. Both package configurations retain full reachable coverage.
The separately tracked mesh/cone contact defect was subsequently resolved as
**GRV-Issue-086**; its cone-volume-query sibling was resolved as
**GRV-Issue-087**.

### GRV-Issue-077 - Local-stack benchmark child fails while the launcher reports success

- **Confirmed:** 2026-09-22 at `21a22ab`, during shared timing baseline capture.
- **Resolved:** 2026-09-27; benchmark tooling only, no physics changes.
- **Evidence:** `world-context --filter '*RunEmptySimulationFrame*'` in local
  stack mode encountered CS2012 while parallel project instances wrote the same
  SwiftCollections intermediate DLL. BenchmarkDotNet reported zero executed
  benchmarks and an NA result, but `Program.Main` returned zero after ignoring
  the returned summaries. The exit code therefore falsely signals success.
  Original log: Chronicler `artifacts/timing/phase0/gravitas-benchmark.log`.
- **Follow-up:** `BuildInParallel=false` lets the generated project compile, but
  its child fails to load GridForge 9.1.0.0. The parent output contains that
  identity while the generated child has GridForge and SwiftCollections 0.0.0.0.
  Log: `gravitas-serial-build.log` beside the original. This is a source-mode
  generated build graph problem, not evidence of a physics runtime regression.
- **Current reproduction:** At `7f499c0`, an invalid option and unmatched filter
  both returned zero. A serialized generated build completed but its child
  failed to load GridForge 9.1.0.0; its output contained GridForge and
  FixedMathSharp 0.0.0.0. The generated root did not inherit
  `DisableTransitiveProjectReferences`, so SDK-added project edges bypassed
  explicit source dependency versions. Capture: `artifacts/grv077-before`.
- **Fix:** Reuse the existing FixedMathSharp runner's argument/result checks.
  Reject empty execution selections, critical validation/build/report failures
  and every reported unsuccessful/nonzero child exit, including failures after
  earlier results. Preserve valid help/list/version/info commands. A build-only
  job mutator carries the compiled configuration and, for source-built runners,
  source mode, explicit-reference protection and serial project builds into
  generated children. CLI jobs such as `Dry` remain unchanged; package mode
  remains the default.
- **Verification:** A one-off launcher probe failed before the repair and passed
  26 cases afterward, including build, partial-launch and post-result exit
  failures. Captures remain under `artifacts/benchmark-exit-codes`; the probe
  was removed after review and is not a maintained test or CI gate. The actual
  `world-context` empty-frame `Dry` child passes in Release and ReleaseLean
  without launch-time source-mode environment variables. All seven standard and
  eight Lean LSF assembly identities match the parent; Lean contains the shim,
  not MemoryPack runtime DLLs. Captures: `artifacts/grv077-release` and
  `artifacts/grv077-lean`. Independent correctness/Ponytail review found no
  remaining blockers.
- **Full matrix:** Both solution configurations build for netstandard2.1 and
  net8.0 without warnings/errors. Release passes 4,287 tests and ReleaseLean
  passes 4,228; each has only the existing enabled GRV-Issue-082 failure, with
  no skips. Runtime line/branch/method coverage remains 100% in both: 44,639 /
  44,637 lines, 13,262 branches, and 4,569 / 4,568 methods. Reports:
  `artifacts/grv077-tests-release` and `artifacts/grv077-tests-lean`.
- **Evidence boundary:** These are Windows/.NET 8 source-stack execution checks,
  not throughput measurements or released-package validation. Continue
  inspecting full logs and expected launches; summaries do not expose every
  separate diagnoser execution. The benchmark README retains the reproduction
  commands and launcher-check instructions.

### GRV-Issue-084 - Mixed cylinder/circle-slab contact duplicates incomplete directions

- **Confirmed / resolved:** 2026-09-25 / 2026-09-26, coordinated with the
  [completed FMS-Issue-024 design and validation record](https://github.com/mrdav30/FixedMathSharp/blob/main/docs/feature-work/done/2026-09-25-cylinder-pair-contact-design.md).
- **Reproduction:** At `e85cda8` with FixedMathSharp `6368582`, create an
  upright circle slab at zero with radius/half-thickness 1 and a radius-1,
  height-2 cylinder at `(7/4,7/4,11/8)`, rotated onto +X with quaternion
  `(0,0,-q,q)`, `q=Fixed64.FromRaw(3037000500)`. Bounds overlap, but exact
  cap-constrained Z reaches sum to `sqrt(7)/2 < 11/8`. The old public mixed
  dispatcher nevertheless reported contact with depth about `0.0672976`.
- **Fix:** Replace the independent incomplete selected-direction reducer with
  FixedMathSharp's complete positive-radius cylinder-pair relation through the
  existing internal geometry boundary. Carry full slab height exactly even for
  `Fixed64.MaxValue` half-thickness. Remove the obsolete helper, preserving
  canonical support anchors, pair ordering, materials and response policy.
  Capsule/circle-slab and other shape families remain separate issues.
- **Verification:** Against the working tree based on `97fee61`, all 14 mixed
  cases pass in both complete source-stack configurations: mirrored separation,
  exact tangency, one-raw and quarter-unit inward/outward neighbors, parallel
  caps, canonical anchors, maximum slab thickness and warmed allocations.
  Release has 4287 passed / one existing GRV-Issue-082 failure; ReleaseLean has
  4228 passed / the same failure, with no skips. Both retain 100% line/branch/
  method coverage: 44639/44637 lines, 13262 branches, 4569/4568 method records.
  Both target frameworks build without warnings/errors. Reports are under
  `artifacts/grv084-final-Release` and `-ReleaseLean`. Neither full suite is
  claimed green while the unrelated triangle/cylinder regression remains red.
- **Performance and release boundary:** The upstream matched capture completed
  every child successfully; difficult exact nonparallel contacts still cost
  roughly 11.57–25.14ms each on the measured machine. That limitation is
  retained in FixedMathSharp's benchmark backlog, not implicitly accepted by
  closing this correctness issue. Released-package validation remains separate
  from source mode. Independent correctness/Ponytail reviews found no
  outstanding findings.

### GRV-Issue-081 - Rounded contact depth is insufficient for strict posture clearance

- **Confirmed / resolved:** 2026-09-24. Independent native posture review
  reproduced positive penetration admitted by existing 3D sphere/capsule and new
  2D circle/capsule replacement because its contact depth rounded to zero. All
  four original regression cases failed before the owning repair.
- **Reproduction:** A size-2 2D box at `(-1,-1)` has its nearest corner at the
  origin. Grow a radius-4 circle or zero-axis capsule to radius 5 at
  `(3+epsilon,4-epsilon)`, where epsilon is one raw Q32.32 unit. The 3D control
  uses a size-2 cuboid at `(-1,0,-1)` and candidate at
  `(3+epsilon,0,4-epsilon)`. Squared distance is
  `25-2*epsilon+2*epsilon*epsilon < 25`: penetration is about 0.2 raw units, but
  nearest-even contact depth is zero. Admission incorrectly returned `Applied`
  rather than `Blocked`.
- **Fix:** Consume FixedMathSharp-owned exact strict classification before
  contact-depth rounding across every supported candidate/target family in both
  dimensions. Complete finite cap/rim and cone-generator authorities replace
  reliance on incomplete selected contact directions. Compound leaves retain
  root blocker identity; 3D mesh admission retains authored rigid frames,
  closed-convex enclosure and open/concave surface semantics. Physical
  filtering, stable order, full registered-body discovery and transactional
  rejection are unchanged. No epsilon, rounded endpoint, sampled-axis acceptance
  fallback or contact-manifold construction is used for posture admission.
- **Review:** Independent reviewers checked finite-feature completeness,
  fixed-width arithmetic bounds, exact common-sign intervals, conservative
  root/leaf bounds, and 2D/3D dispatcher parity. Three circle-polynomial defects
  found during review were reproduced and corrected before final verification.
- **Verification:** All 57 focused 3D strict-clearance cases pass in Release and
  ReleaseLean, including the original sub-raw regressions, finite rims,
  cap-clipped triangles and transformed/scaled compounds and meshes. The native
  2D family, rotation, filtering and lifetime regressions also pass. Complete
  source-mode suites report 4,274 passed / 1 known failure in Release and 4,215
  passed / 1 known failure in Lean, with zero skips or exclusions. Gravitas has
  100% line, branch and method coverage in both configurations (44,687 / 44,685
  sequence points, 13,270 branches, 4,570 / 4,569 methods). Both library targets
  build without warnings/errors. Trailblazer's complete core and adapter suites
  also pass in both source-stack configurations. DocFX builds with warnings
  treated as errors complete without warnings.
- **Measured boundary:** The existing FixedMathSharp benchmark runner reports
  zero managed allocation in all 12 final rows. Matching positive
  cylinder/capsule and cylinder/cylinder geometry costs 0.397 / 0.859 us for
  strict classification versus 27.477 / 24.881 us for contact construction.
  Difficult separated-rim queries cost 95.60 / 291.90 us; no comparison to an
  incorrect old contact result is claimed. These Windows/.NET 8/i7-9700K
  geometry microbenchmarks are not whole-transaction or frame measurements;
  reconfiguration still can scan the registry and allocate. Full JSON is in
  FixedMathSharp `artifacts/grv081-final-benchmarks`, with fixtures retained in
  its existing benchmark project.
- **Evidence:** Owning behavior fixtures are permanent. Full captures are in
  `artifacts/grv081-final-release` and `grv081-final-lean`. At that checkpoint,
  FixedMathSharp's complete suites reached full line/branch/method coverage
  while retaining six ordinary-contact failures. These Windows source-mode
  results do not certify Linux, released packages, or an entirely green stack.
- **Separate findings:** FixedMathSharp FMS-Issue-023 subsequently received a
  complete cylinder/capsule contact repair, exact behavior coverage and measured
  follow-up optimizations. FMS-Issue-024 is also repaired, with its public
  regressions passing and the mixed consumer migrated under GRV-Issue-084 above.
  FixedMathSharp FMS-Issue-025 now has a complete box/cylinder contact repair,
  including the cap-clipped miss, exact minimum-depth and edge/rim regressions.
  Gravitas consumes the unchanged manifold API; no adapter migration is needed.
  GRV-Issue-082 remains an ordinary cylinder/triangle contact-generation defect,
  not unresolved posture classification, and keeps its failing assertion
  enabled. FixedMathSharp FMS-Issue-026 separately bounds a pathological
  depth-correction loop using its existing exact search, without changing
  returned results. Classification alone does not fabricate solver normals,
  depths or witnesses.

### GRV-Issue-080 - Reconfiguration callbacks can invalidate fresh 3D pair ownership

- **Confirmed / resolved:** 2026-09-24, originally reproduced against `198d272`.
  Publisher and separation callbacks that deactivate/reinitialize the source
  could let old retirement clear fresh pairs. Resetting the world during the
  first of two separation callbacks also invalidated a shared retirement
  snapshot and threw `IndexOutOfRangeException`.
- **Fix:** Guard registration lifetimes, retire exact old pair ownership, and
  use an invocation-local snapshot that survives reset and nested transactions.
  Preserve accepted status and aggregate post-commit notification exceptions.
- **Verification:** Registration recycling, reset, nested transactions and
  native 2D counterpart controls pass. Independent review and the complete
  Release/Lean coverage matrix recorded under GRV-Issue-081 include this repair.
  Commit `4bc5010` isolates the production correction; RED evidence remains in
  the owning regression tests and `artifacts/posture-3d-lifetime-red.log`.

### GRV-Issue-079 - Physical grid changes can leave unmoved colliders undiscoverable

- **Confirmed / resolved:** 2026-09-24, originally reproduced against `198d272`.
  Replacing a grid with the same configuration or inserting a sparse voxel left
  unchanged 2D/3D colliders absent from query results; projected-circle
  counterparts reproduced the same omission.
- **Fix:** Reconcile committed partition membership once per batch of physical
  grid changes at the owning fixed-step/query boundary. Ordinary queries and
  obstacle-only changes do not force registry scans; pending authored geometry
  remains unpublished. Callback-deferred 2D refresh completes without relying on
  an incidental grounding query.
- **Verification:** The four original failures and the native/projected,
  sparse/replaced-grid, committed-geometry and callback-deferred controls pass.
  Full Release/Lean tests and coverage are recorded under GRV-Issue-081. Commit
  `622753b` isolates the correction; the original RED capture is
  `artifacts/support-posture-red.log`.

### GRV-Issue-078 - Planar restore conformance test selected the static wall

- **Discovered / resolved:** 2026-09-23 during shared timing acceptance review.
- **Evidence:** At `7edc2f1`,
  `ReplayHashTrace_ShouldMatchAfterChroniclerRestore2D` selected collider ID 1,
  the static box, rather than ID 0, the dynamic circle. Its force/torque edits
  therefore did not exercise restoration of moving-body state. Adding a dynamic
  role assertion failed for both JSON and MemoryPack.
- **Fix:** Select the moving body in both worlds and keep the role assertion.
  The existing serialize/populate and 16-step hash-continuation checks now
  operate on the intended body. Both transport cases pass; this was a test
  coverage weakness, not evidence of a production serialization defect. A
  dedicated box-size/closest-point round trip covers the shape-writing branch
  that the wrong-body case previously exercised incidentally.

### GRV-Issue-076 - Narrow simulation time breaks long-running lifecycle work

- **Confirmed:** 2026-09-22 at `21a22ab1ac8ac2d16fb8a41fcf1f65659029378d`.
- **Resolved:** 2026-09-23 by the shared timing plan's Gravitas migration.
- **Original evidence:** Crossing `int.MaxValue` retained 27 empty partitions
  beyond their two-frame lifetime; the frame-zero control retired them. A
  one-second coroutine resumed early at saturated Fixed64 time, and converting
  67,108,864 seconds at 32 Hz returned 2,147,483,647 instead of 2,147,483,648.
- **Fix:** Compose Chronicler's clock, expose wide elapsed timestamps and long
  absolute frame/phase stamps, count durations by exact raw division, and give
  waits checked deadlines plus reset-lifetime ownership. Contacts, CCD,
  partitions, diagnostics, replay hashes and the affected 3D body record use the
  widened contract; bounded simulation counters remain bounded.
- **Verification:** `GravitasTimingBoundaryTests`, `GravitasWideClockTests`,
  2D/mixed partition retirement, 2D grounding-cache expiry, body transport
  tests, and wide-clock replay/contact controls cover the old boundary and
  genuine exhaustion. Obsolete 3D records and exhausted clocks reject before the
  corresponding mutation boundary. Exact matrix/coverage/benchmark evidence is
  in Chronicler's `docs/feature-work/done/deterministicSimulationTimingPlan.md`,
  Phase 4 execution record and Phase 6 cross-stack closeout. The separate
  benchmark-tooling repair is recorded under `GRV-Issue-077`.

### GRV-Issue-075 - Centerline circle/capsule queries disagree on the surface side

- **Discovered / resolved:** 2026-09-21 during the native Trailblazer 2D
  preflight.
- **Evidence:** At `df19151`, a circle probe at a target circle/capsule center
  returned a normal opposite its surface witness. Away from a capsule's center,
  but on its axis, the center-to-center fallback could instead put the witness
  inside the capsule. Eight new upright, rotated and degenerate cases reproduced
  these failures in public overlap and initial-overlap sweep queries.
- **Fix:** Supply one radial fallback to the existing centered-contact kernel,
  with its required query-to-target sign, then use that contact's outward target
  normal consistently. Coincident circles use world +X; capsule-axis ties use
  local +X from the cached world axis. No extra solver, public API or point
  materialization is needed.

### GRV-Issue-074 - Initially overlapping circle sweeps discard the target witness

- **Discovered / resolved:** 2026-09-21; Trailblazer `TRB-Issue-150` owns the
  cross-stack discovery record.
- **Evidence:** At `df19151`, sweeping a radius-1/2 circle downward from
  `(0,1/2)` against a floor at planar height zero returned `(0,1/2)` as its
  surface point instead of `(0,0)`. `TrySweepCircle` replaced the overlap's
  target anchor with the probe start. It also changed an unrepresentable target
  witness into a falsely representable point at scalar faces.
- **Fix:** Preserve the overlap hit's rigid-frame anchor and normal while
  retaining zero **sweep travel**. Do not copy overlap distance or require an
  absolute witness. Compound owner selection and authored-order ties remain
  unchanged.
- **Regression evidence:** `Physics2DSweepWitnessTests` adds 32 behavioral cases
  across both fixes: circle/capsule/box/polygon/compound touch, penetration and
  ordinary entry; floor support; compound ties; both scalar faces; and
  centerline fallback consistency. The witness regressions failed before the
  fixes; two existing expectations that encoded the wrong probe-center point
  were corrected. The native preflight now passes 52 checks with zero witness
  gaps, plus 15 slope/support geometry checks.

Windows source-stack verification for both fixes: Release **4,053** and
ReleaseLean **3,998** tests pass. Both configurations retain exact
**56,196/56,196 lines, 16,018/16,018 branches and 5,346/5,346 fully covered
methods**; the two fewer branches remove the inconsistent normal override. Full
solution builds pass both target frameworks without warnings or errors. Coverage
is retained under `artifacts/issue150/`; release-package validation remains
deferred with the coordinated upstream releases. The same managed test outputs
also pass on Ubuntu/WSL .NET 8.0.26 in both configurations; this is runtime
execution, not an independent Linux build. Both downstream Trailblazer suites
pass on Windows/Linux in standard/Lean, and both repositories' DocFX builds pass
with zero warnings/errors. Independent source/simplicity review found no
blocking findings.

### GRV-Issue-073 - Rotational static stops use the inward contact normal

- **Resolved:** 2026-09-20 with full local-stack verification and independent
  review.
- **Discovered:** 2026-09-20, earliest-contact controls during Trailblazer
  integration Phase 2.
- **Evidence:** A real closing overlap against a static 3D mesh retained its
  incoming velocity after the rotational stop. `ManifoldContact` and `Contact2D`
  define normals from A toward B, but the closing-velocity projection requires
  the target's outward normal toward the source. Both same-dimensional stop
  paths supplied the opposite direction at baseline `aa48f7c`.
- **Fix:** Reverse only the same-dimensional conversion at this stop boundary,
  respecting canonical collider ordering in 3D. Mixed contact signs are already
  correct and remain unchanged. Focused tests use actual initial contacts and
  prepared body simulation to distinguish CCD from later discrete impulses.
  Closing velocity is removed only at a witnessed contact.

### GRV-Issue-072 - Centered sphere and circle turning can freeze support motion

- **Resolved:** 2026-09-20 with full local-stack verification and independent
  review.
- **Discovered:** 2026-09-20, Trailblazer integration Phase 2 ladder approach
  and dismount.
- **Evidence:** A radius-1/8 centered sphere/circle tangent to a box, requesting
  X displacement 1/1024 and a small yaw, accepted only 1/8192 of the requested
  travel. Pure turning was also clamped. Rotational admission treated pose
  rotation as changing geometry, and interval search repeatedly witnessed the
  unchanged support contact. A nearby but separated rotating blade also admitted
  unrelated tangential support into interval search.
- **Fix:** Distinguish geometric angular travel for exact centered
  spheres/circles without changing orientation, angular velocity, or trajectory
  storage. Preserve rotating-target admission. At static convex primitive
  candidates, reuse non-closing translational admission; do not use one contact
  normal to discard compounds or concave targets. Offset primitives retain their
  rotational sweep.
- **Focused evidence:** The expanded CCD/mixed suite passes 880 tests, including
  uniform/nonuniform scale, pure turning, tangent movement, downward/wall
  blocking, dynamic motion, real blade impact in both registration orders,
  separated blades, and existing offset-sphere/circle controls. Artifacts:
  `artifacts/gravitas-integration/rotation/`. All affected collision paths have
  exact line and branch coverage.

Final verification for both fixes: Release 4,021 tests and ReleaseLean 3,966,
with exact 56,196/56,196 lines, 16,020/16,020 branches, and 5,346/5,346 fully
covered methods in each configuration. Both full solution builds and DocFX are
warning-free. Trailblazer's final adapter consumers pass 84/80 tests with exact
coverage and unchanged repeated/restored traces. Local-source artifacts are
retained under `artifacts/gravitas-integration/coverage/release-verified` and
`releaselean-verified`; released-package validation remains separate.

### GRV-Issue-071 — Local-Stack Outputs Selected Published Math Binaries

**Discovered:** 2026-09-15 **Resolved:** 2026-09-15 **Source:** Trailblazer
direct-visibility cross-stack prerequisite validation **Affected area:**
local-stack library output, test host, and benchmark parent

RCA: Gravitas retained direct sibling `FixedMathSharp` project references in
local-stack mode, but the library, test, and benchmark project boundaries did
not suppress same-version math package assets contributed transitively. NuGet
therefore selected the published package binaries for copied output even while
the sibling projects built successfully. `Release` copied published
`netstandard2.1`/host hashes `EB3B9E...`/`731A3D...` instead of sibling
`496ADE...`/`8B125C...`; `ReleaseLean` copied `B7D265...`/`8A5138...` instead of
`91218E...`/`BF21DA...`.

Fix: in local-stack mode only, the Gravitas library and both executable hosts
retain their direct sibling project reference while excluding compile and
runtime assets from the matching standard or Lean `FixedMathSharp` package.
Normal package-backed evaluation and builds are unchanged.

Verification:

- Both local-stack configurations build the complete solution for the library's
  `netstandard2.1` and `net8.0` targets with zero warnings.
- The final library copy, test host, and benchmark parent match their applicable
  sibling math hashes in both configurations; test/benchmark dependency
  manifests identify math as a direct reference rather than a package runtime
  asset. Copied GridForge binaries also match sibling output.
- Full local-stack tests pass `3,950/3,950` in `Release` and `3,895/3,895` in
  `ReleaseLean`.
- Both coverage runs report `56,091/56,091` lines, `15,906/15,906` branches, and
  `5,340/5,340` methods.
- Fresh normal package-mode solution builds still pass in both configurations,
  and every captured math/GridForge output hash matches its pre-change package
  baseline exactly.
- No benchmark workload was run. This correction certifies the direct benchmark
  parent only; it makes no claim about BenchmarkDotNet-generated child projects.

### GRV-Issue-070 — Collider Reconfiguration Exit Failure Could Interrupt Publication

**Discovered:** 2026-09-08 **Resolved:** 2026-09-08 **Source:** Trailblazer
Navigation Hardening Phase 2 independent review **Affected area:** 3D collider
reconfiguration and pure/mixed pair retirement

RCA: `SolidBody.TryReconfigureCollider(...)` retired existing collision pairs
before publishing the accepted shape and root pose. Pair retirement invokes
separation callbacks, so one callback failure could escape with an already
removed pair while the body still exposed its previous geometry and pose. The
same ordering risk existed for both pure 3D and mixed 3D/2D pairs.

Fix: accepted reconfiguration now publishes geometry, pose, mass properties,
pure and mixed partitions, wake state, and solver-cache invalidation before pair
notifications run. An optional synchronized-state publisher runs after physical
publication and before pair notifications, preventing a coordinating consumer's
callbacks from observing cross-library half-state. Reconfiguration- specific
retirement captures publisher and callback failures and continues retiring every
captured pair in stable order. The public transaction returns one notification
failure directly or several as an `AggregateException`, separately from its
accepted status. The failure is explicitly post-commit and does not make the
transaction retryable.

Verification:

- Added a pure 3D regression where synchronized publication and both separation
  callbacks throw; it proves physical state publishes first, pair callbacks see
  synchronized dependent state, every pair retires, and failures retain stable
  publisher-then-pair order.
- Added a mixed 3D/2D regression proving its throwing exit callback observes the
  accepted shape and pose after the mixed pair is retired.
- All 15 focused collider-reconfiguration tests pass in both `Release` and
  `ReleaseLean`.

### GRV-Issue-069 — Shape-Exact 3D CCD Could Treat Separating Start Contacts As Closing

**Discovered:** 2026-09-06 **Resolved:** 2026-09-06 **Source:** Trailblazer
Navigation Hardening Phase 1, Gravitas stair-contact integration **Affected
area:** 3D shape-exact CCD against stationary and moving targets

RCA: the convex sweep query contract orients its reported normal against the
sweep direction. That is useful for public query results, but a zero-time CCD
contact must classify closing motion against the target's outward surface
normal. Reusing the sweep-oriented normal could therefore treat tangential or
separating motion as closing. The initial stationary-target correction recovered
the outward normal, but the equivalent moving dynamic/kinematic relative-hit
path still consumed the sweep-oriented normal and could publish a false hit or
handoff.

Fix: one shape-exact start-contact helper now recovers the target's outward
surface normal within the existing contact-slop boundary. Stationary and moving
relative-hit paths use that normal only for closing-direction admission; the
original exact hit and deterministic ordering remain unchanged.

Verification:

- Added static-support, adjacent-riser, separating, and multi-grid kinematic
  cylinder regressions through the public late-simulate lifecycle.
- Added a moving-pair regression proving a kinematic cylinder separating from a
  sleeping dynamic support does not publish a TOI iteration, wake the support,
  or mutate its position or velocity.
- The moving-pair regression failed before the shared-normal correction because
  the source reported one false TOI iteration, then passed after the fix.
- The five focused kinematic-cylinder regressions pass in both `Release` and
  `ReleaseLean`; the 19 existing dynamic-relative CCD regressions also pass in
  both configurations against the coordinated local LSF stack.

### GRV-Issue-068 — Scaled Mesh Query Faces Used Authored Unscaled Normals

**Discovered:** 2026-08-01 **Resolved:** 2026-08-01 **Source:** Full-Domain
Triangle-Pair Contact Phase 3 swept-sphere audit **Affected area:**
non-uniformly scaled mesh face queries in `SweptSphereQueryWorker` and
`RaycastSegmentWorker`

RCA: both workers paired committed scaled triangle vertices with an authored
unscaled face normal. Under non-uniform scale, the resulting plane differed from
the triangle used for projection containment and could publish the wrong ray
intersection or swept-sphere distance.

Fix: `PhysicsMesh` now exposes its transactionally committed scaled local face
normal through one bounds-checked internal accessor. Both workers consume it
beside the matching scaled vertices without recomputation or allocation. The
now-unreferenced public authored-normal cache, its lazy state, and its per-mesh
array allocation were deleted after a full cross-stack caller audit found no
remaining owner.

Verification:

- Literal non-uniform-scale raycast and swept-sphere regressions fail on the
  former authored plane, pass on the committed plane, and remain at `0 B`
  managed allocation after warmup.
- The two containing query suites and three stable all-hit/raw-distance ordering
  regressions pass.
- Full `Release` passes 3,925 tests and `ReleaseLean` passes 3,870 tests.
  Independent coverage reports 55,839/55,839 lines, 15,829/15,829 branches, and
  5,320/5,320 methods.
- Standard and Lean package builds pass for `net8.0` and `netstandard2.1` with
  zero warnings. The existing dense-mesh swept-sphere Dry signal remains
  comparable at subdivisions 8/16/32: `26.71 ms`, `49.70 ms`, and `179.33 ms`
  after the fix versus `27.30 ms`, `49.29 ms`, and `181.36 ms` before it.
- Focused implementation, dead-surface, and final whole-change reviews reported
  no findings. The completed plan is retained at
  [`Scaled Mesh Query Normal`](done/2026-08-01-scaled-mesh-query-normal-plan.md).

### GRV-Issue-067 — Mesh Triangle-Triangle SAT Could Saturate Before Axis Classification

**Discovered:** 2026-07-24 **Resolved:** 2026-08-01 **Source:**
canonical-collider coverage closure and collision math review

The former Gravitas mesh-pair fallback projected and ranked triangle axes
through narrowed scalar arithmetic. FixedMathSharp now owns the reusable
full-domain rigid-triangle contact relation, while Gravitas retains stable BVH
candidate traversal and canonical collider-ID direction. The duplicate scalar
SAT, cached collision-triangle wrapper, synthetic witness fallback, and hollow
wrapper tests were deleted.

Closure verified that convex sweeps, sphere sweeps, mixed circle/mesh sweeps,
and mixed mesh prisms already use separate appropriate exact authorities. The
final matrix passes 2,649 FixedMathSharp Release and 2,628 ReleaseLean tests,
plus 3,923 Gravitas Release and 3,868 ReleaseLean tests. Fresh independent
coverage is 47,095/47,095 lines, 8,698/8,698 branches, and 3,419/3,419 methods
in FixedMathSharp, and 55,850/55,850 lines, 15,833/15,833 branches, and
5,322/5,322 methods in Gravitas. Package builds are warning-free across both
target frameworks; 72 Gravitas allocation guards and the direct FixedMathSharp
triangle-pair guard pass at `0 B`. Independent closure review found and closed
one missing distinct-rotation result regression. The remaining measured dense
concave throughput cost stays in
[`benchmark-signal-hardening-backlog.md`](benchmark-signal-hardening-backlog.md)
rather than reopening the correctness issue. The completed implementation plan
is retained at
[`Full-Domain Triangle-Pair Contact`](done/2026-07-31-full-domain-triangle-pair-contact-plan.md).

### GRV-Issue-066 — Radial Segment Parameters Could Collapse Spatially Distinct Query Hits

**Discovered:** 2026-07-20 **Resolved:** 2026-07-31 **Source:** authored-segment
finite-axis distance closure **Affected area:** FixedMathSharp radial segment
output; Gravitas 2D circle raycasts/sweeps, 3D sphere raycasts/sweeps, relative
radial CCD, and mixed radial reducers

RCA: the prior wide radial solver narrowed each root to a Q32.32 ray parameter
before segment consumers reconstructed physical distance. Long authored chords
could therefore map spatially distinct roots to the same parameter, while
callers that normalized a long displacement could erase a small representable
transverse component before admission. Endpoint fallback could not recover an
interior crossing or its root order. Mixed moving-pair review also found that a
proxy-only closing test, an underestimated 2D slab proxy radius, rounded
trajectory reconstruction, and predecessor-owned shared boundaries could
override otherwise exact geometry.

FixedMathSharp now exposes direct circle and sphere physical-distance intervals
on `FixedSegment2d` and `FixedSegment`, backed by the existing
one-final-rounding wide radial kernel. Its narrow internal
direction-and-distance contracts let Gravitas avoid manufacturing an
unrepresentable relative endpoint without exposing wide arithmetic publicly.
Gravitas query and CCD consumers preserve authored source and target
trajectories through exact-distance ordering, then materialize normalized time
only for final arbitration. Mixed CCD uses a ceiling-safe Euclidean slab proxy,
treats proxy intervals as geometry-only, uses exact or sampled contact normals
for closing policy, and gives a right-continuous successor segment ownership of
shared boundaries. `RadialSweepAdmission`, dead relative-sweep wrappers, and
their wrapper-only tests were deleted.

Verification:

- FixedMathSharp passes 2,628 Release and 2,607 ReleaseLean tests at 100% line
  (46,756/46,756), branch (8,648/8,648), and method (3,413/3,413) coverage.
- Gravitas passes 3,919 Release and 3,864 ReleaseLean tests at 100% line
  (56,012/56,012), branch (15,889/15,889), and method (5,348/5,348) coverage.
- All 56 warmed Gravitas allocation guards pass. Existing radial and relative
  CCD Dry benchmark rows retain zero managed allocation, including ordinary and
  100,000-scale radial segments and 256/1,024-body relative sweeps.
- Standard and Lean package builds pass for `net8.0` and `netstandard2.1` with
  zero warnings. Independent whole-change review found no Critical, Important,
  or Minor issue.

### GRV-Issue-065 — 3D Closest-Surface And Overlap-Circle Classification Are Not Full-Domain

**Discovered:** 2026-07-22 **Resolved:** 2026-07-31 **Source:** finite-axis
closest-surface authority audit; exact 2D boundary closure review **Affected
area:** public 3D closest-surface APIs, compound and mesh feature selection, the
complete 3D projected-circle query family, and `Physics3DHit.ContactAnchor`

RCA: closest-feature selection, representable 3D distance, and final point
materialization were coupled. Saturating subtraction or an unrepresentable
conceptual coordinate could therefore hide a valid semantic surface witness,
while the X/Z circle family incorrectly used full-3D distance and vertically
scanned configured GridForge columns.

Fix: FixedMathSharp now owns policy-neutral exact planar relations and semantic
surface anchors. Gravitas dispatches every built-in primitive, mesh triangle,
and compound part through those contracts, preserves exact containment and
subraw separation direction, retains authored-order ties, and materializes a
point only through the existing explicit hit boundary. Public closest,
directional, all-hit, and batch queries share one reducer and a
partition-synchronized planar candidate index. The index remains conservative
over the full scalar domain, repairs its extent metadata after updates and
removals, skips equivalent-width metadata churn, and rebuilds deterministically
when GridForge world ownership changes. Translation updates that remain between
their sorted neighbors preserve query-ready order; actual crossings retain the
deterministic full-sort fallback.

Verification: the completed
[full-domain 3D surface projection plan](done/2026-07-31-full-domain-3d-surface-projection-plan.md)
records full standard/Lean test, exact coverage, zero-allocation, dense/sparse
vertical-scaling benchmark, documentation, and independent-review evidence.

### GRV-Issue-064 — SolidBody Point Transforms Can Saturate Before Their Final World Or Local Coordinate

**Discovered:** 2026-07-22 **Resolved:** 2026-07-30 **Source:** canonical
scale-admission final public-API audit **Affected area:** `FixedTransform`
current-snapshot conversion, authoritative 3D/2D body point conversion,
committed collider scale, and host adapters

The prior helpers chained Q32.32 scale, rotation, translation, subtraction, and
division, allowing a saturated intermediate to hide a representable final
coordinate. FixedMathSharp now owns one internal scaled-3D transform kernel. The
existing forward API and new
`FixedQuaternion.TryInverseTransformScaledPoint(...)` retain the complete
operation in wide arithmetic until one final round-half-to-even materialization
per coordinate. The generic mechanics moved out of the oriented-box owner
without leaving a forwarding-only façade.

`FixedTransform` now owns generic current-snapshot `TransformPoint` /
`InverseTransformPoint` conversion over the strict composed affine hierarchy,
plus explicit X/Z variants that retain in-plane shear and reject Y coupling.
`SolidBody` and `SolidBody2D` expose matching authoritative `GetWorldPoint` /
`GetLocalPoint` and `Try*` pairs. Body conversion consumes committed collider
owner scale rather than mutable host or presentation scale, so interpolation
cannot enter deterministic queries. Failures are atomic with a zero result;
throwing wrappers use `InvalidOperationException`. No path uses collider
dimensions or retains the superseded transform-like body names.

Regression coverage includes forward and inverse scalar-face cancellation,
mirrored anisotropic scale, affine hierarchy shear, X/Z plane admission,
singular and unavailable committed scale, host/presentation divergence, final
overflow, ordinary parity, round trips, and warmed zero-allocation behavior.
FixedMathSharp passes 2,603 `Release` and 2,582 `ReleaseLean` tests at exact
100% coverage across 44,334 lines, 8,393 branches, and 3,320 methods. Gravitas
passes 3,870 `Release` and 3,815 `ReleaseLean` tests at exact 100% coverage
across 43,028 lines, 12,779 branches, and 4,486 methods. ShortRun 3D ordinary,
3D full-domain, and 2D ordinary body round trips measure 3.714 us, 2.467 us, and
2.512 us respectively with zero managed allocation. The implementation and
evidence are retained in
[`Full-Domain SolidBody Point Transform`](done/2026-07-30-full-domain-solid-body-point-transform-plan.md).

### GRV-Issue-063 — Extreme Friction Accumulation And Cone Clamping Are Not Full-Domain

**Discovered:** 2026-07-28 **Resolved:** 2026-07-30

Gravitas now preserves point velocity, effective mass, cached impulse
accumulation and removal, friction-limit construction, Coulomb line/disk
classification, and final body deltas until one deterministic round-to-even
materialization. Proven-safe ordinary contacts retain compact 3D, 2D, and mixed
paths; unsafe intermediates route once to the existing allocation-free exact
response owners. True final overflow rejects the friction delta atomically
without partially mutating either body or publishing partial tangent caches.

The complete Release suite passes 3,861 tests at 43,653/43,653 lines,
12,775/12,775 branches, and 4,501/4,501 methods. ReleaseLean passes 3,806 tests,
focused replay passes 85/85, warmed exact 3D/2D/mixed allocation gates remain at
zero managed bytes, and 42 representative response benchmark rows report zero
allocation without a gross compact-path regression. The implementation and
evidence are retained in
[`Full-Domain Friction Response`](done/2026-07-29-full-domain-friction-response-plan.md).

### GRV-Issue-062 — True Unrepresentable Contact Lever Arms Preserve Physical Response

**Discovered:** 2026-07-22 **Resolved:** 2026-07-28

FixedMathSharp now retains exact semantic 2D/3D levers, mass points, and
positive weights through point-velocity, effective-mass, lever-dependent contact
friction response, warm-start, proportional-share, weighted-center, and
parallel-axis calculations. Gravitas keeps compact materialized vectors as the
ordinary fast path and enters the allocation-free semantic path only when the
complete expression cannot be proven representable. A truly unrepresentable
final body delta still rejects atomically; intermediate scalar overflow no
longer drops a physical contact or child mass contribution.

Pure 2D, 3D, mixed response, rotational CCD, compound mass properties, replay,
diagnostics, mirrored scalar faces, and warmed allocation behavior share that
contract. Embedded 2D mixed volumes now select far-domain circle, capsule,
polygon, box, and compound boundaries semantically; exact compound distance
ranking retains the first authored part on ties and no built-in shape falls
through the public closest-point fallback.

The implementation and release evidence are retained in
[`Exact Contact Lever And Mass Response`](done/2026-07-27-exact-contact-lever-response-plan.md).

### GRV-Issue-061 — Finite-Axis Collider Geometry Uses Canonical Rigid Frames

**Discovered:** 2026-07-19 **Resolved:** 2026-07-27

Capsules, cylinders, and cones now retain center, normalized rigid frame,
normalized local axis, full axis length, and radius as simulation authority.
Collision, query, CCD, replay, response, and diagnostic consumers use semantic
anchors and centered-axis FixedMathSharp relations instead of reconstructed
world endpoints. Broad-phase and presentation values may clip only after the
exact geometric decision.

### GRV-Issue-060 — Oriented Cuboids Use One Canonical `FixedOrientedBox`

**Discovered:** 2026-07-20 **Resolved:** 2026-07-27

`LSCuboidCollider` stores one center/orientation/half-extents representation.
Discrete, mixed, mesh, query, CCD, replay, and diagnostic paths no longer treat
clipped or cached world corners as geometry. Bounds are derived analytically and
clipped only at the representable-domain boundary.

### GRV-Issue-059 — Collider Scale Composition Is Exact And Transactional

**Discovered:** 2026-07-19 **Resolved:** 2026-07-27

FixedMathSharp owns fused scaled-dimension arithmetic, and Gravitas validates
owner/part scale composition before publishing a primitive, mesh, compound, or
bodyless-pose change. Invalid final geometry rejects atomically without leaving
the host transform, partitions, mass properties, or committed collider state
partially updated.

### GRV-Issue-058 — 2D Convex Geometry Retains Local Boundary Authority

**Discovered:** 2026-07-22 **Resolved:** 2026-07-27

2D boxes and polygons retain stable scaled-local boundaries plus canonical
planar rotation. Pure 2D and mixed relations consume origin/rotation/local
offsets directly; absolute vertices and clipped broad-phase bounds are no longer
fed back into exact collision or query decisions.

The shared Task 9 release artifact passes 2,575 FixedMathSharp Release tests and
3,669 Gravitas Release tests at exact 100% line, branch, and method coverage.
FixedMathSharp ReleaseLean passes 2,554 tests; Gravitas ReleaseLean passes
3,614. Three repeated replay runs pass 84 tests each, all 68 allocation guards
pass, and the benchmark comparison plus exact-winner optimization are retained
in
[`benchmark-signal-hardening-backlog.md`](benchmark-signal-hardening-backlog.md)
and closed by the
[`Exact Canonical OBB Throughput Hardening`](done/2026-08-02-exact-canonical-obb-throughput-plan.md)
plan with allocation-free, fully covered relative-frame kernels.

### GRV-Issue-057 — Finite-Slab Projection Support Math Is Full-Domain

**Discovered:** 2026-07-19 **Resolved:** 2026-07-22 **Source:** finite-axis
consumer closure audit **Affected area:** FixedMathSharp capsule/cylinder/cone
finite-slab support; Gravitas mixed circle-against-3D sweeps, sphere-against-2D
convex-slab start admission, and exact 2D polygon predicates

RCA: `FiniteSlabProjectionSweep` scaled its GJK simplex only after capsule,
cylinder, and cone support candidates had already been reconstructed through
saturating `Fixed64` products. Rotated extreme geometry could therefore lose the
winning support before GJK began. The opposite mixed direction squared narrowed
planar and vertical distances, allowing two saturated values to turn an extreme
separated sphere/convex-slab start into a false distance-zero hit. The old
rotated-cone fallback also reproduced a divide-by-zero in its generic GJK
triangle reducer.

FixedMathSharp now owns the public, allocation-free `FixedSlabProjection`
capsule, cylinder, and cone operations. Shape-semantic wide internals retain
candidate construction, stationary roots, comparison, and clipped-support
selection at sufficient precision until the single winning planar point is
narrowed. Adaptive signed multiplication, square-root, and ratio dispatch avoid
paying maximum declared width when the exact operands fit smaller existing
kernels. An exact unclipped-winner admission path reuses the same candidate
construction and falls back to the complete clipped solver when necessary; no
rounded duplicate geometry, public wide integer, threshold, or cached
query-dependent state was introduced.

Gravitas now delegates every capsule, cylinder, and cone orientation to that
shared support surface while retaining sweep/GJK and hit ownership. It removes
the old vertical-only reducers, saturated support reconstruction, rotated-cone
fallback, duplicate interval helpers, and redundant GJK intersection state.
Endpoint representability is admitted once, proving all monotonic intermediate
sweep points representable. Reciprocal convex-slab admission reuses the exact
sphere/slab cross-section plus planar distance predicate, and 2D convex polygon
containment/nearest-edge selection use exact orientation and distance
comparisons.

Regression coverage includes extreme rotated supports, large clipped
cross-sections, cone stationary-root and stable-tie cases, scalar-face support
failure, near-orthogonal advancement overflow, endpoint tolerance, reciprocal
extreme separation, polygon nearest-edge ordering, deterministic oracle cases,
and warmed allocation guards. FixedMathSharp passes 1,784 Release tests, 1,763
ReleaseLean tests, and explicit `netstandard2.1` compilation at 15,072/15,072
lines and 4,684/4,684 branches. Gravitas passes 3,261 Release and 3,206
ReleaseLean tests, both `netstandard2.1` configurations with zero warnings, 52
determinism/replay tests, and 66 allocation tests at 33,466/33,466 lines,
12,045/12,045 branches, and 4,164/4,164 methods.

The exact admitted-path benchmark reports capsule/cylinder/cone means of
`9.498/7.958/11.773 ms` for 64 targets and `155.987/123.107/205.485 ms` for
1,024 targets. Forced partial clipping reports `22.689/29.758/81.119 ms` and
`363.324/443.336/1,292.68 ms` respectively. Every case reports zero managed
allocation. The clipped path is intentionally more expensive than the deleted
narrow arithmetic because it now preserves the actual finite geometry rather
than saturating before the solve.

Independent final review approved the cross-stack ownership, full-domain math,
determinism, allocation behavior, tests, documentation, and scope with no
actionable findings. The guarded 190-bit cone-direction reduction remains a
useful future randomized-oracle target, not a confirmed defect.

### GRV-Issue-056 — Swept-Sphere Cuboid Dilation Uses Exact Rounded Features

**Discovered:** 2026-07-22 **Resolved:** 2026-07-22 **Source:** exact
finite-extrusion caller parity audit **Affected area:** FixedMathSharp
`FixedSegment`; Gravitas cuboid and compound sphere sweeps, grounding, and
static-target sphere/cuboid CCD

The previous cuboid reducer expanded every local half-extent by the swept sphere
radius and clipped against that larger sharp box. The proxy overcontained the
true Minkowski sum at cylindrical edge rounds and spherical corners, producing
ordinary false or early hits.

FixedMathSharp now owns one public, engine-agnostic
`FixedSegment.TryGetSweptSphereBoxIntersectionDistance(...)` contract. Its
allocation-free solver orders at most six exact box-plane transitions, solves
the active point-to-box squared-distance quadratic through existing fixed-width
wide arithmetic, and performs one final round-half-to-even conversion into the
caller's physical-distance range. No interval overload, OBB abstraction, or raw
wide arithmetic was exposed.

Gravitas transforms the original world chord into cuboid-local space through a
shared ray/sweep helper, delegates to the exact FixedMathSharp query, and
reconstructs the contact from the authored world segment. Ordinary, rotated,
compound, edge/corner miss and tangency, exact entry, extreme atomic rejection,
static CCD, and warmed zero-allocation regressions cover the shared worker.

FixedMathSharp passes 1,753 core and eight Chronicler Release tests plus 1,732
core and eight Chronicler ReleaseLean tests. Its hand-authored source reports
14,421/14,421 lines, 4,493/4,493 branches, and 2,001/2,001 ReportGenerator
methods. Locally linked Gravitas passes 3,252/3,252 Release tests and reports
33,706/33,706 lines, 12,129/12,129 branches, and 4,192/4,192 methods. The direct
FixedMathSharp reducer measures 6.635 microseconds at scale 1 and 8.558
microseconds at scale 100,000; the Gravitas worker measures 7.595 and 9.511
microseconds respectively, with zero managed allocation in every row. The exact
path is intentionally slower than the incorrect sharp proxy but faster than the
existing exact finite-capsule comparison at both scales.

### GRV-Issue-055 — Swept-Sphere Finite Extrusions Use Exact Spherical Dilation

**Discovered:** 2026-07-19 **Resolved:** 2026-07-22 **Source:** finite-axis
interval migration and mixed-reducer coverage closure **Affected area:**
FixedMathSharp finite-cylinder geometry; Gravitas 3D cylinder sweeps; mixed
circle- and capsule-slab queries and CCD

The old cylinder reducer independently expanded radius and half-height, forming
a sharp cylinder that overcontained the true spherical Minkowski dilation at its
cap rims. The mixed circle-slab path inherited the same proxy. The mixed
capsule-slab path also expanded each cap-plane core segment as a 3D capsule,
which overexpanded its horizontal rim away from the cap plane. All three paths
could therefore report early or false diagonal-rim hits while their arithmetic
or reducer labels implied exact geometry.

FixedMathSharp now owns allocation-free first-hit and interval APIs for the
exact spherical dilation of a centered finite cylinder. The reducer unions the
original-height expanded side, the two cap-disk cores, and exact tubes around
both cap rim circles. Rim boundaries use a factorized fixed-width quartic Sturm
sequence with repeated-root tangency, degenerate subresultant, near-unit-axis,
full-domain, and round-half-to-even distance handling. First-hit callers do not
pay to refine the toroidal-rim exit root.

Gravitas delegates 3D cylinder and mixed circle-slab sweeps to that shared
contract. Mixed capsule slabs now decompose their remaining finite extrusion
features explicitly: four straight top/bottom rim segments use exact segment
capsules, while both endpoint columns use exact spherically dilated finite
cylinders. The planar side and cap-face core paths remain responsible for their
non-rounded regions. Obsolete sharp-rim proxy logic was removed rather than
retained behind compatibility helpers.

Regression coverage distinguishes diagonal misses from contacts and tangencies;
exercises positive/negative rims, cap cores, straight capsule rims, start and
end containment, stationary geometry, repeated and degenerate quartics,
half-even roots, near-unit axes, opposite scalar faces, and direction symmetry;
and keeps the exact query paths at zero managed allocation. FixedMathSharp
passes 1,742 core and eight Chronicler Release tests at 14,310/14,310 lines,
4,448/4,448 branches, and 1,996/1,996 ReportGenerator methods. Gravitas passes
3,244 Release and 3,189 ReleaseLean tests at 33,706/33,706 lines, 12,129/12,129
branches, and 4,192/4,192 methods.

The final rounded-rim benchmark reports 61.75 microseconds for the 3D cylinder
and 68.24 microseconds for the mixed circle slab at scale 1, and 89.91 and 80.64
microseconds respectively at scale 100,000, with zero managed allocation in
every row. The exact solver is intentionally more expensive than the former
sharp proxy; its specialized fixed-width path remains roughly an order of
magnitude faster than the initial generic exact prototype.

### GRV-Issue-054 — Cone-Triangle Face Interiors Are Reduced Without Edge Crossings

**Discovered:** 2026-07-21 **Resolved:** 2026-07-22 **Source:** conic-query
full-domain migration **Affected area:** FixedMathSharp `FixedTriangle`;
Gravitas concave-mesh cone queries and shared mesh-triangle geometry consumers

FixedMathSharp now owns the complete apex-authored finite-cone/triangle
contract. `FixedTriangle.TryGetFiniteConeIntersectionMinimumAxialPoint(...)`
compares stable AB, BC, CA boundary candidates with the exact face-interior
candidate, while `ContainsProjection(...)` classifies projected barycentric
signs without narrowing. Plane construction, cone coefficients, half-space
tests, and full-domain witnesses remain in fixed-width wide arithmetic. Normal
faces retain the existing closed-form `Signed832` root path; coefficients whose
discriminant products exceed that proven width use an exact, allocation-free
63-step midpoint bisection over the maximum-scale parameter lattice. Edge/face
boundary cells require both bracketing lattice points to remain inside the face,
so stable edge ownership wins an ambiguous same-cell tie.

Gravitas delegates concave mesh candidates directly to the shared triangle
reducer and no longer owns the incomplete edge-plus-axis fallback. All mesh
closest-point and projected-containment callers now use `FixedTriangle`, which
also removes the public `MeshUtils` wrapper and its wrapper-only tests. This
deletes the winding-normal misuse surface and reduces the Gravitas change by
roughly 300 net lines while improving the reusable lower-layer contract. The
problem is 3D-specific; there is no 2D cone/mesh parity path to duplicate.

Regression coverage includes parallel and oblique face interiors without edge
crossings, face hits preceding later edges, exact edge/face ties, maximum-scale
root-cell ownership, perpendicular and out-of-range planes, zero-radius cones,
near-unit axes, cap contact versus one-unit miss, degenerate and reversed
triangles, and raw-domain coefficient/root cases. FixedMathSharp passes 1,714
Release and 1,693 ReleaseLean tests plus eight Chronicler tests in each
configuration at 13,060/13,060 lines, 4,184/4,184 branches, and 1,920/1,920
ReportGenerator methods. Gravitas passes 3,238 Release and 3,183 ReleaseLean
tests at 33,732/33,732 lines, 12,133/12,133 branches, and 4,191/4,191
ReportGenerator methods.

The final warmed 64-target concave-mesh cone query remains statistically flat at
4.189 ms versus the 4.118 ms baseline, with zero managed allocation. The
representative 64-pair mesh/cone collision row reports 236.6 microseconds with
zero managed allocation.

### GRV-Issue-053 — Conic Query Quadratics Remain Full-Domain Until Final Hit Narrowing

**Discovered:** 2026-07-18 **Resolved:** 2026-07-21 **Source:** radial consumer
audit **Affected area:** FixedMathSharp finite-cone segment geometry,
`RaycastSegmentWorker`, and `GravitasQuery3DService` cone-volume mesh reduction

The root cause was duplicated downstream conic math. Both the cone-collider
raycast and the concave-mesh edge path formed axial projections, quadratic
coefficients, discriminants, and roots in saturating `Fixed64`; large valid
coordinates could lose the crossing before any finite-height check. The raycast
path also classified tiny nonzero segments through a squared-length value that
could round to zero.

FixedMathSharp now owns one allocation-free finite-cone segment reducer with
apex-authored and centered contracts, exact supplied-axis length, exact axial
clipping, positive, negative, linear, and constant polynomial handling,
strict/inclusive endpoint classification, and deterministic half-even root
selection. Parameter APIs serve bounded geometry, physical-distance APIs retain
spatially distinct scalar hits, and point APIs retain high-resolution lattice
witnesses on long authored chords. The same lower-stack work adds exact-or-false
ray point reconstruction, q-aware axial projection and radial-plane
normalization for accepted near-unit axes, and a conservative finite-cone AABB
clipped to the representable domain. Gravitas consumes those contracts for
collider raycasts, cone support mapping, broad-phase bounds, axial hit ordering,
closest-axis witnesses, and concave-mesh edge reduction. A rounded lattice point
from an accepted edge interval is retained without reclassifying the already
exact continuous intersection. The tiny-segment sentinel now uses segment
identity instead of a lossy squared magnitude, and a stale test that labeled an
authored apex endpoint as outside was removed.

Regression coverage includes extreme side crossings, tangency versus a
one-raw-unit miss, opposite-lobe clipping, first-root rejection with later-root
admission, starts inside, flat-base and apex endpoints, generator/linear paths,
near-half-raw irrational roots, above- and below-unit non-cardinal axes,
zero-radius generators, centered half-radius classification, long-edge spatial
witnesses, tiny AABB segments, and warmed allocation guards. Dedicated query
benchmarks exercise cone-collider raycasts, concave-mesh edge reduction, and
oblique long/narrow cone bounds. The separate cone/triangle face-interior issue
remains queued because its geometry is distinct from edge intersection.

Verification passed 1,689 FixedMathSharp core and 8 Chronicler tests in
`Release`, 1,668 core and 8 Chronicler tests in `ReleaseLean`, 3,248 Gravitas
tests in `Release`, and 3,193 Gravitas tests in `ReleaseLean`. Authoritative
coverage remains exact in both affected libraries: FixedMathSharp reports
12,523/12,523 lines, 4,090/4,090 branches, and 1,887/1,887 methods; Gravitas
reports 33,825/33,825 lines, 12,163/12,163 branches, and 4,200/4,200 methods.
The final warmed 64-target rows measured about 867.8 microseconds for
cone-collider raycasts, 3.798 milliseconds for cone-volume overlap against
concave meshes, and 1.260 milliseconds for the oblique long/narrow bounds case,
with zero managed allocation. Single-operation `Dry` samples measured 8.446,
37.337, and 21.979 milliseconds respectively; those cold-start samples include
wide-solver JIT cost and are retained as startup evidence rather than throughput
estimates.

### GRV-Issue-052 — Explicit Body Roles Preserve Independent Angular Mobility

**Discovered:** 2026-07-19 **Resolved:** 2026-07-20 **Source:** rotational
moving-pair CCD final mobility review **Affected area:** 2D/3D body roles,
freeze-axis semantics, pure/mixed partitioning, response islands, joints, sleep,
visualization, CCD, serialization, replay, and host integration

The root cause was an overloaded constraint mask: full position freeze also
acted as the runtime's implicit static-body identity, and `CanRotate` depended
on `CanTranslate`. That leaked one authored choice into collider buckets, awake
admission, effective mass, constraints, CCD, serialization, and presentation,
making a translation-locked spinner impossible to model coherently.

Gravitas now owns explicit `Dynamic`, `Kinematic`, and `Static` body roles
through `BodyMotionType`, while translation and rotation mobility remain
independent solver constraints. Role-aware 2D, 3D, and mixed paths preserve
body, collider, pair, joint, and host identity; reconcile simulated-body and
partition membership; and clear incompatible motion, sleep, CCD, warm-start, and
joint caches atomically. Public transitions, ragdoll batches, registered
serialization population, and explicit static pose changes validate before
commit. Static pose refresh is immediately visible to pure and mixed queries,
including a collider leaving and re-entering the grid. `ResetPosition` also uses
a tentative host-pose transaction that validates the exact candidate hierarchy
scale and rotation for every active role and restores exact local components
before any body, collider, or partition mutation on rejection.

Substantive regressions cover all six role transitions, translation-locked
angular response, fully locked bodies, 2D/3D/mixed partition and CCD parity,
fixed-step and callback rejection, invalid and detached registration,
hierarchy-pose rollback, JSON/MemoryPack population, replay, and warmed
zero-allocation transitions. The dedicated warmed transition benchmark reports
about 15.146 microseconds for 3D and 6.751 microseconds for 2D with zero managed
allocation. Final verification passed all 3,237 Release tests at 100% line,
branch, and method coverage: 33,894/33,894 lines, 12,211/12,211 branches, and
4,206/4,206 methods. The final independent review reported no critical or
important findings. See the completed
[`Body Motion Type And Solver Mobility Hardening Plan`](done/2026-07-20-body-motion-type-and-solver-mobility-plan.md).

### GRV-Issue-051 — Translational CCD Preserves Piecewise Target Trajectories

**Discovered:** 2026-07-19 **Resolved:** 2026-07-20 **Source:** rotational
moving-pair CCD final trajectory-consumer review **Affected area:** 2D/3D/mixed
translational moving-pair CCD, kinematic pushes, same-frame requeues, and dirty
candidate overlays

Translational moving-pair CCD now reduces the target's canonical trajectory
segment by segment over the source's remaining frame interval. Each target slice
is paired with the same source-time slice, the existing exact 2D and 3D shape
reducers remain authoritative, and segment-local hits map back to source
distance without flattening an outward-and-return path into its endpoint chord.
The mixed directions retain their documented conservative proxy while consuming
the same piecewise timing contract. Dynamic and kinematic paths share the same
reducers rather than maintaining duplicate endpoint samplers.

Canonical handoff boundaries are right-continuous, so a successor's separating
motion suppresses a predecessor contact reported only at the shared endpoint.
Chronological, non-overlapping segment ranges allow the first admitted
non-boundary hit to terminate the target scan. Full-frame queries take a direct
range fast path, while same-frame requeues select only the overlapping partial
range. Exact-distance cross-dimension selection now shares one explicit policy:
2D precedes 3D.

Regression coverage includes outward-return dynamic and kinematic targets in
pure 2D, pure 3D, and both mixed directions; stop/reverse boundary ownership;
registration-order invariance; repeated replay hashes; causal relay budgets; and
warmed zero-allocation reduction. The dedicated benchmark reports bounded
segment scaling with no managed allocation: one/two/four target segments take
about 0.55/0.93/1.76 microseconds in 2D and 1.08/1.51/2.37 microseconds in 3D on
the current short-run host. Final authoritative verification passed all 3,123
`Release` tests at 100% line, branch, and method coverage: 33,539/33,539 lines,
12,137/12,137 branches, and 4,158/4,158 methods.

### GRV-Issue-050 — Derived Bound Centers And Extents Were Not Full-Domain

**Discovered:** 2026-07-20 **Resolved:** 2026-07-20 **Source:** FixedMathSharp
sphere-construction full-domain audit **Affected area:** FixedMathSharp
bounds/range/circle derivation; SwiftCollections fixed query volumes, octree
subdivision, and BVH insertion; GridForge grid centers; Gravitas mesh
validation, mixed slab queries, and 3D broad-phase proxies

FixedMathSharp now owns one strict 2D/3D derived-bound contract: exact
nearest-even centers, conservative scopes, exact representable sizes, and atomic
centered mutations. Unrepresentable public extents fail explicitly instead of
saturating to an under-bound. Separately named clipped factories provide the
intentional world-domain intersection required by spatial proxies whose
conceptual shape crosses a scalar face. `FixedRange` follows the same exact
midpoint/difference rules.

Downstream, SwiftCollections' `FixedBoundVolume` now delegates to one canonical
`FixedBoundBox`, and its octree compares child spans directly in unsigned raw
space so a full-domain root can subdivide without asking for an unrepresentable
size. Fixed BVH insertion delegates to FixedMathSharp's exact 192-bit
union-volume growth and clamps only the final public `long` metric. GridForge
derives `GridCenter` through the shared exact midpoint. Gravitas mesh validation
accepts valid same-sign extreme centers while still rejecting genuinely
unrepresentable sizes, mixed mesh-slab queries use exact reference centers, and
collider broad-phase bounds opt into explicit domain clipping instead of relying
on incidental saturating operators. Exact rotated OBB geometry at a scalar face
remains the separately tracked canonical-half-extent issue. The audit also
removed the now-unused mesh vector representability wrapper and simplified
collider-bound refresh ownership.

Regression coverage includes same-sign and opposite scalar faces, raw midpoint
ties, representable scope with unrepresentable size, full-domain octree roots
and BVH unions, atomic failure, serialization continuity, circle/proxy clipping,
mesh scale admission, mixed slab hit points, and downstream grid-center parity.
FixedMathSharp passed 1,655 Debug tests at 13,855/13,855 lines, 3,952/3,952
branches, and 1,808/1,808 methods. Gravitas passed 3,105 Release tests at
33,266/33,266 lines, 12,091/12,091 branches, and 4,148/4,148 methods. Centered
bounds construction became materially faster with zero managed allocation;
direct min/max construction remained flat. SwiftCollections' canonical volume
layout also traded a few nanoseconds of repeated metadata recomputation for a
much smaller pass-by-value copy path, with both paths remaining allocation-free.
The exact volume-expansion hot path measured about 10.46 nanoseconds versus
17.15 nanoseconds for the reconstructed legacy formula; its full-domain case
measured about 10.27 nanoseconds, with zero managed allocation. Configuration
gates passed FixedMathSharp `Release` (1,655 core plus 8 Chronicler tests) and
`ReleaseLean` (1,634 plus 8), SwiftCollections `Release` (1,096) and
`ReleaseLean` (1,068), GridForge `Release` and `ReleaseLean` (460), and the
explicitly linked Gravitas `Release` test-project gate (3,105). The Gravitas
`ReleaseLean` local-link gate remains blocked by the already documented
GridForge MemoryPack public-type leak; it is still deferred to the
package-reference release gate rather than masked downstream.

### GRV-Issue-049 — Sphere Construction And Merge Paths Were Not Full-Domain

**Discovered:** 2026-07-18 **Resolved:** 2026-07-20 **Source:** exact radial
predicate migration and release audit **Affected area:** FixedMathSharp
`FixedBoundSphere` factories and exact coordinate interpolation shared with
finite segments

`FixedBoundSphere.CreateFromBoundingBox`, `CreateFromFrustum`,
`CreateFromPoints`, and `CreateMerged` now retain midpoint, squared-distance
ordering, roots, radius sums, and center interpolation in exact wide arithmetic
until the final Q32.32 result. Required radii round outward; successful results
are containment-safe; and deterministic constructions whose required radius is
not representable throw `OverflowException` instead of returning a saturated
under-bound sphere. The docs explicitly retain deterministic Ritter-style
point/frustum construction and Q32.32 lattice-center merge semantics rather than
claiming a mathematically minimum sphere.

Extreme-pair seeding compares exact squared distances from the midpoint to its
two actual endpoints. Reversed secondary axes cannot select the nearer endpoint,
and coordinate-wise synthesis cannot invent a farther corner that falsely
requires an unrepresentable radius.

The shared final-parity coordinate ratio removed duplicate segment interpolation
code and its invariant-specific single-word path reduced existing segment
reconstruction rows from roughly 514/759 nanoseconds to about 121/118
nanoseconds at unit/100,000 scale with zero allocation. Ordinary 1,024-point
array/span sphere construction measured about 8.77/8.80 microseconds with zero
allocation versus the former saturated approximation's roughly 4.84/4.60
microseconds; asymmetric full-domain merge measured about 321 nanoseconds with
zero allocation. FixedMathSharp verification passed 1,640 core plus 8 Chronicler
tests in `Release`, 1,619 core plus 8 Chronicler tests in `ReleaseLean`, and
reported 100% coverage across 13,826 lines, 3,930 branches, and 1,801 methods.
The locally linked Gravitas `Release` test-project gate passed all 3,103 tests.
Its `ReleaseLean` solution gate remains subject to the already documented
GridForge local-link MemoryPack type leak and is deferred to the
package-reference release gate. No Gravitas workaround was added. The audit
exposed the separate derived area/box center and extent issue now first in the
active queue.

### GRV-Issue-048 — Finite-Axis Capsule, Cylinder, And Mesh-Edge Projections Can Saturate Before Solving

**Discovered:** 2026-07-18 **Resolved:** 2026-07-19 **Source:** full-domain
radial interval consumer audit

FixedMathSharp now owns allocation-free finite-axis capsule intervals in 2D and
3D plus finite-cylinder and separately expanded affine-cylinder intervals in 3D.
Endpoint differences, radial projection, quadratic roots, axial clipping, and
root-to-chord reconstruction remain in exact wide integer arithmetic until one
final half-to-even Q32.32 spatial-distance conversion. The contract returns both
distances, reconstructs points directly from the authored segment, preserves
exact inclusive-start and strict-end containment, reduces collapsed capsules to
the sphere or circle limit, and explicitly rejects collapsed cylinders whose
flat cap normal would be undefined.

Gravitas migrated its 2D capsule raycasts and sweeps, 3D capsule/cylinder
raycasts, swept-sphere capsule/cylinder and mesh-edge projections, and mixed
finite-axis reducers to the lower-stack contract. Capsule feature selection now
considers the cylindrical side and both endpoint hemispheres together instead of
short-circuiting seams or far features. Runtime admission also rejects a scaled
cylinder whose axis has collapsed before registration mutates collider or
service state, including compound parts.

The closure preserved exact 100% hand-authored coverage in both repositories.
FixedMathSharp reports 13,691/13,691 lines, 3,908/3,908 branches, and
1,777/1,777 methods. FixedMathSharp Release passed 1,623 core plus 8 Chronicler
tests and ReleaseLean passed 1,602 core plus 8 Chronicler tests. Gravitas's
authoritative coverage-enabled Release run passed 3,103 tests and reports
33,271/33,271 lines, 12,093/12,093 branches, and 4,149/4,149 methods. The
locally linked ReleaseLean gate passes 3,064 tests after configuration- scoped
dependency restores and builds both library targets with zero warnings. CRAP
analysis reports 19 fully covered methods above 30, all structural complexity
floors rather than uncovered risk.

The dedicated Gravitas short-run benchmark measured exact capsule and cylinder
segment intervals, swept-sphere reducers, and the mixed circle-slab reducer at
ordinary and 100,000-unit scales between 8.427 and 13.747 microseconds, with
zero managed allocation in every row. The unchanged radial sphere baseline
measured 1.462-1.821 microseconds. Lower-stack common finite-axis rows measured
approximately 2.3-2.9 microseconds, its arbitrary-raw cylinder stress row
approximately 8.9 microseconds, and warmed centered-capsule distance rows
approximately 4.0-4.8 microseconds across normal and 100,000-unit inputs. All
lower-stack rows allocated zero managed bytes. Existing end-to-end 2D, 3D mesh,
and both mixed capsule sweep benchmarks remained allocation-free at 64 and 1,024
candidate scales. These short runs are regression signals, not canonical
performance claims. Follow-up audits separated the remaining sharp-rim, rounded
capsule-slab rim, and finite convex-slab support-model risks into their own
active issues instead of hiding them inside this arithmetic closure.

### GRV-Issue-047 — Rotational CCD Omits Dynamic And Mixed Targets

**Resolved:** 2026-07-19 **Source:** between-sample rotational CCD final parity
review **Affected area:** 2D/3D rotational candidate gathering, mixed collision
mode, dynamic target response, and same-frame CCD handoffs

RCA: same-dimensional rotational CCD gathered only through static-target query
surfaces, and pure rotation exited before dynamic candidate indexing or mixed
routing. Moving-target snapshots lacked independently sampled angular
trajectories, while the existing linear-mass response and handoff contracts
could not represent contact-point angular response or remaining-frame rotation.

Fix: 2D, 3D, and mixed CCD now share order-independent piecewise frame
trajectories, immutable candidate indices with bounded dirty overlays, stable
dimension-tagged normalized-time arbitration, contact-point inverse-mass and
inertia response, and atomic bounded translation/rotation handoffs. Kinematic
authored motion, dynamic wake/response, `Both` exclusion, callback-driven stale
candidate lifetimes, and disabled-service preparation are explicit contracts.
The 3D response also rejects separating and tangent contacts before mutation.

Verification: the authoritative `Release` coverage run passes 3,056 tests and
reports 100% line (33,555/33,555), branch (12,237/12,237), and method
(4,167/4,167) coverage. Library/package and benchmark builds complete with zero
warnings or errors. Twelve 1/8/32-pair benchmark rows remain approximately
linear and allocation-free except for the separately tracked small 32-pair mixed
discrete broad-phase capacity-growth signal. Independent final review found the
three lifecycle/response defects above; all were corrected with behavior
regressions before closure. The completed design and detailed evidence are
retained in
[`2026-07-18-rotational-moving-pair-ccd-plan.md`](done/2026-07-18-rotational-moving-pair-ccd-plan.md).

### GRV-Issue-046 — 3D CCD Handoff Callback Failure Could Abandon Queue Cleanup

**Resolved:** 2026-07-18 **Source:** same-frame CCD handoff dedupe final
lifecycle review **Affected area:** queued 3D/2D handoff consumption,
`SolidBody.OnMoved`, mixed handoff coordination, budget counters, and replay
continuity

RCA: queued 3D consumption invoked the public movement callback before the
service recorded the completed iteration or closed its queue. A callback could
requeue the same body and throw, leaving that continuation plus unread bodies
pending and replay-visible. A 3D exception also skipped cleanup of an already
prepared 2D queue. Both services merely cleared stale queue ownership at the
next frame boundary without discarding body-local tokens.

Fix: queued 3D consumption now commits body state and service iteration
bookkeeping before invoking host code. Both dimensional drains finalize in
`finally`: successful batches clear ownership, while callback/internal failures
and exhausted budgets discard every unread or requeued body continuation. The
world-context coordinator aborts both prepared dimensional batches while
rethrowing the original exception, and the next-frame boundary defensively
discards rather than merely forgetting stale work. `OnMoved` now documents its
authoritative simulation timing.

Regression coverage proves the exact exception instance escapes, partially
completed counters remain deterministic, same-callback requeues and unread
bodies are neither consumable nor replay-visible, 2D internal failures receive
the same cleanup contract, and a 3D failure aborts a prepared 2D batch. All 51
handoff-focused tests and three relevant warmed allocation guards pass. Full
validation passes 2,793 Release and 2,754 ReleaseLean tests.

### GRV-Issue-045 — Full-Domain Radial Bounds And Query Intervals Were Incomplete

**Resolved:** 2026-07-18 **Source:** relative CCD exact-root migration and
mixed-query parity review **Affected area:** FixedMathSharp radial predicates,
bounded ray intervals, and cross-sections; Gravitas 3D sphere-segment raycasts
and mixed circle-slab/sphere cross-section reducers

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
100,000 respectively, with 0 B allocated. Finite-axis capsule/cylinder/mesh-edge
projection, sphere construction/merge, and conic quadratics remain explicitly
separate active issues rather than being masked by the radial result.

### GRV-Issue-044 — Relative CCD Quadratic Saturation Could Miss Extreme-Range Crossings

**Resolved:** 2026-07-18 **Source:** 95%-to-100% coverage hardening, shared
relative-sweep review **Affected area:** FixedMathSharp radial rays; Gravitas
2D, 3D, and mixed radial sweeps and relative continuous collision

RCA: relative sphere/circle sweeps formed their quadratic directly in Q32.32. At
large separations or displacements, squared terms saturated before the
discriminant and root were evaluated. Query reducers duplicated variants of the
same arithmetic. A crossing could collapse to a wrong endpoint candidate,
produce the wrong normal, and be rejected despite broad-phase admission.

Fix: FixedMathSharp now owns an allocation-free exact first-root solver. It
retains 65-bit endpoint differences, `Signed192` coefficients, a `Signed320`
discriminant, exact bounded-root ordering, and nearest-even conversion only at
the public `Fixed64` boundary. `FixedRay` and `FixedRay2d` expose bounded radial
intersection overloads with exact nonnegative radius expansion. Full-domain
vector direction and endpoint-distance helpers support downstream reconstruction
without saturated subtraction or squared magnitude.

At that checkpoint Gravitas centralized radial admission in
`RadialSweepAdmission`, used exact normalized-frame roots, and retained
normalized public-query directions. The later exact radial segment-distance
closure replaced that intermediate contract: segment consumers now solve and
rank physical distance along the authored chord, relative CCD reconstructs both
authored trajectories before time materialization, and `RadialSweepAdmission`
has been deleted. The separate resolved radial-interval record captures that
final state.

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

### GRV-Issue-043 — Convex Mesh Mode Accepted Invalid Topology And Could Collide In Empty Bounds Space

**Resolved:** 2026-07-18 **Source:** 95%-to-100% coverage hardening, mesh/sphere
fallback review **Affected area:** mesh topology admission, closed-surface
identity, exact surface queries, collision dispatch, and full-domain fixed-point
predicates

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
duplicate faces, disconnected components or vertex links, edge non-manifoldness,
inconsistent winding, reflex closed edges, and open folds, holes, or overlaps.
`Concave` remains the explicit arbitrary open, closed, or disconnected surface
mode. Both modes expose cached `IsClosedSurface` topology, including exact seam
handling and pinched-vertex rejection.

Closest-surface queries now seed an authored triangle, prove a conservative BVH
search cube from that exact upper bound, and fall back to a stable full scan
only when the bound is unrepresentable. Exact FixedMathSharp orientation,
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

### GRV-Issue-042 — Rotational CCD Could Miss Contacts Between Bounded Pose Samples

**Resolved:** 2026-07-18 **Source:** 95%-to-100% coverage hardening, rotational
CCD review **Affected area:** 2D/3D rotational CCD, pivot-centered candidate
proxies, deterministic interval traversal, and conservative fixed-point
separation

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
- The focused rotational, near-miss, and angular-tunneling surface passes all 51
  tests, including existing unsupported-pair and allocation contracts.
- Dense unresolved-candidate benchmarks cover 2D and 3D aggregate interval costs
  at `1`, `8`, and `32` admitted targets. The final short-run means were
  `1.80`/`4.90`/`7.23` ms in 3D and `0.73`/`1.33`/`1.98` ms in 2D.
- Benchmark and full-suite release evidence are recorded in the resolving
  commit.

### GRV-Issue-041 — 3D Angular Impulse Scaled Immediate Velocity By Frame Delta

**Resolved:** 2026-07-18 **Source:** 95%-to-100% coverage hardening, 2D/3D
motion parity review **Affected area:** public force/impulse units, immediate
body motion, and 2D/3D motion API parity

RCA: both public 3D impulse methods multiplied their inverse-mass response by
`DeltaTime`, treating instantaneous momentum transfer as a continuous force.
`AddLinearImpulse(...)` additionally routed through the fixed-step velocity and
pose update, so a host command could apply gravity and move, ground, or sweep a
body before the next simulation phase. Collision, mixed-response, and constraint
solvers already applied velocity deltas directly and did not compensate for this
behavior.

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
- `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj -c Release --no-restore`
  passed all 2,734 tests.
- `dotnet build src/Gravitas/Gravitas.csproj -c Release --no-restore` passed for
  `net8.0` and `netstandard2.1`; the benchmark project also built cleanly.
- `ReleaseLean` remains deferred to the package-reference release gate because
  the intentionally retained local GridForge project link exposes MemoryPack
  interfaces without its package assembly in that configuration.

### GRV-Issue-040 — SolidBody Point Transforms Used Collider Dimensions As Transform Scale

**Resolved:** 2026-07-18 **Source:** 95%-to-100% coverage hardening, 3D compound
`ScaledSize` review **Affected area:** `SolidBody.TransformPoint(...)`,
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
explicitly. Focused regressions pass `3/3`, the complete
`SolidBodyIntegrationTests` surface passes `23/23`, and full locally linked
suites pass `2731/2731` in `Release` and `2692/2692` in `ReleaseLean`. Both
configurations build the `net8.0` and `netstandard2.1` package targets with zero
warnings. Both modified methods report 100% line and branch coverage.

### GRV-Issue-039 — CCD Handoff Dedupe Could Strand A Same-Frame Requeued Body

**Resolved:** 2026-07-18 **Source:** 95%-to-100% coverage hardening, dimensional
CCD service admission review **Affected area:** 2D/3D continuous-collision
handoff queues, same-frame relay cycles, mixed CCD routing, iteration-budget
ownership, and replay continuity

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
expected O(1), allocation-free, and does not expose hash iteration order. Mixed
responses already route target handoffs through these dimensional queues, so no
parallel mixed implementation was required. Final review also exposed the
complementary latest-state edge: a terminal update could clear ignored collider
references while leaving an older pending continuation intact. The 2D and 3D
body paths now discard that superseded continuation atomically when the new
update has no remaining time or resulting motion.

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

### GRV-Issue-038 — 3D Exit Callback Failure Duplicated Reentrant Separation Notifications

**Resolved:** 2026-07-18 **Source:** 95%-to-100% coverage hardening, 3D
collision-pair lifecycle review **Affected area:**
`CollisionPair.NotifyCollidersOfContact()`, `CollisionPair2D.NotifyColliders()`,
`CollisionPairMixed.MarkColliding()`, and callback exception/reentrancy teardown

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
stable exception order, and no notifying-shell pool reuse. Matching 2D and mixed
RED regressions reproduced first-callback short-circuiting and early
deferred-guard release; GREEN assertions prove stable A/B and 3D/2D exception
order with both guards retained through exit dispatch. The combined lifecycle
surface passes `125/125`; full locally linked suites pass `2723/2723` in
`Release` and `2684/2684` in `ReleaseLean`. Both configurations build `net8.0`
and `netstandard2.1` package targets with zero warnings.

### GRV-Issue-037 — Continuous-Collision Modes Accepted Undefined Enum Values

**Resolved:** 2026-07-17 **Source:** 95%-to-100% coverage hardening, 3D CCD
helper review **Affected area:**
`PhysicsSettings.DefaultContinuousCollisionMode`,
`SolidBody.ContinuousCollisionMode`, `SolidBody2D.ContinuousCollisionMode`, and
replay/settings population

RCA: the context setting and both body properties assigned the byte-backed enum
without validating that the value was declared. Chronicler also populated the
body backing fields directly. Undefined values could therefore survive public
assignment or serialization, then resolve as neither continuous nor automatic
and silently use discrete movement.

Fix: one enum-adjacent validation policy now admits only `Inherit`, `Discrete`,
`Continuous`, and `Auto`. Context settings and both body properties reject
undefined values with `ArgumentOutOfRangeException` before changing their stored
mode. Body replay loads through a validated local value, so a rejected payload
does not publish the corrupt mode, while `PhysicsSettingsSaver` materialization
rejects invalid authored or deserialized values before replacing context
settings. Context-level `Inherit` remains valid and deliberately resolves to
`Discrete` when no concrete body or hierarchy override exists.

Verification: first-invalid (`4`) and byte-maximum (`255`) regressions cover 3D
and 2D body assignment, context settings, settings application, and both body
replay paths, including preservation of the previously valid value after
rejection. Existing 2D and 3D context-`Inherit` behavior remains covered. The
focused suite passes `14/14`; the full locally linked suites pass `2716/2716` in
`Release` and `2677/2677` in `ReleaseLean`. Both configurations build the
`net8.0` and `netstandard2.1` package targets with zero warnings.

### GRV-Issue-036 — Non-Unit Quaternion Admission Can Collapse Runtime Shape Axes

**Resolved:** 2026-07-17 **Source:** 95%-to-100% coverage hardening, cone-bounds
fallback review **Affected area:** `SolidBody` rotation admission, compound-part
local rotations, collider shape state, and replay/load population

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
representation. Direct `PhysicsMesh` transform APIs remain intentionally strict
and still reject non-normalized rotations; compound mesh parts reach that
validation only after descriptor normalization.

Verification: focused regressions cover zero, scaled, saturated, near-unit, and
axis-collapsing quaternions across body initialization, public mutation, visual
state, compound primitives, compound meshes, cone bounds, replay population, and
direct mesh rejection. The full locally linked suites pass `2704/2704` in
`Release` and `2665/2665` in `ReleaseLean`; Lean builds both `net8.0` and
`netstandard2.1` and produces both packages with zero warnings.

### GRV-Issue-035 — Registered Joints Can Outlive Their Body And Collider Lifetimes

**Resolved:** 2026-07-17 **Source:** 95%-to-100% coverage hardening, 3D joint
replay-hash lifecycle review **Affected area:** 2D/3D joint ownership,
body/collider deactivation and reuse, linked-collision suppression, replay
identity, and ragdoll lifecycle

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
belong to one registered ragdoll, link teardown or `RemoveRagdoll(...)` removes
the runtime and all owned joints, independent owned-joint removal is rejected,
and stale joint/ragdoll handles cannot mutate or serialize later simulation
lifetimes. Their intrusive registration-order chain gives O(1) removal without
making replay hashes depend on removal history. Context reset and disposal
invalidate all handles, and mutating constraint APIs reject a disposed context.

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

### GRV-Issue-034 — GridForge Reuses Grid Spawn Tokens Across Pooled Generations

**Resolved:** 2026-07-17 **Source:** 95%-to-100% coverage hardening, 3D
partition teardown review **Affected area:** GridForge pooled `VoxelGrid`
identity, exact traversal, and Gravitas 2D/3D/mixed partition and query
consumers

RCA: GridForge derived world and grid allocation tokens from `GetHashCode()`
values. Removing and re-adding an identical pooled grid could reuse both the
world-local slot and token, allowing a stale `WorldVoxelIndex` to resolve
replacement state. Hash-derived voxel tokens also made hash collisions capable
of suppressing distinct traversal results.

Fix: GridForge commit `0c5420f` added process-unique 64-bit world identity and a
nonrepeating grid generation owned by each world, preserved across
non-deactivating reset. `WorldVoxelIndex` now validates the world token, the
recyclable `GridIndex`, the grid generation, and the voxel coordinate. GridForge
commit `cc2c451` changed unique traversal to `SwiftHashSet<WorldVoxelIndex>` and
removed hash-derived voxel and scan-cell identity. Gravitas commit `598c2de`
consumes the exact key in 3D query deduplication and adds same-configuration
replacement regressions for pure 2D, 3D, and mixed partitions.

Verification: with Gravitas locally linked to the corrected GridForge, each
2D/3D/mixed regression proves the stale coordinate fails, the replacement grid
receives a different generation, and its live partition resolves normally. The
focused Gravitas identity/query/order suite passed `159/159`; GridForge's exact
traversal regressions cover duplicate suppression, synthetic hash collisions,
and `0 B` warm reusable-set traversal. The local project links remain temporary
uncommitted release-validation scaffolding.

### GRV-Issue-033 — Extreme Convex Sweeps Can Normalize To Non-Unit Directions

**Resolved:** 2026-07-14 **Source:** 95%-to-100% coverage hardening, convex
sweep termination review **Affected area:** FixedMathSharp vector magnitude,
normalization, comparison, and averaging; Gravitas 2D, 3D, and mixed query/CCD
sweep admission, GJK, conservative advancement, and concave-mesh hit geometry

**Follow-up status:** Closed by
[`FixedMathSharp Foundation Hardening`](https://github.com/mrdav30/FixedMathSharp/blob/main/docs/feature-work/done/2026-07-14-fixedmathsharp-foundation-hardening-plan.md).
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

### GRV-Issue-032 — Extreme Collider Bounds Underestimated CCD Proxy Radius

**Resolved:** 2026-07-13 **Source:** 95%-to-100% coverage hardening, 3D CCD
helper review **Affected area:** FixedMathSharp vector magnitude/distance and
Gravitas 2D/3D continuous-collision proxy radius, candidate admission, and
`Auto` gating

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

### GRV-Issue-031 — FixedMathSharp Rays Now Treat Only Exact-Zero Slab Directions As Parallel

**Resolved:** 2026-07-13 **Source:** 95%-to-100% coverage hardening, shared
segment-box clipping review **Affected area:** FixedMathSharp `FixedRay` and
`FixedRay2d` slab intersection

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

### GRV-Issue-030 — FixedMathSharp Vector Midpoints Saturated Before Halving

**Resolved:** 2026-07-13 **Source:** 95%-to-100% coverage hardening,
physics-material average review **Affected area:** FixedMathSharp scalar and
vector midpoint helpers plus Gravitas `PhysicsMaterialCombine.Average`

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

### GRV-Issue-029 — Overlong Settings Collision Matrix Rows Were Silently Truncated

**Resolved:** 2026-07-13 **Source:** 95%-to-100% coverage hardening, final
settings branch review **Affected area:**
`PhysicsSettingsSaver.CreateCollisionMatrix()`

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

### GRV-Issue-028 — Pending CCD Replay Hashes Depended On Deleted Collider ID History

**Resolved:** 2026-07-13 **Source:** 95%-to-100% coverage hardening, dimensional
body replay review **Affected area:** `SolidBody.ContributeReplayHash(...)` and
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

### GRV-Issue-027 — Mesh-Cone Triangle Containment Used Contact-Oriented Normals

**Resolved:** 2026-07-13 **Source:** 95%-to-100% coverage hardening, mesh-cone
branch review **Affected area:**
`CollisionDetection.TryFindMeshConeTriangleContact(...)`

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

### GRV-Issue-026 — Small CCD Proxy Radii Could Turn Tangency Into A Closing Hit

**Resolved:** 2026-07-13 **Source:** 95%-to-100% coverage hardening, shared
relative-sweep review **Affected area:** `ContinuousCollisionMath` relative
sphere and circle sweeps

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

### GRV-Issue-025 — Mesh Scale And Surface-Shell Mass Did Not Match Authored Geometry

**Resolved:** 2026-07-12 **Source:** 95%-to-100% coverage hardening, mesh
transform/mass follow-up **Affected area:** `LSMeshCollider`, `PhysicsMesh`,
compound mesh parts, and `MeshInertiaPolicy.SurfaceApproximation`

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

### GRV-Issue-024 — 3D Compound Mass And Geometry Used Incompatible Frames And Measures

**Discovered:** 2026-07-12 **Resolved:** 2026-07-12 **Source:** 95%-to-100%
coverage hardening, `LSCompoundCollider` block review **Affected area:** 3D
compound mass distribution, inertia, owner offsets, conservative query radius,
and private part transforms

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

### GRV-Issue-023 — 3D Motion State Could Leak Across Reuse And Apply Incorrect Rotational Dynamics

**Discovered:** 2026-07-12 **Resolved:** 2026-07-12 **Source:** 95%-to-100%
coverage hardening, `SolidBody.Motion` review **Affected area:** 3D body
reset/reuse, grounded angular friction, and gyroscopic precession

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

### GRV-Issue-022 — Synchronous 2D Contact Callbacks Could Corrupt Pair Teardown And Reuse

**Discovered:** 2026-07-12 **Resolved:** 2026-07-12 **Source:** 95%-to-100%
coverage hardening, 2D pair/response lifecycle review **Affected area:**
`GravitasPhysics2DService` response expansion, pair cleanup, deactivation, and
pooling

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

### GRV-Issue-021 — Fixed-Point Sphere Tangency Could Be Rejected By Normalization Residue

**Discovered:** 2026-07-12 **Resolved:** 2026-07-12 **Source:** 95%-to-100%
coverage hardening, 3D raycast segment review **Affected area:**
`RaycastSegmentWorker.CheckSphereOverlaps(...)`

Historical RCA: the former closest-point and normalized-direction quadratic
disagreed by one raw unit for a near-tangent fixture, so the discriminant was
clamped to zero. Full-domain radial hardening later proved that the stored
Q32.32 values in that fixture are an exact one-raw-unit miss, not a tangent; the
clamp therefore invented a contact.

Superseding fix: sphere segments now use the authored segment over `[0, 1]` and
FixedMathSharp's exact bounded interval solver. Exact stored-value tangency
still returns one hit, while the historical `(0,0,0)->(3,4,0)` fixture against
center `(1/5,3/10,0)` and radius `1/50` is retained as an explicit near-miss
regression. No epsilon or discriminant clamp remains.

### GRV-Issue-020 — Context Disposal Ordering Could Admit Inactive Worlds And Invalidate Disabled CCD Handoffs

**Discovered:** 2026-07-11 **Resolved:** 2026-07-11 **Source:** 95%-to-100%
coverage hardening, world-context lifecycle review **Affected area:**
`GravitasWorldContext` world registration/disposal and disabled-service
late-simulate CCD state

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

### GRV-Issue-019 — Partition Teardown Logged Errors After Host Grid Removal

**Discovered:** 2026-07-11 **Resolved:** 2026-07-11 **Source:** 95%-to-100%
coverage hardening, dimensional partition-service review **Affected area:**
2D/3D partition clear and awake-state refresh after host grid lifecycle changes

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

### GRV-Issue-018 — Repeated Bodyless Initialization Could Orphan Collider Registrations

**Discovered:** 2026-07-11 **Resolved:** 2026-07-11 **Source:** 95%-to-100%
coverage hardening, 3D partition service review **Affected area:** 2D/3D
bodyless collider initialization, registry identity, partition membership, and
reset reuse

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

### GRV-Issue-017 — Deactivation Duplicated Teardown And Allowed Stale Collider Ownership

**Discovered:** 2026-07-11 **Resolved:** 2026-07-11 **Source:** 95%-to-100%
coverage hardening, 3D grounding lifecycle review **Affected area:** 2D/3D body
and collider deactivation, physics-service dessimilation, partition ownership,
reusable bindings, and 3D body loading

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

### GRV-Issue-016 — Cuboid Frontal Area Selected The Wrong Face And Ignored Diagonal Projection

**Discovered:** 2026-07-11 **Resolved:** 2026-07-11 **Source:** 95%-to-100%
coverage hardening, `LSCuboidCollider` surface review **Affected area:** 3D
cuboid linear/angular drag area and authored collider geometry surface

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

### GRV-Issue-015 — Capsule Drag And Inertia Ignored Direction And Hemisphere Centroids

**Discovered:** 2026-07-12 **Resolved:** 2026-07-12 **Source:** 95%-to-100%
coverage hardening, `LSCapsuleCollider` block review **Affected area:** 3D
capsule linear/angular drag area and solid mass properties

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

### GRV-Issue-014 — Inactive SolidBody2D Loads Could Preserve Or Invent Runtime Activity

**Discovered:** 2026-07-11 **Resolved:** 2026-07-11 **Source:** 95%-to-100%
coverage hardening, `LSCollider2D` lifecycle review **Affected area:**
`SolidBody2D.RecordData(...)`, inactive body snapshot loads, body/collider
registration teardown, and reusable shells

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

### GRV-Issue-013 — 2D Collider Teardown And Load Paths Could Preserve Invalid Runtime Ownership

**Discovered:** 2026-07-11 **Resolved:** 2026-07-11 **Source:** 95%-to-100%
coverage hardening, `LSCollider2D` parity review **Affected area:** 2D collider
activation, body-owned teardown, unbound compound loads, and primary/mixed
partition normalization

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

### GRV-Issue-012 — 2D Query Version Reuse Could Suppress Live Colliders

**Discovered:** 2026-07-11 **Resolved:** 2026-07-11 **Source:** 95%-to-100%
coverage hardening, 2D query-stamp parity audit **Affected area:**
`GravitasQuery2DService` raycast, sweep, and overlap-query deduplication state

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

### GRV-Issue-011 — 3D Collider Active-State Transitions Could Leave Invalid Partition Ownership

**Discovered:** 2026-07-11 **Resolved:** 2026-07-11 **Source:** 95%-to-100%
coverage hardening, `LSCollider` lifecycle review **Affected area:** 3D collider
activation, primary/mixed partition ownership, query visibility, and collider
state loading

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

### GRV-Issue-010 — 3D Query Version Reuse Could Suppress Live Colliders

**Discovered:** 2026-07-11 **Resolved:** 2026-07-11 **Source:** 95%-to-100%
coverage hardening, 3D query surface review **Affected area:**
`GravitasQuery3DService` raycast, sweep, and circle-query deduplication state

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

### GRV-Issue-009 — CCD Rejected Finite Heavy-Body Response As Zero Inverse Mass

**Discovered:** 2026-07-10 **Resolved:** 2026-07-10 **Source:** 95%-to-100%
coverage hardening, dynamic TOI loop review **Affected area:** 2D, 3D, and mixed
dynamic/kinematic CCD impulse response

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

### GRV-Issue-008 — Exhausted CCD Budget Left Pending Body Handoffs Alive

**Discovered:** 2026-07-10 **Resolved:** 2026-07-10 **Source:** 95%-to-100%
coverage hardening, queued CCD handoff audit **Affected area:**
`GravitasPhysicsService`, `GravitasPhysics2DService`, `SolidBody`, and
`SolidBody2D` continuous-collision handoff state

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

### GRV-Issue-007 — Context-Driven Mixed CCD Handoffs Could Drain Per Service Before The Shared Budget

**Discovered:** 2026-07-06 **Resolved:** 2026-07-06 **Source:** Coverage
Workstream 7 CCD handoff branch audit **Affected area:**
`GravitasWorldContext.LateSimulate`, `GravitasPhysicsService`,
`GravitasPhysics2DService`, mixed 2D/3D continuous collision handoff chains

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

### GRV-Issue-006 — 2D Active-State Toggle Preserved Mixed Partition Membership

**Discovered:** 2026-07-06 **Resolved:** 2026-07-06 **Source:** Coverage
Workstream 4 serialization/replay/authoring branch audit **Affected area:**
`LSCollider2D.IsActive`, mixed 2D/3D static collider partition membership

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

### GRV-Issue-005 — Reduced SAT Helper Could False-Positive Rotated Cuboid And Convex Mesh-Mesh Paths

**Discovered:** 2026-07-06 **Resolved:** 2026-07-06 **Source:** Mesh-cuboid
fallback SAT RCA **Affected area:** `CollisionDetection.Cuboid`,
`CollisionDetection.Mesh`, rotated cuboid vs cuboid and convex mesh vs convex
mesh fallback contact generation

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

### GRV-Issue-004 — Mesh-Cuboid Fallback SAT Could False-Positive Without Edge-Cross Axes

**Discovered:** 2026-07-06 **Resolved:** 2026-07-06 **Source:** Coverage
Workstream 1 zombie-code sweep and subagent geometry review **Affected area:**
`CollisionDetection.Mesh`, convex mesh vs cuboid fallback contact generation

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

### GRV-Issue-003 — Rotated Cuboid Raycast Clipped The Enclosing AABB Instead Of Local Slabs

**Discovered:** 2026-07-06 **Resolved:** 2026-07-06 **Source:** Coverage
Workstream 1 branch inventory and subagent query review **Affected area:**
`RaycastSegmentWorker.CheckOBBoxOverlaps(...)`, 3D raycast queries against
rotated `LSCuboidCollider`

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

### GRV-Issue-002 — 3D Direct Collider Inactive Load Preserved Stale Partition State

**Discovered:** 2026-07-06 **Resolved:** 2026-07-06 **Source:** Coverage Roadmap
E review and 2D/3D serialization parity audit **Affected area:**
`LSCollider.RecordData(...)`, 3D bodyless collider serialization, primary and
mixed partition state cleanup

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

### GRV-Issue-001 — Mixed Discrete Response Can Reverse Restitution-Heavy Kinematic CCD Handoff Velocity

**Discovered:** 2026-06-23 **Resolved:** 2026-06-25 **Source:** CCD
service-level island solver validation **Affected area:**
`CollisionResponseMixed`, mixed CCD handoff tests,
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
