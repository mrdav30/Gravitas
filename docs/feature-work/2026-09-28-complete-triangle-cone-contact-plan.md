# Complete Triangle/Cone Contact

**Status:** Design ready for review; runtime implementation not started.  
**Started:** 2026-09-28.  
**Repositories:** FixedMathSharp and Gravitas.  
**Issue:** [GRV-Issue-086](issue-tracker.md#grv-issue-086---meshcone-contact-uses-incomplete-triangle-sampling-and-scalar-frame-rejection).

## Goal And Scope

Replace the incomplete mesh/cone triangle contact path with one complete
FixedMathSharp relation. Select collision admission, normal, penetration depth
and both contact anchors from the same geometric feature. Preserve deterministic
lockstep behavior, canonical frames and allocation-free warmed execution.

The approved direction includes rejecting genuine positive gaps in the new
triangle query. Keep the repair focused and navigable: reuse existing arithmetic
and delete superseded downstream heuristics instead of retaining a second answer
path. This document is the single cross-repository coordination record; keep
execution results here rather than creating separate progress documents.

Included:

- FixedMathSharp finite cone/triangle geometry and its focused tests.
- Gravitas's shared convex/concave mesh-triangle consumer, including compound
  dispatch that already reaches that consumer.
- Matching benchmarks, allocation/resource gates, coverage and affected public
  documentation.

Not included:

- [GRV-Issue-087](issue-tracker.md#grv-issue-087---cone-volume-queries-reject-an-intersecting-mesh-when-the-apex-cannot-enter-its-scalar-frame): cone-volume queries need a minimum-axial intersection witness, not a penetration contact.
- Replacing the closed-convex containment/GJK fallback, its tolerance policy,
  other cone/primitive or mixed cone/slab solvers, CCD, or collision response.
- A generic collider framework, speculative contact margin, new dependencies,
  persistent caches, a new benchmark project, or new CI infrastructure.

## Evidence And Root Cause

The issue tracker retains the exact fixtures and independent geometric proofs.
Three diagnostic cases establish the current problem:

1. A real intersection is skipped when the cone center cannot be represented
   in the triangle's local scalar frame.
2. An ordinary radius-1, height-2 cone misses a triangle crossing its side:
   neither the triangle point nearest the cone center nor one projected cone
   support detects that intersection.
3. An ordinary contact combines a nearest-cone-surface depth with a separately
   chosen triangle-face normal and inconsistent witnesses.

The source inspection is at Gravitas `3471085` / FixedMathSharp `3f7a606`.
Existing Release binaries reproduced these cases through the actual collider
initializer and dispatcher in PowerShell/.NET 10. These are diagnostic evidence,
not the required fresh .NET 8 regression or performance gates.

## Public Geometry Contract

Add `FixedTriangle.TryGetCenteredFiniteConeContact(...)` alongside the existing
rigid-frame contact APIs. It accepts the triangle origin/normalized rotation,
cone center/normalized rotation, full cone height, nonnegative radius, and an
`out FixedContactAnchors` result. The cone's local axis is +Y, with its base at
`-height/2` and apex at `+height/2`.

- Invalid rotations, nonpositive height and negative radius follow the existing
  argument-validation conventions. Zero radius is a supported axial segment;
  retain its actual minimum exit depth, which can be positive when the segment
  pierces the triangle. Do not equate zero volume with zero exit distance.
- Preserve the established `FixedTriangle.IsDegenerate` admission contract,
  including its documented exact-normal-magnitude threshold. Do not invent a
  second triangle-degeneracy policy or change neighboring relations here.
- For admitted triangles, classify authored geometry before final rounding:
  separation returns `false`, exact touching returns a zero-depth contact, and
  overlap returns a contact whose minimum exit depth may itself round to zero.
  A positive gap must not become contact merely because a scalar approximation
  rounds that gap to zero or places it within `Fixed64.Epsilon`.
- Normal, depth and witnesses belong to the same selected feature. The normal
  points from triangle toward cone. Anchors retain the authored triangle and
  cone frames; overflowing relative centers or absolute world points must not
  reject otherwise admitted geometry.
- Keep odd-raw half-height and other exact anchor terms until the existing final
  materialization boundary. Round final ordinary values nearest-even; preserve
  the existing conceptual-overflow `DepthIsClamped` contract.
- Use explicit deterministic feature order and retain the earlier candidate on
  exact depth ties. Do not promise permutation-invariant normals for genuinely
  nonunique ties; unique geometric winners must be winding/reindex invariant.

Keep `TryGetCenteredFiniteConeSupportContact(...)` unchanged. Its documented job
is to project one caller-selected support, not prove complete intersection.
It must not become an alias for the new relation.

### What A Positive Gap Means To Gravitas

For that triangle, a separated result contributes no manifold point and no
contact impulse. The mesh scan still tests other admitted triangles. Broad-phase
bounds may continue to admit the pair, and subsequent simulation steps evaluate
its new pose. Closed-convex containment still uses its existing separate fallback.

This is not a new global separation policy: the fallback's existing GJK tolerance
is outside this repair. A future speculative-contact margin would need an
explicit physics contract. Fast traversal between discrete poses belongs to CCD,
not to falsely reporting a gap as overlap.

## Geometry Design And Reuse

A finite cone is the convex hull of its apex and base disk. For a local support
direction `m`, full height `H`, radius `R`, and
`rho = sqrt(m.x*m.x + m.z*m.z)`, its support value is:

```text
max((H/2)*m.y, -(H/2)*m.y + R*rho)
```

This partitions directions into apex support, base-rim support, and their shared
lateral-generator boundary `H*m.y = R*rho`; the downward axial direction owns
the whole base disk. Triangle face, edge and vertex support regions must be
combined with these cone regions before any candidate can reject or win.

Use a focused cone/triangle owner with one exact winner. Establish a complete
enumeration of interior stationary candidates and region boundaries, including
zero-radial directions, tangent roots, collapsed charts and the radius-zero
case. Squared stationary equations require their original unsquared signs and
both shapes' feature-admission predicates. A root outside its valid region is
not a separation certificate. Do not assume the cylinder's skipped candidates
or pruning proofs apply to a cone.

Reuse these existing responsibilities rather than reproducing them:

| Responsibility | Existing owners to compose |
| --- | --- |
| Exact rigid frames and coordinate products | `WideRationalBasis3d`, `WideRigidProjection`, the existing finite-axis/polytope frame machinery |
| Signed radial support values, stable depth ranking and final normalization | `CylinderContactAlgebra`, `ConvexContactCandidate`, existing convex-contact materializers |
| Stationary base-rim edge algebra and exact roots | `CylinderEdgeContactPolynomial`, `FiniteAxisValueRoot`, `ConvexContactValueRoot` |
| Canonical endpoints, odd-raw dimensions and final anchor terms | `FixedPointAnchor` and `FixedPointAnchorTerm3d` |

Reuse is conditional on the actual mathematical contract. In particular, the
existing radial stationary expression can serve base-rim candidates, but does
not supply cone-region admission or lateral-generator witnesses. Reuse or
extract proven shared leaf operations; do not copy wide limbs, a root isolator,
normalization or rounding into a new cone implementation.

Keep one clear cone contact entry point. Split analytic selection, stationary
features or witnesses only when they form cohesive responsibilities. Keep each
non-private type in its own file. Avoid both oversized owners and one-method
forwarding files. Rename an existing helper only if its actual shared use makes
the old name misleading; migrate all callers without compatibility wrappers.

## Gravitas Integration

Replace the nearest-center-point and projected-support branches in
`CollisionDetection.Cone.cs` with the selected upstream contact. Consume its
normal, depth, clamp flag and two anchors together.

Preserve the existing BVH candidate traversal, minimum-depth triangle reduction,
earlier-candidate tie behavior, pair orientation, materials and manifold
ownership. Keep the closed-convex fallback: a cone wholly inside a solid mesh
need not intersect a surface triangle. No manifold-enrichment redesign is needed.

Delete the obsolete local-center conversion, unchecked cone-frame conversion,
unrelated surface-distance/normal combination and any now-unreferenced helper
after checking all callers. Do not change cone/sphere or cone/convex behavior as
an incidental consequence of cleanup.

## Acceptance And Execution Outline

### 0. Regressions, Proof And Baselines

- [ ] Reproduce the three recorded failures as behavioral .NET 8 tests using
  existing fixtures; keep the true separated full-domain control.
- [ ] Replace the old mesh-triangle positive-epsilon-gap expectation with an
  independently proved separated case; retain separate convex-fallback tests.
- [ ] Derive cone-specific feature completeness, admission, coefficient-width
  and simultaneous scratch bounds; obtain independent correctness/Ponytail review
  before implementing the solver.
- [ ] Capture matching cone baselines before production changes. Include base,
  side, apex, oblique rim, touch/gap and full-domain cases in existing benchmark
  projects. Label old incorrect outcomes rather than presenting them as an
  equivalent-correctness baseline. Final preflights enforce corrected behavior.

### 1. Complete Upstream Geometry

- [ ] Implement the relation with shared exact arithmetic and winner-only final
  materialization; prove any fast path before bypassing complete feature work.
- [ ] Verify base, side, rim, apex and triangle face/edge/vertex winners;
  containment, exact tangency, raw-neighbor separation/penetration, zero radius,
  degeneracy thresholds, winding/reindexing, rotations, odd-raw heights, extreme
  relative frames and unmaterializable world witnesses.
- [ ] Check known depth/normal/anchor results, not only hit booleans. Account for
  independently rounded output coordinates with derived bounds where needed.
- [ ] Preserve warmed zero allocation and bounded worker-stack behavior using
  existing resource-test patterns, including large extents and distinct frames.

### 2. Downstream Cutover

- [ ] Consume the complete relation and remove superseded code.
- [ ] Verify actual mesh dispatch in both pair orders, convex/concave surfaces,
  compound routing, multiple-triangle selection and closed-convex containment.
- [ ] Verify matched anchors, normals, depth/clamp propagation, repeated results
  and warmed allocations. Keep CCD and unrelated cone policies unchanged.

### 3. Verification, Performance And Closeout

- [ ] Build FixedMathSharp and Gravitas in `Release` and `ReleaseLean` for
  `netstandard2.1` and `net8.0`; run both full .NET 8 test suites and retain
  100% reachable line, branch and method coverage without new exclusions.
- [ ] Rerun identical cone workloads and shared cylinder/capsule-slab controls.
  Report per-case correctness and cost honestly; investigate material ordinary
  path regressions and record accepted expensive features in the benchmark
  tracker. Do not weaken geometry to reach an arbitrary speed target.
- [ ] Obtain final independent correctness and Ponytail review; resolve findings.
- [ ] Update affected XML/wiki/API guidance, benchmark notes and the complexity
  register where needed. Keep public docs evergreen and build both DocFX sites
  with warnings as errors.
- [ ] Resolve GRV-Issue-086 only after all required gates pass. Keep GRV-Issue-087
  separate. Condense final evidence here and move this document to `done`.

Use `UseLocalLsfStack=true` for coordinated validation; this is not evidence of
released-package readiness. Run only one heavy workload at a time, using the
established two-core, BelowNormal launcher and `DOTNET_PROCESSOR_COUNT=2`.
Review agents must not launch competing builds, tests or benchmarks.

Leave all changes unstaged/uncommitted for user review. Do not release packages
or add release-workaround configuration. Include concise commit suggestions
for each changed repository in the implementation handoff.
