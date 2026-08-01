# Full-Domain Triangle-Pair Contact

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> `superpowers:subagent-driven-development` (recommended) or
> `superpowers:executing-plans` to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace Gravitas's narrowed mesh triangle-pair SAT with one public,
full-domain FixedMathSharp rigid-frame contact relation.

**Architecture:** FixedMathSharp will retain all triangle axes, rigid
projections, interval overlap, and normalized-depth ranking in existing wide
arithmetic, then materialize one ordinary fixed-point contact. Gravitas will
keep mesh BVH traversal and manifold policy while deleting its duplicate scalar
SAT and cached collision-triangle wrapper.

**Tech Stack:** C# 11, .NET Standard 2.1/.NET 8, Q32.32 `Fixed64`, xUnit v3,
BenchmarkDotNet, Coverlet, ReportGenerator, local FixedMathSharp project links.

## Global Constraints

- Determinism, performance, maintainability, and correctness are release gates.
- FixedMathSharp owns policy-neutral arithmetic and computational geometry;
  Gravitas owns physics admission and response policy.
- No public or protected signature may expose FixedMathSharp wide types.
- Stable authored axis, triangle, collider, and contact ordering must remain
  deterministic.
- Warmed production paths must allocate zero managed bytes.
- FixedMathSharp and Gravitas must retain 100% reachable line, branch, and
  method coverage without hollow API-shape tests or zombie branches.
- Release and ReleaseLean must pass for both target frameworks with zero
  warnings.
- Local project-link files remain unstaged and uncommitted.
- All implementation changes remain unstaged/uncommitted for user review;
  agents provide recommended commit messages only.

---

**Status:** Phase 3 complete; stopped because Phase 3 added a new release-blocking queue item  
**Started:** 2026-07-31  
**Repositories:** FixedMathSharp, Gravitas  
**Queue item:** `Mesh Triangle-Triangle SAT Can Saturate Before Axis Classification`

## Goal

Provide one reusable FixedMathSharp triangle/triangle contact relation that is
correct across the complete admitted rigid-frame fixed-point domain. Gravitas
will consume that relation instead of projecting and ranking mesh triangles
through narrowed scalar arithmetic.

The work must preserve:

- deterministic axis order and earlier-candidate tie ownership;
- allocation-free warmed mesh/mesh collision;
- 100% reachable line, branch, and method coverage in FixedMathSharp and
  Gravitas;
- FixedMathSharp ownership of policy-neutral exact geometry;
- Gravitas ownership of mesh candidate traversal, manifold admission, material
  policy, and collision response;
- public APIs that expose ordinary fixed-point geometry rather than wide
  implementation types.

## Root Cause

`MeshTriangleContactGenerator.TryTestTriangles(...)` currently creates face,
in-plane edge, and cross-edge axes correctly, but narrows every later SAT
operation:

- triangle vertices are projected with `Vector3d.Dot`;
- interval endpoints are subtracted as `Fixed64`;
- candidate depths are ranked through separately saturated squared products;
- axis magnitude and final depth are narrowed before the winning axis is known;
- normal orientation depends on a narrowed centroid difference.

Large relative geometry and unnormalized cross-product axes can therefore
change separation, penetration ranking, or orientation before the final public
result is materialized. Rescaling, clamping, or retrying in Gravitas cannot
recover the discarded ordering information.

FixedMathSharp already owns the required exact difference products, rational
rigid bases, wide interval arithmetic, normalized-depth comparison, round-to-
even depth materialization, and wide normalization. Its generic convex-hull
contact relation is not a complete substitute: two zero-thickness triangles
also require each face normal crossed with its own edges to classify coplanar
separation.

The existing normalized-depth materializer has one additional proven fast-path
precondition: a non-perfect-square axis magnitude must be at least one half of
one Q32.32 scale unit. Near-parallel cross-edge axes can be smaller while still
being mathematically nonzero. FixedMathSharp will retain the current constant-
time path for proven axes and add an allocation-free exact midpoint-search
fallback for smaller axes. This hardens the shared primitive rather than
asserting a precondition triangle pairs cannot guarantee.

## Locked Public Contract

`FixedTriangle` will expose the rigid-frame relation directly:

```csharp
public readonly bool TryGetContact(
    Vector3d firstOrigin,
    FixedQuaternion firstRotation,
    Vector3d secondOrigin,
    FixedQuaternion secondRotation,
    FixedTriangle second,
    out FixedContactAnchors contact);
```

- Both rotations must be normalized, matching existing rigid-frame triangle
  contact APIs.
- A triangle whose exact face normal is mathematically zero has no SAT surface
  and returns `false`; a nonzero triangle or candidate axis is not discarded
  merely because its narrowed magnitude would fall at or below
  `Fixed64.Epsilon`.
- The returned normal points from the first triangle toward the second.
- The returned anchors remain in their respective triangle frames.
- Negative interval overlap is separation.
- Exact interval equality is a valid zero-depth geometric contact, consistent
  with existing FixedMathSharp contact relations. Gravitas retains its own
  broad-phase and manifold-admission policy.
- Only a mathematically zero axis is skipped. Axis scale must not change
  admission or minimum-depth selection.
- Penetration depth is rounded once, half to even. A conceptual result beyond
  the positive Q32.32 domain returns `Fixed64.MaxValue` with
  `DepthIsClamped == true`.
- Exact normalized-depth ties retain the earlier axis.
- Reversing the triangles inverts the normal when the winning axis has a
  nonzero exact centroid projection. An exact-zero centroid projection retains
  generated axis sign and promises repeatability, not antisymmetry.
- No wide type appears in a public signature.

## FixedMathSharp Design

A focused internal `WideTriangleRelations` owner will implement the relation.
It will reuse the existing `WideArithmetic`, `WideGeometry`,
`WideRationalBasis3d`, and `WideNormalization` contracts rather than creating a
second limb, ratio, or normalization layer.

The reducer will visit axes in the current deterministic Gravitas order:

1. the first triangle face normal;
2. the second triangle face normal;
3. for each authored edge index:
   - first face normal crossed with the matching first edge;
   - second face normal crossed with the matching second edge;
   - that first edge crossed with each second edge in authored order.

Normals, edges, cross products, rigid transforms, vertex projections, interval
overlap, squared axis magnitude, and normalized-depth comparison remain wide.
The first separating axis exits immediately. Otherwise the reducer retains one
winning exact penetration and materializes only its final unit normal, depth,
clamp state, and local-frame witnesses.

The arithmetic widths are part of the implementation contract. A raw coordinate
difference is below 2^64; an exact local face-normal component is below 2^129;
and a local face-normal-cross-edge component is below 2^194. Rational rigid
transformation keeps every generated axis below the conservative 2^267 bound,
so all axes fit Signed320. Their squared magnitudes remain below 2^536, while
common-denominator vertex projections, interval overlap, and centroid-
orientation numerators fit Signed576. The shared rational-basis denominator
product fits Signed320. Implementations must use checked/proven narrowing at
these boundaries rather than silently accepting general Signed320 inputs.

The winning axis is oriented by the exact projection of the second triangle's
vertex sum relative to the first triangle's vertex sum. This is the full-domain
equivalent of the current centroid direction without first rounding either
centroid or subtracting two `Vector3d` values. An exact zero projection retains
the generated axis sign, preserving deterministic axis-order ownership.

Existing common SAT values or rigid projection helpers will be extracted from
`WideOrientedBox` only where both owners genuinely consume them. Callers will
invoke the shared owner directly; no one-line forwarding façade or generic SAT
framework will be introduced.

Contact witnesses will be selected through the existing full-domain rigid
triangle closest-anchor machinery. That reducer is currently misplaced in the
self-contained `WideOrientedBox.TriangleClosestPoint.cs` partial; it will move
into `WideTriangleRelations.ClosestPoint.cs` without a forwarding façade. The
first triangle is queried against the second triangle's deterministic centroid
anchor, then the second triangle is queried against the retained first anchor.
The pair reducer calls the internal closest-point core after its one-time frame
validation instead of repeating both public validation passes. SAT
classification and depth do not depend on this representable witness selection.
The current Gravitas fallback that invents `pointA - normal * depth` will be
removed.

## Gravitas Adoption

`MeshTriangleContactGenerator` will continue to own BVH candidate traversal and
stable triangle ordering. For each admitted pair it will call the public
FixedMathSharp relation using the two meshes' canonical local triangles and
rigid frames.

The adoption will delete the local scalar SAT, projection helpers, axis-depth
comparison, centroid-direction orientation, and synthetic second-point
fallback. The returned anchors, normal, depth, and clamp flag will flow directly
into the existing manifold.

The already transformed triangle remains necessary for the second mesh's local
BVH query. It will not become collision-math authority after candidate
selection.

## Related-Path Audit

The closure pass will inspect convex-mesh query and mixed mesh-triangle reducers
for the same structural failure: narrowed unnormalized-axis projection followed
by scalar overlap or normalized-depth ranking.

- Paths already delegating to FixedMathSharp full-domain triangle or finite-
  slab relations require no change.
- A confirmed consumer of the same triangle-pair relation will adopt the shared
  contract in this workstream.
- A different arithmetic defect will be documented separately rather than
  broadening this implementation with an unrelated solver.

## Verification Contract

Focused FixedMathSharp regressions will cover:

- ordinary intersecting and separated triangles;
- coplanar overlap, coplanar separation, and exact touching;
- mirrored near-minimum/near-maximum rigid frames;
- long triangles and large edge-cross axes;
- tiny nonzero axes that scalar epsilon filtering would discard;
- exact minimum-depth ties and authored axis order;
- round-to-even normalized-depth arithmetic plus public ordinary and clamped
  depth materialization;
- reversed triangle order and normal orientation;
- warmed zero allocation.

Gravitas regressions will prove mesh/mesh adoption, canonical local-anchor
retention, reversed dispatch, stable manifold behavior, clamp propagation, and
warmed zero allocation. Existing mesh/mesh benchmark rows will be measured
before and after the adoption. Both repositories must pass Release,
ReleaseLean, package, and 100% reachable line/branch/method coverage gates.

## Rejected Approaches

- **Generic convex-hull substitution:** misses coplanar in-plane separation
  axes for zero-thickness triangles.
- **Gravitas-local wide SAT:** duplicates policy-neutral geometry and the
  FixedMathSharp limb contract.
- **Scalar rescaling, clamping, or retry:** cannot preserve exact separation or
  normalized-depth ordering across the full domain.
- **A public wide-axis or penetration type:** leaks implementation width without
  adding a useful host-facing geometry abstraction.
- **A generic configurable SAT framework:** adds indirection and speculative
  abstraction to one concrete missing relation.

## Non-Goals

- No mesh candidate traversal or BVH redesign.
- No manifold clipping or multi-point triangle contact redesign.
- No change to Gravitas response, material, or touching-contact policy.
- No public FixedMathSharp wide arithmetic.
- No unrelated query hardening without a separately confirmed root cause.

## File Ownership Map

### FixedMathSharp

- Create `src/FixedMathSharp/Geometry/Primitives/Triangles/FixedTriangle.PairContacts.cs`
  for the public validated rigid-frame triangle-pair API.
- Create `src/FixedMathSharp/Geometry/Wide/Triangles/WideTriangleRelations.cs`
  for triangle-specific axis generation, interval classification, orientation,
  and witness materialization.
- Move
  `src/FixedMathSharp/Geometry/Wide/OrientedBox/WideOrientedBox.TriangleClosestPoint.cs`
  to
  `src/FixedMathSharp/Geometry/Wide/Triangles/WideTriangleRelations.ClosestPoint.cs`
  and update `FixedTriangle.GetClosestPointAnchor(...)` to call its real owner.
- Create `src/FixedMathSharp/Geometry/Wide/Common/WideAxis3.cs` for the
  existing Signed320 three-component axis, exact cross product, negation, zero
  test, and squared magnitude.
- Create
  `src/FixedMathSharp/Geometry/Wide/Common/WidePointSpanPenetration.cs` for the
  exact overlap/axis/common-denominator candidate already shared by box,
  triangle, and hull reducers.
- Create
  `src/FixedMathSharp/Geometry/Wide/Common/WideRigidProjection.cs` for rigid-
  basis axis transformation plus local-offset and origin-difference projection.
- Modify `src/FixedMathSharp/Numerics/Wide/WideArithmetic.Comparison.cs` to
  retain its normalized-depth fast path and add the exact tiny-axis fallback.
- Modify the affected `Geometry/Wide/OrientedBox/WideOrientedBox.*.cs` partials
  to call those common owners directly and delete the displaced nested types and
  helpers. Do not leave forwarding methods behind.
- Create
  `tests/FixedMathSharp.Tests/Geometry/Primitives/FixedTriangle.PairContacts.Tests.cs`
  for public behavior and full-domain regressions.
- Modify
  `tests/FixedMathSharp.Tests/Numerics/Wide/WideNormalizedDepthRounding.Tests.cs`
  to verify the fallback against its existing BigInteger oracle.
- Modify `tests/FixedMathSharp.Benchmarks/OrientedBoxAnchorBenchmarks.cs` to add
  the warmed triangle-pair contact row beside existing semantic-contact rows.
- Modify `docs/wiki/bounds-and-geometry.md`, `docs/MIGRATION.md`, and `README.md`
  only where the additive v7 triangle relation changes the documented surface.
- Modify `docs/complexity-exceptions.md` with evergreen ownership and measured
  complexity evidence if the retained wide reducer crosses its documented
  complexity threshold.

### Gravitas

- Modify
  `src/Gravitas/CollisionHandling/Detection/3D/Mesh/MeshTriangleContactGenerator.cs`
  to retain BVH candidate gathering but delegate contact authority to
  `FixedTriangle.TryGetContact(...)`.
- Delete
  `src/Gravitas/CollisionHandling/Detection/Geometry/CollisionTriangle.cs` and
  `tests/Gravitas.Tests/CollisionHandling/CollisionTriangleTests.cs` after their
  scalar SAT-only cached fields have no production consumer.
- Create
  `tests/Gravitas.Tests/CollisionHandling/MeshTrianglePairContactTests.cs` for
  focused public collision regressions rather than further enlarging the broad
  shape-pair suite.
- Modify
  `tests/Gravitas.Benchmarks/CollisionHandling/CollisionDetectionBenchmarks.cs`
  only if the existing mesh/mesh rows need an extreme-domain fixture; otherwise
  reuse the current ordinary, concave, dense, contact-heavy, and closed-shell
  rows unchanged.
- Modify `docs/wiki/COLLISION_PIPELINE.md`, `docs/feature-work/issue-tracker.md`,
  `docs/feature-work/benchmark-signal-hardening-backlog.md`, and this plan at
  their matching phase/closure checkpoints.

## Battle Plan

### Phase 0: Contract, Reproduction, And Baseline

**Interfaces**

- Consumes: the approved public signature and locked semantics above.
- Produces: reproducible red tests and before-change mesh/mesh benchmark data;
  no production behavior changes.

- [x] Trace every current scalar SAT operation and its callers.
- [x] Confirm the generic convex-hull relation lacks triangle in-plane axes.
- [x] Confirm mixed finite-slab triangle reducers already use FixedMathSharp
  full-domain contracts rather than this scalar SAT.
- [x] Approve public `FixedTriangle.TryGetContact(...)` ownership.
- [x] Run the existing focused FixedMathSharp triangle and Gravitas mesh/mesh
  tests before editing source:

  ```powershell
  dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj `
      --configuration Release `
      --filter "FullyQualifiedName~FixedTriangle|FullyQualifiedName~FixedConvexHullRelations"

  dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj `
      --configuration Release `
      --filter "FullyQualifiedName~CollisionDetectionShapePairTests|FullyQualifiedName~ConcaveMeshCollisionTests"
  ```

- [x] Capture the authoritative before-change Gravitas mesh/mesh benchmark rows:

  ```powershell
  dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj `
      --configuration Release

  dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll `
      collision-detection --filter "*MeshMesh*" -j Short -i `
      --exporters json `
      --artifacts artifacts/benchmarks/2026-07-31-triangle-pair-baseline
  ```

- [x] Add the smallest red public regressions before implementation. The first
  test must prove coplanar separation needs an in-plane axis; a second must
  prove a nonzero axis cannot be skipped by a narrowed epsilon check. Use these
  exact fixtures so the regressions remain reviewable rather than depending on
  a runtime search:

  - coplanar separated: first `[(0,0,0), (4,0,0), (0,4,0)]`, second
    `[(3,3,0), (5,3,0), (3,5,0)]`; their AABBs overlap but their `x+y`
    intervals do not;
  - coplanar touching: the same first triangle and second
    `[(2,2,0), (4,2,0), (2,4,0)]`, with exact zero depth;
  - tiny nonzero in-plane axes: with `m = Fixed64.MinIncrement`, first
    `[(-m,0,0), (m,0,0), (0,MaxValue,0)]` and second
    `[(-m,-m,0), (m,-m,0), (0,MinValue,0)]`.

  Run each separating fixture in both orders. Keep the direct contract shape
  visible:

  ```csharp
  bool hit = first.TryGetContact(
      firstOrigin,
      firstRotation,
      secondOrigin,
      secondRotation,
      second,
      out FixedContactAnchors contact);
  ```

- [x] Run the new tests and record the expected compile failure because
  `FixedTriangle.TryGetContact(...)` does not yet exist. Do not make production
  code compile until that failure has been observed.

**Phase 0 evidence:** FixedMathSharp's focused triangle/hull baseline passed
166/166 and Gravitas's focused mesh baseline passed 119/119. Five existing
mesh/mesh benchmark rows were captured before adoption; reported allocation
columns ranged from 0 B to 288 B and ShortRun medians ranged from 5.173 ms for
the ordinary row to 737.544 ms for the closed dense row. The two literal public
regressions produced exactly five expected `CS1061` errors for the absent
contract and no fixture/type errors. An independent test review found no
Critical, Important, or Minor issue.

### Phase 1: FixedMathSharp Exact Triangle-Pair Contract

**Goal:** Deliver and independently verify the complete reusable public
geometry relation before Gravitas adopts it.

#### Task 1.1: Centralize The Existing Rigid SAT Primitives

**Interfaces**

- Consumes: `Signed192`, `Signed320`, `Signed576`, `WideArithmetic`,
  `WideRationalBasis3d`, and the existing normalized-depth helpers.
- Produces:

  ```csharp
  internal readonly struct WideAxis3
  {
      internal WideAxis3(Signed320 x, Signed320 y, Signed320 z);
      internal bool IsZero { get; }
      internal Signed576 SquaredLength { get; }
      internal static WideAxis3 Cross(WideAxis3 left, WideAxis3 right);
      public static WideAxis3 operator -(WideAxis3 value);
  }

  internal readonly struct WidePointSpanPenetration
  {
      internal WidePointSpanPenetration(
          WideAxis3 axis,
          bool negate,
          Signed576 overlap,
          Signed576 squaredAxisLength,
          Signed320 commonDenominator);
      internal WideAxis3 Axis { get; }
      internal bool Negate { get; }
      internal bool HasValue { get; }
      internal bool ShouldReplace(
          Signed576 overlap,
          Signed576 squaredAxisLength,
          Signed320 commonDenominator);
      internal Fixed64 GetRoundedDepth(out bool isClamped);
  }

  internal static class WideRigidProjection
  {
      internal static WideAxis3 TransformLocalAxis(
          WideRationalBasis3d basis,
          Signed192 x,
          Signed192 y,
          Signed192 z);
      internal static WideAxis3 TransformLocalAxis(
          WideRationalBasis3d basis,
          Signed320 x,
          Signed320 y,
          Signed320 z);
      internal static Signed576 GetTransformedOffsetProjection(
          Vector3d offset,
          WideRationalBasis3d basis,
          WideAxis3 axis);
      internal static Signed576 GetDifferenceProjection(
          Vector3d end,
          Vector3d start,
          WideAxis3 axis);
      internal static Signed576 GetBasisProjection(
          WideAxis3 axis,
          Signed192 basisX,
          Signed192 basisY,
          Signed192 basisZ);
      internal static void IncludeProjection(
          Signed576 candidate,
          ref Signed576 minimum,
          ref Signed576 maximum);
  }
  ```

- [x] Move the existing nested `WideAxis3` unchanged into its common owner,
  then move cross and squared-length arithmetic into that value. Retain the
  current proof that transformed rigid edges fit Signed320 after cross-product
  narrowing.
- [x] Move `PointSpanPenetration` into `WidePointSpanPenetration`. Its
  `ShouldReplace(...)` must call
  `WideArithmetic.CompareNonNegativeNormalizedDepths(...) < 0`, so equality
  retains the earlier candidate. Its depth method must call
  `WideArithmetic.GetRoundedNonNegativeNormalizedDepth(...)` exactly once.
- [x] Move rigid-axis transformation and projection helpers into
  `WideRigidProjection`. The Signed320 local-axis overload must multiply into
  Signed576 and narrow only after the documented triangle normal-cross-edge
  width proof.
- [x] Extend `GetRoundedNonNegativeNormalizedDepth(...)` for axes below the
  existing half-Q32 magnitude proof. Keep the current approximation path when
  `squaredAxisLength >= (2^31)^2`; otherwise binary-search the representable raw
  domain using the existing exact midpoint predicate. At an exact lower
  midpoint, admit the candidate only when its raw value is even:

  ```csharp
  bool roundsAtLeastCandidate = comparison > 0
      || (comparison == 0 && (candidate & 1L) == 0L);
  ```

- [x] Add BigInteger-oracle cases at axis magnitudes `1`, `2^31 - 1`, and
  `2^31`, including lower-midpoint ties, `Fixed64.MaxValue`, and conceptual
  clamping. Prove both fast and fallback paths allocate zero warmed bytes.
- [x] Update existing oriented-box, triangle, and convex-hull consumers to call
  these owners explicitly. Remove the old nested types, `Cross`,
  `GetSquaredLength`, `TransformLocalAxis`,
  `GetTransformedOffsetProjection`, `GetDifferenceProjection`, and point-span
  ranking/materialization helpers. Replace the old projection-interval helper
  with `WideRigidProjection.IncludeProjection(...)`; do not retain forwarding
  wrappers.
- [x] Move the self-contained rigid triangle closest-point implementation from
  `WideOrientedBox` into a `WideTriangleRelations` partial. Preserve every
  Voronoi predicate and tie rule, update the existing public
  `GetClosestPointAnchor(...)` call, and run
  `FixedTriangleRelativeClosestPointTests` before adding pair behavior.
- [x] Run existing FixedMathSharp relation tests after the mechanical move:

  ```powershell
  dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj `
      --configuration Release `
      --filter "FullyQualifiedName~FixedOrientedBox|FullyQualifiedName~FixedConvexHullRelations|FullyQualifiedName~FixedConvexPrismRelations"
  ```

  Expected: all existing tests pass without changed public behavior.

**Task 1.1 evidence:** The tiny-axis regression failed under the prior one-step
approximation (`6521908913688700035` raw versus the exact
`6521908912666391105`) and passes through the new rare binary-search path.
Rounding passed 7/7 with warmed fast and tiny paths at 0 B, rigid triangle
closest-point passed 13/13, affected relation suites passed 208/208, the full
FixedMathSharp suite passed 2633/2633, and the dual-target Release build emitted
zero warnings. The Signed320 transform was intentionally narrowed from the
planned generic overload to the explicit
`TransformLocalTriangleNormalCrossEdgeAxis(...)` contract. Independent review
found no Critical, Important, or Minor issue.

#### Task 1.2: Implement The Public Triangle-Pair Relation Test-First

**Interfaces**

- Consumes: Task 1.1 common owners and the approved public signature.
- Produces: `FixedTriangle.TryGetContact(...)` and
  `WideTriangleRelations.TryGetContact(...)` returning one
  `FixedContactAnchors`.

- [x] Add argument tests proving each non-normalized rotation throws
  `ArgumentException` with the matching parameter name.
- [x] Add exact-zero-degenerate tests proving a triangle with a mathematically
  zero normal returns `false`, while tiny nonzero candidate axes remain
  classifiable.
- [x] Add ordinary, coplanar-overlap, coplanar-separation, touching, and
  reversed-order tests. Assert reversed normal inversion only for a nonzero
  exact centroid projection; test an exact-zero centroid projection separately
  for stable generated-axis ownership. The touching contract must be explicit:

  ```csharp
  Assert.True(first.TryGetContact(
      Vector3d.Zero,
      FixedQuaternion.Identity,
      Vector3d.Zero,
      FixedQuaternion.Identity,
      first,
      out FixedContactAnchors contact));
  Assert.Equal(Fixed64.Zero, contact.Depth);
  Assert.False(contact.DepthIsClamped);
  ```

- [x] Add mirrored scalar-face and translated-equivalence tests. Use
  `FixedPointAnchor.TryGetOffsetFrom(...)` to compare semantic anchors without
  requiring conceptual world points to materialize.
- [x] Add a full-domain edge-cross separating fixture using
  `s = Fixed64.MaxValue / 3`, first local vertices
  `[(1,-2,2), (0,3,1), (-2,2,1)] * s`, and second local vertices
  `[(2,2,-2), (-2,0,-3), (-2,-1,2)] * s`. The exact audit found separation
  only on edge-cross axes; assert `false` in both orders.
- [x] Retain exact large/tiny normalized-depth ordering and half-even midpoint
  parity at the shared arithmetic owner. At the public reducer, cover tiny and
  large-axis classification, an earlier-axis tie, and a conceptual depth beyond
  `Fixed64.MaxValue` that sets `DepthIsClamped`; do not duplicate the arithmetic
  oracle or runtime search through a direct forwarding path.
- [x] Use coincident coplanar triangles with reverse winding for the exact
  centroid-zero tie: expect the first triangle's authored face normal and zero
  depth. Use the alternating cube-face fixture at `Fixed64.MaxValue` for clamp
  coverage: first `[(-1,1,-1), (-1,-1,1), (1,1,1)] * MaxValue`, second
  `[(-1,1,1), (1,-1,1), (1,1,-1)] * MaxValue`; its minimum depth is
  `2 * MaxValue / sqrt(3)` and must clamp.
- [x] Implement `FixedTriangle.PairContacts.cs` as validation plus one direct
  call to `WideTriangleRelations`; do not add a result wrapper or overload.
- [x] Implement exact normals and edges from local vertices. Generate
  normal-cross-edge axes in the local triangle frame before rigid
  transformation; crossing the already transformed wide normal and edge can
  exceed Signed320 and is forbidden.
- [x] Generate cross-edge axes from transformed exact edges. Preserve the
  locked axis order and return immediately on the first negative overlap.
- [x] Project both triangles' three local vertices and origins into one common
  rational denominator. Choose the smaller nonnegative interval push, rank
  through `WidePointSpanPenetration`, and never materialize a candidate depth.
- [x] Orient the retained axis from the exact difference between the two
  projected vertex sums. On exact zero retain the generated axis sign.
- [x] Materialize one normal with `WideNormalization`, one depth through the
  retained penetration, and rigid-frame witnesses through the internal
  `WideTriangleRelations` closest-point core after the public method's one-time
  validation. Do not repeat public validation or reconstruct a point by
  subtracting `normal * depth`.
- [x] Run the focused public suite:

  ```powershell
  dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj `
      --configuration Release `
      --filter "FullyQualifiedName~FixedTrianglePairContactsTests"
  ```

  Expected: all new public tests pass.

**Task 1.2 evidence:** The restored public tests reproduced exactly five
missing-contract `CS1061` errors before production edits. The completed contact
suite passed 10/10, related exact geometry passed 201/201, anchor regressions
passed 27/27, the dual-target Release build was warning-free, and 64 warmed
public calls allocated 0 B. Independent review found one Minor gap in the
rigid-frame test; a nonidentity rotation and transformed-normal assertion were
added, the focused suite remained 10/10, and scoped re-review marked the
finding addressed with no new issue.

#### Task 1.3: FixedMathSharp Phase Gate

- [x] Add ordinary and tiny-axis triangle-pair rows to
  `OrientedBoxAnchorBenchmarks` and assert warmed zero allocation in the focused
  test suite. The ordinary row must retain the constant-time depth path; the
  tiny row measures the rare exact fallback explicitly.
- [x] Run the focused benchmark:

  ```powershell
  dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj `
      --configuration Release

  dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll `
      oriented-box-anchor --filter "*TrianglePair*" -j Short -i `
      --exporters json `
      --artifacts artifacts/benchmarks/2026-07-31-triangle-pair-fixedmath
  ```

- [x] Update public XML, README geometry inventory, v6-to-v7 migration guidance,
  the geometry wiki, and the evergreen complexity ledger with rigid-frame
  arguments, zero-depth touching, clamped depth, anchor ownership, and any
  justified wide-reducer complexity exception.
- [x] Run FixedMathSharp Release, ReleaseLean, package builds, and authoritative
  coverage:

  ```powershell
  dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj `
      --configuration Release
  dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj `
      --configuration ReleaseLean
  dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj `
      --configuration Release --collect:"XPlat Code Coverage" `
      --settings tests/FixedMathSharp.Tests/coverlet.runsettings `
      --results-directory TestResults/triangle-pair-fixedmath-coverage
  ```

- [x] Generate ReportGenerator output from the single new Cobertura artifact
  and prove 100% reachable line, branch, and method coverage. Delete zombie
  branches or consolidate meaningful behavior; do not add API-shape tests.
- [x] Obtain independent arithmetic-width, public-API, test-quality, allocation,
  documentation, and whole-phase reviews. Resolve every actionable finding.
- [x] Update this phase with exact test counts, coverage numerators, benchmark
  medians, allocation results, and review outcome.

**Task 1.3 evidence:** after the first independent review fix,
`TrianglePairPrimary` measured a refreshed 90.821 us median and
`TrianglePairTinyAxisFallback` measured a refreshed 9.567 us median under the
required Short in-process job; neither reported per-operation managed
allocation. These ShortRun values are signals, not a performance-gain claim.
After the first independent
whole-phase review fix, the final Release coverage run passed 2,646/2,646 and
the final ReleaseLean run passed 2,625/2,625. All four
direct core target/configuration builds and both 7.0.0 package builds completed
with zero warnings; standard and Lean `.nupkg`/`.snupkg` IDs and contents were
validated without publishing. The sole fresh Cobertura artifact reported
47,061/47,061 lines, 8,694/8,694 branches, and 3,419/3,419 methods/full methods.
The CRAP audit analyzed 3,415 stable method identities, found the same ten fully
covered >30 complexity floors, renamed the moved closest-point owner, and added
the two measured complexity-20 exceptions. The review fix replaced truncated
Signed832 normalized-depth products with complete 25-word stack products and
added a direct BigInteger-oracle regression; its warmed near-domain path also
measured 0 B. Read-only self-review found no actionable Task 3 issue. One
transient earlier full-Lean run reported two unrelated allocation deltas; both
tests passed in isolation and the next full run was clean. Independent
arithmetic review found the truncated Signed832 product, then confirmed the
25-word replacement addressed it with no remaining Critical or Important
finding. Independent API/test/performance/documentation review approved the
phase with no code or evidence finding.

**Review checkpoint:** Stop with all FixedMathSharp changes unstaged and
uncommitted for user review before modifying Gravitas production code.

**Recommended commit message:**
`feat: add full-domain rigid triangle contact`

### Phase 2: Gravitas Mesh-Pair Adoption And Zombie Deletion

**Goal:** Make the public FixedMathSharp result the sole mesh triangle-pair
contact authority while retaining current BVH complexity and stable ordering.

**Interfaces**

- Consumes: `FixedTriangle.TryGetContact(...)` from Phase 1.
- Produces: unchanged public Gravitas APIs and a smaller internal mesh contact
  path carrying exact anchors, normal, depth, and clamp state.

- [x] Reuse the exhaustive Phase 1 scalar-face, long-triangle, coplanar,
  tiny-axis, and tie-order relation matrix, then add Gravitas boundary
  regressions for full-domain separation, reversed dispatch, exact anchors,
  conservative extreme-frame BVH admission, clamped-depth propagation, and
  warmed allocation. Do not duplicate FixedMathSharp's relation suite.
- [x] Replace the redundant existing
  `MeshMesh_ShouldPreserveTriangleContact` wrapper assertion with a meaningful
  exact-relation/local-anchor regression; do not retain both tests.
- [x] Add one regression proving returned `ContactAnchor` values remain in the
  two canonical mesh frames even when neither conceptual world point can be
  represented.
- [x] In `TryBuildMeshMeshManifold(...)`, canonicalize relation direction by
  stable collider ID, use exact conservative relative triangle bounds solely
  to query the second mesh's local BVH, and load each candidate's canonical
  local `FixedTriangle` before calling:

  ```csharp
  if (!firstTriangle.TryGetContact(
          meshA.Mesh.Origin,
          meshA.Mesh.Rotation,
          meshB.Mesh.Origin,
          meshB.Mesh.Rotation,
          secondTriangle,
          out FixedContactAnchors contact))
  {
      continue;
  }
  ```

- [x] Add the returned anchors, depth, normal, and `DepthIsClamped` directly to
  the manifold. Delete the scalar SAT, projection, magnitude, centroid
  orientation, closest-point reconstruction, and synthetic
  `pointA - normal * depth` fallback.
- [x] Replace `GetTriangleInFrame(...)` with a conservative relative-bounds
  helper. Once no
  production caller needs cached triangle normals, centers, edges, or vertices,
  delete `CollisionTriangle` and its wrapper-only tests.
- [x] Run focused mesh collision and allocation tests:

  ```powershell
  dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj `
      --configuration Release `
      --filter "FullyQualifiedName~MeshTrianglePairContactTests|FullyQualifiedName~CollisionDetectionShapePairTests|FullyQualifiedName~ConcaveMeshCollisionTests"
  ```

- [x] Rerun the unchanged `*MeshMesh*` benchmark rows into
  `artifacts/benchmarks/2026-07-31-triangle-pair-gravitas-after`. Compare each
  median and allocation column against Phase 0; investigate a material
  regression rather than accepting wide arithmetic cost without evidence.
- [x] Restore Gravitas 100% reachable coverage for the adopted/deleted path and
  obtain independent correctness, determinism, performance, and zombie-code
  reviews.
- [x] Update this phase with exact evidence and stop for user review.

**Phase 2 evidence:** Gravitas now retains its two-level BVH traversal but uses
`FixedTriangle.TryGetContact(...)` as the sole concave triangle-pair contact
authority. Stable collider-ID canonicalization makes reversed dispatch retain
the same contact identities while swapping anchors and negating normals.
`FixedBoundBox.FromRelativeRotatedBoundsClippedToDomain(...)` replaced ignored
per-vertex reframe failures, so unrepresentable relative frames cannot shrink a
candidate query before the exact relation runs. The scalar SAT/witness helpers,
`CollisionTriangle`, and wrapper-only tests were deleted. One meaningful
convex-cube regression retains coverage of the separate positive convex-hull
forwarding path without restoring the hollow assertion.

The first exact benchmark pass exposed a material per-candidate regression.
FixedMathSharp now reuses each triangle's three basis-axis projections per axis
and cancels identical positive denominators during exact depth ranking. Those
policy-neutral deletions preserved bit results and warmed `0 B` behavior while
improving the direct triangle-pair mean from `90.907 us` to `65.974 us`. Across
the unchanged 64-pair Gravitas rows, final means were `4.839 ms` ordinary,
`70.553 ms` concave, `400.501 ms` dense, `564.147 ms` contact-heavy, and
`2,532.822 ms` closed dense. The exact dense rows recovered roughly `28-30%`
from the initial exact implementation but remain about `3.4-4.4x` above the
deleted scalar baseline. That measured optimization signal is retained in
[`benchmark-signal-hardening-backlog.md`](benchmark-signal-hardening-backlog.md);
no narrowed prefilter or second collision authority was added.

Fresh FixedMathSharp validation passed `2,648/2,648` Release tests and
`2,627/2,627` ReleaseLean tests. Its single Cobertura artifact reports
`47,095/47,095` lines, `8,698/8,698` branches, and `3,419/3,419` methods/full
methods. Fresh Gravitas validation passed `3,923/3,923` Release tests and
`3,868/3,868` ReleaseLean tests. Its single final Cobertura artifact reports
`55,848/55,848` lines, `15,833/15,833` branches, and `5,321/5,321`
methods/full methods. Both libraries built Release and ReleaseLean for
`netstandard2.1` and `net8.0` with zero warnings. Independent correctness,
determinism, performance, allocation, test-quality, and zombie-code reviews
approved the final phase with no actionable finding; one benchmark wording
correction was applied. All production changes remain unstaged and
uncommitted, and local project links remain preserved.

**Review checkpoint:** Leave all Gravitas changes unstaged/uncommitted after
Phase 2.

**Recommended commit message:**
`fix: adopt exact mesh triangle contact`

### Phase 3: Convex-Mesh Query And Mixed Parity Audit

**Goal:** Prove whether any other triangle consumer shares this exact root cause
without broadening the fix to unrelated geometry.

- [x] Audit `ConvexSweepQueryWorker`, `SweptSphereQueryWorker`, mixed circle-
  against-3D reducers, and mixed mesh finite-prism collision for unnormalized
  triangle-axis projection followed by narrowed interval overlap or normalized-
  depth ranking.
- [x] Confirm reducers already delegating to `FixedTriangle` finite-axis,
  projected-circle, convex-hull, or finite-slab relations need no change.
- [x] If a caller performs the same triangle-pair SAT, adopt
  `FixedTriangle.TryGetContact(...)` and add one family-level regression.
- [x] If the audit finds a different arithmetic root cause, add one ordered
  issue-tracker entry with RCA and evidence; do not hide it inside this plan or
  implement an unrelated solver.
- [x] Record the audited files and outcome in this phase, run any affected
  query/mixed suites, and obtain an independent parity review.

**Phase 3 evidence:** No same-root caller exists. `ConvexSweepQueryWorker`
uses support-mapped conservative advancement/GJK with `FixedTriangle` exact
closest-point and projected-barycentric predicates; it does not run
triangle-pair SAT. `SweptSphereQueryWorker` uses a local normalized-plane face
solve followed by exact `FixedTriangle.ContainsProjection`, plus exact
finite-cylinder-edge and sphere-vertex relations. Mixed
circle-against-3D mesh sweeps delegate each candidate to
`FixedTriangle.TryGetFiniteSlabProjectedCircleSweep`, while mixed mesh
circle/capsule slabs delegate to `FixedTriangle` finite-slab relations and
mixed mesh polygon/AABB prisms delegate to
`FixedConvexPrismRelations.TryGetTriangleContact`. The triangle/prism SAT
already retains wide projection numerators and exact squared-axis-depth
ranking. No path needs `FixedTriangle.TryGetContact(...)` or a competing
answer path. See the two Phase 3 audit reports under
`.superpowers/sdd/2026-07-31-full-domain-triangle-pair-contact-plan/`.

**Validation:** the focused Phase 3 parity slice passed 711/711 under both
Release and ReleaseLean. Independent parity review confirmed the same-root
result. The audit also proved a separate non-uniform-scale mesh-query
face-normal defect, now ordered as queue item 2 in `issue-tracker.md`.

**Review checkpoint:** Stop for user review. Phase 3 added a new
release-blocking issue; queue item 1 remains the current plan and still owns
Phase 4 closure.

### Phase 4: Coverage, Performance, Documentation, And Queue Closure

- [ ] Review the complete plan against the locked design and remove any stale
  helper, wrapper-only test, duplicated projection logic, or uncovered branch
  introduced by the work.
- [ ] Run full Release and ReleaseLean tests in both repositories.
- [ ] Run standard and Lean package builds for `net8.0` and
  `netstandard2.1` with zero warnings.
- [ ] Collect authoritative Cobertura artifacts independently in each repo and
  generate ReportGenerator summaries from only those artifacts. Require 100%
  reachable line, branch, and method coverage.
- [ ] Run all warmed Gravitas allocation guards plus the FixedMathSharp
  triangle-pair allocation assertion.
- [ ] Rerun focused FixedMathSharp and Gravitas benchmark rows; preserve zero
  allocation and document before/after medians plus any intentional complexity
  tradeoff.
- [ ] Update FixedMathSharp and Gravitas XML/wiki/package guidance, resolve the
  issue into historical context, remove it from the ordered queue, and move
  this plan to `docs/feature-work/done`.
- [ ] Dispatch independent reviewers for arithmetic widths and rounding,
  geometry/contact semantics, deterministic order, performance/allocations,
  test quality/coverage, documentation accuracy, and whole-change code quality.
  Resolve every Critical, Important, or Minor finding.
- [ ] Record exact test counts, coverage numerators, package results, benchmark
  results, allocation counts, and reviewer outcomes before declaring the final
  ordered queue closed.

**Recommended commit messages:**

- FixedMathSharp: `feat: add full-domain rigid triangle contact`
- Gravitas: `fix: adopt exact mesh triangle contact`
- Gravitas closure docs, if kept separate:
  `docs: close exact triangle contact hardening`
