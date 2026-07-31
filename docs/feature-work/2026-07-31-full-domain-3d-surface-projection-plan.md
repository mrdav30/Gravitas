# Full-Domain 3D Surface Projection Implementation

**Status:** Active  
**Started:** 2026-07-31  
**Repositories:** FixedMathSharp, Gravitas  
**Queue item:** `3D Closest-Surface And Overlap-Circle Classification Are Not Full-Domain`

## Goal

Make 3D closest-surface ownership and the public X/Z overlap-circle family
correct across the complete authored fixed-point geometry domain without
saturating intermediate coordinates, inventing points, or silently dropping
valid hits.

The work must preserve:

- deterministic authored-order ties;
- allocation-free warmed query paths;
- 100% reachable line, branch, and method coverage in both repositories;
- FixedMathSharp ownership of policy-neutral exact geometry;
- Gravitas ownership of collider dispatch, query semantics, and hit policy.

## Locked Contract

- `OverlapCircle`, `OverlapCircleInDirection`, `OverlapCircleAll`, and their
  batch variants classify the complete X/Z projection of supported 3D collider
  geometry.
- Circle containment is overlap at distance zero, matching normal overlap-query
  semantics and the pure-2D contract.
- Query Y does not affect X/Z classification, distance, direction, ordering, or
  admission.
- A hit retains a deterministic real collider-surface `ContactAnchor`.
  `Physics3DHit.TryGetPoint(...)` remains the explicit materialization boundary;
  no additional public result state is required.
- Exact classification and semantic anchor ownership precede optional
  `Vector3d` materialization.
- Equal compound-part and mesh-triangle candidates retain earlier authored
  order.

## Historical Root Cause

The existing path couples three different concerns:

1. closest-surface feature selection;
2. representable full-3D distance calculation;
3. final point materialization.

Finite-axis helpers can construct a valid `FixedPointAnchor` before failing
only because the full 3D distance cannot fit in `Fixed64`. Sphere and compound
paths additionally use saturating vector arithmetic. The 3D overlap-circle
worker then reuses that full-3D result even though its documented domain is
X/Z.

`Physics3DHit` has since gained semantic `ContactAnchor` ownership, so the old
tracker concern about needing a richer public result is obsolete.

## Battle Plan

### Phase 0: Contract And Baseline

- [x] Trace closest-surface and overlap-circle callers.
- [x] Confirm `Physics3DHit` already owns semantic anchors.
- [x] Resolve containment semantics as zero-distance projected overlap.
- [x] Confirm the existing cuboid-center regression encodes obsolete
  surface-proximity behavior and must change.

### Phase 1: Semantic 3D Surface Ownership

- [x] Separate finite-axis anchor/normal selection from optional scalar signed
  distance materialization in FixedMathSharp.
- [x] Give every built-in 3D collider one internal semantic closest-surface
  anchor path without adding a second public collider API.
- [x] Replace sphere saturation and compound materialized-point ranking.
- [x] Rank compound parts with exact `FixedPointAnchor` distance comparison and
  retain earlier authored order on ties.
- [x] Route public `ClosestPointOnSurface(...)` and normal selection through the
  shared semantic owner, materializing only the requested final result.
- [x] Add focused scalar-face, representable-result, unrepresentable-result,
  and compound-tie regressions.
- [x] Preserve FixedMathSharp and Gravitas 100% coverage.

**Review checkpoint:** Stop after the closest-surface contract is independently
green before changing overlap classification.

**Outcome:** Every built-in 3D collider now selects one semantic surface anchor
before optional point or signed-distance materialization. Finite-axis helpers
retain valid anchors when an unused scalar distance is unrepresentable, sphere
selection no longer depends on saturating subtraction, and compound colliders
rank child anchors exactly with authored-order ties.

**Verification:**

- FixedMathSharp `Release`: 2,604 passed; 100% line (44,443/44,443), branch
  (8,396/8,396), and method (3,329/3,329) coverage.
- FixedMathSharp `ReleaseLean`: 2,583 passed.
- Gravitas `Release`: 3,874 passed; 100% line (43,717/43,717), branch
  (12,777/12,777), and method (4,515/4,515) coverage.
- Gravitas `ReleaseLean`: 3,819 passed after a configuration-aware restore.
  Reusing normal-build NuGet assets with `--no-restore` is not a valid Lean
  gate because it retains the real MemoryPack package beside the Lean shim.

### Phase 2: Exact X/Z Projection Reducers

- [x] Reuse existing exact anchors, convex support, finite-slab projection,
  triangle projection, and oriented-box owners where their contracts fit.
- [x] Add only the missing policy-neutral projection math to FixedMathSharp.
- [x] Classify sphere, capsule, cylinder, cone, cuboid, mesh triangle, and
  compound projections without materializing conceptual world coordinates.
- [x] Return zero distance for containment and exact planar separation
  otherwise.
- [x] Retain a deterministic real surface anchor and outward normal independently
  from classification materialization.
- [x] Keep all reducers stack-only and allocation-free.

**Review checkpoint:** Stop after primitive and mesh/compound projection
reducers are green in isolation.

**Outcome:** FixedMathSharp now owns one exact planar-relation surface for
spheres, centered capsules, finite cylinders, finite cones, oriented boxes, and
triangles. Rational polygon reduction avoids conceptual world-corner
materialization; tilted disks and cone hulls use exact admission and
round-half-to-even distance certification while deriving offsets from the same
closest projected feature. Gravitas adds one collider dispatcher that composes
those reducers with the existing semantic closest-surface anchor owner for all
built-in primitives, compounds, and mesh triangles. Equal representable
compound-part and mesh-triangle distances retain earlier authored order.

The full-domain regressions include containment, exact elliptical-cap
tangency, mirrored stereographic fallback, subraw gaps, huge off-principal
gaps that force the exact fallback, feature-direction ownership, authored-order
ties, all-miss collections, and allocation-free warmed dispatch.

**Verification:**

- FixedMathSharp `Release`: 2,625 passed; 100% line (46,525/46,525), branch
  (8,638/8,638), and method (3,399/3,399) coverage.
- FixedMathSharp `ReleaseLean`: 2,604 passed.
- Gravitas `Release`: 3,881 passed; 100% line (43,857/43,857), branch
  (12,811/12,811), and method (4,520/4,520) coverage.
- Gravitas `ReleaseLean`: 3,826 passed after a configuration-aware restore.
- The warmed reducer allocation assertion reports zero bytes across every
  primitive plus mesh and compound dispatch.

### Phase 3: Query-Family Adoption

- [x] Replace the full-3D broad rejection with an exact or conservative X/Z
  rejection that cannot discard a valid projected hit.
- [x] Adopt the projection reducer in closest, directional, all-hit, and batch
  APIs through one shared worker.
- [x] Make directional filtering explicitly planar and robust for tiny
  representable directions.
- [x] Preserve distance-plus-collider-ID ordering and duplicate suppression.
- [x] Cover vertical invariance, containment, scalar faces, all supported
  shapes, ties, and single/batch parity.

**Review checkpoint:** Stop after the complete public query family is green
before benchmark and documentation closure.

**Outcome:** The closest, directional, all-hit, and batch X/Z circle APIs now
scan the covered X/Z columns across the active world's complete vertical
domain, retain GridForge partition traversal and duplicate suppression, and
delegate final admission to the Phase 2 planar reducer. Query Y no longer
changes candidate discovery or classification. Hits report planar distance and
direction while retaining an independent real 3D surface anchor and normal;
containment therefore reports zero distance without inventing a surface point.

Directional filtering ignores Y, compares the exact planar dot-product sign,
and compares the already-certified distance directly instead of squaring either
operand. The public integration coverage exercises every supported primitive,
mesh, and compound collider, deterministic collider-ID ties, vertical
invariance, tiny representable directions, and existing single/batch parity.
True 3D overlap-sphere admission remains on its separate surface-distance
worker so rotational CCD does not inherit planar semantics.

**Verification:**

- Gravitas focused circle, batch, and rotational-CCD regressions: 39 passed.
- Gravitas `Release`: 3,883 passed.
- Gravitas coverage: 100% line (43,879/43,879), branch
  (12,807/12,807), and method (4,522/4,522) coverage.
- Gravitas `ReleaseLean`: 3,828 passed after a configuration-aware restore.
- Independent review found no correctness issues and identified configured-Y
  traversal scaling as a Phase 4 benchmark gate.

### Phase 4: Closure

- [ ] Run focused and full Release/ReleaseLean tests in both repositories.
- [ ] Re-establish 100% reachable line, branch, and method coverage.
- [ ] Run warmed allocation assertions and X/Z query benchmarks, including
  vertical scaling across tall dense and sparse grids.
- [ ] Update query documentation and remove stale tracker language.
- [ ] Move this plan to `docs/feature-work/done`.

## Non-Goals

- No new public wide-arithmetic surface.
- No fake or clamped hit points.
- No general scene-geometry abstraction or speculative custom-collider API.
- No use of full-3D distance as a proxy for X/Z classification.
- No duplicate Gravitas implementation of FixedMathSharp wide arithmetic.
