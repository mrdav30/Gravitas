# Exact Radial Segment Distance Implementation

**Status:** Active  
**Started:** 2026-07-31  
**Repositories:** FixedMathSharp, Gravitas  
**Queue item:** `Radial Segment Parameters Can Collapse Spatially Distinct Query Hits`

## Goal

Preserve spatially distinct circle and sphere intersections across the complete
authored fixed-point segment domain. Radial query and CCD paths must solve and
rank physical distance directly instead of narrowing first to a Q32.32 ray
parameter or normalizing away a representable transverse component.

The work must preserve:

- deterministic entry/exit ordering and authored-order ties;
- allocation-free warmed query and CCD paths;
- 100% reachable line, branch, and method coverage in FixedMathSharp and
  Gravitas;
- FixedMathSharp ownership of policy-neutral exact geometry;
- Gravitas ownership of query admission, collider dispatch, CCD trajectory,
  and response policy.

## Locked Contract

- `FixedSegment2d` exposes exact circle intersection distance intervals.
- `FixedSegment` exposes exact sphere intersection distance intervals.
- Both APIs follow the established finite-axis distance contract: separate
  radius expansion, caller-supplied total distance, inclusive start
  containment, strict end containment, and one final round-to-even
  materialization.
- `FixedRay` and `FixedRay2d` retain their deliberate parameter-based APIs.
- Relative radial CCD uses a narrow internal direction-and-distance contract;
  it does not manufacture an endpoint that may be unrepresentable.
- Gravitas reconstructs authored source and target trajectories from exact
  physical distance before materializing normalized time.
- `RadialSweepAdmission` is removed after every production caller adopts the
  exact interval. Endpoint fallback is not retained because it cannot recover
  an interior crossing or lost root order.
- No public wide-arithmetic type, radial result wrapper, duplicate quadratic
  solver, or synthetic zero-length capsule is introduced.

## Historical Root Cause

The existing radial solver keeps its coefficients and discriminant wide, but
the public ray result narrows each root to a Q32.32 parameter. On a long
segment, distinct spatial hits can map to the same parameter. Some Gravitas
distance consumers instead normalize the authored displacement before solving;
a small but representable transverse component can then round to zero and
change admission itself. The later endpoint fallback only inspects the final
point, so it cannot repair either failure.

## Battle Plan

### Phase 0: Contract And Baseline

- [x] Trace the FixedMathSharp radial kernels and every Gravitas production
  consumer.
- [x] Confirm the exact one-final-rounding distance solver already exists
  behind finite-axis cap handling.
- [x] Reject a new public result type, a duplicate solver, and changes to the
  normalized-ray contract.
- [x] Lock the direct segment-distance and internal relative-motion design.

### Phase 1: FixedMathSharp Radial Distance Contract

- [x] Add focused failing regressions through the intended public APIs.
- [x] Expose direct circle and sphere distance intervals on the existing
  segment owners.
- [x] Reuse the existing exact radial distance kernel directly.
- [x] Cover radius expansion, start containment, strict end containment,
  misses, invalid distance domains, and final half-even materialization.
- [x] Cover million-unit hits one spatial raw unit apart and two-raw
  transverse interior tangency without duplicating the finite-axis suite.
- [x] Update geometry documentation and the existing finite-axis benchmark
  surface where needed.
- [x] Retain FixedMathSharp Release, ReleaseLean, zero-allocation, and 100%
  reachable coverage gates.

**Review checkpoint:** Stop after the reusable FixedMathSharp contract is green
and independently reviewed.

**Outcome:** `FixedSegment2d` and `FixedSegment` now expose direct circle and
sphere physical-distance intervals with separate radial expansion and exact
endpoint containment. Both dimensional wrappers call the existing wide radial
distance kernel directly; no zero-length capsule, duplicate solver, result
type, or ray-contract change was introduced. The focused regressions now prove
one-raw physical ordering and two-raw transverse tangency through the intended
public API while the separate collapsed-capsule regression retains the finite-
axis limit contract.

**Verification:**

- FixedMathSharp `Release`: 2,626 passed; `ReleaseLean`: 2,605 passed.
- Coverage remains 100% line (46,625/46,625), branch (8,644/8,644), and
  method (3,406/3,406).
- Standard and Lean package builds pass for `net8.0` and `netstandard2.1` with
  zero warnings.
- The direct 2D circle and 3D sphere benchmark rows at scales 1 and 100,000
  report zero managed allocation; the short in-process means range from
  4.010 us to 5.747 us.
- Independent review found no Critical or Important issue in arithmetic width,
  rounding, containment, API shape, test quality, documentation, or benchmark
  coverage.

### Phase 2: Gravitas Query Adoption

- [x] Migrate pure-2D circle raycasts and circle sweeps.
- [x] Migrate 3D sphere raycasts, swept-sphere/sphere tests, and mesh-vertex
  radial tests.
- [x] Migrate mixed circle/sphere and circular finite-slab reducers.
- [x] Reconstruct hit points with exact distance along the authored segment.
- [x] Remove every query dependency on `RadialSweepAdmission` and move its
  meaningful query regressions to the public query families.
- [x] Preserve distance-plus-collider-ID ordering, batch parity, and warmed
  zero allocation.

**Review checkpoint:** Stop after the complete public query family is green and
independently reviewed.

**Outcome:** Pure-2D circle raycasts and sweeps, 3D sphere raycasts and sphere
sweeps, mesh-vertex radial tests, mixed circle/sphere queries, and circular
finite-slab reducers now solve and reconstruct directly from physical distance
along the authored segment. Mixed finite-slab callers pass their authored end
point through the reducer, and the endpoint-only tolerance fallback was
removed. No query path depends on `RadialSweepAdmission`; its remaining 2D
overload serves only the relative-CCD caller scheduled for Phase 3, while its
now-dead 3D overload and copied tests were deleted.

**Verification:**

- Gravitas `Release`: 3,898 passed; `ReleaseLean`: 3,843 passed.
- Coverage remains 100% line (55,969/55,969), branch (15,851/15,851), and
  method (5,353/5,353) in the generated report.
- The focused warmed allocation gate passes all five pure-2D, 3D, and mixed
  query cases with zero managed allocation.
- The existing radial-raycast benchmark executes all 18 normal and
  100,000-scale cases successfully under its Dry validation job with zero
  managed allocation.
- Independent review found no Critical or Important issue in distance and
  containment semantics, reconstruction, ordering, allocation behavior, test
  quality, or plan accuracy.

### Phase 3: Relative CCD Adoption

- [ ] Add the narrow internal direction-and-distance radial contract required
  by Gravitas without exposing wide mechanics publicly.
- [ ] Migrate pure-2D, pure-3D, and mixed relative radial CCD.
- [ ] Reconstruct source and target impact positions from their authored
  trajectories before normalized-time materialization.
- [ ] Compare exact distance against the relative sweep boundary before any
  Q32.32 time conversion.
- [ ] Delete `RadialSweepAdmission` after its final relative-CCD caller is
  migrated.
- [ ] Cover reversed body order, opposite scalar faces, start overlap, strict
  end exclusion, one-raw root ordering, tiny transverse motion, and
  deterministic same-time arbitration.

**Review checkpoint:** Stop after all dimensional CCD paths are green and
independently reviewed.

### Phase 4: Coverage, Performance, Documentation, And Queue Closure

- [ ] Run focused and full Release/ReleaseLean tests in both repositories.
- [ ] Re-establish 100% reachable line, branch, and method coverage without
  hollow API-shape tests or zombie branches.
- [ ] Run the existing radial query and CCD benchmark/allocation gates and add
  only regressions needed to measure the changed hot paths.
- [ ] Update public XML and wiki guidance for physical-distance radial
  intervals and the retained parameter-ray contract.
- [ ] Move the issue to resolved history and this plan to `done` with final
  evidence.
- [ ] Obtain an independent whole-change review and resolve every important
  finding before closure.

## Non-Goals

- No change to general ray parameter semantics.
- No public exposure of FixedMathSharp wide arithmetic.
- No Gravitas copy of FixedMathSharp quadratic or limb arithmetic.
- No endpoint-only repair, normalized-direction workaround, or clamped hit
  position.
- No unrelated convex, mesh, or finite-axis redesign without a separately
  confirmed defect.
