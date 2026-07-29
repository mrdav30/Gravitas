# FixedMathSharp / Gravitas Ownership Boundary Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> `superpowers:test-driven-development` while changing behavior,
> `superpowers:requesting-code-review` at each review checkpoint, and
> `superpowers:verification-before-completion` before reporting a phase
> complete. Steps use checkbox (`- [ ]`) syntax for living progress tracking.

**Goal:** Make FixedMathSharp the focused source of reusable deterministic
mathematics and Gravitas the sole owner of rigid-body response policy, while
removing proven wide-arithmetic duplication and making both codebases easier to
navigate before their first coordinated public releases.

**Architecture:** FixedMathSharp retains standard-library-quality deterministic
math, geometry, and internal allocation-free wide arithmetic. Every recent
semantic type is re-evaluated rather than preserved by default; physics-specific
types and algorithms move into Gravitas, where they may directly compose
FixedMathSharp's internal `Signed*` and `WideArithmetic` primitives. Focused
internal owners replace reusable math currently hidden behind unrelated partial
classes, while one Gravitas-owned exact response subsystem contains the
intentional friend consumption and rigid-body policy.

**Tech Stack:** C# 11, Q32.32 `Fixed64`, FixedMathSharp fixed-width signed wide
arithmetic, Gravitas 2D/3D/mixed response and rotational CCD, xUnit v3,
BenchmarkDotNet, `Release` and `ReleaseLean` package variants.

## Global Constraints

- FixedMathSharp v6 is the released API comparison baseline. Types or methods
  introduced and removed entirely during unreleased v7 development are not
  v6-to-v7 breaking changes and must not be documented as if users could have
  consumed them from v6.
- Preserve the intended final v7 public semantic surface; do not preserve
  intermediate unreleased v7 response APIs with obsolete shims or forwarding
  layers.
- Determinism, performance, maintainability, and correctness are all release
  gates. No phase is complete by trading one away to satisfy another.
- FixedMathSharp remains engine agnostic and must not reference Gravitas.
- Keep `Signed192`, `Signed320`, `Signed576`, `Signed704`, `Signed832`,
  `WideArithmetic`, and other raw wide representations internal.
- `private` is not an architectural boundary. Promote a policy-neutral helper
  to `internal` when Gravitas genuinely needs it, but do not broaden unrelated
  helpers merely because assembly friendship is available.
- Add only `InternalsVisibleTo("Gravitas")`. Do not friend `Gravitas.Tests` or
  `Gravitas.Benchmarks`; they must exercise FixedMathSharp internals through a
  Gravitas-owned internal component.
- Retain FixedMathSharp's existing test and benchmark friendships because those
  projects directly verify its internal wide arithmetic.
- Keep every FixedMathSharp internal type out of Gravitas public and protected
  signatures, serializable layouts, public documentation, and host-facing APIs.
- Concentrate direct FixedMathSharp-internal consumption in the Gravitas exact
  response subsystem. Do not add an integration package, wrapper package,
  general wide façade, or public arbitrary-precision API.
- Retain a recent semantic type in FixedMathSharp only when it represents
  coherent standard math or geometry with value independent of Gravitas.
  Physics-specific types move into Gravitas even when they currently wrap
  FixedMathSharp wide arithmetic.
- Gravitas may implement physics-specific wide algorithms directly with
  FixedMathSharp's internal `Signed*`, `WideArithmetic`, and policy-neutral
  geometry helpers. Do not duplicate foundational limb arithmetic downstream.
- Move restitution, unilateral impulse accumulation, inverse-mass/inertia
  response operands, solver deltas, warm-start policy, and Coulomb friction
  policy into Gravitas.
- Centralize only arithmetic kernels whose width, sign, rounding, and failure
  contracts are proven identical. Do not genericize deliberately unrolled
  fixed-width arithmetic or unrelated root solvers merely because their code
  looks similar.
- Preserve the common compact response path and the allocation-free exact
  fallback. Do not route ordinary representable contacts through wide response
  math merely to simplify control flow.
- Preserve 100% reachable line, branch, and method coverage in FixedMathSharp
  and Gravitas without hollow API-shape tests, reflection-only absence tests,
  behavioral exclusions, or zombie branches.
- Treat roughly 1,200 lines as a hard review warning, not a target. A cohesive
  measured algorithm may exceed it only when splitting would obscure its
  invariant; unrelated responsibilities must not share a file to satisfy an
  arbitrary partial or file-count goal.
- Directory moves and partial consolidation are mechanical cleanup performed
  after ownership deletion and migration, not mixed into solver behavior
  changes.
- Retain temporary local project references throughout implementation. Leave
  them unstaged and uncommitted as validation scaffolding.
- Pause after each phase for owner review. Leave implementation and
  documentation changes unstaged and uncommitted and provide a recommended
  commit message.

---

## Decisions Locked By This Plan

1. No recent semantic v7 type is retained in FixedMathSharp merely because it
   already exists there. Phase 0 classifies `FixedPointAnchor*`,
   `FixedContactAnchors*`, `FixedLever*`, `FixedMassPoint*`, and
   `FixedMassWeight` by general mathematical value, independent production
   consumers, and public API coherence.
2. Policy-neutral operations remain in FixedMathSharp only when their owning
   type survives that classification. Otherwise Gravitas owns the semantic type
   and composes FixedMathSharp internal wide primitives directly.
3. The unreleased v7 response DTOs and solver entry points are removed from
   FixedMathSharp's final v7 surface:
   - `FixedLeverResponseOperand3d`
   - `FixedLeverNormalConstraint3d`
   - `FixedLeverNormalResponse3d`
   - `FixedLeverCoulombResponse3d`
   - `FixedLever.TryGetNormalResponse`
   - `FixedLever.TryGetAccumulatedNormalResponse`
   - `FixedLever.TryGetCoulombLineResponse`
   - `FixedLever.TryGetCoulombDiskResponse`
4. Removing those intermediate v7 APIs does not require a v6-to-v7 migration
   entry or an additional semantic-version marker. FixedMathSharp's migration
   guide documents the final v7 surface relative to v6.
5. `WideOrientedBox` must not remain the hidden owner of general point-anchor,
   lever, or rigid-body response arithmetic. `WideVector2dTransform` must not
   remain the hidden owner of 2D point-anchor response arithmetic.
6. FixedMathSharp will introduce only focused internal owners required by
   current behavior:
   - `WidePointAnchor3d`
   - `WidePointAnchor2d`
   - `WideLever3d`
   - `WideLever2d`
   - `WideRationalBasis3d` only if extracting the current shared rational basis
     is required to remove the unrelated partial dependency.
7. The exact class split may use non-partial types or cohesive partials based on
   file size, but no broad `WidePhysics`, generic expression framework, dynamic
   word collection, or one-interface/one-implementation abstraction is added.
8. Gravitas owns internal, separately navigable response values:
   - `ExactContactResponseOperand3D`
   - `ExactNormalConstraint3D`
   - `ExactNormalResponse3D`
   - `ExactCoulombResponse3D`
9. `ExactContactResponseKernel` is the sole Gravitas owner of the migrated
   normal and Coulomb wide-response algorithms. Existing
   `ExactContactLever3D` and `ExactContactLever2D` remain body/mobility adapters
   rather than becoming duplicate arithmetic owners.
10. The friendship declaration is exactly:

    ```csharp
    [assembly: InternalsVisibleTo("Gravitas")]
    ```

    FixedMathSharp and Gravitas currently retain the same assembly names across
    standard/Lean configurations and target frameworks, so one unsigned friend
    declaration covers the complete build matrix.
11. `InternalsVisibleTo` is assembly-wide, not selective or transitive. The
    boundary is enforced by code organization, review, tests, and documentation
    rather than a nonexistent type-level friendship mechanism.
12. FixedMathSharp and Gravitas are intentionally release-coupled at the
    internal ABI boundary. FixedMathSharp releases first; Gravitas must then
    rebuild and validate against that released package before its own release.
13. Folder organization does not change namespaces. Dimensional counterparts
    stay adjacent; the plan does not create complete parallel 2D/3D directory
    trees or folders for isolated one-file helpers.

## Audit Baseline

- FixedMathSharp currently contains approximately 89,083 physical source lines
  across 220 C# files.
- `Geometry/Wide` contains approximately 47,201 lines across 90 files, roughly
  53% of the source tree.
- `Geometry/Primitives` contains approximately 12,398 lines across 37 files.
- `Numerics/Wide` contains approximately 5,416 lines across 16 files.
- The largest partial owners are:
  - `WideOrientedBox`: 35 files / approximately 20,164 lines.
  - `WideFiniteAxisIntersection`: 25 files / approximately 12,129 lines.
  - `WideConvexPrismRelations`: 10 files / approximately 5,502 lines.
  - `FixedSegment`: 11 files / approximately 4,343 lines.
  - `WideArithmetic`: 8 files / approximately 3,757 lines.
- The concentration is primarily an ownership and navigation problem, not a
  reason to merge every partial into a larger file.
- Proven duplicate candidates include generic magnitude multiplication,
  magnitude bit-length calculation, magnitude word addition/copy/comparison,
  one local wide difference implementation, and duplicated radical-pair
  comparison within `WideConvexPrismRelations`.
- Similar-looking width-specific root solvers, fixed-width unrolled operations,
  and mass-property accumulators have distinct contracts and are not approved
  for automatic consolidation.

---

## Phase 0: Reproduce The Baseline And Freeze The Boundary

**Files:**

- Read: `docs/feature-work/feature-work-overview.md`
- Read: `docs/feature-work/done/2026-07-27-exact-contact-lever-response-plan.md`
- Read: `F:/gamedevrepos/FixedMathSharp/docs/complexity-exceptions.md`
- Read: `F:/gamedevrepos/FixedMathSharp/docs/MIGRATION.md`
- Modify: this plan

**Produces:**

- Fresh correctness, coverage, allocation, and benchmark baselines against the
  locally linked stack.
- A recorded inventory of every FixedMathSharp response API and every Gravitas
  caller that must migrate.

- [x] Complete the structural, duplication, public-surface, and assembly-
  boundary audit summarized above.
- [x] Confirm v6, rather than the intermediate develop worktree, is the public
  API comparison baseline for the upcoming FixedMathSharp v7 release.
- [x] Record clean/intentional worktree state in FixedMathSharp and Gravitas.
  Treat only the known Gravitas solution/project local-link edits as expected
  scaffolding before implementation starts.
- [x] Run the complete FixedMathSharp `Release` and `ReleaseLean` suites and
  record test totals.
- [x] Run the complete Gravitas `Release` and `ReleaseLean` suites and record
  test totals.
- [x] Generate fresh merged coverage reports for both libraries and confirm the
  starting point remains exactly 100% reachable line, branch, and method
  coverage.
- [x] Capture like-for-like short-run baselines for:
  - FixedMathSharp `PointAnchorBenchmarks`.
  - Gravitas `CollisionResponseBenchmarks`.
  - Gravitas `MixedCollisionResponseBenchmarks`.
- [x] Run the warmed Gravitas response allocation assertions and record their
  current zero-allocation results.
- [x] Record all current production callers of the four
  `FixedLever.TryGet*Response` entry points. The caller inventory must include
  pure 2D, pure 3D, mixed response, warm starts, friction, and rotational CCD.
- [x] Produce a keep/move ledger for `FixedPointAnchor*`,
  `FixedContactAnchors*`, `FixedLever*`, `FixedMassPoint*`, and
  `FixedMassWeight`. For each type, record:
  - independent FixedMathSharp production consumers;
  - whether the contract is recognizable standard math/geometry;
  - whether its terminology or invariants encode rigid-body physics policy;
  - the public API and documentation consequence of keeping or moving it.
- [ ] Revise later phase file lists and interfaces to follow the approved
  keep/move ledger rather than the audit's initial preservation assumption.
- [x] Pause for owner review before changing source ownership.

### Phase 0 Evidence

**Worktree state:**

- FixedMathSharp `develop` is clean and ahead of its remote by five owner
  commits.
- Gravitas `develop` contains only this plan/overview work plus the four known
  local-link solution/project edits. Generated coverage and benchmark artifacts
  remain ignored.

**Correctness and package baseline:**

- FixedMathSharp `Release`: 2,638 core tests plus 8 Chronicler tests pass.
- FixedMathSharp `ReleaseLean`: 2,617 core tests plus 8 Chronicler tests pass.
- Gravitas `Release`: 3,776 tests pass.
- Gravitas `ReleaseLean`: 3,721 tests pass.
- Standard and Lean builds resolve the complete local FixedMathSharp,
  SwiftCollections, and GridForge chain in the intended configuration.

**Coverage and CRAP baseline:**

- FixedMathSharp reports 53,473/53,473 lines, 8,806/8,806 branches, and
  3,506/3,506 fully covered methods across the runtime, Chronicler adapter, and
  FluentAssertions package. Ten methods score above CRAP 30; all are fully
  covered complexity floors, so there is no coverage-amplified hotspot.
- Gravitas's authoritative coverage-enabled `Release` run passes all 3,776
  tests and reports 40,072/40,072 lines, 12,365/12,365 branches, and
  4,368/4,368 fully covered methods. Twenty-six methods score above CRAP 30;
  all are fully covered complexity floors.
- A diagnostic Debug coverage run caused allocation assertions to observe
  Coverlet probe costs. It is not an authoritative correctness or allocation
  artifact. The repository's established coverage-enabled `Release` command
  passed every allocation assertion and produced the accepted report.

**Allocation and performance baseline:**

- Five focused warmed response assertions pass at zero managed allocation:
  3D normal impulse, 2D exact friction, mixed normal, mixed exact friction, and
  prepared 3D collision response.
- All 13 `PointAnchorBenchmarks` ShortRun rows report `0 B`. Representative
  means are 32.64 ns for compact cross projection, 724.42 ns for exact
  representable cross projection, 1.943 us to create a full-domain lever,
  870.25 ns for full-domain cross projection, 1.541 us for the quadratic form,
  1.748 us for transformed cross, and 891.96 ns for the 2D squared-cross path.
- All 24 prepared 3D response rows report `0 B`, with ShortRun means from
  0.65 ms through 4.89 ms across 16/64 pairs, six contact shapes, and
  default/distinct materials.
- All 8 mixed response rows report `0 B`. Single-pass means range from
  1.35-3.85 ms and bounded-island means from 3.78-6.34 ms across 16/64 pairs.
- These short rows are comparison baselines, not absolute throughput claims;
  BenchmarkDotNet warns that the mixed iteration times are below its preferred
  100 ms statistical floor.

**Response caller inventory:**

- Exactly eight direct Gravitas production calls reach the four public
  FixedMathSharp response entry points; no production consumer exists elsewhere
  in FixedMathSharp or the scanned LSF sibling sources.
- Normal response:
  - `Response/3D/ExactContactLever3D.cs`: normal and accumulated-normal.
  - `Response/2D/ExactContactLever2D.cs`: normal and accumulated-normal.
  - `Response/Mixed/ContactNormalImpulseMixed.cs`: normal and
    accumulated-normal.
- Coulomb response:
  - `Response/2D/CollisionResponse2D.cs`: line friction.
  - `Response/Mixed/CollisionResponseMixed.cs`: disk friction.
- Indirect migration coverage includes:
  - `CollisionHandling/Pairs/3D/CollisionPair.cs`
  - `Core/3D/GravitasPhysicsService.Response.cs`
  - `CollisionHandling/Pairs/2D/CollisionPair2D.cs`
  - `CollisionHandling/Pairs/2D/CollisionPair2D.ReplayHash.cs`
  - `Core/2D/GravitasPhysics2DService.Response.cs`
  - `Core/Mixed/GravitasMixedCollisionService.Response.cs`
  - `Core/3D/SolidBody.ContinuousCollision.Rotational.Response.cs`
  - `Core/2D/SolidBody2D.ContinuousCollision.Rotational.Response.cs`
  - `Core/3D/SolidBody.ContinuousCollision.Rotational.Mixed.cs`
  - `Core/2D/SolidBody2D.ContinuousCollision.Rotational.Mixed.cs`

**Recommended semantic ownership ledger:**

| Current v7 type | Recommendation | Rationale |
| --- | --- | --- |
| `FixedPointAnchor`, `FixedPointAnchor2d` | Keep public in FixedMathSharp | Exact transformed points and full-domain relative geometry are reusable computational geometry. FixedMathSharp oriented boxes, triangles, segments, and convex relations consume them independently of Gravitas. |
| `FixedContactAnchors`, `FixedContactAnchors2d` | Keep public in FixedMathSharp; remove constraint-policy wording | They are geometric shape-relation results produced throughout FixedMathSharp's public finite-shape relation APIs. Their data is only two anchors, a geometric normal, penetration depth, and depth representability; they carry no body, impulse, material, or solver state. |
| `FixedContactLocalPoints` | Keep public in FixedMathSharp; document as a compact geometric local-point pair | It is caller-owned output storage for reusable multi-feature shape relations. It contains only two local points and no physical-response policy. |
| `FixedLever` | Move to Gravitas; retain only the minimum internal anchor-offset representation/helpers in FixedMathSharp | No independent FixedMathSharp production consumer needs the public semantic type. Its only stack consumer is Gravitas response/CCD, and most of its current surface is point-velocity, effective-mass, impulse, and friction machinery. |
| `FixedLever2d` | Remove from the FixedMathSharp public surface; add a Gravitas-owned type only if migration proves it useful | No Gravitas production path consumes it directly; pure 2D currently embeds its exact response in the 3D representation. Do not migrate unused API for symmetry. |
| `FixedMassPoint`, `FixedMassPoint2d` | Move to Gravitas | Weighted centers, parallel-axis tensors/moments, and mass-point composition are rigid-body mass-property semantics. FixedMathSharp has no independent non-mass consumer. |
| `FixedMassWeight` | Move to Gravitas | Its only FixedMathSharp consumers are the recent triangle/polygon shell-mass APIs, while all real stack consumers are collider mass and inertia calculations. General weighted arithmetic already has a separate policy-neutral internal owner. |

- Moving the mass types also moves or removes the physics-specific public
  triangle/polygon mass-weight and surface-inertia APIs introduced during v7.
  FixedMathSharp retains ordinary geometric area, centroid, and relation math.
- `WideMassProperties` and `WideTriangleMassProperties` must be decomposed:
  foundational signed arithmetic stays in FixedMathSharp, while mass-property
  composition moves to Gravitas and directly consumes the friend-accessible
  `Signed*`/`WideArithmetic` primitives.
- The contact-relation types remain because moving them would either move a
  broad reusable exact geometry surface into Gravitas or force FixedMathSharp
  to depend upward. Their documentation must stop calling the result a physics
  constraint or manifold.
- All ledger types are absent from released FixedMathSharp v6. The final
  approved v7 surface needs no compatibility shim or intermediate-v7 migration
  narrative.

## Phase 1: Centralize Proven Wide-Arithmetic Duplication

**Files:**

- Create:
  `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Numerics/Wide/WideArithmetic.Magnitude.cs`
- Modify:
  `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Numerics/Wide/WideArithmetic.cs`
- Modify:
  `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Numerics/Wide/WideArithmetic.Signed704.cs`
- Modify:
  `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Geometry/Wide/WideConvexPrismRelations.ProjectionArithmetic.cs`
- Modify:
  `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Geometry/Wide/WideConvexPrismRelations.Projection.cs`
- Modify:
  `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Geometry/Wide/WideConvexPrismRelations.WideCandidate.Rounding.cs`
- Modify:
  `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Geometry/Wide/WideOrientedBox.RationalSegmentSweep.cs`
- Modify:
  `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Geometry/Wide/WideGeometry.cs`
- Modify only where the duplicated bit-length loop exists:
  `WideTriangleConeIntersection.cs`, `WideSlabProjection.cs`,
  `WideGeometry.Normalization.cs`,
  `WideFiniteAxisIntersection.NarrowSolver.cs`
- Test:
  `F:/gamedevrepos/FixedMathSharp/tests/FixedMathSharp.Tests/Numerics/Wide`
- Test: the matching convex-prism, slab, finite-axis, normalization, and
  triangle-cone behavior suites.

**Interfaces:**

- Produces internal allocation-free magnitude-span kernels for multiplication,
  zero-safe active length/bit length, word addition, copying, shifting, and
  equal-length comparison.
- Preserves every existing public result, midpoint decision, saturation rule,
  stable tie order, and failure path.

- [ ] Add focused internal tests for zero, one-word, full-carry, highest-word,
  unequal-active-length, and maximum-workspace magnitude operations. Compare
  multiplication and comparison results to `BigInteger` or the existing
  fixed-width oracle used by the current test area.
- [ ] Move the byte-for-byte equivalent generic `MultiplyMagnitudes` kernel
  from the width-specific `WideArithmetic.Signed704.cs` owner into
  `WideArithmetic.Magnitude.cs`.
- [ ] Replace the local magnitude multiplication and word-add copies in
  `WideConvexPrismRelations.ProjectionArithmetic.cs` and
  `WideOrientedBox.RationalSegmentSweep.cs` with direct `WideArithmetic`
  calls.
- [ ] Centralize zero-safe magnitude bit-length and replace the repeated loops
  in the six audited geometry/arithmetic locations.
- [ ] Route `WideGeometry` through `WideArithmetic.Difference` and delete its
  duplicate private difference implementation.
- [ ] Route `WideConvexPrismRelations` wide-candidate radical comparison
  through its existing equal-length radical-pair implementation and delete the
  duplicate local pair.
- [ ] Keep fixed-width unrolled add/subtract/multiply methods and
  contract-specific root solvers unchanged unless a focused test proves they
  are exactly the same operation and a benchmark shows no regression.
- [ ] Run the focused geometry and wide-arithmetic suites in `Release` and
  `ReleaseLean`.
- [ ] Re-run FixedMathSharp coverage and require 100% line, branch, and method
  coverage.
- [ ] Re-run affected point-anchor and finite-geometry benchmarks. Require zero
  managed allocation and no material regression attributable to generic span
  bounds checks, inlining, or expanded `stackalloc` lifetime.
- [ ] Request independent review focused on sign extension, carry propagation,
  zero handling, workspace bounds, stack pressure, and accidental
  genericization.
- [ ] Update this plan with results and pause for owner review.

## Phase 2: Extract Policy-Neutral Anchor And Lever Owners

**Files:**

- Create under:
  `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Geometry/Anchors/Wide`
  - `WidePointAnchor3d.cs`
  - `WidePointAnchor2d.cs`
  - `WideLever3d.cs`
  - `WideLever2d.cs`
  - `WideRationalBasis3d.cs` only when required by the extracted 3D operations.
- Modify:
  `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Geometry/Anchors/FixedPointAnchor.cs`
- Modify:
  `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Geometry/Anchors/FixedPointAnchor2d.cs`
- Modify:
  `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Geometry/Anchors/FixedLever.cs`
- Modify:
  `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Geometry/Anchors/FixedLever2d.cs`
- Remove general anchor/lever ownership from:
  - `Geometry/Wide/WideOrientedBox.PointAnchor.cs`
  - `Geometry/Wide/WideOrientedBox.PointAnchorTerm.cs`
  - `Geometry/Wide/WideOrientedBox.PointAnchorDistance.cs`
  - `Geometry/Wide/WideOrientedBox.PointAnchorResponse.cs`
  - `Numerics/Wide/WideVector2dTransform.PointAnchorResponse.cs`
  - the anchor/lever regions of `Numerics/Wide/WideVector2dTransform.cs`
- Test:
  `F:/gamedevrepos/FixedMathSharp/tests/FixedMathSharp.Tests/Geometry/Primitives/FixedPointAnchor.Tests.cs`
- Test:
  `F:/gamedevrepos/FixedMathSharp/tests/FixedMathSharp.Tests/Geometry/FixedPointAnchor2d.Tests.cs`
- Test: matching distance, lever, mass-property, finite-surface, and oriented-box
  suites.

**Interfaces:**

- `WidePointAnchor3d` and `WidePointAnchor2d` own anchor construction,
  validation, term reduction, distance ordering, and relative-ratio extraction.
- `WideLever3d` and `WideLever2d` own policy-neutral exact lever algebra.
- The few exact ratio primitives required by Gravitas are `internal`; public
  semantic methods continue to delegate to these owners.
- No solver-response DTO, restitution coefficient, impulse accumulator, or
  friction coefficient enters these FixedMathSharp owners.

- [ ] Add or identify focused tests that exercise every public semantic anchor
  and lever operation through ordinary, full-domain, mirrored, degenerate,
  uninitialized, final-overflow, and deterministic-tie cases.
- [ ] Move 3D point-anchor term construction, reduction, distance ordering, and
  relative-ratio operations out of `WideOrientedBox`.
- [ ] Move 2D point-anchor term and lever operations out of
  `WideVector2dTransform`.
- [ ] Move policy-neutral 3D lever materialization, relative point-velocity
  ratio, cross-product quadratic-form ratio, and transformed cross-product
  arithmetic into `WideLever3d`.
- [ ] Promote only the extracted operations Gravitas needs from `private` to
  `internal`. Keep implementation details private when they do not cross the
  assembly boundary.
- [ ] Extract `WideRationalBasis3d` from `WideOrientedBox` only if the anchor
  and lever code otherwise remains coupled to a private nested basis. Do not
  add a generalized rational-vector framework.
- [ ] Update all FixedMathSharp callers to use the new owners and confirm no
  anchor/lever method still delegates through `WideOrientedBox` or
  `WideVector2dTransform`.
- [ ] Keep the existing FixedMathSharp response APIs temporarily operational
  by routing them through the new policy-neutral owners. Their removal belongs
  to Phase 4 after Gravitas parity is proven.
- [ ] Run all FixedMathSharp `Release`, `ReleaseLean`, coverage, and
  point-anchor benchmark gates.
- [ ] Require 100% line, branch, and method coverage and zero managed
  allocation.
- [ ] Request independent review focused on semantic parity, accessibility
  breadth, wide-type leakage, and whether any new owner exists only for naming
  rather than a real responsibility.
- [ ] Update this plan with results and pause for owner review.

## Phase 3: Move Exact Response Policy Into Gravitas

**Files:**

- Modify:
  `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/InternalsVisibleTo.cs`
- Create under:
  `src/Gravitas/CollisionHandling/Response/Exact`
  - `ExactContactResponseOperand3D.cs`
  - `ExactNormalConstraint3D.cs`
  - `ExactNormalResponse3D.cs`
  - `ExactCoulombResponse3D.cs`
  - `ExactContactResponseKernel.Normal.cs`
  - `ExactContactResponseKernel.Coulomb.cs`
- Modify:
  `src/Gravitas/CollisionHandling/Response/3D/ExactContactLever3D.cs`
- Modify:
  `src/Gravitas/CollisionHandling/Response/2D/ExactContactLever2D.cs`
- Modify:
  `src/Gravitas/CollisionHandling/Response/3D/ContactNormalImpulse3D.cs`
- Modify:
  `src/Gravitas/CollisionHandling/Response/2D/ContactNormalImpulse2D.cs`
- Modify:
  `src/Gravitas/CollisionHandling/Response/2D/CollisionResponse2D.cs`
- Modify:
  `src/Gravitas/CollisionHandling/Response/Mixed/ContactNormalImpulseMixed.cs`
- Modify:
  `src/Gravitas/CollisionHandling/Response/Mixed/CollisionResponseMixed.cs`
- Validate and modify where the owned response value changes:
  - `src/Gravitas/CollisionHandling/Pairs/3D/CollisionPair.cs`
  - `src/Gravitas/Core/3D/GravitasPhysicsService.Response.cs`
  - `src/Gravitas/CollisionHandling/Pairs/2D/CollisionPair2D.cs`
  - `src/Gravitas/CollisionHandling/Pairs/2D/CollisionPair2D.ReplayHash.cs`
  - `src/Gravitas/Core/2D/GravitasPhysics2DService.Response.cs`
  - `src/Gravitas/Core/Mixed/GravitasMixedCollisionService.Response.cs`
  - `src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Rotational.Response.cs`
  - `src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.Rotational.Response.cs`
  - `src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Rotational.Mixed.cs`
  - `src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.Rotational.Mixed.cs`
- Test:
  `tests/Gravitas.Tests/CollisionHandling/CollisionResponseExactLeverTests.cs`
- Test:
  `tests/Gravitas.Tests/CollisionHandling/ContactNormalImpulseResponseTests.cs`
- Test:
  `tests/Gravitas.Tests/CollisionHandling/CollisionWarmStartTests.cs`
- Test: matching 2D, mixed, rotational CCD, replay, and allocation suites.
- Benchmark:
  `tests/Gravitas.Benchmarks/CollisionHandling/CollisionResponseBenchmarks.cs`
- Benchmark:
  `tests/Gravitas.Benchmarks/CollisionHandling/MixedCollisionResponseBenchmarks.cs`

**Interfaces:**

- FixedMathSharp produces internal exact ratios, fixed-width wide values, and
  final round-half-to-even materialization helpers.
- `ExactContactResponseKernel` consumes those internals and produces internal
  Gravitas velocity-delta and accumulator results.
- `ExactContactLever3D` and `ExactContactLever2D` adapt bodies, constrained
  mobility, and dimensional embedding to the shared kernel.

- [ ] Add `[assembly: InternalsVisibleTo("Gravitas")]` to FixedMathSharp's
  existing friend declaration file. Do not add friend entries for Gravitas
  tests or benchmarks.
- [ ] Add internal Gravitas response value types in separate files. Preserve
  existing projection flags, atomic final-delta semantics, and optional
  diagnostic projections without exposing FixedMathSharp wide types.
- [ ] Port the exact normal-response algorithm into
  `ExactContactResponseKernel.Normal.cs`, retaining validation for normalized
  axes, nonnegative restitution/thresholds, accumulated-impulse bounds,
  mobility-projected axes, and all-or-nothing final velocity materialization.
- [ ] Port line and disk Coulomb response into
  `ExactContactResponseKernel.Coulomb.cs`, retaining orthogonality,
  participant-consistency, static/dynamic friction, accumulator clamping, and
  atomic final-delta semantics.
- [ ] Use direct internal FixedMathSharp calls from the kernel. Do not copy limb
  arithmetic, add a Gravitas wide façade, or expose internal FixedMathSharp
  values beyond the exact response subsystem.
- [ ] Migrate pure 3D callers to the Gravitas response values and kernel.
- [ ] Migrate pure 2D callers through the existing X/Z embedding and signed
  angular convention without creating a second 2D solver implementation.
- [ ] Migrate mixed response, warm starts, friction, replay-visible state, and
  rotational CCD callers.
- [ ] While the old FixedMathSharp response entry points still exist, add
  temporary parity coverage comparing normal, accumulated-normal, line-
  friction, and disk-friction success/failure and every returned field across:
  ordinary inputs, full-domain levers, different denominators, separating
  contacts, stale accumulators, frozen/bodyless participants, mirrored scalar
  faces, diagnostic projection overflow, and true final overflow.
- [ ] Convert parity cases to stable Gravitas-owned expected-result assertions
  before Phase 4 removes the old FixedMathSharp oracle calls. Do not leave tests
  that merely prove a type or method exists.
- [ ] Run focused 2D, 3D, mixed, warm-start, rotational CCD, replay, and
  allocation tests in `Release` and `ReleaseLean`.
- [ ] Re-run both response benchmark classes against the Phase 0 baseline.
  Require zero allocation and investigate any material regression in inlining,
  bounds checks, or `stackalloc` pressure before proceeding.
- [ ] Generate Gravitas coverage and require 100% line, branch, and method
  coverage.
- [ ] Request independent physics and code-ownership review. Verify restitution
  and friction policy exists only in Gravitas and raw wide mechanics exist only
  in FixedMathSharp.
- [ ] Update this plan with results and pause for owner review.

## Phase 4: Remove The Intermediate v7 FixedMathSharp Solver Surface

**Files:**

- Modify:
  `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Geometry/Anchors/FixedLever.cs`
- Delete:
  - `FixedLeverResponseOperand3d.cs`
  - `FixedLeverNormalConstraint3d.cs`
  - `FixedLeverNormalResponse3d.cs`
  - `FixedLeverCoulombResponse3d.cs`
- Delete:
  - `Geometry/Wide/WideOrientedBox.NormalResponse.cs`
  - `Geometry/Wide/WideOrientedBox.CoulombResponse.cs`
- Delete or narrow:
  `F:/gamedevrepos/FixedMathSharp/tests/FixedMathSharp.Tests/Geometry/Primitives/FixedLeverNormalResponse.Tests.cs`
- Delete or narrow:
  `F:/gamedevrepos/FixedMathSharp/tests/FixedMathSharp.Tests/Geometry/Primitives/FixedLeverCoulombResponse.Tests.cs`
- Modify: the FixedMathSharp README/wiki only where they currently advertise
  solver response rather than general exact lever algebra.
- Modify:
  `F:/gamedevrepos/FixedMathSharp/docs/MIGRATION.md`

**Produces:**

- A final FixedMathSharp v7 public surface containing semantic lever algebra but
  no rigid-body response policy.
- Gravitas-owned regression coverage for every unique response contract worth
  retaining.

- [ ] Remove the four response methods from `FixedLever`.
- [ ] Delete the four intermediate v7 public response DTOs.
- [ ] Delete FixedMathSharp's normal and Coulomb response implementations after
  confirming no production caller remains.
- [ ] Audit the approximately 1,482 lines of FixedMathSharp response tests.
  Move only unique physical or arithmetic regressions into Gravitas; delete
  duplicates already proven by Gravitas response, replay, and allocation
  suites.
- [ ] Keep focused FixedMathSharp tests for the policy-neutral internal
  arithmetic that remains. Do not retain solver terminology solely to preserve
  deleted test cases.
- [ ] Update the v6-to-v7 migration guide to describe the final semantic lever
  and mass-property surface. Do not add a migration step saying unreleased v7
  response APIs were removed.
- [ ] Confirm no `FixedLeverResponse*`, `FixedLeverNormalConstraint*`, or
  `FixedLever.TryGet*Coulomb*Response` reference remains anywhere in the
  locally linked stack.
- [ ] Run complete FixedMathSharp and Gravitas `Release` and `ReleaseLean`
  suites.
- [ ] Generate coverage for both repositories and require 100% line, branch,
  and method coverage.
- [ ] Re-run response allocation and benchmark gates to prove deleting the old
  façade did not alter runtime behavior.
- [ ] Request independent public-surface and ownership review.
- [ ] Update this plan with results and pause for owner review.

## Phase 5: Mechanical Directory And Partial Ownership Cleanup

**Files:**

- Move files under `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Geometry`
  without changing namespaces.
- Modify:
  `F:/gamedevrepos/FixedMathSharp/docs/complexity-exceptions.md`
- Modify: repository navigation documentation only where paths are explicitly
  named.

**Target organization:**

```text
Geometry/
  Anchors/
    Wide/
  Primitives/
    Rays/
    Relations/
    Segments/
    Triangles/
  Wide/
    Common/
    Convex/
    FiniteAxis/
    MassProperties/
    OrientedBox/
```

**Produces:**

- Coherent navigation by mathematical owner without namespace or runtime
  behavior changes.
- Fewer misleading partials, not merely fewer partial files.

- [ ] Move `FixedSegment*` into `Geometry/Primitives/Segments`,
  `FixedTriangle*` into `Geometry/Primitives/Triangles`, `FixedRay*` into
  `Geometry/Primitives/Rays`, and reusable relation families into
  `Geometry/Primitives/Relations`. Keep dimensional counterparts adjacent.
- [ ] Move common wide geometry, convex relations, finite-axis algorithms,
  mass-property algorithms, and remaining true oriented-box algorithms into
  their corresponding `Geometry/Wide` subdirectories.
- [ ] Keep small standalone query/intersection files at `Geometry/Wide` root
  when an additional one-file directory would add no navigation value.
- [ ] Merge only small cohesive partials whose combined file remains focused,
  with initial candidates:
  - `FixedBoundArea`
  - `FixedRay`
  - `FixedOrientedBox`
  - `FixedConvex2dRelations`
  - `WideFiniteAxisProjection`
  - `WideFiniteConeIntersection`
  - `WideSlabProjection`
- [ ] Reduce `FixedSegment`, `FixedSegment2d`, and `FixedTriangle` partials into
  a few behavior-oriented files rather than one monolith or one file per
  method.
- [ ] Reassess `WideOrientedBox`, `WideFiniteAxisIntersection`, and
  `WideConvexPrismRelations` after Phases 1-4 have removed misplaced behavior.
  Split remaining code only where a new type has an independent invariant and
  reason to change.
- [ ] Keep width-specific `WideArithmetic` partials. Merge
  `WideArithmetic.Ratio.cs` into the core file only if the resulting owner
  remains clearer; do not merge width-specific files to reduce a count.
- [ ] Move normalization code out of `WideGeometry` and into a focused internal
  numeric normalization owner if its responsibility remains independent after
  the anchor extraction. Keep exact geometry differences, dot/cross products,
  and distance predicates in `WideGeometry`.
- [ ] Update `docs/complexity-exceptions.md` with the approximately 1,200-line
  review warning and evergreen owner-cohesion rule. Record exceptions only for
  measured cohesive algorithms, not current task status or machine-local
  paths.
- [ ] Run a namespace/API diff and confirm the phase performs no public
  namespace, signature, serialization, or behavior change.
- [ ] Run complete FixedMathSharp `Release`, `ReleaseLean`, and 100% coverage
  gates after the mechanical moves.
- [ ] Build Gravitas against the moved locally linked source and run focused
  exact-response tests to catch accidental internal path/owner errors.
- [ ] Request independent navigation and over-engineering review. Remove any
  new folder or type whose only benefit is satisfying the plan's proposed
  diagram.
- [ ] Update this plan with results and pause for owner review.

## Phase 6: Documentation, Package, And Cross-Stack Closure

**Files:**

- Modify:
  `F:/gamedevrepos/FixedMathSharp/README.md`
- Modify:
  `F:/gamedevrepos/FixedMathSharp/docs/wiki/bounds-and-geometry.md`
- Modify:
  `F:/gamedevrepos/FixedMathSharp/docs/MIGRATION.md`
- Modify: `README.md`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`
- Modify: `docs/feature-work/feature-work-overview.md`
- Modify: this plan

- [ ] Document FixedMathSharp as the owner of semantic exact geometry, lever
  algebra, mass-property arithmetic, and internal wide mechanics.
- [ ] Document Gravitas as the owner of exact rigid-body normal/friction
  response and the sole intentional non-test friend consumer of FixedMathSharp
  internals.
- [ ] State that the friendship is deliberate release coupling, not a public
  FixedMathSharp extension mechanism or a precedent for GridForge,
  SwiftCollections, Trailblazer, or host adapters.
- [ ] Document the release rule: release FixedMathSharp first, replace the local
  link with its package in Gravitas, then rebuild and validate Gravitas before
  release. Treat an internal FixedMathSharp change consumed by Gravitas as a
  coordinated compatibility event even when FixedMathSharp's public SemVer
  surface is unchanged.
- [ ] Confirm no FixedMathSharp internal type appears in Gravitas public XML
  documentation, protected/public signatures, serialization metadata, or host
  examples.
- [ ] Run final complete locally linked `Release`, `ReleaseLean`, replay,
  allocation, benchmark, and 100% coverage gates for both repositories.
- [ ] Build and test both standard and Lean packages for `net8.0` and
  `netstandard2.1`.
- [ ] After the owner releases FixedMathSharp, replace the Gravitas local link
  with the released v7 package and repeat Gravitas `Release`, `ReleaseLean`,
  coverage, replay, allocation, and response benchmark gates.
- [ ] Request independent final review of the source boundary, API surface,
  package matrix, documentation, and plan evidence.
- [ ] Move this plan to `docs/feature-work/done` and update the overview only
  after every exit criterion is proven.

## Current Status

- [x] Read-only ownership, duplication, partial, directory, and assembly-
  boundary audit complete.
- [x] Friend-access and rigid-body-policy boundary approved by the repository
  owner.
- [x] FixedMathSharp v6 confirmed as the released migration baseline; the
  current develop worktree is the unreleased v7 target.
- [x] Phase 0 runtime, coverage, CRAP, allocation, benchmark, caller, and
  semantic-ownership evidence captured.
- [ ] Phase 0 semantic keep/move recommendations approved and later phase file
  lists revised to match.
- [ ] Phase 1 proven wide-arithmetic duplication consolidated.
- [ ] Phase 2 policy-neutral anchor and lever owners extracted.
- [ ] Phase 3 Gravitas exact response policy migrated and parity-proven.
- [ ] Phase 4 intermediate v7 FixedMathSharp response surface removed.
- [ ] Phase 5 directory and partial ownership cleanup complete.
- [ ] Phase 6 documentation, package, and cross-stack closure complete.

## Exit Criteria

- FixedMathSharp exposes only standard-library-quality exact math and geometry
  semantics. Any point-anchor, lever, mass-point, or mass-weight type retained
  there has documented value independent of Gravitas; every physics-specific
  counterpart lives in Gravitas.
- Gravitas owns restitution, unilateral impulse accumulation, constrained
  inverse-mass/inertia response, solver result values, warm-start policy, and
  Coulomb friction response.
- Only `Gravitas`, `FixedMathSharp.Tests`, and
  `FixedMathSharp.Benchmarks` are intentional friends of the FixedMathSharp
  runtime assembly.
- Gravitas tests and benchmarks have no direct friendship with FixedMathSharp
  and no direct raw-wide public surface.
- General anchor/lever arithmetic no longer delegates through
  `WideOrientedBox` or `WideVector2dTransform`.
- No duplicated generic magnitude multiplication, bit-length, difference, or
  equal-length radical-pair implementation remains in the audited locations.
- Fixed-width unrolled arithmetic and contract-specific solvers remain direct
  and allocation-free where consolidation would weaken clarity or performance.
- FixedMathSharp's final v7 migration guide compares the released v6 surface to
  the final v7 surface and does not narrate discarded intermediate develop APIs.
- FixedMathSharp and Gravitas each report exactly 100% reachable line, branch,
  and method coverage without hollow tests or exclusions.
- Exact 2D, 3D, mixed, warm-start, friction, rotational CCD, replay, and final-
  overflow behavior matches the approved pre-migration contract.
- Common compact response and exact fallback benchmarks remain allocation-free,
  with any material throughput change explained and approved.
- Standard and Lean packages build and test for both target frameworks.
- Package-only Gravitas validation passes against the released FixedMathSharp
  v7 dependency before Gravitas release.
