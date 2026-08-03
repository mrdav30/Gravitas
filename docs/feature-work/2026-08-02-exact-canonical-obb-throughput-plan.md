# Exact Canonical OBB Throughput Hardening

**Created:** 2026-08-02  
**Status:** Phase 3 complete; paused for review  
**Signal:** Exact canonical OBB contacts regress ordinary narrow-phase throughput

## Goal

Recover ordinary-domain throughput for the competitive oriented-box contact
families without weakening FixedMathSharp's exact full-domain authority,
deterministic SAT ordering, canonical contact anchors, nearest-even depth
materialization, or zero-allocation contract.

The work should attempt a material improvement for box, triangle, convex-hull,
and finite-capsule relations. Retention is evaluated per family: a change should
normally improve its affected direct and Gravitas rows by at least `5%`
repeatably, while sibling rows remain within benchmark noise. Smaller changes
may be retained only when they delete meaningful work or code and are neutral
across the end-to-end gates.

The deleted saturating Gravitas SAT implementation is historical context, not a
correctness-compatible performance target.

## Preserved Baseline

The baseline was captured before source changes from the Release binaries on
2026-08-02. Every row reported `0 B` through BenchmarkDotNet.

### Direct FixedMathSharp Relations

| Row | Mean | Median |
| --- | ---: | ---: |
| `TrianglePrimary` | `77.951 us` | `77.641 us` |
| `BoxPrimary` | `64.528 us` | `64.503 us` |
| `CapsulePrimary` | `258.988 us` | `258.119 us` |
| `ConvexHullPrimary` | `367.617 us` | `367.314 us` |

Artifacts:
`../FixedMathSharp/artifacts/benchmarks/2026-08-02-obb-investigation-baseline`.

### Gravitas Narrow Phase

| Row | Mean | Median |
| --- | ---: | ---: |
| 64 rotated cuboid/cuboid pairs | `2.473 ms` | `2.471 ms` |
| 64 rotated cuboid/capsule pairs | `11.088 ms` | `11.082 ms` |
| 64 convex mesh/cuboid pairs | `7.122 ms` | `7.113 ms` |
| 64 concave mesh/cuboid pairs | `7.087 ms` | `7.085 ms` |

Artifacts: `artifacts/benchmarks/2026-08-02-obb-investigation-baseline`.

### Baseline Commands

FixedMathSharp:

```powershell
dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj `
    -c Release -f net8.0 --no-restore

dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll `
    oriented-box-anchor -j Short `
    --filter "*TrianglePrimary*" "*BoxPrimary*" "*CapsulePrimary*" "*ConvexHullPrimary*" `
    --exporters json `
    --artifacts artifacts/benchmarks/2026-08-02-obb-investigation-baseline
```

Gravitas uses the project-level Release build while local links are active:

```powershell
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj `
    -c Release -f net8.0 --no-restore `
    -p:GitVersion_AssemblySemVer=0.4.0.0 `
    -p:GitVersion_AssemblySemFileVer=0.4.0.0 `
    -p:GitVersion_FullSemVer=0.4.0 `
    -p:GitVersion_InformationalVersion=0.4.0

dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll `
    collision-detection -j Short `
    --filter "*CheckCuboidCuboidSatPairs*" "*CheckCuboidCapsulePairs*" `
             "*CheckMeshCuboidPairs*" "*CheckConcaveMeshCuboidPairs*" `
    --artifacts artifacts/benchmarks/2026-08-02-obb-investigation-baseline
```

The matching historical scalar rows were `385.66 us`, `411.69 us`,
`1.363 ms`, and `1.368 ms`. Those rows used narrower, saturating geometry and
remain useful only as evidence that ordinary contacts have substantial
optimization headroom.

## Root Cause

Gravitas performs candidate gathering and manifold handoff, then routes the
four rows directly through FixedMathSharp's `FixedOrientedBox` relations. The
remaining regression is not caused by Gravitas orchestration, managed
allocation, or local project-reference configuration.

Sampled Release traces isolate these relation-specific costs:

- Box/box spends roughly `56%` retaining SAT candidates, followed by exact
  support reconstruction and normalized-depth comparison.
- Triangle/box spends roughly `75%` retaining axes. Repeated rigid projection
  and fixed-width products dominate that work.
- Convex-hull/box spends roughly `89%` retaining axes; transformed vertex
  projection alone accounts for more than half of the sampled relation cost.
- Capsule/box spends roughly `91%` in penetration reduction. Candidate-axis
  construction and exact normalized comparison are both material.

The current kernels construct axes in world rational space, repeatedly project
those axes through rigid bases, and carry conservative full-domain widths
through ordinary inputs. Comparator-only specialization cannot address the
dominant triangle/hull projection work or capsule feature construction.

Profile artifacts:

- `../FixedMathSharp/artifacts/benchmarks/2026-08-02-obb-capsule-profile`
- `../FixedMathSharp/artifacts/benchmarks/2026-08-02-obb-other-profiles`

## Approved Design

### Canonical Exact Computation Frame

Use the oriented box's local frame as the exact computation frame. In that
frame the box is centered and axis-aligned, while the target pose is represented
by one exact relative rational rotation and translation.

This changes the algebra, not the answer:

- construct relative rigid state once per query;
- transform each candidate axis into the other shape's local frame once;
- project authored local coordinates against that prepared axis;
- retain the existing SAT axis order, feature ranks, and tie ownership;
- transform only the winning normal and support witnesses for final public
  materialization.

Reuse `WideRationalBasis3d`, `WideRigidProjection`, existing `Signed*` values,
and the current exact comparison/materialization owners wherever they still fit.
Add a focused internal value only when the relation otherwise repeats the same
relative-frame state. Do not add forwarding owners or speculative public APIs.

### Relation Families

1. **Box/box:** evaluate the standard 15 OBB SAT axes from the exact relative
   orientation matrix and local translation. Avoid projecting two boxes through
   separate world bases for every axis.
2. **Triangle and convex hull:** prepare the target-local representation of one
   candidate axis, then scan raw local vertices directly. Do not recompute the
   three basis-axis projections for each vertex.
3. **Finite capsule:** construct box faces, box-edge/capsule-axis crosses,
   vertex-to-axis features, and endpoint-to-edge features in box-local space.
   Preserve the current feature ranking and exact radical comparison.

### Compact Exact Path And Fallback

Ordinary inputs should use the narrowest proven existing fixed-width values.
Admission is checked and exact; it must never clamp, saturate, or approximate an
intermediate. If a required value cannot be represented by the compact kernel,
the query routes atomically to the current full-domain wide kernel.

A compact separation result is authoritative because every predicate evaluated
before it is exact. A compact overflow or unproven-width result is not a physics
answer and must fall back without publishing partial contact state.

Keep the current wide implementation only as long as it remains necessary for
full-domain fallback or differential validation. Delete superseded arithmetic
when the canonical kernel proves the same full domain; do not leave two complete
permanent implementations for convenience.

### API And Ownership

- No FixedMathSharp or Gravitas public API changes.
- No collider-owned world-geometry cache or cross-frame invalidation state.
- No Gravitas-local SAT, wide-arithmetic copy, or approximate prefilter.
- No GJK/EPA/MPR replacement in this workstream. Their bounded convergence,
  degeneracy ownership, and anchor stability need a separate evidence-backed
  design if the exact SAT architecture later proves insufficient.
- FixedMathSharp owns all policy-neutral exact arithmetic and geometry changes.
  Gravitas changes should be limited to benchmarks, focused integration tests,
  and documentation unless profiling exposes a distinct downstream cost.

## Phased Work Plan

Every implementation phase closes independently before review. FixedMathSharp
and Gravitas must each retain 100% reachable line, branch, and method coverage
at that boundary. The phase review also removes newly unreachable branches,
superseded helpers, one-line forwarding hops, and duplicated arithmetic instead
of carrying cleanup debt into Phase 5. Tests must protect meaningful behavior;
do not add reflection or API-shape assertions solely to satisfy coverage.

### Phase 0: Baseline And Design

- [x] Reproduce all four direct FixedMathSharp rows.
- [x] Reproduce all four matching Gravitas rows from the authoritative Release
      project build.
- [x] Confirm zero managed allocation in the benchmark rows.
- [x] Capture sampled relation traces and attribute the dominant work.
- [x] Audit the current Gravitas call graph and deleted scalar history.
- [x] Approve the canonical exact OBB-frame design and per-family evidence gate.

### Phase 1: Relative-Frame Foundation And Box/Box

- [x] Add focused equivalence tests before changing the box/box kernel.
- [x] Centralize only the relative-frame arithmetic repeated by current OBB
      relations.
- [x] Implement exact 15-axis box/box reduction in the canonical box frame.
- [x] Preserve winner ordering, normal direction, anchors, and clamped-depth
      behavior across ordinary, tie, degenerate-axis, and extreme inputs.
- [x] Measure direct `BoxPrimary` and Gravitas cuboid/cuboid rows; retain only a
      proven result.
- [x] Re-achieve 100% reachable coverage in FixedMathSharp and Gravitas, then
      remove any superseded box/box helpers or duplicate projection logic.
- [x] Pause for review.

#### Phase 1 Execution Record

1. Characterize the existing public contract in
   `FixedOrientedBox.Box.Tests.cs`: retain the exact rotated cross-axis winner,
   add an equal-depth face tie, and repeat the rotated fixture near the scalar
   limits to prove translation invariance without materializing world points.
2. Add one focused internal arithmetic test for an exact relative rational
   basis, then implement that composition on `WideRationalBasis3d`. Reuse the
   same basis-dot owner from relative bounds and expose basis rows/columns from
   the basis itself instead of retaining box-local forwarding helpers.
3. Replace the box/box world's repeated rigid projections with the canonical
   15-axis formulas. Preserve the existing candidate order
   `A0, B0, A1, B1, A2, B2`, then `Ai x Bj`; retain the current exact
   normalized-depth comparator and nearest-even materialization; construct the
   world-space axis only for the winner.
4. Delete the superseded generic box/box reducer after the full-domain width
   proof and public differential fixtures pass. Move any touched shared
   projection helper to its existing common owner rather than adding a façade
   or a second box implementation.
5. Run the focused box suite, the complete FixedMathSharp and Gravitas Release
   suites, and project-wide coverage analysis. The phase does not close until
   both repositories report 100% reachable line, branch, and method coverage
   and the diff contains no newly unreachable or assertion-only code.
6. Build the Release benchmark projects and run `BoxPrimary` plus Gravitas
   cuboid/cuboid twice. Retain the rewrite only if both confirmations clear the
   documented per-family gate with `0 B`, or if a smaller result removes
   meaningful code while remaining performance-neutral.
7. Request independent arithmetic/correctness and hot-path reviews, resolve
   findings, update this record with measured evidence, and stop for review.

#### Phase 1 Result

- The old world-axis box reducer was replaced by one exact relative-frame
  kernel. The full normalized-quaternion/raw-coordinate domain fits the existing
  `Signed192`, `Signed320`, and `Signed576` owners, so no compact fallback or
  second implementation remains.
- Candidate order remains `A0, B0, A1, B1, A2, B2`, then the nine `Ai x Bj`
  axes. Only the winning world axis is constructed; the existing exact depth,
  normal, and matched-anchor materializers remain authoritative.
- Relative basis composition, basis-axis access, point projection, extent
  projection, and box-radius projection reuse existing owners. The phase also
  deleted the previous generic box reducer, local basis-axis helper, duplicate
  basis dot product, duplicate rigid-local point projection, and misplaced
  convex-prism radius owner.
- Meaningful fixtures now pin exact cross-axis output, first- and second-box
  face winners, strict equal-depth tie ownership, parallel-axis degeneration,
  skew cross-axis separation, near-limit translation invariance including
  anchor displacement, opposite scalar-face separation, and clamped full-domain
  depth.
- Direct `BoxPrimary` confirmation medians were `32.030 us` and `32.195 us`
  versus `64.503 us` baseline: approximately `50.3%` and `50.1%` faster.
- Gravitas 64-pair cuboid/cuboid confirmation medians were `1.326 ms` and
  `1.314 ms` versus `2.471 ms` baseline: approximately `46.3%` and `46.8%`
  faster. All four confirmation rows reported `0 B`.
- FixedMathSharp Release passed 2,658 core tests plus 8 Chronicler tests;
  ReleaseLean passed 2,637 core tests plus 8 Chronicler tests. Gravitas Release
  passed 3,925 tests and ReleaseLean passed 3,870 tests. With local links,
  switching standard/Lean gates requires a configuration-specific restore so
  shared lower-stack assets cannot leak between configurations.
- Final FixedMathSharp coverage is 53,122/53,122 lines, 8,776/8,776 branches,
  and 3,416/3,416 reported methods. Final Gravitas coverage is 43,911/43,911
  lines, 12,845/12,845 branches, and 4,510/4,510 methods.
- Independent arithmetic, performance, and test reviews found no unresolved
  issue. Their two actionable checks—shared rigid-point adoption and a rotated
  max-to-min separation fixture—were resolved before closure.

Confirmation artifacts:

- `../FixedMathSharp/artifacts/benchmarks/2026-08-02-obb-phase1-confirmation-1`
- `../FixedMathSharp/artifacts/benchmarks/2026-08-02-obb-phase1-confirmation-2`
- `artifacts/benchmarks/2026-08-02-obb-phase1-confirmation-1`
- `artifacts/benchmarks/2026-08-02-obb-phase1-confirmation-2`

### Phase 2: Triangle And Convex-Hull Adoption

- [x] Hoist each exact target-local candidate-axis projection out of vertex
      loops.
- [x] Route fixed triangles and arbitrary convex point spans through the shared
      canonical projection reducer without allocating transformed-vertex
      buffers.
- [x] Preserve face, box-axis, and edge-cross order plus support-point ties.
- [x] Measure `TrianglePrimary`, `ConvexHullPrimary`, and both Gravitas
      mesh/cuboid rows independently.
- [x] Re-achieve 100% reachable coverage in FixedMathSharp and Gravitas, then
      remove superseded point-span projection helpers and duplicate arithmetic.
- [x] Pause for review.

#### Phase 2 Execution Record

1. Capture a fresh pre-source-change baseline for the direct triangle and
   convex-hull relations plus both Gravitas mesh/cuboid rows. Keep the phase
   evidence separate from the earlier investigation baseline so the retained
   result is compared against the exact Phase 1 source boundary.
2. Add behavioral fixtures before changing the reducers: strict first-axis and
   first-support ties, a distinct third-vertex support winner, exact positive
   edge-cross output, rotated near-limit translation equivalence, nonzero point
   span slices, and warmed zero-allocation execution.
3. Reuse `WideRationalBasis3d.CreateRelative(...)` to place target points and
   candidate axes in the box-local exact frame. Compute the three target-local
   axis projections once per candidate, then scan authored raw coordinates
   directly instead of transforming or buffering vertices.
4. Preserve the existing triangle axis order—box axes, triangle face, then
   `AB`, `BC`, and `CA` crossed with each box axis—and preserve convex-hull
   topology order. Retain strict first-candidate and first-support tie ownership;
   transform only the winning axis back to world space.
5. Move the reusable exact axis dot product and prepared rigid projections to
   their existing common owners. Delete the superseded transformed-offset
   projection, duplicate OBB axis projection, duplicate triangle projection,
   and duplicate convex-hull support scan rather than retaining forwarding
   helpers or a fallback kernel.
6. Run full Release, ReleaseLean, coverage, package, and benchmark gates for
   both repositories. Rerun each benchmark twice from the final source and
   request independent correctness, performance/bloat, and test-quality review.

#### Phase 2 Result

- Triangle and arbitrary convex point spans now share the canonical exact
  relative-frame projection primitives. No transformed-vertex buffer, compact
  answer path, second full-domain kernel, allocation, or public API was added.
- Exact classification, candidate order, strict ties, support witnesses,
  nearest-even depth, clamped-depth reporting, and both local contact anchors
  remain authoritative across ordinary, rotated, degenerate, and near-limit
  inputs.
- Fresh direct baselines were `79.198 us` median for `TrianglePrimary` and
  `375.649 us` for `ConvexHullPrimary`. Final confirmation medians were
  `51.208 us` / `51.149 us` and `135.960 us` / `135.085 us`: approximately
  `35.3%`–`35.4%` and `63.8%`–`64.0%` faster, respectively.
- Fresh Gravitas baselines were `7.049 ms` median for convex mesh/cuboid and
  `7.044 ms` for concave mesh/cuboid. Final confirmation medians were
  `4.873 ms` / `4.756 ms` and `4.708 ms` / `4.758 ms`: approximately
  `30.9%`–`32.5%` and `32.5%`–`33.2%` faster. Every measured row reported
  `0 B`.
- FixedMathSharp Release passed 2,662 core tests plus 8 Chronicler tests;
  ReleaseLean passed 2,641 core tests plus 8 Chronicler tests. Gravitas Release
  passed 3,925 tests and ReleaseLean passed 3,870 tests. Both target-framework
  package builds remained warning-free.
- Final FixedMathSharp coverage is 47,301/47,301 lines, 8,708/8,708 branches,
  and 3,425/3,425 methods. Final Gravitas coverage is 43,911/43,911 lines,
  12,845/12,845 branches, and 4,510/4,510 methods.
- Independent review found no unresolved correctness issue. Copy-heavy exact
  operands were changed to `in` parameters, duplicated dot/support/projection
  owners were deleted, the internal prepared-projection fixture was made
  rotation-sensitive, and missing translation/tie/edge-cross contracts were
  added before closure.

Phase 2 artifacts:

- `../FixedMathSharp/artifacts/benchmarks/2026-08-02-obb-phase2-baseline`
- `../FixedMathSharp/artifacts/benchmarks/2026-08-02-obb-phase2-final-confirmation-1`
- `../FixedMathSharp/artifacts/benchmarks/2026-08-02-obb-phase2-final-confirmation-2`
- `artifacts/benchmarks/2026-08-02-obb-phase2-baseline`
- `artifacts/benchmarks/2026-08-02-obb-phase2-final-confirmation-1`
- `artifacts/benchmarks/2026-08-02-obb-phase2-final-confirmation-2`

### Phase 3: Finite-Capsule Adoption

- [x] Express current finite capsule feature axes in the canonical box frame.
- [x] Preserve exact radial separation, feature rank, endpoint/edge ownership,
      support anchors, and full-domain fallback.
- [x] Measure `CapsulePrimary` and Gravitas cuboid/capsule rows.
- [x] Re-achieve 100% reachable coverage in FixedMathSharp and Gravitas, then
      remove superseded capsule-axis helpers and duplicate arithmetic.
- [x] Pause for review.

Phase 3 result:

- FixedMathSharp now expresses box faces, box/capsule cross axes, vertex/core
  axes, and endpoint/edge axes in one exact box-local frame. It transforms only
  the winning normal back to world space and retains the existing exact radial,
  depth, rank, ownership, and anchor contracts.
- Query-wide denominator, scale, extent, capsule-length, and vertex projection
  invariants are computed once. Edge-center work is shared across endpoint
  signs, large exact values use read-only references, and the superseded OBB
  world-radius, world-center, edge-reconstruction, and forwarding helpers were
  deleted. Shared convex-hull/capsule owners remain live and unchanged.
- Exact behavior tests now pin equal-rank first-candidate precedence, later
  rank-zero replacement on an exact tie, rotated endpoint/edge parity,
  perpendicular segment-surface support with a free box coordinate, translated
  full-domain non-face behavior, rigid-pose corner identity, and zero warmed
  allocations. Recorded deliberate mutations failed each matching fixture.
- Fresh direct baseline: `CapsulePrimary` mean `258.064 us`, median
  `257.767 us`, `0 B`. Final confirmations were mean/median
  `156.638/155.690 us` and `155.472/153.880 us`, or `39.6%` and `40.3%`
  faster by median, both `0 B`.
- Fresh Gravitas baseline: 64 cuboid/capsule pairs mean `11.295 ms`, median
  `11.294 ms`, `0 B`. Final confirmations were mean/median `5.040/5.006 ms`
  and `5.114/5.114 ms`, or `55.7%` and `54.7%` faster by median, both `0 B`.
- FixedMathSharp Release passed 2,669 tests plus 8 Chronicler integration tests;
  ReleaseLean passed 2,648 plus 8. Gravitas Release passed 3,925 tests and
  ReleaseLean passed 3,870. Standard and Lean package builds remained
  warning-free for `net8.0` and `netstandard2.1`.
- Final FixedMathSharp reachable coverage is 47,468/47,468 lines,
  8,722/8,722 branches, and 3,486/3,486 methods. Final Gravitas reachable
  coverage is 43,232/43,232 lines, 12,843/12,843 branches, and 4,487/4,487
  methods. Generated MemoryPack formatters and compiler-generated automatic
  property accessors remain outside the reachable-code gate rather than being
  padded with hollow tests.
- Independent correctness and quality re-reviews approved the correlated
  full-domain width proof, local/world algebraic equivalence, deterministic
  feature contracts, invariant hoists, allocation behavior, and deletion of
  zombie helpers. No Gravitas production change, new public API, approximate
  prefilter, cache, fallback kernel, or managed allocation was added.

Phase 3 artifacts:

- `../FixedMathSharp/artifacts/benchmarks/2026-08-02-obb-phase3-baseline`
- `../FixedMathSharp/artifacts/benchmarks/2026-08-02-obb-phase3-final-confirmation-1`
- `../FixedMathSharp/artifacts/benchmarks/2026-08-02-obb-phase3-final-confirmation-2`
- `artifacts/benchmarks/2026-08-02-obb-phase3-baseline`
- `artifacts/benchmarks/2026-08-02-obb-phase3-final-confirmation-1`
- `artifacts/benchmarks/2026-08-02-obb-phase3-final-confirmation-2`
- `../FixedMathSharp/tests/FixedMathSharp.Tests/TestResults/coverage-analysis`
- `tests/Gravitas.Tests/TestResults/coverage-analysis`

### Phase 4: Evidence-Gated Residual Optimization

- [ ] Reprofile every retained family using the same fixtures.
- [ ] Apply only shared arithmetic or invariant-work deletion supported by the
      new trace.
- [ ] Revert any experiment below the per-family gate unless it materially
      simplifies code with neutral performance.
- [ ] Stop rather than introduce a second approximate answer path or permanent
      relation-specific bloat.
- [ ] Re-achieve 100% reachable coverage in FixedMathSharp and Gravitas before
      retaining any residual optimization.

### Phase 5: Coverage, Documentation, And Cross-Stack Closure

- [ ] Run focused ordinary, extreme, tie-order, anchor, and allocation tests.
- [ ] Re-achieve 100% reachable line, branch, and method coverage in every
      modified repository without hollow API-shape tests.
- [ ] Run Release and ReleaseLean suites, both target-framework package builds,
      and relevant CRAP/complexity review.
- [ ] Rerun each direct and Gravitas benchmark twice from authoritative Release
      project builds and record retained/rejected evidence.
- [ ] Update benchmark READMEs, complexity exceptions, this plan, the backlog,
      and feature-work overview.
- [ ] Obtain independent correctness and performance review before closure.
- [ ] Move this plan to `done` only after the retained source and evidence agree.

## Closure Gates

- Every contact result remains deterministic and exact across the complete
  authored raw domain.
- Compact and fallback paths agree on classification, winning feature, normal,
  depth, clamping, and both anchors wherever both can execute.
- All affected direct and Gravitas benchmark rows remain allocation-free after
  warmup.
- Each retained family normally improves by at least `5%` repeatably; the goal
  is to improve all four, not to stop after the first win.
- No sibling competitive collider row shows a repeatable material regression.
- FixedMathSharp and Gravitas retain 100% reachable line, branch, and method
  coverage.
- Standard and Lean packages build warning-free for `net8.0` and
  `netstandard2.1`.

## Rejected Directions

- Restore the deleted Gravitas scalar SAT or world-corner authority.
- Add a narrowed scalar prefilter that can disagree with exact classification.
- Cache transformed collider geometry across frames.
- Optimize only the normalized-depth comparator and declare the family closed.
- Add platform-specific SIMD or floating-point math to deterministic kernels.
- Replace SAT wholesale with an iterative support-mapping solver without a
  separate deterministic convergence and contact-quality design.
