# Exact Canonical OBB Throughput Hardening

**Created:** 2026-08-02  
**Status:** Design approved; implementation pending  
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

### Phase 0: Baseline And Design

- [x] Reproduce all four direct FixedMathSharp rows.
- [x] Reproduce all four matching Gravitas rows from the authoritative Release
      project build.
- [x] Confirm zero managed allocation in the benchmark rows.
- [x] Capture sampled relation traces and attribute the dominant work.
- [x] Audit the current Gravitas call graph and deleted scalar history.
- [x] Approve the canonical exact OBB-frame design and per-family evidence gate.

### Phase 1: Relative-Frame Foundation And Box/Box

- [ ] Add focused equivalence tests before changing the box/box kernel.
- [ ] Centralize only the relative-frame arithmetic repeated by current OBB
      relations.
- [ ] Implement exact 15-axis box/box reduction in the canonical box frame.
- [ ] Preserve winner ordering, normal direction, anchors, and clamped-depth
      behavior across ordinary, tie, degenerate-axis, and extreme inputs.
- [ ] Measure direct `BoxPrimary` and Gravitas cuboid/cuboid rows; retain only a
      proven result.
- [ ] Pause for review.

### Phase 2: Triangle And Convex-Hull Adoption

- [ ] Hoist each exact target-local candidate-axis projection out of vertex
      loops.
- [ ] Route fixed triangles and arbitrary convex point spans through the shared
      canonical projection reducer without allocating transformed-vertex
      buffers.
- [ ] Preserve face, box-axis, and edge-cross order plus support-point ties.
- [ ] Measure `TrianglePrimary`, `ConvexHullPrimary`, and both Gravitas
      mesh/cuboid rows independently.
- [ ] Pause for review.

### Phase 3: Finite-Capsule Adoption

- [ ] Express current finite capsule feature axes in the canonical box frame.
- [ ] Preserve exact radial separation, feature rank, endpoint/edge ownership,
      support anchors, and full-domain fallback.
- [ ] Measure `CapsulePrimary` and Gravitas cuboid/capsule rows.
- [ ] Pause for review.

### Phase 4: Evidence-Gated Residual Optimization

- [ ] Reprofile every retained family using the same fixtures.
- [ ] Apply only shared arithmetic or invariant-work deletion supported by the
      new trace.
- [ ] Revert any experiment below the per-family gate unless it materially
      simplifies code with neutral performance.
- [ ] Stop rather than introduce a second approximate answer path or permanent
      relation-specific bloat.

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
