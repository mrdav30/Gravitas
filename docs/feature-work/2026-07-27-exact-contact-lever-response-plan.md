# Exact Contact Lever And Mass Response Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> `superpowers:test-driven-development` while implementing each behavior and
> `superpowers:verification-before-completion` before reporting completion.

**Goal:** Preserve physically real contacts and compound mass properties when
their semantic lever arms, child centers, or weights exceed the Q32.32 scalar
domain, without saturation, lost torque, or query-local workarounds.

**Architecture:** FixedMathSharp will materialize an exact semantic
`FixedLever` or `FixedLever2d` from two point anchors and cache the relative
coordinate ratio once. Those types expose only the vector, cross, quadratic,
and fused-scale operations required by consumers. Common representable
Gravitas contacts will retain their compact vector path; only true
scalar-domain overflow will reconstruct the larger exact lever from the
manifold's semantic anchors. Raw wide integers will remain internal.

**Tech Stack:** C# 11, Q32.32 `Fixed64`, FixedMathSharp fixed-width wide
arithmetic, xUnit v3, BenchmarkDotNet, Gravitas 2D/3D/mixed solvers and CCD.

## Global Constraints

- Correctness and determinism outrank throughput; do not saturate or silently
  remove angular response.
- Keep FixedMathSharp engine agnostic and expose no raw wide-number types.
- Preserve the compact representable lever path. Exact fallback work must be
  allocation-free and entered only when lever materialization or the complete
  compact-expression proof fails.
- Round once, half to even, at each final public scalar or vector result.
- A truly unrepresentable final velocity update fails atomically and
  diagnostically; it does not partially mutate a body.
- Reuse existing wide helpers before adding arithmetic. Consolidate duplicated
  logic only when this work directly encounters it.
- The 3D quadratic numerator is conservatively below 739 bits and its
  denominator below 532 bits across valid point anchors; keep both in the
  existing signed 832-bit representation instead of adding arbitrary-width
  accumulation.
- Preserve 100% FixedMathSharp and Gravitas hand-authored line and branch
  coverage.
- Pause after each phase for user review. Leave changes unstaged and
  uncommitted.

---

### Phase 0: Root Cause And Contract

**Files:**
- Read: `docs/feature-work/issue-tracker.md`
- Read: `src/Gravitas/CollisionHandling/Response`
- Read: `src/Gravitas/CollisionHandling/CCD`
- Read: `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Geometry/Primitives/FixedPointAnchor.cs`
- Read: `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Geometry/Primitives/FixedPointAnchor2d.cs`

- [x] Confirm 2D, 3D, and mixed response reject an otherwise valid contact when
  a final lever vector cannot be narrowed to Q32.32.
- [x] Confirm rotational CCD reaches the same vector-only angular kernels.
- [x] Confirm compound child centers and weights have independent earlier
  representation boundaries.
- [x] Reject saturation, linear-only response, contact removal, and
  Gravitas-local wide arithmetic.
- [x] Approve exact semantic levers and mass points with a compact fast path and
  rare exact fallback.

### Phase 1: FixedMathSharp Exact Anchored-Vector Algebra

**Files:**
- Modify: `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Geometry/Primitives/FixedPointAnchor.cs`
- Modify: `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Geometry/Primitives/FixedPointAnchor2d.cs`
- Create: `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Geometry/Primitives/FixedLever.cs`
- Create: `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Geometry/Primitives/FixedLever2d.cs`
- Create: `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Geometry/Wide/WideOrientedBox.PointAnchorResponse.cs`
- Modify: `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Numerics/Scalars/Fixed64.WideRatio.cs`
- Modify: `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Numerics/Wide/WideArithmetic.Signed704.cs`
- Modify: `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Numerics/Wide/WideArithmetic.Signed832.cs`
- Modify: `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Numerics/Wide/WideVector2dTransform.cs`
- Create: `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Numerics/Wide/WideVector2dTransform.PointAnchorResponse.cs`
- Modify: `F:/gamedevrepos/FixedMathSharp/tests/FixedMathSharp.Tests/Geometry/Primitives/FixedPointAnchor.Tests.cs`
- Modify: `F:/gamedevrepos/FixedMathSharp/tests/FixedMathSharp.Tests/Geometry/FixedPointAnchor2d.Tests.cs`
- Modify: `F:/gamedevrepos/FixedMathSharp/tests/FixedMathSharp.Benchmarks/PointAnchorBenchmarks.cs`

**Interfaces:**
- 3D exact relative cross projection for point velocity.
- 3D exact cross-product quadratic form for angular effective mass.
- 3D exact transformed cross product with fused scale/divide for angular
  velocity updates.
- 2D exact relative cross product, scaled cross product, and scaled squared
  cross product counterparts.

- [x] Add failing tests for ordinary parity, true unrepresentable levers with
  representable final results, mirrored scalar faces, final overflow, invalid
  anchors, zero divisor, and one-round fused scaling.
- [x] Implement exact semantic lever types backed by cached relative-coordinate
  ratios; do not introduce a public general-purpose wide vector or expression
  tree.
- [x] Reuse one relative-ratio extraction path across existing anchor distance
  and response operations where it removes direct duplication.
- [x] Add focused benchmarks for compact and exact-fallback operations and
  confirm zero allocation.
- [x] Run both target frameworks, `Release`, `ReleaseLean`, and coverage; require
  100% line and branch coverage before Phase 1 review.

### Phase 2: Gravitas 3D Response And Rotational CCD

**Files:**
- Modify: `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Geometry/Primitives/FixedLever.cs`
- Modify: `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Geometry/Wide/WideOrientedBox.PointAnchorResponse.cs`
- Modify: `src/Gravitas/CollisionHandling/Response/3D/CollisionResponse.cs`
- Modify: `src/Gravitas/CollisionHandling/Response/3D/ContactNormalImpulse3D.cs`
- Create: `src/Gravitas/CollisionHandling/Response/3D/ExactContactLever3D.cs`
- Modify: `src/Gravitas/CollisionHandling/Contacts/3D/ContactWarmStartCache.cs`
- Modify: `src/Gravitas/CollisionHandling/Pairs/3D/CollisionPair.cs`
- Modify: `src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Rotational.Response.cs`
- Modify: matching 3D response and CCD tests and benchmarks

- [x] Keep materialized `Vector3d` levers as the common solver path and retain
  semantic anchors only for overflow fallback.
- [x] Route point velocity, angular effective mass, and angular updates through
  exact anchor operations when a lever cannot materialize or the compact
  expression cannot be proven representable.
- [x] Preserve linear and angular response, warm-start state, friction,
  restitution, support/grounding, and deterministic contact ordering.
- [x] Reject only a truly unrepresentable final state, atomically and with one
  stable diagnostic.
- [x] Discard an unrepresentable stale warm start and cold-solve the contact
  instead of permanently stranding an otherwise representable response.
- [x] Fuse exact relative point-velocity projection so individually
  unrepresentable linear/angular terms may cancel before the one final scalar
  conversion.
- [x] Keep exact point velocity, effective mass, shared impulse, and cache
  completion wide until each final body delta; scalar impulse and velocity
  projections are optional diagnostics, not response admission gates.
- [x] Cover discrete response, rotational CCD, replay, mirrored scalar faces,
  and warmed zero-allocation behavior; pause for review.

### Phase 3: 2D And Mixed Response Parity

**Files:**
- Modify: `src/Gravitas/CollisionHandling/Response/2D`
- Modify: `src/Gravitas/CollisionHandling/Response/Mixed`
- Modify: matching 2D/mixed response, replay, diagnostic, and allocation tests

- [x] Apply the same compact/fallback contract to pure 2D.
- [x] Apply the 3D and planar exact operations to mixed contacts without
  duplicating FixedMathSharp arithmetic inside Gravitas.
- [x] Preserve constrained mixed impulse semantics and atomic body mutation.
- [x] Prove 2D/3D/mixed diagnostic, replay, mirrored-face, and warmed allocation
  parity; pause for review.

### Phase 4: Semantic Compound Mass Points And Wide Weights

**Files:**
- Modify: `F:/gamedevrepos/FixedMathSharp/src/FixedMathSharp/Geometry`
- Modify: `src/Gravitas/Colliders/Shapes`
- Modify: `src/Gravitas/CollisionHandling/Response`
- Modify: matching FixedMathSharp and Gravitas compound mass-property tests and
  benchmarks

- [x] Define the minimum semantic mass-point and positive-weight operations
  needed for primitive, shell, and composed-part aggregation.
- [x] Carry exact weights through weighted center and inertia distribution so
  no relative ratio is lost to early saturation.
- [x] Support child centers outside Q32.32 when the final compound center and
  inertia are representable.
- [x] Cover primitive and composed-part ratios, cancellation, parallel-axis terms,
  mirrored scalar faces, final overflow, replay, and warmed allocation.
- [x] Restore both repositories to 100% coverage and pause for review.

### Phase 5: Mixed Query Boundary And Release Closure

**Files:**
- Modify: `src/Gravitas/CollisionHandling/Detection/Mixed`
- Modify: matching mixed contact/query tests
- Modify: `F:/gamedevrepos/FixedMathSharp/docs/wiki/bounds-and-geometry.md`
- Modify: `F:/gamedevrepos/FixedMathSharp/docs/MIGRATION.md`
- Modify: `docs/wiki/COLLISION_PIPELINE.md`
- Modify: `docs/wiki/QUERY_SERVICES.md`
- Modify: `docs/feature-work/issue-tracker.md`
- Modify: this plan

- [ ] Replace the far-circle public-point fallback with an exact semantic
  boundary operation; do not catch exceptions or synthesize saturated points.
- [ ] Run complete locally linked `Release`, `ReleaseLean`, replay, allocation,
  benchmark, and 100% coverage gates across FixedMathSharp and Gravitas.
- [ ] Compare like-for-like benchmark medians and record regressions honestly.
- [ ] Complete final code review, resolve material findings, move the tracker
  issue to resolved history, and move this plan to `done`.

## Current Status

- [x] Root-cause audit and first-class semantic design approved.
- [x] Phase 1 complete and paused for review: semantic lever APIs, exact
  arithmetic, tests, benchmarks, coverage, and independent review are closed.
- [x] Phase 2 complete and paused for review: 3D discrete response and
  rotational CCD preserve full-domain lever behavior through a compact
  descriptor and no-inline exact fallback.
- [x] Phase 3 complete and paused for review: pure 2D and mixed discrete
  response, warm starts, friction, diagnostics, and rotational CCD share the
  compact/exact contract while preserving atomic constrained-body mutation.
- [x] Phase 4 complete and paused for review: primitive, mesh-shell, and flat
  authored compound paths now retain semantic mass points and exact positive
  weights through final center, mass distribution, and parallel-axis
  materialization.
- [ ] Phase 5 pending.

## Evidence To Date

- Existing rejection regressions pass `6/6`, confirming the current 2D, 3D,
  and mixed solvers intentionally drop response when a final lever cannot be
  materialized.
- FixedMathSharp enters this work at `44,047/44,047` lines,
  `8,422/8,422` branches, and `3,312/3,312` methods.
- Gravitas enters this work at `37,548/37,548` lines,
  `11,865/11,865` branches, and `4,246/4,246` methods.
- FixedMathSharp focused semantic-response contracts pass `11/11`; the complete
  Release test project passes `2,601/2,601`.
- FixedMathSharp `Release` and `ReleaseLean` build both `net8.0` and
  `netstandard2.1` with zero warnings or errors. `ReleaseLean` tests pass
  `2,580/2,580`.
- Final coverage is `45,552/45,552` lines, `8,554/8,554` branches, and
  `3,396/3,396` fully covered methods.
- Short-run medians are `33.28 ns` for compact cross projection, `711.14 ns`
  for the equivalent exact representable projection, `1.876 us` to create a
  full-domain lever, `869.19 ns` for full-domain cross projection, `1.490 us`
  for the quadratic form, `1.728 us` for transformed cross, and `894.18 ns`
  for the 2D squared-cross path. All report zero managed allocation; the
  compact/exact gap supports retaining exact algebra as an overflow fallback.
- Independent review found no critical or important findings. Its width review
  replaced an initial arbitrary 26-word accumulator with the existing signed
  832-bit representation and removed the now-unneeded generic span-ratio path.
- Gravitas exact-response regressions cover mirrored scalar faces, warm start,
  restitution, dynamic friction, frozen axes, final-overflow diagnostics,
  atomic rejection, stale-cache cold fallback, deterministic pair priority,
  rotational CCD, different exact-lever denominators, wide shared impulses,
  zero-rounded impulse projections, and replay.
- FixedMathSharp's semantic normal-response operation retains signed
  fixed-width ratios through all four final velocity deltas. An
  unrepresentable diagnostic projection or completed warm-start cache no
  longer discards an otherwise representable physical response.
- Compact response arithmetic is retained only when conservative raw-magnitude
  bounds prove the complete chained expression representable; otherwise the
  same operation uses checked full-domain FixedMathSharp primitives.
- Gravitas Release coverage is `38,695/38,695` lines,
  `12,069/12,069` branches, and `4,310/4,310` fully covered methods; the
  complete Release test project passes `3,713/3,713`.
- Gravitas `Release` and `ReleaseLean` build both `net8.0` and
  `netstandard2.1` with zero warnings or errors; `ReleaseLean` tests pass
  `3,658/3,658`.
- Collision-response short-run benchmarks retain zero managed allocation.
  The stable 16-pair cells are `9.5-21.6%` slower than the pre-Phase-2 compact
  baseline after optimization; that honest throughput signal is retained in
  the benchmark backlog rather than weakening atomic/full-domain behavior.
- Rotational moving-pair CCD reports zero managed allocation at 1, 8, and 32
  pairs; pure 3D medians are `626.9 us`, `4.980 ms`, and `21.680 ms`.
- Independent review drove closure of stale exact warm-start recovery, paired
  current-state preflight, checked impulse composition, immutable
  pre-correction contact geometry, fused relative-point-velocity cancellation,
  different-denominator projection parity, and duplicate compact friction
  evaluation.
- Final semantic-kernel review found one public contract gap: response normals
  were assumed unit length but only checked for nonzero magnitude.
  FixedMathSharp now rejects non-normalized normals and documents the matching
  signed mobility-projected axes. The reviewer found no remaining material
  Phase 2 issue.
- Phase 3 adds allocation-free exact line/disk Coulomb response to
  FixedMathSharp and consumes it from pure 2D and mixed response without
  exposing raw wide arithmetic or duplicating it in Gravitas.
- Pure 2D and mixed response now retain semantic contact anchors, recover stale
  warm starts, preflight final body deltas atomically, and use exact fallback
  for point velocity, effective mass, normal response, friction, and rotational
  CCD when compact expressions cannot be proven representable.
- Review found and closed an overflowing separating-contact edge: compact 2D
  and mixed kernels now cancel the previously applied warm start with checked
  velocity deltas rather than leaving stale velocity or forcing an unnecessary
  exact fallback.
- Phase 3 regressions cover bodyless and frozen-axis participants, mirrored
  faces, different exact denominators, diagnostic projection boundaries,
  replay, stale-cache recovery, compact/exact parity, and warmed zero-allocation
  behavior.
- FixedMathSharp's complete Release suite passes `2,615/2,615`; coverage remains
  `52,319/52,319` lines and `8,704/8,704` branches.
- Gravitas's complete Release suite passes `3,760/3,760`; coverage is
  `40,107/40,107` lines, `12,337/12,337` branches, and `4,346/4,346` methods.
- Gravitas `ReleaseLean` passes `3,705/3,705` tests. `Release` and
  `ReleaseLean` both build and package `net8.0` and `netstandard2.1` with zero
  warnings or errors.
- Independent Phase 3 review found no remaining material issue after the
  separating warm-start correction.
- Phase 4 adds the minimum public mass-property surface to FixedMathSharp:
  `FixedMassWeight`, `FixedMassPoint`, and `FixedMassPoint2d`. Wide arithmetic
  remains internal, normalized 3D rotation follows the existing rational-basis
  contract, and each public type has a dedicated source file.
- Gravitas primitive, mesh-shell, 2D, and 3D compound paths retain exact
  relative weights until final scalar mass distribution. Child centers may
  remain outside Q32.32 when their aggregate center and parallel-axis response
  are representable; true final center and tensor overflow reject atomically.
- Runtime scope remains the existing flat authored compound model. Nested
  runtime compounds were not introduced merely to exercise the semantic API.
- Compound mass distribution rounds cumulative positive-weight prefixes and
  derives each child mass from the prior prefix. This preserves the exact
  parent total without independently rounded shares oversubscribing small
  masses. Detached 2D parts whose representable areas all round to zero retain
  the existing equal-share fallback.
- Uniform mesh-shell mass properties use one FixedMathSharp wide kernel. It
  accumulates area, first moments, and origin moments in one pass, then shifts
  once to the returned center. `PhysicsMesh` retains no semantic per-face
  weights or mass-property scratch buffers.
- FixedMathSharp's complete Release suite passes `2,635/2,635`; coverage is
  `47,164/47,164` lines, `8,664/8,664` branches, and `3,434/3,434` methods.
  ReleaseLean passes `2,614/2,614`.
- Gravitas's complete Release suite passes `3,774/3,774`; coverage is
  `40,111/40,111` lines, `12,379/12,379` branches, and `4,368/4,368` methods.
  ReleaseLean passes `3,719/3,719`.
- FixedMathSharp and Gravitas `Release` and `ReleaseLean` packages build
  `net8.0` and `netstandard2.1` with zero warnings or errors.
- The short-run two-part compound inertia benchmark reports a `20.967 ms`
  mean batch for `1,024` calculations (about `20.5 us` each) with zero
  managed allocation.
- The final one-pass mesh scale-and-surface-inertia row reports `39.244 us`,
  `2.179 ms`, and `8.718 ms` at subdivisions 1, 8, and 16. That is about
  `61-65%` faster than the pre-review Phase 4 baseline. Allocations remain
  `0 B`, `4,032 B`, and `16,320 B`, confirming that the remaining
  scale-dependent signal belongs to prepared BVH rebuild rather than semantic
  mass-property arithmetic; its isolation step remains in the benchmark
  backlog.
- Independent review drove cumulative mass apportionment, detached 2D
  zero-area parity, analytic shell-tensor and translation-invariance coverage,
  preservation of scalar total/frontal-area parity, and removal of the
  duplicate triangle-area pass. No material Phase 4 finding remains.
