# Fixed-Point Arithmetic Boundary Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> `superpowers:subagent-driven-development` (recommended) or
> `superpowers:executing-plans` to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move reusable exact-arithmetic protections into FixedMathSharp, leave
only GJK and sweep policy in Gravitas, and close the remaining extreme-range
correctness gaps without downstream arithmetic workarounds.

**Architecture:** FixedMathSharp keeps its existing saturating operators and
adds result-producing `TryAdd`/`TrySubtract` APIs that share the same private
overflow core. FixedMathSharp also owns full-domain exact projection comparison
and conservative nonnegative projection; Gravitas consumes those APIs while
retaining coordinate-scaling and conservative-advancement policy.

**Tech Stack:** C# 11, Q32.32 `Fixed64`, FixedMathSharp `Vector2d`/`Vector3d`,
xUnit v3, BenchmarkDotNet, Gravitas 2D/3D/mixed GJK and CCD.

## Global Constraints

- Correctness and determinism precede maintainability and performance.
- Preserve the existing saturating behavior of `Fixed64` and vector operators.
- Do not introduce floating-point arithmetic or target-specific `Int128`
  behavior; FixedMathSharp must continue targeting `netstandard2.1` and
  `net8.0` consistently.
- Do not add public `CanAdd`, `CanSubtract`, or combinatorial
  `CanSubtractThen*` APIs. Use result-producing `Try*` operations and compose
  them where an algorithm needs multiple terms.
- Do not duplicate raw overflow detection or 64-by-64 full-width
  multiplication in Gravitas.
- Do not use `Debug.Assert` as a runtime correctness guard.
- Use temporary local project references in the source, test, and benchmark
  projects of each downstream solution. Leave every project-reference change
  unstaged and uncommitted.
- Leave all implementation and documentation changes unstaged for owner review;
  the repository owner will commit accepted work.
- Preserve 100% reachable line, branch, and method coverage in FixedMathSharp
  and 100% line and branch coverage in Gravitas.

---

## Decisions Locked By This Plan

1. `PositiveFixed64RawHighLimit` will not remain in Gravitas. If the
   FixedMathSharp implementation still needs that boundary internally, derive
   it from `long.MaxValue >> FixedMath.SHIFT_AMOUNT_I`; do not copy the hex
   literal downstream.
2. Hex literals provide no runtime advantage over decimal literals. Use them
   only where their bit layout is the clearest representation, and accompany
   them with a decimal or invariant-based explanation. Prefer named constants,
   shifts, `uint.MaxValue`, and `long.MaxValue` derivations.
3. Saturating operators remain the convenient default. Separate `Try*` methods
   are required because an operator can return only the resulting value and
   cannot also report whether saturation occurred. The separate surface is a
   semantic requirement; avoiding the current calculate-and-reverse-check work
   is a secondary performance benefit.
4. `FixedVectorDifference` is deleted from Gravitas. Component-exact vector
   addition and subtraction belong to FixedMathSharp.
5. Full-width projection accumulation and ordering belong to FixedMathSharp.
   Gravitas owns GJK shift selection, safe-product thresholds, sweep admission,
   and conservative-advancement policy.
6. Invalid `ConvexShape` source/target kind use is an internal programming
   error, not an ordinary query miss. It must fail explicitly in every build.

## Current Review Findings

- `FixedVectorDifference` detects saturation by performing a vector operation
  and then an inverse operation. The policy is useful, but the implementation
  belongs at the scalar/vector arithmetic source of truth.
- `GjkSimplexScale` duplicates raw signed-overflow predicates already inherent
  in `Fixed64` operator implementation.
- `ConvexSupportProjection` duplicates FixedMathSharp's 64-by-64-to-128
  multiplication and exposes only a conditionally safe two-word signed sum.
  Two or three full-domain difference-products require an accumulator wider
  than signed 128 bits.
- `SelectThreeTermShift` under-bounds negative odd-raw expansion components.
  For `point.X = Fixed64.MaxValue`, target X bounds at `Fixed64.MinValue`, and
  `expansionRadius = Fixed64.MinIncrement`, the positive radius bound becomes
  zero after shift one while an actual negative expansion component remains
  negative one raw unit. The selector approves a working shift whose third
  subtraction still saturates.
- The unrestricted `int shift` helpers silently treat values outside `0..2`
  inconsistently. Every public-internal entry must reject invalid values before
  shifting.
- Three `Debug.Assert` calls are the only guards against invalid release-state
  geometry behavior.

---

### Task 1: FixedMathSharp Exact Add And Subtract Contract

**Files:**

- Modify: `../FixedMathSharp/src/FixedMathSharp/Numerics/Scalars/Fixed64.Operators.cs`
- Modify: `../FixedMathSharp/src/FixedMathSharp/Numerics/Vectors/Vector2d.Statics.cs`
- Modify: `../FixedMathSharp/src/FixedMathSharp/Numerics/Vectors/Vector3d.Statics.cs`
- Test: `../FixedMathSharp/tests/FixedMathSharp.Tests/Numerics/Scalars/Fixed64.Tests.cs`
- Test: `../FixedMathSharp/tests/FixedMathSharp.Tests/Numerics/Vectors/Vector2d.Tests.cs`
- Test: `../FixedMathSharp/tests/FixedMathSharp.Tests/Numerics/Vectors/Vector3d.Tests.cs`
- Benchmark: `../FixedMathSharp/tests/FixedMathSharp.Benchmarks/Fixed64ArithmeticBenchmarks.cs`
- Benchmark: `../FixedMathSharp/tests/FixedMathSharp.Benchmarks/Vector2dBenchmarks.cs`
- Benchmark: `../FixedMathSharp/tests/FixedMathSharp.Benchmarks/Vector3dBenchmarks.cs`

**Interfaces:**

- Produces:

```csharp
public partial struct Fixed64
{
    public static bool TryAdd(Fixed64 left, Fixed64 right, out Fixed64 result);
    public static bool TrySubtract(Fixed64 left, Fixed64 right, out Fixed64 result);
}

public partial struct Vector2d
{
    public static bool TryAdd(Vector2d left, Vector2d right, out Vector2d result);
    public static bool TrySubtract(Vector2d left, Vector2d right, out Vector2d result);
}

public partial struct Vector3d
{
    public static bool TryAdd(Vector3d left, Vector3d right, out Vector3d result);
    public static bool TrySubtract(Vector3d left, Vector3d right, out Vector3d result);
}
```

- On success, `result` is the exact Q32.32 result.
- On failure, `result` is `default`; no partially saturated vector is exposed.
- Existing operators preserve their current saturated result.

- [ ] **Step 1: Add scalar red tests** covering ordinary values, exact
      `MinValue`/`MaxValue` boundaries, positive and negative overflow, default
      failure output, and unchanged saturating operator results.
- [ ] **Step 2: Run the focused scalar tests and confirm the new API tests fail.**

```powershell
dotnet test ../FixedMathSharp/tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release --filter "FullyQualifiedName~Fixed64"
```

- [ ] **Step 3: Extract one private raw add/subtract overflow core** and route
      both the existing operators and new public `Try*` methods through it.
      Mark the small hot methods for aggressive inlining; do not maintain two
      independent bit predicates.
- [ ] **Step 4: Add component-atomic vector `Try*` methods** using the scalar
      methods. Set the vector result only after every component succeeds.
- [ ] **Step 5: Add vector boundary tests** for a failing first, middle, and
      final component plus ordinary exact translations and differences.
- [ ] **Step 6: Run the scalar and vector tests in `Release` and
      `ReleaseLean`.**
- [ ] **Step 7: Benchmark scalar and vector `Try*` success paths** against the
      current calculate-and-inverse-check pattern. Require zero allocations and
      no material hot-path regression; optimize the shared raw core rather than
      adding a second API shape.
- [ ] **Step 8: Owner review checkpoint.** Leave all FixedMathSharp changes
      unstaged and provide a proposed commit message.

---

### Task 2: FixedMathSharp Full-Domain Projection Arithmetic

**Files:**

- Modify: `../FixedMathSharp/src/FixedMathSharp/Numerics/Scalars/Fixed64.Operators.cs`
- Modify: `../FixedMathSharp/src/FixedMathSharp/Numerics/Vectors/Vector2d.Statics.cs`
- Modify: `../FixedMathSharp/src/FixedMathSharp/Numerics/Vectors/Vector3d.Statics.cs`
- Test: `../FixedMathSharp/tests/FixedMathSharp.Tests/Numerics/Vectors/Vector2d.Tests.cs`
- Test: `../FixedMathSharp/tests/FixedMathSharp.Tests/Numerics/Vectors/Vector3d.Tests.cs`
- Benchmark: `../FixedMathSharp/tests/FixedMathSharp.Benchmarks/Vector2dBenchmarks.cs`
- Benchmark: `../FixedMathSharp/tests/FixedMathSharp.Benchmarks/Vector3dBenchmarks.cs`

**Interfaces:**

- Consumes: Task 1 arithmetic and the existing private full-width multiplication
  implementation.
- Produces:

```csharp
public partial struct Vector2d
{
    public static int CompareProjection(
        Vector2d candidate,
        Vector2d current,
        Vector2d direction);
}

public partial struct Vector3d
{
    public static int CompareProjection(
        Vector3d candidate,
        Vector3d current,
        Vector3d direction);

    public static Fixed64 ProjectNonNegativeDifference(
        Vector3d target,
        Vector3d source,
        Vector3d direction);
}
```

- `CompareProjection` returns the exact sign of
  `(candidate - current) dot direction` without intermediate saturation and
  without requiring a normalized direction.
- `ProjectNonNegativeDifference` accumulates before Q32.32 conversion, floors
  positive fractional remainder to preserve a conservative lower bound, clamps
  negative results to zero, and saturates only the final positive result.

- [ ] **Step 1: Add red tests** for positive, negative, and cancelling
      projections across the complete component range, including a three-term
      positive sum and negative sum that overflow a signed 128-bit accumulator.
- [ ] **Step 2: Add red conservative-projection tests** for zero, negative,
      one-unit, fractional-floor, and final-result saturation behavior.
- [ ] **Step 3: Run the focused vector tests and confirm the new APIs fail.**
- [ ] **Step 4: Implement one internal three-word signed accumulator** inside
      `Fixed64` so vector types reuse the existing full-width multiplier. Sign
      extend each 128-bit product into the third word, propagate carries across
      all words, and decide sign/order from the complete sum.
- [ ] **Step 5: Convert the positive Q64.64 sum to Q32.32 only after checking
      the complete high words.** If a local high-word boundary is needed, derive
      it from `long.MaxValue >> FixedMath.SHIFT_AMOUNT_I` and document the
      invariant rather than its hexadecimal spelling.
- [ ] **Step 6: Run the focused tests in `Release` and `ReleaseLean`.**
- [ ] **Step 7: Benchmark ordinary support-projection comparisons and extreme
      inputs.** Require zero allocations and compare the same fixture before
      and after the wide-accumulator change.
- [ ] **Step 8: Owner review checkpoint.** Leave all FixedMathSharp changes
      unstaged and provide a proposed commit message.

---

### Task 3: Gravitas Consumes FixedMathSharp Arithmetic

**Files:**

- Delete: `src/Gravitas/Support/FixedVectorDifference.cs`
- Delete: `src/Gravitas/CollisionHandling/Detection/3D/ConvexSupportProjection.cs`
- Modify: every current caller returned by:

```powershell
rg -l "FixedVectorDifference|ConvexSupportProjection" src/Gravitas tests/Gravitas.Tests
```

- Modify: `src/Gravitas/Queries/GjkSimplexScale.cs`
- Test: `tests/Gravitas.Tests/CollisionHandling/ContinuousCollisionMathTests.cs`
- Test: `tests/Gravitas.Tests/CollisionHandling/ConvexColliderSupportTests.cs`
- Test: `tests/Gravitas.Tests/Queries/GjkSimplexScaleTests.cs`
- Test: relevant 2D, 3D, and mixed query/CCD suites already exercising those
  callers.

**Interfaces:**

- Consumes: Tasks 1 and 2.
- Produces: no new Gravitas arithmetic API.

- [ ] **Step 1: Add the odd-raw negative-expansion regression** using
      `MaxValue`, `MinValue`, and `Fixed64.MinIncrement`. Assert that the
      selected working shift makes the actual three-term support difference
      exact.
- [ ] **Step 2: Run `GjkSimplexScaleTests` and confirm the regression fails.**
- [ ] **Step 3: Replace endpoint and translation checks** with
      `Vector2d.TryAdd`/`TrySubtract` and `Vector3d.TryAdd`/`TrySubtract`, then
      delete `FixedVectorDifference` and its direct helper tests.
- [ ] **Step 4: Replace support ordering and conservative projection** with the
      FixedMathSharp vector APIs, then delete `ConvexSupportProjection` and its
      duplicated multiplier.
- [ ] **Step 5: Remove raw overflow predicates from `GjkSimplexScale`.** Keep
      the private bounds predicates, but implement them by composing FixedMathSharp
      `Try*` operations.
- [ ] **Step 6: Preserve GJK-owned power-of-two coordinate policy.** Use the
      existing `Fixed64 >>` operator for components. For the nonnegative radius
      bound, reconstruct the shifted value and add `Fixed64.MinIncrement` when
      discarded raw bits require a ceiling; this must conservatively cover
      negative arithmetic shifts.
- [ ] **Step 7: Reject shifts outside `0..2` explicitly** before any shift or
      restore operation. Do not add a new public enum or abstraction for an
      internal three-value implementation detail.
- [ ] **Step 8: Run focused GJK, support, query, and CCD tests in `Release` and
      `ReleaseLean`.**
- [ ] **Step 9: Owner review checkpoint.** Leave Gravitas implementation changes
      unstaged and provide a proposed commit message.

---

### Task 4: Remove Release-Only Assertion Behavior

**Files:**

- Modify: `src/Gravitas/Queries/3D/Sweeps/ConvexShape.cs`
- Modify: `src/Gravitas/CollisionHandling/Detection/3D/ConvexColliderSupport.cs`
- Test: `tests/Gravitas.Tests/Queries/GravitasQuery3DServiceSweepTests.cs`
- Test: `tests/Gravitas.Tests/CollisionHandling/ConvexColliderSupportTests.cs`

**Interfaces:**

- Produces: identical debug and release behavior for invalid internal shape
  use; no new public API.

- [ ] **Step 1: Add tests** proving triangle shapes cannot be used as sweep
      sources and circle slabs cannot be used as target-normal providers.
- [ ] **Step 2: Replace both tagged-union `Debug.Assert` guards** with
      `SwiftThrowHelper.ThrowIfTrue` and remove `System.Diagnostics`.
- [ ] **Step 3: Make zero-direction cone support deterministic** by selecting
      `Vector3d.Right`, matching the existing general support-mapping fallback,
      then remove the cone `Debug.Assert`.
- [ ] **Step 4: Run the focused tests in `Release` and `ReleaseLean`.**
- [ ] **Step 5: Confirm `rg -n "Debug\\.Assert" src/Gravitas` returns no
      matches.**

---

### Task 5: Cross-Stack Validation And Documentation Closure

**Files:**

- Modify: `docs/feature-work/issue-tracker.md`
- Modify: `docs/feature-work/feature-work-overview.md`
- Modify: `../FixedMathSharp/docs/complexity-exceptions.md` only if a new method
  exceeds the registered complexity threshold after coverage/CRAP analysis.
- Modify: relevant FixedMathSharp XML documentation for every new public API.

- [ ] **Step 1: Run full FixedMathSharp `Release` and `ReleaseLean` tests,
      builds, coverage, and CRAP analysis.** Update the complexity exception
      register only from the fresh report.
- [ ] **Step 2: Run the focused FixedMathSharp benchmark rows** for scalar
      arithmetic, vector `Try*`, magnitude/normalization, and projection. Record
      medians and allocations.
- [ ] **Step 3: Validate SwiftCollections, GridForge, and Gravitas** through
      explicit local project references in each library, test, and benchmark
      project that requires them.
- [ ] **Step 4: Run Gravitas full `Release`, `ReleaseLean`, exact coverage,
      replay, and the existing convex-sweep benchmark rows.** Require 100% line
      and branch coverage and zero allocation regression.
- [ ] **Step 5: Update the issue tracker resolution record** with the arithmetic
      ownership correction, odd-raw GJK regression, final test counts, coverage
      artifact, and benchmark evidence. Remove the previous claim that the
      staged downstream arithmetic was already the final ownership boundary.
- [ ] **Step 6: Move this plan to `docs/feature-work/done/`** and update the
      overview only after all package-local and downstream gates pass.
- [ ] **Step 7: Request independent final review** of correctness,
      determinism, API ownership, hot-path cost, and documentation consistency.

## Exit Criteria

- Gravitas contains no generic raw fixed-point overflow detector, full-width
  multiplier, or exact projection accumulator.
- `FixedVectorDifference`, `ConvexSupportProjection`, and
  `PositiveFixed64RawHighLimit` no longer exist in Gravitas.
- Existing operators remain saturating; `Try*` methods report exactness without
  throwing or exposing partial results.
- Projection comparison is correct for the complete `Fixed64` component domain,
  not only normalized axes.
- The odd-raw negative-expansion shift regression passes without saturation.
- No `Debug.Assert` remains in Gravitas runtime source.
- FixedMathSharp and Gravitas retain their current full coverage gates, and all
  measured hot paths remain allocation-free.
- All temporary local project references remain unstaged and are removed before
  package-only release validation.
