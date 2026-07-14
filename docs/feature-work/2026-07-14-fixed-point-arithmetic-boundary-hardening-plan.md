# Fixed-Point Arithmetic Boundary Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> `superpowers:subagent-driven-development` (recommended) or
> `superpowers:executing-plans` to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move reusable exact-arithmetic protections into FixedMathSharp, leave
only GJK and solver policy in Gravitas, and close the remaining extreme-range
correctness gaps without downstream arithmetic workarounds.

**Architecture:** FixedMathSharp first corrects `/` and `FastDiv` to share one
round-half-to-even division core. It keeps its existing saturating operators and
adds result-producing `TryAdd`/`TrySubtract` APIs that share the same private
overflow core. Explicit fused `TryMultiplyDivide` overloads retain 128-bit or
192-bit numerators until one final division and rounding step. FixedMathSharp
also owns full-domain exact projection comparison and conservative nonnegative
projection; Gravitas consumes those APIs while retaining coordinate-scaling,
solver-conditioning, and conservative-advancement policy.

**Tech Stack:** C# 11, Q32.32 `Fixed64`, FixedMathSharp `Vector2d`/`Vector3d`,
xUnit v3, BenchmarkDotNet, Gravitas 2D/3D/mixed GJK and CCD.

## Global Constraints

- Correctness and determinism precede maintainability and performance.
- Preserve the existing saturating behavior of `Fixed64` and vector operators.
- Correct `Fixed64` division midpoint behavior to round-half-to-even, matching
  multiplication, conversions, midpoint arithmetic, and the documented
  division contract. Do not preserve the current half-away-from-zero defect.
- Do not make ordinary operator chains context-sensitive. Parentheses around a
  multiplication do not retain a wide intermediate; fused arithmetic must be
  requested explicitly.
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

1. `Fixed64.operator /` and `FixedMath.FastDiv` use one shared
   round-half-to-even core. For every representable `x`, exact binary reciprocal
   identities such as `x / Fixed64.Two == x * Fixed64.Half` must hold. No such
   identity is promised when the reciprocal was already rounded, such as
   `Fixed64.One / 3`.
2. `PositiveFixed64RawHighLimit` will not remain in Gravitas. If the
   FixedMathSharp implementation still needs that boundary internally, derive
   it from `long.MaxValue >> FixedMath.SHIFT_AMOUNT_I`; do not copy the hex
   literal downstream.
3. Hex literals provide no runtime advantage over decimal literals. Use them
   only where their bit layout is the clearest representation, and accompany
   them with a decimal or invariant-based explanation. Prefer named constants,
   shifts, `uint.MaxValue`, and `long.MaxValue` derivations.
4. Saturating operators remain the convenient default. Separate `Try*` methods
   are required because an operator can return only the resulting value and
   cannot also report whether saturation occurred. The separate surface is a
   semantic requirement; avoiding the current calculate-and-reverse-check work
   is a secondary performance benefit.
5. `FixedVectorDifference` is deleted from Gravitas. Component-exact vector
   addition and subtraction belong to FixedMathSharp.
6. Full-width projection accumulation and ordering belong to FixedMathSharp.
   Gravitas owns GJK shift selection, safe-product thresholds, sweep admission,
   and conservative-advancement policy.
7. Invalid `ConvexShape` source/target kind use is an internal programming
   error, not an ordinary query miss. It must fail explicitly in every build.
8. `TryMultiplyDivide` is a distinct fused operation, not a correction to the
   existing operators. It returns `false` for a zero divisor or an
   unrepresentable final result, writes `default` on failure, and does not fail
   merely because an ordinary intermediate `Fixed64` product would saturate or
   underflow.
9. Do not add saturating, throwing, vector, or arbitrary-factor fused variants
   without a concrete package consumer. The two-factor and three-factor scalar
   overloads are the complete approved public surface.

## Current Review Findings

- `Fixed64.operator /` has claimed round-half-to-even behavior since the initial
  repository commit, but it increments every guarded quotient with a low bit of
  one. Exact midpoint magnitudes therefore round away from zero. Raw inputs
  `1`, `5`, and `9` expose `x / Two != x * Half`; raw inputs `3` and `7` agree
  only because the away-from-zero result is also the even neighbor. The April
  2026 scalar hardening corrected multiplication from half-up to true
  half-to-even but left both division implementations unchanged.
- `FixedMath.FastDiv` duplicates the same division loop and rounding defect.
  Correcting only the public operator would leave normalization and geometry
  paths with different arithmetic semantics.
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
- CCD mobility currently rejects a representable response when
  `normalComponent = 1 / 65536`, `bodyInverseMass = 1`,
  `constrainedInverseMass = Fixed64.MinIncrement`, and `responseSpeed = 1`.
  The exact velocity delta is `65536`, but the ordinary inverse-mass ratio
  saturates before the small normal component reduces it.

---

### Task 1: FixedMathSharp Division Rounding Contract

**Files:**

- Modify: `../FixedMathSharp/src/FixedMathSharp/Numerics/Scalars/Fixed64.Operators.cs`
- Modify: `../FixedMathSharp/src/FixedMathSharp/Core/FixedMath.cs`
- Modify: `../FixedMathSharp/docs/wiki/fixed64-representation.md`
- Test: `../FixedMathSharp/tests/FixedMathSharp.Tests/Numerics/Scalars/Fixed64.Tests.cs`
- Test: `../FixedMathSharp/tests/FixedMathSharp.Tests/Core/FixedMath.Tests.cs`
- Benchmark: `../FixedMathSharp/tests/FixedMathSharp.Benchmarks/Fixed64ArithmeticBenchmarks.cs`

**Interfaces:**

- Produces no new public API. `Fixed64.operator /` and `FixedMath.FastDiv`
  preserve divide-by-zero and final saturation behavior while changing exact
  midpoint results from away-from-zero to round-half-to-even.
- Produces one internal unsigned-magnitude division core and one reusable final
  rounding helper:

```csharp
internal static Fixed64 DivideMagnitude(
    ulong dividendMagnitude,
    ulong divisorMagnitude,
    bool negative);

internal static ulong RoundGuardedQuotientToEven(
    ulong guardedQuotient,
    bool hasTrailingRemainder,
    out bool overflowed);
```

- `DivideMagnitude` requires a nonzero divisor magnitude. Its callers own the
  public divide-by-zero contract and determine the result sign before entering
  the unsigned core.
- `RoundGuardedQuotientToEven` treats bit zero as the guard bit. It increments
  only when that bit is set and either trailing remainder exists or the retained
  magnitude is odd.

- [ ] **Step 1: Add the midpoint regression before changing source.**

```csharp
[Theory]
[InlineData(1L, 0L)]
[InlineData(3L, 2L)]
[InlineData(5L, 2L)]
[InlineData(-1L, 0L)]
[InlineData(-3L, -2L)]
[InlineData(-5L, -2L)]
public void DivideByTwo_MidpointsRoundToEven(long inputRaw, long expectedRaw)
{
    Fixed64 result = Fixed64.FromRaw(inputRaw) / Fixed64.Two;

    Assert.Equal(Fixed64.FromRaw(expectedRaw), result);
}
```

- [ ] **Step 2: Add exact binary reciprocal identity tests** across
      `long.MinValue`, `long.MaxValue`, zero, and raw values `-9..9`:

```csharp
Assert.Equal(value * Fixed64.Half, value / Fixed64.Two);
Assert.Equal(value * Fixed64.Quarter, value / new Fixed64(4));
Assert.Equal(value * Fixed64.Eighth, value / new Fixed64(8));
```

- [ ] **Step 3: Add below/at/above midpoint and fast-path tests.** For raw
      dividend `1`, use divisors `Fixed64.Two + Fixed64.MinIncrement`,
      `Fixed64.Two`, and `Fixed64.Two - Fixed64.MinIncrement`; require raw
      results `0`, `0`, and `1`. Repeat with a negative dividend and assert
      `FastDiv` exactly matches `/` for every positive divisor case.
- [ ] **Step 4: Run the focused tests and confirm the exact-even midpoint cases
      fail while below/above midpoint behavior remains correct.**

```powershell
dotnet test ../FixedMathSharp/tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release --filter "FullyQualifiedName~Fixed64|FullyQualifiedName~FastDiv"
```

- [ ] **Step 5: Extract the existing magnitude division loop** into
      `DivideMagnitude` and route both `/` and the positive-divisor `FastDiv`
      path through it. Keep `FastDiv`'s non-positive-divisor fallback and the
      public operator's divide-by-zero exception unchanged.
- [ ] **Step 6: Replace the false banker-rounding branch** with
      `RoundGuardedQuotientToEven`. Preserve the loop's extra guard bit, pass
      whether its final remainder is nonzero as the sticky condition, and test
      rounded carry against the asymmetric positive `long.MaxValue` and
      negative `2^63` magnitude limits before constructing the result.
- [ ] **Step 7: Add a deterministic `BigInteger` oracle in tests only.** Compare
      `/` and positive-divisor `FastDiv` against exact
      `abs(dividendRaw) * 2^32 / abs(divisorRaw)` quotient/remainder arithmetic.
      Cover seeded raw pairs plus exact ties, both signs, zero dividend,
      `long.MinValue`, `long.MaxValue`, divisor `long.MinValue`, final rounding
      carry, saturation, and divide by zero.
- [ ] **Step 8: Document the arithmetic contract** in
      `fixed64-representation.md`: multiplication and division round exact
      midpoints to the even raw value; exact binary reciprocal identities hold;
      pre-rounded reciprocals such as one third can still differ from direct
      division.
- [ ] **Step 9: Run the scalar and `FastDiv` tests in `Release` and
      `ReleaseLean`, then run exact FixedMathSharp coverage.** Require all
      guard, sticky, retained-parity, sign, carry, saturation, and zero-divisor
      branches to be covered.
- [ ] **Step 10: Benchmark `Divide` and `FastDiv`** before and after the shared
      core. Require zero allocations and no material regression; optimize the
      shared loop rather than restoring duplicated rounding implementations.
- [ ] **Step 11: Owner review checkpoint.** Leave all FixedMathSharp changes
      unstaged and provide a proposed commit message.

---

### Task 2: FixedMathSharp Exact Add And Subtract Contract

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

### Task 3: FixedMathSharp Full-Domain Projection Arithmetic

**Files:**

- Modify: `../FixedMathSharp/src/FixedMathSharp/Numerics/Scalars/Fixed64.Operators.cs`
- Modify: `../FixedMathSharp/src/FixedMathSharp/Numerics/Vectors/Vector2d.Statics.cs`
- Modify: `../FixedMathSharp/src/FixedMathSharp/Numerics/Vectors/Vector3d.Statics.cs`
- Test: `../FixedMathSharp/tests/FixedMathSharp.Tests/Numerics/Vectors/Vector2d.Tests.cs`
- Test: `../FixedMathSharp/tests/FixedMathSharp.Tests/Numerics/Vectors/Vector3d.Tests.cs`
- Benchmark: `../FixedMathSharp/tests/FixedMathSharp.Benchmarks/Vector2dBenchmarks.cs`
- Benchmark: `../FixedMathSharp/tests/FixedMathSharp.Benchmarks/Vector3dBenchmarks.cs`

**Interfaces:**

- Consumes: Task 2 arithmetic and the existing private full-width multiplication
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

### Task 4: FixedMathSharp Fused Multiply-Divide

**Files:**

- Modify: `../FixedMathSharp/src/FixedMathSharp/Numerics/Scalars/Fixed64.Operators.cs`
- Test: `../FixedMathSharp/tests/FixedMathSharp.Tests/Numerics/Scalars/Fixed64.Tests.cs`
- Benchmark: `../FixedMathSharp/tests/FixedMathSharp.Benchmarks/Fixed64ArithmeticBenchmarks.cs`

**Interfaces:**

- Consumes: Task 1's verified `RoundGuardedQuotientToEven` policy plus the
  existing `AbsToUInt64` and `Multiply64To128` arithmetic core.
- Produces:

```csharp
public partial struct Fixed64
{
    public static bool TryMultiplyDivide(
        Fixed64 left,
        Fixed64 right,
        Fixed64 divisor,
        out Fixed64 result);

    public static bool TryMultiplyDivide(
        Fixed64 first,
        Fixed64 second,
        Fixed64 third,
        Fixed64 divisor,
        out Fixed64 result);
}
```

- For raw inputs `A`, `B`, `C`, and `D`, the two-factor overload rounds
  `(A * B) / D` to the nearest raw integer and the three-factor overload rounds
  `(A * B * C) / (D * 2^32)` to the nearest raw integer. Both use
  round-half-to-even exactly once.
- A zero divisor or final magnitude outside `long.MinValue..long.MaxValue`
  returns `false` with `result = default`. Exact `long.MinValue` remains a
  successful negative result.

- [ ] **Step 1: Add two-factor red tests** using raw-value operands for all sign
      combinations, exact zero, exact `MinValue`/`MaxValue`, divisor zero,
      positive and negative final overflow, default failure output, and
      half-to-even cases `1 * 1 / 2 -> 0` and `3 * 1 / 2 -> 2` raw units.
- [ ] **Step 2: Add the rescued-intermediate regressions.** Prove that
      `65536 * 65536 / 65536` returns `65536` although the ordinary product
      saturates, and that `MinIncrement * Half / Half` returns `MinIncrement`
      although the ordinary product rounds to zero.
- [ ] **Step 3: Run the focused scalar tests and confirm both overloads are
      missing.**

```powershell
dotnet test ../FixedMathSharp/tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release --filter "FullyQualifiedName~Fixed64"
```

- [ ] **Step 4: Implement the two-factor overload** with the existing unsigned
      64-by-64-to-128 multiplier and one allocation-free unsigned divide with
      quotient/remainder. Feed its guard and sticky state through Task 1's
      rounding helper, then apply the result sign after comparing the rounded
      magnitude against the asymmetric positive and negative limits.
- [ ] **Step 5: Add three-factor red tests** matching the two-factor sign,
      boundary, zero-divisor, default-output, and half-to-even coverage. Add the
      CCD regression with raw values `2^16`, `2^32`, `2^32`, and `1`; require
      the representable raw result `2^48` (`65536`).
- [ ] **Step 6: Implement the three-factor overload** by extending the existing
      unsigned product to three 64-bit words. Divide that numerator by the
      64-bit divisor, retain the quotient's low 32 discarded bits plus the
      division remainder, reduce them to one guard bit plus sticky state, and
      use Task 1's rounding helper to produce the final Q32.32 raw result once.
      Do not use `BigInteger`, floating point, `Int128`, or a public wide numeric
      type in runtime code.
- [ ] **Step 7: Add deterministic oracle coverage** in the test project using
      `BigInteger` only as an independently computed reference. Cover fixed
      boundary vectors plus a seeded set of raw inputs for both overloads,
      including cases where ordinary grouping saturates or underflows but the
      fused result fits.
- [ ] **Step 8: Run the focused tests in `Release` and `ReleaseLean`, then run
      exact FixedMathSharp coverage.** Every divisor-zero, sign, carry,
      quotient-overflow, rounding, and `long.MinValue` branch must be reachable.
- [ ] **Step 9: Benchmark both overloads** against their ordinary operator
      chains for common in-range inputs and rescued extreme inputs. Require zero
      allocations; optimize a proven common path without weakening the
      single-rounding contract.
- [ ] **Step 10: Owner review checkpoint.** Leave all FixedMathSharp changes
      unstaged and provide a proposed commit message.

---

### Task 5: Gravitas Consumes FixedMathSharp Arithmetic

**Files:**

- Delete: `src/Gravitas/Support/FixedVectorDifference.cs`
- Delete: `src/Gravitas/CollisionHandling/Detection/3D/ConvexSupportProjection.cs`
- Modify: `src/Gravitas/CollisionHandling/Continuous/ContinuousCollisionImpulsePolicy.cs`
- Modify: `src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.Dynamic.cs`
- Modify: `src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.Kinematic.cs`
- Modify: `src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Dynamic.cs`
- Modify: `src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Kinematic.cs`
- Modify: every current caller returned by:

```powershell
rg -l "FixedVectorDifference|ConvexSupportProjection" src/Gravitas tests/Gravitas.Tests
```

- Modify: `src/Gravitas/Queries/GjkSimplexScale.cs`
- Test: `tests/Gravitas.Tests/CollisionHandling/ContinuousCollisionMathTests.cs`
- Test: `tests/Gravitas.Tests/CollisionHandling/ContinuousCollisionPolicyTests.cs`
- Test: `tests/Gravitas.Tests/CollisionHandling/ConvexColliderSupportTests.cs`
- Test: `tests/Gravitas.Tests/Queries/GjkSimplexScaleTests.cs`
- Test: relevant 2D, 3D, and mixed query/CCD suites already exercising those
  callers.

**Interfaces:**

- Consumes: Tasks 1 through 4.
- Produces: no new Gravitas arithmetic API.
- Replaces the internal `ResolveVelocityDelta` overloads with component-atomic
  `TryResolveVelocityDelta` overloads. Failure writes `default` and occurs
  before either collision participant is mutated.

```csharp
internal static bool TryResolveVelocityDelta(
    Vector2d normal,
    Fixed64 responseSpeed,
    Fixed64 bodyInverseMass,
    Fixed64 constrainedInverseMass,
    out Vector2d velocityDelta);

internal static bool TryResolveVelocityDelta(
    Vector3d normal,
    Fixed64 responseSpeed,
    Fixed64 bodyInverseMass,
    Fixed64 constrainedInverseMass,
    out Vector3d velocityDelta);
```

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
- [ ] **Step 9: Delete the arithmetic-driven `IsResolvableMobility` cutoff** and
      replace both `ResolveVelocityDelta` overloads with component-atomic
      `TryResolveVelocityDelta` methods composed from the three-factor
      `TryMultiplyDivide` overload. For two-body response, compute both velocity
      deltas successfully before applying either one. Do not retain a
      solver-conditioning threshold without separate physical evidence.
- [ ] **Step 10: Add 2D and 3D CCD regressions** for the `65536` finite response,
      zero inverse mass, final-result overflow rejection without partial body
      mutation, and unchanged ordinary mobility. Run the focused
      continuous-collision policy and sweep suites.
- [ ] **Step 11: Owner review checkpoint.** Leave Gravitas implementation changes
      unstaged and provide a proposed commit message.

---

### Task 6: Remove Release-Only Assertion Behavior

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

### Task 7: Cross-Stack Validation And Documentation Closure

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
      division, fused multiply-divide, vector `Try*`, magnitude/normalization,
      and projection. Record medians and allocations.
- [ ] **Step 3: Validate SwiftCollections, GridForge, and Gravitas** through
      explicit local project references in each library, test, and benchmark
      project that requires them.
- [ ] **Step 4: Run Gravitas full `Release`, `ReleaseLean`, exact coverage,
      replay, and the existing convex-sweep benchmark rows.** Require 100% line
      and branch coverage and zero allocation regression.
- [ ] **Step 5: Update the issue tracker resolution record** with the arithmetic
      ownership correction, division-rounding RCA, exact reciprocal identities,
      odd-raw GJK regression, final test counts, coverage artifact, and benchmark
      evidence. Remove the previous claim that the staged downstream arithmetic
      was already the final ownership boundary.
- [ ] **Step 6: Move this plan to `docs/feature-work/done/`** and update the
      overview only after all package-local and downstream gates pass.
- [ ] **Step 7: Request independent final review** of correctness,
      determinism, API ownership, hot-path cost, and documentation consistency.

## Exit Criteria

- Gravitas contains no generic raw fixed-point overflow detector, full-width
  multiplier, or exact projection accumulator.
- `FixedVectorDifference`, `ConvexSupportProjection`, and
  `PositiveFixed64RawHighLimit` no longer exist in Gravitas.
- `Fixed64.operator /` and `FixedMath.FastDiv` share one magnitude-division and
  rounding core, match the `BigInteger` oracle, and round exact midpoints to the
  even raw value for both signs.
- `x / Two == x * Half`, `x / 4 == x * Quarter`, and
  `x / 8 == x * Eighth` hold across the tested raw domain including
  `long.MinValue` and `long.MaxValue`.
- Existing operators remain saturating; `Try*` methods report exactness without
  throwing or exposing partial results.
- Both fused multiply-divide overloads retain their complete numerator until one
  final round-half-to-even conversion, report only final representability, and
  allocate no managed memory.
- Gravitas no longer rejects the finite `65536` CCD response because an
  intermediate inverse-mass ratio would saturate.
- Projection comparison is correct for the complete `Fixed64` component domain,
  not only normalized axes.
- The odd-raw negative-expansion shift regression passes without saturation.
- No `Debug.Assert` remains in Gravitas runtime source.
- FixedMathSharp and Gravitas retain their current full coverage gates, and all
  measured hot paths remain allocation-free.
- All temporary local project references remain unstaged and are removed before
  package-only release validation.
