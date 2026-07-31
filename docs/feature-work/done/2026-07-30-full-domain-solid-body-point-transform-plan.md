# Full-Domain SolidBody Point Transform Implementation Plan

**Status:** Complete.

> **For agentic workers:** Use `superpowers:test-driven-development` while
> changing behavior, `superpowers:requesting-code-review` before closure, and
> `superpowers:verification-before-completion` before reporting completion.
> Checkboxes are the living progress record.

**Goal:** Make 3D body-local/world point conversion exact-or-false across the
complete representable `Fixed64` domain without changing the authoritative
body-pose or host hierarchy-scale contract.

**Architecture:** Reuse FixedMathSharp's existing exact forward scaled-point
transform. Add its missing inverse counterpart so world subtraction, inverse
rotation, and component division remain one rational operation until final
round-half-to-even materialization. `SolidBody` exposes matching throwing and
`Try*` pairs; the throwing methods delegate to the nonthrowing contract.

**Tech Stack:** C# 11, Q32.32 `Fixed64`, FixedMathSharp fixed-width wide
arithmetic, Gravitas `SolidBody`, xUnit v3, BenchmarkDotNet, `Release`, and
`ReleaseLean`.

## Global Constraints

- Determinism, performance, maintainability, and correctness are all release
  gates.
- Preserve `Position3d`, `Rotation`, and the host transform's canonical
  hierarchy-derived `LossyScale` as the complete body point-transform contract.
- Never use collider dimensions as transform scale.
- Never silently saturate an intermediate or final coordinate.
- A failed `Try*` operation returns `false` and a zero result atomically.
- Throwing convenience methods use `InvalidOperationException` when scale,
  inverse, or the final coordinate is unavailable.
- Reuse the existing exact forward primitive; do not add a second forward
  implementation.
- Keep raw wide types out of public signatures.
- Keep warmed point conversion allocation-free.
- Retain 100% reachable line, branch, and method coverage in FixedMathSharp and
  Gravitas without hollow API-shape tests, exclusions, or zombie branches.
- Leave source, test, benchmark, and documentation changes unstaged and
  uncommitted.

## Locked Design

1. FixedMathSharp adds
   `FixedQuaternion.TryInverseTransformScaledPoint(origin, worldPoint, scale,
   out localPoint)`.
2. The inverse evaluates
   `InverseRotate(worldPoint - origin) / scale` as one exact rational expression
   per final local coordinate. Zero scale and a zero quaternion fail
   atomically.
3. Generic scaled 3D transform mechanics move from the oriented-box reducer to
   a focused numerics-wide owner alongside the new inverse. Existing public
   `TryTransformScaledPoint` and `Vector3d.TryComposeScaledLocalPoints` retain
   their signatures and behavior; no forwarding-only layer remains.
4. `SolidBody.TryTransformPoint(...)` first obtains strict canonical host scale,
   then delegates to the existing FixedMathSharp forward primitive.
5. `SolidBody.TryInverseTransformPoint(...)` obtains the same strict scale and
   delegates to the new inverse primitive.
6. Existing `TransformPoint(...)` and `InverseTransformPoint(...)` remain the
   concise convenience surface and throw when their corresponding `Try*`
   operation fails.
7. No `SolidBody2D` point-transform API is added. There is no current public
   consumer or defective counterpart; adding one would be speculative. Existing
   2D internal transforms already use exact FixedMathSharp `Try*` operations.

## Alternatives Rejected

- **Matrix construction plus `TryTransformAffinePoint`:** the matrix stores
  already-rounded scale/rotation coefficients and general matrix inversion
  introduces a broader failure contract than the explicit TRS operation.
- **Gravitas-only wide helper:** scale/rotation/translation point conversion is
  reusable deterministic mathematics and the exact forward half already lives
  in FixedMathSharp.
- **Only `Try*` methods:** forcing every ordinary adapter call to handle a
  boolean would make the public surface needlessly cumbersome.
- **Only throwing methods:** expected domain-boundary and singular-transform
  checks need a nonthrowing path.

---

## Phase 0: Root Cause And Contract

- [x] Trace both public `SolidBody` point helpers and every repository caller.
- [x] Confirm the old collider-dimension defect remains fixed.
- [x] Confirm the forward chain saturates before final cancellation.
- [x] Confirm the inverse chain narrows subtraction and rotation before final
      component division.
- [x] Confirm FixedMathSharp already owns the exact forward scaled-point
      primitive.
- [x] Confirm no exact inverse scaled-point primitive exists.
- [x] Confirm pure 2D has no public body point-transform counterpart.
- [x] Obtain owner approval for throwing and `Try*` pairs.

**Phase 0 result:** The reusable forward operation already exists. The missing
inverse and Gravitas adoption are the complete root-cause fix.

## Phase 1: FixedMathSharp Exact Inverse

**Files:**

- Modify:
  `../FixedMathSharp/src/FixedMathSharp/Numerics/Rotations/FixedQuaternion.ScaledTransform.cs`
- Create:
  `../FixedMathSharp/src/FixedMathSharp/Numerics/Wide/WideVector3dTransform.cs`
- Simplify:
  `../FixedMathSharp/src/FixedMathSharp/Geometry/Wide/OrientedBox/WideOrientedBox.cs`
  and
  `../FixedMathSharp/src/FixedMathSharp/Geometry/Wide/OrientedBox/WideOrientedBox.Materialization.cs`
- Modify:
  `../FixedMathSharp/src/FixedMathSharp/Numerics/Vectors/Vector3d.Statics.cs`
- Modify:
  `../FixedMathSharp/tests/FixedMathSharp.Tests/Numerics/ScaledCompositeTransform.Tests.cs`
- Modify:
  `../FixedMathSharp/tests/FixedMathSharp.Benchmarks/QuaternionBenchmarks.cs`
- Update:
  `../FixedMathSharp/docs/MIGRATION.md`

- [x] Add failing ordinary, anisotropic, mirrored-scale, scalar-face
      cancellation, singular-scale, zero-quaternion, true final-overflow,
      round-trip, and warmed-allocation tests.
- [x] Run the focused tests and confirm failure occurs because the inverse API
      is absent.
- [x] Move only the reusable scaled 3D mechanics to the numerics-wide owner.
- [x] Implement the exact inverse with one final rounding per coordinate.
- [x] Preserve existing forward and composed-local results bit-for-bit.
- [x] Add benchmark coverage for ordinary and exact-fallback inverse transforms.
- [x] Pass focused and complete FixedMathSharp tests in `Release` and
      `ReleaseLean`.
- [x] Restore exact 100% FixedMathSharp coverage.

**Phase 1 result:** FixedMathSharp now owns one generic scaled 3D transform
kernel and exposes the missing exact inverse. Full-domain subtraction,
projection, and scale division remain wide until one final round-half-to-even
conversion per coordinate. The complete suites pass 2,595 `Release` and 2,574
`ReleaseLean` tests at 49,865/49,865 lines, 8,426/8,426 branches, and
3,304/3,304 methods. Both focused inverse benchmark rows allocate zero managed
bytes.

## Phase 2: Gravitas Public Adoption

**Files:**

- Modify:
  `src/Gravitas/Core/3D/SolidBody.cs`
- Modify:
  `tests/Gravitas.Tests/Core/SolidBodyIntegrationTests.cs`
- Modify or create a focused benchmark row under:
  `tests/Gravitas.Benchmarks`
- Update:
  `docs/wiki/HOST_INTEGRATION.md`

- [x] Add failing regressions for forward cancellation, inverse cancellation,
      anisotropic mirrored scale, singular scale, unavailable hierarchy scale,
      true final overflow, ordinary parity, round trips, and throwing wrappers.
- [x] Confirm the focused tests fail against the chained implementation.
- [x] Add `TryTransformPoint` and `TryInverseTransformPoint`.
- [x] Route the existing convenience methods through the `Try*` contract and
      `SwiftThrowHelper`.
- [x] Keep strict host hierarchy-scale admission and zero allocation.
- [x] Pass focused SolidBody, host-transform, and replay-sensitive tests.
- [x] Restore exact 100% Gravitas coverage.

**Phase 2 result:** `SolidBody` now offers nonthrowing and throwing point
conversion pairs over the same exact operations. Failed calls return zero
atomically; convenience calls throw for unavailable hierarchy scale, singular
inverse scale, or a truly unrepresentable final point. Root transforms retain
the existing direct-scale fast path. The full Gravitas `Release` coverage run
passes 3,866 tests at 43,664/43,664 lines, 12,775/12,775 branches, and
4,503/4,503 methods; `ReleaseLean` passes 3,811 tests. Ordinary and
full-domain round trips measure 3.520 us and 2.484 us respectively at zero
managed allocation in ShortRun.

## Phase 3: Closure

- [x] Run both repositories' full `Release` and `ReleaseLean` suites.
- [x] Build both target frameworks and package configurations without warnings.
- [x] Run focused point-transform benchmarks and allocation assertions.
- [x] Audit docs and public XML comments for exact failure semantics.
- [x] Audit pure 2D parity and record the no-new-API decision.
- [x] Request independent cross-stack code review and resolve all findings.
- [x] Move this plan to `docs/feature-work/done`.
- [x] Move the issue from the ordered queue to resolved history with verification
      evidence.
- [x] Update `feature-work-overview.md`.

**Phase 3 result:** Standard and Lean multi-target package builds are
warning-free, both full suites and exact coverage gates pass, and all four
focused benchmark rows allocate zero managed bytes. Host and migration docs
state the exact failure contract. Pure 2D intentionally retains its existing
internal exact transforms without adding an unused body-level API. Independent
cross-stack review confirmed the arithmetic derivation, ownership, API,
failure, test, and performance contracts and requested only complete generated
XML failure documentation, generic local-space terminology, and final plan
movement; all findings are resolved.
