# Full-Domain SolidBody Point Transform Implementation Plan

**Status:** Complete, including the approved ownership and 2D-parity fast
follow.

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:executing-plans`
> to implement this plan task-by-task in the owner-approved current `develop`
> worktrees. Use `superpowers:test-driven-development` while changing behavior,
> `superpowers:requesting-code-review` before closure, and
> `superpowers:verification-before-completion` before reporting completion.
> Checkboxes are the living progress record.

**Goal:** Make 3D body-local/world point conversion exact-or-false across the
complete representable `Fixed64` domain without changing the authoritative
body-pose or host hierarchy-scale contract.

**Original Architecture:** Reuse FixedMathSharp's existing exact forward
scaled-point transform. Add its missing inverse counterpart so world
subtraction, inverse rotation, and component division remain one rational
operation until final round-half-to-even materialization. `SolidBody` exposes
matching throwing and `Try*` pairs; the throwing methods delegate to the
nonthrowing contract.

**Tech Stack:** C# 11, Q32.32 `Fixed64`, FixedMathSharp fixed-width wide
arithmetic, Gravitas `SolidBody`, xUnit v3, BenchmarkDotNet, `Release`, and
`ReleaseLean`.

## Global Constraints

- Determinism, performance, maintainability, and correctness are all release
  gates.
- The completed original phases preserve `Position3d`, `Rotation`, and strict
  host hierarchy scale. The approved fast follow replaces the live host-scale
  read with the collider's committed owner-scale snapshot so body conversion
  uses one coherent authoritative state.
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
   `FixedQuaternion.TryInverseTransformScaledPoint(origin, worldPoint, scale, out localPoint)`.
2. The inverse evaluates `InverseRotate(worldPoint - origin) / scale` as one
   exact rational expression per final local coordinate. Zero scale and a zero
   quaternion fail atomically.
3. Generic scaled 3D transform mechanics move from the oriented-box reducer to a
   focused numerics-wide owner alongside the new inverse. Existing public
   `TryTransformScaledPoint` and `Vector3d.TryComposeScaledLocalPoints` retain
   their signatures and behavior; no forwarding-only layer remains.
4. `SolidBody.TryTransformPoint(...)` first obtains strict canonical host scale,
   then delegates to the existing FixedMathSharp forward primitive.
5. `SolidBody.TryInverseTransformPoint(...)` obtains the same strict scale and
   delegates to the new inverse primitive.
6. Existing `TransformPoint(...)` and `InverseTransformPoint(...)` remain the
   concise convenience surface and throw when their corresponding `Try*`
   operation fails.
7. The original issue did not add a `SolidBody2D` point-transform API because no
   public counterpart existed. The approved fast follow below supersedes that
   narrow closure decision after the ownership review established a first-class
   transform and body-level parity contract.

## Alternatives Rejected

- **Matrix construction plus `TryTransformAffinePoint`:** the matrix stores
  already-rounded scale/rotation coefficients and general matrix inversion
  introduces a broader failure contract than the explicit TRS operation.
- **Gravitas-only wide helper:** scale/rotation/translation point conversion is
  reusable deterministic mathematics and the exact forward half already lives in
  FixedMathSharp.
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
- Update: `../FixedMathSharp/docs/MIGRATION.md`

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

- Modify: `src/Gravitas/Core/3D/SolidBody.cs`
- Modify: `tests/Gravitas.Tests/Core/SolidBodyIntegrationTests.cs`
- Modify or create a focused benchmark row under: `tests/Gravitas.Benchmarks`
- Update: `docs/wiki/HOST_INTEGRATION.md`

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
4,503/4,503 methods; `ReleaseLean` passes 3,811 tests. Ordinary and full-domain
round trips measure 3.520 us and 2.484 us respectively at zero managed
allocation in ShortRun.

## Phase 3: Closure

- [x] Run both repositories' full `Release` and `ReleaseLean` suites.
- [x] Build both target frameworks and package configurations without warnings.
- [x] Run focused point-transform benchmarks and allocation assertions.
- [x] Audit docs and public XML comments for exact failure semantics.
- [x] Audit pure 2D parity and record the no-new-API decision.
- [x] Request independent cross-stack code review and resolve all findings.
- [x] Move this plan to `docs/feature-work/done`.
- [x] Move the issue from the ordered queue to resolved history with
      verification evidence.
- [x] Update `feature-work-overview.md`.

**Phase 3 result:** Standard and Lean multi-target package builds are
warning-free, both full suites and exact coverage gates pass, and all four
focused benchmark rows allocate zero managed bytes. Host and migration docs
state the exact failure contract. Pure 2D intentionally retains its existing
internal exact transforms without adding an unused body-level API. Independent
cross-stack review confirmed the arithmetic derivation, ownership, API, failure,
test, and performance contracts and requested only complete generated XML
failure documentation, generic local-space terminology, and final plan movement;
all findings are resolved.

---

## Fast Follow: Transform Ownership And 2D Parity

**Status:** Complete.

**Goal:** Put generic local/world point conversion on `FixedTransform` while
retaining an explicit, dimensionally symmetric Gravitas API for authoritative
physics-pose conversion.

### Locked Contract

1. `FixedTransform` owns current-snapshot conversion:
   - `TransformPoint` / `TryTransformPoint`
   - `InverseTransformPoint` / `TryInverseTransformPoint`
   - explicit `TransformPointXZ` / `TryTransformPointXZ`
   - explicit `InverseTransformPointXZ` / `TryInverseTransformPointXZ`
2. The 3D operations use the strict composed affine hierarchy. They preserve
   hierarchy scale and shear and fail atomically when the matrix, inverse, or
   final coordinate is not representable.
3. The X/Z operations use the composed planar affine projection only when the
   hierarchy preserves the X/Z plane. They do not silently discard coupling
   introduced by pitch, roll, or a tilted parent.
4. Plain `Vector2d` overloads are not added because XY versus XZ would be
   ambiguous in an engine-agnostic 3D transform type.
5. `SolidBody` replaces the legacy transform-like names with authoritative:
   - `GetWorldPoint` / `TryGetWorldPoint`
   - `GetLocalPoint` / `TryGetLocalPoint`
6. `SolidBody2D` exposes the same authoritative API using `Vector2d` and the
   Gravitas X/Z simulation-plane convention.
7. Body conversion uses authoritative position and rotation plus the collider's
   committed owner-scale snapshot. It does not combine authoritative body state
   with a newly read, potentially mutated host scale.
8. `FixedTransform` conversion answers the current authored, host, or
   presentation snapshot. Body conversion answers the current deterministic
   simulation state. The two may intentionally differ during visual
   interpolation.
9. No `GetVisualWorldPoint` or equivalent body API is added; the current host
   transform already owns presentation conversion.
10. Gravitas remains the sole simulation authority in engine adapters. Unity,
    Godot, and Unreal transforms receive presentation output; native physics or
    interpolation must not become a second authority or double-interpolate
    Gravitas output.
11. All ordinary paths remain allocation-free. FixedMathSharp and Gravitas
    retain 100% reachable line, branch, and method coverage in `Release` and
    pass their `ReleaseLean` suites.

### Alternatives Rejected

- **Move everything to `FixedTransform`:** loses authoritative point queries
  whenever the host transform contains an interpolated presentation pose.
- **Keep identical transform-style names on bodies:** obscures the meaningful
  simulation-versus-presentation distinction.
- **Make body conversion follow the visible pose:** allows render timing,
  interpolation alpha, and adapter behavior to influence gameplay queries.
- **Add a separate visual body API:** duplicates the existing host-transform
  capability.

### Task 1: FixedTransform Owns Current-Snapshot Point Conversion

**Files:**

- Modify:
  `../FixedMathSharp/src/FixedMathSharp/Numerics/Matrices/FixedTransform.cs`
- Modify:
  `../FixedMathSharp/tests/FixedMathSharp.Tests/Numerics/Matrices/FixedTransform.Tests.cs`

**Interfaces:**

- Produces: `Vector3d TransformPoint(Vector3d point)`,
  `bool TryTransformPoint(Vector3d point, out Vector3d result)`,
  `Vector3d InverseTransformPoint(Vector3d point)`,
  `bool TryInverseTransformPoint(Vector3d point, out Vector3d result)`, and
  explicit X/Z `Vector2d` counterparts named in the locked contract.
- Reuses: `TryGetLocalToWorldMatrix`, `TryGetVerifiedInverse`, and
  `Fixed4x4.TryTransformAffinePoint`.

- [x] Add focused failing tests proving that 3D conversion uses the complete
      composed affine hierarchy, including representable shear, and that
      forward/inverse conversion round-trips ordinary points.
- [x] Add focused failing tests proving that the X/Z surface round-trips a
      plane-preserving affine hierarchy, retains in-plane shear, and rejects
      Y-axis coupling atomically.
- [x] Add focused failing tests for unrepresentable hierarchy composition,
      singular inverse transforms, truly unrepresentable final coordinates,
      throwing wrappers, and warmed zero allocation.
- [x] Run:

      ```powershell
          dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj `
            -c Release --no-restore `
            --filter "FullyQualifiedName~FixedTransformTests"
          ```

          Confirm the new tests fail because the point-conversion APIs are absent.

- [x] Implement the throwing wrappers as direct delegates to their `Try*`
      counterparts:

      ```csharp
          public Vector3d TransformPoint(Vector3d point)
          {
              if (!TryTransformPoint(point, out Vector3d result))
                  throw new InvalidOperationException(
                      "The composed transform or final world point is not representable.");
              return result;
          }
          ```

- [x] Implement 3D `Try*` conversion through strict matrix composition and one
      exact affine-point materialization:

      ```csharp
          public bool TryTransformPoint(Vector3d point, out Vector3d result)
          {
              if (!TryGetLocalToWorldMatrix(out Fixed4x4 matrix))
              {
                  result = default;
                  return false;
              }

              return Fixed4x4.TryTransformAffinePoint(matrix, point, out result);
          }
          ```

- [x] Make `TryGetWorldToLocalMatrix` obtain the strict composed matrix before
      verifying its inverse; it must not invert the saturating
      `LocalToWorldMatrix` view.
- [x] Build an internal planar affine matrix from the composed X/Z block and X/Z
      translation only. Preserve in-plane shear, use unit Y, and reject nonzero
      `M12`, `M21`, `M23`, or `M32` before forward or inverse conversion.
- [x] Run the focused command again and confirm every new and existing
      `FixedTransformTests` test passes.

**Task 1 result:** `FixedTransform` now owns strict current-snapshot 3D and
explicit X/Z point conversion. The 3D surface retains the complete composed
affine hierarchy, including shear. The X/Z surface retains in-plane affine terms
and rejects pitch-, roll-, or parent-induced plane coupling. Forward and inverse
failures are atomic, and inverse verification uses exact matrix multiplication
with the existing canonical fixed-point error bound.

### Task 2: FixedMathSharp Exact 2D Inverse Scaled-Point Contract

**Files:**

- Modify:
  `../FixedMathSharp/src/FixedMathSharp/Numerics/Vectors/Vector2d.Statics.cs`
- Modify:
  `../FixedMathSharp/src/FixedMathSharp/Numerics/Wide/WideVector2dTransform.cs`
- Modify:
  `../FixedMathSharp/tests/FixedMathSharp.Tests/Numerics/ScaledCompositeTransform.Tests.cs`
- Modify:
  `../FixedMathSharp/tests/FixedMathSharp.Benchmarks/Vector2dBenchmarks.cs`

**Interfaces:**

- Produces:

  ```csharp
  public static bool TryInverseTransformScaledPoint(
      Vector2d origin,
      Vector2d worldPoint,
      Vector2d scale,
      Fixed64 angleInRadians,
      out Vector2d localPoint);
  ```

- Consumes the existing `WideVector2dTransform`, `Signed192`, `Signed320`,
  `Signed576`, and `Fixed64.TryGetSignedRawRatio` owners without exposing wide
  types publicly.

- [x] Add failing ordinary, anisotropic-scale, quarter-turn, final-cancellation,
      zero-scale, true-overflow, round-trip, and warmed-allocation tests.
- [x] Derive expected boundary values from literal raw values or independent
      `Fixed64.TryMultiplyDivide` results; do not calculate expectations with
      the operation under test.
- [x] Run:

      ```powershell
          dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj `
            -c Release --no-restore `
            --filter "FullyQualifiedName~ScaledCompositeTransformTests"
          ```

          Confirm failure occurs because the inverse planar API is absent.

- [x] Implement the public method as the narrow entry to the existing internal
      wide owner.
- [x] In `WideVector2dTransform`, retain `worldPoint - origin`, inverse
      rotation, and component division as signed wide numerators and
      denominators until the final round-half-to-even `Fixed64` conversion. A
      zero scale component fails the complete operation and returns `default`.
- [x] Re-run the focused tests and the affected benchmark smoke test; confirm
      ordinary and full-domain paths allocate zero managed bytes.

**Task 2 result:** `Vector2d.TryInverseTransformScaledPoint` now preserves
subtraction, inverse projection, and scale division in the existing signed-wide
owner until one final round-half-to-even conversion per coordinate. Singular
scale and true final overflow return `false` with a zero result. Ordinary and
full-domain benchmark smoke rows report zero managed allocation.

### Task 3: Gravitas Authoritative 3D And 2D Body Parity

**Files:**

- Modify: `src/Gravitas/Colliders/3D/LSCollider.ShapeTransaction.cs`
- Modify: `src/Gravitas/Colliders/2D/LSCollider2D.ShapeTransaction.cs`
- Modify: `src/Gravitas/Core/3D/SolidBody.cs`
- Modify: `src/Gravitas/Core/2D/SolidBody2D.cs`
- Modify: `tests/Gravitas.Tests/Core/SolidBodyIntegrationTests.cs`
- Modify or create:
  `tests/Gravitas.Tests/Core/SolidBody2DPointTransformTests.cs`
- Modify: `tests/Gravitas.Benchmarks/Core/SolidBodyPointTransformBenchmarks.cs`

**Interfaces:**

- Produces identical authoritative surfaces on both body types:

  ```csharp
  GetWorldPoint(localPoint)
  TryGetWorldPoint(localPoint, out worldPoint)
  GetLocalPoint(worldPoint)
  TryGetLocalPoint(worldPoint, out localPoint)
  ```

- `SolidBody` uses `Vector3d`; `SolidBody2D` uses `Vector2d`.
- Consumes committed owner scale through a focused internal collider
  `TryGetCommittedOwnerScale(out scale)` contract.

- [x] Rename the existing 3D tests to the approved `Get*Point` API and add a
      failing regression where the host transform changes after collider shape
      commitment: body conversion must retain the committed scale.
- [x] Add failing 2D parity tests for ordinary scaled rotation, exact inverse,
      representable cancellation, singular scale/uncommitted shape, true final
      overflow, throwing wrappers, and warmed zero allocation.
- [x] Add a failing interpolation-divergence test proving that body conversion
      follows the authoritative pose while `FixedTransform.TransformPoint`
      follows the current presentation snapshot.
- [x] Run:

      ```powershell
          dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj `
            -c Release --no-restore `
            --filter "FullyQualifiedName~SolidBodyIntegrationTests|FullyQualifiedName~SolidBody2DPointTransformTests"
          ```

          Confirm the renamed/parity tests fail because the approved body APIs and
          committed-scale accessors are absent.

- [x] Add allocation-free internal committed-scale accessors that return `false`
      before the first shape commit rather than falling back to mutable host
      state.
- [x] Replace the legacy 3D `TransformPoint` surface with the authoritative
      `Get*Point` pairs and use the committed scale:

      ```csharp
          public bool TryGetWorldPoint(Vector3d point, out Vector3d result)
          {
              if (!Collider.TryGetCommittedOwnerScale(out Vector3d scale))
              {
                  result = default;
                  return false;
              }

              return Rotation.TryTransformScaledPoint(
                  Position3d,
                  point,
                  scale,
                  out result);
          }
          ```

- [x] Add the 2D counterparts over `_position`, `_rotation`, committed planar
      owner scale, `Vector2d.TryTransformScaledPoint`, and the Task 2 inverse.
- [x] Keep throwing wrappers concise through `SwiftThrowHelper`; failures must
      describe unavailable committed geometry, singular scale, or an
      unrepresentable final coordinate without mentioning presentation state.
- [x] Update benchmark calls to the renamed 3D API and add pure-2D ordinary and
      full-domain rows only when they measure distinct production paths.
- [x] Re-run the focused Gravitas tests and benchmark smoke test.

**Task 3 result:** `SolidBody` and `SolidBody2D` now expose the same
authoritative `GetWorldPoint` / `GetLocalPoint` and `Try*` pairs. Both consume
the collider root's committed owner-scale snapshot, so mutable host or
presentation scale cannot enter deterministic simulation queries. No compound
fallback or compatibility shim remains. The focused 32-test body suite and all
three benchmark smoke rows pass without managed allocation.

### Task 4: Documentation, Coverage, Performance, And Review Closure

**Files:**

- Modify: `../FixedMathSharp/docs/wiki/coordinate-conventions.md`
- Modify: `../FixedMathSharp/docs/MIGRATION.md`
- Modify: `docs/wiki/HOST_INTEGRATION.md`
- Modify: `docs/feature-work/feature-work-overview.md`
- Modify:
  `docs/feature-work/2026-07-30-full-domain-solid-body-point-transform-plan.md`

- [x] Document current-snapshot `FixedTransform` conversion, explicit X/Z plane
      admission, and strict failure behavior.
- [x] Document authoritative body conversion, the expected divergence from
      presentation transforms during interpolation, adapter ownership, and the
      prohibition on native double interpolation.
- [x] Record the clean v7 FixedMathSharp and pre-alpha Gravitas API changes in
      their existing migration/host guidance without compatibility shims.
- [x] Run complete `Release` and `ReleaseLean` suites for FixedMathSharp and
      Gravitas from the individual test projects so local project references
      inherit the correct configuration.
- [x] Run exact coverage gates and retain 100% reachable line, branch, and
      method coverage in both repositories. Remove unreachable branches or
      hollow tests rather than adding coverage-only API-shape assertions.
- [x] Build standard and Lean package configurations for all target frameworks
      without warnings.
- [x] Run focused point-transform benchmarks and allocation assertions; compare
      ordinary paths to the existing baseline and investigate any material
      regression.
- [x] Request an independent cross-stack code review and resolve every
      correctness, determinism, performance, maintainability, API, test, and
      documentation finding.
- [x] Record final test counts, coverage counts, allocation evidence, and
      benchmark results here; mark the fast follow complete and return this plan
      to `docs/feature-work/done`.

**Task 4 result:** FixedMathSharp passes 2,603 `Release` and 2,582 `ReleaseLean`
tests at 44,334/44,334 lines, 8,393/8,393 branches, and 3,320/3,320 methods.
Gravitas passes 3,870 `Release` and 3,815 `ReleaseLean` tests at 43,028/43,028
lines, 12,779/12,779 branches, and 4,486/4,486 methods. Standard and Lean
multi-target package builds are warning-free. ShortRun ordinary and full-domain
planar inverse rows measure 1.673 us and 0.905 us; 3D ordinary, 3D full-domain,
and 2D ordinary body round trips measure 3.714 us, 2.467 us, and 2.512 us. Every
row and warmed allocation assertion reports zero managed bytes. The ordinary 3D
result remains within the prior ShortRun noise band and the full-domain result
is unchanged. Independent cross-stack review reported no findings.
