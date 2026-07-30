# Full-Domain Friction Response Implementation Plan

**Status:** Active. Phases 0-1 are complete; Phase 1 is awaiting owner review.

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> `superpowers:test-driven-development` while changing behavior,
> `superpowers:requesting-code-review` at each review checkpoint, and
> `superpowers:verification-before-completion` before reporting a phase
> complete. Steps use checkbox (`- [ ]`) syntax for living progress tracking.

**Goal:** Preserve exact Coulomb-friction accumulation, clamping, and final
body response across the complete representable `Fixed64` domain in Gravitas
3D, 2D, and mixed solvers.

**Architecture:** Keep ordinary contacts on the existing compact solver paths.
When any intermediate cannot be proven safe, route once into the
Gravitas-owned allocation-free exact response kernel. Extend that single owner
to combine cached two-axis tangent impulses, Coulomb-disk clamping, and final
applied velocity deltas before narrowing; do not add physics policy or new
public APIs to FixedMathSharp.

**Tech Stack:** C# 11, Q32.32 `Fixed64`, FixedMathSharp internal
`WideArithmetic` through the intentional friend boundary, Gravitas 3D/2D/mixed
response, xUnit v3, BenchmarkDotNet, `Release` and `ReleaseLean`.

## Global Constraints

- Determinism, performance, maintainability, and correctness are all release
  gates.
- Preserve the compact friction path for proven-safe ordinary inputs.
- Keep Coulomb accumulation and clamping policy internal to Gravitas.
- Reuse `ExactContactResponseKernel`, `ContactResponseArithmetic3D`, and the
  existing exact line/disk response machinery; do not create a parallel wide
  solver or a public arbitrary-precision API.
- Compute cache addition, cache removal, friction-limit multiplication,
  two-axis magnitude comparison, radial clamping, and final body deltas before
  narrowing to `Fixed64`.
- Preserve atomic response: either every participating body's final linear and
  angular delta is representable and applied, or neither body is mutated.
- Preserve deterministic tangent ordering and existing warm-start cache
  semantics.
- Keep warmed exact response allocation-free.
- Retain 100% reachable line, branch, and method coverage without hollow
  API-shape tests, reflection-only tests, exclusions, or zombie branches.
- Keep FixedMathSharp and GridForge at 100% coverage when this work does not
  intentionally change them.
- Leave implementation and documentation changes unstaged and uncommitted.
  Pause after each phase and provide a recommended commit message.

---

## Locked Design

1. The exact Coulomb-disk kernel becomes the only owner of unsafe two-axis
   friction accumulation and clamping.
2. The kernel accepts the two cached tangent impulses. It adds them to the
   newly solved tangent ratios, classifies the complete desired vector against
   the static disk, projects it to the dynamic disk when required, subtracts
   the cached vector, and materializes only the final applied body deltas.
3. Dynamic projection with a cache is evaluated as one signed
   rational-plus-radical expression per final velocity component. Neither the
   projected accumulated impulse nor the cache response is narrowed first.
4. Existing callers that have an exact normal constraint retain that entry
   path. The 3D solver may supply its already-completed representable normal
   accumulator directly; both paths converge on the same disk core.
5. The response continues to expose representable accumulated tangent
   projections when available for warm-start storage. An unrepresentable cache
   projection does not invalidate an otherwise representable physical delta,
   but true final velocity overflow rejects the response atomically.
6. Compact paths use checked arithmetic or existing conservative admission
   proofs. A failed proof is a route to the exact kernel, not a physical
   rejection.
7. Pure 2D retains its one-axis exact Coulomb-line owner. Mixed response retains
   its uncached two-axis disk semantics; parity work tightens only unsafe
   compact admission and reuses the same disk core.

## Phase 0: Root Cause And Contract

- [x] Trace every Coulomb line/disk caller across 3D, 2D, and mixed response.
- [x] Confirm the 3D exact-lever path narrows tangent ratios before cache
  accumulation and disk clamping.
- [x] Confirm the existing exact disk owner has no cached-impulse inputs.
- [x] Confirm pure 2D already has exact cached line response but its compact
  absolute-value comparison cannot distinguish `Fixed64.MinValue` magnitude.
- [x] Confirm mixed exact response already uses the uncached disk owner while
  its compact proof covers the normal axis rather than every later tangent
  operation.
- [x] Reject always-wide response because it taxes ordinary hot-path contacts.
- [x] Reject new FixedMathSharp Coulomb APIs because friction accumulation is
  Gravitas physics policy.
- [x] Obtain owner approval for the hybrid compact-plus-exact design.

**Phase 0 result:** one shared exact owner is missing cached disk semantics;
2D and mixed require only compact-path parity hardening.

---

## Phase 1: Exact Cached Coulomb-Disk Owner

**Review boundary:** Stop after the kernel and its focused tests pass. The
existing mixed caller may add two explicit zero-cache arguments to consume the
new internal signature, but no solver behavior changes in this phase.

**Files:**

- Modify:
  `tests/Gravitas.Tests/CollisionHandling/ExactContactResponseKernel.Coulomb.Tests.cs`
- Create:
  `tests/Gravitas.Tests/CollisionHandling/ExactContactResponseKernel.CoulombDiskCache.Tests.cs`
- Modify:
  `src/Gravitas/CollisionHandling/Response/Exact/ExactContactResponseKernel.Coulomb.cs`
- Modify mechanically:
  `src/Gravitas/CollisionHandling/Response/Mixed/CollisionResponseMixed.cs`
- Create only if the existing Coulomb partial would otherwise mix unrelated
  responsibilities:
  `src/Gravitas/CollisionHandling/Response/Exact/ExactContactResponseKernel.CoulombDiskCache.cs`
- Update:
  `docs/feature-work/2026-07-29-full-domain-friction-response-plan.md`

**Produces:**

```csharp
internal static bool TryGetCoulombDiskResponse(
    in ExactNormalConstraint3D normalConstraint,
    in ExactContactResponseOperand3D primaryFirst,
    in ExactContactResponseOperand3D primarySecond,
    Vector3d primaryTangent,
    Fixed64 accumulatedPrimaryTangentImpulse,
    in ExactContactResponseOperand3D secondaryFirst,
    in ExactContactResponseOperand3D secondarySecond,
    Vector3d secondaryTangent,
    Fixed64 accumulatedSecondaryTangentImpulse,
    Fixed64 staticFriction,
    Fixed64 dynamicFriction,
    out ExactCoulombResponse3D response);
```

The current uncached callers pass `Fixed64.Zero` for both accumulated values.
An internal overload that accepts an already-completed normal accumulator may
be added in Phase 2 without duplicating the disk core.

- [x] Add a failing static-disk regression where a cached primary tangent and
  the new exact tangent ratio cancel to a representable final accumulator.
- [x] Add a failing dynamic-disk regression where two cached tangent axes are
  projected and removed without narrowing either the radical projection or
  cache response first.
- [x] Add mirrored `Fixed64.MinValue`/positive-extreme cases that prove sign and
  round-to-even symmetry at the representable boundary.
- [x] Add a true final-overflow case proving the response returns `false`
  without a partial result.
- [x] Extend the warmed allocation assertion to exercise nonzero cached disk
  response.
- [x] Run the focused tests and confirm they fail against the uncached kernel:

  ```powershell
  dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj `
    -c Release --no-restore `
    --filter "FullyQualifiedName~ExactContactResponseKernelCoulombTests"
  ```

- [x] Add cached impulses to the exact disk input contract and combine each
  cache with its solved tangent ratio before the static/dynamic disk decision.
- [x] Materialize cached dynamic-disk deltas as one exact signed
  rational-plus-radical expression per linear/angular component.
- [x] Preserve existing zero-cache results and input validation.
- [x] Return each final accumulated tangent projection only when that scalar is
  representable; do not reject a representable body response solely because an
  optional cache scalar is not.
- [x] Rerun the focused tests and confirm they pass with zero warmed
  allocations.
- [x] Run the complete exact-response test area:

  ```powershell
  dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj `
    -c Release --no-restore `
    --filter "FullyQualifiedName~ExactContactResponse"
  ```

- [x] Request an independent code review of Phase 1 and resolve all findings.
- [x] Record the exact focused test counts and Phase 1 summary here, then stop
  for owner review.

**Phase 1 result:** The first red run failed at compile time because the
uncached kernel had no twelve-argument overload. The completed owner now keeps
two-axis cache addition, static/dynamic disk classification, radial projection,
cache removal, and final body deltas exact until round-to-even narrowing.
Focused Coulomb coverage passes `19/19`; the complete exact-response area passes
`30/30`. The authoritative Release gate passes `3,827/3,827` tests with
`43,215/43,215` lines, `12,673/12,673` branches, and `4,506/4,506` methods
covered. Independent review found and closed boundary-rounding, narrow-before-
cache-removal, opposing-sign word-width, stack-width, forwarding-helper, and
file-ownership risks. The final cohesive partials are 949 and 838 lines, and
the warmed nonzero-cache path allocates zero bytes. Normal and Lean
multi-target package builds both pass with zero warnings or errors.

---

## Phase 2: 3D Friction Adoption

**Review boundary:** Stop after 3D behavior, allocation, replay, and focused
coverage pass. Do not change pure 2D or mixed response in this phase.

**Files:**

- Modify:
  `src/Gravitas/CollisionHandling/Response/3D/CollisionResponse.cs`
- Modify:
  `src/Gravitas/CollisionHandling/Response/Exact/ExactContactResponseKernel.Coulomb.cs`
- Modify:
  `tests/Gravitas.Tests/CollisionHandling/CollisionResponseExactLeverTests.cs`
- Modify:
  `tests/Gravitas.Tests/CollisionHandling/CollisionWarmStartTests.cs`
- Modify:
  `tests/Gravitas.Tests/Determinism/GravitasReplayConformanceTests.cs`
- Update:
  `docs/wiki/COLLISION_RESPONSE.md`
- Update:
  `docs/feature-work/2026-07-29-full-domain-friction-response-plan.md`

- [ ] Add failing 3D regressions for compact point-velocity overflow, angular
  effective-mass overflow, cache-plus-delta cancellation, exact cache removal,
  static disk retention, dynamic disk projection, mirrored extremes, and true
  final overflow.
- [ ] Prove atomic rejection by asserting both bodies and all three warm-start
  scalars remain unchanged when a final delta cannot be represented.
- [ ] Add a deterministic repeated-run regression with nonzero cached primary
  and secondary tangent impulses.
- [ ] Add or extend a warmed allocation assertion for the exact cached-disk
  fallback.
- [ ] Add the exact-kernel entry path for an already-completed non-negative
  normal accumulator and converge it on the Phase 1 disk core.
- [ ] Give the compact 3D path one conservative admission gate using existing
  `ContactResponseArithmetic3D` and checked `Fixed64` operations.
- [ ] Route every failed compact proof to the exact disk owner.
- [ ] Replace `TrySolveFrictionImpulseExact`,
  `TryComputeTangentImpulseDeltaExact`, and `ClampTangentImpulsePair` where
  their behavior is fully subsumed; delete rather than retain forwarding hops.
- [ ] Preserve the existing tangent basis, contact order, warm-start storage,
  diagnostics, and compact ordinary-domain results.
- [ ] Run focused 3D response, replay, and allocation tests.
- [ ] Run focused coverage and remove unreachable branches rather than writing
  hollow tests.
- [ ] Request independent review, resolve findings, record results, and stop
  for owner review.

---

## Phase 3: Pure 2D And Mixed Parity

**Review boundary:** Stop after dimensional parity tests and focused coverage
pass. Do not start release-wide closure in this phase.

**Files:**

- Modify:
  `src/Gravitas/CollisionHandling/Response/2D/CollisionResponse2D.cs`
- Modify:
  `src/Gravitas/CollisionHandling/Response/Mixed/CollisionResponseMixed.cs`
- Modify:
  `tests/Gravitas.Tests/CollisionHandling/CollisionResponse2DManifoldTests.cs`
- Modify:
  `tests/Gravitas.Tests/MixedDimensions/MixedResponseTests.Task7.cs`
- Update:
  `docs/wiki/COLLISION_RESPONSE.md`
- Update:
  `docs/feature-work/2026-07-29-full-domain-friction-response-plan.md`

- [ ] Add a failing pure-2D `Fixed64.MinValue` cached-impulse regression that
  distinguishes the mathematical magnitude from saturating `Abs()`.
- [ ] Add pure-2D mirrored cancellation/removal and true final-overflow
  regressions around the existing exact line owner.
- [ ] Add mixed regressions for unsafe point-velocity construction, tangent
  rejection/normalization, angular denominator accumulation, friction-limit
  multiplication, mirrored extremes, and final atomic overflow.
- [ ] Tighten pure-2D compact admission only where the current exact line
  fallback already supplies the complete contract.
- [ ] Prove every mixed compact tangent operation or route once to the exact
  disk path; do not add another mixed wide solver.
- [ ] Preserve mixed planar constraints and the current uncached disk contract.
- [ ] Extend warmed exact 2D and mixed allocation assertions.
- [ ] Run focused 2D/mixed response, replay, allocation, and coverage tests.
- [ ] Request independent review, resolve findings, record results, and stop
  for owner review.

---

## Phase 4: Coverage, Performance, Documentation, And Queue Closure

**Files:**

- Update:
  `docs/wiki/COLLISION_RESPONSE.md`
- Update:
  `docs/feature-work/issue-tracker.md`
- Update if measured evidence changes:
  `docs/feature-work/benchmark-signal-hardening-backlog.md`
- Move when complete:
  `docs/feature-work/2026-07-29-full-domain-friction-response-plan.md`
  to `docs/feature-work/done/`

- [ ] Run the complete Gravitas `Release` suite from the test project so local
  project references inherit the correct configuration.
- [ ] Run `ReleaseLean` and package-reference validation.
- [ ] Collect coverage and confirm 100% reachable line, branch, and method
  coverage.
- [ ] Run deterministic replay and every exact-response allocation gate.
- [ ] Compare the existing representative compact/exact response benchmarks.
  Preserve ordinary compact throughput and zero allocation; record any
  meaningful exact-fallback change in the benchmark backlog.
- [ ] Confirm FixedMathSharp and GridForge remain at their existing 100%
  coverage baselines when their source is unchanged.
- [ ] Review the complete diff for duplicate helpers, forwarding-only methods,
  stale branches, avoidable partials, and physics policy misplaced in
  FixedMathSharp.
- [ ] Update `COLLISION_RESPONSE.md` with the compact-proof/exact-fallback
  contract without exposing internal wide representations.
- [ ] Move the issue-tracker entry into resolved history and renumber the
  ordered queue.
- [ ] Mark this plan complete, move it to `docs/feature-work/done`, request one
  final independent review, and resolve all findings.

## Completion Criteria

- Extreme but representable 3D, 2D, and mixed friction produces the
  mathematically correct deterministic response.
- Cached two-axis accumulation and Coulomb-disk clamping never narrow before
  the final applied body deltas are known.
- True final overflow rejects atomically.
- Ordinary contacts retain the compact path.
- Warmed exact fallbacks allocate zero managed bytes.
- Gravitas retains 100% reachable line, branch, and method coverage.
- No new public API or FixedMathSharp physics policy is introduced.
- Documentation and the ordered issue queue match the shipped behavior.
