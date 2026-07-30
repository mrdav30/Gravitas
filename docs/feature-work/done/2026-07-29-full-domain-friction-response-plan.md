# Full-Domain Friction Response Implementation Plan

**Status:** Complete. Phases 0-4 completed 2026-07-30.

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
  `src/Gravitas/CollisionHandling/Response/3D/ContactNormalImpulse3D.cs`
- Modify:
  `src/Gravitas/CollisionHandling/Response/3D/ContactResponseArithmetic3D.cs`
- Modify:
  `src/Gravitas/CollisionHandling/Response/Exact/ExactContactResponseKernel.Coulomb.cs`
- Modify:
  `src/Gravitas/CollisionHandling/Response/Exact/ExactContactResponseKernel.Normal.cs`
- Simplify:
  `src/Gravitas/CollisionHandling/Response/3D/ExactContactLever3D.cs`,
  `src/Gravitas/CollisionHandling/Response/3D/ResponseBody.cs`,
  `src/Gravitas/CollisionHandling/Response/3D/SolverContact.cs`, and
  `src/Gravitas/CollisionHandling/Response/Exact/ExactLever3D.Arithmetic.cs`
- Modify:
  `tests/Gravitas.Tests/CollisionHandling/CollisionResponseExactLeverTests.cs`
- Modify:
  `tests/Gravitas.Tests/CollisionHandling/CollisionWarmStartTests.cs`
- Modify:
  `tests/Gravitas.Tests/CollisionHandling/ContactNormalImpulseBoundaryTests.cs`
- Modify:
  `tests/Gravitas.Tests/CollisionHandling/ExactContactResponseKernel.Coulomb.Tests.cs`
- Delete when its production owner is removed:
  `tests/Gravitas.Tests/CollisionHandling/ExactLever3D.Tests.cs`
- Update:
  `docs/wiki/COLLISION_RESPONSE.md`
- Update:
  `docs/feature-work/2026-07-29-full-domain-friction-response-plan.md`

- [x] Add failing 3D regressions for compact point-velocity overflow, angular
  effective-mass overflow, cache-plus-delta cancellation, exact cache removal,
  static disk retention, dynamic disk projection, mirrored extremes, and true
  final overflow.
- [x] Prove friction-delta atomicity by asserting that a failed final friction
  delta cannot partially mutate either body or publish partially updated
  tangent caches. The already-applied normal response remains authoritative;
  atomically preflighting normal and friction together would be a separate
  solver-phase redesign.
- [x] Add a deterministic repeated-run regression with nonzero cached primary
  and secondary tangent impulses.
- [x] Add or extend a warmed allocation assertion for the exact cached-disk
  fallback.
- [x] Add the exact-kernel entry path for an already-completed non-negative
  normal accumulator and converge it on the Phase 1 disk core.
- [x] Give the compact 3D path one conservative admission gate using existing
  `ContactResponseArithmetic3D` and checked `Fixed64` operations.
- [x] Route every failed compact proof to the exact disk owner.
- [x] Replace `TrySolveFrictionImpulseExact`,
  `TryComputeTangentImpulseDeltaExact`, and `ClampTangentImpulsePair` where
  their behavior is fully subsumed; delete rather than retain forwarding hops.
- [x] Preserve the existing tangent basis, contact order, warm-start storage,
  diagnostics, and compact ordinary-domain results.
- [x] Run focused 3D response, replay, and allocation tests.
- [x] Run focused coverage and remove unreachable branches rather than writing
  hollow tests.
- [x] Request independent review, resolve findings, record results, and stop
  for owner review.

**Phase 2 result:** 3D friction now keeps proven-safe contacts on the compact
solver while routing any unsafe point velocity, effective mass, cache
accumulation, disk clamp, or final velocity materialization once through the
exact cached Coulomb-disk owner. Friction deltas are preflighted across both
bodies before application; an unrepresentable friction result leaves the
already-applied normal response authoritative and publishes no partial tangent
cache. Duplicate exact-friction arithmetic and its forwarding-only helpers
were removed. Focused 3D response, exact-kernel, warm-start, allocation, and
boundary coverage passes `106/106`; the warmed exact cached-disk path allocates
zero bytes. The authoritative Release gate passes `3,843/3,843` tests with
`43,534/43,534` lines, `12,745/12,745` branches, and `4,507/4,507` methods
covered. ReleaseLean passes `3,788/3,788`; normal and Lean multi-target package
builds pass serially with zero warnings or errors so lower-stack local-link
restores cannot race across configurations. Independent review found no
remaining correctness, scope, or allocation issue. Pure 2D and mixed response
remain unchanged for Phase 3.

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

- [x] Add a failing pure-2D `Fixed64.MinValue` cached-impulse regression that
  distinguishes the mathematical magnitude from saturating `Abs()`.
- [x] Add pure-2D mirrored cancellation/removal and true final-overflow
  regressions around the existing exact line owner.
- [x] Add mixed regressions for unsafe point-velocity construction, tangent
  rejection/normalization, angular denominator accumulation, friction-limit
  multiplication, mirrored extremes, and final atomic overflow.
- [x] Tighten pure-2D compact admission only where the current exact line
  fallback already supplies the complete contract.
- [x] Prove every mixed compact tangent operation or route once to the exact
  disk path; do not add another mixed wide solver.
- [x] Preserve mixed planar constraints and the current uncached disk contract.
- [x] Extend warmed exact 2D and mixed allocation assertions.
- [x] Run focused 2D/mixed response, replay, allocation, and coverage tests.
- [x] Request independent review, resolve findings, record results, and stop
  for owner review.

**Phase 3 result:** Pure 2D now classifies the complete signed static-friction
interval without saturating `Abs()`, checks the complete linear/angular
effective-mass denominator, and routes unsafe cache or final-delta work to its
existing exact Coulomb line. Mixed response now uses canonical X/Z embedding
and proves point velocity, tangent projection/normalization, every effective-
mass term, friction limits, and final impulse components before retaining the
compact path; any failed proof routes once to the existing uncached exact disk.
Subprecision cross, dot, scale, and angular-response loss is rejected by the
shared response arithmetic owners rather than patched per caller. Focused 2D,
mixed, replay, boundary, and allocation coverage passes `278/278`; warmed exact
2D and mixed friction allocate zero bytes. The complete Release suite passes
`3,861/3,861` with `43,654/43,654` lines, `12,775/12,775` branches, and
`4,502/4,502` methods covered. Independent review identified and closed
pure-2D subprecision point-velocity and mixed multi-axis tangent-projection
admission gaps. The final coverage pass removed an impossible zero-impulse
helper branch at its sole nonzero-impulse caller, and independent re-review
found no remaining Phase 3 correctness, allocation, dimensional-parity, API,
or test-quality issue.

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

- [x] Run the complete Gravitas `Release` suite from the test project so local
  project references inherit the correct configuration.
- [x] Run `ReleaseLean` and validate standard/Lean package output across both
  target frameworks. Retain published-package relinking as the existing
  sequential release gate after the lower stack is released.
- [x] Collect coverage and confirm 100% reachable line, branch, and method
  coverage.
- [x] Run deterministic replay and every exact-response allocation gate.
- [x] Compare the existing representative compact/exact response benchmarks.
  Preserve ordinary compact throughput and zero allocation; record any
  meaningful exact-fallback change in the benchmark backlog.
- [x] Confirm FixedMathSharp and GridForge remain at their existing 100%
  coverage baselines when their source is unchanged.
- [x] Review the complete diff for duplicate helpers, forwarding-only methods,
  stale branches, avoidable partials, and physics policy misplaced in
  FixedMathSharp.
- [x] Update `COLLISION_RESPONSE.md` with the compact-proof/exact-fallback
  contract without exposing internal wide representations.
- [x] Move the issue-tracker entry into resolved history and renumber the
  ordered queue.
- [x] Mark this plan complete, move it to `docs/feature-work/done`, request one
  final independent review, and resolve all findings.

**Phase 4 result:** The complete Release suite passes `3,861/3,861` tests at
`43,653/43,653` lines, `12,775/12,775` branches, and `4,501/4,501` methods.
ReleaseLean passes `3,806/3,806`; standard and Lean library builds produce
both target frameworks and their packages with zero warnings or errors.
Focused deterministic replay passes `85/85`, and the warmed exact 3D, 2D, and
mixed friction allocation gates pass `3/3` at zero managed bytes.

Thirty-two prepared 3D/mixed and ten pure-2D ShortRun response cells remain at
`0 B/op` without a gross compact-path regression. The short timing windows are
not precise enough to close the existing exact-response profiling signal, so
the measured follow-up remains in the benchmark backlog rather than spawning a
duplicate issue. FixedMathSharp and GridForge source were unchanged, preserving
their verified 100% coverage baselines. The complete implementation and
documentation audit found no duplicate wide owner, forwarding-only production
hop, stale branch, avoidable partial, public API addition, or physics policy
misplaced in FixedMathSharp. `COLLISION_RESPONSE.md` now states the exact
rational/radical projection contract without exposing internal wide
representations. The issue moved to resolved history and the active queue now
contains four items. Published-package relinking remains the evergreen
sequential release gate after lower-stack releases. Final independent review
removed one forwarding-only 2D admission helper, corrected the wiki's
rational/radical wording, preserved the historical issue anchor, and found no
remaining actionable issue after the fresh coverage and Lean reruns.

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
