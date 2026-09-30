# Complete Capsule/Slab Contact Plan

**Status:** Proposed design and execution scope; awaiting user review. No runtime changes made.  
**Created:** 2026-09-29.  
**Repositories:** FixedMathSharp and Gravitas.  
**Issue:** [GRV-Issue-088](issue-tracker.md#grv-issue-088---mixed-capsulenonzero-core-capsule-slab-contact-retains-incomplete-directions).

## Goal And Scope

Replace the incomplete mixed 3D-capsule/nonzero-core-2D-capsule-slab relation
with complete fixed-point geometry. Determine separation, contact normal and
minimum penetration from the whole shape before rounding. Preserve deterministic
lockstep behavior, canonical contact anchors and allocation-free warmed queries.

FixedMathSharp owns the geometry and exact arithmetic. Gravitas consumes one
narrow internal penetration query and keeps its existing contact, material and
response ownership. This is the single cross-repository design and progress
record; keep proof notes and final evidence here rather than creating parallel
plans or progress reports. The execution outline is gated by the mathematical
proof in phase 0, not a claim that the new candidate enumeration is already proved.

Included:

- Complete 3D capsule versus a planar stadium extruded through flat Y caps.
- Focused reuse of existing cylinder/capsule and constrained-feature arithmetic.
- Gravitas dispatch, canonical support-contact construction, regression tests,
  resource checks and matched benchmarks in the existing projects.

Not included:

- Other mixed cylinder/cone pairs, CCD, broad phase, response or serialization
  redesign; pure 2D and ordinary 3D geometry retain their own contracts.
- A generic collision framework, sampled-direction fallback, new dependency,
  persistent cache, public compatibility wrapper, benchmark project or CI system.
- Optimizing [GRV-Benchmark-020](benchmark-signal-hardening-backlog.md) or resolving
  GRV-Issue-083. Existing cylinder/circle-slab costs are regression controls here.
- Package releases. Coordinated source validation does not prove package readiness.

## Evidence And Current Behavior

Planning inspection starts at Gravitas `5ad457d` and FixedMathSharp `535578e`.
Both worktrees were clean. The tracker retains the earlier diagnostic reproduction;
phase 0 must turn that evidence into an enabled .NET 8 regression.

- 3D capsule: radius 1, total height 22, center `(83/4,7/4,0)`, rotation
  `(0,0,-q,q)`, where `q=Fixed64.FromRaw(3037000500)`.
- Slab: `LSCapsuleCollider2D(radius:10,height:22)`, at the origin, unrotated,
  half-thickness 1. Its planar core has length 2 along Z.
- Closest 3D core endpoint: `(43/4,7/4,0)`. Nearest slab point: `(10,1,0)`.
  The squared gap is `9/8 > 1`, so these shapes are separated despite overlapping
  bounds. Extending the slab along Z cannot close that X/Y gap.
- The current mixed query incorrectly reports contact, depth approximately
  `0.13923784578219056`. The zero-core case already rejects through GRV-Issue-085.

`TryTestCapsuleCapsuleSlab` tests a limited direction set against the whole
slab and keeps the shallowest result. It does **not** combine cylinder contacts.
Exact projection along each chosen direction does not make the selection complete.

A middle box plus two end cylinders can represent the slab as a union and support
an overlap-only decomposition. Combining their independent contact depths does
not give whole-shape minimum penetration: clearing one piece can leave another
intersecting. That shortcut is rejected, not part of the proposed fix.

## Geometry And Contact Contract

Retain authoritative centers, rotations, core lengths, radii and half-thickness.
Do not reconstruct the geometry from rounded world axes, scalar endpoint positions
or a saturated relative-center subtraction. The planar local core is +Z after
X/Z embedding, with the existing planar-rotation convention.

- The slab has flat caps, straight sides and rounded ends. It is neither a
  single cylinder nor a rounded 3D capsule.
- An exactly zero slab core reuses the existing complete cylinder/capsule route.
  Every positive core remains a stadium, including one/two-raw and odd-raw lengths;
  `Fixed64.Epsilon` must not decide a shape change or discard its rotation.
- Preserve existing admitted dimensions and frames. Handle a zero 3D capsule
  core and the zero-radius degeneracies admitted by the reused geometry owners;
  do not add new public shape validation or expand collider admission.
- Classify exact geometry first using the signed contact depth defined below:
  a negative value rejects, zero is exact touching, and a positive value is
  overlap (which may round to zero depth). Adding a tolerance or treating a
  failed proof as separation is not acceptable.
- Choose the exact minimum signed support gap before adding the 3D capsule
  radius and materializing depth. Round ordinary outputs nearest-even; retain
  `DepthIsClamped` for conceptual overflow, distinct from an exact maximum value.
- Return a slab-to-capsule normal upstream; Gravitas reverses it to its established
  3D-to-2D convention. Separation returns default outputs.
- Define stable feature/root order and keep the earlier candidate on exact ties.
  Unique winners must respect admissible rigid transforms; tied normals need not
  be invariant under a different enumeration. Zero-core outputs retain existing ties.
- Gravitas retains `BuildCanonicalSupportContact`: anchors remain in the authored
  frames, with the selected normal/depth/clamp and unchanged material propagation.
  This work does not redesign support-anchor selection into a manifold generator.

### Complete Candidate Regions

For a unit world direction `n`, let `u` be world Up, `s` the exact planar slab
half-core, `b` the exact 3D capsule half-core, and `d` the capsule center minus
slab center. With slab radius `R`, half-thickness `H`, and capsule radius `r`:

```text
g(n) = R * sqrt(1 - (u dot n)^2)
       + H * abs(u dot n) + abs(s dot n) + abs(b dot n) - d dot n
signed contact depth = r + min_over_unit_n(g(n))
```

This is a mathematical specification, not a scalar implementation recipe.
Use the existing exact homogeneous/rational representations. Negative signed
contact depth means separation; a negative core gap alone does not, because
the capsule radius can bridge it.

Partition the direction domain into:

1. **Each rounded-end region:** `s dot n > 0` and `< 0`. Each is a shifted
   cylinder/capsule problem, but every candidate must satisfy that region.
   Keep endpoint offsets exact even when no scalar endpoint is representable.
2. **The straight-side boundary:** `s dot n = 0`. On this normal plane the
   disk projects to a line; the cylinder plus capsule core becomes the sum of
   three planar segments (a 2D zonotope). Enumerate its normal-region boundaries
   and admissible interior stationary directions analytically.
3. **Shared boundaries and degeneracies:** poles, cap/side/rim changes, capsule
   endpoint/interior changes, parallel/perpendicular axes, collapsed projections,
   repeated or zero roots, and exact ties must have explicit owners.

The existing cylinder ellipse solver independently reflects basis directions
and retains one largest-positive stationary root using an unrestricted-domain
proof. Those shortcuts cannot simply be applied inside an end region: reflection
may leave that region, and its global winner may be inadmissible. Enumerate all
needed admitted roots, or prove a narrower regional reduction first. Compare
analytic and curved candidates across regions exactly, including gap signs;
never compare already rounded regional depths.

Early exits require a whole-shape certificate. An endpoint-cylinder closest-point
certificate additionally needs the slab-core support-sign condition. Likewise,
triangle/slab code's negative-gap rejection cannot be copied before the capsule
radius has been included. No iteration cap may silently produce an approximate
answer or a false miss.

## Existing Owners And File Boundaries

Paths in this table are relative to the named repository. Prefer a focused slab
partial and the minimum genuinely shared extraction; do not make every cylinder
query carry a new generic feature framework.

| Repository / files | Responsibility |
| --- | --- |
| FixedMathSharp `src/FixedMathSharp/Geometry/Wide/Convex/WideConvexPrismRelations.CylinderCapsuleFeatures.cs` | Reuse exact rigid-axis geometry, analytic candidate construction/ranking and complete-depth materialization; preserve existing consumers. |
| FixedMathSharp `src/FixedMathSharp/Geometry/Wide/Convex/WideConvexPrismRelations.CylinderCapsuleEllipse.cs` | Reuse stationary-polynomial mechanics only where their scale, sign and domain contracts hold. |
| FixedMathSharp `src/FixedMathSharp/Geometry/Wide/Convex/ConvexContactValueRoot.cs` and `WideConvexPrismRelations.CylinderPairSideFeatures.cs` | Reuse exact value-root mapping/comparison, not the unrestricted winner/root-ordinal assumptions. |
| FixedMathSharp `src/FixedMathSharp/Geometry/Wide/Triangles/TriangleCylinderAnalyticFeatures.cs` and `TriangleCylinderEdgeContacts.cs` | Existing examples of explicit seam ownership and endpoint-region root admission; not a replacement solver for a degenerate triangle. |
| FixedMathSharp new `src/FixedMathSharp/Geometry/Wide/Convex/WideConvexPrismRelations.CapsuleSlab.cs` | Own the complete slab/capsule relation. Split further only by a demonstrated cohesive responsibility. |
| FixedMathSharp new `tests/FixedMathSharp.Tests/Geometry/Primitives/CenteredCapsuleSlabContact.Tests.cs` | Test the policy-neutral geometry directly, including exact outputs, wide-domain and resource behavior. |
| Gravitas `src/Gravitas/CollisionHandling/Detection/Mixed/CollisionDetectionMixed.cs` | Replace nonzero-core direction selection with the upstream query and existing canonical-contact construction. |
| Gravitas new `tests/Gravitas.Tests/MixedDimensions/MixedNarrowPhaseTests.CapsuleSlab.cs` | Exercise initialized colliders through actual mixed dispatch; reuse existing test helpers. |
| Gravitas new `tests/Gravitas.Benchmarks/CollisionHandling/CapsuleSlabContactBenchmarks.cs` | Measure the same nonzero-core fixtures before and after the repair, with semantic preflight and memory diagnostics. |

The narrow internal entry point should accept slab center, authored planar
rotation, core length, radius and half-thickness; capsule center, authored rigid
rotation, core length and radius; and return `bool` with normal, depth and clamp
outputs. Use the existing +Z planar / +Y capsule local-axis conventions of these
colliders. Prefer `TryGetCenteredCapsuleSlabCapsulePenetration` on
`WideConvexPrismRelations`; finalize its exact representation alongside the phase-0
width proof, not by narrowing inputs for convenience. No new public API is needed.

FixedMathSharp internals remain out of Gravitas public/protected signatures,
serialization, tests and benchmarks. Do not add friendship or downstream wide
arithmetic wrappers. Audit all callers before deleting capsule-specific direction
helpers; retain shared cylinder/cone helpers unless separately proved obsolete.
Keep each non-private type in its own file and avoid tiny forwarding partials.

## Acceptance And Execution Outline

### Phase 0 - Regression, Completeness Proof And Baseline

- [ ] Add `CapsuleCapsuleSlab_WithSeparatedRim_ShouldRejectOverlappingBounds`
  in the Gravitas test file above, using the exact recorded fixture. Assert
  bounds overlap, `TryCollide == false` and default/no-contact output. Run it
  against unchanged source and retain the intended assertion failure.
- [ ] Build independent expected-result fixtures for straight side, flat cap,
  straight rim, both rounded ends, oblique capsule-interior rim and containment.
  Include a constrained end-region winner missed by unrestricted-root selection
  and a true straight-side-boundary interior winner. Derive support/separation
  certificates or independently bounded exact roots; do not use the new solver
  or a sampled direction search as its own oracle.
- [ ] Complete the candidate-enumeration/admission proof, stable tie order,
  coefficient-width bounds and peak live scratch calculation. Cover extreme
  relative centers, exact half-core offsets and wide full slab height; the
  cylinder solver's current 40-word proof is not automatically sufficient.
- [ ] Obtain independent correctness and Ponytail review of that derivation
  before implementation. Resolve missing regions, unproved pruning or resource
  limits here; amend this plan if a simpler complete approach is demonstrated.
- [ ] Capture a nonzero-core baseline in the existing Gravitas benchmark project
  before production changes. Keep the same fixtures, runtime, configuration,
  affinity and measurement settings for comparison. Record old hit/depth results
  separately and label incorrect rows as repair-cost baselines. Temporary baseline
  diagnostics must not become a retained switch accepting incorrect answers.
- [ ] Capture fresh unchanged `CapsuleCircleContactBenchmarks` mixed/3D controls
  and affected upstream shared-owner controls. Preserve full logs and binaries
  used for the baseline; do not rebuild them against changed siblings mid-run.

**Exit:** a reproducible red test, independently reviewed complete design and
resource bounds, and a comparable measured baseline. This gate does not close
the issue or make a no-go decision based on old incorrect performance.

### Phase 1 - Complete FixedMathSharp Geometry

- [ ] Add direct upstream regressions first, then implement the reviewed region
  selection with existing exact arithmetic, value ranking and final materializers.
  Keep the zero-core reduction on the existing complete owner.
- [ ] Verify all phase-0 feature fixtures with known normal/depth results, not
  only booleans. Include a containment case where a constituent's minimum exit
  does not clear the whole slab.
- [ ] Pin strict separation, exact touch and one-raw overlap at a straight rim:
  use the recorded X coordinate with capsule center Y=2, radius `5/4 + raw(k)`
  for `k=-1,0,1`, and unchanged core length 20. Expected hit is `k >= 0`, depth
  is `raw(k)` on contact, and slab-to-capsule normal is `(3/5,4/5,0)`.
- [ ] Cover zero and tiny positive/odd-raw slab cores, rotated tiny cores, zero
  capsule core, admitted zero radii, parallel/perpendicular/oblique frames,
  repeated roots and deterministic ties. Assert zero-core cylinder parity.
- [ ] Cover opposite-limit origins, unrepresentable endpoints/full thickness,
  sub-raw exact frame terms, midpoint depth rounding and conceptual overflow.
  Use only transformations that preserve vertical-slab embedding for parity tests.
- [ ] Reuse warmed-allocation helpers and the existing 1 MiB worker-stack test
  pattern with a live dirty 64 KiB caller buffer. Exercise the actual deepest
  region/root-comparison path; do not assume sequential candidate buffers are
  the peak if called helpers keep scratch live.
- [ ] Rerun affected cylinder/capsule, cylinder-pair and triangle/slab tests
  after shared extraction. Review exact admission, ranking and fast-path proofs
  before downstream cutover.

### Phase 2 - Gravitas Cutover And Cleanup

- [ ] Route nonzero-core slabs to the new query and reverse its normal exactly
  once for `BuildCanonicalSupportContact`. Preserve zero-core behavior.
- [ ] Verify the original regression and raw-neighbor cases through initialized
  runtime colliders. Check anchor origins/rotations, normal orientation, depth,
  clamp/default outputs, material propagation and repeated deterministic results.
- [ ] Exercise existing compound and mixed-pair/manifold consumers with a
  meaningful contact; preserve response and dimensional ownership. Confirm pure
  2D and ordinary 3D callers are unchanged except proven shared leaf reuse.
- [ ] Delete superseded capsule-only direction checks and unreferenced wrappers
  after the caller audit. Update misleading tests to check correct behavior;
  do not retain assertions whose only purpose is to preserve removed scaffolding.
- [ ] Verify warmed zero allocation through Gravitas-owned behavior. Its tests
  and benchmarks must not call FixedMathSharp internals directly.

### Phase 3 - Performance, Coverage And Closeout

- [ ] Run matching before/after nonzero-core rows with corrected semantic
  preflights, and rerun unchanged mixed/3D/shared-owner controls. Report units
  explicitly as microseconds per query, plus allocations and run uncertainty.
- [ ] Investigate repeatable ordinary-path or shared-owner regressions before
  acceptance. Explain expensive curved cases separately; no accuracy trade-off
  or silently accepted slowdown. Seek user review of material remaining costs,
  recording accepted signals without reopening unrelated optimization work.
- [ ] Build both library TFMs and run both complete .NET 8 solution test suites
  in `Release` and `ReleaseLean`. Preserve 100% reachable line, branch and method
  coverage per repository/configuration without new exclusions or hollow tests.
- [ ] Check Debug assertions on the focused geometry/resource tests; obtain
  final independent correctness and Ponytail review and resolve all findings.
- [ ] Update affected XML, evergreen shape guidance, benchmark notes and any
  changed complexity exceptions. Build affected DocFX sites with warnings as
  errors; never link feature-work plans from evergreen wiki pages.
- [ ] Resolve GRV-Issue-088 only after the required gates pass. Consolidate final
  evidence here, move this plan to `done`, and update tracker/overview links.
  Keep unrelated issues and benchmark signals separate.

## Validation And Handoff Rules

Use C# 11, `netstandard2.1` / `net8.0`, and `UseLocalLsfStack=true` for coordinated
source builds. Do not alter package versions or project references to bypass
unreleased dependencies. Released-package validation remains a later release gate:
release FixedMathSharp first, then validate Gravitas against that exact package.

Run only one heavy workload at a time with `DOTNET_PROCESSOR_COUNT=2`, two-core
affinity and BelowNormal priority. Review agents are read-only and must not launch
competing builds, tests or benchmarks. Use serialized MSBuild with
`-m:1 -p:BuildInParallel=false -p:UseSharedCompilation=false -nr:false`.

Focused commands, from each owning repository, after applying those limits:

```powershell
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj -c Release -p:UseLocalLsfStack=true --filter 'FullyQualifiedName~CapsuleCapsuleSlab' -m:1 -p:BuildInParallel=false -p:UseSharedCompilation=false -nr:false
dotnet test tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj -c Release -p:UseLocalLsfStack=true --filter 'FullyQualifiedName~CenteredCapsuleSlabContact' -m:1 -p:BuildInParallel=false -p:UseSharedCompilation=false -nr:false
```

Repeat focused checks in Lean, then use the existing solution, coverage and
benchmark workflows in each repository's `AGENTS.md` and benchmark README.
Retain logs and coverage summaries under ignored `artifacts/grv088` directories;
summarize exact commands, source revisions, test counts, coverage counts, resource
checks and benchmark comparisons in this document. Distinguish locally verified
platforms from pending Windows/Linux CI; do not infer cross-runtime validation.

Leave changes unstaged and uncommitted for user review. No tag, push or release.
Include concise commit suggestions for each changed repository at implementation
handoff. Planning approval authorizes the scoped work, not unrelated cleanup.
