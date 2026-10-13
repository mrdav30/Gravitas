# Supported Geometry Range Implementation Plan

> **For agentic workers:** Implement phase by phase in this checkout using
> `superpowers:executing-plans`, test-first changes and an independent ponytail
> review at each phase boundary. Delegate focused investigation or review when
> useful. Leave changes unstaged and uncommitted for the user's review. Record
> concise results at the end of each phase in this plan; do not create a separate
> design document or phase report.

**Date:** 2026-10-12  
**Status:** Planned; the bounded effective-geometry direction is approved.
The numerical ceiling remains subject to phase 1 evidence. Implementation has
not started.  
**Owner:** Gravitas geometry admission and lifecycle transactions.

**Goal:** Close `GRV-Issue-096` with explicit, consistent admission of supported
physical geometry while preserving ordinary shapes at large world coordinates.

**Architecture:** Measure effective local geometry after exact scale composition
and before publication through the existing shape transactions. Reuse canonical
bounds, mesh preparation, dimensional lifecycle owners and query validation.
Keep the supported-workload policy in Gravitas; retain FixedMathSharp's reusable
full-domain arithmetic and geometry.

**Tech stack:** C# 11, `netstandard2.1`/`net8.0`, FixedMathSharp,
SwiftCollections, GridForge, Chronicler, xUnit v3 and BenchmarkDotNet.

**Design:** The contract below is the design source for this implementation.
This plan supersedes #096's open supported-range decision, not its reproduction.

## Evidence And Scope

[#096](issue-tracker.md#grv-issue-096---extreme-scale-capsule-initial-mesh-sweep-can-miss-a-genuine-overlap)
uses capsule radius `Fixed64.FromRaw(long.MaxValue / 2)`, full height
`Fixed64.MaxValue` and a triangle with large authored X coordinates but only a
`5`-unit Y span and `2`-unit Z span. A genuine initial overlap is rejected before
contact-normal resolution. The ordinary-radius
control passes. The reproducer and ignored evidence remain recorded in the
tracker; restore a maintained regression for the new admission behavior.

Representable input dimensions do not establish a supported physics workload.
Scalar squared quantities, area, inertia and bounded GJK working coordinates
have different numerical requirements. Large geometry also increases partition
coverage independently of its mathematical representability.

This work establishes an honest geometry boundary. It does not promise arbitrary
mass, speed, aspect ratio, one-raw-unit features or cheap voxel coverage at every
cell size. It does not reopen manifold algorithms, redesign response, replace
GridForge, change the coordinate representation or restrict FixedMathSharp's
general-purpose APIs. #111's generated benchmark-host references remain separate;
coordinate measurement execution with that issue if it blocks an existing row.

## Supported Contract

1. **Separate position, intrinsic size and attachment reach.** World centers and
   origins retain existing representability rules. Ordinary geometry near either
   scalar limit retains translated-frame and anchor behavior, including conceptual
   supports outside the absolute scalar domain. Actual body-position exhaustion
   continues to fail explicitly.
2. **Bound the final physical envelope.** Start phase 1 with `4,096` world units
   as a candidate maximum conservative center-to-surface reach. It is a workload
   candidate, not a numerical proof or an unconditional final value. The budget is
   inclusive; a measured reach one raw increment above it rejects. Units belong
   to the host: `4,096` metres is about a `8,192`-metre sphere diameter only when
   the host chooses metres consistently throughout its simulation.
3. **Use one reviewed library ceiling.** Publish the approved value through
   existing dimensional collider settings, with one internal source of truth.
   Do not introduce runtime profiles, mutable global limits or an unlimited
   override. Lower host budgets can remain host validation until a concrete use
   requires a library setting.
4. **Compose scale exactly before admission.** Preserve separate owner/part
   factors and existing fused final rounding. Huge authored dimensions or scales
   may produce admitted small geometry. Positive-scale and nondegeneracy checks
   remain in force. Raw authored size, scale factors and absolute vertex
   coordinates are not independent size-limit tests. Collider-owned authored
   mesh construction must not acquire a unit-scale size check before its actual
   host/part scale is available. Public standalone `PhysicsMesh` construction
   commits runtime geometry and must enforce the budget at its supplied scale.
5. **Use candidate-local geometry, not clipped world bounds.** Reuse the existing
   conservative centered proxy formulas and ceiling-magnitude arithmetic;
   document their conservatism. Rotating a rigid shape must not change admission.
   A compact mesh translated in its authored vertex frame retains the existing
   recentering behavior and must not become huge merely because of that origin.
6. **Account for the owning object's complete reach.** A standalone body-owned
   shape includes its effective body-local attachment offset; a bodyless shape
   uses its physical collider origin. A compound includes every effective part
   offset plus that part's centered reach, then its owner attachment offset.
   Compute these offsets in the existing local frames, without subtracting
   absolute world centers. Small leaves cannot bypass admission through enormous
   spacing. Bounding sums may conservatively overestimate the actual envelope;
   they are the documented admission metric, not a minimum enclosing sphere.
7. **Keep dimension parity explicit.** Apply the same reach budget to 3D and pure
   2D. In mixed mode include the finite slab's Y half-thickness in its conservative
   3D envelope. Pure 2D admission must not depend on unused physical Y thickness.
8. **Cover query geometry.** Public overlap and swept-source shapes share the
   applicable reach budget. A finite cone query includes its length and end
   radius in its physical envelope. Ray length and the travel of a bounded swept
   source remain separate from source size and retain their existing contracts.
9. **Reject before publication.** Unsupported geometry produces a descriptive
   argument/validation failure using existing API conventions. It cannot become
   a collision miss, a smaller clamped shape or a clearance `Blocked` result.
   Failed live candidates preserve committed shape, body state, derived mass,
   IDs, bounds, partitions and retained caches. Host-authored transform data
   remains host owned; rollback guarantees concern the existing runtime transaction.
10. **Keep upstream math and valid defenses.** A cap does not eliminate overflowing
    world differences, extreme-position anchors or all derived scalar limits.
    Remove code only when every remaining production caller proves it unnecessary.

## Owners And Reuse Map

| Owner | Planned responsibility |
| --- | --- |
| `src/Gravitas/Colliders/ColliderScalePolicy.cs` | Preserve exact scale composition and positive final dimensions. |
| `src/Gravitas/Colliders/ColliderCanonicalBounds.cs` and `ColliderCanonicalBounds2D.cs` | Reuse centered reach math; provide candidate measurements without relying on committed state. |
| `src/Gravitas/Colliders/ColliderGeometryPolicy.cs` (new only if needed for coherent ownership) | Shared approved ceiling and argument validation; no duplicate wide arithmetic or geometry engine. |
| `src/Gravitas/Colliders/Definitions/ColliderSettings.cs` and `ColliderSettings2D.cs` | Expose read-only `MaximumGeometryReach` from the shared policy. |
| `src/Gravitas/Colliders/3D/LSCollider.ShapeTransaction.cs` and `2D/LSCollider2D.ShapeTransaction.cs` | Validate prepared reach before publication; preserve initialization, pose and rebuild transactions. |
| Primitive, polygon and compound owners under `src/Gravitas/Colliders` | Supply prepared dimensions and complete effective reach using current shape semantics. |
| `src/Gravitas/Colliders/Mesh/PhysicsMesh.Scale.cs` and `3D/LSMeshCollider.cs` | Validate centered scaled vertices during the existing preparation pass; cover standalone mesh transformation as well as runtime colliders. |
| `src/Gravitas/Queries/2D`, `3D` and `Mixed` | Validate public query geometry before tracing, diagnostics or result-buffer mutation. |
| `src/Gravitas/Settings/PhysicsSettings.cs`, 2D mixed-thickness ownership and record/load paths | Apply the mixed envelope rule and prevent load/settings routes bypassing candidate admission. |
| Existing collider, query, reconfiguration, serialization, CCD and replay tests | Prove production admission, rollback, large-position behavior and dimensional parity. |
| Existing benchmark owners and support | Measure ordinary accepted paths, scale rebuilds and workloads at the candidate boundary. |

Before adding prepared fields, reuse values already computed by `PrepareShape`
or mesh vertex preparation. `CanonicalCenteredProxyRadius` is currently computed
in `PublishPreparedShape`; reading it before publication checks old geometry,
while validating after publication breaks atomic rejection. Do not add a second
mesh scan or recompute geometry on every unchanged simulation frame. Changes to
rigid pose can reuse admission only after existing hierarchy/pose checks confirm
unchanged effective owner/part scales, dimensions/vertices, attachment offsets
and applicable mixed thickness/runtime mode. Changes to those inputs invalidate
the envelope; a child rotation under nonuniform ancestry can change scale or
introduce shear. Reuse existing snapshot/geometry-change ownership, not a new
cache framework.

## Global Constraints And Review Focus

- Preserve deterministic fixed-point decisions, explicit stable traversal and
  existing public contact/query semantics for admitted geometry.
- Preserve the current warm performance baseline as a hard phase gate. Require
  no added warmed allocations and no repeatable slowdown beyond measurement
  uncertainty in any affected ordinary control; a faster fixture cannot offset
  a slower one. Baseline accepted contacts, queries, moving/rotating-body stepping
  and unchanged mesh/compound geometry before implementation. Reuse validated
  geometry for pose-only changes and introduce no recurring mesh scan, magnitude
  calculation or new per-contact admission layer. Improvements are welcome but
  must be measured; the size restriction alone promises none.
- Use inherited and MSBuild `UseLocalLsfStack=true` for development, tests and
  generated benchmark hosts. Do not edit dependency pins just to run locally.
- Keep 100% raw reachable line, branch and method coverage in Release and
  ReleaseLean for every changed library. Do not exclude admission or arithmetic
  paths to restore the percentage.
- Require independent ponytail review at each phase. Add focused geometry review
  for range/rounding decisions; resolve introduced defects and regressions within
  their phase instead of adding them to a future backlog.
- Record independently confirmed pre-existing defects and measured performance
  signals in their owning trackers. Unmeasured concerns are not new issues.
- Add comments for non-obvious rounding, frame and transaction decisions. Main
  partial class/struct files retain their required XML `<content>` tag. Extra work
  solely for assertions belongs inside `#if DEBUG`.
- Leave all changes unstaged/uncommitted; provide a recommended commit message at
  each review handoff. The user owns commits and pushes.

Review the five likely bypasses explicitly: compensated huge authoring values;
world-bound clipping near both coordinate limits; compound/attachment spacing;
queries and mixed thickness outside collider preparation; and rejected live or
recorded mutations after derived caches have been prepared. Their tests belong
to phases 1, 2 and 3 below, not to a separate test framework.

## Phase 1: Validate And Lock The Supported Range

**Files:** This plan, `issue-tracker.md`, existing
`tests/Gravitas.Tests/Colliders/ColliderExactScaleAdmissionTests.cs`, shape-pair
and mixed query tests; reuse `MeshMassPropertyBenchmarks`,
`RotationalCcdIntervalBenchmarks` and maintained curved-contact benchmarks.

- [ ] Inventory every runtime geometry-admission route and classify existing
      enormous-geometry tests separately from ordinary-shape/extreme-position
      and upstream arithmetic regressions. Include detached collider rebuilds,
      standalone `PhysicsMesh`, custom/internal collider fallback semantics,
      bodyless geometry, reconfiguration, settings and recording paths.
- [ ] Trace `LSMeshCollider` construction: its current `PhysicsMesh` constructor
      prepares and publishes unit-scale geometry before collider binding. Specify
      the narrow internal authored-data construction/preparation path needed to
      preserve compensated mesh scale while keeping public standalone runtime
      construction validated. Preserve existing authoring validity checks.
- [ ] Restore #096's original failure as a controlled baseline experiment and
      its ordinary-radius control. Record the failing layer without adding a
      new full-domain contact algorithm. Keep the final maintained test focused
      on production rejection of unsupported effective geometry.
- [ ] Exercise the candidate reach across all shape families, compounds and
      mixed slabs: separated/tangent/penetrating contacts, initial and entering
      sweeps, oblique rotations, nonuniform scale, thin finite features and both
      pair orders. Record exact feature sizes/tolerances; do not infer arbitrary
      one-raw-feature support from passing ordinary geometry.
- [ ] Check primitive area/inertia and existing mesh mass-property admission at
      the candidate size with representative masses and mass ratios. Record
      independent mass, triangle multiplicity and cell-size limits; repair any
      newly introduced failures within the phase. The geometry ceiling cannot
      stand in for existing representability validation.
- [ ] Measure ordinary controls and candidate-boundary controls separately, with
      explicit cell sizes, active-grid coverage, warmup and allocation counts.
      Retain the original binaries/source revisions, settings and observable
      result checksums for matched before/after comparisons. Include pose-only
      translation/rotation of unchanged mesh and compound geometry, not just
      sleeping or motionless controls. Compare each row independently and rerun
      ambiguous differences under matched host/JIT conditions.
      Use native out-of-process benchmark evidence where the existing host works;
      label diagnostic timings honestly if #111 blocks a row.
- [ ] Lock the ceiling, conservative shape formulas, inclusive rounding rule,
      attachment frame and mixed-slab formula in this contract after focused
      geometry and ponytail review. If evidence requires changing the approved
      direction or a materially different workload range, bring that decision
      back to the user before enforcement.

**Exit:** A reviewed numerical range and measurement contract with baseline
evidence. `4,096` remains provisional until this exit; no supported-safety claim
is based solely on its proximity to Q32.32 square limits.

**Phase summary:** Pending.

## Phase 2: Enforce Complete Collider Admission

**Files:** Shared policy/settings, canonical bounds, dimensional shape
transactions, primitive/polygon/compound preparation and mesh scale owners from
the reuse map. Extend existing collider, body-pose, reconfiguration and scale tests.

**Interfaces:** One shared `MaximumGeometryReach`; internal
`ValidateReach(Fixed64 reach, string parameterName)` argument validation and
candidate-local measurements supplied by existing dimensional owners. Reuse
FixedMathSharp ceiling magnitudes and checked sums for effective offset plus
centered reach. No internal wide types enter public APIs or serialized layouts.

- [ ] Add failing production tests for the locked limit, one raw increment above
      it, and oversized dimensions that scale down to admitted geometry. Cover
      overflowed intermediate scale products with representable final results.
      Include primitive, mesh and polygon authoring controls, with shrinking host
      and compound-part scales applied after construction.
- [ ] Implement candidate reach and shared validation before publication across
      3D, 2D and mixed colliders, including the full compound/body-local envelope.
      Validate mesh vertices within existing preparation and cached rebuild logic;
      reject unsupported standalone mesh transforms as well as collider updates.
- [ ] Separate collider-owned authored mesh construction from standalone mesh
      runtime commitment through narrow internal reuse. Do not expose a public
      validation bypass or publish unsupported runtime geometry while waiting
      for the effective collider scale. Share topology/preparation ownership;
      do not create a second mesh representation or engine.
- [ ] Prove near-limit clipping cannot hide oversized geometry. Keep ordinary
      colliders near both scalar limits, authored-origin-recentered compact meshes,
      rotation invariance and compensated owner/part scale controls admitted.
- [ ] Prove initialization and failed live pose/scale/reconfiguration candidates
      preserve IDs, committed geometry, mass, bounds, partitions, awake state and
      cache identity. A small leaf with excessive attachment/part separation must
      reject. Test synchronous public API behavior, not private helper behavior alone.
- [ ] Replace deliberately unsupported giant runtime fixtures with explicit
      admission regressions. Retain or relocate useful math/large-position coverage
      through supported production behavior; remove dead branches only after a
      caller audit proves they have no admitted use.
- [ ] Run full Release/Lean coverage and affected warmed allocation/performance
      comparisons. Resolve introduced failures and obtain independent ponytail
      review before completing the phase.

**Exit:** All collider families and standalone mesh transformation enforce the
reviewed budget atomically. Query/load coverage remains explicitly pending;
#096 is still open.

**Phase summary:** Pending.

## Phase 3: Close Query, Settings And Restore Bypasses

**Files:** Public query entry points and common shape preparation under
`src/Gravitas/Queries`, mixed thickness settings/overrides, existing record/load
owners; tests under `Queries`, `Physics2D`, `MixedDimensions` and `Serialization`.

- [ ] Add failing tests for oversized overlap radii, swept-source sizes, finite
      cone volumes and mixed slabs through closest/all-hit/batch variants where
      exposed. Verify rejection precedes tracing and output/diagnostic mutation;
      for batches retain documented caller-buffer and per-request failure semantics.
- [ ] Apply shared geometry admission to query sources and mixed thickness.
      Preserve zero-radius/degenerate input rules that differ by existing query
      API; do not silently tighten them as a side effect of the maximum limit.
- [ ] Prove long rays and long travel of small swept sources remain admitted under
      existing endpoint/distance rules, including ordinary shapes near both world
      limits. Cover 2D, 3D and mixed counterparts and finite cone length/radius.
- [ ] Audit settings changes and every recorded/load path that can change effective
      geometry. Unsupported candidates reject through the existing ownership
      transaction; verify no invalid geometry reaches partitions or solver caches.
      Do not claim arbitrary Chronicler payload application is globally atomic:
      test and document the actual supported load boundary and preserve saved
      sleep/caches for successful loads according to current contracts.
- [ ] Cover effective-size-preserving restored records, oversized restored state
      and failed mixed-thickness mutations. Preserve existing serialization layout
      and hashes unless an implemented state change genuinely requires a version
      change; document any intentional compatibility break.
- [ ] Run full Release/Lean raw coverage, replay fixtures and affected allocation
      gates; obtain independent ponytail review. Resolve every bypass introduced
      by the new contract before phase completion.

**Exit:** Public query, mixed settings and restored-state paths cannot admit
unsupported effective geometry. Existing travel, world-position and load
semantics remain explicit and tested.

**Phase summary:** Pending.

## Phase 4: Compatibility, Performance And #096 Closeout

**Files:** This plan, `issue-tracker.md`, evergreen `docs/wiki/DIMENSIONS.md`,
`HOST_INTEGRATION.md` and `QUERY_SERVICES.md`, `docs/MIGRATION.md`, README/API
guidance when needed, and existing benchmark/replay evidence.

- [ ] Publish the final metric, ceiling, units, rejection behavior and migration
      advice. Explain terrain chunking, host unit conventions, compound offsets,
      retained large-world positioning and independent mass/feature/cell-size
      constraints. Wiki guidance must stand alone without plan/artifact references.
- [ ] Validate a representative terrain-chunk boundary migration against its
      continuous-surface control before claiming equivalent physical behavior.
      Chunking can add collider identities, contact boundaries and pair overhead;
      report those costs and any confirmed pre-existing seam issue. Do not add a
      cross-collider welding framework merely to advertise chunking as free.
- [ ] Compare accepted ordinary initialization, unchanged stepping, geometry
      rebuilds and representative queries with phase 1 controls. Require no added
      warmed allocations and no repeatable ordinary-path regression outside
      measurement uncertainty. Refine introduced costs now; a numerical maximum
      is not an advertised real-time workload size.
- [ ] Complete both solution builds, core target frameworks, raw 100% reachable
      line/branch/method coverage, full-loop replay and the affected CCD/response
      allocation gates. Do not reduce coverage obligations by excluding the
      unsupported-geometry rejection paths.
- [ ] Obtain final independent ponytail and focused geometry reviews. Summarize
      code retained/removed, new runtime footprint, remaining boundaries and
      unchanged upstream ownership. Record native Windows/Linux x64/ARM64 CI
      results when available; local or emulated runs cannot imply native conformance.
- [ ] Close #096 only after its reproducer rejects explicitly before entering
      narrow phase, every admission route is covered, and admitted large-position
      regressions remain green. Keep package-based validation as a release gate;
      release any genuinely necessary upstream math change before Gravitas.
- [ ] Mark complete and archive this plan under `done/`, updating incoming links.
      Add concise dated validation to the phase summaries; keep volatile counts
      and artifact paths out of evergreen documentation.

**Exit:** #096 is resolved by an implemented and documented supported-workload
contract, not by hiding a miss or declaring the original input irrelevant.

**Phase summary:** Pending.

## Verification Commands

Set the property for child builds as well as direct commands:

```powershell
$env:UseLocalLsfStack = 'true'
dotnet restore Gravitas.slnx -p:UseLocalLsfStack=true
dotnet build Gravitas.slnx -c Release -p:UseLocalLsfStack=true --no-restore
dotnet build Gravitas.slnx -c ReleaseLean -p:UseLocalLsfStack=true --no-restore
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj -c Release -p:UseLocalLsfStack=true --no-restore --collect:'XPlat Code Coverage' --settings tests/Gravitas.Tests/coverlet.runsettings --results-directory artifacts/geometry-range/release -- RunConfiguration.TreatNoTestsAsError=true
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj -c ReleaseLean -p:UseLocalLsfStack=true --no-restore --collect:'XPlat Code Coverage' --settings tests/Gravitas.Tests/coverlet.runsettings --results-directory artifacts/geometry-range/lean -- RunConfiguration.TreatNoTestsAsError=true
```

Use fresh result directories per validation pass. Inspect raw OpenCover visited
sequence points, branches and methods, not rounded report percentages. Run the
existing shared replay fixtures with recorded source/dependency/process provenance
and compare actual captures through `.github/scripts/compare_replay_captures.py`.
Record exact benchmark commands, host/runtime, inputs and uncertainty in the
owning phase summary; never label a zero-executed benchmark host as a pass.
