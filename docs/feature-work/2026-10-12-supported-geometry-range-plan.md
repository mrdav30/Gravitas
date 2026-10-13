# Supported Geometry Range Implementation Plan

> **For agentic workers:** Implement phase by phase in this checkout using
> `superpowers:executing-plans`, test-first changes and an independent ponytail
> review at each phase boundary. Delegate focused investigation or review when
> useful. Leave changes unstaged and uncommitted for the user's review. Record
> concise results at the end of each phase in this plan; do not create a separate
> design document or phase report.

**Date:** 2026-10-12  
**Status:** Phase 1 evidence collected; ceiling locking pending numerical
prerequisite #116; #113/#114/#115 are resolved. Mass-property parity #117/#118
is queued before admission implementation, which has not started. The bounded
effective-geometry direction is approved. The proposed
ceiling remains provisional while confirmed in-range numerical defects are
resolved; admission alone cannot establish the intended safety guarantee.  
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
   commits unit-scale runtime geometry and must enforce that budget immediately.
   Its current public constructors have no scale parameter. Preserve compensated
   collider authoring through a narrow internal deferred-commit path; do not add a
   public bypass or an unnecessary construction overload.
5. **Use candidate-local geometry, not clipped world bounds.** Reuse the existing
   conservative centered proxy formulas and ceiling-magnitude arithmetic;
   document their conservatism. Rotating a rigid shape must not change admission.
   A compact mesh translated in its authored vertex frame retains the existing
   recentering behavior and must not become huge merely because of that origin.
   This preserves intrinsic reach; if the authored center becomes a real
   body-local attachment offset, that physical lever still counts under rule 6.
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
   radius in its physical envelope. Measure it about the apex/base midpoint with
   the centered cone collider's conservative formula; the public apex anchor does
   not consume attachment budget. Ray length and travel of a bounded swept
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

### Phase 1 Admission Inventory And Measurement Decisions

The shared preparation seam covers initialization, detached/live rebuild, body
pose and reconfiguration in both dimensions. Validate prepared fields there,
before publication. Reconfiguration already uses detached candidates. Primitive
owners supply prepared dimensions; mesh preparation already caches the maximum
ceiling vertex norm and reuses it for unchanged geometry. Polygon vertex-change
preparation and compound part traversal can supply reach without a second scan.

| Family | Conservative intrinsic reach, rounded outward |
| --- | --- |
| Sphere/circle | Effective radius. |
| Capsule/stadium | Half core length plus radius, retaining an odd raw half-length. |
| Cuboid/AABB | Ceiling norm of final half-extents. |
| Cone/cylinder | Ceiling norm of `[radius, outward-rounded height/2, radius]`; intentionally the bounding-box diagonal, not the tight support radius. |
| Mesh | Maximum ceiling norm of final scaled vertices in the existing recentered local frame. |
| Polygon | Maximum ceiling norm of final scaled local vertices about the actual collider origin. |
| Compound | Maximum of ceiling part-offset norm plus that leaf's intrinsic reach. |
| Mixed slab | Ceiling norm of planar reach and physical half-thickness. |

Add effective owner attachment reach once, in the body's local frame, for a
body-owned collider. A bodyless collider uses its physical origin. Use existing
ceiling-magnitude and checked-sum arithmetic; an unrepresentable envelope rejects.
These formulas are the reviewed measurement direction; `4,096` is still a
candidate ceiling, not a claim of safe higher-order scalar arithmetic.

Four lifecycle details need more than a check at the common preparation seam:

- `LSMeshCollider` currently constructs and commits a unit-scale `PhysicsMesh`
  before host scale is available, and computes a scalar size diagonal. Reuse
  topology/authoring validation through an internal deferred runtime preparation
  path; do not reject compensated authored geometry at the wrong scale.
- Bound dimension/offset and mixed-thickness setters currently assign authored
  fields and wake the body before preparation. Preflight the proposed effective
  candidate before assignment/waking to satisfy synchronous rejection; later
  transaction failure alone cannot preserve the old awake state.
- Mutable, contextless settings currently allow pure-2D admission to acquire a
  mixed slab later. Mode/thickness/settings replacement require an owning-context
  preflight before publishing the changed configuration.
- Recorded fields are written before rebuilding. Preserve the supported runtime
  transaction boundary without claiming globally atomic arbitrary payload loads.

Public 2D AABB and world-vertex polygon queries also need admission. Measure a
query polygon about a derived local midpoint, not absolute world coordinates.
Unknown internal collider fallbacks currently expose a `MaxValue` conservative
proxy; reject unsupported runtime shape admission explicitly rather than treat
the sentinel as a valid prepared envelope.

Retain ordinary-shape/extreme-position, authored-origin translation, compensated
owner/part scale and long-travel regressions. Reframe actual giant dimensions,
standalone mesh spans and excessive compound spacing as rejection tests in
phase 2. Keep useful upstream arithmetic tests independent of runtime workload
policy; the ceiling does not remove translated-anchor defenses.

### In-Range Numerical Prerequisites

Phase 1 reproduced existing failures on the original committed binaries, below
the candidate ceiling. Repair and rerun their controls before locking the
supported safety claim or implementing around an assumed safe range:

| Tracker | Confirmed boundary | Required outcome |
| --- | --- | --- |
| [#113](issue-tracker.md#grv-issue-113---authored-closed-mesh-integrals-saturate-before-normalization) | Side-128 authored cube publishes unit inertia `1,024` instead of `2,730⅔`; side-2,048 cube accepts an unrepresentable volume as `Valid`. | Resolved: exact/checked integration, centered cache and numeric fallback; retain the existing scaled-property guard. |
| [#114](issue-tracker.md#grv-issue-114---polygon-moment-integrals-saturate-before-normalization) | Side-256 square publishes moment `2,730⅔` instead of `10,922⅔`; equivalent AABB remains correct. | Resolved: exact mass-applied normalization with geometry caching; retain the existing final-overflow policy. |
| [#115](issue-tracker.md#grv-issue-115---cone-volume-saturates-before-final-division) | Radius/height `1,024`, reach `1,536`: representable cone volume was reduced from about `1.124B` to `715.8M`. | Resolved: round the complete fixed-Pi product/ratio once; saturate only genuinely unrepresentable final volume. |
| [#116](issue-tracker.md#grv-issue-116---cone-overlap-gjk-loses-scale-and-rotation-invariance) | Length `4,096`, end radius `2,048`: public cone overlaps produce both false positives and false negatives. | Preserve bounded overlap classification, both query variants and corresponding contact callers. |

These are prerequisites, not optimizations and not bugs introduced by admission.
Do not reduce the candidate ceiling to hide failures at ordinary useful sizes.
Resolve coherent numerical domains together where ownership permits, retaining
each regression. Recheck representative mass/shape ratios independently of size;
even an admitted sphere can have unrepresentable inertia at excessive mass.

The oblique host-scale probe also confirmed FixedMathSharp's existing strict
decomposition limitation, recorded as `FMS-Issue-029` in its own tracker. Uniform
scale `2,048` amplifies quaternion reconstruction rounding beyond the fixed
absolute matrix tolerance although the source matrix is representable and its
normalized basis passes. This follows the current round-trip contract; do not
claim a new geometry miss or silently loosen shear checks here. Preserve this
boundary in subsequent compensated-scale tests and coordinate any upstream
contract revision separately.

## Phase 1: Validate And Lock The Supported Range

**Files:** This plan, `issue-tracker.md`, existing
`tests/Gravitas.Tests/Colliders/ColliderExactScaleAdmissionTests.cs`, shape-pair
and mixed query tests; reuse `MeshMassPropertyBenchmarks`,
`RotationalCcdIntervalBenchmarks` and maintained curved-contact benchmarks.

- [x] Inventory every runtime geometry-admission route and classify existing
      enormous-geometry tests separately from ordinary-shape/extreme-position
      and upstream arithmetic regressions. Include detached collider rebuilds,
      standalone `PhysicsMesh`, custom/internal collider fallback semantics,
      bodyless geometry, reconfiguration, settings and recording paths.
- [x] Trace `LSMeshCollider` construction: its current `PhysicsMesh` constructor
      prepares and publishes unit-scale geometry before collider binding. Specify
      the narrow internal authored-data construction/preparation path needed to
      preserve compensated mesh scale while keeping public standalone runtime
      construction validated. Preserve existing authoring validity checks.
- [x] Restore #096's original failure as a controlled baseline experiment and
      its ordinary-radius control. Record the failing layer without adding a
      new full-domain contact algorithm. Keep the final maintained test focused
      on production rejection of unsupported effective geometry.
- [x] Exercise the candidate reach across all shape families, compounds and
      mixed slabs: separated/tangent/penetrating contacts, initial and entering
      sweeps, oblique rotations, nonuniform scale, thin finite features and both
      pair orders. Record exact feature sizes/tolerances; do not infer arbitrary
      one-raw-feature support from passing ordinary geometry.
- [x] Check primitive area/inertia and existing mesh mass-property admission at
      the candidate size with representative masses and mass ratios. Record
      independent mass, triangle multiplicity and cell-size limits; repair any
      newly introduced failures within the phase. The geometry ceiling cannot
      stand in for existing representability validation.
- [x] Measure ordinary controls and candidate-boundary controls separately, with
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

**Phase summary (2026-10-12):** Evidence collection is complete; independent
geometry/ponytail review agrees that ceiling locking remains pending #113-#116.
No production admission, optimization or dependency-pin changes were made.

- #096 reproduces on the original binaries; its radius-1,024 control with a
  one-raw-unit core reports initial contact at distance zero.
- **726** ignored family-probe assertions pass across seven 3D/five 2D families,
  compounds and five mixed slab families, reaching `4,096`. Penetrating/separated
  truth, pair-order parity and entering/initial sweeps retain normalized
  depth/distance within `1e-4`. Selected `1/64`-unit features and billion-unit
  compensated authoring controls pass. Oblique tangencies are observed with
  materialization uncertainty, not universally asserted. Probes use no active
  grid and establish no partition-capacity promise.
- Primitive inertia controls at representative reaches with masses `1/16/64`
  are positive/unsaturated; largest coefficient/mass-share error is about `0.172`
  on `320.6M`. Sphere reach `4,096`, mass `512` independently exceeds scalar
  inertia representation and saturates. The mesh/polygon/cone failures above
  prevent a blanket numerical guarantee; public cone tests confirm #116 with
  72 internal classifications and 18 closest/all comparisons.
- **16** maintained native BDN rows complete, all raw **0 B/op**. The two new
  rotation rows reuse existing fixtures and precomputed poses; independent
  review corrected the compound checksum to observe a changing minimum bound.

| Original production baseline | Mean |
| --- | ---: |
| Move mesh / concave mesh / compound | `27.29 / 49.26 / 66.93 µs` |
| Rotate unchanged mesh / compound | `26.73 / 68.71 µs` |
| Full `Simulate` + `LateSimulate`, 64 spheres | `461.61 µs` |
| 64 non-SAT / cone primitive contacts | `2.215 / 1.197 ms` |
| Cone overlap / sphere sweep, 64 targets | `2.5808 ms / 781.6 µs` |
| Nonuniform mesh rebuild, subdivision `1/8/16` | `46.65 µs / 2.445 ms / 9.645 ms` |
| Cached mesh inertia, subdivision `1/8/16` | `48.26 / 48.32 / 48.84 ns` |

Native Windows x64, .NET `8.0.29`, SDK `10.0.302`, BDN `0.15.8` ShortRun uses
three warmups/measurements. Later ambiguous differences require matched longer
runs, each row an independent gate. Pose/world-step grid: one-unit dense cells,
scan eight, bounds `[-4,-4,-4]` to `[20,4,20]`. Separate sphere boundary diagnostics
at reach `1/4,096` use one dense grid, scan eight, extent `4*reach`, cells `reach/4`.
After 16,384 warm calls, seven 2,000-operation batches allocate zero bytes;
contact medians `8.51/3.34 µs`, public sweep `38.10/39.64 µs`. These are diagnostic
timings, not BDN comparisons or improvement claims.

Release/Lean solution builds pass with zero warnings/errors. Full local-stack
suites pass **5,799 / 5,734**, no skips/failures. Raw reachable line/branch/method
coverage is **100%**: zero unvisited among Release `47,203 / 15,464 / 4,925` and
Lean `47,201 / 15,464 / 4,924` sequence points/branches/methods.

Ignored `artifacts/geometry-range/phase1/` retains original `baseline-bench`
binaries, source/DLL provenance, production probes, native reports, exact
commands/settings, deviations/checksums, boundary diagnostics and coverage.
Source: Gravitas `9752c9f`, FixedMathSharp `819dbde`; original/measurement core
SHA256 `71DA140C72B047918EEEA531C26796C79274263CADD7B8E8641BE9B357E92539`.
No native ARM64 or package-release conformance claim follows. Keep phase 1 open;
repair its numerical prerequisites before approving the ceiling/enforcement.

**Integral refinement (2026-10-13):** #113/#114 are resolved. Full correctness,
coverage, replay and paired warm-performance gates pass.
Mesh determinant/first/second integrals now stay wide until final normalization,
reusing the existing triangle polynomials. The centered source cache also fixes
an authored-translation half-tie discrepancy and removes obsolete absolute
source scaling/recentering. Source volume/tensor or cached scaling failures
retry the actual final geometry, preserving compensated admission. Polygon
area/centroid/moment ratios refresh with effective geometry and reuse committed
values for pose changes; mass is applied before final moment rounding.

No new source files, dependencies, numeric owners or upstream edits are needed;
the production diff is two net added lines. New production regressions cover
analytic cubes/asymmetric tetrahedra, both windings, large translations/raw ties,
tiny/large compensated source geometry, true final overflow, small polygon mass,
compound/AABB parity, changed scale/mass, rollback and warm allocation behavior.
Release/Lean solution builds pass with zero warnings/errors; suites pass
**5,821 / 5,756**. Raw coverage remains **100%**: Release
`47,175 / 15,464 / 4,929`, Lean `47,173 / 15,464 / 4,928` sequence points,
branches and reachable methods, all visited. Six shared replay captures agree
with reviewed expectations and directly across native Windows x64 Release/Lean.
Evidence is under ignored `artifacts/grv-issues-113-114/`; paired performance
artifacts are under `artifacts/grv-113-114/perf/`. Original source is `a5fff89`,
fixed DLL SHA256 `CED874321E77CA5005BE9B5FC13E515CE739662AD260410724067FD8B0CA7BF9`.
Paired native Windows x64/.NET 8.0.29 measurements execute 18 rows per snapshot,
all **0 B/op**. Two-launch repeats clear ambiguous pose/world/scale rows within
measurement uncertainty; none shows a repeatable slowdown. The largest mesh
rebuild row uses five warmups/six measurements with 128 operations per iteration:
`9.6335 -> 9.5950 ms`. This supersedes an underwarmed adaptive run with only four
operations per iteration, which was inconclusive rather than regression evidence.
Polygon pure-2D/mixed cached moments measure `2.97 -> 1.01/0.99 us`, pose changes
`18.99/19.03 -> 11.29/11.07 us`, scale changes `11.83/12.18 -> 7.87/7.95 us`.
These incidental gains come from reusing the integrals needed by the fix.
Independent numerical/ponytail review has no outstanding findings. No native
Linux/ARM64 or released-package validation is claimed by this local run.
The candidate ceiling and #096 remained open at this checkpoint; cone
prerequisites are refined below.

**Cone-volume refinement (2026-10-13):** #115 is resolved. Cone `Volume`/`Area`
uses the existing four-factor product/ratio API with one final nearest-even
rounding. Genuine final overflow saturates the diagnostic scalar, preserving
valid geometry and separate unnarrowed compound mass weighting. Effective
radius/height and exact committed rotation reuse existing volume/axis state.
Body-owned mass-point validation runs once in the shared transaction; bodyless
and compound validation remains intact. No production files, types or fields
are added; the coordinated production diff is **39 net lines**.

FixedMathSharp's public helper now reuses existing common-binary-factor
normalization only when a nonzero numerator shares a cancellable low limb.
The typed fallback and rounding core are unchanged. A private non-inlined leaf
isolates normalization scratch; unconditional or inlined span dispatch was
rejected after slowing the noncancellable control. Final matched means are
`361.57 -> 197.23 ns` for the cancellable case and `401.51 -> 403.59 ns` for
the control, whose confidence intervals overlap. Fresh matched cone scale
updates measure `42.68 -> 40.50 us` in 3D and `42.55 -> 40.43 us` in mixed mode.
All **10** final physics rows and both arithmetic controls allocate **0 B/op**
and preserve the warm gate. Broader physics timing gains may reflect host drift;
they are not attributed to this change.

Release/Lean solution builds pass with zero warnings/errors using
`UseLocalLsfStack=true`. Full Gravitas suites pass **5,835 / 5,770**;
FixedMathSharp passes **5,726 / 5,705**, plus **49** Chronicler tests per lane.
Raw reachable line/branch/method coverage remains **100%** in Gravitas,
FixedMathSharp, its Chronicler adapter and FluentAssertions helper in both lanes.
Gravitas counters are `47,186 / 15,478 / 4,929` and
`47,184 / 15,478 / 4,928`; FixedMathSharp counters are
`59,523 / 16,902 / 4,472` and `59,515 / 16,902 / 4,468`, all visited.
Six shared replay fixtures agree with expectations and directly across native
Windows x64 Release/Lean. Independent numerical/ponytail review has no unresolved
findings in this change.
Ignored `artifacts/grv-115/` retains red regressions, final coverage/replay,
source snapshots and rejected experiments; `perf/` retains native reports and
immutable runtime hashes. The final Gravitas DLL SHA256 is
`51DF2FA669DECB63C859C841EA902FBFBCA889F390E28B7B647AC24B3B4DE9C1`.
No native Linux/ARM64 or released-package validation is claimed. Caller/parity
review also confirmed pre-existing detached cone mass-weight mismatch #117 and
sphere inertia's early tiny-mass rounding #118 on original binaries; both are
recorded separately. #116, the candidate ceiling and #096 remain open; resolve
#117/#118 together before admission implementation.

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
