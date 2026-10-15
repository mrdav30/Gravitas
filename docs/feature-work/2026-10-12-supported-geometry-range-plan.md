# Supported Geometry Range Implementation Plan

> **For agentic workers:** Implement phase by phase in this checkout using
> `superpowers:executing-plans`, test-first changes and an independent ponytail
> review at each phase boundary. Delegate focused investigation or review when
> useful. Leave changes unstaged and uncommitted for the user's review. Record
> concise results at the end of each phase in this plan; do not create a separate
> design document or phase report.

**Date:** 2026-10-12  
**Contract locked:** 2026-10-14  
**Status:** Phase 1 contract review complete: the inclusive `4,096`-unit
conservative reach and measurement rules below are locked. Numerical
prerequisites #113-#124 and the upstream scaled-decomposition prerequisite are
resolved. Phase 2 collider admission is complete and passes correctness,
coverage, replay and warm performance validation. Query, settings
and restore admission remain phase 3; #096 stays open until the complete
contract is implemented and verified. The ceiling is a supported-workload
boundary, not a proof that every derived physics quantity is representable.  
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
mass, speed, aspect ratio, COM/inertia-reference overrides, one-raw-unit features
or cheap voxel coverage at every cell size. Those retain their existing independent
representability and solver contracts. It does not reopen manifold algorithms, redesign response, replace
GridForge, change the coordinate representation or restrict FixedMathSharp's
general-purpose APIs. #111's generated benchmark-host references remain separate;
coordinate measurement execution with that issue if it blocks an existing row.

## Supported Contract

1. **Separate position, intrinsic size and attachment reach.** World centers and
   origins retain existing representability rules. Ordinary geometry near either
   scalar limit retains translated-frame and anchor behavior, including conceptual
   supports outside the absolute scalar domain. Actual body-position exhaustion
   continues to fail explicitly.
2. **Bound the final physical envelope.** The maximum conservative reach is
   `4,096` world units, inclusive, using the measurement rules below. A measured
   reach one raw increment above it rejects; do not add an epsilon allowance.
   This is an evidence-backed workload boundary, not a blanket numerical proof.
   Units belong to the host: `4,096` metres permits an `8,192`-metre unattached
   sphere diameter only when
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
   document their conservatism. A rigid pose change with unchanged prepared
   local geometry must not change admission. Independently rounding rotated
   world-vertex inputs creates slightly different geometry and may change the
   result at the exact cutoff.
   A compact mesh translated in its authored vertex frame retains the existing
   recentering behavior and must not become huge merely because of that origin.
   This preserves intrinsic reach; if the authored center becomes a real
   body-local attachment offset, that physical lever still counts under rule 6.
6. **Account for the owning object's complete reach.** Measure a body-owned
   shape from the body pose origin, independently of derived or overridden COM.
   A bodyless root uses its physical collider origin. The additive metric below
   includes effective part offsets and adds owner attachment once. Measure exact
   owner-scaled offset products in local frames before component rounding or
   world rotation; subtracting materialized world centers can lose reach.
   Small leaves cannot bypass admission through enormous spacing. This deliberately
   conservative metric does not cancel opposed owner/part offsets and is not a
   minimum enclosing sphere.
7. **Keep dimension parity explicit.** Apply the same reach budget to 3D and pure
   2D. Compute the complete planar reach first, then include the finite slab's
   physical, unscaled Y half-thickness through the mixed formula below. Absolute
   host-transform Y is embedding position, not attachment reach. Pure 2D admission
   must not depend on unused thickness; explicit mixed queries must validate the
   slab they use even when the runtime mode is not `Mixed`.
8. **Cover query geometry.** Public overlap and swept-source shapes share the
   applicable reach budget. A finite cone query includes its length and end
   radius in its physical envelope. Measure it about the apex/base midpoint with
   the centered cone collider's conservative formula; the public apex anchor does
   not consume attachment budget. Its existing endpoint-representability checks
   still apply. World-vertex polygon queries use the exact arithmetic vertex mean
   for admission, with wide differences and outward distances, rather than a
   world-axis AABB midpoint. Ray length and travel of a bounded swept source remain
   separate from source size and retain their existing contracts.
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

### Locked Admission Metric

Let `q = 2^-32`. Define `C(v)` as the least nonnegative Q32.32 value whose
exact square encloses the exact squared norm of local vector `v`. Retain exact
scaled attachment products until this outward norm; nearest-even projection of
each offset component must not hide a fractional-raw excess at the ceiling.
Effective shape dimensions and vertices retain their existing fused nearest-even
materialization. Outward measurement bounds that prepared geometry, without
changing it. Unrepresentable norms or checked sums reject rather than saturate.

For a leaf let `R` be its intrinsic reach from the table below. A compound's
intrinsic reach is `max_i(C(partOffset_i) + R_i)`. Let `A` be
`C(ownerAttachment)` for a body-owned root, or zero for a bodyless root. Its
complete planar or spatial reach is `P = A + intrinsicReach`. All additions are
checked, and the root is admitted exactly when `P <= 4,096`. This policy
deliberately rejects owner offset `+3,000`, part offset `-3,000`, radius `1`:
its metric is `6,001` despite cancellation in the final leaf position. Hosts can
normalize their authored frames instead of depending on offset cancellation.

For mixed 2D geometry, admit exactly when `C([P, h]) <= 4,096`, where `P` is
the **complete** planar reach and `h` is the effective physical half-thickness.
A 2D compound's leaves inherit that thickness from its owner snapshot. Do not
apply thickness before adding attachment reach, scale it by host Y scale, or
count absolute host Y. Query sources use their explicit thickness; queries
against stored 2D slabs validate the target slab at use time.

For a public world-vertex polygon with `n` vertices, use its exact rational
vertex mean `sum(v_i)/n` and `max_i C(v_i - mean)` for admission. Compute in wide
arithmetic without materializing the mean or subtracting rounded world centers;
equivalently retain `n*v_i - sum(v)` and the denominator `n`. Reversal and cyclic
vertex order do not change this metric. The existing Q32.32 query-runtime center
can remain unchanged. Center-and-size AABB queries use the supplied center and
`C(preparedHalfExtents)`, preserving their existing nearest-even half-size
materialization, including odd raw sizes; do not invent a new midpoint or
independently enlarge their prepared half-extents.
Finite cone queries use their apex/base midpoint and the centered finite-axis
formula. These choices define admission anchors, not
new public geometry or contact algorithms.

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
| Capsule/stadium | `radiusRaw + ceil(coreLengthRaw / 2)`, retaining an odd raw half-length. |
| Cuboid/AABB | Ceiling norm of final half-extents. |
| Cone/cylinder | `C([radius, ceil(heightRaw / 2), radius])`, with the raw half-height converted back to Q32.32; intentionally the bounding-box diagonal, not the tight support radius. |
| Mesh | Maximum ceiling norm of final scaled vertices in the existing recentered local frame. |
| Polygon | Maximum ceiling norm of final scaled local vertices about the actual collider origin. |
| Compound | Maximum of ceiling part-offset norm plus that leaf's intrinsic reach. |
| Mixed slab | `C([completePlanarReach, physicalHalfThickness])`, after all planar attachments. |

Add effective owner attachment reach once, in the body's pose-origin frame, for a
body-owned collider. A bodyless collider uses its physical origin. Use existing
ceiling-magnitude and checked-sum arithmetic; an unrepresentable envelope rejects.
These formulas and the inclusive `4,096` ceiling are locked admission policy,
not a claim of safe arbitrary higher-order scalar arithmetic. Existing committed
compound proxy helpers that subtract rounded world centers are not admission
measurements; reuse their formula and local prepared data instead.

Admission and physical proxy enclosure remain separate measurements. Exact
attachment products define the admission budget; a compound's physical proxy
must still enclose its published child centers after component rounding. For
example, a two-component one-raw attachment at scale `5/8` has exact ceiling
norm one raw unit, but its rounded center `(1 raw, 1 raw)` needs two. Preserve
a conservative physical enclosure rather than reuse the admission total or add
an arbitrary admission margin. The #126/#130 follow-up publishes a coherent
pivot proxy with the shape. For every nonzero attachment stage, its all-angle
bound adds one raw unit after the exact 3D norm; planar stages first multiply
by `1 + 2e`, where `e = CanonicalSinCosErrorBound`, then add two raw units for
the scalar-product and component rounding. Compound reduction retains each
leaf's attachment stage. Rotating intrinsic vertices use the corresponding
bound; circle/sphere radii and axis-aligned planar boxes remain unchanged.
These allowances belong only to CCD enclosure and do not alter admission.

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
  preflight before publishing the changed configuration, including every observing
  context if a mutable settings instance is shared. Resolve ownership in phase 3;
  one context callback alone is insufficient for a shared instance.
- Recorded fields are written before rebuilding. Preserve the supported runtime
  transaction boundary without claiming globally atomic arbitrary payload loads.

Public 2D AABB and world-vertex polygon queries also need admission. Measure a
query polygon about the exact vertex mean specified above, not absolute world coordinates.
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
| [#116](issue-tracker.md#grv-issue-116---cone-overlap-gjk-loses-scale-and-rotation-invariance) | Length `4,096`, end radius `2,048`: public cone overlaps produce both false positives and false negatives. | Resolved: bounded scalar crosses and exact dot signs; wide simplex predicates where needed. Closest/all queries and shared contact callers pass. |
| [#117](issue-tracker.md#grv-issue-117---detached-compound-cone-mass-weights-disagree-with-committed-geometry) | Equivalent Y-scaled cones have different detached weights despite identical prepared geometry; the parity audit found other primitive cache defaults. | Resolved: detached and prepared mass properties share effective dimensions across all five primitive families. |
| [#118](issue-tracker.md#grv-issue-118---sphere-inertia-rounds-small-mass-before-radius-squared) | Radius `1,024`, three-raw-unit mass gives sphere inertia `1048576` instead of `1258291`; other curved moments also round too soon. | Resolved: exact coefficient/geometry/mass normalization, exact internal capsule weights and corrected 2D cap polar moment. |
| [#119](issue-tracker.md#grv-issue-119---compound-inertia-quantizes-child-masses-before-applying-geometry) | Radius `512`/`1,024` spheres at `0`/`2,048`, total mass three raw units: child masses round to `0`/`3`, biasing the compound tensor. | Resolved: homogeneous child moments, exact reference reduction and one final parent-mass normalization, with warm geometry caching. |
| [#120](issue-tracker.md#grv-issue-120---3d-inertia-tensor-transport-reverses-the-geometry-rotation) | Anisotropic tensors rotate opposite to geometry; principal-axis impulses produce perpendicular spin. | Resolved: consistent active transport in compound, body and sampled CCD owners. |
| [#121](issue-tracker.md#grv-issue-121---box-inertia-rounds-separate-axis-terms-before-their-sum) | A unit cube/square at mass raw `6` gives zero rather than raw `1`. | Resolved: normalize each complete box diagonal/moment once; warmed primitive costs improve. |
| [#122](issue-tracker.md#grv-issue-122---standalone-mesh-inertia-amplifies-rounded-unit-coefficients) | Microcube unit-tensor projection error grows with ordinary representable mass. | Resolved: retained runtime integrals, certified final rounding and exact refinement preserve public projections and warm costs. |
| [#123](issue-tracker.md#grv-issue-123---detached-compound-meshes-omit-authored-part-scale-in-mass-properties) | A scale-two mesh part has fourfold different detached/committed inertia. | Resolved with #122: effective detached mesh geometry and current/prepared publication parity. |
| [#124](issue-tracker.md#grv-issue-124---standalone-polygon-reference-shifts-omit-the-centroid-residual) | Standalone triangle reference shifts omit its rounded centroid's first moment. | Resolved: retained moments and exact reference reduction, with shared first-moment arithmetic and preserved warm costs. |

These are prerequisites, not optimizations and not bugs introduced by admission.
Do not reduce the candidate ceiling to hide failures at ordinary useful sizes.
Resolve coherent numerical domains together where ownership permits, retaining
each regression. Recheck representative mass/shape ratios independently of size;
even an admitted sphere can have unrepresentable inertia at excessive mass.

The oblique host-scale probe also exposed `FMS-Issue-029`: uniform scale `2,048`
amplifies quaternion reconstruction rounding beyond an absolute matrix tolerance
although the source matrix is representable and its normalized basis passes.
The coordinated FixedMathSharp fix retains normalized orthogonality/quaternion
checks and requires representable scaled reconstruction. Its per-row coefficient
error bound is `abs(scale) * (Epsilon + q/2) + q/2`, where `q` is one raw unit.
Gravitas's existing scale policy consumes that shared fix. Compensated-scale
regressions now admit unit-radius 2D/3D geometry under direct and parented rotated
scale-2,048 transforms, covering full and scale-only capture. This resolves the
upstream decomposition prerequisite without weakening physical ancestry,
planar-coupling or shear rejection, or locking this plan's candidate ceiling.
See FixedMathSharp's completed `2026-10-14-scaled-rotation-decomposition.md`
record for the contract proof and coordinated Release/ReleaseLean validation.

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
- [x] Lock the ceiling, conservative shape formulas, inclusive rounding rule,
      attachment frame and mixed-slab formula in this contract after focused
      geometry and ponytail review. If evidence requires changing the approved
      direction or a materially different workload range, bring that decision
      back to the user before enforcement.

**Exit:** A reviewed numerical range and measurement contract with baseline
evidence. The inclusive `4,096` ceiling and exact measurement rules are locked;
enforcement and its no-regression gates remain in phases 2-4. No supported-safety
claim is based solely on proximity to Q32.32 square limits.

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
recorded separately. #116, the candidate ceiling and #096 remained open at this checkpoint; resolve
#117/#118 together before admission implementation.

**GJK refinement (2026-10-13):** #116 is resolved. Its original failures begin at iteration
zero: the second cross in `(ab x ao) x ab` saturates, distorting the search
direction before any budget exhaustion. Original public cone/cuboid and
cone/closed-convex-mesh contacts also fail the separated controls. The analytic
cone/sphere contact dispatcher remains correct; its shared internal GJK predicate
does not. Pure 2D and mixed contacts use separate algorithms.

The existing simplex policy now uses exact side/orientation dot signs and retains
scalar cross directions when every vertex component is at most `256` in
magnitude. This bounds cubic components by `536,870,912`. Larger simplices use
exact projection-region signs and face numerators, normalizing only final
directions. Support separation has its own point/direction bounds:
`3 * 64 * 2^23 < 2^31`; candidates outside those bounds compare the exact dot
against the existing strict epsilon. On the wide path, edge/face membership never
narrows the numerator. Exact face normalization follows edge rejection, avoiding
unnecessary work. The stable collinear fallback and conservative budget result are retained.

The exact edge test directly evaluates
`dot(second,second)*dot(first,ao) - dot(first,second)*dot(second,ao)` using existing
Signed192 dots and Signed320 products. Its full-range raw magnitude is below
`18*2^252`, so no intermediate narrowing is needed. Line directions reuse the
existing public normalized plane projection; face directions reuse existing
exact cross/normalization helpers. No upstream edits, internal ABI changes,
arithmetic owners, types, source files or public APIs are added.

The focused suite passes **116** tests: closest/all queries at scales
`1/256/1,024`, upright/oblique contacts across all six supported convex families
in both pair orders, complete-target separation beyond the reported exit,
cap tangency/thin axial gaps, scalar/exact cutover, scaled simplex regions,
collinear fallback and exact support-dot cancellation. Original public query
regressions fail before the fix. Independent source/test ponytail review has no
blocking findings.

Release/Lean solution builds pass with zero warnings/errors using
`UseLocalLsfStack=true`. Full Gravitas suites pass **5,873 / 5,808**;
FixedMathSharp passes **5,726 / 5,705**, plus **49** Chronicler tests per lane.
Raw reachable line/branch/method coverage remains **100%** in Gravitas,
FixedMathSharp and its validated Chronicler/FluentAssertions adapters. Gravitas
counters are `47,247 / 15,518 / 4,934` and `47,245 / 15,518 / 4,933`;
FixedMathSharp counters are `59,523 / 16,902 / 4,472` and
`59,515 / 16,902 / 4,468`, all visited. Six shared replay fixtures match
expectations and directly agree across native Windows x64 Release/Lean; no
expectations changed. Independent numerical/ponytail review approves the selected
source and its correctness, coverage and replay evidence.

Final native Windows x64/.NET `8.0.29`, SDK `10.0.302`, BDN `0.15.8` comparisons
use two launches, six warmups/six measurements and `500 ms` iterations. The ten
correct original controls and sixteen final rows all allocate **0 B/op**; every
affected row's `99.9%` confidence intervals overlap. Public closest/all groups
measure upright `203.35 -> 203.40 us`, oblique `302.98 -> 305.67 us`; 64 cone
pairs `1.1162 -> 1.0979 ms`, 64-target overlap-all `2.0574 -> 2.0362 ms`.
The latter uses the maintained `130`-unit cone, not the short diagnostic fixture.
An order-reversed repeat with three launches/eight measurements settles the
small nominal predicate increase: original `60.26 us`, fixed `59.05 us`; the
unchanged world-step control also reverses and overlaps. No repeatable warm
regression remains; do not attribute broad timing gains to this fix.

At `S=1,024`, current upright/oblique groups cost predicate `38.40 / 56.19 us`,
public closest/all `162.38 / 265.51 us`, contact predicate/dispatcher
`106.99 / 130.55 us`. Each group evaluates three placements: three predicates,
six public queries, or three predicates plus three collision dispatches. These
are group costs, not per-query latency. Old large-scale results are wrong and
cannot establish a correctness-preserving speed baseline.

The selected production delta is **97 net lines in two existing Gravitas files**.
A shared upstream numerator extraction and layout hints were rejected after
small repeated positive arithmetic-control signals; the direct quartic identity
removes that extraction entirely. FixedMathSharp is unchanged. Its relevant
normalized projection IL/flags/locals match the original. Informational revision
metadata differs between snapshots; no projection code change is retained.
Ignored `artifacts/grv-116/` retains red regressions, rejected patches, fresh
closeout coverage/replay, source provenance and immutable native snapshots.
Final core SHA256 is `457D3EC9752F80484421C88B5C954C8E8220DE306E0EB14EC924D31AC246CAE1`.
No native Linux/ARM64 or released-package validation is claimed. At this
checkpoint #117/#118, the provisional ceiling and #096 remained open; the
mass-property pair was next before locking or enforcing admission.

**Mass-property refinement (2026-10-13):** #117/#118 are resolved together.
Detached/prepared public COM and inertia now agree across sphere, cylinder,
capsule, cone and cuboid parts, including equivalent and unequal-volume scales.
Capsule dimension derivation reuses preparation's existing scale/axis validation.
No public API, runtime cache, numeric owner or production source file is added:
the production delta is **105 net Gravitas lines in seven existing files** and
**53 net FixedMathSharp lines in two existing files**.

Curved moments keep Q32.32 coefficient quantization and exact raw geometry/mass
products until final ties-to-even materialization. In particular the sphere
coefficient stays raw `1717986918`; radius `1,024`, mass raw `1` or `3` now gives
inertia raw `419430/1258291`, while its unit-mass control remains unchanged.
Cylinder/cone intermediates no longer saturate a representable final moment.
Full capsules retain exact internal cylinder/cap weights by cancelling their
common radius factors, then normalize the complete mass-weighted moment once.
The 2D formula also corrects paired semicircle polar inertia and odd-raw-mass
loss. Radius/core `1,024`, total mass raw `3` gives 3D transverse/axial moments
`2291888/1393108` and 2D moment `2767176`. Fractional geometry, circle/sphere
limits, thin rods, final overflow, compounds, real angular response, mass setters
and sleeping populate-existing loads have independent regressions.

FixedMathSharp promotes its existing Q128 final quantizer without changing that
algorithm, and adds the consumed `Signed576/Signed320` raw-ratio overload using
existing magnitude division and half-even rounding. The weighted numerator is
below 330 bits and denominator below 200 bits for the actual shape coefficients
and divisors `1` and `12`; no new limb arithmetic or downstream numeric facade is
needed. Publish FixedMathSharp first, then validate Gravitas against the released
package. The develop CI checkout pin must be advanced to that committed upstream
revision before pushing the consuming Gravitas change; it is deliberately not
changed to an uncommitted revision here.

Four test-backed candidates were rejected: generic normalization and two
partial fast paths regressed warm curved arithmetic, and a faster prototype
still quantized a full capsule's component masses. The final exact-weight
formula removes that intermediate share work. Native original/candidate
snapshots use matched two launches, six warmups/measurements and 500 ms
iterations. All **35** rows and **36** direct warmed output/allocation guards
allocate **0 B/op**; all **27** meaningful timing comparisons improve. Seven
wrong-original rows report corrected output/current cost only; one cached getter
is below timing resolution. 3D capsule ordinary/fractional costs move
`1.669 -> 1.247 us` / `1.768 -> 1.293 us`; 2D costs move
`943 -> 473 ns` / `972 -> 477 ns`. The sphere-world gate is affected by the
per-pose inertia refresh and is not an independent host-drift control.
A fresh reversed candidate/original repeat (three launches, eight measurements,
six warmups, 500 ms) confirms no resolved warm regression: ordinary/fractional
cone mass changes average `12.176 -> 11.948 us` / `12.309 -> 12.081 us`.
Ordinary intervals overlap; fractional intervals are narrowly separated.
Unchanged public arithmetic controls shift about `+0.4%` with overlapping
99.9% intervals and also allocate zero; this is not a repeatable regression.

Both local-stack configurations build with zero warnings/errors. Full suites
pass: Gravitas **5,917 Release / 5,851 Lean**, FixedMathSharp **5,731 / 5,710**,
and its Chronicler adapter **49 / 49**. Raw OpenCover reports have zero unvisited
sequence points, branches or reachable methods in Gravitas, FixedMathSharp,
FixedMathSharp.Chronicler and FixedMathSharp.FluentAssertions in both profiles;
no coverage exclusions were added. The six native Windows x64 shared replay
fixtures retain their checked expectations and directly agree across profiles.
Independent numerical/ponytail
review has no outstanding source findings. Native Linux/ARM64 and released
package validation remain pending.

A separate public compound probe confirms pre-existing **#119** on both frozen
binaries: sphere radii `512/1,024`, offsets `0/2,048`, total mass raw `3`.
Cumulative child allocations still round to raw `0/3`. Original/corrected axial
inertia is `1048576/1258291`, versus an independent uniform-density oracle
`1153434`; both public COMs agree. Exact heterogeneous child contributions need
their own contract, including parallel-axis terms and final overflow, rather
than weighting materialized child tensors. This remains a prerequisite to
locking the supported safety claim; #096 is not closed by primitive fixes.

Ignored `artifacts/grv-117-118/` retains red regressions, rejected experiments,
raw coverage/replay, source provenance, independent probes and immutable native
snapshots. The measured final Release stack hashes are Gravitas
`61E604CE5BFB6F51F0503AC7D27A33BEE949D80F74B2636268822C2A8BF078E2` and FixedMathSharp
`45DDBF81C1C8F0CB3F9B93A80BE9E1EDF5A626CA1CE9668FCBEFF337E64E4900`.

**Compound contribution design (#119, 2026-10-13):** Replace cumulative
rounded child masses with homogeneous weighted geometric moments. Use a common
semantic weight factor of six, cancelling closed tetrahedron volume's divisor
without changing measure projections or normalized shares. A common moment
factor of sixty covers box, polygon, capsule, tetrahedron and triangle-shell
constants. Retain fixed coefficients and the published Q32.32 inertia rotation
matrix, transposed into the active-column frame consistently with geometry;
preserve semantic mass-point guard bits and the public COM reference.
Sum unmaterialized intrinsic and parallel-axis terms, then apply total mass and
round each final tensor entry once. Checked compound overflow must reject a
nonrepresentable aggregate instead of concealing an already saturated child.

Mesh public unit tensors remain rounded projections. Private first/second
integral caches supply compound contributions, retaining first moments when
translating the integration reference to a published COM or explicit reference.
Polygon contributions likewise retain their rounded centroid's first-moment
residual. Reuse existing integration loops and geometry publication. Each mesh
gains a cache reference; its wide current/prepared cache is allocated only for
compound use. Compounds retain the reduced geometry for mass and pose updates;
public mesh data and serialized layouts do not acquire wide fields.
Primitive/polygon denominators cancel against their weights;
there is no arbitrary rational type or per-child applied tensor conversion.
Cache complete aggregate geometry on effective-scale/local-offset changes;
pose and mass updates reuse it. Retain detached/prepared parity, failed
transaction isolation, equal nominal fallback for degenerate 2D shapes, and
stable source order. Independent all-entry integer oracles, representation and
part-order parity, tiny/ordinary masses, final overflow, cache invalidation,
body/load behavior, full raw coverage/replay and immutable native comparisons
must pass before closeout. The current original local-stack Release snapshot
and 44-row warm matrix establish the no-regression gate; introduced cost must
be refined within this issue.

The whole-shape review confirmed two related pre-existing defects, combined
here as **#120/#121**: row-vector matrices were used in the reverse 3D tensor
transport, and box formulas rounded axis terms separately. Correct active
transport in the compound, body and sampled CCD owners; normalize each complete
box diagonal once with the existing bounded wide-ratio division. The first box
candidate using the general ratio kernel failed its warm gate by 75-87%, so it
is rejected rather than deferred. Its bounded nine/five-word replacement still
regressed 9-22%. The refined box numerator fits three words, so FixedMathSharp
now exposes that exact raw-ratio width. Its power-of-two-word denominator
uses bounded division and exact remainder comparison; uncommon denominators
reuse the existing wider owner. The existing wider implementation remains
unchanged, preserving its capsule callers and sharing final rounding. This is
a coordinated internal ABI addition: validate both profiles and libraries, release FixedMathSharp first, then revalidate Gravitas
against that package. No dependency pin is advanced here. Seven independently
integrated triangle cases also rejected the new polygon prototype's missing residual; that introduced
defect was fixed before acceptance, without opening a follow-up issue.

The unchanged standalone mesh path amplifies public rounded unit coefficients
(**#122**), and detached mesh parts omit authored part scale before publication
(**#123**). Frozen original/corrected public probes confirm both are pre-existing.
Resolve these together in the mesh owner before completing the numerical safety
inventory; keep public unit-tensor projections and the existing warm mesh gate.
Neither should be concealed by changing the proposed geometry ceiling.
Standalone polygon reference shifts also omit the rounded centroid's first
moment (**#124**); the same frozen public probe reproduces it on both binaries,
while the corrected compound agrees with the independently integrated triangle.
Reuse its retained moments in the standalone polygon owner as a separate
follow-up, preserving the requested-reference and final-overflow contracts.

**Compound contribution closeout (#119/#120/#121, 2026-10-13):** Resolved all
three issues after independent ponytail review and test-backed refinement.
The old rounded child-mass allocation is removed; four focused internal owners
retain dimensional moments, mesh integrals and their transactional cache.
The three-raw-unit unequal sphere fixture now returns axial raw `1153434`.
Independent triangle integration rejected an introduced first-moment omission;
it was repaired in this change. No introduced defect or timing regression was
deferred. At that checkpoint, the confirmed pre-existing standalone/detached
gaps #122-#124 remained prerequisites; their subsequent closeouts follow below.

Both solutions build Release/Lean and their core target frameworks with zero
warnings/errors using `UseLocalLsfStack=true`. Gravitas passes **5,982/5,916**
tests; FixedMathSharp core passes **5,732/5,711**, plus **49** serialization tests
per profile. Raw OpenCover visited/total sequence points, branches and methods
are equal: Gravitas **47,569/15,634/4,983** (Release) and
**47,567/15,634/4,982** (Lean); FixedMathSharp **59,840/16,988/4,537** and
**59,933/16,988/4,533**. No coverage exclusions changed. Six full-lifecycle
fixtures agree across native Windows x64 Release/Lean. Restoring only the old
body transport reproduces all old expectations; corrected transport changes
three solver-cache hashes without changing commands or authoritative hashes.
Native Linux/ARM lanes and released-package validation remain unexecuted here.

The immutable original baseline and accepted comparison each execute **44**
native rows: **41** comparable timings and three tiny-mass corrected-output
cost diagnostics. All native rows and 44 warm allocation guards report **0 B**.
BenchmarkDotNet **0.15.8**, native Windows x64/.NET **8.0.29**, two launches,
six warmups and six measurements at **500 ms** per iteration are matched.
No comparable row has a resolved positive 99.9% interval; unchanged controls'
small nominal differences overlap their original intervals. All 21 compound
public/mass-change/pose cases and both complete host frames improve.
Representative means:

| Warm workload | Original | Accepted |
| --- | ---: | ---: |
| 3D unequal spheres, public inertia | 17.66 us | 1.80 us |
| 3D fractional spheres, public inertia | 19.98 us | 1.95 us |
| 3D rotated heterogeneous, public inertia | 56.75 us | 3.54 us |
| 3D rotated closed/surface mesh, public inertia | 32.30 / 32.18 us | 3.43 / 3.50 us |
| 2D circles/fractional heterogeneous, public moment | 4.18 / 10.45 us | 0.45 / 0.51 us |
| Eight-actor complete 3D host frame | 609.77 us | 446.08 us |
| Eight-actor complete 2D host frame | 134.87 us | 96.71 us |

Ignored `artifacts/grv-119/` retains red regressions, original/corrected public
probes, rejected candidates, final coverage, replay attribution and captures.
`perf/` retains frozen binaries, source/dependency provenance, 44 guards,
native JSON with uncertainty, and the comparison scripts/commands. The first
12 accepted controls use frozen F; the final 32 use G after a comment-only
correction about lazy cache allocation. FixedMathSharp is identical between
those builds; the retained IL comparison verifies identical Gravitas method
bodies. Release FixedMathSharp's new internal ratio ABI first, then update the
coordinated develop pin and validate Gravitas against released packages.

**Mesh runtime closeout (#122/#123, 2026-10-13):** Runtime mesh
inertia must reduce retained integrals, including the first-moment residual and
requested reference, before mass application. Public unit-tensor data remains
a rounded projection. Reuse the existing weighted tensor owner for exact
fallback, preserving standalone final saturation and compound checked overflow.

Detached mesh mass readers lazily prepare authored part scale through the
existing private mesh transaction. Do not reject oversized authored parts in
the compound constructor when a later host scale can compensate them. Current
support, weight and COM use current geometry; prepared fallback consumes only
prepared points and weights so a detached refresh cannot replace a host candidate. Retain
the two physical policy caches lazily, without allocation on warmed alternation.

The immutable baseline includes fixed mass, actual changing mass/reference,
body mutation, pose, compounds and full host frames. Standalone changing-input
costs were approximately 50-90 ns. A last-result cache or repeated wide
normalization alone cannot satisfy that gate. The runtime cache retains
signed intrinsic coefficients at Q96 and first moments at Q128, both
truncated toward zero. With raw reference differences of magnitude below
2^64, translated diagonal error is strictly below nine Q96 integer units;
off-diagonal error is below five. Parallel terms are exact. Applying any
scalar mass gives uncertainty below nine times 2^-33 raw units. Accept the
projected result only when its entire enclosure has the same half-even scalar
rounding; use retained exact ratios at ambiguous ties, unsupported widths or
semantic points that do not meet the raw-coordinate precondition. Reuse
FixedMathSharp rounding and wide arithmetic, promoting only policy-neutral
operations with implemented production callers. Independent integer oracles,
raw full coverage and native warm comparisons validate this reduction;
#122/#123 are resolved. At this checkpoint, polygon prerequisite #124 remained
before ceiling locking; its subsequent closeout is recorded below.

The ordinary coefficient path first projects Q96 to a 64-bit Q60 mantissa.
For coefficient magnitude below 2^100 and raw mass magnitude below 2^58,
discarded bits plus the original uint-sized error are strictly below two Q60
units. One product can therefore certify rounding when its distance from the
half boundary exceeds twice the raw mass. A zero mantissa certifies zero under
the same bounds. Ineligible or uncertain cases refine through the full Q96
certificate, then the retained exact integral; these are arithmetic fast-path
bounds, not geometry or mass admission limits. Independent integer tests pin
both boundaries and coarse-uncertain/full-certain cases.

Experiments A-E were rejected for warm increases. Disassembly isolated a
1,592-byte shared translation frame and repeated coefficient normalization;
single-launch evidence also hid substantial launch variation. The translated
frame is now separate, and the coarse certificate uses its proven range and
non-tie bounds without redundant generic range checks. Uncertain products
retain full refinement.

Accepted candidate F prepares sign and width metadata in FixedMathSharp's immutable
32-byte `EnclosedQ96Coefficient`. Six prepared coefficients replace six raw
values, adding only 48 bytes per runtime policy cache. All six matched
changing-mass cases improve with disjoint 99.9% confidence intervals. The
complete warm matrix and refreshed full-profile gates pass on the same source.

The zero-weight COM fallback no longer rechecks part support. Admitted solids
and shells retain strictly positive exact weights, even when their public
measure rounds to zero. All-zero weights therefore mean unsupported closed-mesh
parts and retain the existing equal-point average. Removing the obsolete
rounded-weight support filter also removes the proposed prepared-support API;
the independent reviewer confirmed the admission and exact-weight invariants.

Fresh validation uses `UseLocalLsfStack=true`: Gravitas passes 6,008 Release
and 5,942 Lean tests; FixedMathSharp passes 5,737 Release and 5,716 Lean core
tests plus 49 adapter tests per profile. Both libraries and the affected math
adapters retain raw 100% reachable line, branch and method coverage. Six replay
fixtures match their shared expectations and each other across Windows x64
Release/Lean. Independent integer regressions cover complete tensors, signed
final saturation, mass mutation, semantic references, detached/committed scale,
policy alternation and rejected preparation. Independent ponytail review
approved the arithmetic, transactions, cache layout and simplification.
Other platform lanes were not run locally.

The native matrix uses two launches, six warmups and six measurements at
500 ms. All 45 unique cases and 45 strict guards allocate 0 B; 41 valid timing
gates have no unresolved increase. Four corrected-output cases retain costs
without speed ratios against the incorrect original results. Five initial
compound/frame increases were investigated with contemporary original/current
pairs: the original runtime shifted too, and every fresh pair overlaps its
99.9% interval. A separate reversed pair likewise corroborates the sole nominal
standalone reference increase (85.770 ns original versus 85.897 ns current).
Initial measurements remain in the evidence rather than being replaced.

| Warm workload | Original | Current |
| --- | ---: | ---: |
| Mesh, fixed mass/reference | 45.03-48.45 ns | 5.69-6.61 ns |
| Mesh, changing mass | 47.48-53.84 ns | 21.62-39.26 ns |
| Mesh, changing reference | 83.14-88.78 ns | 81.72-86.02 ns |
| Initialized mesh, public inertia | 829-858 ns | 51-52 ns |
| Initialized mesh, body mass mutation | 7.19-8.21 us | 6.66-7.58 us |
| Eight-actor mesh compound frame, contemporary pair | 661.53-667.30 us | 659.00-659.68 us |

The existing mesh benchmark class now retains actual alternating mass and
reference cases for both policies and all three subdivision sizes. All 12
configurations pass functional in-process Dry smoke; these timings are not the
native acceptance evidence. The benchmark project builds with zero warnings/errors.

Ignored `artifacts/grv-122-123/` retains red regressions, full-profile coverage,
replay captures and benchmark smoke. `perf/final45-disposition.json` preserves
every primary comparison and contemporary corroboration, with immutable binary
snapshots and source/dependency hashes. Accepted F's 694 production inputs were
frozen and verified before the ownership cleanup below. The last original
two-row child was cleaned before direct hashing; its
unchanged host/snapshot hashes and build graph are retained, and other primary
and fresh child hashes were observed directly. FixedMathSharp's new internal
coefficient/division ABI must land first; no dependency pin is advanced here.
Revalidate against released packages before releasing Gravitas.

Review cleanup extracts `EnclosedQ96Coefficient` into its own internal
FixedMathSharp scalar file, with its full-precision refinement private to that
owner. The shared scalar rounding primitive remains in `Fixed64`; no arithmetic
is copied downstream. Independent ponytail review approves the ownership and
narrow internal access. Fresh Release/Lean validation retains the test counts,
raw 100% coverage and replay results above. Normalized comparison of 4,690 math
and 6,269 physics methods finds no IL-body changes after accounting for the
intended owner/helper-name/access changes; the coefficient retains its 32-byte
field layout. All 45 strict correctness/allocation guards still pass at 0 B.
During focused timing validation, a reversed accepted-F comparison
terminated with an access violation inside `MeshRuntimeInertia.TryMaterialize`
after six warmups and two actual iterations. The failed statistics are invalid
and retained at `perf/accepted-cleanup-reverse.log`; a preserved Windows dump
shows the Q60 fractional value used as a native write destination, while the
coefficient references and tensor output address remain valid. Source audits
find no unsafe operation or escaping reference in this path. A bounded follow-up
on the same frozen binary did not reproduce the crash. The exact cause remains
unconfirmed; #125 records the agreed low-priority, non-blocking follow-up and
conditions for reopening investigation.

The controlled mitigation changes only the certificate output to
`TryMultiplyRaw(Fixed64, out long)`, preserving its bounds, rounding and zeroed
failure result. Gravitas constructs scalar values at final matrix materialization.
This removes the scalar-struct output assignment implicated by the dump without
new arithmetic, caches, coefficient copies or global PGO changes. Independent
source review approves the candidate; fresh full Release/Lean suites, raw 100%
coverage and six replay fixtures pass. Subsequent stress and matched native
comparisons passed, but do not establish the original cause or prove a fix; the
precise failed compilation's register flow is not available in the small dump.

**Polygon prerequisite #124 closeout (2026-10-14):** Standalone polygons now
reduce retained intrinsic, parallel-axis and residual first moments before one
mass-applied rounding. Published COM and overflow policy remain unchanged;
body-local references retain semantic offset products and authored part rotation,
independent of host world yaw. The compound and standalone paths share the signed
first-moment rotation/dot helper. Cancelling compound weighting factors permits
an exact Q192 calculation with a 576-bit mass-applied numerator and 320-bit
denominator. No new fields, cache, upstream changes or dependency pins are needed.

The original unit-triangle regression fails at raw `1431655764888888888` and now
returns the independently integrated `1431655765333333333`. Eighteen added cases
cover both windings, single-axis residuals, half ties, tiny/large masses,
fractional offsets/scales, reserved parts, detached/committed parity, final
overflow and actual body mass/scale refresh. Release passes **6,026** tests and
Lean **5,960**. Raw coverage is 100%: Release **47,719** sequence points,
**15,702** branches and **5,004** methods; Lean **47,717 / 15,702 / 5,003**.
Both solution profiles build with zero warnings/errors. All six replay fixtures
match shared expectations and each other on native Windows x64; other platform
lanes remain CI validation. Independent ponytail review approves the arithmetic,
widths, ownership and scope.

The initial compound-weighted reduction cost **1,568 ns** for the corrected
triangle reference row and was rejected. The final direct reduction costs
**755–757 ns** for changing/fixed triangle reference workloads, against
**889–902 ns** for the old incorrect results; those correction costs are recorded
separately from equal-output comparisons. Published-COM workloads retain their
baseline (**501–521 ns**); equal-output square reference workloads cost
**593–603 ns**, against **829–847 ns**. All **14** matched native rows and
correctness/allocation guards retain **0 B**, with no resolved increase,
including affected compound reference changes, body mass mutation and pose refresh.
The ten maintained polygon benchmark cases pass functional in-process Dry smoke;
their smoke timings are not acceptance evidence. Ignored `artifacts/grv-124/`
retains red/full-profile coverage/replay evidence, frozen inputs, loaded child
hashes, exact intervals and `final-C-disposition.json`. #124 is resolved; phase 1
ceiling locking and the complete #096 admission contract remained pending at
that checkpoint.

**Phase 1 contract lock (2026-10-14):** Independent ponytail and geometry reviews
approve the inclusive `4,096`-unit ceiling, exact outward attachment measurement,
additive frame policy, complete-planar-then-thickness mixed formula and exact
vertex-mean query anchor. #111, #125 and the remaining benchmark signals are
independent follow-ups, not blockers for this contract decision. Numerical
prerequisites #113-#124 and FixedMathSharp's scaled-decomposition prerequisite
are resolved; no new production or upstream changes were made in this lock step.

Fresh local-stack Release/Lean solution builds pass with zero warnings/errors;
**6,030 / 5,964** tests pass with raw **100%** reachable line/branch/method coverage
in both profiles. All six shared replay fixtures match expectations and directly
agree across native Windows x64 profiles. The unchanged family probe reruns
**726 checks / zero failures** on Gravitas `6a565e3f` and FixedMathSharp `fe4e2188`.
#096's ordinary radius-1,024, one-raw-core control hits at distance zero. The
extreme capsule still initializes and misses, with reach exactly `1,073,741,824`
units, confirming the future policy rejects its scale without treating the
current miss as fixed. Both probe builds pass and their loaded stack hashes match
the frozen validated binaries. Ignored `artifacts/geometry-range/phase1/contract-lock/`
retains full coverage/replay captures, probe results, source revisions and hashes.

Phase 1 is complete. Runtime enforcement, migration and row-wise warm-regression
gates remain in phases 2-4; #096 stays open. Existing timings remain historical
evidence: capture affected controls on the current accepted stack before phase 2
source edits, then compare each row independently. Native Linux/ARM and released
package validation remain CI/release gates. This documentation-only lock introduces
no runtime admission overhead and makes no new timing claim.

## Phase 2: Enforce Complete Collider Admission

**Files:** Shared policy/settings, canonical bounds, dimensional shape
transactions, primitive/polygon/compound preparation and mesh scale owners from
the reuse map. Extend existing collider, body-pose, reconfiguration and scale tests.

**Interfaces:** One shared `MaximumGeometryReach`; internal
`ValidateReach(Fixed64 reach, string parameterName)` argument validation and
candidate-local measurements supplied by existing dimensional owners. Reuse
FixedMathSharp ceiling magnitudes and checked sums for effective offset plus
centered reach. No internal wide types enter public APIs or serialized layouts.

- [x] Freeze the current accepted source stack and capture affected ordinary warm
      controls before admission source edits. Reuse maintained contact, query,
      body-pose and mesh/compound fixtures; older pre-prerequisite binaries are
      historical evidence rather than the sole performance comparison.
- [x] Add failing production tests for the locked limit, one raw increment above
      it, and oversized dimensions that scale down to admitted geometry. Cover
      overflowed intermediate scale products with representable final results.
      Include fractional-raw attachments, odd raw core lengths/heights, deliberate
      offset-cancellation rejection and complete-planar-before-thickness boundaries.
      Include primitive, mesh and polygon authoring controls, with shrinking host
      and compound-part scales applied after construction.
- [x] Implement candidate reach and shared validation before publication across
      3D, 2D and mixed colliders, including the full compound/body-local envelope.
      Validate mesh vertices within existing preparation and cached rebuild logic;
      reject unsupported standalone mesh transforms as well as collider updates.
- [x] Separate collider-owned authored mesh construction from standalone mesh
      runtime commitment through narrow internal reuse. Do not expose a public
      validation bypass or publish unsupported runtime geometry while waiting
      for the effective collider scale. Share topology/preparation ownership;
      do not create a second mesh representation or engine.
- [x] Prove near-limit clipping cannot hide oversized geometry. Keep ordinary
      colliders near both scalar limits, authored-origin-recentered compact meshes,
      rotation invariance and compensated owner/part scale controls admitted.
- [x] Keep compound physical proxies conservative after final center rounding,
      independently of the exact attachment admission budget, with 2D/3D raw-unit
      enclosure regressions and unchanged warm pose costs.
- [x] Prove initialization and failed live pose/scale/reconfiguration candidates
      preserve IDs, committed geometry, mass, bounds, partitions, awake state and
      cache identity. A small leaf with excessive attachment/part separation must
      reject. Test synchronous public API behavior, not private helper behavior alone.
- [x] Replace deliberately unsupported giant runtime fixtures with explicit
      admission regressions. Retain or relocate useful math/large-position coverage
      through supported production behavior; remove dead branches only after a
      caller audit proves they have no admitted use.
- [x] Run full Release/Lean coverage and affected warmed allocation/performance
      comparisons. Resolve introduced failures and obtain independent ponytail
      review before completing the phase.

**Exit:** All collider families and standalone mesh transformation enforce the
reviewed budget atomically. Query/load coverage remains explicitly pending;
#096 is still open.

**Phase summary (2026-10-14):** Collider and standalone-mesh admission now enforce
one shared, read-only ceiling through existing prepared-shape transactions.
Bound setters preflight the complete owner before changing fields or waking;
rejected candidates retain committed geometry and registration. Collider-owned
mesh construction validates authoring/topology without a premature unit-scale
commit; requesting detached runtime geometry prepares it at the current scale.
Public mesh transforms cannot bypass collider ownership. FixedMathSharp adds a
focused internal exact scaled-vector ceiling norm; no physics policy moves upstream.

Exact admission and physical compound proxies remain separate. A new 2D/3D
raw-unit regression exposed admission-total reuse under-enclosing a rounded leaf
center; the corrected physical proxy again uses published center differences.
The red/green enclosure test also checks the body CCD proxy. No rounding pad or
extra cache is needed. Caller audits remove obsolete local overflow scaffolding
and duplicate COM/proxy work while retaining world-position, long-travel,
independent mass-property and real pending-state defenses.

Local-stack Release/Lean solution builds pass with zero warnings/errors; Gravitas
passes **6,093 / 6,027** tests with no skips. Raw reachable line/branch/method
coverage is **100%**: `47,648 / 15,720 / 5,029` and
`47,646 / 15,720 / 5,028`, all visited. FixedMathSharp passes
**5,864 / 5,843**, plus **49** Chronicler tests per lane; its core, Chronicler
adapter and FluentAssertions helper retain **100%** raw coverage in both lanes.
Six shared replay fixtures match expectations and directly across native Windows
x64 Release/Lean. Independent numerical and external ponytail reviews clear the
corrected source.

The final native Windows x64 comparison covers **20** maintained workloads using
frozen original/candidate binaries, two launches, six warmups and six measurements
at 500 ms per iteration. All report **0 B/op**, with no repeatable regression.
Selected observed means (microseconds per benchmark operation, including each
fixture's configured population) are:

| Workload | Original | Phase 2 |
| --- | ---: | ---: |
| Capsule rebuild, 64 colliders | 1,341.855 | 1,245.937 |
| Compound translation, 64 colliders | 57.566 | 48.258 |
| Compound rotation, 64 colliders | 60.519 | 51.954 |
| 2D full step, 64 contacts | 1,216.674 | 1,150.370 |
| 3D partitioned full step, 64 spheres | 430.430 | 376.798 |

The main cone-query row was noisy (`2,250.450` versus `2,217.434` us, overlapping
intervals). A fresh reversed-order candidate/original comparison did not repeat
the nominal increase: approximately `2.101` versus `2.144` ms. The companion
sphere sweep also passes. Treat the observed gains as host-specific measurements,
not universal speedup claims. Direct generated-child DLL hashes match the frozen
snapshots, and all 699 recorded source/project hashes remain unchanged. No
additional admission cache was justified.

Pre-existing follow-ups #126-#130 are recorded separately: pending proxy epochs,
tiny-mass correction/slop, projected capsule precision, detached polygon inspection
and rotated body-attachment enclosure. Original-binary comparisons distinguish
these from the introduced compound regression, which is fixed within this phase.
Ignored `artifacts/geometry-range/phase2/` retains red/green tests, full raw coverage,
replays, source/runtime provenance, original/candidate probes and native reports.
No Linux/ARM64 or released-package validation is claimed. The develop dependency
pin is unchanged; update it after committing the coordinated FixedMathSharp
helper. Release FixedMathSharp before validating Gravitas against its package.
#096 remains open for query, settings and restore admission.

**2026-10-14 proxy follow-up (#126/#130):** Pending authored geometry now publishes
before CCD indexing and inertia integration, while pose-only publication retains
its normal lifecycle. The shape transaction publishes a separate all-angle pivot
proxy, preserves public compound radius semantics and refreshes 2D membership.
Rounded attachment/vertex enclosures cover both dimensions and compound leaves;
obsolete full-domain source-radius scans are removed. Full Release/Lean tests,
raw 100% line/branch/method coverage and all six existing replay fixtures pass.
The 14 native controls retain zero warmed allocations and no confirmed slowdown,
including a fresh reversed check of the initially noisy compound pose control.
Detailed evidence belongs to the two resolved tracker entries.

**2026-10-14 projected-sweep follow-up (#128):** Exact segment/triangle reduction
and edge-derived search normals preserve supported-scale curved entry accuracy;
the radius-1,000/4,000 capsule diagnostics now have zero raw error. Reusing the
resolved advancement direction and skipping non-facing triangle edges retains
the warm baseline without increasing iteration budgets. Release/Lean builds,
6,261/6,195 tests, raw 100% line/branch/method coverage and six existing replay
fixtures pass. All 192 independently checked public entries meet the worker's
accuracy bound; eight native controls retain 0 B/op with no confirmed slowdown.
The clipped 64-target capsule/cylinder/cone workloads improve by approximately
17%/6%/25% on this host. A maintained direct benchmark retains ordinary and
supported-scale controls. Independent ponytail review clears the smaller runtime
implementation. Detailed evidence belongs to resolved #128. Parity checks exposed
pre-existing generic 3D cylinder accuracy defect #131, resolved in the follow-up
below. The projected-sweep fix required no upstream source or pin changes.

**2026-10-15 generic-sweep follow-up (#131):** Exact tetrahedron signs,
support-plane convergence and exact face ranking correct supported-scale early
entry and ordinary-scale normal error. Direct Voronoi weights and deferred
witness construction remove redundant work. An internal FixedMathSharp cylinder
support overload chooses an axial midpoint only for exactly perpendicular GJK
searches; existing public and contact-witness selection stays unchanged. The
radius-2,500 cylinder error falls from about 0.021787 world units to 177 raw units.
Twelve native controls retain 0 B/op with no confirmed slowdown, including default
upstream support calls; direct cylinder and complete CCD costs fall by about
65% and 46%. Independent ponytail review approves the coordinated implementation.
Complete Release/Lean builds and tests pass in both repositories at raw 100%
line/branch/method coverage. All six existing replay fixtures match shared and
cross-profile expectations. Detailed evidence belongs to resolved #131; the #129
follow-up below clears the remaining focused prerequisite. Dependency pins remain
unchanged; release the math change before package-based validation.

**2026-10-15 detached-inspection follow-up (#129):** Detached polygon compound
radius inspection now reuses the existing effective authored-vertex reach
calculation instead of reading an unprepared cache. Fused scaling, outward
rounding and overflow sentinels preserve inspection before compensating host
scales without publishing or admitting geometry. The committed getter and runtime
preparation are unchanged; the 3D mesh counterpart already satisfies this
contract. Eight regression cases, 134 focused tests, full Release/Lean builds
and 6,301/6,235 tests pass at raw 100% line/branch/method coverage. All six existing
replay fixtures match unchanged expectations. Three matched native controls retain
0 B/op with no statistically resolved slowdown, bound to the exact tested stack.
Independent ponytail review clears the focused implementation; detailed evidence
belongs to resolved #129. No upstream source or dependency pin changes are needed.
The focused correctness prerequisites are resolved; resume phase 3 next.

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
      Cover explicit mixed queries in non-`Mixed` runtime modes and world-polygon
      vertex-mean admission under translation, winding and cyclic-order changes.
- [ ] Prove long rays and long travel of small swept sources remain admitted under
      existing endpoint/distance rules, including ordinary shapes near both world
      limits. Cover 2D, 3D and mixed counterparts and finite cone length/radius.
- [ ] Audit settings changes and every recorded/load path that can change effective
      geometry. Unsupported candidates reject through the existing ownership
      transaction; verify no invalid geometry reaches partitions or solver caches.
      Close shared mutable settings as a bypass through explicit ownership or
      complete observer preflight; retain pure-2D independence from unused thickness.
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
