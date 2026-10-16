# Body Dissipation Parity And Configuration Plan

**Date:** 2026-10-16  
**Status:** Proposed; planning only, implementation awaits review.  
**Owner:** Gravitas dimensional body motion and configuration.  
**Baseline:** Committed #137/#138 behavior; preserve the ordinary warm baseline.

## Goal And Scope

Give 3D and pure/mixed 2D bodies consistent, independently configurable passive
dissipation without replacing the existing drag law. Expose angular drag, define
coefficient validation/defaults, and add complete planar integration, recording,
replay and lifecycle support. This document contains the design and phased plan;
record concise results at each phase's end rather than create separate reports.

The user requested this focused plan before implementation. It does not start
#133 or change its issue-queue position. Review this plan before selecting its
implementation slot; do not bundle mesh-seam work into dissipation.

## Retained Physics Contract

Current 3D drag is empirical velocity decay, approximately `dv/dt = -k*v`, where
`k = coefficient * AirDensity * referenceArea`. Angular drag applies equivalent
decay to angular velocity. #138 bounds the discrete decay at rest and evaluates
drag after queued driving input. Preserve that law, rounding and 3D defaults;
do not reinterpret existing coefficients as dimensionless aerodynamic `Cd`.

Mass-independent decay is an intentional tuning capability for this scope.
For identical geometry, coefficients and initial velocity, masses 1 and 4 lose
the same velocity fraction through drag. Mass still affects acceleration from
the same applied force and the response to impulses. That does not make drag
itself mass-dependent. Physical fluid resistance would instead apply a force
and derive acceleration through inverse mass. Adding inverse mass would be
cheap; it would change semantics and tuning rather than provide a performance
optimization or a universal accuracy improvement.

Angular damping remains distinct: opposing torque proportional to angular
velocity, integrated through inertia and bounded at the kinetic-energy minimum.
Retain #138's 3D tensor-aware response and exact constrained intermediates.
The scalar 2D counterpart needs its own moment of inertia, not a 3D tensor.

Quadratic/Stokes drag, fluid viscosity/flow fields, physical aerodynamic torque,
new drag-model selectors and a mass-dependent replacement are outside this
plan. They require an explicit workload and separate physical units/contracts.
The existing empirical model remains useful without claiming fluid accuracy.

## Proposed Public Configuration

Both body types expose these `Fixed64` properties:

| Property | 3D default | 2D default | Meaning |
| --- | --- | --- | --- |
| `LinearDragCoefficient` | Existing `0.75` | `0` | Empirical linear velocity decay, weighted by density and geometry. |
| `AngularDragCoefficient` | Existing `0.75` | `0` | Empirical angular velocity decay, weighted by density and geometry. |
| `AngularDampingScale` | `1` | `0` | Multiplies `Environment.DampingFactor` for this body's damping torque. |

- Zero disables the corresponding contribution. A damping scale of one uses
  the environment coefficient unchanged. Retain its existing default `0.95`.
  Air density zero disables the two drag terms independently of torque damping.
- All five inputs (three body properties, `PhysicsEnvironment.AirDensity` and
  `DampingFactor`) accept nonnegative `Fixed64` values; reject negatives before
  assignment with `ArgumentOutOfRangeException`. Do not clamp invalid settings.
  There is no arbitrary upper coefficient cap; strong finite settings must
  retain bounded dissipation and explicit numeric-exhaustion behavior.
- Preserve the existing fixed-point representations of the 3D defaults.
  Existing 3D default motion and disabled 2D motion must remain unchanged.
- These are independent controls, not interchangeable units. The two empirical
  drag coefficients produce a decay rate after density/area weighting; damping
  scale is dimensionless and scales an inertia-dependent torque coefficient.
- Replace the existing public linear field with a validated property and expose
  the private angular field through the same contract. Document field-to-property
  binary/`ref` access compatibility and rejection of negative legacy settings.
  Do not add a compatibility field that bypasses validation, a settings object,
  nullable override, enable flag or additional damping mode.

### Mutation And Sleep

Hosts apply configuration changes at deterministic command boundaries before
stepping. Equal assignments are no-ops; changing passive coefficients does not
wake a resting body, change its velocities or rebuild geometry/partitions.
Both sleep owners clear motion, so dissipation cannot start movement.

Read new values at the next unprepared motion frame. Never invalidate an already
prepared CCD token and integrate driving input or decay a second time. A setter
must not retroactively rewrite a committed trajectory. Reuse existing lifecycle
ownership; no wake-all registry, environment ownership redesign or cache framework
is needed for these coefficients. Test next-frame use and same-token reuse.

## Planar Geometry And Stepping

Keep geometry estimates internal to the dimensional collider owner:

- Internal `LSCollider2D.GetLinearDragArea(Vector2d direction)`: projected width normal
  to normalized planar travel, multiplied by a fixed reference depth of one
  world unit. Reuse support/projection math in a relative or exact frame; do not
  subtract separately saturated absolute-world projections.
- Internal `LSCollider2D.GetAngularDragArea()`: occupied planar shape area normal to
  the yaw axis. Reuse `CalculateAreaForMassProperties()` and
  `ExactMassWeight` where appropriate, retaining existing geometry caches. Do
  not recompute inertia or integrate polygon area each step. Materialize a scalar
  measure only when representable; helper return types remain internal details.
- Compound estimates sum their parts, including overlaps, matching the existing
  empirical 3D compound convention. They are not projected geometric unions or
  the width of a convex hull spanning disconnected parts. No union solver is
  required. Retain an oversized area/aggregate through the bounded-fraction
  calculation using existing exact measures/arithmetic: it can yield a valid
  fraction with small coefficients, or complete decay. Do not throw solely
  because an intermediate area exceeds `Fixed64`, silently saturate it, or
  tighten geometry admission. Preserve explicit completed-result exhaustion.
- Circle, AABB, polygon, stadium/capsule and compound colliders all participate.
  Honor committed effective geometry, rotations, offsets and supported scaling.
  Reference depth never comes from `Mixed2DHalfThickness`, host Y or a grid cell.

The fixed depth is an empirical planar convention, not an assertion that a
2D body is physically a unit-thick 3D object. Document its units and avoid
promising equivalent drag for arbitrary 2D/3D shape representations.

For enabled 2D dissipation: integrate queued drive, apply bounded linear drag,
then the existing gravity/support contribution and speed-limit handling.
Integrate queued yaw torque, bounded angular drag, then bounded scalar damping
using `DampingFactor * AngularDampingScale` and inverse moment of inertia.
Frozen axes remain frozen; fully frozen/static/kinematic bodies retain their
existing ownership and do not receive dynamic decay.

With linear drag disabled, retain the exact existing sequence: combine queued
acceleration and scaled gravity, apply `RemoveIntoGroundComponent`, multiply by
the step, apply `ProjectLinearMotion`, add to velocity, then apply
`RemoveIntoGroundComponent` to velocity before refreshing speed. Splitting that
sum can change rounding even with all coefficients zero. Retain this route
without a second integrator or general mode framework, including when only yaw
dissipation is enabled.

Prepare velocity once before publishing the CCD trajectory. Pure and mixed 2D
use the same planar owner; 3D keeps its current gravity order. Contact friction,
manual/query grounding policy, gyroscopic integration and solver laws are not
part of this change.

## Implementation Map

Reuse these owners; new files are justified by responsibility or an existing
file approaching the repository's 1,000-line warning threshold, not by phase:

- `src/Gravitas/Core/{3D,2D}/SolidBody*.cs`, `.Motion.cs` and
  `.ContinuousCollision.cs`: properties, dimensional stepping and CCD ownership.
- Corresponding `.Serialization.cs` and `.ReplayHash.cs`: validated backing
  state, explicit defaults and authoritative settings.
- `src/Gravitas/Settings/PhysicsEnvironment.cs`: validate the two coefficients;
  preserve all unrelated environment behavior.
- `src/Gravitas/Colliders/2D/LSCollider2D.Geometry.cs`, the dimensional primitive
  and compound owners, and existing mass-area support: geometry estimates.
- Existing body integration/dissipation, 2D/Mixed, serialization, settings,
  determinism and allocation tests. Extend them with tests of production behavior;
  do not create parser/capture/helper test suites.
- `tests/Gravitas.Benchmarks/Core/BodyDissipationBenchmarks.cs`, existing ground
  friction/full-step/response/query controls and focused planar counterparts.
- Evergreen `RUNTIME_ARCHITECTURE.md`, `DIMENSIONS.md`, `HOST_INTEGRATION.md`
  and `SERIALIZATION.md` under `docs/wiki`, plus `docs/MIGRATION.md` for the API
  and saved-state compatibility changes. Update only pages whose contract changes.

## Phase 1: Coefficient Contract And 3D Configurability

- [ ] Freeze committed #138 source/runtime inputs and ordinary warm controls
      before production edits. Retained artifacts are reusable only with verified
      workload and runtime binding; avoid uncontrolled before/after comparisons.
- [ ] Add failing tests for public angular configuration, both drag zero values,
      damping scale zero/one, and negative-value rejection without mutation.
      Pin unchanged 3D defaults and same-geometry masses 1/4 with equal initial
      velocity: drag fractions match; force/impulse and torque damping still
      respond to mass/inertia. Cover equal assignments and sleeping bodies.
- [ ] Implement validated 3D properties, `AngularDampingScale` and environment
      validation in their current owners. Apply scale to the existing bounded
      torque path; reuse the shared #138 arithmetic instead of another solver.
- [ ] Preserve 3D record keys `LinearDragCoefficient`/`AngularDragCoefficient`.
      Missing new scale loads as one. Validate coefficient locals before their
      publication and load backing state without waking saved sleepers. Do not
      introduce a general transaction layer or promise whole-record rollback.
- [ ] Hash the new scale and version the affected section (`body.3d` currently
      version 7). Test changed settings change authoritative hashes, round trips,
      missing keys, rejected negative records and restored continuation.
- [ ] Run focused red/green tests, full correctness/coverage and matched 3D warm
      gates before accepting the phase. Independent ponytail review must include
      callers using public fields or `ref` recording and retirement opportunities.

Exit: complete usable 3D configuration, preserved default trajectories and warm
performance, validated persistence, and a reviewed compatibility note.

## Phase 2: Complete Pure And Mixed 2D Parity

- [ ] Add failing geometry tests for the two internal estimates: unit circle,
      unit box, rotated/scaled polygon and stadium, and overlapping/disconnected
      compounds with explicit additive expectations. Cover direction reversal,
      supported thin/large geometry and translations near coordinate limits.
      Include an admitted aggregate area exceeding `Fixed64` with small and
      large coefficients, proving representable fractional and complete decay.
- [ ] Implement the internal estimates through existing geometry/math owners.
      Prefer existing cached area and support data; no new public query API or
      upstream wide-arithmetic owner without a demonstrated production need.
- [ ] Add the same validated body properties with all three 2D defaults zero.
      Integrate all enabled terms in the existing preparation owner with the
      disabled rounding/order contract above and scalar inertia-aware damping.
- [ ] Exercise `TwoD`, `Both` and `Mixed` appropriately: rates 4/64 Hz, unit and
      one-eighth scales, both signs, input from rest/cancellation/reversal,
      changed direction, strong coefficients, frozen axes and anisotropic planar
      shapes. Require no passive reversal or energy gain before unrelated solver
      work; verify representative complete host steps and real contact behavior.
- [ ] Pin exact disabled traces with nonzero gravity and queued drive, not only
      empty free flight. Compare selected matching unit-box/3D yaw cases with
      equal reference areas/inertia using the existing dimensional sign mapping.
      Do not impose arbitrary shape-equivalence or identical dimensional hashes.
- [ ] Verify same-token CCD preparation, moving-target sampling and handoff do
      not apply decay twice; changing coefficients takes effect next preparation.
      Prove mixed thickness changes cannot alter planar drag estimates or decay.
- [ ] Record/hash all new 2D coefficients; missing keys load as zero and preserve
      saved sleep. Version `body.2d` (currently 4), test load validation and
      deterministic save/populate continuation using host-created shells.
- [ ] Measure enabled and disabled pure/mixed planar full steps and warmed
      allocations, then obtain independent ponytail review before phase handoff.

Exit: complete configurable planar behavior with unchanged disabled physics,
correct range/geometry handling, and measured enabled cost.

## Phase 3: Conformance, Performance And Product Closeout

- [ ] Run both solution configurations and raw 100% reachable line/branch/method
      coverage. Use `UseLocalLsfStack=true` throughout local builds/tests and
      inherited benchmark builds; released-package validation remains a release
      gate. Preserve source pins and validate changed upstream owners if needed.
- [ ] Compare repeated full-loop commands, observations and supported restored
      continuation across dimensional paths. New authoritative settings require
      hash-schema changes: review/version fixture expectations for that reason,
      while independently proving retained default physical trajectories and
      ordered observations. Never blindly regenerate fixtures to hide drift.
- [ ] Retain captures for native Windows/Linux x64/ARM64 Release/Lean comparison.
      Report local omitted lanes explicitly; full matrix success requires CI or
      actual native execution, not emulation or source inspection.
- [ ] Reuse #138's ordinary motion and unaffected response/query controls, adding
      disabled/enabled 2D workloads at representative counts. New enabled 2D work
      has an explicit measured cost; comparing it with the old absent feature
      does not demonstrate a free performance improvement. Keep correctness
      failure fixtures separate from ordinary speed gates.
- [ ] Require zero added warmed allocations and no resolved regression in
      retained 3D/default-disabled 2D workloads beyond measured uncertainty.
      Preserve raw observations and independent launch evidence. Refine introduced
      costs in this work; do not defer them as a new benchmark signal. Bring a
      material unavoidable tradeoff back before phase acceptance.
- [ ] Publish coefficient units/defaults, empirical model limits, planar reference
      depth/area, independent zero switches, mutation timing and sleep semantics
      in evergreen guides. Document field/property, negative-input, missing-record
      defaults and hash-version compatibility in migration guidance.
- [ ] Obtain final independent review of the complete change, including new files
      and any siblings. Leave changes unstaged/uncommitted with recommended commit
      messages. Capture confirmed out-of-scope defects in the owning tracker;
      newly introduced defects remain this plan's responsibility.
- [ ] Append concise phase evidence here. When all gates pass, mark complete,
      archive this plan in `done/` and update incoming/relative links.

No runtime implementation, tests or benchmark changes are part of this planning
revision. Approval of the plan does not authorize switching to physical fluid
drag, a quadratic default or a new aerodynamic subsystem.
