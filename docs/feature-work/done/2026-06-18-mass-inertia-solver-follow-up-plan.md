# Mass, Inertia, And Center-Of-Mass Solver Follow-Up Plan

**Date:** 2026-06-18 **Status:** Completed 2026-06-19 **Owner:** Gravitas
runtime/collision hardening

## Purpose

Phase 4A gave dynamic mesh bodies physically meaningful closed-volume inertia by
default, while keeping surface inertia behind an explicit opt-in policy. It also
exposed `MeshMassProperties.CenterOfMass`, but several deeper solver and
body-model boundaries were intentionally kept out of that slice.

This completed plan records the mass/inertia solver work that was split out of
the earlier alpha follow-up plan.

## Related Follow-Up Work

Pure 2D center-of-mass and angular dynamics are intentionally tracked in
[`2026-06-19-pure-2d-angular-dynamics-com-plan.md`](2026-06-19-pure-2d-angular-dynamics-com-plan.md).
That plan is deliberately separate so 2D gets a scalar COM/moment model designed
for its own X/Z planar solver instead of a weak copy of the 3D tensor path.

Lower-stack principal-axis/offline tooling, optional COM diagnostic marker
polish, and future richer mesh mass-property payload boundaries are tracked in
[`2026-06-19-mass-inertia-tooling-and-diagnostics-follow-up-plan.md`](../2026-06-19-mass-inertia-tooling-and-diagnostics-follow-up-plan.md).

## Current Baseline

- `PhysicsMesh.CalculateInertiaTensor(mass)` remains a shape/topology API. Body
  mobility policy is applied at `SolidBody.RefreshInertiaTensor()` and response
  wrappers, not inside `PhysicsMesh`.
- Runtime inertia tensors are local `Fixed3x3` tensors. Diagonal tensors use an
  explicit reciprocal fast path, while tensors with products of inertia use
  deterministic full `Fixed3x3` inversion.
- `SolidBody` keeps local inertia, local inverse inertia, world inertia, and
  world inverse inertia separate so repeated orientation refreshes cannot rotate
  an already world-space tensor.
- `SolidBody.LocalCenterOfMassOffset` is the authoritative 3D body-local COM
  offset, and `SolidBody.WorldCenterOfMass` is the response-space COM.
- Collider geometry derives the default body COM through
  `LSCollider.CalculateLocalCenterOfMassOffset()`. Primitives default to their
  scaled collider center, compounds use area-weighted part centers, and closed
  mesh colliders consume `MeshMassProperties.CenterOfMass`.
- Closed mesh inertia now integrates products of inertia from fixed-point signed
  tetrahedra and shifts between reference points with the full parallel-axis
  tensor. Compound colliders also preserve products of inertia from off-axis
  part placement.
- `SolidBody.InverseMass` is the raw reciprocal of `Mass`; immovable and
  kinematic participants now expose zero solver mass through
  `SolidBody.EffectiveInverseMass`.
- `SolidBody` owns the 3D effective response policy through `CanTranslate`,
  `CanRotate`, `EffectiveInverseMass`, and `EffectiveInverseInertiaTensor`.
  `ResponseBody` and mixed response consume that surface instead of restating 3D
  mobility rules locally.

## Workstream 1: Explicit Effective Mass API

**Goal:** Make body mobility gates harder to misuse without moving runtime
ownership into mesh/topology APIs.

Tasks:

- [x] Decide whether `SolidBody` should expose explicit effective mass helpers
      such as `CanTranslate`, `CanRotate`, `EffectiveInverseMass`, and
      `EffectiveInverseInertiaTensor`.
- [x] Preserve the current rule that immovable and kinematic bodies behave as
      infinite mass in response, even if their raw mass/inertia values remain
      available for inspection or serialization.
- [x] Update 3D and mixed response code to use the same effective-mass surface
      if the API is added.
- [x] Add tests for movable, kinematic, immovable, and angular-force-disabled
      participants.

**Progress 2026-06-19:** Workstream 1 added the explicit 3D body-side effective
mass API and moved 3D plus mixed response onto it. Focused coverage lives in
`tests/Gravitas.Tests/Core/SolidBodyEffectiveMassTests.cs`, with existing 3D and
mixed response suites validating the refactor against solver behavior.
Non-positive masses are documented as non-translatable in solver policy while
the raw `InverseMass` value remains available for inspection.

## Workstream 2: Body Center-Of-Mass Offset Model

**Goal:** Allow non-origin mass properties without corrupting contact response,
serialization, or deterministic transforms.

Tasks:

- [x] Define where COM offset lives: body, collider binding, or shape
      definition.
- [x] Apply the offset consistently to contact relative points, torque arms,
      inertia transforms, visual transforms, and debug diagnostics.
- [x] Use the parallel-axis theorem when consuming mesh mass properties whose
      COM differs from the collider reference center.
- [x] Add Chronicler populate-existing-instance coverage for COM state once it
      becomes authoritative runtime data.
- [x] Add tests for closed-volume meshes with off-center COM, including
      collision impulse angular effects and replay continuation.

**Progress 2026-06-19:** Workstream 2 moved 3D response torque arms and mixed 3D
torque arms to `SolidBody.WorldCenterOfMass`, added authoritative
`LocalCenterOfMassOffset` Chronicler state, added
`ResetCenterOfMassFromCollider` for hosts that want to return to derived
geometry COM, and renamed the public inverse inertia property to
`InverseInertiaTensor`. Focused coverage lives in
`tests/Gravitas.Tests/Core/SolidBodyCenterOfMassTests.cs`,
`tests/Gravitas.Tests/Colliders/PhysicsMeshTests.cs`, and
`tests/Gravitas.Tests/Serialization/SolidBodySerializationTests.cs`. Existing
response diagnostics observe the COM-based solver result through contact,
response impulse, and velocity-delta events. Dedicated COM marker polish is
tracked outside this completed solver plan.

## Workstream 3: Full Tensor And Principal-Axis Support

**Goal:** Move beyond diagonal-only local inertia when mesh and compound shapes
justify the extra solver complexity.

Tasks:

- [x] Evaluate deterministic fixed-point full `Fixed3x3` inversion for inertia
      tensors with products of inertia.
- [x] Decide whether principal-axis diagonalization belongs in Gravitas, in
      FixedMathSharp, or in a tooling/preprocess path.
- [x] Benchmark full tensor operations against the existing diagonal path before
      replacing any hot response code.
- [x] Keep a diagonal fast path for simple primitive and aligned compound
      shapes.
- [x] Add tests for rotated non-uniform mass distributions, singular tensors,
      mesh products of inertia, compound products of inertia, and stable world
      tensor orientation. Deterministic ordering/tie tests belong with the
      separate principal-axis tooling follow-up if that evidence-gated work
      starts.

**Progress 2026-06-19:** Workstream 3 added internal `InertiaTensorMath` with a
diagonal inversion fast path, deterministic full `Fixed3x3` inversion, singular
tensor fallback to zero solver inertia, and full parallel-axis tensor shifts.
Closed mesh mass properties now integrate products of inertia, compound
colliders preserve off-axis products, and `SolidBody` separates local and
world-space inertia state to avoid repeated orientation compounding. Runtime
principal-axis diagonalization was intentionally not adopted; possible
FixedMathSharp/offline payload work is tracked outside this completed solver
plan and remains evidence-gated. Focused coverage lives in
`tests/Gravitas.Tests/Core/InertiaTensorMathTests.cs`,
`tests/Gravitas.Tests/Colliders/PhysicsMeshTests.cs`, and
`tests/Gravitas.Tests/Colliders/ColliderRuntimeStateTests.cs`. Benchmark
coverage lives in `tests/Gravitas.Benchmarks/Core/InertiaTensorBenchmarks.cs`.

## Workstream 4: Mesh Inertia API Boundary

**Goal:** Keep mesh mass-property APIs reusable without asking shape data to
infer body mobility.

Tasks:

- [x] Keep `PhysicsMesh` responsible for geometry-derived mass properties only.
- [x] Keep movable/kinematic/immovable/angular-force policy at the body or
      collider-binding boundary.
- [x] Document that richer mass-property payload APIs must make the caller's
      responsibility for applying mobility gates explicit if they are
      introduced.

**Progress 2026-06-19:** Workstream 4 confirmed the existing runtime boundary:
`PhysicsMesh` remains a geometry/topology API and never receives body mobility
state, while `SolidBody.CanUseAngularInertia` gates immovable, kinematic, and
angular-force-disabled bodies before collider inertia is requested. Boundary
coverage now includes default closed-volume rejection for open mesh topology,
explicit surface-approximation opt-in, and non-rotating body policies that can
legally bind open mesh collision surfaces without consuming mesh inertia. Any
future richer mesh mass-property payload work is tracked outside this completed
solver plan.

## Exit Criteria

- Response code has one clear effective-mass policy for 3D and mixed contacts.
- Bodies can represent COM offsets explicitly, deterministically, and with
  serialization coverage.
- Full tensor support is tested, benchmarked, and keeps a simple diagonal fast
  path.
- `PhysicsMesh` remains a shape/topology source of mass properties, not a body
  mobility policy engine.
