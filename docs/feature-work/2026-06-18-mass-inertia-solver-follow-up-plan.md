# Mass, Inertia, And Center-Of-Mass Solver Follow-Up Plan

**Date:** 2026-06-18
**Status:** Backlog / future hardening plan
**Owner:** Gravitas runtime/collision hardening

## Purpose

Phase 4A gave dynamic mesh bodies physically meaningful closed-volume inertia
by default, while keeping surface inertia behind an explicit opt-in policy. It
also exposed `MeshMassProperties.CenterOfMass`, but several deeper solver and
body-model boundaries were intentionally kept out of that slice.

This plan captures those deferred mass/inertia items so the completed alpha
follow-up plan can move to `docs/feature-work/done` without hiding unfinished
solver architecture work.

## Current Baseline

- `PhysicsMesh.CalculateInertiaTensor(mass)` remains a shape/topology API. Body
  mobility policy is applied at `StiffBody.RefreshInertiaTensor()` and response
  wrappers, not inside `PhysicsMesh`.
- Runtime inertia tensors are diagonal local tensors inverted with
  `InvertDiagonal()`.
- `MeshMassProperties.CenterOfMass` is available, but `StiffBody` does not yet
  have an explicit center-of-mass offset model.
- Mesh inertia currently uses the collider reference center because contact
  relative points, transforms, serialization, diagnostics, and the parallel-axis
  theorem need one coherent body COM contract before arbitrary mesh COM can be
  consumed safely.
- `StiffBody.InverseMass` is the raw reciprocal of `Mass`; immovable and
  kinematic participants are mapped to zero effective inverse mass by response
  helpers such as `ResponseBody` and mixed response code.

## Workstream 1: Explicit Effective Mass API

**Goal:** Make body mobility gates harder to misuse without moving runtime
ownership into mesh/topology APIs.

Tasks:

- Decide whether `StiffBody` should expose explicit effective mass helpers such
  as `CanTranslate`, `CanRotate`, `EffectiveInverseMass`, and
  `EffectiveInverseInertiaTensor`.
- Preserve the current rule that immovable and kinematic bodies behave as
  infinite mass in response, even if their raw mass/inertia values remain
  available for inspection or serialization.
- Update 3D and mixed response code to use the same effective-mass surface if
  the API is added.
- Add tests for movable, kinematic, immovable, and angular-force-disabled
  participants.

## Workstream 2: Body Center-Of-Mass Offset Model

**Goal:** Allow non-origin mass properties without corrupting contact response,
serialization, or deterministic transforms.

Tasks:

- Define where COM offset lives: body, collider binding, or shape definition.
- Apply the offset consistently to contact relative points, torque arms,
  inertia transforms, visual transforms, and debug diagnostics.
- Use the parallel-axis theorem when consuming mesh mass properties whose COM
  differs from the collider reference center.
- Add Chronicler populate-existing-instance coverage for COM state once it
  becomes authoritative runtime data.
- Add tests for closed-volume meshes with off-center COM, including collision
  impulse angular effects and replay continuation.

## Workstream 3: Full Tensor And Principal-Axis Support

**Goal:** Move beyond diagonal-only local inertia when mesh and compound shapes
justify the extra solver complexity.

Tasks:

- Evaluate deterministic fixed-point full `Fixed3x3` inversion for inertia
  tensors with products of inertia.
- Decide whether principal-axis diagonalization belongs in Gravitas, in
  FixedMathSharp, or in a tooling/preprocess path.
- Benchmark full tensor operations against the existing diagonal path before
  replacing any hot response code.
- Keep a diagonal fast path for simple primitive and aligned compound shapes.
- Add tests for rotated non-uniform mass distributions, singular tensors, and
  deterministic ordering/tie cases in any diagonalization algorithm.

## Workstream 4: Mesh Inertia API Boundary

**Goal:** Keep mesh mass-property APIs reusable without asking shape data to
infer body mobility.

Tasks:

- Keep `PhysicsMesh` responsible for geometry-derived mass properties only.
- Keep movable/kinematic/immovable/angular-force policy at the body or
  collider-binding boundary.
- If future APIs return richer mass-property structs, make the caller's
  responsibility for applying mobility gates explicit.

## Exit Criteria

- Response code has one clear effective-mass policy for 3D and mixed contacts.
- Bodies can represent COM offsets explicitly, deterministically, and with
  serialization coverage.
- Full tensor support, if adopted, is tested, benchmarked, and keeps a simple
  diagonal fast path.
- `PhysicsMesh` remains a shape/topology source of mass properties, not a body
  mobility policy engine.
