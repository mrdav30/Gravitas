# Mass Inertia Tooling And Diagnostics Follow-Up Plan

**Date:** 2026-06-19
**Status:** Backlog / evidence-gated
**Owner:** Gravitas runtime/collision hardening, with possible FixedMathSharp tooling support

## Purpose

The 3D/mixed mass-inertia solver hardening is complete for Gravitas runtime
alpha scope. This plan preserves the remaining non-core follow-up ideas that
should not live as dangling deferred notes inside the completed solver plan.

These items should start only when there is evidence that the current runtime
shape is insufficient: authoring friction, host diagnostics need, benchmark
data, or lower-stack tooling work that gives Gravitas a cleaner API without
runtime diagonalization.

## Current Baseline

- Gravitas now supports explicit effective inverse mass/inertia policy for 3D
  and mixed response.
- `StiffBody` owns 3D local/world center-of-mass state and full local/world
  inertia tensors.
- Mesh and compound inertia can preserve products of inertia and use full
  deterministic `Fixed3x3` inversion with a diagonal fast path.
- `PhysicsMesh` remains a geometry/topology API. Body mobility and angular
  force policy are applied by the caller before mesh inertia is requested.
- Pure 2D center-of-mass, scalar moment, and angular response work is tracked
  separately in
  [`2026-06-19-pure-2d-angular-dynamics-com-plan.md`](done/2026-06-19-pure-2d-angular-dynamics-com-plan.md).

## Workstream 1: Principal-Axis Offline Tooling Boundary

**Goal:** Decide whether lower-stack or offline tooling should produce a
principal-axis payload for authored mass properties, without adding runtime
diagonalization to Gravitas by default.

Tasks:

- [ ] Collect evidence before implementation: benchmark pressure from full
  tensor inversion, authoring workflows that prefer diagonal natural inertia,
  or FixedMathSharp demand for deterministic symmetric-matrix decomposition.
- [ ] If pursued, design the payload as explicit data: center of mass,
  principal-axis orientation, diagonal inertia tensor, and validation metadata.
- [ ] Keep the eigensolver/diagonalization algorithm out of the hot Gravitas
  solver path unless benchmarks prove runtime generation is needed.
- [ ] Require deterministic tie rules for repeated or near-repeated eigenvalues,
  sign conventions for axes, fixed-point error bounds, and stable serialization
  behavior.
- [ ] Add Gravitas importer/tests only after the lower-stack/tooling contract is
  explicit enough to reject ambiguous or inconsistent payloads.

## Workstream 2: Center-Of-Mass Diagnostic Marker

**Goal:** Add COM visualization only if host diagnostics need a dedicated marker
beyond the response/contact diagnostic payloads already emitted.

Tasks:

- [ ] Confirm a real adapter or debugging workflow needs a dedicated COM marker.
- [ ] Keep COM marker output diagnostic-only and disabled-path allocation-free.
- [ ] Emit deterministic marker data from authoritative body state without
  changing simulation behavior.
- [ ] Update diagnostic adapter docs and tests if a new debug draw kind or event
  payload is introduced.

## Workstream 3: Richer Mesh Mass-Property Payload Boundary

**Goal:** Preserve the completed mesh/body boundary if future APIs return richer
mass-property structs instead of raw tensors.

Tasks:

- [ ] Keep body mobility, kinematic state, and angular-force policy outside
  `PhysicsMesh` and mesh-authored data APIs.
- [ ] Make caller responsibility explicit in any new mass-property return type:
  geometry supplies mass properties; body/collider binding applies solver gates.
- [ ] Add tests that invalid open topology is still rejected by closed-volume
  mesh APIs unless the caller explicitly selects a surface approximation.
- [ ] Add tests that non-rotating body policies can bind legal collision
  surfaces without forcing mesh inertia calculation.

## Exit Criteria

- No runtime principal-axis diagonalization exists without benchmark evidence
  and deterministic lower-stack/tooling support.
- Any COM visualization remains diagnostic-only and cannot mutate authoritative
  physics state.
- Future mesh mass-property APIs preserve the geometry/topology versus body
  mobility boundary.
