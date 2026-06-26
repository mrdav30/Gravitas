# Pure 2D Grounding And Support Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add first-class pure 2D grounded-state support that matches Gravitas' 3D grounding quality bar while preserving the X/Z planar 2D coordinate contract.

**Architecture:** Treat pure 2D grounding as planar support, not world-Y height. `SolidBody2D` should expose `IsGrounded`-style state and automatic/manual/disabled ownership, derive automatic support from 2D contacts and deterministic in-plane probes, and keep host-owned height or visual Y outside pure 2D physics.

**Tech Stack:** .NET 8, xUnit v3, BenchmarkDotNet where hot-path signal is needed, FixedMathSharp `Vector2d`/`Fixed64`, SwiftCollections buffers, Gravitas 2D collision/query/response services, Chronicler explicit state recording.

---

**Date:** 2026-06-26  
**Status:** Planned  
**Owner:** Gravitas 2D runtime hardening

## Purpose

Gravitas 3D bodies expose explicit grounding through deterministic probes,
manual host-owned state, hit points, normals, and serialization. Pure 2D bodies
currently simulate first-class X/Z planar translation and yaw rotation, but they
do not expose equivalent grounded-state behavior. That is a gap for platformer,
side-scroller, tilemap, and deterministic character-controller style hosts.

Pure 2D grounding must not become hidden 3D height behavior. The 2D coordinate
contract remains:

- authoritative 2D position is `Vector2d`.
- `Vector2d.X` maps to world X.
- `Vector2d.Y` maps to world Z.
- world Y is host visual height or mixed embedding metadata, not pure 2D
  physics state.

This plan adds planar support semantics: a 2D body is grounded when it has a
valid support normal in the 2D plane, either from contact manifolds or from a
deterministic in-plane probe. The public API should still use `IsGrounded` for
parity with `SolidBody`; docs can describe the underlying model as 2D support.

## Current Baseline

- `SolidBody` owns 3D grounding in
  `src/Gravitas/Core/3D/SolidBody.Grounding.cs`.
- `GroundingMode` currently supports `Automatic` and `Manual`.
- `GroundProbeMode` currently supports 3D `Auto`, `Ray`, and `SweptSphere`
  probes.
- `SolidBody2D` has no grounding partial, no grounded state, and no ground
  probe buffer.
- `SolidBody2D.LateSimulate(...)` always integrates `Gravity` into planar
  velocity.
- Pure 2D discrete response runs after body integration in
  `GravitasPhysics2DService.LateSimulate(...)`.
- Pure 2D contacts already expose deterministic manifold normals. A
  `ManifoldContact2D.Normal` points from collider A toward collider B; the
  support normal for collider A is `-Normal`, and the support normal for
  collider B is `Normal`.
- Pure 2D query services already expose segment raycasts and swept-circle
  queries suitable for deterministic in-plane support probes.

## Non-Goals

- Do not add `HeightPos`, 3D step offset, 3D platform state, visual
  interpolation, or world-Y heightmap ownership to `SolidBody2D`.
- Do not make pure 2D bodies depend on `GravitasQuery3DService` or mixed slab
  queries for ordinary 2D grounding.
- Do not expose duplicate public names for the same concept. Prefer
  `IsGrounded` over adding a separate public `IsSupported` alias.
- Do not treat ordinary movable dynamic bodies as ground by default.
- Do not add engine-specific tilemap, character-controller, or renderer hooks to
  core Gravitas.

## Guiding Rules

- Preserve deterministic ordering for support candidate selection.
- Keep disabled/manual grounding paths allocation-free.
- Reuse caller/body-owned `SwiftList` buffers for probe hits.
- Keep probe shape choices explicit and dimension-correct:
  2D ray or 2D swept-circle, not 3D swept-sphere.
- Derive automatic support from contacts first when available; use probes to
  cover near-ground, character-controller, and post-separation cases.
- Keep host-owned deterministic tilemap or heightfield-style data possible
  through manual grounding, without introducing engine-specific adapters.
- Serialize authoritative grounding settings and current state required for
  deterministic continuation.

## Proposed API Shape

The exact names should be finalized during Workstream 1, but the intended shape
is:

- Extend or revise `GroundingMode` to be dimension-neutral:
  - `Automatic`: Gravitas owns contact/probe grounding.
  - `Manual`: the host owns grounded state through explicit setter methods.
  - `Disabled`: grounding is off and the body remains ungrounded.
- Add `GroundProbeMode2D`:
  - `Auto`: derive ray versus swept-circle from shape/probe radius.
  - `Ray`: use a segment raycast along the planar down direction.
  - `SweptCircle`: use a swept-circle probe along the planar down direction.
- Add `src/Gravitas/Core/2D/SolidBody2D.Grounding.cs` with:
  - `GroundingMode GroundingMode`
  - `GroundProbeMode2D GroundProbeMode`
  - `bool IsGrounded`
  - `bool WasGrounded`
  - `Vector2d GroundNormal`
  - `Vector2d GroundPoint`
  - `Vector2d LastGroundedPosition`
  - `Vector2d GroundUpDirection`, with a clear gravity-derived/default policy
  - `Fixed64 GroundProbeRadius`
  - `Fixed64 GroundedDistanceRay`
  - `Fixed64 GroundDownDistanceOnAir`
  - `Fixed64 GroundMinNormalDot`
  - `Action<bool>? OnGrounded`
  - `UseManualGrounding(...)`
  - `UseAutomaticGrounding(...)`
  - `DisableGrounding(...)`
  - `SetManualGrounding(...)`
  - `ClearManualGrounding()`
  - `CheckGround()`

The implementation should avoid adding both `GroundPoint` and `SupportPoint`
unless a measured API need appears. Public parity with 3D and common physics
language favors grounded naming.

## Workstream 1: API Semantics And State Model

**Problem**

2D needs grounding parity with 3D without copying 3D's world-Y height model.
The first step is to define mode, probe, direction, and state ownership cleanly.

**Tasks**

- [ ] Add tests that describe the intended state transitions before
  implementation:
  - automatic starts ungrounded when no support exists.
  - `WasGrounded` is false before the first successful automatic or manual
    grounded update.
  - `WasGrounded` is true for the authoritative step after grounded support is
    lost.
  - manual state is preserved until the host changes it.
  - disabled state clears grounding and ignores automatic probes.
  - returning to automatic can immediately refresh support.
- [ ] Decide whether `GroundingMode.Disabled` should be shared by 3D and 2D.
  Prefer a shared dimension-neutral enum if it improves API clarity without
  weakening 3D behavior.
- [ ] Add `GroundProbeMode2D` instead of overloading 3D `GroundProbeMode` with
  shape names that do not fit 2D.
- [ ] Add `SolidBody2D.Grounding.cs` as a focused partial. Keep the root
  `SolidBody2D.cs` file as state ownership and broad body configuration.
- [ ] Define planar up/down behavior explicitly:
  - if gravity-derived mode is chosen, use `-Gravity.Normalized` when gravity is
    non-zero.
  - if gravity is zero or explicit mode is selected, use the configured
    `GroundUpDirection`.
  - reject zero explicit up vectors.
- [ ] Keep public naming aligned with 3D: expose `IsGrounded`, `GroundNormal`,
  and `GroundPoint`; use "support" in docs to explain the 2D model.
- [ ] Add XML docs to all new public members explaining that the values live in
  the X/Z simulation plane.

**Done Criteria**

- `SolidBody2D` exposes a clear, dimension-correct grounding state surface.
- Disabled, manual, and automatic ownership are distinct and tested.
- No 2D API suggests that pure 2D owns world-Y height.

## Workstream 2: Contact-Derived Planar Grounding

**Problem**

Platformer-style grounded state should come from actual 2D contacts when a body
is resting on a floor, slope, or static/kinematic platform. That avoids extra
queries in the common case and keeps grounding consistent with response.

**Tasks**

- [ ] Add tests for contact-derived support:
  - dynamic circle resting on bodyless/static floor becomes grounded.
  - wall contact does not ground the body.
  - ceiling contact does not ground the body.
  - slope contact grounds only when the support normal dot with up meets the
    configured threshold.
  - ordinary movable dynamic bodies are not accepted as ground by default.
  - kinematic and immovable bodies are accepted as ground.
- [ ] Add a service-owned post-response grounding pass in
  `GravitasPhysics2DService.LateSimulate(...)` after
  `SolveDiscreteResponsePairs()` and before `UpdateSleepStatesAfterPhysicsStep()`.
- [ ] Scan current solid `CollisionPair2D` manifolds without allocations.
- [ ] For each contact, orient the support normal for each body:
  - collider A receives `-contact.Normal`.
  - collider B receives `contact.Normal`.
- [ ] Accept a candidate only when:
  - the body is active and automatic grounding is enabled.
  - the other collider is not the same collider.
  - the other body is null, immovable, or kinematic.
  - the oriented normal has `Dot(normal, GroundUpDirection) >= GroundMinNormalDot`.
  - the pair is non-trigger and the contact belongs to the current frame.
- [ ] Choose the winning support candidate deterministically:
  - highest up-dot.
  - then greatest penetration depth.
  - then lowest other collider ID.
  - then lowest contact ID.
- [ ] Store `GroundNormal`, `GroundPoint`, and `LastGroundedPosition` from the
  winning candidate.

**Done Criteria**

- Resting 2D contacts update grounded state without issuing a probe.
- Contact selection is deterministic and covered by tie-break tests.
- Dynamic body contacts do not create unstable "standing on another dynamic"
  semantics unless an explicit future policy adds it.

## Workstream 3: Deterministic 2D Ground Probes

**Problem**

Contact-derived grounding is not enough for character-style controllers. Bodies
need a small in-plane ray or swept-circle probe to detect near-ground support
before contact is rebuilt or immediately after tiny separations.

**Tasks**

- [ ] Add tests for automatic probes:
  - ray probe grounds against a bodyless/static segment-style floor.
  - swept-circle probe grounds a circle body without requiring center-line
    overlap.
  - probes ignore the body's own collider.
  - probes reject triggers when `includeTriggers` is false.
  - probes reject ordinary movable dynamic bodies.
  - all-hit probe ordering picks the closest valid support, then stable collider
    identity.
- [ ] Add a body-owned `SwiftList<Physics2DHit>` probe buffer.
- [ ] Implement `CheckGround()` and simulation-time `CheckGroundForSimulation()`
  for `SolidBody2D`.
- [ ] Use `Context.Query2D.RaycastAll(...)` for `GroundProbeMode2D.Ray`.
- [ ] Use `Context.Query2D.SweepCircleAgainstStaticAll(...)` or an equivalent
  internal static-target collector for `GroundProbeMode2D.SweptCircle`.
- [ ] Resolve `Auto` deterministically:
  - use swept-circle when `GroundProbeRadius` is positive.
  - otherwise derive a radius for circle, AABB, convex polygon, and supported
    compound shapes when the value is meaningful.
  - fall back to ray when no meaningful swept radius exists.
- [ ] Reuse the 3D frame guard idea where it helps:
  - skip repeated probes for grounded bodies that have not moved meaningfully
    and whose hit platform has not moved.
  - force probes when switching to automatic or during initialization.
- [ ] Add allocation tests proving repeated probes allocate `0` bytes after
  warmup.

**Done Criteria**

- 2D probes use the pure 2D query service only.
- Probe hits follow deterministic all-hit ordering and stable valid-target
  filtering.
- Dense support-probe scenes remain allocation-free after warmup.

## Workstream 4: Integration, Gravity, Sleep, And CCD Interactions

**Problem**

Grounding should affect body simulation in physically explainable ways without
breaking CCD, sleep, or deterministic service ordering.

**Tasks**

- [ ] Add tests for grounded gravity behavior:
  - grounded bodies do not accumulate velocity into the support normal.
  - slope tangential gravity remains possible when the configured model allows
    it.
  - leaving ground restores full gravity.
- [ ] Update `SolidBody2D.LateSimulate(...)` to remove the into-ground component
  of gravity/acceleration when grounded rather than relying on positional
  correction every frame.
- [ ] Clamp residual velocity into the support normal when support is active and
  the velocity would push the body deeper into ground.
- [ ] Ensure dynamic CCD still runs from the correct frame-start position and
  displacement after grounding modifies planar acceleration or velocity.
- [ ] Ensure kinematic active-source CCD does not let host-driven movement
  silently inherit stale grounded state.
- [ ] Update sleep behavior so grounded resting bodies can sleep, while support
  loss wakes or remains awake according to existing wake rules.
- [ ] Decide whether mixed 2D/3D contacts should contribute to pure 2D
  grounded state. Prefer no in this plan unless a specific mixed gameplay case
  demands it; mixed vertical impulse is intentionally constrained out of the 2D
  body model.

**Done Criteria**

- Grounded 2D integration is physically explainable and deterministic.
- CCD and support updates do not fight over authoritative pose.
- Sleep behavior remains stable for grounded/resting bodies.

## Workstream 5: Serialization, Diagnostics, Docs, And Release Validation

**Problem**

Grounding state is authoritative enough to affect deterministic continuation.
It must serialize explicitly, show up in docs, and expose diagnostics without
allocating when disabled.

**Tasks**

- [ ] Update `SolidBody2D.Serialization.cs` to record:
  - grounding mode.
  - probe mode.
  - configured up direction or gravity-derived mode.
  - probe radius and distances.
  - normal threshold.
  - current grounded state.
  - previous grounded state.
  - ground normal, ground point, and last grounded position.
- [ ] Add save/populate replay tests covering automatic, manual, and disabled
  grounding states.
- [ ] Add or extend diagnostics for 2D ground probes. Reuse the existing
  diagnostic sink pattern, but keep dimension and probe shape explicit.
- [ ] Prove diagnostics disabled path allocates `0` bytes after warmup.
- [ ] Update wiki docs:
  - `docs/wiki/RUNTIME_ARCHITECTURE.md`
  - `docs/wiki/DIMENSIONS.md`
  - `docs/wiki/HOST_INTEGRATION.md`
  - `docs/wiki/SERIALIZATION.md`
  - `docs/wiki/DIAGNOSTICS.md`
- [ ] Update `AGENTS.md` if the 2D grounding model changes contributor
  guidance.
- [ ] Add benchmark rows only if probe/contact scan cost shows up in focused
  tests or if dense platformer scenes need a baseline.
- [ ] Run:
  - `dotnet build Gravitas.slnx --configuration Release`
  - `dotnet test Gravitas.slnx --configuration Release`
  - `dotnet build Gravitas.slnx --configuration ReleaseLean`
  - `dotnet test Gravitas.slnx --configuration ReleaseLean`

**Done Criteria**

- 2D grounding behavior is documented as planar support, not world-Y height.
- Save/load continuation preserves the support state needed for deterministic
  replay.
- Diagnostics and probes are allocation-conscious.
- Release and Lean configurations pass.

## Final Done Criteria

- `SolidBody2D` has first-class grounded-state parity with `SolidBody` where the
  concepts overlap.
- Automatic 2D grounding works from current-frame contacts and deterministic
  in-plane probes.
- Manual and disabled modes are explicit and tested.
- Grounded integration prevents into-ground gravity/velocity accumulation while
  preserving valid slope behavior.
- Pure 2D grounding remains independent from world-Y height, mixed slabs, and
  engine-specific tilemap systems.
- Docs no longer say pure 2D has no grounded state; they explain the planar
  grounding model and how hosts should use or disable it.
