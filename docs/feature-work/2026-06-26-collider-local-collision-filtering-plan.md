# Collider Local Collision Filtering Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add collider-owned ignore-layer masks for physical collider-to-collider interactions without changing public query include-mask semantics.

**Architecture:** Store an explicit `PhysicsLayerMask` on `LSCollider` and `LSCollider2D`, then route every physical pair gate through shared deterministic filter helpers. The mask is one-way by ownership but pair rejection is symmetric: if either collider ignores the other collider's layer, the physical interaction is rejected.

**Tech Stack:** .NET 8, xUnit v3, FixedMathSharp, SwiftCollections, Gravitas 2D/3D/mixed collision services, CCD helpers, grounding/support checks, Chronicler explicit recording.

---

**Date:** 2026-06-26  
**Status:** Planned  
**Owner:** Gravitas collider/filtering hardening

## Purpose

Global layer collision matrices are useful for broad project policy, but
individual colliders also need local physical filtering. Typical examples are
temporary self-buffs, owner/projectile relationships, hurtbox groups, one-way
gameplay states, or body-local sensor colliders that should ignore selected
physical layers without changing the entire context matrix.

This plan adds collider-owned ignore masks with a narrow scope:

- They affect physical interactions between two colliders.
- They affect discrete collision pairs, internal CCD target eligibility,
  grounding/support target acceptance, and mixed pair generation.
- They do not affect caller-driven public query results. Public query services
  continue to obey the caller's explicit `PhysicsLayerMask`, trigger flag, and
  excluded-collider arguments.

## Current Baseline

- `LSCollider.Layer` and `LSCollider2D.Layer` store each collider's single
  layer.
- `PhysicsSettings.CollisionMatrix` controls global 3D, 2D, and mixed physical
  layer compatibility.
- `GravitasPhysicsService.RequireCollisionPair(...)`,
  `GravitasPhysics2DService.RequireCollisionPair(...)`, and
  `GravitasMixedCollisionService.RequireCollisionPair(...)` own the main
  pair-generation gates.
- Internal CCD helpers duplicate some physical target eligibility rules.
- 3D grounding currently filters valid ground hits after `Query3D` returns
  public query hits.
- Public query services intentionally use include masks rather than physical
  pair filters.

## Non-Goals

- Do not change public query include-mask behavior.
- Do not add per-collider allow lists, categories, tags, or arbitrary predicate
  callbacks.
- Do not replace the context collision matrix.
- Do not make the ignore mask directional in final pair behavior. The property
  reads as "this collider ignores these layers", but a pair is rejected when
  either side ignores the other side.
- Do not add engine-specific owner/team/relationship systems to Gravitas core.

## Guiding Rules

- Keep the hot-path check branch-light and allocation-free.
- Centralize the rule so 3D, 2D, mixed, CCD, and grounding do not drift.
- Preserve deterministic pair ordering; filtering should remove candidates, not
  reorder surviving candidates.
- Keep serialization explicit for both 3D and 2D colliders.
- Add tests proving public query results are unchanged by collider-local ignore
  masks.

## Proposed API Shape

- Add to `LSCollider`:
  - `public PhysicsLayerMask IgnoredCollisionLayers { get; set; }`
  - `internal bool IgnoresCollisionLayer(PhysicsLayer layer)`
- Add the same API to `LSCollider2D`.
- Add internal helpers near the collision services:
  - `ColliderCollisionFilter.AllowsPhysicalPair(LSCollider first, LSCollider second)`
  - `ColliderCollisionFilter.AllowsPhysicalPair(LSCollider2D first, LSCollider2D second)`
  - `ColliderCollisionFilter.AllowsPhysicalPair(LSCollider collider3D, LSCollider2D collider2D)`

The helper should only inspect collider-local masks. Global matrix checks,
sibling/hierarchy checks, active state, trigger state, shape state, and
body-presence gates remain owned by the existing services.

## Workstream 1: API And Serialization Surface

**Problem**

The local ignore mask must be visible, explicit, saved, loaded, and available on
both dimensional collider bases without creating duplicate concepts.

**Tasks**

- [ ] Add focused tests for default values:
  - new 3D colliders default to `PhysicsLayerMask.None`.
  - new 2D colliders default to `PhysicsLayerMask.None`.
  - default masks preserve existing physical collision behavior.
- [ ] Add `IgnoredCollisionLayers` to `LSCollider`.
- [ ] Add `IgnoredCollisionLayers` to `LSCollider2D`.
- [ ] Add internal layer-check helpers on both collider bases or one shared
  internal static helper.
- [ ] Update `LSCollider.RecordData(...)` to record the mask bits.
- [ ] Update `LSCollider2D.RecordData(...)` to record the mask bits.
- [ ] Add serialization tests proving save/populate preserves the masks for 3D
  and 2D colliders.
- [ ] Add XML docs explaining that the mask affects physical
  collider-to-collider interactions only.

**Done Criteria**

- Both collider bases expose the same local physical ignore-mask surface.
- Default behavior is unchanged.
- JSON and MemoryPack-compatible recording preserve the mask.

## Workstream 2: Discrete Pair Gates Across 3D, 2D, And Mixed

**Problem**

Pair creation is the lowest-cost place to reject most physical interactions.
The rule must be applied consistently across dimensional services.

**Tasks**

- [ ] Add 3D tests:
  - collider A ignoring collider B's layer prevents pair creation.
  - collider B ignoring collider A's layer prevents pair creation.
  - no ignore mask plus enabled matrix still creates the pair.
  - trigger pairs are also filtered because the rule is physical
    collider-to-collider interaction, not only response.
- [ ] Add pure 2D tests with the same one-way and symmetric rejection cases.
- [ ] Add mixed 2D/3D tests with 3D-owned and 2D-owned ignore masks.
- [ ] Route `GravitasPhysicsService.RequireCollisionPair(...)` through the
  shared local-filter helper.
- [ ] Route `GravitasPhysics2DService.RequireCollisionPair(...)` through the
  shared local-filter helper.
- [ ] Route `GravitasMixedCollisionService.RequireCollisionPair(...)` through
  the shared local-filter helper.
- [ ] Ensure resting-pair preservation and cleanup re-check the filter so
  changing a mask at runtime removes stale pairs deterministically.

**Done Criteria**

- Discrete 3D, 2D, and mixed pair generation honor local ignore masks.
- Runtime mask changes cannot leave stale solid or trigger pairs alive.
- Filtering removes candidates without changing surviving pair order.

## Workstream 3: CCD And Grounding/Support Eligibility

**Problem**

Internal physical sweeps should not clamp, hand off velocity, or report ground
through a collider that the moving body ignores. Public queries remain caller
intent and should not automatically apply these masks.

**Tasks**

- [ ] Add 3D CCD tests proving an ignored-layer target does not clamp a moving
  body and does not receive dynamic handoff.
- [ ] Add 2D CCD tests with the same behavior for pure 2D movement.
- [ ] Add mixed CCD tests for both source dimensions.
- [ ] Add 3D grounding tests proving ignored-layer ground candidates are
  rejected after query collection.
- [ ] Add 2D grounding/support tests to the 2D grounding plan or this plan,
  depending on execution order, proving ignored-layer support candidates are
  rejected.
- [ ] Route 3D CCD target eligibility helpers through the local filter.
- [ ] Route pure 2D CCD target eligibility helpers through the local filter.
- [ ] Route mixed CCD target eligibility helpers through the local filter.
- [ ] Route `SolidBody.IsValidGroundHit(...)` through the local filter.
- [ ] Leave public `Query3D`, `Query2D`, and `QueryMixed` behavior unchanged.

**Done Criteria**

- Internal physical sweeps and grounding/support checks obey local ignore masks.
- Public query include-mask behavior is unchanged and tested.
- CCD candidate ordering for surviving hits remains deterministic.

## Workstream 4: Public Query Invariance And Documentation

**Problem**

Users need a clear distinction between local physical filtering and query
filtering. Query APIs already expose caller-owned include masks and should not
hide results because a collider happens to ignore a physical layer.

**Tasks**

- [ ] Add query invariance tests:
  - `Query3D.RaycastAll` still returns a collider whose layer is ignored by
    another collider.
  - `Query2D.RaycastAll` and overlap queries still return ignored-layer
    colliders when the caller include mask selects them.
  - `QueryMixed` still returns ignored-layer colliders when the caller include
    mask selects them.
- [ ] Update `docs/wiki/QUERY_SERVICES.md` to state that query masks are caller
  include masks and do not apply collider-local physical ignore masks.
- [ ] Update `docs/wiki/COLLISION_PIPELINE.md` with the discrete/CCD filtering
  rule.
- [ ] Update `docs/wiki/HOST_INTEGRATION.md` with an example of local physical
  layer ignore usage.
- [ ] Update `docs/wiki/SERIALIZATION.md` for collider-local mask recording.

**Done Criteria**

- Public query behavior is explicitly protected by tests.
- Docs describe exactly where collider-local masks apply.
- No wiki page suggests the local mask is a query-layer filter.

## Workstream 5: Validation And Hot-Path Review

**Problem**

This change touches every collision path. It should be validated broadly and
kept cheaper than repeated ad hoc branch logic.

**Tasks**

- [ ] Run focused collision, CCD, grounding, mixed, and query test filters.
- [ ] Run full Release and ReleaseLean test passes.
- [ ] Inspect hot paths for duplicate local-mask checks that can be centralized.
- [ ] Add a benchmark row only if focused collision distribution benchmarks
  show measurable regression.
- [ ] Update `docs/feature-work/feature-work-overview.md` when the plan is
  completed and moved to `done`.

**Done Criteria**

- Release and Lean builds/tests pass.
- No public query regression exists.
- Local filtering is centralized enough for future collider families to inherit
  the behavior automatically.

## Final Done Criteria

- `LSCollider` and `LSCollider2D` support collider-local ignored physical
  layers.
- 3D, pure 2D, mixed, CCD, and grounding/support physical interactions honor the
  local masks.
- Public query services remain caller-mask driven.
- Serialization, docs, and tests cover the new filtering boundary.
