# CCD Active Swept Sources Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:subagent-driven-development (recommended) or
> superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Promote host-driven kinematic movement and rotation from passive CCD
targets to first-class active swept sources without weakening deterministic
ordering, opt-in CCD semantics, or allocation behavior.

**Architecture:** Capture kinematic frame-start pose and frame displacement in
the same context-owned CCD preparation model used by dynamic bodies, then feed
kinematic active sweeps through dimension-specific candidate collection before
any body commits its late-simulate pose.

**Tech Stack:** .NET 8, xUnit v3, BenchmarkDotNet, FixedMathSharp,
SwiftCollections, GridForge, Gravitas runtime/query/collision services.

---

**Date:** 2026-06-21 **Status:** Done 2026-06-23 **Owner:** Gravitas
runtime/collision hardening

## Purpose

The completed continuous-collision depth plan made dynamic bodies much stronger:
they now have rotational CCD, shape-exact reduction where supported, and bounded
same-frame TOI consumption. Kinematic bodies still participate as static-style
targets at their current pose. That is not enough for a first-class engine when
hosts drive fast doors, moving platforms, traps, elevators, animated hazards, or
authoritative character-controller bodies through kinematic transforms.

This plan captures the active-source half of kinematic CCD so it can be designed
deliberately instead of hidden inside body-local patches.

## Current Baseline

- Dynamic 2D and 3D bodies cache frame-start CCD displacement.
- Static-style CCD treats bodyless, immovable, and kinematic colliders as
  passive targets.
- Dynamic angular CCD samples bounded intermediate poses for dynamic bodies.
- Kinematic 2D/3D bodies read host transforms during `LateSimulate` and refresh
  collider state, but they do not emit active swept casts from their previous
  host pose to their new host pose.
- Mixed CCD can compare mixed dynamic candidates, but kinematic mixed sources
  are still passive target-only participants.

## Guiding Rules

- Kinematic source motion must come from deterministic host state observed at
  fixed-step boundaries, not render-frame interpolation.
- Frame-start pose, target pose, and displacement must be explicit and
  context-local.
- Kinematic active sweeps must preserve layer, trigger, hierarchy, mixed slab,
  and stable collider-ID ordering rules.
- Kinematic bodies still have infinite solver mass unless a later response plan
  explicitly changes that contract.
- No floating-point math, wall-clock timing, background work, or hidden global
  state.
- Runtime paths must be allocation-free after warmup.

## Workstream 1: Kinematic CCD Source State Model

**Problem**

Kinematic bodies currently read their host transform at `LateSimulate`, but the
old and new poses are not exposed as a CCD source contract.

**Tasks**

- [x] Add failing tests proving a fast kinematic 2D body can currently pass
      through a dynamic/static target without active CCD source handling.
- [x] Add equivalent 3D and mixed-dimension proof tests.
- [x] Design explicit frame-start/host-target pose state for kinematic bodies.
- [x] Preserve existing serialization rules: host transform identity remains
      host-owned and is not snapshot identity.
- [x] Document fixed-step host obligations for kinematic CCD source movement.

## Workstream 2: Pure 2D And Pure 3D Active Kinematic Sweeps

**Problem**

Pure 2D and 3D static-style collectors already know how to gather passive
targets, but active kinematic sources need deterministic source ordering and
response handoff rules.

**Tasks**

- [x] Implement pure 2D kinematic active sweeps for translational host movement.
- [x] Implement pure 3D kinematic active sweeps for translational host movement.
- [x] Add active kinematic rotation sampling using the same conservative angular
      candidate model as dynamic rotational CCD.
- [x] Ensure dynamic bodies hit by a kinematic source receive deterministic wake
      and velocity/correction behavior consistent with infinite-mass kinematic
      response.
- [x] Add allocation guardrails for repeated active kinematic CCD frames.

## Workstream 3: Mixed-Dimension Kinematic Sources

**Problem**

Mixed CCD has extra embedding rules: 2D sources are finite slabs/prisms and 3D
sources can move vertically while 2D response is plane-constrained.

**Tasks**

- [x] Add mixed tests for fast kinematic 3D sources crossing 2D dynamic slabs.
- [x] Add mixed tests for fast kinematic 2D slab sources crossing 3D dynamic
      targets.
- [x] Preserve `PhysicsRuntimeMode.Mixed` identity and keep `Both` as separate
      pure 2D/3D simulation without mixed contacts.
- [x] Keep vertical impulse/correction constrained by existing mixed response
      rules until the CCD island plan changes that model.

## Workstream 4: Benchmarks, Docs, And Release Validation

**Tasks**

- [x] Add benchmark rows for no-hit, first-hit, dense-hit, and rotational
      kinematic active CCD sources.
- [x] Update `docs/wiki/COLLISION_PIPELINE.md` and host integration docs with
      the new kinematic source contract.
- [x] Run focused CCD tests, full `Release`, full `ReleaseLean`, and relevant
      benchmark smoke rows.

## Done Criteria

- Kinematic host translation and rotation can act as deterministic active CCD
  sources in pure 2D and pure 3D.
- Mixed active-source behavior is either implemented with tests or deliberately
  split into the service-level island plan with a concrete blocker.
- Runtime active-source paths allocate `0` bytes after warmup.
- Host-facing docs explain when kinematic CCD is evaluated and what transform
  state hosts must provide.
