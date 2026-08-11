---
title: Gravitas API
description: API reference and guides for deterministic fixed-point 2D, 3D, and mixed-dimension physics.
---

<div class="grv-hero">
  <p class="grv-kicker">LOCKSTEP PHYSICS FOR .NET</p>
  <h1>Make every frame agree.</h1>
  <p>Gravitas brings fixed-point bodies, colliders, collision response,
  constraints, queries, replay hashes, and diagnostics together inside explicit
  world contexts.</p>
  <div class="grv-actions">
    <a href="xref:Gravitas">Browse the API</a>
    <a href="https://github.com/mrdav30/Gravitas/wiki/GETTING_STARTED">Get started</a>
  </div>
</div>

## Build an explicit physics world

<div class="grv-card-grid">
  <div class="grv-card">
    <h3><a href="xref:Gravitas.GravitasWorldContext">Own the runtime</a></h3>
    <p>Keep clocks, services, bodies, colliders, queries, replay state, and
    diagnostics isolated inside one context per simulation.</p>
  </div>
  <div class="grv-card">
    <h3><a href="xref:Gravitas.PhysicsRuntimeMode">Choose the dimensions</a></h3>
    <p>Run pure 2D, pure 3D, both without cross-contacts, or an explicit mixed
    2D/3D collision model.</p>
  </div>
  <div class="grv-card">
    <h3><a href="xref:Gravitas.Colliders">Author the shapes</a></h3>
    <p>Build runtime geometry from primitives, convex or concave meshes,
    compounds, materials, and engine-neutral shape definitions.</p>
  </div>
</div>

## Simulate, query, and observe

<div class="grv-card-grid">
  <div class="grv-card">
    <h3><a href="xref:Gravitas.Constraints">Connect bodies</a></h3>
    <p>Use deterministic 2D and 3D joints, motors, limits, ragdoll definitions,
    and context-owned solvers.</p>
  </div>
  <div class="grv-card">
    <h3><a href="xref:Gravitas.Queries">Ask the world</a></h3>
    <p>Raycast, overlap, sweep, batch, and reduce 2D, 3D, or mixed hits with
    stable ordering and caller-owned buffers.</p>
  </div>
  <div class="grv-card">
    <h3><a href="xref:Gravitas.Diagnostics">See the physics</a></h3>
    <p>Project events, counters, contacts, query details, and debug-draw data
    into host tools without adding a renderer to the core.</p>
  </div>
</div>

## Package family

| Package         | Serialization profile                                      |
| --------------- | ---------------------------------------------------------- |
| `Gravitas`      | Standard package with built-in MemoryPack support          |
| `Gravitas.Lean` | Same core physics API without the direct MemoryPack runtime dependency |

## Part of the LSF stack

Gravitas builds physics policy on focused lower layers:

- [FixedMathSharp](https://github.com/mrdav30/FixedMathSharp) for deterministic
  fixed-point math, transforms, and full-domain geometry.
- [SwiftCollections](https://github.com/mrdav30/SwiftCollections) for
  low-allocation collections, pools, and spatial structures.
- [GridForge](https://github.com/mrdav30/GridForge) for explicit voxel worlds,
  traversal, and physics-partition backing.
- [Chronicler](https://github.com/mrdav30/Chronicler) for deterministic state
  transfer and replay infrastructure.

## Resources

- [Behavioral guides and getting started](https://github.com/mrdav30/Gravitas/wiki)
- [Source, issues, and releases](https://github.com/mrdav30/Gravitas)
- [Core test-suite coverage](https://mrdav30.github.io/Gravitas/coverage/)

The API reference is generated from the library's XML documentation. The wiki
explains lifecycle ownership, dimensional models, collision, queries,
serialization, replay, and diagnostics in task-oriented prose.
