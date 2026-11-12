---
title: Gravitas
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
    <a href="guides/GETTING_STARTED.md">Get started</a>
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
    stable ordering and caller-owned buffers. Native 2D support probes apply
    physical collision policy before selecting a surface.</p>
  </div>
  <div class="grv-card">
    <h3><a href="xref:Gravitas.Diagnostics">See the physics</a></h3>
    <p>Project events, counters, contacts, query details, and debug-draw data
    into host tools without adding a renderer to the core.</p>
  </div>
</div>

## Package family

| Package         | Serialization profile                                                  |
| --------------- | ---------------------------------------------------------------------- |
| `Gravitas`      | Standard package with built-in MemoryPack support                      |
| `Gravitas.Lean` | Same core physics API without the direct MemoryPack runtime dependency |

Both variants target `netstandard2.1` and `net10.0`.

<a id="part-of-the-lsf-stack"></a>

## Part of the LSF ecosystem

Gravitas builds physics policy on focused lower layers:

- [FixedMathSharp](https://github.com/mrdav30/FixedMathSharp) for deterministic
  fixed-point math, transforms, and full-domain geometry.
- [SwiftCollections](https://github.com/mrdav30/SwiftCollections) for
  low-allocation collections, pools, and spatial structures.
- [GridForge](https://github.com/mrdav30/GridForge) for explicit voxel worlds,
  traversal, and physics-partition backing.
- [Chronicler](https://github.com/mrdav30/Chronicler) for deterministic state
  transfer, wide simulation timing, and replay infrastructure.

[Trailblazer](https://github.com/mrdav30/Trailblazer) provides character
controller and navigation systems in the same ecosystem.

## Guides

- [Getting Started](guides/GETTING_STARTED.md) — Set up a world, push a sphere,
  and read its position after one step.
- [Technical Overview](guides/OVERVIEW.md) — Follow that sphere across the host
  and physics boundary.

### Build the runtime

- [Runtime Architecture](guides/RUNTIME_ARCHITECTURE.md) — Trace a frame through
  the context and find which service owns each part.
- [Host Integration](guides/HOST_INTEGRATION.md) — Bind host objects, drive the
  fixed loop, and release registrations.
- [Workload And Tuning](guides/WORKLOAD_TUNING.md) — Choose a tick budget and
  measure a representative workload.
- [2D, 3D, And Runtime Modes](guides/DIMENSIONS.md) — Choose planar, 3D,
  independent, or mixed simulation.
- [Serialization And Replay](guides/SERIALIZATION.md) — Restore state into
  host-created objects and compare replay hashes.

### Understand collision

- [Collision Pipeline](guides/COLLISION_PIPELINE.md) — Follow a pair from
  candidate discovery through response and cleanup.
- [Collision Broad Phase](guides/COLLISION_BROAD_PHASE.md) — See how voxel
  membership produces candidate pairs.
- [Collider Shape Reference](guides/COLLIDER_SHAPE_REFERENCE.md) — Choose a
  shape and check supported pairings.
- [Continuous Collision Detection](guides/CONTINUOUS_COLLISION_DETECTION.md) —
  Follow a fast sphere's sweep into a thin wall.
- [Collision Response](guides/COLLISION_RESPONSE.md) — See how contacts,
  materials, and joints change motion.

### Query and observe it

- [Query Services](guides/QUERY_SERVICES.md) — Choose a raycast, overlap, sweep,
  all-hit query, or batch.
- [Query Reference](guides/QUERY_REFERENCE.md) — Check hit geometry, filtering,
  ordering, and reentrancy.
- [Diagnostics](guides/DIAGNOSTICS.md) — Read events and counters or collect
  debug-draw commands.
- [Diagnostic Adapters](guides/DIAGNOSTIC_ADAPTERS.md) — Feed diagnostics into a
  renderer, structured log, or replay timeline.

## Resources

- [Source, issues, and releases](https://github.com/mrdav30/Gravitas)
- [Core test-suite coverage](https://mrdav30.github.io/Gravitas/coverage/)
- [Contributing guide](https://github.com/mrdav30/Gravitas/blob/main/CONTRIBUTING.md)

The API reference is generated from the library's XML documentation. The guides
explain lifecycle ownership, dimensional models, collision, queries,
serialization, replay, and diagnostics in task-oriented prose.
