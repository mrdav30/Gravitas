# Gravitas Guides

Start with a sphere in one world, then use the guides below when you need to
wire a host, investigate a contact, run a query, or compare replay state.

Gravitas provides deterministic fixed-point physics for lockstep games and
simulations in 2D, 3D, and mixed 2D/3D. Your host owns commands, rendering, and
the outer loop. A `GravitasWorldContext` owns the clock, bodies, colliders,
constraints, queries, replay state, and diagnostics for each simulation.

## Start Here

[Getting Started](GETTING_STARTED.md) takes you from package installation to
your first fixed step. You will push a sphere and read its new position.
[Technical Overview](OVERVIEW.md) follows the same sphere across the host and
physics boundary and maps the services you will use as the simulation grows.

## Build The Runtime

- [Runtime Architecture](RUNTIME_ARCHITECTURE.md)
  Trace a frame through the context and find which service owns each part.
- [Host Integration](HOST_INTEGRATION.md)
  Bind host objects, drive the fixed loop, and release registrations.
- [Workload And Tuning](WORKLOAD_TUNING.md)
  Choose a tick budget and measure a representative workload.
- [2D, 3D, And Runtime Modes](DIMENSIONS.md)
  Choose planar, 3D, independent, or mixed simulation.
- [Serialization And Replay](SERIALIZATION.md)
  Restore state into host-created objects and compare replay hashes.

## Understand Collision

- [Collision Pipeline](COLLISION_PIPELINE.md)
  Follow a pair from candidate discovery through response and cleanup.
- [Collision Broad Phase](COLLISION_BROAD_PHASE.md)
  See how voxel membership produces candidate pairs.
- [Collider Shape Reference](COLLIDER_SHAPE_REFERENCE.md)
  Choose a shape and check supported pairings.
- [Continuous Collision Detection](CONTINUOUS_COLLISION_DETECTION.md)
  Follow a fast sphere's sweep into a thin wall.
- [Collision Response](COLLISION_RESPONSE.md)
  See how contacts, materials, and joints change motion.

## Query And Observe It

- [Query Services](QUERY_SERVICES.md)
  Choose a raycast, overlap, sweep, all-hit query, or batch.
- [Query Reference](QUERY_REFERENCE.md)
  Check hit geometry, filtering, ordering, and reentrancy.
- [Diagnostics](DIAGNOSTICS.md)
  Read events and counters or collect debug-draw commands.
- [Diagnostic Adapters](DIAGNOSTIC_ADAPTERS.md)
  Feed diagnostics into a renderer, structured log, or replay timeline.

## Packages And Compatibility

| Package         | Profile                                                                |
| --------------- | ---------------------------------------------------------------------- |
| `Gravitas`      | Standard package with built-in MemoryPack support                      |
| `Gravitas.Lean` | Same core physics API without the direct MemoryPack runtime dependency |

Both variants target `netstandard2.1` and `net10.0`. Gravitas builds on
[FixedMathSharp](https://github.com/mrdav30/FixedMathSharp),
[SwiftCollections](https://github.com/mrdav30/SwiftCollections),
[GridForge](https://github.com/mrdav30/GridForge), and
[Chronicler](https://github.com/mrdav30/Chronicler).

## Project Links

- [Documentation site](https://mrdav30.github.io/Gravitas/)
- [API reference](https://mrdav30.github.io/Gravitas/api/Gravitas.html)
- [Coverage report](https://mrdav30.github.io/Gravitas/coverage/)
- [Source, issues, and releases](https://github.com/mrdav30/Gravitas)
- [Contributing guide](https://github.com/mrdav30/Gravitas/blob/main/CONTRIBUTING.md)

Conceptual documentation lives alongside the API reference under
`docs/api/guides`. Repository Markdown is the source of truth; generated HTML is
disposable. Keep guide links relative and include their `.md` extension.
