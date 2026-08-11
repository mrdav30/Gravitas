# Gravitas Wiki

Gravitas is deterministic fixed-point physics for lockstep games, simulations,
and engine-agnostic .NET hosts. It provides first-class 2D and 3D physics plus
an explicit mixed 2D/3D runtime, all owned by isolated world contexts.

The central idea is simple: your host owns commands, rendering, and the outer
loop; one `GravitasWorldContext` owns the clock, bodies, colliders, collisions,
constraints, queries, replay state, and diagnostics for one simulation.

## Start Here

If you are new to Gravitas, follow this path:

1. [Getting Started](GETTING_STARTED.md) — install a package, create a context,
   add spatial coverage, and step your first body.
2. [Technical Overview](OVERVIEW.md) — learn the ownership model and the major
   runtime services.
3. [Host Integration](HOST_INTEGRATION.md) — wire bodies, colliders, lifecycle,
   queries, replay, and teardown into a real host.
4. [2D, 3D, And Runtime Modes](DIMENSIONS.md) — choose the dimensional model
   that matches your simulation.

Already know what you need? Jump to a topic below.

## Build The Runtime

| Guide | Use it for |
| ----- | ---------- |
| [Runtime Architecture](RUNTIME_ARCHITECTURE.md) | Context-owned services, frame phases, registration, and state ownership |
| [Host Integration](HOST_INTEGRATION.md) | Host adapters, bodies, colliders, settings, constraints, and teardown |
| [2D, 3D, And Runtime Modes](DIMENSIONS.md) | Pure 2D, pure 3D, `Both`, mixed embedding, and dimensional queries |
| [Serialization And Replay](SERIALIZATION.md) | Chronicler state transfer, shell ownership, continuation, and replay hashes |

## Understand Collision

| Guide | Use it for |
| ----- | ---------- |
| [Collision Pipeline](COLLISION_PIPELINE.md) | End-to-end phase order, filtering, detection, response, and events |
| [Collision Broad Phase](COLLISION_BROAD_PHASE.md) | GridForge partitions, candidate discovery, mobility buckets, and cleanup |
| [Collider Shape Reference](COLLIDER_SHAPE_REFERENCE.md) | Primitive, compound, mesh, 2D, 3D, and mixed shape support |
| [Continuous Collision Detection](CONTINUOUS_COLLISION_DETECTION.md) | Sweeps, time of impact, active sources, and rotational CCD |
| [Collision Response](COLLISION_RESPONSE.md) | Manifolds, materials, islands, warm starts, sleep, constraints, and notifications |

## Query And Observe It

| Guide | Use it for |
| ----- | ---------- |
| [Query Services](QUERY_SERVICES.md) | Practical 2D, 3D, mixed, all-hit, and batch query workflows |
| [Query Reference](QUERY_REFERENCE.md) | Reducers, witnesses, filtering, mesh boundaries, ordering, and reentrancy |
| [Diagnostics](DIAGNOSTICS.md) | Diagnostic events, counters, typed views, and debug-draw commands |
| [Diagnostic Adapters](DIAGNOSTIC_ADAPTERS.md) | Host-side rendering, structured logging, and replay timeline tools |

## Packages And Compatibility

| Package | Profile |
| ------- | ------- |
| `Gravitas` | Standard package with built-in MemoryPack support |
| `Gravitas.Lean` | Same core physics API without the direct MemoryPack runtime dependency |

Both variants target `netstandard2.1` and `net8.0`. Gravitas builds on
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

The Markdown files in `docs/wiki` are the source of truth for this wiki. Keep
links between wiki pages relative and include their `.md` extension; the sync
workflow rewrites only the routes needed by GitHub Wiki.
