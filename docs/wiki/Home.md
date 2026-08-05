# Gravitas Wiki

Gravitas is an engine-agnostic, deterministic fixed-point physics library for
lockstep simulations and games. Runtime state belongs to an explicit
`GravitasWorldContext`, with first-class 3D, pure 2D, and mixed 2D/3D paths.

Use this wiki for integration guidance and behavioral contracts. Use the
generated API documentation for individual types and members.

## Project Links

| Destination                                                           | Use it for                                          |
| --------------------------------------------------------------------- | --------------------------------------------------- |
| [README](https://github.com/mrdav30/Gravitas/blob/main/README.md)     | Installation, package selection, and first concepts |
| [Documentation Site](https://mrdav30.github.io/Gravitas/)             | Generated documentation landing page                |
| [API Reference](https://mrdav30.github.io/Gravitas/api/Gravitas.html) | Public namespaces, types, and members               |
| [Coverage Report](https://mrdav30.github.io/Gravitas/coverage/)       | Current test coverage details                       |
| [GitHub Repository](https://github.com/mrdav30/Gravitas)              | Source, issues, releases, and contributions         |

## Wiki Navigation

| Page                                                                | Focus                                                         |
| ------------------------------------------------------------------- | ------------------------------------------------------------- |
| [Technical Overview](OVERVIEW.md)                                   | Runtime ownership, major services, and intentional boundaries |
| [Host Integration](HOST_INTEGRATION.md)                             | Host loop, context lifecycle, bodies, and colliders           |
| [Runtime Architecture](RUNTIME_ARCHITECTURE.md)                     | Context-owned services and simulation phases                  |
| [Dimensions](DIMENSIONS.md)                                         | 3D, pure 2D, `Both`, and mixed runtime modes                  |
| [Collision Pipeline](COLLISION_PIPELINE.md)                         | Broad phase, narrow phase, pairs, and response                |
| [Collision Broad Phase](COLLISION_BROAD_PHASE.md)                   | GridForge partitioning and candidate discovery                |
| [Collider Shape Reference](COLLIDER_SHAPE_REFERENCE.md)             | Supported shapes, compounds, meshes, and authored data        |
| [Continuous Collision Detection](CONTINUOUS_COLLISION_DETECTION.md) | Sweeps, time of impact, and active sources                    |
| [Collision Response](COLLISION_RESPONSE.md)                         | Contacts, materials, constraints, and solver behavior         |
| [Query Services](QUERY_SERVICES.md)                                 | Public 2D, 3D, and mixed query workflows                      |
| [Query Reference](QUERY_REFERENCE.md)                               | Query reducers, batching, filtering, and hit details          |
| [Serialization](SERIALIZATION.md)                                   | Chronicler state transfer and replay hashing                  |
| [Diagnostics](DIAGNOSTICS.md)                                       | Runtime diagnostic events and debug draw data                 |
| [Diagnostic Adapters](DIAGNOSTIC_ADAPTERS.md)                       | Host-side rendering, logging, and replay tooling              |

Start with the Technical Overview, then choose the page matching the subsystem
you are integrating or changing.
