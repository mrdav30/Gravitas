# Gravitas Benchmarks

This project is the BenchmarkDotNet scaffold for Gravitas physics hot paths.

The runner, alias catalog, and deterministic fixture helpers are in place.
Benchmark classes cover context lifecycle, registration/partitioning,
simulation, query-service paths, replay hashing, and diagnostics.

## Requirements

- .NET 8 SDK
- `Release` configuration for meaningful measurements

Avoid measuring `Debug` builds except when diagnosing benchmark setup failures.

## Running

Build the benchmark runner first, then execute the compiled DLL through the
configured `dotnet` host:

```bash
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0
```

On Linux/WSL this avoids `dotnet run` launching a generated apphost that does
not inherit capabilities such as `cap_sys_nice`. When the `dotnet` host is
configured for elevated process priority, run the built DLL so BenchmarkDotNet
can use that capability.

### List available benchmark selections

```bash
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll list
```

### Run all benchmarks

```bash
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll all
```

### Run a selection by alias

Aliases are derived from benchmark class names. `Benchmarks` or `Benchmark` is
stripped, and the remaining words are joined with `-`.

For a class named `CollisionDetectionBenchmarks`, the selection alias is
`collision-detection`:

```bash
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll collision-detection
```

Multiple aliases can run together:

```bash
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll collision-detection partitioning
```

### Forward BenchmarkDotNet arguments

Arguments after the selection are forwarded to BenchmarkDotNet:

```bash
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll all --list flat
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll collision-detection --filter "*Sphere*"
```

### Fast development check

Use BenchmarkDotNet's short in-process job for quick local smoke runs. This
verifies benchmark code compiles and produces plausible output without a full
run:

```bash
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll all -j Short -i
```

Do not treat short-run numbers as canonical measurements.

### Continuous collision evidence

`dynamic-ccd-scaling` keeps short dynamic CCD regression rows, while
`kinematic-active-ccd-scaling` covers host-driven kinematic active-source
no-hit, first-hit, dense-hit, rotational, and mixed source rows. Use the heavier
`continuous-collision-evidence` selection when collecting CCD performance
evidence for pure 2D, pure 3D, mixed full-runtime CCD, static query, dynamic
candidate-index, relative sweep, and shape-exact attribution:

```bash
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll continuous-collision-evidence --filter "*Evidence*" --exporters json
```

These rows are intentionally manual for now; do not wire them into CI until the
repo-wide benchmark publication/gating strategy is settled.

`piecewise-translational-ccd` isolates the allocation and bounded-scaling cost
of reducing one-, two-, and four-segment moving-target trajectories in the 2D
and 3D translational narrow phase:

```bash
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll piecewise-translational-ccd --filter "*Piecewise*" --exporters json
```

Rows with `FullRuntime` in the method name include benchmark reset,
host-transform publish, and simulation cost. Prefer the attribution rows when
you need allocation-focused signal for CCD query, candidate-index, or relative
sweep internals.

Shape-exact rows include static 3D non-sphere target false positives, static 2D
false positives, and dynamic 3D/2D relative false positives where proxy spheres
or circles find a candidate but the real mover shape rejects the contact.

### Continuous collision TOI iterations

Use `continuous-collision-toi-iteration` when comparing the bounded same-frame
TOI solver. The selection runs pure 2D and pure 3D two-contact static scenes
with `ContinuousCollisionMaxToiIterations` values of `1`, `2`, and `4`, so the
old first-hit clamp shape remains measurable as the `1`-iteration configuration:

```bash
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll continuous-collision-toi-iteration --filter "*ToiIteration*" --exporters json
```

## Suggested Benchmark Areas

Start with hot paths that can be isolated and repeated deterministically:

- `GravitasWorldContext` simulation phases and service phase ordering.
- `GravitasWorldContext.ComputeReplayHash(...)` conformance signal cost across
  sparse 3D, dense 3D, pure 2D, mixed, and solver-cache modes.
- `GravitasPhysicsService` body/collider registration and collision-pair
  ownership.
- `GravitasCollisionService` partitioning and partition cleanup.
- `CollisionDetection` shape-pair checks.
- `CollisionResponse` contact resolution across primitive, resting, cylinder,
  mesh, and mixed-dimension prepared contacts.
- continuous collision detection policy and swept movement cost.
- production CCD evidence through pure 2D, pure 3D, mixed full-runtime, static
  query, dynamic candidate-index, dynamic relative sweep, and shape-exact
  false-positive scenarios, including dynamic 3D/2D relative shape rejection.
- kinematic active CCD source scaling through no-hit, first-hit, dense-hit,
  rotational, and mixed source rows.
- bounded CCD TOI iteration solving through one-iteration, two-iteration, and
  default multi-iteration two-contact scenes.
- pure 2D host-agent setup, runtime-mode gated integration, GridForge-backed
  broad phase, sweep baselines, narrow-phase pairs, response, and overlap and
  raycast queries.
- collider shape-state rebuilds, capsule derived state, compound aggregate
  bounds, and mesh validation/BVH construction.
- `GravitasQuery2DService` and `GravitasQuery3DService` query gathering,
  filtering, and result ordering.
- Mesh collider preprocessing and convex mesh limits.
- Pooling and allocation behavior for collision pairs, partitions, and temporary
  collections.
- Diagnostics disabled overhead and enabled capture cost for event hooks,
  primitive debug draw commands, and mesh-heavy debug draw capture.

## Authoring Guidelines

- Put benchmark classes in the `Gravitas.Benchmarks` namespace.
- Prefer one benchmark class per subsystem or scenario group.
- Apply `[MemoryDiagnoser]` to benchmark classes unless there is a specific
  reason not to.
- Use deterministic fixtures and fixed seeds. Do not use ambient randomness in
  measured paths.
- Create isolated `GravitasWorldContext` instances for measured scenarios, or
  use `BenchmarkEnvironment.PrepareWorld(...)` when a benchmark only needs raw
  GridForge setup.
- Reset or dispose context/world state between benchmark cases so measurements
  do not depend on previous cases.
- Capture both throughput and allocation impact when changing hot-path
  collections, pooling, collision dispatch, or broad-phase behavior.

Keep support helpers physics-specific. Remove copied template helpers when they
stop serving a Gravitas benchmark scenario.

## Baseline Artifacts

Before starting optimization work, capture a baseline:

```bash
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll all --exporters json
```

BenchmarkDotNet writes results to `BenchmarkDotNet.Artifacts/results/` by
default. Archive the JSON or markdown reports before changing algorithms so
regressions can be compared against known results.

## Allocation Smoke Targets

For quick allocation checks around the current steady-state hot paths, run:

```bash
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll query-service simulation-allocation continuous-collision collision-detection collision-response mixed-collision-response collision-partition partition-culling diagnostics physics-2d mixed-broad-phase replay-hash --filter "*" -j Short -i --exporters json
```

The short in-process job is not canonical timing evidence, but it is useful for
catching obvious managed allocations. These scenarios should report no managed
allocation in steady-state paths unless noted below:

BenchmarkDotNet short in-process runs can occasionally report `1 B/op` noise on
otherwise allocation-guarded paths. Treat repeatable non-zero values as a reason
to add or tighten explicit allocation tests before changing the algorithm.

| Alias                          | Covered paths                                                                                                                                                                                                                                                                           |
| ------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `query-service`                | `RaycastAll`, `OverlapCircleAll`, directional `OverlapCircleInDirection`, swept-sphere queries, convex-source sweeps, high-vertex convex mesh source scaling, and overlapping-context queries.                                                                                          |
| `radial-raycast`               | Direct sphere, capsule, and finite-cylinder segment intervals; swept-sphere capsule/cylinder reduction; and mixed circle-slab reduction at ordinary and saturation-prone Q32.32 scales.                                                                                                  |
| `simulation-allocation`        | `SolidBody.LateSimulate`, grounding raycast probes, collision partition distribution, and active-pair late simulation.                                                                                                                                                                  |
| `continuous-collision`         | Discrete fast body movement baseline and opt-in CCD sweep/clamp against thin static geometry.                                                                                                                                                                                           |
| `kinematic-active-ccd-scaling` | host-driven kinematic active-source CCD rows for no-hit, first-hit, dense-hit, rotational, and mixed source scenarios.                                                                                                                                                                  |
| `collision-partition`          | dynamic/static registration and partitioning, partitioned simulation, and reset plus dynamic re-registration churn.                                                                                                                                                                     |
| `collision-detection`          | prepared primitive pairs, non-SAT primitive pairs, primitive manifold generation, cuboid face-manifold generation, cuboid SAT, cuboid/capsule, mesh/capsule, mesh/cylinder, mesh/cuboid, mesh/mesh, and compound/primitive checks.                                                      |
| `collision-response`           | manifold response solver cost across single-contact, face-manifold, resting face-manifold, cylinder-contact, and mesh-contact prepared pairs, with pair-count scaling.                                                                                                                  |
| `mixed-collision-response`     | constrained mixed 3D/2D response cost for prepared sphere/circle contacts, including single-pass pairs and bounded mixed-iteration loops.                                                                                                                                               |
| `diagnostics`                  | Disabled/enabled force and torque event hooks plus disabled/enabled primitive and mesh collider debug draw capture.                                                                                                                                                                     |
| `partition-culling`            | dynamic collider repartitioning after teleports, direct partition add/remove churn, and culled-pair invalidation after movement.                                                                                                                                                        |
| `physics-2d`                   | pure 2D body integration, GridForge-backed 2D partition response, direct angular contact response, direct two-contact manifold response, convex/convex two-contact manifold detection, sweep baseline comparisons, required 2D shape-pair checks, `OverlapCircleAll`, and `RaycastAll`. |
| `replay-hash`                  | deterministic authoritative replay hash cost for sparse 3D, dense 3D, pure 2D, mixed, and cache-inclusive solver/hash modes.                                                                                                                                                            |

`continuous-collision-evidence` and `continuous-collision-toi-iteration` are
intentionally omitted from the allocation smoke command because they are heavier
manual evidence selections rather than fast local guardrails.

Collider shape work has a focused selection:

```bash
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll collider-shape --filter "*" -j Short -i --exporters json
```

### 2026-07-27 Canonical Geometry Closure

The Task 9 comparison retained the Task 0 and final short in-process artifacts
under `artifacts/benchmarks`. Exact candidate ranking now rounds only the
winning `FixedOrientedBox` contact depth. The affected 64-pair rows improved
from `5.113` to `2.499 ms` for cuboid/cuboid, `25.209` to `10.835 ms` for
cuboid/capsule, and about `12.0` to `7.0 ms` for mesh/cuboid. Capsule runtime
shape rebuild improved from `1.609` to `1.376 ms`, and compound movement from
`62.82` to `58.76 us`.

These are short-run diagnostic measurements, not a claim of parity with the
older saturating geometry. The remaining exact ordinary-domain throughput gap
is tracked in
`docs/feature-work/benchmark-signal-hardening-backlog.md`. Allocation tests,
not the occasional single-digit-byte in-process runner noise, remain the
authoritative allocation gate.

## CI Guidance

CI should at minimum compile the benchmark project in `Release`. The normal
`Gravitas.slnx` build already includes `tests/Gravitas.Benchmarks`; use this
direct command when isolating benchmark compilation locally:

```bash
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj --configuration Release
```

Running full benchmarks in CI is optional until local variance is understood.
When performance gates are introduced, prefer BenchmarkDotNet comparison support
or stored baseline artifacts over raw timing thresholds, which are sensitive to
runner hardware.
