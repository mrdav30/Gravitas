# Gravitas Benchmarks

This project provides BenchmarkDotNet coverage for Gravitas physics hot paths.
Its alias runner and deterministic fixtures cover context lifecycle,
registration/partitioning, simulation, query services, replay hashing, and
diagnostics.

## Requirements

- The SDK selected by the repository's `global.json`
- .NET 8 runtime for the `net8.0` benchmark executable
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
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll collision-detection collision-partition
```

### Forward BenchmarkDotNet arguments

Arguments after the selection are forwarded to BenchmarkDotNet:

```bash
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll all --list flat
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll collision-detection --filter "*Sphere*"
```

The launcher returns nonzero for malformed arguments, unmatched filters,
critical validation errors, build failures, and reported child executions with
missing workload results or nonzero exits. Successful earlier launches do not
hide later failures. Valid help, list, version and information commands return
zero. Retain the complete logs when interpreting measurements; the exit code
checks the results exposed by BenchmarkDotNet, not unreported diagnoser work.

### Coordinated source-stack builds

When validating unreleased sibling libraries, build the runner explicitly in
source mode:

```powershell
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -p:UseLocalLsfStack=true -p:BuildInParallel=false -m:1
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll world-context --filter '*RunEmptySimulationFrame*' --job Dry
```

The compiled runner carries source mode into generated benchmark builds,
disables implicit transitive project references that bypass the explicitly
versioned source edges, and serializes those project builds. No launch-time
environment variable is required. Generated jobs retain the runner's build
configuration: to check Lean, replace `Release` with `ReleaseLean` in both
commands. CLI job choices such as `Dry` still apply. Dry runs validate execution,
not throughput. Package mode remains the default and is a separate release gate.

### Fast development check

Use BenchmarkDotNet's short in-process job for quick local smoke runs. This
verifies benchmark code compiles and produces plausible output without a full
run:

```bash
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll all -j Short -i
```

Do not treat short-run numbers as canonical measurements.

### Pure-2D circle contact simulation

The `circle-contact-simulation` selection measures 64 and 1024 independent
dynamic/static circle pairs with axis-aligned, diagonal and rotated geometry.
GridForge cells are 16 units in X/Z and one unit in Y, matching pair spacing
to keep this a sustained-contact workload rather than a fine-voxel scaling test.
`ResetAndFullSimulationStep` runs both context `Simulate` and `LateSimulate`,
including broad phase, contact generation, response and bookkeeping. Each
invocation resets dynamic pose and all accumulated motion; sleep is disabled
so repeated invocations retain the same contact workload. Setup verifies known
contact depths/normals, one active contact per pair, candidate counts and
repeatable post-response body state across warmed frames.

`ResetAndDetectContacts` measures the same pose reset plus direct dispatcher
queries over those pairs. Both rows include reset cost; their difference is
useful attribution evidence, not an exact measurement of any single stage.
The `circle-contact` selection remains the individual-query geometry baseline.

```powershell
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll circle-contact-simulation --filter '*' --exporters json
```

### Pure-2D partition scaling

`cold-circle-registration` and `circle-partition-maintenance` use the same
radius-five diagonal circle pairs at 16-unit spacing. `ColliderCount` is the
total dynamic plus static collider count (64, 256 or 1024); `CellSize` selects
one-unit or 16-unit X/Z cells, with one-unit Y cells and padded grid bounds.
Setup checks known contact geometry and warmed setup verifies one active
contact/candidate per pair.

`RegisterCirclePairs` constructs and registers bodies/colliders into a fresh
context once per iteration. Context/grid and array creation happen outside the
measurement; registration allocations are intentional and exclude grid setup.
Cleanup validates the registered scene and disposes it outside measurement.
The warmed rows separate translated pose reset plus partition refresh, forced
automatic grounding probes, and retained-candidate distribution. These ground
probes run at reset poses and find no support from the above-body targets.
`DistributeRetainedCandidates` retains processed pair keys to isolate partition
traversal, filtering and duplicate suppression; it deliberately skips repeated
narrow phase and response. Use `circle-contact-simulation` for full-step costs.

```powershell
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll cold-circle-registration circle-partition-maintenance --filter '*' --exporters json
```

These are manual scaling selections. Fine-cell cold registration can be very
expensive before retained-partition scaling fixes; use filtered small counts
when capturing the historical baseline. Compare allocation results per stage,
and retain the cell size and collider count beside timing results.

`circle-automatic-grounding` isolates automatic probes at corrected poses after
one full simulation step. `PairCount` is 64 or 1024 dynamic/static pairs, twice
as many colliders, with 16-unit X/Z cells. Correction retains slop, so the paired
target remains an initial full-radius overlap whose normal rejects support.
Subsequent grid rows can find the preceding row's circle below them. Setup checks
`PairCount - 32` supported bodies and stable results across repeated probes.
The measured operation includes no pose reset or simulation step.

`circle-ground-query-selection` compares collect/sort/filter with nearest
accepted grounding selection on those same corrected poses. Both paths reuse
prepared segments and apply the body's support policy. Setup compares every
complete hit witness and the 32/992 supported bodies at 64/1024 pairs. Timed
methods return the selected-owner/count checksum and include query selection
only; ground-state writes and diagnostics belong to `circle-automatic-grounding`.

`planar-scale-capture` compares the existing scale/admission capture with and
without materializing host yaw at zero and one radian. Both paths use the same
matrix decomposition and validation. The scale-only path represents body-owned
poses that supply their own yaw; bodyless colliders still need host yaw.

```powershell
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll circle-automatic-grounding circle-ground-query-selection planar-scale-capture --filter '*' --exporters json
```

`partition-membership` compares existing sparse, hash and packed collection
owners at 1, 8 and 64 local IDs starting at global ID 65536. It measures one
add/remove plus sorted copy after warmup, with no managed allocations. Hash
membership avoids global-ID-sized storage and wins the one-ID row; sparse
membership is faster in the denser microbenchmarks. Keep this tradeoff visible
when interpreting integrated partition results.

### 3D grounding and closest-query controls

`ground-probe-selection3-d` measures 64 bodies with ray (`UseSphere=false`) or
swept-sphere (`UseSphere=true`) probes and 1 or 8 reachable targets per probe.
`Supported` selects eligible far support or only physically ignored targets.
Near ignored targets remain raw query hits in the eight-target rows.
`ForceAutomaticGroundProbes` includes ground-state writes with diagnostics
disabled; `ClosestRawHits` runs public closest queries over the same geometry
without applying body support eligibility.

Setup validates complete closest witnesses against all-hit results, exact raw
counts, accepted points/normals/distances, candidate counters and diagnostic
events outside timing. Both methods are warmed before measurement. Use
`MemoryDiagnoser` results to assess allocations alongside throughput; this
fixture does not reset pose or advance a simulation step.

```powershell
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll ground-probe-selection3-d --filter '*' --exporters json
```

### Capsule/circle-slab contacts

The `capsule-circle-contact` selection measures mixed capsule/circle-slab side,
cap, endpoint-rim, separated-rim, oblique interior-rim and zero-core contacts.
The paired `CylinderContact` rows use equivalent 3D cylinders to check whether
expensive geometry affects both paths. They compare complete wrappers: the 3D
path also validates inputs and updates its manifold, so their difference is not
an isolated measurement of mixed integration overhead. Setup verifies both
classifications, depth agreement and the independently known quarter-unit depths.
Historical selected-axis results include false contacts and nonminimum depths;
their timings are implementation costs, not equivalent-correctness targets.
These are individual narrow-phase queries, not complete simulation frames.

### Mesh/cone contacts

The `mesh-cone-contact` selection measures base, side, apex, oblique-rim,
interior stationary-rim, touching/separated and full-domain contacts through mesh
dispatch. Setup verifies classification and the known exact depths before
timing; the unrepresentable-relative-center case checks intersection without
forcing a narrowed center. Compare individual rows rather than averaging
ordinary face contacts with curved-feature work.

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

Keep these evidence rows manual unless the repository adopts an explicit
benchmark publication and gating strategy.

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

For quick allocation checks around steady-state hot paths, run:

```bash
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll query-service simulation-allocation continuous-collision collision-detection collision-response mixed-collision-response collision-partition partition-culling diagnostics physics-2d mixed-broad-phase replay-hash --filter "*" -j Short -i --exporters json
```

The short in-process job is not canonical timing evidence, but it is useful for
catching obvious managed allocations. These scenarios should report no managed
allocation in steady-state paths unless noted below:

BenchmarkDotNet short in-process runs can occasionally report `1 B/op` noise on
otherwise allocation-guarded paths. Treat repeatable non-zero values as a reason
to add or tighten explicit allocation tests before changing the algorithm.

| Alias                                    | Covered paths                                                                                                                                                                                                                                                                           |
| ---------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `query-service`                          | `RaycastAll`, `OverlapCircleAll`, directional `OverlapCircleInDirection`, swept-sphere queries, convex-source sweeps, high-vertex convex mesh source scaling, and overlapping-context queries.                                                                                          |
| `query-projection-scaling`               | X/Z `OverlapCircleAll` scaling across dense and sparse grids as the semantically irrelevant world Y range grows.                                                                                                                                                                        |
| `dynamic-candidate-index-update-scaling` | Translation-only update and update-plus-query cost for the unique widest 3D and planar candidate as index population grows.                                                                                                                                                             |
| `radial-raycast`                         | Direct sphere, capsule, and finite-cylinder segment intervals; swept-sphere capsule/cylinder reduction; and mixed circle-slab reduction at ordinary and saturation-prone Q32.32 scales.                                                                                                 |
| `simulation-allocation`                  | `SolidBody.LateSimulate`, grounding raycast probes, collision partition distribution, and active-pair late simulation.                                                                                                                                                                  |
| `continuous-collision`                   | Discrete fast body movement baseline and opt-in CCD sweep/clamp against thin static geometry.                                                                                                                                                                                           |
| `kinematic-active-ccd-scaling`           | host-driven kinematic active-source CCD rows for no-hit, first-hit, dense-hit, rotational, and mixed source scenarios.                                                                                                                                                                  |
| `collision-partition`                    | dynamic/static registration and partitioning, partitioned simulation, and reset plus dynamic re-registration churn.                                                                                                                                                                     |
| `collision-detection`                    | prepared primitive pairs, non-SAT primitive pairs, primitive manifold generation, cuboid face-manifold generation, cuboid SAT, cuboid/capsule, mesh/capsule, mesh/cylinder, mesh/cuboid, mesh/mesh, and compound/primitive checks.                                                      |
| `mesh-cylinder-contact`                  | One-triangle finite-cylinder and mixed circle-slab contacts: cap, side, off-center cap intrusion, oblique rim, exact rim touch, and separated rim. Setup checks contact classification and independently known depths before measurement. |
| `capsule-slab-contact` | Mixed capsule/nonzero-core stadium-slab contacts: cap, side, straight rim, separated rim, rounded end, containment and oblique capsule-interior rim. Setup checks independently derived whole-shape depths. |
| `collision-response`                     | manifold response solver cost across single-contact, face-manifold, resting face-manifold, cylinder-contact, and mesh-contact prepared pairs, with pair-count scaling.                                                                                                                  |
| `mixed-collision-response`               | constrained mixed 3D/2D response cost for prepared sphere/circle contacts, including single-pass pairs and bounded mixed-iteration loops.                                                                                                                                               |
| `diagnostics`                            | Disabled/enabled force and torque event hooks plus disabled/enabled primitive and mesh collider debug draw capture.                                                                                                                                                                     |
| `partition-culling`                      | dynamic collider repartitioning after teleports, direct partition add/remove churn, and culled-pair invalidation after movement.                                                                                                                                                        |
| `physics-2d`                             | pure 2D body integration, GridForge-backed 2D partition response, direct angular contact response, direct two-contact manifold response, convex/convex two-contact manifold detection, sweep baseline comparisons, required 2D shape-pair checks, `OverlapCircleAll`, and `RaycastAll`. |
| `circle-contact` | Direct pure-2D circle contacts: axis-aligned, diagonal, rotated, coincident, large-coordinate overlap and bounds-admitted separation. Setup checks known classification, normal, depth and clamping before measurement. |
| `replay-hash`                            | deterministic authoritative replay hash cost for sparse 3D, dense 3D, pure 2D, mixed, and cache-inclusive solver/hash modes.                                                                                                                                                            |

`continuous-collision-evidence` and `continuous-collision-toi-iteration` are
intentionally omitted from the allocation smoke command because they are heavier
manual evidence selections rather than fast local guardrails.

Collider shape work has a focused selection:

```bash
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll collider-shape --filter "*" -j Short -i --exporters json
```

## CI Guidance

CI should at minimum compile the benchmark project in `Release`. The normal
`Gravitas.slnx` build already includes `tests/Gravitas.Benchmarks`; use this
direct command when isolating benchmark compilation locally:

```bash
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj --configuration Release
```

Full benchmark execution is optional in CI. Performance gates should use
BenchmarkDotNet comparison support or stored baseline artifacts rather than raw
timing thresholds, which are sensitive to runner hardware.
