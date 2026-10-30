# Post-Step 3D Query Pose Publication

**Status:** Complete; GRV-Issue-145 resolved on 2026-10-29 under delegated
correctness/performance judgment.
**Date:** 2026-10-29.

## Defect And Reproduction

3D positional response updated the authoritative body position while its
collider retained the pose prepared before response. Query preparation refreshed
grid topology and deferred 2D membership, but did not publish this 3D pose.
The defect reproduced in pure 3D, Both, Mixed and direct 3D service stepping.
Post-step queries could return stale distances or miss newly covered voxels.
Pure 2D already rebuilt its corrected collider and required no runtime change
for the original stale-geometry defect. The light 2D follow-up below aligns
correction membership publication after that fix.

The original report was verified on Gravitas `3bd4574` with FixedMathSharp
`10a2ae1` and reproduced against the frozen current base `cdc0acd`:

- Use Mixed mode at 16 Hz, zero gravity, air density, damping and minimum speed;
  frictionless materials, manual grounding and disabled sleep.
- Place a mass-one capsule at `(-2,0,0.5)`, radius `0.5`, core length `2`, with
  frozen Y translation and rotation. The bodyless native 2D rectangle covers
  `X=[-8,8], Z=[-2,0]`, with mixed half-thickness `2`.
- Settle 32 complete `Simulate`/`LateSimulate` steps. Run four 64-step traversals
  with tangential X velocity alternating `+1/-1`, adding inward Z impulse
  `-1/16` before every traversal step.
- Raycast backward from `(body.X,body.Y,2)` for distance `4` immediately after
  the last complete step.

The authoritative final Z is raw `2100339672`. Unfixed collider Z is raw
`2083562456`, giving ray distance raw `4358888488` instead of `4342111272`.
The error is `16777216` raw, or `1/256` unit. The corrected implementation keeps
the body trajectory and returns the distance from the authoritative pose.

The maintained reproduction and additional public behavior checks are in
[`PostResponseQueryPoseTests`](../../../tests/Gravitas.Tests/Queries/PostResponseQueryPoseTests.cs).
The frozen unfixed payload fails eight behavior cases; the two allocation
guards already pass. All ten cases pass with the fix, including new-voxel
discovery, ray/all-hit/sweep/overlap/batch/explicit mixed queries and callback
geometry/COM/inertia publication.

## Implementation And Invariants

Response marks changed bodies. Successful context completion publishes their
final collider geometry, 3D membership and mixed membership after response,
retained-contact and supplier bookkeeping. Direct `Physics.LateSimulate()`
also publishes. This ordering preserves solver snapshots and avoids another
same-step retained narrow-phase refresh or replacement of supplier witnesses.

A service flag avoids scanning bodies on steps without correction; a body flag
limits publication to changed bodies. Initialization/reset preserve lifetime
boundaries. There is no new reference queue or query-time registration scan.
Unchanged mass properties are retained for translation-only publication;
accepted authored geometry and scale still refresh COM/inertia. Existing CCD
callers retain their explicit no-mass-refresh semantics. A failed step still
requires restoring a known state under the existing host contract.

The final optimization compacts accepted 3D voxels into the prefix of the
existing scratch list. Publication follows that prefix in original tracer
order, removing a repeated hash-set coverage check in the 3D and mixed owners.
The accepted-coordinate set still drives departure removal. Accepted voxels
still undergo partition lookup/repair and awake synchronization. No coverage
rule, capacity, cache lifetime or topology policy changes.

Existing
[`PartitionDeltaRefreshTests`](../../../tests/Gravitas.Tests/Partitions/PartitionDeltaRefreshTests.cs)
verify coordinate order/storage, departures, membership repair and awake
synchronization. The query and partition regressions exercise production
owners rather than a duplicate compaction algorithm.

## Performance And Disposition

The user explicitly delegated final correctness/performance judgment on
2026-10-29 and requested easy improvements. The small coverage-loop improvement
was retained. The remaining cost is accepted under that delegation: publishing
the completed pose fixes incorrect public results, while removing the rest of
the spatial refresh would require a broader indexing or geometry design.

**Each number below is median microseconds for one complete host step with one
moving collider and one static wall.** The Ray rows additionally perform one
post-step 3D raycast. These are local matched timings, not large-scene capacity
or BenchmarkDotNet confidence intervals.

The workload is maintained in
[`PostResponseQueryBenchmarks`](../../../tests/Gravitas.Benchmarks/Queries/PostResponseQueryBenchmarks.cs):

- A frictionless radius-0.5 capsule with core length `2` and frozen rotation,
  or a two-sphere compound with rotation enabled. The compound parts have radius
  `0.5` and local X offsets `-0.5/+0.5`.
- A pure 3D cuboid wall or a bodyless native 2D wall in Mixed mode. All bodies
  use manual grounding, disabled sleep and frozen Y translation. Gravity,
  air density, damping and minimum speed are zero.
- At 16 Hz, tangential velocity alternates `+1/-1` at X=`-2/+2`, with an inward
  `1/16` impulse each frame. Every benchmark invocation runs 64 complete steps.

| Mode | Mover | Query | Unfixed us/step | First fix us/step | Final us/step | Extra vs unfixed us/step | Extra vs unfixed |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| 3D | Capsule | None | 86.93 | 104.55 | 101.47 | 14.54 | 16.7% |
| 3D | Capsule | Ray | 112.34 | 131.21 | 127.73 | 15.40 | 13.7% |
| 3D | Compound | None | 81.33 | 108.62 | 106.10 | 24.77 | 30.5% |
| 3D | Compound | Ray | 107.36 | 136.35 | 134.16 | 26.80 | 25.0% |
| Mixed | Capsule | None | 181.57 | 202.63 | 199.51 | 17.95 | 9.9% |
| Mixed | Capsule | Ray | 210.63 | 231.67 | 228.27 | 17.64 | 8.4% |
| Mixed | Compound | None | 355.05 | 389.39 | 388.04 | 32.99 | 9.3% |
| Mixed | Compound | Ray | 384.53 | 421.59 | 418.54 | 34.01 | 8.8% |

The first-fix variant has the same deferred publication but publishes from the
unfiltered voxel list through an additional hash-set check. Final code instead
retains the accepted prefix. Capsule and pure-3D compound controls improved in
both run orders, generally by `1-3` us per step. Mixed compound differences were
small/inconsistent enough that no performance improvement is claimed there.

All 48 measurement processes completed: eight workloads, three runtime payloads
and two fresh processes per payload/workload, reversing payload order on the
second run. After the benchmark setup's 16 warmup invocations, each process ran
32 more warmup invocations, then nine samples of 16 invocations each. Each
sample therefore measured 1,024 complete steps. Medians pool the 18 samples
per payload/workload. `DOTNET_TieredCompilation=0` selected fully optimized JIT
code; no builds, tests or profilers competed with measurements.

All measured warm allocations were zero. Final raw positions, rotations,
linear/angular velocities, mass, sleep and COM offsets matched across payloads.
Runtime SHA-256 identities were checked; benchmark and FixedMathSharp payload
hashes matched across all processes. Earlier sampled-stack profiling identified
the additional spatial refresh. An initial rotating mesh-compound control was
stopped without a result because its geometry workload exceeded the bounded
measurement budget; no mesh performance claim is made.

## Validation And Limits

Validation uses Windows x64/.NET `10.0.10`, SDK `10.0.302`, Gravitas base
`cdc0acd`, FixedMathSharp `03f6cbf`, SwiftCollections `89dfa23`, GridForge
`89c69f6` and Chronicler `c547e5c`, with `UseLocalLsfStack=true`.
Only Gravitas has source changes.

Final optimization validation passed without skips:

| Profile | Tests passed | Raw lines covered | Raw branches covered | Methods covered |
| --- | ---: | ---: | ---: | ---: |
| Release | 8,155 / 8,155 | 53,363 / 53,363 | 21,594 / 21,594 | 5,591 / 5,591 |
| ReleaseLean | 8,084 / 8,084 | 53,361 / 53,361 | 21,594 / 21,594 | 5,590 / 5,590 |

Both profiles have exact 100% reachable line, branch and method coverage.
Independent `ponytail-review` found no source blocker in the deferred
publication or the coverage-loop optimization.

All eight shared replay fixtures were audited against the frozen unfixed
runtime. Every per-frame body state, ordered event and query observation
matched. Six fixtures required hash-only updates because final collider
centers, bounds and shape/partition versions contribute to hashes. Inputs and
schema are unchanged; corrected-pose replay hashes intentionally differ.

The `netstandard2.1` library and net10 benchmark builds passed without warnings.
All eight in-process BenchmarkDotNet Dry rows also executed.
Stock out-of-process generation is blocked by older ignored snapshots bearing
the same `Gravitas.Benchmarks.csproj` name. Dry runs are launch checks, separate
from warm allocation/timing evidence.

Package restore fails `NU1101` for unavailable `Chronicler.Diagnostics` in the
configured feeds. Source-stack results do not certify released packages or
other native platform lanes. No release/version claim is made.

Run the full tests and coverage from the repository root:

```powershell
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj -c Release -p:UseLocalLsfStack=true --collect:'XPlat Code Coverage' --settings tests/Gravitas.Tests/coverlet.runsettings
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj -c ReleaseLean -p:UseLocalLsfStack=true --collect:'XPlat Code Coverage' --settings tests/Gravitas.Tests/coverlet.runsettings
```

Build and smoke-test the maintained workload:

```powershell
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -p:UseLocalLsfStack=true
$env:UseLocalLsfStack = 'true'
dotnet tests/Gravitas.Benchmarks/bin/Release/net10.0/Gravitas.Benchmarks.dll --filter '*PostResponseQueryBenchmarks*' --job Dry --inProcess
```

Supplemental captures, raw coverage, traces and frozen payloads are in
`artifacts/grv-145/`; the inputs, variant definitions, protocol, results and
maintained regressions above are sufficient without those temporary artifacts.
Reopen for an observable query/body pose mismatch after successful completion,
missed newly covered voxels, altered solver/supplier behavior or new warm
allocations, retaining the exact fixture and dependency revisions.

## Light 2D Publication Follow-Up

The fast follow to committed #145 (`f6fe03a`) aligns spatial publication with
the existing 3D service/body flags. It adds 41 runtime lines and removes five,
with no new production files, queues, dependencies or public APIs.
2D geometry still updates immediately: static-surface recovery reads current
collider geometry during response. Native and mixed membership instead coalesce
until collision distribution, indexed 2D candidate gathering, or successful
context/service completion. Mixed publication uses committed geometry without
rebuilding, preserving any prepared recovery proposal. Translation-only
corrections retain mass properties; pending shape/scale/slab changes still
refresh them. Existing CCD callers keep their previous mass-refresh behavior.

This is mechanism alignment, not a new pure-2D stale-query defect. The original
3D geometry timing remains necessary for retained-contact bookkeeping. A
pending internal 2D query may flush flagged bodies; ordinary post-step queries
already have current membership.

Ten cases in
[`ResponsePosePublication2DTests`](../../../tests/Gravitas.Tests/Physics2D/ResponsePosePublication2DTests.cs)
cover immediate geometry with coalesced native/mixed membership, query flush,
pre-distribution discovery, completion in TwoD/Both/Mixed and standalone service
steps, dirty-shape COM/inertia, and zero warm allocation. Removing the new
pre-distribution flush makes the voxel-crossing contact regression fail
(`0` contact enters instead of `1`); restoring it passes. The focused recovery
and shared-replay selection also passes without changing fixture expectations.

The performance comparison reuses existing `FullStep` fixtures with
`Layout="Continuous"`: `JoinedStaticSurface2DSimulationBenchmarks` and
`JoinedStaticSurfaceMixedSimulationBenchmarks`. Each has one moving circle,
capsule or square polygon against one bodyless wall, frictionless materials,
manual grounding, disabled sleep and frozen rotation at 16 Hz. The native floor
is one 16-by-2 rectangle, with gravity `1`. The mixed wall is a vertical
two-triangle 3D mesh, with zero gravity, a 2D slab half-thickness of `0.5` and
inward impulse `1/16` each step. Tangential X drive alternates `+1/-1` at
X=`-2/+2`. The circle radius is `0.5`, the capsule uses constructor arguments
`(0.5, 2)`, and the polygon is a unit square. Air density, damping and minimum
speed are zero. These are complete-step timings without an added query.

**Median microseconds per complete host step, one mover and one wall:**

| Mode | Mover | Committed #145 us/step | Follow-up us/step | Change us/step | Change |
| --- | --- | ---: | ---: | ---: | ---: |
| Native 2D | Circle | 25.34 | 25.21 | -0.13 | -0.5% |
| Native 2D | Capsule | 29.74 | 30.04 | +0.30 | +1.0% |
| Native 2D | Polygon | 43.17 | 43.21 | +0.04 | +0.1% |
| Mixed | Circle | 1,571.55 | 1,584.81 | +13.26 | +0.8% |
| Mixed | Capsule | 2,542.32 | 2,552.91 | +10.59 | +0.4% |
| Mixed | Polygon | 793.48 | 816.97 | +23.49 | +3.0% |

All 24 fresh processes completed: six workloads, two payloads and two runs,
reversing payload order on the second run. Each process uses the fixture's
32-step settling and four 64-step warmup invocations, then 16 additional warmup
invocations. Nine samples each measure eight invocations, or 512 complete
steps. Medians pool 18 samples per payload/workload. Fully optimized JIT code
uses `DOTNET_TieredCompilation=0`; no builds, tests or profilers compete.
Only `Gravitas.dll` changes between frozen payloads. Loaded runtime SHA-256
identities match the selected variants; benchmark and FixedMathSharp hashes
are identical. The committed runtime hash starts `E08B35A74A5C556F`, and the
follow-up starts `C84B3DD9C8D7EBA2`.

All measured warm allocations are zero. Final raw position, rotation,
linear/angular velocity, mass, inertia and COM offset match across variants
and runs; the existing fixtures assert tangential velocity every step. Shared
replay fixtures pass unchanged. Small/inconsistent differences are not evidence
of a speedup or exact equivalence. The mixed polygon increase appears in both
orders (`+19.29` and `+22.74` us using individual-run medians). The follow-up
adds mixed membership publication at correction completion, while the old
2D correction path refreshed only its native membership immediately.
The measured cost is retained under the user's delegated judgment: the largest
row is `+23.49` us on a `793.48` us complete step, and keeping this focused
alignment avoids a broader spatial-index redesign. These synthetic local
controls establish neither large-scene scaling nor gameplay-frame percentiles.

Follow-up validation uses the same Windows x64/.NET/local-stack environment
listed above, now based on Gravitas `f6fe03a`. Both full suites pass without
skips, with exact raw coverage:

| Profile | Tests passed | Raw lines covered | Raw branches covered | Methods covered |
| --- | ---: | ---: | ---: | ---: |
| Release | 8,165 / 8,165 | 53,386 / 53,386 | 21,604 / 21,604 | 5,594 / 5,594 |
| ReleaseLean | 8,094 / 8,094 | 53,384 / 53,384 | 21,604 / 21,604 | 5,593 / 5,593 |

The `netstandard2.1` library and net10 benchmark builds pass with zero warnings.
Independent `ponytail-review` found no actionable source, test or documentation
defect. Source-stack validation does not add package or cross-platform evidence
to the limits recorded above. Follow-up captures live under
`artifacts/grv-145/2d-followup/`.
