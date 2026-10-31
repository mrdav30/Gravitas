# Benchmark Signal Hardening Backlog

## Purpose

This document captures benchmark-derived hardening signals that fall outside the
active feature plan. It is intentionally undated and long-lived: individual
entries carry their own discovery dates, evidence, status, and next isolation
step.

Use this backlog for measured performance, allocation, scaling, and benchmark
evidence concerns. Bugs or correctness risks that are not primarily benchmark
signals belong in [`issue-tracker.md`](issue-tracker.md). Broad feature or
architecture work should be promoted into its own dated plan and referenced from
this backlog.

## Intake Rules

- Signal IDs use `GRV-Benchmark-NNN`. The next available ID is
  `GRV-Benchmark-033`.
- Assign an ID at intake and never reuse it, including after a signal closes or
  moves into a dated plan. Check this file's Git history before advancing or
  repairing the counter.
- Add a signal only when it comes from a benchmark, allocation guardrail,
  profiler trace, or repeated validation run.
- Record the command, date, affected row or test, measured value, why it
  matters, and the smallest useful next isolation step.
- Keep benchmark-only instrumentation in tests or benchmark support unless the
  runtime needs a durable diagnostic API.
- Prefer a focused fix when the signal has a narrow cause.
- Promote to a dated feature-work plan when the signal spans multiple
  subsystems, requires API design, or needs staged implementation.
- Close entries only after a runtime/test/docs change lands or after a written
  no-change decision explains why the signal is expected.
- After closure, keep only the signal ID, closure date, brief outcome and
  evidence link in the closed table. Keep detailed reproducers, measurements,
  validation and reopening criteria in an existing completion record or dated
  archive, and preserve signal anchors for inbound links.

## Baseline Commands

Build the benchmark project before capturing evidence:

```powershell
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net10.0
```

List the continuous-collision evidence rows:

```powershell
dotnet tests/Gravitas.Benchmarks/bin/Release/net10.0/Gravitas.Benchmarks.dll continuous-collision-evidence --list flat
```

Run the current continuous-collision evidence smoke:

```powershell
dotnet tests/Gravitas.Benchmarks/bin/Release/net10.0/Gravitas.Benchmarks.dll continuous-collision-evidence --filter "*Evidence*" -j Short -i
```

After runtime changes, validate the package paths:

```powershell
dotnet test Gravitas.slnx --configuration Release
dotnet test Gravitas.slnx --configuration ReleaseLean
```

## Active Signals

### GRV-Benchmark-028 — Mixed Broad-Phase Work With An Empty Opposite Dimension

**Discovered:** 2026-10-15 during #135/#136 warm validation. **Status:**
Profiled pre-existing opportunity; the benefit of a distribution gate has not
been measured. **Owner:** Gravitas mixed candidate distribution and partition
maintenance.

The isolated `QueryOnly136WarmBenchmarks.Stable64Frame(Domain: "Mixed")`
contains 64 sleeping 3D spheres and their 3D static floors, with **zero 2D
colliders**. Automatic support crosses a real collider gap without contact
manifolds. The same fixture runs in pure 3D as a control. On the frozen
pre-#135/#136 Windows x64 stack, a quiet native comparison measured about
**438.6 us** for pure 3D and **804.4 us** for Mixed; both allocate `0 B`.

EventPipe sampling attributes **43.98%** of original whole-frame CPU weight to
`ProcessMixedContacts`; the candidate trace is similar at **44.65%**. These are
inclusive sampled weights, not call counts or proof that all this work can be
removed. Mixed indexing still serves queries and later population changes.
However, `ProcessMixedContacts` also copies, sorts and distributes active
partitions when one dimensional population is empty, so no mixed candidate can
exist. Its untouched-pair cleanup and retained-partition retirement still need
to run, including after callbacks remove the last opposite-dimension collider.

The ignored isolated host can reproduce the row with the local stack:

```powershell
$env:UseLocalLsfStack = 'true'
pwsh -NoProfile -File artifacts/grv-issue-135/perf/run-native.ps1 -Variant original -Label empty-dimension-repro -Filters '*QueryOnly136WarmBenchmarks*'
```

**Next isolation:** Promote the bounded fixture into the maintained benchmark
suite, then measure skipping only impossible candidate distribution while
preserving partition/query freshness, pair retirement, deterministic ordering
and `0 B` warm behavior. Exercise both empty-dimension directions and repeated
spawn/despawn transitions. Keep the #135/#136 regression gate independent of
this pre-existing follow-up.

Ignored `artifacts/grv-issue-135/perf/` retains the frozen original eight-module
stack (`89C15FBD...` core), unchanged fixture inputs, native rows,
`profile-attribution.json`, paired CPU traces and zero-ground-probe diagnostics.

### GRV-Benchmark-029 — Dense Mesh Sweep Cost After Exact Contact Hardening

**Discovered:** 2026-10-18 during #141 ordinary-consumer validation. **Status:**
Confirmed on the complete frozen pre-#141 stack; ownership requires profiling.
This is separate from #026's initially overlapping sources and #006's completed
benchmark instrumentation. **Owner:** Gravitas 3D/mixed mesh sweeps and the
executed FixedMathSharp finite query operations.

Existing `MeshQuery3DTriangleScalingBenchmarks` and
`MixedMeshTriangleScalingBenchmarks` keep the physical quad fixed while changing
subdivision from 8 to 32 (128 to 2,048 triangles). The sources travel from X=-3
to X=3; sphere/circle radius is 0.5, and the mixed circle has half-thickness
0.5. The convex source is the maintained cube fixture. These are ordinary
impacts, not initial overlap.

Frozen-stack Release controls on Windows x64/.NET 8.0.29 measure approximately:

| Public all-hit query                   | 128 triangles | 2,048 triangles |
| -------------------------------------- | ------------: | --------------: |
| Sphere against dense mesh              |    2.1–2.3 ms |    28.8–30.2 ms |
| Convex mesh against dense concave mesh |    5.5–5.7 ms |    88.6–92.0 ms |
| Mixed circle against dense mesh        |    3.0–3.2 ms |    46.1–47.3 ms |

All warmed windows allocate 0 B. Matched #141 controls reproduce the same
millisecond-scale cost; this entry does not defer an introduced regression. The
ranges are independent run means, not frame-tail latency or capacity claims.
Captures and complete loaded-module hashes are under
`artifacts/grv-issue-141/consumer-{refined,final,witness}-*.json`.

Reproduce the maintained fixtures with `UseLocalLsfStack=true`:

```powershell
$env:UseLocalLsfStack = 'true'
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -p:UseLocalLsfStack=true -- --filter '*MeshQuery3DTriangleScalingBenchmarks*' '*MixedMeshTriangleScalingBenchmarks*' --job short
```

**2026-10-30 shared-worker refinement:** GRV-Benchmark-026 defers GJK witness
construction. Frozen-source Release ABBA controls use the same maintained
`MeshQuery3DTriangleScalingBenchmarks`, with 24 warmed windows per launch and
two launches per payload. The convex-source path improves; the unchanged
analytic-sphere path provides an approximate timing control:

| Ordinary all-hit query | Triangles | Baseline ms | Candidate ms | Change |
| ---------------------- | --------: | ----------: | -----------: | -----: |
| Sphere control         |       128 |       2.020 |        1.997 | -1.16% |
| Convex mesh            |       128 |       5.518 |        5.085 | -7.84% |
| Sphere control         |      2048 |      26.989 |       27.099 | +0.41% |
| Convex mesh            |      2048 |      89.657 |       81.539 | -9.05% |

All windows allocate 0 bytes and exact hit snapshots match. At 2,048 triangles
the convex row still costs about 81.5 ms. This is a partial reduction of the
shared worker cost, not closure of the signal, evidence about mixed reducers, or
a game-capacity guarantee. Sphere contrasts change direction across orders at
128 triangles and remain small; no sphere-path improvement is claimed. See the
closed #026 record for source identities and comparison limits.

**Next isolation:** Capture the existing collider/triangle candidate counters
and profile ordinary query work separately from finite initial-contact recovery.
Compare filled coplanar controls, genuine rims and false-positive candidates
before proposing pruning or arithmetic changes. Preserve earliest distance,
authored tie order, finite witnesses and zero warm allocation. Check the current
math throughput owners directly before assigning the cost to an index or
reopening the already-closed upstream signal.

### GRV-Benchmark-030 — Pure 2D Bodyless Static Population Refresh Cost

**Discovered:** 2026-10-19 during static-continuity Phase 6 density controls.
**Status:** Confirmed on the frozen pre-Phase-6 stack; not an introduced
continuity regression. Detailed cost attribution remains open. **Owner:**
Gravitas pure 2D bodyless service refresh; involve GridForge or math owners only
after profiling their executed contribution.

One frictionless circle runs over one unchanged 16-unit floor at 16 Hz. Add 63
remote bodyless two-unit static pieces, all loaded and outside its local contact
neighborhood. Both counts use the same loaded world bounds, manual grounding,
drive, 32 settling frames and 64 full host steps per invocation. No surface
membership is declared in this control. Windows x64/.NET 8.0.29 Release
measurements, 24 warmed windows with zero allocation:

| Stack              | 1 static piece | 64 static pieces |
| ------------------ | -------------: | ---------------: |
| Frozen pre-Phase-6 | 21.57 us/frame |   77.09 us/frame |
| Phase 6 final      | 22.13 us/frame |   79.92 us/frame |

Physical position/velocity/rotation witnesses match all four rows. This is a
population-scaling signal, not a universal throughput or frame-tail claim. The
original `PrepareCollisionPartitions` bodyless refresh loop and
`LSCollider2D.Simulate` -> `Rebuild` path are unchanged by Phase 6.
Loaded-module hashes identify the frozen pre-phase Gravitas payload
(`B500F130...`), matching source `071b25ac2b758f6c1992a769e0713ed27b2c34dc`.

Ignored `artifacts/static-surface/phase6/population/` holds the unchanged public
consumer fixture; `population-{baseline,final}.json` contains the windows and
module hashes. Capture command after building that consumer and copying it
beside the appropriate frozen dependencies:

```powershell
$env:UseLocalLsfStack = 'true'
dotnet artifacts/static-surface/phase6/host/bin/Release/net8.0/Host.dll single artifacts/static-surface/phase6/population-baseline artifacts/static-surface/phase6/population-baseline.json FullStepPopulation1,FullStepPopulation64
```

**Next isolation:** Promote the bounded population fixture into maintained
benchmarks, profile refresh/rebuild separately from contact response, and
compare bodyless with explicit-static owners in pure 2D, 3D and mixed mode.
Preserve host-transform freshness, mutation/filter invalidation, deterministic
order and zero warm allocation before changing refresh policy. Do not add
another index or infer that an entire static rebuild can be skipped without that
evidence.

### GRV-Benchmark-031 — Ordinary Open-Mesh Mixed Full-Step Cost

**Discovered:** 2026-10-28 during static-continuity Phase 12 validation.
**Status:** Measured on the committed pre-phase runtime; attribution open.
**Owner:** Gravitas ordinary mixed mesh/prism contact generation and response;
involve FixedMathSharp after profiling the executed contribution.

`JoinedStaticSolidMixedSimulationBenchmarks`, `Shape=OpenMesh`,
`Layout=Continuous`, costs about **4.7 ms per complete host step at 0 B/frame**
on Windows x64/i7-9700K, .NET 10 Release with `UseLocalLsfStack=true`. The
frozen Gravitas `5e60c61` and Phase 12 candidate use the same benchmark driver
and FixedMathSharp `8fd9ea7`; both preserve the same position, velocity and
rotation witnesses before/after 24 alternating paired timing windows. This
establishes a pre-existing workload cost, not an introduced joined-surface
regression or a scene-capacity measurement.

Normal/reversed paired means are **4.682/4.736 ms** before/after, and a fresh
process gives **4.717/4.754 ms**. An identical-baseline A/A control measures
**4.721/4.782 ms (+1.29%)**, comparable to the 0.74%/1.60%/0.79% contrasts. This
exposes load-context/code-placement uncertainty; it proves neither new runtime
overhead nor exact zero overhead. The joined selection branch is not executed in
this row, and review found no additional ordinary collision work.

The workload is one mass-one, frictionless, open two-triangle X/Z quad with
local coordinates `X/Z=+-0.5`, concave mode and surface-approximation inertia.
It starts at `(-2,0,0.5)` against one bodyless rectangle `X=[-8,8], Z=[-2,0]`,
mixed half-thickness `2`, without continuity membership. Use Mixed mode at 16
Hz, zero gravity/air/damping/minimum speed, frozen Y and rotation, manual
grounding and sleep disabled. Settle 32 full host frames, then advance 64 frames
per invocation: X velocity alternates `+1/-1` at `X=+-2`, with inward Z impulse
`-1/16` before every `Simulate`/`LateSimulate` step. Initialization and settling
are outside the warmed window.

The maintained fixture includes the ordinary row and native joined controls:

```powershell
$env:UseLocalLsfStack = 'true'
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -p:UseLocalLsfStack=true -m:1
dotnet tests/Gravitas.Benchmarks/bin/Release/net10.0/Gravitas.Benchmarks.dll --filter '*JoinedStaticSolidMixedSimulationBenchmarks*' --job short --warmupCount 3 --iterationCount 5
```

**Next isolation:** Profile the ordinary `AppendMeshEmbedded2D` path, separating
native patch selection, finite witnesses and response. Check whether existing
trusted convex-patch inventory removes repeated triangle work without changing
finite suppliers, nonconvex holes, exposed edges, per-member query/event
identity or deterministic ordering. Check 3D parity through its existing owner;
do not add a second contact solver or infer identical response from timing.
Supplemental paired windows and loaded-module hashes are retained in ignored
`artifacts/static-surface/phase12/ordinary-openmesh*.json`; the inputs above and
maintained fixture are sufficient to reproduce the workload.

## Experimental Signals

| Signal                                                                                                                                     | Status                                                | Revisit When                                                                                                                                    |
| ------------------------------------------------------------------------------------------------------------------------------------------ | ----------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| [GRV-Benchmark-024 — Curved-contact workload frequency and step cost](#grv-benchmark-024--curved-contact-workload-frequency-and-step-cost) | Experimental; scene frequency and capacity unmeasured | Representative deterministic scenes or host traces can establish whether exact curved-feature work materially consumes the intended step budget |
| GRV-Benchmark-014 — Exact triangle-pair contacts regress dense concave-mesh throughput                                                     | Capacity-sensitive; local optimization exhausted      | A topology or exact classifier design can reduce complete triangle-pair SAT evaluations without a competing answer path                         |

### GRV-Benchmark-024 — Curved-Contact Workload Frequency And Step Cost

**Discovered:** 2026-10-06, during the workload-budget follow-up to #020.
**Source:** closed
[GRV-Benchmark-018](#grv-benchmark-018--complete-trianglecylinder-contact-cost),
[GRV-Benchmark-019](#grv-benchmark-019--complete-trianglecone-contact-cost),
[GRV-Benchmark-020](#grv-benchmark-020--complete-capsulecircle-slab-contact-cost)
and
[GRV-Benchmark-021](#grv-benchmark-021--complete-capsulestadium-slab-curved-contact-cost)
timings and stage profiles, including shared triangle/cylinder controls.
**Status:** Experimental evidence gap; representative scene frequency and
complete-step capacity have not been measured. This does not reopen these
isolated-query investigations or establish a release-blocking regression.
**Owner:** Gravitas benchmark workloads and stage attribution; FixedMathSharp
only if workload evidence justifies further shared geometry optimization.

**Planned execution scope:**
[`Evolving Game Capacity And Soak`](2026-10-06-evolving-game-capacity-and-soak-plan.md)
uses one evolving strategy scene, comparable 3D controls and shared frame/memory
measurement support. It starts after complete replay stepping and a shared trace
format. The plan owns implementation sequencing; this signal retains its
measured cost context and decision gate. Neither scene capacity nor contact
frequency is established merely by creating that plan.

**Additional bounded evidence (2026-10-15):** The supported-geometry closeout's
single-sphere, flat-mesh traversal measured `162.980 / 157.305 / 170.034 us`
derived mean per frame for continuous floor / chunk interior / chunk seam, all
at `0 B/op`. Each native observation times reset plus 64 full steps and excludes
initialization, buffer priming and 32 settling frames. This is a small
scene-cost sample, not capacity or frame-tail evidence. Pre-existing seam
response differences are tracked as #133; physical outputs differ between
layouts, so these means are not an equivalent-work optimization comparison. The
[supported-geometry closeout](done/2026-10-12-supported-geometry-range-plan.md#phase-4-compatibility-performance-and-096-closeout)
retains the bounded workload and timing provenance. Use it as context for the
representative workload rather than creating a separate speculative signal.

**Additional bounded evidence (2026-10-28):** #146's corrected mixed capsule
reproducer costs **1.997 ms per warm reset plus one complete 16-Hz host step**,
at **0 B/op**, for one six-unit capsule and three unequal-height static members.
Every invocation resets to quarter-unit penetration, so this measures forced
contact reconstruction rather than a settled scene. Named maintained
`JoinedStaticRoundedMixedContactBenchmarks` capsule `ConcaveL` / `HeightStep`
fixtures additionally measured **1.440 / 0.942 ms per pose-refresh/contact
build** at `0 B/op`. The
[#146 record](done/2026-10-15-static-surface-continuity-and-heightfield-plan.md#grv-issue-146-closeout)
retains the complete reproducible geometry, original/corrected behavior,
regressions, frozen dependencies and measurement protocol. These costs extend
this existing frequency/capacity signal; they do not establish a representative
game workload or justify comparing incorrect responses as speed baselines.

The final #020 two-launch fixture costs **696.036 / 699.465 microseconds** per
complete mixed / 3D oblique query, at **0 B/op**, while ordinary
cap/side/zero-core contacts cost roughly 10–11.5 microseconds. #021 retains
roughly 0.53–1.05 ms for its distinct curved fixtures. The
[#020 refinement report](done/2026-10-05-capsule-circle-cost-refinement.md)
records the commands, confidence intervals, final-source checks and profiles.
The [#019 final refinement](done/2026-10-06-triangle-cone-final-refinement.md)
publishes about 0.93/0.67/0.63 ms for interior/oblique/touch triangle-cone rim
queries and 89 us for near-rim gap rejection, at 0 B/op. The
[#018 refinement](done/2026-10-06-triangle-cylinder-cost-refinement.md)
subsequently lowers triangle/cylinder and circle-slab oblique costs to about
0.54 ms and publishes stationary interior-rim costs of about 0.76 ms. These
exact geometry consumers belong in the same frequency and complete-step
investigation. The #020 full matrix can be reproduced after a fresh local-stack
Release build:

```powershell
$env:UseLocalLsfStack = 'true'
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll --filter '*CapsuleCircleContactBenchmarks*' --launchCount 2 --warmupCount 5 --iterationCount 15 --iterationTime 250 --affinity 3 --keepFiles --exporters json --artifacts artifacts/grv-benchmark-024/query-controls
```

These fixtures establish cost when the configuration occurs, not its prevalence
in games. Oblique and interior-rim contacts arise from relative configurations
of ordinary cones, capsules, cylinders, mesh facets or embedded slabs, rather
than separate collider types. Tilt, relative position, dimensions and contact
evolution determine the work. Counting only final rim contacts can miss
expensive feature candidates evaluated before an ordinary feature wins or the
query rejects contact. Neither rarity nor common occurrence is established by
the current evidence. Multiplying isolated query time by a hypothetical count is
illustrative, not measured world capacity.

**Next experiment:** Reuse the existing contact fixtures and world-step
benchmark support to characterize deterministic moving scenes before changing
the solver:

1. Establish the geometric envelope with reproducible pose/orientation sweeps
   around the rim, including ordinary contacts and separated near-rim
   candidates. Then run explicit upright-agent, tilted/tumbling, rim-sliding and
   dense-contact scenarios across shape dimensions and local density. Synthetic
   sweep frequency must remain separate from observed host frequency; publish
   scene assumptions. Include cone/triangle and cylinder/triangle mesh contacts,
   varying facet orientation, cone tilt and relative position as well as
   slab/capsule poses.
2. In benchmark-only support, distinguish dispatched pair queries, expensive
   root-path entries, admitted curved candidates and final winning features.
   Reuse existing hooks where sufficient. Count repeated work across frames as
   well as unique pairs; a final contact label alone is insufficient
   attribution.
3. Measure complete `Simulate` plus `LateSimulate` steps over sustained frames:
   mean, p95/p99 and maximum observed time, warmed allocations, candidate counts
   and expensive-path rate. Profile stages separately from clean timing runs;
   keep counters and wall-clock timing outside authoritative simulation state.
   Assess explicit host tick budgets with headroom for the rest of the host.
4. Retain cone/triangle and cylinder/triangle mesh cases as well as
   capsule/cylinder and mixed circle/stadium-slab cases with their distinct root
   policies. Pure 2D workloads provide a dimensional control, not an assumed
   consumer of the 3D oblique solver. Use fixed initial state/input, repeatable
   seeds, contact preflight and replay checks; verify instrumentation preserves
   answers. Use `UseLocalLsfStack=true` throughout coordinated runs.

**Decision gate:** Publish a capacity characterization or written no-change
decision if the expensive path has a small measured contribution in the target
scenes. Promote a focused optimization or workload/API plan only when
reproducible complete-step evidence shows material budget pressure. Preserve
exact contacts, stable ordering, zero warmed allocations and 100% reachable
line/branch/method coverage for any implementation changes. Do not add
approximate collision, speculative caches, public instrumentation or merged root
solvers merely because the isolated worst fixture is expensive. #018 through
#021 remain closed.

### GRV-Benchmark-014 — Exact Triangle-Pair Contacts Regress Dense Concave-Mesh Throughput

**Discovered:** 2026-08-01 **Source:** full-domain triangle-pair Phase 2
comparison against its preserved scalar mesh/mesh baseline **Status:**
Experimental capacity guidance. Shared exact-projection and depth-ranking
duplication plus the retained signed one-limb specialization recovered
substantial throughput. A final bounded pass found no further local change worth
retaining; dense dynamic concave mesh/mesh contact is not a competitive release
path.

The unchanged 64-pair Short in-process rows reported:

| Row                             | Scalar baseline |  Initial exact | Final optimized exact | Closure confirmation |
| ------------------------------- | --------------: | -------------: | --------------------: | -------------------: |
| Ordinary convex mesh/mesh       |      `5.168 ms` |     `4.921 ms` |            `4.839 ms` |           `4.900 ms` |
| Concave mesh/mesh               |     `16.120 ms` |    `98.489 ms` |           `70.553 ms` |          `70.005 ms` |
| Dense concave mesh/mesh         |    `105.139 ms` |   `570.378 ms` |          `400.501 ms` |         `397.087 ms` |
| Contact-heavy concave mesh/mesh |    `163.956 ms` |   `804.559 ms` |          `564.147 ms` |         `556.138 ms` |
| Closed dense mesh/mesh          |    `747.173 ms` | `3,641.633 ms` |        `2,532.822 ms` |       `2,519.288 ms` |

FixedMathSharp now computes each triangle's basis-axis projections once per axis
and cancels identical positive common denominators during normalized-depth
ranking. Those policy-neutral deletions recovered roughly `28-30%` of the
initial exact dense-row cost without changing axis order, contact results, or
warmed `0 B` behavior. The ordinary convex row remains comparable because it
uses the existing convex-hull relation rather than the concave triangle-pair
generator.

The remaining gap is the measured cost of invoking the complete wide
triangle/triangle relation for every BVH-admitted candidate; candidate counts
and traversal complexity did not change. That evidence led to the per-candidate
profile recorded in the 2026-08-02 follow-up below. Do not restore the deleted
scalar SAT, add a narrowed prefilter, or create a second answer path that can
disagree with the full-domain authority. Preserved artifacts are under
`artifacts/benchmarks/2026-07-31-triangle-pair-baseline`,
`artifacts/benchmarks/2026-07-31-triangle-pair-gravitas-after`, and
`artifacts/benchmarks/2026-07-31-triangle-pair-after-denominator-cancellation`.

The 2026-08-01 closure rerun used the same 64-pair Short in-process job. Its
point estimates stayed within `-1.42%` to `+1.26%` of the final optimized run,
so it confirms the retained signal without supporting another performance claim.
MemoryDiagnoser reported fixed `78 B` / `624 B` readings on the longer
in-process rows; all 72 direct warmed Gravitas allocation guards, including the
concave and dense mesh paths, measured exactly `0 B`, so the direct guards
remain the runtime allocation authority; this document does not assign a cause
to the differing in-process MemoryDiagnoser readings. The closure artifacts are
under `artifacts/benchmarks/2026-08-01-triangle-pair-closure`.

The 2026-08-02 follow-up profiled the unchanged dense row and isolated generic
wide-multiply dispatch inside exact projection as the next shared cost. Raw
`Fixed64` coordinates were widened to `Signed192` even though each operand is a
proven signed one-word factor. FixedMathSharp now owns an exact
`Signed576`-by-`long` specialization, and triangle projection calls that owner
directly without changing the result width, axis order, tie behavior, contact
anchors, or public API.

| Row                             | Refreshed baseline | Retained change | Confirmation |
| ------------------------------- | -----------------: | --------------: | -----------: |
| Ordinary convex mesh/mesh       |         `4.836 ms` |      `4.933 ms` | control only |
| Concave mesh/mesh               |        `70.351 ms` |     `60.213 ms` |  `59.761 ms` |
| Dense concave mesh/mesh         |       `405.224 ms` |    `342.682 ms` | `343.474 ms` |
| Contact-heavy concave mesh/mesh |       `556.972 ms` |    `480.926 ms` | `480.773 ms` |
| Closed dense mesh/mesh          |          `2.566 s` |       `2.170 s` |    `2.155 s` |

The direct FixedMathSharp `TrianglePairPrimary` row improved from the prior
`64.221 us` closure to `54.33 us`, or `15.4%`, with `0 B` reported. All `18`
focused Gravitas triangle/concave/allocation regressions pass, and the direct
warmed guards remain the allocation authority at `0 B`; the small, variable
BenchmarkDotNet allocation readings are not treated as runtime allocations.

Common-denominator hoisting and eager/lazy second-edge preparation were also
measured and reverted because they did not produce a repeatable end-to-end gain
on the unchanged Gravitas rows. The optimized exact rows remain approximately
`2.9-3.7x` slower than the deleted scalar baseline, so the signal remained
material after the retained work. A final experimental pass tested an exact
signed two-limb multiplication specialization and invocation-local rigid frame
preparation. The direct specialization improved only `0.6%`; frame preparation
left the affected Gravitas rows between `0.28%` and `1.04%` slower. Both changes
were reverted exactly.

Evidence now favors reducing complete exact SAT evaluations; the tested two-limb
dispatch and frame preparation were not material. Revisit only through a
separate topology or exact-classifier design; do not grow the current relation
with more local special cases. The focused plans and evidence are preserved in
[`2026-08-02-exact-triangle-pair-throughput-plan.md`](done/2026-08-02-exact-triangle-pair-throughput-plan.md)
and
[`2026-08-02-experimental-triangle-pair-throughput-plan.md`](done/2026-08-02-experimental-triangle-pair-throughput-plan.md).

## Closed Signals

Dates retain the recorded closure, which may precede the commit or package
release. Outcomes include verified fixes and documented no-further-change
decisions; historical validation is not a fresh run or release claim.

Detailed reproducers, measurements, revisions, commands and reopening criteria
are preserved in the [archived records](done/2026-10-31-benchmark-signal-closeouts.md).
Existing signal anchors remain on the rows below.

| Signal | Closed | Outcome | Evidence |
| --- | --- | --- | --- |
| <a id="grv-benchmark-027--nonconvex-rotational-ccd-step-cost"></a>GRV-Benchmark-027 — Nonconvex rotational CCD step cost | 2026-10-31 | Deferred exact contact materialization cuts measured cone steps by 59–64% and the pressed-capsule episode by 65%, at zero warm allocation; conservative progress limits remain documented. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-027--nonconvex-rotational-ccd-step-cost) |
| <a id="grv-benchmark-032--first-isolated-2d-pair-owner-deactivation-allocation"></a>GRV-Benchmark-032 — First isolated 2D pair-owner deactivation allocation | 2026-10-30 | Removed first-lookup boxing in SwiftCollections; cold collection queries and unchanged 2D teardown allocate zero bytes. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-032--first-isolated-2d-pair-owner-deactivation-allocation) |
| <a id="grv-benchmark-026--initial-generic-convex-mesh-contact-resolution-cost"></a>GRV-Benchmark-026 — Initial generic convex mesh contact cost | 2026-10-30 | Deferred GJK witnesses and certified whole-patch sweeps reduce measured contact costs while retaining query tolerances and zero warm allocation. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-026--initial-generic-convex-mesh-contact-resolution-cost) |
| <a id="grv-benchmark-025--debug-ragdoll-steady-state-allocations"></a>GRV-Benchmark-025 — Debug ragdoll steady-state allocations | 2026-10-30 | Restored the warmed ragdoll guard; current runs allocate zero, while the historical 4.3 MB signal remains unattributed. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-025--debug-ragdoll-steady-state-allocations) |
| <a id="grv-benchmark-018--complete-trianglecylinder-contact-cost"></a>GRV-Benchmark-018 — Complete triangle/cylinder contact cost | 2026-10-06 | Reduced exact triangle/cylinder costs through reciprocal-chart and opposite-direction reuse; warm allocation remains zero. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-018--complete-trianglecylinder-contact-cost) |
| <a id="grv-benchmark-019--complete-trianglecone-contact-cost"></a>GRV-Benchmark-019 — Complete triangle/cone contact cost | 2026-10-06 | Reduced complete triangle/cone costs through measured exact-geometry refinements; scene-capacity evidence remains experimental. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-019--complete-trianglecone-contact-cost) |
| <a id="grv-benchmark-020--complete-capsulecircle-slab-contact-cost"></a><a id="capsule-rim-cost-relationship-grv-benchmark-020-and-021"></a>GRV-Benchmark-020 — Complete capsule/circle-slab contact cost | 2026-10-05 | Reduced exact capsule/circle-slab costs through shared root and output refinements; warm allocation remains zero. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-020--complete-capsulecircle-slab-contact-cost) |
| <a id="grv-benchmark-021--complete-capsulestadium-slab-curved-contact-cost"></a>GRV-Benchmark-021 — Complete capsule/stadium-slab curved contact cost | 2026-10-05 | Reduced exact capsule/stadium-slab costs with root isolation and certified-sign refinements; warm allocation remains zero. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-021--complete-capsulestadium-slab-curved-contact-cost) |
| <a id="grv-benchmark-023--circle-workload-partition-and-grounding-scaling"></a>GRV-Benchmark-023 — Circle workload partition and grounding scaling | 2026-10-04 | Retained partition, grounding and 2D gathering improvements; published scaling costs support the no-further-change decision. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-023--circle-workload-partition-and-grounding-scaling) |
| <a id="grv-benchmark-022--exact-pure-2d-circle-contact-cost"></a>GRV-Benchmark-022 — Exact pure-2D circle contact cost | 2026-10-02 | Retained the exact 2D circle contact owner after shared normalization improvements; partition and grounding scaling has a separate closeout. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-022--exact-pure-2d-circle-contact-cost) |
| <a id="grv-benchmark-012--mixed-discrete-broad-phase-allocation-at-32-pairs"></a>GRV-Benchmark-012 — Mixed discrete broad-phase allocation at 32 pairs | 2026-08-04 | Corrected benchmark lifecycle and layout; independent rotational and broad-phase controls reproduce zero warm allocation. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-012--mixed-discrete-broad-phase-allocation-at-32-pairs) |
| <a id="grv-benchmark-017--mixed-public-sweep-traversal-on-extreme-sparse-grid-spans"></a>GRV-Benchmark-017 — Mixed public sweep traversal on extreme sparse-grid spans | 2026-08-04 | Adopted GridForge's two-tier active-grid index, removing extreme sparse-span traversal cost while retaining deterministic query order. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-017--mixed-public-sweep-traversal-on-extreme-sparse-grid-spans) |
| <a id="grv-benchmark-013--mesh-scale-rebuild-allocation"></a>GRV-Benchmark-013 — Mesh scale rebuild allocation | 2026-08-03 | Builds convex support topology once and refits mesh scaling transactionally; measured subdivision allocation falls to zero. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-013--mesh-scale-rebuild-allocation) |
| <a id="grv-benchmark-015--exact-3d-contact-response-ordinary-throughput"></a>GRV-Benchmark-015 — Exact 3D contact-response ordinary throughput | 2026-08-03 | Improved exact 3D contact-response throughput with aligned-frame anchors, retaining zero warm allocation. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-015--exact-3d-contact-response-ordinary-throughput) |
| <a id="grv-benchmark-016--exact-canonical-obb-ordinary-throughput"></a>GRV-Benchmark-016 — Exact canonical OBB ordinary throughput | 2026-08-03 | Improved canonical OBB throughput by reusing one exact relative-frame kernel per relation. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-016--exact-canonical-obb-ordinary-throughput) |
| <a id="grv-benchmark-011--physics-material-combine-numeric-hardening"></a>GRV-Benchmark-011 — Physics-material combine numeric hardening | 2026-07-13 | Hardened material averaging and geometric-mean edge cases without a credible measured response regression. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-011--physics-material-combine-numeric-hardening) |
| <a id="grv-benchmark-010--checked-mesh-scale-and-thin-shell-cache-cost"></a>GRV-Benchmark-010 — Checked mesh scale and thin-shell cache cost | 2026-07-12 | Retained checked linear-time mesh rebuilding and cached validated shell/volume properties; repeated reads allocate zero. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-010--checked-mesh-scale-and-thin-shell-cache-cost) |
| <a id="grv-benchmark-009--replay-hash-collider-id-churn-scaling"></a>GRV-Benchmark-009 — Replay hash collider-ID churn scaling | 2026-07-05 | Replaced ID-peak replay scans with reusable collider slots, dense live traversal and canonical replay ordinals. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-009--replay-hash-collider-id-churn-scaling) |
| <a id="grv-benchmark-008--pure-2d-response-position-correction-repartition-allocation"></a>GRV-Benchmark-008 — Pure 2D response position-correction repartition allocation | 2026-06-28 | Reuses retained partitions and GridForge's inline partition storage for allocation-free response repartitioning. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-008--pure-2d-response-position-correction-repartition-allocation) |
| <a id="grv-benchmark-005--swiftcollections-sort-hot-path-allocation"></a>GRV-Benchmark-005 — SwiftCollections sort hot-path allocation | 2026-06-24 | Adopted SwiftCollections allocation-free sorting and sorted-key APIs; removed Gravitas's sorting workaround. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-005--swiftcollections-sort-hot-path-allocation) |
| <a id="grv-benchmark-006--mixed-mesh-finite-slab-triangle-scaling-signal"></a>GRV-Benchmark-006 — Mixed mesh finite-slab triangle scaling signal | 2026-06-24 | Added mesh candidate diagnostics and scaling controls; ordered lower-bound candidates improve convex-source mesh sweeps. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-006--mixed-mesh-finite-slab-triangle-scaling-signal) |
| <a id="grv-benchmark-001--pure-2d-dynamic-ccd-candidate-asymmetry"></a>GRV-Benchmark-001 — Pure 2D dynamic CCD candidate asymmetry | 2026-06-23 | Introduced planar CCD candidate indexing and avoided mixed indexing in pure 2D mode. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-001--pure-2d-dynamic-ccd-candidate-asymmetry) |
| <a id="grv-benchmark-002--3d-shape-exact-false-positive-cost"></a>GRV-Benchmark-002 — 3D shape-exact false-positive cost | 2026-06-23 | Uses exact-source sweeps before conservative sphere fallback for non-sphere static CCD movers. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-002--3d-shape-exact-false-positive-cost) |
| <a id="grv-benchmark-007--3d-dynamic-shape-exact-bdn-allocation-signal"></a>GRV-Benchmark-007 — 3D dynamic shape-exact BDN allocation signal | 2026-06-23 | Added exact-sweep bounds rejection to remove the dynamic false-positive allocation/time scaling signal. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-007--3d-dynamic-shape-exact-bdn-allocation-signal) |
| <a id="grv-benchmark-003--3d-full-runtime-ccd-allocation"></a>GRV-Benchmark-003 — 3D full-runtime CCD allocation | 2026-06-23 | Adopted GridForge's caller-owned, allocation-free line tracing for full-runtime CCD. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-003--3d-full-runtime-ccd-allocation) |
| <a id="grv-benchmark-004--grounding-raycast-probe-allocation"></a>GRV-Benchmark-004 — Grounding raycast probe allocation | 2026-06-23 | Applied the same tracing fix to eliminate automatic grounding-ray allocation. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-004--grounding-raycast-probe-allocation) |

## Watch Items

- Mixed full-runtime CCD rows were heavier than pure 2D or pure 3D rows at
  `1024` bodies. This is expected because mixed mode exercises both dimensions
  and the mixed broad phase. Revisit if the gap grows after the 3D allocation
  RCA or if mixed CCD becomes an immediate release-critical target.
- Pure 3D swept-sphere dense mesh target rows are visible through
  `MeshQuery3DTriangleScalingBenchmarks` and still scale linearly with triangle
  candidate volume: `550.1 us`, `2.145 ms`, and `8.638 ms` at `128`, `512`, and
  `2048` triangles. The lower-bound ordering optimization was adopted only for
  convex-source sweeps against concave mesh targets, where it proved a real
  bottleneck reduction. Revisit swept-sphere mesh pruning if host workloads need
  many analytic sphere casts against dense single-mesh colliders rather than
  partitioned/decomposed mesh geometry.
- Benchmark publishing, external baseline storage, CI gating, and host-visible
  CCD counters are tracked in
  [`2026-06-21-benchmark-publishing-and-ccd-diagnostics-plan.md`](2026-06-21-benchmark-publishing-and-ccd-diagnostics-plan.md).

## Promotion Criteria

Promote a signal from this backlog into a dedicated dated plan when it has:

- reproducible evidence and a suspected runtime phase.
- enough subsystem breadth that a single focused patch would be misleading.
- API, architecture, or multi-workstream design decisions.
- correctness or ordering invariants that need staged implementation.
- benchmark and allocation evidence that should move with the new plan.
