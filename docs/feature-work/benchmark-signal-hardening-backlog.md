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
  `GRV-Benchmark-039`.
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

### GRV-Benchmark-036 — First Dynamic Membership In A Planar Floor Partition

**Discovered 2026-11-10.** Low-priority allocation signal from the corrected
six-link planar ragdoll drop in
[GRV-Issue-155](2026-10-06-long-duration-physics-quality-and-tuning-plan.md#grv-issue-155-implementation).
Build the package-backed Release benchmark on `2273ea72` plus that correction
and run `verify-quality-ragdoll-duration --strict`. Each eight-second planar
drop allocates 384 B after the verifier's one-second warmup: frame 40 at
32 Hz and frame 238 at 60 Hz, consistently across repeated and observation-free
controls. The supported planar cases and all spatial ragdolls measure zero.
The prior planar drops allocated 8,096/4,896 B.

The left arm, collider 4, first enters a floor-only `PhysicsPartition2D`.
`AddDynamicObject` creates its dynamic membership set and
`SetDynamicObjectAwake` creates the awake subset, 192 B each. Partition rent,
voxel attachment, tracing and joint solving allocate zero in that frame.
This is cold role storage reached during otherwise warmed simulation.

**Next isolation:** Compare first dynamic-role entry with repeated entry in
the same partitions at representative floor coverage. Measure retained memory
and frame cost before choosing any reservation policy; eagerly allocating
dynamic sets for every static floor voxel expands cold memory. Preserve
local-membership scaling, deterministic reuse, replay and the existing zero
warm supported-contact baseline. This signal does not relax a failing
steady-state allocation guard.

### GRV-Benchmark-037 — Finite Duration Of Legacy Constraint Contact Benchmarks

**Discovered 2026-11-10.** Benchmark validity and capacity signal from the
`Constraint2DBenchmarks` / `Constraint3DBenchmarks` resting and two-ragdoll
rows during [#155 validation](2026-10-06-long-duration-physics-quality-and-tuning-plan.md#grv-issue-155-implementation).
The original ragdolls used a grid beginning at -4 while their 24-unit floors
at X offsets +/-2 extended to -14. The planar benchmark configured context
gravity but left per-body `SolidBody2D.Gravity` at zero. The current fixture
assigns planar gravity and uses grid padding 16 for these BDN ragdoll contexts;
other callers retain padding 4.

The original 64-second ABBA timings are excluded from supported-workload
claims. Their unchanged joint row counts concealed different contact work.
The spatial leg first falls outside the original grid at X approximately
-5.95, while still inside authored floor geometry. With complete grid
coverage, the same corrected two-ragdoll scene stays above the floor through
2,384 steps. The corresponding planar control with gravity keeps a single
ragdoll supported, but the two-ragdoll case first crosses a finite floor edge
on step 497, before the first below-floor center on step 837. Neither result
establishes tunneling through admitted, covered support.

These legacy ball-socket inputs also begin with strained anchors: a half-unit
head gap and approximately 0.79-unit leg gaps. They differ from the initially
coincident, limited joints in the opt-in quality scenes. Stronger correction
changes their motion and newly visited partition roles. Counting rows or
reporting a small terminal gap alone does not validate a resting workload.

**Next isolation:** Use the plan's eight-second, physically checked workloads
for Phase 3 quality/cost comparisons. Before reusing a legacy contact row for
long-duration capacity claims, record its valid contact window, floor and
grid coverage, body motion and contact population. A sustained benchmark
requires a deliberate bounded scene or documented reset policy; preserve
the original strained input as a separate control. Do not enlarge geometry
or alter anchors solely to hide a cost or escape.

### GRV-Benchmark-038 — Supported Ragdoll Full-Step Tails At 60 Hz

**Discovered 2026-11-10.** Active duration/cost signal from the corrected
six-link spatial supported-start workload at six solver passes. Build the
package-backed Release benchmark on `2273ea72` plus the
[#155 correction](2026-10-06-long-duration-physics-quality-and-tuning-plan.md#grv-issue-155-implementation),
then run `verify-quality-ragdoll-duration --strict`. Preserve the original
eight-second scenario, pulses at two/four/six seconds, 32/60 Hz controls,
replay and geometry bounds.

Disposable instrumentation timed only `Simulate` plus `LateSimulate` in the
unobserved control after the first second, with 420 samples per 60 Hz launch.
Two corrected launches retained all bounds and zero warm allocation, with a
mean full step of 2.991268 ms and mean per-launch p95 of 5.832350 ms. Their
maxima were 22.8986 and 23.3886 ms, both above the 16.67 ms frame interval.
The prior solver measured 12.038256 ms mean and 37.1377 ms worst step, so
the improvement does not establish that every supported step fits 60 Hz.
At 32 Hz the corrected p95 was 14.8978 ms and worst step 30.3322 ms.
These Windows x64 Core i7-9700K / .NET 10.0.10 results are host-specific.

**Next isolation:** In Phase 3, correlate the long steps with awake population,
contact rows, exact-response work and pulse/recovery phase. Separate sleep
propagation from per-active-step cost, preserving the original support and
replay bounds. Compare 2/6/12 passes against the host's reserved physics
budget; do not infer a portable frame-time guarantee from the mean or p95.
The existing plan retains the settings, commands, raw-capture locations and
balanced before/after measurements.

## Experimental Signals

| Signal                                                                                                                                     | Status                                                | Revisit When                                                                                                                                    |
| ------------------------------------------------------------------------------------------------------------------------------------------ | ----------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| [GRV-Benchmark-033 — 1,024-actor 3D partition cost and first-wake spike](#grv-benchmark-033) | Capacity-sensitive. Sustained cost profiled, first-wake allocation owner open. | An equivalent workload exceeds the host's reserved budget or the first-use burst recurs in a target scene |
| [GRV-Benchmark-024 — Curved-contact workload frequency and step cost](#grv-benchmark-024--curved-contact-workload-frequency-and-step-cost) | Experimental. Synthetic step costs measured, representative frequency open. | Representative deterministic scenes or host traces can establish whether exact curved-feature work materially consumes the intended step budget |
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
**Status:** Experimental evidence gap. Synthetic complete-step envelopes
are measured. Representative scene frequency and its contribution to a host
budget remain open.
**Owner:** Gravitas benchmark workloads and stage attribution; FixedMathSharp
only if workload evidence justifies further shared geometry optimization.

**Phase 2 evidence (2026-11-08):** The 1,024-actor 3D strategy reference
performed zero work in the instrumented curved paths. The opt-in single-pair
sweeps measured dispatch, isolation, admission, and selection. The oblique
stadium gap isolated roots on all 128 sampled frames with zero contacts.
Revisit with a representative scene or trace that measures local density,
pose frequency and budget contribution.
[Evolving Game Capacity And Soak](done/2026-10-06-evolving-game-capacity-and-soak-plan.md#synthetic-curved-contact-step-envelopes)

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

**Next experiment:** Start with a representative deterministic scene or host
trace whose moving shapes enter these paths. Reuse the verified
`verify-curved-workload` sweeps as geometry controls. Measure distinct
upright and tilted or rim-contact populations at known local densities.
In an untimed replay count unique pairs and repeat dispatches, root-path
entries, admitted candidates, and winners or rejections. Compare the trace
with clean complete-step tails and the host's reserved budget. Preserve the
synthetic sweep results as a separate measure of cost when a pose occurs.

**Decision gate:** Publish a capacity characterization or written no-change
decision if the expensive path has a small measured contribution in the target
scenes. Promote a focused optimization or workload/API plan only when
reproducible complete-step evidence shows material budget pressure. Preserve
exact contacts, stable ordering, zero warmed allocations and 100% reachable
line/branch/method coverage for any implementation changes. Do not add
approximate collision, speculative caches, public instrumentation or merged root
solvers merely because the isolated worst fixture is expensive. #018 through
#021 remain closed.

<a id="grv-benchmark-033"></a>

### GRV-Benchmark-033 — 1,024-Actor 3D Partition Cost And First-Wake Spike

**Discovered 2026-11-08.** Capacity-sensitive experimental signal from the
1,024-actor 3D strategy scene on Windows x64, Core i7-9700K and .NET 10
Release with released LSF packages. Gravitas owns partition preparation and
corrected-pose publication. The GridForge typed voxel lookup is a measured
callee. The first-wake allocation owner remains open.

Build the package-backed Release benchmark and set
`GRAVITAS_STRATEGY_FRAME_CSV=1`. Run `verify-strategy-3d 1024 8` and
`verify-strategy-3d 1024 16` in alternating fresh processes, then run
`soak-strategy-3d 1024 8` and `soak-strategy-3d 1024 16` as warmed controls.
Keep Y at 4 and verify each setting's replay, floor support, projectile
stops and connected chains.

- **Crowded steps.** At X/Z 8, partition preparation and corrected-pose
  publication accounted for about half of sampled crowded CPU time in two
  profiles. Over frames 66–96, their combined sampled CPU weight fell from
  239–253 ms at X/Z 8 to 113–130 ms at X/Z 16, while collision distribution
  and touching-pair work increased. Separate counter runs recorded zero new
  monitor contentions. The warmed crowded scenario p99 was 15.96 ms at 8
  and 12.66 ms at 16.
- **First wake.** All six fresh-context frame-65 bursts at X/Z 16 exceeded
  16.67 ms, taking 21.23–34.68 ms and allocating about 5.48 MB in physics.
  X/Z 8 took 10.82–11.68 ms and allocated about 2.02 MB. The warmed control
  fit all 160 measured wakes per setting, with median physics allocations
  of 0 bytes at 8 and 2,400 bytes at 16. The sampled first-wake window had
  too few observations to attribute the allocation spike.

**Next isolation:** If a target scene needs more headroom, count covered
voxels, typed lookups, actual membership changes, and pair visits during
the sustained crowded window. Separately capture allocation stacks for
frame 65 across repeated fresh contexts, using the warmed cycle as a
control. Replay and physical outcomes must agree before comparing a change.
Use the affected host's budget to decide whether either result warrants
runtime work.
[Overrun attribution](done/2026-10-06-evolving-game-capacity-and-soak-plan.md#1024-actor-3d-overrun-attribution),
[cell-size comparison](done/2026-10-06-evolving-game-capacity-and-soak-plan.md#matched-1024-actor-3d-xz-cell-sizes),
[soak results](done/2026-10-06-evolving-game-capacity-and-soak-plan.md#warmed-frame-tails)

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
are preserved in the
[archived records](done/2026-10-31-benchmark-signal-closeouts.md). Existing
signal anchors remain on the rows below.

| Signal                                                                                                                                                                                                    | Closed     | Outcome                                                                                                                                                                                    | Evidence                                                                                                                                 |
| --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="grv-benchmark-035"></a>GRV-Benchmark-035 — Warm pair growth in rotated cuboid stacks | 2026-11-09 | Rejects all 20 disjoint acquisitions in the mixed-load stack capture and cuts warm allocation from 66,032 B to 7,752 B. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-035) |
| <a id="grv-benchmark-034"></a><a id="grv-benchmark-034--rotated-cuboid-support-step-cost"></a>GRV-Benchmark-034 — Rotated cuboid support step cost | 2026-11-09 | Shared-frame exact levers reduce step cost with matching replay and allocations. The dense stress case remains above the 60 Hz interval. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-034) |
| <a id="grv-benchmark-031--ordinary-open-mesh-mixed-full-step-cost"></a>GRV-Benchmark-031 — Ordinary open-mesh mixed full step                                                                             | 2026-11-01 | Certified patch/prism contacts reduce the ordinary step from 4.52 ms to 115 us at zero warm allocation; exit/group selection and small-control limits retained.                            | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-031--ordinary-open-mesh-mixed-full-step-cost)                     |
| <a id="grv-benchmark-030--pure-2d-bodyless-static-population-refresh-cost"></a>GRV-Benchmark-030 — Pure 2D static population refresh                                                                      | 2026-11-01 | Exact unit-basis sampling cuts 64-static 2D full-step time by 46–52% at zero warm allocation; transform freshness and small-control limits retained.                                       | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-030--pure-2d-bodyless-static-population-refresh-cost)             |
| <a id="grv-benchmark-029--dense-mesh-sweep-cost-after-exact-contact-hardening"></a>GRV-Benchmark-029 — Dense mesh sweep cost                                                                              | 2026-10-31 | Certified patches/carriers and duplicate reuse cut query time by 96.6–99.9% at 2,048 triangles, with zero warm allocation; finite geometry and small-control limits retained.              | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-029--dense-mesh-sweep-cost-after-exact-contact-hardening)         |
| <a id="grv-benchmark-028--mixed-broad-phase-work-with-an-empty-opposite-dimension"></a>GRV-Benchmark-028 — Mixed broad phase with an empty opposite dimension                                             | 2026-10-31 | Skipped impossible distribution while preserving index refresh and cleanup; measured empty-population steps improve 30–42% at zero warm allocation.                                        | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-028--mixed-broad-phase-work-with-an-empty-opposite-dimension)     |
| <a id="grv-benchmark-027--nonconvex-rotational-ccd-step-cost"></a>GRV-Benchmark-027 — Nonconvex rotational CCD step cost                                                                                  | 2026-10-31 | Deferred exact contact materialization cuts measured cone steps by 59–64% and the pressed-capsule episode by 65%, at zero warm allocation; conservative progress limits remain documented. | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-027--nonconvex-rotational-ccd-step-cost)                          |
| <a id="grv-benchmark-032--first-isolated-2d-pair-owner-deactivation-allocation"></a>GRV-Benchmark-032 — First isolated 2D pair-owner deactivation allocation                                              | 2026-10-30 | Removed first-lookup boxing in SwiftCollections; cold collection queries and unchanged 2D teardown allocate zero bytes.                                                                    | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-032--first-isolated-2d-pair-owner-deactivation-allocation)        |
| <a id="grv-benchmark-026--initial-generic-convex-mesh-contact-resolution-cost"></a>GRV-Benchmark-026 — Initial generic convex mesh contact cost                                                           | 2026-10-30 | Deferred GJK witnesses and certified whole-patch sweeps reduce measured contact costs while retaining query tolerances and zero warm allocation.                                           | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-026--initial-generic-convex-mesh-contact-resolution-cost)         |
| <a id="grv-benchmark-025--debug-ragdoll-steady-state-allocations"></a>GRV-Benchmark-025 — Debug ragdoll steady-state allocations                                                                          | 2026-10-30 | Restored the warmed ragdoll guard; current runs allocate zero, while the historical 4.3 MB signal remains unattributed.                                                                    | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-025--debug-ragdoll-steady-state-allocations)                      |
| <a id="grv-benchmark-018--complete-trianglecylinder-contact-cost"></a>GRV-Benchmark-018 — Complete triangle/cylinder contact cost                                                                         | 2026-10-06 | Reduced exact triangle/cylinder costs through reciprocal-chart and opposite-direction reuse; warm allocation remains zero.                                                                 | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-018--complete-trianglecylinder-contact-cost)                      |
| <a id="grv-benchmark-019--complete-trianglecone-contact-cost"></a>GRV-Benchmark-019 — Complete triangle/cone contact cost                                                                                 | 2026-10-06 | Reduced complete triangle/cone costs through measured exact-geometry refinements; scene-capacity evidence remains experimental.                                                            | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-019--complete-trianglecone-contact-cost)                          |
| <a id="grv-benchmark-020--complete-capsulecircle-slab-contact-cost"></a><a id="capsule-rim-cost-relationship-grv-benchmark-020-and-021"></a>GRV-Benchmark-020 — Complete capsule/circle-slab contact cost | 2026-10-05 | Reduced exact capsule/circle-slab costs through shared root and output refinements; warm allocation remains zero.                                                                          | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-020--complete-capsulecircle-slab-contact-cost)                    |
| <a id="grv-benchmark-021--complete-capsulestadium-slab-curved-contact-cost"></a>GRV-Benchmark-021 — Complete capsule/stadium-slab curved contact cost                                                     | 2026-10-05 | Reduced exact capsule/stadium-slab costs with root isolation and certified-sign refinements; warm allocation remains zero.                                                                 | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-021--complete-capsulestadium-slab-curved-contact-cost)            |
| <a id="grv-benchmark-023--circle-workload-partition-and-grounding-scaling"></a>GRV-Benchmark-023 — Circle workload partition and grounding scaling                                                        | 2026-10-04 | Retained partition, grounding and 2D gathering improvements; published scaling costs support the no-further-change decision.                                                               | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-023--circle-workload-partition-and-grounding-scaling)             |
| <a id="grv-benchmark-022--exact-pure-2d-circle-contact-cost"></a>GRV-Benchmark-022 — Exact pure-2D circle contact cost                                                                                    | 2026-10-02 | Retained the exact 2D circle contact owner after shared normalization improvements; partition and grounding scaling has a separate closeout.                                               | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-022--exact-pure-2d-circle-contact-cost)                           |
| <a id="grv-benchmark-012--mixed-discrete-broad-phase-allocation-at-32-pairs"></a>GRV-Benchmark-012 — Mixed discrete broad-phase allocation at 32 pairs                                                    | 2026-08-04 | Corrected benchmark lifecycle and layout; independent rotational and broad-phase controls reproduce zero warm allocation.                                                                  | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-012--mixed-discrete-broad-phase-allocation-at-32-pairs)           |
| <a id="grv-benchmark-017--mixed-public-sweep-traversal-on-extreme-sparse-grid-spans"></a>GRV-Benchmark-017 — Mixed public sweep traversal on extreme sparse-grid spans                                    | 2026-08-04 | Adopted GridForge's two-tier active-grid index, removing extreme sparse-span traversal cost while retaining deterministic query order.                                                     | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-017--mixed-public-sweep-traversal-on-extreme-sparse-grid-spans)   |
| <a id="grv-benchmark-013--mesh-scale-rebuild-allocation"></a>GRV-Benchmark-013 — Mesh scale rebuild allocation                                                                                            | 2026-08-03 | Builds convex support topology once and refits mesh scaling transactionally; measured subdivision allocation falls to zero.                                                                | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-013--mesh-scale-rebuild-allocation)                               |
| <a id="grv-benchmark-015--exact-3d-contact-response-ordinary-throughput"></a>GRV-Benchmark-015 — Exact 3D contact-response ordinary throughput                                                            | 2026-08-03 | Improved exact 3D contact-response throughput with aligned-frame anchors, retaining zero warm allocation.                                                                                  | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-015--exact-3d-contact-response-ordinary-throughput)               |
| <a id="grv-benchmark-016--exact-canonical-obb-ordinary-throughput"></a>GRV-Benchmark-016 — Exact canonical OBB ordinary throughput                                                                        | 2026-08-03 | Improved canonical OBB throughput by reusing one exact relative-frame kernel per relation.                                                                                                 | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-016--exact-canonical-obb-ordinary-throughput)                     |
| <a id="grv-benchmark-011--physics-material-combine-numeric-hardening"></a>GRV-Benchmark-011 — Physics-material combine numeric hardening                                                                  | 2026-07-13 | Hardened material averaging and geometric-mean edge cases without a credible measured response regression.                                                                                 | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-011--physics-material-combine-numeric-hardening)                  |
| <a id="grv-benchmark-010--checked-mesh-scale-and-thin-shell-cache-cost"></a>GRV-Benchmark-010 — Checked mesh scale and thin-shell cache cost                                                              | 2026-07-12 | Retained checked linear-time mesh rebuilding and cached validated shell/volume properties; repeated reads allocate zero.                                                                   | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-010--checked-mesh-scale-and-thin-shell-cache-cost)                |
| <a id="grv-benchmark-009--replay-hash-collider-id-churn-scaling"></a>GRV-Benchmark-009 — Replay hash collider-ID churn scaling                                                                            | 2026-07-05 | Replaced ID-peak replay scans with reusable collider slots, dense live traversal and canonical replay ordinals.                                                                            | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-009--replay-hash-collider-id-churn-scaling)                       |
| <a id="grv-benchmark-008--pure-2d-response-position-correction-repartition-allocation"></a>GRV-Benchmark-008 — Pure 2D response position-correction repartition allocation                                | 2026-06-28 | Reuses retained partitions and GridForge's inline partition storage for allocation-free response repartitioning.                                                                           | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-008--pure-2d-response-position-correction-repartition-allocation) |
| <a id="grv-benchmark-005--swiftcollections-sort-hot-path-allocation"></a>GRV-Benchmark-005 — SwiftCollections sort hot-path allocation                                                                    | 2026-06-24 | Adopted SwiftCollections allocation-free sorting and sorted-key APIs; removed Gravitas's sorting workaround.                                                                               | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-005--swiftcollections-sort-hot-path-allocation)                   |
| <a id="grv-benchmark-006--mixed-mesh-finite-slab-triangle-scaling-signal"></a>GRV-Benchmark-006 — Mixed mesh finite-slab triangle scaling signal                                                          | 2026-06-24 | Added mesh candidate diagnostics and scaling controls; ordered lower-bound candidates improve convex-source mesh sweeps.                                                                   | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-006--mixed-mesh-finite-slab-triangle-scaling-signal)              |
| <a id="grv-benchmark-001--pure-2d-dynamic-ccd-candidate-asymmetry"></a>GRV-Benchmark-001 — Pure 2D dynamic CCD candidate asymmetry                                                                        | 2026-06-23 | Introduced planar CCD candidate indexing and avoided mixed indexing in pure 2D mode.                                                                                                       | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-001--pure-2d-dynamic-ccd-candidate-asymmetry)                     |
| <a id="grv-benchmark-002--3d-shape-exact-false-positive-cost"></a>GRV-Benchmark-002 — 3D shape-exact false-positive cost                                                                                  | 2026-06-23 | Uses exact-source sweeps before conservative sphere fallback for non-sphere static CCD movers.                                                                                             | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-002--3d-shape-exact-false-positive-cost)                          |
| <a id="grv-benchmark-007--3d-dynamic-shape-exact-bdn-allocation-signal"></a>GRV-Benchmark-007 — 3D dynamic shape-exact BDN allocation signal                                                              | 2026-06-23 | Added exact-sweep bounds rejection to remove the dynamic false-positive allocation/time scaling signal.                                                                                    | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-007--3d-dynamic-shape-exact-bdn-allocation-signal)                |
| <a id="grv-benchmark-003--3d-full-runtime-ccd-allocation"></a>GRV-Benchmark-003 — 3D full-runtime CCD allocation                                                                                          | 2026-06-23 | Adopted GridForge's caller-owned, allocation-free line tracing for full-runtime CCD.                                                                                                       | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-003--3d-full-runtime-ccd-allocation)                              |
| <a id="grv-benchmark-004--grounding-raycast-probe-allocation"></a>GRV-Benchmark-004 — Grounding raycast probe allocation                                                                                  | 2026-06-23 | Applied the same tracing fix to eliminate automatic grounding-ray allocation.                                                                                                              | [Closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-004--grounding-raycast-probe-allocation)                          |

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
