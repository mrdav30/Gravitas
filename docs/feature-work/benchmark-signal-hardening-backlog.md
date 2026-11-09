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
  `GRV-Benchmark-036`.
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

<a id="grv-benchmark-034"></a>

### GRV-Benchmark-034 — Rotated Cuboid Support Step Cost

**Discovered** 2026-11-09 during the GRV-Issue-151 verification.
**Owner** Gravitas 3D cuboid contact generation and discrete island response.
**Status** Active. Exact response cost is lower at stack density. The
16-stack heavy-bottom workload still exceeds the 60 Hz interval.

Use the `verify-quality-stack-support` fixture in Release at 60 Hz and six
iterations. For timing, disable sleep in both variants, warm 64 steps, and
measure `Simulate` plus `LateSimulate` on frames 65–256. Run the fixture
against stock `4b0d83af` and the issue fix with released packages. Three
separate launches per variant produced medians of 1.412 → 1.533 ms for
offset 100:1 and 0.466 → 0.955 ms for centered 100:1. The heavy-bottom
centered control measured 0.413 → 1.995 ms despite staying on sequential
response. All measured cases had active supports throughout the warm
window and 0 B of per-thread step allocation. The stock and corrected
scenes exercise different trajectories. In the separate replay capture,
the two support interfaces averaged two fresh rows per warm frame on
stock and 7.16–8.00 on the candidate, depending on the case.
[GRV-Issue-151 evidence](2026-10-06-long-duration-physics-quality-and-tuning-plan.md#grv-issue-151-implementation)

In the 2026-11-09 attribution capture, disposable timing scopes recorded
inclusive elapsed time inside the same eight warm workloads. Scope
`DoCuboidsCheck` and its `TryBuildCuboidFaceContacts` child, the island's
`TrySeedSupportedIslandNormals`, and the per-iteration
`CollisionResponse.CalculateImpulse` and its exact-response children.
Count compact failures by stage before the normal fallback. The
percentages describe the instrumented runs. Use uninstrumented launches
for throughput comparisons.

| Stage | Share of warm step time | Observation |
| --- | ---: | --- |
| Cuboid detection | 5–13% | Finite face clipping accounted for 3–7% of the step. |
| Contact-row response | 66–89% | Includes contact preparation, warm starts, normal and friction work. |
| Coupled normal seed | 6% in both top-heavy stacks | The mass-swapped and equal-mass controls stayed sequential. |
| Exact normal fallback | 34% offset, 49% centered in the heavy-bottom controls | 3,264 and 5,568 fallback calls out of 9,216 normal rows, respectively. All followed compact angular-delta rejection. |
| Exact friction velocity deltas | 15% offset, 14% centered in the top-heavy stacks | 48 and 24 calls, respectively. The radical rounding search was expensive even at this frequency. |

Keep the finite witnesses, solver order and iteration count. The first
optimization bounds exact Coulomb rounding by the magnitudes of its
terms, skips incoming point velocity when restitution is zero, and skips
normal angular delta products for zero inverse inertia.

Reproduce the equivalent-work timing with
`measure-quality-stack-support` in the benchmark launcher. Copy the
benchmark-only CLI and fixture changes onto `898dd811` for the corrected
baseline. Both builds use the same released packages. The command disables
sleep, checks both interfaces on each measured frame, and emits warm
allocation counts, fresh support rows and the terminal replay hash. Use
`verify-quality-stack-support` separately to compare per-frame replay,
manifold rows and geometric support with normal sleep settings.

On Windows x64 with .NET SDK 10.0.302 and runtime 10.0.10, five independent
Release launches per revision ran in `A B B A A B B A A B` order. Times
are medians of launch means in milliseconds. Both support interfaces were
active in all 192 measured steps per case. Fresh row totals and sleep-disabled
terminal hashes matched in every launch, with 0 B of managed allocation
on the stepping thread.

| Lower / upper mass | Alignment | Baseline ms | Candidate ms | Change | Fresh support rows |
| --- | --- | ---: | ---: | ---: | ---: |
| 1 / 100 | Offset | 1.607 | 1.391 | −13.4% | 1,418 |
| 1 / 100 | Centered | 0.963 | 0.879 | −8.7% | 1,376 |
| 100 / 1 | Offset | 1.677 | 1.733 | +3.3% | 1,536 |
| 100 / 1 | Centered | 1.988 | 1.933 | −2.8% | 1,536 |
| 100 / 100 | Offset | 0.908 | 0.949 | +4.5% | 1,522 |
| 100 / 100 | Centered | 0.812 | 0.816 | +0.5% | 1,527 |
| 1 / 1 | Offset | 0.888 | 0.887 | −0.2% | 1,524 |
| 1 / 1 | Centered | 0.869 | 0.849 | −2.2% | 1,531 |

The top-heavy timing ranges separated across the five launches in each
variant. The heavy-bottom controls and the offset heavy equal-mass
control had overlapping launch ranges. Both heavy-bottom stacks remain
expensive. The no-sleep top-heavy terminal hashes were
`22a417806fe362272fc563376f09243b` offset and
`9d4805f5d6ddde5238ada6a466fc7519` centered. In Release and ReleaseLean,
the support and quality frame CSVs were byte-identical between builds.
ReleaseLean also matched the warm support counts and terminal hashes
with 0 B allocated. In both configurations, the 1,024-actor 3D strategy
verifier matched its 96-frame trace `d5e9d21184fbb529b3b3b83e86f2b6b1`,
with chains completing at frame 67. The instrumented strategy verifier
recorded 109,416 normal rows across its replay passes, with zero calls
to rotated cuboid clipping, exact normal fallback or exact friction.
That fixture does not exercise the cost identified by this signal.

The full Release and ReleaseLean suites passed 8,399 and 8,328 cases,
respectively. One additional exact restitution regression passed in each
configuration. The combined raw coverage in each configuration reached
54,019 lines, 22,086 branches and 5,634 methods with zero uncovered points.

Endpoint-level counts in the centered heavy-bottom control attributed all
5,568 failures to the inverse-inertia transform. The offset control had
2,880 rejected transforms and 768 rejected torque crosses, with some
rows rejecting both endpoints.

#### Sustained support at scene density

Run `measure-quality-stack-density` for complete physics step timing and
`measure-quality-stack-density verify` for a per-frame replay digest that
includes solver caches. Use the same benchmark-only changes with
`dff545e8` as the runtime baseline. This synthetic, support-heavy fixture
places four or sixteen independent two-body stacks on a shared 16×16-unit
floor. The four-stack lattice has 8-unit spacing and the sixteen-stack
lattice has 4-unit spacing. The heavy-bottom cases alternate centered and
offset 100 / 1 mass loads. The second sixteen-stack case starts at 15°
yaw and repeats 100 / 1 centered, 100 / 1 offset, 1 / 100 centered, and
1 / 1 offset loads. All cases run in pure 3D at 60 Hz with six iterations
and sleep disabled, warming frames 1–64 and measuring frames 65–256.

The baseline stage capture attributed 41% of the dense heavy-bottom step
to exact normal fallback, with 84,807 fallback calls across 147,432
normal rows. The mixed-load case spent 28% of the step in 32,636
fallbacks across 142,296 normal rows. Clipping accounted for 2–4% of these
steps. Both scenes sustained all 32 rotated support interfaces throughout
the timing window.

The solver now reuses common products across the exact normal velocity
components, groups terms that share an effective-mass factor, and uses an
exact zero lever for bodyless endpoints. The 3D island constraint sort
compares the byte-backed kind directly to avoid enum boxing. These
changes preserve contact admission and nearest-even rounding.
In the follow-up stage probe, exact normal fallback still ran 84,807
times in the dense heavy-bottom case. Its share of instrumented step
time fell from 40.9% to 36.3%, with the largest reductions inside
effective-mass and linear-delta calculation.

Three Windows x64 Release launches per revision ran in
`A B B A A B` order on .NET SDK 10.0.302 and runtime 10.0.10 with
released dependencies. The table reports medians of launch percentiles
and launch overrun counts against the 16.67 ms interval.

| Workload | Step p50, ms | Step p95, ms | Overruns / 192 | Fresh rows in window |
| --- | ---: | ---: | ---: | ---: |
| 4 heavy-bottom stacks | 7.154 → 6.632 | 8.386 → 7.831 | 0 → 0 | 6,134 |
| 16 heavy-bottom stacks | 32.346 → 29.975 | 36.640 → 33.955 | 192 → 192 | 24,572 |
| 16 mixed-load stacks | 17.473 → 16.378 | 21.064 → 19.923 | 159 → 61 | 23,716 |

The 16-stack heavy-bottom case fell from 9,216 B to 0 B over the warm
window. The 16-stack mixed-load case fell from 75,248 B to 66,032 B.
The 9,216 B difference was 48 B per step from the constraint sort.
Four stacks allocated 0 B in both builds. The mixed-load scene's
remaining allocations are tracked under [GRV-Benchmark-035](#grv-benchmark-035).

The three scene traces, including solver caches, matched the baseline
in Release and ReleaseLean. Both configurations agreed on the warm
allocation totals. The original support and quality CSVs were
byte-identical, and both 1,024-actor strategy traces and work counts
matched. Release passed 8,401 tests and ReleaseLean passed 8,330. Each
covered 53,953 lines, 22,088 branches and 5,635 methods with zero
uncovered points.

In table order the digests are `ca6a24a2e0b45eef72d47f6cff5319e3`,
`470a322fba9007a7607b3f810bb017be`, and
`20cc20e1f1a705542a6c3d871c1d7109`.
The dense heavy-bottom case still exceeded the interval on all 192
steps. Use that case and the mixed-load control together for follow-up.
The all-heavy case stresses fallback frequency, while the mixed-load
case shows the budget sensitivity when only half the stacks are heavy-bottom.

<a id="grv-benchmark-035"></a>

### GRV-Benchmark-035 — Warm Pair Growth In Rotated Cuboid Stacks

**Discovered** 2026-11-09 during the
[GRV-Benchmark-034 density capture](#grv-benchmark-034).
**Owner** Gravitas 3D partition refresh and collision-pair acquisition.
**Status** Active. Repeatable warm growth on two frames in the sixteen-stack
mixed-load case.

Run `measure-quality-stack-density` with released packages. The mixed-load
case allocated 66,032 B across the 192 warm steps after removing constraint
sort boxing. The `dff545e8` baseline probe placed the excess above its
48 B per-step sort cost on frames 235 and 236. On that baseline,
`CreatePair` accounted for 55,040 B across 20 calls, within 58,368 B in
collision distribution. Partition preparation and corrected position
publication accounted for another 1,224 B and 6,440 B. Exact normal and
friction response allocated 0 B in the stage probe.

All 20 acquisitions were on frame 236 with pooling enabled and the
3D pair cache empty. The keys linked actors from separate stacks, including
`1/5`, `1/13`, and `26/29`. Check whether those candidate pairs can be
rejected before acquiring state. Compare with the same trajectories and
support rows before changing pair admission or reservation policy.

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
