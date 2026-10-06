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
  `GRV-Benchmark-025`.
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

## Baseline Commands

Build the benchmark project before capturing evidence:

```powershell
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0
```

List the continuous-collision evidence rows:

```powershell
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll continuous-collision-evidence --list flat
```

Run the current continuous-collision evidence smoke:

```powershell
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll continuous-collision-evidence --filter "*Evidence*" -j Short -i
```

After runtime changes, validate the package paths:

```powershell
dotnet test Gravitas.slnx --configuration Release
dotnet test Gravitas.slnx --configuration ReleaseLean
```

## Active Signals

### GRV-Benchmark-019 — Complete Triangle/Cone Contact Cost

**Discovered:** 2026-09-29.  
**Status:** Measured performance follow-up to the GRV-Issue-086 correctness repair.  
**Owners:** FixedMathSharp's triangle/cone support selection, rim roots and paired
witnesses; Gravitas's mesh candidate traversal.

Correctness, resource proofs and shared-owner control evidence are retained in
the [completed contact plan](done/2026-09-28-complete-triangle-cone-contact-plan.md).

The complete contact query replaces missed intersections and inconsistent
normal/depth/anchor output. Its exact geometry is the correctness baseline.
The bounded optimization pass reused the existing face-disk certificate and
principal-axis depth reducer; it did not introduce an approximate fallback,
new arithmetic engine, cache or allocation.

Matched Windows/i7-9700K measurements use .NET 8.0.29, SDK 10.0.302, two launches,
five warmups and fifteen measured iterations per launch. The launcher is
BelowNormal with affinity mask 3 and `DOTNET_PROCESSOR_COUNT=2`; only one heavy
workload runs at a time. Values below are **microseconds per dispatched query**,
not batches or simulation frames. Every row reports zero managed allocation.

| Geometry | Old incomplete path | Initial complete solver | Final mean | Final standard deviation |
| --- | ---: | ---: | ---: | ---: |
| Base face | 19.983 | 152.1 | 33.89 | 0.676 |
| Side face | 17.525 | 567.4 | 62.57 | 1.193 |
| Apex face | 28.838 — wrong depth | 169.3 | 55.67 | 1.331 |
| Side intrusion | 15.407 — missed contact | 301.5 | 283.08 | 6.321 |
| Oblique rim | 17.403 — missed contact | 905.3 | 850.98 | 31.440 |
| Interior stationary rim | Not captured | Not captured | 1,407.39 | 32.889 |
| Rim touch | 17.932 — missed touch | 703.1 | 692.58 | 13.452 |
| Rim gap | 17.652 | 122.0 | 120.78 | 2.479 |
| Unrepresentable relative center | 6.069 — missed contact | 1,708.7 | 1,652.05 | 31.686 |

Final base/side/apex costs are about 78% / 89% / 67% lower than the initial
complete implementation. The old base/side/gap rows had the correct answers
for these fixtures and remain cheaper: the final costs are approximately
1.70x / 3.57x / 6.84x theirs. Five other old rows returned the wrong result and
are repair-cost comparisons, not equivalent-correctness speed regressions.
The interior-rim row was added with its independent exact-depth oracle after
the old baseline. General curved/full-domain work near 0.7–1.7 ms per query is
still a meaningful cost; ordinary paths also merit further profiling.

All eighteen final cone child launches passed their semantic setup checks.
BenchmarkDotNet flags multimodal apex and oblique-rim distributions. Small
curved-row differences are descriptive, not established gains: interior-rim,
rim-touch and rim-gap intervals overlap the preceding certificate-only run.

Reproduce from the repository root after a local-stack Release benchmark build:

```powershell
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -p:UseLocalLsfStack=true -m:1 -p:BuildInParallel=false
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll mesh-cone-contact --launchCount 2 --warmupCount 5 --iterationCount 15 --affinity 3 --artifacts artifacts/grv086/cone-recheck
```

Apply the launcher limits above as well; the affinity option alone does not set
process priority or the runtime's processor-count hint. Setup verifies expected
classification and certified depths before measurement. Original and final
reports are under `artifacts/grv086/baseline`, `initial-complete`, `final`
(certificate-only) and `optimized-and-controls`.

**Next useful action:** Measure duplicate axial/face support preparation for
horizontal faces, then profile nonwinning rim charts, root ranking and witness
materialization. Reuse shared improvements where they also help
[triangle/cylinder contacts](#grv-benchmark-018--complete-trianglecylinder-contact-cost).
Preserve complete admission, exact ties, canonical paired witnesses, raw-neighbor
classification, bounded stack use and zero allocations. This is separate from
the cone-volume-query correctness defect, GRV-Issue-087.

### GRV-Benchmark-018 — Complete Triangle/Cylinder Contact Cost

**Discovered:** 2026-09-27; final aggregate capture 2026-09-28.  
**Status:** Measured follow-up after the GRV-Issue-082 correctness repair.  
**Owners:** FixedMathSharp's triangle/cylinder contact selection and witness
materialization; Gravitas's mesh candidate traversal and cap enrichment.

The complete contact owner repairs missed intrusions, false rim contacts and
mismatched normal/depth/witness output. Its exact geometry is the correctness
baseline; the previous center-nearest approximation is not an acceptable fast
fallback.

Same-machine BenchmarkDotNet comparison: Windows, Intel i7-9700K, .NET 8.0.29,
SDK 10.0.302, two-core process affinity, one workload at a time, two launches,
five warmups and fifteen measured iterations per launch. Means are **per batch
of 64 collider pairs**, not per contact or simulation frame. All rows measured
zero managed allocation.

| Row | Previous incomplete path | Unpruned implementation | Face certificate / pruning | Final |
| --- | ---: | ---: | ---: | ---: |
| `CheckMeshCylinderPairs` | 3.812 ms | 32.664 ms | 8.348 ms | **6.064 ms** |
| `CheckConcaveMeshCylinderPairs` | 2.835 ms | 62.688 ms | 5.469 ms | **3.825 ms** |

Final standard deviations were 0.1549 and 0.1034 ms; 99.9% confidence-interval
half-widths were 0.1035 and 0.0691 ms. The final path takes about **59% / 35%
more time than the incomplete predecessor**, but **81% / 94% less than the
unpruned implementation**. The unpruned capture preceded the final one-round
rim-witness correction, so that comparison includes both correctness and
performance changes, not an isolated optimization attribution.

Retained reductions use exact proofs: an inscribed-ball face certificate,
duplicate analytic/root-boundary removal, winner-only world-normal work, and
principal-direction cancellation before depth rounding. None replace an
uncertain result with a tolerance, sampled axis or early false answer. The final
bounded pass cut another 27% / 30% from the preceding refined aggregate rows.

Reproduce from the repository root in coordinated source mode:

```powershell
$env:DOTNET_PROCESSOR_COUNT = '2'
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0 -p:UseLocalLsfStack=true -m:1 -p:BuildInParallel=false -p:UseSharedCompilation=false -nr:false
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll collision-detection --filter '*CheckMeshCylinderPairs*' '*CheckConcaveMeshCylinderPairs*' --launchCount 2 --warmupCount 5 --iterationCount 15 --artifacts artifacts/grv082-final-bench
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll mesh-cylinder-contact --filter '*' --launchCount 2 --warmupCount 5 --iterationCount 15 --artifacts artifacts/grv082-final-focused-bench
```

Also constrain process affinity to two cores when comparing with this capture;
the environment variable alone is not a CPU-affinity setting. The focused rows
preflight expected classification and certified depths outside measurement.
They distinguish cap/side faces, the original intrusion, oblique rim contact,
rim touch and a rim gap for cylinder and mixed circle-slab consumers. Cylinder
cap contacts may build a four-point manifold; mixed circle slabs return one
contact, so those columns are not identical workloads.

Final focused means, **microseconds per query**, with zero managed allocation
in all twelve rows:

| Geometry | Cylinder / triangle | Circle slab / triangle |
| --- | ---: | ---: |
| Cap face | 57.05 us | 21.63 us |
| Side face | 31.94 us | 31.83 us |
| Original cap intrusion | 1,055.12 us | 1,053.47 us |
| Oblique rim | 1,179.01 us | 1,160.16 us |
| Rim touch | 846.88 us | 833.32 us |
| Rim gap | 44.24 us | 44.01 us |

The unpruned cylinder cap/side rows were 269.91 / 828.68 us. Those ordinary
features benefited substantially from the retained certificates and work
removal; the intrusion/rim rows did not show a comparable reduction. General
contact at roughly one millisecond per query remains a meaningful capacity
concern, not a cost hidden by the aggregate improvement. BenchmarkDotNet flagged
multimodal cylinder cap-face and rim-touch distributions and removed four
circle-slab cap-face outliers. Retain the report's uncertainty and do not claim
small percentage changes on these rows without a fresh matched comparison.

The disposable reports are under `artifacts/grv082-before-bench`,
`artifacts/grv082-unpruned-bench`, `artifacts/grv082-refined-bench`,
`artifacts/grv082-final-bench` and the corresponding `*-focused-bench` folders.
The old runtime baselines are Gravitas `0a6f480` and FixedMathSharp `fd1cb8f`;
the final runtime is the GRV-Issue-082 repair on top of those revisions.

**Next useful action:** Isolate the retained edge/rim chart, root-comparison and
paired-witness costs on the focused rows before proposing another optimization.
Prefer a proof that avoids nonwinning work, preserving complete feature
admission, exact tie ordering, one-round witnesses, zero allocations and both
target frameworks. Keep this performance follow-up separate from the resolved
contact-correctness issue and the unrelated positive-core capsule-slab defect.

## Experimental Signals

| Signal                                                                                 | Status                                           | Revisit When                                                                                                            |
| -------------------------------------------------------------------------------------- | ------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------- |
| [GRV-Benchmark-024 — Curved-contact workload frequency and step cost](#grv-benchmark-024--curved-contact-workload-frequency-and-step-cost) | Experimental; scene frequency and capacity unmeasured | Representative deterministic scenes or host traces can establish whether exact curved-feature work materially consumes the intended step budget |
| GRV-Benchmark-014 — Exact triangle-pair contacts regress dense concave-mesh throughput | Capacity-sensitive; local optimization exhausted | A topology or exact classifier design can reduce complete triangle-pair SAT evaluations without a competing answer path |

### GRV-Benchmark-024 — Curved-Contact Workload Frequency And Step Cost

**Discovered:** 2026-10-06, during the workload-budget follow-up to #020.  
**Source:** closed [GRV-Benchmark-020](#grv-benchmark-020--complete-capsulecircle-slab-contact-cost)
and [GRV-Benchmark-021](#grv-benchmark-021--complete-capsulestadium-slab-curved-contact-cost)
timings and stage profiles.  
**Status:** Experimental evidence gap; representative scene frequency and
complete-step capacity have not been measured. This does not reopen either
isolated-query investigation or establish a release-blocking regression.  
**Owner:** Gravitas benchmark workloads and stage attribution; FixedMathSharp
only if workload evidence justifies further shared geometry optimization.

The final #020 two-launch fixture costs **696.036 / 699.465 microseconds** per
complete mixed / 3D oblique query, at **0 B/op**, while ordinary cap/side/zero-core
contacts cost roughly 10–11.5 microseconds. #021 retains roughly 0.53–1.05 ms
for its distinct curved fixtures. The [#020 refinement report](2026-10-05-capsule-circle-cost-refinement.md)
records the commands, confidence intervals, final-source checks and profiles.
Its full matrix can be reproduced after a fresh local-stack Release build:

```powershell
$env:UseLocalLsfStack = 'true'
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll --filter '*CapsuleCircleContactBenchmarks*' --launchCount 2 --warmupCount 5 --iterationCount 15 --iterationTime 250 --affinity 3 --keepFiles --exporters json --artifacts artifacts/grv-benchmark-024/query-controls
```

These fixtures establish cost when the configuration occurs, not its prevalence
in games. An oblique interior-rim contact is a geometric configuration of ordinary
capsules and cylinders or embedded slabs, rather than a separate collider type.
Tilt, relative position, dimensions and contact evolution determine the work.
Counting only final rim contacts can miss expensive feature candidates evaluated
before an ordinary feature wins or the query rejects contact. Neither rarity nor
common occurrence is established by the current evidence. Multiplying isolated
query time by a hypothetical count is illustrative, not measured world capacity.

**Next experiment:** Reuse the existing contact fixtures and world-step benchmark
support to characterize deterministic moving scenes before changing the solver:

1. Establish the geometric envelope with reproducible pose/orientation sweeps
   around the rim, including ordinary contacts and separated near-rim candidates.
   Then run explicit upright-agent, tilted/tumbling, rim-sliding and dense-contact
   scenarios across shape dimensions and local density. Synthetic sweep frequency
   must remain separate from observed host frequency; publish scene assumptions.
2. In benchmark-only support, distinguish dispatched pair queries, expensive
   root-path entries, admitted curved candidates and final winning features.
   Reuse existing hooks where sufficient. Count repeated work across frames as
   well as unique pairs; a final contact label alone is insufficient attribution.
3. Measure complete `Simulate` plus `LateSimulate` steps over sustained frames:
   mean, p95/p99 and maximum observed time, warmed allocations, candidate counts
   and expensive-path rate. Profile stages separately from clean timing runs;
   keep counters and wall-clock timing outside authoritative simulation state.
   Assess explicit host tick budgets with headroom for the rest of the host.
4. Retain equivalent 3D cylinder and mixed circle/stadium-slab cases with their
   distinct root policies. Pure 2D workloads provide a dimensional control, not
   an assumed consumer of the 3D oblique solver. Use fixed initial state/input,
   repeatable seeds, contact preflight and replay checks; verify instrumentation
   preserves answers. Use `UseLocalLsfStack=true` throughout coordinated runs.

**Decision gate:** Publish a capacity characterization or written no-change
decision if the expensive path has a small measured contribution in the target
scenes. Promote a focused optimization or workload/API plan only when reproducible
complete-step evidence shows material budget pressure. Preserve exact contacts,
stable ordering, zero warmed allocations and 100% reachable line/branch/method
coverage for any implementation changes. Do not add approximate collision,
speculative caches, public instrumentation or merged root solvers merely because
the isolated worst fixture is expensive. #020 and #021 remain closed.

### GRV-Benchmark-014 — Exact Triangle-Pair Contacts Regress Dense Concave-Mesh Throughput

**Discovered:** 2026-08-01  
**Source:** full-domain triangle-pair Phase 2 comparison against its preserved
scalar mesh/mesh baseline  
**Status:** Experimental capacity guidance. Shared exact-projection and
depth-ranking duplication plus the retained signed one-limb specialization
recovered substantial throughput. A final bounded pass found no further local
change worth retaining; dense dynamic concave mesh/mesh contact is not a
competitive release path.

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

| Signal                                                                                  | Status | Closed     | Resolution                                                                                                                                                                                                                                                                                      |
| --------------------------------------------------------------------------------------- | ------ | ---------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [GRV-Benchmark-020 — Complete capsule/circle-slab contact cost](#grv-benchmark-020--complete-capsulecircle-slab-contact-cost) | Closed | 2026-10-05 | Exact output bounds, invariant normal products and certified Sturm signs reduce endpoint output to 37–39 µs and representative oblique contacts to 0.70 ms, with shared-consumer gains, 0 B/op and 100% coverage. Remaining exact root cost is published without a host-rate guarantee. |
| [GRV-Benchmark-021 — Complete capsule/stadium-slab curved contact cost](#grv-benchmark-021--complete-capsulestadium-slab-curved-contact-cost) | Closed | 2026-10-05 | Shared exact root isolation and negative global-minimum certificates improve four fixtures by 11.5-26.4%, at 0 B/op and 100% coverage. Committed in Gravitas `989c7f9` / FixedMathSharp `bd8f6a2`; remaining costs are published without a host-rate guarantee. |
| [GRV-Benchmark-023 — Circle workload partition and grounding scaling](#grv-benchmark-023--circle-workload-partition-and-grounding-scaling) | Closed | 2026-10-04 | Ownership/storage/math repairs and lean 2D gathering retained; repeated controls, 100% coverage and published configuration costs support the no-further-change decision. Latest refinement remains unstaged for review; no fixed-Hz guarantee. |
| GRV-Benchmark-022 — Exact pure-2D circle contact cost | Closed | 2026-10-02 | Shared normalization optimization committed; retain exact contact owner under documented no-further-change decision. Scene partition/grounding costs are documented separately in closed GRV-Benchmark-023. |
| GRV-Benchmark-012 — Mixed discrete broad-phase allocation at 32 pairs                   | Closed | 2026-08-04 | Two independent rotational runs and corrected sparse, dense, and churn broad-phase rows reproduce `0 B/op`; the stale benchmark lifecycle and unrepresentative 4,096-collider monolithic-grid row were repaired without speculative runtime preallocation                                       |
| GRV-Benchmark-017 — Mixed public sweep traversal on extreme sparse-grid spans           | Closed | 2026-08-04 | GridForge's two-tier hash/BVH index replaces 64-billion-cell registration with active-grid scaling; Gravitas completes the exact public sweep in 14.8-16.0 us at 0 B with deterministic candidate and hit order; full evidence is retained in GridForge's completed two-tier spatial-index plan |
| GRV-Benchmark-013 — Mesh scale rebuild allocation                                       | Closed | 2026-08-03 | Convex support topology is built once and scale changes refit transactional node bounds in linear time; subdivision 8/16 rows fall from 4,032/16,320 B to 0 B and improve by 7.9%/7.8%                                                                                                          |
| GRV-Benchmark-015 — Exact 3D contact-response ordinary throughput                       | Closed | 2026-08-03 | Exact aligned-frame point anchors improve direct rows by 61.0-95.9% and the unchanged 24-row Gravitas matrix by 46.4% median versus the exact baseline; confirmation remains within 0.7% median at 0 B and 100% coverage                                                                        |
| GRV-Benchmark-016 — Exact canonical OBB ordinary throughput                             | Closed | 2026-08-03 | One exact relative-frame kernel per relation improves matched direct rows by 35.3-64.0% and Gravitas rows by 30.9-55.7%; full DefaultJob confirmations remain at 0 B and 100% reachable coverage                                                                                                |
| GRV-Benchmark-011 — Physics-material combine numeric hardening                          | Closed | 2026-07-13 | Overflow-safe average and geometric-mean edge handling preserve deterministic coefficient semantics; the default geometric-material response benchmark remains allocation-free with no credible timing regression                                                                               |
| GRV-Benchmark-010 — Checked mesh scale and thin-shell cache cost                        | Closed | 2026-07-12 | Scale changes retain deterministic O(triangle-count) checked rebuilding, while successfully validated shell and volume properties are cached and repeated cached reads remain allocation-free                                                                                                  |
| GRV-Benchmark-009 — Replay hash collider-ID churn scaling                               | Closed | 2026-07-05 | 2D and 3D collider registration now uses a shared reusable-slot registry; authoritative replay hashes traverse canonical live registration order with dense replay ordinals, while deleted ID history remains outside replay identity                                                           |
| GRV-Benchmark-008 — Pure 2D response position-correction repartition allocation         | Closed | 2026-06-28 | Gravitas reuses empty retained partitions for immediate repartitioning; GridForge stores the common single voxel partition inline and keeps diagnostic names off success paths                                                                                                                  |
| GRV-Benchmark-005 — SwiftCollections sort hot-path allocation                           | Closed | 2026-06-24 | SwiftCollections owns allocation-free sort and sorted-key APIs; Gravitas removed `SwiftListSortUtility`                                                                                                                                                                                         |
| GRV-Benchmark-006 — Mixed mesh finite-slab triangle scaling signal                      | Closed | 2026-06-24 | Mixed and pure 3D query services expose mesh-triangle candidate counts, dedicated triangle-volume benchmarks cover dense and false-positive mesh targets, and pure 3D convex-source mesh sweeps use ordered lower-bound triangle candidates                                                     |
| GRV-Benchmark-001 — Pure 2D dynamic CCD candidate asymmetry                             | Closed | 2026-06-23 | 2D uses a planar candidate index, skips mixed CCD indexing outside mixed mode, and benchmark resets use 2D reset parity                                                                                                                                                                         |
| GRV-Benchmark-002 — 3D shape-exact false-positive cost                                  | Closed | 2026-06-23 | Static CCD uses exact-source sweeps for non-sphere convex movers before conservative sphere fallback refinement                                                                                                                                                                                 |
| GRV-Benchmark-007 — 3D dynamic shape-exact BDN allocation signal                        | Closed | 2026-06-23 | Shared exact-sweep bounds prefilters removed the scaling allocation/time signal from 3D dynamic false-positive rows                                                                                                                                                                             |
| GRV-Benchmark-003 — 3D full-runtime CCD allocation                                      | Closed | 2026-06-23 | GridForge allocation-free line tracing plus Gravitas 3D raycast adoption                                                                                                                                                                                                                        |
| GRV-Benchmark-004 — Grounding raycast probe allocation                                  | Closed | 2026-06-23 | Same raycast trace fix removed automatic ray-grounding allocation                                                                                                                                                                                                                               |

### GRV-Benchmark-020 — Complete Capsule/Circle-Slab Contact Cost

**Discovered:** 2026-09-29.  
**Status:** Closed 2026-10-05; coordinated refinements are verified for review.
Remaining exact oblique-root cost is published under the judgment-based closure bar.  
**Owner:** FixedMathSharp's shared cylinder/capsule feature selection and exact
output materialization; Gravitas consumes that owner for mixed circle slabs and 3D
cylinders.

The complete query rejects separated cap rims and selects minimum depth before
rounding. The previous mixed direction subset could return a false contact or
nonminimum depth; its cheaper wrong answers are not correctness-equivalent
optimization targets. Ordinary cap, side and zero-core fixtures did return the
correct answers before the repair, and their additional cost is real.

Matched Windows 11/i7-9700K measurements use .NET 8.0.29, SDK 10.0.302, one
launch, three warmups and ten measured iterations per row. The launcher is
BelowNormal with affinity mask 3 and `DOTNET_PROCESSOR_COUNT=2`; only one heavy
workload runs at a time. Values are **microseconds per dispatched query**, not
batches or simulation frames. Every row reports **0 B/op**.

| Geometry | Old mixed path | Complete mixed, first run | Complete mixed, confirmation | Equivalent 3D cylinder, confirmation |
| --- | ---: | ---: | ---: | ---: |
| Cap | 12.27 | 19.32 | 19.07 | 21.51 |
| Side | 16.70 | 25.66 | 25.12 | 26.35 |
| Zero capsule core | 16.67 | 25.78 | 24.75 | 26.01 |
| Endpoint rim | 30.54 — nonminimum depth | 106.69 | 108.18 | 108.30 |
| Separated endpoint rim | 30.62 — false contact | 25.78 | 25.54 | 25.90 |
| Oblique interior rim | 36.70 — nonminimum depth | 1,224.40 | 1,191.51 | 1,196.17 |

The mixed confirmation's standard deviations are 0.099 / 0.080 / 0.103 /
1.051 / 0.159 / 8.028 microseconds in table order. Equivalent 3D controls show
the same expensive curved-feature behavior. These are complete wrapper costs:
3D also validates public inputs and updates a manifold, while mixed constructs
its canonical contact. Their difference does not isolate adapter overhead, and
there is no historical 3D baseline for these new fixtures.

**2026-10-05 progress:** [Exact output refinement](2026-10-05-capsule-circle-cost-refinement.md)
reuses rigorous magnitude-floor bounds and one jointly scaled depth-polynomial
reduction. Fresh committed/retained mixed oblique means improve from
1,186.47 to **745.01 microseconds**; equivalent 3D improves from 1,124.91 to
**724.62 microseconds** (37.2% / 35.6%). All twelve rows remain 0 B/op and both
repositories retain exact 100% standard/Lean line, branch and method coverage.
The report records small, incompletely isolated ordinary-row shifts, targeted
cap repeats, resource proofs and actual timed-child assembly verification;
stale-cache captures are excluded. The signal remained open after that phase.

**2026-10-05 closure:** The same [refinement report](2026-10-05-capsule-circle-cost-refinement.md)
now records certified Sturm point signs, hoisted invariant normal products and
factored radical bounds, all in existing FixedMathSharp owners. The final mixed /
3D endpoint means are **36.578 / 38.962 µs**, versus the fresh committed control's
86.95 / 88.25 µs (about **57.9% / 55.9%** lower). Cap, side and zero-core output
are roughly **10–11.5 µs**. Representative oblique means are **696.036 / 699.465
µs**; across the full investigation they fall from the first phase's fresh
1,186.47 / 1,124.91 µs control by about **41.3% / 37.8%**. These cumulative
figures compare recorded captures across refinements, not one new paired run;
the report retains the stage-specific confidence intervals and repeats.

All twelve final circle/slab rows, all thirty-two before/after shared controls
and the sixteen-row final-source shared recheck report **0 B/op**. The paired
stadium ordinary output improves another 36–54%, and its straight-rim output
improves 23%; the paired separated/oblique/triangle controls show small mean
shifts with overlapping intervals. Final Release/Lean suites retain exact
**100% reachable line, branch and method coverage**, exclusions unchanged;
Debug stack/allocation checks, both target frameworks and both DocFX/local-link
gates pass. Source reviews found no actionable correctness or simplicity issues.
A measured retained-cell certificate was removed because it established no gain.

**Closure decision:** Proven correctness, reproduced gains across mixed/3D and
shared consumers, zero allocation and bounded resources meet the same bar used
for #021. Remaining root acquisition and exact quartic signs are documented;
no cheaper demonstrated refinement remains in this investigation. The roughly
0.70-ms representative oblique query is a workload-budget consideration, not a
fixed-rate host guarantee or proof of an unavoidable lower bound. Preserve
full-domain classification, nearest-even depth, deterministic ties and canonical
anchors. Further work should start from new measured workload evidence, without
restoring incomplete directions or adding an approximate solver.

**Original fixture capture:** Build `Gravitas.slnx -c Release -p:UseLocalLsfStack=true`, then:

```powershell
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll capsule-circle-contact --warmupCount 3 --iterationCount 10 --launchCount 1 --exporters json --artifacts artifacts/grv085/after-confirm
```

Baseline source: Gravitas `04805b8` and FixedMathSharp `9833123`. Fixture geometry
and the measured mixed method were unchanged between captures; corrected-answer
setup assertions and the paired 3D control were added after the legacy capture.
Setup checks classification, known quarter-unit depths and cross-path depth
agreement. Reports and raw measurements are under `artifacts/grv085/before`,
`after` and `after-confirm`; retain the individual rows rather than an average
that hides the curved-feature cost. Correctness evidence is retained in
[GRV-Issue-085](issue-tracker.md#grv-issue-085---mixed-capsulecircle-slab-contact-bypasses-the-complete-upstream-query).

#### Capsule Rim Cost Relationship (GRV-Benchmark-020 And 021)

Source inspection on 2026-09-30 connects these signals without establishing a
shared CPU hotspot. Keep both fixture families and their measured results
distinct; investigate them together, starting with oblique contacts and then
endpoint/straight-rim output materialization.

The 2026-10-04 coordinated profiles now distinguish their expensive owners:
circle/cylinder ellipse magnitude-to-raw comparison takes about 53-55% inclusive
sampled CPU, while stadium constrained root acquisition, sign refinement and
value mapping remain expensive. The [refinement evidence](2026-10-04-capsule-slab-cost-refinement.md)
retains both profiles and mixed/3D/triangle controls. This does not establish a
single shared dominant leaf or justify merging the specialized solvers.
The 2026-10-05 [certified-sign refinement](2026-10-05-capsule-slab-sign-refinement.md)
moves the stadium analytic-winner bottleneck toward root acquisition, while
circle mixed/3D controls retain their distinct ellipse path. The subsequent
[root-isolation refinement](2026-10-05-capsule-slab-root-isolation-refinement.md)
closes #021 with reproduced gains, bounded resources and published remaining
cost. The subsequent circle/slab [output refinement](2026-10-05-capsule-circle-cost-refinement.md)
closes #020 with both dimensional controls and measured shared-consumer gains.
Both signals are closed; shared algebra does not imply shared root-selection policy.

The fixture names describe different features:

| Fixture | Capsule feature | Slab feature |
| --- | --- | --- |
| `EndpointRim` (020) | Core endpoint | Circular cap rim |
| `StraightRim` (021) | Core endpoint | Straight cap edge |
| Oblique interior rim (both) | Interior of tilted core | Curved cap rim |

Both endpoint/straight-rim fixtures have analytic whole-shape closest-point
certificates and reuse `MaterializeCylinderCapsulePenetration` for exact depth
and normal rounding; neither needs the oblique polynomial solver. They are not
matched geometry: the endpoint fixture has irrational depth/normal components,
while the straight-rim fixture has rational results. Their timings alone cannot
establish which solver is intrinsically cheaper.

The oblique paths both require wide arithmetic, polynomial roots, exact
comparisons and final rounding, but use different specialized root-selection
paths. The cylinder ellipse solver selects a largest-positive parameter root
under an unrestricted-domain proof. The stadium solver admits constrained
end-region roots and maps/compares their signed gaps through value roots, reusing
circular-rim algebra also consumed by triangle contacts. Copying the cylinder's
reflection/root-selection shortcut could discard a valid stadium contact.

Future profiling should identify the affected stages before attributing cost to a
shared leaf or merging solvers. Prefer demonstrated shared arithmetic and
materialization wins or proved feature certificates; retain ordinary mixed/3D and affected triangle
controls. Similar exact algebra does not yet prove the same dominant function,
an unavoidable correctness cost, or an optimization that will benefit both.

### GRV-Benchmark-021 — Complete Capsule/Stadium-Slab Curved Contact Cost

**Discovered:** 2026-09-30.  
**Status:** Closed 2026-10-05; Gravitas `989c7f9` and coordinated
FixedMathSharp `bd8f6a2` are committed.
Closure uses reproduced gains, preserved full-domain behavior and published
remaining cost; it does not certify a fixed-rate host budget.  
**Owner:** FixedMathSharp's complete capsule/stadium-slab feature selection,
shared circular-rim algebra and signed value-root comparison.

The complete query fixes false contacts and nonminimum penetration. Subsequent
[retained-root/positive-certificate](2026-10-04-capsule-slab-cost-refinement.md),
[certified-sign](2026-10-05-capsule-slab-sign-refinement.md), and
[root-isolation/global-minimum refinements](2026-10-05-capsule-slab-root-isolation-refinement.md)
retain every constrained chart and exact classification/rounding contract.
The [correctness repair history](done/2026-09-29-complete-capsule-slab-contact-plan.md#refined-matched-performance)
retains the older complete and incorrect/incomplete baselines.

The final phase borrows existing evaluation scratch for certified Sturm signs,
shares batch subdivision counts and reciprocal-chart Sturm chains, and proves
when a negative stationary gap is the complete shape's global minimum before
ending traversal. No speculative cache, second solver or accuracy setting is
added. The matched final confirmation against committed FixedMathSharp
`9b1f173` gives the following microseconds per dispatched query:

| Stadium fixture | Fresh committed baseline | Final | Mean reduction |
| --- | ---: | ---: | ---: |
| Oblique analytic winner | 1,407.0 ± 7.42 | 1,052.1 ± 11.54 | 25.2% |
| Original oblique rim | 716.7 ± 5.16 | 527.3 ± 2.89 | 26.4% |
| Irrational oblique rim | 897.0 ± 7.68 | 794.1 ± 5.06 | 11.5% |
| Positive oblique rim | 841.9 ± 8.42 | 734.7 ± 6.39 | 12.7% |

Error is the 99.9% confidence half-width. Both final accepted confirmations
have intervals disjoint from both committed captures in every fixture. All
four rows and all 28 separate ordinary stadium, mixed circle-slab, equivalent
3D cylinder and triangle controls report **0 B/op** and pass behavior preflight.
The final evidence records resource proofs, independent regression oracles,
rejected experiments, profiles and raw capture locations. Both repositories
pass full Release/ReleaseLean suites with exact **100% reachable line, branch
and method coverage**, focused Debug/resource checks and both DocFX gates.

**Closure boundary:** The residual roughly **0.53–1.05 ms/query** curved cost
remains significant. Root isolation and exact sign/arithmetic work still
consume much of the sampled CPU. Dense curved-contact workloads require their
own complete-step capacity measurements; these microbenchmarks do not establish
MMO/strategy throughput or cross-platform replay. The focused investigation
closes under the user's judgement-based performance bar. GRV-Benchmark-020
is also closed after its distinct ellipse-depth and shared-output investigation;
related algebra alone does not establish one common dominant hotspot.
Released-package validation remains a release gate after the coordinated
FixedMathSharp release.

**Reproduce:** after a Release local-stack benchmark build, run
`dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll --filter '*CapsuleSlabContactBenchmarks*Oblique*' --launchCount 2 --warmupCount 5 --iterationCount 15 --invocationCount 512 --affinity 3 --exporters json --artifacts artifacts/grv-benchmark-021/reproduce`.
Use `DOTNET_PROCESSOR_COUNT=2`, BelowNormal priority and one heavy workload.
Set `UseLocalLsfStack=true` in MSBuild and the environment for child builds.
The command requests affinity 3; these exports display `Affinity=11`, so no
claim of independently verified effective affinity is made.

### GRV-Benchmark-023 — Circle Workload Partition And Grounding Scaling

**Discovered:** 2026-10-02 during GRV-Benchmark-022 isolation.  
**Status:** Closed 2026-10-04 under the predictable-scaling/published-cost
criterion and documented no-further-change decision below. The latest 2D query
gathering, bounds-first rejection, characterization and documentation changes
remain unstaged/uncommitted for user review.
Partition phase committed as `f23d836`; shared radial-math and planar
capture refinements committed as FixedMathSharp `6789c09` and Gravitas `ee613db`.
Prepared 2D query and grounding-selection refinement committed as `6c0c2f0`.
The centered-transform and locked lookup refinements are committed as
FixedMathSharp `5447b34` and GridForge `3f34f8b`; Gravitas's characterization,
benchmarks and documentation are committed as `bf11409`.
Streamed 3D grounding was rejected after repeated sparse-ray regressions;
its public characterization tests and benchmark fixture are retained.
Warmed full-step costs are published below, separate from exact contact-query cost.  
**Owner:** Gravitas retained partition rental/distribution and collider refresh,
2D query preparation and ground-hit selection, plus GridForge voxel partition
lookup used by planar grounding.

**Closure criterion:** Predictable scaling and published costs, selected by the
repository owner, replace a fixed-Hz/frame-budget target for this signal.
Collider/pair counts, geometry, cell size, covered memberships, emitted links
and candidate counts are published beside matched timings and allocations.
Retained changes passed repeated controls, deterministic behavior and full
Release/Lean coverage gates, with material tradeoffs documented. Closure does
not mean that every scene fits a particular host frame budget.

An exploratory `circle-contact-simulation` capture with default unit cells
measured 64 diagonal pairs at 13.35 ms/step and 1024 axis pairs at 42.59 ms/step.
The 1024 diagonal setup was stopped after more than 160 seconds of child CPU
time and approximately 800 MB working set. These are diagnostic signals, not a
completed matched benchmark: the initial grid also clipped the first diagonal
row after positional correction. Raw partial output is retained in
`artifacts/grv-benchmark-022/workload-before.log`; the live process observation
is transcribed in `unit-cell-process-observation.txt` alongside it.

A separate, padded-grid EventPipe capture of 64 diagonal pairs puts 43.0% of
actual-iteration sampled CPU in `Voxel.TryGetPartition` and 15.4% in partition
distribution (exclusive). Positional correction/repartition contributes 27.4%
inclusive and grounding 15.0% inclusive; these overlap. Contact generation
and lever materialization are small in this workload. Traces and actual-only
analysis are in `artifacts/grv-benchmark-022/workload-profile*`.

Source inspection identifies an additional setup risk: when the inactive pool
is empty, `RetainedPartitionLifecycle.TryRetireEmptyForReuse` scans all retained
partitions before each new rental. A fully occupied initial scene has nothing
to retire, allowing quadratic registration work. Per-partition sparse sets also
grow with the highest global collider ID. The rental owner is shared by 2D,
3D and mixed services; dimension parity matters for any repair.

**Original isolation step:** Measure registration and warmed refresh/distribution
separately while varying cell size, covered voxels and collider count. Profile
registration to distinguish rental scans from sparse-set growth. Preserve
deterministic reuse, empty-partition reclamation and allocation guards; do not
remove GridForge locking based on this trace. GRV-Benchmark-022 now uses explicit
16-by-1-by-16 cells to isolate sustained contact work; that fixture choice is
not a runtime fix for this signal.

With explicit 16-by-1-by-16 cells, the completed 1024-pair diagonal workload still
measures 35.99 +/- 0.47 ms/step, versus 4.22 +/- 0.09 ms for reset plus direct
queries. A separate actual-iteration profile puts grounding at 29.6% inclusive,
partition lookup at 18.7% exclusive, and partition distribution at 14.8%
inclusive. The complete capsule contact owner contributes 7.9% inclusive and
anchor offset materialization 1.6%. These are overlapping sampled paths, not a
stage-time decomposition. Fine cells amplify the problem but do not explain all
large-scene cost. The full-step distributions include bimodality and short
iteration warnings; preserve the raw results when assessing the published-cost
and predictable-scaling criterion above. Final coarse results and traces are in
`artifacts/grv-benchmark-022/after` and `coarse-profile*`.

**Partition hardening, 2026-10-02:** The shared retained lifecycle now tracks a
dense eligible-empty list instead of scanning occupied payloads on every cold
rental. Membership transitions maintain independent retained/empty indices;
reuse chooses the last dense eligible entry with ownership/occupancy checks,
while expiry preserves its bounded retained-list sweep. Registration reserves
eligible capacity for every retained payload, so a later wave of emptying cannot
allocate. A counting regression reduces 32 occupied rentals from 496 prior
`IsEmpty` reads to a linear bound. A strict moving-constraint regression exposed
152 bytes of late eligible-list growth; capacity reservation fixes it without
weakening the gate or increasing test warmup.

All three partition types now use existing `SwiftHashSet<int>` membership
storage sized by local population. A global ID of `1 << 20` previously allocated
16,777,584 bytes for one dynamic membership and its awake subset in each mode;
the new dimensional regression requires less than 16,384 bytes. Mutable public
membership fields are internal to prevent edits bypassing lifecycle bookkeeping.
Public role/awake counts and sorted caller-owned copies replace inspection; see
the [migration guide](../MIGRATION.md#partition-membership-ownership).

Pure 2D, 3D and mixed coverage refresh keeps memberships shared by the old/new
voxel sets, removes departures in previous coordinate order and publishes in
tracer order. Full `WorldVoxelIndex` identity includes grid/world lifetimes.
Mobility changes replace the old role; surviving dynamic memberships synchronize
awake state, and freeze/unfreeze transitions refresh awake membership directly.
Bounds, geometry/version publication, planar candidate indexing, deferred
refresh and topology rebuild semantics remain covered. Pure distribution paths
reuse their existing sorted static-style copy helpers. No upstream source change,
new collection owner, math kernel or voxel cache is needed.

**Collection tradeoff:** `partition-membership` compares existing owners with
global IDs starting at 65536. Add/remove plus sorted copy takes 15.06/11.78/19.39
ns for sparse/hash/packed storage at one member; at 64 members it takes
169.56/277.44/239.50 ns. All are 0 B/op after warmup. Hash storage addresses the
global-ID memory failure and wins the one-member row, but its denser copy cost
is a real tradeoff, not a universal collection speedup. Integrated scene rows
remain the evidence for the affected workload.

**Matched cold registration:** `ColliderCount` is the total number of dynamic
plus static colliders, half as many independent radius-five diagonal pairs.
Context/grid/array creation and cleanup are outside the measured registration.
Error is half of the 99.9% confidence interval; allocations are KB/op.

| Total colliders / XZ cell edge | Before mean +/- error (ms) | After mean +/- error (ms) | Before / after allocation (KB) |
| --- | --- | --- | --- |
| 64 / 1 | 118.890 +/- 2.1111 | 12.096 +/- 1.7149 | 5728.29 / 4107.09 |
| 64 / 16 | 2.947 +/- 0.0492 | 2.975 +/- 0.0730 | 256.47 / 237.95 |
| 256 / 1 | 2753.815 +/- 179.1855 | 41.426 +/- 39.1938 | 46411.04 / 16403.90 |
| 256 / 16 | 11.940 +/- 0.6168 | 11.812 +/- 0.1554 | 1220.05 / 912.28 |
| 1024 / 16 | 40.773 +/- 23.1878 | 33.453 +/- 21.3649 | 7982.59 / 3616.41 |

The fine-cell 256-collider mean improves about 66 times and allocation falls
64.7%; the after distribution is noisy (median 29.306 ms), so preserve the full
samples. Coarse timing intervals overlap; do not claim those small differences
as speedups. An additional after-only 1024-collider unit-cell row completes at
123.281 +/- 11.1799 ms with 65,589.79 KB/op. It has no matched before result.
Fine-cell allocations scale approximately fourfold per fourfold count increase
over these three after rows, instead of following the highest global ID in every
voxel. Artifacts: `artifacts/grv-benchmark-023/cold-before`, `cold-verified`.

**Reproduce:** Set `$env:UseLocalLsfStack = 'true'` and
`$env:DOTNET_PROCESSOR_COUNT = '2'`, use BelowNormal priority and one heavy
workload. Build with
`dotnet build Gravitas.slnx -c Release -p:UseLocalLsfStack=true -m:1 -p:BuildInParallel=false`.
Run `cold-circle-registration --filter '*' --launchCount 1 --warmupCount 3 --iterationCount 8 --affinity 3 --exporters json`
through the compiled benchmark DLL. The historical before capture filters out
1024 unit-cell colliders. Warmed stage selections use
`circle-partition-maintenance --filter '*ColliderCount: 64, CellSize: 1)*' '*ColliderCount: 1024, CellSize: 16)*' --launchCount 2 --warmupCount 5 --iterationCount 15 --iterationTime 250 --affinity 3 --exporters json`.
The integrated control uses
`circle-contact-simulation --filter '*ResetAndFullSimulationStep*Diagonal*'`
with the same warmed job settings. Hardware is an i7-9700K on Windows 11,
.NET SDK 10.0.302 / runtime 8.0.29, BenchmarkDotNet 0.15.8. Both source stacks
use FixedMathSharp `00a38bd`; Gravitas before is `05ddcac`, after is the
partition-hardening change committed as `f23d836`.

**Grounding decision and remaining scope:** Preserve automatic ground probing
when contact normals reject support. The paired above-body targets do not provide
ground, and zero gravity or a rejected response normal does not prove that the
probe can be omitted. Existing public support queries differ in allowed phases,
tiny probes, compound-normal filtering and physical-pair policy; replacing the
automatic sweep with them would change behavior. The remaining ground work uses
the existing exact sweep owner. Do not remove GridForge synchronization or add
another cache based on sampled attribution alone. The fresh full-step profile
and warmed stage results below keep the remaining work explicit. Closure uses
published costs and predictable scaling, without a fixed-Hz target. Runtime mass
mutation discovered in the awake review is independently tracked as GRV-Issue-089.

**Matched warmed stages:** The translated row includes pose reset plus coverage
refresh; retained distribution deliberately keeps processed pair keys and skips
repeated narrow phase/response. Forced probes check the same above-body scene
without a complete simulation step. Every row reports 0 B/op. These are separate
stage workloads, not additive components of the full-step measurement.

| Stage / total colliders / XZ cell edge | Before mean +/- error (us) | After mean +/- error (us) |
| --- | --- | --- |
| Translated refresh / 64 / 1 | 763.6 +/- 15.00 | 665.1 +/- 13.88 |
| Retained distribution / 64 / 1 | 1340.9 +/- 11.89 | 1268.5 +/- 18.01 |
| Forced automatic probes / 64 / 1 | 565.0 +/- 21.49 | 523.1 +/- 2.58 |
| Translated refresh / 1024 / 16 | 2678.6 +/- 35.39 | 2605.6 +/- 11.46 |
| Retained distribution / 1024 / 16 | 401.8 +/- 3.62 | 399.0 +/- 2.45 |
| Forced automatic probes / 1024 / 16 | 1638.7 +/- 70.66 | 1467.9 +/- 7.71 |

Fine-cell translated refresh improves 12.9% and retained distribution 5.4%.
Coarse distribution intervals overlap. Grounding benefits from the partition
storage change; its physical acceptance rules and exact sweep are unchanged.
Raw results are in `artifacts/grv-benchmark-023/maintenance-before`,
`maintenance-verified`, and `distribution-complete` (the final distribution
confirmation after helper deduplication).

**Integrated control:** `PairCount` here means independent dynamic/static
pairs, twice as many total colliders; XZ cells have edge 16.

| Diagonal pairs | Before full step mean +/- error (ms) | After full step mean +/- error (ms) | Allocation |
| --- | --- | --- | --- |
| 64 | 1.6085 +/- 0.11412 | 1.548 +/- 0.1146 | 0 B/op |
| 1024 | 35.9930 +/- 0.46672 | 35.573 +/- 0.3858 | 0 B/op |

These intervals overlap. Intermediate after captures range from 35.21 to
35.50 ms for 1024 pairs; do not turn the small point-estimate difference into a
stable full-frame speedup claim. Short-iteration warnings and bimodality remain
in the raw distributions. Before results are in
`artifacts/grv-benchmark-022/after`; final controls are in
`artifacts/grv-benchmark-023/fullstep-complete`.

The final actual-iteration-only EventPipe capture still attributes 31.0%
inclusive to grounding refresh, 19.3% to generic swept-circle detection and
12.5% to circle sweep geometry. Partition distribution accounts for 16.4%
inclusive, with voxel partition lookup at 11.0% exclusive. These are overlapping
sampled paths, not measured stage durations or proof of a lock bottleneck.
The complete contact owner contributes 4.9% inclusive in this sample. Capture
with the integrated 1024-pair filter, `--profiler EP --launchCount 1 --warmupCount 3 --iterationCount 5 --iterationTime 500`;
artifacts are `fullstep-profile-complete` and
`fullstep-profile-complete-actual-only.json` under the same directory.

**Partition-phase validation:** Final local-stack Release/ReleaseLean solution builds cover both
`netstandard2.1` and `net8.0`; all 4401/4342 tests pass with no failures or skips.
Final OpenCover/Cobertura root counts are exactly 44,491/44,491 sequence points,
13,278/13,278 branches and 4,589/4,589 methods in Release; Lean has
44,489/44,489 points, 13,278/13,278 branches and 4,588/4,588 methods.
ReportGenerator also confirms zero uncovered lines, branches and methods.
DocFX passes with warnings as errors; API-site branding, repository links and
the workflow's local-link checks also pass. Logs, TRX and raw coverage are in
`artifacts/grv-benchmark-023/*-final-review`; rendered reports are in
`report-Release` and `report-ReleaseLean`. Regression coverage
includes empty-registry swap-back/reset/reuse, all-dimension high-ID storage,
sorted inspection, negative-ID validation, coverage deltas, retained identities,
mobility changes, awake transitions, foreign contexts and strict allocation
gates. Source review reports no remaining findings. No coverage exclusion changes
or upstream source edits were made. This is Windows source-stack evidence;
released-package validation remains deferred until the upstream release.

**Experiment refinement, 2026-10-02:** The shared FixedMathSharp distance solver
now evaluates unit-interval endpoint signs as `C`, `A + 2B + C`, `B`, and `A + B`
for `f(t) = A*t*t + 2B*t + C`. Widened sums replace general rational endpoint
products only on `[0, 1]`. Bounded intervals, discriminants, root construction,
rounding and final distance mapping retain the existing owner. Circle, sphere,
capsule caps and rounded-cylinder cap consumers share this change; no physics
policy or second math kernel moves upstream.

Gravitas also omits host-yaw materialization when a body-owned requested pose
replaces it, or when the caller only needs scale. Exact matrix composition,
planar decomposition, ancestry/reflection/shear admission and mixed slab height
are unchanged. Bodyless poses still capture host yaw. Tests exercise nonidentity
parented transforms, requested-pose ownership and invalid transform hierarchies.

The new `circle-automatic-grounding` control probes after positional response,
matching the full-step geometry. The paired circle remains an initial full-radius
probe overlap because correction retains slop, but its normal rejects support.
Rows after the first can find the preceding row's circle below them: setup checks
exactly `PairCount - 32` supported bodies and repeatable probe results. Independent
contact pairs do not imply independent support probes. The older maintenance
control resets poses into overlap and remains a separate unsupported workload.

Fresh matched radial controls use two launches, five warmups, fifteen 250-ms
iterations and the environment above. Error is half the 99.9% interval; all rows
report 0 B/op.

| Shared distance interval / scale | Before mean +/- error (ns) | After mean +/- error (ns) |
| --- | --- | --- |
| Circle / 1 | 3828.9 +/- 18.00 | 3304.82 +/- 17.454 |
| Sphere / 1 | 3866.7 +/- 30.14 | 3302.94 +/- 26.690 |
| Circle moving away / 1 | 539.1 +/- 3.72 | 87.88 +/- 1.661 |
| Sphere moving away / 1 | 576.6 +/- 7.16 | 120.68 +/- 1.388 |
| Circle / 100000 | 5455.9 +/- 27.68 | 4921.34 +/- 24.061 |
| Sphere / 100000 | 5514.1 +/- 69.26 | 4980.02 +/- 53.401 |
| Circle moving away / 100000 | 554.3 +/- 17.01 | 89.89 +/- 0.490 |
| Sphere moving away / 100000 | 569.5 +/- 5.26 | 122.01 +/- 0.208 |

Regular interval cost falls 9.7-14.6%; moving-away misses fall 78.6-83.8%.
The direct planar capture control measures 813.7 +/- 8.88 versus
674.4 +/- 9.17 ns at zero host yaw, and 1175.9 +/- 16.23 versus
792.9 +/- 5.11 ns at one radian (17.1% / 32.6% lower, 0 B/op).
For corrected automatic probes, the math-only 1024-pair control changes from
9929.9 +/- 250.65 us to 9233.5 +/- 168.92 us (7.0% lower). The 64-pair
411.2 +/- 2.77 us and 406.4 +/- 7.00 us intervals overlap. The math-only full
step also overlaps its baseline: 36.695 +/- 0.710 versus 35.980 +/- 0.938 ms.

With both refinements the fresh 1024-pair full-step control measures
34.469 +/- 0.3244 ms at 0 B/op. Its matched fresh baseline is
36.695 +/- 0.710 ms (6.1% lower point estimate); the committed partition-phase
capture was already faster at 35.573 +/- 0.3858 ms. Preserve baseline drift,
short-iteration warnings and multimodal distributions rather than treating a
single capture as a universal frame-time gain. The 64-pair combined result is
1.503 +/- 0.1040 ms and supports no small-scene speedup claim.
An independent two-launch confirmation with 500-ms iterations measures
34.404 +/- 0.5312 ms at 0 B/op, consistent with the final capture. It confirms
the after result under a longer iteration protocol; it is not a new matched
before/after pair.
The final corrected-pose 1024-pair probe confirmation measures
9005.3 +/- 71.88 us at 0 B/op (9.3% below its fresh baseline).

Raw results live under `artifacts/grv-benchmark-023/`: `refinement-math-before-verified`,
`refinement-math-after`, `refinement-corrected-before-verified`,
`refinement-corrected-math-after`, `refinement-full-before`,
`refinement-full-math-after`, `refinement-full-final`, `refinement-full-confirmation`,
`refinement-planar-control-verified`, and `refinement-corrected-final`. Failed setup/build
captures are preserved separately and supply no timing evidence. The initial
short full-step pilot reported 3.44 KB/op; the longer matched controls report
0 B/op and existing strict allocation gates remain authoritative.

**Refinement validation:** Both local-stack solutions build for `netstandard2.1`
and `net8.0` in Release/ReleaseLean with zero warnings/errors. Gravitas passes
4403/4344 tests; FixedMathSharp passes 4138/4117 core tests plus 49 Chronicler
tests per configuration, with no failures or skips. Exact OpenCover counts are:

| Repository / configuration | Covered sequence points | Covered branches | Covered methods |
| --- | --- | --- | --- |
| Gravitas / Release | 44496 / 44496 | 13280 / 13280 | 4589 / 4589 |
| Gravitas / ReleaseLean | 44494 / 44494 | 13280 / 13280 | 4588 / 4588 |
| FixedMathSharp / Release | 52675 / 52675 | 12138 / 12138 | 3935 / 3935 |
| FixedMathSharp / ReleaseLean | 52768 / 52768 | 12138 / 12138 | 3931 / 3931 |

No coverage exclusions or allocation gates were changed. All 32 raw final
benchmark GC records have zero allocation and collection counters. Independent
source/evidence review reports no actionable findings. Builds, TRX and coverage
are in `artifacts/grv-benchmark-023/refinement-<repository>-<configuration>-*`.
Rendered reports verify core/FluentAssertions from the core suite and Chronicler
from its owning suite (85/85 lines, 12/12 branches, 18/18 fully covered methods
in both configurations). Merging Chronicler's partial core dependency capture
into the complete core report lost three covered lines in the Lean aggregation;
the raw complete capture and core-only rendered control both remain fully covered.
The failed merged-format control is retained, without modifying raw visits.
ReportGenerator confirms zero uncovered lines, branches and fully covered methods
for every owning assembly/configuration; see `refinement-coverage-summary.json`.
All 463 upstream Short smoke cases have successful child exits and populated
statistics, including 62 zero-allocation finite-axis controls. The smoke uses
one launch, three warmups and three 10-ms iterations; its timings are not
performance evidence. Both DocFX builds pass with warnings as errors, and API
resources, branding, repository actions and local links pass. Logs and exports
are `refinement-math-smoke*` and `refinement-<repository>-docfx.log`.
FixedMathSharp must release before Gravitas's released-package revalidation;
this evidence uses Windows sibling source with `UseLocalLsfStack=true`.

**Prepared-query refinement (2026-10-03):** Closest/all-hit 2D services now pass
their admitted exact segment and representable length through the existing
detector and compound parts. Circle CCD reuses its admitted displacement after
exact endpoint addition. Raw entry points retain their rejection order;
normalization, authored support distance, and the saturating reverse-convex
path are unchanged. No new runtime type, math owner or upstream edit is needed.

Automatic 2D grounding reuses the closest-hit loops with the body's existing
pure acceptance policy. Only accepted aggregate collider witnesses enter the
distance/owner-ID reducer. Compound parts still reduce before support filtering;
ray candidate counts and static-style sweep exclusions are unchanged. This
removes the per-body hit collection, retained capacity and additional hit sort.
Diagnostics, support writes and callback lifetime validation still follow
selection. Public all-hit buffers and explicit `QuerySupport` keep their contracts.

Fresh controls start from Gravitas `ee613db` and FixedMathSharp `6789c09`, using
the same Windows/local-stack environment above. The requested CLI argument is
`--affinity 3`; BenchmarkDotNet records the job display as `Affinity=11` in
every capture. Preserve that command/display pair when reproducing the run.
The 1024-pair scenes contain 2048 colliders. Timings below are ms/op, with the
half-width of the 99.9% confidence interval; every row reports 0 B/op.

| Capture | Corrected-pose automatic probes | Full diagonal simulation step |
| --- | --- | --- |
| Fresh committed baseline | 10.3778 +/- 0.1907 | 39.3280 +/- 0.6776 |
| Prepared segment only | 10.1208 +/- 0.1868 | 38.2636 +/- 0.5886 |
| Prepared segment plus nearest accepted | 9.8572 +/- 0.2201 | 38.8105 +/- 0.7148 |

These use two launches, five warmups and fifteen iterations: 250 ms for probes,
500 ms for full steps. The probe baseline/length captures have short observed
iterations; length/combined distributions are multimodal. The length/combined
full runs also include the detection-only control (4.4819 +/- 0.0606 and
4.5939 +/- 0.0672 ms); the fresh full baseline selects only the full step.
Probe point estimates improve 5.0% from today's baseline, but neither the
isolated length nor selection increment establishes a standalone speedup.
Full-step intervals overlap, including the combined result's higher point
estimate than length alone. No stable full-step gain is claimed. These fresh
baselines are slower than the preceding phase; preserve that drift.

`circle-ground-query-selection` pairs collect/sort/filter against nearest
accepted selection at identical corrected poses, with prepared segment reuse
in both paths. Setup compares the complete witness for every body and checks
32/992 supported bodies at 64/1024 pairs. With the same two-launch job and
500-ms iterations, its ms/op results are:

| Pairs | Collect/sort/filter | Nearest accepted | Allocation |
| --- | --- | --- | --- |
| 64 | 0.4099 +/- 0.00675 | 0.4017 +/- 0.00612 | 0 B/op |
| 1024 | 9.2889 +/- 0.12977 | 9.1682 +/- 0.16019 | 0 B/op |

Intervals overlap in both sizes; the paired run also reports multimodality and
outliers, so retain the raw samples. Retain the existing-owner streaming design
for its removed per-body storage and hit sort, with exact witness parity and
no measured material regression; do not claim an isolated selection speedup.
Raw JSON, logs and summaries are under `artifacts/grv-benchmark-023/` as
`refinement3-ground-{before,length,stream}`, `refinement3-full-{before,length,stream}`,
`refinement3-selection-paired`, `refinement3-timing-summary.json` and
`refinement3-selection-summary.json`.

**Prepared-query validation:** Release/ReleaseLean solution builds cover both
targets with zero warnings/errors. All 4430/4371 tests pass without skips,
including 27 new prepared-query and automatic-probe regression cases.
Exact OpenCover counts are:

| Configuration | Covered sequence points | Covered branches | Covered methods |
| --- | --- | --- | --- |
| Release | 44492 / 44492 | 13284 / 13284 | 4595 / 4595 |
| ReleaseLean | 44490 / 44490 | 13284 / 13284 | 4594 / 4594 |

ReportGenerator also verifies zero uncovered lines, branches and fully covered
methods (56222/56220 lines, 16290 branches, 5405/5404 methods). Existing collector
exclusions and strict allocation gates are unchanged. The first complete capture
found a missing public all-hit geometric-miss branch after grounding stopped
using that path; a direct overlapping-bounds/missed-circle regression closes it.
The failed coverage control is retained separately. Final builds, TRX, raw
coverage and rendered reports use `refinement3-final-Gravitas-<configuration>-*`;
the exact rendered counts are in `refinement3-coverage-summary.json`.
All ten affected public 2D raycast/sweep Short smoke rows have populated
statistics, successful child exits and 0 B/op; short timings are not performance
evidence. Across the before/length/combined, paired and smoke captures, all 34
raw GC records have zero allocation/collection counters. Exports and validation
counts are `refinement3-query-smoke*` and `refinement3-benchmark-validation.json`.
Independent source, evidence and ponytail review found no remaining actionable issues.
Gravitas DocFX builds with warnings as errors and passes API resource, branding,
repository-action and local-link checks; log: `refinement3-Gravitas-docfx.log`.

**Further refinement (2026-10-04):** A fresh actual-iteration-only CPU profile
of the committed `6c0c2f0` workload still attributes 15.666% exclusive CPU to
GridForge typed voxel partition lookup. Callers divide between 2D positional
correction/repartition (43.02%, including fixture reset), planar sweeps (29.64%)
and collider distribution (27.33%). Automatic 2D grounding contributes 27.072%
inclusive; complete shape capture contributes 14.712%, including 7.517% in the
forward-scaled planar transform. These shares overlap and are not independent
frame costs. `refinement4-full-profile-actual-only.json` retains the attribution.
That profiled child reports 24,624 raw allocated bytes over 12 operations;
its cause was not attributed. Preserve this exception separately from the
successful repeated allocation gates rather than assigning it to instrumentation.

Centered 2D collider snapshots repeatedly transform zero local coordinates.
FixedMathSharp's existing wide transform now returns the exact origin when both
local point and displacement are zero, before constructing rotated wide
numerators. This is policy-neutral arithmetic, applies through both public
forward-scaled overloads and preserves zero/mirrored/extreme scales and full
origin/yaw ranges. Nonzero input keeps the existing fused arithmetic and final
rounding. Host scale admission and snapshot ownership remain unchanged.

With two launches, five warmups and fifteen 500-ms iterations, the centered
256-operation microbenchmark changes from 1196.942 +/- 12.674 to
16.280 +/- 0.229 ns per transform; the nonzero control is
1536.411 +/- 15.456 / 1500.423 +/- 18.673 ns. Both report 0 B/op.
Two matched physics captures isolate only this math change:

| Capture | Corrected-pose automatic probes, ms | Full 1024-pair diagonal step, ms |
| --- | --- | --- |
| Initial before | 9.478350 +/- 0.121474 | 36.543975 +/- 0.468481 |
| Initial after | 9.963933 +/- 0.202656 | 34.223361 +/- 0.549934 |
| Repeat before | 9.926890 +/- 0.167635 | 39.031101 +/- 0.605361 |
| Repeat after | 9.767688 +/- 0.210000 | 33.779813 +/- 0.583872 |

All summaries report 0 B/op. One initial after grounding child reports
24,624 raw allocated bytes over 53 operations despite the final-launch zero
summary; both repeat captures have exact zero raw bytes/collections in every
child. Preserve the initial exception rather than inferring allocation from
the last-launch summary. Both full-step pairs improve, but the before values
vary materially; do not collapse them into one precise speedup percentage.
Automatic grounding does not perform this transform work. Its repeat intervals
overlap, preserving the control after the initial shift. Logs and JSON are
`refinement4-transform-{before,after}`, `refinement4-{before,math-after}` and
`refinement4-math-repeat-{before,after}`.

The GridForge experiment resolves the existing exact type key directly through
the provider while holding the same voxel monitor. Typed materialization,
including boxed value copies, stays inside the lock. `HasPartition` and default
lookup reuse this owner. Reset callbacks can clear or pool a still-published
payload, so an unlocked positive-hit shortcut would weaken the current contract.
No cache or synchronization redesign is introduced. Initial first-slot lookup
means change from 19.550/19.555 ns to 18.682/18.516 ns at one/256 voxels, but the
generic provider control also shifts. Native inlining removes a generic call
while growing the voxel machine-code body from 191 to 416 bytes. A fresh
runtime-only toggle confirms retrieval at 256 voxels, 19.6266 +/- 0.2135 to
18.6804 +/- 0.1832 ns; presence checks, 20.0508 +/- 0.2773 to
18.5102 +/- 0.2138 ns; and default lookup, 19.4804 +/- 0.2300 to
18.9223 +/- 0.2364 ns. The untouched type-key control remains
2.0077 +/- 0.0230 / 2.0146 +/- 0.0224 ns. The full 1024-pair step remains
34.0187 +/- 0.4391 / 33.9997 +/- 0.4845 ms and corrected-pose probes remain
9.7303 +/- 0.1491 / 9.7157 +/- 0.1252 ms, with overlapping intervals.
Retain the small lookup improvement; no measurable full-step or grounding gain
is claimed. All 24 child records in these matched micro/consumer captures have
zero raw allocated bytes and collections, alongside 0 B/op summaries.
The controls use the same two-launch/five-warmup/fifteen-500-ms-iteration job
and CLI `--affinity 3`. Evidence is `refinement4-grid-repeat-{before,after}`
and `refinement4-grid-consumer-{before,after}`. GridForge tracks the coordinated experiment as
`GF-Benchmark-013`. Its benchmark runner also propagates the source-stack build
properties explicitly, preserving Release/Lean and CLI job settings after two
reproduced generated-child build failures.

**Rejected 3D grounding experiment:** Streaming accepted supports through the
existing ray/sphere traversal removed the all-hit buffer and sort, while
separately preserving raw diagnostic minima/counts. Characterization confirmed
the existing support policy, compound-owner witnesses, mesh counters and
callback order. However, common one-target ray probes remained slower after
readonly-reference/inlining changes, separate private value-type reducers,
disabled-diagnostic gating and collider-only eligibility checks. Public closest
controls recovered, but sparse automatic grounding did not. The production
grounding and query workers have been restored to `6c0c2f0`.

Representative matched results use two launches, five warmups, fifteen 500-ms
iterations and CLI `--affinity 3`. Errors are 99.9% confidence half-widths:

| 64-probe workload | Existing implementation, microseconds | Rejected final candidate, microseconds |
| --- | --- | --- |
| Ray, one target, unsupported | 179.834 +/- 2.892 | 194.275 +/- 3.036 |
| Ray, one target, supported | 259.449 +/- 3.627 | 283.795 +/- 5.292 |
| Public closest ray control | 169.820 +/- 2.688 | 167.119 +/- 4.718 |
| Ray, eight targets, unsupported | 759.658 +/- 12.927 | 597.001 +/- 9.941 |
| Ray, eight targets, supported | 838.863 +/- 12.895 | 686.102 +/- 10.794 |

The one-target unsupported/control baselines are the old-runtime control repeat;
supported and dense baselines are the initial old-runtime capture. Dense ray
grounding improves about 18-21%, but repeated sparse ray grounding regresses
about 8-10% with disjoint intervals. Sphere results do not establish a consistent
dense-case gain; public closest-sphere control intervals overlap. This is a poor
default tradeoff. All 20 children in the rejected final candidate have zero raw
allocated bytes/collections and successful exits. Evidence retains
`refinement4-ground3d-before-boolean`, `refinement4-ground3d-control-repeat-before`,
the superseded variants, and `refinement4-ground3d-guarded`. The ignored
`refinement4-ground3d-rejected-runtime.patch` preserves the rejected runtime diff.

The retained public-only `ground-probe-selection3-d` fixture checks 64 identical
probe workloads across ray/sphere, one/eight targets and supported/unsupported
cases. Near raw hits may be physically rejected while the far floor is accepted.
Setup checks complete all-hit witnesses, raw diagnostics and work counters;
public closest-query rows control traversal cost. Its 16 cases and 37 public
regression cases characterize the unchanged implementation, including
disabled-to-enabled diagnostic parity. A Boolean mode parameter fixes an initial
generated-build enum-reference failure without changing runtime references;
failed captures remain separate from valid results.

**Final retained-source validation:** All builds and tests use
`UseLocalLsfStack=true`, in Release and ReleaseLean, with no failed or skipped
tests. Solution builds cover both library target frameworks with zero
warnings/errors. Owning raw OpenCover counts are:

| Owner | Tests, Release / Lean | Sequence points, Release / Lean | Branches, each | Methods, Release / Lean |
| --- | --- | --- | --- | --- |
| FixedMathSharp core/FluentAssertions | 4140 / 4119 | 52678 / 52771 | 12142 | 3935 / 3931 |
| GridForge | 1171 / 1171 | 9333 / 9333 | 4171 | 1145 / 1145 |
| Gravitas | 4467 / 4408 | 44492 / 44490 | 13284 | 4595 / 4594 |

Every listed point, branch and method is covered. FixedMathSharp's Chronicler
suite adds 49 passing tests per configuration and an independently filtered
100% report. Rendered reports confirm 100% lines, branches and fully covered
methods without changing exclusions: FixedMathSharp/FluentAssertions and
GridForge match the counts above; Gravitas' report aggregation totals
56222/56220 lines, 16290 branches and 5405/5404 methods, all covered. Preserve
raw and rendered counts separately. Evidence is
`refinement4-coverage-summary.json`, `refinement4-<owner>-<configuration>-*`
and `refinement4-retained-Gravitas-<configuration>-*`. Candidate-era captures
remain historical evidence, not validation of the restored runtime.

All 30 lookup smoke cases, one Lean generated-runner Dry case, five existing
3D all-hit query cases and all 16 retained grounding fixture cases complete
with successful child exits and exact zero raw allocated bytes/GC collections.
The Lean runner retains its configuration and all three source-stack child
properties. `refinement4-benchmark-validation.json` audits populated statistics,
summary allocation and every raw child record, retaining the documented initial
math-capture exception. Independent source, evidence and ponytail review found
no actionable issues. These local source-stack results precede release-package
validation; release FixedMathSharp first.
All three DocFX builds pass with warnings as errors, API resources, branding,
repository actions and local links verified. Logs are
`refinement4-<owner>-docfx.log`.

A fresh actual-iteration-only profile after the retained upstream changes
attributes 16.151% exclusive sampled CPU to typed voxel lookup and 12.626%
exclusive / 20.006% inclusive to 2D collider distribution. Automatic 2D grounding
is 31.764% inclusive. The centered-transform arithmetic frame is effectively
removed. These overlapping attribution shares are not matched timing gains;
the profile child has zero raw allocation/collections. Evidence is
`refinement4-final-profile-actual-only.json` and its trace capture.

**Refinement 5 (2026-10-04), retained and ready for review:** Pure 2D candidate gathering now
visits partition membership directly instead of sorting partitions and copied
IDs before deduplication. The final candidate collider-ID sort remains, keeping
the narrow-phase/hit reduction order stable. No public result ordering changes.
Three-dimensional ray/sweep partition gathering already has direct membership
traversal; the separate planar candidate-index rebuild still uses sorted member
copies, so those paths must not be conflated.

Matched query controls use two launches, five warmups and fifteen 250-ms
iterations, CLI `--affinity 3`, the local source stack and environment above.
Errors are half of the 99.9% intervals; every summary reports 0 B/op:

| Query-gather control | Before mean +/- error | After mean +/- error |
| --- | --- | --- |
| Full diagonal step, 64 pairs | 1.507 +/- 0.2240 ms | 1.350 +/- 0.1643 ms |
| Full diagonal step, 1024 pairs | 32.233 +/- 0.3401 ms | 31.567 +/- 0.3489 ms |
| Corrected automatic probes, 64 pairs | 407.6 +/- 7.18 us | 394.9 +/- 5.53 us |
| Corrected automatic probes, 1024 pairs | 9297.6 +/- 124.25 us | 9111.8 +/- 138.72 us |

Full-step and 1024-probe intervals overlap; this capture does not establish a
stable full-step gain. The paired selection control preserves both complete
witness paths; its 1024 nearest-accepted row is 8951.9 +/- 233.13 versus
8819.0 +/- 404.23 us, also overlapping. Retain the lean gather for removed
redundant work and ordering parity; final correctness and coverage gates pass. Short
observed iterations, outliers and multimodality remain in the raw capture and
audit; do not discard them when interpreting the reported intervals.

The density probe replays compiled admission/canonical owners outside timing.
At 1024 diagonal pairs, unit cells produce 123904 active partitions, 123904
dynamic memberships, 57344 static memberships and 57344 emitted links; canonical
ownership rejects 56320, leaving 1024 accepted pairs. Sixteen-unit cells produce
1024 active partitions and 3969 links, of which bounds reject 2945, also leaving
1024 accepted pairs. At 256 pairs, unit cells produce 30976 active partitions
and 14336 links; canonical ownership rejects 14080, leaving 256 accepted pairs.
Sixteen-unit cells produce 256 active partitions and 945 links, of which bounds
reject 689, leaving the same 256 pairs. The 64-pair cases produce 7744 fine-cell
partitions and 3584 links, or 64 coarse-cell partitions and 189 links; canonical
ownership and bounds reject 3520 and 125 links respectively. All 12 final probe
cases pass witness and candidate checks. Fine-cell memberships and links scale
exactly with pair count in this fixture. These are sparse local memberships replicated over coverage, not
dense collision neighborhoods. Known correction counts are fixture invariants,
not instrumented counts of production correction calls.

Bounds-first pure 2D rejection is retained: disjoint bounds skip
costly physical admission before canonical ownership, while bounds-accepted
pairs retain the existing physical filters. Six matched full-step controls use
the same job and 0 B/op summaries. Representative means +/- errors are
200.219 +/- 1.825 -> 194.252 +/- 1.330 ms for 1024 diagonal pairs/unit cells,
31.542 +/- 0.207 -> 29.849 +/- 0.708 ms for 1024 diagonal pairs/16-unit cells,
and 23.902 +/- 0.467 -> 22.804 +/- 0.162 ms for 1024 axis pairs/16-unit cells.
Do not copy this gate into 3D: retained pair culling/lifecycle rules require a
different admission boundary. Public filtering, notifications and stale-pair
cleanup must retain their dimensional contracts.

A second runtime-only bounds-gate toggle preserves the query-gather change in
both versions and repeats the same six full-step rows. Representative first
and repeated captures are:

| Bounds-first control | Initial before / after, ms +/- error | Repeat before / after, ms +/- error |
| --- | --- | --- |
| 1024 diagonal pairs, unit cells | 200.219 +/- 1.825 / 194.252 +/- 1.330 | 201.122 +/- 1.795 / 200.645 +/- 4.248 |
| 1024 diagonal pairs, 16-unit cells | 31.542 +/- 0.207 / 29.849 +/- 0.708 | 30.448 +/- 0.627 / 29.967 +/- 0.682 |
| 1024 axis pairs, 16-unit cells | 23.902 +/- 0.467 / 22.804 +/- 0.162 | 23.718 +/- 0.529 / 23.333 +/- 0.465 |

The repeat intervals overlap in these rows; initial gains are not stable
speedup evidence. Retain the cheap exact disjoint rejection for avoiding
unnecessary admission work, with no measured material regression and behavior
parity. Final correctness and coverage gates pass. Small-scene baseline drift further limits
attribution. Repeat artifacts are `refinement5-admission-repeat-{before,after}`.
The complete fixture now exposes 36 rows: 64/256/1024 pairs, Axis/Diagonal/Rotated,
one-/16-unit X/Z cells and two measured methods. All 36 smoke rows pass.
The final nine-row full-step count-scaling matrix uses two launches, five
warmups and fifteen 250-ms iterations, CLI `--affinity 3` (displayed as `11`),
and the local source stack. Every row reports 0 B/op:

| Geometry / X/Z cell size | 64 pairs, ms +/- error | 256 pairs, ms +/- error | 1024 pairs, ms +/- error |
| --- | --- | --- | --- |
| Axis / 16 | 0.9574 +/- 0.0537 | 3.9249 +/- 0.0265 | 23.0800 +/- 0.1873 |
| Diagonal / 16 | 1.1440 +/- 0.0105 | 5.5369 +/- 0.0864 | 30.0562 +/- 0.5735 |
| Diagonal / 1 | 9.7818 +/- 0.1831 | 45.0489 +/- 0.4199 | 201.9929 +/- 4.3606 |

Full-frame cost is not uniformly linear. Retained distribution sorts active
partitions with `SortInPlace`, an O(P log P) operation required for deterministic
callback order. The timings are consistent with an increasing ordered working
set alongside covered memberships; this source-based explanation is not a
profiled attribution of the measured scaling. Fine-cell coverage remains a
substantial configuration cost. Evidence is `refinement5-final-scaling`.

An early accepted-key `Contains` before canonical ownership was tested and
rejected. Despite 56320 potential repeated-key hits in the fine 1024 diagonal
probe, the measured full step regresses from 194.252 +/- 1.330 to
236.215 +/- 2.768 ms, a 21.60% regression with disjoint intervals.
Coarse axis also regresses, 22.804 +/- 0.162 to
24.146 +/- 0.114 ms. Preserve these observations; avoid an extra hot-path hash
probe based only on potential skip counts. No runtime early-Contains candidate
is retained.

Evidence: `artifacts/grv-benchmark-023/refinement5-query-{before,after}`,
`refinement5-density-probe/density.json`, `refinement5-admission-{before,after}`
and `refinement5-dedup-after`, plus the admission repeats, final scaling and
smoke captures. `refinement5-audit-captures.py --include-final` passes;
`refinement5-capture-audit.json` verifies 10 captures, 96 positive rows and 151
successful children. All 151 raw GC records contain exactly zero allocated
bytes and Gen0/1/2 collections, all memory summaries are zero, and every child
uses the local-stack build flags. Small-scene baseline drift prevents claiming
a large portable gain from the initial admission capture. Revisit 3D streaming only
after profiling explains its sparse-ray regression. Released-package
validation remains a later release gate for the coordinated upstream changes.

**Refinement-5 final correctness/coverage gates:** Local-stack Release and
ReleaseLean solution builds cover `netstandard2.1` and `net8.0` with zero
warnings/errors. Release passes **4473 tests** and Lean **4414**, with no failures
or skips. Raw and rendered coverage independently retain 100% coverage:

| Configuration | Raw sequence points | Raw branches | Raw methods | Rendered lines | Rendered branches | Rendered fully covered methods |
| --- | --- | --- | --- | --- | --- | --- |
| Release | 44495 / 44495 | 13286 / 13286 | 4596 / 4596 | 56225 / 56225 | 16292 / 16292 | 5406 / 5406 |
| ReleaseLean | 44493 / 44493 | 13286 / 13286 | 4595 / 4595 | 56223 / 56223 | 16292 / 16292 | 5405 / 5405 |

Evidence is `refinement5-final-Gravitas-{Release,ReleaseLean}*`. Upstream edits
in this pass are documentation-only; the previously committed FixedMathSharp
and GridForge changes retain their recorded full owning-suite coverage.
The final nine-row scaling capture, 36-row circle fixture smoke and five public
3D query controls all pass. The 41 short smoke rows verify execution, contracts
and allocation behavior; they do not establish timing improvements. The joint
audit includes these final captures. Gravitas, FixedMathSharp and GridForge
DocFX gates pass with warnings as errors, zero warnings/errors and verified
local API links, resources, branding and repository actions; evidence is
`refinement5-doc-gates.ps1` and its logs.

**Closure decision, 2026-10-04:** Close this signal under the selected
predictable-scaling/published-cost criterion with the final captures, audits
and correctness/coverage gates passed. Retain the measured ownership/storage/math
repairs and lean query gathering; retain exact cheap bounds rejection without
claiming a stable full-step speedup. Reject the regressing early duplicate hash
probe and streamed 3D grounding. Fine-cell coverage has a high configuration
cost that must remain published with counts, geometry and uncertainty; closure
does not certify a fixed-Hz budget. No speculative optimization or new cache is
required merely to close the signal. Reopen with a concrete workload regression,
unexpected scaling or a host-specific performance requirement. Released-package
validation remains a future release gate.

### GRV-Benchmark-022 — Exact Pure-2D Circle Contact Cost

**Discovered:** 2026-09-30 while implementing GRV-Issue-083.  
**Status:** Closed 2026-10-02: shared-owner optimization is committed upstream;
retain the exact contact path under the no-further-change decision below.
Larger-scene scaling is documented separately in closed GRV-Benchmark-023.  
**Owner:** FixedMathSharp's existing centered-capsule relation with two zero
core lengths; Gravitas forwards complete contact anchors, normal, depth and clamp.

Replacing saturated scalar circle arithmetic fixes large-coordinate false hits,
tiny-distance underflow and incorrect depth/clamping. However, the complete
query adds substantial cost even to ordinary cases where the old result was
correct. These fixtures preserve the same geometry and measured dispatcher
before/after; setup verifies classification, normal, depth and clamp, not exact
anchor-representation equivalence with the old rounded-offset implementation.

Windows 11 / i7-9700K, .NET 8.0.29, SDK 10.0.302; two launches, five warmups,
fifteen 250 ms iterations, CPU mask 3, BelowNormal launcher and
`DOTNET_PROCESSOR_COUNT=2`, with one heavy workload at a time. Values are
**microseconds per direct query**, not simulation frames. All rows allocate
**0 B/op**. Error is the half-width of the 99.9% confidence interval.

| Geometry | Previous scalar path | Initial complete query | Optimized complete query | Optimized error |
| --- | ---: | ---: | ---: | ---: |
| Axis | 0.06659 | 5.4363 | 0.9125 | 0.00248 |
| Coincident | 0.04828 | 2.6035 | 0.6277 | 0.00371 |
| Diagonal | 0.11101 | 4.3406 | 1.4513 | 0.00853 |
| Large axis | 0.10016 | 5.4263 | 0.9548 | 0.00236 |
| Rotated | 0.06712 | 5.8454 | 1.6113 | 0.00974 |
| Bounds-admitted separation | 0.02542 | 0.9027 | 0.2182 | 0.00077 |

The initial slowdown was 35.5-87.1x. Shared-owner work removes 66.6-83.2% of
that complete-query time, but the remaining cost is still 8.6-24.0x the old scalar
path. This is a substantial reduction, not scalar-cost parity or an accepted
full-simulation budget. Retain the raw distributions, including a baseline
bimodality warning; the final Gravitas run reported no multimodality warning.
The old path is not a full-domain correctness target. In particular, comparing
an exact squared distance alone would not repair normal, depth or anchor terms.

**Shared-owner optimization, 2026-09-30:** EventPipe sampling of the complete
circle query's Axis fixture put approximately 45% of inclusive query samples in
depth reduction and 29% in its product square root; these overlap and are not additive. Parameter
construction, local offsets, normalization and retained radial terms added work.
FixedMathSharp now represents point cores directly over denominator one, reuses
its narrower exact root, classifies conceptual depth before rounding, and skips
irrelevant axis rotations/zero offsets. Cardinal normalization skips roots and
division. Both dimensions share the existing residual owner, whose retained low
word is computed directly with equivalent unchecked integer arithmetic. No
second Gravitas solver, approximation, cache, public API or dependency was added.

Matched upstream `zero-core-contact` runs use the same settings and geometry as
above, plus a nonzero-core capsule control. Values remain **microseconds per
query**, with **0 B/op** throughout:

| Geometry | Circle before | Circle optimized | Sphere before | Sphere optimized |
| --- | ---: | ---: | ---: | ---: |
| Axis | 5.3077 | 0.9135 | 9.4406 | 1.1843 |
| Coincident | 2.6306 | 0.6104 | 6.6275 | 0.7644 |
| Diagonal | 4.4942 | 1.4536 | 8.7157 | 1.7991 |
| Large axis | 5.6523 | 0.9243 | 9.8707 | 1.2599 |
| Rotated | 6.1178 | 1.6234 | 8.9603 | 1.7449 |
| Separation | 0.9026 | 0.2007 | 2.3985 | 0.1489 |
| Nonzero capsule control | 6.0087 | 3.8004 | 10.1398 | 7.7200 |

The axis-sphere distribution is bimodal; its 99.9% interval is
`1.1843 +/- 0.03092 us`, still well separated from the old result. Full upstream
distributions and logs are in FixedMathSharp's `artifacts/grv083/before` and
`after`; the baseline runtime is `787afae`, with the same benchmark harness.
These direct geometry results are not full-step or cross-platform measurements.
This optimization is separate from the oblique polynomial costs in
GRV-Benchmark-020/021.

**Reproduce:** Build the Release benchmark project with
`-p:UseLocalLsfStack=true -m:1 -p:BuildInParallel=false`, then run
`dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll circle-contact --launchCount 2 --warmupCount 5 --iterationCount 15 --iterationTime 250 --affinity 3 --exporters json --artifacts artifacts/grv083/optimized`.
Baseline runtime source is Gravitas `b8b4115` with FixedMathSharp `787afae`;
the `after` capture adds only the Gravitas zero-core delegation; `optimized`
also includes the upstream changes described above. Reports and raw measurements
are in `artifacts/grv083/before`, `after` and `optimized`; build logs are alongside.

**Shared normalization follow-up, 2026-10-02:** The current local stack already
contains the first optimization above. Actual-iteration EventPipe samples put
local radial-feature work at 41.7% of the diagonal query and 48.5% of the rotated
query, with normalization at 27.6% and 21.5% respectively; inclusive shares
overlap. Depth now contributes only 4.1% and 2.4%. FixedMathSharp's existing
`Vector2d.GetScaleNormalized` and `Vector3d.GetScaleNormalized` rescaled their
already bounded vectors a second time through `GetScaledMagnitude`.

Both helpers now take the square root of the same rounded component squares
directly. The dominant scaled component rounds to exactly +/-One, including
`MinValue` whose excess over the saturated absolute-value scale is below half
a raw unit. The removed divisions and multiplication were therefore identities;
sum order, square-root input, rounding, zero-input preconditions and all retained
anchor terms are unchanged. No new branch, arithmetic owner, circle kernel,
cache, public API or dependency was added. The upstream tests compare the prior
path across 4913 raw-edge triples and 2048 deterministic unequal-scale triples,
with independent BigInteger nearest-even ratio checks.

Matched direct queries use the same machine/settings as above. Before sources
are Gravitas `e4ebf1d` and FixedMathSharp `6a6a7e3`; after adds only the upstream
normalization change. Values are **microseconds/query**, all **0 B/op**:

| Geometry | Current-stack before | After | After error |
| --- | ---: | ---: | ---: |
| Axis | 0.9344 | 0.8944 | 0.00361 |
| Coincident | 0.6396 | 0.6309 | 0.00660 |
| Diagonal | 1.4753 | 1.3685 | 0.01076 |
| Large axis | 0.9574 | 0.9478 | 0.00973 |
| Rotated | 1.6330 | 1.5728 | 0.01852 |
| Separation | 0.2223 | 0.2191 | 0.00211 |

Diagonal improves 7.2% and rotated 3.7%. Coincident, large-axis and separation
intervals overlap; do not claim a measured gain there. The direct before capture
has a multimodality warning; preserve all distributions. The after capture has
none. Upstream sphere rows and a nonzero-core capsule control also pass their
geometry checks with 0 B/op; these are sanity controls, not a newly matched 3D
speedup comparison.

**Sustained-contact workload:** `circle-contact-simulation` runs 64 or 1024
independent dynamic/static pairs through the real context `Simulate` and
`LateSimulate` phases. Every invocation resets pose and motion, keeps sleep
disabled and retains pair state. Setup verifies expected normal/depth, actual
positional response, stable candidate/contact counts and repeated body state.
Explicit 16-by-1-by-16 cells and padded coverage isolate contact work from
fine-voxel fanout. This measures repeated penetration from zero initial velocity;
it is not a moving-impact, sliding or restitution workload.

Both full-step and direct-dispatcher attribution rows include body reset. The
full step includes partition refresh/distribution, contact response, grounding
and bookkeeping. Values below are **milliseconds/invocation**, all **0 B/op**:

| Pairs | Geometry | Full step before | Full step after | After error |
| --- | --- | ---: | ---: | ---: |
| 64 | Axis | 1.1483 | 1.1898 | 0.03229 |
| 64 | Diagonal | 1.5143 | 1.6085 | 0.11412 |
| 64 | Rotated | 1.4777 | 1.5043 | 0.13951 |
| 1024 | Axis | 29.4562 | 28.4869 | 0.74434 |
| 1024 | Diagonal | 36.8367 | 35.9930 | 0.46672 |
| 1024 | Rotated | 31.2055 | 31.4762 | 0.62095 |

Full-step changes are mixed, with overlapping intervals, bimodality in after
rows and short-iteration warnings. They do not establish an overall simulation
speedup or an accepted frame budget. The follow-up profile identifies grounding
and partition work as larger targets than contact anchors; see GRV-Benchmark-023.

**Closure decision, 2026-10-02:** Retain the exact shared contact owner and the
normalization improvement. Further circle-specific kernels, anchor shortcuts or
caches are not justified by the full-step profile. The premium over the
incomplete scalar predecessor remains reference evidence, not a parity target.
Revisit shared circle geometry when a representative host workload and explicit
budget identify it as a material bottleneck. Investigate the separately measured
partition/grounding scaling before accepting the large-scene frame cost.
This closes the direct-query investigation, not an acceptance of whole-frame
cost. GRV-Benchmark-023 has different owners and scaling failure modes across
partition rental, distribution and grounding; it remains a separate signal
even though the circle workload exposed it.

**Reproduce this follow-up:** Set `UseLocalLsfStack=true` and
`DOTNET_PROCESSOR_COUNT=2`, build Release with `-p:UseLocalLsfStack=true -m:1
-p:BuildInParallel=false`, then run each selection with `--launchCount 2
--warmupCount 5 --iterationCount 15 --iterationTime 250 --affinity 3 --exporters
json`, a BelowNormal launcher and one heavy workload. The direct before is
`artifacts/grv-benchmark-022/direct-before`, full-step before is `coarse-before`,
and matched after is `after`. Uninstrumented captures are the timing/allocation
evidence; `direct-profile`, `workload-profile` and `coarse-profile` retain separate
EventPipe traces and actual-iteration sampled-CPU analysis. The exploratory
unit-cell run is partial and must not be mixed into the matched table. Upstream
controls and focused test logs are in FixedMathSharp's
`artifacts/grv-benchmark-022`.

**Final validation, 2026-10-02:** With `UseLocalLsfStack=true`, both repositories'
full solution builds pass for `netstandard2.1` and `net8.0` in Release and
ReleaseLean with zero warnings/errors. Gravitas passes 4363/4304 tests and
FixedMathSharp passes 4125/4104 core tests respectively, plus 49 Chronicler
integration tests in each configuration; all have zero failures/skips. Each
repository/configuration retains exact 100% reachable line, branch and method
coverage. OpenCover reports no unvisited sequence points, branches or methods;
CI-style ReportGenerator gates independently report zero uncovered counts.
Raw collector XML, TRX, build/test logs and HTML/JSON summaries are under each
repository's `artifacts/grv-benchmark-022`, including `coverage-Release`,
`coverage-ReleaseLean`, `report-Release` and `report-ReleaseLean`. Independent
correctness and simplification review found no actionable issues. FixedMathSharp's
normalization change is committed as `00a38bd`; Gravitas benchmark/docs changes
remain pending review.

### GRV-Benchmark-012 — Mixed Discrete Broad-Phase Allocation At 32 Pairs

**Discovered:** 2026-07-19 **Closed:** 2026-08-04

The original `RotationalMovingPairCcdBenchmarks` mixed 3D-to-2D ShortRun was
already allocation-free at 1 and 8 pairs, while repeated 32-pair runs reported a
small, run-dependent `48 B/op` to `10 B/op`. Focused guards excluded CCD
preparation, search, response, handoff, reset, and completion and localized the
sample to mixed discrete partition refresh after CCD.

The current locally linked stack no longer reproduces the signal. Two
independent unchanged rotational ShortRuns reported `0 B/op` at 1, 8, and 32
pairs. The independent broad-phase check then found that
`MixedBroadPhaseBenchmarks` still called only `Simulate()` after mixed contact
work moved to `LateSimulate()` in June, so it had become an empty benchmark. The
repaired workload now:

- executes the complete `Simulate()` / `LateSimulate()` fixed-step contract;
- uses bodyless 3D triggers against dynamic 2D bodies so partitioning, candidate
  generation, narrow phase, and pair lifecycle stay active without solver-driven
  scene drift;
- rejects setup that produces no broad-phase candidates; and
- gives each row target-specific setup and cleanup instead of constructing all
  three worlds in every benchmark process.

Final ShortRun evidence:

| Method                             | Collider count |       Mean | Allocated |
| ---------------------------------- | -------------: | ---------: | --------: |
| SparseCandidateGathering           |             32 | `334.8 us` |     `0 B` |
| DenseCandidateGathering            |             32 | `497.8 us` |     `0 B` |
| RetainedPartitionCleanupAfterChurn |             32 | `817.4 us` |     `0 B` |
| SparseCandidateGathering           |          1,024 | `28.96 ms` |     `0 B` |
| DenseCandidateGathering            |          1,024 | `23.64 ms` |     `0 B` |
| RetainedPartitionCleanupAfterChurn |          1,024 | `84.33 ms` |     `0 B` |

The old 4,096-collider row created one monolithic dense voxel grid for a sparse
address-space workload. Even after target-specific setup, it exceeded `2.6 GB`
before the first timed operation. GridForge already provides sparse storage and
streamed multi-grid ownership for that world shape, so the routine benchmark now
retains the exact 32-pair threshold and a 1,024-collider stress point rather
than measuring an unrepresentative setup-memory ceiling.

**Resolution:** No production capacity hint or preallocation was added. The
original signal is absent under repeated end-to-end and independently corrected
broad-phase measurement, all retained rows are allocation-free, and the only new
findings were benchmark-harness defects corrected in the benchmark itself.
Release and `ReleaseLean` pass 3,930 and 3,875 tests respectively. Fresh
ReportGenerator evidence remains at 100%: 55,869/55,869 lines, 15,833/15,833
branches, and 5,321/5,321 methods. Both package configurations build for
`net8.0` and `netstandard2.1` without warnings.

### GRV-Benchmark-013 — Mesh Scale Rebuild Allocation

**Discovered:** 2026-07-28 **Closed:** 2026-08-03

**Initial evidence:** A refreshed focused ShortRun of
`MeshMassPropertyBenchmarks.UpdateNonUniformMeshScaleAndCalculateSurfaceInertia`
reported:

| Subdivision |        Mean |     Allocated |
| ----------: | ----------: | ------------: |
|           1 | `38.394 us` |      `0 B/op` |
|           8 |  `2.166 ms` |  `4,032 B/op` |
|          16 |  `8.822 ms` | `16,320 B/op` |

**RCA:** Triangle-BVH rebuilding, scaled face data, and surface mass properties
remain allocation-free. Convex support-tree preparation instead sorted every
non-leaf vertex range after every scale change through a retained reference
comparer. Each `Array.Sort(...)` call allocated 64 bytes; the subdivision-8 and
subdivision-16 trees have 63 and 255 non-leaf nodes, exactly accounting for the
measured totals.

**Resolution:** `PhysicsMesh` now builds its support-vertex partition once.
Subsequent scale candidates refit leaf bounds from that immutable partition and
branch bounds bottom-up into the existing prepared node buffer. Publication
still swaps complete committed/prepared node buffers transactionally. The second
support-index array, its publication swap, repeated sorting, and retained
construction comparer were deleted. Exact support selection and authored-order
ties are unchanged.

The unchanged command now reports:

| Subdivision |    Baseline | Confirmation |   Delta | Allocated |
| ----------: | ----------: | -----------: | ------: | --------: |
|           1 | `38.394 us` |  `37.755 us` | `-1.7%` |  `0 B/op` |
|           8 |  `2.166 ms` |   `1.994 ms` | `-7.9%` |  `0 B/op` |
|          16 |  `8.822 ms` |   `8.131 ms` | `-7.8%` |  `0 B/op` |

Gravitas passes 3,928 Release and 3,873 ReleaseLean tests. Coverage remains
55,869/55,869 lines, 15,833/15,833 branches, and 5,321/5,321 methods. The
focused plan and complete evidence are preserved in
[`2026-08-03-mesh-scale-rebuild-throughput-plan.md`](done/2026-08-03-mesh-scale-rebuild-throughput-plan.md).

Artifacts:

- `artifacts/benchmarks/2026-08-03-mesh-scale-rebuild-baseline`
- `artifacts/benchmarks/2026-08-03-mesh-scale-rebuild-topology-refit-first-pass`
- `artifacts/benchmarks/2026-08-03-mesh-scale-rebuild-topology-refit-confirmation`
- `tests/Gravitas.Tests/TestResults/coverage-analysis-mesh-scale-rebuild-20260803`

### GRV-Benchmark-009 — Replay Hash Collider-ID Churn Scaling

**Discovered:** 2026-07-05 **Closed:** 2026-07-05

**Initial evidence:** A focused replay-hash benchmark row,
`ReplayHashBenchmarks.ReplayHash3DChurnedIds`, created an 8x deleted-collider
history by registering and deactivating bodyless 3D static colliders, then
leaving only the final live tail active.

Initial command:

```powershell
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll replay-hash --filter "*ReplayHash3DChurnedIds*" --warmupCount 1 --iterationCount 3
```

Initial measurement on 2026-07-05:

| Row                                        |       Mean | Allocated |
| ------------------------------------------ | ---------: | --------: |
| `ColliderCount=64` live, 512 created IDs   | `119.0 us` |     `0 B` |
| `ColliderCount=256` live, 2048 created IDs | `483.9 us` |     `0 B` |

**RCA:** 2D and 3D services had separate collider ownership structures: compact
live lists, ID dictionaries, and manual next-ID/high-water counters. Replay
hashing walked the high-water ID range and emitted deleted-hole state, while
mixed replay hashing crossed 3D and 2D high-water ranges before checking whether
a mixed pair existed. That made deleted context-local allocation history part of
authoritative replay identity even though serialization treats context-local
collider IDs, service indices, partitions, and pair tables as runtime-owned
state.

**Resolution:** 2D and 3D collider registration now goes through a shared
registry backed by reusable `SwiftBucket` slots plus compact live iteration.
`-1` is the unregistered collider sentinel across dimensions; `0` is a valid
context-local collider ID. Runtime IDs remain lookup and pair keys, while
authoritative replay hashes traverse canonical live registration order and write
dense replay ordinals for collider, hierarchy, and pair identity. Deleted ID
holes, free-list ordering, and allocator history are excluded from authoritative
hashes, while registry peak counts remain cache diagnostics for
`AuthoritativeWithSolverCaches`.

Post-fix command:

```powershell
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll replay-hash --filter "*ChurnedIds*" --warmupCount 1 --iterationCount 3
```

Post-fix measurement on 2026-07-05:

| Row                                                  |       Mean | Allocated |
| ---------------------------------------------------- | ---------: | --------: |
| `replay-hash-3d-churned-ids`, `ColliderCount=64`     | `114.7 us` |     `0 B` |
| `replay-hash-2d-churned-ids`, `ColliderCount=64`     | `124.3 us` |     `0 B` |
| `replay-hash-mixed-churned-ids`, `ColliderCount=64`  | `124.3 us` |     `0 B` |
| `replay-hash-3d-churned-ids`, `ColliderCount=256`    | `489.2 us` |     `0 B` |
| `replay-hash-2d-churned-ids`, `ColliderCount=256`    | `537.9 us` |     `0 B` |
| `replay-hash-mixed-churned-ids`, `ColliderCount=256` | `550.9 us` |     `0 B` |

**Verification:** Added replay-hash tests proving deleted 3D, 2D, and mixed
collider churn and free-list ordering do not affect authoritative hashes, live
collider ordering still affects authoritative hashes, and steady-state replay
hashing remains allocation-free after churn. Added registry tests proving
reusable IDs, context-local lookup, compact service indices, and `-1` inactive
sentinels.

### GRV-Benchmark-008 — Pure 2D Response Position-Correction Repartition Allocation

**Discovered:** 2026-06-26 **Closed:** 2026-06-28

**Evidence:** During the physics material model validation pass, a focused
allocation guard for `CollisionResponse2D.Resolve(...)` initially measured a
stable `2712 B` allocation when the prepared manifold used non-zero depth and
the measured action reset velocity and resolved response in one pass. Re-running
the material solver path with zero penetration depth and measuring only the
prepared response call reported `0 B`, which pointed away from material
resolution and toward the 2D position-correction/repartition path.

**RCA 2026-06-28:** A dedicated guard reproduced the hot path by forcing
non-zero 2D position correction across GridForge voxel partitions. The first
focused repro measured `18,936 B` over four measured iterations. Gravitas kept
empty `PhysicsPartition2D` instances retained on old voxels, but when a moving
collider crossed into fresh voxels and the inactive partition pool was empty,
the service allocated new partition objects and first-use sparse sets instead of
retiring an empty retained partition for immediate reuse. After adding
retained-empty reuse, the repro dropped to `7,712 B`.

The remaining allocation was lower-stack metadata. GridForge's
`PartitionProvider` always allocated a `SwiftDictionary<Type, IVoxelPartition>`
for the first partition attached to a voxel. Physics partitions are commonly the
only partition type on a voxel, so this made every fresh voxel attach pay a
general dictionary allocation. After moving the common single-partition case
inline, the final `224 B` repro remainder came from eager `Type.Name` diagnostic
strings in `Voxel.TryAddPartition(...)` and `Voxel.TryRemovePartition<T>()`;
those names were only needed on failure but were built on the success path.

**Resolution 2026-06-28:** `GravitasCollision2DService`,
`GravitasCollisionService`, and `GravitasMixedCollisionService` now retire an
empty retained partition for immediate reuse when their inactive pool is empty.
GridForge's `PartitionProvider<TPartitionBase>` stores the first partition
inline, upgrades to `SwiftDictionary` only when a second concrete partition type
is attached, keeps multi-partition storage reusable after `Clear()`, and exposes
an internal allocation-free enumerator for voxel reset. `Voxel` now creates
partition type names only on error paths.

**Validation 2026-06-28:** Focused GridForge allocation guards pass for first
single-partition provider attach and voxel add/remove success paths. Focused
Gravitas guards pass for:

- 2D response position correction crossing partitions.
- 3D response position correction followed by collider repartition refresh.
- mixed 3D collider refresh crossing mixed partitions.

The `physics-2d --filter "*Resolve*" --job Short` benchmark smoke reports no
managed allocation across the selected 64-body and 1024-body 2D response rows.

**Touched files:**

- [GridForge `PartitionProvider.cs`](https://github.com/mrdav30/GridForge/blob/main/src/GridForge/Spatial/PartitionProvider.cs)
- [GridForge `Voxel.cs`](https://github.com/mrdav30/GridForge/blob/main/src/GridForge/Grids/Nodes/Voxel.cs)
- [GridForge spatial type tests](https://github.com/mrdav30/GridForge/blob/main/tests/GridForge.Tests/Spatial/SpatialTypes.Tests.cs)
- [GridForge voxel tests](https://github.com/mrdav30/GridForge/blob/main/tests/GridForge.Tests/Grids/Voxel.Tests.cs)
- `src/Gravitas/Core/2D/GravitasCollision2DService.cs`
- `src/Gravitas/Core/3D/GravitasCollisionService.cs`
- `src/Gravitas/Core/Mixed/GravitasMixedCollisionService.Partitioning.cs`
- `tests/Gravitas.Tests/CollisionHandling/CollisionResponse2DManifoldTests.cs`
- `tests/Gravitas.Tests/CollisionHandling/CollisionResponseInvariantTests.cs`
- `tests/Gravitas.Tests/MixedDimensions/MixedBroadPhaseTests.cs`

### GRV-Benchmark-005 — SwiftCollections Sort Hot-Path Allocation

**Discovered:** 2026-06-22 **Closed:** 2026-06-24

**Evidence:** During the post-SwiftCollections v5.1.0 Workstream 3 cleanup,
replacing Gravitas' local sort helpers with package
`SwiftList<T>.SortInPlace(...)` and `SwiftSparseSet.CopySortedKeysTo(...)`
caused the Release allocation guardrails to fail. The full suite reported
recurring allocations in 3D CCD, pure 2D CCD, pure 2D broad phase, and 2D query
tests. The isolated
`Physics2DQueryTests.RaycastAll_ShouldNotAllocateAfterWarmup` test reproduced
the issue at `128 B` after warmup.

**Resolution:** SwiftCollections owns the allocation-free sort primitive.
`SwiftList<T>.SortInPlace(...)` routes default ordering through the optimized
BCL default-comparer path, custom class/interface comparers through an
allocation-free introsort, and struct comparers through a no-boxing generic
introsort. `SwiftSortedList<T>` bulk-load, known-count `IReadOnlyCollection<T>`
range insertion, and `SetComparer(...)` use the same lower-stack helper without
recurring managed sort allocations. Gravitas removed
`src/Gravitas/Support/SwiftListSortUtility.cs` and calls
`SwiftList<T>.SortInPlace(...)` or `SwiftSparseSet.CopySortedKeysTo(...)`
directly from runtime ordering paths.

**Why it matters:** `SwiftCollections` is the lower-stack collection layer for
LSF. Its scratch-buffer sort APIs should be safe for deterministic physics hot
paths so consumers do not need local workarounds.

**Benchmark signal:** Short-run comparer benchmarks show the tradeoff:
`List<T>.Sort(custom class comparer)` remains faster but allocates `64 B/op`,
while `SwiftList<T>.SortInPlace(custom class comparer)` allocates `0 B/op`. For
struct comparers, `List<T>.Sort(struct comparer)` measured about `12.136 ms` at
`100000` integers with `88 B/op`; the Swift struct-comparer path measured about
`13.089 ms` with `0 B/op`, closing most of the CPU gap while preserving the
allocation contract.

**Touched files:**

- [SwiftCollections `SwiftList.cs`](https://github.com/mrdav30/SwiftCollections/blob/main/src/SwiftCollections/Collection/SwiftList.cs)
- [SwiftCollections `SwiftSortedList.cs`](https://github.com/mrdav30/SwiftCollections/blob/main/src/SwiftCollections/Collection/SwiftSortedList.cs)
- [SwiftCollections `SwiftArraySortHelper.cs`](https://github.com/mrdav30/SwiftCollections/blob/main/src/SwiftCollections/Utility/SwiftArraySortHelper.cs)
- [SwiftCollections `SwiftSparseSet.cs`](https://github.com/mrdav30/SwiftCollections/blob/main/src/SwiftCollections/Collection/SwiftSparseSet.cs)
- `src/Gravitas/Core/3D/GravitasCollisionService.cs`
- `src/Gravitas/Core/2D/GravitasPhysics2DService.cs`
- `src/Gravitas/Core/2D/GravitasCollision2DService.cs`
- `src/Gravitas/Core/Mixed/GravitasMixedCollisionService.cs`
- `src/Gravitas/Partitions/*/*Partition*.cs`

**Closure evidence:** SwiftCollections focused allocation guardrails and full
Release/ReleaseLean test suites pass, GridForge and Gravitas validate through
local project references, Gravitas Release/ReleaseLean allocation guardrails
pass, and the Gravitas simulation allocation benchmark smoke rows remain at
`0 B/op`.

### GRV-Benchmark-006 — Mixed Mesh Finite-Slab Triangle Scaling Signal

**Discovered:** 2026-06-23

**Status:** Closed 2026-06-24

**Evidence:** During mixed finite-slab reducer close-out review,
`MixedQueryBenchmarks` covered mesh target scaling mostly through collider
candidate count. The mesh fixtures were tiny triangle sets, so dense mesh
triangle candidate scanning, triangle clipping, and per-triangle reducer cost
could regress without a dedicated row making that cost visible.

**RCA 2026-06-24:** The mixed reducer itself was not hiding an obvious stronger
hot-path algorithm. A speculative sorted/lower-bound candidate pass improved
some dense-hit rows but regressed false-positive-heavy slabs, and a more
conservative hybrid pass still added cost without enough pruning. The final
mixed path keeps authored triangle scan order, tracks the best lower triangle
index directly, and uses the new benchmark/counter coverage as the guardrail.

The carryover pure 3D review did find a real mesh reducer bottleneck:
convex-source sweeps against concave mesh targets scanned exact triangle
reducers in raw candidate order. Ordering concave target triangle candidates by
a deterministic sweep lower bound lets the worker stop once the current best TOI
cannot be beaten by remaining triangles.

**Resolution 2026-06-24:**

- `GravitasQueryMixedService` exposes a context-owned
  `LastMeshTriangleCandidateCount` for mixed mesh-target sweeps.
- `GravitasQuery3DService`, `SweptSphereQueryWorker`, and
  `ConvexSweepQueryWorker` expose matching 3D mesh-triangle candidate counts.
- `MixedMeshTriangleScalingBenchmarks` measures dense and false-positive mixed
  mesh targets by triangle volume rather than collider count.
- `MeshQuery3DTriangleScalingBenchmarks` measures pure 3D swept-sphere mesh
  targets and convex-source sweeps against dense concave mesh targets.
- `ConvexSweepQueryWorker` sorts concave mesh target triangles by deterministic
  lower-bound TOI and authored triangle index, then exits once remaining
  triangles cannot beat the current best hit.

**Validation 2026-06-24:** Re-running
`mixed-mesh-triangle-scaling --filter "*TriangleMeshTarget*" -j Short -i`
reported predictable triangle-volume scaling with no recurring managed
allocation:

- Dense mixed target: `139.4 us` at `128` triangles, `514.2 us` at `512`
  triangles, and `2.031 ms` at `2048` triangles.
- False-positive mixed target: `136.6 us` at `128` triangles, `520.8 us` at
  `512` triangles, and `2.089 ms` at `2048` triangles.

Re-running
`mesh-query3-d-triangle-scaling --filter "*TriangleMeshTarget*" -j Short -i`
after the pure 3D carryover optimization reported:

- Swept-sphere dense mesh targets remain a tracked linear path: `550.1 us`,
  `2.145 ms`, and `8.638 ms` for `128`, `512`, and `2048` triangles.
- Convex-source sweeps against dense concave mesh targets improved from the
  pre-change baseline of about `12.465 ms`, `207.777 ms`, and `3.137 s` to
  `125.4 us`, `441.1 us`, and `1.658 ms` at the same triangle counts.

Focused mixed/3D query tests and the benchmark project build passed after the
change.

**Likely files:**

- `src/Gravitas/Queries/Mixed/GravitasQueryMixedService.cs`
- `src/Gravitas/Queries/3D/GravitasQuery3DService.Raycast.cs`
- `src/Gravitas/Queries/3D/GravitasQuery3DService.Circle.cs`
- `src/Gravitas/Queries/3D/Sweeps/ConvexSweepQueryWorker.cs`
- `src/Gravitas/Queries/3D/Sweeps/SweptSphereQueryWorker.cs`
- `tests/Gravitas.Tests/MixedDimensions/MixedQueryCcdTests.cs`
- `tests/Gravitas.Tests/Queries/GravitasQuery3DServiceSweepTests.cs`
- `tests/Gravitas.Benchmarks/Queries/MixedMeshTriangleScalingBenchmarks.cs`
- `tests/Gravitas.Benchmarks/Queries/MeshQuery3DTriangleScalingBenchmarks.cs`
- `tests/Gravitas.Benchmarks/Support/BenchmarkPhysicsScene.cs`

**Closure criteria:** Met. Benchmarks expose mesh finite-slab and 3D mesh sweep
cost by triangle candidate volume, not only collider count. Mixed keeps stable
authored triangle tie-breaks without speculative reducer overhead, and pure 3D
avoids the measured convex-source concave-mesh triangle-order bottleneck.

### GRV-Benchmark-001 — Pure 2D Dynamic CCD Candidate Asymmetry

**Discovered:** 2026-06-21

**Status:** Closed 2026-06-23

**Evidence:** The short in-process `continuous-collision-evidence`
BenchmarkDotNet smoke reported `Pure2DDynamicCandidateIndexAttributionEvidence`
at about `4.31 ms/op` for `1024` bodies, compared to about `1.47 ms/op` for 3D.
The 2D relative-sweep attribution row was also higher than 3D.

**Fresh baseline 2026-06-23:** Re-running
`continuous-collision-evidence --filter "*Dynamic*AttributionEvidence*" -j Short -i`
reproduced the signal:

- `Pure2DDynamicCandidateIndexAttributionEvidence`, `256` bodies: `598.731 us`
  versus 3D `277.042 us`.
- `Pure2DDynamicCandidateIndexAttributionEvidence`, `1024` bodies: `4.709 ms`
  versus 3D `1.524 ms`.
- `Pure2DDynamicRelativeSweepAttributionEvidence`, `1024` bodies: `4.782 ms`
  versus 3D `3.631 ms`.

**RCA 2026-06-23:** The dense benchmark layout was not accidentally stacking 3D
Y layers into the 2D plane; both dense scenes are planar. The signal had three
concrete causes:

- Pure `TwoD` contexts built both the planar 2D candidate index and the mixed
  2D-as-3D slab candidate index even though mixed CCD can only run in
  `PhysicsRuntimeMode.Mixed`.
- Planar 2D candidate gathering reused the 3D `FixedBoundVolume` index, forcing
  a dead Y axis, `Vector3d` bound construction, and extra comparisons into the
  2D hot path.
- The evidence fixture reset 2D bodies with `Sleep()` followed by
  `SetPosition(...)`, which churned partition awake state and collider rebuilds
  before every attribution query. 3D already had a single `ResetPosition(...)`
  fixture reset path.

**Resolution 2026-06-23:** `GravitasPhysics2DService` builds the mixed dynamic
CCD index only when the runtime mode actually runs mixed contacts. Pure planar
CCD uses `DynamicCcdCandidateIndex2D` and `DynamicCcdPlanarBounds` instead of
projecting circles into a 3D `FixedBoundVolume`. `SolidBody2D` gained
`ResetPosition(...)` parity with 3D, and the continuous-collision benchmark
fixture uses it for deterministic 2D reset/setup without sleep/wake churn.

**Validation 2026-06-23:** Re-running the same attribution benchmark reduced:

- `Pure2DDynamicCandidateIndexAttributionEvidence`, `256` bodies: `598.731 us`
  -> `92.281 us`.
- `Pure2DDynamicCandidateIndexAttributionEvidence`, `1024` bodies: `4.709 ms` ->
  `615.603 us`.
- `Pure2DDynamicRelativeSweepAttributionEvidence`, `256` bodies: `590.845 us` ->
  `139.723 us`.
- `Pure2DDynamicRelativeSweepAttributionEvidence`, `1024` bodies: `4.782 ms` ->
  `718.256 us`.

MemoryDiagnoser reported no recurring managed allocation in the 2D rows; the
remaining `1 B/op` at `1024` is treated as in-process runner noise unless a
focused allocation guard reproduces it.

**Likely files:**

- `src/Gravitas/CollisionHandling/Continuous/DynamicCcdCandidateIndex.cs`
- `src/Gravitas/Core/2D/GravitasPhysics2DService.cs`
- `src/Gravitas/Core/2D/SolidBody2D.cs`
- `src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.Hits.cs`
- `src/Gravitas/Core/2D/SolidBody2D.ContinuousCollision.Kinematic.cs`
- `tests/Gravitas.Tests/CollisionHandling/DynamicCcdCandidateIndexTests.cs`
- `tests/Gravitas.Tests/Core/SolidBody2DAngularDynamicsTests.cs`
- `tests/Gravitas.Benchmarks/Support/ContinuousCollisionBenchmarkSupport.cs`

**Closure criteria:** Met. The gap is explained by mixed-index overwork,
planar-vs-3D index shape, and benchmark reset asymmetry. The runtime path keeps
2D planar candidate ordering stable with duplicate suppression preserved, and
the attribution benchmark no longer shows a 2D candidate-gathering penalty.

### GRV-Benchmark-002 — 3D Shape-Exact False-Positive Cost

**Discovered:** 2026-06-21

**Status:** Closed 2026-06-23

**Evidence:** The short in-process `continuous-collision-evidence`
BenchmarkDotNet smoke reported
`Pure3DFullRuntimeShapeExactFalsePositiveEvidence` as a standout cost compared
with the matching pure 2D row.

**RCA 2026-06-23:** The dominant cost was not the final GJK-style exact reducer
in isolation. Static 3D CCD first gathered hits through the conservative
swept-sphere proxy even for non-sphere exact-capable sources, then refined every
proxy hit afterward. In false-positive-heavy scenes, that made the broad proxy
path manufacture work that the source-shape sweep could reject earlier. The
exact sweep workers also lacked a cheap swept-bounds overlap prefilter, so
obviously disjoint target bounds could still enter the shape reducer.

**Resolution 2026-06-23:** 3D static CCD collects non-sphere convex-source hits
through `GravitasQuery3DService.SweepExactSourceAgainstStaticAll(...)`. That
keeps source shape information during candidate collection and reserves the old
swept-sphere path for sphere or unsupported sources. Shared swept-bounds
prefilters reject disjoint sphere and convex-source targets before entering
per-shape reducer logic.

**Validation 2026-06-23:** Re-running
`continuous-collision-evidence --filter "*ShapeExactFalsePositiveEvidence*" -j Short -i`
reduced:

- `Pure3DFullRuntimeShapeExactFalsePositiveEvidence`, `256` bodies: `37.816 ms`
  -> `19.744 ms`.
- `Pure3DFullRuntimeShapeExactFalsePositiveEvidence`, `1024` bodies:
  `174.326 ms` -> `99.561 ms`.

The focused `ContinuousCollisionDetectionTests` and
`GravitasQuery3DServiceSweepTests` filter passed after the change.

**Likely files:**

- `src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Hits.cs`
- `src/Gravitas/Core/3D/SolidBody.ContinuousCollision.Kinematic.cs`
- `src/Gravitas/Queries/3D/Sweeps/ConvexSweepQueryWorker.cs`
- `src/Gravitas/Queries/3D/GravitasQuery3DService.Raycast.cs`
- `src/Gravitas/Queries/3D/Sweeps/SweepBoundsUtility.cs`
- `src/Gravitas/Queries/3D/Sweeps/SweptSphereQueryWorker.cs`
- `tests/Gravitas.Tests/CollisionHandling/ContinuousCollisionDetectionTests.cs`

**Closure criteria:** Met. The row is explained by conservative proxy overwork,
the runtime path was optimized without weakening exact-source correctness, and
focused CCD/query tests plus before/after benchmarks validate the change.

### GRV-Benchmark-007 — 3D Dynamic Shape-Exact BDN Allocation Signal

**Discovered:** 2026-06-23

**Status:** Closed 2026-06-23

**Evidence:** The short in-process
`dynamic-ccd-scaling --filter "*DynamicShapeExact*" -j Short -i` smoke reported
`SparsePure3DDynamicShapeExactCcdFalsePositiveBatch8` allocation scaling with
body count: about `43,008 B/op` at `64` bodies and `172,110 B/op` at `256`
bodies. The matching 2D rows reported only tiny in-process runner noise.

**Counter-evidence:** The focused xUnit guard
`ContinuousMode_DynamicRelativeShapeExactPath_ShouldNotAllocateAfterWarmup`
passed with `0` allocated bytes after warmup for the same thin-cuboid dynamic
relative false-positive shape family.

**RCA 2026-06-23:** The dynamic row exercised the same false-positive-heavy
exact-source reducer path as the static signal. The allocation scaling did not
reproduce in the focused runtime guard, and after the exact-source swept-bounds
prefilters landed, the BDN row no longer showed the large per-body allocation
slope. The remaining `78 B/op` at `256` bodies matches the tiny in-process
runner noise also reported by the 2D rows in the same run.

**Resolution 2026-06-23:** `ConvexSweepQueryWorker` computes a padded
swept-source bounds interval during `Prepare(...)` and skips disjoint collider,
compound-part, and concave-mesh triangle candidates before exact reducer work.
`SweptSphereQueryWorker` uses the same shared bounds utility for sphere-source
sweeps.

**Validation 2026-06-23:** Re-running
`dynamic-ccd-scaling --filter "*DynamicShapeExact*" -j Short -i` reported:

- `SparsePure3DDynamicShapeExactCcdFalsePositiveBatch8`, `64` bodies:
  `7.736 ms`, `0 B/op`.
- `SparsePure3DDynamicShapeExactCcdFalsePositiveBatch8`, `256` bodies:
  `29.332 ms`, `78 B/op`.

The same run reported `42 B/op` and `78 B/op` for the matching 2D rows, so the
remaining values are treated as BenchmarkDotNet in-process measurement noise
unless a future focused allocation guard reproduces them.

**Likely files:**

- `src/Gravitas/Queries/3D/Sweeps/ConvexSweepQueryWorker.cs`
- `src/Gravitas/Queries/3D/Sweeps/SweepBoundsUtility.cs`
- `src/Gravitas/Queries/3D/Sweeps/SweptSphereQueryWorker.cs`
- `tests/Gravitas.Tests/CollisionHandling/ContinuousCollisionDetectionTests.cs`
- `tests/Gravitas.Benchmarks/Core/DynamicCcdScalingBenchmarks.cs`

**Closure criteria:** Met. The scaling allocation signal was removed from the
benchmark row, the dynamic false-positive xUnit allocation guard remains green,
and the remaining BDN byte counts match runner noise rather than a runtime
allocation slope.

### GRV-Benchmark-003 — 3D Full-Runtime CCD Allocation

**Discovered:** 2026-06-21

**Status:** Closed 2026-06-23

**Evidence:** The short in-process `continuous-collision-evidence`
BenchmarkDotNet smoke reported pure 3D full-runtime rows allocating about
`172,032 B/op` at `256` bodies and `688,138 B/op` at `1024` bodies. Pure 2D
full-runtime rows and CCD attribution rows were effectively allocation-clean.

**Update 2026-06-21:** Discrete response Workstream 3 reproduced the related
steady-state CCD guardrail failures in xUnit after the 3D full-step phase order
started exercising collision distribution during measured CCD frames. Root cause
was comparer-based `Array.Sort` through package sorting in the collision
distribution/island hot path. Gravitas uses a centralized allocation-free
runtime sort helper for active partitions, island buffers, and per-partition
collider ID copies. The focused Release allocation guardrails for 3D substep,
shape-exact translational, and rotational CCD pass under full `Simulate` +
`LateSimulate` measurement. `simulation-allocation` smoke also reported `0 B/op`
for `CollisionPartitionDistributionOnly` and `ActivePairProcessingLateSimulate`.

**Initial read:** The allocation is likely outside core CCD query/index/sweep
math. Suspect reset, host-transform publish, partition refresh, collision-pair
lifecycle, or another 3D full-runtime phase.

**Why it matters:** CCD is a hot-path feature for fast movers. Lockstep
simulations need predictable frame cost and should not introduce GC pressure
that scales with body count.

**Completed isolation:** The original rows were rerun after the Workstream 3
sort fix. Temporary attribution rows split reset, force setup, and late
simulation enough to identify the moving-frame raycast grounding path.

**RCA 2026-06-23:** The remaining allocation was not CCD-specific. The
full-runtime 3D CCD evidence bodies moved each frame with automatic ray
grounding enabled, and `SolidBody` grounding called `Query3D.RaycastAll`. The
raycast service still used GridForge's enumerable `GridTracer.TraceLine` path,
which allocated iterator/mapping state per ray. Reset, force setup, and
non-moving late simulation attribution were allocation-clean.

**Resolution 2026-06-23:** GridForge exposes caller-owned
`GridTracer.TraceLineInto(...)` overloads backed by `GridTraceScratch`.
`GravitasQuery3DService` uses that allocation-free trace buffer for closest-hit
and all-hit raycasts. A focused 3D `RaycastAll` allocation guard protects the
path.

**Validation 2026-06-23:** Re-running the original
`continuous-collision-evidence --filter "*Pure3DFullRuntime*DynamicCcdEvidence*" -j Short -i`
smoke reduced:

- `Pure3DFullRuntimeNoHitDynamicCcdEvidence`, `256` bodies: `168 KB/op` ->
  `0 B/op`.
- `Pure3DFullRuntimeDenseHitDynamicCcdEvidence`, `256` bodies: `168.01 KB/op` ->
  `15 B/op`.
- `Pure3DFullRuntimeNoHitDynamicCcdEvidence`, `1024` bodies: `672.01 KB/op` ->
  `15 B/op`.
- `Pure3DFullRuntimeDenseHitDynamicCcdEvidence`, `1024` bodies: `672.01 KB/op`
  -> `15 B/op`.

The remaining `15 B/op` values were not reproduced by the focused xUnit
allocation guard and are treated as BenchmarkDotNet in-process measurement noise
unless a future guardrail reproduces them.

**Likely files:**

- `src/Gravitas/Core/3D/SolidBody.cs`
- `src/Gravitas/Core/3D/GravitasPhysicsService.cs`
- `src/Gravitas/CollisionHandling/Pairs/*`
- `src/Gravitas/Partitions/*`
- `tests/Gravitas.Tests/Support/AllocationTestHelper.cs`
- `tests/Gravitas.SharedBenchmarkSupport/ContinuousCollisionBenchmarkLayout.cs`
- `tests/Gravitas.Benchmarks/Core/ContinuousCollisionEvidenceBenchmarks.cs`

**Closure criteria:** Met. The allocation source was removed, the original
benchmark rows were rerun, and a 3D `RaycastAll` allocation guard protects the
runtime path.

### GRV-Benchmark-004 — Grounding Raycast Probe Allocation

**Discovered:** 2026-06-21

**Status:** Closed 2026-06-23

**Evidence:** During Discrete Response Workstream 3 verification, the Release
`simulation-allocation` BenchmarkDotNet smoke reported
`GroundingRaycastProbeOnly` at about `181.8 us` and `43,008 B/op` for `64`
colliders. The same run reported no managed allocation for
`SolidBodyLateSimulateOnly`, `GroundingSweptSphereProbeOnly`,
`CollisionPartitionDistributionOnly`, and `ActivePairProcessingLateSimulate`.

**Initial read:** This appears separate from the discrete island work and the
collision distribution sort RCA. It likely belongs to the raycast-backed ground
probe or one of the query result/candidate paths used by that benchmark row.

**Why it matters:** Automatic raycast grounding is a recurring body hot path. If
this allocation is repeatable outside BenchmarkDotNet noise, grounded 3D bodies
can create avoidable GC pressure.

**Completed isolation:** The focused 3D raycast allocation guard and grounding
benchmark confirmed the automatic ray-grounding allocation came from the shared
raycast trace path.

**RCA 2026-06-23:** This was the same root cause as the remaining 3D
full-runtime CCD allocation: automatic ray grounding used `Query3D.RaycastAll`,
which depended on the enumerable GridForge line-trace path.

**Resolution 2026-06-23:** Gravitas 3D raycasts use GridForge's caller-owned
`TraceLineInto(...)` path. The grounding row no longer allocates after warmup.

**Validation 2026-06-23:** Re-running
`simulation-allocation --filter "*Grounding*" -j Short -i` reported
`GroundingRaycastProbeOnly` at `164.9 us` and `0 B/op` for `64` colliders.
`GroundingSweptSphereProbeOnly` also remained allocation-clean.

**Likely files:**

- `src/Gravitas/Core/3D/SolidBody.cs`
- `src/Gravitas/Queries/3D/GravitasQuery3DService.Raycast.cs`
- `tests/Gravitas.Benchmarks/Core/SimulationAllocationBenchmarks.cs`
- `tests/Gravitas.Tests/Core/SolidBodyGroundingTests.cs`

**Closure criteria:** Met. The runtime allocation was eliminated and the 3D
raycast path has a focused xUnit allocation guard.

### GRV-Benchmark-010 — Checked Mesh Scale And Thin-Shell Cache Cost

**Status:** Closed 2026-07-12

**Evidence:** Task 46 extended `MeshMassPropertyBenchmarks` with matching
scale-only and scale-plus-surface-inertia rows. The final post-change command
was:

```powershell
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll `
    mesh-mass-property --filter "*UpdateNonUniformMeshScale*" --job short
```

The checked scale/cache rebuild measured about `5.530 us`, `282.282 us`, and
`1.122 ms` at subdivisions `1`, `8`, and `16`. Scale plus lazy physical
thin-shell integration measured `16.087 us`, `907.933 us`, and `3.609 ms`. Every
row reported no managed allocation.

A cache-focused follow-up used:

```powershell
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll `
    mesh-mass-property --filter "*CalculateCachedClosedVolumeInertiaTensor*" `
    --job short --artifacts `
    "artifacts/benchmarks/2026-07-12-task46-mesh-scale-cache-fix2"
```

Cached closed-volume inertia measured `61.14 ns`, `64.79 ns`, and `64.33 ns` at
subdivisions `1`, `8`, and `16`, with no managed allocation.

**Resolution:** Scale changes pay deterministic O(triangle-count) validation and
scaled face-cache rebuilding. Surface integration stays lazy and its
successfully prevalidated candidate is promoted into the live cache, so callers
that do not select `SurfaceApproximation` do not pay the shell moment pass.
Closed-volume properties are likewise cached by committed scale, including the
default-scale initialization path; checked prevalidation is promoted at commit
and pose-only updates retain the cache. Retain both transform and cached-read
rows as regression signals for future mesh transform or mass-property work.

### GRV-Benchmark-011 — Physics-Material Combine Numeric Hardening

**Status:** Closed 2026-07-13

**Evidence:** Task 48 compared the existing default-material response row at the
pre-change `577cdb1` checkpoint and the final arithmetic implementation. This
row exercises the dominant `GeometricMean` friction policy rather than the
distinct-material `Maximum` path.

| Body count |     Baseline |        Final | Allocated |
| ---------: | -----------: | -----------: | --------: |
|         64 | `148.785 us` | `149.243 us` |     `0 B` |
|       1024 |  `2.8145 ms` |  `2.6088 ms` |     `0 B` |

The 64-body intervals overlap (`+0.31%` point estimate), while the 1024-body
measurement was multimodal and is retained only as a no-regression signal, not
as a speedup claim. Artifacts:

- `artifacts/benchmarks/2026-07-13-task48-geometric-material-baseline`
- `artifacts/benchmarks/2026-07-13-task48-geometric-material-after`

**Resolution:** `Average` now computes an overflow-safe raw midpoint with
ties-to-even rounding. Positive equal geometric inputs retain exact identity;
positive unequal inputs multiply separately rounded square roots so coefficient
products cannot saturate or quantize away before the root. The revised path is
deterministic, allocation-free, symmetric in sampled review, and showed no
credible regression in the default contact-response benchmark.

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

## Current Recommendation

Use GRV-Benchmark-018 for any further triangle/cylinder performance work; its
complete exact contact behavior is the baseline to preserve. Dense concave
mesh/mesh throughput remains experimental capacity guidance; prefer primitive,
convex, compound, or partitioned static-concave authoring. Promote broader work
into a dated feature plan only when the scope outgrows a focused patch.
