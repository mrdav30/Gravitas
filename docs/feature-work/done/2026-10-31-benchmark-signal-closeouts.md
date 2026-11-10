# Closed Benchmark Signal Records

**Archived:** 2026-10-31 from the benchmark signal hardening backlog.

This preserves the detailed closed-signal records and their original summary
table. Measurements, validation counts and worktree/release statements describe
their recorded investigations; they are not claims of fresh validation or a
current package release. Signals without an expanded section retain their
original table evidence. Relative links resolve to the same records as before.

Use the [backlog](../benchmark-signal-hardening-backlog.md) for the current
queue, experimental signals and compact
[closed index](../benchmark-signal-hardening-backlog.md#closed-signals).

## Closed Signals

| Signal                                                                                                                                                                 | Status | Closed     | Resolution                                                                                                                                                                                                                                                                                      |
| ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------ | ---------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [GRV-Benchmark-035 — Warm Pair Growth In Rotated Cuboid Stacks](#grv-benchmark-035) | Closed | 2026-11-09 | Inclusive bounds reject fresh separated pairs before acquisition. Remaining warm growth belongs to real voxel membership and candidate scratch. |
| [GRV-Benchmark-034 — Rotated Cuboid Support Step Cost](#grv-benchmark-034) | Closed | 2026-11-09 | Shared-frame exact lever reduction lowers complete-step cost. Dense awake support remains a documented solver-budget limit. |
| [GRV-Benchmark-031 — Ordinary Open-Mesh Mixed Full-Step Cost](#grv-benchmark-031--ordinary-open-mesh-mixed-full-step-cost)                                             | Closed | 2026-11-01 | Ordinary whole-mesh convex patch/prism contacts reuse native SAT/witness/reduction owners; full-step, physical and compatibility evidence follows.                                                                                                                                              |
| [GRV-Benchmark-030 — Pure 2D Bodyless Static Population Refresh Cost](#grv-benchmark-030--pure-2d-bodyless-static-population-refresh-cost)                             | Closed | 2026-11-01 | Exact unit-basis planar sampling removes redundant decomposition while preserving transform freshness and fixed-point rounding; matched host/refresh controls and limits follow.                                                                                                                |
| [GRV-Benchmark-029 — Dense mesh sweeps](#grv-benchmark-029--dense-mesh-sweep-cost-after-exact-contact-hardening)                                                       | Closed | 2026-10-31 | Certified patch/carrier sweeps and query-local duplicate reuse reduce query costs by 96.6–99.9% at 2,048 triangles and zero warm allocation; finite geometry and small-control limits are explicit.                                                                                             |
| [GRV-Benchmark-028 — Mixed broad phase with an empty opposite dimension](#grv-benchmark-028--mixed-broad-phase-work-with-an-empty-opposite-dimension)                  | Closed | 2026-10-31 | Skipped impossible mixed distribution; 30–42% lower measured empty-population step time, zero warm allocation and preserved query/lifetime behavior.                                                                                                                                            |
| [GRV-Benchmark-027 — Nonconvex rotational CCD step cost](#grv-benchmark-027--nonconvex-rotational-ccd-step-cost)                                                       | Closed | 2026-10-31 | Deferred exact contact materialization reduces measured cone steps by 59–64% and the pressed-capsule episode by 65%, at zero warm allocation; residual conservative progress limits and reopening criteria are explicit.                                                                        |
| [GRV-Benchmark-032 — First isolated 2D pair-owner deactivation allocation](#grv-benchmark-032--first-isolated-2d-pair-owner-deactivation-allocation)                   | Closed | 2026-10-30 | Removed default-value boxing from SwiftCollections dictionary/set nullability initializers; cold collection regressions and unchanged 2D teardown now allocate 0 B, with full standard/Lean consumer passes and 100% reachable coverage.                                                        |
| [GRV-Benchmark-026 — Initial generic convex mesh contact cost](#grv-benchmark-026--initial-generic-convex-mesh-contact-resolution-cost)                                | Closed | 2026-10-30 | Deferred witnesses improve initial sweeps; a certified whole-patch follow-up lowers the ordinary concave capsule row by about 80%, at 0 B/op. Output tolerances and control variation are published; the pre-existing #032 validation exception was subsequently resolved upstream.             |
| [GRV-Benchmark-025 — Debug ragdoll steady-state allocations](#grv-benchmark-025--debug-ragdoll-steady-state-allocations)                                               | Closed | 2026-10-30 | Current Debug/Release allocation stacks agree; restore the strict warmed humanoid guard without a runtime change. Historical 4.3 MB remains unattributed.                                                                                                                                       |
| [GRV-Benchmark-018 — Complete triangle/cylinder contact cost](#grv-benchmark-018--complete-trianglecylinder-contact-cost)                                              | Closed | 2026-10-06 | Exact reciprocal-chart and opposite-direction reuse cut rim-heavy queries by 9-20% and 64-pair batches by 6.5-8.1%, at 0 B/op and 100% coverage. Root retention is reverted; scene capacity remains GRV-Benchmark-024.                                                                          |
| [GRV-Benchmark-019 — Complete triangle/cone contact cost](#grv-benchmark-019--complete-trianglecone-contact-cost)                                                      | Closed | 2026-10-06 | Four measured passes retain exact complete contact geometry at 0 B/op and 100% coverage. Final root-sign refinement lowers interior rim by 4.1%; remaining costs and workload-frequency gate are published in GRV-Benchmark-024.                                                                |
| [GRV-Benchmark-020 — Complete capsule/circle-slab contact cost](#grv-benchmark-020--complete-capsulecircle-slab-contact-cost)                                          | Closed | 2026-10-05 | Exact output bounds, invariant normal products and certified Sturm signs reduce endpoint output to 37–39 µs and representative oblique contacts to 0.70 ms, with shared-consumer gains, 0 B/op and 100% coverage. Remaining exact root cost is published without a host-rate guarantee.         |
| [GRV-Benchmark-021 — Complete capsule/stadium-slab curved contact cost](#grv-benchmark-021--complete-capsulestadium-slab-curved-contact-cost)                          | Closed | 2026-10-05 | Shared exact root isolation and negative global-minimum certificates improve four fixtures by 11.5-26.4%, at 0 B/op and 100% coverage. Committed in Gravitas `989c7f9` / FixedMathSharp `bd8f6a2`; remaining costs are published without a host-rate guarantee.                                 |
| [GRV-Benchmark-023 — Circle workload partition and grounding scaling](#grv-benchmark-023--circle-workload-partition-and-grounding-scaling)                             | Closed | 2026-10-04 | Ownership/storage/math repairs and lean 2D gathering retained; repeated controls, 100% coverage and published configuration costs support the no-further-change decision. Latest refinement remains unstaged for review; no fixed-Hz guarantee.                                                 |
| GRV-Benchmark-022 — Exact pure-2D circle contact cost                                                                                                                  | Closed | 2026-10-02 | Shared normalization optimization committed; retain exact contact owner under documented no-further-change decision. Scene partition/grounding costs are documented separately in closed GRV-Benchmark-023.                                                                                     |
| GRV-Benchmark-012 — Mixed discrete broad-phase allocation at 32 pairs                                                                                                  | Closed | 2026-08-04 | Two independent rotational runs and corrected sparse, dense, and churn broad-phase rows reproduce `0 B/op`; the stale benchmark lifecycle and unrepresentative 4,096-collider monolithic-grid row were repaired without speculative runtime preallocation                                       |
| <a id="grv-benchmark-017--mixed-public-sweep-traversal-on-extreme-sparse-grid-spans"></a>GRV-Benchmark-017 — Mixed public sweep traversal on extreme sparse-grid spans | Closed | 2026-08-04 | GridForge's two-tier hash/BVH index replaces 64-billion-cell registration with active-grid scaling; Gravitas completes the exact public sweep in 14.8-16.0 us at 0 B with deterministic candidate and hit order; full evidence is retained in GridForge's completed two-tier spatial-index plan |
| GRV-Benchmark-013 — Mesh scale rebuild allocation                                                                                                                      | Closed | 2026-08-03 | Convex support topology is built once and scale changes refit transactional node bounds in linear time; subdivision 8/16 rows fall from 4,032/16,320 B to 0 B and improve by 7.9%/7.8%                                                                                                          |
| <a id="grv-benchmark-015--exact-3d-contact-response-ordinary-throughput"></a>GRV-Benchmark-015 — Exact 3D contact-response ordinary throughput                         | Closed | 2026-08-03 | Exact aligned-frame point anchors improve direct rows by 61.0-95.9% and the unchanged 24-row Gravitas matrix by 46.4% median versus the exact baseline; confirmation remains within 0.7% median at 0 B and 100% coverage                                                                        |
| <a id="grv-benchmark-016--exact-canonical-obb-ordinary-throughput"></a>GRV-Benchmark-016 — Exact canonical OBB ordinary throughput                                     | Closed | 2026-08-03 | One exact relative-frame kernel per relation improves matched direct rows by 35.3-64.0% and Gravitas rows by 30.9-55.7%; full DefaultJob confirmations remain at 0 B and 100% reachable coverage                                                                                                |
| GRV-Benchmark-011 — Physics-material combine numeric hardening                                                                                                         | Closed | 2026-07-13 | Overflow-safe average and geometric-mean edge handling preserve deterministic coefficient semantics; the default geometric-material response benchmark remains allocation-free with no credible timing regression                                                                               |
| GRV-Benchmark-010 — Checked mesh scale and thin-shell cache cost                                                                                                       | Closed | 2026-07-12 | Scale changes retain deterministic O(triangle-count) checked rebuilding, while successfully validated shell and volume properties are cached and repeated cached reads remain allocation-free                                                                                                   |
| GRV-Benchmark-009 — Replay hash collider-ID churn scaling                                                                                                              | Closed | 2026-07-05 | 2D and 3D collider registration now uses a shared reusable-slot registry; authoritative replay hashes traverse canonical live registration order with dense replay ordinals, while deleted ID history remains outside replay identity                                                           |
| GRV-Benchmark-008 — Pure 2D response position-correction repartition allocation                                                                                        | Closed | 2026-06-28 | Gravitas reuses empty retained partitions for immediate repartitioning; GridForge stores the common single voxel partition inline and keeps diagnostic names off success paths                                                                                                                  |
| GRV-Benchmark-005 — SwiftCollections sort hot-path allocation                                                                                                          | Closed | 2026-06-24 | SwiftCollections owns allocation-free sort and sorted-key APIs; Gravitas removed `SwiftListSortUtility`                                                                                                                                                                                         |
| GRV-Benchmark-006 — Mixed mesh finite-slab triangle scaling signal                                                                                                     | Closed | 2026-06-24 | Mixed and pure 3D query services expose mesh-triangle candidate counts, dedicated triangle-volume benchmarks cover dense and false-positive mesh targets, and pure 3D convex-source mesh sweeps use ordered lower-bound triangle candidates                                                     |
| GRV-Benchmark-001 — Pure 2D dynamic CCD candidate asymmetry                                                                                                            | Closed | 2026-06-23 | 2D uses a planar candidate index, skips mixed CCD indexing outside mixed mode, and benchmark resets use 2D reset parity                                                                                                                                                                         |
| GRV-Benchmark-002 — 3D shape-exact false-positive cost                                                                                                                 | Closed | 2026-06-23 | Static CCD uses exact-source sweeps for non-sphere convex movers before conservative sphere fallback refinement                                                                                                                                                                                 |
| GRV-Benchmark-007 — 3D dynamic shape-exact BDN allocation signal                                                                                                       | Closed | 2026-06-23 | Shared exact-sweep bounds prefilters removed the scaling allocation/time signal from 3D dynamic false-positive rows                                                                                                                                                                             |
| GRV-Benchmark-003 — 3D full-runtime CCD allocation                                                                                                                     | Closed | 2026-06-23 | GridForge allocation-free line tracing plus Gravitas 3D raycast adoption                                                                                                                                                                                                                        |
| GRV-Benchmark-004 — Grounding raycast probe allocation                                                                                                                 | Closed | 2026-06-23 | Same raycast trace fix removed automatic ray-grounding allocation                                                                                                                                                                                                                               |

<a id="grv-benchmark-035"></a>

### GRV-Benchmark-035 — Warm Pair Growth In Rotated Cuboid Stacks

**Discovered** 2026-11-09 during the
[GRV-Benchmark-034 density capture](#sustained-support-at-scene-density).
**Owner** Gravitas 3D partition refresh and collision-pair acquisition.
**Closed** 2026-11-09. Fresh pair acquisition rejects disjoint bounds.
Membership and scratch still grow when the workload first reaches new occupancy.

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
`1/5`, `1/13`, and `26/29`. The acquisition probe on `c26e2117`
confirmed all 20 had disjoint bounds before the first collision update.

The 3D pair service now checks those bounds when both owner lookups
miss, before pool rent or construction. The pair's existing inclusive
predicate still admits exact touch. Retained pairs keep their update
and notification path. The focused full-step regression begins with
separated colliders in one voxel, then checks zero-depth touch, the
same pair on separation, and one exit. The old build failed the initial
check with one owned pair.

Compared with `c26e2117`, the sixteen-stack mixed-load case retained
23,716 fresh support rows across 192 warm steps. It created none of the
20 disjoint pairs. Warm allocation fell from 66,032 B to 7,752 B, and
the largest measured step allocation fell from 59,592 B to 6,440 B.
The remaining bytes came from 6,440 B in partition membership on frame
235, then 1,224 B in membership and 88 B in the dynamic ID scratch
list on frame 236. The four-, eight- and sixteen-stack heavy-bottom
controls remained at 0 B. Reopen the allocation investigation if actual
membership or scratch growth recurs in a representative host workload.

Three uninstrumented Windows x64 Release launches per build ran in
`A B B A A B` order with the same released packages, SDK 10.0.302 and
runtime 10.0.10. The table reports medians of launch statistics.

| Workload | Step p50, ms | Step p95, ms | Overruns / 192 |
| --- | ---: | ---: | ---: |
| 4 heavy-bottom stacks | 6.268 → 6.263 | 7.920 → 7.607 | 0 → 0 |
| 8 heavy-bottom stacks | 15.214 → 14.608 | 16.314 → 15.811 | 5 → 0 |
| 16 heavy-bottom stacks | 25.431 → 24.976 | 28.291 → 27.791 | 192 → 192 |
| 16 mixed-load stacks | 15.504 → 15.418 | 19.637 → 18.331 | 46 → 36 |

One eight-stack candidate launch still overran seven times. The
[Phase 3 density checks](../2026-10-06-long-duration-physics-quality-and-tuning-plan.md#phase-3-quality-versus-cost)
retain both heavy-bottom budget cases.

A disposable comparison matched raw body positions, rotations and
velocities, along with the support row IDs, depths and normals oriented
toward the upper endpoint, on all 256 frames of each of the four density
cases. Full digests change across builds because they include separated
pair records and cull state. Delaying acquisition can change the initial
pair lead or callback order across versions. The eight authoritative
digests in `mixed-lifecycle-v1` were refreshed after its body, event
and query observations matched. The other seven shared fixtures
retained their expected digests.

Two `measure-quality-stack-density verify` launches per configuration
matched across Release and ReleaseLean, including warm allocation and
support counts. `verify-quality-stacks` and `verify-quality-stack-support`
also matched between configurations. Both 1,024-actor strategy workloads
kept their work counts and first completed all chains on frame 67. The
2D replay digest stayed at `7bece61d4c37790c1c14a89a2fa3db51`. The updated
3D digest matched in both configurations at `ed6d25408c327fbe8522ff6a2fd09c08`.

The full suites passed 8,403 Release and 8,332 ReleaseLean tests. Both
reached 100% line, branch and method coverage (53,967 lines, 22,100
branches and 5,635 methods). DocFX passed with warnings treated as errors.

<a id="grv-benchmark-034"></a>

### GRV-Benchmark-034 — Rotated Cuboid Support Step Cost

**Discovered** 2026-11-09 during the GRV-Issue-151 verification.
**Owner** Gravitas 3D cuboid contact generation and discrete island response.
**Closed** 2026-11-09. Focused exact-response work is complete. The
sixteen-stack heavy-bottom case remains a solver-budget stress case.

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
[GRV-Issue-151 evidence](../2026-10-06-long-duration-physics-quality-and-tuning-plan.md#grv-issue-151-implementation)

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
remaining allocations are tracked under [GRV-Benchmark-035](../benchmark-signal-hardening-backlog.md#grv-benchmark-035).

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
steps. The all-heavy case stresses fallback frequency. The mixed-load case
shows the budget sensitivity when only half the stacks are heavy-bottom.

#### Shared-frame lever reduction and closure

The final pass subtracts local points before rotation when two anchors
share the exact origin, rotation and other additive terms. The
subtraction uses a signed wide value, so opposite ends of the scalar
range remain valid. The solver reuses FixedMathSharp's exact rigid
projection and carries one rotation denominator into response. Other
anchor pairs keep the general relative-offset calculation. A focused
regression rotates a local difference wider than Q32.32 and verifies its
final representable angular delta. The change needs no solver cache or
adjustment to the contact rows, iteration count, or rounding.

The dense heavy-bottom stage probe counted 209,226 uses of the shared
frame. Exact lever construction fell from 9.5% to 2.4% of instrumented
step time. The normal fallback count held at 84,807 calls, with its
inclusive share falling from 36.3% to 31.6%. The remaining cost includes
the full set of normal and friction rows across all six sweeps.

Use the current `measure-quality-stack-density [verify]` fixture on
both builds. For the baseline, apply only the fixture's eight-stack
addition to `5bde8ef2`. The new 4-by-2 lattice keeps 4-unit X spacing
and 8-unit Z spacing. The original four- and sixteen-stack positions
are unchanged. Three fresh Release launches per build ran in
`A B B A A B` order on the same Windows x64 SDK and runtime as the
earlier density capture. Values are medians of per-launch statistics.

| Workload | Step p50, ms | Step p95, ms | Overruns / 192 |
| --- | ---: | ---: | ---: |
| 4 heavy-bottom stacks | 6.720 → 5.899 | 8.085 → 7.425 | 0 → 0 |
| 8 heavy-bottom stacks | 17.889 → 15.089 | 19.592 → 15.780 | 151 → 2 |
| 16 heavy-bottom stacks | 29.743 → 24.810 | 34.177 → 29.179 | 192 → 192 |
| 16 mixed-load stacks | 16.696 → 15.372 | 20.049 → 18.492 | 99 → 44 |

The heavy-bottom p50 launch ranges separated in all three cases.
Eight-stack candidate runs had 0, 2, and 18 overruns. The mixed-load
timings overlapped and one candidate launch reached 43.030 ms p95.
Use those results as a host-specific budget check for sustained awake
contacts. Four stacks stayed within the full interval on this host.
Eight left little headroom for the rest of the host frame. Sixteen
heavy-bottom stacks remained over budget on every measured step.

Release and ReleaseLean matched all four per-frame replay digests,
including solver caches, and the live support work counts. The
eight-stack trace was `6c28992a09deb949a6788b99645ae868` with 12,286
fresh rows. All three
heavy-bottom cases allocated 0 B in the warm window. The mixed-load
case matched the prior 66,032 B total tracked under
[GRV-Benchmark-035](../benchmark-signal-hardening-backlog.md#grv-benchmark-035).
The original stack CSVs and both 1,024-actor strategy traces matched.
Release passed 8,402 tests and ReleaseLean passed 8,331, with zero
uncovered lines, branches or methods in both configurations.

Close the focused optimization with the finite support rows and exact
fallback intact. Carry the remaining density limit into
[Phase 3 solver tuning](../2026-10-06-long-duration-physics-quality-and-tuning-plan.md#phase-3-quality-versus-cost).
Reopen this signal for an equivalent-work regression or when a
representative host workload exceeds its reserved physics budget at
settings that meet its support and replay requirements.

### GRV-Benchmark-031 — Ordinary Open-Mesh Mixed Full-Step Cost

**Status:** Closed 2026-11-01. Certified whole-mesh convex patches reuse the
existing convex-pair SAT/contact owner against finite AABB/polygon prisms. This
closes the ordinary open-quad signal, not arbitrary nonconvex mesh capacity or
curved-contact workload frequency. Experimental #014/#024 remain separate.

#### Attribution and implementation

The existing maintained `JoinedStaticSolidMixedSimulationBenchmarks`,
`Shape=OpenMesh, Layout=Continuous`, reproduces the approximately 4.7 ms frame.
A sampled full-step profile attributes **96.59%** to ordinary mesh/slab contact,
including **84.52%** in `MeshSlabSurfaceBoundary.BuildSurfaceContacts` and
**83.59%** in `SegmentSlabSurfaceContact`. Its prism edge/corner enumeration is
the cost, not missing corner-fan recognition: `MeshSlabFaceRegions.Build`
already consumes certified corners. Percentages are overlapping inclusive sample
weights, not additive CPU fractions or acceptance timings. The capture uses
`dotnet-trace` 10.0.750501 `dotnet-sampled-thread-time`, a one-second warmup and
12-second loop filtered to `RunProfile`, excluding fixture setup.

The runtime diff is **22 net lines across three existing files**. The mixed
owner widens its existing closed-convex/prism route to meshes whose entire
surface is one certified convex coplanar patch. `TryAppendHullContacts` takes
explicit first-geometry spans; `MeshHullSurfaceContacts.FillPatch` supplies
strict corners, corner-fan triangles and genuine perimeter edges into three
context scratch lists. No second solver, new runtime file, geometry cache,
topology mutation or public API is introduced. Scratch capacity grows with
encountered corner count and is reused; its geometry is rebuilt each call.
Existing prism buffers, SAT selection, finite witnesses, sample reduction and
material/namespace publication are reused. The existing 3D closed-hull caller
passes exactly its original geometry and is separately measured below.

The whole-mesh certificate is intentionally conservative. It excludes holes,
nonconvex fill, bent or disconnected surfaces and additional owners. Curved
slabs keep their current owners. A positive-thickness convex prism supplies
full-dimensional SAT axes, so the extra axes required when _both_ operands are
planar are unnecessary. A certified open sheet gains no filled interior.
Geometry uses `GeometryVertices`/`GeometryFrame` and the established bounded
`mesh.Origin` witness chart. Empty witness visitation still declines to the
original finite-surface path. Separation is a handled convex-pair result.

#### Physical and compatibility decisions

The previous perimeter route could publish several edge/corner exit groups for
one convex sheet/prism pair. The native convex owner selects one globally valid
minimum exit and supplies finite paired anchors for that support family. This
changes contact groups, identities and replay hashes across versions; it is
deliberate, not a claim of byte-identical manifolds or replay caches.

The ten new maintained source-behavior cases exercise 16 axis-aligned setups
(both windings, one/four subdivisions, rigid/affine placement, zero/extreme X),
oblique finite rims and overlapping-bounds empty-corner misses, real compound
leaves with independent materials/namespaces, and two repeated 128-frame freely
rotating runs. They check finite matched witnesses, exact selected depth/normal,
deterministic contact identity, whole-shape clearance through the independent
authored-triangle continuous-admission path, drive preservation and same-version
replay including solver caches.

On the confirmed baseline DLL, eight of these new expectations fail: multiple
groups replace the new single family, and the freely moving grazing panel
acquires Y displacement (observed 1.5920000239 by the failing assertion) despite
zero gravity and inward Z drive. The candidate retains Y=0 and passes all ten.
This is evidence for the tested selection/response correction, not a general
stability or scene-capacity claim.

Two existing finite tall-slab cases deliberately update their exact tie winner.
For plane `X+Y=0`, source X/Z extents `[-2,2]`, target X/Z `[-1/2,1/2]` and
half-height 100, either X or Z clears the whole sheet at exactly **2.5**; the
oblique plane exit is `100.5/sqrt(2)`. SAT encounters the prism's Z face first
and retains it on the exact tie. The tests now pin that exact Z axis for
AABB/polygon prisms and retain X for unchanged circle/capsule owners. They do
not accept either axis loosely: depth/four-raw tolerance, sample counts, contact
IDs/anchors across subdivisions and near/far whole-owner clearance assertions
remain intact. Independent review verified this geometric tie and the revised
selection contract before the expectation changed.

#### Matched full-step results

Values are **microseconds per complete host frame**, including drive,
`Simulate`, `LateSimulate` and existing physical assertions. They are steady
local measurements, not game capacity or gameplay-frame tail latency.

| Shape / layout          | Before us/frame | After us/frame |  Change |
| ----------------------- | --------------: | -------------: | ------: |
| OpenMesh/Continuous     |         4523.49 |         114.65 | -97.47% |
| OpenMesh/Joined2        |          269.15 |         265.99 |  -1.17% |
| OpenMesh/Joined16       |          516.39 |         507.12 |  -1.80% |
| ConcaveMesh/Continuous  |         5055.65 |        4931.09 |  -2.46% |
| Box/Continuous          |          112.22 |         113.77 |  +1.39% |
| Sphere/Continuous       |          159.25 |         161.69 |  +1.53% |
| 3DConvexMesh/Continuous |          126.64 |         126.25 |  -0.31% |

All 448 measured windows allocate **zero current-thread bytes**. The frozen
payloads preserve position, linear/angular velocity and rotation for fixed setup
and four 64-frame continuation snapshots in all seven cases. Authority and
solver-cache hashes also match for every control; only ordinary
OpenMesh/Continuous intentionally changes its contact selection/hashes. Repeated
same-version setup hashes match across independent timing launches.

Per-process means retain dispersion and adverse estimates. Each cell contains
the two independent process means; small control differences are inconclusive,
not equivalence, zero overhead or a universal speedup claim:

| Case                    | Baseline processes (us/frame) | Final processes (us/frame) | Mean delta us/frame |
| ----------------------- | ----------------------------: | -------------------------: | ------------------: |
| OpenMesh/Continuous     |           4613.691 / 4433.290 |          117.232 / 112.075 |           -4408.837 |
| OpenMesh/Joined2        |             273.548 / 264.759 |          268.452 / 263.534 |              -3.161 |
| OpenMesh/Joined16       |             526.703 / 506.079 |          504.578 / 509.654 |              -9.275 |
| ConcaveMesh/Continuous  |           5159.552 / 4951.757 |        4911.568 / 4950.622 |            -124.560 |
| Box/Continuous          |             114.143 / 110.288 |          111.699 / 115.849 |              +1.558 |
| Sphere/Continuous       |             163.479 / 155.013 |          157.537 / 165.835 |              +2.440 |
| 3DConvexMesh/Continuous |             129.084 / 124.199 |          125.772 / 126.735 |              -0.388 |

The positive Box/Sphere estimates are **+1.56/+2.44 us/frame** (+1.39/+1.53%),
with overlapping process ranges. They are retained as inconclusive; no
regression absence or zero-overhead claim is made.

A separate A/A check runs the identical baseline and its byte-identical clone in
fresh processes for both method groups. All 35 payload DLLs match, loaded
bindings and complete fixed states/hashes agree, and 224 windows allocate zero
bytes. Its contrasts expose process/environment sensitivity, not an isolated
cause, statistical equivalence bound or proof that an A/B estimate is noise.

| Identical-binary control | Original us/frame | Clone us/frame | Sham change |
| ------------------------ | ----------------: | -------------: | ----------: |
| OpenMesh/Continuous      |          4480.955 |       4535.058 |      +1.21% |
| OpenMesh/Joined2         |           271.049 |        261.972 |      -3.35% |
| OpenMesh/Joined16        |           514.637 |        502.294 |      -2.40% |
| ConcaveMesh/Continuous   |          4950.148 |       4953.648 |      +0.07% |
| Box/Continuous           |           109.558 |        110.770 |      +1.11% |
| Sphere/Continuous        |           158.426 |        157.522 |      -0.57% |
| 3DConvexMesh/Continuous  |           125.063 |        124.989 |      -0.06% |

Reopen a small control on a longer reproducible host-relevant regression; the
large ordinary-row improvement does not license an accepted control tradeoff.

#### Reproducer, environment and bindings

The primary maintained scene is one mass-one frictionless open two-triangle X/Z
quad with coordinates +/-1/2, Concave mode and SurfaceApproximation inertia. It
starts at `(-2,0,1/2)` against one bodyless rectangle X`[-8,8]`, Z`[-2,0]`,
mixed half-thickness 2, with no continuity membership. The mixed world has one
grid from `(-8,-4,-8)` to `(16,4,8)` (the maintained mixed-context helper), 16
Hz, zero gravity/air/damping/minimum speed, frozen Y/rotation, manual grounding
and disabled sleep. After 32 settling frames and four 64-frame setup batches,
every invocation drives X velocity +/-1 (reversing at X=+/-2), applies Z impulse
-1/16 and advances 64 complete frames. Setup/settling are outside timing.
Contact preserves tangential speed and positive source Z every frame.

Controls use the same maintained fixtures: OpenMesh joined2/16 wall pieces;
ordinary six-vertex four-triangle nonconvex L mesh; ordinary Box and Sphere; and
`JoinedStaticSurfaceSimulationBenchmarks` with `Shape=ConvexMesh`,
`Joined=false`, `Workload=Small` for the affected 3D closed-hull/patch owner.
All use their committed fixture inputs and assertions. Joined work is not an
equivalent-work speedup comparison, and the nonconvex row retains its general
finite-surface owner. The baseline revision below preserves these fixture
definitions independently of disposable drivers.

Windows x64 10.0.26300, Intel i7-9700K, SDK 10.0.302, runtime 10.0.10, Release
net10.0, workstation blocking GC, high priority, affinity 192 (CPUs 6/7),
`DOTNET_PROCESSOR_COUNT=2`. Coordinator builds/tests/profiles/timings were
serialized. Four fresh-process rounds use baseline/final/final/baseline order,
with separate mixed and 3D launches each round (eight launches). Each row warms
three seconds, calibrates for at least 200 ms, then records 16 windows targeting
50ms with 1..4096 invocations; one 64-frame invocation is the minimum. Values
divide by 64; means weight the two process means equally. Preliminary root-stack
timings/prototypes are excluded from acceptance means.

Validation uses an isolated committed source graph because concurrent
SwiftCollections work temporarily made the shared checkout fail to compile
(`IStateBacked<>`, `StateJsonConverterFactory`, `DiagnosticChannel` missing). No
concurrent source was reverted, included, staged or repaired here. Frozen
revisions, plus this Gravitas runtime/test diff:

- Gravitas: `a7d5931c60b4ca7b5eba44ac117a9c9216c2e1b6`.
- FixedMathSharp: `4a2d43379d082d475b49e018dc3952330e0fa633`.
- SwiftCollections: `ec9da58c93a7752fccde627e821f3fa5ae130849`.
- GridForge: `54164f0aa197f050b7263f37373d4a55ff7c81fa`.
- Chronicler: `c45f9ab53913e5b1692f0717733425e0f4defeb2`.

The isolated copy initially retained its older baseline DLL because copied
source timestamps preceded its output. DLL checks exposed that stale build;
forcing copied inputs newer and rebuilding produced the tested candidate. The
failing baseline expectations above were confirmed against the baseline hash.
Final acceptance uses the final freshly rebuilt/tested DLL, not that stale copy.
FMS's preceding c3fcd6b->4a2d433 change is documentation only; no numerical
source or dependency worktree changes are part of this fix.

Actual loaded LSF modules and CoreLib hashes/paths are recorded on every
acceptance launch. All payload DLLs are byte-identical except `Gravitas.dll`,
and the final timed DLL matches the ordinary restored Release test DLL.

| Payload DLL                         | SHA256 (baseline / final if changed)                                                                                                    |
| ----------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------- |
| Gravitas.dll                        | `2c1b2a632b2680a537324202e32f5bbb2994451a541d77e733838b9305b913e4` / `008b0817a6f37f5ddefa2f5650ecc3ce35778416b9ea9a13dd2bec1953133cf0` |
| Gravitas.Benchmarks.dll             | `06419d729bf05bb00ee496582cebf4190fc5d04fd47a9b1c8b4e9d620844da13`                                                                      |
| Driver.dll                          | `d68212566ece243798531093a6a57f242479db3389d10965fc37f155e21ffb93`                                                                      |
| FixedMathSharp.dll                  | `af64e83ffa0058d494f85178859a9da58b89fa6847ec65a7373a6631510c9eab`                                                                      |
| FixedMathSharp.Chronicler.dll       | `019c5e17a624bf966805fb95c9a2616a6ba38ed2508f28ad0011d5d1986aef74`                                                                      |
| SwiftCollections.dll                | `91bb0536bb0bfd16e5decb4835e5de06ee2e1d0ef00bc1e617c454af575f6bbd`                                                                      |
| SwiftCollections.FixedMathSharp.dll | `da3a5e159c48ad09fd9343069c3b5a4b6a2459396dd4aa18d2b3645c0bf81a65`                                                                      |
| GridForge.dll                       | `d5e513272676c6e0629c9812cf73a00bed857960c4323bc3f199244c4ed379cc`                                                                      |
| Chronicler.dll                      | `e3ae01662a1dcf4e9a1a64c26ddb53b8a12ead981ccecc52adeab8df76d106c8`                                                                      |
| Chronicler.Diagnostics.dll          | `448097e72f51dca32bc779e17783f71d7dab9e2d043f80a95cc34e12a7e26c67`                                                                      |

Release passes **8,348 tests** and Lean **8,277**, with zero failures/skips. One
fresh, unmerged final capture per configuration retains 100% raw reachable
coverage: Release **53,559/53,559 sequence points, 21,772/21,772 branches,
5,602/5,602 methods**; Lean **53,557/53,557 sequence points, 21,772/21,772
branches, 5,601/5,601 methods**. No exclusions were added. Both
netstandard2.1/net10.0 library targets build in Release/Lean with zero errors.
Nested analyzer-config conflicts and existing generated MemoryPack XML
documentation warnings occur in the isolated graph; they are not clean-warning
claims or changes to suppression policy. The final raw captures are under
`coverage-release-final/` and `coverage-lean-final/`; earlier two-failure
tie-expectation captures are excluded from final validation.

Independent source/geometry review approved the whole-patch proof, scratch
lifetime, explicit first geometry and exact tie-case adaptation. The coordinator
ran the reported tests, raw coverage, physical/hash checks, builds and timings.

All 40 maintained mixed solid simulation rows complete in-process `Dry` mode
with a successful launcher exit. Those cold smoke values validate the fixture,
not warmed acceptance time/allocation. A matching final sampled profile
attributes 47.67% to mesh/slab contact and 31.38% to collider partition updating
(including 29.47% in collision-position publication). These overlapping,
call-site-sensitive weights are not additional acceptance timings; the former
perimeter enumeration is no longer the ordinary-row bottleneck.

Reproduce maintained source-stack rows without editing project references:

```powershell
$env:DOTNET_PROCESSOR_COUNT = '2'
$env:UseLocalLsfStack = 'true'
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -p:UseLocalLsfStack=true -p:DisableTransitiveProjectReferences=true -m:1 -p:BuildInParallel=false -p:UseSharedCompilation=false -nr:false
dotnet tests/Gravitas.Benchmarks/bin/Release/net10.0/Gravitas.Benchmarks.dll --filter '*JoinedStaticSolidMixedSimulationBenchmarks*' --job Short
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj -c Release -p:UseLocalLsfStack=true -p:DisableTransitiveProjectReferences=true --collect:'XPlat Code Coverage' --settings tests/Gravitas.Tests/coverlet.runsettings
# Repeat tests/coverage with -c ReleaseLean; build src/Gravitas in both configurations/target frameworks.
```

Ignored `artifacts/grv-benchmark-031/` retains archived source HEADs/status,
`graph/`, `control-baseline/` and `final/` payloads, driver source, full raw
coverage, profile traces, fixed continuation smoke, per-launch timings/bindings
A/A clone/captures/summary and `verified.log`/`validated-summary.json`. Earlier
root prototypes and failed test expectations are retained separately;
failed/partial runs are not timing or final coverage evidence. Maintained
source/tests/fixtures, inputs, revisions, protocol, results and limitations here
preserve the conclusion without relying on those disposable artifacts. No tests
of benchmark/test projects or launchers were added.

#### Remaining scope and reopening

Nonconvex/holed/disconnected/multiple-owner surfaces, curved contacts, joined
supplier domains and general mesh capacity keep their existing contracts and
costs. Reopen with a finite witness/clearance defect, unstable same-version
identity or replay, warm allocation, reproducible control regression, or a
host-relevant remaining ordinary-patch budget bottleneck. Do not generalize this
scene to a game capacity claim. Revalidate integration with the separate
SwiftCollections changes when they land; they were intentionally excluded.
Native cross-platform and released-package validation were not rerun.

#### Historical intake (2026-10-28; pre-optimization stack)

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

### GRV-Benchmark-030 — Pure 2D Bodyless Static Population Refresh Cost

**Status:** Closed 2026-11-01. The executed transform-sampling cost is
attributed and reduced for an exact unit X/Z basis. Static populations still
require a linear freshness pass; this is not a constant-time refresh or capacity
claim.

The 64-bodyless pure 2D baseline profile attributes **47.27%** of sampled
full-step thread time to `PrepareCollisionPartitions`, including **22.13%** in
`ColliderScalePolicy.TryExtractPlanarTransform`. These inclusive weights
overlap; they are not additive CPU percentages. A 12-second `dotnet-trace`
10.0.750501 `dotnet-sampled-thread-time` capture, filtered to the maintained
fixture's `FullStep`, includes its setup calls to that same method. No profile
timings are used in the acceptance table.

Nine added runtime lines in the existing `ColliderScalePolicy` return unit scale
and zero yaw only for the **exact** X/Z identity basis, after rejecting Y/XZ
coupling. Every call still checks positive authored X/Z ancestry and strictly
composes the current world matrix. Other bases retain the same decomposition,
rounding and admission checks. Direct arbitrary diagonal extraction was
rejected: `One + MinIncrement` can decompose to exactly `One`, so returning the
raw matrix coefficient would change physical scale. No cache, index, lifecycle
change, public API, new runtime file or persistent state is introduced. 3D
extraction is untouched; mixed consumers of the existing planar owner receive
the same benefit.

#### Workload and matched results

`StaticPopulationRefreshBenchmarks` maintains the bounded population control.
All rows use the same loaded grid bounds `(-16,-4,-16)` to `(288,4,16)`, a 16 Hz
host, no continuity declaration, no damping/air drag/minimum speed, a
frictionless floor, radius-1/2 mover, disabled sleep and frozen rotation. The
local floor is 16 units wide; each extra remote piece is two units wide at
`X = 32 + 4*(i-1)`, for `i=1..63`. Pure 2D uses a two-unit-deep planar box at
Z=-1 and a circle starting at `(-2,1/2)`. Pure 3D uses a box of size
`(width,2,4)` at Y=-1 and a sphere at `(-2,1/2,0)`, with Z frozen. Mixed uses
the planar boxes with half-thickness two and a sphere at `(-2,0,1/2)`, with Y
frozen, zero gravity and a 1/16 inward Z impulse each frame. Pure modes use unit
gravity. Owners are either bodyless or explicitly static bodies.

Setup settles 32 frames and runs four 64-frame batches. Each measured full-step
invocation drives X velocity to +/-1, reverses at X=+/-2, and runs 64 complete
`Simulate` / `LateSimulate` frames, asserting preserved tangential speed and
positive support height every frame. Values below are **microseconds per full
frame**. The 1/64-static rows isolate remote population, not local contact
density.

| Mode   | Static owner    | 1 before | 1 after | 64 before | 64 after | 64 change |
| ------ | --------------- | -------: | ------: | --------: | -------: | --------: |
| TwoD   | Bodyless        |    20.66 |   18.59 |     71.25 |    34.00 |   -52.28% |
| TwoD   | Explicit static |    21.09 |   19.31 |     60.35 |    32.34 |   -46.41% |
| ThreeD | Bodyless        |    33.02 |   33.35 |     75.28 |    75.87 |    +0.78% |
| ThreeD | Explicit static |    33.53 |   33.27 |    102.60 |   103.51 |    +0.89% |
| Mixed  | Bodyless        |   165.78 |  167.72 |    224.57 |   185.68 |   -17.32% |
| Mixed  | Explicit static |   165.65 |  167.45 |    209.03 |   182.48 |   -12.70% |

The matching refresh-only pass calls each static collider's existing `Simulate`
without advancing the moving body. Values are **microseconds per 64-collider
pass**, not per collider or per frame:

| Mode   | Static owner    | Before | After |  Change |
| ------ | --------------- | -----: | ----: | ------: |
| TwoD   | Bodyless        |  50.20 | 14.16 | -71.79% |
| TwoD   | Explicit static |  38.44 | 11.51 | -70.06% |
| ThreeD | Bodyless        |  42.58 | 42.09 |  -1.15% |
| ThreeD | Explicit static |  66.44 | 66.13 |  -0.46% |
| Mixed  | Bodyless        |  51.03 | 14.21 | -72.16% |
| Mixed  | Explicit static |  38.88 | 11.46 | -70.54% |

Nonidentity/mutation controls run one pass over 64 bodyless radius-1/2 circles
at X=4*i in a grid bounded by `(-16,0,-16)` to `(288,0,16)`: yaw Pi/4, scale
`(2,1,3)`, identity children of parents scaled `(2,1,2)`, and moving hosts
alternating Z=0/1 each pass. These exercise retained fallback, hierarchy
composition and changed-shape publication. They are disposable RCA controls, not
tests of benchmark machinery.

| Refresh control | Before us | After us |  Change |
| --------------- | --------: | -------: | ------: |
| Yaw             |     59.69 |    60.26 |  +0.96% |
| Scale           |     54.27 |    54.96 |  +1.27% |
| Hierarchy       |     85.56 |    85.77 |  +0.26% |
| Moving          |    167.40 |   128.39 | -23.30% |

All 1,792 measured windows allocate **zero current-thread bytes**. Fixed setup
observations and eight 64-frame continuation snapshots per case match exactly
between payloads, including position, linear/angular velocity, rotation and
authoritative replay hash. Fixture assertions remain active during timing. These
are local steady-state observations, not cross-platform proof or frame tail
latency. Small control differences must not be read as equivalence or universal
speedups; process-level estimates and limits follow below.

The 63 additional bodyless pieces add **50.59 us/frame before and 15.41 us/frame
after** in pure 2D, derived from the matched 64-minus-one-static full steps. The
remaining population cost is retained freshness work.

Small-control process means follow (each entry is an independent process mean;
units follow the named method). Positive estimates remain **inconclusive**, not
proven noise or established zero overhead. In particular, one-static mixed steps
estimate +1.94/+1.80 us/frame despite cheaper planar sampling; no universal
full-step speedup is claimed.

| Control                    | Baseline process A / B (us) | Final process A / B (us) | Mean difference (us) |
| -------------------------- | --------------------------: | -----------------------: | -------------------: |
| ('ThreeD', False, 1) step  |             32.521 / 33.511 |          33.331 / 33.361 |               +0.330 |
| ('ThreeD', False, 64) step |             74.859 / 75.701 |          76.009 / 75.722 |               +0.585 |
| ('ThreeD', True, 1) step   |             33.210 / 33.852 |          32.998 / 33.544 |               -0.260 |
| ('ThreeD', True, 64) step  |           101.373 / 103.828 |        104.224 / 102.800 |               +0.911 |
| ('Mixed', False, 1) step   |           164.971 / 166.583 |        166.955 / 168.486 |               +1.943 |
| ('Mixed', True, 1) step    |           165.674 / 165.621 |        167.242 / 167.649 |               +1.798 |
| Yaw timing                 |             59.704 / 59.666 |          59.872 / 60.641 |               +0.571 |
| Scale timing               |             54.814 / 53.720 |          54.577 / 55.337 |               +0.690 |
| Hierarchy timing           |             86.329 / 84.784 |          86.163 / 85.386 |               +0.219 |

An additional A/A control runs the identical baseline payload and its byte-for-
byte clone in two fresh processes per method group, with the same protocol and
bindings/state checks. Its 896 windows also allocate zero bytes. This exposes
process/environment sensitivity; it does not isolate its cause or establish that
the positive A/B estimates are regression-free. A/A process contrasts:

| Control                    | Original process (us) | Identical clone process (us) | Sham change |
| -------------------------- | --------------------: | ---------------------------: | ----------: |
| ('ThreeD', False, 1) step  |                32.621 |                       35.898 |     +10.05% |
| ('ThreeD', False, 64) step |                74.937 |                       77.511 |      +3.44% |
| ('ThreeD', True, 1) step   |                32.634 |                       35.458 |      +8.65% |
| ('ThreeD', True, 64) step  |               101.358 |                      105.868 |      +4.45% |
| ('Mixed', False, 1) step   |               166.315 |                      167.397 |      +0.65% |
| ('Mixed', True, 1) step    |               166.475 |                      166.903 |      +0.26% |
| Yaw timing                 |                60.258 |                       59.908 |      -0.58% |
| Scale timing               |                54.163 |                       54.188 |      +0.05% |
| Hierarchy timing           |                85.324 |                       85.844 |      +0.61% |

Across all 28 A/A row metrics, sham changes range from -0.58% to +13.33%. The
largest is the unchanged 64-explicit-static 3D refresh pass, 64.624 to 73.238
us. That adverse control is retained; the single A/A pair cannot identify its
cause or provide an equivalence bound for another row.

These small estimates are unresolved/inconclusive, not an accepted measured
regression or a guarantee of no overhead. Reopen on a longer, reproducible
host-relevant fallback/small-population regression. The bounded large-population
improvement and exact behavioral equivalence support this closeout without
speculative tuning of unrelated code.

#### Protocol, bindings and validation

Windows x64 10.0.26300, Intel i7-9700K, SDK 10.0.302, runtime 10.0.10, Release
net10.0, workstation blocking GC, high process priority, affinity mask 192 (CPUs
6/7), `DOTNET_PROCESSOR_COUNT=2`. Builds/tests/profiles and acceptance timings
were serialized. Four fresh-process rounds use baseline, final, final, baseline
order. Each round runs separate population and fallback-control processes (eight
launches total). Each row warms for two seconds, calibrates for at least 200 ms,
then records sixteen windows targeting 50 ms, with 1..65,536 invocations per
window. Full-step values divide each 64-frame invocation by 64. Means weight the
two process means equally. A preliminary baseline-only run and the sampling
profile are excluded from matched acceptance means.

Gravitas baseline is `d6b9c0d53c89a9856afd1f8eb89c0124cddbf8bd` plus the new
unchanged benchmark fixture; final adds only the nine-line runtime change for
timing. Committed sibling sources:

- FixedMathSharp: `c3fcd6bd035afdb4aeea776d2a6a6ca35723585f`.
- SwiftCollections: `ec9da58c93a7752fccde627e821f3fa5ae130849`.
- GridForge: `54164f0aa197f050b7263f37373d4a55ff7c81fa`.
- Chronicler: `c45f9ab53913e5b1692f0717733425e0f4defeb2`.

Actual loaded LSF/host payloads are SHA256-checked on every launch. All payload
DLLs are byte-identical except `Gravitas.dll`; loaded CoreLib is from 10.0.10.

| Loaded assembly                     | SHA256 (baseline / final if changed)                                                                                                    |
| ----------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------- |
| Gravitas.dll                        | `b62f6337255764576e0d7af6ef15f066c2a411fae7fceb90abf58a951ae44a0c` / `9510dd92ee654e12837f0b83708ec886a416da86f374445a60f6d4c0b0c830a2` |
| Gravitas.Benchmarks.dll             | `41148a48bfdb3703dfec4ece97e3024c0a06b894f1a3db5252b15ba98bc33779`                                                                      |
| Driver.dll                          | `1c5ec54065929278112e243998f12e413df2a7f17c4eb0f46f3343518b966aa3`                                                                      |
| FixedMathSharp.dll                  | `d82ad7953feb4d01d4da2dc9c8f7fdfaba4226fc33ef88cd4d2382ed5db34743`                                                                      |
| FixedMathSharp.Chronicler.dll       | `a39af9f53e4249b280abac2bda2e923a4e3874f6fe067ac9755e7f55e7e2793a`                                                                      |
| SwiftCollections.dll                | `b87da81aaf0d551f347249c0a73bd7e67ead993f9c47636b6c4cb92703d0a8f2`                                                                      |
| SwiftCollections.FixedMathSharp.dll | `9ca70b0f4ebad43b1f61bbbfe29d49c0ea69c6341a79838f29f29e3ed95f3d85`                                                                      |
| GridForge.dll                       | `5aa8934a285545a0bcce6af8a6b2ddf7eee45a9ec91a81d6bef7388565fd0a39`                                                                      |
| Chronicler.dll                      | `4de2015a0e6a977db42506e01737c1151a70398cbd395451ff563eb267a78d92`                                                                      |
| Chronicler.Diagnostics.dll          | `bbe0d019808bdfaa3c122fab97235b3f18d1da3b66277e48f6d8dd21e4e97432`                                                                      |

The eleven new source-behavior cases cover exact unit bases, hierarchical
cancellation, retained Y-scale policy, both yaw-capture modes, near-unit
rounding, nonunit/yawed fallbacks, canceled negative ancestry, tiny nonplanar
coupling, unchanged runtime versions and live scale/pose/offset/query freshness
for bodyless/explicit-static owners in pure 2D/mixed. Existing full-suite
replay, serialization, shape admission, contact, query and allocation
regressions remain active. This is an optimization with a passing compatibility
baseline, not a newly reproduced correctness defect.

Release passes **8,338 tests** and ReleaseLean **8,267**, with zero failures or
skips. One fresh unmerged OpenCover capture per configuration retains raw
reachable coverage: Release **53,545/53,545 sequence points, 21,768/21,768
branches, 5,602/5,602 methods**; Lean **53,543/53,543 sequence points,
21,768/21,768 branches, 5,601/5,601 methods**. No coverage exclusions were
added. Both `netstandard2.1` and `net10.0` library targets build in both
configurations with zero errors/warnings. Independent read-only source, caller,
fixture and test review found no actionable findings; the coordinator ran the
reported checks. The ordinary Release test DLL matches the final timed runtime
SHA256. New tests exercise library source; no tests of benchmark/test projects
or launchers were added.

All 24 maintained BenchmarkDotNet rows also complete in-process `Dry` mode with
a successful launcher exit. Those cold smoke measurements validate the fixture,
not the warmed acceptance timings or allocations above.

Reproduce maintained rows using the source graph with `UseLocalLsfStack=true`
propagated into child builds, without editing project references:

```powershell
$env:DOTNET_PROCESSOR_COUNT = '2'
$env:UseLocalLsfStack = 'true'
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -p:UseLocalLsfStack=true -p:DisableTransitiveProjectReferences=true -m:1 -p:BuildInParallel=false -p:UseSharedCompilation=false -nr:false
dotnet tests/Gravitas.Benchmarks/bin/Release/net10.0/Gravitas.Benchmarks.dll static-population-refresh --filter '*' --job Short
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj -c Release -p:UseLocalLsfStack=true -p:DisableTransitiveProjectReferences=true --collect:'XPlat Code Coverage' --settings tests/Gravitas.Tests/coverlet.runsettings
# Repeat test/coverage with -c ReleaseLean; build src/Gravitas in both configurations and target frameworks.
```

Ignored `artifacts/grv-benchmark-030/` retains `heads.json`, frozen
`baseline/`/`final/` payloads, loaded bindings, driver source, A/A
clone/captures/summary, profile, full raw coverage and the eight
`N-variant-{measure,controls}.json` launch captures. `validated-summary.json`
and `verified.log` check DLL parity, fixed trajectories, bindings,
launch/row/window completeness and allocation. The maintained fixture, inputs,
source revisions, protocol, results and limitations above retain the conclusion
independently of those disposable artifacts.

#### Remaining scope and reopening

The retained O(population * hierarchy-depth) sampling pass is intentional:
`FixedTransform` has mutable local components and ancestry with no revision or
dirty-propagation contract. A static label cannot prove a bodyless host
transform unchanged. The existing snapshot still avoids republishing identical
geometry; introducing a transform cache or another collider index is not
justified by this bounded signal. General-scale/rotated transforms retain their
existing costs. A matching final 12-second sampled full-step profile attributes
13.43% to partition preparation and 62.29% to discrete response solving. These
overlapping, call-site-sensitive sample weights confirm the remaining dominant
work is ordinary contact response; they are not additional acceptance timings.
Reopen with a reproducible host budget exceeded by the retained sampling work,
an actionable fallback regression, changed admission/witnesses/replay, or warm
allocation. #031 remains separate ordinary open-mesh mixed full-step work.
Native cross-platform and released-package validation were not rerun; all
results here use the recorded coordinated local source stack.

#### Historical intake (2026-10-19; prior stack)

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

### GRV-Benchmark-029 — Dense Mesh Sweep Cost After Exact Contact Hardening

**Status:** Closed 2026-10-31. Ordinary whole-patch sweep work and repeated
identical-triangle work are reduced without replacing finite surfaces with an
infinite plane, changing tolerances or weakening identity-sensitive admission.
This is a bounded optimization closeout, not a claim that arbitrary meshes or
complete queries now have triangle-independent cost. #031 remains open: it
measures a different, complete mixed simulation workload.

#### Executed cause and retained changes

The existing BVH admits all 128/2,048 triangles in the maintained dense quad
fixtures, despite their identical physical surface. Managed stack-sampling
controls include one second of warmup and a 12-second loop, excluding fixture
setup. Baseline controls attribute 94.7% of sphere loop weight to triangle sweep
work, including 74.8% in the wide sphere distance interval; 93.7% of the convex
loop to GJK distance; and 99.3%/98.7% of the mixed dense/false-positive loops to
finite triangle/circle-slab sweeps. These inclusive weights overlap; they are
not additive CPU shares or acceptance timings.

- The existing certified-patch GJK attempt now admits convex mesh sources. Their
  reference-center chord only selects an attempt; GJK still tests the actual
  hull. `ConvexShape.ContainsCenter` remains false for mesh sources.
- Ordinary sphere sweeps use the certified patch's corner fan with the existing
  analytic kernels when all triangles are candidates and initial source bounds
  are disjoint. Initial contacts, identity-sensitive CCD/supplier admission,
  partial candidate sets and uncertified/nonconvex domains retain authored
  processing.
- Replacement sphere fan faces use cached/robust mesh normals and the existing
  exact-zero-degenerate wide projection predicate. This preserves admitted micro
  faces when internal diagonals disappear. Authored fallback retains its
  existing projection predicate, avoiding unrelated nonfan overhead.
- Mixed circle-slab sweeps use **one analytic carrier triangle** only when exact
  world-plane arithmetic certifies an aligned face hit. X/Z motion must be
  parallel to the nonzero projected plane normal, with nonzero approach. The
  exact approach/vertical signs choose the supporting cap, or axial midpoint
  when the vertical coefficient is zero. Its center chord must cross the finite
  fan, and the published normal must retain a nonzero radial component. Before
  whole-plane entry the complete cylinder is separated. Its entering radial
  extreme follows that cap-center line, so the finite crossing proves the
  plane-entry lower bound is attained by the containing triangle. That
  triangle's existing kernel supplies distance; the crossing supplies paired
  final rounded physical witnesses and the geometric plane normal. TOI and
  witness rounding can leave a small separation at the retained time. This
  removes contact reconstruction entirely. Oblique/rim-only/tangent casts and
  failed certificates stay authored; a declined carrier kernel falls through to
  every original candidate without publishing its synthetic index.
- Consecutive identical ordered triangles reuse only the analytic kernel result
  inside one ordinary mixed query. The frame, chord and slab inputs are fixed
  within that attempt, every authored index still participates in the authored
  reduction, and new queries start with no memo. There is no persistent cache or
  invalidation protocol.

Runtime growth is **105 net lines across three existing query files**, with no
new runtime files, public API, dependency or upstream math change. Counter
semantics remain unchanged: triangle counters report BVH-admitted authored
triangles, not the reduced number of narrow-phase kernel calls.

#### Frozen source and process provenance

Baseline Gravitas is `4e674b08eb7f2de4b036e5f717647fdbc4997251`. Both variants
use committed source archives of these exact dependencies:

| Repository       | Revision                                   |
| ---------------- | ------------------------------------------ |
| FixedMathSharp   | `c3fcd6bd035afdb4aeea776d2a6a6ca35723585f` |
| SwiftCollections | `0418123fa52f78cbff551e6ca291ef816493069b` |
| GridForge        | `76e2bd46380664cfa5403d3b57eca6a82cb6b88a` |
| Chronicler       | `c45f9ab53913e5b1692f0717733425e0f4defeb2` |

Concurrent Chronicler dependency/migration edits in Gravitas, SwiftCollections
and GridForge, including the separately recorded package blocker #149, are
excluded from this frozen graph and preserved in the shared worktrees. The
candidate graph changes only the three query files and focused source tests.
These are source-stack results, not validation of those pending metadata edits
or a released package combination.

Windows x64, OS build `10.0.26300`, i7-9700K, SDK `10.0.302`, .NET `10.0.10`,
Release, `UseLocalLsfStack=true`, `DisableTransitiveProjectReferences=true`,
`DOTNET_PROCESSOR_COUNT=2`. The disposable direct driver uses blocking
workstation GC, high priority and affinity mask 192 (logical CPUs 6/7). Builds,
tests, profiles and timings run serially.

All payload DLLs match except `Gravitas.dll`; actual loaded assembly names,
paths and SHA-256 hashes are checked in every measured process. Loaded
identities include Gravitas `0.0.0.0`, FixedMathSharp `7.1.0.0`,
SwiftCollections `7.0.0.0`, SwiftCollections.FixedMathSharp `7.1.0.0`, GridForge
`9.1.0.0`, and Chronicler / Chronicler.Diagnostics `1.0.0.0`. Package-looking
versions alone do not identify these source-built assemblies.

| Measured module                 | SHA-256                                                            |
| ------------------------------- | ------------------------------------------------------------------ |
| Gravitas baseline               | `C5C9E8F27C3E0DF38324EE58975A6335FD5B880F8645AABCD83C806CE0A2A5A9` |
| System.Private.CoreLib          | `76A49F17E83F613BC8A5B55FC98830B764E7838265AACBDBCC8F46707F80A3C9` |
| FixedMathSharp                  | `E8002520AC021DA7AEBCFE5B790AA739E916A5AA761D0350136DEBA94CCE28E6` |
| Gravitas final                  | `03A321425050490058A53DE31A852F6A26687689AEAED67F06ED91C915B8EB2D` |
| Chronicler                      | `F2D894CB20F5C734510DE9E0B08CAB04A45D05BAAEF20F04D2B1CEFEB1CB9A88` |
| SwiftCollections                | `3BE058EDBDDFBE5E0A26A55DA4295741A4AF1FE357D34E75AD5A9D32F30EE610` |
| GridForge                       | `743CC1BCDF4FE7E2E5E6248ED1B487053B74EE9E1F689C2EB2AACB2327264324` |
| Gravitas.Benchmarks             | `8B6C2EF6371DB43D70BA38BF4DCBEA2DFE57B4733674E279D26AAA94501DE2F5` |
| Chronicler.Diagnostics          | `B401C40A2E669F7F214DB60AF7C09A0AACB006C83E6C313BF6462DC65727E020` |
| SwiftCollections.FixedMathSharp | `6D2CBD38F41874322530011C8117273BC72233B8FDA85DC151E1EE7C2FE844B2` |
| FixedMathSharp.Chronicler       | `61D49349E5D1D7A7C0A05CB140F53E3EA9D7A1C3500953745383F5DEA2755ED7` |

The Release coverage/test DLL and final measured DLL have the same SHA-256. No
instrumented assembly is used for timing.

#### Matched public-query measurements

The direct driver invokes the existing maintained benchmark methods after their
normal `Setup`, and runs `Cleanup` outside measurement. Each row warms for four
seconds, calibrates for at least 200 ms, and records 16 windows targeting 50 ms
each (one to 4,096 invocations/window). The independent process order is
**baseline, final, final, baseline**, with both dense rows and nonfan controls
in each round. Tables give equally weighted means of the two process means; the
ranges retain both process means. Allocation is the current-thread byte delta
around every window. These are synthetic warm operation means, not gameplay
frame-tail latency, cold-start cost or scene-capacity evidence.

The maintained dense geometry is the vertical quad `X=0, Y/Z=+-0.5`, subdivided
8/32 times into 128/2,048 triangles. Probes travel `(-3,0,0)` to `(3,0,0)`;
sphere/circle radius and mixed half-thickness are 0.5. The convex source is the
unit closed cube with half-extents 0.5; its target is concave. The
false-positive fixture repeats the ordered triangle
`(0,-1,0.49),(0,1,0.71),(0,-1,1)` 128/2,048 times with separate vertex slots,
without changing physical geometry.

| Public all-hit query / triangles | Baseline mean ms [launch range] | Final mean ms [launch range] |  Change |
| -------------------------------- | ------------------------------: | ---------------------------: | ------: |
| Sphere / 128                     |             1.945 [1.923–1.967] |       0.0940 [0.0937–0.0943] | -95.17% |
| Convex mesh / 128                |             4.911 [4.882–4.940] |       0.0353 [0.0351–0.0354] | -99.28% |
| Mixed circle / 128               |             2.582 [2.563–2.601] |       0.0526 [0.0522–0.0531] | -97.96% |
| Mixed false-positive / 128       |             3.338 [3.319–3.356] |       0.0457 [0.0451–0.0464] | -98.63% |
| Sphere / 2,048                   |          26.174 [25.941–26.406] |       0.8897 [0.8890–0.8904] | -96.60% |
| Convex mesh / 2,048              |          79.237 [78.215–80.260] |       0.0700 [0.0698–0.0702] | -99.91% |
| Mixed circle / 2,048             |          38.389 [38.044–38.734] |       0.0897 [0.0894–0.0900] | -99.77% |
| Mixed false-positive / 2,048     |          53.523 [53.516–53.529] |       0.1572 [0.1545–0.1600] | -99.71% |

All windows allocate **0 bytes**. Collider count remains one and authored
triangle candidate counts remain 128/2,048. Dense hits keep raw distance
`10737418240` (2.5); false-positive rows remain misses. Sphere fields match
exactly. Convex and mixed paths may change finite physical witnesses; convex
points remain on both the cube's leading face and target patch, and mixed paired
witnesses retain the physical supporting-face normal. Legacy dense mixed output
uses an invalid center-to-witness diagonal normal; the final face normal is
world -X. Repeated same-version queries retain exact outputs and collider
ordering. Cross-version query/replay result hashes need not match when these
fields change.

Nonfan controls use two explicit finite layouts: a bent four-triangle strip with
vertices `(0,-1,-1),(0,1,-1),(0,-1,0),(0,1,0),(0.5,-1,1),(0.5,1,1)` and indices
`0,1,2;2,1,3;2,3,4;4,3,5`; and an eight-triangle coplanar square ring, outer Y/Z
corners `+-1`, inner corners `+-0.25`, with the center left empty. They use the
same chord/radius/slab and closed cube source. The full-step control is
`JoinedStaticSolidMixedSimulationBenchmarks`, `Shape=OpenMesh`,
`Layout=Continuous`; invoke its existing 64-step operation, divide time by 64,
and retain its per-step tangential-motion/nonpenetration assertions. Fixed
post-setup observations agree before timing; all nonfan query fields agree.

Small-patch mixed controls use the same fixed dense quad with subdivisions 1/2/4
(2/8/32 triangles). The five four-triangle controls use corners
`(0,-1,-1),(0,1,-1),(0,1,1),(0,-1,1)` plus center `(0,0,0)`, indexed
`0,1,4;1,2,4;2,3,4;3,0,4`, with radius/half-thickness 0.5. Their chords are:

| Four-triangle layout | Start         | End          |
| -------------------- | ------------- | ------------ |
| Face                 | `(-3,0,0)`    | `(3,0,0)`    |
| Oblique rim          | `(-3,0,-0.2)` | `(3,0,2.8)`  |
| Tangent              | `(-0.5,0,-3)` | `(-0.5,0,3)` |
| Partial-height slab  | `(-3,1.25,0)` | `(3,1.25,0)` |
| Tilted face          | `(-3,0,0)`    | `(3,0,0)`    |

The tilted layout instead uses `(1,-1,-1),(-1,1,-1),(-1,1,1),(1,-1,1)` plus
center zero with the same indices, giving plane X+Y=0.

| Control                             |   Baseline us [launch range] |      Final us [launch range] |  Change |
| ----------------------------------- | ---------------------------: | ---------------------------: | ------: |
| Certified mixed quad / 2 triangles  |       73.088 [73.087–73.088] |       71.778 [71.574–71.982] |  -1.79% |
| Certified mixed quad / 8 triangles  |    208.071 [207.260–208.882] |       49.379 [49.325–49.433] | -76.27% |
| Certified mixed quad / 32 triangles |    744.821 [742.227–747.414] |       49.776 [49.622–49.929] | -93.32% |
| Four-triangle face                  |    155.333 [153.402–157.264] |       49.612 [49.410–49.814] | -68.06% |
| Four-triangle oblique rim           |    136.348 [135.897–136.799] |    137.969 [137.269–138.669] |  +1.19% |
| Four-triangle tangent               |    116.185 [116.019–116.351] |    117.051 [116.552–117.551] |  +0.75% |
| Four-triangle partial-height slab   |       74.484 [74.387–74.581] |       73.840 [73.810–73.871] |  -0.86% |
| Four-triangle tilted face           |    219.554 [218.532–220.576] |       68.892 [67.824–69.960] | -68.62% |
| Bent strip / sphere                 |       67.909 [67.734–68.083] |       67.367 [66.971–67.762] |  -0.80% |
| Bent strip / convex mesh            |    173.955 [172.954–174.955] |    172.514 [172.175–172.854] |  -0.83% |
| Bent strip / mixed circle           |    139.903 [136.984–142.821] |    143.748 [142.256–145.240] |  +2.75% |
| Square ring / sphere                |    164.516 [164.316–164.717] |    164.214 [163.757–164.671] |  -0.18% |
| Square ring / convex mesh           |    240.590 [239.965–241.214] |    238.740 [237.850–239.631] |  -0.77% |
| Square ring / mixed circle          |    321.082 [319.067–323.097] |    323.245 [321.152–325.338] |  +0.67% |
| Open-mesh host step (one step)      | 4641.734 [4595.663–4687.806] | 4569.367 [4563.603–4575.131] |  -1.56% |

The dense gains hold in both launch orders. Small controls do not establish an
equivalence result: the bent mixed mean is +2.75% (+3.85 us), with overlapping
launch ranges; the oblique rim is +1.19% (+1.62 us), tangent +0.75% (+0.87 us),
and ring mixed +0.67% (+2.16 us). Preserve these positive estimates rather than
calling them zero overhead. The remaining control point contrasts range down to
-1.79%; no speedup is claimed for unchanged paths or the full-step control.

A subsequent identical-binary baseline/clone comparison uses the same driver,
affinity, warmup/windows and actual binding checks in four fresh processes
(controls and rims in each variant). Across the control set, sham contrasts
range from -2.85% to +2.13%, while the original baseline bent-mixed process
means span 136.98–142.82 us. These expose process/environmental uncertainty, not
an isolated cause or proof that each positive contrast is noise. These runs do
not settle the small positive estimates; do not infer exact zero cost or a
control speedup. Reopen small-control attribution if longer matched runs
establish a reproducible cost that matters to the host's query budget.

| Identical-binary control | Baseline us | Clone us | Sham contrast |
| ------------------------ | ----------: | -------: | ------------: |
| Bent2                    |     142.302 |  143.843 |        +1.08% |
| Ring2                    |     321.266 |  328.094 |        +2.13% |
| OpenMeshFullStep64       |    4564.768 | 4606.623 |        +0.92% |
| PatchRim                 |     134.634 |  134.341 |        -0.22% |
| PatchTangent             |     115.989 |  116.974 |        +0.85% |

Earlier, rejected controls are retained. Full finite-face reconstruction costs
about 23 ms at 2,048 triangles. A corner-face build costs 3.20–3.28 ms at 128
triangles versus 2.613–2.646 ms baseline. Its plane-support replacement still
costs 349–351 us at eight triangles versus 211–212 us baseline. A scalar-support
replacement reduces that face cost but raises the four-triangle rim control from
about 136 to 241 us and tangent control from 116 to 191 us before authored
retry. All reconstruction helpers and temporary scratch state are removed by the
retained face certificate. An unconditional wide sphere projection adds about
10–15% in bent/ring controls; the predicate is now confined to replacement fans.
These adverse results are not folded into final acceptance averages.

#### Correctness, coverage and reproduction

`CertifiedMeshSweepTests` adds 34 maintained production-behavior cases: three
source families, both windings, common yaw, affine half-raw placement,
near-limit translation, exact repeat/order checks, finite rims and misses,
tilted/horizontal faces, tangential entry, offset slabs, partial candidates,
conservative initial bounds, duplicate hits/misses, changed radius, reversed
sides, nearly aligned casts and endpoint radius hits without a center crossing.
A tetrahedral source whose inspection center lies outside its hull verifies that
the new GJK attempt does not invent a center contact.

The micro regression uses a quad with Y/Z half-extents 0.005, sphere radius
0.00025 and chord X=-0.01 to +0.01 at Y=+0.0002, Z=-0.0002. The prototype misses
after removing the internal diagonals because the public triangle normal and
projection use a squared-area epsilon; baseline and final hit. A vertical quad
with Y/Z half-extents 0.5 and a slab centered at Y=-0.5 reproduces the
prototype's invalid diagonal normal; final publishes -X. Centered/+0.5 slab
variants also expose the legacy normal issue. These regressions were verified
before accepting the corrections; no claim is made about every legacy fallback
normal or every small authored triangle predicate.

Both library TFMs (`netstandard2.1`, `net10.0`) build in Release and ReleaseLean
with zero errors. Full suites pass **8,327 Release / 8,256 ReleaseLean tests**,
without failures or skips. Each final configuration uses one fresh full-suite
coverage capture, with no merged partial or historical acceptance coverage:

| Configuration | Raw sequence points |    Raw branches |   Raw methods |
| ------------- | ------------------: | --------------: | ------------: |
| Release       |     53,540 / 53,540 | 21,766 / 21,766 | 5,602 / 5,602 |
| ReleaseLean   |     53,538 / 53,538 | 21,766 / 21,766 | 5,601 / 5,601 |

Recreate the source graph at the revisions above, retain committed project
files, and apply this change. Build/test with
`-p:UseLocalLsfStack=true -p:DisableTransitiveProjectReferences=true`; source
mode expects all siblings. The maintained workloads can be reproduced through
BenchmarkDotNet as follows; the direct ABBA protocol above, rather than a BDN
short-job mean, supplies this record's acceptance numbers:

```powershell
$env:UseLocalLsfStack = 'true'
$env:DisableTransitiveProjectReferences = 'true'
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -p:UseLocalLsfStack=true -p:DisableTransitiveProjectReferences=true -m:1 -p:BuildInParallel=false -p:UseSharedCompilation=false -nr:false
dotnet tests/Gravitas.Benchmarks/bin/Release/net10.0/Gravitas.Benchmarks.dll --filter '*MeshQuery3DTriangleScalingBenchmarks.SweepSphereAll_DenseTriangleMeshTarget' '*MeshQuery3DTriangleScalingBenchmarks.SweepConvexMeshAll_DenseConcaveTriangleMeshTarget' '*MixedMeshTriangleScalingBenchmarks.SweepCircleAgainst3DAll_DenseTriangleMeshTarget' '*MixedMeshTriangleScalingBenchmarks.SweepCircleAgainst3DAll_FalsePositiveTriangleMeshTarget' --job short
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj -c Release -p:UseLocalLsfStack=true -p:DisableTransitiveProjectReferences=true --collect:'XPlat Code Coverage' --settings tests/Gravitas.Tests/coverlet.runsettings --results-directory artifacts/coverage-release
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj -c ReleaseLean -p:UseLocalLsfStack=true -p:DisableTransitiveProjectReferences=true --collect:'XPlat Code Coverage' --settings tests/Gravitas.Tests/coverlet.runsettings --results-directory artifacts/coverage-lean
```

Captures under `artifacts/grv-benchmark-029/` contain source
archives/`heads.json`, raw baseline/final profiles, rejected `accept-*`,
`final-*`, `small-*` and `rims-*` controls, accepted `carrier-*` timings and
actual bindings, identical- binary `aa-*` controls, focused regressions,
full-suite logs and the separate `coverage-{release,lean}-carrier` reports.
Disposable probes stay outside maintained tests/benchmarks; the inputs,
revisions, protocol, results and limits above remain tracked if those artifacts
are deleted. Independent connected-source review found no unresolved
correctness/complexity findings after the revisions.

#### Remaining cost and reopening criteria

Final managed stack samples put 95.8% of the dense sphere loop under hit
publication and 90.4% under swept-sphere contact resolution, including 64.5% in
per-triangle closest-point reduction; only 5.4% remains in the mesh sweep
worker. Mixed attribution now includes 32.8% finite chord intersection, 30.6% in
the single analytic carrier and 21.9% in mesh triangle gathering. These
inclusive weights overlap and include the same one-second warmup plus 12-second
loop; they are attribution, not operation-time fractions to add together.

BVH gathering and candidate scans remain proportional to admitted triangles.
Sphere publication still resolves the globally closest finite feature through
`ContinuousCollisionContactPolicy` / `LSMeshCollider.FindClosestPointAnchor`;
the impact worker returns only center/distance. Reusing its winning analytic
triangle alone is insufficient after rounded TOI. A future exact closest-point
fan scan needs explicit witness/normal ranking, tie, compound and anchor tests;
it is not an omitted one-line reuse.

Reopen for a maintained complete-query or representative host workload where
that residual exceeds the host's query budget, for expensive uncertified mesh
domains outside this certificate, or for a finite-witness/ordering/allocation
regression. Keep #031's ordinary mixed full-step attribution separate. This
closeout establishes these measured workloads on the frozen Windows source
stack, not released-package compatibility or all OS/architecture behavior.

#### Historical intake and earlier refinement

The following intake and #026 comparison retain their original dated status and
runtime; they are historical evidence, not the final measurements above.

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

### GRV-Benchmark-028 — Mixed Broad-Phase Work With An Empty Opposite Dimension

**Discovered:** 2026-10-15 during #135/#136 warm validation. **Closed:**
2026-10-31. **Owner:** Gravitas mixed candidate distribution.

`ProcessMixedContacts` now skips copying, sorting and distributing active mixed
partitions when either live dimensional collider registry is empty. Index
refresh, scratch clearing, version advancement, response cleanup, untouched-pair
cleanup and retained-partition retirement still run. The next populated pass
uses the ordinary deterministic distribution path. One private phase method sits
beside its sole production caller; no counter, cache, public API or lower-stack
change is added. Production growth is nine net lines in one existing file.

The bounded query-support fixture is now maintained as
`MixedEmptyDimensionBenchmarks`. Each operation runs both `Simulate` and
`LateSimulate` with 64 sleeping movers and 64 static floors in one dimension,
zero colliders in the other, at 16 Hz. An eight-by-eight grid spaces movers by
eight units. Three-dimensional spheres have radius 1/2 and local offset Up 1;
their 6×1×6 floors are centered 1/2 below the body origin. Planar circles use
the corresponding forward offset and 6×1 floor. Automatic grounding crosses a
real 1/2-unit collider gap; there are no supporting contact manifolds. Setup
settles 16 steps, explicitly sleeps the movers and warms 33 more steps. Setup
and cleanup verify stable grounded sleep, zero velocity, the physical gap,
unchanged heights and the empty opposite registry. Pure 3D/2D modes are
controls.

#### Comparable source and measurement

Baseline is Gravitas `76dd8bfd1b16b7b23acc1fc8b3440cb890a62c26`, plus the new
unchanged benchmark fixture. Both versions use frozen committed FixedMathSharp
`b4a489de5ae5ba178a5970499e7d90f7847fe0f9`, SwiftCollections
`0418123fa52f78cbff551e6ca291ef816493069b`, GridForge
`76e2bd46380664cfa5403d3b57eca6a82cb6b88a` and Chronicler
`c45f9ab53913e5b1692f0717733425e0f4defeb2`. Source archives preserve the
baseline; the isolated build graph is updated only after freezing baseline
binaries and completing baseline benchmarks/tests. The candidate changes only
the candidate-distribution phase. Among the 35 DLLs in the direct A/B payloads,
only `Gravitas.dll` differs; module hashes are retained.

Windows 11 x64, i7-9700K, SDK 10.0.302, .NET 10.0.10, Release,
`UseLocalLsfStack=true`, `DisableTransitiveProjectReferences=true`,
`DOTNET_PROCESSOR_COUNT=2`. Builds, tests, profiling and acceptance timings run
serially. BenchmarkDotNet 0.15.8 uses concurrent workstation GC, two launches,
five warmups and eight measurements with 512 invocations per iteration, unroll
one. No BDN affinity is requested. These are synthetic iteration means, not
gameplay frame percentiles or a general host-capacity guarantee.

```powershell
$env:UseLocalLsfStack = 'true'
$env:DisableTransitiveProjectReferences = 'true'
$env:DOTNET_PROCESSOR_COUNT = '2'
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -p:UseLocalLsfStack=true -p:DisableTransitiveProjectReferences=true -m:1 -p:BuildInParallel=false -p:UseSharedCompilation=false -nr:false
dotnet tests/Gravitas.Benchmarks/bin/Release/net10.0/Gravitas.Benchmarks.dll --filter '*MixedEmptyDimensionBenchmarks*' --job short --warmupCount 5 --iterationCount 8 --launchCount 2 --invocationCount 512 --unrollFactor 1 --exporters json --artifacts artifacts/benchmark-028
```

| Complete step                  | Baseline mean (SD) |  Final mean (SD) | Change | Allocated |
| ------------------------------ | -----------------: | ---------------: | -----: | --------: |
| 3D population, Mixed runtime   |  837.96 (30.05) us | 485.33 (3.54) us | -42.1% |    0 B/op |
| 3D population, pure 3D control |   413.02 (5.22) us | 399.08 (2.55) us |  -3.4% |    0 B/op |
| 2D population, Mixed runtime   |  687.84 (19.21) us | 441.28 (3.77) us | -35.8% |    0 B/op |
| 2D population, pure 2D control |   367.92 (9.03) us | 352.46 (2.06) us |  -4.2% |    0 B/op |

Every BDN child exits zero: eight children and exactly 64 Workload/Actual
measurements per payload (four rows × two launches × eight iterations). Pure
runtime changes are controls, not claimed improvements.

A separate disposable driver uses blocking GC, affinity 192 and High process
priority, six seconds of warmup per row, 200 ms calibration and 24 windows
targeting 30 ms each (1–4,096 steps per window). ABBA processes cover the same
four complete-step rows plus maintained sparse mixed broad-phase controls with
32 and 1,024 registered pairs. These existing controls eventually auto-sleep:
their steady broad-phase candidate count is zero, so they measure populated,
sleeping distribution overhead rather than awake pair generation. All measured
windows allocate zero bytes. The four supported-sleep rows retain all sixteen
returned checksums and all 64 final body positions, linear velocities and
grounding/sleep flags. The sparse controls retain their returned candidate
counts. These comparisons match exactly across all four processes. Final
balanced means:

| Direct complete step                    | Baseline us | Final us |  Change |
| --------------------------------------- | ----------: | -------: | ------: |
| 3D / pure                               |      395.91 |   397.45 |  +0.39% |
| 3D / Mixed                              |      704.30 |   492.13 | -30.13% |
| 2D / pure                               |      364.22 |   361.84 |  -0.65% |
| 2D / Mixed                              |      659.23 |   451.71 | -31.48% |
| Sleeping populated control, 32 pairs    |      204.06 |   205.42 |  +0.67% |
| Sleeping populated control, 1,024 pairs |    17880.70 | 17970.28 |  +0.50% |

The sleeping controls' process ranges are 203.98–204.13 us baseline versus
204.79–206.04 us final at 32 pairs, and 17.789–17.973 ms versus 17.699–18.242 ms
at 1,024. The smaller isolated phase check below has overlapping process ranges.
These sub-percent mean differences do not prove exact equivalence; they do not
reproduce the initial 2–3% penalty. Protocols differ in GC, affinity and warmup,
so compare payloads within each protocol rather than mixing absolute means.

The initial inline guard repeatedly slowed the 32-pair sleeping control:
206.44/206.20 us baseline versus 210.36/211.45 us candidate in the full matrix
(+2.2%), then 188.83/189.13 versus 195.59/194.62 us in isolated BAAB processes
(+3.2%). Those results are retained as adverse evidence, not called noise or
zero overhead. A diagnostic with TieredPGO disabled reduces the gap to 0.5%;
evaluating both Boolean comparisons without short circuiting does not resolve
it. Neither experimental configuration nor Boolean rewrite is retained.
Extracting the existing distribution phase gives overlapping default-runtime
process ranges: 187.91–191.33 us baseline versus 189.53–191.48 us final (+0.5%
mean). This is a measured mitigation, not proof of a particular JIT/PGO/layout
cause. The final full matrix checks both population directions and preserves the
original controls; no universal zero-overhead claim is made.

Fresh sampled-thread-time profiling reproduces the original signal: 43.9% of
inclusive full-step loop weight is below `ProcessMixedContacts` in the baseline
3D-only scene. The guarded profile puts 24.9% below mixed processing, almost
entirely in index refresh (24.8%). The driver setup is outside the sampled
method; the loop includes its additional 1.5-second warmup and 15 seconds of
repeated steps. Inclusive weights overlap and are not exclusive CPU shares. The
remaining mixed indexing serves explicit queries and future population changes;
it is not claimed to disappear or reach pure-runtime cost.

#### Correctness, validation and closure

The 44 focused mixed broad-phase cases pass against the baseline and candidate,
including six new source-behavior cases for both empty-dimension directions.
They check query candidates after movement, version advancement, retained
partition retirement after the final removal, repeated last-collider removal
inside trigger callbacks and respawn, stale candidate clearing, pair/exit event
counts and zero warm allocations. No tests or maintained verification harnesses
around benchmark/test infrastructure are added.

Full source-stack gates pass **8,293 Release tests** and **8,222 ReleaseLean
tests**, with no failures or skips. Both library targets (`netstandard2.1` and
`net10.0`) build in both configurations with zero errors. Each configuration has
one fresh coverage capture; no historical coverage is merged into acceptance.
All reachable sequence points, branches and methods are covered:

| Configuration | Raw covered / total sequence points | Raw covered / total branches | Raw covered / total methods |
| ------------- | ----------------------------------: | ---------------------------: | --------------------------: |
| Release       |                     53,477 / 53,477 |              21,710 / 21,710 |               5,602 / 5,602 |
| ReleaseLean   |                     53,475 / 53,475 |              21,710 / 21,710 |               5,601 / 5,601 |

Reproduce full-suite coverage from either configuration's freshly built source:

```powershell
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj -c Release -p:UseLocalLsfStack=true -p:DisableTransitiveProjectReferences=true --collect:"XPlat Code Coverage" --settings tests/Gravitas.Tests/coverlet.runsettings --results-directory artifacts/coverage-release
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj -c ReleaseLean -p:UseLocalLsfStack=true -p:DisableTransitiveProjectReferences=true --collect:"XPlat Code Coverage" --settings tests/Gravitas.Tests/coverlet.runsettings --results-directory artifacts/coverage-lean
```

The Release execution reused built tests with the final runtime DLL/PDB
installed; its runtime DLL exactly matches the final measured payload. Lean
compiles its own configuration. Source/test bytes match the isolated graph.
Existing replay, allocation, query, response and lifecycle regressions remain in
the full runs.

Independent review found no actionable runtime/test/benchmark issue. This closes
the impossible-distribution signal; remaining mixed index refresh is required
work. Reopen for a representative empty-population workload that still spends
material time in distribution, a measured populated-path regression, stale query
results, missed post-registration pairs, nondeterministic order,
retained-payload leaks, incorrect callback lifetime behavior or new warm
allocation.

An additional awake control wraps two successive
`MixedBroadPhaseBenchmarks.RetainedPartitionCleanupAfterChurn` calls per
measured operation, moving 32 planar bodies between the existing fixture's
16/32-unit X offsets. This keeps the mixed distribution path populated and ends
each cycle with **32 broad-phase candidates**. Default-runtime BAAB process
means are 1,380.04/1,367.35 us per two-step cycle baseline versus
1,416.80/1,367.99 us final (+1.4% aggregate, overlapping process ranges). Every
window allocates zero bytes; all sixteen retained/pool return values and final
candidate/collider counts match. The variation is retained without claiming
exact equivalence or a speedup for populated moving scenes. The disposable
`churn` driver mode reuses the same warmup, calibration, window, GC, affinity
and priority protocol as the other direct runs.

Supplemental ignored evidence is under `artifacts/grv-benchmark-028`:
`heads.json`, five source archives, frozen baseline/final payloads,
`final-payload-hashes.json`, the exact `final-matrix-driver`, BDN JSON and child
logs, ABBA direct logs/driver, paired Speedscope traces, focused runs and final
raw coverage captures. The maintained fixture and the revisions, inputs,
commands, outcomes and reopening criteria here stand independently of those
disposable artifacts. Changes were left unstaged/uncommitted for review; no
released-package or cross-platform validation is claimed.

#### Historical intake

The following original profile describes the pre-#135/#136 stack; its timings
are not mixed with the current comparison above.

**Discovered:** 2026-10-15 during #135/#136 warm validation. **Original intake
status:** Profiled pre-existing opportunity; the benefit of a distribution gate
has not been measured. **Owner:** Gravitas mixed candidate distribution and
partition maintenance.

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

**Next isolation at intake:** Promote the bounded fixture into the maintained
benchmark suite, then measure skipping only impossible candidate distribution
while preserving partition/query freshness, pair retirement, deterministic
ordering and `0 B` warm behavior. Exercise both empty-dimension directions and
repeated spawn/despawn transitions. Keep the #135/#136 regression gate
independent of this pre-existing follow-up.

Ignored `artifacts/grv-issue-135/perf/` retains the frozen original eight-module
stack (`89C15FBD...` core), unchanged fixture inputs, native rows,
`profile-attribution.json`, paired CPU traces and zero-ground-probe diagnostics.

### GRV-Benchmark-027 — Nonconvex Rotational CCD Step Cost

**Discovered:** 2026-10-12 during GRV-Issue-101 validation. **Closed:**
2026-10-31 as measured cost hardening and a bounded no-further-change decision.
The conservative capsule progress limit below remains reproducible; closure does
not claim unrestricted rotational capacity or a host frame budget. **Owner:**
Gravitas rotational interval search and finite-contact admission.

#### Retained implementation

The first refinement, committed as Gravitas `88eddd5`, skips same-dimensional
candidate gathering only when the live registry contains the source root alone.
The continuation removes repeated contact materialization:

- Standalone open/concave mesh versus cone interval samples use the existing
  exact finite triangle/cone Boolean predicate in the mesh geometry chart. One
  local plane frame is reused for consecutive triangles with the same canonical
  owner. Closed convex containment and declared static-surface response
  ownership retain their complete existing admission path.
- The complete matched cone manifold and closing velocity are reconstructed at
  the retained witness only when its time equals the admitted safe frontier.
  Moving source and target are both sampled at that exact time. A later witness
  cannot authorize an impulse at an earlier unresolved frontier.
- Both mixed directions apply the same frontier equality before reconstructing
  response rows. T0 response, geometric upper bounds, candidate order,
  conservative terminal classification and retained momentum are unchanged.

No budget, tolerance, response deadband, support-pair approximation, cross-call
cache or public API is added. The 3D interval method moves to its existing
search partial; the five production files grow by **34 net lines**. No
lower-stack source change is retained.

#### Frozen comparison and reproducible procedure

Continuation baseline is Gravitas `88eddd5` with FixedMathSharp
`0e7a8e5760c506db8050714b72fad2421dcd46e6`, SwiftCollections
`d605ec1ebb74baa7d7db929699f68730f9392ada`, GridForge
`76e2bd46380664cfa5403d3b57eca6a82cb6b88a`, and Chronicler
`0a5f962ef47d6b0da52bcfadd7fac400cb4397c0`. Source archives isolate builds from
concurrent, unrelated workspace cleanup. Final production/tests in the archive
match this change byte-for-byte. Among the 35 DLLs in the direct A/B measurement
payloads, **only `Gravitas.dll` differs**. This is source-stack validation, not
validation of the subsequently edited CI dependency pin or a released package.

Host: Windows 11 x64, i7-9700K (8 cores), SDK 10.0.302, .NET 10.0.10, Release,
`UseLocalLsfStack=true`, `DisableTransitiveProjectReferences=true`,
`DOTNET_PROCESSOR_COUNT=2`. Acceptance timings run without competing builds,
tests or profilers. BenchmarkDotNet 0.15.8 uses concurrent workstation GC;
disposable direct drivers use blocking GC. BDN requested affinity 192 but every
child reported that setting failed; **these BDN runs are unpinned**. The later
direct probes successfully set affinity 192 and High process priority.

Build and run each frozen graph separately; carry the local-stack properties
into generated benchmark children through the environment. Representative
commands:

```powershell
$env:UseLocalLsfStack = 'true'
$env:DisableTransitiveProjectReferences = 'true'
$env:DOTNET_PROCESSOR_COUNT = '2'
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -p:UseLocalLsfStack=true -p:DisableTransitiveProjectReferences=true -m:1 -p:BuildInParallel=false -p:UseSharedCompilation=false -nr:false
dotnet tests/Gravitas.Benchmarks/bin/Release/net10.0/Gravitas.Benchmarks.dll --filter '*NonconvexRotationalCcdBenchmarks.JoinedFloorAndWallFirstImpact*' '*NonconvexRotationalCcdBenchmarks.FloorOnlyFirstImpact*' --job short --warmupCount 5 --iterationCount 8 --launchCount 2 --invocationCount 128 --unrollFactor 1 --affinity 192 --exporters json --artifacts artifacts/benchmark-027-confirmation
dotnet tests/Gravitas.Benchmarks/bin/Release/net10.0/Gravitas.Benchmarks.dll --filter '*RotationalMovingPairCcdBenchmarks.Mixed3DTo2DRotationalMovingPairs*' --job short --warmupCount 6 --iterationCount 10 --launchCount 2 --invocationCount 8 --unrollFactor 1 --affinity 192 --exporters json --artifacts artifacts/benchmark-027-moving-controls
```

Each maintained nonconvex operation includes a pose reset and a complete
`Simulate`/`LateSimulate` step, prescribing a 90-degree rotation at 1 Hz with
forces disabled and frictionless materials. They deliberately stress refinement;
their iteration statistics are not gameplay-frame percentiles.

| Complete host-step fixture          | Baseline |    Final |     Change | Warm allocation |
| ----------------------------------- | -------: | -------: | ---------: | --------------: |
| Joined cone floor/wall first impact | 6.596 ms | 2.350 ms | **-64.4%** |          0 B/op |
| Floor-only cone first impact        | 5.238 ms | 2.152 ms | **-58.9%** |          0 B/op |

Joined baseline/final standard deviations are 0.0681/0.0258 ms; floor deviations
are 0.0066/0.0631 ms. Each payload has four successful child exits and exactly
32 Workload/Actual measurements (two fixtures, two launches, eight iterations).
Fixed 128-invocation confirmations supersede the shorter auto-calibrated runs.

Twelve existing interval/moving controls cover pure 2D, pure 3D and both mixed
directions at 1/32 candidates or pairs. These methods run `LateSimulate` only.
Quiet direct ABBA runs retain every returned value and final raw body state at 0
B. Initial adverse means included +7.5% for the one-pair pure-2D moving row and
+6.3% for the 32-pair mixed 3D-source row. Focused checks did not reproduce
those slowdowns: six-second-warmup BAAB pure-2D process means are 424.93/433.37
us baseline and 431.93/430.68 us final (+0.5% aggregate, overlapping ranges).
The mixed 32-pair BDN control is 63.983 ms baseline versus 58.733 ms final (SD
4.079/0.324 ms), at 0 B/op. Its one/eight-pair rows are 4.953/15.294 ms baseline
and 4.907/15.165 ms final; the one-pair row has large startup-sensitive
variation. Each payload has six successful children and 60 Actual measurements.
These controls support absence of a reproduced regression, not universal zero
overhead or an advertised speedup for unchanged paths. Adverse captures remain
part of the investigation.

Facet probes use a floor spanning X `[0,2]`, Z `[-2,2]`, divided into N by N
cells (two triangles each; N = 1/4/8/16), with a unit-mass kinematic cone at
`(3/5,11/20,0)`, radius 1/2 and height 1. Each operation resets pose, authors 1
or 90 degrees about -Z, and runs a complete step. After at least one second of
warmup and 200 ms calibration, each process measures 24 windows, choosing
1–4,096 operations per window to target approximately 30 ms. ABBA means:

| Floor triangles | 1 degree baseline → final | 90 degrees baseline → final |
| --------------: | ------------------------: | --------------------------: |
|               2 |        110.78 → 108.60 us |            5.321 → 2.061 ms |
|              32 |        140.05 → 139.42 us |            6.771 → 2.284 ms |
|             128 |        225.22 → 221.01 us |           10.267 → 3.260 ms |
|             512 |        548.59 → 536.15 us |           18.297 → 5.991 ms |

Every measured window allocates zero bytes. Sixteen returned values and final
pose/velocity/TOI snapshots per row match baseline exactly. Small-angle
differences are controls, not claimed improvements. The seven maintained
nonconvex full-step fixtures also retain their returned values and final raw
states.

#### Preserved capsule capacity boundary and stopping decision

The historical inputs below remain the reproduction. Fresh and warmed 12-frame
episodes explicitly set unit mass, zero gravity/air/damping, MinSpeed 0,
MaxSpeed/MaxFallSpeed 1,024, manual grounding, disabled sleep and frictionless
materials. The driver asserts that the prescribed initial impulse produces X
velocity 1; earlier probes lacking explicit unit mass are discarded.

The pressed planar capsule versus the vertical concave quad reaches the depth
frontier in **7/12 frames**. A measured operation includes reset, impulses and
all **12 complete steps**. Quiet ABBA process means are 2,214.618/2,124.080 ms
baseline versus 756.969/755.322 ms final: **2.169 s → 0.756 s per episode
(-65.1%)**, at zero warm allocation. This remains expensive and does not restore
free tangential progress. Every frame's raw state in both fresh and warmed
episodes matches baseline in all four processes.

Exact final Q32.32 states for the retained probes:

| Probe                                                            | Position                   | Rotation                  | Linear velocity         | Angular velocity  | Limits / 12 |
| ---------------------------------------------------------------- | -------------------------- | ------------------------- | ----------------------- | ----------------- | ----------: |
| Planar capsule / concave quad                                    | (268435456,4294748103)     | yaw 62586953              | (4294967296,-538094915) | 213505992         |           7 |
| Spatial capsule / declared split slabs                           | (-1073348608,0,2147483648) | (0,0,10152,4294967296)    | (4294967296,0,1)        | (1,-19,221847484) |          12 |
| Spatial capsule / unsplit slab, or split with RotationX/Y frozen | (2147483648,0,2147483648)  | (0,0,83187535,4294161607) | (4294967296,0,0)        | (0,0,221847484)   |           0 |

Diagnostic-only instrumentation, removed before final builds, records 22 planar
and 24 declared-spatial search-limit returns across two episodes each. **Every
return is depth 12 after 13 nodes; none exhausts the 64-node budget.** The
spatial sample has zero materialized minimum-Z support sign, tiny transverse
angular components and an 83,613-raw interval motion bound. That is not proof of
penetration. These angular components defeat the exact invariant-plane proof;
nonclosing rows below the response deadband cannot justify inventing an impulse.

Final 15-second sampled-thread-time profiles filter to the warmed loop. The
joined cone spends 38.9% inclusively in Boolean admission, 23.7% in complete
contact construction and 28.1% in shape preparation; these overlapping shares
are not exclusive CPU percentages. The planar episode spends about 99% in
required complete mixed mesh/slab contacts, including 93.9% in finite root
comparison. Two disposable upstream clipping/sign experiments gave no convincing
improvement and were removed. A whole-yaw orbit-envelope certificate was
rejected before implementation: the pressed planar trajectory moves its center
below that envelope's separation plane, so it would not cover the maintained
workload. No original-search fallback or higher search budget is introduced.

This closes the repeated-work investigation with substantial measured gains and
an explicit no-further-change decision for the residual capacity limit. A useful
next improvement requires a stronger directional/time-dependent support proof or
constraint-aware continuation; it is not a small bookkeeping optimization.
Reopen when such a certificate covers the maintained pressed trajectory, a
representative host budget rejects the remaining cost, or a concrete case
violates blocking, momentum, witness, determinism or limit-reporting contracts.
Existing exact-contact frequency signals remain applicable; no duplicate issue
or signal is created for the documented frontier behavior.

#### Validation and evidence

Full source-stack runs pass **8,287 Release tests** and **8,216 ReleaseLean
tests**, with no failures or skips. Each final configuration has one fresh raw
coverage capture; no historical/failed-run coverage is merged into acceptance.
Every reachable sequence point, branch and method is covered:

| Configuration | Covered / total sequence points | Covered / total branches | Covered / total methods |
| ------------- | ------------------------------: | -----------------------: | ----------------------: |
| Release       |                 53,473 / 53,473 |          21,706 / 21,706 |           5,601 / 5,601 |
| ReleaseLean   |                 53,471 / 53,471 |          21,706 / 21,706 |           5,600 / 5,600 |

Both library target frameworks (`netstandard2.1` and `net10.0`) build in Release
and ReleaseLean with zero errors. Full test/coverage commands, run once per
configuration in the isolated graph:

```powershell
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj -c Release -p:UseLocalLsfStack=true -p:DisableTransitiveProjectReferences=true -m:1 -p:BuildInParallel=false -p:UseSharedCompilation=false -nr:false --collect:"XPlat Code Coverage" --settings tests/Gravitas.Tests/coverlet.runsettings --results-directory artifacts/coverage-release-final
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj -c ReleaseLean -p:UseLocalLsfStack=true -p:DisableTransitiveProjectReferences=true -m:1 -p:BuildInParallel=false -p:UseSharedCompilation=false -nr:false --collect:"XPlat Code Coverage" --settings tests/Gravitas.Tests/coverlet.runsettings --results-directory artifacts/coverage-lean-final
```

New library-behavior tests cover exact Boolean/full finite admission against
independent hit/miss oracles (18 theory rows, 108 placements, including common
rigid rotations, affine half-raw charts, extreme translations, hole/disconnected
patches, tangencies and a one-raw positive cone), both moving mesh/cone
directions, dynamic stopping at the retained witness, open convex admission,
closed convex containment and declared-surface exclusions. Existing moving-pair,
replay, allocation and static-continuity regressions remain in the full runs. No
tests of benchmark/test infrastructure are added. Independent review found no
remaining runtime/test defect; the limit above is retained deliberately.

Ignored supplemental evidence is under
`artifacts/grv-benchmark-027/continuation`: `measured-payload-hashes.json`,
`measured-driver`, `bdn-*-confirmation`, `bdn-moving-*`, `q-*-controls.log`,
`l-*-moving2d.log`, `v-*-facets.log`, `c-*-capacity.log`,
`*-final-snapshots.log`, `capacity-trace.txt`, final Speedscope traces,
`multitarget-*.log`, and final raw coverage captures. The revisions, inputs,
procedure, outcomes and reopening criteria in this tracked record stand
independently of those disposable files. Changes were left unstaged/uncommitted
for review; no package or cross-platform release is claimed.

#### Historical evidence through the first refinement

The following checkpoint predates the continuation above. Its timings, status
and suggested next isolation describe that checkpoint, not the closure state.

**Discovered:** 2026-10-12 during GRV-Issue-101 validation. **Status at this
checkpoint:** Open; single-source gathering optimized in both dimensions. Exact
cone-contact cost and general rotational-refinement capacity remain separate
unresolved costs. **Owner:** Gravitas bounded rotational interval search and
exact finite-contact admission; FixedMathSharp only where profiles identify
reusable geometry work.

`NonconvexRotationalCcdBenchmarks` runs complete `Simulate`/`LateSimulate` steps
with an authored 90-degree rotation at frame rate 1, disabled forces and
frictionless geometry. These are deliberately demanding fixtures, not a
representative game-capacity measurement. The original implementation freezes
inside empty aggregate bounds; its faster zero-angle answer is not an acceptable
optimization baseline.

The first corrected mixed triangle/prism fixture cost **125.916 ms** per step.
Exact Boolean mesh/slab admission, one retained-witness manifold build, and
omitting impulse rows for rotating kinematic/immovable pairs reduce it to
**12.31 ms**. Reusing candidates across exact-zero-displacement conservative
restarts then lowers it to **7.007 ms**, with a **0.2803 ms** half-width 99.9%
confidence interval over two launches: approximately **94% below the first
correct implementation**. Dynamic stopping, movable-target response and
translational fallback still build the complete matched manifold. FixedMathSharp
also removes duplicate point seeds from the existing planar contact reducer;
required-manifold smoke measurements improved from 29.59 to 25.57 ms before the
immovable-pair refinement. That intermediate comparison is an isolated timing
smoke, not the final BDN job.

Final Release results on Windows x64, i7-9700K, .NET 8.0.29 / SDK 10.0.302:

| Complete host-step fixture                                      |      Mean | Allocated |
| --------------------------------------------------------------- | --------: | --------: |
| Joined floor/wall mesh, cone first impact                       |  7.153 ms |    0 B/op |
| Floor-only cone first-impact control                            |  5.707 ms |    0 B/op |
| Empty finite triangle corner, rotating 3D cuboid                | 212.18 us |    0 B/op |
| Compound-wall first impact, pure 2D                             | 726.29 us |    0 B/op |
| Empty compound bounds, pure 2D                                  |  32.37 us |    0 B/op |
| Finite triangle first impact, moving 3D mesh against a 2D prism |  7.007 ms |    0 B/op |
| Empty finite triangle corner, rotating 2D prism against a mesh  |  190.0 us |    0 B/op |

Both mixed rows use longer two-launch confirmations; other rows use three
warmups and five measurements. The empty-corner confirmation uses 1,024
invocations per iteration after the short job reported insufficient iteration
time; its 9.07 us confidence half-width contains the prior 188.09 us result.
Matched smoke controls at 1/32 ordinary unresolved candidates retain their poses
and 0 B/op without a material 32-candidate slowdown. Those control methods'
out-of-process BDN build defect is recorded separately as GRV-Issue-111; do not
present reflection timings as BDN results.

```powershell
$env:UseLocalLsfStack = 'true'
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -p:UseLocalLsfStack=true
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll --filter '*NonconvexRotationalCcdBenchmarks*' --job short --warmupCount 3 --iterationCount 5 --launchCount 1 --iterationTime 250 --affinity 3 --exporters json --artifacts artifacts/grv-issue-101/bench-final
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll --filter '*NonconvexRotationalCcdBenchmarks.MixedFiniteTriangleFirstImpact3D*' --job short --warmupCount 5 --iterationCount 7 --launchCount 2 --invocationCount 32 --unrollFactor 1 --affinity 3 --exporters json --artifacts artifacts/grv-issue-101/bench-final-mixed-confirmation
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll --filter '*NonconvexRotationalCcdBenchmarks.MixedEmptyFiniteTriangleCorner2D*' --job short --warmupCount 5 --iterationCount 7 --launchCount 2 --invocationCount 1024 --unrollFactor 1 --affinity 3 --exporters json --artifacts artifacts/grv-issue-101/bench-final-empty-confirmation
```

Ignored `artifacts/grv-issue-101/bench-confirmation`, `bench-final`,
`bench-mixed-confirmation`, `bench-final-mixed-confirmation`,
`bench-final-empty-confirmation`, `timing-complete.log`, `timing-reuse.log` and
paired baseline captures retain the evidence. The initial mixed profile
attributes 87% to repeated full manifold construction and less than 1% to the
new separation certificate; the retained refinement removes that repetition. The
next mixed profile puts 30% in candidate gathering / typed GridForge partition
lookup. That capture includes 19% setup samples, so those percentages guide
isolation rather than establish a steady-step cost share.
Exact-zero-displacement conservative restarts reuse their existing body-owned
candidate lists. Translation and response still regather, since a rounded
shortened chord does not guarantee identical query bounds. The cone profile
attributes 62% to complete mesh/cone contact construction, including 53% in
cone-plane ray events; its exact curved-contact frequency also belongs to
GRV-Benchmark-024.

**2026-10-31 refinement:** Fresh steady-loop profiles of the maintained
full-step fixtures exclude setup. In the mixed mesh/prism case, same-dimensional
gathering occupies 26.4% of sampled loop time, including 18.2% below dense voxel
range collection, although the source is the only registered 3D collider. Both
body owners now clear retained hits/IDs and skip this gather only when their
live dimensional registry contains exactly their own collider root. Mixed
gathering still runs; registration/removal needs no cache invalidation or new
counters. The post-change trace has no samples below the skipped 3D query; exact
Boolean mixed admission now accounts for 42.7% and mixed gathering for 19.4% of
inclusive sampled loop time. These shares overlap and are not exclusive CPU
percentages.

Frozen source baseline: Gravitas `68ca89f`, FixedMathSharp `0e7a8e5`,
SwiftCollections `d605ec1`, GridForge `76e2bd4`, Chronicler `0a5f962`. Release
source-stack builds use SDK 10.0.302 / .NET 10.0.10 on Windows x64, i7-9700K,
blocking GC and `DOTNET_PROCESSOR_COUNT=2`. Only `Gravitas.dll` differs among
the 35 assemblies in the frozen A/B payloads. A disposable driver calls the
existing benchmark methods, with setup outside measurement, at least one second
of warmup per row, 200 ms calibration and 24 approximately 30 ms measurement
windows per process. These are direct fixture timings, not BDN results or
gameplay frame percentiles. Four independent processes per payload run in ABBA
then BAAB order:

| Complete host-step fixture          | Baseline | Candidate |     Change |
| ----------------------------------- | -------: | --------: | ---------: |
| Joined cone floor/wall              | 6.340 ms |  6.455 ms |      +1.8% |
| Floor-only cone                     | 5.131 ms |  5.065 ms |      -1.3% |
| Empty 3D triangle corner            | 172.0 us |  169.5 us |      -1.4% |
| 2D compound first impact            | 628.6 us |  624.7 us |      -0.6% |
| Empty 2D compound bounds            | 24.33 us |  23.73 us |      -2.5% |
| Moving 3D mesh / 2D prism           | 5.307 ms |  4.170 ms | **-21.4%** |
| Moving 2D prism / empty mesh corner | 150.5 us |  137.9 us |  **-8.4%** |

All measured windows allocate 0 B. Sixteen returned values per fixture and each
final raw pose/velocity/TOI-count/limit snapshot match exactly. Twelve existing
interval/moving-target controls cover 1/32 candidates or pairs in pure 2D, pure
3D and both mixed directions. Across the same ABBA/BAAB process orders, their
mean changes span -3.8% to +1.2%; every raw returned value and body-state
snapshot matches, and every measured window allocates 0 B. These interval/moving
fixture methods run `LateSimulate` only; they are controls, not the full-step
rows above. The joined-cone candidate process means span 6.328-6.739 ms versus
6.295-6.376 ms baseline; the reversed-order pair is 6.377 ms versus 6.336 ms
(+0.6%). Its aggregate +1.8% is retained, not described as a cone improvement or
proof of zero overhead.

Smaller-angle probes reuse the same cone and mixed geometry, resets and complete
host step, changing only the authored angle. Candidate 1/5-degree joined-cone
steps cost 117/332 us, floor-only 108/314 us, mixed mesh/prism 1.123/1.166 ms,
and mixed empty-corner 147/161 us. All reach the complete authored angle with
zero TOI iterations and no limit, exactly matching baseline raw states and
allocating 0 B. They are isolated process timings, not evidence that the
separate small-angle capsule/frontier reproductions below are solved.

The joined-cone profile puts 84.9% below manifold construction and 64.3% below
`MeshConeFaceRegions.Build`. Closing admission uses exact velocities at matched
finite anchors. A Boolean overlap cannot replace that decision; deferring the
manifold requires an exact-pose contact design covering joined surfaces and
nonclosing witnesses. No search budget, tolerance, manifold or geometry change
is retained in this refinement.

Validation uses `UseLocalLsfStack=true`: Release passes 8,262 tests and
ReleaseLean passes 8,191. Raw covered/total counts are 53,456/53,456 lines,
21,678/21,678 branches and 5,599/5,599 methods in Release; Lean retains
53,454/53,454, 21,678/21,678 and 5,598/5,598 respectively. Four new
source-behavior theory cases cover static/moving target registration, removal,
re-registration, a complete rotating step after removal, a lone other target and
an empty registry. Independent review finds no remaining runtime/test issue. No
maintained benchmark/test-infrastructure tests or new runtime helpers are added.

Ignored `artifacts/grv-benchmark-027` contains `heads.json`, frozen source
archives/payloads, `payload-hashes.json`, `loaded-bindings.json`, driver source,
raw timing/snapshot logs and 15-second sampled-thread-time traces. Reproduce the
workloads through `NonconvexRotationalCcdBenchmarks`,
`RotationalCcdIntervalBenchmarks` (1/32 candidates) and
`RotationalMovingPairCcdBenchmarks` (1/32 pairs), using the build command above
with current `net10.0` output. Profile warmed method loops with
`dotnet-trace collect --profile dotnet-sampled-thread-time --format Speedscope`;
filter stacks to the measured loop before attributing inclusive sample shares.
The retained inputs, revisions, timing procedure and outputs here are the
record; ignored captures are supplemental.

**Next isolation at this checkpoint:** Profile exact finite-prism admission and
increasing facet counts separately from required cone response contacts and
outer conservative refinements. Preserve the capsule/frontier reproductions
below before changing motion admission or assigning a host step budget. Preserve
whole-shape separation, earliest frontiers, closing admission, matched anchors,
authored ties and 0 B/op; never replace an unresolved interval with a
sample-only miss or weaken the manifold to one support pair.

**Small-angle isolation, 2026-10-28:** The static-continuity CCD regressions
also reproduce limited progress on the unchanged `9175653` payload, without any
continuity declaration. Use one bodyless concave quad at Z=0, X in `[-8,8]`, Y
in `[-2,2]` (vertices bottom-left, top-left, bottom-right, top-right; triangles
`0,1,2,2,1,3`). In Mixed mode at 16 Hz, place a mass-one planar capsule of
radius `1/2`, total height 2 and mixed half-thickness `1/2` at `(-1/4,1)` with
zero yaw. Disable gravity, air drag and damping, use manual grounding, disable
sleep, enable continuous mode and use frictionless materials. Apply X impulse 1
and angular impulse `1/64` once, then planar Y impulse `-1/16` before each of 12
complete host steps.

The committed control and the coordinated implementation before T0 contact
sampling both ended at `(0.125, 0.999922429677099)`, yaw `0.01788772107101977`,
with velocity `(1, -0.00038653193041682243)`. Uninterrupted free tangential
integration would reach X=0.5. The trace retains real floor witnesses but
repeatedly reaches the fixed interval-depth frontier; the documented
conservative terminal clamp can consume the remaining frame while retaining
momentum. This is an existing refinement/capacity limit, not evidence of a new
missed collision or a new issue ID. The maintained continuity regression
supplies the workload; ignored
`artifacts/static-surface/phase13/rotation-unsplit-committed-control.log` and
`rotation-frontier.txt` supplement these inputs. Test-run elapsed time is not a
benchmark: isolate warm frame time, allocation and limit frequency before making
a cost claim or changing the search budget.

The same isolation also covers a **3D capsule against embedded 2D solids**.
Declare two bodyless AABBs with X ranges `[-8,0]` and `[0,8]`, planar Z range
`[-2,0]`, and mixed Y half-thickness 2. In Mixed mode at 16 Hz, initialize a
mass-one 3D capsule with radius `1/2`, total height 2, identity rotation and
position `(-1/4,0,1/2)`. Freeze **PositionY only**; leave every rotation free.
Disable gravity, air density, damping and sleep, use manual grounding and
frictionless materials, and enable continuous mode. Apply X impulse 1 and Z
angular impulse `1/64` once, then Z impulse `-1/16` before each of 12 complete
host steps. The unsplit control replaces the declaration with one AABB spanning
X `[-8,8]`, with the same Z range and finite Y band.

All values below are exact Q32.32 raw integers:

| Payload and geometry                |     Final X |    Final Z | Final angular velocity (X,Y,Z) | Conservative limits |
| ----------------------------------- | ----------: | ---------: | ------------------------------ | ------------------- |
| Committed `9175653`, declared split | -1073741824 | 2147483648 | (53,0,221847484)               | All 12 frames       |
| Committed `9175653`, unsplit        |  2147483648 | 2147459072 | (0,0,221847484)                | None                |
| Phase 13, declared split            | -1073348608 | 2147483648 | (1,-19,221847484)              | All 12 frames       |

The committed split control never advances from its initial X or identity
orientation. Phase 13 advances X to raw `-1073709056` in frame 0, then reaches
small conservative frontiers; its final quaternion is `(0,0,10152,4294967296)`,
final velocity is `(4294967296,0,1)`, and its last frame consumes two TOI
iterations. It retains Z angular velocity instead of inventing a stopping
impulse from a geometric witness with no closing row. Small transverse angular
velocities can still change an intermediate quaternion by one raw unit while the
contact speed remains below the existing response deadband, defeating the exact
invariant-plane certificate and limiting progress.

This **pre-existing general rotational-refinement capacity limit remains
unsolved**; it is not a newly introduced deferral. The maintained
`SpatialCapsuleWithFreeTransverseRotation_ShouldRetainMomentumAtConservativeFrontiers`
regression preserves the linear and angular velocity, floor blocking and limit
checks. Separate parallel-rotation and raised-wall regressions freeze RotationX
and RotationY and require actual seam crossing and a later closing wall
response. The inputs and raw states above are the retained reproduction; ignored
`artifacts/static-surface/phase13/rotational-control-baseline-p.txt` and
`free-capsule-after-t0.txt` contain the supplemental traces.

### GRV-Benchmark-032 — First Isolated 2D Pair-Owner Deactivation Allocation

**Discovered:** 2026-10-30 during the ordinary coplanar-sweep follow-up.
**Status:** Closed 2026-10-30 with the SwiftCollections source correction.
**Owner:** SwiftCollections dictionary/set generic nullability initialization;
Gravitas pair teardown is the unchanged consumer. Also recorded as SC-Issue-005
in SwiftCollections.

**Original signal:**
`Collider2DStateParityTests.DeactivatePairOwner_AfterWarmup_ShouldNotAllocate`
measures **24 bytes**, against a zero-byte assertion at
`tests/Gravitas.Tests/Physics2D/Collider2DStateParityTests.cs:358`. Reproduce
with 32 radius-0.5 dynamic circles at X=0,2,...,62 in a 128-extent pure-2D test
world, each paired with a static radius-0.5 circle offset by +0.5 X. Run the
existing complete `Step(context)`, force collection through
`AllocationTestHelper`, then measure the first dynamic owner's
`Collider.Deactivate()` on that thread.

On committed Gravitas `44607adddc6d191aa681fc0370ff238b14ed679e` and the
ordinary patch candidate, the isolated Lean guard failed twice with coverage and
once without on each payload. The full candidate Lean run passed 8,186/8,187
tests, failing only this guard. The fresh `10d7e7f` control also fails at 24
bytes. This is a first-use allocation, not a recurring 24-byte cost per teardown
or an instrumentation-only failure.

**Measured RCA:** Allocation checkpoints identify this exact path:
`RemovePairReferences` ->
`ColliderPairState<CollisionPair2D>.TryRemoveCollisionPair` ->
`SwiftDictionary<int, CollisionPair2D>.TryGetValue` -> `FindEntry`. All 24 bytes
occur in the first lookup; the subsequent dictionary removal and other teardown
stages allocate zero. The dictionary's cached flag was
`_KeyCanBeNull = default(TKey) is null`. The deferred generic initializer boxes
the default integer while computing that flag. Construction and `Add` need not
read it, so ordinary scene setup does not necessarily initialize it.

A disposable control calls `RuntimeHelpers.RunClassConstructor` for that exact
dictionary type outside the measured teardown: the initializer alone allocates
24 bytes, then the unchanged deactivation allocates zero. This explains why
prior use of the same constructed dictionary type can hide the allocation in a
complete test run. The forced initialization is diagnostic only and is not part
of the fix.

**Correction:** `SwiftDictionary` and `SwiftHashSet` now classify nullability
from cached type metadata: a reference type or `Nullable<T>` can be null;
another value type cannot. This avoids constructing/boxing a default value. The
matching hash-set initializer reproduced the same 24-byte failure in a direct
cold enum lookup. Existing reference/nullable-key admission, comparers,
iteration order, capacity and per-operation paths are preserved. No instance
state, new public API, production warmup or Gravitas runtime change is added.

Two direct collection regressions fail at 24 bytes before the correction and
pass at zero afterward. Private key/value types keep the relevant constructed
generic types cold. They use `Add` before measurement, then a non-inlined first
lookup/removal call between GC counters, with functional assertions afterward.
Integer/enum, generic struct, nullable generic struct and reference cases all
pass. The existing Gravitas test and its zero-byte expectation remain unchanged.

**Final validation:** Windows x64/i7-9700K, SDK 10.0.302, .NET 10.0.10, two
advertised processors and blocking GC. Serialized source-stack validation uses
`UseLocalLsfStack=true` and global transitive-reference suppression. Frozen
bases: Gravitas `10d7e7fa293a371d0870f665e26a956f3d1c7e98`, SwiftCollections
`47a184b1e6dc5480e65295f2228cf8d77b647422` plus the two flag corrections and
regressions, FixedMathSharp `0e7a8e5760c506db8050714b72fad2421dcd46e6`,
GridForge `76e2bd46380664cfa5403d3b57eca6a82cb6b88a`, Chronicler
`0a5f962ef47d6b0da52bcfadd7fac400cb4397c0`. All temporary source checkpoints and
initializer warmup calls were removed before acceptance. The final Gravitas
source and tests match its committed archive exactly.

| Test project / profile                 | Passed | Lines visited/total | Branches visited/total | Methods visited/total |
| -------------------------------------- | -----: | ------------------: | ---------------------: | --------------------: |
| SwiftCollections core / Release        |  1,406 |       5,321 / 5,321 |          2,370 / 2,370 |         1,144 / 1,144 |
| SwiftCollections core / ReleaseLean    |  1,378 |       5,307 / 5,307 |          2,370 / 2,370 |         1,137 / 1,137 |
| SwiftCollections core / Debug          |  1,404 |       7,094 / 7,094 |          2,378 / 2,378 |         1,144 / 1,144 |
| FixedMathSharp companion / Release     |     43 |           121 / 121 |                62 / 62 |               35 / 35 |
| FixedMathSharp companion / ReleaseLean |     43 |           121 / 121 |                62 / 62 |               35 / 35 |
| Gravitas / Release                     |  8,258 |     53,448 / 53,448 |        21,670 / 21,670 |         5,599 / 5,599 |
| Gravitas / ReleaseLean                 |  8,187 |     53,446 / 53,446 |        21,670 / 21,670 |         5,598 / 5,598 |

The isolated consumer guard now passes at **zero bytes without instrumentation
or added warmup**, and both full consumer configurations pass. Independent
read-only review found no remaining source/test issues. This removes a one-time
first-lookup allocation; it makes no general steady-state throughput claim. Both
Swift libraries build for `netstandard2.1` and `net10.0` in Release and
ReleaseLean. Archived builds inherit duplicate analyzer configuration from the
outer checkout and report generated MemoryPack XML-documentation warnings; there
are no build errors or C# warnings in hand-written source.

Reproduce the original consumer guard from Gravitas with the corrected sibling
source selected:

```powershell
$env:DOTNET_PROCESSOR_COUNT = '2'
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj -c ReleaseLean -p:UseLocalLsfStack=true -p:DisableTransitiveProjectReferences=true --filter 'FullyQualifiedName=Gravitas.Tests.Physics2D.Collider2DStateParityTests.DeactivatePairOwner_AfterWarmup_ShouldNotAllocate' -- RunConfiguration.MaxCpuCount=1 xUnit.MaxParallelThreads=1
```

From SwiftCollections, run the direct regressions with the same source-stack
properties and `--filter 'FullyQualifiedName~SwiftFirstLookupAllocationTests'`.
Full validation omits the filter and collects each project's own coverage scope
and exclusions; Swift profiles add OpenCover alongside Cobertura output only.
Publish SwiftCollections first and validate Gravitas against that released
package before making package/release claims; package references are unchanged
in this source-validation change.

**Closure / reopening:** The measured source cause is removed, the unchanged
consumer guard passes, and all seven collected profiles preserve 100% reachable
coverage. Reopen for a failure with the corrected SwiftCollections assembly
actually loaded, a null-key semantic regression, or a new first-operation
allocation caused by the replacement. Do not weaken the zero-byte assertion.

The entry was moved out of Experimental Signals into the ordered active queue
before work, and into Closed Signals after resolution. Ignored
`artifacts/grv-benchmark-032` retains frozen archives/heads, diagnostic
checkpoints, baseline/final logs and raw coverage. The original matched controls
remain in `artifacts/ordinary-convex-patch/lean-allocation-*`; the tracked
inputs, source revisions, attribution and commands above are sufficient without
them.

### GRV-Benchmark-026 — Initial Generic Convex Mesh Contact Resolution Cost

**Discovered:** 2026-10-07 during GRV-Issue-094 correctness validation.
**Status:** Closed 2026-10-30 with deferred GJK witness construction, a scoped
ordinary coplanar-patch refinement and published residual costs. **Owner:**
Gravitas prepared convex sweeps; complete contact/projection mathematics remains
in FixedMathSharp.

**Historical signal and earlier refinements:**

The warmed public `ConvexMeshInitialContactBenchmarks.SweepCuboidAll` compares
ordinary impacts with initially overlapping, separating cuboids. On this Windows
x64 i7-9700K host (.NET 8.0.29), the corrected geometric initial normal costs
**59.01 us** for a convex quad, **151.38 us** for a concave quad, and **59.59
us** for a compound convex mesh leaf, versus the incorrect baseline's **16.51 /
39.42 / 16.21 us**. Ordinary impacts remain **27.56 / 83.36 / 27.30 us** versus
**26.89 / 82.57 / 27.09 us**. All six cases allocate **0 B/op**.

```powershell
$env:UseLocalLsfStack = 'true'
$env:DOTNET_PROCESSOR_COUNT = '2'
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -p:UseLocalLsfStack=true -m:1 -p:BuildInParallel=false -p:UseSharedCompilation=false -nr:false
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll --filter '*ConvexMeshInitialContactBenchmarks*' --job short --warmupCount 3 --iterationCount 5 --artifacts artifacts/grv-issue-094/final-confirmation
```

Ignored baseline/final captures under `artifacts/grv-issue-094` retain the
measurements. The baseline answered a different, incorrect geometric contract;
restoring travel-facing normals or suppressing real edge contacts is not an
optimization. **Next isolation:** profile complete initial contacts and exact
support/projection comparisons, then measure touching, overlap, containment,
curved sources and dense concave initial-contact workloads. Prefer reusable
exact certificates or reduced duplicated work, with the same geometric normals,
leaf ordering, full-domain witnesses and allocation gates. Measure scene
frequency before assigning a host capacity budget.

**2026-10-11 refinement during GRV-Issue-108:** Initial triangle sweeps now
certify contact against the finite trusted filled surface, including nonconvex
patches, before accepting a covered seam witness. The existing distance/authored
index comparator also rejects candidates that could not win even at distance
zero. This removes repeated GJK and face-admission work after an admitted
initial hit, without caches or a different geometric answer path.

The isolated before/after fixtures use the same benchmark source and exclude the
cuboid and capsule query sources from each other's target layer. Native Release
means on the same host, with three warmups and five measurements, are:

| Public query workload                       |    Before |     After |
| ------------------------------------------- | --------: | --------: |
| Concave quad, initially overlapping cuboid  | 128.79 us |  75.53 us |
| Concave quad, initially overlapping capsule | 230.27 us | 129.49 us |
| Capsule, filled quad at subdivision 2       |  1.551 ms |  0.701 ms |
| Capsule, notched patch at subdivision 2     |  1.184 ms |  0.693 ms |
| Capsule, filled quad at subdivision 8       | 10.590 ms |  4.953 ms |
| Capsule, notched patch at subdivision 8     |  6.387 ms |  4.973 ms |

All 16 ordinary/initial, convex/concave/compound and dense-patch rows report
`0 B/op`. Ordinary concave cuboid/capsule controls are 74.29/583.60 us before
and 76.02/587.35 us after; convex and compound controls remain within about 2%
of their isolated baselines. These short-run means are not gameplay-frame
latency or capacity guarantees. Dense initial capsule sweeps still approach 5
ms, so this existing signal remains active. Profile candidate ordering, GJK and
finite-contact work separately before another optimization; keep nonconvex
holes, exposed rims, initial closing admission and authored tie ordering intact.

Reproduce both maintained benchmark classes with `UseLocalLsfStack=true`:

```powershell
$env:UseLocalLsfStack = 'true'
$env:DOTNET_PROCESSOR_COUNT = '2'
dotnet run --project tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -p:UseLocalLsfStack=true -p:BuildInParallel=false -p:UseSharedCompilation=false -- --filter '*ConvexMeshInitialContactBenchmarks*' '*CoplanarPatchInitialSweepBenchmarks*' --warmupCount 3 --iterationCount 5 --artifacts artifacts/grv-issue-108/final-confirmation
```

Ignored `artifacts/grv-issue-108/isolated-baseline` and `final-confirmation`
retain the matched measurements. The preliminary unisolated source capture and
the pre-pruning correctness capture are not the retained comparison.

**2026-10-29 small mixed CCD control:** A millisecond-scale capacity boundary
also appears without dense subdivision. The existing
`JoinedStaticSurfaceMixedSimulationBenchmarks`, `Shape=Circle`,
`Layout=Continuous`, with its body switched to
`ContinuousCollisionMode.Continuous` after setup, costs **6.607/6.896 ms per
full frame** on committed Gravitas `9175653` and **6.816/6.772 ms** on the Phase
13 candidate in normal/reversed paired runs. Both use the same driver, preserve
physical witnesses and allocate **0 B/frame**; the combined difference is
+0.63%, with opposite order effects. This records an existing cost, not a Phase
13 regression or a speedup claim.

The source is a radius-0.5 circle slab, half-thickness 0.5, centered at host
Y=0.25, sliding against one bodyless two-triangle wall at Z=0, X=[-8,8],
Y=[-2,2]. Use the fixture's 16-Hz Mixed settings, frozen rotation, manual
grounding, disabled sleep and alternating X speed +/-1 with inward impulse
-1/16; advance 64 complete host frames per invocation. Windows x64, i7-9700K,
.NET 10.0.10 Release and `UseLocalLsfStack=true`; each order uses 24 alternating
paired windows after warmup. Profile the complete mixed CCD call chain before
attributing this to initial-contact math or indexing. Ignored
`artifacts/static-surface/phase13/ccd-controls-ba-{forward,reverse}.json`
retains loaded module hashes and raw windows.

**2026-10-30 refinement and closeout:** The dense initial-capsule profile places
approximately 47% of sampled thread time under GJK distance evaluation and 31%
under its endpoint fallback (inclusive, overlapping attribution). These are
localization samples, not a decomposition of exact elapsed query cost. GJK
constructed both physical witnesses after every simplex reduction, although
convergence used only the weighted Minkowski point.

`ClosestSimplexResult` now retains that point, the selected support count and
weights. It reconstructs the two witness frames only when `ComputeDistance`
returns a distance or intersection. Tetrahedron containment retains the previous
three support points and their weights; no preceding-pose or overwritten simplex
witness is substituted. A one-support result uses the exact identity of a
positive one-element weighted average, reconstructing the same anchor fields as
the original path rather than retaining a different exact support term. Unused
retained weight slots are explicitly initialized.

No support selection, GJK convergence tolerance, advancement/bracketing rule,
contact normal, source pose, candidate ordering or complete-contact reducer
changes. No cache, new allocator, mathematical shortcut or lower-stack change.

The comparison freezes committed sources into one isolated sibling graph:
Gravitas `e9eb6d27893ab24679bea62c7b566c96994c8c88`, FixedMathSharp
`0e7a8e5760c506db8050714b72fad2421dcd46e6`, SwiftCollections
`47a184b1e6dc5480e65295f2228cf8d77b647422`, GridForge
`76e2bd46380664cfa5403d3b57eca6a82cb6b88a`, Chronicler
`af513b2ad9014cfb96a643de11dfe06bb8e4a132`. Only the three changed GJK partials
are overlaid for the candidate. Concurrent live Chronicler edits are excluded.
Windows x64/i7-9700K, SDK 10.0.302, .NET 10.0.10 Release, two advertised
processors and the source-mode global transitive-reference suppression flag.

Preliminary native BenchmarkDotNet runs execute all 16 rows successfully with
zero warm allocation, but three 100-ms warmups/five measurements show tiering
and launch instability, including conflicting large changes in small rows. Those
runs are retained and excluded from speedup acceptance. The retained comparison
directly invokes the same maintained fixtures and public methods, warms every
row for at least one second, then measures 24 windows of roughly 20 ms each.
Separate launches run baseline/candidate/candidate/baseline, using the same
driver and frozen dependency DLLs. Setup and calibration are excluded. The
task's builds, tests and profiling do not overlap timing; unrelated machine
activity is not controlled. Means below combine both launches per payload:

| Public query (target kind; initial overlap)     | Baseline us | Candidate us |  Change |
| ----------------------------------------------- | ----------: | -----------: | ------: |
| Cuboid, convex (ordinary)                       |       21.56 |        20.07 |  -6.91% |
| Capsule, convex (ordinary)                      |      345.20 |       303.12 | -12.19% |
| Cuboid, convex (initial)                        |       50.45 |        49.68 |  -1.52% |
| Capsule, convex (initial)                       |      207.24 |       204.67 |  -1.24% |
| Cuboid, concave (ordinary)                      |       64.76 |        59.94 |  -7.43% |
| Capsule, concave (ordinary)                     |     1701.05 |      1544.36 |  -9.21% |
| Cuboid, concave (initial)                       |       73.48 |        70.98 |  -3.41% |
| Capsule, concave (initial)                      |      138.45 |       133.14 |  -3.83% |
| Cuboid, compound convex (ordinary)              |       23.23 |        19.97 | -14.03% |
| Capsule, compound convex (ordinary)             |      354.26 |       305.70 | -13.71% |
| Cuboid, compound convex (initial)               |       51.06 |        49.69 |  -2.68% |
| Capsule, compound convex (initial)              |      209.18 |       206.25 |  -1.40% |
| Capsule, filled patch, subdivision 2 (initial)  |      511.39 |       477.88 |  -6.55% |
| Capsule, notched patch, subdivision 2 (initial) |      515.64 |       485.72 |  -5.80% |
| Capsule, filled patch, subdivision 8 (initial)  |      720.13 |       668.13 |  -7.22% |
| Capsule, notched patch, subdivision 8 (initial) |      790.76 |       734.12 |  -7.16% |

Every retained window allocates **0 bytes**. Both execution orders show the same
direction for all 16 rows. Dense initial filled/notched patches improve about
6–7%; ordinary capsule controls improve about 9–14%. Smaller 1–3% point changes
remain close to control/placement uncertainty and are not general throughput
guarantees. The largest retained initial row still costs about 0.73 ms per
public query; these are neither complete game frames nor a host-rate promise.
The older mixed CCD capacity control above was not measured by this refinement
and receives no claimed speedup; representative scene budgeting remains the
experimental GRV-Benchmark-024 gate.

Exact before/after hit snapshots agree in all **84** controls: all 16 original
rows, 64 overlapping rotated-cuboid poses whose target witness is retained from
GJK, and four dense ordinary-mesh query controls. Compare collider ID, raw
numerator/distance, normal, direction, materialized point and authoritative
anchor origin/rotation/local offsets. Existing initial-contact, finite-boundary,
full-domain, same-pose, allocation and replay tests remain the correctness gate;
no maintained tests around benchmark/test machinery are added.

Full frozen-stack validation passes **8,242 Release** and **8,171 ReleaseLean**
tests, including shared replay expectations and allocation guards. Raw OpenCover
counts retain **100% reachable line, branch and method coverage**:

| Profile     | Lines visited/total | Branches visited/total | Methods visited/total |
| ----------- | ------------------: | ---------------------: | --------------------: |
| Release     |     53,385 / 53,385 |        21,626 / 21,626 |         5,594 / 5,594 |
| ReleaseLean |     53,383 / 53,383 |        21,626 / 21,626 |         5,593 / 5,593 |

The changed simplex state covers all 48 sequence points, 12 branch points and
five methods in each profile. Independent read-only review verifies all GJK exit
paths, the previous-simplex invariant, single-support identity and exact
snapshot agreement. Builds have no errors; their only warnings are
`MultipleGlobalAnalyzerKeys` from the archived sources inheriting both their own
and the outer checkout's `.globalconfig`.

The shared GJK change also lowers the convex-source portion of
GRV-Benchmark-029; that signal remains active because its analytic sphere/mixed
paths and remaining dense cost require their own attribution. No common executed
owner was established for the other active signals, so they are not closed here.

Reproduce the maintained query matrix after a source-stack build using:

```powershell
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -p:UseLocalLsfStack=true -p:DisableTransitiveProjectReferences=true -m:1 -p:BuildInParallel=false
dotnet tests/Gravitas.Benchmarks/bin/Release/net10.0/Gravitas.Benchmarks.dll --filter '*ConvexMeshInitialContactBenchmarks*' '*CoplanarPatchInitialSweepBenchmarks*' --warmupCount 6 --iterationCount 6 --iterationTime 500
```

Ignored `artifacts/grv-benchmark-026` retains source archives, payloads, the
localization trace, preliminary BDN results, the disposable exact-output/timing
driver, raw order-balanced windows and validation reports. The inputs, frozen
revisions, workload admission, results and conclusion above remain sufficient
when those captures are removed.

**2026-10-30 ordinary coplanar follow-up:** The ordinary concave capsule row
still costs roughly five times its convex counterpart even though both targets
are the same four-vertex, two-triangle flat quad. The concave path separately
sweeps the finite triangles; its first diagonal-edge candidate performs curved
support work before the second triangle supplies the earlier filled-face hit.
This is call-path attribution, not a measured split of each triangle's time.

The retained refinement borrows the existing exact convex coplanar certificate
only when the **entire mesh** is one certified patch, the BVH has returned every
triangle, and an exact finite center-chord crossing proves an impact
opportunity. It uses the certified boundary corners as support vertices in the
matching `GeometryFrame`/`GeometryVertices` chart, then calls the existing GJK
advancement and endpoint solver once. A patch is a surface target; its
inspection center does not establish solid containment or authorize an arbitrary
corner witness.

Padded initial bounds must be separated and representable. Initial overlap,
coplanar travel, admission callbacks, source meshes, partial BVH candidates,
holes, notches, disconnected patches and center-chord misses keep triangle
selection. Failed or zero-time patch results also fall back: the rounded GJK
working scale can reduce an exact gap just beyond `ContactTolerance` to that
tolerance. No cache, new solver, public API or lower-stack change is added.

The frozen comparison starts at Gravitas
`44607adddc6d191aa681fc0370ff238b14ed679e`, with the same committed lower-stack
revisions and host/runtime settings as the closeout above. The same 16
maintained fixture methods, disposable driver, one-second per-row warmup, 24
approximately 20-ms windows and separate ABBA launches are used. Builds, tests
and profiling remain serialized and do not overlap timing. The measured ordinary
capsule has committed radius **0.5** and core length **2**, centered at **(-3,
0.3, 0.3)**, axis **(-1, 0, 0)**, travelling **(6, 0, 0)** toward the X=0 quad
with Y/Z in [-0.5, 0.5]. The fixture's later authored height-2 assignment has
not been published by its setup's `Simulate` call; this comparison deliberately
retains the existing committed fixture geometry on both sides.

Final matched public-query means, microseconds per operation:

| Workload (target kind; initial overlap)       | Baseline us | Candidate us |  Change |
| --------------------------------------------- | ----------: | -----------: | ------: |
| Cuboid, convex (ordinary)                     |       20.73 |        20.96 |  +1.11% |
| Capsule, convex (ordinary)                    |      314.84 |       310.43 |  -1.40% |
| Cuboid, convex (initial)                      |       50.66 |        50.77 |  +0.22% |
| Capsule, convex (initial)                     |      211.71 |       210.32 |  -0.66% |
| Cuboid, concave (ordinary)                    |       62.21 |        35.26 | -43.31% |
| Capsule, concave (ordinary)                   |     1596.64 |       325.68 | -79.60% |
| Cuboid, concave (initial)                     |       72.51 |        72.62 |  +0.15% |
| Capsule, concave (initial)                    |      138.83 |       137.92 |  -0.65% |
| Cuboid, compound convex leaf (ordinary)       |       20.58 |        21.03 |  +2.20% |
| Capsule, compound convex leaf (ordinary)      |      316.02 |       312.97 |  -0.97% |
| Cuboid, compound convex leaf (initial)        |       50.76 |        51.40 |  +1.26% |
| Capsule, compound convex leaf (initial)       |      210.45 |       210.18 |  -0.13% |
| Initial capsule, filled patch, subdivision 2  |      494.09 |       493.69 |  -0.08% |
| Initial capsule, notched patch, subdivision 2 |      488.22 |       488.21 |  -0.00% |
| Initial capsule, filled patch, subdivision 8  |      689.99 |       689.81 |  -0.03% |
| Initial capsule, notched patch, subdivision 8 |      756.20 |       763.76 |  +1.00% |

All 64 measured row/launch combinations allocate **zero bytes in every window**.
The two ordinary concave gains agree in both launch orders. The preceding ABBA
repeat, before the final rejected-output cleanup, measured 62.66 to 34.84 us for
the cuboid and 1622.99 to 326.34 us for the capsule, with other controls ranging
from -2.02% to +0.83%. Small control costs remain uncertain: the ordinary
compound-cuboid row rises 0.45 us (+2.20%) in the final repeat versus 0.10 us
(+0.48%) in the preceding one. These measurements cannot separate a small
dispatch cost from launch/code-placement variation and do not establish zero
overhead or general throughput. This is an isolated public-query result, not a
scene throughput, frame-latency or fixed-Hz guarantee.

Exactly **78 of 80** raw hit snapshots agree. The ordinary concave cuboid keeps
the same distance and normal but selects another valid face witness. The
ordinary concave capsule changes distance from **6,442,452,056** to
**6,442,450,944** raw units, a **0.00000025891-unit** difference, below the
existing 1/1,048,576 distance tolerance. Its normal is unchanged and both
witnesses lie on the finite target within the source-clearance tolerance. This
refinement preserves geometric query correctness and deterministic
raw-distance/ID reduction; it does **not** promise identical witness fields or
cross-version replay hashes for eligible sweeps. Initial contacts and the other
78 controls retain their exact snapshots.

Focused source-behavior tests cover analytic capsule TOI and physical clearance,
three equivalent triangulations, repeated closest/all-hit ordering for cuboids
and capsules, point-center witnesses, real outer-rim impacts before the center
crosses, coplanar entry, holes, disconnected islands, grazing bodies whose
center misses, scalar-boundary fallback, rounded-tolerance fallback, exhausted
advancement, fractional affine scale/rotation and zero warm allocation. The
affine witness oracle checks the finite plane/domain and moving sphere's radius;
query tolerance does not require an exact tangential center coordinate.

Final frozen-source validation passes **8,258/8,258 Release** tests.
**ReleaseLean passes 8,186/8,187**, with only the pre-existing 24-byte 2D
deactivation guard failure reproduced separately on the committed control; see
[GRV-Benchmark-032](#grv-benchmark-032--first-isolated-2d-pair-owner-deactivation-allocation).
All 16 new sweep cases pass in both profiles. No 2D source, guard or allocation
expectation was changed. The subsequent SwiftCollections correction in #032
restores the complete 8,187/8,187 Lean pass without a Gravitas runtime change.
Raw OpenCover reports retain complete reachable coverage in both profiles,
independently of that assertion failure:

| Profile     | Lines visited/total | Branches visited/total | Methods visited/total |
| ----------- | ------------------: | ---------------------: | --------------------: |
| Release     |     53,448 / 53,448 |        21,670 / 21,670 |         5,599 / 5,599 |
| ReleaseLean |     53,446 / 53,446 |        21,670 / 21,670 |         5,598 / 5,598 |

Library builds pass for both `netstandard2.1` and `net10.0` in Release and
ReleaseLean. The archived graph inherits duplicate analyzer configuration from
the outer checkout; builds report `MultipleGlobalAnalyzerKeys` and, in the
standard netstandard build, CS1591 warnings exclusively in MemoryPack-generated
sources. There are no build errors or new hand-written public APIs. Independent
read-only review reports no remaining findings after the witness, rounding,
finite-rim, false-result cleanup and support-dispatch checks.

Ignored `artifacts/ordinary-convex-patch` retains source archives, the frozen
sibling graph, baseline/candidate payloads, original and final ABBA windows,
snapshots, the disposable geometry inspection and validation reports. The
original inline patch-support prototype showed small control slowdowns; its
corner loop was separated from the common support dispatcher before final
acceptance. These captures are supplemental; the inputs and conclusions above
remain tracked. GRV-Benchmark-029's dense source-mesh and analytic sphere paths
are outside this gate and receive no speedup claim.

**Closure criteria:** Met under the measured-gain and published-cost criterion.
Retain the complete exact contact contract and zero warm allocation. Reopen for
a reproducible regression in these fixtures or a demonstrated workload budget
that warrants another owner-specific optimization; do not recover the earlier
incorrect initial normals or reinterpret query means as frame capacity.

### GRV-Benchmark-025 — Debug Ragdoll Steady-State Allocations

**Discovered:** 2026-10-06, during full-lifecycle replay Phase 1 validation.
**Status:** Closed 2026-10-30 by current-source attribution, restored strict
runtime coverage and a no-runtime-change decision. The historical 4,318,560-byte
Debug observation is not reproduced or retrospectively attributed. **Owner:**
Gravitas contact/partition capacity and warmed workload admission.

The original `ConstraintStressSolve_ShouldNotAllocateAfterWarmup` used six
humanoid links, five joints, a bodyless cuboid floor, disabled sleep, automatic
grounding and twelve solver iterations. It stepped 80 frames, warmed eight more,
stabilized four and measured 128 complete `Simulate`/`LateSimulate` frames. The
October 6 capture reported **4,318,560 bytes in Debug**, with Release/Lean
passing. Those observations remain historical evidence, not proof of a current
Debug-only defect.

Commit `1a6d864` replaced that test with the bounded
`ContactAndJointIsland_ShouldNotAllocateAfterWarmup` floor variants during the
bodyless-static response repair, and switched the shared humanoid fixture to
manual grounding. Its two floor variants and the motorized guard pass unchanged
in the current Debug and Release stress suites. Running only the old method-name
filter before restoring the guard would not execute its former workload.

**Current-source isolation, 2026-10-30:** Windows x64, i7-9700K, SDK 10.0.302,
.NET 10.0.10 and `UseLocalLsfStack=true`. Maintained test runs advertise two
processors; disposable probes use the host processor count. Baselines: Gravitas
`7cce2b01bb328eafd121da6908382b780a415cd1`, FixedMathSharp
`0e7a8e5760c506db8050714b72fad2421dcd46e6`, SwiftCollections
`47a184b1e6dc5480e65295f2228cf8d77b647422`, GridForge
`76e2bd46380664cfa5403d3b57eca6a82cb6b88a` and Chronicler
`af513b2ad9014cfb96a643de11dfe06bb8e4a132`. These IDs identify checkout HEADs.
Concurrent uncommitted Chronicler work was observed at final inspection; the
executions are not represented as a clean commit-only stack or validation of
those later edits. The allocation probes use copied payloads, with these SHA-256
identities for that mutable dependency:

| Probe configuration | Assembly                 | SHA-256                                                            |
| ------------------- | ------------------------ | ------------------------------------------------------------------ |
| Debug               | `Chronicler`             | `e298a717315cb7810dc94e41ecec5c3adf165afb2e63a16a714758ae8fc592ec` |
| Debug               | `Chronicler.Diagnostics` | `d0b1211078022c537c9f558490e5087b08757e339832c55bbc92d583bd4295ce` |
| Release             | `Chronicler`             | `27380b52e03cdfff750b59cf2eeffdbc8554ecabfd5636ae641051c400049f23` |
| Release             | `Chronicler.Diagnostics` | `893b08a6ee64379265c2733dde1ec69e3137ba5682c9a601282724caf0525ed8` |

A disposable probe invokes the existing humanoid fixture, restores automatic
grounding without an immediate probe, and measures the original frame window.
Debug and Release both allocate **13,728 bytes**, at the same measured frame
offsets `0`, `1`, `30` and `69`; the other 124 frames allocate zero.

`dotnet-trace` 10.0.750501 collected runtime allocation events with provider
`Microsoft-Windows-DotNETRuntime:0x3200001:5` and explicit measurement-window
markers. TraceEvent 3.1.21 analysis filters to that window and its simulation
thread. Allocation-event sizes sum to the exact thread-counter total in both
configurations; allocations on diagnostic/background threads are excluded. The
matched stacks attribute the bytes to:

| Allocation owner during the original window         |      Bytes |
| --------------------------------------------------- | ---------: |
| New collision pair and contact manifold             |      2,752 |
| Collider pair-holder and integer-set capacity       |      4,584 |
| Partition dynamic/awake membership sets             |        432 |
| `WorldVoxelIndex` list growth during repartitioning |      5,168 |
| Discrete-island node buffer growth                  |        792 |
| **Total in each configuration**                     | **13,728** |

This window still contains new contact topology and capacity growth as the
ragdoll falls and spreads. The trace does not identify recurring joint-math,
Debug assertion or lower-stack arithmetic allocation. It does not explain the
historical 4.3 MB result, and no claim is made that a particular earlier change
fixed that observation.

**Resolution:** Restore the named humanoid guard alongside the bounded floor and
motorized guards. Explicitly enable its original automatic grounding and warm
the falling topology for 16 simulation seconds, covering two eight-second
pair-retention intervals. At the fixture's 32 Hz this is 512 frames, followed by
the unchanged eight warmup and four stabilization frames. The next 128 complete
frames allocate **0 bytes in Debug and Release** in the isolated probes. The
maintained guard retains the exact zero-byte assertion and also requires solved
joint rows, bounded anchors/positions and links supported above the floor. No
production code, allocation threshold or measurement helper changes. The
maintained stress suite passes all **9 cases in each of Debug, Release and
ReleaseLean**, including the restored humanoid guard. All three builds report
zero warnings and errors. Full Release validation passes **8,242 tests**. The
first full Lean attempt ends with native exit `0xC0000005` after 4,087 completed
tests; it is incomplete and is excluded from passing validation. A bounded rerun
of the same built payload with `--no-build --no-restore`,
`--blame-crash --blame-crash-dump-type mini` and a TRX logger passes all **8,171
tests**, with no crash dump produced. The first native failure remains
unexplained; this allocation closeout does not establish its cause or a fix. If
it recurs, retain crash/last-test evidence and investigate it separately.

A manual-grounding control at the same longer warmup still encounters a new
collider-pair dictionary at measured offset 93: **312 bytes in both Debug and
Release**. Its Debug allocation stacks identify pair-dictionary creation; this
is another topology/capacity event, not evidence of a Debug-only solver leak.
The restored guard covers its original automatic-grounding contract. The
existing bounded manual-grounding contact/joint guards remain separate.

Reproduce the maintained stress suite from a sibling-source checkout (repeat
with `Release` and `ReleaseLean`):

```powershell
$env:UseLocalLsfStack = 'true'
$env:DOTNET_PROCESSOR_COUNT = '2'
dotnet build tests/Gravitas.Tests/Gravitas.Tests.csproj -c Debug -p:UseLocalLsfStack=true -p:DisableTransitiveProjectReferences=true -m:1 -p:BuildInParallel=false -p:UseSharedCompilation=false -nr:false
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj -c Debug --no-build --no-restore -p:UseLocalLsfStack=true -p:DisableTransitiveProjectReferences=true --filter 'FullyQualifiedName~Constraint3DStressTests' -- RunConfiguration.MaxCpuCount=1 xUnit.MaxParallelThreads=1
```

Ignored `artifacts/grv-benchmark-025` contains the disposable probe/analyzer,
exact frame counters, marked Debug/Release traces and build/test logs. The
fixture parameters, revision identities, measured window, stack attribution and
conclusion above do not depend on those files surviving. No maintained test or
benchmark-infrastructure verification harness was added.

**Closure criteria:** Met for the current source: retain strict warmed humanoid,
bounded contact/joint and motorized guards, and document the historical limit.
Reopen if the same warmed automatic-grounding workload allocates again, a
matched Debug/Release comparison develops a persistent difference, or a trace
shows recurring runtime allocations after topology and retained capacity have
stabilized. New contacts and larger spatial coverage can still require capacity.

### GRV-Benchmark-018 — Complete Triangle/Cylinder Contact Cost

**Discovered:** 2026-09-27; final aggregate capture 2026-09-28. **Status:**
Closed 2026-10-06 under the verified-gains and published-cost criterion.
**Owners:** FixedMathSharp's triangle/cylinder contact selection and witness
materialization; Gravitas's mesh candidate traversal and cap enrichment.

The
[completed refinement report](2026-10-06-triangle-cylinder-cost-refinement.md)
records the fresh committed baseline, independent profiling, two retained exact
construction optimizations and the reverted root-retention experiment.
Reciprocal charts share their normalized algebra after independent admission;
opposite analytic directions reuse squared support terms in existing candidate
slots. Feature completeness, strict tie order and one-round witnesses remain.

The fresh two-launch 64-pair batches improve **5.551 -> 5.189 ms (6.5%)** for
mesh/cylinder and **3.119 -> 2.867 ms (8.1%)** for concave mesh/cylinder.
Focused cylinder/mixed circle-slab intrusion, oblique and touching means improve
**15.3-19.6%**; the added stationary-root-winner fixture improves **9.4-9.9%**
to about **0.764 / 0.763 ms**. All rows measure **0 B/op**. The original
cylinder cap-face intervals overlap; its small mean shift is not a proved gain.
Positive-core triangle/capsule-slab controls also remain allocation-free, with
five gains and one overlapping rounded-end interval.

Fresh Release/ReleaseLean reachable line, branch and method coverage, focused
Debug/resource/allocation tests, both target frameworks and API documentation
checks are recorded in the completed report. The winning-root retention trial
was reverted: its apparent one-launch improvement did not repeat in matched
controls and did not justify another 6,773 bytes of persistent traversal
scratch.

Ordinary contacts remain tens of microseconds; complete rim-heavy queries remain
several hundred microseconds. Closure makes no complete-step or fixed-Hz
capacity promise.
[GRV-Benchmark-024](../benchmark-signal-hardening-backlog.md#grv-benchmark-024--curved-contact-workload-frequency-and-step-cost)
retains scene-frequency and step-cost evidence before more geometry work.

**Historical correctness-repair evidence (2026-09-27/28):**

The complete contact owner repairs missed intrusions, false rim contacts and
mismatched normal/depth/witness output. Its exact geometry is the correctness
baseline; the previous center-nearest approximation is not an acceptable fast
fallback.

Same-machine BenchmarkDotNet comparison: Windows, Intel i7-9700K, .NET 8.0.29,
SDK 10.0.302, two-core process affinity, one workload at a time, two launches,
five warmups and fifteen measured iterations per launch. Means are **per batch
of 64 collider pairs**, not per contact or simulation frame. All rows measured
zero managed allocation.

| Row                             | Previous incomplete path | Unpruned implementation | Face certificate / pruning |        Final |
| ------------------------------- | -----------------------: | ----------------------: | -------------------------: | -----------: |
| `CheckMeshCylinderPairs`        |                 3.812 ms |               32.664 ms |                   8.348 ms | **6.064 ms** |
| `CheckConcaveMeshCylinderPairs` |                 2.835 ms |               62.688 ms |                   5.469 ms | **3.825 ms** |

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
preflight expected classification and certified depths outside measurement. They
distinguish cap/side faces, the original intrusion, oblique rim contact, rim
touch and a rim gap for cylinder and mixed circle-slab consumers. Cylinder cap
contacts may build a four-point manifold; mixed circle slabs return one contact,
so those columns are not identical workloads.

Final focused means, **microseconds per query**, with zero managed allocation in
all twelve rows:

| Geometry               | Cylinder / triangle | Circle slab / triangle |
| ---------------------- | ------------------: | ---------------------: |
| Cap face               |            57.05 us |               21.63 us |
| Side face              |            31.94 us |               31.83 us |
| Original cap intrusion |         1,055.12 us |            1,053.47 us |
| Oblique rim            |         1,179.01 us |            1,160.16 us |
| Rim touch              |           846.88 us |              833.32 us |
| Rim gap                |            44.24 us |               44.01 us |

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

**Isolation completed:** The October refinement isolates chart construction,
analytic support construction and stationary-root materialization. The fresh
report supersedes the historical timing rows above for current cost estimates.
Further work is gated on representative scene evidence under GRV-Benchmark-024.

### GRV-Benchmark-019 — Complete Triangle/Cone Contact Cost

**Discovered:** 2026-09-29. **Status:** Closed 2026-10-06 after four measured
refinement passes. **Owners:** FixedMathSharp's triangle/cone support selection,
rim roots and paired witnesses; Gravitas's mesh candidate traversal.

Correctness, resource proofs and shared-owner control evidence are retained in
the [completed contact plan](2026-09-28-complete-triangle-cone-contact-plan.md).

The complete contact query replaces missed intersections and inconsistent
normal/depth/anchor output. Its exact geometry is the correctness baseline. The
bounded optimization pass reused the existing face-disk certificate and
principal-axis depth reducer; it did not introduce an approximate fallback, new
arithmetic engine, cache or allocation.

Matched Windows/i7-9700K measurements use .NET 8.0.29, SDK 10.0.302, two
launches, five warmups and fifteen measured iterations per launch. The launcher
is BelowNormal with affinity mask 3 and `DOTNET_PROCESSOR_COUNT=2`; only one
heavy workload runs at a time. Values below are **microseconds per dispatched
query**, not batches or simulation frames. Every row reports zero managed
allocation.

| Geometry                        |     Old incomplete path | Initial complete solver | September optimized mean | September standard deviation |
| ------------------------------- | ----------------------: | ----------------------: | -----------------------: | ---------------------------: |
| Base face                       |                  19.983 |                   152.1 |                    33.89 |                        0.676 |
| Side face                       |                  17.525 |                   567.4 |                    62.57 |                        1.193 |
| Apex face                       |    28.838 — wrong depth |                   169.3 |                    55.67 |                        1.331 |
| Side intrusion                  | 15.407 — missed contact |                   301.5 |                   283.08 |                        6.321 |
| Oblique rim                     | 17.403 — missed contact |                   905.3 |                   850.98 |                       31.440 |
| Interior stationary rim         |            Not captured |            Not captured |                 1,407.39 |                       32.889 |
| Rim touch                       |   17.932 — missed touch |                   703.1 |                   692.58 |                       13.452 |
| Rim gap                         |                  17.652 |                   122.0 |                   120.78 |                        2.479 |
| Unrepresentable relative center |  6.069 — missed contact |                 1,708.7 |                 1,652.05 |                       31.686 |

September base/side/apex costs are about 78% / 89% / 67% lower than the initial
complete implementation. The old base/side/gap rows had the correct answers for
these fixtures and remain cheaper: the final costs are approximately 1.70x /
3.57x / 6.84x theirs. Five other old rows returned the wrong result and are
repair-cost comparisons, not equivalent-correctness speed regressions. The
interior-rim row was added with its independent exact-depth oracle after the old
baseline. General curved/full-domain work near 0.7–1.7 ms per query is still a
meaningful cost; ordinary paths also merit further profiling.

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

The [2026-10-06 refinement](2026-10-06-triangle-cone-cost-refinement.md)
refreshes the baseline after shared-owner changes and removes duplicate
horizontal-face axes, rational witness search, wholly inadmissible charts and
nonwinning root mapping. It also canonicalizes zero-radius radical signs before
accumulation. Matched two-launch base/apex/side means fall from
28.95/49.29/55.37 us to 22.71/23.60/30.00 us; unrepresentable-center cost falls
from 1,444.04 to 463.18 us. Oblique/interior rim fall from 826.45/1,229.16 to
651.08/1,046.86 us. All nine rows retain zero managed allocation. The linked
report records the complete matrix, uncertainty, private child IL identities,
exact-bound proofs, shared controls, resource tests and coverage gates.

The [generator refinement](2026-10-06-triangle-cone-generator-refinement.md)
retains one local winner, transforms only final analytic normals, shares
generator branch coefficients/denominators and directly squares quadratic gaps
with canonical coefficient signs. A fresh two-launch comparison lowers gap
rejection from 94.95 to 88.70 us/query, apex face from 24.91 to 22.81 us and
side face from 31.10 to 28.32 us. All rows remain 0 B/op. Fresh
cylinder/circle-slab controls differ by less than 2%; stationary interior-rim
cost remains about 1.09 ms/query. The linked report records the full matrix and
verification.

The
[shared root-normal refinement](2026-10-06-triangle-cone-normal-refinement.md)
uses conservative compact Horner bounds to narrow the existing exact integer
search while preserving exact half-raw rounding. A fresh two-launch interior-rim
comparison falls from 1,052.85 to 918.32 us/query (12.8%); all nine cone rows
remain 0 B/op. Two witness-search experiments failed repeatable timing and were
removed. Paired cylinder/slab controls retain small visible differences without
establishing a material regression. The linked report records the full matrices,
proofs, profiles and validation.

The [final root-sign refinement](2026-10-06-triangle-cone-final-refinement.md)
strengthens the existing retained-cell certificate and defers unused defining
height scans. A fresh matched two-launch interior-rim mean falls from 966.98 to
927.41 us/query (4.1%), with separated 99.9% confidence intervals; other cone
row intervals overlap baseline. All cone and shared-control rows remain 0 B/op.
An additional witness-arithmetic experiment adds no established benefit and is
removed. Both owners retain exact 100% reachable line, branch and method
coverage in Release and ReleaseLean, with unchanged exclusions.

**Closure decision:** The isolated complete-query investigation is complete
under the user's judgement-based performance bar. Ordinary face fixtures cost
about 23-29 us; interior/oblique/touch rim remain about 0.93/0.67/0.63 ms and
fan-gap rejection about 89 us on this host. These costs remain material and
published, without assuming feature rarity or a fixed host rate. The final
actual-only profile attributes 50.64% inclusive interior CPU to the root-sign
core and 18.82% to witness-coordinate search; gap rejection spends 82.02% in
analytic traversal, including 59.93% in the generator fan. Inclusive owners
overlap. Preserve complete admission, exact ties, paired witnesses, raw-neighbor
classification, bounded stack use and zero allocations.

Representative cone/triangle and shared triangle/cylinder candidate frequency
and complete-step cost now belong explicitly to
[GRV-Benchmark-024](../benchmark-signal-hardening-backlog.md#grv-benchmark-024--curved-contact-workload-frequency-and-step-cost).
Deeper solver redesign or retained cross-frame state needs measured scene
pressure. The separate
[triangle/cylinder investigation](#grv-benchmark-018--complete-trianglecylinder-contact-cost)
can continue to improve shared owners without automatically reopening this
signal. The cone-volume-query correctness repair remains GRV-Issue-087.

### GRV-Benchmark-020 — Complete Capsule/Circle-Slab Contact Cost

**Discovered:** 2026-09-29. **Status:** Closed 2026-10-05; coordinated
refinements are verified for review. Remaining exact oblique-root cost is
published under the judgment-based closure bar. **Owner:** FixedMathSharp's
shared cylinder/capsule feature selection and exact output materialization;
Gravitas consumes that owner for mixed circle slabs and 3D cylinders.

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

| Geometry               |           Old mixed path | Complete mixed, first run | Complete mixed, confirmation | Equivalent 3D cylinder, confirmation |
| ---------------------- | -----------------------: | ------------------------: | ---------------------------: | -----------------------------------: |
| Cap                    |                    12.27 |                     19.32 |                        19.07 |                                21.51 |
| Side                   |                    16.70 |                     25.66 |                        25.12 |                                26.35 |
| Zero capsule core      |                    16.67 |                     25.78 |                        24.75 |                                26.01 |
| Endpoint rim           | 30.54 — nonminimum depth |                    106.69 |                       108.18 |                               108.30 |
| Separated endpoint rim |    30.62 — false contact |                     25.78 |                        25.54 |                                25.90 |
| Oblique interior rim   | 36.70 — nonminimum depth |                  1,224.40 |                     1,191.51 |                             1,196.17 |

The mixed confirmation's standard deviations are 0.099 / 0.080 / 0.103 / 1.051 /
0.159 / 8.028 microseconds in table order. Equivalent 3D controls show the same
expensive curved-feature behavior. These are complete wrapper costs: 3D also
validates public inputs and updates a manifold, while mixed constructs its
canonical contact. Their difference does not isolate adapter overhead, and there
is no historical 3D baseline for these new fixtures.

**2026-10-05 progress:**
[Exact output refinement](2026-10-05-capsule-circle-cost-refinement.md) reuses
rigorous magnitude-floor bounds and one jointly scaled depth-polynomial
reduction. Fresh committed/retained mixed oblique means improve from 1,186.47 to
**745.01 microseconds**; equivalent 3D improves from 1,124.91 to **724.62
microseconds** (37.2% / 35.6%). All twelve rows remain 0 B/op and both
repositories retain exact 100% standard/Lean line, branch and method coverage.
The report records small, incompletely isolated ordinary-row shifts, targeted
cap repeats, resource proofs and actual timed-child assembly verification;
stale-cache captures are excluded. The signal remained open after that phase.

**2026-10-05 closure:** The same
[refinement report](2026-10-05-capsule-circle-cost-refinement.md) now records
certified Sturm point signs, hoisted invariant normal products and factored
radical bounds, all in existing FixedMathSharp owners. The final mixed / 3D
endpoint means are **36.578 / 38.962 µs**, versus the fresh committed control's
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
shifts with overlapping intervals. Final Release/Lean suites retain exact **100%
reachable line, branch and method coverage**, exclusions unchanged; Debug
stack/allocation checks, both target frameworks and both DocFX/local-link gates
pass. Source reviews found no actionable correctness or simplicity issues. A
measured retained-cell certificate was removed because it established no gain.

**Closure decision:** Proven correctness, reproduced gains across mixed/3D and
shared consumers, zero allocation and bounded resources meet the same bar used
for #021. Remaining root acquisition and exact quartic signs are documented; no
cheaper demonstrated refinement remains in this investigation. The roughly
0.70-ms representative oblique query is a workload-budget consideration, not a
fixed-rate host guarantee or proof of an unavoidable lower bound. Preserve
full-domain classification, nearest-even depth, deterministic ties and canonical
anchors. Further work should start from new measured workload evidence, without
restoring incomplete directions or adding an approximate solver.

**Original fixture capture:** Build
`Gravitas.slnx -c Release -p:UseLocalLsfStack=true`, then:

```powershell
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll capsule-circle-contact --warmupCount 3 --iterationCount 10 --launchCount 1 --exporters json --artifacts artifacts/grv085/after-confirm
```

Baseline source: Gravitas `04805b8` and FixedMathSharp `9833123`. Fixture
geometry and the measured mixed method were unchanged between captures;
corrected-answer setup assertions and the paired 3D control were added after the
legacy capture. Setup checks classification, known quarter-unit depths and
cross-path depth agreement. Reports and raw measurements are under
`artifacts/grv085/before`, `after` and `after-confirm`; retain the individual
rows rather than an average that hides the curved-feature cost. Correctness
evidence is retained in
[GRV-Issue-085](../issue-tracker.md#grv-issue-085---mixed-capsulecircle-slab-contact-bypasses-the-complete-upstream-query).

#### Capsule Rim Cost Relationship (GRV-Benchmark-020 And 021)

Source inspection on 2026-09-30 connects these signals without establishing a
shared CPU hotspot. Keep both fixture families and their measured results
distinct; investigate them together, starting with oblique contacts and then
endpoint/straight-rim output materialization.

The 2026-10-04 coordinated profiles now distinguish their expensive owners:
circle/cylinder ellipse magnitude-to-raw comparison takes about 53-55% inclusive
sampled CPU, while stadium constrained root acquisition, sign refinement and
value mapping remain expensive. The
[refinement evidence](2026-10-04-capsule-slab-cost-refinement.md) retains both
profiles and mixed/3D/triangle controls. This does not establish a single shared
dominant leaf or justify merging the specialized solvers. The 2026-10-05
[certified-sign refinement](2026-10-05-capsule-slab-sign-refinement.md) moves
the stadium analytic-winner bottleneck toward root acquisition, while circle
mixed/3D controls retain their distinct ellipse path. The subsequent
[root-isolation refinement](2026-10-05-capsule-slab-root-isolation-refinement.md)
closes #021 with reproduced gains, bounded resources and published remaining
cost. The subsequent circle/slab
[output refinement](2026-10-05-capsule-circle-cost-refinement.md) closes #020
with both dimensional controls and measured shared-consumer gains. Both signals
are closed; shared algebra does not imply shared root-selection policy.

The fixture names describe different features:

| Fixture                     | Capsule feature         | Slab feature      |
| --------------------------- | ----------------------- | ----------------- |
| `EndpointRim` (020)         | Core endpoint           | Circular cap rim  |
| `StraightRim` (021)         | Core endpoint           | Straight cap edge |
| Oblique interior rim (both) | Interior of tilted core | Curved cap rim    |

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
end-region roots and maps/compares their signed gaps through value roots,
reusing circular-rim algebra also consumed by triangle contacts. Copying the
cylinder's reflection/root-selection shortcut could discard a valid stadium
contact.

Future profiling should identify the affected stages before attributing cost to
a shared leaf or merging solvers. Prefer demonstrated shared arithmetic and
materialization wins or proved feature certificates; retain ordinary mixed/3D
and affected triangle controls. Similar exact algebra does not yet prove the
same dominant function, an unavoidable correctness cost, or an optimization that
will benefit both.

### GRV-Benchmark-021 — Complete Capsule/Stadium-Slab Curved Contact Cost

**Discovered:** 2026-09-30. **Status:** Closed 2026-10-05; Gravitas `989c7f9`
and coordinated FixedMathSharp `bd8f6a2` are committed. Closure uses reproduced
gains, preserved full-domain behavior and published remaining cost; it does not
certify a fixed-rate host budget. **Owner:** FixedMathSharp's complete
capsule/stadium-slab feature selection, shared circular-rim algebra and signed
value-root comparison.

The complete query fixes false contacts and nonminimum penetration. Subsequent
[retained-root/positive-certificate](2026-10-04-capsule-slab-cost-refinement.md),
[certified-sign](2026-10-05-capsule-slab-sign-refinement.md), and
[root-isolation/global-minimum refinements](2026-10-05-capsule-slab-root-isolation-refinement.md)
retain every constrained chart and exact classification/rounding contract. The
[correctness repair history](2026-09-29-complete-capsule-slab-contact-plan.md#refined-matched-performance)
retains the older complete and incorrect/incomplete baselines.

The final phase borrows existing evaluation scratch for certified Sturm signs,
shares batch subdivision counts and reciprocal-chart Sturm chains, and proves
when a negative stationary gap is the complete shape's global minimum before
ending traversal. No speculative cache, second solver or accuracy setting is
added. The matched final confirmation against committed FixedMathSharp `9b1f173`
gives the following microseconds per dispatched query:

| Stadium fixture         | Fresh committed baseline |           Final | Mean reduction |
| ----------------------- | -----------------------: | --------------: | -------------: |
| Oblique analytic winner |           1,407.0 ± 7.42 | 1,052.1 ± 11.54 |          25.2% |
| Original oblique rim    |             716.7 ± 5.16 |    527.3 ± 2.89 |          26.4% |
| Irrational oblique rim  |             897.0 ± 7.68 |    794.1 ± 5.06 |          11.5% |
| Positive oblique rim    |             841.9 ± 8.42 |    734.7 ± 6.39 |          12.7% |

Error is the 99.9% confidence half-width. Both final accepted confirmations have
intervals disjoint from both committed captures in every fixture. All four rows
and all 28 separate ordinary stadium, mixed circle-slab, equivalent 3D cylinder
and triangle controls report **0 B/op** and pass behavior preflight. The final
evidence records resource proofs, independent regression oracles, rejected
experiments, profiles and raw capture locations. Both repositories pass full
Release/ReleaseLean suites with exact **100% reachable line, branch and method
coverage**, focused Debug/resource checks and both DocFX gates.

**Closure boundary:** The residual roughly **0.53–1.05 ms/query** curved cost
remains significant. Root isolation and exact sign/arithmetic work still consume
much of the sampled CPU. Dense curved-contact workloads require their own
complete-step capacity measurements; these microbenchmarks do not establish
MMO/strategy throughput or cross-platform replay. The focused investigation
closes under the user's judgement-based performance bar. GRV-Benchmark-020 is
also closed after its distinct ellipse-depth and shared-output investigation;
related algebra alone does not establish one common dominant hotspot.
Released-package validation remains a release gate after the coordinated
FixedMathSharp release.

**Reproduce:** after a Release local-stack benchmark build, run
`dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll --filter '*CapsuleSlabContactBenchmarks*Oblique*' --launchCount 2 --warmupCount 5 --iterationCount 15 --invocationCount 512 --affinity 3 --exporters json --artifacts artifacts/grv-benchmark-021/reproduce`.
Use `DOTNET_PROCESSOR_COUNT=2`, BelowNormal priority and one heavy workload. Set
`UseLocalLsfStack=true` in MSBuild and the environment for child builds. The
command requests affinity 3; these exports display `Affinity=11`, so no claim of
independently verified effective affinity is made.

### GRV-Benchmark-023 — Circle Workload Partition And Grounding Scaling

**Discovered:** 2026-10-02 during GRV-Benchmark-022 isolation. **Status:**
Closed 2026-10-04 under the predictable-scaling/published-cost criterion and
documented no-further-change decision below. The latest 2D query gathering,
bounds-first rejection, characterization and documentation changes remain
unstaged/uncommitted for user review. Partition phase committed as `f23d836`;
shared radial-math and planar capture refinements committed as FixedMathSharp
`6789c09` and Gravitas `ee613db`. Prepared 2D query and grounding-selection
refinement committed as `6c0c2f0`. The centered-transform and locked lookup
refinements are committed as FixedMathSharp `5447b34` and GridForge `3f34f8b`;
Gravitas's characterization, benchmarks and documentation are committed as
`bf11409`. Streamed 3D grounding was rejected after repeated sparse-ray
regressions; its public characterization tests and benchmark fixture are
retained. Warmed full-step costs are published below, separate from exact
contact-query cost. **Owner:** Gravitas retained partition rental/distribution
and collider refresh, 2D query preparation and ground-hit selection, plus
GridForge voxel partition lookup used by planar grounding.

**Closure criterion:** Predictable scaling and published costs, selected by the
repository owner, replace a fixed-Hz/frame-budget target for this signal.
Collider/pair counts, geometry, cell size, covered memberships, emitted links
and candidate counts are published beside matched timings and allocations.
Retained changes passed repeated controls, deterministic behavior and full
Release/Lean coverage gates, with material tradeoffs documented. Closure does
not mean that every scene fits a particular host frame budget.

An exploratory `circle-contact-simulation` capture with default unit cells
measured 64 diagonal pairs at 13.35 ms/step and 1024 axis pairs at 42.59
ms/step. The 1024 diagonal setup was stopped after more than 160 seconds of
child CPU time and approximately 800 MB working set. These are diagnostic
signals, not a completed matched benchmark: the initial grid also clipped the
first diagonal row after positional correction. Raw partial output is retained
in `artifacts/grv-benchmark-022/workload-before.log`; the live process
observation is transcribed in `unit-cell-process-observation.txt` alongside it.

A separate, padded-grid EventPipe capture of 64 diagonal pairs puts 43.0% of
actual-iteration sampled CPU in `Voxel.TryGetPartition` and 15.4% in partition
distribution (exclusive). Positional correction/repartition contributes 27.4%
inclusive and grounding 15.0% inclusive; these overlap. Contact generation and
lever materialization are small in this workload. Traces and actual-only
analysis are in `artifacts/grv-benchmark-022/workload-profile*`.

Source inspection identifies an additional setup risk: when the inactive pool is
empty, `RetainedPartitionLifecycle.TryRetireEmptyForReuse` scans all retained
partitions before each new rental. A fully occupied initial scene has nothing to
retire, allowing quadratic registration work. Per-partition sparse sets also
grow with the highest global collider ID. The rental owner is shared by 2D, 3D
and mixed services; dimension parity matters for any repair.

**Original isolation step:** Measure registration and warmed
refresh/distribution separately while varying cell size, covered voxels and
collider count. Profile registration to distinguish rental scans from sparse-set
growth. Preserve deterministic reuse, empty-partition reclamation and allocation
guards; do not remove GridForge locking based on this trace. GRV-Benchmark-022
now uses explicit 16-by-1-by-16 cells to isolate sustained contact work; that
fixture choice is not a runtime fix for this signal.

With explicit 16-by-1-by-16 cells, the completed 1024-pair diagonal workload
still measures 35.99 +/- 0.47 ms/step, versus 4.22 +/- 0.09 ms for reset plus
direct queries. A separate actual-iteration profile puts grounding at 29.6%
inclusive, partition lookup at 18.7% exclusive, and partition distribution at
14.8% inclusive. The complete capsule contact owner contributes 7.9% inclusive
and anchor offset materialization 1.6%. These are overlapping sampled paths, not
a stage-time decomposition. Fine cells amplify the problem but do not explain
all large-scene cost. The full-step distributions include bimodality and short
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
the [migration guide](../../MIGRATION.md#partition-membership-ownership).

Pure 2D, 3D and mixed coverage refresh keeps memberships shared by the old/new
voxel sets, removes departures in previous coordinate order and publishes in
tracer order. Full `WorldVoxelIndex` identity includes grid/world lifetimes.
Mobility changes replace the old role; surviving dynamic memberships synchronize
awake state, and freeze/unfreeze transitions refresh awake membership directly.
Bounds, geometry/version publication, planar candidate indexing, deferred
refresh and topology rebuild semantics remain covered. Pure distribution paths
reuse their existing sorted static-style copy helpers. No upstream source
change, new collection owner, math kernel or voxel cache is needed.

**Collection tradeoff:** `partition-membership` compares existing owners with
global IDs starting at 65536. Add/remove plus sorted copy takes
15.06/11.78/19.39 ns for sparse/hash/packed storage at one member; at 64 members
it takes 169.56/277.44/239.50 ns. All are 0 B/op after warmup. Hash storage
addresses the global-ID memory failure and wins the one-member row, but its
denser copy cost is a real tradeoff, not a universal collection speedup.
Integrated scene rows remain the evidence for the affected workload.

**Matched cold registration:** `ColliderCount` is the total number of dynamic
plus static colliders, half as many independent radius-five diagonal pairs.
Context/grid/array creation and cleanup are outside the measured registration.
Error is half of the 99.9% confidence interval; allocations are KB/op.

| Total colliders / XZ cell edge | Before mean +/- error (ms) | After mean +/- error (ms) | Before / after allocation (KB) |
| ------------------------------ | -------------------------- | ------------------------- | ------------------------------ |
| 64 / 1                         | 118.890 +/- 2.1111         | 12.096 +/- 1.7149         | 5728.29 / 4107.09              |
| 64 / 16                        | 2.947 +/- 0.0492           | 2.975 +/- 0.0730          | 256.47 / 237.95                |
| 256 / 1                        | 2753.815 +/- 179.1855      | 41.426 +/- 39.1938        | 46411.04 / 16403.90            |
| 256 / 16                       | 11.940 +/- 0.6168          | 11.812 +/- 0.1554         | 1220.05 / 912.28               |
| 1024 / 16                      | 40.773 +/- 23.1878         | 33.453 +/- 21.3649        | 7982.59 / 3616.41              |

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
Run
`cold-circle-registration --filter '*' --launchCount 1 --warmupCount 3 --iterationCount 8 --affinity 3 --exporters json`
through the compiled benchmark DLL. The historical before capture filters out
1024 unit-cell colliders. Warmed stage selections use
`circle-partition-maintenance --filter '*ColliderCount: 64, CellSize: 1)*' '*ColliderCount: 1024, CellSize: 16)*' --launchCount 2 --warmupCount 5 --iterationCount 15 --iterationTime 250 --affinity 3 --exporters json`.
The integrated control uses
`circle-contact-simulation --filter '*ResetAndFullSimulationStep*Diagonal*'`
with the same warmed job settings. Hardware is an i7-9700K on Windows 11, .NET
SDK 10.0.302 / runtime 8.0.29, BenchmarkDotNet 0.15.8. Both source stacks use
FixedMathSharp `00a38bd`; Gravitas before is `05ddcac`, after is the
partition-hardening change committed as `f23d836`.

**Grounding decision and remaining scope:** Preserve automatic ground probing
when contact normals reject support. The paired above-body targets do not
provide ground, and zero gravity or a rejected response normal does not prove
that the probe can be omitted. Existing public support queries differ in allowed
phases, tiny probes, compound-normal filtering and physical-pair policy;
replacing the automatic sweep with them would change behavior. The remaining
ground work uses the existing exact sweep owner. Do not remove GridForge
synchronization or add another cache based on sampled attribution alone. The
fresh full-step profile and warmed stage results below keep the remaining work
explicit. Closure uses published costs and predictable scaling, without a
fixed-Hz target. Runtime mass mutation discovered in the awake review is
independently tracked as GRV-Issue-089.

**Matched warmed stages:** The translated row includes pose reset plus coverage
refresh; retained distribution deliberately keeps processed pair keys and skips
repeated narrow phase/response. Forced probes check the same above-body scene
without a complete simulation step. Every row reports 0 B/op. These are separate
stage workloads, not additive components of the full-step measurement.

| Stage / total colliders / XZ cell edge | Before mean +/- error (us) | After mean +/- error (us) |
| -------------------------------------- | -------------------------- | ------------------------- |
| Translated refresh / 64 / 1            | 763.6 +/- 15.00            | 665.1 +/- 13.88           |
| Retained distribution / 64 / 1         | 1340.9 +/- 11.89           | 1268.5 +/- 18.01          |
| Forced automatic probes / 64 / 1       | 565.0 +/- 21.49            | 523.1 +/- 2.58            |
| Translated refresh / 1024 / 16         | 2678.6 +/- 35.39           | 2605.6 +/- 11.46          |
| Retained distribution / 1024 / 16      | 401.8 +/- 3.62             | 399.0 +/- 2.45            |
| Forced automatic probes / 1024 / 16    | 1638.7 +/- 70.66           | 1467.9 +/- 7.71           |

Fine-cell translated refresh improves 12.9% and retained distribution 5.4%.
Coarse distribution intervals overlap. Grounding benefits from the partition
storage change; its physical acceptance rules and exact sweep are unchanged. Raw
results are in `artifacts/grv-benchmark-023/maintenance-before`,
`maintenance-verified`, and `distribution-complete` (the final distribution
confirmation after helper deduplication).

**Integrated control:** `PairCount` here means independent dynamic/static pairs,
twice as many total colliders; XZ cells have edge 16.

| Diagonal pairs | Before full step mean +/- error (ms) | After full step mean +/- error (ms) | Allocation |
| -------------- | ------------------------------------ | ----------------------------------- | ---------- |
| 64             | 1.6085 +/- 0.11412                   | 1.548 +/- 0.1146                    | 0 B/op     |
| 1024           | 35.9930 +/- 0.46672                  | 35.573 +/- 0.3858                   | 0 B/op     |

These intervals overlap. Intermediate after captures range from 35.21 to 35.50
ms for 1024 pairs; do not turn the small point-estimate difference into a stable
full-frame speedup claim. Short-iteration warnings and bimodality remain in the
raw distributions. Before results are in `artifacts/grv-benchmark-022/after`;
final controls are in `artifacts/grv-benchmark-023/fullstep-complete`.

The final actual-iteration-only EventPipe capture still attributes 31.0%
inclusive to grounding refresh, 19.3% to generic swept-circle detection and
12.5% to circle sweep geometry. Partition distribution accounts for 16.4%
inclusive, with voxel partition lookup at 11.0% exclusive. These are overlapping
sampled paths, not measured stage durations or proof of a lock bottleneck. The
complete contact owner contributes 4.9% inclusive in this sample. Capture with
the integrated 1024-pair filter,
`--profiler EP --launchCount 1 --warmupCount 3 --iterationCount 5 --iterationTime 500`;
artifacts are `fullstep-profile-complete` and
`fullstep-profile-complete-actual-only.json` under the same directory.

**Partition-phase validation:** Final local-stack Release/ReleaseLean solution
builds cover both `netstandard2.1` and `net8.0`; all 4401/4342 tests pass with
no failures or skips. Final OpenCover/Cobertura root counts are exactly
44,491/44,491 sequence points, 13,278/13,278 branches and 4,589/4,589 methods in
Release; Lean has 44,489/44,489 points, 13,278/13,278 branches and 4,588/4,588
methods. ReportGenerator also confirms zero uncovered lines, branches and
methods. DocFX passes with warnings as errors; API-site branding, repository
links and the workflow's local-link checks also pass. Logs, TRX and raw coverage
are in `artifacts/grv-benchmark-023/*-final-review`; rendered reports are in
`report-Release` and `report-ReleaseLean`. Regression coverage includes
empty-registry swap-back/reset/reuse, all-dimension high-ID storage, sorted
inspection, negative-ID validation, coverage deltas, retained identities,
mobility changes, awake transitions, foreign contexts and strict allocation
gates. Source review reports no remaining findings. No coverage exclusion
changes or upstream source edits were made. This is Windows source-stack
evidence; released-package validation remains deferred until the upstream
release.

**Experiment refinement, 2026-10-02:** The shared FixedMathSharp distance solver
now evaluates unit-interval endpoint signs as `C`, `A + 2B + C`, `B`, and
`A + B` for `f(t) = A*t*t + 2B*t + C`. Widened sums replace general rational
endpoint products only on `[0, 1]`. Bounded intervals, discriminants, root
construction, rounding and final distance mapping retain the existing owner.
Circle, sphere, capsule caps and rounded-cylinder cap consumers share this
change; no physics policy or second math kernel moves upstream.

Gravitas also omits host-yaw materialization when a body-owned requested pose
replaces it, or when the caller only needs scale. Exact matrix composition,
planar decomposition, ancestry/reflection/shear admission and mixed slab height
are unchanged. Bodyless poses still capture host yaw. Tests exercise nonidentity
parented transforms, requested-pose ownership and invalid transform hierarchies.

The new `circle-automatic-grounding` control probes after positional response,
matching the full-step geometry. The paired circle remains an initial
full-radius probe overlap because correction retains slop, but its normal
rejects support. Rows after the first can find the preceding row's circle below
them: setup checks exactly `PairCount - 32` supported bodies and repeatable
probe results. Independent contact pairs do not imply independent support
probes. The older maintenance control resets poses into overlap and remains a
separate unsupported workload.

Fresh matched radial controls use two launches, five warmups, fifteen 250-ms
iterations and the environment above. Error is half the 99.9% interval; all rows
report 0 B/op.

| Shared distance interval / scale | Before mean +/- error (ns) | After mean +/- error (ns) |
| -------------------------------- | -------------------------- | ------------------------- |
| Circle / 1                       | 3828.9 +/- 18.00           | 3304.82 +/- 17.454        |
| Sphere / 1                       | 3866.7 +/- 30.14           | 3302.94 +/- 26.690        |
| Circle moving away / 1           | 539.1 +/- 3.72             | 87.88 +/- 1.661           |
| Sphere moving away / 1           | 576.6 +/- 7.16             | 120.68 +/- 1.388          |
| Circle / 100000                  | 5455.9 +/- 27.68           | 4921.34 +/- 24.061        |
| Sphere / 100000                  | 5514.1 +/- 69.26           | 4980.02 +/- 53.401        |
| Circle moving away / 100000      | 554.3 +/- 17.01            | 89.89 +/- 0.490           |
| Sphere moving away / 100000      | 569.5 +/- 5.26             | 122.01 +/- 0.208          |

Regular interval cost falls 9.7-14.6%; moving-away misses fall 78.6-83.8%. The
direct planar capture control measures 813.7 +/- 8.88 versus 674.4 +/- 9.17 ns
at zero host yaw, and 1175.9 +/- 16.23 versus 792.9 +/- 5.11 ns at one radian
(17.1% / 32.6% lower, 0 B/op). For corrected automatic probes, the math-only
1024-pair control changes from 9929.9 +/- 250.65 us to 9233.5 +/- 168.92 us
(7.0% lower). The 64-pair 411.2 +/- 2.77 us and 406.4 +/- 7.00 us intervals
overlap. The math-only full step also overlaps its baseline: 36.695 +/- 0.710
versus 35.980 +/- 0.938 ms.

With both refinements the fresh 1024-pair full-step control measures 34.469 +/-
0.3244 ms at 0 B/op. Its matched fresh baseline is 36.695 +/- 0.710 ms (6.1%
lower point estimate); the committed partition-phase capture was already faster
at 35.573 +/- 0.3858 ms. Preserve baseline drift, short-iteration warnings and
multimodal distributions rather than treating a single capture as a universal
frame-time gain. The 64-pair combined result is 1.503 +/- 0.1040 ms and supports
no small-scene speedup claim. An independent two-launch confirmation with 500-ms
iterations measures 34.404 +/- 0.5312 ms at 0 B/op, consistent with the final
capture. It confirms the after result under a longer iteration protocol; it is
not a new matched before/after pair. The final corrected-pose 1024-pair probe
confirmation measures 9005.3 +/- 71.88 us at 0 B/op (9.3% below its fresh
baseline).

Raw results live under `artifacts/grv-benchmark-023/`:
`refinement-math-before-verified`, `refinement-math-after`,
`refinement-corrected-before-verified`, `refinement-corrected-math-after`,
`refinement-full-before`, `refinement-full-math-after`, `refinement-full-final`,
`refinement-full-confirmation`, `refinement-planar-control-verified`, and
`refinement-corrected-final`. Failed setup/build captures are preserved
separately and supply no timing evidence. The initial short full-step pilot
reported 3.44 KB/op; the longer matched controls report 0 B/op and existing
strict allocation gates remain authoritative.

**Refinement validation:** Both local-stack solutions build for `netstandard2.1`
and `net8.0` in Release/ReleaseLean with zero warnings/errors. Gravitas passes
4403/4344 tests; FixedMathSharp passes 4138/4117 core tests plus 49 Chronicler
tests per configuration, with no failures or skips. Exact OpenCover counts are:

| Repository / configuration   | Covered sequence points | Covered branches | Covered methods |
| ---------------------------- | ----------------------- | ---------------- | --------------- |
| Gravitas / Release           | 44496 / 44496           | 13280 / 13280    | 4589 / 4589     |
| Gravitas / ReleaseLean       | 44494 / 44494           | 13280 / 13280    | 4588 / 4588     |
| FixedMathSharp / Release     | 52675 / 52675           | 12138 / 12138    | 3935 / 3935     |
| FixedMathSharp / ReleaseLean | 52768 / 52768           | 12138 / 12138    | 3931 / 3931     |

No coverage exclusions or allocation gates were changed. All 32 raw final
benchmark GC records have zero allocation and collection counters. Independent
source/evidence review reports no actionable findings. Builds, TRX and coverage
are in `artifacts/grv-benchmark-023/refinement-<repository>-<configuration>-*`.
Rendered reports verify core/FluentAssertions from the core suite and Chronicler
from its owning suite (85/85 lines, 12/12 branches, 18/18 fully covered methods
in both configurations). Merging Chronicler's partial core dependency capture
into the complete core report lost three covered lines in the Lean aggregation;
the raw complete capture and core-only rendered control both remain fully
covered. The failed merged-format control is retained, without modifying raw
visits. ReportGenerator confirms zero uncovered lines, branches and fully
covered methods for every owning assembly/configuration; see
`refinement-coverage-summary.json`. All 463 upstream Short smoke cases have
successful child exits and populated statistics, including 62 zero-allocation
finite-axis controls. The smoke uses one launch, three warmups and three 10-ms
iterations; its timings are not performance evidence. Both DocFX builds pass
with warnings as errors, and API resources, branding, repository actions and
local links pass. Logs and exports are `refinement-math-smoke*` and
`refinement-<repository>-docfx.log`. FixedMathSharp must release before
Gravitas's released-package revalidation; this evidence uses Windows sibling
source with `UseLocalLsfStack=true`.

**Prepared-query refinement (2026-10-03):** Closest/all-hit 2D services now pass
their admitted exact segment and representable length through the existing
detector and compound parts. Circle CCD reuses its admitted displacement after
exact endpoint addition. Raw entry points retain their rejection order;
normalization, authored support distance, and the saturating reverse-convex path
are unchanged. No new runtime type, math owner or upstream edit is needed.

Automatic 2D grounding reuses the closest-hit loops with the body's existing
pure acceptance policy. Only accepted aggregate collider witnesses enter the
distance/owner-ID reducer. Compound parts still reduce before support filtering;
ray candidate counts and static-style sweep exclusions are unchanged. This
removes the per-body hit collection, retained capacity and additional hit sort.
Diagnostics, support writes and callback lifetime validation still follow
selection. Public all-hit buffers and explicit `QuerySupport` keep their
contracts.

Fresh controls start from Gravitas `ee613db` and FixedMathSharp `6789c09`, using
the same Windows/local-stack environment above. The requested CLI argument is
`--affinity 3`; BenchmarkDotNet records the job display as `Affinity=11` in
every capture. Preserve that command/display pair when reproducing the run. The
1024-pair scenes contain 2048 colliders. Timings below are ms/op, with the
half-width of the 99.9% confidence interval; every row reports 0 B/op.

| Capture                                | Corrected-pose automatic probes | Full diagonal simulation step |
| -------------------------------------- | ------------------------------- | ----------------------------- |
| Fresh committed baseline               | 10.3778 +/- 0.1907              | 39.3280 +/- 0.6776            |
| Prepared segment only                  | 10.1208 +/- 0.1868              | 38.2636 +/- 0.5886            |
| Prepared segment plus nearest accepted | 9.8572 +/- 0.2201               | 38.8105 +/- 0.7148            |

These use two launches, five warmups and fifteen iterations: 250 ms for probes,
500 ms for full steps. The probe baseline/length captures have short observed
iterations; length/combined distributions are multimodal. The length/combined
full runs also include the detection-only control (4.4819 +/- 0.0606 and 4.5939
+/- 0.0672 ms); the fresh full baseline selects only the full step. Probe point
estimates improve 5.0% from today's baseline, but neither the isolated length
nor selection increment establishes a standalone speedup. Full-step intervals
overlap, including the combined result's higher point estimate than length
alone. No stable full-step gain is claimed. These fresh baselines are slower
than the preceding phase; preserve that drift.

`circle-ground-query-selection` pairs collect/sort/filter against nearest
accepted selection at identical corrected poses, with prepared segment reuse in
both paths. Setup compares the complete witness for every body and checks 32/992
supported bodies at 64/1024 pairs. With the same two-launch job and 500-ms
iterations, its ms/op results are:

| Pairs | Collect/sort/filter | Nearest accepted   | Allocation |
| ----- | ------------------- | ------------------ | ---------- |
| 64    | 0.4099 +/- 0.00675  | 0.4017 +/- 0.00612 | 0 B/op     |
| 1024  | 9.2889 +/- 0.12977  | 9.1682 +/- 0.16019 | 0 B/op     |

Intervals overlap in both sizes; the paired run also reports multimodality and
outliers, so retain the raw samples. Retain the existing-owner streaming design
for its removed per-body storage and hit sort, with exact witness parity and no
measured material regression; do not claim an isolated selection speedup. Raw
JSON, logs and summaries are under `artifacts/grv-benchmark-023/` as
`refinement3-ground-{before,length,stream}`,
`refinement3-full-{before,length,stream}`, `refinement3-selection-paired`,
`refinement3-timing-summary.json` and `refinement3-selection-summary.json`.

**Prepared-query validation:** Release/ReleaseLean solution builds cover both
targets with zero warnings/errors. All 4430/4371 tests pass without skips,
including 27 new prepared-query and automatic-probe regression cases. Exact
OpenCover counts are:

| Configuration | Covered sequence points | Covered branches | Covered methods |
| ------------- | ----------------------- | ---------------- | --------------- |
| Release       | 44492 / 44492           | 13284 / 13284    | 4595 / 4595     |
| ReleaseLean   | 44490 / 44490           | 13284 / 13284    | 4594 / 4594     |

ReportGenerator also verifies zero uncovered lines, branches and fully covered
methods (56222/56220 lines, 16290 branches, 5405/5404 methods). Existing
collector exclusions and strict allocation gates are unchanged. The first
complete capture found a missing public all-hit geometric-miss branch after
grounding stopped using that path; a direct overlapping-bounds/missed-circle
regression closes it. The failed coverage control is retained separately. Final
builds, TRX, raw coverage and rendered reports use
`refinement3-final-Gravitas-<configuration>-*`; the exact rendered counts are in
`refinement3-coverage-summary.json`. All ten affected public 2D raycast/sweep
Short smoke rows have populated statistics, successful child exits and 0 B/op;
short timings are not performance evidence. Across the before/length/combined,
paired and smoke captures, all 34 raw GC records have zero allocation/collection
counters. Exports and validation counts are `refinement3-query-smoke*` and
`refinement3-benchmark-validation.json`. Independent source, evidence and
ponytail review found no remaining actionable issues. Gravitas DocFX builds with
warnings as errors and passes API resource, branding, repository-action and
local-link checks; log: `refinement3-Gravitas-docfx.log`.

**Further refinement (2026-10-04):** A fresh actual-iteration-only CPU profile
of the committed `6c0c2f0` workload still attributes 15.666% exclusive CPU to
GridForge typed voxel partition lookup. Callers divide between 2D positional
correction/repartition (43.02%, including fixture reset), planar sweeps (29.64%)
and collider distribution (27.33%). Automatic 2D grounding contributes 27.072%
inclusive; complete shape capture contributes 14.712%, including 7.517% in the
forward-scaled planar transform. These shares overlap and are not independent
frame costs. `refinement4-full-profile-actual-only.json` retains the
attribution. That profiled child reports 24,624 raw allocated bytes over 12
operations; its cause was not attributed. Preserve this exception separately
from the successful repeated allocation gates rather than assigning it to
instrumentation.

Centered 2D collider snapshots repeatedly transform zero local coordinates.
FixedMathSharp's existing wide transform now returns the exact origin when both
local point and displacement are zero, before constructing rotated wide
numerators. This is policy-neutral arithmetic, applies through both public
forward-scaled overloads and preserves zero/mirrored/extreme scales and full
origin/yaw ranges. Nonzero input keeps the existing fused arithmetic and final
rounding. Host scale admission and snapshot ownership remain unchanged.

With two launches, five warmups and fifteen 500-ms iterations, the centered
256-operation microbenchmark changes from 1196.942 +/- 12.674 to 16.280 +/-
0.229 ns per transform; the nonzero control is 1536.411 +/- 15.456 / 1500.423
+/- 18.673 ns. Both report 0 B/op. Two matched physics captures isolate only
this math change:

| Capture        | Corrected-pose automatic probes, ms | Full 1024-pair diagonal step, ms |
| -------------- | ----------------------------------- | -------------------------------- |
| Initial before | 9.478350 +/- 0.121474               | 36.543975 +/- 0.468481           |
| Initial after  | 9.963933 +/- 0.202656               | 34.223361 +/- 0.549934           |
| Repeat before  | 9.926890 +/- 0.167635               | 39.031101 +/- 0.605361           |
| Repeat after   | 9.767688 +/- 0.210000               | 33.779813 +/- 0.583872           |

All summaries report 0 B/op. One initial after grounding child reports 24,624
raw allocated bytes over 53 operations despite the final-launch zero summary;
both repeat captures have exact zero raw bytes/collections in every child.
Preserve the initial exception rather than inferring allocation from the
last-launch summary. Both full-step pairs improve, but the before values vary
materially; do not collapse them into one precise speedup percentage. Automatic
grounding does not perform this transform work. Its repeat intervals overlap,
preserving the control after the initial shift. Logs and JSON are
`refinement4-transform-{before,after}`, `refinement4-{before,math-after}` and
`refinement4-math-repeat-{before,after}`.

The GridForge experiment resolves the existing exact type key directly through
the provider while holding the same voxel monitor. Typed materialization,
including boxed value copies, stays inside the lock. `HasPartition` and default
lookup reuse this owner. Reset callbacks can clear or pool a still-published
payload, so an unlocked positive-hit shortcut would weaken the current contract.
No cache or synchronization redesign is introduced. Initial first-slot lookup
means change from 19.550/19.555 ns to 18.682/18.516 ns at one/256 voxels, but
the generic provider control also shifts. Native inlining removes a generic call
while growing the voxel machine-code body from 191 to 416 bytes. A fresh
runtime-only toggle confirms retrieval at 256 voxels, 19.6266 +/- 0.2135 to
18.6804 +/- 0.1832 ns; presence checks, 20.0508 +/- 0.2773 to 18.5102 +/- 0.2138
ns; and default lookup, 19.4804 +/- 0.2300 to 18.9223 +/- 0.2364 ns. The
untouched type-key control remains 2.0077 +/- 0.0230 / 2.0146 +/- 0.0224 ns. The
full 1024-pair step remains 34.0187 +/- 0.4391 / 33.9997 +/- 0.4845 ms and
corrected-pose probes remain 9.7303 +/- 0.1491 / 9.7157 +/- 0.1252 ms, with
overlapping intervals. Retain the small lookup improvement; no measurable
full-step or grounding gain is claimed. All 24 child records in these matched
micro/consumer captures have zero raw allocated bytes and collections, alongside
0 B/op summaries. The controls use the same
two-launch/five-warmup/fifteen-500-ms-iteration job and CLI `--affinity 3`.
Evidence is `refinement4-grid-repeat-{before,after}` and
`refinement4-grid-consumer-{before,after}`. GridForge tracks the coordinated
experiment as `GF-Benchmark-013`. Its benchmark runner also propagates the
source-stack build properties explicitly, preserving Release/Lean and CLI job
settings after two reproduced generated-child build failures.

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

| 64-probe workload               | Existing implementation, microseconds | Rejected final candidate, microseconds |
| ------------------------------- | ------------------------------------- | -------------------------------------- |
| Ray, one target, unsupported    | 179.834 +/- 2.892                     | 194.275 +/- 3.036                      |
| Ray, one target, supported      | 259.449 +/- 3.627                     | 283.795 +/- 5.292                      |
| Public closest ray control      | 169.820 +/- 2.688                     | 167.119 +/- 4.718                      |
| Ray, eight targets, unsupported | 759.658 +/- 12.927                    | 597.001 +/- 9.941                      |
| Ray, eight targets, supported   | 838.863 +/- 12.895                    | 686.102 +/- 10.794                     |

The one-target unsupported/control baselines are the old-runtime control repeat;
supported and dense baselines are the initial old-runtime capture. Dense ray
grounding improves about 18-21%, but repeated sparse ray grounding regresses
about 8-10% with disjoint intervals. Sphere results do not establish a
consistent dense-case gain; public closest-sphere control intervals overlap.
This is a poor default tradeoff. All 20 children in the rejected final candidate
have zero raw allocated bytes/collections and successful exits. Evidence retains
`refinement4-ground3d-before-boolean`,
`refinement4-ground3d-control-repeat-before`, the superseded variants, and
`refinement4-ground3d-guarded`. The ignored
`refinement4-ground3d-rejected-runtime.patch` preserves the rejected runtime
diff.

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

| Owner                                | Tests, Release / Lean | Sequence points, Release / Lean | Branches, each | Methods, Release / Lean |
| ------------------------------------ | --------------------- | ------------------------------- | -------------- | ----------------------- |
| FixedMathSharp core/FluentAssertions | 4140 / 4119           | 52678 / 52771                   | 12142          | 3935 / 3931             |
| GridForge                            | 1171 / 1171           | 9333 / 9333                     | 4171           | 1145 / 1145             |
| Gravitas                             | 4467 / 4408           | 44492 / 44490                   | 13284          | 4595 / 4594             |

Every listed point, branch and method is covered. FixedMathSharp's Chronicler
suite adds 49 passing tests per configuration and an independently filtered 100%
report. Rendered reports confirm 100% lines, branches and fully covered methods
without changing exclusions: FixedMathSharp/FluentAssertions and GridForge match
the counts above; Gravitas' report aggregation totals 56222/56220 lines, 16290
branches and 5405/5404 methods, all covered. Preserve raw and rendered counts
separately. Evidence is `refinement4-coverage-summary.json`,
`refinement4-<owner>-<configuration>-*` and
`refinement4-retained-Gravitas-<configuration>-*`. Candidate-era captures remain
historical evidence, not validation of the restored runtime.

All 30 lookup smoke cases, one Lean generated-runner Dry case, five existing 3D
all-hit query cases and all 16 retained grounding fixture cases complete with
successful child exits and exact zero raw allocated bytes/GC collections. The
Lean runner retains its configuration and all three source-stack child
properties. `refinement4-benchmark-validation.json` audits populated statistics,
summary allocation and every raw child record, retaining the documented initial
math-capture exception. Independent source, evidence and ponytail review found
no actionable issues. These local source-stack results precede release-package
validation; release FixedMathSharp first. All three DocFX builds pass with
warnings as errors, API resources, branding, repository actions and local links
verified. Logs are `refinement4-<owner>-docfx.log`.

A fresh actual-iteration-only profile after the retained upstream changes
attributes 16.151% exclusive sampled CPU to typed voxel lookup and 12.626%
exclusive / 20.006% inclusive to 2D collider distribution. Automatic 2D
grounding is 31.764% inclusive. The centered-transform arithmetic frame is
effectively removed. These overlapping attribution shares are not matched timing
gains; the profile child has zero raw allocation/collections. Evidence is
`refinement4-final-profile-actual-only.json` and its trace capture.

**Refinement 5 (2026-10-04), retained and ready for review:** Pure 2D candidate
gathering now visits partition membership directly instead of sorting partitions
and copied IDs before deduplication. The final candidate collider-ID sort
remains, keeping the narrow-phase/hit reduction order stable. No public result
ordering changes. Three-dimensional ray/sweep partition gathering already has
direct membership traversal; the separate planar candidate-index rebuild still
uses sorted member copies, so those paths must not be conflated.

Matched query controls use two launches, five warmups and fifteen 250-ms
iterations, CLI `--affinity 3`, the local source stack and environment above.
Errors are half of the 99.9% intervals; every summary reports 0 B/op:

| Query-gather control                   | Before mean +/- error | After mean +/- error |
| -------------------------------------- | --------------------- | -------------------- |
| Full diagonal step, 64 pairs           | 1.507 +/- 0.2240 ms   | 1.350 +/- 0.1643 ms  |
| Full diagonal step, 1024 pairs         | 32.233 +/- 0.3401 ms  | 31.567 +/- 0.3489 ms |
| Corrected automatic probes, 64 pairs   | 407.6 +/- 7.18 us     | 394.9 +/- 5.53 us    |
| Corrected automatic probes, 1024 pairs | 9297.6 +/- 124.25 us  | 9111.8 +/- 138.72 us |

Full-step and 1024-probe intervals overlap; this capture does not establish a
stable full-step gain. The paired selection control preserves both complete
witness paths; its 1024 nearest-accepted row is 8951.9 +/- 233.13 versus 8819.0
+/- 404.23 us, also overlapping. Retain the lean gather for removed redundant
work and ordering parity; final correctness and coverage gates pass. Short
observed iterations, outliers and multimodality remain in the raw capture and
audit; do not discard them when interpreting the reported intervals.

The density probe replays compiled admission/canonical owners outside timing. At
1024 diagonal pairs, unit cells produce 123904 active partitions, 123904 dynamic
memberships, 57344 static memberships and 57344 emitted links; canonical
ownership rejects 56320, leaving 1024 accepted pairs. Sixteen-unit cells produce
1024 active partitions and 3969 links, of which bounds reject 2945, also leaving
1024 accepted pairs. At 256 pairs, unit cells produce 30976 active partitions
and 14336 links; canonical ownership rejects 14080, leaving 256 accepted pairs.
Sixteen-unit cells produce 256 active partitions and 945 links, of which bounds
reject 689, leaving the same 256 pairs. The 64-pair cases produce 7744 fine-cell
partitions and 3584 links, or 64 coarse-cell partitions and 189 links; canonical
ownership and bounds reject 3520 and 125 links respectively. All 12 final probe
cases pass witness and candidate checks. Fine-cell memberships and links scale
exactly with pair count in this fixture. These are sparse local memberships
replicated over coverage, not dense collision neighborhoods. Known correction
counts are fixture invariants, not instrumented counts of production correction
calls.

Bounds-first pure 2D rejection is retained: disjoint bounds skip costly physical
admission before canonical ownership, while bounds-accepted pairs retain the
existing physical filters. Six matched full-step controls use the same job and 0
B/op summaries. Representative means +/- errors are 200.219 +/- 1.825 -> 194.252
+/- 1.330 ms for 1024 diagonal pairs/unit cells, 31.542 +/- 0.207 -> 29.849 +/-
0.708 ms for 1024 diagonal pairs/16-unit cells, and 23.902 +/- 0.467 -> 22.804
+/- 0.162 ms for 1024 axis pairs/16-unit cells. Do not copy this gate into 3D:
retained pair culling/lifecycle rules require a different admission boundary.
Public filtering, notifications and stale-pair cleanup must retain their
dimensional contracts.

A second runtime-only bounds-gate toggle preserves the query-gather change in
both versions and repeats the same six full-step rows. Representative first and
repeated captures are:

| Bounds-first control               | Initial before / after, ms +/- error  | Repeat before / after, ms +/- error   |
| ---------------------------------- | ------------------------------------- | ------------------------------------- |
| 1024 diagonal pairs, unit cells    | 200.219 +/- 1.825 / 194.252 +/- 1.330 | 201.122 +/- 1.795 / 200.645 +/- 4.248 |
| 1024 diagonal pairs, 16-unit cells | 31.542 +/- 0.207 / 29.849 +/- 0.708   | 30.448 +/- 0.627 / 29.967 +/- 0.682   |
| 1024 axis pairs, 16-unit cells     | 23.902 +/- 0.467 / 22.804 +/- 0.162   | 23.718 +/- 0.529 / 23.333 +/- 0.465   |

The repeat intervals overlap in these rows; initial gains are not stable speedup
evidence. Retain the cheap exact disjoint rejection for avoiding unnecessary
admission work, with no measured material regression and behavior parity. Final
correctness and coverage gates pass. Small-scene baseline drift further limits
attribution. Repeat artifacts are `refinement5-admission-repeat-{before,after}`.
The complete fixture now exposes 36 rows: 64/256/1024 pairs,
Axis/Diagonal/Rotated, one-/16-unit X/Z cells and two measured methods. All 36
smoke rows pass. The final nine-row full-step count-scaling matrix uses two
launches, five warmups and fifteen 250-ms iterations, CLI `--affinity 3`
(displayed as `11`), and the local source stack. Every row reports 0 B/op:

| Geometry / X/Z cell size | 64 pairs, ms +/- error | 256 pairs, ms +/- error | 1024 pairs, ms +/- error |
| ------------------------ | ---------------------- | ----------------------- | ------------------------ |
| Axis / 16                | 0.9574 +/- 0.0537      | 3.9249 +/- 0.0265       | 23.0800 +/- 0.1873       |
| Diagonal / 16            | 1.1440 +/- 0.0105      | 5.5369 +/- 0.0864       | 30.0562 +/- 0.5735       |
| Diagonal / 1             | 9.7818 +/- 0.1831      | 45.0489 +/- 0.4199      | 201.9929 +/- 4.3606      |

Full-frame cost is not uniformly linear. Retained distribution sorts active
partitions with `SortInPlace`, an O(P log P) operation required for
deterministic callback order. The timings are consistent with an increasing
ordered working set alongside covered memberships; this source-based explanation
is not a profiled attribution of the measured scaling. Fine-cell coverage
remains a substantial configuration cost. Evidence is
`refinement5-final-scaling`.

An early accepted-key `Contains` before canonical ownership was tested and
rejected. Despite 56320 potential repeated-key hits in the fine 1024 diagonal
probe, the measured full step regresses from 194.252 +/- 1.330 to 236.215 +/-
2.768 ms, a 21.60% regression with disjoint intervals. Coarse axis also
regresses, 22.804 +/- 0.162 to 24.146 +/- 0.114 ms. Preserve these observations;
avoid an extra hot-path hash probe based only on potential skip counts. No
runtime early-Contains candidate is retained.

Evidence: `artifacts/grv-benchmark-023/refinement5-query-{before,after}`,
`refinement5-density-probe/density.json`, `refinement5-admission-{before,after}`
and `refinement5-dedup-after`, plus the admission repeats, final scaling and
smoke captures. `refinement5-audit-captures.py --include-final` passes;
`refinement5-capture-audit.json` verifies 10 captures, 96 positive rows and 151
successful children. All 151 raw GC records contain exactly zero allocated bytes
and Gen0/1/2 collections, all memory summaries are zero, and every child uses
the local-stack build flags. Small-scene baseline drift prevents claiming a
large portable gain from the initial admission capture. Revisit 3D streaming
only after profiling explains its sparse-ray regression. Released-package
validation remains a later release gate for the coordinated upstream changes.

**Refinement-5 final correctness/coverage gates:** Local-stack Release and
ReleaseLean solution builds cover `netstandard2.1` and `net8.0` with zero
warnings/errors. Release passes **4473 tests** and Lean **4414**, with no
failures or skips. Raw and rendered coverage independently retain 100% coverage:

| Configuration | Raw sequence points | Raw branches  | Raw methods | Rendered lines | Rendered branches | Rendered fully covered methods |
| ------------- | ------------------- | ------------- | ----------- | -------------- | ----------------- | ------------------------------ |
| Release       | 44495 / 44495       | 13286 / 13286 | 4596 / 4596 | 56225 / 56225  | 16292 / 16292     | 5406 / 5406                    |
| ReleaseLean   | 44493 / 44493       | 13286 / 13286 | 4595 / 4595 | 56223 / 56223  | 16292 / 16292     | 5405 / 5405                    |

Evidence is `refinement5-final-Gravitas-{Release,ReleaseLean}*`. Upstream edits
in this pass are documentation-only; the previously committed FixedMathSharp and
GridForge changes retain their recorded full owning-suite coverage. The final
nine-row scaling capture, 36-row circle fixture smoke and five public 3D query
controls all pass. The 41 short smoke rows verify execution, contracts and
allocation behavior; they do not establish timing improvements. The joint audit
includes these final captures. Gravitas, FixedMathSharp and GridForge DocFX
gates pass with warnings as errors, zero warnings/errors and verified local API
links, resources, branding and repository actions; evidence is
`refinement5-doc-gates.ps1` and its logs.

**Closure decision, 2026-10-04:** Close this signal under the selected
predictable-scaling/published-cost criterion with the final captures, audits and
correctness/coverage gates passed. Retain the measured ownership/storage/math
repairs and lean query gathering; retain exact cheap bounds rejection without
claiming a stable full-step speedup. Reject the regressing early duplicate hash
probe and streamed 3D grounding. Fine-cell coverage has a high configuration
cost that must remain published with counts, geometry and uncertainty; closure
does not certify a fixed-Hz budget. No speculative optimization or new cache is
required merely to close the signal. Reopen with a concrete workload regression,
unexpected scaling or a host-specific performance requirement. Released-package
validation remains a future release gate.

### GRV-Benchmark-022 — Exact Pure-2D Circle Contact Cost

**Discovered:** 2026-09-30 while implementing GRV-Issue-083. **Status:** Closed
2026-10-02: shared-owner optimization is committed upstream; retain the exact
contact path under the no-further-change decision below. Larger-scene scaling is
documented separately in closed GRV-Benchmark-023. **Owner:** FixedMathSharp's
existing centered-capsule relation with two zero core lengths; Gravitas forwards
complete contact anchors, normal, depth and clamp.

Replacing saturated scalar circle arithmetic fixes large-coordinate false hits,
tiny-distance underflow and incorrect depth/clamping. However, the complete
query adds substantial cost even to ordinary cases where the old result was
correct. These fixtures preserve the same geometry and measured dispatcher
before/after; setup verifies classification, normal, depth and clamp, not exact
anchor-representation equivalence with the old rounded-offset implementation.

Windows 11 / i7-9700K, .NET 8.0.29, SDK 10.0.302; two launches, five warmups,
fifteen 250 ms iterations, CPU mask 3, BelowNormal launcher and
`DOTNET_PROCESSOR_COUNT=2`, with one heavy workload at a time. Values are
**microseconds per direct query**, not simulation frames. All rows allocate **0
B/op**. Error is the half-width of the 99.9% confidence interval.

| Geometry                   | Previous scalar path | Initial complete query | Optimized complete query | Optimized error |
| -------------------------- | -------------------: | ---------------------: | -----------------------: | --------------: |
| Axis                       |              0.06659 |                 5.4363 |                   0.9125 |         0.00248 |
| Coincident                 |              0.04828 |                 2.6035 |                   0.6277 |         0.00371 |
| Diagonal                   |              0.11101 |                 4.3406 |                   1.4513 |         0.00853 |
| Large axis                 |              0.10016 |                 5.4263 |                   0.9548 |         0.00236 |
| Rotated                    |              0.06712 |                 5.8454 |                   1.6113 |         0.00974 |
| Bounds-admitted separation |              0.02542 |                 0.9027 |                   0.2182 |         0.00077 |

The initial slowdown was 35.5-87.1x. Shared-owner work removes 66.6-83.2% of
that complete-query time, but the remaining cost is still 8.6-24.0x the old
scalar path. This is a substantial reduction, not scalar-cost parity or an
accepted full-simulation budget. Retain the raw distributions, including a
baseline bimodality warning; the final Gravitas run reported no multimodality
warning. The old path is not a full-domain correctness target. In particular,
comparing an exact squared distance alone would not repair normal, depth or
anchor terms.

**Shared-owner optimization, 2026-09-30:** EventPipe sampling of the complete
circle query's Axis fixture put approximately 45% of inclusive query samples in
depth reduction and 29% in its product square root; these overlap and are not
additive. Parameter construction, local offsets, normalization and retained
radial terms added work. FixedMathSharp now represents point cores directly over
denominator one, reuses its narrower exact root, classifies conceptual depth
before rounding, and skips irrelevant axis rotations/zero offsets. Cardinal
normalization skips roots and division. Both dimensions share the existing
residual owner, whose retained low word is computed directly with equivalent
unchecked integer arithmetic. No second Gravitas solver, approximation, cache,
public API or dependency was added.

Matched upstream `zero-core-contact` runs use the same settings and geometry as
above, plus a nonzero-core capsule control. Values remain **microseconds per
query**, with **0 B/op** throughout:

| Geometry                | Circle before | Circle optimized | Sphere before | Sphere optimized |
| ----------------------- | ------------: | ---------------: | ------------: | ---------------: |
| Axis                    |        5.3077 |           0.9135 |        9.4406 |           1.1843 |
| Coincident              |        2.6306 |           0.6104 |        6.6275 |           0.7644 |
| Diagonal                |        4.4942 |           1.4536 |        8.7157 |           1.7991 |
| Large axis              |        5.6523 |           0.9243 |        9.8707 |           1.2599 |
| Rotated                 |        6.1178 |           1.6234 |        8.9603 |           1.7449 |
| Separation              |        0.9026 |           0.2007 |        2.3985 |           0.1489 |
| Nonzero capsule control |        6.0087 |           3.8004 |       10.1398 |           7.7200 |

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
Baseline runtime source is Gravitas `b8b4115` with FixedMathSharp `787afae`; the
`after` capture adds only the Gravitas zero-core delegation; `optimized` also
includes the upstream changes described above. Reports and raw measurements are
in `artifacts/grv083/before`, `after` and `optimized`; build logs are alongside.

**Shared normalization follow-up, 2026-10-02:** The current local stack already
contains the first optimization above. Actual-iteration EventPipe samples put
local radial-feature work at 41.7% of the diagonal query and 48.5% of the
rotated query, with normalization at 27.6% and 21.5% respectively; inclusive
shares overlap. Depth now contributes only 4.1% and 2.4%. FixedMathSharp's
existing `Vector2d.GetScaleNormalized` and `Vector3d.GetScaleNormalized`
rescaled their already bounded vectors a second time through
`GetScaledMagnitude`.

Both helpers now take the square root of the same rounded component squares
directly. The dominant scaled component rounds to exactly +/-One, including
`MinValue` whose excess over the saturated absolute-value scale is below half a
raw unit. The removed divisions and multiplication were therefore identities;
sum order, square-root input, rounding, zero-input preconditions and all
retained anchor terms are unchanged. No new branch, arithmetic owner, circle
kernel, cache, public API or dependency was added. The upstream tests compare
the prior path across 4913 raw-edge triples and 2048 deterministic unequal-scale
triples, with independent BigInteger nearest-even ratio checks.

Matched direct queries use the same machine/settings as above. Before sources
are Gravitas `e4ebf1d` and FixedMathSharp `6a6a7e3`; after adds only the
upstream normalization change. Values are **microseconds/query**, all **0
B/op**:

| Geometry   | Current-stack before |  After | After error |
| ---------- | -------------------: | -----: | ----------: |
| Axis       |               0.9344 | 0.8944 |     0.00361 |
| Coincident |               0.6396 | 0.6309 |     0.00660 |
| Diagonal   |               1.4753 | 1.3685 |     0.01076 |
| Large axis |               0.9574 | 0.9478 |     0.00973 |
| Rotated    |               1.6330 | 1.5728 |     0.01852 |
| Separation |               0.2223 | 0.2191 |     0.00211 |

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
fine-voxel fanout. This measures repeated penetration from zero initial
velocity; it is not a moving-impact, sliding or restitution workload.

Both full-step and direct-dispatcher attribution rows include body reset. The
full step includes partition refresh/distribution, contact response, grounding
and bookkeeping. Values below are **milliseconds/invocation**, all **0 B/op**:

| Pairs | Geometry | Full step before | Full step after | After error |
| ----- | -------- | ---------------: | --------------: | ----------: |
| 64    | Axis     |           1.1483 |          1.1898 |     0.03229 |
| 64    | Diagonal |           1.5143 |          1.6085 |     0.11412 |
| 64    | Rotated  |           1.4777 |          1.5043 |     0.13951 |
| 1024  | Axis     |          29.4562 |         28.4869 |     0.74434 |
| 1024  | Diagonal |          36.8367 |         35.9930 |     0.46672 |
| 1024  | Rotated  |          31.2055 |         31.4762 |     0.62095 |

Full-step changes are mixed, with overlapping intervals, bimodality in after
rows and short-iteration warnings. They do not establish an overall simulation
speedup or an accepted frame budget. The follow-up profile identifies grounding
and partition work as larger targets than contact anchors; see
GRV-Benchmark-023.

**Closure decision, 2026-10-02:** Retain the exact shared contact owner and the
normalization improvement. Further circle-specific kernels, anchor shortcuts or
caches are not justified by the full-step profile. The premium over the
incomplete scalar predecessor remains reference evidence, not a parity target.
Revisit shared circle geometry when a representative host workload and explicit
budget identify it as a material bottleneck. Investigate the separately measured
partition/grounding scaling before accepting the large-scene frame cost. This
closes the direct-query investigation, not an acceptance of whole-frame cost.
GRV-Benchmark-023 has different owners and scaling failure modes across
partition rental, distribution and grounding; it remains a separate signal even
though the circle workload exposed it.

**Reproduce this follow-up:** Set `UseLocalLsfStack=true` and
`DOTNET_PROCESSOR_COUNT=2`, build Release with
`-p:UseLocalLsfStack=true -m:1 -p:BuildInParallel=false`, then run each
selection with
`--launchCount 2 --warmupCount 5 --iterationCount 15 --iterationTime 250 --affinity 3 --exporters json`,
a BelowNormal launcher and one heavy workload. The direct before is
`artifacts/grv-benchmark-022/direct-before`, full-step before is
`coarse-before`, and matched after is `after`. Uninstrumented captures are the
timing/allocation evidence; `direct-profile`, `workload-profile` and
`coarse-profile` retain separate EventPipe traces and actual-iteration
sampled-CPU analysis. The exploratory unit-cell run is partial and must not be
mixed into the matched table. Upstream controls and focused test logs are in
FixedMathSharp's `artifacts/grv-benchmark-022`.

**Final validation, 2026-10-02:** With `UseLocalLsfStack=true`, both
repositories' full solution builds pass for `netstandard2.1` and `net8.0` in
Release and ReleaseLean with zero warnings/errors. Gravitas passes 4363/4304
tests and FixedMathSharp passes 4125/4104 core tests respectively, plus 49
Chronicler integration tests in each configuration; all have zero
failures/skips. Each repository/configuration retains exact 100% reachable line,
branch and method coverage. OpenCover reports no unvisited sequence points,
branches or methods; CI-style ReportGenerator gates independently report zero
uncovered counts. Raw collector XML, TRX, build/test logs and HTML/JSON
summaries are under each repository's `artifacts/grv-benchmark-022`, including
`coverage-Release`, `coverage-ReleaseLean`, `report-Release` and
`report-ReleaseLean`. Independent correctness and simplification review found no
actionable issues. FixedMathSharp's normalization change is committed as
`00a38bd`; Gravitas benchmark/docs changes remain pending review.

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
[`2026-08-03-mesh-scale-rebuild-throughput-plan.md`](2026-08-03-mesh-scale-rebuild-throughput-plan.md).

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

**Follow-up 2026-10-12:** Fresh exact-geometry controls measure roughly
13/52/206 ms at those triangle counts, all 0 B/op. Direct calls to the unchanged
FixedMathSharp projected-circle/triangle sweep reproduce the dominant cost
without a Gravitas manifold, BVH or response. The owning
[FixedMathSharp throughput signal](../../../../FixedMathSharp/docs/feature-work/benchmark-signal-hardening-backlog.md#finite-slab-projected-circletriangle-sweep-throughput)
records the literal fixture, independent math probe and next profiling step.
This instrumentation signal remains closed; the newly isolated math throughput
work belongs upstream.

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
