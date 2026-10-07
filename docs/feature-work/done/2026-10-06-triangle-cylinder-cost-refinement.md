# Triangle/Cylinder Cost Refinement

**Status:** Complete; GRV-Benchmark-018 closed 2026-10-06.  
**Archived:** 2026-10-06.

This investigation addresses GRV-Benchmark-018. Starting revisions are Gravitas
`32d2cdf` and FixedMathSharp `340f91f`. The fresh baseline includes the shared
math improvements from the preceding cone and capsule/slab investigations;
the September correctness-repair measurements are historical comparisons.

## Retained Changes

Two reciprocal triangle-edge charts now share their exact coefficient
construction. Each chart independently passes the existing cap, triangle-cone
and core-region admission checks. If both are admitted, the first finishes
before the shared algebra is reversed for the second. Admission coefficients
are rewritten from exact dot products; the previous chart cannot leave stale
endpoint-region data behind. Chart order and strict winner comparisons preserve
canonical ties.

For `n(t)=first+t*second`, the swapped chart is
`nbar(s)=second+s*first=s*n(1/s)`. The shared quadratic and quartic coefficients
therefore reverse at their fixed nominal degrees, with the derived derivative
terms changing sign. Zero leading/trailing coefficients retain their nominal
places. Reversal preserves the existing power-of-two normalization because it
preserves each normalized coefficient multiset. Tests compare every coefficient
and sign against an independent swapped-basis build, including zero
coefficients, dirty caller-owned admission slots and wide signed inputs.

Opposite analytic directions reuse `R²(Nx²+Nz²)` and `RawScale²|N|²` in the
working candidate's existing disjoint slots. Reuse begins only after an
admitted build: endpoint-region policy can admit the second direction alone.
The candidate's signed support term is rebuilt for each direction. Alias tests
compare the complete reused candidate against independent construction for
positive, negative and zero gaps, including high-limb inputs.

Both changes live in FixedMathSharp's existing exact geometry owners. Gravitas
retains its complete contact consumer and adds a preflight-checked `InteriorRim`
benchmark for both 3D cylinder/triangle and mixed circle-slab/triangle contacts.
Its exact depth is `13/256`; its stationary rim root wins, whereas the original
intrusion, oblique and touching fixtures select analytic contacts.

The source-derived resource ceilings remain below **512 KiB** explicit
simultaneous traversal scratch and **420 KiB** for mapped-endpoint sign work.
The shared chart buffer moves to the paired-chart caller without adding another
coefficient buffer; materialization's rebuilt chart is allocated only after
traversal chart frames return. These bounds exclude structs, JIT spills,
alignment and host frames. Existing 1 MiB thread-stack tests with 64 KiB of live,
dirty caller storage supplement the full-domain width proof for cylinder and
positive-core slab contacts.

## Matched Measurements

Captures use Windows 11, Intel i7-9700K, .NET 8.0.29 and SDK 10.0.302,
`UseLocalLsfStack=true` in both MSBuild and the inherited environment, serial
builds, and one timed workload at a time. Clean confirmation uses two launches,
five warmups and fifteen measured 250 ms iterations per launch. The command
requests affinity `3`; BenchmarkDotNet exports job affinity `11`, consistently
for the compared jobs. Launcher priority is the default. Report the exported
configuration rather than inferring a different affinity from the command.

Each capture records source hashes and compares the launcher with the actual
generated child's method IL hashes. The new interior fixture has its own
two-launch baseline with the committed math source restored, then the retained
source is restored and verified. Profiling is separate from clean timing.
All reported rows measure **0 B/op**.

Means +/- standard deviations, **ms per 64-pair batch**.

| Row | Fresh baseline | Retained | Change | 99.9% CI |
| --- | ---: | ---: | ---: | --- |
| `CheckMeshCylinderPairs(PairCount: 64)` | 5.551 +/- 0.152 | 5.189 +/- 0.070 | -6.52% | Separate |
| `CheckConcaveMeshCylinderPairs(PairCount: 64)` | 3.119 +/- 0.054 | 2.867 +/- 0.033 | -8.06% | Separate |

Means +/- standard deviations, **us per query**.

| Row | Fresh baseline | Retained | Change | 99.9% CI |
| --- | ---: | ---: | ---: | --- |
| `CylinderTriangle(Geometry: "CapFace")` | 51.292 +/- 1.403 | 50.046 +/- 0.971 | -2.43% | Overlap |
| `CircleSlabTriangle(Geometry: "CapFace")` | 19.937 +/- 0.389 | 18.541 +/- 0.291 | -7.00% | Separate |
| `CylinderTriangle(Geometry: "CapIntrusion")` | 511.371 +/- 10.692 | 410.881 +/- 2.887 | -19.65% | Separate |
| `CircleSlabTriangle(Geometry: "CapIntrusion")` | 509.925 +/- 12.557 | 416.981 +/- 6.083 | -18.23% | Separate |
| `CylinderTriangle(Geometry: "InteriorRim")` | 842.945 +/- 11.706 | 763.990 +/- 3.918 | -9.37% | Separate |
| `CircleSlabTriangle(Geometry: "InteriorRim")` | 846.766 +/- 14.363 | 762.610 +/- 8.970 | -9.94% | Separate |
| `CylinderTriangle(Geometry: "ObliqueRim")` | 638.263 +/- 13.267 | 540.570 +/- 14.583 | -15.31% | Separate |
| `CircleSlabTriangle(Geometry: "ObliqueRim")` | 637.178 +/- 12.397 | 535.979 +/- 7.278 | -15.88% | Separate |
| `CylinderTriangle(Geometry: "RimGap")` | 36.636 +/- 0.790 | 33.050 +/- 0.316 | -9.79% | Separate |
| `CircleSlabTriangle(Geometry: "RimGap")` | 37.171 +/- 0.583 | 33.362 +/- 0.329 | -10.25% | Separate |
| `CylinderTriangle(Geometry: "RimTouch")` | 583.403 +/- 11.491 | 481.280 +/- 6.526 | -17.50% | Separate |
| `CircleSlabTriangle(Geometry: "RimTouch")` | 582.526 +/- 14.206 | 485.178 +/- 5.827 | -16.71% | Separate |
| `CylinderTriangle(Geometry: "SideFace")` | 24.559 +/- 0.447 | 23.506 +/- 0.342 | -4.29% | Separate |
| `CircleSlabTriangle(Geometry: "SideFace")` | 24.719 +/- 0.422 | 23.327 +/- 0.163 | -5.63% | Separate |

Positive-core shared controls:

Means +/- standard deviations, **us per query**.

| Row | Fresh baseline | Retained | Change | 99.9% CI |
| --- | ---: | ---: | ---: | --- |
| `CapFace` | 46.941 +/- 0.687 | 43.762 +/- 0.491 | -6.77% | Separate |
| `StraightSideSeam` | 70.628 +/- 0.444 | 67.826 +/- 0.947 | -3.97% | Separate |
| `RoundedEndOddCore` | 162.109 +/- 0.798 | 160.855 +/- 2.853 | -0.77% | Overlap |
| `ObliqueRimOverlap` | 768.035 +/- 15.559 | 674.891 +/- 3.587 | -12.13% | Separate |
| `CertifiedRimGap` | 49.135 +/- 0.497 | 44.517 +/- 0.208 | -9.40% | Separate |
| `UnmaterializedScalarFace` | 73.873 +/- 1.083 | 68.901 +/- 0.274 | -6.73% | Separate |


The cylinder cap-face intervals overlap; its small mean change is not a
confirmed gain. The positive-core rounded-end control also overlaps. Other
retained comparison intervals separate. Shared controls exercise the distinct
positive-core triangle/capsule-slab path; they are direct math queries, unlike
the Gravitas contact/manifold consumers.

Raw evidence is under ignored `artifacts/grv-benchmark-018`: `baseline`,
`interior-baseline`, `retained`, `stadium-baseline`, `stadium-retained`, and their
source/launcher/child identities. `retained-comparison.json` and
`stadium-retained-comparison.json` retain the raw means, standard deviations and
99.9% confidence intervals. The one-launch `reciprocal-trial` and
`paired-invariant-trial` captures guided selection; the tables use the later
two-launch confirmations.

## Profile And Final Experiment

The original actual-workload profiles attribute about 35-38% inclusive CPU to
chart construction and 23-33% to analytic candidate construction. Root
acquisition itself contributes only about 0.4-3.8% in those fixtures. These samples
support reducing repeated exact construction rather than adding another
root-isolation strategy.

After the retained changes, the new interior-rim fixture attributes about 27%
inclusive CPU to final materialization, with about 21% in root-point output.
The original fixtures have analytic winners and no sampled rim-root
materializer work. Inclusive parent/child shares overlap and cannot be added;
they are diagnostic distributions, not timing gains. Attribution includes only
`Activity WorkloadActual` samples carrying triangle/cylinder work, excluding
pilot, warmup and JIT activities and removing CPU/native pseudo-markers before
exclusive-leaf attribution. The full distributions are retained in
`profile-actual-summary.json` and `retained-profile-actual-summary.json`.

The winning-parameter-root retention experiment is **reverted**. It reused
`FiniteAxisValueRoot.CopyTo` to avoid rebuilding and re-isolating the final
parameter, but added 6,773 bytes of persistent traversal scratch. A one-launch
trial appeared about 3% faster on the interior fixture; an unrelated side-face
control also improved, so that result required a fresh matched comparison.
Both two-launch controls below passed the launcher/child IL identity gate.

Means +/- standard deviations, **us per query**, all **0 B/op**.

| Row | Accepted two optimizations | Root retention experiment | Change |
| --- | ---: | ---: | ---: |
| `CylinderTriangle(Geometry: "CapIntrusion")` | 411.087 +/- 3.578 | 410.473 +/- 1.344 | -0.15% |
| `CircleSlabTriangle(Geometry: "CapIntrusion")` | 418.330 +/- 3.557 | 417.913 +/- 0.878 | -0.10% |
| `CylinderTriangle(Geometry: "InteriorRim")` | 766.912 +/- 8.220 | 776.151 +/- 21.000 | 1.20% |
| `CircleSlabTriangle(Geometry: "InteriorRim")` | 765.438 +/- 12.646 | 772.284 +/- 10.002 | 0.89% |
| `CylinderTriangle(Geometry: "SideFace")` | 23.278 +/- 0.407 | 23.091 +/- 0.249 | -0.80% |
| `CircleSlabTriangle(Geometry: "SideFace")` | 23.082 +/- 0.378 | 23.351 +/- 0.253 | 1.17% |

All six 99.9% confidence intervals overlap. Interior-rim means are slightly
higher; there is no repeatable gain to justify the added persistent storage.
The experimental job also reported short minimum iteration durations (about
60–62 ms) despite the requested 250 ms iteration time, reinforcing the need
to retain uncertainty. Final source is byte-verified against `before-retention.cs`;
`accepted-control`, `retained-root-control` and
`retained-root-control-comparison.json` preserve the decision evidence.



Two initial controlled captures were discarded when launcher/child IL verification
detected a stale incremental child binary from the previous experiment. Fresh
source-snapshot timestamps force recompilation; the local capture script also
removes only its validated BenchmarkDotNet generated-job directories before
capture. No timings from that mismatched run support a
retained change. Earlier confirmation captures passed the identity gate.

## Verification And Closure

Fresh serial gates pass with `UseLocalLsfStack=true`. Coverage below is exact
visited/coverable data, not rounded percentages; paired raw OpenCover totals
independently confirm complete owning coverage. Existing exclusions are
unchanged.

| Owner / configuration | Lines | Branches | Fully covered methods |
| --- | ---: | ---: | ---: |
| FixedMathSharp.Chronicler-Release | 85/85 | 12/12 | 18/18 |
| FixedMathSharp-Release | 53126/53126 | 12338/12338 | 3961/3961 |
| FixedMathSharp-ReleaseLean | 53219/53219 | 12338/12338 | 3957/3957 |
| FixedMathSharp.Chronicler-ReleaseLean | 85/85 | 12/12 | 18/18 |
| Gravitas-Release | 56272/56272 | 16302/16302 | 5413/5413 |
| Gravitas-ReleaseLean | 56270/56270 | 16302/16302 | 5412/5412 |

Raw OpenCover counts are recorded separately from rendered report counts:

| Owner / configuration | Sequence points | Branch points | Visited methods |
| --- | ---: | ---: | ---: |
| FixedMathSharp.Chronicler-Release | 85/85 | 12/12 | 18/18 |
| FixedMathSharp-Release | 53126/53126 | 12338/12338 | 3961/3961 |
| FixedMathSharp-ReleaseLean | 53219/53219 | 12338/12338 | 3957/3957 |
| FixedMathSharp.Chronicler-ReleaseLean | 85/85 | 12/12 | 18/18 |
| Gravitas-Release | 44542/44542 | 13296/13296 | 4603/4603 |
| Gravitas-ReleaseLean | 44540/44540 | 13296/13296 | 4602/4602 |

Full-suite results:

- FixedMathSharp Release: **49 passed**, `FixedMathSharp.Chronicler.Tests.dll`.
- FixedMathSharp Release: **4422 passed**, `FixedMathSharp.Tests.dll`.
- FixedMathSharp ReleaseLean: **49 passed**, `FixedMathSharp.Chronicler.Tests.dll`.
- FixedMathSharp ReleaseLean: **4401 passed**, `FixedMathSharp.Tests.dll`.
- FixedMathSharp focused Debug: **893 passed**, including existing resource/allocation guards.
- Gravitas Release: **4537 passed**, `Gravitas.Tests.dll`.
- Gravitas ReleaseLean: **4476 passed**, `Gravitas.Tests.dll`.
- Gravitas focused Debug: **25 passed**, including existing mesh/contact allocation guards.

Both `netstandard2.1` and `net8.0` source configurations build without warnings
or errors. Both DocFX builds pass with warnings as errors, local API links and
branding/resources checked. Independent math/resource and final code/docs reviews
report no actionable defects. The existing upstream complexity register records
current admission/traversal rationale and exact per-method coverage; it remains
evergreen. The cancelled Gravitas register was not created.

The fresh gate evidence is `artifacts/grv-benchmark-018/final-gates-20261006T205735103Z-e808b10473954cba8c5ae0fe6e8154cf`;
`result.json` and `coverage-summary.json` record completed checks. The earlier
interrupted gate invocation is not completion evidence.


Closure follows the same accepted criterion as the preceding signals: verified
local gains, complete deterministic geometry, published remaining costs and no
unsupported workload-rate claim. Ordinary faces remain in the tens of
microseconds; complete rim-heavy queries remain several hundred microseconds.
The 64-pair batches above are candidate/contact work, not complete simulation
steps. GRV-Benchmark-024 owns representative scene frequency and complete-step
capacity evidence before further geometry work is justified.

No approximation, sampled-axis fallback, tolerance, public API, host-specific
code or physics policy was introduced upstream. Exact feature admission,
strict tie order, correctly rounded depth and paired witnesses remain the
authority in both 3D and the mixed circle-slab path.

## Reproduction

From the Gravitas repository root, with the unreleased sibling sources present:

```powershell
$env:UseLocalLsfStack = 'true'
$env:DOTNET_PROCESSOR_COUNT = '2'
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -t:Rebuild -c Release -p:UseLocalLsfStack=true -m:1 -p:BuildInParallel=false -p:UseSharedCompilation=false -nr:false
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll --filter '*MeshCylinderContactBenchmarks*' '*CollisionDetectionBenchmarks.CheckMeshCylinderPairs*' '*CollisionDetectionBenchmarks.CheckConcaveMeshCylinderPairs*' --launchCount 2 --warmupCount 5 --iterationCount 15 --iterationTime 250 --affinity 3 --keepFiles --exporters json --artifacts artifacts/grv-benchmark-018/reproduction
```

Run profiling separately by selecting the cylinder intrusion/interior/oblique/
touch rows with `--profiler EP`; do not compare instrumented times with the
clean matrix. In FixedMathSharp, build its benchmark project in the same local
source mode and select `*TriangleCapsuleSlabContactBenchmarks*` for shared
positive-core controls. The disposable capture scripts add source and binary
identity checks around these commands.
