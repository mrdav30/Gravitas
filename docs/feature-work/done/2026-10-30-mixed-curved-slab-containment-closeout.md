# Mixed Curved Slab Containment

**Status:** GRV-Issue-148 resolved in source; coordinated package validation remains a release gate.
**Date:** 2026-10-30.

## Reproduction And Scope

Source baselines are Gravitas `fed2aaf` and FixedMathSharp `b64b582`, immediately
after #147, with SwiftCollections `22c0de8`, GridForge `89c69f6` and Chronicler
`c547e5c`. Validation uses Windows x64, .NET SDK 10.0.302/runtime 10.0.10 and
`UseLocalLsfStack=true`. Builds, tests, coverage and timings run serially.

In a Mixed context, initialize a bodyless unit-transform closed `Convex` mesh
cube `[-1,1]` using doubled `MeshTestFixtures.CreateConvexCube` vertices. At the
origin initialize either a circle with radius `0.25`, a capsule with radius
`0.25` and full height `0.5`, or a capsule with radius `0.25` and full height
`1`. Give each a mixed half-thickness of `0.25`.
`CollisionDetectionMixed.TryCollide(mesh, slab, out _)` returns false in all
three cases. The recorded height-0.5 capsule has zero straight-core length;
the height-1 case additionally verifies the real stadium shape.

The maintained initial regression contains those three failures, three passing
exterior controls at X=`1.125`, and three failing complete-host trajectories.
For host response, use a bodyless cube, a mass-one planar body starting at
X=`0.5`, frozen rotation, disabled sleep/manual grounding, 16 Hz, and zero
gravity, air density, damping and minimum speed. Six of nine cases fail before
the fix; the exterior controls pass. The initial contained classification
observations were also independently reproduced on the older `b4115b4` payload
during #147.

`AppendMeshSlabContacts` previously selected whole-hull contact only for polygon
prisms. Curved slabs have no polygon offsets and gather finite surface
triangles instead. A fully enclosed slab reaches no wall triangle. This is a
volume-versus-surface dispatch defect, not an approximation error in the
surface triangle solver.

## Ownership And Geometry

Gravitas selects filled-volume contact for interior and boundary centers of
closed `Convex` meshes before surface processing. Exterior centers and
open/`Concave` meshes retain their surface behavior. The shared dispatch also
serves mixed circle-query contacts and closed-hull continuous admission.
Supplier materials and compound namespaces follow the retained contact.
The runtime change extends existing owners without adding runtime files,
public APIs, persistent caches, configuration flags or dependencies.

FixedMathSharp extends the existing hull/cylinder reducer with the triangle
solver's planar core. A circle or zero-core capsule reduces to a finite
cylinder. A positive-core slab is the Minkowski sum of that cylinder and an
actual finite planar segment. Endpoint regions and the straight-section seam
are reduced together; two independent endpoint cylinders do not represent it.

- Every hull feature uses one exact post-core multiplier and hull-wide value
  scale. Whole-hull support comparisons transform all vertices into the same
  scaled and endpoint-translated chart. No endpoint rounds before admission.
- Both remaining triangle-only minimum certificates are disabled for restricted
  hull domains. Analytic and rim-root candidates retain exact global ranking.
  Unsupported triangle directions are rejected before constructing their
  expensive candidate values.
- Finite seam construction can decline a supporting triangle that lacks the
  matching point. The reducer then tries another support triangle, preserving
  finite side intervals, cap disks, core intervals and exact endpoint terms.
- Root materialization reconstructs the selected endpoint chart and uses the
  actual core axis. A discriminating rigid/affine regression fails when both
  mistakes from the reviewed prototype are restored: the anchor residual is
  approximately `(0,-0.1243,0.1175)` instead of zero. The fixed implementation
  passes finite witnesses and independent whole-shape clearance checks.

The affine seam's compact normal is below 344 bits, tangent below 380, blended
weights below 2,515 and materialization coefficients below 4,000. Rigid fields
retain forty words; affine seam fields use eighty. The shared quadratic slice
parses each input's actual stride and sizes workspace from its output field.
The cap helper's rational temporary and normal inputs retain their original
forty-word layout. Rational cap-radius squares stay below 3,440 bits; quadratic
sign comparisons retain dynamically sized complete squared products.

The seed must also preserve the seam's normal-width contract. A hull face can
carry an unreduced direction much wider than the native seam inventory. The
admitted cube half extents `(0.25,2048,2048)`, uniform owner/part scales raw
`Q+1`/`Q+3`, and equal source/cylinder quaternion `(0,q,0,2q)` with
`q=1920767767`, with `Q=2^32`, produce a 345-bit face component. Core-perpendicular seed faces
use a parallel edge/core cross before selection, keeping the retained normal
within the existing rigid/affine seam bounds. A focused regression checks the
wide input, retained direction, exact depth and finite witness. This closes a
proof gap in the new seed; it is not evidence of a released overflow defect.

## Exact Early Certification

Let `H` be the hull, `S` the symmetric planar core segment, and `C` the finite
cylinder. The slab exit is the boundary of `H + S + C`. The polytope `H + S`
has original hull-face facets and extrusion facets generated by hull edges
crossed with the core direction. Its support is exactly
`h_H(n) + |coreOffset dot n|`.

The certificate enumerates all original faces and all triangle-edge/core
crosses, admitting only whole-hull supporting planes and trying the opposite
orientation at zero plane constants. Triangle diagonals are redundant or fail
support admission; every true hull edge is present. These planes bound a
centered inscribed ball of radius `d`. The cylinder contains a ball of radius
`m = min(radius, exact half-height)`, so every exit is at least `d + m`.
A retained exact contact attaining that bound is globally minimal. Forty-word
certificate fields suffice: conservative plane constants are below 861 bits
and squared/radical terms below 1,725 bits.

A cheap positive-core seed tests hull faces and the first triangle's existing
seam directions. The original ball certificate can finish a simple aligned
case immediately; the extrusion certificate checks the remaining seed. If
certification declines, the seed is discarded before the complete feature
search, preserving its tie order. Zero-core inputs retain #147's path and ties.
Certified positive-core ties use the deterministic seed order.

## Mixed Mobility And Query Boundary

Geometric contact does not grant a planar body vertical mobility. At the exact
center of the symmetric cube, the existing minimum order can select Up. With a
bodyless mesh and only a dynamic 2D participant, that row has zero response
mobility and the body remains enclosed while contact is reported. This change
preserves that mixed response policy. Separate maintained tests verify planar
recovery where the shortest exit is uniquely planar, and vertical recovery
where the 3D mesh body can move.

The explicit mixed circle sweep reports an initial hit at distance zero for
contained circles, including scaled hulls and multiple travel directions.
Continuous admission recognizes all three slab variants. Completed continuous
host steps allow outward escape and stop inward travel, retaining planar Y
and the existing zero/one time-of-impact iteration behavior. These controls do
not establish a historical tunneling defect or add a public capsule sweep API.

## Validation

| Project/configuration | Tests | Raw sequence points | Raw branch points | Raw methods |
| --- | ---: | ---: | ---: | ---: |
| FixedMathSharp Release core | 7,257 pass | 68,344/68,344 | 22,564/22,564 | 5,322/5,322 |
| FixedMathSharp ReleaseLean core | 7,236 pass | 68,437/68,437 | 22,564/22,564 | 5,318/5,318 |
| Gravitas Release | 8,235 pass, plus 6 CCD cases | 53,398/53,398 | 21,620/21,620 | 5,594/5,594 |
| Gravitas ReleaseLean | 8,164 pass, plus 6 CCD cases | 53,396/53,396 | 21,620/21,620 | 5,593/5,593 |

The math Chronicler suite also passes 49 tests in both configurations. The six
additional completed-host CCD cases were added after the full Gravitas runs
and pass separately in Release and Lean; production code and coverage points
are unchanged. Every final raw sequence, branch and method point is visited.

Focused evidence includes the failing original host regressions, all existing capsule-slab and
hull-cylinder cases, exact depths against the independent planar
capsule/polygon solver, transformed whole-shape clearance through independent
finite-section intersection, finite hull/slab anchors, odd core lengths,
wide authored vertices under tiny affine scale, and one-MiB worker-stack tests
with live dirty 64 KiB caller buffers. Warm reset-plus-complete-step tests verify
zero allocations and repeated authoritative replay hashes.

Canonical commands are `dotnet test FixedMathSharp.slnx` in the sibling repo
and `dotnet test Gravitas.slnx`, with `-c Release` or `-c ReleaseLean`,
`-p:UseLocalLsfStack=true` and `-m:1`. Coverage uses each core project's
`coverlet.runsettings` with `--collect:"XPlat Code Coverage"`; the math capture
additionally enables OpenCover to inspect unrounded method/branch totals.

## Performance

Each maintained full-step benchmark operation resets
one planar mover's position, then executes `Simulate` and `LateSimulate` against
one bodyless closed cube at 16 Hz, with the reproduction settings above.
It measures repeated correcting steps, not cached stationary contact.

The final standalone comparison uses the same maintained benchmark assembly,
two frozen runtime pairs, two balanced rounds (AB then BA), nine batches of
256 operations per process, and two seconds of warmup. All 80 processes across
20 workloads completed: 720 measured batches and 184,320 operations. Loaded
assembly hashes match the intended payloads; the benchmark hash is identical.
Both current-thread and precise process-wide managed allocation counters report
zero in every measured process. All fourteen unchanged controls retain identical
physical outputs; the six contained fixtures intentionally gain response.

| Full correcting step | Original missed response (µs) | Corrected (µs) |
| --- | ---: | ---: |
| Circle | 9.18 | 1,020.71 |
| Zero-core capsule | 10.48 | 1,015.23 |
| Aligned positive-core capsule | 11.61 | 354.37 |
| Yawed positive-core capsule | 11.41 | 1,225.05 |
| Affine aligned capsule | 34.38 | 553.84 |
| Affine yawed capsule | 34.54 | 2,209.55 |

Values are medians of eighteen batch means per payload. The affine cases use
authored vertices at ±`(1 + 1 raw)` and owner scale `1.5 + 1 raw`; setup verifies
that the exact source chart is retained. This is one contained pair, not a
multi-body frame budget or gameplay percentile. Original contained ratios compare
unequal work because the old payload skips the contact and response entirely.

| Existing control | Original (µs) | Corrected (µs) | Change |
| --- | ---: | ---: | ---: |
| Native cylinder cap | 39.48 | 39.65 | +0.42% |
| Native cylinder rim | 730.74 | 734.92 | +0.57% |
| Native cylinder side | 20.42 | 20.98 | +2.73% |
| Mixed circle rim | 784.01 | 784.19 | +0.02% |
| Cone base | 97.31 | 97.02 | −0.29% |
| Cone oblique rim | 1,304.53 | 1,304.10 | −0.03% |
| #147 contained cylinder step | 344.68 | 346.48 | +0.52% |
| #147 exterior cylinder step | 1,235.22 | 1,231.34 | −0.31% |
| Contained sphere step | 53.14 | 53.27 | +0.25% |
| Mixed circle exterior step | 1,229.75 | 1,222.49 | −0.59% |
| Mixed capsule exterior step | 2,206.93 | 2,214.56 | +0.35% |
| Mixed capsule miss step | 6.39 | 6.43 | +0.64% |
| Stadium surface interior | 188.02 | 187.21 | −0.43% |
| Stadium surface tilted | 2,464.52 | 2,484.38 | +0.81% |

The side control's initial +0.56 µs warranted a bounded follow-up. A separate
ABBA run measured 20.77 µs original versus 20.58 µs corrected (−0.9%); an
identical-original-payload ABBA calibration measured 20.68 versus 20.65 µs.
All eight processes have matching expected hashes, unchanged outputs and zero
on both allocation counters. The initial side increase is not reproducible;
these measurements establish neither a persistent regression nor zero overhead.

A sampled-thread-time profile of the first working yawed implementation
attributed about 93% of hull time to wide scratch clearing. Shared per-call
scratch reuse did not improve it and was removed. Support rejection before
candidate construction, exact early certification and a compact seed reduce
the work. A final matched ABBA comparison against the preserved working
pre-filter prototype measures 4,810.16 versus 1,218.28 µs: a 74.7% reduction,
with identical authoritative positions/checksums and zero on both allocation
counters. This gain compares two working implementations, not the old missed
contact. The final identity-transform shortcut also avoids extra chart work
on #147's zero-core path.

BenchmarkDotNet's in-process Short job completes all nine maintained cases,
with eight actual iterations each, five warmups and one launch (96.79 seconds
total). Means in µs are circle 1,086.33, zero-core 1,071.10, aligned capsule
373.52, yawed capsule 1,290.79, affine aligned 547.14, affine yawed 2,281.90,
circle exterior 1,318.91, capsule exterior 2,353.25 and miss 6.46. The in-process
toolchain follows #147's existing local benchmark-discovery limitation; no new
out-of-process failure is claimed here.

The multi-case MemoryDiagnoser reports 2–20 B/op on seven rows and zero on two.
Three bounded isolated `CapsuleExterior` jobs each complete eight actual
iterations and report zero: current/default GC, original/default GC, and
current with process-local `DOTNET_gcConcurrent=0`. Their means are 2.209,
2.203 and 2.204 ms. This does not reproduce the earlier 20 B/op reading and
does not establish its cause. Preserve the discrepancy; the zero-allocation
claim applies to the standalone counters and host-step regression tests, not
to every BenchmarkDotNet diagnostic in the multi-case capture.

Supplemental captures live under `artifacts/grv-148`: `accepted-controls.jsonl`,
`native-side-calibration.jsonl`, `optimization-accepted.jsonl`, `bdn-accepted`,
`bdn-memory-*`, `accepted-validation.json` and the matching validation logs.
Final runtime SHA-256 prefixes are Gravitas `E7678123785C` and FixedMathSharp
`E46352F252C1`; the common benchmark is `51A1D697D8DA` and the working
pre-filter math prototype is `01C4EBCD2D4A`. The maintained reproduction,
results and reopening criteria do not depend on preserving ignored artifacts.
Reopen on a repeatable lost contained contact, nonfinite/mismatched witness,
changed unchanged-control output, new warm allocation, or persistent matched
timing regression; retain the geometry, payload identities and complete steps.

### Bounded Follow-up Optimization Check

A follow-up profile of the final circle fixture still attributes about 93% of
sampled hull time to wide scratch clearing. Two small, exact-preserving changes
were tested together: allocate root-search banks only when root search is needed,
and hoist the unchanged ball-radius/scale squares out of both certificate loops.
Independent review found no alias, initialization, resource or tie-order issue.

Sixteen fresh processes compare the existing correct payload and the prototype
in balanced ABBA order, using the same benchmark and nine batches of 256
operations per process. Loaded runtime/benchmark hashes and authoritative
positions/checksums agree; the math hashes distinguish the intended variants.
Both allocation counters remain zero in every process.

| Correcting step | Existing (µs) | Prototype (µs) | Change |
| --- | ---: | ---: | ---: |
| Circle | 1,034.66 | 1,016.61 | −1.75% |
| Aligned capsule | 359.17 | 359.38 | +0.06% |
| Yawed capsule | 1,225.37 | 1,238.42 | +1.06% |
| Affine yawed capsule | 2,253.47 | 2,229.35 | −1.07% |

These mixed, small differences do not establish a useful overall improvement
or resolve the millisecond-scale cost. Both prototype changes were removed;
the validated production source and the user's staged index were preserved.
No additional algorithm, arithmetic-width reduction, uninitialized-stack option
or cross-frame cache was introduced. A larger search/arithmetic optimization
needs its own measured design and correctness evidence. The approximately
0.35 ms #147 centered-cylinder fixture is not a matched control for these
off-center movers: its first analytic axis can already attain the exact minimum
certificate. The off-center fixture must find a smaller planar exit.

The complete comparison is `artifacts/grv-148/followup-balanced.jsonl`; its
profile is `followup-circle.nettrace`/`followup-circle.speedscope.json`. A prior
partial attempt in `followup-candidate.jsonl` is excluded because a directly
built math DLL had assembly version `0.0.0.0` instead of the local stack's
`7.1.0.0`, so the first prototype process failed before producing measurements.
The balanced comparison rebuilt through the owning Gravitas project and verifies
the loaded identities. No correctness or timing conclusion uses that failed
process.

## Release Boundary

This changes a FixedMathSharp internal ABI consumed by Gravitas. Release
FixedMathSharp first, then validate Gravitas against that exact released package.
Local-stack results do not replace package validation or establish other OS/CPU
execution evidence. No dependency pins, public APIs or package versions change.
