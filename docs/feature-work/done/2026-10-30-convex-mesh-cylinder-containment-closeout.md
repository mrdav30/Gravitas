# Closed Convex Mesh Cylinder Containment

**Status:** Complete in coordinated source for GRV-Issue-147; package validation remains a release gate.
**Date:** 2026-10-30.

## Reproduction And Root Cause

The ordinary mesh/cylinder dispatch always gathered nearby finite surface
triangles. A cylinder enclosed by a filled convex hull reached no wall triangle
and received no discrete response. This was unrelated to static surfaces.

The maintained regression uses a bodyless `Convex` cube with bounds `[-1,1]`,
made by doubling `MeshTestFixtures.CreateConvexCube` vertices, and a mass-one
cylinder at the origin with radius `0.25`, full height `0.5`, frozen rotation,
disabled sleep and manual grounding. At 16 Hz, with zero gravity, air density,
damping and minimum speed, one complete `Simulate`/`LateSimulate` step left its
position at raw `(0,0,0)`. The sphere control, exterior X=`1.125` controls and
intentional `Concave` shell controls behaved as expected. The test builder's
grid is `[-16,16]`; the original report's `[-8,8]` grid gives the same local
geometry and failure. The six-case regression failed only for the contained
cylinder on `b4115b4` before the fix.

A first surface-first fallback passed that single-step check but failed a
maintained repeated-step check: after its initial correction, the cylinder
reached a two-sided wall triangle whose normal pushed it inward. Volume contact
must have precedence through the inside-to-boundary transition.

## Implementation And Invariants

Gravitas uses the existing exact convex-hull half-space owner to classify the
cylinder center. Interior and boundary centers in closed `Convex` meshes use
whole-hull finite-cylinder contact. Exterior centers preserve existing finite
surface manifolds; open and `Concave` meshes preserve their surface meaning.
The dimensional collider supplies committed physical height and scaled radius.

FixedMathSharp owns the added policy-neutral hull/cylinder geometry. It reuses
the complete triangle/cylinder support-fan features and rim root algebra:

- A single cylinder chart, common coordinate reduction and hull-wide value
  scale keep analytic, analytic/root and root/root comparisons exact across
  triangles. Per-triangle rounded depths cannot decide the winner.
- Candidate support is checked against every hull vertex before ranking or
  rejecting overlap. A hull feature appears in its incident surface triangles;
  boundaries introduced by restricting a triangle fan occur in neighboring
  face, edge or vertex fans. Triangle-only minimum certificates do not apply
  after restricting the support domain.
- The final minimum is materialized once. A supporting triangle may not contain
  the matched witness, so finite side intervals, cap disks, rim barycentric
  weights and matching residuals are admitted before final coordinate rounding.
  Edge/rim roots retain exact segment admission. Larger support features are
  searched before their subsets, with deterministic topology order for ties.
- An exact early certificate uses the hull's centered inscribed-ball bound and
  the cylinder's ball of radius `min(radius, exact half-height)`. Their sum is a
  lower bound on every exit. A retained candidate attaining that bound is
  globally minimal, so the remaining features can be skipped. Every face's
  support orientation and nonnegative plane constant are checked; declined
  certificates retain the complete fan. No rounded distance makes this choice.
- Affine witnesses use the bounded cylinder chart, preserving authored scale
  and placement without materializing a distant source point first. Existing
  rigid/affine coordinate bounds and polynomial banks are retained; native
  prism algorithms and their narrower assumptions are unchanged.

The skew-hull regression uses `|x|<=1`, `|3x+4y|<=5`, and `|z|<=4`, with a
thin cylinder approximately along `(1,-2,0)`. A face-only exit is about `1.9`
units and can project outside the finite face; the complete edge/side exit is
below `1.2`. This prevents replacing the missing contact with a convenient
face-only correction or a rounded capsule proxy.

Independent review caught an affine cap-disk admission width issue in the first
implementation. Its barycentric coordinate numerators fit forty words, but
their squares can exceed that bank. Squared scratch now uses complete active
input widths plus carry. The adversarial independent-yaw/odd-scale regression
proves its squared values exceed 2,560 bits and compares disk admission with a
BigInteger oracle. Rim residual admission retains separate proven rigid/affine
banks. Native-domain parameters are passed by readonly reference rather than
copying the large hull frame into each feature selection.

The runtime change adds 484 net lines across both libraries: two focused math
owner files and extensions to existing analytic, root and finite-witness owners,
plus the Gravitas dispatch. There is no public API, persistent cache or new
dependency. Independent review covered the connected geometry and native callers,
width bounds, finite witnesses, performance evidence and deletion opportunities.

## Dimensional And Query Audit

The contained-cylinder sweep reports its existing initial overlap at distance
zero. #147 does not change sweep admission or establish a historical tunneling
defect.

Mixed circle and capsule slabs both independently reproduced missing closed-hull
containment on frozen `b4115b4` assemblies. The mixed owner selects whole-volume
contact only for polygon prisms; curved slabs have no nearby wall triangles.
This is tracked as GRV-Issue-148 with complete inputs in the issue tracker.
The finite-cylinder owner can support a circle slab, but a capsule slab also
needs its finite planar core. Neither mixed case is claimed fixed by #147.

## Validation

The local source baseline is Gravitas `b4115b4`, FixedMathSharp `03f6cbf`,
SwiftCollections `22c0de8`, GridForge `89c69f6` and Chronicler `c547e5c`.
The candidate is that baseline plus the coordinated working-tree change.
Validation uses Windows x64, .NET SDK 10.0.302/runtime 10.0.10, and
`UseLocalLsfStack=true` throughout. Builds, tests, coverage and measurements
run serially. Supplemental captures are under `artifacts/grv-147`; the
maintained tests and reproduction above do not depend on that directory.

| Project/configuration | Tests | Raw sequence points | Raw branch points | Raw methods |
| --- | ---: | ---: | ---: | ---: |
| FixedMathSharp Release core | 7,233 pass | 68,212/68,212 | 22,470/22,470 | 5,314/5,314 |
| FixedMathSharp ReleaseLean core | 7,214 pass | 68,305/68,305 | 22,470/22,470 | 5,310/5,310 |
| Gravitas Release | 8,200 pass | 53,390/53,390 | 21,610/21,610 | 5,594/5,594 |
| Gravitas ReleaseLean | 8,129 pass | 53,388/53,388 | 21,610/21,610 | 5,593/5,593 |

The FixedMathSharp Chronicler suite also passes 49 tests in each configuration. Two added
whole-hull resource tests pass separately in Release: both rigid and affine
uncertified rim-root cases run twice on a 1 MiB worker stack with a live, dirty
64 KiB caller buffer. Exact contact outputs match under both dirty patterns,
and caller contents remain intact. The new resource tests do not change
runtime coverage points and are included in the full Lean run.

The maintained Gravitas tests include monotonic repeated correction, scaled
mesh/cylinder witnesses, reversed collider input, exact boundary centers,
touch/one-raw separation, existing initial sweep overlap, repeated local replay
hashes and a zero-allocation warm complete-host-step guard. Math regressions
also compare exact depths with the independent OBB/cylinder owner, verify
whole-shape clearance around the selected exit, exercise zero radius and
clamped full-domain depth, and cover independently placed/scaled authored frames.

Canonical correctness commands are `dotnet test FixedMathSharp.slnx` in the
sibling repository and `dotnet test Gravitas.slnx`, each with
`-c Release` or `-c ReleaseLean`, `-p:UseLocalLsfStack=true`, and `-m:1`.
Coverage adds `--collect:"XPlat Code Coverage"` and the corresponding core
test project's `coverlet.runsettings`; FixedMathSharp's capture additionally
enables OpenCover alongside Cobertura to inspect raw method/branch counts.

## Performance

Each host-step operation resets the mover to its fixture position, then runs
`Simulate` and `LateSimulate`: one dynamic cylinder/sphere and one bodyless
closed convex cube, at 16 Hz with the reproduction settings above. These are
repeated correcting steps, not a stationary cached-contact throughput claim.

Initial warm measurements of the complete feature solver were about 2.1 ms for
the contained cube. A `dotnet-sampled-thread-time` trace attributed about 88%
of sampled hull time to clearing wide scratch during repeated analytic
evaluation. The exact global certificate and readonly domain passing reduce
that common case to about 0.35 ms, preserving the exact corrected position.
Uncertified oblique and rim cases retain the complete solver.

A final initial/final/final/initial control uses the same maintained benchmark
and standalone driver against preserved pre-optimization and final runtime
payloads. Its 18 batches per payload measure 2,088.33 µs versus 342.28 µs
(83.6% less time), with identical corrected outputs and zero allocations under
both counters. The pre-optimization math hash begins `E3ECC273658F`; loaded
runtime/math hashes were verified. This compares two working implementations,
separately from the broken-baseline comparison below.

The final matched standalone controls execute the maintained benchmark methods
against frozen original and final runtime assemblies. Two rounds reverse order
(baseline/candidate, then candidate/baseline), with a two-second warmup per
process and nine batches of 256 operations. All 40 processes completed: 360
measured batches, 92,160 operations. Both current-thread allocation and precise
process-total allocation deltas are zero in every measured batch. Outputs agree
for every control; the intended containment change moves the cylinder to raw
`(0,4260607622,0)` rather than leaving it at zero.

| Workload | Original median, µs | Final median, µs | Change |
| --- | ---: | ---: | ---: |
| Full step, contained cylinder | 12.72 | 345.66 | Corrected behavior; unequal work |
| Full step, exterior cylinder | 1,244.72 | 1,262.44 | +1.42% |
| Full step, cylinder corner miss | 57.95 | 58.50 | +0.95% |
| Full step, contained sphere | 52.59 | 52.77 | +0.36% |
| Native triangle/cylinder cap face | 40.24 | 40.38 | +0.35% |
| Native triangle/cylinder side face | 21.01 | 21.34 | +1.58% |
| Native triangle/cylinder interior rim | 749.28 | 753.22 | +0.53% |
| Mixed triangle/circle slab interior rim | 799.82 | 796.51 | -0.41% |
| Native triangle/cone base face | 97.84 | 98.80 | +0.99% |
| Native triangle/cone oblique rim | 1,327.57 | 1,321.27 | -0.47% |

Medians combine the 18 batch samples per payload/workload. The original
contained-cylinder result skips the missing response and can reuse a no-contact
cache; its ratio is not an equivalent-work regression. The final contained case
uses about 0.55% of a 62.5 ms step budget for this single pair, without predicting
many-pair scaling. Small control differences vary between captures: the earlier
40-process control set measured -0.8% to +0.8%, before the final combined
center-admission entry. Retain both captures; neither proves universal zero
overhead or a stable regression from these small differences.

Final loaded SHA-256 prefixes are Gravitas `13E7C9D54E77`, FixedMathSharp
`29F07474B41B`; original payloads are `665A53BD29C7` and `9484E8AD78C4`.
Both use benchmark `70EF87718B76`. Every process reports its actual loaded
paths and full hashes, verified against the selected files.

BenchmarkDotNet 0.15.8 also completed all four in-process rows, with five
warmups and eight actual timing iterations each: contained cylinder 333.29 µs,
exterior cylinder 1,271.94 µs, corner miss 58.03 µs and contained sphere
51.78 µs. Its first exterior memory diagnostic reported 8 B/op across 512
operations. An isolated candidate rerun reports 0 B/op (1,247 µs), as does an
isolated original-payload exterior run. Together with the precise standalone
counters this did not reproduce a warm allocation; the original reading remains
unexplained and is not discarded or attributed to a proven harness cause.
The other three original rows report 0 B/op. Default BDN outlier removal is
retained, and every actual iteration and successful report was inspected.

The out-of-process launcher was attempted from the repository and project
directories. Existing ignored archived projects with the same project name
prevent generated-project discovery; both attempts execute zero rows and are
excluded. The in-process results and separately hash-verified controls are the
available timing evidence, not a repaired out-of-process launcher.

Reproduce the maintained workload after a local-stack Release build with
`dotnet tests/Gravitas.Benchmarks/bin/Release/net10.0/Gravitas.Benchmarks.dll
convex-mesh-cylinder-containment --job Short --iterationCount 8 --warmupCount 5
--launchCount 1 --inProcess`. Supplemental captures include
`verified-controls.jsonl`, `verified-summary.json`, `final-controls.jsonl`,
`optimization-verified.jsonl`, `containment-initial.nettrace`, `bdn-final-in`,
`bdn-verified-exterior` and
`bdn-baseline-exterior` under `artifacts/grv-147`. The temporary standalone
driver invokes the maintained methods, without an alternate physics algorithm.

## Release Boundary

This is a coordinated FixedMathSharp internal ABI change. Release FixedMathSharp
first, then rerun package-based Gravitas validation against that exact package
before releasing Gravitas. Package restore currently fails because the configured
feeds do not provide `Chronicler.Diagnostics`; local-stack evidence does not
substitute for release-package validation.
