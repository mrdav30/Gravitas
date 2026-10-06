# Capsule/Stadium-Slab Root Isolation Refinement

**Status:** Complete; GRV-Benchmark-021 closed 2026-10-05.  
**Archived:** 2026-10-06. Earlier phase assessments below are historical.

This records the next GRV-Benchmark-021 investigation after the
[certified-sign refinement](2026-10-05-capsule-slab-sign-refinement.md).
The starting revisions are Gravitas `62c9be8` and FixedMathSharp `9b1f173`.
The complete whole-shape query remains the correctness baseline.

## Retained Changes

Sturm variation counting borrows the existing returning evaluation scratch for
the existing certified point-sign evaluator. A certified nonzero sign replaces
homogeneous integer evaluation; an uncertain result still invokes exact
evaluation and the first nonzero derivative's right-limit sign. Constant rows,
zero and one retain their exact routes. The extracted arithmetic core does not
compute unused prediction hints; the existing prediction wrapper still supplies
the same normalized magnitude and exponent.

The trial's precision cap fits its two disjoint borrowed buffers. If `E` is
evaluation capacity, `Q=(shift+64)/64`, `L=ceilLog2(degree+1)` and
`A=floor((E-Q)/2)`, the cap is `64*A-L-64`. Consequently
`words=floor((precision+L+127)/64)<=A` and `2*words+Q<=E`.
This caps work, never the sign acceptance threshold. The existing 128-bit point
guard remains; no tolerance or precision setting is exposed.

For ordinary isolation, retained chain slots consume at most
`ceil(221*(B+64)/64)+45` words, leaving at least
`floor(659*(B+64)/64)+467` evaluation words. The existing isolation bound is
`shift<=16*(B+11)+89`, so `E-Q` is well above the four words needed for a
positive cap. Refinement separately reserves at least
`3*(B+64)+2*n*(targetShift+1)+318` evaluation bits, plus the existing rounding
and 512-word margin, with `targetShift>=shift`. Contact rounding comparisons
use at most 462 denominator bits. Repeated-family mapped comparisons can use
a finer source cell: their degree-four repeated factor and degree-eight input
separation bounds conservatively give `shift<=4*7424+16*B+265`, still below
the same arena capacity. No new arena or retained root state is added.

Batch isolation shares dyadic subdivision counts instead of restarting each
ordinal from `(0,1]`. A pending interval occupies the caller's output slot of
its first ordinal, with two eight-element integer arrays for its count and
lower variations. A genuine split saves the right sibling in its first ordinal
slot and continues left. Completed roots form an ascending prefix, so the next
ordinal's pending state is initialized before use. All-left and all-right
subdivision needs no additional pending storage, regardless of cluster depth.
The explicit scratch increase is **64 bytes**.

The left cell includes the midpoint and the right excludes it, following the
existing right-limit convention. A dyadic root may be recognized later than
in independent ordinal isolation; removing its numerator's powers of two
restores the same minimal denominator. Nonrational roots reach the same first
singleton cell and use the existing exact endpoint-cleanup/refinement tail.
At compact depth `M`, successful old isolation implies each rational root is a
distinct grid point and each nonrational root has an endpoint-free singleton
cell. Two roots therefore cannot share one depth-M cell. Shared traversal
preserves compact success/failure, authoritative count, global repetition,
root ordinals and the cleared rational mask on failure.

Stadium reciprocal charts share one Sturm construction when the defining
polynomial's constant and nominal leading coefficients are nonzero. For the
already oriented chain rows `F_i`, the transformed rows are
`R_i(s)=(-1)^i*s^degree(F_i)*F_i(1/s)` on positive `s`. Neighboring rows keep
opposite signs at an internal row zero. At a defining root, the first neighbor
agrees with `s*G'`, up to the original positive derivative scale. Dividing
conceptually by the common reversed gcd gives the same property for repeated
roots. Actual row reversal followed by leading-zero trimming therefore gives
a valid generalized Sturm sequence even with abnormal degree drops.

Both views compute their own endpoint variations, counts and ordinals; original
right-limit signs cannot be copied because reciprocity swaps sides. With
nonzero endpoint coefficients, reversal preserves global repetition. Constants,
zero constant terms and nominal leading zeros use the existing independent
owners; this preserves zero-factor metadata when factors map to infinity or
appear at zero. Either compact failure still permits the other view's complete
count and independent outcome. Both Bernstein trials retain their successful
cells, and a chain is built only if at least one trial fails.

Every original signed chart remains in its previous order. The chart-swap
identity is `P_bar=sP(1/s)`, `Q_bar=s^2Q(1/s)`, `M_bar=s^2M(1/s)`, hence
`K_bar=-sK(1/s)`, `L_bar=-s^2L(1/s)` and `W_bar=s^4W(1/s)`. Common
power-of-two normalization preserves exact quartic reversal. The first chart
copies its winning parameter/value before its full data buffer is rebuilt for
the second chart. A Debug assertion checks the rebuilt stationary polynomial.
The paired producer explicitly returns before consumer scratch is allocated.

The extra reciprocal polynomial and batch buffers consume **2,149 bytes**,
raising the prior explicit outer-buffer total from 44,601 to **46,750 bytes**,
inside the existing 48 KiB reserve. Limb-by-limb row reversal adds no array.
Conservatively charging these buffers and the 64-byte subdivision frame to the
previous stadium budgets gives **464,453 / 420,898 bytes**, including the live
caller/control reserves, below 512 KiB. Focused dirty-stack/1 MiB tests
supplement the width and lifetime proof.

A strictly winning negative stationary rim gap can now terminate traversal
when its radial support projects within the finite capsule core. Let
`e=CapsuleAxis`, `B=CapsuleHalf`, `c=CapOffset`, `a=e.c`, `H=e.B>0`,
`n=first+t*second`, `Q=R²|n_h|²` and `T=R²(e.n_h)`. The support point
`p=c+R*n_h/|n_h|` has `e.p=a+T/sqrt(Q)`. Bounds `-H<=e.p<=H` make
`q=p-(e.p/H)*B` feasible in the complete core Minkowski difference.
Admitted stationarity gives `q=RawScale*g*n_unit`. For negative `g`, support
admission gives `n_unit.x<=RawScale*g<0` throughout that convex difference,
hence `q.(x-q)>=0`. This proves its unique closest point and the global
minimum signed support gap. Exact finite endpoints remain feasible. Earlier
equal candidates still win before certification; positive and zero gaps retain
their full traversal.

The shared circular algebra tests `T+(a+H)*sqrt(Q)>=0` and
`T+(a-H)*sqrt(Q)<=0`. Zero or equal-sign operands decide directly; opposite
signs use `sign(T)*sign(T²-(a±H)²Q)`, including equality. It uses the original
parameter `Q`, which is unaffected by stationary/value normalization. Under
the shared bounds, `a,H<2^432`, `T<2^861` and comparison coefficients below
`2^1723` fit forty words. The helper owns geometry feasibility, independent of
stationarity; the stadium owner supplies the global-minimum policy.

After the winning value is copied, its old mapped cell supplies the helper's
seven forty-word scratch slots. Parameter refinement is then copied before
the Boolean result exits to the existing materializer. This adds no retained
state or wide scratch array and preserves the prior workspace reserve. The
paired chart admission also drops the redundant `second.Y` rejection:
its sole caller supplies the original `GetBasis` order, where that coordinate
is always zero.

For the new degree-two support query, stadium parameter height at most 1,900
bits and query height at most 1,723 give `nonzeroBits<=8981` and
`targetShift<=10714`. Retained rows plus exact evaluation need at most
168,526 bits, inside the original 1,728,320-bit allowance; its returning arena
remains 220,136 bytes. An already finer parameter skips refinement before
allocating that arena. The five sign bytes and scalar locals fit the existing
control reserve; the reviewed wide-buffer peaks above do not increase.

The shared evaluator and batch changes also serve the existing 3D box/cylinder,
cylinder pair, triangle/cylinder and triangle/cone consumers. Reciprocal pairing
is used only by the measured stadium owner. No new public API, friendship,
physics policy, speculative root cache or alternate admission algorithm is added.

## Experiments And Matched Measurements

Captures use `UseLocalLsfStack=true` in MSBuild and the environment, Windows 11,
an i7-9700K, .NET 8.0.29 and SDK 10.0.302. One BelowNormal heavy workload runs
at a time with `DOTNET_PROCESSOR_COUNT=2` and serial builds. The requested
CLI option is `--affinity 3`; exported job metadata displays `Affinity=11` in
both matched captures. These are recorded separately without interpreting the
display as a different requested mask.
Each row has two launches, five warmups and fifteen measured 512-query
iterations per launch. Values below are microseconds per dispatched query;
error is the 99.9% confidence half-width. Every row reports **0 B/op**.

| Source / capture | Analytic winner | Original rim | Irrational rim | Positive rim |
| --- | ---: | ---: | ---: | ---: |
| Committed / `phase3-baseline` | 1424.8 ± 14.26 | 717.6 ± 6.89 | 906.4 ± 9.06 | 861.0 ± 11.43 |
| Whole-quadrant chord / rejected | 1354.7 ± 13.92 | 1367.4 ± 10.31 | 795.3 ± 6.98 | 1535.4 ± 15.28 |
| Borrowed point certificate | 1228.8 ± 15.95 | 705.1 ± 6.64 | 886.5 ± 3.09 | 839.5 ± 7.31 |
| Normalized fixed-128 trial / rejected | 1245.2 ± 13.72 | 712.5 ± 10.12 | 894.4 ± 10.51 | 830.9 ± 8.72 |
| Shared subdivision | 1111.9 ± 13.61 | 700.3 ± 5.55 | 871.4 ± 8.05 | 813.5 ± 6.93 |
| Skip unused counting hints | 1064.2 ± 12.78 | 696.7 ± 5.70 | 880.5 ± 9.02 | 815.9 ± 6.98 |
| Reciprocal chain reuse | 1096.9 ± 6.43 | 623.2 ± 1.77 | 811.0 ± 4.25 | 773.4 ± 4.35 |
| Sturm-only 64-bit guard / rejected | 1086.5 ± 11.35 | 623.5 ± 4.84 | 826.1 ± 6.76 | 774.6 ± 4.51 |
| Committed / `phase3-baseline-confirm` | 1407.0 ± 7.42 | 716.7 ± 5.16 | 897.0 ± 7.68 | 841.9 ± 8.42 |
| Retained root reuse / `phase3-confirm` | 1067.5 ± 9.06 | 634.9 ± 3.93 | 823.0 ± 8.25 | 764.2 ± 5.05 |
| Negative closest-point certificate | 1054.9 ± 6.30 | 532.4 ± 2.68 | 807.6 ± 9.34 | 742.5 ± 5.58 |
| Final / `phase3-close-confirm` | 1052.1 ± 11.54 | 527.3 ± 2.89 | 794.1 ± 5.06 | 734.7 ± 6.39 |

The whole-quadrant chord was mathematically valid. Its homogeneous substitution
and seam/endpoint ownership passed an independent BigInteger proof, and 70
focused tests passed. Its roughly 5% / 12% gains came with 78-90% regressions
on the other fixtures, so it was removed. These fixtures do not prove a cheap
geometry classifier for selecting the favorable cases; winner-based selection
would require solving first. No branch based on benchmark names is retained.
The normalization variant and separate smaller Sturm guard offered no consistent
advantage and were also removed.

Reciprocal reuse trades about 3% on the analytic-winner row against 5-11% gains
on the three curved-winner rows relative to the immediately preceding candidate.
It retains the original parameter charts and conditioning, unlike the discarded
whole-quadrant substitution. Repeated final measurements and controls determine
the retained-source assessment below.

A further exact probe tested whole-chart rejection using uniform positive-gap
signs and `4*N-upperTwiceRaw²*D>=0`. It rejected none of the analytic-winner
fixture's eight admitted charts: the comparison is actually negative on a
nonempty interval near zero in every chart. A more elaborate sign classifier
cannot certify that false inequality, so no classifier or subtree machinery
was added. This does not invalidate the existing comparison at stationary roots.

Raw logs, full compressed BenchmarkDotNet JSON, rejected source/proof snapshots
and accepted source hashes remain under `artifacts/grv-benchmark-021/phase3-*`.
No production source is edited during measurement. Baseline confirmation
temporarily selects the committed production files, parks the new production
partial, and restores exact bytes with hash verification before rebuilding the
accepted candidate. No staging, resetting or committing is performed.

## Verification And Workload Assessment

The added rational fixtures independently verify excluded zero, included one,
repeated midpoint right limits and tiny real splits versus complex roots under
both signs. Batch cells also match independent single-ordinal isolation.
One-word compact tests distinguish 63-bit success from 64-bit failure after an
earlier root has completed, retaining full count/global repetition and clearing
partial rational masks. Reciprocal tests verify input immutability, exact
reversal, materialized cells and asymmetric Bernstein/compact outcomes against
separate construction of each polynomial, including irrational roots and
nominal-degree/zero-factor fallbacks.

Nineteen direct segment-projection cases use an independent rational BigInteger
oracle, with both orientations, exact endpoints, one-coordinate-unit outside,
zero radial axis dot and zero endpoint coefficients. The initial false stub
failed the eleven accepting cases and passed the eight rejecting cases; the
implementation passes all nineteen. Six complete-contact translations along
the capsule axis preserve the known closest point, exact endpoint ownership,
raw-neighbor touching, depth and normal. Compact-capacity regressions also
materialize authoritative fallback roots after failure, including singleton
cleanup adjacent to zero or an already completed quarter root.

The final fixed-count confirmation reduces mean costs **25.2%, 26.4%, 11.5%
and 12.7%** against the fresh committed confirmation, respectively. Each of the
two final accepted-source confirmations has 99.9% intervals disjoint from
both committed captures in every fixture. There is no retained fixture tradeoff
and every row reports **0 B/op**. Small changes among intermediate candidates
are not independently claimed where their intervals overlap.

Fresh gates ran with the final production source under
`artifacts/grv-benchmark-021/final-gates-20261005T175707569Z-4b6b749c70394f928d0c83d31bdc1fd0/`.
Both repositories build all targets in Release and ReleaseLean, and their
complete owning suites pass:

| Owner | Release tests | ReleaseLean tests |
| --- | ---: | ---: |
| FixedMathSharp core / FluentAssertions | 4,214 | 4,193 |
| FixedMathSharp.Chronicler | 49 | 49 |
| Gravitas | 4,537 | 4,476 |

The focused Debug gates pass **585 FixedMathSharp / 28 Gravitas** cases,
including live dirty caller storage, a 1 MiB worker stack and strict warmed
allocation checks. Both DocFX builds pass warnings-as-errors, local links,
branding, repository actions and retained API resources.

Coverage settings/exclusions are unchanged. Exact raw OpenCover totals and
rendered ReportGenerator totals are checked separately, including fully covered
methods; no rounded 99.9% result counts as complete. Rendered totals are:

| Owner / configuration | Lines | Branches | Fully covered methods |
| --- | ---: | ---: | ---: |
| FixedMathSharp / Release | 52,851 / 52,851 | 12,228 / 12,228 | 3,943 / 3,943 |
| FixedMathSharp / ReleaseLean | 52,944 / 52,944 | 12,228 / 12,228 | 3,939 / 3,939 |
| FixedMathSharp.Chronicler / both | 85 / 85 | 12 / 12 | 18 / 18 |
| Gravitas / Release | 56,272 / 56,272 | 16,302 / 16,302 | 5,413 / 5,413 |
| Gravitas / ReleaseLean | 56,270 / 56,270 | 16,302 / 16,302 | 5,412 / 5,412 |

FixedMathSharp's raw counts match those owning rendered totals. Gravitas's
OpenCover mapping reports **44,542 / 44,542 sequence points, 13,296 / 13,296
branches and 4,603 / 4,603 methods** in Release; Lean reports **44,540 / 44,540,
13,296 / 13,296 and 4,602 / 4,602**. These formats have different mappings;
both must independently reach exact 100%. Chronicler's partial core dependency
is not merged over the complete core suite. Complexity exceptions are refreshed
from both exact raw captures; wrappers now below ten are removed from the
register rather than retaining stale exception entries.

## Final Controls And Remaining Cost

The final adaptive control run uses two launches, five warmups and fifteen
measured iterations with a 250 ms target. All **28 fixtures** pass their
behavior preflight and report **0 B/op**. Values below are microseconds/query
with BenchmarkDotNet's 99.9% confidence half-width. These are current absolute
costs; older captures with different settings do not establish control speedups.

| Stadium control | Mean ± error |
| --- | ---: |
| Cap | 15.85 ± 0.071 |
| Containment | 23.42 ± 0.285 |
| End | 19.59 ± 0.152 |
| Side | 19.55 ± 0.122 |
| Straight rim | 69.38 ± 0.377 |
| Separated rim | 45.22 ± 0.183 |
| Oblique analytic winner | 1,053.55 ± 5.998 |
| Original oblique rim | 542.30 ± 8.228 |
| Irrational oblique rim | 794.85 ± 6.588 |
| Positive oblique rim | 736.36 ± 7.007 |

| Circle/cylinder control | Mixed circle slab | Equivalent 3D cylinder |
| --- | ---: | ---: |
| Cap | 16.51 ± 0.115 | 17.59 ± 0.244 |
| Endpoint gap | 21.97 ± 0.090 | 22.08 ± 0.117 |
| Endpoint rim | 80.15 ± 0.549 | 81.70 ± 1.144 |
| Oblique interior rim | 1,098.17 ± 22.223 | 1,051.79 ± 7.033 |
| Side | 22.02 ± 0.196 | 22.44 ± 0.210 |
| Zero core | 21.81 ± 0.137 | 22.22 ± 0.432 |

| Triangle/cylinder control | Mean ± error |
| --- | ---: |
| Cap face | 45.06 ± 0.558 |
| Straight side | 72.45 ± 0.468 |
| Rounded odd rim | 158.38 ± 0.835 |
| Oblique rim | 755.71 ± 3.480 |
| Gap | 47.32 ± 0.470 |
| Scalar face | 71.76 ± 0.820 |

Raw results are in `phase3-close-controls/` and `phase3-triangle-close/` under
`artifacts/grv-benchmark-021/`. The fixed-count table above is the matched
performance comparison; these adaptive captures are separate controls.

Four final EventPipe captures select only `WorkloadActual`, using one launch,
three warmups and eight measured iterations with a 500 ms target. The inclusive
sampled CPU attribution is:

| Stadium fixture | Paired reciprocal root producer | Sign queries |
| --- | ---: | ---: |
| Analytic winner | 42.213% | 35.513% |
| Original rim | 16.674% | 39.222% |
| Irrational rim | 16.013% | 45.523% |
| Positive rim | 18.283% | 42.632% |

These are inclusive shares of two specific frames, not exclusive arithmetic
costs; other parent/child profile entries overlap and must not be added. The prior 55.6% root-acquisition
figure includes other root calls; the new paired-producer frame is a narrower
attribution, not a directly comparable total. The analytic-winner trace's
largest exclusive frames are wide multiplication (15.935%), division (12.833%)
and approximate sign evaluation (9.016%). The other fixtures distribute their
remaining cost among exact polynomial evaluation, multiplication, shifted
addition and sign evaluation. Inlining or absence from the selected sampled
frames does not prove zero work. Raw traces and the method summary remain in
`phase3-close-profile/` and `phase3-close-profile.json`.

## Closure Assessment

A final endpoint-aware Bernstein trial retained detected midpoint/upper roots
without increasing its depth/node/workspace bounds. Focused tests passed and a
read-only mathematical review found no production defect. Matched means were
**1,042.3 ± 5.69 / 540.5 ± 5.30 / 792.7 ± 2.47 / 747.5 ± 11.04 us** in the
four-fixture order above, all 0 B/op. It did not establish a useful gain and
the original rim was slower than the final accepted confirmation with disjoint
intervals. It is rejected: production and tests are restored exactly from the
validated pre-trial backups. Raw results remain in `phase3-bernstein-endpoints/`.
Restored production hashes match the final-gate manifest exactly; the restored
test file also matches its pre-trial backup. A fresh local-stack benchmark build
and **275 focused Release tests** pass after restoration. The complete
Release/Lean coverage gates above apply to that same restored source.

The 2026-10-05 closure criterion is a reproducible improvement across the
complete curved fixtures, preserved ordinary/3D behavior, bounded scratch,
zero allocation and exact coverage. It is not an arbitrary host latency target.
The retained changes satisfy that criterion without dropping constrained
directions, relaxing exact classification/rounding, adding a cache or exposing
an accuracy setting. Independent mathematical, numerical and simplicity reviews
have no outstanding findings.

The remaining roughly **0.53–1.05 ms/query** stadium cost is substantial.
Microbenchmark latency is not a simulation frame budget: many simultaneous
oblique contacts need representative world-step measurements before selecting
a host rate or capacity. This closes the focused optimization investigation,
not a universal MMO/strategy throughput claim or a cross-platform replay gate.
At the end of this phase, GRV-Benchmark-020 remained open for its distinct ellipse-depth owner. A future
root-arithmetic or workload redesign should be justified by new measurements,
retain these fixtures and preserve the full-domain contract. Released-package
validation remains required after FixedMathSharp is released first.
