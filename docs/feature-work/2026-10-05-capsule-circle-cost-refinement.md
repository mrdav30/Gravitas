# Capsule/Circle-Slab Output Refinement

This records successive GRV-Benchmark-020 refinements after closing
GRV-Benchmark-021. The starting committed revisions are Gravitas `989c7f9`
and FixedMathSharp `bd8f6a2`. The complete shared cylinder/capsule query is
the correctness baseline; the former sampled mixed direction subset is not
a valid performance target.

## Retained Design

The retained positive ellipse root already encloses the winning exact value.
Its existing signed interval Horner evaluator now also supplies bounds for
`N(alpha)/D(alpha)`. Both polynomials keep the same padded degree and therefore
the same homogeneous denominator scale. A strictly positive denominator
minimum permits independent lower and upper ratio bounds. An uncertain
denominator returns the original complete search range. A nonpositive
numerator minimum is bounded by zero under the caller's nonnegative-value
proof; it is never treated as a negative square-root input.

The existing wide integer division and square-root owners yield bounds on
the **floor** of the magnitude. Division uses full-width quotient, remainder
and scratch buffers. Comparison against `cap^2*D` precedes packing the quotient
into two words; the unclipped quotient is strictly below `2^128`. Constants
do not copy root coordinates, including after deep repeated-root refinement.
Rational roots use a zero-width interval.

The depth caller still performs its exact conceptual-clamping comparison
first. For a negative gap, contact admission proves magnitude at most the
capsule radius. A magnitude floor upper bound `u` gives
`floor(radius-magnitude) >= max(0,radius-u-1)`: the extra raw unit accounts
for the unknown fractional part. For a positive winning gap, the admitted
major-axis candidate bounds the magnitude by the cylinder radius. Normal
components retain their unit-magnitude bound. The original exact binary
threshold and final half-raw nearest-even comparisons decide every result.

Depth materialization then reduces its numerator and denominator once at
the retained root. Common low powers of the strictly positive parameter
cancel together. Each positive pseudo-step multiplies **both** polynomial
values by `abs(leading(P))`, even when one leading coefficient is zero.
Joint degree trimming and shared power-of-two normalization preserve their
relative scale. The previous single-query reducer reuses the same arithmetic
step; there is no second division or root-sign solver.

The original structured ratio supplies the initial interval bounds before
reduction: reduced coefficients can have a weaker enclosure. For nonzero
ellipse `Q`, the defining root is quartic and one step leaves degree at most
three. When `Q=0`, the root owner removes stationary zero factors and retains
a quadratic. Canceling the common numerator/denominator `t^2` first leaves
one shared step and a constant ratio. Normal-component queries already have
degree at most two and keep their original representation.

This is reusable deterministic arithmetic in FixedMathSharp. Gravitas retains
contact, manifold, anchor, material and mixed response policy. Both mixed
circle slabs and 3D cylinders consume the same improvement. No new public API,
friendship, configuration, approximate contact decision or retained cache is
introduced.

## Resources And Correctness

For interval width `W`, the four live bounds and returning division frame need
at most `9W+7` words. Under the existing ellipse bounds, `W<=1071`, hence
**77,168 bytes** of explicit scratch. The non-inlined bounds owner returns
before exact sign or reduced-pair work begins.

Paired pseudo-division increases coefficient height by at most `Bp+1` bits
per common step. The production ellipse uses one step after common-factor
cancellation, so coefficients stay below 4,309 bits; the sizing rule reserves
69 words. The retained pair adds at most **5,530 bytes**, including signs.
Depth threshold slots become 72 words; normal threshold slots remain 43.
The shared pseudo-step's three product buffers return before the next step
or sign query. The existing complete ellipse bound remains below 448 KiB
of explicit stack scratch; the unchanged analytic-winner comparison remains
larger than the new materialization path. This is a source-derived buffer
bound, not a total JIT stack measurement. Existing 1 MiB dirty-stack tests
with 64 KiB of live caller storage supplement the width and lifetime proof.

Independent tests exercise rational and irrational cells, negative denominator
shifts, loose or uncertain interval bounds, a zero numerator, unsigned caps,
wide division, constant ratios after deep repeated-root refinement, both
defining-polynomial leading signs, a zero leading numerator, common root
powers, unequal degree drops, repeated roots, differing coefficient widths
and dirty output tails. Paired quotient identity is checked independently in
the quadratic field rather than by comparing two production routes. Existing
complete-contact tests retain classification, canonical ordering, reflected
and permuted normals, raw-neighbor and half-raw ties, conceptual clamping,
full-domain rigid frames and strict allocation guards.

## Measurements And Validation

Captures use `UseLocalLsfStack=true` in MSBuild and the environment, Windows 11,
an i7-9700K, .NET 8.0.29 and SDK 10.0.302. One BelowNormal heavy job runs at
a time, with `DOTNET_PROCESSOR_COUNT=2` and serial builds. The requested CLI
option is `--affinity 3`; job metadata reports `Affinity=11` in both matched
captures. These are recorded separately without inferring a different mask.
Each row uses two launches, five warmups and fifteen measured 250 ms adaptive
iterations per launch. Values are microseconds per dispatched query; errors
are 99.9% confidence half-widths. Setup asserts complete classification,
known quarter-unit depths and mixed/3D depth agreement.

The initial committed oblique baseline is **1,158.03 ± 17.39 / 1,128.60 ±
18.11 µs** for mixed / 3D. Bounds alone reduce it to **976.53 ± 15.30 /
967.80 ± 17.62 µs**. The shared-ratio smoke is **763.0 ± 13.11 / 737.4 ±
17.80 µs**. Its 3D distribution warning is retained in the raw log; smoke
is not substituted for the complete matched confirmation. All measured rows
report **0 B/op**. Initial twelve-row captures, stage backups, source hashes
and logs are under `artifacts/grv-benchmark-020`.

The `retained-confirm` and `retained-rebuilt-confirm` captures and their
profiles are **excluded**. Restoring the candidate also restored older file
timestamps. The generated child used a different intermediate directory and
reused its baseline upstream DLL even after the launcher was rebuilt.
Reflection confirmed that the timed child lacked both new helpers; checking
only the launcher was insufficient. The corrected `retained-verified-confirm`
refreshes restored timestamps, uses `-t:Rebuild`, verifies source hashes and
launcher DLL equality, and retains the generated child with `--keepFiles`.
Fresh-process checks verify helper presence in the **actual timed and profiled
child DLLs**, whose SHA-256 hashes and MVIDs are retained in
`retained-verified-child-assembly.json` and
`retained-verified-profile-child-assembly.json`. Use only
`retained-verified-profile` for the candidate profile.

| Geometry | Committed mixed | Retained mixed | Committed 3D | Retained 3D |
| --- | ---: | ---: | ---: | ---: |
| Cap | 17.49 ± 0.214 | 17.63 ± 0.270 | 17.96 ± 0.245 | 18.51 ± 0.259 |
| Endpoint gap | 23.61 ± 0.307 | 23.66 ± 0.283 | 23.98 ± 0.312 | 23.69 ± 0.357 |
| Endpoint rim | 86.61 ± 1.195 | 86.39 ± 1.174 | 87.01 ± 1.583 | 87.18 ± 1.296 |
| Oblique interior rim | 1,186.47 ± 22.205 | **745.01 ± 12.541** | 1,124.91 ± 17.824 | **724.62 ± 15.125** |
| Side | 23.54 ± 0.325 | 23.98 ± 0.385 | 24.07 ± 0.312 | 24.49 ± 0.539 |
| Zero capsule core | 23.45 ± 0.298 | 23.46 ± 0.294 | 23.55 ± 0.315 | 23.96 ± 0.275 |

These are `baseline-confirm` versus `retained-verified-confirm`. The oblique
means improve **37.2% mixed / 35.6% 3D** against the fresh committed control,
and **35.7% / 35.8%** against the initial committed capture. This establishes
the curved improvement across repeated controls, not an ordinary-path speedup.

The full matrix's ordinary means shift between approximately -1.2% and +3.1%.
The 3D cap row's +3.1% prompted a targeted back-to-back repeat, with the same
two-launch settings and actual-child assembly checks:

| Cap repeat | Committed | Retained |
| --- | ---: | ---: |
| Mixed | 17.50 ± 0.222 | 18.09 ± 0.235 |
| 3D | 18.19 ± 0.270 | 18.36 ± 0.348 |

The repeated 3D difference is +0.9%, with overlapping confidence intervals;
the mixed repeat moves +3.4%, while its full-matrix difference was +0.8%.
Those small ordinary shifts are not fully isolated. Publish them rather than
claiming zero regression or an improvement in code paths that do not invoke
the new root materializer. All repeat rows remain **0 B/op**. Captures and
child identities are under `cap-baseline-control` / `cap-retained-control`.

Fresh owning-suite validation is retained under
`artifacts/grv-benchmark-020/final-gates-20261006T011442939Z-0be29b185d644823977d584f4ccc0350`.
The invocation's `result.json` confirms all serial gates passed:

- FixedMathSharp: **4,239 Release / 4,218 Lean** tests, plus **49** Chronicler
  owning tests per configuration; **867** focused Debug tests.
- Gravitas: **4,537 Release / 4,476 Lean** tests; **28** focused Debug tests.
- Release/Lean builds cover both library target frameworks, standard and Lean
  variants, tests and benchmarks. Both DocFX builds pass with warnings as
  errors, local API links and branding/resource checks.
- Every owning capture has exactly **100% reachable line, branch and method
  coverage**, checked independently in rendered and paired raw reports.
  Coverage exclusions remain unchanged. Chronicler's partially exercised core
  dependency is not unioned into the independently complete core capture.

| Rendered owning capture | Lines | Branches | Fully covered methods |
| --- | ---: | ---: | ---: |
| FixedMathSharp Release | 52,958 / 52,958 | 12,266 / 12,266 | 3,949 / 3,949 |
| FixedMathSharp Lean | 53,051 / 53,051 | 12,266 / 12,266 | 3,945 / 3,945 |
| FixedMathSharp.Chronicler Release / Lean, each | 85 / 85 | 12 / 12 | 18 / 18 |
| Gravitas Release | 56,272 / 56,272 | 16,302 / 16,302 | 5,413 / 5,413 |
| Gravitas Lean | 56,270 / 56,270 | 16,302 / 16,302 | 5,412 / 5,412 |

FixedMathSharp's paired raw totals equal those rendered totals. Gravitas's raw
OpenCover summaries independently report **44,542 / 44,542 sequence points,
13,296 / 13,296 branches, 4,603 / 4,603 methods** in Release and **44,540 /
44,540, 13,296 / 13,296, 4,602 / 4,602** in Lean. Each reporting format is
checked against its own complete totals rather than substituting a rounded
percentage or combining unrelated captures. Source reviews found no actionable
correctness or simplicity issues; reviewers ran no concurrent heavy jobs.

## Remaining Cost And Next Investigation

Four matched `WorkloadActual` profiles use one launch, three warmups and eight
500 ms iterations for endpoint and oblique fixtures, with identical runtime,
local-stack and affinity settings. The retained timed/profiled child MVID is
`9a5f65c9-9ce9-4ba5-aeaa-d7f30eb6634e`; the verified baseline cap child MVID is
`c38ca9cd-d4dc-42d5-9b98-e87ab67a96a6`. The new helpers also appear in the
retained oblique traces. Inclusive sampled CPU percentages below overlap and
must not be added together:

| Named frame | Baseline mixed / 3D | Retained mixed / 3D |
| --- | ---: | ---: |
| Ellipse magnitude-to-raw comparison | 58.666% / 58.722% | 39.436% / 38.609% |
| Exact retained-root sign | 49.757% / 50.270% | 39.640% / 36.802% |
| Largest-positive root acquisition | 30.335% / 29.859% | 43.029% / 42.943% |
| Ellipse depth materialization | Not separately sampled | 40.103% / 39.687% |
| Shared polynomial interval bounds | Old interval-sign wrapper: 19.745% / 19.826% | 23.724% / 21.447% |
| Endpoint analytic materialization | 79.947% / 78.345% | 81.431% / 79.313% |

The larger retained interval percentage is not evidence of a wall-time
regression: the complete query is substantially shorter and the extracted
frame now also serves the initial magnitude bounds. Profiles localize work;
the twelve-row timing capture establishes the complete-query cost.

At the end of the first refinement, GRV-Benchmark-020 remained **open**. Root acquisition and exact depth sign work
still account for substantial oblique cost, while endpoint output retains its
analytic rounding searches. Next, investigate reuse of existing certified
point evaluation and exact rational-square materialization before adding a
new root representation, cache or solver. Keep the full mixed/3D matrix,
signed/zero-`Q` correctness families, ordinary controls, shared analytic
consumers, zero-allocation gates and stack proofs. Any shared-owner refinement
must preserve all consumers, not merely the timed quarter-unit fixture.

The representative oblique query remains about **0.72–0.75 ms** on this host;
an endpoint rim remains about **86–87 µs**. These costs do not establish a
fixed-rate simulation capacity or cross-platform timing guarantee. Budget
complete representative world steps and expected curved-contact frequency.
The accepted implementation exposes no accuracy setting or approximate route.

To reproduce the candidate, build with local source selection and fresh source
timestamps after any controlled restoration, then verify the generated child:

```powershell
$env:UseLocalLsfStack = 'true'
$env:DOTNET_PROCESSOR_COUNT = '2'
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -t:Rebuild -c Release -p:UseLocalLsfStack=true -m:1 -p:BuildInParallel=false -p:UseSharedCompilation=false -nr:false
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll --filter '*CapsuleCircleContactBenchmarks*' --launchCount 2 --warmupCount 5 --iterationCount 15 --iterationTime 250 --affinity 3 --keepFiles --exporters json --artifacts artifacts/grv-benchmark-020/reproduction
```

The ignored `measure-verified.ps1`, `verify-assembly.ps1` and
`cap-controls.ps1` retain the complete assembly/source checks used here.
Changing restoration timestamps does not change source content or its hash.
No benchmark-only instrumentation is added to runtime library code.

## Certified Isolation And Invariant Normal Products

This follow-up starts from committed Gravitas `5a36d1e` and FixedMathSharp
`54edc3b`. It retains two focused changes in existing owners; the defining
polynomials, retained roots, analytic candidate representation and exact
output comparisons remain the correctness baseline.

Sturm variation counting for positive quartic roots now reuses the existing
value-root certified point evaluator for nonconstant rows strictly inside
`(0,1)`. Its bounded integer Horner evaluation returns a nonzero sign only
when the result exceeds its proved coefficient/product truncation error.
Zero means uncertainty, so the original exact polynomial and derivative
right-limit loop still decides vanishing rows and ill-conditioned points.
Constants, zero endpoints and points at or above one retain the existing
exact evaluator. No root chart, point prediction, equality rule or isolation
ordering changes.

For dyadic shift `q`, numerator bit length `b` and row degree `d`, precision
reuses `q+128+d*(q-b+1)`. With `1<=b<=q`, `d<=4` and
`q<=max(9B+64,8B+129)`, the returning certificate scratch is at most
**22,648 bytes** for the ellipse's `B<=1818`, alongside its existing
40,000-byte Sturm arena. This includes later noncrossing refinements, not
only acquisition. The certificate returns before exact fallback evaluation.
The cell capacity and separation proof remain unchanged.

Analytic normal materialization now computes its two invariant
`(2*scale)^2` times component-square coefficient products once per nonzero
component. Binary thresholds and the final half-raw comparison reuse them;
only the two threshold-dependent norm products are recomputed. One product
scratch serves both norm coefficients sequentially after each signed
difference is retained. Borrowed square slots are never scaled in place.

Candidate fields use at most forty words. The normal-square width is
`W=max(2R,2L+K,R+L+1)+2<=122`; product width remains `P=W+3<=125`.
Two retained products replace one comparator scratch product, giving a net
maximum explicit live increase of **1,000 bytes**. The doubled nonnegative
Fixed64 raw scale fits ulong even at MaxValue, and its square fits two words.
The component frame returns before the next component. This preserves scaled
triangle-cylinder/cone radial anchors as well as unit normals for
capsule/slab, capsule/cylinder, box/cylinder and cylinder-pair consumers.

Eight acquisition theory cases pin subunit irrational roots of both
multiplicities and leading signs, dyadic midpoint/right-limit behavior, and
two roots separated by `2^-200`. At their midpoint, a normalized certificate
cannot resolve the exact value `-1`; exact nonzero fallback must retain the
larger root. Ten scaled analytic-normal cases independently pin both half-raw
tie parities, mirrored signs, zero/unit/maximum scale, opposing radical
cancellation and borrowed-input preservation. A forty-word rational/radical/
radicand common factor exercises the full 122-word normal workspace while
canceling to the independently known direction `(-3,4,0)/5`.

The initial seventeen new cases passed against the committed behavior before the optimization
(110 focused cases), and the first combined focused selection passed 476
cases. The maximum-workspace variant was then added for full owner validation.
The source and separate Ponytail reviews found no actionable correctness or
complexity findings. No new public API, friendship, cache, configuration,
general radical solver or downstream arithmetic facade is introduced.

The full confirmation uses two launches, five warmups and fifteen adaptive
250-ms iterations per launch. The fresh committed control and retained
combined means are below, in microseconds; each error is BenchmarkDotNet's
99.9% confidence half-width. Every row reports **0 B/op**.

| Geometry | Committed mixed | Combined mixed | Committed 3D | Combined 3D |
| --- | ---: | ---: | ---: | ---: |
| Cap | 17.96 ± 0.260 | 17.43 ± 0.236 | 18.67 ± 0.209 | 18.21 ± 0.386 |
| Endpoint gap | 23.85 ± 0.324 | 23.52 ± 0.257 | 23.86 ± 0.328 | 23.88 ± 0.327 |
| Endpoint rim | 86.95 ± 1.177 | 81.87 ± 1.894 | 88.25 ± 1.545 | 83.03 ± 1.628 |
| Oblique rim | 716.65 ± 15.076 | 694.16 ± 11.080 | 766.53 ± 16.341 | 698.33 ± 10.497 |
| Side | 24.18 ± 0.492 | 23.57 ± 0.304 | 24.36 ± 0.331 | 24.38 ± 0.315 |
| Zero core | 23.76 ± 0.434 | 23.27 ± 0.317 | 23.82 ± 0.325 | 23.70 ± 0.488 |

A back-to-back committed repeat gives endpoint means **86.53 ± 1.247 /
89.95 ± 1.354 µs** and oblique means **722.41 ± 16.022 / 731.45 ±
16.303 µs** (mixed/3D). Against that repeat, the combined changes reduce
endpoint means by **5.4% / 7.7%** and oblique means by **3.9% / 4.5%**.
Small ordinary-control shifts are not independent optimization claims.
Multimodal warnings remain in the raw captures; the initial mixed oblique
confidence intervals overlap. A short root-only trial is exploratory and
does not establish a separately confirmed root-only improvement.

Captures live under ignored `artifacts/grv-benchmark-020/phase2`:
`committed-control`, `combined-confirm`, `committed-repeat` and
`combined-profile`. Both changed private method IL hashes match between
each rebuilt parent and its actual generated timed/profiled child. The
committed repeat restores exact retained bytes in `finally`, refreshes
timestamps and asserts the retained source hashes. This avoids relying on
an incremental child build after controlled source restoration.

The refined endpoint profile still attributes **52.7%** inclusively to
normal-component rounding and **17.4%** to depth rounding in the mixed
fixture. The mixed oblique profile attributes **37.8%** to root acquisition
and **41.6%** to ellipse depth materialization. These frames overlap and must
not be summed. The subsequent experiment narrows analytic rounding searches
with certified square-root enclosures, reusing existing integer root and
division operations while keeping exact floor, midpoint and clamp decisions.

This combined implementation passed fresh serial gates in
`final-gates-20261006T023426601Z-cf1f0936ac7a410c85b092972a8f3084`.
FixedMathSharp passed **4,257 / 4,236** Release/Lean tests; Gravitas passed
**4,537 / 4,476**. Both configurations preserve **100% reachable line,
branch and method coverage** in raw OpenCover and rendered owner reports,
with exclusions unchanged. Debug dirty-stack/one-MiB-worker and allocation
selections passed, both library target frameworks built, and both DocFX
builds and generated local-link checks passed. Coordinated builds, tests
and benchmark children use `UseLocalLsfStack=true` throughout.

## Factored Quadratic Output Bounds

The next accepted experiment narrows the shared analytic depth and scaled
normal searches with conservative radical enclosures. For a radicand with
`b` active bits, choose `k=max(0,ceil((b-192)/2))`, retain
`T=floor(C/2^(2k))` and reuse the existing narrow integer square root
`q=floor(sqrt(T))`. Then `q*2^k <= sqrt(C) <= (q+1)*2^k`.
The endpoints coincide only when the prefix remainder and every discarded
bit are zero. A perfect truncated prefix alone is insufficient.

Both endpoints remain factored as at most 97-bit integers plus `k`.
Existing shifted signed-magnitude accumulation constructs bounds for
`A+B*sqrt(C)`, reversing root endpoints for negative `B`. This handles
cancellation without new shifted-root storage or a wide square-root engine.
For a forty-word maximum radicand, the upper root can be `2^1280`, which
would require twenty-one words if materialized; its factored endpoint still
fits the two-word prefix representation.

The existing clipped full-width ratio floor implementation moves directly
from the polynomial-root owner to `WideArithmetic.GetRatioFloorSquareRoot`.
It returns `min(cap,floor(sqrt(N/D)))` for positive `D` and equal padded
input widths of at least two words. The full-width comparison and division
precede its proven low-128-bit quotient packing. Existing polynomial-root
consumers call that owner directly; no forwarding layer or new friendship
is introduced.

For normals, denominator bounds are shared across all components. Numerator
products remain `(2*S)^2*n_i^2`, so their corresponding denominator is
`4*|n|^2`. Nonpositive denominator minima preserve the original full search;
nonpositive numerator minima bound the proven nonnegative square by zero.
Depth bounds include the extra fractional raw unit needed when subtracting
a magnitude floor from the radius. The exact final half-raw and MaxValue
comparisons still own nearest-even rounding and conceptual clamping.

Maximum depth and normal bounds use **62 / 147 words**. Shared denominator
bounds and prefix endpoints add 298 retained words; temporary component
numerator bounds return before exact searches. The maximum analytic-normal
exact-sign explicit buffer peak is **23,176 bytes**, excluding caller
buffers and JIT spills. The existing ellipse stack and resource proofs are
unchanged.

Twenty-nine new analytic cases passed against the previous implementation
before the optimization (68 cases in that selection). Thirty-nine arithmetic
cases independently verify prefix/discard/carry boundaries and clipped ratio
floors, including 147-word division. The optimized Debug selection passed
953 cases. Six further cases pin zero normals, pure radical squared gaps and
usable denominator bounds with negative numerator minima. Both source and
Ponytail reviews found no actionable issues.

The full two-launch capture `phase3/enclosure-confirm-valid` uses the same
host and measurement settings as the preceding matrix. All five inspected
private-method IL hashes match the rebuilt parent and actual timed child;
all three affected production source hashes remain unchanged. Every row
reports **0 B/op**. Values are microseconds with 99.9% confidence half-widths.

| Geometry | Mixed mean ± error | 3D mean ± error |
| --- | ---: | ---: |
| Cap | 10.008 ± 0.128 | 10.471 ± 0.176 |
| Endpoint gap | 23.799 ± 0.268 | 23.941 ± 0.296 |
| Endpoint rim | 36.382 ± 0.552 | 37.470 ± 0.628 |
| Oblique rim | 699.005 ± 11.791 | 704.199 ± 12.403 |
| Side | 10.796 ± 0.185 | 11.451 ± 0.182 |
| Zero core | 10.886 ± 0.163 | 11.181 ± 0.148 |

Compared with the preceding combined capture, endpoint means improve
**55.6% / 54.9%**, cap **42.6% / 42.5%**, side **54.2% / 53.0%** and zero
core about **53.2% / 52.8%**. Endpoint-gap and oblique shifts do not establish
an independent improvement. Multimodal/outlier warnings remain in the raw
captures. The first incorrectly filtered launcher run was interrupted and
excluded; explicit string-array filter typing fixes its argument expansion.

## Rejected Retained-Cell Certificate

A further experiment tried the existing normalized point-sign evaluator
before full interval Horner work. For a subunit dyadic cell, the existing
normalized derivative bound makes a nonzero result a valid whole-cell sign
when `q>=p+2*ceilLog2(d+1)`. Uncertainty always used the original exact
interval bounds, including the linear zero-in-cell shortcut. Both reviewers
confirmed the proof and bounded scratch; no equality or root chart changed.

Twelve additional cases passed before and after the experiment. Simple and
repeated subunit roots retain decisive signs without changing their cells;
scaled partial factors have independently known tiny negative, zero and
positive values; mirrored linear queries change sign inside a deep cell and
retain the exact boundary comparison. The focused selection passed **971**
cases for the baseline and both work budgets.

The one-launch exploratory captures use fifteen 250-ms iterations, matching
the other settings. All rows report 0 B/op and all six inspected method IL
hashes match their rebuilt parents and actual children.

| Variant | Mixed endpoint | 3D endpoint | Mixed oblique | 3D oblique |
| --- | ---: | ---: | ---: | ---: |
| Confirmed enclosure, two launches | 36.382 ± 0.552 | 37.470 ± 0.628 | 699.005 ± 11.791 | 704.199 ± 12.403 |
| Cell certificate, 32-bit budget | 36.765 ± 1.302 | 37.222 ± 1.046 | 724.392 ± 23.961 | 742.312 ± 25.652 |
| Cell certificate, 64-bit budget | 36.761 ± 0.672 | 38.339 ± 1.125 | 735.84 ± 19.690 | 747.62 ± 18.817 |

Neither budget establishes a benefit, so the extra certificate is **removed**.
These are exploratory comparisons, not an isolated claim about an unavoidable
regression or the certificate's applicability to other fixtures. The existing
exact interval owner remains unchanged. All three retained production source
hashes match the confirmed enclosure capture after restoration. The twelve
independent regression cases stay; the failed optimization adds no runtime
branch, precision setting, alternate solver or cache.

## Shared Consumer Controls

A controlled back-to-back pair restores the preceding certified-Sturm/invariant-
product implementation (`shared-phase2`) and then the retained factored bounds
(`shared-enclosure`). Restorations refresh timestamps and verify the original
production hashes. Both sixteen-row captures use two launches, five warmups and
fifteen 250-ms iterations, with matching runtime and local-stack settings.
Rebuilt launcher and actual child private-method IL hashes match in each capture;
the retained source hashes also match the full circle/slab confirmation.
All **32 rows report 0 B/op**. Values are microseconds with 99.9% confidence
half-widths; raw multimodal warnings are retained.

| Shared fixture / method | Preceding implementation | Factored bounds |
| --- | ---: | ---: |
| `CapsuleSlab.Cap` | 17.436 ± 0.350 | 11.138 ± 0.168 |
| `CapsuleSlab.Containment` | 23.351 ± 0.280 | 10.667 ± 0.144 |
| `CapsuleSlab.EndRegion` | 20.779 ± 0.243 | 11.751 ± 0.147 |
| `CapsuleSlab.ObliqueFanWins` | 1118.131 ± 16.103 | 1107.069 ± 34.476 |
| `CapsuleSlab.ObliqueInteriorRim` | 573.754 ± 9.763 | 571.837 ± 11.152 |
| `CapsuleSlab.ObliqueIrrationalRim` | 849.472 ± 14.033 | 836.796 ± 12.912 |
| `CapsuleSlab.ObliquePositiveRim` | 778.439 ± 12.992 | 776.810 ± 13.949 |
| `CapsuleSlab.SeparatedRim` | 49.155 ± 0.798 | 49.357 ± 0.686 |
| `CapsuleSlab.Side` | 21.173 ± 0.258 | 11.910 ± 0.140 |
| `CapsuleSlab.StraightRim` | 72.157 ± 1.048 | 55.360 ± 0.988 |
| `MeshCone.ConeTriangle(ObliqueRim)` | 821.371 ± 13.537 | 802.945 ± 15.571 |
| `MeshCone.ConeTriangle(SideFace)` | 53.832 ± 0.786 | 54.452 ± 0.829 |
| `MeshCylinder.CylinderTriangle(ObliqueRim)` | 670.693 ± 15.581 | 654.370 ± 10.865 |
| `MeshCylinder.CircleSlabTriangle(ObliqueRim)` | 679.930 ± 12.061 | 663.519 ± 12.932 |
| `MeshCylinder.CylinderTriangle(SideFace)` | 25.323 ± 0.380 | 25.394 ± 0.337 |
| `MeshCylinder.CircleSlabTriangle(SideFace)` | 24.920 ± 0.335 | 25.446 ± 0.373 |

The stadium cap, containment, end-region and side means improve **36.1%,
54.3%, 43.4% and 43.7%**; straight-rim materialization improves **23.3%**.
The separated, four oblique stadium and six triangle controls have overlapping
confidence intervals. Their mean shifts range from approximately **-2.4% to
+2.1%**; they do not establish independent gains or an isolated regression.
This includes scaled radial-anchor consumers and mixed circle-slab/3D-cylinder
triangle parity. These measurements support reusing the shared owner without
claiming that every consumer becomes faster.

## Enclosure Profile And Remaining Cost

The retained `phase3/enclosure-profile` uses four endpoint/oblique mixed/3D
`WorkloadActual` traces, one launch, three warmups and eight 500-ms iterations.
The actual profiled child MVID is `25eb44df-a429-4845-936c-d732439e7d73`;
all five private-method IL hashes from the full timed confirmation match this
child. The sixth inspected method, the original exact interval-sign owner,
also matches the rebuilt shared-control child. All three production source
hashes remain unchanged. These sampled percentages localize remaining work;
they do not replace the two-launch complete-query timings.

| Inclusive sampled frame | Mixed | 3D |
| --- | ---: | ---: |
| Endpoint analytic materialization | 31.023% | 25.585% |
| Endpoint normal-component exact search | 5.487% | 6.395% |
| Endpoint depth exact search | 4.838% | 3.699% |
| Oblique largest-positive root acquisition | 34.857% | 42.629% |
| Oblique ellipse depth materialization | 41.210% | 35.483% |
| Oblique exact retained-root sign | 42.460% | 38.683% |
| Oblique retained-cell interval sign | 27.180% | 25.501% |
| Oblique Sturm variations | 23.980% | 23.980% |

Inclusive frames overlap and must not be summed. In both oblique traces the
largest exclusive leaf is the existing wide polynomial multiply, at
**17.442% / 17.683%**; the certified point evaluator follows at **14.374% /
14.891%**. Endpoint work is now distributed across complete feature construction,
materialization and anchors rather than dominated by repeated rounding thresholds.
The factored-bound refinement targets analytic output and does not claim a
new oblique-root improvement.

The complete representative circle/slab oblique contact is now approximately
**0.699 / 0.704 ms** mixed/3D on this host. Endpoint rim output is **36.382 /
37.470 µs**, ordinary cap/side/zero-core contacts are roughly **10–11.5 µs**,
and the separated endpoint fixture is about **24 µs**. Remaining root isolation
and exact quartic sign work are explicitly retained. The failed cell-certificate
trial is evidence to stop that experiment, not proof that further improvements
are impossible. Complete world-step costs and curved-contact frequency still
need host-workload budgeting; these query timings promise no fixed rate or
universal multiplayer scale.

## Final Coverage Boundary Checks

The first complete final capture passed all 4,343 Release tests and covered every
line and method, but exposed two uncovered branch paths. One was the negative-gap
lower-bound clamp at exact tangency; an added pure-radical magnitude-two/radius-two
case directly verifies zero depth without conceptual clamping. The other was an
unreachable prefix read guard. For a nonzero discarded remainder `r` (an even
value from 2 through 62) and retained bit count `t` (191 or 192), the input bit
count is `64*w+r+t`. Since `193<=r+t<=254`, its active input length is exactly
`w+4`, guaranteeing the last prefix upper-word read at `w+3`. The redundant
length conjunct is removed; the zero-shift guard remains. The existing first
shifted-square test now uses its unpadded four-word input. Independent review
confirmed the proof. Exclusions are unchanged; a fresh full owner invocation
and final-source benchmark/profile confirmations follow this cleanup.

## Fresh Final Owner Gates

The final cleanup passed the complete serial invocation
`final-gates-20261006T032954069Z-c485b9d8d9004eb69fc3cdbb0d02cb2c`;
its `result.json` reports `Passed=true`, local stack selection and processor count.
No benchmark or profiler overlapped these gates.

- FixedMathSharp: **4,344 Release / 4,323 Lean** tests, **49** Chronicler adapter
  owning tests in each configuration, and **972** focused Debug cases.
- Gravitas: **4,537 Release / 4,476 Lean** tests, plus **28** focused Debug cases.
- Complete solution builds cover `netstandard2.1` and `net8.0`, both standard
  and Lean variants, tests and benchmarks, with `UseLocalLsfStack=true`.
- Both DocFX builds pass with warnings as errors; generated local links,
  branding and repository/resource checks pass.
- Coverage exclusions remain unchanged. Every owning raw and rendered capture
  has exactly **100% reachable line, branch and method coverage**.

| Rendered owning capture | Lines | Branches | Fully covered methods |
| --- | ---: | ---: | ---: |
| FixedMathSharp-Release | 53057/53057 | 12306/12306 | 3953/3953 |
| FixedMathSharp.Chronicler-Release | 85/85 | 12/12 | 18/18 |
| FixedMathSharp.Chronicler-ReleaseLean | 85/85 | 12/12 | 18/18 |
| FixedMathSharp-ReleaseLean | 53150/53150 | 12306/12306 | 3949/3949 |
| Gravitas-Release | 56272/56272 | 16302/16302 | 5413/5413 |
| Gravitas-ReleaseLean | 56270/56270 | 16302/16302 | 5412/5412 |

FixedMathSharp's paired raw totals equal its rendered totals. Gravitas's raw
OpenCover owning summaries independently report **44,542 / 44,542 sequence
points, 13,296 / 13,296 branches, 4,603 / 4,603 methods** in Release and
**44,540 / 44,540, 13,296 / 13,296, 4,602 / 4,602** in Lean. Each format is
checked against its own complete totals; no rounded percentage or merged
partial dependency capture substitutes for completeness. The new prefix owner
has measured complexity 16, scaled-normal setup 12, and depth rounding retains
14; their individual Release/Lean lines and branches are fully covered and
the exception register is refreshed. Independent source, resource and Ponytail
reviews found no actionable correctness or simplicity issues.

## Final-Source Timing Confirmation

After the unreachable-guard cleanup, `phase3/final-confirm` repeats all twelve
circle/slab rows with two launches and the same fifteen 250-ms iterations.
All **twelve rows report 0 B/op**. All six inspected private-method IL hashes
match the rebuilt launcher and actual timed child (MVID `4edca8c2-91eb-4675-b8fb-53d46e526531`);
all three production source hashes match the gated source. Values remain
microseconds with 99.9% confidence half-widths; multimodal/outlier warnings
are preserved in the raw log.

| Geometry | Final mixed mean ± error | Final 3D mean ± error |
| --- | ---: | ---: |
| Cap | 10.272 ± 0.327 | 10.605 ± 0.226 |
| Endpoint gap | 23.549 ± 0.299 | 23.383 ± 0.275 |
| Endpoint rim | 36.578 ± 0.568 | 38.962 ± 0.536 |
| Oblique rim | 696.036 ± 11.119 | 699.465 ± 12.521 |
| Side | 11.455 ± 0.494 | 11.235 ± 0.142 |
| Zero core | 10.753 ± 0.146 | 11.531 ± 0.295 |

Against the follow-up's fresh committed control, final endpoint means improve
**57.9% mixed / 55.9% 3D**; ordinary cap/side/zero-core means improve about
**43–55%**. Against the preceding combined implementation, endpoints improve
**55.3% / 53.1%**. The new oblique means **696.036 / 699.465 µs** overlap the
preceding combined capture; the radical enclosure does not establish an
independent oblique speedup.

The final 3D endpoint mean is 4.0% above the first enclosure capture's
37.470 µs, with nonoverlapping reported intervals. Other repeated rows move
by roughly -2.3% to +6.1%; their intervals overlap. These are separate
confirmations rather than an isolated causal test of the removed guard, so
no cleanup speedup or universal zero regression is claimed. Both confirmations
retain the substantial output improvement, exact answers and zero allocation.
The closing figures use the final source rather than selecting the fastest run.

## Final-Source Shared Recheck

`phase3/final-shared` repeats the complete sixteen-row shared matrix after the
coverage cleanup, using the same two-launch protocol. Every row reports **0 B/op**;
all six inspected private-method IL hashes match the rebuilt launcher and actual
child, and the production source matches both the full circle/slab capture and
owner gates. Values are microseconds with 99.9% confidence half-widths.

| Shared fixture / method | Final source mean ± error |
| --- | ---: |
| `CapsuleSlab.Contact(Cap)` | 10.816 ± 0.132 |
| `CapsuleSlab.Contact(Containment)` | 10.620 ± 0.125 |
| `CapsuleSlab.Contact(EndRegion)` | 11.459 ± 0.120 |
| `CapsuleSlab.Contact(ObliqueFanWins)` | 1043.110 ± 13.680 |
| `CapsuleSlab.Contact(ObliqueInteriorRim)` | 561.599 ± 7.816 |
| `CapsuleSlab.Contact(ObliqueIrrationalRim)` | 821.848 ± 11.278 |
| `CapsuleSlab.Contact(ObliquePositiveRim)` | 754.968 ± 9.943 |
| `CapsuleSlab.Contact(SeparatedRim)` | 47.459 ± 0.418 |
| `CapsuleSlab.Contact(Side)` | 11.391 ± 0.093 |
| `CapsuleSlab.Contact(StraightRim)` | 53.525 ± 0.650 |
| `MeshCone.ConeTriangle(ObliqueRim)` | 788.195 ± 10.153 |
| `MeshCone.ConeTriangle(SideFace)` | 53.032 ± 0.606 |
| `MeshCylinder.CylinderTriangle(ObliqueRim)` | 634.561 ± 8.934 |
| `MeshCylinder.CircleSlabTriangle(ObliqueRim)` | 643.950 ± 10.861 |
| `MeshCylinder.CylinderTriangle(SideFace)` | 24.803 ± 0.288 |
| `MeshCylinder.CircleSlabTriangle(SideFace)` | 24.654 ± 0.256 |

The substantial analytic-output reductions persist. Several curved/control means
also move downward in this later repeat, including rows whose preceding paired
intervals overlapped. This is a final-source confirmation rather than another
paired isolation of the cleanup, so those shifts are not attributed to a new
root optimization. The complete prior before/after pair and its warnings remain
available; the final shared matrix has no setup failure or allocation regression.

## Closure Decision And Final Profile

The final four-row `phase3/final-profile` uses the same one-launch, three-warmup,
eight-500-ms `WorkloadActual` protocol. Its actual child MVID is `4edca8c2-91eb-4675-b8fb-53d46e526531`.
All six inspected private-method IL hashes match both final timed matrices;
production hashes match the owner gates. Inclusive percentages below overlap
and must not be summed.

| Final inclusive sampled frame | Mixed | 3D |
| --- | ---: | ---: |
| Endpoint analytic materialization | 33.699% | 34.556% |
| Oblique root acquisition | 36.877% | 36.908% |
| Oblique ellipse depth | 42.691% | 43.705% |
| Oblique retained-root sign | 41.146% | 43.203% |
| Oblique retained-cell interval sign | 25.173% | 26.919% |
| Oblique Sturm variations | 24.215% | 23.785% |

The largest exclusive oblique leaf remains wide polynomial multiplication,
**17.084% / 18.376%**, followed by certified point evaluation at **14.871% /
14.770%**. Profiles support the remaining-stage diagnosis; the complete timings
establish the gain. Sampling and JIT attribution can vary between traces, so
percentages are neither additive stage budgets nor standalone regression gates.

**GRV-Benchmark-020 closes 2026-10-05** under the same judgment-based bar as
#021: correct complete contacts, reproduced mixed/3D and shared-consumer gains,
zero allocation, bounded resources, independent review and fresh exact full
coverage. The representative oblique query remains about **0.696 / 0.699 ms**;
it is a significant retained host-workload cost, not a fixed-rate capacity
promise or an unavoidable lower bound. No cheaper demonstrated refinement
remains in this investigation. The root-cell experiment is removed rather
than retained as an alternate path; its independent regression tests remain.
Future optimization should start from new representative workload/profile
evidence. No new public API, configuration, cache, approximate contact decision,
solver facade, public wide type or additional friendship is introduced.

The coordinated changes remain **unstaged and uncommitted**. Release
FixedMathSharp first, then validate Gravitas against the released package
before releasing Gravitas; local-source validation is not that release gate.
Ignored captures, source/IL identities, stage backups and serial driver scripts
are retained under `artifacts/grv-benchmark-020/phase3`; only the verified
invocation cited above supplies final owner completeness. Earlier failed,
stale-child and incorrectly filtered captures remain explicitly excluded.

For the final shared controls, reuse the local-stack environment and fresh
Release build shown above, then run:

```powershell
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll --filter '*CapsuleSlabContactBenchmarks*' '*MeshCylinderContactBenchmarks*SideFace*' '*MeshCylinderContactBenchmarks*ObliqueRim*' '*MeshConeContactBenchmarks*SideFace*' '*MeshConeContactBenchmarks*ObliqueRim*' --launchCount 2 --warmupCount 5 --iterationCount 15 --iterationTime 250 --affinity 3 --keepFiles --exporters json --artifacts artifacts/grv-benchmark-020/shared-reproduction
```

Final circle/slab reproduction uses the earlier full twelve-row command with
the same two-launch settings. For stage localization use the endpoint/oblique
filters with `--profiler EP --launchCount 1 --warmupCount 3 --iterationCount 8
--iterationTime 500`; never run that profiler concurrently with timing or owner
coverage. Check the actual generated child rather than only the launcher.
