# Long-Duration Physics Quality And Tuning Plan

**Date:** 2026-10-06  
**Status:** Phase 1 complete. Phase 2 duration and continuation is next.  
**Owner:** Gravitas contact/constraint quality and configuration guidance.

## Goal

Publish how existing settings trade physical quality for cost over meaningful
simulation durations. Reuse current deterministic contact/island/constraint
owners; add solver behavior or public tuning only when evidence establishes a
specific limitation. This is characterization and hardening, not a presumption
that the current solver needs replacement.

## Existing Foundation

The completed
[discrete-response plan](done/2026-06-21-discrete-response-and-contact-quality-hardening-plan.md)
and
[3D stress plan](done/2026-07-02-3d-constraint-solver-stress-and-tuning-hardening-plan.md)
cover warm starts, islands, resting contacts, chain/ragdoll stress and
deterministic joint metrics. Reuse
[`Constraint3DStressTests`](../../tests/Gravitas.Tests/Constraints/Constraint3DStressTests.cs),
the dimensional constraint tests, sleep tests and
[`Constraint3DBenchmarks`](../../tests/Gravitas.Benchmarks/Core/Constraint3DBenchmarks.cs).
The [capacity/soak plan](done/2026-10-06-evolving-game-capacity-and-soak-plan.md)
owns frame-time/memory collection; share its measurement contract rather than
add a second runner or generic physics-quality framework.

## Phase 1: Physical Fixtures And Metrics

- [x] Add or extend freely simulated stacks without hidden axis locks or joints
      that manufacture stability. Include ordinary primitives and an explicit
      tilted/curved-feature stress case. Extend long chains and resting/contact
      ragdolls from existing fixtures instead of rebuilding them.
- [x] Cover equal and varied mass ratios, initially 1:1, 1:10 and 1:100, with
      fixed documented lengths/masses and an admitted fixed-point range. Cover
      pure 2D and 3D equivalents and focused mixed-response parity where the
      constrained model applies; do not demand 3D behavior from planar bodies.
- [x] Measure maximum/residual penetration, positional/angular drift, joint
      anchor/limit error, contact identity churn and sleep/wake transitions over
      time. Report units and normalize geometric errors to declared shape scale.
- [x] Track kinetic/potential energy and work from gravity, impulses and motors
      with the fixture's friction, restitution and damping policy. Conservation
      is a gate only in an appropriate conservative fixture; resting friction,
      damping and stabilization need their own explainable behavior bounds.
- [x] Keep physical comparisons deterministic and overflow-safe. Reuse neutral
      upstream math when needed; do not move rigid-body policy into
      FixedMathSharp. Wall-clock measurements and chart formatting remain
      outside physics logic.

Each fixture declares its expected physical behavior and numeric acceptance
bounds before an optimization is accepted. Baseline observations help choose
scale-aware bounds; an unstable baseline does not become correct by definition.

### Free-Stack And Tilted-Capsule Checkpoint

The opt-in [quality verifier](../../tests/Gravitas.Benchmarks/README.md#opt-in-physics-quality-stacks)
runs eight small fixtures on the released LSF packages. The first capture
used Gravitas `3f1a83fb` with the benchmark changes below, .NET SDK 10.0.302
and runtime 10.0.10 on Windows x64. Each fixture advances 256 complete steps
at 60 Hz (4.27 simulated seconds) with six discrete solver iterations.
The 2D fixtures set gravity on each body in the simulation plane. The 3D
fixtures use world gravity. Both use magnitude 9.8 units/s², manual
grounding so collision response supplies support, and zero restitution
with static/dynamic friction of 1/2 and 1/3. Active grid axes use 4-unit
cells. Sleep is enabled. The fixtures advance under gravity and contact
response without host impulses, motors, joints, or axis locks.

The ordinary stack consists of two 2-by-1 rotatable convex polygons in 2D
or 2-by-1-by-2 cuboids in 3D. The floor is 16 units across, 1 unit thick,
with its top at height zero. The lower body has unit mass and starts
1/32 unit above the floor. The upper body starts 1/32 unit above the lower
body with a lateral offset of 1/64 unit. Its mass is 1, 10, or 100. The
curved fixture drops a unit-mass capsule of radius 1/2 and total height 2,
tilted 15 degrees from upright, with its center at height 9/8. All shapes
have room to move and rotate. The reference length is 1 unit, the box
height or capsule diameter.

Each case uses two fresh observed contexts and one observation-free control.
The observed runs hash state and measurements every frame. The control
matches terminal state. The verifier requires active grid coverage,
bounded sample values, and observation of the intended support pairs.
The provisional quality bounds are independent of replay. The COM must
stay above the floor plane, final residual depth must be at most 1/16 of
the reference length, and every original support pair must still touch.
The stack's final lateral drift must be at most 1/8 of the reference
length and its rotation change at most 5 degrees. The capsule must turn
at least 5 degrees during the run and finish within 2 reference lengths
laterally. Report failures as baseline observations.

Read live manifold depths as retained solver inputs. Resample geometry
after publication to measure residual overlap using detached 3D work items
and the shared-depth 2D contact query for these shapes. Count original
neighbors separately from all observed overlaps. Contact churn counts
frames whose pair-scoped sorted manifold IDs changed. A changing row ID
can occur while the bodies stay in contact.

| Fixture | Peak / final residual (units) | Final lateral drift (units) | Final angle change (degrees) | Lowest COM (units) | Final original supports | Bounds |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| 2D stack 1:1 | 0.064349 / 0.022688 | 0.053052 | 0.1099 | 0.453711 | 2 / 2 | Within |
| 2D stack 1:10 | 0.508188 / 0.012991 | 0.373486 | 0.4314 | 0.021710 | 1 / 2 | Outside |
| 2D stack 1:100 | 0.518161 / 0.012749 | 0.330707 | 0.6742 | 0.022959 | 1 / 2 | Outside |
| 2D tilted capsule | 0.010000 / 0.010000 | 1.031789 | 75.0722 | 0.490006 | 1 / 1 | Within |
| 3D stack 1:1 | 0.091235 / 0.069350 | 0.234716 | 19.0428 | 0.449891 | 2 / 2 | Outside |
| 3D stack 1:10 | 0.554031 / 0.453783 | 2.237423 | 32.0414 | 0.018583 | 2 / 2 | Outside |
| 3D stack 1:100 | 0.569897 / 0.506129 | 0.154365 | 15.8677 | −0.035170 | 2 / 2 | Outside |
| 3D tilted capsule | 0.014457 / 0.010934 | 1.032760 | 75.0388 | 0.489014 | 1 / 1 | Within |

All stack fixtures first showed both original supports on frame 7. The
heavy planar stacks had two overlapping pairs at the end, with only one
of their original neighbors still touching. In the 3D 1:100 case a COM
first crossed the floor plane at frame 29, the peak-residual frame, and
was at or below the plane in 94 sampled frames. This is evidence of deep
overlap in the fixture. The COM check does not establish a complete
passage through the floor. None of the cases transitioned to sleep over
the captured window. Manifold IDs changed on 247–252 of 256 frames,
including fixtures that retained their expected support topology.

Translational kinetic energy uses mass and COM velocity. Rotational energy
uses the planar moment or 3D local inertia with world angular velocity
rotated into body space. Potential energy uses COM height relative to the
floor. Output includes final kinetic components, potential energy decrease,
and total mechanical energy change. Positional correction and dissipative
contact policy affect this balance. A signed work ledger and conservation
control remain separate measurements.

Release and ReleaseLean matched all eight trace and terminal digests.
The optional CSV capture matched those digests and recorded frames 0–256
for each fixture. Default BenchmarkDotNet discovery excludes the verifier.
Raw captures are in ignored `artifacts/physics-quality-phase1`. The two
tilted capsules and 2D equal-mass stack met the initial bounds. Retain the
five other cases as stress evidence. The matched controls below isolate the
high-mass support failure. The chain, ragdoll, mixed parity, and controlled
work checkpoints remain in Phase 1.

### 3D High-Mass Support Controls

The [opt-in support verifier](../../tests/Gravitas.Benchmarks/README.md#opt-in-physics-quality-stacks)
reuses the 3D cuboid stack, step settings, and replay checks from the
checkpoint above. This capture used `d7ffa17b` plus the support verifier,
.NET SDK 10.0.302, and runtime 10.0.10 on Windows x64 with released LSF
packages. Each lower/upper mass combination runs with the original 1/64-unit
upper X offset and with both boxes centered on the same vertical axis. The
100/1 swap preserves total mass and gravity load. The 100/100 control
preserves the upper mass and raises the total mass from 101 to 200. The
1/1 case is the original equal-light comparison. Positions, gaps, and
solver settings stay fixed within each alignment.

At each completed step, detached narrow-phase probes record overlap and
maximum residual depth separately for floor/lower, lower/upper, and
floor/upper. The live pair sample counts manifold rows stamped with the
current frame and the subset whose normal points upward toward the higher
box in the original stack. These counts measure contact generation. The
solver can skip rows, and pair processing can refresh a manifold after
response, so these counts do not measure applied impulse.

| Lower / upper mass | Upper X offset | First floor/lower residual > 1/16 | First direct floor/upper overlap | First lower COM ≤ 0 | Minimum lower COM (units) | Peak floor/lower residual (units) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 / 100 | 1/64 | 11 | 29 | 29 | −0.035170 | 0.569897 |
| 1 / 100 | 0 | 12 | 29 | 29 | −0.040109 | 0.559860 |
| 100 / 1 | 1/64 | — | — | — | 0.489545 | 0.013801 |
| 100 / 1 | 0 | — | — | — | 0.488809 | 0.013894 |
| 100 / 100 | 1/64 | 57 | — | — | 0.449891 | 0.063323 |
| 100 / 100 | 0 | 84 | — | — | 0.442539 | 0.068811 |
| 1 / 1 | 1/64 | 57 | — | — | 0.449891 | 0.063323 |
| 1 / 1 | 0 | 84 | — | — | 0.442539 | 0.068811 |

Every case first showed upward manifold rows at the floor on frame 5 and
at both original interfaces on frame 7. The floor manifold went from four
upward rows on frame 5 to one on frame 6 in every case, before inter-box
contact. On frame 7 the lower box in the 1/100 cases was already moving
downward at 0.559092 (offset) or 0.480323 (centered) units/s. Its floor
residual was still 0.011480 units in both. Both original interfaces first
exceeded 1/16 unit together at frame 11 (offset) or 12 (centered). On frame
29, about 0.48 seconds in, the lower COM crossed the plane and the upper
box first overlapped the floor directly. Each 1/100 case recorded 94
samples with the lower COM at or below zero. Both original interfaces
retained fresh upward manifold rows on every frame from 7 through 256.

In this fixture, severe compression required the heavy box above the light
one. Centering did not prevent it, reversing the same total load avoided it,
and keeping the heavy upper box while strengthening the lower box avoided
it even with a larger total load. Both equal-mass cases still exceeded the
residual threshold later. Contact generation was present during the failure.
The [response trace](#3d-support-response-on-frames-712) below separates
velocity changes from positional correction during the onset window.

All eight cases matched two independent state/observation traces, exact
per-frame support samples within each configuration, and terminal hashes
from observation-free runs. Release and ReleaseLean produced identical
printed CSV and hashes. The offset 1/100 case reproduced the prior trace
`ec4e508ec695a2e0a07088629c41be44` and terminal
`b492487c1f6acea7800655f5b5ddce32`. The original eight-fixture verifier
output was unchanged in both configurations. Raw support captures are in
ignored `artifacts/physics-quality-phase1/support-*.csv.txt`.

### 3D Support Response On Frames 7–12

Reused the eight cases above against released LSF packages on Windows x64
(.NET SDK 10.0.302 and runtime 10.0.10). The runtime was `d7ffa17b` with
read-only probes applied to a disposable copy of the support-control tree.
Run `verify-quality-stack-support` with `GRAVITAS_QUALITY_FRAME_CSV=1` for
the clean baseline. In the disposable copy, sample both endpoints' raw
Q32.32 `Position3d.Y` and `LinearVelocity.Y` in the internal
`CollisionResponse.CalculateImpulse` overload at entry, after the first-pass
correction/warm-start loop, and after the incremental normal and friction
loop. Log pair IDs, frame, and first-pass flag. Compare the two observed
runs and capture published COM Y and vertical velocity on frames 6–12 to
reconcile the pair deltas with completed steps. The cuboids' COM is at the
body origin. The observer-free control runs with the probes disabled.

Each frame had six passes in floor/lower then lower/upper order, with
position correction on the first pass only. Across all cases the raw
per-pass deltas reconciled exactly with the published position and velocity.
The inter-box velocity changes summed to zero vertical momentum at the
reported precision. The warm-start loop produced zero net vertical velocity
change in this window. The measured velocity response came from the
incremental normal and friction loop, which this probe measured together.

The table sums frames 7–12. Corrections are signed Y displacements in
world units. Floor momentum is the lower mass times its net floor-pair
vertical velocity change. The percentage compares that momentum with the
magnitude lost to gravity by both boxes over the same six steps.

| Lower / upper mass | Upper X offset | Floor correction to lower | Inter-box correction to lower | Net correction to lower | Lower integration | Floor momentum / gravity loss |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 / 100 | 1/64 | +0.202231 | −0.170460 | +0.031771 | −0.093681 | 1.32% |
| 1 / 100 | 0 | +0.176994 | −0.162497 | +0.014497 | −0.077801 | 1.81% |
| 100 / 1 | 1/64 | +0.044032 | −0.000798 | +0.043234 | −0.042605 | 79.51% |
| 100 / 1 | 0 | +0.044200 | −0.000725 | +0.043475 | −0.042698 | 79.53% |
| 100 / 100 | 1/64 | +0.111500 | −0.059454 | +0.052046 | −0.066246 | 50.22% |
| 100 / 100 | 0 | +0.101010 | −0.055213 | +0.045797 | −0.060428 | 58.48% |
| 1 / 1 | 1/64 | +0.111500 | −0.059454 | +0.052046 | −0.066246 | 50.22% |
| 1 / 1 | 0 | +0.101010 | −0.055213 | +0.045797 | −0.060428 | 58.48% |

The velocity shortfall was visible on frame 7, before the first inter-box
position correction. With the offset, the floor added 0.026171 units/s to
the lower box while the upper box drove its velocity down by 0.287130 units/s.
When centered, the floor's net contribution was zero and the inter-box
response drove the lower box down by 0.182191 units/s. Over the six frames,
the two 1/100 cases gained just 1.306699 (offset) or 1.786828 (centered)
mass·units/s from the floor against 98.980000 lost to gravity. The upper
box received only +0.001705 (offset) or +0.001625 (centered) units of
inter-box position correction as inverse-mass weighting placed almost all
of the separation on the lower box. The lower box still received net
upward correction in both cases, but integrated downward by more. This
establishes an early velocity support deficit and shows how correction is
distributed. The floor-row trace below isolates the initial mechanism and
tracks the remaining runtime work under
[GRV-Issue-151](issue-tracker.md#grv-issue-151).

Release and ReleaseLean produced identical traces and measurements. Each
of the eight cases matched both instrumented repeats exactly, including
72 pair calls and seven published checkpoints per repeat. Removing probe
lines reproduced the full clean support-control output, including hashes
and all 257 frame samples per case. The existing verifier also checked
terminal replay with observation disabled. Ignored
`artifacts/physics-quality-phase1/support-response-probe` contains the
probe patch, raw captures, and per-frame reduction. The maintained change
is this investigation record.

#### Frame 7 Floor Contact And The Mass Swap

At `7d350b2f`, a disposable extension of the response probe traced frame 7
in all eight support cases. Read the actual solver lever, normal, contact
point speed before and after each normal and friction pass, effective-mass
terms, and accumulated normal impulse from `CollisionResponse.CalculateImpulse`.
For the floor pair, transform all eight lower-box corners at the admitted
rotation, translate by the corrected body position, and sample each corner's
vertical speed from `v + ω × r` immediately after the floor visit. The floor
top is at Y = 0 and all four bottom corners project inside its X/Z bounds.

The centered 1/100 and 100/1 runs chose the same single floor row. Its normal
was +Y, depth was 0.017399 units, and the lower-box lever was approximately
`(0.999511, −0.501975, 0.999500)`. The first pre-solve point speed was
−0.416945 units/s in both runs to six decimal places. The normal solve took
that speed to zero. The angular term contributed 4.795251 of the 5.795251
inverse effective mass for the light lower box. With lower mass 100 the
denominator scaled down to 0.057953 and the first normal impulse scaled up
100-fold, giving the lower boxes the same initial vertical velocity change.
The floor friction contributed zero direct vertical velocity change, but
changed the point speed through angular response.

| Floor pass | 1/100 point speed before normal (units/s) | 1/100 accumulated normal impulse (mass·units/s) | 100/1 point speed before normal (units/s) | 100/1 accumulated normal impulse (mass·units/s) |
| --- | ---: | ---: | ---: | ---: |
| 1 | −0.416945 | 0.071946 | −0.416945 | 7.194589 |
| 2 | +0.358931 | 0.010011 | −0.043724 | 7.949064 |
| 3 | +0.188343 | 0.000000 | +0.004684 | 7.868232 |
| 6 | +0.171068 | 0.000000 | −0.000088 | 7.858762 |

The first inter-box normal impulse was about 0.12646 mass·units/s in both
cases. It reduced the light lower box's vertical speed by 0.12646 units/s
and the heavy lower box's by 0.001265 units/s. In the centered 1/100 case,
rotation made the sampled floor corner separate by pass 2 while the lower
COM still moved downward. By pass 3 the unilateral solve had removed the
floor impulse. At that point, all four bottom corners still penetrated the
floor after correction. The selected corner moved upward at 0.136012 units/s,
and the other three moved downward at 1.075515, 0.469850, and 0.469654
units/s. The 1/64-offset 1/100 case retained a small net floor impulse on
frame 7. Complete removal is specific to the centered trace.

The oriented cuboid path supplies one support witness after any nonidentity
rotation. The trace shows an uncovered, closing part of the floor contact
patch while the sampled row behaves as a unilateral constraint should. A
disposable floor-only experiment added penetrating bottom corners under a
near-horizontal face, using the same SAT admission and existing response.
It kept the lower COM above zero in both 1/100 alignments, with minimum
heights of 0.052501 units offset and 0.071714 centered. Direct upper-floor
overlap still began on frame 28 in both, one frame earlier than baseline.
This isolates the value of additional floor contacts while leaving the
support chain failure open. The experiment only covers this floor setup.

Each read-only trace matched its independent repeat exactly for 12 solver
row visits and 48 corner samples per case. Release and ReleaseLean matched
bit for bit, and stripping the probe records reproduced the clean support
output with all replay hashes and published frame samples. The floor-only
experiment also replayed within each configuration and matched across them.
Its probes and experiment are under ignored
`artifacts/physics-quality-phase1/issue-151-probe`. The maintained runtime
is unchanged.

#### GRV-Issue-151 Fix Experiments

Started at `4b0d83af` against the released stack on Windows x64 (.NET SDK
10.0.302 and runtime 10.0.10). A frozen test uses a 1/16-degree tilt about
two axes, with the incident cuboid centered at Y = 0.49 over a flat face.
It checks both pair orders, matched witnesses, per-point depth, and finite
reference bounds. The wide face contains all four incident corners. The
2-unit face with a half-unit offset requires side clipping. Stock geometry
emits one row in each case where four rows are expected. The rounded
prototype passed the wide case. The clipped case reached the witness
checks and its reported depth differed from the materialized witness
separation by one Q32.32 unit.

The disposable face prototype preserves SAT admission and the one-witness
fallback. It chooses a reference face by rounded normal alignment, clips the
incident rectangle against the four finite side planes and the face plane
in bounded local coordinates, and reuses the existing four-row manifold
reduction. This is a rounded prototype. Its stable-ID variant uses the
clipping boundaries and face identities for rows produced by the prototype.
The unchanged axis-aligned and fallback paths still use their own IDs.

Each variant ran the eight-case `verify-quality-stack-support` workload.
The table lists the 1/100 cases only. Each cell lists offset then centered.
The lower COM minimum and first direct upper-floor overlap are the
comparison gates because the detached depth probe also uses the changed
manifold generator. A dash means the COM stayed above zero for all 256 steps.

| Disposable variant | First direct upper-floor overlap | First lower COM ≤ 0 |
| --- | ---: | ---: |
| Stock, 6 passes | 29 / 29 | 29 / 29 |
| Stock, 48 passes | 29 / 29 | 30 / 30 |
| Clipped faces at both interfaces, 6 passes | 31 / 32 | 33 / 33 |
| Clipped faces and stable patch IDs, 6 passes | 32 / 30 | — / 44 |
| Clipped faces and stable patch IDs, 48 passes | 91 / 83 | — / — |
| Stable patch IDs and island-wide warm start, 6 passes | 33 / 31 | — / — |
| Same, plus six refreshed position sweeps | 39 / 34 | — / — |
| Same, with the supported lower body held fixed for inter-box correction | 55 / 100 | — / — |

At six passes the stable-ID prototype supplied four upward rows at both
interfaces on frame 7 in both heavy-top cases. It still lost the stack.
The last variant recomputed the residual from endpoint translations between
position sweeps and let the upper box absorb the inter-box correction while
the two-interface upward support arrangement held. The minimum lower COM
was 0.489545 in both alignments, yet the upper box still reached the floor.
The six matched controls in that variant never reached direct floor contact
with their upper box. These results isolate a remaining coupled support
problem beyond the initial missing floor rows. The experimental
position policy is specific to this fixture's topology and pair order.

For a frozen normal-only comparison, capture frame 7 of the two centered
mass swaps from the stable-ID face prototype before warm starting. The
capture provides four normals and levers at each interface, both bodies'
velocities, inverse masses and inverse inertia tensors, and the compatible
cached normal impulses. The fixture uses free axes and zero restitution.
Reconstruct `K = J M⁻¹ Jᵀ` and `q = J v` from the raw values.
Run sequential projected passes seeded with the cached impulses in the
captured row order. Independently enumerate active sets for the eight
unilateral constraints and require nonnegative impulse and normal speed
with complementary products zero to the analysis tolerance. The analysis
uses high-precision decimal arithmetic outside the runtime. It holds the
geometry fixed and isolates normal response from friction, position
correction, and later frames.

| Lower / upper mass | Normal solve | Minimum floor point speed (units/s) | Total floor normal impulse (mass·units/s) |
| --- | --- | ---: | ---: |
| 1 / 100 | 6 projected passes | −1.928190 | 3.295213 |
| 1 / 100 | 48 projected passes | −1.199451 | 24.412916 |
| 1 / 100 | Coupled reference | 0.000000 | 75.473201 |
| 100 / 1 | 6 projected passes | −0.000159 | 17.043354 |
| 100 / 1 | 48 projected passes | 0.000000 | 17.078979 |
| 100 / 1 | Coupled reference | 0.000000 | 17.078979 |

The coupled reference leaves no closing normal speed at either interface
to the reported precision. With the same candidate rows, sequential
projection transfers the heavy top load too slowly in this frozen case.
Both eight-row systems have rank six. Use feasible contact velocities and
aggregate response as the comparison targets, since redundant rows can
admit more than one impulse distribution. Full-step stability, friction
coupling, fixed-point range, and runtime cost remain separate gates.

The verifier matched independent repeats and observation-free terminal
hashes for every variant. The frozen capture and the final pose variant
matched byte for byte across Release and ReleaseLean. Fresh stock runs in
both configurations matched the prior clean six-pass captures. The maintained
cuboid geometry and discrete island test filters passed 21 cases in each
configuration. Ignored `artifacts/physics-quality-phase1/issue-151-fix`
contains the frozen test, probe source, CSVs, and reference reduction. The
next implementation needed finite matched witnesses and a bounded coupled
solve or preconditioner. Phase 1 expansion stayed on hold during this
investigation.

#### GRV-Issue-151 Implementation

Registered rotated cuboids now retain finite face patches after exact SAT
admission and a face-normal match. Each contact retains matched local
witnesses and an individual depth. Clipping features keep their identities
when penetration changes. The aligned fast path and the existing
single-witness fallback remain available.

The 3D response can seed a two-contact, joint-free cuboid island when a
freely moving cuboid carries a heavier one over a fixed support. The
bounded normal solve handles redundant rows and checks the prepared body
velocities before publication. The row sweeps refine friction after the
coupled seed, using cached tangents only when their basis matches. When
each interface has one normal direction, the two deepest depths also
drive a coupled translation correction. Unsupported or unrepresentable
normal solves keep the sequential path. Unsupported translation solves
keep the existing per-pair position correction.

The stock frozen geometry regression produced one row where four finite
rows were expected. The frozen heavy-load response left the lower body
closing at 0.942045 units/s after six sequential passes. The maintained
regressions cover the clipped and full faces, reversed order, the coupled
normal solution, warmed allocations, and a representability fallback that
preserves both bodies and their caches. A complete host-step regression
exercises the rotating heavy-top stack through the original onset window.

The opt-in `verify-quality-stack-support` now fails if either original
support interface disappears after establishment, if either residual
exceeds 1/16 unit, or if the upper box overlaps the floor or the lower
COM reaches its plane. This keeps the specialized 256-step matrix separate
from the base test suite.

All eight cases retained both supports from frame 7 through frame 256.
The two 100:1 cases had zero direct upper-floor overlaps and zero lower COM
crossings. The original 3D 1:1 and 1:10 fixtures now meet their provisional
quality bounds. The 3D 1:100 offset fixture still drifts 0.572327 units by
the end, above the separate 1/8-unit drift bound. Its final orientation
change is 1.0925 degrees. Lateral drift remains a Phase 1 quality signal.

| 100:1 alignment | Minimum lower COM | Peak floor/lower depth | Peak lower/upper depth | Final original supports |
| --- | ---: | ---: | ---: | ---: |
| 1/64-unit offset | 0.489330 | 0.011098 | 0.011964 | 2 / 2 |
| Centered | 0.489372 | 0.013735 | 0.014527 | 2 / 2 |

The support and original quality captures matched across Release and
ReleaseLean against released packages. Each case compared fresh observed
replays and an observation-free terminal hash. The 2D and tilted capsule
digests match the original capture. The two 100:1 terminal digests are
`b4cce723bd2bd44084ccedfa44c71409` (offset) and
`5409c5048ccaa8d56357ada24b3b4d18` (centered).
The 1,024-actor 3D strategy verifier also retained its 96-frame trace
`d5e9d21184fbb529b3b3b83e86f2b6b1` and frame 67 chain completion in both
configurations.

An isolated warm probe disabled sleep in both the stock and corrected
fixtures. It warmed 64 steps and measured only `Simulate` plus
`LateSimulate` for frames 65–256, checking upward rows at both interfaces
outside the measured window. All 192 steps in each of the eight cases
had active support rows and allocated 0 B on the stepping thread, in
Release and ReleaseLean. The Release timing sequence alternated three
stock and three corrected process launches in `A B B A A B` order. The
medians of launch means were 1.412 → 1.533 ms for the offset 100:1 stack
and 0.466 → 0.955 ms for the centered one. The six matched controls
measured 0.351–0.790 ms stock and 0.822–1.995 ms corrected. The contact
geometry and physical trajectories differ between revisions, so these are
scenario costs rather than equivalent-work kernel timings. The isolated
step cost and the remaining attribution work are tracked under
[GRV-Benchmark-034](benchmark-signal-hardening-backlog.md#grv-benchmark-034).

The package-backed library built for `net10.0` and `netstandard2.1` in both
configurations. The complete suites and strict coverage gate passed on the
same runtime snapshot. The resolved [issue record](issue-tracker.md#grv-issue-151)
links back to these measurements.

| Configuration | Passing tests | Covered lines | Covered branches | Covered methods |
| --- | ---: | ---: | ---: | ---: |
| Release | 8,393 | 54,012 / 54,012 | 22,080 / 22,080 | 5,634 / 5,634 |
| ReleaseLean | 8,322 | 54,010 / 54,010 | 22,080 / 22,080 | 5,633 / 5,633 |

[GRV-Benchmark-034](benchmark-signal-hardening-backlog.md#grv-benchmark-034)
closed after reducing the exact response cost of sustained cuboid support.
The [density closeout](done/2026-10-31-benchmark-signal-closeouts.md#shared-frame-lever-reduction-and-closure)
preserves the four-, eight-, and sixteen-stack replay and budget evidence.
[GRV-Benchmark-035](benchmark-signal-hardening-backlog.md#grv-benchmark-035)
closed after rejecting disjoint cross-stack pairs before acquisition.

### Current Stack Baseline And Planar Support Follow-Up

At `50c5d555` with released packages, `verify-quality-stacks` matched between
Release and ReleaseLean. The 3D equal-mass and 10:1 top-loaded cases now meet
the initial bounds. The 100:1 offset case retains both supports, with peak
residual depth 0.011964 unit and lowest COM 0.489330 unit. Its final lateral
drift is 0.572327 unit, above the provisional 1/8-unit bound. All eight
`verify-quality-stack-support` controls meet their separate support bounds.
The two tilted capsules and the equal-mass planar stack remain within their
initial bounds.

The offset 2D 10:1 and 100:1 stacks still lose the lower link's floor
support. A disposable probe reused the same planar setup and sampled each
pair's published-pose depth separately. It changed only the lower and upper
masses and the upper body's lateral offset (1/64 unit or zero). Fresh
upward rows first reached the floor on frame 5 and both support interfaces
on frame 8 in all ten cases. These row timestamps differ from the first
geometric overlaps sampled after each step.

| Lower / upper mass | Alignment | First lower-floor depth > 1/16 | Peak lower-floor depth | First upper-floor overlap with positive depth | Lowest lower COM |
| --- | --- | ---: | ---: | ---: | ---: |
| 1 / 10 | Offset | 12 | 0.480540 | 65 | 0.021710 |
| 1 / 100 | Offset | 12 | 0.499387 | 29 | 0.022959 |
| 1 / 10 | Centered | 28 | 0.081449 | — | 0.418551 |
| 1 / 100 | Centered | 12 | 0.502097 | 34 | −0.002097 |
| 100 / 1 | Offset / centered | — | 0.010154 / 0.010112 | — | 0.489888 / 0.489888 |
| 100 / 100 | Offset / centered | — | 0.049561 / 0.016754 | — | 0.453711 / 0.483246 |
| 1 / 1 | Offset / centered | — | 0.049561 / 0.016754 | — | 0.453711 / 0.483246 |

The lower COM of the centered 100:1 case first reached the floor plane on
frame 34 and was at or below it on six sampled frames. The same-total-mass
swap stayed within the floor-depth bound at both alignments. This isolates
the support loss to top-heavy loading in these fixtures. It leaves the
specific response cause open. [GRV-Issue-152](issue-tracker.md#grv-issue-152)
owns the pure 2D investigation. Keep the passing 3D support results and the
planar failure distinct in later duration and solver comparisons.

### Aligned Long-Chain Checkpoint

On `1839af2e` with the new opt-in `verify-quality-chains` command, use
released LSF packages on Windows x64 (.NET SDK 10.0.302, runtime 10.0.10).
Each dimension runs 256 complete steps at 60 Hz and six discrete solver
iterations. The verifier reuses the long-chain benchmark builders with
32 centered, radius-1/2 circles or spheres spaced one unit apart. Half-unit
local anchors coincide initially. The first link is static. The remaining
links move freely through unrestricted planar pins or spatial ball sockets.
Adjacent collision is suppressed, and the verifier checks for zero physical
contacts throughout. Gravity, motors, aerodynamic drag, angular damping and
sleep are disabled for this checkpoint. The spatial pulse lies in X/Y and
the planar equivalent uses the simulation's X/Z axes.

Non-tip links have unit mass. The matched-speed cases give the tip an
impulse of its mass times `(3/16, 1/16)`, yielding the same initial velocity
to fixed-point precision. The mass-100 control receives the unit-mass case's
impulse, so its initial velocity is 1/100 as large. The no-pulse control uses
unit mass throughout. Frame 0 verifies coincident anchors and the initial
kinetic energy against the energy supplied by the authored impulse. Gravity
and motor work are zero. Kinetic energy includes body-local rotational
energy. The matched inputs leave each dimension's inertia and solver policy
in effect. Constraint stabilization can change mechanical energy, so this
checkpoint uses an energy growth bound and measures decay separately.

The no-pulse control must remain motionless with coincident anchors. Loaded
cases use provisional bounds of 1/16 unit for peak anchor separation, 1/64
unit for terminal separation, and one unit for peak tip displacement. These
are lengths relative to the one-unit link diameter. Kinetic energy must stay
within twice the input energy. The verifier reconstructs anchor separation
from published poses and checks each joint's latest solve metrics for the
prepared gap and two planar or three spatial rows per step. Frame samples
also report maximum link rotation in degrees and linear/angular energy in
mass × distance²/s².

| Tip load | 2D peak / final gap | 3D peak / final gap | 2D / 3D peak tip travel | Bounds |
| --- | ---: | ---: | ---: | --- |
| No pulse, unit mass | 0 / 0 | 0 / 0 | 0 / 0 | Within |
| 1, matched speed | 0.003294 / 0.001441 | 0.003294 / 0.001446 | 0.218119 / 0.223470 | Within |
| 10, matched speed | 0.003835 / 0.003710 | 0.003835 / 0.003711 | 0.260301 / 0.260868 | Within |
| 100, matched speed | 0.018635 / 0.018635 | 0.018643 / 0.018643 | 0.601408 / 0.601458 | Outside final gap |
| 100, matched impulse | 0.000185 / 0.000185 | 0.000185 / 0.000185 | 0.006054 / 0.006046 | Within |

Peak energy stayed at the frame-0 input in all ten cases. The mass-100
matched-speed cases began at 1.953125 and finished at 0.558412 (2D) and
0.558567 (3D). Their final frames were also their peak gap frames. The
matched-impulse controls show that mass ratio alone is insufficient to
reproduce the bound miss. Carry the matched-speed row into the equal-duration
continuation and iteration comparisons to measure whether the gap subsides
and how much additional solve work improves it.

Each case matched two observed runs and the terminal hash of an
observation-free run. Both Release and ReleaseLean matched across launches.
Prepared row totals were 15,872 per planar case and 23,808 per spatial case.
The 192 warm steps in frames 65–256 allocated 0 B in nine cases. The spatial
mass-100 matched-speed case allocated 4,536 B on frame 227. A disposable
partition probe measured 1,080 B in partition rental and 3,456 B in voxel
membership, with no pair acquisitions or joint-solver allocation in that
frame. This is first-use growth of the kind recorded in the
[#035 closeout](done/2026-10-31-benchmark-signal-closeouts.md#grv-benchmark-035).
Keep the measured burst visible when extending the workload. Captures are
under ignored `artifacts/physics-quality-phase1/chains`.

The focused constraint suites passed 90 Release and 86 ReleaseLean tests.
Both existing long-chain BenchmarkDotNet rows completed a Dry smoke run.

### Resting And Contact Ragdolls

`verify-quality-ragdolls` reuses the benchmark's six-link torso and limbs
in both dimensions. The torso is mass 3 with width 1 and height 3/2, and
the spatial cuboid is 1/2 unit deep. The planar torso uses a rotating
polygon. The five limbs are mass 1 circles or spheres of radius 1/2. The
24-unit floor is one unit thick with its top at −1/2. Joints start with
coincident midpoint anchors. The planar pins have ±10° limits and the
spatial joints have 10° cone and twist limits. Linked neighbors suppress
collision and each link remains free to translate and rotate.

The drop uses the original benchmark centers. The supported-start control
places the links directly on the floor at X coordinates `(0, 3, −2, 2,
−1, 1)` in torso-to-limb order and registers the midpoint anchors there.
Both start still and use 256 complete steps at 60 Hz, six iterations,
gravity 10 units/s², static/dynamic friction 1/2 and 1/3, zero restitution,
and zero aerodynamic drag and angular damping. The supported-start case
uses 1/20 unit/s and 1/20 rad/s sleep speed thresholds with the existing
16-step quiet window. Both apply a 3/16 angular impulse to the last link
before step 193. There are no motors.

The verifier records retained solver depth across all pairs, pair-scoped
contact ID changes, and retained upward floor pairs. It measures
floor-normal penetration from the published shape bounds while each link
remains inside the floor footprint. It separately reconstructs the
published anchor gap and reads the latest prepared anchor and angular
limit error. The one-unit limb diameter is the length scale. The
provisional bounds are 1/8 peak and 1/16 final floor depth, 1/4 peak and
1/16 final anchor gap, 10° peak and 3° final prepared limit error, and one
unit peak lateral drift. The supported-start case also expects all six
links to sleep before the impulse, wake on the kick step, and return to
sleep by the final frame.

| Case | Peak / final floor depth | Peak / final anchor gap | Peak / final prepared limit error | Final floor pairs | Asleep before kick / awake after / asleep at end |
| --- | ---: | ---: | ---: | ---: | ---: |
| 2D drop | 0.010000 / 0.010000 | 0.790916 / 0.790916 | 7.8408° / 6.9186° | 4 | 0 / 6 / 0 |
| 3D drop | 0.011283 / 0.010862 | 0.837215 / 0.837215 | 7.2683° / 6.4339° | 4 | 0 / 6 / 0 |
| 2D supported start | 0.010000 / 0.010000 | 0.015389 / 0.015031 | 0° / 0° | 6 | 6 / 6 / 2 |
| 3D supported start | 0.010696 / 0.010696 | 0.018521 / 0.016577 | 0° / 0° | 6 | 6 / 6 / 5 |

Both drops retain upward floor pairs from frame 13 through frame 256 and
exceed the joint-gap and final prepared-limit bounds. Neither sleeps.
The supported-start cases meet the geometric bounds and show connected
wake on frame 193. They miss the all-links resettled bound at frame 256.
They log 26 sleep and 24 wake transitions in 2D, and 29 sleep and 24 wake
transitions in 3D. Contact IDs change on 244 drop frames and 81
supported-start frames in each dimension. A changed row ID can occur
while a pair continues to provide support.

The energy samples track translational and rotational kinetic energy,
potential energy relative to the floor, signed gravity work as potential
decrease, and work from the angular impulse as the instantaneous kinetic
change of the kicked link. In the supported-start cases, impulse work is
0.140625 in 2D and 0.175781 in 3D. Both finish below their initial
mechanical energy with friction, inelastic contact, and joint stabilization
in effect. The drop and supported-start replays match their repeats and
the terminal state of the observation-free control. Across frames 65–256,
the drop cases allocate 3,440 B in 2D and 7,704 B in 3D. The supported-start
cases allocate 312 B in 2D and 0 B in 3D. The largest individual step
allocations are 1,912 B on 2D drop frame 146, 3,608 B on 3D drop frame 124,
and 312 B on 2D supported-start frame 194.

### Focused Mixed-Response Parity

`verify-quality-mixed` runs four centered, frictionless impacts for 16
complete steps at 60 Hz. The moving radius-1/2 circle or sphere begins
7/8 unit from the target center, with normal speed 1/4 unit/s. The dynamic
participants have unit mass, zero restitution, and no gravity, drag,
angular damping or sleep. Coordinates and speeds stay below two units
and two units/s. The one-unit combined reach provides an independent
centerline penetration measurement.

- The side impact compares a mixed sphere against an embedded circle
  with two pure 2D circles. Both cases share the initial momentum equally,
  giving each body normal speed 0.125 unit/s after the first solve.
- The vertical impact compares the sphere over a dynamic embedded 4-by-4
  planar box of half-thickness 1/2 with the same sphere over a bodyless
  4-by-1-by-4 spatial cuboid. Both spheres stop. The embedded body keeps
  its exact planar position, velocity, rotation, spin and embedding height.

Both mixed impacts report the intended dimensional pair, the expected
axis normal and one applied impulse on frame 1. The pure controls each
produce one fresh contact row. The shared-speed or stop tolerance is
1/128 unit/s, and the terminal residual depth bound is 1/32 unit.
All four finish at 0.010000 unit residual. Peak depth after position
publication is 0.033833 unit in both mixed cases and the pure 3D control,
and 0.010000 unit in pure 2D. This comparison establishes the expected
velocity response under the embedding rule. The dimensions use their own
positional correction policy.

Repeated traces and terminal hashes match, including an observation-free
control for each case. Release and ReleaseLean produce identical results.

### Controlled Work, Energy, And Angular Limits

`verify-quality-work-energy` advances each case for 120 complete steps at
60 Hz. It uses radius-1/2, mass-2 circles or spheres, frictionless material,
manual grounding, and disabled sleep, drag and environmental damping.
Position and velocity magnitudes stay below 4 units and 4 units/s in the
translation cases. Rotors stay at their fixed joint center with angular
speed magnitude below 4 rad/s and angle magnitude below one radian.
The fixtures check for contact-free motion. Length and time are in
simulation units and seconds, and work and energy use mass × length²/s².

- The conservation control applies a central linear impulse of magnitude 1
  to the body at rest, then coasts in zero gravity. The expected input
  work `J²/(2m)` and measured kinetic energy both equal 0.25 exactly at
  every sample. Position follows the fixed-step linear trajectory.
- The gravity control applies the same impulse from height 2 with gravity
  1 unit/s². It compares signed gravity work `m g (h₀ − h)` with the kinetic
  energy gain after accounting for semi-implicit Euler integration. The
  integrator contribution is `−m (g Δt)² n/2` after `n` steps. Both dimensions
  end at kinetic energy 4.250000, potential energy −0.033333, and gravity
  work +4.033333. The observed discrete energy change is −0.033333 and the
  trajectory and work residuals stay within 1/4096 in their respective units.
- The centered rotor uses one solve iteration to isolate a single motor row
  with target 1/2 radian, strength 4, damping 1/2 and impulse cap 1/64. The
  stationary anchor supplies no angular work. The verifier compares the
  measured momentum change with the row's impulse magnitude, assigns its
  direction from the change, and multiplies by midpoint angular speed.
  It sums energy supplied and energy removed separately. The cumulative
  signed work tracks kinetic energy within 1/4096. The final angle must
  stay within 1/16 radian of the target.
- The passive limit case disables the motor, starts the centered rotor at
  3/4 radian, and imposes a ±1/2-radian limit. It requires a correcting
  first-step spin, checks the prepared 1/4-radian violation, and samples
  the published angle independently. Peak violation must stay within the
  initial 1/4 radian plus rounding tolerance. The terminal stress bound
  is 3/16 radian. The reported residual still exceeds the authored limit.

| Dimension | Motor positive / negative work | Final motor angle (rad) | Peak / final passive violation (rad) |
| --- | ---: | ---: | ---: |
| 2D | +0.183803 / −0.183600 | 0.500410 | 0.250000 / 0.150833 |
| 3D | +0.179049 / −0.179035 | 0.499861 | 0.250000 / 0.168028 |

These energy checks isolate their work inputs. Stack, chain, and ragdoll
curves include contact, stabilization, and any configured damping or
friction in their stated budgets.

### Phase 1 Verification

Validated against `50c5d555` plus the Phase 1 fixture additions with released
packages on Windows x64 using SDK 10.0.302 and runtime 10.0.10. Release and
ReleaseLean match for `verify-quality-stacks`, `verify-quality-stack-support`,
`verify-quality-chains`, `verify-quality-ragdolls`, `verify-quality-mixed`,
and `verify-quality-work-energy`. The stack, 3D support, and chain captures
also match the pre-change outputs. The new commands are opt-in. The existing
resting ragdoll BenchmarkDotNet rows both completed a Dry smoke run.

The full suites passed 8,403 Release and 8,332 ReleaseLean tests. Each
configuration reached 53,967 / 53,967 lines, 22,100 / 22,100 branches, and
5,635 / 5,635 methods with the repository coverage filters. The solution
build covers `netstandard2.1` and `net10.0`. The raw replay, frame samples,
and coverage reports are in ignored `artifacts/physics-quality-phase1/closeout`.

Phase 1 completes the fixture and measurement checklist. Carry the
100:1 matched-speed chain and the ragdoll drop and supported-start cases
into duration and iteration comparisons with the same physical bounds.
The planar support failure remains active under
[GRV-Issue-152](issue-tracker.md#grv-issue-152).

## Phase 2: Duration And Continuation

- [ ] Measure settling, sustained rest and repeated controlled disturbances over
      equal simulated durations at different frame rates. Begin with 32 and 60
      Hz; use a shorter regression plus an extended local characterization, not
      a huge soak in every unit test.
- [ ] Exercise repeated sleeping/waking, connected wake propagation, runtime
      mass changes between complete steps and deterministic spawn/despawn.
      Verify the scene still contains the intended dynamic bodies and contacts.
- [ ] Compare repeated command traces, then supported save/populate continuation
      into host-created matching shells and a matching clock timeline.
- [ ] Respect the [serialization contract](../api/guides/SERIALIZATION.md): body
      records do not restore a world, arbitrary host/coroutine state or solver
      caches. Compare exact traces only where continuation state is equivalent.
      If population invalidates caches, use an equivalently cold-cache control
      and separately measure the recovery transient against uninterrupted
      simulation. Do not disguise a genuine supported-contract divergence as a
      quality tolerance.

Exit: long-duration quality curves, repeatable disturbances and supported
continuation evidence. Record failures as focused issues with reproducing tests;
repair their owning layer before calling an affected configuration acceptable.

## Phase 3: Quality Versus Cost

- [ ] Compare current default solver settings with a lower/higher iteration
      setting, initially 2/6/12 iterations where supported. Add tuning
      dimensions only when a fixture identifies a meaningful sensitivity.
      Retain the #151 support bounds when assessing lower iteration counts.
- [ ] Resolve the eight- and sixteen-stack awake heavy-bottom budget limits
      from the [density closeout](done/2026-10-31-benchmark-signal-closeouts.md#shared-frame-lever-reduction-and-closure).
      Keep the four-stack control and compare each setting against the six-
      iteration baseline. Check support and per-setting replay, and report
      complete physics-step p95, maximum, and 60 Hz overruns across repeated
      launches. For each density, identify a setting that meets the support
      bounds and the host's reserved physics budget, or record the remaining
      capacity limit and the next solver change to evaluate.
- [ ] Report quality and measured complete-step time together at each setting,
      including awake population, contact/island counts and relevant CCD work.
      Changing frame rate requires equal simulated duration, not equal frame
      count.
- [ ] Separate contact and joint metrics; distinguish physical effects from
      partition/query cost and presentation interpolation. Keep engine adapters
      outside the experiment.
- [ ] Retain a solver change only with a reproduced physical improvement,
      deterministic ordering/rounding, bounded resource behavior, benchmark
      comparison and no unexplained regression in other dimensions/features.

Exit: a small set of measured, usable configurations or a scoped proposal for
the demonstrated solver limitation. Do not add named public presets, compliance
APIs, clamping or alternate penetration kernels solely to fill a tuning table.

## Completion And Publication

- [ ] Publish reproducible settings and quality/time curves, limitations and
      recommended usage boundaries. Feed lasting guidance into the relevant wiki
      pages without plan links, capture paths or dated implementation history.
- [ ] Preserve current public contracts unless an intentional redesign is
      justified. Follow normal ownership and dependency release order if an
      upstream change is required.
- [ ] Run focused quality/replay/allocation/resource checks, full Release/Lean
      suites, both target-framework builds and exact 100% reachable line/branch/
      method coverage in every changed library. Validate unreleased siblings
      with MSBuild and inherited `UseLocalLsfStack=true`; validate released
      packages before release.
- [ ] Archive completed evidence in `done/`; retain only concrete unresolved
      issues or explicitly evidence-gated follow-ups in coordination documents.

Offline mass-property tooling and richer diagnostic/publication platforms retain
their existing separate plans. This work does not activate those deferred
features without a measured need.
