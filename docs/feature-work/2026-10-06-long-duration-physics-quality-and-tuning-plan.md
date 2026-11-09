# Long-Duration Physics Quality And Tuning Plan

**Date:** 2026-10-06  
**Status:** Phase 1 in progress. The free-stack and tilted-capsule baseline is verified.  
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

- [ ] Add or extend freely simulated stacks without hidden axis locks or joints
      that manufacture stability. Include ordinary primitives and an explicit
      tilted/curved-feature stress case. Extend long chains and resting/contact
      ragdolls from existing fixtures instead of rebuilding them.
- [ ] Cover equal and varied mass ratios, initially 1:1, 1:10 and 1:100, with
      fixed documented lengths/masses and an admitted fixed-point range. Cover
      pure 2D and 3D equivalents and focused mixed-response parity where the
      constrained model applies; do not demand 3D behavior from planar bodies.
- [ ] Measure maximum/residual penetration, positional/angular drift, joint
      anchor/limit error, contact identity churn and sleep/wake transitions over
      time. Report units and normalize geometric errors to declared shape scale.
- [ ] Track kinetic/potential energy and work from gravity, impulses and motors
      with the fixture's friction, restitution and damping policy. Conservation
      is a gate only in an appropriate conservative fixture; resting friction,
      damping and stabilization need their own explainable behavior bounds.
- [ ] Keep physical comparisons deterministic and overflow-safe. Reuse neutral
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
five other cases as stress evidence. Before expanding to long duration,
isolate the high-mass support failure using matched alignment and load
controls. The chain, ragdoll, mixed parity, and controlled work checkpoints
remain in Phase 1.

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
