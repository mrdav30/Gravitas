# Continuous Collision Depth Hardening Plan

**Date:** 2026-06-18
**Status:** Active / Workstream 4 complete
**Owner:** Gravitas runtime/collision hardening

## Purpose

Phases 8, 8B, 8C, and 8D made continuous collision detection materially stronger
for alpha: deterministic dynamic-vs-dynamic CCD, mixed CCD, pure static-style
collectors, conservative non-sphere proxy policy, public-query visibility tests,
and benchmark guardrails are now in place.

This plan captures the deeper CCD work that remains important but should not be
implemented casually. These items affect physical correctness, precision,
benchmark quality, and solver architecture. Each item needs focused tests,
measured cost, and deterministic ordering before it graduates into runtime code.

## Current CCD Baseline

Current runtime CCD is frame-local:

- moving bodies prepare frame-start displacement before late integration mutates
  individual bodies.
- static-style CCD sweeps against bodyless, immovable, and kinematic targets.
- dynamic-vs-dynamic CCD uses relative motion against prepared dynamic target
  candidates.
- mixed CCD uses dedicated mixed query paths and partition refresh caching.
- dynamic 2D and 3D angular CCD uses conservative angular candidate bounds and
  deterministic bounded pose samples that are verified through the existing
  exact narrow-phase before a contact is accepted.
- 3D sphere and 2D circle movers use exact radius proxies.
- static-style 2D CCD gathers AABB, convex polygon, and compound movers with
  conservative bounds radii, then reduces supported candidates with exact
  translated-shape sweeps before accepting a hit.
- static-style 3D CCD gathers non-sphere movers with a conservative
  bounds-sphere proxy; sphere targets are then reduced through a reversed
  swept-sphere check against the moving source shape when supported by the
  existing 3D sweep worker.
- accepted translational hits advance to the earliest time of impact, remove the
  closing velocity component, and continue through a bounded number of remaining
  same-frame substeps before ordinary discrete response handles resting contact
  resolution.

That is a good deterministic release-boundary contract, but it does not yet claim
full shape-exact swept polytope support, kinematic-host active angular casts,
production benchmark gating, or a global continuous solver island model.

## Guiding Rules

- Preserve deterministic replay before improving precision.
- Add red/green tests before changing runtime CCD behavior.
- Measure before optimizing and preserve the baseline artifact path in this
  plan.
- Avoid hidden global cost. CCD should stay opt-in/auto and should scale with
  the number of CCD-capable movers and relevant candidates.
- Prefer shape-specific exact solvers only when they beat the conservative proxy
  on meaningful correctness without unacceptable hot-path cost.
- Do not introduce floating-point math, native dependencies, nondeterministic
  ordering, or background work into runtime CCD.
- If a change requires solver architecture rather than query/narrow-phase
  helpers, split it into its own phase instead of forcing it through a small CCD
  patch.

## Workstream 1: Rotational CCD

**Problem**

Current CCD treats frame motion as linear displacement of a swept proxy center.
Fast angular motion on long or thin shapes can tunnel even when the center
barely moves. Examples include a rotating blade, a long cuboid door swinging
through a small object, or a capsule/compound weapon turning around a pivot.

**Why It Matters**

Lockstep games often care about melee arcs, rotating hazards, vehicle parts,
doors, turrets, and authored compound bodies. A translational-only CCD contract
is still honest, but it leaves an important class of fast motion outside the
guarantee.

**Initial Scope**

- 2D angular CCD for circles, AABBs, convex polygons, and 2D compounds.
- 3D angular CCD for sphere, capsule, cuboid, cylinder, mesh, and compound
  proxies.
- Kinematic host rotation and dynamic angular velocity should both be considered
  as sources of angular swept motion.

**Research Questions**

- Should rotational CCD start as conservative swept angular bounds, exact
  shape-specific angular casts, or substep-based angular sampling?
- Can angular sweep bounds be built cheaply from existing fixed-point bounds and
  shape definitions without excessive false positives?
- How should tie-breaking work when translational and rotational hits occur at
  the same time of impact?
- Should compound colliders sweep each part independently or use an aggregate
  rotational proxy first and exact part reduction second?

**Candidate Approach**

Start with conservative angular swept bounds and tests that prove the current
translational CCD misses the scenario. Then compare:

1. conservative swept angular bounds plus existing exact static-style sweeps.
2. bounded deterministic substeps only for bodies whose angular displacement
   exceeds a threshold.
3. shape-specific angular sweeps for the most common cases: 2D convex polygon
   and 3D cuboid/capsule.

The likely first implementation should be conservative and opt-in through the
same `ContinuousCollisionMode` contract. Exact angular shape solvers should be
promoted only after benchmarks prove they are not worse than conservative
bounds in typical lockstep scenes.

**Tests To Add First**

- 2D rotating thin convex polygon sweeps through a circle while center
  translation is zero.
- 2D rotating AABB or compound part sweeps through a static wall with no center
  crossing.
- 3D rotating long cuboid sweeps through a sphere while center translation is
  zero.
- 3D rotating compound collider part sweeps through a kinematic target while
  the owning body center stays outside contact range.
- Deterministic tie test where linear and angular candidate hits have equal TOI.
- No-hit tests for near misses to quantify conservative false positives.

**Benchmark Targets**

- Angular CCD disabled baseline.
- Angular CCD enabled with no angular motion.
- Angular CCD enabled with sparse angular movers.
- Angular CCD enabled with dense angular movers.
- Compound angular CCD with many private parts.

**Implementation Notes - 2026-06-20**

Workstream 1 chose conservative angular candidate bounds plus bounded
deterministic pose samples as the first runtime model. This keeps the
implementation inside the existing opt-in `ContinuousCollisionMode` contract and
avoids shape-specific angular casts until benchmarks show a need for them.

- 2D and 3D dynamic angular sources gather static-style targets with a
  conservative angular radius, then sample up to a fixed maximum number of
  angular substeps. Each sampled pose rebuilds local runtime shape state and
  uses the existing narrow-phase before accepting a contact.
- Accepted rotational hits clamp to the previous safe sample and stop angular
  motion for the frame. Linear closing velocity is removed using the accepted
  contact normal so tangential motion can still be handled by the ordinary
  response path.
- Focused regressions now cover a rotating 2D thin polygon crossing a circle,
  a rotating 3D long cuboid crossing a sphere, no-hit angular near misses in
  both dimensions, and a rotated cuboid/sphere closest-point narrow-phase bug
  found while hardening the 3D case.
- 2D and 3D rotational CCD late-simulate paths now have allocation guardrails
  that require zero managed allocations after warmup.
- Benchmark rows now cover angular CCD with no angular motion, sparse angular
  movers, and dense angular movers for pure 2D and pure 3D CCD scaling.

Remaining work stays in later CCD phases: kinematic host rotation as an active
swept source, exact shape-specific angular time-of-impact solvers, explicit
compound-part angular benchmark scenes, and continuous solver-island handling
for multiple CCD impacts in one frame.

**Likely Files**

- `src/Gravitas/Core/StiffBody.cs`
- `src/Gravitas/Core/StiffBody2D.cs`
- `src/Gravitas/Queries/GravitasQuery3DService.*.cs`
- `src/Gravitas/Queries/GravitasQuery2DService.cs`
- `src/Gravitas/CollisionHandling/Detection/*`
- `tests/Gravitas.Tests/CollisionHandling/ContinuousCollisionDetectionTests.cs`
- `tests/Gravitas.Tests/Physics2D/ContinuousCollision2DTests.cs`
- `tests/Gravitas.Benchmarks/Core/DynamicCcdScalingBenchmarks.cs`

## Workstream 2: Shape-Exact Swept Mover Proxies

**Problem**

The current non-sphere/non-circle mover policy is conservative: use a
bounds-derived radius so wide or elongated shapes do not tunnel when their
wider portions cross a target away from the center path. This removes
false-negative risk, but it can stop early and report contacts that exact swept
shape tests would reject.

**Why It Matters**

Conservative early stops are acceptable for alpha safety, but they can feel bad
for fast large movers, long capsules, thin cuboids, compound weapons, and dense
mesh movers. Exact swept movers could improve precision and reduce unnecessary
velocity removal.

**Initial Scope**

Prioritize exact movers in this order:

1. 2D convex polygon / AABB against static-style 2D targets.
2. 3D capsule and cuboid against static-style primitive targets.
3. 2D and 3D compound part reduction using exact part sweeps where available.
4. Convex mesh movers against primitive/static-style convex targets.
5. Concave mesh movers only after mesh collision benchmarks show the runtime
   cost is controlled.

**Research Questions**

- Which exact swept pairs reduce false positives enough to justify their cost?
- Can exact swept mover support reuse existing static target query workers
  without duplicating shape math?
- Should exact mover support be selected by shape kind automatically, or exposed
  as an explicit precision mode?
- How should compound exact sweeps reduce part hits while preserving owner-level
  collider identity and deterministic ordering?

**Candidate Approach**

Keep the conservative bounds-radius proxy as the fallback for every unsupported
shape. Add shape-exact movers one pair family at a time behind tests and
benchmarks. Each exact solver must preserve:

- starting-overlap behavior.
- zero displacement rejection.
- layer, trigger, hierarchy, and self exclusion.
- hit sorting by distance then collider ID.
- deterministic normal orientation for closing velocity removal.
- allocation-free behavior after warmup.

**Tests To Add First**

- Wide cuboid false-positive case that currently stops early but should pass
  under an exact cuboid mover.
- Capsule end-cap and side-sweep cases where a sphere proxy is too wide.
- 2D convex polygon corner miss that the bounds-radius proxy over-reports.
- Compound owner hit reduction where two parts have different TOIs.
- Mesh mover fixture proving the fallback remains conservative until exact mesh
  mover support is intentionally added.

**Benchmark Targets**

- Exact mover vs proxy mover for sparse no-hit scenes.
- Exact mover vs proxy mover for dense hit scenes.
- False-positive-heavy scenes where exact movers should reduce response work.
- Compound part count scaling.
- Mesh mover query cost if mesh exact support is attempted.

**Implementation Notes - 2026-06-20**

Workstream 2 kept conservative proxy sweeps as the candidate-gathering stage and
added exact candidate reduction only where it materially removes false positives
without changing the public CCD mode contract.

- Pure 2D static-style CCD now refines every candidate with exact translated
  mover sweeps. Circles reuse existing swept-circle logic, convex/AABB movers
  use deterministic swept SAT against convex targets, convex movers against
  circles use the existing circle sweep in reverse, and 2D compounds reduce
  through private parts while preserving owner-level target identity.
- Pure 3D static-style CCD now refines sphere targets by sweeping the target
  sphere backward against the moving source collider with the existing
  `SweptSphereQueryWorker`. This gives exact false-positive rejection for
  cuboid, capsule, cylinder, mesh, and compound movers against static-style
  sphere targets without duplicating 3D shape math.
- Unsupported 3D target shapes, mixed CCD, and dynamic-vs-dynamic CCD still use
  the conservative proxy path. That fallback is intentional until exact
  relative-motion and mixed-shape reducers have tests and benchmark evidence.
- Focused regressions now cover 2D thin polygon/AABB false positives, a 2D
  compound aggregate-radius false positive, true 2D polygon hits, 3D thin
  cuboid/capsule false positives against spheres, true 3D cuboid hits, and
  allocation-free 2D/3D exact translational CCD after warmup.
- Benchmark rows now cover false-positive-heavy 2D and 3D shape-exact CCD
  scenes through `DynamicCcdScalingBenchmarks`.

Remaining work stays in later CCD phases: exact 3D reducers against non-sphere
primitive targets, exact dynamic-vs-dynamic shape reducers, mixed-dimension
shape-exact reducers, and mesh-specific production benchmark decisions.

**Likely Files**

- `src/Gravitas/Core/StiffBody.cs`
- `src/Gravitas/Core/StiffBody2D.cs`
- `src/Gravitas/Queries/GravitasQuery3DService.Raycast.cs`
- `src/Gravitas/Queries/GravitasQuery3DService.Circle.cs`
- `src/Gravitas/Queries/GravitasQuery2DService.cs`
- `src/Gravitas/CollisionHandling/Detection/*`
- `src/Gravitas/Colliders/Primitives/*`
- `src/Gravitas/Colliders/Primitives2D/*`
- `tests/Gravitas.Tests/CollisionHandling/ContinuousCollisionDetectionTests.cs`
- `tests/Gravitas.Tests/Physics2D/ContinuousCollision2DTests.cs`
- `tests/Gravitas.Benchmarks/Core/DynamicCcdScalingBenchmarks.cs`

## Workstream 3: Production-Grade CCD Benchmark Signal

**Problem**

The current CCD benchmark rows are valuable regression guardrails, but several
Phase 8D smoke rows produced BenchmarkDotNet short-iteration warnings. That is
fine for exploratory hardening, but not strong enough for long-term performance
claims.

**Why It Matters**

CCD is expensive by nature. Future contributors need benchmark rows that can
distinguish real improvements from noise, especially in sparse/dense,
mixed/pure, and many-CCD-body scenes.

**Initial Scope**

Build a heavier CCD benchmark selection that complements the short smoke rows:

- pure 3D dynamic CCD scaling.
- pure 2D dynamic CCD scaling.
- mixed CCD scaling.
- static-style query attribution.
- dynamic candidate-index attribution.
- exact sweep attribution.
- response handoff / movement clamp attribution.
- no-hit, first-hit, dense-hit, and false-positive-heavy scenes.

**Research Questions**

- What operations-per-invoke or manual batching removes the short-iteration
  warnings without making local runs unbearable?
- Which benchmark selection should CI run versus which should remain manual?
- Should benchmark output include custom counters for attempted static sweeps,
  attempted dynamic candidates, accepted CCD hits, and proxy false positives?
- Should a synthetic deterministic scene generator live in benchmarks so CCD
  scaling fixtures do not drift from test fixtures?

**Candidate Approach**

Keep the current `DynamicCcdScalingBenchmarks` rows for quick regression
checks. Add a separate heavier benchmark mode or parameter set for serious CCD
performance evidence. Prefer explicit benchmark methods over reflection-heavy
or runtime-configured scenarios so benchmark names remain stable in artifacts.

**Tests To Add First**

This workstream is primarily benchmark infrastructure, but it should include
one focused determinism test for any reusable benchmark scene generator:

- same seed/settings produce identical body positions, collider IDs, and layer
  assignments across repeated construction.

**Benchmark Targets**

- No BenchmarkDotNet `MinIterationTime` warning for the primary CCD evidence
  rows.
- Zero managed allocations after warmup for runtime CCD loops that should be
  allocation-free.
- Separate rows for pure 2D, pure 3D, and mixed so regressions are attributable.
- Size parameters that include small gameplay scenes and large lockstep scenes.

**Implementation Notes - 2026-06-21**

Workstream 3 now separates quick CCD scaling smoke rows from heavier manual CCD
evidence rows. `DynamicCcdScalingBenchmarks` remains the fast regression
selection, while `ContinuousCollisionEvidenceBenchmarks` provides primary
performance evidence with larger size parameters and `OperationsPerInvoke = 64`.

- Added a shared deterministic CCD benchmark layout under
  `tests/Gravitas.SharedBenchmarkSupport` and linked it into both the benchmark
  and test projects. A focused test now verifies repeated descriptor generation
  produces stable positions, collider ordinals, and layer assignments.
- Extracted reusable CCD benchmark context creation, body creation, reset, sum,
  static-query attribution, dynamic candidate-index attribution, and relative
  sweep helpers into benchmark support files. The dynamic CCD scaling benchmark
  now uses a fixture instead of owning duplicate setup/query plumbing.
- Added production evidence rows for pure 3D, pure 2D, mixed full-runtime CCD,
  static query attribution, dynamic candidate-index attribution, dynamic
  relative sweep attribution, and shape-exact false-positive scenes.
- A full short in-process evidence smoke completed all 28 rows without the old
  `MinIterationTime` warning. Full-runtime 3D and mixed rows intentionally
  include benchmark reset/host-transform publish cost; the attribution rows
  isolate the CCD query/index/sweep paths and are the cleaner allocation signal.
- Removed the unused copied `BenchmarkScenarioFactory` helper and simplified the
  benchmark alias catalog by deleting an empty qualifier hook.
- CI benchmark execution remains intentionally deferred. For now these rows are
  manual evidence runs while repo-wide benchmark publishing/gating is still
  being evaluated.

Remaining work stays outside this workstream: benchmark publishing/gating,
external baseline storage/comparison tooling, and host-visible CCD counters if
the engine later needs runtime diagnostics rather than benchmark-only
attribution.

**Likely Files**

- `tests/Gravitas.Benchmarks/Core/DynamicCcdScalingBenchmarks.cs`
- `tests/Gravitas.Benchmarks/Core/ContinuousCollisionEvidenceBenchmarks.cs`
- `tests/Gravitas.Benchmarks/Support/*`
- `tests/Gravitas.Tests/Support/*` if scene generation is shared with tests.
- `docs/wiki/DIAGNOSTICS.md` if CCD counters become host-visible diagnostics.

## Workstream 4: Continuous TOI Solver And Substep Island Model

**Problem**

Current CCD clamps the moving body to the earliest accepted hit and removes the
closing component of velocity. That is deterministic and simple, but it is not a
full continuous solver. Multiple contacts in one frame, chained fast bodies,
sliding across several CCD impacts, and angular/linear combined impacts may need
a more integrated time-of-impact model.

**Why It Matters**

Large lockstep simulations may have many player-controlled or important dynamic
objects moving quickly at once. A deeper CCD solver could reduce artifacts such
as over-clamping, missed secondary contacts after the first TOI, or solver
order dependence in dense high-speed scenes.

**Initial Scope**

Start with 2D and 3D pure dynamic islands before mixed islands:

- collect CCD candidates for all bodies.
- find earliest TOI per island or per body pair.
- advance affected bodies to the TOI.
- resolve or clamp contacts in deterministic order.
- optionally continue with bounded remaining frame time.

Mixed islands should be evaluated after pure islands because mixed response has
additional plane-constrained impulse rules.

**Research Questions**

- Is a bounded substep model sufficient, or does Gravitas need explicit
  continuous contact islands?
- What deterministic limit prevents pathological frames from exploding in cost?
- How should sleeping, kinematic, immovable, and bodyless participants affect
  island construction?
- Can the existing collision-pair and response layers consume TOI contacts, or
  does this require a separate CCD manifold path?
- How should body center-of-mass and future full inertia tensor work integrate
  with CCD response?

**Candidate Approach**

Treat this as a solver architecture phase, not a query optimization. Begin with
tests that demonstrate current first-hit clamp limitations. Prototype a bounded
substep island model in tests/benchmarks before changing the default runtime
path. Promote only if it improves correctness without unacceptable allocation or
time-complexity growth.

The implementation path should strengthen `Continuous` itself rather than
adding a weaker legacy/advanced split. `Continuous` should be the first-class
bounded TOI solver contract, while `Auto` should use the same solver after it
decides the frame displacement needs CCD.

**Tests To Add First**

- Fast body hits two static targets in one frame and should slide/continue to
  the second target under the advanced model.
- Two fast dynamic bodies hit each other, then one immediately hits an immovable
  target in the remaining frame time.
- Dense same-TOI candidates resolve in stable collider-ID order.
- Mixed 3D/2D candidate produces the same result across repeated runs once mixed
  islands are attempted.
- Bounded-iteration guard test proves pathological scenes stop deterministically
  with an explicit status/counter.

**Benchmark Targets**

- Current first-hit clamp baseline.
- One-substep, two-substep, and bounded-N-substep variants.
- Sparse no-hit advanced solver overhead.
- Dense many-hit island overhead.
- Mixed island overhead after pure islands are stable.

**Likely Files**

- `src/Gravitas/Core/StiffBody.cs`
- `src/Gravitas/Core/StiffBody2D.cs`
- `src/Gravitas/Core/GravitasPhysicsService.cs`
- `src/Gravitas/Core/GravitasPhysics2DService.cs`
- `src/Gravitas/Core/GravitasMixedCollisionService.cs`
- `src/Gravitas/Settings/PhysicsSettings.cs`
- `src/Gravitas/Settings/PhysicsSettingsSaver.cs`
- `src/Gravitas/CollisionHandling/Response/*`
- `src/Gravitas/CollisionHandling/Pairs/*`
- `tests/Gravitas.Tests/CollisionHandling/ContinuousCollisionDetectionTests.cs`
- `tests/Gravitas.Tests/Physics2D/ContinuousCollision2DTests.cs`
- `tests/Gravitas.Tests/MixedDimensions/MixedQueryCcdTests.cs`
- `tests/Gravitas.Tests/Settings/*`
- `tests/Gravitas.Tests/Serialization/PhysicsSettingsSerializationTests.cs`
- `tests/Gravitas.Benchmarks/Core/DynamicCcdScalingBenchmarks.cs`
- `tests/Gravitas.Benchmarks/Core/ContinuousCollisionSubstepBenchmarks.cs`

**Implementation Notes - 2026-06-21**

Workstream 4 upgraded `Continuous` and `Auto` from single-hit clamping to a
bounded body-owned TOI substep solver for pure 2D, pure 3D, and the existing
mixed candidate comparison path.

- `PhysicsSettings.ContinuousCollisionMaxSubsteps` now controls the deterministic
  same-frame impact budget, with
  `PhysicsSettings.DefaultContinuousCollisionMaxSubsteps` defaulting to `4`.
- On each accepted translational hit, the body advances to the TOI, removes only
  the closing component of linear velocity, consumes that portion of frame time,
  and continues sweeping the remaining segment with the updated velocity.
- Dynamic target prediction is segment-aware: later substeps sample target
  frame-start displacement at the elapsed frame fraction and sweep only through
  the remaining frame fraction.
- Exact mover refinement now samples the moving collider's runtime shape at the
  intermediate substep start before evaluating the next sweep. This prevents
  convex/AABB/compound mover reduction from accidentally using frame-start shape
  state after an earlier hit.
- `StiffBody.LastContinuousCollisionSubstepCount`,
  `StiffBody.LastContinuousCollisionSubstepLimitReached`,
  `StiffBody2D.LastContinuousCollisionSubstepCount`, and
  `StiffBody2D.LastContinuousCollisionSubstepLimitReached` expose deterministic
  last-step solver status for diagnostics and tests.
- Focused 2D and 3D regressions now cover same-frame two-contact sliding,
  bounded-limit reporting, and zero-allocation steady-state substep paths. 2D
  also covers intermediate-shape sampling for non-circle movers.
- `ContinuousCollisionSubstepBenchmarks` adds two-contact pure 2D and pure 3D
  rows across `1`, `2`, and `4` max-substep settings, preserving the first-hit
  clamp shape as measurable evidence without keeping it as the default runtime
  behavior.
- A short in-process substep benchmark smoke completed all 12 new rows. The run
  is useful as setup validation, not canonical timing evidence. It also repeated
  the already-tracked 3D full-runtime allocation shape, so allocation RCA stays
  in the benchmark-signal backlog rather than broadening this workstream.

Remaining work is narrower than the original research question: global
service-level CCD island solving, exact dynamic mesh/compound relative TOI,
kinematic bodies as active swept sources, and mixed-specific island response
remain future hardening work if benchmark evidence or game scenarios justify
the added complexity.

## Recommended Order

1. Production-grade CCD benchmark signal.
2. Shape-exact swept mover proxies for the highest false-positive cases.
3. Rotational CCD conservative bounds and proof tests.
4. Continuous TOI solver / substep island model.

This order keeps evidence quality ahead of deeper implementation work. Shape
precision and rotational coverage are meaningful only if the benchmark suite can
show their cost clearly. The continuous solver model should come last because it
can absorb decisions from exact movers, rotational CCD, body mass/inertia work,
and future center-of-mass modeling.

## Promotion Criteria

A workstream can graduate into an active implementation phase when it has:

- failing tests that prove a real current limitation.
- a baseline benchmark artifact captured before runtime changes.
- a deterministic algorithm sketch with explicit tie-breakers.
- an allocation plan for hot-path buffers.
- updated docs that explain the new CCD contract and any remaining conservative
  behavior.
- full `Release` and `ReleaseLean` validation after implementation.

## Current Recommendation

The core CCD hardening workstreams in this plan are complete. Do not reopen CCD
for another broad implementation phase unless benchmark evidence or a concrete
game scenario requires global islands, exact dynamic mesh/compound relative TOI,
or mixed-specific continuous response before release. Smaller measured concerns
should be tracked in
`docs/feature-work/2026-06-21-benchmark-signal-hardening-backlog-plan.md`.
