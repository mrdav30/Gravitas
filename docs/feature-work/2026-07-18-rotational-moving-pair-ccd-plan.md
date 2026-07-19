# Rotational Moving-Pair CCD Hardening Plan

> **For agentic workers:** Use systematic debugging, test-driven development,
> verification before completion, and an independent final code review for
> every commit. Keep this document current as implementation evidence changes.

**Status:** In progress

**Goal:** Make 2D, 3D, and mixed rotational continuous collision a first-class
moving-pair contract. Pure rotation and combined translation/rotation must not
tunnel through dynamic or kinematic bodies, and rotational impacts must use
contact-point angular response plus bounded same-frame handoffs rather than
treating moving targets as static proxy spheres.

**Non-goals:** Do not replace the established body-owned bounded TOI lifecycle
with a global event solver. Do not add threading, floating-point arithmetic,
scene-graph ownership, engine dependencies, or per-frame allocations.

## Confirmed RCA

- 2D and 3D rotational candidate gathering uses static-target query surfaces.
- Pure rotation exits translational CCD before dynamic candidate indices and
  mixed routing run.
- Existing moving-target snapshots contain only start rotation; they cannot
  sample a target's angular trajectory independently of service order.
- Translational handoffs carry no pose-at-impact or angular-velocity delta, so
  they cannot continue rotation through the remaining frame.
- Existing CCD response denominators are linear-mass-only. Reusing them for a
  rotational hit would ignore `omega x r`, angular inertia, and pure-rotation
  closing speed.
- Moving kinematic bodies are absent from dynamic candidate indices and can be
  observed at different poses depending on service order.

## Invariants

1. Frame trajectories are captured before either dimensional body phase mutates
   pose. Sampling a target is independent of body/service processing order.
2. Candidate order is normalized time, target dimension, then stable collider
   ID. `Both` never admits cross-dimensional candidates.
3. Interval pruning bounds source and target linear travel, angular arc travel,
   and fixed-point uncertainty. Unrepresentable bounds subdivide
   conservatively; they never certify separation.
4. A witnessed contact uses contact-point relative velocity and constrained
   inverse mass/inertia. A conservative unwitnessed clamp never invents an
   impulse.
5. Dynamic impacts update both bodies atomically. A kinematic source follows its
   authored trajectory while the dynamic target receives a bounded handoff.
6. Handoffs continue both translation and rotation for the remaining time and
   encode all pending authoritative state in replay hashing.
7. Existing static/bodyless behavior, queue budgets, callback-abort cleanup,
   and warmed zero-allocation paths remain intact.
8. Prepared motion is a canonical piecewise trajectory, not one overwritten
   start/end pair. Same-frame impacts preserve the pre-impact history that
   later bodies must sample. Phase 1 retains every distinct increasing impact;
   the Phase 3 arbiter must reject a pair atomically at its deterministic
   mutation budget before making the final storage bound structural.
9. The frame-start candidate index is immutable after preparation. Impact-
   changed trajectories enter a service-owned, TOI-budget-bounded dirty
   overlay so later sources cannot miss a body accelerated outside its original
   swept bounds.

## Refined Architecture Decision

The Phase 0 RCA rejected grafting moving candidates onto the current
post-translation rotational pass. Translational and rotational hits must enter
one normalized-time arbiter; otherwise translation always wins by call order,
kinematic rotation cannot continue after pushing a target, and a handoff cannot
represent the remaining combined pose. The implementation therefore lands two
behavior-preserving pre-refactors first: order-independent piecewise motion
trajectories, then contact-point normal impulse kernels shared with discrete
response. Admission changes begin only after both contracts are green.

## Phase 0: Baseline And Contract Regressions

- [x] Record focused current 2D/3D rotational tests and benchmark baselines.
      The existing rotational slice passed 23/23 before source changes. The
      short static-near-miss baseline at candidate counts 1/8/32 measured
      1.599/8.609/32.471 ms in 3D and 0.609/3.032/11.082 ms in 2D, with zero
      managed allocation.
- [x] Add failing pure-rotation regressions for dynamic 2D and 3D targets.
- [x] Add failing kinematic-source regressions proving target wake/response and
      authored source completion.
- [ ] Add failing mixed 3D-to-2D and 2D-to-3D regressions plus `Both` exclusion.
- [ ] Add combined moving-target translation/rotation and equal-time ordering
      regressions.

## Phase 1: Order-Independent Frame Pose Trajectories

- [x] Extend 2D/3D CCD snapshots with final frame rotation, angular velocity,
      angular distance, and kinematic authored endpoints.
- [x] Prepare linear and angular integration once from the normal body policy
      before ordered processing; do not predict through a divergent duplicate
      or consume force/torque twice.
- [x] Represent prepared motion as pre-sized canonical piecewise trajectories
      and consume queued force/torque exactly once before ordered body
      processing. Equal- or earlier-fraction mutations replace obsolete tail
      pieces; distinct increasing impacts remain lossless until Phase 3 owns
      the mutation bound.
- [x] Include moving kinematic bodies in the bounded candidate indices.
- [ ] Exclude moving kinematics from duplicate static-query admission when the
      unified arbiter begins consuming the new index.
- [x] Add replay-cache hashing for every new trajectory field.
- [ ] Prove body registration/service order does not change sampled poses once
      moving-pair admission consumes the trajectories.

## Phase 2: Contact-Point CCD Impact Kernels

- [x] Extract allocation-free 2D, 3D, and mixed normal-impact math from the
      discrete response model: contact velocity, constrained inverse
      mass/inertia, restitution, and paired linear/angular deltas.
- [x] Keep response calculation side-effect-free until every finite delta is
      proven, then apply both participants atomically.
- [x] Cover frozen axes, infinite/zero effective mass, separating/tangent
      contact, pure angular closing speed, and mixed planar constraints.
      The focused 2D/3D/mixed response and warm-start slice passes 106 tests
      with zero allocations in the scalar-free kernels. A wider 3D fused
      inverse-inertia matrix product-sum/divide remains a lower-math-layer
      hardening candidate; the current extraction does not regress the
      discrete solver's established Q32.32 boundary.

## Phase 3: Rotational Handoff Continuation

- [ ] Extend 2D/3D handoffs with pose-at-impact and angular-velocity delta.
- [ ] Continue translation and rotation together for the remaining frame under
      the shared TOI budget, including requeue and already-processed targets.
- [ ] Preserve ignored-pair identity across same-dimensional and mixed chains.
- [ ] Extend authoritative and solver-cache replay hashing and abort/discard
      cleanup for the new state.

## Phase 4: Moving-Pair Interval Search And Arbitration

- [ ] Gather bodyless/frozen, dynamic, moving kinematic, and mixed candidates
      through bounded indices into one dimension-tagged stable ordering.
- [ ] Sample both participant poses at every interval midpoint and rebuild only
      the two runtime shapes under test.
- [ ] Sum both motion bounds for separation proofs while preserving the current
      fixed stack, depth limit, and node budget.
- [ ] Apply witnessed dynamic response/handoffs; retain static clamps and
      conservative no-impulse fallback for unresolved intervals.
- [ ] Continue source translation/rotation after an impact until time, motion,
      or the shared TOI budget is exhausted.

## Phase 5: Validation, Performance, And Closure

- [ ] Run focused 2D/3D/mixed/static/kinematic/dynamic handoff suites.
- [ ] Prove deterministic replay and registration-order invariance.
- [ ] Extend rotational CCD benchmarks with 1/8/32 same-dimensional dynamic
      targets and both mixed directions; require zero steady-state allocation.
- [ ] Run full `Release`, `ReleaseLean`, exact coverage, and CRAP analysis.
- [ ] Update the issue tracker and relevant collision/runtime wiki contracts.
- [ ] Obtain an independent final correctness/performance review.

## Commit Boundaries

1. Order-independent piecewise trajectory/lifecycle infrastructure with no
   admission change.
2. Shared 2D/3D/mixed contact-point normal impulse kernels with no admission
   change.
3. Rotational handoff continuation, dirty candidate overlay, and replay
   ownership.
4. Unified 2D/3D translational/rotational arbitration and moving-pair response.
5. Mixed admission, deterministic arbitration, benchmarks, and closure.

Each commit must build and pass its focused tests independently. The three
local-link project files remain unstaged validation scaffolding throughout.
