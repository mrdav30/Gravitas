# Full-Lifecycle Replay Conformance Plan

**Date:** 2026-10-06  
**Status:** Phase 1 complete; shared fixtures and platform comparison pending.  
**Owner:** Gravitas test harness and CI; lower-stack owners only for reproduced defects.

## Goal

Prove that the same ordered commands produce the same authoritative state through
the complete host loop, across supported runtime modes, package profiles and
native CPU platforms. This strengthens evidence for lockstep hosts without
changing physics merely to preserve existing test output.

## Verified Baseline Before Phase 1

- [`ReplayConformanceHarness`](../../tests/Gravitas.Tests/Determinism/ReplayConformanceHarness.cs)
  advanced only `LateSimulate()` in both `RunTrace` and `AssertNextFramesMatch`.
  Commands precede the late phase, and hashes follow it.
- [`GravitasReplayConformanceTests`](../../tests/Gravitas.Tests/Determinism/GravitasReplayConformanceTests.cs)
  already exercises both phases in its wide-clock test. Existing scenarios cover
  3D CCD, mesh/compound, pure 2D, mixed queries and body restore continuation.
- [`Constraint3DStressTests`](../../tests/Gravitas.Tests/Constraints/Constraint3DStressTests.cs)
  supplied `Simulate()` through `beforeFrame`; changing the default without
  migrating these callers would have advanced twice.
- [`build-and-test.yml`](../../.github/workflows/build-and-test.yml) runs
  Windows/Linux and Release/ReleaseLean suites, but has no explicit architecture
  matrix or shared cross-job trace comparison.

The confirmed defect is incomplete default test-harness lifecycle coverage.
It does not demonstrate a runtime determinism failure. Repeated agreement within
one process cannot establish cross-platform agreement or detect two equally
incorrect runs.

## Phase 1: Complete Default Stepping

- [x] Add a regression proving command application, one `Simulate()`, one
      `LateSimulate()`, then the post-step hash occur in that order.
- [x] Assert clock progression, ordered lifecycle hooks and a coroutine with an
      observable authoritative effect. Hash equality alone cannot validate
      arbitrary host state that the context does not hash.
- [x] Fix both default helpers and audit every caller, including restore warmup
      helpers and callbacks that currently supply the missing phase. Preserve
      deliberately phase-specific tests under an explicit name where necessary;
      avoid a general configurable phase scheduler.
- [x] Exercise `TwoD`, `ThreeD`, `Both` and `Mixed` with actual motion/contact
      preconditions, not empty-clock traces. Verify that visualization does not
      change authoritative results.

Exit: default conformance follows the documented host loop exactly once per
frame; the regression fails against the baseline helper and migrated callers do
not double-step. Existing lifecycle/CCD/constraint tests remain valid.

Completed 2026-10-06. Both helpers, restore warmup and the separate repeated
3D hash trace now follow the full loop; stress callbacks no longer double-step.
Six regressions fail against the baseline helper and pass with the change.
Full Release/Lean coverage remains exact 100%; no runtime or lower-stack source
change was required. The [Phase 1 report](done/2026-10-06-replay-conformance-phase-1-report.md)
records validation and the independently reproduced Debug-only allocation
signal [GRV-Benchmark-025](benchmark-signal-hardening-backlog.md#grv-benchmark-025--debug-ragdoll-steady-state-allocations).
Shared expected fixtures and native cross-platform comparisons remain pending.

## Phase 2: Shared Command And Result Fixtures

- [ ] Reuse current scenario builders and ChronicleHash. Define a small,
      versioned fixture manifest containing ordered commands, frame count,
      settings, stable host entity identity and expected per-frame hashes.
- [ ] Cover sleep/wake, spawn/despawn and registration reuse, queries, CCD,
      connected contacts/constraints and supported restore continuation with a
      few focused traces. Assert the named contact/CCD event actually occurs;
      a one-shot force and hash agreement alone may never reach the target.
      Reuse these fixtures in subsequent workload plans.
- [ ] Check every frame against a reviewed shared expectation as well as a
      repeated run. Include semantic assertions for events and representative
      raw state so the hash producer is not the sole correctness oracle.
- [ ] Use `Authoritative` as the mandatory portable comparison. Compare
      `AuthoritativeWithSolverCaches` separately under identical fixture/cache
      contracts; do not mix the two modes or compare unsupported restore caches.
- [ ] Report the first divergent frame, preceding commands, raw body state and
      relevant ordered events. Keep machine/runtime/dependency metadata outside
      the equality payload. Never regenerate expected values automatically on
      failure; investigate intentional changes and review fixture versions.

Body payloads do not restore the containing clock, object graph, coroutines or
all solver caches. Restore fixtures must reconstruct supported shells and match
the host timeline under the [serialization contract](../wiki/SERIALIZATION.md).
This plan does not introduce a live-world rewind API.

## Phase 3: Platform Comparison

- [ ] First publish and compare the same fixture results between existing
      Windows/Linux Release/Lean lanes. Separate independently passing unit
      tests from an actual shared-result comparison.
- [ ] Inventory native x64 and ARM64 runner availability and record OS, actual
      architecture, SDK/runtime, source revision and dependency identity.
      Add native ARM64 lanes where hosting is approved and available. Emulation
      or compilation alone is not native execution evidence.
- [ ] Require expected traces and direct cross-lane comparison to agree for
      identical supported fixtures. Missing lanes remain visible evidence gaps,
      never passes. Add a tested comparator failure case using one altered frame.
- [ ] Keep small conformance traces in normal CI; retain artifacts for diagnosis.
      Build both library target frameworks without claiming that a
      `netstandard2.1` compile is another execution platform.

The complete native matrix is Windows/Linux x64/ARM64, each in Release/Lean.
Runner availability can stage its rollout; closure must name the actually
executed matrix and any remaining lanes rather than claim universal CPU proof.
Fixture authoring for the next plan can begin once Phase 1 and the shared trace
format are established; unavailable ARM hosting need not block that authoring.

## Validation And Completion

- [x] Run focused replay/lifecycle/coroutine/restore tests, full Release/Lean
      suites, both target-framework builds and exact 100% reachable line,
      branch and method coverage for every changed owning library.
- [x] Use `UseLocalLsfStack=true` in MSBuild and the environment throughout
      coordinated unreleased-stack work; revalidate released packages before
      release. Preserve deterministic ordering and warmed allocation guarantees.
- [ ] Publish the verified matrix, fixture/version provenance and limitations;
      update evergreen replay guidance without linking it to this plan.
- [ ] Archive this plan in `done/` only after the claimed execution/comparison
      gates pass, or explicitly split an unavailable platform rollout into a
      named deferred scope. Track any reproduced runtime defect separately.

No physics redesign, network protocol, new hash algorithm or benchmark timing
gate is implied. The completed [original replay plan](done/2026-06-26-deterministic-replay-hash-conformance-harness-plan.md)
remains historical; this plan adds the lifecycle and portability evidence.
