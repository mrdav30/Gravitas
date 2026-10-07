# Full-Lifecycle Replay Conformance Plan

**Date:** 2026-10-06  
**Status:** Completed 2026-10-07; all eight native platform/profile lanes and direct comparison verified.  
**Owner:** Gravitas test harness and CI; lower-stack owners only for reproduced defects.

## Goal

Prove that the same ordered commands produce the same authoritative state through
the complete host loop, across supported runtime modes, package profiles and
native CPU platforms. This strengthens evidence for lockstep hosts without
changing physics merely to preserve existing test output.

## Verified Baseline Before Phase 1

- [`ReplayConformanceHarness`](../../../tests/Gravitas.Tests/Determinism/ReplayConformanceHarness.cs)
  advanced only `LateSimulate()` in both `RunTrace` and `AssertNextFramesMatch`.
  Commands precede the late phase, and hashes follow it.
- [`GravitasReplayConformanceTests`](../../../tests/Gravitas.Tests/Determinism/GravitasReplayConformanceTests.cs)
  already exercises both phases in its wide-clock test. Existing scenarios cover
  3D CCD, mesh/compound, pure 2D, mixed queries and body restore continuation.
- [`Constraint3DStressTests`](../../../tests/Gravitas.Tests/Constraints/Constraint3DStressTests.cs)
  supplied `Simulate()` through `beforeFrame`; changing the default without
  migrating these callers would have advanced twice.
- [`build-and-test.yml`](../../../.github/workflows/build-and-test.yml) runs
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
Release/Lean passed 4,543/4,482 tests with exact 100% line, branch and method
coverage. Both target-framework builds and documentation checks passed using
`UseLocalLsfStack=true`; no runtime or lower-stack source change was required.
The Debug subset passed 123 tests, excluding the independently reproduced
ragdoll allocation failure captured as
[GRV-Benchmark-025](../benchmark-signal-hardening-backlog.md#grv-benchmark-025--debug-ragdoll-steady-state-allocations).
Detailed captures remain under ignored `artifacts/replay-conformance-phase1/`.

## Phase 2: Shared Command And Result Fixtures

- [x] Reuse current scenario builders and ChronicleHash. Define a small,
      versioned fixture manifest containing ordered commands, frame count,
      settings, stable host entity identity and expected per-frame hashes.
- [x] Cover sleep/wake, spawn/despawn and registration reuse, queries, CCD,
      connected contacts/constraints and supported restore continuation with a
      few focused traces. Assert the named contact/CCD event actually occurs;
      a one-shot force and hash agreement alone may never reach the target.
      Reuse these fixtures in subsequent workload plans.
- [x] Check every frame against a reviewed shared expectation as well as a
      repeated run. Include semantic assertions for events and representative
      raw state so the hash producer is not the sole correctness oracle.
- [x] Use `Authoritative` as the mandatory portable comparison. Compare
      `AuthoritativeWithSolverCaches` separately under identical fixture/cache
      contracts; do not mix the two modes or compare unsupported restore caches.
- [x] Report the first divergent frame, preceding commands, raw body state and
      relevant ordered events. Keep machine/runtime/dependency metadata outside
      the equality payload. Never regenerate expected values automatically on
      failure; investigate intentional changes and review fixture versions.

Body payloads do not restore the containing clock, object graph, coroutines or
all solver caches. Restore fixtures must reconstruct supported shells and match
the host timeline under the [serialization contract](../../wiki/SERIALIZATION.md).
This plan does not introduce a live-world rewind API.

Completed 2026-10-06. Five version-1 fixtures cover 36 frames, all four modes,
lifecycle/query/CCD behavior, connected constraints and supported free-body
restore. Shared expectations, repeated runs and uninterrupted continuation agree;
analytic stop/momentum assertions supplement raw state and ordered observations.
Existing builders and Chronicler hashes are reused. Explicit layer-zero settings
and manual 3D grounding isolate the fixture contract; a regression removed an
introduced dependency on ambient layer names.

Windows x64 local-stack validation passed 47 new tests, 4,590/4,529 full
Release/Lean tests and 170 focused Debug tests. Exact 100% line, branch and method
coverage, both target-framework builds, DocFX and link checks passed; Debug still
excludes #025. No runtime or upstream source changed. The separate wall-support
defect is captured as [GRV-Issue-090](../issue-tracker.md#grv-issue-090---3d-automatic-swept-ground-probes-accept-vertical-wall-contacts).
Baseline Gravitas `5be7224`, SDK 10.0.302, sibling identities, fixture fingerprints
and detailed gates are recorded under ignored `artifacts/replay-conformance-phase2/`;
the successful capture is `final-gates-20261007T024929080Z-743bdd00006b46008b6072d799376a3a`.
Native cross-OS/architecture comparison remains Phase 3.

## Phase 3: Platform Comparison

- [x] First publish and compare the same fixture results between existing
      Windows/Linux Release/Lean lanes. Separate independently passing unit
      tests from an actual shared-result comparison.
- [x] Inventory native x64 and ARM64 runner availability and record OS, actual
      architecture, SDK/runtime, source revision and dependency identity.
      Add native ARM64 lanes where hosting is approved and available. Emulation
      or compilation alone is not native execution evidence.
- [x] Require expected traces and direct cross-lane comparison to agree for
      identical supported fixtures. Missing lanes remain visible evidence gaps,
      never passes. Add a tested comparator failure case using one altered frame.
- [x] Configure small conformance traces in normal CI; retain artifacts for
      diagnosis. Build both library target frameworks without claiming that a
      `netstandard2.1` compile is another execution platform.
- [x] Confirm the first hosted eight-lane execution and artifact comparison.
      Native Windows/Linux x64/ARM64 Release/Lean results all passed.

The complete native matrix is Windows/Linux x64/ARM64, each in Release/Lean.
Runner availability can stage its rollout; closure must name the actually
executed matrix and any remaining lanes rather than claim universal CPU proof.
Fixture authoring for the next plan can begin once Phase 1 and the shared trace
format are established; unavailable ARM hosting need not block that authoring.

Native runner inventory was verified 2026-10-06 against GitHub's
[standard public-repository runners](https://docs.github.com/en/actions/reference/runners/github-hosted-runners#standard-github-hosted-runners-for-public-repositories).
The public repository can use `windows-2025`, `ubuntu-24.04`, `windows-11-arm`
and `ubuntu-24.04-arm` for Windows/Linux x64/ARM64 respectively. Each runs both
profiles with a native SDK. Captures record actual OS and process architecture;
the comparator validates both against the lane rather than trusting its label.
Develop pushes and PRs targeting `develop` check out identical immutable sibling
revisions in all lanes with `UseLocalLsfStack=true`. Main and other branches use
released packages with `UseLocalLsfStack=false` and no sibling checkouts. PRs
select their target branch, so `develop` -> `main` validates packages; upstream
publication must precede promotion. Captures record SDK/runtime, fixture
fingerprints and the selected dependency mode. Source mode requires sibling
commit IDs; package mode requires an empty source map and matching loaded
assembly/informational versions. Mixed modes fail comparison.

The tested comparator rejects missing lanes/files, incompatible provenance,
ordered observation changes, lost integer precision and equally incorrect
results. Direct lane comparison and shared expectations are separate required
guards. An explicit local submatrix reports every omitted lane; default CI
requires all eight. Observed results never overwrite reviewed fixtures.

Validated 2026-10-07. Independently compiled Windows x64 and Ubuntu 24.04.1
x64 under WSL2 agree directly and with all five version-1 fixtures' 36 frames
in Release/Lean. The native captures record Windows SDK/runtime
10.0.302/8.0.29 and Linux 10.0.203/8.0.26, Gravitas baseline `88a28c0` plus the
reviewed batch, and full sibling identities. Source pins are FixedMathSharp
`9774e649`, SwiftCollections `3668bb4d`, GridForge `2ccf9783` and Chronicler
`5397348a`; the Linux snapshot verified 1,988 files across the five repositories.
Actual native captures remain under ignored `artifacts/replay-conformance-phase3/`.

Review removed the capture/comparator helper suites and the earlier fixture
contract suite, retaining actual library lifecycle, physics, query, CCD and
restore assertions. The comparator still validates real CI captures; controlled
copies confirm that missing lanes, one altered raw unit and equally incorrect
lanes fail. Capture instructions belong in [contributor guidance](../../../AGENTS.md#replay-conformance-captures);
the wiki describes host-facing contracts. Source/package selection now follows
the target branch, including package validation for `develop` -> `main`.

Cleanup validation passed 4,549/4,488 full Windows Release/Lean tests and 129
focused Debug tests with the known #025 guard excluded. Exact production line,
branch and method coverage remains 100% in both profiles; both target-framework
builds, DocFX, links and workflow lint pass using `UseLocalLsfStack=true`.
Fresh captures are under
`artifacts/replay-conformance-phase3/final-gates-20261007T144355786Z-21016bb3baee499290cf0467988db06f/`.
No runtime, upstream or reviewed fixture expectation changed.

Closed 2026-10-07 after [hosted run 37640701967](https://github.com/mrdav30/Gravitas/actions/runs/37640701967)
on develop commit `23cd1380a3a37ed636ab448a2637ec8502c99056`. All eight native
Windows/Linux x64/ARM64 Release/Lean suites, artifact uploads and direct replay
comparison passed. Downloaded artifacts independently reproduce full-matrix
agreement for all five fixtures and 36 frames, including the separate cache
trace; no lanes are missing or emulated. Every lane records SDK 10.0.401,
runtime 8.0.31, matching actual OS/process architecture and the pinned source
graph. Local copies are retained under ignored
`artifacts/replay-conformance-phase3/hosted-37640701967/`. This establishes the
claimed source-stack conformance matrix; released-package validation remains a
mandatory promotion/release gate after upstream publication.

## Validation And Completion

- [x] Run focused replay/lifecycle/coroutine/restore tests, full Release/Lean
      suites, both target-framework builds and exact 100% reachable line,
      branch and method coverage for every changed owning library.
- [x] Use `UseLocalLsfStack=true` in MSBuild and the environment throughout
      coordinated unreleased-stack work; revalidate released packages before
      release. Preserve deterministic ordering and warmed allocation guarantees.
- [x] Publish the verified matrix, fixture/version provenance and limitations;
      update evergreen replay guidance without linking it to this plan.
- [x] Archive this plan in `done/` only after the claimed execution/comparison
      gates pass, or explicitly split an unavailable platform rollout into a
      named deferred scope. Track any reproduced runtime defect separately.

No physics redesign, network protocol, new hash algorithm or benchmark timing
gate is implied. The completed [original replay plan](2026-06-26-deterministic-replay-hash-conformance-harness-plan.md)
remains historical; this plan adds the lifecycle and portability evidence.
