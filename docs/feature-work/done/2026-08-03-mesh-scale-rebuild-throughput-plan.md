# Mesh Scale Rebuild Throughput Hardening

**Created:** 2026-08-03  
**Status:** Completed  
**Signal:** Mesh scale rebuild allocates with subdivision count

## Goal

Make repeated convex-mesh scale preparation allocation-free without weakening
support-point correctness, deterministic authored-order tie handling,
transactional scale publication, or query throughput.

Retain the smallest proven change. Do not expose or duplicate a general sorting
API when scale changes do not require rebuilding support-tree topology.

## Preserved Baseline

The focused ShortRun benchmark reports:

| Subdivision | Mean | Allocated |
| ---: | ---: | ---: |
| 1 | `38.394 us` | `0 B/op` |
| 8 | `2.166 ms` | `4,032 B/op` |
| 16 | `8.822 ms` | `16,320 B/op` |

Artifact:
`artifacts/benchmarks/2026-08-03-mesh-scale-rebuild-baseline`.

## Root Cause

`PhysicsMesh.PrepareScaledGeometry(...)` rebuilds the convex support-tree
topology after every scale change. Each non-leaf `Array.Sort(...)` invocation
allocates 64 bytes through the reference comparer. The 8- and 16-subdivision
meshes contain 63 and 255 non-leaf nodes, exactly accounting for the observed
`4,032 B/op` and `16,320 B/op`.

Diagnostic isolation confirms that repeated scale preparation is
allocation-free when the support tree is absent; triangle-BVH reconstruction,
scaled face data, and mesh mass properties are not allocating owners.

## Approved Design

Build the support-tree vertex partition once during mesh construction. A scale
change preserves vertex identity and topology, so rebuild only node bounds from
the newly prepared vertices:

1. Keep one immutable, sorted support-vertex index array.
2. Retain committed and prepared node buffers for transactional publication.
3. Refit leaves from their existing vertex ranges.
4. Refit branches bottom-up from their children.
5. Swap only node buffers after the complete scale candidate is accepted.

This removes unnecessary sorting and the second vertex-index buffer. Support
queries retain exact authored-order tie behavior because node membership and
minimum source indices do not change.

## Work Plan

### Phase 1: Regression Contract

- [x] Add a dense convex-mesh scale-rebuild allocation regression.
- [x] Confirm the existing scaled support-tree parity regression exercises the
      refit path against authored-order brute force.
- [x] Run the focused tests against the allocating implementation. Two measured
      subdivision-8 scale changes reproduced exactly `8,064 B`.

### Phase 2: Immutable Topology And Bounds Refit

- [x] Remove the prepared support-index buffer and its publication swap.
- [x] Build topology once and refit prepared node bounds on later scale changes.
- [x] Preserve transactional failure behavior and zero-allocation queries.
- [x] Delete superseded scale-time rebuild logic and stop retaining the
      construction-only comparer.

### Phase 3: Evidence And Release Gates

- [x] Run focused, Release, and ReleaseLean tests.
- [x] Retain 100% reachable line, branch, and method coverage.
- [x] Repeat the exact benchmark and confirm `0 B/op` at every subdivision.
- [x] Confirm support correctness and query throughput do not regress. An
      extreme `32:1:1/32` scale probe matched a topology built directly from
      the scaled vertices and remained within `-3.0%` to `+0.6%` across five
      alternating rounds.

### Phase 4: Documentation And Closure

- [x] Update the benchmark-signal backlog and feature-work overview.
- [x] Review the diff for duplicate code, zombie branches, and unnecessary API.
- [x] Complete an independent code review. The first pass requested stronger
      query and failed-preparation evidence; both findings were closed, and the
      confirmation pass reported no remaining actionable issue.
- [x] Move this plan to `docs/feature-work/done` only after every gate passes.

## Final Evidence

The unchanged focused ShortRun was repeated twice. The confirmation run reports:

| Subdivision | Baseline | Confirmation | Delta | Allocated |
| ---: | ---: | ---: | ---: | ---: |
| 1 | `38.394 us` | `37.755 us` | `-1.7%` | `0 B/op` |
| 8 | `2.166 ms` | `1.994 ms` | `-7.9%` | `0 B/op` |
| 16 | `8.822 ms` | `8.131 ms` | `-7.8%` | `0 B/op` |

The dense unit guard independently measures two subdivision-8 scale changes at
exactly `0 B`, down from the pre-change `8,064 B`. Existing authored-order
brute-force support parity and transactional failure regressions remain green.

Validation:

- 3,928 Release tests passed.
- 3,873 ReleaseLean tests passed.
- Coverage is 55,869/55,869 lines, 15,833/15,833 branches, and 5,321/5,321
  methods.
- Release and ReleaseLean `netstandard2.1` builds completed with zero warnings.

Artifacts:

- `artifacts/benchmarks/2026-08-03-mesh-scale-rebuild-baseline`
- `artifacts/benchmarks/2026-08-03-mesh-scale-rebuild-topology-refit-first-pass`
- `artifacts/benchmarks/2026-08-03-mesh-scale-rebuild-topology-refit-confirmation`
- `tests/Gravitas.Tests/TestResults/coverage-analysis-mesh-scale-rebuild-20260803`
