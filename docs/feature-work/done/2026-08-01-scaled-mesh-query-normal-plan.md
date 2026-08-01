# Scaled Mesh Query Normal Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> `superpowers:subagent-driven-development` or
> `superpowers:executing-plans` to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make non-uniformly scaled mesh raycasts and swept-sphere queries
solve against the same committed triangle plane as their scaled vertices.

**Architecture:** `PhysicsMesh` remains the sole owner of scale-derived face
normals. One bounds-checked internal accessor exposes the committed scaled
local normal; the two local-frame query workers consume it without per-query
recomputation or a new abstraction.

**Tech Stack:** C#, .NET 8 / .NET Standard 2.1, FixedMathSharp Q32.32 geometry,
SwiftCollections, xUnit v3, Microsoft code coverage.

## Global Constraints

- Preserve deterministic fixed-point behavior and stable candidate ordering.
- Keep the query hot paths allocation-free after warmup.
- Delete the now-unreferenced authored `PhysicsMesh.FaceNormals` cache rather
  than retaining its allocation and lazy branches through coverage-only tests.
- Keep local project references unstaged and out of release artifacts.
- Retain 100% reachable line, branch, and method coverage.
- Leave implementation changes unstaged and uncommitted for owner review.

---

### Task 1: Lock The Two Query Regressions

**Files:**

- Test: `tests/Gravitas.Tests/Queries/GravitasQuery3DServiceSweepTests.cs`
- Test: `tests/Gravitas.Tests/Colliders/LSMeshColliderQueryTests.cs`

**Interfaces:**

- Consumes: existing `LSMeshCollider`, `SweptSphereQueryWorker`, and
  `RaycastSegmentWorker` APIs.
- Produces: literal non-uniform-scale regressions that fail when a worker pairs
  committed vertices with authored normals.

- [x] **Step 1: Add the swept-sphere reproducer**

  Use authored `A=(0,0,0)`, `B=(1,0,0)`, `C=(0,1,1)`, committed scale
  `(1,1,2)`, interior point `P=(1/3,1/3,2/3)`, authored unit normal
  `n0=(0,-1/sqrt(2),1/sqrt(2))`, radius `1/10`, and segment `P-n0` to
  `P+n0`. Assert the first distance is the independently derived Q32.32 value
  `Fixed64.FromRaw(3_842_237_992)` (`1 - sqrt(10)/30`), plus the matching
  center.

- [x] **Step 2: Add the raycast reproducer**

  Reuse the same committed triangle and segment. Assert the unique
  intersection is `P`; the authored-normal plane instead returns the distinct
  point `(1/3,1/2,1/2)`.

- [x] **Step 3: Keep both warmed reproductions allocation-free**

  Measure each new direct worker operation with
  `AllocationTestHelper.MeasureSteadyState(...)`. Reuse the worker and collider;
  clear and reuse the ray hit buffer on every measured invocation. Require
  exactly zero managed bytes.

- [x] **Step 4: Prove both regressions fail before production changes**

  Run the two filtered tests from `tests/Gravitas.Tests/Gravitas.Tests.csproj`
  in `Release`. Expect the swept distance and ray point assertions to fail on
  the authored-normal results documented above.

### Task 2: Adopt The Committed Normal Owner

**Files:**

- Modify: `src/Gravitas/Colliders/Mesh/PhysicsMesh.cs`
- Modify: `src/Gravitas/Queries/3D/Sweeps/SweptSphereQueryWorker.cs`
- Modify: `src/Gravitas/Queries/3D/RaycastSegmentWorker.cs`

**Interfaces:**

- Produces: `internal Vector3d GetScaledLocalFaceNormal(int index)`.
- Consumes: the committed `_scaledFaceNormals` array already published with
  `_scaledLocalVertices` by the mesh scale transaction.

- [x] **Step 1: Add the focused mesh accessor**

  Bounds-check `index` with `SwiftThrowHelper.ThrowIfArrayIndexInvalid`, then
  return `_scaledFaceNormals[index]`. Do not allocate, normalize again, or
  expose the internal array.

- [x] **Step 2: Replace both authored-normal reads**

  Beside each `GetLocalTriangleVertices(...)` call, replace
  `FaceNormals[triangleIndex]` with
  `GetScaledLocalFaceNormal(triangleIndex)`.

- [x] **Step 3: Prove both regressions pass**

  Rerun the two filtered `Release` tests and their containing test classes.

### Task 3: Release Closure

**Files:**

- Modify: `docs/wiki/QUERY_SERVICES.md`
- Modify: `docs/feature-work/issue-tracker.md`
- Move: this plan to `docs/feature-work/done/`

**Interfaces:**

- Consumes: the corrected query behavior from Task 2.
- Produces: an empty correctness queue backed by current verification counts.

- [x] **Step 1: Audit parity and ordering**

  Confirm no other scaled-vertex consumer reads authored normals, then run the
  mesh all-hit ordering regression and the existing raycast raw-distance
  ordering regressions. Remove the authored `FaceNormals` property, backing
  cache, and calculators if the full cross-stack caller audit proves they are
  otherwise dead.

- [x] **Step 2: Prove allocation and performance safety**

  Run the new warmed mesh raycast and swept-sphere allocation assertions.
  Compare the existing dense-mesh swept-sphere benchmark row; no dedicated
  mesh-raycast row or new benchmark is warranted for replacing one cached array
  read with another.

- [x] **Step 3: Run the complete gates sequentially**

  Run Gravitas `Release`, `ReleaseLean`, independent coverage, and standard and
  Lean package builds. Require 100% reachable line, branch, and method coverage
  and zero build warnings.

- [x] **Step 4: Complete independent review and documentation closure**

  Request a read-only whole-change review, resolve any findings, document the
  committed-normal contract in the query wiki, move the issue to resolved, and
  archive this completed plan under `docs/feature-work/done/`.

## Closure Evidence

- Literal raycast and swept-sphere regressions pass and each reports zero
  steady-state managed allocation.
- The complete caller/parity audit found only the two corrected consumers. The
  authored `FaceNormals` surface and its cache were otherwise dead and were
  removed, reducing each mesh by one allocated array.
- Stable mesh all-hit and raw-distance ordering regressions pass.
- Gravitas passes 3,925 `Release` and 3,870 `ReleaseLean` tests. Independent
  coverage is 55,839/55,839 lines, 15,829/15,829 branches, and 5,320/5,320
  methods.
- Standard and Lean package builds pass for `net8.0` and `netstandard2.1` with
  zero warnings.
- Independent whole-change review reported no Critical, Important, or Minor
  findings.
- The existing dense-mesh swept-sphere Dry signal remains comparable at
  subdivisions 8/16/32: `26.71 ms`, `49.70 ms`, and `179.33 ms` after the fix
  versus `27.30 ms`, `49.29 ms`, and `181.36 ms` before it. These single-shot
  cold-start rows are a complexity guard, not a throughput claim.
