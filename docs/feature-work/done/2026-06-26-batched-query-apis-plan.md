# Batched Query APIs Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Add deterministic high-throughput batched query APIs for 3D, pure 2D, and mixed query services so large lockstep simulations can issue many ray, area, and sweep queries with caller-owned buffers and stable result ordering.

**Architecture:** Keep the existing exact query reducers as the source of geometric truth, but add batch request/result surfaces, shared per-batch scratch, stable per-request output ranges, and benchmark-backed broad-phase reuse where it is measurably stronger than repeated individual calls. Maintain the existing same-thread query service contract and avoid hidden allocations.

**Tech Stack:** .NET 8, xUnit v3, FixedMathSharp vectors and bounds, SwiftCollections caller-owned buffers, GridForge traversal, Gravitas 2D/3D/mixed query services, BenchmarkDotNet.

---

**Date:** 2026-06-26  
**Status:** Completed  
**Owner:** Gravitas query-service hardening

## Purpose

Gravitas query services are exact, deterministic, and caller-buffered, but the
public surface is currently query-at-a-time. LSF hosts with many agents often
need to issue hundreds or thousands of queries per frame:

- perception rays.
- steering and obstacle probes.
- area overlap checks.
- character grounding/support probes.
- projectile sweeps.
- AI line-of-sight and cone-adjacent checks.

Batched query APIs can improve end-user development experience and runtime
predictability by making the many-query case explicit. The goal is not a thin
wrapper around repeated calls as the final solution. The goal is a public batch
contract that preserves exact reducer behavior while enabling shared scratch,
stable output ranges, diagnostics, and measured broad-phase reuse where it
helps.

## Current Baseline

- `GravitasQuery3DService` owns raycast, swept-sphere, convex-source sweep, and
  X/Z circle overlap/proximity query buffers.
- `GravitasQuery2DService` owns pure 2D overlap, raycast, and swept-circle
  buffers.
- `GravitasQueryMixedService` owns explicit mixed swept-sphere and
  swept-circle finite-slab query buffers.
- All-hit query APIs write into caller-owned `SwiftList<T>` buffers.
- Query services keep mutable context-owned scratch and are documented as
  same-thread, non-reentrant services.
- Query hit ordering is stable by distance and collider identity.
- Diagnostics emit per-query events and query summary events in selected paths.
- Benchmark coverage exists for query services and mesh triangle scaling, but
  not for batched many-agent usage.

## Non-Goals

- Do not make query services thread-safe or reentrant in this plan.
- Do not reduce exact reducer quality to gain batch throughput.
- Do not introduce LINQ, iterator allocations, or hidden array allocations.
- Do not return heap-allocated per-query result arrays.
- Do not merge pure 2D, pure 3D, and mixed query semantics into one ambiguous
  mega-query API.
- Do not add engine-specific perception, AI, or navigation concepts.

## Guiding Rules

- Batch input order must be preserved in output metadata.
- Hits within each request must keep existing deterministic ordering.
- All-hit output should use a shared caller-owned hit buffer plus per-request
  ranges.
- Closest-hit batches should use one output element per request.
- Invalid requests should produce explicit miss/range-zero results without
  throwing unless the input itself is structurally invalid.
- Batch APIs should support pre-sizing and reuse.
- Diagnostics should avoid one event per sub-query by default in high-volume
  batch paths; use summary events unless detailed capture is explicitly enabled.

## Proposed API Shape

The exact names should be finalized during Workstream 1. The intended shape is:

```csharp
public readonly struct PhysicsQueryHitRange
{
    public int Start { get; }
    public int Count { get; }
}

public readonly struct PhysicsRaycast3DRequest
{
    public Vector3d Start { get; }
    public Vector3d End { get; }
    public PhysicsLayerMask LayerMask { get; }
}

public int GravitasQuery3DService.RaycastBatch(
    ReadOnlySpan<PhysicsRaycast3DRequest> requests,
    Span<Physics3DHit> closestHits);

public int GravitasQuery3DService.RaycastAllBatch(
    ReadOnlySpan<PhysicsRaycast3DRequest> requests,
    SwiftList<Physics3DHit> hits,
    Span<PhysicsQueryHitRange> ranges);
```

Use matching typed request structs for pure 2D and mixed query families. Prefer
typed batch methods over a tagged union when that keeps call sites clear and
avoids unused fields in hot paths.

## Workstream 1: Batch Result Contract And 3D Raycasts

**Problem**

The project needs a batch result shape that can cover closest-hit and all-hit
queries without per-request allocations.

**Tasks**

- [x] Add `src/Gravitas/Queries/Common/PhysicsQueryHitRange.cs`.
- [x] Add `src/Gravitas/Queries/3D/PhysicsRaycast3DRequest.cs`.
- [x] Add tests in
  `tests/Gravitas.Tests/Queries/GravitasQuery3DBatchTests.cs`:
  - closest-hit output length must be at least request count.
  - all-hit batch clears and fills caller-owned hit/range buffers.
  - invalid zero-length ray produces a miss and zero range.
  - request order is preserved in closest-hit output and ranges.
  - hits inside each request are sorted like `RaycastAll`.
- [x] Implement `GravitasQuery3DService.RaycastBatch(...)`.
- [x] Implement `GravitasQuery3DService.RaycastAllBatch(...)`.
- [x] Reuse existing raycast reducer and hit builder logic.
- [x] Ensure batch setup resets only per-request scratch, not caller buffers
  that should accumulate all hits.
- [x] Add allocation tests proving repeated 3D ray batches allocate `0` bytes
  after warmup.

**Done Criteria**

- 3D raycast batching has a stable public output contract.
- Closest and all-hit variants are deterministic and allocation-free after
  warmup.
- The result range model is ready for other query families.

## Workstream 2: Pure 2D Batch Query Parity

**Problem**

Pure 2D query users need the same many-agent ergonomics for raycasts, overlap
areas, and swept circles.

**Tasks**

- [x] Add request structs:
  - `PhysicsRaycast2DRequest`
  - `PhysicsOverlapCircle2DRequest`
  - `PhysicsOverlapAabb2DRequest`
  - `PhysicsOverlapPolygon2DRequest`
  - `PhysicsSweepCircle2DRequest`
- [x] Add tests in `tests/Gravitas.Tests/Physics2D/Physics2DBatchQueryTests.cs`
  for closest and all-hit variants where the single-query API already has both.
- [x] Implement pure 2D batch methods on `GravitasQuery2DService`.
- [x] Keep polygon request ownership explicit. If vertices are supplied by
  span, document that callers must keep them stable for the duration of the
  batch call.
- [x] Preserve existing pure 2D hit ordering by distance and collider ID.
- [x] Add allocation tests for ray, area, and swept-circle batches.

**Done Criteria**

- Pure 2D has batch parity for current public query families.
- Polygon inputs have clear lifetime rules.
- Batch output ordering matches existing single-query behavior.

## Workstream 3: 3D Sweep And Area Batch Families

**Problem**

Raycasts are only part of the 3D query load. Sweeps and X/Z area/proximity
queries need typed batch coverage without duplicating reducer logic.

**Tasks**

- [x] Add request structs:
  - `PhysicsSweepSphere3DRequest`
  - `PhysicsSweepCapsule3DRequest`
  - `PhysicsSweepCuboid3DRequest`
  - `PhysicsSweepCylinder3DRequest`
  - `PhysicsSweepConvexMesh3DRequest`
  - `PhysicsSweepCompound3DRequest`
  - `PhysicsOverlapCircle3DRequest`
- [x] Add closest and all-hit batch tests for sphere sweeps and X/Z circle
  overlaps first.
- [x] Add registered-source sweep batch tests for capsule, cuboid, cylinder,
  convex mesh, and compound sources.
- [x] Implement batch methods by reusing prepared-source workers and existing
  exact reducers.
- [x] Avoid re-preparing the same source collider when consecutive batch
  requests reference the same source and displacement.
- [x] Keep concave mesh source rejection behavior identical to single-query
  APIs.
- [x] Add allocation tests for representative sweep batches.

**Done Criteria**

- Current 3D public query families have batch coverage.
- Exact reducer behavior and source rejection policy stay unchanged.
- Repeated source preparation is avoided where the batch makes reuse obvious.

## Workstream 4: Mixed Batch Query Families

**Problem**

Mixed mode has explicit finite-slab query families. Large mixed worlds need
batched access without blurring pure 2D and pure 3D services.

**Tasks**

- [x] Add request structs:
  - `PhysicsSweepSphereAgainst2DRequest`
  - `PhysicsSweepCircleAgainst3DRequest`
- [x] Add tests in
  `tests/Gravitas.Tests/MixedDimensions/MixedBatchQueryTests.cs`:
  - request order is preserved.
  - closest mixed hits match individual query calls.
  - all-hit ranges match individual all-hit query calls.
  - reducer kind remains exact for supported paths.
  - invalid zero displacement produces miss/range zero.
- [x] Implement closest and all-hit batch methods on
  `GravitasQueryMixedService`.
- [x] Preserve mixed result ordering by distance and dimension-tagged collider
  identity.
- [x] Keep mixed diagnostics as summary-first to avoid high-volume event spam.
- [x] Add allocation tests for mixed batches after warmup.

**Done Criteria**

- Mixed batch APIs remain explicit and dimension-safe.
- Existing finite-slab reducer behavior is preserved.
- Mixed high-volume query usage has allocation guardrails.

## Workstream 5: Shared Batch Scratch, Diagnostics, And Broad-Phase Reuse

**Problem**

A final batch solution should do more than loop over public calls. It needs
shared scratch ownership and should adopt broad-phase reuse only where the batch
shape provides measured value without compromising request-local ordering.

**Tasks**

- [x] Reuse service-owned ray/sweep/circle/finite-slab scratch and add
  dedicated batch hit scratch for all-hit append paths.
- [x] Preserve request-local duplicate suppression and hit ordering instead of
  sharing candidate stamps across the whole batch.
- [x] Keep request-local exact reducer processing as the default path.
- [x] Do not add broad-phase grouping without benchmark evidence; the benchmark
  smoke gives a stable baseline for a future measured optimization if a signal
  appears.
- [x] Add diagnostics:
  - batch request count.
  - total hit count.
  - total candidate count.
  - mixed mesh-triangle candidate count.
- [x] Add tests proving diagnostics disabled path allocates `0` bytes.
- [x] Add tests proving diagnostics can be enabled without changing hit order.

**Done Criteria**

- Batch APIs own reusable scratch and do not depend on hidden per-request
  allocations.
- Broad-phase grouping remains evidence-gated; no speculative grouping is part
  of this completed pass.
- Public batch summary counters are useful without adding high-volume event
  spam by default.

## Workstream 6: Benchmarks, Docs, And Release Validation

**Problem**

Batched query APIs exist to improve throughput. They need benchmark evidence and
clear docs before being presented as a first-class LSF feature.

**Tasks**

- [x] Add benchmark rows:
  - `query-batch-3d-raycast`
  - `query-batch-3d-sweep-sphere`
  - `query-batch-2d-raycast`
  - `query-batch-2d-area`
  - `query-batch-2d-sweep-circle`
  - `query-batch-mixed-sweeps`
- [x] Compare batch APIs against equivalent individual calls for sparse and
  dense scenes.
- [x] Track hit count, request count, candidate count, and allocation.
- [x] Update `docs/wiki/QUERY_SERVICES.md` with batch API contracts and
  same-thread rules.
- [x] Update `docs/wiki/HOST_INTEGRATION.md` with caller-owned batch buffer
  examples.
- [x] Update `docs/wiki/DIAGNOSTICS.md` if batch summary diagnostics are added.
- [x] Run focused query tests:
  `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter Query`
- [x] Run full validation:
  `dotnet test Gravitas.slnx --configuration Release`
- [x] Run Lean validation:
  `dotnet test Gravitas.slnx --configuration ReleaseLean`
- [x] Run benchmark smoke:
  `dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll query-batch --filter "*" -j Short -i`

**Done Criteria**

- Batched query APIs are documented, tested, benchmarked, and allocation-clean
  after warmup.
- Benchmarks show where batching improves throughput and where individual calls
  remain equivalent.
- Release and Lean configurations pass.

## Final Done Criteria

- 3D, pure 2D, and mixed query services expose typed batch APIs for current
  public query families.
- Closest-hit and all-hit batch results use caller-owned buffers with stable
  per-request metadata.
- Exact reducer behavior, filtering, and ordering match individual query APIs.
- Batch processing owns reusable scratch and has measured throughput evidence.
