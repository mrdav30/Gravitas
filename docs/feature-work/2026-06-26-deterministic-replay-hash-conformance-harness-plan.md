# Deterministic Replay Hash Conformance Harness Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a deterministic authoritative-state hash and replay conformance harness so Gravitas can prove repeated, restored, and cross-mode simulations produce the same frame-by-frame physics state.

**Architecture:** Implement a fixed-order, fixed-width hash writer that never uses runtime `GetHashCode`, reflection, JSON text, or platform-dependent byte order. Expose a context-owned replay hash API for hosts and tests, then build conformance fixtures that compare uninterrupted simulation, restored simulation, and repeated simulation across 3D, pure 2D, mixed, CCD, queries that mutate caches, and serialization paths.

**Tech Stack:** .NET 8, xUnit v3, FixedMathSharp fixed-point values, SwiftCollections ordered scratch buffers, Chronicler save/populate tests, Gravitas runtime services, optional BenchmarkDotNet guardrails.

---

**Date:** 2026-06-26  
**Status:** Planned  
**Owner:** Gravitas determinism/replay hardening

## Purpose

Gravitas already has focused replay-style tests and explicit Chronicler state
transfer. A first-class lockstep physics engine should also expose a compact
deterministic state hash that hosts and tests can use as a frame-by-frame
conformance signal.

The hash should answer:

- did two runs with the same inputs produce identical authoritative physics
  state at every frame?
- did save/populate at frame N continue with the same state as uninterrupted
  simulation?
- did a cache, query, pair, warm-start, or service ordering change affect
  authoritative simulation?
- can host lockstep tooling cheaply compare physics state without serializing a
  full object graph each frame?

This is not a replacement for tests or serialization. It is a deterministic
alarm system that makes drift visible early.

## Current Baseline

- `docs/wiki/SERIALIZATION.md` documents populate-existing-shell replay flow.
- `SolidBody`, `SolidBody2D`, `LSCollider`, and `LSCollider2D` record explicit
  Chronicler state.
- Existing tests compare selected replay values in:
  - `tests/Gravitas.Tests/Runtime/GravitasSimulationPhaseOrderTests.cs`
  - `tests/Gravitas.Tests/Serialization/SolidBodySerializationTests.cs`
  - `tests/Gravitas.Tests/Serialization/SolidBody2DSerializationTests.cs`
  - `tests/Gravitas.Tests/MixedDimensions/MixedResponseTests.cs`
  - `tests/Gravitas.Tests/Physics2D/Physics2DSimulationTests.cs`
- Several value types implement `GetHashCode()`, but .NET hash APIs are not a
  deterministic replay contract and must not be used for the conformance hash.
- Query services and collision services own mutable caches that can affect
  runtime behavior if their lifecycle ordering drifts.

## Non-Goals

- Do not use platform-dependent `GetHashCode()` output.
- Do not hash JSON or MemoryPack payload text as the primary signal.
- Do not include diagnostics buffers, debug draw buffers, delegates, or host
  engine object identity in authoritative hashes.
- Do not make hashing mutate simulation state.
- Do not require hosts to enable diagnostics to compute hashes.
- Do not make conformance hashing a per-frame default runtime cost.

## Guiding Rules

- Hash order must be stable by context-owned IDs and explicit section tags.
- Every value must be written in a fixed byte order.
- Fixed-point values must hash their raw deterministic representation, not a
  decimal string.
- Optional sections must be explicitly tagged as present or absent.
- Rebuildable caches should be excluded from the authoritative hash unless they
  directly affect deterministic continuation.
- Warm-start, pair, CCD, and constraint state should be included only when that
  state can affect future authoritative simulation.
- The same scene should hash the same in Release and ReleaseLean.

## Proposed API Shape

The exact names should be finalized during Workstream 1. The intended surface is:

```csharp
public readonly struct GravitasReplayHash : IEquatable<GravitasReplayHash>
{
    public ulong Low { get; }
    public ulong High { get; }
}

public enum GravitasReplayHashMode
{
    Authoritative,
    AuthoritativeWithSolverCaches
}

public GravitasReplayHash GravitasWorldContext.ComputeReplayHash(
    GravitasReplayHashMode mode = GravitasReplayHashMode.Authoritative);
```

The implementation may expose the actual work through an internal
`GravitasReplayHashService` or static `GravitasReplayHasher` if that keeps
`GravitasWorldContext` small.

## Workstream 1: Stable Hash Writer And Primitive Canonicalization

**Problem**

The replay hash needs a deterministic writer before any runtime state can be
hashed safely.

**Tasks**

- [ ] Add `src/Gravitas/Determinism/GravitasReplayHash.cs`.
- [ ] Add `src/Gravitas/Determinism/GravitasReplayHashMode.cs`.
- [ ] Add `src/Gravitas/Determinism/GravitasReplayHashWriter.cs`.
- [ ] Use a documented fixed algorithm such as two-lane FNV-1a 64-bit or a
  similarly simple deterministic integer mixer.
- [ ] Add explicit writer methods for:
  - `bool`
  - `byte`
  - `int`
  - `uint`
  - `long`
  - `ulong`
  - enum values as explicit integer widths
  - `Fixed64`
  - `Vector2d`
  - `Vector3d`
  - `Vector4d`
  - `FixedQuaternion`
  - `FixedTransform`
  - `Fixed3x3`
  - `PhysicsLayer`
  - `PhysicsLayerMask`
- [ ] Verify the FixedMathSharp raw-value API and hash that raw value directly.
  If no public raw-value member exists, add a lower-stack FixedMathSharp helper
  rather than hashing formatted text.
- [ ] Prefix every logical section with a stable ASCII section tag and version
  integer.
- [ ] Add tests in
  `tests/Gravitas.Tests/Determinism/GravitasReplayHashWriterTests.cs` proving:
  - equal values produce equal hashes.
  - different section order produces different hashes.
  - vector/quaternion component changes affect the hash.
  - `GetHashCode()` is not used by the writer.

**Done Criteria**

- The hash writer is deterministic, fixed-width, and independently tested.
- Fixed-point values hash raw deterministic data.
- The writer can be reused by runtime state contributors without allocations.

## Workstream 2: Authoritative Runtime State Contributors

**Problem**

The hash must include state that determines future physics while excluding host
identity and rebuildable caches.

**Tasks**

- [ ] Add `src/Gravitas/Determinism/IGravitasReplayHashContributor.cs` if an
  interface keeps contributors clean without forcing public API noise.
- [ ] Add hash contribution methods for `PhysicsSettings` and
  `PhysicsEnvironment`.
- [ ] Add 3D body contribution for authoritative fields currently recorded by
  `SolidBody.Serialization.cs`.
- [ ] Add pure 2D body contribution for authoritative fields currently recorded
  by `SolidBody2D.Serialization.cs`.
- [ ] Add 3D collider contribution for:
  - collider ID.
  - active/trigger/layer/filter state.
  - hierarchy keys.
  - shape kind and shape values.
  - runtime shape version values that affect future simulation.
- [ ] Add pure 2D collider contribution with matching 2D and mixed-slab state.
- [ ] Add compound and mesh contribution through stable authored part and
  triangle order.
- [ ] Exclude host delegates, diagnostics, visual interpolation buffers, object
  references, query scratch buffers, and partition scratch buffers.
- [ ] Add tests proving two equivalent scenes built with different object
  allocation order hash the same after IDs and registration order are aligned.
- [ ] Add tests proving a single authoritative field change changes the hash.

**Done Criteria**

- Context settings, environment, bodies, and colliders hash authoritative state.
- Rebuildable or host-owned state is intentionally excluded.
- Hash contributors use stable explicit ordering.

## Workstream 3: Service State, Solver Cache Modes, And Ordering

**Problem**

Some service-owned state affects deterministic continuation. Other service
caches are rebuildable. The hash API needs a clear policy so it is useful
without becoming noisy.

**Tasks**

- [ ] Audit 3D, pure 2D, and mixed service fields for continuation impact:
  - active pair warm-start state.
  - retained collision pairs.
  - retained partitions.
  - CCD frame-start state.
  - processed-body handoff queues.
  - query scratch buffers.
  - diagnostic buffers.
- [ ] Include continuation-affecting state in `Authoritative`.
- [ ] Include optional solver/cache state in
  `AuthoritativeWithSolverCaches` only when it helps diagnose drift.
- [ ] Hash active collision pairs in sorted pair-key order.
- [ ] Hash warm-start contact impulses by stable contact ID order.
- [ ] Hash CCD handoff counters and queue state only if they can cross frame
  boundaries.
- [ ] Exclude query counters such as `LastQueryCandidateCount` from
  `Authoritative`.
- [ ] Add tests where pair registration churn still yields stable hashes when
  authoritative state is equal.
- [ ] Add tests where stale warm-start state changes
  `AuthoritativeWithSolverCaches` but not `Authoritative` if the cache is
  rebuildable.

**Done Criteria**

- The hash policy distinguishes authoritative truth from diagnostic cache
  signal.
- Service-owned ordering is explicit and tested.
- Hashing does not allocate after warmup.

## Workstream 4: Replay Conformance Harness

**Problem**

The hash becomes valuable when tests can run full scenarios and compare traces
across repeated and restored execution.

**Tasks**

- [ ] Add `tests/Gravitas.Tests/Determinism/ReplayConformanceHarness.cs`.
- [ ] Add a `ReplayHashTrace` test helper that stores one hash per frame in a
  `SwiftList<GravitasReplayHash>` or array.
- [ ] Add helpers for:
  - repeated run conformance.
  - save/populate-at-frame conformance.
  - ReleaseLean-compatible JSON-only conformance when MemoryPack is disabled.
- [ ] Add 3D conformance scenarios:
  - resting stack with warm-started friction.
  - dynamic CCD with one handoff.
  - kinematic active-source CCD.
  - mesh or compound contact.
- [ ] Add pure 2D conformance scenarios:
  - angular contact response.
  - manifold warm-start.
  - dynamic CCD.
  - planned 2D grounding once implemented.
- [ ] Add mixed conformance scenarios:
  - mixed dynamic response.
  - mixed static and dynamic CCD.
  - mixed query calls between simulation frames that mutate query caches.
- [ ] Add save/populate tests that compare hashes from restored and
  uninterrupted scenes for at least 16 frames after restore.
- [ ] Keep fixtures deterministic and independent from wall-clock time.

**Done Criteria**

- Test helpers can prove repeated and restored simulations hash identically.
- Queries and diagnostics that mutate caches do not perturb authoritative hash.
- Conformance failures produce frame index and hash values for quick RCA.

## Workstream 5: Public Host Utility, Docs, And Diagnostics

**Problem**

Hosts should be able to use the same hash signal for lockstep debugging without
depending on test-only helpers.

**Tasks**

- [ ] Expose the final hash API from `GravitasWorldContext`.
- [ ] Add XML docs that describe included and excluded state.
- [ ] Add a small sample in `docs/wiki/HOST_INTEGRATION.md` showing frame hash
  comparison between peers.
- [ ] Update `docs/wiki/SERIALIZATION.md` with hash-based replay conformance.
- [ ] Update `docs/wiki/RUNTIME_ARCHITECTURE.md` with the hash service or
  helper location.
- [ ] Update `docs/wiki/DIAGNOSTICS.md` only if the hash is also emitted as an
  optional diagnostic event. Prefer keeping it as a direct host call unless an
  event has a concrete use.
- [ ] Add allocation tests proving repeated `ComputeReplayHash(...)` calls
  allocate `0` bytes after warmup for representative 3D, pure 2D, and mixed
  scenes.

**Done Criteria**

- Hosts can compute frame hashes without enabling diagnostics.
- Docs state the hash contract clearly.
- Hashing is allocation-free after warmup for representative scenes.

## Workstream 6: Benchmarks And Release Validation

**Problem**

Hashing should be cheap enough for optional lockstep debug usage, and release
validation must cover standard and Lean builds.

**Tasks**

- [ ] Add benchmark rows under `tests/Gravitas.Benchmarks`:
  - `replay-hash-3d-sparse`
  - `replay-hash-3d-dense`
  - `replay-hash-2d-sparse`
  - `replay-hash-mixed`
  - `replay-hash-with-solver-caches`
- [ ] Record candidate/body/contact counts in benchmark setup so results are
  interpretable.
- [ ] Run focused determinism tests:
  `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter Determinism`
- [ ] Run full validation:
  `dotnet test Gravitas.slnx --configuration Release`
- [ ] Run Lean validation:
  `dotnet test Gravitas.slnx --configuration ReleaseLean`
- [ ] Run benchmark smoke:
  `dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll replay-hash --filter "*" -j Short -i`

**Done Criteria**

- Replay hashing has correctness, allocation, and benchmark evidence.
- Release and Lean configurations pass.
- Benchmark rows give future solver work a cheap conformance-cost signal.

## Final Done Criteria

- `GravitasWorldContext` exposes a deterministic replay hash API.
- Hashes are fixed-order, fixed-width, allocation-free after warmup, and do not
  use platform-dependent hash APIs.
- Conformance tests compare repeated, restored, 3D, pure 2D, mixed, CCD, and
  query-cache scenarios.
- Docs explain how hosts can use frame hashes for lockstep debugging.
