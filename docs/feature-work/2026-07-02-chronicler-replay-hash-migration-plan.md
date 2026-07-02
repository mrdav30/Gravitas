# Chronicler Replay Hash Migration Implementation Plan

**Date:** 2026-07-02  
**Status:** Planned  
**Owner:** Gravitas determinism and replay-hash hardening

---

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace Gravitas-local generic replay hash infrastructure with Chronicler and FixedMathSharp.Chronicler APIs while preserving Gravitas-owned replay inclusion policy, deterministic ordering, and no-allocation steady-state behavior.

**Architecture:** Use `Chronicler.ChronicleHash` and `Chronicler.ChronicleHashWriter` as the generic hash value/writer, use `FixedMathSharp.Chronicler` extension methods for fixed-point math primitives, and keep Gravitas-specific contributors for physics-domain ordering and inclusion modes. Remove duplicate local hash primitives instead of carrying compatibility aliases before the first public release.

**Tech Stack:** .NET 8, xUnit v3, BenchmarkDotNet, Chronicler.Core `0.3.x`, FixedMathSharp.Chronicler `6.x`, FixedMathSharp v6, SwiftCollections, GridForge, Gravitas replay/conformance harness.

## Purpose

Gravitas introduced a useful deterministic replay hash harness before the lower
stack had shared infrastructure. That local implementation proved the shape:

- a fixed-width deterministic hash value.
- an allocation-free primitive writer.
- fixed little-endian primitive writes.
- ASCII section tags and schema versions.
- domain-owned replay inclusion modes.
- conformance tests and allocation guardrails.

Chronicler now owns the generic hash framework, and FixedMathSharp now owns
fixed-point/vector/matrix/transform/geometry writer extensions in the
`FixedMathSharp.Chronicler` companion package. Gravitas should consume those
released lower-stack APIs and delete the duplicate generic writer/value code,
while keeping physics-specific replay policy in Gravitas.

This plan was extracted from:

- `F:/gamedevrepos/Chronicler/docs/feature-work/done/2026-06-26-deterministic-record-hash-framework-plan.md`

## Current Baseline

- `GravitasWorldContext.ComputeReplayHash(...)` returns
  `GravitasReplayHash`.
- `src/Gravitas/Determinism/GravitasReplayHash.cs` duplicates
  `Chronicler.ChronicleHash`.
- `src/Gravitas/Determinism/GravitasReplayHashWriter.cs` duplicates
  `Chronicler.ChronicleHashWriter` plus FixedMathSharp primitive writer
  methods.
- `GravitasReplayHashMode` is real Gravitas policy and should stay.
- `ContributeReplayHash(...)` partials manually walk context-owned services,
  bodies, colliders, pairs, constraints, warm-start caches, and mixed state in
  deterministic order.
- `src/Gravitas/Gravitas.csproj` references `Chronicler.Core`, but does not yet
  reference `FixedMathSharp.Chronicler`.
- Existing determinism tests cover repeated 3D, pure 2D, mixed, CCD,
  Chronicler restore continuation, query-cache exclusion, and no-allocation
  replay hashing.

## Non-Goals

- Do not remove `GravitasReplayHashMode`; replay inclusion modes are physics
  domain policy.
- Do not replace service-owned replay contributors with blind
  `IRecordable.RecordData(...)` traversal where ordering or inclusion policy
  would change.
- Do not hash JSON, MemoryPack, or serialized byte payloads as the authoritative
  replay signal.
- Do not preserve `GravitasReplayHash` or `GravitasReplayHashWriter` as public
  compatibility aliases unless implementation discovers a strong reason. This
  is pre-public-release cleanup.
- Do not introduce reflection, LINQ, iterator allocations, unordered collection
  traversal, culture formatting, wall-clock state, or runtime object identity.
- Do not accept allocation regressions in warmed replay-hash paths.

## Guiding Rules

- Gravitas owns what gets hashed and in which deterministic order.
- Chronicler owns generic hash value/writer mechanics.
- FixedMathSharp.Chronicler owns fixed-point math primitive writer extensions.
- Physics-layer, material, runtime-mode, collider-ID, pair-ID, and solver-cache
  helpers belong in Gravitas.
- If the lower-stack hash output changes from the old local writer, that is
  acceptable as long as repeated equivalent runs remain identical and docs do
  not promise stable hash values across package versions.
- Every public API change must be reflected in tests and wiki docs.

## Proposed API Shape

Preferred public result type:

```csharp
public ChronicleHash ComputeReplayHash(
    GravitasReplayHashMode mode = GravitasReplayHashMode.Authoritative)
```

`ChronicleHash` already has the same useful surface as `GravitasReplayHash`:

- `Low`
- `High`
- equality/operators
- stable lowercase 32-character hex `ToString()`

Keep `GravitasReplayHashMode` unchanged:

```csharp
public enum GravitasReplayHashMode
{
    Authoritative,
    AuthoritativeWithSolverCaches
}
```

Add a small Gravitas extension/helper file only for domain-specific writes that
do not belong in FixedMathSharp.Chronicler:

```csharp
internal static class GravitasChronicleHashWriterExtensions
{
    public static void WritePhysicsLayer(this ref ChronicleHashWriter writer, PhysicsLayer value)
    {
        writer.WriteInt32(value.Index);
    }

    public static void WritePhysicsLayerMask(this ref ChronicleHashWriter writer, PhysicsLayerMask value)
    {
        writer.WriteInt32(value.Bits);
    }
}
```

The final helper may include material or shape-policy writers only if it
removes real duplication from existing contributors.

## Workstream 1: Dependency Surface And Baseline Evidence

**Problem**

The package graph now has Chronicler and FixedMathSharp hash packages, but
Gravitas does not yet reference the FixedMathSharp companion package. Baseline
tests and allocation evidence should be captured before deleting local writer
code.

**Files**

- Modify: `src/Gravitas/Gravitas.csproj`
- Review: `tests/Gravitas.Tests/Gravitas.Tests.csproj`
- Review: `tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj`

**Tasks**

- [ ] Confirm the worktree only contains intended package-reference migration
      changes before editing.
- [ ] Add standard build package reference:

```xml
<PackageReference Include="FixedMathSharp.Chronicler" Version="6.0.0" />
```

- [ ] Add lean build package reference:

```xml
<PackageReference Include="FixedMathSharp.Chronicler.Lean" Version="6.0.0" />
```

- [ ] If local project references are used during active migration, add matching
      references to the library, tests, and benchmarks because local restore can
      require explicit child-project links.
- [ ] Run baseline determinism tests before source changes:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter Determinism
```

- [ ] Build the benchmark project:

```bash
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj --configuration Release -f net8.0
```

- [ ] Capture replay-hash benchmark smoke:

```bash
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll replay-hash --filter "*" -j Short -i --exporters json
```

**Done Criteria**

- Gravitas can compile against the lower-stack hash packages.
- Baseline determinism tests and benchmark/allocation smoke exist before
  migration.

## Workstream 2: Public Hash Value Migration

**Problem**

`GravitasReplayHash` duplicates `Chronicler.ChronicleHash`. Keeping both would
muddy the public API and force unnecessary conversions.

**Files**

- Remove or modify: `src/Gravitas/Determinism/GravitasReplayHash.cs`
- Modify: `src/Gravitas/Runtime/GravitasWorldContext.cs`
- Modify: `src/Gravitas/Determinism/GravitasReplayHashService.cs`
- Modify: `tests/Gravitas.Tests/Determinism`
- Modify: `tests/Gravitas.Benchmarks/Core/ReplayHashBenchmarks.cs`

**Tasks**

- [ ] Update failing tests first to expect `ChronicleHash` from
      `ComputeReplayHash(...)`.
- [ ] Change `GravitasWorldContext.ComputeReplayHash(...)` to return
      `ChronicleHash`.
- [ ] Change `GravitasReplayHashService.Compute(...)` to return
      `ChronicleHash`.
- [ ] Delete `GravitasReplayHash.cs` if no strong domain-wrapper need appears.
- [ ] Update replay conformance helpers and tests from `GravitasReplayHash` to
      `ChronicleHash`.
- [ ] Update benchmark return types from `GravitasReplayHash` to
      `ChronicleHash`.
- [ ] Run:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~GravitasReplayHashContextTests|FullyQualifiedName~GravitasReplayConformanceTests"
```

**Done Criteria**

- Public replay hash APIs use `ChronicleHash` directly.
- No compatibility alias exists unless explicitly justified in this plan's
  completion notes.
- Existing conformance semantics still pass with the new value type.

## Workstream 3: Writer And Contributor Migration

**Problem**

`GravitasReplayHashWriter` duplicates Chronicler primitive writing and
FixedMathSharp.Chronicler fixed-point helpers. Gravitas should keep only
physics-domain writer extensions.

**Files**

- Remove or modify: `src/Gravitas/Determinism/GravitasReplayHashWriter.cs`
- Create: `src/Gravitas/Determinism/GravitasChronicleHashWriterExtensions.cs`
- Modify: all `*.ReplayHash.cs` partials and helpers under `src/Gravitas`
- Modify: `tests/Gravitas.Tests/Determinism/GravitasReplayHashWriterTests.cs`

**Tasks**

- [ ] Replace `new GravitasReplayHashWriter()` with
      `new ChronicleHashWriter()`.
- [ ] Add `using Chronicler;` where contributors need the writer or hash value.
- [ ] Add `using FixedMathSharp.Chronicler;` where contributors write
      `Fixed64`, vectors, quaternions, transforms, matrices, bounds, rays, or
      planes.
- [ ] Move `WritePhysicsLayer(...)` and `WritePhysicsLayerMask(...)` into a
      Gravitas-owned extension helper over `ChronicleHashWriter`.
- [ ] Replace local writer fixed-math calls with FixedMathSharp.Chronicler
      extension methods:
  - `WriteFixed64`
  - `WriteVector2d`
  - `WriteVector3d`
  - `WriteVector4d`
  - `WriteQuaternion`
  - `WriteTransform`
  - `WriteFixed3x3`
  - bounds/geometry methods where replay contributors need them.
- [ ] Delete `GravitasReplayHashWriter.cs` after all references are removed.
- [ ] Replace writer-specific tests with Gravitas-domain extension tests:
  - physics layer writes are deterministic.
  - physics layer mask writes are deterministic.
  - section ordering remains order-sensitive through Chronicler's writer.
  - fixed-point raw payload behavior is covered by FixedMathSharp.Chronicler
    tests and does not need duplicate Gravitas assertions.
- [ ] Run:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter Determinism
```

**Done Criteria**

- Gravitas no longer owns a generic primitive hash writer.
- Gravitas replay contributors compile against `ChronicleHashWriter`.
- Domain writer helpers are small, explicit, and physics-specific.

## Workstream 4: `IRecordable` Contribution Evaluation

**Problem**

Chronicler can hash `IRecordable.RecordData(...)`, but Gravitas replay hashing
also includes ordered service state, pair tables, solver caches, and mode-based
policy. The migration should use `ChronicleHashSerializer.Contribute(...)` only
where it preserves the existing replay signal and allocation profile.

**Files**

- Review: `src/Gravitas/Core/3D/SolidBody.ReplayHash.cs`
- Review: `src/Gravitas/Core/2D/SolidBody2D.ReplayHash.cs`
- Review: `src/Gravitas/Colliders/3D/LSCollider.ReplayHash.cs`
- Review: `src/Gravitas/Colliders/2D/LSCollider2D.ReplayHash.cs`
- Review: `src/Gravitas/Settings/*.ReplayHash.cs`
- Test: `tests/Gravitas.Tests/Determinism`

**Tasks**

- [ ] Pick one representative simple Gravitas `IRecordable` state shell, such
      as `PhysicsSettings` or `PhysicsEnvironment`, and compare:
  - current manual contributor hash payload.
  - `ChronicleHashSerializer.Contribute(...)` payload.
- [ ] Add a regression test that documents whether the chosen shell can safely
      use `ChronicleHashSerializer.Contribute(...)`.
- [ ] If serializer contribution preserves intended semantics and does not
      allocate after warmup, use it for that shell.
- [ ] If serializer contribution adds unwanted field-name/type/schema payload or
      allocates, keep manual contributors and document the no-change decision in
      completion notes.
- [ ] Do not route service-owned ordered collections, collider tables,
      collision pairs, warm-start caches, constraint islands, or mixed contact
      state through generic `IRecordable` traversal.
- [ ] Run the replay allocation test after any serializer adoption:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~ComputeReplayHash_Authoritative_ShouldNotAllocateAfterWarmup"
```

**Done Criteria**

- Gravitas intentionally chooses where generic `IRecordable` hashing is useful.
- Manual contributors remain where physics-domain ordering or inclusion policy
  matters.
- Any serializer adoption has correctness and allocation evidence.

## Workstream 5: Docs, Benchmarks, Lean Validation, And Closure

**Problem**

The migration changes public replay hash type names and lower-stack ownership.
Docs, tests, and benchmark guardrails must prove the new public surface is
clear and performance-neutral.

**Files**

- Modify: `docs/wiki/SERIALIZATION.md`
- Modify: `docs/wiki/HOST_INTEGRATION.md`
- Modify: `docs/wiki/DIAGNOSTICS.md` only if replay-hash diagnostics are
  mentioned there.
- Modify: `tests/Gravitas.Benchmarks/README.md` only if benchmark guidance or
  return type wording changes.
- Modify: this plan's completion notes.
- Modify: `docs/feature-work/feature-work-overview.md`

**Tasks**

- [ ] Update wiki docs to describe `ChronicleHash` as the replay/conformance
      hash value returned by Gravitas.
- [ ] Document that hash values are deterministic conformance signals, not
      cryptographic hashes and not stable compatibility promises across package
      version changes.
- [ ] Run full Release tests:

```bash
dotnet test Gravitas.slnx --configuration Release
```

- [ ] Run full ReleaseLean tests:

```bash
dotnet test Gravitas.slnx --configuration ReleaseLean
```

- [ ] Re-run replay-hash benchmark smoke:

```bash
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj --configuration Release -f net8.0
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll replay-hash --filter "*" -j Short -i --exporters json
```

- [ ] Compare benchmark smoke against Workstream 1:
  - no steady-state managed allocations.
  - no obvious timing regression in sparse 3D, dense 3D, pure 2D, mixed, or
    cache-inclusive rows.
  - any noisy timing delta is called out honestly rather than overclaimed.
- [ ] Update this plan with completion notes:
  - public API choice.
  - deleted local files.
  - any retained manual contributors.
  - test results.
  - benchmark/allocation evidence.
- [ ] Mark this plan `Done` and move it to `docs/feature-work/done` after
      review.
- [ ] Move this plan from active release scope to recently completed in
      `docs/feature-work/feature-work-overview.md`.

**Done Criteria**

- Gravitas consumes Chronicler and FixedMathSharp.Chronicler for generic replay
  hash mechanics.
- Gravitas keeps only physics-domain replay inclusion policy and domain writer
  helpers.
- Local `GravitasReplayHash` and `GravitasReplayHashWriter` are removed unless
  a documented implementation finding justifies keeping a wrapper.
- Determinism tests pass across 3D, pure 2D, mixed, CCD, query-cache mutation,
  and Chronicler restore continuation scenarios.
- Release and ReleaseLean validation pass.
- Replay-hash benchmark smoke remains allocation-free after warmup.

## Review Checklist

- [ ] `rg -n "GravitasReplayHash\\b|GravitasReplayHashWriter\\b" src tests`
      returns no active references except historical docs or an explicitly
      justified wrapper.
- [ ] `rg -n "ChronicleHash|ChronicleHashWriter" src tests` shows Gravitas
      contributors using lower-stack hash types directly.
- [ ] `rg -n "FixedMathSharp.Chronicler" src tests` confirms fixed-point writer
      extensions are used instead of duplicated local methods.
- [ ] `GravitasReplayHashMode` remains in Gravitas.
- [ ] All replay contributors keep stable ordering.
- [ ] No hot path uses LINQ, iterator blocks, reflection, runtime object
      identity, or unordered collection traversal.
- [ ] Release and ReleaseLean tests pass.
- [ ] Replay-hash benchmark smoke is no-allocation after warmup.
