# Gravitas Contributor Guide

## Purpose

Gravitas is a framework-agnostic deterministic physics library for lockstep simulations and games. The library currently targets `netstandard2.1` and `net8.0`, uses fixed-point math via `FixedMathSharp`.

Current priorities:

1. Preserve deterministic behavior first.
2. Reduce time complexity and avoid unnecessary allocations in hot paths.
3. Fix correctness issues before broad refactors.
4. Add XML documentation and concise comments for non-obvious logic.
5. Close test coverage gaps and keep the suite reliable in `Release`.

## Start Here

Read these in order before making non-trivial changes:

1. [`README.md`](README.md)
3. The relevant source folder under [`src/Gravitas`](src/Gravitas)
4. The matching test area under [`tests/Gravitas.Tests`](tests/Gravitas.Tests)
5. [`src/Gravitas/Gravitas.csproj`](src/Gravitas/Gravitas.csproj) and [`tests/Gravitas.Tests/Gravitas.Tests.csproj`](tests/Gravitas.Tests/Gravitas.Tests.csproj)

## Source of Truth

When code and docs disagree, prefer the code.

Keep these aligned whenever behavior or public API changes:

- [`README.md`](README.md)

## Repository Map

| Path | Purpose | Notes |
| --- | --- | --- |
| [`docs`](docs) | N/A |
| [`src/Gravitas`](src/Gravitas) | Main library project | Multi-targets `netstandard2.1` and `net8.0`. |
| [`tests/Gravitas.Tests`](tests/Gravitas.Tests) | xUnit v3 test project | Uses FluentAssertions, Moq, FixedMathSharp, GridForge. |

Ignore generated output when reviewing structure:

- `bin/`
- `obj/`
- `TestResults/`
- `artifacts/`
- `.vs/`

## Runtime Architecture

N/A

## Serialization Status

Gravitas currently uses the Chronicler serialization.

Important current rules:

- Gravitas serializes through explicit `IRecordable.RecordData(...)` implementations rather than relying on serializer attributes for runtime graphs.
- The active transports are `JsonRecordSerializer` and `MemoryPackRecordSerializer`.
- The current Gravitas coverage is: none
- The load model is populate-existing-instance only. Hosts create and initialize runtime shells first, then Chronicler populates supported state.
- Gravitas intentionally does not use Chronicler as a construct-from-data object factory.
- Host bindings are not serialized.

## External Dependencies

The main external packages shape how this project should be changed:

- `FixedMathSharp`: fixed-point math and deterministic vector/quaternion types.
- `GridForge`: voxel grids, spatial queries, global grid management, and chart backing data.
- `SwiftCollections`: dictionaries, lists, queues, object pools, and related low-allocation collection types.

Do not casually replace these with standard floating-point or non-deterministic alternatives.

## Determinism Rules

Any change that affects simulation order, iteration order, rounding, path scoring, or update timing is high risk.

Always prefer:

- `Fixed64`, `Vector3d`, and `FixedQuaternion` over `float`, `double`, and `System.Numerics`.
- Frame-based reasoning through `GravitasManager.FrameRate`, `DeltaTime`, and `FrameCount`.
- Stable and explicit ordering when cache keys, path scoring, or traversal decisions depend on iteration.
- Existing lockstep-friendly patterns over convenience shortcuts.

Avoid introducing:

- Floating-point math in simulation logic.
- Time-dependent APIs such as `DateTime.Now`, timers, or wall-clock scheduling in runtime code.
- Randomness without a deterministic seed and explicit ownership.
- Hidden allocations or LINQ in per-frame or per-node hot paths unless a benchmark or profile justifies it.
- Changes that make results depend on platform-specific collection ordering.

## Coding Style and Documentation

Observed project conventions:

- `LangVersion` is `11.0`.
- `ImplicitUsings` are disabled.
- Library nullable context is disabled; tests use nullable enabled.
- XML doc output is generated for the library, but warning `1591` is suppressed.
- Namespace-folder matching is not enforced.

Contributor expectations for code and docs:

- Add or improve XML `<summary>` tags for public and externally meaningful internal APIs when touching them.
- Add brief comments only where the logic is hard to infer from the code alone.
- Preserve ASCII unless the file already requires otherwise.
- Keep comments factual. Explain invariants, edge conditions, or reasons behind tricky logic.
- Do not add comment noise around obvious assignments or straight-line code.
- Split reusable or generic infrastructure into focused types and files instead of bundling it into an unrelated runtime class. Prefer one primary type per file unless the extra type is tightly scoped and truly private to that implementation.
- Prefer `SwiftCollections` over `System.Collections*` types when a suitable collection already exists there, especially in runtime or hot-path code. If you intentionally keep a BCL collection, the reason should be obvious from the code or called out in review.

## Performance Guidance

Optimization work should focus on proven hot paths and data-structure behavior, not cosmetic micro-tuning.

Likely hotspots:

- N/A

Optimization rules:

- Preserve path correctness before reducing allocations.
- Do not knowingly land avoidable steady-state inefficiencies in new runtime or pathing infrastructure with the expectation of "optimizing it later"; new stateful runtime code should start lean in both allocation behavior and update complexity.
- Pool only when lifetime management stays obvious and testable.
- Be careful with cache invalidation; stale guide reuse is worse than a small allocation.
- Avoid broad refactors across pathing and navigation in one change set.
- If complexity changes, add or update tests that pin the edge cases affected by the new logic.

## Testing Workflow

Use these baseline commands:

```bash
dotnet restore Gravitas.slnx
dotnet build Gravitas.slnx --configuration Release
dotnet test Gravitas.slnx --configuration Release
```

Important note:

- Building the library also produces NuGet packages because `GeneratePackageOnBuild` is enabled in the library project.

For focused work, prefer targeted runs first, then a full solution run:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter FullyQualifiedName~Gravitas
```

## Test Design Expectations

Tests should mirror the runtime area being changed.

Current coverage is strongest around:

- N/A

Coverage appears lighter or absent around:

- N/A

## Recommended Change Workflow

For both humans and AI agents, use this order:

1. Read the relevant doc page and the touched source file.
2. Read the matching tests before changing the implementation.
3. Identify deterministic invariants and global-state implications.
4. Make the smallest coherent code change that addresses the issue.
5. Add or update tests in the same change.
6. Add XML docs or clarifying comments while the code is open.
7. Run focused tests.
8. Run the full `Release` suite before closing the work.
9. Update `README.md` or `docs/*` if public behavior or developer workflow changed.
10. If serialization behavior or load semantics changed, update both serialization docs in the same pass.

## Guidance for AI Agents

If you are an automated coding agent working in this repository:

- Do not trust high-level docs blindly; validate against the code and tests.
- Do not broaden scope from one subsystem into another unless the change truly requires it.
- Call out any build or test failures explicitly, with exact file references.
- Treat cache invalidation, chart ownership, partition reuse, and static manager state as high-risk areas.
- Treat serialization boundaries and load semantics as high-risk areas. Avoid silently broadening from populate-existing-instance loads into construct-from-data behavior.
- Prefer focused edits plus verification over sweeping cleanup.
- If you change a public API or behavior, update both tests and docs in the same pass.
- If you add comments, comment the invariant or the reason, not the syntax.
- Do not leave generic helpers buried inside unrelated classes when they can stand alone as reusable support types.
- Reach for `SwiftCollections` first before introducing `System.Collections`, `System.Collections.Generic`, or `System.Collections.Concurrent` into library code.

## Guidance for Human Contributors

This codebase is small enough that local consistency matters more than abstract purity.

Prefer:

- mirror source/test naming when adding files
- focused patches over broad folder-wide rewrites
- release-mode verification for pathing/navigation behavior
- documenting assumptions about voxel topology, unit size, and line-of-sight rules

Be especially careful when changing:

- path cache keys
- partition ownership and neighbor binding
- locomotion transitions
- stop/arrival thresholds
- line-of-sight shortcut logic
- any logic guarded by `#if DEBUG`
