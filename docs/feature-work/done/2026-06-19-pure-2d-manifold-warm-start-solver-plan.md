# Pure 2D Manifold And Warm-Start Solver Implementation Plan

**Date:** 2026-06-19
**Status:** Done
**Owner:** Gravitas pure 2D solver hardening

## Purpose

The pure 2D angular dynamics plan gave 2D bodies proper COM, scalar moment,
torque, angular impulses, and off-center contact response. The remaining
solver gap is contact quality: `CollisionPair2D` still resolves one contact per
pair. That is enough for the first angular response milestone, but it is not
strong enough for box/box face contacts, stacked contacts, stable resting
friction, or warm-started response.

This plan gives pure 2D its own manifold and solver helper model instead of
trying to squeeze scalar 2D math through the 3D tensor solver helpers.

## Original Baseline Before This Plan

- `Contact2D` was a single contact with `PointA`, `PointB`, `Normal`, and
  `Depth`.
- `CollisionDetection2D.TryCollide(...)` returned one `Contact2D`.
- `CollisionPair2D` owned pair priority, resting/separated state, wake
  propagation, and notifications.
- `CollisionResponse2D.Resolve(CollisionPair2D pair, Contact2D contact)` read
  `SolidBody2D.EffectiveInverseMass`,
  `SolidBody2D.EffectiveInverseMomentOfInertia`, and
  `SolidBody2D.WorldCenterOfMass`, then applies one-pass positional correction,
  normal impulse, and tangent Coulomb friction impulse.
- 3D already has `ContactManifold`, `ManifoldContact`,
  `ContactWarmStartCache`, `ResponseBody`, `SolverContact`, and
  `SolverContactBuffer`.
- Pure 2D dynamic-vs-dynamic CCD is already implemented through relative circle
  sweeps against prepared dynamic target candidates with stable ordering. CCD
  is not the target of this plan except for preserving existing behavior.

## Design Decisions

- Add 2D-specific manifold and solver helper types instead of genericizing the
  3D tensor path. This keeps vector/scalar math explicit and easier to audit.
- Use a fixed two-contact manifold for pure 2D convex contacts. Circle/circle
  and circle/convex usually produce one contact; convex/convex face contacts
  can produce two. Compound reduction should also reduce to the deepest stable
  two owner-level contacts.
- Contact identity must be deterministic and stable across repeated frames.
  Use quantized/fixed raw contact point values and owner-level collider IDs or
  part-local stable indices where needed. Do not rely on hash iteration order.
- `CollisionPair2D` should own:
  - `ContactManifold2D`.
  - `ContactWarmStartCache2D`.
  - pair lifecycle and notification state.
- `CollisionResponse2D` should own:
  - fixed-order solver-contact construction.
  - optional cached impulse application.
  - deterministic normal and tangent impulse solving.
  - cache update after the solve.
- Warm starting should be real impulse application, not only storage. Keep the
  first implementation bounded and pair-local; full island solving remains a
  separate solver architecture plan.
- Positional correction can remain translation-only, but it must be shared
  across active manifold contacts so a two-contact face does not correct twice.
- Mixed 2D/3D response is not changed by this plan unless a helper extraction
  removes duplication without changing behavior.

## File Map

- Create `src/Gravitas/CollisionHandling/Contacts/ManifoldContact2D.cs`.
- Create `src/Gravitas/CollisionHandling/Contacts/ContactManifold2D.cs`.
- Create `src/Gravitas/CollisionHandling/Contacts/ContactWarmStartCache2D.cs`
  or generalize existing warm-start cache only if the result remains clearer
  than a 2D-specific type.
- Create `src/Gravitas/CollisionHandling/Response/ResponseBody2D.cs`.
- Create `src/Gravitas/CollisionHandling/Response/SolverContact2D.cs`.
- Create `src/Gravitas/CollisionHandling/Response/SolverContactBuffer2D.cs`.
- Modify `src/Gravitas/CollisionHandling/Contacts/Contact2D.cs` if it remains
  as the public single-contact compatibility view.
- Modify `src/Gravitas/CollisionHandling/Detection/CollisionDetection2D.cs`.
- Modify `src/Gravitas/CollisionHandling/Pairs/CollisionPair2D.cs`.
- Modify `src/Gravitas/CollisionHandling/Response/CollisionResponse2D.cs`.
- Modify `tests/Gravitas.Tests/Physics2D/CollisionDetection2DTests.cs`.
- Create `tests/Gravitas.Tests/CollisionHandling/ContactManifold2DTests.cs`.
- Create `tests/Gravitas.Tests/CollisionHandling/CollisionResponse2DManifoldTests.cs`.
- Modify `tests/Gravitas.Tests/CollisionHandling/CollisionResponse2DAngularTests.cs`.
- Modify `tests/Gravitas.Benchmarks/Physics2D/Physics2DBenchmarks.cs`.
- Update `docs/wiki/COLLISION_PIPELINE.md` and `docs/wiki/DIMENSIONS.md`.

## Workstream 1: 2D Manifold Data Model

**Goal:** Add stable fixed-capacity 2D manifold types before changing narrow
phase or response.

Tasks:

- [x] Add failing tests in
  `tests/Gravitas.Tests/CollisionHandling/ContactManifold2DTests.cs` for:
  - empty manifold state.
  - `BeginUpdate(frame)` clearing contacts and recording frame.
  - adding one contact.
  - adding duplicate contact identity with deeper depth replacing the old
    contact.
  - keeping the deepest two contacts.
  - sorting exposed contacts by stable contact identity.
  - selecting `PrimaryContact` by deepest depth, then lowest contact ID.

- [x] Create `ManifoldContact2D` with:
  - `ulong ContactId`.
  - `Vector2d PointA`.
  - `Vector2d PointB`.
  - `Fixed64 Depth`.
  - `Vector2d Normal`.

- [x] Create `ContactManifold2D` with `MaxContactCount = 2`, two stored contact
  fields, `Count`, `HasContact`, `LastUpdatedFrame`, indexer, `BeginUpdate`,
  `Reset`, `SetContact`, `AddContact`, and `PrimaryContact`.

- [x] Use deterministic contact IDs derived from fixed raw contact point values.
  Match the 3D manifold style where practical, but keep the implementation 2D
  and allocation-free.

- [x] Run the focused manifold tests:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter FullyQualifiedName~ContactManifold2DTests
```

Expected result: 2D manifold state is deterministic, fixed-capacity, and tested
without touching the existing solver.

Notes:

- Added `ManifoldContact2D` and `ContactManifold2D` under
  `Gravitas.CollisionHandling`. The 2D manifold mirrors the 3D manifold's
  fixed-storage, deepest-contact replacement, stable ID sorting, and primary
  contact tie-break behavior with two-contact 2D capacity.
- Verified TDD red with the missing `ContactManifold2D` type, then green with
  `dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter FullyQualifiedName~ContactManifold2DTests --nologo`.

## Workstream 2: 2D Narrow-Phase Manifold Generation

**Goal:** Generate stable owner-level 2D manifolds from existing shape pairs.

Tasks:

- [x] Add failing narrow-phase tests for:
  - circle/circle producing one contact.
  - circle/convex producing one contact.
  - convex/convex face overlap producing two contacts on the incident edge.
  - convex/convex corner overlap producing one contact.
  - reversed pair order producing equivalent owner-level contacts with reversed
    normal.
  - compound/primitive selecting owner-level contacts in stable part order and
    reducing to the deepest two.

- [x] Add an internal manifold-building entry point, recommended shape:

```csharp
internal static bool TryCollide(CollisionPair2D pair, ContactManifold2D manifold, int frame)
```

- [x] Keep `Contact2D` only as an internal primitive-candidate helper and remove
  it from the public 2D collision API during final cleanup. Public-facing 2D
  collision semantics should be manifold-first.

- [x] Convert circle/circle and circle/convex paths to write into
  `ContactManifold2D`.

- [x] Implement convex/convex clipping:
  - select the minimum-penetration SAT axis as the contact normal.
  - choose a reference edge on the reference collider.
  - choose the most anti-parallel incident edge on the incident collider.
  - clip the incident segment against the reference edge side planes.
  - emit up to two contacts with deterministic IDs.

- [x] Convert compound paths to collect candidate contacts in stable part order
  and add owner-level contacts to the owner pair manifold.

- [x] Run focused 2D narrow-phase tests:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~CollisionDetection2DTests|FullyQualifiedName~ContactManifold2DTests"
```

Expected result: 2D narrow phase can produce deterministic one- or two-contact
manifolds without changing response behavior yet.

Notes:

- Added the internal pair manifold entry point plus a work-item overload used by
  tests and compound part traversal. The single-contact `Contact2D` path now
  exists only as an internal primitive-candidate helper used by the manifold
  builder.
- Circle/circle and circle/convex now populate `ContactManifold2D`; reversed
  convex/circle owner order writes reversed points and normal without
  allocating a temporary manifold.
- Convex/convex manifold generation now selects the minimum SAT axis, picks a
  reference edge, clips the incident edge against side planes, and emits stable
  owner-level contacts. Edge selection uses the outward `LeftHandNormal` for
  the current counter-clockwise 2D vertex convention.
- Compound manifold generation walks parts in authored order and lets the fixed
  manifold retain the deepest two owner-level contacts. The legacy
  single-contact compound fallback now selects the deepest candidate instead of
  the shallowest candidate.
- Verified TDD red on the missing manifold entry point, then green with:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~CollisionDetection2DTests|FullyQualifiedName~ContactManifold2DTests" --nologo
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter FullyQualifiedName~Physics2D --nologo
```

## Workstream 3: Pair-Owned Manifold And Warm-Start State

**Goal:** Make `CollisionPair2D` own persistent manifold and cached impulses.

Tasks:

- [x] Add failing pair tests proving:
  - `CollisionPair2D` exposes or internally retains a manifold across frames.
  - separated pairs reset active contacts.
  - warm-start cache clears when pairs separate or are reused for different
    collider IDs.
  - resting pairs preserve contact identity when narrow phase still reports the
    same contacts.

- [x] Add `ContactManifold2D` and `ContactWarmStartCache2D` fields to
  `CollisionPair2D`.

- [x] Change `CollisionPair2D.MarkColliding` to accept or use the pair-owned
  manifold rather than a single `Contact2D`.

- [x] Keep pair notification and wake ownership in `CollisionPair2D`.

- [x] Ensure pair pool reuse calls reset manifold and cache state.

- [x] Run focused pair and 2D simulation tests:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~CollisionPair2D|FullyQualifiedName~Physics2DSimulationTests|FullyQualifiedName~Collider2DStateParityTests"
```

Expected result: pair lifecycle owns the data needed for manifold response and
warm starting without changing event semantics.

Notes:

- Added `ContactWarmStartCache2D` as a fixed two-slot cache keyed by
  `ManifoldContact2D.ContactId`, reusing the existing scalar
  `ContactWarmStartImpulse` payload.
- `CollisionPair2D` now owns `ContactManifold2D` and warm-start state. Pair
  initialization, separation, and pooled reuse clear both manifold and cache.
- `CollisionPair2D.MarkColliding(frame)` now consumes the pair-owned manifold;
  the legacy `MarkColliding(frame, Contact2D)` path and the unused
  `CollisionDetection2D.TryCollide(CollisionPair2D, out Contact2D)` overload
  were removed.
- `GravitasPhysics2DService` now creates/reuses a pair before narrow phase for
  active candidates and writes detection output directly into
  `pair.Manifold`. Resting-pair preservation refreshes the same manifold before
  `MarkResting(frame)`.
- Verified TDD red on missing pair-owned manifold/cache APIs, then green with:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~CollisionPair2D|FullyQualifiedName~Physics2DSimulationTests|FullyQualifiedName~Collider2DStateParityTests" --nologo
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter FullyQualifiedName~Physics2D --nologo
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~CollisionResponse2DAngularTests|FullyQualifiedName~CollisionPair2DManifoldTests|FullyQualifiedName~CollisionDetection2DTests|FullyQualifiedName~ContactManifold2DTests" --nologo
```

## Workstream 4: 2D Solver Helper Types And Manifold Response

**Goal:** Replace the single-contact 2D response path with bounded manifold
response through explicit 2D solver helpers.

Tasks:

- [x] Add failing response tests for:
  - two symmetric contacts on a box face not injecting angular velocity.
  - off-center single contact still spinning the body.
  - manifold positional correction applying once per pair, not once per contact.
  - friction impulses at two contacts opposing tangential motion.
  - angular-disabled, immovable, kinematic, inactive, and zero-mass bodies
    contributing zero inverse mass/moment.
  - cached impulses being applied on a repeated contact before the fresh solve.

- [x] Create `ResponseBody2D` from a collider:
  - body reference.
  - effective inverse mass.
  - effective inverse scalar moment.
  - `CanTranslate`.
  - `CanRotate`.

- [x] Create `SolverContact2D` with:
  - contact ID.
  - `ResponseBody2D` A/B.
  - point A/B.
  - relative contact arms.
  - depth.
  - normal.
  - tangent.
  - cached normal/tangent impulse values.

- [x] Create `SolverContactBuffer2D` with fixed capacity matching
  `ContactManifold2D.MaxContactCount`.

- [x] Update `CollisionResponse2D.Resolve(...)` to:
  - build response bodies.
  - build solver contacts from the pair manifold.
  - apply translation-only positional correction shared by contact count.
  - apply cached normal/tangent impulses when matching cache entries exist.
  - solve normal impulses in stable contact order.
  - solve tangent friction impulses in stable contact order.
  - update the pair warm-start cache with final impulse scalars.

- [x] Keep solver iteration count fixed and internal. Recommended first value:
  one warm-start application plus one deterministic normal/friction pass over
  the bounded contacts. Increase only with tests and benchmarks showing the
  physics benefit.

- [x] Run focused response tests:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~CollisionResponse2DManifoldTests|FullyQualifiedName~CollisionResponse2DAngularTests"
```

Expected result: pure 2D response consumes manifolds and warm-start cache while
preserving the scalar COM/angular behavior from the previous plan.

Notes:

- Added explicit pure 2D response helpers:
  `ResponseBody2D`, `SolverContact2D`, and `SolverContactBuffer2D`.
  `ResponseBody2D` uses `SolidBody2D.EffectiveInverseMass` and
  `EffectiveInverseMomentOfInertia`, so angular-disabled, immovable,
  kinematic, inactive, and zero-mass bodies contribute zero solver mass/moment.
- Replaced the legacy primary-contact response path with bounded manifold
  response. `CollisionResponse2D.Resolve(pair)` now builds solver contacts from
  `pair.Manifold`, shares positional correction across the active contact
  count, applies cached normal/tangent impulses first, performs one stable
  normal pass and one stable tangent-friction pass, and refreshes pair-owned
  warm-start cache entries by contact ID.
- Normal impulses are accumulated and clamped at zero after warm start, so
  stale cache entries can unwind instead of injecting artificial separation.
  Tangent impulses are likewise clamped to the current Coulomb bound and unwind
  when the normal impulse falls to zero.
- Single-contact angular response is preserved through one-contact manifolds;
  symmetric two-contact box-face response no longer injects artificial angular
  velocity.
- Verified with:

```bash
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~CollisionResponse2DManifoldTests|FullyQualifiedName~CollisionResponse2DAngularTests" --nologo
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~CollisionPair2D|FullyQualifiedName~Physics2DSimulationTests|FullyQualifiedName~Collider2DStateParityTests" --nologo
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter "FullyQualifiedName~CollisionDetection2DTests|FullyQualifiedName~ContactManifold2DTests" --nologo
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --filter FullyQualifiedName~Physics2D --nologo
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj --configuration Release --nologo
```

## Workstream 5: Benchmarks, Docs, And Release Validation

**Goal:** Document the new 2D solver contract and prove it does not regress the
hot path unexpectedly.

Tasks:

- [x] Extend `tests/Gravitas.Benchmarks/Physics2D/Physics2DBenchmarks.cs` with
  benchmark coverage for convex/convex two-contact manifold detection and
  two-contact manifold response.

- [x] Update `docs/wiki/COLLISION_PIPELINE.md` to describe:
  - `ContactManifold2D`.
  - two-contact 2D convex manifolds.
  - pair-owned warm-start cache.
  - `ResponseBody2D`, `SolverContact2D`, and `SolverContactBuffer2D`.
  - remaining solver limits: no island solver, no static resting friction, and
    no rotational CCD.

- [x] Update `docs/wiki/DIMENSIONS.md` if public 2D contact semantics change.

- [x] Run benchmark build/list validation:

```bash
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj --configuration Release -f net8.0
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll physics-2d --list flat
```

- [x] Run full Release validation:

```bash
dotnet build Gravitas.slnx --configuration Release
dotnet test Gravitas.slnx --configuration Release
```

- [x] Run full ReleaseLean validation:

```bash
dotnet build Gravitas.slnx --configuration ReleaseLean
dotnet test Gravitas.slnx --configuration ReleaseLean
```

Expected result: docs describe pure 2D as a manifold-capable solver, benchmarks
cover the new costs, and both package paths pass.

Notes:

- Added `DetectConvexConvexTwoContactManifolds` and
  `ResolveTwoContactManifoldPairs` to the `physics-2d` benchmark selection, and
  updated existing 2D and 2D-compound benchmark collision checks to use
  reusable `ContactManifold2D` state instead of the old single-contact
  convenience path.
- Final cleanup demoted `CollisionDetection2D` and `Contact2D` to internal
  implementation surfaces. `Contact2D` now remains only as a primitive
  candidate helper inside the 2D manifold builder; public-facing 2D collision
  semantics are manifold-first.
- Updated `docs/wiki/COLLISION_PIPELINE.md`, `docs/wiki/DIMENSIONS.md`,
  `docs/wiki/OVERVIEW.md`, and the benchmark README to describe pure 2D
  two-contact manifolds, pair-owned warm-start cache, explicit 2D solver helper
  types, and the remaining solver limits.
- Verified with:

```bash
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj --configuration Release -f net8.0 --nologo
dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll physics-2d --list flat
dotnet build Gravitas.slnx --configuration Release --nologo
dotnet test Gravitas.slnx --configuration Release --nologo
dotnet build Gravitas.slnx --configuration ReleaseLean --nologo
dotnet test Gravitas.slnx --configuration ReleaseLean --nologo
```

## Exit Criteria

- Pure 2D collision pairs own deterministic fixed-capacity contact manifolds.
- Convex/convex pure 2D contacts can produce up to two stable contact points.
- Compound 2D contacts reduce part contacts in stable owner-level order.
- `CollisionResponse2D` uses `ResponseBody2D`, `SolverContact2D`, and
  `SolverContactBuffer2D`.
- Pair-local warm-start cache is applied and refreshed for persistent contacts.
- Existing pure 2D COM, angular impulse, CCD, query, trigger, contact event, and
  serialization tests continue to pass.
- Release and ReleaseLean validation pass.
