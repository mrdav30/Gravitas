# Context Reset Reference Release

**Status:** Complete; GRV-Issue-144 resolved on 2026-10-29.
**Issue:** GRV-Issue-144. **Date:** 2026-10-29.

## Defect And Reproduction

On Gravitas `587c60a`, a retained `GravitasWorldContext` kept retired collider,
body and host graphs alive after `Reset()`. This was bounded retention through
reusable backing capacity, rather than an unbounded allocation leak or an
incorrect collision result. The tracker originally verified the defect on
`3f0b759` before the static-surface Phase 9 changes.

The maintained
[`ContextResetLifetimeTests`](../../../tests/Gravitas.Tests/Runtime/ContextResetLifetimeTests.cs)
reproduce it through public lifecycle operations. A no-inline factory registers
objects, optionally completes host steps and replay inspection, then returns
only weak references. The caller resets the still-live context and runs
`GC.Collect()`, `GC.WaitForPendingFinalizers()`, `GC.Collect()`. The fixed code
releases every tracked graph, preserves the empty replay hash and supports a
second session. `GC.KeepAlive` retains the context across the collection check.

All 18 cases fail against the frozen committed runtime specifically because
retired targets remain alive. All 18 pass with the fix:

- Registry-only, completed-contact and large-then-small workloads in ThreeD,
  TwoD, Both and Mixed. The shrinking workload first uses five groups, then
  deactivates four and completes a smaller step and replay snapshot.
- A sphere or equivalent zero-core capsule against two declared static 2D
  rectangles in Mixed mode. The declaration handle stays alive across reset;
  controls also dispose it before reset. Declaration cleanup alone cannot
  release context-owned roots.
- Moving three-body continuous-collision chains in pure 2D and 3D, confirming
  that a handoff island actually executes before a later step and reset.

The contact fixtures run at 16 Hz with zero gravity, air density, damping and
minimum speed. Movers have unit mass, disabled sleep and frozen rotation;
native 3D movers use manual grounding. The 2D cases exercise automatic grounding
snapshots. The CCD chains run at 1 Hz, with bodies initially at X=`0,2,-5` and
an X impulse of `10` on the last body. The maintained source records all other
shape, wall and host inputs.

## Ownership And Fix

`ColliderRegistry.Clear()` used `FastClear()` on the live registry and replay
snapshot. The dimensional physics services likewise reset reference-bearing
work lists by count. `FastClear()` leaves backing slots intact. A later smaller
snapshot can overwrite only its prefix, so even ordinary `Clear()` at reset
would miss an older tail beyond the current count.

The fix stays in four existing Gravitas owners, adding 33 runtime lines and
removing seven. It introduces no fields, helpers, production files, dependencies
or public APIs:

- Live registries, service-refresh lists, declaration owner lists, and the 3D
  active-pair queue/cached-pair stack use ordinary `Clear()`. Their normal use
  preserves the populated-slot boundary.
- Replay snapshots, response/pending pair buffers, island nodes and constraints,
  dirty CCD body lists and handoff arrays clear their entire backing arrays at
  reset. Both 2D grounding snapshots receive the same cleanup.
- Handoff work is discarded before its references are cleared. Declarations
  release their member/mover state before their owner lists are cleared.
- Numeric IDs, bounds and candidate-index entries retain their existing fast
  clearing. Reference-free partition memberships and correctly cleared query,
  constraint and coroutine owners require no rewrite.

Capacity and normal simulation clearing are unchanged. Reset intentionally
costs work proportional to retained reference-buffer capacity where a smaller
previous count cannot describe all populated slots. It remains a host-coordinated
session boundary. Caller-owned results and retained host objects continue to
own their own references.

Reset already creates a new clock lifetime token to invalidate old waits. Its
24-byte allocation on this runtime is preserved. The reuse regressions compare
populated reset allocation with a warmed empty reset; releasing populated
reference storage adds no allocation.

## Reset Cost

The maintained
[`WorldContextResetBenchmarks`](../../../tests/Gravitas.Benchmarks/Runtime/WorldContextBenchmarks.cs)
prepares registrations, one complete host step and replay inspection outside
measurement, then times `ResetPopulatedContext()` alone. It reuses the same
context and body shells; setup deactivates the old shells and reinitializes
them with their existing host agents between measured resets. Initial fixture
calibration rejected fresh host-agent rebinding and an already-active 2D body;
those setup errors were corrected before the accepted comparison.

**Each value is median microseconds for one populated context reset, not a
physics step. `BodyCount` is per active dimension:** Both and Mixed contain
32 or 256 movers in total. Each active dimension also has one bodyless floor.
The 3D floor is a 32-by-2-by-32 cuboid centered at `(0,-1,0)`; the 2D floor is
a 32-by-2 rectangle centered at X/Z=`(0,-16)`. Movers are radius-0.5 spheres
and circles in eight columns at spacing `2`, with zero gravity, air density,
damping and minimum speed at 16 Hz. Rotation is frozen and sleep disabled;
3D uses manual grounding and 2D exercises automatic grounding. No authored
declaration is needed for this cost control.

| Mode | Movers per dimension | Committed us/reset | Fixed us/reset | Change us/reset | Change |
| --- | ---: | ---: | ---: | ---: | ---: |
| ThreeD | 16 | 256.40 | 263.27 | +6.88 | +2.7% |
| ThreeD | 128 | 362.41 | 364.07 | +1.66 | +0.5% |
| TwoD | 16 | 1.02 | 2.55 | +1.53 | +148.8% |
| TwoD | 128 | 41.06 | 44.49 | +3.43 | +8.3% |
| Both | 16 | 259.41 | 262.15 | +2.74 | +1.1% |
| Both | 128 | 413.08 | 421.89 | +8.81 | +2.1% |
| Mixed | 16 | 582.21 | 589.41 | +7.20 | +1.2% |
| Mixed | 128 | 1,056.29 | 1,066.72 | +10.44 | +1.0% |

All 32 accepted measurement processes completed: eight workloads, two frozen
runtime payloads and two fresh processes per variant/workload, reversing
payload order on the second run. Each process performs four complete warmup
cycles, then nine samples of four prepared resets. Medians pool 18 samples per
variant/workload. Setup, simulation, replay inspection and shell deactivation
are outside the timed and allocation-counted reset. `DOTNET_TieredCompilation=0`
selects fully optimized JIT code. No builds, tests or profilers competed.

Only `Gravitas.dll` differs between payload directories. Actual loaded runtime
SHA-256 identities match the frozen variants: committed `15B6D539B50F91F8...`,
fixed `86921FB1FDDAF3F2...`. Benchmark and FixedMathSharp hashes match in every
process. Every measured populated replay hash matches its corresponding cycle
across variants and runs; each reset restores the same empty hash. Warm
allocation is exactly 24 bytes per reset in every process, preserving the clock
token baseline. The full suites' existing simulation allocation guards pass.

The reference release necessarily adds reset-boundary memory clearing. Its
absolute cost is small in these controls, while the tiny TwoD/16 baseline makes
the percentage large. Small timing differences include local measurement noise;
these are paired controls, not BenchmarkDotNet confidence intervals, gameplay
frame percentiles or proof for arbitrary retained capacities. No normal-step
algorithm or clearing path changes. All eight in-process BenchmarkDotNet Dry
rows execute; their cold-start timings/allocations are launch checks, separate
from the warm comparison above.

## Validation And Limits

The focused selection passes all 27 cases: 18 new lifetime cases plus the
existing shared replay tests. Replay expectations are unchanged. Both full
suites pass without skips, with exact raw coverage:

| Profile | Tests passed | Raw lines covered | Raw branches covered | Methods covered |
| --- | ---: | ---: | ---: | ---: |
| Release | 8,183 / 8,183 | 53,409 / 53,409 | 21,604 / 21,604 | 5,594 / 5,594 |
| ReleaseLean | 8,112 / 8,112 | 53,407 / 53,407 | 21,604 / 21,604 | 5,593 / 5,593 |

The `netstandard2.1` library and net10 benchmark builds pass with zero warnings.
Independent `ponytail-review` found no remaining collider/body roots or
actionable source/test/documentation defects in the connected reset paths.

Validation uses Windows x64, .NET `10.0.10`, SDK `10.0.302`,
`UseLocalLsfStack=true`, Gravitas base `587c60a`, FixedMathSharp `03f6cbf`,
SwiftCollections `89dfa23`, GridForge `89c69f6` and Chronicler `c547e5c`.
Only Gravitas changes. Package restore was not repeated; the #145 closeout
records `NU1101` for `Chronicler.Diagnostics` in the configured feeds. No release
or cross-platform conformance claim is made.

Run the maintained regressions and benchmark launch check from the repository
root:

```powershell
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj -c Release -p:UseLocalLsfStack=true --filter FullyQualifiedName~ContextResetLifetimeTests
dotnet test tests/Gravitas.Tests/Gravitas.Tests.csproj -c ReleaseLean -p:UseLocalLsfStack=true --collect:'XPlat Code Coverage' --settings tests/Gravitas.Tests/coverlet.runsettings
dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -p:UseLocalLsfStack=true
$env:UseLocalLsfStack = 'true'
dotnet tests/Gravitas.Benchmarks/bin/Release/net10.0/Gravitas.Benchmarks.dll --filter '*WorldContextResetBenchmarks*' --job Dry --inProcess
```

Supplemental logs, raw coverage and frozen runtime payloads are under
`artifacts/grv-144/`. The maintained regressions, inputs, revisions and results
in this record remain sufficient when temporary captures are removed. Reopen
for an ended collider/body/host graph retained solely by a reset context,
broken reuse or changed warm allocation, keeping the exact lifecycle fixture
and dependency revisions.

## Explicit Collection-Owner Cleanup Follow-Up (2026-10-30)

The owner requested `Clear(bool clearRetainedReferences)` across SwiftCollections
after reviewing the raw array cleanup above. Parameterless `Clear()` and
`FastClear()` keep their existing contracts; explicit `true` releases retained
reference-containing storage without shrinking capacity. The option is available
on all 12 public core collection types, including inherited bidirectional and
observable behavior. Numeric-only collections avoid extra reference scans.

Gravitas now calls `Clear(clearRetainedReferences: true)` in the registry,
dimensional reset owners and existing 2D static-surface invalidation. This
replaces backing-array access and paired bookkeeping calls. The final Gravitas
runtime diff from `587c60a` is five existing files, 35 added and 32 removed
lines; the retained-reference fix still introduces no runtime fields or helper
owners. The original manual-cleanup timing table remains historical evidence.

SwiftCollections preserves existing virtual `Clear()` dispatch by cleaning
retained tails before clearing the live range. That retains custom overrides,
observable notifications, and bidirectional reverse-map cleanup. Queue cleanup
handles wrapped storage directly. Generational-bucket clearing preserves
generation history; restoration derives allocation high-water from live/free
slots and serialized `Peak`, so a cleared-state snapshot preserves subsequent
handle assignments. A discriminating regression failed with `Handle(2:0)`
after restore instead of the direct `Handle(0:1)` before that correction.

This is coordinated unreleased source work. Gravitas now requires the new
SwiftCollections API. Its CI dependency pin still identifies `89dfa23`; the
owner must commit SwiftCollections and update that pin to the actual immutable
commit before consumer CI can use this API. Release SwiftCollections before
validating/releasing Gravitas against packages. No placeholder SHA, version
change or automatic commit is included.

The final tests pass without skips and retain exact reachable coverage:

| Library/profile | Tests passed | Raw lines covered | Raw branches covered | Methods covered |
| --- | ---: | ---: | ---: | ---: |
| SwiftCollections Release | 1,404 / 1,404 | 5,319 / 5,319 | 2,362 / 2,362 | 1,144 / 1,144 |
| SwiftCollections ReleaseLean | 1,376 / 1,376 | 5,305 / 5,305 | 2,362 / 2,362 | 1,137 / 1,137 |
| FixedMathSharp companion Release | 43 / 43 | 121 / 121 | 62 / 62 | 35 / 35 |
| FixedMathSharp companion ReleaseLean | 43 / 43 | 121 / 121 | 62 / 62 | 35 / 35 |
| Gravitas Release | 8,183 / 8,183 | 53,386 / 53,386 | 21,604 / 21,604 | 5,594 / 5,594 |
| Gravitas ReleaseLean | 8,112 / 8,112 | 53,384 / 53,384 | 21,604 / 21,604 | 5,593 / 5,593 |

The 35 new collection cases cover all collection families, old reference tails,
empty/smaller refills, wrapped queues, offset sorted storage, reference-containing
structs, comparers/reuse, custom and observable dispatch, generational handle
invalidation, immediate cleared-state restoration, value collection, and zero
warm allocation. Shared Gravitas replay expectations remain unchanged.
Independent review found and verified the generational-state correction, then
found no remaining source/test blocker.

Initial measurements and JIT inspection showed an avoidable empty-list cost:
parameterless `SwiftList.Clear()` still looked up generic reference metadata
and called `Array.Clear` with length zero. The final implementation checks
`Count` before that work while retaining count/version updates and virtual
dispatch. This changes neither the reference-release contract nor `FastClear`.

Both SwiftCollections solution profiles and the Gravitas library/benchmark
targets build successfully. SwiftCollections' unchanged benchmark project emits
the existing `NU1510` warning about its `System.Text.Json` package reference;
Gravitas builds have no warnings. Captures for this follow-up are under
`artifacts/grv-144/swift-clear/`.

The maintained `RetainedReferenceClearBenchmarks` in SwiftCollections' existing
`ConditionalClear.IntegerString.Benchmarks.cs` measures a reference list at
capacities 128 and 16,384, with 0 or 16 live elements. Each operation refills
that live prefix before either the manual whole-array clear plus `FastClear`,
the explicit API, ordinary `Clear`, or `FastClear`. Setup initially fills the
capacity then fast-clears it; subsequent clears write the same memory ranges
even when those slots already contain nulls. These are clearing/refill kernels,
not full simulation or object-graph collection measurements.

| Capacity | Live count | Manual full cleanup ns/op | Explicit API ns/op |
| ---: | ---: | ---: | ---: |
| 128 | 0 | 31.85 | 31.59 |
| 128 | 16 | 99.74 | 101.47 |
| 16,384 | 0 | 4,004.80 | 3,984.45 |
| 16,384 | 16 | 4,074.19 | 4,143.95 |

The kernel comparison uses 48 fresh processes: original/current ordinary and
fast clearing plus current manual/API cleanup, each at all four input pairs,
with two reversed-order runs. Every process warms 256 invocations, then measures
nine samples of 8,192 invocations. Medians pool 18 samples per case. All measured
allocations are zero. The original and current benchmark assembly hashes match;
loaded SwiftCollections hashes identify the selected payloads.

Ordinary empty-list clearing decreases from `8.14-8.17` to `4.36-4.47` ns/op
in these controls. Populated ordinary clearing and `FastClear` remain close to
their original controls. Full reference cleanup is comparable to the manual
operation; the differences above do not establish a general speedup. JIT
inspection identifies the removed empty generic-metadata lookup/zero-length
clear. Direct field access itself is not the demonstrated improvement.

The final host-boundary comparison adds a control using the manual Gravitas
implementation with current SwiftCollections. This separates library/default
clearing changes from the new call sites. Units and populations match the
earlier reset fixture: microseconds per populated reset, with movers counted
per active dimension.

| Mode | Movers per dimension | Manual + original Swift us/reset | Manual + current Swift us/reset | Explicit API + current Swift us/reset |
| --- | ---: | ---: | ---: | ---: |
| ThreeD | 16 | 263.02 | 265.85 | 268.70 |
| ThreeD | 128 | 384.10 | 386.57 | 390.45 |
| TwoD | 16 | 2.67 | 2.40 | 2.51 |
| TwoD | 128 | 48.66 | 47.05 | 48.36 |
| Both | 16 | 276.10 | 274.82 | 272.11 |
| Both | 128 | 450.67 | 451.96 | 463.80 |
| Mixed | 16 | 598.50 | 601.42 | 606.44 |
| Mixed | 128 | 1,136.59 | 1,110.62 | 1,105.09 |

All 48 final reset processes complete: eight workloads, three payload
combinations and two reversed-order runs. Each process warms four prepared
resets and measures nine samples of four resets. Populated replay hashes match
at every corresponding cycle across combinations/runs, and every reset restores
the same empty hash. Allocation stays exactly 24 bytes per reset. Actual loaded
Gravitas and SwiftCollections hashes are verified against the selected payloads;
benchmark and FixedMathSharp hashes match across all runs. The final
SwiftCollections hash starts `47FAF7C326ECE5B5`; the migrated Gravitas hash starts
`B4086B7E1BE35ECB`. Original hashes remain recorded in the earlier captures.

These controls include small positive and negative host-boundary differences;
they do not establish a general reset speedup or exact performance equivalence.
The API centralizes reference ownership and removes raw backing-array plumbing,
with the measured empty-list improvement above. All 96 final kernel/reset
processes use fully optimized JIT code with no competing builds/tests/profilers.
Earlier exploratory measurements are preserved, including the pre-empty-check
candidate and a reset capture lacking explicit loaded-Swift hashing; they are
not substituted for this final comparison. All 16 collection-kernel and eight
reset BenchmarkDotNet in-process Dry rows execute; cold Dry statistics are
launch checks only.
