# Benchmark Host And Native Crash Closeout

**Date:** 2026-10-30.
**Status:** GRV-Issue-111 fixed in source; GRV-Issue-125 investigation closed as not reproduced.

Baseline source is Gravitas `7c152db`, FixedMathSharp `d0d3160` and Chronicler
`c547e5c` on Windows x64, .NET SDK 10.0.302. Current jobs use .NET 10.0.10; historical crash replays
explicitly retain .NET 8.0.29. Builds, launches and diagnostic runs are serialized.
No authoritative physics or math source changes in this closeout.

## GRV-Issue-111

### Reproduction And Cause

From the repository root after a local-stack Release build, run:

```powershell
dotnet tests/Gravitas.Benchmarks/bin/Release/net10.0/Gravitas.Benchmarks.dll --filter '*RotationalCcdIntervalBenchmarks*' --job Dry --keepFiles
```

The current baseline first fails project discovery: BenchmarkDotNet 0.15.8
recursively searches the solution tree and finds the canonical benchmark project
and multiple ignored historical snapshots with the same filename. Changing the
working directory does not avoid its solution-root search. Zero cases execute
and the launcher returns one.

After selecting the canonical project alone, the original #111 failure is
reproduced: generated delegates mentioning `Fixed64` and `FixedQuaternion`
fail with `CS0246`. The generated host has one direct benchmark-project reference
and disables implicit transitive source references. This suppression protects
the explicitly versioned sibling edges, but does not give the host the compile
references needed by dependency-typed benchmark signatures.

### Repair And Scope

The benchmark assembly records its own project path through MSBuild-generated
assembly metadata. A small `CsProjGenerator` override selects that exact project
and, in local-stack mode, adds references to the non-framework dependencies
actually resolved by the launcher. The references also participate in copying
and dependency metadata: compile-only references built successfully but left
the first child unable to load its runnable type. That failed attempt has no
accepted workload results.

The custom toolchain retains the standard builder, executor and complete
standard toolchain validation, replacing only its generator. The existing job
mutator still propagates configuration and local-stack settings without
overriding `--job Dry`. The override is restricted to ordinary .NET 10 jobs and
explicit current-runtime requests. Custom CLI and package paths are forwarded;
in-process, CoreRun, CLR and other requested runtimes retain their original
toolchains. The benchmark project itself currently targets only `net10.0`.

Source mode preserves `UseLocalLsfStack=true`, disabled implicit transitive
project references and serialized builds. There are no dependency pin changes,
global graph relaxations or runtime assembly-resolution hooks.

The final real Lean job exposed a separate blocking configuration defect:
`Chronicler.Diagnostics` declared `ReleaseLean` but inherited the SDK's
unoptimized default for custom configurations. Its owning project now explicitly
enables optimization for that configuration. Debug and ordinary Release settings
remain unchanged. Running the actual empty-world frame in Lean verifies this
dependency setting with the optimization validator enabled.

The implementation uses BenchmarkDotNet's supported generator and validation
extension points. See its version-pinned [project generator](https://github.com/dotnet/BenchmarkDotNet/blob/v0.15.8/src/BenchmarkDotNet/Toolchains/CsProj/CsProjGenerator.cs),
[standard toolchain](https://github.com/dotnet/BenchmarkDotNet/blob/v0.15.8/src/BenchmarkDotNet/Toolchains/CsProj/CsProjCoreToolchain.cs)
and [CLI job construction](https://github.com/dotnet/BenchmarkDotNet/blob/v0.15.8/src/BenchmarkDotNet/ConsoleArguments/ConfigParser.cs).

### Validation

The existing rotational CCD and empty-world workloads run through the repaired
launcher in both configurations:

| Check | Release | ReleaseLean |
| --- | ---: | ---: |
| Actual local empty-world frame | 1 complete row | 1 complete row |
| Actual local rotational CCD | 6 complete rows | 6 complete rows |
| Diagnostics tests | 53 pass | 53 pass |
| Diagnostics covered lines | 323/323 | 323/323 |
| Diagnostics covered branches | 158/158 | 158/158 |
| Diagnostics fully covered reported methods | 124/124 | 124/124 |

Every Dry row has one actual workload iteration. Both rotational methods run
at candidate counts 1, 8 and 32. All their children
exit zero with complete results. The eight LSF runtime assemblies in each real
generated host match their respective launcher files by SHA-256. Diagnostics
also builds for both `netstandard2.1` and `net10.0` in optimized Lean with zero
warnings/errors. No complete physics test rerun is needed for these tooling and
diagnostics build-setting changes; no physics or math implementation changed.

One-off diagnostic probes also exercised dependency-typed arguments/returns,
ordinary and explicit .NET 10 jobs, in-process jobs, custom CLI/package paths,
duplicate project names, failure status and stock validation. Their 48 successful
signature rows used the expected dependency hashes. Package-backed probes used
released public dependencies; they do not establish package-mode release
validation of current physics with unreleased lower-stack internals.
The verification script, synthetic probe projects and checked inertia fixture
are not retained as a test suite. Existing source tests and ordinary benchmark
runs remain the validation path. Supplemental investigation files are under
`artifacts/grv-infra-111-125` and `artifacts/benchmark-host-checks`.

## GRV-Issue-125

### Original Failure And Exact Fixture

On 2026-10-13 one frozen pre-cleanup benchmark process terminated with native
access violation `0xC0000005`, after six warmups and two actual iterations.
The run used .NET 8.0.29 x64 RyuJIT with concurrent workstation GC and default
tiering/PGO. Its partial timings are invalid.

The frozen payloads are identified by full SHA-256, rather than assuming a
committed revision represents that intermediate implementation:

- Gravitas: `D0F088DD4C177DE89C20D0427DFB5DD209F546F5F5AE192778C529F194114183`.
- FixedMathSharp: `55E6509C4F4313EE77F1802010B5C7EDEC7B7D5EB4FFBBB368F400E5DD91B23E`.

Construct `BenchmarkPhysicsScene.CreateSubdividedClosedCubeTopology(1)`, then
multiply every vertex componentwise by
`(Fixed64.FromFraction(6,5), Fixed64.FromFraction(11,4), Fixed64.FromFraction(7,6))`.
Construct `PhysicsMesh` with zero origin and identity rotation, without another
transform or scale. Use `MeshInertiaPolicy.SurfaceApproximation`, its default
center of mass, and alternate `Fixed64.FromFraction(7,3) + Fixed64.One` followed
by `Fixed64.FromFraction(7,3)` according to an incrementing tick's parity.
Call `CalculateInertiaTensor` on every invocation. The original setup reports
M11 raw `10637165705` at the lower mass. Its checksum is
`M11 ^ (M22<<1) ^ (M33<<2) ^ (M12<<3) ^ (M13<<4) ^ (M23<<5)`, using raw entries.

The recovered native fault is `MeshRuntimeInertia.TryMaterialize` entry +`0x1c0`,
instruction `mov qword ptr [rcx],rdx`. RCX is `0305279c88aa23d5`, exactly the Q60
coefficient fraction. RDX is `000000027a064489`, the expected M11 raw scalar.
R10 is `04fad8637755dc2b`; R10+RCX equals Q60 half `0800000000000000`.
The coefficient byrefs and tensor output have valid-looking locations. This
identifies the bad native store, not the instruction sequence that produced its
address: the small dump lacks the preceding JIT pages and function entry.
A JIT defect remains a hypothesis, not a confirmed cause.

### Existing Mitigation And Evidence

The retained implementation already changed the certificate's output to
`TryMultiplyRaw(..., out long)` and constructs final `Fixed64` entries in the
tensor materializer. This is an earlier controlled mitigation of the observed
store shape. It is not a new fix here and does not reconstruct the failed native
body. The existing normalized IL comparison limited differences to the two
certificate bodies and tensor materializer; coefficient layout remained 32 bytes.

The earlier frozen replay completed two launches without reproducing the crash.
Earlier raw-output stress verified 400 million calls across fractional and
translated/nonuniform changing-mass, changing-reference and recreated-geometry
cases. A separate worker completed 618 blocking compacting gen2 collections
without collector errors. Captured successful Tier1 bodies formed valid scalar
output addresses. These checks do not prove that a collection landed at a
particular instruction or that the historical crash was repaired.

### Bounded Final Reproduction

A deliberately caught-null .NET 8 control validates first-chance AV collection:
the debugger captures the native AV and writes a dump before the managed catch
prints its marker. Debugger command strings must escape backslashes in the dump
path and verify the file exists. The control and final debugger sessions exit
zero. The fault handler used for reproduction stops at the first AV after
dumping exception records, registers and stack.

Three valid frozen-process replays each complete all six actual iterations,
with zero reported managed allocations and no first-chance AV. They retain the
original .NET 8.0.29 runtime and frozen DLL hashes. They use the frozen benchmark
assembly in-process so all calculation stays in the debugger's target; the
original generated failing child was not preserved and cannot be reconstructed.
Therefore this is a bounded replay of the original inputs/payload, not a claim
that its original JIT state was recreated.

```powershell
dotnet artifacts/grv-122-123/perf/rca/failed-host/Gravitas.Benchmarks.dll --filter '*MeshStandaloneWarm.AlternatingMass*Fractional*True*' --inProcess --warmupCount 6 --iterationCount 6 --iterationTime 500 --exporters json
```

For collection, launch that command with `cdb -cf <commands>` and commands
`sxi ibp`, `sxe -c ".echo INFRA_FIRST_CHANCE_AV; .dump /ma <escaped-absolute-path>; .exr -1; r; k; q" av`,
`g`, `q`. No child following is needed for this in-process workload. Earlier
descendant-following attempts completed six workload iterations but returned
debugger status `0x8007001e` with incomplete exports; those attempts are excluded
from accepted benchmark-completion evidence. The caught-null control continues
with `gn` instead of stopping at the fault, to verify the managed catch too.

An investigation-only fixture used the same topology, dimensions, policy and
mass sequence on current source, checking every matrix entry against its
corresponding setup output on every invocation. Those setup-derived values
are not an independent mathematical oracle. Three out-of-process .NET 10
launches each complete six actual iterations, all children and the launcher
exit zero, and MemoryDiagnoser reports zero allocation. No arithmetic, GC,
tiering or PGO policy changes were needed. This is diagnostic evidence, not a
throughput comparison across the two runtimes or payloads.

The temporary fixture is not retained in the benchmark project. Its complete
inputs and checksum are described above for a future bounded reproduction.

### Separate 2026-10-19 CLR Failure

The later #142 observation is one `Fatal error. Internal CLR error.
(0x80131506)` before case output, without a dump. Its .NET 8 host loads two
noncollectible assembly contexts and uses nonconcurrent GC. There is no evidence
connecting it to the inertia scalar store. Earlier isolated, balanced and
identical-payload reruns passed. The final parent-only debugger replay uses the
preserved .NET 8 host, `baseline-stack` and
`candidate-before-domain-simplification` payloads. All eight cases complete 24
rounds each, with matching physical witnesses/checksums and zero warmed allocation
on both sides. All loaded module hashes match those payloads; CDB exits zero,
with no AV marker or fatal CLR error. Passing cannot attribute the original cause.

```powershell
dotnet artifacts/grv-issue-142/host/bin/Release/net8.0/Host.dll artifacts/grv-issue-142/baseline-stack artifacts/grv-issue-142/candidate-before-domain-simplification artifacts/grv-infra-111-125/clr142-replay.json
```

The final replay uses the first-chance debugger command pattern above. It checks
`CylinderBaseFace`, `MixedCylinderBaseFace`, `CylinderObliqueRim`,
`MixedCylinderObliqueRim`, `CylinderRimGap`, `MixedCylinderRimGap`,
`FullStepSingle` and `FullStepJoined`. The old host verifies witnesses before
timing, checks zero warmed allocation and equal checksums each round, and writes
all loaded payload hashes. This run is diagnostic, not performance acceptance.

### Disposition And Reopening

Close #125 as an investigation with no reproduced actionable defect, retaining
the earlier mitigation and all historical uncertainty. No new runtime guard,
retry, arithmetic replacement or JIT/GC switch is justified by the available
evidence. A finite successful replay cannot prove universal fault absence.

Reopen on a native crash or inconsistent tensor in current runtime/CI, or on a
reliable reproducer for either historical failure. Capture a full first-chance
dump before the process disappears, complete native method/disassembly, fault
and exception context, source and loaded module hashes, SDK/runtime/architecture,
GC/tiering environment and the ordered fixture. Stop at the first recurrence;
exclude partial benchmark results. Do not replace missing native evidence with
speculative source changes or infer that the two failures share a cause.

Historical supplemental evidence is under `artifacts/grv-122-123/perf/rca` and
`artifacts/grv-issue-142`. Final collection/replay logs and reports are under
`artifacts/grv-infra-111-125`. This record preserves the reproduction and
disposition independently of those ignored artifacts.
