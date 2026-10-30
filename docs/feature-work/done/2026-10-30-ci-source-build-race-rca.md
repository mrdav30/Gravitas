# Native CI Source Build Race

**Date:** 2026-10-30. **Status:** Workflow fix prepared; hosted validation awaits commit/push.

## Observed Failures

The destination [run 41113094482](https://github.com/mrdav30/Gravitas/actions/runs/41113094482)
builds develop revision `d07393534931192159440b2e8279918ac2dede8d`.
Seven native .NET lanes and wiki validation pass. Windows ARM64 Release fails
in compilation with `CS2012`: the compiler reports that another process holds
`Chronicler/src/Chronicler.Diagnostics/obj/windows/Release/net10.0/net10.0/Chronicler.Diagnostics.dll`.
Tests never start in that lane. Upload then fails because there is no capture;
the final comparison fails because `windows-arm64-Release` is missing.

Those upload/comparison errors follow the failed build. Node 20 deprecation
warnings are separate: the artifact actions are forced onto Node 24, and the
upload warning appears after the C# compilation error. Other lanes complete
with the same artifact actions.

The link's display text names the older [run 40944271898](https://github.com/mrdav30/Gravitas/actions/runs/40944271898).
That run fails earlier: FixedMathSharp checkout requests
`03f6cbf46b08b5e3fd6ae10311f09a48a6091ca2` and GitHub rejects it with
`upload-pack: not our ref`. The current workflow already selects the fetchable
`d0d316053f6207b5624182ee1336ecaf142f25c6`, whose checkout succeeds in the newer
run. No further source-pin change is required by this RCA.

## Cause And Discriminating Build

Explicit sibling `ProjectReference` edges carry `SemVer` and `AssemblySemVer`
through `AdditionalProperties`. SDK-inferred transitive references are bare
project references; they can inherit the parent's identity instead. This reaches
the same diagnostics project with multiple property sets but shared intermediate
and output paths. Parallel compiler tasks can then contend for the same file.
Even when the build succeeds, the last writer can publish the wrong identity.
This matches Microsoft's [build-race guidance](https://learn.microsoft.com/en-us/visualstudio/msbuild/fix-intermittent-build-failures).

The comparison uses tracked-source archives of the exact run revisions:

| Repository | Revision |
| --- | --- |
| Gravitas | `d07393534931192159440b2e8279918ac2dede8d` |
| FixedMathSharp | `d0d316053f6207b5624182ee1336ecaf142f25c6` |
| SwiftCollections | `22c0de88c7449e1cd15d54007876bceade643b44` |
| GridForge | `89c69f6ebf0bb00cadbc63685b59946e60872b30` |
| Chronicler | `c547e5c633ec71bb3cd0e072720a75fb9b8580ad` |

Both clean source copies use Windows x64, SDK 10.0.302, the run's GitVersion
values, and eight-way parallel solution builds. The hosted failed lane uses
Windows ARM64 and SDK 10.0.403; the local baseline does not reproduce its file
lock. Diagnostic and binary logs do reproduce the competing output writers:

| Diagnostics build evidence | Baseline | Global inference suppression |
| --- | --- | --- |
| Concrete target-framework identities | `1.0.0` and `7.1.0` | `1.0.0` |
| Distinct C# compiler tasks per target framework/output path | 2 | 1 |
| Emitted assembly version, both target frameworks | `7.1.0.0` | `1.0.0.0` |
| Parallel build | Pass, unsafe duplicate writers observed | Pass |

The fixed graph still has metadata-only evaluations, including the parent
GitVersion identity. These do not produce another compiler invocation. The
discriminating evidence is the actual compiler tasks and emitted assemblies,
not merely the number of evaluated projects.

The archived builds inherit ancestor analyzer configuration from the containing
checkout and report documentation/configuration warnings. Both have zero build
errors; these warnings are unrelated to the native file contention. Their
timings are not performance measurements.

## Workflow Changes

- Pass `DisableTransitiveProjectReferences=${{ env.UseLocalLsfStack }}` as a
  global command-line property on restore, build and test. Source builds retain
  the explicitly versioned edges throughout pinned siblings. Package builds
  retain transitive references. Parallel compilation remains enabled.
- Upload replay observations only after tests actually ran, on either success
  or failure. Keep `if-no-files-found: error`: a test/discovery failure that
  produces no capture still fails capture validation.
- Keep the final gate's `if: always()`. Fail its first step explicitly when
  native validation did not succeed, then skip download/comparison. Do not let
  a skipped final job mask failed native lanes for required status checks.
  Successful native validation still requires all eight captures and unchanged
  shared replay expectations.
- Update artifact upload to v7 and download to v8. Their official
  [upload manifest](https://github.com/actions/upload-artifact/blob/v7/action.yml)
  and [download manifest](https://github.com/actions/download-artifact/blob/v8/action.yml)
  select Node 24. Existing artifact naming, archive defaults and per-artifact
  download directories remain compatible with the comparator.

## Validation And Limits

The corrected pinned source graph passes the existing Windows x64 suites:

| Configuration | Tests | Replay fixtures captured |
| --- | ---: | ---: |
| Release | 8,241 pass | 8 |
| ReleaseLean | 8,170 pass | 8 |

The existing comparator passes an explicit two-lane comparison of those real
captures, both against the shared expectations and directly between profiles.
It reports the six unexecuted Windows ARM64/Linux lanes. The first comparison
attempt correctly rejected the live working checkout's CRLF fixture bytes;
the executed archives use LF. Comparing against the exact archived manifests
used by the tests passes without changing fixtures or weakening fingerprints.

Normal working-checkout parallel builds also pass in both configurations with
zero warnings/errors. Workflow static validation passes with actionlint 1.7.12. The updated
`lsf-development` skill and its validation reference prohibit maintained tests
or harnesses around test/benchmark projects; the skill passes its supplied
validator. This work adds no test suite or physics/math implementation change.

Supplemental diagnostic/binary logs, source archives, contexts, test output and
captures are under `artifacts/workflow-rca-41113094482`. The cause, exact pins,
comparison and commands remain documented here if those files are deleted.
The decisive build property is `-p:DisableTransitiveProjectReferences=true`
alongside `-p:UseLocalLsfStack=true`; apply both to restore/build/test.

No workflow run has been retried or published by this change. A new hosted run
after commit/push must validate Windows/Linux x64/ARM64 and the full eight-lane
comparison, including artifact action execution. Local Windows x64 evidence is
not completion of that native matrix.
