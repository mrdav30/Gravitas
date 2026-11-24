# Linux ARM64 VSTest Artifact Failure

**Date:** 2026-11-23. **Status:** Workflow mitigation prepared; native hosted
validation awaits commit/push. **Issue:** GRV-Issue-158.

## Observed Failure

[Run 45251904171](https://github.com/mrdav30/Gravitas/actions/runs/45251904171)
validates develop revision `b99188f34170a9dcb224c97c7c580c30bf3d1128` with
`UseLocalLsfStack=true`. Both Linux ARM64 builds succeed, and every test passes:

| Failed job | Configuration | Passed tests |
| --- | --- | ---: |
| [137963463159](https://github.com/mrdav30/Gravitas/actions/runs/45251904171/job/137963463159) | Release | 8,572 |
| [137963462541](https://github.com/mrdav30/Gravitas/actions/runs/45251904171/job/137963462541) | ReleaseLean | 8,495 |

After `Test Run Successful`, VSTest reports that it cannot open
`/run/user/1001/<session>/<pid_guid>` with `errno: 22`, then reports that the
session directory allows access by other users. The test command exits 1,
skipping the coverage gate. Replay uploads succeed. The final comparison job
fails its prerequisite check because native validation did not succeed; it
never compares replay data. The other six native validation lanes succeed.

## Root Cause

This matches the independently reported
[VSTest ARM64 regression #16657](https://github.com/microsoft/vstest/issues/16657).
The artifact postprocessor uses `0x30000` for `O_DIRECTORY | O_NOFOLLOW` on
Linux. Those values are correct on x64, but mean `O_DIRECT | O_LARGEFILE` on
ARM64. Opening a directory with `O_DIRECT` returns `EINVAL` (22). The failed
open prevents `fchmod`, leaving the new directory at `0755`; the later access
error follows that first failure.

The failed lanes use SDK `10.0.404`. Its bundled VSTest version is
`18.7.1-hotfix-26529-119`, listed in Microsoft's
[VSTest advisory](https://github.com/microsoft/vstest/issues/16654).
[Upstream fix #16696](https://github.com/microsoft/vstest/pull/16696) selects the
flags by architecture. A different artifact directory would still encounter
the incorrect flags. Updating `Microsoft.NET.Test.Sdk` alone does not replace
the SDK's bundled postprocessor.

## Focused Mitigation

Set `VSTEST_DISABLE_ARTIFACTS_POSTPROCESSING=1` only in the Linux ARM64 test
step. Set it to `0` in the other lanes, preserving their normal path.
SDK [10.0.404's test command](https://github.com/dotnet/sdk/blob/v10.0.404/src/Cli/dotnet/Commands/Test/VSTest/TestCommand.cs)
honors this switch before both session-artifact collection setup and the final
postprocessing invocation.

This workflow runs one test project targeting only `net10.0`, so no
cross-project or cross-framework attachment aggregation is required. Coverlet
writes its Cobertura and OpenCover reports directly to the results directory.
Keep the collector, strict coverage gate, `TreatNoTestsAsError`, replay capture,
artifact uploads, and final native-validation prerequisite unchanged. The
mitigation does not disable directory security verification or downgrade the
SDK. It changes no runtime source, tests, dependencies, or CI source pins.

Remove the switch after a serviced SDK includes #16696, then verify both
native Linux ARM64 configurations with coverage and replay comparison enabled.
If the failure recurs, first inspect the selected SDK and bundled VSTest
revision, the first artifact error, direct coverage files, and replay uploads.

## Local Validation

Validation uses clean tracked-source archives, avoiding unrelated sibling
working-tree edits. Source revisions match the failed run:

| Repository | Revision |
| --- | --- |
| Gravitas | `b99188f34170a9dcb224c97c7c580c30bf3d1128` |
| FixedMathSharp | `8fdf889322f1bb82501c2afec63d6bb2f304ace2` |
| SwiftCollections | `41a2f770aed17f5af388ba3f4b3f769769ada7c6` |
| GridForge | `4b2bc64e503a1bd3c8cdf7e81b4e565cfa9651e8` |
| Chronicler | `7b90d5923d4bb6e0e42c033ec697b9678d6b8ece` |

The local host is Ubuntu under WSL, Linux x64, SDK `10.0.404`, with
`UseLocalLsfStack=true` in the environment and on all restore/build/test commands.
Builds also use `DisableTransitiveProjectReferences=true`, `-m:1`, and
`BuildInParallel=false`. Both profiles exercise the workaround with
`VSTEST_DISABLE_ARTIFACTS_POSTPROCESSING=1`.

For each of `Release` and `ReleaseLean`, run the existing solution restore and
build, then the same test project, collector, runsettings, and no-tests gate as
CI. Use separate fresh results directories per profile, and run
`.assets/scripts/assert-full-coverage.ps1` on each. Compare the real captures
with `.github/scripts/compare_replay_captures.py --lanes linux-x64-Release
linux-x64-ReleaseLean` against the archived LF fixture manifests. No maintained
test or harness is added for workflow infrastructure.

Both local solution builds pass with zero warnings/errors. Both test commands
exit 0 and attach their direct Cobertura and OpenCover reports:

| Configuration | Passed tests | Covered/reported lines | Covered/reported branches | Covered executable methods |
| --- | ---: | ---: | ---: | ---: |
| Release | 8,572 | 54,812/54,812 | 22,847/22,847 | 5,787 |
| ReleaseLean | 8,495 | 54,810/54,810 | 22,847/22,847 | 5,786 |

The unchanged strict coverage script passes for each profile. The existing
comparator also passes the two new local replay lanes across all eight fixtures.
An intentionally unmatched test filter with the workaround enabled and
`TreatNoTestsAsError=true` still exits 1. This disposable negative control adds
no test fixture or maintained infrastructure.

Separately, download all eight replay artifacts from the original failed run,
verify their SHA-256 digests against GitHub's artifact metadata, extract each
into its lane directory, and invoke the existing comparator with the archived
fixture manifests and no `--lanes` override. This full native matrix passes:
eight lanes, eight fixtures, shared expectations and direct cross-lane captures
agree. These are the original hosted executions, including both failed ARM64
jobs; they are not post-mitigation hosted executions.

Workflow static validation passes with actionlint `1.7.12`; independent ponytail
review found no actionable workflow or RCA defects. Native ARM64 execution of
the changed workflow remains a hosted check, not a claim from local x64
validation.
