# Experimental Exact Triangle-Pair Throughput Pass

**Created:** 2026-08-02  
**Status:** Complete; no second-pass production change retained  
**Signal:** Exact triangle-pair contacts regress dense concave-mesh throughput

## Goal

Attempt one final bounded optimization pass without weakening the exact
full-domain triangle relation or creating concave-specific production
machinery. Retain a change only when unchanged Gravitas rows improve by at
least `5%` repeatably, controls remain stable, warmed allocation stays `0 B`,
and every modified repository retains 100% reachable line, branch, and method
coverage.

If the experiments miss that gate, revert them and classify the remaining
signal as experimental capacity guidance. The deleted scalar relation is not a
correctness-compatible target.

## Evidence And Design

The retained signed one-limb specialization improved the affected rows by
`13.7-15.4%`, but the post-change profile still centers on
`WideArithmetic.MultiplySigned576`, `MultiplySigned320`, and
`GetMagnitudeBitLength`. Quaternion-derived rational-basis coefficients and
denominators are exact signed two-limb values in ordinary normalized inputs,
yet they still pay the generic `Signed576` by `Signed192` dispatch.

The experiments are ordered by breadth and expected value:

1. Add an auto-detected signed two-limb fast path inside the existing
   FixedMathSharp wide-multiply owner. Preserve the generic fallback and exact
   nine-word result; add no geometry-specific answer path or public API.
2. Only if the first experiment misses the gate, measure hoisting the two exact
   rigid-frame bases from each candidate triangle relation to the owning
   Gravitas mesh-pair pass. Keep the cache invocation-local and allocation-free.
3. Stop. Do not add a narrowed prefilter, alternate scalar SAT, persistent mesh
   cache, coplanar-patch subsystem, or automatic convex promotion in this pass.

Large-struct `in` churn is excluded because the runtime already passes large
structs indirectly and no evidence shows a material copy cost. Coplanar patch
preprocessing and a separate exact intersection classifier may be worthwhile
research, but both require their own topology, memory, and correctness design.

## Work Plan

Expected files for the first experiment:

- `../FixedMathSharp/src/FixedMathSharp/Numerics/Wide/WideArithmetic.Signed576.cs`
- `../FixedMathSharp/tests/FixedMathSharp.Tests/Numerics/Wide/WideFiniteAxisArithmetic.Tests.cs`
- this plan and `benchmark-signal-hardening-backlog.md`

- [x] Preserve the current direct FixedMathSharp and unchanged Gravitas
      benchmark artifacts as the comparison baseline.
- [x] Add focused signed two-limb multiply regressions before production code,
      including zero, signs, carry, signed-boundary, and full-width left values.
- [x] Implement the smallest exact fast path in the existing arithmetic owner.
- [x] Run focused tests and direct arithmetic/triangle benchmarks.
- [x] Run the unchanged Gravitas `*MeshMesh*` rows for any experiment that
      survives the direct gate. The arithmetic experiment stopped at its direct
      row; the frame-preparation experiment ran every mesh/mesh row.
- [x] Retain the change only if the affected rows clear the `5%` repeatability
      gate without a meaningful control regression.
- [x] If rejected, revert it and test only the invocation-local frame hoist
      under the same gate.
- [x] Prove both source experiments were reverted exactly; the established
      Release, ReleaseLean, package, warmed-allocation, and 100% coverage gates
      therefore remain the production authority.
- [x] Obtain independent correctness and performance review, update the
      benchmark backlog, and move this plan to `done` with the evidence.

Focused commands:

```powershell
dotnet test ../FixedMathSharp/tests/FixedMathSharp.Tests/FixedMathSharp.Tests.csproj `
    --configuration Release `
    --filter "FullyQualifiedName~WideArithmetic"

dotnet ../FixedMathSharp/tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll `
    oriented-box-anchor --filter "*TrianglePair*" -j Short -i

dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll `
    collision-detection --filter "*MeshMesh*" -j Short -i
```

## Stop Condition

One retained shared optimization is sufficient. If neither bounded experiment
clears the gate, the remaining dense concave cost becomes an experimental
capacity signal while release work proceeds on competitive primitive, convex,
and authored compound collider paths.

## Outcome

Neither bounded experiment cleared the gate, so both were reverted.

The signed two-limb multiplication experiment moved the direct
`TrianglePairPrimary` mean from `54.33 us` to `54.009 us`, about `0.6%`. That
return did not justify another branch in the shared arithmetic owner.

Invocation-local rigid-frame preparation produced the following unchanged
64-pair Short in-process results:

| Row | Prior confirmation | Frame preparation | Change |
| --- | ---: | ---: | ---: |
| Concave mesh/mesh | `59.761 ms` | `60.220 ms` | `+0.77%` |
| Dense concave mesh/mesh | `343.474 ms` | `344.444 ms` | `+0.28%` |
| Contact-heavy concave mesh/mesh | `480.773 ms` | `482.460 ms` | `+0.35%` |
| Closed dense mesh/mesh | `2.155 s` | `2.177 s` | `+1.04%` |

The ordinary convex mesh/mesh control measured `4.798 ms` and continued to use
its separate convex relation. The focused 13 FixedMathSharp triangle-contact
tests and 18 Gravitas mesh-contact/allocation tests passed while the experiment
was present. The source trees were then restored exactly, so no production or
test code from either rejected experiment remains.

The evidence now favors reducing complete exact SAT evaluations: every
BVH-admitted triangle pair can require up to 17 exact axes, while the tested
two-limb dispatch and frame preparation were immaterial. A material improvement
likely requires a separately designed topology or algorithm change, such as exact
coplanar patch ownership or a proven exact miss classifier, rather than another
local arithmetic or setup tweak. Dense dynamic concave mesh/mesh collision is
therefore classified as experimental capacity-sensitive behavior. Primitive,
convex, decomposed compound, and partitioned static-concave authoring remain the
competitive release paths.

Rejected experiment artifacts are preserved under
`../FixedMathSharp/artifacts/benchmarks/2026-08-02-triangle-pair-signed128-experiment`
and
`artifacts/benchmarks/2026-08-02-triangle-pair-frame-hoist-experiment`.
