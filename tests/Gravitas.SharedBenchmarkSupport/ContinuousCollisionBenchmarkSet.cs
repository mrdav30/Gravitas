#nullable enable

namespace Gravitas.Benchmarking;

internal enum ContinuousCollisionBenchmarkSet : byte
{
    SparsePure3D,
    DensePure3D,
    SparsePure2D,
    DensePure2D,
    ShapeExactPure3D,
    ShapeExactPure2D,
    SparseMixed3D,
    SparseMixed2D,
    DenseMixed3D,
    DenseMixed2D
}
