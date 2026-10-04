using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using System;

namespace Gravitas.Benchmarks;

/// <summary>Measures host yaw work that body-owned planar poses discard.</summary>
[MemoryDiagnoser]
public class PlanarScaleCaptureBenchmarks
{
    private FixedTransform _transform;

    [Params(0, 1)]
    public int HostYawRadians { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _transform = new FixedTransform(Vector3d.Zero,
            FixedQuaternion.FromAxisAngle(Vector3d.Up, (Fixed64)HostYawRadians),
            new Vector3d(2, 1, 3));
        Vector2d withYaw = ColliderScalePolicy.CapturePlanar(_transform, out Fixed4x4 matrix, out _);
        Vector2d scaleOnly = ColliderScalePolicy.CapturePlanar(_transform,
            out Fixed4x4 scaleOnlyMatrix, out Fixed64 omittedYaw, captureRotation: false);
        if (withYaw != scaleOnly || matrix != scaleOnlyMatrix || omittedYaw != Fixed64.Zero)
            throw new InvalidOperationException("Omitting host yaw changed planar scale admission.");
    }

    [Benchmark(Baseline = true)]
    public long CaptureWithHostYaw() => ColliderScalePolicy.CapturePlanar(
        _transform, out _, out _).X.m_rawValue;

    [Benchmark]
    public long CaptureScaleOnly() => ColliderScalePolicy.CapturePlanar(_transform,
        out _, out _, captureRotation: false).X.m_rawValue;
}
