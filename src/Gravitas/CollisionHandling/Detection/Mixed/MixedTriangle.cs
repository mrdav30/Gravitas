using FixedMathSharp;
using SwiftCollections.Query;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// World-space triangle data used by mixed mesh-vs-slab checks.
/// </summary>
internal readonly struct MixedTriangle
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public MixedTriangle(Vector3d a, Vector3d b, Vector3d c, Vector3d normal, FixedBoundVolume bounds)
    {
        A = a;
        B = b;
        C = c;
        Normalized = normal;
        Bounds = bounds;
        Center = (a + b + c) / (Fixed64)3;
    }

    public Vector3d A { get; }

    public Vector3d B { get; }

    public Vector3d C { get; }

    public Vector3d Normalized { get; }

    public FixedBoundVolume Bounds { get; }

    public Vector3d Center { get; }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d GetEdge(int index) =>
        index switch
        {
            0 => B - A,
            1 => C - B,
            _ => A - C
        };
}
