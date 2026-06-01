using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// Deterministic minimum-penetration axis selected by mixed narrow-phase SAT checks.
/// </summary>
internal readonly struct MixedAxisPenetration
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public MixedAxisPenetration(Vector3d axis, Fixed64 depth)
    {
        Axis = axis;
        Depth = depth;
        HasValue = true;
    }

    public Vector3d Axis { get; }

    public Fixed64 Depth { get; }

    public bool HasValue { get; }
}
