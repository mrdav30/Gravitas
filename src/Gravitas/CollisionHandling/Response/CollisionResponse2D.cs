using FixedMathSharp;

namespace Gravitas;

/// <summary>
/// Constants for the alpha pure 2D contact response path.
/// </summary>
public static class CollisionResponse2D
{
    public static readonly Fixed64 PenetrationSlop = (Fixed64)0.01f;

    public static readonly Fixed64 PenetrationCorrectionPercent = Fixed64.One;

    public static readonly Fixed64 RestitutionVelocityThreshold = (Fixed64)0.25f;
}
