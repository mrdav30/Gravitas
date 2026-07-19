//=======================================================================
// ContinuousCollisionMotionSegment2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <summary>
/// One immutable, normalized-time piece of a 2D body's prepared CCD trajectory.
/// </summary>
internal readonly struct ContinuousCollisionMotionSegment2D
{
    public ContinuousCollisionMotionSegment2D(
        Fixed64 startFraction,
        Fixed64 endFraction,
        Vector2d startPosition,
        Vector2d endPosition,
        Vector2d displacement,
        Fixed64 startRotation,
        Fixed64 angularDelta,
        Fixed64 angularVelocity)
    {
        StartFraction = startFraction;
        EndFraction = endFraction;
        StartPosition = startPosition;
        EndPosition = endPosition;
        Displacement = displacement;
        StartRotation = startRotation;
        AngularDelta = angularDelta;
        AngularVelocity = angularVelocity;
    }

    public Fixed64 StartFraction { get; }
    public Fixed64 EndFraction { get; }
    public Vector2d StartPosition { get; }
    public Vector2d EndPosition { get; }
    public Vector2d Displacement { get; }
    public Fixed64 StartRotation { get; }
    public Fixed64 AngularDelta { get; }
    public Fixed64 AngularVelocity { get; }
    public Fixed64 AngularDistance => AngularDelta.Abs();

    public Vector2d SamplePosition(Fixed64 frameFraction) =>
        Vector2d.Lerp(StartPosition, EndPosition, ResolveLocalFraction(frameFraction));

    public Fixed64 SampleRotation(Fixed64 frameFraction) =>
        StartRotation + AngularDelta * ResolveLocalFraction(frameFraction);

    public bool TryResolveMotionBound(
        Fixed64 lowerFraction,
        Fixed64 upperFraction,
        Fixed64 pivotRadius,
        out Fixed64 motionBound)
    {
        Fixed64 segmentSpan = EndFraction - StartFraction;
        if (segmentSpan <= Fixed64.Zero)
        {
            motionBound = Fixed64.Zero;
            return true;
        }

        Fixed64 localSpan = (upperFraction - lowerFraction) / segmentSpan;
        if (!Vector2d.TrySubtract(EndPosition, StartPosition, out Vector2d exactDisplacement))
        {
            motionBound = default;
            return false;
        }

        return ContinuousCollisionMath.TryResolveRotationalIntervalMotionBound(
            exactDisplacement,
            AngularDistance,
            pivotRadius,
            localSpan,
            out motionBound);
    }

    private Fixed64 ResolveLocalFraction(Fixed64 frameFraction)
    {
        Fixed64 span = EndFraction - StartFraction;
        return span <= Fixed64.Zero
            ? Fixed64.Zero
            : FixedMath.Clamp01((frameFraction - StartFraction) / span);
    }
}
