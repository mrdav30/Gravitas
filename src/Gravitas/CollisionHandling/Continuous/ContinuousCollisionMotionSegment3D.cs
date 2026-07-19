//=======================================================================
// ContinuousCollisionMotionSegment3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

internal enum ContinuousCollisionRotationPath3D : byte
{
    IntegratedAngularVelocity,
    KinematicSlerp
}

/// <summary>
/// One immutable, normalized-time piece of a 3D body's prepared CCD trajectory.
/// </summary>
internal readonly struct ContinuousCollisionMotionSegment3D
{
    public ContinuousCollisionMotionSegment3D(
        Fixed64 startFraction,
        Fixed64 endFraction,
        Vector3d startPosition,
        Vector3d endPosition,
        Vector3d displacement,
        FixedQuaternion startRotation,
        Vector3d angularVelocity,
        FixedQuaternion targetRotation,
        Fixed64 angularDistance,
        ContinuousCollisionRotationPath3D rotationPath)
    {
        StartFraction = startFraction;
        EndFraction = endFraction;
        StartPosition = startPosition;
        EndPosition = endPosition;
        Displacement = displacement;
        StartRotation = startRotation;
        AngularVelocity = angularVelocity;
        TargetRotation = targetRotation;
        AngularDistance = angularDistance;
        RotationPath = rotationPath;
    }

    public Fixed64 StartFraction { get; }
    public Fixed64 EndFraction { get; }
    public Vector3d StartPosition { get; }
    public Vector3d EndPosition { get; }
    public Vector3d Displacement { get; }
    public FixedQuaternion StartRotation { get; }
    public Vector3d AngularVelocity { get; }
    public FixedQuaternion TargetRotation { get; }
    public Fixed64 AngularDistance { get; }
    public ContinuousCollisionRotationPath3D RotationPath { get; }

    public Vector3d SamplePosition(Fixed64 frameFraction)
    {
        Fixed64 localFraction = ResolveLocalFraction(frameFraction);
        return Vector3d.Lerp(StartPosition, EndPosition, localFraction);
    }

    public FixedQuaternion SampleRotation(Fixed64 frameFraction, Fixed64 frameDeltaTime)
    {
        Fixed64 localFraction = ResolveLocalFraction(frameFraction);
        if (RotationPath == ContinuousCollisionRotationPath3D.KinematicSlerp)
            return FixedQuaternion.Slerp(StartRotation, TargetRotation, localFraction).Normalized;

        Fixed64 elapsedTime = frameDeltaTime * (frameFraction - StartFraction);
        FixedQuaternion angularVelocityQuaternion = new(
            AngularVelocity.X,
            AngularVelocity.Y,
            AngularVelocity.Z,
            Fixed64.Zero);
        FixedQuaternion spin = angularVelocityQuaternion * StartRotation * Fixed64.Half * elapsedTime;
        return (StartRotation + spin).Normalized;
    }

    public bool TryResolveMotionBound(
        Fixed64 lowerFraction,
        Fixed64 upperFraction,
        Fixed64 pivotRadius,
        out Fixed64 motionBound)
    {
        Fixed64 segmentSpan = EndFraction - StartFraction;
        Fixed64 localSpan = (upperFraction - lowerFraction) / segmentSpan;
        if (!Vector3d.TrySubtract(EndPosition, StartPosition, out Vector3d exactDisplacement))
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
        return FixedMath.Clamp01((frameFraction - StartFraction) / span);
    }
}
