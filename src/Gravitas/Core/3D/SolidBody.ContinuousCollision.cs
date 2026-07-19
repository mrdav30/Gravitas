//=======================================================================
// SolidBody.ContinuousCollision.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System.Runtime.CompilerServices;

namespace Gravitas;

public partial class SolidBody
{
    private LSCollider? _continuousCollisionHandoffIgnoredCollider3D;
    private LSCollider2D? _continuousCollisionHandoffIgnoredCollider2D;

    internal Vector3d ContinuousCollisionFrameStart
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionTrajectory.Count == 0
            ? Position3d
            : ResolveLegacyContinuousCollisionFrameStart();
    }

    internal Vector3d ContinuousCollisionFrameDisplacement
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionTrajectory.Count == 0
            ? Vector3d.Zero
            : ResolveLegacyContinuousCollisionFrameDisplacement();
    }

    internal FixedQuaternion ContinuousCollisionFrameRotation
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionTrajectory.Count == 0
            ? Rotation
            : _continuousCollisionTrajectory[0].StartRotation;
    }

    internal Vector3d ContinuousCollisionFrameEnd
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionTrajectory.Count == 0
            ? Position3d
            : _continuousCollisionTrajectory[_continuousCollisionTrajectory.Count - 1].EndPosition;
    }

    internal FixedQuaternion ContinuousCollisionFrameTargetRotation
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionTrajectory.Count == 0
            ? Rotation
            : _continuousCollisionTrajectory[_continuousCollisionTrajectory.Count - 1].TargetRotation;
    }

    internal Vector3d ContinuousCollisionFrameAngularVelocity
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionTrajectory.Count == 0
            ? _angularVelocity
            : _continuousCollisionTrajectory[0].AngularVelocity;
    }

    internal int ContinuousCollisionTrajectoryCount =>
        _continuousCollisionTrajectory.Count;

    internal Vector3d SampleContinuousCollisionPosition(Fixed64 frameFraction) =>
        ResolveContinuousCollisionSegment(frameFraction).SamplePosition(frameFraction);

    internal FixedQuaternion SampleContinuousCollisionRotation(Fixed64 frameFraction) =>
        ResolveContinuousCollisionSegment(frameFraction).SampleRotation(
            frameFraction,
            Context.DeltaTime);

    internal Vector3d SampleContinuousCollisionLinearVelocity(Fixed64 frameFraction)
    {
        ContinuousCollisionMotionSegment3D segment =
            ResolveContinuousCollisionSegment(frameFraction);
        Fixed64 duration = (segment.EndFraction - segment.StartFraction) * Context.DeltaTime;
        return duration > Fixed64.Zero
            ? ProjectLinearMotion(segment.Displacement / duration)
            : Vector3d.Zero;
    }

    internal Vector3d SampleContinuousCollisionAngularVelocity(Fixed64 frameFraction) =>
        ResolveContinuousCollisionSegment(frameFraction).AngularVelocity;

    internal bool TrySampleContinuousCollisionDisplacement(
        Fixed64 startFraction,
        Fixed64 endFraction,
        out Vector3d startPosition,
        out Vector3d displacement)
    {
        startPosition = SampleContinuousCollisionPosition(startFraction);
        Vector3d endPosition = SampleContinuousCollisionPosition(endFraction);
        return Vector3d.TrySubtract(endPosition, startPosition, out displacement);
    }

    private Vector3d ResolveLegacyContinuousCollisionFrameStart()
    {
        ContinuousCollisionMotionSegment3D segment =
            _continuousCollisionTrajectory[_continuousCollisionTrajectory.Count - 1];
        Vector3d frameDisplacement = ResolveLegacyContinuousCollisionFrameDisplacement();
        return segment.StartPosition - frameDisplacement * segment.StartFraction;
    }

    private Vector3d ResolveLegacyContinuousCollisionFrameDisplacement()
    {
        ContinuousCollisionMotionSegment3D segment =
            _continuousCollisionTrajectory[_continuousCollisionTrajectory.Count - 1];
        Fixed64 remainingFraction = Fixed64.One - segment.StartFraction;
        return remainingFraction > Fixed64.Zero
            ? segment.Displacement / remainingFraction
            : Vector3d.Zero;
    }

    internal void EnsureContinuousCollisionFramePrepared(int token)
    {
        if (_continuousCollisionFrameToken == token)
            return;

        Vector3d startPosition = Position3d;
        FixedQuaternion startRotation = Rotation;
        Vector3d displacement;
        Vector3d endPosition;
        FixedQuaternion targetRotation;
        Vector3d angularVelocity;
        Fixed64 angularDistance;
        ContinuousCollisionRotationPath3D rotationPath;
        if (IsKinematic)
        {
            _continuousCollisionAngularVelocityStepStart = Vector3d.Zero;
            Vector3d targetPosition = ProjectLinearEndpoint(
                startPosition,
                _positionTransform.WorldPosition);
            endPosition = targetPosition;
            if (ShouldUseContinuousCollision(out _))
            {
                displacement = ContinuousCollisionSweepRange.ValidateEndpoint(
                    startPosition,
                    targetPosition,
                    out _);
            }
            else
            {
                displacement = targetPosition - startPosition;
            }

            targetRotation = _rotationTransform.WorldRotation.Normalized;
            angularVelocity = Context.DeltaTime > Fixed64.Zero
                ? FixedQuaternion.ToAngularVelocity(
                    targetRotation,
                    startRotation,
                    Context.DeltaTime)
                : Vector3d.Zero;
            angularDistance = ResolveKinematicAngularDistanceRadians(
                startRotation,
                targetRotation);
            rotationPath = ContinuousCollisionRotationPath3D.KinematicSlerp;
        }
        else
        {
            _continuousCollisionAngularVelocityStepStart = _angularVelocity;
            if (!IsPositionFullyFrozen && !_isSleeping)
            {
                ApplyLinearForces();
                UpdateLinearVelocity();
                if (CanRotate)
                {
                    ApplyAngularTorques();
                    UpdateAngularVelocity();
                }
            }

            displacement = ProjectLinearMotion(_linearVelocity) * Context.DeltaTime;
            endPosition = startPosition + displacement;
            angularVelocity = _angularVelocity;
            targetRotation = IntegrateAngularRotation(
                startRotation,
                angularVelocity,
                Context.DeltaTime);
            angularDistance = angularVelocity.Magnitude * Context.DeltaTime;
            rotationPath = ContinuousCollisionRotationPath3D.IntegratedAngularVelocity;
        }

        _continuousCollisionFrameToken = token;
        SetSingleContinuousCollisionSegment(new ContinuousCollisionMotionSegment3D(
            Fixed64.Zero,
            Fixed64.One,
            startPosition,
            endPosition,
            displacement,
            startRotation,
            angularVelocity,
            targetRotation,
            angularDistance,
            rotationPath));
    }

    private void InvalidateContinuousCollisionTrajectory()
    {
        _continuousCollisionFrameToken = int.MinValue;
        _continuousCollisionAngularVelocityStepStart = Vector3d.Zero;
        _continuousCollisionTrajectory.FastClear();
    }

    private void SetSingleContinuousCollisionSegment(
        ContinuousCollisionMotionSegment3D segment)
    {
        if (_continuousCollisionTrajectory.Count == 0)
            _continuousCollisionTrajectory.Add(segment);
        else
            _continuousCollisionTrajectory[0] = segment;

        while (_continuousCollisionTrajectory.Count > 1)
            _continuousCollisionTrajectory.RemoveAt(
                _continuousCollisionTrajectory.Count - 1);
    }

    private void AppendContinuousCollisionSegment(
        Vector3d positionAtElapsedTime,
        Vector3d velocity,
        Fixed64 elapsedTime)
    {
        Fixed64 deltaTime = Context.DeltaTime;
        Fixed64 startFraction = FixedMath.Clamp01(elapsedTime / deltaTime);
        Fixed64 remainingTime = FixedMath.Max(Fixed64.Zero, deltaTime - elapsedTime);
        Vector3d displacement = ProjectLinearMotion(velocity) * remainingTime;
        FixedQuaternion startRotation = _continuousCollisionTrajectory.Count == 0
            ? Rotation
            : SampleContinuousCollisionRotation(startFraction);
        Vector3d angularVelocity = _angularVelocity;
        FixedQuaternion targetRotation = IntegrateAngularRotation(
            startRotation,
            angularVelocity,
            remainingTime);
        _continuousCollisionFrameToken = Context.LateSimulateToken;
        RemoveSupersededContinuousCollisionSegments(startFraction);
        _continuousCollisionTrajectory.Add(new ContinuousCollisionMotionSegment3D(
            startFraction,
            Fixed64.One,
            positionAtElapsedTime,
            positionAtElapsedTime + displacement,
            displacement,
            startRotation,
            angularVelocity,
            targetRotation,
            angularVelocity.Magnitude * remainingTime,
            ContinuousCollisionRotationPath3D.IntegratedAngularVelocity));
    }

    private void RemoveSupersededContinuousCollisionSegments(Fixed64 startFraction)
    {
        while (_continuousCollisionTrajectory.Count > 0
            && _continuousCollisionTrajectory[_continuousCollisionTrajectory.Count - 1]
                .StartFraction >= startFraction)
        {
            _continuousCollisionTrajectory.RemoveAt(
                _continuousCollisionTrajectory.Count - 1);
        }
    }

    private ContinuousCollisionMotionSegment3D ResolveContinuousCollisionSegment(
        Fixed64 frameFraction)
    {
        for (int i = _continuousCollisionTrajectory.Count - 1; i >= 0; i--)
        {
            ContinuousCollisionMotionSegment3D segment =
                _continuousCollisionTrajectory[i];
            if (frameFraction >= segment.StartFraction)
                return segment;
        }

        return _continuousCollisionTrajectory[0];
    }
}
