//=======================================================================
// SolidBody.ContinuousCollision.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using SwiftCollections.Query;
using System.Runtime.CompilerServices;

namespace Gravitas;

public partial class SolidBody
{
    private LSCollider? _continuousCollisionHandoffIgnoredCollider3D;
    private LSCollider2D? _continuousCollisionHandoffIgnoredCollider2D;

    internal Vector3d ContinuousCollisionFrameStart
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => ResolveLegacyContinuousCollisionFrameStart();
    }

    internal Vector3d ContinuousCollisionFrameDisplacement
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => ResolveLegacyContinuousCollisionFrameDisplacement();
    }

    internal FixedQuaternion ContinuousCollisionFrameRotation
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionTrajectory[0].StartRotation;
    }

    internal Vector3d ContinuousCollisionFrameEnd
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionTrajectory[_continuousCollisionTrajectory.Count - 1].EndPosition;
    }

    internal FixedQuaternion ContinuousCollisionFrameTargetRotation
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionTrajectory[_continuousCollisionTrajectory.Count - 1].TargetRotation;
    }

    internal int ContinuousCollisionTrajectoryCount =>
        _continuousCollisionTrajectory.Count;

    internal ContinuousCollisionMotionSegment3D GetContinuousCollisionTrajectorySegment(
        int index) =>
        _continuousCollisionTrajectory[index];

    internal bool HasContinuousCollisionMotion
    {
        get
        {
            for (int i = 0; i < _continuousCollisionTrajectory.Count; i++)
            {
                ContinuousCollisionMotionSegment3D segment =
                    _continuousCollisionTrajectory[i];
                if (segment.Displacement.MagnitudeSquared > Fixed64.Epsilon
                    || segment.AngularDistance > Fixed64.Epsilon)
                {
                    return true;
                }
            }

            return false;
        }
    }

    internal bool ShouldOwnContinuousCollisionMovingPair(bool otherHasRotationalMotion)
    {
        if (!IsKinematic || !ShouldUseContinuousCollision(out ContinuousCollisionMode mode))
            return false;

        EnsureContinuousCollisionFramePrepared(Context.LateSimulateToken);
        if (!HasContinuousCollisionMotion)
            return false;
        if (mode == ContinuousCollisionMode.Continuous)
            return true;

        Fixed64 proxyRadius = ResolveContinuousCollisionProxyRadius();
        Vector3d displacement = ContinuousCollisionFrameDisplacement;

        Fixed64 angularDistance = ResolveKinematicAngularDistanceRadians(
            ContinuousCollisionFrameRotation,
            ContinuousCollisionFrameTargetRotation);
        bool requiresTargetSampling = angularDistance <= Fixed64.Epsilon
            && otherHasRotationalMotion;
        return requiresTargetSampling
            | angularDistance * proxyRadius > proxyRadius
            | !ContinuousCollisionMath.IsWithinProxyRadius(
                displacement,
                displacement.MagnitudeSquared,
                proxyRadius);
    }

    internal bool HasContinuousCollisionRotationalMotion
    {
        get
        {
            for (int i = 0; i < _continuousCollisionTrajectory.Count; i++)
            {
                if (_continuousCollisionTrajectory[i].AngularDistance > Fixed64.Epsilon)
                    return true;
            }

            return false;
        }
    }

    internal Vector3d SampleContinuousCollisionPosition(Fixed64 frameFraction) =>
        ResolveContinuousCollisionSegment(frameFraction).SamplePosition(frameFraction);

    internal FixedQuaternion SampleContinuousCollisionRotation(Fixed64 frameFraction) =>
        ResolveContinuousCollisionSegment(frameFraction).SampleRotation(
            frameFraction,
            Context.DeltaTime);

    internal Vector3d SampleContinuousCollisionLinearVelocity(Fixed64 frameFraction)
    {
        if (_continuousCollisionTrajectory.Count == 0)
            return ProjectLinearMotion(_linearVelocity);

        ContinuousCollisionMotionSegment3D segment =
            ResolveContinuousCollisionSegment(frameFraction);
        Fixed64 duration = (segment.EndFraction - segment.StartFraction) * Context.DeltaTime;
        return ProjectLinearMotion(segment.Displacement / duration);
    }

    internal Vector3d SampleContinuousCollisionAngularVelocity(Fixed64 frameFraction) =>
        _continuousCollisionTrajectory.Count == 0
            ? _angularVelocity
            : ResolveContinuousCollisionSegment(frameFraction).AngularVelocity;

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
        return segment.Displacement / remainingFraction;
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
            angularVelocity = FixedQuaternion.ToAngularVelocity(
                targetRotation,
                startRotation,
                Context.DeltaTime);
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
        _continuousCollisionTrajectory.FastClear();
        _continuousCollisionTrajectory.Add(segment);
    }

    private void AppendContinuousCollisionSegment(
        Vector3d positionAtElapsedTime,
        Vector3d velocity,
        Fixed64 elapsedTime)
    {
        Fixed64 startFraction = FixedMath.Clamp01(elapsedTime / Context.DeltaTime);
        FixedQuaternion startRotation = SampleContinuousCollisionRotation(startFraction);
        AppendContinuousCollisionSegment(
            positionAtElapsedTime,
            startRotation,
            velocity,
            _angularVelocity,
            elapsedTime);
    }

    private void AppendContinuousCollisionSegment(
        Vector3d positionAtElapsedTime,
        FixedQuaternion rotationAtElapsedTime,
        Vector3d linearVelocity,
        Vector3d angularVelocity,
        Fixed64 elapsedTime)
    {
        Fixed64 deltaTime = Context.DeltaTime;
        Fixed64 startFraction = FixedMath.Clamp01(elapsedTime / deltaTime);
        Fixed64 remainingTime = FixedMath.Max(Fixed64.Zero, deltaTime - elapsedTime);
        Vector3d displacement = ProjectLinearMotion(linearVelocity) * remainingTime;
        FixedQuaternion targetRotation = IntegrateAngularRotation(
            rotationAtElapsedTime,
            angularVelocity,
            remainingTime);
        _continuousCollisionFrameToken = Context.LateSimulateToken;
        RemoveSupersededContinuousCollisionSegments(
            startFraction,
            positionAtElapsedTime,
            rotationAtElapsedTime);
        if (startFraction >= Fixed64.One && _continuousCollisionTrajectory.Count > 0)
            return;

        _continuousCollisionTrajectory.Add(new ContinuousCollisionMotionSegment3D(
            startFraction,
            Fixed64.One,
            positionAtElapsedTime,
            positionAtElapsedTime + displacement,
            displacement,
            rotationAtElapsedTime,
            angularVelocity,
            targetRotation,
            angularVelocity.Magnitude * remainingTime,
            ContinuousCollisionRotationPath3D.IntegratedAngularVelocity));
    }

    internal bool CanAppendContinuousCollisionSegment(Fixed64 elapsedTime)
    {
        Fixed64 startFraction = FixedMath.Clamp01(elapsedTime / Context.DeltaTime);
        int retainedCount = _continuousCollisionTrajectory.Count;
        while (retainedCount > 0
            && _continuousCollisionTrajectory[retainedCount - 1].StartFraction >= startFraction)
        {
            retainedCount--;
        }

        return retainedCount <= Context.Settings.ContinuousCollisionMaxToiIterations;
    }

    internal bool TryResolveContinuousCollisionMotionBound(
        Fixed64 lowerFraction,
        Fixed64 upperFraction,
        Fixed64 pivotRadius,
        out Fixed64 motionBound)
    {
        motionBound = Fixed64.Zero;
        for (int i = 0; i < _continuousCollisionTrajectory.Count; i++)
        {
            ContinuousCollisionMotionSegment3D segment = _continuousCollisionTrajectory[i];
            Fixed64 lower = FixedMath.Max(lowerFraction, segment.StartFraction);
            Fixed64 upper = FixedMath.Min(upperFraction, segment.EndFraction);
            if (upper <= lower)
                continue;

            if (!segment.TryResolveMotionBound(lower, upper, pivotRadius, out Fixed64 segmentBound)
                || !Fixed64.TryAdd(motionBound, segmentBound, out motionBound))
            {
                motionBound = default;
                return false;
            }
        }

        return true;
    }

    internal FixedBoundVolume ResolveContinuousCollisionTrajectoryBounds(Fixed64 radius)
    {
        Vector3d min = Position3d;
        Vector3d max = Position3d;
        for (int i = 0; i < _continuousCollisionTrajectory.Count; i++)
        {
            ContinuousCollisionMotionSegment3D segment = _continuousCollisionTrajectory[i];
            min = Vector3d.Min(min, Vector3d.Min(segment.StartPosition, segment.EndPosition));
            max = Vector3d.Max(max, Vector3d.Max(segment.StartPosition, segment.EndPosition));
        }

        Vector3d extents = Vector3d.One * radius;
        return new FixedBoundVolume(min - extents, max + extents);
    }

    private void RemoveSupersededContinuousCollisionSegments(
        Fixed64 startFraction,
        Vector3d positionAtElapsedTime,
        FixedQuaternion rotationAtElapsedTime)
    {
        while (_continuousCollisionTrajectory.Count > 0
            && _continuousCollisionTrajectory[_continuousCollisionTrajectory.Count - 1]
                .StartFraction >= startFraction)
        {
            _continuousCollisionTrajectory.RemoveAt(
                _continuousCollisionTrajectory.Count - 1);
        }

        if (_continuousCollisionTrajectory.Count == 0)
            return;

        int predecessorIndex = _continuousCollisionTrajectory.Count - 1;
        ContinuousCollisionMotionSegment3D predecessor =
            _continuousCollisionTrajectory[predecessorIndex];
        if (predecessor.EndFraction <= startFraction)
            return;

        Fixed64 localFraction = (startFraction - predecessor.StartFraction)
            / (predecessor.EndFraction - predecessor.StartFraction);
        _continuousCollisionTrajectory[predecessorIndex] =
            new ContinuousCollisionMotionSegment3D(
                predecessor.StartFraction,
                startFraction,
                predecessor.StartPosition,
                positionAtElapsedTime,
                positionAtElapsedTime - predecessor.StartPosition,
                predecessor.StartRotation,
                predecessor.AngularVelocity,
                rotationAtElapsedTime,
                predecessor.AngularDistance * localFraction,
                predecessor.RotationPath);
    }

    private ContinuousCollisionMotionSegment3D ResolveContinuousCollisionSegment(
        Fixed64 frameFraction) =>
        _continuousCollisionTrajectory[
            GetContinuousCollisionTrajectorySegmentIndex(frameFraction)];

    internal int GetContinuousCollisionTrajectorySegmentIndex(
        Fixed64 frameFraction)
    {
        for (int i = _continuousCollisionTrajectory.Count - 1; i > 0; i--)
        {
            ContinuousCollisionMotionSegment3D segment =
                _continuousCollisionTrajectory[i];
            if (frameFraction >= segment.StartFraction)
                return i;
        }

        return 0;
    }

    internal void GetContinuousCollisionTrajectorySegmentRange(
        Fixed64 startFraction,
        out int startIndex,
        out int endExclusive)
    {
        startIndex = startFraction <= Fixed64.Zero
            ? 0
            : GetContinuousCollisionTrajectorySegmentIndex(startFraction);
        endExclusive = _continuousCollisionTrajectory.Count;
    }
}
