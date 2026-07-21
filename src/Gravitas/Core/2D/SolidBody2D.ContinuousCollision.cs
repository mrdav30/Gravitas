//=======================================================================
// SolidBody2D.ContinuousCollision.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System.Runtime.CompilerServices;

namespace Gravitas;

public sealed partial class SolidBody2D
{
    private LSCollider? _continuousCollisionHandoffIgnoredCollider3D;
    private LSCollider2D? _continuousCollisionHandoffIgnoredCollider2D;

    internal Vector2d ContinuousCollisionFrameStart
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => ResolveLegacyContinuousCollisionFrameStart();
    }

    internal Vector2d ContinuousCollisionFrameDisplacement
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => ResolveLegacyContinuousCollisionFrameDisplacement();
    }

    internal Fixed64 ContinuousCollisionFrameRotation
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionTrajectory[0].StartRotation;
    }

    internal Vector2d ContinuousCollisionFrameEnd
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionTrajectory[_continuousCollisionTrajectory.Count - 1].EndPosition;
    }

    internal Fixed64 ContinuousCollisionFrameTargetRotation
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get
        {
            ContinuousCollisionMotionSegment2D segment =
                _continuousCollisionTrajectory[_continuousCollisionTrajectory.Count - 1];
            return CanonicalizeRotation(segment.StartRotation + segment.AngularDelta);
        }
    }

    internal int ContinuousCollisionTrajectoryCount =>
        _continuousCollisionTrajectory.Count;

    internal ContinuousCollisionMotionSegment2D GetContinuousCollisionTrajectorySegment(
        int index) =>
        _continuousCollisionTrajectory[index];

    internal bool HasContinuousCollisionMotion
    {
        get
        {
            for (int i = 0; i < _continuousCollisionTrajectory.Count; i++)
            {
                ContinuousCollisionMotionSegment2D segment =
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
        Vector2d displacement = ContinuousCollisionFrameDisplacement;

        Fixed64 angularDistance = Fixed64.Zero;
        for (int i = 0; i < _continuousCollisionTrajectory.Count; i++)
            angularDistance += _continuousCollisionTrajectory[i].AngularDistance;

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

    internal Vector2d SampleContinuousCollisionPosition(Fixed64 frameFraction) =>
        _continuousCollisionTrajectory.Count == 0
            ? _position
            : ResolveContinuousCollisionSegment(frameFraction).SamplePosition(frameFraction);

    internal Fixed64 SampleContinuousCollisionRotation(Fixed64 frameFraction) =>
        _continuousCollisionTrajectory.Count == 0
            ? _rotation
            : CanonicalizeRotation(
                ResolveContinuousCollisionSegment(frameFraction).SampleRotation(frameFraction));

    internal Vector2d SampleContinuousCollisionLinearVelocity(Fixed64 frameFraction)
    {
        if (_continuousCollisionTrajectory.Count == 0)
            return ProjectLinearMotion(_linearVelocity);

        ContinuousCollisionMotionSegment2D segment =
            ResolveContinuousCollisionSegment(frameFraction);
        Fixed64 duration = (segment.EndFraction - segment.StartFraction) * Context.DeltaTime;
        return ProjectLinearMotion(segment.Displacement / duration);
    }

    internal Fixed64 SampleContinuousCollisionAngularVelocity(Fixed64 frameFraction) =>
        _continuousCollisionTrajectory.Count == 0
            ? _angularVelocity
            : ResolveContinuousCollisionSegment(frameFraction).AngularVelocity;

    private Vector2d ResolveLegacyContinuousCollisionFrameStart()
    {
        ContinuousCollisionMotionSegment2D segment =
            _continuousCollisionTrajectory[_continuousCollisionTrajectory.Count - 1];
        Vector2d frameDisplacement = ResolveLegacyContinuousCollisionFrameDisplacement();
        return segment.StartPosition - frameDisplacement * segment.StartFraction;
    }

    private Vector2d ResolveLegacyContinuousCollisionFrameDisplacement()
    {
        ContinuousCollisionMotionSegment2D segment =
            _continuousCollisionTrajectory[_continuousCollisionTrajectory.Count - 1];
        Fixed64 remainingFraction = Fixed64.One - segment.StartFraction;
        return segment.Displacement / remainingFraction;
    }

    internal void EnsureContinuousCollisionFramePrepared(int token)
    {
        if (_continuousCollisionFrameToken == token)
            return;

        Vector2d startPosition = _position;
        Fixed64 startRotation = _rotation;
        Vector2d displacement;
        Vector2d endPosition;
        Fixed64 angularDelta;
        Fixed64 angularVelocity;
        if (IsKinematic)
        {
            Vector2d targetPosition = ProjectLinearEndpoint(
                startPosition,
                Agent.Transform.WorldPositionXZ);
            endPosition = targetPosition;
            displacement = ShouldUseContinuousCollision(out _)
                ? ContinuousCollisionSweepRange.ValidateEndpoint(
                    startPosition,
                    targetPosition,
                    out _)
                : targetPosition - startPosition;

            Fixed64 targetRotation = CanonicalizeRotation(Agent.Transform.WorldRotationXZRadians);
            angularDelta = CanonicalizeRotation(targetRotation - startRotation);
            angularVelocity = angularDelta / Context.DeltaTime;
        }
        else
        {
            PrepareMovableContinuousCollisionMotion();
            displacement = ProjectLinearMotion(_linearVelocity) * Context.DeltaTime;
            endPosition = startPosition + displacement;
            angularVelocity = _angularVelocity;
            angularDelta = angularVelocity * Context.DeltaTime;
        }

        _continuousCollisionFrameToken = token;
        SetContinuousCollisionFrameSegment(new ContinuousCollisionMotionSegment2D(
            Fixed64.Zero,
            Fixed64.One,
            startPosition,
            endPosition,
            displacement,
            startRotation,
            angularDelta,
            angularVelocity));
    }

    private void PrepareMovableContinuousCollisionMotion()
    {
        if (_isSleeping)
            return;

        Fixed64 deltaTime = Context.DeltaTime;
        if (CanTranslate)
        {
            _linearAccelerationStore = RemoveIntoGroundComponent(
                _deltaAcceleration + Gravity * _gravityScale);
            _deltaAcceleration = Vector2d.Zero;
            _linearVelocity += ProjectLinearMotion(_linearAccelerationStore * deltaTime);
            _linearVelocity = RemoveIntoGroundComponent(_linearVelocity);
            _linearAccelerationStore = Vector2d.Zero;
            RefreshLinearSpeed();
        }
        else
        {
            _linearAccelerationStore = Vector2d.Zero;
            _deltaAcceleration = Vector2d.Zero;
        }

        if (CanRotate)
        {
            _angularAccelerationStore = _deltaAngularAcceleration;
            _deltaAngularAcceleration = Fixed64.Zero;
            _angularVelocity += _angularAccelerationStore * deltaTime;
            RefreshAngularSpeed();
        }
        else
        {
            _angularAccelerationStore = Fixed64.Zero;
            _deltaAngularAcceleration = Fixed64.Zero;
        }
    }

    private void SetContinuousCollisionFrameSegment(ContinuousCollisionMotionSegment2D segment)
    {
        _continuousCollisionTrajectory.FastClear();
        _continuousCollisionTrajectory.Add(segment);
    }

    private void AppendContinuousCollisionFrameSegment(
        Vector2d positionAtElapsedTime,
        Vector2d velocity,
        Fixed64 elapsedTime)
    {
        Fixed64 startFraction = FixedMath.Clamp01(elapsedTime / Context.DeltaTime);
        Fixed64 rotationAtElapsedTime = SampleContinuousCollisionRotation(startFraction);
        AppendContinuousCollisionFrameSegment(
            positionAtElapsedTime,
            rotationAtElapsedTime,
            velocity,
            _angularVelocity,
            elapsedTime);
    }

    private void AppendContinuousCollisionFrameSegment(
        Vector2d positionAtElapsedTime,
        Fixed64 rotationAtElapsedTime,
        Vector2d velocity,
        Fixed64 angularVelocity,
        Fixed64 elapsedTime)
    {
        Fixed64 deltaTime = Context.DeltaTime;
        Fixed64 startFraction = FixedMath.Clamp01(elapsedTime / deltaTime);
        Fixed64 remainingTime = FixedMath.Max(Fixed64.Zero, deltaTime - elapsedTime);
        Vector2d displacement = ProjectLinearMotion(velocity) * remainingTime;
        _continuousCollisionFrameToken = Context.LateSimulateToken;
        RemoveSupersededContinuousCollisionFrameSegments(
            startFraction,
            positionAtElapsedTime);
        if (startFraction >= Fixed64.One && _continuousCollisionTrajectory.Count > 0)
            return;

        _continuousCollisionTrajectory.Add(new ContinuousCollisionMotionSegment2D(
            startFraction,
            Fixed64.One,
            positionAtElapsedTime,
            positionAtElapsedTime + displacement,
            displacement,
            rotationAtElapsedTime,
            angularVelocity * remainingTime,
            angularVelocity));
    }

    internal bool CanAppendContinuousCollisionFrameSegment(Fixed64 elapsedTime)
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
            ContinuousCollisionMotionSegment2D segment = _continuousCollisionTrajectory[i];
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

    internal DynamicCcdPlanarBounds ResolveContinuousCollisionTrajectoryBounds(Fixed64 radius)
    {
        ContinuousCollisionMotionSegment2D first = _continuousCollisionTrajectory[0];
        Fixed64 minX = FixedMath.Min(first.StartPosition.X, first.EndPosition.X);
        Fixed64 minZ = FixedMath.Min(first.StartPosition.Y, first.EndPosition.Y);
        Fixed64 maxX = FixedMath.Max(first.StartPosition.X, first.EndPosition.X);
        Fixed64 maxZ = FixedMath.Max(first.StartPosition.Y, first.EndPosition.Y);
        for (int i = 1; i < _continuousCollisionTrajectory.Count; i++)
        {
            ContinuousCollisionMotionSegment2D segment = _continuousCollisionTrajectory[i];
            minX = FixedMath.Min(minX, FixedMath.Min(segment.StartPosition.X, segment.EndPosition.X));
            minZ = FixedMath.Min(minZ, FixedMath.Min(segment.StartPosition.Y, segment.EndPosition.Y));
            maxX = FixedMath.Max(maxX, FixedMath.Max(segment.StartPosition.X, segment.EndPosition.X));
            maxZ = FixedMath.Max(maxZ, FixedMath.Max(segment.StartPosition.Y, segment.EndPosition.Y));
        }

        return new DynamicCcdPlanarBounds(
            minX - radius,
            minZ - radius,
            maxX + radius,
            maxZ + radius);
    }

    private void RemoveSupersededContinuousCollisionFrameSegments(
        Fixed64 startFraction,
        Vector2d positionAtElapsedTime)
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
        ContinuousCollisionMotionSegment2D predecessor =
            _continuousCollisionTrajectory[predecessorIndex];
        if (predecessor.EndFraction <= startFraction)
            return;

        Fixed64 localFraction = (startFraction - predecessor.StartFraction)
            / (predecessor.EndFraction - predecessor.StartFraction);
        _continuousCollisionTrajectory[predecessorIndex] =
            new ContinuousCollisionMotionSegment2D(
                predecessor.StartFraction,
                startFraction,
                predecessor.StartPosition,
                positionAtElapsedTime,
                positionAtElapsedTime - predecessor.StartPosition,
                predecessor.StartRotation,
                predecessor.AngularDelta * localFraction,
                predecessor.AngularVelocity);
    }

    private ContinuousCollisionMotionSegment2D ResolveContinuousCollisionSegment(
        Fixed64 frameFraction) =>
        _continuousCollisionTrajectory[
            GetContinuousCollisionTrajectorySegmentIndex(frameFraction)];

    internal int GetContinuousCollisionTrajectorySegmentIndex(
        Fixed64 frameFraction)
    {
        for (int i = _continuousCollisionTrajectory.Count - 1; i > 0; i--)
        {
            ContinuousCollisionMotionSegment2D segment =
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

    private void InvalidateContinuousCollisionFrame()
    {
        _continuousCollisionFrameToken = int.MinValue;
        _continuousCollisionTrajectory.FastClear();
    }
}
