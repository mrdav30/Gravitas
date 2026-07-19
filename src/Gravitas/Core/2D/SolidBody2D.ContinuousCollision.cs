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
        get => _continuousCollisionTrajectory.Count == 0
            ? _position
            : ResolveLegacyContinuousCollisionFrameStart();
    }

    internal Vector2d ContinuousCollisionFrameDisplacement
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionTrajectory.Count == 0
            ? Vector2d.Zero
            : ResolveLegacyContinuousCollisionFrameDisplacement();
    }

    internal Fixed64 ContinuousCollisionFrameRotation
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionTrajectory.Count == 0
            ? _rotation
            : _continuousCollisionTrajectory[0].StartRotation;
    }

    internal Vector2d ContinuousCollisionFrameEnd
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionTrajectory.Count == 0
            ? _position
            : _continuousCollisionTrajectory[_continuousCollisionTrajectory.Count - 1].EndPosition;
    }

    internal Fixed64 ContinuousCollisionFrameTargetRotation
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get
        {
            if (_continuousCollisionTrajectory.Count == 0)
                return _rotation;

            ContinuousCollisionMotionSegment2D segment =
                _continuousCollisionTrajectory[_continuousCollisionTrajectory.Count - 1];
            return CanonicalizeRotation(segment.StartRotation + segment.AngularDelta);
        }
    }

    internal Fixed64 ContinuousCollisionFrameAngularVelocity
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionTrajectory.Count == 0
            ? _angularVelocity
            : _continuousCollisionTrajectory[0].AngularVelocity;
    }

    internal int ContinuousCollisionTrajectoryCount =>
        _continuousCollisionTrajectory.Count;

    internal Vector2d SampleContinuousCollisionPosition(Fixed64 frameFraction) =>
        ResolveContinuousCollisionSegment(frameFraction).SamplePosition(frameFraction);

    internal Fixed64 SampleContinuousCollisionRotation(Fixed64 frameFraction) =>
        CanonicalizeRotation(
            ResolveContinuousCollisionSegment(frameFraction).SampleRotation(frameFraction));

    internal Vector2d SampleContinuousCollisionLinearVelocity(Fixed64 frameFraction)
    {
        ContinuousCollisionMotionSegment2D segment =
            ResolveContinuousCollisionSegment(frameFraction);
        Fixed64 duration = (segment.EndFraction - segment.StartFraction) * Context.DeltaTime;
        return duration > Fixed64.Zero
            ? ProjectLinearMotion(segment.Displacement / duration)
            : Vector2d.Zero;
    }

    internal Fixed64 SampleContinuousCollisionAngularVelocity(Fixed64 frameFraction) =>
        ResolveContinuousCollisionSegment(frameFraction).AngularVelocity;

    internal bool TrySampleContinuousCollisionDisplacement(
        Fixed64 startFraction,
        Fixed64 endFraction,
        out Vector2d startPosition,
        out Vector2d displacement)
    {
        startPosition = SampleContinuousCollisionPosition(startFraction);
        Vector2d endPosition = SampleContinuousCollisionPosition(endFraction);
        return Vector2d.TrySubtract(endPosition, startPosition, out displacement);
    }

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
        return remainingFraction > Fixed64.Zero
            ? segment.Displacement / remainingFraction
            : Vector2d.Zero;
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
            angularVelocity = Context.DeltaTime > Fixed64.Zero
                ? angularDelta / Context.DeltaTime
                : Fixed64.Zero;
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
        if (!CanTranslate)
        {
            _linearAccelerationStore = Vector2d.Zero;
            _deltaAcceleration = Vector2d.Zero;
            _angularAccelerationStore = Fixed64.Zero;
            _deltaAngularAcceleration = Fixed64.Zero;
            return;
        }

        if (_isSleeping)
            return;

        Fixed64 deltaTime = Context.DeltaTime;
        _linearAccelerationStore = RemoveIntoGroundComponent(
            _deltaAcceleration + Gravity * _gravityScale);
        _deltaAcceleration = Vector2d.Zero;
        _linearVelocity += ProjectLinearMotion(_linearAccelerationStore * deltaTime);
        _linearVelocity = RemoveIntoGroundComponent(_linearVelocity);
        _linearAccelerationStore = Vector2d.Zero;
        RefreshLinearSpeed();

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
        if (_continuousCollisionTrajectory.Count == 0)
            _continuousCollisionTrajectory.Add(segment);
        else
            _continuousCollisionTrajectory[0] = segment;

        while (_continuousCollisionTrajectory.Count > 1)
            _continuousCollisionTrajectory.RemoveAt(
                _continuousCollisionTrajectory.Count - 1);
    }

    private void AppendContinuousCollisionFrameSegment(
        Vector2d positionAtElapsedTime,
        Vector2d velocity,
        Fixed64 elapsedTime)
    {
        Fixed64 deltaTime = Context.DeltaTime;
        Fixed64 startFraction = FixedMath.Clamp01(elapsedTime / deltaTime);
        Fixed64 remainingTime = FixedMath.Max(Fixed64.Zero, deltaTime - elapsedTime);
        Vector2d displacement = ProjectLinearMotion(velocity) * remainingTime;
        Fixed64 startRotation = _continuousCollisionTrajectory.Count == 0
            ? _rotation
            : SampleContinuousCollisionRotation(startFraction);
        Fixed64 angularVelocity = _angularVelocity;
        _continuousCollisionFrameToken = Context.LateSimulateToken;
        RemoveSupersededContinuousCollisionFrameSegments(startFraction);
        _continuousCollisionTrajectory.Add(new ContinuousCollisionMotionSegment2D(
            startFraction,
            Fixed64.One,
            positionAtElapsedTime,
            positionAtElapsedTime + displacement,
            displacement,
            startRotation,
            angularVelocity * remainingTime,
            angularVelocity));
    }

    private void RemoveSupersededContinuousCollisionFrameSegments(Fixed64 startFraction)
    {
        while (_continuousCollisionTrajectory.Count > 0
            && _continuousCollisionTrajectory[_continuousCollisionTrajectory.Count - 1]
                .StartFraction >= startFraction)
        {
            _continuousCollisionTrajectory.RemoveAt(
                _continuousCollisionTrajectory.Count - 1);
        }
    }

    private ContinuousCollisionMotionSegment2D ResolveContinuousCollisionSegment(
        Fixed64 frameFraction)
    {
        for (int i = _continuousCollisionTrajectory.Count - 1; i >= 0; i--)
        {
            ContinuousCollisionMotionSegment2D segment =
                _continuousCollisionTrajectory[i];
            if (frameFraction >= segment.StartFraction)
                return segment;
        }

        return _continuousCollisionTrajectory[0];
    }

    private void InvalidateContinuousCollisionFrame()
    {
        _continuousCollisionFrameToken = int.MinValue;
        _continuousCollisionTrajectory.FastClear();
    }
}
