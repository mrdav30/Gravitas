//=======================================================================
// SolidBody.Visualization.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas;

public partial class SolidBody
{
    internal void OnVisualize()
    {
        if (!HasSolverMobility || !SettingVisuals)
            return;

        if (Context.ResetAccumulationThisVisualize)
        {
            if (CanSetVisualPosition)
                SetVisualPosition(Position3d);
            if (CanSetVisualRotation)
                StoreVisualRotation(_rotation);
        }

        if (CanSetVisualPosition)
        {
            Vector3d expectedPosition = Vector3d.SpeedLerp(
                _lastVisualPosition,
                _visualPosition,
                Fixed64.One,
                Context.ExpectedAccumulation);
            SetPositionTransformWorldPosition(expectedPosition);
        }

        if (!CanSetVisualRotation)
            return;

        Fixed64 targetSpeed = ResolveVisualRotationStep();
        FixedQuaternion expectedRotation = _rotationInterpoleSpeed > Fixed64.Zero
            ? FixedQuaternion.Slerp(
                _rotationTransform.WorldRotation,
                _visualRotation,
                targetSpeed)
            : FixedQuaternion.Slerp(
                _lastVisualRotation,
                _visualRotation,
                targetSpeed);
        SetRotationTransformWorldRotation(expectedRotation);
    }

    private Fixed64 ResolveVisualRotationStep()
    {
        if (_rotationInterpoleSpeed <= Fixed64.Zero)
            return Context.ExpectedAccumulation;

        return FixedMath.Clamp01(
            Context.DeltaTime
            * _rotationInterpoleSpeed
            * _rotationSpeed);
    }

    private void SetPositionTransformWorldPosition(Vector3d position)
    {
        SwiftThrowHelper.ThrowIfTrue(
            !_positionTransform.TrySetWorldPosition(position),
            nameof(FixedTransform),
            "Position transform cannot represent the requested world position.");
    }

    private void SetRotationTransformWorldRotation(FixedQuaternion rotation)
    {
        SwiftThrowHelper.ThrowIfTrue(
            !_rotationTransform.TrySetWorldPose(
                _rotationTransform.WorldPosition,
                rotation),
            nameof(FixedTransform),
            "Rotation transform cannot represent the requested world rotation.");
    }

    private void PublishAuthoritativePose()
    {
        SetPositionTransformWorldPosition(Position3d);
        SetRotationTransformWorldRotation(Rotation);
    }

    public void CheckChangedValues()
    {
        // Keep each buffer set until the next visualization pass so the host
        // observes at least one presentation update after a state change.
        if (_positionMutated)
        {
            _positionChangedBuffer = _positionMutated;
            _positionMutated = false;
            _settingVisualsCounter = Context.FrameRate;
        }
        else
        {
            _positionChangedBuffer = false;
        }

        if (_rotationMutated)
        {
            _rotationChangedBuffer = _rotationMutated;
            _rotationMutated = false;
            _settingVisualsCounter = Context.FrameRate;
        }
        else
        {
            _rotationChangedBuffer = false;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetVisualPosition(Vector3d position)
    {
        _lastVisualPosition = _visualPosition;
        _visualPosition = position;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetVisualRotation(FixedQuaternion rotation) =>
        StoreVisualRotation(rotation.Normalized);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void StoreVisualRotation(FixedQuaternion rotation)
    {
        _lastVisualRotation = _visualRotation;
        _visualRotation = rotation;
    }
}
