//=======================================================================
// SolidBody2D.ContinuousCollision.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using System.Runtime.CompilerServices;

namespace Gravitas;

public sealed partial class SolidBody2D
{
    private LSCollider? _continuousCollisionHandoffIgnoredCollider3D;
    private LSCollider2D? _continuousCollisionHandoffIgnoredCollider2D;

    internal Vector2d ContinuousCollisionFrameStart
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionFrameStart;
    }

    internal Vector2d ContinuousCollisionFrameDisplacement
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionFrameDisplacement;
    }

    internal Fixed64 ContinuousCollisionFrameRotation
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionFrameRotation;
    }

    internal void EnsureContinuousCollisionFramePrepared(int token)
    {
        if (_continuousCollisionFrameToken == token)
            return;

        _continuousCollisionFrameToken = token;
        _continuousCollisionFrameStart = _position;
        _continuousCollisionFrameDisplacement = PredictContinuousCollisionDisplacement();
        _continuousCollisionFrameRotation = _rotation;
    }

    private Vector2d PredictContinuousCollisionDisplacement()
    {
        if (!CanTranslate || _isSleeping)
            return Vector2d.Zero;

        Fixed64 deltaTime = Context.DeltaTime;
        Vector2d predictedVelocity = _linearVelocity + (_deltaAcceleration + Gravity) * deltaTime;
        return predictedVelocity.MagnitudeSquared > Fixed64.Epsilon
            ? predictedVelocity * deltaTime
            : Vector2d.Zero;
    }
}
