//=======================================================================
// SolidBody.ContinuousCollision.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using System.Runtime.CompilerServices;

namespace Gravitas;

public partial class SolidBody
{
    private LSCollider? _continuousCollisionHandoffIgnoredCollider3D;
    private LSCollider2D? _continuousCollisionHandoffIgnoredCollider2D;

    internal Vector3d ContinuousCollisionFrameStart
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionFrameStart;
    }

    internal Vector3d ContinuousCollisionFrameDisplacement
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionFrameDisplacement;
    }

    internal FixedQuaternion ContinuousCollisionFrameRotation
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _continuousCollisionFrameRotation;
    }

    internal void EnsureContinuousCollisionFramePrepared(int token)
    {
        if (_continuousCollisionFrameToken == token)
            return;

        _continuousCollisionFrameToken = token;
        _continuousCollisionFrameStart = Position3d;
        _continuousCollisionFrameDisplacement = PredictContinuousCollisionDisplacement();
        _continuousCollisionFrameRotation = Rotation;
    }

    private Vector3d PredictContinuousCollisionDisplacement()
    {
        if (!CanTranslate || _isSleeping)
            return Vector3d.Zero;

        Fixed64 deltaTime = Context.DeltaTime;
        PhysicsEnvironment environment = Context.Environment;
        Vector3d predictedVelocity = _linearVelocity
            + _impulseStore
            + (ProjectLinearMotion(_deltaAcceleration) * deltaTime);
        if (!IsGrounded && (_freezeAxes & BodyFreezeAxes3D.PositionY) != BodyFreezeAxes3D.PositionY)
            predictedVelocity.Y -= environment.Gravity * _gravityScale * deltaTime;

        predictedVelocity.Y = FixedMath.Max(predictedVelocity.Y, -environment.MaxFallSpeed);
        Fixed64 predictedSpeed = predictedVelocity.Magnitude;
        if (predictedSpeed > environment.MaxSpeed)
            predictedVelocity = predictedVelocity.Normalized * environment.MaxSpeed;
        else if (predictedSpeed <= environment.MinSpeed)
            predictedVelocity = Vector3d.Zero;

        return ProjectLinearMotion(predictedVelocity) * deltaTime;
    }
}
