//=======================================================================
// ContactNormalImpulseResult2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Side-effect-free 2D contact-normal impulse result for two participants.
/// </summary>
internal readonly struct ContactNormalImpulseResult2D
{
    public ContactNormalImpulseResult2D(
        Fixed64 normalVelocity,
        Fixed64 impulseScalar,
        Vector2d linearVelocityDeltaA,
        Fixed64 angularVelocityDeltaA,
        Vector2d linearVelocityDeltaB,
        Fixed64 angularVelocityDeltaB)
        : this(
            normalVelocity,
            impulseScalar,
            impulseScalar,
            linearVelocityDeltaA,
            angularVelocityDeltaA,
            linearVelocityDeltaB,
            angularVelocityDeltaB)
    {
    }

    public ContactNormalImpulseResult2D(
        Fixed64 normalVelocity,
        Fixed64 impulseScalar,
        Fixed64 appliedImpulseScalar,
        Vector2d linearVelocityDeltaA,
        Fixed64 angularVelocityDeltaA,
        Vector2d linearVelocityDeltaB,
        Fixed64 angularVelocityDeltaB,
        bool hasRepresentableNormalVelocity = true,
        bool hasRepresentableAppliedImpulse = true,
        bool hasRepresentableAccumulatedImpulse = true)
    {
        NormalVelocity = normalVelocity;
        ImpulseScalar = impulseScalar;
        AppliedImpulseScalar = appliedImpulseScalar;
        LinearVelocityDeltaA = linearVelocityDeltaA;
        AngularVelocityDeltaA = angularVelocityDeltaA;
        LinearVelocityDeltaB = linearVelocityDeltaB;
        AngularVelocityDeltaB = angularVelocityDeltaB;
        HasRepresentableNormalVelocity = hasRepresentableNormalVelocity;
        HasRepresentableAppliedImpulse = hasRepresentableAppliedImpulse;
        HasRepresentableAccumulatedImpulse =
            hasRepresentableAccumulatedImpulse;
    }

    public Fixed64 NormalVelocity { get; }

    public Fixed64 ImpulseScalar { get; }

    public Fixed64 AppliedImpulseScalar { get; }

    public bool HasRepresentableNormalVelocity { get; }

    public bool HasRepresentableAppliedImpulse { get; }

    public bool HasRepresentableAccumulatedImpulse { get; }

    public Vector2d LinearVelocityDeltaA { get; }

    public Fixed64 AngularVelocityDeltaA { get; }

    public Vector2d LinearVelocityDeltaB { get; }

    public Fixed64 AngularVelocityDeltaB { get; }
}
