//=======================================================================
// ContactNormalImpulseResult3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Side-effect-free 3D contact-normal impulse result for two participants.
/// </summary>
internal readonly struct ContactNormalImpulseResult3D
{
    public ContactNormalImpulseResult3D(
        Fixed64 normalVelocity,
        Fixed64 impulseScalar,
        Vector3d linearVelocityDeltaA,
        Vector3d angularVelocityDeltaA,
        Vector3d linearVelocityDeltaB,
        Vector3d angularVelocityDeltaB)
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

    public ContactNormalImpulseResult3D(
        Fixed64 normalVelocity,
        Fixed64 impulseScalar,
        Fixed64 appliedImpulseScalar,
        Vector3d linearVelocityDeltaA,
        Vector3d angularVelocityDeltaA,
        Vector3d linearVelocityDeltaB,
        Vector3d angularVelocityDeltaB,
        bool hasRepresentableNormalVelocity = true,
        bool hasRepresentableAppliedImpulse = true)
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
    }

    public Fixed64 NormalVelocity { get; }

    public Fixed64 ImpulseScalar { get; }

    public Fixed64 AppliedImpulseScalar { get; }

    public bool HasRepresentableNormalVelocity { get; }

    public bool HasRepresentableAppliedImpulse { get; }

    public Vector3d LinearVelocityDeltaA { get; }

    public Vector3d AngularVelocityDeltaA { get; }

    public Vector3d LinearVelocityDeltaB { get; }

    public Vector3d AngularVelocityDeltaB { get; }
}
