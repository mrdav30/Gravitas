//=======================================================================
// ContactNormalImpulseResultMixed.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Side-effect-free mixed 3D/2D contact-normal impulse result.
/// </summary>
internal readonly struct ContactNormalImpulseResultMixed
{
    public ContactNormalImpulseResultMixed(
        Fixed64 normalVelocity,
        Fixed64 impulseScalar,
        Vector3d linearVelocityDelta3D,
        Vector3d angularVelocityDelta3D,
        Vector2d linearVelocityDelta2D,
        Fixed64 angularVelocityDelta2D)
        : this(
            normalVelocity,
            impulseScalar,
            impulseScalar,
            linearVelocityDelta3D,
            angularVelocityDelta3D,
            linearVelocityDelta2D,
            angularVelocityDelta2D)
    {
    }

    public ContactNormalImpulseResultMixed(
        Fixed64 normalVelocity,
        Fixed64 impulseScalar,
        Fixed64 appliedImpulseScalar,
        Vector3d linearVelocityDelta3D,
        Vector3d angularVelocityDelta3D,
        Vector2d linearVelocityDelta2D,
        Fixed64 angularVelocityDelta2D,
        bool hasRepresentableNormalVelocity = true,
        bool hasRepresentableAppliedImpulse = true)
    {
        NormalVelocity = normalVelocity;
        ImpulseScalar = impulseScalar;
        AppliedImpulseScalar = appliedImpulseScalar;
        LinearVelocityDelta3D = linearVelocityDelta3D;
        AngularVelocityDelta3D = angularVelocityDelta3D;
        LinearVelocityDelta2D = linearVelocityDelta2D;
        AngularVelocityDelta2D = angularVelocityDelta2D;
        HasRepresentableNormalVelocity = hasRepresentableNormalVelocity;
        HasRepresentableAppliedImpulse = hasRepresentableAppliedImpulse;
    }

    public Fixed64 NormalVelocity { get; }

    public Fixed64 ImpulseScalar { get; }

    public Fixed64 AppliedImpulseScalar { get; }

    public bool HasRepresentableNormalVelocity { get; }

    public bool HasRepresentableAppliedImpulse { get; }

    public Vector3d LinearVelocityDelta3D { get; }

    public Vector3d AngularVelocityDelta3D { get; }

    public Vector2d LinearVelocityDelta2D { get; }

    public Fixed64 AngularVelocityDelta2D { get; }
}
