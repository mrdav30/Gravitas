//=======================================================================
// ContactNormalVelocityDeltaResultMixed.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Side-effect-free mixed velocity deltas for a normal response whose impulse
/// scalar does not need to be representable.
/// </summary>
internal readonly struct ContactNormalVelocityDeltaResultMixed
{
    public ContactNormalVelocityDeltaResultMixed(
        Fixed64 normalVelocity,
        Vector3d linearVelocityDelta3D,
        Vector3d angularVelocityDelta3D,
        Vector2d linearVelocityDelta2D,
        Fixed64 angularVelocityDelta2D)
        : this(
            normalVelocity,
            linearVelocityDelta3D,
            angularVelocityDelta3D,
            linearVelocityDelta2D,
            angularVelocityDelta2D,
            normalVelocity < Fixed64.Zero,
            hasRepresentableNormalVelocity: true)
    {
    }

    public ContactNormalVelocityDeltaResultMixed(
        Fixed64 normalVelocity,
        Vector3d linearVelocityDelta3D,
        Vector3d angularVelocityDelta3D,
        Vector2d linearVelocityDelta2D,
        Fixed64 angularVelocityDelta2D,
        bool isClosing,
        bool hasRepresentableNormalVelocity)
    {
        NormalVelocity = normalVelocity;
        LinearVelocityDelta3D = linearVelocityDelta3D;
        AngularVelocityDelta3D = angularVelocityDelta3D;
        LinearVelocityDelta2D = linearVelocityDelta2D;
        AngularVelocityDelta2D = angularVelocityDelta2D;
        IsClosing = isClosing;
        HasRepresentableNormalVelocity = hasRepresentableNormalVelocity;
    }

    public Fixed64 NormalVelocity { get; }

    public bool IsClosing { get; }

    public bool HasRepresentableNormalVelocity { get; }

    public Vector3d LinearVelocityDelta3D { get; }

    public Vector3d AngularVelocityDelta3D { get; }

    public Vector2d LinearVelocityDelta2D { get; }

    public Fixed64 AngularVelocityDelta2D { get; }
}
