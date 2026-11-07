//=======================================================================
// ContactNormalVelocityDeltaResult2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Side-effect-free 2D velocity deltas for a normal response whose impulse
/// scalar does not need to be representable.
/// </summary>
internal readonly struct ContactNormalVelocityDeltaResult2D
{
    public ContactNormalVelocityDeltaResult2D(
        Fixed64 normalVelocity,
        Vector2d linearVelocityDeltaA,
        Fixed64 angularVelocityDeltaA,
        Vector2d linearVelocityDeltaB,
        Fixed64 angularVelocityDeltaB)
        : this(
            normalVelocity,
            linearVelocityDeltaA,
            angularVelocityDeltaA,
            linearVelocityDeltaB,
            angularVelocityDeltaB,
            normalVelocity < Fixed64.Zero,
            hasRepresentableNormalVelocity: true)
    {
    }

    public ContactNormalVelocityDeltaResult2D(
        Fixed64 normalVelocity,
        Vector2d linearVelocityDeltaA,
        Fixed64 angularVelocityDeltaA,
        Vector2d linearVelocityDeltaB,
        Fixed64 angularVelocityDeltaB,
        bool isClosing,
        bool hasRepresentableNormalVelocity)
    {
        NormalVelocity = normalVelocity;
        LinearVelocityDeltaA = linearVelocityDeltaA;
        AngularVelocityDeltaA = angularVelocityDeltaA;
        LinearVelocityDeltaB = linearVelocityDeltaB;
        AngularVelocityDeltaB = angularVelocityDeltaB;
        IsClosing = isClosing;
        HasRepresentableNormalVelocity = hasRepresentableNormalVelocity;
    }

    public Fixed64 NormalVelocity { get; }

    public bool IsClosing { get; }

    public bool HasRepresentableNormalVelocity { get; }

    public Vector2d LinearVelocityDeltaA { get; }

    public Fixed64 AngularVelocityDeltaA { get; }

    public Vector2d LinearVelocityDeltaB { get; }

    public Fixed64 AngularVelocityDeltaB { get; }
}
