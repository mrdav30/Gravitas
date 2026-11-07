//=======================================================================
// ContactNormalVelocityDeltaResult3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Side-effect-free 3D velocity deltas for a normal response whose impulse
/// scalar does not need to be representable.
/// </summary>
internal readonly struct ContactNormalVelocityDeltaResult3D
{
    public ContactNormalVelocityDeltaResult3D(
        Fixed64 normalVelocity,
        Vector3d linearVelocityDeltaA,
        Vector3d angularVelocityDeltaA,
        Vector3d linearVelocityDeltaB,
        Vector3d angularVelocityDeltaB)
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

    public ContactNormalVelocityDeltaResult3D(
        Fixed64 normalVelocity,
        Vector3d linearVelocityDeltaA,
        Vector3d angularVelocityDeltaA,
        Vector3d linearVelocityDeltaB,
        Vector3d angularVelocityDeltaB,
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

    public Vector3d LinearVelocityDeltaA { get; }

    public Vector3d AngularVelocityDeltaA { get; }

    public Vector3d LinearVelocityDeltaB { get; }

    public Vector3d AngularVelocityDeltaB { get; }
}
