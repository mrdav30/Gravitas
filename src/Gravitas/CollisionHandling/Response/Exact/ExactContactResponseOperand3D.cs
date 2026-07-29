//=======================================================================
// ExactContactResponseOperand3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Describes one participant in an exact rigid-body contact response.
/// </summary>
internal readonly struct ExactContactResponseOperand3D
{
    internal ExactContactResponseOperand3D(
        in ExactLever3D lever,
        Vector3d linearVelocity,
        Vector3d angularVelocity,
        Vector3d linearImpulseAxis,
        Fixed64 inverseMass,
        Fixed3x3 inverseInertia)
    {
        Lever = lever;
        LinearVelocity = linearVelocity;
        AngularVelocity = angularVelocity;
        LinearImpulseAxis = linearImpulseAxis;
        InverseMass = inverseMass;
        InverseInertia = inverseInertia;
    }

    internal ExactLever3D Lever { get; }

    internal Vector3d LinearVelocity { get; }

    internal Vector3d AngularVelocity { get; }

    internal Vector3d LinearImpulseAxis { get; }

    internal Fixed64 InverseMass { get; }

    internal Fixed3x3 InverseInertia { get; }
}
