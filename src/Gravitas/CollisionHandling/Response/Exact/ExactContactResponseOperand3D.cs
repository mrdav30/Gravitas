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
        Fixed3x3 inverseInertia,
        Fixed64? planarAngularVelocity = null)
    {
        Lever = lever;
        LinearVelocity = linearVelocity;
        AngularVelocity = angularVelocity;
        LinearImpulseAxis = linearImpulseAxis;
        InverseMass = inverseMass;
        InverseInertia = inverseInertia;
        PlanarAngularVelocity = planarAngularVelocity;
    }

    internal ExactLever3D Lever { get; }

    internal Vector3d LinearVelocity { get; }

    internal Vector3d AngularVelocity { get; }

    // The spatial sign conversion of minimum planar yaw does not fit Fixed64.
    internal Fixed64? PlanarAngularVelocity { get; }

    // Positive 2^31 yaw delta cannot be represented by the planar delta API.
    internal bool CanRepresentAngularDelta(Vector3d delta) =>
        !PlanarAngularVelocity.HasValue || delta.Y != Fixed64.MinValue;

    internal bool CanApplyAngularDelta(Vector3d delta) => PlanarAngularVelocity.HasValue
        ? CanRepresentAngularDelta(delta) && Fixed64.TrySubtract(PlanarAngularVelocity.Value, delta.Y, out _)
        : Vector3d.TryAdd(AngularVelocity, delta, out _);

    internal Vector3d LinearImpulseAxis { get; }

    internal Fixed64 InverseMass { get; }

    internal Fixed3x3 InverseInertia { get; }
}
