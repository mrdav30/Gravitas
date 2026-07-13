//=======================================================================
// ContactWarmStartImpulse.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Stores deterministic impulse scalars from the previous solve for one stable manifold contact.
/// </summary>
internal readonly struct ContactWarmStartImpulse
{
    public ContactWarmStartImpulse(Fixed64 normalImpulse, Fixed64 tangentImpulse)
        : this(Vector3d.Zero, normalImpulse, tangentImpulse, Fixed64.Zero) { }

    public ContactWarmStartImpulse(
        Vector3d normal,
        Fixed64 normalImpulse,
        Fixed64 tangentImpulse,
        Fixed64 secondaryTangentImpulse)
    {
        Normal = normal;
        NormalImpulse = normalImpulse;
        TangentImpulse = tangentImpulse;
        SecondaryTangentImpulse = secondaryTangentImpulse;
    }

    public Vector3d Normal { get; }

    public Fixed64 NormalImpulse { get; }

    public Fixed64 TangentImpulse { get; }

    /// <summary>
    /// Secondary tangent impulse for 3D contacts. Pure 2D contacts leave this value at zero.
    /// </summary>
    public Fixed64 SecondaryTangentImpulse { get; }
}
