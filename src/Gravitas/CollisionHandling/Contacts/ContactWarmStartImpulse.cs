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
    {
        NormalImpulse = normalImpulse;
        TangentImpulse = tangentImpulse;
    }

    public Fixed64 NormalImpulse { get; }

    public Fixed64 TangentImpulse { get; }
}
