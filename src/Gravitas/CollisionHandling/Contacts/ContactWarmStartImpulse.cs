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
