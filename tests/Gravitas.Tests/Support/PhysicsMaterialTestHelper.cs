using FixedMathSharp;
using Gravitas.Materials;

namespace Gravitas.Tests.Support;

internal static class PhysicsMaterialTestHelper
{
    public static PhysicsMaterial WithRestitution(Fixed64 restitution) =>
        new(Fixed64.One, Fixed64.One, restitution);

    public static PhysicsMaterial WithFrictionAndRestitution(Fixed64 friction, Fixed64 restitution) =>
        new(friction, friction, restitution);
}
