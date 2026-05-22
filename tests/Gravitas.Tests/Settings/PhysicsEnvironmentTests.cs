using FixedMathSharp;
using FluentAssertions;
using Xunit;

namespace Gravitas.Tests.Settings;

public sealed class PhysicsEnvironmentTests
{
    [Fact]
    public void Default_ShouldMatchLegacyPhysicsConstants()
    {
        PhysicsEnvironment environment = PhysicsEnvironment.Default();

        environment.Gravity.Should().Be((Fixed64)9.8f);
        environment.AirDensity.Should().Be((Fixed64)1.225f);
        environment.MinSpeed.Should().Be((Fixed64)0.00001f);
        environment.MaxSpeed.Should().Be((Fixed64)7f);
        environment.MaxFallSpeed.Should().Be((Fixed64)9.8f);
        environment.FrictionTransitionSpeed.Should().Be((Fixed64)0.2f);
        environment.DecelerationMultiplier.Should().Be((Fixed64)10f);
        environment.DampingFactor.Should().Be((Fixed64)0.95f);
        environment.CullDistanceMax.Should().Be(PhysicsSettings.DefaultFrameRate / 3);
        environment.CullFastDistanceMax.Should().Be(Fixed64.One * 4 * (Fixed64.One * 4));
        environment.CullVelocityStep.Should().Be(2);
        environment.CullVelocityMax.Should().Be(4);
        environment.CullTimeStep.Should().Be(PhysicsSettings.DefaultFrameRate * 3);
        environment.CullTimeMax.Should().Be(PhysicsSettings.DefaultFrameRate / 5);
        PhysicsEnvironment.PoundToNewton.Should().Be((Fixed64)4.44822162f);
        PhysicsEnvironment.KilogramToPound.Should().Be((Fixed64)2.20462262f);
    }

    [Fact]
    public void Contexts_ShouldKeepEnvironmentStateIsolated()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();

        contextA.Environment.Gravity = (Fixed64)1.62f;
        contextA.Environment.CullDistanceMax = 4;
        contextB.Environment.Gravity = (Fixed64)24.79f;
        contextB.Environment.CullDistanceMax = 12;

        contextA.Environment.Gravity.Should().Be((Fixed64)1.62f);
        contextB.Environment.Gravity.Should().Be((Fixed64)24.79f);
        contextA.Environment.CullDistanceMax.Should().Be(4);
        contextB.Environment.CullDistanceMax.Should().Be(12);
    }
}
