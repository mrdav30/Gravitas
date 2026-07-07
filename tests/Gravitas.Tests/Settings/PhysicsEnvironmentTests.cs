using FixedMathSharp;
using FluentAssertions;
using System;
using Xunit;

namespace Gravitas.Tests.Settings;

public sealed class PhysicsEnvironmentTests
{
    [Fact]
    public void Default_ShouldMatchStandardPhysicsConstants()
    {
        PhysicsEnvironment environment = PhysicsEnvironment.Default();

        PhysicsEnvironment.DefaultGravity.Should().Be((Fixed64)9.8f);
        PhysicsEnvironment.DefaultAirDensity.Should().Be((Fixed64)1.225f);
        PhysicsEnvironment.DefaultMinSpeed.Should().Be((Fixed64)0.00001f);
        PhysicsEnvironment.DefaultMaxSpeed.Should().Be((Fixed64)7f);
        PhysicsEnvironment.DefaultMaxFallSpeed.Should().Be((Fixed64)9.8f);
        PhysicsEnvironment.DefaultFrictionTransitionSpeed.Should().Be((Fixed64)0.2f);
        PhysicsEnvironment.DefaultDecelerationMultiplier.Should().Be((Fixed64)10f);
        PhysicsEnvironment.DefaultDampingFactor.Should().Be((Fixed64)0.95f);
        PhysicsEnvironment.DefaultCullDistanceFrameDivisor.Should().Be(3);
        PhysicsEnvironment.DefaultCullFastDistance.Should().Be(4);
        PhysicsEnvironment.DefaultCullFastDistanceMax.Should().Be(Fixed64.One * 4 * (Fixed64.One * 4));
        PhysicsEnvironment.DefaultCullVelocityStep.Should().Be(2);
        PhysicsEnvironment.DefaultCullVelocityMax.Should().Be(4);
        PhysicsEnvironment.DefaultCullTimeStepFrameMultiplier.Should().Be(3);
        PhysicsEnvironment.DefaultCullTimeMaxFrameDivisor.Should().Be(5);
        PhysicsEnvironment.PoundToNewton.Should().Be((Fixed64)4.44822162f);
        PhysicsEnvironment.KilogramToPound.Should().Be((Fixed64)2.20462262f);

        environment.Gravity.Should().Be(PhysicsEnvironment.DefaultGravity);
        environment.AirDensity.Should().Be(PhysicsEnvironment.DefaultAirDensity);
        environment.MinSpeed.Should().Be(PhysicsEnvironment.DefaultMinSpeed);
        environment.MaxSpeed.Should().Be(PhysicsEnvironment.DefaultMaxSpeed);
        environment.MaxFallSpeed.Should().Be(PhysicsEnvironment.DefaultMaxFallSpeed);
        environment.FrictionTransitionSpeed.Should().Be(PhysicsEnvironment.DefaultFrictionTransitionSpeed);
        environment.DecelerationMultiplier.Should().Be(PhysicsEnvironment.DefaultDecelerationMultiplier);
        environment.DampingFactor.Should().Be(PhysicsEnvironment.DefaultDampingFactor);
        environment.CullDistanceMax.Should().Be(
            PhysicsSettings.DefaultFrameRate / PhysicsEnvironment.DefaultCullDistanceFrameDivisor);
        environment.CullFastDistanceMax.Should().Be(PhysicsEnvironment.DefaultCullFastDistanceMax);
        environment.CullVelocityStep.Should().Be(PhysicsEnvironment.DefaultCullVelocityStep);
        environment.CullVelocityMax.Should().Be(PhysicsEnvironment.DefaultCullVelocityMax);
        environment.CullTimeStep.Should().Be(
            PhysicsSettings.DefaultFrameRate * PhysicsEnvironment.DefaultCullTimeStepFrameMultiplier);
        environment.CullTimeMax.Should().Be(
            PhysicsSettings.DefaultFrameRate / PhysicsEnvironment.DefaultCullTimeMaxFrameDivisor);
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

    [Fact]
    public void Default_ShouldRejectFrameRatesThatQuantizeDeltaTimeAtOrBelowEpsilon()
    {
        Action action = () => _ = PhysicsEnvironment.Default(PhysicsSettings.MaxResolvableFrameRate + 1);

        action.Should().Throw<ArgumentOutOfRangeException>().WithParameterName("frameRate");
    }
}
