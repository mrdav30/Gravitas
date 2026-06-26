using FixedMathSharp;
using FluentAssertions;
using System;
using Xunit;

namespace Gravitas.Tests.Settings;

public sealed class PhysicsSettingsTests
{
    [Fact]
    public void ApplySettings_ShouldKeepFrameRateAndCollisionMatrixContextLocal()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();
        var matrixA = new[,]
        {
            { true, false },
            { false, true }
        };
        var matrixB = new[,]
        {
            { false, true },
            { true, false }
        };

        contextA.ApplySettings(new PhysicsSettings(20, matrixA) { PoolingEnabled = false });
        contextB.ApplySettings(new PhysicsSettings(64, matrixB) { PoolingEnabled = true });

        contextA.Settings.FrameRate.Should().Be(20);
        contextB.Settings.FrameRate.Should().Be(64);
        contextA.FrameRate.Should().Be(20);
        contextB.FrameRate.Should().Be(64);
        contextA.DeltaTime.Should().Be(Fixed64.One / (Fixed64)20);
        contextB.DeltaTime.Should().Be(Fixed64.One / (Fixed64)64);
        contextA.Settings.CollisionMatrix.Should().BeSameAs(matrixA);
        contextB.Settings.CollisionMatrix.Should().BeSameAs(matrixB);
        contextA.Settings.PoolingEnabled.Should().BeFalse();
        contextB.Settings.PoolingEnabled.Should().BeTrue();
        contextA.Settings.DefaultContinuousCollisionMode.Should().Be(ContinuousCollisionMode.Discrete);
        contextB.Settings.DefaultContinuousCollisionMode.Should().Be(ContinuousCollisionMode.Discrete);
        contextA.Settings.ContinuousCollisionMaxToiIterations.Should().Be(PhysicsSettings.DefaultContinuousCollisionMaxToiIterations);
        contextB.Settings.ContinuousCollisionMaxToiIterations.Should().Be(PhysicsSettings.DefaultContinuousCollisionMaxToiIterations);
        contextA.Settings.DiscreteSolverIterations.Should().Be(PhysicsSettings.DefaultDiscreteSolverIterations);
        contextB.Settings.DiscreteSolverIterations.Should().Be(PhysicsSettings.DefaultDiscreteSolverIterations);
        contextA.Settings.RestitutionVelocityThreshold.Should().Be(PhysicsSettings.DefaultRestitutionVelocityThreshold);
        contextB.Settings.RestitutionVelocityThreshold.Should().Be(PhysicsSettings.DefaultRestitutionVelocityThreshold);
    }

    [Fact]
    public void SetFrameRate_ShouldUpdateSettingsAndClockTogether()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();

        context.SetFrameRate(48);

        context.Settings.FrameRate.Should().Be(48);
        context.FrameRate.Should().Be(48);
        context.DeltaTime.Should().Be(Fixed64.One / (Fixed64)48);
        context.InvDeltaTime.Should().Be(Fixed64.One / context.DeltaTime);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(-1)]
    public void ContinuousCollisionMaxToiIterations_ShouldRejectNonPositiveValues(int value)
    {
        var settings = PhysicsSettings.DefaultSettings();

        Action action = () => settings.ContinuousCollisionMaxToiIterations = value;

        action.Should().Throw<ArgumentException>().WithParameterName("value");
    }

    [Theory]
    [InlineData(0)]
    [InlineData(-1)]
    public void DiscreteSolverIterations_ShouldRejectNonPositiveValues(int value)
    {
        var settings = PhysicsSettings.DefaultSettings();

        Action action = () => settings.DiscreteSolverIterations = value;

        action.Should().Throw<ArgumentException>().WithParameterName("value");
    }

    [Fact]
    public void RestitutionVelocityThreshold_ShouldStoreNonNegativeValues()
    {
        var settings = PhysicsSettings.DefaultSettings();

        settings.RestitutionVelocityThreshold.Should().Be((Fixed64)0.25f);
        settings.RestitutionVelocityThreshold = Fixed64.Zero;
        settings.RestitutionVelocityThreshold.Should().Be(Fixed64.Zero);
        settings.RestitutionVelocityThreshold = Fixed64.FromFraction(3, 2);
        settings.RestitutionVelocityThreshold.Should().Be(Fixed64.FromFraction(3, 2));
    }

    [Fact]
    public void RestitutionVelocityThreshold_ShouldRejectNegativeValues()
    {
        var settings = PhysicsSettings.DefaultSettings();

        Action action = () => settings.RestitutionVelocityThreshold = -Fixed64.Epsilon;

        action.Should().Throw<ArgumentException>().WithParameterName("value");
    }
}
