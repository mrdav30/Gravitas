using FixedMathSharp;
using FluentAssertions;
using Xunit;

namespace Gravitas.Tests.Runtime;

public sealed class GravitasClockTests
{
    [Fact]
    public void NewContext_ShouldUsePhysicsSettingsDefaultFrameRate()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();

        context.FrameRate.Should().Be(PhysicsSettings.DefaultFrameRate);
        context.DeltaTime.Should().Be(Fixed64.One / (Fixed64)PhysicsSettings.DefaultFrameRate);
        context.InvDeltaTime.Should().Be((Fixed64)PhysicsSettings.DefaultFrameRate);
    }

    [Fact]
    public void Simulate_ShouldAdvanceFrameCountAndTotalTime()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();

        context.Simulate();
        context.Simulate();

        context.FrameCount.Should().Be(2);
        context.TotalTime.Should().Be(context.DeltaTime * 2);
    }

    [Fact]
    public void LateSimulateAndVisualize_ShouldTrackAccumulation()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();

        context.Visualize();
        context.Visualize();
        context.LateSimulate();

        context.ResetAccumulation.Should().BeTrue();

        context.Visualize();

        context.ResetAccumulation.Should().BeFalse();
        context.AccumulatedTime.Should().Be(context.DeltaTime);
        context.ExpectedAccumulation.Should().Be(Fixed64.One);
    }

    [Fact]
    public void SetFrameRate_ShouldUpdateDeltaTime()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();

        context.SetFrameRate(64);

        context.FrameRate.Should().Be(64);
        context.DeltaTime.Should().Be(Fixed64.One / (Fixed64)64);
        context.InvDeltaTime.Should().Be((Fixed64)64);
    }
}
