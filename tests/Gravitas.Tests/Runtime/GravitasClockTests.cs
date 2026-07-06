using FixedMathSharp;
using FluentAssertions;
using System;
using System.Collections.Generic;
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

    [Fact]
    public void GetFrameFromTime_ShouldUseCurrentFrameRate()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.SetFrameRate(4);

        context.GetFrameFromTime(Fixed64.Zero).Should().Be(0);
        context.GetFrameFromTime(Fixed64.FromFraction(1, 4)).Should().Be(1);
        context.GetFrameFromTime(Fixed64.FromFraction(3, 4)).Should().Be(3);
    }

    [Fact]
    public void ResetAndFrameRateHooks_ShouldRunInDeterministicOrderAndUnregister()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var calls = new List<string>();

        IDisposable resetLater = context.RegisterOnReset("reset.later", 10, () => calls.Add("reset-later"));
        IDisposable resetEarlier = context.RegisterOnReset("reset.earlier", -10, () => calls.Add("reset-earlier"));

        context.Reset();

        calls.Should().Equal("reset-earlier", "reset-later");

        resetEarlier.Dispose();
        resetEarlier.Dispose();
        resetLater.Dispose();
        calls.Clear();

        context.Reset();

        calls.Should().BeEmpty();

        IDisposable frameRateB = context.RegisterOnFrameRateChanged(
            "frame-rate.b",
            0,
            () => calls.Add($"b:{context.FrameRate}"));
        IDisposable frameRateA = context.RegisterOnFrameRateChanged(
            "frame-rate.a",
            0,
            () => calls.Add($"a:{context.FrameRate}"));

        context.SetFrameRate(12);
        context.ApplySettings(new PhysicsSettings(8, null));

        calls.Should().Equal("a:12", "b:12", "a:8", "b:8");

        frameRateA.Dispose();
        frameRateB.Dispose();
    }
}
