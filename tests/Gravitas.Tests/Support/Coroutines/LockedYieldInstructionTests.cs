using FixedMathSharp;
using FluentAssertions;
using Gravitas.Support;
using System;
using System.Reflection;
using Xunit;

namespace Gravitas.Tests.Support.Coroutines;

public sealed class LockedYieldInstructionTests
{
    [Fact]
    public void WaitForFrames_ShouldRejectNegativeFrameCount()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();

        Action act = () => _ = new WaitForFrames(context, -1);

        act.Should().Throw<ArgumentOutOfRangeException>()
            .WithParameterName("numberOfFrames");
    }

    [Fact]
    public void WaitForFrames_ShouldUseBoundContextFrameCount()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();
        ILockedYieldInstruction wait = contextA.Coroutines.WaitForFrames(2);

        contextB.Simulate();
        wait.KeepWaiting.Should().BeTrue();

        contextA.Simulate();
        wait.KeepWaiting.Should().BeTrue();

        contextA.Simulate();
        wait.KeepWaiting.Should().BeFalse();
    }

    [Fact]
    public void WaitForFrames_ShouldImplementEnumeratorContract()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        ILockedYieldInstruction wait = new WaitForFrames(context, 1);

        wait.Current.Should().BeNull();
        wait.MoveNext().Should().BeTrue();
        wait.Reset();
        wait.Dispose();

        context.Simulate();
        wait.MoveNext().Should().BeFalse();
    }

    [Fact]
    public void WaitForNextSimulate_ShouldUseBoundContextFrameCount()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();
        ILockedYieldInstruction wait = contextA.Coroutines.WaitForNextSimulate();

        contextB.Simulate();
        wait.KeepWaiting.Should().BeTrue();

        contextA.Simulate();
        wait.KeepWaiting.Should().BeFalse();
    }

    [Fact]
    public void WaitForNextSimulate_ShouldImplementEnumeratorContract()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        ILockedYieldInstruction wait = new WaitForNextSimulate(context);

        wait.Current.Should().BeNull();
        wait.MoveNext().Should().BeTrue();
        wait.Reset();
        wait.Dispose();

        context.Simulate();
        wait.MoveNext().Should().BeFalse();
    }

    [Fact]
    public void WaitForFrames_WhenFrameCounterWraps_ShouldCompleteAfterRequestedFrames()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        SetFrameCount(context, int.MaxValue - 1);
        ILockedYieldInstruction wait = context.Coroutines.WaitForFrames(4);

        context.Simulate();
        wait.KeepWaiting.Should().BeTrue();

        context.Simulate();
        wait.KeepWaiting.Should().BeTrue();
        context.Simulate();
        wait.KeepWaiting.Should().BeTrue();
        context.Simulate();
        wait.KeepWaiting.Should().BeFalse();
    }

    [Fact]
    public void WaitForNextSimulate_WhenFrameCounterWraps_ShouldCompleteOnNextFrame()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        SetFrameCount(context, int.MaxValue);
        ILockedYieldInstruction wait = context.Coroutines.WaitForNextSimulate();

        context.Simulate();

        wait.KeepWaiting.Should().BeFalse();
    }

    [Fact]
    public void WaitForRealSeconds_ShouldObserveBoundContextClockWithoutGetterMutation()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();
        contextA.SetFrameRate(4);
        contextB.SetFrameRate(2);

        ILockedYieldInstruction waitA = contextA.Coroutines.WaitForRealSeconds(Fixed64.Half);
        ILockedYieldInstruction waitB = contextB.Coroutines.WaitForRealSeconds(Fixed64.Half);

        waitA.Context.Should().BeSameAs(contextA);
        waitA.KeepWaiting.Should().BeTrue();
        waitA.KeepWaiting.Should().BeTrue();
        waitB.KeepWaiting.Should().BeTrue();

        contextA.Simulate();
        contextB.Simulate();

        waitA.KeepWaiting.Should().BeTrue();
        waitB.KeepWaiting.Should().BeFalse();

        contextA.Simulate();

        waitA.KeepWaiting.Should().BeFalse();
    }

    [Fact]
    public void WaitForRealSeconds_ShouldImplementEnumeratorContract()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.SetFrameRate(2);
        ILockedYieldInstruction wait = new WaitForRealSeconds(context, Fixed64.One);

        wait.Current.Should().BeNull();
        wait.MoveNext().Should().BeTrue();
        wait.Reset();
        wait.Dispose();

        context.Simulate();
        wait.MoveNext().Should().BeTrue();
        context.Simulate();
        wait.MoveNext().Should().BeFalse();
    }

    [Fact]
    public void WaitForRealSeconds_ShouldRejectNegativeDuration()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();

        Action act = () => _ = context.Coroutines.WaitForRealSeconds(-Fixed64.One);

        act.Should().Throw<ArgumentException>();
    }

    [Fact]
    public void WaitForRealSeconds_ShouldRejectNullContext()
    {
        Action act = () => _ = new WaitForRealSeconds(null!, Fixed64.Zero);

        act.Should().Throw<ArgumentNullException>().WithParameterName("context");
    }

    private static void SetFrameCount(GravitasWorldContext context, int frameCount)
    {
        FieldInfo clockField = typeof(GravitasWorldContext).GetField(
            "_clock",
            BindingFlags.Instance | BindingFlags.NonPublic)!;
        object clock = clockField.GetValue(context)!;
        PropertyInfo frameCountProperty = clock.GetType().GetProperty(
            nameof(GravitasWorldContext.FrameCount),
            BindingFlags.Instance | BindingFlags.Public)!;
        frameCountProperty.SetValue(clock, frameCount);
    }
}
