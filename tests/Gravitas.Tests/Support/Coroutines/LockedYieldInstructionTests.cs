using FixedMathSharp;
using FluentAssertions;
using Gravitas.Support;
using System;
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
    public void WaitForRealSeconds_ShouldUseBoundContextDeltaTime()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();
        contextA.SetFrameRate(4);
        contextB.SetFrameRate(2);

        ILockedYieldInstruction waitA = contextA.Coroutines.WaitForRealSeconds(Fixed64.Half);
        ILockedYieldInstruction waitB = contextB.Coroutines.WaitForRealSeconds(Fixed64.Half);

        waitA.KeepWaiting.Should().BeTrue();
        waitA.KeepWaiting.Should().BeFalse();
        waitB.KeepWaiting.Should().BeFalse();
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
        wait.MoveNext().Should().BeFalse();
    }
}
