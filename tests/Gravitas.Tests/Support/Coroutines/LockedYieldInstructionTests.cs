using FixedMathSharp;
using FluentAssertions;
using Gravitas.Support;
using Xunit;

namespace Gravitas.Tests.Support.Coroutines;

public sealed class LockedYieldInstructionTests
{
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
}
