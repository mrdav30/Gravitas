using FluentAssertions;
using Gravitas.Support;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Support.Coroutines;

public sealed class GravitasCoroutineServiceTests
{
    [Fact]
    public void StartCoroutine_ShouldResumeAgainstOwningContextClock()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        int step = 0;

        context.Coroutines.StartCoroutine(Run());

        context.Simulate();
        step.Should().Be(1);
        context.Coroutines.ActiveCoroutineCount.Should().Be(1);

        context.Simulate();
        step.Should().Be(1);

        context.Simulate();
        step.Should().Be(2);
        context.Coroutines.ActiveCoroutineCount.Should().Be(0);

        IEnumerator<ILockedYieldInstruction> Run()
        {
            step++;
            yield return context.Coroutines.WaitForFrames(2);
            step++;
        }
    }

    [Fact]
    public void StartCoroutine_ShouldAdvanceOnlyOwningContext()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();
        int resumedA = 0;
        int resumedB = 0;

        contextA.Coroutines.StartCoroutine(WaitForNext(contextA, () => resumedA++));
        contextB.Coroutines.StartCoroutine(WaitForNext(contextB, () => resumedB++));

        contextA.Simulate();
        contextA.Simulate();

        resumedA.Should().Be(1);
        resumedB.Should().Be(0);
        contextA.Coroutines.ActiveCoroutineCount.Should().Be(0);
        contextB.Coroutines.ActiveCoroutineCount.Should().Be(1);

        contextB.Simulate();
        contextB.Simulate();

        resumedB.Should().Be(1);
        contextB.Coroutines.ActiveCoroutineCount.Should().Be(0);
    }

    private static IEnumerator<ILockedYieldInstruction> WaitForNext(
        GravitasWorldContext context,
        System.Action onResume)
    {
        yield return context.Coroutines.WaitForNextSimulate();
        onResume();
    }
}
