using FluentAssertions;
using Gravitas.Support;
using System;
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

    [Fact]
    public void Simulate_ShouldNotRunCoroutinesStartedDuringCurrentTick()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        int parentSteps = 0;
        int childSteps = 0;

        context.Coroutines.StartCoroutine(Parent());

        context.Simulate();

        parentSteps.Should().Be(1);
        childSteps.Should().Be(0);
        context.Coroutines.ActiveCoroutineCount.Should().Be(2);

        context.Simulate();

        childSteps.Should().Be(1);

        IEnumerator<ILockedYieldInstruction> Parent()
        {
            parentSteps++;
            context.Coroutines.StartCoroutine(Child());
            yield return context.Coroutines.WaitForNextSimulate();
        }

        IEnumerator<ILockedYieldInstruction> Child()
        {
            childSteps++;
            yield return context.Coroutines.WaitForNextSimulate();
        }
    }

    [Fact]
    public void StopCoroutine_ShouldRejectForeignOwnerAndIgnoreInactiveHandles()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();
        LSCoroutine coroutine = contextA.Coroutines.StartCoroutine(WaitForever(contextA));

        Action wrongOwner = () => contextB.Coroutines.StopCoroutine(coroutine);
        wrongOwner.Should().Throw<ArgumentException>().WithParameterName("coroutine");

        contextA.Coroutines.StopCoroutine(coroutine);
        contextA.Coroutines.StopCoroutine(coroutine);

        coroutine.Active.Should().BeFalse();
        contextA.Coroutines.ActiveCoroutineCount.Should().Be(0);
        contextB.Coroutines.ActiveCoroutineCount.Should().Be(0);
    }

    [Fact]
    public void ResetInitializeAndDeactivate_ShouldEndActiveCoroutinesAndClearServiceState()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        int disposed = 0;

        LSCoroutine resetCoroutine = context.Coroutines.StartCoroutine(DisposableWait(context, () => disposed++));
        context.Simulate();
        context.Coroutines.Reset();

        resetCoroutine.Active.Should().BeFalse();
        disposed.Should().Be(1);
        context.Coroutines.ActiveCoroutineCount.Should().Be(0);

        LSCoroutine initializedCoroutine = context.Coroutines.StartCoroutine(DisposableWait(context, () => disposed++));
        context.Simulate();
        context.Coroutines.Initialize();

        initializedCoroutine.Active.Should().BeFalse();
        disposed.Should().Be(2);
        context.Coroutines.ActiveCoroutineCount.Should().Be(0);

        LSCoroutine deactivatedCoroutine = context.Coroutines.StartCoroutine(DisposableWait(context, () => disposed++));
        context.Simulate();
        context.Coroutines.Deactivate();
        context.Coroutines.Deactivate();

        deactivatedCoroutine.Active.Should().BeFalse();
        disposed.Should().Be(3);
        context.Coroutines.ActiveCoroutineCount.Should().Be(0);
        context.Coroutines.Context.Should().BeSameAs(context);
    }

    private static IEnumerator<ILockedYieldInstruction> WaitForNext(
        GravitasWorldContext context,
        System.Action onResume)
    {
        yield return context.Coroutines.WaitForNextSimulate();
        onResume();
    }

    private static IEnumerator<ILockedYieldInstruction> WaitForever(GravitasWorldContext context)
    {
        while (true)
            yield return context.Coroutines.WaitForNextSimulate();
    }

    private static IEnumerator<ILockedYieldInstruction> DisposableWait(
        GravitasWorldContext context,
        Action onDispose)
    {
        try
        {
            yield return context.Coroutines.WaitForFrames(16);
        }
        finally
        {
            onDispose();
        }
    }
}
