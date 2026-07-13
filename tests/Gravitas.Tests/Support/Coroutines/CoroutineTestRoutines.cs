using Gravitas.Support;
using System;
using System.Collections.Generic;

namespace Gravitas.Tests.Support.Coroutines;

internal static class CoroutineTestRoutines
{
    internal static IEnumerator<ILockedYieldInstruction> WaitForever(GravitasWorldContext context)
    {
        while (true)
            yield return context.Coroutines.WaitForNextSimulate();
    }

    internal static IEnumerator<ILockedYieldInstruction> DisposableWait(
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

    internal static IEnumerator<ILockedYieldInstruction> CountAndDispose(Action onStep, Action onDispose)
    {
        try
        {
            while (true)
            {
                onStep();
                yield return null!;
            }
        }
        finally
        {
            onDispose();
        }
    }
}
