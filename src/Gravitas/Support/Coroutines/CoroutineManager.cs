using SwiftCollections;
using System.Collections.Generic;

namespace Gravitas.Support;

public static class CoroutineManager
{

    static SwiftBucket<LSCoroutine>? Coroutines;

    public static void Initialize()
    {
        Coroutines?.Clear();
    }

    public static void Simulate()
    {
        if (Coroutines == null || Coroutines.PeakCount == 0)
            return;

        for (int i = 0; i < Coroutines.PeakCount; i++)
        {
            if (!Coroutines.IsAllocated(i)) continue;
            LSCoroutine coroutine = Coroutines[i];
            if (!coroutine.Active) continue;
            coroutine.Simulate();
        }
    }

    /// <summary>
    /// Starts coroutine that returns number of frames to wait.
    /// </summary>
    /// <returns>The coroutine.</returns>
    /// <param name="enumerator">Enumerator.</param>
    public static LSCoroutine StartCoroutine(IEnumerator<ILockedYieldInstruction> enumerator)
    {
        LSCoroutine coroutine = new(enumerator);
        Coroutines ??= new();
        coroutine.Index = Coroutines.Add(coroutine);
        return coroutine;
    }

    public static void StopCoroutine(LSCoroutine coroutine)
    {
        if (coroutine.Active == false)
            GravitasLogger.Channel.Error($"Coroutine already stopped");

        Coroutines?.TryRemoveAt(coroutine.Index);
        coroutine.Active = false;
        coroutine.End();
    }

    public static void Deactivate()
    {
        Coroutines?.Clear();
    }
}