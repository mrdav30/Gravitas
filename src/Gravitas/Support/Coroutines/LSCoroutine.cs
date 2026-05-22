using System.Collections.Generic;

namespace Gravitas.Support;

public struct LSCoroutine
{
    public IEnumerator<ILockedYieldInstruction> Enumerator;
    public bool Active = true;
    public int Index;

    public LSCoroutine(IEnumerator<ILockedYieldInstruction> enumerator)
    {
        Enumerator = enumerator;
        Active = true;
    }

    public void Simulate()
    {
        if (Enumerator.Current != null && Enumerator.Current.KeepWaiting)
            return;

        if (Enumerator.MoveNext())
            return;
        else
            CoroutineManager.StopCoroutine(this);
    }
    public void End()
    {
        Active = false;
        Enumerator.Dispose();
    }
}