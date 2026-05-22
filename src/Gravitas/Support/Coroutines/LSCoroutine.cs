using System.Collections.Generic;

namespace Gravitas.Support;

/// <summary>
/// Represents one context-owned lockstep coroutine.
/// </summary>
public sealed class LSCoroutine
{
    private readonly IEnumerator<ILockedYieldInstruction> _enumerator;

    internal LSCoroutine(GravitasCoroutineService owner, IEnumerator<ILockedYieldInstruction> enumerator)
    {
        Owner = owner;
        _enumerator = enumerator;
    }

    /// <summary>
    /// Gets whether this coroutine is still active.
    /// </summary>
    public bool Active { get; private set; } = true;

    internal GravitasCoroutineService Owner { get; }

    internal int Index { get; set; } = -1;

    internal void Simulate()
    {
        if (_enumerator.Current != null && _enumerator.Current.KeepWaiting)
            return;

        if (_enumerator.MoveNext())
            return;

        Owner.StopCoroutine(this);
    }

    internal void End()
    {
        if (!Active)
            return;

        Active = false;
        _enumerator.Dispose();
    }
}
