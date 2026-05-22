using System;
using System.Collections;

namespace Gravitas.Support;

/// <summary>
/// Repurposed from Unity's CustomYieldInstruction
/// </summary>
public interface ILockedYieldInstruction : IEnumerator, IDisposable
{
    /// <summary>
    /// Indicates if coroutine should be kept suspended.
    /// </summary>
    bool KeepWaiting { get; }
}