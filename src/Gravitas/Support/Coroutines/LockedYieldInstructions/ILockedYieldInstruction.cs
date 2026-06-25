//=======================================================================
// ILockedYieldInstruction.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Collections;

namespace Gravitas.Support;

/// <summary>
/// A coroutine yield instruction that is locked to a GravitasWorldContext and can be used in a deterministic simulation.
/// </summary>
public interface ILockedYieldInstruction : IEnumerator, IDisposable
{
    /// <summary>
    /// Indicates if coroutine should be kept suspended.
    /// </summary>
    bool KeepWaiting { get; }
}