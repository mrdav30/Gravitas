//=======================================================================
// WaitForRealSeconds.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.Support;

/// <summary>
/// A coroutine yield instruction that waits for a specified number of real-time seconds, using the GravitasWorldContext's delta time for accumulation.
/// </summary>
public struct WaitForRealSeconds : ILockedYieldInstruction
{
    private readonly GravitasWorldContext _context;
    private Fixed64 _accumulator;
    private Fixed64 _waitTime;

    public WaitForRealSeconds(GravitasWorldContext context, Fixed64 seconds)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));

        _context = context;
        _accumulator = Fixed64.Zero;
        _waitTime = seconds;
    }

    public bool KeepWaiting
    {
        get
        {
            _accumulator += _context.DeltaTime;
            return _accumulator < _waitTime;
        }
    }

    public object? Current => null;

    public bool MoveNext() => KeepWaiting;

    public void Reset() { }

    public void Dispose() { }
}
