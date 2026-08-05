//=======================================================================
// WaitForRealSeconds.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.Support;

/// <summary>
/// A coroutine yield instruction that waits for a specified duration on the owning context's deterministic clock.
/// </summary>
public readonly struct WaitForRealSeconds : ILockedYieldInstruction
{
    private readonly GravitasWorldContext _context;
    private readonly Fixed64 _targetTime;

    /// <summary>
    /// Creates an instruction that waits for a nonnegative duration on the context's deterministic clock.
    /// </summary>
    public WaitForRealSeconds(GravitasWorldContext context, Fixed64 seconds)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        SwiftThrowHelper.ThrowIfArgument(seconds < Fixed64.Zero, nameof(seconds), "Wait duration cannot be negative.");

        _context = context;
        _targetTime = context.TotalTime + seconds;
    }

    /// <inheritdoc />
    public GravitasWorldContext Context => _context;

    /// <inheritdoc />
    public bool KeepWaiting => _context.TotalTime < _targetTime;

    /// <inheritdoc />
    public object? Current => null;

    /// <inheritdoc />
    public bool MoveNext() => KeepWaiting;

    /// <inheritdoc />
    public void Reset() { }

    /// <inheritdoc />
    public void Dispose() { }
}
