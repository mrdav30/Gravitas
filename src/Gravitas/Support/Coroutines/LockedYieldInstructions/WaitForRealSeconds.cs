//=======================================================================
// WaitForRealSeconds.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler.Timing;
using FixedMathSharp;
using FixedMathSharp.Chronicler;

namespace Gravitas.Support;

/// <summary>
/// A coroutine yield instruction that waits for a specified duration on the owning context's deterministic clock.
/// </summary>
/// <remarks>
/// Uses a wide timestamp deadline, preserving the requested duration across frame-rate
/// changes. Reading an instruction after context reset or disposal throws; reads never
/// consume time. An unrepresentable deadline throws during construction.
/// </remarks>
public readonly struct WaitForRealSeconds : ILockedYieldInstruction
{
    private readonly GravitasWorldContext _context;
    private readonly object _lifetime;
    private readonly ChronicleTimestamp _targetTime;

    /// <summary>
    /// Creates an instruction that waits for a nonnegative duration on the context's deterministic clock.
    /// </summary>
    public WaitForRealSeconds(GravitasWorldContext context, Fixed64 seconds)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        SwiftThrowHelper.ThrowIfArgument(seconds < Fixed64.Zero, nameof(seconds), "Wait duration cannot be negative.");

        _context = context;
        _lifetime = context.CaptureClockLifetime();
        _targetTime = context.ElapsedTime + FixedChronicleTime.FromFixed64(seconds);
    }

    /// <inheritdoc />
    public GravitasWorldContext Context => _context;

    /// <inheritdoc />
    public bool KeepWaiting
    {
        get
        {
            _context.ValidateClockLifetime(_lifetime);
            return _context.ElapsedTime < _targetTime;
        }
    }

    /// <inheritdoc />
    public object? Current => null;

    /// <inheritdoc />
    public bool MoveNext() => KeepWaiting;

    /// <inheritdoc />
    public void Reset() { }

    /// <inheritdoc />
    public void Dispose() { }
}
