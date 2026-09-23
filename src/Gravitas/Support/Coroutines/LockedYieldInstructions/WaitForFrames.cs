//=======================================================================
// WaitForFrames.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Support;

/// <summary>
/// A coroutine yield instruction that waits for a specified number of Gravitas simulation frames.
/// </summary>
/// <remarks>
/// Uses a checked deadline in the context's current clock lifetime. Reading an
/// instruction after context reset or disposal throws; reads never consume frames.
/// </remarks>
public readonly struct WaitForFrames : ILockedYieldInstruction
{
    private readonly GravitasWorldContext _context;
    private readonly object _lifetime;
    private readonly long _deadlineFrame;

    /// <summary>
    /// Creates an instruction that waits for a nonnegative number of simulation frames.
    /// </summary>
    public WaitForFrames(GravitasWorldContext context, long numberOfFrames)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        SwiftThrowHelper.ThrowIfArgumentOutOfRange(numberOfFrames < 0, null,
            nameof(numberOfFrames), "Wait frame count cannot be negative.");

        _context = context;
        _lifetime = context.CaptureClockLifetime();
        _deadlineFrame = context.GetDeadlineFrame(numberOfFrames);
    }

    /// <inheritdoc />
    public GravitasWorldContext Context => _context;

    /// <inheritdoc />
    public bool KeepWaiting
    {
        get
        {
            _context.ValidateClockLifetime(_lifetime);
            return _context.FrameCount < _deadlineFrame;
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
