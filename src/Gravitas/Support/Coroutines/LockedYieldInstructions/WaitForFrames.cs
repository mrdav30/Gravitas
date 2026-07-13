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
public readonly struct WaitForFrames : ILockedYieldInstruction
{
    private readonly GravitasWorldContext _context;
    private readonly int _startFrameCount;
    private readonly uint _numberOfFrames;

    public WaitForFrames(GravitasWorldContext context, int numberOfFrames)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        SwiftThrowHelper.ThrowIfNegative(numberOfFrames, nameof(numberOfFrames));

        _context = context;
        _startFrameCount = context.FrameCount;
        _numberOfFrames = (uint)numberOfFrames;
    }

    /// <inheritdoc />
    public GravitasWorldContext Context => _context;

    public bool KeepWaiting =>
        unchecked((uint)(_context.FrameCount - _startFrameCount)) < _numberOfFrames;

    public object? Current => null;

    public bool MoveNext() => KeepWaiting;

    public void Reset() { }

    public void Dispose() { }
}
