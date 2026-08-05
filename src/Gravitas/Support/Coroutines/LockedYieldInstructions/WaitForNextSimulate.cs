//=======================================================================
// WaitForNextSimulate.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Support;

/// <summary>
/// A coroutine yield instruction that waits until the next Gravitas simulation frame.
/// </summary>
public readonly struct WaitForNextSimulate : ILockedYieldInstruction
{
    private readonly GravitasWorldContext _context;
    private readonly int _checkedInFrameCount;

    /// <summary>
    /// Creates an instruction that waits until the context advances to another simulation frame.
    /// </summary>
    public WaitForNextSimulate(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));

        _context = context;
        _checkedInFrameCount = context.FrameCount;
    }

    /// <inheritdoc />
    public GravitasWorldContext Context => _context;

    /// <inheritdoc />
    public bool KeepWaiting =>
         _context.FrameCount == _checkedInFrameCount;

    /// <inheritdoc />
    public object? Current => null;

    /// <inheritdoc />
    public bool MoveNext() => KeepWaiting;

    /// <inheritdoc />
    public void Reset() { }

    /// <inheritdoc />
    public void Dispose() { }
}
