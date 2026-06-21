//=======================================================================
// WaitForFrames.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Support;

public readonly struct WaitForFrames : ILockedYieldInstruction
{
    private readonly GravitasWorldContext _context;
    private readonly int _targetFrameCount;

    public WaitForFrames(GravitasWorldContext context, int numberOfFrames)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        SwiftThrowHelper.ThrowIfNegative(numberOfFrames, nameof(numberOfFrames));

        _context = context;
        _targetFrameCount = context.FrameCount + numberOfFrames;
    }

    public bool KeepWaiting =>
        _context.FrameCount < _targetFrameCount;

    public object? Current => null;

    public bool MoveNext() => KeepWaiting;

    public void Reset() { }

    public void Dispose() { }
}
