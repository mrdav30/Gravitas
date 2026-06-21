//=======================================================================
// WaitForNextSimulate.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Support;

public readonly struct WaitForNextSimulate : ILockedYieldInstruction
{
    private readonly GravitasWorldContext _context;
    private readonly int _checkedInFrameCount;

    public WaitForNextSimulate(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));

        _context = context;
        _checkedInFrameCount = context.FrameCount;
    }

    public bool KeepWaiting =>
         _context.FrameCount <= _checkedInFrameCount;

    public object? Current => null;

    public bool MoveNext() => KeepWaiting;

    public void Reset() { }

    public void Dispose() { }
}
