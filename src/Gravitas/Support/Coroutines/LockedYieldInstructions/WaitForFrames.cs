using SwiftCollections;

namespace Gravitas.Support;

/// <summary>
/// src - https://forum.unity.com/threads/coroutine-wait-x-frames-not-seconds.550168/
/// </summary>
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
