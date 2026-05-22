namespace Gravitas.Support;

/// <summary>
/// src - https://forum.unity.com/threads/coroutine-wait-x-frames-not-seconds.550168/
/// </summary>
public class WaitForFrames : ILockedYieldInstruction
{
    private readonly int _targetFrameCount;

    public WaitForFrames(int numberOfFrames) =>
        _targetFrameCount = PhysicsManager.FrameCount + numberOfFrames;

    public bool KeepWaiting =>
        PhysicsManager.FrameCount < _targetFrameCount;

    public object? Current => null;

    public bool MoveNext() => KeepWaiting;

    public void Reset() { }

    public void Dispose() { }
}
