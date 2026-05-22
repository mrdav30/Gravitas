namespace Gravitas.Support;

public class WaitForNextSimulate : ILockedYieldInstruction
{
    private readonly int _checkedInFrameCount;

    public WaitForNextSimulate() =>
        _checkedInFrameCount = PhysicsManager.FrameCount;

    public bool KeepWaiting =>
         PhysicsManager.FrameCount <= _checkedInFrameCount;

    public object? Current => null;

    public bool MoveNext() => KeepWaiting;

    public void Reset() { }

    public void Dispose() { }
}
