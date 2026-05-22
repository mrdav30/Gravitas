using FixedMathSharp;

namespace Gravitas.Support;

/// <summary>
/// src - https://stackoverflow.com/questions/30056471/how-to-make-the-script-wait-sleep-in-a-simple-way-in-unity
/// </summary>
public struct WaitForRealSeconds : ILockedYieldInstruction
{
    private Fixed64 _accumulator;
    private Fixed64 _waitTime;

    public WaitForRealSeconds(Fixed64 seconds)
    {
        _accumulator = Fixed64.Zero;
        _waitTime = seconds;
    }

    public bool KeepWaiting
    {
        get
        {
            _accumulator += PhysicsManager.DeltaTime;
            return _accumulator < _waitTime;
        }
    }

    public object? Current => null;

    public bool MoveNext() => KeepWaiting;

    public void Reset() { }

    public void Dispose() { }
}
