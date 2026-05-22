using FixedMathSharp;
using SwiftCollections;

namespace Gravitas.Support;

/// <summary>
/// src - https://stackoverflow.com/questions/30056471/how-to-make-the-script-wait-sleep-in-a-simple-way-in-unity
/// </summary>
public struct WaitForRealSeconds : ILockedYieldInstruction
{
    private readonly GravitasWorldContext _context;
    private Fixed64 _accumulator;
    private Fixed64 _waitTime;

    public WaitForRealSeconds(GravitasWorldContext context, Fixed64 seconds)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));

        _context = context;
        _accumulator = Fixed64.Zero;
        _waitTime = seconds;
    }

    public bool KeepWaiting
    {
        get
        {
            _accumulator += _context.DeltaTime;
            return _accumulator < _waitTime;
        }
    }

    public object? Current => null;

    public bool MoveNext() => KeepWaiting;

    public void Reset() { }

    public void Dispose() { }
}
