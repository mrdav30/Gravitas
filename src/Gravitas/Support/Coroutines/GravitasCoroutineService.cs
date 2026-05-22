using FixedMathSharp;
using SwiftCollections;
using System.Collections.Generic;

namespace Gravitas.Support;

/// <summary>
/// Owns lockstep coroutine state for one <see cref="GravitasWorldContext"/>.
/// </summary>
public sealed class GravitasCoroutineService
{
    private readonly GravitasWorldContext _context;
    private readonly SwiftBucket<LSCoroutine> _coroutines = new();

    /// <summary>
    /// Initializes a new coroutine service for the supplied context.
    /// </summary>
    /// <param name="context">The owning world context.</param>
    public GravitasCoroutineService(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
    }

    /// <summary>
    /// Gets the owning world context.
    /// </summary>
    public GravitasWorldContext Context => _context;

    /// <summary>
    /// Gets the number of active coroutines owned by this context.
    /// </summary>
    public int ActiveCoroutineCount => _coroutines.Count;

    /// <summary>
    /// Clears context-local coroutine state.
    /// </summary>
    public void Initialize() => Reset();

    /// <summary>
    /// Advances active coroutines once for the current simulation frame.
    /// </summary>
    public void Simulate()
    {
        if (_coroutines.PeakCount == 0)
            return;

        for (int i = 0; i < _coroutines.PeakCount; i++)
        {
            if (!_coroutines.IsAllocated(i))
                continue;

            LSCoroutine coroutine = _coroutines[i];
            if (coroutine.Active)
                coroutine.Simulate();
        }
    }

    /// <summary>
    /// Starts a context-local coroutine.
    /// </summary>
    /// <param name="enumerator">The lockstep yield instruction enumerator to run.</param>
    /// <returns>The started coroutine handle.</returns>
    public LSCoroutine StartCoroutine(IEnumerator<ILockedYieldInstruction> enumerator)
    {
        SwiftThrowHelper.ThrowIfNull(enumerator, nameof(enumerator));

        LSCoroutine coroutine = new(this, enumerator);
        coroutine.Index = _coroutines.Add(coroutine);
        return coroutine;
    }

    /// <summary>
    /// Stops a context-local coroutine.
    /// </summary>
    /// <param name="coroutine">The coroutine to stop.</param>
    public void StopCoroutine(LSCoroutine coroutine)
    {
        SwiftThrowHelper.ThrowIfNull(coroutine, nameof(coroutine));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(coroutine.Owner, this),
            nameof(coroutine),
            "Coroutine must be stopped through its owning coroutine service.");

        if (!coroutine.Active)
            return;

        int index = coroutine.Index;
        if (index >= 0
            && index < _coroutines.PeakCount
            && _coroutines.IsAllocated(index)
            && ReferenceEquals(_coroutines[index], coroutine))
        {
            _coroutines.TryRemoveAt(index);
        }

        coroutine.End();
    }

    /// <summary>
    /// Stops all active coroutines and clears service state.
    /// </summary>
    public void Reset()
    {
        for (int i = 0; i < _coroutines.PeakCount; i++)
        {
            if (_coroutines.IsAllocated(i))
                _coroutines[i].End();
        }

        _coroutines.Clear();
    }

    /// <summary>
    /// Deactivates this coroutine service and clears all active coroutine state.
    /// </summary>
    public void Deactivate() => Reset();

    /// <summary>
    /// Creates a frame-count wait instruction bound to this service's context.
    /// </summary>
    public WaitForFrames WaitForFrames(int frames) => new(_context, frames);

    /// <summary>
    /// Creates a next-simulation-frame wait instruction bound to this service's context.
    /// </summary>
    public WaitForNextSimulate WaitForNextSimulate() => new(_context);

    /// <summary>
    /// Creates a fixed-duration wait instruction bound to this service's context.
    /// </summary>
    public WaitForRealSeconds WaitForRealSeconds(Fixed64 seconds) => new(_context, seconds);
}
