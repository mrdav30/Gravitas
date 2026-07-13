//=======================================================================
// GravitasCoroutineService.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using SwiftCollections;
using System;
using System.Collections.Generic;
using System.Runtime.ExceptionServices;

namespace Gravitas.Support;

/// <summary>
/// Owns lockstep coroutine state for one <see cref="GravitasWorldContext"/>.
/// </summary>
public sealed class GravitasCoroutineService
{
    private readonly GravitasWorldContext _context;
    private readonly SwiftBucket<LSCoroutine> _coroutines = new();
    private readonly SwiftList<LSCoroutine> _simulationSnapshot = new();
    private bool _simulating;
    private bool _resetting;
    private bool _deactivated;

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
    /// Clears context-local coroutine state and reactivates a manually deactivated service.
    /// </summary>
    /// <exception cref="InvalidOperationException">
    /// The service is already resetting or its context has been disposed.
    /// </exception>
    public void Initialize()
    {
        SwiftThrowHelper.ThrowIfTrue(
            _resetting || _context.IsDisposed,
            nameof(GravitasCoroutineService),
            "Coroutine service cannot initialize while resetting or after its context is disposed.");

        Reset();
        _deactivated = false;
    }

    /// <summary>
    /// Advances active coroutines once for the current simulation frame.
    /// </summary>
    public void Simulate()
    {
        if (_simulating || _resetting || _deactivated)
            return;

        int peak = _coroutines.PeakCount;
        if (peak == 0)
            return;

        _simulating = true;
        try
        {
            _simulationSnapshot.EnsureCapacity(_coroutines.Count);
            for (int i = 0; i < peak; i++)
            {
                if (_coroutines.TryGetValue(i, out LSCoroutine coroutine))
                    _simulationSnapshot.Add(coroutine);
            }

            for (int i = 0; i < _simulationSnapshot.Count; i++)
            {
                LSCoroutine coroutine = _simulationSnapshot[i];
                if (!coroutine.Active)
                    continue;

                try
                {
                    coroutine.Simulate();
                }
                catch (Exception simulationException)
                {
                    try
                    {
                        StopCoroutine(coroutine);
                    }
                    catch (Exception cleanupException)
                    {
                        throw new AggregateException(simulationException, cleanupException);
                    }

                    throw;
                }
            }
        }
        finally
        {
            _simulationSnapshot.Clear();
            _simulating = false;
        }
    }

    /// <summary>
    /// Starts a context-local coroutine.
    /// </summary>
    /// <param name="enumerator">The lockstep yield instruction enumerator to run.</param>
    /// <returns>The started coroutine handle.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="enumerator"/> is <see langword="null"/>.</exception>
    /// <exception cref="InvalidOperationException">
    /// The service is resetting or deactivated, or its context has been disposed.
    /// </exception>
    public LSCoroutine StartCoroutine(IEnumerator<ILockedYieldInstruction> enumerator)
    {
        SwiftThrowHelper.ThrowIfNull(enumerator, nameof(enumerator));
        SwiftThrowHelper.ThrowIfTrue(
            _resetting || _deactivated || _context.IsDisposed,
            nameof(GravitasCoroutineService),
            "Coroutine service cannot start work while resetting, deactivated, or disposed.");

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

        // Clear while the final slot is still live so SwiftBucket also resets its peak/free-slot state.
        if (_coroutines.Count == 1)
            _coroutines.Clear();
        else
            _coroutines.TryRemoveAt(coroutine.Index);

        coroutine.End();
    }

    /// <summary>
    /// Stops all active coroutines and clears service state.
    /// </summary>
    public void Reset()
    {
        if (_resetting)
            return;

        _resetting = true;
        Exception? firstException = null;
        int peak = _coroutines.PeakCount;
        try
        {
            for (int i = 0; i < peak; i++)
            {
                if (!_coroutines.TryGetValue(i, out LSCoroutine coroutine))
                    continue;

                // End marks the handle inactive before callbacks. Retaining its slot ensures the
                // final Clear resets SwiftBucket high-water state even if callbacks stop later handles.
                try
                {
                    coroutine.End();
                }
                catch (Exception exception)
                {
                    firstException ??= exception;
                }
            }
        }
        finally
        {
            _coroutines.Clear();
            _simulationSnapshot.Clear();
            _resetting = false;
        }

        if (firstException != null)
            ExceptionDispatchInfo.Capture(firstException).Throw();
    }

    /// <summary>
    /// Deactivates this coroutine service, disposes all active coroutine state, and rejects new work.
    /// </summary>
    /// <remarks>
    /// A manually deactivated service can be reactivated by <see cref="Initialize"/> while its context remains active.
    /// </remarks>
    public void Deactivate()
    {
        if (_deactivated)
            return;

        _deactivated = true;
        Reset();
    }

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
