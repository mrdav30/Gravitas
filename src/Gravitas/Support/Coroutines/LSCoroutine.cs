//=======================================================================
// LSCoroutine.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Collections.Generic;
using System.Runtime.ExceptionServices;

namespace Gravitas.Support;

/// <summary>
/// Represents one context-owned lockstep coroutine.
/// </summary>
public sealed class LSCoroutine
{
    private readonly IEnumerator<ILockedYieldInstruction> _enumerator;
    private ILockedYieldInstruction? _currentInstruction;
    // Cancellation marks Active immediately, but disposal waits until the current user callback unwinds.
    private bool _executing;

    internal LSCoroutine(GravitasCoroutineService owner, IEnumerator<ILockedYieldInstruction> enumerator)
    {
        Owner = owner;
        _enumerator = enumerator;
    }

    /// <summary>
    /// Gets whether this coroutine is still active.
    /// </summary>
    public bool Active { get; private set; } = true;

    internal GravitasCoroutineService Owner { get; }

    internal int Index { get; set; } = -1;

    internal void Simulate()
    {
        Exception? executionException = null;
        _executing = true;
        try
        {
            SimulateCore();
        }
        catch (Exception exception)
        {
            executionException = exception;
        }
        finally
        {
            _executing = false;
            if (!Active)
            {
                try
                {
                    DisposeResources();
                }
                catch (Exception cleanupException)
                {
                    if (executionException != null)
                        throw new AggregateException(executionException, cleanupException);

                    throw;
                }
            }
        }

        if (executionException != null)
            ExceptionDispatchInfo.Capture(executionException).Throw();
    }

    private void SimulateCore()
    {
        if (_currentInstruction != null)
        {
            bool keepWaiting = _currentInstruction.KeepWaiting;
            if (!Active || keepWaiting)
                return;

            DisposeCurrentInstruction();
            if (!Active)
                return;
        }

        if (!_enumerator.MoveNext())
        {
            Owner.StopCoroutine(this);
            return;
        }

        ILockedYieldInstruction? nextInstruction = _enumerator.Current;
        _currentInstruction = nextInstruction;
        if (!Active)
            return;

        if (_currentInstruction != null
            && !ReferenceEquals(_currentInstruction.Context, Owner.Context))
        {
            throw new InvalidOperationException(
                "Coroutine yield instructions must belong to the coroutine service context.");
        }
    }

    internal void End()
    {
        Active = false;
        if (!_executing)
            DisposeResources();
    }

    private void DisposeResources()
    {
        ILockedYieldInstruction? instruction = _currentInstruction;
        _currentInstruction = null;

        if (ReferenceEquals(instruction, _enumerator))
        {
            _enumerator.Dispose();
            return;
        }

        Exception? instructionException = null;
        try
        {
            instruction?.Dispose();
        }
        catch (Exception exception)
        {
            instructionException = exception;
        }

        try
        {
            _enumerator.Dispose();
        }
        catch (Exception enumeratorException)
        {
            if (instructionException != null)
                throw new AggregateException(instructionException, enumeratorException);

            throw;
        }

        if (instructionException != null)
            ExceptionDispatchInfo.Capture(instructionException).Throw();
    }

    private void DisposeCurrentInstruction()
    {
        ILockedYieldInstruction instruction = _currentInstruction!;
        _currentInstruction = null;
        if (!ReferenceEquals(instruction, _enumerator))
            instruction.Dispose();
    }
}
