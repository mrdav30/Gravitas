using Gravitas.Support;
using SwiftCollections;
using System;

namespace Gravitas;

/// <summary>
/// Owns ordered lifecycle hook storage for one Gravitas runtime owner.
/// </summary>
internal sealed class GravitasLifecycleHooks
{
    private readonly LifecycleHookHandler _hookHandler = new();

    private readonly SwiftList<OrderedLifecycleHook> _simulateHooks = new();

    private readonly SwiftList<OrderedLifecycleHook> _lateSimulateHooks = new();

    private readonly SwiftList<OrderedLifecycleHook> _visualizeHooks = new();

    private readonly SwiftList<OrderedLifecycleHook> _lateVisualizeHooks = new();

    private readonly SwiftList<OrderedLifecycleHook> _resetHooks = new();

    private readonly SwiftList<OrderedLifecycleHook> _frameRateChangedHooks = new();

    internal IDisposable RegisterOnSimulate(string owner, int order, Action callback) =>
        _hookHandler.RegisterHook(_simulateHooks, owner, order, callback);

    internal IDisposable RegisterOnLateSimulate(string owner, int order, Action callback) =>
        _hookHandler.RegisterHook(_lateSimulateHooks, owner, order, callback);

    internal IDisposable RegisterOnVisualize(string owner, int order, Action callback) =>
        _hookHandler.RegisterHook(_visualizeHooks, owner, order, callback);

    internal IDisposable RegisterOnLateVisualize(string owner, int order, Action callback) =>
        _hookHandler.RegisterHook(_lateVisualizeHooks, owner, order, callback);

    internal IDisposable RegisterOnReset(string owner, int order, Action callback) =>
        _hookHandler.RegisterHook(_resetHooks, owner, order, callback);

    internal IDisposable RegisterOnFrameRateChanged(string owner, int order, Action callback) =>
        _hookHandler.RegisterHook(_frameRateChangedHooks, owner, order, callback);

    internal void InvokeSimulate()
    {
        if (_simulateHooks.Count != 0)
            _hookHandler.InvokeHooks(_simulateHooks);
    }

    internal void InvokeLateSimulate()
    {
        if (_lateSimulateHooks.Count != 0)
            _hookHandler.InvokeHooks(_lateSimulateHooks);
    }

    internal void InvokeVisualize()
    {
        if (_visualizeHooks.Count != 0)
            _hookHandler.InvokeHooks(_visualizeHooks);
    }

    internal void InvokeLateVisualize()
    {
        if (_lateVisualizeHooks.Count != 0)
            _hookHandler.InvokeHooks(_lateVisualizeHooks);
    }

    internal void InvokeReset()
    {
        if (_resetHooks.Count != 0)
            _hookHandler.InvokeHooks(_resetHooks);
    }

    internal void InvokeFrameRateChanged()
    {
        if (_frameRateChangedHooks.Count != 0)
            _hookHandler.InvokeHooks(_frameRateChangedHooks);
    }
}
