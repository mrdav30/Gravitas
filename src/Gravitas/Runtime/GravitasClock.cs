//=======================================================================
// GravitasClock.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler.Timing;
using FixedMathSharp;
using FixedMathSharp.Chronicler;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// Stores deterministic fixed-step timing state for one Gravitas runtime owner.
/// </summary>
internal sealed class GravitasClock
{
    private readonly ChronicleClock _timeline = new(FixedChronicleTime.FromFixed64(
        Fixed64.One / (Fixed64)PhysicsSettings.DefaultFrameRate));

    internal object Lifetime { get; private set; } = new();

    private int _frameRate = PhysicsSettings.DefaultFrameRate;

    private Fixed64 _deltaTime = Fixed64.One / (Fixed64)PhysicsSettings.DefaultFrameRate;

    private Fixed64 _invDeltaTime = Fixed64.One / (Fixed64.One / (Fixed64)PhysicsSettings.DefaultFrameRate);

    /// <summary>
    /// Gets the fixed simulation frame rate.
    /// </summary>
    public int FrameRate => _frameRate;

    /// <summary>
    /// Gets the fixed time step for each simulation frame.
    /// </summary>
    public Fixed64 DeltaTime => _deltaTime;

    /// <summary>
    /// Gets the reciprocal of the current fixed time step.
    /// </summary>
    public Fixed64 InvDeltaTime => _invDeltaTime;

    /// <summary>
    /// Gets the number of simulated frames.
    /// </summary>
    public long FrameCount => _timeline.FrameCount;

    /// <summary>
    /// Gets the total simulated time in seconds.
    /// </summary>
    public ChronicleTimestamp ElapsedTime => _timeline.ElapsedTime;

    internal long GetDeadlineFrame(long framesFromNow) => _timeline.GetDeadlineFrame(framesFromNow);

    /// <summary>
    /// Gets the accumulated visualization time since the last late-simulate reset.
    /// </summary>
    public Fixed64 AccumulatedTime { get; private set; }

    /// <summary>
    /// Gets whether the next visualization step should reset accumulated time.
    /// </summary>
    public bool ResetAccumulation { get; private set; }

    /// <summary>
    /// Gets whether the current visualization step started from a reset.
    /// </summary>
    public bool ResetAccumulationThisVisualize { get; private set; }

    /// <summary>
    /// Gets the accumulated visualization time expressed in simulation frames.
    /// </summary>
    public Fixed64 ExpectedAccumulation { get; private set; }

    /// <summary>
    /// Advances the fixed simulation frame.
    /// </summary>
    public void Simulate()
    {
        _timeline.Advance();
    }

    /// <summary>
    /// Marks visualization accumulation for reset after a fixed simulation frame completes.
    /// </summary>
    public void LateSimulate()
    {
        ResetAccumulation = true;
    }

    /// <summary>
    /// Advances deterministic visualization accumulation by one fixed step.
    /// </summary>
    public void Visualize()
    {
        ResetAccumulationThisVisualize = ResetAccumulation;
        if (ResetAccumulation)
        {
            AccumulatedTime = Fixed64.Zero;
            ResetAccumulation = false;
        }

        AccumulatedTime += _deltaTime;
        ExpectedAccumulation = FixedMath.Clamp01(AccumulatedTime / _deltaTime);
    }

    /// <summary>
    /// Resets elapsed timing state while preserving the configured frame rate.
    /// </summary>
    public void Reset()
    {
        Lifetime = new object();
        _timeline.Reset();
        AccumulatedTime = Fixed64.Zero;
        ExpectedAccumulation = Fixed64.Zero;
        ResetAccumulation = false;
        ResetAccumulationThisVisualize = false;
    }

    /// <summary>
    /// Updates the fixed simulation frame rate.
    /// </summary>
    /// <param name="frameRate">The new frame rate. Must be within the supported physics settings range.</param>
    public void SetFrameRate(int frameRate)
    {
        PhysicsSettings.ThrowIfInvalidFrameRate(frameRate);
        _frameRate = frameRate;
        _deltaTime = Fixed64.One / (Fixed64)_frameRate;
        _invDeltaTime = Fixed64.One / _deltaTime;
        _timeline.SetStepDuration(FixedChronicleTime.FromFixed64(_deltaTime));
    }

    /// <summary>
    /// Counts complete steps at the current step size, not historical frame indices.
    /// </summary>
    /// <param name="duration">The nonnegative duration in seconds.</param>
    /// <returns>The number of complete steps in the duration.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public long GetFrameCountForDuration(Fixed64 duration)
    {
        return FixedChronicleTime.GetFrameCountForDuration(duration, _deltaTime);
    }
}
