using Chronicler.Timing;
using FixedMathSharp;
using Gravitas.Support;
using Gravitas.Diagnostics;
using Gravitas.Tests.Support;
using System;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Runtime;

public sealed class GravitasWideClockTests
{
    [Fact]
    public void WideTimingAndConcreteWaitsStayAllocationFreeAfterWarmup()
    {
        using var context = GravitasWorldContext.CreateOwned();
        TimingTestUtility.SetClock(context, (long)int.MaxValue + 17, new ChronicleTimestamp(3155760000L, 0));
        var frames = context.Coroutines.WaitForFrames(10000);
        var seconds = context.Coroutines.WaitForRealSeconds((Fixed64)10000);
        long allocated = AllocationTestHelper.MeasureSteadyState(() =>
        {
            context.Simulate();
            context.LateSimulate();
            _ = frames.KeepWaiting;
            _ = seconds.KeepWaiting;
            _ = new WaitForNextSimulate(context).KeepWaiting;
            _ = context.Coroutines.WaitForFrames(2).KeepWaiting;
            _ = context.Coroutines.WaitForRealSeconds(Fixed64.One).KeepWaiting;
            _ = context.GetFrameCountForDuration(Fixed64.One);
        });
        Assert.Equal(0, allocated);
        Assert.True(frames.KeepWaiting);
        Assert.True(seconds.KeepWaiting);
    }

    [Fact]
    public void SecondsDeadlineRejectsOverflowButZeroWaitStillCompletes()
    {
        using var context = GravitasWorldContext.CreateOwned();
        TimingTestUtility.SetClock(context, 1, new ChronicleTimestamp(long.MaxValue, uint.MaxValue));
        Assert.False(context.Coroutines.WaitForRealSeconds(Fixed64.Zero).KeepWaiting);
        var before = context.ComputeReplayHash();
        Assert.Throws<OverflowException>(() => context.Coroutines.WaitForRealSeconds(Fixed64.FromRaw(1)));
        Assert.Equal(before, context.ComputeReplayHash());
    }

    [Fact]
    public void RetainedWaitsRejectDisposedContext()
    {
        var context = GravitasWorldContext.CreateOwned();
        var frames = context.Coroutines.WaitForFrames(2);
        var seconds = context.Coroutines.WaitForRealSeconds(Fixed64.One);
        var next = context.Coroutines.WaitForNextSimulate();
        context.Dispose();
        Assert.Throws<ObjectDisposedException>(() => frames.KeepWaiting);
        Assert.Throws<ObjectDisposedException>(() => seconds.KeepWaiting);
        Assert.Throws<ObjectDisposedException>(() => next.KeepWaiting);
    }

    [Fact]
    public void UnsetGroundingStampNearFrameExhaustionDoesNotSuppressProbe()
    {
        using var scenario = PhysicsScenarioBuilder.Create();
        TimingTestUtility.SetClock(scenario.Context, long.MaxValue, new ChronicleTimestamp(1, 0));
        var body = scenario.CreateSphere(Vector3d.Zero).Body;
        body.GravityScale = Fixed64.Zero;
        body.RecordData(new InvalidRecordPayloadChronicler(new Dictionary<string, object>
        {
            ["LastGroundCheckFrame"] = -1L
        }));
        scenario.Context.Diagnostics.Enable();
        body.LateSimulate();
        Assert.Contains(scenario.Context.Diagnostics.Events.ToArray(),
            item => item.Kind == GravitasDiagnosticEventKind.GroundProbe);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ExhaustedClockRejectsBeforeWorldMutation(bool timestampExhausted)
    {
        using var scenario = PhysicsScenarioBuilder.Create();
        var context = scenario.Context;
        scenario.CreateSphere(Vector3d.Zero).Body.AddForce(Vector3d.Right);
        TimingTestUtility.SetClock(context,
            timestampExhausted ? 17L : long.MaxValue,
            timestampExhausted ? new ChronicleTimestamp(long.MaxValue, uint.MaxValue) : new ChronicleTimestamp(1, 0));
        int callbacks = 0;
        using var hook = context.RegisterOnSimulate("test", 0, () => callbacks++);
        var before = context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches);
        if (timestampExhausted)
            Assert.Throws<OverflowException>(context.Simulate);
        else
            Assert.Throws<InvalidOperationException>(context.Simulate);
        Assert.Equal(before, context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches));
        Assert.Equal(0, callbacks);
        context.ThrowIfFixedStepMutationNotAllowed();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ExhaustedLateTokenRejectsBeforeWorldMutation(bool latePhase)
    {
        using var context = GravitasWorldContext.CreateOwned();
        TimingTestUtility.SetLateToken(context, long.MaxValue);
        var before = context.ComputeReplayHash();
        Assert.Throws<InvalidOperationException>(latePhase ? context.LateSimulate : context.Simulate);
        Assert.Equal(before, context.ComputeReplayHash());
        Assert.False(context.ResetAccumulation);
        context.ThrowIfFixedStepMutationNotAllowed();
    }

    [Fact]
    public void SecondsWaitPreservesElapsedDurationAcrossStepChangeAndLargeOrigin()
    {
        using var context = GravitasWorldContext.CreateOwned();
        var origin = new ChronicleTimestamp(3155760000L, 0);
        TimingTestUtility.SetClock(context, 123, origin);
        context.SetFrameRate(4);
        ILockedYieldInstruction wait = context.Coroutines.WaitForRealSeconds(Fixed64.One);
        context.Simulate();
        Assert.True(wait.KeepWaiting);
        Assert.True(wait.KeepWaiting);
        context.SetFrameRate(2);
        context.Simulate();
        Assert.True(wait.KeepWaiting);
        context.SetFrameRate(4);
        context.Simulate();
        Assert.False(wait.KeepWaiting);
        Assert.Equal(origin + new ChronicleDuration(1, 0), context.ElapsedTime);
    }

    [Fact]
    public void DurationConversionCountsCompleteRawStepsWithoutReciprocalRounding()
    {
        using var context = GravitasWorldContext.CreateOwned();
        context.SetFrameRate(60);
        Assert.Equal(60L, context.GetFrameCountForDuration(Fixed64.One));
        Assert.Equal(1L, context.GetFrameCountForDuration(context.DeltaTime));
        Assert.Equal(0L, context.GetFrameCountForDuration(context.DeltaTime - Fixed64.FromRaw(1)));
        Assert.Throws<ArgumentOutOfRangeException>(() => context.GetFrameCountForDuration(-Fixed64.One));
    }

    [Fact]
    public void FrameWaitSupportsWideCountAndRejectsUnrepresentableDeadline()
    {
        using var context = GravitasWorldContext.CreateOwned();
        var wait = context.Coroutines.WaitForFrames((long)int.MaxValue + 5);
        TimingTestUtility.SetClock(context, (long)int.MaxValue + 4, new ChronicleTimestamp(1, 0));
        Assert.True(wait.KeepWaiting);
        Assert.True(wait.KeepWaiting);
        context.Simulate();
        Assert.False(wait.KeepWaiting);
        TimingTestUtility.SetClock(context, long.MaxValue, new ChronicleTimestamp(1, 0));
        Assert.False(context.Coroutines.WaitForFrames(0).KeepWaiting);
        Assert.Throws<OverflowException>(() => context.Coroutines.WaitForFrames(1));
        Assert.Throws<OverflowException>(() => context.Coroutines.WaitForNextSimulate());
    }
}
