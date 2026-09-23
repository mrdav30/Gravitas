using System;
using System.Collections.Generic;
using Chronicler.Timing;
using FixedMathSharp;
using FixedMathSharp.Chronicler;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using Xunit;

namespace Gravitas.Tests.Runtime;

public sealed class GravitasTimingBoundaryTests
{
    [Theory]
    [InlineData(0L)]
    [InlineData((long)int.MaxValue)]
    [InlineData(long.MaxValue - 3)]
    public void EmptyPartitionsRetireAfterTheirConfiguredLifetime(long initialFrame)
    {
        using var context = GravitasWorldContext.CreateOwned();
        context.Settings.RetainedPartitionTimeToKillFrames = 2;
        context.Settings.RetainedPartitionRetirementSweepBudget = 1024;
        Assert.True(context.World.TryAddGrid(new GridConfiguration(
            new Vector3d(-2, -2, -2), new Vector3d(2, 2, 2)), out _));
        var collider = new LSSphereCollider();
        var body = new SolidBody(new TestMatterAgent(context), collider) { Mass = Fixed64.One };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
        TimingTestUtility.SetClock(context, initialFrame, new ChronicleTimestamp(1, 0));
        Step(context);
        collider.Deactivate();
        Assert.True(context.Collisions.RetainedPartitionCount > 0);
        Step(context);
        Step(context);
        Assert.Equal(0, context.Collisions.RetainedPartitionCount);
        Assert.True(context.Collisions.InactivePartitionCount > 0);
    }

    [Fact]
    public void FrameCoroutineResumesExactlyOnceAcrossNarrowBoundary()
    {
        using var context = GravitasWorldContext.CreateOwned();
        TimingTestUtility.SetClock(context, int.MaxValue - 1L, new ChronicleTimestamp(1, 0));
        int resumes = 0;
        context.Coroutines.StartCoroutine(Run());
        Step(context);
        Step(context);
        Assert.Equal(0, resumes);
        Step(context);
        Assert.Equal(1, resumes);
        Step(context);
        Assert.Equal(1, resumes);
        Assert.Equal(0, context.Coroutines.ActiveCoroutineCount);
        IEnumerator<ILockedYieldInstruction> Run()
        {
            yield return context.Coroutines.WaitForFrames(2);
            resumes++;
        }
    }

    [Fact]
    public void SecondsCoroutineDoesNotCompleteEarlyAtNarrowTimeBoundary()
    {
        using var context = GravitasWorldContext.CreateOwned();
        TimingTestUtility.SetClock(context, 1, ChronicleTimestamp.Zero
            + FixedChronicleTime.FromFixed64(Fixed64.MaxValue - context.DeltaTime));
        int resumes = 0;
        context.Coroutines.StartCoroutine(Run());
        Step(context);
        Step(context);
        Assert.Equal(0, resumes);
        IEnumerator<ILockedYieldInstruction> Run()
        {
            yield return context.Coroutines.WaitForRealSeconds(Fixed64.One);
            resumes++;
        }
    }

    [Fact]
    public void ElapsedTimeContinuesAfterNarrowRange()
    {
        using var context = GravitasWorldContext.CreateOwned();
        TimingTestUtility.SetClock(context, 1, new ChronicleTimestamp(int.MaxValue, uint.MaxValue));
        ChronicleTimestamp before = context.ElapsedTime;
        Step(context);
        Assert.Equal(FixedChronicleTime.FromFixed64(context.DeltaTime), context.ElapsedTime - before);
    }

    [Fact]
    public void DurationConversionCanExceedIntFrameRange()
    {
        using var context = GravitasWorldContext.CreateOwned();
        Assert.Equal(2147483648L, context.GetFrameCountForDuration((Fixed64)67108864));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void WaitFromPreviousLifetimeRejectsAfterReset(int kind)
    {
        using var context = GravitasWorldContext.CreateOwned();
        ILockedYieldInstruction wait = kind switch
        {
            0 => context.Coroutines.WaitForFrames(2),
            1 => context.Coroutines.WaitForNextSimulate(),
            _ => context.Coroutines.WaitForRealSeconds(Fixed64.One)
        };
        context.Reset();
        Assert.Throws<InvalidOperationException>(() => wait.KeepWaiting);
    }

    private static void Step(GravitasWorldContext context)
    {
        context.Simulate();
        context.LateSimulate();
    }
}
