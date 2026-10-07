using Chronicler.Hashing;
using FluentAssertions;
using System;

namespace Gravitas.Tests.Determinism;

internal sealed class ReplayHashTrace
{
    public ReplayHashTrace(int frameCount)
    {
        Hashes = new ChronicleHash[frameCount];
    }

    public ChronicleHash[] Hashes { get; }

    public ChronicleHash this[int frame]
    {
        get => Hashes[frame];
        set => Hashes[frame] = value;
    }
}

internal static class ReplayConformanceHarness
{
    public static void AssertRepeatedRunsMatch(
        Func<GravitasWorldContext> createScenario,
        int frameCount,
        Action<GravitasWorldContext, int>? beforeFrame = null,
        GravitasReplayHashMode mode = GravitasReplayHashMode.Authoritative)
    {
        using GravitasWorldContext first = createScenario();
        using GravitasWorldContext second = createScenario();

        ReplayHashTrace firstTrace = RunTrace(first, frameCount, beforeFrame, mode);
        ReplayHashTrace secondTrace = RunTrace(second, frameCount, beforeFrame, mode);

        secondTrace.Hashes.Should().Equal(firstTrace.Hashes);
    }

    public static ReplayHashTrace RunTrace(
        GravitasWorldContext context,
        int frameCount,
        Action<GravitasWorldContext, int>? beforeFrame = null,
        GravitasReplayHashMode mode = GravitasReplayHashMode.Authoritative)
    {
        var trace = new ReplayHashTrace(frameCount);
        for (int frame = 0; frame < frameCount; frame++)
        {
            // The callback applies host commands; the harness owns both fixed phases.
            beforeFrame?.Invoke(context, frame);
            context.Simulate();
            context.LateSimulate();
            trace[frame] = context.ComputeReplayHash(mode);
        }

        return trace;
    }

    public static void AssertNextFramesMatch(
        GravitasWorldContext first,
        GravitasWorldContext second,
        int frameCount,
        GravitasReplayHashMode mode = GravitasReplayHashMode.Authoritative)
    {
        first.ComputeReplayHash(mode).Should().Be(second.ComputeReplayHash(mode));
        for (int frame = 0; frame < frameCount; frame++)
        {
            first.Simulate();
            first.LateSimulate();
            second.Simulate();
            second.LateSimulate();
            second.ComputeReplayHash(mode)
                .Should()
                .Be(first.ComputeReplayHash(mode), $"replay hash drifted at continuation frame {frame}");
        }
    }
}
