using System;
using Chronicler.Hashing;
using FixedMathSharp;
using static FixedMathSharp.FixedMath;
using static Gravitas.Benchmarks.StrategyWorkload;

namespace Gravitas.Benchmarks;

internal static partial class QualityStackWorkload
{
    private static readonly Fixed64 StackPulseSpeed = -Fixed64.Quarter;

    internal static int VerifyDuration(bool strict = false)
    {
        bool csv = Environment.GetEnvironmentVariable("GRAVITAS_QUALITY_FRAME_CSV") == "1";
        if (csv)
            Console.WriteLine("fixture,frame,seconds,pulse,floor_lower_depth,lower_upper_depth,upper_floor,original_supports,quiet_supported,sleeping,drift,angle_deg,mechanical_energy");

        bool supportBoundsMet = true;
        foreach (bool planar in new[] { true, false })
        foreach (var (upperMass, disturbed) in new[] { (1, true), (100, false), (100, true) })
        foreach (int rate in new[] { 32, 60 })
        {
            string id = $"quality-{(planar ? "2d" : "3d")}-stack-m{upperMass}-{rate}hz-{(disturbed ? "pulse" : "control")}-v1";
            StackDuration first = RunDuration(planar, upperMass, rate, disturbed, id, observe: true, csv);
            StackDuration repeat = RunDuration(planar, upperMass, rate, disturbed, id, observe: true, csv: false);
            StackDuration control = RunDuration(planar, upperMass, rate, disturbed, id, observe: false, csv: false);
            Require(first == repeat && first.Terminal == control.Terminal, rate * QualityDuration.Seconds,
                $"{id} duration replay diverged");
            bool supported = first.SupportLossFrames == 0 && first.UpperFloorFrames == 0 && first.FirstDeepFrame < 0
                && first.Lowest > Fixed64.Zero;
            supportBoundsMet &= supported;
            Console.WriteLine($"{id} trace {first.Trace}, terminal {first.Terminal}, frames {rate * QualityDuration.Seconds}, first both supports {first.FirstSupport}, unsupported after first {first.SupportLossFrames}, upper-floor frames {first.UpperFloorFrames}, support bounds {(supported ? "within" : "outside")}");
            Console.WriteLine($"  First depth above 1/16 {first.FirstDeepFrame}, first upper-floor {first.FirstUpperFloorFrame}, first 0.5s supported quiet window per segment {first.QuietOffsets}");
            Console.WriteLine(FormattableString.Invariant(
                $"  Floor/lower and lower/upper peak depths {first.PeakFloorDepth:F6}/{first.PeakLoadDepth:F6}, final depth {first.FinalDepth:F6}, minimum COM {first.Lowest:F6}, peak drift {first.PeakDrift:F6}, peak rotation {first.PeakAngle:F4} deg"));
            Console.WriteLine($"  First both-asleep step per segment {first.RestOffsets}, both-asleep steps per segment {first.RestFrames}, pulses starting asleep {first.PulsesFromRest}, connected wakes {first.ConnectedWakes}, sleeps/wakes {first.Sleeps}/{first.Wakes}, warm bytes {control.Bytes}, peak {control.PeakBytes}");
        }
        return strict && !supportBoundsMet ? 1 : 0;
    }

    private static StackDuration RunDuration(bool planar, int upperMass, int rate, bool disturbed, string id, bool observe, bool csv)
    {
        using GravitasWorldContext context = CreateQualityContext(planar);
        context.SetFrameRate(rate);
        var load = new StackLoad(1, upperMass, Aligned: true);
        Scene scene = planar ? Planar(context, load, tilted: false, measure: false, quietFrames: rate / 2)
            : Spatial(context, load, tilted: false, measure: false, quietFrames: rate / 2);
        int frames = rate * QualityDuration.Seconds, period = rate * QualityDuration.PeriodSeconds;
        var trace = new ChronicleHashWriter();
        trace.WriteSection(id, 1);
        var sleeping = new bool[scene.Count];
        SupportSample[] samples = observe ? new SupportSample[frames + 1] : null;
        var restOffsets = new int[QualityDuration.Episodes];
        var restFrames = new int[restOffsets.Length];
        var quietOffsets = new int[restOffsets.Length];
        Array.Fill(restOffsets, -1);
        Array.Fill(quietOffsets, -1);
        int firstSupport = -1, supportLoss = 0, upperFloor = 0, pulsesFromRest = 0, connectedWakes = 0;
        int firstDeep = -1, firstUpperFloor = -1, quietStreak = 0;
        int sleeps = 0, wakes = 0, pulses = 0;
        Fixed64 floorDepth = Fixed64.Zero, loadDepth = Fixed64.Zero, lowest = (Fixed64)16;
        Fixed64 drift = Fixed64.Zero, angle = Fixed64.Zero;
        long bytes = 0, peakBytes = 0;
        Reading row = default;
        for (int frame = 0; frame <= frames; frame++)
        {
            int previousFrame = frame - 1;
            bool pulse = disturbed && QualityDuration.IsPulse(frame, rate);
            bool fromRest = pulse && row.Sleeping == scene.Count;
            bool hadBothSupports = row.NeighborPairs == 2;
            if (pulse)
            {
                Fixed64 incoming = observe ? scene.ReadBody(scene.Count - 1).Linear.Y : Fixed64.Zero;
                scene.PulseUpper(StackPulseSpeed);
                pulses++;
                if (observe)
                    Require((scene.ReadBody(scene.Count - 1).Linear.Y - incoming - StackPulseSpeed).Abs()
                        <= Fixed64.FromRaw(4096), frame, $"{id} missed its authored velocity change");
            }
            if (frame > 0)
            {
                bool measure = !observe && frame > rate;
                long before = measure ? GC.GetAllocatedBytesForCurrentThread() : 0;
                context.Simulate();
                context.LateSimulate();
                if (measure)
                {
                    long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
                    bytes += allocated;
                    peakBytes = Math.Max(peakBytes, allocated);
                }
            }
            Require(context.FrameCount == frame && context.LateSimulateToken == frame
                && (planar ? context.Physics2D.BodyCount : context.Physics.BodyCount) == scene.Count,
                frame, $"{id} lost a frame or stack body");
            if (!observe)
                continue;

            row = Read(scene, sleeping, frame, ref trace, samples);
            SupportSample support = samples[frame];
            if (row.NeighborPairs == 2 && firstSupport < 0)
                firstSupport = frame;
            if (firstSupport >= 0 && row.NeighborPairs != 2)
                supportLoss++;
            if (support.FloorUpper.Overlap)
            {
                upperFloor++;
                if (firstUpperFloor < 0)
                    firstUpperFloor = frame;
            }
            floorDepth = Max(floorDepth, support.FloorLower.Residual);
            loadDepth = Max(loadDepth, support.LowerUpper.Residual);
            if (firstDeep < 0 && Max(floorDepth, loadDepth) > ResidualBound)
                firstDeep = frame;
            lowest = Min(lowest, row.LowestHeight);
            drift = Max(drift, row.Drift);
            angle = Max(angle, row.Angle);
            sleeps += row.Sleeps;
            wakes += row.Wakes;
            if (fromRest)
            {
                pulsesFromRest++;
                if (hadBothSupports && row.NeighborPairs == 2 && row.Sleeping == 0)
                    connectedWakes++;
            }
            bool quietSupported = row.Quiet && row.NeighborPairs == 2 && !support.FloorUpper.Overlap;
            if (frame > 0)
            {
                int segment = previousFrame / period;
                int offset = frame - segment * period;
                if (offset == 1 || !quietSupported)
                    quietStreak = 0;
                if (quietSupported && ++quietStreak == rate / 2 && quietOffsets[segment] < 0)
                    quietOffsets[segment] = offset;
                if (row.Sleeping == scene.Count)
                {
                    restFrames[segment]++;
                    if (restOffsets[segment] < 0)
                        restOffsets[segment] = offset;
                }
            }

            ChronicleHash hash = context.ComputeReplayHash();
            trace.WriteUInt64(hash.Low);
            trace.WriteUInt64(hash.High);
            trace.WriteBool(quietSupported);
            if (csv)
                Console.WriteLine(FormattableString.Invariant(
                    $"{id},{frame},{Fixed64.FromFraction(frame, rate):F6},{pulse},{support.FloorLower.Residual:F6},{support.LowerUpper.Residual:F6},{support.FloorUpper.Overlap},{row.NeighborPairs},{quietSupported},{row.Sleeping},{row.Drift:F6},{row.Angle:F6},{row.Energy:F6}"));
        }

        Require(pulses == (disturbed ? restOffsets.Length - 1 : 0), frames, $"{id} missed its pulse schedule");
        if (observe)
            Require(firstSupport > 0, frames, $"{id} never formed the authored support chain");
        return new StackDuration(trace.ToHash(), context.ComputeReplayHash(), firstSupport, supportLoss, upperFloor,
            floorDepth, loadDepth, row.Residual, lowest, drift, angle, firstDeep, firstUpperFloor,
            string.Join("/", quietOffsets), string.Join("/", restOffsets), string.Join("/", restFrames),
            pulsesFromRest, connectedWakes, sleeps, wakes, bytes, peakBytes);
    }

    private readonly record struct StackDuration(ChronicleHash Trace, ChronicleHash Terminal, int FirstSupport,
        int SupportLossFrames, int UpperFloorFrames, Fixed64 PeakFloorDepth, Fixed64 PeakLoadDepth, Fixed64 FinalDepth,
        Fixed64 Lowest, Fixed64 PeakDrift, Fixed64 PeakAngle, int FirstDeepFrame, int FirstUpperFloorFrame,
        string QuietOffsets, string RestOffsets, string RestFrames, int PulsesFromRest, int ConnectedWakes,
        int Sleeps, int Wakes, long Bytes, long PeakBytes);
}
