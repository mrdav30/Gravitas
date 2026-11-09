using System;
using System.Diagnostics;
using Chronicler.Hashing;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using static Gravitas.Benchmarks.StrategyWorkload;

namespace Gravitas.Benchmarks;

internal static partial class QualityStackWorkload
{
    internal static int RunDensity(bool verify)
    {
        RunDensityCase(4, mixedLoads: false, verify);
        RunDensityCase(8, mixedLoads: false, verify);
        RunDensityCase(16, mixedLoads: false, verify);
        RunDensityCase(16, mixedLoads: true, verify);
        return 0;
    }

    private static void RunDensityCase(int count, bool mixedLoads, bool verify)
    {
        using GravitasWorldContext context = CreateSpatialContext();
        ConfigureQualityContext(context);
        LSCuboidCollider floor = CreateFloor(context);
        var lower = new LSCuboidCollider[count];
        var upper = new LSCuboidCollider[count];
        var loads = new[] { new StackLoad(100, 1, true), new StackLoad(100, 1, false),
            new StackLoad(1, 100, true), new StackLoad(1, 1, false) };
        FixedQuaternion rotation = mixedLoads
            ? FixedQuaternion.FromAxisAngle(Vector3d.Up, TiltRadians)
            : FixedQuaternion.Identity;
        int columns = count == 4 ? 2 : 4;
        int pitchX = 16 / columns;
        int pitchZ = 16 / (count / columns);
        for (int i = 0; i < count; i++)
        {
            StackLoad load = loads[i % (mixedLoads ? loads.Length : 2)];
            Fixed64 x = (Fixed64)((pitchX - 16) / 2 + i % columns * pitchX);
            Fixed64 z = (Fixed64)((pitchZ - 16) / 2 + i / columns * pitchZ);
            Vector2d bottom = Start(0, tilted: false, load.Aligned);
            Vector2d top = Start(1, tilted: false, load.Aligned);
            lower[i] = CreateStackCuboid();
            upper[i] = CreateStackCuboid();
            CreateSpatialBody(context, lower[i], new Vector3d(x + bottom.X, bottom.Y, z),
                rotation, load.Lower, sleep: false);
            CreateSpatialBody(context, upper[i], new Vector3d(x + top.X, top.Y, z),
                rotation, load.Upper, sleep: false);
        }

        var trace = new ChronicleHashWriter();
        trace.WriteSection("quality-stack-density-v1", 1);
        var elapsed = new long[Frames - WarmupFrames];
        long totalBytes = 0, maxBytes = 0;
        int freshRows = 0, rotatedSupports = 0, overruns = 0;
        for (int frame = 1; frame <= Frames; frame++)
        {
            bool warm = frame > WarmupFrames;
            long bytesBefore = warm ? GC.GetAllocatedBytesForCurrentThread() : 0;
            long start = warm ? Stopwatch.GetTimestamp() : 0;
            context.Simulate();
            context.LateSimulate();
            if (warm)
            {
                long ticks = Stopwatch.GetTimestamp() - start;
                long bytes = GC.GetAllocatedBytesForCurrentThread() - bytesBefore;
                elapsed[frame - WarmupFrames - 1] = ticks;
                if (ticks * 60 > Stopwatch.Frequency) overruns++;
                totalBytes += bytes;
                maxBytes = Math.Max(maxBytes, bytes);
                for (int i = 0; i < count; i++)
                {
                    freshRows += RequireUpwardRows(floor, lower[i], frame)
                        + RequireUpwardRows(lower[i], upper[i], frame);
                    if (lower[i].Shape == ColliderType.OBBox) rotatedSupports++;
                    if (lower[i].Shape == ColliderType.OBBox || upper[i].Shape == ColliderType.OBBox)
                        rotatedSupports++;
                }
            }

            Require(context.FrameCount == frame && context.LateSimulateToken == frame
                && context.Physics.BodyCount == count * 2, frame, "density workload lost a step or actor");
            if (verify)
            {
                ChronicleHash state = context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches);
                trace.WriteUInt64(state.Low);
                trace.WriteUInt64(state.High);
            }
        }

        Require(rotatedSupports == count * 2 * elapsed.Length, Frames,
            "density workload lost a rotated support");
        ChronicleHash terminal = context.ComputeReplayHash();
        Console.WriteLine($"density-v1 stacks {count}, mixed {mixedLoads}, warm frames {elapsed.Length}, active supports per frame {count * 2}, fresh rows {freshRows}, rotated support samples {rotatedSupports}, bytes {totalBytes}, max bytes {maxBytes}, terminal {terminal}");
        if (verify)
            Console.WriteLine($"trace {trace.ToHash()}");
        else
        {
            Array.Sort(elapsed);
            double msPerTick = 1000d / Stopwatch.Frequency;
            Console.WriteLine(FormattableString.Invariant(
                $"step ms p50 {elapsed[(elapsed.Length - 1) / 2] * msPerTick:F3}, p95 {elapsed[(elapsed.Length * 95 + 99) / 100 - 1] * msPerTick:F3}, max {elapsed[^1] * msPerTick:F3}, overruns {overruns}"));
        }
    }

    private static int RequireUpwardRows(LSCollider lower, LSCollider upper, int frame)
    {
        Require((lower.TryGetCollisionPair(upper.Id, out CollisionPair pair)
                || upper.TryGetCollisionPair(lower.Id, out pair))
            && pair.Active && pair.Manifold.LastUpdatedFrame == frame, frame, "density stack lost a fresh support");
        int rows = 0, upward = 0;
        foreach (ManifoldContact row in pair.Manifold)
        {
            Require(!row.DepthIsClamped, frame, "density stack has a clamped support depth");
            rows++;
            if ((pair.ColliderB == upper ? row.Normal.Y : -row.Normal.Y) > Fixed64.Zero)
                upward++;
        }
        Require(upward > 0, frame, "density stack lost upward support");
        return rows;
    }
}
