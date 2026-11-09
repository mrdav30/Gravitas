using System;
using System.Diagnostics;
using System.Globalization;
using System.Linq;
using System.Runtime.CompilerServices;
using Chronicler.Hashing;
using static Gravitas.Benchmarks.StrategyWorkload;

namespace Gravitas.Benchmarks;

internal sealed class StrategySoak
{
    internal const int RecoveryStart = FrameCount + 1;
    private const int CycleFrames = FrameCount + StageFrames;
    private const int WarmupCycles = 4;
    private const int RetirementFrames = 1024;

    private readonly string _id;
    private readonly Process _process;
    private readonly int _cycles;
    private readonly FrameTiming[] _samples;
    private readonly long _startupBytes = GC.GetAllocatedBytesForCurrentThread();
    private bool MeasuresHeap => _samples.Length == 0;

    private StrategySoak(string id, Process process, int cycles, bool heap)
    {
        _id = id;
        _process = process;
        _cycles = cycles;
        _samples = heap ? Array.Empty<FrameTiming>() : new FrameTiming[cycles * CycleFrames];
    }

    internal static int Run(string id, Action<StrategySoak> run)
    {
        int cycles = 80;
        string configuredCycles = Environment.GetEnvironmentVariable("GRAVITAS_STRATEGY_SOAK_CYCLES");
        if (configuredCycles != null &&
            (!int.TryParse(configuredCycles, NumberStyles.None, CultureInfo.InvariantCulture, out cycles) || cycles is < 1 or > 4096))
        {
            Console.Error.WriteLine("GRAVITAS_STRATEGY_SOAK_CYCLES must be an integer from 1 to 4096.");
            return 1;
        }

        bool heap = Environment.GetEnvironmentVariable("GRAVITAS_STRATEGY_SOAK_HEAP") == "1";
        using Process process = Process.GetCurrentProcess();
        var soak = new StrategySoak(id, process, cycles, heap);
        Console.WriteLine($"{id}: {WarmupCycles * CycleFrames} warmup frames, {cycles * CycleFrames} " +
            $"{(heap ? "untimed heap probe" : "timed")} frames.");
        soak.WriteMemory("before-context", forceCollection: heap);
        Invoke(run, soak);
        soak.WriteMemory("disposed", forceCollection: heap);
        if (!heap)
            soak.WriteTimings();
        return 0;
    }

    // Leave the context and host actor arrays out of scope before the disposal sample.
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void Invoke(Action<StrategySoak> run, StrategySoak soak) => run(soak);

    internal void Capture<TRegion>(GravitasWorldContext context, TRegion[] regions,
        Action<TRegion, int> commands, Action<TRegion> queries, QueryRegion<TRegion> observeQueries,
        Func<ChronicleHash?, int, Observation> observeWork, Action<TRegion> retire)
    {
        WriteMemory("initialized", context);
        bool heap = MeasuresHeap;
        int warmup = WarmupCycles * CycleFrames;
        int last = warmup + _cycles * CycleFrames;
        var queryTrace = new ChronicleHashWriter();
        queryTrace.WriteSection(_id, 1);
        var checkpoints = new ChronicleHashWriter();
        checkpoints.WriteSection(_id, 1);
        int gen0 = 0, gen1 = 0, gen2 = 0;
        Observation terminal = default;
        for (int frame = 1; frame <= last; frame++)
        {
            int phase = (frame - 1) % CycleFrames + 1;
            bool sample = !heap && frame > warmup;
            ChronicleHash? query = Step(context, regions, commands, queries, observeQueries, frame, phase,
                sample, measureScenarioBytes: sample, out FrameTiming timing);
            if (sample)
                _samples[frame - warmup - 1] = timing;
            if (query is ChronicleHash hash)
            {
                queryTrace.WriteInt32(frame);
                queryTrace.WriteUInt64(hash.Low);
                queryTrace.WriteUInt64(hash.High);
            }

            if (heap && phase is 1 or SparseStart or FireFrame or (CrowdedStart - 1) or CrowdedStart
                or (CrowdedStart + 2) or CycleFrames)
            {
                terminal = observeWork(query, phase == CycleFrames ? StageFrames : phase);
                if (phase == CrowdedStart + 2)
                    Require(terminal.ChainContacts == regions.Length * (EncounterActors - 1) &&
                        terminal.LargestChain == EncounterActors, frame, "the encounter chains did not complete");
                checkpoints.WriteInt32(frame);
                checkpoints.WriteUInt64(terminal.State.Low);
                checkpoints.WriteUInt64(terminal.State.High);
                checkpoints.WriteInt32(terminal.Pairs);
                checkpoints.WriteInt32(terminal.ToiIterations);
            }

            if (frame == warmup)
            {
                WriteMemory("warmed", context, forceCollection: heap);
                gen0 = GC.CollectionCount(0);
                gen1 = GC.CollectionCount(1);
                gen2 = GC.CollectionCount(2);
            }
            if (heap && frame > warmup && phase == CycleFrames)
            {
                int cycle = (frame - warmup) / CycleFrames;
                if (cycle % 20 == 0 || frame == last)
                    WriteMemory($"cycle-{cycle}", context, forceCollection: true);
            }
        }

        if (!heap)
        {
            Console.WriteLine($"Measured collections (gen0 / gen1 / gen2): {GC.CollectionCount(0) - gen0} / " +
                $"{GC.CollectionCount(1) - gen1} / {GC.CollectionCount(2) - gen2}");
            WriteMemory("measured-end", context);
            terminal = observeWork(null, StageFrames);
        }
        Console.WriteLine($"Replay: {last} frames, ordered queries {queryTrace.ToHash()}, terminal {terminal.State}");
        if (!heap)
            return;

        Console.WriteLine($"Heap-pass checkpoints: {checkpoints.ToHash()}");
        // Release the heap probe's last replay snapshot before retiring its actors.
        context.Physics.PrepareReplayColliders().Clear(clearRetainedReferences: true);
        context.Physics2D.PrepareReplayColliders().Clear(clearRetainedReferences: true);
        foreach (TRegion region in regions)
            retire(region);
        for (int i = 0; i < RetirementFrames; i++)
        {
            context.Simulate();
            context.LateSimulate();
        }
        int retiredFrame = last + RetirementFrames;
        Require(context.FrameCount == retiredFrame && context.LateSimulateToken == retiredFrame &&
            context.Physics.BodyCount == 0 && context.Physics2D.BodyCount == 0,
            retiredFrame, "retirement did not drain the dynamic population");
        WriteMemory($"retired-{RetirementFrames}", context, forceCollection: true);
    }

    private void WriteMemory(string label, GravitasWorldContext context = null, bool forceCollection = false)
    {
        long allocated = GC.GetAllocatedBytesForCurrentThread() - _startupBytes;
        if (forceCollection)
        {
            GC.Collect(GC.MaxGeneration, GCCollectionMode.Forced, blocking: true, compacting: true);
            GC.WaitForPendingFinalizers();
            GC.Collect(GC.MaxGeneration, GCCollectionMode.Forced, blocking: true, compacting: true);
        }
        long heap = GC.GetTotalMemory(forceFullCollection: false);
        _process.Refresh();
        Console.WriteLine($"Memory {label}: thread allocated {allocated}, managed {heap}, " +
            $"working set {_process.WorkingSet64}, peak working set {_process.PeakWorkingSet64} bytes");
        if (context != null)
            Console.WriteLine($"  Bodies {context.Physics.BodyCount + context.Physics2D.BodyCount}, " +
                $"colliders {context.Physics.ColliderCount + context.Physics2D.ColliderCount}, partitions " +
                $"active {context.Collisions.ActivePartitionCount + context.Collisions2D.ActivePartitionCount}, " +
                $"retained {context.Collisions.RetainedPartitionCount + context.Collisions2D.RetainedPartitionCount}, " +
                $"pooled {context.Collisions.InactivePartitionCount + context.Collisions2D.InactivePartitionCount}");
    }

    private void WriteTimings()
    {
        int warmup = WarmupCycles * CycleFrames;
        WriteGroup("All measured", Enumerable.Range(0, _samples.Length).ToArray());
        string[] stages = { "Quiet", "Sparse", "Crowded", "Rearm", "Recovery" };
        for (int stage = 0; stage < stages.Length; stage++)
            WriteGroup(stages[stage], Enumerable.Range(0, _samples.Length).Where(index => Stage(index) == stage).ToArray());

        if (Environment.GetEnvironmentVariable("GRAVITAS_STRATEGY_FRAME_CSV") != "1")
            return;
        Console.WriteLine("frame,cycle,phase,physics_ms,scenario_ms,physics_bytes,scenario_bytes");
        for (int i = 0; i < _samples.Length; i++)
        {
            FrameTiming sample = _samples[i];
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"{warmup + i + 1},{i / CycleFrames + 1},{i % CycleFrames + 1},{Milliseconds(sample.StepTicks):F6}," +
                $"{Milliseconds(sample.FrameTicks):F6},{sample.StepBytes},{sample.FrameBytes}"));
        }
    }

    private void WriteGroup(string name, int[] frames)
    {
        Console.WriteLine($"{name}:");
        WriteWindow("Physics step", frames.Select(index => _samples[index].StepTicks).Order().ToArray());
        WriteWindow("Scenario frame", frames.Select(index => _samples[index].FrameTicks).Order().ToArray());
        Console.WriteLine($"  Allocated bytes (physics / scenario): {frames.Sum(index => _samples[index].StepBytes)} / " +
            $"{frames.Sum(index => _samples[index].FrameBytes)}");
        int physicsMax = frames.MaxBy(index => _samples[index].StepTicks);
        int scenarioMax = frames.MaxBy(index => _samples[index].FrameTicks);
        Console.WriteLine($"  Max frames (physics / scenario): {WarmupCycles * CycleFrames + physicsMax + 1} / " +
            $"{WarmupCycles * CycleFrames + scenarioMax + 1}");
    }

    private static int Stage(int index) => (index % CycleFrames + 1) switch
    {
        <= StageFrames => 0,
        < CrowdedStart => 1,
        <= FrameCount => 2,
        RecoveryStart => 3,
        _ => 4
    };
}
