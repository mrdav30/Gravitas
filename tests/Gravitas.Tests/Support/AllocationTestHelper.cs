using System;

namespace Gravitas.Tests.Support;

internal static class AllocationTestHelper
{
    /// <summary>
    /// Measures an already-warmed single action and requires the action itself to stay allocation-free.
    /// </summary>
    public static long MeasureSinglePass(Action action)
    {
        ArgumentNullException.ThrowIfNull(action);

        ForceFullCollection();
        return Measure(action, iterations: 1);
    }

    /// <summary>
    /// Measures recurring hot-path work after warmup. A measured stabilization pass absorbs one-time
    /// runtime/test-runner allocations; the final measured pass remains a strict allocation check.
    /// </summary>
    public static long MeasureSteadyState(
        Action action,
        int warmupIterations = 128,
        int stabilizationIterations = 16,
        int measurementIterations = 64)
    {
        ArgumentNullException.ThrowIfNull(action);
        ValidateIterations(warmupIterations, nameof(warmupIterations));
        ValidateIterations(stabilizationIterations, nameof(stabilizationIterations));
        ValidateIterations(measurementIterations, nameof(measurementIterations));

        for (int i = 0; i < warmupIterations; i++)
            action();

        ForceFullCollection();
        _ = Measure(action, stabilizationIterations);
        ForceFullCollection();
        return Measure(action, measurementIterations);
    }

    private static long Measure(Action action, int iterations)
    {
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < iterations; i++)
            action();

        return GC.GetAllocatedBytesForCurrentThread() - before;
    }

    private static void ForceFullCollection()
    {
        GC.Collect();
        GC.WaitForPendingFinalizers();
        GC.Collect();
    }

    private static void ValidateIterations(int iterations, string parameterName)
    {
        if (iterations <= 0)
            throw new ArgumentOutOfRangeException(parameterName, iterations, "Iteration count must be positive.");
    }
}
