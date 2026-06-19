using FluentAssertions;
using System;
using Xunit;

namespace Gravitas.Tests.Support;

public sealed class AllocationTestHelperTests
{
    [Fact]
    public void MeasureSteadyState_ShouldIgnoreSinglePostWarmupAllocation()
    {
        int callCount = 0;

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            () =>
            {
                callCount++;
                if (callCount == 2)
                    GC.KeepAlive(new byte[32]);
            },
            warmupIterations: 1,
            stabilizationIterations: 1,
            measurementIterations: 1);

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void MeasureSteadyState_ShouldReportPersistentAllocations()
    {
        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            () => GC.KeepAlive(new byte[32]),
            warmupIterations: 1,
            stabilizationIterations: 1,
            measurementIterations: 1);

        allocatedBytes.Should().BeGreaterThan(0);
    }
}
