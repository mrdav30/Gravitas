using Gravitas.Queries;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class PhysicsQueryScratchTests
{
    [Fact]
    public void RememberCandidate_ChargesBeforeShiftingAndPreservesExistingIdsOnExhaustion()
    {
        var scratch = new PhysicsQueryScratch(0, 2);
        var meter = new PhysicsQueryMeter(new PhysicsQueryBudget(0, 1, 0));

        Assert.Equal(PhysicsQueryExecutionStatus.Completed, scratch.RememberCandidate(2, ref meter, out bool added));
        Assert.True(added);
        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded, scratch.RememberCandidate(1, ref meter, out added));
        Assert.False(added);
        Assert.Equal(1UL, meter.Work.Candidates);

        meter = new PhysicsQueryMeter(new PhysicsQueryBudget(0, 5, 0));
        Assert.Equal(PhysicsQueryExecutionStatus.Completed, scratch.RememberCandidate(1, ref meter, out added));
        Assert.True(added);
        Assert.Equal(2UL, meter.Work.Candidates);
        Assert.Equal(PhysicsQueryExecutionStatus.Completed, scratch.RememberCandidate(2, ref meter, out added));
        Assert.False(added);
        Assert.Equal(4UL, meter.Work.Candidates);
    }
}
