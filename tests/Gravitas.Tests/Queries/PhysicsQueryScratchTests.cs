using System;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using GridForge.Configuration;
using GridForge.Grids;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class PhysicsQueryScratchTests
{
    [Fact]
    public void ScratchExposesFixedCapacitiesAndRejectsMeshCapacityOutsideSupportedRange()
    {
        var scratch = new PhysicsQueryScratch(3, 2);
        Assert.Equal(3, scratch.TraversalCapacity);
        Assert.Equal(2, scratch.CandidateCapacity);
        Assert.Equal(0, scratch.MeshTriangleCapacity);
        Assert.Throws<ArgumentOutOfRangeException>("meshTriangleCapacity", () => new PhysicsQueryScratch(0, 0, -1));
        Assert.Throws<ArgumentOutOfRangeException>("meshTriangleCapacity",
            () => new PhysicsQueryScratch(0, 0, PhysicsMesh.MaxTriangleCount + 1));
    }

    [Fact]
    public void PublishedViewIsRequiredEvenAfterTheContextObservesTheChange()
    {
        using var context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.ThreeD;
        var scratch = new PhysicsQueryScratch(8, 1);
        var budget = new PhysicsQueryBudget(64, 64, 64);
        int callbacks = 0;
        ulong observedSequence = 0;
        ulong eventSequence = 0;
        ulong certificationWork = 0;
        PhysicsQueryExecutionStatus? certification = null;
        PhysicsQueryExecutionStatus? duringPublication = null;
        Physics3DHit? duringHit = null;
        PhysicsQueryWork duringWork = default;
        context.World.OnChangeCommitted += change =>
        {
            if (change.ChangeKind != GridEventKind.GridAdded)
                return;
            callbacks++;
            observedSequence = context.ObservedGridChangeSequence;
            eventSequence = change.ChangeSequence;
            var meter = new PhysicsQueryMeter(budget);
            certification = PhysicsQueryScratch.CheckView(context, change.ChangeSequence, ref meter);
            certificationWork = meter.Work.Discovery;
            duringPublication = Sweep(out duringHit, out duringWork);
        };
        Assert.True(context.World.TryAddGrid(new GridConfiguration(Vector3d.Zero, Vector3d.One), out _));
        Assert.Equal(1, callbacks);
        Assert.Equal(eventSequence, observedSequence);
        Assert.Equal(PhysicsQueryExecutionStatus.Stale, certification);
        Assert.Equal(1UL, certificationWork);
        Assert.Equal(PhysicsQueryExecutionStatus.WorldNotReady, duringPublication);
        Assert.Null(duringHit);
        Assert.Equal(default, duringWork);
        Assert.Equal(PhysicsQueryExecutionStatus.Completed, Sweep(out var recovered, out _));
        Assert.Null(recovered);

        PhysicsQueryExecutionStatus Sweep(out Physics3DHit? hit, out PhysicsQueryWork work) =>
            context.Query3D.SweepSphere(Vector3d.Up, Fixed64.Half, Vector3d.Down, Fixed64.One,
                PhysicsLayerMask.All, null, budget, scratch, out hit, out work);
    }

    [Fact]
    public void ViewCheckChargesBeforeCertificationAndRejectsAnAcquiredGenerationAfterReset()
    {
        using var context = GravitasWorldContext.CreateOwned();
        Assert.True(context.World.TryAddGrid(new GridConfiguration(Vector3d.Zero, Vector3d.One), out _));
        ulong acquired = context.ObservedGridChangeSequence;
        var zero = new PhysicsQueryMeter(default);
        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
            PhysicsQueryScratch.CheckView(context, acquired, ref zero));
        Assert.Equal(default, zero.Work);

        var meter = new PhysicsQueryMeter(new PhysicsQueryBudget(1, 0, 0));
        Assert.Equal(PhysicsQueryExecutionStatus.Completed, PhysicsQueryScratch.CheckView(context, acquired, ref meter));
        Assert.Equal(1UL, meter.Work.Discovery);

        context.World.Reset();
        Assert.True(context.World.TryAddGrid(new GridConfiguration(Vector3d.Zero, Vector3d.One), out _));
        Assert.Equal(PhysicsQueryExecutionStatus.Stale, PhysicsQueryScratch.CheckView(context, acquired, ref meter));
        Assert.Equal(1UL, meter.Work.Discovery);
    }

    [Fact]
    public void ClosedHostWorldCannotCertifyABoundedQueryView()
    {
        using var world = new GridWorld();
        using var context = GravitasWorldContext.Attach(world);
        ulong sequence = context.ObservedGridChangeSequence;
        world.Dispose();
        var meter = new PhysicsQueryMeter(new PhysicsQueryBudget(1, 0, 0));
        Assert.Equal(PhysicsQueryExecutionStatus.Stale, PhysicsQueryScratch.CheckView(context, sequence, ref meter));
        Assert.Equal(default, meter.Work);
    }

    [Fact]
    public void WorkMeterRejectsOverflowUnderMaximumAllowances()
    {
        var meter = new PhysicsQueryMeter(new PhysicsQueryBudget(ulong.MaxValue, ulong.MaxValue, ulong.MaxValue));
        Assert.True(meter.ChargeCandidates(ulong.MaxValue));
        Assert.True(meter.ChargeGeometry(ulong.MaxValue - 1));
        Assert.False(meter.ChargeCandidate());
        Assert.False(meter.ChargeGeometry(2));
        Assert.True(meter.ChargeGeometry(1));
        Assert.Equal(new PhysicsQueryWork(0, ulong.MaxValue, ulong.MaxValue), meter.Work);
    }

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
