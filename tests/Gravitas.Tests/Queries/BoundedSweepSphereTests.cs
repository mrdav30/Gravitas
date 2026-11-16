using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Serialization;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class BoundedSweepSphereTests
{
    public static TheoryData<GravitasSerializationTransport> Transports => GravitasSerializationTransportCases.All();

    [Theory]
    [MemberData(nameof(Transports))]
    public void BoundedSweep_ReplaysAfterRemovedMembershipsAndCheckpointRestore(GravitasSerializationTransport transport)
    {
        using var live = CreateContext();
        using var restored = CreateContext();
        var liveFloor = AddBoxFloor(live);
        var restoredFloor = AddBoxFloor(restored);
        var liveSecond = AddBoxFloor(live);
        var restoredSecond = AddBoxFloor(restored);
        var discarded = new LSCuboidCollider[16];
        for (int i = 0; i < discarded.Length; i++)
            discarded[i] = AddBoxFloor(live);
        foreach (var collider in discarded)
            collider.Deactivate();
        Vector3d floorPosition = liveFloor.Position;
        liveFloor.Position = floorPosition + Vector3d.Right * (Fixed64)32;
        liveFloor.Position = floorPosition;
        live.Simulate();
        live.LateSimulate();
        GravitasSerializationHarness.Populate(restoredFloor,
            GravitasSerializationHarness.Serialize(liveFloor, transport), transport);
        GravitasSerializationHarness.Populate(restoredSecond,
            GravitasSerializationHarness.Serialize(liveSecond, transport), transport);
        restored.Simulate();
        restored.LateSimulate();

        var scratch = new PhysicsQueryScratch(512, 16);
        var ample = new PhysicsQueryBudget(8192, 8192, 8192);
        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            Sweep(restored, ample, scratch, out var hit, out var expected));
        Assert.Same(restoredFloor, hit.GetValueOrDefault().Collider);
        var exact = new PhysicsQueryBudget(expected.Discovery, expected.Candidates, expected.Geometry);
        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            Sweep(live, exact, scratch, out var replay, out var used));
        Assert.Equal(expected, used);
        Assert.Same(liveFloor, replay.GetValueOrDefault().Collider);
        Assert.Equal(hit!.Value.Point, replay!.Value.Point);
        Assert.Equal(hit.Value.Distance, replay.Value.Distance);
        Assert.Equal(hit.Value.Normal, replay.Value.Normal);

        var oneBelow = new PhysicsQueryBudget(expected.Discovery, expected.Candidates - 1, expected.Geometry);
        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
            Sweep(live, oneBelow, scratch, out replay, out used));
        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
            Sweep(restored, oneBelow, scratch, out hit, out expected));
        Assert.Equal(expected, used);
        Assert.Null(hit);
        Assert.Null(replay);
    }

    [Fact]
    public void BoxFloorMatchesSynchronousSweepAndReplaysAtExactLimits()
    {
        using GravitasWorldContext context = CreateContext();
        AddBoxFloor(context);
        AddBoxFloor(context);
        context.Simulate();
        context.LateSimulate();

        Assert.True(context.Query3D.SweepSphere(Vector3d.Up, Fixed64.Half,
            Vector3d.Down, Fixed64.One, out Physics3DHit expected, PhysicsLayerMask.All));
        Assert.Equal(Fixed64.Half, expected.Distance);
        Assert.Equal(Vector3d.Zero, expected.Point);

        var scratch = new PhysicsQueryScratch(512, 2);
        var ample = new PhysicsQueryBudget(8192, 8192, 8192);
        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            Sweep(context, ample, scratch, out Physics3DHit? hit, out PhysicsQueryWork used));
        Assert.Equal(expected, hit!.Value);

        var exact = new PhysicsQueryBudget(used.Discovery, used.Candidates, used.Geometry);
        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
            Sweep(context, new PhysicsQueryBudget(used.Discovery, used.Candidates, used.Geometry - 1),
                scratch, out hit, out PhysicsQueryWork failed));
        Assert.Null(hit);
        Assert.True(failed.Geometry < used.Geometry);

        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
            Sweep(context, new PhysicsQueryBudget(used.Discovery - 1, used.Candidates, used.Geometry),
                scratch, out hit, out _));
        Assert.Null(hit);
        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
            Sweep(context, new PhysicsQueryBudget(used.Discovery, used.Candidates - 1, used.Geometry),
                scratch, out hit, out _));
        Assert.Null(hit);
        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
            Sweep(context, new PhysicsQueryBudget(used.Discovery, 0, used.Geometry),
                scratch, out hit, out PhysicsQueryWork noCandidates));
        Assert.Null(hit);
        Assert.Equal(0UL, noCandidates.Candidates);

        Assert.Equal(PhysicsQueryExecutionStatus.ScratchCapacityExceeded,
            Sweep(context, exact, new PhysicsQueryScratch(512, 1), out hit, out _));
        Assert.Null(hit);

        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            Sweep(context, exact, scratch, out hit, out PhysicsQueryWork replay));
        Assert.Equal(expected, hit!.Value);
        Assert.Equal(used, replay);
    }

    [Fact]
    public void QueryWaitsForHostReconciliationAndDoesNotRunIt()
    {
        using GravitasWorldContext context = CreateContext();
        AddBoxFloor(context);
        context.Simulate();
        context.LateSimulate();
        Assert.True(context.World.TryAddGrid(
            new GridConfiguration(new Vector3d(8, 8, 8), new Vector3d(12, 12, 12)), out _));

        var scratch = new PhysicsQueryScratch(512, 1);
        var ample = new PhysicsQueryBudget(8192, 8192, 8192);
        for (int i = 0; i < 2; ++i)
        {
            Assert.Equal(PhysicsQueryExecutionStatus.WorldNotReady,
                Sweep(context, ample, scratch, out Physics3DHit? hit, out PhysicsQueryWork work));
            Assert.Null(hit);
            Assert.Equal(default, work);
        }

        context.Simulate();
        context.LateSimulate();
        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            Sweep(context, ample, scratch, out Physics3DHit? recovered, out _));
        Assert.Equal(Vector3d.Zero, recovered!.Value.Point);
    }

    [Fact]
    public void UnprovedGeometryReturnsNoHitAndAnExcludedTargetLeavesTheFloor()
    {
        using GravitasWorldContext context = CreateContext();
        AddBoxFloor(context);
        var sphere = new LSSphereCollider { Radius = Fixed64.Half };
        sphere.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        context.Simulate();
        context.LateSimulate();

        var scratch = new PhysicsQueryScratch(512, 2);
        var ample = new PhysicsQueryBudget(8192, 8192, 8192);
        Assert.Equal(PhysicsQueryExecutionStatus.UnsupportedGeometry,
            Sweep(context, ample, scratch, out Physics3DHit? hit, out _));
        Assert.Null(hit);

        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            context.Query3D.SweepSphere(Vector3d.Up, Fixed64.Half, Vector3d.Down, Fixed64.One,
                PhysicsLayerMask.All, sphere, ample, scratch, out hit, out _));
        Assert.Equal(Vector3d.Zero, hit!.Value.Point);
    }

    private static PhysicsQueryExecutionStatus Sweep(
        GravitasWorldContext context, PhysicsQueryBudget budget, PhysicsQueryScratch scratch,
        out Physics3DHit? hit, out PhysicsQueryWork work) =>
        context.Query3D.SweepSphere(Vector3d.Up, Fixed64.Half, Vector3d.Down, Fixed64.One,
            PhysicsLayerMask.All, null, budget, scratch, out hit, out work);

    private static GravitasWorldContext CreateContext()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.ThreeD;
        Assert.True(context.World.TryAddGrid(
            new GridConfiguration(new Vector3d(-4, -4, -4), new Vector3d(4, 4, 4)), out _));
        return context;
    }

    private static LSCuboidCollider AddBoxFloor(GravitasWorldContext context)
    {
        var floor = new LSCuboidCollider { Size = new Vector3d(3, 1, 3) };
        floor.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(-Vector3d.Up * Fixed64.Half, FixedQuaternion.Identity, Vector3d.One)));
        return floor;
    }
}
