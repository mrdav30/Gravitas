using System;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Constraints;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Serialization;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using GridForge.Grids;
using GridForge.Grids.Storage;
using GridForge.Grids.Topology;
using GridForge.Spatial;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class Physics2DSupportQueryTests
{
    public static TheoryData<GravitasSerializationTransport> Transports => GravitasSerializationTransportCases.All();

    [Theory]
    [MemberData(nameof(Transports))]
    public void BoundedSupport_ReplaysAfterRemovedMembershipsAndCheckpointRestore(GravitasSerializationTransport transport)
    {
        using var live = Physics2DTestWorld.CreateContext();
        using var restored = Physics2DTestWorld.CreateContext();
        SolidBody2D liveSource = Source(live);
        SolidBody2D restoredSource = Source(restored);
        var liveFloor = new LSAABBoxCollider2D(new Vector2d(3, 1));
        var restoredFloor = new LSAABBoxCollider2D(new Vector2d(3, 1));
        var floorPosition = new Vector2d(Fixed64.Zero, -Fixed64.Half);
        Initialize(live, liveFloor, floorPosition);
        Initialize(restored, restoredFloor, floorPosition);
        var liveSecond = new LSAABBoxCollider2D(new Vector2d(3, 1));
        var restoredSecond = new LSAABBoxCollider2D(new Vector2d(3, 1));
        Initialize(live, liveSecond, floorPosition);
        Initialize(restored, restoredSecond, floorPosition);
        var discarded = new LSAABBoxCollider2D[16];
        for (int i = 0; i < discarded.Length; i++)
        {
            discarded[i] = new LSAABBoxCollider2D(new Vector2d(3, 1));
            Initialize(live, discarded[i], floorPosition);
        }
        foreach (var collider in discarded)
            collider.Deactivate();
        liveFloor.LocalOffset = Vector2d.Right * (Fixed64)32;
        liveFloor.Simulate();
        liveFloor.LocalOffset = Vector2d.Zero;
        liveFloor.Simulate();
        var transient = new LSAABBoxCollider2D(Vector2d.One);
        Initialize(live, transient, Vector2d.Forward * (Fixed64)2);
        Assert.True(live.World.TryGetVoxel(Vector3d.Forward * (Fixed64)2, out var occupiedHistory));
        Assert.NotNull(occupiedHistory!.GetPartitionOrDefault<PhysicsPartition2D>());
        transient.Deactivate();
        var spatialTransient = new LSCuboidCollider { Size = Vector3d.One };
        spatialTransient.InitializeWithNoBody(new TestMatterAgent(live,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        Assert.True(live.World.TryGetVoxel(Vector3d.Zero, out var spatialHistory));
        Assert.NotNull(spatialHistory!.GetPartitionOrDefault<PhysicsPartition>());
        spatialTransient.Deactivate();
        live.Simulate();
        live.LateSimulate();
        GravitasSerializationHarness.Populate(restoredSource,
            GravitasSerializationHarness.Serialize(liveSource, transport), transport);
        GravitasSerializationHarness.Populate(restoredFloor,
            GravitasSerializationHarness.Serialize(liveFloor, transport), transport);
        GravitasSerializationHarness.Populate(restoredSecond,
            GravitasSerializationHarness.Serialize(liveSecond, transport), transport);
        restored.Simulate();
        restored.LateSimulate();
        Assert.True(restored.World.TryGetVoxel(Vector3d.Forward * (Fixed64)2, out var untouched));
        Assert.True(Assert.IsType<PhysicsPartition2D>(occupiedHistory.GetPartitionOrDefault<PhysicsPartition2D>()).IsEmpty);
        Assert.Null(untouched!.GetPartitionOrDefault<PhysicsPartition2D>());
        Assert.True(restored.World.TryGetVoxel(Vector3d.Zero, out var untouchedSpatial));
        Assert.True(Assert.IsType<PhysicsPartition>(spatialHistory.GetPartitionOrDefault<PhysicsPartition>()).IsEmpty);
        Assert.Null(untouchedSpatial!.GetPartitionOrDefault<PhysicsPartition>());

        var liveRequest = Request(liveSource, Vector2d.Forward * (Fixed64)2, (Fixed64)2);
        var restoredRequest = Request(restoredSource, Vector2d.Forward * (Fixed64)2, (Fixed64)2);
        var scratch = new PhysicsQueryScratch(128, 16);
        var ample = new PhysicsQueryBudget(8192, 8192, 8192);
        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            restored.Query2D.QuerySupport(restoredRequest, ample, scratch, out var hit, out var expected));
        Assert.Same(restoredFloor, hit.GetValueOrDefault().Hit.Collider);
        var exact = new PhysicsQueryBudget(expected.Discovery, expected.Candidates, expected.Geometry);
        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            live.Query2D.QuerySupport(liveRequest, exact, scratch, out var replay, out var used));
        Assert.Equal(expected, used);
        Assert.Same(liveFloor, replay.GetValueOrDefault().Hit.Collider);
        Assert.Equal(hit!.Value.Hit.Point, replay!.Value.Hit.Point);
        Assert.Equal(hit.Value.Hit.Distance, replay.Value.Hit.Distance);
        Assert.Equal(hit.Value.Hit.Normal, replay.Value.Hit.Normal);

        foreach (var oneBelow in new[]
        {
            new PhysicsQueryBudget(expected.Discovery - 1, expected.Candidates, expected.Geometry),
            new PhysicsQueryBudget(expected.Discovery, expected.Candidates - 1, expected.Geometry),
            new PhysicsQueryBudget(expected.Discovery, expected.Candidates, expected.Geometry - 1)
        })
        {
            Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
                live.Query2D.QuerySupport(liveRequest, oneBelow, scratch, out replay, out used));
            Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
                restored.Query2D.QuerySupport(restoredRequest, oneBelow, scratch, out hit, out var restoredWork));
            Assert.Equal(restoredWork, used);
            if (oneBelow.GeometryLimit < expected.Geometry)
                Assert.InRange(used.Geometry, 1UL, expected.Geometry - 1);
            Assert.Null(hit);
            Assert.Null(replay);
        }
    }

    [Fact]
    public void BoundedFloor_ReplaysExactWitnessAndStopsBeforeUnchargedWork()
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        var floor = new LSAABBoxCollider2D(new Vector2d(12, 1));
        Initialize(context, floor, new Vector2d(Fixed64.Zero, -Fixed64.Half));
        var request = Request(source, new Vector2d(Fixed64.Zero, Fixed64.One), Fixed64.One);
        context.Simulate();
        context.LateSimulate();
        context.Query2D.QuerySupport(request, out var legacy).Should().Be(Physics2DSupportQueryStatus.Found);

        var scratch = new PhysicsQueryScratch(128, 8);
        var ample = new PhysicsQueryBudget(1024, 1024, 1024);
        context.Query2D.QuerySupport(request, ample, scratch, out var hit, out var work)
            .Should().Be(PhysicsQueryExecutionStatus.Completed);
        hit!.Value.Hit.Collider.Should().BeSameAs(legacy.Hit.Collider);
        hit.Value.Hit.Point.Should().Be(legacy.Hit.Point);
        hit.Value.Hit.Normal.Should().Be(legacy.Hit.Normal);
        hit.Value.Hit.Distance.Should().Be(legacy.Hit.Distance);
        var exact = new PhysicsQueryBudget(work.Discovery, work.Candidates, work.Geometry);

        // Allow complete voxel discovery with no remaining partition lookup work.
        var voxels = new Voxel?[scratch.TraversalCapacity];
        Assert.Equal(GridBoundedVoxelStatus.Completed, context.World.CollectCoveredVoxelsBounded(
            new Vector3d(-Fixed64.Half, Fixed64.Zero, -Fixed64.Half),
            new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.One + Fixed64.Half),
            true, ample.DiscoveryLimit, voxels, out ulong voxelDiscovery, out int voxelCount, out _));
        Assert.True(voxelCount > 0);
        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
            context.Query2D.QuerySupport(request, new PhysicsQueryBudget(voxelDiscovery, work.Candidates, work.Geometry),
                scratch, out hit, out var discoveryOnly));
        Assert.Null(hit);
        Assert.Equal(new PhysicsQueryWork(voxelDiscovery, 0, 0), discoveryOnly);

        foreach (var shortBudget in new[]
        {
            new PhysicsQueryBudget(work.Discovery - 1, work.Candidates, work.Geometry),
            new PhysicsQueryBudget(work.Discovery, work.Candidates - 1, work.Geometry),
            new PhysicsQueryBudget(work.Discovery, work.Candidates, work.Geometry - 1)
        })
        {
            context.Query2D.QuerySupport(request, shortBudget, scratch, out hit, out var shortWork)
                .Should().Be(PhysicsQueryExecutionStatus.WorkBudgetExceeded);
            hit.Should().BeNull();
            shortWork.Discovery.Should().BeLessThanOrEqualTo(shortBudget.DiscoveryLimit);
            shortWork.Candidates.Should().BeLessThanOrEqualTo(shortBudget.CandidateLimit);
            shortWork.Geometry.Should().BeLessThanOrEqualTo(shortBudget.GeometryLimit);
        }
        context.Query2D.QuerySupport(request, exact, new PhysicsQueryScratch(128, 0), out hit, out _)
            .Should().Be(PhysicsQueryExecutionStatus.ScratchCapacityExceeded);
        hit.Should().BeNull();
        context.Query2D.QuerySupport(request, exact, new PhysicsQueryScratch(0, 8), out hit, out _)
            .Should().Be(PhysicsQueryExecutionStatus.ScratchCapacityExceeded);
        hit.Should().BeNull();
        context.Query2D.QuerySupport(request, exact, scratch, out hit, out var used)
            .Should().Be(PhysicsQueryExecutionStatus.Completed);
        used.Should().Be(work);
        hit!.Value.Hit.Point.Should().Be(legacy.Hit.Point);
    }

    [Fact]
    public void BoundedFloor_WaitsForTheCompletedHostStepAndDoesNotAllocateWhenWarm()
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        var floor = new LSAABBoxCollider2D(new Vector2d(12, 1));
        Initialize(context, floor, new Vector2d(Fixed64.Zero, -Fixed64.Half));
        var request = Request(source, new Vector2d(Fixed64.Zero, Fixed64.One), Fixed64.One);
        var scratch = new PhysicsQueryScratch(128, 8);
        var ample = new PhysicsQueryBudget(1024, 1024, 1024);
        context.Simulate();
        context.Query2D.QuerySupport(request, ample, scratch, out var hit, out var work)
            .Should().Be(PhysicsQueryExecutionStatus.WorldNotReady);
        hit.Should().BeNull();
        work.Should().Be(default(PhysicsQueryWork));
        context.LateSimulate();

        for (int i = 0; i < 32; i++)
            Assert.Equal(PhysicsQueryExecutionStatus.Completed,
                context.Query2D.QuerySupport(request, ample, scratch, out _, out _));
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 32; i++)
            if (context.Query2D.QuerySupport(request, ample, scratch, out hit, out _) != PhysicsQueryExecutionStatus.Completed
                || hit.GetValueOrDefault().Hit.Collider != floor)
                throw new InvalidOperationException("The bounded floor query must complete during the allocation sample.");
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        allocated.Should().Be(0);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(256)]
    [InlineData(257)]
    [InlineData(4294967296L)]
    public void TouchingFloor_PreservesWitnessForEveryPositiveDistance(long rawDistance)
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        var floor = new LSAABBoxCollider2D(new Vector2d(12, 1));
        Initialize(context, floor, new Vector2d(Fixed64.Zero, -Fixed64.Half));
        var request = Request(source, new Vector2d(Fixed64.Zero, Fixed64.Half), Fixed64.FromRaw(rawDistance));

        context.Query2D.QuerySupport(request, out var support).Should().Be(Physics2DSupportQueryStatus.Found);

        support.Hit.Collider.Should().BeSameAs(floor);
        support.Hit.Point.Should().Be(Vector2d.Zero);
        support.Hit.Normal.Should().Be(Vector2d.Forward);
        support.Hit.Distance.Should().Be(Fixed64.Zero);
        support.SampledFrame.Should().Be(context.FrameCount);
        support.IsCurrentLifetime.Should().BeTrue();
        context.Reset();
        support.IsCurrentLifetime.Should().BeFalse();
    }

    [Fact]
    public void CompoundWallBeforeFloor_DoesNotHideSupport()
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.AABBox(new Vector2d(1, 4), new Vector2d(-Fixed64.Half, Fixed64.Zero)),
            CompoundColliderPart2D.AABBox(new Vector2d(4, 1), new Vector2d(Fixed64.Zero, -Fixed64.Half)));
        Initialize(context, compound, Vector2d.Zero);
        var request = Request(source, new Vector2d(Fixed64.Half, Fixed64.Half), Fixed64.One);

        context.Query2D.QuerySupport(request, out var support).Should().Be(Physics2DSupportQueryStatus.Found);

        support.Hit.Collider.Should().BeSameAs(compound);
        support.Hit.Point.Should().Be(new Vector2d(Fixed64.Half, Fixed64.Zero));
        support.Hit.Normal.Should().Be(Vector2d.Forward);

        AssertReadyBoundedWitness(context, request);
    }

    [Theory]
    [InlineData(ColliderType2D.Circle)]
    [InlineData(ColliderType2D.Capsule)]
    [InlineData(ColliderType2D.ConvexPolygon)]
    [InlineData(ColliderType2D.Compound)]
    public void BoundedSupport_MatchesNativeShapeWitnessAndRejectsIncompleteGeometry(ColliderType2D shape)
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        Vector2d[] vertices =
        [
            new(-2, -1), new(2, -1), new(3, 0), new(3, 1),
            new(2, 2), new(-2, 2), new(-3, 1), new(-3, 0)
        ];
        LSCollider2D target = shape switch
        {
            ColliderType2D.Circle => new LSCircleCollider2D(Fixed64.Half),
            ColliderType2D.Capsule => new LSCapsuleCollider2D(Fixed64.Half, Fixed64.Two),
            ColliderType2D.ConvexPolygon => new LSPolygonCollider2D(vertices),
            _ => new LSCompoundCollider2D(
                CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Right * (Fixed64)4),
                CompoundColliderPart2D.ConvexPolygon(vertices, Vector2d.Zero))
        };
        Initialize(context, target, Vector2d.Zero);
        context.Simulate();
        context.LateSimulate();
        Fixed64 top = shape switch
        {
            ColliderType2D.Circle => Fixed64.Half,
            ColliderType2D.Capsule => Fixed64.One,
            _ => Fixed64.Two
        };
        var scratch = new PhysicsQueryScratch(128, 8);
        var ample = new PhysicsQueryBudget(8192, 8192, 8192);
        foreach (Fixed64 travel in new[] { Fixed64.Zero, Fixed64.One })
        {
            var request = Request(source, Vector2d.Forward * (top + Fixed64.Half + travel), Fixed64.One);
            Assert.Equal(Physics2DSupportQueryStatus.Found, context.Query2D.QuerySupport(request, out var expected));
            Assert.Same(target, expected.Hit.Collider);
            Assert.Equal(travel, expected.Hit.Distance);

            Assert.Equal(PhysicsQueryExecutionStatus.Completed,
                context.Query2D.QuerySupport(request, ample, scratch, out var actual, out var used));
            AssertSupportWitness(expected, actual);

            var shortGeometry = new PhysicsQueryBudget(used.Discovery, used.Candidates, used.Geometry - 1);
            Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
                context.Query2D.QuerySupport(request, shortGeometry, scratch, out actual, out var partial));
            Assert.Null(actual);
            Assert.True(partial.Geometry < used.Geometry);
            if (shape == ColliderType2D.Compound)
            {
                Assert.True(partial.Geometry > 0);
                Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
                    context.Query2D.QuerySupport(request, new PhysicsQueryBudget(used.Discovery, used.Candidates, 0),
                        scratch, out actual, out var beforePartVisits));
                Assert.Null(actual);
                Assert.Equal(0UL, beforePartVisits.Geometry);
            }

            var exact = new PhysicsQueryBudget(used.Discovery, used.Candidates, used.Geometry);
            Assert.Equal(PhysicsQueryExecutionStatus.Completed,
                context.Query2D.QuerySupport(request, exact, scratch, out actual, out var replayed));
            Assert.Equal(used, replayed);
            AssertSupportWitness(expected, actual);
        }
    }

    [Theory]
    [InlineData(GridTopologyKind.RectangularPrism, HexOrientation.PointyTop, GridStorageKind.Sparse)]
    [InlineData(GridTopologyKind.HexPrism, HexOrientation.PointyTop, GridStorageKind.Dense)]
    [InlineData(GridTopologyKind.HexPrism, HexOrientation.FlatTop, GridStorageKind.Dense)]
    [InlineData(GridTopologyKind.HexPrism, HexOrientation.PointyTop, GridStorageKind.Sparse)]
    [InlineData(GridTopologyKind.HexPrism, HexOrientation.FlatTop, GridStorageKind.Sparse)]
    public void BoundedSupport_MatchesPhysicalPresenceOnSparseAndHexGrids(
        GridTopologyKind topology, HexOrientation orientation, GridStorageKind storage)
    {
        using var context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        var metrics = topology == GridTopologyKind.HexPrism
            ? GridTopologyMetrics.Hex(Fixed64.One, Fixed64.One, orientation)
            : default;
        Assert.True(context.World.TryAddGrid(new GridConfiguration(Vector3d.Zero, new Vector3d(24, 0, 24),
            topologyKind: topology, topologyMetrics: metrics, storageKind: storage), out ushort gridIndex));
        var grid = context.World.ActiveGrids[gridIndex];
        var floorIndex = new VoxelIndex(4, 0, 4);
        var neighborIndex = new VoxelIndex(5, 0, 4);
        if (storage == GridStorageKind.Sparse)
        {
            Assert.True(grid.TryAddVoxel(floorIndex, out _));
            Assert.True(grid.TryAddVoxel(neighborIndex, out _));
        }
        Assert.True(grid.TryGetVoxel(floorIndex, out var floorVoxel));
        Assert.True(grid.TryGetVoxel(neighborIndex, out var neighbor));
        Vector3d floorPosition = Vector3d.Midpoint(floorVoxel!.WorldPosition, neighbor!.WorldPosition);
        if (storage == GridStorageKind.Sparse)
        {
            Assert.True(grid.TryRemoveVoxel(floorIndex));
            Assert.True(grid.TryRemoveVoxel(neighborIndex));
        }

        SolidBody2D source = Source(context);
        var floorCenter = new Vector2d(floorPosition.X, floorPosition.Z);
        var floor = new LSAABBoxCollider2D(Vector2d.One);
        Initialize(context, floor, floorCenter);
        var request = Request(source, floorCenter + Vector2d.Forward * (Fixed64.One + Fixed64.Half), Fixed64.One);
        var scratch = new PhysicsQueryScratch(128, 8);
        var ample = new PhysicsQueryBudget(8192, 8192, 8192);
        context.Simulate();
        context.LateSimulate();
        if (storage == GridStorageKind.Sparse)
        {
            Assert.Equal(PhysicsQueryExecutionStatus.Completed,
                context.Query2D.QuerySupport(request, ample, scratch, out var absent, out var missWork));
            Assert.Null(absent);
            Assert.Equal(0UL, missWork.Geometry);
            Assert.Equal(Physics2DSupportQueryStatus.NoSupport, context.Query2D.QuerySupport(request, out _));

            PhysicsQueryExecutionStatus? callbackStatus = null;
            context.World.OnActiveGridChange += change =>
            {
                if (change.ChangeKind == GridEventKind.SparseVoxelAdded)
                    callbackStatus = context.Query2D.QuerySupport(request, ample, scratch, out _, out _);
            };
            Assert.True(grid.TryAddVoxel(floorIndex, out _));
            Assert.Equal(PhysicsQueryExecutionStatus.WorldNotReady, callbackStatus);
            Assert.Equal(PhysicsQueryExecutionStatus.WorldNotReady,
                context.Query2D.QuerySupport(request, ample, scratch, out absent, out var pendingWork));
            Assert.Null(absent);
            Assert.Equal(default, pendingWork);
            context.Simulate();
            context.LateSimulate();
        }

        Assert.Equal(Physics2DSupportQueryStatus.Found, context.Query2D.QuerySupport(request, out var expected));
        Assert.Same(floor, expected.Hit.Collider);
        Assert.Equal(Fixed64.Half, expected.Hit.Distance);
        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            context.Query2D.QuerySupport(request, ample, scratch, out var actual, out var used));
        AssertSupportWitness(expected, actual);

        var shortDiscovery = new PhysicsQueryBudget(used.Discovery - 1, used.Candidates, used.Geometry);
        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
            context.Query2D.QuerySupport(request, shortDiscovery, scratch, out actual, out var partial));
        Assert.Null(actual);
        Assert.True(partial.Discovery < used.Discovery);

        var exact = new PhysicsQueryBudget(used.Discovery, used.Candidates, used.Geometry);
        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            context.Query2D.QuerySupport(request, exact, scratch, out actual, out var replayed));
        Assert.Equal(used, replayed);
        AssertSupportWitness(expected, actual);
    }

    private static void AssertReadyBoundedWitness(GravitasWorldContext context, in Physics2DSupportQuery request)
    {
        context.Simulate();
        context.LateSimulate();
        Assert.Equal(Physics2DSupportQueryStatus.Found, context.Query2D.QuerySupport(request, out var expected));
        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            context.Query2D.QuerySupport(request, new PhysicsQueryBudget(8192, 8192, 8192),
                new PhysicsQueryScratch(128, 8), out var bounded, out _));
        AssertSupportWitness(expected, bounded);
    }

    private static void AssertSupportWitness(in Physics2DSupportHit expected, Physics2DSupportHit? actual)
    {
        Assert.True(actual.HasValue);
        var support = actual.Value;
        Assert.Same(expected.Hit.Collider, support.Hit.Collider);
        Assert.Equal(expected.Hit.Point, support.Hit.Point);
        Assert.Equal(expected.Hit.Anchor, support.Hit.Anchor);
        Assert.Equal(expected.Hit.Normal, support.Hit.Normal);
        Assert.Equal(expected.Hit.Distance, support.Hit.Distance);
        Assert.Equal(expected.SampledFrame, support.SampledFrame);
        Assert.True(support.IsCurrentLifetime);
    }

    [Fact]
    public void DownwardProbe_ReportsTravelAndTargetPoint()
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        var floor = new LSAABBoxCollider2D(new Vector2d(12, 1));
        Initialize(context, floor, new Vector2d(Fixed64.Zero, -Fixed64.Half));
        var request = Request(source, new Vector2d(Fixed64.Zero, Fixed64.Two), (Fixed64)3);

        context.Query2D.QuerySupport(request, out var support).Should().Be(Physics2DSupportQueryStatus.Found);

        support.Hit.Distance.Should().Be(Fixed64.One + Fixed64.Half);
        support.Hit.Point.Should().Be(Vector2d.Zero);
    }

    private static Physics2DSupportQuery Request(SolidBody2D source, Vector2d center, Fixed64 distance) =>
        new(source.Collider, center, Fixed64.Half, Vector2d.Forward, distance, Fixed64.Half, PhysicsLayerMask.All);

    [Theory]
    [InlineData(1, ColliderType2D.AABox)]
    [InlineData(256, ColliderType2D.AABox)]
    [InlineData(257, ColliderType2D.AABox)]
    [InlineData(1, ColliderType2D.Circle)]
    [InlineData(256, ColliderType2D.Circle)]
    [InlineData(257, ColliderType2D.Circle)]
    [InlineData(1, ColliderType2D.Capsule)]
    [InlineData(256, ColliderType2D.Capsule)]
    [InlineData(257, ColliderType2D.Capsule)]
    public void SeparatedTinyProbe_ReachesSurfaceAtEndpoint(long rawDistance, ColliderType2D shape)
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        LSCollider2D target = shape switch
        {
            ColliderType2D.Circle => new LSCircleCollider2D(Fixed64.Half),
            ColliderType2D.Capsule => new LSCapsuleCollider2D(Fixed64.Half, Fixed64.Two),
            _ => new LSAABBoxCollider2D(new Vector2d(12, 1))
        };
        Initialize(context, target, Vector2d.Zero);
        Fixed64 top = shape == ColliderType2D.Capsule ? Fixed64.One : Fixed64.Half;
        Fixed64 distance = Fixed64.FromRaw(rawDistance);
        var request = Request(source, new Vector2d(Fixed64.Zero, top + Fixed64.Half + distance), distance);

        context.Query2D.QuerySupport(request, out var support).Should().Be(Physics2DSupportQueryStatus.Found);
        support.Hit.Distance.Should().Be(distance);
        support.Hit.Point.Should().Be(new Vector2d(Fixed64.Zero, top));
        support.Hit.Normal.Should().Be(Vector2d.Forward);
        AssertReadyBoundedWitness(context, request);
    }

    [Theory]
    [InlineData(0)] // Trigger.
    [InlineData(1)] // Dynamic body.
    [InlineData(2)] // Source's ignored layers.
    [InlineData(3)] // Target's ignored layers.
    [InlineData(4)] // Query's include mask.
    [InlineData(5)] // Layer matrix.
    [InlineData(6)] // Hierarchy sibling.
    public void IneligibleNearSurface_DoesNotHideFarSupport(int exclusion)
    {
        using var context = Physics2DTestWorld.CreateContext();
        if (exclusion == 5)
            context.ApplySettings(new PhysicsSettings(4, new[,]
            {
                { true, true, false }, { true, true, true }, { false, true, true }
            })
            { RuntimeMode = PhysicsRuntimeMode.TwoD });
        SolidBody2D source = Source(context);
        var near = new LSAABBoxCollider2D(new Vector2d(12, 1));
        near.Layer = new PhysicsLayer(2);
        if (exclusion == 0)
            near.IsTrigger = true;
        if (exclusion == 1)
            new SolidBody2D(new TestMatterAgent(context: context), near).Initialize(Vector2d.Zero);
        else
            Initialize(context, near, Vector2d.Zero);
        if (exclusion == 2)
            source.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(near.Layer);
        if (exclusion == 3)
            near.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(source.Collider.Layer);
        if (exclusion == 6)
            near.SetParent(source.Collider);
        var floor = new LSAABBoxCollider2D(new Vector2d(12, 1));
        Initialize(context, floor, new Vector2d(Fixed64.Zero, -Fixed64.Two));
        var request = new Physics2DSupportQuery(source.Collider, Vector2d.Forward * (Fixed64)2,
            Fixed64.Half, Vector2d.Forward, (Fixed64)4, Fixed64.Half,
            exclusion == 4 ? PhysicsLayerMask.FromLayer(floor.Layer) : PhysicsLayerMask.All);

        context.Query2D.QuerySupport(request, out var support).Should().Be(Physics2DSupportQueryStatus.Found);
        support.Hit.Collider.Should().BeSameAs(floor);
        support.Hit.Distance.Should().Be((Fixed64)3);
        AssertReadyBoundedWitness(context, request);
    }

    [Theory]
    [InlineData(PhysicsMobilityType.Static)]
    [InlineData(PhysicsMobilityType.Kinematic)]
    public void BodyBackedSupport_PreservesRegistrationIdentity(PhysicsMobilityType motionType)
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        var floor = new SolidBody2D(new TestMatterAgent(context: context), new LSAABBoxCollider2D(new Vector2d(12, 1)));
        floor.Initialize(Vector2d.Zero, motionType: motionType);
        var request = Request(source, Vector2d.Forward, Fixed64.One);
        context.Query2D.QuerySupport(request, out var support).Should().Be(Physics2DSupportQueryStatus.Found);
        support.Hit.Body.Should().BeSameAs(floor);
        support.IsCurrentLifetime.Should().BeTrue();
        floor.Deactivate();
        support.IsCurrentLifetime.Should().BeFalse();
        floor.Initialize(Vector2d.Zero, motionType: motionType);
        support.IsCurrentLifetime.Should().BeFalse();
        context.Query2D.QuerySupport(request, out var fresh).Should().Be(Physics2DSupportQueryStatus.Found);
        fresh.IsCurrentLifetime.Should().BeTrue();
        AssertReadyBoundedWitness(context, request);
        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
            context.Query2D.QuerySupport(request, new PhysicsQueryBudget(8192, 0, 8192),
                new PhysicsQueryScratch(128, 8), out var bounded, out var work));
        Assert.Null(bounded);
        Assert.Equal(0UL, work.Candidates);
        Assert.Equal(0UL, work.Geometry);
    }

    [Fact]
    public void OpenStep_IsNotACompleteNoSupportResult()
    {
        using var context = Physics2DTestWorld.CreateContext();
        context.Environment.Gravity = Fixed64.Zero;
        SolidBody2D source = Source(context);
        var floor = new SolidBody2D(new TestMatterAgent(context: context), new LSAABBoxCollider2D(new Vector2d(12, 1)));
        floor.Initialize(Vector2d.Zero, motionType: PhysicsMobilityType.Dynamic);
        floor.UseManualGrounding();
        var request = Request(source, Vector2d.Forward, Fixed64.One);
        context.Simulate();
        context.Query2D.QuerySupport(request, out var pending).Should().Be(Physics2DSupportQueryStatus.WorldNotReady);
        pending.IsCurrentLifetime.Should().BeFalse();
        context.LateSimulate();
        context.Query2D.QuerySupport(request, out var absent).Should().Be(Physics2DSupportQueryStatus.NoSupport);
        absent.IsCurrentLifetime.Should().BeFalse();

        var scratch = new PhysicsQueryScratch(128, 8);
        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            context.Query2D.QuerySupport(request, new PhysicsQueryBudget(8192, 8192, 8192),
                scratch, out var bounded, out var completeWork));
        Assert.Null(bounded);
        Assert.True(completeWork.Candidates > 0);
        Assert.Equal(0UL, completeWork.Geometry);
        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
            context.Query2D.QuerySupport(request, new PhysicsQueryBudget(completeWork.Discovery, 0, 8192),
                scratch, out bounded, out var partialWork));
        Assert.Null(bounded);
        Assert.Equal(0UL, partialWork.Candidates);
        Assert.Equal(0UL, partialWork.Geometry);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void UnrepresentableProbe_ClearsResult(bool endpoint)
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        var request = Request(source,
            endpoint ? new Vector2d(Fixed64.Zero, Fixed64.MinValue) : new Vector2d(Fixed64.MaxValue, Fixed64.Zero), Fixed64.One);
        context.Query2D.QuerySupport(request, out var support).Should().Be(Physics2DSupportQueryStatus.Unrepresentable);
        support.IsCurrentLifetime.Should().BeFalse();
        support.Hit.Collider.Should().BeNull();
        Assert.Equal(PhysicsQueryExecutionStatus.Unrepresentable,
            context.Query2D.QuerySupport(request, default, new PhysicsQueryScratch(0, 0), out var bounded, out var used));
        Assert.Null(bounded);
        Assert.Equal(default, used);
    }

    [Fact]
    public void BoundedSupport_RejectsNonpureRuntimeBeforeCharging()
    {
        using var context = Physics2DTestWorld.CreateContext();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Both;
        SolidBody2D source = Source(context);
        var request = Request(source, Vector2d.Forward, Fixed64.One);

        Assert.Equal(PhysicsQueryExecutionStatus.UnsupportedRuntimeMode,
            context.Query2D.QuerySupport(request, default, new PhysicsQueryScratch(0, 0), out var bounded, out var used));
        Assert.Null(bounded);
        Assert.Equal(default, used);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void NegativeScalarFace_RejectsEndpointOrRadiusOverflow(bool endpoint)
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        var request = new Physics2DSupportQuery(source.Collider,
            new Vector2d(Fixed64.MinValue, Fixed64.Zero), Fixed64.Half,
            endpoint ? Vector2d.Right : Vector2d.Forward, Fixed64.One, Fixed64.Half, PhysicsLayerMask.All);
        context.Query2D.QuerySupport(request, out var support).Should().Be(Physics2DSupportQueryStatus.Unrepresentable);
        support.Hit.Collider.Should().BeNull();
    }

    [Fact]
    public void CompoundWithoutEligibleNormal_ClearsHit()
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        var wall = new LSCompoundCollider2D(
            CompoundColliderPart2D.AABBox(new Vector2d(1, 4), new Vector2d(-Fixed64.Half, Fixed64.Zero)));
        Initialize(context, wall, Vector2d.Zero);
        var request = Request(source, new Vector2d(Fixed64.Half, Fixed64.Half), Fixed64.One);
        context.Query2D.QuerySupport(request, out var support).Should().Be(Physics2DSupportQueryStatus.NoSupport);
        support.Hit.Collider.Should().BeNull();
        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            context.Query2D.QuerySupport(request, new PhysicsQueryBudget(8192, 8192, 8192),
                new PhysicsQueryScratch(128, 8), out var bounded, out var work));
        Assert.Null(bounded);
        Assert.True(work.Geometry > 0);
    }

    [Fact]
    public void Request_RejectsInvalidInputAndNormalizesUp()
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        Assert.Throws<ArgumentNullException>(() => new Physics2DSupportQuery(null!, Vector2d.Zero, Fixed64.One,
            Vector2d.Forward, Fixed64.One, Fixed64.Half, PhysicsLayerMask.All));
        foreach (Fixed64 invalid in new[] { Fixed64.Zero, -Fixed64.One })
        {
            Assert.Throws<ArgumentException>(() => new Physics2DSupportQuery(source.Collider, Vector2d.Zero, invalid,
                Vector2d.Forward, Fixed64.One, Fixed64.Half, PhysicsLayerMask.All));
            Assert.Throws<ArgumentException>(() => new Physics2DSupportQuery(source.Collider, Vector2d.Zero, Fixed64.One,
                Vector2d.Forward, invalid, Fixed64.Half, PhysicsLayerMask.All));
        }
        foreach (Fixed64 invalid in new[] { Fixed64.Zero, Fixed64.Two })
            Assert.Throws<ArgumentException>(() => new Physics2DSupportQuery(source.Collider, Vector2d.Zero, Fixed64.One,
                Vector2d.Forward, Fixed64.One, invalid, PhysicsLayerMask.All));
        Assert.Throws<ArgumentException>(() => new Physics2DSupportQuery(source.Collider, Vector2d.Zero, Fixed64.One,
            Vector2d.Zero, Fixed64.One, Fixed64.Half, PhysicsLayerMask.All));
        var request = new Physics2DSupportQuery(source.Collider, Vector2d.Zero, Fixed64.One,
            Vector2d.Forward * (Fixed64)7, Fixed64.One, Fixed64.Half, PhysicsLayerMask.All);
        request.Up.Should().Be(Vector2d.Forward);
        Assert.Throws<ArgumentNullException>(() => context.Query2D.QuerySupport(default, out _));
        source.Deactivate();
        Assert.Throws<ArgumentException>(() => context.Query2D.QuerySupport(request, out _));
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void BoundedSupport_ReplaysAfterRemovedLinksAndCheckpointRestore(GravitasSerializationTransport transport)
    {
        using var live = Physics2DTestWorld.CreateContext();
        using var restored = Physics2DTestWorld.CreateContext();
        var first = CreateLinkedFloorScene(live);
        var second = CreateLinkedFloorScene(restored);
        var discarded = new SolidBody2D[32];
        for (int i = 0; i < discarded.Length; i++)
        {
            var body = new SolidBody2D(new TestMatterAgent(context: live), new LSCircleCollider2D(Fixed64.Half));
            body.Initialize(new Vector2d(15, 15), motionType: PhysicsMobilityType.Kinematic);
            discarded[i] = body;
            Link(live, first.Source, body);
        }
        foreach (var body in discarded)
            body.Deactivate();
        Assert.Equal(1, live.Constraints2D.RegisteredJointCount);
        GravitasSerializationHarness.Populate(second.Joint,
            GravitasSerializationHarness.Serialize(first.Joint, transport), transport);
        live.Simulate();
        live.LateSimulate();
        restored.Simulate();
        restored.LateSimulate();

        var liveRequest = Request(first.Source, Vector2d.Forward, Fixed64.One);
        var restoredRequest = Request(second.Source, Vector2d.Forward, Fixed64.One);
        var scratch = new PhysicsQueryScratch(128, 4);
        var ample = new PhysicsQueryBudget(8192, 8192, 8192);
        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            restored.Query2D.QuerySupport(restoredRequest, ample, scratch, out var hit, out var expected));
        Assert.Same(second.Floor, hit.GetValueOrDefault().Hit.Collider);
        var exact = new PhysicsQueryBudget(expected.Discovery, expected.Candidates, expected.Geometry);
        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            live.Query2D.QuerySupport(liveRequest, exact, scratch, out var replay, out var used));
        Assert.Equal(expected, used);
        Assert.Same(first.Floor, replay.GetValueOrDefault().Hit.Collider);
        Assert.Equal(hit!.Value.Hit.Point, replay!.Value.Hit.Point);
        Assert.Equal(hit.Value.Hit.Distance, replay.Value.Hit.Distance);
        Assert.Equal(hit.Value.Hit.Normal, replay.Value.Hit.Normal);

        // The kinematic membership fits. The first linked-pair comparison does not.
        var beforePairFilter = new PhysicsQueryBudget(expected.Discovery, 1, expected.Geometry);
        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
            live.Query2D.QuerySupport(liveRequest, beforePairFilter, scratch, out replay, out used));
        Assert.Equal(1UL, used.Candidates);
        Assert.Equal(0UL, used.Geometry);
        Assert.Null(replay);
        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
            restored.Query2D.QuerySupport(restoredRequest, beforePairFilter, scratch, out var partialHit, out var partialWork));
        Assert.Equal(used, partialWork);
        Assert.Null(partialHit);

        var oneBelow = new PhysicsQueryBudget(expected.Discovery, expected.Candidates - 1, expected.Geometry);
        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
            live.Query2D.QuerySupport(liveRequest, oneBelow, scratch, out replay, out used));
        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
            restored.Query2D.QuerySupport(restoredRequest, oneBelow, scratch, out hit, out expected));
        Assert.Equal(expected, used);
        Assert.Null(hit);
        Assert.Null(replay);

        static (SolidBody2D Source, LSAABBoxCollider2D Floor, Joint2D Joint)
            CreateLinkedFloorScene(GravitasWorldContext context)
        {
            context.Environment.Gravity = Fixed64.Zero;
            var source = Source(context);
            source.SetMotionType(PhysicsMobilityType.Dynamic);
            source.UseManualGrounding();
            var floorPosition = new Vector2d(Fixed64.Zero, -Fixed64.Half);
            var linked = new SolidBody2D(new TestMatterAgent(context: context),
                new LSAABBoxCollider2D(new Vector2d(3, 1)));
            linked.Initialize(floorPosition, motionType: PhysicsMobilityType.Kinematic);
            var floor = new LSAABBoxCollider2D(new Vector2d(3, 1));
            Initialize(context, floor, floorPosition);
            return (source, floor, Link(context, source, linked));
        }

        static Joint2D Link(GravitasWorldContext context, SolidBody2D source, SolidBody2D target)
        {
            Joint2D joint = context.Constraints2D.RegisterJoint(new JointDefinition2D(source, target,
                JointFrame2D.Identity, JointFrame2D.Identity, JointType2D.Pin, JointLimit2D.Unrestricted,
                JointMotor2D.Disabled, JointCollisionPolicy.SuppressLinked));
            joint.IsEnabled = false;
            return joint;
        }
    }

    [Fact]
    public void LinkedSupport_UsesPhysicalJointExclusion()
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        source.SetMotionType(PhysicsMobilityType.Dynamic);
        var floor = new SolidBody2D(new TestMatterAgent(context: context), new LSAABBoxCollider2D(new Vector2d(12, 1)));
        floor.Initialize(Vector2d.Zero, motionType: PhysicsMobilityType.Kinematic);
        var request = Request(source, Vector2d.Forward, Fixed64.One);
        context.Query2D.QuerySupport(request, out _).Should().Be(Physics2DSupportQueryStatus.Found);
        Joint2D joint = context.Constraints2D.RegisterJoint(new JointDefinition2D(source, floor,
            JointFrame2D.Identity, JointFrame2D.Identity, JointType2D.Pin, JointLimit2D.Unrestricted,
            JointMotor2D.Disabled, JointCollisionPolicy.SuppressLinked));
        context.Query2D.QuerySupport(request, out _).Should().Be(Physics2DSupportQueryStatus.NoSupport);
        context.Constraints2D.RemoveJoint(joint.Id).Should().BeTrue();
        context.Query2D.QuerySupport(request, out _).Should().Be(Physics2DSupportQueryStatus.Found);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SupportRanking_UsesDistanceThenOwnerId(bool closerSecond)
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        var first = new LSAABBoxCollider2D(new Vector2d(12, 1));
        var second = new LSAABBoxCollider2D(new Vector2d(12, 1));
        Initialize(context, first, Vector2d.Zero);
        Initialize(context, second, closerSecond ? Vector2d.Forward : Vector2d.Zero);
        var request = Request(source, Vector2d.Forward * (Fixed64)3, (Fixed64)4);
        context.Query2D.QuerySupport(request, out var support).Should().Be(Physics2DSupportQueryStatus.Found);
        support.Hit.Collider.Should().BeSameAs(closerSecond ? second : first);
        support.Hit.Distance.Should().Be(closerSecond ? Fixed64.One : Fixed64.Two);
        AssertReadyBoundedWitness(context, request);
    }

    [Fact]
    public void ArbitraryUp_SelectsSurfaceAndRejectsTooSteepNormal()
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        var wall = new LSAABBoxCollider2D(new Vector2d(1, 12));
        Initialize(context, wall, Vector2d.Zero);
        // Probe down-left at 45 degrees toward the wall's right side.
        var up = new Vector2d(Fixed64.One, Fixed64.One);
        var admitted = new Physics2DSupportQuery(source.Collider, Vector2d.Right * Fixed64.Two,
            Fixed64.Half, up, (Fixed64)3, Fixed64.Half, PhysicsLayerMask.All);
        context.Query2D.QuerySupport(admitted, out var support).Should().Be(Physics2DSupportQueryStatus.Found);
        support.Hit.Normal.Should().Be(Vector2d.Right);
        var rejected = new Physics2DSupportQuery(source.Collider, admitted.Center, admitted.Radius,
            up, admitted.Distance, Fixed64.One, PhysicsLayerMask.All);
        context.Query2D.QuerySupport(rejected, out var absent).Should().Be(Physics2DSupportQueryStatus.NoSupport);
        absent.IsCurrentLifetime.Should().BeFalse();
    }

    [Fact]
    public void WarmQueries_ReuseStorageAndKeepObservableHits()
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        var floor = new LSAABBoxCollider2D(new Vector2d(12, 1));
        Initialize(context, floor, Vector2d.Zero);
        var request = Request(source, Vector2d.Forward, Fixed64.One);
        int hits = 0;
        long allocated = AllocationTestHelper.MeasureSteadyState(() =>
        {
            if (context.Query2D.QuerySupport(request, out var hit) == Physics2DSupportQueryStatus.Found
                && ReferenceEquals(hit.Hit.Collider, floor))
                hits++;
        });
        hits.Should().Be(128 + 16 + 64);
        allocated.Should().Be(0);
    }

    private static SolidBody2D Source(GravitasWorldContext context)
    {
        var agent = new TestMatterAgent(context: context);
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half));
        body.Initialize(new Vector2d(10, 10), motionType: PhysicsMobilityType.Kinematic);
        return body;
    }

    private static void Initialize(GravitasWorldContext context, LSCollider2D collider, Vector2d position)
    {
        var agent = new TestMatterAgent(context: context);
        agent.Transform.TrySetWorldPosition(new Vector3d(position.X, Fixed64.Zero, position.Y)).Should().BeTrue();
        collider.InitializeWithNoBody(agent);
    }
}
