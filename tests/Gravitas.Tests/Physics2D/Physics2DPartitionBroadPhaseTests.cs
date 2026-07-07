using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using GridForge.Grids;
using GridForge.Spatial;
using SwiftCollections;
using System;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class Physics2DPartitionBroadPhaseTests
{
    [Fact]
    public void Simulate_WithSparseScene_ShouldProcessOnlyPartitionCandidates()
    {
        using GravitasWorldContext context = CreateContext(extent: 512);
        SolidBody2D dynamicBody = CreateCircle(context, Vector2d.Zero, immovable: false);
        _ = CreateCircle(context, new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: true);

        for (int i = 0; i < 160; i++)
            _ = CreateCircle(context, new Vector2d((Fixed64)(16 + (i * 3)), (Fixed64)32), immovable: true);

        Step(context);

        dynamicBody.Position.X.Should().BeLessThan(Fixed64.Zero);
        context.Physics2D.LastBroadPhaseCandidateCount.Should().Be(1);
        context.Collisions2D.ActivePartitionCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void OverlapCircleAll_WithColliderSpanningMultipleVoxels_ShouldReturnOneHit()
    {
        using GravitasWorldContext context = CreateContext(extent: 32);
        SolidBody2D body = CreateBody(
            context,
            new LSAABBoxCollider2D(new Vector2d((Fixed64)8, (Fixed64)8)),
            Vector2d.Zero,
            immovable: true);
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.OverlapCircleAll(Vector2d.Zero, (Fixed64)6, hits);

        count.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(body.Collider);
        context.Query2D.LastQueryCandidateCount.Should().Be(1);
    }

    [Fact]
    public void MovingBody_ShouldLeaveOldPartitionsAndEnterNewPartitions()
    {
        using GravitasWorldContext context = CreateContext(extent: 32);
        SolidBody2D body = CreateCircle(context, Vector2d.Zero, immovable: false);
        var hits = new SwiftList<Physics2DHit>();

        context.Query2D.OverlapCircleAll(Vector2d.Zero, Fixed64.One, hits).Should().Be(1);

        body.SetPosition(new Vector2d((Fixed64)10, Fixed64.Zero));

        context.Query2D.OverlapCircleAll(Vector2d.Zero, Fixed64.One, hits).Should().Be(0);
        context.Query2D.OverlapCircleAll(new Vector2d((Fixed64)10, Fixed64.Zero), Fixed64.One, hits).Should().Be(1);
        hits[0].Collider.Should().BeSameAs(body.Collider);
    }

    [Fact]
    public void ShapeRefresh_During2DDistribution_ShouldDeferUntilNextQueryBoundary()
    {
        using GravitasWorldContext context = CreateContext(extent: 32);
        _ = CreateCircle(context, Vector2d.Zero, immovable: false);
        LSCircleCollider2D trigger = CreateBodylessCircle(
            context,
            new Vector2d(Fixed64.Half, Fixed64.Zero),
            isTrigger: true);
        LSCircleCollider2D expanding = CreateBodylessCircle(
            context,
            new Vector2d((Fixed64)8, Fixed64.Zero),
            isTrigger: false);
        var hits = new SwiftList<Physics2DHit>();
        int entered = 0;

        context.Query2D.OverlapCircleAll(Vector2d.Zero, Fixed64.One, hits);
        hits.Should().NotContain(hit => ReferenceEquals(hit.Collider, expanding));

        trigger.OnTriggerEnter += _ =>
        {
            entered++;
            expanding.Radius = (Fixed64)9;
            expanding.Simulate();
        };

        Step(context);

        entered.Should().Be(1);
        context.Query2D.OverlapCircleAll(Vector2d.Zero, Fixed64.One, hits).Should().BeGreaterThan(0);
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, expanding));
    }

    [Fact]
    public void Deactivate_WithRetainedPartitionTtk_ShouldRetireAndPoolEmpty2DPartitions()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        context.Settings.RetainedPartitionTimeToKillFrames = 1;
        context.Settings.RetainedPartitionRetirementSweepBudget = 1024;
        SolidBody2D body = CreateCircle(context, Vector2d.Zero, immovable: false);
        int retainedBeforeDeactivate = context.Collisions2D.RetainedPartitionCount;

        body.Deactivate();
        Step(context);

        context.Collisions2D.ActivePartitionCount.Should().Be(0);
        context.Collisions2D.RetainedPartitionCount.Should().BeLessThan(retainedBeforeDeactivate);
        context.Collisions2D.InactivePartitionCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void Reset_WithRetained2DPartitions_ShouldDetachOwnedVoxelPartitions()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        SolidBody2D body = CreateCircle(context, Vector2d.Zero, immovable: false);
        WorldVoxelIndex coordinate = body.Collider.PartitionCoordinates![0];
        context.World.TryGetVoxel(coordinate, out Voxel? voxel).Should().BeTrue();
        voxel!.TryGetPartition(out PhysicsPartition2D? partition).Should().BeTrue();
        context.Collisions2D.RetainedPartitionCount.Should().BeGreaterThan(0);

        context.Reset();

        context.Collisions2D.RetainedPartitionCount.Should().Be(0);
        context.Collisions2D.ActivePartitionCount.Should().Be(0);
        context.Collisions2D.InactivePartitionCount.Should().Be(0);
        voxel.TryGetPartition<PhysicsPartition2D>(out _).Should().BeFalse();
        (partition!.ContainedDynamicObjects?.Count ?? 0).Should().Be(0);
        partition.AwakeDynamicObjectCount.Should().Be(0);
        (partition.ContainedKinematicObjects?.Count ?? 0).Should().Be(0);
        (partition.ContainedStaticObjects?.Count ?? 0).Should().Be(0);
        partition.IsAllocated.Should().BeFalse();

        SolidBody2D replacement = CreateCircle(context, Vector2d.Zero, immovable: false);
        WorldVoxelIndex replacementCoordinate = replacement.Collider.PartitionCoordinates![0];
        context.World.TryGetVoxel(replacementCoordinate, out Voxel? replacementVoxel).Should().BeTrue();
        replacementVoxel!.TryGetPartition(out PhysicsPartition2D? replacementPartition).Should().BeTrue();
        replacementPartition!.ContainedDynamicObjects!.Contains(replacement.Collider.Id).Should().BeTrue();
        context.Collisions2D.RetainedPartitionCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void MobilityChanges_ShouldMoveColliderBetweenDynamicKinematicAndStaticBuckets()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        SolidBody2D body = CreateCircle(context, Vector2d.Zero, immovable: false);
        int colliderId = body.Collider.Id;

        PhysicsPartition2D partition = GetFirstPartition(context, body.Collider);
        AssertPartitionMembership(partition, colliderId, dynamic: true, kinematic: false, @static: false);

        body.IsKinematic = true;
        body.Collider.Simulate();

        partition = GetFirstPartition(context, body.Collider);
        AssertPartitionMembership(partition, colliderId, dynamic: false, kinematic: true, @static: false);

        body.FreezeAxes = BodyFreezeAxes2D.Position;
        body.Collider.Simulate();

        partition = GetFirstPartition(context, body.Collider);
        AssertPartitionMembership(partition, colliderId, dynamic: false, kinematic: false, @static: true);

        body.IsKinematic = false;
        body.FreezeAxes = BodyFreezeAxes2D.None;
        body.Collider.Simulate();

        partition = GetFirstPartition(context, body.Collider);
        AssertPartitionMembership(partition, colliderId, dynamic: true, kinematic: false, @static: false);
    }

    [Fact]
    public void ResetRetainedMembership_ShouldClearAllBucketsAndMarkPartitionEmpty()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        context.Simulate();
        PhysicsPartition2D partition = context.Collisions2D.RentPartition();
        partition.AddDynamicObject(7);
        partition.AddKinematicObject(3);
        partition.AddStaticObject(11);
        partition.SetDynamicObjectAwake(7, awake: false);
        int activationId = partition.ActivationId;

        context.Collisions2D.DeactivatePartition(activationId);
        partition.ResetRetainedMembership();

        partition.IsEmpty.Should().BeTrue();
        partition.EmptySinceFrame.Should().Be(context.FrameCount);
        partition.IsAllocated.Should().BeFalse();
        partition.AwakeDynamicObjectCount.Should().Be(0);
        partition.ContainedDynamicObjects!.Count.Should().Be(0);
        partition.ContainedAwakeDynamicObjects!.Count.Should().Be(0);
        partition.ContainedKinematicObjects!.Count.Should().Be(0);
        partition.ContainedStaticObjects!.Count.Should().Be(0);
        context.Collisions2D.ReleasePartition(partition);
    }

    [Fact]
    public void StaticAndKinematicRemoval_ShouldMarkPartitionEmptyAndIgnoreMissingIds()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        PhysicsPartition2D partition = context.Collisions2D.RentPartition();

        partition.RemoveStaticObject(99);
        partition.RemoveKinematicObject(99);
        partition.AddStaticObject(11);
        partition.AddKinematicObject(3);

        partition.IsEmpty.Should().BeFalse();
        partition.RemoveStaticObject(12);
        partition.RemoveKinematicObject(4);
        partition.ContainedStaticObjects!.Should().Contain(11);
        partition.ContainedKinematicObjects!.Should().Contain(3);

        partition.RemoveStaticObject(11);
        partition.IsEmpty.Should().BeFalse();
        partition.RemoveKinematicObject(3);

        partition.IsEmpty.Should().BeTrue();
        partition.ContainedStaticObjects.Count.Should().Be(0);
        partition.ContainedKinematicObjects.Count.Should().Be(0);
        context.Collisions2D.ReleasePartition(partition);
    }

    [Fact]
    public void ResetRetainedMembership_WithFreshPartition_ShouldBeIdempotent()
    {
        var partition = new PhysicsPartition2D();

        partition.ResetRetainedMembership();
        partition.ResetRetainedMembership();

        partition.IsEmpty.Should().BeTrue();
        partition.EmptySinceFrame.Should().Be(0);
        partition.IsAllocated.Should().BeFalse();
        partition.AwakeDynamicObjectCount.Should().Be(0);
        partition.ContainsAwakeDynamicObject(7).Should().BeFalse();
    }

    [Fact]
    public void EmptyPartitionCopyHelpers_ShouldReturnEmptySortedBuffers()
    {
        var partition = new PhysicsPartition2D();
        var ids = new SwiftList<int>();

        partition.CopyAllColliderIds(ids);
        ids.Count.Should().Be(0);

        partition.CopyStaticStyleColliderIds(ids);
        ids.Count.Should().Be(0);
    }

    [Fact]
    public void ContextReset_ShouldClear2DPartitionAndColliderRegistryState()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        SolidBody2D body = CreateCircle(context, Vector2d.Zero, immovable: false);
        int id = body.Collider.Id;

        context.Collisions2D.ActivePartitionCount.Should().BeGreaterThan(0);
        context.Collisions2D.RetainedPartitionCount.Should().BeGreaterThan(0);
        body.Collider.IsPartitioned.Should().BeTrue();

        context.Reset();

        context.Collisions2D.ActivePartitionCount.Should().Be(0);
        context.Collisions2D.RetainedPartitionCount.Should().Be(0);
        context.Collisions2D.InactivePartitionCount.Should().Be(0);
        context.Physics2D.ColliderCount.Should().Be(0);
        context.Physics2D.TryGetColliderById(id, out LSCollider2D? resolved).Should().BeFalse();
        resolved.Should().BeNull();
        body.Collider.IsPartitioned.Should().BeFalse();
        (body.Collider.PartitionCoordinates?.Count ?? 0).Should().Be(0);
        body.Collider.Id.Should().Be(-1);
    }

    [Fact]
    public void Simulate_WithOnlySleepingDynamicAndStaticObjects_ShouldSkipPartitionWork()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        SolidBody2D sleeper = CreateCircle(context, Vector2d.Zero, immovable: false);
        _ = CreateCircle(context, new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: true);
        sleeper.Sleep();

        Step(context);

        sleeper.IsSleeping.Should().BeTrue();
        context.Physics2D.LastBroadPhaseCandidateCount.Should().Be(0);
    }

    [Fact]
    public void Simulate_WhenExistingSolidPairFallsAsleep_ShouldRetainRestingContactWithoutExit()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        SolidBody2D sleeper = CreateCircle(context, Vector2d.Zero, immovable: false);
        _ = CreateCircle(context, new Vector2d((Fixed64)0.75f, Fixed64.Zero), immovable: true);
        int exited = 0;
        sleeper.Collider.OnContactExit += _ => exited++;

        Step(context);
        sleeper.Sleep();
        Step(context);

        sleeper.IsSleeping.Should().BeTrue();
        exited.Should().Be(0);
        context.Physics2D.LastBroadPhaseCandidateCount.Should().Be(0);
    }

    [Fact]
    public void Simulate_WithAwakeDynamicTouchingSleepingDynamic_ShouldWakeSleeper()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        SolidBody2D sleeper = CreateCircle(context, Vector2d.Zero, immovable: false);
        _ = CreateCircle(context, new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: false);
        sleeper.Sleep();

        Step(context);

        sleeper.IsSleeping.Should().BeFalse();
        context.Physics2D.LastBroadPhaseCandidateCount.Should().Be(1);
    }

    [Fact]
    public void Simulate_WithConnectedSleepingContactIsland_ShouldWakeWholeIsland()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        SolidBody2D far = CreateCircle(context, Vector2d.Zero, immovable: false);
        SolidBody2D middle = CreateCircle(
            context,
            new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: false);
        SolidBody2D driver = CreateCircle(
            context,
            new Vector2d(Fixed64.FromFraction(3, 2), Fixed64.Zero),
            immovable: false);
        far.SleepFrameThreshold = 64;
        middle.SleepFrameThreshold = 64;
        driver.SleepFrameThreshold = 64;
        driver.SleepEnabled = false;

        Step(context);
        far.SetPosition(Vector2d.Zero);
        middle.SetPosition(new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero));
        driver.SetPosition(new Vector2d(Fixed64.FromFraction(3, 2), Fixed64.Zero));
        GetPair(far, middle).IsColliding.Should().BeTrue();
        GetPair(middle, driver).IsColliding.Should().BeTrue();
        far.Sleep();
        middle.Sleep();
        driver.AddForce(Vector2d.Left);
        driver.IsSleeping.Should().BeFalse();

        Step(context);

        CollisionPair2D farMiddle = GetPair(far, middle);
        CollisionPair2D middleDriver = GetPair(middle, driver);
        driver.IsSleeping.Should().BeFalse(
            $"driver should remain the awake source; sleepEnabled={driver.SleepEnabled}, threshold={driver.SleepFrameThreshold}, velocity={driver.LinearVelocity}, speed={driver.LinearSpeed}");
        middle.IsSleeping.Should().BeFalse(
            $"middle should wake through the connected 2D island; velocity={middle.LinearVelocity}, speed={middle.LinearSpeed}");
        far.IsSleeping.Should().BeFalse(
            $"far should wake through the pre-existing sleeping edge; velocity={far.LinearVelocity}, speed={far.LinearSpeed}, farMiddleLast={farMiddle.LastFrame}, farMiddleContact={farMiddle.Manifold.HasContact}, middleDriverLast={middleDriver.LastFrame}, middleDriverContact={middleDriver.Manifold.HasContact}");
    }

    [Fact]
    public void ReplayedMovingPartitionScenario_ShouldProduceSame2DStateAndCandidateCounts()
    {
        ReplayResult first = RunReplayScenario();
        ReplayResult second = RunReplayScenario();

        second.Should().Be(first);
    }

    [Fact]
    public void Simulate_WithDenseOverlappingPairs_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = CreateContext(extent: 128);
        var bodies = new SwiftList<SolidBody2D>();
        for (int i = 0; i < 64; i++)
        {
            Vector2d position = PositionForIndex(i, spacing: (Fixed64)2);
            bodies.Add(CreateCircle(context, position, immovable: false));
            _ = CreateCircle(context, position + new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: true);
        }

        for (int i = 0; i < 3; i++)
        {
            ResetBodyPositions(bodies);
            Step(context);
        }

        long allocatedBytes = MeasureAllocatedBytes(() =>
        {
            ResetBodyPositions(bodies);
            Step(context);
        });

        allocatedBytes.Should().Be(0);
    }

    private static ReplayResult RunReplayScenario()
    {
        using GravitasWorldContext context = CreateContext(extent: 256, frameRate: 8);
        var bodies = new SwiftList<SolidBody2D>();
        for (int i = 0; i < 48; i++)
            bodies.Add(CreateCircle(context, new Vector2d((Fixed64)(i * 3), Fixed64.Zero), immovable: false));

        _ = CreateCircle(context, new Vector2d((Fixed64)12, Fixed64.Half), immovable: true);
        int candidateTotal = 0;
        for (int frame = 0; frame < 8; frame++)
        {
            SolidBody2D moved = bodies[frame % bodies.Count];
            moved.SetPosition(moved.Position + new Vector2d(Fixed64.Half, Fixed64.Zero));
            context.Simulate();
            context.LateSimulate();
            candidateTotal += context.Physics2D.LastBroadPhaseCandidateCount;
        }

        SolidBody2D body = bodies[4];
        return new ReplayResult(body.Position, body.LinearVelocity, candidateTotal, context.Collisions2D.RetainedPartitionCount);
    }

    private static GravitasWorldContext CreateContext(int extent, int frameRate = 4)
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.SetFrameRate(frameRate);
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-16), Fixed64.Zero, (Fixed64)(-16)),
                new Vector3d((Fixed64)extent, Fixed64.Zero, (Fixed64)extent)),
            out _).Should().BeTrue();
        return context;
    }

    private static void Step(GravitasWorldContext context)
    {
        context.Simulate();
        context.LateSimulate();
    }

    private static CollisionPair2D GetPair(SolidBody2D first, SolidBody2D second)
    {
        if (first.Collider.TryGetCollisionPair(second.Collider.Id, out CollisionPair2D? firstPair) && firstPair != null)
            return firstPair;

        second.Collider.TryGetCollisionPair(first.Collider.Id, out CollisionPair2D? secondPair).Should().BeTrue();
        return secondPair!;
    }

    private static SolidBody2D CreateCircle(GravitasWorldContext context, Vector2d position, bool immovable)
    {
        return CreateBody(context, new LSCircleCollider2D(Fixed64.Half), position, immovable);
    }

    private static LSCircleCollider2D CreateBodylessCircle(
        GravitasWorldContext context,
        Vector2d position,
        bool isTrigger)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var collider = new LSCircleCollider2D(Fixed64.Half)
        {
            IsTrigger = isTrigger
        };
        collider.InitializeWithNoBody(agent);
        return collider;
    }

    private static SolidBody2D CreateBody(
        GravitasWorldContext context,
        LSCollider2D collider,
        Vector2d position,
        bool immovable)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One,
            FreezeAxes = immovable ? BodyFreezeAxes2D.Position : BodyFreezeAxes2D.None
        };
        body.Initialize(position);
        return body;
    }

    private static PhysicsPartition2D GetFirstPartition(GravitasWorldContext context, LSCollider2D collider)
    {
        context.World.TryGetVoxel(collider.PartitionCoordinates![0], out Voxel? voxel).Should().BeTrue();
        voxel!.TryGetPartition(out PhysicsPartition2D? partition).Should().BeTrue();
        return partition!;
    }

    private static void AssertPartitionMembership(
        PhysicsPartition2D partition,
        int colliderId,
        bool dynamic,
        bool kinematic,
        bool @static)
    {
        (partition.ContainedDynamicObjects?.Contains(colliderId) ?? false).Should().Be(dynamic);
        (partition.ContainedKinematicObjects?.Contains(colliderId) ?? false).Should().Be(kinematic);
        (partition.ContainedStaticObjects?.Contains(colliderId) ?? false).Should().Be(@static);
    }

    private static Vector2d PositionForIndex(int index, Fixed64 spacing)
    {
        int width = 8;
        int x = index % width;
        int y = index / width;
        return new Vector2d((Fixed64)x * spacing, (Fixed64)y * spacing);
    }

    private static void ResetBodyPositions(SwiftList<SolidBody2D> bodies)
    {
        for (int i = 0; i < bodies.Count; i++)
            bodies[i].SetPosition(PositionForIndex(i, spacing: (Fixed64)2));
    }

    private static long MeasureAllocatedBytes(Action action)
        => AllocationTestHelper.MeasureSinglePass(action);

    private readonly record struct ReplayResult(
        Vector2d Position,
        Vector2d Velocity,
        int CandidateTotal,
        int RetainedPartitionCount);
}
