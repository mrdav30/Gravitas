using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Diagnostics;
using Gravitas.Tests.Support;
using GridForge.Grids;
using GridForge.Spatial;
using System;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Partitions;

public sealed class PhysicsPartitionAwakeTests
{
    [Fact]
    public void SleepingDynamicBodies_ShouldRemainPartitionedButNotAwake()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        PhysicsPartition partition = GetFirstPartition(scenario, body.Collider);

        body.Body.Sleep();

        partition.ContainedDynamicObjects!.Contains(body.Collider.Id).Should().BeTrue();
        partition.ContainsAwakeDynamicObject(body.Collider.Id).Should().BeFalse();
        partition.AwakeDynamicObjectCount.Should().Be(0);
    }

    [Fact]
    public void Distribute_WithOnlySleepingDynamicBodies_ShouldSkipPairGeneration()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        first.Body.Sleep();
        second.Body.Sleep();

        scenario.Context.Collisions.CheckAndDistributeCollisions();

        first.Collider.TryGetCollisionPair(second.Collider.Id, out _).Should().BeFalse();
    }

    [Fact]
    public void Distribute_WithAwakeDynamicAgainstSleepingDynamic_ShouldProcessPairAndWakeSleepingBody()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> awake = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> sleeping = scenario.CreateSphere(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        sleeping.Body.Sleep();

        scenario.Context.Collisions.CheckAndDistributeCollisions();

        awake.Collider.TryGetCollisionPair(sleeping.Collider.Id, out CollisionPair? pair).Should().BeTrue();
        pair!.Manifold.HasContact.Should().BeTrue();
        sleeping.Body.IsSleeping.Should().BeFalse();
    }

    [Fact]
    public void Simulate_WithAwakeDynamicAgainstSleepingDynamic_ShouldStillNotifyContactEnter()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> awake = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> sleeping = scenario.CreateSphere(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        int awakeEnterCount = 0;
        int sleepingEnterCount = 0;
        awake.Collider.OnContactEnter += _ => awakeEnterCount++;
        sleeping.Collider.OnContactEnter += _ => sleepingEnterCount++;
        sleeping.Body.Sleep();

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        awakeEnterCount.Should().Be(1);
        sleepingEnterCount.Should().Be(1);
        sleeping.Body.IsSleeping.Should().BeFalse();
    }

    [Fact]
    public void SleepingRestingContact_ShouldNotAgeOutAsFalseExit()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        int firstExitCount = 0;
        int secondExitCount = 0;
        first.Collider.OnContactExit += _ => firstExitCount++;
        second.Collider.OnContactExit += _ => secondExitCount++;
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();
        first.Body.Sleep();
        second.Body.Sleep();

        for (int i = 0; i <= scenario.Context.FrameRate * 8; i++)
        {
            scenario.Context.Simulate();
            scenario.Context.LateSimulate();
        }

        firstExitCount.Should().Be(0);
        secondExitCount.Should().Be(0);
        first.Collider.TryGetCollisionPair(second.Collider.Id, out CollisionPair? pair).Should().BeTrue();
        pair!.Active.Should().BeTrue();
    }

    [Fact]
    public void Simulate_RepeatedOverlappingDynamics_ShouldProduceStableContactOrder()
    {
        CaptureContactOrder().Should().Equal(CaptureContactOrder());
    }

    [Fact]
    public void Distribute_AfterDifferentPartitionChurnOrders_ShouldProduceStableContactOrder()
    {
        CaptureChurnedContactOrder(new[] { 0, 1, 2, 3 })
            .Should()
            .Equal(CaptureChurnedContactOrder(new[] { 3, 2, 1, 0 }));
    }

    [Fact]
    public void ClearPartitionedObject_WhenPartitionBecomesEmpty_ShouldKeepVoxelPartitionForReuse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        var originalCoordinate = body.Collider.PartitionCoordinates![0];
        scenario.Context.World.TryGetVoxel(originalCoordinate, out Voxel? originalVoxel)
            .Should()
            .BeTrue();
        originalVoxel!.TryGetPartition(out PhysicsPartition? originalPartition).Should().BeTrue();

        body.Body.SetPosition(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));
        body.Collider.Simulate();

        originalVoxel.TryGetPartition(out PhysicsPartition? retainedPartition).Should().BeTrue();
        retainedPartition.Should().BeSameAs(originalPartition);
        retainedPartition!.ContainedDynamicObjects!.Count.Should().Be(0);
        retainedPartition.AwakeDynamicObjectCount.Should().Be(0);
        retainedPartition.IsAllocated.Should().BeFalse();
    }

    [Fact]
    public void MobilityChanges_ShouldMoveColliderBetweenDynamicKinematicAndStaticBuckets()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        int colliderId = body.Collider.Id;

        PhysicsPartition partition = GetFirstPartition(scenario, body.Collider);
        AssertPartitionMembership(partition, colliderId, dynamic: true, kinematic: false, @static: false);

        body.Body.IsKinematic = true;
        body.Collider.Simulate();

        partition = GetFirstPartition(scenario, body.Collider);
        AssertPartitionMembership(partition, colliderId, dynamic: false, kinematic: true, @static: false);

        body.Body.Immovable = true;
        body.Collider.Simulate();

        partition = GetFirstPartition(scenario, body.Collider);
        AssertPartitionMembership(partition, colliderId, dynamic: false, kinematic: false, @static: true);

        body.Body.IsKinematic = false;
        body.Body.Immovable = false;
        body.Collider.Simulate();

        partition = GetFirstPartition(scenario, body.Collider);
        AssertPartitionMembership(partition, colliderId, dynamic: true, kinematic: false, @static: false);
    }

    [Fact]
    public void Reset_WithRetainedVoxelPartitions_ShouldDetachOwnedVoxelPartitions()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        WorldVoxelIndex coordinate = body.Collider.PartitionCoordinates![0];
        scenario.Context.World.TryGetVoxel(coordinate, out Voxel? voxel).Should().BeTrue();
        voxel!.TryGetPartition(out PhysicsPartition? partition).Should().BeTrue();
        scenario.Context.Collisions.RetainedPartitionCount.Should().BeGreaterThan(0);

        scenario.Context.Reset();

        scenario.Context.Collisions.RetainedPartitionCount.Should().Be(0);
        scenario.Context.Collisions.ActivePartitionCount.Should().Be(0);
        scenario.Context.Collisions.InactivePartitionCount.Should().Be(0);
        voxel.TryGetPartition<PhysicsPartition>(out _).Should().BeFalse();
        (partition!.ContainedDynamicObjects?.Count ?? 0).Should().Be(0);
        partition.AwakeDynamicObjectCount.Should().Be(0);
        (partition.ContainedKinematicObjects?.Count ?? 0).Should().Be(0);
        (partition.ContainedStaticObjects?.Count ?? 0).Should().Be(0);
        partition.IsAllocated.Should().BeFalse();

        ScenarioBody<LSSphereCollider> replacement = scenario.CreateSphere(Vector3d.Zero);
        WorldVoxelIndex replacementCoordinate = replacement.Collider.PartitionCoordinates![0];
        scenario.Context.World.TryGetVoxel(replacementCoordinate, out Voxel? replacementVoxel).Should().BeTrue();
        replacementVoxel!.TryGetPartition(out PhysicsPartition? replacementPartition).Should().BeTrue();
        replacementPartition!.ContainedDynamicObjects!.Contains(replacement.Collider.Id).Should().BeTrue();
        scenario.Context.Collisions.RetainedPartitionCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void Reset_WithDenseRetainedVoxelPartitions_ShouldDetachEveryOwnedVoxelPartition()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var coordinates = new List<WorldVoxelIndex>();

        for (int i = 0; i < 8; i++)
        {
            ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(
                new Vector3d((Fixed64)(i % 4), Fixed64.Zero, (Fixed64)(i / 4)));

            for (int j = 0; j < body.Collider.PartitionCoordinates!.Count; j++)
                coordinates.Add(body.Collider.PartitionCoordinates[j]);
        }

        scenario.Context.Collisions.RetainedPartitionCount.Should().BeGreaterThan(1);

        scenario.Context.Reset();

        scenario.Context.Collisions.RetainedPartitionCount.Should().Be(0);
        for (int i = 0; i < coordinates.Count; i++)
        {
            scenario.Context.World.TryGetVoxel(coordinates[i], out Voxel? voxel).Should().BeTrue();
            voxel!.TryGetPartition<PhysicsPartition>(out _).Should().BeFalse();
        }
    }

    [Fact]
    public void Simulate_WhenRetainedPartitionExceedsTimeToKill_ShouldRemoveItAndPoolPartition()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Settings.RetainedPartitionTimeToKillFrames = 2;
        scenario.Context.Settings.RetainedPartitionRetirementSweepBudget = 1024;
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        WorldVoxelIndex originalCoordinate = body.Collider.PartitionCoordinates![0];
        scenario.Context.World.TryGetVoxel(originalCoordinate, out Voxel? originalVoxel).Should().BeTrue();
        originalVoxel!.TryGetPartition(out PhysicsPartition? originalPartition).Should().BeTrue();
        int inactiveBeforeMove = scenario.Context.Collisions.InactivePartitionCount;

        body.Body.SetPosition(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));
        AdvancePhysicsStep(scenario);
        originalVoxel.TryGetPartition(out PhysicsPartition? retainedPartition).Should().BeTrue();
        retainedPartition.Should().BeSameAs(originalPartition);

        AdvancePhysicsStep(scenario);
        originalVoxel.TryGetPartition<PhysicsPartition>(out _).Should().BeTrue();

        AdvancePhysicsStep(scenario);

        originalVoxel.TryGetPartition<PhysicsPartition>(out _).Should().BeFalse();
        scenario.Context.Collisions.InactivePartitionCount.Should().BeGreaterThan(inactiveBeforeMove);
    }

    [Fact]
    public void Simulate_WhenColliderReturnsBeforePartitionTimeToKill_ShouldReuseRetainedPartition()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Settings.RetainedPartitionTimeToKillFrames = 2;
        scenario.Context.Settings.RetainedPartitionRetirementSweepBudget = 1024;
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        WorldVoxelIndex originalCoordinate = body.Collider.PartitionCoordinates![0];
        scenario.Context.World.TryGetVoxel(originalCoordinate, out Voxel? originalVoxel).Should().BeTrue();
        originalVoxel!.TryGetPartition(out PhysicsPartition? originalPartition).Should().BeTrue();

        body.Body.SetPosition(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));
        AdvancePhysicsStep(scenario);
        body.Body.SetPosition(Vector3d.Zero);
        AdvancePhysicsStep(scenario);
        AdvancePhysicsStep(scenario);

        originalVoxel.TryGetPartition(out PhysicsPartition? reusedPartition).Should().BeTrue();
        reusedPartition.Should().BeSameAs(originalPartition);
        reusedPartition!.ContainedDynamicObjects!.Contains(body.Collider.Id).Should().BeTrue();
    }

    [Fact]
    public void CheckAndDistributeCollisions_ShouldRetireNoMoreThanPartitionRetirementBudget()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Settings.RetainedPartitionTimeToKillFrames = 0;
        scenario.Context.Settings.RetainedPartitionRetirementSweepBudget = 1;
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        int retainedBeforeClear = scenario.Context.Collisions.RetainedPartitionCount;
        retainedBeforeClear.Should().BeGreaterThan(1);
        int inactiveBeforeClear = scenario.Context.Collisions.InactivePartitionCount;

        scenario.Context.Collisions.ClearPartitionedObject(body.Collider, force: true).Should().BeTrue();
        scenario.Context.Collisions.CheckAndDistributeCollisions();

        scenario.Context.Collisions.RetainedPartitionCount.Should().Be(retainedBeforeClear - 1);
        scenario.Context.Collisions.InactivePartitionCount.Should().Be(inactiveBeforeClear + 1);
    }

    private static PhysicsPartition GetFirstPartition(PhysicsScenarioBuilder scenario, LSCollider collider)
    {
        scenario.Context.World.TryGetVoxel(collider.PartitionCoordinates![0], out GridForge.Grids.Voxel? voxel)
            .Should().BeTrue();
        voxel!.TryGetPartition(out PhysicsPartition? partition).Should().BeTrue();
        return partition!;
    }

    private static void AssertPartitionMembership(
        PhysicsPartition partition,
        int colliderId,
        bool dynamic,
        bool kinematic,
        bool @static)
    {
        (partition.ContainedDynamicObjects?.Contains(colliderId) ?? false).Should().Be(dynamic);
        (partition.ContainedKinematicObjects?.Contains(colliderId) ?? false).Should().Be(kinematic);
        (partition.ContainedStaticObjects?.Contains(colliderId) ?? false).Should().Be(@static);
    }

    private static void AdvancePhysicsStep(PhysicsScenarioBuilder scenario)
    {
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();
    }

    private static int[] CaptureContactOrder()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Diagnostics.Enable(eventCapacity: 16, drawCommandCapacity: 0);
        scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        scenario.CreateSphere(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        scenario.CreateSphere(new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero));

        scenario.Context.Simulate();

        var order = new List<int>();
        ReadOnlySpan<GravitasDiagnosticEvent> events = scenario.Context.Diagnostics.Events;
        for (int i = 0; i < events.Length; i++)
        {
            GravitasDiagnosticEvent diagnosticEvent = events[i];
            if (diagnosticEvent.Kind != GravitasDiagnosticEventKind.Contact)
                continue;

            order.Add((diagnosticEvent.ColliderAId << 16) | diagnosticEvent.ColliderBId);
        }

        return order.ToArray();
    }

    private static int[] CaptureChurnedContactOrder(int[] moveOrder)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider>[] bodies = new ScenarioBody<LSSphereCollider>[4];
        for (int i = 0; i < bodies.Length; i++)
        {
            bodies[i] = scenario.CreateSphere(new Vector3d((Fixed64)(-8 + i * 4), Fixed64.Zero, Fixed64.Zero));
            PhysicsScenarioBuilder.SetTrigger(bodies[i].Collider);
        }

        for (int i = 0; i < moveOrder.Length; i++)
        {
            int index = moveOrder[i];
            bodies[index].Body.SetPosition(Vector3d.Zero);
            bodies[index].Collider.Simulate();
        }

        scenario.Context.Diagnostics.Enable(eventCapacity: 32, drawCommandCapacity: 0);
        scenario.Context.Collisions.CheckAndDistributeCollisions();

        var order = new List<int>();
        ReadOnlySpan<GravitasDiagnosticEvent> events = scenario.Context.Diagnostics.Events;
        for (int i = 0; i < events.Length; i++)
        {
            GravitasDiagnosticEvent diagnosticEvent = events[i];
            if (diagnosticEvent.Kind != GravitasDiagnosticEventKind.Contact)
                continue;

            order.Add((diagnosticEvent.ColliderAId << 16) | diagnosticEvent.ColliderBId);
        }

        return order.ToArray();
    }

}
