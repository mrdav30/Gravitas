using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using GridForge.Grids;
using GridForge.Spatial;
using SwiftCollections;
using System;
using Xunit;

namespace Gravitas.Tests.Partitions;

public sealed class PhysicsPartitionTests
{
    [Fact]
    public void OnRemoveFromVoxel_WithoutOwner_ShouldThrowInvariantViolation()
    {
        var partition = new PhysicsPartition();

        Action removeWithoutOwner = () => partition.OnRemoveFromVoxel(null!);

        removeWithoutOwner.Should()
            .Throw<InvalidOperationException>()
            .WithMessage("*owner*");
    }

    [Fact]
    public void Distribute_ShouldResolvePairsThroughOwningServiceWithoutSharedScratchState()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();
        (LSSphereCollider colliderA1, LSSphereCollider colliderA2) = CreateOverlappingPair(contextA);
        (LSSphereCollider colliderB1, LSSphereCollider colliderB2) = CreateOverlappingPair(contextB);

        contextA.Collisions.CheckAndDistributeCollisions();
        contextB.Collisions.CheckAndDistributeCollisions();

        CollisionPair? pairA = contextA.Physics.GetCollisionPair(colliderA1.Id, colliderA2.Id);
        CollisionPair? pairB = contextB.Physics.GetCollisionPair(colliderB1.Id, colliderB2.Id);
        pairA.Should().NotBeNull();
        pairB.Should().NotBeNull();
        pairA!.Context.Should().BeSameAs(contextA);
        pairB!.Context.Should().BeSameAs(contextB);
        pairA.PartitionVersion.Should().Be(contextA.Collisions.Version);
        pairB.PartitionVersion.Should().Be(contextB.Collisions.Version);
    }

    [Fact]
    public void PhysicsPartition_ShouldTrackMembershipAwakeStateAndPoolReset()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var partition = new PhysicsPartition();
        partition.SetOwner(context.Collisions);

        partition.AddDynamicObject(3);
        partition.AddDynamicObject(3);
        partition.AddStaticObject(7);
        partition.AddStaticObject(7);
        partition.AddKinematicObject(5);
        partition.AddKinematicObject(5);

        partition.ContainedDynamicObjects!.Count.Should().Be(1);
        partition.ContainedStaticObjects!.Count.Should().Be(1);
        partition.ContainedKinematicObjects!.Count.Should().Be(1);
        partition.IsAllocated.Should().BeTrue();
        partition.ContainsAwakeDynamicObject(3).Should().BeTrue();
        partition.SetDynamicObjectAwake(99, awake: true);
        partition.SetDynamicObjectAwake(3, awake: false);
        partition.ContainsAwakeDynamicObject(3).Should().BeFalse();
        partition.SetDynamicObjectAwake(3, awake: true);
        partition.ContainsAwakeDynamicObject(3).Should().BeTrue();

        partition.RemoveDynamicObject(99);
        partition.RemoveStaticObject(99);
        partition.RemoveKinematicObject(99);
        partition.RemoveDynamicObject(3);
        partition.IsAllocated.Should().BeFalse();
        partition.EmptySinceFrame.Should().Be(-1);
        partition.RemoveStaticObject(7);
        partition.RemoveKinematicObject(5);
        partition.IsEmpty.Should().BeTrue();
        partition.EmptySinceFrame.Should().Be(context.FrameCount);

        var activePartition = new PhysicsPartition();
        activePartition.SetOwner(context.Collisions);
        activePartition.AddDynamicObject(12);
        activePartition.ResetForPool();
        activePartition.IsAllocated.Should().BeFalse();
        activePartition.IsPartitioned.Should().BeFalse();
        activePartition.Invoking(static partition => partition.Owner)
            .Should()
            .Throw<InvalidOperationException>();
    }

    [Fact]
    public void PhysicsPartition_WithMultipleDynamicObjects_ShouldStayActiveUntilLastDynamicRemoval()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext otherContext = GravitasWorldContext.CreateOwned();
        var partition = new PhysicsPartition();
        partition.SetOwner(context.Collisions);
        partition.SetOwner(context.Collisions);
        Action setDifferentOwner = () => partition.SetOwner(otherContext.Collisions);

        partition.AddDynamicObject(3);
        partition.AddDynamicObject(4);
        int activationId = partition.ActivationId;
        partition.RemoveDynamicObject(3);

        partition.IsAllocated.Should().BeTrue();
        partition.ActivationId.Should().Be(activationId);
        partition.ContainsAwakeDynamicObject(4).Should().BeTrue();
        partition.EmptySinceFrame.Should().Be(-1);
        setDifferentOwner.Should().Throw<ArgumentException>().WithParameterName("owner");

        partition.RemoveDynamicObject(4);
        partition.IsAllocated.Should().BeFalse();
        partition.ActivationId.Should().Be(-1);

        var emptyPartition = new PhysicsPartition();
        emptyPartition.SetOwner(context.Collisions);
        emptyPartition.ResetForPool();
        emptyPartition.IsAllocated.Should().BeFalse();
        emptyPartition.ActivationId.Should().Be(-1);
    }

    [Fact]
    public void PhysicsPartition_WithAlreadySleepingDynamicObject_ShouldAvoidAwakeMembership()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.Sleep();
        var partition = new PhysicsPartition();
        partition.SetOwner(scenario.Context.Collisions);

        partition.AddDynamicObject(body.Collider.Id);

        partition.ContainedDynamicObjects!.Contains(body.Collider.Id).Should().BeTrue();
        partition.ContainedAwakeDynamicObjects.Should().BeNull();
        partition.ContainsAwakeDynamicObject(body.Collider.Id).Should().BeFalse();

        partition.RemoveDynamicObject(body.Collider.Id);

        partition.IsAllocated.Should().BeFalse();
        partition.ContainedDynamicObjects.Count.Should().Be(0);
    }

    [Fact]
    public void PhysicsPartition_WithBodylessRegisteredDynamicObject_ShouldTreatItAsAwake()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider collider = CreateBodylessSphere(context, Vector3d.Zero);
        var partition = new PhysicsPartition();
        partition.SetOwner(context.Collisions);

        partition.AddDynamicObject(collider.Id);

        partition.ContainsAwakeDynamicObject(collider.Id).Should().BeTrue();

        partition.RemoveDynamicObject(collider.Id);
    }

    [Fact]
    public void PhysicsPartition_CopyHelpers_WithEmptyBuckets_ShouldClearDestination()
    {
        var partition = new PhysicsPartition();
        var ids = new SwiftList<int> { 99 };

        partition.CopyAllColliderIds(ids);
        ids.Count.Should().Be(0);

        ids.Add(99);
        partition.CopyStaticStyleColliderIds(ids);
        ids.Count.Should().Be(0);
    }

    [Fact]
    public void PhysicsPartition_RemoveMissingObjects_WithNoBuckets_ShouldBeIdempotent()
    {
        var partition = new PhysicsPartition();

        partition.RemoveDynamicObject(3);
        partition.RemoveStaticObject(5);
        partition.RemoveKinematicObject(7);

        partition.IsEmpty.Should().BeTrue();
        partition.IsAllocated.Should().BeFalse();
        partition.EmptySinceFrame.Should().Be(-1);
    }

    [Fact]
    public void PhysicsPartition2D_ShouldTrackMembershipAwakeStateAndPoolReset()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        var partition = new PhysicsPartition2D();
        partition.SetOwner(context.Collisions2D);

        partition.AddDynamicObject(3);
        partition.AddDynamicObject(3);
        partition.AddStaticObject(7);
        partition.AddStaticObject(7);
        partition.AddKinematicObject(5);
        partition.AddKinematicObject(5);

        partition.ContainedDynamicObjects!.Count.Should().Be(1);
        partition.ContainedStaticObjects!.Count.Should().Be(1);
        partition.ContainedKinematicObjects!.Count.Should().Be(1);
        partition.IsAllocated.Should().BeTrue();
        partition.ContainsAwakeDynamicObject(3).Should().BeTrue();
        partition.SetDynamicObjectAwake(99, awake: true);
        partition.SetDynamicObjectAwake(3, awake: false);
        partition.ContainsAwakeDynamicObject(3).Should().BeFalse();
        partition.SetDynamicObjectAwake(3, awake: true);
        partition.ContainsAwakeDynamicObject(3).Should().BeTrue();

        partition.RemoveDynamicObject(99);
        partition.RemoveStaticObject(99);
        partition.RemoveKinematicObject(99);
        partition.RemoveDynamicObject(3);
        partition.IsAllocated.Should().BeFalse();
        partition.EmptySinceFrame.Should().Be(-1);
        partition.RemoveStaticObject(7);
        partition.RemoveKinematicObject(5);
        partition.IsEmpty.Should().BeTrue();
        partition.EmptySinceFrame.Should().Be(context.FrameCount);

        var activePartition = new PhysicsPartition2D();
        activePartition.SetOwner(context.Collisions2D);
        activePartition.AddDynamicObject(12);
        activePartition.ResetForPool();
        activePartition.IsAllocated.Should().BeFalse();
        activePartition.IsPartitioned.Should().BeFalse();
        activePartition.Invoking(static partition => partition.Owner)
            .Should()
            .Throw<InvalidOperationException>();
    }

    [Fact]
    public void PhysicsPartition2D_WithMultipleDynamicObjects_ShouldStayActiveUntilLastDynamicRemoval()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        using GravitasWorldContext otherContext = GravitasWorldContext.CreateOwned();
        otherContext.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        var partition = new PhysicsPartition2D();
        partition.SetOwner(context.Collisions2D);
        partition.SetOwner(context.Collisions2D);
        Action setDifferentOwner = () => partition.SetOwner(otherContext.Collisions2D);

        partition.AddDynamicObject(3);
        partition.AddDynamicObject(4);
        int activationId = partition.ActivationId;
        partition.RemoveDynamicObject(3);

        partition.IsAllocated.Should().BeTrue();
        partition.ActivationId.Should().Be(activationId);
        partition.ContainsAwakeDynamicObject(4).Should().BeTrue();
        partition.EmptySinceFrame.Should().Be(-1);
        setDifferentOwner.Should().Throw<ArgumentException>().WithParameterName("owner");

        partition.RemoveDynamicObject(4);
        partition.IsAllocated.Should().BeFalse();
        partition.ActivationId.Should().Be(-1);

        var emptyPartition = new PhysicsPartition2D();
        emptyPartition.SetOwner(context.Collisions2D);
        emptyPartition.ResetForPool();
        emptyPartition.IsAllocated.Should().BeFalse();
        emptyPartition.ActivationId.Should().Be(-1);
    }

    [Fact]
    public void PhysicsPartition2D_WithAlreadySleepingDynamicObject_ShouldAvoidAwakeMembership()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        SolidBody2D body = CreateCircle2D(context, Vector2d.Zero);
        body.Sleep();
        var partition = new PhysicsPartition2D();
        partition.SetOwner(context.Collisions2D);

        partition.AddDynamicObject(body.Collider.Id);

        partition.ContainedDynamicObjects!.Contains(body.Collider.Id).Should().BeTrue();
        partition.ContainedAwakeDynamicObjects.Should().BeNull();
        partition.ContainsAwakeDynamicObject(body.Collider.Id).Should().BeFalse();

        partition.RemoveDynamicObject(body.Collider.Id);

        partition.IsAllocated.Should().BeFalse();
        partition.ContainedDynamicObjects.Count.Should().Be(0);
    }

    [Fact]
    public void PhysicsPartition2D_WithBodylessRegisteredDynamicObject_ShouldTreatItAsAwake()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        LSCircleCollider2D collider = CreateBodylessCircle2D(context, Vector2d.Zero);
        var partition = new PhysicsPartition2D();
        partition.SetOwner(context.Collisions2D);

        partition.AddDynamicObject(collider.Id);

        partition.ContainsAwakeDynamicObject(collider.Id).Should().BeTrue();

        partition.RemoveDynamicObject(collider.Id);
    }

    [Fact]
    public void PhysicsPartition2D_CopyHelpers_WithEmptyBuckets_ShouldClearDestination()
    {
        var partition = new PhysicsPartition2D();
        var ids = new SwiftList<int> { 99 };

        partition.CopyAllColliderIds(ids);
        ids.Count.Should().Be(0);

        ids.Add(99);
        partition.CopyStaticStyleColliderIds(ids);
        ids.Count.Should().Be(0);
    }

    [Fact]
    public void PhysicsPartition2D_RemoveMissingObjects_WithNoBuckets_ShouldBeIdempotent()
    {
        var partition = new PhysicsPartition2D();

        partition.RemoveDynamicObject(3);
        partition.RemoveStaticObject(5);
        partition.RemoveKinematicObject(7);

        partition.IsEmpty.Should().BeTrue();
        partition.IsAllocated.Should().BeFalse();
        partition.EmptySinceFrame.Should().Be(-1);
    }

    [Fact]
    public void PhysicsMixedPartition_ShouldTrackDimensionalMembershipAwakeStateAndPoolReset()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        var partition = new PhysicsMixedPartition();
        partition.SetOwner(context.MixedCollisions);

        partition.AddDynamic3DObject(3);
        partition.AddDynamic3DObject(3);
        partition.AddDynamic2DObject(4);
        partition.AddDynamic2DObject(4);
        partition.AddStatic3DObject(7);
        partition.AddStatic3DObject(7);
        partition.AddKinematic3DObject(9);
        partition.AddKinematic3DObject(9);
        partition.AddStatic2DObject(8);
        partition.AddStatic2DObject(8);
        partition.AddKinematic2DObject(10);
        partition.AddKinematic2DObject(10);

        partition.ContainedDynamic3DObjects!.Count.Should().Be(1);
        partition.ContainedDynamic2DObjects!.Count.Should().Be(1);
        partition.ContainedStatic3DObjects!.Count.Should().Be(1);
        partition.ContainedKinematic3DObjects!.Count.Should().Be(1);
        partition.ContainedStatic2DObjects!.Count.Should().Be(1);
        partition.ContainedKinematic2DObjects!.Count.Should().Be(1);
        partition.IsAllocated.Should().BeTrue();
        partition.AwakeDynamicObjectCount.Should().Be(2);

        partition.SetDynamic3DObjectAwake(99, awake: true);
        partition.SetDynamic2DObjectAwake(99, awake: true);
        partition.SetDynamic3DObjectAwake(3, awake: false);
        partition.SetDynamic2DObjectAwake(4, awake: false);
        partition.AwakeDynamicObjectCount.Should().Be(0);
        partition.SetDynamic3DObjectAwake(3, awake: true);
        partition.SetDynamic2DObjectAwake(4, awake: true);
        partition.AwakeDynamicObjectCount.Should().Be(2);

        partition.RemoveDynamic3DObject(99);
        partition.RemoveStatic3DObject(99);
        partition.RemoveKinematic3DObject(99);
        partition.RemoveDynamic2DObject(99);
        partition.RemoveStatic2DObject(99);
        partition.RemoveKinematic2DObject(99);
        partition.RemoveDynamic3DObject(3);
        partition.IsAllocated.Should().BeTrue();
        partition.RemoveDynamic2DObject(4);
        partition.IsAllocated.Should().BeFalse();
        partition.EmptySinceFrame.Should().Be(-1);
        partition.RemoveStatic3DObject(7);
        partition.RemoveKinematic3DObject(9);
        partition.RemoveStatic2DObject(8);
        partition.RemoveKinematic2DObject(10);
        partition.IsEmpty.Should().BeTrue();
        partition.EmptySinceFrame.Should().Be(context.FrameCount);

        var activePartition = new PhysicsMixedPartition();
        activePartition.SetOwner(context.MixedCollisions);
        activePartition.AddDynamic3DObject(12);
        activePartition.ResetForPool();
        activePartition.IsAllocated.Should().BeFalse();
        activePartition.IsPartitioned.Should().BeFalse();
        activePartition.Invoking(static partition => partition.Owner)
            .Should()
            .Throw<InvalidOperationException>();
    }

    [Fact]
    public void PhysicsMixedPartition_WithMultipleDynamicObjects_ShouldStayActiveUntilLastDynamicRemoval()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        using GravitasWorldContext otherContext = GravitasWorldContext.CreateOwned();
        otherContext.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        var partition = new PhysicsMixedPartition();
        partition.SetOwner(context.MixedCollisions);
        partition.SetOwner(context.MixedCollisions);
        Action setDifferentOwner = () => partition.SetOwner(otherContext.MixedCollisions);

        partition.AddDynamic3DObject(3);
        partition.AddDynamic3DObject(4);
        partition.AddDynamic2DObject(5);
        int activationId = partition.ActivationId;
        partition.RemoveDynamic3DObject(3);
        partition.RemoveDynamic2DObject(5);

        partition.IsAllocated.Should().BeTrue();
        partition.ActivationId.Should().Be(activationId);
        partition.AwakeDynamicObjectCount.Should().Be(1);
        partition.EmptySinceFrame.Should().Be(-1);
        setDifferentOwner.Should().Throw<ArgumentException>().WithParameterName("owner");

        partition.RemoveDynamic3DObject(4);
        partition.IsAllocated.Should().BeFalse();
        partition.ActivationId.Should().Be(-1);

        var emptyPartition = new PhysicsMixedPartition();
        emptyPartition.SetOwner(context.MixedCollisions);
        emptyPartition.ResetForPool();
        emptyPartition.IsAllocated.Should().BeFalse();
        emptyPartition.ActivationId.Should().Be(-1);
    }

    [Fact]
    public void PhysicsMixedPartition_WithBodylessRegisteredDynamicObjects_ShouldTreatThemAsAwake()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        LSSphereCollider collider3D = CreateBodylessSphere(context, Vector3d.Zero);
        LSCircleCollider2D collider2D = CreateBodylessCircle2D(context, Vector2d.Zero);
        var partition = new PhysicsMixedPartition();
        partition.SetOwner(context.MixedCollisions);

        partition.AddDynamic3DObject(collider3D.Id);
        partition.AddDynamic2DObject(collider2D.Id);

        partition.AwakeDynamicObjectCount.Should().Be(2);

        partition.RemoveDynamic3DObject(collider3D.Id);
        partition.RemoveDynamic2DObject(collider2D.Id);
    }

    [Fact]
    public void PhysicsMixedPartition_CopyHelpers_WithEmptyBuckets_ShouldClearDestination()
    {
        var partition = new PhysicsMixedPartition();
        var ids = new SwiftList<int> { 99 };

        partition.Copy3DColliderIds(ids);
        ids.Count.Should().Be(0);

        ids.Add(99);
        partition.Copy2DColliderIds(ids);
        ids.Count.Should().Be(0);

        ids.Add(99);
        partition.CopyStaticStyle3DColliderIds(ids);
        ids.Count.Should().Be(0);

        ids.Add(99);
        partition.CopyStaticStyle2DColliderIds(ids);
        ids.Count.Should().Be(0);
    }

    [Fact]
    public void PhysicsMixedPartition_RemoveMissingObjects_WithNoBuckets_ShouldBeIdempotent()
    {
        var partition = new PhysicsMixedPartition();

        partition.RemoveDynamic3DObject(3);
        partition.RemoveStatic3DObject(5);
        partition.RemoveKinematic3DObject(7);
        partition.RemoveDynamic2DObject(11);
        partition.RemoveStatic2DObject(13);
        partition.RemoveKinematic2DObject(17);

        partition.IsEmpty.Should().BeTrue();
        partition.IsAllocated.Should().BeFalse();
        partition.EmptySinceFrame.Should().Be(-1);
    }

    [Fact]
    public void RetainedPartitionLifecycle_Untrack_ShouldMaintainDenseIndicesAndRetirementCursor()
    {
        var owner = new object();
        var retained = new SwiftList<RetainedPartitionProbe>();
        RetainedPartitionProbe first = CreateProbe(owner);
        RetainedPartitionProbe second = CreateProbe(owner);
        RetainedPartitionProbe third = CreateProbe(owner);

        RetainedPartitionLifecycle.Track(retained, owner, first, nameof(RetainedPartitionProbe));
        RetainedPartitionLifecycle.Track(retained, owner, second, nameof(RetainedPartitionProbe));
        RetainedPartitionLifecycle.Track(retained, owner, third, nameof(RetainedPartitionProbe));
        int retirementCursor = 2;

        RetainedPartitionLifecycle.Untrack(retained, owner, second, ref retirementCursor);

        retained.Count.Should().Be(2);
        retained[0].Should().BeSameAs(first);
        retained[1].Should().BeSameAs(third);
        second.RetainedIndex.Should().Be(-1);
        third.RetainedIndex.Should().Be(1);
        retirementCursor.Should().Be(1);

        RetainedPartitionLifecycle.Untrack(retained, owner, second, ref retirementCursor);

        retained.Count.Should().Be(2);
        second.RetainedIndex.Should().Be(-1);

        RetainedPartitionLifecycle.Untrack(retained, owner, third, ref retirementCursor);

        retained.Count.Should().Be(1);
        retirementCursor.Should().Be(0);
    }

    [Fact]
    public void RetainedPartitionLifecycle_Untrack_WithStaleCollidingIndex_ShouldNotRemoveTrackedPartition()
    {
        var owner = new object();
        var retained = new SwiftList<RetainedPartitionProbe>();
        RetainedPartitionProbe tracked = CreateProbe(owner);
        RetainedPartitionProbe stale = CreateProbe(owner);
        RetainedPartitionLifecycle.Track(retained, owner, tracked, nameof(RetainedPartitionProbe));
        stale.SetRetainedIndex(tracked.RetainedIndex);
        int retirementCursor = 0;

        RetainedPartitionLifecycle.Untrack(retained, owner, stale, ref retirementCursor);

        retained.Count.Should().Be(1);
        retained[0].Should().BeSameAs(tracked);
        tracked.RetainedIndex.Should().Be(0);
        stale.RetainedIndex.Should().Be(-1);
        retirementCursor.Should().Be(0);
    }

    [Fact]
    public void RetainedPartitionLifecycle_Track_ShouldRejectAlreadyTrackedPartition()
    {
        var owner = new object();
        var retained = new SwiftList<RetainedPartitionProbe>();
        RetainedPartitionProbe partition = CreateProbe(owner);

        RetainedPartitionLifecycle.Track(retained, owner, partition, nameof(RetainedPartitionProbe));

        Action trackAgain = () =>
            RetainedPartitionLifecycle.Track(retained, owner, partition, nameof(RetainedPartitionProbe));

        trackAgain.Should()
            .Throw<ArgumentException>()
            .WithMessage("*already tracked*");
    }

    [Fact]
    public void RetainedPartitionLifecycle_TryRetireEmptyForReuse_ShouldSkipForeignOrAllocatedPartitions()
    {
        var owner = new object();
        var foreignOwner = new object();
        var retained = new SwiftList<RetainedPartitionProbe>();
        var pool = new SwiftStack<RetainedPartitionProbe>();
        var release = new RetainedPartitionProbeRelease(retained, pool, owner);
        RetainedPartitionProbe foreign = CreateProbe(foreignOwner);
        RetainedPartitionProbe allocated = CreateProbe(owner, isAllocated: true);
        RetainedPartitionProbe reusable = CreateProbe(owner);

        RetainedPartitionLifecycle.Track(retained, foreignOwner, foreign, nameof(RetainedPartitionProbe));
        RetainedPartitionLifecycle.Track(retained, owner, allocated, nameof(RetainedPartitionProbe));
        RetainedPartitionLifecycle.Track(retained, owner, reusable, nameof(RetainedPartitionProbe));

        bool retired = RetainedPartitionLifecycle.TryRetireEmptyForReuse(
            retained,
            pool,
            new GridWorld(),
            owner,
            release.Release,
            ref release.Cursor);

        retired.Should().BeTrue();
        pool.Count.Should().Be(1);
        pool.Peek().Should().BeSameAs(reusable);
        retained.Count.Should().Be(2);
        retained[0].Should().BeSameAs(foreign);
        retained[1].Should().BeSameAs(allocated);
        release.Cursor.Should().Be(0);
    }

    [Fact]
    public void RetainedPartitionLifecycle_RetireExpired_ShouldRetireOnlyExpiredOwnedEmptyPartitions()
    {
        var owner = new object();
        var retained = new SwiftList<RetainedPartitionProbe>();
        var pool = new SwiftStack<RetainedPartitionProbe>();
        var release = new RetainedPartitionProbeRelease(retained, pool, owner);
        RetainedPartitionProbe notExpired = CreateProbe(owner, emptySinceFrame: 9);
        RetainedPartitionProbe notYetEmpty = CreateProbe(owner, emptySinceFrame: -1);
        RetainedPartitionProbe allocated = CreateProbe(owner, emptySinceFrame: 0, isAllocated: true);
        RetainedPartitionProbe expired = CreateProbe(owner, emptySinceFrame: 0);

        RetainedPartitionLifecycle.Track(retained, owner, notExpired, nameof(RetainedPartitionProbe));
        RetainedPartitionLifecycle.Track(retained, owner, notYetEmpty, nameof(RetainedPartitionProbe));
        RetainedPartitionLifecycle.Track(retained, owner, allocated, nameof(RetainedPartitionProbe));
        RetainedPartitionLifecycle.Track(retained, owner, expired, nameof(RetainedPartitionProbe));

        RetainedPartitionLifecycle.RetireExpired(
            retained,
            new GridWorld(),
            owner,
            budget: 4,
            currentFrame: 10,
            timeToKillFrames: 5,
            release.Release,
            ref release.Cursor);

        pool.Count.Should().Be(1);
        pool.Peek().Should().BeSameAs(expired);
        retained.Count.Should().Be(3);
        retained[0].Should().BeSameAs(notExpired);
        retained[1].Should().BeSameAs(notYetEmpty);
        retained[2].Should().BeSameAs(allocated);
        release.Cursor.Should().Be(0);
    }

    [Fact]
    public void RetainedPartitionLifecycle_TryRetireEmptyForReuse_ShouldStopWhenReleaseDoesNotPool()
    {
        var owner = new object();
        var retained = new SwiftList<RetainedPartitionProbe>();
        var pool = new SwiftStack<RetainedPartitionProbe>();
        RetainedPartitionProbe releasedWithoutPooling = CreateProbe(owner);
        RetainedPartitionProbe reusable = CreateProbe(owner);
        int cursor = 0;

        RetainedPartitionLifecycle.Track(retained, owner, releasedWithoutPooling, nameof(RetainedPartitionProbe));
        RetainedPartitionLifecycle.Track(retained, owner, reusable, nameof(RetainedPartitionProbe));

        void Release(RetainedPartitionProbe partition)
        {
            RetainedPartitionLifecycle.Untrack(retained, owner, partition, ref cursor);
            partition.ResetForPool(owner);
            if (!ReferenceEquals(partition, releasedWithoutPooling))
                pool.Push(partition);
        }

        bool retired = RetainedPartitionLifecycle.TryRetireEmptyForReuse(
            retained,
            pool,
            new GridWorld(),
            owner,
            Release,
            ref cursor);

        retired.Should().BeFalse();
        pool.Count.Should().Be(0);
        retained.Count.Should().Be(1);
        retained[0].Should().BeSameAs(reusable);
        releasedWithoutPooling.RetainedIndex.Should().Be(-1);
    }

    [Fact]
    public void RetainedPartitionLifecycle_RetireExpired_WithStaleVoxelAttachment_ShouldReleasePartition()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        EnsureGrid(context);
        context.World.TryGetVoxel(Vector3d.Zero, out Voxel? voxel).Should().BeTrue();
        var owner = new object();
        var retained = new SwiftList<RetainedPartitionProbe>();
        var pool = new SwiftStack<RetainedPartitionProbe>();
        var release = new RetainedPartitionProbeRelease(retained, pool, owner);
        RetainedPartitionProbe attachedOther = CreateProbe(owner);
        RetainedPartitionProbe stale = CreateProbe(owner);

        voxel!.TryAddPartition(attachedOther).Should().BeTrue();
        stale.SetParentIndex(attachedOther.WorldIndex);
        RetainedPartitionLifecycle.Track(retained, owner, stale, nameof(RetainedPartitionProbe));

        RetainedPartitionLifecycle.RetireExpired(
            retained,
            context.World,
            owner,
            budget: 1,
            currentFrame: 10,
            timeToKillFrames: 1,
            release.Release,
            ref release.Cursor);

        pool.Count.Should().Be(1);
        pool.Peek().Should().BeSameAs(stale);
        retained.Count.Should().Be(0);
        voxel.HasPartition<RetainedPartitionProbe>().Should().BeTrue();
    }

    [Fact]
    public void RetainedPartitionLifecycle_RetireExpired_ShouldSkipForeignPartitionAndReleaseMissingAttachment()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        EnsureGrid(context);
        context.World.TryGetVoxel(Vector3d.Zero, out Voxel? voxel).Should().BeTrue();
        var owner = new object();
        var foreignOwner = new object();
        var retained = new SwiftList<RetainedPartitionProbe>();
        var pool = new SwiftStack<RetainedPartitionProbe>();
        var release = new RetainedPartitionProbeRelease(retained, pool, owner);
        RetainedPartitionProbe foreign = CreateProbe(foreignOwner);
        RetainedPartitionProbe occupied = CreateProbe(owner);
        RetainedPartitionProbe missing = CreateProbe(owner);
        occupied.IsEmpty = false;
        foreign.SetParentIndex(voxel!.WorldIndex);
        occupied.SetParentIndex(voxel.WorldIndex);
        missing.SetParentIndex(voxel.WorldIndex);
        RetainedPartitionLifecycle.Track(retained, foreignOwner, foreign, nameof(RetainedPartitionProbe));
        RetainedPartitionLifecycle.Track(retained, owner, occupied, nameof(RetainedPartitionProbe));
        RetainedPartitionLifecycle.Track(retained, owner, missing, nameof(RetainedPartitionProbe));

        RetainedPartitionLifecycle.RetireExpired(
            retained,
            context.World,
            owner,
            budget: 3,
            currentFrame: 10,
            timeToKillFrames: 1,
            release.Release,
            ref release.Cursor);

        pool.Count.Should().Be(1);
        pool.Peek().Should().BeSameAs(missing);
        retained.Count.Should().Be(2);
        retained[0].Should().BeSameAs(foreign);
        retained[1].Should().BeSameAs(occupied);
        foreign.RetainedIndex.Should().Be(0);
        release.Cursor.Should().Be(0);
    }

    [Fact]
    public void RetainedPartitionLifecycle_DetachAll_ShouldReleaseOwnedPartitionsAndDropForeignRetainedEntries()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        EnsureGrid(context);
        context.World.TryGetVoxel(Vector3d.Zero, out Voxel? voxel).Should().BeTrue();
        var owner = new object();
        var foreignOwner = new object();
        var retained = new SwiftList<RetainedPartitionProbe>();
        var pool = new SwiftStack<RetainedPartitionProbe>();
        var release = new RetainedPartitionProbeRelease(retained, pool, owner);
        RetainedPartitionProbe attached = CreateProbe(owner);
        RetainedPartitionProbe detached = CreateProbe(owner);
        RetainedPartitionProbe foreign = CreateProbe(foreignOwner);
        attached.Removed = release.Release;

        RetainedPartitionLifecycle.Track(retained, owner, detached, nameof(RetainedPartitionProbe));
        RetainedPartitionLifecycle.Track(retained, owner, attached, nameof(RetainedPartitionProbe));
        RetainedPartitionLifecycle.Track(retained, foreignOwner, foreign, nameof(RetainedPartitionProbe));
        voxel!.TryAddPartition(attached).Should().BeTrue();

        RetainedPartitionLifecycle.DetachAll(
            retained,
            context.World,
            owner,
            release.Release,
            nameof(RetainedPartitionProbe),
            "detach failed");

        retained.Count.Should().Be(0);
        pool.Count.Should().Be(2);
        foreign.RetainedIndex.Should().Be(-1);
        voxel.HasPartition<RetainedPartitionProbe>().Should().BeFalse();
        attached.RemovedFromVoxelCount.Should().Be(1);
    }

    [Fact]
    public void RetainedPartitionLifecycle_DetachAll_ShouldRecoverAfterRemovalCallbackPartiallyUntracks()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        EnsureGrid(context);
        context.World.TryGetVoxel(Vector3d.Zero, out Voxel? voxel).Should().BeTrue();
        var owner = new object();
        var retained = new SwiftList<RetainedPartitionProbe>();
        var pool = new SwiftStack<RetainedPartitionProbe>();
        var release = new RetainedPartitionProbeRelease(retained, pool, owner);
        RetainedPartitionProbe attached = CreateProbe(owner);
        attached.Removed = partition =>
        {
            int cursor = -1;
            RetainedPartitionLifecycle.Untrack(retained, owner, partition, ref cursor);
            throw new InvalidOperationException("simulated partial release failure");
        };
        RetainedPartitionLifecycle.Track(retained, owner, attached, nameof(RetainedPartitionProbe));
        voxel!.TryAddPartition(attached).Should().BeTrue();

        RetainedPartitionLifecycle.DetachAll(
            retained,
            context.World,
            owner,
            release.Release,
            nameof(RetainedPartitionProbe),
            "detach failed");

        retained.Count.Should().Be(0);
        pool.Count.Should().Be(1);
        pool.Peek().Should().BeSameAs(attached);
        attached.IsOwnedBy(owner).Should().BeFalse();
        attached.RemovedFromVoxelCount.Should().Be(1);
    }

    private static (LSSphereCollider First, LSSphereCollider Second) CreateOverlappingPair(GravitasWorldContext context)
    {
        LSSphereCollider first = CreateDynamicSphere(context);
        LSSphereCollider second = CreateDynamicSphere(context);
        return (first, second);
    }

    private static LSSphereCollider CreateDynamicSphere(GravitasWorldContext context)
    {
        EnsureGrid(context);
        var agent = new TestMatterAgent(context);
        var collider = new LSSphereCollider();
        var body = new SolidBody(agent, collider)
        {
            Mass = Fixed64.One
        };

        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
        return collider;
    }

    private static SolidBody2D CreateCircle2D(GravitasWorldContext context, Vector2d position)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var body = new SolidBody2D(new TestMatterAgent(context, transform), new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One
        };

        body.Initialize(position);
        return body;
    }

    private static LSSphereCollider CreateBodylessSphere(GravitasWorldContext context, Vector3d position)
    {
        EnsureGrid(context);
        var transform = new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One);
        var collider = new LSSphereCollider();
        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
        return collider;
    }

    private static LSCircleCollider2D CreateBodylessCircle2D(GravitasWorldContext context, Vector2d position)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var collider = new LSCircleCollider2D(Fixed64.Half);
        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
        return collider;
    }

    private static void EnsureGrid(GravitasWorldContext context)
    {
        if (context.World.ActiveGrids.Count > 0)
            return;

        GridConfiguration configuration = new(
            new Vector3d(-2, -2, -2),
            new Vector3d(2, 2, 2));

        context.World.TryAddGrid(configuration, out _).Should().BeTrue();
    }

    private static RetainedPartitionProbe CreateProbe(
        object owner,
        int emptySinceFrame = 0,
        bool isAllocated = false) =>
        new(owner, emptySinceFrame, isAllocated);

    private sealed class RetainedPartitionProbe : IRetainedPhysicsPartition<object>
    {
        private object? _owner;

        public RetainedPartitionProbe(
            object owner,
            int emptySinceFrame,
            bool isAllocated)
        {
            _owner = owner;
            EmptySinceFrame = emptySinceFrame;
            IsAllocated = isAllocated;
        }

        public WorldVoxelIndex WorldIndex { get; private set; }

        public int RetainedIndex { get; private set; } = -1;

        public bool IsEmpty { get; set; } = true;

        public bool IsAllocated { get; }

        public int EmptySinceFrame { get; }

        public int RemovedFromVoxelCount { get; private set; }

        public Action<RetainedPartitionProbe>? Removed { get; set; }

        public bool IsOwnedBy(object owner) => ReferenceEquals(_owner, owner);

        public void SetRetainedIndex(int index) => RetainedIndex = index;

        public void ClearRetainedIndex() => RetainedIndex = -1;

        public void SetParentIndex(WorldVoxelIndex parentIndex) => WorldIndex = parentIndex;

        public void OnAddToVoxel(Voxel voxel) => WorldIndex = voxel.WorldIndex;

        public void OnRemoveFromVoxel(Voxel voxel)
        {
            RemovedFromVoxelCount++;
            Removed?.Invoke(this);
        }

        public void ResetForPool(object owner)
        {
            if (IsOwnedBy(owner))
                _owner = null;

            ClearRetainedIndex();
        }
    }

    private sealed class RetainedPartitionProbeRelease
    {
        private readonly SwiftList<RetainedPartitionProbe> _retained;
        private readonly SwiftStack<RetainedPartitionProbe> _pool;
        private readonly object _owner;

        public RetainedPartitionProbeRelease(
            SwiftList<RetainedPartitionProbe> retained,
            SwiftStack<RetainedPartitionProbe> pool,
            object owner)
        {
            _retained = retained;
            _pool = pool;
            _owner = owner;
        }

        public int Cursor;

        public void Release(RetainedPartitionProbe partition)
        {
            RetainedPartitionLifecycle.Untrack(_retained, _owner, partition, ref Cursor);
            partition.ResetForPool(_owner);
            _pool.Push(partition);
        }
    }
}
