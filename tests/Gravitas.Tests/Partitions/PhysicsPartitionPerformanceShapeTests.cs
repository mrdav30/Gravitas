using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Partitions;

public sealed class PhysicsPartitionPerformanceShapeTests
{
    [Fact]
    public void LateSimulate_ShouldRepartitionTeleportedDynamicBodiesBeforeCollisionDistribution()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(8, 0, 0));
        first.Collider.PartitionChanged = false;
        second.Collider.PartitionChanged = false;

        Vector3d teleportedPosition = new(Fixed64.Half, Fixed64.Zero, Fixed64.Zero);
        second.Body.SetPosition(teleportedPosition);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        second.Collider.Bounds.Center.X.Should().Be(teleportedPosition.X);
        second.Collider.Bounds.Center.Z.Should().Be(teleportedPosition.Z);
        second.Collider.PartitionChanged.Should().BeTrue();
        first.Collider.TryGetCollisionPair(second.Collider.Id, out CollisionPair? pair).Should().BeTrue();
        pair!.Manifold.HasContact.Should().BeTrue();
    }

    [Fact]
    public void DynamicObjectRemoval_ShouldKeepPartitionActivationStableAcrossChurn()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        PhysicsPartition partition = context.Collisions.RentPartition();

        partition.AddDynamicObject(1);
        partition.AddDynamicObject(2);
        partition.AddDynamicObject(3);
        int activationId = partition.ActivationId;

        partition.RemoveDynamicObject(2);
        partition.AddDynamicObject(4);
        partition.RemoveDynamicObject(3);
        partition.RemoveDynamicObject(4);
        partition.RemoveDynamicObject(1);

        activationId.Should().BeGreaterThanOrEqualTo(0);
        partition.ActivationId.Should().Be(-1);
        context.Collisions.ActivePartitionCount.Should().Be(0);
        partition.ContainedDynamicObjects!.Count.Should().Be(0);
    }

    [Fact]
    public void ResetRetainedMembership_ShouldClearAllBucketsAndMarkPartitionEmpty()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Simulate();
        PhysicsPartition partition = context.Collisions.RentPartition();
        partition.AddDynamicObject(7);
        partition.AddKinematicObject(3);
        partition.AddStaticObject(11);
        partition.SetDynamicObjectAwake(7, awake: false);
        int activationId = partition.ActivationId;

        context.Collisions.DeactivatePartition(activationId);
        partition.ResetRetainedMembership();

        partition.IsEmpty.Should().BeTrue();
        partition.EmptySinceFrame.Should().Be(context.FrameCount);
        partition.IsAllocated.Should().BeFalse();
        partition.AwakeDynamicObjectCount.Should().Be(0);
        partition.ContainedDynamicObjects!.Count.Should().Be(0);
        partition.ContainedAwakeDynamicObjects!.Count.Should().Be(0);
        partition.ContainedKinematicObjects!.Count.Should().Be(0);
        partition.ContainedStaticObjects!.Count.Should().Be(0);
        context.Collisions.ReleasePartition(partition);
    }

    [Fact]
    public void ResetRetainedMembership_WithFreshPartition_ShouldBeIdempotent()
    {
        var partition = new PhysicsPartition();

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
        var partition = new PhysicsPartition();
        var ids = new SwiftList<int>();

        partition.CopyAllColliderIds(ids);
        ids.Count.Should().Be(0);

        partition.CopyStaticStyleColliderIds(ids);
        ids.Count.Should().Be(0);
    }
}
