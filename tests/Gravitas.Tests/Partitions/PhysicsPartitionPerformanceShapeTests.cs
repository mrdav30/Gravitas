using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
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
}
