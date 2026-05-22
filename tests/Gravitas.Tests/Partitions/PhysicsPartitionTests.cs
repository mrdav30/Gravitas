using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using GridForge.Configuration;
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
        var body = new StiffBody(agent, collider)
        {
            Mass = Fixed64.One
        };

        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
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
}
