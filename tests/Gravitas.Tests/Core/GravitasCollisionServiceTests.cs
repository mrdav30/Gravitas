using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using GridForge.Grids;
using GridForge.Spatial;
using System;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class GravitasCollisionServiceTests
{
    [Fact]
    public void PartitionObject_ShouldActivatePartitionWithinOwningContext()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();

        LSSphereCollider collider = CreateDynamicSphere(context);

        context.Collisions.ActivePartitionCount.Should().BeGreaterThan(0);
        collider.PartitionCoordinates.Should().NotBeNull();
        collider.PartitionCoordinates!.Count.Should().BeGreaterThan(0);

        WorldVoxelIndex coordinate = collider.PartitionCoordinates[0];
        context.World.TryGetVoxel(coordinate, out Voxel? voxel).Should().BeTrue();
        voxel!.TryGetPartition(out PhysicsPartition? partition).Should().BeTrue();
        partition!.Owner.Should().BeSameAs(context.Collisions);
        partition.ActivationId.Should().BeGreaterThanOrEqualTo(0);
    }

    [Fact]
    public void CheckAndDistributeCollisions_ShouldAdvanceOnlyOwningContextVersion()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();
        CreateDynamicSphere(contextA);
        CreateDynamicSphere(contextB);
        uint versionA = contextA.Collisions.Version;
        uint versionB = contextB.Collisions.Version;

        contextA.Collisions.CheckAndDistributeCollisions();

        contextA.Collisions.Version.Should().Be(versionA + 1);
        contextB.Collisions.Version.Should().Be(versionB);
        contextA.Collisions.ActivePartitionCount.Should().BeGreaterThan(0);
        contextB.Collisions.ActivePartitionCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void ClearPartitionedObject_ShouldReleaseEmptyPartitionsOnlyOnce()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider collider = CreateDynamicSphere(context);
        int partitionCount = collider.PartitionCoordinates!.Count;
        int inactiveBeforeClear = context.Collisions.InactivePartitionCount;

        context.Collisions.ClearPartitionedObject(collider, force: true).Should().BeTrue();
        int inactiveAfterClear = context.Collisions.InactivePartitionCount;
        context.Collisions.ClearPartitionedObject(collider, force: true).Should().BeTrue();

        context.Collisions.ActivePartitionCount.Should().Be(0);
        inactiveAfterClear.Should().Be(inactiveBeforeClear + partitionCount);
        context.Collisions.InactivePartitionCount.Should().Be(inactiveAfterClear);
    }

    [Fact]
    public void ClearPartitionedObject_InOneContext_ShouldNotAffectOverlappingCoordinatesInAnotherContext()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();
        LSSphereCollider colliderA = CreateDynamicSphere(contextA);
        LSSphereCollider colliderB = CreateDynamicSphere(contextB);

        contextA.Collisions.ClearPartitionedObject(colliderA, force: true).Should().BeTrue();

        contextA.Collisions.ActivePartitionCount.Should().Be(0);
        contextB.Collisions.ActivePartitionCount.Should().BeGreaterThan(0);
        contextB.World.TryGetVoxel(colliderB.PartitionCoordinates![0], out Voxel? voxelB).Should().BeTrue();
        voxelB!.TryGetPartition(out PhysicsPartition? partitionB).Should().BeTrue();
        partitionB!.Owner.Should().BeSameAs(contextB.Collisions);
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
        GridConfiguration configuration = new(
            new Vector3d(-2, -2, -2),
            new Vector3d(2, 2, 2));

        context.World.TryAddGrid(configuration, out _).Should().BeTrue();
    }
}
