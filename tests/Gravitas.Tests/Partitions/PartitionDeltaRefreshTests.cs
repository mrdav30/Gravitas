using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using GridForge.Grids;
using GridForge.Grids.Topology;
using GridForge.Spatial;
using SwiftCollections;
using System;
using Xunit;

namespace Gravitas.Tests.Partitions;

public sealed class PartitionDeltaRefreshTests
{
    [Fact]
    public void PartitionCollider_WithForeignPartitionedCollider_ShouldRejectContextMismatch()
    {
        using GravitasWorldContext owner = CreateContext(mixed: false);
        using GravitasWorldContext other = CreateContext(mixed: false);
        var (_, body2D) = CreateBody(owner, new Vector3d(4, 4, 4), planar: true, mixed: false);
        body2D!.Collider.IsPartitioned.Should().BeTrue();

        Action partition = () => other.Collisions2D.PartitionCollider(body2D.Collider);

        partition.Should().Throw<ArgumentException>().WithParameterName("collider");
        body2D.Collider.IsPartitioned.Should().BeTrue();
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void MovingInsideCoveredVoxels_ShouldKeepMembershipsAndCoordinateStorage(bool planar, bool mixed)
    {
        using GravitasWorldContext context = CreateContext(mixed);
        var position = new Vector3d(4, 4, 4);
        var (body3D, body2D) = CreateBody(context, position, planar, mixed);
        SwiftList<WorldVoxelIndex> coordinates = Coordinates(body3D, body2D, mixed);
        var previousCoordinates = new WorldVoxelIndex[coordinates.Count];
        for (int i = 0; i < coordinates.Count; i++)
            previousCoordinates[i] = coordinates[i];
        coordinates.Count.Should().BeGreaterThan(0);
        SwiftHashSet<int> membership = GetDynamicMembership(context, coordinates[0], planar, mixed);
        var enumeration = membership.GetEnumerator();
        uint version = planar ? body2D!.Collider.BroadPhaseVersion : body3D!.Collider.BroadPhaseVersion;

        position.X += Fixed64.FromFraction(1, 4);
        Move(body3D, body2D, position, mixed);

        // A surviving voxel membership must never be transiently removed: besides
        // wasted work, that invalidates stable membership enumeration and activation.
        Action advance = () => enumeration.MoveNext();
        advance.Should().NotThrow();
        enumeration.Current.Should().Be(planar ? body2D!.Collider.Id : body3D!.Collider.Id);
        coordinates.Should().Equal(previousCoordinates);
        (planar
            ? (mixed ? body2D!.Collider.MixedPartitionCoordinates : body2D!.Collider.PartitionCoordinates)
            : (mixed ? body3D!.Collider.MixedPartitionCoordinates : body3D!.Collider.PartitionCoordinates))
            .Should().BeSameAs(coordinates);
        (planar ? body2D!.Collider.BroadPhaseVersion : body3D!.Collider.BroadPhaseVersion)
            .Should().BeGreaterThan(version);
    }

    private static GravitasWorldContext CreateContext(bool mixed)
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = mixed ? PhysicsRuntimeMode.Mixed : PhysicsRuntimeMode.Both;
        context.World.TryAddGrid(new GridConfiguration(new Vector3d(-32, -32, -32),
            new Vector3d(32, 32, 32), topologyMetrics: GridTopologyMetrics.Rectangular((Fixed64)16)), out _)
            .Should().BeTrue();
        return context;
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void CrossingVoxelBoundary_ShouldRemoveDeparturesAndMatchFreshCoverage(bool planar, bool mixed)
    {
        using GravitasWorldContext context = CreateContext(mixed);
        var (body3D, body2D) = CreateBody(context, new Vector3d(4, 4, 4), planar, mixed);
        SwiftList<WorldVoxelIndex> coordinates = Coordinates(body3D, body2D, mixed);
        var oldCoordinates = new WorldVoxelIndex[coordinates.Count];
        for (int i = 0; i < coordinates.Count; i++)
            oldCoordinates[i] = coordinates[i];
        int id = planar ? body2D!.Collider.Id : body3D!.Collider.Id;

        Move(body3D, body2D, new Vector3d(20, 4, 4), mixed);
        var (fresh3D, fresh2D) = CreateBody(context, new Vector3d(20, 4, 4), planar, mixed);

        coordinates.Should().Equal(Coordinates(fresh3D, fresh2D, mixed));
        Coordinates(body3D, body2D, mixed).Should().BeSameAs(coordinates);
        int departed = 0;
        foreach (WorldVoxelIndex coordinate in oldCoordinates)
        {
            if (coordinates.Contains(coordinate))
                continue;

            departed++;
            (FindDynamicMembership(context, coordinate, planar, mixed)?.Contains(id) ?? false)
                .Should().BeFalse();
        }
        departed.Should().BeGreaterThan(0);
        foreach (WorldVoxelIndex coordinate in coordinates)
            GetDynamicMembership(context, coordinate, planar, mixed).Contains(id).Should().BeTrue();

        Move(body3D, body2D, new Vector3d(256, 4, 4), mixed);
        coordinates.Count.Should().Be(0);
        if (mixed)
            (planar ? body2D!.Collider.IsMixedPartitioned : body3D!.Collider.IsMixedPartitioned).Should().BeFalse();
        else if (planar)
            body2D!.Collider.IsPartitioned.Should().BeFalse();
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void ChangedBounds_ShouldRepairExternallyRemovedSurvivingPartition(bool planar, bool mixed)
    {
        using GravitasWorldContext context = CreateContext(mixed);
        var (body3D, body2D) = CreateBody(context, new Vector3d(4, 4, 4), planar, mixed);
        SwiftList<WorldVoxelIndex> coordinates = Coordinates(body3D, body2D, mixed);
        WorldVoxelIndex removedCoordinate = coordinates[0];
        context.World.TryGetVoxel(removedCoordinate, out Voxel? voxel).Should().BeTrue();
        bool removed = mixed ? voxel!.TryRemovePartition<PhysicsMixedPartition>()
            : planar ? voxel!.TryRemovePartition<PhysicsPartition2D>()
            : voxel!.TryRemovePartition<PhysicsPartition>();
        removed.Should().BeTrue();

        Move(body3D, body2D, new Vector3d(Fixed64.FromFraction(17, 4), (Fixed64)4, (Fixed64)4), mixed);

        Coordinates(body3D, body2D, mixed).Should().Contain(removedCoordinate);
        GetDynamicMembership(context, removedCoordinate, planar, mixed)
            .Contains(planar ? body2D!.Collider.Id : body3D!.Collider.Id).Should().BeTrue();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void UnfreezingInitializedBody_ShouldSynchronizeSurvivingAwakeMembership(bool planar)
    {
        using GravitasWorldContext context = CreateContext(mixed: false);
        var (body3D, body2D) = CreateBody(context, new Vector3d(4, 4, 4), planar, mixed: false, frozen: true);
        SwiftList<WorldVoxelIndex> coordinates = Coordinates(body3D, body2D, mixed: false);
        WorldVoxelIndex surviving = coordinates[0];
        ContainsAwake(context, surviving, body3D, body2D).Should().BeFalse();

        if (planar)
            body2D!.FreezeAxes = BodyFreezeAxes2D.None;
        else
            body3D!.FreezeAxes = BodyFreezeAxes3D.None;

        ContainsAwake(context, surviving, body3D, body2D).Should().BeTrue();
        Resize(body3D, body2D);
        coordinates.Should().Contain(surviving);
        ContainsAwake(context, surviving, body3D, body2D).Should().BeTrue();

        if (planar)
            body2D!.FreezeAxes = BodyFreezeAxes2D.All;
        else
            body3D!.FreezeAxes = BodyFreezeAxes3D.All;
        ContainsAwake(context, surviving, body3D, body2D).Should().BeFalse();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ChangedShape_ShouldResynchronizeSurvivingAwakeMembership(bool planar)
    {
        using GravitasWorldContext context = CreateContext(mixed: false);
        var (body3D, body2D) = CreateBody(context, new Vector3d(4, 4, 4), planar, mixed: false, frozen: true);
        WorldVoxelIndex surviving = Coordinates(body3D, body2D, mixed: false)[0];
        int id = planar ? body2D!.Collider.Id : body3D!.Collider.Id;
        // A surviving membership can retain an earlier disabled awake bit.
        // Shape refresh must reconcile it against the body without removing ID.
        if (planar)
            body2D!.FreezeAxes = BodyFreezeAxes2D.None;
        else
            body3D!.FreezeAxes = BodyFreezeAxes3D.None;
        context.World.TryGetVoxel(surviving, out Voxel? voxel).Should().BeTrue();
        if (planar)
        {
            voxel!.TryGetPartition(out PhysicsPartition2D? partition).Should().BeTrue();
            partition!.SetDynamicObjectAwake(id, awake: false);
        }
        else
        {
            voxel!.TryGetPartition(out PhysicsPartition? partition).Should().BeTrue();
            partition!.SetDynamicObjectAwake(id, awake: false);
        }
        ContainsAwake(context, surviving, body3D, body2D).Should().BeFalse();
        var enumeration = GetDynamicMembership(context, surviving, planar, mixed: false).GetEnumerator();

        Resize(body3D, body2D);

        Action advance = () => enumeration.MoveNext();
        advance.Should().NotThrow();
        enumeration.Current.Should().Be(id);
        Coordinates(body3D, body2D, mixed: false).Should().Contain(surviving);
        ContainsAwake(context, surviving, body3D, body2D).Should().BeTrue();
    }

    private static void Resize(SolidBody? spatial, SolidBody2D? planar)
    {
        if (planar != null)
        {
            ((LSCircleCollider2D)planar.Collider).Radius = Fixed64.One;
            planar.Collider.Simulate();
        }
        else
        {
            spatial!.Collider.Radius = Fixed64.One;
            spatial.Collider.Simulate();
        }
    }

    private static bool ContainsAwake(
        GravitasWorldContext context, WorldVoxelIndex coordinate, SolidBody? spatial, SolidBody2D? planar)
    {
        context.World.TryGetVoxel(coordinate, out Voxel? voxel).Should().BeTrue();
        if (planar != null)
        {
            voxel!.TryGetPartition(out PhysicsPartition2D? partition).Should().BeTrue();
            return partition!.ContainsAwakeDynamicObject(planar.Collider.Id);
        }
        voxel!.TryGetPartition(out PhysicsPartition? partition3D).Should().BeTrue();
        return partition3D!.ContainsAwakeDynamicObject(spatial!.Collider.Id);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MixedChangedBounds_ShouldSynchronizeBodyAwakeMembership(bool planar)
    {
        using GravitasWorldContext context = CreateContext(mixed: true);
        var (body3D, body2D) = CreateBody(context, new Vector3d(4, 4, 4), planar, mixed: true);
        WorldVoxelIndex surviving = Coordinates(body3D, body2D, mixed: true)[0];
        if (planar)
        {
            body2D!.Sleep();
            ((LSCircleCollider2D)body2D.Collider).Radius = Fixed64.One;
            context.MixedCollisions.Refresh2DColliderPartition(body2D.Collider);
        }
        else
        {
            body3D!.Sleep();
            body3D.Collider.Radius = Fixed64.One;
            body3D.Collider.Simulate();
            context.MixedCollisions.Refresh3DColliderPartition(body3D.Collider);
        }

        Coordinates(body3D, body2D, mixed: true).Should().Contain(surviving);
        context.World.TryGetVoxel(surviving, out Voxel? voxel).Should().BeTrue();
        voxel!.TryGetPartition(out PhysicsMixedPartition? partition).Should().BeTrue();
        ((planar ? partition!.ContainedAwakeDynamic2DObjects : partition!.ContainedAwakeDynamic3DObjects)?
            .Contains(planar ? body2D!.Collider.Id : body3D!.Collider.Id) ?? false)
            .Should().Be(planar ? body2D!.IsAwakeForCollision : body3D!.IsAwakeForCollision);
    }

    private static (SolidBody? Spatial, SolidBody2D? Planar) CreateBody(
        GravitasWorldContext context, Vector3d position, bool planar, bool mixed, bool frozen = false)
    {
        var agent = new TestMatterAgent(context,
            new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One));
        SolidBody? body3D = null;
        SolidBody2D? body2D = null;
        if (planar)
        {
            body2D = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
            {
                Mass = Fixed64.One,
                FreezeAxes = frozen ? BodyFreezeAxes2D.All : BodyFreezeAxes2D.None
            };
            body2D.Initialize(position.ToVector2d());
            if (mixed)
                context.MixedCollisions.Refresh2DColliderPartition(body2D.Collider);
        }
        else
        {
            body3D = new SolidBody(agent, new LSSphereCollider { Radius = Fixed64.Half })
            {
                Mass = Fixed64.One,
                FreezeAxes = frozen ? BodyFreezeAxes3D.All : BodyFreezeAxes3D.None
            };
            body3D.Initialize(position, FixedQuaternion.Identity);
            if (mixed)
                context.MixedCollisions.Refresh3DColliderPartition(body3D.Collider);
        }
        return (body3D, body2D);
    }

    private static SwiftList<WorldVoxelIndex> Coordinates(SolidBody? spatial, SolidBody2D? planar, bool mixed) =>
        (planar != null ? (mixed ? planar.Collider.MixedPartitionCoordinates : planar.Collider.PartitionCoordinates)
            : (mixed ? spatial!.Collider.MixedPartitionCoordinates : spatial!.Collider.PartitionCoordinates))!;

    private static void Move(SolidBody? spatial, SolidBody2D? planar, Vector3d position, bool mixed)
    {
        if (planar != null)
        {
            planar.ResetPosition(position.ToVector2d());
            if (mixed)
                planar.Context.MixedCollisions.Refresh2DColliderPartition(planar.Collider);
        }
        else
        {
            spatial!.ResetPosition(position);
            spatial.Collider.Simulate();
            if (mixed)
                spatial.Context.MixedCollisions.Refresh3DColliderPartition(spatial.Collider);
        }
    }

    private static SwiftHashSet<int> GetDynamicMembership(
        GravitasWorldContext context, WorldVoxelIndex coordinate, bool planar, bool mixed)
    {
        SwiftHashSet<int>? membership = FindDynamicMembership(context, coordinate, planar, mixed);
        membership.Should().NotBeNull();
        return membership!;
    }

    private static SwiftHashSet<int>? FindDynamicMembership(
        GravitasWorldContext context, WorldVoxelIndex coordinate, bool planar, bool mixed)
    {
        if (!context.World.TryGetVoxel(coordinate, out Voxel? voxel))
            return null;
        if (mixed)
        {
            return voxel!.TryGetPartition(out PhysicsMixedPartition? partition)
                ? (planar ? partition!.ContainedDynamic2DObjects : partition!.ContainedDynamic3DObjects) : null;
        }
        if (planar)
        {
            return voxel!.TryGetPartition(out PhysicsPartition2D? partition) ? partition!.ContainedDynamicObjects : null;
        }
        return voxel!.TryGetPartition(out PhysicsPartition? spatialPartition) ? spatialPartition!.ContainedDynamicObjects : null;
    }
}
