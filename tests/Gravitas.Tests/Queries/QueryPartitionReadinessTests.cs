using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using GridForge.Grids;
using GridForge.Grids.Storage;
using GridForge.Spatial;
using System;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class QueryPartitionReadinessTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Query_AfterOrdinaryRegistration_DoesNotRepartitionCurrentMembership(bool twoDimensional)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.World.TryAddGrid(CreateConfiguration(), out _).Should().BeTrue();
        object collider = CreateCollider(context, twoDimensional);
        uint version = collider is LSCollider2D planar ? planar.BroadPhaseVersion : ((LSCollider)collider).BroadPhaseVersion;

        Query(context, twoDimensional).Should().BeSameAs(collider);

        uint currentVersion = collider is LSCollider2D currentPlanar ? currentPlanar.BroadPhaseVersion : ((LSCollider)collider).BroadPhaseVersion;
        currentVersion.Should().Be(version);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Query_AfterGridReplacement_RetainsUnmovedCollider(bool twoDimensional)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        GridConfiguration configuration = CreateConfiguration();
        context.World.TryAddGrid(configuration, out _).Should().BeTrue();
        object collider = CreateCollider(context, twoDimensional);
        Query(context, twoDimensional).Should().BeSameAs(collider);

        context.World.TryRemoveGrid(0).Should().BeTrue();
        context.World.TryAddGrid(configuration, out _).Should().BeTrue();

        Query(context, twoDimensional).Should().BeSameAs(collider);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Query_AfterSparseVoxelAdded_DiscoversUnmovedCollider(bool twoDimensional)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.World.TryAddGrid(CreateConfiguration(sparse: true), out ushort gridIndex).Should().BeTrue();
        VoxelGrid grid = context.World.ActiveGrids[gridIndex];
        object collider = CreateCollider(context, twoDimensional);
        Query(context, twoDimensional).Should().BeNull();

        grid!.TryAddVoxel(new VoxelIndex(0, 0, 0), out _).Should().BeTrue();

        Query(context, twoDimensional).Should().BeSameAs(collider);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Query_AfterSparseVoxelRemovalAndReaddition_TracksPhysicalPresence(bool twoDimensional)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.World.TryAddGrid(CreateConfiguration(sparse: true), out ushort gridIndex).Should().BeTrue();
        VoxelGrid grid = context.World.ActiveGrids[gridIndex];
        VoxelIndex index = new(0, 0, 0);
        grid.TryAddVoxel(index, out Voxel? voxel).Should().BeTrue();
        object collider = CreateCollider(context, twoDimensional);
        Query(context, twoDimensional).Should().BeSameAs(collider);

        // GridForge requires hosts to detach the voxel's runtime payloads before
        // removing sparse storage; the collider registration itself stays alive.
        if (twoDimensional)
            voxel!.TryRemovePartition<PhysicsPartition2D>().Should().BeTrue();
        else
            voxel!.TryRemovePartition<PhysicsPartition>().Should().BeTrue();
        grid.TryRemoveVoxel(index).Should().BeTrue();
        Query(context, twoDimensional).Should().BeNull();
        if (!twoDimensional)
            context.Query3D.OverlapCircle(Vector3d.Zero, Fixed64.One, out _, PhysicsLayerMask.All).Should().BeFalse();

        grid.TryAddVoxel(index, out _).Should().BeTrue();
        Query(context, twoDimensional).Should().BeSameAs(collider);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Query_AfterWorldResetAndGridReplacement_RebindsExistingRegistration(bool twoDimensional)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        GridConfiguration configuration = CreateConfiguration();
        context.World.TryAddGrid(configuration, out _).Should().BeTrue();
        object collider = CreateCollider(context, twoDimensional);
        Query(context, twoDimensional).Should().BeSameAs(collider);

        context.World.Reset();
        Query(context, twoDimensional).Should().BeNull();
        context.World.TryAddGrid(configuration, out _).Should().BeTrue();

        Query(context, twoDimensional).Should().BeSameAs(collider);
    }

    [Fact]
    public void RefreshQueryPartitions_WithDisposedHostWorld_DoesNotCertifyReadiness()
    {
        using var world = new GridWorld();
        using GravitasWorldContext context = GravitasWorldContext.Attach(world);
        context.RefreshQueryPartitions().Should().BeTrue();

        world.Dispose();

        context.IsDisposed.Should().BeFalse();
        context.RefreshQueryPartitions().Should().BeFalse();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Query_WithOnlyObstacleChanges_DoesNotRepartitionUnmovedCollider(bool twoDimensional)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.World.TryAddGrid(CreateConfiguration(), out ushort gridIndex).Should().BeTrue();
        object collider = CreateCollider(context, twoDimensional);
        Query(context, twoDimensional).Should().BeSameAs(collider);
        uint version = collider is LSCollider2D planar ? planar.BroadPhaseVersion : ((LSCollider)collider).BroadPhaseVersion;

        VoxelGrid grid = context.World.ActiveGrids[gridIndex];
        grid.TryAddObstacle(Vector3d.Zero, context.World.AllocateObstacleToken()).Should().BeTrue();
        for (int i = 0; i < 3; i++)
            Query(context, twoDimensional).Should().BeSameAs(collider);

        uint currentVersion = collider is LSCollider2D currentPlanar ? currentPlanar.BroadPhaseVersion : ((LSCollider)collider).BroadPhaseVersion;
        currentVersion.Should().Be(version);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Query_AfterGridReplacement_DoesNotPublishPendingAuthoredShape(bool twoDimensional)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        GridConfiguration configuration = CreateConfiguration();
        context.World.TryAddGrid(configuration, out _).Should().BeTrue();
        object collider = CreateCollider(context, twoDimensional);
        Query(context, twoDimensional).Should().BeSameAs(collider);
        if (collider is LSCircleCollider2D circle)
            circle.Radius = (Fixed64)2;
        else
            ((LSSphereCollider)collider).Radius = (Fixed64)2;

        context.World.TryRemoveGrid(0).Should().BeTrue();
        context.World.TryAddGrid(configuration, out _).Should().BeTrue();
        Query(context, twoDimensional).Should().BeSameAs(collider);

        Fixed64 publishedRadius = collider is LSCircleCollider2D planar ? planar.ScaledRadius : ((LSSphereCollider)collider).ScaledRadius;
        publishedRadius.Should().Be(Fixed64.Half);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CompletedStep_WithRecurrentGridReplacement_ReconcilesBeforeAnyQuery(bool twoDimensional)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = twoDimensional ? PhysicsRuntimeMode.TwoD : PhysicsRuntimeMode.ThreeD;
        GridConfiguration configuration = CreateConfiguration();
        context.World.TryAddGrid(configuration, out _).Should().BeTrue();
        object collider = CreateCollider(context, twoDimensional);
        Query(context, twoDimensional).Should().BeSameAs(collider);
        using IDisposable hook = context.RegisterOnLateSimulate("query-readiness-test", 0, () =>
        {
            context.World.TryRemoveGrid(0).Should().BeTrue();
            context.World.TryAddGrid(configuration, out _).Should().BeTrue();
        });

        for (int frame = 0; frame < 2; frame++)
        {
            context.Simulate();
            context.LateSimulate();
            long generation = context.World.ActiveGrids[0].SpawnToken;
            var coordinates = collider is LSCollider2D planar ? planar.PartitionCoordinates : ((LSCollider)collider).PartitionCoordinates;
            coordinates.Should().NotBeNullOrEmpty();
            coordinates.Should().OnlyContain(coordinate => coordinate.GridSpawnToken == generation);
            Query(context, twoDimensional).Should().BeSameAs(collider);
        }
    }

    [Fact]
    public void RefreshQueryPartitions_DuringUnobservedCommittedCallback_RejectsUntilNotificationCompletes()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        bool? readyDuringCallback = null;
        context.World.OnActiveGridAdded += _ => readyDuringCallback = context.RefreshQueryPartitions();

        context.World.TryAddGrid(CreateConfiguration(), out _).Should().BeTrue();

        readyDuringCallback.Should().BeFalse();
        context.RefreshQueryPartitions().Should().BeTrue();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ProjectedCircleQuery_AfterPhysicalGridChange_DiscoversUnmovedCollider(bool sparse)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        GridConfiguration configuration = CreateConfiguration(sparse);
        context.World.TryAddGrid(configuration, out ushort gridIndex).Should().BeTrue();
        object collider = CreateCollider(context, twoDimensional: false);
        if (sparse)
            context.World.ActiveGrids[gridIndex].TryAddVoxel(new VoxelIndex(0, 0, 0), out _).Should().BeTrue();
        else
        {
            context.World.TryRemoveGrid(gridIndex).Should().BeTrue();
            context.World.TryAddGrid(configuration, out _).Should().BeTrue();
        }

        context.Query3D.OverlapCircle(Vector3d.Zero, Fixed64.One, out Physics3DHit hit, PhysicsLayerMask.All)
            .Should().BeTrue();
        hit.Collider.Should().BeSameAs(collider);
    }

    private static GridConfiguration CreateConfiguration(bool sparse = false) =>
        new(Vector3d.Zero, new Vector3d(4, 4, 4),
            storageKind: sparse ? GridStorageKind.Sparse : GridStorageKind.Dense);

    private static object CreateCollider(GravitasWorldContext context, bool twoDimensional)
    {
        var agent = new TestMatterAgent(context);
        if (twoDimensional)
        {
            var circle = new LSCircleCollider2D(Fixed64.Half);
            circle.InitializeWithNoBody(agent);
            return circle;
        }

        var sphere = new LSSphereCollider { Radius = Fixed64.Half };
        sphere.InitializeWithNoBody(agent);
        return sphere;
    }

    private static object? Query(GravitasWorldContext context, bool twoDimensional)
    {
        if (twoDimensional)
        {
            return context.Query2D.SweepCircle(
                -Vector2d.Right * (Fixed64)2, Vector2d.Right * (Fixed64)2, Fixed64.Half,
                out Physics2DHit hit) ? hit.Collider : null;
        }

        return context.Query3D.SweepSphere(
            -Vector3d.Right * (Fixed64)2, Fixed64.Half, Vector3d.Right, (Fixed64)4,
            out Physics3DHit hit3D, PhysicsLayerMask.All) ? hit3D.Collider : null;
    }
}
