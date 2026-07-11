using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using GridForge;
using GridForge.Configuration;
using GridForge.Grids;
using GridForge.Spatial;
using SwiftCollections.Diagnostics;
using System;
using System.Collections.Generic;
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
    public void ClearPartitionedObject_ShouldRetainEmptyPartitionsWithoutPoolingTwice()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider collider = CreateDynamicSphere(context);
        int inactiveBeforeClear = context.Collisions.InactivePartitionCount;
        var coordinates = new List<WorldVoxelIndex>();
        for (int i = 0; i < collider.PartitionCoordinates!.Count; i++)
            coordinates.Add(collider.PartitionCoordinates[i]);

        context.Collisions.ClearPartitionedObject(collider, force: true).Should().BeTrue();
        int inactiveAfterClear = context.Collisions.InactivePartitionCount;
        context.Collisions.ClearPartitionedObject(collider, force: true).Should().BeFalse();

        collider.IsPartitioned.Should().BeFalse();
        collider.PartitionCoordinates.Should().BeEmpty();
        context.Collisions.ActivePartitionCount.Should().Be(0);
        inactiveAfterClear.Should().Be(inactiveBeforeClear);
        context.Collisions.InactivePartitionCount.Should().Be(inactiveAfterClear);
        for (int i = 0; i < coordinates.Count; i++)
        {
            context.World.TryGetVoxel(coordinates[i], out Voxel? voxel).Should().BeTrue();
            voxel!.TryGetPartition(out PhysicsPartition? partition).Should().BeTrue();
            partition!.ContainedDynamicObjects!.Count.Should().Be(0);
            partition.IsAllocated.Should().BeFalse();
        }
    }

    [Fact]
    public void Deactivate_WithZeroRetirementBudget_ShouldKeepRetainedPartitionsUntilBudgetRestored()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RetainedPartitionTimeToKillFrames = 1;
        context.Settings.RetainedPartitionRetirementSweepBudget = 0;
        LSSphereCollider collider = CreateDynamicSphere(context);
        int retainedBeforeDeactivate = context.Collisions.RetainedPartitionCount;

        collider.Deactivate();
        Step(context);

        context.Collisions.ActivePartitionCount.Should().Be(0);
        context.Collisions.RetainedPartitionCount.Should().Be(retainedBeforeDeactivate);
        context.Collisions.InactivePartitionCount.Should().Be(0);

        context.Settings.RetainedPartitionRetirementSweepBudget = 1024;
        Step(context);

        context.Collisions.RetainedPartitionCount.Should().BeLessThan(retainedBeforeDeactivate);
        context.Collisions.InactivePartitionCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void Reset_WithRetainedPartitionAlreadyDetached_ShouldReleaseWithoutVoxelDetach()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider collider = CreateDynamicSphere(context);
        WorldVoxelIndex coordinate = collider.PartitionCoordinates![0];
        context.World.TryGetVoxel(coordinate, out Voxel? voxel).Should().BeTrue();
        voxel!.TryGetPartition(out PhysicsPartition? partition).Should().BeTrue();
        voxel.TryRemovePartition<PhysicsPartition>().Should().BeTrue();

        context.Reset();

        context.Collisions.RetainedPartitionCount.Should().Be(0);
        context.Collisions.ActivePartitionCount.Should().Be(0);
        context.Collisions.InactivePartitionCount.Should().Be(0);
        partition!.IsAllocated.Should().BeFalse();
        Action readOwner = () => _ = partition.Owner;
        readOwner.Should().Throw<InvalidOperationException>();
    }

    [Fact]
    public void RentPartition_WhenPoolEmptyAndRetainedEmptyPartitionExists_ShouldReuseRetiredPartition()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RetainedPartitionRetirementSweepBudget = 0;
        LSSphereCollider collider = CreateDynamicSphere(context);

        collider.Deactivate();
        Step(context);

        context.Collisions.ActivePartitionCount.Should().Be(0);
        context.Collisions.InactivePartitionCount.Should().Be(0);
        context.Collisions.RetainedPartitionCount.Should().BeGreaterThan(0);

        int retainedBeforeRent = context.Collisions.RetainedPartitionCount;
        PhysicsPartition rented = context.Collisions.RentPartition();

        rented.Owner.Should().BeSameAs(context.Collisions);
        context.Collisions.RetainedPartitionCount.Should().Be(retainedBeforeRent - 1);
        context.Collisions.InactivePartitionCount.Should().Be(0);
        context.Collisions.ReleasePartition(rented);
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

    [Fact]
    public void ClearPartitionedObject_AfterVoxelPartitionDetach_ShouldNormalizeStaleColliderState()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider collider = CreateDynamicSphere(context);
        WorldVoxelIndex coordinate = collider.PartitionCoordinates![0];
        context.World.TryGetVoxel(coordinate, out Voxel? voxel).Should().BeTrue();
        voxel!.TryRemovePartition<PhysicsPartition>().Should().BeTrue();

        context.Collisions.ClearPartitionedObject(collider, force: true).Should().BeTrue();

        collider.IsPartitioned.Should().BeFalse();
        collider.PartitionCoordinates.Should().BeEmpty();
        context.Collisions.ActivePartitionCount.Should().Be(0);
    }

    [Fact]
    public void ClearPartitionedObject_AfterGridRemoval_ShouldNormalizeStaleColliderState()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider collider = CreateDynamicSphere(context);

        List<(DiagnosticLevel Level, string Message)> entries = CaptureGridLogs(() =>
        {
            context.World.TryRemoveGrid(0).Should().BeTrue();
            context.Collisions.ClearPartitionedObject(collider, force: true).Should().BeTrue();
        });

        collider.IsPartitioned.Should().BeFalse();
        collider.PartitionCoordinates.Should().BeEmpty();
        context.Collisions.ActivePartitionCount.Should().Be(0);
        entries.Should().NotContain(entry => entry.Level == DiagnosticLevel.Error);
    }

    [Fact]
    public void ClearPartitionedObject_AfterGridSlotReplacement_ShouldIgnoreStaleVoxelAddresses()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider collider = CreateDynamicSphere(context);

        List<(DiagnosticLevel Level, string Message)> entries = CaptureGridLogs(() =>
        {
            ReplacePrimaryGrid(context);
            context.Collisions.ClearPartitionedObject(collider, force: true).Should().BeTrue();
        });

        collider.IsPartitioned.Should().BeFalse();
        collider.PartitionCoordinates.Should().BeEmpty();
        entries.Should().NotContain(entry => entry.Level == DiagnosticLevel.Error);
    }

    [Fact]
    public void RefreshPartitionAwakeState_WithMissingGridOrVoxelPartition_ShouldIgnoreStaleCoordinates()
    {
        using GravitasWorldContext detachedPartitionContext = GravitasWorldContext.CreateOwned();
        LSSphereCollider detachedPartitionCollider = CreateDynamicSphere(detachedPartitionContext);
        WorldVoxelIndex coordinate = detachedPartitionCollider.PartitionCoordinates![0];
        detachedPartitionContext.World.TryGetVoxel(coordinate, out Voxel? voxel).Should().BeTrue();
        voxel!.TryRemovePartition<PhysicsPartition>().Should().BeTrue();

        detachedPartitionContext.Collisions.RefreshPartitionAwakeState(detachedPartitionCollider);

        using GravitasWorldContext removedGridContext = GravitasWorldContext.CreateOwned();
        LSSphereCollider removedGridCollider = CreateDynamicSphere(removedGridContext);
        List<(DiagnosticLevel Level, string Message)> entries = CaptureGridLogs(() =>
        {
            removedGridContext.World.TryRemoveGrid(0).Should().BeTrue();
            removedGridContext.Collisions.RefreshPartitionAwakeState(removedGridCollider);
        });

        entries.Should().NotContain(entry => entry.Level == DiagnosticLevel.Error);

        using GravitasWorldContext replacementGridContext = GravitasWorldContext.CreateOwned();
        LSSphereCollider replacementGridCollider = CreateDynamicSphere(replacementGridContext);
        entries = CaptureGridLogs(() =>
        {
            ReplacePrimaryGrid(replacementGridContext);
            replacementGridContext.Collisions.RefreshPartitionAwakeState(replacementGridCollider);
        });

        entries.Should().NotContain(entry => entry.Level == DiagnosticLevel.Error);
    }

    [Fact]
    public void RentPartition_AfterExplicitRelease_ShouldReuseInactivePoolEntry()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        PhysicsPartition first = context.Collisions.RentPartition();
        context.Collisions.ReleasePartition(first);

        PhysicsPartition second = context.Collisions.RentPartition();

        second.Should().BeSameAs(first);
        context.Collisions.InactivePartitionCount.Should().Be(0);
        context.Collisions.ReleasePartition(second);
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

    private static void Step(GravitasWorldContext context)
    {
        context.Simulate();
        context.LateSimulate();
    }

    private static void EnsureGrid(GravitasWorldContext context)
    {
        GridConfiguration configuration = new(
            new Vector3d(-2, -2, -2),
            new Vector3d(2, 2, 2));

        context.World.TryAddGrid(configuration, out _).Should().BeTrue();
    }

    private static void ReplacePrimaryGrid(GravitasWorldContext context)
    {
        GridConfiguration spareConfiguration = new(
            new Vector3d(10, 10, 10),
            new Vector3d(14, 14, 14));
        context.World.TryAddGrid(spareConfiguration, out _).Should().BeTrue();
        context.World.TryRemoveGrid(0).Should().BeTrue();

        GridConfiguration replacementConfiguration = new(
            new Vector3d(20, 20, 20),
            new Vector3d(21, 21, 21));
        context.World.TryAddGrid(replacementConfiguration, out ushort replacementIndex).Should().BeTrue();
        replacementIndex.Should().Be(0);
    }

    private static List<(DiagnosticLevel Level, string Message)> CaptureGridLogs(Action action)
    {
        Action<DiagnosticLevel, string, string> originalHandler = GridForgeLogger.LogHandler;
        var entries = new List<(DiagnosticLevel Level, string Message)>();
        try
        {
            GridForgeLogger.LogHandler = (level, message, _) => entries.Add((level, message));
            action();
            return entries;
        }
        finally
        {
            GridForgeLogger.LogHandler = originalHandler;
        }
    }
}
