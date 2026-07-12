using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge;
using GridForge.Configuration;
using GridForge.Grids;
using GridForge.Spatial;
using SwiftCollections;
using SwiftCollections.Diagnostics;
using System;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class Physics2DPartitionBroadPhaseTests
{
    [Fact]
    public void CanonicalSharedPartitionPolicy_ShouldUseMinimumVoxelPerGridIdentity()
    {
        WorldVoxelIndex current = CreateWorldVoxel(1, 10, 1, 0, 0);
        var first = new LSCircleCollider2D(Fixed64.Half);
        var second = new LSCircleCollider2D(Fixed64.Half);

        GravitasPhysics2DService.IsCanonicalSharedPartition(first, second, current).Should().BeTrue();

        SwiftList<WorldVoxelIndex> firstCoordinates = first.GetOrCreatePartitionCoordinates();
        firstCoordinates.Add(CreateWorldVoxel(1, 11, -8, 0, -8));
        firstCoordinates.Add(CreateWorldVoxel(1, 10, 0, 0, 1));
        firstCoordinates.Add(CreateWorldVoxel(1, 10, 2, 0, 0));
        SwiftList<WorldVoxelIndex> secondCoordinates = second.GetOrCreatePartitionCoordinates();
        secondCoordinates.Add(CreateWorldVoxel(1, 10, 1, 0, 0));
        secondCoordinates.Add(CreateWorldVoxel(1, 10, 3, 0, 4));

        GravitasPhysics2DService.IsCanonicalSharedPartition(first, second, current).Should().BeTrue();
        GravitasPhysics2DService
            .IsCanonicalSharedPartition(first, second, CreateWorldVoxel(1, 10, 0, 0, 1))
            .Should()
            .BeFalse();
        GravitasPhysics2DService
            .IsCanonicalSharedPartition(first, second, CreateWorldVoxel(1, 12, 0, 0, 1))
            .Should()
            .BeTrue();
    }

    [Fact]
    public void TryGetMinimumVoxelIndexForGrid_ShouldIgnoreOtherGridIdentities()
    {
        var coordinates = new SwiftList<WorldVoxelIndex>();
        WorldVoxelIndex identity = CreateWorldVoxel(2, 20, 0, 0, 0);
        coordinates.Add(new WorldVoxelIndex(2, 1, 20, new VoxelIndex(-11, 0, -11)));
        coordinates.Add(CreateWorldVoxel(2, 21, -9, 0, -9));
        coordinates.Add(CreateWorldVoxel(2, 20, 3, 0, 5));
        coordinates.Add(CreateWorldVoxel(2, 20, 1, 0, 7));
        coordinates.Add(CreateWorldVoxel(2, 20, 2, 0, -1));

        GravitasPhysics2DService.TryGetMinimumVoxelIndexForGrid(coordinates, identity, out VoxelIndex minimum)
            .Should()
            .BeTrue();
        minimum.x.Should().Be(1);
        minimum.z.Should().Be(-1);

        GravitasPhysics2DService
            .TryGetMinimumVoxelIndexForGrid(coordinates, CreateWorldVoxel(2, 22, 0, 0, 0), out _)
            .Should()
            .BeFalse();
    }

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
    public void Simulate_With2DCollidersSharingMultiplePartitions_ShouldProcessPairOnce()
    {
        using GravitasWorldContext context = CreateContext(extent: 32);
        SolidBody2D dynamicBody = CreateBody(
            context,
            new LSAABBoxCollider2D(new Vector2d((Fixed64)8, (Fixed64)8)),
            Vector2d.Zero,
            immovable: false);
        SolidBody2D staticBody = CreateBody(
            context,
            new LSAABBoxCollider2D(new Vector2d((Fixed64)8, (Fixed64)8)),
            new Vector2d(Fixed64.Half, Fixed64.Zero),
            immovable: true);
        int entered = 0;
        dynamicBody.Collider.OnContactEnter += otherBody =>
        {
            otherBody.Should().BeSameAs(staticBody);
            entered++;
        };

        Step(context);

        context.Physics2D.LastBroadPhaseCandidateCount.Should().Be(1);
        entered.Should().Be(1);
        dynamicBody.Collider.TryGetCollisionPair(staticBody.Collider.Id, out CollisionPair2D? pair).Should().BeTrue();
        pair!.Manifold.HasContact.Should().BeTrue();
        pair.LastFrame.Should().Be(context.FrameCount);
    }

    [Fact]
    public void ProcessPartitionCandidate_WithStaleIdsOrDuplicatePair_ShouldRemainDeterministic()
    {
        using GravitasWorldContext context = CreateContext(extent: 32);
        context.Settings.PoolingEnabled = false;
        SolidBody2D dynamicBody = CreateCircle(context, Vector2d.Zero, immovable: false);
        SolidBody2D staticBody = CreateCircle(context, new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: true);
        WorldVoxelIndex partitionIndex = ResolveCanonicalSharedPartition(dynamicBody.Collider, staticBody.Collider);

        context.Physics2D.ProcessPartitionCandidate(-1, staticBody.Collider.Id, partitionIndex);
        context.Physics2D.ProcessPartitionCandidate(dynamicBody.Collider.Id, -1, partitionIndex);
        context.Physics2D.ProcessPartitionCandidate(dynamicBody.Collider.Id, staticBody.Collider.Id, partitionIndex);
        context.Physics2D.ProcessPartitionCandidate(dynamicBody.Collider.Id, staticBody.Collider.Id, partitionIndex);

        context.Physics2D.LastBroadPhaseCandidateCount.Should().Be(1);
        dynamicBody.Collider.TryGetCollisionPair(staticBody.Collider.Id, out CollisionPair2D? pair).Should().BeTrue();
        pair!.Manifold.HasContact.Should().BeTrue();
        pair.LastFrame.Should().Be(context.FrameCount);
    }

    [Fact]
    public void BoundsQueryCandidates_ShouldFilterInactiveWrongLayerDuplicateStaticStyleAndBoundsMisses()
    {
        using GravitasWorldContext context = CreateContext(extent: 32);
        SolidBody2D included = CreateBody(
            context,
            new LSAABBoxCollider2D(new Vector2d((Fixed64)8, (Fixed64)8)),
            Vector2d.Zero,
            immovable: false);
        SolidBody2D inactive = CreateCircle(context, new Vector2d(Fixed64.One, Fixed64.Zero), immovable: false);
        SolidBody2D wrongLayer = CreateCircle(context, new Vector2d(Fixed64.Zero, Fixed64.One), immovable: false);
        SolidBody2D staticStyle = CreateCircle(context, new Vector2d((Fixed64)3, Fixed64.Zero), immovable: false);
        SolidBody2D boundsMiss = CreateBody(
            context,
            new LSAABBoxCollider2D(new Vector2d((Fixed64)8, (Fixed64)8)),
            new Vector2d((Fixed64)7, Fixed64.Zero),
            immovable: false);
        SolidBody2D negativeXMiss = CreateBody(
            context,
            new LSAABBoxCollider2D(new Vector2d((Fixed64)8, (Fixed64)8)),
            new Vector2d((Fixed64)(-7), Fixed64.Zero),
            immovable: false);
        SolidBody2D positiveYMiss = CreateBody(
            context,
            new LSAABBoxCollider2D(new Vector2d((Fixed64)8, (Fixed64)8)),
            new Vector2d(Fixed64.Zero, (Fixed64)7),
            immovable: false);
        SolidBody2D negativeYMiss = CreateBody(
            context,
            new LSAABBoxCollider2D(new Vector2d((Fixed64)8, (Fixed64)8)),
            new Vector2d(Fixed64.Zero, (Fixed64)(-7)),
            immovable: false);
        var hits = new SwiftList<Physics2DHit>();

        inactive.Collider.IsActive = false;
        wrongLayer.Collider.Layer = new PhysicsLayer(1);
        staticStyle.IsKinematic = true;
        for (int i = 0; i < included.Collider.PartitionCoordinates!.Count; i++)
        {
            context.World.TryGetVoxel(included.Collider.PartitionCoordinates[i], out Voxel? voxel).Should().BeTrue();
            voxel!.TryGetPartition(out PhysicsPartition2D? partition).Should().BeTrue();
            partition!.AddDynamicObject(999);
            partition.AddDynamicObject(inactive.Collider.Id);
        }

        int count = context.Query2D.OverlapCircleAll(
            Vector2d.Zero,
            Fixed64.One,
            PhysicsLayerMask.FromLayer(0),
            hits);

        count.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(included.Collider);
        context.Query2D.LastQueryCandidateCount.Should().Be(1);
        hits.Should().NotContain(hit => ReferenceEquals(hit.Collider, inactive.Collider));
        hits.Should().NotContain(hit => ReferenceEquals(hit.Collider, wrongLayer.Collider));
        hits.Should().NotContain(hit => ReferenceEquals(hit.Collider, staticStyle.Collider));
        hits.Should().NotContain(hit => ReferenceEquals(hit.Collider, boundsMiss.Collider));
        hits.Should().NotContain(hit => ReferenceEquals(hit.Collider, negativeXMiss.Collider));
        hits.Should().NotContain(hit => ReferenceEquals(hit.Collider, positiveYMiss.Collider));
        hits.Should().NotContain(hit => ReferenceEquals(hit.Collider, negativeYMiss.Collider));

        int staticOnlyCount = context.Query2D.SweepCircleAgainstStaticAll(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            new Vector2d((Fixed64)4, Fixed64.Zero),
            Fixed64.Half,
            PhysicsLayerMask.FromLayer(0),
            hits);

        staticOnlyCount.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(staticStyle.Collider);
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
    public void RefreshColliderPartition_WithUnchangedOrInactiveCollider_ShouldKeepMembershipDeterministic()
    {
        using GravitasWorldContext context = CreateContext(extent: 32);
        SolidBody2D body = CreateCircle(context, Vector2d.Zero, immovable: false);
        WorldVoxelIndex originalCoordinate = body.Collider.PartitionCoordinates![0];

        context.Collisions2D.RefreshColliderPartition(body.Collider).Should().BeFalse();
        context.Collisions2D.PartitionCollider(body.Collider).Should().BeFalse();
        context.Collisions2D.ClearPartitionedCollider(body.Collider).Should().BeFalse();

        body.Collider.IsPartitioned.Should().BeTrue();
        body.Collider.PartitionCoordinates![0].Should().Be(originalCoordinate);

        context.Collisions2D.ClearPartitionedCollider(body.Collider, force: true).Should().BeTrue();
        body.Collider.IsActive = false;
        context.Collisions2D.PartitionCollider(body.Collider).Should().BeFalse();

        context.Collisions2D.RefreshColliderPartition(body.Collider).Should().BeFalse();

        body.Collider.IsPartitioned.Should().BeFalse();
        body.Collider.PartitionCoordinates.Should().BeEmpty();
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
    public void ShapeRefresh_During2DDistribution_ShouldIgnoreColliderMadeInactiveBeforeRefresh()
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
        LSCircleCollider2D deactivated = CreateBodylessCircle(
            context,
            new Vector2d((Fixed64)9, Fixed64.Zero),
            isTrigger: false);
        var hits = new SwiftList<Physics2DHit>();
        int entered = 0;

        trigger.OnTriggerEnter += _ =>
        {
            entered++;
            expanding.Radius = (Fixed64)9;
            expanding.Simulate();
            expanding.IsActive = false;
            deactivated.Radius = (Fixed64)10;
            deactivated.Simulate();
            deactivated.Deactivate();
        };

        Step(context);

        entered.Should().Be(1);
        context.Query2D.OverlapCircleAll(Vector2d.Zero, Fixed64.One, hits).Should().BeGreaterThan(0);
        hits.Should().NotContain(hit => ReferenceEquals(hit.Collider, expanding));
        hits.Should().NotContain(hit => ReferenceEquals(hit.Collider, deactivated));
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
    public void Deactivate_WithZeroRetirementBudget_ShouldKeepRetained2DPartitionsUntilBudgetRestored()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        context.Settings.RetainedPartitionTimeToKillFrames = 1;
        context.Settings.RetainedPartitionRetirementSweepBudget = 0;
        SolidBody2D body = CreateCircle(context, Vector2d.Zero, immovable: false);
        int retainedBeforeDeactivate = context.Collisions2D.RetainedPartitionCount;

        body.Deactivate();
        Step(context);

        context.Collisions2D.ActivePartitionCount.Should().Be(0);
        context.Collisions2D.RetainedPartitionCount.Should().Be(retainedBeforeDeactivate);
        context.Collisions2D.InactivePartitionCount.Should().Be(0);

        context.Settings.RetainedPartitionRetirementSweepBudget = 1024;
        Step(context);

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
    public void Reset_WithRetained2DPartitionAlreadyDetached_ShouldReleaseWithoutVoxelDetach()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        SolidBody2D body = CreateCircle(context, Vector2d.Zero, immovable: false);
        WorldVoxelIndex coordinate = body.Collider.PartitionCoordinates![0];
        context.World.TryGetVoxel(coordinate, out Voxel? voxel).Should().BeTrue();
        voxel!.TryGetPartition(out PhysicsPartition2D? partition).Should().BeTrue();
        voxel.TryRemovePartition<PhysicsPartition2D>().Should().BeTrue();

        context.Reset();

        context.Collisions2D.RetainedPartitionCount.Should().Be(0);
        context.Collisions2D.ActivePartitionCount.Should().Be(0);
        context.Collisions2D.InactivePartitionCount.Should().Be(0);
        partition!.IsAllocated.Should().BeFalse();
        Action readOwner = () => _ = partition.Owner;
        readOwner.Should().Throw<InvalidOperationException>();
    }

    [Fact]
    public void RentPartition_WhenPoolEmptyAndRetainedEmpty2DPartitionExists_ShouldReuseRetiredPartition()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        context.Settings.RetainedPartitionRetirementSweepBudget = 0;
        SolidBody2D body = CreateCircle(context, Vector2d.Zero, immovable: false);

        body.Deactivate();
        Step(context);

        context.Collisions2D.ActivePartitionCount.Should().Be(0);
        context.Collisions2D.InactivePartitionCount.Should().Be(0);
        context.Collisions2D.RetainedPartitionCount.Should().BeGreaterThan(0);

        int retainedBeforeRent = context.Collisions2D.RetainedPartitionCount;
        PhysicsPartition2D rented = context.Collisions2D.RentPartition();

        rented.Owner.Should().BeSameAs(context.Collisions2D);
        context.Collisions2D.RetainedPartitionCount.Should().Be(retainedBeforeRent - 1);
        context.Collisions2D.InactivePartitionCount.Should().Be(0);
        context.Collisions2D.ReleasePartition(rented);
    }

    [Fact]
    public void RentPartition_AfterExplicitRelease_ShouldReuseInactive2DPoolEntry()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        PhysicsPartition2D first = context.Collisions2D.RentPartition();
        context.Collisions2D.ReleasePartition(first);

        PhysicsPartition2D second = context.Collisions2D.RentPartition();

        second.Should().BeSameAs(first);
        context.Collisions2D.InactivePartitionCount.Should().Be(0);
        context.Collisions2D.ReleasePartition(second);
    }

    [Fact]
    public void ClearPartitionedCollider_WithMissingGridVoxelOrPartition_ShouldNormalizeStateWithoutErrors()
    {
        using GravitasWorldContext detachedContext = CreateContext(extent: 16);
        SolidBody2D detached = CreateCircle(detachedContext, Vector2d.Zero, immovable: false);
        WorldVoxelIndex coordinate = detached.Collider.PartitionCoordinates![0];
        detachedContext.World.TryGetVoxel(coordinate, out Voxel? voxel).Should().BeTrue();
        voxel!.TryRemovePartition<PhysicsPartition2D>().Should().BeTrue();
        detachedContext.Collisions2D.ClearPartitionedCollider(detached.Collider, force: true).Should().BeTrue();
        detached.Collider.IsPartitioned.Should().BeFalse();

        using GravitasWorldContext removedContext = CreateContext(extent: 16);
        SolidBody2D removed = CreateCircle(removedContext, Vector2d.Zero, immovable: false);
        List<(DiagnosticLevel Level, string Message)> entries = CaptureGridLogs(() =>
        {
            removedContext.World.TryRemoveGrid(0).Should().BeTrue();
            removedContext.Collisions2D.ClearPartitionedCollider(removed.Collider, force: true).Should().BeTrue();
        });
        entries.Should().NotContain(entry => entry.Level == DiagnosticLevel.Error);

        using GravitasWorldContext replacedContext = CreateContext(extent: 16);
        SolidBody2D replaced = CreateCircle(replacedContext, Vector2d.Zero, immovable: false);
        entries = CaptureGridLogs(() =>
        {
            ReplacePrimaryGrid(replacedContext);
            replacedContext.Collisions2D.ClearPartitionedCollider(replaced.Collider, force: true).Should().BeTrue();
        });
        replaced.Collider.IsPartitioned.Should().BeFalse();
        entries.Should().NotContain(entry => entry.Level == DiagnosticLevel.Error);
    }

    [Fact]
    public void RefreshPartitionAwakeState_WithMissingGridVoxelOrPartition_ShouldSkipStaleCoordinatesWithoutErrors()
    {
        using GravitasWorldContext detachedContext = CreateContext(extent: 16);
        SolidBody2D detached = CreateCircle(detachedContext, Vector2d.Zero, immovable: false);
        WorldVoxelIndex coordinate = detached.Collider.PartitionCoordinates![0];
        detachedContext.World.TryGetVoxel(coordinate, out Voxel? voxel).Should().BeTrue();
        voxel!.TryRemovePartition<PhysicsPartition2D>().Should().BeTrue();
        detachedContext.Collisions2D.RefreshPartitionAwakeState(detached.Collider);

        using GravitasWorldContext removedContext = CreateContext(extent: 16);
        SolidBody2D removed = CreateCircle(removedContext, Vector2d.Zero, immovable: false);
        List<(DiagnosticLevel Level, string Message)> entries = CaptureGridLogs(() =>
        {
            removedContext.World.TryRemoveGrid(0).Should().BeTrue();
            removedContext.Collisions2D.RefreshPartitionAwakeState(removed.Collider);
        });
        entries.Should().NotContain(entry => entry.Level == DiagnosticLevel.Error);

        using GravitasWorldContext replacedContext = CreateContext(extent: 16);
        SolidBody2D replaced = CreateCircle(replacedContext, Vector2d.Zero, immovable: false);
        entries = CaptureGridLogs(() =>
        {
            ReplacePrimaryGrid(replacedContext);
            replacedContext.Collisions2D.RefreshPartitionAwakeState(replaced.Collider);
        });
        entries.Should().NotContain(entry => entry.Level == DiagnosticLevel.Error);
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
    public void RefreshPartitionAwakeState_ShouldLeaveStaticAndKinematicMembershipUnchanged()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        SolidBody2D staticBody = CreateCircle(context, Vector2d.Zero, immovable: true);
        SolidBody2D kinematicBody = CreateCircle(context, new Vector2d((Fixed64)3, Fixed64.Zero), immovable: false);
        kinematicBody.IsKinematic = true;
        kinematicBody.Collider.Simulate();

        PhysicsPartition2D staticPartition = GetFirstPartition(context, staticBody.Collider);
        PhysicsPartition2D kinematicPartition = GetFirstPartition(context, kinematicBody.Collider);

        context.Collisions2D.RefreshPartitionAwakeState(staticBody.Collider);
        context.Collisions2D.RefreshPartitionAwakeState(kinematicBody.Collider);

        AssertPartitionMembership(staticPartition, staticBody.Collider.Id, dynamic: false, kinematic: false, @static: true);
        AssertPartitionMembership(kinematicPartition, kinematicBody.Collider.Id, dynamic: false, kinematic: true, @static: false);
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
    public void DynamicObjectRemoval_ShouldIgnoreMissingIdsAndDeactivateOnlyAfterLastDynamic()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        PhysicsPartition2D partition = context.Collisions2D.RentPartition();
        partition.AddDynamicObject(1);
        partition.AddDynamicObject(2);
        int activationId = partition.ActivationId;

        partition.RemoveDynamicObject(99);
        partition.RemoveDynamicObject(1);

        partition.ActivationId.Should().Be(activationId);
        context.Collisions2D.ActivePartitionCount.Should().Be(1);
        partition.ContainedDynamicObjects!.Should().Contain(2);
        partition.AwakeDynamicObjectCount.Should().Be(1);

        partition.RemoveDynamicObject(2);

        partition.ActivationId.Should().Be(-1);
        context.Collisions2D.ActivePartitionCount.Should().Be(0);
        partition.ContainedDynamicObjects.Count.Should().Be(0);
        partition.AwakeDynamicObjectCount.Should().Be(0);
    }

    [Fact]
    public void Distribute_WithEmptyOrSleepingDynamicMembership_ShouldReturnWithoutOwnerAccess()
    {
        var dynamicIds = new SwiftList<int>();
        var staticIds = new SwiftList<int>();
        var partition = new PhysicsPartition2D();

        partition.Distribute(dynamicIds, staticIds);
        dynamicIds.Count.Should().Be(0);
        staticIds.Count.Should().Be(0);

        partition.ContainedDynamicObjects = new SwiftSparseSet();
        partition.Distribute(dynamicIds, staticIds);
        dynamicIds.Count.Should().Be(0);
        staticIds.Count.Should().Be(0);

        partition.ContainedDynamicObjects.Add(7);
        partition.Distribute(dynamicIds, staticIds);
        dynamicIds.Count.Should().Be(0);
        staticIds.Count.Should().Be(0);

        partition.ContainedAwakeDynamicObjects = new SwiftSparseSet();
        partition.Distribute(dynamicIds, staticIds);
        dynamicIds.Count.Should().Be(0);
        staticIds.Count.Should().Be(0);
    }

    [Fact]
    public void DuplicateMembershipAdds_ShouldNotReactivateOrDuplicatePartitionBuckets()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        PhysicsPartition2D partition = context.Collisions2D.RentPartition();

        partition.AddDynamicObject(1);
        partition.AddStaticObject(2);
        partition.AddKinematicObject(3);
        int activationId = partition.ActivationId;

        partition.AddDynamicObject(1);
        partition.AddStaticObject(2);
        partition.AddKinematicObject(3);

        partition.ActivationId.Should().Be(activationId);
        context.Collisions2D.ActivePartitionCount.Should().Be(1);
        partition.ContainedDynamicObjects!.Count.Should().Be(1);
        partition.ContainedStaticObjects!.Count.Should().Be(1);
        partition.ContainedKinematicObjects!.Count.Should().Be(1);
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
    public void Simulate_WithSleepingBodyRestingOnStaticAndTouchedByAwakeBody_ShouldKeepRetainedSupportPairCurrent()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        SolidBody2D sleeper = CreateCircle(context, Vector2d.Zero, immovable: false);
        SolidBody2D support = CreateCircle(context, new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero), immovable: true);
        int exited = 0;
        sleeper.Collider.OnContactExit += _ => exited++;

        Step(context);
        CollisionPair2D supportPair = GetPair(sleeper, support);
        supportPair.IsColliding.Should().BeTrue();
        sleeper.Sleep();

        SolidBody2D driver = CreateCircle(
            context,
            new Vector2d(-Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: false);
        driver.SleepEnabled = false;
        Step(context);

        supportPair.LastFrame.Should().Be(context.FrameCount);
        supportPair.Manifold.HasContact.Should().BeTrue();
        supportPair.IsColliding.Should().BeTrue();
        sleeper.IsSleeping.Should().BeFalse();
        exited.Should().Be(0);
    }

    [Fact]
    public void Simulate_WithAwakeBodyTouchingSleepingIsland_ShouldCullInvalidExisting2DPairs()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        SolidBody2D sleeper = CreateCircle(context, Vector2d.Zero, immovable: false);
        SolidBody2D validSupport = CreateCircle(context, new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero), immovable: true);
        SolidBody2D filteredSupport = CreateCircle(context, new Vector2d(Fixed64.Zero, Fixed64.FromFraction(3, 4)), immovable: true);
        SolidBody2D movedSupport = CreateCircle(context, new Vector2d(-Fixed64.FromFraction(3, 4), Fixed64.Zero), immovable: true);
        LSCircleCollider2D trigger = CreateBodylessCircle(context, new Vector2d(Fixed64.Zero, -Fixed64.FromFraction(3, 4)), isTrigger: true);
        int exits = 0;
        int triggerStay = 0;
        sleeper.Collider.OnContactExit += _ => exits++;
        sleeper.Collider.OnTriggerStay += _ => triggerStay++;

        Step(context);
        CollisionPair2D validPair = GetPair(sleeper, validSupport);
        CollisionPair2D filteredPair = GetPair(sleeper, filteredSupport);
        CollisionPair2D movedPair = GetPair(sleeper, movedSupport);
        sleeper.Collider.TryGetCollisionPair(trigger.Id, out CollisionPair2D? triggerPair).Should().BeTrue();
        triggerPair!.IsColliding.Should().BeTrue();
        sleeper.Sleep();

        filteredSupport.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(sleeper.Collider.Layer);
        movedSupport.SetPosition(new Vector2d((Fixed64)(-6), Fixed64.Zero));
        SolidBody2D driver = CreateCircle(context, new Vector2d(-Fixed64.FromFraction(3, 4), Fixed64.Zero), immovable: false);
        driver.SleepEnabled = false;

        Step(context);

        validPair.LastFrame.Should().Be(context.FrameCount);
        validPair.IsColliding.Should().BeTrue();
        filteredPair.IsColliding.Should().BeFalse();
        movedPair.IsColliding.Should().BeFalse();
        triggerPair.LastFrame.Should().Be(context.FrameCount);
        triggerPair.IsColliding.Should().BeTrue();
        triggerStay.Should().BeGreaterThan(0);
        sleeper.IsSleeping.Should().BeFalse();
        exits.Should().BeGreaterThanOrEqualTo(2);
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

    private static void ReplacePrimaryGrid(GravitasWorldContext context)
    {
        GridConfiguration spareConfiguration = new(
            new Vector3d((Fixed64)100, Fixed64.Zero, (Fixed64)100),
            new Vector3d((Fixed64)104, Fixed64.Zero, (Fixed64)104));
        context.World.TryAddGrid(spareConfiguration, out _).Should().BeTrue();
        context.World.TryRemoveGrid(0).Should().BeTrue();

        GridConfiguration replacementConfiguration = new(
            new Vector3d((Fixed64)200, Fixed64.Zero, (Fixed64)200),
            new Vector3d((Fixed64)201, Fixed64.Zero, (Fixed64)201));
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

    private static WorldVoxelIndex ResolveCanonicalSharedPartition(LSCollider2D first, LSCollider2D second)
    {
        SwiftList<WorldVoxelIndex> firstCoordinates = first.PartitionCoordinates!;
        SwiftList<WorldVoxelIndex> secondCoordinates = second.PartitionCoordinates!;
        for (int i = 0; i < firstCoordinates.Count; i++)
        {
            WorldVoxelIndex candidate = firstCoordinates[i];
            if (!ContainsCoordinate(secondCoordinates, candidate))
                continue;

            if (GravitasPhysics2DService.IsCanonicalSharedPartition(first, second, candidate))
                return candidate;
        }

        throw new InvalidOperationException("Expected a canonical shared 2D partition coordinate.");
    }

    private static bool ContainsCoordinate(SwiftList<WorldVoxelIndex> coordinates, WorldVoxelIndex candidate)
    {
        for (int i = 0; i < coordinates.Count; i++)
        {
            if (coordinates[i].Equals(candidate))
                return true;
        }

        return false;
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

    private static WorldVoxelIndex CreateWorldVoxel(
        int worldSpawnToken,
        int gridSpawnToken,
        int x,
        int y,
        int z) =>
        new(worldSpawnToken, 0, gridSpawnToken, new VoxelIndex(x, y, z));

    private static long MeasureAllocatedBytes(Action action)
        => AllocationTestHelper.MeasureSinglePass(action);

    private readonly record struct ReplayResult(
        Vector2d Position,
        Vector2d Velocity,
        int CandidateTotal,
        int RetainedPartitionCount);
}
