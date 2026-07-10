using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using GridForge.Grids;
using GridForge.Spatial;
using SwiftCollections;
using System;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed class MixedBroadPhaseTests
{
    [Fact]
    public void Simulate_WithSparseMixedOverlap_ShouldEmitStableCandidateKey()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero, immovable: false);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero, immovable: true);

        Step(context);

        context.MixedCollisions.LastBroadPhaseCandidateCount.Should().Be(1);
        MixedColliderKey candidate = context.MixedCollisions.GetCandidate(0);
        candidate.Collider3DId.Should().Be(body3D.Collider.Id);
        candidate.Collider2DId.Should().Be(body2D.Collider.Id);
        context.MixedCollisions.ActivePartitionCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void Simulate_WithDenseMixedOverlap_ShouldEmitEachPairOnceInDeterministicOrder()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var colliders3D = new SwiftList<LSSphereCollider>();
        var bodies2D = new SwiftList<SolidBody2D>();

        for (int i = 0; i < 4; i++)
        {
            LSSphereCollider bodyless3D = CreateBodylessSphere3D(context, Vector3d.Zero);
            bodyless3D.Radius = (Fixed64)12;
            bodyless3D.Simulate();
            bodyless3D.IsTrigger = true;
            colliders3D.Add(bodyless3D);
            bodies2D.Add(CreateCircle2D(context, new Vector2d((Fixed64)(i * 3), Fixed64.Zero), immovable: false));
        }

        Step(context);

        context.MixedCollisions.LastBroadPhaseCandidateCount.Should().Be(16);
        ulong previousKey = 0;
        for (int i = 0; i < context.MixedCollisions.LastBroadPhaseCandidateCount; i++)
        {
            MixedColliderKey candidate = context.MixedCollisions.GetCandidate(i);
            if (i > 0)
                candidate.Key.Should().BeGreaterThan(previousKey);

            previousKey = candidate.Key;
        }
    }

    [Fact]
    public void SimulateAndCollectCandidates_WithStalePartitionIds_ShouldKeepOnlyValidPair()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero, immovable: false);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero, immovable: false);
        var candidates3D = new SwiftList<LSCollider>();
        var candidates2D = new SwiftList<LSCollider2D>();
        const int stale3DId = 31;
        const int stale2DId = 37;

        Step(context);
        PhysicsMixedPartition partition = GetFirstMixedPartition(context, body3D.Collider.MixedPartitionCoordinates!);
        partition.AddStatic3DObject(stale3DId);
        partition.AddStatic2DObject(stale2DId);
        context.MixedCollisions.ProcessPartitionCandidate(stale3DId, body2D.Collider.Id);
        context.MixedCollisions.ProcessPartitionCandidate(body3D.Collider.Id, stale2DId);

        Step(context);
        context.MixedCollisions.Collect3DCandidatesInMixedBounds(
            new Vector3d((Fixed64)(-2), (Fixed64)(-2), (Fixed64)(-2)),
            new Vector3d((Fixed64)2, (Fixed64)2, (Fixed64)2),
            PhysicsLayerMask.All,
            candidates3D);
        context.MixedCollisions.Collect2DCandidatesInMixedBounds(
            new Vector3d((Fixed64)(-2), (Fixed64)(-2), (Fixed64)(-2)),
            new Vector3d((Fixed64)2, (Fixed64)2, (Fixed64)2),
            PhysicsLayerMask.All,
            candidates2D);

        context.MixedCollisions.LastBroadPhaseCandidateCount.Should().Be(1);
        MixedColliderKey candidate = context.MixedCollisions.GetCandidate(0);
        candidate.Collider3DId.Should().Be(body3D.Collider.Id);
        candidate.Collider2DId.Should().Be(body2D.Collider.Id);
        candidates3D.Should().ContainSingle().Which.Should().BeSameAs(body3D.Collider);
        candidates2D.Should().ContainSingle().Which.Should().BeSameAs(body2D.Collider);

        Step(context);
        context.MixedCollisions.LastBroadPhaseCandidateCount.Should().Be(1);
        context.MixedCollisions.GetCandidate(0).Should().Be(candidate);
    }

    [Fact]
    public void Simulate_WithLargeSparseMixedWorld_ShouldCullFarCrossDimensionPairs()
    {
        using GravitasWorldContext context = CreateMixedContext(extent: 128);
        _ = CreateSphere3D(context, Vector3d.Zero, immovable: false);
        _ = CreateCircle2D(context, Vector2d.Zero, immovable: true);

        for (int i = 0; i < 32; i++)
        {
            Fixed64 offset = (Fixed64)(8 + (i * 2));
            _ = CreateSphere3D(context, new Vector3d(offset, Fixed64.Zero, (Fixed64)48), immovable: false);
            _ = CreateCircle2D(context, new Vector2d(offset, (Fixed64)(-48)), immovable: true);
        }

        Step(context);

        context.MixedCollisions.LastBroadPhaseCandidateCount.Should().Be(1);
    }

    [Fact]
    public void Simulate_WithSleepingMixedDynamics_ShouldSkipUntilOneParticipantWakes()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero, immovable: false);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero, immovable: false);
        body3D.Body.Sleep();
        body2D.Sleep();

        Step(context);

        context.MixedCollisions.LastBroadPhaseCandidateCount.Should().Be(0);

        body3D.Body.Wake();
        Step(context);

        context.MixedCollisions.LastBroadPhaseCandidateCount.Should().Be(1);
    }

    [Fact]
    public void Simulate_WithTriggerLayerAndSameAgentFilters_ShouldApplyMixedBroadPhaseRules()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var allowed3D = CreateSphere3D(context, Vector3d.Zero, immovable: false);
        LSCircleCollider2D trigger2D = CreateBodylessCircle2D(context, Vector2d.Zero);
        trigger2D.IsTrigger = true;

        var blocked3D = CreateSphere3D(context, new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero), immovable: false, layer: new PhysicsLayer(1));
        _ = CreateCircle2D(context, new Vector2d((Fixed64)4, Fixed64.Zero), immovable: true, layer: new PhysicsLayer(2));

        IMatterAgent sharedAgent = new TestMatterAgent(
            context,
            new FixedTransform(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One));
        var sameAgent3D = CreateSphere3D(context, sharedAgent, immovable: false);
        _ = CreateCircle2D(context, sharedAgent, immovable: true);

        Step(context);

        context.MixedCollisions.LastBroadPhaseCandidateCount.Should().Be(1);
        MixedColliderKey candidate = context.MixedCollisions.GetCandidate(0);
        candidate.Collider3DId.Should().Be(allowed3D.Collider.Id);
        candidate.Collider2DId.Should().Be(trigger2D.Id);
        candidate.Collider3DId.Should().NotBe(blocked3D.Collider.Id);
        candidate.Collider3DId.Should().NotBe(sameAgent3D.Collider.Id);
    }

    [Fact]
    public void Simulate_WithMixedParentChildHierarchy_ShouldSuppressCandidate()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> parent3D = CreateSphere3D(context, Vector3d.Zero, immovable: false);
        SolidBody2D child2D = CreateCircle2D(context, Vector2d.Zero, immovable: false);

        child2D.Collider.SetParent(parent3D.Collider);

        Step(context);

        context.MixedCollisions.LastBroadPhaseCandidateCount.Should().Be(0);
        parent3D.Collider.HierarchyChildCount.Should().Be(1);
        child2D.Collider.Parent3D.Should().BeSameAs(parent3D.Collider);
        child2D.Collider.Parent2D.Should().BeNull();
        child2D.Collider.TopParent3D.Should().BeSameAs(parent3D.Collider);
        child2D.Collider.TopParent2D.Should().BeNull();
        child2D.Collider.ParentKey.Should().Be(parent3D.Collider.HierarchyKey);
    }

    [Fact]
    public void Simulate_WithMixedSiblingHierarchy_ShouldSuppressCandidate()
    {
        using GravitasWorldContext context = CreateMixedContext();
        SolidBody2D parent2D = CreateCircle2D(context, new Vector2d((Fixed64)8, Fixed64.Zero), immovable: true);
        ScenarioBody<LSSphereCollider> child3D = CreateSphere3D(context, Vector3d.Zero, immovable: false);
        SolidBody2D child2D = CreateCircle2D(context, Vector2d.Zero, immovable: false);

        child3D.Collider.SetParent(parent2D.Collider);
        child2D.Collider.SetParent(parent2D.Collider);

        Step(context);

        context.MixedCollisions.LastBroadPhaseCandidateCount.Should().Be(0);
        parent2D.Collider.HierarchyChildCount.Should().Be(2);
        child3D.Collider.Parent2D.Should().BeSameAs(parent2D.Collider);
        child3D.Collider.Parent3D.Should().BeNull();
        child3D.Collider.TopParent2D.Should().BeSameAs(parent2D.Collider);
        child3D.Collider.TopParent3D.Should().BeNull();
        child3D.Collider.ParentKey.Should().Be(parent2D.Collider.HierarchyKey);
        child2D.Collider.Parent2D.Should().BeSameAs(parent2D.Collider);
        child2D.Collider.Parent3D.Should().BeNull();
        child2D.Collider.TopParent2D.Should().BeSameAs(parent2D.Collider);
        child2D.Collider.TopParent3D.Should().BeNull();
        child2D.Collider.ParentKey.Should().Be(parent2D.Collider.HierarchyKey);
    }

    [Fact]
    public void Simulate_WithRetainedMixedPartitions_ShouldRetireAndPoolAfterTtk()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Settings.RetainedPartitionTimeToKillFrames = 1;
        context.Settings.RetainedPartitionRetirementSweepBudget = 1024;
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero, immovable: false);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero, immovable: false);

        Step(context);
        int retainedBeforeDeactivate = context.MixedCollisions.RetainedPartitionCount;

        body3D.Collider.Deactivate();
        body2D.Collider.Deactivate();
        Step(context);

        context.MixedCollisions.ActivePartitionCount.Should().Be(0);
        context.MixedCollisions.RetainedPartitionCount.Should().BeLessThan(retainedBeforeDeactivate);
        context.MixedCollisions.InactivePartitionCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void Simulate_WithRetainedMixedPartitionsAndZeroSweepBudget_ShouldKeepRetainedPartitionsUntilBudgetRestored()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Settings.RetainedPartitionTimeToKillFrames = 1;
        context.Settings.RetainedPartitionRetirementSweepBudget = 0;
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero, immovable: false);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero, immovable: false);

        Step(context);
        int retainedBeforeDeactivate = context.MixedCollisions.RetainedPartitionCount;
        body3D.Collider.Deactivate();
        body2D.Collider.Deactivate();
        Step(context);

        context.MixedCollisions.ActivePartitionCount.Should().Be(0);
        context.MixedCollisions.RetainedPartitionCount.Should().Be(retainedBeforeDeactivate);
        context.MixedCollisions.InactivePartitionCount.Should().Be(0);

        context.Settings.RetainedPartitionRetirementSweepBudget = 1024;
        Step(context);

        context.MixedCollisions.RetainedPartitionCount.Should().BeLessThan(retainedBeforeDeactivate);
        context.MixedCollisions.InactivePartitionCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void Reset_WithRetainedMixedPartitionAlreadyDetached_ShouldReleaseWithoutVoxelDetach()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero, immovable: false);
        _ = CreateCircle2D(context, Vector2d.Zero, immovable: false);

        Step(context);
        WorldVoxelIndex coordinate = body3D.Collider.MixedPartitionCoordinates![0];
        context.World.TryGetVoxel(coordinate, out Voxel? voxel).Should().BeTrue();
        voxel!.TryGetPartition(out PhysicsMixedPartition? partition).Should().BeTrue();
        voxel.TryRemovePartition<PhysicsMixedPartition>().Should().BeTrue();

        context.Reset();

        context.MixedCollisions.RetainedPartitionCount.Should().Be(0);
        context.MixedCollisions.ActivePartitionCount.Should().Be(0);
        context.MixedCollisions.InactivePartitionCount.Should().Be(0);
        partition!.IsAllocated.Should().BeFalse();
        Action readOwner = () => _ = partition.Owner;
        readOwner.Should().Throw<InvalidOperationException>();
    }

    [Fact]
    public void RentPartition_WhenPoolEmptyAndRetainedEmptyPartitionExists_ShouldReuseRetiredPartition()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Settings.RetainedPartitionRetirementSweepBudget = 0;
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero, immovable: false);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero, immovable: false);

        Step(context);
        _ = GetFirstMixedPartition(context, body3D.Collider.MixedPartitionCoordinates!);
        body3D.Collider.Deactivate();
        body2D.Collider.Deactivate();
        Step(context);

        context.MixedCollisions.ActivePartitionCount.Should().Be(0);
        context.MixedCollisions.InactivePartitionCount.Should().Be(0);
        context.MixedCollisions.RetainedPartitionCount.Should().BeGreaterThan(0);

        int retainedBeforeRent = context.MixedCollisions.RetainedPartitionCount;
        PhysicsMixedPartition rented = context.MixedCollisions.RentPartition();

        rented.Owner.Should().BeSameAs(context.MixedCollisions);
        context.MixedCollisions.RetainedPartitionCount.Should().Be(retainedBeforeRent - 1);
        context.MixedCollisions.InactivePartitionCount.Should().Be(0);
        context.MixedCollisions.ReleasePartition(rented);
    }

    [Fact]
    public void RentPartition_AfterRelease_ShouldReuseInactivePoolEntry()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.MixedCollisions.InactivePartitionCount.Should().Be(0);

        PhysicsMixedPartition first = context.MixedCollisions.RentPartition();
        context.MixedCollisions.ReleasePartition(first);
        context.MixedCollisions.InactivePartitionCount.Should().Be(1);

        PhysicsMixedPartition second = context.MixedCollisions.RentPartition();

        second.Should().BeSameAs(first);
        second.Owner.Should().BeSameAs(context.MixedCollisions);
        context.MixedCollisions.InactivePartitionCount.Should().Be(0);
        context.MixedCollisions.ReleasePartition(second);
        context.MixedCollisions.InactivePartitionCount.Should().Be(1);
    }

    [Fact]
    public void ClearPartitionedMixedColliders_ShouldRequireForceWhenBoundsAreUnchanged()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero, immovable: false);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero, immovable: false);

        Step(context);

        context.MixedCollisions.ClearPartitioned3DCollider(body3D.Collider).Should().BeFalse();
        context.MixedCollisions.ClearPartitioned2DCollider(body2D.Collider).Should().BeFalse();

        context.MixedCollisions.ClearPartitioned3DCollider(body3D.Collider, force: true).Should().BeTrue();
        context.MixedCollisions.ClearPartitioned2DCollider(body2D.Collider, force: true).Should().BeTrue();

        body3D.Collider.IsMixedPartitioned.Should().BeFalse();
        body2D.Collider.IsMixedPartitioned.Should().BeFalse();
        body3D.Collider.MixedPartitionCoordinates!.Count.Should().Be(0);
        body2D.Collider.MixedPartitionCoordinates!.Count.Should().Be(0);
    }

    [Fact]
    public void ClearAndRefreshMixedPartitionState_WithDetachedAndStaleCoordinates_ShouldSkipMissingVoxels()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero, immovable: false);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero, immovable: false);

        Step(context);
        SwiftList<WorldVoxelIndex> coordinates3D = body3D.Collider.MixedPartitionCoordinates!;
        SwiftList<WorldVoxelIndex> coordinates2D = body2D.Collider.MixedPartitionCoordinates!;
        context.World.TryGetVoxel(coordinates3D[0], out Voxel? voxel).Should().BeTrue();
        voxel!.TryRemovePartition<PhysicsMixedPartition>().Should().BeTrue();
        coordinates3D.Add(default);
        coordinates2D.Add(default);

        context.MixedCollisions.Refresh3DPartitionAwakeState(body3D.Collider);
        context.MixedCollisions.Refresh2DPartitionAwakeState(body2D.Collider);
        context.MixedCollisions.ClearPartitioned3DCollider(body3D.Collider, force: true).Should().BeTrue();
        context.MixedCollisions.ClearPartitioned2DCollider(body2D.Collider, force: true).Should().BeTrue();

        body3D.Collider.IsMixedPartitioned.Should().BeFalse();
        body2D.Collider.IsMixedPartitioned.Should().BeFalse();
        coordinates3D.Count.Should().Be(0);
        coordinates2D.Count.Should().Be(0);
    }

    [Fact]
    public void Refresh3DColliderPartition_WithInactiveCollider_ShouldClearMixedMembership()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero, immovable: false);

        Step(context);
        body3D.Collider.IsMixedPartitioned.Should().BeTrue();
        WorldVoxelIndex originalCoordinate = body3D.Collider.MixedPartitionCoordinates![0];

        context.MixedCollisions.Refresh3DColliderPartition(body3D.Collider).Should().BeFalse();

        body3D.Collider.IsMixedPartitioned.Should().BeTrue();
        body3D.Collider.MixedPartitionCoordinates![0].Should().Be(originalCoordinate);

        body3D.Collider.SetStatus(false);

        context.MixedCollisions.Refresh3DColliderPartition(body3D.Collider).Should().BeFalse();

        body3D.Collider.IsMixedPartitioned.Should().BeFalse();
        body3D.Collider.MixedPartitionCoordinates!.Count.Should().Be(0);
        context.MixedCollisions.Refresh3DColliderPartition(body3D.Collider).Should().BeFalse();
    }

    [Fact]
    public void Refresh3DColliderPartition_WithColliderOutsideWorld_ShouldRemainUnpartitioned()
    {
        using GravitasWorldContext context = CreateMixedContext(extent: 4);
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d((Fixed64)16, Fixed64.Zero, Fixed64.Zero),
            immovable: false);

        bool partitioned = context.MixedCollisions.Refresh3DColliderPartition(body3D.Collider);

        partitioned.Should().BeFalse();
        body3D.Collider.IsMixedPartitioned.Should().BeFalse();
        body3D.Collider.MixedPartitionCoordinates.Should().NotBeNull();
        body3D.Collider.MixedPartitionCoordinates!.Count.Should().Be(0);
        context.MixedCollisions.ActivePartitionCount.Should().Be(0);
        context.MixedCollisions.RetainedPartitionCount.Should().Be(0);
    }

    [Fact]
    public void Refresh2DColliderPartition_WithInactiveCollider_ShouldClearMixedMembership()
    {
        using GravitasWorldContext context = CreateMixedContext();
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero, immovable: false);

        Step(context);
        body2D.Collider.IsMixedPartitioned.Should().BeTrue();
        WorldVoxelIndex originalCoordinate = body2D.Collider.MixedPartitionCoordinates![0];

        context.MixedCollisions.Refresh2DColliderPartition(body2D.Collider).Should().BeFalse();

        body2D.Collider.IsMixedPartitioned.Should().BeTrue();
        body2D.Collider.MixedPartitionCoordinates![0].Should().Be(originalCoordinate);

        body2D.Collider.IsActive = false;

        context.MixedCollisions.Refresh2DColliderPartition(body2D.Collider).Should().BeFalse();

        body2D.Collider.IsMixedPartitioned.Should().BeFalse();
        body2D.Collider.MixedPartitionCoordinates!.Count.Should().Be(0);
        context.MixedCollisions.Refresh2DColliderPartition(body2D.Collider).Should().BeFalse();
    }

    [Fact]
    public void RefreshMixedPartitionAwakeState_ShouldIgnoreStaticAndUpdateDynamicBuckets()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> dynamic3D = CreateSphere3D(context, Vector3d.Zero, immovable: false);
        SolidBody2D dynamic2D = CreateCircle2D(context, Vector2d.Zero, immovable: false);
        LSSphereCollider static3D = CreateBodylessSphere3D(context, new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));
        LSCircleCollider2D static2D = CreateBodylessCircle2D(context, new Vector2d((Fixed64)4, Fixed64.Zero));

        Step(context);
        dynamic3D.Body.Sleep();
        dynamic2D.Sleep();

        context.MixedCollisions.Refresh3DPartitionAwakeState(static3D);
        context.MixedCollisions.Refresh2DPartitionAwakeState(static2D);
        context.MixedCollisions.Refresh3DPartitionAwakeState(dynamic3D.Collider);
        context.MixedCollisions.Refresh2DPartitionAwakeState(dynamic2D.Collider);

        PhysicsMixedPartition dynamic3DPartition = GetFirstMixedPartition(context, dynamic3D.Collider.MixedPartitionCoordinates!);
        PhysicsMixedPartition dynamic2DPartition = GetFirstMixedPartition(context, dynamic2D.Collider.MixedPartitionCoordinates!);
        ContainsId(dynamic3DPartition.ContainedAwakeDynamic3DObjects, dynamic3D.Collider.Id).Should().BeFalse();
        ContainsId(dynamic2DPartition.ContainedAwakeDynamic2DObjects, dynamic2D.Collider.Id).Should().BeFalse();

        context.MixedCollisions.Refresh3DPartitionAwakeState(new LSSphereCollider());
        context.MixedCollisions.Refresh2DPartitionAwakeState(new LSCircleCollider2D(Fixed64.Half));
    }

    [Fact]
    public void CollectMixedCandidates_WithStaticStyleOnlyAndCachedRefresh_ShouldFilterDynamicsAndLayers()
    {
        using GravitasWorldContext context = CreateMixedContext(extent: 64);
        ScenarioBody<LSSphereCollider> dynamic3D = CreateSphere3D(context, Vector3d.Zero, immovable: false);
        ScenarioBody<LSSphereCollider> kinematic3D = CreateSphere3D(context, Vector3d.Right * (Fixed64)2, immovable: false);
        kinematic3D.Body.IsKinematic = true;
        ScenarioBody<LSSphereCollider> staticBody3D = CreateSphere3D(context, Vector3d.Right * (Fixed64)4, immovable: true);
        _ = CreateSphere3D(context, Vector3d.Right * (Fixed64)6, immovable: true, layer: new PhysicsLayer(1));
        LSSphereCollider inactive3D = CreateBodylessSphere3D(context, Vector3d.Right * (Fixed64)7);
        inactive3D.SetStatus(false);

        SolidBody2D dynamic2D = CreateCircle2D(context, Vector2d.Zero, immovable: false);
        SolidBody2D kinematic2D = CreateCircle2D(context, Vector2d.Right * (Fixed64)2, immovable: false, isKinematic: true);
        SolidBody2D staticBody2D = CreateCircle2D(context, Vector2d.Right * (Fixed64)4, immovable: true);
        _ = CreateCircle2D(context, Vector2d.Right * (Fixed64)6, immovable: true, layer: new PhysicsLayer(1));
        LSCircleCollider2D inactive2D = CreateBodylessCircle2D(context, Vector2d.Right * (Fixed64)7);
        inactive2D.IsActive = false;
        var candidates3D = new SwiftList<LSCollider>();
        var candidates2D = new SwiftList<LSCollider2D>();

        context.MixedCollisions.Collect3DCandidatesInMixedBounds(
            new Vector3d((Fixed64)(-2), (Fixed64)(-2), (Fixed64)(-2)),
            new Vector3d((Fixed64)8, (Fixed64)2, (Fixed64)2),
            PhysicsLayerMask.FromLayer(0),
            candidates3D,
            staticStyleOnly: true,
            cachePartitionRefresh: true);
        context.MixedCollisions.Collect3DCandidatesInMixedBounds(
            new Vector3d((Fixed64)(-2), (Fixed64)(-2), (Fixed64)(-2)),
            new Vector3d((Fixed64)8, (Fixed64)2, (Fixed64)2),
            PhysicsLayerMask.FromLayer(0),
            candidates3D,
            staticStyleOnly: true,
            cachePartitionRefresh: true);
        context.MixedCollisions.Collect2DCandidatesInMixedBounds(
            new Vector3d((Fixed64)(-2), (Fixed64)(-2), (Fixed64)(-2)),
            new Vector3d((Fixed64)8, (Fixed64)2, (Fixed64)2),
            PhysicsLayerMask.FromLayer(0),
            candidates2D,
            staticStyleOnly: true,
            cachePartitionRefresh: true);
        context.MixedCollisions.Collect2DCandidatesInMixedBounds(
            new Vector3d((Fixed64)(-2), (Fixed64)(-2), (Fixed64)(-2)),
            new Vector3d((Fixed64)8, (Fixed64)2, (Fixed64)2),
            PhysicsLayerMask.FromLayer(0),
            candidates2D,
            staticStyleOnly: true,
            cachePartitionRefresh: true);

        candidates3D.Should().Contain(collider => ReferenceEquals(collider, kinematic3D.Collider));
        candidates3D.Should().Contain(collider => ReferenceEquals(collider, staticBody3D.Collider));
        candidates3D.Should().NotContain(collider => ReferenceEquals(collider, dynamic3D.Collider));
        candidates3D.Should().NotContain(collider => ReferenceEquals(collider, inactive3D));
        candidates2D.Should().Contain(collider => ReferenceEquals(collider, kinematic2D.Collider));
        candidates2D.Should().Contain(collider => ReferenceEquals(collider, staticBody2D.Collider));
        candidates2D.Should().NotContain(collider => ReferenceEquals(collider, dynamic2D.Collider));
        candidates2D.Should().NotContain(collider => ReferenceEquals(collider, inactive2D));
        for (int i = 0; i < candidates3D.Count; i++)
            candidates3D[i].Layer.Should().Be(new PhysicsLayer(0));
        for (int i = 0; i < candidates2D.Count; i++)
            candidates2D[i].Layer.Should().Be(new PhysicsLayer(0));
    }

    [Fact]
    public void RefreshMovedMixed3DColliderAcrossPartitions_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = CreateMixedContext(extent: 128);
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero, immovable: false);
        context.MixedCollisions.Refresh3DColliderPartition(body3D.Collider);

        Fixed64 stepDistance = context.VoxelSize;
        int step = 0;
        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            () =>
            {
                step++;
                Vector3d position = new(stepDistance * (Fixed64)step, Fixed64.Zero, Fixed64.Zero);
                body3D.Body.ResetPosition(position);
                body3D.Collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
                context.MixedCollisions.Refresh3DColliderPartition(body3D.Collider);
            },
            warmupIterations: 4,
            stabilizationIterations: 2,
            measurementIterations: 4);

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void Reset_WithRetainedMixedPartitions_ShouldDetachOwnedVoxelPartitions()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero, immovable: false);
        _ = CreateCircle2D(context, Vector2d.Zero, immovable: false);

        Step(context);

        WorldVoxelIndex coordinate = body3D.Collider.MixedPartitionCoordinates![0];
        context.World.TryGetVoxel(coordinate, out Voxel? voxel).Should().BeTrue();
        voxel!.TryGetPartition(out PhysicsMixedPartition? partition).Should().BeTrue();
        context.MixedCollisions.RetainedPartitionCount.Should().BeGreaterThan(0);

        context.Reset();

        context.MixedCollisions.RetainedPartitionCount.Should().Be(0);
        context.MixedCollisions.ActivePartitionCount.Should().Be(0);
        context.MixedCollisions.InactivePartitionCount.Should().Be(0);
        voxel.TryGetPartition<PhysicsMixedPartition>(out _).Should().BeFalse();
        (partition!.ContainedDynamic3DObjects?.Count ?? 0).Should().Be(0);
        (partition.ContainedAwakeDynamic3DObjects?.Count ?? 0).Should().Be(0);
        (partition.ContainedKinematic3DObjects?.Count ?? 0).Should().Be(0);
        (partition.ContainedStatic3DObjects?.Count ?? 0).Should().Be(0);
        (partition.ContainedDynamic2DObjects?.Count ?? 0).Should().Be(0);
        (partition.ContainedAwakeDynamic2DObjects?.Count ?? 0).Should().Be(0);
        (partition.ContainedKinematic2DObjects?.Count ?? 0).Should().Be(0);
        (partition.ContainedStatic2DObjects?.Count ?? 0).Should().Be(0);
        partition.IsAllocated.Should().BeFalse();

        ScenarioBody<LSSphereCollider> replacement3D = CreateSphere3D(context, Vector3d.Zero, immovable: false);
        _ = CreateCircle2D(context, Vector2d.Zero, immovable: false);
        Step(context);

        WorldVoxelIndex replacementCoordinate = replacement3D.Collider.MixedPartitionCoordinates![0];
        context.World.TryGetVoxel(replacementCoordinate, out Voxel? replacementVoxel).Should().BeTrue();
        replacementVoxel!.TryGetPartition(out PhysicsMixedPartition? replacementPartition).Should().BeTrue();
        replacementPartition!.ContainedDynamic3DObjects!.Contains(replacement3D.Collider.Id).Should().BeTrue();
        context.MixedCollisions.RetainedPartitionCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void Simulate_When3DMixedBodyMobilityChanges_ShouldMovePartitionMembershipSets()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero, immovable: false);
        _ = CreateCircle2D(context, Vector2d.Zero, immovable: true);

        Step(context);
        PhysicsMixedPartition partition = GetFirstMixedPartition(context, body3D.Collider.MixedPartitionCoordinates!);
        ContainsId(partition.ContainedDynamic3DObjects, body3D.Collider.Id).Should().BeTrue();

        body3D.Body.IsKinematic = true;
        Step(context);
        partition = GetFirstMixedPartition(context, body3D.Collider.MixedPartitionCoordinates!);

        ContainsId(partition.ContainedDynamic3DObjects, body3D.Collider.Id).Should().BeFalse();
        ContainsId(partition.ContainedKinematic3DObjects, body3D.Collider.Id).Should().BeTrue();

        body3D.Body.IsKinematic = false;
        body3D.Body.FreezeAxes = BodyFreezeAxes3D.Position;
        Step(context);
        partition = GetFirstMixedPartition(context, body3D.Collider.MixedPartitionCoordinates!);

        ContainsId(partition.ContainedKinematic3DObjects, body3D.Collider.Id).Should().BeFalse();
        ContainsId(partition.ContainedStatic3DObjects, body3D.Collider.Id).Should().BeTrue();

        body3D.Body.FreezeAxes = BodyFreezeAxes3D.None;
        Step(context);
        partition = GetFirstMixedPartition(context, body3D.Collider.MixedPartitionCoordinates!);

        ContainsId(partition.ContainedStatic3DObjects, body3D.Collider.Id).Should().BeFalse();
        ContainsId(partition.ContainedDynamic3DObjects, body3D.Collider.Id).Should().BeTrue();
    }

    [Fact]
    public void Simulate_When2DMixedBodyMobilityChanges_ShouldMovePartitionMembershipSets()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateSphere3D(context, Vector3d.Zero, immovable: false);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero, immovable: false);

        Step(context);
        PhysicsMixedPartition partition = GetFirstMixedPartition(context, body2D.Collider.MixedPartitionCoordinates!);
        ContainsId(partition.ContainedDynamic2DObjects, body2D.Collider.Id).Should().BeTrue();

        body2D.IsKinematic = true;
        Step(context);
        partition = GetFirstMixedPartition(context, body2D.Collider.MixedPartitionCoordinates!);

        ContainsId(partition.ContainedDynamic2DObjects, body2D.Collider.Id).Should().BeFalse();
        ContainsId(partition.ContainedKinematic2DObjects, body2D.Collider.Id).Should().BeTrue();

        body2D.IsKinematic = false;
        body2D.FreezeAxes = BodyFreezeAxes2D.Position;
        Step(context);
        partition = GetFirstMixedPartition(context, body2D.Collider.MixedPartitionCoordinates!);

        ContainsId(partition.ContainedKinematic2DObjects, body2D.Collider.Id).Should().BeFalse();
        ContainsId(partition.ContainedStatic2DObjects, body2D.Collider.Id).Should().BeTrue();

        body2D.FreezeAxes = BodyFreezeAxes2D.None;
        Step(context);
        partition = GetFirstMixedPartition(context, body2D.Collider.MixedPartitionCoordinates!);

        ContainsId(partition.ContainedStatic2DObjects, body2D.Collider.Id).Should().BeFalse();
        ContainsId(partition.ContainedDynamic2DObjects, body2D.Collider.Id).Should().BeTrue();
    }

    [Fact]
    public void ResetRetainedMembership_ShouldClearAllDimensionalBucketsAndMarkPartitionEmpty()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Simulate();
        PhysicsMixedPartition partition = context.MixedCollisions.RentPartition();
        partition.AddDynamic3DObject(7);
        partition.AddKinematic3DObject(3);
        partition.AddStatic3DObject(11);
        partition.AddDynamic2DObject(13);
        partition.AddKinematic2DObject(17);
        partition.AddStatic2DObject(19);
        partition.SetDynamic3DObjectAwake(7, awake: false);
        partition.SetDynamic2DObjectAwake(13, awake: false);
        int activationId = partition.ActivationId;

        context.MixedCollisions.DeactivatePartition(activationId);
        partition.ResetRetainedMembership();

        partition.IsEmpty.Should().BeTrue();
        partition.EmptySinceFrame.Should().Be(context.FrameCount);
        partition.IsAllocated.Should().BeFalse();
        partition.AwakeDynamicObjectCount.Should().Be(0);
        partition.ContainedDynamic3DObjects!.Count.Should().Be(0);
        partition.ContainedAwakeDynamic3DObjects!.Count.Should().Be(0);
        partition.ContainedKinematic3DObjects!.Count.Should().Be(0);
        partition.ContainedStatic3DObjects!.Count.Should().Be(0);
        partition.ContainedDynamic2DObjects!.Count.Should().Be(0);
        partition.ContainedAwakeDynamic2DObjects!.Count.Should().Be(0);
        partition.ContainedKinematic2DObjects!.Count.Should().Be(0);
        partition.ContainedStatic2DObjects!.Count.Should().Be(0);
        context.MixedCollisions.ReleasePartition(partition);
    }

    [Fact]
    public void ResetRetainedMembership_WithFreshMixedPartition_ShouldBeIdempotent()
    {
        var partition = new PhysicsMixedPartition();

        partition.ResetRetainedMembership();
        partition.ResetRetainedMembership();

        partition.IsEmpty.Should().BeTrue();
        partition.EmptySinceFrame.Should().Be(0);
        partition.IsAllocated.Should().BeFalse();
        partition.AwakeDynamicObjectCount.Should().Be(0);
        ContainsId(partition.ContainedDynamic3DObjects, 7).Should().BeFalse();
        ContainsId(partition.ContainedDynamic2DObjects, 11).Should().BeFalse();
    }

    [Fact]
    public void MixedPartition_AddRemoveAndCopyHelpers_ShouldKeepDimensionalBucketsCoherent()
    {
        using GravitasWorldContext context = CreateMixedContext();
        PhysicsMixedPartition partition = context.MixedCollisions.RentPartition();
        var ids = new SwiftList<int>();

        partition.AddDynamic3DObject(11);
        partition.AddDynamic3DObject(11);
        partition.AddKinematic3DObject(7);
        partition.AddStatic3DObject(19);
        partition.AddDynamic2DObject(13);
        partition.AddDynamic2DObject(13);
        partition.AddKinematic2DObject(5);
        partition.AddStatic2DObject(17);
        partition.SetDynamic3DObjectAwake(999, awake: true);
        partition.SetDynamic2DObjectAwake(999, awake: true);

        partition.IsAllocated.Should().BeTrue();
        partition.AwakeDynamicObjectCount.Should().Be(2);

        partition.Copy3DColliderIds(ids);
        ids.Count.Should().Be(3);
        ids[0].Should().Be(7);
        ids[1].Should().Be(11);
        ids[2].Should().Be(19);
        partition.Copy2DColliderIds(ids);
        ids.Count.Should().Be(3);
        ids[0].Should().Be(5);
        ids[1].Should().Be(13);
        ids[2].Should().Be(17);
        partition.CopyStaticStyle3DColliderIds(ids);
        ids.Count.Should().Be(2);
        ids[0].Should().Be(7);
        ids[1].Should().Be(19);
        partition.CopyStaticStyle2DColliderIds(ids);
        ids.Count.Should().Be(2);
        ids[0].Should().Be(5);
        ids[1].Should().Be(17);

        partition.RemoveDynamic3DObject(999);
        partition.RemoveStatic3DObject(999);
        partition.RemoveKinematic3DObject(999);
        partition.RemoveDynamic2DObject(999);
        partition.RemoveStatic2DObject(999);
        partition.RemoveKinematic2DObject(999);

        partition.RemoveDynamic3DObject(11);
        partition.RemoveKinematic3DObject(7);
        partition.RemoveStatic3DObject(19);
        partition.RemoveDynamic2DObject(13);
        partition.RemoveKinematic2DObject(5);
        partition.RemoveStatic2DObject(17);

        partition.IsEmpty.Should().BeTrue();
        partition.AwakeDynamicObjectCount.Should().Be(0);
        partition.IsAllocated.Should().BeFalse();
        context.MixedCollisions.ReleasePartition(partition);
    }

    private static GravitasWorldContext CreateMixedContext(int extent = 32)
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.ApplySettings(new PhysicsSettings(
            4,
            new[,]
            {
                { true, true, true },
                    { true, true, false },
                    { true, false, true }
                }));
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.Environment.Gravity = Fixed64.Zero;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-extent), (Fixed64)(-4), (Fixed64)(-extent)),
                new Vector3d((Fixed64)extent, (Fixed64)4, (Fixed64)extent)),
            out _).Should().BeTrue();
        return context;
    }

    private static void Step(GravitasWorldContext context)
    {
        context.Simulate();
        context.LateSimulate();
    }

    private static ScenarioBody<LSSphereCollider> CreateSphere3D(
        GravitasWorldContext context,
        Vector3d position,
        bool immovable,
        PhysicsLayer? layer = null)
    {
        var transform = new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        return CreateSphere3D(context, agent, immovable, layer);
    }

    private static ScenarioBody<LSSphereCollider> CreateSphere3D(
        GravitasWorldContext context,
        IMatterAgent agent,
        bool immovable,
        PhysicsLayer? layer = null)
    {
        var collider = new LSSphereCollider();
        if (layer.HasValue)
            collider.Layer = layer.Value;

        var body = new SolidBody(agent, collider)
        {
            Mass = Fixed64.One,
            FreezeAxes = immovable ? BodyFreezeAxes3D.Position : BodyFreezeAxes3D.None
        };
        body.Initialize(agent.Transform.Position, agent.Transform.Rotation);
        return new ScenarioBody<LSSphereCollider>(body, collider);
    }

    private static LSSphereCollider CreateBodylessSphere3D(
        GravitasWorldContext context,
        Vector3d position,
        PhysicsLayer? layer = null)
    {
        var transform = new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One);
        var collider = new LSSphereCollider();
        if (layer.HasValue)
            collider.Layer = layer.Value;

        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
        return collider;
    }

    private static SolidBody2D CreateCircle2D(
        GravitasWorldContext context,
        Vector2d position,
        bool immovable,
        PhysicsLayer? layer = null,
        bool isKinematic = false)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        return CreateCircle2D(context, agent, immovable, layer, isKinematic);
    }

    private static SolidBody2D CreateCircle2D(
        GravitasWorldContext context,
        IMatterAgent agent,
        bool immovable,
        PhysicsLayer? layer = null,
        bool isKinematic = false)
    {
        var collider = new LSCircleCollider2D(Fixed64.Half);
        if (layer.HasValue)
            collider.Layer = layer.Value;

        var body = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One,
            FreezeAxes = immovable ? BodyFreezeAxes2D.Position : BodyFreezeAxes2D.None,
            IsKinematic = isKinematic
        };
        body.Initialize(agent.Transform.Position.ToVector2d());
        return body;
    }

    private static LSCircleCollider2D CreateBodylessCircle2D(
        GravitasWorldContext context,
        Vector2d position,
        PhysicsLayer? layer = null)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var collider = new LSCircleCollider2D(Fixed64.Half);
        if (layer.HasValue)
            collider.Layer = layer.Value;

        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
        return collider;
    }

    private static PhysicsMixedPartition GetFirstMixedPartition(
        GravitasWorldContext context,
        SwiftList<WorldVoxelIndex> coordinates)
    {
        context.World.TryGetVoxel(coordinates[0], out Voxel? voxel).Should().BeTrue();
        voxel!.TryGetPartition(out PhysicsMixedPartition? partition).Should().BeTrue();
        return partition!;
    }

    private static bool ContainsId(SwiftSparseSet? set, int id) => set?.Contains(id) == true;
}
