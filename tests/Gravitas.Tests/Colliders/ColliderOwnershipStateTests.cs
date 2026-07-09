using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Grids;
using GridForge.Spatial;
using System;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class ColliderOwnershipStateTests
{
    [Fact]
    public void UnboundColliderPositionAndRotationSetters_ShouldThrow()
    {
        var collider = new LSSphereCollider();

        Action setPosition = () => collider.Position = Vector3d.Right;
        Action setRotation = () => collider.Rotation = FixedQuaternion.Identity;

        setPosition.Should().Throw<InvalidOperationException>();
        setRotation.Should().Throw<InvalidOperationException>();
    }

    [Fact]
    public void UnboundColliderWorldAndTransform_ShouldExposeMissingBinding()
    {
        var collider = new LSSphereCollider();

        collider.World.Should().BeNull();
        Action readTransform = () => _ = collider.Transform;

        readTransform.Should().Throw<InvalidOperationException>();
    }

    [Fact]
    public void BodylessColliderWorldTransformAndRotation_ShouldUseAgentBinding()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var transform = new FixedTransform(
            new Vector3d(Fixed64.One, (Fixed64)2, (Fixed64)3),
            FixedQuaternion.Identity,
            Vector3d.One);
        var agent = new TestMatterAgent(scenario.Context, transform);
        var collider = new LSSphereCollider();

        collider.InitializeWithNoBody(agent);
        collider.World.Should().BeSameAs(scenario.Context.World);
        collider.Transform.Should().BeSameAs(transform);
        collider.Position = transform.Position;
        agent.Transform.Position.Should().Be(transform.Position);
        collider.Position = Vector3d.Forward;
        agent.Transform.Position.Should().Be(Vector3d.Forward);
        collider.Rotation = FixedQuaternion.Identity;
        collider.Rotation.Should().Be(FixedQuaternion.Identity);

        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.Half);
        collider.Rotation = rotation;

        agent.Transform.Rotation.Should().Be(collider.Rotation);
        agent.Transform.Rotation.Should().NotBe(FixedQuaternion.Identity);
        collider.Transform.Should().BeSameAs(transform);
    }

    [Fact]
    public void BindingAndHierarchyIdentity_ShouldTrackUnregisteredAndRegisteredState()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var collider3D = new LSSphereCollider();
        var collider2D = new LSCircleCollider2D(Fixed64.Half);

        collider3D.HasHostBinding.Should().BeFalse();
        collider2D.HasHostBinding.Should().BeFalse();
        collider3D.HierarchyKey.Should().Be(ColliderHierarchyKey.None);
        collider2D.HierarchyKey.Should().Be(ColliderHierarchyKey.None);
        collider2D.Position.Should().Be(Vector2d.Zero);
        collider2D.Rotation.Should().Be(Fixed64.Zero);

        scenario.InitializeStaticCollider(collider3D, Vector3d.Zero);
        collider2D.InitializeWithNoBody(new TestMatterAgent(scenario.Context));

        collider3D.HasHostBinding.Should().BeTrue();
        collider2D.HasHostBinding.Should().BeTrue();
        collider3D.HierarchyKey.Should().Be(ColliderHierarchyKey.Create3D(collider3D.Id));
        collider2D.HierarchyKey.Should().Be(ColliderHierarchyKey.Create2D(collider2D.Id));

        collider3D.Deactivate();
        collider2D.Deactivate();

        collider3D.HasHostBinding.Should().BeTrue();
        collider2D.HasHostBinding.Should().BeFalse();
        collider3D.HierarchyKey.Should().Be(ColliderHierarchyKey.None);
        collider2D.HierarchyKey.Should().Be(ColliderHierarchyKey.None);
    }

    [Fact]
    public void PreRegistration2DActiveAndTriggerSetters_ShouldRemainLocal()
    {
        var collider = new LSCircleCollider2D(Fixed64.Half);

        collider.IsActive = true;
        collider.IsActive.Should().BeTrue();
        collider.IsActive = false;
        collider.IsActive.Should().BeFalse();
        collider.IsActive = false;
        collider.IsActive.Should().BeFalse();
        collider.IsTrigger = false;
        collider.IsTrigger.Should().BeFalse();
        collider.IsTrigger = true;
        collider.IsTrigger.Should().BeTrue();
        collider.IsTrigger = true;
        collider.IsTrigger.Should().BeTrue();

        collider.Deactivate();

        collider.HasHostBinding.Should().BeFalse();
        collider.IsActive.Should().BeFalse();
    }

    [Fact]
    public void IgnoredCollisionLayers_ShouldIgnoreSameValueAndWakeOnChange()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body3D = scenario.CreateSphere(Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(scenario.Context, Vector2d.Zero);
        body3D.Body.Sleep();
        body2D.Sleep();

        body3D.Collider.IgnoredCollisionLayers = PhysicsLayerMask.None;
        body2D.Collider.IgnoredCollisionLayers = PhysicsLayerMask.None;

        body3D.Body.IsAwakeForCollision.Should().BeFalse();
        body2D.IsAwakeForCollision.Should().BeFalse();

        PhysicsLayerMask mask = PhysicsLayerMask.FromLayer(3);
        body3D.Collider.IgnoredCollisionLayers = mask;
        body2D.Collider.IgnoredCollisionLayers = mask;

        body3D.Collider.IgnoredCollisionLayers.Should().Be(mask);
        body2D.Collider.IgnoredCollisionLayers.Should().Be(mask);
        body3D.Body.IsAwakeForCollision.Should().BeTrue();
        body2D.IsAwakeForCollision.Should().BeTrue();
    }

    [Fact]
    public void BodyColliderWorldAndTransform_ShouldUseSimulatedBodyBinding()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(1, 2, 3));

        body.Collider.World.Should().BeSameAs(scenario.Context.World);
        body.Collider.Transform.Should().BeSameAs(body.Body.PositionTransform);
        body.Collider.Velocity.Should().Be(body.Body.LinearVelocity);
    }

    [Fact]
    public void CompoundPartTransform_ShouldUseOwnerBinding()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var collider = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero)),
            CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero)));
        var transform = new FixedTransform(Vector3d.Right, FixedQuaternion.Identity, Vector3d.One);
        var body = new SolidBody(new TestMatterAgent(scenario.Context, transform), collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(transform.Position, transform.Rotation);
        LSCollider part = collider.GetPartCollider(0);

        part.World.Should().BeSameAs(scenario.Context.World);
        part.Transform.Should().BeSameAs(collider.Transform);
        part.Velocity.Should().Be(body.LinearVelocity);
    }

    [Theory]
    [InlineData(0, 1, 1)]
    [InlineData(1, 0, 1)]
    [InlineData(1, 1, 0)]
    [InlineData(-1, 1, 1)]
    [InlineData(1, -1, 1)]
    [InlineData(1, 1, -1)]
    public void CuboidSize_WithNonPositiveComponent_ShouldThrow(int x, int y, int z)
    {
        var collider = new LSCuboidCollider();

        Action setSize = () => collider.Size = new Vector3d((Fixed64)x, (Fixed64)y, (Fixed64)z);

        setSize.Should().Throw<ArgumentException>().WithParameterName("value");
    }

    [Fact]
    public void ExplicitParentBinding_ShouldSuppressParentChildAndSiblingPairs()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> parent = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> firstChild = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(1, 0, 0));
        ScenarioBody<LSSphereCollider> secondChild = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(2, 0, 0));
        ScenarioBody<LSSphereCollider> unrelated = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(3, 0, 0));

        firstChild.Collider.SetParent(parent.Collider);
        secondChild.Collider.SetParent(parent.Collider);

        scenario.Context.Physics.RequireCollisionPair(parent.Collider, firstChild.Collider).Should().BeFalse();
        scenario.Context.Physics.RequireCollisionPair(firstChild.Collider, secondChild.Collider).Should().BeFalse();
        scenario.Context.Physics.RequireCollisionPair(firstChild.Collider, unrelated.Collider).Should().BeTrue();
        parent.Collider.HierarchyChildCount.Should().Be(2);
        firstChild.Collider.ParentId.Should().Be(parent.Collider.Id);
    }

    [Fact]
    public void ExplicitParentBinding_ShouldCacheTopParentForDescendants()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> topParent = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> middleParent = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(1, 0, 0));
        ScenarioBody<LSSphereCollider> child = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(2, 0, 0));

        middleParent.Collider.SetParent(topParent.Collider);
        child.Collider.SetParent(middleParent.Collider);

        middleParent.Collider.TopParent3D.Should().BeSameAs(topParent.Collider);
        child.Collider.TopParent3D.Should().BeSameAs(topParent.Collider);
        child.Collider.Parent3D.Should().BeSameAs(middleParent.Collider);
        child.Collider.ParentId.Should().Be(topParent.Collider.Id);
    }

    [Fact]
    public void HierarchyLookup_ShouldResolveRegistered2DAnd3DKeysAcrossDimensions()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> collider3D = scenario.CreateSphere(Vector3d.Zero);
        SolidBody2D collider2D = CreateCircle2D(scenario.Context, Vector2d.Zero);
        var node3D = (IColliderHierarchyNode)collider3D.Collider;
        var node2D = (IColliderHierarchyNode)collider2D.Collider;

        node3D.TryGetHierarchyColliderByKey(ColliderHierarchyKey.None, out _).Should().BeFalse();
        node2D.TryGetHierarchyColliderByKey(ColliderHierarchyKey.None, out _).Should().BeFalse();
        node3D.TryGetHierarchyColliderByKey(collider3D.Collider.HierarchyKey, out IColliderHierarchyNode? resolved3D).Should().BeTrue();
        node3D.TryGetHierarchyColliderByKey(collider2D.Collider.HierarchyKey, out IColliderHierarchyNode? resolved2DFrom3D).Should().BeTrue();
        node2D.TryGetHierarchyColliderByKey(collider2D.Collider.HierarchyKey, out IColliderHierarchyNode? resolved2D).Should().BeTrue();
        node2D.TryGetHierarchyColliderByKey(collider3D.Collider.HierarchyKey, out IColliderHierarchyNode? resolved3DFrom2D).Should().BeTrue();

        resolved3D.Should().BeSameAs(collider3D.Collider);
        resolved2DFrom3D.Should().BeSameAs(collider2D.Collider);
        resolved2D.Should().BeSameAs(collider2D.Collider);
        resolved3DFrom2D.Should().BeSameAs(collider3D.Collider);
    }

    [Fact]
    public void DeactivateOwnedPairSide_ShouldRemovePairAndHolderReferences()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> owner = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> holder = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        AdvancePhysicsStep(scenario);

        owner.Collider.CollisionPairCount.Should().Be(1);
        holder.Collider.CollisionPairHolderCount.Should().Be(1);
        int ownerId = owner.Collider.Id;

        owner.Collider.Deactivate();

        owner.Collider.CollisionPairCount.Should().Be(0);
        holder.Collider.CollisionPairHolderCount.Should().Be(0);
        scenario.Context.Physics.TryGetColliderById(owner.Collider.Id, out _).Should().BeFalse();
        holder.Collider.TryRemoveCollisionPairHolder(ownerId).Should().BeFalse();
    }

    [Fact]
    public void DeactivateParent_ShouldClearChildrenBeforeRegisteringReplacementCollider()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> parent = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> child = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(1, 0, 0));
        child.Collider.SetParent(parent.Collider);
        int parentId = parent.Collider.Id;

        parent.Collider.Deactivate();
        ScenarioBody<LSSphereCollider> replacement = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(2, 0, 0));

        replacement.Collider.Id.Should().Be(parentId);
        child.Collider.ParentId.Should().Be(-1);
        child.Collider.Parent3D.Should().BeNull();
        child.Collider.Parent2D.Should().BeNull();
        scenario.Context.Physics.RequireCollisionPair(child.Collider, replacement.Collider).Should().BeTrue();
    }

    [Fact]
    public void Deactivate3DParent_ShouldClear2DChildHierarchyState()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> parent = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        SolidBody2D child = CreateCircle2D(scenario.Context, Vector2d.Zero);
        child.Collider.SetParent(parent.Collider);

        parent.Collider.Deactivate();

        child.Collider.ParentId.Should().Be(-1);
        child.Collider.Parent2D.Should().BeNull();
        child.Collider.Parent3D.Should().BeNull();
    }

    [Fact]
    public void Deactivate2DParent_ShouldClear3DChildHierarchyState()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        SolidBody2D parent = CreateCircle2D(scenario.Context, Vector2d.Zero);
        ScenarioBody<LSSphereCollider> child = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(1, 0, 0));
        child.Collider.SetParent(parent.Collider);

        parent.Collider.Deactivate();

        child.Collider.ParentId.Should().Be(-1);
        child.Collider.Parent2D.Should().BeNull();
        child.Collider.Parent3D.Should().BeNull();
    }

    [Fact]
    public void ClearParent_ShouldRemoveParentChildReferencesAndRestoreCollisionEligibility()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> parent = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> child = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(1, 0, 0));
        child.Collider.SetParent(parent.Collider);

        child.Collider.ClearParent();

        parent.Collider.HierarchyChildCount.Should().Be(0);
        child.Collider.ParentId.Should().Be(-1);
        child.Collider.Parent3D.Should().BeNull();
        child.Collider.Parent2D.Should().BeNull();
        scenario.Context.Physics.RequireCollisionPair(parent.Collider, child.Collider).Should().BeTrue();
    }

    [Fact]
    public void ReparentChild_ShouldMoveTopParentChildReference()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> firstParent = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> secondParent = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(1, 0, 0));
        ScenarioBody<LSSphereCollider> child = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(2, 0, 0));

        child.Collider.SetParent(firstParent.Collider);
        child.Collider.SetParent(secondParent.Collider);

        firstParent.Collider.HierarchyChildCount.Should().Be(0);
        secondParent.Collider.HierarchyChildCount.Should().Be(1);
        child.Collider.Parent3D.Should().BeSameAs(secondParent.Collider);
        child.Collider.TopParent3D.Should().BeSameAs(secondParent.Collider);
        child.Collider.ParentId.Should().Be(secondParent.Collider.Id);
    }

    [Fact]
    public void SetParent_ShouldRejectSelfCrossContextAndCycles()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        using PhysicsScenarioBuilder foreignScenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> parent = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> child = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(1, 0, 0));
        ScenarioBody<LSSphereCollider> foreign = foreignScenario.CreateSphere(PhysicsScenarioBuilder.Vector(2, 0, 0));

        Action selfParent = () => parent.Collider.SetParent(parent.Collider);
        Action crossContextParent = () => child.Collider.SetParent(foreign.Collider);
        child.Collider.SetParent(parent.Collider);
        Action cyclicParent = () => parent.Collider.SetParent(child.Collider);

        selfParent.Should().Throw<ArgumentException>().WithParameterName("parent");
        crossContextParent.Should().Throw<ArgumentException>().WithParameterName("parent");
        cyclicParent.Should().Throw<ArgumentException>().WithParameterName("parent");
    }

    [Fact]
    public void ClearParent_ShouldRestoreConfiguredParentFlagWhenLastChildIsRemoved()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> parent = CreateSphere(scenario, PhysicsScenarioBuilder.Vector(0, 0, 0), isParent: false);
        ScenarioBody<LSSphereCollider> child = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(1, 0, 0));
        child.Collider.SetParent(parent.Collider);
        parent.Collider.IsParent.Should().BeTrue();

        child.Collider.ClearParent();

        parent.Collider.IsParent.Should().BeFalse();
        parent.Collider.HierarchyChildCount.Should().Be(0);
        child.Collider.ParentId.Should().Be(-1);
    }

    [Fact]
    public void ClearParent_WithRemainingChildren_ShouldKeepParentRoleUntilLastChildIsRemoved()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> parent = CreateSphere(scenario, PhysicsScenarioBuilder.Vector(0, 0, 0), isParent: false);
        ScenarioBody<LSSphereCollider> firstChild = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(1, 0, 0));
        ScenarioBody<LSSphereCollider> secondChild = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(2, 0, 0));
        firstChild.Collider.SetParent(parent.Collider);
        secondChild.Collider.SetParent(parent.Collider);

        firstChild.Collider.ClearParent();

        parent.Collider.IsParent.Should().BeTrue();
        parent.Collider.HierarchyChildCount.Should().Be(1);
        secondChild.Collider.ParentId.Should().Be(parent.Collider.Id);

        secondChild.Collider.ClearParent();

        parent.Collider.IsParent.Should().BeFalse();
        parent.Collider.HierarchyChildCount.Should().Be(0);
    }

    [Fact]
    public void ReparentChildWithinSameTopParent_ShouldNotDuplicateTopParentChildReference()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> topParent = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> firstMiddle = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(1, 0, 0));
        ScenarioBody<LSSphereCollider> secondMiddle = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(2, 0, 0));
        ScenarioBody<LSSphereCollider> child = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(3, 0, 0));
        firstMiddle.Collider.SetParent(topParent.Collider);
        secondMiddle.Collider.SetParent(topParent.Collider);
        child.Collider.SetParent(firstMiddle.Collider);

        child.Collider.SetParent(secondMiddle.Collider);

        topParent.Collider.HierarchyChildCount.Should().Be(3);
        child.Collider.Parent3D.Should().BeSameAs(secondMiddle.Collider);
        child.Collider.TopParent3D.Should().BeSameAs(topParent.Collider);
        child.Collider.ParentId.Should().Be(topParent.Collider.Id);
    }

    [Fact]
    public void DeactivateHolderSide_ShouldRemoveOwningPairReference()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> owner = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> holder = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        AdvancePhysicsStep(scenario);
        int holderId = holder.Collider.Id;

        holder.Collider.Deactivate();

        owner.Collider.CollisionPairCount.Should().Be(0);
        holder.Collider.CollisionPairHolderCount.Should().Be(0);
        scenario.Context.Physics.TryGetColliderById(holder.Collider.Id, out _).Should().BeFalse();
        owner.Collider.TryRemoveCollisionPair(holderId).Should().BeFalse();
    }

    [Fact]
    public void StaticColliderPartitionRefresh_ShouldAdvanceRuntimeAndBroadPhaseVersionsOnce()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider collider = scenario.CreateStaticSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        uint initialRuntimeVersion = collider.RuntimeShapeVersion;
        uint initialBroadPhaseVersion = collider.BroadPhaseVersion;

        collider.Position = new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero);
        collider.Rotation = PhysicsScenarioBuilder.Yaw(45);
        collider.Simulate();

        collider.RuntimeShapeVersion.Should().Be(initialRuntimeVersion + 1);
        collider.BroadPhaseVersion.Should().Be(initialBroadPhaseVersion + 1);
        collider.PartitionChanged.Should().BeTrue();
        collider.Bounds.Center.Should().Be(new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero));

        uint rebuiltRuntimeVersion = collider.RuntimeShapeVersion;
        uint rebuiltBroadPhaseVersion = collider.BroadPhaseVersion;
        collider.Simulate();

        collider.RuntimeShapeVersion.Should().Be(rebuiltRuntimeVersion);
        collider.BroadPhaseVersion.Should().Be(rebuiltBroadPhaseVersion);
        collider.PartitionChanged.Should().BeFalse();
    }

    [Fact]
    public void Initialize_ShouldResetQueryVersions()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var collider = new LSSphereCollider
        {
            RaycastVersion = 17,
            CircleQueryVersion = 23
        };

        scenario.InitializeStaticCollider(collider, PhysicsScenarioBuilder.Vector(0, 0, 0));

        collider.RaycastVersion.Should().Be(0);
        collider.CircleQueryVersion.Should().Be(0);
    }

    [Fact]
    public void IsActiveSetter_WithMixed2DStaticCollider_ShouldRefreshPrimaryAndMixedPartitions()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        LSCircleCollider2D collider = CreateStaticCircle2D(scenario.Context, Vector2d.Zero);
        scenario.Context.MixedCollisions.Refresh2DColliderPartition(collider);

        collider.IsPartitioned.Should().BeTrue();
        collider.IsMixedPartitioned.Should().BeTrue();
        WorldVoxelIndex mixedCoordinate = collider.MixedPartitionCoordinates![0];
        PhysicsMixedPartition mixedPartition = GetMixedPartition(scenario.Context, mixedCoordinate);
        PartitionContains2DCollider(mixedPartition, collider.Id).Should().BeTrue();

        collider.IsActive = false;

        collider.IsPartitioned.Should().BeFalse();
        collider.IsMixedPartitioned.Should().BeFalse();
        (collider.PartitionCoordinates?.Count ?? 0).Should().Be(0);
        (collider.MixedPartitionCoordinates?.Count ?? 0).Should().Be(0);
        PartitionContains2DCollider(mixedPartition, collider.Id).Should().BeFalse();

        collider.IsActive = true;

        collider.IsPartitioned.Should().BeTrue();
        collider.IsMixedPartitioned.Should().BeTrue();
        PhysicsMixedPartition refreshedMixedPartition =
            GetMixedPartition(scenario.Context, collider.MixedPartitionCoordinates![0]);
        PartitionContains2DCollider(refreshedMixedPartition, collider.Id).Should().BeTrue();
    }

    [Fact]
    public void IsActiveSetter_WithPure2DStaticCollider_ShouldRefreshOnlyPrimaryPartition()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        LSCircleCollider2D collider = CreateStaticCircle2D(scenario.Context, Vector2d.Zero);

        collider.IsPartitioned.Should().BeTrue();
        collider.IsMixedPartitioned.Should().BeFalse();

        collider.IsActive = false;

        collider.IsPartitioned.Should().BeFalse();
        collider.IsMixedPartitioned.Should().BeFalse();

        collider.IsActive = true;

        collider.IsPartitioned.Should().BeTrue();
        collider.IsMixedPartitioned.Should().BeFalse();
    }

    private static void AdvancePhysicsStep(PhysicsScenarioBuilder scenario)
    {
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();
    }

    private static PhysicsMixedPartition GetMixedPartition(
        GravitasWorldContext context,
        WorldVoxelIndex coordinate)
    {
        context.World.TryGetVoxel(coordinate, out Voxel? voxel).Should().BeTrue();
        voxel!.TryGetPartition(out PhysicsMixedPartition? partition).Should().BeTrue();
        return partition!;
    }

    private static bool PartitionContains2DCollider(PhysicsMixedPartition partition, int colliderId) =>
        partition.ContainedDynamic2DObjects?.Contains(colliderId) == true
        || partition.ContainedKinematic2DObjects?.Contains(colliderId) == true
        || partition.ContainedStatic2DObjects?.Contains(colliderId) == true;

    private static ScenarioBody<LSSphereCollider> CreateSphere(
        PhysicsScenarioBuilder scenario,
        Vector3d position,
        bool isParent)
    {
        var collider = new LSSphereCollider();
        var transform = new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(scenario.Context, transform, isParent);
        var body = new SolidBody(agent, collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(position, FixedQuaternion.Identity);
        return new ScenarioBody<LSSphereCollider>(body, collider);
    }

    private static SolidBody2D CreateCircle2D(GravitasWorldContext context, Vector2d position)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One
        };
        body.Initialize(position);
        return body;
    }

    private static LSCircleCollider2D CreateStaticCircle2D(GravitasWorldContext context, Vector2d position)
    {
        var collider = new LSCircleCollider2D(Fixed64.Half);
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
        return collider;
    }
}
