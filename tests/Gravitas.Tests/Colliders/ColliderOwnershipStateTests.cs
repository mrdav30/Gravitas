using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class ColliderOwnershipStateTests
{
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
    public void DeactivateOwnedPairSide_ShouldRemovePairAndHolderReferences()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> owner = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> holder = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        scenario.Context.Simulate();

        owner.Collider.CollisionPairCount.Should().Be(1);
        holder.Collider.CollisionPairHolderCount.Should().Be(1);

        owner.Collider.Deactivate();

        owner.Collider.CollisionPairCount.Should().Be(0);
        holder.Collider.CollisionPairHolderCount.Should().Be(0);
        scenario.Context.Physics.TryGetColliderById(owner.Collider.Id, out _).Should().BeFalse();
    }

    [Fact]
    public void DeactivateParent_ShouldClearChildrenBeforeColliderIdReuse()
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
    public void DeactivateHolderSide_ShouldRemoveOwningPairReference()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> owner = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> holder = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        scenario.Context.Simulate();

        holder.Collider.Deactivate();

        owner.Collider.CollisionPairCount.Should().Be(0);
        holder.Collider.CollisionPairHolderCount.Should().Be(0);
        scenario.Context.Physics.TryGetColliderById(holder.Collider.Id, out _).Should().BeFalse();
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
}
