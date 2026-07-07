using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using SwiftCollections;
using System;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class Collider2DStateParityTests
{
    [Fact]
    public void Simulate_WithUnchangedCollider_ShouldNotAdvanceRuntimeOrBroadPhaseVersions()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        uint runtimeVersion = body.Collider.RuntimeShapeVersion;
        uint broadPhaseVersion = body.Collider.BroadPhaseVersion;

        body.Collider.Simulate();

        body.Collider.RuntimeShapeVersion.Should().Be(runtimeVersion);
        body.Collider.BroadPhaseVersion.Should().Be(broadPhaseVersion);
    }

    [Fact]
    public void ColliderIsStatic_ShouldBeTrueForBodylessAndPositionFrozenColliders()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var bodyless = new LSCircleCollider2D(Fixed64.Half);
        bodyless.InitializeWithNoBody(new TestMatterAgent(context));
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        SolidBody2D nonDynamicBody = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)4, Fixed64.Zero),
            immovable: false,
            isDynamic: false);

        bodyless.IsStatic.Should().BeTrue();
        body.Collider.IsStatic.Should().BeFalse();
        nonDynamicBody.Collider.IsStatic.Should().BeTrue();

        body.FreezeAxes = BodyFreezeAxes2D.Position;
        body.Collider.IsStatic.Should().BeTrue();

        body.FreezeAxes = BodyFreezeAxes2D.None;
        body.Collider.IsStatic.Should().BeFalse();
        body.IsKinematic = true;
        body.Collider.IsStatic.Should().BeFalse();
    }

    [Fact]
    public void ColliderRegistration_ShouldUseReusableIdsAndNegativeInactiveSentinel()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        LSCollider2D first = CreateStaticCollider(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero);
        LSCollider2D second = CreateStaticCollider(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)2, Fixed64.Zero));

        first.Id.Should().Be(0);
        second.Id.Should().Be(1);

        second.Deactivate();
        LSCollider2D replacement = CreateStaticCollider(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)4, Fixed64.Zero));

        second.Id.Should().Be(-1);
        replacement.Id.Should().Be(1);
        context.Physics2D.TryGetColliderById(replacement.Id, out LSCollider2D? resolved).Should().BeTrue();
        resolved.Should().BeSameAs(replacement);
    }

    [Fact]
    public void SetPosition_WithChangedCollider_ShouldAdvanceRuntimeAndBroadPhaseVersionsOnce()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(extent: 64);
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        uint runtimeVersion = body.Collider.RuntimeShapeVersion;
        uint broadPhaseVersion = body.Collider.BroadPhaseVersion;

        body.SetPosition(new Vector2d((Fixed64)8, Fixed64.Zero));

        body.Collider.RuntimeShapeVersion.Should().Be(runtimeVersion + 1);
        body.Collider.BroadPhaseVersion.Should().BeGreaterThan(broadPhaseVersion);

        uint rebuiltRuntimeVersion = body.Collider.RuntimeShapeVersion;
        uint rebuiltBroadPhaseVersion = body.Collider.BroadPhaseVersion;
        body.Collider.Simulate();

        body.Collider.RuntimeShapeVersion.Should().Be(rebuiltRuntimeVersion);
        body.Collider.BroadPhaseVersion.Should().Be(rebuiltBroadPhaseVersion);
    }

    [Fact]
    public void Initialize_ShouldReset2DQueryVersions()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var collider = new LSCircleCollider2D(Fixed64.Half)
        {
            RaycastVersion = 17,
            CircleQueryVersion = 23
        };

        _ = CreateStaticCollider(context, collider, Vector2d.Zero);

        collider.RaycastVersion.Should().Be(0);
        collider.CircleQueryVersion.Should().Be(0);
    }

    [Fact]
    public void ShapeMutation_ShouldDirtyRuntimeShapeAndRefreshBoundsOnNextSimulate()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        var circle = (LSCircleCollider2D)body.Collider;
        uint runtimeVersion = circle.RuntimeShapeVersion;
        uint broadPhaseVersion = circle.BroadPhaseVersion;

        circle.Radius = Fixed64.One;
        circle.Simulate();

        circle.RuntimeShapeVersion.Should().Be(runtimeVersion + 1);
        circle.BroadPhaseVersion.Should().BeGreaterThan(broadPhaseVersion);
        circle.MaxX.Should().Be(Fixed64.One);
        circle.MinX.Should().Be(-Fixed64.One);
    }

    [Fact]
    public void CapsuleShapeMutation_ShouldValidateDimensionsAndDirtyShapeOnlyWhenValuesChange()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D body = CreateBody(context, new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3), Vector2d.Zero, immovable: false);
        var capsule = (LSCapsuleCollider2D)body.Collider;
        uint runtimeVersion = capsule.RuntimeShapeVersion;
        uint broadPhaseVersion = capsule.BroadPhaseVersion;

        capsule.Radius = Fixed64.Half;
        capsule.Height = (Fixed64)3;
        capsule.Simulate();

        capsule.RuntimeShapeVersion.Should().Be(runtimeVersion);
        capsule.BroadPhaseVersion.Should().Be(broadPhaseVersion);

        Action zeroRadius = () => capsule.Radius = Fixed64.Zero;
        Action tooLargeRadius = () => capsule.Radius = (Fixed64)2;
        Action tooShortHeight = () => capsule.Height = Fixed64.Half;
        zeroRadius.Should().Throw<ArgumentException>().WithParameterName("radius");
        tooLargeRadius.Should().Throw<ArgumentException>().WithParameterName("height");
        tooShortHeight.Should().Throw<ArgumentException>().WithParameterName("height");

        capsule.Radius = Fixed64.FromFraction(3, 4);
        capsule.Simulate();
        capsule.RuntimeShapeVersion.Should().Be(runtimeVersion + 1);
        capsule.BroadPhaseVersion.Should().BeGreaterThan(broadPhaseVersion);

        uint radiusVersion = capsule.RuntimeShapeVersion;
        capsule.Height = (Fixed64)4;
        capsule.Simulate();

        capsule.RuntimeShapeVersion.Should().Be(radiusVersion + 1);
        capsule.Height.Should().Be((Fixed64)4);
        capsule.Radius.Should().Be(Fixed64.FromFraction(3, 4));
    }

    [Fact]
    public void ExplicitParentBinding_ShouldSuppressParentChildAndSiblingPairs()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D parent = CreateBody(context, new LSCircleCollider2D(Fixed64.One), Vector2d.Zero, immovable: false);
        SolidBody2D firstChild = CreateBody(context, new LSCircleCollider2D(Fixed64.One), new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: false);
        SolidBody2D secondChild = CreateBody(context, new LSCircleCollider2D(Fixed64.One), new Vector2d(-Fixed64.Half, Fixed64.Zero), immovable: false);
        SolidBody2D unrelated = CreateBody(context, new LSCircleCollider2D(Fixed64.One), new Vector2d((Fixed64)2, Fixed64.Zero), immovable: false);

        firstChild.Collider.SetParent(parent.Collider);
        secondChild.Collider.SetParent(parent.Collider);

        context.Physics2D.RequireCollisionPair(parent.Collider, firstChild.Collider).Should().BeFalse();
        context.Physics2D.RequireCollisionPair(firstChild.Collider, secondChild.Collider).Should().BeFalse();
        context.Physics2D.RequireCollisionPair(firstChild.Collider, unrelated.Collider).Should().BeTrue();
        parent.Collider.HierarchyChildCount.Should().Be(2);
        firstChild.Collider.ParentId.Should().Be(parent.Collider.Id);
    }

    [Fact]
    public void DeactivateOwnedPairSide_ShouldRemovePairAndHolderReferences()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D owner = CreateBody(context, new LSCircleCollider2D(Fixed64.One), Vector2d.Zero, immovable: false);
        SolidBody2D holder = CreateBody(context, new LSCircleCollider2D(Fixed64.One), Vector2d.Zero, immovable: false);
        Step(context);

        owner.Collider.CollisionPairCount.Should().Be(1);
        holder.Collider.CollisionPairHolderCount.Should().Be(1);
        int ownerId = owner.Collider.Id;

        owner.Collider.Deactivate();

        owner.Collider.CollisionPairCount.Should().Be(0);
        holder.Collider.CollisionPairHolderCount.Should().Be(0);
        context.Physics2D.TryGetColliderById(owner.Collider.Id, out _).Should().BeFalse();
        holder.Collider.TryRemoveCollisionPairHolder(ownerId).Should().BeFalse();
    }

    [Fact]
    public void DeactivateHolderSide_ShouldRemoveOwningPairReference()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D owner = CreateBody(context, new LSCircleCollider2D(Fixed64.One), Vector2d.Zero, immovable: false);
        SolidBody2D holder = CreateBody(context, new LSCircleCollider2D(Fixed64.One), Vector2d.Zero, immovable: false);
        Step(context);
        int holderId = holder.Collider.Id;

        holder.Collider.Deactivate();

        owner.Collider.CollisionPairCount.Should().Be(0);
        holder.Collider.CollisionPairHolderCount.Should().Be(0);
        context.Physics2D.TryGetColliderById(holder.Collider.Id, out _).Should().BeFalse();
        owner.Collider.TryRemoveCollisionPair(holderId, out _).Should().BeFalse();
    }

    [Fact]
    public void DeactivatePairOwner_AfterWarmup_ShouldNotAllocate()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(extent: 128);
        var owners = new SwiftList<SolidBody2D>();
        for (int i = 0; i < 32; i++)
        {
            Vector2d position = new((Fixed64)(i * 2), Fixed64.Zero);
            SolidBody2D owner = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), position, immovable: false);
            _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), position + new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: true);
            owners.Add(owner);
        }

        Step(context);

        long allocatedBytes = MeasureAllocatedBytes(() => owners[0].Collider.Deactivate());

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void DeactivateAllPairOwners_AfterWarmup_ShouldNotAllocate()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(extent: 128);
        var owners = new SwiftList<SolidBody2D>();
        for (int i = 0; i < 32; i++)
        {
            Vector2d position = new((Fixed64)(i * 2), Fixed64.Zero);
            SolidBody2D owner = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), position, immovable: false);
            _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), position + new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: true);
            owners.Add(owner);
        }

        Step(context);

        long allocatedBytes = MeasureAllocatedBytes(() =>
        {
            for (int i = 0; i < owners.Count; i++)
                owners[i].Collider.Deactivate();
        });

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void DeactivateParent_ShouldClearChildHierarchyState()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D parent = CreateBody(context, new LSCircleCollider2D(Fixed64.One), Vector2d.Zero, immovable: false);
        SolidBody2D child = CreateBody(context, new LSCircleCollider2D(Fixed64.One), new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: false);
        child.Collider.SetParent(parent.Collider);

        parent.Collider.Deactivate();

        child.Collider.ParentId.Should().Be(-1);
        child.Collider.Parent2D.Should().BeNull();
        child.Collider.Parent3D.Should().BeNull();
    }

    [Fact]
    public void ClearParent_ShouldRemoveParentChildReferencesAndRestoreCollisionEligibility()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D parent = CreateBody(context, new LSCircleCollider2D(Fixed64.One), Vector2d.Zero, immovable: false);
        SolidBody2D child = CreateBody(context, new LSCircleCollider2D(Fixed64.One), new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: false);
        child.Collider.SetParent(parent.Collider);

        child.Collider.ClearParent();

        parent.Collider.HierarchyChildCount.Should().Be(0);
        child.Collider.ParentId.Should().Be(-1);
        child.Collider.Parent2D.Should().BeNull();
        child.Collider.Parent3D.Should().BeNull();
        context.Physics2D.RequireCollisionPair(parent.Collider, child.Collider).Should().BeTrue();
    }

    [Fact]
    public void ClearParent_ShouldRestoreConfiguredParentFlagWhenLastChildIsRemoved()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D parent = CreateBody(context, new LSCircleCollider2D(Fixed64.One), Vector2d.Zero, immovable: false, isParent: false);
        SolidBody2D child = CreateBody(context, new LSCircleCollider2D(Fixed64.One), new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: false);
        child.Collider.SetParent(parent.Collider);
        parent.Collider.IsParent.Should().BeTrue();

        child.Collider.ClearParent();

        parent.Collider.IsParent.Should().BeFalse();
        parent.Collider.HierarchyChildCount.Should().Be(0);
        child.Collider.ParentId.Should().Be(-1);
    }

    [Theory]
    [InlineData(ColliderType2D.Circle, ColliderType2D.Circle, CollisionType2D.Circle_Circle)]
    [InlineData(ColliderType2D.Circle, ColliderType2D.AABox, CollisionType2D.Circle_Convex)]
    [InlineData(ColliderType2D.AABox, ColliderType2D.Circle, CollisionType2D.Convex_Circle)]
    [InlineData(ColliderType2D.AABox, ColliderType2D.ConvexPolygon, CollisionType2D.Convex_Convex)]
    public void ColliderSettings2D_ShouldResolveCollisionType(ColliderType2D first, ColliderType2D second, CollisionType2D expected)
    {
        ColliderSettings2D.GetCollisionType(first, second).Should().Be(expected);
    }

    private static SolidBody2D CreateBody(
        GravitasWorldContext context,
        LSCollider2D collider,
        Vector2d position,
        bool immovable,
        bool isDynamic = true,
        bool isParent = true)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var agent = new TestMatterAgent(context, transform, isParent);
        var body = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One,
            FreezeAxes = immovable ? BodyFreezeAxes2D.Position : BodyFreezeAxes2D.None
        };
        body.Initialize(position, isDynamic: isDynamic);
        return body;
    }

    private static void Step(GravitasWorldContext context)
    {
        context.Simulate();
        context.LateSimulate();
    }

    private static LSCollider2D CreateStaticCollider(
        GravitasWorldContext context,
        LSCollider2D collider,
        Vector2d position)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        collider.InitializeWithNoBody(agent);
        return collider;
    }

    private static long MeasureAllocatedBytes(Action action)
        => AllocationTestHelper.MeasureSinglePass(action);
}
