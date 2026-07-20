using FixedMathSharp;
using FixedMathSharp.Bounds;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Grids;
using GridForge.Spatial;
using SwiftCollections.Diagnostics;
using System;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class ColliderOwnershipStateTests
{
    [Fact]
    public void BodyInitialization_ShouldRejectNonPositiveConsumedScaleBeforeRuntimeMutation()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();

        var transform3D = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.One));
        var collider3D = new LSSphereCollider { Radius = Fixed64.Half };
        var body3D = new SolidBody(new TestMatterAgent(context, transform3D), collider3D);

        Action initialize3D = () => body3D.Initialize(Vector3d.Right, FixedQuaternion.Identity);

        initialize3D.Should().Throw<ArgumentException>().WithParameterName("scale");
        body3D.Active.Should().BeFalse();
        body3D.DynamicId.Should().Be(-1);
        collider3D.Id.Should().Be(-1);
        collider3D.Body.Should().BeNull();
        collider3D.HasHostBinding.Should().BeFalse();
        context.Physics.BodyCount.Should().Be(0);
        context.Physics.ColliderCount.Should().Be(0);

        var transform2D = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.One, Fixed64.One, -Fixed64.One));
        var collider2D = new LSCircleCollider2D(Fixed64.Half);
        bool collider2DActiveBefore = collider2D.IsActive;
        var body2D = new SolidBody2D(new TestMatterAgent(context, transform2D), collider2D);

        Action initialize2D = () => body2D.Initialize(Vector2d.Right);

        initialize2D.Should().Throw<ArgumentException>().WithParameterName("scale");
        body2D.Active.Should().BeFalse();
        body2D.DynamicId.Should().Be(-1);
        collider2D.Id.Should().Be(-1);
        collider2D.Body.Should().BeNull();
        collider2D.HasHostBinding.Should().BeFalse();
        collider2D.IsActive.Should().Be(collider2DActiveBefore);
        context.Physics2D.BodyCount.Should().Be(0);
        context.Physics2D.ColliderCount.Should().Be(0);
    }

    [Fact]
    public void StandaloneColliderInitialization_ShouldRejectNonPositiveConsumedWorldScaleBeforeBinding()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Vector3d[] invalid3DScales =
        {
            new(Fixed64.Zero, Fixed64.One, Fixed64.One),
            new(-Fixed64.One, Fixed64.One, Fixed64.One),
            new(Fixed64.One, Fixed64.Zero, Fixed64.One),
            new(Fixed64.One, -Fixed64.One, Fixed64.One),
            new(Fixed64.One, Fixed64.One, Fixed64.Zero),
            new(Fixed64.One, Fixed64.One, -Fixed64.One),
            new(-Fixed64.One, Fixed64.One, -Fixed64.One)
        };
        Vector3d[] invalid2DScales =
        {
            new(Fixed64.Zero, Fixed64.One, Fixed64.One),
            new(-Fixed64.One, Fixed64.One, Fixed64.One),
            new(Fixed64.One, Fixed64.One, Fixed64.Zero),
            new(Fixed64.One, Fixed64.One, -Fixed64.One),
            new(-Fixed64.One, Fixed64.One, -Fixed64.One)
        };

        for (int i = 0; i < invalid3DScales.Length; i++)
        {
            var collider = new LSSphereCollider { Radius = Fixed64.Half };
            var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, invalid3DScales[i]);

            Action initialize = () => collider.InitializeWithNoBody(new TestMatterAgent(context, transform));

            initialize.Should().Throw<ArgumentException>().WithParameterName("scale");
            collider.Id.Should().Be(-1);
            collider.HasHostBinding.Should().BeFalse();
            transform.LocalScale.Should().Be(invalid3DScales[i]);
        }

        for (int i = 0; i < invalid2DScales.Length; i++)
        {
            var collider = new LSCircleCollider2D(Fixed64.Half);
            var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, invalid2DScales[i]);

            Action initialize = () => collider.InitializeWithNoBody(new TestMatterAgent(context, transform));

            initialize.Should().Throw<ArgumentException>().WithParameterName("scale");
            collider.Id.Should().Be(-1);
            collider.HasHostBinding.Should().BeFalse();
            transform.LocalScale.Should().Be(invalid2DScales[i]);
        }

        context.Physics.ColliderCount.Should().Be(0);
        context.Physics2D.ColliderCount.Should().Be(0);
    }

    [Fact]
    public void StandaloneColliderInitialization_ShouldRejectCanceledAncestryReflectionsBeforeBinding()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var parent3D = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(-Fixed64.One, Fixed64.One, Fixed64.One));
        var transform3D = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(-Fixed64.One, Fixed64.One, Fixed64.One),
            parent3D);
        var collider3D = new LSSphereCollider { Radius = Fixed64.Half };

        Action initialize3D = () => collider3D.InitializeWithNoBody(new TestMatterAgent(context, transform3D));

        initialize3D.Should().Throw<ArgumentException>().WithParameterName("scale");
        collider3D.Id.Should().Be(-1);
        collider3D.HasHostBinding.Should().BeFalse();

        var parent2D = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.One, Fixed64.One, -Fixed64.One));
        var transform2D = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.One, Fixed64.One, -Fixed64.One),
            parent2D);
        var collider2D = new LSCircleCollider2D(Fixed64.Half);

        Action initialize2D = () => collider2D.InitializeWithNoBody(new TestMatterAgent(context, transform2D));

        initialize2D.Should().Throw<ArgumentException>().WithParameterName("scale");
        collider2D.Id.Should().Be(-1);
        collider2D.HasHostBinding.Should().BeFalse();
        context.Physics.ColliderCount.Should().Be(0);
        context.Physics2D.ColliderCount.Should().Be(0);
    }

    [Fact]
    public void RuntimeScaleRebuild_ShouldRejectBeforeMutatingColliderStateAndPreserveAuthoredScale()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var transform3D = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        var collider3D = new LSSphereCollider { Radius = Fixed64.Half };
        collider3D.InitializeWithNoBody(new TestMatterAgent(context, transform3D));
        FixedBoundBox bounds3D = collider3D.Bounds;
        int id3D = collider3D.Id;

        var transform2D = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        var collider2D = new LSCircleCollider2D(Fixed64.Half);
        collider2D.InitializeWithNoBody(new TestMatterAgent(context, transform2D));
        FixedBoundArea bounds2D = collider2D.Bounds;
        int id2D = collider2D.Id;

        transform3D.LocalScale = new Vector3d(-Fixed64.One, Fixed64.One, Fixed64.One);
        transform2D.LocalScale = new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero);

        Action rebuild3D = collider3D.Simulate;
        Action rebuild2D = collider2D.Simulate;

        rebuild3D.Should().Throw<ArgumentException>().WithParameterName("scale");
        rebuild2D.Should().Throw<ArgumentException>().WithParameterName("scale");
        collider3D.Bounds.Should().Be(bounds3D);
        collider2D.Bounds.Should().Be(bounds2D);
        collider3D.Id.Should().Be(id3D);
        collider2D.Id.Should().Be(id2D);
        transform3D.LocalScale.Should().Be(new Vector3d(-Fixed64.One, Fixed64.One, Fixed64.One));
        transform2D.LocalScale.Should().Be(new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero));
    }

    [Fact]
    public void RuntimeScaleRebuild_ShouldRejectCanceledAncestryReflectionsWithoutMutation()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var parent3D = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        var transform3D = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One,
            parent3D);
        var collider3D = new LSSphereCollider { Radius = Fixed64.Half };
        collider3D.InitializeWithNoBody(new TestMatterAgent(context, transform3D));
        FixedBoundBox bounds3D = collider3D.Bounds;
        uint version3D = collider3D.RuntimeShapeVersion;
        bool partitioned3D = collider3D.IsPartitioned;

        var parent2D = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        var transform2D = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One,
            parent2D);
        var collider2D = new LSCircleCollider2D(Fixed64.Half);
        collider2D.InitializeWithNoBody(new TestMatterAgent(context, transform2D));
        FixedBoundArea bounds2D = collider2D.Bounds;
        uint version2D = collider2D.RuntimeShapeVersion;
        bool partitioned2D = collider2D.IsPartitioned;

        parent3D.LocalScale = new Vector3d(-Fixed64.One, Fixed64.One, Fixed64.One);
        transform3D.LocalScale = new Vector3d(-Fixed64.One, Fixed64.One, Fixed64.One);
        parent2D.LocalScale = new Vector3d(Fixed64.One, Fixed64.One, -Fixed64.One);
        transform2D.LocalScale = new Vector3d(Fixed64.One, Fixed64.One, -Fixed64.One);

        Action rebuild3D = collider3D.Simulate;
        Action rebuild2D = collider2D.Simulate;

        rebuild3D.Should().Throw<ArgumentException>().WithParameterName("scale");
        rebuild2D.Should().Throw<ArgumentException>().WithParameterName("scale");
        collider3D.Bounds.Should().Be(bounds3D);
        collider3D.RuntimeShapeVersion.Should().Be(version3D);
        collider3D.IsPartitioned.Should().Be(partitioned3D);
        collider2D.Bounds.Should().Be(bounds2D);
        collider2D.RuntimeShapeVersion.Should().Be(version2D);
        collider2D.IsPartitioned.Should().Be(partitioned2D);
        collider3D.Id.Should().BeGreaterThanOrEqualTo(0);
        collider2D.Id.Should().BeGreaterThanOrEqualTo(0);
    }

    [Fact]
    public void ParentedColliderScale_ShouldConsumePositiveHierarchyLossyScale()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var parent = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One * (Fixed64)2);
        var transform3D = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One * (Fixed64)3,
            parent);
        var collider3D = new LSSphereCollider { Radius = Fixed64.Half };
        collider3D.InitializeWithNoBody(new TestMatterAgent(context, transform3D));

        var transform2D = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One * (Fixed64)3,
            parent);
        var collider2D = new LSCircleCollider2D(Fixed64.Half);
        collider2D.InitializeWithNoBody(new TestMatterAgent(context, transform2D));

        collider3D.LocalScale.Should().Be(Vector3d.One * (Fixed64)6);
        collider3D.ScaledRadius.Should().Be((Fixed64)3);
        collider2D.LocalScale.Should().Be(Vector2d.One * (Fixed64)6);
        collider2D.ScaledRadius.Should().Be((Fixed64)3);
    }

    [Fact]
    public void UnboundColliderPositionAndRotationSetters_ShouldThrow()
    {
        var collider = new LSSphereCollider();

        collider.IsActive = false;
        collider.IsActive = false;
        collider.IsActive = true;
        collider.IsActive = true;

        Action setPosition = () => collider.Position = Vector3d.Right;
        Action setRotation = () => collider.Rotation = FixedQuaternion.Identity;

        setPosition.Should().Throw<InvalidOperationException>();
        setRotation.Should().Throw<InvalidOperationException>();
        collider.IsActive.Should().BeTrue();
    }

    [Fact]
    public void UnboundColliderWorldAndTransform_ShouldExposeMissingBinding()
    {
        var collider = new LSSphereCollider();

        Action readContext = () => _ = collider.Context;
        Action readWorld = () => _ = collider.World;
        Action readTransform = () => _ = collider.Transform;
        Action readPosition = () => _ = collider.Position;
        Action readRotation = () => _ = collider.Rotation;

        readContext.Should().Throw<InvalidOperationException>();
        readWorld.Should().Throw<InvalidOperationException>();
        readTransform.Should().Throw<InvalidOperationException>();
        readPosition.Should().Throw<InvalidOperationException>();
        readRotation.Should().Throw<InvalidOperationException>();
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
        collider.Position = transform.WorldPosition;
        agent.Transform.LocalPosition.Should().Be(transform.LocalPosition);
        collider.Position = Vector3d.Forward;
        agent.Transform.LocalPosition.Should().Be(Vector3d.Forward);
        collider.Rotation = FixedQuaternion.Identity;
        collider.Rotation.Should().Be(FixedQuaternion.Identity);

        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.Half);
        collider.Rotation = rotation;

        agent.Transform.LocalRotation.Should().Be(collider.Rotation);
        agent.Transform.LocalRotation.Should().NotBe(FixedQuaternion.Identity);
        collider.Transform.Should().BeSameAs(transform);
    }

    [Fact]
    public void BodylessColliderPose_ShouldUseParentedWorldContractAndRejectSingularParentWritesAtomically()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var parent = new FixedTransform(
            new Vector3d((Fixed64)10, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);
        var transform = new FixedTransform(
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One,
            parent);
        var collider = new LSSphereCollider();
        collider.InitializeWithNoBody(new TestMatterAgent(scenario.Context, transform));
        FixedQuaternion worldRotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.Half);

        collider.Position = new Vector3d((Fixed64)20, Fixed64.One, (Fixed64)3);
        collider.Rotation = worldRotation;

        collider.Position.Should().Be(new Vector3d((Fixed64)20, Fixed64.One, (Fixed64)3));
        transform.WorldPosition.Should().Be(collider.Position);
        transform.LocalPosition.Should().Be(new Vector3d((Fixed64)10, Fixed64.One, (Fixed64)3));
        transform.WorldRotation.Should().Be(worldRotation.Normalized);

        var singularParent = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.One));
        transform.SetParentKeepingLocal(singularParent);
        Vector3d localPosition = transform.LocalPosition;
        FixedQuaternion localRotation = transform.LocalRotation;

        Action setPosition = () => collider.Position = Vector3d.Right;
        Action setRotation = () => collider.Rotation = FixedQuaternion.Identity;

        setPosition.Should().Throw<InvalidOperationException>();
        setRotation.Should().Throw<InvalidOperationException>();
        transform.LocalPosition.Should().Be(localPosition);
        transform.LocalRotation.Should().Be(localRotation);
    }

    [Fact]
    public void BodylessInitialization_ShouldRejectDuplicateAndForeignBindingButAllowSameAgentAfterReset()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var agent3D = new TestMatterAgent(scenario.Context);
        var agent2D = new TestMatterAgent(scenario.Context);
        var collider3D = new LSSphereCollider();
        var collider2D = new LSCircleCollider2D(Fixed64.Half);
        collider3D.InitializeWithNoBody(agent3D);
        collider2D.InitializeWithNoBody(agent2D);
        int id3D = collider3D.Id;
        int id2D = collider2D.Id;

        Action duplicate3D = () => collider3D.InitializeWithNoBody(agent3D);
        Action duplicate2D = () => collider2D.InitializeWithNoBody(agent2D);

        duplicate3D.Should().Throw<ArgumentException>().WithParameterName("agent");
        duplicate2D.Should().Throw<ArgumentException>().WithParameterName("agent");
        collider3D.Id.Should().Be(id3D);
        collider2D.Id.Should().Be(id2D);
        scenario.Context.Physics.ColliderCount.Should().Be(1);
        scenario.Context.Physics2D.ColliderCount.Should().Be(1);

        scenario.Context.Reset();
        var foreignAgent3D = new TestMatterAgent(scenario.Context);
        var foreignAgent2D = new TestMatterAgent(scenario.Context);

        Action foreign3D = () => collider3D.InitializeWithNoBody(foreignAgent3D);
        Action foreign2D = () => collider2D.InitializeWithNoBody(foreignAgent2D);

        foreign3D.Should().Throw<ArgumentException>().WithParameterName("agent");
        foreign2D.Should().Throw<ArgumentException>().WithParameterName("agent");
        collider3D.Transform.Should().BeSameAs(agent3D.Transform);
        collider2D.Context.Should().BeSameAs(agent2D.Context);

        collider3D.InitializeWithNoBody(agent3D);
        collider2D.InitializeWithNoBody(agent2D);

        collider3D.Id.Should().BeGreaterThanOrEqualTo(0);
        collider2D.Id.Should().BeGreaterThanOrEqualTo(0);
        collider3D.IsPartitioned.Should().BeTrue();
        collider2D.IsPartitioned.Should().BeTrue();
        scenario.Context.Physics.ColliderCount.Should().Be(1);
        scenario.Context.Physics2D.ColliderCount.Should().Be(1);
        scenario.Context.Query3D.Raycast(
            -Vector3d.Right * (Fixed64)2,
            Vector3d.Right,
            (Fixed64)4,
            out _,
            PhysicsLayerMask.All).Should().BeTrue();
        scenario.Context.Query2D.OverlapCircle(
            Vector2d.Zero,
            Fixed64.One,
            out _).Should().BeTrue();
    }

    [Fact]
    public void CustomColliderExtensionHooks_ShouldPreserveRadiusPolicyAndAllowInactiveInitialization()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var collider = new UnsupportedTestCollider3D { DeactivateOnInitialize = true };
        Vector3d authoredSize = collider.Size;

        collider.Radius = (Fixed64)2;
        collider.InitializeWithNoBody(new TestMatterAgent(scenario.Context));

        collider.Radius.Should().Be((Fixed64)2);
        collider.Size.Should().Be(authoredSize);
        collider.IsActive.Should().BeFalse();
        collider.IsPartitioned.Should().BeFalse();
        collider.PartitionCoordinates.Should().BeNull();
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

        collider3D.HasHostBinding.Should().BeFalse();
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
        body.Initialize(transform.WorldPosition, transform.WorldRotation);
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
        parent.Collider.IsChild.Should().BeFalse();
        firstChild.Collider.IsChild.Should().BeTrue();
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

        node3D.TryGetHierarchyColliderByKey(ColliderHierarchyKey.Create2D(10_000), out _).Should().BeFalse();
        node2D.TryGetHierarchyColliderByKey(ColliderHierarchyKey.Create3D(10_000), out _).Should().BeFalse();
        node3D.TryGetHierarchyColliderByKey(ColliderHierarchyKey.Create3D(10_000), out _).Should().BeFalse();
        node2D.TryGetHierarchyColliderByKey(ColliderHierarchyKey.Create2D(10_000), out _).Should().BeFalse();
    }

    [Fact]
    public void CompoundParts_ShouldRejectStandaloneBindingAndForeignOwners()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var owner3D = new LSCompoundCollider(CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero));
        var foreign3D = new LSCompoundCollider(CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero));
        var owner2D = new LSCompoundCollider2D(CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero));
        var foreign2D = new LSCompoundCollider2D(CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero));
        LSCollider part3D = owner3D.GetPartCollider(0);
        LSCollider2D part2D = owner2D.GetPartCollider(0);
        var standalone3D = new LSSphereCollider();
        var standalone2D = new LSCircleCollider2D(Fixed64.Half);
        scenario.InitializeStaticCollider(standalone3D, Vector3d.Zero);
        standalone2D.InitializeWithNoBody(new TestMatterAgent(scenario.Context));

        Action reserveBound3D = () => standalone3D.ReserveCompoundPart(owner3D);
        Action reserveBound2D = () => standalone2D.ReserveCompoundPart(owner2D, Fixed64.Zero, Vector2d.One);
        Action rebindPart3D = () => part3D.ReserveCompoundPart(foreign3D);
        Action rebindPart2D = () => part2D.ReserveCompoundPart(foreign2D, Fixed64.Zero, Vector2d.One);

        reserveBound3D.Should().Throw<ArgumentException>().WithParameterName("owner");
        reserveBound2D.Should().Throw<ArgumentException>().WithParameterName("owner");
        rebindPart3D.Should().Throw<ArgumentException>().WithParameterName("owner");
        rebindPart2D.Should().Throw<ArgumentException>().WithParameterName("owner");
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
    public void Deactivate_ShouldUseSingleServiceOwnedTeardownForBodyOwnedBodylessAndInactiveColliders()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> direct = scenario.CreateSphere(Vector3d.Zero);
        LSSphereCollider bodyless = scenario.CreateStaticSphere(Vector3d.Right * (Fixed64)3);
        ScenarioBody<LSSphereCollider> inactive = scenario.CreateSphere(Vector3d.Right * (Fixed64)6);
        inactive.Collider.IsActive = false;
        inactive.Collider.Simulate();
        inactive.Collider.IsPartitioned.Should().BeFalse();
        bool originalDebugLogging = GravitasLogger.EnableDebugLogging;
        DiagnosticLevel originalMinimumLevel = GravitasLogger.MinimumLevel;
        Action<DiagnosticLevel, string, string> originalLogHandler = GravitasLogger.LogHandler;
        var entries = new List<(DiagnosticLevel Level, string Message)>();

        try
        {
            GravitasLogger.EnableDebugLogging = true;
            GravitasLogger.MinimumLevel = DiagnosticLevel.Info;
            GravitasLogger.LogHandler = (level, message, _) => entries.Add((level, message));

            direct.Collider.Deactivate();
            direct.Collider.Deactivate();
            bodyless.Deactivate();
            bodyless.Deactivate();
            inactive.Body.Deactivate();
            inactive.Body.Deactivate();

            direct.Body.Active.Should().BeFalse();
            direct.Body.DynamicId.Should().Be(-1);
            direct.Collider.IsActive.Should().BeFalse();
            direct.Collider.Id.Should().Be(-1);
            bodyless.IsActive.Should().BeFalse();
            bodyless.Id.Should().Be(-1);
            inactive.Body.Active.Should().BeFalse();
            inactive.Body.DynamicId.Should().Be(-1);
            inactive.Collider.Id.Should().Be(-1);
            scenario.Context.Physics.BodyCount.Should().Be(0);
            scenario.Context.Physics.ColliderCount.Should().Be(0);
            direct.Collider.HasHostBinding.Should().BeFalse();
            entries.Should().NotContain(entry =>
                entry.Message.Contains("non-partitioned collider", StringComparison.Ordinal)
                || entry.Message.Contains("cannot be dessimilated", StringComparison.Ordinal));

            using GravitasWorldContext reuseContext = GravitasWorldContext.CreateOwned();
            var reuseTransform = new FixedTransform(
                Vector3d.Right * (Fixed64)9,
                FixedQuaternion.Identity,
                Vector3d.One);
            direct.Collider.InitializeWithNoBody(new TestMatterAgent(reuseContext, reuseTransform));

            direct.Collider.Body.Should().BeNull();
            direct.Collider.Context.Should().BeSameAs(reuseContext);
            direct.Collider.Transform.Should().BeSameAs(reuseTransform);
            direct.Collider.Position.Should().Be(Vector3d.Right * (Fixed64)9);
            direct.Collider.Id.Should().Be(0);
            reuseContext.Physics.ColliderCount.Should().Be(1);

            direct.Body.Deactivate();

            direct.Collider.IsActive.Should().BeTrue();
            direct.Collider.Id.Should().Be(0);
            direct.Collider.Context.Should().BeSameAs(reuseContext);
            reuseContext.Physics.ColliderCount.Should().Be(1);

            Action staleInitialize = () => direct.Body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);

            staleInitialize.Should().Throw<ArgumentException>().WithParameterName("Collider");
            direct.Body.Active.Should().BeFalse();
            scenario.Context.Physics.BodyCount.Should().Be(0);
            scenario.Context.Physics.ColliderCount.Should().Be(0);
            direct.Collider.Body.Should().BeNull();
            direct.Collider.Context.Should().BeSameAs(reuseContext);
            direct.Collider.Transform.Should().BeSameAs(reuseTransform);
            direct.Collider.Position.Should().Be(Vector3d.Right * (Fixed64)9);
            direct.Collider.Id.Should().Be(0);
            reuseContext.Physics.ColliderCount.Should().Be(1);

            reuseContext.Reset();
            direct.Collider.Id.Should().Be(-1);
            direct.Collider.HasHostBinding.Should().BeTrue();

            staleInitialize.Should().Throw<ArgumentException>().WithParameterName("Collider");
            direct.Body.Active.Should().BeFalse();
            scenario.Context.Physics.BodyCount.Should().Be(0);
            scenario.Context.Physics.ColliderCount.Should().Be(0);
            direct.Collider.Body.Should().BeNull();
            direct.Collider.Context.Should().BeSameAs(reuseContext);
            direct.Collider.Transform.Should().BeSameAs(reuseTransform);
            direct.Collider.Position.Should().Be(Vector3d.Right * (Fixed64)9);
            reuseContext.Physics.ColliderCount.Should().Be(0);
        }
        finally
        {
            GravitasLogger.LogHandler = originalLogHandler;
            GravitasLogger.MinimumLevel = originalMinimumLevel;
            GravitasLogger.EnableDebugLogging = originalDebugLogging;
        }
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
    public void IsActiveSetter_WithMixed3DStaticCollider_ShouldRefreshPartitionsAndQueryVisibility()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        LSSphereCollider collider = scenario.CreateStaticSphere(Vector3d.Zero);
        scenario.Context.MixedCollisions.Refresh3DColliderPartition(collider);
        PhysicsMixedPartition mixedPartition =
            GetMixedPartition(scenario.Context, collider.MixedPartitionCoordinates![0]);

        collider.IsPartitioned.Should().BeTrue();
        collider.IsMixedPartitioned.Should().BeTrue();
        PartitionContains3DCollider(mixedPartition, collider.Id).Should().BeTrue();

        collider.IsActive = false;

        collider.IsPartitioned.Should().BeFalse();
        collider.IsMixedPartitioned.Should().BeFalse();
        (collider.PartitionCoordinates?.Count ?? 0).Should().Be(0);
        (collider.MixedPartitionCoordinates?.Count ?? 0).Should().Be(0);
        PartitionContains3DCollider(mixedPartition, collider.Id).Should().BeFalse();
        scenario.Context.Query3D.Raycast(
            -Vector3d.Right * (Fixed64)2,
            Vector3d.Right,
            (Fixed64)4,
            out _,
            PhysicsLayerMask.All).Should().BeFalse();

        collider.IsActive = true;

        collider.IsPartitioned.Should().BeTrue();
        collider.IsMixedPartitioned.Should().BeTrue();
        PhysicsMixedPartition refreshedMixedPartition =
            GetMixedPartition(scenario.Context, collider.MixedPartitionCoordinates![0]);
        PartitionContains3DCollider(refreshedMixedPartition, collider.Id).Should().BeTrue();
        scenario.Context.Query3D.Raycast(
            -Vector3d.Right * (Fixed64)2,
            Vector3d.Right,
            (Fixed64)4,
            out _,
            PhysicsLayerMask.All).Should().BeTrue();
    }

    [Fact]
    public void IsActiveSetter_WithPure3DStaticCollider_ShouldRefreshOnlyPrimaryPartition()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider collider = scenario.CreateStaticSphere(Vector3d.Zero);

        collider.IsPartitioned.Should().BeTrue();
        collider.IsMixedPartitioned.Should().BeFalse();

        collider.IsActive = false;

        collider.IsPartitioned.Should().BeFalse();
        collider.IsMixedPartitioned.Should().BeFalse();

        collider.IsActive = true;

        collider.IsPartitioned.Should().BeTrue();
        collider.IsMixedPartitioned.Should().BeFalse();
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

    [Fact]
    public void DirectColliderNotifications_ShouldHonorTriggerBodyAndBodylessContactPolicy()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere(scenario, Vector3d.Zero, isParent: false);
        SolidBody2D body2D = CreateCircle2D(scenario.Context, Vector2d.Zero);
        var bodyless3D = new LSSphereCollider();
        var bodyless2D = new LSCircleCollider2D(Fixed64.Half);
        var trigger3D = new LSSphereCollider { IsTrigger = true };
        var trigger2D = new LSCircleCollider2D(Fixed64.Half) { IsTrigger = true };
        int entered3D = 0;
        int stayed3D = 0;
        int contacted3D = 0;
        int entered2D = 0;
        int stayed2D = 0;
        int contacted2D = 0;
        int mixedEntered3D = 0;
        int mixedStayed3D = 0;
        int mixedEntered2D = 0;
        int mixedStayed2D = 0;

        trigger3D.OnTriggerEnter += _ => entered3D++;
        trigger3D.OnTriggerStay += _ => stayed3D++;
        body3D.Collider.OnContact += _ => contacted3D++;
        trigger2D.OnTriggerEnter += _ => entered2D++;
        trigger2D.OnTriggerStay += _ => stayed2D++;
        body2D.Collider.OnContact += _ => contacted2D++;
        trigger3D.OnMixedTriggerEnter += _ => mixedEntered3D++;
        trigger3D.OnMixedTriggerStay += _ => mixedStayed3D++;
        trigger2D.OnMixedTriggerEnter += _ => mixedEntered2D++;
        trigger2D.OnMixedTriggerStay += _ => mixedStayed2D++;

        trigger3D.NotifyContact(body3D.Collider, isColliding: true, isChanged: true);
        trigger3D.NotifyContact(body3D.Collider, isColliding: false, isChanged: true);
        trigger3D.NotifyContact(bodyless3D, isColliding: true, isChanged: true);
        body3D.Collider.NotifyContact(bodyless3D, isColliding: true, isChanged: true);

        trigger2D.NotifyContact(body2D.Collider, isColliding: true, isChanged: true);
        trigger2D.NotifyContact(body2D.Collider, isColliding: false, isChanged: true);
        trigger2D.NotifyContact(bodyless2D, isColliding: true, isChanged: true);
        body2D.Collider.NotifyContact(bodyless2D, isColliding: true, isChanged: true);

        trigger3D.NotifyMixedContact(body2D.Collider, isColliding: true, isChanged: true, isTriggerPair: true);
        trigger3D.NotifyMixedContact(body2D.Collider, isColliding: false, isChanged: true, isTriggerPair: true);
        trigger3D.NotifyMixedContact(bodyless2D, isColliding: true, isChanged: true, isTriggerPair: true);
        trigger2D.NotifyMixedContact(body3D.Collider, isColliding: true, isChanged: true, isTriggerPair: true);
        trigger2D.NotifyMixedContact(body3D.Collider, isColliding: false, isChanged: true, isTriggerPair: true);
        trigger2D.NotifyMixedContact(bodyless3D, isColliding: true, isChanged: true, isTriggerPair: true);

        entered3D.Should().Be(1);
        stayed3D.Should().Be(1);
        contacted3D.Should().Be(0);
        entered2D.Should().Be(1);
        stayed2D.Should().Be(1);
        contacted2D.Should().Be(0);
        mixedEntered3D.Should().Be(1);
        mixedStayed3D.Should().Be(1);
        mixedEntered2D.Should().Be(1);
        mixedStayed2D.Should().Be(1);
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

    private static bool PartitionContains3DCollider(PhysicsMixedPartition partition, int colliderId) =>
        partition.ContainedDynamic3DObjects?.Contains(colliderId) == true
        || partition.ContainedKinematic3DObjects?.Contains(colliderId) == true
        || partition.ContainedStatic3DObjects?.Contains(colliderId) == true;

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
