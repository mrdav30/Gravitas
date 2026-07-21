using FixedMathSharp;
using FixedMathSharp.Bounds;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using System;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class BodyMotionTypeTests
{
    [Theory]
    [InlineData(BodyMotionType.Dynamic)]
    [InlineData(BodyMotionType.Static)]
    public void SetMotionType_BeforeInitialization_ShouldRejectSameOrDifferentRoleWithoutChangingEitherBody(
        BodyMotionType requestedMotionType)
    {
        using GravitasWorldContext context3D = GravitasWorldContext.CreateOwned();
        var body3D = new SolidBody(
            new TestMatterAgent(
                context3D,
                new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)),
            new LSSphereCollider());
        using GravitasWorldContext context2D = Physics2DTestWorld.CreateContext();
        var body2D = new SolidBody2D(
            new TestMatterAgent(
                context2D,
                new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)),
            new LSCircleCollider2D(Fixed64.One));

        Action transition3D = () => body3D.SetMotionType(requestedMotionType);
        Action transition2D = () => body2D.SetMotionType(requestedMotionType);

        transition3D.Should().Throw<InvalidOperationException>();
        transition2D.Should().Throw<InvalidOperationException>();
        body3D.MotionType.Should().Be(BodyMotionType.Dynamic);
        body2D.MotionType.Should().Be(BodyMotionType.Dynamic);
        context3D.Physics.BodyCount.Should().Be(0);
        context2D.Physics2D.BodyCount.Should().Be(0);
    }

    [Theory]
    [InlineData(BodyMotionType.Dynamic)]
    [InlineData(BodyMotionType.Static)]
    public void SetMotionType_AfterContextReset_ShouldRejectSameOrDifferentRoleWithoutMutation(
        BodyMotionType requestedMotionType)
    {
        using GravitasWorldContext context3D = GravitasWorldContext.CreateOwned();
        (SolidBody body3D, _) = Create3DBody(context3D, BodyMotionType.Dynamic);
        using GravitasWorldContext context2D = Physics2DTestWorld.CreateContext();
        (SolidBody2D body2D, _) = Create2DBody(context2D, BodyMotionType.Dynamic);
        int dynamicId3D = body3D.DynamicId;
        int dynamicId2D = body2D.DynamicId;
        context3D.Reset();
        context2D.Reset();

        Action transition3D = () => body3D.SetMotionType(requestedMotionType);
        Action transition2D = () => body2D.SetMotionType(requestedMotionType);

        transition3D.Should().Throw<InvalidOperationException>();
        transition2D.Should().Throw<InvalidOperationException>();
        body3D.MotionType.Should().Be(BodyMotionType.Dynamic);
        body2D.MotionType.Should().Be(BodyMotionType.Dynamic);
        body3D.DynamicId.Should().Be(dynamicId3D);
        body2D.DynamicId.Should().Be(dynamicId2D);
        context3D.Physics.BodyCount.Should().Be(0);
        context2D.Physics2D.BodyCount.Should().Be(0);
    }

    [Fact]
    public void SolidBody_StaticInitialization_ShouldUseExplicitStaticRole()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var collider = new LSSphereCollider();
        var body = new SolidBody(
            new TestMatterAgent(
                context,
                new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)),
            collider)
        {
            Mass = Fixed64.One
        };

        body.Initialize(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            BodyMotionType.Static);

        body.MotionType.Should().Be(BodyMotionType.Static);
        body.IsStatic.Should().BeTrue();
        body.IsKinematic.Should().BeFalse();
        body.IsDynamic.Should().BeFalse();
        body.DynamicId.Should().Be(-1);
        collider.IsStatic.Should().BeTrue();
        context.Physics.BodyCount.Should().Be(0);
    }

    [Fact]
    public void SolidBody2D_StaticInitialization_ShouldUseExplicitStaticRole()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var collider = new LSCircleCollider2D(Fixed64.One);
        var body = new SolidBody2D(
            new TestMatterAgent(
                context,
                new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)),
            collider)
        {
            Mass = Fixed64.One
        };

        body.Initialize(
            Vector2d.Zero,
            Fixed64.Zero,
            BodyMotionType.Static);

        body.MotionType.Should().Be(BodyMotionType.Static);
        body.IsStatic.Should().BeTrue();
        body.IsKinematic.Should().BeFalse();
        body.IsDynamic.Should().BeFalse();
        body.DynamicId.Should().Be(-1);
        collider.IsStatic.Should().BeTrue();
        context.Physics2D.BodyCount.Should().Be(0);
    }

    [Fact]
    public void SolidBody_DynamicToStatic_ShouldReleaseOnlySimulatedBodyMembership()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        (SolidBody body, LSSphereCollider collider) = Create3DBody(context, BodyMotionType.Dynamic);
        int colliderId = collider.Id;

        body.SetMotionType(BodyMotionType.Static);

        body.MotionType.Should().Be(BodyMotionType.Static);
        body.DynamicId.Should().Be(-1);
        context.Physics.BodyCount.Should().Be(0);
        collider.Id.Should().Be(colliderId);
        collider.Body.Should().BeSameAs(body);
        collider.IsStatic.Should().BeTrue();
    }

    [Fact]
    public void SolidBody_StaticToDynamic_ShouldAcquireSimulatedBodyMembership()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        (SolidBody body, LSSphereCollider collider) = Create3DBody(context, BodyMotionType.Static);
        int colliderId = collider.Id;

        body.SetMotionType(BodyMotionType.Dynamic);

        body.MotionType.Should().Be(BodyMotionType.Dynamic);
        body.DynamicId.Should().BeGreaterThanOrEqualTo(0);
        context.Physics.BodyCount.Should().Be(1);
        collider.Id.Should().Be(colliderId);
        collider.Body.Should().BeSameAs(body);
        collider.IsStatic.Should().BeFalse();
    }

    [Fact]
    public void SolidBody2D_DynamicToStatic_ShouldReleaseOnlySimulatedBodyMembership()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        (SolidBody2D body, LSCircleCollider2D collider) = Create2DBody(context, BodyMotionType.Dynamic);
        int colliderId = collider.Id;

        body.SetMotionType(BodyMotionType.Static);

        body.MotionType.Should().Be(BodyMotionType.Static);
        body.DynamicId.Should().Be(-1);
        context.Physics2D.BodyCount.Should().Be(0);
        collider.Id.Should().Be(colliderId);
        collider.Body.Should().BeSameAs(body);
        collider.IsStatic.Should().BeTrue();
    }

    [Fact]
    public void SolidBody2D_StaticToDynamic_ShouldAcquireSimulatedBodyMembership()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        (SolidBody2D body, LSCircleCollider2D collider) = Create2DBody(context, BodyMotionType.Static);
        int colliderId = collider.Id;

        body.SetMotionType(BodyMotionType.Dynamic);

        body.MotionType.Should().Be(BodyMotionType.Dynamic);
        body.DynamicId.Should().BeGreaterThanOrEqualTo(0);
        context.Physics2D.BodyCount.Should().Be(1);
        collider.Id.Should().Be(colliderId);
        collider.Body.Should().BeSameAs(body);
        collider.IsStatic.Should().BeFalse();
    }

    [Theory]
    [InlineData(BodyMotionType.Kinematic, BodyMotionType.Dynamic)]
    [InlineData(BodyMotionType.Kinematic, BodyMotionType.Static)]
    [InlineData(BodyMotionType.Static, BodyMotionType.Kinematic)]
    public void SolidBody_RemainingMotionTypeTransitions_ShouldPreserveIdentityAndReconcileMembership(
        BodyMotionType initialMotionType,
        BodyMotionType targetMotionType)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        (SolidBody body, LSSphereCollider collider) = Create3DBody(context, initialMotionType);
        int colliderId = collider.Id;

        body.SetMotionType(targetMotionType);

        body.MotionType.Should().Be(targetMotionType);
        collider.Id.Should().Be(colliderId);
        collider.Body.Should().BeSameAs(body);
        collider.IsStatic.Should().Be(targetMotionType == BodyMotionType.Static);
        context.Physics.BodyCount.Should().Be(targetMotionType == BodyMotionType.Static ? 0 : 1);
        if (targetMotionType == BodyMotionType.Static)
            body.DynamicId.Should().Be(-1);
        else
            body.DynamicId.Should().BeGreaterThanOrEqualTo(0);

        if (targetMotionType == BodyMotionType.Dynamic)
        {
            body.AddAngularImpulse(Vector3d.Up);
            body.AngularVelocity.Should().NotBe(Vector3d.Zero);
        }
    }

    [Theory]
    [InlineData(BodyMotionType.Kinematic, BodyMotionType.Dynamic)]
    [InlineData(BodyMotionType.Kinematic, BodyMotionType.Static)]
    [InlineData(BodyMotionType.Static, BodyMotionType.Kinematic)]
    public void SolidBody2D_RemainingMotionTypeTransitions_ShouldPreserveIdentityAndReconcileMembership(
        BodyMotionType initialMotionType,
        BodyMotionType targetMotionType)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        (SolidBody2D body, LSCircleCollider2D collider) = Create2DBody(context, initialMotionType);
        int colliderId = collider.Id;

        body.SetMotionType(targetMotionType);

        body.MotionType.Should().Be(targetMotionType);
        collider.Id.Should().Be(colliderId);
        collider.Body.Should().BeSameAs(body);
        collider.IsStatic.Should().Be(targetMotionType == BodyMotionType.Static);
        context.Physics2D.BodyCount.Should().Be(targetMotionType == BodyMotionType.Static ? 0 : 1);
        if (targetMotionType == BodyMotionType.Static)
            body.DynamicId.Should().Be(-1);
        else
            body.DynamicId.Should().BeGreaterThanOrEqualTo(0);

        if (targetMotionType == BodyMotionType.Dynamic)
        {
            body.AddAngularImpulse(Fixed64.One);
            body.AngularVelocity.Should().NotBe(Fixed64.Zero);
        }
    }

    [Fact]
    public void SolidBody_SameAndUndefinedMotionTypeRequests_ShouldBeNoOpOrRejectWithoutMutation()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        (SolidBody body, _) = Create3DBody(context, BodyMotionType.Dynamic);
        int dynamicId = body.DynamicId;
        body.AddAngularImpulse(Vector3d.Up);
        Vector3d angularVelocity = body.AngularVelocity;

        body.SetMotionType(BodyMotionType.Dynamic);
        Action invalid = () => body.SetMotionType((BodyMotionType)byte.MaxValue);

        invalid.Should().Throw<ArgumentOutOfRangeException>();
        body.MotionType.Should().Be(BodyMotionType.Dynamic);
        body.DynamicId.Should().Be(dynamicId);
        body.AngularVelocity.Should().Be(angularVelocity);
    }

    [Fact]
    public void SolidBody2D_SameAndUndefinedMotionTypeRequests_ShouldBeNoOpOrRejectWithoutMutation()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        (SolidBody2D body, _) = Create2DBody(context, BodyMotionType.Dynamic);
        int dynamicId = body.DynamicId;
        body.AddAngularImpulse(Fixed64.One);
        Fixed64 angularVelocity = body.AngularVelocity;

        body.SetMotionType(BodyMotionType.Dynamic);
        Action invalid = () => body.SetMotionType((BodyMotionType)byte.MaxValue);

        invalid.Should().Throw<ArgumentOutOfRangeException>();
        body.MotionType.Should().Be(BodyMotionType.Dynamic);
        body.DynamicId.Should().Be(dynamicId);
        body.AngularVelocity.Should().Be(angularVelocity);
    }

    [Fact]
    public void SolidBody_MotionTypeChangeDuringOpenFixedStep_ShouldFailWithoutMutation()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        (SolidBody body, _) = Create3DBody(context, BodyMotionType.Dynamic);
        int dynamicId = body.DynamicId;
        context.Simulate();

        Action transition = () => body.SetMotionType(BodyMotionType.Static);

        transition.Should().Throw<InvalidOperationException>();
        body.MotionType.Should().Be(BodyMotionType.Dynamic);
        body.DynamicId.Should().Be(dynamicId);
        context.Physics.BodyCount.Should().Be(1);

        context.LateSimulate();
        body.SetMotionType(BodyMotionType.Static);
        body.IsStatic.Should().BeTrue();
    }

    [Fact]
    public void SolidBody2D_MotionTypeChangeDuringOpenFixedStep_ShouldFailWithoutMutation()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        (SolidBody2D body, _) = Create2DBody(context, BodyMotionType.Dynamic);
        int dynamicId = body.DynamicId;
        context.Simulate();

        Action transition = () => body.SetMotionType(BodyMotionType.Static);

        transition.Should().Throw<InvalidOperationException>();
        body.MotionType.Should().Be(BodyMotionType.Dynamic);
        body.DynamicId.Should().Be(dynamicId);
        context.Physics2D.BodyCount.Should().Be(1);

        context.LateSimulate();
        body.SetMotionType(BodyMotionType.Static);
        body.IsStatic.Should().BeTrue();
    }

    [Fact]
    public void SolidBody_MotionTypeChangeFromSimulateCallback_ShouldFailAndReleasePhaseGuard()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        (SolidBody body, _) = Create3DBody(context, BodyMotionType.Dynamic);
        using IDisposable hook = context.RegisterOnSimulate(
            "motion-type-transition",
            0,
            () => body.SetMotionType(BodyMotionType.Static));

        Action simulate = context.Simulate;

        simulate.Should().Throw<InvalidOperationException>();
        body.MotionType.Should().Be(BodyMotionType.Dynamic);
        hook.Dispose();
        body.SetMotionType(BodyMotionType.Static);
        body.MotionType.Should().Be(BodyMotionType.Static);
    }

    [Fact]
    public void SolidBody2D_MotionTypeChangeFromLateSimulateCallback_ShouldFailAndReleasePhaseGuard()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        (SolidBody2D body, _) = Create2DBody(context, BodyMotionType.Dynamic);
        using IDisposable hook = context.RegisterOnLateSimulate(
            "motion-type-transition",
            0,
            () => body.SetMotionType(BodyMotionType.Static));

        Action lateSimulate = context.LateSimulate;

        lateSimulate.Should().Throw<InvalidOperationException>();
        body.MotionType.Should().Be(BodyMotionType.Dynamic);
        hook.Dispose();
        body.SetMotionType(BodyMotionType.Static);
        body.MotionType.Should().Be(BodyMotionType.Static);
    }

    [Fact]
    public void SolidBody_DynamicToKinematic_ShouldPublishAuthoritativePoseAndClearMotion()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        (SolidBody body, _) = Create3DBody(context, BodyMotionType.Dynamic);
        Vector3d position = new((Fixed64)3, (Fixed64)2, Fixed64.One);
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.Half);
        body.SetPosition(position);
        body.SetRotation(rotation);
        body.AddLinearImpulse(Vector3d.Right);
        body.AddAngularImpulse(Vector3d.Up);

        body.SetMotionType(BodyMotionType.Kinematic);

        body.IsKinematic.Should().BeTrue();
        body.PositionTransform.WorldPosition.Should().Be(position);
        body.RotationTransform.WorldRotation.Should().Be(rotation);
        body.LinearVelocity.Should().Be(Vector3d.Zero);
        body.AngularVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void SolidBody2D_DynamicToKinematic_ShouldPublishAuthoritativePoseAndClearMotion()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        (SolidBody2D body, _) = Create2DBody(context, BodyMotionType.Dynamic);
        Vector2d position = new((Fixed64)3, Fixed64.One);
        Fixed64 rotation = Fixed64.Half;
        body.SetPosition(position);
        body.SetRotation(rotation);
        body.AddLinearImpulse(Vector2d.Right);
        body.AddAngularImpulse(Fixed64.One);

        body.SetMotionType(BodyMotionType.Kinematic);

        body.IsKinematic.Should().BeTrue();
        body.Agent.Transform.WorldPositionXZ.Should().Be(position);
        body.Agent.Transform.WorldRotation.Should().Be(
            FixedQuaternion.FromAxisAngle(Vector3d.Up, -rotation));
        body.LinearVelocity.Should().Be(Vector2d.Zero);
        body.AngularVelocity.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void SolidBody_MotionTypeChange_ShouldClearConnectedContactWarmStartFromEitherPairSide(
        bool transitionPairOwner)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        (SolidBody first, LSSphereCollider firstCollider) = Create3DBody(context, BodyMotionType.Dynamic);
        (SolidBody second, LSSphereCollider secondCollider) = Create3DBody(context, BodyMotionType.Dynamic);
        var pair = new CollisionPair(firstCollider, secondCollider);
        firstCollider.TryAddCollisionPair(secondCollider.Id, pair).Should().BeTrue();
        secondCollider.TryAddCollisionPairHolder(firstCollider.Id).Should().BeTrue();
        pair.StoreWarmStartImpulse(7, Vector3d.Right, Fixed64.One, Fixed64.Half);

        (transitionPairOwner ? first : second).SetMotionType(BodyMotionType.Kinematic);

        pair.TryGetWarmStartImpulse(7, out _).Should().BeFalse();
        firstCollider.TryGetCollisionPair(secondCollider.Id, out CollisionPair? retained).Should().BeTrue();
        retained.Should().BeSameAs(pair);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void SolidBody2D_MotionTypeChange_ShouldClearConnectedContactWarmStartFromEitherPairSide(
        bool transitionPairOwner)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        (SolidBody2D first, LSCircleCollider2D firstCollider) = Create2DBody(context, BodyMotionType.Dynamic);
        (SolidBody2D second, LSCircleCollider2D secondCollider) = Create2DBody(context, BodyMotionType.Dynamic);
        var pair = new CollisionPair2D(firstCollider, secondCollider);
        firstCollider.TryAddCollisionPair(secondCollider.Id, pair).Should().BeTrue();
        secondCollider.TryAddCollisionPairHolder(firstCollider.Id).Should().BeTrue();
        pair.StoreWarmStartImpulse(7, Fixed64.One, Fixed64.Half);

        (transitionPairOwner ? first : second).SetMotionType(BodyMotionType.Kinematic);

        pair.TryGetWarmStartImpulse(7, out _).Should().BeFalse();
        firstCollider.TryGetCollisionPair(secondCollider.Id, out CollisionPair2D? retained).Should().BeTrue();
        retained.Should().BeSameAs(pair);
    }

    [Fact]
    public void SolidBody_StaticReposition_ShouldRefreshPartitionImmediately()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateBody(
            new LSSphereCollider(),
            Vector3d.Zero,
            FixedQuaternion.Identity,
            isDynamic: false);
        var originalCoordinate = body.Collider.PartitionCoordinates![0];

        body.Body.SetPosition(Vector3d.Right * (Fixed64)20);

        body.Collider.PartitionCoordinates.Should().NotContain(originalCoordinate);
        body.Collider.Bounds.Center.Should().Be(body.Body.Position3d);
    }

    [Fact]
    public void SolidBody2D_StaticReposition_ShouldRefreshPartitionImmediately()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        (SolidBody2D body, LSCircleCollider2D collider) = Create2DBody(context, BodyMotionType.Static);
        var originalCoordinate = collider.PartitionCoordinates![0];
        Vector2d position = Vector2d.Right * (Fixed64)20;

        body.SetPosition(position);

        collider.PartitionCoordinates.Should().NotContain(originalCoordinate);
        collider.Bounds.Center.Should().Be(position);
    }

    [Fact]
    public void StaticReposition_InMixedMode_ShouldRefreshBothDimensionEmbeddingsImmediately()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        ScenarioBody<LSSphereCollider> body3D = scenario.CreateBody(
            new LSSphereCollider(),
            Vector3d.Zero,
            FixedQuaternion.Identity,
            isDynamic: false);
        (SolidBody2D body2D, LSCircleCollider2D collider2D) =
            Create2DBody(scenario.Context, BodyMotionType.Static);
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();
        var original3DCoordinate = body3D.Collider.MixedPartitionCoordinates![0];
        var original2DCoordinate = collider2D.MixedPartitionCoordinates![0];

        body3D.Body.SetPosition(Vector3d.Right * (Fixed64)20);
        body2D.SetPosition(Vector2d.Forward * (Fixed64)20);

        body3D.Collider.MixedPartitionCoordinates.Should().NotContain(original3DCoordinate);
        collider2D.MixedPartitionCoordinates.Should().NotContain(original2DCoordinate);
    }

    [Fact]
    public void SolidBody_MotionTypeTransitionWithInvalidRuntimeScale_ShouldRejectBeforeMutation()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        (SolidBody body, LSSphereCollider collider) = Create3DBody(context, BodyMotionType.Dynamic);
        int dynamicId = body.DynamicId;
        int bodyCount = context.Physics.BodyCount;
        body.Agent.Transform.LocalScale = new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.One);

        Action transition = () => body.SetMotionType(BodyMotionType.Static);

        transition.Should().Throw<ArgumentException>();
        body.MotionType.Should().Be(BodyMotionType.Dynamic);
        body.DynamicId.Should().Be(dynamicId);
        context.Physics.BodyCount.Should().Be(bodyCount);
        collider.IsStatic.Should().BeFalse();
        collider.IsPartitioned.Should().BeTrue();
    }

    [Fact]
    public void SolidBody2D_MotionTypeTransitionWithInvalidRuntimeScale_ShouldRejectBeforeMutation()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        (SolidBody2D body, LSCircleCollider2D collider) = Create2DBody(context, BodyMotionType.Dynamic);
        int dynamicId = body.DynamicId;
        int bodyCount = context.Physics2D.BodyCount;
        body.Agent.Transform.LocalScale = new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.One);

        Action transition = () => body.SetMotionType(BodyMotionType.Static);

        transition.Should().Throw<ArgumentException>();
        body.MotionType.Should().Be(BodyMotionType.Dynamic);
        body.DynamicId.Should().Be(dynamicId);
        context.Physics2D.BodyCount.Should().Be(bodyCount);
        collider.IsStatic.Should().BeFalse();
        collider.IsPartitioned.Should().BeTrue();
    }

    [Fact]
    public void SolidBody_StaticPoseChangesDuringOpenFixedStep_ShouldRejectBeforeMutation()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        (SolidBody body, LSSphereCollider collider) = Create3DBody(context, BodyMotionType.Static);
        Vector3d originalPosition = body.Position3d;
        FixedQuaternion originalRotation = body.Rotation;
        FixedBoundBox originalBounds = collider.Bounds;
        context.Simulate();

        Action setPosition = () => body.SetPosition(Vector3d.Right);
        Action setHeight = () => body.SetHeight(Fixed64.One);
        Action setRotation = () => body.SetRotation(FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi));
        Action updateRotation = () => body.UpdateRotation(
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi),
            Fixed64.One);
        Action resetPosition = () => body.ResetPosition(Vector3d.Forward, FixedQuaternion.Identity);

        setPosition.Should().Throw<InvalidOperationException>();
        setHeight.Should().Throw<InvalidOperationException>();
        setRotation.Should().Throw<InvalidOperationException>();
        updateRotation.Should().Throw<InvalidOperationException>();
        resetPosition.Should().Throw<InvalidOperationException>();
        body.Position3d.Should().Be(originalPosition);
        body.Rotation.Should().Be(originalRotation);
        collider.Bounds.Should().Be(originalBounds);
        context.LateSimulate();
    }

    [Fact]
    public void SolidBody2D_StaticPoseChangesDuringOpenFixedStep_ShouldRejectBeforeMutation()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        (SolidBody2D body, LSCircleCollider2D collider) = Create2DBody(context, BodyMotionType.Static);
        Vector2d originalPosition = body.Position;
        Fixed64 originalRotation = body.Rotation;
        FixedBoundArea originalBounds = collider.Bounds;
        context.Simulate();

        Action setPosition = () => body.SetPosition(Vector2d.Right);
        Action setRotation = () => body.SetRotation(Fixed64.HalfPi);
        Action resetPosition = () => body.ResetPosition(Vector2d.Forward, Fixed64.Zero);

        setPosition.Should().Throw<InvalidOperationException>();
        setRotation.Should().Throw<InvalidOperationException>();
        resetPosition.Should().Throw<InvalidOperationException>();
        body.Position.Should().Be(originalPosition);
        body.Rotation.Should().Be(originalRotation);
        collider.Bounds.Should().Be(originalBounds);
        context.LateSimulate();
    }

    [Fact]
    public void SolidBody2D_StaticResetPositionInMixedMode_ShouldRefreshMixedEmbeddingImmediately()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        (SolidBody2D body, LSCircleCollider2D collider) = Create2DBody(context, BodyMotionType.Static);
        context.Simulate();
        context.LateSimulate();
        var originalCoordinate = collider.MixedPartitionCoordinates![0];

        body.ResetPosition(Vector2d.Right * (Fixed64)20);

        collider.MixedPartitionCoordinates!.Contains(originalCoordinate).Should().BeFalse();
        collider.MixedBounds3D.Center.ToVector2d().Should().Be(body.Position);
    }

    [Fact]
    public void SolidBody_StaticUpdateRotationInMixedMode_ShouldRefreshBoundsImmediately()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        var collider = new LSCuboidCollider
        {
            Size = new Vector3d((Fixed64)4, (Fixed64)2, Fixed64.One)
        };
        ScenarioBody<LSCuboidCollider> body = scenario.CreateBody(
            collider,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            isDynamic: false);
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();
        FixedBoundBox originalBounds = collider.Bounds;

        body.Body.UpdateRotation(
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi),
            Fixed64.One);

        collider.Bounds.Should().NotBe(originalBounds);
        Vector3d rotatedSize = collider.Bounds.Max - collider.Bounds.Min;
        Vector3d originalSize = originalBounds.Max - originalBounds.Min;
        rotatedSize.X.Should().Be(originalSize.Z);
        rotatedSize.Z.Should().Be(originalSize.X);
        collider.MixedPartitionCoordinates.Should().NotBeNull();
        collider.IsMixedPartitioned.Should().BeTrue();
    }

    [Fact]
    public void StaticPoseChangeAfterContextResetInMixedMode_ShouldRejectBeforeMutation()
    {
        using GravitasWorldContext context3D = GravitasWorldContext.CreateOwned();
        context3D.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        (SolidBody body3D, LSSphereCollider collider3D) = Create3DBody(context3D, BodyMotionType.Static);
        using GravitasWorldContext context2D = Physics2DTestWorld.CreateContext();
        context2D.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        (SolidBody2D body2D, LSCircleCollider2D collider2D) = Create2DBody(context2D, BodyMotionType.Static);
        Vector3d originalPosition3D = body3D.Position3d;
        Vector2d originalPosition2D = body2D.Position;
        context3D.Reset();
        context2D.Reset();

        Action move3D = () => body3D.SetPosition(Vector3d.Right);
        Action move2D = () => body2D.SetPosition(Vector2d.Right);

        move3D.Should().Throw<InvalidOperationException>();
        move2D.Should().Throw<InvalidOperationException>();
        body3D.Position3d.Should().Be(originalPosition3D);
        body2D.Position.Should().Be(originalPosition2D);
        collider3D.Id.Should().Be(-1);
        collider2D.Id.Should().Be(-1);
        collider3D.IsMixedPartitioned.Should().BeFalse();
        collider2D.IsMixedPartitioned.Should().BeFalse();
        context3D.Physics.BodyCount.Should().Be(0);
        context2D.Physics2D.BodyCount.Should().Be(0);
    }

    [Fact]
    public void SolidBody_ResetPositionAfterContextReset_ShouldRejectBeforeMutation()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        (SolidBody body, LSSphereCollider collider) = Create3DBody(context, BodyMotionType.Dynamic);
        Vector3d originalPosition = body.Position3d;
        FixedQuaternion originalRotation = body.Rotation;
        Vector3d originalHostPosition = body.PositionTransform.WorldPosition;
        FixedQuaternion originalHostRotation = body.RotationTransform.WorldRotation;
        context.Reset();

        Action reset = () => body.ResetPosition(Vector3d.Right, FixedQuaternion.Identity);

        reset.Should().Throw<InvalidOperationException>();
        body.Position3d.Should().Be(originalPosition);
        body.Rotation.Should().Be(originalRotation);
        body.PositionTransform.WorldPosition.Should().Be(originalHostPosition);
        body.RotationTransform.WorldRotation.Should().Be(originalHostRotation);
        collider.Id.Should().Be(-1);
    }

    [Fact]
    public void StaticPoseChangeWithInvalidRuntimeScaleInMixedMode_ShouldRejectBeforeMutation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        (SolidBody body3D, LSSphereCollider collider3D) =
            Create3DBody(scenario.Context, BodyMotionType.Static);
        (SolidBody2D body2D, LSCircleCollider2D collider2D) =
            Create2DBody(scenario.Context, BodyMotionType.Static);
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();
        Vector3d originalPosition3D = body3D.Position3d;
        Vector2d originalPosition2D = body2D.Position;
        FixedBoundBox originalBounds3D = collider3D.Bounds;
        FixedBoundArea originalBounds2D = collider2D.Bounds;
        var originalPureCoordinate3D = collider3D.PartitionCoordinates![0];
        var originalPureCoordinate2D = collider2D.PartitionCoordinates![0];
        var originalMixedCoordinate3D = collider3D.MixedPartitionCoordinates![0];
        var originalMixedCoordinate2D = collider2D.MixedPartitionCoordinates![0];
        body3D.Agent.Transform.LocalScale = new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.One);
        body2D.Agent.Transform.LocalScale = new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.One);

        Action move3D = () => body3D.SetPosition(Vector3d.Right);
        Action move2D = () => body2D.SetPosition(Vector2d.Right);

        move3D.Should().Throw<ArgumentException>();
        move2D.Should().Throw<ArgumentException>();
        body3D.Position3d.Should().Be(originalPosition3D);
        body2D.Position.Should().Be(originalPosition2D);
        collider3D.Bounds.Should().Be(originalBounds3D);
        collider2D.Bounds.Should().Be(originalBounds2D);
        collider3D.PartitionCoordinates!.Contains(originalPureCoordinate3D).Should().BeTrue();
        collider2D.PartitionCoordinates!.Contains(originalPureCoordinate2D).Should().BeTrue();
        collider3D.MixedPartitionCoordinates!.Contains(originalMixedCoordinate3D).Should().BeTrue();
        collider2D.MixedPartitionCoordinates!.Contains(originalMixedCoordinate2D).Should().BeTrue();
    }

    [Fact]
    public void SolidBody_ResetPositionWithUnrepresentableParentRelativePose_ShouldRejectBeforeMutation()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        (SolidBody body, LSSphereCollider collider) = Create3DBody(context, BodyMotionType.Static);
        var parent = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.One));
        body.PositionTransform.SetParentKeepingLocal(parent);
        Vector3d originalPosition = body.Position3d;
        FixedQuaternion originalRotation = body.Rotation;
        Vector3d originalHostPosition = body.PositionTransform.WorldPosition;
        FixedQuaternion originalHostRotation = body.RotationTransform.WorldRotation;
        FixedBoundBox originalBounds = collider.Bounds;

        Action reset = () => body.ResetPosition(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi));

        reset.Should().Throw<InvalidOperationException>();
        body.Position3d.Should().Be(originalPosition);
        body.Rotation.Should().Be(originalRotation);
        body.PositionTransform.WorldPosition.Should().Be(originalHostPosition);
        body.RotationTransform.WorldRotation.Should().Be(originalHostRotation);
        collider.Bounds.Should().Be(originalBounds);
    }

    [Theory]
    [InlineData(BodyMotionType.Dynamic)]
    [InlineData(BodyMotionType.Kinematic)]
    [InlineData(BodyMotionType.Static)]
    public void SolidBody_ResetPositionWithInvalidPostPoseMeshScale_ShouldRejectBeforeMutation(
        BodyMotionType motionType)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var parent = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d((Fixed64)65536, Fixed64.Half, Fixed64.One));
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One,
            parent);
        LSMeshCollider collider = MeshTestFixtures.CreateConvexPolygonFan(
            boundaryVertexCount: 3,
            radius: Fixed64.FromFraction(1, 1024));
        var body = new SolidBody(new TestMatterAgent(context, transform), collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity, motionType);
        Vector3d originalPosition = body.Position3d;
        FixedQuaternion originalRotation = body.Rotation;
        Vector3d originalLocalPosition = transform.LocalPosition;
        FixedQuaternion originalLocalRotation = transform.LocalRotation;
        FixedBoundBox originalBounds = collider.Bounds;
        uint originalShapeVersion = collider.RuntimeShapeVersion;

        Action reset = () => body.ResetPosition(
            Vector3d.Right,
            FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.HalfPi * Fixed64.Half));

        reset.Should().Throw<ArgumentException>().WithMessage("*representable*");
        body.Position3d.Should().Be(originalPosition);
        body.Rotation.Should().Be(originalRotation);
        transform.LocalPosition.Should().Be(originalLocalPosition);
        transform.LocalRotation.Should().Be(originalLocalRotation);
        collider.Bounds.Should().Be(originalBounds);
        collider.RuntimeShapeVersion.Should().Be(originalShapeVersion);
    }

    [Fact]
    public void SolidBody_StaticReentryIntoGrid_ShouldRestoreImmediatePureQueryVisibility()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        (SolidBody body, LSSphereCollider collider) =
            Create3DBody(scenario.Context, BodyMotionType.Static);
        var hits = new SwiftList<Physics3DHit>();

        body.SetPosition(Vector3d.Right * (Fixed64)1000);

        collider.IsPartitioned.Should().BeFalse();

        body.SetPosition(Vector3d.Zero);
        int hitCount = scenario.Context.Query3D.OverlapSphereAgainstStaticAll(
            Vector3d.Zero,
            Fixed64.One,
            PhysicsLayerMask.All,
            hits);

        collider.IsPartitioned.Should().BeTrue();
        hitCount.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(collider);
    }

    [Fact]
    public void DeactivatedStaticBodies_ShouldAcceptOfflinePosePreparationWithoutReregistration()
    {
        using GravitasWorldContext context3D = GravitasWorldContext.CreateOwned();
        (SolidBody body3D, LSSphereCollider collider3D) = Create3DBody(context3D, BodyMotionType.Static);
        using GravitasWorldContext context2D = Physics2DTestWorld.CreateContext();
        (SolidBody2D body2D, LSCircleCollider2D collider2D) = Create2DBody(context2D, BodyMotionType.Static);
        body3D.Deactivate();
        body2D.Deactivate();
        Vector3d position3D = new((Fixed64)2, (Fixed64)3, (Fixed64)4);
        Vector2d position2D = new((Fixed64)5, (Fixed64)6);

        body3D.SetPosition(Vector3d.One);
        body3D.ResetPosition(position3D, FixedQuaternion.Identity);
        body2D.SetPosition(position2D);

        body3D.Position3d.Should().Be(position3D);
        body2D.Position.Should().Be(position2D);
        body3D.MotionType.Should().Be(BodyMotionType.Static);
        body2D.MotionType.Should().Be(BodyMotionType.Static);
        body3D.DynamicId.Should().Be(-1);
        body2D.DynamicId.Should().Be(-1);
        collider3D.Id.Should().Be(-1);
        collider2D.Id.Should().Be(-1);
        context3D.Physics.BodyCount.Should().Be(0);
        context2D.Physics2D.BodyCount.Should().Be(0);
    }

    [Fact]
    public void WarmedMotionTypeTransitionCycles_ShouldAllocateNoManagedMemory()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body3D = scenario.CreateSphere(Vector3d.Zero);
        using GravitasWorldContext context2D = Physics2DTestWorld.CreateContext();
        (SolidBody2D body2D, _) = Create2DBody(context2D, BodyMotionType.Dynamic);

        void TransitionCycle()
        {
            body3D.Body.SetMotionType(BodyMotionType.Static);
            body3D.Body.SetMotionType(BodyMotionType.Kinematic);
            body3D.Body.SetMotionType(BodyMotionType.Dynamic);
            body2D.SetMotionType(BodyMotionType.Static);
            body2D.SetMotionType(BodyMotionType.Kinematic);
            body2D.SetMotionType(BodyMotionType.Dynamic);
        }

        TransitionCycle();

        AllocationTestHelper.MeasureSinglePass(TransitionCycle).Should().Be(0);
    }

    private static (SolidBody Body, LSSphereCollider Collider) Create3DBody(
        GravitasWorldContext context,
        BodyMotionType motionType)
    {
        var collider = new LSSphereCollider();
        var body = new SolidBody(
            new TestMatterAgent(
                context,
                new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)),
            collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity, motionType);
        return (body, collider);
    }

    private static (SolidBody2D Body, LSCircleCollider2D Collider) Create2DBody(
        GravitasWorldContext context,
        BodyMotionType motionType)
    {
        var collider = new LSCircleCollider2D(Fixed64.One);
        var body = new SolidBody2D(
            new TestMatterAgent(
                context,
                new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)),
            collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(Vector2d.Zero, motionType: motionType);
        return (body, collider);
    }
}
