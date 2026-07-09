using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class BodyFreezeConstraintTests
{
    [Fact]
    public void SolidBody_DefaultFreezeAxes_ShouldAllowTranslationAndRotation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        body.Body.FreezeAxes.Should().Be(BodyFreezeAxes3D.None);
        body.Body.IsPositionFullyFrozen.Should().BeFalse();
        body.Body.AngularMotionFrozen.Should().BeFalse();
        body.Body.CanTranslate.Should().BeTrue();
        body.Body.CanRotate.Should().BeTrue();
    }

    [Fact]
    public void SolidBody_PositionYFreeze_ShouldProjectForcesAndCollisionMotion()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.FreezeAxes = BodyFreezeAxes3D.PositionY;

        body.Body.AddForce(new Vector3d((Fixed64)8, (Fixed64)8, (Fixed64)8));
        scenario.Context.LateSimulate();
        body.Body.ApplyCollisionLinearVelocityDelta(new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One));
        body.Body.ApplyCollisionPositionCorrection(new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One));

        body.Body.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        body.Body.LinearVelocity.Y.Should().Be(Fixed64.Zero);
        body.Body.LinearVelocity.Z.Should().BeGreaterThan(Fixed64.Zero);
        body.Body.Position3d.X.Should().BeGreaterThan(Fixed64.Zero);
        body.Body.Position3d.Y.Should().Be(Fixed64.Zero);
        body.Body.Position3d.Z.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void SolidBody_RotationYFreeze_ShouldProjectAngularForces()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> body = scenario.CreateCuboid(Vector3d.Zero);
        body.Body.FreezeAxes = BodyFreezeAxes3D.RotationY;

        body.Body.AddAngularImpulse(new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One));
        body.Body.ApplyCollisionAngularVelocityDelta(new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One));

        body.Body.AngularVelocity.X.Should().NotBe(Fixed64.Zero);
        body.Body.AngularVelocity.Y.Should().Be(Fixed64.Zero);
        body.Body.AngularVelocity.Z.Should().NotBe(Fixed64.Zero);
    }

    [Fact]
    public void SolidBody_InternalProjectionHelpers_ShouldRespectFullAndPerAxisFreeze()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> body = scenario.CreateCuboid(Vector3d.Zero);
        Vector3d motion = new(Fixed64.One, (Fixed64)2, (Fixed64)3);

        body.Body.ProjectLinearMotion(Vector3d.Zero).Should().Be(Vector3d.Zero);
        body.Body.ProjectAngularMotion(Vector3d.Zero).Should().Be(Vector3d.Zero);

        body.Body.FreezeAxes = BodyFreezeAxes3D.Position;
        body.Body.ProjectLinearMotion(motion).Should().Be(Vector3d.Zero);
        body.Body.ProjectAngularMotion(motion).Should().Be(Vector3d.Zero);

        body.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        body.Body.ProjectLinearMotion(motion).Should().Be(motion);
        body.Body.ProjectAngularMotion(motion).Should().Be(Vector3d.Zero);

        body.Body.FreezeAxes = BodyFreezeAxes3D.PositionX | BodyFreezeAxes3D.RotationZ;
        body.Body.ProjectLinearMotion(motion).Should().Be(new Vector3d(Fixed64.Zero, (Fixed64)2, (Fixed64)3));
        body.Body.ProjectAngularMotion(motion).Should().Be(new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.Zero));
    }

    [Fact]
    public void SolidBody_ConstrainedMassAndInertia_ShouldReflectAxisFreeze()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> body = scenario.CreateCuboid(Vector3d.Zero);

        body.Body.GetConstrainedInverseMass(Vector3d.Zero).Should().Be(Fixed64.Zero);
        body.Body.ApplyConstrainedInverseInertia(Vector3d.Zero).Should().Be(Vector3d.Zero);

        body.Body.FreezeAxes = BodyFreezeAxes3D.PositionX;

        body.Body.GetConstrainedInverseMass(Vector3d.Right).Should().Be(Fixed64.Zero);
        body.Body.GetConstrainedInverseMass(Vector3d.Up).Should().Be(body.Body.InverseMass);
        body.Body.GetConstrainedInverseMass(Vector3d.Right + Vector3d.Up).Should().Be(body.Body.InverseMass / (Fixed64)2);

        body.Body.FreezeAxes = BodyFreezeAxes3D.RotationY;
        body.Body.ApplyConstrainedInverseInertia(Vector3d.Up).Should().Be(Vector3d.Zero);
        body.Body.ApplyConstrainedInverseInertia(Vector3d.Right).Should().NotBe(Vector3d.Zero);

        body.Body.FreezeAxes = BodyFreezeAxes3D.Position;
        body.Body.GetConstrainedInverseMass(Vector3d.Up).Should().Be(Fixed64.Zero);
        body.Body.ApplyConstrainedInverseInertia(Vector3d.Right).Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void CollisionResponse_WithPositionYFreeze_ShouldTreatFrozenAxisAsInfiniteMass()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> frozen = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> moving = scenario.CreateSphere(new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Zero));
        frozen.Body.FreezeAxes = BodyFreezeAxes3D.PositionY;
        moving.Body.ApplyCollisionLinearVelocityDelta(-Vector3d.Up * (Fixed64)4);
        CollisionPair pair = scenario.CreatePair(frozen.Collider, moving.Collider);
        pair.Manifold.SetContact(
            frozen.Collider.Center,
            moving.Collider.Center,
            Fixed64.FromFraction(1, 10),
            Vector3d.Up);

        CollisionResponse.CalculateImpulse(pair);

        frozen.Body.LinearVelocity.Y.Should().Be(Fixed64.Zero);
        frozen.Body.Position3d.Y.Should().Be(Fixed64.Zero);
        moving.Body.LinearVelocity.Y.Should().BeGreaterThan(-(Fixed64)4);
    }

    [Fact]
    public void SolidBody2D_DefaultFreezeAxes_ShouldAllowTranslationAndRotation()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D body = Create2DBody(context);

        body.FreezeAxes.Should().Be(BodyFreezeAxes2D.None);
        body.IsPositionFullyFrozen.Should().BeFalse();
        body.AngularMotionFrozen.Should().BeFalse();
        body.CanTranslate.Should().BeTrue();
        body.CanRotate.Should().BeTrue();
    }

    [Fact]
    public void SolidBody2D_PositionYFreeze_ShouldProjectPlanarForcesAndCollisionMotion()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D body = Create2DBody(context);
        body.FreezeAxes = BodyFreezeAxes2D.PositionY;

        body.AddForce(new Vector2d((Fixed64)8, (Fixed64)8));
        context.LateSimulate();
        body.ApplyCollisionLinearVelocityDelta(new Vector2d(Fixed64.One, Fixed64.One));
        body.ApplyCollisionPositionCorrection(new Vector2d(Fixed64.One, Fixed64.One));

        body.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        body.LinearVelocity.Y.Should().Be(Fixed64.Zero);
        body.Position.X.Should().BeGreaterThan(Fixed64.Zero);
        body.Position.Y.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void SolidBody2D_RotationFreeze_ShouldBlockYawImpulse()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D body = Create2DBody(context);
        body.FreezeAxes = BodyFreezeAxes2D.Rotation;

        body.AddAngularImpulse(Fixed64.One);
        body.ApplyCollisionAngularVelocityDelta(Fixed64.One);

        body.AngularVelocity.Should().Be(Fixed64.Zero);
        body.CanRotate.Should().BeFalse();
        body.AngularMotionFrozen.Should().BeTrue();
    }

    [Fact]
    public void CollisionResponse2D_WithPositionYFreeze_ShouldTreatFrozenPlanarAxisAsInfiniteMass()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D frozen = Create2DBody(context);
        SolidBody2D moving = Create2DBody(context, new Vector2d(Fixed64.Zero, Fixed64.Half));
        frozen.FreezeAxes = BodyFreezeAxes2D.PositionY;
        moving.ApplyCollisionLinearVelocityDelta(new Vector2d(Fixed64.Zero, -(Fixed64)4));
        var pair = new CollisionPair2D(frozen.Collider, moving.Collider);
        pair.Manifold.SetContact(
            frozen.Position,
            moving.Position,
            Fixed64.FromFraction(1, 10),
            new Vector2d(Fixed64.Zero, Fixed64.One));

        pair.MarkColliding(context.FrameCount);

        frozen.LinearVelocity.Y.Should().Be(Fixed64.Zero);
        frozen.Position.Y.Should().Be(Fixed64.Zero);
        moving.LinearVelocity.Y.Should().BeGreaterThan(-(Fixed64)4);
    }

    [Fact]
    public void CollisionResponseMixed_With2DPositionXFreeze_ShouldNotMoveFrozenPlanarAxis()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = Create3DSphere(context, new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        SolidBody2D body2D = Create2DBody(context);
        body2D.FreezeAxes = BodyFreezeAxes2D.PositionX;
        body3D.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)4);
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        var contact = new MixedContact(
            body3D.Collider.Center,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right,
            Fixed64.FromFraction(1, 10));

        CollisionResponseMixed.Resolve(pair, contact);

        body2D.Position.X.Should().Be(Fixed64.Zero);
        body2D.LinearVelocity.X.Should().Be(Fixed64.Zero);
        body3D.Body.LinearVelocity.X.Should().BeLessThan((Fixed64)4);
    }

    private static SolidBody2D Create2DBody(GravitasWorldContext context) => Create2DBody(context, Vector2d.Zero);

    private static SolidBody2D Create2DBody(GravitasWorldContext context, Vector2d position)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var body = new SolidBody2D(
            new TestMatterAgent(context, transform),
            new LSCircleCollider2D(Fixed64.One))
        {
            Mass = Fixed64.One
        };
        body.Initialize(position);
        return body;
    }

    private static GravitasWorldContext CreateMixedContext()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.Environment.Gravity = Fixed64.Zero;
        return context;
    }

    private static ScenarioBody<LSSphereCollider> Create3DSphere(GravitasWorldContext context, Vector3d position)
    {
        var collider = new LSSphereCollider();
        var transform = new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One);
        var body = new SolidBody(new TestMatterAgent(context, transform), collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(position, FixedQuaternion.Identity);
        return new ScenarioBody<LSSphereCollider>(body, collider);
    }
}
