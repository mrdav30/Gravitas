using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Support;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Response;

public sealed class CollisionResponse2DManifoldTests
{
    [Fact]
    public void Resolve_WithSymmetricFaceContacts_ShouldNotIntroduceAngularVelocity()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        StiffBody2D moving = CreateBox(context, Vector2d.Zero);
        StiffBody2D wall = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        moving.ApplyCollisionLinearVelocityDelta(new Vector2d((Fixed64)4, Fixed64.Zero));
        AddFaceContacts(pair, depth: Fixed64.Half);

        pair.MarkColliding(context.FrameCount);

        moving.LinearVelocity.X.Should().BeLessThan((Fixed64)4);
        moving.AngularVelocity.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithOffCenterSingleContact_ShouldStillIntroduceAngularVelocity()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        StiffBody2D moving = CreateBox(context, Vector2d.Zero);
        StiffBody2D wall = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        moving.ApplyCollisionLinearVelocityDelta(new Vector2d((Fixed64)4, Fixed64.Zero));
        pair.Manifold.SetContact(
            new Vector2d(Fixed64.One, Fixed64.One),
            new Vector2d(Fixed64.One, Fixed64.One),
            Fixed64.Half,
            Vector2d.Right);

        pair.MarkColliding(context.FrameCount);

        moving.AngularVelocity.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithTwoContacts_ShouldApplyPositionCorrectionOnceForPair()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        StiffBody2D moving = CreateBox(context, Vector2d.Zero);
        StiffBody2D wall = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        AddFaceContacts(pair, depth: Fixed64.Half);
        Fixed64 expectedCorrection = Fixed64.Half - CollisionResponse2D.PenetrationSlop;

        pair.MarkColliding(context.FrameCount);

        moving.Position.Should().Be(new Vector2d(-expectedCorrection, Fixed64.Zero));
    }

    [Fact]
    public void Resolve_WithTwoFrictionContacts_ShouldOpposeTangentialMotionAndCacheBothContacts()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        StiffBody2D moving = CreateBox(context, Vector2d.Zero);
        StiffBody2D wall = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        moving.ApplyCollisionLinearVelocityDelta(new Vector2d((Fixed64)4, (Fixed64)20));
        Fixed64 tangentialSpeed = moving.LinearVelocity.Y.Abs();
        AddFaceContacts(pair, depth: Fixed64.Half);

        pair.MarkColliding(context.FrameCount);

        moving.LinearVelocity.Y.Abs().Should().BeLessThan(tangentialSpeed);
        pair.TryGetWarmStartImpulse(pair.Manifold[0].ContactId, out ContactWarmStartImpulse first).Should().BeTrue();
        pair.TryGetWarmStartImpulse(pair.Manifold[1].ContactId, out ContactWarmStartImpulse second).Should().BeTrue();
        first.TangentImpulse.Abs().Should().BeGreaterThan(Fixed64.Zero);
        second.TangentImpulse.Abs().Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void ResponseBody2D_Create_ShouldUseEffectiveMassAndScalarMoment()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        StiffBody2D movable = CreateBox(context, Vector2d.Zero);
        ResponseBody2D movableBody = ResponseBody2D.Create(movable.Collider);
        movableBody.CanTranslate.Should().BeTrue();
        movableBody.CanRotate.Should().BeTrue();
        movableBody.InverseMass.Should().BeGreaterThan(Fixed64.Zero);
        movableBody.InverseMoment.Should().BeGreaterThan(Fixed64.Zero);

        movable.PreventAngularForces = true;
        ResponseBody2D angularDisabled = ResponseBody2D.Create(movable.Collider);
        angularDisabled.CanTranslate.Should().BeTrue();
        angularDisabled.CanRotate.Should().BeFalse();
        angularDisabled.InverseMoment.Should().Be(Fixed64.Zero);

        StiffBody2D immovable = CreateBox(context, new Vector2d((Fixed64)4, Fixed64.Zero), immovable: true);
        ResponseBody2D immovableBody = ResponseBody2D.Create(immovable.Collider);
        immovableBody.CanTranslate.Should().BeFalse();
        immovableBody.CanRotate.Should().BeFalse();
        immovableBody.InverseMass.Should().Be(Fixed64.Zero);

        StiffBody2D kinematic = CreateBox(context, new Vector2d((Fixed64)8, Fixed64.Zero), isKinematic: true);
        ResponseBody2D kinematicBody = ResponseBody2D.Create(kinematic.Collider);
        kinematicBody.CanTranslate.Should().BeFalse();
        kinematicBody.CanRotate.Should().BeFalse();

        StiffBody2D zeroMass = CreateBox(context, new Vector2d((Fixed64)12, Fixed64.Zero));
        zeroMass.Mass = Fixed64.Zero;
        ResponseBody2D zeroMassBody = ResponseBody2D.Create(zeroMass.Collider);
        zeroMassBody.CanTranslate.Should().BeFalse();
        zeroMassBody.CanRotate.Should().BeFalse();
        zeroMassBody.InverseMass.Should().Be(Fixed64.Zero);

        StiffBody2D inactive = CreateBox(context, new Vector2d((Fixed64)16, Fixed64.Zero));
        inactive.Deactivate();
        ResponseBody2D inactiveBody = ResponseBody2D.Create(inactive.Collider);
        inactiveBody.CanTranslate.Should().BeFalse();
        inactiveBody.CanRotate.Should().BeFalse();
        inactiveBody.InverseMass.Should().Be(Fixed64.Zero);
        inactiveBody.InverseMoment.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithStaleCachedImpulse_ShouldClampWarmStartCacheAfterFreshSolve()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        StiffBody2D moving = CreateBox(context, Vector2d.Zero);
        StiffBody2D wall = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        pair.Manifold.SetContact(Vector2d.Right, Vector2d.Right, Fixed64.Half, Vector2d.Right);
        ulong contactId = pair.Manifold.PrimaryContact.ContactId;
        pair.StoreWarmStartImpulse(contactId, Fixed64.One, Fixed64.One);

        pair.MarkColliding(context.FrameCount);

        moving.LinearVelocity.Should().Be(Vector2d.Zero);
        moving.AngularVelocity.Should().Be(Fixed64.Zero);
        pair.TryGetWarmStartImpulse(contactId, out ContactWarmStartImpulse impulse).Should().BeTrue();
        impulse.NormalImpulse.Should().Be(Fixed64.Zero);
        impulse.TangentImpulse.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithClosingVelocity_ShouldRefreshWarmStartCache()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        StiffBody2D moving = CreateBox(context, Vector2d.Zero);
        StiffBody2D wall = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        moving.ApplyCollisionLinearVelocityDelta(new Vector2d((Fixed64)4, Fixed64.Zero));
        pair.Manifold.SetContact(Vector2d.Right, Vector2d.Right, Fixed64.Half, Vector2d.Right);
        ulong contactId = pair.Manifold.PrimaryContact.ContactId;

        pair.MarkColliding(context.FrameCount);

        pair.TryGetWarmStartImpulse(contactId, out ContactWarmStartImpulse impulse).Should().BeTrue();
        impulse.NormalImpulse.Should().BeGreaterThan(Fixed64.Zero);
    }

    private static void AddFaceContacts(CollisionPair2D pair, Fixed64 depth)
    {
        pair.Manifold.SetContact(
            new Vector2d(Fixed64.One, -Fixed64.One),
            new Vector2d(Fixed64.One, -Fixed64.One),
            depth,
            Vector2d.Right);
        pair.Manifold.AddContact(
            new Vector2d(Fixed64.One, Fixed64.One),
            new Vector2d(Fixed64.One, Fixed64.One),
            depth,
            Vector2d.Right);
    }

    private static StiffBody2D CreateBox(
        GravitasWorldContext context,
        Vector2d position,
        bool immovable = false,
        bool isKinematic = false)
    {
        var body = new StiffBody2D(
            new TestMatterAgent(context, new FixedTransform(
                new Vector3d(position.X, Fixed64.Zero, position.Y),
                FixedQuaternion.Identity,
                Vector3d.One)),
            new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)))
        {
            Mass = Fixed64.One,
            Immovable = immovable,
            IsKinematic = isKinematic
        };
        body.Initialize(position);
        return body;
    }
}
