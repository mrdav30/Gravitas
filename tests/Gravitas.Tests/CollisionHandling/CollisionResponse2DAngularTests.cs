using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Response;

public sealed class CollisionResponse2DAngularTests
{
    [Fact]
    public void OffCenterCollision_ShouldApplyAngularVelocityDelta()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D moving = CreateBox(context, Vector2d.Zero);
        SolidBody2D wall = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        moving.ApplyCollisionLinearVelocityDelta(new Vector2d((Fixed64)4, Fixed64.Zero));

        MarkColliding(
            pair,
            context.FrameCount,
            new Vector2d(Fixed64.Zero, Fixed64.One),
            new Vector2d((Fixed64)2, Fixed64.One),
            Vector2d.Right,
            Fixed64.Half);

        moving.AngularVelocity.Should().BeGreaterThan(Fixed64.Zero);
        wall.AngularVelocity.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void CenteredCollision_ShouldNotIntroduceAngularVelocity()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D moving = CreateBox(context, Vector2d.Zero);
        SolidBody2D wall = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        moving.ApplyCollisionLinearVelocityDelta(new Vector2d((Fixed64)4, Fixed64.Zero));

        MarkColliding(
            pair,
            context.FrameCount,
            Vector2d.Zero,
            new Vector2d((Fixed64)2, Fixed64.Zero),
            Vector2d.Right,
            Fixed64.Half);

        moving.AngularVelocity.Should().Be(Fixed64.Zero);
        wall.AngularVelocity.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void AngularDisabledBody_ShouldKeepLinearResponseAndRejectAngularVelocityDelta()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D moving = CreateBox(context, Vector2d.Zero);
        SolidBody2D wall = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        moving.FreezeAxes = BodyFreezeAxes2D.Rotation;
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        moving.ApplyCollisionLinearVelocityDelta(new Vector2d((Fixed64)4, Fixed64.Zero));

        MarkColliding(
            pair,
            context.FrameCount,
            new Vector2d(Fixed64.Zero, Fixed64.One),
            new Vector2d((Fixed64)2, Fixed64.One),
            Vector2d.Right,
            Fixed64.Half);

        moving.LinearVelocity.X.Should().BeLessThan((Fixed64)4);
        moving.AngularVelocity.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [InlineData(true, false)]
    [InlineData(false, true)]
    public void ConstrainedBody_ShouldRejectAngularVelocityDelta(bool immovable, bool isKinematic)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D constrained = CreateBox(
            context,
            Vector2d.Zero,
            immovable: immovable,
            isKinematic: isKinematic);
        SolidBody2D moving = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero));
        var pair = new CollisionPair2D(constrained.Collider, moving.Collider);
        moving.ApplyCollisionLinearVelocityDelta(new Vector2d((Fixed64)(-4), Fixed64.Zero));

        MarkColliding(
            pair,
            context.FrameCount,
            new Vector2d(Fixed64.Zero, Fixed64.One),
            new Vector2d((Fixed64)2, Fixed64.One),
            Vector2d.Right,
            Fixed64.Half);

        constrained.LinearVelocity.Should().Be(Vector2d.Zero);
        constrained.AngularVelocity.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void TangentialContactVelocity_ShouldApplyFrictionAndAngularVelocityDelta()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D moving = CreateBox(context, Vector2d.Zero);
        SolidBody2D wall = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        moving.ApplyCollisionLinearVelocityDelta(new Vector2d((Fixed64)4, (Fixed64)2));
        Fixed64 tangentialSpeedBefore = moving.LinearVelocity.Y.Abs();

        MarkColliding(
            pair,
            context.FrameCount,
            Vector2d.Right,
            new Vector2d((Fixed64)2, Fixed64.Zero),
            Vector2d.Right,
            Fixed64.Half);

        moving.LinearVelocity.Y.Abs().Should().BeLessThan(tangentialSpeedBefore);
        moving.AngularVelocity.Should().NotBe(Fixed64.Zero);
    }

    private static SolidBody2D CreateBox(
        GravitasWorldContext context,
        Vector2d position,
        bool immovable = false,
        bool isKinematic = false)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var body = new SolidBody2D(
            new TestMatterAgent(context, transform),
            new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)))
        {
            Mass = Fixed64.One,
            FreezeAxes = immovable ? BodyFreezeAxes2D.Position : BodyFreezeAxes2D.None,
            IsKinematic = isKinematic
        };
        body.Initialize(position);
        return body;
    }

    private static void MarkColliding(
        CollisionPair2D pair,
        int frame,
        Vector2d pointA,
        Vector2d pointB,
        Vector2d normal,
        Fixed64 depth)
    {
        pair.Manifold.SetContact(pointA, pointB, depth, normal);
        pair.MarkColliding(frame);
    }
}
