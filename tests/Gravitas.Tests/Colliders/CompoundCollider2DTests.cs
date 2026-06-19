using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class CompoundCollider2DTests
{
    [Fact]
    public void Initialize_ShouldRegisterOnlyOwningColliderAndAggregatePartBounds()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(-Fixed64.One, Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)2, Fixed64.Zero)));

        StiffBody2D body = CreateBody(context, compound, Vector2d.Zero);

        body.Collider.Shape.Should().Be(ColliderType2D.Compound);
        compound.PartCount.Should().Be(2);
        compound.GetPartId(0).Should().Be(0);
        compound.GetPartId(1).Should().Be(1);
        context.Physics2D.ColliderCount.Should().Be(1);
        context.Physics2D.TryGetColliderById(compound.GetPartCollider(0).Id, out _).Should().BeFalse();
        context.Physics2D.TryGetColliderById(compound.GetPartCollider(1).Id, out _).Should().BeFalse();

        compound.Bounds.MinX.Should().Be(-Fixed64.FromFraction(3, 2));
        compound.Bounds.MaxX.Should().Be(Fixed64.FromFraction(5, 2));
        compound.Bounds.MinY.Should().Be(-Fixed64.Half);
        compound.Bounds.MaxY.Should().Be(Fixed64.Half);
        compound.Center.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void Constructor_ShouldRejectDefaultParts()
    {
        Action act = () => _ = new LSCompoundCollider2D(default(CompoundColliderPart2D));

        act.Should().Throw<ArgumentException>()
            .WithMessage("*default*");
    }

    [Fact]
    public void Constructor_ShouldReservePartsForCompoundLifecycleOnly()
    {
        var compound = new LSCompoundCollider2D(CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero));
        LSCollider2D part = compound.GetPartCollider(0);

        Action act = part.Simulate;

        act.Should().Throw<InvalidOperationException>()
            .WithMessage("*standalone lifecycle*");
    }

    [Fact]
    public void Initialize_ShouldApplyOwnerLocalOffsetToAggregateBounds()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(-Fixed64.One, Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)2, Fixed64.Zero)))
        {
            LocalOffset = new Vector2d((Fixed64)5, (Fixed64)(-2))
        };

        _ = CreateBody(context, compound, Vector2d.Zero);

        compound.Bounds.MinX.Should().Be(Fixed64.FromFraction(7, 2));
        compound.Bounds.MaxX.Should().Be(Fixed64.FromFraction(15, 2));
        compound.Bounds.MinY.Should().Be(-Fixed64.FromFraction(5, 2));
        compound.Bounds.MaxY.Should().Be(-Fixed64.FromFraction(3, 2));
        compound.Center.Should().Be(new Vector2d((Fixed64)5, (Fixed64)(-2)));
    }

    [Fact]
    public void PartShapeMutation_ShouldRefreshAggregateBounds()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero));
        _ = CreateBody(context, compound, Vector2d.Zero);
        var part = (LSCircleCollider2D)compound.GetPartCollider(0);

        part.Radius = Fixed64.One;
        compound.Simulate();

        compound.Bounds.MinX.Should().Be(-Fixed64.One);
        compound.Bounds.MaxX.Should().Be(Fixed64.One);
        compound.Bounds.MinY.Should().Be(-Fixed64.One);
        compound.Bounds.MaxY.Should().Be(Fixed64.One);
        compound.RuntimeShapeVersion.Should().BeGreaterThan(1u);
    }

    private static StiffBody2D CreateBody(GravitasWorldContext context, LSCollider2D collider, Vector2d position)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new StiffBody2D(agent, collider)
        {
            Mass = Fixed64.One,
            Immovable = true
        };
        body.Initialize(position);
        return body;
    }
}
