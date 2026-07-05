using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class CompoundColliderTests
{
    [Fact]
    public void Initialize_ShouldRegisterOnlyOwningColliderAndAggregatePartBounds()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero)),
            CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero)));

        ScenarioBody<LSCompoundCollider> body = scenario.CreateBody(
            compound,
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity);

        body.Collider.Shape.Should().Be(ColliderType.Compound);
        body.Collider.PartCount.Should().Be(2);
        body.Collider.GetPartId(0).Should().Be(0);
        body.Collider.GetPartId(1).Should().Be(1);
        scenario.Context.Physics.ColliderCount.Should().Be(1);
        scenario.Context.Physics.TryGetColliderById(body.Collider.GetPartCollider(0).Id, out _).Should().BeFalse();
        scenario.Context.Physics.TryGetColliderById(body.Collider.GetPartCollider(1).Id, out _).Should().BeFalse();

        body.Collider.BoundsMin.Should().Be(new Vector3d(-Fixed64.FromFraction(3, 2), -Fixed64.Half, -Fixed64.Half));
        body.Collider.BoundsMax.Should().Be(new Vector3d(Fixed64.FromFraction(5, 2), Fixed64.Half, Fixed64.Half));
        body.Collider.Center.Should().Be(PhysicsScenarioBuilder.Vector(0, 0, 0));
    }

    [Fact]
    public void Constructor_ShouldRejectDefaultParts()
    {
        Action act = () => _ = new LSCompoundCollider(default(CompoundColliderPart));

        act.Should().Throw<ArgumentException>()
            .WithMessage("*default*");
    }

    [Fact]
    public void Constructor_ShouldReservePartsForCompoundLifecycleOnly()
    {
        var compound = new LSCompoundCollider(CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero));
        LSCollider part = compound.GetPartCollider(0);

        Action act = part.Simulate;

        act.Should().Throw<InvalidOperationException>()
            .WithMessage("*standalone lifecycle*");
    }

    [Fact]
    public void PartShapeMutation_ShouldRefreshAggregateBounds()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var compound = new LSCompoundCollider(CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero));
        ScenarioBody<LSCompoundCollider> body = scenario.CreateBody(
            compound,
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity);
        var part = (LSSphereCollider)body.Collider.GetPartCollider(0);

        part.Radius = Fixed64.One;
        body.Collider.Simulate();

        body.Collider.BoundsMin.Should().Be(new Vector3d(-Fixed64.One, -Fixed64.One, -Fixed64.One));
        body.Collider.BoundsMax.Should().Be(new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One));
        body.Collider.RuntimeShapeVersion.Should().BeGreaterThan(1u);
    }
}
