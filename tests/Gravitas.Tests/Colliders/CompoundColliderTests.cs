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
            new CompoundColliderPart(new LSSphereCollider { LocalOffset = new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero) }),
            new CompoundColliderPart(new LSSphereCollider { LocalOffset = new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero) }));

        ScenarioBody<LSCompoundCollider> body = scenario.CreateBody(
            compound,
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity);

        body.Collider.Shape.Should().Be(ColliderType.Compound);
        body.Collider.PartCount.Should().Be(2);
        body.Collider.GetPartId(0).Should().Be(0);
        body.Collider.GetPartId(1).Should().Be(1);
        scenario.Context.Physics.AssimilatedColliderCount.Should().Be(1);
        scenario.Context.Physics.TryGetColliderById(body.Collider.GetPartCollider(0).Id, out _).Should().BeFalse();
        scenario.Context.Physics.TryGetColliderById(body.Collider.GetPartCollider(1).Id, out _).Should().BeFalse();

        body.Collider.BoundsMin.Should().Be(new Vector3d(-Fixed64.Fraction(3, 2), -Fixed64.Half, -Fixed64.Half));
        body.Collider.BoundsMax.Should().Be(new Vector3d(Fixed64.Fraction(5, 2), Fixed64.Half, Fixed64.Half));
        body.Collider.Center.Should().Be(PhysicsScenarioBuilder.Vector(0, 0, 0));
    }

    [Fact]
    public void Constructor_ShouldRejectConcaveMeshParts()
    {
        Action act = () => _ = new LSCompoundCollider(
            new CompoundColliderPart(MeshTestFixtures.CreateInsideCorner(MeshColliderMode.Concave)));

        act.Should().Throw<ArgumentException>()
            .WithMessage("*Concave*");
    }

    [Fact]
    public void Constructor_ShouldRejectNestedCompoundParts()
    {
        var nested = new LSCompoundCollider(new CompoundColliderPart(new LSSphereCollider()));

        Action act = () => _ = new LSCompoundCollider(new CompoundColliderPart(nested));

        act.Should().Throw<ArgumentException>()
            .WithMessage("*Compound*");
    }

    [Fact]
    public void Constructor_ShouldRejectStandaloneInitializedParts()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> initialized = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));

        Action act = () => _ = new LSCompoundCollider(new CompoundColliderPart(initialized.Collider));

        act.Should().Throw<ArgumentException>()
            .WithMessage("*initialized*");
    }

    [Fact]
    public void Constructor_ShouldReservePartsForCompoundLifecycleOnly()
    {
        var part = new LSSphereCollider();
        _ = new LSCompoundCollider(new CompoundColliderPart(part));

        Action act = part.Simulate;

        act.Should().Throw<InvalidOperationException>()
            .WithMessage("*standalone lifecycle*");
    }

    [Fact]
    public void PartShapeMutation_ShouldRefreshAggregateBounds()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var part = new LSSphereCollider();
        var compound = new LSCompoundCollider(new CompoundColliderPart(part));
        ScenarioBody<LSCompoundCollider> body = scenario.CreateBody(
            compound,
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity);

        part.Radius = Fixed64.One;
        body.Collider.Simulate();

        body.Collider.BoundsMin.Should().Be(new Vector3d(-Fixed64.One, -Fixed64.One, -Fixed64.One));
        body.Collider.BoundsMax.Should().Be(new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One));
        body.Collider.RuntimeShapeVersion.Should().BeGreaterThan(1u);
    }
}
