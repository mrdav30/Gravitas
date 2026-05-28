using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class StiffBodyDimensionTests
{
    [Fact]
    public void StiffBodyAndCurrentPrimitiveColliders_ShouldDefaultToThreeDimensions()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        body.Body.Dimension.Should().Be(PhysicsDimension.ThreeD);
        body.Collider.Dimension.Should().Be(PhysicsDimension.ThreeD);
        new LSCapsuleCollider().Dimension.Should().Be(PhysicsDimension.ThreeD);
        new LSCuboidCollider().Dimension.Should().Be(PhysicsDimension.ThreeD);
        new LSCylinderCollider().Dimension.Should().Be(PhysicsDimension.ThreeD);
    }

    [Fact]
    public void Initialize_WithMismatchedBodyAndColliderDimensions_ShouldThrow()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var agent = new TestMatterAgent(context);
        var collider = new LSSphereCollider();
        var body = new StiffBody(agent, collider)
        {
            Dimension = PhysicsDimension.TwoD,
            Mass = Fixed64.One
        };

        Action initialize = () => body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);

        initialize.Should()
            .Throw<ArgumentException>()
            .WithMessage("*same simulation dimension*");
    }

    [Fact]
    public void Dimension_SetAfterInitialize_ShouldThrow()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        Action changeDimension = () => body.Body.Dimension = PhysicsDimension.TwoD;

        changeDimension.Should()
            .Throw<InvalidOperationException>()
            .WithMessage("*cannot change after initialization*");
    }

    [Fact]
    public void Dimension_WithUnsupportedValue_ShouldThrow()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var agent = new TestMatterAgent(context);
        var collider = new LSSphereCollider();
        var body = new StiffBody(agent, collider);

        Action setInvalidDimension = () => body.Dimension = (PhysicsDimension)255;

        setInvalidDimension.Should()
            .Throw<ArgumentException>()
            .WithMessage("*Unsupported physics dimension*");
    }
}
