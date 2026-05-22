using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class MatterAgentContextTests
{
    [Fact]
    public void StiffBodyInitialize_WithContextBoundAgent_ShouldRegisterWithAgentContext()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var agent = new TestMatterAgent(context);
        var collider = new LSSphereCollider();
        var body = new StiffBody(agent, collider)
        {
            Mass = Fixed64.One
        };

        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);

        body.Context.Should().BeSameAs(context);
        body.World.Should().BeSameAs(context.World);
        collider.Context.Should().BeSameAs(context);
        collider.World.Should().BeSameAs(context.World);
        context.Physics.AssimilatedBodyCount.Should().Be(1);
        context.Physics.AssimilatedColliderCount.Should().Be(1);
    }

    [Fact]
    public void InitializeWithNoBody_WithContextBoundAgent_ShouldRegisterStaticColliderWithAgentContext()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var agent = new TestMatterAgent(context);
        var collider = new LSSphereCollider();

        collider.InitializeWithNoBody(agent);

        collider.Context.Should().BeSameAs(context);
        collider.World.Should().BeSameAs(context.World);
        collider.Id.Should().Be(1);
        context.Physics.AssimilatedColliderCount.Should().Be(1);
        context.Physics.TryGetColliderById(collider.Id, out LSCollider? resolved).Should().BeTrue();
        resolved.Should().BeSameAs(collider);
    }

    [Fact]
    public void StiffBodySetup_WithColliderBoundToDifferentContext_ShouldThrowClearException()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();
        var agentA = new TestMatterAgent(contextA);
        var agentB = new TestMatterAgent(contextB);
        var collider = new LSSphereCollider();
        collider.InitializeWithNoBody(agentA);

        Action createBodyWithCrossContextCollider = () => _ = new StiffBody(agentB, collider);

        createBodyWithCrossContextCollider.Should()
            .Throw<ArgumentException>()
            .WithMessage("*same context*");
    }
}
