using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using SwiftCollections;
using System;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class MatterAgentContextTests
{
    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);

    [Fact]
    public void SolidBodyInitialize_WithContextBoundAgent_ShouldRegisterWithAgentContext()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var agent = new TestMatterAgent(context);
        var collider = new LSSphereCollider();
        var body = new SolidBody(agent, collider)
        {
            Mass = Fixed64.One
        };

        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);

        body.Context.Should().BeSameAs(context);
        body.World.Should().BeSameAs(context.World);
        collider.Context.Should().BeSameAs(context);
        collider.World.Should().BeSameAs(context.World);
        context.Physics.BodyCount.Should().Be(1);
        context.Physics.ColliderCount.Should().Be(1);
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
        collider.Id.Should().Be(0);
        context.Physics.ColliderCount.Should().Be(1);
        context.Physics.TryGetColliderById(collider.Id, out LSCollider? resolved).Should().BeTrue();
        resolved.Should().BeSameAs(collider);
    }

    [Fact]
    public void LateSimulate_WithMovedBodylessCollider_ShouldRefreshBoundsAndPartitionsFromAgentTransform()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        EnsureGrid(context);
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var collider = new LSSphereCollider();
        var hits = new SwiftList<Physics3DHit>();

        collider.InitializeWithNoBody(agent);

        transform.LocalPosition = new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero);
        context.Simulate();
        context.LateSimulate();

        context.Query3D.RaycastAll(
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero),
            IncludeLayerZero,
            hits).Should().Be(1);
        hits[0].Collider.Should().BeSameAs(collider);

        context.Query3D.RaycastAll(
            new Vector3d((Fixed64)(-1), Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero),
            IncludeLayerZero,
            hits).Should().Be(0);
    }

    [Fact]
    public void SolidBodySetup_WithColliderBoundToDifferentContext_ShouldThrowClearException()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();
        var agentA = new TestMatterAgent(contextA);
        var agentB = new TestMatterAgent(contextB);
        var collider = new LSSphereCollider();
        collider.InitializeWithNoBody(agentA);

        Action createBodyWithCrossContextCollider = () => _ = new SolidBody(agentB, collider);

        createBodyWithCrossContextCollider.Should()
            .Throw<ArgumentException>()
            .WithMessage("*same context*");
    }

    private static void EnsureGrid(GravitasWorldContext context)
    {
        if (context.World.ActiveGrids.Count > 0)
            return;

        GridConfiguration configuration = new(
            new Vector3d((Fixed64)(-4), (Fixed64)(-4), (Fixed64)(-4)),
            new Vector3d((Fixed64)8, (Fixed64)8, (Fixed64)8));

        context.World.TryAddGrid(configuration, out _).Should().BeTrue();
    }
}
