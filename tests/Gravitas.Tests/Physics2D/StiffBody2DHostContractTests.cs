using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class StiffBody2DHostContractTests
{
    [Fact]
    public void Constructor_ShouldBindAgentContextAndCollider()
    {
        using GravitasWorldContext context = Create2DContext();
        var transform = new FixedTransform(new Vector3d((Fixed64)2, (Fixed64)7, (Fixed64)3), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var collider = new LSCircleCollider2D(Fixed64.One);

        var body = new StiffBody2D(agent, collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(transform.Position.ToVector2d());

        body.Agent.Should().BeSameAs(agent);
        body.Context.Should().BeSameAs(context);
        body.Collider.Should().BeSameAs(collider);
        body.Position.Should().Be(new Vector2d((Fixed64)2, (Fixed64)3));
        collider.Agent.Should().BeSameAs(agent);
        collider.Context.Should().BeSameAs(context);
    }

    [Fact]
    public void LateSimulate_WithKinematicBody_ShouldProjectHostTransformXZIntoPure2DPosition()
    {
        using GravitasWorldContext context = Create2DContext();
        var transform = new FixedTransform(new Vector3d(Fixed64.One, (Fixed64)9, (Fixed64)2), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new StiffBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            IsKinematic = true,
            Mass = Fixed64.One
        };
        body.Initialize(transform.Position.ToVector2d());

        transform.Position = new Vector3d((Fixed64)5, (Fixed64)11, (Fixed64)7);
        context.LateSimulate();

        body.Position.Should().Be(new Vector2d((Fixed64)5, (Fixed64)7));
        body.Collider.Center.Should().Be(new Vector2d((Fixed64)5, (Fixed64)7));
        var hits = new SwiftList<Physics2DHit>();
        context.Physics2D.OverlapCircleAll(new Vector2d((Fixed64)5, (Fixed64)7), Fixed64.Half, hits).Should().Be(1);
    }

    [Fact]
    public void InitializeWithNoBody_ShouldBindStaticColliderToAgentAndQueries()
    {
        using GravitasWorldContext context = Create2DContext();
        var transform = new FixedTransform(new Vector3d((Fixed64)4, (Fixed64)8, (Fixed64)6), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var collider = new LSCircleCollider2D(Fixed64.Half);

        collider.InitializeWithNoBody(agent);

        collider.Agent.Should().BeSameAs(agent);
        collider.Body.Should().BeNull();
        collider.Center.Should().Be(new Vector2d((Fixed64)4, (Fixed64)6));
        context.Physics2D.ColliderCount.Should().Be(1);
        var hits = new SwiftList<Physics2DHit>();
        context.Physics2D.OverlapCircleAll(new Vector2d((Fixed64)4, (Fixed64)6), Fixed64.Half, hits).Should().Be(1);
    }

    [Fact]
    public void Simulate_WithBodylessStaticCollider_ShouldResolveDynamicBody()
    {
        using GravitasWorldContext context = Create2DContext();
        var staticTransform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        var staticAgent = new TestMatterAgent(context, staticTransform);
        var staticCollider = new LSCircleCollider2D(Fixed64.Half);
        staticCollider.InitializeWithNoBody(staticAgent);
        StiffBody2D dynamicBody = CreateDynamicCircle(context, new Vector2d(Fixed64.Half, Fixed64.Zero));

        context.Simulate();

        dynamicBody.Position.x.Should().BeGreaterThan(Fixed64.Half);
    }

    [Fact]
    public void Simulate_WithSameAgent2DColliders_ShouldSkipCollision()
    {
        using GravitasWorldContext context = Create2DContext();
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var first = new LSCircleCollider2D(Fixed64.Half) { IsTrigger = true };
        var second = new LSCircleCollider2D(Fixed64.Half) { IsTrigger = true };
        int triggerCount = 0;
        first.OnTriggerEnter += _ => triggerCount++;

        first.InitializeWithNoBody(agent);
        second.InitializeWithNoBody(agent);
        context.Simulate();

        triggerCount.Should().Be(0);
    }

    private static GravitasWorldContext Create2DContext()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        return context;
    }

    private static StiffBody2D CreateDynamicCircle(GravitasWorldContext context, Vector2d position)
    {
        var transform = new FixedTransform(new Vector3d(position.x, Fixed64.Zero, position.y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new StiffBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One
        };
        body.Initialize(position);
        return body;
    }
}
