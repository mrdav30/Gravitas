using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using System;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class ColliderTriggerTests
{
    [Fact]
    public void Bodyless3DTrigger_ShouldNotifyBothCollidersEnterStayExitWithoutContact()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        LSSphereCollider trigger = scenario.CreateStaticSphere(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        trigger.IsTrigger = true;
        int triggerEnter = 0;
        int triggerStay = 0;
        int triggerExit = 0;
        int bodyEnter = 0;
        int bodyStay = 0;
        int bodyExit = 0;
        int contactEnter = 0;
        int contactStay = 0;

        trigger.OnTriggerEnter += other =>
        {
            other.Should().BeSameAs(body.Collider);
            triggerEnter++;
        };
        trigger.OnTriggerStay += other =>
        {
            other.Should().BeSameAs(body.Collider);
            triggerStay++;
        };
        trigger.OnTriggerExit += other =>
        {
            other.Should().BeSameAs(body.Collider);
            triggerExit++;
        };
        body.Collider.OnTriggerEnter += other =>
        {
            other.Should().BeSameAs(trigger);
            bodyEnter++;
        };
        body.Collider.OnTriggerStay += other =>
        {
            other.Should().BeSameAs(trigger);
            bodyStay++;
        };
        body.Collider.OnTriggerExit += other =>
        {
            other.Should().BeSameAs(trigger);
            bodyExit++;
        };
        body.Collider.OnContactEnter += _ => contactEnter++;
        body.Collider.OnContact += _ => contactStay++;

        Step(scenario.Context);
        Step(scenario.Context);
        trigger.Deactivate();

        triggerEnter.Should().Be(1);
        bodyEnter.Should().Be(1);
        triggerStay.Should().Be(2);
        bodyStay.Should().Be(2);
        triggerExit.Should().Be(1);
        bodyExit.Should().Be(1);
        contactEnter.Should().Be(0);
        contactStay.Should().Be(0);
    }

    [Fact]
    public void Bodyless2DTrigger_ShouldNotifyBothCollidersEnterStayExitWithoutContact()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(frameRate: 4);
        SolidBody2D body = CreateCircle2D(context, Vector2d.Zero);
        LSCircleCollider2D trigger = CreateBodylessCircle2D(context, new Vector2d(Fixed64.Half, Fixed64.Zero));
        trigger.IsTrigger = true;
        int triggerEnter = 0;
        int triggerStay = 0;
        int triggerExit = 0;
        int bodyEnter = 0;
        int bodyStay = 0;
        int bodyExit = 0;
        int contactEnter = 0;
        int contactStay = 0;

        trigger.OnTriggerEnter += other =>
        {
            other.Should().BeSameAs(body.Collider);
            triggerEnter++;
        };
        trigger.OnTriggerStay += other =>
        {
            other.Should().BeSameAs(body.Collider);
            triggerStay++;
        };
        trigger.OnTriggerExit += other =>
        {
            other.Should().BeSameAs(body.Collider);
            triggerExit++;
        };
        body.Collider.OnTriggerEnter += other =>
        {
            other.Should().BeSameAs(trigger);
            bodyEnter++;
        };
        body.Collider.OnTriggerStay += other =>
        {
            other.Should().BeSameAs(trigger);
            bodyStay++;
        };
        body.Collider.OnTriggerExit += other =>
        {
            other.Should().BeSameAs(trigger);
            bodyExit++;
        };
        body.Collider.OnContactEnter += _ => contactEnter++;
        body.Collider.OnContact += _ => contactStay++;

        Step(context);
        body.Sleep();
        Step(context);
        body.SetPosition(new Vector2d((Fixed64)4, Fixed64.Zero));
        Step(context);

        triggerEnter.Should().Be(1);
        bodyEnter.Should().Be(1);
        triggerStay.Should().Be(2);
        bodyStay.Should().Be(2);
        triggerExit.Should().Be(1);
        bodyExit.Should().Be(1);
        contactEnter.Should().Be(0);
        contactStay.Should().Be(0);
    }

    [Fact]
    public void IsTrigger_WhenEnabledOnBodyCollider_ShouldReject()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body3D = scenario.CreateSphere(Vector3d.Zero);
        using GravitasWorldContext context2D = Physics2DTestWorld.CreateContext();
        SolidBody2D body2D = CreateCircle2D(context2D, Vector2d.Zero);

        Action set3D = () => body3D.Collider.IsTrigger = true;
        Action set2D = () => body2D.Collider.IsTrigger = true;

        set3D.Should().Throw<ArgumentException>();
        set2D.Should().Throw<ArgumentException>();
    }

    [Fact]
    public void InitializeWithBody_WhenColliderIsAlreadyTrigger_ShouldReject()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var trigger3D = new LSSphereCollider { IsTrigger = true };
        var agent3D = new TestMatterAgent(scenario.Context);
        var body3D = new SolidBody(agent3D, trigger3D);
        using GravitasWorldContext context2D = Physics2DTestWorld.CreateContext();
        var trigger2D = new LSCircleCollider2D(Fixed64.Half) { IsTrigger = true };
        var agent2D = new TestMatterAgent(context2D);
        var body2D = new SolidBody2D(agent2D, trigger2D);

        Action initialize3D = () => body3D.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
        Action initialize2D = () => body2D.Initialize(Vector2d.Zero);

        initialize3D.Should().Throw<ArgumentException>();
        initialize2D.Should().Throw<ArgumentException>();
    }

    [Fact]
    public void BodylessTriggerWithoutBodyParticipant_ShouldNotNotify()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(frameRate: 4);
        LSCircleCollider2D trigger = CreateBodylessCircle2D(context, Vector2d.Zero);
        LSCircleCollider2D bodyless = CreateBodylessCircle2D(context, new Vector2d(Fixed64.Half, Fixed64.Zero));
        trigger.IsTrigger = true;
        int triggerEnter = 0;
        int bodylessEnter = 0;
        trigger.OnTriggerEnter += _ => triggerEnter++;
        bodyless.OnTriggerEnter += _ => bodylessEnter++;

        Step(context);

        triggerEnter.Should().Be(0);
        bodylessEnter.Should().Be(0);
    }

    [Fact]
    public void BodylessMixed2DTrigger_ShouldNotifyBothCollidersEnterStayExitWithoutMixedContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        SolidBody body3D = CreateSphere3D(context, Vector3d.Zero);
        LSCircleCollider2D trigger = CreateBodylessCircle2D(context, Vector2d.Zero);
        trigger.IsTrigger = true;
        int triggerEnter = 0;
        int triggerStay = 0;
        int triggerExit = 0;
        int bodyEnter = 0;
        int bodyStay = 0;
        int bodyExit = 0;
        int contactEnter = 0;

        trigger.OnMixedTriggerEnter += other =>
        {
            other.Should().BeSameAs(body3D.Collider);
            triggerEnter++;
        };
        trigger.OnMixedTriggerStay += other =>
        {
            other.Should().BeSameAs(body3D.Collider);
            triggerStay++;
        };
        trigger.OnMixedTriggerExit += other =>
        {
            other.Should().BeSameAs(body3D.Collider);
            triggerExit++;
        };
        body3D.Collider.OnMixedTriggerEnter += other =>
        {
            other.Should().BeSameAs(trigger);
            bodyEnter++;
        };
        body3D.Collider.OnMixedTriggerStay += other =>
        {
            other.Should().BeSameAs(trigger);
            bodyStay++;
        };
        body3D.Collider.OnMixedTriggerExit += other =>
        {
            other.Should().BeSameAs(trigger);
            bodyExit++;
        };
        body3D.Collider.OnMixedContactEnter += _ => contactEnter++;

        Step(context);
        Step(context);
        trigger.Deactivate();

        triggerEnter.Should().Be(1);
        bodyEnter.Should().Be(1);
        triggerStay.Should().Be(2);
        bodyStay.Should().Be(2);
        triggerExit.Should().Be(1);
        bodyExit.Should().Be(1);
        contactEnter.Should().Be(0);
    }

    private static void Step(GravitasWorldContext context)
    {
        context.Simulate();
        context.LateSimulate();
    }

    private static SolidBody2D CreateCircle2D(GravitasWorldContext context, Vector2d position)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One
        };
        body.Initialize(position);
        return body;
    }

    private static SolidBody CreateSphere3D(GravitasWorldContext context, Vector3d position)
    {
        var transform = new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One);
        var body = new SolidBody(new TestMatterAgent(context, transform), new LSSphereCollider())
        {
            Mass = Fixed64.One
        };
        body.Initialize(position, FixedQuaternion.Identity);
        return body;
    }

    private static LSCircleCollider2D CreateBodylessCircle2D(GravitasWorldContext context, Vector2d position)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var collider = new LSCircleCollider2D(Fixed64.Half);
        collider.InitializeWithNoBody(agent);
        return collider;
    }

    private static GravitasWorldContext CreateMixedContext()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.SetFrameRate(4);
        context.Environment.Gravity = Fixed64.Zero;
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        var configuration = new GridConfiguration(
            new Vector3d((Fixed64)(-8), (Fixed64)(-8), (Fixed64)(-8)),
            new Vector3d((Fixed64)8, (Fixed64)8, (Fixed64)8));
        context.World.TryAddGrid(configuration, out _).Should().BeTrue();
        return context;
    }
}
