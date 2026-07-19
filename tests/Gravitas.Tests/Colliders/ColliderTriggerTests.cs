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
    public void BodylessTriggerPolicy_ShouldRequireExactlyOneTriggerAndOneBodyParticipant()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        LSSphereCollider trigger3D = scenario.CreateStaticSphere(Vector3d.Right * Fixed64.Half);
        LSSphereCollider bodyless3D = scenario.CreateStaticSphere(Vector3d.Right);
        trigger3D.IsTrigger = true;
        bodyless3D.IsTrigger = true;
        int triggerEnter = 0;
        int bodyEnter = 0;
        int bodylessEnter = 0;

        trigger3D.OnTriggerEnter += _ => triggerEnter++;
        body.Collider.OnTriggerEnter += _ => bodyEnter++;
        bodyless3D.OnTriggerEnter += _ => bodylessEnter++;
        bodyless3D.OnTriggerExit += _ => bodylessEnter++;

        trigger3D.NotifyContact(body.Collider, isColliding: true, isChanged: true);
        body.Collider.NotifyContact(trigger3D, isColliding: true, isChanged: true);
        trigger3D.NotifyContact(bodyless3D, isColliding: true, isChanged: true);
        trigger3D.NotifyContact(bodyless3D, isColliding: false, isChanged: true);
        bodyless3D.NotifyContact(trigger3D, isColliding: true, isChanged: true);
        bodyless3D.NotifyContact(trigger3D, isColliding: false, isChanged: true);

        triggerEnter.Should().Be(1);
        bodyEnter.Should().Be(1);
        bodylessEnter.Should().Be(0);
    }

    [Fact]
    public void Bodyless2DTriggerPolicy_ShouldRequireExactlyOneTriggerAndOneBodyParticipant()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(frameRate: 4);
        SolidBody2D body = CreateCircle2D(context, Vector2d.Zero);
        LSCircleCollider2D trigger = CreateBodylessCircle2D(context, new Vector2d(Fixed64.Half, Fixed64.Zero));
        LSCircleCollider2D bodyless = CreateBodylessCircle2D(context, Vector2d.Right);
        trigger.IsTrigger = true;
        bodyless.IsTrigger = true;
        int triggerEnter = 0;
        int bodyEnter = 0;
        int bodylessEnter = 0;

        trigger.OnTriggerEnter += _ => triggerEnter++;
        body.Collider.OnTriggerEnter += _ => bodyEnter++;
        bodyless.OnTriggerEnter += _ => bodylessEnter++;
        bodyless.OnTriggerExit += _ => bodylessEnter++;

        trigger.NotifyContact(body.Collider, isColliding: true, isChanged: true);
        body.Collider.NotifyContact(trigger, isColliding: true, isChanged: true);
        trigger.NotifyContact(bodyless, isColliding: true, isChanged: true);
        trigger.NotifyContact(bodyless, isColliding: false, isChanged: true);
        bodyless.NotifyContact(trigger, isColliding: true, isChanged: true);
        bodyless.NotifyContact(trigger, isColliding: false, isChanged: true);

        triggerEnter.Should().Be(1);
        bodyEnter.Should().Be(1);
        bodylessEnter.Should().Be(0);
    }

    [Fact]
    public void NotifyContact_ShouldSuppressInactiveUnchangedAndBodylessTargetCallbacks()
    {
        using GravitasWorldContext context = CreateMixedContext();
        SolidBody body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody other3D = CreateSphere3D(context, Vector3d.Right * Fixed64.Half);
        LSSphereCollider bodyless3D = CreateBodylessSphere3D(context, Vector3d.Right);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        SolidBody2D other2D = CreateCircle2D(context, Vector2d.Right * Fixed64.Half);
        LSCircleCollider2D bodyless2D = CreateBodylessCircle2D(context, Vector2d.Right);
        int callbackCount = 0;

        body3D.Collider.OnContactEnter += _ => callbackCount++;
        body3D.Collider.OnContact += _ => callbackCount++;
        body3D.Collider.OnContactExit += _ => callbackCount++;
        body3D.Collider.OnMixedContactEnter += _ => callbackCount++;
        body3D.Collider.OnMixedContact += _ => callbackCount++;
        body3D.Collider.OnMixedContactExit += _ => callbackCount++;
        body2D.Collider.OnContactEnter += _ => callbackCount++;
        body2D.Collider.OnContact += _ => callbackCount++;
        body2D.Collider.OnContactExit += _ => callbackCount++;
        body2D.Collider.OnMixedContactEnter += _ => callbackCount++;
        body2D.Collider.OnMixedContact += _ => callbackCount++;
        body2D.Collider.OnMixedContactExit += _ => callbackCount++;

        body3D.Collider.IsActive = false;
        body3D.Collider.NotifyContact(other3D.Collider, isColliding: true, isChanged: true);
        body3D.Collider.NotifyMixedContact(body2D.Collider, isColliding: true, isChanged: true, isTriggerPair: false);
        body3D.Collider.IsActive = true;

        body3D.Collider.IsActive = false;
        body2D.Collider.NotifyMixedContact(body3D.Collider, isColliding: true, isChanged: true, isTriggerPair: false);
        body3D.Collider.IsActive = true;

        body2D.Collider.Deactivate();
        body2D.Collider.NotifyContact(other2D.Collider, isColliding: true, isChanged: true);
        body2D.Collider.NotifyMixedContact(body3D.Collider, isColliding: true, isChanged: true, isTriggerPair: false);

        body3D.Collider.NotifyContact(other3D.Collider, isColliding: false, isChanged: false);
        other2D.Collider.NotifyContact(bodyless2D, isColliding: false, isChanged: false);
        body3D.Collider.NotifyMixedContact(other2D.Collider, isColliding: false, isChanged: false, isTriggerPair: false);
        other2D.Collider.NotifyMixedContact(body3D.Collider, isColliding: false, isChanged: false, isTriggerPair: false);

        body3D.Collider.NotifyContact(bodyless3D, isColliding: true, isChanged: true);
        body3D.Collider.NotifyContact(bodyless3D, isColliding: false, isChanged: true);
        other2D.Collider.NotifyContact(bodyless2D, isColliding: true, isChanged: true);
        other2D.Collider.NotifyContact(bodyless2D, isColliding: false, isChanged: true);

        callbackCount.Should().Be(0);
    }

    [Fact]
    public void NotifyContact_ShouldRaiseSubscribedSameDimensionContactAndTriggerLifecycle()
    {
        using GravitasWorldContext context = CreateMixedContext();
        SolidBody body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody other3D = CreateSphere3D(context, Vector3d.Right * Fixed64.Half);
        LSSphereCollider trigger3D = CreateBodylessSphere3D(context, Vector3d.Right);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        SolidBody2D other2D = CreateCircle2D(context, Vector2d.Right * Fixed64.Half);
        LSCircleCollider2D trigger2D = CreateBodylessCircle2D(context, Vector2d.Right);
        trigger3D.IsTrigger = true;
        trigger2D.IsTrigger = true;
        int contactEnter3D = 0;
        int contactStay3D = 0;
        int contactExit3D = 0;
        int triggerEnter3D = 0;
        int triggerStay3D = 0;
        int triggerExit3D = 0;
        int contactEnter2D = 0;
        int contactStay2D = 0;
        int contactExit2D = 0;
        int triggerEnter2D = 0;
        int triggerStay2D = 0;
        int triggerExit2D = 0;

        body3D.Collider.OnContactEnter += other =>
        {
            other.Should().BeSameAs(other3D);
            contactEnter3D++;
        };
        body3D.Collider.OnContact += other =>
        {
            other.Should().BeSameAs(other3D);
            contactStay3D++;
        };
        body3D.Collider.OnContactExit += other =>
        {
            other.Should().BeSameAs(other3D);
            contactExit3D++;
        };
        trigger3D.OnTriggerEnter += other =>
        {
            other.Should().BeSameAs(body3D.Collider);
            triggerEnter3D++;
        };
        trigger3D.OnTriggerStay += other =>
        {
            other.Should().BeSameAs(body3D.Collider);
            triggerStay3D++;
        };
        trigger3D.OnTriggerExit += other =>
        {
            other.Should().BeSameAs(body3D.Collider);
            triggerExit3D++;
        };
        body2D.Collider.OnContactEnter += other =>
        {
            other.Should().BeSameAs(other2D);
            contactEnter2D++;
        };
        body2D.Collider.OnContact += other =>
        {
            other.Should().BeSameAs(other2D);
            contactStay2D++;
        };
        body2D.Collider.OnContactExit += other =>
        {
            other.Should().BeSameAs(other2D);
            contactExit2D++;
        };
        trigger2D.OnTriggerEnter += other =>
        {
            other.Should().BeSameAs(body2D.Collider);
            triggerEnter2D++;
        };
        trigger2D.OnTriggerStay += other =>
        {
            other.Should().BeSameAs(body2D.Collider);
            triggerStay2D++;
        };
        trigger2D.OnTriggerExit += other =>
        {
            other.Should().BeSameAs(body2D.Collider);
            triggerExit2D++;
        };

        NotifyContactLifecycle(body3D.Collider, other3D.Collider);
        NotifyContactLifecycle(body2D.Collider, other2D.Collider);
        NotifyContactLifecycle(trigger3D, body3D.Collider);
        NotifyContactLifecycle(trigger2D, body2D.Collider);

        contactEnter3D.Should().Be(1);
        contactStay3D.Should().Be(2);
        contactExit3D.Should().Be(1);
        triggerEnter3D.Should().Be(1);
        triggerStay3D.Should().Be(2);
        triggerExit3D.Should().Be(1);
        contactEnter2D.Should().Be(1);
        contactStay2D.Should().Be(2);
        contactExit2D.Should().Be(1);
        triggerEnter2D.Should().Be(1);
        triggerStay2D.Should().Be(2);
        triggerExit2D.Should().Be(1);
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

    [Fact]
    public void BodylessMixed3DTrigger_ShouldNotifyBothCollidersEnterStayExitWithoutMixedContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        LSSphereCollider trigger = CreateBodylessSphere3D(context, Vector3d.Zero);
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
            other.Should().BeSameAs(body2D.Collider);
            triggerEnter++;
        };
        trigger.OnMixedTriggerStay += other =>
        {
            other.Should().BeSameAs(body2D.Collider);
            triggerStay++;
        };
        trigger.OnMixedTriggerExit += other =>
        {
            other.Should().BeSameAs(body2D.Collider);
            triggerExit++;
        };
        body2D.Collider.OnMixedTriggerEnter += other =>
        {
            other.Should().BeSameAs(trigger);
            bodyEnter++;
        };
        body2D.Collider.OnMixedTriggerStay += other =>
        {
            other.Should().BeSameAs(trigger);
            bodyStay++;
        };
        body2D.Collider.OnMixedTriggerExit += other =>
        {
            other.Should().BeSameAs(trigger);
            bodyExit++;
        };
        body2D.Collider.OnMixedContactEnter += _ => contactEnter++;

        Step(context);
        body2D.Sleep();
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

    [Fact]
    public void BodylessMixedTriggerPolicy_ShouldRequireExactlyOneTriggerAndOneBodyParticipant()
    {
        using GravitasWorldContext context = CreateMixedContext();
        SolidBody body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        LSSphereCollider trigger3D = CreateBodylessSphere3D(context, Vector3d.Right);
        LSCircleCollider2D trigger2D = CreateBodylessCircle2D(context, Vector2d.Right);
        LSSphereCollider bodyless3D = CreateBodylessSphere3D(context, Vector3d.Right * (Fixed64)2);
        LSCircleCollider2D bodyless2D = CreateBodylessCircle2D(context, Vector2d.Right * (Fixed64)2);
        trigger3D.IsTrigger = true;
        trigger2D.IsTrigger = true;
        bodyless3D.IsTrigger = true;
        bodyless2D.IsTrigger = true;
        int allowedEnter = 0;
        int allowedExit = 0;
        int bodylessEnter = 0;
        int bodylessExit = 0;

        trigger3D.OnMixedTriggerEnter += _ => allowedEnter++;
        trigger3D.OnMixedTriggerExit += _ => allowedExit++;
        trigger2D.OnMixedTriggerEnter += _ => allowedEnter++;
        trigger2D.OnMixedTriggerExit += _ => allowedExit++;
        body3D.Collider.OnMixedTriggerEnter += _ => allowedEnter++;
        body3D.Collider.OnMixedTriggerExit += _ => allowedExit++;
        body2D.Collider.OnMixedTriggerEnter += _ => allowedEnter++;
        body2D.Collider.OnMixedTriggerExit += _ => allowedExit++;
        bodyless3D.OnMixedTriggerEnter += _ => bodylessEnter++;
        bodyless3D.OnMixedTriggerExit += _ => bodylessExit++;
        bodyless2D.OnMixedTriggerEnter += _ => bodylessEnter++;
        bodyless2D.OnMixedTriggerExit += _ => bodylessExit++;

        NotifyMixedTriggerLifecycle(trigger3D, body2D.Collider);
        NotifyMixedTriggerLifecycle(body3D.Collider, trigger2D);
        NotifyMixedTriggerLifecycle(trigger3D, bodyless2D);
        NotifyMixedTriggerLifecycle(bodyless3D, trigger2D);

        allowedEnter.Should().Be(4);
        allowedExit.Should().Be(4);
        bodylessEnter.Should().Be(0);
        bodylessExit.Should().Be(0);
    }

    [Fact]
    public void MixedExit_WithReusedOtherColliderLifetime_ShouldNotNotifyCurrent2DCollider()
    {
        using GravitasWorldContext context = CreateMixedContext();
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        SolidBody body3D = CreateSphere3D(context, Vector3d.Zero);
        var registration2D = new ColliderLifetimeToken2D(body2D.Collider);
        var retiredRegistration3D = new ColliderLifetimeToken(body3D.Collider);
        int exits = 0;
        body2D.Collider.OnMixedContactExit += _ => exits++;

        body3D.Deactivate();
        body3D.Initialize(Vector3d.Right, FixedQuaternion.Identity);
        body2D.Collider.NotifyMixedContact(
            body3D.Collider,
            isColliding: false,
            isChanged: true,
            isTriggerPair: false,
            allowInactive: true,
            registration2D,
            retiredRegistration3D,
            shouldRaiseTrigger: false);

        exits.Should().Be(0);
        body3D.Collider.LifetimeVersion.Should().BeGreaterThan(
            retiredRegistration3D.LifetimeVersion);
    }

    [Fact]
    public void MixedExit_WithReusedLocalColliderLifetime_ShouldNotNotifyCurrent2DCollider()
    {
        using GravitasWorldContext context = CreateMixedContext();
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        SolidBody body3D = CreateSphere3D(context, Vector3d.Zero);
        var retiredRegistration2D = new ColliderLifetimeToken2D(body2D.Collider);
        var registration3D = new ColliderLifetimeToken(body3D.Collider);
        int exits = 0;
        body2D.Collider.OnMixedContactExit += _ => exits++;

        body2D.Deactivate();
        body2D.Initialize(Vector2d.Right);
        body2D.Collider.NotifyMixedContact(
            body3D.Collider,
            isColliding: false,
            isChanged: true,
            isTriggerPair: false,
            allowInactive: true,
            retiredRegistration2D,
            registration3D,
            shouldRaiseTrigger: false);

        exits.Should().Be(0);
        body2D.Collider.LifetimeVersion.Should().BeGreaterThan(
            retiredRegistration2D.LifetimeVersion);
    }

    [Fact]
    public void MixedEnter_WithInactive3DCollider_ShouldNotNotify2DCollider()
    {
        using GravitasWorldContext context = CreateMixedContext();
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        SolidBody body3D = CreateSphere3D(context, Vector3d.Zero);
        var registration2D = new ColliderLifetimeToken2D(body2D.Collider);
        var registration3D = new ColliderLifetimeToken(body3D.Collider);
        int enters = 0;
        body2D.Collider.OnMixedContactEnter += _ => enters++;
        body3D.Deactivate();

        body2D.Collider.NotifyMixedContact(
            body3D.Collider,
            isColliding: true,
            isChanged: true,
            isTriggerPair: false,
            allowInactive: false,
            registration2D,
            registration3D,
            shouldRaiseTrigger: false);

        enters.Should().Be(0);
        body2D.Collider.IsActive.Should().BeTrue();
        body3D.Collider.IsActive.Should().BeFalse();
    }

    private static void Step(GravitasWorldContext context)
    {
        context.Simulate();
        context.LateSimulate();
    }

    private static void NotifyMixedTriggerLifecycle(LSCollider collider3D, LSCollider2D collider2D)
    {
        collider3D.NotifyMixedContact(collider2D, isColliding: true, isChanged: true, isTriggerPair: true);
        collider2D.NotifyMixedContact(collider3D, isColliding: true, isChanged: true, isTriggerPair: true);
        collider3D.NotifyMixedContact(collider2D, isColliding: false, isChanged: true, isTriggerPair: true);
        collider2D.NotifyMixedContact(collider3D, isColliding: false, isChanged: true, isTriggerPair: true);
    }

    private static void NotifyContactLifecycle(LSCollider first, LSCollider second)
    {
        first.NotifyContact(second, isColliding: true, isChanged: true);
        first.NotifyContact(second, isColliding: true, isChanged: false);
        first.NotifyContact(second, isColliding: false, isChanged: true);
    }

    private static void NotifyContactLifecycle(LSCollider2D first, LSCollider2D second)
    {
        first.NotifyContact(second, isColliding: true, isChanged: true);
        first.NotifyContact(second, isColliding: true, isChanged: false);
        first.NotifyContact(second, isColliding: false, isChanged: true);
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

    private static LSSphereCollider CreateBodylessSphere3D(GravitasWorldContext context, Vector3d position)
    {
        var transform = new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var collider = new LSSphereCollider();
        collider.InitializeWithNoBody(agent);
        return collider;
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
