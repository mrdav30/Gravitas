using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedResponseTests
{
    [Fact]
    public void Simulate_WithSeparatedFormerMixedPair_ShouldEmitExitAndRecyclePair()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        int exited3D = 0;
        int exited2D = 0;
        body3D.Collider.OnMixedContactExit += other =>
        {
            other.Should().BeSameAs(body2D.Collider);
            exited3D++;
        };
        body2D.Collider.OnMixedContactExit += other =>
        {
            other.Should().BeSameAs(body3D.Collider);
            exited2D++;
        };

        Step(context);
        body3D.Body.SetPosition(new Vector3d((Fixed64)6, Fixed64.Zero, Fixed64.Zero));
        Step(context);

        exited3D.Should().Be(1);
        exited2D.Should().Be(1);
        context.MixedCollisions.ActivePairCount.Should().Be(0);
        context.MixedCollisions.PooledPairCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void Deactivate2DParticipant_WithActiveMixedPair_ShouldEmitExitAndRecyclePair()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        int exited3D = 0;
        int exited2D = 0;
        body3D.Collider.OnMixedContactExit += other =>
        {
            other.Should().BeSameAs(body2D.Collider);
            exited3D++;
        };
        body2D.Collider.OnMixedContactExit += other =>
        {
            other.Should().BeSameAs(body3D.Collider);
            exited2D++;
        };

        Step(context);
        body2D.Deactivate();

        exited3D.Should().Be(1);
        exited2D.Should().Be(1);
        context.MixedCollisions.ActivePairCount.Should().Be(0);
        context.MixedCollisions.PooledPairCount.Should().BeGreaterThan(0);
        body2D.Collider.IsActive.Should().BeFalse();
    }

    [Fact]
    public void Deactivate3DParticipant_WithActiveMixedPair_ShouldEmitExitAndRecyclePair()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        int exited3D = 0;
        int exited2D = 0;
        body3D.Collider.OnMixedContactExit += other =>
        {
            other.Should().BeSameAs(body2D.Collider);
            exited3D++;
        };
        body2D.Collider.OnMixedContactExit += other =>
        {
            other.Should().BeSameAs(body3D.Collider);
            exited2D++;
        };

        Step(context);
        body3D.Body.Deactivate();

        exited3D.Should().Be(1);
        exited2D.Should().Be(1);
        context.MixedCollisions.ActivePairCount.Should().Be(0);
        context.MixedCollisions.PooledPairCount.Should().BeGreaterThan(0);
        body3D.Collider.IsActive.Should().BeFalse();
    }

    [Fact]
    public void Deactivate3DParticipant_WhenExitRebinds2DParticipant_ShouldNotNotifyNewLifetime()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        Vector2d reboundPosition = new((Fixed64)8, Fixed64.Zero);
        string events3D = string.Empty;
        string events2D = string.Empty;
        bool rebound = false;
        body3D.Collider.OnMixedContactExit += _ =>
        {
            events3D += "exit;";
            body2D.Deactivate();
            body2D.Initialize(reboundPosition);
            rebound = true;
        };
        body2D.Collider.OnMixedContactExit += _ => events2D += rebound ? "new-exit;" : "old-exit;";

        Step(context);
        body3D.Body.Deactivate();

        events3D.Should().Be("exit;");
        events2D.Should().BeEmpty();
        body2D.Active.Should().BeTrue();
        body2D.Position.Should().Be(reboundPosition);
        context.MixedCollisions.ActivePairCount.Should().Be(0);
    }

    [Fact]
    public void Deactivate3DParticipant_WhenExitDeactivates2DParticipant_ShouldFinishBothLifecycles()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        int exited3D = 0;
        int exited2D = 0;
        body3D.Collider.OnMixedContactExit += _ =>
        {
            exited3D++;
            body2D.Deactivate();
        };
        body2D.Collider.OnMixedContactExit += _ => exited2D++;

        Step(context);
        body3D.Body.Deactivate();

        exited3D.Should().Be(1);
        exited2D.Should().Be(1);
        body2D.Active.Should().BeFalse();
        context.MixedCollisions.ActivePairCount.Should().Be(0);
    }

    [Fact]
    public void MixedEnter_WhenCallbackDeactivatesParticipant_ShouldDeferExitUntilCallbackReturns()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        string events = string.Empty;
        int entered2D = 0;
        int exited2D = 0;
        body3D.Collider.OnMixedContactEnter += _ =>
        {
            events += "enter-start;";
            body3D.Body.Deactivate();
            events += "enter-end;";
        };
        body3D.Collider.OnMixedContactExit += _ => events += "exit;";
        body3D.Collider.OnMixedContact += _ => events += "stay;";
        body2D.Collider.OnMixedContactEnter += _ => entered2D++;
        body2D.Collider.OnMixedContactExit += _ => exited2D++;

        Step(context);

        events.Should().Be("enter-start;enter-end;exit;");
        entered2D.Should().Be(0);
        exited2D.Should().Be(0);
        context.MixedCollisions.ActivePairCount.Should().Be(0);
    }

    [Fact]
    public void MarkColliding_WhenDeferredExitCallbacksThrow_ShouldRetainGuardAndAggregateInDimensionOrder()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        int exits3D = 0;
        int exits2D = 0;
        bool guarded3D = false;
        bool guarded2D = false;
        body2D.Collider.OnMixedContactEnter += _ => pair.MarkSeparated();
        body3D.Collider.OnMixedContactExit += _ =>
        {
            exits3D++;
            guarded3D = pair.IsNotificationInProgress;
            throw new InvalidOperationException("3D exit failure");
        };
        body2D.Collider.OnMixedContactExit += _ =>
        {
            exits2D++;
            guarded2D = pair.IsNotificationInProgress;
            throw new ArgumentException("2D exit failure");
        };

        Action notify = () => pair.MarkColliding(frame: 1, contact: default);

        AggregateException exception = notify.Should().Throw<AggregateException>().Which.Flatten();
        exception.InnerExceptions.Should().HaveCount(2);
        exception.InnerExceptions[0].Should().BeOfType<InvalidOperationException>()
            .Which.Message.Should().Be("3D exit failure");
        exception.InnerExceptions[1].Should().BeOfType<ArgumentException>()
            .Which.Message.Should().Be("2D exit failure");
        exits3D.Should().Be(1);
        exits2D.Should().Be(1);
        guarded3D.Should().BeTrue();
        guarded2D.Should().BeTrue();
        pair.IsNotificationInProgress.Should().BeFalse();

        Action retry = pair.MarkSeparated;

        retry.Should().NotThrow();
        exits3D.Should().Be(1);
        exits2D.Should().Be(1);
    }

    [Fact]
    public void MarkColliding_When3DEnterThrows_ShouldResetGuardAndRetainOnlyDeliveredAdmission()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        int entered2D = 0;
        int exited3D = 0;
        int exited2D = 0;
        body3D.Collider.OnMixedContactEnter += _ => throw new InvalidOperationException("3D enter failure");
        body2D.Collider.OnMixedContactEnter += _ => entered2D++;
        body3D.Collider.OnMixedContactExit += _ => exited3D++;
        body2D.Collider.OnMixedContactExit += _ => exited2D++;

        Action notify = () => pair.MarkColliding(frame: 1, contact: default);

        notify.Should().Throw<InvalidOperationException>().WithMessage("3D enter failure");
        entered2D.Should().Be(0);
        pair.IsNotificationInProgress.Should().BeFalse();

        pair.MarkSeparated();

        exited3D.Should().Be(1);
        exited2D.Should().Be(0);
    }
}
