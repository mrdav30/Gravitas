using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class SolidBodyColliderReconfigurationLifetimeTests
{
    [Fact]
    public void NestedReconfigurationDuringSeparation_ShouldRetireEachInvocationSnapshot()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> outer = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> inner = scenario.CreateSphere(new Vector3d(4, 0, 0));
        for (int i = 0; i < 2; i++)
        {
            scenario.CreateStaticSphere(Vector3d.Zero).IsTrigger = true;
            scenario.CreateStaticSphere(new Vector3d(4, 0, 0)).IsTrigger = true;
        }
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();
        outer.Collider.CollisionPairCount.Should().Be(2);
        inner.Collider.CollisionPairCount.Should().Be(2);
        int outerExits = 0;
        int innerExits = 0;
        inner.Collider.OnTriggerExit += _ => innerExits++;
        outer.Collider.OnTriggerExit += ignoredCollider =>
        {
            if (++outerExits != 1) return;
            var nested = inner.Body.TryReconfigureCollider(ColliderShapeDefinition.Sphere(Fixed64.One),
                Vector3d.Zero, new Vector3d(12, 0, 0), out _, out var nestedFailure);
            nested.Should().Be(ColliderReconfigurationStatus.Applied);
            nestedFailure.Should().BeNull();
        };

        var result = outer.Body.TryReconfigureCollider(ColliderShapeDefinition.Sphere(Fixed64.One),
            Vector3d.Zero, new Vector3d(8, 0, 0), out _, out var failure);

        result.Should().Be(ColliderReconfigurationStatus.Applied);
        failure.Should().BeNull();
        outerExits.Should().Be(2);
        innerExits.Should().Be(2);
        outer.Collider.CollisionPairCount.Should().Be(0);
        inner.Collider.CollisionPairCount.Should().Be(0);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ResetDuringSeparation_ShouldReturnAcceptedResultAndOnlyCallbackFailure(bool throwAfterReset)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> actor = scenario.CreateSphere(Vector3d.Zero);
        LSSphereCollider first = scenario.CreateStaticSphere(Vector3d.Zero);
        LSSphereCollider second = scenario.CreateStaticSphere(Vector3d.Zero);
        first.IsTrigger = true;
        second.IsTrigger = true;
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();
        scenario.Context.Physics.GetCollisionPair(actor.Collider.Id, first.Id)!.Active.Should().BeTrue();
        scenario.Context.Physics.GetCollisionPair(actor.Collider.Id, second.Id)!.Active.Should().BeTrue();
        int exits = 0;
        var callbackFailure = new InvalidOperationException("exit after reset");
        actor.Collider.OnTriggerExit += _ =>
        {
            if (++exits != 1) return;
            scenario.Context.Reset();
            if (throwAfterReset) throw callbackFailure;
        };
        ColliderReconfigurationStatus result = default;
        Exception? failure = null;

        Action operation = () => result = actor.Body.TryReconfigureCollider(
            ColliderShapeDefinition.Sphere(Fixed64.One), Vector3d.Zero,
            Vector3d.Right * (Fixed64)5, out _, out failure);

        operation.Should().NotThrow("reset during notification must not invalidate the retirement snapshot");
        result.Should().Be(ColliderReconfigurationStatus.Applied);
        failure.Should().BeSameAs(throwAfterReset ? callbackFailure : null);
        exits.Should().BeGreaterThan(0);
        actor.Body.Position3d.Should().Be(Vector3d.Right * (Fixed64)5);
        actor.Collider.ScaledRadius.Should().Be(Fixed64.One);
        scenario.Context.Physics.ColliderCount.Should().Be(0);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Reconfiguration_ShouldPreservePairsCreatedAfterCallbackReinitializesBody(bool inPublisher)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> actor = scenario.CreateSphere(Vector3d.Zero);
        LSSphereCollider trigger = scenario.CreateStaticSphere(Vector3d.Zero);
        trigger.IsTrigger = true;
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();
        scenario.Context.Physics.GetCollisionPair(actor.Collider.Id, trigger.Id)!.Active.Should().BeTrue();
        CollisionPair? freshPair = null;
        bool reinitialized = false;
        Action recycle = () =>
        {
            if (reinitialized) return;
            reinitialized = true;
            actor.Body.Deactivate();
            actor.Body.Initialize(Vector3d.Zero, FixedQuaternion.Identity, BodyMotionType.Dynamic);
            scenario.Context.LateSimulate();
            freshPair = scenario.Context.Physics.GetCollisionPair(actor.Collider.Id, trigger.Id);
            freshPair.Should().NotBeNull();
            freshPair!.Active.Should().BeTrue();
        };
        if (!inPublisher) actor.Collider.OnTriggerExit += _ => recycle();

        var result = actor.Body.TryReconfigureCollider(ColliderShapeDefinition.Sphere(Fixed64.One),
            Vector3d.Zero, Vector3d.Right * (Fixed64)5, out _, out var failure,
            inPublisher ? recycle : null);

        result.Should().Be(ColliderReconfigurationStatus.Applied);
        failure.Should().BeNull();
        reinitialized.Should().BeTrue();
        freshPair!.Active.Should().BeTrue();
        scenario.Context.Physics.GetCollisionPair(actor.Collider.Id, trigger.Id).Should().BeSameAs(freshPair);
    }
}
