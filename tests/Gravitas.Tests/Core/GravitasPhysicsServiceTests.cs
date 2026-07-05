using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class GravitasPhysicsServiceTests
{
    [Fact]
    public void AssimilateCollider_ShouldAllocateContextLocalColliderIds()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();
        var colliderA = new LSSphereCollider();
        var colliderB = new LSSphereCollider();

        int idA = contextA.Physics.AssimilateCollider(colliderA);
        int idB = contextB.Physics.AssimilateCollider(colliderB);

        idA.Should().Be(0);
        idB.Should().Be(0);
        colliderA.Id.Should().Be(0);
        colliderB.Id.Should().Be(0);
        colliderA.Context.Should().BeSameAs(contextA);
        colliderB.Context.Should().BeSameAs(contextB);
        contextA.Physics.ColliderCount.Should().Be(1);
        contextB.Physics.ColliderCount.Should().Be(1);
    }

    [Fact]
    public void TryGetColliderById_ShouldResolveOnlyWithinOwningContext()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();
        var colliderA1 = new LSSphereCollider();
        var colliderA2 = new LSSphereCollider();
        var colliderB1 = new LSSphereCollider();

        int idA1 = contextA.Physics.AssimilateCollider(colliderA1);
        int idA2 = contextA.Physics.AssimilateCollider(colliderA2);
        int idB1 = contextB.Physics.AssimilateCollider(colliderB1);

        contextA.Physics.TryGetColliderById(idA1, out LSCollider? foundA1).Should().BeTrue();
        contextA.Physics.TryGetColliderById(idA2, out LSCollider? foundA2).Should().BeTrue();
        contextB.Physics.TryGetColliderById(idB1, out LSCollider? foundB1).Should().BeTrue();
        contextB.Physics.TryGetColliderById(idA2, out LSCollider? missingInB).Should().BeFalse();
        foundA1.Should().BeSameAs(colliderA1);
        foundA2.Should().BeSameAs(colliderA2);
        foundB1.Should().BeSameAs(colliderB1);
        missingInB.Should().BeNull();
    }

    [Fact]
    public void ColliderRegistration_ShouldUseCompactServiceIndicesAndReusableIds()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var first = new LSSphereCollider();
        var second = new LSSphereCollider();
        var third = new LSSphereCollider();

        int firstId = context.Physics.AssimilateCollider(first);
        int secondId = context.Physics.AssimilateCollider(second);
        int thirdId = context.Physics.AssimilateCollider(third);

        context.Physics.ColliderCount.Should().Be(3);
        context.Physics.TryGetColliderByServiceIndex(0, out LSCollider? firstByIndex).Should().BeTrue();
        context.Physics.TryGetColliderByServiceIndex(1, out LSCollider? secondByIndex).Should().BeTrue();
        context.Physics.TryGetColliderByServiceIndex(2, out LSCollider? thirdByIndex).Should().BeTrue();
        firstByIndex.Should().BeSameAs(first);
        secondByIndex.Should().BeSameAs(second);
        thirdByIndex.Should().BeSameAs(third);

        context.Physics.DessimilateCollider(second);
        var replacement = new LSSphereCollider();
        int replacementId = context.Physics.AssimilateCollider(replacement);

        firstId.Should().Be(0);
        secondId.Should().Be(1);
        thirdId.Should().Be(2);
        replacementId.Should().Be(1);
        second.Id.Should().Be(-1);
        context.Physics.PeakColliderCount.Should().Be(3);
        context.Physics.ColliderCount.Should().Be(3);
        context.Physics.TryGetColliderById(secondId, out LSCollider? replacementById).Should().BeTrue();
        context.Physics.TryGetColliderByServiceIndex(0, out LSCollider? compactFirst).Should().BeTrue();
        context.Physics.TryGetColliderByServiceIndex(1, out LSCollider? compactThird).Should().BeTrue();
        context.Physics.TryGetColliderByServiceIndex(2, out LSCollider? compactReplacement).Should().BeTrue();
        replacementById.Should().BeSameAs(replacement);
        compactFirst.Should().BeSameAs(first);
        compactThird.Should().BeSameAs(third);
        compactReplacement.Should().BeSameAs(replacement);
    }

    [Fact]
    public void ColliderIsStatic_ShouldBeTrueForBodylessAndPositionFrozenColliders()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var bodyless = new LSSphereCollider();
        bodyless.InitializeWithNoBody(new TestMatterAgent(context));
        SolidBody body = CreateInitializedBody(context);
        SolidBody nonDynamicBody = CreateInitializedBody(context, isDynamic: false);

        bodyless.IsStatic.Should().BeTrue();
        body.Collider.IsStatic.Should().BeFalse();
        nonDynamicBody.Collider.IsStatic.Should().BeTrue();

        body.FreezeAxes = BodyFreezeAxes3D.Position;
        body.Collider.IsStatic.Should().BeTrue();

        body.FreezeAxes = BodyFreezeAxes3D.None;
        body.Collider.IsStatic.Should().BeFalse();
        body.IsKinematic = true;
        body.Collider.IsStatic.Should().BeFalse();
    }

    [Fact]
    public void CollisionPair_ShouldRejectCollidersFromDifferentContexts()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();
        var colliderA = new LSSphereCollider();
        var colliderB = new LSSphereCollider();
        contextA.Physics.AssimilateCollider(colliderA);
        contextB.Physics.AssimilateCollider(colliderB);

        Action crossContextPair = () => _ = new CollisionHandling.CollisionPair(colliderA, colliderB);

        crossContextPair.Should()
            .Throw<ArgumentException>()
            .WithMessage("*same context*");
    }

    [Fact]
    public void DessimilateCollider_ShouldClear3DPairReferencesBeforeReusingId()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * Fixed64.Half);
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();
        first.Collider.TryGetCollisionPair(second.Collider.Id, out _).Should().BeTrue();
        int firstId = first.Collider.Id;

        scenario.Context.Physics.DessimilateCollider(first.Collider);
        ScenarioBody<LSSphereCollider> replacement = scenario.CreateSphere(Vector3d.Up * (Fixed64)4);

        first.Collider.Id.Should().Be(-1);
        replacement.Collider.Id.Should().Be(firstId);
        second.Collider.CollisionPairHolderCount.Should().Be(0);
    }

    [Fact]
    public void SolidBodyInitialize_ShouldRegisterBodyAndColliderWithContextPhysics()
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
        collider.Context.Should().BeSameAs(context);
        body.DynamicId.Should().Be(0);
        collider.Id.Should().Be(0);
        context.Physics.BodyCount.Should().Be(1);
        context.Physics.ColliderCount.Should().Be(1);
        context.Physics.TryGetColliderById(collider.Id, out LSCollider? resolved).Should().BeTrue();
        resolved.Should().BeSameAs(collider);
    }

    [Fact]
    public void AddLinearImpulse_ShouldUseOwningContextDeltaTime()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();
        contextA.SetFrameRate(10);
        contextB.SetFrameRate(100);
        SolidBody bodyA = CreateInitializedBody(contextA);
        SolidBody bodyB = CreateInitializedBody(contextB);

        bodyA.AddLinearImpulse(Vector3d.Right);
        bodyB.AddLinearImpulse(Vector3d.Right);

        bodyA.LinearVelocity.X.Should().Be(Fixed64.One / (Fixed64)10);
        bodyB.LinearVelocity.X.Should().Be(Fixed64.One / (Fixed64)100);
    }

    [Fact]
    public void Reset_ShouldClearContextLocalColliderTable()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var collider = new LSSphereCollider();
        int id = context.Physics.AssimilateCollider(collider);

        context.Physics.Reset();

        context.Physics.ColliderCount.Should().Be(0);
        context.Physics.TryGetColliderById(id, out LSCollider? resolved).Should().BeFalse();
        resolved.Should().BeNull();
    }

    [Fact]
    public void DessimilateBody_AfterReset_ShouldNotUnderflowBodyCount()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var agent = new TestMatterAgent(context);
        var collider = new LSSphereCollider();
        var body = new SolidBody(agent, collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);

        context.Physics.Reset();
        body.Deactivate();

        context.Physics.BodyCount.Should().Be(0);
    }

    private static SolidBody CreateInitializedBody(GravitasWorldContext context, bool isDynamic = true)
    {
        var agent = new TestMatterAgent(context);
        var collider = new LSSphereCollider();
        var body = new SolidBody(agent, collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity, isDynamic);
        return body;
    }
}
