using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using GridForge.Grids;
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

        idA.Should().Be(1);
        idB.Should().Be(1);
        colliderA.Id.Should().Be(1);
        colliderB.Id.Should().Be(1);
        colliderA.Context.Should().BeSameAs(contextA);
        colliderB.Context.Should().BeSameAs(contextB);
        contextA.Physics.AssimilatedColliderCount.Should().Be(1);
        contextB.Physics.AssimilatedColliderCount.Should().Be(1);
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
    public void StiffBodyInitialize_ShouldRegisterBodyAndColliderWithContextPhysics()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var agent = new TestMatterAgent(context.World);
        var collider = new LSSphereCollider();
        var body = new StiffBody(agent, collider)
        {
            Mass = Fixed64.One
        };

        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);

        body.Context.Should().BeSameAs(context);
        collider.Context.Should().BeSameAs(context);
        body.DynamicId.Should().Be(0);
        collider.Id.Should().Be(1);
        context.Physics.AssimilatedBodyCount.Should().Be(1);
        context.Physics.AssimilatedColliderCount.Should().Be(1);
        context.Physics.TryGetColliderById(collider.Id, out LSCollider? resolved).Should().BeTrue();
        resolved.Should().BeSameAs(collider);
    }

    [Fact]
    public void Reset_ShouldClearContextLocalColliderTable()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var collider = new LSSphereCollider();
        int id = context.Physics.AssimilateCollider(collider);

        context.Physics.Reset();

        context.Physics.AssimilatedColliderCount.Should().Be(0);
        context.Physics.TryGetColliderById(id, out LSCollider? resolved).Should().BeFalse();
        resolved.Should().BeNull();
    }

    [Fact]
    public void DessimilateBody_AfterReset_ShouldNotUnderflowBodyCount()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var agent = new TestMatterAgent(context.World);
        var collider = new LSSphereCollider();
        var body = new StiffBody(agent, collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);

        context.Physics.Reset();
        body.Deactivate();

        context.Physics.AssimilatedBodyCount.Should().Be(0);
    }

    private sealed class TestMatterAgent : IMatterAgent
    {
        public TestMatterAgent(GridWorld world)
        {
            World = world;
            Transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        }

        public GridWorld World { get; }

        public FixedTransform Transform { get; }

        public bool IsParent => true;

        public bool IsInteracting => false;
    }
}
