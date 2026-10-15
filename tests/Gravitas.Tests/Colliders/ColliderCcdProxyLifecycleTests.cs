using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class ColliderCcdProxyLifecycleTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void StationaryPendingAuthoring_ShouldRefreshPlanarAndMixedMemberships(bool mixed)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        context.Settings.RuntimeMode = mixed ? PhysicsRuntimeMode.Mixed : PhysicsRuntimeMode.TwoD;
        var collider = new LSCircleCollider2D(Fixed64.Half);
        var body = new SolidBody2D(new TestMatterAgent(context), collider);
        body.Initialize(Vector2d.Zero, motionType: BodyMotionType.Kinematic);
        var hits = new SwiftList<Physics2DHit>();
        Vector2d grownPoint = Vector2d.Right * (Fixed64)3;
        context.Query2D.OverlapCircleAll(grownPoint, Fixed64.Quarter, hits).Should().Be(0);

        collider.Radius = (Fixed64)4;
        context.Simulate();
        context.LateSimulate();

        body.Position.Should().Be(Vector2d.Zero);
        context.Query2D.OverlapCircleAll(grownPoint, Fixed64.Quarter, hits).Should().Be(1);
        hits[0].Collider.Should().BeSameAs(collider);
        var previousCoordinates = collider.PartitionCoordinates!.ToArray();
        var previousMixedCoordinates = collider.MixedPartitionCoordinates?.ToArray();

        collider.Radius = Fixed64.Half;
        collider.LocalOffset = Vector2d.Right * (Fixed64)12;
        context.Simulate();
        context.LateSimulate();

        body.Position.Should().Be(Vector2d.Zero);
        collider.PartitionCoordinates.Should().NotIntersectWith(previousCoordinates);
        context.Query2D.OverlapCircleAll(grownPoint, Fixed64.Quarter, hits).Should().Be(0);
        context.Query2D.OverlapCircleAll(Vector2d.Right * (Fixed64)12,
            Fixed64.Quarter, hits).Should().Be(1);
        hits[0].Collider.Should().BeSameAs(collider);
        if (mixed)
        {
            previousMixedCoordinates.Should().NotBeNullOrEmpty();
            collider.MixedPartitionCoordinates.Should().NotIntersectWith(previousMixedCoordinates!);
            context.QueryMixed.SweepSphereAgainst2D(new Vector3d(12, 3, 0),
                new Vector3d(12, -3, 0), Fixed64.Quarter, PhysicsLayerMask.All,
                out PhysicsMixedHit mixedHit).Should().BeTrue();
            mixedHit.Collider2D.Should().BeSameAs(collider);
        }
    }
}
