using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed class MixedBatchQueryTests
{
    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);

    [Fact]
    public void MixedSweepBatches_ShouldPreserveOrderRangesAndReducerKind()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D platform = CreateBodylessBox2D(context, Vector2d.Zero, new Vector2d((Fixed64)4, (Fixed64)4));
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(context, Vector3d.Zero);
        PhysicsSweepSphereAgainst2DRequest[] sphereRequests =
        {
            new(new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero), new Vector3d(Fixed64.Zero, (Fixed64)(-3), Fixed64.Zero), Fixed64.Half, IncludeLayerZero),
            new(Vector3d.Zero, Vector3d.Zero, Fixed64.Half, IncludeLayerZero)
        };
        PhysicsSweepCircleAgainst3DRequest[] circleRequests =
        {
            new(new Vector2d((Fixed64)(-3), Fixed64.Zero), new Vector2d((Fixed64)3, Fixed64.Zero), Fixed64.Half, Fixed64.Zero, Fixed64.Half, IncludeLayerZero),
            new(Vector2d.Zero, Vector2d.Zero, Fixed64.Half, Fixed64.Zero, Fixed64.Half, IncludeLayerZero)
        };
        PhysicsMixedHit[] closest = new PhysicsMixedHit[2];
        var hits = new SwiftList<PhysicsMixedHit>();
        PhysicsQueryHitRange[] ranges = new PhysicsQueryHitRange[2];

        int sphereClosestCount = context.QueryMixed.SweepSphereAgainst2DBatch(sphereRequests, closest);
        int sphereAllCount = context.QueryMixed.SweepSphereAgainst2DAllBatch(sphereRequests, hits, ranges);

        sphereClosestCount.Should().Be(1);
        closest[0].Collider2D.Should().BeSameAs(platform);
        closest[0].ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
        closest[1].Collider2D.Should().BeNull();
        sphereAllCount.Should().Be(1);
        ranges[0].Count.Should().Be(1);
        ranges[1].Count.Should().Be(0);
        hits[ranges[0].Start].Collider2D.Should().BeSameAs(platform);
        hits[ranges[0].Start].ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);

        int circleClosestCount = context.QueryMixed.SweepCircleAgainst3DBatch(circleRequests, closest);
        int circleAllCount = context.QueryMixed.SweepCircleAgainst3DAllBatch(circleRequests, hits, ranges);

        circleClosestCount.Should().Be(1);
        closest[0].Collider3D.Should().BeSameAs(sphere.Collider);
        closest[0].ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
        closest[1].Collider3D.Should().BeNull();
        circleAllCount.Should().Be(1);
        ranges[0].Count.Should().Be(1);
        ranges[1].Count.Should().Be(0);
        hits[ranges[0].Start].Collider3D.Should().BeSameAs(sphere.Collider);
        hits[ranges[0].Start].ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void MixedSweepBatches_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBodylessBox2D(context, Vector2d.Zero, new Vector2d((Fixed64)4, (Fixed64)4));
        _ = CreateSphere3D(context, Vector3d.Zero);
        PhysicsSweepSphereAgainst2DRequest[] sphereRequests =
        {
            new(new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero), new Vector3d(Fixed64.Zero, (Fixed64)(-3), Fixed64.Zero), Fixed64.Half, IncludeLayerZero)
        };
        PhysicsSweepCircleAgainst3DRequest[] circleRequests =
        {
            new(new Vector2d((Fixed64)(-3), Fixed64.Zero), new Vector2d((Fixed64)3, Fixed64.Zero), Fixed64.Half, Fixed64.Zero, Fixed64.Half, IncludeLayerZero)
        };
        PhysicsMixedHit[] closest = new PhysicsMixedHit[1];
        var hits = new SwiftList<PhysicsMixedHit>(4);
        PhysicsQueryHitRange[] ranges = new PhysicsQueryHitRange[1];

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(() =>
        {
            context.QueryMixed.SweepSphereAgainst2DBatch(sphereRequests, closest);
            context.QueryMixed.SweepSphereAgainst2DAllBatch(sphereRequests, hits, ranges);
            context.QueryMixed.SweepCircleAgainst3DBatch(circleRequests, closest);
            context.QueryMixed.SweepCircleAgainst3DAllBatch(circleRequests, hits, ranges);
        });

        allocatedBytes.Should().Be(0);
    }

    private static GravitasWorldContext CreateMixedContext()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.ApplySettings(new PhysicsSettings(4, null));
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-8), (Fixed64)(-4), (Fixed64)(-8)),
                new Vector3d((Fixed64)8, (Fixed64)4, (Fixed64)8)),
            out _).Should().BeTrue();
        return context;
    }

    private static ScenarioBody<LSSphereCollider> CreateSphere3D(GravitasWorldContext context, Vector3d position)
    {
        var collider = new LSSphereCollider();
        var agent = new TestMatterAgent(context, new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One));
        var body = new SolidBody(agent, collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(position, FixedQuaternion.Identity, BodyMotionType.Static);
        return new ScenarioBody<LSSphereCollider>(body, collider);
    }

    private static LSCollider2D CreateBodylessBox2D(
        GravitasWorldContext context,
        Vector2d position,
        Vector2d size)
    {
        var collider = new LSAABBoxCollider2D(size);
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One));
        collider.InitializeWithNoBody(agent);
        return collider;
    }
}
