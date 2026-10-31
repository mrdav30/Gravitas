using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class OrdinaryConvexPatchSweepTests
{
    private static readonly PhysicsLayerMask LayerZero = PhysicsLayerMask.FromLayer(0);
    private static readonly Fixed64 Tolerance = ConvexSweepQueryWorker.ContactTolerance;

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void FaceCrossing_ShouldKeepFiniteWitnessAndAnalyticCapsuleToi(int variant)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider target = Quad(variant);
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        var source = new LSCapsuleCollider { Radius = Fixed64.Half, Size = new Vector3d(1, 2, 1) };
        scenario.CreateBody(source, new Vector3d((Fixed64)(-3), Fixed64.Quarter, Fixed64.Quarter),
            FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.HalfPi));
        var hits = new SwiftList<Physics3DHit>();
        Vector3d travel = Vector3d.Right * (Fixed64)6;

        scenario.Context.Query3D.SweepCapsule(source, travel, LayerZero, out Physics3DHit expected).Should().BeTrue();
        expected.Collider.Should().BeSameAs(target);
        (expected.Distance - (Fixed64)2).Abs().Should().BeLessThanOrEqualTo(Tolerance);
        AssertOnQuad(expected);
        expected.Normal.Should().Be(Vector3d.Left);
        // At impact the leading hemisphere's center is one radius from the plane.
        Vector3d coreTip = source.Center + travel.Normalized * (expected.Distance + source.AxisLength / 2);
        ((expected.Point - coreTip).Magnitude - source.ScaledRadius).Abs().Should().BeLessThanOrEqualTo(Tolerance);
        AllocationTestHelper.MeasureSteadyState(() =>
        {
            scenario.Context.Query3D.SweepCapsuleAll(source, travel, LayerZero, hits);
        }, warmupIterations: 4, stabilizationIterations: 4, measurementIterations: 16).Should().Be(0);
        hits.Count.Should().Be(1);
        hits[0].Should().Be(expected);
    }

    [Fact]
    public void PointCrossingPatchCenter_ShouldKeepWitnessAtTheMovingPoint()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider target = Quad(0);
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        var worker = new ConvexSweepQueryWorker();
        worker.PrepareSphereSource(Vector3d.Left * (Fixed64)3, Fixed64.Zero, Vector3d.Right * (Fixed64)6);
        worker.TrySweepPreparedSource(target, out Physics3DHit hit).Should().BeTrue();
        (hit.Distance - (Fixed64)3).Abs().Should().BeLessThanOrEqualTo(Tolerance);
        hit.Point.Magnitude.Should().BeLessThanOrEqualTo(Tolerance);
        hit.Normal.Should().Be(Vector3d.Left);
    }

    [Fact]
    public void RoundedToleranceBoundary_ShouldRetainFiniteContact()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider target = Quad(0);
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        var worker = new ConvexSweepQueryWorker();
        worker.PrepareSphereSource(Vector3d.Left * (Tolerance + Fixed64.Epsilon),
            Fixed64.Zero, Vector3d.Right);
        worker.TrySweepPreparedSource(target, out Physics3DHit hit).Should().BeTrue();
        hit.Distance.Should().BeInRange(Fixed64.Zero, Tolerance * 2);
        hit.Point.Magnitude.Should().BeLessThanOrEqualTo(Tolerance);
        hit.Normal.Should().Be(Vector3d.Left);
    }

    [Fact]
    public void ExhaustedAdvancementBudget_ShouldNotInventAnEndpointContact()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider target = Quad(0);
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        var worker = new ConvexSweepQueryWorker(maxConservativeAdvancementIterations: 0);
        worker.PrepareSphereSource(Vector3d.Left * (Fixed64)3, Fixed64.Quarter, Vector3d.Right * (Fixed64)6);
        worker.TrySweepPreparedSource(target, out Physics3DHit hit).Should().BeFalse();
        hit.Should().Be(default(Physics3DHit));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RimEntry_ShouldKeepFiniteContactBeforeCenterCrossing(bool coplanar)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider target = Quad(0);
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        var source = new LSCapsuleCollider { Radius = Fixed64.Quarter,
            Size = new Vector3d(Fixed64.Half, (Fixed64)4, Fixed64.Half) };
        Vector3d center = coplanar ? new Vector3d(0, 3, 0) : new Vector3d(-3, -2, 0);
        Vector3d travel = coplanar ? Vector3d.Down * (Fixed64)4 : new Vector3d(6, 4, 0);
        scenario.CreateBody(source, center,
            FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.Pi / 4));
        var worker = new ConvexSweepQueryWorker();
        worker.PreparePrimitiveSource(source, travel);
        worker.TrySweepPreparedSource(target, out Physics3DHit hit).Should().BeTrue();
        AssertOnQuad(hit);
        hit.Distance.Should().BeGreaterThan(Fixed64.Zero);
        hit.Distance.Should().BeLessThan(travel.Magnitude / 2);
        if (!coplanar)
            (hit.Point.Y + Fixed64.One).Abs().Should().BeLessThanOrEqualTo(Tolerance);
        Vector3d movedCenter = center + travel.Normalized * hit.Distance;
        // Independent capsule geometry: distance to its finite core equals its radius.
        Vector3d axis = source.WorldAxis;
        Fixed64 along = FixedMath.Clamp(Vector3d.Dot(hit.Point - movedCenter, axis),
            -source.AxisLength / 2, source.AxisLength / 2);
        ((hit.Point - (movedCenter + axis * along)).Magnitude - source.ScaledRadius).Abs()
            .Should().BeLessThanOrEqualTo(Tolerance * 2);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void EqualDistanceTargets_ShouldKeepColliderIdOrderAndRepeatExactly(bool capsule)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider first = Quad(0), second = Quad(1);
        scenario.InitializeStaticCollider(first, Vector3d.Zero);
        scenario.InitializeStaticCollider(second, Vector3d.Zero);
        LSCollider source = capsule
            ? new LSCapsuleCollider { Radius = Fixed64.Half, Size = new Vector3d(1, 2, 1) }
            : new LSCuboidCollider();
        scenario.CreateBody(source, new Vector3d((Fixed64)(-3), Fixed64.Quarter, Fixed64.Quarter),
            FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.HalfPi));
        var hits = new SwiftList<Physics3DHit>();
        Vector3d travel = Vector3d.Right * (Fixed64)6;
        QueryAll().Should().Be(2);
        hits[0].Collider.Should().BeSameAs(first);
        hits[1].Collider.Should().BeSameAs(second);
        hits[0].Distance.Should().Be(hits[1].Distance);
        Physics3DHit expected = hits[0];
        for (int repeat = 0; repeat < 4; repeat++)
        {
            bool found = capsule
                ? scenario.Context.Query3D.SweepCapsule((LSCapsuleCollider)source, travel, LayerZero, out Physics3DHit closest)
                : scenario.Context.Query3D.SweepCuboid((LSCuboidCollider)source, travel, LayerZero, out closest);
            found.Should().BeTrue();
            closest.Should().Be(expected);
            QueryAll().Should().Be(2);
            hits[0].Should().Be(expected);
            hits[1].Collider.Should().BeSameAs(second);
        }
        int QueryAll() => capsule
            ? scenario.Context.Query3D.SweepCapsuleAll((LSCapsuleCollider)source, travel, LayerZero, hits)
            : scenario.Context.Query3D.SweepCuboidAll((LSCuboidCollider)source, travel, LayerZero, hits);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ScalarBoundary_ShouldRetainFiniteTriangleFallback(bool maximum)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider target = Quad(0);
        Fixed64 coordinate = maximum ? Fixed64.MaxValue - Fixed64.One : Fixed64.MinValue + Fixed64.One;
        scenario.InitializeStaticCollider(target, new Vector3d(Fixed64.Zero, coordinate, Fixed64.Zero));
        var source = new LSCuboidCollider();
        scenario.InitializeStaticCollider(source, new Vector3d((Fixed64)(-3),
            maximum ? Fixed64.MaxValue - Fixed64.Quarter : Fixed64.MinValue + Fixed64.Quarter, Fixed64.Zero));
        var worker = new ConvexSweepQueryWorker();
        worker.PreparePrimitiveSource(source, Vector3d.Right * (Fixed64)6);
        worker.TrySweepPreparedSource(target, out Physics3DHit hit).Should().BeTrue();
        (hit.Distance - Fixed64.FromFraction(5, 2)).Abs().Should().BeLessThanOrEqualTo(Tolerance);
        hit.Point.X.Abs().Should().BeLessThanOrEqualTo(Tolerance);
        hit.Normal.Should().Be(Vector3d.Left);
    }

    [Fact]
    public void HoleCrossing_ShouldNotSweepTheConvexHullOfTheRing()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var target = new LSMeshCollider(new[]
        {
            new Vector3d(0, -2, -2), new Vector3d(0, 2, -2), new Vector3d(0, 2, 2), new Vector3d(0, -2, 2),
            new Vector3d(0, -1, -1), new Vector3d(0, 1, -1), new Vector3d(0, 1, 1), new Vector3d(0, -1, 1)
        }, new[] { 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        var source = new LSCuboidCollider();
        scenario.CreateBody(source, Vector3d.Left * (Fixed64)3, FixedQuaternion.Identity);
        scenario.Context.Query3D.SweepCuboid(source, Vector3d.Right * (Fixed64)6, LayerZero, out _).Should().BeFalse();
    }

    [Fact]
    public void GrazingBodyWithCenterOutsidePatch_ShouldRetainFiniteTriangleContact()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider target = Quad(0);
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        var source = new LSCapsuleCollider { Radius = Fixed64.Half, Size = new Vector3d(1, 2, 1) };
        scenario.CreateBody(source, new Vector3d((Fixed64)(-3), Fixed64.FromFraction(5, 4), Fixed64.Zero), FixedQuaternion.Identity);
        scenario.Context.Query3D.SweepCapsule(source, Vector3d.Right * (Fixed64)6, LayerZero, out Physics3DHit hit)
            .Should().BeTrue();
        (hit.Distance - Fixed64.FromFraction(5, 2)).Abs().Should().BeLessThanOrEqualTo(Tolerance);
        AssertOnQuad(hit);
    }

    [Fact]
    public void DisconnectedConvexPatches_ShouldRetainTheirSeparateFiniteDomains()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var target = new LSMeshCollider(new[]
        {
            new Vector3d(0, -3, -1), new Vector3d(0, -1, -1), new Vector3d(0, -3, 1), new Vector3d(0, -1, 1),
            new Vector3d(0, 1, -1), new Vector3d(0, 3, -1), new Vector3d(0, 1, 1), new Vector3d(0, 3, 1)
        }, new[] { 0, 2, 1, 1, 2, 3, 4, 6, 5, 5, 6, 7 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        var source = new LSCuboidCollider { Size = new Vector3d(1, 8, 1) };
        scenario.CreateBody(source, Vector3d.Left * (Fixed64)3, FixedQuaternion.Identity);
        scenario.Context.Query3D.SweepCuboid(source, Vector3d.Right * (Fixed64)6, LayerZero, out Physics3DHit hit)
            .Should().BeTrue();
        (hit.Distance - Fixed64.FromFraction(5, 2)).Abs().Should().BeLessThanOrEqualTo(Tolerance);
        hit.Point.X.Abs().Should().BeLessThanOrEqualTo(Tolerance);
        hit.Point.Y.Abs().Should().BeInRange(Fixed64.One - Tolerance, (Fixed64)3 + Tolerance);
        hit.Point.Z.Abs().Should().BeLessThanOrEqualTo(Fixed64.One + Tolerance);
    }

    [Fact]
    public void RotatedScaledPatch_ShouldPreserveAuthoredFiniteGeometry()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 extent = Fixed64.One + Fixed64.Epsilon;
        var target = new LSMeshCollider(new[]
        {
            new Vector3d(Fixed64.Zero, -extent, -extent), new Vector3d(Fixed64.Zero, extent, -extent),
            new Vector3d(Fixed64.Zero, -extent, extent), new Vector3d(Fixed64.Zero, extent, extent)
        }, new[] { 0, 2, 1, 1, 2, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.Pi / 4);
        Vector3d origin = new(3, 2, 1);
        target.InitializeWithNoBody(new TestMatterAgent(scenario.Context,
            new FixedTransform(origin, rotation, new Vector3d(Fixed64.Two, Fixed64.FromFraction(1, 3), Fixed64.FromFraction(1, 5)))));
        Vector3d travel = rotation * Vector3d.Right * (Fixed64)6;
        var worker = new ConvexSweepQueryWorker();
        worker.PrepareSphereSource(origin - travel / 2, Fixed64.Quarter, travel);
        worker.TrySweepPreparedSource(target, out Physics3DHit hit).Should().BeTrue();
        (hit.Distance - Fixed64.FromFraction(11, 4)).Abs().Should().BeLessThanOrEqualTo(Tolerance);
        Vector3d localPoint = rotation.Inverse() * (hit.Point - origin);
        localPoint.X.Abs().Should().BeLessThanOrEqualTo(Tolerance);
        localPoint.Y.Abs().Should().BeLessThanOrEqualTo(Fixed64.FromFraction(1, 3) + Tolerance);
        localPoint.Z.Abs().Should().BeLessThanOrEqualTo(Fixed64.FromFraction(1, 5) + Tolerance);
        Vector3d impactCenter = origin - travel / 2 + travel.Normalized * hit.Distance;
        ((hit.Point - impactCenter).Magnitude - Fixed64.Quarter).Abs().Should().BeLessThanOrEqualTo(Tolerance);
        Vector3d.Dot(hit.Normal, -(rotation * Vector3d.Right)).Should().BeGreaterThan(Fixed64.FromFraction(9999, 10000));
    }

    private static void AssertOnQuad(Physics3DHit hit)
    {
        hit.Point.X.Abs().Should().BeLessThanOrEqualTo(Tolerance);
        hit.Point.Y.Abs().Should().BeLessThanOrEqualTo(Fixed64.One + Tolerance);
        hit.Point.Z.Abs().Should().BeLessThanOrEqualTo(Fixed64.One + Tolerance);
    }

    private static LSMeshCollider Quad(int variant) => new(variant == 2 ? new[]
        { new Vector3d(0, -1, -1), new Vector3d(0, 1, -1), new Vector3d(0, -1, 1), new Vector3d(0, 1, 1), Vector3d.Zero }
            : new[] { new Vector3d(0, -1, -1), new Vector3d(0, 1, -1), new Vector3d(0, -1, 1), new Vector3d(0, 1, 1) },
        variant == 1 ? new[] { 3, 2, 1, 1, 2, 0 }
            : variant == 2 ? new[] { 0, 1, 4, 1, 3, 4, 3, 2, 4, 2, 0, 4 }
            : new[] { 0, 2, 1, 1, 2, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
}
