using FixedMathSharp;
using FixedMathSharp.Bounds;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class FiniteAxisProjectionWorkerTests
{
    [Fact]
    public void CapsuleWorkers_WithBentLegacyAxisAtScalarFace_ShouldRejectPhysicalMiss()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCapsuleCollider capsule = CreateCapsuleAtScalarFace(context);
        var start = new Vector3d(Fixed64.MaxValue, (Fixed64)7, Fixed64.Zero);
        var end = new Vector3d(Fixed64.MaxValue, (Fixed64)8, Fixed64.Zero);
        var raycast = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();
        raycast.PrepareSegmentCheck(start, end);
        var sweep = new SweptSphereQueryWorker();
        sweep.Prepare(start, end, Fixed64.Quarter);

        raycast.CheckCapsuleOverlaps(capsule, ref hits).Should().BeFalse();
        hits.Should().BeEmpty();
        sweep.TrySweep(capsule, out _, out _).Should().BeFalse();
    }

    [Fact]
    public void Capsule2DQueries_WithBentLegacyAxisAtScalarFace_ShouldRejectPhysicalMiss()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        LSCapsuleCollider2D capsule = CreateCapsule2DAtScalarFace(context);
        var start = new Vector2d(Fixed64.MaxValue, (Fixed64)7);
        var end = new Vector2d(Fixed64.MaxValue, (Fixed64)8);

        capsule.ContainsPoint(start).Should().BeFalse();
        QueryDetection2D.TryRaycast(start, end, capsule, out _).Should().BeFalse();
        QueryDetection2D.TrySweepCircle(start, end, Fixed64.Quarter, capsule, out _).Should().BeFalse();
    }

    [Fact]
    public void MixedCapsuleReducer_WithUnrepresentableConceptualAxis_ShouldKeepCenteredStartAdmission()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        LSCapsuleCollider2D capsule = CreateCapsule2DAtScalarFace(context);
        Vector2d planarStart = capsule.Center
            + capsule.WorldAxis * Fixed64.FromFraction(27, 4);
        capsule.ContainsPoint(planarStart).Should().BeTrue();
        var legacyAxis = new FixedMathSharp.Bounds.FixedSegment2d(
            capsule.SegmentStart,
            capsule.SegmentEnd);
        Vector2d legacyClosest = legacyAxis.ClosestPoint(planarStart);
        Vector2d.TryGetDistance(planarStart, legacyClosest, out Fixed64 legacyDistance).Should().BeTrue();
        legacyDistance.Should().BeGreaterThan(capsule.ScaledRadius + Fixed64.Quarter);

        var containedStart = new Vector3d(planarStart.X, Fixed64.Zero, planarStart.Y);
        var containedEnd = containedStart - Vector3d.Forward;
        GravitasQueryMixedService.TrySweepSphereAgainstCapsuleSlab(
            containedStart,
            containedEnd,
            -Vector3d.Forward,
            Fixed64.One,
            Fixed64.Quarter,
            capsule,
            out PhysicsMixedHit containedHit).Should().BeTrue();
        containedHit.Distance.Should().Be(Fixed64.Zero);
        containedHit.Collider2D.Should().BeSameAs(capsule);
    }

    [Fact]
    public void CompoundOverlap_WithUnrepresentableCapsuleAxisPoint_ShouldKeepClosestPartOrdering()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(
                Fixed64.Half,
                new Vector2d((Fixed64)3, (Fixed64)8)),
            CompoundColliderPart2D.Capsule(
                (Fixed64)2,
                (Fixed64)24,
                Vector2d.Zero,
                -Fixed64.PiOver4,
                Vector2d.One));
        compound.InitializeWithNoBody(new TestMatterAgent(context, transform));
        transform.TrySetWorldPosition(
            new Vector3d(Fixed64.MaxValue - (Fixed64)5, Fixed64.Zero, Fixed64.Zero)).Should().BeTrue();
        compound.RebuildRuntimeShapeOnly().Should().BeTrue();
        var queryCenter = new Vector2d(Fixed64.MaxValue - Fixed64.One, (Fixed64)9);
        var circle = (LSCircleCollider2D)compound.GetPartCollider(0);
        var capsule = (LSCapsuleCollider2D)compound.GetPartCollider(1);
        Vector2d capsuleNormal = capsule.GetNormalFromCenteredAxis(queryCenter);

        capsule.TryGetSurfacePointFromCenteredAxis(
            queryCenter,
            capsuleNormal,
            out _).Should().BeFalse();
        Fixed64 capsuleDistance = FixedSegment2d.GetDistanceToCenteredCapsule(
            queryCenter,
            capsule.Center,
            capsule.WorldAxis,
            capsule.AxisHalfLength,
            capsule.ScaledRadius);
        QueryDetection2D.TryOverlapCircle(
            queryCenter,
            (Fixed64)2,
            circle,
            out Physics2DHit expectedCircleHit).Should().BeTrue();
        capsuleDistance.Should().BeGreaterThan(expectedCircleHit.Distance);

        QueryDetection2D.TryOverlapCircle(
            queryCenter,
            (Fixed64)2,
            compound,
            out Physics2DHit hit).Should().BeTrue();

        hit.Collider.Should().BeSameAs(compound);
        hit.Distance.Should().Be(expectedCircleHit.Distance);
    }

    [Fact]
    public void SweptSphereCapsuleStartOverlap_ShouldReportTargetSurfacePoint()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCapsuleCollider capsule = CreateOrdinaryCapsule(context);

        Vector3d point = ContinuousCollisionContactPolicy.ResolveSweptSpherePoint(
            capsule,
            capsule.Center,
            Vector3d.Right,
            Fixed64.Quarter);

        point.Should().Be(capsule.Center + Vector3d.Right * capsule.ScaledRadius);
    }

    [Fact]
    public void SweptSphereCapsuleStartOverlap_WithRoundedRotatedFallback_ShouldReportNormalizedSurface()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Up,
            Fixed64.FromFraction(1472, 997));
        LSCapsuleCollider capsule = CreateOrdinaryCapsule(context, rotation);
        Vector3d roundedFallback = rotation * Vector3d.Right;
        roundedFallback.IsNormalized().Should().BeFalse();

        Vector3d normal = capsule.GetNormalAtPoint(capsule.Center);
        Vector3d point = ContinuousCollisionContactPolicy.ResolveSweptSpherePoint(
            capsule,
            capsule.Center,
            Vector3d.Forward,
            Fixed64.Quarter);

        normal.IsNormalized().Should().BeTrue();
        point.Should().Be(capsule.Center + normal * capsule.ScaledRadius);
    }

    [Fact]
    public void CapsuleClosestPoint_InsideTopHemisphere_ShouldProjectToRoundedSurface()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCapsuleCollider capsule = CreateOrdinaryCapsule(context);
        Vector3d pointInsideTopHemisphere = capsule.LineSegmentEnd + Vector3d.Right * Fixed64.Quarter;

        Vector3d closest = capsule.ClosestPointOnSurface(pointInsideTopHemisphere);

        closest.Should().Be(capsule.LineSegmentEnd + Vector3d.Right * capsule.ScaledRadius);
    }

    [Fact]
    public void Capsule2DClosestPoint_OnCenterAxis_ShouldUseStableRotatedRadialDirection()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var capsule = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3);
        capsule.InitializeWithNoBody(new TestMatterAgent(
            context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));

        Vector2d closest = capsule.GetClosestPoint(capsule.Center);

        closest.Should().Be(capsule.Center + Vector2d.Right * capsule.ScaledRadius);
    }

    [Fact]
    public void SweptSphereCapsulePoint_WithUnrepresentableTargetSurface_ShouldUseSphereFallback()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCapsuleCollider capsule = CreateCapsuleAtScalarFace(context);
        var sphereCenter = new Vector3d(Fixed64.MaxValue - Fixed64.One, (Fixed64)9, Fixed64.Zero);
        Vector3d normal = capsule.GetNormalAtPoint(sphereCenter);

        FixedSegment.TryGetSurfacePointOnCenteredCapsule(
            sphereCenter,
            capsule.Center,
            capsule.WorldAxis,
            capsule.AxisHalfLength,
            capsule.ScaledRadius,
            normal,
            out _).Should().BeFalse();

        Vector3d point = ContinuousCollisionContactPolicy.ResolveSweptSpherePoint(
            capsule,
            sphereCenter,
            Vector3d.Forward,
            Fixed64.Quarter);

        point.Should().Be(sphereCenter - normal * Fixed64.Quarter);
    }

    [Fact]
    public void Capsule2DRaycast_WithRepresentableTangentAtScalarFace_ShouldRetainRayContact()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var transform = new FixedTransform(
            new Vector3d(Fixed64.MaxValue - Fixed64.One, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)15, Fixed64.Zero),
            Vector3d.One);
        var capsule = new LSCapsuleCollider2D(Fixed64.One, (Fixed64)22);
        capsule.InitializeWithNoBody(new TestMatterAgent(context, transform));
        var start = new Vector2d(Fixed64.MaxValue, (Fixed64)(-18));
        var end = new Vector2d(Fixed64.MaxValue, (Fixed64)18);

        QueryDetection2D.TryRaycast(
            start,
            end,
            capsule,
            out Physics2DHit hit).Should().BeTrue();

        capsule.TryGetSurfacePointFromCenteredAxis(
            hit.Point,
            hit.Normal,
            out Vector2d projectedSurface).Should().BeTrue();
        projectedSurface.Should().Be(hit.Point);
        hit.Point.X.Should().Be(Fixed64.MaxValue);
        hit.Normal.X.Should().BeGreaterThan(Fixed64.Zero);
        hit.Distance.Should().BeGreaterThan(Fixed64.Zero);
        hit.Distance.Should().BeLessThan((Fixed64)36);
    }

    [Fact]
    public void Capsule2DRaycast_AtScalarFace_ShouldPreserveRayWitness()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        LSCapsuleCollider2D capsule = CreateCapsule2DAtScalarFace(context);
        var start = new Vector2d(Fixed64.MaxValue, (Fixed64)12);
        var end = new Vector2d(Fixed64.MaxValue, Fixed64.Zero);

        QueryDetection2D.TryRaycast(
            start,
            end,
            capsule,
            out Physics2DHit hit).Should().BeTrue();

        hit.Point.Should().Be(new FixedSegment2d(start, end).GetPointAtDistance(
            hit.Distance,
            (start.Y - end.Y).Abs()));
    }

    [Fact]
    public void Capsule2DSweep_EnteringScalarFaceProjection_ShouldUseRepresentableCircleContact()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        LSCapsuleCollider2D capsule = CreateCapsule2DAtScalarFace(context);
        var start = new Vector2d(Fixed64.MaxValue, (Fixed64)12);
        var end = new Vector2d(Fixed64.MaxValue, Fixed64.Zero);
        Fixed64 queryRadius = (Fixed64)2;

        QueryDetection2D.TrySweepCircle(
            start,
            end,
            queryRadius,
            capsule,
            out Physics2DHit hit).Should().BeTrue();

        capsule.TryGetSurfacePointFromCenteredAxis(
            hit.Point + hit.Normal * queryRadius,
            hit.Normal,
            out _).Should().BeFalse();
        hit.Distance.Should().BeGreaterThan(Fixed64.Zero);
        var sweep = new FixedSegment2d(start, end);
        Vector2d sweptCenter = sweep.GetPointAtDistance(
            hit.Distance,
            (end.Y - start.Y).Abs());
        hit.Point.Should().Be(sweptCenter - hit.Normal * queryRadius);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void MixedCapsuleSlab_WithUnrepresentableVerticalGap_ShouldReject(bool sphereAbove)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        Fixed64 capsuleY = sphereAbove ? -Fixed64.One : Fixed64.One;
        Fixed64 sphereY = sphereAbove ? Fixed64.MaxValue : Fixed64.MinValue;
        var capsule = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3);
        capsule.InitializeWithNoBody(new TestMatterAgent(
            context,
            new FixedTransform(
                new Vector3d(Fixed64.Zero, capsuleY, Fixed64.Zero),
                FixedQuaternion.Identity,
                Vector3d.One)));
        var sphereCenter = new Vector3d(Fixed64.Zero, sphereY, Fixed64.Zero);

        GravitasQueryMixedService.TrySweepSphereAgainstCapsuleSlab(
            sphereCenter,
            sphereCenter,
            Vector3d.Zero,
            Fixed64.Zero,
            Fixed64.Half,
            capsule,
            out PhysicsMixedHit hit).Should().BeFalse();

        hit.Should().Be(default(PhysicsMixedHit));
    }

    [Fact]
    public void FiniteSlabCapsuleProjection_WithEqualEndpointSupports_ShouldKeepStableHit()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCapsuleCollider capsule = CreateOrdinaryCapsule(
            context,
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)(-90)));

        bool found = FiniteSlabProjectionSweep.TrySweepCircleAgainstCapsule(
            new Vector2d(Fixed64.Zero, (Fixed64)(-3)),
            Vector2d.Forward,
            (Fixed64)6,
            Fixed64.Zero,
            -Fixed64.Half,
            Fixed64.Half,
            capsule,
            out Fixed64 distance);

        found.Should().BeTrue();
        (distance - (Fixed64)2.5m).Abs().Should().BeLessThan(Fixed64.FromFraction(1, 100_000));
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void SweptSphereWorker_WithCylinderAtScalarEdge_ShouldKeepPhysicalExpandedCap(bool positive)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCylinderCollider cylinder = CreateCylinderAtScalarEdge(context, positive);
        Vector3d start = ScalarEdgePoint(positive, outsideOffset: 5);
        Vector3d end = ScalarEdgePoint(positive, outsideOffset: 4);
        var worker = new SweptSphereQueryWorker();
        worker.Prepare(start, end, Fixed64.One);

        bool hit = worker.TrySweep(cylinder, out Vector3d centerAtImpact, out Fixed64 distance);

        hit.Should().BeTrue();
        centerAtImpact.Should().Be(end);
        distance.Should().Be(Fixed64.One);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void RaycastWorker_WithCylinderAtScalarEdge_ShouldKeepPhysicalAuthoredCap(bool positive)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCylinderCollider cylinder = CreateCylinderAtScalarEdge(context, positive);
        Vector3d start = ScalarEdgePoint(positive, outsideOffset: 4);
        Vector3d end = ScalarEdgePoint(positive, outsideOffset: 3);
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();
        worker.PrepareSegmentCheck(start, end);

        bool hit = worker.CheckCylinderOverlaps(cylinder, ref hits);

        hit.Should().BeTrue();
        hits.Should().ContainSingle().Which.Should().Be(end);
    }

    private static LSCylinderCollider CreateCylinderAtScalarEdge(
        GravitasWorldContext context,
        bool positive)
    {
        Fixed64 centerY = positive
            ? Fixed64.MaxValue - Fixed64.One
            : Fixed64.MinValue + Fixed64.One;
        var transform = new FixedTransform(
            new Vector3d(Fixed64.Zero, centerY, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);
        var collider = new LSCylinderCollider
        {
            Size = new Vector3d(Fixed64.One, (Fixed64)4, Fixed64.One)
        };
        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
        return collider;
    }

    private static LSCapsuleCollider CreateCapsuleAtScalarFace(GravitasWorldContext context)
    {
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)(-45)),
            Vector3d.One);
        var collider = new LSCapsuleCollider
        {
            Radius = Fixed64.One,
            Size = new Vector3d((Fixed64)2, (Fixed64)22, (Fixed64)2)
        };
        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
        transform.TrySetWorldPosition(
            new Vector3d(Fixed64.MaxValue - (Fixed64)5, Fixed64.Zero, Fixed64.Zero)).Should().BeTrue();
        collider.RebuildRuntimeShapeOnly(refreshMassProperties: false).Should().BeTrue();
        return collider;
    }

    private static LSCapsuleCollider CreateOrdinaryCapsule(GravitasWorldContext context) =>
        CreateOrdinaryCapsule(context, FixedQuaternion.Identity);

    private static LSCapsuleCollider CreateOrdinaryCapsule(
        GravitasWorldContext context,
        FixedQuaternion rotation)
    {
        var collider = new LSCapsuleCollider
        {
            Radius = Fixed64.Half,
            Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One)
        };
        collider.InitializeWithNoBody(new TestMatterAgent(
            context,
            new FixedTransform(Vector3d.Zero, rotation, Vector3d.One)));
        return collider;
    }

    private static LSCapsuleCollider2D CreateCapsule2DAtScalarFace(GravitasWorldContext context)
    {
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)45, Fixed64.Zero),
            Vector3d.One);
        var collider = new LSCapsuleCollider2D(Fixed64.One, (Fixed64)22);
        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
        transform.TrySetWorldPosition(
            new Vector3d(Fixed64.MaxValue - (Fixed64)5, Fixed64.Zero, Fixed64.Zero)).Should().BeTrue();
        collider.RebuildRuntimeShapeOnly().Should().BeTrue();
        collider.WorldAxis.X.Should().BeGreaterThan(Fixed64.Zero);
        collider.WorldAxis.Y.Should().BeGreaterThan(Fixed64.Zero);
        return collider;
    }

    private static Vector3d ScalarEdgePoint(bool positive, int outsideOffset) =>
        new(
            Fixed64.Zero,
            positive
                ? Fixed64.MaxValue - (Fixed64)outsideOffset
                : Fixed64.MinValue + (Fixed64)outsideOffset,
            Fixed64.Zero);
}
