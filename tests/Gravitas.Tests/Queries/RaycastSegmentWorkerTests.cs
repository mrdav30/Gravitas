using FixedMathSharp;
using FixedMathSharp.Bounds;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class RaycastSegmentWorkerTests
{
    [Theory]
    [MemberData(nameof(BoxBoundaryPoints))]
    public void CheckAABBoxOverlaps_WithPointInsideOrOnBoundary_ShouldReturnPoint(Vector3d point)
    {
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(point, point);

        bool hit = worker.CheckAABBoxOverlaps(
            new Vector3d((Fixed64)(-1), (Fixed64)(-1), (Fixed64)(-1)),
            new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One),
            ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(1);
        hits[0].Should().Be(point);
    }

    [Theory]
    [InlineData(-2, 0, 0)]
    [InlineData(2, 0, 0)]
    [InlineData(0, -2, 0)]
    [InlineData(0, 2, 0)]
    [InlineData(0, 0, -2)]
    [InlineData(0, 0, 2)]
    public void CheckAABBoxOverlaps_WithPointOutsideBox_ShouldReturnFalse(int x, int y, int z)
    {
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();
        Vector3d point = new((Fixed64)x, (Fixed64)y, (Fixed64)z);

        worker.PrepareSegmentCheck(point, point);

        bool hit = worker.CheckAABBoxOverlaps(
            new Vector3d((Fixed64)(-1), (Fixed64)(-1), (Fixed64)(-1)),
            new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One),
            ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckAABBoxOverlaps_WithPointInsideAndIntersectionsDisabled_ShouldNotWritePoint()
    {
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(Vector3d.Zero, Vector3d.Zero, calculateIntersectionPoints: false);

        bool hit = worker.CheckAABBoxOverlaps(
            new Vector3d((Fixed64)(-1), (Fixed64)(-1), (Fixed64)(-1)),
            new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One),
            ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckSphereOverlaps_WithPointInsideSphere_ShouldReturnPoint()
    {
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(Vector3d.Zero, Vector3d.Zero);

        bool hit = worker.CheckSphereOverlaps(Sphere(Vector3d.Zero, Fixed64.One), ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(1);
        hits[0].Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void CheckSphereOverlaps_WithPointOutsideSphere_ShouldReturnFalse()
    {
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));

        bool hit = worker.CheckSphereOverlaps(Sphere(Vector3d.Zero, Fixed64.One), ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckSphereOverlaps_WithSegmentIntersectionsDisabled_ShouldReturnTrueWithoutPoints()
    {
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            calculateIntersectionPoints: false);

        bool hit = worker.CheckSphereOverlaps(Sphere(Vector3d.Zero, Fixed64.One), ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckSphereOverlaps_WithSegmentStartingInsideSphere_ShouldReturnOriginOnly()
    {
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));

        bool hit = worker.CheckSphereOverlaps(Sphere(Vector3d.Zero, Fixed64.One), ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(1);
        hits[0].Should().Be(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
    }

    [Fact]
    public void CheckSphereOverlaps_WithTangentSegment_ShouldReturnSingleIntersection()
    {
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d((Fixed64)(-2), Fixed64.One, Fixed64.Zero),
            new Vector3d((Fixed64)2, Fixed64.One, Fixed64.Zero));

        bool hit = worker.CheckSphereOverlaps(Sphere(Vector3d.Zero, Fixed64.One), ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(1);
        hits[0].Should().Be(new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));
    }

    [Fact]
    public void CheckSphereOverlaps_WithQuantizedNearTangent_ShouldReturnFalse()
    {
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();
        worker.PrepareSegmentCheck(
            Vector3d.Zero,
            new Vector3d((Fixed64)3, (Fixed64)4, Fixed64.Zero));

        bool hit = worker.CheckSphereOverlaps(
            Sphere(
                new Vector3d(
                    Fixed64.FromFraction(1, 5),
                    Fixed64.FromFraction(3, 10),
                    Fixed64.Zero),
                Fixed64.FromFraction(1, 50)),
            ref hits);

        hit.Should().BeFalse();
        hits.Should().BeEmpty();
    }

    [Fact]
    public void CheckSphereOverlaps_WithPointInsideAndIntersectionsDisabled_ShouldNotWritePoint()
    {
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(Vector3d.Zero, Vector3d.Zero, calculateIntersectionPoints: false);

        bool hit = worker.CheckSphereOverlaps(Sphere(Vector3d.Zero, Fixed64.One), ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckSphereOverlaps_WithNonzeroSegmentWhoseSquaredLengthRoundsToZero_ShouldUseSegment()
    {
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();
        Vector3d end = new(Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero);
        worker.PrepareSegmentCheck(Vector3d.Zero, end);

        bool hit = worker.CheckSphereOverlaps(Sphere(end, Fixed64.Zero), ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(1);
        hits[0].Should().Be(end);
    }

    [Fact]
    public void CheckSphereOverlaps_WithExitPastSegmentEnd_ShouldReturnEntryOnly()
    {
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));

        bool hit = worker.CheckSphereOverlaps(Sphere(Vector3d.Zero, Fixed64.One), ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(1);
        hits[0].Should().Be(new Vector3d(Fixed64.FromRaw(-4_294_967_297L), Fixed64.Zero, Fixed64.Zero));
    }

    [Fact]
    public void CheckSphereOverlaps_WithExtremeOffAxisCrossing_ShouldReturnExactInterval()
    {
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();
        worker.PrepareSegmentCheck(new Vector3d(-200_000, 0, 0), new Vector3d(200_000, 0, 0));

        bool hit = worker.CheckSphereOverlaps(
            Sphere(
                new Vector3d(Fixed64.Zero, (Fixed64)60_000, Fixed64.Zero),
                (Fixed64)100_000),
            ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(2);
        // The exact roots are +/-80,000. The worker publishes points from the
        // independently nearest-even segment parameters, so a 400,000-unit
        // segment deterministically amplifies that sub-raw parameter rounding.
        hits[0].Should().Be(new Vector3d(Fixed64.FromRaw(-343_597_383_600_000L), Fixed64.Zero, Fixed64.Zero));
        hits[1].Should().Be(new Vector3d(Fixed64.FromRaw(343_597_383_600_000L), Fixed64.Zero, Fixed64.Zero));
    }

    [Fact]
    public void CheckSphereOverlaps_WithBoundaryAtSegmentEnd_ShouldReturnAuthoredEndpoint()
    {
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();
        Vector3d end = Vector3d.Up;
        worker.PrepareSegmentCheck(new Vector3d(-2, -1, 0), end);

        bool hit = worker.CheckSphereOverlaps(Sphere(Vector3d.Zero, Fixed64.One), ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(2);
        hits[1].Should().Be(end);
    }

    [Fact]
    public void CheckCylinderOverlaps_WithPointInsideFiniteCylinder_ShouldReturnPoint()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCylinderCollider cylinder = scenario.CreateCylinder(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(Vector3d.Zero, Vector3d.Zero);

        bool hit = worker.CheckCylinderOverlaps(cylinder, ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(1);
        hits[0].Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void CheckCylinderOverlaps_WithPointOutsideFiniteCylinder_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCylinderCollider cylinder = scenario.CreateCylinder(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));

        bool hit = worker.CheckCylinderOverlaps(cylinder, ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckCylinderOverlaps_WithShortVerticalSegmentBeforeCaps_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCylinderCollider cylinder = scenario.CreateCylinder(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero));

        bool hit = worker.CheckCylinderOverlaps(cylinder, ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckCapsuleOverlaps_WithSegmentHittingStartHemisphereAfterMissingCylinder_ShouldReturnCapHit()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCapsuleCollider capsule = scenario.CreateCapsule(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d((Fixed64)(-2), capsule.LineSegmentStart.Y, Fixed64.Zero),
            new Vector3d((Fixed64)2, capsule.LineSegmentStart.Y, Fixed64.Zero));

        bool hit = worker.CheckCapsuleOverlaps(capsule, ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(2);
        hits.Should().Contain(new Vector3d(-capsule.ScaledRadius, capsule.LineSegmentStart.Y, Fixed64.Zero));
        hits.Should().Contain(new Vector3d(capsule.ScaledRadius, capsule.LineSegmentStart.Y, Fixed64.Zero));
    }

    [Fact]
    public void CheckCapsuleOverlaps_WithSegmentHittingHemisphereAfterMissingCylinder_ShouldReturnCapHit()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCapsuleCollider capsule = scenario.CreateCapsule(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d((Fixed64)(-2), capsule.LineSegmentEnd.Y, Fixed64.Zero),
            new Vector3d((Fixed64)2, capsule.LineSegmentEnd.Y, Fixed64.Zero));

        bool hit = worker.CheckCapsuleOverlaps(capsule, ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(2);
        hits.Should().Contain(new Vector3d(-capsule.ScaledRadius, capsule.LineSegmentEnd.Y, Fixed64.Zero));
        hits.Should().Contain(new Vector3d(capsule.ScaledRadius, capsule.LineSegmentEnd.Y, Fixed64.Zero));
    }

    [Fact]
    public void CheckCapsuleOverlaps_WithSegmentMissingCylinderAndHemispheres_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCapsuleCollider capsule = scenario.CreateCapsule(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, (Fixed64)2),
            new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)2));

        bool hit = worker.CheckCapsuleOverlaps(capsule, ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckCylinderOverlaps_WithSegmentStartingInsideAndIntersectionsDisabled_ShouldNotWritePoint()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCylinderCollider cylinder = scenario.CreateCylinder(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            Vector3d.Zero,
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            calculateIntersectionPoints: false);

        bool hit = worker.CheckCylinderOverlaps(cylinder, ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckCylinderOverlaps_WithSegmentCrossingCylinderSide_ShouldReturnEntryAndExit()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCylinderCollider cylinder = scenario.CreateCylinder(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));

        bool hit = worker.CheckCylinderOverlaps(cylinder, ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(2);
        hits.Should().Contain(new Vector3d(-cylinder.ScaledRadius, Fixed64.Zero, Fixed64.Zero));
        hits.Should().Contain(new Vector3d(cylinder.ScaledRadius, Fixed64.Zero, Fixed64.Zero));
    }

    [Fact]
    public void CheckCylinderOverlaps_WithSegmentParallelToAxisOutsideRadius_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCylinderCollider cylinder = scenario.CreateCylinder(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d((Fixed64)2, (Fixed64)(-2), Fixed64.Zero),
            new Vector3d((Fixed64)2, (Fixed64)2, Fixed64.Zero));

        bool hit = worker.CheckCylinderOverlaps(cylinder, ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckCylinderOverlaps_WithSegmentMissingSideAtFiniteHeight_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCylinderCollider cylinder = scenario.CreateCylinder(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, (Fixed64)2),
            new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)2));

        bool hit = worker.CheckCylinderOverlaps(cylinder, ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckCylinderOverlaps_WithTangentSideSegment_ShouldReturnSingleIntersection()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCylinderCollider cylinder = scenario.CreateCylinder(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, cylinder.ScaledRadius),
            new Vector3d((Fixed64)2, Fixed64.Zero, cylinder.ScaledRadius));

        bool hit = worker.CheckCylinderOverlaps(cylinder, ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(1);
        hits[0].Should().Be(new Vector3d(Fixed64.Zero, Fixed64.Zero, cylinder.ScaledRadius));
    }

    [Fact]
    public void CheckCylinderOverlaps_WithSideIntersectionsOutsideSegment_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCylinderCollider cylinder = scenario.CreateCylinder(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));

        bool hit = worker.CheckCylinderOverlaps(cylinder, ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckCylinderOverlaps_WithSegmentCrossingCapOutsideRadius_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCylinderCollider cylinder = scenario.CreateCylinder(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d((Fixed64)2, (Fixed64)(-2), Fixed64.Zero),
            new Vector3d((Fixed64)2, (Fixed64)2, Fixed64.Zero));

        bool hit = worker.CheckCylinderOverlaps(cylinder, ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckCylinderOverlaps_WithSegmentCrossingCapInsideRadius_ShouldReturnCapIntersections()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCylinderCollider cylinder = scenario.CreateCylinder(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d(Fixed64.Half, (Fixed64)(-2), Fixed64.Zero),
            new Vector3d(Fixed64.Half, (Fixed64)2, Fixed64.Zero));

        bool hit = worker.CheckCylinderOverlaps(cylinder, ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(2);
        hits.Should().Contain(new Vector3d(Fixed64.Half, -cylinder.HalfHeight, Fixed64.Zero));
        hits.Should().Contain(new Vector3d(Fixed64.Half, cylinder.HalfHeight, Fixed64.Zero));
    }

    [Fact]
    public void CheckConeOverlaps_WithPointInsideCone_ShouldReturnPoint()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSConeCollider cone = scenario.CreateCone(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(Vector3d.Zero, Vector3d.Zero);

        bool hit = worker.CheckConeOverlaps(cone, ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(1);
        hits[0].Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void CheckConeOverlaps_WithPointInsideAndIntersectionsDisabled_ShouldNotWritePoint()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSConeCollider cone = scenario.CreateCone(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(Vector3d.Zero, Vector3d.Zero, calculateIntersectionPoints: false);

        bool hit = worker.CheckConeOverlaps(cone, ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckConeOverlaps_WithPointOutsideCone_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSConeCollider cone = scenario.CreateCone(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));

        bool hit = worker.CheckConeOverlaps(cone, ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckConeOverlaps_WithSegmentStartingInsideAndIntersectionsDisabled_ShouldNotWritePoint()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSConeCollider cone = scenario.CreateCone(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            Vector3d.Zero,
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            calculateIntersectionPoints: false);

        bool hit = worker.CheckConeOverlaps(cone, ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckConeOverlaps_WithSegmentCrossingConeSide_ShouldReturnSideIntersections()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSConeCollider cone = scenario.CreateCone(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));

        bool hit = worker.CheckConeOverlaps(cone, ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(2);
        hits.Should().Contain(new Vector3d(-Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero));
        hits.Should().Contain(new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero));
    }

    [Fact]
    public void CheckConeOverlaps_WithSegmentCrossingConeBase_ShouldReturnBaseIntersection()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSConeCollider cone = scenario.CreateCone(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d(Fixed64.Zero, (Fixed64)(-2), Fixed64.Zero),
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero));

        bool hit = worker.CheckConeOverlaps(cone, ref hits);

        hit.Should().BeTrue();
        hits.Should().Contain(new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero));
    }

    [Fact]
    public void CheckConeOverlaps_WithSegmentStartingInsideCone_ShouldReturnOriginOnly()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSConeCollider cone = scenario.CreateCone(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));

        bool hit = worker.CheckConeOverlaps(cone, ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(1);
        hits[0].Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void CheckConeOverlaps_WithVerticalSegmentOutsideBaseRadius_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSConeCollider cone = scenario.CreateCone(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d((Fixed64)2, (Fixed64)(-2), Fixed64.Zero),
            new Vector3d((Fixed64)2, (Fixed64)2, Fixed64.Zero));

        bool hit = worker.CheckConeOverlaps(cone, ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckConeOverlaps_WithHorizontalSegmentAboveApex_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSConeCollider cone = scenario.CreateCone(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d((Fixed64)(-2), (Fixed64)2, Fixed64.Zero),
            new Vector3d((Fixed64)2, (Fixed64)2, Fixed64.Zero));

        bool hit = worker.CheckConeOverlaps(cone, ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckConeOverlaps_WithHorizontalSegmentBelowBase_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSConeCollider cone = scenario.CreateCone(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();
        Fixed64 y = -cone.HalfHeight - cone.Height;

        worker.PrepareSegmentCheck(
            new Vector3d(-cone.ScaledRadius * (Fixed64)3, y, Fixed64.Zero),
            new Vector3d(cone.ScaledRadius * (Fixed64)3, y, Fixed64.Zero));

        bool hit = worker.CheckConeOverlaps(cone, ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckConeOverlaps_WithSegmentThroughBaseRim_ShouldSuppressDuplicatePoint()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSConeCollider cone = scenario.CreateCone(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d(cone.ScaledRadius, -cone.HalfHeight - cone.Height, Fixed64.Zero),
            new Vector3d(cone.ScaledRadius, cone.HalfHeight + cone.Height, Fixed64.Zero));

        bool hit = worker.CheckConeOverlaps(cone, ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(1);
        hits[0].Should().Be(new Vector3d(cone.ScaledRadius, -cone.HalfHeight, Fixed64.Zero));
    }

    [Fact]
    public void CheckConeOverlaps_WithIntersectionsDisabled_ShouldReturnTrueWithoutPoints()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSConeCollider cone = scenario.CreateCone(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d(cone.ScaledRadius, -cone.HalfHeight - cone.Height, Fixed64.Zero),
            new Vector3d(cone.ScaledRadius, cone.HalfHeight + cone.Height, Fixed64.Zero),
            calculateIntersectionPoints: false);

        bool hit = worker.CheckConeOverlaps(cone, ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckConeOverlaps_WithHorizontalSegmentMissingMidsection_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSConeCollider cone = scenario.CreateCone(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, (Fixed64)2),
            new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)2));

        bool hit = worker.CheckConeOverlaps(cone, ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckConeOverlaps_WithGeneratorParallelSegmentOutsideCone_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSConeCollider cone = scenario.CreateCone(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d((Fixed64)(-3), -cone.HalfHeight, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, cone.HalfHeight, Fixed64.Zero));

        bool hit = worker.CheckConeOverlaps(cone, ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckConeOverlaps_WithExactGeneratorSlopeSegmentBelowCone_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSConeCollider cone = scenario.CreateCone(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d(-cone.ScaledRadius * (Fixed64)2, -cone.HalfHeight - cone.Height, Fixed64.Zero),
            new Vector3d(-cone.ScaledRadius * Fixed64.FromFraction(3, 2), -cone.HalfHeight - cone.Height * Fixed64.Half, Fixed64.Zero));

        bool hit = worker.CheckConeOverlaps(cone, ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckConeOverlaps_WithOffsetGeneratorSlopeSegmentBelowCone_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSConeCollider cone = scenario.CreateCone(Vector3d.Zero).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();
        Fixed64 offset = cone.ScaledRadius * Fixed64.FromFraction(1, 4);

        worker.PrepareSegmentCheck(
            new Vector3d(-cone.ScaledRadius * (Fixed64)2 - offset, -cone.HalfHeight - cone.Height, Fixed64.Zero),
            new Vector3d(-cone.ScaledRadius * Fixed64.FromFraction(3, 2) - offset, -cone.HalfHeight - cone.Height * Fixed64.Half, Fixed64.Zero));

        bool hit = worker.CheckConeOverlaps(cone, ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckAABBoxOverlaps_WithSegmentCrossingBox_ShouldReturnEntryAndExit()
    {
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));

        bool hit = worker.CheckAABBoxOverlaps(
            new Vector3d((Fixed64)(-1), (Fixed64)(-1), (Fixed64)(-1)),
            new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One),
            ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(2);
        hits[0].Should().Be(new Vector3d((Fixed64)(-1), Fixed64.Zero, Fixed64.Zero));
        hits[1].Should().Be(new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero));
    }

    [Fact]
    public void CheckAABBoxOverlaps_WithNonzeroSegmentStartingInside_ShouldReturnOriginAndBoxExit()
    {
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();
        worker.PrepareSegmentCheck(Vector3d.Zero, Vector3d.Right * (Fixed64)3);

        bool hit = worker.CheckAABBoxOverlaps(-Vector3d.One, Vector3d.One, ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(2);
        hits[0].Should().Be(Vector3d.Zero);
        hits[1].Should().Be(Vector3d.Right);
    }

    [Fact]
    public void CheckAABBoxOverlaps_WhenSmallestDirectionComponentReachesBoundaryAtEndpoint_ShouldHit()
    {
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();
        Fixed64 smallestIncrement = Fixed64.FromRaw(1);
        Vector3d start = new(-Fixed64.One - smallestIncrement, Fixed64.Zero, -Fixed64.Half);
        Vector3d end = new(-Fixed64.One, Fixed64.Zero, Fixed64.Half);
        worker.PrepareSegmentCheck(start, end);

        bool hit = worker.CheckAABBoxOverlaps(-Vector3d.One, Vector3d.One, ref hits);

        hit.Should().BeTrue();
        hits.Should().ContainSingle().Which.Should().Be(end);
    }

    [Fact]
    public void CheckAABBoxOverlaps_WithSegmentMissAfterAxisClip_ShouldReturnFalse()
    {
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d((Fixed64)(-3), (Fixed64)2, Fixed64.Zero),
            new Vector3d((Fixed64)3, (Fixed64)4, Fixed64.Zero));

        bool hit = worker.CheckAABBoxOverlaps(
            new Vector3d((Fixed64)(-1), (Fixed64)(-1), (Fixed64)(-1)),
            new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One),
            ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckAABBoxOverlaps_WithSegmentCrossingBoxAndIntersectionsDisabled_ShouldNotWritePoints()
    {
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            calculateIntersectionPoints: false);

        bool hit = worker.CheckAABBoxOverlaps(
            new Vector3d((Fixed64)(-1), (Fixed64)(-1), (Fixed64)(-1)),
            new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One),
            ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckOBBoxOverlaps_WithPointInsideRotatedBox_ShouldReturnPoint()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCuboidCollider box = scenario.CreateCuboid(
            Vector3d.Zero,
            PhysicsScenarioBuilder.Yaw(45)).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(Vector3d.Zero, Vector3d.Zero);

        bool hit = worker.CheckOBBoxOverlaps(box, ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(1);
        hits[0].Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void CheckOBBoxOverlaps_WithPointOutsideRotatedBox_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCuboidCollider box = scenario.CreateCuboid(
            Vector3d.Zero,
            PhysicsScenarioBuilder.Yaw(45)).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));

        bool hit = worker.CheckOBBoxOverlaps(box, ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Theory]
    [InlineData(1, 0, 0)]
    [InlineData(-1, 0, 0)]
    [InlineData(0, -1, 0)]
    [InlineData(0, 1, 0)]
    [InlineData(0, 0, -1)]
    [InlineData(0, 0, 1)]
    public void CheckOBBoxOverlaps_WithPointOutsideEachLocalAxis_ShouldReturnFalse(int x, int y, int z)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCuboidCollider box = scenario.CreateCuboid(
            Vector3d.Zero,
            PhysicsScenarioBuilder.Yaw(45)).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();
        Vector3d localPoint = new(
            x == 0 ? Fixed64.Zero : (Fixed64)x,
            y == 0 ? Fixed64.Zero : (Fixed64)y,
            z == 0 ? Fixed64.Zero : (Fixed64)z);
        Vector3d worldPoint = LocalToWorld(box, localPoint);

        worker.PrepareSegmentCheck(worldPoint, worldPoint);

        bool hit = worker.CheckOBBoxOverlaps(box, ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckOBBoxOverlaps_WithSegmentCrossingRotatedBox_ShouldReturnEntryAndExit()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCuboidCollider box = scenario.CreateCuboid(
            Vector3d.Zero,
            PhysicsScenarioBuilder.Yaw(45)).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));

        bool hit = worker.CheckOBBoxOverlaps(box, ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(2);
        hits[0].X.Should().BeLessThan(Fixed64.Zero);
        hits[1].X.Should().BeGreaterThan(Fixed64.Zero);
        hits[0].Y.Should().Be(Fixed64.Zero);
        hits[1].Y.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void CheckOBBoxOverlaps_WithSegmentCrossingRotatedBoxAndIntersectionsDisabled_ShouldNotWritePoints()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCuboidCollider box = scenario.CreateCuboid(
            Vector3d.Zero,
            PhysicsScenarioBuilder.Yaw(45)).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            calculateIntersectionPoints: false);

        bool hit = worker.CheckOBBoxOverlaps(box, ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckOBBoxOverlaps_WithSegmentTangentToRotatedBoxCorner_ShouldReturnSingleIntersection()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCuboidCollider box = scenario.CreateCuboid(
            Vector3d.Zero,
            PhysicsScenarioBuilder.Yaw(45)).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();
        Vector3d localStart = new((Fixed64)(-2), -Fixed64.One, Fixed64.Half);
        Vector3d localEnd = new((Fixed64)2, (Fixed64)3, Fixed64.Half);

        worker.PrepareSegmentCheck(LocalToWorld(box, localStart), LocalToWorld(box, localEnd));

        bool hit = worker.CheckOBBoxOverlaps(box, ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(1);
        Vector3d.Distance(
            hits[0],
            LocalToWorld(box, new Vector3d(-Fixed64.Half, Fixed64.Half, Fixed64.Half)))
            .Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
    }

    [Fact]
    public void CheckOBBoxOverlaps_WithLargeScaleTangentAndCompoundRotation_ShouldReturnSingleIntersection()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 halfExtent = (Fixed64)50_000;
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)23,
            (Fixed64)37,
            (Fixed64)11);
        LSCuboidCollider box = scenario.CreateBody(
            new LSCuboidCollider(),
            Vector3d.Zero,
            rotation,
            immovable: true).Collider;
        box.Size = Vector3d.One * (halfExtent * 2);
        box.RebuildRuntimeShapeOnly();
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();
        Vector3d localStart = new(-halfExtent * 4, -halfExtent * 2, halfExtent);
        Vector3d localEnd = new(halfExtent * 4, halfExtent * 6, halfExtent);
        Vector3d localTangent = new(-halfExtent, halfExtent, halfExtent);

        worker.PrepareSegmentCheck(LocalToWorld(box, localStart), LocalToWorld(box, localEnd));

        bool hit = worker.CheckOBBoxOverlaps(box, ref hits);

        hit.Should().BeTrue();
        hits.Count.Should().Be(1);
        Vector3d.Distance(hits[0], LocalToWorld(box, localTangent))
            .Should().BeLessThanOrEqualTo(Fixed64.FromFraction(1, 1_000));

        hits.FastClear();
        Vector3d outsideOffset = Vector3d.Forward * Fixed64.FromFraction(1, 100);
        worker.PrepareSegmentCheck(
            LocalToWorld(box, localStart + outsideOffset),
            LocalToWorld(box, localEnd + outsideOffset));

        worker.CheckOBBoxOverlaps(box, ref hits).Should().BeFalse();
        hits.Should().BeEmpty();

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(() =>
        {
            hits.FastClear();
            worker.PrepareSegmentCheck(LocalToWorld(box, localStart), LocalToWorld(box, localEnd));
            _ = worker.CheckOBBoxOverlaps(box, ref hits);
        });

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void CheckOBBoxOverlaps_WithSegmentOutsideParallelAxis_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCuboidCollider box = scenario.CreateCuboid(
            Vector3d.Zero,
            PhysicsScenarioBuilder.Yaw(45)).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            new Vector3d((Fixed64)(-3), (Fixed64)2, Fixed64.Zero),
            new Vector3d((Fixed64)3, (Fixed64)2, Fixed64.Zero));

        bool hit = worker.CheckOBBoxOverlaps(box, ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckOBBoxOverlaps_WithSegmentOutsideFirstLocalAxis_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCuboidCollider box = scenario.CreateCuboid(
            Vector3d.Zero,
            PhysicsScenarioBuilder.Yaw(45)).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            LocalToWorld(box, new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)(-2))),
            LocalToWorld(box, new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)2)));

        bool hit = worker.CheckOBBoxOverlaps(box, ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void CheckOBBoxOverlaps_WithSegmentMissingThirdLocalAxis_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCuboidCollider box = scenario.CreateCuboid(
            Vector3d.Zero,
            PhysicsScenarioBuilder.Yaw(45)).Collider;
        var worker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();

        worker.PrepareSegmentCheck(
            LocalToWorld(box, new Vector3d((Fixed64)(-2), Fixed64.Zero, (Fixed64)2)),
            LocalToWorld(box, new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)2)));

        bool hit = worker.CheckOBBoxOverlaps(box, ref hits);

        hit.Should().BeFalse();
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void PrepareSegmentCheck_WithSmallMultiAxisSegment_ShouldKeepUnitDirection()
    {
        var worker = new RaycastSegmentWorker();
        Vector3d end = new(
            Fixed64.FromRaw(300),
            Fixed64.FromRaw(300),
            Fixed64.Zero);

        worker.PrepareSegmentCheck(Vector3d.Zero, end);

        FixedMath.Abs(worker.SegmentDirection.Magnitude - Fixed64.One)
            .Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
    }

    [Fact]
    public void PrepareSegmentCheck_WithInvalidFixedRange_ShouldRejectEveryShapeWorker()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var componentOverflowWorker = new RaycastSegmentWorker();
        var magnitudeOverflowWorker = new RaycastSegmentWorker();
        var hits = new SwiftList<Vector3d>();
        LSMeshCollider mesh = MeshTestFixtures.CreateConvexCube();
        LSConeCollider cone = scenario.CreateCone(Vector3d.Zero).Collider;
        LSCylinderCollider cylinder = scenario.CreateCylinder(Vector3d.Zero).Collider;
        LSCuboidCollider cuboid = scenario.CreateCuboid(Vector3d.Zero).Collider;

        componentOverflowWorker.PrepareSegmentCheck(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero));
        magnitudeOverflowWorker.PrepareSegmentCheck(
            Vector3d.Zero,
            new Vector3d(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.Zero));

        componentOverflowWorker.CheckSphereOverlaps(Sphere(Vector3d.Zero, Fixed64.One), ref hits).Should().BeFalse();
        componentOverflowWorker.CheckConeOverlaps(cone, ref hits).Should().BeFalse();
        componentOverflowWorker.CheckMeshOverlaps(mesh, ref hits).Should().BeFalse();
        componentOverflowWorker.CheckCylinderOverlaps(cylinder, ref hits).Should().BeFalse();
        componentOverflowWorker.CheckAABBoxOverlaps(-Vector3d.One, Vector3d.One, ref hits).Should().BeFalse();
        componentOverflowWorker.CheckOBBoxOverlaps(cuboid, ref hits).Should().BeFalse();
        magnitudeOverflowWorker.CheckSphereOverlaps(Sphere(Vector3d.Zero, Fixed64.One), ref hits).Should().BeFalse();
        hits.Should().BeEmpty();
    }

    public static TheoryData<Vector3d> BoxBoundaryPoints() => new()
    {
        Vector3d.Zero,
        new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero),
        new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero),
        new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One)
    };

    private static Vector3d LocalToWorld(LSCuboidCollider box, Vector3d localPoint) =>
        box.Center + box.Rotation * localPoint;

    private static FixedBoundSphere Sphere(Vector3d center, Fixed64 radius) => new(center, radius);
}
