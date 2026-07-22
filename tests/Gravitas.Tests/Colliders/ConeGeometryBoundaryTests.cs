using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class ConeGeometryBoundaryTests
{
    [Fact]
    public void ContainsWorldPoint_AxialToleranceShouldApplyAtBothCaps()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSConeCollider cone = scenario.CreateBody(
            new LSConeCollider(ColliderShapeDefinition.Cone(Fixed64.One, Fixed64.Two)),
            Vector3d.Zero,
            FixedQuaternion.Identity).Collider;
        Fixed64 outsideOffset = Fixed64.Epsilon + Fixed64.FromRaw(1L);

        cone.ContainsWorldPoint(new Vector3d(Fixed64.Zero, -Fixed64.One - Fixed64.Epsilon, Fixed64.Zero), Fixed64.Epsilon)
            .Should().BeTrue();
        cone.ContainsWorldPoint(new Vector3d(Fixed64.Zero, -Fixed64.One - outsideOffset, Fixed64.Zero), Fixed64.Epsilon)
            .Should().BeFalse();
        cone.ContainsWorldPoint(new Vector3d(Fixed64.Zero, Fixed64.One + Fixed64.Epsilon, Fixed64.Zero), Fixed64.Epsilon)
            .Should().BeTrue();
        cone.ContainsWorldPoint(new Vector3d(Fixed64.Zero, Fixed64.One + outsideOffset, Fixed64.Zero), Fixed64.Epsilon)
            .Should().BeFalse();
    }

    [Fact]
    public void CreateFiniteConeBounds_DegenerateAxis_ShouldUseStableUpFallback()
    {
        ConeGeometry.CreateFiniteConeBounds(
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.Two,
            out Vector3d min,
            out Vector3d max);

        min.Should().Be(new Vector3d(-Fixed64.Two, Fixed64.Zero, -Fixed64.Two));
        max.Should().Be(new Vector3d(Fixed64.Two, Fixed64.Zero, Fixed64.Two));
    }

    [Fact]
    public void CreateFiniteConeBounds_NearUnitAxis_ShouldContainExactBaseDiskSupport()
    {
        var axis = new Vector3d(
            Fixed64.FromFraction(3, 5),
            Fixed64.FromFraction(4, 5) + Fixed64.MinIncrement,
            Fixed64.Zero);
        var baseCenter = new Vector3d(Fixed64.One, Fixed64.Two, (Fixed64)3);

        ConeGeometry.CreateFiniteConeBounds(
            Vector3d.Zero,
            baseCenter,
            axis,
            Fixed64.One,
            out Vector3d min,
            out Vector3d max);

        Vector3d support = baseCenter
            + Vector3d.GetNormalizedProjectionOnPlane(Vector3d.Right, axis);
        min.X.Should().BeLessThanOrEqualTo(support.X);
        max.X.Should().BeGreaterThanOrEqualTo(support.X);
        min.Y.Should().BeLessThanOrEqualTo(support.Y);
        max.Y.Should().BeGreaterThanOrEqualTo(support.Y);
        min.Z.Should().BeLessThanOrEqualTo(support.Z);
        max.Z.Should().BeGreaterThanOrEqualTo(support.Z);
        max.X.Should().BeGreaterThan(baseCenter.X + Fixed64.FromFraction(4, 5));
    }

    [Fact]
    public void GetFiniteConeSupportPoint_NearUnitAxis_ShouldUseExactPlaneRejection()
    {
        var axis = new Vector3d(
            Fixed64.FromFraction(3, 5),
            Fixed64.FromFraction(4, 5) + Fixed64.MinIncrement,
            Fixed64.Zero);
        var ray = new FixedMathSharp.Bounds.FixedRay(Vector3d.Zero, axis);
        ray.TryGetPoint(Fixed64.Two, out Vector3d baseCenter).Should().BeTrue();
        Vector3d radialDirection = Vector3d.GetNormalizedProjectionOnPlane(Vector3d.Right, axis);
        Vector3d expected = baseCenter + radialDirection;

        Vector3d support = ConeGeometry.GetFiniteConeSupportPoint(
            Vector3d.Zero,
            baseCenter,
            axis,
            Fixed64.One,
            Vector3d.Right);

        support.Should().Be(expected);
    }
}
