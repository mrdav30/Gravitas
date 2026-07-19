using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedNarrowPhaseTests
{
    [Theory]
    [InlineData(3, 2)]
    [InlineData(-3, 2)]
    [InlineData(0, 0)]
    public void RotationalSeparationGap_ForSphereInsidePolygonPlanarFootprint_ShouldUseFiniteSlabDistance(
        int sphereY,
        int expectedGap)
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, (Fixed64)sphereY, Fixed64.Zero));
        SolidBody2D polygon = CreateBody2D(
            context,
            CreateSquarePolygon(),
            Vector2d.Zero);

        bool calculated = CollisionDetectionMixed.TryGetRotationalSeparationGap(
            sphere.Collider,
            polygon.Collider,
            out Fixed64 gap,
            out bool supported);

        supported.Should().BeTrue();
        calculated.Should().BeTrue();
        gap.Should().Be((Fixed64)expectedGap);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void RotationalSeparationGap_ForOppositeFullDomainSphereAndSlabCenters_ShouldRemainConservative(
        bool sphereIsPositive)
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(
            context,
            new Vector3d(
                Fixed64.Zero,
                sphereIsPositive ? Fixed64.MaxValue : Fixed64.MinValue,
                Fixed64.Zero));
        SolidBody2D polygon = CreateBody2D(
            context,
            CreateSquarePolygon(),
            Vector2d.Zero);
        polygon.Agent.Transform.LocalPosition = new Vector3d(
            Fixed64.Zero,
            sphereIsPositive ? Fixed64.MinValue : Fixed64.MaxValue,
            Fixed64.Zero);
        polygon.Collider.RebuildRuntimeShapeOnly();

        bool calculated = CollisionDetectionMixed.TryGetRotationalSeparationGap(
            sphere.Collider,
            polygon.Collider,
            out _,
            out bool supported);

        supported.Should().BeTrue();
        calculated.Should().BeFalse();
    }

    [Fact]
    public void RotationalSeparationGap_ForYawCuboidShouldHonorRotationAndVerticalSlabDistance()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCuboidCollider> cuboid = CreateCuboid3D(
            context,
            Vector3d.Zero,
            FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                FixedMath.DegToRad((Fixed64)45)));
        SolidBody2D circle = CreateBody2D(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Right * (Fixed64)2);

        CollisionDetectionMixed.TryGetRotationalSeparationGap(
                cuboid.Collider,
                circle.Collider,
                out Fixed64 yawGap,
                out bool yawSupported)
            .Should()
            .BeTrue();

        circle.Agent.Transform.LocalPosition = new Vector3d(
            Fixed64.Zero,
            (Fixed64)3,
            Fixed64.Zero);
        circle.SetPosition(Vector2d.Zero);
        circle.Collider.RebuildRuntimeShapeOnly();
        CollisionDetectionMixed.TryGetRotationalSeparationGap(
                cuboid.Collider,
                circle.Collider,
                out Fixed64 verticalGap,
                out bool verticalSupported)
            .Should()
            .BeTrue();

        yawSupported.Should().BeTrue();
        yawGap.Should().BeGreaterThan(Fixed64.Zero);
        verticalSupported.Should().BeTrue();
        verticalGap.Should().Be((Fixed64)2);
    }

    [Fact]
    public void RotationalSeparationGap_ForTiltedCuboidOrUnsupportedPair_ShouldDeclineOwnership()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCuboidCollider> tilted = CreateCuboid3D(
            context,
            Vector3d.Zero,
            FixedQuaternion.FromAxisAngle(
                Vector3d.Right,
                FixedMath.DegToRad((Fixed64)45)));
        SolidBody2D circle = CreateBody2D(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero);
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(context, Vector3d.Zero);

        CollisionDetectionMixed.TryGetRotationalSeparationGap(
                tilted.Collider,
                circle.Collider,
                out _,
                out bool tiltedSupported)
            .Should()
            .BeFalse();
        CollisionDetectionMixed.TryGetRotationalSeparationGap(
                sphere.Collider,
                circle.Collider,
                out _,
                out bool pairSupported)
            .Should()
            .BeFalse();

        tiltedSupported.Should().BeFalse();
        pairSupported.Should().BeFalse();
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void RotationalSeparationGap_ForCuboidAtVerticalDomainEdgeShouldNotCertifyOverflowedBounds(
        bool positive)
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCuboidCollider> cuboid = CreateCuboid3D(
            context,
            new Vector3d(
                Fixed64.Zero,
                positive ? Fixed64.MaxValue : Fixed64.MinValue,
                Fixed64.Zero));
        SolidBody2D circle = CreateBody2D(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero);

        bool calculated = CollisionDetectionMixed.TryGetRotationalSeparationGap(
            cuboid.Collider,
            circle.Collider,
            out _,
            out bool supported);

        supported.Should().BeTrue();
        calculated.Should().BeFalse();
    }

    [Fact]
    public void RotationalSeparationGap_ForRepresentableDiagonalBeyondDistanceDomain_ShouldRemainConservative()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCuboidCollider> cuboid = CreateCuboid3D(context, Vector3d.Zero);
        Fixed64 distantCoordinate = (Fixed64)1_600_000_000;
        SolidBody2D circle = CreateBody2D(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d(distantCoordinate, distantCoordinate));

        bool calculated = CollisionDetectionMixed.TryGetRotationalSeparationGap(
            cuboid.Collider,
            circle.Collider,
            out _,
            out bool supported);

        supported.Should().BeTrue();
        calculated.Should().BeFalse();
    }

    [Fact]
    public void RotationalSeparationGap_ForOppositeRepresentableCuboidAndSlabBounds_ShouldRejectUnrepresentableGap()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCuboidCollider> cuboid = CreateCuboid3D(context, Vector3d.Zero);
        SolidBody2D circle = CreateBody2D(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero);
        cuboid.Collider.LocalOffset = new Vector3d(
            Fixed64.Zero,
            Fixed64.MinValue + Fixed64.One,
            Fixed64.Zero);
        circle.Collider.MixedHalfThicknessOverride = Fixed64.FromRaw(1);
        circle.Agent.Transform.LocalPosition = new Vector3d(
            Fixed64.Zero,
            Fixed64.MaxValue - Fixed64.One,
            Fixed64.Zero);
        cuboid.Collider.RebuildRuntimeShapeOnly();
        circle.Collider.RebuildRuntimeShapeOnly();

        bool calculated = CollisionDetectionMixed.TryGetRotationalSeparationGap(
            cuboid.Collider,
            circle.Collider,
            out _,
            out bool supported);

        supported.Should().BeTrue();
        calculated.Should().BeFalse();
    }
}
