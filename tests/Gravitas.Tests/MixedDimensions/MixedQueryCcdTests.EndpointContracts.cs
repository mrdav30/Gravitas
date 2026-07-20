using FixedMathSharp;
using FixedMathSharp.Bounds;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedQueryCcdTests
{
    [Fact]
    public void SweepCircleAgainstCuboid_WhenFirstContactIsAtEndpoint_ShouldPreserveEndpointDistance()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBody3D(
            context,
            new LSCuboidCollider(),
            Vector3d.Zero,
            immovable: true);

        context.QueryMixed.SweepCircleAgainst3D(
                new Vector2d((Fixed64)(-2), Fixed64.Zero),
                -Vector2d.Right,
                Fixed64.Half,
                Fixed64.Zero,
                Fixed64.Half,
                IncludeLayerZero,
                out PhysicsMixedHit hit)
            .Should()
            .BeTrue();

        hit.Distance.Should().Be(Fixed64.One);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepCircleAgainstSphere_WhenFirstContactIsAtEndpoint_ShouldPreserveEndpointDistance()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateSphere3D(context, Vector3d.Zero, immovable: true);

        context.QueryMixed.SweepCircleAgainst3D(
                new Vector2d((Fixed64)(-2), Fixed64.Zero),
                -Vector2d.Right,
                Fixed64.Half,
                Fixed64.Zero,
                Fixed64.Half,
                IncludeLayerZero,
                out PhysicsMixedHit hit)
            .Should()
            .BeTrue();

        hit.Distance.Should().Be(Fixed64.One);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepCircleAgainstCapsule_WhenFirstContactIsAtEndpoint_ShouldPreserveEndpointDistance()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBody3D(
            context,
            new LSCapsuleCollider
            {
                Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One)
            },
            Vector3d.Zero,
            immovable: true);

        context.QueryMixed.SweepCircleAgainst3D(
                new Vector2d((Fixed64)(-2), Fixed64.Zero),
                -Vector2d.Right,
                Fixed64.Half,
                Fixed64.Zero,
                Fixed64.Half,
                IncludeLayerZero,
                out PhysicsMixedHit hit)
            .Should()
            .BeTrue();

        hit.Distance.Should().Be(Fixed64.One);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepCircleAgainstMesh_WhenFirstContactIsAtEndpoint_ShouldPreserveEndpointDistance()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateMesh3D(
            context,
            MeshTestFixtures.CreateVerticalQuad(Fixed64.Zero, -Fixed64.One, Fixed64.One),
            Vector3d.Zero,
            immovable: true);

        context.QueryMixed.SweepCircleAgainst3D(
                new Vector2d(Fixed64.FromFraction(-3, 2), Fixed64.Zero),
                new Vector2d(-Fixed64.Half, Fixed64.Zero),
                Fixed64.Half,
                Fixed64.Zero,
                Fixed64.Half,
                IncludeLayerZero,
                out PhysicsMixedHit hit)
            .Should()
            .BeTrue();

        hit.Distance.Should().Be(Fixed64.One);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepSphereAgainstAabb_WhenFirstContactIsAtEndpoint_ShouldPreserveEndpointDistance()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBodylessBox2D(context, Vector2d.Zero, Vector2d.One * Fixed64.Two);

        context.QueryMixed.SweepSphereAgainst2D(
                new Vector3d(Fixed64.FromFraction(-5, 2), Fixed64.Zero, Fixed64.Zero),
                new Vector3d(Fixed64.FromFraction(-3, 2), Fixed64.Zero, Fixed64.Zero),
                Fixed64.Half,
                IncludeLayerZero,
                out PhysicsMixedHit hit)
            .Should()
            .BeTrue();

        hit.Distance.Should().Be(Fixed64.One);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepSphereAgainstCapsule_WhenFirstContactIsAtEndpoint_ShouldPreserveEndpointDistance()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessCapsule2D(context, Vector2d.Zero);

        bool found = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        found.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(target);
        hit.Distance.Should().Be(Fixed64.One);
        hit.Point3D.Should().Be(new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        hit.Normal3DTo2D.Should().Be(Vector3d.Right);
    }

    [Fact]
    public void SweepSphereAgainstCapsule_WhenCapAndRimMeetAtEndpoint_ShouldKeepCapNormal()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessCapsule2D(context, Vector2d.Zero);

        bool found = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d(Fixed64.Half, (Fixed64)2, Fixed64.Zero),
            new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        found.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(target);
        hit.Distance.Should().Be(Fixed64.One);
        hit.Point3D.Should().Be(new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Zero));
        hit.Normal3DTo2D.Should().Be(-Vector3d.Up);
    }

    [Fact]
    public void SweepSphereAgainstCapsule_WhenStartOverlapsCapInterior_ShouldReportZeroDistance()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessCapsule2D(context, Vector2d.Zero);
        Vector3d start = new(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero);

        bool found = context.QueryMixed.SweepSphereAgainst2D(
            start,
            new Vector3d((Fixed64)3, start.Y, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        found.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(target);
        hit.Distance.Should().Be(Fixed64.Zero);
        hit.Normal3DTo2D.Should().Be(-Vector3d.Up);
        hit.Point2D.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Zero));
    }

    [Fact]
    public void CapsuleSlabReducer_WithExtremeFiniteAxisCoefficients_ShouldKeepExactPointAndNormal()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var target = (LSCapsuleCollider2D)CreateBodylessCapsule2D(context, Vector2d.Zero);
        Vector3d start = new((Fixed64)(-200_000), Fixed64.Zero, Fixed64.Zero);
        Vector3d end = new((Fixed64)200_000, Fixed64.Zero, Fixed64.Zero);

        bool found = GravitasQueryMixedService.TrySweepSphereAgainstCapsuleSlab(
            start,
            end,
            Vector3d.Right,
            (Fixed64)400_000,
            Fixed64.FromFraction(199_999, 2),
            target,
            out PhysicsMixedHit hit);

        found.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(target);
        hit.Distance.Should().Be((Fixed64)100_000);
        hit.Point3D.Should().Be(new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        hit.Normal3DTo2D.Should().Be(Vector3d.Right);
    }

    [Fact]
    public void CapsuleSlabReducer_WithPlanarEntryAboveSlab_ShouldKeepLaterBoundaryEntry()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessCapsule2D(context, Vector2d.Zero);
        Vector3d start = new((Fixed64)(-3), (Fixed64)2, Fixed64.Zero);
        Vector3d end = new((Fixed64)3, Fixed64.Zero, Fixed64.Zero);
        Vector3d.TryGetDistance(start, end, out Fixed64 length).Should().BeTrue();

        bool found = context.QueryMixed.SweepSphereAgainst2D(
            start,
            end,
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        found.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(target);
        hit.Distance.Should().BeGreaterThan(length * Fixed64.FromFraction(1, 3));
        hit.Distance.Should().BeLessThan(length * Fixed64.Half);
    }

    [Fact]
    public void SweepSphereAgainstCapsuleSlab_WithSubRawVerticalDirection_ShouldReconstructFromExactDistance()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessCapsule2D(context, Vector2d.Zero);
        Vector3d start = new((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero);
        Vector3d end = new((Fixed64)3, Fixed64.FromRaw(3), Fixed64.Zero);
        Vector3d.TryGetDistance(start, end, out Fixed64 length).Should().BeTrue();
        Vector3d expectedCenter = new FixedSegment(start, end).GetPointAtDistance(Fixed64.Two, length);

        bool found = context.QueryMixed.SweepSphereAgainst2D(
            start,
            end,
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        found.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(target);
        hit.Distance.Should().Be(Fixed64.Two);
        hit.Point3D.Should().Be(expectedCenter + Vector3d.Right * Fixed64.Half);
        hit.Point2D.Y.Should().Be(expectedCenter.Y);
        hit.Normal3DTo2D.Should().Be(Vector3d.Right);
    }

    [Fact]
    public void CircleSlabReducer_WithSaturatedLocalHeight_ShouldKeepExactTopCapEntry()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCircleCollider2D target = CreateExtremeCircleSlab(context);
        Vector3d start = new((Fixed64)(-1_000_000_000), (Fixed64)2_000_000_000, Fixed64.Zero);
        Vector3d end = new((Fixed64)(-1_000_000_000), (Fixed64)1_800_000_000, Fixed64.Zero);

        bool found = GravitasQueryMixedService.TrySweepSphereAgainstCircleSlab(
            start,
            end,
            -Vector3d.Up,
            (Fixed64)200_000_000,
            (Fixed64)1_000_000_000,
            target,
            out PhysicsMixedHit hit);

        found.Should().BeTrue();
        hit.Distance.Should().Be((Fixed64)100_000_000);
    }

    [Fact]
    public void CircleSlabReducer_OneRawSeparatedLongSweepEntries_ShouldRemainOrdered()
    {
        using GravitasWorldContext context = CreateMixedContext();
        Fixed64 raw = Fixed64.FromRaw(1);
        var first = (LSCircleCollider2D)CreateBodylessCircle2D(
            context,
            new Vector2d((Fixed64)101, Fixed64.Zero));
        var second = (LSCircleCollider2D)CreateBodylessCircle2D(
            context,
            new Vector2d((Fixed64)101 + raw, Fixed64.Zero));
        Vector3d start = Vector3d.Zero;
        Vector3d end = new((Fixed64)1_000_000, Fixed64.Zero, Fixed64.Zero);

        GravitasQueryMixedService.TrySweepSphereAgainstCircleSlab(
                start,
                end,
                Vector3d.Right,
                (Fixed64)1_000_000,
                Fixed64.Half,
                first,
                out PhysicsMixedHit firstHit)
            .Should()
            .BeTrue();
        GravitasQueryMixedService.TrySweepSphereAgainstCircleSlab(
                start,
                end,
                Vector3d.Right,
                (Fixed64)1_000_000,
                Fixed64.Half,
                second,
                out PhysicsMixedHit secondHit)
            .Should()
            .BeTrue();

        firstHit.Distance.Should().Be((Fixed64)100);
        secondHit.Distance.Should().Be((Fixed64)100 + raw);
        secondHit.Distance.Should().BeGreaterThan(firstHit.Distance);
    }

    [Fact]
    public void CircleSlabReducer_WithRadialCrossingAboveExactTopCap_ShouldRejectSide()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCircleCollider2D target = CreateExtremeCircleSlab(context);
        Vector3d start = new((Fixed64)1_500_000_000, (Fixed64)2_000_000_000, Fixed64.Zero);
        Vector3d end = new((Fixed64)1_300_000_000, (Fixed64)2_000_000_000, Fixed64.Zero);

        bool found = GravitasQueryMixedService.TrySweepSphereAgainstCircleSlab(
            start,
            end,
            -Vector3d.Right,
            (Fixed64)200_000_000,
            (Fixed64)1_000_000_000,
            target,
            out PhysicsMixedHit hit);

        found.Should().BeFalse();
        hit.Should().Be(default(PhysicsMixedHit));
    }

    [Fact]
    public void CircleSlabReducer_NearPositiveDomainEdge_ShouldKeepRepresentableExpandedLowerCap()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCircleCollider2D target = CreateCircleSlabAtScalarEdge(context, positive: true);
        Vector3d start = new(Fixed64.Zero, Fixed64.MaxValue - (Fixed64)5, Fixed64.Zero);
        Vector3d end = new(Fixed64.Zero, Fixed64.MaxValue - (Fixed64)4, Fixed64.Zero);

        bool found = GravitasQueryMixedService.TrySweepSphereAgainstCircleSlab(
            start,
            end,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.One,
            target,
            out PhysicsMixedHit hit);

        found.Should().BeTrue();
        hit.Distance.Should().Be(Fixed64.One);
        hit.Point3D.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.MaxValue - (Fixed64)3, Fixed64.Zero));
        hit.Normal3DTo2D.Should().Be(Vector3d.Up);
    }

    [Fact]
    public void CircleSlabReducer_NearNegativeDomainEdge_ShouldKeepRepresentableExpandedUpperCap()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCircleCollider2D target = CreateCircleSlabAtScalarEdge(context, positive: false);
        Vector3d start = new(Fixed64.Zero, Fixed64.MinValue + (Fixed64)5, Fixed64.Zero);
        Vector3d end = new(Fixed64.Zero, Fixed64.MinValue + (Fixed64)4, Fixed64.Zero);

        bool found = GravitasQueryMixedService.TrySweepSphereAgainstCircleSlab(
            start,
            end,
            -Vector3d.Up,
            Fixed64.One,
            Fixed64.One,
            target,
            out PhysicsMixedHit hit);

        found.Should().BeTrue();
        hit.Distance.Should().Be(Fixed64.One);
        hit.Point3D.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.MinValue + (Fixed64)3, Fixed64.Zero));
        hit.Normal3DTo2D.Should().Be(-Vector3d.Up);
    }

    [Theory]
    [InlineData(ColliderType2D.Capsule, true)]
    [InlineData(ColliderType2D.Capsule, false)]
    [InlineData(ColliderType2D.AABox, true)]
    [InlineData(ColliderType2D.AABox, false)]
    public void SphereSlabReducer_WithSlabAtVerticalDomainEdge_ShouldKeepPlanarSideHit(
        ColliderType2D targetShape,
        bool positiveEdge)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = targetShape == ColliderType2D.Capsule
            ? CreateBodylessCapsule2D(context, Vector2d.Zero)
            : CreateBodylessBox2D(context, Vector2d.Zero, new Vector2d(Fixed64.One, Fixed64.One));
        Fixed64 y = positiveEdge ? Fixed64.MaxValue : Fixed64.MinValue;
        target.AgentOrNull!.Transform.LocalPosition = new Vector3d(Fixed64.Zero, y, Fixed64.Zero);
        target.MixedHalfThicknessOverride = Fixed64.One;
        target.RebuildRuntimeShapeOnly().Should().BeTrue();

        Vector3d start = new((Fixed64)(-3), y, Fixed64.Zero);
        Vector3d end = new((Fixed64)3, y, Fixed64.Zero);
        bool found = target is LSCapsuleCollider2D capsule
            ? GravitasQueryMixedService.TrySweepSphereAgainstCapsuleSlab(
                start,
                end,
                Vector3d.Right,
                (Fixed64)6,
                Fixed64.Half,
                capsule,
                out PhysicsMixedHit hit)
            : GravitasQueryMixedService.TrySweepSphereAgainstConvexSlab(
                start,
                end,
                Vector3d.Right,
                (Fixed64)6,
                Fixed64.Half,
                target,
                out hit);

        found.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(target);
        hit.Distance.Should().BeGreaterThan(Fixed64.Zero);
        hit.Normal3DTo2D.Should().Be(Vector3d.Right);
    }

    private static LSCircleCollider2D CreateExtremeCircleSlab(GravitasWorldContext context)
    {
        var target = (LSCircleCollider2D)CreateBodylessCircle2D(context, Vector2d.Zero);
        target.AgentOrNull!.Transform.LocalPosition = new Vector3d(
            (Fixed64)(-1_000_000_000),
            (Fixed64)(-500_000_000),
            Fixed64.Zero);
        target.Radius = (Fixed64)1_400_000_000;
        target.MixedHalfThicknessOverride = (Fixed64)1_400_000_000;
        target.RebuildRuntimeShapeOnly().Should().BeTrue();
        return target;
    }

    private static LSCircleCollider2D CreateCircleSlabAtScalarEdge(
        GravitasWorldContext context,
        bool positive)
    {
        var target = (LSCircleCollider2D)CreateBodylessCircle2D(context, Vector2d.Zero);
        Fixed64 centerY = positive
            ? Fixed64.MaxValue - Fixed64.One
            : Fixed64.MinValue + Fixed64.One;
        target.AgentOrNull!.Transform.LocalPosition = new Vector3d(Fixed64.Zero, centerY, Fixed64.Zero);
        target.MixedHalfThicknessOverride = Fixed64.Two;
        target.RebuildRuntimeShapeOnly().Should().BeTrue();
        return target;
    }

}
