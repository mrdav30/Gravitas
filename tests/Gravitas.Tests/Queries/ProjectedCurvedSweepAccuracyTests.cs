using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class ProjectedCurvedSweepAccuracyTests
{
    private static readonly Fixed64 EntryTolerance = Fixed64.FromFraction(1, 1_048_576);

    [Theory]
    [InlineData(ColliderType.Capsule, 1)]
    [InlineData(ColliderType.Capsule, 200)]
    [InlineData(ColliderType.Capsule, 800)]
    [InlineData(ColliderType.Cylinder, 1)]
    [InlineData(ColliderType.Cylinder, 200)]
    [InlineData(ColliderType.Cylinder, 500)]
    [InlineData(ColliderType.Cone, 1)]
    [InlineData(ColliderType.Cone, 200)]
    [InlineData(ColliderType.Cone, 500)]
    public void PointSweep_WithHorizontalThreeFourFiveSection_ShouldPreserveEntryDistance(
        ColliderType shape,
        int scale)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCollider target = CreateTarget(context, shape, scale, Vector3d.Zero);
        Fixed64 chordX = GetChordX(target);
        Fixed64 radius = (Fixed64)(5 * scale);
        Fixed64 slabY = (Fixed64)(3 * scale);

        bool found = TrySweep(
            target,
            new Vector2d(chordX, -radius),
            new Vector2d(chordX, radius),
            Fixed64.Zero,
            slabY,
            slabY,
            out Fixed64 distance);

        found.Should().BeTrue();
        (distance - (Fixed64)scale).Abs().Should().BeLessThanOrEqualTo(EntryTolerance);
    }

    [Theory]
    [InlineData(ColliderType.Capsule, 0)]
    [InlineData(ColliderType.Capsule, -1)]
    [InlineData(ColliderType.Capsule, 1)]
    [InlineData(ColliderType.Cylinder, 0)]
    [InlineData(ColliderType.Cylinder, -1)]
    [InlineData(ColliderType.Cylinder, 1)]
    [InlineData(ColliderType.Cone, 0)]
    [InlineData(ColliderType.Cone, -1)]
    [InlineData(ColliderType.Cone, 1)]
    public void CircleSweep_WithTranslatedThickSlab_ShouldPreserveEntryFromBothDirections(
        ColliderType shape,
        int scalarFace)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        int scale = scalarFace == 0 ? 500 : 200;
        Fixed64 radius = (Fixed64)(5 * scale);
        Vector3d center = scalarFace switch
        {
            -1 => new Vector3d(Fixed64.MinValue + Fixed64.Two, Fixed64.Zero,
                Fixed64.MinValue + radius + Fixed64.One),
            1 => new Vector3d(Fixed64.MaxValue - Fixed64.Two, Fixed64.Zero,
                Fixed64.MaxValue - radius - Fixed64.One),
            _ => new Vector3d((Fixed64)17, (Fixed64)(-11), (Fixed64)23)
        };
        LSCollider target = CreateTarget(context, shape, scale, center);
        Fixed64 chordX = GetChordX(target);
        Fixed64 sourceRadius = scalarFace == 0 ? (Fixed64)(scale / 4) : Fixed64.Quarter;
        Fixed64 slabMin = center.Y + (Fixed64)(3 * scale);
        Fixed64 slabMax = center.Y + (Fixed64)(4 * scale);
        var lower = new Vector2d(chordX, center.Z - radius);
        var upper = new Vector2d(chordX, center.Z + radius);

        // The nearest slab plane gives the same 4*scale silhouette; the source
        // disk expands its two Z extrema by precisely sourceRadius.
        TrySweep(target, lower, upper, sourceRadius, slabMin, slabMax,
            out Fixed64 forwardDistance).Should().BeTrue();
        TrySweep(target, upper, lower, sourceRadius, slabMin, slabMax,
            out Fixed64 reverseDistance).Should().BeTrue();

        Fixed64 expected = (Fixed64)scale - sourceRadius;
        (forwardDistance - expected).Abs().Should().BeLessThanOrEqualTo(EntryTolerance);
        (reverseDistance - expected).Abs().Should().BeLessThanOrEqualTo(EntryTolerance);
    }

    [Theory]
    [InlineData(ColliderType.Capsule, 41, 177687535722L)]
    [InlineData(ColliderType.Cylinder, 55, 239207958602L)]
    [InlineData(ColliderType.Cone, 10, 46792824785L)]
    public void CircleSweep_WithOrdinaryTiltedClippedTarget_ShouldPreserveEntryDistance(
        ColliderType shape,
        int targetX,
        long expectedDistanceRaw)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCollider target = shape switch
        {
            ColliderType.Capsule => new LSCapsuleCollider(),
            ColliderType.Cylinder => new LSCylinderCollider(),
            _ => new LSConeCollider()
        };
        target.Radius = Fixed64.Half;
        target.Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One);
        Fixed64 targetZ = shape == ColliderType.Capsule ? Fixed64.Quarter
            : shape == ColliderType.Cylinder ? Fixed64.FromFraction(3, 4) : -Fixed64.Quarter;
        // Freeze the authored 45-degree quaternion. Expected entries are independent
        // high-precision roots of the clipped capsule, cylinder and cone projections.
        var rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero,
            Fixed64.FromRaw(1643612817), Fixed64.FromRaw(3968032378));
        target.InitializeWithNoBody(new TestMatterAgent(context, new FixedTransform(
            new Vector3d((Fixed64)targetX, Fixed64.Zero, targetZ), rotation, Vector3d.One)));

        TrySweep(target, new Vector2d((Fixed64)(-2), Fixed64.Zero),
            new Vector2d((Fixed64)64, Fixed64.Zero), Fixed64.Half,
            -Fixed64.Half, Fixed64.Half, out Fixed64 distance).Should().BeTrue();

        (distance - Fixed64.FromRaw(expectedDistanceRaw)).Abs()
            .Should().BeLessThanOrEqualTo(EntryTolerance);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(4)]
    public void PointSweep_WithShallowCapsuleCapEntry_ShouldPreserveSpatialTolerance(int scale)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCollider target = CreateTarget(context, ColliderType.Capsule, 200 * scale, Vector3d.Zero);
        Fixed64 chordX = Fixed64.One + (Fixed64)(768 * scale);
        Fixed64 radius = (Fixed64)(1000 * scale);
        Fixed64 slabY = (Fixed64)(600 * scale);

        bool found = TrySweep(target,
            new Vector2d(chordX, -radius),
            new Vector2d(chordX, radius),
            Fixed64.Zero, slabY, slabY, out Fixed64 distance);

        found.Should().BeTrue();
        // The 3-4-5 sphere slice has radius 800*scale. Its 7-24-25 cap
        // chord enters at Z=-224*scale; normal closing is 224/800, so a
        // world separation tolerance permits under five times that along Z.
        (distance - (Fixed64)(776 * scale)).Abs()
            .Should().BeLessThanOrEqualTo(EntryTolerance * (Fixed64)5);
    }

    [Fact]
    public void CircleSweep_WithWideMinkowskiEdge_ShouldRetainInitialOverlap()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var target = new LSCapsuleCollider
        {
            Radius = Fixed64.Half,
            Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One)
        };
        var rotation = new FixedQuaternion(Fixed64.One, Fixed64.Zero,
            Fixed64.Zero, Fixed64.One).Normalized;
        target.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, rotation, Vector3d.One)));
        // Individual supports fit the working frame, while their edge can span
        // more than the scalar range. This is a worker arithmetic control.
        var point = new Vector2d(Fixed64.One, Fixed64.One);

        TrySweep(target, point, point, Fixed64.MaxValue - (Fixed64)100,
            Fixed64.Zero, Fixed64.Zero, out Fixed64 distance).Should().BeTrue();

        distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void PublicMixedCircleSweep_WithClippedHorizontalCapsule_ShouldPreserveEntryDistance()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.World.TryAddGrid(new GridConfiguration(
            new Vector3d((Fixed64)(-8), (Fixed64)(-8), (Fixed64)(-8)),
            new Vector3d((Fixed64)8, (Fixed64)8, (Fixed64)8)), out _).Should().BeTrue();
        LSCollider target = CreateTarget(context, ColliderType.Capsule, 1, Vector3d.Zero);

        bool found = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d(Fixed64.Zero, (Fixed64)(-5)),
            new Vector2d(Fixed64.Zero, (Fixed64)5),
            Fixed64.Quarter,
            Fixed64.FromFraction(7, 2),
            Fixed64.Half,
            PhysicsLayerMask.FromLayer(0),
            out PhysicsMixedHit hit);

        found.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(target);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
        (hit.Distance - Fixed64.FromFraction(3, 4)).Abs()
            .Should().BeLessThanOrEqualTo(EntryTolerance);
    }

    private static LSCollider CreateTarget(
        GravitasWorldContext context,
        ColliderType shape,
        int scale,
        Vector3d center)
    {
        Fixed64 radius = (Fixed64)(5 * scale);
        LSCollider target = shape switch
        {
            ColliderType.Capsule => new LSCapsuleCollider(),
            ColliderType.Cylinder => new LSCylinderCollider(),
            _ => new LSConeCollider()
        };
        target.Radius = radius;
        target.Size = new Vector3d(radius * Fixed64.Two,
            shape == ColliderType.Capsule ? radius * Fixed64.Two + Fixed64.Two : Fixed64.Two,
            radius * Fixed64.Two);
        // Equal components give an exact quarter turn, avoiding Euler-angle error.
        FixedQuaternion rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero,
            Fixed64.One, Fixed64.One).Normalized;
        (rotation * Vector3d.Up).Normalized.Should().Be(-Vector3d.Right);
        target.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(center, rotation, Vector3d.One)));
        return target;
    }

    private static Fixed64 GetChordX(LSCollider target)
    {
        if (target is not LSConeCollider cone)
            return target.Center.X;

        FixedSegment.TryGetCenteredAxisEndpoint(cone.Center, cone.WorldAxis,
            cone.Height, positive: false, out Vector3d baseCenter).Should().BeTrue();
        return baseCenter.X;
    }

    private static bool TrySweep(
        LSCollider target,
        Vector2d start,
        Vector2d end,
        Fixed64 sourceRadius,
        Fixed64 slabMin,
        Fixed64 slabMax,
        out Fixed64 distance)
    {
        Vector2d displacement = end - start;
        Vector2d direction = displacement.Normalized;
        Fixed64 length = displacement.Magnitude;
        return target switch
        {
            LSCapsuleCollider capsule => FiniteSlabProjectionSweep.TrySweepCircleAgainstCapsule(
                start, end, direction, length, sourceRadius, slabMin, slabMax, capsule, out distance),
            LSCylinderCollider cylinder => FiniteSlabProjectionSweep.TrySweepCircleAgainstCylinder(
                start, end, direction, length, sourceRadius, slabMin, slabMax, cylinder, out distance),
            _ => FiniteSlabProjectionSweep.TrySweepCircleAgainstCone(
                start, end, direction, length, sourceRadius, slabMin, slabMax,
                (LSConeCollider)target, out distance)
        };
    }
}
