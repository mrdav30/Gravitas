using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class FiniteSlabProjectionRangeTests
{
    [Fact]
    public void CylinderProjection_WhenOwnerRelativeSupportCannotFit_ShouldDeclineWithoutTruncation()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero, Fixed64.Zero, (Fixed64)(-45));
        LSCylinderCollider cylinder = CreateCylinder(context,
            new Vector3d(Fixed64.MinValue + Fixed64.One, Fixed64.Zero, Fixed64.Zero),
            rotation, Fixed64.MaxValue, Fixed64.MaxValue);
        var slab = new FixedRange(Fixed64.MinValue, Fixed64.MaxValue);

        Assert.True(FixedSlabProjection.TryGetCylinderSupport(cylinder.Center, cylinder.WorldAxis,
            cylinder.Height, cylinder.ScaledRadius, slab, Vector2d.Right, out _));
        Assert.False(FixedSlabProjection.TryGetCylinderSupport(cylinder.Center, cylinder.WorldAxis,
            cylinder.Height, cylinder.ScaledRadius, slab, Vector2d.Left, out _));
        Assert.False(FixedSlabProjection.TryGetCylinderSupport(Vector3d.Zero, cylinder.WorldAxis,
            cylinder.Height, cylinder.ScaledRadius, slab, Vector2d.Left, out _));

        Vector2d point = new(cylinder.Center.X + Fixed64.One, Fixed64.Zero);
        Assert.False(FiniteSlabProjectionSweep.TrySweepCircleAgainstCylinder(point, point,
            Vector2d.Zero, Fixed64.Zero, Fixed64.Zero, slab.Min, slab.Max, cylinder, out Fixed64 distance));
        Assert.Equal(Fixed64.Zero, distance);
    }

    [Fact]
    public void CylinderProjection_WhenConservativeEnvelopeCannotFit_ShouldDeclineUnsupportedScale()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        FixedQuaternion rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero,
            -Fixed64.One, (Fixed64)20).Normalized;
        LSCylinderCollider cylinder = CreateCylinder(context,
            new Vector3d(Fixed64.MinValue + Fixed64.One, Fixed64.Zero, Fixed64.Zero),
            rotation, Fixed64.MaxValue, Fixed64.MinIncrement);
        var slab = new FixedRange(Fixed64.MinValue, Fixed64.MaxValue);

        Assert.True(FixedSlabProjection.TryGetCylinderSupport(Vector3d.Zero, cylinder.WorldAxis,
            cylinder.Height, cylinder.ScaledRadius, slab, Vector2d.Left, out _));
        Assert.False(FixedSlabProjection.TryGetCylinderSupport(cylinder.Center, cylinder.WorldAxis,
            cylinder.Height, cylinder.ScaledRadius, slab, Vector2d.Left, out _));
        Vector2d point = new(cylinder.Center.X, Fixed64.Zero);
        Assert.False(FiniteSlabProjectionSweep.TrySweepCircleAgainstCylinder(point, point,
            Vector2d.Zero, Fixed64.Zero, Fixed64.One, slab.Min, slab.Max, cylinder, out Fixed64 distance));
        Assert.Equal(Fixed64.Zero, distance);
    }

    [Fact]
    public void CylinderProjection_WhenLaterSupportCannotFit_ShouldDeclineWithoutPartialDistance()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)45, Fixed64.Zero, Fixed64.Zero);
        LSCylinderCollider cylinder = CreateCylinder(context,
            new Vector3d(Fixed64.MinValue + Fixed64.One, Fixed64.Zero, Fixed64.Zero),
            rotation, Fixed64.MaxValue, Fixed64.MaxValue);
        var slab = new FixedRange(Fixed64.MinValue, Fixed64.MaxValue);

        Assert.True(FixedSlabProjection.TryGetCylinderSupport(Vector3d.Zero, cylinder.WorldAxis,
            cylinder.Height, cylinder.ScaledRadius, slab, Vector2d.Right, out _));
        Assert.False(FixedSlabProjection.TryGetCylinderSupport(Vector3d.Zero, cylinder.WorldAxis,
            cylinder.Height, cylinder.ScaledRadius, slab, Vector2d.Forward, out _));

        Vector2d point = new(cylinder.Center.X, Fixed64.One);
        Assert.False(FiniteSlabProjectionSweep.TrySweepCircleAgainstCylinder(point, point,
            Vector2d.Zero, Fixed64.Zero, Fixed64.Zero, slab.Min, slab.Max, cylinder, out Fixed64 distance));
        Assert.Equal(Fixed64.Zero, distance);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CylinderProjection_WhenEndpointSupportCannotFit_ShouldDeclineTheWholeSweep(bool positiveZ)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)45, Fixed64.Zero, Fixed64.Zero);
        LSCylinderCollider cylinder = CreateCylinder(context,
            new Vector3d(Fixed64.MinValue + Fixed64.One, Fixed64.Zero, Fixed64.Zero),
            rotation, Fixed64.MaxValue, Fixed64.MaxValue);
        Fixed64 z = positiveZ ? Fixed64.MaxValue : -Fixed64.MaxValue;
        Vector2d start = new(cylinder.Center.X + Fixed64.MaxValue * Fixed64.FromFraction(9, 10), z);
        Vector2d end = new(cylinder.Center.X + Fixed64.MaxValue * Fixed64.FromFraction(3, 4), z);
        Fixed64 length = start.X - end.X;

        // The chord approaches the outer rim but stops short; unsupported endpoint support must not become a hit.
        Assert.False(FiniteSlabProjectionSweep.TrySweepCircleAgainstCylinder(start, end,
            Vector2d.Left, length, Fixed64.Zero, Fixed64.MinValue, Fixed64.MaxValue,
            cylinder, out Fixed64 distance));
        Assert.Equal(Fixed64.Zero, distance);
    }

    [Fact]
    public void CylinderProjection_WhenOnlyAbsoluteSupportFits_ShouldPreserveItsBoundaryContact()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        FixedQuaternion rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero,
            -Fixed64.One, (Fixed64)3).Normalized;
        LSCylinderCollider cylinder = CreateCylinder(context,
            new Vector3d((Fixed64)(-500_000_000), Fixed64.Zero, Fixed64.Zero),
            rotation, Fixed64.MaxValue, Fixed64.MaxValue);
        var slab = new FixedRange(-Fixed64.MaxValue, Fixed64.MaxValue);
        Assert.True(FixedSlabProjection.TryGetCylinderSupport(cylinder.Center, cylinder.WorldAxis,
            cylinder.Height, cylinder.ScaledRadius, slab, Vector2d.Right, out Vector2d point));
        Assert.False(FixedSlabProjection.TryGetCylinderSupport(Vector3d.Zero, cylinder.WorldAxis,
            cylinder.Height, cylinder.ScaledRadius, slab, Vector2d.Right, out _));

        Assert.True(FiniteSlabProjectionSweep.TrySweepCircleAgainstCylinder(point, point,
            Vector2d.Zero, Fixed64.Zero, Fixed64.Zero, slab.Min, slab.Max, cylinder, out Fixed64 distance));
        Assert.Equal(Fixed64.Zero, distance);
        Assert.True(context.QueryMixed.TrySweepCircleAgainst3DCollider(cylinder, point, point,
            Vector2d.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.MaxValue,
            Vector3d.Zero, null, out PhysicsMixedHit hit));
        Assert.Equal(Fixed64.Zero, hit.Distance);
    }

    private static LSCylinderCollider CreateCylinder(GravitasWorldContext context, Vector3d center,
        FixedQuaternion rotation, Fixed64 radius, Fixed64 height)
    {
        var cylinder = new LSCylinderCollider();
        cylinder.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(center, rotation, Vector3d.One)));
        cylinder.Radius = radius;
        cylinder.Size = new Vector3d(Fixed64.One, height, Fixed64.One);
        Assert.True(cylinder.RebuildRuntimeShapeOnly());
        return cylinder;
    }
}
