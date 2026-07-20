using FixedMathSharp;
using Gravitas.Queries;
using System;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class GjkSimplexScaleTests
{
    [Fact]
    public void ScaleForProducts_ShouldPreserveOrdinaryAndBoundExtremePlanarCoordinates()
    {
        Span<Vector2d> ordinary = stackalloc Vector2d[2];
        ordinary[0] = new Vector2d((Fixed64)2, (Fixed64)(-3));
        ordinary[1] = new Vector2d((Fixed64)4, (Fixed64)5);

        Fixed64 ordinaryScale = GjkSimplexScale.ScaleForProducts(ordinary);

        Assert.Equal(Fixed64.One, ordinaryScale);
        Assert.Equal(new Vector2d((Fixed64)2, (Fixed64)(-3)), ordinary[0]);
        Assert.Equal(new Vector2d((Fixed64)4, (Fixed64)5), ordinary[1]);

        Span<Vector2d> extreme = stackalloc Vector2d[2];
        extreme[0] = new Vector2d(Fixed64.MaxValue, Fixed64.MaxValue);
        extreme[1] = new Vector2d(-Fixed64.MaxValue, Fixed64.MaxValue);

        Fixed64 extremeScale = GjkSimplexScale.ScaleForProducts(extreme);

        Assert.True(extremeScale < Fixed64.One);
        Assert.Equal(extreme[0].X, extreme[0].Y);
        Assert.Equal(-extreme[1].X, extreme[1].Y);
        Assert.True(extreme[0].X >= (Fixed64)4);
        Assert.True(extreme[0].X <= (Fixed64)8);
    }

    [Fact]
    public void WorkingDifferences_ShouldRemainRepresentableAcrossTheFullScalarDomain()
    {
        Vector3d twoTerm = GjkSimplexScale.CreateWorkingDifference(
            new Vector3d(Fixed64.MaxValue, Fixed64.MinValue, Fixed64.MaxValue),
            new Vector3d(Fixed64.MinValue, Fixed64.MaxValue, Fixed64.Zero));
        Vector2d threeTerm = GjkSimplexScale.CreateWorkingDifference(
            new Vector2d(Fixed64.MaxValue, Fixed64.MinValue),
            new Vector2d(Fixed64.MinValue, Fixed64.MaxValue),
            new Vector2d(Fixed64.MinValue, Fixed64.MaxValue));
        Vector2d planarTwoTerm = GjkSimplexScale.CreateWorkingDifference(
            new Vector2d(Fixed64.MaxValue, Fixed64.MinValue),
            new Vector2d(Fixed64.MinValue, Fixed64.MaxValue));

        Assert.Equal(Fixed64.MaxValue, twoTerm.X);
        Assert.Equal(-Fixed64.MaxValue, twoTerm.Y);
        Assert.Equal(Fixed64.FromRaw(long.MaxValue >> 1), twoTerm.Z);
        Assert.Equal(Fixed64.FromRaw(3L * (1L << 61) - 1L), threeTerm.X);
        Assert.Equal(Fixed64.FromRaw(-3L * (1L << 61) + 2L), threeTerm.Y);
        Assert.Equal(Fixed64.FromRaw(long.MaxValue >> 1), planarTwoTerm.X);
        Assert.Equal(Fixed64.FromRaw(-(long.MaxValue >> 1)), planarTwoTerm.Y);
        Assert.Equal((Fixed64)500, GjkSimplexScale.RestoreTwoTermDistance((Fixed64)250));
        Assert.Equal((Fixed64)1000, GjkSimplexScale.RestoreThreeTermDistance((Fixed64)250));
    }

    [Fact]
    public void ShiftSelection_ShouldPreserveFullPrecisionUntilBoundsRequireScaling()
    {
        int ordinaryTwoTerm = GjkSimplexScale.SelectTwoTermShift(
            Vector3d.Zero,
            Vector3d.One,
            Vector3d.One * (Fixed64)2,
            Vector3d.One * (Fixed64)3);
        int extremeTwoTerm = GjkSimplexScale.SelectTwoTermShift(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero));
        int ordinaryThreeTerm = GjkSimplexScale.SelectThreeTermShift(
            Vector2d.Zero,
            -Vector2d.One,
            Vector2d.One,
            Fixed64.Half);
        int halfScaleThreeTerm = GjkSimplexScale.SelectThreeTermShift(
            new Vector2d((Fixed64)1_500_000_000, Fixed64.Zero),
            new Vector2d((Fixed64)(-1_500_000_000), Fixed64.Zero),
            new Vector2d((Fixed64)(-1_500_000_000), Fixed64.Zero),
            Fixed64.One);
        int quarterScalePositive = GjkSimplexScale.SelectThreeTermShift(
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            Fixed64.MaxValue);
        int quarterScaleNegative = GjkSimplexScale.SelectThreeTermShift(
            new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            Fixed64.MaxValue);
        int subtractOverflowThreeTerm = GjkSimplexScale.SelectThreeTermShift(
            new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            Fixed64.Zero);

        Assert.Equal(0, ordinaryTwoTerm);
        Assert.Equal(1, extremeTwoTerm);
        Assert.Equal(0, ordinaryThreeTerm);
        Assert.Equal(1, halfScaleThreeTerm);
        Assert.Equal(2, quarterScalePositive);
        Assert.Equal(2, quarterScaleNegative);
        Assert.Equal(1, subtractOverflowThreeTerm);
        Assert.Equal(Fixed64.One, GjkSimplexScale.GetCoordinateScale(0));
        Assert.Equal(Fixed64.Half, GjkSimplexScale.GetCoordinateScale(1));
        Assert.Equal(Fixed64.Quarter, GjkSimplexScale.GetCoordinateScale(2));
        Assert.Equal((Fixed64)250, GjkSimplexScale.RestoreDistance((Fixed64)250, 0));
    }

    [Fact]
    public void ThreeTermShift_ShouldCoverNegativeOddRawExpansion()
    {
        Vector2d point = new(Fixed64.MaxValue, Fixed64.Zero);
        Vector2d target = new(Fixed64.MinValue, Fixed64.Zero);
        Vector2d negativeExpansion = new(-Fixed64.MinIncrement, Fixed64.Zero);

        int shift = GjkSimplexScale.SelectThreeTermShift(
            point,
            target,
            target,
            Fixed64.MinIncrement);
        Vector2d difference = GjkSimplexScale.CreateWorkingDifference(
            point,
            target,
            negativeExpansion,
            shift);

        Assert.Equal(2, shift);
        Assert.Equal(Fixed64.FromRaw(1L << 62), difference.X);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(3)]
    public void CoordinateShiftOutsideSupportedRange_ShouldThrow(int shift)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => GjkSimplexScale.GetCoordinateScale(shift));
        Assert.Throws<ArgumentOutOfRangeException>(() => GjkSimplexScale.RestoreDistance(Fixed64.One, shift));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            GjkSimplexScale.CreateWorkingDifference(Vector3d.Zero, Vector3d.Zero, shift));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            GjkSimplexScale.CreateWorkingDifference(Vector2d.Zero, Vector2d.Zero, shift));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            GjkSimplexScale.CreateWorkingDifference(Vector2d.Zero, Vector2d.Zero, Vector2d.Zero, shift));
    }

    [Fact]
    public void WorkingDifference_WithInsufficientValidShift_ShouldThrow()
    {
        Assert.Throws<InvalidOperationException>(() => GjkSimplexScale.CreateWorkingDifference(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            shift: 0));
        Assert.Throws<InvalidOperationException>(() => GjkSimplexScale.CreateWorkingDifference(
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            shift: 0));
        Assert.Throws<InvalidOperationException>(() => GjkSimplexScale.CreateWorkingDifference(
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            new Vector2d(-Fixed64.MinIncrement, Fixed64.Zero),
            shift: 1));
        Assert.Throws<InvalidOperationException>(() => GjkSimplexScale.CreateWorkingDifference(
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            Vector2d.Zero,
            shift: 0));
    }

    [Fact]
    public void ScaleForProducts_ShouldBoundExtremeSpatialCoordinates()
    {
        Span<Vector3d> points = stackalloc Vector3d[2];
        points[0] = new Vector3d(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MaxValue);
        points[1] = new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.MaxValue);

        Fixed64 scale = GjkSimplexScale.ScaleForProducts(points);

        Assert.True(scale < Fixed64.One);
        Assert.True(points[0].X >= (Fixed64)4);
        Assert.True(points[0].X <= (Fixed64)8);
        Assert.Equal(points[0].X, points[0].Y);
        Assert.Equal(points[0].Y, points[0].Z);
        Assert.True(points[1].X < Fixed64.Zero);
    }
}
