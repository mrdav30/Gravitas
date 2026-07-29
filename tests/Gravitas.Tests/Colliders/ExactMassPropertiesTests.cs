//=======================================================================
// ExactMassPropertiesTests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

using FixedMathSharp;
using FixedMathSharp.Geometry;

using Gravitas.Colliders;

using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class ExactMassPropertiesTests
{
    [Fact]
    public void UniformTriangleShell_RetainsExactCombinedFirstMoment()
    {
        Vector3d[] vertices =
        {
            new(Fixed64.FromRaw(-4), Fixed64.Zero, Fixed64.Zero),
            new(Fixed64.FromRaw(-3), Fixed64.One, Fixed64.Zero),
            new(Fixed64.FromRaw(-3), Fixed64.Zero, Fixed64.One),
            new(Fixed64.FromRaw(-3), Fixed64.Zero, Fixed64.Zero),
            new(Fixed64.FromRaw(-2), Fixed64.One, Fixed64.Zero),
            new(Fixed64.FromRaw(-2), Fixed64.Zero, Fixed64.One)
        };
        int[] indices = { 0, 1, 2, 3, 4, 5 };
        var first = new FixedTriangle(
            vertices[0],
            vertices[1],
            vertices[2]);
        var second = new FixedTriangle(
            vertices[3],
            vertices[4],
            vertices[5]);

        Assert.True(TriangleShellMassProperties.TryCreateUniformShell(
            vertices,
            indices,
            out ExactMassWeight surfaceWeight,
            out Vector3d center,
            out Fixed3x3 tensor));
        Fixed64 expectedArea = first.Area + second.Area;
        Assert.True(surfaceWeight.TryGetMeasure(out Fixed64 area));
        Assert.Equal(expectedArea, area);
        Assert.Equal(Fixed64.FromRaw(-3), center.X);
        Assert.Equal(Fixed64.FromFraction(1, 3), center.Y);
        Assert.Equal(Fixed64.FromFraction(1, 3), center.Z);
        Assert.True(tensor.M11 > Fixed64.Zero);
        Assert.Equal(tensor.M12, tensor.M21);
        Assert.Equal(tensor.M13, tensor.M31);
        Assert.Equal(tensor.M23, tensor.M32);
    }

    [Fact]
    public void UniformTriangleShell_MatchesAnalyticRightTriangleAndTranslation()
    {
        Vector3d[] vertices =
        {
            Vector3d.Zero,
            Vector3d.Right * Fixed64.Two,
            Vector3d.Up * Fixed64.Two
        };
        Vector3d translation = new(
            (Fixed64)100,
            (Fixed64)(-50),
            (Fixed64)25);
        Vector3d[] translatedVertices =
        {
            vertices[0] + translation,
            vertices[1] + translation,
            vertices[2] + translation
        };
        int[] indices = { 0, 1, 2 };

        Assert.True(TriangleShellMassProperties.TryCreateUniformShell(
            vertices,
            indices,
            out _,
            out Vector3d center,
            out Fixed3x3 tensor));
        Assert.True(TriangleShellMassProperties.TryCreateUniformShell(
            translatedVertices,
            indices,
            out _,
            out Vector3d translatedCenter,
            out Fixed3x3 translatedTensor));

        Assert.Equal(new Vector3d(
            Fixed64.FromFraction(2, 3),
            Fixed64.FromFraction(2, 3),
            Fixed64.Zero), center);
        Assert.Equal(center + translation, translatedCenter);
        Assert.Equal(tensor, translatedTensor);
        AssertRawNear(tensor.M11, Fixed64.FromFraction(2, 9));
        AssertRawNear(tensor.M22, Fixed64.FromFraction(2, 9));
        AssertRawNear(tensor.M33, Fixed64.FromFraction(4, 9));
        AssertRawNear(tensor.M12, Fixed64.FromFraction(1, 9));
        Assert.Equal(Fixed64.Zero, tensor.M13);
        Assert.Equal(Fixed64.Zero, tensor.M23);
    }

    [Fact]
    public void UniformTriangleShell_ValidatesTopologyAndRejectsMissingSurface()
    {
        Vector3d[] vertices =
        {
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Up
        };

        Assert.Throws<ArgumentException>(
            () => TriangleShellMassProperties.TryCreateUniformShell(
                vertices,
                new[] { 0, 1 },
                out _,
                out _,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => TriangleShellMassProperties.TryCreateUniformShell(
                vertices,
                new[] { 3, 1, 2 },
                out _,
                out _,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => TriangleShellMassProperties.TryCreateUniformShell(
                vertices,
                new[] { 0, 3, 2 },
                out _,
                out _,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => TriangleShellMassProperties.TryCreateUniformShell(
                vertices,
                new[] { 0, 1, 3 },
                out _,
                out _,
                out _));
        Assert.False(TriangleShellMassProperties.TryCreateUniformShell(
            vertices,
            Array.Empty<int>(),
            out ExactMassWeight surfaceWeight,
            out Vector3d center,
            out Fixed3x3 tensor));
        Assert.Equal(default, surfaceWeight);
        Assert.Equal(default, center);
        Assert.Equal(default, tensor);
    }

    [Fact]
    public void UniformTriangleShell_RejectsOnlyFinalTensorOverflow()
    {
        Vector3d[] vertices =
        {
            new((Fixed64)(-50001), Fixed64.Zero, Fixed64.Zero),
            new((Fixed64)(-50000), Fixed64.Zero, Fixed64.Zero),
            new((Fixed64)(-50001), Fixed64.One, Fixed64.Zero),
            new((Fixed64)50000, Fixed64.Zero, Fixed64.Zero),
            new((Fixed64)50001, Fixed64.Zero, Fixed64.Zero),
            new((Fixed64)50000, Fixed64.One, Fixed64.Zero)
        };

        Assert.False(TriangleShellMassProperties.TryCreateUniformShell(
            vertices,
            new[] { 0, 1, 2, 3, 4, 5 },
            out ExactMassWeight surfaceWeight,
            out Vector3d center,
            out Fixed3x3 tensor));
        Assert.Equal(default, surfaceWeight);
        Assert.Equal(default, center);
        Assert.Equal(default, tensor);
    }

    [Fact]
    public void WideWeights_PreserveUnrepresentableRelativeMeasures()
    {
        Fixed64 extent = (Fixed64)1_500_000;
        ExactMassWeight first = ExactMassWeight.FromProduct(
            extent,
            extent,
            extent);
        ExactMassWeight second = ExactMassWeight.FromProduct(
            extent,
            extent,
            extent,
            Fixed64.Two);
        ExactMassWeight total = first.Add(second);

        Assert.False(first.TryGetMeasure(out _));
        Assert.True(first.TryGetProportionalShare(
            (Fixed64)3,
            total,
            out Fixed64 firstShare));
        Assert.True(second.TryGetProportionalShare(
            (Fixed64)3,
            total,
            out Fixed64 secondShare));
        Assert.Equal(Fixed64.One, firstShare);
        Assert.Equal(Fixed64.Two, secondShare);
    }

    [Fact]
    public void WeightedMassPoints3d_CancelOutsideTheScalarDomain()
    {
        ExactMassPoint3D negative = ExactMassPoint3D.CreateScaledLocalComposition(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Two, Fixed64.One, Fixed64.One),
            Vector3d.Zero,
            Vector3d.One,
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ExactMassPoint3D positive = ExactMassPoint3D.CreateScaledLocalComposition(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Two, Fixed64.One, Fixed64.One),
            Vector3d.Zero,
            Vector3d.One,
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ExactMassPoint3D[] points = { negative, positive };
        ExactMassWeight[] weights =
        {
            ExactMassWeight.One,
            ExactMassWeight.One,
        };

        Assert.False(negative.TryGetPoint(out _));
        Assert.False(positive.TryGetPoint(out _));
        Assert.True(ExactMassPoint3D.TryGetWeightedAverage(
            points,
            weights,
            out Vector3d average));
        Assert.Equal(
            new Vector3d(Fixed64.FromRaw(-1), Fixed64.Zero, Fixed64.Zero),
            average);
    }

    [Fact]
    public void WeightedMassPoints2d_CancelOutsideTheScalarDomain()
    {
        ExactMassPoint2D negative = ExactMassPoint2D.CreateScaledLocalComposition(
            new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            new Vector2d(Fixed64.Two, Fixed64.One),
            Vector2d.Zero,
            Vector2d.One,
            Vector2d.Zero,
            Fixed64.Zero);
        ExactMassPoint2D positive = ExactMassPoint2D.CreateScaledLocalComposition(
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            new Vector2d(Fixed64.Two, Fixed64.One),
            Vector2d.Zero,
            Vector2d.One,
            Vector2d.Zero,
            Fixed64.Zero);
        ExactMassPoint2D[] points = { negative, positive };
        ExactMassWeight[] weights =
        {
            ExactMassWeight.One,
            ExactMassWeight.One,
        };

        Assert.False(negative.TryGetPoint(out _));
        Assert.False(positive.TryGetPoint(out _));
        Assert.True(ExactMassPoint2D.TryGetWeightedAverage(
            points,
            weights,
            out Vector2d average));
        Assert.Equal(
            new Vector2d(Fixed64.FromRaw(-1), Fixed64.Zero),
            average);
    }

    [Fact]
    public void ParallelAxis3d_RetainsWidePointUntilTheFinalTensor()
    {
        ExactMassPoint3D point = ExactMassPoint3D.CreateScaledLocalComposition(
            new Vector3d((Fixed64)1_500_000_000, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Two, Fixed64.One, Fixed64.One),
            Vector3d.Zero,
            Vector3d.One,
            Vector3d.Zero,
            FixedQuaternion.Identity);

        Assert.False(point.TryGetPoint(out _));
        Assert.True(point.TryAddParallelAxisTensor(
            Fixed3x3.Zero,
            Fixed64.FromRaw(1),
            Vector3d.Zero,
            out Fixed3x3 tensor));
        Assert.Equal(Fixed64.Zero, tensor.M11);
        Assert.Equal(
            Fixed64.FromRaw(9_000_000_000_000_000_000L),
            tensor.M22);
        Assert.Equal(tensor.M22, tensor.M33);

        Assert.False(point.TryAddParallelAxisTensor(
            Fixed3x3.Zero,
            Fixed64.FromRaw(2),
            Vector3d.Zero,
            out _));
    }

    [Fact]
    public void ParallelAxis2d_RetainsWidePointUntilTheFinalMoment()
    {
        ExactMassPoint2D point = ExactMassPoint2D.CreateScaledLocalComposition(
            new Vector2d((Fixed64)1_500_000_000, Fixed64.Zero),
            new Vector2d(Fixed64.Two, Fixed64.One),
            Vector2d.Zero,
            Vector2d.One,
            Vector2d.Zero,
            Fixed64.Zero);

        Assert.False(point.TryGetPoint(out _));
        Assert.True(point.TryAddParallelAxisMoment(
            Fixed64.Zero,
            Fixed64.FromRaw(1),
            Vector2d.Zero,
            out Fixed64 moment));
        Assert.Equal(
            Fixed64.FromRaw(9_000_000_000_000_000_000L),
            moment);

        Assert.False(point.TryAddParallelAxisMoment(
            Fixed64.Zero,
            Fixed64.FromRaw(2),
            Vector2d.Zero,
            out _));
    }

    [Fact]
    public void SemanticMassInputs_RejectInvalidContracts()
    {
        Assert.Throws<ArgumentOutOfRangeException>(
            () => ExactMassWeight.FromMeasure(-Fixed64.One));
        Assert.Throws<ArgumentException>(
            () => ExactMassPoint3D.CreateScaledLocalComposition(
                Vector3d.Zero,
                Vector3d.One,
                Vector3d.Zero,
                Vector3d.One,
                Vector3d.Zero,
                default));

        ExactMassPoint3D[] points = { ExactMassPoint3D.FromPoint(Vector3d.Zero) };
        Assert.Throws<ArgumentException>(
            () => ExactMassPoint3D.TryGetWeightedAverage(
                points,
                Array.Empty<ExactMassWeight>(),
                out _));
        Assert.False(ExactMassPoint3D.TryGetWeightedAverage(
            points,
            new[] { ExactMassWeight.Zero },
            out _));
    }

    [Fact]
    public void MassPoint3d_MatchesScaleInvariantQuaternionComposition()
    {
        FixedQuaternion rotation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)17,
                (Fixed64)29,
                (Fixed64)43);
        Vector3d displacement = new(
            (Fixed64)1_000_000_000,
            (Fixed64)(-500_000_000),
            (Fixed64)250_000_000);

        Assert.NotEqual(Fixed64.One, rotation.MagnitudeSquared);
        Assert.True(Vector3d.TryComposeScaledLocalPoints(
            Vector3d.Zero,
            Vector3d.One,
            Vector3d.Zero,
            Vector3d.One,
            displacement,
            rotation,
            out Vector3d expected));
        ExactMassPoint3D point =
            ExactMassPoint3D.CreateScaledLocalComposition(
                Vector3d.Zero,
                Vector3d.One,
                Vector3d.Zero,
                Vector3d.One,
                displacement,
                rotation);

        Assert.True(point.TryGetPoint(out Vector3d actual));
        Assert.Equal(expected, actual);
    }

    [Fact]
    public void WideWeightAddition_RejectsSemanticOverflow()
    {
        ExactMassWeight weight = ExactMassWeight.One;
        bool overflowed = false;
        for (int i = 0; i < 256; i++)
        {
            if (weight.TryAdd(weight, out ExactMassWeight doubled))
            {
                weight = doubled;
                continue;
            }

            overflowed = true;
            Assert.Throws<OverflowException>(() =>
            {
                _ = weight.Add(weight);
            });
            break;
        }

        Assert.True(overflowed);
    }

    [Fact]
    public void WeightShares_RejectInvalidTotalsAndOversubscribedWeights()
    {
        ExactMassWeight weight =
            ExactMassWeight.FromProduct(Fixed64.Two, Fixed64.Two);
        ExactMassWeight smallerTotal = ExactMassWeight.One;

        Assert.True(ExactMassWeight.Zero.IsZero);
        Assert.False(weight.TryGetProportionalShare(
            -Fixed64.One,
            weight,
            out _));
        Assert.False(weight.TryGetProportionalShare(
            Fixed64.One,
            ExactMassWeight.Zero,
            out _));
        Assert.False(weight.TryGetProportionalShare(
            Fixed64.One,
            smallerTotal,
            out _));
    }

    [Fact]
    public void WeightedAverages_RejectZeroWeightAndUnrepresentableResults()
    {
        ExactMassWeight[] zeroWeight = { ExactMassWeight.Zero };
        ExactMassPoint2D representablePoint =
            ExactMassPoint2D.FromPoint(Vector2d.One);
        Assert.True(representablePoint.TryGetPoint(out Vector2d materialized));
        Assert.Equal(Vector2d.One, materialized);
        Assert.False(ExactMassPoint2D.TryGetWeightedAverage(
            new[] { representablePoint },
            zeroWeight,
            out _));

        ExactMassPoint2D point2d =
            ExactMassPoint2D.CreateScaledLocalComposition(
                new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
                new Vector2d(Fixed64.Two, Fixed64.One),
                Vector2d.Zero,
                Vector2d.One,
                Vector2d.Zero,
                Fixed64.Zero);
        Assert.False(point2d.TryGetPoint(out _));
        Assert.False(ExactMassPoint2D.TryGetWeightedAverage(
            new[] { point2d },
            new[] { ExactMassWeight.One },
            out _));

        ExactMassPoint3D point3d =
            ExactMassPoint3D.CreateScaledLocalComposition(
                new Vector3d(
                    Fixed64.MaxValue,
                    Fixed64.Zero,
                    Fixed64.Zero),
                new Vector3d(
                    Fixed64.Two,
                    Fixed64.One,
                    Fixed64.One),
                Vector3d.Zero,
                Vector3d.One,
                Vector3d.Zero,
                FixedQuaternion.Identity);
        Assert.False(ExactMassPoint3D.TryGetWeightedAverage(
            new[] { point3d },
            new[] { ExactMassWeight.One },
            out _));
    }

    [Fact]
    public void ParallelAxis3d_HandlesNonPositiveMassAndFinalAdditionOverflow()
    {
        ExactMassPoint3D point = ExactMassPoint3D.FromPoint(Vector3d.Right);
        Fixed3x3 centerTensor = new(
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);

        Assert.False(point.TryAddParallelAxisTensor(
            Fixed3x3.Zero,
            -Fixed64.One,
            Vector3d.Zero,
            out _));
        Assert.True(point.TryAddParallelAxisTensor(
            centerTensor,
            Fixed64.Zero,
            Vector3d.Zero,
            out Fixed3x3 zeroMassTensor));
        Assert.Equal(centerTensor, zeroMassTensor);
        Assert.False(point.TryAddParallelAxisTensor(
            centerTensor,
            Fixed64.One,
            Vector3d.Zero,
            out _));
    }

    [Fact]
    public void ParallelAxis2d_HandlesNonPositiveMassAndFinalAdditionOverflow()
    {
        ExactMassPoint2D point =
            ExactMassPoint2D.FromPoint(Vector2d.Right);

        Assert.False(point.TryAddParallelAxisMoment(
            Fixed64.Zero,
            -Fixed64.One,
            Vector2d.Zero,
            out _));
        Assert.True(point.TryAddParallelAxisMoment(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Vector2d.Zero,
            out Fixed64 zeroMassMoment));
        Assert.Equal(Fixed64.MaxValue, zeroMassMoment);
        Assert.False(point.TryAddParallelAxisMoment(
            Fixed64.MaxValue,
            Fixed64.One,
            Vector2d.Zero,
            out _));
    }

    [Fact]
    public void PolygonWeightAndCentroid_ShouldPreserveWideAreaAndRejectZeroArea()
    {
        Fixed64 extent = (Fixed64)1_000_000_000;
        Vector2d[] square =
        {
            Vector2d.Zero,
            new(extent, Fixed64.Zero),
            new(extent, extent),
            new(Fixed64.Zero, extent)
        };
        Vector2d[] line =
        {
            Vector2d.Zero,
            Vector2d.Right,
            Vector2d.Right * Fixed64.Two
        };
        Vector2d[] clockwise =
        {
            Vector2d.Zero,
            new(Fixed64.Zero, extent),
            new(extent, extent),
            new(extent, Fixed64.Zero)
        };

        Assert.True(PolygonMassProperties2D.TryGetWeightAndCentroid(
            square,
            out ExactMassWeight weight,
            out Vector2d centroid));
        Assert.False(weight.TryGetMeasure(out _));
        Assert.Equal(
            new Vector2d(extent * Fixed64.Half, extent * Fixed64.Half),
            centroid);
        Assert.True(PolygonMassProperties2D.TryGetWeightAndCentroid(
            clockwise,
            out ExactMassWeight clockwiseWeight,
            out Vector2d clockwiseCentroid));
        ExactMassWeight totalWeight = weight.Add(clockwiseWeight);
        Assert.True(weight.TryGetProportionalShare(
            Fixed64.Two,
            totalWeight,
            out Fixed64 share));
        Assert.Equal(Fixed64.One, share);
        Assert.Equal(centroid, clockwiseCentroid);
        Assert.False(PolygonMassProperties2D.TryGetWeightAndCentroid(
            line,
            out ExactMassWeight zeroWeight,
            out Vector2d zeroCentroid));
        Assert.True(zeroWeight.IsZero);
        Assert.Equal(Vector2d.Zero, zeroCentroid);
    }

    private static void AssertRawNear(
        Fixed64 actual,
        Fixed64 expected)
    {
        Fixed64 tolerance = Fixed64.FromRaw(2);
        Assert.True(actual >= expected - tolerance);
        Assert.True(actual <= expected + tolerance);
    }
}
