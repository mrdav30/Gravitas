using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests;

public sealed class MeshConeFaceRegionsTests
{
    [Fact]
    public void UnequalSubRawMaxima_SelectNegativeEvenWhenBothRoundToTwo()
    {
        var rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(1), Fixed64.One);
        var mesh = new PhysicsMesh(new[] { new Vector3d(-4, 0, -4), new Vector3d(4, 0, -4),
            new Vector3d(4, 0, 4), new Vector3d(-4, 0, 4) }, new[] { 0, 1, 2, 0, 2, 3 },
            Vector3d.Zero, rotation, MeshColliderMode.Concave);
        var regions = new MeshConeFaceRegions();
        var connectivity = new MeshConeSurfaceConnectivity();
        // The shared rotation makes the plane perpendicular to the cone axis.
        // One raw X translation projects only about 2^-31 of a raw unit onto
        // that axis: exact directional maxima differ, but both round to two.
        Build(regions, connectivity, mesh, new Vector3d(Fixed64.FromRaw(-1), Fixed64.Zero, Fixed64.Zero),
            rotation, (Fixed64)4, Fixed64.Two, new[] { 0, 1 });
        Assert.Equal(1, regions.RegionCount);
        Assert.Equal(-1, regions.GetOrientation(0));
        Assert.True(regions.TryGetSelectedRay(0, out _, out _, out Fixed64 depth));
        Assert.Equal(Fixed64.Two, depth);
    }

    [Fact]
    public void Grouping_ConsumesOnlyTheConnectivityOwnersTriangles()
    {
        PhysicsMesh mesh = Create(new[]
        {
            new Vector3d(-4,0,-4), new Vector3d(4,0,-4), new Vector3d(4,0,4), new Vector3d(-4,0,4),
            new Vector3d(-1,1,-1), new Vector3d(1,1,-1), new Vector3d(0,1,1)
        }, new[] { 0,1,2, 0,2,3, 4,5,6 });
        var regions = new MeshConeFaceRegions();
        var connectivity = new MeshConeSurfaceConnectivity();
        Build(regions, connectivity, mesh, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two, new[] { 2,1,0 });
        Assert.Equal(1, regions.RegionCount);
        Assert.Equal(2, regions.GetTriangleIndices(0).Length);
        Assert.False(regions.GetTriangleIndices(0).Contains(2));
    }

    [Theory]
    [InlineData(false, false, false)]
    [InlineData(false, false, true)]
    [InlineData(false, true, false)]
    [InlineData(false, true, true)]
    [InlineData(true, false, false)]
    [InlineData(true, false, true)]
    [InlineData(true, true, false)]
    [InlineData(true, true, true)]
    public void ExactTiedDirections_ChooseCanonicalPositiveAcrossAuthoredVariants(bool diagonal, bool winding, bool discovery)
    {
        PhysicsMesh mesh = Quad(diagonal, winding);
        var regions = new MeshConeFaceRegions();
        var connectivity = new MeshConeSurfaceConnectivity();
        int[] admitted = discovery ? new[] { 1, 0 } : new[] { 0, 1 };
        Build(regions, connectivity, mesh, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two, admitted);
        Assert.Equal(1, regions.RegionCount);
        Assert.Equal(1, regions.GetOrientation(0));
        Assert.Equal(2, regions.GetTriangleIndices(0).Length);
        Assert.True(regions.TryGetSelectedRay(0, out Vector3d lower, out Vector3d upper, out Fixed64 depth));
        Assert.Equal(Fixed64.Two, depth);
        Assert.Equal(Vector3d.Zero, lower);
        Assert.Equal(new Vector3d(0, 2, 0), upper);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void SmallerNegativeMaximum_KeepsTheSameGeometricWinner(bool diagonal, bool winding)
    {
        PhysicsMesh mesh = Quad(diagonal, winding);
        var regions = new MeshConeFaceRegions();
        var connectivity = new MeshConeSurfaceConnectivity();
        Build(regions, connectivity, mesh, new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Zero),
            FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two, new[] { 1, 0 });
        Assert.Equal(1, regions.RegionCount);
        Assert.Equal(-1, regions.GetOrientation(0));
        Assert.True(regions.TryGetSelectedRay(0, out Vector3d lower, out Vector3d upper, out Fixed64 depth));
        Assert.Equal((Fixed64)3 / 2, depth);
        Assert.Equal(new Vector3d(-(Fixed64)5 / 4, Fixed64.Zero, Fixed64.Zero), lower);
        Assert.Equal(new Vector3d(-(Fixed64)5 / 4, -(Fixed64)3 / 2, Fixed64.Zero), upper);
    }

    [Fact]
    public void LongTiltedInterior_UsesTheWholeRegionMaximumBeyondTheMidpointExit()
    {
        PhysicsMesh mesh = Quad(false, false, 2);
        var regions = new MeshConeFaceRegions();
        var connectivity = new MeshConeSurfaceConnectivity();
        Build(regions, connectivity, mesh, Vector3d.Zero, Tilt(), (Fixed64)1000, Fixed64.One, new[] { 0, 1 });
        Assert.Equal(1, regions.RegionCount);
        Assert.Equal(1, regions.GetOrientation(0));
        Assert.True(regions.TryGetSelectedRay(0, out Vector3d lower, out Vector3d upper, out Fixed64 depth));
        // A ray from the plane midpoint travels roughly half this distance.
        // The region extremum starts near its lateral boundary instead.
        Assert.InRange(depth.m_rawValue, Fixed64.One.m_rawValue, ((Fixed64)11 / 10).m_rawValue);
        Assert.True(FixedMath.Abs(lower.X) > Fixed64.One);
        Assert.Equal(Fixed64.Zero, lower.Y);
        Assert.Equal(lower.X, upper.X);
        Assert.Equal(lower.Z, upper.Z);
        Assert.Equal(depth, upper.Y);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DisconnectedRingSections_ChooseTheirDirectionsIndependently(bool reverse)
    {
        PhysicsMesh mesh = Ring();
        var regions = new MeshConeFaceRegions();
        var connectivity = new MeshConeSurfaceConnectivity();
        int[] admitted = reverse ? new[] { 7, 2, 6, 3 } : new[] { 2, 3, 6, 7 };
        Build(regions, connectivity, mesh, Vector3d.Zero, Tilt(), (Fixed64)1000, Fixed64.Half, admitted);
        Assert.Equal(2, regions.RegionCount);
        // The ideal rotation has radial axis (7/25,24/25) and axial
        // axis (-24/25,7/25). At negative X, increasing Y enters the cone;
        // the nearby exit is downward. The right component is the converse.
        Assert.Equal(-1, regions.GetOrientation(0));
        Assert.Equal(1, regions.GetOrientation(1));
        Assert.Equal(2, regions.GetTriangleIndices(0).Length);
        Assert.Equal(2, regions.GetTriangleIndices(1).Length);
        Assert.True(regions.TryGetSelectedRay(0, out Vector3d left, out Vector3d leftExit, out Fixed64 leftDepth));
        Assert.True(regions.TryGetSelectedRay(1, out Vector3d right, out Vector3d rightExit, out Fixed64 rightDepth));
        Assert.Equal(new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero), left);
        Assert.Equal(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero), right);
        Assert.Equal(-leftDepth, leftExit.Y);
        Assert.Equal(rightDepth, rightExit.Y);
        // At z=0, the negative ray length is
        // (1/4+(7/25+12/25000)x)/(24/25-7/50000), maximal at x=-1/2.
        // The positive counterpart is maximal at x=1/2. Quaternion raw
        // rounding is far below these independent 11/100..12/100 bounds.
        Assert.InRange(leftDepth.m_rawValue, ((Fixed64)11 / 100).m_rawValue, ((Fixed64)12 / 100).m_rawValue);
        Assert.InRange(rightDepth.m_rawValue, ((Fixed64)11 / 100).m_rawValue, ((Fixed64)12 / 100).m_rawValue);
    }

    [Fact]
    public void EmptyAndReusedBuilds_ClearOldRegionsAndRejectStaleOrdinals()
    {
        PhysicsMesh mesh = Quad(false, false);
        var regions = new MeshConeFaceRegions();
        var connectivity = new MeshConeSurfaceConnectivity();
        Build(regions, connectivity, mesh, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two, new[] { 0, 1 });
        Assert.Equal(1, regions.RegionCount);
        Build(regions, connectivity, mesh, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two, ReadOnlySpan<int>.Empty);
        Assert.Equal(0, regions.RegionCount);
        Assert.False(regions.TryGetSelectedRay(0, out _, out _, out _));
        Assert.False(regions.TryGetSelectedRay(-1, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => regions.GetOrientation(0));
        Assert.Throws<ArgumentOutOfRangeException>(() => { regions.GetTriangleIndices(0); });
        Build(regions, connectivity, mesh, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two, new[] { 1, 0 });
        Assert.Equal(1, regions.RegionCount);
        Assert.Equal(1, regions.GetOrientation(0));
    }

    [Fact]
    public void RepeatedBuildAndWinnerReconstruction_AllocateZeroAfterWarmup()
    {
        PhysicsMesh mesh = Ring();
        var regions = new MeshConeFaceRegions();
        var connectivity = new MeshConeSurfaceConnectivity();
        int[] admitted = { 7, 2, 6, 3 };
        bool valid = true;
        long bytes = AllocationTestHelper.MeasureSteadyState(() =>
        {
            Build(regions, connectivity, mesh, Vector3d.Zero, Tilt(), (Fixed64)1000, Fixed64.Half, admitted);
            valid &= regions.TryGetSelectedRay(0, out _, out _, out _);
            valid &= regions.TryGetSelectedRay(1, out _, out _, out _);
        }, warmupIterations: 8, stabilizationIterations: 4, measurementIterations: 8);
        Assert.True(valid);
        Assert.Equal(0, bytes);
    }

    private static void Build(MeshConeFaceRegions regions, MeshConeSurfaceConnectivity connectivity, PhysicsMesh mesh,
        Vector3d center, FixedQuaternion rotation, Fixed64 height, Fixed64 radius, ReadOnlySpan<int> admitted)
    {
        connectivity.Build(mesh, 0, center, rotation, height, radius, admitted);
        regions.Build(mesh, 0, center, rotation, height, radius, connectivity, admitted);
    }

    private static FixedQuaternion Tilt() =>
        new(Fixed64.Zero, Fixed64.Zero, (Fixed64)3 / 5, (Fixed64)4 / 5);

    private static PhysicsMesh Quad(bool diagonal, bool winding, int extent = 4)
    {
        int[] triangles = diagonal ? new[] { 0, 1, 3, 1, 2, 3 } : new[] { 0, 1, 2, 0, 2, 3 };
        if (winding)
            for (int i = 0; i < triangles.Length; i += 3)
                (triangles[i + 1], triangles[i + 2]) = (triangles[i + 2], triangles[i + 1]);
        return Create(new[] { new Vector3d(-extent, 0, -extent), new Vector3d(extent, 0, -extent),
            new Vector3d(extent, 0, extent), new Vector3d(-extent, 0, extent) }, triangles);
    }

    private static PhysicsMesh Ring() => Create(new[]
    {
        new Vector3d(-2,0,-2), new Vector3d(2,0,-2), new Vector3d(2,0,2), new Vector3d(-2,0,2),
        new Vector3d(-Fixed64.Half,Fixed64.Zero,-Fixed64.Half), new Vector3d(Fixed64.Half,Fixed64.Zero,-Fixed64.Half),
        new Vector3d(Fixed64.Half,Fixed64.Zero,Fixed64.Half), new Vector3d(-Fixed64.Half,Fixed64.Zero,Fixed64.Half)
    }, new[] { 0,1,5, 0,5,4, 1,2,6, 1,6,5, 2,3,7, 2,7,6, 3,0,4, 3,4,7 });

    private static PhysicsMesh Create(Vector3d[] vertices, int[] triangles) =>
        new(vertices, triangles, Vector3d.Zero, FixedQuaternion.Identity, MeshColliderMode.Concave);
}
