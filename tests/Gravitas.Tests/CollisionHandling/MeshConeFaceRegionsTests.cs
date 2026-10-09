using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests;

public sealed class MeshConeFaceRegionsTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void FinitePoolReduction_IgnoresAuthoredSeamsAndWinding(bool diagonal, bool winding)
    {
        var mesh = Quad(diagonal, winding);
        var regions = new MeshConeFaceRegions();
        var connectivity = new MeshConeSurfaceConnectivity();
        Build(regions, connectivity, mesh, Vector3d.Zero, FixedQuaternion.Identity,
            (Fixed64)4, Fixed64.Two, new[] { 1, 0 });
        Assert.Equal(4, regions.GetSampleCount(0));
        Vector3d[] expected = { Vector3d.Zero, -Vector3d.Right, -Vector3d.Forward, Vector3d.Forward };
        for (int index = 0; index < expected.Length; index++)
        {
            Assert.True(regions.TryGetSample(0, index, out Vector3d point, out _, out Fixed64 depth));
            Assert.Equal(expected[index], point);
            Assert.Equal(index == 0 ? Fixed64.Two : Fixed64.Zero, depth);
        }
        Assert.False(regions.TryGetSample(0, -1, out _, out _, out _));
        Assert.False(regions.TryGetSample(0, 4, out _, out _, out _));
        Assert.False(regions.TryGetSample(-1, 0, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => regions.GetSampleCount(1));
    }

    [Fact]
    public void FinitePoolReduction_CollinearSubdivisionKeepsIdenticalSamples()
    {
        var baseline = Quad(false, false);
        var subdivided = Create(new[] { new Vector3d(-4,0,-4), new Vector3d(0,0,-4),
            new Vector3d(4,0,-4), new Vector3d(4,0,4), new Vector3d(-4,0,4), Vector3d.Zero },
            new[] { 5,0,1, 5,1,2, 5,2,3, 5,3,4, 5,4,0 });
        var a = new MeshConeFaceRegions(); var b = new MeshConeFaceRegions();
        var connectivity = new MeshConeSurfaceConnectivity();
        Build(a, connectivity, baseline, Vector3d.Zero, Tilt(), (Fixed64)1000, Fixed64.One, new[] { 0,1 });
        Build(b, connectivity, subdivided, Vector3d.Zero, Tilt(), (Fixed64)1000, Fixed64.One, new[] { 4,3,2,1,0 });
        Assert.Equal(a.GetSampleCount(0), b.GetSampleCount(0));
        for (int index = 0; index < a.GetSampleCount(0); index++)
        {
            Assert.True(a.TryGetSample(0, index, out Vector3d ap, out Vector3d aq, out Fixed64 ad));
            Assert.True(b.TryGetSample(0, index, out Vector3d bp, out Vector3d bq, out Fixed64 bd));
            Assert.Equal(ap, bp); Assert.Equal(aq, bq); Assert.Equal(ad, bd);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConvexFanReuse_PartialOwnerCannotWidenTheAdmittedDomain(bool reverse)
    {
        int[] triangles = { 4,0,1, 4,1,2, 4,2,3, 4,3,0 };
        if (reverse)
            for (int index = 0; index < triangles.Length; index += 3)
                (triangles[index + 1], triangles[index + 2]) = (triangles[index + 2], triangles[index + 1]);
        var mesh = Create(new[] { new Vector3d(-4,0,-4), new Vector3d(4,0,-4),
            new Vector3d(4,0,4), new Vector3d(-4,0,4), Vector3d.Zero }, triangles);
        var regions = new MeshConeFaceRegions(); var connectivity = new MeshConeSurfaceConnectivity();
        int[] full = reverse ? new[] { 3,2,1,0 } : new[] { 0,1,2,3 };
        int[] subset = reverse ? new[] { 2,1,0 } : new[] { 0,1,2 };
        var fullPoints = new Vector3d[4]; var fullExits = new Vector3d[4]; var fullDepths = new Fixed64[4];
        for (int stage = 0; stage < 3; stage++)
        {
            bool partial = stage == 1;
            // Deliberately supply only three of four intersecting triangles:
            // one region and a smaller canonical fan do not prove completeness.
            Build(regions, connectivity, mesh, Vector3d.Zero, FixedQuaternion.Identity,
                (Fixed64)4, Fixed64.Two, partial ? subset : full);
            Assert.Equal(1, regions.RegionCount);
            Assert.Equal(partial ? 3 : 4, regions.GetTriangleIndices(0).Length);
            Assert.Equal(4, regions.GetSampleCount(0));
            bool hasLeft = false, hasRight = false;
            for (int index = 0; index < 4; index++)
            {
                Assert.True(regions.TryGetSample(0, index, out Vector3d point, out Vector3d exit, out Fixed64 depth));
                hasLeft |= point == Vector3d.Left; hasRight |= point == Vector3d.Right;
                if (partial)
                {
                    Assert.Equal(Fixed64.Zero, point.Y);
                    // The omitted left wedge is x < -abs(z). Its exact
                    // complementary union includes the two diagonal seams.
                    Assert.True(point.X >= -FixedMath.Abs(point.Z));
                }
                else if (stage == 0)
                {
                    fullPoints[index] = point; fullExits[index] = exit; fullDepths[index] = depth;
                }
                else
                {
                    Assert.Equal(fullPoints[index], point); Assert.Equal(fullExits[index], exit);
                    Assert.Equal(fullDepths[index], depth);
                }
            }
            Assert.Equal(!partial, hasLeft);
            if (partial) Assert.True(hasRight);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AxialIntervals_ReconstructEndpointsBeforeDiscoveryDeduplication(bool reverse)
    {
        var mesh = Create(new[] { new Vector3d(0,-2,-2), new Vector3d(0,2,-2),
            new Vector3d(0,2,2), new Vector3d(0,-2,2) }, new[] { 0,1,2, 0,2,3 });
        var a = new MeshConeFaceRegions(); var b = new MeshConeFaceRegions();
        var connectivity = new MeshConeSurfaceConnectivity();
        Build(a, connectivity, mesh, Vector3d.Zero, FixedQuaternion.Identity,
            Fixed64.Two, Fixed64.One, reverse ? new[] { 1,0 } : new[] { 0,1 });
        Build(b, connectivity, mesh, Vector3d.Zero, FixedQuaternion.Identity,
            Fixed64.Two, Fixed64.One, reverse ? new[] { 0,1 } : new[] { 1,0 });
        Assert.Equal(4, a.GetSampleCount(0));
        Assert.Equal(a.GetSampleCount(0), b.GetSampleCount(0));
        for (int i = 0; i < a.GetSampleCount(0); i++)
        {
            Assert.True(a.TryGetSample(0, i, out Vector3d ap, out Vector3d aq, out Fixed64 ad));
            Assert.True(b.TryGetSample(0, i, out Vector3d bp, out Vector3d bq, out Fixed64 bd));
            Assert.Equal(ap, bp); Assert.Equal(aq, bq); Assert.Equal(ad, bd);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CoincidentBoundaryStationaryEvents_RetainBothDirectionalCertificates(bool reverseWinding)
    {
        var mesh = Create(new[] { -Vector3d.Forward * Fixed64.Half, Vector3d.Forward * Fixed64.Half,
            Vector3d.Up * Fixed64.Half }, reverseWinding ? new[] { 2,1,0 } : new[] { 0,1,2 });
        mesh.UpdatePosition(Vector3d.Up * (Fixed64.One / 4), FixedQuaternion.Identity);
        var regions = new MeshConeFaceRegions(); var connectivity = new MeshConeSurfaceConnectivity();
        Build(regions, connectivity, mesh, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two, new[] { 0 });
        // The exposed horizontal edge has two stationary certificates at its
        // center, one for each exit. Their exact common point merges once.
        Assert.Equal(4, regions.GetSampleCount(0));
        Assert.True(regions.TryGetSelectedRay(0, out Vector3d point, out Vector3d exit, out Fixed64 depth));
        Assert.Equal(Vector3d.Zero, point); Assert.Equal(Vector3d.Right, exit); Assert.Equal(Fixed64.One, depth);
        Assert.True(regions.TryGetSample(0, 0, out Vector3d sample, out Vector3d sampleExit, out Fixed64 sampleDepth));
        Assert.Equal(point, sample); Assert.Equal(exit, sampleExit); Assert.Equal(depth, sampleDepth);
    }

    [Fact]
    public void UnrepresentableEligibleRay_FailsTheRegionExplicitly()
    {
        var mesh = Quad(false, false);
        Vector3d position = new(Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero);
        mesh.UpdatePosition(position, FixedQuaternion.Identity);
        var regions = new MeshConeFaceRegions();
        var connectivity = new MeshConeSurfaceConnectivity();
        Build(regions, connectivity, mesh, position, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two, new[] { 0,1 });
        Assert.Equal(1, regions.RegionCount);
        Assert.False(regions.TryGetSelectedRay(0, out _, out _, out _));
        Assert.False(regions.TryGetSample(0, 0, out _, out _, out _));
        Assert.Throws<OverflowException>(() => regions.GetSampleCount(0));
    }

    [Fact]
    public void UnrepresentableAdmittedMeshAnchor_FailsTheRegionExplicitly()
    {
        var mesh = Create(new[] { new Vector3d(-2,2,-2), new Vector3d(2,-2,-2),
            new Vector3d(2,-2,2), new Vector3d(-2,2,2) }, new[] { 0,1,2, 0,2,3 });
        Vector3d center = new(Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero);
        mesh.UpdatePosition(center, FixedQuaternion.Identity);
        var regions = new MeshConeFaceRegions(); var connectivity = new MeshConeSurfaceConnectivity();
        Build(regions, connectivity, mesh, center, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two, new[] { 0,1 });
        Assert.False(regions.TryGetSelectedRay(0, out _, out _, out _));
        Assert.Throws<OverflowException>(() => regions.GetSampleCount(0));
        Assert.False(regions.TryGetSample(0, 0, out _, out _, out _));
    }

    [Fact]
    public void UnrepresentableOppositeRay_DoesNotPoisonTheSelectedOrientation()
    {
        var mesh = Quad(false, false);
        Vector3d center = new(Fixed64.Zero, Fixed64.MaxValue - Fixed64.One, Fixed64.Zero);
        mesh.UpdatePosition(center - Vector3d.Up, FixedQuaternion.Identity);
        var regions = new MeshConeFaceRegions();
        var connectivity = new MeshConeSurfaceConnectivity();
        Build(regions, connectivity, mesh, center, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two, new[] { 0,1 });
        Assert.True(regions.TryGetSelectedRay(0, out Vector3d point, out Vector3d exit, out Fixed64 depth));
        Assert.Equal(center.Y - Fixed64.One, point.Y);
        Assert.Equal(point - Vector3d.Up, exit);
        Assert.Equal(Fixed64.One, depth);
        // The upward apex lies above MaxValue; only the chosen downward
        // certificate determines whether this finite pool can be represented.
        Assert.Equal(4, regions.GetSampleCount(0));
        for (int index = 0; index < regions.GetSampleCount(0); index++)
            Assert.True(regions.TryGetSample(0, index, out _, out _, out _));
    }

    [Fact]
    public void UnchosenEligibleBoundaryExitOverflow_FailsThePoolEvenWhenSelectedSamplesFit()
    {
        var mesh = Create(new[] { new Vector3d(-4,1,-4), new Vector3d(4,-1,-4),
            new Vector3d(4,-1,4), new Vector3d(-4,1,4) }, new[] { 0,1,2, 0,2,3 });
        var regions = new MeshConeFaceRegions(); var connectivity = new MeshConeSurfaceConnectivity();
        Fixed64 margin = (Fixed64)33 / 8;
        Build(regions, connectivity, mesh, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)8, new[] { 0,1 });
        Assert.Equal(4, regions.GetSampleCount(0));
        for (int index = 0; index < regions.GetSampleCount(0); index++)
        {
            Assert.True(regions.TryGetSample(0, index, out _, out Vector3d sampleExit, out _));
            Assert.True(sampleExit.X < margin);
        }
        // The unchosen stationary ray at the right edge's center exits farther
        // right than every selected sample. Its overflow must still fail the pool.
        Vector3d center = new(Fixed64.MaxValue - margin, Fixed64.Zero, Fixed64.Zero);
        mesh.UpdatePosition(center, FixedQuaternion.Identity);
        Build(regions, connectivity, mesh, center, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)8, new[] { 0,1 });
        Assert.True(regions.TryGetSelectedRay(0, out _, out _, out _));
        Assert.Throws<OverflowException>(() => regions.GetSampleCount(0));
        Assert.False(regions.TryGetSample(0, 0, out _, out _, out _));
    }

    [Fact]
    public void UnrepresentableOppositeDepth_DoesNotPoisonTheSelectedOrientation()
    {
        Fixed64 extent = Fixed64.MaxValue / 2, width = Fixed64.MaxValue / 4;
        var mesh = Create(new[] { new Vector3d(-extent,extent,-width), new Vector3d(extent,-extent,-width),
            new Vector3d(extent,-extent,width), new Vector3d(-extent,extent,width) }, new[] { 0,1,2, 0,2,3 });
        var regions = new MeshConeFaceRegions(); var connectivity = new MeshConeSurfaceConnectivity();
        Build(regions, connectivity, mesh, Vector3d.Zero, FixedQuaternion.Identity,
            Fixed64.MaxValue, Fixed64.MaxValue, new[] { 0,1 });
        Assert.True(regions.TryGetSelectedRay(0, out _, out _, out Fixed64 depth));
        Assert.True(depth > Fixed64.Zero && depth < Fixed64.MaxValue);
        Assert.Equal(4, regions.GetSampleCount(0));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FiniteAxialPatch_ClipsGeneratorAndAxisFamiliesToExposedEdges(bool reverse)
    {
        var mesh = Create(new[] {
            new Vector3d(Fixed64.Zero,-Fixed64.Half,-Fixed64.Two),
            new Vector3d(Fixed64.Zero,Fixed64.Half,-Fixed64.Two),
            new Vector3d(Fixed64.Zero,Fixed64.Half,Fixed64.Two),
            new Vector3d(Fixed64.Zero,-Fixed64.Half,Fixed64.Two) }, new[] { 0,1,2, 0,2,3 });
        var regions = new MeshConeFaceRegions(); var connectivity = new MeshConeSurfaceConnectivity();
        Build(regions, connectivity, mesh, Vector3d.Zero, FixedQuaternion.Identity,
            Fixed64.Two, Fixed64.One, reverse ? new[] { 1,0 } : new[] { 0,1 });
        Assert.Equal(4, regions.GetSampleCount(0));
        for (int i = 0; i < regions.GetSampleCount(0); i++)
        {
            Assert.True(regions.TryGetSample(0, i, out Vector3d p, out Vector3d q, out Fixed64 depth));
            Assert.Equal(Fixed64.Zero, p.X);
            Assert.InRange(p.Y.m_rawValue, (-Fixed64.Half).m_rawValue, Fixed64.Half.m_rawValue);
            Assert.Equal(p.Y, q.Y); Assert.Equal(p.Z, q.Z);
            Assert.Equal(depth, q.X);
        }
    }

    [Fact]
    public void TiltedFiniteCone_RetainsTheSelectedOrientationCertificates()
    {
        var mesh = Quad(false, false);
        var regions = new MeshConeFaceRegions(); var connectivity = new MeshConeSurfaceConnectivity();
        Build(regions, connectivity, mesh, Vector3d.Zero, Tilt(), (Fixed64)4, Fixed64.Two, new[] { 0,1 });
        Assert.Equal(4, regions.GetSampleCount(0));
        Assert.True(regions.TryGetSelectedRay(0, out Vector3d winner, out Vector3d exit, out Fixed64 maximum));
        for (int i = 0; i < regions.GetSampleCount(0); i++)
        {
            Assert.True(regions.TryGetSample(0, i, out Vector3d p, out Vector3d q, out Fixed64 depth));
            Assert.Equal(Fixed64.Zero, p.Y);
            Assert.Equal(p.X, q.X); Assert.Equal(p.Z, q.Z);
            Assert.True(depth <= maximum);
            if (i == 0) { Assert.Equal(winner, p); Assert.Equal(exit, q); Assert.Equal(maximum, depth); }
        }
    }

    [Fact]
    public void TangentGeneratorPatch_ClipsTheContinuousSideToTrueBoundaryEndpoints()
    {
        var mesh = Create(new[] {
            new Vector3d(Fixed64.One / 4,-Fixed64.Half,-Fixed64.One),
            new Vector3d(-Fixed64.One / 4,Fixed64.Half,-Fixed64.One),
            new Vector3d(-Fixed64.One / 4,Fixed64.Half,Fixed64.One),
            new Vector3d(Fixed64.One / 4,-Fixed64.Half,Fixed64.One) }, new[] { 0,1,2, 0,2,3 });
        mesh.UpdatePosition(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity);
        var regions = new MeshConeFaceRegions(); var connectivity = new MeshConeSurfaceConnectivity();
        Build(regions, connectivity, mesh, Vector3d.Zero, FixedQuaternion.Identity,
            Fixed64.Two, Fixed64.One, new[] { 0,1 });
        Assert.Equal(2, regions.GetSampleCount(0));
        for (int i = 0; i < 2; i++)
        {
            Assert.True(regions.TryGetSample(0, i, out Vector3d p, out Vector3d q, out Fixed64 depth));
            Assert.Equal(p, q); Assert.Equal(Fixed64.Zero, depth);
            Assert.Equal(Fixed64.Zero, p.Z);
            Assert.Equal(Fixed64.Half, FixedMath.Abs(p.Y));
            Assert.Equal((Fixed64.One - p.Y) / 2, p.X);
        }
    }

    [Fact]
    public void InteriorQuadBoundary_AllSamplesRemainInsideTheFinitePatch()
    {
        var mesh = Create(new[] {
            new Vector3d(-Fixed64.Half,Fixed64.Zero,-Fixed64.Half),
            new Vector3d(Fixed64.Half,Fixed64.Zero,-Fixed64.Half),
            new Vector3d(Fixed64.Half,Fixed64.Zero,Fixed64.Half),
            new Vector3d(-Fixed64.Half,Fixed64.Zero,Fixed64.Half), Vector3d.Zero },
            new[] { 4,0,1, 4,1,2, 4,2,3, 4,3,0 });
        var regions = new MeshConeFaceRegions(); var connectivity = new MeshConeSurfaceConnectivity();
        // Four authored triangles use the smaller complete convex fan while
        // the warmed build and exact sample materialization stay allocation-free.
        int[] admitted = { 0,1,2,3 };
        bool valid = true;
        long bytes = AllocationTestHelper.MeasureSteadyState(() =>
        {
            Build(regions, connectivity, mesh, Vector3d.Zero, FixedQuaternion.Identity,
                (Fixed64)4, Fixed64.Two, admitted);
            valid &= regions.GetSampleCount(0) == 4;
        }, warmupIterations: 16, stabilizationIterations: 16, measurementIterations: 256);
        Assert.True(valid);
        Assert.Equal(0, bytes);
        Assert.Equal(4, regions.GetSampleCount(0));
        for (int i = 0; i < 4; i++)
        {
            Assert.True(regions.TryGetSample(0, i, out Vector3d p, out Vector3d q, out Fixed64 depth));
            Assert.InRange(p.X.m_rawValue, (-Fixed64.Half).m_rawValue, Fixed64.Half.m_rawValue);
            Assert.InRange(p.Z.m_rawValue, (-Fixed64.Half).m_rawValue, Fixed64.Half.m_rawValue);
            Assert.Equal(Fixed64.Zero, p.Y);
            Assert.Equal(p.X, q.X); Assert.Equal(p.Z, q.Z); Assert.Equal(depth, q.Y);
        }
    }

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

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void NonConvexRing_RejectsThePlaneIntrinsicMaximumInsideItsHole(bool reverse)
    {
        var mesh = Ring();
        var regions = new MeshConeFaceRegions(); var connectivity = new MeshConeSurfaceConnectivity();
        int[] admitted = reverse ? new[] { 7,6,5,4,3,2,1,0 } : new[] { 0,1,2,3,4,5,6,7 };
        Build(regions, connectivity, mesh, Vector3d.Zero, FixedQuaternion.Identity,
            (Fixed64)4, Fixed64.Two, admitted);
        Assert.Equal(1, regions.RegionCount);
        Assert.Equal(1, regions.GetOrientation(0));
        Assert.True(regions.TryGetSelectedRay(0, out Vector3d p, out Vector3d q, out Fixed64 depth));
        Assert.Equal(new Vector3d(-Fixed64.Half,Fixed64.Zero,Fixed64.Zero), p);
        Assert.Equal(new Vector3d(-Fixed64.Half,Fixed64.One,Fixed64.Zero), q);
        Assert.Equal(Fixed64.One, depth);
        for (int i = 0; i < regions.GetSampleCount(0); i++)
        {
            Assert.True(regions.TryGetSample(0, i, out Vector3d sample, out _, out _));
            Assert.True(FixedMath.Abs(sample.X) >= Fixed64.Half || FixedMath.Abs(sample.Z) >= Fixed64.Half);
        }
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
