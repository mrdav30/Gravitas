using System;
using System.Numerics;
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests;

public sealed class MeshCapsuleFaceRegionsTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void OffsetSphereAndAxialSections_ChooseTheSameShorterExitForEverySample(bool sphere, bool above)
    {
        PhysicsMesh mesh = Quad(false, false, 4);
        var connectivity = new MeshSurfaceConnectivity();
        var regions = new MeshCapsuleFaceRegions();
        Vector3d center = Vector3d.Up * (above ? Fixed64.Quarter : -Fixed64.Quarter);
        // A sphere's arbitrary core frame cannot affect its signed exit policy.
        FixedQuaternion rotation = sphere ? FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.PiOver4)
            : FixedQuaternion.Identity;
        Build(regions, connectivity, mesh, center, rotation, sphere ? Fixed64.Zero : Fixed64.Two,
            Fixed64.One, new[] { 1, 0 });
        Assert.Equal(1, regions.GetSampleCount(0));
        Fixed64 maximum = (sphere ? Fixed64.One : Fixed64.Two) - Fixed64.Quarter;
        for (int sample = 0; sample < regions.GetSampleCount(0); sample++)
        {
            Assert.True(regions.TryGetSample(0, sample, out FixedContactAnchors contact));
            Assert.Equal(above ? Vector3d.Up : Vector3d.Down, contact.Normal);
            Assert.Equal(Fixed64.Zero, contact.FirstAnchor.LocalPoint.Y);
            Assert.InRange(contact.Depth.m_rawValue, 0, maximum.m_rawValue);
            if (sample == 0) Assert.Equal(maximum, contact.Depth);
        }
    }

    [Theory]
    [InlineData(false, false, false)]
    [InlineData(false, true, true)]
    [InlineData(true, false, true)]
    [InlineData(true, true, false)]
    public void ContainedSection_PreservesExactMaximumAndSeamIndependentCoverage(bool sphere, bool diagonal, bool winding)
    {
        PhysicsMesh mesh = Quad(diagonal, winding, 4);
        var connectivity = new MeshSurfaceConnectivity();
        var regions = new MeshCapsuleFaceRegions();
        Build(regions, connectivity, mesh, Vector3d.Zero, FixedQuaternion.Identity,
            sphere ? Fixed64.Zero : Fixed64.Two, Fixed64.One, new[] { 1, 0 });
        Assert.Equal(1, regions.RegionCount);
        Assert.False(regions.HasBoundaryIntersection);
        Assert.Equal(Vector3d.Down, regions.GetNormal(0));
        Assert.Equal(1, regions.GetSampleCount(0));
        Vector3d[] points = { Vector3d.Zero };
        var expected = new FixedContactAnchors[points.Length];
        for (int sample = 0; sample < points.Length; sample++)
        {
            Assert.True(regions.TryGetSample(0, sample, out FixedContactAnchors contact));
            expected[sample] = contact;
            Assert.Equal(points[sample], contact.FirstAnchor.LocalPoint);
            Assert.Equal(sphere ? Fixed64.One : Fixed64.Two, contact.Depth);
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d exit));
            Fixed64 coreY = FixedMath.Clamp(exit.Y, -Fixed64.One, Fixed64.One);
            Vector3d radial = sphere ? exit : exit - Vector3d.Up * coreY;
            Assert.Equal(Fixed64.One, Vector3d.Dot(radial, radial));
        }
        Build(regions, connectivity, mesh, Vector3d.Zero, FixedQuaternion.Identity,
            sphere ? Fixed64.Zero : Fixed64.Two, Fixed64.One, new[] { 0, 1 });
        Assert.Equal(expected.Length, regions.GetSampleCount(0));
        for (int sample = 0; sample < expected.Length; sample++)
        {
            Assert.True(regions.TryGetSample(0, sample, out FixedContactAnchors contact));
            Assert.Equal(expected[sample].FirstAnchor, contact.FirstAnchor);
            Assert.Equal(expected[sample].SecondAnchor, contact.SecondAnchor);
            Assert.Equal(expected[sample].Normal, contact.Normal);
            Assert.Equal(expected[sample].Depth, contact.Depth);
        }
        Assert.False(regions.TryGetSample(-1, 0, out _));
        Assert.False(regions.TryGetSample(0, -1, out _));
        Assert.False(regions.TryGetSample(0, 1, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => regions.GetSampleCount(1));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void FlatOrZeroRadiusSupport_PreservesTheFirstExactTieAndRepeatedCoverage(bool axial, bool zeroRadius)
    {
        PhysicsMesh mesh = Quad(false, false, 4);
        var connectivity = new MeshSurfaceConnectivity();
        var regions = new MeshCapsuleFaceRegions();
        FixedQuaternion rotation = axial ? FixedQuaternion.Identity
            : new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized;
        Fixed64 radius = zeroRadius ? Fixed64.Zero : Fixed64.One;
        Build(regions, connectivity, mesh, Vector3d.Zero, rotation, Fixed64.Two, radius, new[] { 1, 0 });
        Assert.Equal(1, regions.RegionCount);
        Assert.False(regions.HasBoundaryIntersection);
        Assert.Equal(Vector3d.Down, regions.GetNormal(0));
        Assert.True(regions.TryGetSample(0, 0, out FixedContactAnchors primary));
        Assert.Equal(axial ? Fixed64.One + radius : radius, primary.Depth);
        Assert.Equal(axial ? Vector3d.Zero : Vector3d.Left, primary.FirstAnchor.LocalPoint);
        int count = regions.GetSampleCount(0);
        var expected = new FixedContactAnchors[count];
        for (int sample = 0; sample < count; sample++)
        {
            Assert.True(regions.TryGetSample(0, sample, out expected[sample]));
            Assert.Equal(Vector3d.Down, expected[sample].Normal);
            if (zeroRadius)
            {
                Assert.Equal(axial ? Fixed64.One : Fixed64.Zero, expected[sample].Depth);
                Assert.True(expected[sample].SecondAnchor.TryGetPoint(out Vector3d exit));
                Assert.Equal(axial ? Vector3d.Up : expected[sample].FirstAnchor.LocalPoint, exit);
            }
        }
        Build(regions, connectivity, mesh, Vector3d.Zero, FixedQuaternion.Identity,
            Fixed64.Zero, Fixed64.One, ReadOnlySpan<int>.Empty);
        Build(regions, connectivity, mesh, Vector3d.Zero, rotation, Fixed64.Two, radius, new[] { 0, 1 });
        Assert.Equal(count, regions.GetSampleCount(0));
        for (int sample = 0; sample < count; sample++)
        {
            Assert.True(regions.TryGetSample(0, sample, out FixedContactAnchors contact));
            Assert.Equal(expected[sample].FirstAnchor, contact.FirstAnchor);
            Assert.Equal(expected[sample].SecondAnchor, contact.SecondAnchor);
            Assert.Equal(expected[sample].Depth, contact.Depth);
            Assert.Equal(expected[sample].Normal, contact.Normal);
        }
    }

    [Fact]
    public void ContainedObliqueSection_WithUnadmittedEndpointProjection_KeepsTheFiniteBodyMaximum()
    {
        PhysicsMesh mesh = Quad(false, false, 4);
        var connectivity = new MeshSurfaceConnectivity();
        var regions = new MeshCapsuleFaceRegions();
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Forward, -Fixed64.PiOver4);
        Build(regions, connectivity, mesh, Vector3d.Zero, rotation, (Fixed64)8, Fixed64.Quarter, new[] { 1, 0 });
        Assert.Equal(1, regions.RegionCount);
        Assert.False(regions.HasBoundaryIntersection);
        Assert.Equal(4, regions.GetSampleCount(0));
        Assert.True(regions.TryGetSample(0, 0, out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Down, contact.Normal);
        // Use the authored quaternion's exact rational rotation, rather than
        // treating its rounded sine/cosine as an ideal 45-degree angle. Its
        // core direction is (p,h,0)/D, with p²+h²=D². The finite section rim
        // is X=rD/h and its upward first exit is 2rD/p. Distant core caps do
        // not participate because their plane projections miss this section.
        BigInteger w = rotation.W.m_rawValue, z = rotation.Z.m_rawValue;
        BigInteger denominator = w * w + z * z, p = -2 * w * z, h = w * w - z * z;
        BigInteger radius = Fixed64.Quarter.m_rawValue;
        Assert.Equal(Fixed64.Zero, rotation.X); Assert.Equal(Fixed64.Zero, rotation.Y);
        Assert.True(p.Sign > 0 && h.Sign > 0);
        long rimRaw = RoundPositiveRatio(radius * denominator, h);
        long depthRaw = RoundPositiveRatio(2 * radius * denominator, p);
        long coreRaw = RoundPositiveRatio(radius * (p * p + 2 * h * h), p * h);
        Assert.Equal(rimRaw, contact.FirstAnchor.LocalPoint.X.m_rawValue);
        Assert.Equal(Fixed64.Zero, contact.FirstAnchor.LocalPoint.Y);
        Assert.Equal(Fixed64.Zero, contact.FirstAnchor.LocalPoint.Z);
        Assert.Equal(depthRaw, contact.Depth.m_rawValue);
        Assert.Equal(Vector3d.Up * Fixed64.FromRaw(coreRaw), contact.SecondAnchor.LocalPoint);
        Assert.Equal(Vector3d.Left * Fixed64.Quarter, contact.SecondAnchor.LocalDisplacement);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d exit));
        // The body core coordinate rounds once (<=1/2 raw), then its rigid
        // world materialization rounds once. Each rotated coefficient is <1,
        // so the resulting world component differs by at most one raw unit
        // from the independently rounded exact ray coordinate.
        Assert.InRange(FixedMath.Abs(exit.X - Fixed64.FromRaw(rimRaw)).m_rawValue, 0L, 1L);
        Assert.InRange(FixedMath.Abs(exit.Y - Fixed64.FromRaw(depthRaw)).m_rawValue, 0L, 1L);
        Assert.Equal(Fixed64.Zero, exit.Z);

        static long RoundPositiveRatio(BigInteger numerator, BigInteger divisor)
        {
            BigInteger rounded = BigInteger.DivRem(numerator, divisor, out BigInteger remainder);
            int comparison = (2 * remainder).CompareTo(divisor);
            if (comparison > 0 || comparison == 0 && !rounded.IsEven) rounded++;
            return (long)rounded;
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void BoundaryTouchAndTranslatedScalarFace_KeepFiniteLocalWitnesses(bool positive)
    {
        PhysicsMesh mesh = Quad(true, true, 1);
        var connectivity = new MeshSurfaceConnectivity();
        var regions = new MeshCapsuleFaceRegions();
        Vector3d center = new(Fixed64.Zero, positive ? Fixed64.MaxValue : Fixed64.MinValue, Fixed64.Zero);
        mesh.UpdatePosition(center, FixedQuaternion.Identity);
        Build(regions, connectivity, mesh, center, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One, new[] { 0, 1 });
        Assert.True(regions.HasBoundaryIntersection);
        Assert.Equal(4, regions.GetSampleCount(0));
        Assert.True(regions.TryGetSample(0, 0, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Two, contact.Depth);
        Assert.Equal(Vector3d.Zero, contact.FirstAnchor.LocalPoint);
        Assert.Equal(Vector3d.Up, contact.SecondAnchor.LocalPoint);
        Assert.Equal(Vector3d.Up, contact.SecondAnchor.LocalDisplacement);
        Assert.Equal(!positive, contact.SecondAnchor.TryGetPoint(out _));
        // Reusing the same owner after an empty query cannot publish stale rays.
        Build(regions, connectivity, mesh, center, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One, ReadOnlySpan<int>.Empty);
        Assert.Equal(0, regions.RegionCount);
        Assert.False(regions.TryGetSample(0, 0, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => regions.GetOrientation(0));
    }

    [Fact]
    public void HoleSplitSection_KeepsBothRegionalMaximaAndCoverageCaps()
    {
        var mesh = new PhysicsMesh(new[]
        {
            new Vector3d(-2,0,-2), new Vector3d(2,0,-2), new Vector3d(2,0,2), new Vector3d(-2,0,2),
            new Vector3d(-Fixed64.Half,Fixed64.Zero,-Fixed64.Half), new Vector3d(Fixed64.Half,Fixed64.Zero,-Fixed64.Half),
            new Vector3d(Fixed64.Half,Fixed64.Zero,Fixed64.Half), new Vector3d(-Fixed64.Half,Fixed64.Zero,Fixed64.Half)
        }, new[] { 0,1,5, 0,5,4, 1,2,6, 1,6,5, 2,3,7, 2,7,6, 3,0,4, 3,4,7 },
            Vector3d.Zero, FixedQuaternion.Identity, MeshColliderMode.Concave);
        var horizontal = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized;
        var connectivity = new MeshSurfaceConnectivity();
        var regions = new MeshCapsuleFaceRegions();
        Build(regions, connectivity, mesh, Vector3d.Zero, horizontal, (Fixed64)8,
            Fixed64.Quarter, new[] { 7, 2, 6, 3 });
        Assert.Equal(2, regions.RegionCount);
        for (int region = 0; region < regions.RegionCount; region++)
        {
            Assert.Equal(Vector3d.Down, regions.GetNormal(region));
            Assert.Equal(4, regions.GetSampleCount(region));
            for (int sample = 0; sample < regions.GetSampleCount(region); sample++)
            {
                Assert.True(regions.TryGetSample(region, sample, out FixedContactAnchors contact));
                Vector3d p = contact.FirstAnchor.LocalPoint;
                Assert.True(region == 0 ? p.X <= -Fixed64.Half : p.X >= Fixed64.Half);
                Assert.Equal(Fixed64.Zero, p.Y);
                Assert.InRange(FixedMath.Abs(p.Z).m_rawValue, 0L, Fixed64.Quarter.m_rawValue);
                if (sample == 0) Assert.Equal(Fixed64.Quarter, contact.Depth);
            }
        }
    }

    [Fact]
    public void AxialYaw_PreservesAuthoredCoverageAndWorldBoundaryWitnesses()
    {
        PhysicsMesh mesh = Quad(false, false, 4);
        var connectivity = new MeshSurfaceConnectivity();
        var baseline = new MeshCapsuleFaceRegions();
        var yawed = new MeshCapsuleFaceRegions();
        var yaw = new FixedQuaternion(Fixed64.Zero, Fixed64.FromFraction(3, 5), Fixed64.Zero, Fixed64.FromFraction(4, 5));
        foreach (Fixed64 length in new[] { Fixed64.Zero, Fixed64.Two })
        {
            Build(baseline, connectivity, mesh, Vector3d.Zero, FixedQuaternion.Identity, length, Fixed64.One, new[] { 0, 1 });
            Build(yawed, connectivity, mesh, Vector3d.Zero, yaw, length, Fixed64.One, new[] { 1, 0 });
            Assert.Equal(1, baseline.GetSampleCount(0));
            Assert.Equal(baseline.GetSampleCount(0), yawed.GetSampleCount(0));
            for (int sample = 0; sample < baseline.GetSampleCount(0); sample++)
            {
                Assert.True(baseline.TryGetSample(0, sample, out FixedContactAnchors a));
                Assert.True(yawed.TryGetSample(0, sample, out FixedContactAnchors b));
                Assert.Equal(a.FirstAnchor.LocalPoint, b.FirstAnchor.LocalPoint);
                Assert.Equal(a.Depth, b.Depth);
                Assert.True(a.SecondAnchor.TryGetPoint(out Vector3d first));
                Assert.True(b.SecondAnchor.TryGetPoint(out Vector3d second));
                Vector3d difference = first - second;
                Assert.InRange(FixedMath.Abs(difference.X).m_rawValue, 0L, 2L);
                Assert.InRange(FixedMath.Abs(difference.Y).m_rawValue, 0L, 2L);
                Assert.InRange(FixedMath.Abs(difference.Z).m_rawValue, 0L, 2L);
            }
        }
    }

    [Fact]
    public void SubdividedClippedSection_PreservesFinitePerimeterCoverage()
    {
        Fixed64 half = Fixed64.Quarter;
        Vector3d[] corners = { new(-half, Fixed64.Zero, -half), new(half, Fixed64.Zero, -half),
            new(half, Fixed64.Zero, half), new(-half, Fixed64.Zero, half) };
        var coarse = new PhysicsMesh(corners, new[] { 0, 1, 2, 0, 2, 3 },
            Vector3d.Zero, FixedQuaternion.Identity, MeshColliderMode.Concave);
        var refined = new PhysicsMesh(new[] { corners[0], corners[1], corners[2], corners[3], Vector3d.Zero },
            new[] { 0,1,4, 1,2,4, 2,3,4, 3,0,4 }, Vector3d.Zero, FixedQuaternion.Identity, MeshColliderMode.Concave);
        FixedQuaternion rotation = FixedQuaternion.LookRotation(new Vector3d(1, 0, 1).Normalized,
            new Vector3d(1, 1, -1).Normalized);
        var connectivity = new MeshSurfaceConnectivity();
        var baseline = new MeshCapsuleFaceRegions();
        var subdivided = new MeshCapsuleFaceRegions();
        Build(baseline, connectivity, coarse, Vector3d.Zero, rotation, (Fixed64)8, Fixed64.Half, new[] { 0, 1 });
        Build(subdivided, connectivity, refined, Vector3d.Zero, rotation, (Fixed64)8, Fixed64.Half, new[] { 3, 1, 0, 2 });
        Assert.True(subdivided.HasBoundaryIntersection);
        Assert.Equal(1, subdivided.RegionCount);
        Assert.Equal(4, subdivided.GetSampleCount(0));
        for (int sample = 0; sample < subdivided.GetSampleCount(0); sample++)
        {
            Assert.True(baseline.TryGetSample(0, sample, out FixedContactAnchors expected));
            Assert.True(subdivided.TryGetSample(0, sample, out FixedContactAnchors actual));
            Assert.Equal(expected.FirstAnchor.LocalPoint, actual.FirstAnchor.LocalPoint);
            Assert.Equal(expected.Depth, actual.Depth);
            Assert.Equal(expected.Normal, actual.Normal);
            Vector3d p = actual.FirstAnchor.LocalPoint;
            Assert.InRange(p.X.m_rawValue, -half.m_rawValue, half.m_rawValue);
            Assert.InRange(p.Z.m_rawValue, -half.m_rawValue, half.m_rawValue);
            Assert.Equal(Fixed64.Zero, p.Y);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RefinedFaceInsideTheSection_PreservesItsInteriorMaximumAndFiniteCoverage(bool sphere)
    {
        Fixed64 half = Fixed64.Quarter;
        Vector3d[] corners = { new(-half, Fixed64.Zero, -half), new(half, Fixed64.Zero, -half),
            new(half, Fixed64.Zero, half), new(-half, Fixed64.Zero, half) };
        var coarse = new PhysicsMesh(corners, new[] { 0, 1, 2, 0, 2, 3 },
            Vector3d.Zero, FixedQuaternion.Identity, MeshColliderMode.Concave);
        var refined = new PhysicsMesh(new[] { corners[0], corners[1], corners[2], corners[3], Vector3d.Zero },
            new[] { 0,1,4, 1,2,4, 2,3,4, 3,0,4 }, Vector3d.Zero, FixedQuaternion.Identity, MeshColliderMode.Concave);
        var connectivity = new MeshSurfaceConnectivity();
        var baseline = new MeshCapsuleFaceRegions();
        var subdivided = new MeshCapsuleFaceRegions();
        Fixed64 length = sphere ? Fixed64.Zero : Fixed64.Two;
        Build(baseline, connectivity, coarse, Vector3d.Zero, FixedQuaternion.Identity, length, Fixed64.One, new[] { 0, 1 });
        Build(subdivided, connectivity, refined, Vector3d.Zero, FixedQuaternion.Identity, length, Fixed64.One, new[] { 3, 1, 0, 2 });
        Assert.True(subdivided.HasBoundaryIntersection);
        Assert.Equal(1, subdivided.RegionCount);
        Assert.Equal(4, subdivided.GetSampleCount(0));
        for (int sample = 0; sample < subdivided.GetSampleCount(0); sample++)
        {
            Assert.True(baseline.TryGetSample(0, sample, out FixedContactAnchors expected));
            Assert.True(subdivided.TryGetSample(0, sample, out FixedContactAnchors actual));
            Assert.Equal(expected.FirstAnchor, actual.FirstAnchor);
            Assert.Equal(expected.SecondAnchor, actual.SecondAnchor);
            Assert.Equal(expected.Depth, actual.Depth);
            Assert.Equal(Vector3d.Down, actual.Normal);
            Assert.Equal(Fixed64.Zero, actual.FirstAnchor.LocalPoint.Y);
            Assert.InRange(actual.FirstAnchor.LocalPoint.X.m_rawValue, -half.m_rawValue, half.m_rawValue);
            Assert.InRange(actual.FirstAnchor.LocalPoint.Z.m_rawValue, -half.m_rawValue, half.m_rawValue);
            if (sample == 0)
            {
                Assert.Equal(Vector3d.Zero, actual.FirstAnchor.LocalPoint);
                Assert.Equal(sphere ? Fixed64.One : Fixed64.Two, actual.Depth);
            }
        }
    }

    [Fact]
    public void UniqueAxialSupport_WithOddCoreLength_PreservesItsHalfRawEndpoint()
    {
        PhysicsMesh mesh = Quad(false, false, 4);
        var connectivity = new MeshSurfaceConnectivity();
        var regions = new MeshCapsuleFaceRegions();
        Fixed64 length = Fixed64.Two + Fixed64.FromRaw(1);
        Build(regions, connectivity, mesh, Vector3d.Zero, FixedQuaternion.Identity, length, Fixed64.One, new[] { 1, 0 });
        Assert.Equal(1, regions.GetSampleCount(0));
        Assert.True(regions.TryGetSample(0, 0, out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Zero, contact.FirstAnchor.LocalPoint);
        Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.Equal(Fixed64.Two, contact.Depth);
        // Doubling before final materialization makes the half-raw endpoint
        // visible; rounding the core before adding its radius would lose it.
        Assert.True(contact.SecondAnchor.TryGetScaledOffsetFrom(contact.FirstAnchor, Fixed64.Two, out Vector3d doubled));
        Assert.Equal(Vector3d.Up * (Fixed64.FromRaw(4 * Fixed64.One.m_rawValue + 1)), doubled);
    }

    [Fact]
    public void ReusedSectionOwner_AfterTiltAndBoundaryContact_RestoresItsUniqueSupport()
    {
        PhysicsMesh mesh = Quad(false, false, 4);
        var connectivity = new MeshSurfaceConnectivity();
        var regions = new MeshCapsuleFaceRegions();
        Build(regions, connectivity, mesh, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One, new[] { 0, 1 });
        Assert.Equal(1, regions.GetSampleCount(0));
        Assert.True(regions.TryGetSample(0, 0, out FixedContactAnchors expected));

        FixedQuaternion tilted = FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.FromFraction(1, 64));
        Build(regions, connectivity, mesh, Vector3d.Zero, tilted, Fixed64.Two, Fixed64.One, new[] { 1, 0 });
        Assert.Equal(4, regions.GetSampleCount(0));
        Assert.True(regions.TryGetSample(0, 0, out FixedContactAnchors tiltedContact));
        Assert.Equal(Vector3d.Down, tiltedContact.Normal);
        Assert.True(tiltedContact.Depth > Fixed64.One && tiltedContact.Depth <= Fixed64.Two);

        Build(regions, connectivity, mesh, new Vector3d(4, 0, 3), FixedQuaternion.Identity,
            Fixed64.Two, Fixed64.One, new[] { 0, 1 });
        Assert.True(regions.HasBoundaryIntersection);
        Assert.Equal(4, regions.GetSampleCount(0));
        Build(regions, connectivity, mesh, Vector3d.Zero, FixedQuaternion.Identity,
            Fixed64.Two, Fixed64.One, new[] { 1, 0 });
        Assert.Equal(1, regions.GetSampleCount(0));
        Assert.True(regions.TryGetSample(0, 0, out FixedContactAnchors restored));
        Assert.Equal(expected.FirstAnchor, restored.FirstAnchor);
        Assert.Equal(expected.SecondAnchor, restored.SecondAnchor);
        Assert.Equal(expected.Normal, restored.Normal);
        Assert.Equal(expected.Depth, restored.Depth);
        Assert.False(regions.TryGetSample(0, 1, out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DenseFloor_WithOnlyItsCentralFacesAdmitted_PreservesTheUniqueSupport(bool sphere)
    {
        int[] coordinates = { -4, -1, 1, 4 };
        var vertices = new Vector3d[16];
        var triangles = new int[54];
        for (int z = 0; z < 4; z++)
            for (int x = 0; x < 4; x++)
                vertices[z * 4 + x] = new Vector3d(coordinates[x], 0, coordinates[z]);
        int index = 0;
        for (int z = 0; z < 3; z++)
            for (int x = 0; x < 3; x++)
            {
                int a = z * 4 + x;
                triangles[index++] = a; triangles[index++] = a + 1; triangles[index++] = a + 5;
                triangles[index++] = a; triangles[index++] = a + 5; triangles[index++] = a + 4;
            }
        var dense = new PhysicsMesh(vertices, triangles, Vector3d.Zero, FixedQuaternion.Identity, MeshColliderMode.Concave);
        var connectivity = new MeshSurfaceConnectivity();
        var coarseRegions = new MeshCapsuleFaceRegions();
        var denseRegions = new MeshCapsuleFaceRegions();
        Fixed64 length = sphere ? Fixed64.Zero : Fixed64.Two;
        Build(coarseRegions, connectivity, Quad(false, false, 4), Vector3d.Zero,
            FixedQuaternion.Identity, length, Fixed64.Half, new[] { 0, 1 });
        // The plane section lies strictly inside the central [-1,1] cell.
        // Outer floor triangles are outside the capsule and are not admitted.
        Build(denseRegions, connectivity, dense, Vector3d.Zero, FixedQuaternion.Identity,
            length, Fixed64.Half, new[] { 9, 8 });
        Assert.Equal(1, denseRegions.RegionCount);
        Assert.False(connectivity.IsCompleteSingleRegion(dense));
        Assert.False(denseRegions.HasBoundaryIntersection);
        Assert.Equal(1, denseRegions.GetSampleCount(0));
        Assert.True(coarseRegions.TryGetSample(0, 0, out FixedContactAnchors expected));
        Assert.True(denseRegions.TryGetSample(0, 0, out FixedContactAnchors actual));
        Assert.Equal(Vector3d.Zero, actual.FirstAnchor.LocalPoint);
        Assert.Equal(sphere ? Fixed64.Half : Fixed64.One + Fixed64.Half, actual.Depth);
        Assert.Equal(expected.FirstAnchor, actual.FirstAnchor);
        Assert.Equal(expected.SecondAnchor, actual.SecondAnchor);
        Assert.Equal(expected.Normal, actual.Normal);
        Assert.Equal(expected.Depth, actual.Depth);
    }

    [Fact]
    public void RepeatedBuildAndFinalRayReconstruction_AllocateZeroAfterWarmup()
    {
        PhysicsMesh mesh = Quad(false, false, 4);
        var connectivity = new MeshSurfaceConnectivity();
        var regions = new MeshCapsuleFaceRegions();
        int[] admitted = { 0, 1 };
        bool valid = true;
        long bytes = AllocationTestHelper.MeasureSteadyState(() =>
        {
            Build(regions, connectivity, mesh, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One, admitted);
            valid &= regions.TryGetSample(0, 0, out _);
        }, warmupIterations: 8, stabilizationIterations: 4, measurementIterations: 8);
        Assert.True(valid);
        Assert.Equal(0, bytes);
    }

    private static void Build(MeshCapsuleFaceRegions regions, MeshSurfaceConnectivity connectivity,
        PhysicsMesh mesh, Vector3d center, FixedQuaternion rotation, Fixed64 length, Fixed64 radius, ReadOnlySpan<int> admitted)
    {
        connectivity.BuildCapsule(mesh, 0, center, rotation, length, radius, admitted);
        regions.Build(mesh, 0, center, rotation, length, radius, connectivity);
    }

    private static PhysicsMesh Quad(bool diagonal, bool winding, int extent)
    {
        int[] triangles = diagonal ? new[] { 0, 1, 3, 1, 2, 3 } : new[] { 0, 1, 2, 0, 2, 3 };
        if (winding)
            for (int i = 0; i < triangles.Length; i += 3)
                (triangles[i + 1], triangles[i + 2]) = (triangles[i + 2], triangles[i + 1]);
        return new PhysicsMesh(new[] { new Vector3d(-extent, 0, -extent), new Vector3d(extent, 0, -extent),
            new Vector3d(extent, 0, extent), new Vector3d(-extent, 0, extent) }, triangles,
            Vector3d.Zero, FixedQuaternion.Identity, MeshColliderMode.Concave);
    }
}
