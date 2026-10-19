using System;
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests;

public sealed class MeshCapsuleJoinedFaceRegionsTests
{
    [Theory]
    [InlineData("Pose")]
    [InlineData("Scale")]
    [InlineData("Replacement")]
    [InlineData("Lifetime")]
    [InlineData("Owner")]
    [InlineData("Filter")]
    [InlineData("Reorder")]
    public void ReusedBoundaryPreparation_ShouldFollowCommittedMembersAndGeometry(string mutation)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider left = Quad(context, (Fixed64)(-4), Fixed64.Zero);
        var right = new LSMeshCollider(new[] { new Vector3d(0, 0, -4), new(4, 0, -4), new(4, 0, 4), new(0, 0, 4),
            new(0, 8, -4), new(4, 8, -4), new(4, 8, 4), new(0, 8, 4) },
            new[] { 0, 2, 1, 0, 3, 2, 4, 6, 5, 4, 7, 6 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        right.InitializeWithNoBody(new TestMatterAgent(context));
        var members = new[] { (left, 0), (right, 0) };
        Vector3d center = new(Fixed64.Zero, Fixed64.Quarter, Fixed64.One - Fixed64.Quarter);
        var reused = new MeshCapsuleFaceRegions();
        _ = Snapshot(reused);
        switch (mutation)
        {
            case "Pose": right.Position = new(Fixed64.Half, Fixed64.Zero, Fixed64.Zero); break;
            case "Scale": right.Transform.LocalScale = new(Fixed64.One, Fixed64.One, Fixed64.Quarter); right.Simulate(); break;
            case "Replacement": members = new[] { (left, 0), (Rectangle(context, Fixed64.Zero, (Fixed64)4, -Fixed64.One, Fixed64.One), 0) }; break;
            case "Lifetime":
                right.Deactivate();
                right.InitializeWithNoBody(new TestMatterAgent(context,
                    new FixedTransform(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One)));
                break;
            case "Owner": members = new[] { (left, 0), (right, 2) }; break;
            case "Filter": members = new[] { (left, 0) }; break;
            case "Reorder": members = new[] { (right, 0), (left, 0) }; break;
        }
        long[] expected = Snapshot(new MeshCapsuleFaceRegions());
        Assert.Equal(expected, Snapshot(reused));
        Assert.Equal(expected, Snapshot(reused));
        void Rebuild()
        {
            reused.BuildJoined(members, center, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One);
            for (int region = 0; region < reused.RegionCount; region++)
            for (int sample = 0; sample < reused.GetSampleCount(region); sample++)
                Assert.True(reused.TryGetJoinedSample(region, sample, out _, out _));
        }
        Assert.Equal(0, AllocationTestHelper.MeasureSteadyState(Rebuild, warmupIterations: 8, stabilizationIterations: 2, measurementIterations: 8));

        long[] Snapshot(MeshCapsuleFaceRegions faces)
        {
            faces.BuildJoined(members, center, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One);
            var raw = new System.Collections.Generic.List<long> { faces.RegionCount, faces.HasBoundaryIntersection ? 1 : 0 };
            for (int region = 0; region < faces.RegionCount; region++)
            {
                raw.Add(faces.GetSampleCount(region));
                for (int sample = 0; sample < faces.GetSampleCount(region); sample++)
                {
                    Assert.True(faces.TryGetJoinedSample(region, sample, out FixedContactAnchors contact, out LSMeshCollider? supplier));
                    Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
                    Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
                    raw.AddRange(new[] { supplier!.Id, contact.Depth.m_rawValue, contact.Normal.X.m_rawValue,
                        contact.Normal.Y.m_rawValue, contact.Normal.Z.m_rawValue, first.X.m_rawValue, first.Y.m_rawValue,
                        first.Z.m_rawValue, second.X.m_rawValue, second.Y.m_rawValue, second.Z.m_rawValue });
                }
            }
            return raw.ToArray();
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ContainedJoinedSection_ShouldMatchContinuousSupportAcrossSignedOffsets(bool sphere, bool below)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider left = Quad(context, (Fixed64)(-4), Fixed64.Zero), right = Quad(context, Fixed64.Zero, (Fixed64)4);
        LSMeshCollider continuous = Quad(context, (Fixed64)(-4), (Fixed64)4);
        Vector3d center = new(Fixed64.Zero, below ? -Fixed64.Quarter : Fixed64.Quarter, Fixed64.Zero);
        Fixed64 core = sphere ? Fixed64.Zero : Fixed64.Two;
        var connectivity = new MeshSurfaceConnectivity();
        connectivity.BuildCapsule(continuous.Mesh, 0, center, FixedQuaternion.Identity, core, Fixed64.One, new[] { 0, 1 });
        var expected = new MeshCapsuleFaceRegions();
        expected.Build(continuous.Mesh, 0, center, FixedQuaternion.Identity, core, Fixed64.One, connectivity);
        var joined = new MeshCapsuleFaceRegions();
        joined.BuildJoined(new[] { (right, 0), (left, 0) }, center, FixedQuaternion.Identity, core, Fixed64.One);
        Assert.Equal(1, joined.RegionCount);
        Assert.Equal(1, joined.GetSampleCount(0));
        Assert.False(joined.HasBoundaryIntersection);
        Assert.True(expected.TryGetSample(0, 0, out FixedContactAnchors first));
        Assert.True(joined.TryGetJoinedSample(0, 0, out FixedContactAnchors second, out LSMeshCollider? supplier));
        Assert.NotNull(supplier);
        Assert.Equal((sphere ? Fixed64.One : Fixed64.Two) - Fixed64.Quarter, second.Depth);
        Assert.Equal(below ? Vector3d.Down : Vector3d.Up, second.Normal);
        Assert.Equal(first.Depth, second.Depth);
        Assert.Equal(first.Normal, second.Normal);
        Assert.True(first.FirstAnchor.TryGetPoint(out Vector3d firstA));
        Assert.True(second.FirstAnchor.TryGetPoint(out Vector3d secondA));
        Assert.True(first.SecondAnchor.TryGetPoint(out Vector3d firstB));
        Assert.True(second.SecondAnchor.TryGetPoint(out Vector3d secondB));
        Assert.Equal(firstA, secondA);
        Assert.Equal(firstB, secondB);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void UnadmittedMember_ShouldNotChangeTheJoinedSupport(bool angled, bool affine)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider left = Quad(context, (Fixed64)(-4), Fixed64.Zero), right = Quad(context, Fixed64.Zero, (Fixed64)4);
        LSMeshCollider far = Quad(context, (Fixed64)10, (Fixed64)12);
        if (affine)
            foreach (LSMeshCollider member in new[] { left, right, far })
            {
                member.Transform.LocalScale = new(Fixed64.FromRaw(2863311531), Fixed64.One, Fixed64.One);
                member.Simulate();
            }
        var joined = new MeshCapsuleFaceRegions();
        FixedQuaternion rotation = angled ? FixedQuaternion.LookRotation(
            new Vector3d(1, 0, 1).Normalized, new Vector3d(1, 1, -1).Normalized) : FixedQuaternion.Identity;
        joined.BuildJoined(new[] { (far, 0), (right, 0), (left, 0) }, new Vector3d(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero),
            rotation, angled ? Fixed64.Two : Fixed64.Zero, Fixed64.One);
        Assert.Equal(1, joined.RegionCount);
        Assert.Equal(angled ? 4 : 1, joined.GetSampleCount(0));
        Assert.False(joined.HasBoundaryIntersection);
        Assert.True(joined.TryGetJoinedSample(0, 0, out FixedContactAnchors contact, out LSMeshCollider? supplier));
        Assert.True(supplier == left || supplier == right);
        Assert.True(contact.Depth > Fixed64.Zero);
        if (!angled) Assert.Equal(Fixed64.One - Fixed64.Quarter, contact.Depth);
        Assert.Equal(Vector3d.Up, contact.Normal);
    }

    [Fact]
    public void SharedWholeEdgeOutsideTheFiniteCapsule_ShouldNotJoinItsSeparatedSections()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider upper = Bent(false), lower = Bent(true);
        var joined = new MeshCapsuleFaceRegions();
        joined.BuildJoined(new[] { (upper, 0), (lower, 0) }, new Vector3d(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero),
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One);
        Assert.Equal(2, joined.RegionCount);
        Assert.True(joined.HasBoundaryIntersection);
        for (int region = 0; region < joined.RegionCount; region++)
        {
            Assert.True(joined.TryGetJoinedSample(region, 0, out FixedContactAnchors contact, out LSMeshCollider? supplier));
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
            Assert.True(FixedMath.Abs(first.Z) >= Fixed64.Quarter);
            Assert.Same(first.Z > Fixed64.Zero ? upper : lower, supplier);
        }

        LSMeshCollider Bent(bool below)
        {
            Vector3d[] vertices = { new((Fixed64)(-4), Fixed64.Zero, Fixed64.Quarter), new((Fixed64)3, Fixed64.Zero, Fixed64.Quarter), new(3, 0, 0),
                new(4, 0, 0), new((Fixed64)4, Fixed64.Zero, Fixed64.Quarter), new(4, 0, 4), new(-4, 0, 4) };
            if (below) for (int vertex = 0; vertex < vertices.Length; vertex++) vertices[vertex].Z = -vertices[vertex].Z;
            var collider = new LSMeshCollider(vertices, new[] { 6, 0, 1, 6, 1, 4, 6, 4, 5, 1, 2, 3, 1, 3, 4 },
                MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
            collider.InitializeWithNoBody(new TestMatterAgent(context));
            return collider;
        }
    }

    [Fact]
    public void UnselectedNeighborOwner_ShouldRetainTheSelectedSurfaceRim()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider left = Quad(context, (Fixed64)(-4), Fixed64.Zero);
        var right = new LSMeshCollider(new[] { new Vector3d(0, 0, -4), new(4, 0, -4), new(4, 0, 4), new(0, 0, 4),
            new(0, 8, -4), new(4, 8, -4), new(4, 8, 4), new(0, 8, 4) },
            new[] { 0, 2, 1, 0, 3, 2, 4, 6, 5, 4, 7, 6 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        right.InitializeWithNoBody(new TestMatterAgent(context));
        var joined = new MeshCapsuleFaceRegions();
        joined.BuildJoined(new[] { (left, 0), (right, 2) }, new Vector3d(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero),
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One);
        Assert.Equal(1, joined.RegionCount);
        Assert.True(joined.HasBoundaryIntersection);
        Assert.True(joined.GetSampleCount(0) > 1);
        for (int index = 0; index < joined.GetSampleCount(0); index++)
        {
            Assert.True(joined.TryGetJoinedSample(0, index, out FixedContactAnchors contact, out LSMeshCollider? supplier));
            Assert.Same(left, supplier);
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
            Assert.True(first.X <= Fixed64.Zero);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void JoinedInterior_ShouldPreserveTheContinuousSupportAndItsSupplyingMember(bool reverse)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider left = Quad(context, (Fixed64)(-4), Fixed64.Zero), right = Quad(context, Fixed64.Zero, (Fixed64)4);
        var surfaces = reverse ? new[] { (right, 0), (left, 0) } : new[] { (left, 0), (right, 0) };
        var faces = new MeshCapsuleFaceRegions();
        Vector3d center = new(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero);
        faces.BuildJoined(surfaces, center, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One);
        Assert.Equal(1, faces.RegionCount);
        Assert.Equal(1, faces.GetSampleCount(0));
        Assert.True(faces.TryGetJoinedSample(0, 0, out FixedContactAnchors contact, out LSMeshCollider? source));
        Assert.Same(left, source);
        Assert.Equal(Fixed64.One - Fixed64.Quarter, contact.Depth);
        Assert.Equal(Vector3d.Up, contact.Normal);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.Equal(Vector3d.Zero, first);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.Quarter - Fixed64.One, Fixed64.Zero), second);
        Assert.False(faces.TryGetJoinedSample(-1, 0, out _, out _));
        Assert.False(faces.TryGetJoinedSample(0, 1, out _, out _));
        AssertWholeShapeExit(surfaces, center, Vector3d.Up, Fixed64.Zero, Fixed64.One, contact);
    }

    [Fact]
    public void ARealGap_ShouldRetainSeparateFiniteRegions()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider left = Quad(context, (Fixed64)(-4), -Fixed64.Quarter), right = Quad(context, Fixed64.Quarter, (Fixed64)4);
        var faces = new MeshCapsuleFaceRegions();
        faces.BuildJoined(new[] { (left, 0), (right, 0) }, Vector3d.Zero, FixedQuaternion.Identity,
            Fixed64.Zero, Fixed64.One);
        Assert.Equal(2, faces.RegionCount);
        for (int region = 0; region < 2; region++)
        {
            Assert.True(faces.TryGetJoinedSample(region, 0, out var contact, out var source));
            Assert.NotNull(source);
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d point));
            Assert.True(FixedMath.Abs(point.X) >= Fixed64.Quarter);
        }
    }

    [Theory]
    [InlineData(false, false, false)]
    [InlineData(false, true, false)]
    [InlineData(true, false, false)]
    [InlineData(true, true, false)]
    [InlineData(false, false, true)]
    [InlineData(true, true, true)]
    public void IndependentlyFramedOppositeWindingMembers_ShouldRetainExactWorldSupport(bool limits, bool reverse, bool flippedNormal)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Vector3d origin = limits ? new(Fixed64.MaxValue - (Fixed64)16, Fixed64.MinValue + (Fixed64)16,
            Fixed64.MaxValue - (Fixed64)16) : Vector3d.Zero;
        LSMeshCollider left = Rectangle(context, (Fixed64)(-4), Fixed64.Zero, (Fixed64)(-4), (Fixed64)4,
            new FixedTransform(origin, FixedQuaternion.Identity, Vector3d.One));
        FixedQuaternion quarter = flippedNormal ? new(Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero)
            : new FixedQuaternion(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.One).Normalized;
        LSMeshCollider right = Rectangle(context, flippedNormal ? (Fixed64)(-2) : (Fixed64)(-4),
            flippedNormal ? Fixed64.Two : (Fixed64)4, flippedNormal ? (Fixed64)(-4) : (Fixed64)(-2), flippedNormal ? (Fixed64)4 : Fixed64.Two,
            new FixedTransform(origin + new Vector3d(2, 0, 0), quarter, Vector3d.One), reverseWinding: true);
        var faces = new MeshCapsuleFaceRegions();
        var surfaces = reverse ? new[] { (right, 1), (left, 1) } : new[] { (left, 1), (right, 1) };
        faces.BuildJoined(surfaces,
            origin + new Vector3d(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One);
        Assert.Equal(1, faces.RegionCount);
        Assert.Equal(1, faces.GetSampleCount(0));
        Assert.False(faces.HasBoundaryIntersection);
        Assert.True(faces.TryGetJoinedSample(0, 0, out FixedContactAnchors contact, out LSMeshCollider? supplier));
        Assert.Same(left, supplier);
        Assert.Equal(Vector3d.Up, contact.Normal);
        Assert.Equal(Fixed64.One - Fixed64.Quarter, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.Equal(origin, first);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(origin + new Vector3d(Fixed64.Zero, Fixed64.Quarter - Fixed64.One, Fixed64.Zero), second);
        AssertWholeShapeExit(surfaces, origin + new Vector3d(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero),
            Vector3d.Up, Fixed64.Zero, Fixed64.One, contact);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void NonzeroCoreAcrossAJoinedSpan_ShouldPreservePointSectionAndPositiveRadiusDepth(bool zeroRadius)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider left = Quad(context, (Fixed64)(-4), Fixed64.Zero), right = Quad(context, Fixed64.Zero, (Fixed64)4);
        var faces = new MeshCapsuleFaceRegions();
        faces.BuildJoined(new[] { (left, 0), (right, 0) }, new Vector3d(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero),
            FixedQuaternion.Identity, Fixed64.Two, zeroRadius ? Fixed64.Zero : Fixed64.One);
        Assert.Equal(1, faces.RegionCount);
        Assert.True(faces.TryGetJoinedSample(0, 0, out FixedContactAnchors contact, out LSMeshCollider? supplier));
        Assert.Same(left, supplier);
        Assert.Equal(Vector3d.Up, contact.Normal);
        Assert.Equal((zeroRadius ? Fixed64.One : Fixed64.Two) - Fixed64.Quarter, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.Equal(Vector3d.Zero, first);
        AssertWholeShapeExit(new[] { (left, 0), (right, 0) }, new Vector3d(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero),
            Vector3d.Up, Fixed64.Two, zeroRadius ? Fixed64.Zero : Fixed64.One, contact);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void HorizontalCoreSamples_ShouldRetainSeedAndNonseedSupplyingMaterials(bool independentlyFramed)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider left = Quad(context, (Fixed64)(-4), Fixed64.Zero);
        FixedQuaternion rightRotation = new FixedQuaternion(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.One).Normalized;
        LSMeshCollider right = independentlyFramed
            ? Rectangle(context, (Fixed64)(-4), (Fixed64)4, (Fixed64)(-2), Fixed64.Two,
                new FixedTransform(new Vector3d(2, 0, 0), rightRotation, Vector3d.One))
            : Quad(context, Fixed64.Zero, (Fixed64)4);
        left.Material = PhysicsMaterial.Frictionless; right.Material = PhysicsMaterial.Bouncy;
        FixedQuaternion quarter = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized;
        var faces = new MeshCapsuleFaceRegions();
        faces.BuildJoined(new[] { (right, 0), (left, 0) }, new Vector3d(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero),
            quarter, Fixed64.Two, Fixed64.One);
        Assert.Equal(1, faces.RegionCount);
        Assert.InRange(faces.GetSampleCount(0), 2, ContactManifold.MaxContactsPerGroup);
        bool suppliedLeft = false, suppliedRight = false;
        for (int sample = 0; sample < faces.GetSampleCount(0); sample++)
        {
            Assert.True(faces.TryGetJoinedSample(0, sample, out FixedContactAnchors contact, out LSMeshCollider? supplier));
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
            Assert.True(contact.SecondAnchor.TryGetPoint(out _));
            Assert.Equal(Vector3d.Up, contact.Normal);
            if (first.X < Fixed64.Zero)
            { Assert.Same(left, supplier); Assert.Equal(PhysicsMaterial.Frictionless, supplier!.Material); suppliedLeft = true; }
            if (first.X > Fixed64.Zero)
            { Assert.Same(right, supplier); Assert.Equal(PhysicsMaterial.Bouncy, supplier!.Material); suppliedRight = true; }
            if (sample == 0)
            {
                // The common capsule chart's negative Y endpoint is world +X after this quarter turn.
                Assert.Same(right, supplier);
                Assert.Equal(new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero), first);
                Assert.Equal(Fixed64.One - Fixed64.Quarter, contact.Depth);
                AssertWholeShapeExit(new[] { (left, 0), (right, 0) }, new Vector3d(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero),
                    Vector3d.Left, Fixed64.Two, Fixed64.One, contact);
            }
        }
        Assert.True(suppliedLeft && suppliedRight);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(4)]
    public void PartiallyJoinedEdge_ShouldKeepItsIntersectedExposedRemainder(int radius)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider left = Quad(context, (Fixed64)(-4), Fixed64.Zero);
        LSMeshCollider right = Rectangle(context, Fixed64.Zero, (Fixed64)4, -Fixed64.One, Fixed64.One);
        var faces = new MeshCapsuleFaceRegions();
        faces.BuildJoined(new[] { (left, 0), (right, 0) }, new Vector3d(Fixed64.Zero, Fixed64.Quarter, Fixed64.One - Fixed64.Quarter),
            FixedQuaternion.Identity, Fixed64.Zero, (Fixed64)radius);
        Assert.Equal(1, faces.RegionCount);
        Assert.True(faces.HasBoundaryIntersection);
        Assert.True(faces.GetSampleCount(0) > 1);
        bool retainedExposed = false;
        for (int sample = 0; sample < faces.GetSampleCount(0); sample++)
        {
            Assert.True(faces.TryGetJoinedSample(0, sample, out FixedContactAnchors contact, out LSMeshCollider? supplier));
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
            Assert.Equal(Vector3d.Up, contact.Normal);
            // A wide section may reduce away the short top edge while retaining a farther exposed rim.
            if (radius == 1 && first.Z == Fixed64.One && first.X > Fixed64.Zero)
            { Assert.Same(right, supplier); retainedExposed = true; }
            if (radius == 4 && sample > 0)
            {
                bool onExposedRim = supplier == left
                    ? first.X == (Fixed64)(-4) || first.Z == (Fixed64)(-4) || first.Z == (Fixed64)4
                        || first.X == Fixed64.Zero && (first.Z < -Fixed64.One || first.Z > Fixed64.One)
                    : first.X == (Fixed64)4 || first.Z == -Fixed64.One || first.Z == Fixed64.One;
                Assert.True(first.X >= (supplier == left ? (Fixed64)(-4) : Fixed64.Zero)
                    && first.X <= (supplier == left ? Fixed64.Zero : (Fixed64)4));
                Assert.True(first.Z >= (supplier == left ? (Fixed64)(-4) : -Fixed64.One)
                    && first.Z <= (supplier == left ? (Fixed64)4 : Fixed64.One));
                retainedExposed |= onExposedRim;
            }
        }
        Assert.True(retainedExposed);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SharedOuterEndpoint_ShouldChooseTheLowestSupplyingColliderRegardlessOfInputOrder(bool reverse)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider left = Quad(context, (Fixed64)(-4), Fixed64.Zero), right = Quad(context, Fixed64.Zero, (Fixed64)4);
        left.Material = PhysicsMaterial.Frictionless;
        right.Material = PhysicsMaterial.Bouncy;
        var members = reverse ? new[] { (right, 0), (left, 0) } : new[] { (left, 0), (right, 0) };
        Vector3d center = new(Fixed64.Zero, Fixed64.Quarter, (Fixed64)4);
        var faces = new MeshCapsuleFaceRegions();
        faces.BuildJoined(members, center, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One);
        Assert.Equal(1, faces.RegionCount);
        Assert.True(faces.HasBoundaryIntersection);
        Assert.InRange(faces.GetSampleCount(0), 2, 4);
        Assert.True(faces.TryGetJoinedSample(0, 0, out FixedContactAnchors contact, out LSMeshCollider? supplier));
        Assert.Same(left, supplier);
        Assert.Equal(PhysicsMaterial.Frictionless, supplier!.Material);
        Assert.Equal(Vector3d.Up, contact.Normal);
        Assert.Equal(Fixed64.One - Fixed64.Quarter, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(new Vector3d(0, 0, 4), first);
        Assert.Equal(new Vector3d(Fixed64.Zero, -contact.Depth, (Fixed64)4), second);
        AssertWholeShapeExit(members, center, Vector3d.Up, Fixed64.Zero, Fixed64.One, contact);
    }

    [Theory]
    [InlineData(0)] // Two quads meet only at one point.
    [InlineData(1)] // A one-raw gap keeps both finite boundaries.
    [InlineData(2)] // Duplicate opposite coverage is ambiguous.
    [InlineData(3)] // A folded incident sheet prevents deleting the shared edge.
    public void UncertifiedCoverage_ShouldKeepSeparateAdmittedRegions(int variant)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider left = Rectangle(context, (Fixed64)(-4), Fixed64.Zero, (Fixed64)(-4), variant == 0 ? Fixed64.Zero : (Fixed64)4);
        LSMeshCollider right = Rectangle(context, variant == 1 ? Fixed64.FromRaw(1) : Fixed64.Zero, (Fixed64)4,
            variant == 0 ? Fixed64.Zero : (Fixed64)(-4), (Fixed64)4);
        var surfaces = new[] { (left, 0), (right, 0) };
        if (variant == 2)
            surfaces = new[] { (left, 0), (right, 0), (Quad(context, Fixed64.Zero, (Fixed64)4), 0) };
        if (variant == 3)
        {
            LSMeshCollider folded = Mesh(context, new[] { new Vector3d(0, 0, -4), new(0, 4, -4), new(0, 4, 4), new(0, 0, 4) });
            surfaces = new[] { (left, 0), (right, 0), (folded, 0) };
        }
        var faces = new MeshCapsuleFaceRegions();
        faces.BuildJoined(surfaces, new Vector3d(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero),
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One);
        Assert.Equal(variant < 2 ? 2 : 3, faces.RegionCount);
        Assert.True(faces.HasBoundaryIntersection);
        for (int region = 0; region < faces.RegionCount; region++)
        {
            Assert.True(faces.GetSampleCount(region) > 0);
            for (int sample = 0; sample < faces.GetSampleCount(region); sample++)
            {
                Assert.True(faces.TryGetJoinedSample(region, sample, out FixedContactAnchors contact, out LSMeshCollider? supplier));
                Assert.NotNull(supplier);
                Assert.True(contact.FirstAnchor.TryGetPoint(out _));
                Assert.True(contact.SecondAnchor.TryGetPoint(out _));
            }
        }
    }

    [Fact]
    public void TransitiveJoins_ShouldRetainTheInteriorSupplyingMember()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider left = Quad(context, (Fixed64)(-4), -Fixed64.Quarter), right = Quad(context, Fixed64.Quarter, (Fixed64)4);
        LSMeshCollider middle = Quad(context, -Fixed64.Quarter, Fixed64.Quarter);
        var faces = new MeshCapsuleFaceRegions();
        faces.BuildJoined(new[] { (right, 0), (middle, 0), (left, 0) }, new Vector3d(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero),
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One);
        Assert.Equal(1, faces.RegionCount);
        Assert.False(faces.HasBoundaryIntersection);
        Assert.Equal(1, faces.GetSampleCount(0));
        Assert.True(faces.TryGetJoinedSample(0, 0, out FixedContactAnchors contact, out LSMeshCollider? supplier));
        Assert.Same(middle, supplier);
        Assert.Equal(Fixed64.One - Fixed64.Quarter, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.Equal(Vector3d.Zero, first);
    }

    [Fact]
    public void OverlappingMembersWithoutBoundaryHits_ShouldNotClaimAContainedUnionCertificate()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider first = Quad(context, (Fixed64)(-4), (Fixed64)4), second = Quad(context, (Fixed64)(-4), (Fixed64)4);
        var faces = new MeshCapsuleFaceRegions();
        faces.BuildJoined(new[] { (first, 0), (second, 0) }, new Vector3d(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero),
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One);
        Assert.Equal(2, faces.RegionCount);
        Assert.False(faces.HasBoundaryIntersection);
        for (int region = 0; region < faces.RegionCount; region++)
        {
            Assert.True(faces.GetSampleCount(region) > 1);
            Assert.True(faces.TryGetJoinedSample(region, 0, out FixedContactAnchors contact, out LSMeshCollider? supplier));
            Assert.Same(region == 0 ? first : second, supplier);
            Assert.Equal(Fixed64.One - Fixed64.Quarter, contact.Depth);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RepeatedColliderInputs_ShouldPrepareEachSurfaceOwnerOnceWithoutDuplicatingWholeMeshCoverage(bool duplicateOwner)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var left = new LSMeshCollider(new[] { new Vector3d(-4, 0, -4), new(0, 0, -4), new(0, 0, 4), new(-4, 0, 4),
            new(-4, 8, -4), new(0, 8, -4), new(0, 8, 4), new(-4, 8, 4) },
            new[] { 0, 2, 1, 0, 3, 2, 4, 6, 5, 4, 7, 6 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        left.InitializeWithNoBody(new TestMatterAgent(context));
        LSMeshCollider right = Quad(context, Fixed64.Zero, (Fixed64)4);
        Assert.Equal(left.Mesh.GetManifoldSurfaceOwner(0), left.Mesh.GetManifoldSurfaceOwner(1));
        Assert.NotEqual(left.Mesh.GetManifoldSurfaceOwner(0), left.Mesh.GetManifoldSurfaceOwner(2));
        var surfaces = new[] { (left, duplicateOwner ? 1 : 2), (right, 0), (left, 0) };
        var faces = new MeshCapsuleFaceRegions();
        Vector3d center = new(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero);
        faces.BuildJoined(surfaces, center, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One);
        Assert.Equal(1, faces.RegionCount);
        Assert.False(faces.HasBoundaryIntersection);
        Assert.Equal(1, faces.GetSampleCount(0));
        Assert.True(faces.TryGetJoinedSample(0, 0, out FixedContactAnchors contact, out LSMeshCollider? supplier));
        Assert.Same(left, supplier);
        Assert.Equal(Fixed64.One - Fixed64.Quarter, contact.Depth);
        AssertWholeShapeExit(surfaces, center, Vector3d.Up, Fixed64.Zero, Fixed64.One, contact);
    }

    [Fact]
    public void RebuildingMembership_ShouldClearStaleRegionsAndReusePreparedStorage()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider left = Quad(context, (Fixed64)(-4), Fixed64.Zero), right = Quad(context, Fixed64.Zero, (Fixed64)4);
        var members = new[] { (left, 0), (right, 0) };
        var faces = new MeshCapsuleFaceRegions();
        Assert.False(faces.TryGetJoinedSample(0, 0, out _, out _));
        Vector3d center = new(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero);
        void Rebuild()
        {
            faces.BuildJoined(members, center, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One);
            faces.BuildJoined(members.AsSpan(0, 1), center, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One);
            faces.BuildJoined(ReadOnlySpan<(LSMeshCollider, int)>.Empty, center, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One);
        }
        Assert.Equal(0, AllocationTestHelper.MeasureSteadyState(Rebuild, warmupIterations: 8, stabilizationIterations: 2, measurementIterations: 8));
        Assert.Equal(0, faces.RegionCount);
        Assert.False(faces.HasBoundaryIntersection);
        Assert.False(faces.TryGetJoinedSample(0, 0, out _, out _));
        faces.BuildJoined(members, center, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One);
        Assert.Equal(1, faces.RegionCount);
        Assert.Equal(1, faces.GetSampleCount(0));
        Assert.True(faces.TryGetJoinedSample(0, 0, out _, out LSMeshCollider? supplier));
        Assert.Same(left, supplier);
    }

    [Fact]
    public void OrdinaryBuildAfterJoinedBuild_ShouldDeclineStaleJoinedSamplesAndAllowReuse()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider left = Quad(context, (Fixed64)(-4), Fixed64.Zero), right = Quad(context, Fixed64.Zero, (Fixed64)4);
        var members = new[] { (left, 0), (right, 0) };
        Vector3d center = new(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero);
        var faces = new MeshCapsuleFaceRegions();
        faces.BuildJoined(members, center, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One);
        var connectivity = new MeshSurfaceConnectivity();
        Vector3d ordinaryCenter = new((Fixed64)(-2), Fixed64.Quarter, Fixed64.Zero);
        connectivity.BuildCapsule(left.Mesh, 0, ordinaryCenter, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, new[] { 0, 1 });
        faces.Build(left.Mesh, 0, ordinaryCenter, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, connectivity);
        Assert.True(faces.TryGetSample(0, 0, out _));
        Assert.False(faces.TryGetJoinedSample(0, 0, out _, out _));
        faces.BuildJoined(members, center, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One);
        Assert.Equal(1, faces.GetSampleCount(0));
        Assert.True(faces.TryGetJoinedSample(0, 0, out _, out LSMeshCollider? supplier));
        Assert.Same(left, supplier);
    }

    private static LSMeshCollider Quad(GravitasWorldContext context, Fixed64 start, Fixed64 end)
        => Rectangle(context, start, end, (Fixed64)(-4), (Fixed64)4);

    private static LSMeshCollider Rectangle(GravitasWorldContext context, Fixed64 start, Fixed64 end, Fixed64 near, Fixed64 far,
        FixedTransform? transform = null, bool reverseWinding = false)
        => Mesh(context, new[] { new Vector3d(start, Fixed64.Zero, near), new(end, Fixed64.Zero, near),
            new(end, Fixed64.Zero, far), new(start, Fixed64.Zero, far) }, transform, reverseWinding);

    private static LSMeshCollider Mesh(GravitasWorldContext context, Vector3d[] vertices, FixedTransform? transform = null,
        bool reverseWinding = false)
    {
        var mesh = new LSMeshCollider(vertices, reverseWinding ? new[] { 0, 1, 2, 0, 2, 3 } : new[] { 0, 2, 1, 0, 3, 2 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        mesh.InitializeWithNoBody(new TestMatterAgent(context, transform));
        return mesh;
    }

    private static void AssertWholeShapeExit(ReadOnlySpan<(LSMeshCollider Collider, int Triangle)> surfaces,
        Vector3d center, Vector3d axis, Fixed64 length, Fixed64 radius, FixedContactAnchors contact)
    {
        Vector3d touchingCenter = center + contact.Normal * contact.Depth;
        Vector3d separatedCenter = touchingCenter + contact.Normal * Fixed64.FromRaw(1);
        bool touching = false;
        foreach (var surface in surfaces)
        {
            PhysicsMesh mesh = surface.Collider.Mesh;
            int owner = mesh.GetManifoldSurfaceOwner(surface.Triangle);
            for (int triangle = 0; triangle < mesh.TriangleCount; triangle++)
            {
                if (mesh.GetManifoldSurfaceOwner(triangle) != owner) continue;
                mesh.GetLocalTriangleVertices(triangle, out Vector3d a, out Vector3d b, out Vector3d c);
                Assert.True(new FixedPointAnchor(mesh.Origin, mesh.Rotation, a).TryGetPoint(out a));
                Assert.True(new FixedPointAnchor(mesh.Origin, mesh.Rotation, b).TryGetPoint(out b));
                Assert.True(new FixedPointAnchor(mesh.Origin, mesh.Rotation, c).TryGetPoint(out c));
                var worldTriangle = new FixedTriangle(a, b, c);
                touching |= worldTriangle.DoesCenteredCapsuleOverlap(touchingCenter, axis, length, radius);
                Assert.False(worldTriangle.DoesCenteredCapsuleOverlap(separatedCenter, axis, length, radius));
            }
        }
        Assert.True(touching);
    }
}
