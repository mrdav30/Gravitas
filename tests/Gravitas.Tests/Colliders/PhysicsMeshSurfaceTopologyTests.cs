using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using System;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class PhysicsMeshSurfaceTopologyTests
{
    [Theory]
    [InlineData(0)] // Alternate diagonal.
    [InlineData(1)] // Triangle order, vertex order and winding.
    [InlineData(2)] // Duplicate authored vertices.
    [InlineData(3)] // Removable collinear boundary subdivision.
    public void EquivalentDomains_ShouldKeepCanonicalOrdinalIndependentOfAuthoredTopology(int variant)
    {
        Vector3d[] first = { new(-1, 0, -1), new(1, 0, -1), new(1, 0, 1), new(-1, 0, 1) };
        Vector3d[] second;
        int[] secondTriangles;
        if (variant == 0)
        {
            second = first;
            secondTriangles = new[] { 0, 1, 3, 1, 2, 3 };
        }
        else if (variant == 1)
        {
            second = new[] { first[2], first[0], first[3], first[1] };
            secondTriangles = new[] { 2, 0, 1, 0, 3, 1 };
        }
        else if (variant == 2)
        {
            second = new[] { first[0], first[1], first[2], first[0], first[2], first[3] };
            secondTriangles = new[] { 3, 4, 5, 0, 1, 2 };
        }
        else
        {
            second = new[] { first[0], new Vector3d(0, 0, -1), first[1], first[2], first[3], Vector3d.Zero };
            secondTriangles = new[] { 5, 0, 1, 5, 1, 2, 5, 2, 3, 5, 3, 4, 5, 4, 0 };
        }

        // Author the right surface first in the baseline, and last in the
        // variant. Comparing two ordinals exposes an authored-index ordering.
        PhysicsMesh baseline = TwoDomains(first, new[] { 0, 1, 2, 0, 2, 3 }, true);
        PhysicsMesh equivalent = TwoDomains(second, secondTriangles, false);
        baseline.GetCanonicalSurfaceOrdinal(0).Should().Be(1);
        baseline.GetCanonicalSurfaceOrdinal(2).Should().Be(0);
        for (int i = 0; i < equivalent.TriangleCount / 2; i++)
        {
            equivalent.GetCanonicalSurfaceOrdinal(i).Should().Be(baseline.GetCanonicalSurfaceOrdinal(2));
            equivalent.GetCanonicalSurfaceOrdinal(i + equivalent.TriangleCount / 2).Should()
                .Be(baseline.GetCanonicalSurfaceOrdinal(0));
        }
        ReadOnlySpan<int> canonical = equivalent.GetCanonicalSurfaceBoundaryVertexPairs(0);
        canonical.Length.Should().Be(8);
        for (int edge = 0; edge < canonical.Length; edge += 2)
        {
            Vector3d a = equivalent.ScaledLocalVertices[canonical[edge]];
            Vector3d b = equivalent.ScaledLocalVertices[canonical[edge + 1]];
            (a - b).MagnitudeSquared.Should().Be((Fixed64)4);
        }
    }

    [Fact]
    public void WeldedTopology_ShouldRetainEdgeNeighborsAndVertexIncidence()
    {
        Vector3d[] vertices = { new(-1, 0, -1), new(1, 0, -1), new(1, 0, 1),
            new(-1, 0, -1), new(1, 0, 1), new(-1, 0, 1), new(-2, 0, -1), new(-1, 0, -2) };
        PhysicsMesh mesh = Create(vertices, new[] { 0, 1, 2, 3, 4, 5, 0, 6, 7 });

        mesh.GetCoplanarTriangleNeighbors(0).ToArray().Should().Equal(1);
        mesh.GetCoplanarTriangleNeighbors(1).ToArray().Should().Equal(0);
        mesh.GetCoplanarTriangleNeighbors(2).IsEmpty.Should().BeTrue();
        mesh.GetWeldedTriangleVertexIndices(1).ToArray().Should().Equal(0, 2, 5);
        mesh.GetWeldedVertexTriangleIndices(0).ToArray().Should().Equal(0, 1, 2);
        mesh.GetWeldedVertexTriangleIndices(3).ToArray().Should().Equal(0, 1, 2);
        mesh.GetCanonicalSurfaceOrdinal(2).Should().NotBe(mesh.GetCanonicalSurfaceOrdinal(0));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CollinearSubdivision_ShouldNotChangeOrderingAgainstAnotherDomainSharingItsFirstCorner(bool subdivided)
    {
        Vector3d[] vertices = { new(-1, 0, -1), new(1, 0, -1), new(1, 0, 1), new(-1, 0, 1),
            new(-1, 0, 0), Vector3d.Zero, new(-Fixed64.One, Fixed64.Zero, Fixed64.Half), new(2, 0, 1) };
        int[] faces = subdivided
            ? new[] { 5, 0, 1, 5, 1, 2, 5, 2, 3, 5, 3, 4, 5, 4, 0, 0, 6, 7 }
            : new[] { 0, 1, 2, 0, 2, 3, 0, 4, 5 };
        if (!subdivided)
            vertices = new[] { vertices[0], vertices[1], vertices[2], vertices[3], vertices[6], vertices[7] };
        PhysicsMesh mesh = Create(vertices, faces);

        // Both domains start at the same exact corner. Without removing the
        // midpoint, the square's first segment sorts before the triangle's
        // half-length segment, reversing their geometric order.
        mesh.GetCanonicalSurfaceOrdinal(0).Should().Be(1);
        mesh.GetCanonicalSurfaceOrdinal(mesh.TriangleCount - 1).Should().Be(0);
    }

    [Fact]
    public void DeclinedAmbiguousPatch_ShouldKeepIndividualProvenanceWithoutTrustedNeighbors()
    {
        PhysicsMesh mesh = Create(new[] { Vector3d.Zero, Vector3d.Right, Vector3d.Forward,
            -Vector3d.Forward }, new[] { 0, 1, 2, 0, 1, 3 });
        mesh.GetCoplanarPatchId(0).Should().Be(-1);
        mesh.GetCanonicalSurfaceOrdinal(0).Should().NotBe(mesh.GetCanonicalSurfaceOrdinal(1));
        mesh.GetCoplanarTriangleNeighbors(0).IsEmpty.Should().BeTrue();
        mesh.GetWeldedVertexTriangleIndices(0).ToArray().Should().Equal(0, 1);
    }

    [Fact]
    public void RejectedScaleAfterTopologyPreparation_ShouldKeepCommittedSpansAndOrdinals()
    {
        Vector3d[] vertices = { new(0, 0, 0), new(Fixed64.One, Fixed64.Quarter, Fixed64.Zero),
            new((Fixed64)3, Fixed64.FromFraction(3, 4), Fixed64.One), new(0, 0, 1) };
        PhysicsMesh mesh = Create(vertices, new[] { 0, 1, 2, 0, 2, 3 });
        ReadOnlySpan<int> neighbors = mesh.GetCoplanarTriangleNeighbors(0);
        ReadOnlySpan<int> incidence = mesh.GetWeldedVertexTriangleIndices(0);
        int ordinal = mesh.GetCanonicalSurfaceOrdinal(0);
        int owner = mesh.GetManifoldSurfaceOwner(1);
        ReadOnlySpan<int> boundary = mesh.GetManifoldSurfaceBoundaryVertexPairs(1);
        int[] boundaryBefore = boundary.ToArray();
        ReadOnlySpan<int> canonical = mesh.GetCanonicalSurfaceBoundaryVertexPairs(1);
        int[] canonicalBefore = canonical.ToArray();
        Action prepare = () => mesh.PrepareTransformation(Vector3d.One, FixedQuaternion.Identity,
            new Vector3d(Fixed64.FromFraction(1, 3), Fixed64.One, (Fixed64)1000000),
            Vector3d.One, MeshInertiaPolicy.SurfaceApproximation);

        prepare.Should().Throw<ArgumentException>().WithMessage("*surface mass properties*");
        neighbors.ToArray().Should().Equal(1);
        incidence.ToArray().Should().Equal(0, 1);
        mesh.GetCanonicalSurfaceOrdinal(1).Should().Be(ordinal);
        mesh.GetCoplanarTriangleNeighbors(0).ToArray().Should().Equal(1);
        mesh.GetManifoldSurfaceOwner(1).Should().Be(owner);
        boundary.ToArray().Should().Equal(boundaryBefore);
        mesh.GetManifoldSurfaceBoundaryVertexPairs(1).ToArray().Should().Equal(boundaryBefore);
        canonical.ToArray().Should().Equal(canonicalBefore);
        mesh.GetCanonicalSurfaceBoundaryVertexPairs(1).ToArray().Should().Equal(canonicalBefore);
        mesh.UpdatePosition(Vector3d.One, FixedQuaternion.Identity);
        mesh.GetWeldedVertexTriangleIndices(0).ToArray().Should().Equal(0, 1);
    }

    [Fact]
    public void ExactDuplicateTriangles_ShouldShareGeometryProvenanceWithoutSuppressingAmbiguousEdges()
    {
        PhysicsMesh mesh = Create(new[] { Vector3d.Zero, Vector3d.Right, Vector3d.Forward },
            new[] { 0, 1, 2, 2, 1, 0 });
        mesh.GetCanonicalSurfaceOrdinal(0).Should().Be(0);
        mesh.GetCanonicalSurfaceOrdinal(1).Should().Be(0);
        mesh.GetCoplanarPatchId(0).Should().Be(-1);
        mesh.GetCoplanarTriangleNeighbors(0).IsEmpty.Should().BeTrue();
        mesh.GetWeldedVertexTriangleIndices(1).ToArray().Should().Equal(0, 1);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void NonSharedVertexInsideAnotherFace_ShouldDeclineMulticoverSurface(bool containingFaceFirst)
    {
        Vector3d[] vertices = { new(Fixed64.Zero, Fixed64.Zero, Fixed64.Quarter), new(-3,0,1),
            new(-2,0,-3), new(2,0,-3), new(3,0,1), Vector3d.Zero };
        int[] faces = { 5,0,2, 5,2,4, 5,4,1, 5,1,3, 5,3,0 };
        if (containingFaceFirst) faces = new[] { 5,4,1, 5,0,2, 5,2,4, 5,1,3, 5,3,0 };
        PhysicsMesh mesh = Create(vertices, faces);
        for (int triangle = 0; triangle < mesh.TriangleCount; triangle++)
        {
            mesh.GetManifoldSurfaceOwner(triangle).Should().Be(triangle);
            mesh.GetCoplanarTriangleNeighbors(triangle).IsEmpty.Should().BeTrue();
            mesh.GetManifoldSurfaceBoundaryVertexPairs(triangle).IsEmpty.Should().BeTrue();
        }
        mesh.GetCoplanarPatchId(0).Should().Be(0);
    }

    [Fact]
    public void PreparedCoplanarityChange_ShouldPublishMatchingNeighborsAndProvenanceTogether()
    {
        Vector3d[] vertices = { new(0, 0, 0), new(Fixed64.One, Fixed64.Quarter, Fixed64.Zero),
            new((Fixed64)3, Fixed64.FromFraction(3, 4), Fixed64.One), new(0, 0, 1) };
        PhysicsMesh mesh = Create(vertices, new[] { 0, 1, 2, 0, 2, 3 });
        mesh.PrepareTransformation(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(Fixed64.FromFraction(1, 3), Fixed64.One, Fixed64.One), Vector3d.One, null);
        mesh.GetCoplanarTriangleNeighbors(0).ToArray().Should().Equal(1);
        mesh.GetCanonicalSurfaceOrdinal(0).Should().Be(mesh.GetCanonicalSurfaceOrdinal(1));
        mesh.PublishPreparedTransformation();
        mesh.GetCoplanarTriangleNeighbors(0).IsEmpty.Should().BeTrue();
        mesh.GetCanonicalSurfaceOrdinal(0).Should().NotBe(mesh.GetCanonicalSurfaceOrdinal(1));
        mesh.GetWeldedVertexTriangleIndices(0).ToArray().Should().Equal(0, 1);
        mesh.UpdateTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        mesh.GetCoplanarTriangleNeighbors(0).ToArray().Should().Equal(1);
        mesh.GetCanonicalSurfaceOrdinal(0).Should().Be(mesh.GetCanonicalSurfaceOrdinal(1));
    }

    [Theory]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(8, false)]
    [InlineData(8, true)]
    public void SubdividedAnnulus_ShouldRetainItsHoleAndSharedOwner(int sideSegments, bool reverse)
    {
        PhysicsMesh mesh = Annulus(sideSegments, reverse);
        for (int triangle = 0; triangle < mesh.TriangleCount; triangle++)
        {
            mesh.GetManifoldSurfaceOwner(triangle).Should().Be(0);
            mesh.GetCanonicalSurfaceOrdinal(triangle).Should().Be(0);
        }
        // The trust/key certificate reduces corners, while runtime perimeter
        // access preserves all real segments and both oriented boundary cycles.
        mesh.GetManifoldSurfaceBoundaryVertexPairs(0).Length.Should().Be(sideSegments * 16);
        mesh.GetManifoldSurfaceBoundaryVertexPairs(0).ToArray().Should()
            .Equal(mesh.GetCoplanarPatchBoundaryVertexPairs(0).ToArray());
    }

    [Fact]
    public void WarmedNonconvexScalePreparation_ShouldReuseBoundaryCertificateStorage()
    {
        PhysicsMesh mesh = Annulus(8, false);
        var expanded = new Vector3d(2, 3, 4);
        for (int i = 0; i < 8; i++)
        {
            mesh.UpdateTransform(Vector3d.Zero, FixedQuaternion.Identity, expanded);
            mesh.UpdateTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        }
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 4; i++)
        {
            mesh.UpdateTransform(Vector3d.Zero, FixedQuaternion.Identity, expanded);
            mesh.UpdateTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        }
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        allocated.Should().Be(0);
        mesh.GetManifoldSurfaceOwner(mesh.TriangleCount - 1).Should().Be(0);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void BoundaryVertexOnUnsharedEdge_ShouldDeclineManifoldOwner(bool reverse)
    {
        // The upper vertex lies strictly inside the nonincident horizontal
        // rim edge (-3,1)--(3,1). Welding does not turn a partial edge into a seam.
        Vector3d[] vertices = { new(0,0,1), new(-3,0,1), new(-2,0,-3),
            new(2,0,-3), new(3,0,1), Vector3d.Zero };
        int[] faces = { 5,0,2, 5,2,4, 5,4,1, 5,1,3, 5,3,0 };
        if (reverse)
            for (int i = 0; i < faces.Length; i += 3) (faces[i + 1], faces[i + 2]) = (faces[i + 2], faces[i + 1]);
        PhysicsMesh mesh = Create(vertices, faces);
        mesh.GetCoplanarPatchId(0).Should().Be(0);
        for (int i = 0; i < mesh.TriangleCount; i++)
        {
            mesh.GetManifoldSurfaceOwner(i).Should().Be(i);
            mesh.GetCoplanarTriangleNeighbors(i).IsEmpty.Should().BeTrue();
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void BoundaryNesting_ShouldRequireUnitWindingInEveryCell(bool reverse, bool sameWinding)
    {
        Vector3d[] vertices = { new(-4,0,-4), new(4,0,-4), new(4,0,4), new(-4,0,4),
            new(-2,0,-2), new(2,0,-2), new(2,0,2), new(-2,0,2),
            new(-1,0,-1), new(1,0,-1), new(1,0,1), new(-1,0,1) };
        int[] corners = { 0,1,2,3, 4,7,6,5, 8,9,10,11 };
        if (sameWinding) Array.Reverse(corners, 4, 4);
        if (reverse)
            for (int i = 0; i < corners.Length; i += 4) Array.Reverse(corners, i, 4);
        // This production predicate receives directed reduced cycles. A hole
        // and its island are valid; an equally wound inner cover reaches two.
        PhysicsMesh.IsEmbeddedSurfaceBoundary(vertices, corners, new[] { 0,4,8,12 }, 1, reverse ? -1 : 1)
            .Should().Be(!sameWinding);
    }

    [Fact]
    public void BoundaryNesting_ShouldRejectNegativeWindingAndAllowSeparateDomains()
    {
        Vector3d[] vertices = { new(-4,0,-1), new(-2,0,-1), new(-2,0,1), new(-4,0,1),
            new(2,0,-1), new(4,0,-1), new(4,0,1), new(2,0,1) };
        int[] corners = { 0,1,2,3, 4,5,6,7 };
        PhysicsMesh.IsEmbeddedSurfaceBoundary(vertices, corners, new[] { 0,4,8 }, 1, 1).Should().BeTrue();
        Array.Reverse(corners, 4, 4);
        PhysicsMesh.IsEmbeddedSurfaceBoundary(vertices, corners, new[] { 0,4,8 }, 1, 1).Should().BeFalse();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void BoundaryCycles_ShouldRejectTouchAndPartialCollinearOverlap(bool overlap)
    {
        Vector3d[] vertices = { new(-4,0,-4), new(4,0,-4), new(4,0,4), new(-4,0,4),
            new(0,0,-4), new(-1,0,-2), new(1,0,-2) };
        if (overlap)
        {
            vertices[4] = new Vector3d(-1,0,-4);
            vertices[5] = new Vector3d(0,0,-2);
            vertices[6] = new Vector3d(1,0,-4);
        }
        PhysicsMesh.IsEmbeddedSurfaceBoundary(vertices, new[] { 0,1,2,3, 4,5,6 }, new[] { 0,4,7 }, 1, 1)
            .Should().BeFalse();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void BoundaryCycles_WithOverlappingBoundsButSeparatedEdges_ShouldRemainDisjoint(bool reverseCycles)
    {
        // The first triangle has X+Z <= 4; the second has X+Z >= 5.
        // Their boxes overlap. The lines (0,0)--(2,2) and (1,4)--(4,1)
        // cross only beyond the first segment, so both orientation products
        // are needed. Swapping cycles reverses which product rejects the pair.
        Vector3d[] vertices = { new(0,0,0), new(2,0,2), new(0,0,2),
            new(1,0,4), new(4,0,1), new(4,0,4) };
        int[] corners = reverseCycles ? new[] { 3,4,5, 0,1,2 } : new[] { 0,1,2, 3,4,5 };
        PhysicsMesh.IsEmbeddedSurfaceBoundary(vertices, corners, new[] { 0,3,6 }, 1, 1).Should().BeTrue();
    }

    [Theory]
    [InlineData(0)] // Empty boundary.
    [InlineData(1)] // Two-corner retrace.
    [InlineData(2)] // A reversal retained by monotone collinear reduction.
    [InlineData(3)] // Nonadjacent proper crossing.
    public void BoundaryCycles_ShouldRejectDegenerateAndSelfCrossingWalks(int example)
    {
        Vector3d[] vertices = { new(0,0,0), new(2,0,0), new(1,0,0), new(1,0,2), new(0,0,2) };
        int[] corners = example == 0 ? Array.Empty<int>() : example == 1 ? new[] { 0,1 }
            : example == 2 ? new[] { 1,2,3,4,0 } : new[] { 0,3,1,4 };
        int[] offsets = example == 0 ? new[] { 0 } : new[] { 0, corners.Length };
        PhysicsMesh.IsEmbeddedSurfaceBoundary(vertices, corners, offsets, 1, 1).Should().BeFalse();
    }

    private static PhysicsMesh Annulus(int sideSegments, bool reverse)
    {
        int count = sideSegments * 4;
        var vertices = new Vector3d[count * 2];
        var triangles = new int[count * 6];
        for (int ring = 0; ring < 2; ring++)
        {
            Fixed64 radius = (Fixed64)(ring == 0 ? 4 : 2);
            for (int side = 0; side < 4; side++)
            for (int i = 0; i < sideSegments; i++)
            {
                Fixed64 along = -radius + radius * (Fixed64)(2 * i) / (Fixed64)sideSegments;
                vertices[ring * count + side * sideSegments + i] = side switch
                {
                    0 => new Vector3d(along, Fixed64.Zero, -radius),
                    1 => new Vector3d(radius, Fixed64.Zero, along),
                    2 => new Vector3d(-along, Fixed64.Zero, radius),
                    _ => new Vector3d(-radius, Fixed64.Zero, -along)
                };
            }
        }
        for (int i = 0; i < count; i++)
        {
            int next = (i + 1) % count;
            triangles[6 * i] = i; triangles[6 * i + 1] = next; triangles[6 * i + 2] = count + next;
            triangles[6 * i + 3] = i; triangles[6 * i + 4] = count + next; triangles[6 * i + 5] = count + i;
        }
        if (reverse)
            for (int i = 0; i < triangles.Length; i += 3) (triangles[i + 1], triangles[i + 2]) = (triangles[i + 2], triangles[i + 1]);
        return Create(vertices, triangles);
    }

    private static PhysicsMesh TwoDomains(Vector3d[] shape, int[] triangles, bool rightFirst)
    {
        Vector3d[] vertices = new Vector3d[shape.Length * 2];
        int[] faces = new int[triangles.Length * 2];
        for (int side = 0; side < 2; side++)
        {
            Vector3d offset = Vector3d.Right * (Fixed64)((side == 0) == rightFirst ? 4 : 0);
            for (int i = 0; i < shape.Length; i++)
                vertices[side * shape.Length + i] = shape[i] + offset;
            for (int i = 0; i < triangles.Length; i++)
                faces[side * triangles.Length + i] = triangles[i] + side * shape.Length;
        }
        return Create(vertices, faces);
    }

    private static PhysicsMesh Create(Vector3d[] vertices, int[] triangles) =>
        new(vertices, triangles, Vector3d.Zero, FixedQuaternion.Identity, MeshColliderMode.Concave);
}
