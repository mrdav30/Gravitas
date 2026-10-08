using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using System;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class PhysicsMeshCoplanarPatchTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Quad_ShouldRetainOnlyOrientedExposedEdges(bool reversed)
    {
        int[] triangles = reversed ? new[] { 2, 1, 0, 3, 2, 0 } : new[] { 0, 1, 2, 0, 2, 3 };
        PhysicsMesh mesh = Create(Quad(), triangles);

        mesh.GetCoplanarPatchId(0).Should().Be(0);
        mesh.GetCoplanarPatchId(1).Should().Be(0);
        mesh.GetCoplanarPatchBoundaryVertexPairs(0).ToArray().Should().Equal(
            reversed ? new[] { 1, 0, 0, 3, 2, 1, 3, 2 } : new[] { 0, 1, 3, 0, 1, 2, 2, 3 });
        mesh.GetCoplanarPatchBoundaryVertexPairs(1).ToArray().Should()
            .Equal(mesh.GetCoplanarPatchBoundaryVertexPairs(0).ToArray());
        mesh.GetConvexCoplanarPatchCornerVertexIndices(0).ToArray().Should().Equal(
            reversed ? new[] { 0, 3, 2, 1 } : new[] { 0, 1, 2, 3 });
        mesh.GetConvexCoplanarPatchCornerVertexIndices(1).ToArray().Should()
            .Equal(mesh.GetConvexCoplanarPatchCornerVertexIndices(0).ToArray());
    }

    [Fact]
    public void ExactPositionDuplicates_ShouldWeldTheSeamWithoutChangingAuthoredTriangles()
    {
        Vector3d[] quad = Quad();
        PhysicsMesh mesh = Create(new[] { quad[0], quad[1], quad[2], quad[0], quad[2], quad[3] },
            new[] { 0, 1, 2, 3, 4, 5 });

        mesh.GetCoplanarPatchId(1).Should().Be(0);
        mesh.GetCoplanarPatchBoundaryVertexPairs(0).ToArray().Should().Equal(0, 1, 5, 0, 1, 2, 2, 5);
        mesh.Triangles.ToArray().Should().Equal(0, 1, 2, 3, 4, 5);
        mesh.GetConvexCoplanarPatchCornerVertexIndices(0).ToArray().Should().Equal(0, 1, 2, 5);
    }

    [Fact]
    public void LShape_ShouldPreserveTheNotchBoundary()
    {
        Vector3d[] vertices = { new(0, 0, 0), new(2, 0, 0), new(2, 0, 1),
            new(1, 0, 1), new(1, 0, 2), new(0, 0, 2) };
        PhysicsMesh mesh = Create(vertices, new[] { 0, 1, 3, 1, 2, 3, 0, 3, 5, 3, 4, 5 });

        for (int i = 0; i < 4; i++)
            mesh.GetCoplanarPatchId(i).Should().Be(0);
        mesh.GetCoplanarPatchBoundaryVertexPairs(0).ToArray().Should().Equal(
            0, 1, 5, 0, 1, 2, 2, 3, 3, 4, 4, 5);
        mesh.GetConvexCoplanarPatchCornerVertexIndices(0).IsEmpty.Should().BeTrue();
    }

    [Fact]
    public void Ring_ShouldRetainBothOuterAndHoleBoundaries()
    {
        Vector3d[] vertices = { new(-2, 0, -2), new(2, 0, -2), new(2, 0, 2), new(-2, 0, 2),
            new(-1, 0, -1), new(1, 0, -1), new(1, 0, 1), new(-1, 0, 1) };
        PhysicsMesh mesh = Create(vertices, new[] { 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5,
            2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7 });

        for (int i = 0; i < 8; i++)
            mesh.GetCoplanarPatchId(i).Should().Be(0);
        mesh.GetCoplanarPatchBoundaryVertexPairs(0).ToArray().Should().Equal(
            0, 1, 3, 0, 1, 2, 2, 3, 5, 4, 4, 7, 6, 5, 7, 6);
        mesh.GetConvexCoplanarPatchCornerVertexIndices(0).IsEmpty.Should().BeTrue();
    }

    [Fact]
    public void HoleTouchingOuterBoundaryAtOneWeldedVertex_ShouldDeclinePinchedPatch()
    {
        Vector3d[] vertices = { new(-2, 0, -2), new(2, 0, -2), new(2, 0, 2), new(-2, 0, 2),
            new(-2, 0, -2), new(1, 0, -1), new(1, 0, 1), new(-1, 0, 1) };
        PhysicsMesh mesh = Create(vertices, new[] { 0, 1, 5, 1, 2, 6, 1, 6, 5,
            2, 3, 7, 2, 7, 6, 3, 4, 7 });

        for (int i = 0; i < mesh.TriangleCount; i++)
        {
            mesh.GetCoplanarPatchId(i).Should().Be(-1);
            mesh.GetCoplanarPatchBoundaryVertexPairs(i).IsEmpty.Should().BeTrue();
            mesh.GetConvexCoplanarPatchCornerVertexIndices(i).IsEmpty.Should().BeTrue();
        }
    }

    [Fact]
    public void DisconnectedQuads_ShouldHaveSeparateMinimumAuthoredTriangleIds()
    {
        Vector3d[] vertices = new Vector3d[8];
        Quad().CopyTo(vertices, 0);
        Quad().CopyTo(vertices, 4);
        for (int i = 4; i < vertices.Length; i++)
            vertices[i] += Vector3d.Right * (Fixed64)4;
        PhysicsMesh mesh = Create(vertices, new[] { 0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7 });

        mesh.GetCoplanarPatchId(0).Should().Be(0);
        mesh.GetCoplanarPatchId(1).Should().Be(0);
        mesh.GetCoplanarPatchId(2).Should().Be(2);
        mesh.GetCoplanarPatchId(3).Should().Be(2);
        mesh.GetCoplanarPatchBoundaryVertexPairs(2).ToArray().Should().Equal(4, 5, 7, 4, 5, 6, 6, 7);
        mesh.GetConvexCoplanarPatchCornerVertexIndices(0).ToArray().Should().Equal(0, 1, 2, 3);
        mesh.GetConvexCoplanarPatchCornerVertexIndices(3).ToArray().Should().Equal(4, 5, 6, 7);
    }

    [Fact]
    public void CreaseBesideValidPatch_ShouldRemainAnExposedPatchEdge()
    {
        Vector3d[] vertices = new Vector3d[5];
        Quad().CopyTo(vertices, 0);
        vertices[4] = new Vector3d(-1, 1, -1);
        PhysicsMesh mesh = Create(vertices, new[] { 0, 1, 2, 0, 2, 3, 1, 0, 4 });

        mesh.GetCoplanarPatchId(1).Should().Be(0);
        mesh.GetCoplanarPatchId(2).Should().Be(-1);
        mesh.GetCoplanarPatchBoundaryVertexPairs(0).ToArray().Should().Equal(0, 1, 3, 0, 1, 2, 2, 3);
        mesh.GetConvexCoplanarPatchCornerVertexIndices(0).ToArray().Should().Equal(0, 1, 2, 3);
        mesh.GetConvexCoplanarPatchCornerVertexIndices(2).IsEmpty.Should().BeTrue();
    }

    [Fact]
    public void NonmanifoldEdgeBesideAValidSeam_ShouldRejectItsEntireConnectedPatch()
    {
        Vector3d[] vertices = new Vector3d[6];
        Quad().CopyTo(vertices, 0);
        vertices[4] = new Vector3d(-1, 0, -2);
        vertices[5] = new Vector3d(1, 0, -3);
        PhysicsMesh mesh = Create(vertices, new[] { 0, 1, 2, 0, 2, 3, 1, 0, 4, 1, 0, 5 });

        for (int i = 0; i < mesh.TriangleCount; i++)
            mesh.GetCoplanarPatchId(i).Should().Be(-1);
    }

    [Theory]
    [InlineData(0)] // A single face has no seam to suppress.
    [InlineData(1)] // The two faces meet at a crease.
    [InlineData(2)] // Opposite edge winding but both interiors occupy the same side.
    [InlineData(3)] // Geometrically opposite sides with inconsistent winding.
    [InlineData(4)] // Three incident faces make the edge nonmanifold.
    public void UnsupportedOwnership_ShouldDeclineThePatch(int kind)
    {
        Vector3d[] vertices;
        int[] triangles;
        if (kind == 0)
        {
            vertices = new[] { Vector3d.Zero, Vector3d.Right, Vector3d.Forward };
            triangles = new[] { 0, 1, 2 };
        }
        else
        {
            vertices = new[] { Vector3d.Zero, Vector3d.Right, Vector3d.Forward,
                kind == 1 ? Vector3d.Up : kind == 2 ? new Vector3d(1, 0, 1) : -Vector3d.Forward,
                new Vector3d(0, 0, -2) };
            if (kind == 4)
                triangles = new[] { 0, 1, 2, 1, 0, 3, 1, 0, 4 };
            else
            {
                Array.Resize(ref vertices, 4);
                triangles = kind == 3 ? new[] { 0, 1, 2, 0, 1, 3 } : new[] { 0, 1, 2, 1, 0, 3 };
            }
        }
        PhysicsMesh mesh = Create(vertices, triangles);

        for (int i = 0; i < mesh.TriangleCount; i++)
        {
            mesh.GetCoplanarPatchId(i).Should().Be(-1);
            mesh.GetCoplanarPatchBoundaryVertexPairs(i).IsEmpty.Should().BeTrue();
            mesh.GetConvexCoplanarPatchCornerVertexIndices(i).IsEmpty.Should().BeTrue();
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void UnrepresentableSeamDifferences_ShouldDeclineWithoutNarrowingTheMesh(int wideDifference)
    {
        Fixed64 left = Fixed64.MinValue, right = Fixed64.MaxValue;
        Vector3d[] vertices = { new(left, Fixed64.Zero, Fixed64.Zero),
            new(wideDifference == 0 ? right : left + Fixed64.One, Fixed64.Zero, Fixed64.Zero),
            new(wideDifference == 1 ? right : left, Fixed64.Zero, Fixed64.One),
            new(wideDifference == 2 ? right : left, Fixed64.Zero, -Fixed64.One) };
        PhysicsMesh mesh = Create(vertices, new[] { 0, 1, 2, 1, 0, 3 });

        // These valid faces need a wider difference carrier than this optional
        // topology certificate. Declining must preserve their authored geometry.
        mesh.GetCoplanarPatchId(0).Should().Be(-1);
        mesh.GetCoplanarPatchId(1).Should().Be(-1);
        mesh.VertexCount.Should().Be(4);
        mesh.TriangleCount.Should().Be(2);
    }

    [Fact]
    public void FailedScale_ShouldKeepCommittedPatchAndSubsequentPoseUpdate()
    {
        PhysicsMesh mesh = Create(Quad(), new[] { 0, 1, 2, 0, 2, 3 });
        int[] boundary = mesh.GetCoplanarPatchBoundaryVertexPairs(0).ToArray();
        int[] corners = mesh.GetConvexCoplanarPatchCornerVertexIndices(0).ToArray();
        Vector3d[] vertices = mesh.ScaledLocalVertices.ToArray();
        Action update = () => mesh.UpdateTransform(Vector3d.One, FixedQuaternion.Identity,
            new Vector3d(Fixed64.MinIncrement, Fixed64.One, Fixed64.MinIncrement));

        update.Should().Throw<ArgumentException>();
        mesh.GetCoplanarPatchId(1).Should().Be(0);
        mesh.GetCoplanarPatchBoundaryVertexPairs(0).ToArray().Should().Equal(boundary);
        mesh.GetConvexCoplanarPatchCornerVertexIndices(0).ToArray().Should().Equal(corners);
        mesh.ScaledLocalVertices.ToArray().Should().Equal(vertices);
        mesh.UpdatePosition(Vector3d.One, FixedQuaternion.Identity);
        mesh.GetCoplanarPatchBoundaryVertexPairs(1).ToArray().Should().Equal(boundary);
        mesh.GetConvexCoplanarPatchCornerVertexIndices(1).ToArray().Should().Equal(corners);
        mesh.ScaledLocalVertices.ToArray().Should().Equal(vertices);
    }

    [Fact]
    public void PreparedScale_ShouldPublishPatchOnlyWithItsMatchingGeometry()
    {
        // y=x/4 is exactly planar before scaling. Centered X/3 has half-raw
        // products with different tie parity, breaking their common slope.
        Vector3d[] vertices = { new(0, 0, 0), new(Fixed64.One, Fixed64.Quarter, Fixed64.Zero),
            new((Fixed64)3, Fixed64.FromFraction(3, 4), Fixed64.One), new(0, 0, 1) };
        PhysicsMesh mesh = Create(vertices, new[] { 0, 1, 2, 0, 2, 3 });
        mesh.GetCoplanarPatchId(0).Should().Be(0);
        mesh.GetConvexCoplanarPatchCornerVertexIndices(0).ToArray().Should().Equal(0, 1, 2, 3);
        mesh.PrepareTransformation(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(Fixed64.FromFraction(1, 3), Fixed64.One, Fixed64.One), Vector3d.One, null);
        mesh.GetCoplanarPatchId(0).Should().Be(0);
        mesh.GetConvexCoplanarPatchCornerVertexIndices(0).ToArray().Should().Equal(0, 1, 2, 3);
        mesh.PublishPreparedTransformation();
        mesh.GetCoplanarPatchId(0).Should().Be(-1);
        mesh.GetCoplanarPatchId(1).Should().Be(-1);
        mesh.GetCoplanarPatchBoundaryVertexPairs(0).IsEmpty.Should().BeTrue();
        mesh.GetConvexCoplanarPatchCornerVertexIndices(0).IsEmpty.Should().BeTrue();
        mesh.UpdateTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        mesh.GetCoplanarPatchId(0).Should().Be(0);
        mesh.GetConvexCoplanarPatchCornerVertexIndices(0).ToArray().Should().Equal(0, 1, 2, 3);
    }

    [Fact]
    public void RejectedPreparedMassProperties_ShouldNotPublishNewPatchOwnership()
    {
        Vector3d[] vertices = { new(0, 0, 0), new(Fixed64.One, Fixed64.Quarter, Fixed64.Zero),
            new((Fixed64)3, Fixed64.FromFraction(3, 4), Fixed64.One), new(0, 0, 1) };
        PhysicsMesh mesh = Create(vertices, new[] { 0, 1, 2, 0, 2, 3 });
        int[] boundary = mesh.GetCoplanarPatchBoundaryVertexPairs(0).ToArray();
        int[] corners = mesh.GetConvexCoplanarPatchCornerVertexIndices(0).ToArray();
        Action prepare = () => mesh.PrepareTransformation(Vector3d.One, FixedQuaternion.Identity,
            new Vector3d(Fixed64.FromFraction(1, 3), Fixed64.One, (Fixed64)1000000),
            Vector3d.One, MeshInertiaPolicy.SurfaceApproximation);

        // The geometry candidate loses coplanarity, then its million-unit
        // thin-shell moments fail the existing representability contract.
        prepare.Should().Throw<ArgumentException>().WithMessage("*surface mass properties*");
        mesh.GetCoplanarPatchId(0).Should().Be(0);
        mesh.GetCoplanarPatchBoundaryVertexPairs(0).ToArray().Should().Equal(boundary);
        mesh.GetConvexCoplanarPatchCornerVertexIndices(0).ToArray().Should().Equal(corners);
        mesh.OwnerScale.Should().Be(Vector3d.One);
        mesh.UpdatePosition(Vector3d.One, FixedQuaternion.Identity);
        mesh.GetCoplanarPatchId(1).Should().Be(0);
        mesh.GetConvexCoplanarPatchCornerVertexIndices(1).ToArray().Should().Equal(corners);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void NonuniformExactScaleAndPose_ShouldPreservePatchAcrossCoordinatePlanes(int plane)
    {
        Vector3d[] vertices = Quad();
        for (int i = 0; i < vertices.Length; i++)
            vertices[i] = plane == 0 ? new Vector3d(Fixed64.Zero, vertices[i].X, vertices[i].Z)
                : plane == 1 ? vertices[i] : new Vector3d(vertices[i].X, vertices[i].Z, Fixed64.Zero);
        PhysicsMesh mesh = Create(vertices, new[] { 0, 1, 2, 0, 2, 3 });
        int[] boundary = mesh.GetCoplanarPatchBoundaryVertexPairs(0).ToArray();
        mesh.UpdateTransform(Vector3d.One, FixedQuaternion.Identity, new Vector3d(2, 3, 4));
        mesh.GetCoplanarPatchId(1).Should().Be(0);
        mesh.GetCoplanarPatchBoundaryVertexPairs(0).ToArray().Should().Equal(boundary);
        mesh.GetConvexCoplanarPatchCornerVertexIndices(0).ToArray().Should().Equal(0, 1, 2, 3);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConvexBoundarySubdivisions_ShouldRetainOnlyStrictOrientedCorners(bool reversed)
    {
        // Vertex zero is inside a straight boundary edge. In either winding
        // the successor walk reaches a larger strict corner before vertex one.
        // Removing zero must rotate the ring to the later minimum true corner.
        Vector3d[] vertices = { new(0, 0, -1), new(-1, 0, 1), new(1, 0, 1),
            new(1, 0, -1), new(-1, 0, -1), Vector3d.Zero };
        int[] triangles = { 5, 0, 3, 5, 3, 2, 5, 2, 1, 5, 1, 4, 5, 4, 0 };
        if (reversed)
            for (int i = 0; i < triangles.Length; i += 3)
                (triangles[i + 1], triangles[i + 2]) = (triangles[i + 2], triangles[i + 1]);
        PhysicsMesh mesh = Create(vertices, triangles);

        mesh.GetCoplanarPatchBoundaryVertexPairs(0).Length.Should().Be(10);
        mesh.GetConvexCoplanarPatchCornerVertexIndices(0).ToArray().Should().Equal(
            reversed ? new[] { 1, 2, 3, 4 } : new[] { 1, 4, 3, 2 });
    }

    [Fact]
    public void SameTurnStarBoundary_ShouldDeclineConvexTrustWithoutLosingPatchEdges()
    {
        // Each origin-centered triangle has positive projected orientation,
        // and every shared radial seam has opposite sides. The boundary is
        // nevertheless a twice-wound star, not its convex polygon's fill.
        Vector3d[] vertices = { new(0, 0, 3), new(-3, 0, 1), new(-2, 0, -3),
            new(2, 0, -3), new(3, 0, 1), Vector3d.Zero };
        PhysicsMesh mesh = Create(vertices, new[] { 5, 0, 2, 5, 2, 4, 5, 4, 1, 5, 1, 3, 5, 3, 0 });

        mesh.GetCoplanarPatchId(0).Should().Be(0);
        mesh.GetCoplanarPatchBoundaryVertexPairs(0).Length.Should().Be(10);
        mesh.GetConvexCoplanarPatchCornerVertexIndices(0).IsEmpty.Should().BeTrue();
    }

    [Fact]
    public void WarmedScaleQuantization_ShouldReuseBuffersWithoutExposingPreviousCornerTails()
    {
        // At unit scale, vertex zero protrudes by one raw unit and is a true
        // corner. Halving Z rounds it onto the straight edge at a nearest-even
        // midpoint, reducing the ring from five corners to four.
        Vector3d[] vertices = { new(Fixed64.Zero, Fixed64.Zero, -Fixed64.One - Fixed64.MinIncrement),
            new(-1, 0, 1), new(1, 0, 1), new(1, 0, -1), new(-1, 0, -1), Vector3d.Zero };
        PhysicsMesh mesh = Create(vertices, new[] { 5, 0, 3, 5, 3, 2, 5, 2, 1, 5, 1, 4, 5, 4, 0 });
        Vector3d halfScale = new(Fixed64.One, Fixed64.One, Fixed64.Half);
        mesh.GetConvexCoplanarPatchCornerVertexIndices(0).ToArray().Should().Equal(0, 3, 2, 1, 4);
        mesh.UpdateTransform(Vector3d.Zero, FixedQuaternion.Identity, halfScale);
        mesh.GetConvexCoplanarPatchCornerVertexIndices(0).ToArray().Should().Equal(1, 4, 3, 2);
        mesh.UpdateTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        mesh.GetConvexCoplanarPatchCornerVertexIndices(0).ToArray().Should().Equal(0, 3, 2, 1, 4);
        mesh.UpdateTransform(Vector3d.Zero, FixedQuaternion.Identity, halfScale);
        mesh.UpdateTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 4; i++)
        {
            mesh.UpdateTransform(Vector3d.Zero, FixedQuaternion.Identity, halfScale);
            if (mesh.GetConvexCoplanarPatchCornerVertexIndices(0).Length != 4)
                throw new InvalidOperationException("Quantized ring retained a stale corner tail.");
            mesh.UpdateTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
            if (mesh.GetConvexCoplanarPatchCornerVertexIndices(0).Length != 5)
                throw new InvalidOperationException("Restored ring lost its protruding corner.");
        }
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        allocated.Should().Be(0);
        mesh.GetCoplanarPatchBoundaryVertexPairs(0).Length.Should().Be(10);
    }

    private static Vector3d[] Quad() => new[]
        { new Vector3d(-1, 0, -1), new Vector3d(1, 0, -1), new Vector3d(1, 0, 1), new Vector3d(-1, 0, 1) };

    private static PhysicsMesh Create(Vector3d[] vertices, int[] triangles) =>
        new(vertices, triangles, Vector3d.Zero, FixedQuaternion.Identity, MeshColliderMode.Concave);
}
