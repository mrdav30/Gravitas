using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests;

public sealed class MeshSurfaceFamilyBoundaryProofTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Cone_PartialJoin_ShouldProduceItsNewCutAndKeepTheNativeSupplier(bool independentNeighbor)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        LSMeshCollider source = Triangle(scene.Context, new[] { new Vector3d(-2, 0, 0), new(2, 0, 0), new(-2, 2, 0) });
        LSMeshCollider neighbor = Triangle(scene.Context, new[] { new Vector3d(Fixed64.Quarter, Fixed64.Zero, Fixed64.Zero),
            new(2, 0, 0), new(2, -2, 0) }, independentNeighbor);
        var cone = new LSConeCollider { Radius = Fixed64.One, Size = new(2, 2, 2) };
        cone.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(Vector3d.Zero,
            new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized, Vector3d.One)));
        var spans = Sources(new[] { source, neighbor }, sourceEdgeOnly: true);
        Assert.Contains(spans, item => item.Span.Kind == MeshSurfaceBoundaryDomain.SpanKind.Joined);

        Assert.True(MeshConeFaceRegions.TryGetBoundaryWitness(spans, cone, Orientation(spans[0].Collider, Vector3d.Forward),
            out FixedContactAnchors witness, out int supplier));
        Assert.Same(source, spans[supplier].Collider);
        Assert.Equal(source.Mesh.Origin, witness.FirstAnchor.Origin);
        Assert.Equal(source.Mesh.Rotation, witness.FirstAnchor.Rotation);
        Assert.Equal(cone.Center, witness.SecondAnchor.Origin);
        Assert.Equal(cone.Rotation, witness.SecondAnchor.Rotation);
        Assert.True(witness.FirstAnchor.TryGetPoint(out Vector3d point));
        Assert.Equal(new Vector3d(Fixed64.Quarter, Fixed64.Zero, Fixed64.Zero), point);
        Assert.Equal(Fixed64.FromFraction(5, 8), witness.Depth);
        Assert.Equal(-Vector3d.Forward, witness.Normal);
        Assert.True(witness.SecondAnchor.TryGetPoint(out Vector3d exit));
        Assert.Equal(point - witness.Normal * witness.Depth, exit);
        bool valid = true;
        long allocated = AllocationTestHelper.MeasureSteadyState(() => valid &= MeshConeFaceRegions.TryGetBoundaryWitness(
            spans, cone, Orientation(spans[0].Collider, Vector3d.Forward), out _, out _),
            warmupIterations: 2, stabilizationIterations: 2, measurementIterations: 4);
        Assert.True(valid);
        Assert.Equal(0, allocated);
        // This is the maximum over retained boundary spans; the covered face
        // interior can be deeper, so this case makes no whole-face exit claim.
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Cone_BoundedCapExit_ShouldClearAllTrianglesAndKeepStableMaterialOwnership(bool duplicate)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        LSMeshCollider[] members = CapPatch(scene.Context, duplicate);
        members[0].Material = PhysicsMaterial.Frictionless;
        members[1].Material = PhysicsMaterial.Bouncy;
        var cone = new LSConeCollider { Radius = Fixed64.One, Size = new(2, 2, 2) };
        scene.InitializeStaticCollider(cone, new(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero));
        Vector3d? selectedPoint = null;
        foreach (bool reverse in new[] { false, true })
        {
            var spans = Sources(reverse ? new[] { members[1], members[0] } : members);
            Assert.True(MeshConeFaceRegions.TryGetBoundaryWitness(spans, cone, Orientation(spans[0].Collider, Vector3d.Down),
                out FixedContactAnchors witness, out int supplier));
            Assert.Same(members[0], spans[supplier].Collider);
            Assert.Equal(PhysicsMaterial.Frictionless, spans[supplier].Collider.Material);
            Assert.Equal(members[0].Mesh.Origin, witness.FirstAnchor.Origin);
            Assert.Equal(cone.Center, witness.SecondAnchor.Origin);
            Assert.Equal(cone.Rotation, witness.SecondAnchor.Rotation);
            Assert.True(witness.FirstAnchor.TryGetPoint(out Vector3d point));
            if (selectedPoint.HasValue) Assert.Equal(selectedPoint.Value, point);
            selectedPoint = point;
            Assert.Equal(Fixed64.One - Fixed64.Quarter, witness.Depth);
            Assert.Equal(Vector3d.Up, witness.Normal);
            AssertWholeExit(members, cone.Center, witness, (mesh, triangle, center) =>
                LocalTriangle(mesh, triangle).TryGetCenteredFiniteConeContact(mesh.Mesh.Origin, mesh.Mesh.Rotation,
                    center, cone.Rotation, cone.Height, cone.ScaledRadius, out _));
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void NonaxialEmbeddedSlabs_BoundaryControlledExit_ShouldClearEveryProvidedTriangle(int shape)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Vector3d[] first = { new((Fixed64)3, -Fixed64.Half, Fixed64.Zero), new((Fixed64)4, -Fixed64.Half, Fixed64.Zero),
            new((Fixed64)3, Fixed64.Half, Fixed64.Zero) };
        LSMeshCollider[] members = { Triangle(scene.Context, first), Triangle(scene.Context,
            new[] { first[1], new Vector3d((Fixed64)4, Fixed64.Quarter, Fixed64.Zero), first[2] }, independent: true, reversed: true) };
        LSCollider2D embedded = shape == 0 ? new LSCircleCollider2D((Fixed64)5)
            : shape == 1 ? new LSCapsuleCollider2D((Fixed64)5, (Fixed64)12)
            : new LSPolygonCollider2D(new(0, -5), new(5, 0), new(0, 5));
        embedded.MixedHalfThicknessOverride = Fixed64.One;
        embedded.InitializeWithNoBody(new TestMatterAgent(scene.Context));
        var spans = Sources(new[] { members[1], members[0] });
        Assert.True(SlabWitness(spans, embedded, Orientation(spans[0].Collider, Vector3d.Forward), out var witness, out int supplier));
        Assert.Same(members[0], spans[supplier].Collider);
        Assert.Equal(MixedEmbedded2DGeometry.GetCenter3D(embedded), witness.SecondAnchor.Origin);
        Assert.Equal((Fixed64)(shape == 0 ? 4 : shape == 1 ? 5 : 2), witness.Depth);
        Assert.Equal(-Vector3d.Forward, witness.Normal);
        AssertWholeExit(members, MixedEmbedded2DGeometry.GetCenter3D(embedded), witness,
            (mesh, triangle, center) => SlabIntersects(mesh, triangle, center, embedded));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void NonaxialEmbeddedSlabs_PartialJoin_ShouldRegenerateTheControllingBoundaryCut(int shape)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        LSMeshCollider source = Triangle(scene.Context, new[] { new Vector3d(-2, 0, 0), new(2, 0, 0), new(-2, 2, 0) });
        LSMeshCollider neighbor = Triangle(scene.Context, new[] { new Vector3d(-2, 0, 0), new(Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
            new(-2, -2, 0) }, independent: true);
        LSCollider2D embedded = Embedded(scene.Context, shape,
            shape == 2 ? Vector3d.Forward : Vector3d.Zero, Fixed64.Zero, Fixed64.Quarter, triangular: true);
        var spans = Sources(new[] { source, neighbor }, sourceEdgeOnly: true);
        Assert.True(SlabWitness(spans, embedded, Orientation(source, Vector3d.Forward), out var witness, out int supplier));
        Assert.Same(source, spans[supplier].Collider);
        Assert.Equal(source.Mesh.Origin, witness.FirstAnchor.Origin);
        Assert.Equal(MixedEmbedded2DGeometry.GetCenter3D(embedded), witness.SecondAnchor.Origin);
        Assert.True(witness.FirstAnchor.TryGetPoint(out Vector3d point));
        Assert.Equal(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero), point);
        Fixed64 expected = shape == 2 ? Fixed64.Half
            : FixedMath.Sqrt(Fixed64.One - Fixed64.Quarter) + (shape == 1 ? Fixed64.One : Fixed64.Zero);
        Assert.InRange(witness.Depth.m_rawValue, expected.m_rawValue - 1, expected.m_rawValue + 1);
        Assert.Equal(-Vector3d.Forward, witness.Normal);
        bool valid = true;
        int orientation = Orientation(source, Vector3d.Forward);
        long allocated = AllocationTestHelper.MeasureSteadyState(() => valid &= SlabWitness(spans, embedded, orientation, out _, out _),
            warmupIterations: 2, stabilizationIterations: 2, measurementIterations: 4);
        Assert.True(valid);
        Assert.Equal(0, allocated);
        // The complete nonaxial owner must still search its physical support;
        // an edge-only maximum is not a whole-shape separation certificate.
    }

    [Fact]
    public void SlabBoundaryMaximum_ShouldIgnoreOutsideSpansAndKeepTheDeeperMemberInEitherBlockOrder()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        LSMeshCollider deep = Triangle(scene.Context, new[] { new Vector3d((Fixed64)3, -Fixed64.Half, Fixed64.Zero),
            new((Fixed64)6, -Fixed64.Half, Fixed64.Zero), new((Fixed64)6, Fixed64.Half, Fixed64.Zero) });
        LSMeshCollider shallow = Triangle(scene.Context, new[] { new Vector3d((Fixed64)4, Fixed64.One - Fixed64.Quarter, Fixed64.Zero),
            new(Fixed64.FromFraction(9, 2), Fixed64.One - Fixed64.Quarter, Fixed64.Zero), new(4, 1, 0) }, independent: true);
        var embedded = new LSCircleCollider2D((Fixed64)5) { MixedHalfThicknessOverride = Fixed64.One };
        embedded.InitializeWithNoBody(new TestMatterAgent(scene.Context));
        Assert.True(SlabIntersects(deep, 0, Vector3d.Zero, embedded));
        Assert.True(SlabIntersects(shallow, 0, Vector3d.Zero, embedded));
        Vector3d? selectedPoint = null;
        foreach (bool reverse in new[] { false, true })
        {
            // Both members intersect the slab, but deep's x=6 boundary is
            // outside its footprint. Shallow's maximum is only 3, versus 4.
            var spans = Sources(reverse ? new[] { shallow, deep } : new[] { deep, shallow });
            Assert.True(SlabWitness(spans, embedded, Orientation(spans[0].Collider, Vector3d.Forward), out var witness, out int supplier));
            Assert.Same(deep, spans[supplier].Collider);
            Assert.Equal((Fixed64)4, witness.Depth);
            Assert.Equal(-Vector3d.Forward, witness.Normal);
            Assert.True(witness.FirstAnchor.TryGetPoint(out Vector3d point));
            Assert.Equal(new Vector3d((Fixed64)3, -Fixed64.Half, Fixed64.Zero), point);
            if (selectedPoint.HasValue) Assert.Equal(selectedPoint.Value, point);
            selectedPoint = point;
        }
    }

    [Fact]
    public void EmptyAndJoinedOnlySources_ShouldNotManufactureABoundaryWitness()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var cone = new LSConeCollider();
        scene.InitializeStaticCollider(cone, Vector3d.Zero);
        var empty = Array.Empty<(LSMeshCollider Collider, int SurfaceTriangle, MeshSurfaceBoundaryDomain.BoundarySpan Span)>();
        Assert.False(MeshConeFaceRegions.TryGetBoundaryWitness(empty, cone, 1, out _, out _));
        LSCollider2D embedded = Embedded(scene.Context, 0, Vector3d.Zero, Fixed64.Zero, Fixed64.Half);
        Assert.False(SlabWitness(empty, embedded, 1, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => MeshConeFaceRegions.TryGetBoundaryWitness(empty, cone, 0, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => SlabWitness(empty, embedded, 0, out _, out _));
        var spans = Sources(CapPatch(scene.Context, duplicate: false));
        var joined = Array.FindAll(spans, item => item.Span.Kind == MeshSurfaceBoundaryDomain.SpanKind.Joined);
        Assert.NotEmpty(joined);
        Assert.False(MeshConeFaceRegions.TryGetBoundaryWitness(joined, cone, 1, out _, out _));
        Assert.False(SlabWitness(joined, embedded, 1, out _, out _));
    }

    private static bool SlabWitness((LSMeshCollider Collider, int SurfaceTriangle, MeshSurfaceBoundaryDomain.BoundarySpan Span)[] spans,
        LSCollider2D embedded, int orientation, out FixedContactAnchors witness, out int supplier)
    {
        Span<Vector2d> box = stackalloc Vector2d[4];
        return MeshSlabFaceRegions.TryGetBoundaryWitness(spans, MixedEmbedded2DGeometry.GetCenter3D(embedded),
            FixedQuaternion.FromAxisAngle(Vector3d.Up, -embedded.Rotation),
            embedded is LSCapsuleCollider2D capsule ? capsule.AxisLength : Fixed64.Zero,
            embedded is LSCircleCollider2D circle ? circle.ScaledRadius : embedded is LSCapsuleCollider2D stadium ? stadium.ScaledRadius : Fixed64.Zero,
            embedded.MixedHalfThickness, CollisionDetectionMixed.GetConvexPrismOffsets(embedded, box), orientation, out witness, out supplier);
    }

    private static bool SlabIntersects(LSMeshCollider mesh, int triangle, Vector3d center, LSCollider2D embedded)
    {
        FixedTriangle face = LocalTriangle(mesh, triangle);
        if (embedded is LSCircleCollider2D circle)
            return face.TryGetCircleSlabContact(mesh.Mesh.Origin, mesh.Mesh.Rotation, center,
                embedded.Rotation, embedded.MixedHalfThickness, circle.ScaledRadius, out _);
        if (embedded is LSCapsuleCollider2D capsule)
            return face.TryGetCenteredCapsuleSlabContact(mesh.Mesh.Origin, mesh.Mesh.Rotation, center, embedded.Rotation,
                Vector2d.Forward, capsule.AxisLength, capsule.ScaledRadius, embedded.MixedHalfThickness, out _);
        Span<Vector2d> box = stackalloc Vector2d[4];
        return FixedConvexPrismRelations.TryGetTriangleContact(mesh.Mesh.Origin, mesh.Mesh.Rotation, face, center,
            embedded.ConvexRotation, CollisionDetectionMixed.GetConvexPrismOffsets(embedded, box), embedded.MixedHalfThickness, out _);
    }

    private static void AssertWholeExit(LSMeshCollider[] members, Vector3d center, FixedContactAnchors witness,
        Func<LSMeshCollider, int, Vector3d, bool> intersects)
    {
        Vector3d touching = center + witness.Normal * witness.Depth;
        Vector3d separated = touching + witness.Normal * Fixed64.FromRaw(1);
        foreach (LSMeshCollider member in members)
            for (int triangle = 0; triangle < member.Mesh.TriangleCount; triangle++)
            {
                Assert.True(intersects(member, triangle, touching));
                Assert.False(intersects(member, triangle, separated));
            }
    }

    private static FixedTriangle LocalTriangle(LSMeshCollider mesh, int triangle)
    {
        mesh.Mesh.GetLocalTriangleVertices(triangle, out Vector3d a, out Vector3d b, out Vector3d c);
        return new FixedTriangle(a, b, c);
    }

    private static int Orientation(LSMeshCollider source, Vector3d ray)
    {
        Vector3d normal = LocalTriangle(source, 0).Normal;
        Fixed64 first = normal.X != Fixed64.Zero ? normal.X : normal.Y != Fixed64.Zero ? normal.Y : normal.Z;
        if (first < Fixed64.Zero) normal = -normal;
        return Vector3d.Dot(source.Mesh.Rotation * normal, ray) > Fixed64.Zero ? 1 : -1;
    }

    private static (LSMeshCollider Collider, int SurfaceTriangle, MeshSurfaceBoundaryDomain.BoundarySpan Span)[] Sources(
        LSMeshCollider[] members, bool sourceEdgeOnly = false)
    {
        var meshes = Array.ConvertAll(members, member => member.Mesh);
        var spans = new List<(LSMeshCollider, int, MeshSurfaceBoundaryDomain.BoundarySpan)>();
        for (int member = 0; member < (sourceEdgeOnly ? 1 : members.Length); member++)
        {
            var domain = new MeshSurfaceBoundaryDomain();
            domain.Build(meshes, member, 0);
            foreach (var span in domain.Spans)
                if (!sourceEdgeOnly || span.First == 0 && span.Second == 1) spans.Add((members[member], 0, span));
        }
        return spans.ToArray();
    }

    private static LSMeshCollider[] CapPatch(GravitasWorldContext context, bool duplicate)
    {
        Vector3d[] first = { new(-Fixed64.Half, Fixed64.Zero, -Fixed64.Half), new(Fixed64.Half, Fixed64.Zero, -Fixed64.Half),
            new(-Fixed64.Half, Fixed64.Zero, Fixed64.Half) };
        Vector3d[] second = duplicate ? first : new[] { first[1], new Vector3d(Fixed64.Quarter, Fixed64.Zero, Fixed64.Quarter), first[2] };
        return new[] { Triangle(context, first), Triangle(context, second, independent: true, reversed: true) };
    }

    private static LSMeshCollider Triangle(GravitasWorldContext context, Vector3d[] vertices, bool independent = false, bool reversed = false)
    {
        if (independent) vertices = Array.ConvertAll(vertices, v => new Vector3d(Fixed64.One - v.X, v.Y, Fixed64.One - v.Z));
        var collider = new LSMeshCollider(vertices, reversed ? new[] { 0, 2, 1 } : new[] { 0, 1, 2 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        collider.InitializeWithNoBody(new TestMatterAgent(context, new FixedTransform(independent ? new(1, 0, 1) : Vector3d.Zero,
            independent ? new(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero) : FixedQuaternion.Identity, Vector3d.One)));
        return collider;
    }

    private static LSCollider2D Embedded(GravitasWorldContext context, int shape, Vector3d position, Fixed64 yaw,
        Fixed64 halfThickness, bool triangular = false)
    {
        LSCollider2D collider = shape == 0 ? new LSCircleCollider2D(Fixed64.One)
            : shape == 1 ? new LSCapsuleCollider2D(Fixed64.One, (Fixed64)4)
            : triangular ? new LSPolygonCollider2D(new(-1, -1), new(1, -1), new(-1, 1))
            : new LSPolygonCollider2D(new(-1, -1), new(1, -1), new(1, 1), new(-1, 1));
        collider.MixedHalfThicknessOverride = halfThickness;
        collider.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(position, FixedQuaternion.FromAxisAngle(Vector3d.Up, -yaw), Vector3d.One)));
        return collider;
    }
}
