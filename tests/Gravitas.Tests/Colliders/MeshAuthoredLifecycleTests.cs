using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Serialization;
using Gravitas.Tests.Support;
using SwiftCollections;
using SwiftCollections.Query;
using System;
using System.Numerics;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class MeshAuthoredLifecycleTests
{
    [Fact]
    public void ZeroPlacementTransitions_ShouldRebuildOnlyTheChangedGeometryChart()
    {
        using var context = GravitasWorldContext.CreateOwned();
        Vector3d[] vertices = { new(-2, 0, -2), new(3, 0, -2), new(0, 0, 3) };
        var collider = new LSMeshCollider(vertices, new[] { 0, 2, 1 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        Vector3d scale = Vector3d.One * Fixed64.Half;
        var body = new SolidBody(new TestMatterAgent(context, new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, scale)), collider)
            { Mass = Fixed64.One };
        body.Initialize(Vector3d.One, FixedQuaternion.Identity);
        var rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)17, (Fixed64)23, (Fixed64)31);
        Vector3d reference = collider.Mesh.LocalBounds.Center;
        int builds = collider.Mesh.TriangleBvhBuildCount;
        foreach (Vector3d offset in new[] { Vector3d.Zero, reference, Vector3d.Zero })
        {
            collider.LocalOffset = offset; collider.Simulate();
            Assert.Equal(++builds, collider.Mesh.TriangleBvhBuildCount);
            for (int index = 0; index < vertices.Length; index++)
                Assert.Equal(Evaluate(vertices[index], reference, body.Position3d, body.Rotation, offset,
                    scale, body.Rotation, Vector3d.One), collider.Mesh.GetVertexWorld(index));
            body.SetRotation(body.Rotation == FixedQuaternion.Identity ? rotation : FixedQuaternion.Identity);
            collider.Simulate();
            Assert.Equal(builds, collider.Mesh.TriangleBvhBuildCount);
            for (int index = 0; index < vertices.Length; index++)
                Assert.Equal(Evaluate(vertices[index], reference, body.Position3d, body.Rotation, offset,
                    scale, body.Rotation, Vector3d.One), collider.Mesh.GetVertexWorld(index));
        }
    }

    [Fact]
    public void DetachedAffineMassProperties_ShouldRemainStableAcrossPolicyOrderAndPosePublication()
    {
        var definition = MeshTestFixtures.CreateConvexCubeDefinition();
        var vertices = new Vector3d[definition.MeshVertexCount]; var triangles = new int[definition.MeshTriangleIndexCount];
        for (int index = 0; index < vertices.Length; index++) vertices[index] = definition.GetMeshVertex(index);
        for (int index = 0; index < triangles.Length; index++) triangles[index] = definition.GetMeshTriangleIndex(index);
        var mesh = new PhysicsMesh(vertices, triangles, Vector3d.Zero, FixedQuaternion.Identity, MeshColliderMode.Convex);
        Vector3d scale = new(Fixed64.FromFraction(2, 3), Fixed64.FromFraction(3, 5), Fixed64.FromFraction(5, 7));
        mesh.UpdateTransform(Vector3d.Zero, FixedQuaternion.Identity, scale);
        var shell = mesh.CalculateInertiaTensor(Fixed64.One, MeshInertiaPolicy.SurfaceApproximation);
        Assert.True(mesh.TryGetClosedVolumeMassProperties(out var volume, out var validation));
        Assert.Equal(MeshVolumeValidationResult.Valid, validation);
        Assert.Equal(Vector3d.Zero, volume.CenterOfMass);
        BigInteger denominator = BigInteger.One << 64;
        BigInteger product = (BigInteger)scale.X.m_rawValue * scale.Y.m_rawValue * scale.Z.m_rawValue;
        BigInteger quotient = BigInteger.DivRem(product, denominator, out var remainder);
        if (remainder * 2 > denominator || remainder * 2 == denominator && !quotient.IsEven) quotient++;
        Assert.Equal((long)quotient, volume.Volume.m_rawValue);
        var solid = mesh.CalculateInertiaTensor(Fixed64.One, MeshInertiaPolicy.RequireClosedVolume);
        Assert.NotEqual(solid, shell);
        mesh.UpdatePosition(new Vector3d(3, 5, 7), FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)17, (Fixed64)23, (Fixed64)31));
        Assert.Equal(shell, mesh.CalculateInertiaTensor(Fixed64.One, MeshInertiaPolicy.SurfaceApproximation));
        Assert.Equal(solid, mesh.CalculateInertiaTensor(Fixed64.One, MeshInertiaPolicy.RequireClosedVolume));
    }

    [Fact]
    public void AffineSupportTree_ShouldMatchExactAuthoredProjectionAndStableTies()
    {
        const int count = 40;
        var vertices = new Vector3d[count + 1];
        var triangles = new int[count * 3];
        vertices[0] = new Vector3d(3, 2, 0);
        for (int index = 0; index < count; index++)
        {
            Fixed64 offset = Fixed64.FromFraction(index % 10, 5);
            vertices[index + 1] = vertices[0] + ((index / 10) switch
            {
                0 => new Vector3d(-Fixed64.One + offset, -Fixed64.One, Fixed64.Zero),
                1 => new Vector3d(Fixed64.One, -Fixed64.One + offset, Fixed64.Zero),
                2 => new Vector3d(Fixed64.One - offset, Fixed64.One, Fixed64.Zero),
                _ => new Vector3d(-Fixed64.One, Fixed64.One - offset, Fixed64.Zero)
            });
            triangles[index * 3] = 0; triangles[index * 3 + 1] = index + 1;
            triangles[index * 3 + 2] = (index + 1) % count + 1;
        }
        var mesh = new PhysicsMesh(vertices, triangles, Vector3d.Zero, FixedQuaternion.Identity, MeshColliderMode.Convex);
        Vector3d scale = new(Fixed64.FromFraction(2, 3), Fixed64.FromFraction(3, 5), Fixed64.FromFraction(5, 7));
        foreach (int degrees in new[] { 0, 37 })
        {
            var rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)degrees, (Fixed64)degrees, (Fixed64)degrees);
            mesh.UpdateTransform(new Vector3d(5, 7, 11), rotation, scale);
            BigInteger x = rotation.X.m_rawValue, y = rotation.Y.m_rawValue, z = rotation.Z.m_rawValue, w = rotation.W.m_rawValue;
            BigInteger[,] matrix = { { w*w+x*x-y*y-z*z, 2*(x*y-z*w), 2*(x*z+y*w) },
                { 2*(x*y+z*w), w*w-x*x+y*y-z*z, 2*(y*z-x*w) },
                { 2*(x*z-y*w), 2*(y*z+x*w), w*w-x*x-y*y+z*z } };
            foreach (var direction in new[] { Vector3d.Right, Vector3d.Left, Vector3d.Up, Vector3d.Down,
                Vector3d.Forward, -Vector3d.Forward, Vector3d.One, -Vector3d.One, new Vector3d(-2, 3, -5), Vector3d.Zero })
            {
                int best = 0;
                for (int index = 1; index < vertices.Length; index++)
                    if (Projection(index) > Projection(best)) best = index;
                Assert.Equal(mesh.GetVertexWorld(best), mesh.GetSupportVertexWorld(direction));
                BigInteger Projection(int index)
                {
                    if (direction == Vector3d.Zero) return vertices[index].X.m_rawValue;
                    BigInteger projection = 0;
                    for (int axis = 0; axis < 3; axis++)
                        for (int coordinate = 0; coordinate < 3; coordinate++)
                            projection += direction[axis].m_rawValue * matrix[axis, coordinate]
                                * vertices[index][coordinate].m_rawValue * scale[coordinate].m_rawValue;
                    return projection;
                }
            }
        }
    }

    [Fact]
    public void AffineNearestPoint_ShouldSearchTheAuthoredBvhBeyondItsFirstTriangle()
    {
        using var context = GravitasWorldContext.CreateOwned();
        var vertices = new Vector3d[8 * 8 * 4]; var triangles = new int[8 * 8 * 6];
        for (int row = 0; row < 8; row++)
            for (int column = 0; column < 8; column++)
            {
                int cell = row * 8 + column, vertex = cell * 4, triangle = cell * 6;
                vertices[vertex] = new Vector3d(column * 4, 0, row * 4);
                vertices[vertex + 1] = vertices[vertex] + Vector3d.Right * 2;
                vertices[vertex + 2] = vertices[vertex] + Vector3d.Forward * 2;
                vertices[vertex + 3] = vertices[vertex] + new Vector3d(2, 0, 2);
                int[] local = { 0, 2, 1, 1, 2, 3 };
                for (int index = 0; index < local.Length; index++) triangles[triangle + index] = vertex + local[index];
            }
        var mesh = new LSMeshCollider(vertices, triangles, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        Vector3d scale = new(Fixed64.FromFraction(2, 3), Fixed64.One, Fixed64.FromFraction(3, 5));
        mesh.InitializeWithNoBody(new TestMatterAgent(context, new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, scale)));
        foreach (int row in new[] { 0, 3, 7 })
            foreach (int column in new[] { 0, 3, 7 })
            {
                Vector3d query = new((Fixed64)(column * 4 + 1) * scale.X, Fixed64.Two, (Fixed64)(row * 4 + 1) * scale.Z);
                Assert.Equal(new Vector3d(query.X, Fixed64.Zero, query.Z), mesh.ClosestPointOnSurface(query));
            }
        Assert.Equal(Vector3d.Zero, mesh.ClosestPointOnSurface(new Vector3d(-1, 2, -1)));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AffineNearestPoint_ShouldSelectTheExactNearestFaceBeforeRounding(bool reverseTriangleOrder)
    {
        using var context = GravitasWorldContext.CreateOwned();
        Fixed64 fiveRaw = Fixed64.FromRaw(5);
        Fixed64 fourRaw = Fixed64.FromRaw(4);
        Vector3d[] vertices =
        {
            new(-(Fixed64)2, -Fixed64.One + fiveRaw, -(Fixed64)2),
            new((Fixed64)2, Fixed64.One + fiveRaw, -(Fixed64)2),
            new((Fixed64)0, fiveRaw, (Fixed64)2),
            new(-(Fixed64)2, fourRaw, -(Fixed64)2),
            new((Fixed64)2, fourRaw, -(Fixed64)2),
            new((Fixed64)0, fourRaw, (Fixed64)2)
        };
        int[] triangles = reverseTriangleOrder
            ? new[] { 3, 5, 4, 0, 2, 1 }
            : new[] { 0, 2, 1, 3, 5, 4 };
        var mesh = new LSMeshCollider(vertices, triangles, MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);
        Vector3d scale = Vector3d.One * Fixed64.FromFraction(1, 4);
        mesh.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, scale)));

        // The sloped face's exact closest point is (-1/2,1,0) raw, with
        // squared distance 5/4 raw²; the horizontal face has distance 1 raw².
        // Both witnesses round to (0,1,0), so only exact ranking picks its normal.
        Vector3d expectedPoint = new(Fixed64.Zero, Fixed64.FromRaw(1), Fixed64.Zero);
        Assert.Equal(expectedPoint, mesh.ClosestPointOnSurface(Vector3d.Zero));
        Assert.Equal(Vector3d.Down, mesh.GetNormalAtPoint(Vector3d.Zero));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void DetachedMesh_PoseAndFailedScale_ShouldPreserveItsAuthoredMappingAndIndex(bool largeSource, bool scalarFace)
    {
        Fixed64 sourceCenter = largeSource ? (Fixed64)1500000000 : Fixed64.Zero;
        Vector3d[] vertices = { new(sourceCenter - Fixed64.Two, Fixed64.Zero, (Fixed64)(-2)), new(sourceCenter + Fixed64.Two, Fixed64.Zero, (Fixed64)(-2)), new(sourceCenter, Fixed64.Zero, Fixed64.Two) };
        var mesh = new PhysicsMesh(vertices, new[] { 0, 2, 1 }, Vector3d.Zero, FixedQuaternion.Identity, MeshColliderMode.Concave);
        Vector3d scale = new(Fixed64.FromFraction(2, 3), Fixed64.One, Fixed64.FromFraction(3, 5));
        mesh.UpdateTransform(Vector3d.Zero, FixedQuaternion.Identity, scale);
        var nearby = new SwiftList<int>();
        mesh.GetTrianglesInLocalBounds(new FixedBoundVolume(mesh.ScaledLocalBounds.Min, mesh.ScaledLocalBounds.Max), nearby);
        Assert.Equal(new[] { 0 }, nearby.ToArray());
        mesh.GetTrianglesInLocalBounds(new FixedBoundVolume(new Vector3d((Fixed64)100, (Fixed64)100, (Fixed64)100),
            new Vector3d((Fixed64)101, (Fixed64)101, (Fixed64)101)), nearby);
        Assert.Empty(nearby);
        int builds = mesh.TriangleBvhBuildCount;
        Vector3d position = scalarFace ? new(Fixed64.MaxValue - (Fixed64)8, Fixed64.MinValue + (Fixed64)8, Fixed64.Zero) : new(3, 2, 1);
        var rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)45);
        for (int repeat = 0; repeat < 3; repeat++)
        {
            mesh.UpdateTransform(position, rotation, scale);
            for (int vertex = 0; vertex < vertices.Length; vertex++)
            {
                Vector3d expected = Evaluate(vertices[vertex], mesh.LocalBounds.Center, position, rotation, Vector3d.Zero,
                    scale, rotation, Vector3d.One);
                Assert.Equal(expected, mesh.GetVertexWorld(vertex));
                for (int axis = 0; axis < 3; axis++)
                    Assert.InRange(expected[axis].m_rawValue, mesh.Bounds.Min[axis].m_rawValue, mesh.Bounds.Max[axis].m_rawValue);
            }
            Assert.Equal(builds, mesh.TriangleBvhBuildCount);
        }
        Vector3d prior = mesh.GetVertexWorld(0);
        FixedBoundBox bounds = mesh.Bounds;
        Assert.Throws<ArgumentException>(() => mesh.UpdateTransform(Vector3d.One, FixedQuaternion.Identity, Vector3d.Zero));
        Assert.Equal(prior, mesh.GetVertexWorld(0));
        Assert.Equal(bounds, mesh.Bounds);
        Assert.Equal(builds, mesh.TriangleBvhBuildCount);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void OffsetAndCompoundPlacement_ShouldRoundTheCompleteAuthoredPointOnce(bool compound)
    {
        using var context = GravitasWorldContext.CreateOwned();
        Vector3d[] vertices = { new(-2, 0, -2), new(3, 0, -2), new(0, 0, 3) };
        int[] triangles = { 0, 2, 1 };
        Vector3d ownerScale = new(Fixed64.FromFraction(2, 3), Fixed64.One, Fixed64.FromFraction(3, 5));
        Vector3d partScale = compound ? new(Fixed64.FromFraction(7, 9), Fixed64.One, Fixed64.FromFraction(11, 13)) : Vector3d.One;
        Vector3d offset = new(Fixed64.FromFraction(1, 3), Fixed64.FromFraction(-2, 7), Fixed64.FromFraction(3, 11));
        var parentRotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)17, (Fixed64)23, (Fixed64)31);
        var partRotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)13, (Fixed64)(-19), (Fixed64)7);
        LSCollider owner = compound
            ? new LSCompoundCollider(CompoundColliderPart.ConvexMesh(vertices, triangles, offset, partRotation, partScale, MeshInertiaPolicy.SurfaceApproximation))
            : new LSMeshCollider(vertices, triangles, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation) { LocalOffset = offset };
        LSMeshCollider mesh = compound ? (LSMeshCollider)((LSCompoundCollider)owner).GetPartCollider(0) : (LSMeshCollider)owner;
        _ = mesh.Mesh.TotalArea; // Detached inspection must not become the attached geometry frame.
        var body = new SolidBody(new TestMatterAgent(context, new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, ownerScale)), owner) { Mass = Fixed64.One };
        Vector3d position = new(3, -2, 5);
        body.Initialize(position, parentRotation);
        Vector3d placementOrigin = compound ? owner.Center : position;
        for (int vertex = 0; vertex < vertices.Length; vertex++)
            Assert.Equal(Evaluate(vertices[vertex], mesh.Mesh.LocalBounds.Center, placementOrigin, parentRotation, offset,
                ownerScale, mesh.Rotation, partScale), mesh.Mesh.GetVertexWorld(vertex));
        body.Deactivate();
        body.Initialize(-position, FixedQuaternion.Identity);
        placementOrigin = compound ? owner.Center : -position;
        for (int vertex = 0; vertex < vertices.Length; vertex++)
            Assert.Equal(Evaluate(vertices[vertex], mesh.Mesh.LocalBounds.Center, placementOrigin, FixedQuaternion.Identity, offset,
                ownerScale, mesh.Rotation, partScale), mesh.Mesh.GetVertexWorld(vertex));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(45)]
    public void FractionalScale_ShouldKeepAnActualSubrawGapUnjoined(int degrees)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Vector3d scale = new(Fixed64.FromFraction(2, 3), Fixed64.One, Fixed64.FromFraction(3, 5));
        var rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)degrees);
        Fixed64 seam = Fixed64.FromRaw(1);
        LSMeshCollider first = Floor((Fixed64)(-4), seam), second = Floor(seam + Fixed64.MinIncrement, (Fixed64)5);
        var domain = new MeshSurfaceBoundaryDomain();
        domain.Build(new[] { first.Mesh, second.Mesh }, 0, 0);
        foreach (var span in domain.Spans) Assert.NotEqual(MeshSurfaceBoundaryDomain.SpanKind.Joined, span.Kind);

        LSMeshCollider Floor(Fixed64 lower, Fixed64 upper)
        {
            var collider = new LSMeshCollider(new[] { new Vector3d(lower, Fixed64.Zero, (Fixed64)(-4)),
                new Vector3d(upper, Fixed64.Zero, (Fixed64)(-4)), new Vector3d(lower, Fixed64.Zero, (Fixed64)4),
                new Vector3d(upper, Fixed64.Zero, (Fixed64)4) }, new[] { 0, 2, 1, 1, 2, 3 },
                MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
            collider.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(Vector3d.Zero, rotation, scale)));
            return collider;
        }
    }

    public static TheoryData<GravitasSerializationTransport> Transports => GravitasSerializationTransportCases.All();

    [Theory]
    [MemberData(nameof(Transports))]
    public void AffineMeshRestore_ShouldRetainSleepAndFullStepReplay(GravitasSerializationTransport transport)
    {
        using var sourceContext = GravitasWorldContext.CreateOwned();
        using var targetContext = GravitasWorldContext.CreateOwned();
        SolidBody source = CreateBody(sourceContext), target = CreateBody(targetContext);
        source.SetPosition(new Vector3d(3, 2, 1));
        source.SetRotation(FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)17, (Fixed64)23, (Fixed64)31));
        source.Sleep();
        GravitasSerializationHarness.Populate(target, GravitasSerializationHarness.Serialize(source, transport), transport);
        Assert.True(target.IsSleeping);
        for (int vertex = 0; vertex < 3; vertex++)
            Assert.Equal(((LSMeshCollider)source.Collider).Mesh.GetVertexWorld(vertex), ((LSMeshCollider)target.Collider).Mesh.GetVertexWorld(vertex));
        Assert.Equal(sourceContext.ComputeReplayHash(), targetContext.ComputeReplayHash());
        for (int frame = 0; frame < 4; frame++)
        {
            sourceContext.Simulate(); sourceContext.LateSimulate();
            targetContext.Simulate(); targetContext.LateSimulate();
            Assert.Equal(sourceContext.ComputeReplayHash(), targetContext.ComputeReplayHash());
        }

        static SolidBody CreateBody(GravitasWorldContext context)
        {
            var collider = new LSMeshCollider(new[] { new Vector3d(-2, 0, -2), new Vector3d(3, 0, -2), new Vector3d(0, 0, 3) },
                new[] { 0, 2, 1 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
            var body = new SolidBody(new TestMatterAgent(context, new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity,
                new Vector3d(Fixed64.FromFraction(2, 3), Fixed64.One, Fixed64.FromFraction(3, 5)))), collider) { Mass = Fixed64.One };
            body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
            return body;
        }
    }

    private static Vector3d Evaluate(Vector3d vertex, Vector3d reference, Vector3d translation,
        FixedQuaternion placement, Vector3d offset, Vector3d ownerScale, FixedQuaternion geometry, Vector3d partScale)
    {
        var p = Rotation(placement); var g = Rotation(geometry);
        BigInteger q = BigInteger.One << 32, denominator = q * q * p.Denominator * g.Denominator;
        Vector3d result = default;
        for (int axis = 0; axis < 3; axis++)
        {
            BigInteger value = translation[axis].m_rawValue * denominator;
            for (int coordinate = 0; coordinate < 3; coordinate++)
            {
                value += p.Matrix[axis, coordinate] * offset[coordinate].m_rawValue * ownerScale[coordinate].m_rawValue * q * g.Denominator;
                value += g.Matrix[axis, coordinate] * ((BigInteger)vertex[coordinate].m_rawValue - reference[coordinate].m_rawValue)
                    * ownerScale[coordinate].m_rawValue * partScale[coordinate].m_rawValue * p.Denominator;
            }
            int sign = value.Sign;
            BigInteger quotient = BigInteger.DivRem(BigInteger.Abs(value), denominator, out BigInteger remainder);
            if (2 * remainder > denominator || 2 * remainder == denominator && !quotient.IsEven) quotient++;
            result[axis] = Fixed64.FromRaw((long)(sign * quotient));
        }
        return result;

        static (BigInteger[,] Matrix, BigInteger Denominator) Rotation(FixedQuaternion r)
        {
            BigInteger x = r.X.m_rawValue, y = r.Y.m_rawValue, z = r.Z.m_rawValue, w = r.W.m_rawValue;
            return (new[,] { { w*w+x*x-y*y-z*z, 2*(x*y-z*w), 2*(x*z+y*w) },
                { 2*(x*y+z*w), w*w-x*x+y*y-z*z, 2*(y*z-x*w) },
                { 2*(x*z-y*w), 2*(y*z+x*w), w*w-x*x-y*y+z*z } }, x*x+y*y+z*z+w*w);
        }
    }
}
