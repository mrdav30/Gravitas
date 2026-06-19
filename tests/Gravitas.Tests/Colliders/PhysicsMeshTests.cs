using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using SwiftCollections;
using SwiftCollections.Query;
using System;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class PhysicsMeshTests
{
    [Fact]
    public void Constructor_ShouldRejectInvalidMeshInput()
    {
        Action nullVertices = () => _ = new PhysicsMesh(null!, ValidTriangles(), Vector3d.Zero, FixedQuaternion.Identity);
        Action nullTriangles = () => _ = new PhysicsMesh(ValidVertices(), null!, Vector3d.Zero, FixedQuaternion.Identity);
        Action invalidTriangleCount = () => _ = new PhysicsMesh(ValidVertices(), new[] { 0, 1 }, Vector3d.Zero, FixedQuaternion.Identity);
        Action outOfRangeTriangle = () => _ = new PhysicsMesh(ValidVertices(), new[] { 0, 1, 3 }, Vector3d.Zero, FixedQuaternion.Identity);
        Action duplicateTriangleIndex = () => _ = new PhysicsMesh(ValidVertices(), new[] { 0, 1, 1 }, Vector3d.Zero, FixedQuaternion.Identity);
        Action degenerateTriangle = () => _ = new PhysicsMesh(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Right * 2
            },
            ValidTriangles(),
            Vector3d.Zero,
            FixedQuaternion.Identity);

        nullVertices.Should().Throw<ArgumentNullException>();
        nullTriangles.Should().Throw<ArgumentNullException>();
        invalidTriangleCount.Should().Throw<ArgumentException>();
        outOfRangeTriangle.Should().Throw<ArgumentOutOfRangeException>();
        duplicateTriangleIndex.Should().Throw<ArgumentException>();
        degenerateTriangle.Should().Throw<ArgumentException>();
    }

    [Fact]
    public void Constructor_ShouldRejectMeshesBeyondDeterministicLimits()
    {
        Vector3d[] tooManyVertices = new Vector3d[PhysicsMesh.MaxVertexCount + 1];
        int[] triangles = ValidTriangles();

        Action create = () => _ = new PhysicsMesh(
            tooManyVertices,
            triangles,
            Vector3d.Zero,
            FixedQuaternion.Identity);

        create.Should().Throw<ArgumentOutOfRangeException>();
    }

    [Fact]
    public void TriangleBVH_ShouldStoreTriangleBoundsAsMinMax()
    {
        var mesh = new PhysicsMesh(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Up,
                new Vector3d((Fixed64)10, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)11, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)10, Fixed64.One, Fixed64.Zero)
            },
            new[] { 0, 1, 2, 3, 4, 5 },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        var hits = new SwiftList<int>();

        mesh.TriangleBVH.Query(
            new FixedBoundVolume(
                new Vector3d((Fixed64)10, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)11, Fixed64.One, Fixed64.Zero)),
            hits);

        hits.Should().Contain(1);
        hits.Should().NotContain(0);
    }

    [Fact]
    public void TriangleBVH_ShouldStayLocalWhenRigidMeshMoves()
    {
        var mesh = new PhysicsMesh(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Up,
                new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)2, Fixed64.One, Fixed64.Zero)
            },
            new[] { 0, 1, 2, 3, 4, 5 },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        var hits = new SwiftList<int>();

        mesh.TriangleBVH.Query(
            new FixedBoundVolume(Vector3d.Zero, Vector3d.One),
            hits);
        int buildCount = mesh.TriangleBvhBuildCount;

        mesh.UpdatePosition(
            new Vector3d((Fixed64)10, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90));
        mesh.TriangleBVH.Query(
            new FixedBoundVolume(Vector3d.Zero, Vector3d.One),
            hits);

        mesh.TriangleBvhBuildCount.Should().Be(buildCount);
        hits.Should().Contain(0);
    }

    [Fact]
    public void Constructor_ShouldStoreExplicitMeshColliderMode()
    {
        var collider = new LSMeshCollider(
            ValidVertices(),
            ValidTriangles(),
            MeshColliderMode.Concave);

        collider.Mode.Should().Be(MeshColliderMode.Concave);
        collider.Mesh.Mode.Should().Be(MeshColliderMode.Concave);
    }

    [Fact]
    public void ConcaveMesh_WithSurfaceApproximation_ShouldAllowDynamicBodyInitialization()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var collider = new LSMeshCollider(
            ValidVertices(),
            ValidTriangles(),
            MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);

        ScenarioBody<LSMeshCollider> body = scenario.CreateBody(
            collider,
            Vector3d.Zero,
            FixedQuaternion.Identity);

        body.Collider.Mode.Should().Be(MeshColliderMode.Concave);
        body.Body.IsKinematic.Should().BeFalse();
    }

    [Fact]
    public void ConcaveMesh_ShouldAllowKinematicBodyInitialization()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var collider = new LSMeshCollider(
            ValidVertices(),
            ValidTriangles(),
            MeshColliderMode.Concave);

        ScenarioBody<LSMeshCollider> body = scenario.CreateBody(
            collider,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            isKinematic: true);

        body.Collider.Mode.Should().Be(MeshColliderMode.Concave);
    }

    [Fact]
    public void CalculateInertiaTensor_ShouldUseLocalGeometryForRigidMovement()
    {
        PhysicsMesh mesh = MeshTestFixtures.CreateConvexCube().Mesh;

        Fixed3x3 originTensor = mesh.CalculateInertiaTensor((Fixed64)3);
        mesh.UpdatePosition(new Vector3d((Fixed64)10, (Fixed64)4, (Fixed64)(-2)), FixedQuaternion.Identity);

        mesh.CalculateInertiaTensor((Fixed64)3).Should().Be(originTensor);
    }

    [Fact]
    public void CalculateInertiaTensor_ForClosedUnitCube_ShouldMatchSolidBox()
    {
        PhysicsMesh mesh = MeshTestFixtures.CreateConvexCube().Mesh;

        Fixed3x3 tensor = mesh.CalculateInertiaTensor((Fixed64)3);

        Fixed64 expected = Fixed64.Half;
        AssertNear(tensor.M11, expected);
        AssertNear(tensor.M22, expected);
        AssertNear(tensor.M33, expected);
        tensor.M12.Should().Be(Fixed64.Zero);
        tensor.M13.Should().Be(Fixed64.Zero);
        tensor.M21.Should().Be(Fixed64.Zero);
        tensor.M23.Should().Be(Fixed64.Zero);
        tensor.M31.Should().Be(Fixed64.Zero);
        tensor.M32.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void CalculateInertiaTensor_ForReversedClosedVolume_ShouldMatchForwardWinding()
    {
        LSMeshCollider forward = MeshTestFixtures.CreateConvexCube();
        LSMeshCollider reversed = CreateReversedCube();

        Fixed3x3 forwardTensor = forward.Mesh.CalculateInertiaTensor((Fixed64)3);
        Fixed3x3 reversedTensor = reversed.Mesh.CalculateInertiaTensor((Fixed64)3);

        reversedTensor.Should().Be(forwardTensor);
    }

    [Fact]
    public void CalculateInertiaTensor_ForRectangularPrism_ShouldMatchSolidBox()
    {
        var mesh = new PhysicsMesh(
            BoxVertices(Fixed64.One, Fixed64.Half, (Fixed64)2),
            CubeTriangles(),
            Vector3d.Zero,
            FixedQuaternion.Identity);

        Fixed3x3 tensor = mesh.CalculateInertiaTensor((Fixed64)6);

        AssertNear(tensor.M11, Fixed64.FromFraction(17, 2));
        AssertNear(tensor.M22, (Fixed64)10);
        AssertNear(tensor.M33, Fixed64.FromFraction(5, 2));
        tensor.M12.Should().Be(Fixed64.Zero);
        tensor.M13.Should().Be(Fixed64.Zero);
        tensor.M21.Should().Be(Fixed64.Zero);
        tensor.M23.Should().Be(Fixed64.Zero);
        tensor.M31.Should().Be(Fixed64.Zero);
        tensor.M32.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void TryGetClosedVolumeMassProperties_ForTetrahedron_ShouldCalculateVolumeAndCenterOfMass()
    {
        var mesh = new PhysicsMesh(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Up,
                Vector3d.Forward
            },
            new[]
            {
                1, 2, 3,
                0, 2, 1,
                0, 1, 3,
                0, 3, 2
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);

        bool valid = mesh.TryGetClosedVolumeMassProperties(
            out MeshMassProperties properties,
            out MeshVolumeValidationResult result);

        valid.Should().BeTrue();
        result.Should().Be(MeshVolumeValidationResult.Valid);
        AssertNear(properties.Volume, Fixed64.FromFraction(1, 6));
        AssertNear(properties.CenterOfMass.X, Fixed64.FromFraction(1, 4));
        AssertNear(properties.CenterOfMass.Y, Fixed64.FromFraction(1, 4));
        AssertNear(properties.CenterOfMass.Z, Fixed64.FromFraction(1, 4));
    }

    [Fact]
    public void CalculateInertiaTensor_ForOffCenterClosedVolume_ShouldShiftReferenceTensorToCenterOfMass()
    {
        var mesh = new PhysicsMesh(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Up,
                Vector3d.Forward
            },
            new[]
            {
                1, 2, 3,
                0, 2, 1,
                0, 1, 3,
                0, 3, 2
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        mesh.TryGetClosedVolumeMassProperties(
            out MeshMassProperties properties,
            out _).Should().BeTrue();
        Fixed64 mass = (Fixed64)6;
        Fixed3x3 referenceTensor = properties.UnitMassInertiaTensor * mass;
        Vector3d referenceOffset = properties.InertiaReferencePoint - properties.CenterOfMass;

        Fixed3x3 tensor = mesh.CalculateInertiaTensor(mass);

        AssertNear(
            referenceTensor.M11,
            tensor.M11 + mass * ((referenceOffset.Y * referenceOffset.Y) + (referenceOffset.Z * referenceOffset.Z)));
        AssertNear(
            referenceTensor.M22,
            tensor.M22 + mass * ((referenceOffset.X * referenceOffset.X) + (referenceOffset.Z * referenceOffset.Z)));
        AssertNear(
            referenceTensor.M33,
            tensor.M33 + mass * ((referenceOffset.X * referenceOffset.X) + (referenceOffset.Y * referenceOffset.Y)));
    }

    [Fact]
    public void CalculateInertiaTensor_WithSurfaceApproximationOptIn_ShouldAllowOpenMesh()
    {
        var mesh = new PhysicsMesh(
            ValidVertices(),
            ValidTriangles(),
            Vector3d.Zero,
            FixedQuaternion.Identity);

        Action calculate = () => _ = mesh.CalculateInertiaTensor((Fixed64)3, MeshInertiaPolicy.SurfaceApproximation);

        calculate.Should().NotThrow();
    }

    [Theory]
    [MemberData(nameof(InvalidClosedVolumeMeshes))]
    public void TryGetClosedVolumeMassProperties_ShouldReturnExplicitFailureReasons(
        Vector3d[] vertices,
        int[] triangles,
        MeshVolumeValidationResult expectedResult)
    {
        var mesh = new PhysicsMesh(vertices, triangles, Vector3d.Zero, FixedQuaternion.Identity);

        bool valid = mesh.TryGetClosedVolumeMassProperties(
            out MeshMassProperties properties,
            out MeshVolumeValidationResult result);

        valid.Should().BeFalse();
        properties.Should().Be(default(MeshMassProperties));
        result.Should().Be(expectedResult);
    }

    [Fact]
    public void MeshColliderRotation_ShouldRefreshWorldBoundsFromTransformedVertices()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var meshCollider = new LSMeshCollider(
            new[]
            {
                new Vector3d((Fixed64)(-2), (Fixed64)(-1), Fixed64.Zero),
                new Vector3d((Fixed64)2, (Fixed64)(-1), Fixed64.Zero),
                new Vector3d((Fixed64)(-2), Fixed64.One, Fixed64.Zero),
                new Vector3d((Fixed64)2, Fixed64.One, Fixed64.Zero)
            },
            new[] { 0, 1, 2, 1, 3, 2 },
            MeshColliderMode.Convex,
            MeshInertiaPolicy.SurfaceApproximation);
        ScenarioBody<LSMeshCollider> body = scenario.CreateBody(
            meshCollider,
            Vector3d.Zero,
            FixedQuaternion.Identity);

        body.Collider.Bounds.Proportions.X.Should().Be((Fixed64)4);
        body.Collider.Bounds.Proportions.Y.Should().Be((Fixed64)2);

        body.Body.SetRotation(FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90));
        body.Collider.Simulate();

        AssertNear(body.Collider.Bounds.Proportions.X, (Fixed64)2);
        AssertNear(body.Collider.Bounds.Proportions.Y, (Fixed64)4);
    }

    private static Vector3d[] ValidVertices() =>
        new[]
        {
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Up
        };

    private static int[] ValidTriangles() => new[] { 0, 1, 2 };

    public static TheoryData<Vector3d[], int[], MeshVolumeValidationResult> InvalidClosedVolumeMeshes()
    {
        TheoryData<Vector3d[], int[], MeshVolumeValidationResult> data = new();
        data.Add(ValidVertices(), ValidTriangles(), MeshVolumeValidationResult.BoundaryEdge);
        data.Add(CubeVertices(), CubeTrianglesWithDuplicateFace(), MeshVolumeValidationResult.DuplicateTriangle);
        data.Add(NonManifoldEdgeVertices(), NonManifoldEdgeTriangles(), MeshVolumeValidationResult.NonManifoldEdge);
        data.Add(DisconnectedCubeVertices(), DisconnectedCubeTriangles(), MeshVolumeValidationResult.DisconnectedShell);
        return data;
    }

    private static LSMeshCollider CreateReversedCube()
    {
        Vector3d[] vertices = CubeVertices();
        int[] triangles = CubeTriangles();
        for (int i = 0; i < triangles.Length; i += 3)
            (triangles[i + 1], triangles[i + 2]) = (triangles[i + 2], triangles[i + 1]);

        return new LSMeshCollider(vertices, triangles);
    }

    private static Vector3d[] CubeVertices()
    {
        Fixed64 half = Fixed64.Half;
        return BoxVertices(half, half, half);
    }

    private static Vector3d[] BoxVertices(Fixed64 halfX, Fixed64 halfY, Fixed64 halfZ)
    {
        return new[]
        {
            new Vector3d(-halfX, -halfY, -halfZ),
            new Vector3d(halfX, -halfY, -halfZ),
            new Vector3d(-halfX, halfY, -halfZ),
            new Vector3d(halfX, halfY, -halfZ),
            new Vector3d(-halfX, -halfY, halfZ),
            new Vector3d(halfX, -halfY, halfZ),
            new Vector3d(-halfX, halfY, halfZ),
            new Vector3d(halfX, halfY, halfZ)
        };
    }

    private static int[] CubeTriangles() =>
        new[]
        {
            0, 2, 1, 1, 2, 3,
            4, 5, 6, 5, 7, 6,
            0, 4, 2, 2, 4, 6,
            1, 3, 5, 5, 3, 7,
            0, 1, 4, 1, 5, 4,
            2, 6, 3, 3, 6, 7
        };

    private static int[] CubeTrianglesWithDuplicateFace()
    {
        int[] triangles = CubeTriangles();
        int[] result = new int[triangles.Length + 3];
        Array.Copy(triangles, result, triangles.Length);
        result[^3] = triangles[0];
        result[^2] = triangles[1];
        result[^1] = triangles[2];
        return result;
    }

    private static Vector3d[] NonManifoldEdgeVertices() =>
        new[]
        {
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Up,
            Vector3d.Forward,
            new Vector3d(Fixed64.Zero, -Fixed64.One, Fixed64.Zero)
        };

    private static int[] NonManifoldEdgeTriangles() =>
        new[]
        {
            0, 1, 2,
            1, 0, 3,
            0, 1, 4
        };

    private static Vector3d[] DisconnectedCubeVertices()
    {
        Vector3d[] first = CubeVertices();
        Vector3d[] result = new Vector3d[first.Length * 2];
        Array.Copy(first, result, first.Length);
        for (int i = 0; i < first.Length; i++)
            result[first.Length + i] = first[i] + new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero);

        return result;
    }

    private static int[] DisconnectedCubeTriangles()
    {
        int[] first = CubeTriangles();
        int[] result = new int[first.Length * 2];
        Array.Copy(first, result, first.Length);
        for (int i = 0; i < first.Length; i++)
            result[first.Length + i] = first[i] + 8;

        return result;
    }

    private static void AssertNear(Fixed64 actual, Fixed64 expected) =>
        (actual - expected).Abs().Should().BeLessThan(Fixed64.FromFraction(1, 1000));
}
