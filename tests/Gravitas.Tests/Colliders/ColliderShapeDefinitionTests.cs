using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System;
using System.Linq;
using System.Reflection;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class ColliderShapeDefinitionTests
{
    public static TheoryData<ColliderShapeDefinition, Type> RuntimeColliderDefinitions => new()
    {
        { ColliderShapeDefinition.Sphere(Fixed64.Half), typeof(LSSphereCollider) },
        { ColliderShapeDefinition.Capsule(Fixed64.Half, (Fixed64)2), typeof(LSCapsuleCollider) },
        { ColliderShapeDefinition.Cuboid(Vector3d.One), typeof(LSCuboidCollider) },
        { ColliderShapeDefinition.Cylinder(Fixed64.Half, (Fixed64)2), typeof(LSCylinderCollider) },
        { ColliderShapeDefinition.Cone(Fixed64.Half, (Fixed64)2), typeof(LSConeCollider) },
        { ColliderShapeDefinition.ConvexMesh(
            MeshTestFixtures.CreateConvexCube().Mesh.LocalVertices.ToArray(),
            MeshTestFixtures.CreateConvexCube().Mesh.Triangles.ToArray()),
            typeof(LSMeshCollider) }
    };

    [Theory]
    [MemberData(nameof(RuntimeColliderDefinitions))]
    public void ShapeDefinition_ShouldCreateEverySupportedRuntimeColliderKind(
        ColliderShapeDefinition definition,
        Type expectedType)
    {
        LSCollider collider = definition.CreateCollider();

        collider.Should().BeOfType(expectedType);
    }

    [Fact]
    public void CuboidDefinition_ShouldBuildEquivalentStandaloneCollider()
    {
        Vector3d size = new((Fixed64)2, (Fixed64)3, (Fixed64)4);
        ColliderShapeDefinition definition = ColliderShapeDefinition.Cuboid(size);

        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> body = scenario.CreateBody(
            new LSCuboidCollider(definition),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity);

        definition.Kind.Should().Be(ColliderShapeDefinitionKind.Cuboid);
        definition.Size.Should().Be(size);
        body.Collider.ScaledSize.Should().Be(size);
        body.Collider.BoundsMin.Should().Be(new Vector3d(-Fixed64.One, Fixed64.FromFraction(-3, 2), (Fixed64)(-2)));
        body.Collider.BoundsMax.Should().Be(new Vector3d(Fixed64.One, Fixed64.FromFraction(3, 2), (Fixed64)2));
    }

    [Fact]
    public void ConeDefinition_ShouldBuildEquivalentStandaloneCollider()
    {
        ColliderShapeDefinition definition = ColliderShapeDefinition.Cone(Fixed64.Half, (Fixed64)2);

        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSConeCollider> body = scenario.CreateBody(
            new LSConeCollider(definition),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity);

        definition.Kind.Should().Be(ColliderShapeDefinitionKind.Cone);
        definition.Radius.Should().Be(Fixed64.Half);
        definition.Height.Should().Be((Fixed64)2);
        definition.Size.Should().Be(new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One));
        body.Collider.Shape.Should().Be(ColliderType.Cone);
        body.Collider.BaseCenter.Should().Be(new Vector3d(Fixed64.Zero, -Fixed64.One, Fixed64.Zero));
        body.Collider.Apex.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));
        body.Body.LocalCenterOfMassOffset.Should().Be(new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero));
    }

    [Fact]
    public void ConeDefinition_ShouldRejectInvalidDimensions()
    {
        Action zeroRadius = () => ColliderShapeDefinition.Cone(Fixed64.Zero, Fixed64.One);
        Action zeroHeight = () => ColliderShapeDefinition.Cone(Fixed64.Half, Fixed64.Zero);

        zeroRadius.Should().Throw<ArgumentException>().WithParameterName("radius");
        zeroHeight.Should().Throw<ArgumentException>().WithParameterName("height");
    }

    [Theory]
    [InlineData(0, 1, 1)]
    [InlineData(1, 0, 1)]
    [InlineData(1, 1, 0)]
    public void CuboidDefinition_ShouldRejectNonPositiveSizeComponents(int x, int y, int z)
    {
        Action create = () => _ = ColliderShapeDefinition.Cuboid(new Vector3d((Fixed64)x, (Fixed64)y, (Fixed64)z));

        create.Should().Throw<ArgumentException>().WithParameterName("size");
    }

    [Fact]
    public void ConvexMeshDefinition_ShouldSnapshotSourceArrays()
    {
        LSMeshCollider source = MeshTestFixtures.CreateConvexCube();
        Vector3d originalVertex = source.Mesh.LocalVertices[0];
        int originalIndex = source.Mesh.Triangles[0];

        Vector3d[] vertices = source.Mesh.LocalVertices.ToArray();
        int[] triangles = source.Mesh.Triangles.ToArray();
        ColliderShapeDefinition definition = ColliderShapeDefinition.ConvexMesh(
            vertices,
            triangles,
            MeshInertiaPolicy.RequireClosedVolume);

        vertices[0] = new Vector3d((Fixed64)99, (Fixed64)99, (Fixed64)99);
        triangles[0] = triangles[0] + 1;

        definition.Kind.Should().Be(ColliderShapeDefinitionKind.ConvexMesh);
        definition.MeshVertexCount.Should().Be(source.Mesh.VertexCount);
        definition.MeshTriangleIndexCount.Should().Be(source.Mesh.Triangles.Length);
        definition.GetMeshVertex(0).Should().Be(originalVertex);
        definition.GetMeshTriangleIndex(0).Should().Be(originalIndex);
        definition.MeshInertiaPolicy.Should().Be(MeshInertiaPolicy.RequireClosedVolume);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(3)]
    public void ConvexMeshDefinition_ShouldRejectTriangleIndicesOutsideVertexArray(int vertexIndex)
    {
        Vector3d[] vertices =
        {
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Up
        };

        Action create = () => ColliderShapeDefinition.ConvexMesh(vertices, new[] { 0, 1, vertexIndex });

        create.Should().Throw<ArgumentException>().WithParameterName("triangles");
    }

    [Fact]
    public void ShapeDefinitionAccessors_ShouldRejectUndefinedAndWrongShapeFamilies()
    {
        ColliderShapeDefinition undefined = default;
        ColliderShapeDefinition sphere = ColliderShapeDefinition.Sphere(Fixed64.One);

        Action createUndefined = () => undefined.CreateCollider();
        Action readUndefinedMeshVertex = () => _ = undefined.GetMeshVertex(0);
        Action readSphereMeshVertex = () => _ = sphere.GetMeshVertex(0);
        Action readSphereTriangleIndex = () => _ = sphere.GetMeshTriangleIndex(0);
        Action buildSphereFromCuboidDefinition = () => _ = new LSSphereCollider(ColliderShapeDefinition.Cuboid(Vector3d.One));

        createUndefined.Should().Throw<ArgumentException>().WithParameterName(nameof(ColliderShapeDefinition));
        readUndefinedMeshVertex.Should().Throw<ArgumentException>().WithParameterName(nameof(ColliderShapeDefinition));
        readSphereMeshVertex.Should().Throw<ArgumentException>().WithParameterName(nameof(ColliderShapeDefinition));
        readSphereTriangleIndex.Should().Throw<ArgumentException>().WithParameterName(nameof(ColliderShapeDefinition));
        buildSphereFromCuboidDefinition.Should().Throw<ArgumentException>().WithParameterName(nameof(ColliderShapeDefinition));
    }

    [Fact]
    public void ShapeDefinitionEqualityAndHash_ShouldEncodeAuthoredShapeMaterialAndMeshPayload()
    {
        PhysicsMaterial material = PhysicsMaterialTestHelper.WithFrictionAndRestitution(
            Fixed64.FromFraction(3, 4),
            Fixed64.FromFraction(1, 4));
        PhysicsMaterial otherMaterial = PhysicsMaterialTestHelper.WithFrictionAndRestitution(
            Fixed64.Half,
            Fixed64.FromFraction(1, 8));
        Vector3d[] vertices =
        {
            new(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            new(Fixed64.One, Fixed64.Zero, Fixed64.Zero),
            new(Fixed64.Zero, Fixed64.One, Fixed64.Zero)
        };
        int[] triangles = { 0, 1, 2 };
        ColliderShapeDefinition definition = ColliderShapeDefinition.ConvexMesh(
            vertices,
            triangles,
            MeshInertiaPolicy.RequireClosedVolume,
            material);
        ColliderShapeDefinition same = ColliderShapeDefinition.ConvexMesh(
            vertices.ToArray(),
            triangles.ToArray(),
            MeshInertiaPolicy.RequireClosedVolume,
            material);

        definition.Should().Be(same);
        definition.Equals((object)same).Should().BeTrue();
        definition.GetHashCode().Should().Be(same.GetHashCode());
        (definition == same).Should().BeTrue();
        (definition != same).Should().BeFalse();

        definition.Equals("mesh").Should().BeFalse();
        definition.Should().NotBe(ColliderShapeDefinition.Sphere(Fixed64.One, material));
        definition.Should().NotBe(ColliderShapeDefinition.ConvexMesh(
            vertices,
            triangles,
            MeshInertiaPolicy.SurfaceApproximation,
            material));
        definition.Should().NotBe(ColliderShapeDefinition.ConvexMesh(vertices, triangles));
        definition.Should().NotBe(ColliderShapeDefinition.ConvexMesh(
            vertices,
            triangles,
            MeshInertiaPolicy.RequireClosedVolume,
            otherMaterial));
        definition.Should().NotBe(ColliderShapeDefinition.ConvexMesh(
            new[] { vertices[1], vertices[0], vertices[2] },
            triangles,
            MeshInertiaPolicy.RequireClosedVolume,
            material));
        definition.Should().NotBe(ColliderShapeDefinition.ConvexMesh(
            vertices,
            new[] { 0, 2, 1 },
            MeshInertiaPolicy.RequireClosedVolume,
            material));
        definition.Should().NotBe(ColliderShapeDefinition.ConvexMesh(
            new[]
            {
                vertices[0],
                vertices[1],
                vertices[2],
                new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.One)
            },
            triangles,
            MeshInertiaPolicy.RequireClosedVolume,
            material));
        definition.Should().NotBe(ColliderShapeDefinition.ConvexMesh(
            vertices,
            new[] { 0, 1, 2, 0, 2, 1 },
            MeshInertiaPolicy.RequireClosedVolume,
            material));
        ColliderShapeDefinition.Capsule(Fixed64.Half, (Fixed64)2)
            .Should()
            .NotBe(ColliderShapeDefinition.Capsule(Fixed64.Half, (Fixed64)3));
        ColliderShapeDefinition.Cuboid(Vector3d.One)
            .Should()
            .NotBe(ColliderShapeDefinition.Cuboid(new Vector3d(Fixed64.One, Fixed64.One, (Fixed64)2)));
    }

    [Fact]
    public void ShapeDefinitionEqualityAndHash_ShouldHandleDefaultMaterialAndObjectNull()
    {
        ColliderShapeDefinition sphere = ColliderShapeDefinition.Sphere(Fixed64.Half);
        ColliderShapeDefinition same = ColliderShapeDefinition.Sphere(Fixed64.Half);
        ColliderShapeDefinition explicitDefaultMaterial = ColliderShapeDefinition.Sphere(
            Fixed64.Half,
            PhysicsMaterial.Default);
        ColliderShapeDefinition differentRadius = ColliderShapeDefinition.Sphere(Fixed64.One);
        ColliderShapeDefinition capsule = ColliderShapeDefinition.Capsule(Fixed64.Half, (Fixed64)2);

        sphere.Should().Be(same);
        sphere.GetHashCode().Should().Be(same.GetHashCode());
        sphere.Material.Should().Be(explicitDefaultMaterial.Material);
        sphere.Should().NotBe(explicitDefaultMaterial);
        sphere.Equals((object?)null).Should().BeFalse();
        sphere.Should().NotBe(differentRadius);
        sphere.Should().NotBe(capsule);
    }

    [Fact]
    public void ShapeDefinitions_ShouldMaterializeEverySupportedRuntimeCollider()
    {
        Vector3d[] vertices =
        {
            new(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            new(Fixed64.One, Fixed64.Zero, Fixed64.Zero),
            new(Fixed64.Zero, Fixed64.One, Fixed64.Zero)
        };
        int[] triangles = { 0, 1, 2 };

        ColliderShapeDefinition.Sphere(Fixed64.Half).CreateCollider().Should().BeOfType<LSSphereCollider>();
        ColliderShapeDefinition.Capsule(Fixed64.Half, (Fixed64)2).CreateCollider().Should().BeOfType<LSCapsuleCollider>();
        ColliderShapeDefinition.Cuboid(Vector3d.One).CreateCollider().Should().BeOfType<LSCuboidCollider>();
        ColliderShapeDefinition.Cylinder(Fixed64.Half, Fixed64.One).CreateCollider().Should().BeOfType<LSCylinderCollider>();
        ColliderShapeDefinition.Cone(Fixed64.Half, Fixed64.One).CreateCollider().Should().BeOfType<LSConeCollider>();
        ColliderShapeDefinition.ConvexMesh(vertices, triangles).CreateCollider().Should().BeOfType<LSMeshCollider>();
    }

    [Fact]
    public void CylinderAndConeFrontalArea_ShouldUseAxialRadialAndZeroDirectionGeometry()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCylinderCollider> cylinder = scenario.CreateCylinder(Vector3d.Zero);
        ScenarioBody<LSConeCollider> cone = scenario.CreateCone(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));

        Fixed64 cylinderBaseArea = Fixed64.Pi * cylinder.Collider.ScaledRadiusSqr;
        Fixed64 coneBaseArea = Fixed64.Pi * cone.Collider.ScaledRadiusSqr;

        cylinder.Collider.GetFrontalArea(Vector3d.Zero).Should().Be(cylinder.Collider.Area);
        cylinder.Collider.GetFrontalArea(Vector3d.Up).Should().Be(cylinderBaseArea);
        cylinder.Collider.GetFrontalArea(Vector3d.Right)
            .Should()
            .Be((Fixed64)2 * cylinder.Collider.ScaledRadius * cylinder.Collider.Height);

        cone.Collider.GetFrontalArea(Vector3d.Zero).Should().Be(cone.Collider.Area);
        cone.Collider.GetFrontalArea(Vector3d.Up).Should().Be(coneBaseArea);
        cone.Collider.GetFrontalArea(Vector3d.Right)
            .Should()
            .Be(cone.Collider.ScaledRadius * cone.Collider.Height);
    }

    [Fact]
    public void CuboidFrontalArea_ShouldUseExactOrthographicProjectionInWorldSpace()
    {
        Vector3d size = new((Fixed64)2, (Fixed64)4, (Fixed64)6);
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> axisAligned = scenario.CreateBody(
            new LSCuboidCollider { Size = size },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSCuboidCollider> rotated = scenario.CreateBody(
            new LSCuboidCollider { Size = size },
            new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90));
        Vector3d diagonal = new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One).Normalized;
        Fixed64 expectedDiagonal =
            (Fixed64)24 * diagonal.X.Abs()
            + (Fixed64)12 * diagonal.Y.Abs()
            + (Fixed64)8 * diagonal.Z.Abs();

        axisAligned.Collider.GetFrontalArea(Vector3d.Zero).Should().Be(axisAligned.Collider.Area);
        axisAligned.Collider.GetFrontalArea(Vector3d.Right).Should().Be((Fixed64)24);
        axisAligned.Collider.GetFrontalArea(Vector3d.Up).Should().Be((Fixed64)12);
        axisAligned.Collider.GetFrontalArea(Vector3d.Forward).Should().Be((Fixed64)8);
        axisAligned.Collider.GetFrontalArea(diagonal).Should().Be(expectedDiagonal);
        rotated.Collider.GetFrontalArea(Vector3d.Up).Should().Be((Fixed64)24);
        rotated.Collider.GetFrontalArea(Vector3d.Right).Should().Be((Fixed64)12);
    }

    [Fact]
    public void CapsuleNormal_ShouldUseDeterministicFallbacksAtCapCentersAndAxis()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateCapsule(Vector3d.Zero);

        Vector3d topCenter = capsule.Collider.Center + capsule.Collider.Rotation * capsule.Collider.HemisphereCenterTop;
        Vector3d bottomCenter = capsule.Collider.Center + capsule.Collider.Rotation * capsule.Collider.HemisphereCenterBottom;

        capsule.Collider.GetNormalAtPoint(topCenter).Should().Be(Vector3d.Right);
        capsule.Collider.GetNormalAtPoint(bottomCenter).Should().Be(Vector3d.Right);
        capsule.Collider.GetNormalAtPoint(capsule.Collider.Center).Should().Be(Vector3d.Right);
        capsule.Collider.GetNormalAtPoint(topCenter + Vector3d.Up * capsule.Collider.ScaledRadius)
            .Should()
            .Be(Vector3d.Up);
        capsule.Collider.GetNormalAtPoint(bottomCenter - Vector3d.Up * capsule.Collider.ScaledRadius)
            .Should()
            .Be(-Vector3d.Up);
    }

    [Fact]
    public void ShapeDefinition_ShouldNotExposeRuntimeLifecycleState()
    {
        Type definitionType = typeof(ColliderShapeDefinition);
        string[] publicMemberNames = definitionType
            .GetMembers(BindingFlags.Instance | BindingFlags.Public)
            .Select(member => member.Name)
            .Distinct()
            .ToArray();

        publicMemberNames.Should().NotContain(new[]
        {
            nameof(LSCollider.Id),
            nameof(LSCollider.Body),
            nameof(LSCollider.Context),
            nameof(LSCollider.PartitionCoordinates),
            nameof(LSCollider.OnContact),
            nameof(LSCollider.OnTriggerEnter)
        });

        definitionType.GetFields(BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public)
            .Select(field => field.FieldType)
            .Should()
            .NotContain(type => typeof(LSCollider).IsAssignableFrom(type));
    }
}
