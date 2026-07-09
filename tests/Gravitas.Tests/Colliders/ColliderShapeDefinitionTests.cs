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
