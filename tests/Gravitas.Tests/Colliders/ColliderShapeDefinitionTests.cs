using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System;
using System.Linq;
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
        body.Collider.OrientedBox.HalfExtents.Should().Be(size / 2);
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
        FixedSegment.TryGetCenteredAxisEndpoint(
            body.Collider.Center,
            body.Collider.WorldAxis,
            body.Collider.Height,
            positive: false,
            out Vector3d baseCenter).Should().BeTrue();
        FixedSegment.TryGetCenteredAxisEndpoint(
            body.Collider.Center,
            body.Collider.WorldAxis,
            body.Collider.Height,
            positive: true,
            out Vector3d apex).Should().BeTrue();
        baseCenter.Should().Be(new Vector3d(Fixed64.Zero, -Fixed64.One, Fixed64.Zero));
        apex.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));
        body.Body.LocalCenterOfMassOffset.Should().Be(new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero));
    }

    [Fact]
    public void ConeBounds_WithCollapsedNonUnitBodyRotation_ShouldNormalizeBeforeBuildingShape()
    {
        Fixed64 component = FixedMath.Sqrt(Fixed64.Half);
        var collapsedRotation = new FixedQuaternion(
            component,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero);

        (component * component).Should().Be(Fixed64.Half);
        (collapsedRotation * Vector3d.Up).Should().Be(Vector3d.Zero);

        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var collider = new LSConeCollider
        {
            Radius = Fixed64.Half,
            Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One)
        };
        var agent = new TestMatterAgent(
            scenario.Context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One));
        var body = new SolidBody(agent, collider) { Mass = Fixed64.One };
        body.UseManualGrounding();
        body.Initialize(Vector3d.Zero, collapsedRotation);

        body.Rotation.Should().Be(collapsedRotation.Normalized);
        collider.WorldAxis.Should().Be(Vector3d.Down);
        collider.BoundsMin.Should().Be(new Vector3d(-Fixed64.Half, -Fixed64.One, -Fixed64.Half));
        collider.BoundsMax.Should().Be(new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.Half));
    }

    [Fact]
    public void ConeDefinition_ShouldRejectInvalidDimensions()
    {
        Action zeroRadius = () => ColliderShapeDefinition.Cone(Fixed64.Zero, Fixed64.One);
        Action zeroHeight = () => ColliderShapeDefinition.Cone(Fixed64.Half, Fixed64.Zero);

        zeroRadius.Should().Throw<ArgumentException>().WithParameterName("radius");
        zeroHeight.Should().Throw<ArgumentException>().WithParameterName("height");
    }

    [Fact]
    public void CapsuleDefinition_ShouldRejectHeightBelowExactDiameter()
    {
        Fixed64 oversizedRadius = Fixed64.FromRaw(
            (Fixed64.MaxValue.m_rawValue / 2L) + 1L);

        Action create = () =>
            ColliderShapeDefinition.Capsule(
                oversizedRadius,
                Fixed64.MaxValue);

        create.Should()
            .Throw<ArgumentException>()
            .WithParameterName("height");
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
    public void CylinderRuntimeShape_ShouldPreserveAFullHeightWithNoRepresentableHalf()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.One, Fixed64.FromRaw(1), Fixed64.One));
        var agent = new TestMatterAgent(scenario.Context, transform);
        var body = new SolidBody(agent, new LSCylinderCollider());

        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);

        var cylinder = (LSCylinderCollider)body.Collider;
        cylinder.Height.Should().Be(Fixed64.FromRaw(1));
        cylinder.Bounds.Min.Y.Should().Be(Fixed64.FromRaw(-1));
        cylinder.Bounds.Max.Y.Should().Be(Fixed64.FromRaw(1));
        scenario.Context.Physics.BodyCount.Should().Be(1);
        scenario.Context.Physics.ColliderCount.Should().Be(1);
    }

    [Fact]
    public void CylinderRuntimeShape_ShouldCommitAnOddRawFullHeightWithoutEndpointAuthority()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCylinderCollider> source = scenario.CreateCylinder(Vector3d.Zero);
        LSCylinderCollider collider = source.Collider;
        FixedTransform transform = source.Body.Agent.Transform;
        uint version = collider.RuntimeShapeVersion;

        transform.LocalScale = new Vector3d(Fixed64.One, Fixed64.FromRaw(1), Fixed64.One);
        collider.Simulate();

        collider.Height.Should().Be(Fixed64.FromRaw(1));
        collider.Bounds.Min.Y.Should().Be(Fixed64.FromRaw(-1));
        collider.Bounds.Max.Y.Should().Be(Fixed64.FromRaw(1));
        collider.RuntimeShapeVersion.Should().Be(version + 1);
        collider.IsPartitioned.Should().BeTrue();

        transform.LocalScale = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One);
        collider.Simulate();

        collider.RuntimeShapeVersion.Should().Be(version + 2);
        collider.Height.Should().Be((Fixed64)2);
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
    public void CuboidCanonicalHalfExtents_ShouldSurviveWhenDerivedPropertiesExceedTheScalarDomain()
    {
        Fixed64.TryMultiplyDivide(
            Fixed64.MaxValue,
            Fixed64.One,
            Fixed64.Two,
            out Fixed64 expectedHalfExtent).Should().BeTrue();
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var collider = new LSCuboidCollider
        {
            Size = new Vector3d(
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                Fixed64.MaxValue)
        };
        var ownerSnapshot = new ColliderShapeSnapshot(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One,
            Vector3d.One,
            Vector3d.Zero,
            Vector3d.One,
            Fixed64.Half);
        collider.PrepareCompoundPart(
            ownerSnapshot,
            FixedQuaternion.Identity,
            Vector3d.One,
            context);
        collider.PublishCompoundPart(
            FixedQuaternion.Identity,
            Vector3d.One,
            context);

        collider.OrientedBox.HalfExtents.Should().Be(
            new Vector3d(expectedHalfExtent, expectedHalfExtent, expectedHalfExtent));
        collider.Area.Should().Be(Fixed64.MaxValue);
        collider.CalculateMassPropertyWeight().Should().Be(Fixed64.MaxValue);
        collider.GetFrontalArea(Vector3d.Right).Should().Be(Fixed64.MaxValue);

        Fixed3x3 inertia = collider.CalculateInertiaTensor(
            Fixed64.One,
            Vector3d.Zero);
        inertia.M11.Should().Be(Fixed64.MaxValue);
        inertia.M22.Should().Be(Fixed64.MaxValue);
        inertia.M33.Should().Be(Fixed64.MaxValue);
    }

    [Fact]
    public void CapsuleNormal_ShouldUseDeterministicFallbacksAtCapCentersAndAxis()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateCapsule(Vector3d.Zero);

        FixedSegment.TryGetCenteredAxisEndpoint(
            capsule.Collider.Center,
            capsule.Collider.WorldAxis,
            capsule.Collider.AxisLength,
            positive: true,
            out Vector3d topCenter).Should().BeTrue();
        FixedSegment.TryGetCenteredAxisEndpoint(
            capsule.Collider.Center,
            capsule.Collider.WorldAxis,
            capsule.Collider.AxisLength,
            positive: false,
            out Vector3d bottomCenter).Should().BeTrue();

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

}
