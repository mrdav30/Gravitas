using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System;
using System.Linq;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class ColliderShapeDefinition2DTests
{
    public static TheoryData<ColliderShapeDefinition2D, Type> RuntimeColliderDefinitions => new()
    {
        { ColliderShapeDefinition2D.Circle(Fixed64.Half), typeof(LSCircleCollider2D) },
        { ColliderShapeDefinition2D.Capsule(Fixed64.Half, (Fixed64)3), typeof(LSCapsuleCollider2D) },
        { ColliderShapeDefinition2D.AABBox(Vector2d.One), typeof(LSAABBoxCollider2D) },
        { ColliderShapeDefinition2D.ConvexPolygon(
            new Vector2d(-Fixed64.One, -Fixed64.One),
            new Vector2d(Fixed64.One, -Fixed64.One),
            new Vector2d(Fixed64.One, Fixed64.One),
            new Vector2d(-Fixed64.One, Fixed64.One)),
            typeof(LSPolygonCollider2D) }
    };

    [Theory]
    [MemberData(nameof(RuntimeColliderDefinitions))]
    public void ShapeDefinition2D_ShouldCreateEverySupportedRuntimeColliderKind(
        ColliderShapeDefinition2D definition,
        Type expectedType)
    {
        LSCollider2D collider = definition.CreateCollider();

        collider.Should().BeOfType(expectedType);
    }

    [Fact]
    public void AABBoxDefinition_ShouldBuildEquivalentStandaloneCollider()
    {
        Vector2d size = new((Fixed64)2, (Fixed64)4);
        ColliderShapeDefinition2D definition = ColliderShapeDefinition2D.AABBox(size);

        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D body = CreateBody(context, new LSAABBoxCollider2D(definition), Vector2d.Zero);

        definition.Kind.Should().Be(ColliderShapeDefinition2DKind.AABBox);
        definition.Size.Should().Be(size);
        body.Collider.Bounds.Min.X.Should().Be(-Fixed64.One);
        body.Collider.Bounds.Max.X.Should().Be(Fixed64.One);
        body.Collider.Bounds.Min.Y.Should().Be((Fixed64)(-2));
        body.Collider.Bounds.Max.Y.Should().Be((Fixed64)2);
    }

    [Fact]
    public void ConvexPolygonDefinition_ShouldSnapshotSourceVertices()
    {
        Vector2d[] vertices =
        {
            new(-Fixed64.One, -Fixed64.One),
            new(Fixed64.One, -Fixed64.One),
            new(Fixed64.One, Fixed64.One),
            new(-Fixed64.One, Fixed64.One)
        };
        Vector2d originalVertex = vertices[0];

        ColliderShapeDefinition2D definition = ColliderShapeDefinition2D.ConvexPolygon(vertices);
        vertices[0] = new Vector2d((Fixed64)99, (Fixed64)99);

        definition.Kind.Should().Be(ColliderShapeDefinition2DKind.ConvexPolygon);
        definition.PolygonVertexCount.Should().Be(4);
        definition.GetPolygonVertex(0).Should().Be(originalVertex);
    }

    [Fact]
    public void CapsuleDefinition_ShouldBuildEquivalentStandaloneCollider()
    {
        ColliderShapeDefinition2D definition = ColliderShapeDefinition2D.Capsule(Fixed64.Half, (Fixed64)3);

        LSCollider2D collider = definition.CreateCollider();

        definition.Kind.Should().Be(ColliderShapeDefinition2DKind.Capsule);
        definition.Radius.Should().Be(Fixed64.Half);
        definition.Size.Should().Be(new Vector2d(Fixed64.One, (Fixed64)3));
        collider.Should().BeOfType<LSCapsuleCollider2D>();
        var capsule = (LSCapsuleCollider2D)collider;
        capsule.Radius.Should().Be(Fixed64.Half);
        capsule.Height.Should().Be((Fixed64)3);
    }

    [Theory]
    [InlineData(0, 1)]
    [InlineData(-1, 1)]
    [InlineData(1, 1)]
    public void CapsuleDefinition_ShouldRejectInvalidDimensions(int radiusValue, int heightValue)
    {
        Action act = () => ColliderShapeDefinition2D.Capsule((Fixed64)radiusValue, (Fixed64)heightValue);

        act.Should().Throw<ArgumentException>();
    }

    [Fact]
    public void CapsuleDefinition_ShouldIncludeRadiusHeightAndMaterialInEquality()
    {
        ColliderShapeDefinition2D first = ColliderShapeDefinition2D.Capsule(Fixed64.Half, (Fixed64)3);
        ColliderShapeDefinition2D same = ColliderShapeDefinition2D.Capsule(Fixed64.Half, (Fixed64)3);
        ColliderShapeDefinition2D differentHeight = ColliderShapeDefinition2D.Capsule(Fixed64.Half, (Fixed64)4);

        first.Should().Be(same);
        first.GetHashCode().Should().Be(same.GetHashCode());
        first.Should().NotBe(differentHeight);
    }

    [Fact]
    public void ShapeDefinition2DAccessors_ShouldRejectUndefinedWrongShapeFamiliesAndInvalidAabbSize()
    {
        ColliderShapeDefinition2D undefined = default;
        ColliderShapeDefinition2D circle = ColliderShapeDefinition2D.Circle(Fixed64.One);

        Action createUndefined = () => undefined.CreateCollider();
        Action readUndefinedPolygonVertex = () => _ = undefined.GetPolygonVertex(0);
        Action readCirclePolygonVertex = () => _ = circle.GetPolygonVertex(0);
        Action createInvalidAabbX = () => ColliderShapeDefinition2D.AABBox(new Vector2d(Fixed64.Zero, Fixed64.One));
        Action createInvalidAabbY = () => ColliderShapeDefinition2D.AABBox(new Vector2d(Fixed64.One, Fixed64.Zero));
        Action buildCircleFromBoxDefinition = () => _ = new LSCircleCollider2D(ColliderShapeDefinition2D.AABBox(Vector2d.One));

        createUndefined.Should().Throw<ArgumentException>().WithParameterName(nameof(ColliderShapeDefinition2D));
        readUndefinedPolygonVertex.Should().Throw<ArgumentException>().WithParameterName(nameof(ColliderShapeDefinition2D));
        readCirclePolygonVertex.Should().Throw<ArgumentException>().WithParameterName(nameof(ColliderShapeDefinition2D));
        createInvalidAabbX.Should().Throw<ArgumentException>().WithParameterName("size");
        createInvalidAabbY.Should().Throw<ArgumentException>().WithParameterName("size");
        buildCircleFromBoxDefinition.Should().Throw<ArgumentException>().WithParameterName(nameof(ColliderShapeDefinition2D));
    }

    [Fact]
    public void ShapeDefinition2DEqualityAndHash_ShouldEncodeAuthoredShapeMaterialAndPolygonPayload()
    {
        PhysicsMaterial material = PhysicsMaterialTestHelper.WithFrictionAndRestitution(
            Fixed64.FromFraction(3, 4),
            Fixed64.FromFraction(1, 4));
        PhysicsMaterial otherMaterial = PhysicsMaterialTestHelper.WithFrictionAndRestitution(
            Fixed64.Half,
            Fixed64.FromFraction(1, 8));
        Vector2d[] vertices =
        {
            new(-Fixed64.One, -Fixed64.One),
            new(Fixed64.One, -Fixed64.One),
            new(Fixed64.One, Fixed64.One),
            new(-Fixed64.One, Fixed64.One)
        };
        ColliderShapeDefinition2D definition = ColliderShapeDefinition2D.ConvexPolygon(material, vertices);
        ColliderShapeDefinition2D same = ColliderShapeDefinition2D.ConvexPolygon(material, vertices.ToArray());

        definition.Should().Be(same);
        definition.Equals((object)same).Should().BeTrue();
        definition.GetHashCode().Should().Be(same.GetHashCode());
        (definition == same).Should().BeTrue();
        (definition != same).Should().BeFalse();

        definition.Equals("polygon").Should().BeFalse();
        definition.Should().NotBe(ColliderShapeDefinition2D.Circle(Fixed64.One, material));
        definition.Should().NotBe(ColliderShapeDefinition2D.ConvexPolygon(vertices));
        definition.Should().NotBe(ColliderShapeDefinition2D.ConvexPolygon(otherMaterial, vertices));
        definition.Should().NotBe(ColliderShapeDefinition2D.ConvexPolygon(
            material,
            vertices[1],
            vertices[2],
            vertices[3],
            vertices[0]));
        definition.Should().NotBe(ColliderShapeDefinition2D.Triangle(vertices[0], vertices[1], vertices[2], material));
        ColliderShapeDefinition2D.Capsule(Fixed64.Half, (Fixed64)3)
            .Should()
            .NotBe(ColliderShapeDefinition2D.Capsule(Fixed64.Half, (Fixed64)4));
        ColliderShapeDefinition2D.AABBox(Vector2d.One)
            .Should()
            .NotBe(ColliderShapeDefinition2D.AABBox(new Vector2d(Fixed64.One, (Fixed64)2)));
    }

    [Fact]
    public void ShapeDefinition2DEqualityAndHash_ShouldHandleDefaultMaterialAndObjectNull()
    {
        ColliderShapeDefinition2D circle = ColliderShapeDefinition2D.Circle(Fixed64.Half);
        ColliderShapeDefinition2D same = ColliderShapeDefinition2D.Circle(Fixed64.Half);
        ColliderShapeDefinition2D differentRadius = ColliderShapeDefinition2D.Circle(Fixed64.One);
        ColliderShapeDefinition2D capsule = ColliderShapeDefinition2D.Capsule(Fixed64.Half, (Fixed64)3);

        circle.Should().Be(same);
        circle.GetHashCode().Should().Be(same.GetHashCode());
        circle.Equals((object?)null).Should().BeFalse();
        circle.Should().NotBe(differentRadius);
        circle.Should().NotBe(capsule);
    }

    [Fact]
    public void TriangleDefinition_ShouldMaterializeAsConvexPolygon()
    {
        ColliderShapeDefinition2D definition = ColliderShapeDefinition2D.Triangle(
            new Vector2d(Fixed64.Zero, Fixed64.Zero),
            new Vector2d(Fixed64.One, Fixed64.Zero),
            new Vector2d(Fixed64.Zero, Fixed64.One));

        LSCollider2D collider = definition.CreateCollider();

        definition.Kind.Should().Be(ColliderShapeDefinition2DKind.ConvexPolygon);
        definition.PolygonVertexCount.Should().Be(3);
        collider.Should().BeOfType<LSPolygonCollider2D>();
    }

    [Fact]
    public void ShapeDefinitions2D_ShouldMaterializeEverySupportedRuntimeCollider()
    {
        ColliderShapeDefinition2D.Circle(Fixed64.Half).CreateCollider().Should().BeOfType<LSCircleCollider2D>();
        ColliderShapeDefinition2D.AABBox(Vector2d.One).CreateCollider().Should().BeOfType<LSAABBoxCollider2D>();
        ColliderShapeDefinition2D.Capsule(Fixed64.Half, (Fixed64)2).CreateCollider().Should().BeOfType<LSCapsuleCollider2D>();
        ColliderShapeDefinition2D.Triangle(Vector2d.Zero, Vector2d.Right, Vector2d.Forward)
            .CreateCollider()
            .Should()
            .BeOfType<LSPolygonCollider2D>();
    }

    [Fact]
    public void TriangleDefinition_ShouldUseConvexPolygonValidation()
    {
        Action act = () => ColliderShapeDefinition2D.Triangle(
            Vector2d.Zero,
            Vector2d.Right,
            new Vector2d((Fixed64)2, Fixed64.Zero));

        act.Should().Throw<ArgumentException>();
    }

    private static SolidBody2D CreateBody(GravitasWorldContext context, LSCollider2D collider, Vector2d position)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(position, motionType: BodyMotionType.Static);
        return body;
    }
}
