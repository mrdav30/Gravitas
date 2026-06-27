using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System;
using System.Linq;
using System.Reflection;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class ColliderShapeDefinition2DTests
{
    [Fact]
    public void AABBoxDefinition_ShouldBuildEquivalentStandaloneCollider()
    {
        Vector2d size = new((Fixed64)2, (Fixed64)4);
        ColliderShapeDefinition2D definition = ColliderShapeDefinition2D.AABBox(size);

        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D body = CreateBody(context, new LSAABBoxCollider2D(definition), Vector2d.Zero);

        definition.Kind.Should().Be(ColliderShapeDefinition2DKind.AABBox);
        definition.Size.Should().Be(size);
        body.Collider.Bounds.MinX.Should().Be(-Fixed64.One);
        body.Collider.Bounds.MaxX.Should().Be(Fixed64.One);
        body.Collider.Bounds.MinY.Should().Be((Fixed64)(-2));
        body.Collider.Bounds.MaxY.Should().Be((Fixed64)2);
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
    public void ShapeDefinition2D_ShouldNotExposeRuntimeLifecycleState()
    {
        Type definitionType = typeof(ColliderShapeDefinition2D);
        string[] publicMemberNames = definitionType
            .GetMembers(BindingFlags.Instance | BindingFlags.Public)
            .Select(member => member.Name)
            .Distinct()
            .ToArray();

        publicMemberNames.Should().NotContain(new[]
        {
            nameof(LSCollider2D.Id),
            nameof(LSCollider2D.Body),
            nameof(LSCollider2D.Context),
            nameof(LSCollider2D.PartitionCoordinates),
            nameof(LSCollider2D.OnContact),
            nameof(LSCollider2D.OnTriggerEnter)
        });

        definitionType.GetFields(BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public)
            .Select(field => field.FieldType)
            .Should()
            .NotContain(type => typeof(LSCollider2D).IsAssignableFrom(type));
    }

    private static SolidBody2D CreateBody(GravitasWorldContext context, LSCollider2D collider, Vector2d position)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One,
            FreezeAxes = BodyFreezeAxes2D.Position
        };
        body.Initialize(position);
        return body;
    }
}
