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
            new[] { 0, 1, 2, 1, 3, 2 });
        ScenarioBody<LSMeshCollider> body = scenario.CreateBody(
            meshCollider,
            Vector3d.Zero,
            FixedQuaternion.Identity);

        body.Collider.Bounds.Proportions.x.Should().Be((Fixed64)4);
        body.Collider.Bounds.Proportions.y.Should().Be((Fixed64)2);

        body.Body.SetRotation(FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90));
        body.Collider.Simulate();

        AssertNear(body.Collider.Bounds.Proportions.x, (Fixed64)2);
        AssertNear(body.Collider.Bounds.Proportions.y, (Fixed64)4);
    }

    private static Vector3d[] ValidVertices() =>
        new[]
        {
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Up
        };

    private static int[] ValidTriangles() => new[] { 0, 1, 2 };

    private static void AssertNear(Fixed64 actual, Fixed64 expected) =>
        (actual - expected).Abs().Should().BeLessThan(Fixed64.Fraction(1, 1000));
}
