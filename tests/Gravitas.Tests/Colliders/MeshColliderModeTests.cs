using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using SwiftCollections;
using SwiftCollections.Query;
using System;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class MeshColliderModeTests
{
    [Fact]
    public void ConcaveMesh_ShouldInitializeForBodylessImmovableKinematicAndDynamicBodies()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();

        Action bodyless = () => scenario.InitializeStaticCollider(
            MeshTestFixtures.CreateInsideCorner(),
            PhysicsScenarioBuilder.Vector(0, 0, 0));
        Action immovable = () => scenario.CreateBody(
            MeshTestFixtures.CreateInsideCorner(),
            PhysicsScenarioBuilder.Vector(6, 0, 0),
            FixedQuaternion.Identity,
            immovable: true);
        Action kinematic = () => scenario.CreateBody(
            MeshTestFixtures.CreateInsideCorner(),
            PhysicsScenarioBuilder.Vector(12, 0, 0),
            FixedQuaternion.Identity,
            isKinematic: true);
        Action dynamic = () => scenario.CreateBody(
            MeshTestFixtures.CreateInsideCorner(),
            PhysicsScenarioBuilder.Vector(18, 0, 0),
            FixedQuaternion.Identity);

        bodyless.Should().NotThrow();
        immovable.Should().NotThrow();
        kinematic.Should().NotThrow();
        dynamic.Should().NotThrow();
    }

    [Fact]
    public void DynamicConcaveMesh_ShouldMoveThroughLocalBvhWithoutRebuildingTopology()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> body = scenario.CreateBody(
            MeshTestFixtures.CreateUChannel(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity);
        var indices = new SwiftList<int>();

        body.Collider.GetTrianglesInBounds(
            new FixedBoundVolume(
                new Vector3d(Fixed64.Fraction(3, 2), Fixed64.Zero, Fixed64.One),
                new Vector3d((Fixed64)3, (Fixed64)2, (Fixed64)3)),
            indices);
        int buildCount = body.Collider.Mesh.TriangleBvhBuildCount;

        body.Body.SetPosition(new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero));
        body.Collider.Simulate();
        body.Collider.GetTrianglesInBounds(
            new FixedBoundVolume(
                new Vector3d((Fixed64)5 + Fixed64.Fraction(3, 2), Fixed64.Zero, Fixed64.One),
                new Vector3d((Fixed64)8, (Fixed64)2, (Fixed64)3)),
            indices);

        indices.Count.Should().BeGreaterThan(0);
        body.Collider.Mesh.TriangleBvhBuildCount.Should().Be(buildCount);
    }
}
