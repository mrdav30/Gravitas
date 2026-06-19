using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using SwiftCollections;
using SwiftCollections.Query;
using System;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class MeshTriangleQueryAllocationTests
{
    [Fact]
    public void GetTrianglesInBounds_AfterWarmup_ShouldNotAllocate()
    {
        LSMeshCollider collider = MeshTestFixtures.CreateUChannel();
        var results = new SwiftList<int>(8);
        var bounds = new FixedBoundVolume(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)2, (Fixed64)2, (Fixed64)4));

        collider.GetTrianglesInBounds(bounds, results);

        long allocatedBytes = MeasureAllocatedBytes(() =>
        {
            collider.GetTrianglesInBounds(bounds, results);
        });

        allocatedBytes.Should().Be(0);
        results.Count.Should().BeGreaterThan(0);
    }

    [Fact]
    public void MoveMeshRuntimeShapeStateAndQueryTriangles_AfterWarmup_ShouldNotAllocate()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider collider = MeshTestFixtures.CreateConvexCube();
        ScenarioBody<LSMeshCollider> body = scenario.CreateBody(
            collider,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            preventAngularForces: true);
        var results = new SwiftList<int>(8);
        var queryBounds = new FixedBoundVolume(Vector3d.Zero, Vector3d.One);

        body.Body.SetPosition(Vector3d.Zero);
        collider.Simulate();
        collider.GetTrianglesInBounds(queryBounds, results);
        body.Body.SetPosition(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        collider.Simulate();
        collider.GetTrianglesInBounds(
            new FixedBoundVolume(
                new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
                new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.One, Fixed64.One)),
            results);
        body.Body.SetPosition(Vector3d.Zero);
        collider.Simulate();
        collider.GetTrianglesInBounds(queryBounds, results);

        Vector3d movedPosition = new(Fixed64.Half, Fixed64.Zero, Fixed64.Zero);
        FixedBoundVolume movedBounds = new(
            new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.One, Fixed64.One));

        long setPositionBytes = MeasureAllocatedBytes(() => body.Body.SetPosition(movedPosition));
        long simulateBytes = MeasureAllocatedBytes(collider.Simulate);
        long queryBytes = MeasureAllocatedBytes(() => collider.GetTrianglesInBounds(movedBounds, results));
        long allocatedBytes = setPositionBytes + simulateBytes + queryBytes;

        allocatedBytes.Should().Be(
            0,
            "steady-state mesh movement and triangle query should reuse bounds, partition, and BVH query storage (set={0}, simulate={1}, query={2})",
            setPositionBytes,
            simulateBytes,
            queryBytes);
        results.Count.Should().BeGreaterThan(0);
    }

    private static long MeasureAllocatedBytes(Action action)
        => AllocationTestHelper.MeasureSinglePass(action);
}
