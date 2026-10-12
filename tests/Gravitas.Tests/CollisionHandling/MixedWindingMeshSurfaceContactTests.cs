//=======================================================================
// MixedWindingMeshSurfaceContactTests.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class MixedWindingMeshSurfaceContactTests
{
    [Theory]
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(2, false)]
    [InlineData(2, true)]
    public void SelectivelyReversedInteriorFaces_ShouldPreserveTheCoherentSurfaceManifold(int shape, bool subdivide)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider coherent = CreateFloor(false, subdivide), mixed = CreateFloor(true, subdivide);
        scenario.InitializeStaticCollider(coherent, Vector3d.Zero);
        scenario.InitializeStaticCollider(mixed, Vector3d.Zero);
        LSCollider curved = CreateShape(shape);
        // Even the cone's small plane section intersects the internal seam.
        scenario.InitializeStaticCollider(curved,
            new(Fixed64.FromFraction(1, 16), -Fixed64.Quarter, Fixed64.FromFraction(1, 16)));
        CollisionPair baseline = scenario.CreatePair(coherent, curved), changed = scenario.CreatePair(mixed, curved);
        Assert.True(CollisionDetection.DoCollisionCheck(baseline));
        Assert.True(CollisionDetection.DoCollisionCheck(changed));

        AssertSameManifold(baseline, changed);
        Assert.Equal(1, changed.Manifold.GroupCount);
        foreach (ManifoldContact contact in changed.Manifold)
            Assert.Equal(Vector3d.Down, contact.Normal);
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(2, false)]
    [InlineData(2, true)]
    public void SelectivelyReversedFaces_ShouldPreserveTheGenuinePerimeterConstraint(int shape, bool subdivide)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider coherent = CreateFloor(false, subdivide), mixed = CreateFloor(true, subdivide);
        scenario.InitializeStaticCollider(coherent, Vector3d.Zero);
        scenario.InitializeStaticCollider(mixed, Vector3d.Zero);
        LSCollider curved = CreateShape(shape);
        scenario.InitializeStaticCollider(curved,
            new(Fixed64.Two + Fixed64.FromFraction(1, 16), -Fixed64.Quarter, Fixed64.Zero));
        CollisionPair baseline = scenario.CreatePair(coherent, curved), changed = scenario.CreatePair(mixed, curved);
        Assert.True(CollisionDetection.DoCollisionCheck(baseline));
        Assert.True(CollisionDetection.DoCollisionCheck(changed));

        AssertSameManifold(baseline, changed);
        bool right = false;
        foreach (ManifoldContact contact in changed.Manifold)
            right |= contact.Normal.X > Fixed64.Zero;
        Assert.True(right, "Removing an internal winding seam must preserve the actual right perimeter.");
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(2, false)]
    [InlineData(2, true)]
    public void SelectivelyReversedFacesFullLoop_ShouldPreserveFrictionlessTangentialMotion(int shape, bool subdivide)
    {
        using PhysicsScenarioBuilder baseline = PhysicsScenarioBuilder.Create();
        using PhysicsScenarioBuilder changed = PhysicsScenarioBuilder.Create();
        SolidBody first = CreateMovingBody(baseline, shape, false, subdivide);
        SolidBody second = CreateMovingBody(changed, shape, true, subdivide);
        baseline.Context.Simulate();
        baseline.Context.LateSimulate();
        changed.Context.Simulate();
        changed.Context.LateSimulate();

        Assert.Equal(first.Position3d, second.Position3d);
        Assert.Equal(first.LinearVelocity, second.LinearVelocity);
        Assert.Equal(-Fixed64.Half, second.LinearVelocity.X);
        Assert.Equal(-Fixed64.Half, second.LinearVelocity.Z);
        Assert.True(second.LinearVelocity.Y <= Fixed64.Zero);
        Assert.Equal(Vector3d.Zero, second.AngularVelocity);
    }

    private static void AssertSameManifold(CollisionPair baseline, CollisionPair changed)
    {
        Assert.Equal(baseline.Manifold.GroupCount, changed.Manifold.GroupCount);
        Assert.Equal(baseline.Manifold.Count, changed.Manifold.Count);
        for (int group = 0; group < baseline.Manifold.GroupCount; group++)
            Assert.Equal(baseline.Manifold.GetGroup(group).Key, changed.Manifold.GetGroup(group).Key);
        for (int point = 0; point < baseline.Manifold.Count; point++)
        {
            ManifoldContact first = baseline.Manifold[point], second = changed.Manifold[point];
            Assert.Equal(first.PointA, second.PointA);
            Assert.Equal(first.PointB, second.PointB);
            Assert.Equal(first.AnchorA, second.AnchorA);
            Assert.Equal(first.AnchorB, second.AnchorB);
            Assert.Equal(first.Normal, second.Normal);
            Assert.Equal(first.Depth, second.Depth);
            Assert.Equal(first.ContactId, second.ContactId);
        }
    }

    private static SolidBody CreateMovingBody(PhysicsScenarioBuilder scenario, int shape, bool mixed, bool subdivide)
    {
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        LSMeshCollider floor = CreateFloor(mixed, subdivide);
        floor.Material = PhysicsMaterial.Frictionless;
        scenario.CreateBody(floor, Vector3d.Zero, FixedQuaternion.Identity, immovable: true);
        LSCollider curved = CreateShape(shape);
        curved.Material = PhysicsMaterial.Frictionless;
        SolidBody body = scenario.CreateBody(curved,
            new(Fixed64.FromFraction(1, 16), -Fixed64.Quarter, Fixed64.FromFraction(1, 16)),
            FixedQuaternion.Identity, preventAngularForces: true).Body;
        body.UseManualGrounding();
        body.SleepEnabled = false;
        body.AddLinearImpulse(new(-Fixed64.Half, Fixed64.Half, -Fixed64.Half));
        return body;
    }

    private static LSCollider CreateShape(int shape) => shape switch
    {
        0 => new LSSphereCollider { Radius = Fixed64.Half },
        1 => new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) },
        _ => new LSConeCollider { Radius = Fixed64.Half, Size = Vector3d.One }
    };

    private static LSMeshCollider CreateFloor(bool mixed, bool subdivide)
    {
        Vector3d[] vertices = { new(-2, 0, -2), new(2, 0, -2), new(-2, 0, 2), new(2, 0, 2), Vector3d.Zero };
        if (!subdivide) System.Array.Resize(ref vertices, 4);
        int[] triangles = subdivide ? new[] { 0, 2, 4, 2, 3, 4, 3, 1, 4, 1, 0, 4 }
            : new[] { 0, 2, 1, 1, 2, 3 };
        if (mixed)
            for (int triangle = 1; triangle < triangles.Length / 3; triangle += 2)
            {
                int first = triangle * 3;
                (triangles[first], triangles[first + 2]) = (triangles[first + 2], triangles[first]);
            }
        return new(vertices, triangles, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
    }
}
