using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests;

public sealed class StaticSurfaceFiniteBoundaryContinuationTests
{
    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void RotatingCapsule_ShouldHonorALaterWallAfterResolvingADeclaredSpatialFloor(bool separateSpatialSurface)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        scene.Context.SetFrameRate(16);
        scene.Context.Environment.MaxSpeed = scene.Context.Environment.MaxFallSpeed = (Fixed64)1024;
        scene.Context.Environment.Gravity = scene.Context.Environment.AirDensity =
            scene.Context.Environment.DampingFactor = scene.Context.Environment.MinSpeed = Fixed64.Zero;
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { FloorSheet(scene, -8, 0), FloorSheet(scene, 0, 8) });
        using var wallSurface = separateSpatialSurface ? scene.Context.Physics.CreateStaticSurface(new[] { SpatialWall() }) : null;
        if (!separateSpatialSurface)
        {
            var wall = new LSAABBoxCollider2D(new Vector2d(Fixed64.Half, (Fixed64)4)) { Material = PhysicsMaterial.Frictionless };
            var wallBody = new SolidBody2D(new TestMatterAgent(scene.Context), wall);
            wallBody.Initialize(new(Fixed64.One, Fixed64.One), Fixed64.Zero, BodyMotionType.Static);
        }
        var capsule = new LSCapsuleCollider2D(Fixed64.Half, Fixed64.Two)
        { Material = PhysicsMaterial.Frictionless, MixedHalfThicknessOverride = Fixed64.Half };
        var body = new SolidBody2D(new TestMatterAgent(scene.Context), capsule)
        { Mass = Fixed64.One, SleepEnabled = false, ContinuousCollisionMode = ContinuousCollisionMode.Continuous };
        body.Initialize(new(-Fixed64.Quarter, Fixed64.One)); body.UseManualGrounding();
        body.AddLinearImpulse(new((Fixed64)8, -Fixed64.FromFraction(1, 16)));
        body.AddAngularImpulse(Fixed64.FromFraction(1, 64));
        scene.Context.Simulate(); scene.Context.LateSimulate();
        Assert.True(body.Position.X < Fixed64.Half, $"The source crossed the later wall: {body.Position}.");
        Assert.True(body.Position.Y > Fixed64.Quarter);
        Assert.True(body.LinearVelocity.X < (Fixed64)8, "The later wall must supply an actual response.");

        LSMeshCollider SpatialWall()
        {
            Fixed64 x = Fixed64.FromFraction(3, 4);
            var wall = new LSMeshCollider(new[] { new Vector3d(x, -(Fixed64)2, -Fixed64.One), new(x, (Fixed64)2, -Fixed64.One),
                new(x, -(Fixed64)2, (Fixed64)3), new(x, (Fixed64)2, (Fixed64)3) }, new[] { 0, 1, 2, 2, 1, 3 },
                MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation) { Material = PhysicsMaterial.Frictionless };
            scene.InitializeStaticCollider(wall, Vector3d.Zero);
            return wall;
        }
    }

    [Theory]
    [InlineData(false, false)] [InlineData(false, true)] [InlineData(true, false)]
    public void RotatingCompound_ShouldKeepEarlierLeafContactsWhenTheFinalLeafChangesItsLocalSupplierGraph(bool finalLeafMisses, bool nearby)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        scene.Context.SetFrameRate(16);
        scene.Context.Environment.MaxSpeed = scene.Context.Environment.MaxFallSpeed = (Fixed64)1024;
        scene.Context.Environment.Gravity = scene.Context.Environment.AirDensity =
            scene.Context.Environment.DampingFactor = scene.Context.Environment.MinSpeed = Fixed64.Zero;
        var left = FloorSheet(scene, nearby ? -2 : -4, nearby ? 0 : -2);
        var right = FloorSheet(scene, nearby ? 1 : 2, nearby ? 3 : 4);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
        int leftEntered = 0, rightEntered = 0;
        left.OnMixedContactEnter += _ => leftEntered++;
        right.OnMixedContactEnter += _ => rightEntered++;
        var first = CompoundColliderPart2D.Capsule(Fixed64.Half, Fixed64.Two, new(nearby ? -1 : -3, 0));
        var second = CompoundColliderPart2D.Capsule(Fixed64.Half, Fixed64.Two, new(nearby ? 2 : 3, 0));
        var compound = finalLeafMisses
            ? new LSCompoundCollider2D(first, second, CompoundColliderPart2D.Capsule(Fixed64.Half, Fixed64.Two, new(9, 0)))
            : new LSCompoundCollider2D(first, second);
        compound.Material = PhysicsMaterial.Frictionless;
        compound.MixedHalfThicknessOverride = Fixed64.Half;
        var body = new SolidBody2D(new TestMatterAgent(scene.Context), compound)
        { Mass = Fixed64.One, SleepEnabled = false, ContinuousCollisionMode = ContinuousCollisionMode.Continuous };
        body.Initialize(new(Fixed64.Zero, Fixed64.FromFraction(3, 4))); body.UseManualGrounding();
        body.AddLinearImpulse(new(Fixed64.Quarter, -Fixed64.One));
        body.AddAngularImpulse(Fixed64.FromFraction(1, 16));
        scene.Context.Simulate(); scene.Context.LateSimulate();
        Assert.Equal(1, leftEntered); Assert.Equal(1, rightEntered);
        Assert.True(body.LastContinuousCollisionToiIterationCount > 0);
        Assert.True(body.LinearVelocity.Y > -Fixed64.Half, "Both earlier physical leaves must retain support response.");
        Assert.True(body.Position.Y > Fixed64.Half);
    }

    [Fact]
    public void FastRotatingCapsule_ShouldKeepSupportAsNewChunksEnterTheLocalFamilyGraph()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        scene.Context.SetFrameRate(16);
        scene.Context.Environment.MaxSpeed = scene.Context.Environment.MaxFallSpeed = (Fixed64)1024;
        scene.Context.Environment.Gravity = scene.Context.Environment.AirDensity =
            scene.Context.Environment.DampingFactor = scene.Context.Environment.MinSpeed = Fixed64.Zero;
        var chunks = new LSMeshCollider[12];
        for (int index = 0; index < chunks.Length; index++) chunks[index] = FloorSheet(scene, -8 + 2 * index, -6 + 2 * index);
        using var surface = scene.Context.Physics.CreateStaticSurface(chunks);
        var capsule = new LSCapsuleCollider2D(Fixed64.Half, Fixed64.Two)
        { Material = PhysicsMaterial.Frictionless, MixedHalfThicknessOverride = Fixed64.Half };
        var body = new SolidBody2D(new TestMatterAgent(scene.Context), capsule)
        { Mass = Fixed64.One, SleepEnabled = false, ContinuousCollisionMode = ContinuousCollisionMode.Continuous };
        body.Initialize(new(-Fixed64.Quarter, Fixed64.One)); body.UseManualGrounding();
        body.AddLinearImpulse(new((Fixed64)128, -Fixed64.FromFraction(1, 16)));
        body.AddAngularImpulse(Fixed64.FromFraction(1, 64));
        scene.Context.Simulate(); scene.Context.LateSimulate();
        Assert.True(body.Position.X > Fixed64.Zero, $"The source never crossed its initial chunk: {body.Position}.");
        Assert.True(body.Position.Y > Fixed64.Quarter);
        Assert.Equal((Fixed64)128, body.LinearVelocity.X);
        Assert.True(body.LastContinuousCollisionToiIterationCount > 0);
    }

    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void RotatingPlanarFootprint_ShouldNotIgnoreTheFarExposedEdgeAcrossAFiniteHole(bool polygon)
    {
        var ordinary = Run(joined: false, polygon);
        var joined = Run(joined: true, polygon);
        Assert.True(ordinary.Position.X < Fixed64.One,
            $"The independent-edge control did not encounter the corner: {ordinary}.");
        Assert.True(joined.Position.X < Fixed64.One,
            $"Declared continuity skipped an exposed edge across the hole: ordinary={ordinary}; joined={joined}.");
        if (polygon)
        {
            Assert.True(joined.Limit, "An unproved finite prism cut must keep its conservative stationary tail.");
            Assert.Equal((Fixed64)64, joined.Velocity.X);
        }

        static (Vector2d Position, Vector2d Velocity, bool Limit) Run(bool joined, bool polygon)
        {
            using var scene = PhysicsScenarioBuilder.Create();
            scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
            scene.Context.SetFrameRate(16);
            scene.Context.Environment.MaxSpeed = scene.Context.Environment.MaxFallSpeed = (Fixed64)1024;
            scene.Context.Environment.Gravity = scene.Context.Environment.AirDensity =
                scene.Context.Environment.DampingFactor = scene.Context.Environment.MinSpeed = Fixed64.Zero;
            var left = Sheet(-(Fixed64)8, Fixed64.Half, -(Fixed64)2, (Fixed64)2);
            var right = Sheet(Fixed64.One, Fixed64.Two, -(Fixed64)2, (Fixed64)2);
            // This bridge is coplanar and joins both pieces globally. It lies
            // outside the moving finite slab, so the gap retains its true rim.
            var bridge = Sheet(Fixed64.Half, Fixed64.One, Fixed64.One, Fixed64.Two);
            using var surface = joined ? scene.Context.Physics.CreateStaticSurface(new[] { left, right, bridge }) : null;
            LSCollider2D footprint = polygon
                ? new LSPolygonCollider2D(new Vector2d(Fixed64.Zero, -Fixed64.One),
                    new(Fixed64.Half, Fixed64.One), new(-Fixed64.Half, Fixed64.One))
                : new LSCapsuleCollider2D(Fixed64.Half, Fixed64.Two);
            footprint.Material = PhysicsMaterial.Frictionless;
            footprint.MixedHalfThicknessOverride = Fixed64.Half;
            var body = new SolidBody2D(new TestMatterAgent(scene.Context), footprint)
            { Mass = Fixed64.One, SleepEnabled = false, ContinuousCollisionMode = ContinuousCollisionMode.Continuous };
            body.Initialize(new(Fixed64.Zero, Fixed64.FromFraction(99, 100)));
            body.UseManualGrounding();
            body.AddLinearImpulse(new((Fixed64)64, -Fixed64.One));
            body.AddAngularImpulse(Fixed64.FromFraction(1, 64));
            scene.Context.Simulate(); scene.Context.LateSimulate();
            return (body.Position, body.LinearVelocity, body.LastContinuousCollisionToiIterationLimitReached);

            LSMeshCollider Sheet(Fixed64 minX, Fixed64 maxX, Fixed64 minY, Fixed64 maxY)
            {
                var mesh = new LSMeshCollider(new[] { new Vector3d(minX, minY, Fixed64.Zero), new(minX, maxY, Fixed64.Zero),
                    new(maxX, minY, Fixed64.Zero), new(maxX, maxY, Fixed64.Zero) },
                    new[] { 0, 1, 2, 2, 1, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
                { Material = PhysicsMaterial.Frictionless };
                scene.InitializeStaticCollider(mesh, Vector3d.Zero);
                return mesh;
            }
        }
    }

    private static LSMeshCollider FloorSheet(PhysicsScenarioBuilder scene, int minX, int maxX)
    {
        var mesh = new LSMeshCollider(new[] { new Vector3d(minX, -2, 0), new(minX, 2, 0),
            new(maxX, -2, 0), new(maxX, 2, 0) }, new[] { 0, 1, 2, 2, 1, 3 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation) { Material = PhysicsMaterial.Frictionless };
        scene.InitializeStaticCollider(mesh, Vector3d.Zero);
        return mesh;
    }
}
