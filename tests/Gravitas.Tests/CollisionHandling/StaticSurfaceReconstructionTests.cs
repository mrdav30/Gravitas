using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using SwiftCollections;
using System.Collections.Generic;
using System.Reflection;
using Xunit;

namespace Gravitas.Tests;

public sealed class StaticSurfaceReconstructionTests
{
    [Theory]
    [InlineData(PhysicsRuntimeMode.ThreeD)]
    [InlineData(PhysicsRuntimeMode.TwoD)]
    [InlineData(PhysicsRuntimeMode.Mixed)]
    public void RecycledRegistrySlots_ShouldPreserveMultipleSupplierOrderAndResponse(PhysicsRuntimeMode mode)
    {
        var original = RunMultiple(mode, false);
        var reconstructed = RunMultiple(mode, true);
        Assert.Equal(original.Suppliers, reconstructed.Suppliers);
        Assert.Equal(original.Velocity, reconstructed.Velocity);
        Assert.Equal(original.AngularVelocity, reconstructed.AngularVelocity);
        Assert.Equal(original.Position, reconstructed.Position);
    }

    [Fact]
    public void RecycledRegistrySlots_ShouldPreserveTheActualSharedSupplierAndResponse()
    {
        var original = Run(false);
        var reconstructed = Run(true);
        Assert.Equal(original.Supplier, reconstructed.Supplier);
        Assert.Equal(original.Velocity, reconstructed.Velocity);
        Assert.Equal(original.Position, reconstructed.Position);

        static (PhysicsMaterial Supplier, Vector3d Velocity, Vector3d Position) Run(bool recycle)
        {
            using var scene = PhysicsScenarioBuilder.Create();
            scene.Context.SetFrameRate(16);
            scene.Context.Environment.Gravity = scene.Context.Environment.AirDensity =
                scene.Context.Environment.DampingFactor = scene.Context.Environment.MinSpeed = Fixed64.Zero;
            if (recycle)
            {
                var first = new LSSphereCollider(); var second = new LSSphereCollider();
                scene.InitializeStaticCollider(first, new(12, 0, 12));
                scene.InitializeStaticCollider(second, new(13, 0, 12));
                first.Deactivate(); second.Deactivate();
            }
            LSMeshCollider left = Floor(-8, 0), right = Floor(0, 8);
            left.Material = PhysicsMaterial.Frictionless; right.Material = PhysicsMaterial.Bouncy;
            if (recycle) Assert.True(left.Id > right.Id); else Assert.True(left.Id < right.Id);
            using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
            var sphere = new LSSphereCollider { Radius = Fixed64.Half, Material = PhysicsMaterial.Frictionless };
            SolidBody body = scene.CreateBody(sphere, Vector3d.Up * Fixed64.Quarter, FixedQuaternion.Identity).Body;
            body.FreezeAxes = BodyFreezeAxes3D.Rotation; body.SleepEnabled = false; body.UseManualGrounding();
            body.AddLinearImpulse(Vector3d.Down);
            scene.Context.Simulate(); scene.Context.LateSimulate();
            var suppliers = new List<PhysicsMaterial>();
            foreach (LSMeshCollider member in new[] { left, right })
            {
                if (!member.TryGetCollisionPair(sphere.Id, out CollisionPair? pair)
                    && !sphere.TryGetCollisionPair(member.Id, out pair)) continue;
                foreach (ManifoldContact row in pair!.Manifold)
                    suppliers.Add(pair.ColliderA == member ? row.MaterialA : row.MaterialB);
            }
            return (Assert.Single(suppliers), body.LinearVelocity, body.Position3d);

            LSMeshCollider Floor(int from, int to)
            {
                var floor = new LSMeshCollider(new[] { new Vector3d(from, 0, -2), new(from, 0, 2),
                    new(to, 0, -2), new(to, 0, 2) }, new[] { 0, 1, 2, 2, 1, 3 },
                    MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
                scene.InitializeStaticCollider(floor, Vector3d.Zero);
                return floor;
            }
        }
    }

    private static (PhysicsMaterial[] Suppliers, Vector3d Velocity, Vector3d AngularVelocity, Vector3d Position)
        RunMultiple(PhysicsRuntimeMode mode, bool recycle)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = mode; scene.Context.SetFrameRate(16);
        scene.Context.Environment.Gravity = scene.Context.Environment.AirDensity =
            scene.Context.Environment.DampingFactor = scene.Context.Environment.MinSpeed = Fixed64.Zero;
        if (recycle)
        {
            if (mode == PhysicsRuntimeMode.ThreeD)
            {
                var first = new LSSphereCollider(); var second = new LSSphereCollider();
                scene.InitializeStaticCollider(first, new(12, 0, 12));
                scene.InitializeStaticCollider(second, new(13, 0, 12));
                first.Deactivate(); second.Deactivate();
            }
            else
            {
                var first = new LSCircleCollider2D(Fixed64.Half); var second = new LSCircleCollider2D(Fixed64.Half);
                first.InitializeWithNoBody(new TestMatterAgent(scene.Context,
                    new FixedTransform(new(12, 0, 12), FixedQuaternion.Identity, Vector3d.One)));
                second.InitializeWithNoBody(new TestMatterAgent(scene.Context,
                    new FixedTransform(new(13, 0, 12), FixedQuaternion.Identity, Vector3d.One)));
                first.Deactivate(); second.Deactivate();
            }
        }
        var suppliers = new List<PhysicsMaterial>();
        int contactCarriers = 0;
        if (mode == PhysicsRuntimeMode.ThreeD)
        {
            LSMeshCollider left = Floor(-4), right = Floor(4);
            Assert.Equal(recycle, left.Id > right.Id);
            left.Material = PhysicsMaterial.Frictionless; right.Material = PhysicsMaterial.Bouncy;
            using var surface = scene.Context.Physics.CreateStaticSurface(new[] { right, left });
            var mover = new LSCuboidCollider { Size = Vector3d.One, Material = PhysicsMaterial.Bouncy };
            var body = scene.CreateBody(mover, Vector3d.Up * Fixed64.Quarter, FixedQuaternion.Identity).Body;
            body.SleepEnabled = false; body.UseManualGrounding();
            body.AddLinearImpulse(new(Fixed64.Quarter, -Fixed64.One, Fixed64.Zero)); Step();
            foreach (LSMeshCollider member in new[] { left, right })
            {
                if (!member.TryGetCollisionPair(mover.Id, out CollisionPair? pair)
                    && !mover.TryGetCollisionPair(member.Id, out pair)) continue;
                if (pair!.Manifold.HasContact) contactCarriers++;
                foreach (ManifoldContact row in pair!.Manifold)
                    suppliers.Add(pair.ColliderA == member ? row.MaterialA : row.MaterialB);
            }
            return Finish(body);
        }
        LSCollider2D planarLeft = Rectangle(-4), planarRight = Rectangle(4);
        Assert.Equal(recycle, planarLeft.Id > planarRight.Id);
        planarLeft.Material = PhysicsMaterial.Frictionless; planarRight.Material = PhysicsMaterial.Bouncy;
        using var planarSurface = scene.Context.Physics2D.CreateStaticSurface(new[] { planarRight, planarLeft });
        if (mode == PhysicsRuntimeMode.Mixed)
        {
            var mover = new LSCuboidCollider { Size = Vector3d.One, Material = PhysicsMaterial.Bouncy };
            var body = scene.CreateBody(mover, Vector3d.Forward * Fixed64.Quarter, FixedQuaternion.Identity).Body;
            body.SleepEnabled = false; body.UseManualGrounding();
            body.AddLinearImpulse(new(Fixed64.Quarter, Fixed64.Zero, -Fixed64.One)); Step();
            var pairs = (SwiftDictionary<ulong, CollisionPairMixed>)typeof(GravitasMixedCollisionService)
                .GetField("_pairs", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(scene.Context.MixedCollisions)!;
            foreach (LSCollider2D member in new[] { planarLeft, planarRight })
                if (pairs.TryGetValue(MixedColliderKey.CreateKey(mover.Id, member.Id), out CollisionPairMixed pair))
                {
                    if (pair.Manifold.HasContact) contactCarriers++;
                    foreach (ManifoldContact row in pair.Manifold) suppliers.Add(row.MaterialB);
                }
            return Finish(body);
        }
        Fixed64 h = Fixed64.Half;
        var polygon = new LSPolygonCollider2D(new(-h, -h), new(h, -h), new(h, h), new(-h, h))
        { Material = PhysicsMaterial.Bouncy };
        var planarBody = new SolidBody2D(new TestMatterAgent(scene.Context), polygon) { Mass = Fixed64.One, SleepEnabled = false };
        planarBody.Initialize(new(Fixed64.Zero, Fixed64.Quarter)); planarBody.UseManualGrounding();
        planarBody.AddLinearImpulse(new(Fixed64.Quarter, -Fixed64.One)); Step();
        foreach (LSCollider2D member in new[] { planarLeft, planarRight })
        {
            if (!member.TryGetCollisionPair(polygon.Id, out CollisionPair2D? pair)
                && !polygon.TryGetCollisionPair(member.Id, out pair)) continue;
            if (pair!.Manifold.HasContact) contactCarriers++;
            foreach (ManifoldContact2D row in pair!.Manifold)
                suppliers.Add(pair.ColliderA == member ? row.MaterialA : row.MaterialB);
        }
        Assert.Equal(1, contactCarriers);
        Assert.Contains(PhysicsMaterial.Frictionless, suppliers); Assert.Contains(PhysicsMaterial.Bouncy, suppliers);
        for (int frame = 0; frame < 3; frame++) Step();
        return (suppliers.ToArray(), new(planarBody.LinearVelocity.X, Fixed64.Zero, planarBody.LinearVelocity.Y),
            Vector3d.Up * planarBody.AngularVelocity, new(planarBody.Position.X, Fixed64.Zero, planarBody.Position.Y));

        (PhysicsMaterial[], Vector3d, Vector3d, Vector3d) Finish(SolidBody body)
        {
            // This is solver row order inside one shared carrier, not pair-table enumeration.
            Assert.Equal(1, contactCarriers);
            Assert.Contains(PhysicsMaterial.Frictionless, suppliers); Assert.Contains(PhysicsMaterial.Bouncy, suppliers);
            for (int frame = 0; frame < 3; frame++) Step();
            return (suppliers.ToArray(), body.LinearVelocity, body.AngularVelocity, body.Position3d);
        }
        void Step() { scene.Context.Simulate(); scene.Context.LateSimulate(); }
        LSMeshCollider Floor(int center)
        {
            var floor = MeshTestFixtures.CreateConvexQuadFloor(MeshColliderMode.Concave);
            floor.InitializeWithNoBody(new TestMatterAgent(scene.Context,
                new FixedTransform(new(center, 0, 0), FixedQuaternion.Identity, new(2, 1, 1))));
            return floor;
        }
        LSCollider2D Rectangle(int center)
        {
            var rectangle = new LSAABBoxCollider2D(new Vector2d(8, 2)) { MixedHalfThicknessOverride = Fixed64.Two };
            rectangle.InitializeWithNoBody(new TestMatterAgent(scene.Context,
                new FixedTransform(new(center, 0, -1), FixedQuaternion.Identity, Vector3d.One)));
            return rectangle;
        }
    }
}
