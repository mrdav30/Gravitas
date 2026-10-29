using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Support;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests;

public sealed class StaticSurfaceFilterParityTests
{
    [Theory]
    [InlineData("Exclude")]
    [InlineData("NoOp")]
    [InlineData("UnusedLayer")]
    public void BorrowedMeshSupplierMatrixChange_ShouldWakeOnlyChangedSleepingContacts(string change)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, PhysicsRuntimeMode.ThreeD);
        scene.Context.ApplySettings(new PhysicsSettings(16, new[,]
        {
            { true, true, true }, { true, true, true }, { true, true, true }
        }) { RuntimeMode = PhysicsRuntimeMode.ThreeD });
        LSMeshCollider left = Wall(-16, 0), right = Wall(0, 16);
        right.Layer = new PhysicsLayer(1); right.Material = PhysicsMaterial.Bouncy;
        var cube = MeshTestFixtures.CreateConvexCube();
        Vector3d[] vertices = cube.Mesh.LocalVertices.ToArray();
        for (int vertex = 0; vertex < vertices.Length; vertex++) vertices[vertex].X *= (Fixed64)10;
        var mover = new LSMeshCollider(vertices, cube.Mesh.Triangles.ToArray(), MeshColliderMode.Convex,
            MeshInertiaPolicy.RequireClosedVolume) { Material = PhysicsMaterial.Frictionless };
        // A sheet has two exits. This side makes the nearer full-hull support
        // lie on the right sheet even though its plane crossing is on the left.
        SolidBody body = scene.CreateBody(mover, new(-Fixed64.FromFraction(19, 5), Fixed64.Zero, Fixed64.Half),
            PhysicsScenarioBuilder.Yaw(30)).Body;
        body.FreezeAxes = BodyFreezeAxes3D.PositionY | BodyFreezeAxes3D.PositionZ | BodyFreezeAxes3D.Rotation;
        body.UseManualGrounding();
        Assert.False(CollisionDetection.DoCollisionCheck(scene.CreatePair(mover, right)));
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
        int supplierEnters = 0;
        right.OnContactEnter += _ => supplierEnters++;
        Step(scene.Context);
        Assert.True(TryPair(mover, left, out CollisionPair? pair));
        // 3D retains broad-phase noncontact pairs; they must not be mistaken
        // for a supplier overlap whose later exit could wake the body.
        Assert.Equal(0, supplierEnters);
        if (TryPair(mover, right, out CollisionPair? separated)) Assert.Empty(separated!.Manifold);
        Assert.NotEmpty(pair!.Manifold);
        bool first = pair.ColliderA == left;
        for (int group = 0; group < pair.Manifold.GroupCount; group++)
            Assert.Equal(right.Id + 1, Source(pair.Manifold.GetGroup(group).Key, first));
        body.Sleep(); Step(scene.Context); Assert.True(body.IsSleeping);
        Vector3d position = body.Position3d;

        if (change == "Exclude")
        {
            scene.Context.Settings.CollisionMatrix[0, 1] = false;
            scene.Context.Settings.CollisionMatrix[1, 0] = false;
        }
        else if (change == "UnusedLayer") scene.Context.Settings.CollisionMatrix[2, 0] = false;
        else scene.Context.Settings.CollisionMatrix[1, 0] = true;
        Assert.Equal(position, body.Position3d);
        Step(scene.Context);

        if (change != "Exclude")
        {
            Assert.True(body.IsSleeping); Assert.Equal(position, body.Position3d);
            for (int group = 0; group < pair.Manifold.GroupCount; group++)
                Assert.Equal(right.Id + 1, Source(pair.Manifold.GetGroup(group).Key, first));
            return;
        }
        Assert.False(body.IsSleeping);
        Assert.NotEmpty(pair.Manifold);
        for (int group = 0; group < pair.Manifold.GroupCount; group++)
            Assert.NotEqual(right.Id + 1, Source(pair.Manifold.GetGroup(group).Key, first));
        foreach (ManifoldContact row in pair.Manifold)
            Assert.Equal(left.Material, first ? row.MaterialA : row.MaterialB);

        LSMeshCollider Wall(int from, int to)
        {
            var member = new LSMeshCollider(new[] { new Vector3d(from, -4, 0), new(to, -4, 0),
                new(from, 4, 0), new(to, 4, 0) }, new[] { 0, 2, 1, 1, 2, 3 },
                MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation) { Material = PhysicsMaterial.Frictionless };
            scene.InitializeStaticCollider(member, Vector3d.Zero);
            return member;
        }
    }

    [Theory]
    [InlineData("Exclude")]
    [InlineData("NoOp")]
    [InlineData("UnusedLayer")]
    public void BorrowedPlanarSupplierMatrixChange_ShouldWakeOnlyChangedSleepingContacts(string change)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, PhysicsRuntimeMode.TwoD);
        scene.Context.ApplySettings(new PhysicsSettings(16, new[,]
        {
            { true, true, true }, { true, true, true }, { true, true, true }
        }) { RuntimeMode = PhysicsRuntimeMode.TwoD });
        LSCollider2D left = Rectangle(-8), right = Rectangle(8);
        right.Layer = new PhysicsLayer(1); right.Material = PhysicsMaterial.Bouncy;
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right });
        var mover = new LSPolygonCollider2D(new(-(Fixed64)5, -Fixed64.Half), new((Fixed64)5, -Fixed64.Half),
            new((Fixed64)5, Fixed64.Half), new(-(Fixed64)5, Fixed64.Half)) { Material = PhysicsMaterial.Frictionless };
        var body = new SolidBody2D(new TestMatterAgent(scene.Context), mover)
        { Mass = Fixed64.One, FreezeAxes = BodyFreezeAxes2D.PositionY | BodyFreezeAxes2D.Rotation };
        body.Initialize(new(-Fixed64.FromFraction(19, 5), -Fixed64.Half), -FixedMath.DegToRad((Fixed64)30));
        body.UseManualGrounding();
        Assert.False(CollisionDetection2D.TryCollide(mover, right, out _));
        Step(scene.Context);
        Assert.True(TryPair(mover, left, out CollisionPair2D? pair));
        Assert.False(TryPair(mover, right, out _));
        Assert.NotEmpty(pair!.Manifold);
        bool first = pair.ColliderA == left;
        // Native filled-2D contacts retain both the original finite crossing
        // and its connected borrowed support, unlike mixed solid exit ranking.
        bool own = false, borrowed = false;
        for (int group = 0; group < pair.Manifold.GroupCount; group++)
        {
            ref ContactGroup2D contacts = ref pair.Manifold.GetGroup(group);
            int supplier = Source(contacts.Key, first);
            Assert.True(supplier == left.Id + 1 || supplier == right.Id + 1);
            own |= supplier == left.Id + 1; borrowed |= supplier == right.Id + 1;
            for (int point = 0; point < contacts.Count; point++)
                Assert.Equal(supplier == left.Id + 1 ? left.Material : right.Material,
                    first ? contacts[point].MaterialA : contacts[point].MaterialB);
        }
        Assert.True(own); Assert.True(borrowed);
        body.Sleep(); Step(scene.Context); Assert.True(body.IsSleeping);
        Vector2d position = body.Position;

        if (change == "Exclude")
        {
            scene.Context.Settings.CollisionMatrix[0, 1] = false;
            scene.Context.Settings.CollisionMatrix[1, 0] = false;
        }
        else if (change == "UnusedLayer") scene.Context.Settings.CollisionMatrix[0, 2] = false;
        else scene.Context.Settings.CollisionMatrix[0, 1] = true;
        Assert.Equal(position, body.Position);
        Step(scene.Context);

        if (change != "Exclude")
        {
            Assert.True(body.IsSleeping); Assert.Equal(position, body.Position);
            bool retainedBorrow = false;
            for (int group = 0; group < pair.Manifold.GroupCount; group++)
                retainedBorrow |= Source(pair.Manifold.GetGroup(group).Key, first) == right.Id + 1;
            Assert.True(retainedBorrow);
            return;
        }
        Assert.False(body.IsSleeping);
        Assert.NotEmpty(pair.Manifold);
        for (int group = 0; group < pair.Manifold.GroupCount; group++)
            Assert.NotEqual(right.Id + 1, Source(pair.Manifold.GetGroup(group).Key, first));
        foreach (ManifoldContact2D row in pair.Manifold)
            Assert.Equal(left.Material, first ? row.MaterialA : row.MaterialB);

        LSCollider2D Rectangle(int center)
        {
            var member = new LSAABBoxCollider2D(new Vector2d(16, 2)) { Material = PhysicsMaterial.Frictionless };
            member.InitializeWithNoBody(new TestMatterAgent(scene.Context,
                new FixedTransform(new(center, 0, -1), FixedQuaternion.Identity, Vector3d.One)));
            return member;
        }
    }

    private static int Source(in ContactGroupKey group, bool first) => first ? group.SourceIdA : group.SourceIdB;

    private static bool TryPair(LSCollider a, LSCollider b, out CollisionPair? pair) =>
        a.TryGetCollisionPair(b.Id, out pair) || b.TryGetCollisionPair(a.Id, out pair);

    private static bool TryPair(LSCollider2D a, LSCollider2D b, out CollisionPair2D? pair) =>
        a.TryGetCollisionPair(b.Id, out pair) || b.TryGetCollisionPair(a.Id, out pair);

    private static void Configure(GravitasWorldContext context, PhysicsRuntimeMode mode)
    {
        context.ApplySettings(new PhysicsSettings(16, new[,] { { true, true }, { true, true } }) { RuntimeMode = mode });
        context.Environment.Gravity = context.Environment.AirDensity = context.Environment.DampingFactor = context.Environment.MinSpeed = Fixed64.Zero;
    }

    private static void Step(GravitasWorldContext context) { context.Simulate(); context.LateSimulate(); }
}
