using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using System.Reflection;
using Xunit;

namespace Gravitas.Tests;

public sealed class StaticSurfaceFilterContinuationTests
{
    [Theory]
    [InlineData("Matrix")]
    [InlineData("MoverMask")]
    [InlineData("MoverLayer")]
    [InlineData("Hierarchy")]
    [InlineData("NoOpMatrix")]
    [InlineData("UnusedMatrixLayer")]
    [InlineData("OtherMatrixDirection")]
    public void BorrowedOnlySupplierFilterChange_ShouldKeepTheRetainedCarrierCurrentWithoutAPoseChange(string change)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.ApplySettings(new PhysicsSettings(16, new[,]
        {
            { true, true, true }, { true, true, false }, { true, false, true }
        }) { RuntimeMode = PhysicsRuntimeMode.Mixed });
        scene.Context.Environment.Gravity = Fixed64.Zero;
        scene.Context.Environment.AirDensity = Fixed64.Zero;
        scene.Context.Environment.DampingFactor = Fixed64.Zero;
        scene.Context.Environment.MinSpeed = Fixed64.Zero;
        LSCollider2D left = Rectangle(-4), right = Rectangle(4), remote = Rectangle(24);
        right.Layer = new PhysicsLayer(1); right.Material = PhysicsMaterial.Bouncy;
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right, remote });
        var mover = new LSCuboidCollider { Size = new(10, 1, 1), Material = PhysicsMaterial.Frictionless };
        var body = scene.CreateBody(mover, new(-Fixed64.FromFraction(19, 5), Fixed64.Zero, -Fixed64.Half),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)30, Fixed64.Zero)).Body;
        // Retain the actual shallow overlap while leaving the exposed rim's X
        // response available. The unchanged contact must not move or wake it.
        body.FreezeAxes = BodyFreezeAxes3D.PositionY | BodyFreezeAxes3D.PositionZ | BodyFreezeAxes3D.Rotation;
        body.UseManualGrounding();
        var otherFloor = new LSAABBoxCollider2D(new Vector2d(2, 2)) { MixedHalfThicknessOverride = Fixed64.Two };
        otherFloor.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(
            new(12, 0, -1), FixedQuaternion.Identity, Vector3d.One)));
        var otherBody = scene.CreateBody(new LSSphereCollider { Radius = Fixed64.Half },
            new((Fixed64)12, Fixed64.Zero, Fixed64.Quarter), FixedQuaternion.Identity).Body;
        otherBody.FreezeAxes = BodyFreezeAxes3D.PositionZ | BodyFreezeAxes3D.Rotation;
        otherBody.UseManualGrounding();
        var remoteBody = scene.CreateBody(new LSSphereCollider { Radius = Fixed64.Half },
            new((Fixed64)24, Fixed64.Zero, Fixed64.Quarter), FixedQuaternion.Identity).Body;
        remoteBody.FreezeAxes = BodyFreezeAxes3D.PositionZ | BodyFreezeAxes3D.Rotation;
        remoteBody.UseManualGrounding();
        var ordinary = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, left, ordinary));
        Assert.False(CollisionDetectionMixed.TryBuildManifold(mover, right, ordinary));
        Step();
        var pairs = (SwiftDictionary<ulong, CollisionPairMixed>)typeof(GravitasMixedCollisionService)
            .GetField("_pairs", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(scene.Context.MixedCollisions)!;
        ulong key = MixedColliderKey.CreateKey(mover.Id, left.Id);
        Assert.True(pairs.TryGetValue(key, out CollisionPairMixed pair));
        Assert.False(pairs.ContainsKey(MixedColliderKey.CreateKey(mover.Id, right.Id)));
        Assert.NotEmpty(pair.Manifold);
        for (int group = 0; group < pair.Manifold.GroupCount; group++)
            Assert.Equal(right.Id + 1, pair.Manifold.GetGroup(group).Key.SourceIdB);
        foreach (ManifoldContact row in pair.Manifold)
        {
            Assert.True(row.HasMaterialOverride);
            Assert.Equal(right.Material, row.MaterialB);
        }
        body.Sleep(); otherBody.Sleep(); remoteBody.Sleep(); Step();
        Vector3d remotePosition = remoteBody.Position3d;
        Assert.True(otherBody.IsSleeping);
        Vector3d otherPosition = otherBody.Position3d;
        Assert.True(body.IsSleeping); Assert.True(pair.HasCurrentManifoldCache);
        Vector3d position = body.Position3d;
        FixedQuaternion rotation = body.Rotation;

        switch (change)
        {
            case "Matrix":
                scene.Context.Settings.CollisionMatrix[0, 1] = false;
                scene.Context.Settings.CollisionMatrix[1, 0] = false;
                break;
            case "MoverMask": mover.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(1); break;
            case "MoverLayer": mover.Layer = new PhysicsLayer(2); break;
            case "Hierarchy": right.SetParent(mover); break;
            case "NoOpMatrix": scene.Context.Settings.CollisionMatrix[0, 1] = true; break;
            case "UnusedMatrixLayer": scene.Context.Settings.CollisionMatrix[0, 2] = false; break;
            case "OtherMatrixDirection": scene.Context.Settings.CollisionMatrix[2, 0] = false; break;
        }
        bool changedContacts = change is "Matrix" or "MoverMask" or "MoverLayer" or "Hierarchy";
        Assert.Equal(position, body.Position3d); Assert.Equal(rotation, body.Rotation);
        Assert.True(scene.Context.MixedCollisions.RequireCollisionPair(mover, left));
        Assert.Equal(!changedContacts, scene.Context.MixedCollisions.RequireCollisionPair(mover, right));
        Step();

        Assert.True(otherBody.IsSleeping);
        Assert.Equal(otherPosition, otherBody.Position3d);
        Assert.True(remoteBody.IsSleeping);
        Assert.Equal(remotePosition, remoteBody.Position3d);
        Assert.True(pairs.TryGetValue(key, out CollisionPairMixed current));
        Assert.Same(pair, current); Assert.NotEmpty(current.Manifold);
        if (!changedContacts)
        {
            Assert.True(body.IsSleeping); Assert.Equal(position, body.Position3d);
            for (int group = 0; group < current.Manifold.GroupCount; group++)
                Assert.Equal(right.Id + 1, current.Manifold.GetGroup(group).Key.SourceIdB);
            return;
        }
        AssertEligibleRows(current.Manifold);
        Assert.False(body.IsSleeping);
        var fresh = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildDiscreteManifold(mover, left, fresh));
        Assert.NotEmpty(fresh); AssertEligibleRows(fresh);

        void AssertEligibleRows(ContactManifold contacts)
        {
            for (int group = 0; group < contacts.GroupCount; group++)
                Assert.NotEqual(right.Id + 1, contacts.GetGroup(group).Key.SourceIdB);
            foreach (ManifoldContact row in contacts)
            {
                Assert.Equal(row.HasMaterialOverride ? left.Material : PhysicsMaterial.Default, row.MaterialB);
                Assert.True(row.AnchorB.TryGetWorldPoint(out Vector3d point));
                Assert.True(left.ContainsPoint(new(point.X, point.Z)));
                Assert.InRange(point.Y.m_rawValue, (-(Fixed64)4).m_rawValue, ((Fixed64)4).m_rawValue);
            }
        }

        void Step() { scene.Context.Simulate(); scene.Context.LateSimulate(); }
        LSCollider2D Rectangle(int centerX)
        {
            var collider = new LSAABBoxCollider2D(new Vector2d(8, 2))
            { MixedHalfThicknessOverride = (Fixed64)4, Material = PhysicsMaterial.Frictionless };
            collider.InitializeWithNoBody(new TestMatterAgent(scene.Context,
                new FixedTransform(new(centerX, 0, -1), FixedQuaternion.Identity, Vector3d.One)));
            return collider;
        }
    }
}
