using Chronicler.Hashing;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests;

public sealed class StaticSurfaceDependencyRefreshTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void UnusedMatrixEdits_ShouldHashPendingRefreshButKeepEquivalentContactsSleeping(bool planar)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, planar);
        if (planar)
        {
            LSCollider2D member = Rectangle(scene.Context, new(4, 2), new(0, -1));
            using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { member });
            var mover = new LSCircleCollider2D(Fixed64.Half);
            SolidBody2D body = PlanarBody(scene.Context, mover, new(Fixed64.Zero, Fixed64.Quarter));
            var pair = new CollisionPair2D(member, mover);
            Assert.True(CollisionDetection2D.TryCollide(pair, pair.Manifold, 0));
            pair.MarkCollidingDeferred(0);
            body.Sleep();
            ChronicleHash initial = Hash(pair);
            for (int edit = 0; edit < 2; edit++)
            {
                scene.Context.Settings.CollisionMatrix[0, 1] = edit != 0;
                Assert.NotEqual(initial, Hash(pair));
                Assert.True(CollisionDetection2D.TryCollide(pair, pair.Manifold, 0));
                Assert.False(pair.HasRecovery);
                Assert.True(body.IsSleeping);
                Assert.Equal(initial, Hash(pair));
            }
        }
        else
        {
            LSMeshCollider member = MeshTestFixtures.CreateConvexQuadFloor(MeshColliderMode.Concave);
            scene.InitializeStaticCollider(member, Vector3d.Zero);
            using var surface = scene.Context.Physics.CreateStaticSurface(new[] { member });
            var mover = scene.CreateSphere(new(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero));
            mover.Body.UseManualGrounding();
            CollisionPair pair = scene.CreatePair(member, mover.Collider);
            pair.UpdateCollisionDeferred();
            Assert.NotEmpty(pair.Manifold);
            mover.Body.Sleep();
            ChronicleHash initial = Hash(pair);
            for (int edit = 0; edit < 2; edit++)
            {
                scene.Context.Settings.CollisionMatrix[1, 0] = edit != 0;
                Assert.NotEqual(initial, Hash(pair));
                Assert.True(CollisionDetection.DoCollisionCheck(pair));
                Assert.True(mover.Body.IsSleeping);
                Assert.Equal(initial, Hash(pair));
            }
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ExistingOrdinaryContact_ShouldAdoptANewSurfaceWithoutAnOldMatrixSnapshot(bool planar)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, planar);
        if (planar)
        {
            LSCollider2D member = Rectangle(scene.Context, new(4, 2), new(0, -1));
            var mover = new LSCircleCollider2D(Fixed64.Half);
            SolidBody2D body = PlanarBody(scene.Context, mover, new(Fixed64.Zero, Fixed64.Quarter));
            var pair = new CollisionPair2D(member, mover);
            Assert.True(CollisionDetection2D.TryCollide(pair, pair.Manifold, 0));
            pair.MarkCollidingDeferred(0);
            using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { member });
            var previous = pair.BeginCollisionLayerUpdate(pair.Manifold);
            Assert.True(previous.Layers.HasValue);
            Assert.Null(previous.Before);
            Assert.True(CollisionDetection2D.TryCollide(pair, pair.Manifold, 0));
            Assert.NotEmpty(pair.Manifold);
            Assert.False(pair.HasRecovery);
            Assert.Equal(new Vector2d(Fixed64.Zero, Fixed64.Quarter), body.Position);
        }
        else
        {
            LSMeshCollider member = MeshTestFixtures.CreateConvexQuadFloor(MeshColliderMode.Concave);
            scene.InitializeStaticCollider(member, Vector3d.Zero);
            var mover = scene.CreateSphere(new(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero));
            CollisionPair pair = scene.CreatePair(member, mover.Collider);
            pair.UpdateCollisionDeferred();
            Assert.NotEmpty(pair.Manifold);
            using var surface = scene.Context.Physics.CreateStaticSurface(new[] { member });
            var previous = pair.BeginCollisionLayerUpdate();
            Assert.True(previous.Layers.HasValue);
            Assert.Null(previous.Before);
            Assert.True(CollisionDetection.DoCollisionCheck(pair));
            Assert.NotEmpty(pair.Manifold);
            Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero), mover.Body.Position3d);
        }
    }

    [Fact]
    public void CulledNoncontact_ShouldHashThePendingSurfaceMatrixEpoch()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, false);
        LSMeshCollider member = MeshTestFixtures.CreateConvexQuadFloor(MeshColliderMode.Concave);
        scene.InitializeStaticCollider(member, Vector3d.Zero);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { member });
        var mover = scene.CreateSphere(new(12, 12, 12));
        CollisionPair pair = scene.CreatePair(member, mover.Collider);
        // The explicit narrow phase also records the dependency of a negative result.
        Assert.False(CollisionDetection.DoCollisionCheck(pair));
        pair.UpdateCollisionDeferred();
        Assert.Empty(pair.Manifold);
        Assert.True(pair.CullCounter > 0);
        ChronicleHash initial = Hash(pair);
        scene.Context.Settings.CollisionMatrix[1, 0] = false;
        Assert.NotEqual(initial, Hash(pair));
        scene.Context.Settings.CollisionMatrix[1, 0] = true;
        Assert.Equal(initial, Hash(pair));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void NoncontactSnapshotHistory_ShouldNotChangeAuthoritativePairHash(bool planar)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, planar);
        if (planar)
        {
            LSCollider2D member = Rectangle(scene.Context, new(4, 2), new(0, -1));
            var mover = new LSCircleCollider2D(Fixed64.Half);
            PlanarBody(scene.Context, mover, new(10, 10));
            var pair = new CollisionPair2D(member, mover);
            Assert.False(CollisionDetection2D.TryCollide(pair, pair.Manifold, 0));
            ChronicleHash untracked = Hash(pair);
            using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { member });
            Assert.False(CollisionDetection2D.TryCollide(pair, pair.Manifold, 0));
            Assert.Equal(untracked, Hash(pair));
            scene.Context.Settings.CollisionMatrix[0, 1] = false;
            Assert.Equal(untracked, Hash(pair));
        }
        else
        {
            LSMeshCollider member = MeshTestFixtures.CreateConvexQuadFloor(MeshColliderMode.Concave);
            scene.InitializeStaticCollider(member, Vector3d.Zero);
            var mover = scene.CreateSphere(new(10, 10, 10));
            CollisionPair pair = scene.CreatePair(member, mover.Collider);
            Assert.False(CollisionDetection.DoCollisionCheck(pair));
            ChronicleHash untracked = Hash(pair);
            using var surface = scene.Context.Physics.CreateStaticSurface(new[] { member });
            Assert.False(CollisionDetection.DoCollisionCheck(pair));
            Assert.Equal(untracked, Hash(pair));
            scene.Context.Settings.CollisionMatrix[1, 0] = false;
            Assert.Equal(untracked, Hash(pair));
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void SupplierInvalidation_ShouldRefreshRowsButNotWakeUnstartedOrTriggerPairs(bool planar, bool trigger)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, planar);
        var suppliers = new SwiftHashSet<int>();
        if (planar)
        {
            LSCollider2D member = Rectangle(scene.Context, new(4, 2), new(0, -1));
            member.IsTrigger = trigger;
            using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { member });
            var mover = new LSCircleCollider2D(Fixed64.Half);
            SolidBody2D body = PlanarBody(scene.Context, mover, new(Fixed64.Zero, Fixed64.Quarter));
            var pair = new CollisionPair2D(member, mover);
            Assert.True(CollisionDetection2D.TryCollide(pair, pair.Manifold, 0));
            if (trigger) pair.MarkCollidingDeferred(0);
            Assert.NotEmpty(pair.Manifold);
            body.Sleep();
            suppliers.Add(member.Id + 1);
            uint version = mover.ContactDependencyVersion;
            ChronicleHash before = Hash(pair);
            pair.InvalidateSurfaceContact(member, suppliers);
            Assert.True(pair.ContactDependenciesChanged);
            Assert.Equal(version, mover.ContactDependencyVersion);
            Assert.True(body.IsSleeping);
            // An unstarted pair rebuilds before response, so its old row cache is irrelevant.
            if (trigger) Assert.NotEqual(before, Hash(pair));
            else
            {
                Assert.Equal(before, Hash(pair));
                pair.MarkCollidingDeferred(0);
                ChronicleHash retained = Hash(pair);
                body.Sleep();
                pair.InvalidateSurfaceContact(member, suppliers);
                Assert.False(body.IsSleeping);
                Assert.NotEqual(retained, Hash(pair));
                Assert.Equal(version, mover.ContactDependencyVersion);
            }
        }
        else
        {
            LSMeshCollider member = MeshTestFixtures.CreateConvexQuadFloor(MeshColliderMode.Concave);
            scene.InitializeStaticCollider(member, Vector3d.Zero);
            member.IsTrigger = trigger;
            using var surface = scene.Context.Physics.CreateStaticSurface(new[] { member });
            var mover = scene.CreateBody(new LSSphereCollider(),
                new(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity);
            CollisionPair pair = scene.CreatePair(member, mover.Collider);
            Assert.True(CollisionDetection.DoCollisionCheck(pair));
            if (trigger) pair.UpdateCollisionDeferred();
            Assert.NotEmpty(pair.Manifold);
            mover.Body.Sleep();
            suppliers.Add(member.Id + 1);
            uint version = mover.Collider.ContactDependencyVersion;
            ChronicleHash before = Hash(pair);
            pair.InvalidateSurfaceContact(member, suppliers);
            Assert.Equal(version, mover.Collider.ContactDependencyVersion);
            Assert.True(mover.Body.IsSleeping);
            if (trigger) Assert.NotEqual(before, Hash(pair));
            else
            {
                Assert.Equal(before, Hash(pair));
                pair.UpdateCollisionDeferred();
                ChronicleHash retained = Hash(pair);
                mover.Body.Sleep();
                pair.InvalidateSurfaceContact(member, suppliers);
                Assert.False(mover.Body.IsSleeping);
                Assert.NotEqual(retained, Hash(pair));
                Assert.Equal(version, mover.Collider.ContactDependencyVersion);
            }
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PendingPlanarRecovery_ShouldSurviveEquivalentMatrixChangesAndWakeWhenItsTargetChanges(bool removeRecovery)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, true);
        LSCollider2D member = Rectangle(scene.Context, new(8, 8), Vector2d.Zero);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { member });
        var mover = new LSCircleCollider2D(Fixed64.Half);
        SolidBody2D body = PlanarBody(scene.Context, mover, Vector2d.Zero);
        var pair = new CollisionPair2D(member, mover);
        Assert.True(CollisionDetection2D.TryCollide(pair, pair.Manifold, 0));
        Assert.True(pair.HasRecovery);
        pair.MarkCollidingDeferred(0);
        body.Sleep();
        scene.Context.Settings.CollisionMatrix[0, 1] = false;
        Assert.True(CollisionDetection2D.TryCollide(pair, pair.Manifold, 0));
        Assert.True(pair.HasRecovery);
        Assert.True(body.IsSleeping);

        // Exercise the actual prepare/reduce/publish boundary: the rows are empty
        // for deep recovery, so the target itself must participate in comparison.
        scene.Context.Settings.CollisionMatrix[0, 1] = true;
        var previous = pair.BeginCollisionLayerUpdate(pair.Manifold);
        Assert.NotNull(previous.Before);
        Assert.True(previous.Recovery.HasValue);
        if (removeRecovery) pair.ClearRecovery();
        else pair.SetRecovery(mover, previous.Recovery.Value + Vector2d.Right);
        pair.CompleteCollisionLayerUpdate(pair.Manifold, previous);
        Assert.False(body.IsSleeping);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SameRootReparentingAndRepeatedLostParentCleanup_ShouldNotRepublishContactDependencies(bool planarChild)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Both;
        LSCollider root = scene.CreateStaticSphere(new(10, 0, 10));
        LSCollider2D middle = Rectangle(scene.Context, Vector2d.One, new(12, 12));
        middle.SetParent(root);
        if (planarChild)
        {
            var child = new LSCircleCollider2D(Fixed64.Half);
            SolidBody2D body = PlanarBody(scene.Context, child, Vector2d.Zero);
            child.SetParent(root); body.Sleep();
            uint version = child.ContactDependencyVersion;
            child.SetParent(root); child.SetParent(middle);
            Assert.Equal(version, child.ContactDependencyVersion);
            Assert.True(body.IsSleeping);
            ((IColliderHierarchyNode)child).ClearParentReference();
            Assert.False(body.IsSleeping);
            Assert.NotEqual(version, child.ContactDependencyVersion);
            body.Sleep(); version = child.ContactDependencyVersion;
            // The old root's cleanup can repeat a lost-parent notification safely.
            root.Deactivate();
            Assert.Equal(version, child.ContactDependencyVersion);
            Assert.True(body.IsSleeping);
            Assert.False(child.ParentKey.IsValid);
        }
        else
        {
            var child = scene.CreateSphere(Vector3d.Zero);
            child.Collider.SetParent(middle); child.Body.Sleep();
            uint version = child.Collider.ContactDependencyVersion;
            child.Collider.SetParent(middle); child.Collider.SetParent(root);
            Assert.Equal(version, child.Collider.ContactDependencyVersion);
            Assert.True(child.Body.IsSleeping);
            ((IColliderHierarchyNode)child.Collider).ClearParentReference();
            Assert.False(child.Body.IsSleeping);
            Assert.NotEqual(version, child.Collider.ContactDependencyVersion);
            child.Body.Sleep(); version = child.Collider.ContactDependencyVersion;
            root.Deactivate();
            Assert.Equal(version, child.Collider.ContactDependencyVersion);
            Assert.True(child.Body.IsSleeping);
            Assert.False(child.Collider.ParentKey.IsValid);
        }
    }

    [Fact]
    public void BodylessChild_ShouldRefreshItsContactDependenciesWhenItsParentIsRemoved()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        LSCollider parent = scene.CreateStaticSphere(new(10, 0, 10));
        LSCollider child = scene.CreateStaticSphere(Vector3d.Zero);
        child.SetParent(parent);
        uint version = child.ContactDependencyVersion;
        parent.Deactivate();
        Assert.False(child.ParentKey.IsValid);
        Assert.NotEqual(version, child.ContactDependencyVersion);
        Assert.True(child.IsActive);
    }

    [Fact]
    public void SupplyingMaterialChange_ShouldWakeTheDirectMixedContactWithoutMovingItsCollider()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        LSCollider2D member = Rectangle(scene.Context, new(4, 2), new(0, -1));
        LSCollider2D remote = Rectangle(scene.Context, new(4, 2), new(8, -1));
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { member, remote });
        var sphere = scene.CreateSphere(new(Fixed64.Zero, Fixed64.Zero, Fixed64.Quarter));
        var unrelated = scene.CreateSphere(new((Fixed64)8, Fixed64.Zero, Fixed64.Quarter));
        unrelated.Body.FreezeAxes = BodyFreezeAxes3D.PositionZ | BodyFreezeAxes3D.Rotation;
        unrelated.Body.UseManualGrounding();
        sphere.Body.FreezeAxes = BodyFreezeAxes3D.PositionZ | BodyFreezeAxes3D.Rotation;
        sphere.Body.UseManualGrounding();
        scene.Context.Simulate(); scene.Context.LateSimulate();
        sphere.Body.Sleep();
        unrelated.Body.Sleep();
        Assert.True(sphere.Body.IsSleeping);
        Vector2d position = member.Center;
        member.Material = PhysicsMaterial.Bouncy;
        Assert.False(sphere.Body.IsSleeping);
        Assert.True(unrelated.Body.IsSleeping);
        Assert.Equal(position, member.Center);
        var contacts = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildDiscreteManifold(sphere.Collider, member, contacts));
        foreach (ManifoldContact row in contacts) Assert.Equal(member.Material, row.MaterialB);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void InvalidatingAnUnresolvedOrTriggerSurfacePair_ShouldNotWakeItsBody(bool trigger)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        LSCollider2D member = Rectangle(scene.Context, new(4, 2), new(0, -1));
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { member });
        var collider = new LSSphereCollider { Radius = Fixed64.Half };
        SolidBody body = scene.CreateBody(collider, new(Fixed64.Zero, Fixed64.Zero, Fixed64.Quarter),
            FixedQuaternion.Identity).Body;
        body.UseManualGrounding();
        var pair = new CollisionPairMixed(collider, member);
        if (trigger)
        {
            member.IsTrigger = true;
            var contacts = new ContactManifold();
            Assert.True(CollisionDetectionMixed.TryBuildDiscreteManifold(collider, member, contacts));
            pair.MarkColliding(scene.Context.FrameCount, contacts);
        }
        body.Sleep();
        pair.InvalidateSurfaceContact(wake: true);
        Assert.True(body.IsSleeping);
        Assert.False(pair.HasCurrentManifoldCache);
    }

    private static LSCollider2D Rectangle(GravitasWorldContext context, Vector2d size, Vector2d center)
    {
        var collider = new LSAABBoxCollider2D(size);
        collider.InitializeWithNoBody(new TestMatterAgent(context, new FixedTransform(
            new(center.X, Fixed64.Zero, center.Y), FixedQuaternion.Identity, Vector3d.One)));
        return collider;
    }

    private static SolidBody2D PlanarBody(GravitasWorldContext context, LSCollider2D collider, Vector2d center)
    {
        var body = new SolidBody2D(new TestMatterAgent(context), collider) { Mass = Fixed64.One };
        body.Initialize(center); body.UseManualGrounding();
        return body;
    }

    private static void Configure(GravitasWorldContext context, bool planar)
    {
        context.ApplySettings(new PhysicsSettings(16, new[,] { { true, true }, { true, true } })
        { RuntimeMode = planar ? PhysicsRuntimeMode.TwoD : PhysicsRuntimeMode.ThreeD });
        context.Environment.Gravity = context.Environment.AirDensity = context.Environment.DampingFactor =
            context.Environment.MinSpeed = Fixed64.Zero;
    }

    private static ChronicleHash Hash(CollisionPair pair)
    {
        var writer = new ChronicleHashWriter();
        pair.ContributeReplayHash(ref writer, GravitasReplayHashMode.Authoritative);
        return writer.ToHash();
    }

    private static ChronicleHash Hash(CollisionPair2D pair)
    {
        var writer = new ChronicleHashWriter();
        pair.ContributeReplayHash(ref writer, GravitasReplayHashMode.Authoritative);
        return writer.ToHash();
    }
}
