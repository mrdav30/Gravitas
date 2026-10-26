using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using System;
using System.Collections.Generic;
using System.Reflection;
using Xunit;

namespace Gravitas.Tests;

public sealed class JoinedStaticSurfaceMixedTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SymmetricExitTie_ShouldSampleTheChosenAsymmetricFootprintFace(bool reversed)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        LSMeshCollider wall = Patch(scene,
            new[] { new Vector3d(-4, -2, 0), new(-4, 2, 0), new(4, -2, 0), new(4, 2, 0) },
            reversed ? new[] { 2, 1, 0, 3, 1, 2 } : new[] { 0, 1, 2, 2, 1, 3 });
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { wall });
        var triangle = new LSPolygonCollider2D(new(-1, -1), new(1, -1), new(-1, 1))
        { MixedHalfThicknessOverride = Fixed64.Half };
        var body = new SolidBody2D(new TestMatterAgent(scene.Context), triangle) { Mass = Fixed64.One };
        body.Initialize(Vector2d.Zero);
        var manifold = new ContactManifold();

        Assert.True(CollisionDetectionMixed.TryBuildDiscreteManifold(wall, triangle, manifold));

        // The canonical +Z exit uses the complete base edge times slab height,
        // rather than the opposite tip's line: choose orientation before dimension.
        Assert.Equal(4, manifold.Count);
        var corners = new HashSet<(Fixed64 X, Fixed64 Y)>();
        foreach (ManifoldContact row in manifold)
        {
            Assert.Equal(Vector3d.Forward, row.Normal);
            Assert.Equal(Fixed64.One, row.Depth);
            Assert.True(row.AnchorA.TryGetWorldPoint(out Vector3d point));
            Assert.Equal(Fixed64.One, FixedMath.Abs(point.X));
            Assert.Equal(Fixed64.Half, FixedMath.Abs(point.Y));
            Assert.Equal(Fixed64.Zero, point.Z);
            corners.Add((point.X, point.Y));
        }
        Assert.Equal(4, corners.Count);
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(1, false)]
    [InlineData(2, false)]
    [InlineData(3, false)]
    [InlineData(4, false)]
    [InlineData(5, false)]
    [InlineData(0, true)]
    [InlineData(1, true)]
    [InlineData(2, true)]
    [InlineData(3, true)]
    [InlineData(4, true)]
    public void Crossing_ShouldMatchContinuousSurfaceWithoutLosingMemberEvents(int shape, bool horizontal)
    {
        List<long> expected = Trace(false, false);
        Assert.Equal(expected, Trace(true, false));
        Assert.Equal(expected, Trace(true, true));

        List<long> Trace(bool joined, bool reverse)
        {
            using var scene = PhysicsScenarioBuilder.Create();
            Configure(scene.Context);
            LSMeshCollider[] members = joined
                ? new[] { Surface(scene, -8, 0, horizontal), Surface(scene, 0, 8, horizontal) }
                : new[] { Surface(scene, -8, 8, horizontal) };
            using var surface = joined ? scene.Context.Physics.CreateStaticSurface(reverse
                ? new[] { members[1], members[0] } : members) : null;
            int entered = 0, exited = 0;
            foreach (LSMeshCollider member in members)
            {
                member.OnMixedContactEnter += _ => entered++;
                member.OnMixedContactExit += _ => exited++;
            }
            LSCollider2D collider = Shape(shape);
            Fixed64 support = shape == 2 ? Fixed64.One : Fixed64.Half;
            var agent = new TestMatterAgent(scene.Context,
                new FixedTransform(new Vector3d(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero),
                    FixedQuaternion.Identity, Vector3d.One));
            var body = new SolidBody2D(agent, collider)
            { Mass = Fixed64.One, FreezeAxes = BodyFreezeAxes2D.Rotation, SleepEnabled = false };
            body.Initialize(new(-Fixed64.Two, horizontal ? Fixed64.Zero : support));
            body.UseManualGrounding();
            for (int frame = 0; frame < 32; frame++) Step();
            body.AddLinearImpulse(Vector2d.Right);
            var trace = new List<long>();
            for (int frame = 0; frame < 64; frame++)
            {
                Step();
                Assert.Equal(Fixed64.One, body.LinearVelocity.X);
                Assert.Equal(Fixed64.Zero, body.AngularVelocity);
                Assert.Equal(Fixed64.Quarter, agent.Transform.LocalPosition.Y);
                trace.Add(body.Position.X.m_rawValue); trace.Add(body.Position.Y.m_rawValue);
                trace.Add(body.LinearVelocity.X.m_rawValue); trace.Add(body.LinearVelocity.Y.m_rawValue);
                trace.Add(body.Rotation.m_rawValue); trace.Add(body.AngularVelocity.m_rawValue);
            }
            Assert.True(body.Position.X > Fixed64.One);
            Assert.Equal(joined ? 2 : 1, entered);
            Assert.Equal(joined ? 1 : 0, exited);
            return trace;

            void Step()
            {
                // A vertical wall must provide actual planar response. A horizontal
                // face alone cannot prove mixed sliding because 2D has no Y mobility.
                if (!horizontal) body.AddLinearImpulse(new(Fixed64.Zero, -Fixed64.FromFraction(1, 16)));
                scene.Context.Simulate(); scene.Context.LateSimulate();
            }
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void GenuineOuterWallRim_ShouldKeepItsFiniteRadialContact(bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        LSMeshCollider left = Surface(scene, -8, 0, false), right = Surface(scene, 0, 8, false);
        using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { right, left } : new[] { left, right });
        var circle = new LSCircleCollider2D(Fixed64.Half) { MixedHalfThicknessOverride = Fixed64.Half };
        var body = new SolidBody2D(new TestMatterAgent(scene.Context), circle) { Mass = Fixed64.One };
        body.Initialize(new((Fixed64)(-8) - Fixed64.Quarter, Fixed64.Quarter));
        var manifold = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildDiscreteManifold(left, circle, manifold));
        bool radial = false;
        foreach (ManifoldContact row in manifold)
        {
            Assert.True(row.AnchorA.TryGetWorldPoint(out Vector3d point));
            Assert.InRange(point.X.m_rawValue, ((Fixed64)(-8)).m_rawValue, Fixed64.Zero.m_rawValue);
            Assert.Equal(Fixed64.Zero, point.Z);
            if (row.Normal.X == Fixed64.Zero) continue;
            radial = true;
            Assert.Equal((Fixed64)(-8), point.X);
            Assert.True(row.Normal.X < Fixed64.Zero);
        }
        Assert.True(radial);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    public void JoinedWallRows_ShouldRetainSupplierMaterialsAndRefreshWhenNeighborBecomesTrigger(int shape)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        LSMeshCollider left = Surface(scene, -8, 0, false), right = Surface(scene, 0, 8, false);
        right.Material = PhysicsMaterial.Bouncy;
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { right, left });
        LSCollider2D collider = Shape(shape);
        var body = new SolidBody2D(new TestMatterAgent(scene.Context), collider) { Mass = Fixed64.One };
        body.Initialize(new(Fixed64.Quarter, Fixed64.Quarter));
        int supplied = 0;
        Inspect(includeRight: true);
        Assert.True(supplied > 0);
        right.IsTrigger = true;
        supplied = 0;
        Inspect(includeRight: false);
        Assert.True(supplied > 0);

        void Inspect(bool includeRight)
        {
            var manifold = new ContactManifold();
            foreach (LSMeshCollider member in new[] { left, right })
            {
                if (!includeRight && member == right) continue;
                CollisionDetectionMixed.TryBuildDiscreteManifold(member, collider, manifold);
                for (int group = 0; group < manifold.GroupCount; group++)
                {
                    int supplier = manifold.GetGroup(group).Key.SourceIdA;
                    Assert.True(supplier == left.Id + 1 || includeRight && supplier == right.Id + 1);
                    PhysicsMaterial material = supplier == left.Id + 1 ? left.Material : right.Material;
                    int start = manifold.GetGroupStartIndex(group);
                    for (int row = 0; row < manifold.GetGroupContactCount(group); row++)
                    {
                        ManifoldContact contact = manifold[start + row];
                        Assert.True(contact.HasMaterialOverride);
                        Assert.Equal(material, contact.MaterialA);
                        Assert.Equal(collider.Material, contact.MaterialB);
                        Assert.True(contact.AnchorA.TryGetWorldPoint(out Vector3d point));
                        Assert.InRange(point.X.m_rawValue,
                            (supplier == left.Id + 1 ? (Fixed64)(-8) : Fixed64.Zero).m_rawValue,
                            (supplier == left.Id + 1 ? Fixed64.Zero : (Fixed64)8).m_rawValue);
                        supplied++;
                    }
                }
            }
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void LaterCallbackRetiringCarrierOrSupplier_ShouldRebuildBeforeResponse(bool retireSupplier, bool filter)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        LSMeshCollider left = Surface(scene, -8, 0, false), right = Surface(scene, 0, 8, false);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
        var observer = new LSSphereCollider { Radius = Fixed64.One, IsTrigger = true };
        scene.InitializeStaticCollider(observer, new(Fixed64.Quarter, Fixed64.Zero, Fixed64.Quarter));
        var circle = new LSCircleCollider2D(Fixed64.Half) { Material = PhysicsMaterial.Frictionless };
        var body = new SolidBody2D(new TestMatterAgent(scene.Context), circle)
        {
            Mass = Fixed64.One, SleepEnabled = false,
            // Removing a member exposes a real lateral rim. Freeze that axis so
            // this regression isolates carrier transfer from legitimate side motion.
            FreezeAxes = BodyFreezeAxes2D.Rotation | BodyFreezeAxes2D.PositionX
        };
        body.Initialize(new(Fixed64.Quarter, Fixed64.Quarter));
        body.UseManualGrounding();
        body.AddLinearImpulse(-Vector2d.Forward);
        int leftEntered = 0, rightEntered = 0;
        left.OnMixedContactEnter += _ => leftEntered++;
        right.OnMixedContactEnter += _ => rightEntered++;
        bool invalidated = false;
        LSMeshCollider retired = retireSupplier ? right : left, surviving = retireSupplier ? left : right;
        int retiredSupplier = retired.Id + 1;
        observer.OnMixedTriggerEnter += _ =>
        {
            Assert.Equal(1, leftEntered); Assert.Equal(1, rightEntered);
            CollisionPairMixed carrier = Pair(scene.Context, left, circle);
            CollisionPairMixed observation = Pair(scene.Context, right, circle);
            Assert.NotEmpty(carrier.Manifold);
            Assert.Empty(observation.Manifold);
            bool rightSupplied = false;
            for (int group = 0; group < carrier.Manifold.GroupCount; group++)
                rightSupplied |= carrier.Manifold.GetGroup(group).Key.SourceIdA == right.Id + 1;
            Assert.True(rightSupplied);
            if (filter) retired.IgnoredCollisionLayers = PhysicsLayerMask.All;
            else retired.Deactivate();
            invalidated = true;
        };

        scene.Context.Simulate(); scene.Context.LateSimulate();

        Assert.True(invalidated);
        CollisionPairMixed response = Pair(scene.Context, surviving, circle);
        Assert.NotEmpty(response.Manifold);
        for (int group = 0; group < response.Manifold.GroupCount; group++)
            Assert.NotEqual(retiredSupplier, response.Manifold.GetGroup(group).Key.SourceIdA);
        Assert.Equal(Fixed64.Zero, body.LinearVelocity.Y);
        Assert.True(body.Position.Y > Fixed64.Quarter);
        Assert.Equal(1, leftEntered); Assert.Equal(1, rightEntered);
    }

    [Fact]
    public void EmptyNoncarrier_ShouldKeepMemberObservationsWithoutEnteringResponse()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        LSMeshCollider left = Surface(scene, -8, 0, false), right = Surface(scene, 0, 8, false);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { right, left });
        var circle = new LSCircleCollider2D(Fixed64.Half) { Material = PhysicsMaterial.Frictionless };
        var body = new SolidBody2D(new TestMatterAgent(scene.Context), circle)
        { Mass = Fixed64.One, FreezeAxes = BodyFreezeAxes2D.Rotation, SleepEnabled = false };
        body.Initialize(new(Fixed64.Zero, Fixed64.Half)); body.UseManualGrounding();
        int entered = 0, stayed = 0;
        right.OnMixedContactEnter += _ => entered++;
        right.OnMixedContact += _ => stayed++;
        for (int frame = 0; frame < 2; frame++)
        {
            scene.Context.Simulate(); scene.Context.LateSimulate();
            CollisionPairMixed noncarrier = Pair(scene.Context, right, circle);
            Assert.Empty(noncarrier.Manifold);
            var response = (SwiftList<CollisionPairMixed>)typeof(GravitasMixedCollisionService)
                .GetField("_mixedResponsePairs", BindingFlags.Instance | BindingFlags.NonPublic)!
                .GetValue(scene.Context.MixedCollisions)!;
            Assert.DoesNotContain(noncarrier, response);
            Assert.Equal(new Vector2d(Fixed64.Zero, Fixed64.Half), body.Position);
            Assert.Equal(Vector2d.Zero, body.LinearVelocity);
        }
        Assert.Equal(1, entered); Assert.Equal(2, stayed);
    }

    [Fact]
    public void CompoundMover_ShouldRetainPartNamespacesAndMaterialsAcrossTheJoin()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        LSMeshCollider left = Surface(scene, -8, 0, false), right = Surface(scene, 0, 8, false);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { right, left });
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, -Vector2d.Right * Fixed64.Quarter, PhysicsMaterial.Frictionless),
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Right * Fixed64.Quarter, PhysicsMaterial.Bouncy));
        var body = new SolidBody2D(new TestMatterAgent(scene.Context), compound) { Mass = Fixed64.One };
        body.Initialize(new(Fixed64.Zero, Fixed64.Quarter));
        bool first = false, second = false;
        var manifold = new ContactManifold();
        foreach (LSMeshCollider member in new[] { left, right })
        {
            Assert.True(CollisionDetectionMixed.TryBuildDiscreteManifold(member, compound, manifold));
            for (int group = 0; group < manifold.GroupCount; group++)
            {
                ContactGroupKey key = manifold.GetGroup(group).Key;
                Assert.True(key.NamespaceB is -1 or -2);
                first |= key.NamespaceB == -1; second |= key.NamespaceB == -2;
                PhysicsMaterial expected = key.NamespaceB == -1 ? PhysicsMaterial.Frictionless : PhysicsMaterial.Bouncy;
                int start = manifold.GetGroupStartIndex(group);
                for (int row = 0; row < manifold.GetGroupContactCount(group); row++)
                {
                    Assert.Equal(key.NamespaceB, manifold[start + row].FeatureNamespaceB);
                    Assert.Equal(expected, manifold[start + row].MaterialB);
                }
            }
        }
        Assert.True(first); Assert.True(second);
    }

    private static CollisionPairMixed Pair(GravitasWorldContext context, LSCollider spatial, LSCollider2D planar)
    {
        var pairs = (SwiftDictionary<ulong, CollisionPairMixed>)typeof(GravitasMixedCollisionService)
            .GetField("_pairs", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(context.MixedCollisions)!;
        Assert.True(pairs.TryGetValue(MixedColliderKey.CreateKey(spatial.Id, planar.Id), out CollisionPairMixed pair));
        return pair;
    }

    [Fact]
    public void LaterCallbackReusingAnEarlierQueuedPair_ShouldSolveTheNewLifetimeOnlyOnce()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        scene.Context.Settings.PoolingEnabled = true;
        scene.Context.Settings.RestitutionVelocityThreshold = Fixed64.Zero;
        LSMeshCollider left = Surface(scene, -8, 0, false);
        var observer = new LSSphereCollider { Radius = Fixed64.One, IsTrigger = true };
        scene.InitializeStaticCollider(observer, new(Fixed64.Quarter, Fixed64.Zero, Fixed64.Quarter));
        LSMeshCollider right = Surface(scene, 0, 8, false);
        right.Material = PhysicsMaterial.Bouncy;
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
        var circle = new LSCircleCollider2D(Fixed64.Half) { Material = PhysicsMaterial.Bouncy };
        var body = new SolidBody2D(new TestMatterAgent(scene.Context), circle)
        {
            Mass = Fixed64.One, SleepEnabled = false,
            FreezeAxes = BodyFreezeAxes2D.PositionX | BodyFreezeAxes2D.Rotation
        };
        body.Initialize(new(Fixed64.Quarter, Fixed64.Quarter)); body.UseManualGrounding();
        body.AddLinearImpulse(-Vector2d.Forward);
        CollisionPairMixed? original = null;
        left.OnMixedContactEnter += _ => original = Pair(scene.Context, left, circle);
        bool retired = false, reused = false;
        observer.OnMixedTriggerEnter += _ =>
        {
            Assert.NotNull(original);
            Assert.NotEmpty(original.Manifold);
            left.Deactivate();
            retired = true;
        };
        right.OnMixedContactEnter += _ =>
        {
            Assert.True(retired);
            Assert.Same(original, Pair(scene.Context, right, circle));
            reused = true;
        };

        scene.Context.Simulate(); scene.Context.LateSimulate();

        Assert.True(reused);
        CollisionPairMixed current = Pair(scene.Context, right, circle);
        var response = (SwiftList<CollisionPairMixed>)typeof(GravitasMixedCollisionService)
            .GetField("_mixedResponsePairs", BindingFlags.Instance | BindingFlags.NonPublic)!
            .GetValue(scene.Context.MixedCollisions)!;
        Assert.Same(current, Assert.Single(response));
        Assert.Equal(Vector2d.Forward, body.LinearVelocity);
    }

    [Theory]
    [InlineData(2)]
    [InlineData(4)]
    public void FreelyRotatingObliqueSlab_ShouldMatchTheContinuousWall(int shape)
    {
        List<Fixed64[]> expected = Trace(false);
        List<Fixed64[]> actual = Trace(true);
        // As in pure 2D, finite lever sampling may differ while the same contact
        // field evolves. Bound the full 64-step state difference to 0.001 units.
        Fixed64 tolerance = Fixed64.FromFraction(1, 1000);
        for (int frame = 0; frame < expected.Count; frame++)
            for (int component = 0; component < expected[frame].Length; component++)
                Assert.True(FixedMath.Abs(expected[frame][component] - actual[frame][component]) <= tolerance,
                    $"Frame {frame}, component {component}: {expected[frame][component]} versus {actual[frame][component]}.");

        List<Fixed64[]> Trace(bool joined)
        {
            using var scene = PhysicsScenarioBuilder.Create();
            Configure(scene.Context);
            LSMeshCollider[] members = joined
                ? new[] { Surface(scene, -8, 0, false), Surface(scene, 0, 8, false) }
                : new[] { Surface(scene, -8, 8, false) };
            using var surface = joined ? scene.Context.Physics.CreateStaticSurface(new[] { members[1], members[0] }) : null;
            LSCollider2D collider = Shape(shape);
            var body = new SolidBody2D(new TestMatterAgent(scene.Context), collider)
            { Mass = Fixed64.One, SleepEnabled = false };
            Fixed64 initialRotation = Fixed64.PiOver4 * Fixed64.Half;
            body.Initialize(new(-Fixed64.Quarter, Fixed64.One - Fixed64.Quarter), initialRotation);
            body.UseManualGrounding();
            body.AddLinearImpulse(new(Fixed64.One, -Fixed64.Quarter));
            body.AddAngularImpulse(Fixed64.FromFraction(1, 64));
            bool rotated = false;
            var trace = new List<Fixed64[]>();
            for (int frame = 0; frame < 64; frame++)
            {
                scene.Context.Simulate(); scene.Context.LateSimulate();
                Assert.Equal(Fixed64.One, body.LinearVelocity.X);
                rotated |= body.Rotation != initialRotation && body.AngularVelocity != Fixed64.Zero;
                trace.Add(new[] { body.Position.X, body.Position.Y, body.LinearVelocity.X,
                    body.LinearVelocity.Y, body.Rotation, body.AngularVelocity });
            }
            Assert.True(rotated);
            Assert.True(body.Position.X > Fixed64.One);
            return trace;
        }
    }

    [Fact]
    public void UpperWallRim_ShouldRetainItsFiniteHeightWithoutManufacturingASeamNormal()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        LSMeshCollider left = Surface(scene, -8, 0, false), right = Surface(scene, 0, 8, false);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { right, left });
        LSCollider2D circle = Shape(0);
        var agent = new TestMatterAgent(scene.Context, new FixedTransform(
            new Vector3d(Fixed64.Zero, Fixed64.Two + Fixed64.Quarter, Fixed64.Zero),
            FixedQuaternion.Identity, Vector3d.One));
        var body = new SolidBody2D(agent, circle) { Mass = Fixed64.One };
        body.Initialize(new(Fixed64.Zero, Fixed64.Quarter));
        bool rim = false;
        var manifold = new ContactManifold();
        foreach (LSMeshCollider member in new[] { left, right })
        {
            Assert.True(CollisionDetectionMixed.TryBuildDiscreteManifold(member, circle, manifold));
            foreach (ManifoldContact row in manifold)
            {
                Assert.Equal(Fixed64.Zero, row.Normal.X);
                Assert.True(row.AnchorA.TryGetWorldPoint(out Vector3d point));
                Assert.InRange(point.Y.m_rawValue, (-Fixed64.Two).m_rawValue, Fixed64.Two.m_rawValue);
                if (row.Normal.Y == Fixed64.Zero) continue;
                Assert.Equal(Fixed64.Two, point.Y);
                Assert.True(row.Normal.Y > Fixed64.Zero);
                rim = true;
            }
        }
        Assert.True(rim);
    }

    [Fact]
    public void LaterCallbackMovingBothMembersAway_ShouldDiscardAllPendingResponse()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        LSMeshCollider left = Surface(scene, -8, 0, false), right = Surface(scene, 0, 8, false);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
        var observer = new LSSphereCollider { Radius = Fixed64.One, IsTrigger = true };
        scene.InitializeStaticCollider(observer, new(Fixed64.Zero, Fixed64.Zero, Fixed64.Quarter));
        LSCollider2D circle = Shape(0);
        var body = new SolidBody2D(new TestMatterAgent(scene.Context), circle)
        { Mass = Fixed64.One, FreezeAxes = BodyFreezeAxes2D.Rotation, SleepEnabled = false };
        body.Initialize(new(Fixed64.Zero, Fixed64.Quarter)); body.UseManualGrounding();
        body.AddLinearImpulse(-Vector2d.Forward);
        int entered = 0;
        left.OnMixedContactEnter += _ => entered++;
        right.OnMixedContactEnter += _ => entered++;
        bool moved = false;
        observer.OnMixedTriggerEnter += _ =>
        {
            Assert.Equal(2, entered);
            foreach (LSMeshCollider member in new[] { left, right })
            {
                member.Transform.LocalPosition = Vector3d.Forward * (Fixed64)8;
                member.Simulate();
            }
            moved = true;
        };

        scene.Context.Simulate(); scene.Context.LateSimulate();

        Assert.True(moved);
        Assert.Equal(-Vector2d.Forward, body.LinearVelocity);
        Assert.Equal(new Vector2d(Fixed64.Zero, Fixed64.FromFraction(3, 16)), body.Position);
        var response = (SwiftList<CollisionPairMixed>)typeof(GravitasMixedCollisionService)
            .GetField("_mixedResponsePairs", BindingFlags.Instance | BindingFlags.NonPublic)!
            .GetValue(scene.Context.MixedCollisions)!;
        Assert.Empty(response);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void AffineFilledFace_ShouldPublishPhysicalMixedManifoldAnchorsAndItsWholeShapeExit(int shape)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var mesh = Patch(scene, new[] { new Vector3d(-16, 0, -16), new(16, 0, -16), new(0, 0, 18) }, new[] { 0, 1, 2 });
        mesh.Transform.LocalScale = Vector3d.One * Fixed64.Half;
        mesh.Simulate();
        LSCollider2D embedded = shape == 0 ? new LSCircleCollider2D(Fixed64.One)
            : shape == 1 ? new LSCapsuleCollider2D(Fixed64.One, (Fixed64)4)
            : new LSPolygonCollider2D(new(-1, -1), new(1, -1), new(1, 1), new(-1, 1));
        embedded.MixedHalfThicknessOverride = Fixed64.Half;
        embedded.InitializeWithNoBody(new TestMatterAgent(scene.Context,
            new FixedTransform(Vector3d.Up * Fixed64.Quarter, FixedQuaternion.Identity, Vector3d.One)));
        var manifold = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mesh, embedded, manifold));
        Assert.True(manifold.Count > 0);
        for (int index = 0; index < manifold.Count; index++)
        {
            ManifoldContact row = manifold[index];
            Assert.Equal(Vector3d.Up, row.Normal); Assert.Equal(Fixed64.Quarter, row.Depth);
            Assert.True(row.TryGetPointA(out Vector3d source)); Assert.True(row.TryGetPointB(out Vector3d exit));
            Assert.Equal(Fixed64.Zero, source.Y); Assert.Equal(-Fixed64.Quarter, exit.Y);
            Assert.Equal(source.X, exit.X); Assert.Equal(source.Z, exit.Z);
        }
        embedded.Agent.Transform.LocalPosition = Vector3d.Up * (Fixed64.Half + Fixed64.MinIncrement);
        embedded.Simulate();
        Assert.False(CollisionDetectionMixed.TryBuildManifold(mesh, embedded, manifold));
        Assert.Equal(0, manifold.Count);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void PartialRationalJoin_ShouldMatchTheExplicitlyTriangulatedFiniteFill(int shape)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        // The cut is representable, but its parameter on the four-unit edge
        // has a sub-raw remainder. It must be clipped before materialization.
        Fixed64 cut = Fixed64.FromFraction(1, 3);
        Vector3d[] vertices = { new(-2, 0, 0), new(cut, Fixed64.Zero, Fixed64.Zero),
            new(2, 0, 0), new(-2, 2, 0), new(-2, -2, 0) };
        LSMeshCollider whole = Patch(scene, vertices, new[] { 0, 1, 3, 1, 2, 3, 0, 4, 1 });
        LSMeshCollider source = Patch(scene, new[] { vertices[0], vertices[2], vertices[3] }, new[] { 0, 1, 2 });
        LSMeshCollider neighbor = Patch(scene, new[] { vertices[0], vertices[1], vertices[4] }, new[] { 0, 2, 1 });
        LSCollider2D embedded = shape == 0 ? new LSCircleCollider2D(Fixed64.One)
            : shape == 1 ? new LSCapsuleCollider2D(Fixed64.One, (Fixed64)4)
            : new LSPolygonCollider2D(new(-1, -1), new(1, -1), new(-1, 1));
        embedded.MixedHalfThicknessOverride = Fixed64.Quarter;
        var body = new SolidBody2D(new TestMatterAgent(scene.Context), embedded) { Mass = Fixed64.One };
        // A positive offset makes the nearer planar exit unique; the exactly
        // centered sheet has two valid symmetric exits with owner-specific ties.
        body.Initialize(shape == 2 ? Vector2d.Forward : Vector2d.Forward * Fixed64.FromFraction(1, 16));
        ReadRows(new[] { whole }, out _, out var expectedConstraints);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { neighbor, source });
        List<string> joined = ReadRows(new[] { source, neighbor }, out bool cutRetained, out var constraints);
        Assert.NotEmpty(joined);
        // Independent finite sample pools need not choose identical torque arms.
        // Their physical normal/depth fields must agree, and every actual seam
        // endpoint remains subject to the exposed-domain checks below.
        Assert.True(expectedConstraints.SetEquals(constraints),
            "Continuous normal/depth fields: " + Describe(expectedConstraints)
            + "\nJoined normal/depth fields: " + Describe(constraints));
        Assert.True(cutRetained);
        surface.ReplaceMembers(new[] { source, neighbor });
        Assert.Equal(joined, ReadRows(new[] { neighbor, source }, out _, out _));
        var warm = new ContactManifold();
        bool valid = true;
        long allocated = AllocationTestHelper.MeasureSteadyState(() =>
        {
            valid &= CollisionDetectionMixed.TryBuildDiscreteManifold(source, embedded, warm);
            int count = warm.Count;
            valid &= CollisionDetectionMixed.TryBuildDiscreteManifold(neighbor, embedded, warm);
            valid &= count + warm.Count > 0;
        }, warmupIterations: 2, stabilizationIterations: 2, measurementIterations: 4);
        Assert.True(valid); Assert.Equal(0, allocated);

        static string Describe(HashSet<(Vector3d Normal, Fixed64 Depth)> fields)
        {
            var rows = new List<string>();
            foreach (var field in fields)
                rows.Add($"({field.Normal.X.m_rawValue},{field.Normal.Y.m_rawValue},{field.Normal.Z.m_rawValue}) depth={field.Depth.m_rawValue}");
            rows.Sort(StringComparer.Ordinal);
            return string.Join("; ", rows);
        }

        List<string> ReadRows(LSMeshCollider[] members, out bool foundCut,
            out HashSet<(Vector3d Normal, Fixed64 Depth)> fields)
        {
            var result = new List<string>();
            var manifold = new ContactManifold();
            fields = new();
            foundCut = false;
            foreach (LSMeshCollider member in members)
            {
                Assert.True(CollisionDetectionMixed.TryBuildDiscreteManifold(member, embedded, manifold));
                foreach (ManifoldContact row in manifold)
                {
                    Assert.True(row.AnchorA.TryGetWorldPoint(out Vector3d a));
                    Assert.True(row.AnchorB.TryGetWorldPoint(out Vector3d b));
                    Assert.Equal(Fixed64.Zero, a.Z);
                    Assert.InRange(a.X.m_rawValue, ((Fixed64)(-2)).m_rawValue, Fixed64.Two.m_rawValue);
                    Assert.InRange(a.Y.m_rawValue, ((Fixed64)(-2)).m_rawValue, Fixed64.Two.m_rawValue);
                    if (a.Y == Fixed64.Zero && row.Normal.Y < Fixed64.Zero)
                        Assert.True(a.X >= cut, "A downward perimeter row cannot use the covered half-neighborhood.");
                    foundCut |= a == vertices[1];
                    fields.Add((row.Normal, row.Depth));
                    result.Add($"{row.Normal.X.m_rawValue},{row.Normal.Y.m_rawValue},{row.Normal.Z.m_rawValue};"
                        + $"{row.Depth.m_rawValue};{a.X.m_rawValue},{a.Y.m_rawValue},{a.Z.m_rawValue};"
                        + $"{b.X.m_rawValue},{b.Y.m_rawValue},{b.Z.m_rawValue}");
                }
            }
            result.Sort(StringComparer.Ordinal);
            return result;
        }
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(1, false)]
    [InlineData(2, false)]
    [InlineData(0, true)]
    [InlineData(1, true)]
    [InlineData(2, true)]
    public void FiniteJoinedWallInsideASlab_ShouldRetainAnActualWholeShapeExit(int shape, bool affine)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        Vector3d[] first = { new((Fixed64)3, -Fixed64.Half, Fixed64.Zero), new((Fixed64)4, -Fixed64.Half, Fixed64.Zero),
            new((Fixed64)3, Fixed64.Half, Fixed64.Zero) };
        Vector3d[] second = { first[1], new((Fixed64)4, Fixed64.Quarter, Fixed64.Zero), first[2] };
        LSMeshCollider left = Patch(scene, affine ? Array.ConvertAll(first, p => p * Fixed64.Two) : first, new[] { 0, 1, 2 });
        LSMeshCollider right = Patch(scene, affine ? Array.ConvertAll(second, p => p * Fixed64.Two) : second, new[] { 0, 2, 1 });
        if (affine)
            foreach (LSMeshCollider member in new[] { left, right })
            { member.Transform.LocalScale = Vector3d.One * Fixed64.Half; member.Simulate(); }
        LSCollider2D embedded = shape == 0 ? new LSCircleCollider2D((Fixed64)5)
            : shape == 1 ? new LSCapsuleCollider2D((Fixed64)5, (Fixed64)12)
            : new LSPolygonCollider2D(new(0, -5), new(5, 0), new(0, 5));
        embedded.MixedHalfThicknessOverride = Fixed64.One;
        var body = new SolidBody2D(new TestMatterAgent(scene.Context), embedded) { Mass = Fixed64.One };
        body.Initialize(Vector2d.Zero);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { right, left });
        var manifold = new ContactManifold();
        bool wholeExit = false;
        foreach (bool reverse in new[] { false, true })
        {
            surface.ReplaceMembers(reverse ? new[] { left, right } : new[] { right, left });
            bool found = false;
            foreach (LSMeshCollider member in new[] { left, right })
            {
                Assert.True(CollisionDetectionMixed.TryBuildDiscreteManifold(member, embedded, manifold));
                foreach (ManifoldContact row in manifold)
                {
                    Assert.True(row.AnchorA.TryGetWorldPoint(out Vector3d point));
                    Assert.Equal(Fixed64.Zero, point.Z);
                    Assert.InRange(point.X.m_rawValue, ((Fixed64)3).m_rawValue, ((Fixed64)4).m_rawValue);
                    // A radial exit can be shorter than an arbitrarily selected
                    // normal-ray maximum. Verify the finite shape, not that old proof's direction.
                    Vector3d touching, separated;
                    if (shape == 2)
                    {
                        // The triangle's two sloping sides have x +/- z = 5.
                        // At x=3 they leave by a unit shift on both planar axes,
                        // whose distance is sqrt(2), shorter than the axial exit2.
                        if (row.Normal.X >= Fixed64.Zero || row.Normal.Y != Fixed64.Zero
                            || FixedMath.Abs(row.Normal.X) != FixedMath.Abs(row.Normal.Z)) continue;
                        Fixed64 expected = FixedMath.Sqrt(Fixed64.Two);
                        Assert.InRange(row.Depth.m_rawValue, expected.m_rawValue - 1, expected.m_rawValue + 1);
                        Fixed64 sign = row.Normal.Z < Fixed64.Zero ? -Fixed64.One : Fixed64.One;
                        touching = new(-Fixed64.One, Fixed64.Zero, sign);
                        separated = new(-Fixed64.One - Fixed64.MinIncrement, Fixed64.Zero, sign * (Fixed64.One + Fixed64.MinIncrement));
                        Vector3d correction = row.Normal * row.Depth;
                        // Rounded unit normal, rounded depth and their product
                        // introduce at most two raw units per correction coordinate.
                        Assert.InRange((correction.X - touching.X).m_rawValue, -2, 2);
                        Assert.InRange((correction.Z - touching.Z).m_rawValue, -2, 2);
                    }
                    else
                    {
                        if (row.Normal != -Vector3d.Right) continue;
                        Assert.Equal(Fixed64.Two, row.Depth);
                        touching = row.Normal * row.Depth;
                        separated = row.Normal * (row.Depth + Fixed64.MinIncrement);
                    }
                    Assert.True(Intersects(left, touching) || Intersects(right, touching));
                    Assert.False(Intersects(left, separated)); Assert.False(Intersects(right, separated));
                    found = true;
                }
            }
            Assert.True(found);
            wholeExit |= found;
        }
        Assert.True(wholeExit);

        bool Intersects(LSMeshCollider member, Vector3d center)
        {
            // The independent oracle uses the known physical triangle, so the
            // affine representation cannot accidentally validate its own chart.
            Vector3d[] points = member == left ? first : second;
            var triangle = new FixedTriangle(points[0], points[1], points[2]);
            if (embedded is LSCircleCollider2D circle)
                return triangle.TryGetCircleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
                    center, embedded.Rotation, embedded.MixedHalfThickness, circle.ScaledRadius, out _);
            if (embedded is LSCapsuleCollider2D capsule)
                return triangle.TryGetCenteredCapsuleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
                    center, embedded.Rotation, Vector2d.Forward, capsule.AxisLength, capsule.ScaledRadius,
                    embedded.MixedHalfThickness, out _);
            Span<Vector2d> box = stackalloc Vector2d[4];
            ReadOnlySpan<Vector2d> offsets = CollisionDetectionMixed.GetConvexPrismOffsets(embedded, box);
            return FixedConvexPrismRelations.TryGetTriangleContact(Vector3d.Zero, FixedQuaternion.Identity,
                triangle, center, embedded.ConvexRotation, offsets, embedded.MixedHalfThickness, out _);
        }
    }

    private static LSMeshCollider Patch(PhysicsScenarioBuilder scene, Vector3d[] vertices, int[] triangles)
    {
        var member = new LSMeshCollider(vertices, triangles, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
        { Material = PhysicsMaterial.Frictionless };
        scene.InitializeStaticCollider(member, Vector3d.Zero);
        return member;
    }

    private static void Configure(GravitasWorldContext context)
    {
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.SetFrameRate(16);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        context.Environment.MinSpeed = Fixed64.Zero;
    }

    private static LSCollider2D Shape(int shape)
    {
        LSCollider2D result = shape switch
        {
            0 => new LSCircleCollider2D(Fixed64.Half),
            1 => new LSCapsuleCollider2D(Fixed64.Half, Fixed64.One),
            2 => new LSCapsuleCollider2D(Fixed64.Half, Fixed64.Two),
            3 => new LSAABBoxCollider2D(Vector2d.One),
            4 => new LSPolygonCollider2D(new(-Fixed64.Half, -Fixed64.Half), new(Fixed64.Half, -Fixed64.Half),
                new(Fixed64.Half, Fixed64.Half), new(-Fixed64.Half, Fixed64.Half)),
            _ => new LSPolygonCollider2D(new(-Fixed64.Half, -Fixed64.Half), new(Fixed64.Half, -Fixed64.Half),
                new(Fixed64.One - Fixed64.Quarter, Fixed64.Zero), new(Fixed64.Half, Fixed64.Half),
                new(-Fixed64.Half, Fixed64.Half))
        };
        result.Material = PhysicsMaterial.Frictionless;
        result.MixedHalfThicknessOverride = Fixed64.Half;
        return result;
    }

    private static LSMeshCollider Surface(PhysicsScenarioBuilder scene, int left, int right, bool horizontal)
    {
        Vector3d[] points = horizontal
            ? new[] { new Vector3d(left, 0, -2), new(left, 0, 2), new(right, 0, -2), new(right, 0, 2) }
            : new[] { new Vector3d(left, -2, 0), new(left, 2, 0), new(right, -2, 0), new(right, 2, 0) };
        var mesh = new LSMeshCollider(points, new[] { 0, 1, 2, 2, 1, 3 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
        { Material = PhysicsMaterial.Frictionless };
        scene.InitializeStaticCollider(mesh, Vector3d.Zero);
        return mesh;
    }
}
