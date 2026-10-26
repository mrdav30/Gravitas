using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using System;
using System.Collections.Generic;
using System.Reflection;
using System.Runtime.CompilerServices;
using Xunit;

namespace Gravitas.Tests;

/// <content>Checks retained ownership, native rim connectivity and ended lifetimes.</content>
public sealed partial class JoinedStaticSolidMixedTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void UniqueDownwardSupportVertex_ShouldConnectTheJoinedCapsAtItsExactProjection(bool registerRightFirst)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        LSCollider2D first = Rectangle(scene, registerRightFirst ? 0 : -4, registerRightFirst ? 4 : 0, -2, 0, Fixed64.Two);
        LSCollider2D second = Rectangle(scene, registerRightFirst ? -4 : 0, registerRightFirst ? 0 : 4, -2, 0, Fixed64.Two);
        var members = registerRightFirst ? new[] { second, first } : new[] { first, second };
        var tetra = new LSMeshCollider(new Vector3d[] { new(0, -1, 0), new(-1, 1, -1), new(1, 1, -1), new(0, 1, 1) },
            new[] { 0, 1, 2, 0, 3, 1, 0, 2, 3, 1, 3, 2 }, MeshColliderMode.Convex, MeshInertiaPolicy.RequireClosedVolume);
        scene.CreateBody(tetra, new(Fixed64.Zero, 11 * Fixed64.Quarter, -Fixed64.One), FixedQuaternion.Identity);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var rows = Rows(tetra, members); Assert.NotEmpty(rows);
        var regions = new HashSet<int>();
        foreach (var sample in rows)
        {
            Assert.Equal(Vector3d.Down, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(new Vector3d(Fixed64.Zero, 7 * Fixed64.Quarter, -Fixed64.One), moving);
            Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.Two, -Fixed64.One), source);
            regions.Add(sample.Region);
        }
        Assert.Single(regions);
        AssertWholeShapeExit(tetra, members, new(0, 3, -1), Vector3d.Up);
    }

    [Fact]
    public void RemoteCapJoin_ShouldKeepTheActuallyDisconnectedProjectedSupports()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        LSCollider2D left = Rectangle(scene, -2, 0, -2, 2, Fixed64.Two);
        var right = new LSPolygonCollider2D(new(0, -2), new(2, 2), new(0, -1))
            { MixedHalfThicknessOverride = Fixed64.Two, Material = PhysicsMaterial.Frictionless };
        right.InitializeWithNoBody(new TestMatterAgent(scene.Context));
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right });
        LSCollider mover = Mover(scene, false, new(Fixed64.Zero, 7 * Fixed64.Quarter, Fixed64.Zero),
            FixedQuaternion.Identity, new(4, 1, 1));
        var rows = Rows(mover, left, right); Assert.NotEmpty(rows);
        var regions = new HashSet<int>(); var suppliers = new HashSet<int>();
        foreach (var sample in rows)
        {
            Assert.Equal(Vector3d.Down, sample.Row.Normal); Assert.Equal(3 * Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d point));
            Assert.Equal(Fixed64.Two, point.Y);
            Assert.InRange(point.Z.m_rawValue, (-Fixed64.Half).m_rawValue, Fixed64.Half.m_rawValue);
            regions.Add(sample.Region); suppliers.Add(sample.Supplier);
        }
        Assert.Equal(2, regions.Count);
        Assert.Contains(left.Id + 1, suppliers); Assert.Contains(right.Id + 1, suppliers);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void CoplanarFaceAndVerticalTip_ShouldRetainTheirActualProjectedConnectivity(bool hull, bool connected)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        int x = connected ? 0 : 2;
        var tip = new LSPolygonCollider2D(new(x, 0), new(x, -2), new(x + 2, -2))
            { MixedHalfThicknessOverride = Fixed64.Two, Material = PhysicsMaterial.Frictionless };
        // Exercise both supplier orders, including an edge node preceding its face.
        if (hull) tip.InitializeWithNoBody(new TestMatterAgent(scene.Context));
        LSCollider2D face = Rectangle(scene, connected ? -2 : -4, connected ? 0 : -2, -2, 0, Fixed64.Two);
        if (!hull) tip.InitializeWithNoBody(new TestMatterAgent(scene.Context));
        LSCollider2D[] members = connected ? new LSCollider2D[] { face, tip }
            : new[] { face, tip, Rectangle(scene, -2, 2, -2, -1, Fixed64.Two) };
        LSCollider mover = Mover(scene, hull, Vector3d.Forward * Fixed64.Quarter, FixedQuaternion.Identity,
            new(connected ? 3 : 7, 1, 1));
        var ordinary = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, face, ordinary));
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, tip, ordinary));
        Assert.Equal(Vector3d.Backward, ordinary.PrimaryContact.Normal);
        Assert.Equal(Fixed64.Quarter, ordinary.PrimaryContact.Depth);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        var regions = new HashSet<int>(); var suppliers = new HashSet<int>();
        foreach (var sample in rows)
        {
            Assert.Equal(Vector3d.Backward, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d point));
            Assert.Equal(Fixed64.Zero, point.Z);
            Assert.InRange(point.Y.m_rawValue, (-Fixed64.Half).m_rawValue, Fixed64.Half.m_rawValue);
            if (connected) Assert.True(point.X <= Fixed64.Zero);
            else Assert.True(point.X <= -Fixed64.Two || point.X == Fixed64.Two);
            regions.Add(sample.Region); suppliers.Add(sample.Supplier);
        }
        Assert.Equal(connected ? 1 : 2, regions.Count);
        Assert.Contains(face.Id + 1, suppliers);
        if (connected) Assert.True(rows.Count <= ContactManifold.MaxContactsPerGroup);
        else Assert.Contains(tip.Id + 1, suppliers);
    }

    [Fact]
    public void ObliqueSolidCorner_ShouldRetainTheSelectedPrismVertex()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two), Rectangle(scene, 0, 4, -2, 0, Fixed64.Two) };
        var tetra = new LSMeshCollider(new Vector3d[] { new(-1, -1, 2), new(-1, 2, -1), new(2, -1, -1), new(2, 2, 2) },
            new[] { 0, 1, 2, 0, 3, 1, 0, 2, 3, 1, 3, 2 }, MeshColliderMode.Convex, MeshInertiaPolicy.RequireClosedVolume);
        Vector3d corner = new(4, 2, 0);
        scene.CreateBody(tetra, corner - Vector3d.One * Fixed64.Quarter, FixedQuaternion.Identity);
        var ordinary = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(tetra, members[1], ordinary));
        Fixed64 depth = ordinary.PrimaryContact.Depth;
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var rows = Rows(tetra, members); Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.True(sample.Row.Normal.X < Fixed64.Zero && sample.Row.Normal.Y < Fixed64.Zero && sample.Row.Normal.Z < Fixed64.Zero);
            Assert.Equal(depth, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d point)); Assert.Equal(corner, point);
        }
        // The support face is x+y+z=0 in the original tetrahedron chart.
        AssertWholeShapeExit(tetra, members, corner, Vector3d.One);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void EnclosedCoplanarBandPatches_ShouldConnectThroughTheirFiniteSharedSegments(bool hull)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var left = Rectangle(scene, -2, 0, -2, 0, Fixed64.Two);
        var right = Rectangle(scene, 0, 2, -2, 0, Fixed64.Half);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right });
        var mover = Mover(scene, hull, new(-Fixed64.One, Fixed64.Zero, Fixed64.Quarter), FixedQuaternion.Identity, new(8, 8, 1));
        var regions = new HashSet<int>();
        foreach (var sample in Rows(mover, left, right))
        {
            Assert.Equal(Vector3d.Backward, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d point)); Assert.Equal(Fixed64.Zero, point.Z);
            Assert.InRange(point.X.m_rawValue, (-Fixed64.Two).m_rawValue, Fixed64.Two.m_rawValue);
            Fixed64 height = sample.Supplier == left.Id + 1 ? Fixed64.Two : Fixed64.Half;
            Assert.InRange(point.Y.m_rawValue, (-height).m_rawValue, height.m_rawValue);
            regions.Add(sample.Region);
        }
        Assert.Single(regions);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void IneligibleNeighbor_ShouldExposeTheSurvivingSideAndRestoreItsJoin(bool trigger)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var left = Rectangle(scene, -4, 0, -2, 0, Fixed64.Two);
        var right = Rectangle(scene, 0, 4, -2, 0, Fixed64.Two);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right });
        var mover = Mover(scene, false, new(Fixed64.Quarter, Fixed64.Zero, -Fixed64.One), FixedQuaternion.Identity);
        foreach (var sample in Rows(mover, left, right)) Assert.Equal(Fixed64.Zero, sample.Row.Normal.X);
        if (trigger) right.IsTrigger = true; else right.IgnoredCollisionLayers = PhysicsLayerMask.All;
        var exposed = Rows(mover, left); Assert.NotEmpty(exposed);
        foreach (var sample in exposed)
        {
            Assert.Equal(Vector3d.Left, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
        }
        right.IsTrigger = false; right.IgnoredCollisionLayers = PhysicsLayerMask.None;
        foreach (var sample in Rows(mover, left, right)) Assert.Equal(Fixed64.Zero, sample.Row.Normal.X);
    }

    [Fact]
    public void SharedPlaneVertexSupport_ShouldKeepOnePhysicalRegion()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two), Rectangle(scene, 0, 4, -2, 0, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var tetra = new LSMeshCollider(new Vector3d[] { new(0, 0, -1), new(-1, -1, 1), new(1, -1, 1), new(0, 1, 1) },
            new[] { 0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3 }, MeshColliderMode.Convex, MeshInertiaPolicy.RequireClosedVolume);
        scene.CreateBody(tetra, Vector3d.Forward * (3 * Fixed64.Quarter), FixedQuaternion.Identity);
        var regions = new HashSet<int>();
        foreach (var sample in Rows(tetra, members))
        {
            Assert.Equal(Fixed64.Quarter, sample.Row.Depth); Assert.Equal(Vector3d.Backward, sample.Row.Normal);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d point)); Assert.Equal(Vector3d.Zero, point);
            regions.Add(sample.Region);
        }
        Assert.Single(regions);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void SharedUpperRim_ShouldKeepOneConnectedNonfaceRegion(bool hull, bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var left = Rectangle(scene, -4, 0, -2, 0, Fixed64.Two);
        var right = Rectangle(scene, 0, 4, -2, 0, Fixed64.Two);
        var mover = Mover(scene, hull, new(Fixed64.Zero, Fixed64.Two + Fixed64.FromFraction(3, 10), Fixed64.FromFraction(3, 10)),
            FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)45, Fixed64.Zero, Fixed64.Zero), new(3, 1, 1));
        var ordinary = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, left, ordinary));
        Fixed64 depth = ordinary.PrimaryContact.Depth;
        using var surface = scene.Context.Physics2D.CreateStaticSurface(reverse ? new[] { right, left } : new[] { left, right });
        var regions = new HashSet<int>(); var suppliers = new HashSet<int>();
        foreach (var sample in Rows(mover, left, right))
        {
            Assert.True(sample.Row.Normal.Y < Fixed64.Zero && sample.Row.Normal.Z < Fixed64.Zero);
            Assert.Equal(Fixed64.Zero, sample.Row.Normal.X); Assert.Equal(depth, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d point));
            Assert.Equal(Fixed64.Two, point.Y); Assert.Equal(Fixed64.Zero, point.Z);
            Assert.InRange(point.X.m_rawValue, (-3 * Fixed64.Half).m_rawValue, (3 * Fixed64.Half).m_rawValue);
            regions.Add(sample.Region); suppliers.Add(sample.Supplier);
        }
        Assert.Single(regions); Assert.Contains(left.Id + 1, suppliers); Assert.Contains(right.Id + 1, suppliers);
    }

    [Fact]
    public void CompoundBoxes_ShouldRetainLeafNamespacesAndMaterials()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two), Rectangle(scene, 0, 4, -2, 0, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Cuboid(Vector3d.One, Vector3d.Left * Fixed64.Quarter, PhysicsMaterial.Frictionless),
            CompoundColliderPart.Cuboid(Vector3d.One, Vector3d.Right * Fixed64.Quarter, PhysicsMaterial.Bouncy));
        scene.CreateBody(compound, Vector3d.Forward * Fixed64.Quarter, FixedQuaternion.Identity);
        var manifold = new ContactManifold(); bool first = false, second = false;
        var expectedRows = new List<(int Member, ContactGroupKey Key, ManifoldContact Row)>();
        foreach (var member in members)
        {
            Assert.True(CollisionDetectionMixed.TryBuildDiscreteManifold(compound, member, manifold));
            for (int group = 0; group < manifold.GroupCount; group++)
            {
                ref var rows = ref manifold.GetGroup(group);
                Assert.True(rows.Key.NamespaceA is 1 or 2);
                first |= rows.Key.NamespaceA == 1; second |= rows.Key.NamespaceA == 2;
                PhysicsMaterial expected = rows.Key.NamespaceA == 1 ? PhysicsMaterial.Frictionless : PhysicsMaterial.Bouncy;
                for (int index = 0; index < rows.Count; index++)
                {
                    var row = rows[index];
                    Assert.Equal(expected, row.MaterialA); Assert.Equal(PhysicsMaterial.Frictionless, row.MaterialB);
                    Assert.Equal(rows.Key.NamespaceA, row.FeatureNamespaceA);
                    expectedRows.Add((member.Id, rows.Key, row));
                }
            }
        }
        Assert.True(first && second);
        var repeated = new List<(int Member, ContactGroupKey Key, ManifoldContact Row)>();
        foreach (var member in members)
        {
            Assert.True(CollisionDetectionMixed.TryBuildDiscreteManifold(compound, member, manifold));
            for (int group = 0; group < manifold.GroupCount; group++)
            {
                ref var rows = ref manifold.GetGroup(group);
                for (int index = 0; index < rows.Count; index++) repeated.Add((member.Id, rows.Key, rows[index]));
            }
        }
        Assert.Equal(expectedRows, repeated);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void LaterPlanarCallback_ShouldRetireChangedCarriersAndSuppliersBeforeResponse(bool retireSupplier, bool filter)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var left = Rectangle(scene, -4, 0, -2, 0, Fixed64.Two);
        var right = Rectangle(scene, 0, 4, -2, 0, Fixed64.Two);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right });
        var observer = new LSCircleCollider2D(Fixed64.Two) { IsTrigger = true, MixedHalfThicknessOverride = Fixed64.Two };
        observer.InitializeWithNoBody(new TestMatterAgent(scene.Context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        var mover = Mover(scene, false, new(Fixed64.Zero, Fixed64.Zero, Fixed64.Quarter), FixedQuaternion.Identity);
        SolidBody body = mover.Body!;
        body.FreezeAxes = BodyFreezeAxes3D.PositionX | BodyFreezeAxes3D.PositionY | BodyFreezeAxes3D.Rotation;
        body.SleepEnabled = false; body.UseManualGrounding(); body.AddLinearImpulse(Vector3d.Backward);
        var retired = retireSupplier ? right : left; var surviving = retireSupplier ? left : right;
        int retiredId = retired.Id + 1, entered = 0;
        left.OnMixedContactEnter += _ => entered++; right.OnMixedContactEnter += _ => entered++;
        bool changed = false;
        observer.OnMixedTriggerEnter += _ =>
        {
            Assert.Equal(2, entered);
            var carrier = NativePair(scene.Context, mover, left);
            Assert.NotEmpty(carrier.Manifold); Assert.Empty(NativePair(scene.Context, mover, right).Manifold);
            bool borrowed = false;
            for (int group = 0; group < carrier.Manifold.GroupCount; group++)
                borrowed |= carrier.Manifold.GetGroup(group).Key.SourceIdB == right.Id + 1;
            Assert.True(borrowed);
            if (filter) retired.IgnoredCollisionLayers = PhysicsLayerMask.All;
            else retired.Deactivate();
            changed = true;
        };
        scene.Context.Simulate(); scene.Context.LateSimulate();
        Assert.True(changed);
        var response = NativePair(scene.Context, mover, surviving);
        Assert.NotEmpty(response.Manifold);
        for (int group = 0; group < response.Manifold.GroupCount; group++)
            Assert.NotEqual(retiredId, response.Manifold.GetGroup(group).Key.SourceIdB);
        Assert.Equal(Fixed64.Zero, body.LinearVelocity.Z); Assert.True(body.Position3d.Z > Fixed64.Quarter);
    }

    [Fact]
    public void RetainedDisposedDeclaration_ShouldReleaseItsPreparedColliders()
    {
        var retained = RetiredNativeDeclaration();
        GC.Collect(); GC.WaitForPendingFinalizers(); GC.Collect();
        Assert.False(retained.Mover.IsAlive, "The ended moving collider remains retained.");
        Assert.False(retained.Member.IsAlive, "The ended surface member remains retained.");
        GC.KeepAlive(retained.Surface);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static (StaticSurface2D Surface, WeakReference Mover, WeakReference Member) RetiredNativeDeclaration()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two), Rectangle(scene, 0, 4, -2, 0, Fixed64.Two) };
        var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var mover = new LSCuboidCollider { Size = Vector3d.One };
        var body = new SolidBody(new TestMatterAgent(scene.Context), mover) { Mass = Fixed64.One };
        // Keep this check about declaration ownership; query lifetime has its own regressions.
        body.UseManualGrounding();
        body.Initialize(Vector3d.Forward * Fixed64.Quarter, FixedQuaternion.Identity);
        Assert.NotEmpty(Rows(mover, members));
        mover.Body!.Deactivate();
        foreach (var member in members) member.Deactivate();
        surface.Dispose();
        return (surface, new(mover), new(members[1]));
    }

    private static CollisionPairMixed NativePair(GravitasWorldContext context, LSCollider mover, LSCollider2D member)
    {
        var pairs = (SwiftDictionary<ulong, CollisionPairMixed>)typeof(GravitasMixedCollisionService)
            .GetField("_pairs", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(context.MixedCollisions)!;
        Assert.True(pairs.TryGetValue(MixedColliderKey.CreateKey(mover.Id, member.Id), out var pair));
        return pair!;
    }
}

