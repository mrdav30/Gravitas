using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests;

public sealed class JoinedStaticSurfaceMeshTests
{
    [Fact]
    public void OpposingHoleRims_ShouldRetainIndependentDirectedConstraints()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var source = Patch(scene, new Fixed64[] { (Fixed64)(-3), (Fixed64)(-1), Fixed64.One, (Fixed64)3 },
            new Fixed64[] { (Fixed64)(-3), (Fixed64)(-2), (Fixed64)3 }, (column, row) => row == 0 || column != 1);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { source });
        var mover = ScaledCube(new Vector3d(4, 10, 2));
        Fixed64 height = Fixed64.FromFraction(3, 10);
        scene.CreateBody(mover, new Vector3d(Fixed64.Zero, height, Fixed64.Zero), FixedQuaternion.Identity);
        var pair = scene.CreatePair(source, mover);
        Assert.True(CollisionDetection.DoCollisionCheck(pair)); Assert.NotEmpty(pair.Manifold);
        bool sourceA = pair.ColliderA == source;
        var directions = new HashSet<Vector3d>(); var regions = new HashSet<int>();
        for (int index = 0; index < pair.Manifold.GroupCount; index++)
        {
            ref var group = ref pair.Manifold.GetGroup(index);
            for (int point = 0; point < group.Count; point++)
            {
                var row = group[point]; Vector3d normal = sourceA ? row.Normal : -row.Normal;
                Assert.False(row.DepthIsClamped);
                Assert.True((sourceA ? row.AnchorA : row.AnchorB).TryGetWorldPoint(out var anchor));
                Assert.True((sourceA ? row.AnchorB : row.AnchorA).TryGetWorldPoint(out var moving));
                Assert.Equal(Fixed64.Zero, anchor.Y);
                string detail = $"U contact normal {normal}, depth {row.Depth}, source {anchor}, moving {moving}.";
                Assert.True(normal == Vector3d.Up || normal == Vector3d.Down || normal == Vector3d.Right
                    || normal == Vector3d.Left || normal == Vector3d.Forward || normal == Vector3d.Backward, detail);
                Assert.True(anchor.X >= (Fixed64)(-3) && anchor.X <= (Fixed64)3
                    && anchor.Z >= (Fixed64)(-3) && anchor.Z <= (Fixed64)3
                    && (anchor.Z <= (Fixed64)(-2) || anchor.X.Abs() >= Fixed64.One), detail);
                if (normal.X != Fixed64.Zero)
                {
                    Assert.True(anchor.X.Abs() == (Fixed64)3
                        || anchor.X.Abs() == Fixed64.One && anchor.Z >= (Fixed64)(-2), detail);
                    Assert.Equal(-normal.X * Fixed64.Two, moving.X);
                    if (row.Depth == Fixed64.One)
                    {
                        Assert.Equal(-normal.X, anchor.X);
                        directions.Add(normal); regions.Add(group.Key.Region);
                    }
                }
                else if (normal.Y != Fixed64.Zero)
                {
                    Assert.Equal((Fixed64)5 - normal.Y * height, row.Depth);
                    Assert.Equal(height - normal.Y * (Fixed64)5, moving.Y);
                }
                else
                {
                    Assert.True(anchor.Z.Abs() == (Fixed64)3
                        || anchor.Z == (Fixed64)(-2) && anchor.X.Abs() <= Fixed64.One, detail);
                    Assert.Equal(-normal.Z, moving.Z);
                }
                for (int coordinate = 0; coordinate < 3; coordinate++)
                {
                    Fixed64 delta = anchor[coordinate] - moving[coordinate];
                    if (normal[coordinate] != Fixed64.Zero)
                        Assert.Equal(normal[coordinate] * row.Depth, delta);
                    else
                        // Two half-raw barycentric weight errors plus product
                        // rounding give <= max triangle edge extent + 1 raw;
                        // the largest source/body edge extent here is 10.
                        Assert.InRange(delta.m_rawValue, -11L, 11L);
                    Assert.InRange(moving[coordinate].m_rawValue, mover.BoundsMin[coordinate].m_rawValue - 11,
                        mover.BoundsMax[coordinate].m_rawValue + 11);
                }
            }
        }
        Assert.Equal(2, directions.Count); Assert.Equal(2, regions.Count);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void SteppedRim_ShouldRetainDistinctParallelLocalDepths(bool mirrorX, bool mirrorZ)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var vertices = new[] { new Vector3d(-3, 0, -2), new(-1, 0, -2), new(-3, 0, 0), new(-1, 0, 0),
            new(0, 0, 0), new(-3, 0, 2), new(-1, 0, 2), new(0, 0, 2) };
        for (int index = 0; index < vertices.Length; index++)
        {
            if (mirrorX) vertices[index].X = -vertices[index].X;
            if (mirrorZ) vertices[index].Z = -vertices[index].Z;
        }
        var source = new LSMeshCollider(vertices, new[] { 0, 2, 1, 1, 2, 3, 2, 5, 3, 3, 5, 6, 3, 6, 4, 4, 6, 7 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.InitializeStaticCollider(source, Vector3d.Zero);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { source });
        // Tall/deep support exposes two x-rim depths while native interior
        // triangles may also retain genuine plane contacts.
        var mover = ScaledCube(new Vector3d(4, 10, 10));
        Fixed64 height = Fixed64.FromFraction(3, 10);
        scene.CreateBody(mover, new Vector3d(Fixed64.Zero, height, Fixed64.Zero), FixedQuaternion.Identity);
        var pair = scene.CreatePair(source, mover);
        Assert.True(CollisionDetection.DoCollisionCheck(pair)); Assert.NotEmpty(pair.Manifold);
        bool sourceA = pair.ColliderA == source;
        Vector3d expected = mirrorX ? Vector3d.Left : Vector3d.Right;
        var depths = new HashSet<Fixed64>(); var regions = new HashSet<int>();
        for (int index = 0; index < pair.Manifold.GroupCount; index++)
        {
            ref var group = ref pair.Manifold.GetGroup(index);
            for (int point = 0; point < group.Count; point++)
            {
                var row = group[point];
                Vector3d normal = sourceA ? row.Normal : -row.Normal;
                Assert.False(row.DepthIsClamped);
                Assert.True((sourceA ? row.AnchorA : row.AnchorB).TryGetWorldPoint(out var anchor));
                Assert.True((sourceA ? row.AnchorB : row.AnchorA).TryGetWorldPoint(out var moving));
                Assert.Equal(Fixed64.Zero, anchor.Y);
                Fixed64 side = mirrorZ ? -anchor.Z : anchor.Z;
                if (normal == Vector3d.Up || normal == Vector3d.Down)
                {
                    Assert.Equal((Fixed64)5 - normal.Y * height, row.Depth);
                    Assert.Equal(height - normal.Y * (Fixed64)5, moving.Y);
                    Fixed64 x = mirrorX ? -anchor.X : anchor.X;
                    Assert.InRange(side.m_rawValue, ((Fixed64)(-2)).m_rawValue, Fixed64.Two.m_rawValue);
                    Assert.InRange(x.m_rawValue, ((Fixed64)(-2)).m_rawValue, Fixed64.Zero.m_rawValue);
                    Assert.True(side >= Fixed64.Zero || x <= -Fixed64.One,
                        "a plane witness must stay inside the actual stepped domain");
                }
                else
                {
                    Assert.True(normal == expected,
                        $"Unexpected stepped-rim normal {normal}, depth {row.Depth}, source {anchor}.");
                    Assert.True(row.Depth == Fixed64.One || row.Depth == Fixed64.Two);
                    depths.Add(row.Depth); regions.Add(group.Key.Region);
                    Assert.Equal(expected.X * (row.Depth - Fixed64.Two), anchor.X);
                    Assert.InRange(side.m_rawValue, (row.Depth == Fixed64.One ? (Fixed64)(-2) : Fixed64.Zero).m_rawValue,
                        (row.Depth == Fixed64.One ? Fixed64.Zero : Fixed64.Two).m_rawValue);
                }
                for (int coordinate = 0; coordinate < 3; coordinate++)
                {
                    Fixed64 delta = anchor[coordinate] - moving[coordinate];
                    if (normal[coordinate] != Fixed64.Zero)
                        Assert.Equal(normal[coordinate] * row.Depth, delta);
                    else
                        // Native closest-point barycentric weights each round
                        // by <= half a raw unit. Edge extent <=10 plus product
                        // rounding bounds the tangential residual by 11 raw.
                        Assert.InRange(delta.m_rawValue, -11L, 11L);
                }
            }
        }
        Assert.Equal(2, depths.Count); Assert.Equal(2, regions.Count);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AlternatingOwners_ShouldJoinOnlyTheReciprocalSupportWindows(bool bridgeFirst)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        // A | B | A | C: B touches two owners in one collider; the second A
        // owner also touches a different collider on its opposite side.
        var outer = new LSMeshCollider(new[] { new Vector3d(-4, 0, -2), new(-2, 0, -2),
            new(-4, 0, 2), new(-2, 0, 2), new(0, 0, -2), new(2, 0, -2), new(0, 0, 2), new(2, 0, 2) },
            new[] { 0, 2, 1, 1, 2, 3, 4, 6, 5, 5, 6, 7 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        if (!bridgeFirst) scene.InitializeStaticCollider(outer, Vector3d.Zero);
        var bridge = Patch(scene, new Fixed64[] { (Fixed64)(-2), Fixed64.Zero }, new Fixed64[] { (Fixed64)(-2), (Fixed64)2 });
        if (bridgeFirst) scene.InitializeStaticCollider(outer, Vector3d.Zero);
        var end = Patch(scene, new Fixed64[] { (Fixed64)2, (Fixed64)4 }, new Fixed64[] { (Fixed64)(-2), (Fixed64)2 });
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { outer, bridge, end });
        var mover = ScaledCube(new Vector3d(9, 1, 3));
        scene.CreateBody(mover, Vector3d.Up * Fixed64.Quarter, FixedQuaternion.Identity);
        Assert.Equal(new Vector3d(9, 1, 3), mover.BoundsMax - mover.BoundsMin);
        var regions = new HashSet<int>();
        int contacts = 0;
        foreach (var member in new[] { outer, bridge, end })
        {
            var pair = scene.CreatePair(member, mover);
            Assert.True(CollisionDetection.DoCollisionCheck(pair));
            bool sourceA = pair.ColliderA == member;
            for (int index = 0; index < pair.Manifold.GroupCount; index++)
            {
                ref var group = ref pair.Manifold.GetGroup(index);
                regions.Add(group.Key.Region);
                int supplier = sourceA ? group.Key.SourceIdA : group.Key.SourceIdB;
                Assert.True(supplier == outer.Id + 1 || supplier == bridge.Id + 1 || supplier == end.Id + 1);
                for (int point = 0; point < group.Count; point++)
                {
                    var row = group[point]; contacts++;
                    Assert.Equal(Vector3d.Up, sourceA ? row.Normal : -row.Normal);
                    Assert.Equal(Fixed64.Quarter, row.Depth); Assert.False(row.DepthIsClamped);
                    Assert.True((sourceA ? row.AnchorA : row.AnchorB).TryGetWorldPoint(out var source));
                    Assert.Equal(Fixed64.Zero, source.Y);
                    Assert.InRange(source.Z.m_rawValue, (-3 * Fixed64.Half).m_rawValue, (3 * Fixed64.Half).m_rawValue);
                    if (supplier == outer.Id + 1)
                        Assert.True(source.X >= (Fixed64)(-4) && source.X <= (Fixed64)(-2)
                            || source.X >= Fixed64.Zero && source.X <= Fixed64.Two);
                    else if (supplier == bridge.Id + 1)
                        Assert.InRange(source.X.m_rawValue, ((Fixed64)(-2)).m_rawValue, Fixed64.Zero.m_rawValue);
                    else
                        Assert.InRange(source.X.m_rawValue, Fixed64.Two.m_rawValue, ((Fixed64)4).m_rawValue);
                }
            }
        }
        Assert.Equal(4, contacts);
        // Material/provenance groups may differ, but their finite support is
        // one connected region across all four canonical owners.
        Assert.Single(regions);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void PartialJoin_ShouldRetainTheActualExposedRim(bool crossesCut, bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Patch(scene, new Fixed64[] { (Fixed64)(-4), (Fixed64)0 }, new Fixed64[] { (Fixed64)(-4), (Fixed64)4 });
        var right = Patch(scene, new Fixed64[] { (Fixed64)0, (Fixed64)4 }, new Fixed64[] { (Fixed64)(-2), (Fixed64)2 });
        using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { right, left } : new[] { left, right });
        var mover = MeshTestFixtures.CreateConvexCube();
        Vector3d start = new(Fixed64.FromFraction(2, 5), Fixed64.FromFraction(3, 10),
            crossesCut ? Fixed64.Two + Fixed64.FromFraction(3, 10) : (Fixed64)3);
        var body = scene.CreateBody(mover, start, FixedQuaternion.Identity).Body;
        var rows = Contacts(scene, mover, reverse, left, right);
        Assert.NotEmpty(rows);
        Fixed64 exit = Fixed64.Zero;
        bool found = false;
        foreach (var row in rows)
        {
            if (row.Normal != Vector3d.Right) continue;
            found = true; exit = FixedMath.Max(exit, row.Depth);
            Assert.Equal(Fixed64.Half - start.X, row.Depth);
            Assert.Equal(Fixed64.Zero, row.Source.X);
            Assert.True(row.Source.Z >= Fixed64.Two);
            Assert.Equal(left.Id + 1, row.Supplier);
        }
        Assert.True(found, "the exposed x=0 rim above the partial join remains physical");
        surface.Dispose();
        body.SetPosition(start + Vector3d.Right * (exit + Fixed64.FromRaw(2))); mover.RebuildRuntimeShapeOnly();
        Assert.False(CollisionDetection.DoCollisionCheck(scene.CreatePair(left, mover)));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Hole_ShouldRemainEmptyAndRetainOnlyItsActualFilledSide(bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Fixed64[] z = { (Fixed64)(-2), -Fixed64.Half, Fixed64.Half, (Fixed64)2 };
        var left = Patch(scene, new Fixed64[] { (Fixed64)(-2), -Fixed64.Half, (Fixed64)0 }, z, (column, row) => row != 1 || column != 1);
        var right = Patch(scene, new Fixed64[] { (Fixed64)0, Fixed64.Half, (Fixed64)2 }, z, (column, row) => row != 1 || column != 0);
        using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { right, left } : new[] { left, right });
        var mover = ScaledCube(Vector3d.One * Fixed64.Half);
        var body = scene.CreateBody(mover, Vector3d.Up * (Fixed64.One / 8), FixedQuaternion.Identity).Body;
        Assert.Equal(Vector3d.One * Fixed64.Half, mover.BoundsMax - mover.BoundsMin);
        foreach (var member in new[] { left, right })
        {
            var pair = scene.CreatePair(member, mover);
            Assert.False(CollisionDetection.DoCollisionCheck(pair)); Assert.Empty(pair.Manifold);
        }
        body.SetPosition(new Vector3d(Fixed64.Half, Fixed64.One / 8, Fixed64.Zero)); mover.RebuildRuntimeShapeOnly();
        var rows = Contacts(scene, mover, reverse, left, right);
        Assert.NotEmpty(rows);
        foreach (var row in rows)
        {
            Assert.Equal(Fixed64.Zero, row.Source.Y);
            Assert.True(row.Source.X.Abs() >= Fixed64.Half || row.Source.Z.Abs() >= Fixed64.Half,
                "support witnesses must lie on the filled ring, including its genuine hole rim");
            Assert.True(row.Depth > Fixed64.Zero);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RemotelyConnectedPatches_ShouldRetainBothDisconnectedSupportBands(bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Fixed64[] z = { (Fixed64)(-2), -Fixed64.Quarter, Fixed64.Quarter, (Fixed64)2 };
        var left = Patch(scene, new Fixed64[] { (Fixed64)(-2), (Fixed64)(-1), (Fixed64)0 }, z, (column, row) => row != 1 || column != 1);
        var right = Patch(scene, new Fixed64[] { (Fixed64)0, (Fixed64)1, (Fixed64)2 }, z, (column, row) => row != 1 || column != 0);
        using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { right, left } : new[] { left, right });
        var mover = MeshTestFixtures.CreateConvexCube();
        scene.CreateBody(mover, Vector3d.Up * (3 * Fixed64.One / 8), FixedQuaternion.Identity);
        var rows = Contacts(scene, mover, reverse, left, right);
        Assert.Equal(8, rows.Count);
        bool lower = false, upper = false;
        foreach (var row in rows)
        {
            Assert.Equal(Vector3d.Up, row.Normal); Assert.Equal(Fixed64.One / 8, row.Depth);
            Assert.Equal(Fixed64.Zero, row.Source.Y);
            Assert.True(row.Source.Z.Abs() >= Fixed64.Quarter, "a remote spine cannot fill the gap under the hull");
            lower |= row.Source.Z < Fixed64.Zero; upper |= row.Source.Z > Fixed64.Zero;
        }
        Assert.True(lower && upper, "both finite support bands must contribute retained levers");
        var regions = new HashSet<int>();
        foreach (var member in new[] { left, right })
        {
            var pair = reverse ? scene.CreatePair(mover, member) : scene.CreatePair(member, mover);
            CollisionDetection.DoCollisionCheck(pair);
            for (int index = 0; index < pair.Manifold.GroupCount; index++)
            {
                ref var group = ref pair.Manifold.GetGroup(index);
                regions.Add(group.Key.Region);
                int band = 0;
                for (int pointIndex = 0; pointIndex < group.Count; pointIndex++)
                {
                    var row = group[pointIndex];
                    ContactAnchor source = pair.ColliderA == member ? row.AnchorA : row.AnchorB;
                    Assert.True(source.TryGetWorldPoint(out var point));
                    int sign = point.Z < Fixed64.Zero ? -1 : 1;
                    Assert.True(band == 0 || band == sign, "one response region cannot bridge the empty middle of the two U-shaped members");
                    band = sign;
                }
            }
        }
        Assert.Equal(2, regions.Count);
    }

    [Theory]
    [InlineData(30)]
    [InlineData(45)]
    public void RotatedHullExit_ShouldClearTheCompleteShapeAgainstBothFiniteMembers(int angle)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Patch(scene, new Fixed64[] { (Fixed64)(-8), (Fixed64)0 }, new Fixed64[] { (Fixed64)(-4), (Fixed64)4 });
        var right = Patch(scene, new Fixed64[] { (Fixed64)0, (Fixed64)8 }, new Fixed64[] { (Fixed64)(-4), (Fixed64)4 });
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { right, left });
        var mover = MeshTestFixtures.CreateConvexCube();
        Vector3d start = Vector3d.Up * Fixed64.Quarter;
        var body = scene.CreateBody(mover, start,
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)angle)).Body;
        var rows = Contacts(scene, mover, false, left, right);
        Assert.NotEmpty(rows);
        Fixed64 depth = Fixed64.Zero;
        foreach (var row in rows)
        {
            Assert.Equal(Vector3d.Up, row.Normal);
            Assert.Equal(Fixed64.Zero, row.Source.Y);
            depth = FixedMath.Max(depth, row.Depth);
        }
        Fixed64 minimumY = Fixed64.MaxValue;
        foreach (Vector3d vertex in mover.Mesh.ScaledLocalVertices)
        {
            Assert.True(new FixedPointAnchor(mover.Mesh.Origin, mover.Mesh.Rotation, vertex).TryGetPoint(out var point));
            minimumY = FixedMath.Min(minimumY, point.Y);
        }
        Assert.InRange(depth.m_rawValue, (-minimumY).m_rawValue - 1, (-minimumY).m_rawValue + 1);
        // Disable continuity for the independent native complete-shape oracle.
        surface.Dispose();
        body.SetPosition(start + Vector3d.Up * (depth + Fixed64.FromRaw(4))); mover.RebuildRuntimeShapeOnly();
        foreach (var member in new[] { left, right })
            Assert.False(CollisionDetection.DoCollisionCheck(scene.CreatePair(member, mover)));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void IndependentMemberFrames_ShouldRetainSupplierMaterialsAcrossPairAndDeclarationOrder(bool reverseDeclaration, bool reversePair)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Patch(scene, new Fixed64[] { (Fixed64)(-4), (Fixed64)0 }, new Fixed64[] { (Fixed64)(-4), (Fixed64)4 });
        var right = new LSMeshCollider(new[] { new Vector3d(-4, 0, 0), new(4, 0, 0), new(-4, 0, 4), new(4, 0, 4) },
            new[] { 0, 2, 1, 1, 2, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        FixedQuaternion rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.One).Normalized;
        right.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(Vector3d.Zero, rotation, Vector3d.One)));
        left.Material = PhysicsMaterial.Frictionless; right.Material = PhysicsMaterial.Bouncy;
        using var surface = scene.Context.Physics.CreateStaticSurface(reverseDeclaration ? new[] { right, left } : new[] { left, right });
        var mover = MeshTestFixtures.CreateConvexCube();
        scene.CreateBody(mover, new Vector3d(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity);
        var rows = Contacts(scene, mover, reversePair, left, right);
        Assert.NotEmpty(rows);
        bool suppliedLeft = false, suppliedRight = false;
        foreach (var row in rows)
        {
            bool fromLeft = row.Supplier == left.Id + 1;
            Assert.True(fromLeft || row.Supplier == right.Id + 1);
            suppliedLeft |= fromLeft; suppliedRight |= !fromLeft;
            LSMeshCollider supplier = fromLeft ? left : right;
            Assert.Equal(supplier.Material, row.Material); Assert.Equal(supplier.Mesh.Origin, row.Origin);
            Assert.Equal(supplier.Mesh.Rotation, row.Rotation);
            Assert.Equal(Vector3d.Up, row.Normal); Assert.Equal(Fixed64.Quarter, row.Depth);
            Assert.Equal(Fixed64.Zero, row.Source.Y);
            Assert.True(fromLeft ? row.Source.X <= Fixed64.Zero : row.Source.X >= Fixed64.Zero);
        }
        Assert.True(suppliedLeft && suppliedRight);
    }

    [Fact]
    public void EnclosedClosedConcaveMember_ShouldStillOverlapTheConvexVolume()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var enclosed = MeshTestFixtures.CreateConvexCube(MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.InitializeStaticCollider(enclosed, Vector3d.Zero);
        var neighbor = Patch(scene, new Fixed64[] { Fixed64.Half, (Fixed64)4 }, new Fixed64[] { -Fixed64.Half, Fixed64.Half }, height: Fixed64.Half);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { enclosed, neighbor });
        var mover = ScaledCube(Vector3d.One * (Fixed64)4);
        scene.CreateBody(mover, Vector3d.Zero, FixedQuaternion.Identity);
        Assert.Equal(new Vector3d(-2, -2, -2), mover.BoundsMin);
        Assert.Equal(new Vector3d(2, 2, 2), mover.BoundsMax);
        var rows = Contacts(scene, mover, false, enclosed, neighbor);
        bool found = false;
        foreach (var row in rows)
        {
            if (row.Supplier != enclosed.Id + 1) continue;
            found = true;
            Assert.True(row.Depth > Fixed64.Zero);
            Assert.Equal(Fixed64.Half, FixedMath.Max(row.Source.X.Abs(), FixedMath.Max(row.Source.Y.Abs(), row.Source.Z.Abs())));
        }
        Assert.True(found, "a closed concave surface wholly inside the hull must not disappear through boundary-only admission");
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ProjectedSupplier_ShouldBlockTheHullThroughItsCarrierWithoutFalseEvents(bool obliqueFloor, bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Environment.Gravity = scene.Context.Environment.AirDensity = scene.Context.Environment.DampingFactor = Fixed64.Zero;
        Vector3d floorNormal = new Vector3d(-1, 1, 0).Normalized;
        Fixed64[] z = { (Fixed64)(-4), (Fixed64)4 };
        var supplier = obliqueFloor
            ? Patch(scene, new Fixed64[] { Fixed64.FromFraction(13, 20), (Fixed64)4 }, z, sloped: true)
            : Patch(scene, new Fixed64[] { -Fixed64.FromFraction(3, 2), -Fixed64.FromFraction(36, 25) }, z);
        var carrier = obliqueFloor
            ? Patch(scene, new Fixed64[] { (Fixed64)(-4), Fixed64.FromFraction(13, 20) }, z, sloped: true)
            : Patch(scene, new Fixed64[] { -Fixed64.FromFraction(36, 25), (Fixed64)2 }, z);
        supplier.Material = carrier.Material = PhysicsMaterial.Frictionless;
        using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { carrier, supplier } : new[] { supplier, carrier });
        int supplierEnters = 0, carrierEnters = 0;
        supplier.OnContactEnter += _ => supplierEnters++; carrier.OnContactEnter += _ => carrierEnters++;
        var mover = ScaledCube(new Vector3d(4, 1, 2));
        mover.Material = PhysicsMaterial.Frictionless;
        Vector3d start = obliqueFloor ? floorNormal * Fixed64.FromFraction(1, 10) : Vector3d.Up * Fixed64.Quarter;
        var body = scene.CreateBody(mover, start, obliqueFloor ? FixedQuaternion.Identity
            : FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)30)).Body;
        body.UseManualGrounding();
        if (obliqueFloor) Assert.Equal(new Vector3d(4, 1, 2), mover.BoundsMax - mover.BoundsMin);
        else
        {
            Assert.True(mover.BoundsMax.X - mover.BoundsMin.X > (Fixed64)3);
            Assert.True(mover.BoundsMax.Y - mover.BoundsMin.Y > Fixed64.Two);
        }
        if (obliqueFloor) Assert.True(supplier.BoundsMin.Y > mover.BoundsMax.Y);
        Assert.False(CollisionDetection.DoCollisionCheck(scene.CreatePair(supplier, mover)),
            "projected support does not turn its initially separated supplier into a collision event owner");
        scene.Context.Simulate(); scene.Context.LateSimulate();
        Assert.Equal(0, supplierEnters); Assert.Equal(1, carrierEnters);
        Vector3d normal = obliqueFloor ? floorNormal : Vector3d.Up;
        Assert.True(Vector3d.Dot(body.Position3d - start, normal) > Fixed64.Half);
        var pair = scene.Context.Physics.GetCollisionPair(carrier.Id, mover.Id);
        Assert.NotNull(pair); Assert.NotEmpty(pair.Manifold);
        bool supplierA = pair.ColliderA == carrier;
        foreach (var row in pair.Manifold)
        {
            Assert.Equal(PhysicsMaterial.Frictionless, supplierA ? row.MaterialA : row.MaterialB);
            ContactAnchor anchor = supplierA ? row.AnchorA : row.AnchorB;
            Assert.Equal(supplier.Mesh.Origin, anchor.Origin);
            Assert.Equal(supplier.Mesh.Rotation, anchor.Rotation);
            Assert.True(anchor.TryGetWorldPoint(out var source));
            Assert.True(obliqueFloor ? source.Y > Fixed64.Half : source.X < -Fixed64.FromFraction(36, 25));
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CoplanarOpenTriangles_ShouldRetainTheirNativeInPlaneSeparation(bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var first = new LSMeshCollider(new[] { new Vector3d(0, 0, 0), new(4, 0, 0), new(0, 0, 4) },
            new[] { 0, 2, 1 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        var neighbor = new LSMeshCollider(new[] { new Vector3d(0, 0, 0), new(0, 0, -4), new(4, 0, 0) },
            new[] { 0, 2, 1 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.InitializeStaticCollider(first, Vector3d.Zero); scene.InitializeStaticCollider(neighbor, Vector3d.Zero);
        using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { neighbor, first } : new[] { first, neighbor });
        var mover = new LSMeshCollider(new[] { new Vector3d(3, 0, 3), new(5, 0, 3), new(3, 0, 5) },
            new[] { 0, 2, 1 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.CreateBody(mover, Vector3d.Zero, FixedQuaternion.Identity);
        Assert.True(mover.BoundsMin.X < first.BoundsMax.X && mover.BoundsMin.Z < first.BoundsMax.Z);
        // Face normals and pairwise edge crosses are all vertical here. Only
        // the native normal-cross-own-edge axes prove this coplanar gap.
        foreach (var member in new[] { first, neighbor })
        {
            var pair = reverse ? scene.CreatePair(mover, member) : scene.CreatePair(member, mover);
            Assert.False(CollisionDetection.DoCollisionCheck(pair)); Assert.Empty(pair.Manifold);
        }
    }

    [Theory]
    [InlineData(MeshColliderMode.Concave, false, false)]
    [InlineData(MeshColliderMode.Concave, false, true)]
    [InlineData(MeshColliderMode.Concave, true, false)]
    [InlineData(MeshColliderMode.Concave, true, true)]
    [InlineData(MeshColliderMode.Convex, false, false)]
    [InlineData(MeshColliderMode.Convex, false, true)]
    [InlineData(MeshColliderMode.Convex, true, false)]
    [InlineData(MeshColliderMode.Convex, true, true)]
    public void OpenSheetAcrossJoinedSeam_ShouldRetainItsFiniteVerticalExit(MeshColliderMode mode, bool alternate, bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Patch(scene, new Fixed64[] { (Fixed64)(-4), Fixed64.Zero }, new Fixed64[] { (Fixed64)(-4), (Fixed64)4 });
        var right = Patch(scene, new Fixed64[] { Fixed64.Zero, (Fixed64)4 }, new Fixed64[] { (Fixed64)(-4), (Fixed64)4 });
        using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { right, left } : new[] { left, right });
        var mover = new LSMeshCollider(new[] { new Vector3d(-Fixed64.Half, -Fixed64.Quarter, Fixed64.Zero),
            new(Fixed64.Half, -Fixed64.Quarter, Fixed64.Zero), new(-Fixed64.Half, 3 * Fixed64.One / 4, Fixed64.Zero),
            new(Fixed64.Half, 3 * Fixed64.One / 4, Fixed64.Zero) },
            alternate ? new[] { 0, 1, 3, 0, 3, 2 } : new[] { 0, 1, 2, 1, 3, 2 },
            mode, MeshInertiaPolicy.SurfaceApproximation);
        Vector3d start = Vector3d.Right * Fixed64.Quarter;
        var body = scene.CreateBody(mover, start, FixedQuaternion.Identity).Body;
        var rows = Contacts(scene, mover, reverse, left, right);
        Assert.NotEmpty(rows);
        foreach (var row in rows)
        {
            Assert.Equal(Vector3d.Up, row.Normal); Assert.Equal(Fixed64.Quarter, row.Depth);
            Assert.Equal(Fixed64.Zero, row.Source.Y); Assert.Equal(Fixed64.Zero, row.Source.Z);
            Assert.InRange(row.Source.X.m_rawValue, (-Fixed64.Quarter).m_rawValue, (3 * Fixed64.One / 4).m_rawValue);
        }
        surface.Dispose();
        body.SetPosition(start + Vector3d.Up * (Fixed64.Quarter + Fixed64.FromRaw(2))); mover.RebuildRuntimeShapeOnly();
        foreach (var member in new[] { left, right })
            Assert.False(CollisionDetection.DoCollisionCheck(scene.CreatePair(member, mover)));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConcaveCubeFreeRotation_ShouldMatchItsAlternateFaceTriangulationThroughTheHostLoop(bool reverse)
    {
        var original = Trace(false);
        var alternate = Trace(true);
        Assert.Equal(original, alternate);

        List<(Vector3d Position, FixedQuaternion Rotation, Vector3d Linear, Vector3d Angular)> Trace(bool changeDiagonals)
        {
            using var scene = PhysicsScenarioBuilder.Create();
            scene.Context.SetFrameRate(16); scene.Context.Environment.Gravity = Fixed64.One;
            scene.Context.Environment.AirDensity = scene.Context.Environment.DampingFactor = scene.Context.Environment.MinSpeed = Fixed64.Zero;
            var left = Patch(scene, new Fixed64[] { (Fixed64)(-8), Fixed64.Zero }, new Fixed64[] { (Fixed64)(-4), (Fixed64)4 });
            var right = Patch(scene, new Fixed64[] { Fixed64.Zero, (Fixed64)8 }, new Fixed64[] { (Fixed64)(-4), (Fixed64)4 });
            using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { right, left } : new[] { left, right });
            var cube = MeshTestFixtures.CreateConvexCube(MeshColliderMode.Concave, MeshInertiaPolicy.RequireClosedVolume);
            if (changeDiagonals)
            {
                int[] indices = cube.Mesh.Triangles.ToArray();
                for (int face = 0; face < indices.Length; face += 6)
                {
                    int a = indices[face], b = indices[face + 1], c = indices[face + 2], opposite = -1;
                    for (int vertex = face + 3; vertex < face + 6; vertex++)
                        if (indices[vertex] != a && indices[vertex] != b && indices[vertex] != c) opposite = indices[vertex];
                    Assert.True(opposite >= 0);
                    indices[face] = a; indices[face + 1] = b; indices[face + 2] = opposite;
                    indices[face + 3] = a; indices[face + 4] = opposite; indices[face + 5] = c;
                }
                cube = new(cube.Mesh.LocalVertices.ToArray(), indices, MeshColliderMode.Concave, MeshInertiaPolicy.RequireClosedVolume);
            }
            cube.Material = PhysicsMaterial.Frictionless;
            FixedQuaternion initial = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)15);
            var body = scene.CreateBody(cube, new Vector3d(-3 * Fixed64.One / 4, Fixed64.Half, Fixed64.Zero), initial).Body;
            body.UseManualGrounding(); body.SleepEnabled = false; body.AddLinearImpulse(Vector3d.Right);
            var trace = new List<(Vector3d, FixedQuaternion, Vector3d, Vector3d)>();
            bool rotated = false;
            for (int frame = 0; frame < 48; frame++)
            {
                scene.Context.Simulate(); scene.Context.LateSimulate();
                rotated |= body.Rotation != initial;
                Assert.Equal(Fixed64.One, body.LinearVelocity.X); Assert.Equal(Fixed64.Zero, body.LinearVelocity.Z);
                trace.Add((body.Position3d, body.Rotation, body.LinearVelocity, body.AngularVelocity));
            }
            Assert.True(rotated); Assert.True(body.Position3d.X > Fixed64.One);
            return trace;
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void OffsetCompoundMeshLeaf_ShouldRetainItsOwnPoseAndBothSupplierMaterials(bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Patch(scene, new Fixed64[] { (Fixed64)(-4), Fixed64.Zero }, new Fixed64[] { (Fixed64)(-4), (Fixed64)4 });
        var right = Patch(scene, new Fixed64[] { Fixed64.Zero, (Fixed64)4 }, new Fixed64[] { (Fixed64)(-4), (Fixed64)4 });
        left.Material = PhysicsMaterial.Default; right.Material = PhysicsMaterial.Bouncy;
        using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { right, left } : new[] { left, right });
        Vector3d offset = new(Fixed64.Two, Fixed64.Half, Fixed64.One);
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero);
        var part = new CompoundColliderPart(MeshTestFixtures.CreateConvexCubeDefinition(), offset,
            rotation, Vector3d.One, PhysicsMaterial.Frictionless);
        var compound = new LSCompoundCollider(CompoundColliderPart.Sphere(Fixed64.Quarter, new Vector3d(-2, 3, 0)), part);
        scene.CreateBody(compound, new Vector3d(-7 * Fixed64.One / 4, -Fixed64.Quarter, -Fixed64.One), FixedQuaternion.Identity);
        var pair = reverse ? scene.CreatePair(compound, left) : scene.CreatePair(left, compound);
        Assert.True(CollisionDetection.DoCollisionCheck(pair)); Assert.Same(compound, pair.ColliderA);
        bool fromLeft = false, fromRight = false;
        for (int groupIndex = 0; groupIndex < pair.Manifold.GroupCount; groupIndex++)
        {
            ref var group = ref pair.Manifold.GetGroup(groupIndex);
            Assert.Equal(2, group.Key.NamespaceA);
            bool suppliedLeft = group.Key.SourceIdB == left.Id + 1;
            Assert.True(suppliedLeft || group.Key.SourceIdB == right.Id + 1);
            fromLeft |= suppliedLeft; fromRight |= !suppliedLeft;
            for (int index = 0; index < group.Count; index++)
            {
                var row = group[index];
                Assert.Equal(PhysicsMaterial.Frictionless, row.MaterialA);
                Assert.Equal(suppliedLeft ? left.Material : right.Material, row.MaterialB);
                Assert.Equal(Vector3d.Down, row.Normal); Assert.Equal(Fixed64.Quarter, row.Depth);
                Assert.Equal(new Vector3d(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Zero), row.AnchorA.Origin);
                Assert.Equal(rotation, row.AnchorA.Rotation);
                Assert.True(row.AnchorA.TryGetWorldPoint(out var moving)); Assert.Equal(-Fixed64.Quarter, moving.Y);
                Assert.True(row.AnchorB.TryGetWorldPoint(out var source)); Assert.Equal(Fixed64.Zero, source.Y);
            }
        }
        Assert.True(fromLeft && fromRight);
    }

    private static List<Contact> Contacts(PhysicsScenarioBuilder scene, LSMeshCollider mover, bool reverse,
        params LSMeshCollider[] members)
    {
        var rows = new List<Contact>();
        foreach (var member in members)
        {
            var pair = reverse ? scene.CreatePair(mover, member) : scene.CreatePair(member, mover);
            if (!CollisionDetection.DoCollisionCheck(pair)) { Assert.Empty(pair.Manifold); continue; }
            bool sourceA = pair.ColliderA == member;
            for (int index = 0; index < pair.Manifold.GroupCount; index++)
            {
                ref var group = ref pair.Manifold.GetGroup(index);
                int supplier = sourceA ? group.Key.SourceIdA : group.Key.SourceIdB;
                for (int point = 0; point < group.Count; point++)
                {
                    var row = group[point];
                    ContactAnchor source = sourceA ? row.AnchorA : row.AnchorB;
                    ContactAnchor exit = sourceA ? row.AnchorB : row.AnchorA;
                    Assert.True(source.TryGetWorldPoint(out var first)); Assert.True(exit.TryGetWorldPoint(out var second));
                    Assert.False(row.DepthIsClamped);
                    rows.Add(new(first, second, sourceA ? row.Normal : -row.Normal, row.Depth, supplier,
                        sourceA ? row.MaterialA : row.MaterialB, source.Origin, source.Rotation));
                }
            }
        }
        return rows;
    }

    private static LSMeshCollider ScaledCube(Vector3d scale)
    {
        var unit = MeshTestFixtures.CreateConvexCube();
        Vector3d[] vertices = unit.Mesh.LocalVertices.ToArray();
        for (int index = 0; index < vertices.Length; index++) vertices[index] *= scale;
        return new(vertices, unit.Mesh.Triangles.ToArray(), MeshColliderMode.Convex, MeshInertiaPolicy.RequireClosedVolume);
    }

    private static LSMeshCollider Patch(PhysicsScenarioBuilder scene, Fixed64[] x, Fixed64[] z,
        Func<int, int, bool>? keep = null, Fixed64 height = default, bool sloped = false)
    {
        var vertices = new Vector3d[x.Length * z.Length]; var triangles = new List<int>();
        for (int row = 0; row < z.Length; row++)
            for (int column = 0; column < x.Length; column++)
            {
                int vertex = row * x.Length + column; vertices[vertex] = new(x[column], sloped ? x[column] : height, z[row]);
                if (row == z.Length - 1 || column == x.Length - 1 || keep != null && !keep(column, row)) continue;
                triangles.AddRange(new[] { vertex, vertex + x.Length, vertex + 1, vertex + 1, vertex + x.Length, vertex + x.Length + 1 });
            }
        var mesh = new LSMeshCollider(vertices, triangles.ToArray(), MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.InitializeStaticCollider(mesh, Vector3d.Zero); return mesh;
    }

    private readonly record struct Contact(Vector3d Source, Vector3d Exit, Vector3d Normal, Fixed64 Depth,
        int Supplier, PhysicsMaterial Material, Vector3d Origin, FixedQuaternion Rotation);
}
