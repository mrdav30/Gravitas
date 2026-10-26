using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests;

/// <content>Checks finite boundary ambiguity, separated heights and polygon changes.</content>
public sealed partial class JoinedStaticSolidMixedTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DistinctCoplanarTips_ShouldNotMergeTheirVerticalSupportRegions(bool hull)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var left = new LSPolygonCollider2D(new(-2, 0), new(-4, -2), new(-2, -2))
            { MixedHalfThicknessOverride = Fixed64.Two, Material = PhysicsMaterial.Frictionless };
        var right = new LSPolygonCollider2D(new(2, 0), new(2, -2), new(4, -2))
            { MixedHalfThicknessOverride = Fixed64.Two, Material = PhysicsMaterial.Frictionless };
        left.InitializeWithNoBody(new TestMatterAgent(scene.Context)); right.InitializeWithNoBody(new TestMatterAgent(scene.Context));
        LSCollider2D rear = Rectangle(scene, -4, 4, -3, -2, Fixed64.Two);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right, rear });
        LSCollider mover = Mover(scene, hull, Vector3d.Forward * Fixed64.Quarter, FixedQuaternion.Identity, new(7, 1, 1));
        var rows = Rows(mover, left, right); Assert.NotEmpty(rows);
        var regions = new HashSet<int>(); var suppliers = new HashSet<int>();
        foreach (var sample in rows)
        {
            Assert.Equal(Vector3d.Backward, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(sample.Supplier == left.Id + 1 ? -Fixed64.Two : Fixed64.Two, source.X);
            Assert.Equal(Fixed64.Zero, source.Z);
            Assert.InRange(source.Y.m_rawValue, (-Fixed64.Half).m_rawValue, Fixed64.Half.m_rawValue);
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
    public void SplitCornerWithDisjointRetainedWindows_ShouldKeepItsTwoVerticalRegions(bool hull, bool unequalOriginalHeights)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var first = new LSPolygonCollider2D(new(-2, -2), new(0, -2), new(0, 0))
            { MixedHalfThicknessOverride = unequalOriginalHeights ? Fixed64.One : Fixed64.Two,
                Material = PhysicsMaterial.Frictionless };
        var second = new LSPolygonCollider2D(new(-2, -2), new(0, 0), new(-2, 0))
            { MixedHalfThicknessOverride = unequalOriginalHeights ? 3 * Fixed64.Half : Fixed64.Two,
                Material = PhysicsMaterial.Frictionless };
        first.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(
            unequalOriginalHeights ? Vector3d.Down : Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        second.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(
            unequalOriginalHeights ? Vector3d.Up * Fixed64.Half : Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        Fixed64 coveredHeight = unequalOriginalHeights ? Fixed64.One : Fixed64.Half;
        LSCollider2D right = Rectangle(scene, 0, 2, -2, 0, coveredHeight);
        LSCollider2D front = Rectangle(scene, -2, 0, 0, 2, coveredHeight);
        Fixed64 offset = Fixed64.FromFraction(3, 10);
        LSCollider mover = Mover(scene, hull, new(offset, Fixed64.Zero, offset),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)45, Fixed64.Zero), new(3, 6, 3));
        var ordinary = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, first, ordinary));
        Fixed64 depth = ordinary.PrimaryContact.Depth;
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new LSCollider2D[] { first, second, right, front });
        var regions = new HashSet<int>(); bool lower = false, upper = false;
        foreach (var sample in Rows(mover, first, second))
        {
            if (sample.Row.Normal.X >= Fixed64.Zero || sample.Row.Normal.Z >= Fixed64.Zero) continue;
            Assert.Equal(Fixed64.Zero, sample.Row.Normal.Y); Assert.Equal(depth, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(Fixed64.Zero, source.X); Assert.Equal(Fixed64.Zero, source.Z);
            Assert.True(source.Y < -coveredHeight || source.Y > coveredHeight);
            Assert.InRange(source.Y.m_rawValue, (-Fixed64.Two).m_rawValue, Fixed64.Two.m_rawValue);
            lower |= source.Y < -coveredHeight; upper |= source.Y > coveredHeight;
            if (unequalOriginalHeights)
                Assert.Equal(source.Y < Fixed64.Zero ? first.Id + 1 : second.Id + 1, sample.Supplier);
            regions.Add(sample.Region);
        }
        // The original slabs overlap. Their retained exposed windows do not;
        // the unequal case also lies outside the other original vertical edge.
        Assert.Equal(2, regions.Count); Assert.True(lower && upper);
    }

    [Fact]
    public void CoplanarSteppedRimsWithDifferentEndpoints_ShouldKeepSeparateRegions()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two), Rectangle(scene, 0, 4, -1, 1, Fixed64.One) };
        LSMeshCollider cube = MeshTestFixtures.CreateConvexCube();
        Vector3d[] vertices = cube.Mesh.LocalVertices.ToArray();
        for (int vertex = 0; vertex < vertices.Length; vertex++)
        {
            Vector3d point = vertices[vertex];
            vertices[vertex] = new(8 * point.X, 3 * (point.Y - point.Z), 3 * (point.Y + point.Z));
        }
        var diamond = new LSMeshCollider(vertices, cube.Mesh.Triangles.ToArray(),
            MeshColliderMode.Convex, MeshInertiaPolicy.RequireClosedVolume);
        scene.CreateBody(diamond, new(Fixed64.Zero, Fixed64.FromFraction(23, 10), Fixed64.FromFraction(13, 10)), FixedQuaternion.Identity);
        var ordinary = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(diamond, members[0], ordinary));
        Fixed64 depth = ordinary.PrimaryContact.Depth;
        Assert.True(ordinary.PrimaryContact.Normal.Y < Fixed64.Zero && ordinary.PrimaryContact.Normal.Z < Fixed64.Zero);
        Assert.Equal(ordinary.PrimaryContact.Normal.Y, ordinary.PrimaryContact.Normal.Z);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var rows = Rows(diamond, members); Assert.NotEmpty(rows);
        var regions = new HashSet<int>(); var suppliers = new HashSet<int>();
        foreach (var sample in rows)
        {
            Assert.Equal(depth, sample.Row.Depth); Assert.Equal(Fixed64.Zero, sample.Row.Normal.X);
            Assert.True(sample.Row.Normal.Y < Fixed64.Zero); Assert.Equal(sample.Row.Normal.Y, sample.Row.Normal.Z);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            bool left = sample.Supplier == members[0].Id + 1;
            Assert.Equal(left ? Fixed64.Two : Fixed64.One, source.Y);
            Assert.Equal(left ? Fixed64.Zero : Fixed64.One, source.Z);
            Assert.InRange(source.X.m_rawValue, (left ? -(Fixed64)4 : Fixed64.Zero).m_rawValue,
                (left ? Fixed64.Zero : (Fixed64)4).m_rawValue);
            regions.Add(sample.Region); suppliers.Add(sample.Supplier);
        }
        Assert.Equal(2, regions.Count);
        Assert.Contains(members[0].Id + 1, suppliers); Assert.Contains(members[1].Id + 1, suppliers);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CoincidentTipsWithDisjointHeights_ShouldKeepTheirSeparateVerticalRegions(bool hull)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var lower = new LSPolygonCollider2D(new(0, 0), new(-2, -2), new(0, -2))
            { MixedHalfThicknessOverride = Fixed64.Half, Material = PhysicsMaterial.Frictionless };
        var upper = new LSPolygonCollider2D(new(0, 0), new(-2, -2), new(0, -2))
            { MixedHalfThicknessOverride = Fixed64.Half, Material = PhysicsMaterial.Frictionless };
        lower.InitializeWithNoBody(new TestMatterAgent(scene.Context,
            new FixedTransform(Vector3d.Down * (3 * Fixed64.Half), FixedQuaternion.Identity, Vector3d.One)));
        upper.InitializeWithNoBody(new TestMatterAgent(scene.Context,
            new FixedTransform(Vector3d.Up * (3 * Fixed64.Half), FixedQuaternion.Identity, Vector3d.One)));
        LSCollider2D rear = Rectangle(scene, -2, 0, -4, -2, Fixed64.Two);
        Fixed64 offset = Fixed64.FromFraction(3, 10);
        LSCollider mover = Mover(scene, hull, new(offset, Fixed64.Zero, offset),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)45, Fixed64.Zero), new(3, 6, 3));
        var ordinary = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, lower, ordinary));
        Fixed64 depth = ordinary.PrimaryContact.Depth;
        Assert.True(ordinary.PrimaryContact.Normal.X < Fixed64.Zero && ordinary.PrimaryContact.Normal.Z < Fixed64.Zero);
        Assert.Equal(Fixed64.Zero, ordinary.PrimaryContact.Normal.Y);
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, upper, ordinary));
        Assert.Equal(depth, ordinary.PrimaryContact.Depth);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { lower, upper, rear });
        var rows = Rows(mover, lower, upper); Assert.NotEmpty(rows);
        var regions = new HashSet<int>(); var suppliers = new HashSet<int>();
        foreach (var sample in rows)
        {
            Assert.Equal(depth, sample.Row.Depth);
            Assert.True(sample.Row.Normal.X < Fixed64.Zero && sample.Row.Normal.Z < Fixed64.Zero);
            Assert.Equal(Fixed64.Zero, sample.Row.Normal.Y);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(Fixed64.Zero, source.X); Assert.Equal(Fixed64.Zero, source.Z);
            if (sample.Supplier == lower.Id + 1)
                Assert.InRange(source.Y.m_rawValue, (-Fixed64.Two).m_rawValue, (-Fixed64.One).m_rawValue);
            else
            {
                Assert.Equal(upper.Id + 1, sample.Supplier);
                Assert.InRange(source.Y.m_rawValue, Fixed64.One.m_rawValue, Fixed64.Two.m_rawValue);
            }
            regions.Add(sample.Region); suppliers.Add(sample.Supplier);
        }
        Assert.Equal(2, regions.Count);
        Assert.Contains(lower.Id + 1, suppliers); Assert.Contains(upper.Id + 1, suppliers);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void EnclosedGridCellWithoutExposedSupport_ShouldBorrowTheActualOuterResponse(bool hull)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new LSCollider2D[9];
        for (int x = 0; x < 3; x++)
            for (int z = 0; z < 3; z++)
                members[3 * x + z] = Rectangle(scene, 2 * x - 3, 2 * x - 1, 2 * z - 3, 2 * z - 1, Fixed64.One);
        LSCollider mover = Mover(scene, hull, Vector3d.Zero,
            FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)17, (Fixed64)31, (Fixed64)23),
            new((Fixed64)6, Fixed64.Quarter, Fixed64.Quarter));
        Assert.True(CollisionDetectionMixed.TryCollide(mover, members[4], out _));
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        bool neighboringSupplier = false;
        foreach (var sample in rows)
        {
            Assert.True(sample.Row.Depth > Fixed64.Zero); Assert.False(sample.Row.DepthIsClamped);
            Assert.NotEqual(Vector3d.Zero, sample.Row.Normal);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out _));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d point));
            LSCollider2D supplier = System.Array.Find(members, member => member.Id + 1 == sample.Supplier)!;
            Assert.NotNull(supplier); Assert.True(supplier.ContainsPoint(new(point.X, point.Z)));
            Assert.InRange(point.Y.m_rawValue, (-Fixed64.One).m_rawValue, Fixed64.One.m_rawValue);
            // The entire grid is one rectangular solid. A response may use its
            // outer planes, but never an interior grid line as an exposed wall.
            Assert.True(point.Y.Abs() == Fixed64.One || point.X.Abs() == (Fixed64)3 || point.Z.Abs() == (Fixed64)3);
            neighboringSupplier |= supplier != members[4];
        }
        Assert.True(neighboringSupplier);
        SolidBody body = mover.Body!;
        body.FreezeAxes = BodyFreezeAxes3D.Rotation; body.SleepEnabled = false; body.UseManualGrounding();
        scene.Context.Simulate(); scene.Context.LateSimulate();
        Assert.NotEqual(Vector3d.Zero, body.Position3d);
    }

    [Fact]
    public void AffineHullOverJoinedCaps_ShouldRetainExactHeightAndWholeShapeClearance()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two), Rectangle(scene, 0, 4, -2, 0, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = Mover(scene, true, new(Fixed64.Zero, 9 * Fixed64.Quarter, -Fixed64.One), FixedQuaternion.Identity);
        // The odd raw X scale leaves half-raw authored vertices, so this
        // exercises affine geometry instead of an exactly compacted rigid cube.
        mover.Body!.Agent.Transform.LocalScale = new(Fixed64.FromFraction(2, 3), 3 * Fixed64.Quarter, 5 * Fixed64.Quarter);
        mover.Simulate();
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.Equal(Vector3d.Down, sample.Row.Normal); Assert.Equal(Fixed64.FromFraction(1, 8), sample.Row.Depth);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(Fixed64.FromFraction(15, 8), moving.Y); Assert.Equal(Fixed64.Two, source.Y);
            Assert.True((moving.X - source.X).Abs() <= Fixed64.MinIncrement);
            Assert.True((moving.Z - source.Z).Abs() <= Fixed64.MinIncrement);
            LSCollider2D supplier = sample.Supplier == members[0].Id + 1 ? members[0] : members[1];
            Assert.True(supplier.ContainsPoint(new(source.X, source.Z)));
        }
        AssertWholeShapeExit(mover, members, new(Fixed64.Zero, Fixed64.FromFraction(19, 8), -Fixed64.One), Vector3d.Up);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SplitCornerWithAnOpenVerticalBand_ShouldKeepOneDeclaredPhysicalRegion(bool hull)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var first = new LSPolygonCollider2D(new(-2, -2), new(0, -2), new(0, 0))
            { MixedHalfThicknessOverride = Fixed64.Two, Material = PhysicsMaterial.Frictionless };
        var second = new LSPolygonCollider2D(new(-2, -2), new(0, 0), new(-2, 0))
            { MixedHalfThicknessOverride = Fixed64.Two, Material = PhysicsMaterial.Frictionless };
        first.InitializeWithNoBody(new TestMatterAgent(scene.Context)); second.InitializeWithNoBody(new TestMatterAgent(scene.Context));
        var lower = Rectangle(scene, 0, 2, -2, 0, 3 * Fixed64.Quarter, -5 * Fixed64.Quarter);
        var upper = Rectangle(scene, 0, 2, -2, 0, 3 * Fixed64.Quarter, 5 * Fixed64.Quarter);
        Fixed64 offset = Fixed64.FromFraction(3, 10);
        LSCollider mover = Mover(scene, hull, new(offset, Fixed64.Zero, offset),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)45, Fixed64.Zero), new(3, 3, 3));
        var ordinary = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, first, ordinary));
        Fixed64 depth = ordinary.PrimaryContact.Depth;
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new LSCollider2D[] { first, second, lower, upper });
        var regions = new HashSet<int>();
        foreach (var sample in Rows(mover, first, second, lower, upper))
        {
            if (sample.Row.Normal.X >= Fixed64.Zero || sample.Row.Normal.Z >= Fixed64.Zero) continue;
            Assert.Equal(Fixed64.Zero, sample.Row.Normal.Y); Assert.Equal(depth, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d point));
            Assert.Equal(Fixed64.Zero, point.X); Assert.Equal(Fixed64.Zero, point.Z);
            Assert.True(point.Y > -Fixed64.Half && point.Y < Fixed64.Half);
            Assert.True(sample.Supplier == first.Id + 1 || sample.Supplier == second.Id + 1);
            regions.Add(sample.Region);
        }
        // Original cap endpoints Y=Â±2 are covered. Continuity belongs to the
        // retained open interval itself, not to those discarded endpoints.
        Assert.Single(regions);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AmbiguousOverlappingRimsOutsideTheirOriginalEndpoints_ShouldStayIndependent(bool hull)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var first = Rectangle(scene, -4, 4, -2, 0, Fixed64.Two);
        var second = Rectangle(scene, -2, 6, -2, 0, Fixed64.Two);
        var rear = Rectangle(scene, -4, 6, -4, -2, Fixed64.Two);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { first, second, rear });
        LSCollider mover = Mover(scene, hull, new(Fixed64.Half, (Fixed64)2 + Fixed64.FromFraction(3, 10), Fixed64.FromFraction(3, 10)),
            FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)45, Fixed64.Zero, Fixed64.Zero), new(1, 3, 3));
        var regions = new HashSet<int>(); var suppliers = new HashSet<int>();
        foreach (var sample in Rows(mover, first, second))
        {
            Assert.True(sample.Row.Normal.Y < Fixed64.Zero && sample.Row.Normal.Z < Fixed64.Zero);
            Assert.Equal(Fixed64.Zero, sample.Row.Normal.X);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d point));
            Assert.Equal(Fixed64.Two, point.Y); Assert.Equal(Fixed64.Zero, point.Z);
            Assert.InRange(point.X.m_rawValue, Fixed64.Zero.m_rawValue, Fixed64.One.m_rawValue);
            regions.Add(sample.Region); suppliers.Add(sample.Supplier);
        }
        Assert.Equal(2, regions.Count);
        Assert.Contains(first.Id + 1, suppliers); Assert.Contains(second.Id + 1, suppliers);
    }

    [Fact]
    public void AmbiguousMiddleOfCapJoin_ShouldKeepCoincidentCapSupportsIndependent()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        LSCollider2D left = Rectangle(scene, -2, 0, -2, 2, Fixed64.Two);
        LSCollider2D right = Rectangle(scene, 0, 2, -2, 2, Fixed64.Two);
        LSCollider2D middle = Rectangle(scene, -1, 1, -1, 1, Fixed64.Two);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right, middle });
        LSMeshCollider cube = MeshTestFixtures.CreateConvexCube();
        Vector3d[] vertices = cube.Mesh.LocalVertices.ToArray();
        for (int vertex = 0; vertex < vertices.Length; vertex++)
        {
            Vector3d point = vertices[vertex];
            vertices[vertex] = new(point.X + point.Z, point.Y, (point.Z - point.X) * Fixed64.Half);
        }
        var rhombus = new LSMeshCollider(vertices, cube.Mesh.Triangles.ToArray(),
            MeshColliderMode.Convex, MeshInertiaPolicy.RequireClosedVolume);
        scene.CreateBody(rhombus, Vector3d.Up * (9 * Fixed64.Quarter), FixedQuaternion.Identity);
        // The bottom diamond crosses the original seam only inside the
        // ambiguous span. The real joins outside Z=[-1,1] cannot connect it.
        // Its vertical sides also keep every lateral exit deeper than the cap.
        var rows = Rows(rhombus, left, right); Assert.NotEmpty(rows);
        var regions = new HashSet<int>(); var suppliers = new HashSet<int>();
        foreach (var sample in rows)
        {
            Assert.Equal(Vector3d.Down, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(7 * Fixed64.Quarter, moving.Y); Assert.Equal(Fixed64.Two, source.Y);
            Assert.True((moving.X - source.X).Abs() <= Fixed64.MinIncrement);
            Assert.True((moving.Z - source.Z).Abs() <= Fixed64.MinIncrement);
            Assert.InRange(source.Z.m_rawValue, (-Fixed64.Half).m_rawValue, Fixed64.Half.m_rawValue);
            if (sample.Supplier == left.Id + 1)
                Assert.InRange(source.X.m_rawValue, (-Fixed64.One).m_rawValue, Fixed64.Zero.m_rawValue);
            else
            {
                Assert.Equal(right.Id + 1, sample.Supplier);
                Assert.InRange(source.X.m_rawValue, Fixed64.Zero.m_rawValue, Fixed64.One.m_rawValue);
            }
            regions.Add(sample.Region); suppliers.Add(sample.Supplier);
        }
        Assert.Equal(2, regions.Count);
        Assert.Contains(left.Id + 1, suppliers); Assert.Contains(right.Id + 1, suppliers);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void UnequalHeightFaceAndTipAtTheirSharedEndpoint_ShouldKeepOnePhysicalRegion(bool hull)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        LSCollider2D face = Rectangle(scene, -2, 0, -2, 0, Fixed64.One, -Fixed64.One);
        var tip = new LSPolygonCollider2D(new(0, 0), new(0, -2), new(2, -2))
            { MixedHalfThicknessOverride = Fixed64.Two, Material = PhysicsMaterial.Frictionless };
        tip.InitializeWithNoBody(new TestMatterAgent(scene.Context));
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { face, tip });
        LSCollider mover = Mover(scene, hull, Vector3d.Forward * Fixed64.Quarter, FixedQuaternion.Identity, new(3, 3, 1));
        var rows = Rows(mover, face, tip); Assert.NotEmpty(rows);
        var regions = new HashSet<int>(); var suppliers = new HashSet<int>();
        foreach (var sample in rows)
        {
            Assert.Equal(Vector3d.Backward, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(Fixed64.Zero, source.Z);
            if (sample.Supplier == face.Id + 1)
            {
                Assert.InRange(source.X.m_rawValue, (-3 * Fixed64.Half).m_rawValue, Fixed64.Zero.m_rawValue);
                Assert.InRange(source.Y.m_rawValue, (-3 * Fixed64.Half).m_rawValue, Fixed64.Zero.m_rawValue);
            }
            else
            {
                Assert.Equal(tip.Id + 1, sample.Supplier); Assert.Equal(Fixed64.Zero, source.X);
                Assert.InRange(source.Y.m_rawValue, (-3 * Fixed64.Half).m_rawValue, (3 * Fixed64.Half).m_rawValue);
            }
            regions.Add(sample.Region); suppliers.Add(sample.Supplier);
        }
        Assert.Single(regions);
        Assert.Contains(face.Id + 1, suppliers); Assert.Contains(tip.Id + 1, suppliers);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CoplanarFaceAndTipWithDisjointHeights_ShouldKeepSeparatePhysicalRegions(bool hull)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        LSCollider2D face = Rectangle(scene, -2, 0, -2, 0, Fixed64.One);
        var tip = new LSPolygonCollider2D(new(0, 0), new(0, -2), new(2, -2))
            { MixedHalfThicknessOverride = Fixed64.One, Material = PhysicsMaterial.Frictionless };
        tip.InitializeWithNoBody(new TestMatterAgent(scene.Context,
            new FixedTransform(new(0, 4, 0), FixedQuaternion.Identity, Vector3d.One)));
        LSCollider2D bridge = Rectangle(scene, -2, 2, -4, -2, (Fixed64)3, Fixed64.Two);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { face, tip, bridge });
        LSCollider mover = Mover(scene, hull, new(Fixed64.Zero, Fixed64.Two, Fixed64.Quarter),
            FixedQuaternion.Identity, new(3, 6, 1));
        // The footprints meet at (0,0), and both pieces join the remote bridge,
        // but the actual front supports are separated by the open Y=(1,3) gap.
        var rows = Rows(mover, face, tip); Assert.NotEmpty(rows);
        var regions = new HashSet<int>(); var suppliers = new HashSet<int>();
        foreach (var sample in rows)
        {
            Assert.Equal(Vector3d.Backward, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(-Fixed64.Quarter, moving.Z); Assert.Equal(Fixed64.Zero, source.Z);
            if (sample.Supplier == face.Id + 1)
            {
                Assert.InRange(source.X.m_rawValue, (-3 * Fixed64.Half).m_rawValue, Fixed64.Zero.m_rawValue);
                Assert.InRange(source.Y.m_rawValue, (-Fixed64.One).m_rawValue, Fixed64.One.m_rawValue);
            }
            else
            {
                Assert.Equal(tip.Id + 1, sample.Supplier); Assert.Equal(Fixed64.Zero, source.X);
                Assert.InRange(source.Y.m_rawValue, ((Fixed64)3).m_rawValue, ((Fixed64)5).m_rawValue);
            }
            regions.Add(sample.Region); suppliers.Add(sample.Supplier);
        }
        Assert.Equal(2, regions.Count);
        Assert.Contains(face.Id + 1, suppliers); Assert.Contains(tip.Id + 1, suppliers);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SixVertexCapAfterScaleChange_ShouldKeepItsFinitePolygonWitnesses(bool hull)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var left = new LSPolygonCollider2D(new(-4, -1), new(-3, -2), new(0, -2), new(0, 2), new(-3, 2), new(-4, 1))
            { MixedHalfThicknessOverride = Fixed64.Two, Material = PhysicsMaterial.Frictionless };
        left.InitializeWithNoBody(new TestMatterAgent(scene.Context));
        LSCollider2D right = Rectangle(scene, 0, 4, -2, 2, Fixed64.Two);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right });
        LSCollider mover = Mover(scene, hull, Vector3d.Up * (9 * Fixed64.Quarter), FixedQuaternion.Identity, new(6, 1, 3));
        for (int stage = 0; stage < 2; stage++)
        {
            if (stage == 1) { left.Agent.Transform.LocalScale = new(Fixed64.One, Fixed64.One, Fixed64.Half); left.Simulate(); }
            var rows = Rows(mover, left, right); Assert.NotEmpty(rows);
            var regions = new HashSet<int>(); var suppliers = new HashSet<int>();
            foreach (var sample in rows)
            {
                Assert.Equal(Vector3d.Down, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
                Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
                Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
                Assert.Equal(7 * Fixed64.Quarter, moving.Y); Assert.Equal(Fixed64.Two, source.Y);
                // Rigid feature materialization can round paired tangential
                // coordinates independently by one raw unit; planes stay exact.
                Assert.True((moving.X - source.X).Abs() <= Fixed64.MinIncrement);
                Assert.True((moving.Z - source.Z).Abs() <= Fixed64.MinIncrement);
                LSCollider2D supplier = sample.Supplier == left.Id + 1 ? left : right;
                Assert.True(supplier.ContainsPoint(new(source.X, source.Z)));
                if (sample.Supplier == left.Id + 1 && stage == 1)
                    Assert.InRange(source.Z.m_rawValue, (-Fixed64.One).m_rawValue, Fixed64.One.m_rawValue);
                regions.Add(sample.Region); suppliers.Add(sample.Supplier);
            }
            Assert.Single(regions);
            Assert.Contains(left.Id + 1, suppliers); Assert.Contains(right.Id + 1, suppliers);
        }
    }
}
