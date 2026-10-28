using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Support;
using Gravitas.Tests.Support;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests;

/// <content>Checks current native curved suppliers across declaration and geometry mutations.</content>
public sealed partial class JoinedStaticSolidMixedTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void CurvedBodylessOrOffcenterCompoundLeaf_ShouldPreserveItsGeometryAndMaterial(bool cone, bool compoundMover)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two), Rectangle(scene, 0, 4, -2, 0, Fixed64.Two) };
        LSCollider2D continuous = Rectangle(scene, -4, 4, -2, 0, Fixed64.Two);
        if (!compoundMover)
            foreach (LSCollider2D member in members)
            {
                Vector2d center = member.Center; member.Deactivate();
                var stationary = new SolidBody2D(new TestMatterAgent(scene.Context), member);
                stationary.Initialize(center, motionType: BodyMotionType.Static);
            }
        LSCollider mover, leaf;
        if (compoundMover)
        {
            ColliderShapeDefinition shape = cone
                ? ColliderShapeDefinition.Cone(Fixed64.Half, Fixed64.Two, PhysicsMaterial.Bouncy)
                : ColliderShapeDefinition.Cylinder(Fixed64.Half, Fixed64.Two, PhysicsMaterial.Bouncy);
            var owner = new LSCompoundCollider(new CompoundColliderPart(shape, Vector3d.Right, FixedQuaternion.Identity, Vector3d.One),
                CompoundColliderPart.Sphere(Fixed64.Half, new(-1, 3, 3)));
            SolidBody body = scene.CreateBody(owner, new(-3 * Fixed64.Quarter, Fixed64.Zero, Fixed64.Quarter), FixedQuaternion.Identity).Body;
            mover = owner; leaf = owner.GetPartCollider(0);
            Assert.NotEqual(leaf.Center, body.WorldCenterOfMass);
        }
        else
        {
            leaf = cone ? new LSConeCollider { Radius = Fixed64.Half, Size = new(1, 2, 1), Material = PhysicsMaterial.Bouncy }
                : new LSCylinderCollider { Radius = Fixed64.Half, Size = new(1, 2, 1), Material = PhysicsMaterial.Bouncy };
            leaf.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(
                new(Fixed64.Quarter, Fixed64.Zero, Fixed64.Quarter), FixedQuaternion.Identity, Vector3d.One)));
            mover = leaf;
        }
        Assert.Null(leaf.Body);
        var ordinary = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, continuous, ordinary));
        Assert.Equal(Vector3d.Backward, ordinary.PrimaryContact.Normal); Assert.Equal(Fixed64.Quarter, ordinary.PrimaryContact.Depth);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.Equal(Vector3d.Backward, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            Assert.Equal(compoundMover ? 1 : 0, sample.Row.FeatureNamespaceA);
            Assert.True(sample.Row.HasMaterialOverride); Assert.Equal(PhysicsMaterial.Bouncy, sample.Row.MaterialA);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(Fixed64.Quarter, source.X); Assert.Equal(Fixed64.Zero, source.Z);
            Assert.Equal(source.X, moving.X); Assert.Equal(source.Y, moving.Y); Assert.Equal(-Fixed64.Quarter, moving.Z);
        }
    }
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CurvedMembershipRemovalAndRejoin_ShouldRestoreTheRealCornerAndThenTheContinuousWall(bool cone)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var left = Rectangle(scene, -4, 0, -2, 0, Fixed64.Two);
        var right = Rectangle(scene, 0, 4, -2, 0, Fixed64.Two);
        LSCollider mover = CurvedMover(scene, cone, new(Fixed64.Quarter, Fixed64.Zero, Fixed64.Quarter));
        var ordinary = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, left, ordinary));
        ManifoldContact corner = ordinary.PrimaryContact;
        Assert.True(corner.Normal.X < Fixed64.Zero && corner.Normal.Z < Fixed64.Zero);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right });
        Assert.Contains(Rows(mover, left, right), sample => sample.Row.Normal == Vector3d.Backward && sample.Row.Depth == Fixed64.Quarter);
        surface.Remove(right);
        var exposed = Rows(mover, left); Assert.NotEmpty(exposed);
        foreach (var sample in exposed)
        { Assert.Equal(corner.Normal, sample.Row.Normal); Assert.Equal(corner.Depth, sample.Row.Depth); Assert.Equal(left.Id + 1, sample.Supplier); }
        surface.ReplaceMembers(new[] { right, left });
        var restored = Rows(mover, left, right); Assert.NotEmpty(restored);
        foreach (var sample in restored)
        { Assert.Equal(Vector3d.Backward, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth); }
    }

    [Theory]
    [InlineData(false, 0)]
    [InlineData(true, 0)]
    [InlineData(false, 1)]
    [InlineData(true, 1)]
    [InlineData(false, 2)]
    [InlineData(true, 2)]
    public void CurvedNeighborEligibilityOrPose_ShouldRetireItsHiddenBoundaryAndRestoreIt(bool cone, int change)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var left = Rectangle(scene, -4, 0, -2, 0, Fixed64.Two);
        var right = Rectangle(scene, 0, 4, -2, 0, Fixed64.Two);
        LSCollider mover = CurvedMover(scene, cone, new(Fixed64.Quarter, Fixed64.Zero, Fixed64.Quarter));
        var ordinary = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, left, ordinary));
        ManifoldContact corner = ordinary.PrimaryContact;
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right });
        Assert.Contains(Rows(mover, left, right), sample => sample.Row.Normal == Vector3d.Backward);
        if (change == 0) right.IgnoredCollisionLayers = PhysicsLayerMask.All;
        if (change == 1) right.IsTrigger = true;
        if (change == 2) { right.LocalOffset = Vector2d.Right * (Fixed64)4; right.Simulate(); }
        var exposed = Rows(mover, left); Assert.NotEmpty(exposed);
        foreach (var sample in exposed)
        { Assert.Equal(corner.Normal, sample.Row.Normal); Assert.Equal(corner.Depth, sample.Row.Depth); Assert.Equal(left.Id + 1, sample.Supplier); }
        right.IgnoredCollisionLayers = PhysicsLayerMask.None; right.IsTrigger = false;
        right.LocalOffset = Vector2d.Zero; right.Simulate();
        var restored = Rows(mover, left, right); Assert.NotEmpty(restored);
        foreach (var sample in restored)
        { Assert.Equal(Vector3d.Backward, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth); }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CurvedMixedThicknessMutation_ShouldExposeAndThenHideTheUpperSharedSide(bool cone)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var left = Rectangle(scene, -4, 0, -2, 0, Fixed64.Two);
        var right = Rectangle(scene, 0, 4, -2, 0, Fixed64.Two);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right });
        LSCollider mover = CurvedMover(scene, cone, new(Fixed64.Quarter, 3 * Fixed64.Half, -Fixed64.One));
        Assert.DoesNotContain(Rows(mover, left, right), sample => sample.Row.Normal == Vector3d.Left);
        right.MixedHalfThicknessOverride = Fixed64.Half; right.Simulate();
        var exposed = Rows(mover, left, right);
        Assert.Contains(exposed, sample => sample.Supplier == left.Id + 1 && sample.Row.Normal == Vector3d.Left
            && sample.Row.Depth == Fixed64.Quarter);
        foreach (var sample in exposed)
            if (sample.Row.Normal == Vector3d.Left)
            {
                Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d point));
                Assert.Equal(Fixed64.Zero, point.X);
                Assert.InRange(point.Y.m_rawValue, Fixed64.Half.m_rawValue, Fixed64.Two.m_rawValue);
            }
        right.MixedHalfThicknessOverride = Fixed64.Two; right.Simulate();
        var restored = Rows(mover, left, right); Assert.NotEmpty(restored);
        Assert.DoesNotContain(restored, sample => sample.Row.Normal == Vector3d.Left);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CurvedFiniteSupplierAndMaterial_ShouldFollowTheCurrentSourceAcrossTheJoin(bool cone)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var left = Rectangle(scene, -4, 0, -2, 0, Fixed64.Two);
        var right = Rectangle(scene, 0, 4, -2, 0, Fixed64.Two); right.Material = PhysicsMaterial.Bouncy;
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right });
        LSCollider mover = CurvedMover(scene, cone, Vector3d.Forward * Fixed64.Quarter);
        for (int pass = 0; pass < 3; pass++)
        {
            Fixed64 x = pass == 1 ? -Fixed64.Quarter : Fixed64.Quarter;
            if (pass == 2) right.Material = PhysicsMaterial.Default;
            SetPosition(mover, new(x, Fixed64.Zero, Fixed64.Quarter));
            LSCollider2D supplier = x < Fixed64.Zero ? left : right;
            var rows = Rows(mover, left, right); Assert.NotEmpty(rows);
            foreach (var sample in rows)
            {
                Assert.Equal(supplier.Id + 1, sample.Supplier); Assert.True(sample.Row.HasMaterialOverride);
                Assert.Equal(mover.Material, sample.Row.MaterialA); Assert.Equal(supplier.Material, sample.Row.MaterialB);
                Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d point));
                Assert.Equal(x, point.X); Assert.Equal(Fixed64.Zero, point.Z);
                Assert.True(supplier.ContainsPoint(new(point.X, point.Z)));
            }
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CurvedRemoteHeightBands_ShouldPreserveTheSameLocalPhysicalContact(bool cone)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var left = Rectangle(scene, -4, 0, -1, 0, Fixed64.Two);
        var right = Rectangle(scene, 0, 4, -1, 0, Fixed64.Two);
        var remote = Rectangle(scene, -4, 4, -3, -1, Fixed64.One);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right });
        LSCollider mover = CurvedMover(scene, cone, new(Fixed64.Quarter, Fixed64.Zero, Fixed64.Quarter));
        var expected = Capture(); Assert.NotEmpty(expected);
        surface.ReplaceMembers(new[] { remote, right, left });
        Assert.Equal(expected, Capture());
        remote.MixedHalfThicknessOverride = (Fixed64)3; remote.Simulate();
        Assert.Equal(expected, Capture());

        List<(Vector3d Normal, Fixed64 Depth, Vector3d Moving, Vector3d Source)> Capture()
        {
            var result = new List<(Vector3d Normal, Fixed64 Depth, Vector3d Moving, Vector3d Source)>();
            foreach (var sample in Rows(mover, left, right, remote))
            {
                Assert.NotEqual(remote.Id + 1, sample.Supplier);
                Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
                Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
                result.Add((sample.Row.Normal, sample.Row.Depth, moving, source));
            }
            result.Sort(static (a, b) =>
            {
                int order = a.Source.X.CompareTo(b.Source.X);
                if (order == 0) order = a.Source.Y.CompareTo(b.Source.Y);
                if (order == 0) order = a.Source.Z.CompareTo(b.Source.Z);
                return order != 0 ? order : a.Depth.CompareTo(b.Depth);
            });
            return result;
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void CurvedMemberRegistrationAndRepresentation_ShouldPreserveThePhysicalRows(bool cone, bool polygon)
    {
        Assert.Equal(Capture(false, false), Capture(true, polygon));

        List<(Vector3d Normal, Fixed64 Depth, Vector3d Moving, Vector3d Source)> Capture(bool reverse, bool polygons)
        {
            using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
            LSCollider2D left, right;
            if (reverse)
            {
                right = Rectangle(scene, 0, 4, -2, 0, Fixed64.Two, polygon: polygons);
                left = Rectangle(scene, -4, 0, -2, 0, Fixed64.Two, polygon: polygons);
            }
            else
            {
                left = Rectangle(scene, -4, 0, -2, 0, Fixed64.Two, polygon: polygons);
                right = Rectangle(scene, 0, 4, -2, 0, Fixed64.Two, polygon: polygons);
            }
            using var surface = scene.Context.Physics2D.CreateStaticSurface(reverse ? new[] { right, left } : new[] { left, right });
            LSCollider mover = CurvedMover(scene, cone, new(Fixed64.Quarter, Fixed64.Zero, Fixed64.Quarter));
            var result = new List<(Vector3d Normal, Fixed64 Depth, Vector3d Moving, Vector3d Source)>();
            foreach (var sample in Rows(mover, left, right))
            {
                Assert.Equal(right.Id + 1, sample.Supplier);
                Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
                Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
                result.Add((sample.Row.Normal, sample.Row.Depth, moving, source));
            }
            Assert.NotEmpty(result); result.Sort(static (a, b) =>
            {
                int order = a.Source.X.CompareTo(b.Source.X);
                if (order == 0) order = a.Source.Y.CompareTo(b.Source.Y);
                if (order == 0) order = a.Source.Z.CompareTo(b.Source.Z);
                return order != 0 ? order : a.Depth.CompareTo(b.Depth);
            });
            return result;
        }
    }
}
