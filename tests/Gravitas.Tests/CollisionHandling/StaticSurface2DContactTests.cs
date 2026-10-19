using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Support;
using Gravitas.Tests.Support;
using System.Linq;
using Xunit;

namespace Gravitas.Tests;

public sealed class StaticSurface2DContactTests
{
    [Theory]
    [InlineData(0)] [InlineData(1)] [InlineData(2)] [InlineData(3)] [InlineData(4)]
    public void CachedContacts_ShouldFollowMemberIdentityMoverPoseGeometryAndMaterial(int change)
    {
        using var context = Physics2DTestWorld.CreateContext();
        LSCollider2D left = Rectangle(context, new(8, 2), new(-4, -1));
        LSCollider2D right = Rectangle(context, new(8, 2), new(4, -1));
        using var surface = context.Physics2D.CreateStaticSurface(new[] { left, right });
        var capsule = new LSCapsuleCollider2D(Fixed64.Half, Fixed64.Two);
        var body = new SolidBody2D(new TestMatterAgent(context), capsule) { Mass = Fixed64.One };
        body.Initialize(new((Fixed64)(change == 0 ? -3 : -2), Fixed64.Quarter));
        Assert.Equal(Fixed64.One - Fixed64.Quarter, Collide(capsule, left).PrimaryContact.Depth);
        if (change == 0)
        {
            body.ResetPosition(new((Fixed64)3, Fixed64.Quarter));
            var rows = Collide(capsule, right);
            for (int g = 0; g < rows.GroupCount; g++) Assert.Equal(right.Id + 1, rows.GetGroup(g).Key.SourceIdB);
        }
        if (change == 1)
        {
            body.ResetPosition(body.Position, Fixed64.HalfPi);
            Assert.Equal(Fixed64.Quarter, Collide(capsule, left).PrimaryContact.Depth);
        }
        if (change == 2)
        {
            capsule.Height = (Fixed64)3;
            capsule.Simulate();
            Assert.Equal(Fixed64.One + Fixed64.Quarter, Collide(capsule, left).PrimaryContact.Depth);
        }
        if (change == 3)
        {
            capsule.Material = PhysicsMaterial.Bouncy;
            Assert.All(Collide(capsule, left), row => Assert.Equal(PhysicsMaterial.Bouncy, row.MaterialA));
        }
        if (change == 4)
        {
            var other = new LSCapsuleCollider2D(Fixed64.Half, Fixed64.Two) { Material = PhysicsMaterial.Bouncy };
            var otherBody = new SolidBody2D(new TestMatterAgent(context), other) { Mass = Fixed64.One };
            otherBody.Initialize(body.Position);
            Assert.Equal(Fixed64.One - Fixed64.Quarter, Collide(other, left).PrimaryContact.Depth);
            Assert.All(Collide(other, left), row => Assert.Equal(PhysicsMaterial.Bouncy, row.MaterialA));
        }
    }

    [Theory]
    [InlineData(0, false)] [InlineData(1, false)] [InlineData(2, false)]
    [InlineData(0, true)] [InlineData(1, true)] [InlineData(2, true)]
    public void FilledConcaveCorner_ShouldRetainIndependentFiniteNormalsAndSupplierMaterials(int shape, bool reverse)
    {
        using var context = Physics2DTestWorld.CreateContext();
        LSCollider2D floor = Rectangle(context, new(8, 4), new(0, -2));
        LSCollider2D wall = Rectangle(context, new(4, 4), new(-2, 2));
        floor.Material = PhysicsMaterial.Frictionless; wall.Material = PhysicsMaterial.Bouncy;
        using var surface = context.Physics2D.CreateStaticSurface(new[] { wall, floor });
        LSCollider2D mover = shape == 0 ? new LSCircleCollider2D(Fixed64.Half)
            : shape == 1 ? new LSCapsuleCollider2D(Fixed64.Half, Fixed64.Two)
            : new LSPolygonCollider2D(new(-Fixed64.Half, -Fixed64.One), new(Fixed64.Half, -Fixed64.One),
                new(Fixed64.Half, Fixed64.One), new(-Fixed64.Half, Fixed64.One));
        var body = new SolidBody2D(new TestMatterAgent(context), mover) { Mass = Fixed64.One };
        body.Initialize(new(Fixed64.Quarter, shape == 0 ? Fixed64.Quarter : Fixed64.One - Fixed64.Quarter));
        var manifold = Collide(reverse ? floor : mover, reverse ? mover : floor);
        Assert.True(manifold.GroupCount >= 2);
        Assert.Contains(manifold, row => row.Normal == (reverse ? Vector2d.Forward : -Vector2d.Forward));
        Assert.Contains(manifold, row => row.Normal == (reverse ? Vector2d.Right : -Vector2d.Right));
        foreach (var row in manifold)
        {
            PhysicsMaterial supplier = reverse ? row.MaterialA : row.MaterialB;
            Assert.Equal(row.Normal.Y != Fixed64.Zero ? floor.Material : wall.Material, supplier);
            Assert.Equal(Fixed64.Quarter, row.Depth);
        }
        Assert.Empty(Collide(mover, wall));
    }

    [Theory]
    [InlineData(0)] [InlineData(1)] [InlineData(2)]
    public void ExposingAJoinedRim_ShouldRebuildRowsAtAnUnchangedMoverPose(int change)
    {
        using var context = Physics2DTestWorld.CreateContext();
        LSCollider2D left = Rectangle(context, new(8, 2), new(-4, -1));
        LSCollider2D right = Rectangle(context, new(8, 2), new(4, -1));
        using var surface = context.Physics2D.CreateStaticSurface(new[] { left, right });
        var mover = new LSCircleCollider2D(Fixed64.Half);
        var body = new SolidBody2D(new TestMatterAgent(context), mover) { Mass = Fixed64.One };
        body.Initialize(new(Fixed64.Quarter, Fixed64.Quarter));
        Assert.All(Collide(mover, left), row => Assert.Equal(Fixed64.Zero, row.Normal.X));
        if (change == 0) right.IsTrigger = true;
        else if (change == 1) right.IgnoredCollisionLayers = PhysicsLayerMask.All;
        else surface.ReplaceMembers(new[] { left });
        Assert.Contains(Collide(mover, left), row => row.Normal.X != Fixed64.Zero);
    }

    [Fact]
    public void RationallyClippedPolygonFeature_ShouldRegenerateTheCoveredEndpoint()
    {
        using var context = Physics2DTestWorld.CreateContext();
        LSCollider2D floor = Rectangle(context, new(8, 4), new(0, -2));
        LSCollider2D wall = Rectangle(context, new(4, 4), new(-2, 2));
        using var surface = context.Physics2D.CreateStaticSurface(new[] { floor, wall });
        var mover = new LSPolygonCollider2D(new(-Fixed64.Half, -Fixed64.Half), new(Fixed64.Half, -Fixed64.Half),
            new(Fixed64.Half, Fixed64.Half), new(-Fixed64.Half, Fixed64.Half));
        var body = new SolidBody2D(new TestMatterAgent(context), mover) { Mass = Fixed64.One };
        body.Initialize(new(Fixed64.Quarter, Fixed64.Quarter));
        var manifold = Collide(mover, floor);
        var floorRows = manifold.Where(row => row.Normal == -Vector2d.Forward).ToArray();
        Assert.Contains(floorRows, row => row.AnchorB.TryGetWorldPoint(out var point) && point == Vector2d.Zero);
        Assert.DoesNotContain(floorRows, row => row.AnchorB.TryGetWorldPoint(out var point) && point.X < Fixed64.Zero);
    }

    [Fact]
    public void ConnectedBoundaryOutsideTheMover_ShouldNotInventWitnessesOnItsExtendedPlane()
    {
        using var context = Physics2DTestWorld.CreateContext();
        LSCollider2D left = Rectangle(context, new(1, 2), new(-Fixed64.Half, -Fixed64.One));
        LSCollider2D middle = Rectangle(context, new(Fixed64.Quarter, (Fixed64)2), new(Fixed64.One / (Fixed64)8, -Fixed64.One));
        LSCollider2D right = Rectangle(context, new(Fixed64.Quarter, (Fixed64)2), new((Fixed64)3 / (Fixed64)8, -Fixed64.One));
        using var surface = context.Physics2D.CreateStaticSurface(new[] { left, middle, right });
        var mover = new LSCircleCollider2D(Fixed64.Half);
        var body = new SolidBody2D(new TestMatterAgent(context), mover) { Mass = Fixed64.One };
        body.Initialize(new(-Fixed64.Half, Fixed64.Quarter));
        var rows = Collide(mover, left);
        Assert.NotEmpty(rows);
        for (int g = 0; g < rows.GroupCount; g++) Assert.Equal(left.Id + 1, rows.GetGroup(g).Key.SourceIdB);
        Assert.All(rows, row => Assert.Equal(-Vector2d.Forward, row.Normal));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConnectedFaceWithoutItsOwnPlaneCrossing_ShouldRetainItsFiniteBorrowedWitness(bool reverse)
    {
        using var context = Physics2DTestWorld.CreateContext();
        LSCollider2D left = Rectangle(context, new(4, 2), new(-2, -1));
        LSCollider2D right = Rectangle(context, new(4, 2), new(2, -1));
        left.Material = PhysicsMaterial.Frictionless; right.Material = PhysicsMaterial.Bouncy;
        using var surface = context.Physics2D.CreateStaticSurface(reverse ? new[] { right, left } : new[] { left, right });
        var mover = new LSCircleCollider2D(Fixed64.Half);
        var body = new SolidBody2D(new TestMatterAgent(context), mover) { Mass = Fixed64.One };
        body.Initialize(new(-Fixed64.Half, -Fixed64.Quarter));
        var rows = Collide(reverse ? left : mover, reverse ? mover : left);
        bool borrowed = false;
        Fixed64 deepest = Fixed64.Zero;
        for (int g = 0; g < rows.GroupCount; g++)
        {
            ref ContactGroup2D group = ref rows.GetGroup(g);
            bool suppliedByRight = (reverse ? group.Key.SourceIdA : group.Key.SourceIdB) == right.Id + 1;
            for (int p = 0; p < group.Count; p++)
            {
                ManifoldContact2D row = group[p];
                Assert.Equal(reverse ? Vector2d.Forward : -Vector2d.Forward, row.Normal);
                Assert.True((reverse ? row.AnchorA : row.AnchorB).TryGetWorldPoint(out Vector2d target));
                Assert.True((reverse ? row.AnchorB : row.AnchorA).TryGetWorldPoint(out Vector2d moving));
                Assert.True((suppliedByRight ? right : left).ContainsPoint(target));
                Assert.True(mover.ContainsPoint(moving));
                deepest = FixedMath.Max(deepest, row.Depth);
                if (!suppliedByRight) continue;
                borrowed = true;
                Assert.Equal(Vector2d.Zero, target);
                Assert.Equal(new Vector2d(Fixed64.Zero, -Fixed64.Quarter), moving);
                Assert.Equal(Fixed64.Quarter, row.Depth);
                Assert.Equal(PhysicsMaterial.Bouncy, reverse ? row.MaterialA : row.MaterialB);
            }
        }
        Assert.True(borrowed);
        Assert.Equal(Fixed64.One - Fixed64.Quarter, deepest);
        body.SetPosition(body.Position + Vector2d.Forward * (deepest + Fixed64.FromRaw(1)));
        Assert.False(CollisionDetection2D.TryCollide(mover, left, out _));
        Assert.False(CollisionDetection2D.TryCollide(mover, right, out _));
    }

    private static ContactManifold2D Collide(LSCollider2D a, LSCollider2D b)
    {
        var result = new ContactManifold2D();
        Assert.True(CollisionDetection2D.TryCollide(new CollisionWorkItem2D(a, b,
            ColliderSettings2D.GetCollisionType(a.Shape, b.Shape)), result, 0));
        return result;
    }

    private static LSCollider2D Rectangle(GravitasWorldContext context, Vector2d size, Vector2d center)
    {
        var collider = new LSAABBoxCollider2D(size);
        collider.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(new Vector3d(center.X, Fixed64.Zero, center.Y), FixedQuaternion.Identity, Vector3d.One)));
        return collider;
    }
}
