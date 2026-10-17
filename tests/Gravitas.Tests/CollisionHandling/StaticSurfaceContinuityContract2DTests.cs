using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests;

public sealed class StaticSurfaceContinuityContract2DTests
{
    [Theory]
    [InlineData(false, ColliderType2D.Circle, 1, 1)]
    [InlineData(true, ColliderType2D.Circle, 1, 1)]
    [InlineData(false, ColliderType2D.Capsule, 1, 2)]
    [InlineData(true, ColliderType2D.Capsule, 1, 2)]
    [InlineData(false, ColliderType2D.ConvexPolygon, 2, 2)]
    [InlineData(true, ColliderType2D.ConvexPolygon, 2, 2)]
    public void LUnion_FiniteMemberManifolds_ShouldRetainIndependentNormalAndMaterialGroups(
        bool polygonMembers, ColliderType2D movingShape, int bottomCount, int leftCount)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        LSCollider2D bottom = Rectangle(scene.Context, polygonMembers, new(8, 4), new(0, -2));
        LSCollider2D left = Rectangle(scene.Context, polygonMembers, new(4, 4), new(-2, 2));
        bottom.Material = PhysicsMaterial.Frictionless;
        left.Material = PhysicsMaterial.Bouncy;
        Vector2d center = movingShape == ColliderType2D.Circle
            ? new(Fixed64.Quarter, Fixed64.Quarter)
            : new(Fixed64.Quarter, Fixed64.One - Fixed64.Quarter);
        LSCollider2D mover = movingShape == ColliderType2D.Circle ? new LSCircleCollider2D(Fixed64.Half)
            : movingShape == ColliderType2D.Capsule ? new LSCapsuleCollider2D(Fixed64.Half, Fixed64.Two)
            : new LSPolygonCollider2D(new(-Fixed64.Half, -Fixed64.One), new(Fixed64.Half, -Fixed64.One),
                new(Fixed64.Half, Fixed64.One), new(-Fixed64.Half, Fixed64.One));
        mover.InitializeWithNoBody(new TestMatterAgent(scene.Context,
            new FixedTransform(new Vector3d(center.X, Fixed64.Zero, center.Y), FixedQuaternion.Identity, Vector3d.One)));

        foreach (LSCollider2D member in new[] { bottom, left })
        {
            var manifold = new ContactManifold2D();
            var item = new CollisionWorkItem2D(mover, member, ColliderSettings2D.GetCollisionType(mover.Shape, member.Shape));
            Assert.True(CollisionDetection2D.TryCollide(item, manifold, 0));
            Assert.Equal(member == bottom ? bottomCount : leftCount, manifold.Count);
            foreach (ManifoldContact2D row in manifold)
            {
                Assert.Equal(member == bottom ? -Vector2d.Forward : -Vector2d.Right, row.Normal);
                Assert.Equal(Fixed64.Quarter, row.Depth);
                Assert.True(row.HasMaterialOverride);
                Assert.Equal(member.Material, row.MaterialB);
                Assert.True(row.AnchorB.TryGetWorldPoint(out Vector2d point));
                Assert.True(member.ContainsPoint(point));
                Assert.True(mover.ContainsPoint(point));
            }
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void LUnion_DeepCircle_ShouldRequireBothExpandedBoundaryConstraints(bool polygonMembers)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        LSCollider2D bottom = Rectangle(scene.Context, polygonMembers, new(8, 4), new(0, -2));
        LSCollider2D left = Rectangle(scene.Context, polygonMembers, new(4, 4), new(-2, 2));
        Vector2d initial = new(-1, -1);
        Vector2d naive = initial + Vector2d.One.Normalized * ((-initial).Magnitude + Fixed64.Half);
        Vector2d touching = new(Fixed64.Half, Fixed64.Half);
        Vector2d separated = touching + Vector2d.One * Fixed64.FromRaw(1);

        foreach (LSCollider2D member in new[] { bottom, left })
        {
            Vector2d[] offsets = VertexOffsets(member);
            Assert.True(FixedConvex2dRelations.IntersectsSweptUprightCapsuleStrict(
                naive, naive, Fixed64.Zero, Fixed64.Half, member.Center, offsets));
            Assert.True(FixedConvex2dRelations.IntersectsUprightCapsule(
                touching, Fixed64.Zero, Fixed64.Half, member.Center, offsets));
            Assert.False(FixedConvex2dRelations.IntersectsSweptUprightCapsuleStrict(
                touching, touching, Fixed64.Zero, Fixed64.Half, member.Center, offsets));
            Assert.False(FixedConvex2dRelations.IntersectsUprightCapsule(
                separated, Fixed64.Zero, Fixed64.Half, member.Center, offsets));
        }
        var hits = new SwiftList<Physics2DHit>();
        Assert.Equal(2, scene.Context.Query2D.OverlapCircleAll(touching, Fixed64.Half, hits));
        Assert.Equal(0, scene.Context.Query2D.OverlapCircleAll(separated, Fixed64.Half, hits));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void LUnion_SeparatedCandidate_ShouldBeCheckedAgainstMembersOutsideTheInitialOverlap(bool polygonMembers)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        LSCollider2D bottom = Rectangle(scene.Context, polygonMembers, new(8, 4), new(0, -2));
        LSCollider2D left = Rectangle(scene.Context, polygonMembers, new(4, 4), new(-2, 2));
        LSCollider2D blocker = Rectangle(scene.Context, polygonMembers, new(Fixed64.Half, Fixed64.Half),
            new(Fixed64.One - Fixed64.Quarter, Fixed64.One - Fixed64.Quarter));
        var hits = new SwiftList<Physics2DHit>();
        Assert.Equal(1, scene.Context.Query2D.OverlapCircleAll(new(-1, -1), Fixed64.Half, hits));
        Assert.Same(bottom, hits[0].Collider);
        Vector2d candidate = new Vector2d(Fixed64.Half, Fixed64.Half) + Vector2d.One * Fixed64.FromRaw(1);
        Assert.False(QueryDetection2D.TryOverlapCircle(candidate, Fixed64.Half, bottom, out _));
        Assert.False(QueryDetection2D.TryOverlapCircle(candidate, Fixed64.Half, left, out _));

        Assert.Equal(1, scene.Context.Query2D.OverlapCircleAll(candidate, Fixed64.Half, hits));
        Assert.Same(blocker, hits[0].Collider);
        Assert.True(FixedConvex2dRelations.IntersectsSweptUprightCapsuleStrict(
            candidate, candidate, Fixed64.Zero, Fixed64.Half, blocker.Center, VertexOffsets(blocker)));
    }

    [Theory]
    [InlineData(false, 4)]
    [InlineData(true, 4)]
    [InlineData(false, 8)]
    [InlineData(true, 8)]
    public void FilledInteriorMember_WithNoExposedLocalSpan_ShouldRemainAnOverlapAndZeroDistanceSweep(
        bool polygonMembers, int tileSize)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var members = new LSCollider2D[9];
        for (int y = -1; y <= 1; y++)
            for (int x = -1; x <= 1; x++)
                members[(y + 1) * 3 + x + 1] = Rectangle(scene.Context, polygonMembers,
                    new(tileSize, tileSize), new(x * tileSize, y * tileSize));
        var domain = new MeshSurfaceBoundaryDomain2D();
        domain.Build(members, 4);
        Assert.Equal(4, domain.Spans.Length);
        foreach (var span in domain.Spans)
            Assert.Equal(MeshSurfaceBoundaryDomain2D.SpanKind.Joined, span.Kind);

        var hits = new SwiftList<Physics2DHit>();
        Assert.Equal(1, scene.Context.Query2D.OverlapCircleAll(Vector2d.Zero, Fixed64.Half, hits));
        Assert.Same(members[4], hits[0].Collider);
        Assert.True(scene.Context.Query2D.SweepCircle(Vector2d.Zero, Vector2d.Right, Fixed64.Half, out Physics2DHit hit));
        Assert.Same(members[4], hit.Collider);
        Assert.Equal(Fixed64.Zero, hit.Distance);
    }

    private static Vector2d[] VertexOffsets(LSCollider2D member)
    {
        var offsets = new Vector2d[member.VertexCount];
        for (int i = 0; i < offsets.Length; i++) offsets[i] = member.GetScaledLocalVertexUnchecked(i);
        return offsets;
    }

    private static LSCollider2D Rectangle(GravitasWorldContext context, bool polygon, Vector2d size, Vector2d center)
    {
        Vector2d half = size * Fixed64.Half;
        LSCollider2D member = polygon
            ? new LSPolygonCollider2D(new(-half.X, -half.Y), new(half.X, -half.Y), new(half.X, half.Y), new(-half.X, half.Y))
            : new LSAABBoxCollider2D(size);
        member.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(new Vector3d(center.X, Fixed64.Zero, center.Y), FixedQuaternion.Identity, Vector3d.One)));
        return member;
    }
}
