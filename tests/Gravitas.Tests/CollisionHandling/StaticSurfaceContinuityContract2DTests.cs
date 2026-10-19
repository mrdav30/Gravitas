using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using SwiftCollections;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests;

public sealed class StaticSurfaceContinuityContract2DTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void FreelyRotatingObliquePolygon_ShouldMatchTheContinuousFloor(bool polygonMembers, bool reverse)
    {
        var baseline = Trace(false);
        var joined = Trace(true);
        Fixed64 tolerance = Fixed64.FromFraction(1, 1000);
        for (int frame = 0; frame < baseline.Count; frame++)
            for (int component = 0; component < baseline[frame].Length; component++)
                Assert.True(FixedMath.Abs(baseline[frame][component] - joined[frame][component]) <= tolerance,
                    $"Frame {frame}, component {component}: {baseline[frame][component]} versus {joined[frame][component]}.");

        List<Fixed64[]> Trace(bool declare)
        {
            using var context = Physics2DTestWorld.CreateContext(frameRate: 16);
            context.Environment.Gravity = Fixed64.One;
            context.Environment.AirDensity = Fixed64.Zero;
            context.Environment.DampingFactor = Fixed64.Zero;
            context.Environment.MinSpeed = Fixed64.Zero;
            LSCollider2D[] members = declare
                ? new[] { Rectangle(context, polygonMembers, new(8, 2), new(-4, -1)),
                    Rectangle(context, polygonMembers, new(8, 2), new(4, -1)) }
                : new[] { Rectangle(context, polygonMembers, new(16, 2), new(0, -1)) };
            foreach (LSCollider2D member in members) member.Material = PhysicsMaterial.Frictionless;
            using var surface = declare ? context.Physics2D.CreateStaticSurface(reverse
                ? new[] { members[1], members[0] } : members) : null;
            var polygon = new LSPolygonCollider2D(new(-Fixed64.Half, -Fixed64.Half),
                new(Fixed64.Half, -Fixed64.Half), new(Fixed64.Half, Fixed64.Half), new(-Fixed64.Half, Fixed64.Half))
            { Material = PhysicsMaterial.Frictionless };
            var body = new SolidBody2D(new TestMatterAgent(context), polygon)
            { Mass = Fixed64.One, SleepEnabled = false };
            body.Initialize(new(-Fixed64.Quarter, Fixed64.One - Fixed64.Quarter), Fixed64.PiOver4 * Fixed64.Half);
            body.UseManualGrounding();
            body.AddLinearImpulse(new(Fixed64.One, -Fixed64.Quarter));
            body.AddAngularImpulse(Fixed64.FromFraction(1, 64));
            var trace = new List<Fixed64[]>();
            bool rotated = false;
            for (int frame = 0; frame < 64; frame++)
            {
                context.Simulate(); context.LateSimulate();
                Assert.Equal(Fixed64.One, body.LinearVelocity.X);
                rotated |= body.AngularVelocity != Fixed64.Zero;
                trace.Add(new[] { body.Position.X, body.Position.Y, body.LinearVelocity.X,
                    body.LinearVelocity.Y, body.Rotation, body.AngularVelocity });
            }
            Assert.True(rotated);
            Assert.True(body.Position.X > Fixed64.One);
            return trace;
        }
    }

    [Fact]
    public void EnterCallbackRemovingANeighbor_ShouldRefreshTheCarrierBeforeResponse()
    {
        using var context = Physics2DTestWorld.CreateContext();
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        LSCollider2D left = Rectangle(context, false, new(4, 2), new(-2, -1));
        LSCollider2D right = Rectangle(context, false, new(4, 2), new(2, -1));
        using var surface = context.Physics2D.CreateStaticSurface(new[] { left, right });
        var circle = new LSCircleCollider2D(Fixed64.Half);
        var body = new SolidBody2D(new TestMatterAgent(context), circle)
        { Mass = Fixed64.One, FreezeAxes = BodyFreezeAxes2D.Rotation, SleepEnabled = false };
        body.Initialize(new(Fixed64.Quarter, Fixed64.Quarter));
        body.UseManualGrounding();
        int rightSupplier = right.Id + 1;
        bool removed = false, supplied = false;
        left.OnContactEnter += _ =>
        {
            CollisionPair2D pair = GetPair();
            for (int group = 0; group < pair.Manifold.GroupCount; group++)
            {
                ContactGroupKey key = pair.Manifold.GetGroup(group).Key;
                supplied |= key.SourceIdA == rightSupplier || key.SourceIdB == rightSupplier;
            }
            removed = true;
            right.IsActive = false;
        };
        context.Simulate(); context.LateSimulate();
        Assert.True(removed);
        Assert.True(supplied);
        CollisionPair2D current = GetPair();
        Assert.True(current.Manifold.HasContact);
        for (int group = 0; group < current.Manifold.GroupCount; group++)
        {
            ContactGroupKey key = current.Manifold.GetGroup(group).Key;
            Assert.NotEqual(rightSupplier, key.SourceIdA);
            Assert.NotEqual(rightSupplier, key.SourceIdB);
        }

        CollisionPair2D GetPair()
        {
            if (left.TryGetCollisionPair(circle.Id, out var pair)) return pair!;
            Assert.True(circle.TryGetCollisionPair(left.Id, out pair));
            return pair!;
        }
    }

    [Theory]
    [InlineData(false, false, 0)]
    [InlineData(true, false, 0)]
    [InlineData(false, true, 0)]
    [InlineData(true, true, 0)]
    [InlineData(false, false, 1)]
    [InlineData(true, false, 1)]
    [InlineData(false, true, 1)]
    [InlineData(true, true, 1)]
    [InlineData(false, false, 2)]
    [InlineData(true, false, 2)]
    [InlineData(false, true, 2)]
    [InlineData(true, true, 2)]
    public void JoinedFloorCrossing_ShouldMatchTheContinuousFullLoopTrace(
        bool polygonMembers, bool staticBodies, int shape)
    {
        List<long> continuous = Trace(joined: false, reverse: false);
        Assert.Equal(continuous, Trace(joined: true, reverse: false));
        Assert.Equal(continuous, Trace(joined: true, reverse: true));

        List<long> Trace(bool joined, bool reverse)
        {
            using var context = Physics2DTestWorld.CreateContext(frameRate: 16);
            context.Environment.Gravity = Fixed64.One;
            context.Environment.AirDensity = Fixed64.Zero;
            context.Environment.DampingFactor = Fixed64.Zero;
            context.Environment.MinSpeed = Fixed64.Zero;
            LSCollider2D[] members = joined
                ? new[] { Rectangle(context, polygonMembers, new(8, 2), new(-4, -1), staticBodies),
                    Rectangle(context, polygonMembers, new(8, 2), new(4, -1), staticBodies) }
                : new[] { Rectangle(context, polygonMembers, new(16, 2), new(0, -1), staticBodies) };
            using var surface = joined ? context.Physics2D.CreateStaticSurface(reverse
                ? new[] { members[1], members[0] } : members) : null;
            int enters = 0, exits = 0;
            foreach (LSCollider2D member in members)
            {
                member.Material = PhysicsMaterial.Frictionless;
                member.OnContactEnter += _ => enters++;
                member.OnContactExit += _ => exits++;
            }
            LSCollider2D mover = shape == 0 ? new LSCircleCollider2D(Fixed64.Half)
                : new LSCapsuleCollider2D(Fixed64.Half, shape == 1 ? Fixed64.Two : Fixed64.One);
            mover.Material = PhysicsMaterial.Frictionless;
            var body = new SolidBody2D(new TestMatterAgent(context), mover)
            { Mass = Fixed64.One, FreezeAxes = BodyFreezeAxes2D.Rotation, SleepEnabled = false };
            body.Initialize(new(-Fixed64.Two, shape == 1 ? Fixed64.One : Fixed64.Half));
            body.UseManualGrounding();
            for (int frame = 0; frame < 32; frame++) Step();
            body.AddLinearImpulse(Vector2d.Right);
            var trace = new List<long>();
            for (int frame = 0; frame < 64; frame++)
            {
                Step();
                Assert.Equal(Fixed64.One, body.LinearVelocity.X);
                Assert.Equal(Fixed64.Zero, body.AngularVelocity);
                trace.Add(body.Position.X.m_rawValue); trace.Add(body.Position.Y.m_rawValue);
                trace.Add(body.LinearVelocity.X.m_rawValue); trace.Add(body.LinearVelocity.Y.m_rawValue);
            }
            Assert.Equal(Fixed64.Two, body.Position.X);
            Assert.Equal(joined ? 2 : 1, enters);
            Assert.Equal(joined ? 1 : 0, exits);
            return trace;

            void Step() { context.Simulate(); context.LateSimulate(); }
        }
    }

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

    private static LSCollider2D Rectangle(GravitasWorldContext context, bool polygon, Vector2d size, Vector2d center,
        bool staticBody = false)
    {
        Vector2d half = size * Fixed64.Half;
        LSCollider2D member = polygon
            ? new LSPolygonCollider2D(new(-half.X, -half.Y), new(half.X, -half.Y), new(half.X, half.Y), new(-half.X, half.Y))
            : new LSAABBoxCollider2D(size);
        var agent = new TestMatterAgent(context,
            new FixedTransform(new Vector3d(center.X, Fixed64.Zero, center.Y), FixedQuaternion.Identity, Vector3d.One));
        if (staticBody)
        {
            var body = new SolidBody2D(agent, member) { Mass = Fixed64.One };
            body.Initialize(center, motionType: BodyMotionType.Static);
        }
        else member.InitializeWithNoBody(agent);
        return member;
    }
}
