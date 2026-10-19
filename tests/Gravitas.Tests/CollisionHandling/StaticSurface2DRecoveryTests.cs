using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using System;
using Xunit;

namespace Gravitas.Tests;

public sealed class StaticSurface2DRecoveryTests
{
    [Theory]
    [InlineData(0, false)] [InlineData(1, false)] [InlineData(2, false)]
    [InlineData(0, true)] [InlineData(1, true)] [InlineData(2, true)]
    public void CompleteHostStep_ShouldPublishTheCertifiedRecoveryAndKeepInitialOverlapEvents(int shape, bool moverFirst)
    {
        using var context = Physics2DTestWorld.CreateContext();
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        LSCollider2D mover = Mover(shape);
        var body = new SolidBody2D(new TestMatterAgent(context), mover)
        { Mass = Fixed64.One, SleepEnabled = false };
        if (moverFirst) body.Initialize(Vector2d.Zero);
        LSCollider2D member = Rectangle(context, new(8, 8), Vector2d.Zero);
        if (!moverFirst) body.Initialize(Vector2d.Zero);
        body.UseManualGrounding();
        using var surface = context.Physics2D.CreateStaticSurface(new[] { member });
        int enters = 0, exits = 0;
        member.OnContactEnter += _ => enters++;
        member.OnContactExit += _ => exits++;
        context.Simulate(); context.LateSimulate();
        CollisionPair2D pair = Pair(member, mover);
        Assert.True(pair.IsColliding);
        Assert.Empty(pair.Manifold);
        Assert.False(pair.HasRecovery);
        Assert.Equal(1, enters);
        Assert.Equal(0, exits);
        Assert.NotEqual(Vector2d.Zero, body.Position);
        Assert.False(CollisionDetection2D.DoesPostureCandidatePenetrate(mover, mover.Center, member));
        Assert.True(surface.TryGetRecovery(mover, out Vector2d alreadyClear));
        Assert.Equal(mover.Center, alreadyClear);
        context.Simulate(); context.LateSimulate();
        Assert.Equal(1, exits);
    }

    [Fact]
    public void BlockerLoadedByAnEnterCallback_ShouldRejectTheDeferredTargetBeforePublishingIt()
    {
        using var context = Physics2DTestWorld.CreateContext();
        context.Environment.Gravity = Fixed64.Zero;
        var member = Rectangle(context, new(4, 4), Vector2d.Zero);
        using var surface = context.Physics2D.CreateStaticSurface(new[] { member });
        var mover = Mover(0);
        var body = new SolidBody2D(new TestMatterAgent(context), mover) { Mass = Fixed64.One, SleepEnabled = false };
        body.Initialize(Vector2d.Zero); body.UseManualGrounding();
        bool loaded = false;
        member.OnContactEnter += _ =>
        {
            Assert.True(Pair(member, mover).HasRecovery);
            Assert.True(surface.TryGetRecovery(mover, out Vector2d center));
            Rectangle(context, Vector2d.One, center);
            loaded = true;
        };
        context.Simulate();
        var error = Assert.Throws<InvalidOperationException>(() => context.LateSimulate());
        Assert.Contains("no longer has a certified clear final pose", error.Message);
        Assert.True(loaded);
        Assert.Equal(Vector2d.Zero, body.Position);
        Assert.True(Pair(member, mover).IsColliding);
    }

    [Theory]
    [InlineData(0)] [InlineData(1)] [InlineData(2)] [InlineData(3)]
    public void DeepObservations_ShouldNotRequireMovableBodyRecovery(int kind)
    {
        using var context = Physics2DTestWorld.CreateContext();
        LSCollider2D member;
        if (kind == 0)
        {
            member = new LSAABBoxCollider2D(new Vector2d(4, 4));
            var staticBody = new SolidBody2D(new TestMatterAgent(context), member) { Mass = Fixed64.One };
            staticBody.Initialize(Vector2d.Zero, motionType: BodyMotionType.Static);
        }
        else member = Rectangle(context, new(4, 4), Vector2d.Zero);
        using var surface = context.Physics2D.CreateStaticSurface(new[] { member });
        var mover = Mover(0);
        if (kind == 0) mover.InitializeWithNoBody(new TestMatterAgent(context));
        else if (kind == 1)
        {
            mover.IsTrigger = true;
            mover.InitializeWithNoBody(new TestMatterAgent(context));
        }
        else
        {
            var body = new SolidBody2D(new TestMatterAgent(context), mover)
            { Mass = kind == 2 ? Fixed64.Zero : Fixed64.One };
            body.Initialize(Vector2d.Zero, motionType: kind == 3 ? BodyMotionType.Static : BodyMotionType.Dynamic);
        }
        var pair = new CollisionPair2D(member, mover);
        Assert.True(CollisionDetection2D.TryCollide(pair, pair.Manifold, 0));
        Assert.False(pair.HasRecovery);
        CollisionResponse2D.Resolve(pair);
        Assert.True(CollisionDetection2D.TryCollide(mover, member, out _));
    }

    [Theory]
    [InlineData(0)] [InlineData(1)] [InlineData(2)] [InlineData(3)] [InlineData(4)] [InlineData(5)]
    public void ChangingAPreparedRecoveryBeforeResponse_ShouldNeverPublishAnUncertifiedPose(int change)
    {
        using var context = Physics2DTestWorld.CreateContext();
        var member = Rectangle(context, new(4, 4), Vector2d.Zero);
        using var surface = context.Physics2D.CreateStaticSurface(new[] { member });
        var mover = Mover(0);
        var body = new SolidBody2D(new TestMatterAgent(context), mover) { Mass = Fixed64.One };
        body.Initialize(Vector2d.Zero); body.UseManualGrounding();
        var pair = new CollisionPair2D(member, mover);
        Assert.True(CollisionDetection2D.TryCollide(pair, pair.Manifold, 0));
        Assert.True(pair.HasRecovery);
        if (change == 0) body.FreezeAxes = BodyFreezeAxes2D.Position;
        if (change == 1) surface.Dispose();
        if (change == 2) mover.LocalOffset = Vector2d.Right;
        if (change == 3) mover.Agent.Transform.LocalScale = new(2, 1, 2);
        if (change == 4) mover.Agent.Transform.LocalScale = Vector3d.Zero;
        if (change == 5)
        {
            Assert.True(surface.TryGetRecovery(mover, out Vector2d target));
            Fixed64 upper = Fixed64.MaxValue - Fixed64.Half, lower = Fixed64.MinValue + Fixed64.Half;
            body.SetPosition(new(target.X < Fixed64.Zero ? upper : target.X > Fixed64.Zero ? lower : Fixed64.Zero,
                target.Y < Fixed64.Zero ? upper : target.Y > Fixed64.Zero ? lower : Fixed64.Zero));
        }
        Vector2d beforeResponse = body.Position;
        Assert.Throws<InvalidOperationException>(() => CollisionResponse2D.Resolve(pair));
        Assert.Equal(beforeResponse, body.Position);
    }

    [Fact]
    public void CompoundWithOneDeepLeaf_ShouldFailRatherThanRecoverOnlyThatLeaf()
    {
        using var context = Physics2DTestWorld.CreateContext();
        var member = Rectangle(context, new(4, 4), Vector2d.Zero);
        using var surface = context.Physics2D.CreateStaticSurface(new[] { member });
        var mover = new LSCompoundCollider2D(CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero),
            CompoundColliderPart2D.Circle(Fixed64.Half, new(4, 0)));
        var body = new SolidBody2D(new TestMatterAgent(context), mover) { Mass = Fixed64.One };
        body.Initialize(Vector2d.Zero); body.UseManualGrounding();
        var pair = new CollisionPair2D(member, mover);
        Assert.Throws<InvalidOperationException>(() => CollisionDetection2D.TryCollide(pair, pair.Manifold, 0));
        Assert.True(pair.Manifold.NeedsSurfaceRecovery);
        Assert.Equal(Vector2d.Zero, body.Position);
    }

    [Fact]
    public void OffsetMidpointTie_ShouldCertifyTheActualPublishedCenterAgainstAOneRawBlocker()
    {
        using var context = Physics2DTestWorld.CreateContext();
        var member = Rectangle(context, new(Fixed64.Two + Fixed64.FromRaw(2), (Fixed64)4), Vector2d.Zero);
        using var surface = context.Physics2D.CreateStaticSurface(new[] { member });
        var mover = new LSCircleCollider2D(Fixed64.Half) { LocalOffset = new(Fixed64.FromRaw(1), Fixed64.Zero) };
        var agent = new TestMatterAgent(context, new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity,
            new(Fixed64.Half, Fixed64.One, Fixed64.One)));
        var body = new SolidBody2D(agent, mover) { Mass = Fixed64.One };
        body.Initialize(Vector2d.Zero); body.UseManualGrounding();
        Seed(mover, member);
        Assert.Equal(Vector2d.Zero, mover.Center);
        Assert.True(surface.TryGetRecovery(mover, out Vector2d candidate));
        Assert.True(mover.TryPrepareRecoveryPose(candidate, body.Rotation, out Vector2d published));
        Assert.Equal(1, FixedMath.Abs(published.X - candidate.X).m_rawValue);
        Vector2d towardPublished = published.X > candidate.X ? Vector2d.Right : -Vector2d.Right;
        Rectangle(context, new(Fixed64.FromRaw(2), Fixed64.One),
            candidate + towardPublished * (mover.ScaledRadius + Fixed64.FromRaw(1)));
        Assert.False(surface.IsRecoveryClear(mover, candidate));
        Assert.Equal(Vector2d.Zero, body.Position);
    }

    [Theory]
    [InlineData(0)] [InlineData(1)] [InlineData(2)] [InlineData(3)]
    public void PolygonRecovery_ShouldInspectActualPrimitiveAndCompoundBlockers(int kind)
    {
        using var context = Physics2DTestWorld.CreateContext();
        var member = Rectangle(context, new(4, 4), Vector2d.Zero);
        using var surface = context.Physics2D.CreateStaticSurface(new[] { member });
        LSCollider2D mover = Mover(2);
        var body = new SolidBody2D(new TestMatterAgent(context), mover) { Mass = Fixed64.One };
        body.Initialize(Vector2d.Zero);
        Seed(mover, member);
        Assert.True(surface.TryGetRecovery(mover, out Vector2d candidate));
        LSCollider2D blocker = kind == 0 ? new LSCircleCollider2D(Fixed64.Quarter)
            : kind == 1 ? new LSCapsuleCollider2D(Fixed64.Quarter, Fixed64.One)
            : new LSCompoundCollider2D(CompoundColliderPart2D.Circle(Fixed64.Quarter, new(-2, 0)),
                CompoundColliderPart2D.Circle(Fixed64.Quarter, new(0, 2)),
                CompoundColliderPart2D.Circle(Fixed64.Quarter, kind == 2 ? new(2, 0) : Vector2d.Zero));
        blocker.InitializeWithNoBody(new TestMatterAgent(context, new FixedTransform(
            new(candidate.X, Fixed64.Zero, candidate.Y), FixedQuaternion.Identity, Vector3d.One)));
        Assert.Equal(kind == 2, surface.IsRecoveryClear(mover, candidate));
        Assert.Equal(Vector2d.Zero, body.Position);
    }

    [Fact]
    public void CandidateCenterAndBodyPositionOverflow_ShouldDeclineCertification()
    {
        using var context = Physics2DTestWorld.CreateContext();
        using var surface = context.Physics2D.CreateStaticSurface(Array.Empty<LSCollider2D>());
        var mover = new LSCircleCollider2D(Fixed64.Quarter) { LocalOffset = -Vector2d.Right * Fixed64.Half };
        var body = new SolidBody2D(new TestMatterAgent(context), mover) { Mass = Fixed64.One };
        body.Initialize(new(Fixed64.MaxValue - Fixed64.Half, Fixed64.Zero));
        Assert.False(surface.IsRecoveryClear(mover, new(Fixed64.MinValue, Fixed64.Zero)));
        Assert.False(surface.IsRecoveryClear(mover, new(Fixed64.MaxValue - Fixed64.Quarter, Fixed64.Zero)));
    }

    [Fact]
    public void RotatingAnOffsetAfterCertification_ShouldRejectAnUnrepresentableFinalCenter()
    {
        using var context = Physics2DTestWorld.CreateContext();
        Assert.True(context.World.TryAddGrid(new GridConfiguration(
            new(Fixed64.MaxValue - (Fixed64)16, Fixed64.Zero, (Fixed64)(-8)),
            new(Fixed64.MaxValue, Fixed64.Zero, (Fixed64)8)), out _));
        var mover = new LSCircleCollider2D(Fixed64.Quarter) { LocalOffset = -Vector2d.Right };
        var body = new SolidBody2D(new TestMatterAgent(context), mover) { Mass = Fixed64.One };
        body.Initialize(new(Fixed64.MaxValue - Fixed64.Quarter, Fixed64.Zero));
        var member = Rectangle(context, new(Fixed64.One, Fixed64.Two),
            mover.Center + Vector2d.Right * Fixed64.FromFraction(1, 16));
        using var surface = context.Physics2D.CreateStaticSurface(new[] { member });
        var pair = new CollisionPair2D(member, mover);
        Assert.True(CollisionDetection2D.TryCollide(pair, pair.Manifold, 0));
        Assert.True(pair.HasRecovery);
        body.SetPosition(new(Fixed64.MaxValue - Fixed64.Two, Fixed64.Zero));
        body.SetRotation(Fixed64.Pi);
        Vector2d before = body.Position;
        Assert.Throws<InvalidOperationException>(() => CollisionResponse2D.Resolve(pair));
        Assert.Equal(before, body.Position);
    }

    [Theory]
    [InlineData(0)] [InlineData(1)] [InlineData(2)]
    public void Recovery_ShouldCertifyTheCompleteFinalNeighborhoodWithoutPublishingTheProposal(int shape)
    {
        using var context = Physics2DTestWorld.CreateContext();
        var member = Rectangle(context, new(4, 4), Vector2d.Zero);
        using var surface = context.Physics2D.CreateStaticSurface(new[] { member });
        LSCollider2D mover = shape == 0 ? new LSCircleCollider2D(Fixed64.Half)
            : shape == 1 ? new LSCapsuleCollider2D(Fixed64.Half, Fixed64.Two)
            : new LSPolygonCollider2D(new(-Fixed64.Half, -Fixed64.Half), new(Fixed64.Half, -Fixed64.Half),
                new(Fixed64.Half, Fixed64.Half), new(-Fixed64.Half, Fixed64.Half));
        var body = new SolidBody2D(new TestMatterAgent(context), mover) { Mass = Fixed64.One };
        body.Initialize(Vector2d.Zero);
        Seed(mover, member);
        Assert.True(surface.TryGetRecovery(mover, out Vector2d candidate));
        Assert.NotEqual(Vector2d.Zero, candidate);
        Assert.Equal(Vector2d.Zero, body.Position);
        Assert.Equal(Vector2d.Zero, mover.Center);
        Assert.False(CollisionDetection2D.DoesPostureCandidatePenetrate(mover, candidate, member));

        var blocker = Rectangle(context, Vector2d.One, candidate);
        Assert.False(CollisionDetection2D.TryCollide(mover, blocker, out _));
        Assert.False(surface.IsRecoveryClear(mover, candidate));
        Assert.False(surface.TryGetRecovery(mover, out _));
        blocker.IsTrigger = true;
        Assert.True(surface.IsRecoveryClear(mover, candidate));
        blocker.IsTrigger = false;
        blocker.IgnoredCollisionLayers = PhysicsLayerMask.All;
        Assert.True(surface.IsRecoveryClear(mover, candidate));
        Assert.Equal(Vector2d.Zero, body.Position);
    }

    [Fact]
    public void SurroundedMember_ShouldNotCertifyAnExitIntoAnInitiallyAbsentNeighbor()
    {
        using var context = Physics2DTestWorld.CreateContext();
        var members = new LSCollider2D[9];
        for (int y = -1; y <= 1; y++)
            for (int x = -1; x <= 1; x++) members[(y + 1) * 3 + x + 1] = Rectangle(context, new(4, 4), new(x * 4, y * 4));
        using var surface = context.Physics2D.CreateStaticSurface(members);
        var mover = new LSCircleCollider2D(Fixed64.Half);
        var body = new SolidBody2D(new TestMatterAgent(context), mover) { Mass = Fixed64.One };
        body.Initialize(Vector2d.Zero);
        Seed(mover, members[4]);
        Assert.False(surface.TryGetRecovery(mover, out _));
        Assert.Equal(Vector2d.Zero, body.Position);
        Assert.True(CollisionDetection2D.TryCollide(mover, members[4], out _));
        Assert.True(context.Query2D.SweepCircle(Vector2d.Zero, Vector2d.Right, Fixed64.Half, out var hit));
        Assert.Equal(Fixed64.Zero, hit.Distance);
        var pair = new CollisionPair2D(mover, members[4]);
        Assert.Throws<InvalidOperationException>(() => CollisionDetection2D.TryCollide(pair, pair.Manifold, 0));
        Assert.True(pair.Manifold.NeedsSurfaceRecovery);
    }

    [Fact]
    public void Certificate_ShouldCheckCompoundBlockersAndProjectFrozenTranslation()
    {
        using var context = Physics2DTestWorld.CreateContext();
        var member = Rectangle(context, new(4, 4), Vector2d.Zero);
        using var surface = context.Physics2D.CreateStaticSurface(new[] { member });
        var mover = new LSCircleCollider2D(Fixed64.Half);
        var body = new SolidBody2D(new TestMatterAgent(context), mover) { Mass = Fixed64.One };
        body.Initialize(Vector2d.Zero);
        Seed(mover, member);
        Assert.True(surface.TryGetRecovery(mover, out Vector2d candidate));
        var blocker = new LSCompoundCollider2D(CompoundColliderPart2D.Circle(Fixed64.Half, new(4, 0)),
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero));
        blocker.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(new Vector3d(candidate.X, Fixed64.Zero, candidate.Y), FixedQuaternion.Identity, Vector3d.One)));
        Assert.False(surface.IsRecoveryClear(mover, candidate));
        blocker.Deactivate();
        body.FreezeAxes = BodyFreezeAxes2D.PositionY;
        Assert.False(surface.TryGetRecovery(mover, out _));
        body.FreezeAxes = BodyFreezeAxes2D.Position;
        Assert.False(surface.TryGetRecovery(mover, out _));
        Assert.Equal(Vector2d.Zero, body.Position);
    }

    private static void Seed(LSCollider2D mover, LSCollider2D member)
    {
        var manifold = new ContactManifold2D();
        Assert.True(CollisionDetection2D.TryCollide(new CollisionWorkItem2D(mover, member,
            ColliderSettings2D.GetCollisionType(mover.Shape, member.Shape)), manifold, 0));
        Assert.Empty(manifold);
    }

    private static LSCollider2D Mover(int shape) => shape == 0 ? new LSCircleCollider2D(Fixed64.Half)
        : shape == 1 ? new LSCapsuleCollider2D(Fixed64.Half, Fixed64.Two)
        : new LSPolygonCollider2D(new(-Fixed64.Half, -Fixed64.Half), new(Fixed64.Half, -Fixed64.Half),
            new(Fixed64.Half, Fixed64.Half), new(-Fixed64.Half, Fixed64.Half));

    private static CollisionPair2D Pair(LSCollider2D first, LSCollider2D second)
    {
        if (first.TryGetCollisionPair(second.Id, out var pair)) return pair!;
        Assert.True(second.TryGetCollisionPair(first.Id, out pair));
        return pair!;
    }

    private static LSCollider2D Rectangle(GravitasWorldContext context, Vector2d size, Vector2d center)
    {
        var collider = new LSAABBoxCollider2D(size);
        collider.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(new Vector3d(center.X, Fixed64.Zero, center.Y), FixedQuaternion.Identity, Vector3d.One)));
        return collider;
    }
}
