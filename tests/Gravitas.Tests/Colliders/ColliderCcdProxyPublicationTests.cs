using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class ColliderCcdProxyPublicationTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void FramePreparation_ShouldPublishCompensatedAuthoringBeforeBuildingCandidateBounds(bool planar, bool mixed)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = mixed ? PhysicsRuntimeMode.Mixed
            : planar ? PhysicsRuntimeMode.TwoD : PhysicsRuntimeMode.ThreeD;
        context.SetFrameRate(1);
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One * (Fixed64)4096);
        var agent = new TestMatterAgent(context, transform);
        var farPosition = new Vector3d(100000, 0, 0);
        var farBounds = DynamicCcdCandidateIndex.CreateBoundsBetween(farPosition, farPosition, Vector3d.One);

        if (planar)
        {
            var farTarget = new LSCircleCollider2D(Fixed64.Half);
            farTarget.InitializeWithNoBody(new TestMatterAgent(context,
                new FixedTransform(farPosition, FixedQuaternion.Identity, Vector3d.One)));
            var collider = new LSCircleCollider2D(Fixed64.Half);
            var body = new SolidBody2D(agent, collider);
            body.Initialize(Vector2d.Zero, motionType: BodyMotionType.Kinematic);
            body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            transform.LocalScale = Vector3d.One * Fixed64.MinIncrement;
            collider.Radius = (Fixed64)(1 << 30);
            collider.LocalOffset = new Vector2d(Fixed64.MaxValue, Fixed64.Zero);
            body.ResolveContinuousCollisionProxyRadius().Should().Be((Fixed64)2048);

            context.AdvanceLateSimulateToken();
            context.Physics2D.PrepareContinuousCollisionFrame();

            collider.ScaledRadius.Should().Be(Fixed64.Quarter);
            collider.Center.Should().Be(Vector2d.Right * Fixed64.Half);
            Fixed64 radius = body.ResolveContinuousCollisionProxyRadius();
            // This checks the publication epoch; rotation-rounding enclosure has separate raw-unit regressions.
            radius.Should().BeInRange(Fixed64.FromFraction(3, 4), Fixed64.One);
            body.GatherRotationalContinuousCollisionCandidates(Vector2d.Zero, Vector2d.Zero,
                Vector2d.Zero, radius).Should().Be(0);
            context.Physics2D.QueryPlanarContinuousCollisionCandidates(
                DynamicCcdCandidateIndex2D.CreateBoundsBetween(farPosition.ToVector2d(),
                    farPosition.ToVector2d(), Fixed64.One)).Should().NotContain(body.DynamicId);
            if (mixed)
                context.Physics2D.QueryMixedContinuousCollisionCandidates(farBounds).Should().NotContain(body.DynamicId);
        }
        else
        {
            var farTarget = new LSSphereCollider { Radius = Fixed64.Half };
            farTarget.InitializeWithNoBody(new TestMatterAgent(context,
                new FixedTransform(farPosition, FixedQuaternion.Identity, Vector3d.One)));
            var collider = new LSSphereCollider { Radius = Fixed64.Half };
            var body = new SolidBody(agent, collider);
            body.Initialize(Vector3d.Zero, FixedQuaternion.Identity, BodyMotionType.Kinematic);
            body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            transform.LocalScale = Vector3d.One * Fixed64.MinIncrement;
            collider.Radius = (Fixed64)(1 << 30);
            collider.LocalOffset = new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero);
            body.ResolveContinuousCollisionProxyRadius().Should().Be((Fixed64)2048);

            context.AdvanceLateSimulateToken();
            context.Physics.PrepareContinuousCollisionFrame();

            collider.ScaledRadius.Should().Be(Fixed64.Quarter);
            collider.Center.Should().Be(Vector3d.Right * Fixed64.Half);
            Fixed64 radius = body.ResolveContinuousCollisionProxyRadius();
            radius.Should().BeInRange(Fixed64.FromFraction(3, 4), Fixed64.One);
            body.GatherRotationalContinuousCollisionCandidates(Vector3d.Zero, Vector3d.Zero,
                Vector3d.Zero, radius).Should().Be(0);
            context.Physics.QueryContinuousCollisionCandidates(farBounds).Should().NotContain(body.DynamicId);
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void FramePreparation_ShouldIndexPendingGrowthBeforeAnyRotationalSampling(bool planar, bool mixed)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = mixed ? PhysicsRuntimeMode.Mixed
            : planar ? PhysicsRuntimeMode.TwoD : PhysicsRuntimeMode.ThreeD;
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var targetPosition = new Vector3d(7, 0, 0);
        var targetBounds = DynamicCcdCandidateIndex.CreateBoundsBetween(targetPosition, targetPosition,
            Vector3d.One * Fixed64.MinIncrement);

        if (planar)
        {
            var collider = new LSCircleCollider2D(Fixed64.Half);
            var body = new SolidBody2D(agent, collider);
            body.Initialize(Vector2d.Zero, motionType: BodyMotionType.Kinematic);
            transform.LocalScale = Vector3d.One * (Fixed64)4;
            collider.Radius = Fixed64.Two;
            body.ResolveContinuousCollisionProxyRadius().Should().Be(Fixed64.Half);

            context.AdvanceLateSimulateToken();
            context.Physics2D.PrepareContinuousCollisionFrame();

            collider.ScaledRadius.Should().Be((Fixed64)8);
            context.Physics2D.QueryPlanarContinuousCollisionCandidates(
                DynamicCcdCandidateIndex2D.CreateBoundsBetween(targetPosition.ToVector2d(),
                    targetPosition.ToVector2d(), Fixed64.MinIncrement)).Should().Contain(body.DynamicId);
            if (mixed)
                context.Physics2D.QueryMixedContinuousCollisionCandidates(targetBounds).Should().Contain(body.DynamicId);
        }
        else
        {
            var collider = new LSSphereCollider { Radius = Fixed64.Half };
            var body = new SolidBody(agent, collider);
            body.Initialize(Vector3d.Zero, FixedQuaternion.Identity, BodyMotionType.Kinematic);
            transform.LocalScale = Vector3d.One * (Fixed64)4;
            collider.Radius = Fixed64.Two;
            body.ResolveContinuousCollisionProxyRadius().Should().Be(Fixed64.Half);

            context.AdvanceLateSimulateToken();
            context.Physics.PrepareContinuousCollisionFrame();

            collider.ScaledRadius.Should().Be((Fixed64)8);
            context.Physics.QueryContinuousCollisionCandidates(targetBounds).Should().Contain(body.DynamicId);
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void FramePreparation_ShouldPublishPlanarGeometryAndMixedThicknessTogether(int mutation)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.Settings.Mixed2DHalfThickness = Fixed64.Half;
        var collider = new LSCircleCollider2D(Fixed64.One);
        var body = new SolidBody2D(new TestMatterAgent(context), collider);
        body.Initialize(Vector2d.Zero, motionType: BodyMotionType.Kinematic);
        new Vector2d(Fixed64.One, Fixed64.Half).TryGetMagnitudeCeiling(
            out Fixed64 committedRadius).Should().BeTrue();
        if (mutation == 0)
        {
            collider.Radius = Fixed64.Two;
            collider.MixedHalfThicknessOverride = (Fixed64)3;
        }
        else if (mutation == 1)
            context.Settings.Mixed2DHalfThickness = (Fixed64)3;
        else
            body.Agent.Transform.LocalPosition = new Vector3d(0, 2, 0);
        body.ResolveMixedContinuousCollisionProxyRadius().Should().Be(committedRadius);

        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();

        Fixed64 radius = mutation == 0 ? Fixed64.Two : Fixed64.One;
        Fixed64 thickness = mutation == 2 ? Fixed64.Half : (Fixed64)3;
        Fixed64 centerY = mutation == 2 ? Fixed64.Two : Fixed64.Zero;
        new Vector2d(radius, thickness).TryGetMagnitudeCeiling(
            out Fixed64 publishedRadius).Should().BeTrue();
        body.ResolveMixedContinuousCollisionProxyRadius().Should().Be(publishedRadius);
        collider.MixedSlabCenterY.Should().Be(centerY);
        var newSlice = new Vector3d(Fixed64.Zero, centerY + thickness - Fixed64.Quarter, Fixed64.Zero);
        context.Physics2D.QueryMixedContinuousCollisionCandidates(
            DynamicCcdCandidateIndex.CreateBoundsBetween(newSlice, newSlice,
                Vector3d.One * Fixed64.MinIncrement)).Should().Contain(body.DynamicId);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FramePreparation_ShouldObserveScaleChangesWithoutDirtyAuthoring(bool planar)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = planar ? PhysicsRuntimeMode.TwoD : PhysicsRuntimeMode.ThreeD;
        var agent = new TestMatterAgent(context);
        if (planar)
        {
            var collider = new LSCircleCollider2D(Fixed64.Half);
            var body = new SolidBody2D(agent, collider);
            body.Initialize(Vector2d.Zero, motionType: BodyMotionType.Kinematic);
            agent.Transform.LocalScale = Vector3d.One * (Fixed64)4;
            context.AdvanceLateSimulateToken();
            context.Physics2D.PrepareContinuousCollisionFrame();
            collider.ScaledRadius.Should().Be(Fixed64.Two);
            body.ResolveContinuousCollisionProxyRadius().Should().Be(Fixed64.Two);
        }
        else
        {
            var collider = new LSSphereCollider { Radius = Fixed64.Half };
            var body = new SolidBody(agent, collider);
            body.Initialize(Vector3d.Zero, FixedQuaternion.Identity, BodyMotionType.Kinematic);
            agent.Transform.LocalScale = Vector3d.One * (Fixed64)4;
            context.AdvanceLateSimulateToken();
            context.Physics.PrepareContinuousCollisionFrame();
            collider.ScaledRadius.Should().Be(Fixed64.Two);
            body.ResolveContinuousCollisionProxyRadius().Should().Be(Fixed64.Two);
        }
    }
}
