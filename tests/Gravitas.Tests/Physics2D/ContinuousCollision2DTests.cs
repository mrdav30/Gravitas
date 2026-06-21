using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using GridForge.Grids.Topology;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class ContinuousCollision2DTests
{
    [Theory]
    [InlineData(ColliderType2D.Circle)]
    [InlineData(ColliderType2D.AABox)]
    [InlineData(ColliderType2D.ConvexPolygon)]
    [InlineData(ColliderType2D.Compound)]
    public void ContinuousMode_ShouldPreventFastCircleTunnelingThroughStaticTargets(ColliderType2D targetShape)
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        StiffBody2D mover = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        _ = CreateBody(context, CreateCollider(targetShape), new Vector2d((Fixed64)5, Fixed64.Zero), immovable: true);
        mover.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        mover.AddForce(new Vector2d((Fixed64)10, Fixed64.Zero));
        context.LateSimulate();

        mover.Position.X.Should().Be((Fixed64)4);
        mover.LinearVelocity.X.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void ContinuousMode_ShouldUseCompoundOwnerProxyRadius()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero));
        StiffBody2D mover = CreateBody(context, compound, Vector2d.Zero, immovable: false);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)5, Fixed64.Zero), immovable: true);
        mover.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        mover.AddForce(new Vector2d((Fixed64)10, Fixed64.Zero));
        context.LateSimulate();

        mover.Position.X.Should().Be((Fixed64)4);
        mover.LinearVelocity.X.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void DiscreteMode_ShouldKeepExistingFastMovementPath()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        StiffBody2D mover = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)5, Fixed64.Zero), immovable: true);
        context.Settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        mover.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;

        mover.AddForce(new Vector2d((Fixed64)10, Fixed64.Zero));
        context.LateSimulate();

        mover.Position.X.Should().Be((Fixed64)10);
    }

    [Fact]
    public void AutoMode_ShouldSweepOnlyWhenMovementExceedsProxyRadius()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        StiffBody2D slow = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        StiffBody2D fast = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d(Fixed64.Zero, (Fixed64)3), immovable: false);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)5, Fixed64.Zero), immovable: true);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)5, (Fixed64)3), immovable: true);
        slow.ContinuousCollisionMode = ContinuousCollisionMode.Auto;
        fast.ContinuousCollisionMode = ContinuousCollisionMode.Auto;

        slow.AddForce(new Vector2d(Fixed64.Half, Fixed64.Zero));
        fast.AddForce(new Vector2d((Fixed64)10, Fixed64.Zero));
        context.LateSimulate();

        slow.Position.X.Should().Be(Fixed64.Half);
        fast.Position.X.Should().Be((Fixed64)4);
    }

    [Fact]
    public void ContinuousMode_WithOpposingDynamicBodies_ShouldClampBothAtSharedTimeOfImpact()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        StiffBody2D left = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)(-5), Fixed64.Zero), immovable: false);
        StiffBody2D right = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)5, Fixed64.Zero), immovable: false);
        left.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        right.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        left.AddForce(Vector2d.Right * (Fixed64)5);
        right.AddForce(-Vector2d.Right * (Fixed64)5);
        context.LateSimulate();

        left.Position.X.Should().BeLessThanOrEqualTo(-Fixed64.Half);
        right.Position.X.Should().BeGreaterThanOrEqualTo(Fixed64.Half);
        (right.Position.X - left.Position.X).Should().BeGreaterThanOrEqualTo(Fixed64.One);
        left.LinearVelocity.X.Should().Be(Fixed64.Zero);
        right.LinearVelocity.X.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void ContinuousMode_StaticCollector_ShouldSkipMovableDynamicTargets()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        StiffBody2D left = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)(-5), Fixed64.Zero), immovable: false);
        StiffBody2D right = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)5, Fixed64.Zero), immovable: false);
        left.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        right.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        left.AddForce(Vector2d.Right * (Fixed64)5);
        right.AddForce(-Vector2d.Right * (Fixed64)5);
        context.LateSimulate();

        context.Query2D.LastQueryCandidateCount.Should().Be(0);
        left.Position.X.Should().BeLessThanOrEqualTo(-Fixed64.Half);
        right.Position.X.Should().BeGreaterThanOrEqualTo(Fixed64.Half);
    }

    [Fact]
    public void ContinuousMode_StaticCollector_ShouldIncludeKinematicTargets()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)5, Fixed64.Zero), immovable: false, isKinematic: true);
        StiffBody2D mover = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        mover.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        mover.AddForce(new Vector2d((Fixed64)10, Fixed64.Zero));
        context.LateSimulate();

        context.Query2D.LastQueryCandidateCount.Should().Be(1);
        mover.Position.X.Should().Be((Fixed64)4);
        mover.LinearVelocity.X.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void ContinuousMode_WithPlanarDynamicsAtDifferentHostY_ShouldStillClampInPure2D()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        StiffBody2D left = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            immovable: false,
            hostY: (Fixed64)(-8));
        StiffBody2D right = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d((Fixed64)5, Fixed64.Zero),
            immovable: false,
            hostY: (Fixed64)8);
        left.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        right.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        left.AddForce(Vector2d.Right * (Fixed64)5);
        right.AddForce(-Vector2d.Right * (Fixed64)5);
        context.LateSimulate();

        left.Position.X.Should().BeLessThanOrEqualTo(-Fixed64.Half);
        right.Position.X.Should().BeGreaterThanOrEqualTo(Fixed64.Half);
        (right.Position.X - left.Position.X).Should().BeGreaterThanOrEqualTo(Fixed64.One);
    }

    [Fact]
    public void ContinuousMode_ShouldClampRotatingThinPolygonBeforeAngularTunnelingThroughStaticCircle()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        var bladeCollider = new LSPolygonCollider2D(
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(1, 10)),
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(1, 10)));
        StiffBody2D blade = CreateBody(context, bladeCollider, Vector2d.Zero, immovable: false);
        _ = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 4)),
            new Vector2d((Fixed64)2, (Fixed64)2),
            immovable: true);
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        blade.AddAngularImpulse(FixedMath.DegToRad((Fixed64)90) / blade.EffectiveInverseMomentOfInertia);
        context.LateSimulate();

        blade.Rotation.Should().BeLessThan(FixedMath.DegToRad((Fixed64)90));
        blade.AngularVelocity.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void ContinuousMode_ShouldNotClampRotatingThinPolygonForAngularNearMiss()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        var bladeCollider = new LSPolygonCollider2D(
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(1, 10)),
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(1, 10)));
        StiffBody2D blade = CreateBody(context, bladeCollider, Vector2d.Zero, immovable: false);
        _ = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 4)),
            new Vector2d((Fixed64)2, (Fixed64)(-2)),
            immovable: true);
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        Fixed64 angularVelocity = FixedMath.DegToRad((Fixed64)90);
        blade.AddAngularImpulse(angularVelocity / blade.EffectiveInverseMomentOfInertia);
        context.LateSimulate();

        blade.Rotation.Should().Be(angularVelocity);
        blade.AngularVelocity.Should().Be(angularVelocity);
    }

    [Fact]
    public void ContinuousMode_RotationalPath_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        var bladeCollider = new LSPolygonCollider2D(
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(1, 10)),
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(1, 10)));
        StiffBody2D blade = CreateBody(context, bladeCollider, Vector2d.Zero, immovable: false);
        _ = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 4)),
            new Vector2d((Fixed64)2, (Fixed64)2),
            immovable: true);
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Fixed64 angularImpulse = FixedMath.DegToRad((Fixed64)90) / blade.EffectiveInverseMomentOfInertia;

        void SimulateRotationalCcd()
        {
            blade.SetPosition(Vector2d.Zero);
            blade.SetRotation(Fixed64.Zero);
            blade.AddAngularImpulse(angularImpulse);
            context.LateSimulate();
        }

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            SimulateRotationalCcd,
            warmupIterations: 16,
            stabilizationIterations: 4,
            measurementIterations: 8);

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void Physics2DLateSimulate_DirectCalls_ShouldRefreshDynamicCcdFrame()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1, extent: 128);
        StiffBody2D mover = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)(-5), Fixed64.Zero), immovable: false);
        StiffBody2D target = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)100, Fixed64.Zero), immovable: false);
        mover.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        context.Physics2D.LateSimulate();
        target.SetPosition(new Vector2d((Fixed64)5, Fixed64.Zero));
        mover.AddForce(Vector2d.Right * (Fixed64)10);
        context.Physics2D.LateSimulate();

        mover.Position.X.Should().BeLessThanOrEqualTo((Fixed64)4);
        mover.LinearVelocity.X.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void InheritMode_ShouldResolveFromContextDefault()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        context.Settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        StiffBody2D mover = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)5, Fixed64.Zero), immovable: true);

        mover.AddForce(new Vector2d((Fixed64)10, Fixed64.Zero));
        context.LateSimulate();

        mover.Position.X.Should().Be((Fixed64)4);
    }

    [Fact]
    public void SweepCircleAll_ShouldReturnOrderedHitsAndFilterExcludedHierarchy()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        StiffBody2D parent = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)3, Fixed64.Zero), immovable: true);
        StiffBody2D child = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        StiffBody2D far = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)6, Fixed64.Zero), immovable: true);
        child.Collider.SetParent(parent.Collider);
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.SweepCircleAll(
            Vector2d.Zero,
            new Vector2d((Fixed64)8, Fixed64.Zero),
            Fixed64.Half,
            PhysicsLayerMask.All,
            hits,
            child.Collider,
            includeTriggers: false);

        count.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(far.Collider);
    }

    [Fact]
    public void SweepCircleAll_ShouldIncludeMovableKinematicImmovableAndBodylessTargets()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        StiffBody2D source = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)(-4), Fixed64.Zero), immovable: false);
        StiffBody2D movable = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        StiffBody2D kinematic = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)2, Fixed64.Zero), immovable: false, isKinematic: true);
        StiffBody2D immovable = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)4, Fixed64.Zero), immovable: true);
        LSCircleCollider2D bodyless = CreateBodylessCircle(context, new Vector2d((Fixed64)6, Fixed64.Zero));
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.SweepCircleAll(
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            new Vector2d((Fixed64)8, Fixed64.Zero),
            Fixed64.Half,
            PhysicsLayerMask.All,
            hits,
            source.Collider,
            includeTriggers: false);

        count.Should().Be(4);
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, movable.Collider));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, kinematic.Collider));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, immovable.Collider));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, bodyless));
    }

    [Fact]
    public void SweepCircleAgainstStaticAll_ShouldSkipMovableDynamicTargets()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        StiffBody2D source = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)(-4), Fixed64.Zero), immovable: false);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false);
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.SweepCircleAgainstStaticAll(
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            new Vector2d((Fixed64)2, Fixed64.Zero),
            Fixed64.Half,
            PhysicsLayerMask.All,
            hits,
            source.Collider,
            includeTriggers: false);

        count.Should().Be(0);
        context.Query2D.LastQueryCandidateCount.Should().Be(0);
    }

    [Fact]
    public void SweepCircleAgainstStaticAll_ShouldIncludeKinematicImmovableAndBodylessTargets()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        StiffBody2D source = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)(-4), Fixed64.Zero), immovable: false);
        StiffBody2D kinematic = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: false, isKinematic: true);
        StiffBody2D immovable = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        LSCircleCollider2D bodyless = CreateBodylessCircle(context, new Vector2d((Fixed64)4, Fixed64.Zero));
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.SweepCircleAgainstStaticAll(
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            new Vector2d((Fixed64)6, Fixed64.Zero),
            Fixed64.Half,
            PhysicsLayerMask.All,
            hits,
            source.Collider,
            includeTriggers: false);

        count.Should().Be(3);
        context.Query2D.LastQueryCandidateCount.Should().Be(3);
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, kinematic.Collider));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, immovable.Collider));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, bodyless));
    }

    [Fact]
    public void SweepCircleAll_ShouldApplyLayerAndTriggerFilters()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)3, Fixed64.Zero), immovable: true, layer: new PhysicsLayer(0));
        StiffBody2D trigger = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)4, Fixed64.Zero), immovable: true, layer: new PhysicsLayer(1));
        StiffBody2D included = CreateBody(context, new LSAABBoxCollider2D(Vector2d.One), new Vector2d((Fixed64)6, Fixed64.Zero), immovable: true, layer: new PhysicsLayer(1));
        trigger.Collider.IsTrigger = true;
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.SweepCircleAll(
            Vector2d.Zero,
            new Vector2d((Fixed64)8, Fixed64.Zero),
            Fixed64.Half,
            PhysicsLayerMask.FromLayer(1),
            hits,
            excludedCollider: null,
            includeTriggers: false);

        count.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(included.Collider);
    }

    [Fact]
    public void SweepCircleAll_ShouldOrderHitsByDistanceThenColliderId()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        StiffBody2D first = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)4, Fixed64.One), immovable: true);
        StiffBody2D second = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)4, -Fixed64.One), immovable: true);
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.SweepCircleAll(
            Vector2d.Zero,
            new Vector2d((Fixed64)8, Fixed64.Zero),
            Fixed64.Half,
            hits);

        count.Should().Be(2);
        hits[0].Distance.Should().Be(hits[1].Distance);
        hits[0].Collider.Should().BeSameAs(first.Collider);
        hits[1].Collider.Should().BeSameAs(second.Collider);
    }

    [Fact]
    public void SweepCircleAll_WithHexGrid_ShouldOrderHitsByDistanceThenColliderId()
    {
        using GravitasWorldContext context = CreateHexContext(frameRate: 1);
        StiffBody2D first = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)4, Fixed64.One), immovable: true);
        StiffBody2D second = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)4, -Fixed64.One), immovable: true);
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.SweepCircleAll(
            Vector2d.Zero,
            new Vector2d((Fixed64)8, Fixed64.Zero),
            Fixed64.Half,
            hits);

        count.Should().Be(2);
        hits[0].Distance.Should().Be(hits[1].Distance);
        hits[0].Collider.Should().BeSameAs(first.Collider);
        hits[1].Collider.Should().BeSameAs(second.Collider);
    }

    [Fact]
    public void SweepCircleAll_ShouldReturnNoHitsForZeroDisplacement()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, immovable: true);
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.SweepCircleAll(Vector2d.Zero, Vector2d.Zero, Fixed64.Half, hits);
        bool hasClosest = context.Query2D.SweepCircle(Vector2d.Zero, Vector2d.Zero, Fixed64.Half, out _);

        count.Should().Be(0);
        hasClosest.Should().BeFalse();
    }

    [Fact]
    public void SweepCircleAll_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1, extent: 128);
        for (int i = 0; i < 64; i++)
            _ = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d((Fixed64)(i * 2), Fixed64.Zero), immovable: true);

        var hits = new SwiftList<Physics2DHit>(64);
        Vector2d start = new((Fixed64)(-4), Fixed64.Zero);
        Vector2d end = new((Fixed64)140, Fixed64.Zero);
        for (int i = 0; i < 3; i++)
            context.Query2D.SweepCircleAll(start, end, Fixed64.Half, hits);

        long allocatedBytes = MeasureAllocatedBytes(() => context.Query2D.SweepCircleAll(start, end, Fixed64.Half, hits));

        allocatedBytes.Should().Be(0);
    }

    private static GravitasWorldContext CreateContext(int frameRate, int extent = 32) =>
        Physics2DTestWorld.CreateContext(frameRate, extent);

    private static GravitasWorldContext CreateHexContext(int frameRate, int extent = 32)
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.SetFrameRate(frameRate);
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;

        GridConfiguration configuration = new(
            new Vector3d((Fixed64)(-extent), Fixed64.Zero, (Fixed64)(-extent)),
            new Vector3d((Fixed64)extent, Fixed64.Zero, (Fixed64)extent),
            topologyKind: GridTopologyKind.HexPrism,
            topologyMetrics: GridTopologyMetrics.Hex(Fixed64.One, Fixed64.One, HexOrientation.PointyTop));

        context.World.TryAddGrid(configuration, out _).Should().BeTrue();
        return context;
    }

    private static StiffBody2D CreateBody(
        GravitasWorldContext context,
        LSCollider2D collider,
        Vector2d position,
        bool immovable,
        PhysicsLayer layer = default,
        Fixed64 hostY = default,
        bool isKinematic = false)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, hostY, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new StiffBody2D(agent, collider)
        {
            Mass = Fixed64.One,
            Immovable = immovable,
            IsKinematic = isKinematic
        };
        body.Collider.Layer = layer;
        body.Initialize(position);
        return body;
    }

    private static LSCircleCollider2D CreateBodylessCircle(
        GravitasWorldContext context,
        Vector2d position,
        Fixed64 hostY = default)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, hostY, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var collider = new LSCircleCollider2D(Fixed64.Half);
        collider.InitializeWithNoBody(agent);
        return collider;
    }

    private static LSCollider2D CreateCollider(ColliderType2D type) =>
        type switch
        {
            ColliderType2D.Circle => new LSCircleCollider2D(Fixed64.Half),
            ColliderType2D.AABox => new LSAABBoxCollider2D(Vector2d.One),
            ColliderType2D.ConvexPolygon => new LSPolygonCollider2D(
                new Vector2d(-Fixed64.Half, -Fixed64.Half),
                new Vector2d(Fixed64.Half, -Fixed64.Half),
                new Vector2d(Fixed64.Half, Fixed64.Half),
                new Vector2d(-Fixed64.Half, Fixed64.Half)),
            ColliderType2D.Compound => new LSCompoundCollider2D(
                CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero)),
            _ => new LSCircleCollider2D(Fixed64.Half)
        };

    private static long MeasureAllocatedBytes(System.Action action)
        => AllocationTestHelper.MeasureSinglePass(action);
}
