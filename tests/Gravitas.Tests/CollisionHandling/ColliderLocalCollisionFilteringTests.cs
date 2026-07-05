using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ColliderLocalCollisionFilteringTests
{
    private static readonly Fixed64 WallThickness = Fixed64.Half;

    [Fact]
    public void NewColliders_ShouldDefaultIgnoredCollisionLayersToNone()
    {
        new LSSphereCollider().IgnoredCollisionLayers.Should().Be(PhysicsLayerMask.None);
        new LSCircleCollider2D(Fixed64.Half).IgnoredCollisionLayers.Should().Be(PhysicsLayerMask.None);
    }

    [Fact]
    public void RequireCollisionPair3D_WhenFirstIgnoresSecondLayer_ShouldRejectPair()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        first.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(second.Collider.Layer);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        HasPair(first.Collider, second.Collider).Should().BeFalse();
    }

    [Fact]
    public void RequireCollisionPair3D_WhenSecondIgnoresFirstLayer_ShouldRejectPair()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        second.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(first.Collider.Layer);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        HasPair(first.Collider, second.Collider).Should().BeFalse();
    }

    [Fact]
    public void RequireCollisionPair3D_WithDefaultMasks_ShouldKeepExistingPairBehavior()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        HasPair(first.Collider, second.Collider).Should().BeTrue();
    }

    [Fact]
    public void RequireCollisionPair3D_WhenTriggerIgnoresOtherLayer_ShouldRejectTriggerPair()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCuboidCollider trigger = CreateStaticWall(scenario, Fixed64.Zero, default);
        ScenarioBody<LSSphereCollider> other = scenario.CreateSphere(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        trigger.IsTrigger = true;
        trigger.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(other.Collider.Layer);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        HasPair(trigger, other.Collider).Should().BeFalse();
    }

    [Fact]
    public void RequireCollisionPair3D_WhenMaskChangesAtRuntime_ShouldRemoveStalePair()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();
        HasPair(first.Collider, second.Collider).Should().BeTrue();

        first.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(second.Collider.Layer);
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        HasPair(first.Collider, second.Collider).Should().BeFalse();
    }

    [Fact]
    public void RequireCollisionPair2D_WhenEitherColliderIgnoresOtherLayer_ShouldRejectPair()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D first = CreateCircle2D(context, Vector2d.Zero);
        SolidBody2D second = CreateCircle2D(context, new Vector2d(Fixed64.Half, Fixed64.Zero));
        first.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(second.Collider.Layer);

        Step(context);

        HasPair(first.Collider, second.Collider).Should().BeFalse();

        first.Collider.IgnoredCollisionLayers = PhysicsLayerMask.None;
        second.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(first.Collider.Layer);

        Step(context);

        HasPair(first.Collider, second.Collider).Should().BeFalse();
    }

    [Fact]
    public void RequireCollisionPair2D_WithDefaultMasks_ShouldKeepExistingPairBehavior()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D first = CreateCircle2D(context, Vector2d.Zero);
        SolidBody2D second = CreateCircle2D(context, new Vector2d(Fixed64.Half, Fixed64.Zero));

        Step(context);

        HasPair(first.Collider, second.Collider).Should().BeTrue();
    }

    [Fact]
    public void RequireCollisionPair2D_WhenTriggerIgnoresOtherLayer_ShouldRejectTriggerPair()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        LSAABBoxCollider2D trigger = CreateStaticBox2D(context, Vector2d.Zero, Vector2d.One, default);
        SolidBody2D other = CreateCircle2D(context, new Vector2d(Fixed64.Half, Fixed64.Zero));
        trigger.IsTrigger = true;
        trigger.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(other.Collider.Layer);

        Step(context);

        HasPair(trigger, other.Collider).Should().BeFalse();
    }

    [Fact]
    public void RequireCollisionPair2D_WhenMaskChangesAtRuntime_ShouldRemoveStalePair()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D first = CreateCircle2D(context, Vector2d.Zero);
        SolidBody2D second = CreateCircle2D(context, new Vector2d(Fixed64.Half, Fixed64.Zero));

        Step(context);
        HasPair(first.Collider, second.Collider).Should().BeTrue();

        first.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(second.Collider.Layer);
        Step(context);

        HasPair(first.Collider, second.Collider).Should().BeFalse();
    }

    [Fact]
    public void RequireCollisionPairMixed_WhenEitherColliderIgnoresOtherLayer_ShouldRejectPair()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        body3D.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(body2D.Collider.Layer);

        Step(context);

        context.MixedCollisions.ActivePairCount.Should().Be(0);

        body3D.Collider.IgnoredCollisionLayers = PhysicsLayerMask.None;
        body2D.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(body3D.Collider.Layer);

        Step(context);

        context.MixedCollisions.ActivePairCount.Should().Be(0);
    }

    [Fact]
    public void RequireCollisionPairMixed_WithDefaultMasks_ShouldKeepExistingPairBehavior()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateSphere3D(context, Vector3d.Zero);
        _ = CreateCircle2D(context, Vector2d.Zero);

        Step(context);

        context.MixedCollisions.ActivePairCount.Should().Be(1);
    }

    [Fact]
    public void RequireCollisionPairMixed_WhenMaskChangesAtRuntime_ShouldRemoveStalePair()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);

        Step(context);
        context.MixedCollisions.ActivePairCount.Should().Be(1);

        body3D.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(body2D.Collider.Layer);
        Step(context);

        context.MixedCollisions.ActivePairCount.Should().Be(0);
    }

    [Fact]
    public void ContinuousCollision3D_WhenStaticTargetLayerIsIgnored_ShouldNotClampMover()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        LSCuboidCollider wall = CreateStaticWall(scenario, Fixed64.Zero, new PhysicsLayer(1));
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero));
        mover.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        mover.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(wall.Layer);
        DisableGroundQueries(mover.Body);

        ApplyFastImpulse(mover.Body);

        mover.Body.Position3d.X.Should().Be((Fixed64)2);
        mover.Body.LinearVelocity.X.Should().Be((Fixed64)4);
    }

    [Fact]
    public void ContinuousCollision3D_WhenDynamicTargetLayerIsIgnored_ShouldNotClampOrHandOffVelocity()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(Vector3d.Zero);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Collider.Layer = new PhysicsLayer(1);
        source.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(target.Collider.Layer);
        DisableGroundQueries(source.Body);
        DisableGroundQueries(target.Body);
        target.Body.Sleep();

        ApplyFastImpulse(source.Body);

        source.Body.Position3d.X.Should().Be(Fixed64.One);
        source.Body.LinearVelocity.X.Should().Be((Fixed64)4);
        target.Body.Position3d.Should().Be(Vector3d.Zero);
        target.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        target.Body.IsSleeping.Should().BeTrue();
    }

    [Fact]
    public void ContinuousCollision2D_WhenStaticTargetLayerIsIgnored_ShouldNotClampMover()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(frameRate: 1);
        SolidBody2D mover = CreateCircle2D(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));
        SolidBody2D target = CreateCircle2D(context, Vector2d.Zero, immovable: true, layer: new PhysicsLayer(1));
        mover.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        mover.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(target.Collider.Layer);

        mover.AddForce(Vector2d.Right * (Fixed64)10);
        Step(context);

        mover.Position.X.Should().BeGreaterThan(Fixed64.One);
        mover.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void MixedContinuousCollision_When3DSourceIgnores2DTargetLayer_ShouldNotClamp()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        ScenarioBody<LSSphereCollider> source3D = CreateSphere3D(context, new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero));
        SolidBody2D target2D = CreateCircle2D(context, Vector2d.Zero, immovable: true, layer: new PhysicsLayer(1));
        source3D.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(target2D.Collider.Layer);

        source3D.Body.AddForce(Vector3d.Down * (Fixed64)10);
        Step(context);

        source3D.Body.Position3d.Y.Should().BeLessThan(Fixed64.Zero);
        source3D.Body.LinearVelocity.Y.Should().BeLessThan(Fixed64.Zero);
    }

    [Fact]
    public void MixedContinuousCollision_When2DSourceIgnores3DTargetLayer_ShouldNotClamp()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        ScenarioBody<LSSphereCollider> target3D = CreateSphere3D(context, Vector3d.Zero, immovable: true, layer: new PhysicsLayer(1));
        SolidBody2D source2D = CreateCircle2D(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));
        source2D.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(target3D.Collider.Layer);

        source2D.AddForce(Vector2d.Right * (Fixed64)10);
        Step(context);

        source2D.Position.X.Should().BeGreaterThan(Fixed64.One);
        source2D.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void Grounding3D_WhenBodyIgnoresGroundLayer_ShouldRejectGroundCandidate()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCuboidCollider ground = CreateStaticFloor3D(scenario, new PhysicsLayer(1));
        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(ground.Layer);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(ground.Layer);

        body.Body.CheckGround();

        body.Body.IsGrounded.Should().BeFalse();
        body.Body.WasGrounded.Should().BeTrue();
    }

    [Fact]
    public void Grounding2D_WhenBodyIgnoresSupportLayer_ShouldRejectProbeAndContactSupport()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        LSAABBoxCollider2D support = CreateStaticBox2D(
            context,
            new Vector2d(Fixed64.Zero, Fixed64.Zero),
            new Vector2d((Fixed64)8, Fixed64.One),
            new PhysicsLayer(1));
        context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(support.Layer);
        SolidBody2D body = CreateCircle2D(context, new Vector2d(Fixed64.Zero, Fixed64.One));

        body.IsGrounded.Should().BeTrue("the support candidate must be valid before the local ignore mask is applied");
        body.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(support.Layer);

        Step(context);

        body.IsGrounded.Should().BeFalse("ignored support layers should be rejected by contact refresh");
        body.WasGrounded.Should().BeTrue("losing support through a local ignore mask should preserve the previous grounded state for the frame");

        body.CheckGround();

        body.IsGrounded.Should().BeFalse("ignored support layers should also be rejected by explicit probes");
    }

    [Fact]
    public void Query3D_ShouldReturnIgnoredPhysicalLayerWhenCallerMaskIncludesIt()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(Vector3d.Zero);
        target.Collider.Layer = new PhysicsLayer(1);
        source.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(target.Collider.Layer);
        var hits = new SwiftList<Physics3DHit>();

        int count = scenario.Context.Query3D.RaycastAll(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            PhysicsLayerMask.FromLayer(target.Collider.Layer),
            hits);

        count.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(target.Collider);
    }

    [Fact]
    public void Query2D_ShouldReturnIgnoredPhysicalLayerWhenCallerMaskIncludesIt()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = CreateCircle2D(context, new Vector2d((Fixed64)(-4), Fixed64.Zero));
        SolidBody2D target = CreateCircle2D(context, Vector2d.Zero, layer: new PhysicsLayer(1));
        source.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(target.Collider.Layer);
        var rayHits = new SwiftList<Physics2DHit>();
        var overlapHits = new SwiftList<Physics2DHit>();

        int rayCount = context.Query2D.RaycastAll(
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            new Vector2d((Fixed64)2, Fixed64.Zero),
            PhysicsLayerMask.FromLayer(target.Collider.Layer),
            rayHits);
        int overlapCount = context.Query2D.OverlapCircleAll(
            Vector2d.Zero,
            Fixed64.One,
            PhysicsLayerMask.FromLayer(target.Collider.Layer),
            overlapHits);

        rayCount.Should().Be(1);
        rayHits[0].Collider.Should().BeSameAs(target.Collider);
        overlapCount.Should().Be(1);
        overlapHits[0].Collider.Should().BeSameAs(target.Collider);
    }

    [Fact]
    public void QueryMixed_ShouldReturnIgnoredPhysicalLayerWhenCallerMaskIncludesIt()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> target3D = CreateSphere3D(context, Vector3d.Zero, immovable: true, layer: new PhysicsLayer(1));
        SolidBody2D source2D = CreateCircle2D(context, new Vector2d((Fixed64)(-4), Fixed64.Zero));
        source2D.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(target3D.Collider.Layer);

        bool hit = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            new Vector2d((Fixed64)2, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            PhysicsLayerMask.FromLayer(target3D.Collider.Layer),
            out PhysicsMixedHit mixedHit);

        hit.Should().BeTrue();
        mixedHit.Collider3D.Should().BeSameAs(target3D.Collider);
    }

    private static bool HasPair(LSCollider first, LSCollider second) =>
        first.TryGetCollisionPair(second.Id, out CollisionPair? firstPair) && firstPair != null
        || second.TryGetCollisionPair(first.Id, out CollisionPair? secondPair) && secondPair != null;

    private static bool HasPair(LSCollider2D first, LSCollider2D second) =>
        first.TryGetCollisionPair(second.Id, out CollisionPair2D? firstPair) && firstPair != null
        || second.TryGetCollisionPair(first.Id, out CollisionPair2D? secondPair) && secondPair != null;

    private static SolidBody2D CreateCircle2D(
        GravitasWorldContext context,
        Vector2d position,
        bool immovable = false,
        PhysicsLayer layer = default)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var body = new SolidBody2D(new TestMatterAgent(context, transform), new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One,
            FreezeAxes = immovable ? BodyFreezeAxes2D.Position : BodyFreezeAxes2D.None
        };
        body.Collider.Layer = layer;
        body.Initialize(position);
        return body;
    }

    private static ScenarioBody<LSSphereCollider> CreateSphere3D(
        GravitasWorldContext context,
        Vector3d position,
        bool immovable = false,
        PhysicsLayer layer = default)
    {
        var transform = new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One);
        var collider = new LSSphereCollider();
        collider.Layer = layer;
        var body = new SolidBody(new TestMatterAgent(context, transform), collider)
        {
            Mass = Fixed64.One,
            FreezeAxes = immovable ? BodyFreezeAxes3D.Position : BodyFreezeAxes3D.None
        };
        body.Initialize(position, FixedQuaternion.Identity);
        return new ScenarioBody<LSSphereCollider>(body, collider);
    }

    private static PhysicsScenarioBuilder CreateCcdScenario()
    {
        PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.SetFrameRate(1);
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        scenario.Context.Environment.MaxSpeed = (Fixed64)16;
        scenario.Context.Environment.MaxFallSpeed = (Fixed64)16;
        return scenario;
    }

    private static LSCuboidCollider CreateStaticWall(
        PhysicsScenarioBuilder scenario,
        Fixed64 x,
        PhysicsLayer layer)
    {
        var wall = new LSCuboidCollider
        {
            Layer = layer,
            Size = new Vector3d(WallThickness, (Fixed64)8, (Fixed64)8)
        };

        scenario.InitializeStaticCollider(wall, new Vector3d(x, Fixed64.Zero, Fixed64.Zero));
        return wall;
    }

    private static LSCuboidCollider CreateStaticFloor3D(PhysicsScenarioBuilder scenario, PhysicsLayer layer)
    {
        var floor = new LSCuboidCollider
        {
            Layer = layer,
            Size = new Vector3d((Fixed64)8, Fixed64.One, (Fixed64)8)
        };

        scenario.InitializeStaticCollider(floor, new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero));
        return floor;
    }

    private static LSAABBoxCollider2D CreateStaticBox2D(
        GravitasWorldContext context,
        Vector2d position,
        Vector2d size,
        PhysicsLayer layer)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var collider = new LSAABBoxCollider2D(size)
        {
            Layer = layer
        };
        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
        return collider;
    }

    private static void ApplyFastImpulse(SolidBody body) =>
        body.AddLinearImpulse(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));

    private static void DisableGroundQueries(SolidBody body)
    {
        body.GroundedDistanceRay = Fixed64.Zero;
        body.GroundDownDistanceOnAir = Fixed64.Zero;
    }

    private static GravitasWorldContext CreateMixedContext(int frameRate = 4)
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.SetFrameRate(frameRate);
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.MinSpeed = Fixed64.Zero;
        context.Environment.MaxSpeed = (Fixed64)16;
        context.Environment.MaxFallSpeed = (Fixed64)16;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-32), (Fixed64)(-8), (Fixed64)(-32)),
                new Vector3d((Fixed64)32, (Fixed64)8, (Fixed64)32)),
            out _).Should().BeTrue();
        return context;
    }

    private static void Step(GravitasWorldContext context)
    {
        context.Simulate();
        context.LateSimulate();
    }
}
