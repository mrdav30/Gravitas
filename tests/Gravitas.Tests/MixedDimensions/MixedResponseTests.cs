using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed class MixedResponseTests
{
    [Fact]
    public void Simulate_WithDynamic3DOnStatic2DSlab_ShouldResolveVerticallyAndNotify()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero));
        LSCollider2D platform = CreateBodylessBox2D(context, Vector2d.Zero, new Vector2d((Fixed64)4, (Fixed64)4));
        int entered3D = 0;
        int stayed3D = 0;
        int entered2D = 0;
        body3D.Collider.OnMixedContactEnter += other =>
        {
            other.Should().BeSameAs(platform);
            entered3D++;
        };
        body3D.Collider.OnMixedContact += _ => stayed3D++;
        platform.OnMixedContactEnter += other =>
        {
            other.Should().BeSameAs(body3D.Collider);
            entered2D++;
        };

        Step(context);
        Fixed64 resolvedY = body3D.Body.Position3d.Y;
        Step(context);

        resolvedY.Should().BeGreaterThan(Fixed64.FromFraction(3, 4));
        body3D.Body.Position3d.Y.Should().BeGreaterThanOrEqualTo(resolvedY);
        platform.Center.Should().Be(Vector2d.Zero);
        entered3D.Should().Be(1);
        entered2D.Should().Be(1);
        stayed3D.Should().Be(2);
        context.MixedCollisions.ActivePairCount.Should().Be(1);
    }

    [Fact]
    public void Simulate_WithDynamic3DPushingDynamic2D_ShouldMoveBothPlanarly()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero));
        StiffBody2D body2D = CreateCircle2D(context, Vector2d.Zero);

        Step(context);

        body3D.Body.Position3d.X.Should().BeLessThan(-Fixed64.FromFraction(1, 4));
        body2D.Position.X.Should().BeGreaterThan(Fixed64.Zero);
        body2D.Agent.Transform.Position.Y.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void LateSimulate_ShouldRefreshMovedMixedCollidersAndDistributeContactsAfterIntegration()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.FromFraction(5, 4), Fixed64.Zero, Fixed64.Zero));
        StiffBody2D body2D = CreateCircle2D(context, Vector2d.Zero, immovable: true);
        Vector3d startPosition = body3D.Body.Position3d;
        int entered = 0;
        body3D.Collider.OnMixedContactEnter += other =>
        {
            other.Should().BeSameAs(body2D.Collider);
            entered++;
        };

        body3D.Body.AddForce(new Vector3d((Fixed64)16, Fixed64.Zero, Fixed64.Zero));
        context.Simulate();

        body3D.Body.Position3d.Should().Be(startPosition);
        context.MixedCollisions.ActivePairCount.Should().Be(0);
        entered.Should().Be(0);

        context.LateSimulate();

        body3D.Body.Position3d.X.Should().BeGreaterThan(startPosition.X);
        context.MixedCollisions.ActivePairCount.Should().Be(1);
        entered.Should().Be(1);
    }

    [Fact]
    public void Resolve_WithVerticalOnlyMixedImpulse_ShouldNotTranslateOrSpin2DParticipant()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero));
        StiffBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        var contact = new MixedContact(
            new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Half),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half),
            Vector3d.Up,
            Fixed64.FromFraction(1, 10));
        body3D.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Up);

        CollisionResponseMixed.Resolve(pair, contact);

        body2D.Position.Should().Be(Vector2d.Zero);
        body2D.LinearVelocity.Should().Be(Vector2d.Zero);
        body2D.AngularVelocity.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithPlanarOffsetMixedImpulse_ShouldSpin2DParticipantAroundCom()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Half));
        StiffBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        var contact = new MixedContact(
            new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Half),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half),
            Vector3d.Right,
            Fixed64.FromFraction(1, 10));
        body3D.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right);

        CollisionResponseMixed.Resolve(pair, contact);

        body2D.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        body2D.LinearVelocity.Y.Should().Be(Fixed64.Zero);
        body2D.AngularVelocity.Should().BeLessThan(Fixed64.Zero);
    }

    [Fact]
    public void Simulate_WithKinematic3DAgainstDynamic2D_ShouldOnlyMove2DParticipant()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero),
            isKinematic: true);
        StiffBody2D body2D = CreateCircle2D(context, Vector2d.Zero);

        Step(context);

        body3D.Body.Position3d.X.Should().Be(-Fixed64.FromFraction(1, 4));
        body2D.Position.X.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void Simulate_WithBodylessMixedTrigger_ShouldNotifyTriggerWithoutResponse()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        LSCollider2D trigger = CreateBodylessCircle2D(context, Vector2d.Zero, isTrigger: true);
        int triggerEntered = 0;
        int contactEntered = 0;
        trigger.OnMixedTriggerEnter += other =>
        {
            other.Should().BeSameAs(body3D.Collider);
            triggerEntered++;
        };
        body3D.Collider.OnMixedContactEnter += _ => contactEntered++;

        Step(context);

        body3D.Body.Position3d.Should().Be(Vector3d.Zero);
        triggerEntered.Should().Be(1);
        contactEntered.Should().Be(0);
    }

    [Fact]
    public void Simulate_WithAwake3DAgainstSleeping2D_ShouldWakeSleepingParticipant()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateSphere3D(context, new Vector3d(-Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero));
        StiffBody2D sleeping2D = CreateCircle2D(context, Vector2d.Zero);
        sleeping2D.Sleep();

        Step(context);

        sleeping2D.IsSleeping.Should().BeFalse();
    }

    [Fact]
    public void Simulate_WithLayerBlockedMixedPair_ShouldNotCreatePairOrRespond()
    {
        using GravitasWorldContext context = CreateMixedContextWithLayerBlock();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero, layer: new PhysicsLayer(1));
        StiffBody2D body2D = CreateCircle2D(context, Vector2d.Zero, layer: new PhysicsLayer(2));
        int entered = 0;
        body3D.Collider.OnMixedContactEnter += _ => entered++;

        Step(context);

        context.MixedCollisions.ActivePairCount.Should().Be(0);
        body3D.Body.Position3d.Should().Be(Vector3d.Zero);
        body2D.Position.Should().Be(Vector2d.Zero);
        entered.Should().Be(0);
    }

    [Fact]
    public void Simulate_WithSeparatedFormerMixedPair_ShouldEmitExitAndRecyclePair()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        _ = CreateCircle2D(context, Vector2d.Zero);
        int exited = 0;
        body3D.Collider.OnMixedContactExit += _ => exited++;

        Step(context);
        body3D.Body.SetPosition(new Vector3d((Fixed64)6, Fixed64.Zero, Fixed64.Zero));
        Step(context);

        exited.Should().Be(1);
        context.MixedCollisions.ActivePairCount.Should().Be(0);
        context.MixedCollisions.PooledPairCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void ReplayedMixedResponseScenario_ShouldProduceSameState()
    {
        (Vector3d position3D, Vector3d velocity3D, Vector2d position2D, Vector2d velocity2D) first = RunReplayScenario();
        (Vector3d position3D, Vector3d velocity3D, Vector2d position2D, Vector2d velocity2D) second = RunReplayScenario();

        second.Should().Be(first);
    }

    private static (Vector3d position3D, Vector3d velocity3D, Vector2d position2D, Vector2d velocity2D) RunReplayScenario()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 8);
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero));
        StiffBody2D body2D = CreateCircle2D(context, Vector2d.Zero);

        for (int i = 0; i < 5; i++)
        {
            body3D.Body.AddForce(Vector3d.Right);
            body2D.AddForce(-Vector2d.Right);
            context.Simulate();
            context.LateSimulate();
        }

        return (body3D.Body.Position3d, body3D.Body.LinearVelocity, body2D.Position, body2D.LinearVelocity);
    }

    private static GravitasWorldContext CreateMixedContext(int frameRate = 4)
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.ApplySettings(new PhysicsSettings(frameRate, null));
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.Environment.Gravity = Fixed64.Zero;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-8), (Fixed64)(-4), (Fixed64)(-8)),
                new Vector3d((Fixed64)8, (Fixed64)4, (Fixed64)8)),
            out _).Should().BeTrue();
        return context;
    }

    private static void Step(GravitasWorldContext context)
    {
        context.Simulate();
        context.LateSimulate();
    }

    private static GravitasWorldContext CreateMixedContextWithLayerBlock()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.ApplySettings(new PhysicsSettings(
            4,
            new[,]
            {
                { true, true, true },
                { true, true, false },
                { true, false, true }
            }));
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.Environment.Gravity = Fixed64.Zero;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-8), (Fixed64)(-4), (Fixed64)(-8)),
                new Vector3d((Fixed64)8, (Fixed64)4, (Fixed64)8)),
            out _).Should().BeTrue();
        return context;
    }

    private static ScenarioBody<LSSphereCollider> CreateSphere3D(
        GravitasWorldContext context,
        Vector3d position,
        bool immovable = false,
        bool isKinematic = false,
        PhysicsLayer? layer = null)
    {
        var collider = new LSSphereCollider();
        if (layer.HasValue)
            collider.Layer = layer.Value;

        var agent = new TestMatterAgent(context, new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One));
        var body = new StiffBody(agent, collider)
        {
            Mass = Fixed64.One,
            Immovable = immovable,
            IsKinematic = isKinematic,
            RestitutionCoefficient = Fixed64.Zero
        };
        body.Initialize(position, FixedQuaternion.Identity);
        return new ScenarioBody<LSSphereCollider>(body, collider);
    }

    private static StiffBody2D CreateCircle2D(
        GravitasWorldContext context,
        Vector2d position,
        bool immovable = false,
        PhysicsLayer? layer = null)
    {
        var collider = new LSCircleCollider2D(Fixed64.Half);
        if (layer.HasValue)
            collider.Layer = layer.Value;

        var agent = new TestMatterAgent(
            context,
            new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One));
        var body = new StiffBody2D(agent, collider)
        {
            Mass = Fixed64.One,
            Immovable = immovable,
            RestitutionCoefficient = Fixed64.Zero
        };
        body.Initialize(position);
        return body;
    }

    private static LSCollider2D CreateBodylessCircle2D(
        GravitasWorldContext context,
        Vector2d position,
        bool isTrigger = false)
    {
        var collider = new LSCircleCollider2D(Fixed64.Half)
        {
            IsTrigger = isTrigger
        };
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One));
        collider.InitializeWithNoBody(agent);
        return collider;
    }

    private static LSCollider2D CreateBodylessBox2D(
        GravitasWorldContext context,
        Vector2d position,
        Vector2d size)
    {
        var collider = new LSAABBoxCollider2D(size);
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One));
        collider.InitializeWithNoBody(agent);
        return collider;
    }
}
