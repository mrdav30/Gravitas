using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Support;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedResponseTests
{
    [Fact]
    public void Simulate_WhenEarlierMixedCallbackDeactivatesLater2DCandidate_ShouldSkipStaleCandidate()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D first2D = CreateCircle2D(context, Vector2d.Zero);
        SolidBody2D stale2D = CreateCircle2D(context, Vector2d.Zero);
        string staleEvents = string.Empty;
        body3D.Collider.OnMixedContactEnter += other =>
        {
            if (ReferenceEquals(other, first2D.Collider))
                stale2D.Deactivate();
        };
        stale2D.Collider.OnMixedContactEnter += _ => staleEvents += "enter;";
        stale2D.Collider.OnMixedContactExit += _ => staleEvents += "exit;";

        Step(context);

        staleEvents.Should().BeEmpty();
        stale2D.Active.Should().BeFalse();
        context.MixedCollisions.ActivePairCount.Should().Be(1);
    }

    [Fact]
    public void Simulate_WhenEarlierMixedCallbackDeactivatesLater3DCandidate_ShouldSkipStaleCandidate()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> first3D = CreateSphere3D(context, Vector3d.Zero);
        ScenarioBody<LSSphereCollider> stale3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        string staleEvents = string.Empty;
        first3D.Collider.OnMixedContactEnter += _ => stale3D.Body.Deactivate();
        stale3D.Collider.OnMixedContactEnter += _ => staleEvents += "enter;";
        stale3D.Collider.OnMixedContactExit += _ => staleEvents += "exit;";

        Step(context);

        staleEvents.Should().BeEmpty();
        stale3D.Body.Active.Should().BeFalse();
        context.MixedCollisions.ActivePairCount.Should().Be(1);
        body2D.Active.Should().BeTrue();
    }

    [Fact]
    public void Simulate_WhenEarlierMixedCallbackFreezesLaterNewPair_ShouldSkipImmovableCandidate()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D first2D = CreateCircle2D(context, Vector2d.Zero);
        SolidBody2D immovable2D = CreateCircle2D(context, Vector2d.Zero, immovable: true);
        int immovableEnters = 0;
        body3D.Collider.OnMixedContactEnter += other =>
        {
            if (ReferenceEquals(other, first2D.Collider))
                body3D.Body.FreezeAxes = BodyFreezeAxes3D.All;
        };
        immovable2D.Collider.OnMixedContactEnter += _ => immovableEnters++;

        Step(context);

        immovableEnters.Should().Be(0);
        body3D.Body.CanTranslate.Should().BeFalse();
        immovable2D.CanTranslate.Should().BeFalse();
        context.MixedCollisions.ActivePairCount.Should().Be(1);
    }

    [Fact]
    public void Simulate_WhenSingleResponsePairFallsAsleepDuringNotification_ShouldSkipResponse()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        body3D.Collider.OnMixedContactEnter += _ =>
        {
            body3D.Body.Sleep();
            body2D.Sleep();
        };
        Vector3d position3D = body3D.Body.Position3d;
        Vector2d position2D = body2D.Position;

        Step(context);

        body3D.Body.IsSleeping.Should().BeTrue();
        body2D.IsSleeping.Should().BeTrue();
        body3D.Body.Position3d.Should().Be(position3D);
        body2D.Position.Should().Be(position2D);
        body3D.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        body2D.LinearVelocity.Should().Be(Vector2d.Zero);
        context.MixedCollisions.ActivePairCount.Should().Be(1);
    }

    [Fact]
    public void Simulate_WhenMixedPairPooled_ShouldReusePairForNextContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Settings.PoolingEnabled = true;
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D retired2D = CreateCircle2D(context, Vector2d.Zero);
        Step(context);
        retired2D.Deactivate();

        context.MixedCollisions.PooledPairCount.Should().Be(1);
        SolidBody2D replacement2D = CreateCircle2D(context, Vector2d.Zero);
        int entered = 0;
        replacement2D.Collider.OnMixedContactEnter += _ => entered++;

        Step(context);

        entered.Should().Be(1);
        context.MixedCollisions.ActivePairCount.Should().Be(1);
        context.MixedCollisions.PooledPairCount.Should().Be(0);
        body3D.Body.Active.Should().BeTrue();
    }

    [Fact]
    public void Simulate_WhenPairPoolingDisabled_ShouldDiscardSeparatedShell()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Settings.PoolingEnabled = false;
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);

        Step(context);
        body2D.Deactivate();

        context.MixedCollisions.ActivePairCount.Should().Be(0);
        context.MixedCollisions.PooledPairCount.Should().Be(0);
        body3D.Body.Active.Should().BeTrue();
    }

    [Fact]
    public void MarkSeparated_BeforeMixedPairAdmission_ShouldNotEmitExit()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D body2D = CreateCircle2D(context, new Vector2d((Fixed64)4, Fixed64.Zero));
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        int exits = 0;
        body3D.Collider.OnMixedContactExit += _ => exits++;
        body2D.Collider.OnMixedContactExit += _ => exits++;

        pair.MarkSeparated();

        exits.Should().Be(0);
        pair.LastFrame.Should().Be(-1);
        pair.Contact.Should().Be(default(MixedContact));
    }

    [Fact]
    public void Simulate_WithPlanarContactBetweenSleeping3DAndAwake2D_ShouldWake3DParticipant()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sleeping3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        _ = CreateCircle2D(context, Vector2d.Zero);
        sleeping3D.Body.Sleep();

        Step(context);

        sleeping3D.Body.IsSleeping.Should().BeFalse();
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void MixedTriggerEnter_When3DCallbackDeactivatesEitherParticipant_ShouldSkipStayAndExitOnlyAdmittedSide(
        bool deactivate3D)
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        LSCollider2D trigger2D = CreateBodylessCircle2D(context, Vector2d.Zero, isTrigger: true);
        string events3D = string.Empty;
        string events2D = string.Empty;
        body3D.Collider.OnMixedTriggerEnter += _ =>
        {
            events3D += "enter;";
            if (deactivate3D)
                body3D.Body.Deactivate();
            else
                trigger2D.Deactivate();
        };
        body3D.Collider.OnMixedTriggerStay += _ => events3D += "stay;";
        body3D.Collider.OnMixedTriggerExit += _ => events3D += "exit;";
        trigger2D.OnMixedTriggerEnter += _ => events2D += "enter;";
        trigger2D.OnMixedTriggerStay += _ => events2D += "stay;";
        trigger2D.OnMixedTriggerExit += _ => events2D += "exit;";

        Step(context);

        events3D.Should().Be("enter;exit;");
        events2D.Should().BeEmpty();
        context.MixedCollisions.ActivePairCount.Should().Be(0);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void MixedTriggerEnter_When2DCallbackDeactivatesEitherParticipant_ShouldSkip2DStayAndFinishBothLifecycles(
        bool deactivate2D)
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        LSCollider2D trigger2D = CreateBodylessCircle2D(context, Vector2d.Zero, isTrigger: true);
        string events3D = string.Empty;
        string events2D = string.Empty;
        body3D.Collider.OnMixedTriggerEnter += _ => events3D += "enter;";
        body3D.Collider.OnMixedTriggerStay += _ => events3D += "stay;";
        body3D.Collider.OnMixedTriggerExit += _ => events3D += "exit;";
        trigger2D.OnMixedTriggerEnter += _ =>
        {
            events2D += "enter;";
            if (deactivate2D)
                trigger2D.Deactivate();
            else
                body3D.Body.Deactivate();
        };
        trigger2D.OnMixedTriggerStay += _ => events2D += "stay;";
        trigger2D.OnMixedTriggerExit += _ => events2D += "exit;";

        Step(context);

        events3D.Should().Be("enter;stay;exit;");
        events2D.Should().Be("enter;exit;");
        context.MixedCollisions.ActivePairCount.Should().Be(0);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void MixedContactEnter_When2DCallbackDeactivatesEitherParticipant_ShouldSkip2DStayAndFinishBothLifecycles(
        bool deactivate2D)
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        string events3D = string.Empty;
        string events2D = string.Empty;
        body3D.Collider.OnMixedContactEnter += _ => events3D += "enter;";
        body3D.Collider.OnMixedContact += _ => events3D += "stay;";
        body3D.Collider.OnMixedContactExit += _ => events3D += "exit;";
        body2D.Collider.OnMixedContactEnter += _ =>
        {
            events2D += "enter;";
            if (deactivate2D)
                body2D.Deactivate();
            else
                body3D.Body.Deactivate();
        };
        body2D.Collider.OnMixedContact += _ => events2D += "stay;";
        body2D.Collider.OnMixedContactExit += _ => events2D += "exit;";

        Step(context);

        events3D.Should().Be("enter;stay;exit;");
        events2D.Should().Be("enter;exit;");
        context.MixedCollisions.ActivePairCount.Should().Be(0);
    }

    [Fact]
    public void Simulate_WhenQueuedMixedPairsBecomeImmovable_ShouldKeepContactsWithoutBuildingResponseConstraints()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Diagnostics.Enable(eventCapacity: 64, drawCommandCapacity: 0);
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        _ = CreateBodylessCircle2D(context, new Vector2d(-Fixed64.FromFraction(1, 4), Fixed64.Zero));
        _ = CreateBodylessCircle2D(context, new Vector2d(Fixed64.FromFraction(1, 4), Fixed64.Zero));
        Step(context);
        context.MixedCollisions.ActivePairCount.Should().Be(2);
        body3D.Body.SetPosition(Vector3d.Zero);
        body3D.Collider.OnMixedContact += _ => body3D.Body.FreezeAxes = BodyFreezeAxes3D.All;
        context.Diagnostics.Clear();

        Step(context);

        body3D.Body.CanTranslate.Should().BeFalse();
        body3D.Body.Position3d.Should().Be(Vector3d.Zero);
        context.MixedCollisions.ActivePairCount.Should().Be(2);
        CountMixedIslandEvents(context).Should().Be(0);
    }

    [Fact]
    public void Simulate_WithRootlessAndMovableQueuedPairs_ShouldSkipOnlyRootlessConstraint()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> rootless3D = CreateSphere3D(context, new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> movable3D = CreateSphere3D(context, new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));
        _ = CreateBodylessCircle2D(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));
        _ = CreateBodylessCircle2D(context, new Vector2d((Fixed64)3, Fixed64.Zero));
        Step(context);
        rootless3D.Body.SetPosition(new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero));
        movable3D.Body.SetPosition(new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));
        rootless3D.Collider.OnMixedContact += _ => rootless3D.Body.FreezeAxes = BodyFreezeAxes3D.All;

        Step(context);

        rootless3D.Body.CanTranslate.Should().BeFalse();
        rootless3D.Body.Position3d.Should().Be(new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero));
        movable3D.Body.Position3d.Should().NotBe(new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));
        context.MixedCollisions.ActivePairCount.Should().Be(2);
    }

    [Fact]
    public void Simulate_WithSleepingAndAwakeQueuedRoots_ShouldSolveOnlyAwakeRoot()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sleeping3D = CreateSphere3D(context, new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> awake3D = CreateSphere3D(context, new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));
        SolidBody2D sleeping2D = CreateCircle2D(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));
        SolidBody2D awake2D = CreateCircle2D(context, new Vector2d((Fixed64)3, Fixed64.Zero));
        Step(context);
        Vector3d sleepingPosition3D = new((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero);
        Vector3d awakePosition3D = new(Fixed64.FromFraction(11, 4), Fixed64.Zero, Fixed64.Zero);
        Vector2d sleepingPosition2D = new((Fixed64)(-3), Fixed64.Zero);
        Vector2d awakePosition2D = new((Fixed64)3, Fixed64.Zero);
        sleeping3D.Body.SetPosition(sleepingPosition3D);
        awake3D.Body.SetPosition(awakePosition3D);
        sleeping2D.SetPosition(sleepingPosition2D);
        awake2D.SetPosition(awakePosition2D);
        sleeping3D.Collider.OnMixedContact += _ =>
        {
            sleeping3D.Body.Sleep();
            sleeping2D.Sleep();
        };

        Step(context);

        sleeping3D.Body.IsSleeping.Should().BeTrue();
        sleeping2D.IsSleeping.Should().BeTrue();
        awake3D.Body.IsSleeping.Should().BeFalse();
        awake2D.IsSleeping.Should().BeFalse();
        sleeping3D.Body.Position3d.Should().Be(sleepingPosition3D);
        sleeping2D.Position.Should().Be(sleepingPosition2D);
        awake3D.Body.Position3d.Should().NotBe(awakePosition3D);
        awake2D.Position.Should().NotBe(awakePosition2D);
        context.MixedCollisions.ActivePairCount.Should().Be(2);
    }

    [Fact]
    public void Deactivate3DParticipant_WhenFirstExitRebindsLater2DParticipant_ShouldIgnoreStaleRemovalSnapshot()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D first2D = CreateCircle2D(context, Vector2d.Zero);
        SolidBody2D rebound2D = CreateCircle2D(context, Vector2d.Zero);
        Vector2d reboundPosition = new((Fixed64)7, Fixed64.Zero);
        int exits3D = 0;
        int exitsFirst2D = 0;
        int exitsRebound2D = 0;
        bool rebound = false;
        body3D.Collider.OnMixedContactExit += other =>
        {
            exits3D++;
            if (!rebound && ReferenceEquals(other, first2D.Collider))
            {
                rebound2D.Deactivate();
                rebound2D.Initialize(reboundPosition);
                rebound = true;
            }
        };
        first2D.Collider.OnMixedContactExit += _ => exitsFirst2D++;
        rebound2D.Collider.OnMixedContactExit += _ => exitsRebound2D++;
        Step(context);

        body3D.Body.Deactivate();

        exits3D.Should().Be(2);
        exitsFirst2D.Should().Be(1);
        exitsRebound2D.Should().Be(1);
        rebound2D.Active.Should().BeTrue();
        rebound2D.Position.Should().Be(reboundPosition);
        context.MixedCollisions.ActivePairCount.Should().Be(0);
    }

    [Fact]
    public void Deactivate2DParticipant_WhenFirstExitRebindsLater3DParticipant_ShouldIgnoreStaleRemovalSnapshot()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> first3D = CreateSphere3D(context, Vector3d.Zero);
        ScenarioBody<LSSphereCollider> rebound3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        Vector3d reboundPosition = new((Fixed64)7, Fixed64.Zero, Fixed64.Zero);
        int exits2D = 0;
        int exitsFirst3D = 0;
        int exitsRebound3D = 0;
        bool rebound = false;
        body2D.Collider.OnMixedContactExit += other =>
        {
            exits2D++;
            if (!rebound && ReferenceEquals(other, first3D.Collider))
            {
                rebound3D.Body.Deactivate();
                rebound3D.Body.Initialize(reboundPosition, FixedQuaternion.Identity);
                rebound = true;
            }
        };
        first3D.Collider.OnMixedContactExit += _ => exitsFirst3D++;
        rebound3D.Collider.OnMixedContactExit += _ => exitsRebound3D++;
        Step(context);

        body2D.Deactivate();

        exits2D.Should().Be(2);
        exitsFirst3D.Should().Be(1);
        exitsRebound3D.Should().Be(1);
        rebound3D.Body.Active.Should().BeTrue();
        rebound3D.Body.Position3d.Should().Be(reboundPosition);
        context.MixedCollisions.ActivePairCount.Should().Be(0);
    }

    [Fact]
    public void Deactivate3DParticipant_WhenLaterSnapshotPairShellIsReused_ShouldLeaveReplacementPairActive()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Settings.PoolingEnabled = true;
        ScenarioBody<LSSphereCollider> owner3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D first2D = CreateCircle2D(context, Vector2d.Zero);
        SolidBody2D retired2D = CreateCircle2D(context, Vector2d.Zero);
        ScenarioBody<LSSphereCollider> replacement3D = CreateSphere3D(
            context,
            new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));
        SolidBody2D replacement2D = CreateCircle2D(context, new Vector2d((Fixed64)6, Fixed64.Zero));
        int replacementEnters = 0;
        replacement2D.Collider.OnMixedContactEnter += _ => replacementEnters++;
        owner3D.Collider.OnMixedContactExit += other =>
        {
            if (!ReferenceEquals(other, first2D.Collider))
                return;

            retired2D.Deactivate();
            replacement2D.SetPosition(new Vector2d((Fixed64)4, Fixed64.Zero));
            Step(context);
        };
        Step(context);

        owner3D.Body.Deactivate();

        replacementEnters.Should().Be(1);
        replacement3D.Body.Active.Should().BeTrue();
        replacement2D.Active.Should().BeTrue();
        context.MixedCollisions.ActivePairCount.Should().Be(1);
        context.MixedCollisions.PooledPairCount.Should().Be(1);
    }

    [Fact]
    public void Simulate_WithSleepingMixedTriggerPair_ShouldRetainStayWithoutAwakeDistribution()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        LSCollider2D trigger2D = CreateBodylessCircle2D(context, Vector2d.Zero, isTrigger: true);
        int stays = 0;
        trigger2D.OnMixedTriggerStay += _ => stays++;
        Step(context);
        body3D.Body.Sleep();

        Step(context);

        stays.Should().Be(2);
        body3D.Body.IsSleeping.Should().BeTrue();
        context.MixedCollisions.ActivePairCount.Should().Be(1);
    }

    [Fact]
    public void Simulate_WithSleeping2DAgainst3DTrigger_ShouldRetainStayWithoutAwakeDistribution()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider trigger3D = CreateBodylessTrigger3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        int stays = 0;
        trigger3D.OnMixedTriggerStay += _ => stays++;
        Step(context);
        body2D.Sleep();

        Step(context);

        stays.Should().Be(2);
        body2D.IsSleeping.Should().BeTrue();
        context.MixedCollisions.ActivePairCount.Should().Be(1);
    }

    [Fact]
    public void Simulate_WhenSleepingMixedPairBecomesLocallyFiltered_ShouldRemoveUntouchedPair()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        int exits = 0;
        body3D.Collider.OnMixedContactExit += _ => exits++;
        body2D.Collider.OnMixedContactExit += _ => exits++;
        Step(context);
        body3D.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(body2D.Collider.Layer);
        body3D.Body.Sleep();
        body2D.Sleep();

        Step(context);

        exits.Should().Be(2);
        context.MixedCollisions.ActivePairCount.Should().Be(0);
    }

    [Fact]
    public void MixedEnter_When3DCallbackRebindsItself_ShouldSuppressOldLifetimeExit()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        Vector3d reboundPosition = new((Fixed64)7, Fixed64.Zero, Fixed64.Zero);
        string events3D = string.Empty;
        string events2D = string.Empty;
        body3D.Collider.OnMixedContactEnter += _ =>
        {
            events3D += "old-enter;";
            body3D.Body.Deactivate();
            body3D.Body.Initialize(reboundPosition, FixedQuaternion.Identity);
        };
        body3D.Collider.OnMixedContactExit += _ => events3D += "exit;";
        body2D.Collider.OnMixedContactEnter += _ => events2D += "enter;";
        body2D.Collider.OnMixedContactExit += _ => events2D += "exit;";

        Step(context);

        events3D.Should().Be("old-enter;");
        events2D.Should().BeEmpty();
        body3D.Body.Active.Should().BeTrue();
        body3D.Body.Position3d.Should().Be(reboundPosition);
        context.MixedCollisions.ActivePairCount.Should().Be(0);
    }

    private static LSCollider CreateBodylessTrigger3D(GravitasWorldContext context, Vector3d position)
    {
        var collider = new LSSphereCollider { IsTrigger = true };
        var agent = new TestMatterAgent(context, new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One));
        collider.InitializeWithNoBody(agent);
        return collider;
    }
}
