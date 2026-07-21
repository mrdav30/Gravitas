using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using System;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class Physics2DPairLifecycleHardeningTests
{
    [Fact]
    public void Simulate_WithNewSleepingTriggerAndAwakeBystander_ShouldNotifyWithoutWaking()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        SolidBody2D sleeper = CreateCircle(context, Vector2d.Zero, immovable: false);
        LSCircleCollider2D trigger = CreateBodylessCircle(
            context,
            new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero),
            isTrigger: true);
        SolidBody2D bystander = CreateCircle(context, Vector2d.Zero, immovable: false);
        bystander.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(sleeper.Collider.Layer);
        int entered = 0;
        int stayed = 0;
        trigger.OnTriggerEnter += other =>
        {
            other.Should().BeSameAs(sleeper.Collider);
            entered++;
        };
        trigger.OnTriggerStay += other =>
        {
            other.Should().BeSameAs(sleeper.Collider);
            stayed++;
        };
        sleeper.Sleep();
        Vector2d position = sleeper.Position;

        Step(context);

        sleeper.IsSleeping.Should().BeTrue();
        sleeper.Position.Should().Be(position);
        sleeper.Collider.TryGetCollisionPair(trigger.Id, out CollisionPair2D? pair).Should().BeTrue();
        pair!.IsColliding.Should().BeTrue();
        pair.LastFrame.Should().Be(context.FrameCount);
        entered.Should().Be(1);
        stayed.Should().Be(1);
    }

    [Fact]
    public void Simulate_WithNewSleepingSolidPairAndAwakeBystander_ShouldSkipPairCreation()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        SolidBody2D sleeper = CreateCircle(context, Vector2d.Zero, immovable: false);
        SolidBody2D support = CreateCircle(
            context,
            new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true);
        SolidBody2D bystander = CreateCircle(context, Vector2d.Zero, immovable: false);
        bystander.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(sleeper.Collider.Layer);
        int entered = 0;
        sleeper.Collider.OnContactEnter += _ => entered++;
        sleeper.Sleep();
        Vector2d position = sleeper.Position;

        Step(context);

        sleeper.IsSleeping.Should().BeTrue();
        sleeper.Position.Should().Be(position);
        sleeper.LinearVelocity.Should().Be(Vector2d.Zero);
        sleeper.Collider.TryGetCollisionPair(support.Collider.Id, out _).Should().BeFalse();
        entered.Should().Be(0);
    }

    [Fact]
    public void Simulate_WithSleepingPairAndAwakeBystander_ShouldKeepSingleResponsePairResting()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        SolidBody2D sleeper = CreateCircle(context, Vector2d.Zero, immovable: false);
        SolidBody2D support = CreateCircle(
            context,
            new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true);
        int entered = 0;
        int stayed = 0;
        sleeper.Collider.OnContactEnter += _ => entered++;
        sleeper.Collider.OnContact += _ => stayed++;

        Step(context);
        sleeper.SetPosition(Vector2d.Zero);
        sleeper.Sleep();
        SolidBody2D bystander = CreateCircle(context, Vector2d.Zero, immovable: false);
        bystander.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(sleeper.Collider.Layer);
        Vector2d position = sleeper.Position;
        Vector2d velocity = sleeper.LinearVelocity;

        Step(context);

        CollisionPair2D pair = GetPair(sleeper, support);
        sleeper.IsSleeping.Should().BeTrue();
        sleeper.Position.Should().Be(position);
        sleeper.LinearVelocity.Should().Be(velocity);
        pair.IsColliding.Should().BeTrue();
        pair.LastFrame.Should().Be(context.FrameCount);
        entered.Should().Be(1);
        stayed.Should().Be(1);
    }

    [Fact]
    public void Simulate_WithFilteredSleepingPairAndPoolingDisabled_ShouldExitAndNotReusePair()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        context.Settings.PoolingEnabled = false;
        SolidBody2D sleeper = CreateCircle(context, Vector2d.Zero, immovable: false);
        SolidBody2D firstSupport = CreateCircle(
            context,
            new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true);
        int exited = 0;
        sleeper.Collider.OnContactExit += _ => exited++;

        Step(context);
        CollisionPair2D removedPair = GetPair(sleeper, firstSupport);
        sleeper.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(firstSupport.Collider.Layer);
        sleeper.SetPosition(Vector2d.Zero);
        sleeper.Sleep();

        Step(context);

        removedPair.IsColliding.Should().BeFalse();
        sleeper.Collider.TryGetCollisionPair(firstSupport.Collider.Id, out _).Should().BeFalse();
        exited.Should().Be(1);

        sleeper.Collider.IgnoredCollisionLayers = PhysicsLayerMask.None;
        sleeper.Wake();
        SolidBody2D secondSupport = CreateCircle(
            context,
            new Vector2d(-Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true);
        Step(context);

        CollisionPair2D replacementPair = GetPair(sleeper, secondSupport);
        replacementPair.Should().NotBeSameAs(removedPair);
        replacementPair.IsColliding.Should().BeTrue();
        replacementPair.LastFrame.Should().Be(context.FrameCount);
    }

    [Fact]
    public void Simulate_WhenExpandedPairCallbackDeactivatesQueuedBodies_ShouldSkipStaleResponseState()
    {
        using GravitasWorldContext context = CreateContext(extent: 32);
        SolidBody2D bridge = CreateCircle(context, Vector2d.Zero, immovable: false);
        SolidBody2D retainedSupport = CreateCircle(
            context,
            new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true);
        Step(context);
        bridge.SetPosition(Vector2d.Zero);
        CollisionPair2D retainedPair = GetPair(bridge, retainedSupport);
        context.Collisions2D.ClearPartitionedCollider(retainedSupport.Collider, force: true).Should().BeTrue();

        SolidBody2D queued = CreateCircle(context, new Vector2d((Fixed64)8, Fixed64.Zero), immovable: false);
        SolidBody2D queuedSupport = CreateCircle(
            context,
            new Vector2d((Fixed64)8 + Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true);
        SolidBody2D bridgeSupport = CreateCircle(
            context,
            new Vector2d(-Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true);
        int callbackCount = 0;
        bridge.Collider.OnContact += other =>
        {
            if (!ReferenceEquals(other, retainedSupport))
                return;

            callbackCount++;
            queued.Deactivate();
            queuedSupport.Deactivate();
        };

        Step(context);

        callbackCount.Should().Be(1);
        queued.Active.Should().BeFalse();
        queuedSupport.Active.Should().BeFalse();
        queued.DynamicId.Should().Be(-1);
        queuedSupport.DynamicId.Should().Be(-1);
        bridge.Collider.TryGetCollisionPair(bridgeSupport.Collider.Id, out CollisionPair2D? bridgePair)
            .Should()
            .BeTrue();
        bridgePair!.IsColliding.Should().BeTrue();
        bridgePair.LastFrame.Should().Be(context.FrameCount);
        retainedPair.IsColliding.Should().BeTrue();
        retainedPair.LastFrame.Should().Be(context.FrameCount);
    }

    [Fact]
    public void Simulate_WhenExpandedPairCallbackRemovesCurrentPair_ShouldSkipItWithoutInvalidatingEnumeration()
    {
        using GravitasWorldContext context = CreateContext(extent: 32);
        SolidBody2D bridge = CreateCircle(context, Vector2d.Zero, immovable: false);
        SolidBody2D retainedSupport = CreateCircle(
            context,
            new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true);
        SolidBody2D retainedSecondSupport = CreateCircle(
            context,
            new Vector2d(-Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true);
        Step(context);
        bridge.SetPosition(Vector2d.Zero);
        CollisionPair2D removedPair = GetPair(bridge, retainedSupport);
        CollisionPair2D removedSecondPair = GetPair(bridge, retainedSecondSupport);
        context.Collisions2D.ClearPartitionedCollider(retainedSupport.Collider, force: true).Should().BeTrue();
        context.Collisions2D.ClearPartitionedCollider(retainedSecondSupport.Collider, force: true).Should().BeTrue();
        SolidBody2D bridgeSupport = CreateCircle(
            context,
            new Vector2d(Fixed64.Zero, Fixed64.FromFraction(3, 4)),
            immovable: true);
        int callbacks = 0;
        bridge.Collider.OnContact += other =>
        {
            if (!ReferenceEquals(other, retainedSupport))
                return;

            callbacks++;
            retainedSupport.Deactivate();
            retainedSecondSupport.Deactivate();
        };

        Action simulate = () => Step(context);

        simulate.Should().NotThrow();
        callbacks.Should().Be(1);
        retainedSupport.Active.Should().BeFalse();
        retainedSecondSupport.Active.Should().BeFalse();
        removedPair.IsColliding.Should().BeFalse();
        removedSecondPair.IsColliding.Should().BeFalse();
        bridge.Collider.TryGetCollisionPair(bridgeSupport.Collider.Id, out CollisionPair2D? currentPair)
            .Should()
            .BeTrue();
        currentPair!.IsColliding.Should().BeTrue();
        currentPair.LastFrame.Should().Be(context.FrameCount);
    }

    [Fact]
    public void Simulate_WhenRetainedTriggersAreDiscoveredThroughResponseExpansion_ShouldKeepBothPairOrders()
    {
        using GravitasWorldContext context = CreateContext(extent: 32);
        LSCircleCollider2D firstTrigger = CreateBodylessCircle(context, Vector2d.Zero, isTrigger: true);
        SolidBody2D firstBridge = CreateCircle(
            context,
            new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: false);
        SolidBody2D secondBridge = CreateCircle(context, new Vector2d((Fixed64)8, Fixed64.Zero), immovable: false);
        LSCircleCollider2D secondTrigger = CreateBodylessCircle(
            context,
            new Vector2d((Fixed64)8 + Fixed64.FromFraction(3, 4), Fixed64.Zero),
            isTrigger: true);
        int firstEntered = 0;
        int firstStayed = 0;
        int secondEntered = 0;
        int secondStayed = 0;
        firstTrigger.OnTriggerEnter += _ => firstEntered++;
        firstTrigger.OnTriggerStay += _ => firstStayed++;
        secondTrigger.OnTriggerEnter += _ => secondEntered++;
        secondTrigger.OnTriggerStay += _ => secondStayed++;

        Step(context);
        firstTrigger.TryGetCollisionPair(firstBridge.Collider.Id, out CollisionPair2D? firstPair).Should().BeTrue();
        secondBridge.Collider.TryGetCollisionPair(secondTrigger.Id, out CollisionPair2D? secondPair).Should().BeTrue();
        firstPair!.ColliderA.Should().BeSameAs(firstTrigger);
        secondPair!.ColliderB.Should().BeSameAs(secondTrigger);
        context.Collisions2D.ClearPartitionedCollider(firstTrigger, force: true).Should().BeTrue();
        context.Collisions2D.ClearPartitionedCollider(secondTrigger, force: true).Should().BeTrue();
        _ = CreateCircle(context, new Vector2d(Fixed64.FromFraction(3, 2), Fixed64.Zero), immovable: true);
        _ = CreateCircle(
            context,
            new Vector2d((Fixed64)8 - Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true);

        Step(context);

        firstPair.IsColliding.Should().BeTrue();
        secondPair.IsColliding.Should().BeTrue();
        firstPair.LastFrame.Should().Be(context.FrameCount);
        secondPair.LastFrame.Should().Be(context.FrameCount);
        firstEntered.Should().Be(1);
        secondEntered.Should().Be(1);
        firstStayed.Should().Be(2);
        secondStayed.Should().Be(2);
    }

    [Fact]
    public void Simulate_WhenExitCallbackRemovesCurrentPair_ShouldRecycleItOnlyOnce()
    {
        using GravitasWorldContext context = CreateContext(extent: 32);
        SolidBody2D first = CreateCircle(context, Vector2d.Zero, immovable: false);
        SolidBody2D removedSupport = CreateCircle(
            context,
            new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true);
        SolidBody2D removedSecondSupport = CreateCircle(
            context,
            new Vector2d(-Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true);
        int removedSupportId = removedSupport.Collider.Id;
        int exits = 0;

        Step(context);
        first.Collider.OnContactExit += other =>
        {
            exits++;
            if (ReferenceEquals(other, removedSupport))
            {
                removedSupport.Deactivate();
                removedSecondSupport.Deactivate();
            }
        };
        first.SetPosition(new Vector2d((Fixed64)(-4), Fixed64.Zero));

        Action cleanup = () => Step(context);

        cleanup.Should().NotThrow();
        exits.Should().Be(2);
        removedSupport.Active.Should().BeFalse();
        removedSecondSupport.Active.Should().BeFalse();
        first.Collider.TryGetCollisionPair(removedSupportId, out _).Should().BeFalse();

        SolidBody2D second = CreateCircle(context, Vector2d.Zero, immovable: false);
        SolidBody2D secondSupport = CreateCircle(
            context,
            new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true);
        SolidBody2D third = CreateCircle(context, new Vector2d((Fixed64)8, Fixed64.Zero), immovable: false);
        SolidBody2D thirdSupport = CreateCircle(
            context,
            new Vector2d((Fixed64)8 + Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true);

        Step(context);

        CollisionPair2D secondPair = GetPair(second, secondSupport);
        CollisionPair2D thirdPair = GetPair(third, thirdSupport);
        secondPair.Should().NotBeSameAs(thirdPair);
        secondPair.ColliderA.Should().BeOneOf(second.Collider, secondSupport.Collider);
        secondPair.ColliderB.Should().BeOneOf(second.Collider, secondSupport.Collider);
        thirdPair.ColliderA.Should().BeOneOf(third.Collider, thirdSupport.Collider);
        thirdPair.ColliderB.Should().BeOneOf(third.Collider, thirdSupport.Collider);
        secondPair.LastFrame.Should().Be(context.FrameCount);
        thirdPair.LastFrame.Should().Be(context.FrameCount);
    }

    [Fact]
    public void Deactivate_WhenExitCallbackRemovesAnotherOwnedPair_ShouldClearEveryPairOnce()
    {
        using GravitasWorldContext context = CreateContext(extent: 32);
        SolidBody2D owner = CreateCircle(context, Vector2d.Zero, immovable: false);
        SolidBody2D firstSupport = CreateCircle(
            context,
            new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true);
        SolidBody2D secondSupport = CreateCircle(
            context,
            new Vector2d(-Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true);
        int exits = 0;

        Step(context);
        CollisionPair2D firstPair = GetPair(owner, firstSupport);
        CollisionPair2D secondPair = GetPair(owner, secondSupport);
        owner.Collider.OnContactExit += other =>
        {
            exits++;
            if (ReferenceEquals(other, firstSupport))
                secondSupport.Deactivate();
        };

        Action deactivate = owner.Deactivate;

        deactivate.Should().NotThrow();
        exits.Should().Be(2);
        owner.Active.Should().BeFalse();
        secondSupport.Active.Should().BeFalse();
        firstPair.IsColliding.Should().BeFalse();
        secondPair.IsColliding.Should().BeFalse();
        firstSupport.Collider.CollisionPairHolderCount.Should().Be(0);
        context.Physics2D.ColliderCount.Should().Be(1);
    }

    [Fact]
    public void Simulate_WhenEnterCallbackRemovesCurrentPair_ShouldNotReuseQueuedPairReference()
    {
        EnterDeactivationResult unpooled = RunEnterDeactivationScenario(poolingEnabled: false);
        EnterDeactivationResult pooled = RunEnterDeactivationScenario(poolingEnabled: true);

        unpooled.FirstEvents.Should().Be("enter;exit;");
        unpooled.RemovedSupportEvents.Should().BeEmpty();
        unpooled.SecondEvents.Should().Be("enter;");
        pooled.Should().Be(unpooled);
    }

    [Fact]
    public void Simulate_WhenSecondEnterCallbackDeactivatesFirst_ShouldCompleteBothOrderedLifecycles()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        SolidBody2D first = CreateCircle(context, Vector2d.Zero, immovable: false);
        SolidBody2D second = CreateCircle(
            context,
            new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true);
        string firstEvents = string.Empty;
        string secondEvents = string.Empty;
        first.Collider.OnContactEnter += _ => firstEvents += "enter;";
        first.Collider.OnContactExit += _ => firstEvents += "exit;";
        second.Collider.OnContactEnter += _ =>
        {
            secondEvents += "enter;";
            first.Deactivate();
        };
        second.Collider.OnContactExit += _ => secondEvents += "exit;";

        Step(context);

        first.Active.Should().BeFalse();
        second.Active.Should().BeTrue();
        firstEvents.Should().Be("enter;exit;");
        secondEvents.Should().Be("enter;exit;");
        second.Collider.CollisionPairHolderCount.Should().Be(0);
    }

    [Fact]
    public void Simulate_WhenFirstEnterCallbackRebindsSecondBeforeItsTurn_ShouldNotNotifyReboundLifetime()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        SolidBody2D first = CreateCircle(context, Vector2d.Zero, immovable: false);
        SolidBody2D second = CreateCircle(
            context,
            new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true);
        int originalSecondId = second.Collider.Id;
        string firstEvents = string.Empty;
        string secondEvents = string.Empty;
        bool secondRebound = false;
        first.Collider.OnContactEnter += _ =>
        {
            firstEvents += "enter;";
            second.Deactivate();
            second.Initialize(new Vector2d((Fixed64)8, Fixed64.Zero));
            second.Collider.Id.Should().Be(originalSecondId);
            secondRebound = true;
        };
        first.Collider.OnContact += _ => firstEvents += "contact;";
        first.Collider.OnContactExit += _ => firstEvents += "exit;";
        second.Collider.OnContactEnter += _ => secondEvents += secondRebound ? "new-enter;" : "old-enter;";
        second.Collider.OnContact += _ => secondEvents += secondRebound ? "new-contact;" : "old-contact;";
        second.Collider.OnContactExit += _ => secondEvents += secondRebound ? "new-exit;" : "old-exit;";

        Step(context);

        second.Collider.Id.Should().Be(originalSecondId);
        second.Active.Should().BeTrue();
        second.Position.Should().Be(new Vector2d((Fixed64)8, Fixed64.Zero));
        firstEvents.Should().Be("enter;exit;");
        secondEvents.Should().BeEmpty();
        first.Collider.CollisionPairHolderCount.Should().Be(0);
        second.Collider.CollisionPairHolderCount.Should().Be(0);
    }

    [Fact]
    public void Simulate_WhenFirstEnterCallbackRebindsItself_ShouldNotResumeOldPairAgainstNewLifetime()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        SolidBody2D first = CreateCircle(context, Vector2d.Zero, immovable: false);
        SolidBody2D second = CreateCircle(
            context,
            new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true);
        int originalFirstId = first.Collider.Id;
        string firstEvents = string.Empty;
        string secondEvents = string.Empty;
        bool firstRebound = false;
        first.Collider.OnContactEnter += _ =>
        {
            firstEvents += "old-enter;";
            first.Deactivate();
            first.Initialize(new Vector2d((Fixed64)8, Fixed64.Zero));
            first.Collider.Id.Should().Be(originalFirstId);
            firstRebound = true;
        };
        first.Collider.OnContact += _ => firstEvents += firstRebound ? "new-contact;" : "old-contact;";
        first.Collider.OnContactExit += _ => firstEvents += firstRebound ? "new-exit;" : "old-exit;";
        second.Collider.OnContactEnter += _ => secondEvents += "enter;";
        second.Collider.OnContact += _ => secondEvents += "contact;";
        second.Collider.OnContactExit += _ => secondEvents += "exit;";

        Step(context);

        first.Collider.Id.Should().Be(originalFirstId);
        first.Active.Should().BeTrue();
        first.Position.Should().Be(new Vector2d((Fixed64)8, Fixed64.Zero));
        firstEvents.Should().Be("old-enter;");
        secondEvents.Should().BeEmpty();
        first.Collider.CollisionPairHolderCount.Should().Be(0);
        second.Collider.CollisionPairHolderCount.Should().Be(0);
    }

    [Fact]
    public void Simulate_WhenFirstEnterCallbackRebindsThenDeactivatesItself_ShouldNotNotifyReboundLifetime()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        SolidBody2D first = CreateCircle(context, Vector2d.Zero, immovable: false);
        SolidBody2D second = CreateCircle(
            context,
            new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true);
        int originalFirstId = first.Collider.Id;
        string firstEvents = string.Empty;
        string secondEvents = string.Empty;
        bool firstRebound = false;
        first.Collider.OnContactEnter += _ =>
        {
            firstEvents += "old-enter;";
            first.Deactivate();
            first.Initialize(new Vector2d((Fixed64)8, Fixed64.Zero));
            first.Collider.Id.Should().Be(originalFirstId);
            firstRebound = true;
            first.Deactivate();
        };
        first.Collider.OnContact += _ => firstEvents += firstRebound ? "new-contact;" : "old-contact;";
        first.Collider.OnContactExit += _ => firstEvents += firstRebound ? "new-exit;" : "old-exit;";
        second.Collider.OnContactEnter += _ => secondEvents += "enter;";
        second.Collider.OnContact += _ => secondEvents += "contact;";
        second.Collider.OnContactExit += _ => secondEvents += "exit;";

        Step(context);

        first.Active.Should().BeFalse();
        first.Collider.Id.Should().Be(-1);
        firstEvents.Should().Be("old-enter;");
        secondEvents.Should().BeEmpty();
        first.Collider.CollisionPairHolderCount.Should().Be(0);
        second.Collider.CollisionPairHolderCount.Should().Be(0);
    }

    [Fact]
    public void Simulate_WhenTriggerEnterDeactivatesBody_ShouldSkipStayAndPreservePendingExitEligibility()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        LSCircleCollider2D trigger = CreateBodylessCircle(context, Vector2d.Zero, isTrigger: true);
        SolidBody2D body = CreateCircle(
            context,
            new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: false);
        string triggerEvents = string.Empty;
        string bodyEvents = string.Empty;
        trigger.OnTriggerEnter += other =>
        {
            other.Should().BeSameAs(body.Collider);
            triggerEvents += "enter;";
            body.Deactivate();
        };
        trigger.OnTriggerStay += _ => triggerEvents += "stay;";
        trigger.OnTriggerExit += _ => triggerEvents += "exit;";
        body.Collider.OnTriggerEnter += _ => bodyEvents += "enter;";
        body.Collider.OnTriggerStay += _ => bodyEvents += "stay;";
        body.Collider.OnTriggerExit += _ => bodyEvents += "exit;";

        Step(context);

        body.Active.Should().BeFalse();
        triggerEvents.Should().Be("enter;exit;");
        bodyEvents.Should().BeEmpty();
        trigger.CollisionPairHolderCount.Should().Be(0);
    }

    [Fact]
    public void Simulate_WhenTriggerEnterDeactivatesItself_ShouldSkipStayAndPreservePendingExitEligibility()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        LSCircleCollider2D trigger = CreateBodylessCircle(context, Vector2d.Zero, isTrigger: true);
        SolidBody2D body = CreateCircle(
            context,
            new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: false);
        string triggerEvents = string.Empty;
        string bodyEvents = string.Empty;
        trigger.OnTriggerEnter += _ =>
        {
            triggerEvents += "enter;";
            trigger.Deactivate();
        };
        trigger.OnTriggerStay += _ => triggerEvents += "stay;";
        trigger.OnTriggerExit += _ => triggerEvents += "exit;";
        body.Collider.OnTriggerEnter += _ => bodyEvents += "enter;";
        body.Collider.OnTriggerStay += _ => bodyEvents += "stay;";
        body.Collider.OnTriggerExit += _ => bodyEvents += "exit;";

        Step(context);

        trigger.IsActive.Should().BeFalse();
        triggerEvents.Should().Be("enter;exit;");
        bodyEvents.Should().BeEmpty();
        body.Collider.CollisionPairHolderCount.Should().Be(0);
    }

    [Fact]
    public void Simulate_WhenSecondEnterCallbackRebindsItself_ShouldNotSendPendingExitToNewLifetime()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        SolidBody2D first = CreateCircle(context, Vector2d.Zero, immovable: false);
        SolidBody2D second = CreateCircle(
            context,
            new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true);
        int originalSecondId = second.Collider.Id;
        string firstEvents = string.Empty;
        string secondEvents = string.Empty;
        bool secondRebound = false;
        first.Collider.OnContactEnter += _ => firstEvents += "enter;";
        first.Collider.OnContact += _ => firstEvents += "contact;";
        first.Collider.OnContactExit += _ => firstEvents += "exit;";
        second.Collider.OnContactEnter += _ =>
        {
            secondEvents += "old-enter;";
            second.Deactivate();
            second.Initialize(new Vector2d((Fixed64)8, Fixed64.Zero));
            second.Collider.Id.Should().Be(originalSecondId);
            secondRebound = true;
        };
        second.Collider.OnContact += _ => secondEvents += secondRebound ? "new-contact;" : "old-contact;";
        second.Collider.OnContactExit += _ => secondEvents += secondRebound ? "new-exit;" : "old-exit;";

        Step(context);

        second.Collider.Id.Should().Be(originalSecondId);
        second.Active.Should().BeTrue();
        second.Position.Should().Be(new Vector2d((Fixed64)8, Fixed64.Zero));
        firstEvents.Should().Be("enter;contact;exit;");
        secondEvents.Should().Be("old-enter;");
        first.Collider.CollisionPairHolderCount.Should().Be(0);
        second.Collider.CollisionPairHolderCount.Should().Be(0);
    }

    [Fact]
    public void Simulate_WhenEnterCallbackRecursivelyCreatesPair_ShouldNotRecycleNotifyingPair()
    {
        using GravitasWorldContext context = CreateContext(extent: 32);
        context.Settings.PoolingEnabled = true;
        SolidBody2D first = CreateCircle(context, Vector2d.Zero, immovable: false);
        SolidBody2D removedSupport = CreateCircle(
            context,
            new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true);
        CollisionPair2D? notifyingPair = null;
        CollisionPair2D? recursivePair = null;
        SolidBody2D? recursiveFirst = null;
        SolidBody2D? recursiveSecond = null;
        string firstEvents = string.Empty;
        first.Collider.OnContactEnter += _ =>
        {
            firstEvents += "enter;";
            notifyingPair = GetPair(first, removedSupport);
            removedSupport.Deactivate();
            recursiveFirst = CreateCircle(context, new Vector2d((Fixed64)8, Fixed64.Zero), immovable: false);
            recursiveSecond = CreateCircle(
                context,
                new Vector2d((Fixed64)8 + Fixed64.FromFraction(3, 4), Fixed64.Zero),
                immovable: true);
            Step(context);
            recursivePair = GetPair(recursiveFirst, recursiveSecond);
        };
        first.Collider.OnContact += _ => firstEvents += "contact;";
        first.Collider.OnContactExit += _ => firstEvents += "exit;";

        Step(context);

        notifyingPair.Should().NotBeNull();
        recursivePair.Should().NotBeNull();
        recursivePair.Should().NotBeSameAs(notifyingPair);
        recursivePair!.ColliderA.Should().BeOneOf(recursiveFirst!.Collider, recursiveSecond!.Collider);
        recursivePair.ColliderB.Should().BeOneOf(recursiveFirst.Collider, recursiveSecond.Collider);
        firstEvents.Should().Be("enter;exit;");
        first.Collider.CollisionPairHolderCount.Should().Be(0);
    }

    [Fact]
    public void MarkCollidingDeferred_WhenExitCallbacksThrow_ShouldRetainGuardAndAggregateInPairOrder()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        SolidBody2D first = CreateCircle(context, Vector2d.Zero, immovable: false);
        SolidBody2D second = CreateCircle(
            context,
            new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true);
        var pair = new CollisionPair2D(first.Collider, second.Collider);
        pair.Manifold.SetContact(Vector2d.Zero, Vector2d.Zero, Fixed64.Half, Vector2d.Right);
        int exitsA = 0;
        int exitsB = 0;
        bool guardedA = false;
        bool guardedB = false;
        pair.ColliderB.OnContactEnter += _ => pair.MarkSeparated();
        pair.ColliderA.OnContactExit += _ =>
        {
            exitsA++;
            guardedA = pair.IsNotificationInProgress;
            throw new InvalidOperationException("collider A exit failure");
        };
        pair.ColliderB.OnContactExit += _ =>
        {
            exitsB++;
            guardedB = pair.IsNotificationInProgress;
            throw new ArgumentException("collider B exit failure");
        };

        Action notify = () => pair.MarkCollidingDeferred(frame: 1);

        AggregateException exception = notify.Should().Throw<AggregateException>().Which.Flatten();
        exception.InnerExceptions.Should().HaveCount(2);
        exception.InnerExceptions[0].Should().BeOfType<InvalidOperationException>()
            .Which.Message.Should().Be("collider A exit failure");
        exception.InnerExceptions[1].Should().BeOfType<ArgumentException>()
            .Which.Message.Should().Be("collider B exit failure");
        exitsA.Should().Be(1);
        exitsB.Should().Be(1);
        guardedA.Should().BeTrue();
        guardedB.Should().BeTrue();
        pair.IsNotificationInProgress.Should().BeFalse();
        pair.IsColliding.Should().BeFalse();

        Action retry = pair.MarkSeparated;

        retry.Should().NotThrow();
        exitsA.Should().Be(1);
        exitsB.Should().Be(1);
    }

    [Fact]
    public void MarkCollidingDeferred_WhenLaterStaySeparatesPair_ShouldExitBothAdmittedSides()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        SolidBody2D first = CreateCircle(context, Vector2d.Zero, immovable: false);
        SolidBody2D second = CreateCircle(
            context,
            new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true);
        var pair = new CollisionPair2D(first.Collider, second.Collider);
        pair.Manifold.SetContact(Vector2d.Zero, Vector2d.Zero, Fixed64.Half, Vector2d.Right);
        int entersB = 0;
        int exitsA = 0;
        int exitsB = 0;
        pair.ColliderB.OnContactEnter += _ => entersB++;
        pair.ColliderA.OnContactExit += _ => exitsA++;
        pair.ColliderB.OnContactExit += _ => exitsB++;
        pair.MarkCollidingDeferred(frame: 1);
        pair.ColliderA.OnContact += _ => pair.MarkSeparated();

        pair.MarkCollidingDeferred(frame: 2);

        entersB.Should().Be(1);
        exitsA.Should().Be(1);
        exitsB.Should().Be(1);
        pair.IsColliding.Should().BeFalse();
        pair.IsNotificationInProgress.Should().BeFalse();
    }

    [Fact]
    public void MarkSeparated_AfterFirstEnterThrows_ShouldNotExitUnadmittedSecondSide()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        SolidBody2D first = CreateCircle(context, Vector2d.Zero, immovable: false);
        SolidBody2D second = CreateCircle(
            context,
            new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true);
        var pair = new CollisionPair2D(first.Collider, second.Collider);
        pair.Manifold.SetContact(Vector2d.Zero, Vector2d.Zero, Fixed64.Half, Vector2d.Right);
        int entersB = 0;
        int exitsA = 0;
        int exitsB = 0;
        pair.ColliderA.OnContactEnter += _ => throw new InvalidOperationException("collider A enter failure");
        pair.ColliderA.OnContactExit += _ => exitsA++;
        pair.ColliderB.OnContactEnter += _ => entersB++;
        pair.ColliderB.OnContactExit += _ => exitsB++;

        Action enter = () => pair.MarkCollidingDeferred(frame: 1);

        InvalidOperationException exception = enter.Should().Throw<InvalidOperationException>()
            .WithMessage("collider A enter failure")
            .Which;
        exception.StackTrace.Should().Contain(nameof(MarkSeparated_AfterFirstEnterThrows_ShouldNotExitUnadmittedSecondSide));

        pair.MarkSeparated();

        entersB.Should().Be(0);
        exitsA.Should().Be(1);
        exitsB.Should().Be(0);
        pair.IsColliding.Should().BeFalse();
        pair.IsNotificationInProgress.Should().BeFalse();
    }

    private static EnterDeactivationResult RunEnterDeactivationScenario(bool poolingEnabled)
    {
        using GravitasWorldContext context = CreateContext(extent: 32);
        context.Settings.PoolingEnabled = poolingEnabled;
        SolidBody2D first = CreateCircle(context, Vector2d.Zero, immovable: false);
        SolidBody2D removedSupport = CreateCircle(
            context,
            new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true);
        SolidBody2D second = CreateCircle(context, new Vector2d((Fixed64)8, Fixed64.Zero), immovable: false);
        SolidBody2D secondSupport = CreateCircle(
            context,
            new Vector2d((Fixed64)8 + Fixed64.FromFraction(3, 4), Fixed64.Zero),
            immovable: true);
        int removedSupportId = removedSupport.Collider.Id;
        string firstEvents = string.Empty;
        string removedSupportEvents = string.Empty;
        string secondEvents = string.Empty;
        first.Collider.OnContactEnter += other =>
        {
            other.Should().BeSameAs(removedSupport);
            firstEvents += "enter;";
            removedSupport.Deactivate();
        };
        first.Collider.OnContactExit += _ => firstEvents += "exit;";
        removedSupport.Collider.OnContactEnter += _ => removedSupportEvents += "enter;";
        removedSupport.Collider.OnContactExit += _ => removedSupportEvents += "exit;";
        second.Collider.OnContactEnter += other =>
        {
            other.Should().BeSameAs(secondSupport);
            secondEvents += "enter;";
        };
        second.Collider.OnContactExit += _ => secondEvents += "exit;";

        Step(context);

        first.Collider.TryGetCollisionPair(removedSupportId, out _).Should().BeFalse();
        CollisionPair2D secondPair = GetPair(second, secondSupport);
        secondPair.IsColliding.Should().BeTrue();
        secondPair.LastFrame.Should().Be(context.FrameCount);
        return new EnterDeactivationResult(
            second.Position,
            second.LinearVelocity,
            firstEvents,
            removedSupportEvents,
            secondEvents,
            context.Physics2D.ColliderCount);
    }

    private static GravitasWorldContext CreateContext(int extent, int frameRate = 4)
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.SetFrameRate(frameRate);
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-16), Fixed64.Zero, (Fixed64)(-16)),
                new Vector3d((Fixed64)extent, Fixed64.Zero, (Fixed64)extent)),
            out _).Should().BeTrue();
        return context;
    }

    private static void Step(GravitasWorldContext context)
    {
        context.Simulate();
        context.LateSimulate();
    }

    private static CollisionPair2D GetPair(SolidBody2D first, SolidBody2D second)
    {
        if (first.Collider.TryGetCollisionPair(second.Collider.Id, out CollisionPair2D? firstPair) && firstPair != null)
            return firstPair;

        second.Collider.TryGetCollisionPair(first.Collider.Id, out CollisionPair2D? secondPair).Should().BeTrue();
        return secondPair!;
    }

    private static SolidBody2D CreateCircle(GravitasWorldContext context, Vector2d position, bool immovable)
    {
        return CreateBody(context, new LSCircleCollider2D(Fixed64.Half), position, immovable);
    }

    private static LSCircleCollider2D CreateBodylessCircle(
        GravitasWorldContext context,
        Vector2d position,
        bool isTrigger)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var collider = new LSCircleCollider2D(Fixed64.Half)
        {
            IsTrigger = isTrigger
        };
        collider.InitializeWithNoBody(agent);
        return collider;
    }

    private static SolidBody2D CreateBody(
        GravitasWorldContext context,
        LSCollider2D collider,
        Vector2d position,
        bool immovable)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(
            position,
            motionType: immovable ? BodyMotionType.Static : BodyMotionType.Dynamic);
        return body;
    }

    private readonly record struct EnterDeactivationResult(
        Vector2d Position,
        Vector2d Velocity,
        string FirstEvents,
        string RemovedSupportEvents,
        string SecondEvents,
        int ColliderCount);
}
