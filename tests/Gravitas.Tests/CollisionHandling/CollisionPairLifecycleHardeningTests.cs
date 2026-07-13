using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Support;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CollisionPairLifecycleHardeningTests
{
    [Fact]
    public void Simulate_WhenEnterCallbackCreatesRecursivePair_ShouldNotReuseNotifyingShell()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Settings.PoolingEnabled = true;
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> removedSupport = scenario.CreateSphere(
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        CollisionPair? notifyingPair = null;
        CollisionPair? recursivePair = null;
        ScenarioBody<LSSphereCollider> recursiveFirst = default;
        ScenarioBody<LSSphereCollider> recursiveSecond = default;
        string firstEvents = string.Empty;
        first.Collider.OnContactEnter += _ =>
        {
            firstEvents += "enter;";
            notifyingPair = GetPair(first.Collider, removedSupport.Collider);
            removedSupport.Body.Deactivate();
            recursiveFirst = scenario.CreateSphere(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
            recursiveSecond = scenario.CreateSphere(
                new Vector3d((Fixed64)8 + Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
                immovable: true);
            Step(scenario.Context);
            recursivePair = GetPair(recursiveFirst.Collider, recursiveSecond.Collider);
        };
        first.Collider.OnContact += _ => firstEvents += "contact;";
        first.Collider.OnContactExit += _ => firstEvents += "exit;";

        Step(scenario.Context);

        notifyingPair.Should().NotBeNull();
        recursivePair.Should().NotBeNull();
        recursivePair.Should().NotBeSameAs(notifyingPair);
        recursivePair!.ColliderA.Should().BeOneOf(recursiveFirst.Collider, recursiveSecond.Collider);
        recursivePair.ColliderB.Should().BeOneOf(recursiveFirst.Collider, recursiveSecond.Collider);
        firstEvents.Should().Be("enter;exit;");
    }

    [Fact]
    public void Simulate_WhenContactCallbackThrows_ShouldRestoreUnprocessedQueueEntries()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        _ = scenario.CreateSphere(
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        _ = scenario.CreateSphere(
            new Vector3d((Fixed64)8 + Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        string secondEvents = string.Empty;
        first.Collider.OnContactEnter += ThrowFromContact;
        second.Collider.OnContactEnter += _ => secondEvents += "enter;";
        second.Collider.OnContact += _ => secondEvents += "contact;";

        Action firstStep = () => Step(scenario.Context);
        firstStep.Should().Throw<InvalidOperationException>().WithMessage("callback failure");
        first.Collider.OnContactEnter -= ThrowFromContact;

        Step(scenario.Context);

        secondEvents.Should().Be("enter;contact;");

        static void ThrowFromContact(SolidBody _) => throw new InvalidOperationException("callback failure");
    }

    [Fact]
    public void Simulate_WhenFirstTriggerEnterThrows_ShouldResumeOnlyUnadmittedSide()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var trigger = new LSSphereCollider { IsTrigger = true };
        scenario.InitializeStaticCollider(trigger, Vector3d.Zero);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        int triggerEnters = 0;
        int triggerStays = 0;
        int bodyEnters = 0;
        int bodyStays = 0;
        trigger.OnTriggerEnter += _ =>
        {
            triggerEnters++;
            if (triggerEnters == 1)
                throw new InvalidOperationException("trigger failure");
        };
        trigger.OnTriggerStay += _ => triggerStays++;
        body.Collider.OnTriggerEnter += _ => bodyEnters++;
        body.Collider.OnTriggerStay += _ => bodyStays++;

        Action firstStep = () => Step(scenario.Context);
        firstStep.Should().Throw<InvalidOperationException>().WithMessage("trigger failure");

        Step(scenario.Context);

        triggerEnters.Should().Be(1);
        triggerStays.Should().Be(1);
        bodyEnters.Should().Be(1);
        bodyStays.Should().Be(1);
    }

    [Fact]
    public void GetCollisionPair_WithInvalidAndReversedIds_ShouldReturnNullOrStableOwnedPair()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));

        scenario.Context.Physics.GetCollisionPair(-1, second.Collider.Id).Should().BeNull();
        scenario.Context.Physics.GetCollisionPair(first.Collider.Id, -1).Should().BeNull();
        CollisionPair pair = scenario.Context.Physics.GetCollisionPair(second.Collider.Id, first.Collider.Id)!;

        pair.Should().NotBeNull();
        scenario.Context.Physics.GetCollisionPair(first.Collider.Id, second.Collider.Id).Should().BeSameAs(pair);
        pair.ColliderA.TryGetCollisionPair(pair.ColliderB.Id, out CollisionPair? owned).Should().BeTrue();
        owned.Should().BeSameAs(pair);
        pair.ColliderB.CollisionPairHolderCount.Should().Be(1);
    }

    [Fact]
    public void GetCollisionPair_WhenOnlySecondColliderOwnsPair_ShouldResolveHolderSide()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));
        CollisionPair pair = scenario.CreatePair(second.Collider, first.Collider);
        second.Collider.TryAddCollisionPair(first.Collider.Id, pair).Should().BeTrue();
        first.Collider.TryAddCollisionPairHolder(second.Collider.Id).Should().BeTrue();

        scenario.Context.Physics.GetCollisionPair(first.Collider.Id, second.Collider.Id).Should().BeSameAs(pair);
    }

    [Fact]
    public void IsLayerCollisionDisabled_WithSmallMatrix_ShouldHandleEachBoundsAndValueOutcome()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var settings = new PhysicsSettings(60, new[,] { { true } });
        scenario.Context.ApplySettings(settings);
        var inRange = new PhysicsLayer(0);
        var outOfRange = new PhysicsLayer(1);

        scenario.Context.Physics.IsLayerCollisionDisabled(outOfRange, inRange).Should().BeFalse();
        scenario.Context.Physics.IsLayerCollisionDisabled(inRange, outOfRange).Should().BeFalse();
        scenario.Context.Physics.IsLayerCollisionDisabled(inRange, inRange).Should().BeFalse();
        settings.CollisionMatrix[0, 0] = false;
        scenario.Context.Physics.IsLayerCollisionDisabled(inRange, inRange).Should().BeTrue();
    }

    [Fact]
    public void ProcessActiveCollisionPairs_WhenUntouchedPairExpires_ShouldDeactivateIt()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        CollisionPair pair = scenario.CreatePair(first.Collider, second.Collider);
        pair.UpdateCollision();
        for (int i = 0; i < scenario.Context.FrameRate * 9; i++)
            scenario.Context.Simulate();

        scenario.Context.Physics.CompleteLateSimulatePhysicsStep();

        pair.Active.Should().BeFalse();
    }

    [Fact]
    public void ProcessActiveCollisionPairs_WhenQueuedColliderDeactivates_ShouldDeactivatePair()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        CollisionPair pair = scenario.CreatePair(first.Collider, second.Collider);
        pair.UpdateCollision();
        first.Body.Deactivate();

        scenario.Context.Physics.CompleteLateSimulatePhysicsStep();

        pair.Active.Should().BeFalse();
    }

    [Fact]
    public void CreatePair_WhenPoolingIsDisabledAfterCachePopulation_ShouldNotReuseCachedShell()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Settings.PoolingEnabled = true;
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));
        CollisionPair cached = scenario.Context.Physics.GetCollisionPair(first.Collider.Id, second.Collider.Id)!;
        scenario.Context.Physics.FullDeactivateCollisionPair(cached);
        scenario.Context.Settings.PoolingEnabled = false;
        ScenarioBody<LSSphereCollider> third = scenario.CreateSphere(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));

        CollisionPair unpooled = scenario.Context.Physics.GetCollisionPair(first.Collider.Id, third.Collider.Id)!;

        unpooled.Should().NotBeSameAs(cached);
        scenario.Context.Physics.FullDeactivateCollisionPair(unpooled);
        unpooled.Active.Should().BeFalse();
    }

    [Fact]
    public void Deactivate_WhenExitCallbackRemovesAnotherHolderPair_ShouldClearEveryPairOnce()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(
            new Vector3d(-Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> support = scenario.CreateSphere(Vector3d.Zero, immovable: true);
        int exits = 0;
        Step(scenario.Context);
        support.Collider.CollisionPairHolderCount.Should().Be(2);
        support.Collider.OnContactExit += other =>
        {
            exits++;
            if (ReferenceEquals(other, first.Body))
                second.Body.Deactivate();
        };

        Action deactivate = support.Body.Deactivate;

        deactivate.Should().NotThrow();
        exits.Should().Be(2);
        first.Collider.CollisionPairCount.Should().Be(0);
        second.Collider.CollisionPairCount.Should().Be(0);
        support.Collider.CollisionPairHolderCount.Should().Be(0);
    }

    [Fact]
    public void Simulate_WhenLaterQueuedPairIsDeactivatedBeforeNotification_ShouldEmitNoPhantomExit()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        _ = scenario.CreateSphere(
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> secondSupport = scenario.CreateSphere(
            new Vector3d((Fixed64)8 + Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        string secondEvents = string.Empty;
        first.Collider.OnContactEnter += _ => secondSupport.Body.Deactivate();
        second.Collider.OnContactEnter += _ => secondEvents += "enter;";
        second.Collider.OnContactExit += _ => secondEvents += "exit;";

        Step(scenario.Context);

        secondEvents.Should().BeEmpty();
        second.Collider.CollisionPairCount.Should().Be(0);
        secondSupport.Body.Active.Should().BeFalse();
    }

    [Fact]
    public void Simulate_WhenSecondEnterCallbackDeactivatesFirst_ShouldFinishOnlyAdmittedLifecycles()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        string firstEvents = string.Empty;
        string secondEvents = string.Empty;
        first.Collider.OnContactEnter += _ => firstEvents += "enter;";
        first.Collider.OnContact += _ => firstEvents += "contact;";
        first.Collider.OnContactExit += _ => firstEvents += "exit;";
        second.Collider.OnContactEnter += _ =>
        {
            secondEvents += "enter;";
            first.Body.Deactivate();
        };
        second.Collider.OnContact += _ => secondEvents += "contact;";
        second.Collider.OnContactExit += _ => secondEvents += "exit;";

        Step(scenario.Context);

        firstEvents.Should().Be("enter;contact;exit;");
        secondEvents.Should().Be("enter;exit;");
        first.Body.Active.Should().BeFalse();
        second.Collider.CollisionPairHolderCount.Should().Be(0);
    }

    [Fact]
    public void Simulate_WhenTriggerEnterDeactivatesBody_ShouldSkipStayAndExitOnlyAdmittedSide()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var trigger = new LSSphereCollider { IsTrigger = true };
        scenario.InitializeStaticCollider(trigger, Vector3d.Zero);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        string bodyEvents = string.Empty;
        string triggerEvents = string.Empty;
        body.Collider.OnTriggerEnter += _ => bodyEvents += "enter;";
        body.Collider.OnTriggerStay += _ => bodyEvents += "stay;";
        body.Collider.OnTriggerExit += _ => bodyEvents += "exit;";
        trigger.OnTriggerEnter += _ =>
        {
            triggerEvents += "enter;";
            body.Body.Deactivate();
        };
        trigger.OnTriggerStay += _ => triggerEvents += "stay;";
        trigger.OnTriggerExit += _ => triggerEvents += "exit;";

        Step(scenario.Context);

        bodyEvents.Should().BeEmpty();
        triggerEvents.Should().Be("enter;exit;");
        trigger.CollisionPairHolderCount.Should().Be(0);
    }

    [Fact]
    public void Deactivate_WhenPreviouslyNotifiedPairSeparatedBeforeQueueTurn_ShouldDeliverExistingExit()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> secondSupport = scenario.CreateSphere(
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        string secondEvents = string.Empty;
        second.Collider.OnContactEnter += _ => secondEvents += "enter;";
        second.Collider.OnContactExit += _ => secondEvents += "exit;";
        Step(scenario.Context);
        secondEvents.Should().Be("enter;");
        CollisionPair pair = GetPair(second.Collider, secondSupport.Collider);
        secondSupport.Body.ResetPosition(new Vector3d((Fixed64)12, Fixed64.Zero, Fixed64.Zero));
        pair.UpdateCollisionDeferred();

        secondSupport.Body.Deactivate();

        secondEvents.Should().Be("enter;exit;");
        second.Collider.CollisionPairCount.Should().Be(0);
    }

    [Fact]
    public void Deactivate_WhenExitCallbackStepsRecursively_ShouldNotRediscoverDyingCollider()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> owner = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> support = scenario.CreateSphere(
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        Step(scenario.Context);
        owner.Collider.OnContactExit += _ => Step(scenario.Context);

        owner.Body.Deactivate();

        owner.Collider.CollisionPairCount.Should().Be(0);
        owner.Collider.CollisionPairHolderCount.Should().Be(0);
        support.Collider.CollisionPairCount.Should().Be(0);
        support.Collider.CollisionPairHolderCount.Should().Be(0);
    }

    [Fact]
    public void Simulate_WhenContactCallbackStepsRecursively_ShouldNotifyAlreadyQueuedPairOnce()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        _ = scenario.CreateSphere(
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        _ = scenario.CreateSphere(
            new Vector3d((Fixed64)8 + Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        string secondEvents = string.Empty;
        first.Collider.OnContactEnter += _ => Step(scenario.Context);
        second.Collider.OnContactEnter += _ => secondEvents += "enter;";
        second.Collider.OnContact += _ => secondEvents += "contact;";

        Step(scenario.Context);

        secondEvents.Should().Be("enter;contact;");
    }

    [Fact]
    public void Simulate_WhenQueuedPairShellIsReused_ShouldProcessReplacementOnce()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Settings.PoolingEnabled = true;
        ScenarioBody<LSSphereCollider> retired = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> retiredSupport = scenario.CreateSphere(
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            immovable: true);

        Step(scenario.Context);
        CollisionPair retiredPair = GetPair(retired.Collider, retiredSupport.Collider);
        retiredSupport.Body.Deactivate();

        ScenarioBody<LSSphereCollider> replacement = scenario.CreateSphere(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> replacementSupport = scenario.CreateSphere(
            new Vector3d((Fixed64)8 + Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        string events = string.Empty;
        replacement.Collider.OnContactEnter += _ => events += "enter;";
        replacement.Collider.OnContact += _ => events += "contact;";

        Step(scenario.Context);

        CollisionPair replacementPair = GetPair(replacement.Collider, replacementSupport.Collider);
        replacementPair.Should().BeSameAs(retiredPair);
        events.Should().Be("enter;contact;");
        replacementPair.Active.Should().BeTrue();
    }

    [Fact]
    public void Simulate_WhenFirstEnterCallbackRebindsSecond_ShouldNotNotifyReboundLifetime()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        int originalSecondId = second.Collider.Id;
        string firstEvents = string.Empty;
        string secondEvents = string.Empty;
        bool secondRebound = false;
        first.Collider.OnContactEnter += _ =>
        {
            firstEvents += "enter;";
            second.Body.Deactivate();
            second.Body.Initialize(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity);
            second.Collider.Id.Should().Be(originalSecondId);
            secondRebound = true;
        };
        first.Collider.OnContact += _ => firstEvents += "contact;";
        first.Collider.OnContactExit += _ => firstEvents += "exit;";
        second.Collider.OnContactEnter += _ => secondEvents += secondRebound ? "new-enter;" : "old-enter;";
        second.Collider.OnContact += _ => secondEvents += secondRebound ? "new-contact;" : "old-contact;";
        second.Collider.OnContactExit += _ => secondEvents += secondRebound ? "new-exit;" : "old-exit;";

        Step(scenario.Context);

        second.Collider.Id.Should().Be(originalSecondId);
        second.Body.Active.Should().BeTrue();
        second.Body.Position3d.Should().Be(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        firstEvents.Should().Be("enter;exit;");
        secondEvents.Should().BeEmpty();
        first.Collider.CollisionPairCount.Should().Be(0);
        second.Collider.CollisionPairCount.Should().Be(0);
    }

    [Fact]
    public void Simulate_WhenFirstStayCallbackRebindsSecond_ShouldExitFirstWithoutNotifyingReboundLifetime()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        int originalSecondId = second.Collider.Id;
        int firstStayCount = 0;
        bool secondRebound = false;
        string firstEvents = string.Empty;
        string secondEvents = string.Empty;
        first.Collider.OnContactEnter += _ => firstEvents += "enter;";
        first.Collider.OnContact += _ =>
        {
            firstEvents += "contact;";
            firstStayCount++;
            if (firstStayCount != 2)
                return;

            second.Body.Deactivate();
            second.Body.Initialize(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity);
            second.Collider.Id.Should().Be(originalSecondId);
            secondRebound = true;
        };
        first.Collider.OnContactExit += _ => firstEvents += secondRebound ? "exit;" : "early-exit;";
        second.Collider.OnContactEnter += _ => secondEvents += secondRebound ? "new-enter;" : "old-enter;";
        second.Collider.OnContact += _ => secondEvents += secondRebound ? "new-contact;" : "old-contact;";
        second.Collider.OnContactExit += _ => secondEvents += secondRebound ? "new-exit;" : "old-exit;";

        Step(scenario.Context);

        CollisionPair originalPair = GetPair(first.Collider, second.Collider);
        originalPair.ColliderA.Should().BeSameAs(first.Collider);
        originalPair.ColliderB.Should().BeSameAs(second.Collider);
        originalPair.Active.Should().BeTrue();
        first.Collider.CollisionPairCount.Should().Be(1);
        second.Collider.CollisionPairHolderCount.Should().Be(1);
        firstEvents.Should().Be("enter;contact;");
        secondEvents.Should().Be("old-enter;old-contact;");

        Step(scenario.Context);

        second.Collider.Id.Should().Be(originalSecondId);
        second.Body.Active.Should().BeTrue();
        second.Body.Position3d.Should().Be(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        firstEvents.Should().Be("enter;contact;contact;exit;");
        secondEvents.Should().Be("old-enter;old-contact;");
        originalPair.Active.Should().BeFalse();
        first.Collider.CollisionPairCount.Should().Be(0);
        first.Collider.CollisionPairHolderCount.Should().Be(0);
        second.Collider.CollisionPairCount.Should().Be(0);
        second.Collider.CollisionPairHolderCount.Should().Be(0);
    }

    [Fact]
    public void Simulate_WhenFirstExitCallbackRebindsSecond_ShouldNotExitFirstTwice()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        int originalSecondId = second.Collider.Id;
        int firstExitCount = 0;
        bool secondRebound = false;
        string secondEvents = string.Empty;
        first.Collider.OnContactExit += _ =>
        {
            firstExitCount++;
            if (firstExitCount != 1)
                return;

            second.Body.Deactivate();
            second.Body.Initialize(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity);
            second.Collider.Id.Should().Be(originalSecondId);
            secondRebound = true;
        };
        second.Collider.OnContactEnter += _ => secondEvents += secondRebound ? "new-enter;" : "old-enter;";
        second.Collider.OnContact += _ => secondEvents += secondRebound ? "new-contact;" : "old-contact;";
        second.Collider.OnContactExit += _ => secondEvents += secondRebound ? "new-exit;" : "old-exit;";

        Step(scenario.Context);

        CollisionPair originalPair = GetPair(first.Collider, second.Collider);
        originalPair.ColliderA.Should().BeSameAs(first.Collider);
        originalPair.ColliderB.Should().BeSameAs(second.Collider);

        first.Body.SetPosition(new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        Step(scenario.Context);

        firstExitCount.Should().Be(1);
        first.Body.Active.Should().BeTrue();
        second.Collider.Id.Should().Be(originalSecondId);
        second.Body.Active.Should().BeTrue();
        second.Body.Position3d.Should().Be(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        secondEvents.Should().Be("old-enter;old-contact;");
        originalPair.Active.Should().BeFalse();
        first.Collider.CollisionPairCount.Should().Be(0);
        first.Collider.CollisionPairHolderCount.Should().Be(0);
        second.Collider.CollisionPairCount.Should().Be(0);
        second.Collider.CollisionPairHolderCount.Should().Be(0);
    }

    [Fact]
    public void Simulate_WhenFirstEnterCallbackRebindsItself_ShouldNotResumeOldLifecycle()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        int originalFirstId = first.Collider.Id;
        string firstEvents = string.Empty;
        string secondEvents = string.Empty;
        bool firstRebound = false;
        first.Collider.OnContactEnter += _ =>
        {
            firstEvents += "old-enter;";
            first.Body.Deactivate();
            first.Body.Initialize(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity);
            first.Collider.Id.Should().Be(originalFirstId);
            firstRebound = true;
        };
        first.Collider.OnContact += _ => firstEvents += firstRebound ? "new-contact;" : "old-contact;";
        first.Collider.OnContactExit += _ => firstEvents += firstRebound ? "new-exit;" : "old-exit;";
        second.Collider.OnContactEnter += _ => secondEvents += "enter;";
        second.Collider.OnContact += _ => secondEvents += "contact;";
        second.Collider.OnContactExit += _ => secondEvents += "exit;";

        Step(scenario.Context);

        first.Collider.Id.Should().Be(originalFirstId);
        first.Body.Active.Should().BeTrue();
        first.Body.Position3d.Should().Be(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        firstEvents.Should().Be("old-enter;");
        secondEvents.Should().BeEmpty();
        first.Collider.CollisionPairCount.Should().Be(0);
        second.Collider.CollisionPairHolderCount.Should().Be(0);
    }

    [Fact]
    public void Deactivate_WhenExitCallbackRemovesAnotherOwnedPair_ShouldClearEveryPairOnce()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> owner = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> firstSupport = scenario.CreateSphere(
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        ScenarioBody<LSSphereCollider> secondSupport = scenario.CreateSphere(
            new Vector3d(-Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        int exits = 0;

        Step(scenario.Context);
        CollisionPair firstPair = GetPair(owner.Collider, firstSupport.Collider);
        CollisionPair secondPair = GetPair(owner.Collider, secondSupport.Collider);
        owner.Collider.OnContactExit += other =>
        {
            exits++;
            if (ReferenceEquals(other, firstSupport.Body))
                secondSupport.Body.Deactivate();
        };

        Action deactivate = owner.Body.Deactivate;

        deactivate.Should().NotThrow();
        exits.Should().Be(2);
        owner.Body.Active.Should().BeFalse();
        secondSupport.Body.Active.Should().BeFalse();
        firstPair.Active.Should().BeFalse();
        secondPair.Active.Should().BeFalse();
        firstSupport.Collider.CollisionPairHolderCount.Should().Be(0);
        scenario.Context.Physics.ColliderCount.Should().Be(1);
    }

    [Fact]
    public void Deactivate_WhenExitCallbackRebindsLaterSnapshotCollider_ShouldLeaveNewLifetimeUntouched()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> owner = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> firstSupport = scenario.CreateSphere(
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        ScenarioBody<LSSphereCollider> reboundSupport = scenario.CreateSphere(
            new Vector3d(-Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        int reboundId = reboundSupport.Collider.Id;
        Step(scenario.Context);
        owner.Collider.OnContactExit += other =>
        {
            if (!ReferenceEquals(other, firstSupport.Body))
                return;

            reboundSupport.Body.Deactivate();
            reboundSupport.Body.Initialize(
                new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero),
                FixedQuaternion.Identity);
        };

        owner.Body.Deactivate();

        reboundSupport.Body.Active.Should().BeTrue();
        reboundSupport.Collider.Id.Should().Be(reboundId);
        reboundSupport.Body.Position3d.Should().Be(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        reboundSupport.Collider.CollisionPairCount.Should().Be(0);
        reboundSupport.Collider.CollisionPairHolderCount.Should().Be(0);
    }

    [Fact]
    public void Deactivate_WhenExitCallbackThrows_ShouldLeaveRetryableCleanupState()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> owner = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> firstSupport = scenario.CreateSphere(
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        ScenarioBody<LSSphereCollider> secondSupport = scenario.CreateSphere(
            new Vector3d(-Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        Step(scenario.Context);
        CollisionPair firstPair = GetPair(owner.Collider, firstSupport.Collider);
        CollisionPair secondPair = GetPair(owner.Collider, secondSupport.Collider);
        bool throwOnce = true;
        owner.Collider.OnContactExit += _ =>
        {
            if (!throwOnce)
                return;

            throwOnce = false;
            throw new InvalidOperationException("exit failure");
        };

        Action firstDeactivate = owner.Body.Deactivate;
        firstDeactivate.Should().Throw<InvalidOperationException>().WithMessage("exit failure");

        owner.Body.Active.Should().BeTrue();
        owner.Collider.IsActive.Should().BeTrue();
        owner.Collider.IsDeactivationInProgress.Should().BeFalse();
        new[] { firstPair.Active, secondPair.Active }.Should().ContainSingle(active => !active);

        owner.Body.Deactivate();

        owner.Body.Active.Should().BeFalse();
        firstPair.Active.Should().BeFalse();
        secondPair.Active.Should().BeFalse();
        firstSupport.Collider.CollisionPairHolderCount.Should().Be(0);
        secondSupport.Collider.CollisionPairHolderCount.Should().Be(0);
    }

    private static void Step(GravitasWorldContext context)
    {
        context.Simulate();
        context.LateSimulate();
    }

    private static CollisionPair GetPair(LSCollider first, LSCollider second)
    {
        if (first.TryGetCollisionPair(second.Id, out CollisionPair? firstPair) && firstPair != null)
            return firstPair;

        second.TryGetCollisionPair(first.Id, out CollisionPair? secondPair).Should().BeTrue();
        return secondPair!;
    }
}
