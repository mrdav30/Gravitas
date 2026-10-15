using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class UntouchedCollisionPairMutationTests
{
    [Fact]
    public void UnchangedSleepingContact_ShouldRetainItsManifoldAndWarmImpulseDuringLongRest()
    {
        using PhysicsScenarioBuilder scene = CreateScene();
        var (target, mover) = AddContact(scene, Vector3d.Zero);
        CollisionPair pair = SettleAndSleep(scene, target, mover);
        ulong contactId = pair.Manifold.PrimaryContact.ContactId;
        pair.TryGetWarmStartImpulse(contactId, out ContactWarmStartImpulse impulse).Should().BeTrue();
        impulse.NormalImpulse.Should().BeGreaterThan(Fixed64.Zero);
        Vector3d position = mover.Position3d;
        int exits = 0;
        target.OnContactExit += _ => exits++;

        for (int frame = 0; frame < scene.Context.FrameRate * 10; frame++)
            Step(scene.Context);

        pair.Active.Should().BeTrue();
        pair.Manifold.PrimaryContact.ContactId.Should().Be(contactId);
        pair.TryGetWarmStartImpulse(contactId, out ContactWarmStartImpulse retained).Should().BeTrue();
        retained.Should().Be(impulse);
        mover.Position3d.Should().Be(position);
        mover.IsSleeping.Should().BeTrue();
        exits.Should().Be(0);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ChangedSupport_ShouldEndTheSleepingParticipantsStaleContact(bool shrink)
    {
        using PhysicsScenarioBuilder scene = CreateScene();
        var (target, mover) = AddContact(scene, Vector3d.Zero);
        CollisionPair pair = SettleAndSleep(scene, target, mover);
        ulong contactId = pair.Manifold.PrimaryContact.ContactId;
        int exits = 0, stays = 0;
        target.OnContactExit += _ => exits++;
        target.OnContact += _ => stays++;
        if (shrink) target.Radius = Fixed64.FromFraction(1, 16);
        else target.Position = Vector3d.Left * (Fixed64)8;
        target.Simulate();

        Step(scene.Context);

        exits.Should().Be(1);
        stays.Should().Be(0);
        pair.Active.Should().BeFalse();
        pair.Manifold.HasContact.Should().BeFalse();
        pair.TryGetWarmStartImpulse(contactId, out _).Should().BeFalse();
        Step(scene.Context);
        exits.Should().Be(1);
    }

    [Fact]
    public void ChangedSupportWithOverlappingBounds_ShouldStillCheckTheActualShapes()
    {
        using PhysicsScenarioBuilder scene = CreateScene();
        var (target, mover) = AddContact(scene, Vector3d.Zero);
        CollisionPair pair = SettleAndSleep(scene, target, mover);
        int exits = 0;
        target.OnContactExit += _ => exits++;
        // Both component gaps are below the diameter; their diagonal exceeds it.
        target.Position = mover.Collider.Center + new Vector3d(Fixed64.FromFraction(3, 4),
            Fixed64.FromFraction(3, 4), Fixed64.Zero);
        target.Simulate();
        target.BoundsMin.X.Should().BeLessThan(mover.Collider.BoundsMax.X);
        target.BoundsMin.Y.Should().BeLessThan(mover.Collider.BoundsMax.Y);

        Step(scene.Context);

        exits.Should().Be(1);
        pair.Active.Should().BeFalse();
        pair.Manifold.HasContact.Should().BeFalse();
    }

    [Fact]
    public void ChangedSupportThatStillTouches_ShouldRefreshTheWitnessWithoutLosingTheWarmImpulse()
    {
        using PhysicsScenarioBuilder scene = CreateScene();
        var (target, mover) = AddContact(scene, Vector3d.Zero);
        CollisionPair pair = SettleAndSleep(scene, target, mover);
        ulong contactId = pair.Manifold.PrimaryContact.ContactId;
        pair.TryGetWarmStartImpulse(contactId, out ContactWarmStartImpulse impulse).Should().BeTrue();
        Vector3d previousWitness = GetTargetWitness(pair, target);
        int exits = 0;
        target.OnContactExit += _ => exits++;
        Fixed64 shift = Fixed64.FromFraction(1, 64);
        target.Position = Vector3d.Right * shift;
        target.Simulate();

        Step(scene.Context);

        pair.Active.Should().BeTrue();
        pair.Manifold.HasContact.Should().BeTrue();
        GetTargetWitness(pair, target).Should().Be(previousWitness + Vector3d.Right * shift);
        pair.TryGetWarmStartImpulse(contactId, out ContactWarmStartImpulse retained).Should().BeTrue();
        retained.Should().Be(impulse);
        mover.IsSleeping.Should().BeTrue();
        exits.Should().Be(0);
    }

    [Fact]
    public void PendingSeparation_ShouldNotifyEvenWhenFurtherCollisionChecksAreCulled()
    {
        using PhysicsScenarioBuilder scene = CreateScene();
        var (target, mover) = AddContact(scene, Vector3d.Zero);
        Step(scene.Context);
        CollisionPair pair = scene.Context.Physics.GetCollisionPair(target.Id, mover.Collider.Id)!;
        int exits = 0, stays = 0;
        target.OnContactExit += _ => exits++;
        target.OnContact += _ => stays++;
        mover.SetPosition(Vector3d.Right * (Fixed64)8);
        pair.UpdateCollision();
        pair.Manifold.HasContact.Should().BeFalse();
        pair.CullCounter.Should().BeGreaterThan(0);

        Step(scene.Context);

        exits.Should().Be(1);
        stays.Should().Be(0);
        Step(scene.Context);
        exits.Should().Be(1);
    }

    [Fact]
    public void ThrowingUntouchedExit_ShouldLeaveLaterQueuedSeparationRetryable()
    {
        using PhysicsScenarioBuilder scene = CreateScene();
        var first = AddContact(scene, Vector3d.Zero);
        var second = AddContact(scene, Vector3d.Right * (Fixed64)8);
        Step(scene.Context);
        int firstExits = 0, secondExits = 0;
        first.Target.OnContactExit += _ => { firstExits++; throw new InvalidOperationException("untouched exit failure"); };
        second.Target.OnContactExit += _ => secondExits++;
        first.Mover.SetPosition(first.Mover.Position3d + Vector3d.Up * (Fixed64)8);
        second.Mover.SetPosition(second.Mover.Position3d + Vector3d.Up * (Fixed64)8);

        Action firstStep = () => Step(scene.Context);
        firstStep.Should().Throw<InvalidOperationException>().WithMessage("untouched exit failure");
        firstExits.Should().Be(1);
        secondExits.Should().Be(0);

        Step(scene.Context);

        firstExits.Should().Be(1);
        secondExits.Should().Be(1);
        (first.Target.CollisionPairCount + first.Target.CollisionPairHolderCount).Should().Be(0);
        (second.Target.CollisionPairCount + second.Target.CollisionPairHolderCount).Should().Be(0);
    }

    [Fact]
    public void UntouchedExitThatRebindsAndSteps_ShouldNotNotifyTheLaterRetiredLifetimeAgain()
    {
        using PhysicsScenarioBuilder scene = CreateScene();
        var first = AddContact(scene, Vector3d.Zero);
        var second = AddContact(scene, Vector3d.Right * (Fixed64)8);
        Step(scene.Context);
        int oldExits = 0, newEnters = 0, newStays = 0, phantomExits = 0;
        bool rebound = false;
        second.Target.OnContactEnter += _ => { if (rebound) newEnters++; };
        second.Target.OnContact += _ => { if (rebound) newStays++; };
        second.Target.OnContactExit += _ => { if (rebound) phantomExits++; else oldExits++; };
        first.Target.OnContactExit += _ =>
        {
            second.Mover.Deactivate();
            rebound = true;
            second.Mover.Initialize(new Vector3d((Fixed64)8 + Fixed64.FromFraction(3, 4),
                Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity);
            second.Mover.UseManualGrounding();
            second.Mover.SleepEnabled = false;
            Step(scene.Context);
        };
        first.Mover.SetPosition(first.Mover.Position3d + Vector3d.Up * (Fixed64)8);
        second.Mover.SetPosition(second.Mover.Position3d + Vector3d.Up * (Fixed64)8);

        Step(scene.Context);

        oldExits.Should().Be(1);
        newEnters.Should().Be(1);
        newStays.Should().Be(1);
        phantomExits.Should().Be(0);
        Step(scene.Context);
        newEnters.Should().Be(1);
        phantomExits.Should().Be(0);
    }

    private static PhysicsScenarioBuilder CreateScene()
    {
        PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
        scene.Context.Environment.Gravity = Fixed64.Zero;
        scene.Context.Environment.AirDensity = Fixed64.Zero;
        scene.Context.Environment.DampingFactor = Fixed64.Zero;
        scene.Context.Environment.MinSpeed = Fixed64.Zero;
        return scene;
    }

    private static (LSSphereCollider Target, SolidBody Mover) AddContact(
        PhysicsScenarioBuilder scene, Vector3d targetCenter)
    {
        var target = new LSSphereCollider { Material = PhysicsMaterial.Frictionless };
        scene.InitializeStaticCollider(target, targetCenter);
        SolidBody mover = scene.CreateSphere(targetCenter + Vector3d.Right * Fixed64.FromFraction(3, 4),
            preventAngularForces: true).Body;
        mover.Collider.Material = PhysicsMaterial.Frictionless;
        mover.UseManualGrounding();
        mover.SleepEnabled = false;
        mover.AddLinearImpulse(Vector3d.Left);
        return (target, mover);
    }

    private static CollisionPair SettleAndSleep(PhysicsScenarioBuilder scene, LSCollider target, SolidBody mover)
    {
        Step(scene.Context);
        mover.SleepEnabled = true;
        mover.Sleep();
        Step(scene.Context);
        return scene.Context.Physics.GetCollisionPair(target.Id, mover.Collider.Id)!;
    }

    private static Vector3d GetTargetWitness(CollisionPair pair, LSCollider target) =>
        ReferenceEquals(pair.ColliderA, target) ? pair.Manifold.PrimaryContact.PointA : pair.Manifold.PrimaryContact.PointB;

    private static void Step(GravitasWorldContext context) { context.Simulate(); context.LateSimulate(); }
}
