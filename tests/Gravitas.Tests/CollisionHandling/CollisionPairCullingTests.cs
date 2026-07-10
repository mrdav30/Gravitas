using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CollisionPairCullingTests
{
    [Fact]
    public void UpdateCollision_AfterPairDeactivation_ShouldBeCompleteNoOp()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * Fixed64.Half);
        CollisionPair pair = scenario.Context.Physics.GetCollisionPair(
            first.Collider.Id,
            second.Collider.Id)!;
        int exitCount = 0;
        first.Collider.OnContactExit += _ => exitCount++;
        second.Collider.OnContactExit += _ => exitCount++;
        scenario.Context.Diagnostics.Enable(eventCapacity: 4, drawCommandCapacity: 0);

        pair.UpdateCollision();
        pair.Manifold.HasContact.Should().BeTrue();
        pair.Deactivate();

        int lastFrame = pair.LastFrame;
        short cullCounter = pair.CullCounter;
        int diagnosticCount = scenario.Context.Diagnostics.EventCount;
        int exitCountAfterDeactivate = exitCount;
        var replayHash = scenario.Context.ComputeReplayHash(
            GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        pair.UpdateCollision();

        pair.Active.Should().BeFalse();
        pair.Manifold.HasContact.Should().BeFalse();
        pair.LastFrame.Should().Be(lastFrame);
        pair.CullCounter.Should().Be(cullCounter);
        scenario.Context.Diagnostics.EventCount.Should().Be(diagnosticCount);
        exitCount.Should().Be(exitCountAfterDeactivate);
        scenario.Context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should().Be(replayHash);
    }

    [Fact]
    public void UpdateCollision_WithPositiveCullCounter_ShouldRecheckMovedCollider()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(8, 0, 0));
        CollisionPair pair = scenario.CreatePair(first.Collider, second.Collider);

        ClearPartitionFlags(first.Collider, second.Collider);
        pair.UpdateCollision();

        pair.Manifold.HasContact.Should().BeFalse();
        pair.CullCounter.Should().BeGreaterThan(0);

        ClearPartitionFlags(first.Collider, second.Collider);
        second.Body.SetPosition(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        second.Collider.Simulate();
        ClearPartitionFlags(first.Collider, second.Collider);

        pair.UpdateCollision();

        pair.Manifold.HasContact.Should().BeTrue();
        pair.CullCounter.Should().Be(0);
    }

    [Fact]
    public void UpdateCollision_ShouldCullFastMovingPairsLessAggressivelyThanStationaryPairs()
    {
        using PhysicsScenarioBuilder stationaryScenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> stationaryFirst = stationaryScenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> stationarySecond = stationaryScenario.CreateSphere(PhysicsScenarioBuilder.Vector(8, 0, 0));
        CollisionPair stationaryPair = stationaryScenario.CreatePair(stationaryFirst.Collider, stationarySecond.Collider);
        ClearPartitionFlags(stationaryFirst.Collider, stationarySecond.Collider);

        stationaryPair.UpdateCollision();

        short stationaryCullCounter = stationaryPair.CullCounter;
        stationaryCullCounter.Should().BeGreaterThan(0);

        using PhysicsScenarioBuilder fastScenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> fastFirst = fastScenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> fastSecond = fastScenario.CreateSphere(PhysicsScenarioBuilder.Vector(8, 0, 0));
        CollisionPair fastPair = fastScenario.CreatePair(fastFirst.Collider, fastSecond.Collider);
        fastFirst.Body.ApplyCollisionLinearVelocityDelta(new Vector3d((Fixed64)7, Fixed64.Zero, Fixed64.Zero));
        fastSecond.Body.ApplyCollisionLinearVelocityDelta(new Vector3d((Fixed64)(-7), Fixed64.Zero, Fixed64.Zero));
        ClearPartitionFlags(fastFirst.Collider, fastSecond.Collider);

        fastPair.UpdateCollision();

        fastPair.CullCounter.Should().BeLessThan(stationaryCullCounter);
    }

    [Fact]
    public void UpdateCollision_WithDisabledCullSteps_ShouldNotDivideByZero()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(8, 0, 0));
        CollisionPair pair = scenario.CreatePair(first.Collider, second.Collider);
        scenario.Context.Environment.CullDistanceMax = 0;
        scenario.Context.Environment.CullVelocityStep = 0;
        scenario.Context.Environment.CullTimeStep = 0;
        ClearPartitionFlags(first.Collider, second.Collider);

        Action updateCollision = pair.UpdateCollision;

        updateCollision.Should().NotThrow();
        pair.CullCounter.Should().Be(0);
    }

    [Fact]
    public void UpdateCollision_WithShapeChangeOnly_ShouldRecheckActiveContact()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        CollisionPair pair = scenario.CreatePair(first.Collider, second.Collider);

        pair.UpdateCollision();
        pair.Manifold.HasContact.Should().BeTrue();
        ClearBodyChangeFlags(first.Body, second.Body);
        ClearPartitionFlags(first.Collider, second.Collider);

        second.Collider.Radius = Fixed64.FromFraction(1, 16);
        second.Collider.Simulate();
        ClearPartitionFlags(first.Collider, second.Collider);

        pair.UpdateCollision();

        pair.Manifold.HasContact.Should().BeFalse();
    }

    [Fact]
    public void UpdateCollision_WithTouchingCuboids_ShouldKeepZeroDepthContact()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> first = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSCuboidCollider> second = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(1, 0, 0));
        CollisionPair pair = scenario.CreatePair(first.Collider, second.Collider);
        ClearPartitionFlags(first.Collider, second.Collider);

        pair.UpdateCollision();

        pair.Manifold.HasContact.Should().BeTrue();
        pair.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void WakeSleepingBodiesForCollision_ShouldWakeOnlyLinkedSleepingBodyWhenOtherCanDriveCollision()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> sleepingA = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> awakeB = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(1, 0, 0));
        CollisionPair pair = scenario.CreatePair(sleepingA.Collider, awakeB.Collider);
        sleepingA.Body.Sleep();

        pair.WakeSleepingBodiesForCollision();

        sleepingA.Body.IsSleeping.Should().BeFalse();
        awakeB.Body.IsSleeping.Should().BeFalse();

        awakeB.Body.Sleep();

        pair.WakeSleepingBodiesForCollision();

        awakeB.Body.IsSleeping.Should().BeFalse();

        LSSphereCollider bodylessA = scenario.CreateStaticSphere(PhysicsScenarioBuilder.Vector(2, 0, 0));
        scenario.CreatePair(bodylessA, awakeB.Collider).WakeSleepingBodiesForCollision();
        scenario.CreatePair(awakeB.Collider, bodylessA).WakeSleepingBodiesForCollision();
    }

    private static void ClearPartitionFlags(LSCollider first, LSCollider second)
    {
        first.PartitionChanged = false;
        second.PartitionChanged = false;
    }

    private static void ClearBodyChangeFlags(SolidBody first, SolidBody second)
    {
        first.CheckChangedValues();
        second.CheckChangedValues();
        first.CheckChangedValues();
        second.CheckChangedValues();
        first.PositionChangePending.Should().BeFalse();
        second.PositionChangePending.Should().BeFalse();
        first.RotationChangePending.Should().BeFalse();
        second.RotationChangePending.Should().BeFalse();
    }
}
