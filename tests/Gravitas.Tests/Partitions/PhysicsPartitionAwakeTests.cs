using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Diagnostics;
using Gravitas.Tests.Support;
using System;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Partitions;

public sealed class PhysicsPartitionAwakeTests
{
    [Fact]
    public void SleepingDynamicBodies_ShouldRemainPartitionedButNotAwake()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        PhysicsPartition partition = GetFirstPartition(scenario, body.Collider);

        body.Body.Sleep();

        partition.ContainedDynamicObjects!.Contains(body.Collider.Id).Should().BeTrue();
        partition.ContainsAwakeDynamicObject(body.Collider.Id).Should().BeFalse();
        partition.AwakeDynamicObjectCount.Should().Be(0);
    }

    [Fact]
    public void Distribute_WithOnlySleepingDynamicBodies_ShouldSkipPairGeneration()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        first.Body.Sleep();
        second.Body.Sleep();

        scenario.Context.Collisions.CheckAndDistributeCollisions();

        first.Collider.TryGetCollisionPair(second.Collider.Id, out _).Should().BeFalse();
    }

    [Fact]
    public void Distribute_WithAwakeDynamicAgainstSleepingDynamic_ShouldProcessPairAndWakeSleepingBody()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> awake = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> sleeping = scenario.CreateSphere(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        sleeping.Body.Sleep();

        scenario.Context.Collisions.CheckAndDistributeCollisions();

        awake.Collider.TryGetCollisionPair(sleeping.Collider.Id, out CollisionPair? pair).Should().BeTrue();
        pair!.Manifold.HasContact.Should().BeTrue();
        sleeping.Body.IsSleeping.Should().BeFalse();
    }

    [Fact]
    public void Simulate_WithAwakeDynamicAgainstSleepingDynamic_ShouldStillNotifyContactEnter()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> awake = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> sleeping = scenario.CreateSphere(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        int awakeEnterCount = 0;
        int sleepingEnterCount = 0;
        awake.Collider.OnContactEnter += _ => awakeEnterCount++;
        sleeping.Collider.OnContactEnter += _ => sleepingEnterCount++;
        sleeping.Body.Sleep();

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        awakeEnterCount.Should().Be(1);
        sleepingEnterCount.Should().Be(1);
        sleeping.Body.IsSleeping.Should().BeFalse();
    }

    [Fact]
    public void SleepingRestingContact_ShouldNotAgeOutAsFalseExit()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        int firstExitCount = 0;
        int secondExitCount = 0;
        first.Collider.OnContactExit += _ => firstExitCount++;
        second.Collider.OnContactExit += _ => secondExitCount++;
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();
        first.Body.Sleep();
        second.Body.Sleep();

        for (int i = 0; i <= scenario.Context.FrameRate * 8; i++)
        {
            scenario.Context.Simulate();
            scenario.Context.LateSimulate();
        }

        firstExitCount.Should().Be(0);
        secondExitCount.Should().Be(0);
        first.Collider.TryGetCollisionPair(second.Collider.Id, out CollisionPair? pair).Should().BeTrue();
        pair!.Active.Should().BeTrue();
    }

    [Fact]
    public void Simulate_RepeatedOverlappingDynamics_ShouldProduceStableContactOrder()
    {
        CaptureContactOrder().Should().Equal(CaptureContactOrder());
    }

    private static PhysicsPartition GetFirstPartition(PhysicsScenarioBuilder scenario, LSCollider collider)
    {
        scenario.Context.World.TryGetVoxel(collider.PartitionCoordinates![0], out GridForge.Grids.Voxel? voxel)
            .Should().BeTrue();
        voxel!.TryGetPartition(out PhysicsPartition? partition).Should().BeTrue();
        return partition!;
    }

    private static int[] CaptureContactOrder()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Diagnostics.Enable(eventCapacity: 16, drawCommandCapacity: 0);
        scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        scenario.CreateSphere(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        scenario.CreateSphere(new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero));

        scenario.Context.Simulate();

        var order = new List<int>();
        ReadOnlySpan<GravitasDiagnosticEvent> events = scenario.Context.Diagnostics.Events;
        for (int i = 0; i < events.Length; i++)
        {
            GravitasDiagnosticEvent diagnosticEvent = events[i];
            if (diagnosticEvent.Kind != GravitasDiagnosticEventKind.Contact)
                continue;

            order.Add((diagnosticEvent.ColliderAId << 16) | diagnosticEvent.ColliderBId);
        }

        return order.ToArray();
    }
}
