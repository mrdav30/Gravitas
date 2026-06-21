using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Runtime;

public sealed class GravitasSimulationPhaseOrderTests
{
    [Fact]
    public void FixedStepScenario_ShouldReplaySameAuthoritativeState()
    {
        SimulationSnapshot first = RunReplayScenario();
        SimulationSnapshot second = RunReplayScenario();

        second.Should().Be(first);
    }

    [Fact]
    public void LateSimulate_ShouldRefreshTeleportedCollidersAndDistributeContactsAfterIntegration()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(8, 0, 0));
        PhysicsScenarioBuilder.SetTrigger(second.Collider);
        Vector3d teleportedPosition = new(Fixed64.Half, Fixed64.Zero, Fixed64.Zero);

        second.Body.SetPosition(teleportedPosition);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        second.Body.Position3d.Should().Be(teleportedPosition);
        first.Collider.TryGetCollisionPair(second.Collider.Id, out CollisionPair? pair).Should().BeTrue();
        pair!.Manifold.HasContact.Should().BeTrue();
        pair.LastFrame.Should().Be(scenario.Context.FrameCount);
    }

    [Fact]
    public void LateSimulate_ShouldIntegrateForcesAfterSimulate()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        Vector3d startPosition = body.Body.Position3d;

        body.Body.AddForce(new Vector3d((Fixed64)12, Fixed64.Zero, Fixed64.Zero));
        scenario.Context.Simulate();

        body.Body.Position3d.Should().Be(startPosition);
        body.Body.LinearVelocity.Should().Be(Vector3d.Zero);

        scenario.Context.LateSimulate();

        body.Body.Position3d.X.Should().BeGreaterThan(startPosition.X);
        body.Body.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void Visualize_ShouldNotMutateAuthoritativeBodyState()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        body.Body.CanSetVisualPosition = true;
        body.Body.AddForce(new Vector3d((Fixed64)12, Fixed64.Zero, Fixed64.Zero));
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();
        Vector3d authoritativePosition = body.Body.Position3d;
        Vector3d authoritativeVelocity = body.Body.LinearVelocity;

        scenario.Context.Visualize();
        scenario.Context.LateVisualize();

        body.Body.Position3d.Should().Be(authoritativePosition);
        body.Body.LinearVelocity.Should().Be(authoritativeVelocity);
    }

    [Fact]
    public void Hooks_ShouldRunAfterBuiltInPhaseWork()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(8, 0, 0));
        PhysicsScenarioBuilder.SetTrigger(second.Collider);
        Vector3d teleportedPosition = new(Fixed64.Half, Fixed64.Zero, Fixed64.Zero);
        Vector3d startPosition = first.Body.Position3d;
        bool simulateHookRanBeforeContacts = false;
        bool lateSimulateHookSawSolvedFrame = false;

        using IDisposable simulateHook = scenario.Context.RegisterOnSimulate(
            "PhaseOrder.ContactProbe",
            0,
            () => simulateHookRanBeforeContacts = !first.Collider.TryGetCollisionPair(second.Collider.Id, out _)
                && scenario.Context.FrameCount == 1);
        using IDisposable lateSimulateHook = scenario.Context.RegisterOnLateSimulate(
            "PhaseOrder.IntegrationProbe",
            0,
            () => lateSimulateHookSawSolvedFrame =
                first.Body.Position3d.X > startPosition.X
                && first.Collider.TryGetCollisionPair(second.Collider.Id, out CollisionPair? pair)
                && pair!.Manifold.HasContact);

        second.Body.SetPosition(teleportedPosition);
        first.Body.AddForce(new Vector3d((Fixed64)12, Fixed64.Zero, Fixed64.Zero));
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        simulateHookRanBeforeContacts.Should().BeTrue();
        lateSimulateHookSawSolvedFrame.Should().BeTrue();
    }

    private static SimulationSnapshot RunReplayScenario()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.SetFrameRate(30);
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            preventAngularForces: true);
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(
            PhysicsScenarioBuilder.Vector(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            preventAngularForces: true);

        for (int frame = 0; frame < 6; frame++)
        {
            if (frame == 1)
                left.Body.AddForce(new Vector3d((Fixed64)18, Fixed64.Zero, Fixed64.Zero));

            if (frame == 2)
                right.Body.AddForce(new Vector3d((Fixed64)(-9), Fixed64.Zero, Fixed64.Zero));

            if (frame == 3)
                right.Body.SetPosition(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));

            scenario.Context.Simulate();
            scenario.Context.LateSimulate();
            scenario.Context.Visualize();
            scenario.Context.LateVisualize();
        }

        left.Collider.TryGetCollisionPair(right.Collider.Id, out CollisionPair? pair);
        bool hasContact = pair?.Manifold.HasContact ?? false;
        return new SimulationSnapshot(
            scenario.Context.FrameCount,
            scenario.Context.TotalTime,
            left.Body.Position3d,
            right.Body.Position3d,
            left.Body.LinearVelocity,
            right.Body.LinearVelocity,
            left.Collider.Bounds.Center,
            right.Collider.Bounds.Center,
            hasContact,
            hasContact ? pair!.Manifold.PrimaryContact.Depth : Fixed64.Zero);
    }

    private readonly record struct SimulationSnapshot(
        int FrameCount,
        Fixed64 TotalTime,
        Vector3d LeftPosition,
        Vector3d RightPosition,
        Vector3d LeftVelocity,
        Vector3d RightVelocity,
        Vector3d LeftColliderCenter,
        Vector3d RightColliderCenter,
        bool HasContact,
        Fixed64 ContactDepth);
}
