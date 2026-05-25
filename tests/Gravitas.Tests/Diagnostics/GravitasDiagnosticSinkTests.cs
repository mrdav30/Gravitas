using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Diagnostics;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Diagnostics;

public sealed class GravitasDiagnosticSinkTests
{
    [Fact]
    public void DisabledDiagnostics_ShouldNotAllocateFromRuntimeHooks()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));

        long allocatedBytes = MeasureAllocatedBytes(() =>
        {
            sphere.Body.AddForce(new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero));
            sphere.Body.AddTorque(new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));
            scenario.Context.Diagnostics.CaptureCollider(sphere.Collider, GravitasDiagnosticColor.Cyan);
        });

        allocatedBytes.Should().Be(0);
        scenario.Context.Diagnostics.EventCount.Should().Be(0);
        scenario.Context.Diagnostics.DrawCommandCount.Should().Be(0);
    }

    [Fact]
    public void EnabledDiagnostics_ShouldRecordDeterministicEventsAndStayContextScoped()
    {
        using PhysicsScenarioBuilder firstScenario = PhysicsScenarioBuilder.Create();
        using PhysicsScenarioBuilder secondScenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = firstScenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> second = secondScenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));

        firstScenario.Context.Diagnostics.Enable(eventCapacity: 8, drawCommandCapacity: 8);
        first.Body.AddForce(new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));
        first.Body.AddTorque(new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero));
        first.Body.CheckGround();
        second.Body.AddForce(new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero));

        ReadOnlySpan<GravitasDiagnosticEvent> events = firstScenario.Context.Diagnostics.Events;
        events.Length.Should().Be(4);
        events[0].Kind.Should().Be(GravitasDiagnosticEventKind.ForceDelta);
        events[0].Sequence.Should().Be(0);
        events[0].BodyId.Should().Be(first.Body.DynamicId);
        events[0].ColliderAId.Should().Be(first.Collider.Id);
        events[0].Vector.Should().Be(new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));
        events[1].Kind.Should().Be(GravitasDiagnosticEventKind.TorqueDelta);
        events[1].Sequence.Should().Be(1);
        events[2].Kind.Should().Be(GravitasDiagnosticEventKind.RayQuery);
        events[2].Sequence.Should().Be(2);
        events[3].Kind.Should().Be(GravitasDiagnosticEventKind.GroundProbe);
        events[3].Sequence.Should().Be(3);
        events[3].BodyId.Should().Be(first.Body.DynamicId);
        events[3].Hit.Should().BeFalse();

        secondScenario.Context.Diagnostics.EventCount.Should().Be(0);
    }

    [Fact]
    public void CollisionDiagnostics_ShouldRecordContactResponseAndVelocityDeltasInOrder()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        first.Body.AddLinearImpulse(new Vector3d((Fixed64)10, Fixed64.Zero, Fixed64.Zero));
        CollisionPair pair = scenario.CreatePair(first.Collider, second.Collider);

        scenario.Context.Diagnostics.Enable(eventCapacity: 16, drawCommandCapacity: 16);
        pair.UpdateCollision();

        ReadOnlySpan<GravitasDiagnosticEvent> events = scenario.Context.Diagnostics.Events;
        events.Length.Should().BeGreaterThanOrEqualTo(4);
        events[0].Kind.Should().Be(GravitasDiagnosticEventKind.Contact);
        events[0].ColliderAId.Should().Be(pair.ColliderA.Id);
        events[0].ColliderBId.Should().Be(pair.ColliderB.Id);
        events[0].Hit.Should().BeTrue();
        events[1].Kind.Should().Be(GravitasDiagnosticEventKind.ResponseImpulse);
        events[1].ColliderAId.Should().Be(pair.ColliderA.Id);
        events[1].ColliderBId.Should().Be(pair.ColliderB.Id);
        events[1].Vector.SqrMagnitude.Should().BeGreaterThan(Fixed64.Zero);
        events[2].Kind.Should().Be(GravitasDiagnosticEventKind.LinearVelocityDelta);
        events[3].Kind.Should().Be(GravitasDiagnosticEventKind.LinearVelocityDelta);

        for (int i = 0; i < events.Length; i++)
            events[i].Sequence.Should().Be(i);
    }

    [Fact]
    public void CaptureCollider_ShouldEmitEngineAgnosticDrawCommands()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateCapsule(PhysicsScenarioBuilder.Vector(2, 0, 0));
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(4, 0, 0), PhysicsScenarioBuilder.Yaw(35));
        ScenarioBody<LSCylinderCollider> cylinder = scenario.CreateCylinder(PhysicsScenarioBuilder.Vector(6, 0, 0));
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            CreateTriangleMesh(),
            PhysicsScenarioBuilder.Vector(8, 0, 0),
            FixedQuaternion.Identity);

        scenario.Context.Diagnostics.Enable(eventCapacity: 4, drawCommandCapacity: 16);
        scenario.Context.Diagnostics.CaptureCollider(sphere.Collider, GravitasDiagnosticColor.Cyan);
        scenario.Context.Diagnostics.CaptureCollider(capsule.Collider, GravitasDiagnosticColor.Green);
        scenario.Context.Diagnostics.CaptureCollider(cuboid.Collider, GravitasDiagnosticColor.Yellow);
        scenario.Context.Diagnostics.CaptureCollider(cylinder.Collider, GravitasDiagnosticColor.Red);
        scenario.Context.Diagnostics.CaptureCollider(mesh.Collider, GravitasDiagnosticColor.White);
        scenario.Context.Diagnostics.CaptureLine(
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            PhysicsScenarioBuilder.Vector(1, 0, 0),
            GravitasDiagnosticColor.Blue);
        scenario.Context.Diagnostics.CaptureRay(
            PhysicsScenarioBuilder.Vector(1, 0, 0),
            Vector3d.Forward,
            (Fixed64)2,
            GravitasDiagnosticColor.Green);
        scenario.Context.Diagnostics.CapturePoint(
            PhysicsScenarioBuilder.Vector(2, 0, 0),
            Fixed64.Half,
            GravitasDiagnosticColor.Red);

        ReadOnlySpan<GravitasDebugDrawCommand> commands = scenario.Context.Diagnostics.DrawCommands;
        commands.Length.Should().Be(8);
        commands[0].Kind.Should().Be(GravitasDebugDrawKind.WireSphere);
        commands[0].ColliderId.Should().Be(sphere.Collider.Id);
        commands[1].Kind.Should().Be(GravitasDebugDrawKind.WireCapsule);
        commands[2].Kind.Should().Be(GravitasDebugDrawKind.WireBox);
        commands[2].Rotation.Should().Be(cuboid.Collider.Rotation);
        commands[3].Kind.Should().Be(GravitasDebugDrawKind.WireCylinder);
        commands[4].Kind.Should().Be(GravitasDebugDrawKind.WireTriangle);
        commands[4].PointA.Should().Be(mesh.Collider.Mesh.Vertices[0]);
        commands[4].PointB.Should().Be(mesh.Collider.Mesh.Vertices[1]);
        commands[4].PointC.Should().Be(mesh.Collider.Mesh.Vertices[2]);
        commands[5].Kind.Should().Be(GravitasDebugDrawKind.Line);
        commands[5].Start.Should().Be(PhysicsScenarioBuilder.Vector(0, 0, 0));
        commands[5].End.Should().Be(PhysicsScenarioBuilder.Vector(1, 0, 0));
        commands[6].Kind.Should().Be(GravitasDebugDrawKind.Ray);
        commands[6].Start.Should().Be(PhysicsScenarioBuilder.Vector(1, 0, 0));
        commands[6].End.Should().Be(PhysicsScenarioBuilder.Vector(1, 0, 2));
        commands[7].Kind.Should().Be(GravitasDebugDrawKind.Point);
        commands[7].Center.Should().Be(PhysicsScenarioBuilder.Vector(2, 0, 0));
        commands[7].Radius.Should().Be(Fixed64.Half);
    }

    private static long MeasureAllocatedBytes(Action action)
    {
        action();

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 64; i++)
            action();

        return GC.GetAllocatedBytesForCurrentThread() - before;
    }

    private static LSMeshCollider CreateTriangleMesh()
    {
        return new LSMeshCollider(
            new[]
            {
                new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
                new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero),
                new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.One)
            },
            new[] { 0, 1, 2 });
    }
}
