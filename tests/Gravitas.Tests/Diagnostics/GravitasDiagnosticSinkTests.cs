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
        events[1].Vector.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
        events[2].Kind.Should().Be(GravitasDiagnosticEventKind.LinearVelocityDelta);
        events[3].Kind.Should().Be(GravitasDiagnosticEventKind.LinearVelocityDelta);

        for (int i = 0; i < events.Length; i++)
            events[i].Sequence.Should().Be(i);
    }

    [Fact]
    public void ClearAndDisable_ShouldResetSequencesAndRetainReservedCapacity()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));

        scenario.Context.Diagnostics.Enable(eventCapacity: 4, drawCommandCapacity: 4);
        int eventCapacity = scenario.Context.Diagnostics.EventCapacity;
        int drawCapacity = scenario.Context.Diagnostics.DrawCommandCapacity;

        sphere.Body.AddForce(new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero));
        scenario.Context.Diagnostics.CaptureCollider(sphere.Collider, GravitasDiagnosticColor.Cyan);
        scenario.Context.Diagnostics.Clear();

        scenario.Context.Diagnostics.EventCount.Should().Be(0);
        scenario.Context.Diagnostics.DrawCommandCount.Should().Be(0);
        scenario.Context.Diagnostics.EventCapacity.Should().Be(eventCapacity);
        scenario.Context.Diagnostics.DrawCommandCapacity.Should().Be(drawCapacity);

        sphere.Body.AddTorque(new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));
        scenario.Context.Diagnostics.CaptureLine(Vector3d.Zero, Vector3d.Right, GravitasDiagnosticColor.Yellow);

        scenario.Context.Diagnostics.Events[0].Sequence.Should().Be(0);
        scenario.Context.Diagnostics.DrawCommands[0].Sequence.Should().Be(0);

        scenario.Context.Diagnostics.Disable();

        scenario.Context.Diagnostics.Enabled.Should().BeFalse();
        scenario.Context.Diagnostics.EventCount.Should().Be(0);
        scenario.Context.Diagnostics.DrawCommandCount.Should().Be(0);
        scenario.Context.Diagnostics.EventCapacity.Should().Be(eventCapacity);
        scenario.Context.Diagnostics.DrawCommandCapacity.Should().Be(drawCapacity);

        sphere.Body.AddForce(new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero));
        scenario.Context.Diagnostics.CaptureCollider(sphere.Collider, GravitasDiagnosticColor.Cyan);

        scenario.Context.Diagnostics.EventCount.Should().Be(0);
        scenario.Context.Diagnostics.DrawCommandCount.Should().Be(0);
    }

    [Fact]
    public void Diagnostics_ShouldCaptureCurrentContextFrameAndResetPerBufferSequences()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));

        scenario.Context.Diagnostics.Enable(eventCapacity: 4, drawCommandCapacity: 4);
        scenario.Context.Simulate();
        int firstFrame = scenario.Context.FrameCount;

        sphere.Body.AddForce(new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero));
        scenario.Context.Diagnostics.CapturePoint(Vector3d.Zero, Fixed64.Half, GravitasDiagnosticColor.Red);

        scenario.Context.Diagnostics.Events[0].Frame.Should().Be(firstFrame);
        scenario.Context.Diagnostics.Events[0].Sequence.Should().Be(0);
        scenario.Context.Diagnostics.DrawCommands[0].Frame.Should().Be(firstFrame);
        scenario.Context.Diagnostics.DrawCommands[0].Sequence.Should().Be(0);

        scenario.Context.Diagnostics.Clear();
        scenario.Context.Simulate();
        int secondFrame = scenario.Context.FrameCount;

        sphere.Body.AddTorque(new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));
        scenario.Context.Diagnostics.CaptureLine(Vector3d.Zero, Vector3d.Right, GravitasDiagnosticColor.Yellow);

        scenario.Context.Diagnostics.Events[0].Frame.Should().Be(secondFrame);
        scenario.Context.Diagnostics.Events[0].Sequence.Should().Be(0);
        scenario.Context.Diagnostics.DrawCommands[0].Frame.Should().Be(secondFrame);
        scenario.Context.Diagnostics.DrawCommands[0].Sequence.Should().Be(0);
    }

    [Fact]
    public void CaptureCollider_ShouldEmitEngineAgnosticDrawCommands()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateCapsule(PhysicsScenarioBuilder.Vector(2, 0, 0));
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(4, 0, 0), PhysicsScenarioBuilder.Yaw(35));
        ScenarioBody<LSCylinderCollider> cylinder = scenario.CreateCylinder(PhysicsScenarioBuilder.Vector(6, 0, 0));
        ScenarioBody<LSCompoundCollider> compound = scenario.CreateBody(
            new LSCompoundCollider(
                new CompoundColliderPart(new LSSphereCollider { LocalOffset = new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero) }),
                new CompoundColliderPart(new LSCuboidCollider { LocalOffset = new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero) })),
            PhysicsScenarioBuilder.Vector(8, 0, 0),
            FixedQuaternion.Identity);
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            CreateTriangleMesh(),
            PhysicsScenarioBuilder.Vector(10, 0, 0),
            FixedQuaternion.Identity);

        scenario.Context.Diagnostics.Enable(eventCapacity: 4, drawCommandCapacity: 16);
        scenario.Context.Diagnostics.CaptureCollider(sphere.Collider, GravitasDiagnosticColor.Cyan);
        scenario.Context.Diagnostics.CaptureCollider(capsule.Collider, GravitasDiagnosticColor.Green);
        scenario.Context.Diagnostics.CaptureCollider(cuboid.Collider, GravitasDiagnosticColor.Yellow);
        scenario.Context.Diagnostics.CaptureCollider(cylinder.Collider, GravitasDiagnosticColor.Red);
        scenario.Context.Diagnostics.CaptureCollider(compound.Collider, GravitasDiagnosticColor.Blue);
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
        commands.Length.Should().Be(10);
        commands[0].Kind.Should().Be(GravitasDebugDrawKind.WireSphere);
        commands[0].ColliderId.Should().Be(sphere.Collider.Id);
        commands[1].Kind.Should().Be(GravitasDebugDrawKind.WireCapsule);
        commands[2].Kind.Should().Be(GravitasDebugDrawKind.WireBox);
        commands[2].Rotation.Should().Be(cuboid.Collider.Rotation);
        commands[3].Kind.Should().Be(GravitasDebugDrawKind.WireCylinder);
        commands[4].Kind.Should().Be(GravitasDebugDrawKind.WireSphere);
        commands[4].ColliderId.Should().Be(compound.Collider.Id);
        commands[4].ColliderType.Should().Be(ColliderType.Compound);
        commands[5].Kind.Should().Be(GravitasDebugDrawKind.WireBox);
        commands[5].ColliderId.Should().Be(compound.Collider.Id);
        commands[5].ColliderType.Should().Be(ColliderType.Compound);
        commands[6].Kind.Should().Be(GravitasDebugDrawKind.WireTriangle);
        commands[6].PointA.Should().Be(mesh.Collider.Mesh.Vertices[0]);
        commands[6].PointB.Should().Be(mesh.Collider.Mesh.Vertices[1]);
        commands[6].PointC.Should().Be(mesh.Collider.Mesh.Vertices[2]);
        commands[7].Kind.Should().Be(GravitasDebugDrawKind.Line);
        commands[7].Start.Should().Be(PhysicsScenarioBuilder.Vector(0, 0, 0));
        commands[7].End.Should().Be(PhysicsScenarioBuilder.Vector(1, 0, 0));
        commands[8].Kind.Should().Be(GravitasDebugDrawKind.Ray);
        commands[8].Start.Should().Be(PhysicsScenarioBuilder.Vector(1, 0, 0));
        commands[8].End.Should().Be(PhysicsScenarioBuilder.Vector(1, 0, 2));
        commands[9].Kind.Should().Be(GravitasDebugDrawKind.Point);
        commands[9].Center.Should().Be(PhysicsScenarioBuilder.Vector(2, 0, 0));
        commands[9].Radius.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void CaptureCollider_ShouldEmitOneWireTrianglePerHighVolumeMeshTriangle()
    {
        const int quadCount = 128;
        int expectedTriangles = quadCount * 2;
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = CreateStripMesh(quadCount);
        scenario.InitializeStaticCollider(mesh, PhysicsScenarioBuilder.Vector(0, 0, 0));

        scenario.Context.Diagnostics.Enable(eventCapacity: 0, drawCommandCapacity: expectedTriangles);
        scenario.Context.Diagnostics.CaptureCollider(mesh, GravitasDiagnosticColor.White);

        ReadOnlySpan<GravitasDebugDrawCommand> commands = scenario.Context.Diagnostics.DrawCommands;
        commands.Length.Should().Be(expectedTriangles);
        scenario.Context.Diagnostics.DrawCommandCapacity.Should().BeGreaterThanOrEqualTo(expectedTriangles);

        for (int i = 0; i < commands.Length; i++)
        {
            commands[i].Kind.Should().Be(GravitasDebugDrawKind.WireTriangle);
            commands[i].Sequence.Should().Be(i);
            commands[i].ColliderId.Should().Be(mesh.Id);
            commands[i].ColliderType.Should().Be(ColliderType.Mesh);
        }
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
            new[] { 0, 1, 2 },
            MeshColliderMode.Convex,
            MeshInertiaPolicy.SurfaceApproximation);
    }

    private static LSMeshCollider CreateStripMesh(int quadCount)
    {
        var vertices = new Vector3d[(quadCount + 1) * 2];
        var triangles = new int[quadCount * 6];

        for (int i = 0; i <= quadCount; i++)
        {
            vertices[i * 2] = new Vector3d((Fixed64)i, Fixed64.Zero, Fixed64.Zero);
            vertices[i * 2 + 1] = new Vector3d((Fixed64)i, Fixed64.Zero, Fixed64.One);
        }

        for (int i = 0; i < quadCount; i++)
        {
            int vertex = i * 2;
            int triangle = i * 6;
            triangles[triangle] = vertex;
            triangles[triangle + 1] = vertex + 1;
            triangles[triangle + 2] = vertex + 2;
            triangles[triangle + 3] = vertex + 2;
            triangles[triangle + 4] = vertex + 1;
            triangles[triangle + 5] = vertex + 3;
        }

        return new LSMeshCollider(vertices, triangles);
    }
}
