using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Constraints;
using Gravitas.Diagnostics;
using Gravitas.Queries;
using Gravitas.Support;
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
    public void DisabledDiagnostics_ShouldIgnorePublicCaptureApisBeforeArgumentValidation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        using GravitasWorldContext context2D = Physics2DTestWorld.CreateContext();

        Action captureCollider = () => scenario.Context.Diagnostics.CaptureCollider(null!, GravitasDiagnosticColor.Cyan);
        Action captureMixed = () => context2D.Diagnostics.CaptureMixedCollider(null!, GravitasDiagnosticColor.Cyan);
        Action captureJoint3D = () => scenario.Context.Diagnostics.CaptureJoint((Joint3D)null!, GravitasDiagnosticColor.Cyan);
        Action captureJoint2D = () => context2D.Diagnostics.CaptureJoint((Joint2D)null!, GravitasDiagnosticColor.Cyan);

        captureCollider.Should().NotThrow();
        captureMixed.Should().NotThrow();
        captureJoint3D.Should().NotThrow();
        captureJoint2D.Should().NotThrow();
        scenario.Context.Diagnostics.CaptureLine(Vector3d.Zero, Vector3d.Right, GravitasDiagnosticColor.Cyan);
        scenario.Context.Diagnostics.CaptureRay(Vector3d.Zero, Vector3d.Right, Fixed64.One, GravitasDiagnosticColor.Cyan);
        scenario.Context.Diagnostics.CapturePoint(Vector3d.Zero, Fixed64.Half, GravitasDiagnosticColor.Cyan);

        scenario.Context.Diagnostics.DrawCommandCount.Should().Be(0);
        context2D.Diagnostics.DrawCommandCount.Should().Be(0);
    }

    [Fact]
    public void EnabledDiagnostics_ShouldRejectNullDrawCaptureInputs()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        using GravitasWorldContext context2D = Physics2DTestWorld.CreateContext();
        scenario.Context.Diagnostics.Enable(eventCapacity: 0, drawCommandCapacity: 1);
        context2D.Diagnostics.Enable(eventCapacity: 0, drawCommandCapacity: 1);

        Action captureCollider = () => scenario.Context.Diagnostics.CaptureCollider(null!, GravitasDiagnosticColor.Cyan);
        Action captureMixed = () => context2D.Diagnostics.CaptureMixedCollider(null!, GravitasDiagnosticColor.Cyan);
        Action captureJoint3D = () => scenario.Context.Diagnostics.CaptureJoint((Joint3D)null!, GravitasDiagnosticColor.Cyan);
        Action captureJoint2D = () => context2D.Diagnostics.CaptureJoint((Joint2D)null!, GravitasDiagnosticColor.Cyan);

        captureCollider.Should().Throw<ArgumentNullException>().WithParameterName("collider");
        captureMixed.Should().Throw<ArgumentNullException>().WithParameterName("collider");
        captureJoint3D.Should().Throw<ArgumentNullException>().WithParameterName("joint");
        captureJoint2D.Should().Throw<ArgumentNullException>().WithParameterName("joint");
    }

    [Fact]
    public void CaptureUnsupportedColliders_ShouldNotConsumeDrawSequence()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        using GravitasWorldContext context2D = Physics2DTestWorld.CreateContext();
        scenario.Context.Diagnostics.Enable(eventCapacity: 0, drawCommandCapacity: 1);
        context2D.Diagnostics.Enable(eventCapacity: 0, drawCommandCapacity: 1);

        scenario.Context.Diagnostics.CaptureCollider(new UnsupportedTestCollider3D(), GravitasDiagnosticColor.Cyan);
        context2D.Diagnostics.CaptureMixedCollider(new UnsupportedTestCollider2D(), GravitasDiagnosticColor.Cyan);
        scenario.Context.Diagnostics.CaptureLine(Vector3d.Zero, Vector3d.Right, GravitasDiagnosticColor.Green);
        context2D.Diagnostics.CaptureLine(Vector3d.Zero, Vector3d.Right, GravitasDiagnosticColor.Green);

        GravitasDebugDrawCommand command3D = scenario.Context.Diagnostics.DrawCommands.Should().ContainSingle().Which;
        command3D.Kind.Should().Be(GravitasDebugDrawKind.Line);
        command3D.Sequence.Should().Be(0);
        GravitasDebugDrawCommand command2D = context2D.Diagnostics.DrawCommands.Should().ContainSingle().Which;
        command2D.Kind.Should().Be(GravitasDebugDrawKind.Line);
        command2D.Sequence.Should().Be(0);
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
    public void SuccessfulGroundProbeDiagnostics_ShouldIdentifyHitCollider()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var ground = new LSCuboidCollider
        {
            Layer = new PhysicsLayer(1),
            Size = new Vector3d((Fixed64)8, Fixed64.One, (Fixed64)8)
        };
        scenario.InitializeStaticCollider(
            ground,
            new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero));
        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        scenario.Context.Diagnostics.Enable(eventCapacity: 2, drawCommandCapacity: 0);

        body.Body.CheckGround();

        GravitasDiagnosticEvent probe = scenario.Context.Diagnostics.Events[^1];
        probe.Kind.Should().Be(GravitasDiagnosticEventKind.GroundProbe);
        probe.BodyId.Should().Be(body.Body.DynamicId);
        probe.ColliderAId.Should().Be(body.Collider.Id);
        probe.ColliderBId.Should().Be(ground.Id);
        probe.ColliderBType.Should().Be(ground.Shape);
        probe.ColliderBDimension.Should().Be(GravitasColliderDimension.ThreeD);
        probe.PointA.Should().Be(body.Body.HitPoint);
        probe.Vector.Should().Be(body.Body.GroundNormal);
        probe.Hit.Should().BeTrue();
    }

    [Fact]
    public void CollisionDiagnostics_ShouldRecordContactResponseAndVelocityDeltasInOrder()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        first.Body.AddLinearImpulse(Vector3d.Right * Fixed64.FromFraction(5, 16) * first.Body.Mass);
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
    public void EnabledDiagnostics_ShouldRecordNoHitQuerySummaryAndContactPayloads()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(4, 0, 0));
        CollisionPair pair = scenario.CreatePair(first.Collider, second.Collider);
        var noHit = new Physics3DHit(
            null,
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.Zero,
            Vector3d.Zero);

        scenario.Context.Diagnostics.Enable(eventCapacity: 4, drawCommandCapacity: 0);
        scenario.Context.Diagnostics.EmitCircleQuery(
            Vector3d.Zero,
            Fixed64.One,
            Vector3d.Right,
            (Fixed64)4,
            layerMaskBits: 0x2,
            hit: false,
            hitCount: 0,
            noHit);
        scenario.Context.Diagnostics.EmitQuerySummary(
            GravitasColliderDimension.ThreeD,
            GravitasColliderDimension.ThreeD,
            Vector3d.Zero,
            Vector3d.Right,
            exactReducerAttempts: 1,
            acceptedHits: 0,
            fallbackHits: 0,
            rejectedConservativeCandidates: 0);
        scenario.Context.Diagnostics.EmitContact(pair, hit: false);

        ReadOnlySpan<GravitasDiagnosticEvent> events = scenario.Context.Diagnostics.Events;
        events.Length.Should().Be(3);
        events[0].Kind.Should().Be(GravitasDiagnosticEventKind.CircleQuery);
        events[0].ColliderAId.Should().Be(-1);
        events[0].ColliderAType.Should().Be(ColliderType.None);
        events[0].Hit.Should().BeFalse();
        events[1].Kind.Should().Be(GravitasDiagnosticEventKind.QuerySummary);
        events[1].Hit.Should().BeFalse();
        events[2].Kind.Should().Be(GravitasDiagnosticEventKind.Contact);
        events[2].Hit.Should().BeFalse();
        events[2].DataA.Should().Be(0);
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
        ScenarioBody<LSConeCollider> cone = scenario.CreateCone(PhysicsScenarioBuilder.Vector(8, 0, 0));
        ScenarioBody<LSCompoundCollider> compound = scenario.CreateBody(
            new LSCompoundCollider(
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero)),
                CompoundColliderPart.Cone(Fixed64.Half, Fixed64.One, Vector3d.Zero),
                CompoundColliderPart.Cuboid(Vector3d.One, new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero))),
            PhysicsScenarioBuilder.Vector(10, 0, 0),
            FixedQuaternion.Identity);
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            CreateTriangleMesh(),
            PhysicsScenarioBuilder.Vector(12, 0, 0),
            FixedQuaternion.Identity);

        scenario.Context.Diagnostics.Enable(eventCapacity: 4, drawCommandCapacity: 16);
        scenario.Context.Diagnostics.CaptureCollider(sphere.Collider, GravitasDiagnosticColor.Cyan);
        scenario.Context.Diagnostics.CaptureCollider(capsule.Collider, GravitasDiagnosticColor.Green);
        scenario.Context.Diagnostics.CaptureCollider(cuboid.Collider, GravitasDiagnosticColor.Yellow);
        scenario.Context.Diagnostics.CaptureCollider(cylinder.Collider, GravitasDiagnosticColor.Red);
        scenario.Context.Diagnostics.CaptureCollider(cone.Collider, GravitasDiagnosticColor.Cyan);
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
        commands.Length.Should().Be(12);
        commands[0].Kind.Should().Be(GravitasDebugDrawKind.WireSphere);
        commands[0].ColliderId.Should().Be(sphere.Collider.Id);
        commands[1].Kind.Should().Be(GravitasDebugDrawKind.WireCapsule);
        commands[2].Kind.Should().Be(GravitasDebugDrawKind.WireBox);
        commands[2].Rotation.Should().Be(cuboid.Collider.Rotation);
        commands[3].Kind.Should().Be(GravitasDebugDrawKind.WireCylinder);
        commands[4].Kind.Should().Be(GravitasDebugDrawKind.WireCone);
        commands[4].ColliderId.Should().Be(cone.Collider.Id);
        commands[4].Radius.Should().Be(cone.Collider.ScaledRadius);
        commands[4].Height.Should().Be(cone.Collider.Height);
        commands[5].Kind.Should().Be(GravitasDebugDrawKind.WireSphere);
        commands[5].ColliderId.Should().Be(compound.Collider.Id);
        commands[5].ColliderType.Should().Be(ColliderType.Compound);
        commands[6].Kind.Should().Be(GravitasDebugDrawKind.WireCone);
        commands[6].ColliderId.Should().Be(compound.Collider.Id);
        commands[6].ColliderType.Should().Be(ColliderType.Compound);
        commands[7].Kind.Should().Be(GravitasDebugDrawKind.WireBox);
        commands[7].ColliderId.Should().Be(compound.Collider.Id);
        commands[7].ColliderType.Should().Be(ColliderType.Compound);
        commands[8].Kind.Should().Be(GravitasDebugDrawKind.WireTriangle);
        commands[8].PointA.Should().Be(mesh.Collider.Mesh.Vertices[0]);
        commands[8].PointB.Should().Be(mesh.Collider.Mesh.Vertices[1]);
        commands[8].PointC.Should().Be(mesh.Collider.Mesh.Vertices[2]);
        commands[9].Kind.Should().Be(GravitasDebugDrawKind.Line);
        commands[9].Start.Should().Be(PhysicsScenarioBuilder.Vector(0, 0, 0));
        commands[9].End.Should().Be(PhysicsScenarioBuilder.Vector(1, 0, 0));
        commands[10].Kind.Should().Be(GravitasDebugDrawKind.Ray);
        commands[10].Start.Should().Be(PhysicsScenarioBuilder.Vector(1, 0, 0));
        commands[10].End.Should().Be(PhysicsScenarioBuilder.Vector(1, 0, 2));
        commands[11].Kind.Should().Be(GravitasDebugDrawKind.Point);
        commands[11].Center.Should().Be(PhysicsScenarioBuilder.Vector(2, 0, 0));
        commands[11].Radius.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void CaptureRay_WithZeroDirection_ShouldEmitDegenerateRayAtOrigin()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Vector3d origin = PhysicsScenarioBuilder.Vector(3, 4, 5);

        scenario.Context.Diagnostics.Enable(eventCapacity: 0, drawCommandCapacity: 1);
        scenario.Context.Diagnostics.CaptureRay(
            origin,
            Vector3d.Zero,
            (Fixed64)8,
            GravitasDiagnosticColor.Green);

        ReadOnlySpan<GravitasDebugDrawCommand> commands = scenario.Context.Diagnostics.DrawCommands;
        commands.Length.Should().Be(1);
        commands[0].Kind.Should().Be(GravitasDebugDrawKind.Ray);
        commands[0].Start.Should().Be(origin);
        commands[0].End.Should().Be(origin);
    }

    [Fact]
    public void CaptureMixedCollider_WithCompound2D_ShouldEmitPartCommandsUsingOwnerId()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(-Fixed64.One, Fixed64.Zero)),
            CompoundColliderPart2D.AABBox(Vector2d.One, new Vector2d(Fixed64.One, Fixed64.Zero)));
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One));
        compound.InitializeWithNoBody(agent);

        context.Diagnostics.Enable(eventCapacity: 4, drawCommandCapacity: 8);
        context.Diagnostics.CaptureMixedCollider(compound, GravitasDiagnosticColor.Cyan);

        ReadOnlySpan<GravitasDebugDrawCommand> commands = context.Diagnostics.DrawCommands;
        commands.Length.Should().Be(2);
        commands[0].ColliderId.Should().Be(compound.Id);
        commands[0].ColliderDimension.Should().Be(GravitasColliderDimension.TwoD);
        commands[0].Collider2DType.Should().Be(ColliderType2D.Compound);
        commands[1].ColliderId.Should().Be(compound.Id);
        commands[1].ColliderDimension.Should().Be(GravitasColliderDimension.TwoD);
        commands[1].Collider2DType.Should().Be(ColliderType2D.Compound);
    }

    [Fact]
    public void CaptureCollider_WithCompoundCapsuleCylinderAndMeshParts_ShouldEmitOwnerCommands()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCompoundCollider> compound = scenario.CreateBody(
            new LSCompoundCollider(
                CompoundColliderPart.Capsule(
                    Fixed64.Half,
                    (Fixed64)2,
                    new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero)),
                CompoundColliderPart.Cylinder(
                    Fixed64.Half,
                    (Fixed64)2,
                    Vector3d.Zero),
                CompoundColliderPart.ConvexMesh(
                    TriangleVertices(),
                    TriangleIndices(),
                    new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero),
                    MeshInertiaPolicy.SurfaceApproximation)),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity);

        scenario.Context.Diagnostics.Enable(eventCapacity: 0, drawCommandCapacity: 3);
        scenario.Context.Diagnostics.CaptureCollider(compound.Collider, GravitasDiagnosticColor.Blue);

        ReadOnlySpan<GravitasDebugDrawCommand> commands = scenario.Context.Diagnostics.DrawCommands;
        commands.Length.Should().Be(3);
        AssertDrawCommandMetadata(commands[0], GravitasDebugDrawKind.WireCapsule, compound.Collider.Id, ColliderType.Compound);
        AssertDrawCommandMetadata(commands[1], GravitasDebugDrawKind.WireCylinder, compound.Collider.Id, ColliderType.Compound);
        AssertDrawCommandMetadata(commands[2], GravitasDebugDrawKind.WireTriangle, compound.Collider.Id, ColliderType.Compound);
    }

    [Fact]
    public void CaptureMixedCollider_WithPolygonAndCapsuleShapes_ShouldEmitSlabCommands()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        var polygon = new LSPolygonCollider2D(DiamondVertices());
        var capsule = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3);
        var degenerateCapsule = new LSCapsuleCollider2D(Fixed64.Half, Fixed64.One);
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Capsule(
                Fixed64.Half,
                (Fixed64)3,
                new Vector2d(-Fixed64.One, Fixed64.Zero)),
            CompoundColliderPart2D.ConvexPolygon(
                DiamondVertices(),
                new Vector2d(Fixed64.One, Fixed64.Zero)));
        InitializeBodylessCollider(context, polygon, Vector2d.Zero);
        InitializeBodylessCollider(context, capsule, new Vector2d((Fixed64)3, Fixed64.Zero));
        InitializeBodylessCollider(context, degenerateCapsule, new Vector2d((Fixed64)6, Fixed64.Zero));
        InitializeBodylessCollider(context, compound, new Vector2d((Fixed64)9, Fixed64.Zero));

        context.Diagnostics.Enable(eventCapacity: 0, drawCommandCapacity: 40);
        context.Diagnostics.CaptureMixedCollider(polygon, GravitasDiagnosticColor.Cyan);
        context.Diagnostics.CaptureMixedCollider(capsule, GravitasDiagnosticColor.Green);
        context.Diagnostics.CaptureMixedCollider(degenerateCapsule, GravitasDiagnosticColor.Yellow);
        context.Diagnostics.CaptureMixedCollider(compound, GravitasDiagnosticColor.Blue);

        ReadOnlySpan<GravitasDebugDrawCommand> commands = context.Diagnostics.DrawCommands;
        commands.Length.Should().Be(32);
        Assert2DDrawCommandRange(commands, 0, 12, GravitasDebugDrawKind.Line, polygon.Id, ColliderType2D.ConvexPolygon);
        Assert2DDrawCommandMetadata(commands[12], GravitasDebugDrawKind.WireCylinder, capsule.Id, ColliderType2D.Capsule);
        Assert2DDrawCommandMetadata(commands[13], GravitasDebugDrawKind.WireCylinder, capsule.Id, ColliderType2D.Capsule);
        Assert2DDrawCommandMetadata(commands[14], GravitasDebugDrawKind.WireBox, capsule.Id, ColliderType2D.Capsule);
        Assert2DDrawCommandMetadata(commands[15], GravitasDebugDrawKind.WireCylinder, degenerateCapsule.Id, ColliderType2D.Capsule);
        Assert2DDrawCommandMetadata(commands[16], GravitasDebugDrawKind.WireCylinder, degenerateCapsule.Id, ColliderType2D.Capsule);
        Assert2DDrawCommandRange(commands, 17, 15, null, compound.Id, ColliderType2D.Compound);
        commands[17].Kind.Should().Be(GravitasDebugDrawKind.WireCylinder);
        commands[18].Kind.Should().Be(GravitasDebugDrawKind.WireCylinder);
        commands[19].Kind.Should().Be(GravitasDebugDrawKind.WireBox);
        Assert2DDrawCommandRange(commands, 20, 12, GravitasDebugDrawKind.Line, compound.Id, ColliderType2D.Compound);
    }

    [Fact]
    public void EnabledDiagnostics_ShouldEmitCircleQueryJointLimitAndRagdollPayloads()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> hit = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(2, 0, 0));
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(4, 0, 0));
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));
        var queryHit = new Physics3DHit(
            hit.Collider,
            PhysicsScenarioBuilder.Vector(1, 0, 0),
            Vector3d.Left,
            (Fixed64)3,
            Vector3d.Right);

        scenario.Context.Diagnostics.Enable(eventCapacity: 4, drawCommandCapacity: 0);
        scenario.Context.Diagnostics.EmitCircleQuery(
            PhysicsScenarioBuilder.Vector(-2, 0, 0),
            Fixed64.Half,
            Vector3d.Right,
            (Fixed64)5,
            layerMaskBits: 0x1F,
            hit: true,
            hitCount: 2,
            queryHit);
        scenario.Context.Diagnostics.EmitJointLimitReached(joint, Fixed64.Half);
        scenario.Context.Diagnostics.EmitRagdollActivated(ragdollId: 9, linkCount: 4, jointCount: 3, isActive: true);

        ReadOnlySpan<GravitasDiagnosticEvent> events = scenario.Context.Diagnostics.Events;
        events.Length.Should().Be(3);
        events[0].Kind.Should().Be(GravitasDiagnosticEventKind.CircleQuery);
        events[0].ColliderAId.Should().Be(hit.Collider.Id);
        events[0].ColliderAType.Should().Be(ColliderType.Sphere);
        events[0].End.Should().Be(PhysicsScenarioBuilder.Vector(3, 0, 0));
        events[0].ScalarA.Should().Be(Fixed64.Half);
        events[0].ScalarB.Should().Be((Fixed64)3);
        events[0].DataA.Should().Be(0x1F);
        events[0].DataB.Should().Be(2);
        events[0].Hit.Should().BeTrue();
        events[1].Kind.Should().Be(GravitasDiagnosticEventKind.JointLimitReached);
        events[1].JointId.Should().Be(joint.Id);
        events[1].ColliderAId.Should().Be(first.Collider.Id);
        events[1].ColliderBId.Should().Be(second.Collider.Id);
        events[1].ScalarB.Should().Be(Fixed64.Half);
        events[2].Kind.Should().Be(GravitasDiagnosticEventKind.RagdollActivated);
        events[2].BodyId.Should().Be(9);
        events[2].DataA.Should().Be(4);
        events[2].DataB.Should().Be(3);
        events[2].Hit.Should().BeTrue();
    }

    [Fact]
    public void EnabledDiagnostics_ShouldEmit2DJointLimitPayload()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D first = CreateBody2D(context, Vector2d.Zero);
        SolidBody2D second = CreateBody2D(context, Vector2d.Right * (Fixed64)2);
        Joint2D joint = context.Constraints2D.RegisterJoint(CreatePrismatic2D(first, second));

        context.Diagnostics.Enable(eventCapacity: 2, drawCommandCapacity: 0);
        context.Diagnostics.EmitJointLimitReached(joint, Fixed64.Half);

        ReadOnlySpan<GravitasDiagnosticEvent> events = context.Diagnostics.Events;
        events.Length.Should().Be(1);
        events[0].Kind.Should().Be(GravitasDiagnosticEventKind.JointLimitReached);
        events[0].JointId.Should().Be(joint.Id);
        events[0].ColliderADimension.Should().Be(GravitasColliderDimension.TwoD);
        events[0].ColliderBDimension.Should().Be(GravitasColliderDimension.TwoD);
        events[0].ColliderA2DType.Should().Be(ColliderType2D.Circle);
        events[0].ColliderB2DType.Should().Be(ColliderType2D.Circle);
        events[0].ScalarB.Should().Be(Fixed64.Half);
        events[0].Hit.Should().BeTrue();
    }

    [Fact]
    public void CaptureJoint_ShouldEmitDimensionalJointDrawCommands()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first3D = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> second3D = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(2, 0, 0));
        Joint3D hinge = scenario.Context.Constraints3D.RegisterJoint(Create3DJoint(first3D.Body, second3D.Body, JointType3D.Hinge));
        Joint3D coneTwist = scenario.Context.Constraints3D.RegisterJoint(Create3DJoint(first3D.Body, second3D.Body, JointType3D.ConeTwist));
        Joint3D fixedJoint = scenario.Context.Constraints3D.RegisterJoint(Create3DJoint(first3D.Body, second3D.Body, JointType3D.Fixed));
        using GravitasWorldContext context2D = Physics2DTestWorld.CreateContext();
        SolidBody2D first2D = CreateBody2D(context2D, Vector2d.Zero);
        SolidBody2D second2D = CreateBody2D(context2D, Vector2d.Right * (Fixed64)2);
        Joint2D prismatic = context2D.Constraints2D.RegisterJoint(CreatePrismatic2D(first2D, second2D));

        scenario.Context.Diagnostics.Enable(eventCapacity: 0, drawCommandCapacity: 20);
        scenario.Context.Diagnostics.CaptureJoint(hinge, GravitasDiagnosticColor.Cyan);
        scenario.Context.Diagnostics.CaptureJoint(coneTwist, GravitasDiagnosticColor.Green);
        scenario.Context.Diagnostics.CaptureJoint(fixedJoint, GravitasDiagnosticColor.Yellow);
        context2D.Diagnostics.Enable(eventCapacity: 0, drawCommandCapacity: 4);
        context2D.Diagnostics.CaptureJoint(prismatic, GravitasDiagnosticColor.Blue);

        ReadOnlySpan<GravitasDebugDrawCommand> commands3D = scenario.Context.Diagnostics.DrawCommands;
        commands3D.Length.Should().Be(16);
        AssertDrawCommandMetadata(commands3D[0], GravitasDebugDrawKind.Point, first3D.Collider.Id, ColliderType.Sphere);
        AssertDrawCommandMetadata(commands3D[1], GravitasDebugDrawKind.Point, second3D.Collider.Id, ColliderType.Sphere);
        AssertDrawCommandMetadata(commands3D[2], GravitasDebugDrawKind.Line, first3D.Collider.Id, ColliderType.Sphere);
        AssertDrawCommandMetadata(commands3D[3], GravitasDebugDrawKind.Ray, first3D.Collider.Id, ColliderType.Sphere);
        AssertDrawCommandMetadata(commands3D[4], GravitasDebugDrawKind.Ray, second3D.Collider.Id, ColliderType.Sphere);
        AssertDrawCommandMetadata(commands3D[8], GravitasDebugDrawKind.Ray, first3D.Collider.Id, ColliderType.Sphere);
        AssertDrawCommandMetadata(commands3D[9], GravitasDebugDrawKind.Ray, second3D.Collider.Id, ColliderType.Sphere);
        AssertDrawCommandMetadata(commands3D[13], GravitasDebugDrawKind.Ray, first3D.Collider.Id, ColliderType.Sphere);
        AssertDrawCommandMetadata(commands3D[14], GravitasDebugDrawKind.Ray, first3D.Collider.Id, ColliderType.Sphere);
        AssertDrawCommandMetadata(commands3D[15], GravitasDebugDrawKind.Ray, first3D.Collider.Id, ColliderType.Sphere);

        ReadOnlySpan<GravitasDebugDrawCommand> commands2D = context2D.Diagnostics.DrawCommands;
        commands2D.Length.Should().Be(4);
        Assert2DDrawCommandMetadata(commands2D[0], GravitasDebugDrawKind.Point, first2D.Collider.Id, ColliderType2D.Circle);
        Assert2DDrawCommandMetadata(commands2D[1], GravitasDebugDrawKind.Point, second2D.Collider.Id, ColliderType2D.Circle);
        Assert2DDrawCommandMetadata(commands2D[2], GravitasDebugDrawKind.Line, first2D.Collider.Id, ColliderType2D.Circle);
        Assert2DDrawCommandMetadata(commands2D[3], GravitasDebugDrawKind.Ray, first2D.Collider.Id, ColliderType2D.Circle);
    }

    [Fact]
    public void CaptureJoint_WithZeroAuthoredFrameRotation_ShouldEmitUnitAxisRays()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(2, 0, 0));
        var zeroRotationFrame = new FixedTransform(Vector3d.Zero, default, Vector3d.One);
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(
            new JointDefinition3D(
                first.Body,
                second.Body,
                zeroRotationFrame,
                zeroRotationFrame,
                JointType3D.Hinge,
                JointLimit3D.Unrestricted,
                JointMotor3D.Disabled,
                JointCollisionPolicy.SuppressLinked));

        scenario.Context.Diagnostics.Enable(eventCapacity: 0, drawCommandCapacity: 5);
        scenario.Context.Diagnostics.CaptureJoint(joint, GravitasDiagnosticColor.Cyan);

        ReadOnlySpan<GravitasDebugDrawCommand> commands = scenario.Context.Diagnostics.DrawCommands;
        commands.Length.Should().Be(5);
        commands[3].End.Should().Be(commands[3].Start + Vector3d.Right);
        commands[4].End.Should().Be(commands[4].Start + Vector3d.Right);
    }

    [Fact]
    public void CaptureJoint_WithNonPrismatic2DJoint_ShouldEmitAnchorsWithoutAxisRay()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D first = CreateBody2D(context, Vector2d.Zero);
        SolidBody2D second = CreateBody2D(context, Vector2d.Right * (Fixed64)2);
        Joint2D joint = context.Constraints2D.RegisterJoint(Create2DJoint(first, second, JointType2D.Distance));

        context.Diagnostics.Enable(eventCapacity: 0, drawCommandCapacity: 4);
        context.Diagnostics.CaptureJoint(joint, GravitasDiagnosticColor.Cyan);

        ReadOnlySpan<GravitasDebugDrawCommand> commands = context.Diagnostics.DrawCommands;
        commands.Length.Should().Be(3);
        Assert2DDrawCommandMetadata(commands[0], GravitasDebugDrawKind.Point, first.Collider.Id, ColliderType2D.Circle);
        Assert2DDrawCommandMetadata(commands[1], GravitasDebugDrawKind.Point, second.Collider.Id, ColliderType2D.Circle);
        Assert2DDrawCommandMetadata(commands[2], GravitasDebugDrawKind.Line, first.Collider.Id, ColliderType2D.Circle);
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
        => AllocationTestHelper.MeasureSteadyState(action, warmupIterations: 1);

    private static void InitializeBodylessCollider(
        GravitasWorldContext context,
        LSCollider2D collider,
        Vector2d position)
    {
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One));
        collider.InitializeWithNoBody(agent);
    }

    private static SolidBody2D CreateBody2D(GravitasWorldContext context, Vector2d position)
    {
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One));
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half));
        body.Initialize(position);
        return body;
    }

    private static JointDefinition3D CreateBallSocket(SolidBody first, SolidBody second) =>
        Create3DJoint(first, second, JointType3D.BallSocket);

    private static JointDefinition3D Create3DJoint(SolidBody first, SolidBody second, JointType3D type) =>
        new(
            first,
            second,
            IdentityTransform(),
            IdentityTransform(),
            type,
            JointLimit3D.Unrestricted,
            JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked);

    private static JointDefinition2D CreatePrismatic2D(SolidBody2D first, SolidBody2D second) =>
        Create2DJoint(first, second, JointType2D.Prismatic);

    private static JointDefinition2D Create2DJoint(SolidBody2D first, SolidBody2D second, JointType2D type) =>
        new(
            first,
            second,
            new JointFrame2D(Vector2d.Right * Fixed64.Half, Fixed64.Zero),
            new JointFrame2D(-Vector2d.Right * Fixed64.Half, Fixed64.Zero),
            type,
            JointLimit2D.Unrestricted,
            JointMotor2D.Disabled,
            JointCollisionPolicy.SuppressLinked);

    private static FixedTransform IdentityTransform() =>
        new(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);

    private static void AssertDrawCommandMetadata(
        GravitasDebugDrawCommand command,
        GravitasDebugDrawKind kind,
        int colliderId,
        ColliderType colliderType)
    {
        command.Kind.Should().Be(kind);
        command.ColliderId.Should().Be(colliderId);
        command.ColliderType.Should().Be(colliderType);
        command.ColliderDimension.Should().Be(GravitasColliderDimension.ThreeD);
    }

    private static void Assert2DDrawCommandRange(
        ReadOnlySpan<GravitasDebugDrawCommand> commands,
        int start,
        int count,
        GravitasDebugDrawKind? kind,
        int colliderId,
        ColliderType2D colliderType)
    {
        for (int i = start; i < start + count; i++)
        {
            if (kind.HasValue)
                commands[i].Kind.Should().Be(kind.Value);
            Assert2DDrawCommandMetadata(commands[i], commands[i].Kind, colliderId, colliderType);
        }
    }

    private static void Assert2DDrawCommandMetadata(
        GravitasDebugDrawCommand command,
        GravitasDebugDrawKind kind,
        int colliderId,
        ColliderType2D colliderType)
    {
        command.Kind.Should().Be(kind);
        command.ColliderId.Should().Be(colliderId);
        command.ColliderDimension.Should().Be(GravitasColliderDimension.TwoD);
        command.Collider2DType.Should().Be(colliderType);
    }

    private static LSMeshCollider CreateTriangleMesh()
    {
        Vector3d[] vertices = TriangleVertices();
        return new LSMeshCollider(
            vertices,
            TriangleIndices(),
            MeshColliderMode.Convex,
            MeshInertiaPolicy.SurfaceApproximation);
    }

    private static Vector2d[] DiamondVertices() =>
        new[]
        {
            new Vector2d(Fixed64.Zero, Fixed64.One),
            new Vector2d(Fixed64.One, Fixed64.Zero),
            new Vector2d(Fixed64.Zero, -Fixed64.One),
            new Vector2d(-Fixed64.One, Fixed64.Zero)
        };

    private static Vector3d[] TriangleVertices() =>
        new[]
        {
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.One)
        };

    private static int[] TriangleIndices() => new[] { 0, 1, 2 };

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
