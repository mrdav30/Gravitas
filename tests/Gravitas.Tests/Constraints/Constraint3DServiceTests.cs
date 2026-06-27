using Chronicler;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Constraints;
using Gravitas.CollisionHandling;
using Gravitas.Diagnostics;
using Gravitas.Tests.Serialization;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Constraints;

public sealed class Constraint3DServiceTests
{
    public static TheoryData<GravitasSerializationTransport> Transports => GravitasSerializationTransportCases.All();

    [Fact]
    public void NewContext_ShouldOwnEmptyConstraintService()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();

        scenario.Context.Constraints3D.Should().NotBeNull();
        scenario.Context.Constraints3D.RegisteredJointCount.Should().Be(0);
        scenario.Context.Constraints3D.PeakJointCount.Should().Be(0);
    }

    [Fact]
    public void RegisterJoint_ShouldAssignDeterministicMonotonicIdsAndAllowDuplicateBodyPairs()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        JointDefinition3D definition = CreateBallSocket(first.Body, second.Body);

        Joint3D firstJoint = scenario.Context.Constraints3D.RegisterJoint(definition);
        Joint3D secondJoint = scenario.Context.Constraints3D.RegisterJoint(definition);

        firstJoint.Id.Should().Be(1);
        secondJoint.Id.Should().Be(2);
        firstJoint.Should().NotBeSameAs(secondJoint);
        scenario.Context.Constraints3D.RegisteredJointCount.Should().Be(2);
        scenario.Context.Constraints3D.PeakJointCount.Should().Be(2);
        scenario.Context.Constraints3D.TryGetJoint(1, out Joint3D? resolved).Should().BeTrue();
        resolved.Should().BeSameAs(firstJoint);
    }

    [Fact]
    public void RemoveJoint_ShouldReleaseRuntimeStateAndPreventLookup()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));

        scenario.Context.Constraints3D.RemoveJoint(joint.Id).Should().BeTrue();

        joint.IsActive.Should().BeFalse();
        scenario.Context.Constraints3D.RegisteredJointCount.Should().Be(0);
        scenario.Context.Constraints3D.TryGetJoint(joint.Id, out _).Should().BeFalse();
    }

    [Fact]
    public void RegisterJoint_WithInvalidDefinition_ShouldFailBeforeSolverStateIsCreated()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        Action sameBody = () => scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(body.Body, body.Body));
        Action nullFrame = () => scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            body.Body,
            scenario.CreateSphere(Vector3d.Right * (Fixed64)2).Body,
            localFrameA: null!,
            localFrameB: LocalFrame(Vector3d.Zero),
            JointType3D.BallSocket,
            JointLimit3D.Unrestricted,
            JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        sameBody.Should().Throw<ArgumentException>();
        nullFrame.Should().Throw<ArgumentNullException>();
        scenario.Context.Constraints3D.RegisteredJointCount.Should().Be(0);
    }

    [Fact]
    public void RegisterJoint_WithBodiesFromDifferentContexts_ShouldFail()
    {
        using PhysicsScenarioBuilder firstScenario = PhysicsScenarioBuilder.Create();
        using PhysicsScenarioBuilder secondScenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = firstScenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = secondScenario.CreateSphere(Vector3d.Right * (Fixed64)2);

        Action act = () => firstScenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));

        act.Should().Throw<ArgumentException>();
    }

    [Fact]
    public void DirectJoint_ShouldSuppressAdjacentLinkedCollisionByDefault()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * Fixed64.Half);

        scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        first.Collider.TryGetCollisionPair(second.Collider.Id, out _).Should().BeFalse();
        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(first.Collider, second.Collider).Should().BeTrue();
    }

    [Fact]
    public void DirectJoint_WithCollidePolicy_ShouldAllowLinkedCollision()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * Fixed64.Half);

        scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(
            first.Body,
            second.Body,
            collisionPolicy: JointCollisionPolicy.Collide));
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        first.Collider.TryGetCollisionPair(second.Collider.Id, out CollisionPair? pair).Should().BeTrue();
        pair!.Manifold.HasContact.Should().BeTrue();
    }

    [Fact]
    public void RagdollFiltering_ShouldSuppressAdjacentLinksButAllowNonAdjacentByDefault()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> root = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> middle = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        ScenarioBody<LSSphereCollider> end = scenario.CreateSphere(Vector3d.Right * Fixed64.Half);
        RagdollRuntime3D runtime = scenario.Context.Constraints3D.RegisterRagdoll(new RagdollDefinition3D(
            new[]
            {
                new RagdollLinkDefinition3D(0, root.Body, root.Collider),
                new RagdollLinkDefinition3D(1, middle.Body, middle.Collider),
                new RagdollLinkDefinition3D(2, end.Body, end.Collider)
            },
            new[]
            {
                new RagdollJointDefinition3D(0, 1, JointType3D.BallSocket, LocalFrame(Vector3d.Zero), LocalFrame(Vector3d.Zero)),
                new RagdollJointDefinition3D(1, 2, JointType3D.BallSocket, LocalFrame(Vector3d.Zero), LocalFrame(Vector3d.Zero))
            }));

        runtime.LinkCount.Should().Be(3);
        runtime.JointCount.Should().Be(2);
        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(root.Collider, middle.Collider).Should().BeTrue();
        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(middle.Collider, end.Collider).Should().BeTrue();
        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(root.Collider, end.Collider).Should().BeFalse();
    }

    [Fact]
    public void RagdollFiltering_WithSuppressAllPolicy_ShouldSuppressNonAdjacentLinks()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> root = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> middle = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        ScenarioBody<LSSphereCollider> end = scenario.CreateSphere(Vector3d.Right * Fixed64.Half);

        scenario.Context.Constraints3D.RegisterRagdoll(new RagdollDefinition3D(
            new[]
            {
                new RagdollLinkDefinition3D(0, root.Body, root.Collider),
                new RagdollLinkDefinition3D(1, middle.Body, middle.Collider),
                new RagdollLinkDefinition3D(2, end.Body, end.Collider)
            },
            new[]
            {
                new RagdollJointDefinition3D(0, 1, JointType3D.BallSocket, LocalFrame(Vector3d.Zero), LocalFrame(Vector3d.Zero)),
                new RagdollJointDefinition3D(1, 2, JointType3D.BallSocket, LocalFrame(Vector3d.Zero), LocalFrame(Vector3d.Zero))
            },
            RagdollSelfCollisionPolicy.SuppressAllLinks));

        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(root.Collider, end.Collider).Should().BeTrue();
    }

    [Fact]
    public void BallSocketJoint_ShouldReduceAnchorSeparationThroughImpulses()
    {
        ConstraintState before;
        ConstraintState after;

        using (PhysicsScenarioBuilder scenario = CreateConstraintScenario())
        {
            ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero, preventAngularForces: true);
            ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)3, preventAngularForces: true);
            Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
                first.Body,
                second.Body,
                LocalFrame(Vector3d.Right * Fixed64.Half),
                LocalFrame(-Vector3d.Right * Fixed64.Half),
                JointType3D.BallSocket,
                JointLimit3D.Unrestricted,
                JointMotor3D.Disabled,
                JointCollisionPolicy.SuppressLinked));

            before = CaptureConstraintState(first.Body, second.Body, joint);
            Step(scenario.Context, 12);
            after = CaptureConstraintState(first.Body, second.Body, joint);
        }

        after.AnchorDistanceSquared.Should().BeLessThan(before.AnchorDistanceSquared);
        after.JointSolvedRowCount.Should().BeGreaterThan(0);
        after.AccumulatedImpulseMagnitude.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void FixedJoint_ShouldReduceAngularFrameError()
    {
        Fixed64 beforeError;
        Fixed64 afterError;

        using (PhysicsScenarioBuilder scenario = CreateConstraintScenario())
        {
            ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
            ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(
                Vector3d.Right * (Fixed64)2,
                FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.FromFraction(1, 3)));
            scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
                first.Body,
                second.Body,
                LocalFrame(Vector3d.Zero),
                LocalFrame(Vector3d.Zero),
                JointType3D.Fixed,
                JointLimit3D.Unrestricted,
                JointMotor3D.Disabled,
                JointCollisionPolicy.SuppressLinked));

            beforeError = FixedQuaternion.Angle(first.Body.Rotation, second.Body.Rotation);
            Step(scenario.Context, 16);
            afterError = FixedQuaternion.Angle(first.Body.Rotation, second.Body.Rotation);
        }

        afterError.Should().BeLessThan(beforeError);
    }

    [Fact]
    public void HingeJoint_ShouldAlignHingeAxesWithoutLockingHingeRotation()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(
            Vector3d.Right * (Fixed64)2,
            FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.FromFraction(1, 4)));
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            first.Body,
            second.Body,
            LocalFrame(Vector3d.Zero),
            LocalFrame(Vector3d.Zero),
            JointType3D.Hinge,
            JointLimit3D.Hinge(Fixed64.FromFraction(1, 2)),
            JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked));
        Fixed64 before = Vector3d.Cross(first.Body.Right, second.Body.Right).MagnitudeSquared;

        Step(scenario.Context, 16);

        Fixed64 after = Vector3d.Cross(first.Body.Right, second.Body.Right).MagnitudeSquared;
        after.Should().BeLessThan(before);
        joint.LastSolvedRowCount.Should().BeGreaterThanOrEqualTo(4);
    }

    [Fact]
    public void ConstraintSolver_ShouldRespectFrozenBodyAxes()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero, preventAngularForces: true);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)3, preventAngularForces: true);
        second.Body.FreezeAxes = BodyFreezeAxes3D.PositionX | BodyFreezeAxes3D.Rotation;
        scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));
        Vector3d frozenStart = second.Body.Position3d;

        Step(scenario.Context, 12);

        second.Body.Position3d.X.Should().Be(frozenStart.X);
        first.Body.Position3d.X.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void ConstraintIsland_ShouldWakeSleepingLinkedBodiesAsOneIsland()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> sleeping = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> driver = scenario.CreateSphere(Vector3d.Right * (Fixed64)3);
        scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(sleeping.Body, driver.Body));
        sleeping.Body.Sleep();
        driver.Body.AddLinearImpulse(-Vector3d.Right * (Fixed64)16);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        sleeping.Body.IsSleeping.Should().BeFalse();
        driver.Body.IsSleeping.Should().BeFalse();
    }

    [Fact]
    public void ConstraintSolveOrder_ShouldBeDeterministicAcrossJointRegistrationOrder()
    {
        ConstraintState first = RunConstraintChain(registerForward: true);
        ConstraintState second = RunConstraintChain(registerForward: false);

        second.Should().Be(first);
    }

    [Fact]
    public void MotorTarget_ShouldPullTowardTargetDeterministicallyAndRespectDisabledStrength()
    {
        MotorState disabled = RunMotorScenario(strength: Fixed64.Zero);
        MotorState enabledA = RunMotorScenario(strength: (Fixed64)4);
        MotorState enabledB = RunMotorScenario(strength: (Fixed64)4);

        disabled.AngularErrorAfter.Should().Be(disabled.AngularErrorBefore);
        enabledA.AngularErrorAfter.Should().BeLessThan(enabledA.AngularErrorBefore);
        enabledB.Should().Be(enabledA);
    }

    [Fact]
    public void ConstraintServiceMotorHelpers_ShouldUpdateJointAndRagdollTargets()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> root = scenario.CreateSphere(Vector3d.Zero, isKinematic: true);
        ScenarioBody<LSSphereCollider> child = scenario.CreateSphere(Vector3d.Right * (Fixed64)2, isKinematic: true);
        RagdollRuntime3D ragdoll = scenario.Context.Constraints3D.RegisterRagdoll(CreateTwoLinkRagdoll(root, child));
        Joint3D joint = ragdoll.GetJoint(0);
        var motor = new JointMotor3D(
            FixedQuaternion.Identity,
            (Fixed64)2,
            Fixed64.Half,
            Fixed64.One);

        scenario.Context.Constraints3D.SetRagdollPoseTargets(ragdoll, new[] { motor });
        scenario.Context.Constraints3D.SetJointMotorTarget(joint.Id, FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.FromFraction(1, 5))).Should().BeTrue();

        joint.Motor.AngularDriveStrength.Should().Be((Fixed64)2);
        joint.Motor.TargetLocalRotation.Should().Be(FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.FromFraction(1, 5)).Normalized);

        scenario.Context.Constraints3D.ClearJointMotorTarget(joint.Id).Should().BeTrue();

        joint.Motor.IsEnabled.Should().BeFalse();
    }

    [Fact]
    public void RagdollRuntime_ShouldActivateDynamicAndDeactivateToKinematicDeterministically()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> root = scenario.CreateSphere(Vector3d.Zero, isKinematic: true);
        ScenarioBody<LSSphereCollider> child = scenario.CreateSphere(Vector3d.Right * (Fixed64)2, isKinematic: true);
        RagdollRuntime3D runtime = scenario.Context.Constraints3D.RegisterRagdoll(CreateTwoLinkRagdoll(root, child));

        runtime.ActivateDynamic();

        runtime.IsActive.Should().BeTrue();
        root.Body.IsKinematic.Should().BeFalse();
        child.Body.IsKinematic.Should().BeFalse();
        runtime.GetJoint(0).IsEnabled.Should().BeTrue();

        runtime.DeactivateToKinematic();

        runtime.IsActive.Should().BeFalse();
        root.Body.IsKinematic.Should().BeTrue();
        child.Body.IsKinematic.Should().BeTrue();
        runtime.GetJoint(0).IsEnabled.Should().BeFalse();
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void JointRecordData_ShouldRoundTripAuthoritativeState(GravitasSerializationTransport transport)
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        Joint3D source = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            first.Body,
            second.Body,
            LocalFrame(Vector3d.Right),
            LocalFrame(-Vector3d.Right),
            JointType3D.ConeTwist,
            JointLimit3D.ConeTwist(Fixed64.FromFraction(1, 3), Fixed64.FromFraction(1, 4)),
            new JointMotor3D(FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.FromFraction(1, 5)), (Fixed64)3, Fixed64.Half, Fixed64.One),
            JointCollisionPolicy.Collide));
        source.IsEnabled = false;

        object payload = GravitasSerializationHarness.Serialize(source, transport);

        Joint3D target = scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));
        GravitasSerializationHarness.Populate(target, payload, transport);

        target.IsEnabled.Should().BeFalse();
        target.Type.Should().Be(JointType3D.ConeTwist);
        target.LocalFrameA.Position.Should().Be(Vector3d.Right);
        target.LocalFrameB.Position.Should().Be(-Vector3d.Right);
        target.Limits.Should().Be(source.Limits);
        target.Motor.Should().Be(source.Motor);
        target.CollisionPolicy.Should().Be(JointCollisionPolicy.Collide);
    }

    [Fact]
    public void EnabledDiagnostics_ShouldRecordJointLifecycleAndImpulseEvents()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)3);
        scenario.Context.Diagnostics.Enable(eventCapacity: 16, drawCommandCapacity: 16);

        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));
        scenario.Context.Diagnostics.CaptureJoint(joint, GravitasDiagnosticColor.Cyan);
        Step(scenario.Context, 4);
        scenario.Context.Constraints3D.RemoveJoint(joint.Id);

        ReadOnlySpan<GravitasDiagnosticEvent> events = scenario.Context.Diagnostics.Events;
        events[0].Kind.Should().Be(GravitasDiagnosticEventKind.JointRegistered);
        events.Should().Contain(e => e.Kind == GravitasDiagnosticEventKind.JointImpulse && e.JointId == joint.Id);
        events[^1].Kind.Should().Be(GravitasDiagnosticEventKind.JointRemoved);
        scenario.Context.Diagnostics.DrawCommandCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void ConstraintFilteringAndDisabledDiagnostics_ShouldNotAllocateAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)3);
        scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));
        Step(scenario.Context, 8);
        bool linkedFilterResult = false;

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            () =>
            {
                linkedFilterResult = scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(first.Collider, second.Collider);
                scenario.Context.Simulate();
                scenario.Context.LateSimulate();
            },
            warmupIterations: 8,
            stabilizationIterations: 4,
            measurementIterations: 8);

        linkedFilterResult.Should().BeTrue();
        allocatedBytes.Should().Be(0);
    }

    private static PhysicsScenarioBuilder CreateConstraintScenario()
    {
        PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        scenario.Context.Settings.DiscreteSolverIterations = 8;
        return scenario;
    }

    private static JointDefinition3D CreateBallSocket(
        SolidBody first,
        SolidBody second,
        JointCollisionPolicy collisionPolicy = JointCollisionPolicy.SuppressLinked)
    {
        return new JointDefinition3D(
            first,
            second,
            LocalFrame(Vector3d.Zero),
            LocalFrame(Vector3d.Zero),
            JointType3D.BallSocket,
            JointLimit3D.Unrestricted,
            JointMotor3D.Disabled,
            collisionPolicy);
    }

    private static FixedTransform LocalFrame(Vector3d position) =>
        new(position, FixedQuaternion.Identity, Vector3d.One);

    private static void Step(GravitasWorldContext context, int frames)
    {
        for (int i = 0; i < frames; i++)
        {
            context.Simulate();
            context.LateSimulate();
        }
    }

    private static ConstraintState CaptureConstraintState(SolidBody first, SolidBody second, Joint3D joint)
    {
        Vector3d anchorA = first.Position3d + first.Rotation * joint.LocalFrameA.Position;
        Vector3d anchorB = second.Position3d + second.Rotation * joint.LocalFrameB.Position;
        return new ConstraintState(
            first.Position3d,
            second.Position3d,
            first.LinearVelocity,
            second.LinearVelocity,
            (anchorB - anchorA).MagnitudeSquared,
            joint.LastSolvedRowCount,
            joint.AccumulatedImpulseMagnitude);
    }

    private static ConstraintState RunConstraintChain(bool registerForward)
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero, preventAngularForces: true);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)3, preventAngularForces: true);
        ScenarioBody<LSSphereCollider> third = scenario.CreateSphere(Vector3d.Right * (Fixed64)6, preventAngularForces: true);
        if (registerForward)
        {
            scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));
            scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(second.Body, third.Body));
        }
        else
        {
            scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(second.Body, third.Body));
            scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));
        }

        Step(scenario.Context, 12);
        Joint3D firstJoint = scenario.Context.Constraints3D.GetJoint(1);
        Joint3D secondJoint = scenario.Context.Constraints3D.GetJoint(2);
        return new ConstraintState(
            first.Body.Position3d,
            third.Body.Position3d,
            first.Body.LinearVelocity,
            third.Body.LinearVelocity,
            (third.Body.Position3d - first.Body.Position3d).MagnitudeSquared,
            firstJoint.LastSolvedRowCount + secondJoint.LastSolvedRowCount,
            firstJoint.AccumulatedImpulseMagnitude + secondJoint.AccumulatedImpulseMagnitude);
    }

    private static MotorState RunMotorScenario(Fixed64 strength)
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero, immovable: true);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(
            Vector3d.Right * (Fixed64)2,
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.FromFraction(1, 3)));
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            first.Body,
            second.Body,
            LocalFrame(Vector3d.Zero),
            LocalFrame(Vector3d.Zero),
            JointType3D.BallSocket,
            JointLimit3D.Unrestricted,
            new JointMotor3D(FixedQuaternion.Identity, strength, Fixed64.Half, (Fixed64)2),
            JointCollisionPolicy.SuppressLinked));
        Fixed64 before = FixedQuaternion.Angle(FixedQuaternion.Identity, second.Body.Rotation);

        Step(scenario.Context, 16);

        return new MotorState(
            before,
            FixedQuaternion.Angle(FixedQuaternion.Identity, second.Body.Rotation),
            second.Body.AngularVelocity,
            joint.AccumulatedImpulseMagnitude);
    }

    private static RagdollDefinition3D CreateTwoLinkRagdoll(
        ScenarioBody<LSSphereCollider> root,
        ScenarioBody<LSSphereCollider> child)
    {
        return new RagdollDefinition3D(
            new[]
            {
                new RagdollLinkDefinition3D(0, root.Body, root.Collider),
                new RagdollLinkDefinition3D(1, child.Body, child.Collider)
            },
            new[]
            {
                new RagdollJointDefinition3D(0, 1, JointType3D.BallSocket, LocalFrame(Vector3d.Zero), LocalFrame(Vector3d.Zero))
            });
    }

    private readonly record struct ConstraintState(
        Vector3d FirstPosition,
        Vector3d SecondPosition,
        Vector3d FirstVelocity,
        Vector3d SecondVelocity,
        Fixed64 AnchorDistanceSquared,
        int JointSolvedRowCount,
        Fixed64 AccumulatedImpulseMagnitude);

    private readonly record struct MotorState(
        Fixed64 AngularErrorBefore,
        Fixed64 AngularErrorAfter,
        Vector3d AngularVelocity,
        Fixed64 AccumulatedImpulseMagnitude);
}
