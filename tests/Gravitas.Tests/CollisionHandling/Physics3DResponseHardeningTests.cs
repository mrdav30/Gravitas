using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Constraints;
using Gravitas.Diagnostics;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class Physics3DResponseHardeningTests
{
    [Fact]
    public void Simulate_WithSeparateSleepingJointAndAwakeContact_ShouldSolveOnlyContactIsland()
    {
        using PhysicsScenarioBuilder scenario = CreateScenario();
        ScenarioBody<LSSphereCollider> sleepingA = scenario.CreateSphere(new Vector3d((Fixed64)(-8), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> sleepingB = scenario.CreateSphere(new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero));
        Joint3D sleepingJoint = scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(sleepingA.Body, sleepingB.Body));
        sleepingA.Body.Sleep();
        sleepingB.Body.Sleep();
        ScenarioBody<LSSphereCollider> contactA = scenario.CreateSphere(Vector3d.Zero, preventAngularForces: true);
        ScenarioBody<LSSphereCollider> contactB = scenario.CreateSphere(Vector3d.Right * Fixed64.FromFraction(3, 4), preventAngularForces: true);
        Vector3d contactAStart = contactA.Body.Position3d;
        Vector3d contactBStart = contactB.Body.Position3d;

        Step(scenario.Context);

        sleepingA.Body.IsSleeping.Should().BeTrue();
        sleepingB.Body.IsSleeping.Should().BeTrue();
        sleepingJoint.LastSolvedRowCount.Should().Be(0);
        contactA.Body.Position3d.Should().NotBe(contactAStart);
        contactB.Body.Position3d.Should().NotBe(contactBStart);
    }

    [Fact]
    public void Simulate_WithSparseJointIdsAndMovableBodyA_ShouldSolveSurvivingAnchoredJoint()
    {
        using PhysicsScenarioBuilder scenario = CreateScenario();
        ScenarioBody<LSSphereCollider> removedA = scenario.CreateSphere(new Vector3d((Fixed64)(-8), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> removedB = scenario.CreateSphere(new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero));
        Joint3D removed = scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(removedA.Body, removedB.Body));
        ScenarioBody<LSSphereCollider> movable = scenario.CreateSphere(Vector3d.Zero, preventAngularForces: true);
        ScenarioBody<LSSphereCollider> anchor = scenario.CreateSphere(Vector3d.Right * (Fixed64)3, immovable: true);
        Joint3D surviving = scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(movable.Body, anchor.Body));
        Vector3d movableStart = movable.Body.Position3d;
        Vector3d anchorStart = anchor.Body.Position3d;

        scenario.Context.Constraints3D.RemoveJoint(removed.Id).Should().BeTrue();
        Step(scenario.Context, 2);

        scenario.Context.Constraints3D.GetJoint(surviving.Id).Should().BeSameAs(surviving);
        movable.Body.Position3d.Should().NotBe(movableStart);
        anchor.Body.Position3d.Should().Be(anchorStart);
        surviving.LastSolvedRowCount.Should().BeGreaterThan(0);
        surviving.AccumulatedImpulseMagnitude.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void Simulate_WithZeroMassContactAndUnrelatedMovableJoint_ShouldSkipRootlessContact()
    {
        using PhysicsScenarioBuilder scenario = CreateScenario();
        ScenarioBody<LSSphereCollider> inertA = scenario.CreateSphere(Vector3d.Zero, mass: Fixed64.Zero);
        ScenarioBody<LSSphereCollider> inertB = scenario.CreateSphere(
            Vector3d.Right * Fixed64.FromFraction(3, 4),
            mass: Fixed64.Zero);
        ScenarioBody<LSSphereCollider> movable = scenario.CreateSphere(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> anchor = scenario.CreateSphere(new Vector3d((Fixed64)7, Fixed64.Zero, Fixed64.Zero), immovable: true);
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(movable.Body, anchor.Body));
        Vector3d inertAStart = inertA.Body.Position3d;
        Vector3d inertBStart = inertB.Body.Position3d;
        int inertContactEnters = 0;
        inertA.Collider.OnContactEnter += other =>
        {
            if (object.ReferenceEquals(other, inertB.Body))
                inertContactEnters++;
        };

        Step(scenario.Context);

        inertContactEnters.Should().Be(1);
        inertA.Body.Position3d.Should().Be(inertAStart);
        inertB.Body.Position3d.Should().Be(inertBStart);
        joint.LastSolvedRowCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void Simulate_WithCollidingJoint_ShouldBeDeterministicAcrossEndpointOrder()
    {
        CollidingJointState forward = RunCollidingJointScenario(reverseEndpoints: false);
        CollidingJointState reversed = RunCollidingJointScenario(reverseEndpoints: true);

        forward.ContactCount.Should().BeGreaterThan(0);
        forward.FirstJointSolvedRowCount.Should().BeGreaterThan(0);
        forward.SecondJointSolvedRowCount.Should().BeGreaterThan(0);
        forward.ThirdJointSolvedRowCount.Should().BeGreaterThan(0);
        forward.FirstJointImpulse.Should().BeGreaterThan(Fixed64.Zero);
        forward.SecondJointImpulse.Should().BeGreaterThan(Fixed64.Zero);
        forward.ThirdJointImpulse.Should().BeGreaterThan(Fixed64.Zero);
        forward.DiagnosticOrder.ResponseSequence.Should().BeGreaterThanOrEqualTo(0);
        forward.DiagnosticOrder.ResponseSequence.Should().BeLessThan(forward.DiagnosticOrder.FirstJointSequence);
        forward.DiagnosticOrder.FirstJointSequence.Should().BeLessThan(forward.DiagnosticOrder.SecondJointSequence);
        forward.DiagnosticOrder.SecondJointSequence.Should().BeLessThan(forward.DiagnosticOrder.ThirdJointSequence);
        reversed.Should().Be(forward);
    }

    private static PhysicsScenarioBuilder CreateScenario()
    {
        PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        return scenario;
    }

    private static JointDefinition3D CreateBallSocket(
        SolidBody first,
        SolidBody second,
        JointCollisionPolicy collisionPolicy = JointCollisionPolicy.SuppressLinked,
        Vector3d localFrameA = default,
        Vector3d localFrameB = default) =>
        new(
            first,
            second,
            new FixedTransform(localFrameA, FixedQuaternion.Identity, Vector3d.One),
            new FixedTransform(localFrameB, FixedQuaternion.Identity, Vector3d.One),
            JointType3D.BallSocket,
            JointLimit3D.Unrestricted,
            JointMotor3D.Disabled,
            collisionPolicy);

    private static CollidingJointState RunCollidingJointScenario(bool reverseEndpoints)
    {
        using PhysicsScenarioBuilder scenario = CreateScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(
            Vector3d.Zero,
            preventAngularForces: true);
        ScenarioBody<LSSphereCollider> intermediate = scenario.CreateSphere(
            Vector3d.Right * (Fixed64)4,
            preventAngularForces: true);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(
            Vector3d.Right * Fixed64.FromFraction(3, 4),
            preventAngularForces: true);
        Step(scenario.Context);
        JointDefinition3D firstDefinition = reverseEndpoints
            ? CreateBallSocket(second.Body, first.Body, JointCollisionPolicy.Collide)
            : CreateBallSocket(first.Body, second.Body, JointCollisionPolicy.Collide);
        Vector3d offset = Vector3d.Right * Fixed64.FromFraction(1, 4);
        JointDefinition3D secondDefinition = reverseEndpoints
            ? CreateBallSocket(second.Body, first.Body, JointCollisionPolicy.Collide, Vector3d.Zero, offset)
            : CreateBallSocket(first.Body, second.Body, JointCollisionPolicy.Collide, offset, Vector3d.Zero);
        Joint3D firstJoint = scenario.Context.Constraints3D.RegisterJoint(firstDefinition);
        Joint3D secondJoint = scenario.Context.Constraints3D.RegisterJoint(secondDefinition);
        Joint3D thirdJoint = scenario.Context.Constraints3D.RegisterJoint(
            CreateBallSocket(intermediate.Body, second.Body));
        scenario.Context.Diagnostics.Enable(eventCapacity: 128, drawCommandCapacity: 0);

        scenario.Context.Simulate();
        first.Body.AddLinearImpulse(Vector3d.Right * Fixed64.FromFraction(1, 8) * first.Body.Mass);
        scenario.Context.LateSimulate();

        first.Collider.TryGetCollisionPair(second.Collider.Id, out CollisionHandling.CollisionPair? pair).Should().BeTrue();
        return new CollidingJointState(
            first.Body.Position3d,
            intermediate.Body.Position3d,
            second.Body.Position3d,
            first.Body.LinearVelocity,
            intermediate.Body.LinearVelocity,
            second.Body.LinearVelocity,
            pair!.Manifold.Count,
            firstJoint.LastSolvedRowCount,
            secondJoint.LastSolvedRowCount,
            thirdJoint.LastSolvedRowCount,
            firstJoint.AccumulatedImpulseMagnitude,
            secondJoint.AccumulatedImpulseMagnitude,
            thirdJoint.AccumulatedImpulseMagnitude,
            CaptureConstraintDiagnosticOrder(
                scenario.Context.Diagnostics.Events,
                firstJoint.Id,
                secondJoint.Id,
                thirdJoint.Id));
    }

    private static ConstraintDiagnosticOrder CaptureConstraintDiagnosticOrder(
        ReadOnlySpan<GravitasDiagnosticEvent> events,
        int firstJointId,
        int secondJointId,
        int thirdJointId)
    {
        int responseSequence = -1;
        int firstJointSequence = -1;
        int secondJointSequence = -1;
        int thirdJointSequence = -1;
        for (int i = 0; i < events.Length; i++)
        {
            GravitasDiagnosticEvent diagnosticEvent = events[i];
            if (responseSequence < 0 && diagnosticEvent.Kind == GravitasDiagnosticEventKind.ResponseImpulse)
                responseSequence = diagnosticEvent.Sequence;
            else if (diagnosticEvent.Kind == GravitasDiagnosticEventKind.JointImpulse)
            {
                if (firstJointSequence < 0 && diagnosticEvent.JointId == firstJointId)
                    firstJointSequence = diagnosticEvent.Sequence;
                else if (secondJointSequence < 0 && diagnosticEvent.JointId == secondJointId)
                    secondJointSequence = diagnosticEvent.Sequence;
                else if (thirdJointSequence < 0 && diagnosticEvent.JointId == thirdJointId)
                    thirdJointSequence = diagnosticEvent.Sequence;
            }
        }

        return new ConstraintDiagnosticOrder(
            responseSequence,
            firstJointSequence,
            secondJointSequence,
            thirdJointSequence);
    }

    private static void Step(GravitasWorldContext context, int count = 1)
    {
        for (int i = 0; i < count; i++)
        {
            context.Simulate();
            context.LateSimulate();
        }
    }

    private readonly record struct CollidingJointState(
        Vector3d FirstPosition,
        Vector3d IntermediatePosition,
        Vector3d SecondPosition,
        Vector3d FirstVelocity,
        Vector3d IntermediateVelocity,
        Vector3d SecondVelocity,
        int ContactCount,
        int FirstJointSolvedRowCount,
        int SecondJointSolvedRowCount,
        int ThirdJointSolvedRowCount,
        Fixed64 FirstJointImpulse,
        Fixed64 SecondJointImpulse,
        Fixed64 ThirdJointImpulse,
        ConstraintDiagnosticOrder DiagnosticOrder);

    private readonly record struct ConstraintDiagnosticOrder(
        int ResponseSequence,
        int FirstJointSequence,
        int SecondJointSequence,
        int ThirdJointSequence);
}
