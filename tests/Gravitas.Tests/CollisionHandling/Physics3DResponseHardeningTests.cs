using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Constraints;
using Gravitas.Tests.Support;
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

    private static PhysicsScenarioBuilder CreateScenario()
    {
        PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        return scenario;
    }

    private static JointDefinition3D CreateBallSocket(SolidBody first, SolidBody second) =>
        new(
            first,
            second,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One),
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One),
            JointType3D.BallSocket,
            JointLimit3D.Unrestricted,
            JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked);

    private static void Step(GravitasWorldContext context, int count = 1)
    {
        for (int i = 0; i < count; i++)
        {
            context.Simulate();
            context.LateSimulate();
        }
    }
}
