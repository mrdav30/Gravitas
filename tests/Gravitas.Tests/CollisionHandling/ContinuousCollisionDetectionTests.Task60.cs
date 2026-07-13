using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed partial class ContinuousCollisionDetectionTests
{
    [Fact]
    public void DiscreteMode_WithKinematicHostRotation_ShouldApplyAuthoredRotationWithoutCcd()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSCuboidCollider> blade = CreateKinematicRotationalCcdBlade(scenario);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;
        FixedQuaternion hostRotation = PhysicsScenarioBuilder.Yaw(90);
        blade.Body.Agent.Transform.Rotation = hostRotation;

        scenario.Context.LateSimulate();

        blade.Body.Rotation.Should().Be(hostRotation);
        blade.Body.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Fact]
    public void ContinuousMode_WithUnsupportedStaticRotationalCandidate_ShouldPreserveAuthoredAngularMotion()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        var target = new UnsupportedTestCollider3D();
        scenario.InitializeStaticCollider(target, Vector3d.Right * (Fixed64)2);
        ScenarioBody<LSCuboidCollider> blade = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.One, Fixed64.FromFraction(1, 5))
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(blade.Body);
        Fixed64 angularVelocity = FixedMath.DegToRad((Fixed64)90);
        FixedQuaternion expectedRotation = new(
            Fixed64.Zero,
            angularVelocity * Fixed64.Half,
            Fixed64.Zero,
            Fixed64.One);
        expectedRotation = expectedRotation.Normalized;
        var candidates = new SwiftList<Physics3DHit>();

        ColliderSettings.GetCollisionType(blade.Collider.Shape, target.Shape).Should().Be(CollisionType.None);
        scenario.Context.Query3D.OverlapSphereAgainstStaticAll(
            blade.Body.Position3d,
            blade.Collider.ScaledRadius,
            PhysicsLayerMask.All,
            candidates,
            blade.Collider,
            includeTriggers: false).Should().Be(1);
        candidates[0].Collider.Should().BeSameAs(target);

        blade.Body.AddAngularImpulse(Vector3d.Up * (angularVelocity / blade.Body.EffectiveInverseInertiaTensor.M22));
        scenario.Context.LateSimulate();

        blade.Body.Rotation.Should().Be(expectedRotation);
        blade.Body.AngularVelocity.Should().Be(Vector3d.Up * angularVelocity);
        blade.Body.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Fact]
    public void ContinuousMode_WithColliderFilterRejectedStaticRotationalCandidate_ShouldPreserveAuthoredKinematicRotation()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        LSSphereCollider target = scenario.CreateStaticSphere(
            new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.FromFraction(-5, 4)));
        ScenarioBody<LSCuboidCollider> blade = CreateKinematicRotationalCcdBlade(scenario);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        blade.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(target.Layer);
        FixedQuaternion hostRotation = PhysicsScenarioBuilder.Yaw(90);
        var candidates = new SwiftList<Physics3DHit>();

        ColliderSettings.GetCollisionType(blade.Collider.Shape, target.Shape)
            .Should().Be(CollisionType.Cuboid_Sphere);
        scenario.Context.Physics.RequireCollisionPair(blade.Collider, target).Should().BeFalse();
        scenario.Context.Query3D.OverlapSphereAgainstStaticAll(
            blade.Body.Position3d,
            blade.Collider.ScaledRadius,
            PhysicsLayerMask.All,
            candidates,
            blade.Collider,
            includeTriggers: false).Should().Be(1);
        candidates[0].Collider.Should().BeSameAs(target);

        blade.Body.Agent.Transform.Rotation = hostRotation;
        scenario.Context.LateSimulate();

        blade.Body.Rotation.Should().Be(hostRotation);
        blade.Body.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }
}
