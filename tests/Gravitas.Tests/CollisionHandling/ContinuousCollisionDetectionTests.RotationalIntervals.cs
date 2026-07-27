using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed partial class ContinuousCollisionDetectionTests
{
    [Fact]
    public void ContinuousMode_Kinematic3DRotation_ShouldCatchShiftedContactBetweenEndpointAndMidpointSamples()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSCuboidCollider> blade = CreateKinematicRotationalCcdBlade(scenario);
        FixedQuaternion contactRotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Up,
            FixedMath.DegToRad(Fixed64.FromFraction(5, 2)));
        Vector3d targetPosition = contactRotation
            * new Vector3d(Fixed64.FromFraction(16, 5), Fixed64.Zero, Fixed64.Zero);
        ScenarioBody<LSSphereCollider> target = scenario.CreateBody(
            new LSSphereCollider { Radius = Fixed64.FromFraction(401, 2000) },
            targetPosition,
            FixedQuaternion.Identity,
            immovable: true);
        FixedQuaternion startRotation = PhysicsScenarioBuilder.Yaw(-5);
        FixedQuaternion targetRotation = PhysicsScenarioBuilder.Yaw(5);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        blade.Body.SetRotation(startRotation);
        blade.Body.Agent.Transform.LocalRotation = targetRotation;

        AssertRotationalWitness(blade, target, startRotation, contactRotation, targetRotation);
        blade.Body.SetRotation(startRotation);

        scenario.Context.LateSimulate();

        blade.Body.Rotation.Should().NotBe(targetRotation);
        blade.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
    }

    [Fact]
    public void ContinuousMode_OffsetSphereRotation_ShouldUseBodyPivotRadius()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        var sourceCollider = new LSSphereCollider
        {
            Radius = Fixed64.FromFraction(1, 8),
            LocalOffset = new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero)
        };
        ScenarioBody<LSSphereCollider> source = scenario.CreateBody(
            sourceCollider,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            isKinematic: true);
        _ = scenario.CreateSphere(
            new Vector3d(Fixed64.FromFraction(16, 5), Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        FixedQuaternion startRotation = PhysicsScenarioBuilder.Yaw(-5);
        FixedQuaternion targetRotation = PhysicsScenarioBuilder.Yaw(5);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Body.SetRotation(startRotation);
        source.Body.Agent.Transform.LocalRotation = targetRotation;

        scenario.Context.LateSimulate();

        source.Body.Rotation.Should().NotBe(targetRotation);
        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
    }

    [Fact]
    public void ContinuousMode_UnrepresentablePivotRadius_ShouldUseBoundedRegisteredCandidateFallback()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        var sourceCollider = new LSSphereCollider { Radius = Fixed64.One };
        ScenarioBody<LSSphereCollider> source = scenario.CreateBody(
            sourceCollider,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            isKinematic: true);
        _ = scenario.CreateStaticSphere(Vector3d.Zero);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        sourceCollider.LocalOffset = new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero);
        sourceCollider.RebuildRuntimeShapeOnly();
        source.Body.ResolveContinuousCollisionProxyRadius().Should().Be(Fixed64.MaxValue);

        source.Body.GatherRotationalContinuousCollisionCandidates(
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.MaxValue).Should().Be(1);
    }

    private static void AssertRotationalWitness(
        ScenarioBody<LSCuboidCollider> source,
        ScenarioBody<LSSphereCollider> target,
        FixedQuaternion startRotation,
        FixedQuaternion contactRotation,
        FixedQuaternion endRotation)
    {
        IsCollidingAtRotation(source, target, startRotation).Should().BeFalse();
        IsCollidingAtRotation(source, target, contactRotation).Should().BeTrue();
        IsCollidingAtRotation(source, target, endRotation).Should().BeFalse();
    }

    private static bool IsCollidingAtRotation(
        ScenarioBody<LSCuboidCollider> source,
        ScenarioBody<LSSphereCollider> target,
        FixedQuaternion rotation)
    {
        source.Body.SetRotation(rotation);
        CollisionPair pair = new(source.Collider, target.Collider);
        return CollisionDetection.DoCollisionCheck(pair);
    }
}
