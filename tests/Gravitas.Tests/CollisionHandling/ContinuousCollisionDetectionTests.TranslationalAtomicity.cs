using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed partial class ContinuousCollisionDetectionTests
{
    [Fact]
    public void TranslationalDynamicResponse_WhenTargetTrajectoryIsFull_ShouldRejectAtomically3D()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Settings.ContinuousCollisionMaxToiIterations = 1;
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(-Vector3d.Right * (Fixed64)2);
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(Vector3d.Zero);
        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        source.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)4);
        target.Body.ApplyContinuousCollisionHandoff(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Right,
                Vector3d.Zero,
                scenario.Context.DeltaTime * Fixed64.FromFraction(3, 4))
            .Should()
            .BeTrue();
        scenario.Context.Diagnostics.Enable(eventCapacity: 8, drawCommandCapacity: 0);

        Vector3d sourcePosition = source.Body.Position3d;
        Vector3d sourceVelocity = source.Body.LinearVelocity;
        Vector3d targetPosition = target.Body.Position3d;
        FixedQuaternion targetRotation = target.Body.Rotation;
        Vector3d targetVelocity = target.Body.LinearVelocity;
        Vector3d targetAngularVelocity = target.Body.AngularVelocity;
        bool sourceSleeping = source.Body.IsSleeping;
        bool targetSleeping = target.Body.IsSleeping;
        int sourceTrajectoryCount = source.Body.ContinuousCollisionTrajectoryCount;
        int targetTrajectoryCount = target.Body.ContinuousCollisionTrajectoryCount;
        int diagnosticCount = scenario.Context.Diagnostics.EventCount;
        var replayHash = scenario.Context.ComputeReplayHash(
            GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        bool applied = InvokeTranslationalDynamicResponse3D(
            source.Body,
            target.Body,
            -Vector3d.Right,
            -Vector3d.Right,
            scenario.Context.DeltaTime * Fixed64.Half,
            scenario.Context.DeltaTime * Fixed64.Half);

        applied.Should().BeFalse();
        source.Body.Position3d.Should().Be(sourcePosition);
        source.Body.LinearVelocity.Should().Be(sourceVelocity);
        target.Body.Position3d.Should().Be(targetPosition);
        target.Body.Rotation.Should().Be(targetRotation);
        target.Body.LinearVelocity.Should().Be(targetVelocity);
        target.Body.AngularVelocity.Should().Be(targetAngularVelocity);
        source.Body.IsSleeping.Should().Be(sourceSleeping);
        target.Body.IsSleeping.Should().Be(targetSleeping);
        source.Body.ContinuousCollisionTrajectoryCount.Should().Be(sourceTrajectoryCount);
        target.Body.ContinuousCollisionTrajectoryCount.Should().Be(targetTrajectoryCount);
        scenario.Context.Diagnostics.EventCount.Should().Be(diagnosticCount);
        scenario.Context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should()
            .Be(replayHash);
    }

    private static bool InvokeTranslationalDynamicResponse3D(
        SolidBody source,
        SolidBody target,
        Vector3d normal,
        Vector3d sourcePositionAtImpact,
        Fixed64 hitElapsedTime,
        Fixed64 remainingTime)
    {
        Fixed64 frameFraction = FixedMath.Clamp01(hitElapsedTime / source.Context.DeltaTime);
        return source.TryApplyContinuousCollisionDynamicResponse(
            target,
            normal,
            sourcePositionAtImpact,
            target.SampleContinuousCollisionPosition(frameFraction),
            hitElapsedTime,
            remainingTime);
    }
}
