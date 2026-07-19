using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed partial class ContinuousCollisionDetectionTests
{
    [Fact]
    public void CandidateRefreshAdmission_ShouldAcceptEveryRegisteredBody3D()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(-Vector3d.Right);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right);

        scenario.Context.Physics.CanAdmitContinuousCollisionCandidateRefresh(first.Body)
            .Should()
            .BeTrue();
        scenario.Context.Physics.CanAdmitContinuousCollisionCandidateRefresh(
                first.Body,
                second.Body)
            .Should()
            .BeTrue();
        scenario.Context.Physics.TryReserveContinuousCollisionCandidateRefresh(
                first.Body,
                second.Body)
            .Should()
            .BeTrue();
    }

    [Fact]
    public void KinematicHandoff_NonClosingPair_ShouldRejectWithoutMutation3D()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(
            -Vector3d.Right * (Fixed64)2,
            isKinematic: true);
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(Vector3d.Zero);
        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        var before = scenario.Context.ComputeReplayHash(
            GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        InvokeKinematicHandoff3D(
                source.Body,
                target.Body,
                Vector3d.Right,
                -Vector3d.Right,
                Fixed64.Half,
                Fixed64.One)
            .Should()
            .BeFalse();

        scenario.Context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should()
            .Be(before);
    }

    [Fact]
    public void KinematicUnsupportedSweep_ShouldUseConservativeDynamicFallback3D()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        ScenarioBody<UnsupportedTestCollider3D> source = scenario.CreateBody(
            new UnsupportedTestCollider3D(),
            -Vector3d.Right * (Fixed64)3,
            FixedQuaternion.Identity,
            isKinematic: true);
        ScenarioBody<LSCuboidCollider> target = scenario.CreateCuboid(Vector3d.Zero);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Body.Agent.Transform.LocalPosition = Vector3d.Right * (Fixed64)3;

        scenario.Context.LateSimulate();

        source.Body.Position3d.Should().Be(Vector3d.Right * (Fixed64)3);
        target.Body.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        source.Body.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void MovingPairOwnership_StationaryKinematicBody_ShouldRemainWithMovingSource3D()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(
            Vector3d.Zero,
            isKinematic: true);
        body.Body.ContinuousCollisionMode = ContinuousCollisionMode.Auto;
        scenario.Context.AdvanceLateSimulateToken();

        body.Body.ShouldOwnContinuousCollisionMovingPair(otherHasRotationalMotion: true)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void ContinuousCollisionMotionBound_UnrepresentableRotationalExcursion_ShouldFail3D()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(
            Vector3d.Zero,
            isKinematic: true);
        body.Body.ContinuousCollisionMode = ContinuousCollisionMode.Auto;
        body.Body.Agent.Transform.LocalRotation = PhysicsScenarioBuilder.Yaw(180);
        scenario.Context.AdvanceLateSimulateToken();
        body.Body.EnsureContinuousCollisionFramePrepared(scenario.Context.LateSimulateToken);

        body.Body.TryResolveContinuousCollisionMotionBound(
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.MaxValue,
                out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void TranslationalDynamicResponse_ExhaustedSourceTrajectory_ShouldRejectWithoutMutation3D()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Settings.ContinuousCollisionMaxToiIterations = 1;
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(-Vector3d.Right * (Fixed64)2);
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(Vector3d.Zero);
        source.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)4);
        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        source.Body.ApplyContinuousCollisionHandoff(
                -Vector3d.Right,
                FixedQuaternion.Identity,
                Vector3d.Right * (Fixed64)4,
                Vector3d.Zero,
                Fixed64.FromFraction(3, 4))
            .Should()
            .BeTrue();
        var before = scenario.Context.ComputeReplayHash(
            GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        InvokeTranslationalDynamicResponse3D(
                source.Body,
                target.Body,
                -Vector3d.Right,
                Vector3d.Zero,
                scenario.Context.DeltaTime * Fixed64.Half,
                scenario.Context.DeltaTime * Fixed64.Half)
            .Should()
            .BeFalse();

        scenario.Context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should()
            .Be(before);
    }

    [Fact]
    public void ContinuousCollisionHandoff_UnrepresentableVelocity_ShouldRejectWithoutMutation3D()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.MaxSpeed = Fixed64.MaxValue;
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * Fixed64.MaxValue);
        var before = scenario.Context.ComputeReplayHash(
            GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        body.Body.ApplyContinuousCollisionHandoff(
                Vector3d.Zero,
                Vector3d.Right,
                scenario.Context.DeltaTime)
            .Should()
            .BeFalse();

        scenario.Context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should()
            .Be(before);
    }

    [Fact]
    public void TranslationalDynamicResponse_InvalidNormal_ShouldRejectWithoutMutation3D()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(-Vector3d.Right * (Fixed64)2);
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(Vector3d.Zero);
        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        source.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)4);
        var before = scenario.Context.ComputeReplayHash(
            GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        InvokeTranslationalDynamicResponse3D(
                source.Body,
                target.Body,
                Vector3d.Zero,
                -Vector3d.Right,
                scenario.Context.DeltaTime * Fixed64.Half,
                scenario.Context.DeltaTime * Fixed64.Half)
            .Should()
            .BeFalse();

        scenario.Context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should()
            .Be(before);
    }

    [Fact]
    public void TranslationalDynamicResponse_SeparatingPair_ShouldRejectWithoutMutation3D()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(-Vector3d.Right * (Fixed64)2);
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(Vector3d.Zero);
        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        source.Body.ApplyCollisionLinearVelocityDelta(-Vector3d.Right * (Fixed64)4);
        var before = scenario.Context.ComputeReplayHash(
            GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        InvokeTranslationalDynamicResponse3D(
                source.Body,
                target.Body,
                -Vector3d.Right,
                -Vector3d.Right,
                scenario.Context.DeltaTime * Fixed64.Half,
                scenario.Context.DeltaTime * Fixed64.Half)
            .Should()
            .BeFalse();

        scenario.Context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should()
            .Be(before);
    }

    [Fact]
    public void TranslationalDynamicResponse_UnrepresentableRelativeVelocity_ShouldRejectWithoutMutation3D()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.MaxSpeed = Fixed64.MaxValue;
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(-Vector3d.Right * (Fixed64)2);
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(Vector3d.Zero);
        source.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * Fixed64.MaxValue);
        target.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Left * Fixed64.MaxValue);
        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        var before = scenario.Context.ComputeReplayHash(
            GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        InvokeTranslationalDynamicResponse3D(
                source.Body,
                target.Body,
                -Vector3d.Right,
                -Vector3d.Right,
                scenario.Context.DeltaTime * Fixed64.Half,
                scenario.Context.DeltaTime * Fixed64.Half)
            .Should()
            .BeFalse();

        scenario.Context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should()
            .Be(before);
    }

    [Fact]
    public void TranslationalDynamicResponse_UnrepresentableRestitutionSpeed_ShouldRejectWithoutMutation3D()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.MaxSpeed = Fixed64.MaxValue;
        scenario.Context.Settings.RestitutionVelocityThreshold = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(-Vector3d.Right * (Fixed64)2);
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(Vector3d.Zero);
        source.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        target.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        source.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * Fixed64.MaxValue);
        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        var before = scenario.Context.ComputeReplayHash(
            GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        InvokeTranslationalDynamicResponse3D(
                source.Body,
                target.Body,
                -Vector3d.Right,
                -Vector3d.Right,
                scenario.Context.DeltaTime * Fixed64.Half,
                scenario.Context.DeltaTime * Fixed64.Half)
            .Should()
            .BeFalse();

        scenario.Context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should()
            .Be(before);
    }

    private static bool InvokeKinematicHandoff3D(
        SolidBody source,
        SolidBody target,
        Vector3d displacement,
        Vector3d normal,
        Fixed64 hitDistance,
        Fixed64 sourceLength) =>
        source.ApplyKinematicContinuousCollisionHandoff(
            target,
            displacement,
            normal,
            hitDistance,
            sourceLength);
}
