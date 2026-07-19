using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed partial class ContinuousCollision2DTests
{
    [Fact]
    public void KinematicHandoff_NonClosingPair_ShouldRejectWithoutMutation2D()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            -Vector2d.Right * (Fixed64)2,
            immovable: false,
            isKinematic: true);
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();
        var before = context.ComputeReplayHash(
            GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        InvokeKinematicHandoff2D(
                source,
                target,
                Vector2d.Right,
                -Vector2d.Right,
                Fixed64.Half,
                Fixed64.One)
            .Should()
            .BeFalse();

        context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should()
            .Be(before);
    }

    [Fact]
    public void MovingPairOwnership_StationaryKinematicBody_ShouldRemainWithMovingSource2D()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D body = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false,
            isKinematic: true);
        body.ContinuousCollisionMode = ContinuousCollisionMode.Auto;
        context.AdvanceLateSimulateToken();

        body.ShouldOwnContinuousCollisionMovingPair(otherHasRotationalMotion: true)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void ContinuousCollisionMotionBound_UnrepresentableRotationalExcursion_ShouldFail2D()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D body = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false,
            isKinematic: true);
        body.ContinuousCollisionMode = ContinuousCollisionMode.Auto;
        body.Agent.Transform.LocalRotationXZRadians = Fixed64.Pi;
        context.AdvanceLateSimulateToken();
        body.EnsureContinuousCollisionFramePrepared(context.LateSimulateToken);

        body.TryResolveContinuousCollisionMotionBound(
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.MaxValue,
                out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void TranslationalDynamicResponse_ExhaustedSourceTrajectory_ShouldRejectWithoutMutation2D()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        context.Settings.ContinuousCollisionMaxToiIterations = 1;
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            -Vector2d.Right * (Fixed64)2,
            immovable: false);
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        source.ApplyCollisionLinearVelocityDelta(Vector2d.Right * (Fixed64)4);
        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();
        source.ApplyContinuousCollisionHandoffState(
                -Vector2d.Right,
                Fixed64.Zero,
                Vector2d.Right * (Fixed64)4,
                Fixed64.Zero,
                Fixed64.FromFraction(3, 4))
            .Should()
            .BeTrue();
        var before = context.ComputeReplayHash(
            GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        InvokeTranslationalDynamicResponse2D(
                source,
                target,
                -Vector2d.Right,
                Vector2d.Zero,
                context.DeltaTime * Fixed64.Half,
                context.DeltaTime * Fixed64.Half)
            .Should()
            .BeFalse();

        context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should()
            .Be(before);
    }

    [Fact]
    public void ContinuousCollisionHandoff_UnrepresentableVelocity_ShouldRejectWithoutMutation2D()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        context.Environment.MaxSpeed = Fixed64.MaxValue;
        SolidBody2D body = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        body.ApplyCollisionLinearVelocityDelta(Vector2d.Right * Fixed64.MaxValue);
        var before = context.ComputeReplayHash(
            GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        body.ApplyContinuousCollisionHandoff(
                Vector2d.Zero,
                Vector2d.Right,
                context.DeltaTime)
            .Should()
            .BeFalse();

        context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should()
            .Be(before);
    }

    [Fact]
    public void TranslationalDynamicResponse_InvalidNormal_ShouldRejectWithoutMutation2D()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            -Vector2d.Right * (Fixed64)2,
            immovable: false);
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();
        source.ApplyCollisionLinearVelocityDelta(Vector2d.Right * (Fixed64)4);
        var before = context.ComputeReplayHash(
            GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        InvokeTranslationalDynamicResponse2D(
                source,
                target,
                Vector2d.Zero,
                -Vector2d.Right,
                context.DeltaTime * Fixed64.Half,
                context.DeltaTime * Fixed64.Half)
            .Should()
            .BeFalse();

        context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should()
            .Be(before);
    }

    [Fact]
    public void TranslationalDynamicResponse_SeparatingPair_ShouldRejectWithoutMutation2D()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            -Vector2d.Right * (Fixed64)2,
            immovable: false);
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();
        source.ApplyCollisionLinearVelocityDelta(-Vector2d.Right * (Fixed64)4);
        var before = context.ComputeReplayHash(
            GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        InvokeTranslationalDynamicResponse2D(
                source,
                target,
                -Vector2d.Right,
                -Vector2d.Right,
                context.DeltaTime * Fixed64.Half,
                context.DeltaTime * Fixed64.Half)
            .Should()
            .BeFalse();

        context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should()
            .Be(before);
    }

    [Fact]
    public void TranslationalDynamicResponse_UnrepresentableRelativeVelocity_ShouldRejectWithoutMutation2D()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            -Vector2d.Right * (Fixed64)2,
            immovable: false);
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        source.ApplyCollisionLinearVelocityDelta(Vector2d.Right * Fixed64.MaxValue);
        target.ApplyCollisionLinearVelocityDelta(Vector2d.Left * Fixed64.MaxValue);
        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();
        var before = context.ComputeReplayHash(
            GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        InvokeTranslationalDynamicResponse2D(
                source,
                target,
                -Vector2d.Right,
                -Vector2d.Right,
                context.DeltaTime * Fixed64.Half,
                context.DeltaTime * Fixed64.Half)
            .Should()
            .BeFalse();

        context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should()
            .Be(before);
    }

    [Fact]
    public void TranslationalDynamicResponse_UnrepresentableRestitutionSpeed_ShouldRejectWithoutMutation2D()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        context.Environment.MaxSpeed = Fixed64.MaxValue;
        context.Settings.RestitutionVelocityThreshold = Fixed64.Zero;
        SolidBody2D source = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            -Vector2d.Right * (Fixed64)2,
            immovable: false);
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            Vector2d.Zero,
            immovable: false);
        source.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        target.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        source.ApplyCollisionLinearVelocityDelta(Vector2d.Right * Fixed64.MaxValue);
        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();
        var before = context.ComputeReplayHash(
            GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        InvokeTranslationalDynamicResponse2D(
                source,
                target,
                -Vector2d.Right,
                -Vector2d.Right,
                context.DeltaTime * Fixed64.Half,
                context.DeltaTime * Fixed64.Half)
            .Should()
            .BeFalse();

        context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should()
            .Be(before);
    }

    private static bool InvokeKinematicHandoff2D(
        SolidBody2D source,
        SolidBody2D target,
        Vector2d displacement,
        Vector2d normal,
        Fixed64 hitDistance,
        Fixed64 sourceLength) =>
        source.ApplyKinematicContinuousCollisionHandoff(
            target,
            displacement,
            normal,
            hitDistance,
            sourceLength);
}
