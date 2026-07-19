using Chronicler;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedQueryCcdTests
{
    [Fact]
    public void RotationalMixedResponse_From2D_When3DTargetTrajectoryIsFull_ShouldRemainAtomic()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Settings.ContinuousCollisionMaxToiIterations = 1;
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        SolidBody2D source = CreateMixedTranslationSource2D(
            context,
            -Vector2d.Right);
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(
            context,
            Vector3d.Zero);
        source.ApplyCollisionLinearVelocityDelta(Vector2d.Right * Fixed64.Two);
        PrepareMixedContinuousCollisionFrame(context);
        target.Body.ApplyContinuousCollisionHandoff(
                target.Body.Position3d,
                target.Body.Rotation,
                Vector3d.Zero,
                Vector3d.Zero,
                context.DeltaTime * Fixed64.FromFraction(3, 4))
            .Should()
            .BeTrue();
        ChronicleHash before = context.ComputeReplayHash(
            GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        bool applied = source.TryApplyMixedRotationalContinuousCollisionResponse(
            target.Collider,
            CreateMixedContact(Vector3d.Left),
            Fixed64.Half,
            source.Position,
            Vector2d.Zero,
            source.Rotation,
            Fixed64.Zero,
            Fixed64.Zero,
            context.DeltaTime);

        applied.Should().BeFalse();
        context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should()
            .Be(before);
    }

    [Fact]
    public void RotationalMixedResponse_From3D_When2DTargetTrajectoryIsFull_ShouldRemainAtomic()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Settings.ContinuousCollisionMaxToiIterations = 1;
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            -Vector3d.Right);
        SolidBody2D target = CreateCircle2D(context, Vector2d.Zero);
        source.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * Fixed64.Two);
        PrepareMixedContinuousCollisionFrame(context);
        target.ApplyContinuousCollisionHandoffState(
                target.Position,
                target.Rotation,
                Vector2d.Zero,
                Fixed64.Zero,
                context.DeltaTime * Fixed64.FromFraction(3, 4))
            .Should()
            .BeTrue();
        ChronicleHash before = context.ComputeReplayHash(
            GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        bool applied = source.Body.TryApplyMixedRotationalContinuousCollisionResponse(
            target.Collider,
            CreateMixedContact(Vector3d.Right),
            Fixed64.Half,
            source.Body.Position3d,
            Vector3d.Zero,
            source.Body.Rotation,
            source.Body.Rotation,
            Fixed64.Zero,
            context.DeltaTime,
            sourceIsKinematic: false);

        applied.Should().BeFalse();
        context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should()
            .Be(before);
    }

    [Fact]
    public void RotationalMixedResponse_From3D_ShouldApplyAtomicallyToDynamic2DTarget()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            -Vector3d.Right);
        SolidBody2D target = CreateCircle2D(context, Vector2d.Zero);
        source.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * Fixed64.Two);
        PrepareMixedContinuousCollisionFrame(context);

        bool applied = source.Body.TryApplyMixedRotationalContinuousCollisionResponse(
            target.Collider,
            CreateMixedContact(Vector3d.Right),
            Fixed64.Half,
            source.Body.Position3d,
            Vector3d.Zero,
            source.Body.Rotation,
            source.Body.Rotation,
            Fixed64.Zero,
            context.DeltaTime,
            sourceIsKinematic: false);

        applied.Should().BeTrue();
        target.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        target.ContinuousCollisionTrajectoryCount.Should().BeGreaterThan(1);
    }

    [Fact]
    public void RotationalMixedResponse_From3D_ShouldApplyOnlyToSourceForBodyless2DTarget()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            -Vector3d.Right);
        LSCollider2D target = CreateBodylessCircle2D(context, Vector2d.Zero);
        source.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * Fixed64.Two);
        PrepareMixedContinuousCollisionFrame(context);
        Vector2d targetPosition = target.Center;

        bool applied = source.Body.TryApplyMixedRotationalContinuousCollisionResponse(
            target,
            CreateMixedContact(Vector3d.Right),
            Fixed64.Half,
            source.Body.Position3d,
            Vector3d.Zero,
            source.Body.Rotation,
            source.Body.Rotation,
            Fixed64.Zero,
            context.DeltaTime,
            sourceIsKinematic: false);

        applied.Should().BeTrue();
        source.Body.LinearVelocity.X.Should().BeLessThan(Fixed64.Two);
        source.Body.ContinuousCollisionTrajectoryCount.Should().BeGreaterThan(1);
        target.Center.Should().Be(targetPosition);
    }

    private static void PrepareMixedContinuousCollisionFrame(GravitasWorldContext context)
    {
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
    }

    private static MixedContact CreateMixedContact(Vector3d normal) =>
        new(
            -Vector3d.Right * Fixed64.Half,
            -Vector3d.Right * Fixed64.Half,
            normal,
            Fixed64.Zero);
}
