using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedQueryCcdTests
{
    [Fact]
    public void MixedDynamicResponse_InvalidNormal_ShouldRejectAtomicallyForEitherSourceDimension()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        SolidBody2D source2D = CreateCircle2D(
            context,
            new Vector2d((Fixed64)(-2), Fixed64.Zero));
        ScenarioBody<LSSphereCollider> source3D = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D target2D = CreateCircle2D(context, Vector2d.Zero);
        ScenarioBody<LSSphereCollider> target3D = CreateSphere3D(context, Vector3d.Zero);
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        var before = context.ComputeReplayHash(
            GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        InvokeMixedDynamicResponse2D(
                source2D,
                target3D.Body,
                Vector2d.Zero,
                source2D.Position,
                target3D.Body.Position3d,
                Fixed64.Half,
                Fixed64.Half)
            .Should()
            .BeFalse();
        InvokeMixedDynamicResponse3D(
                source3D.Body,
                target2D,
                Vector3d.Zero,
                source3D.Body.Position3d,
                target2D.Position,
                Fixed64.Half,
                Fixed64.Half)
            .Should()
            .BeFalse();

        context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should()
            .Be(before);
    }

    [Fact]
    public void MixedKinematicHandoff_NonClosingPair_ShouldRejectAtomicallyForEitherSourceDimension()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        SolidBody2D source2D = CreateCircle2D(
            context,
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            isKinematic: true);
        ScenarioBody<LSSphereCollider> source3D = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            isKinematic: true);
        SolidBody2D target2D = CreateCircle2D(context, Vector2d.Zero);
        ScenarioBody<LSSphereCollider> target3D = CreateSphere3D(context, Vector3d.Zero);
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        var before = context.ComputeReplayHash(
            GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        InvokeMixedKinematicHandoff2D(
                source2D,
                target3D.Body,
                Vector2d.Right,
                -Vector2d.Right,
                Fixed64.Half,
                Fixed64.One)
            .Should()
            .BeFalse();
        InvokeMixedKinematicHandoff3D(
                source3D.Body,
                target2D,
                Vector3d.Right,
                -Vector3d.Right,
                Fixed64.Half,
                Fixed64.One)
            .Should()
            .BeFalse();

        context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should()
            .Be(before);
    }

    private static bool InvokeMixedDynamicResponse2D(
        SolidBody2D source,
        SolidBody target,
        Vector2d normal,
        Vector2d sourcePositionAtImpact,
        Vector3d targetPositionAtImpact,
        Fixed64 hitElapsedTime,
        Fixed64 remainingTime) =>
        source.TryApplyContinuousCollisionMixed3DResponse(
            target,
            normal,
            sourcePositionAtImpact,
            targetPositionAtImpact,
            hitElapsedTime,
            remainingTime);

    private static bool InvokeMixedDynamicResponse3D(
        SolidBody source,
        SolidBody2D target,
        Vector3d normal,
        Vector3d sourcePositionAtImpact,
        Vector2d targetPositionAtImpact,
        Fixed64 hitElapsedTime,
        Fixed64 remainingTime) =>
        source.TryApplyContinuousCollisionMixed2DResponse(
            target,
            normal,
            sourcePositionAtImpact,
            targetPositionAtImpact,
            hitElapsedTime,
            remainingTime);

    private static bool InvokeMixedKinematicHandoff2D(
        SolidBody2D source,
        SolidBody target,
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

    private static bool InvokeMixedKinematicHandoff3D(
        SolidBody source,
        SolidBody2D target,
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
