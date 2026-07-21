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
    public void MixedContinuousMode_PiecewiseOutAndReturn3DTarget_ShouldBlock2DSource()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Settings.ContinuousCollisionMaxToiIterations = 4;
        SolidBody2D source = CreateCircle2D(
            context,
            new Vector2d((Fixed64)(-5), Fixed64.Zero));
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)3));
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.ApplyCollisionLinearVelocityDelta(Vector2d.Right * (Fixed64)10);
        target.Body.ApplyCollisionLinearVelocityDelta(
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)(-6)));
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        target.Body.ApplyContinuousCollisionHandoff(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)6),
                Vector3d.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();

        source.LateSimulate(updateSleepState: false, updateColliderState: true);

        source.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
        source.Position.X.Should().BeLessThan((Fixed64)5);
    }

    [Fact]
    public void MixedContinuousMode_PiecewiseOutAndReturn2DTarget_ShouldBlock3DSource()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Settings.ContinuousCollisionMaxToiIterations = 4;
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D target = CreateCircle2D(
            context,
            new Vector2d(Fixed64.Zero, (Fixed64)3));
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)10);
        target.ApplyCollisionLinearVelocityDelta(new Vector2d(Fixed64.Zero, (Fixed64)(-6)));
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        target.ApplyContinuousCollisionHandoffState(
                Vector2d.Zero,
                Fixed64.Zero,
                new Vector2d(Fixed64.Zero, (Fixed64)6),
                Fixed64.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();

        source.Body.LateSimulate(updateSleepState: false, updateColliderState: true);

        source.Body.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
        source.Body.Position3d.X.Should().BeLessThan((Fixed64)5);
    }

    [Fact]
    public void MixedContinuousMode_PiecewiseOutAndReturn2DTarget_ShouldReceive3DKinematicHandoff()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        SolidBody2D target = CreateCircle2D(
            context,
            new Vector2d(Fixed64.Zero, (Fixed64)3));
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Body.Agent.Transform.LocalPosition = new Vector3d(
            (Fixed64)5,
            Fixed64.Zero,
            Fixed64.Zero);
        target.ApplyCollisionLinearVelocityDelta(
            new Vector2d(Fixed64.Zero, (Fixed64)(-6)));
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        target.ApplyContinuousCollisionHandoffState(
                Vector2d.Zero,
                Fixed64.Zero,
                new Vector2d(Fixed64.Zero, (Fixed64)6),
                Fixed64.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();

        source.Body.LateSimulate(updateSleepState: false, updateColliderState: true);

        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        target.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void MixedContinuousMode_PiecewiseOutAndReturn3DTarget_ShouldReceive2DKinematicHandoff()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)3));
        SolidBody2D source = CreateCircle2D(
            context,
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Agent.Transform.LocalPosition = new Vector3d(
            (Fixed64)5,
            Fixed64.Zero,
            Fixed64.Zero);
        target.Body.ApplyCollisionLinearVelocityDelta(
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)(-6)));
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        target.Body.ApplyContinuousCollisionHandoff(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)6),
                Vector3d.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();

        source.LateSimulate(updateSleepState: false, updateColliderState: true);

        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
        target.Body.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void MixedContinuousMode_3DTargetReversingAtTouchBoundary_ShouldUseSeparatingSegment()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        SolidBody2D source = CreateCircle2D(
            context,
            new Vector2d((Fixed64)(-3), Fixed64.Zero));
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)4));
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.ApplyCollisionLinearVelocityDelta(Vector2d.Right * (Fixed64)6);
        target.Body.ApplyCollisionLinearVelocityDelta(
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)(-6)));
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        target.Body.ApplyContinuousCollisionHandoff(
                new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.One),
                FixedQuaternion.Identity,
                new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)6),
                Vector3d.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();

        source.LateSimulate(updateSleepState: false, updateColliderState: true);

        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        source.Position.X.Should().Be((Fixed64)3);
    }

    [Fact]
    public void MixedContinuousMode_2DTargetReversingAtTouchBoundary_ShouldUseSeparatingSegment()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D target = CreateCircle2D(
            context,
            new Vector2d(Fixed64.Zero, (Fixed64)4));
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)6);
        target.ApplyCollisionLinearVelocityDelta(
            new Vector2d(Fixed64.Zero, (Fixed64)(-6)));
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        target.ApplyContinuousCollisionHandoffState(
                new Vector2d(Fixed64.Zero, Fixed64.One),
                Fixed64.Zero,
                new Vector2d(Fixed64.Zero, (Fixed64)6),
                Fixed64.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();

        source.Body.LateSimulate(updateSleepState: false, updateColliderState: true);

        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(0);
        source.Body.Position3d.X.Should().Be((Fixed64)3);
    }
}
