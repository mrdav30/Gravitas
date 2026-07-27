using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedQueryCcdTests
{
    [Fact]
    public void MixedDynamic3DSphere_WithRotation_ShouldRetainTranslationalContact()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D target = CreateCircle2D(
            context,
            Vector2d.Zero,
            isKinematic: true);
        target.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;
        target.Agent.Transform.LocalPosition = Vector3d.Right;
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Body.ApplyCollisionLinearVelocityDelta(
            Vector3d.Right * (Fixed64)10);
        source.Body.ApplyCollisionAngularVelocityDelta(
            Vector3d.Up * Fixed64.MinIncrement);

        context.LateSimulate();

        source.Body.LastContinuousCollisionToiIterationCount
            .Should()
            .BeGreaterThan(0);
        source.Body.Position3d.X.Should().BeLessThan((Fixed64)5);
        target.Position.Should().Be(Vector2d.Right);
    }

    [Fact]
    public void MixedDynamic2DCircle_WithRotation_ShouldRetainTranslationalContact()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        SolidBody2D source = CreateCircle2D(
            context,
            new Vector2d((Fixed64)(-5), Fixed64.Zero));
        ScenarioBody<LSSphereCollider> target =
            CreateSphere3D(context, Vector3d.Zero, isKinematic: true);
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;
        target.Body.Agent.Transform.LocalPosition = Vector3d.Right;
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.ApplyCollisionLinearVelocityDelta(
            Vector2d.Right * (Fixed64)10);
        source.ApplyCollisionAngularVelocityDelta(
            Fixed64.MinIncrement);

        context.LateSimulate();

        source.LastContinuousCollisionToiIterationCount
            .Should()
            .BeGreaterThan(0);
        source.Position.X.Should().BeLessThan((Fixed64)5);
        target.Body.Position3d.Should().Be(Vector3d.Right);
    }

    [Fact]
    public void MixedKinematic3DSphere_WithOffsetRotation_ShouldRejectChordOnlyContact()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        SolidBody2D target = CreateCircle2D(context, Vector2d.Zero);
        target.Sleep();
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            -Vector3d.Right,
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Collider.LocalOffset = Vector3d.Forward * Fixed64.Two;
        source.Collider.RebuildRuntimeShapeOnly().Should().BeTrue();
        source.Body.Agent.Transform.LocalPosition = Vector3d.Right;
        source.Body.Agent.Transform.LocalRotation =
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.Pi);

        context.LateSimulate();

        source.Body.Position3d.Should().Be(Vector3d.Right);
        target.LinearVelocity.Should().Be(Vector2d.Zero);
        target.IsSleeping.Should().BeTrue();
    }

    [Fact]
    public void MixedKinematic2DCircle_WithOffsetRotation_ShouldRejectChordOnlyContact()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> target =
            CreateSphere3D(context, Vector3d.Zero);
        target.Body.Sleep();
        SolidBody2D source = CreateCircle2D(
            context,
            -Vector2d.Right,
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Collider.LocalOffset = Vector2d.Forward * Fixed64.Two;
        source.Collider.RebuildRuntimeShapeOnly().Should().BeTrue();
        source.Agent.Transform.LocalPosition = Vector3d.Right;
        source.Agent.Transform.LocalRotationXZRadians = Fixed64.Pi;

        context.LateSimulate();

        source.Position.Should().Be(Vector2d.Right);
        target.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        target.Body.IsSleeping.Should().BeTrue();
    }

    [Fact]
    public void MixedKinematic3DSphere_AtScalarEdge_ShouldFallbackWhenRelativeEndpointCannotMaterialize()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        Fixed64 edge = Fixed64.MaxValue;
        SolidBody2D target = CreateCircle2D(
            context,
            new Vector2d(edge - Fixed64.Two, Fixed64.Zero));
        target.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.ApplyCollisionLinearVelocityDelta(
            -Vector2d.Right * (Fixed64)10);
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            new Vector3d(edge - (Fixed64)10, Fixed64.Zero, Fixed64.Zero),
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector3d requested =
            new(edge - (Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        source.Body.Agent.Transform.LocalPosition = requested;

        context.LateSimulate();

        source.Body.Position3d.Should().Be(requested);
        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        target.LinearVelocity.X.Should().BeGreaterThan((Fixed64)(-10));
    }

    [Fact]
    public void MixedKinematic2DCircle_AtScalarEdge_ShouldFallbackWhenRelativeEndpointCannotMaterialize()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        Fixed64 edge = Fixed64.MaxValue;
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(
            context,
            new Vector3d(edge - (Fixed64)10, Fixed64.Zero, Fixed64.Zero));
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ApplyCollisionLinearVelocityDelta(
            Vector3d.Right * (Fixed64)5);
        SolidBody2D source = CreateCircle2D(
            context,
            new Vector2d(edge - Fixed64.Two, Fixed64.Zero),
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector2d requested = new(
            edge - (Fixed64)12,
            Fixed64.Zero);
        source.Agent.Transform.LocalPosition =
            requested.ToVector3d(Fixed64.Zero);

        context.LateSimulate();

        source.Position.Should().Be(requested);
        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
        target.Body.LinearVelocity.X.Should().BeLessThan((Fixed64)5);
    }

    [Fact]
    public void MixedKinematic3DSphere_OutsideTallSlabRadius_ShouldFallbackConservatively()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        SolidBody2D target = CreateCircle2D(context, Vector2d.Zero);
        target.Collider.MixedHalfThicknessOverride = (Fixed64)10;
        target.Collider.RebuildRuntimeShapeOnly().Should().BeTrue();
        target.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Sleep();
        Fixed64 cornerOffset = Fixed64.FromFraction(9, 10);
        Vector3d start = new(
            cornerOffset,
            (Fixed64)(-5),
            cornerOffset);
        Vector3d requested = new(
            cornerOffset,
            (Fixed64)5,
            cornerOffset);
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            start,
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Body.Agent.Transform.LocalPosition = requested;

        context.LateSimulate();

        source.Body.Position3d.Should().Be(start);
        target.IsSleeping.Should().BeTrue();
        target.LinearVelocity.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void MixedKinematic2DCircle_OutsideTallSlabRadius_ShouldRejectBroadProxyOnlyHit()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> target =
            CreateSphere3D(context, Vector3d.Zero);
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.Sleep();
        Vector2d start = new((Fixed64)(-5), (Fixed64)5);
        Vector2d requested = new((Fixed64)5, (Fixed64)5);
        SolidBody2D source = CreateCircle2D(
            context,
            start,
            isKinematic: true);
        source.Collider.MixedHalfThicknessOverride = (Fixed64)10;
        source.Collider.RebuildRuntimeShapeOnly().Should().BeTrue();
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Agent.Transform.LocalPosition =
            requested.ToVector3d(Fixed64.Zero);

        context.LateSimulate();

        source.Position.Should().Be(requested);
        target.Body.IsSleeping.Should().BeTrue();
        target.Body.LinearVelocity.Should().Be(Vector3d.Zero);
    }
}
