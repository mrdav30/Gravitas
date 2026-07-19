using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed partial class ContinuousCollision2DTests
{
    [Fact]
    public void ContinuousMode_KinematicRotation_ShouldCatchContactBetweenSeparatedEndpoints()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D blade = CreateRotationalBlade(context);
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 4)),
            new Vector2d(Fixed64.FromFraction(16, 5), Fixed64.Zero),
            immovable: true);
        Fixed64 startRotation = FixedMath.DegToRad((Fixed64)(-5));
        Fixed64 contactRotation = Fixed64.Zero;
        Fixed64 targetRotation = -startRotation;

        AssertRotationalWitness(blade, target, startRotation, contactRotation, targetRotation);
        blade.SetRotation(startRotation);
        blade.Agent.Transform.LocalRotationXZRadians = targetRotation;

        context.LateSimulate();

        blade.Rotation.Should().BeLessThan(targetRotation);
        blade.Rotation.Should().BeGreaterThanOrEqualTo(startRotation);
        blade.LastContinuousCollisionToiIterationCount.Should().Be(1);
    }

    [Fact]
    public void ContinuousMode_KinematicRotation_ShouldCatchShiftedContactBetweenEndpointAndMidpointSamples()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        SolidBody2D blade = CreateRotationalBlade(context);
        Fixed64 contactRotation = FixedMath.DegToRad(Fixed64.FromFraction(5, 2));
        Vector2d targetPosition = Vector2d.Rotate(
            new Vector2d(Fixed64.FromFraction(16, 5), Fixed64.Zero),
            contactRotation);
        SolidBody2D target = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.FromFraction(401, 2000)),
            targetPosition,
            immovable: true);
        Fixed64 startRotation = FixedMath.DegToRad((Fixed64)(-5));
        Fixed64 targetRotation = -startRotation;

        AssertRotationalWitness(blade, target, startRotation, contactRotation, targetRotation);
        blade.SetRotation(Fixed64.Zero);
        CollisionDetection2D.TryCollide(blade.Collider, target.Collider, out _).Should().BeFalse();
        blade.SetRotation(startRotation);
        blade.Agent.Transform.LocalRotationXZRadians = targetRotation;

        context.LateSimulate();

        blade.Rotation.Should().BeLessThan(targetRotation);
        blade.Rotation.Should().BeGreaterThanOrEqualTo(startRotation);
        blade.LastContinuousCollisionToiIterationCount.Should().Be(1);
    }

    [Fact]
    public void ContinuousMode_OffsetCircleRotation_ShouldUseBodyPivotRadius()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        var sourceCollider = new LSCircleCollider2D(Fixed64.FromFraction(1, 8))
        {
            LocalOffset = new Vector2d((Fixed64)3, Fixed64.Zero)
        };
        SolidBody2D source = CreateBody(
            context,
            sourceCollider,
            Vector2d.Zero,
            immovable: false,
            isKinematic: true);
        _ = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.FromFraction(1, 8)),
            new Vector2d(Fixed64.FromFraction(16, 5), Fixed64.Zero),
            immovable: true);
        Fixed64 startRotation = FixedMath.DegToRad((Fixed64)(-5));
        Fixed64 targetRotation = -startRotation;
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.SetRotation(startRotation);
        source.Agent.Transform.LocalRotationXZRadians = targetRotation;

        context.LateSimulate();

        source.Rotation.Should().BeLessThan(targetRotation);
        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
    }

    [Fact]
    public void ContinuousMode_UnrepresentablePivotRadius_ShouldUseBoundedRegisteredCandidateFallback()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        var sourceCollider = new LSCircleCollider2D(Fixed64.One);
        SolidBody2D source = CreateBody(
            context,
            sourceCollider,
            Vector2d.Zero,
            immovable: false,
            isKinematic: true);
        _ = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.One),
            Vector2d.Zero,
            immovable: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        sourceCollider.LocalOffset = new Vector2d(Fixed64.MaxValue, Fixed64.Zero);
        source.ResolveContinuousCollisionProxyRadius().Should().Be(Fixed64.MaxValue);

        source.GatherRotationalContinuousCollisionCandidates(
            Vector2d.Zero,
            Vector2d.Zero,
            Vector2d.Zero,
            Fixed64.MaxValue).Should().Be(1);
    }

    [Fact]
    public void CapsuleProxyRadius_WithUnrepresentableBoundsDistance_ShouldRemainConservative()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        var collider = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3);
        SolidBody2D source = CreateBody(
            context,
            collider,
            Vector2d.Zero,
            immovable: false);
        collider.LocalOffset = new Vector2d(Fixed64.MaxValue, Fixed64.MaxValue);
        collider.RebuildRuntimeShapeOnly();

        source.ResolveContinuousCollisionProxyRadius().Should().Be(Fixed64.MaxValue);
    }

    private static SolidBody2D CreateRotationalBlade(GravitasWorldContext context)
    {
        var collider = new LSPolygonCollider2D(
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(1, 10)),
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(1, 10)));
        SolidBody2D blade = CreateBody(
            context,
            collider,
            Vector2d.Zero,
            immovable: false,
            isKinematic: true);
        blade.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        return blade;
    }

    private static void AssertRotationalWitness(
        SolidBody2D source,
        SolidBody2D target,
        Fixed64 startRotation,
        Fixed64 contactRotation,
        Fixed64 endRotation)
    {
        source.SetRotation(startRotation);
        CollisionDetection2D.TryCollide(source.Collider, target.Collider, out _).Should().BeFalse();
        source.SetRotation(contactRotation);
        CollisionDetection2D.TryCollide(source.Collider, target.Collider, out _).Should().BeTrue();
        source.SetRotation(endRotation);
        CollisionDetection2D.TryCollide(source.Collider, target.Collider, out _).Should().BeFalse();
    }
}
