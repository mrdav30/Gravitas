using FixedMathSharp;
using FluentAssertions;
using Gravitas.CollisionHandling;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ContinuousCollisionMathTests
{
    [Fact]
    public void TrySweepRelativeSpheres_WithCoincidentOverlap_ShouldUseOpposingRelativeMotionNormal()
    {
        bool hit = ContinuousCollisionMath.TrySweepRelativeSpheres(
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.One,
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.One,
            out Fixed64 normalizedTime,
            out Vector3d normalForSource,
            out Fixed64 closingSpeed);

        hit.Should().BeTrue();
        normalizedTime.Should().Be(Fixed64.Zero);
        normalForSource.Should().Be(-Vector3d.Right);
        closingSpeed.Should().Be(Fixed64.One);
    }

    [Fact]
    public void TrySweepRelativeCircles_WithCoincidentOverlap_ShouldUseOpposingRelativeMotionNormal()
    {
        bool hit = ContinuousCollisionMath.TrySweepRelativeCircles(
            Vector2d.Zero,
            Vector2d.Right,
            Fixed64.One,
            Vector2d.Zero,
            Vector2d.Zero,
            Fixed64.One,
            out Fixed64 normalizedTime,
            out Vector2d normalForSource,
            out Fixed64 closingSpeed);

        hit.Should().BeTrue();
        normalizedTime.Should().Be(Fixed64.Zero);
        normalForSource.Should().Be(-Vector2d.Right);
        closingSpeed.Should().Be(Fixed64.One);
    }

    [Fact]
    public void ResolveContactPointOnTarget_ShouldPreferNormalThenCenterDeltaThenTargetCenter()
    {
        Vector3d targetCenter = new((Fixed64)4, (Fixed64)5, (Fixed64)6);

        ContinuousCollisionMath.ResolveContactPointOnTarget(
                sourceCenter: Vector3d.Zero,
                targetCenter,
                normalForSource: Vector3d.Up,
                targetRadius: (Fixed64)2)
            .Should().Be(targetCenter + Vector3d.Up * (Fixed64)2);

        ContinuousCollisionMath.ResolveContactPointOnTarget(
                sourceCenter: targetCenter + Vector3d.Right * (Fixed64)3,
                targetCenter,
                normalForSource: Vector3d.Zero,
                targetRadius: (Fixed64)2)
            .Should().Be(targetCenter + Vector3d.Right * (Fixed64)2);

        ContinuousCollisionMath.ResolveContactPointOnTarget(
                sourceCenter: targetCenter,
                targetCenter,
                normalForSource: Vector3d.Zero,
                targetRadius: (Fixed64)2)
            .Should().Be(targetCenter);
    }

    [Fact]
    public void ShouldReplaceContinuousCollisionHit_ShouldPreferFirstHitThenEarlierTimeThenLowerColliderId()
    {
        ContinuousCollisionMath.ShouldReplaceContinuousCollisionHit(
                candidateSafeTime: Fixed64.One,
                candidateTargetId: 10,
                hasCurrent: false,
                currentSafeTime: Fixed64.Zero,
                currentTargetId: 0)
            .Should().BeTrue();

        ContinuousCollisionMath.ShouldReplaceContinuousCollisionHit(
                candidateSafeTime: Fixed64.Half,
                candidateTargetId: 10,
                hasCurrent: true,
                currentSafeTime: Fixed64.One,
                currentTargetId: 1)
            .Should().BeTrue();

        ContinuousCollisionMath.ShouldReplaceContinuousCollisionHit(
                candidateSafeTime: Fixed64.One,
                candidateTargetId: 10,
                hasCurrent: true,
                currentSafeTime: Fixed64.Half,
                currentTargetId: 1)
            .Should().BeFalse();

        ContinuousCollisionMath.ShouldReplaceContinuousCollisionHit(
                candidateSafeTime: Fixed64.Half,
                candidateTargetId: 3,
                hasCurrent: true,
                currentSafeTime: Fixed64.Half,
                currentTargetId: 4)
            .Should().BeTrue();

        ContinuousCollisionMath.ShouldReplaceContinuousCollisionHit(
                candidateSafeTime: Fixed64.Half,
                candidateTargetId: 5,
                hasCurrent: true,
                currentSafeTime: Fixed64.Half,
                currentTargetId: 4)
            .Should().BeFalse();
    }
}
