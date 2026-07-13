using FixedMathSharp;
using FluentAssertions;
using Gravitas.CollisionHandling;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ContinuousCollisionMathTests
{
    [Theory]
    [InlineData(0, 0)]
    [InlineData(1, 1)]
    [InlineData(30, 2)]
    [InlineData(360, 16)]
    public void ResolveRotationalSubstepCount_ShouldBoundAngularSampling(int degrees, int expected)
    {
        int steps = ContinuousCollisionMath.ResolveRotationalSubstepCount(
            FixedMath.DegToRad((Fixed64)degrees));

        steps.Should().Be(expected);
    }

    [Fact]
    public void TrySweepRelativeSpheres_ShouldCoverClosingSeparatingAndOutOfRangeCases()
    {
        ContinuousCollisionMath.TrySweepRelativeSpheres(
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.Half,
            Vector3d.Right,
            Vector3d.Zero,
            Fixed64.Half,
            out _,
            out _,
            out _).Should().BeFalse();

        ContinuousCollisionMath.TrySweepRelativeSpheres(
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.Zero,
            Vector3d.Right,
            Vector3d.Zero,
            Fixed64.Zero,
            out _,
            out _,
            out _).Should().BeFalse();

        ContinuousCollisionMath.TrySweepRelativeSpheres(
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.Half,
            Vector3d.Right * Fixed64.Half,
            Vector3d.Zero,
            Fixed64.Half,
            out Fixed64 overlapTime,
            out Vector3d overlapNormal,
            out Fixed64 overlapClosingSpeed).Should().BeTrue();
        overlapTime.Should().Be(Fixed64.Zero);
        overlapNormal.Should().Be(-Vector3d.Right);
        overlapClosingSpeed.Should().Be(Fixed64.One);

        ContinuousCollisionMath.TrySweepRelativeSpheres(
            Vector3d.Zero,
            -Vector3d.Right,
            Fixed64.Half,
            Vector3d.Right * Fixed64.Half,
            Vector3d.Zero,
            Fixed64.Half,
            out _,
            out _,
            out _).Should().BeFalse();

        ContinuousCollisionMath.TrySweepRelativeSpheres(
            -Vector3d.Right * (Fixed64)4,
            Vector3d.Right * (Fixed64)4,
            Fixed64.Half,
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.Half,
            out Fixed64 hitTime,
            out Vector3d hitNormal,
            out Fixed64 closingSpeed).Should().BeTrue();
        hitTime.Should().BeInRange(Fixed64.Zero, Fixed64.One);
        hitNormal.Should().Be(-Vector3d.Right);
        closingSpeed.Should().Be((Fixed64)4);

        ContinuousCollisionMath.TrySweepRelativeSpheres(
            -Vector3d.Right * (Fixed64)4,
            -Vector3d.Right,
            Fixed64.Half,
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.Half,
            out _,
            out _,
            out _).Should().BeFalse();

        ContinuousCollisionMath.TrySweepRelativeSpheres(
            new Vector3d((Fixed64)(-4), (Fixed64)4, Fixed64.Zero),
            Vector3d.Right,
            Fixed64.Half,
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.Half,
            out _,
            out _,
            out _).Should().BeFalse();

        ContinuousCollisionMath.TrySweepRelativeSpheres(
            -Vector3d.Right * (Fixed64)4,
            Vector3d.Right,
            Fixed64.Half,
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.Half,
            out _,
            out _,
            out _).Should().BeFalse();
    }

    [Fact]
    public void TrySweepRelativeSpheres_AtClosingSpeedEpsilon_ShouldRejectNearTangency()
    {
        Fixed64 relativeClosingSpeed = Fixed64.Epsilon;
        Fixed64 combinedRadius = (Fixed64)128;
        Fixed64 radius = combinedRadius * Fixed64.Half;
        bool hit = ContinuousCollisionMath.TrySweepRelativeSpheres(
            new Vector3d(
                combinedRadius + relativeClosingSpeed * Fixed64.Half,
                -Fixed64.Half,
                Fixed64.Zero),
            new Vector3d(-relativeClosingSpeed, Fixed64.One, Fixed64.Zero),
            radius,
            Vector3d.Zero,
            Vector3d.Zero,
            radius,
            out Fixed64 normalizedTime,
            out _,
            out Fixed64 closingSpeed);

        hit.Should().BeFalse();
        normalizedTime.Should().Be(Fixed64.Zero);
        closingSpeed.Should().Be(Fixed64.Epsilon);
    }

    [Fact]
    public void TrySweepRelative_WithUnderflowingNonzeroTangentRadius_ShouldUseScaledImpactNormal()
    {
        Fixed64 radius = Fixed64.Epsilon + Fixed64.MinIncrement;
        Fixed64 combinedRadius = radius + radius;

        radius.Should().BeGreaterThan(Fixed64.Epsilon);
        (combinedRadius * combinedRadius).Should().Be(Fixed64.Zero);

        bool sphereHit = ContinuousCollisionMath.TrySweepRelativeSpheres(
            new Vector3d(-Fixed64.One, combinedRadius, Fixed64.Zero),
            Vector3d.Right * (Fixed64)2,
            radius,
            Vector3d.Zero,
            Vector3d.Zero,
            radius,
            out _,
            out _,
            out _);
        bool circleHit = ContinuousCollisionMath.TrySweepRelativeCircles(
            new Vector2d(-Fixed64.One, combinedRadius),
            Vector2d.Right * (Fixed64)2,
            radius,
            Vector2d.Zero,
            Vector2d.Zero,
            radius,
            out _,
            out _,
            out _);

        sphereHit.Should().BeFalse();
        circleHit.Should().BeFalse();
    }

    [Fact]
    public void TrySweepRelativeCircles_ShouldMirrorSphereRelativeSweepSemantics()
    {
        ContinuousCollisionMath.TrySweepRelativeCircles(
            -Vector2d.Right * (Fixed64)4,
            Vector2d.Right * (Fixed64)4,
            Fixed64.Half,
            Vector2d.Zero,
            Vector2d.Zero,
            Fixed64.Half,
            out Fixed64 hitTime,
            out Vector2d hitNormal,
            out Fixed64 closingSpeed).Should().BeTrue();
        hitTime.Should().BeInRange(Fixed64.Zero, Fixed64.One);
        hitNormal.Should().Be(-Vector2d.Right);
        closingSpeed.Should().Be((Fixed64)4);

        ContinuousCollisionMath.TrySweepRelativeCircles(
            Vector2d.Zero,
            Vector2d.Right,
            Fixed64.Half,
            Vector2d.Right * Fixed64.Half,
            Vector2d.Zero,
            Fixed64.Half,
            out Fixed64 overlapTime,
            out Vector2d overlapNormal,
            out Fixed64 overlapClosingSpeed).Should().BeTrue();
        overlapTime.Should().Be(Fixed64.Zero);
        overlapNormal.Should().Be(-Vector2d.Right);
        overlapClosingSpeed.Should().Be(Fixed64.One);

        ContinuousCollisionMath.TrySweepRelativeCircles(
            Vector2d.Zero,
            -Vector2d.Right,
            Fixed64.Half,
            Vector2d.Right * Fixed64.Half,
            Vector2d.Zero,
            Fixed64.Half,
            out _,
            out _,
            out _).Should().BeFalse();

        ContinuousCollisionMath.TrySweepRelativeCircles(
            new Vector2d((Fixed64)(-3), (Fixed64)2),
            Vector2d.Right * (Fixed64)4,
            Fixed64.Half,
            Vector2d.Zero,
            Vector2d.Zero,
            Fixed64.Half,
            out _,
            out _,
            out _).Should().BeFalse();

        ContinuousCollisionMath.TrySweepRelativeCircles(
            -Vector2d.Right * (Fixed64)5,
            Vector2d.Right * (Fixed64)2,
            Fixed64.Half,
            Vector2d.Zero,
            Vector2d.Zero,
            Fixed64.Half,
            out _,
            out _,
            out _).Should().BeFalse();

        ContinuousCollisionMath.TrySweepRelativeCircles(
            new Vector2d((Fixed64)(-1), Fixed64.One),
            Vector2d.Right * (Fixed64)2,
            Fixed64.Half,
            Vector2d.Zero,
            Vector2d.Zero,
            Fixed64.Half,
            out _,
            out _,
            out _).Should().BeFalse();

        ContinuousCollisionMath.TrySweepRelativeCircles(
            new Vector2d((Fixed64)(-4), Fixed64.One),
            Vector2d.Right * (Fixed64)4,
            Fixed64.Half,
            Vector2d.Zero,
            Vector2d.Zero,
            Fixed64.Half,
            out _,
            out _,
            out _).Should().BeFalse();

        ContinuousCollisionMath.TrySweepRelativeCircles(
            Vector2d.Zero,
            Vector2d.Right,
            Fixed64.Half,
            Vector2d.Zero,
            Vector2d.Zero,
            Fixed64.Half,
            out Fixed64 centeredOverlapTime,
            out Vector2d centeredOverlapNormal,
            out Fixed64 centeredOverlapClosingSpeed).Should().BeTrue();
        centeredOverlapTime.Should().Be(Fixed64.Zero);
        centeredOverlapNormal.Should().Be(-Vector2d.Right);
        centeredOverlapClosingSpeed.Should().Be(Fixed64.One);
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
