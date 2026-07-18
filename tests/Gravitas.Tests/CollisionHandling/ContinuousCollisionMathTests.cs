using FixedMathSharp;
using FluentAssertions;
using Gravitas.CollisionHandling;
using System;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ContinuousCollisionMathTests
{
    [Fact]
    public void RotationalIntervalMotionBound_WithLargePivot_ShouldCoverScaleDependentPoseRounding()
    {
        Fixed64 pivotRadius = (Fixed64)1_000_000;

        ContinuousCollisionMath.TryResolveRotationalIntervalMotionBound(
            Vector2d.Zero,
            Fixed64.Zero,
            pivotRadius,
            Fixed64.One,
            out Fixed64 planarBound).Should().BeTrue();
        ContinuousCollisionMath.TryResolveRotationalIntervalMotionBound(
            Vector3d.Zero,
            Fixed64.Zero,
            pivotRadius,
            Fixed64.One,
            out Fixed64 spatialBound).Should().BeTrue();

        Fixed64.TryMultiplyDivide(
            pivotRadius,
            Fixed64.MinIncrement,
            Fixed64.One,
            out Fixed64 oneRawUnitAtScale).Should().BeTrue();
        planarBound.Should().BeGreaterThan(oneRawUnitAtScale);
        spatialBound.Should().Be(planarBound);
    }

    [Fact]
    public void RotationalIntervalMotionBound_ShouldCoverEvaluatedLargePivotPose()
    {
        Fixed64 pivotRadius = (Fixed64)1_000;
        Fixed64 midpointAngle = Fixed64.FromRaw(-23_617_013_816);
        Fixed64 halfSpan = Fixed64.FromRaw(4_294_967);
        Vector2d pivotOffset = new(pivotRadius, Fixed64.Zero);
        Vector2d midpoint = Vector2d.Rotate(pivotOffset, midpointAngle);
        Vector2d endpoint = Vector2d.Rotate(pivotOffset, midpointAngle + halfSpan);

        Vector2d.TrySubtract(endpoint, midpoint, out Vector2d evaluatedMotion).Should().BeTrue();
        Vector2d.TryGetMagnitude(evaluatedMotion, out Fixed64 evaluatedDistance).Should().BeTrue();
        ContinuousCollisionMath.TryResolveRotationalIntervalMotionBound(
            Vector2d.Zero,
            Fixed64.One,
            pivotRadius,
            halfSpan * Fixed64.Two,
            out Fixed64 planarMotionBound).Should().BeTrue();

        planarMotionBound.Should().BeGreaterThanOrEqualTo(evaluatedDistance);

        Vector3d spatialPivotOffset = new(pivotRadius, Fixed64.Zero, Fixed64.Zero);
        Vector3d spatialMidpoint = FixedQuaternion.FromAxisAngle(Vector3d.Up, midpointAngle)
            * spatialPivotOffset;
        Vector3d spatialEndpoint = FixedQuaternion.FromAxisAngle(Vector3d.Up, midpointAngle + halfSpan)
            * spatialPivotOffset;
        Vector3d.TrySubtract(
            spatialEndpoint,
            spatialMidpoint,
            out Vector3d evaluatedSpatialMotion).Should().BeTrue();
        Vector3d.TryGetMagnitude(
            evaluatedSpatialMotion,
            out Fixed64 evaluatedSpatialDistance).Should().BeTrue();
        ContinuousCollisionMath.TryResolveRotationalIntervalMotionBound(
            Vector3d.Zero,
            Fixed64.One,
            pivotRadius,
            halfSpan * Fixed64.Two,
            out Fixed64 spatialMotionBound).Should().BeTrue();

        spatialMotionBound.Should().BeGreaterThanOrEqualTo(evaluatedSpatialDistance);
    }

    [Fact]
    public void IsWithinProxyRadius_WhenBothSquaresSaturate_ShouldCompare3DLengths()
    {
        Vector3d displacement = new((Fixed64)20000, (Fixed64)40000, (Fixed64)40000);
        Fixed64 proxyRadius = (Fixed64)50000;

        displacement.MagnitudeSquared.Should().Be(Fixed64.MaxValue);
        (proxyRadius * proxyRadius).Should().Be(Fixed64.MaxValue);
        ContinuousCollisionMath.IsWithinProxyRadius(
            displacement,
            displacement.MagnitudeSquared,
            proxyRadius).Should().BeFalse();

        Vector3d maximumAxisDisplacement = new(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero);
        ContinuousCollisionMath.IsWithinProxyRadius(
            maximumAxisDisplacement,
            maximumAxisDisplacement.MagnitudeSquared,
            Fixed64.MaxValue).Should().BeTrue();

        Vector3d unrepresentableLength = new(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.Zero);
        ContinuousCollisionMath.IsWithinProxyRadius(
            unrepresentableLength,
            unrepresentableLength.MagnitudeSquared,
            Fixed64.MaxValue).Should().BeFalse();
    }

    [Fact]
    public void IsWithinProxyRadius_WhenBothSquaresSaturate_ShouldCompare2DLengths()
    {
        Vector2d displacement = new((Fixed64)40000, (Fixed64)40000);
        Fixed64 proxyRadius = (Fixed64)50000;

        displacement.MagnitudeSquared.Should().Be(Fixed64.MaxValue);
        (proxyRadius * proxyRadius).Should().Be(Fixed64.MaxValue);
        ContinuousCollisionMath.IsWithinProxyRadius(
            displacement,
            displacement.MagnitudeSquared,
            proxyRadius).Should().BeFalse();

        Vector2d shorterDisplacement = new((Fixed64)30000, (Fixed64)40000);
        ContinuousCollisionMath.IsWithinProxyRadius(
            shorterDisplacement,
            shorterDisplacement.MagnitudeSquared,
            (Fixed64)60000).Should().BeTrue();

        Vector2d unrepresentableLength = new(Fixed64.MaxValue, Fixed64.MaxValue);
        ContinuousCollisionMath.IsWithinProxyRadius(
            unrepresentableLength,
            unrepresentableLength.MagnitudeSquared,
            Fixed64.MaxValue).Should().BeFalse();

        Vector2d maximumAxisDisplacement = new(Fixed64.MaxValue, Fixed64.Zero);
        ContinuousCollisionMath.IsWithinProxyRadius(
            maximumAxisDisplacement,
            maximumAxisDisplacement.MagnitudeSquared,
            Fixed64.MaxValue).Should().BeTrue();
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

    [Fact]
    public void ValidateSweepEndpoint_ShouldPreserveRepresentableMotionAndRejectExtreme3DMotion()
    {
        Vector3d ordinaryEnd = new((Fixed64)3, (Fixed64)4, Fixed64.Zero);
        Vector3d ordinaryDisplacement = ContinuousCollisionSweepRange.ValidateEndpoint(
            Vector3d.Zero,
            ordinaryEnd,
            out Fixed64 ordinaryLength);
        ordinaryDisplacement.Should().Be(ordinaryEnd);
        ordinaryLength.Should().Be((Fixed64)5);

        Vector3d requestedExtremeEnd = new(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MaxValue);
        Action validate = () => ContinuousCollisionSweepRange.ValidateEndpoint(
            Vector3d.Zero,
            requestedExtremeEnd,
            out _);

        validate.Should().Throw<ArgumentOutOfRangeException>();
    }

    [Fact]
    public void ValidateSweepEndpoint_ShouldRejectComponentOverflowAndAcceptZero()
    {
        Vector2d start2D = new(Fixed64.MinValue, Fixed64.MinValue);
        Vector2d end2D = new(Fixed64.MaxValue, Fixed64.MaxValue);
        Action validate2D = () => ContinuousCollisionSweepRange.ValidateEndpoint(start2D, end2D, out _);
        Action validate3D = () => ContinuousCollisionSweepRange.ValidateEndpoint(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            out _);

        validate2D.Should().Throw<ArgumentOutOfRangeException>();
        validate3D.Should().Throw<ArgumentOutOfRangeException>();
        ContinuousCollisionSweepRange.ValidateEndpoint(
                Vector3d.Zero,
                Vector3d.Zero,
                out Fixed64 zeroLength)
            .Should().Be(Vector3d.Zero);
        zeroLength.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void ValidateSweepEndpoint_WithRequestedDisplacement_ShouldRejectSaturatedAddition()
    {
        Vector2d start2D = new(Fixed64.MaxValue - Fixed64.Half, Fixed64.Zero);
        Vector2d requested2D = Vector2d.Right;
        Vector2d saturatedEnd2D = start2D + requested2D;
        Vector3d start3D = new(Fixed64.MaxValue - Fixed64.Half, Fixed64.Zero, Fixed64.Zero);
        Vector3d requested3D = Vector3d.Right;
        Vector3d saturatedEnd3D = start3D + requested3D;

        Action validate2D = () => ContinuousCollisionSweepRange.ValidateEndpoint(
            start2D,
            saturatedEnd2D,
            requested2D,
            out _);
        Action validate3D = () => ContinuousCollisionSweepRange.ValidateEndpoint(
            start3D,
            saturatedEnd3D,
            requested3D,
            out _);

        validate2D.Should().Throw<ArgumentOutOfRangeException>();
        validate3D.Should().Throw<ArgumentOutOfRangeException>();
        ContinuousCollisionSweepRange.ValidateEndpoint(
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Right,
                out Fixed64 exactLength)
            .Should().Be(Vector3d.Right);
        exactLength.Should().Be(Fixed64.One);
        ContinuousCollisionSweepRange.ValidateEndpoint(
                Vector2d.Zero,
                Vector2d.Right,
                Vector2d.Right,
                out _)
            .Should().Be(Vector2d.Right);

    }

    [Fact]
    public void ValidateRelativeDisplacement_ShouldPreserveSmallMotionAndRejectRangeOverflow()
    {
        Vector3d source3D = new(
            Fixed64.FromRaw(1_000),
            Fixed64.FromRaw(1_000),
            Fixed64.Zero);
        Vector3d target3D = new(
            Fixed64.FromRaw(700),
            Fixed64.FromRaw(700),
            Fixed64.Zero);

        Vector3d relative3D = ContinuousCollisionSweepRange.ValidateRelativeDisplacement(
            source3D,
            target3D,
            out Fixed64 length3D);

        relative3D.Should().Be(new Vector3d(Fixed64.FromRaw(300), Fixed64.FromRaw(300), Fixed64.Zero));
        length3D.Should().BeGreaterThan(Fixed64.Zero);
        FixedMath.Abs(relative3D.Normalized.Magnitude - Fixed64.One).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);

        Action componentOverflow2D = () => ContinuousCollisionSweepRange.ValidateRelativeDisplacement(
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            new Vector2d(-Fixed64.One, Fixed64.Zero),
            out _);
        Action magnitudeOverflow2D = () => ContinuousCollisionSweepRange.ValidateRelativeDisplacement(
            new Vector2d(Fixed64.MaxValue, Fixed64.MaxValue),
            Vector2d.Zero,
            out _);
        Action componentOverflow3D = () => ContinuousCollisionSweepRange.ValidateRelativeDisplacement(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero),
            out _);
        Action magnitudeOverflow3D = () => ContinuousCollisionSweepRange.ValidateRelativeDisplacement(
            new Vector3d(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.Zero),
            Vector3d.Zero,
            out _);

        componentOverflow2D.Should().Throw<ArgumentOutOfRangeException>();
        magnitudeOverflow2D.Should().Throw<ArgumentOutOfRangeException>();
        componentOverflow3D.Should().Throw<ArgumentOutOfRangeException>();
        magnitudeOverflow3D.Should().Throw<ArgumentOutOfRangeException>();
        ContinuousCollisionSweepRange.ValidateRelativeDisplacement(
                Vector2d.Zero,
                Vector2d.Zero,
                out Fixed64 zeroLength)
            .Should().Be(Vector2d.Zero);
        zeroLength.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void RelativeSweepEntryPoints_ShouldRejectUnrepresentableRelativeMotionBeforeProxyMath()
    {
        Action sweepSpheres = () => ContinuousCollisionMath.TrySweepRelativeSpheres(
            Vector3d.Zero,
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            Vector3d.Right,
            -Vector3d.Right,
            Fixed64.Half,
            out _,
            out _,
            out _);
        Action sweepCircles = () => ContinuousCollisionMath.TrySweepRelativeCircles(
            Vector2d.Zero,
            new Vector2d(Fixed64.MaxValue, Fixed64.MaxValue),
            Fixed64.Half,
            Vector2d.Right,
            Vector2d.Zero,
            Fixed64.Half,
            out _,
            out _,
            out _);

        sweepSpheres.Should().Throw<ArgumentOutOfRangeException>();
        sweepCircles.Should().Throw<ArgumentOutOfRangeException>();
    }
}
