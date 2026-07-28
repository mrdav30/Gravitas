using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedQueryCcdTests
{
    [Fact]
    public void MixedMode_Dynamic2DRotation_ShouldClampConservativelyWithoutExactContactWitness()
    {
        var first = RunUnresolved2DRotationalSearch();
        var second = RunUnresolved2DRotationalSearch();

        second.Should().Be(first);
        first.Rotation.Should().BeLessThan(RotationalMixedQuarterTurn);
        first.AngularVelocity.Should().Be(Fixed64.Zero);
        first.ToiIterations.Should().BeGreaterThan(0);
        first.TargetPosition.Should().Be(first.ExpectedTargetPosition);
    }

    [Fact]
    public void MixedMode_Dynamic3DRotation_ShouldClampConservativelyWithoutExactContactWitness()
    {
        var first = RunUnresolved3DRotationalSearch();
        var second = RunUnresolved3DRotationalSearch();

        second.Should().Be(first);
        Fixed64 retainedRotation = FixedQuaternion.Angle(
            FixedQuaternion.Identity,
            first.Rotation);
        ((Fixed64)90 - retainedRotation)
            .Should()
            .BeGreaterThan(Fixed64.Zero);
        first.AngularVelocity.Should().Be(Vector3d.Zero);
        first.ToiIterations.Should().BeGreaterThan(0);
        first.TargetPosition.Should().Be(first.ExpectedTargetPosition);
    }

    [Fact]
    public void MixedMode_Dynamic3DRotation_ShouldResolveAlternatingDimensionImpactsDeterministically()
    {
        var first = RunAlternating3DRotationalContacts(registerMixedFirst: false);
        var second = RunAlternating3DRotationalContacts(registerMixedFirst: true);

        second.Should().Be(first);
        first.ToiIterations.Should().BeGreaterThanOrEqualTo(2, because: first.ToString());
    }

    [Fact]
    public void MixedMode_Dynamic2DRotation_ShouldReadmitSameDimensionAfterMixedImpact()
    {
        var first = RunAlternating2DRotationalContacts(registerMixedFirst: false);
        var second = RunAlternating2DRotationalContacts(registerMixedFirst: true);

        second.Should().Be(first);
        first.ToiIterations.Should().BeGreaterThanOrEqualTo(3, because: first.ToString());
    }

    [Fact]
    public void MixedMode_ExactRotationalTie_With3DSource_ShouldSelect2DTargetIndependentlyOfRegistrationOrder()
    {
        var first = RunKinematic3DRotationalTie(registerMixedFirst: false);
        var second = RunKinematic3DRotationalTie(registerMixedFirst: true);

        second.Should().Be(first);
        (first.Target2DVelocity.MagnitudeSquared + first.Target2DAngularVelocity.Abs())
            .Should()
            .BeGreaterThan(Fixed64.Zero, because: first.ToString());
        (first.Target3DVelocity.MagnitudeSquared + first.Target3DAngularVelocity.MagnitudeSquared)
            .Should()
            .Be(Fixed64.Zero);
    }

    [Fact]
    public void MixedMode_ExactRotationalTie_With2DSource_ShouldSelect2DTargetIndependentlyOfRegistrationOrder()
    {
        var first = RunKinematic2DRotationalTie(registerMixedFirst: false);
        var second = RunKinematic2DRotationalTie(registerMixedFirst: true);

        second.Should().Be(first);
        (first.Target2DVelocity.MagnitudeSquared + first.Target2DAngularVelocity.Abs())
            .Should()
            .BeGreaterThan(Fixed64.Zero, because: first.ToString());
        (first.Target3DVelocity.MagnitudeSquared + first.Target3DAngularVelocity.MagnitudeSquared)
            .Should()
            .Be(Fixed64.Zero);
    }

    [Fact]
    public void MixedMode_Kinematic3DRotation_ShouldSampleCombined2DTargetTrajectoryIndependentlyOfRegistrationOrder()
    {
        var first = RunCombined2DTargetTrajectory(targetFirst: false);
        var second = RunCombined2DTargetTrajectory(targetFirst: true);

        second.Should().Be(first);
        first.PreparedTranslation.Should().BeTrue();
        first.PreparedRotation.Should().BeTrue();
        first.SourceToiIterations.Should().BeGreaterThan(0);
    }

    [Fact]
    public void MixedMode_Kinematic2DRotation_ShouldSampleCombined3DTargetTrajectoryIndependentlyOfRegistrationOrder()
    {
        var first = RunCombined3DTargetTrajectory(targetFirst: false);
        var second = RunCombined3DTargetTrajectory(targetFirst: true);

        second.Should().Be(first);
        first.PreparedTranslation.Should().BeTrue();
        first.PreparedRotation.Should().BeTrue();
        first.SourceToiIterations.Should().BeGreaterThan(0);
    }

    [Fact]
    public void MixedMode_TranslationOnly3DSource_ShouldSampleIntermediateDiscreteKinematic2DTargetRotation()
    {
        var sourceFirst = RunTranslationOnly3DSourceAgainstRotating2DTarget(
            targetFirst: false);
        var targetFirst = RunTranslationOnly3DSourceAgainstRotating2DTarget(
            targetFirst: true);

        sourceFirst.Should().Be(targetFirst);
        sourceFirst.SourceToiIterations.Should().BeGreaterThan(0);
        sourceFirst.SourceLinearVelocity.Should().NotBe(
            Vector3d.Forward * Fixed64.FromFraction(1, 5));
    }

    [Fact]
    public void MixedMode_TranslationOnly2DSource_ShouldSampleIntermediateDiscreteKinematic3DTargetRotation()
    {
        var sourceFirst = RunTranslationOnly2DSourceAgainstRotating3DTarget(
            targetFirst: false);
        var targetFirst = RunTranslationOnly2DSourceAgainstRotating3DTarget(
            targetFirst: true);

        sourceFirst.Should().Be(targetFirst);
        sourceFirst.SourceToiIterations.Should().BeGreaterThan(0);
        sourceFirst.SourceLinearVelocity.Should().NotBe(
            Vector2d.Forward * Fixed64.FromFraction(1, 5));
    }

    [Fact]
    public void MixedMode_DualContinuous3DSource_ShouldSampleRotatingKinematic2DTargetIndependentlyOfRegistrationOrder()
    {
        var sourceFirst = RunTranslationOnly3DSourceAgainstRotating2DTarget(
            targetFirst: false,
            targetMode: ContinuousCollisionMode.Continuous);
        var targetFirst = RunTranslationOnly3DSourceAgainstRotating2DTarget(
            targetFirst: true,
            targetMode: ContinuousCollisionMode.Continuous);

        sourceFirst.Should().Be(targetFirst);
        sourceFirst.SourceLinearVelocity.Should().NotBe(
            Vector3d.Forward * Fixed64.FromFraction(1, 5));
    }

    [Fact]
    public void MixedMode_DualContinuous2DSource_ShouldSampleRotatingKinematic3DTargetIndependentlyOfRegistrationOrder()
    {
        var sourceFirst = RunTranslationOnly2DSourceAgainstRotating3DTarget(
            targetFirst: false,
            targetMode: ContinuousCollisionMode.Continuous);
        var targetFirst = RunTranslationOnly2DSourceAgainstRotating3DTarget(
            targetFirst: true,
            targetMode: ContinuousCollisionMode.Continuous);

        sourceFirst.Should().Be(targetFirst);
        sourceFirst.SourceLinearVelocity.Should().NotBe(
            Vector2d.Forward * Fixed64.FromFraction(1, 5));
    }

    private static (
        Fixed64 Rotation,
        Fixed64 AngularVelocity,
        int ToiIterations,
        bool LimitReached,
        Vector3d TargetPosition,
        Vector3d ExpectedTargetPosition) RunUnresolved2DRotationalSearch()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.DampingFactor = Fixed64.Zero;
        SolidBody2D blade = CreateRotationalMixedBlade2D(context);
        blade.SetMotionType(BodyMotionType.Dynamic);
        Vector3d targetPosition = Vector2d.Rotate(
                new Vector2d(Fixed64.FromFraction(16, 5), Fixed64.Zero),
                FixedMath.DegToRad((Fixed64)45))
            .ToVector3d(Fixed64.Zero);
        LSCollider target = CreateBodyless3D(
            context,
            new UnsupportedTestCollider3D(),
            targetPosition);

        blade.ApplyCollisionAngularVelocityDelta(RotationalMixedQuarterTurn);
        context.LateSimulate();

        return (
            blade.Rotation,
            blade.AngularVelocity,
            blade.LastContinuousCollisionToiIterationCount,
            blade.LastContinuousCollisionToiIterationLimitReached,
            target.Center,
            targetPosition);
    }

    private static (
        FixedQuaternion Rotation,
        Vector3d AngularVelocity,
        int ToiIterations,
        bool LimitReached,
        Vector2d TargetPosition,
        Vector2d ExpectedTargetPosition) RunUnresolved3DRotationalSearch()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.DampingFactor = Fixed64.Zero;
        var sourceCollider = new UnsupportedTestCollider3D
        {
            InertiaTensor = Fixed3x3.Identity,
            MassPropertyWeight = FixedMassWeight.One
        };
        ScenarioBody<UnsupportedTestCollider3D> blade = CreateBody3D(
            context,
            sourceCollider,
            Vector3d.Zero);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector2d targetPosition = Vector2d.Right * Fixed64.Half;
        LSCollider2D target = CreateBodylessCircle2D(context, targetPosition);

        blade.Body.AddAngularImpulse(Vector3d.Up * RotationalMixedQuarterTurn);
        context.LateSimulate();

        return (
            blade.Body.Rotation,
            blade.Body.AngularVelocity,
            blade.Body.LastContinuousCollisionToiIterationCount,
            blade.Body.LastContinuousCollisionToiIterationLimitReached,
            target.Center,
            targetPosition);
    }
}
