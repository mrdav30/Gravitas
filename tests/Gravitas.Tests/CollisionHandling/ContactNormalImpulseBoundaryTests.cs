using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Response;

public sealed class ContactNormalImpulseBoundaryTests
{
    [Fact]
    public void ResponseTransform_WithSubPrecisionNonzeroSum_ShouldRejectCompactResult()
    {
        Fixed64 coefficient = Fixed64.FromFraction(2, 5);
        var matrix = new Fixed3x3(
            coefficient, Fixed64.Zero, Fixed64.Zero,
            coefficient, Fixed64.Zero, Fixed64.Zero,
            coefficient, Fixed64.Zero, Fixed64.Zero);
        Vector3d direction = new(
            Fixed64.MinIncrement,
            Fixed64.MinIncrement,
            Fixed64.MinIncrement);

        Fixed3x3.TransformDirection(matrix, direction)
            .Should()
            .Be(Vector3d.Zero);
        Fixed3x3.TryTransformDirection(
                matrix,
                direction,
                out Vector3d exact)
            .Should()
            .BeTrue();
        exact.X.Should().Be(Fixed64.MinIncrement);

        ContactResponseArithmetic3D.TryTransformDirection(
                matrix,
                direction,
                out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void ExactUnaccumulatedKernel_ShouldHandleSeparatingAndImmovablePairs()
    {
        ExactLever3D exactParallel = CreateLever(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right * Fixed64.MinIncrement);
        ExactLever3D zero = CreateLever(Vector3d.Zero, Vector3d.Zero);

        ContactNormalImpulse3D.TryCalculateVelocityDeltasExact(
                null,
                Vector3d.Left,
                Vector3d.Zero,
                exactParallel,
                null,
                Vector3d.Zero,
                Vector3d.Zero,
                zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                out ContactNormalVelocityDeltaResult3D separating)
            .Should()
            .BeTrue();
        separating.NormalVelocity.Should().Be(Fixed64.One);
        separating.LinearVelocityDeltaA.Should().Be(Vector3d.Zero);
        separating.AngularVelocityDeltaA.Should().Be(Vector3d.Zero);
        separating.LinearVelocityDeltaB.Should().Be(Vector3d.Zero);
        separating.AngularVelocityDeltaB.Should().Be(Vector3d.Zero);

        ContactNormalImpulse3D.TryCalculateVelocityDeltasExact(
                null,
                Vector3d.Right,
                Vector3d.Zero,
                exactParallel,
                null,
                Vector3d.Zero,
                Vector3d.Zero,
                zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                out _)
            .Should()
            .BeFalse();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ExactUnaccumulatedKernel_ShouldApplyConfiguredRestitution(
        bool aboveThreshold)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body =
            scenario.CreateSphere(Vector3d.Zero);
        body.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        ExactLever3D exactParallel = CreateLever(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right * Fixed64.MinIncrement);
        ExactLever3D zero = CreateLever(Vector3d.Zero, Vector3d.Zero);

        ContactNormalImpulse3D.TryCalculateVelocityDeltasExact(
                body.Body,
                Vector3d.Right * Fixed64.Half,
                Vector3d.Zero,
                exactParallel,
                null,
                Vector3d.Zero,
                Vector3d.Zero,
                zero,
                Vector3d.Right,
                Fixed64.One,
                aboveThreshold ? Fixed64.Zero : Fixed64.One,
                out ContactNormalVelocityDeltaResult3D result)
            .Should()
            .BeTrue();

        result.LinearVelocityDeltaA.Should().Be(
            Vector3d.Left
            * (aboveThreshold ? Fixed64.One : Fixed64.Half));
        result.AngularVelocityDeltaA.Should().Be(Vector3d.Zero);
        result.LinearVelocityDeltaB.Should().Be(Vector3d.Zero);
        result.AngularVelocityDeltaB.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void ExactUnaccumulatedKernel_ShouldNarrowOnlyFinalEffectiveMassResponse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(
            Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(
            Vector3d.Right);
        ScenarioBody<LSSphereCollider> ordinary =
            scenario.CreateSphere(Vector3d.Up);
        ScenarioBody<LSSphereCollider> ordinarySecond =
            scenario.CreateSphere(Vector3d.Forward);
        ExactLever3D exactParallel = CreateLever(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right * Fixed64.MinIncrement);
        ExactLever3D exactParallelWithUnitOffset = CreateLever(
            new Vector3d(Fixed64.MaxValue, Fixed64.One, Fixed64.Zero),
            Vector3d.Right * Fixed64.MinIncrement);
        ExactLever3D hugePerpendicular = CreateLever(
            new Vector3d(Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero),
            Vector3d.Up * Fixed64.MinIncrement);
        ExactLever3D zero = CreateLever(Vector3d.Zero, Vector3d.Zero);

        first.Body.Mass = Fixed64.MinIncrement;
        second.Body.Mass = Fixed64.MinIncrement;
        first.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        second.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        ContactNormalImpulse3D.TryCalculateVelocityDeltasExact(
                first.Body,
                Vector3d.Right,
                Vector3d.Zero,
                exactParallel,
                second.Body,
                Vector3d.Zero,
                Vector3d.Zero,
                exactParallel,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                out ContactNormalVelocityDeltaResult3D equalMass)
            .Should()
            .BeTrue();
        equalMass.LinearVelocityDeltaA.Should().Be(
            Vector3d.Left * Fixed64.Half);
        equalMass.AngularVelocityDeltaA.Should().Be(Vector3d.Zero);
        equalMass.LinearVelocityDeltaB.Should().Be(
            Vector3d.Right * Fixed64.Half);
        equalMass.AngularVelocityDeltaB.Should().Be(Vector3d.Zero);

        ordinary.Body.Mass = Fixed64.MinIncrement;
        ordinarySecond.Body.Mass = Fixed64.MinIncrement;
        ContactNormalImpulse3D.TryCalculateVelocityDeltasExact(
                ordinary.Body,
                Vector3d.Right,
                Vector3d.Zero,
                exactParallelWithUnitOffset,
                null,
                Vector3d.Zero,
                Vector3d.Zero,
                zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                out ContactNormalVelocityDeltaResult3D bodyAOnly)
            .Should()
            .BeTrue();
        bodyAOnly.LinearVelocityDeltaA.X.Should()
            .BeLessThan(-Fixed64.Half)
            .And.BeGreaterThan(-Fixed64.One);
        bodyAOnly.AngularVelocityDeltaA.Z.Should()
            .BeGreaterThan(Fixed64.Zero);
        ContactNormalImpulse3D.TryCalculateVelocityDeltasExact(
                null,
                Vector3d.Right,
                Vector3d.Zero,
                zero,
                ordinarySecond.Body,
                Vector3d.Zero,
                Vector3d.Zero,
                exactParallelWithUnitOffset,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                out ContactNormalVelocityDeltaResult3D bodyBOnly)
            .Should()
            .BeTrue();
        bodyBOnly.LinearVelocityDeltaB.X.Should()
            .BeGreaterThan(Fixed64.Half)
            .And.BeLessThan(Fixed64.One);
        bodyBOnly.AngularVelocityDeltaB.Z.Should()
            .BeLessThan(Fixed64.Zero);
        ordinary.Body.Mass = Fixed64.One;
    }

    [Fact]
    public void ExactUnaccumulatedKernel_ShouldRetainWidePointSpeedAndEffectiveMass()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        SolidBody body = CreateBodyWithInertia(
            scenario,
            Fixed3x3.Identity);
        body.FreezeAxes = BodyFreezeAxes3D.Position;
        ExactLever3D hugePerpendicular = CreateLever(
            new Vector3d(Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero));
        ExactLever3D zero = CreateLever(Vector3d.Zero, Vector3d.Zero);

        ContactNormalImpulse3D.TryCalculateVelocityDeltasExact(
                body,
                Vector3d.Zero,
                -Vector3d.Forward * Fixed64.Two,
                hugePerpendicular,
                null,
                Vector3d.Zero,
                Vector3d.Zero,
                zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                out ContactNormalVelocityDeltaResult3D result)
            .Should()
            .BeTrue();

        result.HasRepresentableNormalVelocity.Should().BeFalse();
        result.IsClosing.Should().BeTrue();
        result.LinearVelocityDeltaA.Should().Be(Vector3d.Zero);
        result.AngularVelocityDeltaA.Should()
            .Be(Vector3d.Forward * Fixed64.Two);
        result.LinearVelocityDeltaB.Should().Be(Vector3d.Zero);
        result.AngularVelocityDeltaB.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void ExactAccumulatedKernel_ShouldApplyWideSharedImpulseToRepresentableDeltas()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(
            Vector3d.Zero,
            mass: Fixed64.MaxValue);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(
            Vector3d.Right,
            mass: Fixed64.MaxValue);
        first.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        second.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        first.Body.InverseMass.Should().Be(Fixed64.MinIncrement * Fixed64.Two);
        second.Body.InverseMass.Should().Be(Fixed64.MinIncrement * Fixed64.Two);
        ExactLever3D zero = CreateLever(Vector3d.Zero, Vector3d.Zero);

        ContactNormalImpulse3D.TryCalculateAccumulatedDeltaExact(
                first.Body,
                Vector3d.Right * (Fixed64)6,
                Vector3d.Zero,
                zero,
                second.Body,
                Vector3d.Zero,
                Vector3d.Zero,
                zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                out ContactNormalImpulseResult3D result)
            .Should()
            .BeTrue();

        result.ImpulseScalar.Should().Be(Fixed64.Zero);
        result.AppliedImpulseScalar.Should().Be(Fixed64.Zero);
        result.HasRepresentableAppliedImpulse.Should().BeFalse();
        result.LinearVelocityDeltaA.Should()
            .Be(Vector3d.Left * (Fixed64)3);
        result.LinearVelocityDeltaB.Should()
            .Be(Vector3d.Right * (Fixed64)3);
        result.AngularVelocityDeltaA.Should().Be(Vector3d.Zero);
        result.AngularVelocityDeltaB.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void ExactUnaccumulatedKernel_ShouldRejectUnrepresentableFinalVelocity()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> bodyA = scenario.CreateSphere(
            Vector3d.Zero,
            mass: Fixed64.MaxValue);
        ScenarioBody<LSSphereCollider> bodyB = scenario.CreateSphere(
            Vector3d.Right,
            mass: Fixed64.MaxValue);
        ExactLever3D exactParallel = CreateLever(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right * Fixed64.MinIncrement);
        ExactLever3D zero = CreateLever(Vector3d.Zero, Vector3d.Zero);

        ContactNormalImpulse3D.TryCalculateVelocityDeltasExact(
                bodyA.Body,
                Vector3d.Right * Fixed64.MaxValue,
                Vector3d.Zero,
                exactParallel,
                null,
                Vector3d.Zero,
                Vector3d.Zero,
                zero,
                Vector3d.Right,
                Fixed64.One,
                Fixed64.Zero,
                out _)
            .Should()
            .BeFalse();
        ContactNormalImpulse3D.TryCalculateVelocityDeltasExact(
                null,
                Vector3d.Zero,
                Vector3d.Zero,
                zero,
                bodyB.Body,
                Vector3d.Left * Fixed64.MaxValue,
                Vector3d.Zero,
                exactParallel,
                Vector3d.Right,
                Fixed64.One,
                Fixed64.Zero,
                out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void ExactAccumulatedKernel_ShouldReturnZeroWhenNoImpulseCanBeApplied()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body =
            scenario.CreateSphere(Vector3d.Zero);
        body.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        ExactLever3D exactParallel = CreateLever(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right * Fixed64.MinIncrement);
        ExactLever3D zero = CreateLever(Vector3d.Zero, Vector3d.Zero);

        ContactNormalImpulse3D.TryCalculateAccumulatedDeltaExact(
                null,
                Vector3d.Right,
                Vector3d.Zero,
                exactParallel,
                null,
                Vector3d.Zero,
                Vector3d.Zero,
                zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                out ContactNormalImpulseResult3D immovable)
            .Should()
            .BeTrue();
        immovable.NormalVelocity.Should().Be(-Fixed64.One);
        immovable.ImpulseScalar.Should().Be(Fixed64.Zero);
        immovable.LinearVelocityDeltaA.Should().Be(Vector3d.Zero);
        immovable.AngularVelocityDeltaA.Should().Be(Vector3d.Zero);
        immovable.LinearVelocityDeltaB.Should().Be(Vector3d.Zero);
        immovable.AngularVelocityDeltaB.Should().Be(Vector3d.Zero);

        ContactNormalImpulse3D.TryCalculateAccumulatedDeltaExact(
                body.Body,
                Vector3d.Left,
                Vector3d.Zero,
                exactParallel,
                null,
                Vector3d.Zero,
                Vector3d.Zero,
                zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                out ContactNormalImpulseResult3D separating)
            .Should()
            .BeTrue();
        separating.NormalVelocity.Should().Be(Fixed64.One);
        separating.ImpulseScalar.Should().Be(Fixed64.Zero);
        separating.LinearVelocityDeltaA.Should().Be(Vector3d.Zero);
        separating.AngularVelocityDeltaA.Should().Be(Vector3d.Zero);
        separating.LinearVelocityDeltaB.Should().Be(Vector3d.Zero);
        separating.AngularVelocityDeltaB.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void ExactAccumulatedKernel_ShouldRejectNegativeAccumulator()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> bodyA =
            scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> bodyB =
            scenario.CreateSphere(Vector3d.Right);
        ExactLever3D exactParallelWithUnitOffset = CreateLever(
            new Vector3d(Fixed64.MaxValue, Fixed64.One, Fixed64.Zero),
            Vector3d.Right * Fixed64.MinIncrement);
        ExactLever3D zero = CreateLever(Vector3d.Zero, Vector3d.Zero);

        AssertAccumulatedUnresolved(
            bodyA.Body,
            Vector3d.Left,
            Vector3d.Zero,
            exactParallelWithUnitOffset,
            null,
            Vector3d.Zero,
            Vector3d.Zero,
            zero,
            Fixed64.MinValue);
        AssertAccumulatedUnresolved(
            null,
            Vector3d.Zero,
            Vector3d.Zero,
            zero,
            bodyB.Body,
            Vector3d.Right,
            Vector3d.Zero,
            exactParallelWithUnitOffset,
            Fixed64.MinValue);
    }

    [Fact]
    public void ExactAccumulatedKernel_ShouldRejectUnrepresentableLinearVelocityDelta()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body =
            scenario.CreateSphere(Vector3d.Zero);
        SolidBody2D body2D =
            CreateBody2D(scenario.Context, Fixed64.MinIncrement);
        body.Body.Mass = Fixed64.MinIncrement;
        body.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        body2D.FreezeAxes = BodyFreezeAxes2D.Rotation;
        ExactLever3D exactParallel = CreateLever(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right * Fixed64.MinIncrement);
        ExactLever3D zero = CreateLever(Vector3d.Zero, Vector3d.Zero);

        ContactNormalImpulse3D.TryCalculateAccumulatedDeltaExact(
                body.Body,
                Vector3d.Right * Fixed64.MaxValue,
                Vector3d.Zero,
                exactParallel,
                null,
                Vector3d.Zero,
                Vector3d.Zero,
                zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.MaxValue,
                Fixed64.One,
                out _)
            .Should()
            .BeFalse();
        ContactNormalImpulse2D.TryCalculateAccumulatedDeltaExact(
                body2D,
                Vector2d.Right * Fixed64.MaxValue,
                Fixed64.Zero,
                exactParallel,
                null,
                Vector2d.Zero,
                Fixed64.Zero,
                zero,
                Vector2d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.MaxValue,
                Fixed64.One,
                out _)
            .Should()
            .BeFalse();
        ContactNormalImpulseMixed.TryCalculateAccumulatedDeltaExact(
                body.Body,
                Vector3d.Right * Fixed64.MaxValue,
                Vector3d.Zero,
                exactParallel,
                null,
                Vector2d.Zero,
                Fixed64.Zero,
                zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.MaxValue,
                Fixed64.One,
                out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void CompactAccumulatedKernel_ShouldRejectUnrepresentableLinearVelocityDelta()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body =
            scenario.CreateSphere(Vector3d.Zero);
        SolidBody2D body2D =
            CreateBody2D(scenario.Context, Fixed64.MinIncrement);
        body.Body.Mass = Fixed64.MinIncrement;
        body.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        body2D.FreezeAxes = BodyFreezeAxes2D.Rotation;

        ContactNormalImpulse3D.TryCalculateAccumulatedDelta(
                body.Body,
                Vector3d.Right * Fixed64.MaxValue,
                Vector3d.Zero,
                Vector3d.Zero,
                null,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.MaxValue,
                Fixed64.One,
                out _)
            .Should()
            .BeFalse();
        ContactNormalImpulse2D.TryCalculateAccumulatedDelta(
                body2D,
                Vector2d.Right * Fixed64.MaxValue,
                Fixed64.Zero,
                Vector2d.Zero,
                null,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.MaxValue,
                Fixed64.One,
                out _)
            .Should()
            .BeFalse();
        ContactNormalImpulseMixed.TryCalculateAccumulatedDelta(
                body.Body,
                Vector3d.Right * Fixed64.MaxValue,
                Vector3d.Zero,
                Vector3d.Zero,
                null,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.MaxValue,
                Fixed64.One,
                out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void AccumulatedKernels_ShouldHandleUnrepresentableCacheGrowthByDomain()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body =
            scenario.CreateSphere(Vector3d.Zero);
        SolidBody2D body2D =
            CreateBody2D(scenario.Context, Fixed64.One);
        body.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        body2D.FreezeAxes = BodyFreezeAxes2D.Rotation;
        ExactLever3D zero = CreateLever(Vector3d.Zero, Vector3d.Zero);

        ContactNormalImpulse3D.TryCalculateAccumulatedDelta(
                body.Body,
                Vector3d.Right,
                Vector3d.Zero,
                Vector3d.Zero,
                null,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.MaxValue,
                Fixed64.One,
                Fixed64.One,
                out _)
            .Should()
            .BeFalse();
        ContactNormalImpulse3D.TryCalculateAccumulatedDeltaExact(
                body.Body,
                Vector3d.Right,
                Vector3d.Zero,
                zero,
                null,
                Vector3d.Zero,
                Vector3d.Zero,
                zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.MaxValue,
                Fixed64.One,
                Fixed64.One,
                out ContactNormalImpulseResult3D exact)
            .Should()
            .BeTrue();
        exact.ImpulseScalar.Should().Be(-Fixed64.MaxValue);
        exact.AppliedImpulseScalar.Should().Be(Fixed64.One);
        exact.LinearVelocityDeltaA.Should().Be(Vector3d.Left);

        ContactNormalImpulse2D.TryCalculateAccumulatedDelta(
                body2D,
                Vector2d.Right,
                Fixed64.Zero,
                Vector2d.Zero,
                null,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.MaxValue,
                Fixed64.One,
                Fixed64.One,
                out _)
            .Should()
            .BeFalse();
        ContactNormalImpulse2D.TryCalculateAccumulatedDeltaExact(
                body2D,
                Vector2d.Right,
                Fixed64.Zero,
                zero,
                null,
                Vector2d.Zero,
                Fixed64.Zero,
                zero,
                Vector2d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.MaxValue,
                Fixed64.One,
                Fixed64.One,
                out ContactNormalImpulseResult2D exact2D)
            .Should()
            .BeTrue();
        exact2D.HasRepresentableAccumulatedImpulse.Should().BeFalse();
        exact2D.AppliedImpulseScalar.Should().Be(Fixed64.One);
        exact2D.LinearVelocityDeltaA.Should().Be(Vector2d.Left);

        ContactNormalImpulseMixed.TryCalculateAccumulatedDelta(
                body.Body,
                Vector3d.Right,
                Vector3d.Zero,
                Vector3d.Zero,
                null,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.MaxValue,
                Fixed64.One,
                Fixed64.One,
                out _)
            .Should()
            .BeFalse();
        ContactNormalImpulseMixed.TryCalculateAccumulatedDeltaExact(
                body.Body,
                Vector3d.Right,
                Vector3d.Zero,
                zero,
                null,
                Vector2d.Zero,
                Fixed64.Zero,
                zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.MaxValue,
                Fixed64.One,
                Fixed64.One,
                out ContactNormalImpulseResultMixed exactMixed)
            .Should()
            .BeTrue();
        exactMixed.HasRepresentableAppliedImpulse.Should().BeTrue();
        exactMixed.AppliedImpulseScalar.Should().Be(Fixed64.One);
        exactMixed.LinearVelocityDelta3D.Should().Be(Vector3d.Left);
    }

    [Fact]
    public void CompactKernels_ShouldRejectUnrepresentablePointVelocityIntermediates()
    {
        AssertCompactUnaccumulatedUnresolved(
            null,
            Vector3d.Zero,
            Vector3d.Forward * Fixed64.MaxValue,
            Vector3d.Up * Fixed64.MaxValue,
            null,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Right);
        AssertCompactUnaccumulatedUnresolved(
            null,
            Vector3d.Right * Fixed64.MaxValue,
            Vector3d.Backward * Fixed64.MaxValue,
            Vector3d.Up,
            null,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Right);
        AssertCompactUnaccumulatedUnresolved(
            null,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            null,
            Vector3d.Right * Fixed64.MaxValue,
            Vector3d.Backward * Fixed64.MaxValue,
            Vector3d.Up,
            Vector3d.Right);
        AssertCompactUnaccumulatedUnresolved(
            null,
            new Vector3d(
                Fixed64.MinValue,
                Fixed64.Zero,
                Fixed64.Zero),
            Vector3d.Zero,
            Vector3d.Zero,
            null,
            Vector3d.Right * Fixed64.MaxValue,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Right);
        AssertCompactAccumulatedUnresolved(
            null,
            Vector3d.Zero,
            Vector3d.Forward * Fixed64.MaxValue,
            Vector3d.Up * Fixed64.MaxValue,
            null,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.Zero);

        ContactNormalImpulse2D.TryCalculateVelocityDeltas(
                null,
                Vector2d.Zero,
                Fixed64.MaxValue,
                Vector2d.Forward * Fixed64.MaxValue,
                null,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                out _)
            .Should()
            .BeFalse();
        ContactNormalImpulse2D.TryCalculateAccumulatedDelta(
                null,
                Vector2d.Zero,
                Fixed64.MaxValue,
                Vector2d.Forward * Fixed64.MaxValue,
                null,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                out _)
            .Should()
            .BeFalse();
        ContactNormalImpulseMixed.TryCalculateVelocityDeltas(
                null,
                Vector3d.Zero,
                Vector3d.Forward * Fixed64.MaxValue,
                Vector3d.Up * Fixed64.MaxValue,
                null,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                out _)
            .Should()
            .BeFalse();
        ContactNormalImpulseMixed.TryCalculateAccumulatedDelta(
                null,
                Vector3d.Zero,
                Vector3d.Forward * Fixed64.MaxValue,
                Vector3d.Up * Fixed64.MaxValue,
                null,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void CompactPointVelocity_ShouldRejectSubprecisionAngularContribution()
    {
        ContactResponseArithmetic3D.TryGetRelativePointVelocity(
                Vector3d.Zero,
                Vector3d.Right * Fixed64.MinIncrement,
                Vector3d.Up * Fixed64.Half,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Right,
                out Vector3d relativeVelocity)
            .Should()
            .BeFalse();

        relativeVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void CompactKernels_ShouldNarrowOnlyFinalEffectiveMassResponse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first =
            scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second =
            scenario.CreateSphere(Vector3d.Right);
        first.Body.Mass = Fixed64.MinIncrement;
        second.Body.Mass = Fixed64.MinIncrement;
        first.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        second.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;

        ContactNormalImpulse3D.TryCalculateVelocityDeltas(
                first.Body,
                Vector3d.Right,
                Vector3d.Zero,
                Vector3d.Zero,
                second.Body,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                out ContactNormalVelocityDeltaResult3D unaccumulated)
            .Should()
            .BeTrue();
        unaccumulated.LinearVelocityDeltaA.Should().Be(
            Vector3d.Left * Fixed64.Half);
        unaccumulated.LinearVelocityDeltaB.Should().Be(
            Vector3d.Right * Fixed64.Half);
        ContactNormalImpulse3D.TryCalculateAccumulatedDelta(
                first.Body,
                Vector3d.Right,
                Vector3d.Zero,
                Vector3d.Zero,
                second.Body,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                out ContactNormalImpulseResult3D accumulated)
            .Should()
            .BeTrue();
        accumulated.LinearVelocityDeltaA.Should().Be(
            Vector3d.Left * Fixed64.Half);
        accumulated.LinearVelocityDeltaB.Should().Be(
            Vector3d.Right * Fixed64.Half);
    }

    [Fact]
    public void CompactKernels_ShouldRejectUnrepresentableAngularProducts()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        SolidBody identityInertia = CreateBodyWithInertia(
            scenario,
            Fixed3x3.Identity);
        SolidBody maximumInverseInertia = CreateBodyWithInertia(
            scenario,
            new Fixed3x3(
                Fixed64.MinIncrement, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.MinIncrement, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.MinIncrement));
        Vector3d diagonalAxis =
            new Vector3d(Fixed64.Zero, Fixed64.One, -Fixed64.One)
                .Normalized;

        AssertCompactUnaccumulatedUnresolved(
            identityInertia,
            diagonalAxis,
            Vector3d.Zero,
            new Vector3d(
                Fixed64.Zero,
                Fixed64.MaxValue,
                Fixed64.MaxValue),
            null,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            diagonalAxis);
        AssertCompactUnaccumulatedUnresolved(
            maximumInverseInertia,
            Vector3d.Forward,
            Vector3d.Zero,
            Vector3d.Up * Fixed64.Two,
            null,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Forward);
        AssertCompactUnaccumulatedUnresolved(
            identityInertia,
            Vector3d.Forward,
            Vector3d.Zero,
            Vector3d.Up * Fixed64.MaxValue,
            null,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Forward);
        AssertCompactUnaccumulatedUnresolved(
            identityInertia,
            Vector3d.One.Normalized,
            Vector3d.Zero,
            new Vector3d(40000, -40000, 0),
            null,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.One.Normalized);

        AssertCompactAccumulatedUnresolved(
            identityInertia,
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Up * Fixed64.Two,
            null,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.MinValue);
        AssertCompactAccumulatedUnresolved(
            maximumInverseInertia,
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Up
                * (Fixed64.MinIncrement * (Fixed64)3),
            null,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.MinValue);
    }

    [Fact]
    public void CompactUnaccumulatedKernel_ShouldPreserveCheckedAngularCancellation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Right,
            Fixed64.PiOver4);
        var collider = new UnsupportedTestCollider3D
        {
            InertiaTensor = new Fixed3x3(
                Fixed64.One, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.Quarter)
        };
        SolidBody body = scenario.CreateBody(
            collider,
            Vector3d.Zero,
            rotation).Body;
        Vector3d normal = Vector3d.Right;
        Vector3d torque = (Vector3d.Up + Vector3d.Backward).Normalized
            * (Fixed64)1_500_000_000;
        Vector3d lever = Vector3d.Cross(normal, torque);
        Fixed3x3 inverseInertia =
            body.GetConstrainedInverseInertiaTensor();

        Vector3d.TryCross(lever, normal, out Vector3d denominatorTorque)
            .Should()
            .BeTrue("the compact lever and unit normal have a representable cross product");
        Fixed3x3.TryTransformDirection(
                inverseInertia,
                denominatorTorque,
                out Vector3d denominatorResponse)
            .Should()
            .BeTrue(
                "the exact matrix-vector sum cancels into the scalar domain; inverse inertia {0}, torque {1}",
                inverseInertia,
                denominatorTorque);
        Vector3d.TryCross(
                denominatorResponse,
                lever,
                out Vector3d angular)
            .Should()
            .BeTrue("the checked angular denominator cross product is representable");
        Vector3d.TryDot(angular, normal, out Fixed64 angularDenominator)
            .Should()
            .BeTrue("the checked angular denominator is representable");
        Fixed64.TryAdd(
                body.GetConstrainedInverseMass(normal),
                angularDenominator,
                out Fixed64 denominator)
            .Should()
            .BeTrue("the complete effective mass is representable");
        Vector3d.TryCross(lever, -normal, out Vector3d responseTorque)
            .Should()
            .BeTrue("the signed response torque is representable");
        ContactResponseArithmetic3D.TryCross(
                lever,
                -normal,
                out responseTorque)
            .Should()
            .BeTrue("the response arithmetic keeps the checked torque");
        ContactResponseArithmetic3D.TryTransformDirection(
                inverseInertia,
                responseTorque,
                out Vector3d response)
            .Should()
            .BeTrue("the signed matrix-vector sum cancels into the scalar domain");
        Fixed64.TryMultiplyDivide(
                response.X,
                -Fixed64.One,
                -Fixed64.One,
                denominator,
                out Fixed64 expectedX)
            .Should()
            .BeTrue("the final X response is representable");
        Fixed64.TryMultiplyDivide(
                response.Y,
                -Fixed64.One,
                -Fixed64.One,
                denominator,
                out Fixed64 expectedY)
            .Should()
            .BeTrue("the final Y response is representable");
        Fixed64.TryMultiplyDivide(
                response.Z,
                -Fixed64.One,
                -Fixed64.One,
                denominator,
                out Fixed64 expectedZ)
            .Should()
            .BeTrue("the final Z response is representable");

        ContactNormalImpulse3D.TryCalculateVelocityDeltas(
                body,
                normal,
                Vector3d.Zero,
                lever,
                null,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Zero,
                normal,
                Fixed64.Zero,
                Fixed64.Zero,
                out ContactNormalVelocityDeltaResult3D result)
            .Should()
            .BeTrue("the checked compact kernel has a representable final response");

        result.AngularVelocityDeltaA.Should().Be(
            new Vector3d(expectedX, expectedY, expectedZ));
    }

    [Fact]
    public void UnaccumulatedKernels_ShouldReturnZeroForSeparatingImmovablePairs()
    {
        ContactNormalImpulse3D.TryCalculateVelocityDeltas(
                null,
                Vector3d.Left,
                Vector3d.Zero,
                Vector3d.Zero,
                null,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                out ContactNormalVelocityDeltaResult3D separating3D)
            .Should()
            .BeTrue();
        separating3D.NormalVelocity.Should().Be(Fixed64.One);
        separating3D.LinearVelocityDeltaA.Should().Be(Vector3d.Zero);
        separating3D.AngularVelocityDeltaA.Should().Be(Vector3d.Zero);
        separating3D.LinearVelocityDeltaB.Should().Be(Vector3d.Zero);
        separating3D.AngularVelocityDeltaB.Should().Be(Vector3d.Zero);

        ContactNormalImpulse2D.TryCalculateVelocityDeltas(
                null,
                Vector2d.Left,
                Fixed64.Zero,
                Vector2d.Zero,
                null,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                out ContactNormalVelocityDeltaResult2D separating2D)
            .Should()
            .BeTrue();
        separating2D.NormalVelocity.Should().Be(Fixed64.One);
        separating2D.LinearVelocityDeltaA.Should().Be(Vector2d.Zero);
        separating2D.AngularVelocityDeltaA.Should().Be(Fixed64.Zero);
        separating2D.LinearVelocityDeltaB.Should().Be(Vector2d.Zero);
        separating2D.AngularVelocityDeltaB.Should().Be(Fixed64.Zero);

        ContactNormalImpulseMixed.TryCalculateVelocityDeltas(
                null,
                Vector3d.Left,
                Vector3d.Zero,
                Vector3d.Zero,
                null,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                out ContactNormalVelocityDeltaResultMixed separatingMixed)
            .Should()
            .BeTrue();
        separatingMixed.NormalVelocity.Should().Be(Fixed64.One);
        separatingMixed.LinearVelocityDelta3D.Should().Be(Vector3d.Zero);
        separatingMixed.AngularVelocityDelta3D.Should().Be(Vector3d.Zero);
        separatingMixed.LinearVelocityDelta2D.Should().Be(Vector2d.Zero);
        separatingMixed.AngularVelocityDelta2D.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void UnaccumulatedKernels_ShouldRejectClosingImmovablePairs()
    {
        ContactNormalImpulse3D.TryCalculateVelocityDeltas(
                null,
                Vector3d.Right,
                Vector3d.Zero,
                Vector3d.Zero,
                null,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                out _)
            .Should()
            .BeFalse();
        ContactNormalImpulse2D.TryCalculateVelocityDeltas(
                null,
                Vector2d.Right,
                Fixed64.Zero,
                Vector2d.Zero,
                null,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                out _)
            .Should()
            .BeFalse();
        ContactNormalImpulseMixed.TryCalculateVelocityDeltas(
                null,
                Vector3d.Right,
                Vector3d.Zero,
                Vector3d.Zero,
                null,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void UnaccumulatedKernels_ShouldSuppressRestitutionBelowVelocityThreshold()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body3D = scenario.CreateSphere(Vector3d.Zero);
        SolidBody2D body2D = CreateBody2D(scenario.Context, Fixed64.One);

        ContactNormalImpulse3D.TryCalculateVelocityDeltas(
                body3D.Body,
                Vector3d.Right * Fixed64.Half,
                Vector3d.Zero,
                Vector3d.Zero,
                null,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Right,
                Fixed64.One,
                Fixed64.One,
                out ContactNormalVelocityDeltaResult3D result3D)
            .Should()
            .BeTrue();
        result3D.LinearVelocityDeltaA.Should().Be(Vector3d.Left * Fixed64.Half);

        ContactNormalImpulse2D.TryCalculateVelocityDeltas(
                body2D,
                Vector2d.Right * Fixed64.Half,
                Fixed64.Zero,
                Vector2d.Zero,
                null,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.One,
                Fixed64.One,
                out ContactNormalVelocityDeltaResult2D result2D)
            .Should()
            .BeTrue();
        result2D.LinearVelocityDeltaA.Should().Be(Vector2d.Left * Fixed64.Half);

        ContactNormalImpulseMixed.TryCalculateVelocityDeltas(
                body3D.Body,
                Vector3d.Right * Fixed64.Half,
                Vector3d.Zero,
                Vector3d.Zero,
                null,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector3d.Right,
                Fixed64.One,
                Fixed64.One,
                out ContactNormalVelocityDeltaResultMixed resultMixed)
            .Should()
            .BeTrue();
        resultMixed.LinearVelocityDelta3D.Should().Be(Vector3d.Left * Fixed64.Half);
    }

    [Fact]
    public void UnaccumulatedKernels_ShouldRejectUnrepresentableBounceAtomically()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body3D = scenario.CreateSphere(
            Vector3d.Zero,
            mass: Fixed64.MaxValue);
        SolidBody2D body2D = CreateBody2D(scenario.Context, Fixed64.MaxValue);

        ContactNormalImpulse3D.TryCalculateVelocityDeltas(
                body3D.Body,
                Vector3d.Right * Fixed64.MaxValue,
                Vector3d.Zero,
                Vector3d.Zero,
                null,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Right,
                Fixed64.One,
                Fixed64.Zero,
                out ContactNormalVelocityDeltaResult3D result3D)
            .Should()
            .BeFalse();
        result3D.Should().Be(default(ContactNormalVelocityDeltaResult3D));

        ContactNormalImpulse3D.TryCalculateVelocityDeltas(
                null,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Zero,
                body3D.Body,
                Vector3d.Left * Fixed64.MaxValue,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Right,
                Fixed64.One,
                Fixed64.Zero,
                out result3D)
            .Should()
            .BeFalse();
        result3D.Should().Be(default(ContactNormalVelocityDeltaResult3D));

        ContactNormalImpulse2D.TryCalculateVelocityDeltas(
                body2D,
                Vector2d.Right * Fixed64.MaxValue,
                Fixed64.Zero,
                Vector2d.Zero,
                null,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.One,
                Fixed64.Zero,
                out ContactNormalVelocityDeltaResult2D result2D)
            .Should()
            .BeFalse();
        result2D.Should().Be(default(ContactNormalVelocityDeltaResult2D));

        ContactNormalImpulseMixed.TryCalculateVelocityDeltas(
                body3D.Body,
                Vector3d.Right * Fixed64.MaxValue,
                Vector3d.Zero,
                Vector3d.Zero,
                null,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector3d.Right,
                Fixed64.One,
                Fixed64.Zero,
                out ContactNormalVelocityDeltaResultMixed resultMixed)
            .Should()
            .BeFalse();
        resultMixed.Should().Be(default(ContactNormalVelocityDeltaResultMixed));
    }

    [Fact]
    public void AccumulatedKernels_ShouldRejectUnrepresentableCompactImpulse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 largeFiniteMass = (Fixed64)1_000_000;
        ScenarioBody<LSSphereCollider> body3D = scenario.CreateSphere(
            Vector3d.Zero,
            mass: largeFiniteMass);
        ScenarioBody<LSSphereCollider> target3D = scenario.CreateSphere(
            Vector3d.Right * Fixed64.Two,
            mass: largeFiniteMass);
        SolidBody2D body2D = CreateBody2D(scenario.Context, largeFiniteMass);
        SolidBody2D target2D = CreateBody2D(scenario.Context, largeFiniteMass);

        ContactNormalImpulse3D.TryCalculateAccumulatedDelta(
                body3D.Body,
                Vector3d.Right * Fixed64.MaxValue,
                Vector3d.Zero,
                Vector3d.Zero,
                target3D.Body,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Right,
                Fixed64.One,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                out ContactNormalImpulseResult3D closing3D)
            .Should()
            .BeFalse();
        ContactNormalImpulse3D.TryCalculateAccumulatedDelta(
                body3D.Body,
                Vector3d.Left * Fixed64.MaxValue,
                Vector3d.Zero,
                Vector3d.Zero,
                target3D.Body,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Right,
                Fixed64.One,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                out ContactNormalImpulseResult3D separating3D)
            .Should()
            .BeTrue();
        closing3D.Should().Be(default(ContactNormalImpulseResult3D));
        separating3D.ImpulseScalar.Should().Be(Fixed64.Zero);

        ContactNormalImpulse2D.TryCalculateAccumulatedDelta(
                body2D,
                Vector2d.Right * Fixed64.MaxValue,
                Fixed64.Zero,
                Vector2d.Zero,
                target2D,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.One,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                out ContactNormalImpulseResult2D closing2D)
            .Should()
            .BeFalse();
        ContactNormalImpulse2D.TryCalculateAccumulatedDelta(
                body2D,
                Vector2d.Left * Fixed64.MaxValue,
                Fixed64.Zero,
                Vector2d.Zero,
                target2D,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.One,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                out ContactNormalImpulseResult2D separating2D)
            .Should()
            .BeTrue();
        closing2D.Should().Be(default(ContactNormalImpulseResult2D));
        separating2D.ImpulseScalar.Should().Be(Fixed64.Zero);

        ContactNormalImpulseMixed.TryCalculateAccumulatedDelta(
                body3D.Body,
                Vector3d.Right * Fixed64.MaxValue,
                Vector3d.Zero,
                Vector3d.Zero,
                body2D,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector3d.Right,
                Fixed64.One,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                out ContactNormalImpulseResultMixed closingMixed)
            .Should()
            .BeFalse();
        ContactNormalImpulseMixed.TryCalculateAccumulatedDelta(
                body3D.Body,
                Vector3d.Left * Fixed64.MaxValue,
                Vector3d.Zero,
                Vector3d.Zero,
                body2D,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector3d.Right,
                Fixed64.One,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                out ContactNormalImpulseResultMixed separatingMixed)
            .Should()
            .BeTrue();
        closingMixed.Should().Be(default(ContactNormalImpulseResultMixed));
        separatingMixed.ImpulseScalar.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void AccumulatedKernels_ShouldLeaveBodylessParticipantsImmovable()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body3D = scenario.CreateSphere(
            Vector3d.Zero,
            mass: Fixed64.One);
        SolidBody2D body2D = CreateBody2D(scenario.Context, Fixed64.One);

        ContactNormalImpulseResult3D result3D = ContactNormalImpulse3D.CalculateAccumulatedDelta(
            null,
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Zero,
            body3D.Body,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One);
        result3D.ImpulseScalar.Should().BeGreaterThan(Fixed64.Zero);
        result3D.LinearVelocityDeltaA.Should().Be(Vector3d.Zero);
        result3D.AngularVelocityDeltaA.Should().Be(Vector3d.Zero);
        result3D.LinearVelocityDeltaB.Should().NotBe(Vector3d.Zero);

        ContactNormalImpulse2D.TryCalculateAccumulatedDelta(
                null,
                Vector2d.Right,
                Fixed64.Zero,
                Vector2d.Zero,
                body2D,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                out ContactNormalImpulseResult2D result2D)
            .Should()
            .BeTrue();
        result2D.ImpulseScalar.Should().BeGreaterThan(Fixed64.Zero);
        result2D.LinearVelocityDeltaA.Should().Be(Vector2d.Zero);
        result2D.AngularVelocityDeltaA.Should().Be(Fixed64.Zero);
        result2D.LinearVelocityDeltaB.Should().NotBe(Vector2d.Zero);

        ContactNormalImpulse2D.TryCalculateVelocityDeltas(
                null,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                body2D,
                Vector2d.Left,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                out ContactNormalVelocityDeltaResult2D
                    unaccumulated2D)
            .Should()
            .BeTrue();
        unaccumulated2D.LinearVelocityDeltaA.Should()
            .Be(Vector2d.Zero);
        unaccumulated2D.AngularVelocityDeltaA.Should()
            .Be(Fixed64.Zero);
        unaccumulated2D.LinearVelocityDeltaB.X.Should()
            .BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void CompactPlanarKernels_ShouldRejectUnrepresentableEffectiveMassSum()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body3D =
            scenario.CreateSphere(Vector3d.Zero);
        SolidBody2D first2D =
            CreateBody2D(scenario.Context, Fixed64.MinIncrement);
        SolidBody2D second2D =
            CreateBody2D(scenario.Context, Fixed64.MinIncrement);
        body3D.Body.Mass = Fixed64.MinIncrement;
        body3D.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        first2D.FreezeAxes = BodyFreezeAxes2D.Rotation;
        second2D.FreezeAxes = BodyFreezeAxes2D.Rotation;

        ContactNormalImpulse2D.TryCalculateVelocityDeltas(
                first2D,
                Vector2d.Right,
                Fixed64.Zero,
                Vector2d.Zero,
                second2D,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                out _)
            .Should()
            .BeFalse();
        ContactNormalImpulse2D.TryCalculateAccumulatedDelta(
                first2D,
                Vector2d.Right,
                Fixed64.Zero,
                Vector2d.Zero,
                second2D,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                out _)
            .Should()
            .BeFalse();
        ContactNormalImpulseMixed.TryCalculateVelocityDeltas(
                body3D.Body,
                Vector3d.Right,
                Vector3d.Zero,
                Vector3d.Zero,
                first2D,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                out _)
            .Should()
            .BeFalse();
        ContactNormalImpulseMixed.TryCalculateAccumulatedDelta(
                body3D.Body,
                Vector3d.Right,
                Vector3d.Zero,
                Vector3d.Zero,
                first2D,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void CompactPlanarKernel_ShouldRejectSubprecisionAngularEffectiveMass()
    {
        using PhysicsScenarioBuilder scenario =
            PhysicsScenarioBuilder.Create();
        SolidBody2D body = CreateBody2D(
            scenario.Context,
            Fixed64.MinIncrement,
            Fixed64.Two);
        body.FreezeAxes =
            BodyFreezeAxes2D.PositionX
            | BodyFreezeAxes2D.PositionY;

        ContactNormalImpulse2D.TryComputeAngularDenominator(
                body,
                new Vector2d(
                    Fixed64.MinIncrement,
                    Fixed64.Zero),
                Vector2d.Forward,
                out Fixed64 denominator)
            .Should()
            .BeFalse();

        denominator.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void CompactPlanarKernel_ShouldRejectUnrepresentableLeverCross()
    {
        using PhysicsScenarioBuilder scenario =
            PhysicsScenarioBuilder.Create();
        SolidBody2D body =
            CreateBody2D(scenario.Context, Fixed64.One);
        Vector2d diagonal =
            new Vector2d(Fixed64.One, Fixed64.One).Normalized;

        ContactNormalImpulse2D.TryCalculateVelocityDeltas(
                body,
                Vector2d.Right,
                Fixed64.Zero,
                new Vector2d(
                    Fixed64.MaxValue,
                    Fixed64.MinValue),
                null,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                diagonal,
                Fixed64.Zero,
                Fixed64.Zero,
                out ContactNormalVelocityDeltaResult2D result)
            .Should()
            .BeFalse();

        result.Should().Be(default(ContactNormalVelocityDeltaResult2D));
    }

    [Fact]
    public void CompactPlanarKernel_ShouldRejectSubprecisionLeverCross()
    {
        using PhysicsScenarioBuilder scenario =
            PhysicsScenarioBuilder.Create();
        SolidBody2D body =
            CreateBody2D(scenario.Context, Fixed64.One);
        Vector2d axis =
            new Vector2d(Fixed64.One, (Fixed64)3).Normalized;

        ContactNormalImpulse2D.TryComputeAngularVelocityDelta(
                body,
                Vector2d.Forward * Fixed64.MinIncrement,
                axis,
                Fixed64.MaxValue,
                out Fixed64 delta)
            .Should()
            .BeFalse();

        delta.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CompactPlanarKernel_ShouldRejectUnrepresentableAngularVelocityDelta(
        bool crossIsUnrepresentable)
    {
        using PhysicsScenarioBuilder scenario =
            PhysicsScenarioBuilder.Create();
        SolidBody2D body =
            CreateBody2D(scenario.Context, Fixed64.One);
        Vector2d relativeContactPoint = crossIsUnrepresentable
            ? new Vector2d(Fixed64.MaxValue, Fixed64.MinValue)
            : Vector2d.Forward;
        Vector2d axis = crossIsUnrepresentable
            ? new Vector2d(Fixed64.One, Fixed64.One).Normalized
            : Vector2d.Right;
        Fixed64 impulse = crossIsUnrepresentable
            ? Fixed64.One
            : Fixed64.MaxValue;

        ContactNormalImpulse2D.TryComputeAngularVelocityDelta(
                body,
                relativeContactPoint,
                axis,
                impulse,
                out Fixed64 delta)
            .Should()
            .BeFalse();

        delta.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void AccumulatedMixedKernel_ShouldCancelOverflowingSeparatingWarmStart()
    {
        using PhysicsScenarioBuilder scenario =
            PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(
            Vector3d.Zero,
            mass: Fixed64.MaxValue);

        ContactNormalImpulseMixed.TryCalculateAccumulatedDelta(
                body.Body,
                Vector3d.Left * Fixed64.Two,
                Vector3d.Zero,
                Vector3d.Zero,
                null,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                Fixed64.One,
                out ContactNormalImpulseResultMixed compact)
            .Should()
            .BeTrue();

        ExactLever3D zero =
            CreateLever(Vector3d.Zero, Vector3d.Zero);
        ContactNormalImpulseMixed.TryCalculateAccumulatedDeltaExact(
                body.Body,
                Vector3d.Left * Fixed64.Two,
                Vector3d.Zero,
                zero,
                null,
                Vector2d.Zero,
                Fixed64.Zero,
                zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                Fixed64.One,
                out ContactNormalImpulseResultMixed result)
            .Should()
            .BeTrue();

        result.ImpulseScalar.Should().Be(-Fixed64.One);
        compact.ImpulseScalar.Should().Be(result.ImpulseScalar);
        compact.LinearVelocityDelta3D.Should().Be(
            result.LinearVelocityDelta3D);
        compact.AngularVelocityDelta3D.Should().Be(
            result.AngularVelocityDelta3D);
        compact.LinearVelocityDelta2D.Should().Be(
            result.LinearVelocityDelta2D);
        compact.AngularVelocityDelta2D.Should().Be(
            result.AngularVelocityDelta2D);
        result.LinearVelocityDelta3D.Should().Be(
            Vector3d.Right * body.Body.EffectiveInverseMass);
        result.AngularVelocityDelta3D.Should().Be(Vector3d.Zero);
        result.LinearVelocityDelta2D.Should().Be(Vector2d.Zero);
        result.AngularVelocityDelta2D.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void AccumulatedPlanarKernels_ShouldRejectNegativeSolverInputs(
        int invalidInput)
    {
        Fixed64 accumulatedImpulse =
            invalidInput == 0 ? -Fixed64.One : Fixed64.Zero;
        Fixed64 positiveImpulseScale =
            invalidInput == 1 ? -Fixed64.One : Fixed64.One;
        Fixed64 negativeImpulseScale =
            invalidInput == 2 ? -Fixed64.One : Fixed64.One;

        ContactNormalImpulse2D.TryCalculateAccumulatedDelta(
                null,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                null,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                accumulatedImpulse,
                positiveImpulseScale,
                negativeImpulseScale,
                out _)
            .Should()
            .BeFalse();
        ContactNormalImpulseMixed.TryCalculateAccumulatedDelta(
                null,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Zero,
                null,
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                accumulatedImpulse,
                positiveImpulseScale,
                negativeImpulseScale,
                out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void ExactPlanarParticipant_ShouldRespectIndependentMobility()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        SolidBody2D body =
            CreateBody2D(scenario.Context, Fixed64.One);
        ExactLever3D lever =
            CreateLever(Vector3d.Zero, Vector3d.Forward);

        body.FreezeAxes = BodyFreezeAxes2D.Position;
        ExactContactLever2D.TryGetParticipantVelocityDeltas(
                body,
                lever,
                Vector3d.Right,
                Fixed64.One,
                Vector3d.Zero,
                Fixed64.Zero,
                out Vector2d rotationOnlyLinear,
                out Fixed64 rotationOnlyAngular)
            .Should()
            .BeTrue();
        rotationOnlyLinear.Should().Be(Vector2d.Zero);
        rotationOnlyAngular.Should().NotBe(Fixed64.Zero);

        body.FreezeAxes = BodyFreezeAxes2D.Rotation;
        ExactContactLever2D.TryGetParticipantVelocityDeltas(
                body,
                lever,
                Vector3d.Right,
                Fixed64.One,
                Vector3d.Zero,
                Fixed64.Zero,
                out Vector2d translationOnlyLinear,
                out Fixed64 translationOnlyAngular)
            .Should()
            .BeTrue();
        translationOnlyLinear.Should().NotBe(Vector2d.Zero);
        translationOnlyAngular.Should().Be(Fixed64.Zero);

        body.FreezeAxes = BodyFreezeAxes2D.All;
        ExactContactLever2D.TryGetParticipantVelocityDeltas(
                body,
                lever,
                Vector3d.Right,
                Fixed64.One,
                Vector3d.Zero,
                Fixed64.Zero,
                out Vector2d frozenLinear,
                out Fixed64 frozenAngular)
            .Should()
            .BeTrue();
        frozenLinear.Should().Be(Vector2d.Zero);
        frozenAngular.Should().Be(Fixed64.Zero);

        ExactContactLever2D.TryGetParticipantVelocityDeltas(
                null,
                lever,
                Vector3d.Right,
                Fixed64.One,
                Vector3d.Zero,
                Fixed64.Zero,
                out Vector2d bodylessLinear,
                out Fixed64 bodylessAngular)
            .Should()
            .BeTrue();
        bodylessLinear.Should().Be(Vector2d.Zero);
        bodylessAngular.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void VelocityDeltaPolicy_ShouldRejectFusedFinalOverflowAtomically()
    {
        ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
                Vector3d.Right,
                Fixed64.MaxValue,
                Fixed64.Two,
                Fixed64.MaxValue,
                Fixed64.MinIncrement,
                out Vector3d delta3D)
            .Should()
            .BeFalse();
        ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
                Vector2d.Right,
                Fixed64.MaxValue,
                Fixed64.Two,
                Fixed64.MaxValue,
                Fixed64.MinIncrement,
                out Vector2d delta2D)
            .Should()
            .BeFalse();
        delta3D.Should().Be(Vector3d.Zero);
        delta2D.Should().Be(Vector2d.Zero);
    }

    private static SolidBody2D CreateBody2D(
        GravitasWorldContext context,
        Fixed64 mass,
        Fixed64? radius = null)
    {
        var body = new SolidBody2D(
            new TestMatterAgent(
                context,
                new FixedTransform(
                    Vector3d.Zero,
                    FixedQuaternion.Identity,
                    Vector3d.One)),
            new LSCircleCollider2D(radius ?? Fixed64.Half))
        {
            Mass = mass
        };
        body.Initialize(Vector2d.Zero);
        return body;
    }

    private static void AssertAccumulatedUnresolved(
        SolidBody? bodyA,
        Vector3d linearVelocityA,
        Vector3d angularVelocityA,
        ExactLever3D leverA,
        SolidBody? bodyB,
        Vector3d linearVelocityB,
        Vector3d angularVelocityB,
        ExactLever3D leverB,
        Fixed64 accumulatedImpulse) =>
        ContactNormalImpulse3D.TryCalculateAccumulatedDeltaExact(
                bodyA,
                linearVelocityA,
                angularVelocityA,
                leverA,
                bodyB,
                linearVelocityB,
                angularVelocityB,
                leverB,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                accumulatedImpulse,
                Fixed64.One,
                Fixed64.One,
                out _)
            .Should()
            .BeFalse();

    private static void AssertCompactUnaccumulatedUnresolved(
        SolidBody? bodyA,
        Vector3d linearVelocityA,
        Vector3d angularVelocityA,
        Vector3d leverA,
        SolidBody? bodyB,
        Vector3d linearVelocityB,
        Vector3d angularVelocityB,
        Vector3d leverB,
        Vector3d normal) =>
        ContactNormalImpulse3D.TryCalculateVelocityDeltas(
                bodyA,
                linearVelocityA,
                angularVelocityA,
                leverA,
                bodyB,
                linearVelocityB,
                angularVelocityB,
                leverB,
                normal,
                Fixed64.Zero,
                Fixed64.Zero,
                out ContactNormalVelocityDeltaResult3D result)
            .Should()
            .BeFalse();

    private static void AssertCompactAccumulatedUnresolved(
        SolidBody? bodyA,
        Vector3d linearVelocityA,
        Vector3d angularVelocityA,
        Vector3d leverA,
        SolidBody? bodyB,
        Vector3d linearVelocityB,
        Vector3d angularVelocityB,
        Vector3d leverB,
        Vector3d normal,
        Fixed64 accumulatedImpulse) =>
        ContactNormalImpulse3D.TryCalculateAccumulatedDelta(
                bodyA,
                linearVelocityA,
                angularVelocityA,
                leverA,
                bodyB,
                linearVelocityB,
                angularVelocityB,
                leverB,
                normal,
                Fixed64.Zero,
                Fixed64.Zero,
                accumulatedImpulse,
                Fixed64.One,
                Fixed64.One,
                out ContactNormalImpulseResult3D result)
            .Should()
            .BeFalse();

    private static SolidBody CreateBodyWithInertia(
        PhysicsScenarioBuilder scenario,
        Fixed3x3 inertiaTensor)
    {
        var collider = new UnsupportedTestCollider3D
        {
            InertiaTensor = inertiaTensor
        };
        return scenario.CreateBody(
            collider,
            Vector3d.Zero,
            FixedQuaternion.Identity).Body;
    }

    private static ExactLever3D CreateLever(
        Vector3d origin,
        Vector3d localPoint)
    {
        var point = new FixedPointAnchor(
            origin,
            FixedQuaternion.Identity,
            localPoint);
        var center = new FixedPointAnchor(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        return ExactLever3D.Create(point, center);
    }
}
