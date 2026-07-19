using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Response;

public sealed class ContactNormalImpulseBoundaryTests
{
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
    public void AccumulatedKernels_ShouldSaturateOnlyTheFinalUnrepresentableImpulse()
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

        ContactNormalImpulseResult3D closing3D = ContactNormalImpulse3D.CalculateAccumulatedDelta(
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
            Fixed64.One);
        ContactNormalImpulseResult3D separating3D = ContactNormalImpulse3D.CalculateAccumulatedDelta(
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
            Fixed64.One);
        closing3D.ImpulseScalar.Should().Be(Fixed64.MaxValue);
        separating3D.ImpulseScalar.Should().Be(Fixed64.Zero);

        ContactNormalImpulseResult2D closing2D = ContactNormalImpulse2D.CalculateAccumulatedDelta(
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
            Fixed64.One);
        ContactNormalImpulseResult2D separating2D = ContactNormalImpulse2D.CalculateAccumulatedDelta(
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
            Fixed64.One);
        closing2D.ImpulseScalar.Should().Be(Fixed64.MaxValue);
        separating2D.ImpulseScalar.Should().Be(Fixed64.Zero);

        ContactNormalImpulseResultMixed closingMixed = ContactNormalImpulseMixed.CalculateAccumulatedDelta(
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
            Fixed64.One);
        ContactNormalImpulseResultMixed separatingMixed = ContactNormalImpulseMixed.CalculateAccumulatedDelta(
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
            Fixed64.One);
        closingMixed.ImpulseScalar.Should().Be(Fixed64.MaxValue);
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

        ContactNormalImpulseResult2D result2D = ContactNormalImpulse2D.CalculateAccumulatedDelta(
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
            Fixed64.One);
        result2D.ImpulseScalar.Should().BeGreaterThan(Fixed64.Zero);
        result2D.LinearVelocityDeltaA.Should().Be(Vector2d.Zero);
        result2D.AngularVelocityDeltaA.Should().Be(Fixed64.Zero);
        result2D.LinearVelocityDeltaB.Should().NotBe(Vector2d.Zero);
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
        Fixed64 mass)
    {
        var body = new SolidBody2D(
            new TestMatterAgent(
                context,
                new FixedTransform(
                    Vector3d.Zero,
                    FixedQuaternion.Identity,
                    Vector3d.One)),
            new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = mass
        };
        body.Initialize(Vector2d.Zero);
        return body;
    }
}
