using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedResponseTests
{
    [Fact]
    public void MixedNormalKernel_WithExtremeRestitution_ShouldNotSaturateBeforeFusedRatio()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(
            context,
            new Vector2d((Fixed64)2, Fixed64.Zero));
        body3D.Body.Mass = Fixed64.MaxValue;
        body2D.Mass = Fixed64.MaxValue;

        bool resolved = ContactNormalImpulseMixed.TryCalculateVelocityDeltas(
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
            out ContactNormalVelocityDeltaResultMixed result);

        resolved.Should().BeTrue();
        result.LinearVelocityDelta3D.Should().Be(Vector3d.Left * Fixed64.MaxValue);
        result.LinearVelocityDelta2D.Should().Be(Vector2d.Right * Fixed64.MaxValue);
    }

    [Fact]
    public void MixedNormalKernel_WithMaxMassPair_ShouldResolveRepresentableDeltasWithoutImpulseScalar()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, new Vector2d((Fixed64)2, Fixed64.Zero));
        body3D.Body.Mass = Fixed64.MaxValue;
        body2D.Mass = Fixed64.MaxValue;

        bool resolved = ContactNormalImpulseMixed.TryCalculateVelocityDeltas(
            body3D.Body,
            Vector3d.Right * (Fixed64)8,
            Vector3d.Zero,
            Vector3d.Zero,
            body2D,
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Zero,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            out ContactNormalVelocityDeltaResultMixed result);

        resolved.Should().BeTrue();
        result.NormalVelocity.Should().Be((Fixed64)(-8));
        result.LinearVelocityDelta3D.Should().Be(-Vector3d.Right * (Fixed64)4);
        result.AngularVelocityDelta3D.Should().Be(Vector3d.Zero);
        result.LinearVelocityDelta2D.Should().Be(Vector2d.Right * (Fixed64)4);
        result.AngularVelocityDelta2D.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void MixedNormalKernel_WithAuthoredKinematic3DAngularVelocity_ShouldDriveDynamic2DTarget()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, new Vector2d((Fixed64)2, Fixed64.Zero));
        body3D.Body.IsKinematic = true;

        bool resolved = ContactNormalImpulseMixed.TryCalculateVelocityDeltas(
            body3D.Body,
            Vector3d.Zero,
            -Vector3d.Forward,
            Vector3d.Up,
            body2D,
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Forward,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            out ContactNormalVelocityDeltaResultMixed result);

        resolved.Should().BeTrue();
        result.NormalVelocity.Should().Be(-Fixed64.One);
        result.LinearVelocityDelta3D.Should().Be(Vector3d.Zero);
        result.AngularVelocityDelta3D.Should().Be(Vector3d.Zero);
        result.LinearVelocityDelta2D.X.Should().BeGreaterThan(Fixed64.Zero);
        result.AngularVelocityDelta2D.Should().NotBe(Fixed64.Zero);
    }

    [Fact]
    public void MixedNormalKernel_WithAuthoredKinematic2DAngularVelocityAndFrozen3DAxis_ShouldApplyOnly3DAngularDelta()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, new Vector2d((Fixed64)2, Fixed64.Zero));
        body3D.Body.FreezeAxes = BodyFreezeAxes3D.PositionX;
        body2D.IsKinematic = true;

        bool resolved = ContactNormalImpulseMixed.TryCalculateVelocityDeltas(
            body3D.Body,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Up,
            body2D,
            Vector2d.Zero,
            Fixed64.One,
            Vector2d.Forward,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            out ContactNormalVelocityDeltaResultMixed result);

        resolved.Should().BeTrue();
        result.NormalVelocity.Should().Be(-Fixed64.One);
        result.LinearVelocityDelta3D.Should().Be(Vector3d.Zero);
        result.AngularVelocityDelta3D.Should().NotBe(Vector3d.Zero);
        result.LinearVelocityDelta2D.Should().Be(Vector2d.Zero);
        result.AngularVelocityDelta2D.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void MixedNormalKernel_WithFrozen2DNormalAxis_ShouldApplyOnly2DAngularDelta()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, new Vector2d((Fixed64)2, Fixed64.Zero));
        body3D.Body.IsKinematic = true;
        body2D.FreezeAxes = BodyFreezeAxes2D.PositionX;

        bool resolved = ContactNormalImpulseMixed.TryCalculateVelocityDeltas(
            body3D.Body,
            Vector3d.Zero,
            -Vector3d.Forward,
            Vector3d.Up,
            body2D,
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Forward,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            out ContactNormalVelocityDeltaResultMixed result);

        resolved.Should().BeTrue();
        result.LinearVelocityDelta3D.Should().Be(Vector3d.Zero);
        result.AngularVelocityDelta3D.Should().Be(Vector3d.Zero);
        result.LinearVelocityDelta2D.Should().Be(Vector2d.Zero);
        result.AngularVelocityDelta2D.Should().NotBe(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithBothNormalAxesFrozen_ShouldStillApplyAngularNormalResponse()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(
            context,
            new Vector2d((Fixed64)2, Fixed64.Zero));
        body3D.Body.FreezeAxes = BodyFreezeAxes3D.PositionX;
        body2D.FreezeAxes = BodyFreezeAxes2D.PositionX;
        body3D.Body.ApplyCollisionAngularVelocityDelta(-Vector3d.Forward);
        Vector3d angularVelocity3DBefore = body3D.Body.AngularVelocity;
        Fixed64 angularVelocity2DBefore = body2D.AngularVelocity;
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        var contact = new MixedContact(
            body3D.Body.WorldCenterOfMass + Vector3d.Up,
            (body2D.WorldCenterOfMass + Vector2d.Forward).ToVector3d(Fixed64.Zero),
            Vector3d.Right,
            Fixed64.FromFraction(1, 10));

        bool applied = CollisionResponseMixed.Resolve(pair, contact);

        applied.Should().BeTrue();
        body3D.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        body2D.LinearVelocity.Should().Be(Vector2d.Zero);
        body3D.Body.AngularVelocity.Should().NotBe(angularVelocity3DBefore);
        body2D.AngularVelocity.Should().NotBe(angularVelocity2DBefore);
    }

    [Fact]
    public void Resolve_WithKinematic3DRotation_ShouldUseAuthoredAngularVelocityForFriction()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            -Vector3d.Right * Fixed64.Half,
            isKinematic: true);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        PhysicsMaterial friction = new(Fixed64.One, Fixed64.One, Fixed64.Zero);
        body3D.Collider.Material = friction;
        body2D.Collider.Material = friction;
        body2D.ApplyCollisionLinearVelocityDelta(-Vector2d.Right * (Fixed64)4);
        body3D.Body.Agent.Transform.LocalRotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Up,
            Fixed64.HalfPi);
        body3D.Body.EnsureContinuousCollisionFramePrepared(context.LateSimulateToken);
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        var contact = new MixedContact(
            body3D.Body.WorldCenterOfMass + Vector3d.Right,
            body2D.WorldCenterOfMass.ToVector3d(Fixed64.Zero),
            Vector3d.Right,
            Fixed64.FromFraction(1, 10));

        bool applied = CollisionResponseMixed.Resolve(
            pair,
            contact,
            iteration: 0,
            iterationLimit: 1,
            applyPositionCorrection: false);

        applied.Should().BeTrue();
        body3D.Body.SampleContinuousCollisionAngularVelocity(Fixed64.One)
            .Should()
            .NotBe(Vector3d.Zero);
        body2D.LinearVelocity.Y.Should().NotBe(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithKinematic2DRotation_ShouldUseAuthoredAngularVelocityForFriction()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            -Vector3d.Right * Fixed64.Half);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        body2D.IsKinematic = true;
        PhysicsMaterial friction = new(Fixed64.One, Fixed64.One, Fixed64.Zero);
        body3D.Collider.Material = friction;
        body2D.Collider.Material = friction;
        body3D.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)4);
        body2D.Agent.Transform.LocalRotationXZRadians = Fixed64.HalfPi;
        body2D.EnsureContinuousCollisionFramePrepared(context.LateSimulateToken);
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        var contact = new MixedContact(
            body3D.Body.WorldCenterOfMass,
            (body2D.WorldCenterOfMass + Vector2d.Right).ToVector3d(Fixed64.Zero),
            Vector3d.Right,
            Fixed64.FromFraction(1, 10));

        bool applied = CollisionResponseMixed.Resolve(
            pair,
            contact,
            iteration: 0,
            iterationLimit: 1,
            applyPositionCorrection: false);

        applied.Should().BeTrue();
        body2D.SampleContinuousCollisionAngularVelocity(Fixed64.One)
            .Should()
            .NotBe(Fixed64.Zero);
        body3D.Body.LinearVelocity.Z.Should().NotBe(Fixed64.Zero);
    }

    [Fact]
    public void MixedNormalKernel_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, new Vector2d((Fixed64)2, Fixed64.Zero));
        body3D.Body.Mass = Fixed64.MaxValue;
        body2D.Mass = Fixed64.MaxValue;

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            () =>
            {
                _ = ContactNormalImpulseMixed.TryCalculateVelocityDeltas(
                    body3D.Body,
                    Vector3d.Right * (Fixed64)8,
                    Vector3d.Zero,
                    Vector3d.Zero,
                    body2D,
                    Vector2d.Zero,
                    Fixed64.Zero,
                    Vector2d.Zero,
                    Vector3d.Right,
                    Fixed64.Zero,
                    Fixed64.Zero,
                    out _);
            },
            warmupIterations: 4,
            stabilizationIterations: 2,
            measurementIterations: 4);

        allocatedBytes.Should().Be(0);
    }
}
