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
    public void Resolve_WithSubEpsilonNormalMobility_ShouldResolveExactImpulse()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, -Fixed64.Half));
        LSCollider2D collider2D = CreateBodylessCircle2D(context, Vector2d.Zero);
        body3D.Body.FreezeAxes = BodyFreezeAxes3D.PositionX | BodyFreezeAxes3D.Rotation;
        body3D.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Forward);
        Vector3d normal = new(Fixed64.One, Fixed64.Zero, Fixed64.FromFraction(1, 65536));
        Vector3d velocityBefore = body3D.Body.LinearVelocity;
        var pair = new CollisionPairMixed(body3D.Collider, collider2D);
        var contact = new MixedContact(
            body3D.Collider.Center,
            Vector3d.Zero,
            normal,
            Fixed64.FromFraction(1, 5));

        normal.Normalized.Should().Be(normal);
        body3D.Body.GetConstrainedInverseMass(normal).Should().Be(Fixed64.FromRaw(1));
        Vector3d.Dot(-velocityBefore, normal).Should().BeLessThan(Fixed64.Zero);

        bool appliedImpulse = CollisionResponseMixed.Resolve(
            pair,
            contact,
            iteration: 0,
            iterationLimit: 1,
            applyPositionCorrection: false);

        appliedImpulse.Should().BeTrue();
        body3D.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        body3D.Body.AngularVelocity.Should().Be(Vector3d.Zero);
        body3D.Body.Position3d.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.Zero, -Fixed64.Half));
    }

    [Fact]
    public void Resolve_WithEpsilonFrictionMobility_ShouldRetainFrictionResponse()
    {
        Fixed64 residual = Fixed64.FromFraction(1, 4096);
        Vector3d normal = new(
            residual,
            Fixed64.Zero,
            Fixed64.One - Fixed64.Epsilon * Fixed64.Half);
        PhysicsMaterial frictionlessBounce = new(Fixed64.Zero, Fixed64.Zero, Fixed64.One);
        PhysicsMaterial frictionalBounce = new(Fixed64.One, Fixed64.One, Fixed64.One);

        (bool AppliedImpulse, Vector3d Velocity, Fixed64 TangentMagnitudeSquared, Fixed64 TangentInverseMass) normalOnly =
            ResolveFrozenXMixedContact(normal, frictionlessBounce);
        (bool AppliedImpulse, Vector3d Velocity, Fixed64 TangentMagnitudeSquared, Fixed64 TangentInverseMass) frictional =
            ResolveFrozenXMixedContact(normal, frictionalBounce);

        normal.MagnitudeSquared.Should().Be(Fixed64.One);
        normalOnly.AppliedImpulse.Should().BeTrue();
        normalOnly.TangentMagnitudeSquared.Should().BeGreaterThan(Fixed64.Epsilon);
        normalOnly.TangentInverseMass.Should().Be(Fixed64.Epsilon);
        frictional.AppliedImpulse.Should().BeTrue();
        frictional.TangentMagnitudeSquared
            .Should()
            .BeLessThan(normalOnly.TangentMagnitudeSquared);
    }

    private static (bool AppliedImpulse, Vector3d Velocity, Fixed64 TangentMagnitudeSquared, Fixed64 TangentInverseMass)
        ResolveFrozenXMixedContact(Vector3d normal, PhysicsMaterial material)
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Settings.RestitutionVelocityThreshold = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, -Fixed64.Half));
        LSCollider2D collider2D = CreateBodylessCircle2D(context, Vector2d.Zero);
        body3D.Body.FreezeAxes = BodyFreezeAxes3D.PositionX | BodyFreezeAxes3D.Rotation;
        body3D.Collider.Material = material;
        collider2D.Material = material;
        body3D.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Forward * (Fixed64)4);
        var pair = new CollisionPairMixed(body3D.Collider, collider2D);
        var contact = new MixedContact(
            body3D.Collider.Center,
            Vector3d.Zero,
            normal,
            Fixed64.FromFraction(1, 5));

        bool appliedImpulse = CollisionResponseMixed.Resolve(
            pair,
            contact,
            iteration: 0,
            iterationLimit: 1,
            applyPositionCorrection: false);

        Vector3d relativeVelocity = -body3D.Body.LinearVelocity;
        Vector3d tangentVelocity = relativeVelocity - normal * Vector3d.Dot(relativeVelocity, normal);
        Vector3d tangent = tangentVelocity.Normalized;
        return (
            appliedImpulse,
            body3D.Body.LinearVelocity,
            tangentVelocity.MagnitudeSquared,
            body3D.Body.GetConstrainedInverseMass(tangent));
    }
}
