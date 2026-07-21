using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ContinuousCollisionPolicyTests
{
    [Fact]
    public void ContinuousCollisionContactPolicy_ShouldResolveSweptSpherePointsAndNormals()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider sphere = scenario.CreateStaticSphere(Vector3d.Zero);
        LSCuboidCollider cuboid = scenario.CreateCuboid(Vector3d.Zero).Collider;

        ContinuousCollisionContactPolicy.ResolveSweptSpherePoint(
                sphere,
                sphere.Center,
                Vector3d.Right,
                Fixed64.Zero)
            .Should().Be(sphere.Center - Vector3d.Right * sphere.ScaledRadius);

        ContinuousCollisionContactPolicy.ResolveSweptSpherePoint(
                sphere,
                Vector3d.Right * (Fixed64)2,
                -Vector3d.Right,
                Fixed64.Zero)
            .Should().Be(Vector3d.Right * sphere.ScaledRadius);

        ContinuousCollisionContactPolicy.ResolveSweptSphereNormal(
                cuboid,
                Vector3d.Zero,
                Vector3d.Right,
                -Vector3d.Right)
            .Should().Be(Vector3d.Right);

        ContinuousCollisionContactPolicy.ResolveSweptSphereNormal(
                sphere,
                Vector3d.Right * sphere.ScaledRadius,
                Vector3d.Right * (Fixed64)2,
                -Vector3d.Right)
            .Should().Be(Vector3d.Right);

        ContinuousCollisionContactPolicy.ResolveSweptSphereNormal(
                sphere,
                sphere.Center,
                Vector3d.Up,
                -Vector3d.Up)
            .Should().Be(Vector3d.Up);

        ContinuousCollisionContactPolicy.ResolveSweptSphereNormal(
                sphere,
                sphere.Center,
                sphere.Center,
                Vector3d.Right)
            .Should().Be(-Vector3d.Right);

        ContinuousCollisionContactPolicy.ResolveSweptSphereNormal(
                sphere,
                sphere.Center,
                sphere.Center,
                Vector3d.Zero)
            .Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void ContinuousCollisionImpulsePolicy_ShouldResolveNormalsFromHitOrSourceMotion()
    {
        ContinuousCollisionImpulsePolicy.TryResolveSourceNormal(
            Vector3d.Right * (Fixed64)2,
            Vector3d.Up,
            out Vector3d normal3D).Should().BeTrue();
        normal3D.Should().Be(Vector3d.Right);

        ContinuousCollisionImpulsePolicy.TryResolveSourceNormal(
            Vector3d.Zero,
            Vector3d.Right * (Fixed64)2,
            out normal3D).Should().BeTrue();
        normal3D.Should().Be(-Vector3d.Right);

        ContinuousCollisionImpulsePolicy.TryResolveSourceNormal(
            Vector3d.Zero,
            Vector3d.Zero,
            out normal3D).Should().BeFalse();
        normal3D.Should().Be(Vector3d.Zero);

        ContinuousCollisionImpulsePolicy.TryResolveImpactNormal(
            Vector3d.Up * (Fixed64)2,
            out normal3D).Should().BeTrue();
        normal3D.Should().Be(Vector3d.Up);

        ContinuousCollisionImpulsePolicy.TryResolveImpactNormal(
            Vector3d.Zero,
            out normal3D).Should().BeFalse();

        ContinuousCollisionImpulsePolicy.TryResolveSourceNormal(
            Vector2d.Right * (Fixed64)2,
            Vector2d.Forward,
            out Vector2d normal2D).Should().BeTrue();
        normal2D.Should().Be(Vector2d.Right);

        ContinuousCollisionImpulsePolicy.TryResolveSourceNormal(
            Vector2d.Zero,
            Vector2d.Right * (Fixed64)2,
            out normal2D).Should().BeTrue();
        normal2D.Should().Be(-Vector2d.Right);

        ContinuousCollisionImpulsePolicy.TryResolveSourceNormal(
            Vector2d.Zero,
            Vector2d.Zero,
            out normal2D).Should().BeFalse();

        ContinuousCollisionImpulsePolicy.TryResolveImpactNormal(
            Vector2d.Forward * (Fixed64)2,
            out normal2D).Should().BeTrue();
        normal2D.Should().Be(Vector2d.Forward);

        ContinuousCollisionImpulsePolicy.TryResolveImpactNormal(
            Vector2d.Zero,
            out normal2D).Should().BeFalse();
    }

    [Fact]
    public void TryResolveVelocityDelta_ShouldAcceptExactFiniteAndZeroMobility()
    {
        Fixed64 normalComponent = Fixed64.FromFraction(1, 65536);

        ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            new Vector2d(Fixed64.Zero, normalComponent),
            Fixed64.One,
            Fixed64.One,
            Fixed64.MinIncrement,
            out Vector2d delta2D).Should().BeTrue();
        ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            new Vector3d(Fixed64.Zero, normalComponent, Fixed64.Zero),
            Fixed64.One,
            Fixed64.One,
            Fixed64.MinIncrement,
            out Vector3d delta3D).Should().BeTrue();
        ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            Vector3d.One,
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero,
            out Vector3d zeroDelta).Should().BeTrue();
        ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            Vector2d.One,
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero,
            out Vector2d zeroDelta2D).Should().BeTrue();

        delta2D.Should().Be(new Vector2d(Fixed64.Zero, (Fixed64)65536));
        delta3D.Should().Be(new Vector3d(Fixed64.Zero, (Fixed64)65536, Fixed64.Zero));
        zeroDelta.Should().Be(Vector3d.Zero);
        zeroDelta2D.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void TryResolveVelocityDelta_ShouldRejectOnlyFinalOverflowAndRemainComponentAtomic()
    {
        ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            Vector2d.Right,
            Fixed64.Two,
            Fixed64.One,
            Fixed64.One,
            out Vector2d ordinaryDelta).Should().BeTrue();
        ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            new Vector2d(Fixed64.MinIncrement, Fixed64.One),
            Fixed64.Two,
            Fixed64.MaxValue,
            Fixed64.One,
            out Vector2d overflow2D).Should().BeFalse();
        ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            new Vector3d(Fixed64.MinIncrement, Fixed64.Zero, Fixed64.One),
            Fixed64.Two,
            Fixed64.MaxValue,
            Fixed64.One,
            out Vector3d overflow3D).Should().BeFalse();
        ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            Vector2d.One,
            Fixed64.Two,
            Fixed64.MaxValue,
            Fixed64.One,
            out _).Should().BeFalse();
        ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            new Vector3d(Fixed64.MinIncrement, Fixed64.One, Fixed64.Zero),
            Fixed64.Two,
            Fixed64.MaxValue,
            Fixed64.One,
            out _).Should().BeFalse();
        ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            Vector3d.One,
            Fixed64.Two,
            Fixed64.MaxValue,
            Fixed64.One,
            out _).Should().BeFalse();

        ordinaryDelta.Should().Be(Vector2d.Right * Fixed64.Two);
        overflow2D.Should().Be(default);
        overflow3D.Should().Be(default);
    }
}
