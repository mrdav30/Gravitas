//=======================================================================
// MixedQueryCcdTests.RelativeIntervals.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedQueryCcdTests
{
    [Fact]
    public void MixedContinuousMode_ProxyEntryBeforeShortLivedExactContact_ShouldResolveLaterContactDeterministically()
    {
        var first = RunOffsetCircleAcrossSphere();
        var second = RunOffsetCircleAcrossSphere();

        second.Should().Be(first);
        first.TargetVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        first.SourceToiIterations.Should().BeGreaterThan(0);
        first.SourceLimitReached.Should().BeFalse();
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void MixedContinuousMode_HeadOnMovingSphereCirclePair_ShouldResolveOnTheContinuousOwner(
        bool sphereOwnsContinuousPair)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        var circleCollider = new LSCircleCollider2D(Fixed64.Half);
        var circleAgent = new TestMatterAgent(
            context,
            new FixedTransform(
                new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
                FixedQuaternion.Identity,
                Vector3d.One));
        var circle = new SolidBody2D(circleAgent, circleCollider)
        {
            Mass = Fixed64.One
        };
        circle.Initialize(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            motionType: BodyMotionType.Dynamic);
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(
            context,
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));
        circle.ContinuousCollisionMode = sphereOwnsContinuousPair
            ? ContinuousCollisionMode.Discrete
            : ContinuousCollisionMode.Continuous;
        sphere.Body.ContinuousCollisionMode = sphereOwnsContinuousPair
            ? ContinuousCollisionMode.Continuous
            : ContinuousCollisionMode.Discrete;

        circle.ApplyCollisionLinearVelocityDelta(
            Vector2d.Right * (Fixed64)4);
        sphere.Body.ApplyCollisionLinearVelocityDelta(
            Vector3d.Left * (Fixed64)4);
        context.LateSimulate();

        circle.Position.X.Should().BeLessThan(sphere.Body.Position3d.X);
        (circle.LinearVelocity.X - (Fixed64)4).Abs()
            .Should()
            .BeGreaterThan(Fixed64.Epsilon);
        (sphere.Body.LinearVelocity.X + (Fixed64)4).Abs()
            .Should()
            .BeGreaterThan(Fixed64.Epsilon);
    }

    private static (
        Vector2d SourcePosition,
        Vector2d SourceVelocity,
        Vector3d TargetPosition,
        Vector3d TargetVelocity,
        int SourceToiIterations,
        bool SourceLimitReached) RunOffsetCircleAcrossSphere()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        var sourceCollider = new LSCircleCollider2D(Fixed64.Half)
        {
            LocalOffset = Vector2d.Forward * Fixed64.Two
        };
        var sourceAgent = new TestMatterAgent(
            context,
            new FixedTransform(
                new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
                FixedQuaternion.Identity,
                Vector3d.One));
        var source = new SolidBody2D(sourceAgent, sourceCollider)
        {
            Mass = Fixed64.One
        };
        sourceCollider.Material =
            PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero);
        source.Initialize(
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            motionType: BodyMotionType.Dynamic);
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Two));
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.ApplyCollisionLinearVelocityDelta(
            Vector2d.Right * (Fixed64)10);
        context.LateSimulate();

        return (
            source.Position,
            source.LinearVelocity,
            target.Body.Position3d,
            target.Body.LinearVelocity,
            source.LastContinuousCollisionToiIterationCount,
            source.LastContinuousCollisionToiIterationLimitReached);
    }
}
