//=======================================================================
// MixedQueryCcdTests.RetryDistanceOrdering.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedQueryCcdTests
{
    [Fact]
    public void MixedContinuous3D_WhenStaticHitFallsInsideRetryAdvance_ShouldKeepEarlierStaticOrdering()
    {
        var dynamicFirst = Run3DRetryDistanceOrdering(blockerFirst: false);
        var blockerFirst = Run3DRetryDistanceOrdering(blockerFirst: true);

        blockerFirst.Should().Be(dynamicFirst);
        dynamicFirst.TargetPosition.Should().Be(Vector2d.Zero);
        dynamicFirst.TargetVelocity.Should().Be(Vector2d.Zero);
        dynamicFirst.SourceVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void MixedContinuous2D_WhenStaticHitFallsInsideRetryAdvance_ShouldKeepEarlierStaticOrdering()
    {
        var dynamicFirst = Run2DRetryDistanceOrdering(blockerFirst: false);
        var blockerFirst = Run2DRetryDistanceOrdering(blockerFirst: true);

        blockerFirst.Should().Be(dynamicFirst);
        dynamicFirst.TargetIsSleeping.Should().BeTrue();
        dynamicFirst.TargetPosition.Should().Be(Vector3d.Zero);
        dynamicFirst.TargetVelocity.Should().Be(Vector3d.Zero);
        dynamicFirst.SourcePosition.X.Should().BeLessThan(Fixed64.Zero);
    }

    private static (
        Vector3d SourcePosition,
        Vector3d SourceVelocity,
        Vector2d TargetPosition,
        Vector2d TargetVelocity,
        bool TargetIsSleeping) Run3DRetryDistanceOrdering(bool blockerFirst)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;

        SolidBody2D target;
        LSCollider2D blocker;
        if (blockerFirst)
        {
            blocker = CreateBodylessCircle2D(
                context,
                new Vector2d(Fixed64.FromRaw(4), Fixed64.Zero));
            target = CreateCircle2D(context, Vector2d.Zero);
        }
        else
        {
            target = CreateCircle2D(context, Vector2d.Zero);
            blocker = CreateBodylessCircle2D(
                context,
                new Vector2d(Fixed64.FromRaw(4), Fixed64.Zero));
        }

        var blockerLayer = new PhysicsLayer(1);
        blocker.Layer = blockerLayer;
        target.Collider.IgnoredCollisionLayers =
            PhysicsLayerMask.FromLayer(blockerLayer);
        target.Sleep();

        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero));
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        context.LateSimulate();
        target.Sleep();
        source.Body.AddForce(Vector3d.Right * (Fixed64)10);

        context.LateSimulate();

        return (
            source.Body.Position3d,
            source.Body.LinearVelocity,
            target.Position,
            target.LinearVelocity,
            target.IsSleeping);
    }

    private static (
        Vector2d SourcePosition,
        Vector3d TargetPosition,
        Vector3d TargetVelocity,
        bool TargetIsSleeping) Run2DRetryDistanceOrdering(bool blockerFirst)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;

        ScenarioBody<LSSphereCollider> target;
        LSCollider blocker;
        if (blockerFirst)
        {
            blocker = CreateBodyless3D(
                context,
                new LSSphereCollider(),
                new Vector3d(Fixed64.FromRaw(3), Fixed64.Zero, Fixed64.Zero));
            target = CreateSphere3D(context, Vector3d.Zero);
        }
        else
        {
            target = CreateSphere3D(context, Vector3d.Zero);
            blocker = CreateBodyless3D(
                context,
                new LSSphereCollider(),
                new Vector3d(Fixed64.FromRaw(3), Fixed64.Zero, Fixed64.Zero));
        }

        var blockerLayer = new PhysicsLayer(1);
        blocker.Layer = blockerLayer;
        target.Collider.IgnoredCollisionLayers =
            PhysicsLayerMask.FromLayer(blockerLayer);
        target.Body.Sleep();

        SolidBody2D source = CreateCircle2D(
            context,
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Agent.Transform.LocalPosition =
            new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero);

        context.LateSimulate();

        return (
            source.Position,
            target.Body.Position3d,
            target.Body.LinearVelocity,
            target.Body.IsSleeping);
    }
}
