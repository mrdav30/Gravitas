using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class SolidBody2DContinuousCollisionHitTests
{
    [Fact]
    public void ContinuousMode_WithCloserStatic3DHit_ShouldSelectMixedTargetAndRestoreSourceShape()
    {
        using GravitasWorldContext context = CreateMixedContext();
        SolidBody2D source = CreateBody2D(context, new Vector2d((Fixed64)(-5), Fixed64.Zero));
        LSCollider static3D = CreateBodyless3D(context, Vector3d.Zero);
        LSCollider2D static2D = CreateBodyless2D(context, new Vector2d((Fixed64)2, Fixed64.Zero));
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.AddForce(Vector2d.Right * (Fixed64)10);
        context.LateSimulate();

        static3D.Id.Should().BeGreaterThanOrEqualTo(0);
        static2D.Id.Should().BeGreaterThanOrEqualTo(0);
        source.Position.Should().Be(-Vector2d.Right);
        source.LinearVelocity.Should().Be(Vector2d.Zero);
        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
        source.Collider.Center.Should().Be(source.Position);
    }

    [Fact]
    public void ContinuousMode_WithStalePlanarAndMixedCandidates_ShouldResetHitAndReachEndPose()
    {
        using GravitasWorldContext context = CreateMixedContext();
        SolidBody deactivator = CreateBody3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)3));
        SolidBody2D stale2D = CreateBody2D(context, Vector2d.Zero);
        SolidBody stale3D = CreateBody3D(context, Vector3d.Zero);
        SolidBody2D source = CreateBody2D(context, new Vector2d((Fixed64)(-5), Fixed64.Zero));
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        stale2D.Sleep();
        stale3D.Sleep();
        deactivator.OnMoved += stale2D.Deactivate;
        deactivator.OnMoved += stale3D.Deactivate;

        deactivator.AddForce(Vector3d.Right);
        source.AddForce(Vector2d.Right * (Fixed64)10);
        context.LateSimulate();

        stale2D.Active.Should().BeFalse();
        stale3D.Active.Should().BeFalse();
        source.Position.Should().Be(Vector2d.Right * (Fixed64)5);
        source.LinearVelocity.Should().Be(Vector2d.Right * (Fixed64)10);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        source.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
        source.Collider.Center.Should().Be(source.Position);
    }

    [Fact]
    public void ContinuousMode_WithNewlyFilteredMixedCandidate_ShouldResetHitAndReachEndPose()
    {
        using GravitasWorldContext context = CreateMixedContext();
        SolidBody deactivator = CreateBody3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)3));
        SolidBody target = CreateBody3D(context, Vector3d.Zero);
        SolidBody2D source = CreateBody2D(context, new Vector2d((Fixed64)(-5), Fixed64.Zero));
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Sleep();
        deactivator.OnMoved += () =>
            target.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(source.Collider.Layer);

        deactivator.AddForce(Vector3d.Right);
        source.AddForce(Vector2d.Right * (Fixed64)10);
        context.LateSimulate();

        target.Active.Should().BeTrue();
        target.Collider.IgnoredCollisionLayers.Includes(source.Collider.Layer).Should().BeTrue();
        target.Position3d.Should().Be(Vector3d.Zero);
        target.LinearVelocity.Should().Be(Vector3d.Zero);
        source.Position.Should().Be(Vector2d.Right * (Fixed64)5);
        source.LinearVelocity.Should().Be(Vector2d.Right * (Fixed64)10);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Fact]
    public void ContinuousMode_WithTwoDynamicMixedCandidates_ShouldKeepNearestTarget()
    {
        using GravitasWorldContext context = CreateMixedContext();
        SolidBody nearest = CreateBody3D(context, Vector3d.Zero);
        SolidBody farther = CreateBody3D(context, new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));
        SolidBody2D source = CreateBody2D(context, new Vector2d((Fixed64)(-5), Fixed64.Zero));
        nearest.FreezeAxes = BodyFreezeAxes3D.PositionX;
        nearest.Sleep();
        farther.Sleep();
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.AddForce(Vector2d.Right * (Fixed64)10);
        context.LateSimulate();

        Fixed64 candidateOrderingTolerance = Fixed64.Epsilon * (Fixed64)8;
        (source.Position.X + Fixed64.One).Abs().Should().BeLessThanOrEqualTo(candidateOrderingTolerance);
        source.Position.Y.Should().Be(Fixed64.Zero);
        source.LinearVelocity.Should().Be(Vector2d.Zero);
        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
        nearest.Collider.Id.Should().BeLessThan(farther.Collider.Id);
        nearest.Position3d.Should().Be(Vector3d.Zero);
        nearest.LinearVelocity.Should().Be(Vector3d.Zero);
        farther.Position3d.Should().Be(new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));
        farther.LinearVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void ContinuousMode_WithProxyClosingButExactNormalNotClosing_ShouldRejectHitAndRestoreShapes()
    {
        using GravitasWorldContext context = CreateMixedContext();
        SolidBody rotationMutator = CreateBody3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)8));
        SolidBody2D source = CreateBody2D(
            context,
            Vector2d.Zero,
            new LSAABBoxCollider2D(new Vector2d((Fixed64)6, Fixed64.Half)));
        SolidBody2D target = CreateBody2D(context, Vector2d.Zero);
        Fixed64 sourceRotation = Fixed64.FromFraction(1, 7);
        Fixed64 targetRotationAfterFramePreparation = Fixed64.FromFraction(2, 7);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.SetRotation(sourceRotation);
        target.Sleep();
        rotationMutator.OnMoved += () => target.SetRotation(targetRotationAfterFramePreparation);

        rotationMutator.AddForce(Vector3d.Right);
        source.AddForce(-Vector2d.Right * (Fixed64)4);
        context.LateSimulate();

        source.Position.Should().Be(-Vector2d.Right * (Fixed64)4);
        source.LinearVelocity.Should().Be(-Vector2d.Right * (Fixed64)4);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        source.Collider.Center.Should().Be(source.Position);
        source.Rotation.Should().Be(sourceRotation);
        source.Collider.Rotation.Should().Be(sourceRotation);
        target.Position.Should().Be(Vector2d.Zero);
        target.LinearVelocity.Should().Be(Vector2d.Zero);
        target.Collider.Center.Should().Be(target.Position);
        target.Rotation.Should().Be(targetRotationAfterFramePreparation);
        target.Collider.Rotation.Should().Be(targetRotationAfterFramePreparation);
    }

    private static GravitasWorldContext CreateMixedContext()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.SetFrameRate(1);
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.Environment.Gravity = Fixed64.Zero;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-16), (Fixed64)(-4), (Fixed64)(-16)),
                new Vector3d((Fixed64)16, (Fixed64)4, (Fixed64)16)),
            out _).Should().BeTrue();
        return context;
    }

    private static SolidBody2D CreateBody2D(
        GravitasWorldContext context,
        Vector2d position,
        LSCollider2D? collider = null)
    {
        collider ??= new LSCircleCollider2D(Fixed64.Half);
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(position.ToVector3d(Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One));
        var body = new SolidBody2D(agent, collider) { Mass = Fixed64.One };
        body.Initialize(position);
        return body;
    }

    private static SolidBody CreateBody3D(GravitasWorldContext context, Vector3d position)
    {
        var collider = new LSSphereCollider();
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One));
        var body = new SolidBody(agent, collider) { Mass = Fixed64.One };
        body.Initialize(position, FixedQuaternion.Identity);
        return body;
    }

    private static LSCollider CreateBodyless3D(GravitasWorldContext context, Vector3d position)
    {
        var collider = new LSSphereCollider();
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One));
        collider.InitializeWithNoBody(agent);
        return collider;
    }

    private static LSCollider2D CreateBodyless2D(GravitasWorldContext context, Vector2d position)
    {
        var collider = new LSCircleCollider2D(Fixed64.Half);
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(position.ToVector3d(Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One));
        collider.InitializeWithNoBody(agent);
        return collider;
    }
}
