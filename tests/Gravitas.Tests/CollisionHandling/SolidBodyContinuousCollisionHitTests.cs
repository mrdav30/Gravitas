using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class SolidBodyContinuousCollisionHitTests
{
    [Fact]
    public void ContinuousMode_WithEpsilonScaleDynamicSphere_ShouldUseStableFallbackNormal()
    {
        using GravitasWorldContext context = CreateContext();
        Fixed64 fallbackRadius = Fixed64.FromFraction(1, 65536);
        fallbackRadius.Should().BeGreaterThan(Fixed64.Epsilon);
        (fallbackRadius * fallbackRadius).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        SolidBody target = CreateBody3D(
            context,
            Vector3d.Zero,
            new LSSphereCollider { Radius = fallbackRadius });
        SolidBody source = CreateBody3D(
            context,
            -Vector3d.Right * (Fixed64)5,
            new LSCuboidCollider { Size = Vector3d.One });
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Sleep();

        source.AddForce(Vector3d.Right * (Fixed64)10);
        context.LateSimulate();

        source.Position3d.X.Should().BeLessThan((Fixed64)5);
        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
        target.Position3d.X.Should().BeGreaterThan(Fixed64.Zero);
        source.Collider.Center.Should().Be(source.Position3d);
        target.Collider.Center.Should().Be(target.Position3d);
    }

    [Fact]
    public void ContinuousMode_WithProxyClosingButExactNormalNotClosing_ShouldRejectHitAndRestoreShapes()
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody rotationMutator = CreateBody3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)8),
            new LSSphereCollider());
        SolidBody source = CreateBody3D(
            context,
            Vector3d.Zero,
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.Half, Fixed64.Half)
            });
        SolidBody target = CreateBody3D(context, Vector3d.Zero, new LSSphereCollider());
        FixedQuaternion sourceRotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)10,
            Fixed64.Zero);
        FixedQuaternion targetRotationAfterFramePreparation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)20,
            Fixed64.Zero);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.SetRotation(sourceRotation);
        target.Sleep();
        rotationMutator.OnMoved += () => target.SetRotation(targetRotationAfterFramePreparation);

        rotationMutator.AddForce(Vector3d.Right);
        source.AddForce(-Vector3d.Right * (Fixed64)4);
        context.LateSimulate();

        source.Position3d.Should().Be(-Vector3d.Right * (Fixed64)4);
        source.LinearVelocity.Should().Be(-Vector3d.Right * (Fixed64)4);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        source.Collider.Center.Should().Be(source.Position3d);
        source.Rotation.Should().Be(sourceRotation);
        source.Collider.Rotation.Should().Be(sourceRotation);
        target.Position3d.Should().Be(Vector3d.Zero);
        target.LinearVelocity.Should().Be(Vector3d.Zero);
        target.Collider.Center.Should().Be(target.Position3d);
        target.Rotation.Should().Be(targetRotationAfterFramePreparation);
        target.Collider.Rotation.Should().Be(targetRotationAfterFramePreparation);
    }

    [Fact]
    public void ContinuousMode_WithNewlyFilteredDynamic2DCandidate_ShouldReachEndPose()
    {
        using GravitasWorldContext context = CreateContext(PhysicsRuntimeMode.Mixed);
        SolidBody deactivator = CreateBody3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)3),
            new LSSphereCollider());
        SolidBody2D target = CreateBody2D(context, Vector2d.Zero);
        SolidBody source = CreateBody3D(
            context,
            -Vector3d.Right * (Fixed64)5,
            new LSSphereCollider());
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Sleep();
        deactivator.OnMoved += () =>
            target.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(source.Collider.Layer);

        deactivator.AddForce(Vector3d.Right);
        source.AddForce(Vector3d.Right * (Fixed64)10);
        context.LateSimulate();

        target.Active.Should().BeTrue();
        target.Collider.IgnoredCollisionLayers.Includes(source.Collider.Layer).Should().BeTrue();
        target.Position.Should().Be(Vector2d.Zero);
        target.LinearVelocity.Should().Be(Vector2d.Zero);
        source.Position3d.Should().Be(Vector3d.Right * (Fixed64)5);
        source.LinearVelocity.Should().Be(Vector3d.Right * (Fixed64)10);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    private static GravitasWorldContext CreateContext(
        PhysicsRuntimeMode runtimeMode = PhysicsRuntimeMode.ThreeD)
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.SetFrameRate(1);
        context.Settings.RuntimeMode = runtimeMode;
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.MinSpeed = Fixed64.Zero;
        context.Environment.MaxSpeed = (Fixed64)16;
        context.Environment.MaxFallSpeed = (Fixed64)16;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-16), (Fixed64)(-4), (Fixed64)(-16)),
                new Vector3d((Fixed64)16, (Fixed64)4, (Fixed64)16)),
            out _).Should().BeTrue();
        return context;
    }

    private static SolidBody CreateBody3D(
        GravitasWorldContext context,
        Vector3d position,
        LSCollider collider)
    {
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One));
        var body = new SolidBody(agent, collider)
        {
            Mass = Fixed64.One,
            GroundedDistanceRay = Fixed64.Zero,
            GroundDownDistanceOnAir = Fixed64.Zero
        };
        body.Initialize(position, FixedQuaternion.Identity);
        return body;
    }

    private static SolidBody2D CreateBody2D(GravitasWorldContext context, Vector2d position)
    {
        var collider = new LSCircleCollider2D(Fixed64.Half);
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(position.ToVector3d(Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One));
        var body = new SolidBody2D(agent, collider) { Mass = Fixed64.One };
        body.Initialize(position);
        return body;
    }
}
