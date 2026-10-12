using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

/// <content>Checks moving compound sources and nonresponsive static mesh CCD ownership.</content>
public sealed partial class MixedQueryCcdTests
{
    [Fact]
    public void TranslationalMixedMeshSource_ShouldTransferTheActualFinitePlaneImpulse()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        Fixed64 x = Fixed64.FromFraction(2, 5);
        var mesh = new LSMeshCollider(new[]
        {
            new Vector3d(x, (Fixed64)(-2), (Fixed64)(-2)), new Vector3d(x, (Fixed64)2, (Fixed64)(-2)),
            new Vector3d(x, (Fixed64)(-2), (Fixed64)2), new Vector3d(x, (Fixed64)2, (Fixed64)2)
        }, new[] { 0, 2, 1, 1, 2, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        SolidBody source = CreateBody3D(context, mesh, Vector3d.Zero).Body;
        SolidBody2D target = CreateCircle2D(context, Vector2d.Zero);
        source.FreezeAxes = BodyFreezeAxes3D.Rotation;
        target.FreezeAxes = BodyFreezeAxes2D.Rotation;
        source.Collider.Material = target.Collider.Material = PhysicsMaterial.Frictionless;
        source.UseManualGrounding();
        target.UseManualGrounding();
        source.ApplyCollisionLinearVelocityDelta(Vector3d.Left);
        PrepareMixedContinuousCollisionFrame(context);

        source.TryApplyContinuousCollisionMixed2DResponse(target, Vector3d.Right,
            Vector3d.Zero, Vector2d.Zero, Fixed64.Zero, Fixed64.Half).Should().BeTrue();

        source.LinearVelocity.Should().Be(Vector3d.Left * Fixed64.Half);
        target.LinearVelocity.Should().Be(Vector2d.Left * Fixed64.Half);
        source.Position3d.Should().Be(Vector3d.Zero);
        target.Position.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void StaticMixedMeshCcd_WithUnrepresentableElasticDelta_ShouldStopConservatively()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 64);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        context.Environment.MaxSpeed = Fixed64.MaxValue;
        context.Settings.ContinuousCollisionMaxToiIterations = 1;
        context.Settings.RestitutionVelocityThreshold = Fixed64.Zero;
        var material = new PhysicsMaterial(Fixed64.Zero, Fixed64.Zero, Fixed64.One);
        Fixed64 x = Fixed64.FromFraction(2, 5);
        var mesh = new LSMeshCollider(new[]
        {
            new Vector3d(x, (Fixed64)(-2), (Fixed64)(-2)), new Vector3d(x, (Fixed64)2, (Fixed64)(-2)),
            new Vector3d(x, (Fixed64)(-2), (Fixed64)2), new Vector3d(x, (Fixed64)2, (Fixed64)2)
        }, new[] { 0, 2, 1, 1, 2, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation) { Material = material };
        mesh.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        SolidBody2D source = CreateCircle2D(context, Vector2d.Zero);
        source.Collider.Material = material;
        source.FreezeAxes = BodyFreezeAxes2D.Rotation;
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.UseManualGrounding();
        source.ApplyCollisionLinearVelocityDelta(Vector2d.Right * Fixed64.MaxValue);
        PrepareMixedContinuousCollisionFrame(context);

        source.LateSimulate(updateSleepState: false, updateColliderState: false);

        source.Position.Should().Be(Vector2d.Zero);
        source.LinearVelocity.Should().Be(Vector2d.Zero);
        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void KinematicCompoundMixedImpact_ShouldTransferMotionWithoutChangingTheSource(bool source3D)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        SolidBody body3D;
        SolidBody2D body2D;
        if (source3D)
        {
            var compound = new LSCompoundCollider(CompoundColliderPart.Sphere(Fixed64.Half,
                Vector3d.Zero, PhysicsMaterial.Frictionless));
            body3D = CreateBody3D(context, compound, Vector3d.Zero, isKinematic: true).Body;
            body2D = CreateCircle2D(context, Vector2d.Right * Fixed64.FromFraction(3, 4));
            body3D.Agent.Transform.LocalPosition = Vector3d.Right;
        }
        else
        {
            body3D = CreateSphere3D(context, Vector3d.Right * Fixed64.FromFraction(3, 4)).Body;
            var compound = new LSCompoundCollider2D(CompoundColliderPart2D.Circle(Fixed64.Half,
                Vector2d.Zero, PhysicsMaterial.Frictionless));
            body2D = new SolidBody2D(new TestMatterAgent(context,
                new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)), compound) { Mass = Fixed64.One };
            body2D.Initialize(Vector2d.Zero, motionType: BodyMotionType.Kinematic);
            body2D.Agent.Transform.LocalPosition = Vector3d.Right;
        }
        body3D.FreezeAxes = BodyFreezeAxes3D.Rotation;
        body2D.FreezeAxes = BodyFreezeAxes2D.Rotation;
        body3D.Collider.Material = body2D.Collider.Material = PhysicsMaterial.Frictionless;
        body3D.UseManualGrounding();
        body2D.UseManualGrounding();
        if (source3D) body3D.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        else body2D.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        PrepareMixedContinuousCollisionFrame(context);
        body3D.EnsureContinuousCollisionFramePrepared(context.LateSimulateToken);
        body2D.EnsureContinuousCollisionFramePrepared(context.LateSimulateToken);

        bool applied = source3D
            ? body3D.TryApplyContinuousCollisionMixed2DResponse(body2D, Vector3d.Left,
                Vector3d.Zero, body2D.Position, Fixed64.Zero, Fixed64.Half)
            : body2D.TryApplyContinuousCollisionMixed3DResponse(body3D, Vector2d.Left,
                Vector2d.Zero, body3D.Position3d, Fixed64.Zero, Fixed64.Half);

        applied.Should().BeTrue();
        if (source3D)
        {
            body3D.Position3d.Should().Be(Vector3d.Zero);
            body3D.Agent.Transform.LocalPosition.Should().Be(Vector3d.Right);
            body2D.LinearVelocity.Should().Be(Vector2d.Right);
        }
        else
        {
            body2D.Position.Should().Be(Vector2d.Zero);
            body2D.Agent.Transform.LocalPosition.Should().Be(Vector3d.Right);
            body3D.LinearVelocity.Should().Be(Vector3d.Right);
        }
    }

    [Fact]
    public void KinematicCircularSlabAgainstBodylessMesh_ShouldKeepAConservativeStopWithoutAResponseDelta()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Settings.ContinuousCollisionMaxToiIterations = 1;
        var mesh = new LSMeshCollider(new[]
        {
            new Vector3d(1, -2, -2), new Vector3d(1, 2, -2),
            new Vector3d(1, -2, 2), new Vector3d(1, 2, 2)
        }, new[] { 0, 2, 1, 1, 2, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        mesh.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        SolidBody2D source = CreateCircle2D(context, Vector2d.Zero, isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Agent.Transform.LocalPosition = Vector3d.Right;
        PrepareMixedContinuousCollisionFrame(context);

        source.LateSimulate(updateSleepState: false, updateColliderState: false);

        source.Position.Should().Be(Vector2d.Right * Fixed64.Half);
        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
        source.LinearVelocity.Should().Be(Vector2d.Zero);
    }
}
