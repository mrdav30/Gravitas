using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedQueryCcdTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Continuous2D_WitnessedStaticContact_ShouldStopRotationWithoutUnresolvedLimit(
        bool mixedTarget)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        SolidBody2D source = CreateRefinementSource2D(context, Vector2d.Zero);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector2d velocity = Vector2d.Right * Fixed64.FromFraction(mixedTarget ? -1 : 1, 10);
        if (mixedTarget)
        {
            LSSphereCollider target = CreateBodyless3D(
                context, new LSSphereCollider { Radius = Fixed64.Half },
                Vector3d.Right * Fixed64.Half);
            CollisionDetectionMixed.TryCollide(target, source.Collider, out _).Should().BeTrue();
        }
        else
        {
            LSCollider2D target = CreateBodylessCircle2D(context, Vector2d.Right * Fixed64.Half);
            CollisionDetection2D.TryCollide(source.Collider, target, out Contact2D contact).Should().BeTrue();
            Vector2d.Dot(velocity, contact.Normal).Should().BeGreaterThan(Fixed64.Zero);
        }
        Fixed64 angularVelocity = Fixed64.FromFraction(1, 100);
        source.AddLinearImpulse(velocity);
        source.AddAngularImpulse(angularVelocity / source.EffectiveInverseMomentOfInertia);

        // Observe the real CCD operator before the subsequent discrete contact solve.
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        source.LateSimulate(updateSleepState: false, updateColliderState: false);

        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
        source.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
        source.AngularVelocity.Should().Be(Fixed64.Zero);
        source.SampleContinuousCollisionAngularVelocity(Fixed64.One).Should().Be(Fixed64.Zero);
        source.SampleContinuousCollisionLinearVelocity(Fixed64.One).Should().Be(Vector2d.Zero);
        if (mixedTarget)
            source.LinearVelocity.Should().Be(velocity);
        else
            source.LinearVelocity.MagnitudeSquared.Should().BeLessThan(velocity.MagnitudeSquared);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Continuous3D_WitnessedStaticContact_ShouldStopRotationWithoutUnresolvedLimit(
        bool mixedTarget)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        ScenarioBody<LSCuboidCollider> source = CreateRefinementSource3D(context, Vector3d.Zero);
        source.Body.UseManualGrounding();
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector3d velocity = Vector3d.Right * Fixed64.FromFraction(mixedTarget ? -1 : 1, 10);
        if (mixedTarget)
        {
            LSCollider2D target = CreateBodylessCircle2D(context, Vector2d.Right * Fixed64.Half);
            CollisionDetectionMixed.TryCollide(source.Collider, target, out _).Should().BeTrue();
        }
        else
        {
            LSMeshCollider target = CreateBodyless3D(
                context,
                MeshTestFixtures.CreateConvexCube(inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
                Vector3d.Right * Fixed64.FromFraction(5, 8));
            source.Collider.Priority.Should().BeLessThan(target.Priority);
            var pair = new CollisionPair(source.Collider, target);
            CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
            Vector3d towardTarget = pair.ColliderA == source.Collider
                ? pair.Manifold.PrimaryContact.Normal
                : -pair.Manifold.PrimaryContact.Normal;
            Vector3d.Dot(velocity, towardTarget).Should().BeGreaterThan(Fixed64.Zero);
        }
        Fixed64 angularVelocity = Fixed64.FromFraction(1, 100);
        source.Body.AddLinearImpulse(velocity);
        source.Body.AddAngularImpulse(
            Vector3d.Up * (angularVelocity / source.Body.EffectiveInverseInertiaTensor.M22));

        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        source.Body.LateSimulate(updateSleepState: false, updateColliderState: false);

        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        source.Body.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
        source.Body.AngularVelocity.Should().Be(Vector3d.Zero);
        source.Body.SampleContinuousCollisionAngularVelocity(Fixed64.One).Should().Be(Vector3d.Zero);
        source.Body.SampleContinuousCollisionLinearVelocity(Fixed64.One).Should().Be(Vector3d.Zero);
        if (mixedTarget)
            source.Body.LinearVelocity.Should().Be(velocity);
        else
            source.Body.LinearVelocity.MagnitudeSquared.Should().BeLessThan(velocity.MagnitudeSquared);
    }
}
