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
    public void Continuous3D_ClosingMixedContact_WithFullReceiverTrajectory_ShouldStopWithoutMutatingReceiver()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = context.Environment.AirDensity = context.Environment.DampingFactor = Fixed64.Zero;
        context.Settings.ContinuousCollisionMaxToiIterations = 1;
        ScenarioBody<LSCuboidCollider> source = CreateRefinementSource3D(context, Vector3d.Zero);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        SolidBody2D receiver = CreateCircle2D(context, Vector2d.Right * Fixed64.Half);
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        receiver.ApplyContinuousCollisionHandoffState(receiver.Position, receiver.Rotation,
            Vector2d.Forward * Fixed64.FromFraction(1, 100), Fixed64.Zero, Fixed64.FromFraction(3, 4)).Should().BeTrue();
        receiver.CanApplyContinuousCollisionHandoffState(receiver.Position, receiver.Rotation,
            Fixed64.Half, out _).Should().BeFalse();
        source.Body.ApplyContinuousCollisionHandoff(source.Body.Position3d, source.Body.Rotation,
            Vector3d.Right * Fixed64.FromFraction(1, 10), Vector3d.Forward * Fixed64.FromFraction(1, 100), Fixed64.Half).Should().BeTrue();
        var manifold = new ContactManifold();
        CollisionDetectionMixed.TryBuildDiscreteManifold(source.Collider, receiver.Collider, manifold).Should().BeTrue();
        MixedContact.TrySelectClosingContact(manifold, source.Body.GetCenterOfMassAnchor(),
            source.Body.LinearVelocity, source.Body.AngularVelocity,
            ContactAnchor.FromWorldPoint(receiver.Position.ToVector3d(receiver.Collider.MixedSlabCenterY)),
            receiver.LinearVelocity, receiver.AngularVelocity, out _, receiver.Collider).Should().BeTrue();
        Vector2d receiverPosition = receiver.Position;
        Vector2d receiverVelocity = receiver.LinearVelocity;
        Vector2d receiverContinuation = receiver.SampleContinuousCollisionPosition(Fixed64.One);

        source.Body.TryConsumeContinuousCollisionHandoff(updateSleepState: false, updateColliderState: false).Should().BeTrue();

        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        source.Body.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
        source.Body.AngularVelocity.Should().Be(Vector3d.Zero);
        source.Body.LinearVelocity.MagnitudeSquared.Should().BeLessThan(Fixed64.FromFraction(1, 100));
        receiver.Position.Should().Be(receiverPosition);
        receiver.LinearVelocity.Should().Be(receiverVelocity);
        receiver.SampleContinuousCollisionPosition(Fixed64.One).Should().Be(receiverContinuation);
    }

    [Fact]
    public void Continuous2D_ClosingMixedContact_WithFullReceiverTrajectory_ShouldStopWithoutMutatingReceiver()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = context.Environment.AirDensity = context.Environment.DampingFactor = Fixed64.Zero;
        context.Settings.ContinuousCollisionMaxToiIterations = 1;
        SolidBody2D source = CreateRefinementSource2D(context, Vector2d.Zero);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        ScenarioBody<LSSphereCollider> receiver = CreateSphere3D(context, Vector3d.Right * Fixed64.Half);
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        // An earlier real handoff consumes the receiver's single mutation
        // budget. The source arrives through its own later queued continuation.
        receiver.Body.ApplyContinuousCollisionHandoff(receiver.Body.Position3d, receiver.Body.Rotation,
            Vector3d.Forward * Fixed64.FromFraction(1, 100), Vector3d.Zero, Fixed64.FromFraction(3, 4)).Should().BeTrue();
        receiver.Body.CanApplyContinuousCollisionHandoff(receiver.Body.Position3d, receiver.Body.Rotation,
            Fixed64.Half, out _).Should().BeFalse();
        source.ApplyContinuousCollisionHandoffState(source.Position, source.Rotation,
            Vector2d.Right * Fixed64.FromFraction(1, 10), Fixed64.FromFraction(1, 100), Fixed64.Half).Should().BeTrue();
        var manifold = new ContactManifold();
        CollisionDetectionMixed.TryBuildDiscreteManifold(receiver.Collider, source.Collider, manifold).Should().BeTrue();
        MixedContact.TrySelectClosingContact(manifold, receiver.Body.GetCenterOfMassAnchor(),
            receiver.Body.LinearVelocity, receiver.Body.AngularVelocity,
            ContactAnchor.FromWorldPoint(source.Position.ToVector3d(source.Collider.MixedSlabCenterY)),
            source.LinearVelocity, source.AngularVelocity, out _, source.Collider).Should().BeTrue();
        Vector3d receiverPosition = receiver.Body.Position3d;
        Vector3d receiverVelocity = receiver.Body.LinearVelocity;
        Vector3d receiverContinuation = receiver.Body.SampleContinuousCollisionPosition(Fixed64.One);

        source.TryConsumeContinuousCollisionHandoff(updateSleepState: false, updateColliderState: false).Should().BeTrue();

        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
        source.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
        source.AngularVelocity.Should().Be(Fixed64.Zero);
        source.LinearVelocity.MagnitudeSquared.Should().BeLessThan(Fixed64.FromFraction(1, 100));
        receiver.Body.Position3d.Should().Be(receiverPosition);
        receiver.Body.LinearVelocity.Should().Be(receiverVelocity);
        receiver.Body.SampleContinuousCollisionPosition(Fixed64.One).Should().Be(receiverContinuation);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    public void Continuous2D_WitnessedStaticContact_ShouldStopOnlyWhenResponseIsAuthorized(
        bool mixedTarget, bool zeroTranslation)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        SolidBody2D source = CreateRefinementSource2D(context, Vector2d.Zero);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector2d velocity = zeroTranslation ? Vector2d.Zero
            : Vector2d.Right * Fixed64.FromFraction(mixedTarget ? -1 : 1, 10);
        LSSphereCollider? mixedCollider = null;
        if (mixedTarget)
        {
            LSSphereCollider target = CreateBodyless3D(
                context, new LSSphereCollider { Radius = Fixed64.Half },
                Vector3d.Right * Fixed64.Half);
            mixedCollider = target;
            CollisionDetectionMixed.TryCollide(target, source.Collider, out _).Should().BeTrue();
        }
        else
        {
            LSCollider2D target = CreateBodylessCircle2D(context, Vector2d.Right * Fixed64.Half);
            CollisionDetection2D.TryCollide(source.Collider, target, out Contact2D contact).Should().BeTrue();
            if (zeroTranslation)
                Vector2d.Dot(velocity, contact.Normal).Should().Be(Fixed64.Zero);
            else
                Vector2d.Dot(velocity, contact.Normal).Should().BeGreaterThan(Fixed64.Zero);
        }
        Fixed64 angularVelocity = Fixed64.FromFraction(1, 100);
        source.AddLinearImpulse(velocity);
        source.AddAngularImpulse(angularVelocity / source.EffectiveInverseMomentOfInertia);
        Fixed64 originalAngularVelocity = source.AngularVelocity;
        if (mixedTarget)
        {
            var manifold = new ContactManifold();
            CollisionDetectionMixed.TryBuildDiscreteManifold(mixedCollider!, source.Collider, manifold).Should().BeTrue();
            // The symmetric source is centered on its body origin. Every mixed
            // row separates, including the actual angular point velocities.
            MixedContact.TrySelectClosingContact(manifold, ContactAnchor.FromWorldPoint(mixedCollider!.Center),
                Vector3d.Zero, Vector3d.Zero,
                ContactAnchor.FromWorldPoint(source.Position.ToVector3d(source.Collider.MixedSlabCenterY)),
                source.LinearVelocity, source.AngularVelocity, out _, source.Collider).Should().BeFalse();
        }

        // Observe the real CCD operator before the subsequent discrete contact solve.
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        source.LateSimulate(updateSleepState: false, updateColliderState: false);

        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
        source.LastContinuousCollisionToiIterationLimitReached.Should().Be(mixedTarget);
        source.AngularVelocity.Should().Be(mixedTarget ? originalAngularVelocity : Fixed64.Zero);
        source.SampleContinuousCollisionAngularVelocity(Fixed64.One).Should().Be(Fixed64.Zero);
        source.SampleContinuousCollisionLinearVelocity(Fixed64.One).Should().Be(Vector2d.Zero);
        if (mixedTarget || zeroTranslation)
            source.LinearVelocity.Should().Be(velocity);
        else
            source.LinearVelocity.MagnitudeSquared.Should().BeLessThan(velocity.MagnitudeSquared);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Continuous3D_WitnessedStaticContact_ShouldStopOnlyWhenResponseIsAuthorized(
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
        LSCollider2D? mixedCollider = null;
        if (mixedTarget)
        {
            LSCollider2D target = CreateBodylessCircle2D(context, Vector2d.Right * Fixed64.Half);
            mixedCollider = target;
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
        Vector3d originalAngularVelocity = source.Body.AngularVelocity;
        if (mixedTarget)
        {
            var manifold = new ContactManifold();
            CollisionDetectionMixed.TryBuildDiscreteManifold(source.Collider, mixedCollider!, manifold).Should().BeTrue();
            MixedContact.TrySelectClosingContact(manifold, source.Body.GetCenterOfMassAnchor(),
                source.Body.LinearVelocity, source.Body.AngularVelocity,
                ContactAnchor.FromWorldPoint(new(mixedCollider!.Center.X, mixedCollider.MixedSlabCenterY, mixedCollider.Center.Y)),
                Vector2d.Zero, Fixed64.Zero, out _, mixedCollider).Should().BeFalse();
        }

        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        source.Body.LateSimulate(updateSleepState: false, updateColliderState: false);

        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        source.Body.LastContinuousCollisionToiIterationLimitReached.Should().Be(mixedTarget);
        source.Body.AngularVelocity.Should().Be(mixedTarget ? originalAngularVelocity : Vector3d.Zero);
        source.Body.SampleContinuousCollisionAngularVelocity(Fixed64.One).Should().Be(Vector3d.Zero);
        source.Body.SampleContinuousCollisionLinearVelocity(Fixed64.One).Should().Be(Vector3d.Zero);
        if (mixedTarget)
            source.Body.LinearVelocity.Should().Be(velocity);
        else
            source.Body.LinearVelocity.MagnitudeSquared.Should().BeLessThan(velocity.MagnitudeSquared);
    }
}
