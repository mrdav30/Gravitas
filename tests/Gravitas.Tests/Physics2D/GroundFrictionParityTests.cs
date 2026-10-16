using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Support;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class GroundFrictionParityTests
{
    public static TheoryData<PhysicsRuntimeMode, int, int> DimensionsAndScales
    {
        get
        {
            var cases = new TheoryData<PhysicsRuntimeMode, int, int>();
            foreach (PhysicsRuntimeMode mode in new[] { PhysicsRuntimeMode.TwoD, PhysicsRuntimeMode.Mixed })
            foreach (int rate in new[] { 4, 64 })
            foreach (int sizeDivisor in new[] { 1, 8 })
                cases.Add(mode, rate, sizeDivisor);
            return cases;
        }
    }

    [Theory]
    [MemberData(nameof(DimensionsAndScales))]
    public void ManualGroundingWithoutContact_ShouldNotIntroduceSyntheticPlanarFriction(
        PhysicsRuntimeMode mode, int rate, int sizeDivisor)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, mode, rate);
        Fixed64 radius = Fixed64.FromFraction(1, 2 * sizeDivisor);
        var circle = new LSCircleCollider2D(radius)
        {
            Material = new PhysicsMaterial(Fixed64.One, Fixed64.One, Fixed64.Zero)
        };
        SolidBody2D body = CreateBody(scene.Context, circle, Vector2d.Zero);
        body.Gravity = -Vector2d.Forward;
        body.SetManualGrounding(-Vector2d.Forward * radius, Vector2d.Forward);
        body.AddLinearImpulse(Vector2d.Right);
        body.AddAngularImpulse(body.MomentOfInertia);

        for (int frame = 0; frame < 4; frame++)
        {
            if (frame == 2)
            {
                body.AddLinearImpulse(Vector2d.Left * Fixed64.Two);
                body.AddAngularImpulse(-Fixed64.Two * body.MomentOfInertia);
            }
            scene.Context.Simulate();
            scene.Context.LateSimulate();

            Fixed64 sign = frame < 2 ? Fixed64.One : -Fixed64.One;
            body.LinearVelocity.Should().Be(Vector2d.Right * sign);
            body.AngularVelocity.Should().Be(sign);
            body.IsGrounded.Should().BeTrue();
        }
        (circle.CollisionPairCount + circle.CollisionPairHolderCount).Should().Be(0);
        scene.Context.MixedCollisions.ActivePairCount.Should().Be(0);
    }

    [Theory]
    [MemberData(nameof(DimensionsAndScales))]
    public void PhysicalContactFriction_ShouldReduceSlipWithoutReversalInEitherDirection(
        PhysicsRuntimeMode mode, int rate, int sizeDivisor)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, mode, rate);
        var material = new PhysicsMaterial((Fixed64)64, (Fixed64)64, Fixed64.Zero);
        var floor = new LSAABBoxCollider2D(new Vector2d(8, 1)) { Material = material };
        floor.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(
            new Vector3d(Fixed64.Zero, Fixed64.Zero, -Fixed64.Half),
            FixedQuaternion.Identity, Vector3d.One)));
        Fixed64 size = Fixed64.FromFraction(1, sizeDivisor);
        var box = new LSAABBoxCollider2D(new Vector2d(size, size)) { Material = material };
        SolidBody2D body = CreateBody(scene.Context, box, Vector2d.Forward * (size * Fixed64.Half));
        body.FreezeAxes = BodyFreezeAxes2D.Rotation;
        body.UseManualGrounding();
        Fixed64 slip = Fixed64.FromFraction(1, 16);

        foreach (Fixed64 sign in new[] { Fixed64.One, -Fixed64.One })
        {
            Vector2d incoming = new(sign * slip, -slip);
            body.AddLinearImpulse(incoming - body.LinearVelocity);
            scene.Context.Simulate();
            scene.Context.LateSimulate();

            bool found = box.TryGetCollisionPair(floor.Id, out CollisionPair2D? pair)
                || floor.TryGetCollisionPair(box.Id, out pair);
            found.Should().BeTrue();
            pair!.Manifold.HasContact.Should().BeTrue();
            (body.LinearVelocity.X * sign).Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
            body.LinearVelocity.X.Abs().Should().BeLessThan(slip);
            body.LinearVelocity.MagnitudeSquared.Should().BeLessThanOrEqualTo(incoming.MagnitudeSquared);
            body.AngularVelocity.Should().Be(Fixed64.Zero);
        }
    }

    private static SolidBody2D CreateBody(GravitasWorldContext context, LSCollider2D collider, Vector2d position)
    {
        var body = new SolidBody2D(new TestMatterAgent(context, new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One)), collider)
        {
            Mass = Fixed64.One,
            Gravity = Vector2d.Zero,
            SleepEnabled = false
        };
        body.Initialize(position);
        return body;
    }

    private static void Configure(GravitasWorldContext context, PhysicsRuntimeMode mode, int rate)
    {
        context.SetFrameRate(rate);
        context.Settings.RuntimeMode = mode;
        context.Environment.Gravity = Fixed64.One;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        context.Environment.MinSpeed = Fixed64.Zero;
    }
}
