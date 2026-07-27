using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Diagnostics;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using SwiftCollections.Diagnostics;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedResponseTests
{
    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void Resolve_WithUnrepresentableMixedLeverArm_ShouldRejectAtomically(
        bool loggingEnabled)
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D =
            CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        var contact = new MixedContact(
            new ContactAnchor(
                new Vector3d(
                    Fixed64.MaxValue,
                    Fixed64.Zero,
                    Fixed64.Zero),
                Vector3d.Right),
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            Vector3d.Right,
            Fixed64.Half);
        Vector3d velocity3D = body3D.Body.LinearVelocity;
        Vector2d velocity2D = body2D.LinearVelocity;
        string? loggedMessage = null;
        DiagnosticLevel originalLevel = GravitasLogger.MinimumLevel;
        var originalHandler = GravitasLogger.LogHandler;

        try
        {
            GravitasLogger.MinimumLevel = loggingEnabled
                ? DiagnosticLevel.Error
                : DiagnosticLevel.None;
            GravitasLogger.LogHandler =
                (_, message, _) => loggedMessage = message;

            CollisionResponseMixed.Resolve(pair, contact).Should().BeFalse();
        }
        finally
        {
            GravitasLogger.MinimumLevel = originalLevel;
            GravitasLogger.LogHandler = originalHandler;
        }

        body3D.Body.LinearVelocity.Should().Be(velocity3D);
        body2D.LinearVelocity.Should().Be(velocity2D);
        if (loggingEnabled)
        {
            loggedMessage.Should().Contain(
                "cannot be rebased onto its response centers");
        }
        else
        {
            loggedMessage.Should().BeNull();
        }
    }

    [Fact]
    public void Simulate_WithOffCenterVerticalContact_ShouldApplyFrictionOnlyTo3DParticipant()
    {
        PhysicsMaterial frictional = new((Fixed64)2, (Fixed64)2, Fixed64.Zero);
        var frictionlessResult = RunOffCenterVerticalFrictionScenario(PhysicsMaterial.Frictionless);
        var frictionalResult = RunOffCenterVerticalFrictionScenario(frictional);

        frictionalResult.NormalImpulsePlanar.Should().Be(Vector2d.Zero);
        frictionalResult.ContactPoint2D.X.Should().NotBe(Fixed64.Zero);
        frictionalResult.Sleeping2D.Should().BeTrue();
        frictionalResult.LinearVelocity2D.Should().Be(Vector2d.Zero);
        frictionalResult.AngularVelocity2D.Should().Be(Fixed64.Zero);
        frictionalResult.VerticalVelocity3D.Should().BeGreaterThan(-Fixed64.One);
        frictionalResult.PlanarSpeed3D.Should().BeLessThan(frictionlessResult.PlanarSpeed3D);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void Simulate_WithVerticalOnlyContact_ShouldKeepWakeStateIndependentOfUnrelatedResponsePair(bool sleep3D)
    {
        (bool Sleeping3D, bool Sleeping2D) isolated = RunVerticalWakeScenario(includeUnrelatedPair: false, sleep3D);
        (bool Sleeping3D, bool Sleeping2D) withUnrelatedPair = RunVerticalWakeScenario(includeUnrelatedPair: true, sleep3D);

        withUnrelatedPair.Should().Be(isolated);
        isolated.Sleeping3D.Should().Be(sleep3D);
        isolated.Sleeping2D.Should().Be(!sleep3D);
    }

    [Fact]
    public void Simulate_WithMultipleVerticalOnlyMixedContacts_ShouldNotWakeOrConnectSleeping2DParticipants()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> left3D = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-3), Fixed64.FromFraction(3, 4), Fixed64.Zero));
        ScenarioBody<LSSphereCollider> right3D = CreateSphere3D(
            context,
            new Vector3d((Fixed64)3, Fixed64.FromFraction(3, 4), Fixed64.Zero));
        SolidBody2D left2D = CreateCircle2D(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));
        SolidBody2D right2D = CreateCircle2D(context, new Vector2d((Fixed64)3, Fixed64.Zero));
        PhysicsMaterial frictional = new((Fixed64)2, (Fixed64)2, Fixed64.Zero);
        left3D.Collider.Material = frictional;
        right3D.Collider.Material = frictional;
        left2D.Collider.Material = frictional;
        right2D.Collider.Material = frictional;
        Step(context);
        context.MixedCollisions.ActivePairCount.Should().Be(2);
        left3D.Body.SetPosition(new Vector3d((Fixed64)(-3), Fixed64.FromFraction(3, 4), Fixed64.Zero));
        right3D.Body.SetPosition(new Vector3d((Fixed64)3, Fixed64.FromFraction(3, 4), Fixed64.Zero));
        left2D.SetPosition(new Vector2d((Fixed64)(-3), Fixed64.Zero));
        right2D.SetPosition(new Vector2d((Fixed64)3, Fixed64.Zero));
        left2D.Sleep();
        right2D.Sleep();
        left3D.Body.AddLinearImpulse(new Vector3d(Fixed64.One, -Fixed64.One, Fixed64.Zero));
        right3D.Body.AddLinearImpulse(new Vector3d(-Fixed64.One, -Fixed64.One, Fixed64.Zero));

        Step(context);

        left2D.IsSleeping.Should().BeTrue();
        right2D.IsSleeping.Should().BeTrue();
        left2D.LinearVelocity.Should().Be(Vector2d.Zero);
        right2D.LinearVelocity.Should().Be(Vector2d.Zero);
        left2D.AngularVelocity.Should().Be(Fixed64.Zero);
        right2D.AngularVelocity.Should().Be(Fixed64.Zero);
        left3D.Body.LinearVelocity.Y.Should().BeGreaterThan(-Fixed64.One);
        right3D.Body.LinearVelocity.Y.Should().BeGreaterThan(-Fixed64.One);
    }

    private static (bool Sleeping3D, bool Sleeping2D) RunVerticalWakeScenario(
        bool includeUnrelatedPair,
        bool sleep3D)
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero));
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        if (includeUnrelatedPair)
        {
            ScenarioBody<LSSphereCollider> unrelated3D = CreateSphere3D(
                context,
                new Vector3d((Fixed64)5, Fixed64.FromFraction(3, 4), Fixed64.Zero));
            _ = CreateCircle2D(context, new Vector2d((Fixed64)5, Fixed64.Zero));
            unrelated3D.Body.AddLinearImpulse(-Vector3d.Up);
        }

        Step(context);
        context.MixedCollisions.ActivePairCount.Should().Be(includeUnrelatedPair ? 2 : 1);
        body3D.Body.SetPosition(new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero));
        body2D.SetPosition(Vector2d.Zero);
        if (sleep3D)
        {
            body3D.Body.Sleep();
            body2D.AddLinearImpulse(Vector2d.Right);
        }
        else
        {
            body2D.Sleep();
            body3D.Body.AddLinearImpulse(new Vector3d(Fixed64.One, -Fixed64.One, Fixed64.Zero));
        }

        Step(context);

        return (body3D.Body.IsSleeping, body2D.IsSleeping);
    }

    private static (
        Fixed64 PlanarSpeed3D,
        Fixed64 VerticalVelocity3D,
        bool Sleeping2D,
        Vector2d LinearVelocity2D,
        Fixed64 AngularVelocity2D,
        Vector2d NormalImpulsePlanar,
        Vector2d ContactPoint2D) RunOffCenterVerticalFrictionScenario(PhysicsMaterial material)
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Diagnostics.Enable(eventCapacity: 16, drawCommandCapacity: 0);
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Half, Fixed64.FromFraction(3, 4), Fixed64.Zero));
        SolidBody2D body2D = CreateBox2DTask7(context, Vector2d.Zero, new Vector2d((Fixed64)4, (Fixed64)4));
        body3D.Collider.Material = material;
        body2D.Collider.Material = material;
        Step(context);
        context.MixedCollisions.ActivePairCount.Should().Be(1);
        context.Diagnostics.Clear();
        body3D.Body.SetPosition(new Vector3d(Fixed64.Half, Fixed64.FromFraction(3, 4), Fixed64.Zero));
        body2D.SetPosition(Vector2d.Zero);
        body2D.Sleep();
        body3D.Body.AddLinearImpulse(new Vector3d(Fixed64.Zero, -Fixed64.One, Fixed64.One));

        Step(context);

        GravitasMixedResponseImpulseDiagnosticView impulse = FindFirstMixedImpulse(context);
        return (
            body3D.Body.LinearVelocity.Z.Abs(),
            body3D.Body.LinearVelocity.Y,
            body2D.IsSleeping,
            body2D.LinearVelocity,
            body2D.AngularVelocity,
            impulse.Impulse.ToVector2d(),
            impulse.Point2D.ToVector2d());
    }

    private static SolidBody2D CreateBox2DTask7(
        GravitasWorldContext context,
        Vector2d position,
        Vector2d size)
    {
        var collider = new LSAABBoxCollider2D(size);
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(
                new Vector3d(position.X, Fixed64.Zero, position.Y),
                FixedQuaternion.Identity,
                Vector3d.One));
        var body = new SolidBody2D(agent, collider) { Mass = Fixed64.One };
        collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero);
        body.Initialize(position, motionType: BodyMotionType.Dynamic);
        return body;
    }
}
