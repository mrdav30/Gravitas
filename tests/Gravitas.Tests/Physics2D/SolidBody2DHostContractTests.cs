using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class SolidBody2DHostContractTests
{
    [Fact]
    public void Constructor_ShouldBindAgentContextAndCollider()
    {
        using GravitasWorldContext context = Create2DContext();
        var transform = new FixedTransform(new Vector3d((Fixed64)2, (Fixed64)7, (Fixed64)3), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var collider = new LSCircleCollider2D(Fixed64.One);

        var body = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(transform.WorldPositionXZ);

        body.Agent.Should().BeSameAs(agent);
        body.Context.Should().BeSameAs(context);
        body.Collider.Should().BeSameAs(collider);
        body.Position.Should().Be(new Vector2d((Fixed64)2, (Fixed64)3));
        collider.Agent.Should().BeSameAs(agent);
        collider.Context.Should().BeSameAs(context);
    }

    [Fact]
    public void LateSimulate_WithKinematicBody_ShouldProjectHostTransformXZIntoPure2DPosition()
    {
        using GravitasWorldContext context = Create2DContext();
        var transform = new FixedTransform(new Vector3d(Fixed64.One, (Fixed64)9, (Fixed64)2), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            IsKinematic = true,
            Mass = Fixed64.One
        };
        body.Initialize(transform.WorldPositionXZ);

        transform.LocalPosition = new Vector3d((Fixed64)5, (Fixed64)11, (Fixed64)7);
        context.LateSimulate();

        body.Position.Should().Be(new Vector2d((Fixed64)5, (Fixed64)7));
        body.Collider.Center.Should().Be(new Vector2d((Fixed64)5, (Fixed64)7));
        var hits = new SwiftList<Physics2DHit>();
        context.Query2D.OverlapCircleAll(new Vector2d((Fixed64)5, (Fixed64)7), Fixed64.Half, hits).Should().Be(1);
    }

    [Fact]
    public void Visualize_WithDynamic2DBody_ShouldPublishPlanarStateToHostTransform()
    {
        using GravitasWorldContext context = Create2DContext();
        var transform = new FixedTransform(new Vector3d(Fixed64.Zero, (Fixed64)9, Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One
        };
        body.Initialize(Vector2d.Zero);

        body.SetPosition(new Vector2d((Fixed64)4, (Fixed64)6));
        body.SetRotation(FixedMath.DegToRad((Fixed64)90));
        context.Visualize();

        transform.WorldPosition.Should().Be(new Vector3d((Fixed64)4, (Fixed64)9, (Fixed64)6));
        FixedQuaternion.Angle(
            transform.WorldRotation,
            FixedQuaternion.FromAxisAngle(Vector3d.Up, -(Fixed64.Pi * Fixed64.Half)))
            .Should().BeLessThan(Fixed64.Epsilon);
    }

    [Fact]
    public void Visualize_WhenRuntimeModeIsThreeD_ShouldNotPublish2DTransform()
    {
        using GravitasWorldContext context = Create2DContext();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.ThreeD;
        var transform = new FixedTransform(new Vector3d(Fixed64.Zero, (Fixed64)9, Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One
        };
        body.Initialize(Vector2d.Zero);

        body.SetPosition(new Vector2d((Fixed64)4, (Fixed64)6));
        context.Visualize();

        transform.WorldPosition.Should().Be(new Vector3d(Fixed64.Zero, (Fixed64)9, Fixed64.Zero));
    }

    [Fact]
    public void DeactivatedBody_ShouldKeepResetPoseAndIgnoreRepeatedCleanupAndDirectLateStep()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D body = CreateDynamicCircle(context, Vector2d.Zero);
        Vector2d resetPosition = new((Fixed64)3, (Fixed64)4);

        body.Deactivate();
        body.Deactivate();
        body.ResetPosition(resetPosition, Fixed64.Half);
        body.LateSimulate();

        body.Active.Should().BeFalse();
        body.DynamicId.Should().Be(-1);
        body.Position.Should().Be(resetPosition);
        body.Rotation.Should().Be(Fixed64.Half);
        context.Physics2D.ColliderCount.Should().Be(0);
    }

    [Fact]
    public void DirectLateSimulate_WithOrdinaryDynamicMotion_ShouldRefreshColliderAndSleepState()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D body = CreateDynamicCircle(context, Vector2d.Zero);
        body.SleepFrameThreshold = 1;
        body.SleepLinearSpeedThreshold = (Fixed64)10;
        body.SleepAngularSpeedThreshold = (Fixed64)10;
        body.AddForce(Vector2d.Right);

        body.LateSimulate();

        body.Position.X.Should().BeGreaterThan(Fixed64.Zero);
        body.Collider.Center.Should().Be(body.Position);
        body.IsSleeping.Should().BeTrue();
        body.LinearVelocity.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void DirectLateSimulate_WithKinematicBody_ShouldRefreshColliderAndPreserveHostVisualization()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D body = CreateDynamicCircle(context, Vector2d.Zero);
        FixedTransform transform = body.Agent.Transform;
        Vector3d hostPosition = new((Fixed64)5, (Fixed64)9, (Fixed64)7);
        Fixed64 hostRotation = Fixed64.Pi * Fixed64.Half;
        body.IsKinematic = true;
        transform.LocalPosition = hostPosition;
        transform.LocalRotationXZRadians = hostRotation;

        body.LateSimulate();

        body.Position.Should().Be(new Vector2d((Fixed64)5, (Fixed64)7));
        body.Collider.Center.Should().Be(body.Position);
        body.Rotation.Should().Be(hostRotation);

        context.Visualize();

        transform.WorldPosition.Should().Be(hostPosition);
        transform.WorldRotationXZRadians.Should().Be(hostRotation);
    }

    [Fact]
    public void InitializeWithNoBody_ShouldBindStaticColliderToAgentAndQueries()
    {
        using GravitasWorldContext context = Create2DContext();
        var transform = new FixedTransform(new Vector3d((Fixed64)4, (Fixed64)8, (Fixed64)6), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var collider = new LSCircleCollider2D(Fixed64.Half);

        collider.InitializeWithNoBody(agent);

        collider.Agent.Should().BeSameAs(agent);
        collider.Body.Should().BeNull();
        collider.Center.Should().Be(new Vector2d((Fixed64)4, (Fixed64)6));
        context.Physics2D.ColliderCount.Should().Be(1);
        var hits = new SwiftList<Physics2DHit>();
        context.Query2D.OverlapCircleAll(new Vector2d((Fixed64)4, (Fixed64)6), Fixed64.Half, hits).Should().Be(1);
    }

    [Fact]
    public void LateSimulate_WithMovedBodylessCollider_ShouldRefreshBoundsAndPartitionsFromAgentTransform()
    {
        using GravitasWorldContext context = Create2DContext();
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var collider = new LSCircleCollider2D(Fixed64.Half);
        var hits = new SwiftList<Physics2DHit>();

        collider.InitializeWithNoBody(agent);

        transform.LocalPosition = new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero);
        Step(context);

        collider.Center.Should().Be(new Vector2d((Fixed64)4, Fixed64.Zero));
        context.Query2D.OverlapCircleAll(new Vector2d((Fixed64)4, Fixed64.Zero), Fixed64.Half, hits).Should().Be(1);
        hits[0].Collider.Should().BeSameAs(collider);
        context.Query2D.OverlapCircleAll(Vector2d.Zero, Fixed64.Half, hits).Should().Be(0);
    }

    [Fact]
    public void Simulate_WithBodylessStaticCollider_ShouldResolveDynamicBody()
    {
        using GravitasWorldContext context = Create2DContext();
        var staticTransform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        var staticAgent = new TestMatterAgent(context, staticTransform);
        var staticCollider = new LSCircleCollider2D(Fixed64.Half);
        staticCollider.InitializeWithNoBody(staticAgent);
        SolidBody2D dynamicBody = CreateDynamicCircle(context, new Vector2d(Fixed64.Half, Fixed64.Zero));

        Step(context);

        dynamicBody.Position.X.Should().BeGreaterThan(Fixed64.Half);
    }

    [Fact]
    public void Simulate_WithSameAgent2DColliders_ShouldSkipCollision()
    {
        using GravitasWorldContext context = Create2DContext();
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var first = new LSCircleCollider2D(Fixed64.Half) { IsTrigger = true };
        var second = new LSCircleCollider2D(Fixed64.Half) { IsTrigger = true };
        int triggerCount = 0;
        first.OnTriggerEnter += _ => triggerCount++;

        first.InitializeWithNoBody(agent);
        second.InitializeWithNoBody(agent);
        Step(context);

        triggerCount.Should().Be(0);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void PlanarHostRotation_ShouldPreserveSignedHalfPiAcrossPublishAndKinematicReadback(int sign)
    {
        using GravitasWorldContext context = Create2DContext();
        var transform = new FixedTransform(
            new Vector3d(Fixed64.Zero, (Fixed64)9, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);
        var body = new SolidBody2D(
            new TestMatterAgent(context, transform),
            new LSAABBoxCollider2D(new Vector2d((Fixed64)2, Fixed64.One)))
        {
            Mass = Fixed64.One
        };
        Fixed64 halfPi = Fixed64.Pi * Fixed64.Half;
        Fixed64 angle = sign > 0 ? halfPi : -halfPi;
        body.Initialize(Vector2d.Zero);

        body.SetRotation(angle);
        context.Visualize();

        Vector2d embeddedRight = transform.WorldRotation.Rotate(Vector3d.Right).ToVector2d();
        Vector2d expectedRight = Vector2d.Rotate(Vector2d.Right, angle);
        embeddedRight.FuzzyEqualAbsolute(expectedRight, Fixed64.Epsilon).Should().BeTrue();
        transform.WorldRotationXZRadians.Should().Be(angle);

        body.IsKinematic = true;
        transform.LocalRotationXZRadians = -angle;
        context.LateSimulate();

        body.Rotation.Should().Be(-angle);
    }

    [Fact]
    public void PlanarRotationAssignments_ShouldUseOneCanonicalHalfOpenRepresentative()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D body = CreateDynamicCircle(context, Vector2d.Zero, Fixed64.Pi);

        body.Rotation.Should().Be(-Fixed64.Pi);

        body.ResetPosition(Vector2d.Zero, -Fixed64.Pi);
        body.Rotation.Should().Be(-Fixed64.Pi);

        body.SetRotation(Fixed64.Pi * (Fixed64)3);
        body.Rotation.Should().Be(-Fixed64.Pi);

        body.SetRotation(-Fixed64.Pi * (Fixed64)3);
        body.Rotation.Should().Be(-Fixed64.Pi);

        body.SetRotation(Fixed64.TwoPi + Fixed64.Half);
        body.Rotation.Should().Be(Fixed64.Half);

        body.SetRotation(-Fixed64.TwoPi - Fixed64.Half);
        body.Rotation.Should().Be(-Fixed64.Half);
    }

    [Fact]
    public void DynamicAngularIntegration_ShouldCanonicalizeWhenCrossingPositivePi()
    {
        using GravitasWorldContext context = Create2DContext();
        SolidBody2D body = CreateDynamicCircle(context, Vector2d.Zero);
        body.SleepEnabled = false;
        body.SetRotation(Fixed64.Pi - Fixed64.FromFraction(1, 100));
        body.AddAngularImpulse(Fixed64.One);

        body.LateSimulate();

        body.Rotation.Should().BeGreaterThanOrEqualTo(-Fixed64.Pi);
        body.Rotation.Should().BeLessThan(Fixed64.Pi);
        body.Rotation.Should().BeLessThan(Fixed64.Zero);
    }

    [Fact]
    public void ParentedHostTransform_ShouldSynchronizeUsingWorldPlanarPoseAndPreserveWorldY()
    {
        using GravitasWorldContext context = Create2DContext();
        var parent = new FixedTransform(
            new Vector3d((Fixed64)10, (Fixed64)3, (Fixed64)(-4)),
            FixedQuaternion.Identity,
            Vector3d.One);
        var child = new FixedTransform(
            new Vector3d((Fixed64)2, (Fixed64)6, (Fixed64)5),
            FixedQuaternion.Identity,
            Vector3d.One,
            parent);
        var body = new SolidBody2D(
            new TestMatterAgent(context, child),
            new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One
        };
        body.Initialize(child.WorldPositionXZ);

        body.SetPosition(new Vector2d((Fixed64)20, (Fixed64)30));
        Fixed64 halfPi = Fixed64.Pi * Fixed64.Half;
        body.SetRotation(halfPi);
        context.Visualize();

        child.WorldPosition.Should().Be(new Vector3d((Fixed64)20, (Fixed64)9, (Fixed64)30));
        child.WorldRotationXZRadians.Should().Be(halfPi);

        body.IsKinematic = true;
        child.LocalPosition = new Vector3d((Fixed64)(-3), (Fixed64)6, (Fixed64)8);
        child.LocalRotationXZRadians = -halfPi;
        context.LateSimulate();

        body.Position.Should().Be(child.WorldPositionXZ);
        body.Rotation.Should().Be(-halfPi);
    }

    private static GravitasWorldContext Create2DContext()
    {
        return Physics2DTestWorld.CreateContext();
    }

    private static void Step(GravitasWorldContext context)
    {
        context.Simulate();
        context.LateSimulate();
    }

    private static SolidBody2D CreateDynamicCircle(
        GravitasWorldContext context,
        Vector2d position,
        Fixed64 rotation = default)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One
        };
        body.Initialize(position, rotation);
        return body;
    }
}
