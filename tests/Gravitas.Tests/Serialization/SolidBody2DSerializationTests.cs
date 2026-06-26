using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Serialization;

public sealed class SolidBody2DSerializationTests
{
    public static TheoryData<GravitasSerializationTransport> Transports => GravitasSerializationTransportCases.All();

    [Theory]
    [MemberData(nameof(Transports))]
    public void Populate_ShouldRestoreAuthoritativeBodyAndColliderState(GravitasSerializationTransport transport)
    {
        using GravitasWorldContext sourceContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        var sourceCollider = new LSCircleCollider2D((Fixed64)2)
        {
            IsTrigger = true,
            Layer = new PhysicsLayer(3),
            LocalOffset = new Vector2d(Fixed64.Half, Fixed64.FromFraction(1, 4)),
            MixedHalfThicknessOverride = Fixed64.FromFraction(3, 2)
        };
        var sourceAgent = new TestMatterAgent(sourceContext);
        var source = new SolidBody2D(sourceAgent, sourceCollider)
        {
            Mass = (Fixed64)3,
            Immovable = true,
            IsKinematic = true,
            RestitutionCoefficient = Fixed64.FromFraction(3, 4),
            FrictionCoefficient = Fixed64.FromFraction(5, 4),
            Gravity = new Vector2d(Fixed64.Zero, (Fixed64)(-2)),
            SleepEnabled = false,
            SleepFrameThreshold = 11,
            SleepLinearSpeedThreshold = Fixed64.FromFraction(1, 128),
            SleepAngularSpeedThreshold = Fixed64.FromFraction(1, 64),
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous,
            PreventAngularForces = true
        };
        source.Initialize(new Vector2d((Fixed64)5, (Fixed64)(-2)), Fixed64.FromFraction(1, 8));
        source.LocalCenterOfMassOffset = new Vector2d(Fixed64.FromFraction(1, 3), -Fixed64.FromFraction(1, 4));

        object payload = GravitasSerializationHarness.Serialize(source, transport);

        using GravitasWorldContext targetContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        var targetCollider = new LSCircleCollider2D(Fixed64.Half);
        var target = new SolidBody2D(new TestMatterAgent(targetContext), targetCollider)
        {
            Mass = Fixed64.One
        };
        target.Initialize(Vector2d.Zero);

        GravitasSerializationHarness.Populate(target, payload, transport);

        target.Active.Should().BeTrue();
        target.Position.Should().Be(source.Position);
        target.Rotation.Should().Be(source.Rotation);
        target.Mass.Should().Be(source.Mass);
        target.Immovable.Should().BeTrue();
        target.IsKinematic.Should().BeTrue();
        target.RestitutionCoefficient.Should().Be(source.RestitutionCoefficient);
        target.FrictionCoefficient.Should().Be(source.FrictionCoefficient);
        target.Gravity.Should().Be(source.Gravity);
        target.SleepEnabled.Should().BeFalse();
        target.SleepFrameThreshold.Should().Be(source.SleepFrameThreshold);
        target.SleepLinearSpeedThreshold.Should().Be(source.SleepLinearSpeedThreshold);
        target.SleepAngularSpeedThreshold.Should().Be(source.SleepAngularSpeedThreshold);
        target.ContinuousCollisionMode.Should().Be(source.ContinuousCollisionMode);
        target.PreventAngularForces.Should().BeTrue();
        target.LocalCenterOfMassOffset.Should().Be(source.LocalCenterOfMassOffset);
        target.WorldCenterOfMass.Should().Be(source.WorldCenterOfMass);
        targetCollider.Radius.Should().Be(sourceCollider.Radius);
        targetCollider.IsTrigger.Should().BeTrue();
        targetCollider.Layer.Should().Be(sourceCollider.Layer);
        targetCollider.LocalOffset.Should().Be(sourceCollider.LocalOffset);
        targetCollider.MixedHalfThicknessOverride.Should().Be(sourceCollider.MixedHalfThicknessOverride);
        targetCollider.Bounds.Should().Be(sourceCollider.Bounds);
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void PopulateSnapshot_WithQueuedForce_ShouldReplaySameNextFrame(GravitasSerializationTransport transport)
    {
        using GravitasWorldContext uninterruptedContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        SolidBody2D uninterrupted = CreateDynamicCircle(uninterruptedContext);
        uninterrupted.AddForce(new Vector2d((Fixed64)8, (Fixed64)4));

        object payload = GravitasSerializationHarness.Serialize(uninterrupted, transport);

        using GravitasWorldContext restoredContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        SolidBody2D restored = CreateDynamicCircle(restoredContext);
        GravitasSerializationHarness.Populate(restored, payload, transport);

        uninterruptedContext.LateSimulate();
        restoredContext.LateSimulate();

        restored.Position.Should().Be(uninterrupted.Position);
        restored.LinearVelocity.Should().Be(uninterrupted.LinearVelocity);
        restored.LinearSpeed.Should().Be(uninterrupted.LinearSpeed);
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void PopulateSnapshot_WithQueuedTorque_ShouldReplaySameNextFrame(GravitasSerializationTransport transport)
    {
        using GravitasWorldContext uninterruptedContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        SolidBody2D uninterrupted = CreateDynamicCircle(uninterruptedContext);
        uninterrupted.SleepAngularSpeedThreshold = Fixed64.FromFraction(1, 64);
        uninterrupted.AddAngularImpulse((Fixed64)3);
        uninterrupted.AddTorque((Fixed64)4);

        object payload = GravitasSerializationHarness.Serialize(uninterrupted, transport);

        using GravitasWorldContext restoredContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        SolidBody2D restored = CreateDynamicCircle(restoredContext);
        GravitasSerializationHarness.Populate(restored, payload, transport);

        restored.AngularVelocity.Should().Be(uninterrupted.AngularVelocity);
        restored.AngularSpeed.Should().Be(uninterrupted.AngularSpeed);
        restored.SleepAngularSpeedThreshold.Should().Be(uninterrupted.SleepAngularSpeedThreshold);

        uninterruptedContext.LateSimulate();
        restoredContext.LateSimulate();

        restored.AngularVelocity.Should().Be(uninterrupted.AngularVelocity);
        restored.AngularAcceleration.Should().Be(uninterrupted.AngularAcceleration);
        restored.AngularSpeed.Should().Be(uninterrupted.AngularSpeed);
        restored.Rotation.Should().Be(uninterrupted.Rotation);
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void Populate_ShouldNotWakeSleepingBodyWhenShapeStateChanges(GravitasSerializationTransport transport)
    {
        using GravitasWorldContext sourceContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        SolidBody2D source = CreateDynamicCircle(sourceContext);
        source.Sleep();

        object payload = GravitasSerializationHarness.Serialize(source, transport);

        using GravitasWorldContext targetContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        var target = new SolidBody2D(new TestMatterAgent(targetContext), new LSCircleCollider2D((Fixed64)4))
        {
            Mass = Fixed64.One
        };
        target.Initialize(Vector2d.Zero);

        GravitasSerializationHarness.Populate(target, payload, transport);

        target.IsSleeping.Should().BeTrue();
        ((LSCircleCollider2D)target.Collider).Radius.Should().Be(((LSCircleCollider2D)source.Collider).Radius);
    }

    private static SolidBody2D CreateDynamicCircle(GravitasWorldContext context)
    {
        var agent = new TestMatterAgent(context);
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = (Fixed64)2
        };
        body.Initialize(Vector2d.Zero);
        return body;
    }
}
