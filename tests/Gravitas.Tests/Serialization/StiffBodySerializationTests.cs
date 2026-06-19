using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Serialization;

public sealed class StiffBodySerializationTests
{
    public static TheoryData<GravitasSerializationTransport> Transports => GravitasSerializationTransportCases.All();

    [Theory]
    [MemberData(nameof(Transports))]
    public void Populate_ShouldRestoreAuthoritativeBodyAndColliderStateWithoutReplacingHostTransform(
        GravitasSerializationTransport transport)
    {
        using PhysicsScenarioBuilder sourceScenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> source = sourceScenario.CreateSphere(
            new Vector3d((Fixed64)3, Fixed64.Half, (Fixed64)(-2)),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)35, Fixed64.Zero),
            mass: (Fixed64)4,
            immovable: true,
            isKinematic: true);
        source.Body.GroundProbeMode = GroundProbeMode.SweptSphere;
        source.Body.GroundProbeRadius = Fixed64.FromFraction(1, 3);
        source.Body.SleepEnabled = false;
        source.Body.SleepFrameThreshold = 9;
        source.Body.SleepLinearSpeedThreshold = Fixed64.FromFraction(1, 64);
        source.Body.SleepAngularSpeedThreshold = Fixed64.FromFraction(1, 32);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Body.LocalCenterOfMassOffset = new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 8), -Fixed64.FromFraction(1, 2));
        source.Body.RestitutionCoefficient = Fixed64.FromFraction(3, 4);
        source.Body.FrictionCoefficient = Fixed64.FromFraction(5, 4);
        source.Collider.Radius = Fixed64.FromFraction(3, 2);
        source.Collider.LocalOffset = new Vector3d(Fixed64.Half, Fixed64.FromFraction(1, 4), -Fixed64.Half);
        source.Collider.Layer = new PhysicsLayer(4);
        PhysicsScenarioBuilder.SetTrigger(source.Collider);
        source.Collider.Simulate();

        object payload = GravitasSerializationHarness.Serialize(source.Body, transport);

        using PhysicsScenarioBuilder targetScenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> target = targetScenario.CreateSphere(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            mass: Fixed64.One);
        FixedTransform targetTransform = target.Body.Agent.Transform;

        GravitasSerializationHarness.Populate(target.Body, payload, transport);

        target.Body.Active.Should().BeTrue();
        target.Body.Position3d.Should().Be(source.Body.Position3d);
        target.Body.Rotation.Should().Be(source.Body.Rotation);
        target.Body.Mass.Should().Be(source.Body.Mass);
        target.Body.Immovable.Should().BeTrue();
        target.Body.IsKinematic.Should().BeTrue();
        target.Body.GroundProbeMode.Should().Be(GroundProbeMode.SweptSphere);
        target.Body.GroundProbeRadius.Should().Be(Fixed64.FromFraction(1, 3));
        target.Body.SleepEnabled.Should().BeFalse();
        target.Body.SleepFrameThreshold.Should().Be(9);
        target.Body.SleepLinearSpeedThreshold.Should().Be(Fixed64.FromFraction(1, 64));
        target.Body.SleepAngularSpeedThreshold.Should().Be(Fixed64.FromFraction(1, 32));
        target.Body.ContinuousCollisionMode.Should().Be(ContinuousCollisionMode.Continuous);
        target.Body.LocalCenterOfMassOffset.Should().Be(source.Body.LocalCenterOfMassOffset);
        target.Body.WorldCenterOfMass.Should().Be(source.Body.WorldCenterOfMass);
        target.Body.RestitutionCoefficient.Should().Be(source.Body.RestitutionCoefficient);
        target.Body.FrictionCoefficient.Should().Be(source.Body.FrictionCoefficient);
        target.Body.PositionTransform.Should().BeSameAs(targetTransform);
        target.Body.RotationTransform.Should().BeSameAs(targetTransform);
        targetTransform.Position.Should().Be(source.Body.Position3d);
        targetTransform.Rotation.Should().Be(source.Body.Rotation);
        target.Collider.Radius.Should().Be(source.Collider.Radius);
        target.Collider.LocalOffset.Should().Be(source.Collider.LocalOffset);
        target.Collider.Layer.Should().Be(source.Collider.Layer);
        target.Collider.IsTrigger.Should().BeTrue();
        target.Collider.Bounds.Center.Should().Be(source.Collider.Bounds.Center);
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void PopulateSnapshot_WithQueuedTorque_ShouldReplaySameNextFrame(GravitasSerializationTransport transport)
    {
        using PhysicsScenarioBuilder uninterruptedScenario = CreateReplayScenario();
        ScenarioBody<LSCuboidCollider> uninterrupted = uninterruptedScenario.CreateCuboid(Vector3d.Zero);
        uninterrupted.Body.AddTorque(new Vector3d(Fixed64.Zero, (Fixed64)4, Fixed64.Zero));

        object payload = GravitasSerializationHarness.Serialize(uninterrupted.Body, transport);

        using PhysicsScenarioBuilder restoredScenario = CreateReplayScenario();
        ScenarioBody<LSCuboidCollider> restored = restoredScenario.CreateCuboid(Vector3d.Zero);
        GravitasSerializationHarness.Populate(restored.Body, payload, transport);

        uninterruptedScenario.Context.LateSimulate();
        restoredScenario.Context.LateSimulate();

        restored.Body.AngularVelocity.Should().Be(uninterrupted.Body.AngularVelocity);
        restored.Body.Rotation.Should().Be(uninterrupted.Body.Rotation);
    }

    [Fact]
    public void JsonSnapshot_ShouldExcludeHostBindingsAndPresentationOnlyVisualState()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.CanSetVisualPosition = true;
        body.Body.CanSetVisualRotation = true;
        body.Body.SetVisualPosition(new Vector3d((Fixed64)9, (Fixed64)8, (Fixed64)7));
        body.Body.SetVisualRotation(FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)45, Fixed64.Zero));

        string json = (string)GravitasSerializationHarness.Serialize(body.Body, GravitasSerializationTransport.Json);

        json.Should().NotContain("PositionTransform");
        json.Should().NotContain("RotationTransform");
        json.Should().NotContain("VisualPosition");
        json.Should().NotContain("LastVisualPosition");
        json.Should().NotContain("CanSetVisualPosition");
        json.Should().NotContain("VisualRotation");
        json.Should().NotContain("LastVisualRotation");
        json.Should().NotContain("CanSetVisualRotation");
        json.Should().NotContain("DefaultRotationSpeed");
        json.Should().NotContain("InteractionRotationSpeed");
        json.Should().NotContain("RotationSpeed");
        json.Should().NotContain("RotationInterpoleSpeed");
        json.Should().NotContain("SettingVisualsCounter");
        json.Should().NotContain("PositionChangedBuffer");
        json.Should().NotContain("RotationChangedBuffer");
    }

    private static PhysicsScenarioBuilder CreateReplayScenario()
    {
        PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.SetFrameRate(8);
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        return scenario;
    }
}
