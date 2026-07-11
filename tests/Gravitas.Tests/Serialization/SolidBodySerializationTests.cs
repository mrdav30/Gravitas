using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Spatial;
using System;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Serialization;

public sealed class SolidBodySerializationTests
{
    public static TheoryData<GravitasSerializationTransport> Transports => GravitasSerializationTransportCases.All();

    [Fact]
    public void RecordData_BeforeHostBinding_ShouldDeferShapeRebuildUntilInitialization()
    {
        var collider = new LSSphereCollider();
        var chronicler = new InvalidRecordPayloadChronicler(new Dictionary<string, object>
        {
            ["Radius"] = (Fixed64)2
        });

        collider.RecordData(chronicler);

        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        collider.InitializeWithNoBody(new TestMatterAgent(scenario.Context));

        collider.Radius.Should().Be((Fixed64)2);
        collider.BoundsMin.Should().Be(Vector3d.One * (Fixed64)(-2));
        collider.BoundsMax.Should().Be(Vector3d.One * (Fixed64)2);
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void PopulateActiveColliderIntoDeactivatedShell_ShouldNotPartitionUnregisteredIdentity(
        GravitasSerializationTransport transport)
    {
        using PhysicsScenarioBuilder sourceScenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider source = sourceScenario.CreateStaticSphere(Vector3d.Zero);
        object activePayload = GravitasSerializationHarness.Serialize(source, transport);

        using PhysicsScenarioBuilder targetScenario = PhysicsScenarioBuilder.Create();
        targetScenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        LSSphereCollider target = targetScenario.CreateStaticSphere(Vector3d.Zero);
        targetScenario.Context.MixedCollisions.Refresh3DColliderPartition(target);
        target.Deactivate();
        target.Id.Should().Be(-1);

        GravitasSerializationHarness.Populate(target, activePayload, transport);

        target.IsActive.Should().BeTrue();
        target.Id.Should().Be(-1);
        target.IsPartitioned.Should().BeFalse();
        target.IsMixedPartitioned.Should().BeFalse();
        (target.PartitionCoordinates?.Count ?? 0).Should().Be(0);
        (target.MixedPartitionCoordinates?.Count ?? 0).Should().Be(0);
        targetScenario.Context.Physics.ColliderCount.Should().Be(0);
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void PopulateInactiveBodyIntoRegisteredShell_ShouldReleaseRuntimeRegistration(
        GravitasSerializationTransport transport)
    {
        using PhysicsScenarioBuilder sourceScenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> source = sourceScenario.CreateSphere(Vector3d.Zero);
        object activePayload = GravitasSerializationHarness.Serialize(source.Body, transport);
        source.Body.Deactivate();
        object inactivePayload = GravitasSerializationHarness.Serialize(source.Body, transport);

        using PhysicsScenarioBuilder targetScenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> target = targetScenario.CreateSphere(Vector3d.Zero);
        targetScenario.Context.Physics.BodyCount.Should().Be(1);
        targetScenario.Context.Physics.ColliderCount.Should().Be(1);

        GravitasSerializationHarness.Populate(target.Body, inactivePayload, transport);

        target.Body.Active.Should().BeFalse();
        target.Body.DynamicId.Should().Be(-1);
        target.Collider.Id.Should().Be(-1);
        targetScenario.Context.Physics.BodyCount.Should().Be(0);
        targetScenario.Context.Physics.ColliderCount.Should().Be(0);
        target.Collider.IsPartitioned.Should().BeFalse();
        target.Collider.IsMixedPartitioned.Should().BeFalse();

        target.Body.Deactivate();

        targetScenario.Context.Physics.BodyCount.Should().Be(0);
        targetScenario.Context.Physics.ColliderCount.Should().Be(0);

        GravitasSerializationHarness.Populate(target.Body, activePayload, transport);

        target.Body.Active.Should().BeFalse();
        target.Body.DynamicId.Should().Be(-1);
        target.Collider.Id.Should().Be(-1);
        targetScenario.Context.Physics.BodyCount.Should().Be(0);
        targetScenario.Context.Physics.ColliderCount.Should().Be(0);

        target.Body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);

        target.Body.Active.Should().BeTrue();
        target.Body.DynamicId.Should().Be(0);
        target.Collider.Id.Should().Be(0);
        targetScenario.Context.Physics.BodyCount.Should().Be(1);
        targetScenario.Context.Physics.ColliderCount.Should().Be(1);
    }

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
        source.Body.FreezeAxes = BodyFreezeAxes3D.Position | BodyFreezeAxes3D.RotationY;
        source.Body.SleepEnabled = false;
        source.Body.SleepFrameThreshold = 9;
        source.Body.SleepLinearSpeedThreshold = Fixed64.FromFraction(1, 64);
        source.Body.SleepAngularSpeedThreshold = Fixed64.FromFraction(1, 32);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Body.LocalCenterOfMassOffset = new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 8), -Fixed64.FromFraction(1, 2));
        source.Collider.Material = PhysicsMaterialTestHelper.WithFrictionAndRestitution(
            Fixed64.FromFraction(5, 4),
            Fixed64.FromFraction(3, 4));
        source.Body.GravityScale = Fixed64.FromFraction(3, 8);
        source.Collider.Radius = Fixed64.FromFraction(3, 2);
        source.Collider.LocalOffset = new Vector3d(Fixed64.Half, Fixed64.FromFraction(1, 4), -Fixed64.Half);
        source.Collider.Layer = new PhysicsLayer(4);
        source.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayers(new PhysicsLayer(2), new PhysicsLayer(7));
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
        target.Body.FreezeAxes.Should().Be(source.Body.FreezeAxes);
        target.Body.IsPositionFullyFrozen.Should().BeTrue();
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
        target.Collider.Material.Should().Be(source.Collider.Material);
        target.Body.GravityScale.Should().Be(source.Body.GravityScale);
        target.Body.PositionTransform.Should().BeSameAs(targetTransform);
        target.Body.RotationTransform.Should().BeSameAs(targetTransform);
        targetTransform.Position.Should().Be(source.Body.Position3d);
        targetTransform.Rotation.Should().Be(source.Body.Rotation);
        target.Collider.Radius.Should().Be(source.Collider.Radius);
        target.Collider.LocalOffset.Should().Be(source.Collider.LocalOffset);
        target.Collider.Layer.Should().Be(source.Collider.Layer);
        target.Collider.IgnoredCollisionLayers.Should().Be(source.Collider.IgnoredCollisionLayers);
        target.Collider.IsTrigger.Should().BeFalse();
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

    [Theory]
    [MemberData(nameof(Transports))]
    public void Populate_ShouldRestoreManualGroundingAuthority(GravitasSerializationTransport transport)
    {
        using PhysicsScenarioBuilder sourceScenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> source = sourceScenario.CreateSphere(Vector3d.Zero);
        Vector3d hitPoint = new(Fixed64.Zero, Fixed64.FromFraction(5, 4), Fixed64.Zero);
        source.Body.SetManualGrounding(hitPoint, Vector3d.Up);

        object payload = GravitasSerializationHarness.Serialize(source.Body, transport);

        using PhysicsScenarioBuilder targetScenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> target = targetScenario.CreateSphere(Vector3d.Zero);

        GravitasSerializationHarness.Populate(target.Body, payload, transport);
        target.Body.CheckGround();

        target.Body.GroundingMode.Should().Be(GroundingMode.Manual);
        target.Body.IsGrounded.Should().BeTrue();
        target.Body.HitPoint.Should().Be(hitPoint);
        target.Body.GroundNormal.Should().Be(Vector3d.Up);
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void Populate_ShouldRestoreConeShapeStateAndMassProperties(GravitasSerializationTransport transport)
    {
        using PhysicsScenarioBuilder sourceScenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSConeCollider> source = sourceScenario.CreateBody(
            new LSConeCollider
            {
                Radius = Fixed64.FromFraction(3, 4),
                Size = new Vector3d(Fixed64.FromFraction(3, 2), (Fixed64)3, Fixed64.FromFraction(3, 2))
            },
            new Vector3d((Fixed64)2, Fixed64.One, Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)45),
            mass: (Fixed64)5);
        source.Collider.LocalOffset = new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero);
        source.Collider.Simulate();

        object payload = GravitasSerializationHarness.Serialize(source.Body, transport);

        using PhysicsScenarioBuilder targetScenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSConeCollider> target = targetScenario.CreateBody(
            new LSConeCollider(),
            Vector3d.Zero,
            FixedQuaternion.Identity,
            mass: Fixed64.One);

        GravitasSerializationHarness.Populate(target.Body, payload, transport);

        target.Collider.Radius.Should().Be(source.Collider.Radius);
        target.Collider.Size.Should().Be(source.Collider.Size);
        target.Collider.LocalOffset.Should().Be(source.Collider.LocalOffset);
        target.Collider.BaseCenter.Should().Be(source.Collider.BaseCenter);
        target.Collider.Apex.Should().Be(source.Collider.Apex);
        target.Body.LocalCenterOfMassOffset.Should().Be(source.Body.LocalCenterOfMassOffset);
        target.Body.InverseInertiaTensor.Should().Be(source.Body.InverseInertiaTensor);
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void Populate_ShouldRestoreWasGroundedTransitionState(GravitasSerializationTransport transport)
    {
        using PhysicsScenarioBuilder sourceScenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> source = sourceScenario.CreateSphere(Vector3d.Zero);
        source.Body.SetManualGrounding(new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero), Vector3d.Up);
        source.Body.ClearManualGrounding();

        object payload = GravitasSerializationHarness.Serialize(source.Body, transport);

        using PhysicsScenarioBuilder targetScenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> target = targetScenario.CreateSphere(Vector3d.Zero);

        GravitasSerializationHarness.Populate(target.Body, payload, transport);

        target.Body.IsGrounded.Should().BeFalse();
        target.Body.WasGrounded.Should().BeTrue();
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void PopulateInactiveCollider_ShouldClearExistingPartitionMembership(GravitasSerializationTransport transport)
    {
        using PhysicsScenarioBuilder sourceScenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider source = sourceScenario.CreateStaticSphere(Vector3d.Zero);
        object activePayload = GravitasSerializationHarness.Serialize(source, transport);
        source.Deactivate();
        source.IsActive.Should().BeFalse();
        object inactivePayload = GravitasSerializationHarness.Serialize(source, transport);

        using PhysicsScenarioBuilder targetScenario = PhysicsScenarioBuilder.Create();
        targetScenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        LSSphereCollider target = targetScenario.CreateStaticSphere(Vector3d.Zero);
        targetScenario.Context.MixedCollisions.Refresh3DColliderPartition(target);
        target.IsPartitioned.Should().BeTrue();
        target.IsMixedPartitioned.Should().BeTrue();
        WorldVoxelIndex mixedCoordinate = target.MixedPartitionCoordinates![0];
        SerializationPartitionAssertions.Mixed3DPartitionContains(
            targetScenario.Context,
            mixedCoordinate,
            target.Id).Should().BeTrue();

        GravitasSerializationHarness.Populate(target, inactivePayload, transport);

        target.IsActive.Should().BeFalse();
        target.IsPartitioned.Should().BeFalse();
        target.IsMixedPartitioned.Should().BeFalse();
        target.PartitionCoordinates.Should().BeEmpty();
        (target.MixedPartitionCoordinates?.Count ?? 0).Should().Be(0);
        SerializationPartitionAssertions.Mixed3DPartitionContains(
            targetScenario.Context,
            mixedCoordinate,
            target.Id).Should().BeFalse();

        GravitasSerializationHarness.Populate(target, inactivePayload, transport);

        target.IsActive.Should().BeFalse();
        target.IsPartitioned.Should().BeFalse();
        target.IsMixedPartitioned.Should().BeFalse();
        target.PartitionCoordinates.Should().BeEmpty();
        (target.MixedPartitionCoordinates?.Count ?? 0).Should().Be(0);

        GravitasSerializationHarness.Populate(target, activePayload, transport);

        target.IsActive.Should().BeTrue();
        target.IsPartitioned.Should().BeTrue();
        target.IsMixedPartitioned.Should().BeTrue();
        SerializationPartitionAssertions.Primary3DPartitionsShouldContain(
            targetScenario.Context,
            target.PartitionCoordinates!,
            target.Id);
        SerializationPartitionAssertions.Mixed3DPartitionsShouldContain(
            targetScenario.Context,
            target.MixedPartitionCoordinates!,
            target.Id);
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void PopulateInactiveCollider_ShouldClearPrimaryPartitionMembershipInPureRuntime(GravitasSerializationTransport transport)
    {
        using PhysicsScenarioBuilder sourceScenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider source = sourceScenario.CreateStaticSphere(Vector3d.Zero);
        object activePayload = GravitasSerializationHarness.Serialize(source, transport);
        source.Deactivate();
        object inactivePayload = GravitasSerializationHarness.Serialize(source, transport);

        using PhysicsScenarioBuilder targetScenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider target = targetScenario.CreateStaticSphere(Vector3d.Zero);
        target.IsPartitioned.Should().BeTrue();
        target.IsMixedPartitioned.Should().BeFalse();
        WorldVoxelIndex coordinate = target.PartitionCoordinates![0];
        SerializationPartitionAssertions.Primary3DPartitionContains(
            targetScenario.Context,
            coordinate,
            target.Id).Should().BeTrue();

        GravitasSerializationHarness.Populate(target, inactivePayload, transport);

        target.IsActive.Should().BeFalse();
        target.IsPartitioned.Should().BeFalse();
        target.IsMixedPartitioned.Should().BeFalse();
        target.PartitionCoordinates.Should().BeEmpty();
        (target.MixedPartitionCoordinates?.Count ?? 0).Should().Be(0);
        SerializationPartitionAssertions.Primary3DPartitionContains(
            targetScenario.Context,
            coordinate,
            target.Id).Should().BeFalse();

        GravitasSerializationHarness.Populate(target, inactivePayload, transport);

        target.IsActive.Should().BeFalse();
        target.IsPartitioned.Should().BeFalse();
        target.IsMixedPartitioned.Should().BeFalse();
        target.PartitionCoordinates.Should().BeEmpty();
        (target.MixedPartitionCoordinates?.Count ?? 0).Should().Be(0);

        GravitasSerializationHarness.Populate(target, activePayload, transport);

        target.IsActive.Should().BeTrue();
        target.IsPartitioned.Should().BeTrue();
        target.IsMixedPartitioned.Should().BeFalse();
        SerializationPartitionAssertions.Primary3DPartitionsShouldContain(
            targetScenario.Context,
            target.PartitionCoordinates!,
            target.Id);
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void PopulateActiveColliderShape_ShouldRefreshPrimaryAndMixedPartitionMembership(
        GravitasSerializationTransport transport)
    {
        using PhysicsScenarioBuilder sourceScenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider source = sourceScenario.CreateStaticSphere(Vector3d.Zero);
        source.LocalOffset = new Vector3d((Fixed64)6, Fixed64.Zero, Fixed64.Zero);
        source.Simulate();
        object payload = GravitasSerializationHarness.Serialize(source, transport);

        using PhysicsScenarioBuilder targetScenario = PhysicsScenarioBuilder.Create();
        targetScenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        LSSphereCollider target = targetScenario.CreateStaticSphere(Vector3d.Zero);
        targetScenario.Context.MixedCollisions.Refresh3DColliderPartition(target);
        WorldVoxelIndex[] primaryCoordinatesBeforeLoad =
            SerializationPartitionAssertions.CopyCoordinates(target.PartitionCoordinates!);
        WorldVoxelIndex[] mixedCoordinatesBeforeLoad =
            SerializationPartitionAssertions.CopyCoordinates(target.MixedPartitionCoordinates!);

        GravitasSerializationHarness.Populate(target, payload, transport);

        target.IsActive.Should().BeTrue();
        target.Radius.Should().Be(source.Radius);
        target.LocalOffset.Should().Be(source.LocalOffset);
        target.Bounds.Should().Be(source.Bounds);
        target.IsPartitioned.Should().BeTrue();
        target.IsMixedPartitioned.Should().BeTrue();
        SerializationPartitionAssertions.Primary3DPartitionsShouldContain(
            targetScenario.Context,
            target.PartitionCoordinates!,
            target.Id);
        SerializationPartitionAssertions.Mixed3DPartitionsShouldContain(
            targetScenario.Context,
            target.MixedPartitionCoordinates!,
            target.Id);
        SerializationPartitionAssertions.StalePrimary3DPartitionsShouldBeCleared(
            targetScenario.Context,
            primaryCoordinatesBeforeLoad,
            target.PartitionCoordinates!,
            target.Id).Should().BeTrue("the loaded local offset should move the collider out of an original primary partition");
        SerializationPartitionAssertions.StaleMixed3DPartitionsShouldBeCleared(
            targetScenario.Context,
            mixedCoordinatesBeforeLoad,
            target.MixedPartitionCoordinates!,
            target.Id).Should().BeTrue("the loaded local offset should move the collider out of an original mixed partition");
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void PopulateActiveColliderShape_ShouldRefreshPrimaryPartitionMembershipInPureRuntime(
        GravitasSerializationTransport transport)
    {
        using PhysicsScenarioBuilder sourceScenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider source = sourceScenario.CreateStaticSphere(Vector3d.Zero);
        source.LocalOffset = new Vector3d((Fixed64)6, Fixed64.Zero, Fixed64.Zero);
        source.Simulate();
        object payload = GravitasSerializationHarness.Serialize(source, transport);

        using PhysicsScenarioBuilder targetScenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider target = targetScenario.CreateStaticSphere(Vector3d.Zero);
        WorldVoxelIndex[] primaryCoordinatesBeforeLoad =
            SerializationPartitionAssertions.CopyCoordinates(target.PartitionCoordinates!);

        GravitasSerializationHarness.Populate(target, payload, transport);

        target.IsActive.Should().BeTrue();
        target.LocalOffset.Should().Be(source.LocalOffset);
        target.Bounds.Should().Be(source.Bounds);
        target.IsPartitioned.Should().BeTrue();
        target.IsMixedPartitioned.Should().BeFalse();
        (target.MixedPartitionCoordinates?.Count ?? 0).Should().Be(0);
        SerializationPartitionAssertions.Primary3DPartitionsShouldContain(
            targetScenario.Context,
            target.PartitionCoordinates!,
            target.Id);
        SerializationPartitionAssertions.StalePrimary3DPartitionsShouldBeCleared(
            targetScenario.Context,
            primaryCoordinatesBeforeLoad,
            target.PartitionCoordinates!,
            target.Id).Should().BeTrue("the loaded local offset should move the collider out of an original primary partition");
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void PopulateTriggerColliderIntoBodyCollider_ShouldRejectInvalidLoadedState(GravitasSerializationTransport transport)
    {
        using PhysicsScenarioBuilder sourceScenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider source = sourceScenario.CreateStaticSphere(Vector3d.Zero);
        source.IsTrigger = true;
        object payload = GravitasSerializationHarness.Serialize(source, transport);

        using PhysicsScenarioBuilder targetScenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> target = targetScenario.CreateSphere(Vector3d.Zero);
        Action loadTriggerOntoBodyCollider = () => GravitasSerializationHarness.Populate(target.Collider, payload, transport);

        loadTriggerOntoBodyCollider.Should().Throw<ArgumentException>().WithParameterName(nameof(LSCollider.IsTrigger));
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
