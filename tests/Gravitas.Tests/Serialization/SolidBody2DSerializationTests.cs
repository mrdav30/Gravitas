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
            Layer = new PhysicsLayer(3),
            LocalOffset = new Vector2d(Fixed64.Half, Fixed64.FromFraction(1, 4)),
            MixedHalfThicknessOverride = Fixed64.FromFraction(3, 2),
            IgnoredCollisionLayers = PhysicsLayerMask.FromLayers(new PhysicsLayer(2), new PhysicsLayer(7))
        };
        var sourceAgent = new TestMatterAgent(sourceContext);
        var source = new SolidBody2D(sourceAgent, sourceCollider)
        {
            Mass = (Fixed64)3,
            FreezeAxes = BodyFreezeAxes2D.All,
            IsKinematic = true,
            Gravity = new Vector2d(Fixed64.Zero, (Fixed64)(-2)),
            GravityScale = Fixed64.FromFraction(3, 8),
            SleepEnabled = false,
            SleepFrameThreshold = 11,
            SleepLinearSpeedThreshold = Fixed64.FromFraction(1, 128),
            SleepAngularSpeedThreshold = Fixed64.FromFraction(1, 64),
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous
        };
        sourceCollider.Material = PhysicsMaterialTestHelper.WithFrictionAndRestitution(
            Fixed64.FromFraction(5, 4),
            Fixed64.FromFraction(3, 4));
        source.Initialize(new Vector2d((Fixed64)5, (Fixed64)(-2)), Fixed64.FromFraction(1, 8));
        source.LocalCenterOfMassOffset = new Vector2d(Fixed64.FromFraction(1, 3), -Fixed64.FromFraction(1, 4));
        source.UseGravityDerivedGroundUpDirection = false;
        source.GroundUpDirection = Vector2d.Forward;
        source.GroundProbeMode = GroundProbeMode2D.SweptCircle;
        source.GroundProbeRadius = Fixed64.FromFraction(1, 3);
        source.GroundedDistanceRay = Fixed64.FromFraction(3, 4);
        source.GroundDownDistanceOnAir = Fixed64.FromFraction(5, 4);
        source.GroundMinNormalDot = Fixed64.FromFraction(3, 5);
        source.SetManualGrounding(
            new Vector2d((Fixed64)5, Fixed64.FromFraction(-3, 2)),
            Vector2d.Forward);

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
        target.FreezeAxes.Should().Be(source.FreezeAxes);
        target.IsPositionFullyFrozen.Should().BeTrue();
        target.IsKinematic.Should().BeTrue();
        targetCollider.Material.Should().Be(sourceCollider.Material);
        target.Gravity.Should().Be(source.Gravity);
        target.GravityScale.Should().Be(source.GravityScale);
        target.SleepEnabled.Should().BeFalse();
        target.SleepFrameThreshold.Should().Be(source.SleepFrameThreshold);
        target.SleepLinearSpeedThreshold.Should().Be(source.SleepLinearSpeedThreshold);
        target.SleepAngularSpeedThreshold.Should().Be(source.SleepAngularSpeedThreshold);
        target.ContinuousCollisionMode.Should().Be(source.ContinuousCollisionMode);
        target.GroundingMode.Should().Be(GroundingMode.Manual);
        target.GroundProbeMode.Should().Be(source.GroundProbeMode);
        target.UseGravityDerivedGroundUpDirection.Should().BeFalse();
        target.GroundUpDirection.Should().Be(Vector2d.Forward);
        target.GroundProbeRadius.Should().Be(source.GroundProbeRadius);
        target.GroundedDistanceRay.Should().Be(source.GroundedDistanceRay);
        target.GroundDownDistanceOnAir.Should().Be(source.GroundDownDistanceOnAir);
        target.GroundMinNormalDot.Should().Be(source.GroundMinNormalDot);
        target.IsGrounded.Should().BeTrue();
        target.WasGrounded.Should().BeFalse();
        target.GroundPoint.Should().Be(source.GroundPoint);
        target.GroundNormal.Should().Be(Vector2d.Forward);
        target.LastGroundedPosition.Should().Be(source.LastGroundedPosition);
        target.AngularMotionFrozen.Should().BeTrue();
        target.LocalCenterOfMassOffset.Should().Be(source.LocalCenterOfMassOffset);
        target.WorldCenterOfMass.Should().Be(source.WorldCenterOfMass);
        targetCollider.Radius.Should().Be(sourceCollider.Radius);
        targetCollider.IsTrigger.Should().BeFalse();
        targetCollider.Layer.Should().Be(sourceCollider.Layer);
        targetCollider.IgnoredCollisionLayers.Should().Be(sourceCollider.IgnoredCollisionLayers);
        targetCollider.LocalOffset.Should().Be(sourceCollider.LocalOffset);
        targetCollider.MixedHalfThicknessOverride.Should().Be(sourceCollider.MixedHalfThicknessOverride);
        targetCollider.Bounds.Should().Be(sourceCollider.Bounds);
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void Populate_ShouldRestoreAutomaticAndManualClearedGroundingState(GravitasSerializationTransport transport)
    {
        using GravitasWorldContext automaticContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        CreateStaticFloor(automaticContext);
        SolidBody2D automatic = CreateDynamicCircle(automaticContext, new Vector2d(Fixed64.Zero, Fixed64.One));
        automatic.GroundProbeMode = GroundProbeMode2D.Ray;
        automatic.CheckGround();

        object automaticPayload = GravitasSerializationHarness.Serialize(automatic, transport);

        using GravitasWorldContext automaticTargetContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        SolidBody2D automaticTarget = CreateDynamicCircle(automaticTargetContext);
        GravitasSerializationHarness.Populate(automaticTarget, automaticPayload, transport);

        automaticTarget.GroundingMode.Should().Be(GroundingMode.Automatic);
        automaticTarget.GroundProbeMode.Should().Be(GroundProbeMode2D.Ray);
        automaticTarget.IsGrounded.Should().BeTrue();
        automaticTarget.GroundPoint.Should().Be(automatic.GroundPoint);
        automaticTarget.GroundNormal.Should().Be(Vector2d.Forward);

        automatic.UseManualGrounding();
        object manualPayload = GravitasSerializationHarness.Serialize(automatic, transport);

        using GravitasWorldContext manualTargetContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        SolidBody2D manualTarget = CreateDynamicCircle(manualTargetContext);
        GravitasSerializationHarness.Populate(manualTarget, manualPayload, transport);

        manualTarget.GroundingMode.Should().Be(GroundingMode.Manual);
        manualTarget.IsGrounded.Should().BeFalse();
        manualTarget.WasGrounded.Should().BeTrue();
        manualTarget.GroundNormal.Should().Be(Vector2d.Zero);
        manualTarget.GroundPoint.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void LoadPayload_WithInvalidGroundingValues_ShouldCanonicalize2DGroundingState()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(frameRate: 8);
        SolidBody2D body = CreateDynamicCircle(context);
        body.UseGravityDerivedGroundUpDirection = false;
        body.GroundUpDirection = Vector2d.Right;
        body.GroundProbeRadius = Fixed64.Half;
        var chronicler = new InvalidRecordPayloadChronicler(new Dictionary<string, object>
        {
            ["UseGravityDerivedGroundUpDirection"] = false,
            ["GroundUpDirection"] = Vector2d.Zero,
            ["GroundProbeRadius"] = -Fixed64.One,
            ["GroundNormal"] = Vector2d.Right * (Fixed64)2
        });

        body.RecordData(chronicler);

        body.UseGravityDerivedGroundUpDirection.Should().BeFalse();
        body.GroundUpDirection.Should().Be(Vector2d.Forward);
        body.GroundProbeRadius.Should().Be(Fixed64.Zero);
        body.GroundNormal.Should().Be(Vector2d.Right);
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void Populate_ShouldRestoreCapsuleColliderShapeState(GravitasSerializationTransport transport)
    {
        using GravitasWorldContext sourceContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        var sourceCollider = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3)
        {
            LocalOffset = new Vector2d(Fixed64.One, -Fixed64.Half)
        };
        var source = new SolidBody2D(new TestMatterAgent(sourceContext), sourceCollider)
        {
            Mass = (Fixed64)2
        };
        source.Initialize(new Vector2d((Fixed64)4, Fixed64.One), FixedMath.DegToRad((Fixed64)30));

        object payload = GravitasSerializationHarness.Serialize(source, transport);

        using GravitasWorldContext targetContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        var targetCollider = new LSCapsuleCollider2D(Fixed64.One, (Fixed64)5);
        var target = new SolidBody2D(new TestMatterAgent(targetContext), targetCollider)
        {
            Mass = Fixed64.One
        };
        target.Initialize(Vector2d.Zero);

        GravitasSerializationHarness.Populate(target, payload, transport);

        target.Position.Should().Be(source.Position);
        target.Rotation.Should().Be(source.Rotation);
        targetCollider.Radius.Should().Be(sourceCollider.Radius);
        targetCollider.Height.Should().Be(sourceCollider.Height);
        targetCollider.LocalOffset.Should().Be(sourceCollider.LocalOffset);
        targetCollider.Bounds.Should().Be(sourceCollider.Bounds);
        targetCollider.SegmentStart.Should().Be(sourceCollider.SegmentStart);
        targetCollider.SegmentEnd.Should().Be(sourceCollider.SegmentEnd);
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void Populate_ShouldRestorePolygonColliderShapeStateAndReplayNextFrame(GravitasSerializationTransport transport)
    {
        Vector2d[] sourceVertices =
        {
            new(-Fixed64.One, -Fixed64.Half),
            new(Fixed64.One, -Fixed64.Half),
            new(Fixed64.One, Fixed64.Half),
            new(-Fixed64.One, Fixed64.Half)
        };

        using GravitasWorldContext sourceContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        var sourceCollider = new LSPolygonCollider2D(sourceVertices)
        {
            LocalOffset = new Vector2d(Fixed64.Half, -Fixed64.FromFraction(1, 4))
        };
        var source = new SolidBody2D(new TestMatterAgent(sourceContext), sourceCollider)
        {
            Mass = (Fixed64)3
        };
        source.Initialize(new Vector2d((Fixed64)2, Fixed64.One), FixedMath.DegToRad((Fixed64)15));
        source.AddForce(new Vector2d((Fixed64)6, (Fixed64)(-2)));

        object payload = GravitasSerializationHarness.Serialize(source, transport);

        using GravitasWorldContext targetContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        var targetCollider = new LSPolygonCollider2D(
            new Vector2d(Fixed64.Zero, Fixed64.Zero),
            new Vector2d(Fixed64.One, Fixed64.Zero),
            new Vector2d(Fixed64.Zero, Fixed64.One));
        var target = new SolidBody2D(new TestMatterAgent(targetContext), targetCollider)
        {
            Mass = Fixed64.One
        };
        target.Initialize(Vector2d.Zero);

        GravitasSerializationHarness.Populate(target, payload, transport);

        target.Position.Should().Be(source.Position);
        target.Rotation.Should().Be(source.Rotation);
        targetCollider.LocalOffset.Should().Be(sourceCollider.LocalOffset);
        targetCollider.Count.Should().Be(sourceCollider.Count);
        for (int i = 0; i < sourceCollider.Count; i++)
            targetCollider.GetWorldVertex(i).Should().Be(sourceCollider.GetWorldVertex(i));

        targetCollider.Bounds.Should().Be(sourceCollider.Bounds);
        target.LocalCenterOfMassOffset.Should().Be(source.LocalCenterOfMassOffset);
        target.MomentOfInertia.Should().Be(source.MomentOfInertia);
        target.InverseMomentOfInertia.Should().Be(source.InverseMomentOfInertia);

        sourceContext.LateSimulate();
        targetContext.LateSimulate();

        target.Position.Should().Be(source.Position);
        target.LinearVelocity.Should().Be(source.LinearVelocity);
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

    [Theory]
    [MemberData(nameof(Transports))]
    public void PopulateInactiveBodyIntoRegisteredShell_ShouldReleaseRuntimeRegistration(
        GravitasSerializationTransport transport)
    {
        using GravitasWorldContext sourceContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        SolidBody2D source = CreateDynamicCircle(sourceContext);
        object activePayload = GravitasSerializationHarness.Serialize(source, transport);
        source.Deactivate();
        object payload = GravitasSerializationHarness.Serialize(source, transport);

        using GravitasWorldContext targetContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        SolidBody2D target = CreateDynamicCircle(targetContext);
        targetContext.Physics2D.BodyCount.Should().Be(1);
        targetContext.Physics2D.ColliderCount.Should().Be(1);

        GravitasSerializationHarness.Populate(target, payload, transport);

        target.Active.Should().BeFalse();
        target.DynamicId.Should().Be(-1);
        target.Collider.Id.Should().Be(-1);
        targetContext.Physics2D.BodyCount.Should().Be(0);
        targetContext.Physics2D.ColliderCount.Should().Be(0);
        target.Collider.IsPartitioned.Should().BeFalse();
        target.Collider.IsMixedPartitioned.Should().BeFalse();

        target.Deactivate();

        targetContext.Physics2D.BodyCount.Should().Be(0);
        targetContext.Physics2D.ColliderCount.Should().Be(0);

        GravitasSerializationHarness.Populate(target, activePayload, transport);

        target.Active.Should().BeFalse();
        target.DynamicId.Should().Be(-1);
        target.Collider.Id.Should().Be(-1);
        targetContext.Physics2D.BodyCount.Should().Be(0);
        targetContext.Physics2D.ColliderCount.Should().Be(0);

        target.Initialize(Vector2d.Zero);

        target.Active.Should().BeTrue();
        target.DynamicId.Should().Be(0);
        target.Collider.Id.Should().Be(0);
        targetContext.Physics2D.BodyCount.Should().Be(1);
        targetContext.Physics2D.ColliderCount.Should().Be(1);
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void PopulateInactiveCollider_ShouldClearExistingPartitionMembership(GravitasSerializationTransport transport)
    {
        using GravitasWorldContext sourceContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        LSCircleCollider2D source = CreateStaticCircle(sourceContext, Vector2d.Zero);
        object activePayload = GravitasSerializationHarness.Serialize(source, transport);
        source.Deactivate();
        source.IsActive.Should().BeFalse();
        object payload = GravitasSerializationHarness.Serialize(source, transport);

        using GravitasWorldContext targetContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        targetContext.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        LSCircleCollider2D target = CreateStaticCircle(targetContext, Vector2d.Zero);
        targetContext.MixedCollisions.Refresh2DColliderPartition(target);
        target.IsPartitioned.Should().BeTrue();
        target.IsMixedPartitioned.Should().BeTrue();
        WorldVoxelIndex mixedCoordinate = target.MixedPartitionCoordinates![0];
        SerializationPartitionAssertions.Mixed2DPartitionContains(
            targetContext,
            mixedCoordinate,
            target.Id).Should().BeTrue();

        GravitasSerializationHarness.Populate(target, payload, transport);

        target.IsActive.Should().BeFalse();
        target.IsPartitioned.Should().BeFalse();
        target.IsMixedPartitioned.Should().BeFalse();
        target.PartitionCoordinates.Should().BeEmpty();
        (target.MixedPartitionCoordinates?.Count ?? 0).Should().Be(0);
        SerializationPartitionAssertions.Mixed2DPartitionContains(
            targetContext,
            mixedCoordinate,
            target.Id).Should().BeFalse();

        GravitasSerializationHarness.Populate(target, payload, transport);

        target.IsActive.Should().BeFalse();
        target.IsPartitioned.Should().BeFalse();
        target.IsMixedPartitioned.Should().BeFalse();
        target.PartitionCoordinates.Should().BeEmpty();
        (target.MixedPartitionCoordinates?.Count ?? 0).Should().Be(0);

        GravitasSerializationHarness.Populate(target, activePayload, transport);

        target.IsActive.Should().BeTrue();
        target.IsPartitioned.Should().BeTrue();
        target.IsMixedPartitioned.Should().BeTrue();
        SerializationPartitionAssertions.Primary2DPartitionsShouldContain(
            targetContext,
            target.PartitionCoordinates!,
            target.Id);
        SerializationPartitionAssertions.Mixed2DPartitionsShouldContain(
            targetContext,
            target.MixedPartitionCoordinates!,
            target.Id);
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void PopulateActiveColliderIntoDeactivatedShell_ShouldNotRegisterInvalidId(
        GravitasSerializationTransport transport)
    {
        using GravitasWorldContext sourceContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        LSCircleCollider2D source = CreateStaticCircle(sourceContext, Vector2d.Zero);
        object activePayload = GravitasSerializationHarness.Serialize(source, transport);

        using GravitasWorldContext targetContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        targetContext.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        LSCircleCollider2D target = CreateStaticCircle(targetContext, Vector2d.Zero);
        targetContext.MixedCollisions.Refresh2DColliderPartition(target);
        target.Deactivate();

        GravitasSerializationHarness.Populate(target, activePayload, transport);

        target.IsActive.Should().BeTrue();
        target.Id.Should().Be(-1);
        targetContext.Physics2D.ColliderCount.Should().Be(0);
        target.IsPartitioned.Should().BeFalse();
        target.IsMixedPartitioned.Should().BeFalse();
        (target.PartitionCoordinates?.Count ?? 0).Should().Be(0);
        (target.MixedPartitionCoordinates?.Count ?? 0).Should().Be(0);
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void PopulateInactiveCollider_ShouldClearPrimaryPartitionMembershipInPureRuntime(GravitasSerializationTransport transport)
    {
        using GravitasWorldContext sourceContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        LSCircleCollider2D source = CreateStaticCircle(sourceContext, Vector2d.Zero);
        source.Deactivate();
        object payload = GravitasSerializationHarness.Serialize(source, transport);

        using GravitasWorldContext targetContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        LSCircleCollider2D target = CreateStaticCircle(targetContext, Vector2d.Zero);
        target.IsPartitioned.Should().BeTrue();
        target.IsMixedPartitioned.Should().BeFalse();
        WorldVoxelIndex coordinate = target.PartitionCoordinates![0];
        SerializationPartitionAssertions.Primary2DPartitionContains(
            targetContext,
            coordinate,
            target.Id).Should().BeTrue();

        GravitasSerializationHarness.Populate(target, payload, transport);

        target.IsActive.Should().BeFalse();
        target.IsPartitioned.Should().BeFalse();
        target.IsMixedPartitioned.Should().BeFalse();
        target.PartitionCoordinates.Should().BeEmpty();
        (target.MixedPartitionCoordinates?.Count ?? 0).Should().Be(0);
        SerializationPartitionAssertions.Primary2DPartitionContains(
            targetContext,
            coordinate,
            target.Id).Should().BeFalse();

        GravitasSerializationHarness.Populate(target, payload, transport);

        target.IsActive.Should().BeFalse();
        target.IsPartitioned.Should().BeFalse();
        target.IsMixedPartitioned.Should().BeFalse();
        target.PartitionCoordinates.Should().BeEmpty();
        (target.MixedPartitionCoordinates?.Count ?? 0).Should().Be(0);
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void PopulateActiveColliderShape_ShouldRefreshPrimaryAndMixedPartitionMembership(
        GravitasSerializationTransport transport)
    {
        using GravitasWorldContext sourceContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        LSCircleCollider2D source = CreateStaticCircle(sourceContext, Vector2d.Zero);
        source.LocalOffset = new Vector2d((Fixed64)6, Fixed64.Zero);
        source.Simulate();
        object payload = GravitasSerializationHarness.Serialize(source, transport);

        using GravitasWorldContext targetContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        targetContext.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        LSCircleCollider2D target = CreateStaticCircle(targetContext, Vector2d.Zero);
        targetContext.MixedCollisions.Refresh2DColliderPartition(target);
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
        SerializationPartitionAssertions.Primary2DPartitionsShouldContain(
            targetContext,
            target.PartitionCoordinates!,
            target.Id);
        SerializationPartitionAssertions.Mixed2DPartitionsShouldContain(
            targetContext,
            target.MixedPartitionCoordinates!,
            target.Id);
        SerializationPartitionAssertions.StalePrimary2DPartitionsShouldBeCleared(
            targetContext,
            primaryCoordinatesBeforeLoad,
            target.PartitionCoordinates!,
            target.Id).Should().BeTrue("the loaded local offset should move the collider out of an original primary partition");
        SerializationPartitionAssertions.StaleMixed2DPartitionsShouldBeCleared(
            targetContext,
            mixedCoordinatesBeforeLoad,
            target.MixedPartitionCoordinates!,
            target.Id).Should().BeTrue("the loaded local offset should move the collider out of an original mixed partition");
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void PopulateActiveColliderShape_ShouldRefreshPrimaryPartitionMembershipInPureRuntime(
        GravitasSerializationTransport transport)
    {
        using GravitasWorldContext sourceContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        LSCircleCollider2D source = CreateStaticCircle(sourceContext, Vector2d.Zero);
        source.LocalOffset = new Vector2d((Fixed64)6, Fixed64.Zero);
        source.Simulate();
        object payload = GravitasSerializationHarness.Serialize(source, transport);

        using GravitasWorldContext targetContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        LSCircleCollider2D target = CreateStaticCircle(targetContext, Vector2d.Zero);
        WorldVoxelIndex[] primaryCoordinatesBeforeLoad =
            SerializationPartitionAssertions.CopyCoordinates(target.PartitionCoordinates!);

        GravitasSerializationHarness.Populate(target, payload, transport);

        target.IsActive.Should().BeTrue();
        target.LocalOffset.Should().Be(source.LocalOffset);
        target.Bounds.Should().Be(source.Bounds);
        target.IsPartitioned.Should().BeTrue();
        target.IsMixedPartitioned.Should().BeFalse();
        (target.MixedPartitionCoordinates?.Count ?? 0).Should().Be(0);
        SerializationPartitionAssertions.Primary2DPartitionsShouldContain(
            targetContext,
            target.PartitionCoordinates!,
            target.Id);
        SerializationPartitionAssertions.StalePrimary2DPartitionsShouldBeCleared(
            targetContext,
            primaryCoordinatesBeforeLoad,
            target.PartitionCoordinates!,
            target.Id).Should().BeTrue("the loaded local offset should move the collider out of an original primary partition");
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void PopulateTriggerColliderIntoBodyCollider_ShouldRejectInvalidLoadedState(GravitasSerializationTransport transport)
    {
        using GravitasWorldContext sourceContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        LSCircleCollider2D source = CreateStaticCircle(sourceContext, Vector2d.Zero);
        source.IsTrigger = true;
        object payload = GravitasSerializationHarness.Serialize(source, transport);

        using GravitasWorldContext targetContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        SolidBody2D target = CreateDynamicCircle(targetContext);
        Action loadTriggerOntoBodyCollider = () => GravitasSerializationHarness.Populate(target.Collider, payload, transport);

        loadTriggerOntoBodyCollider.Should().Throw<ArgumentException>().WithParameterName(nameof(LSCollider2D.IsTrigger));
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void PopulateUnboundCollider_ShouldApplyShapeStateWithoutPartitionRefresh(GravitasSerializationTransport transport)
    {
        using GravitasWorldContext sourceContext = Physics2DTestWorld.CreateContext(frameRate: 8);
        LSCircleCollider2D source = CreateStaticCircle(sourceContext, Vector2d.Zero);
        source.Radius = Fixed64.FromFraction(3, 2);
        source.LocalOffset = new Vector2d(Fixed64.Half, -Fixed64.Half);
        source.MixedHalfThicknessOverride = Fixed64.FromFraction(5, 4);
        source.Layer = new PhysicsLayer(3);
        source.Simulate();
        object payload = GravitasSerializationHarness.Serialize(source, transport);
        var target = new LSCircleCollider2D(Fixed64.Half);

        GravitasSerializationHarness.Populate(target, payload, transport);

        target.Radius.Should().Be(source.Radius);
        target.LocalOffset.Should().Be(source.LocalOffset);
        target.MixedHalfThicknessOverride.Should().Be(source.MixedHalfThicknessOverride);
        target.Layer.Should().Be(source.Layer);
        target.IsActive.Should().BeTrue();
        target.IsPartitioned.Should().BeFalse();
        target.IsMixedPartitioned.Should().BeFalse();
    }

    private static SolidBody2D CreateDynamicCircle(GravitasWorldContext context, Vector2d position = default)
    {
        var agent = new TestMatterAgent(context, new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One));
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = (Fixed64)2
        };
        body.Initialize(position);
        return body;
    }

    private static LSCircleCollider2D CreateStaticCircle(GravitasWorldContext context, Vector2d position)
    {
        var agent = new TestMatterAgent(context, new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One));
        var collider = new LSCircleCollider2D(Fixed64.Half);
        collider.InitializeWithNoBody(agent);
        return collider;
    }

    private static void CreateStaticFloor(GravitasWorldContext context)
    {
        var agent = new TestMatterAgent(context);
        var collider = new LSAABBoxCollider2D(new Vector2d((Fixed64)8, Fixed64.One));
        collider.InitializeWithNoBody(agent);
    }

}
