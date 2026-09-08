//=======================================================================
// SolidBodyColliderReconfigurationTests.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Materials;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Serialization;
using Gravitas.Tests.Support;
using GridForge.Spatial;
using SwiftCollections.Diagnostics;
using System;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class SolidBodyColliderReconfigurationTests
{
    [Fact]
    public void TryReconfigureCollider_WhenDefinitionPoseAndOffsetMatch_ShouldReturnUnchanged()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCapsuleCollider> actor = CreateCapsuleActor(
            scenario,
            new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Zero),
            isKinematic: false);
        ChronicleHash replayHash = scenario.Context.ComputeReplayHash();
        uint shapeVersion = actor.Collider.RuntimeShapeVersion;
        int synchronizedPublications = 0;

        ColliderReconfigurationStatus status = actor.Body.TryReconfigureCollider(
            ColliderShapeDefinition.Capsule(Fixed64.Half, Fixed64.One),
            Vector3d.Zero,
            actor.Body.Position3d,
            out LSCollider? blocker,
            out Exception? notificationException,
            publishSynchronizedState: () => synchronizedPublications++);

        status.Should().Be(ColliderReconfigurationStatus.Unchanged);
        blocker.Should().BeNull();
        notificationException.Should().BeNull();
        synchronizedPublications.Should().Be(1);
        actor.Collider.RuntimeShapeVersion.Should().Be(shapeVersion);
        scenario.Context.ComputeReplayHash().Should().Be(replayHash);
    }

    [Fact]
    public void TryReconfigureCollider_WhenMatchingAuthoredGeometryIsNotCommitted_ShouldApply()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCapsuleCollider> actor = CreateCapsuleActor(scenario, Vector3d.Zero);
        uint shapeVersion = actor.Collider.RuntimeShapeVersion;
        actor.Collider.Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One);

        ColliderReconfigurationStatus status = actor.Body.TryReconfigureCollider(
            ColliderShapeDefinition.Capsule(Fixed64.Half, (Fixed64)2),
            Vector3d.Zero,
            actor.Body.Position3d,
            out _,
            out _);

        status.Should().Be(ColliderReconfigurationStatus.Applied);
        actor.Collider.RuntimeShapeVersion.Should().Be(shapeVersion + 1);
        actor.Collider.AxisLength.Should().Be(Fixed64.One);
    }

    [Fact]
    public void TryReconfigureCollider_WhenCapsuleFits_ShouldAtomicallyPublishShapePoseAndMassState()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCapsuleCollider> actor = CreateCapsuleActor(
            scenario,
            new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Zero),
            isKinematic: false);
        var retainedMaterial = new PhysicsMaterial(
            Fixed64.FromFraction(3, 4),
            Fixed64.Half,
            Fixed64.FromFraction(1, 4));
        actor.Collider.Material = retainedMaterial;
        actor.Collider.Layer = new PhysicsLayer(3);
        actor.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(7);
        var transformParent = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        actor.Body.PositionTransform.SetParentKeepingLocal(transformParent);
        FixedQuaternion retainedRotation = PhysicsScenarioBuilder.Yaw(45);
        actor.Body.SetRotation(retainedRotation);
        FixedQuaternion retainedHostRotation = actor.Body.RotationTransform.WorldRotation;
        LSSphereCollider hierarchyParent = scenario.CreateStaticSphere(
            Vector3d.Left * (Fixed64)8);
        actor.Collider.SetParent(hierarchyParent);
        actor.Body.AddLinearImpulse(Vector3d.Right);
        actor.Body.AddAngularImpulse(Vector3d.Up);
        int colliderId = actor.Collider.Id;
        BodyMotionType motionType = actor.Body.MotionType;
        Vector3d retainedLinearVelocity = actor.Body.LinearVelocity;
        Vector3d retainedAngularVelocity = actor.Body.AngularVelocity;
        Vector3d oldCenterOfMass = actor.Body.LocalCenterOfMassOffset;
        Fixed3x3 oldInverseInertia = actor.Body.InverseInertiaTensor;
        uint oldShapeVersion = actor.Collider.RuntimeShapeVersion;
        Vector3d localOffset = Vector3d.Up;
        Vector3d rootPosition = new((Fixed64)5, (Fixed64)2, Fixed64.Zero);

        ColliderReconfigurationStatus status = actor.Body.TryReconfigureCollider(
            ColliderShapeDefinition.Capsule(
                Fixed64.Half,
                (Fixed64)2,
                PhysicsMaterial.Frictionless),
            localOffset,
            rootPosition,
            out LSCollider? blocker,
            out Exception? notificationException);

        status.Should().Be(ColliderReconfigurationStatus.Applied);
        blocker.Should().BeNull();
        notificationException.Should().BeNull();
        actor.Collider.Id.Should().Be(colliderId);
        actor.Collider.Body.Should().BeSameAs(actor.Body);
        actor.Body.Collider.Should().BeSameAs(actor.Collider);
        actor.Body.MotionType.Should().Be(motionType);
        actor.Body.LinearVelocity.Should().Be(retainedLinearVelocity);
        actor.Body.AngularVelocity.Should().Be(retainedAngularVelocity);
        actor.Body.Rotation.Should().Be(retainedRotation);
        actor.Body.RotationTransform.WorldRotation.Should().Be(retainedHostRotation);
        actor.Collider.Parent3D.Should().BeSameAs(hierarchyParent);
        actor.Collider.Material.Should().Be(retainedMaterial);
        actor.Collider.Layer.Should().Be(new PhysicsLayer(3));
        actor.Collider.IgnoredCollisionLayers.Should().Be(PhysicsLayerMask.FromLayer(7));
        actor.Collider.Radius.Should().Be(Fixed64.Half);
        actor.Collider.Size.Should().Be(new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One));
        actor.Collider.LocalOffset.Should().Be(localOffset);
        actor.Body.Position3d.Should().Be(rootPosition);
        actor.Body.PositionTransform.WorldPosition.Should().Be(rootPosition);
        actor.Collider.Center.Should().Be(rootPosition + localOffset);
        actor.Collider.RuntimeShapeVersion.Should().Be(oldShapeVersion + 1);
        actor.Body.LocalCenterOfMassOffset.Should().Be(localOffset);
        actor.Body.LocalCenterOfMassOffset.Should().NotBe(oldCenterOfMass);
        actor.Body.InverseInertiaTensor.Should().NotBe(oldInverseInertia);
    }

    [Fact]
    public void TryReconfigureCollider_WhenCandidateOverlapsTriggerOnly_ShouldApply()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> actor = scenario.CreateBody(
            new LSSphereCollider(ColliderShapeDefinition.Sphere(Fixed64.Half)),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        LSSphereCollider trigger = scenario.CreateStaticSphere(Vector3d.Right);
        trigger.IsTrigger = true;

        ColliderReconfigurationStatus status = actor.Body.TryReconfigureCollider(
            ColliderShapeDefinition.Sphere((Fixed64)2),
            Vector3d.Zero,
            Vector3d.Zero,
            out LSCollider? blocker,
            out Exception? notificationException);

        status.Should().Be(ColliderReconfigurationStatus.Applied);
        blocker.Should().BeNull();
        notificationException.Should().BeNull();
        actor.Collider.ScaledRadius.Should().Be((Fixed64)2);
    }

    [Fact]
    public void TryReconfigureCollider_WhenEqualPriorityColliderWouldBePenetrated_ShouldBlock()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> actor = scenario.CreateBody(
            new LSSphereCollider(ColliderShapeDefinition.Sphere(Fixed64.Half)),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        LSSphereCollider obstacle = scenario.CreateStaticSphere(Vector3d.Right);
        int synchronizedPublications = 0;

        ColliderReconfigurationStatus status = actor.Body.TryReconfigureCollider(
            ColliderShapeDefinition.Sphere((Fixed64)2),
            Vector3d.Zero,
            Vector3d.Zero,
            out LSCollider? blocker,
            out Exception? notificationException,
            publishSynchronizedState: () => synchronizedPublications++);

        status.Should().Be(ColliderReconfigurationStatus.Blocked);
        blocker.Should().BeSameAs(obstacle);
        notificationException.Should().BeNull();
        synchronizedPublications.Should().Be(0);
        actor.Collider.ScaledRadius.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void TryReconfigureCollider_WhenOnlyTouchingSupport_ShouldApply()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCapsuleCollider> actor = CreateCapsuleActor(
            scenario,
            new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Zero));
        LSCuboidCollider floor = CreateStaticCuboid(
            scenario,
            new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero),
            new Vector3d((Fixed64)8, Fixed64.One, (Fixed64)8));

        ColliderReconfigurationStatus status = actor.Body.TryReconfigureCollider(
            ColliderShapeDefinition.Capsule(Fixed64.Half, (Fixed64)2),
            Vector3d.Zero,
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero),
            out LSCollider? blocker,
            out Exception? notificationException);

        status.Should().Be(ColliderReconfigurationStatus.Applied);
        blocker.Should().BeNull();
        notificationException.Should().BeNull();
        actor.Collider.BoundsMin.Y.Should().Be(Fixed64.Zero);
        floor.BoundsMax.Y.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void TryReconfigureCollider_WhenCeilingWouldBePenetrated_ShouldPreserveStateAndReplayEvidence()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCapsuleCollider> actor = CreateCapsuleActor(
            scenario,
            new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Zero));
        LSCuboidCollider floor = CreateStaticCuboid(
            scenario,
            new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero),
            new Vector3d((Fixed64)8, Fixed64.One, (Fixed64)8));
        LSCuboidCollider ceiling = CreateStaticCuboid(
            scenario,
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero),
            new Vector3d((Fixed64)8, Fixed64.Half, (Fixed64)8));
        CollisionHandling.CollisionPair existingPair =
            scenario.Context.Physics.GetCollisionPair(actor.Collider.Id, floor.Id)!;
        object serializedBefore = GravitasSerializationHarness.Serialize(
            actor.Body,
            GravitasSerializationTransport.Json);
        ChronicleHash replayHashBefore = scenario.Context.ComputeReplayHash();
        WorldVoxelIndex[] partitionsBefore =
            SerializationPartitionAssertions.CopyCoordinates(actor.Collider.PartitionCoordinates!);
        Vector3d positionBefore = actor.Body.Position3d;
        Vector3d hostPositionBefore = actor.Body.PositionTransform.WorldPosition;
        Vector3d centerBefore = actor.Collider.Center;
        FixedBoundBox boundsBefore = actor.Collider.Bounds;
        uint shapeVersionBefore = actor.Collider.RuntimeShapeVersion;

        ColliderReconfigurationStatus status = actor.Body.TryReconfigureCollider(
            ColliderShapeDefinition.Capsule(Fixed64.Half, (Fixed64)2),
            Vector3d.Zero,
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero),
            out LSCollider? blocker,
            out Exception? notificationException);

        status.Should().Be(ColliderReconfigurationStatus.Blocked);
        blocker.Should().BeSameAs(ceiling);
        notificationException.Should().BeNull();
        actor.Body.Position3d.Should().Be(positionBefore);
        actor.Body.PositionTransform.WorldPosition.Should().Be(hostPositionBefore);
        actor.Collider.Center.Should().Be(centerBefore);
        actor.Collider.Bounds.Should().Be(boundsBefore);
        actor.Collider.Radius.Should().Be(Fixed64.Half);
        actor.Collider.Size.Should().Be(Vector3d.One);
        actor.Collider.LocalOffset.Should().Be(Vector3d.Zero);
        actor.Collider.RuntimeShapeVersion.Should().Be(shapeVersionBefore);
        actor.Collider.PartitionCoordinates.Should().Equal(partitionsBefore);
        floor.TryGetCollisionPair(actor.Collider.Id, out CollisionHandling.CollisionPair? pairAfter)
            .Should().BeTrue("a blocked transaction must retain the existing pair registration");
        pairAfter.Should().BeSameAs(existingPair);
        existingPair.Active.Should().BeTrue("a blocked transaction must not invalidate pair state");
        scenario.Context.ComputeReplayHash().Should().Be(replayHashBefore);
        object serializedAfter = GravitasSerializationHarness.Serialize(
            actor.Body,
            GravitasSerializationTransport.Json);
        AssertSerializedPayloadEqual(serializedBefore, serializedAfter);
    }

    [Fact]
    public void TryReconfigureCollider_WhenApplied_ShouldRefreshPureMixedPartitionsPairsAndQueries()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        ScenarioBody<LSCapsuleCollider> actor = CreateCapsuleActor(
            scenario,
            new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Zero));
        LSCuboidCollider support = CreateStaticCuboid(
            scenario,
            new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero),
            new Vector3d((Fixed64)2, Fixed64.One, (Fixed64)2));
        scenario.Context.MixedCollisions.Refresh3DColliderPartition(actor.Collider).Should().BeTrue();
        CollisionHandling.CollisionPair oldPair =
            scenario.Context.Physics.GetCollisionPair(actor.Collider.Id, support.Id)!;
        WorldVoxelIndex[] oldPrimary =
            SerializationPartitionAssertions.CopyCoordinates(actor.Collider.PartitionCoordinates!);
        WorldVoxelIndex[] oldMixed =
            SerializationPartitionAssertions.CopyCoordinates(actor.Collider.MixedPartitionCoordinates!);
        Vector3d newPosition = new((Fixed64)8, Fixed64.One, Fixed64.Zero);

        ColliderReconfigurationStatus status = actor.Body.TryReconfigureCollider(
            ColliderShapeDefinition.Capsule(Fixed64.Half, (Fixed64)2),
            Vector3d.Zero,
            newPosition,
            out _,
            out Exception? notificationException);

        status.Should().Be(ColliderReconfigurationStatus.Applied);
        notificationException.Should().BeNull();
        oldPair.Active.Should().BeFalse();
        SerializationPartitionAssertions.StalePrimary3DPartitionsShouldBeCleared(
            scenario.Context,
            oldPrimary,
            actor.Collider.PartitionCoordinates!,
            actor.Collider.Id).Should().BeTrue();
        SerializationPartitionAssertions.StaleMixed3DPartitionsShouldBeCleared(
            scenario.Context,
            oldMixed,
            actor.Collider.MixedPartitionCoordinates!,
            actor.Collider.Id).Should().BeTrue();
        SerializationPartitionAssertions.Primary3DPartitionsShouldContain(
            scenario.Context,
            actor.Collider.PartitionCoordinates!,
            actor.Collider.Id);
        SerializationPartitionAssertions.Mixed3DPartitionsShouldContain(
            scenario.Context,
            actor.Collider.MixedPartitionCoordinates!,
            actor.Collider.Id);
        scenario.Context.Query3D.Raycast(
            newPosition - Vector3d.Right * (Fixed64)2,
            Vector3d.Right,
            (Fixed64)4,
            out Physics3DHit hit,
            PhysicsLayerMask.All).Should().BeTrue();
        hit.Collider.Should().BeSameAs(actor.Collider);
    }

    [Fact]
    public void TryReconfigureCollider_WhenPublishersThrow_ShouldReturnFailuresAfterRetiringAllPairs()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> actor = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> rightSupport = scenario.CreateSphere(
            Vector3d.Right * Fixed64.FromFraction(3, 4),
            immovable: true);
        ScenarioBody<LSSphereCollider> leftSupport = scenario.CreateSphere(
            Vector3d.Left * Fixed64.FromFraction(3, 4),
            immovable: true);
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();
        CollisionHandling.CollisionPair rightPair =
            scenario.Context.Physics.GetCollisionPair(actor.Collider.Id, rightSupport.Collider.Id)!;
        CollisionHandling.CollisionPair leftPair =
            scenario.Context.Physics.GetCollisionPair(actor.Collider.Id, leftSupport.Collider.Id)!;
        Vector3d newPosition = Vector3d.Right * (Fixed64)8;
        Fixed64 newRadius = Fixed64.One;
        int exits = 0;
        bool synchronizedStatePublished = false;
        bool callbacksObservedPublishedState = true;
        actor.Collider.OnContactExit += _ =>
        {
            exits++;
            callbacksObservedPublishedState &= synchronizedStatePublished
                && actor.Body.Position3d == newPosition
                && actor.Collider.ScaledRadius == newRadius;
            SwiftThrowHelper.ThrowIfTrue(
                true,
                message: $"reconfiguration exit failure {exits}");
        };

        ColliderReconfigurationStatus status = actor.Body.TryReconfigureCollider(
            ColliderShapeDefinition.Sphere(newRadius),
            Vector3d.Zero,
            newPosition,
            out _,
            out Exception? notificationException,
            publishSynchronizedState: () =>
            {
                synchronizedStatePublished = true;
                actor.Body.Position3d.Should().Be(newPosition);
                actor.Collider.ScaledRadius.Should().Be(newRadius);
                SwiftThrowHelper.ThrowIfTrue(
                    true,
                    message: "synchronized state publication failure");
            });

        status.Should().Be(ColliderReconfigurationStatus.Applied);
        AggregateException aggregate = notificationException
            .Should().BeOfType<AggregateException>().Which.Flatten();
        aggregate.InnerExceptions.Should().HaveCount(3);
        aggregate.InnerExceptions[0].Message.Should().Be("synchronized state publication failure");
        aggregate.InnerExceptions[1].Message.Should().Be("reconfiguration exit failure 1");
        aggregate.InnerExceptions[2].Message.Should().Be("reconfiguration exit failure 2");
        synchronizedStatePublished.Should().BeTrue();
        callbacksObservedPublishedState.Should().BeTrue();
        exits.Should().Be(2, "retirement should continue after one callback fails");
        actor.Body.Position3d.Should().Be(newPosition);
        actor.Body.PositionTransform.WorldPosition.Should().Be(newPosition);
        actor.Collider.ScaledRadius.Should().Be(newRadius);
        actor.Collider.Center.Should().Be(newPosition);
        rightPair.Active.Should().BeFalse();
        leftPair.Active.Should().BeFalse();
        actor.Collider.CollisionPairCount.Should().Be(0);
        actor.Collider.CollisionPairHolderCount.Should().Be(0);
        rightSupport.Collider.CollisionPairHolderCount.Should().Be(0);
        leftSupport.Collider.CollisionPairHolderCount.Should().Be(0);
    }

    [Fact]
    public void TryReconfigureCollider_WhenMixedExitCallbackThrows_ShouldPublishAndReturnFailure()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> actor = scenario.CreateBody(
            new LSSphereCollider(ColliderShapeDefinition.Sphere(Fixed64.Half)),
            Vector3d.Zero,
            FixedQuaternion.Identity,
            isKinematic: false);
        var supportAgent = new TestMatterAgent(
            scenario.Context,
            new FixedTransform(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.One));
        var support = new SolidBody2D(
            supportAgent,
            new LSCircleCollider2D(Fixed64.Half));
        support.Initialize(Vector2d.Zero, motionType: BodyMotionType.Static);
        var secondSupportAgent = new TestMatterAgent(
            scenario.Context,
            new FixedTransform(
                Vector3d.Right * Fixed64.Half,
                FixedQuaternion.Identity,
                Vector3d.One));
        var secondSupport = new SolidBody2D(
            secondSupportAgent,
            new LSCircleCollider2D(Fixed64.Half));
        secondSupport.Initialize(
            new Vector2d(Fixed64.Half, Fixed64.Zero),
            motionType: BodyMotionType.Static);
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();
        scenario.Context.MixedCollisions.ActivePairCount.Should().Be(2);
        Vector3d newPosition = Vector3d.Right * (Fixed64)8;
        Fixed64 newRadius = Fixed64.One;
        bool callbacksObservedPublishedState = true;
        int exits = 0;
        actor.Collider.OnMixedContactExit += _ =>
        {
            exits++;
            callbacksObservedPublishedState &= actor.Body.Position3d == newPosition
                && actor.Collider.ScaledRadius == newRadius;
            SwiftThrowHelper.ThrowIfTrue(
                exits == 1,
                message: "mixed reconfiguration exit failure");
        };

        ColliderReconfigurationStatus status = actor.Body.TryReconfigureCollider(
            ColliderShapeDefinition.Sphere(newRadius),
            Vector3d.Zero,
            newPosition,
            out _,
            out Exception? notificationException);

        status.Should().Be(ColliderReconfigurationStatus.Applied);
        notificationException.Should().BeOfType<InvalidOperationException>()
            .Which.Message.Should().Be("mixed reconfiguration exit failure");
        callbacksObservedPublishedState.Should().BeTrue();
        exits.Should().Be(2, "mixed retirement should continue after one callback fails");
        actor.Body.Position3d.Should().Be(newPosition);
        actor.Body.PositionTransform.WorldPosition.Should().Be(newPosition);
        actor.Collider.ScaledRadius.Should().Be(newRadius);
        scenario.Context.MixedCollisions.ActivePairCount.Should().Be(0);
    }

    [Fact]
    public void TryReconfigureCollider_WhenSphereOrCylinderFamilyMatches_ShouldApplyExactGeometry()
    {
        using PhysicsScenarioBuilder sphereScenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> sphere = sphereScenario.CreateBody(
            new LSSphereCollider(ColliderShapeDefinition.Sphere(Fixed64.Half)),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        using PhysicsScenarioBuilder cylinderScenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCylinderCollider> cylinder = cylinderScenario.CreateBody(
            new LSCylinderCollider(ColliderShapeDefinition.Cylinder(Fixed64.Half, Fixed64.One)),
            Vector3d.Zero,
            FixedQuaternion.Identity);

        sphere.Body.TryReconfigureCollider(
                ColliderShapeDefinition.Sphere(Fixed64.One),
                Vector3d.Zero,
                Vector3d.Zero,
                out _,
                out _)
            .Should().Be(ColliderReconfigurationStatus.Applied);
        cylinder.Body.TryReconfigureCollider(
                ColliderShapeDefinition.Cylinder(Fixed64.One, (Fixed64)3),
                Vector3d.Zero,
                Vector3d.Zero,
                out _,
                out _)
            .Should().Be(ColliderReconfigurationStatus.Applied);

        sphere.Collider.ScaledRadius.Should().Be(Fixed64.One);
        sphere.Collider.Bounds.Scope.Should().Be(Vector3d.One);
        cylinder.Collider.ScaledRadius.Should().Be(Fixed64.One);
        cylinder.Collider.Height.Should().Be((Fixed64)3);
    }

    [Fact]
    public void TryReconfigureCollider_WithInvalidDefaultWrongOrUnsupportedInput_ShouldRejectBeforeMutation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCapsuleCollider> capsule = CreateCapsuleActor(scenario, Vector3d.Zero);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(Vector3d.Right * (Fixed64)4);
        FixedBoundBox capsuleBounds = capsule.Collider.Bounds;
        FixedBoundBox cuboidBounds = cuboid.Collider.Bounds;

        Action useDefault = () => capsule.Body.TryReconfigureCollider(
            default,
            Vector3d.Zero,
            Vector3d.Zero,
            out _,
            out _);
        Action useWrongFamily = () => capsule.Body.TryReconfigureCollider(
            ColliderShapeDefinition.Sphere(Fixed64.Half),
            Vector3d.Zero,
            Vector3d.Zero,
            out _,
            out _);
        Action useUnsupportedFamily = () => cuboid.Body.TryReconfigureCollider(
            ColliderShapeDefinition.Cuboid(Vector3d.One),
            Vector3d.Zero,
            cuboid.Body.Position3d,
            out _,
            out _);
        Action useUnrepresentableCandidate = () => capsule.Body.TryReconfigureCollider(
            ColliderShapeDefinition.Capsule(Fixed64.Half, (Fixed64)2),
            Vector3d.Right,
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            out _,
            out _);

        useDefault.Should().Throw<ArgumentException>().WithParameterName(nameof(ColliderShapeDefinition));
        useWrongFamily.Should().Throw<ArgumentException>().WithParameterName("definition");
        useUnsupportedFamily.Should().Throw<ArgumentException>().WithParameterName("definition");
        useUnrepresentableCandidate.Should().Throw<ArgumentException>();
        capsule.Collider.Bounds.Should().Be(capsuleBounds);
        cuboid.Collider.Bounds.Should().Be(cuboidBounds);
    }

    [Fact]
    public void TryReconfigureCollider_DuringFixedStep_ShouldRejectBeforeMutation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCapsuleCollider> actor = CreateCapsuleActor(scenario, Vector3d.Zero);
        FixedBoundBox bounds = actor.Collider.Bounds;
        Vector3d position = actor.Body.Position3d;
        using IDisposable hook = scenario.Context.RegisterOnSimulate(
            "collider-reconfiguration",
            0,
            () => actor.Body.TryReconfigureCollider(
                ColliderShapeDefinition.Capsule(Fixed64.Half, (Fixed64)2),
                Vector3d.Zero,
                Vector3d.Up,
                out _,
                out _));

        Action simulate = scenario.Context.Simulate;

        simulate.Should().Throw<InvalidOperationException>();
        actor.Body.Position3d.Should().Be(position);
        actor.Collider.Bounds.Should().Be(bounds);
    }

    [Fact]
    public void TryReconfigureCollider_AfterDeactivation_ShouldRejectBeforeMutation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCapsuleCollider> actor = CreateCapsuleActor(scenario, Vector3d.Zero);
        actor.Body.Deactivate();

        Action reconfigure = () => actor.Body.TryReconfigureCollider(
            ColliderShapeDefinition.Capsule(Fixed64.Half, (Fixed64)2),
            Vector3d.Zero,
            Vector3d.Up,
            out _,
            out _);

        reconfigure.Should().Throw<InvalidOperationException>();
        actor.Collider.Id.Should().Be(-1);
        actor.Body.Active.Should().BeFalse();
    }

    [Fact]
    public void TryReconfigureCollider_WhenParentCannotRepresentRoot_ShouldRejectBeforeMutation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCapsuleCollider> actor = CreateCapsuleActor(scenario, Vector3d.Zero);
        var parent = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.One));
        actor.Body.PositionTransform.SetParentKeepingLocal(parent);
        Vector3d position = actor.Body.Position3d;
        FixedBoundBox bounds = actor.Collider.Bounds;

        Action reconfigure = () => actor.Body.TryReconfigureCollider(
            ColliderShapeDefinition.Capsule(Fixed64.Half, (Fixed64)2),
            Vector3d.Zero,
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            out _,
            out _);

        reconfigure.Should().Throw<InvalidOperationException>();
        actor.Body.Position3d.Should().Be(position);
        actor.Collider.Bounds.Should().Be(bounds);
    }

    private static ScenarioBody<LSCapsuleCollider> CreateCapsuleActor(
        PhysicsScenarioBuilder scenario,
        Vector3d position,
        bool isKinematic = true) =>
        scenario.CreateBody(
            new LSCapsuleCollider(
                ColliderShapeDefinition.Capsule(Fixed64.Half, Fixed64.One)),
            position,
            FixedQuaternion.Identity,
            isKinematic: isKinematic);

    private static LSCuboidCollider CreateStaticCuboid(
        PhysicsScenarioBuilder scenario,
        Vector3d position,
        Vector3d size)
    {
        var collider = new LSCuboidCollider(ColliderShapeDefinition.Cuboid(size));
        scenario.InitializeStaticCollider(collider, position);
        return collider;
    }

    private static void AssertSerializedPayloadEqual(object expected, object actual)
    {
        if (expected is byte[] expectedBytes)
        {
            actual.Should().BeOfType<byte[]>().Which.Should().Equal(expectedBytes);
            return;
        }

        actual.Should().Be(expected);
    }
}
