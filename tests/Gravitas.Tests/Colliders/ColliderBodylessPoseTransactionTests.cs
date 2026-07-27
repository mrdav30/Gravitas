using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Serialization;
using Gravitas.Tests.Support;
using GridForge.Spatial;
using System;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class ColliderBodylessPoseTransactionTests
{
    [Fact]
    public void Bodyless3DInitialization_ShouldUseOneAdmittedMatrixForCanonicalPose()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        FixedTransform transform = CreateRoundingSensitivePlanarHierarchy();
        transform.TryGetLocalToWorldMatrix(
            out Fixed4x4 admittedMatrix).Should().BeTrue();
        Fixed4x4.Decompose(
            admittedMatrix,
            out _,
            out FixedQuaternion matrixRotation,
            out _).Should().BeTrue();
        transform.WorldRotation.Should().NotBe(matrixRotation);
        var collider = new LSCuboidCollider();

        collider.InitializeWithNoBody(
            new TestMatterAgent(scenario.Context, transform));

        collider.Rotation.Should().Be(matrixRotation);
        collider.OrientedBox.Orientation.Should().Be(matrixRotation);
    }

    [Fact]
    public void Bodyless2DInitialization_ShouldUseOneAdmittedMatrixForCanonicalPose()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        FixedTransform transform = CreateRoundingSensitivePlanarHierarchy();
        transform.TryGetLocalToWorldMatrix(
            out Fixed4x4 admittedMatrix).Should().BeTrue();
        Fixed4x4.Decompose(
            admittedMatrix,
            out _,
            out FixedQuaternion matrixRotation,
            out _).Should().BeTrue();
        Vector3d matrixRight = matrixRotation.Rotate(Vector3d.Right);
        Fixed64 matrixYaw = PlanarRotation.Canonicalize(
            FixedMath.Atan2(matrixRight.Z, matrixRight.X));
        transform.WorldRotationXZRadians.Should().NotBe(matrixYaw);
        transform.WorldPosition.Y.Should().NotBe(admittedMatrix.M42);
        var collider = new LSAABBoxCollider2D(Vector2d.One);

        collider.InitializeWithNoBody(
            new TestMatterAgent(context, transform));

        collider.Rotation.Should().Be(matrixYaw);
        collider.MixedSlabCenterY.Should().Be(admittedMatrix.M42);
    }

    [Fact]
    public void Position_WhenParentedShapeAdmissionFails_ShouldRestorePoseAndCommittedMixedState()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        FixedTransform transform = CreateParentedTransform();
        var collider = new LSCuboidCollider
        {
            LocalOffset = new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
            Size = new Vector3d((Fixed64)4, (Fixed64)2, Fixed64.One)
        };
        collider.InitializeWithNoBody(new TestMatterAgent(scenario.Context, transform));
        scenario.Context.MixedCollisions.Refresh3DColliderPartition(collider).Should().BeTrue();

        Vector3d localPosition = transform.LocalPosition;
        FixedQuaternion localRotation = transform.LocalRotation;
        FixedBoundBox bounds = collider.Bounds;
        uint runtimeVersion = collider.RuntimeShapeVersion;
        uint broadPhaseVersion = collider.BroadPhaseVersion;
        Fixed64 area = collider.Area;
        Vector3d centerOfMass = collider.CalculateLocalCenterOfMassOffset();
        Fixed3x3 inertia = collider.CalculateInertiaTensor(Fixed64.One, centerOfMass);
        WorldVoxelIndex[] primaryCoordinates =
            SerializationPartitionAssertions.CopyCoordinates(collider.PartitionCoordinates!);
        WorldVoxelIndex[] mixedCoordinates =
            SerializationPartitionAssertions.CopyCoordinates(collider.MixedPartitionCoordinates!);
        transform.LocalScale = new Vector3d(-Fixed64.One, Fixed64.One, Fixed64.One);

        Action setPosition = () => collider.Position = new Vector3d((Fixed64)6, Fixed64.Zero, Fixed64.Zero);

        setPosition.Should().Throw<ArgumentException>().WithParameterName("scale");
        transform.LocalPosition.Should().Be(localPosition);
        transform.LocalRotation.Should().Be(localRotation);
        collider.Bounds.Should().Be(bounds);
        collider.RuntimeShapeVersion.Should().Be(runtimeVersion);
        collider.BroadPhaseVersion.Should().Be(broadPhaseVersion);
        collider.Area.Should().Be(area);
        collider.CalculateLocalCenterOfMassOffset().Should().Be(centerOfMass);
        collider.CalculateInertiaTensor(Fixed64.One, centerOfMass).Should().Be(inertia);
        collider.PartitionCoordinates.Should().Equal(primaryCoordinates);
        collider.MixedPartitionCoordinates.Should().Equal(mixedCoordinates);
        SerializationPartitionAssertions.Primary3DPartitionsShouldContain(
            scenario.Context,
            collider.PartitionCoordinates!,
            collider.Id);
        SerializationPartitionAssertions.Mixed3DPartitionsShouldContain(
            scenario.Context,
            collider.MixedPartitionCoordinates!,
            collider.Id);
    }

    [Fact]
    public void Rotation_WhenParentedShapeAdmissionFails_ShouldRestorePoseAndCommittedPureState()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        FixedTransform transform = CreateParentedTransform();
        var collider = new LSCuboidCollider
        {
            LocalOffset = new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
            Size = new Vector3d((Fixed64)4, (Fixed64)2, Fixed64.One)
        };
        collider.InitializeWithNoBody(new TestMatterAgent(scenario.Context, transform));

        Vector3d localPosition = transform.LocalPosition;
        FixedQuaternion localRotation = transform.LocalRotation;
        FixedBoundBox bounds = collider.Bounds;
        uint runtimeVersion = collider.RuntimeShapeVersion;
        uint broadPhaseVersion = collider.BroadPhaseVersion;
        Fixed64 area = collider.Area;
        Vector3d centerOfMass = collider.CalculateLocalCenterOfMassOffset();
        Fixed3x3 inertia = collider.CalculateInertiaTensor(Fixed64.One, centerOfMass);
        WorldVoxelIndex[] primaryCoordinates =
            SerializationPartitionAssertions.CopyCoordinates(collider.PartitionCoordinates!);
        transform.LocalScale = new Vector3d(Fixed64.One, -Fixed64.One, Fixed64.One);

        Action setRotation = () => collider.Rotation = PhysicsScenarioBuilder.Yaw(90);

        setRotation.Should().Throw<ArgumentException>().WithParameterName("scale");
        transform.LocalPosition.Should().Be(localPosition);
        transform.LocalRotation.Should().Be(localRotation);
        collider.Bounds.Should().Be(bounds);
        collider.RuntimeShapeVersion.Should().Be(runtimeVersion);
        collider.BroadPhaseVersion.Should().Be(broadPhaseVersion);
        collider.Area.Should().Be(area);
        collider.CalculateLocalCenterOfMassOffset().Should().Be(centerOfMass);
        collider.CalculateInertiaTensor(Fixed64.One, centerOfMass).Should().Be(inertia);
        collider.PartitionCoordinates.Should().Equal(primaryCoordinates);
        SerializationPartitionAssertions.Primary3DPartitionsShouldContain(
            scenario.Context,
            collider.PartitionCoordinates!,
            collider.Id);
    }

    [Fact]
    public void BodylessPose_WhenValid_ShouldRebuildAndRepartitionPureAndMixedStateImmediately()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        FixedTransform transform = CreateParentedTransform();
        var collider = new LSCuboidCollider
        {
            Size = new Vector3d((Fixed64)6, (Fixed64)2, (Fixed64)2)
        };
        collider.InitializeWithNoBody(new TestMatterAgent(scenario.Context, transform));
        scenario.Context.MixedCollisions.Refresh3DColliderPartition(collider).Should().BeTrue();

        uint runtimeVersion = collider.RuntimeShapeVersion;
        uint broadPhaseVersion = collider.BroadPhaseVersion;
        WorldVoxelIndex[] oldPrimaryCoordinates =
            SerializationPartitionAssertions.CopyCoordinates(collider.PartitionCoordinates!);
        WorldVoxelIndex[] oldMixedCoordinates =
            SerializationPartitionAssertions.CopyCoordinates(collider.MixedPartitionCoordinates!);
        Vector3d newPosition = new((Fixed64)6, Fixed64.Zero, Fixed64.Zero);

        collider.Position = newPosition;

        transform.WorldPosition.Should().Be(newPosition);
        collider.Center.Should().Be(newPosition);
        collider.Bounds.Center.Should().Be(newPosition);
        collider.RuntimeShapeVersion.Should().Be(runtimeVersion + 1);
        collider.BroadPhaseVersion.Should().Be(broadPhaseVersion + 1);
        SerializationPartitionAssertions.StalePrimary3DPartitionsShouldBeCleared(
            scenario.Context,
            oldPrimaryCoordinates,
            collider.PartitionCoordinates!,
            collider.Id).Should().BeTrue();
        SerializationPartitionAssertions.StaleMixed3DPartitionsShouldBeCleared(
            scenario.Context,
            oldMixedCoordinates,
            collider.MixedPartitionCoordinates!,
            collider.Id).Should().BeTrue();
        SerializationPartitionAssertions.Primary3DPartitionsShouldContain(
            scenario.Context,
            collider.PartitionCoordinates!,
            collider.Id);
        SerializationPartitionAssertions.Mixed3DPartitionsShouldContain(
            scenario.Context,
            collider.MixedPartitionCoordinates!,
            collider.Id);

        FixedBoundBox translatedBounds = collider.Bounds;
        runtimeVersion = collider.RuntimeShapeVersion;
        broadPhaseVersion = collider.BroadPhaseVersion;
        oldPrimaryCoordinates =
            SerializationPartitionAssertions.CopyCoordinates(collider.PartitionCoordinates!);
        oldMixedCoordinates =
            SerializationPartitionAssertions.CopyCoordinates(collider.MixedPartitionCoordinates!);
        FixedQuaternion newRotation = PhysicsScenarioBuilder.Yaw(90);

        collider.Rotation = newRotation;

        transform.WorldRotation.Should().Be(newRotation);
        collider.Rotation.Should().Be(newRotation);
        collider.Bounds.Should().NotBe(translatedBounds);
        collider.RuntimeShapeVersion.Should().Be(runtimeVersion + 1);
        collider.BroadPhaseVersion.Should().Be(broadPhaseVersion + 1);
        SerializationPartitionAssertions.StalePrimary3DPartitionsShouldBeCleared(
            scenario.Context,
            oldPrimaryCoordinates,
            collider.PartitionCoordinates!,
            collider.Id).Should().BeTrue();
        SerializationPartitionAssertions.StaleMixed3DPartitionsShouldBeCleared(
            scenario.Context,
            oldMixedCoordinates,
            collider.MixedPartitionCoordinates!,
            collider.Id).Should().BeTrue();
        SerializationPartitionAssertions.Primary3DPartitionsShouldContain(
            scenario.Context,
            collider.PartitionCoordinates!,
            collider.Id);
        SerializationPartitionAssertions.Mixed3DPartitionsShouldContain(
            scenario.Context,
            collider.MixedPartitionCoordinates!,
            collider.Id);
    }

    [Fact]
    public void PoseSetters_WhenColliderHasBodyOrCompoundOwner_ShouldRejectExplicitly()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> body = scenario.CreateCuboid(Vector3d.Zero);
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero));
        LSCollider part = compound.GetPartCollider(0);
        Vector3d bodyPosition = body.Collider.Position;
        FixedQuaternion bodyRotation = body.Collider.Rotation;

        Action setBodyPosition = () => body.Collider.Position = Vector3d.Right;
        Action setBodyRotation = () => body.Collider.Rotation = PhysicsScenarioBuilder.Yaw(45);
        Action setPartPosition = () => part.Position = Vector3d.Right;
        Action setPartRotation = () => part.Rotation = PhysicsScenarioBuilder.Yaw(45);

        setBodyPosition.Should().Throw<InvalidOperationException>();
        setBodyRotation.Should().Throw<InvalidOperationException>();
        setPartPosition.Should().Throw<InvalidOperationException>();
        setPartRotation.Should().Throw<InvalidOperationException>();
        body.Collider.Position.Should().Be(bodyPosition);
        body.Collider.Rotation.Should().Be(bodyRotation);
    }

    [Fact]
    public void BodylessPose_DuringFixedStep_ShouldRejectBeforeTransformOrGeometryMutation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider collider = scenario.CreateStaticSphere(Vector3d.Zero);
        Vector3d localPosition = collider.Transform.LocalPosition;
        FixedBoundBox bounds = collider.Bounds;
        uint runtimeVersion = collider.RuntimeShapeVersion;
        using IDisposable hook = scenario.Context.RegisterOnSimulate(
            "bodyless-pose-transaction",
            0,
            () => collider.Position = Vector3d.Right);

        Action simulate = scenario.Context.Simulate;

        simulate.Should().Throw<InvalidOperationException>();
        collider.Transform.LocalPosition.Should().Be(localPosition);
        collider.Bounds.Should().Be(bounds);
        collider.RuntimeShapeVersion.Should().Be(runtimeVersion);
    }

    [Fact]
    public void InactiveBodylessPose_ShouldCommitGeometryWithoutRepartitioning()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider collider = scenario.CreateStaticSphere(Vector3d.Zero);
        collider.IsActive = false;
        uint runtimeVersion = collider.RuntimeShapeVersion;

        collider.Position = Vector3d.Right * (Fixed64)4;

        collider.Center.Should().Be(Vector3d.Right * (Fixed64)4);
        collider.Bounds.Center.Should().Be(collider.Center);
        collider.RuntimeShapeVersion.Should().Be(runtimeVersion + 1);
        collider.IsPartitioned.Should().BeFalse();

        collider.IsActive = true;

        collider.IsPartitioned.Should().BeTrue();
        SerializationPartitionAssertions.Primary3DPartitionsShouldContain(
            scenario.Context,
            collider.PartitionCoordinates!,
            collider.Id);
    }

    private static FixedTransform CreateParentedTransform()
    {
        var parent = new FixedTransform(
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);
        return new FixedTransform(
            new Vector3d((Fixed64)(-8), Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One,
            parent);
    }

    private static FixedTransform CreateRoundingSensitivePlanarHierarchy()
    {
        var root = new FixedTransform(
            new Vector3d(
                Fixed64.Zero,
                Fixed64.FromFraction(1, 7),
                Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees(
                Fixed64.Zero,
                (Fixed64)(-179),
                Fixed64.Zero),
            new Vector3d(
                Fixed64.One,
                Fixed64.FromFraction(6, 7),
                Fixed64.One));
        var parent = new FixedTransform(
            new Vector3d(
                Fixed64.Zero,
                Fixed64.FromFraction(1, 6),
                Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees(
                Fixed64.Zero,
                (Fixed64)(-179),
                Fixed64.Zero),
            new Vector3d(
                Fixed64.One,
                Fixed64.FromFraction(5, 6),
                Fixed64.One),
            root);
        var intermediate = new FixedTransform(
            new Vector3d(
                Fixed64.Zero,
                Fixed64.FromFraction(2, 7),
                Fixed64.Zero),
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.One,
                Fixed64.FromFraction(7, 6),
                Fixed64.One),
            parent);
        return new FixedTransform(
            new Vector3d(
                Fixed64.Zero,
                Fixed64.Half,
                Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees(
                Fixed64.Zero,
                (Fixed64)(-170),
                Fixed64.Zero),
            Vector3d.One,
            intermediate);
    }
}
