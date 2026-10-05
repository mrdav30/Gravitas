using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class SolidBody2DMassPropertiesTests
{
    [Fact]
    public void EffectiveMassHelpers_ShouldSeparateTranslationAndRotationPolicy()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.One), mass: (Fixed64)2);

        body.CanTranslate.Should().BeTrue();
        body.CanRotate.Should().BeTrue();
        body.EffectiveInverseMass.Should().Be(Fixed64.Half);
        body.MomentOfInertia.Should().Be(Fixed64.One);
        body.EffectiveInverseMomentOfInertia.Should().Be(Fixed64.One);

        body.FreezeAxes = BodyFreezeAxes2D.Rotation;

        body.CanTranslate.Should().BeTrue();
        body.CanRotate.Should().BeFalse();
        body.EffectiveInverseMass.Should().Be(Fixed64.Half);
        body.EffectiveInverseMomentOfInertia.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [InlineData(false, true)]
    [InlineData(true, false)]
    public void EffectiveMassHelpers_ForKinematicOrPositionFrozenBody_ShouldApplyRoleAndAxisIndependently(
        bool positionFrozen,
        bool isKinematic)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D body = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.One),
            mass: (Fixed64)2,
            positionFrozen: positionFrozen,
            motionType: isKinematic ? BodyMotionType.Kinematic : BodyMotionType.Dynamic);

        body.InverseMass.Should().Be(Fixed64.Half);
        body.InverseMomentOfInertia.Should().Be(Fixed64.One);
        body.CanTranslate.Should().BeFalse();
        body.CanRotate.Should().Be(!isKinematic);
        body.EffectiveInverseMass.Should().Be(Fixed64.Zero);
        body.EffectiveInverseMomentOfInertia.Should().Be(isKinematic ? Fixed64.Zero : Fixed64.One);
    }

    [Fact]
    public void EffectiveMassHelpers_ForZeroMassBody_ShouldDisableSolverMotion()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.One), mass: Fixed64.Zero);

        body.InverseMass.Should().Be(Fixed64.Zero);
        body.MomentOfInertia.Should().Be(Fixed64.Zero);
        body.CanTranslate.Should().BeFalse();
        body.CanRotate.Should().BeFalse();
        body.EffectiveInverseMass.Should().Be(Fixed64.Zero);
        body.EffectiveInverseMomentOfInertia.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void MassSetter_ShouldRefreshScalarMomentFromColliderShape()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D((Fixed64)2), mass: (Fixed64)2);

        body.MomentOfInertia.Should().Be((Fixed64)4);

        body.Mass = (Fixed64)4;

        body.MomentOfInertia.Should().Be((Fixed64)8);
        body.InverseMomentOfInertia.Should().Be(Fixed64.FromFraction(1, 8));
        body.InverseMass.Should().Be(Fixed64.FromFraction(1, 4));
        body.CanTranslate.Should().BeTrue();
        body.CanRotate.Should().BeTrue();
        body.IsAwakeForCollision.Should().BeTrue();
        body.MotionType.Should().Be(BodyMotionType.Dynamic);
    }

    [Theory]
    [InlineData(false, 0)]
    [InlineData(true, 0)]
    [InlineData(false, -2)]
    [InlineData(true, -2)]
    public void MassSetter_AcrossNonpositiveMass_ShouldPublishMobilityAndAwakeMembershipWithoutChangingDynamicRole(
        bool initiallyNonpositive, int nonpositiveMass)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.One),
            mass: initiallyNonpositive ? (Fixed64)nonpositiveMass : (Fixed64)2);
        var coordinates = body.Collider.PartitionCoordinates!;
        int coordinateCount = coordinates.Count;
        int dynamicId = body.DynamicId;
        coordinateCount.Should().BeGreaterThan(0);
        body.IsAwakeForCollision.Should().Be(!initiallyNonpositive);

        body.Mass = initiallyNonpositive ? (Fixed64)2 : (Fixed64)nonpositiveMass;

        body.MotionType.Should().Be(BodyMotionType.Dynamic);
        body.DynamicId.Should().Be(dynamicId);
        body.Collider.PartitionCoordinates.Should().BeSameAs(coordinates);
        coordinates.Count.Should().Be(coordinateCount);
        body.CanTranslate.Should().Be(initiallyNonpositive);
        body.CanRotate.Should().Be(initiallyNonpositive);
        body.HasSolverMobility.Should().Be(initiallyNonpositive);
        body.IsAwakeForCollision.Should().Be(initiallyNonpositive);
        body.InverseMass.Should().Be(initiallyNonpositive ? Fixed64.Half : Fixed64.Zero);
        body.MomentOfInertia.Should().Be(initiallyNonpositive ? Fixed64.One : Fixed64.Zero);
        body.InverseMomentOfInertia.Should().Be(initiallyNonpositive ? Fixed64.One : Fixed64.Zero);
        foreach (var coordinate in coordinates)
        {
            context.World.TryGetVoxel(coordinate, out var voxel).Should().BeTrue();
            voxel!.TryGetPartition(out PhysicsPartition2D? partition).Should().BeTrue();
            partition!.ContainedDynamicObjects!.Contains(body.Collider.Id).Should().BeTrue();
            partition.DynamicObjectCount.Should().Be(1);
            partition.KinematicObjectCount.Should().Be(0);
            partition.StaticObjectCount.Should().Be(0);
            partition.AwakeDynamicObjectCount.Should().Be(initiallyNonpositive ? 1 : 0);
        }

        body.AddLinearImpulse(Vector2d.Right);
        body.LinearVelocity.Should().Be(initiallyNonpositive ? Vector2d.Right * Fixed64.Half : Vector2d.Zero);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(-2)]
    [InlineData(4)]
    public void MassSetter_WhenSleeping_ShouldWakeOnlyForAnActualMassChange(int changedMass)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.One), (Fixed64)2);
        body.Sleep();
        body.IsSleeping.Should().BeTrue();

        body.Mass = (Fixed64)2;

        body.IsSleeping.Should().BeTrue();
        body.MomentOfInertia.Should().Be(Fixed64.One);
        foreach (var coordinate in body.Collider.PartitionCoordinates!)
        {
            context.World.TryGetVoxel(coordinate, out var voxel).Should().BeTrue();
            voxel!.TryGetPartition(out PhysicsPartition2D? partition).Should().BeTrue();
            partition!.AwakeDynamicObjectCount.Should().Be(0);
        }

        body.Mass = (Fixed64)changedMass;

        body.IsSleeping.Should().BeFalse();
        body.Mass.Should().Be((Fixed64)changedMass);
        body.IsAwakeForCollision.Should().Be(changedMass > 0);
        foreach (var coordinate in body.Collider.PartitionCoordinates!)
        {
            context.World.TryGetVoxel(coordinate, out var voxel).Should().BeTrue();
            voxel!.TryGetPartition(out PhysicsPartition2D? partition).Should().BeTrue();
            partition!.AwakeDynamicObjectCount.Should().Be(changedMass > 0 ? 1 : 0);
        }
    }

    [Fact]
    public void MassSetter_ShouldPreserveMotionAndQueuedAccelerationAcrossZeroMass()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        context.Environment.Gravity = Fixed64.Zero;
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.One), (Fixed64)2);
        body.AddLinearImpulse(Vector2d.Right * (Fixed64)2);
        body.AddAngularImpulse((Fixed64)2);
        body.AddForce(Vector2d.Right * (Fixed64)4);
        body.AddTorque((Fixed64)3);

        body.Mass = Fixed64.Zero;

        body.LinearVelocity.Should().Be(Vector2d.Right);
        body.AngularVelocity.Should().Be((Fixed64)2);
        body.HasSolverMobility.Should().BeFalse();

        body.Mass = (Fixed64)4;

        body.LinearVelocity.Should().Be(Vector2d.Right);
        body.AngularVelocity.Should().Be((Fixed64)2);
        context.Simulate();
        context.LateSimulate();

        // Forces are converted to queued accelerations when applied. Changing
        // mass must not clear or reinterpret those already accepted accelerations.
        body.LinearVelocity.Should().Be(Vector2d.Right * Fixed64.FromFraction(3, 2));
        body.AngularVelocity.Should().Be(Fixed64.FromFraction(11, 4));
    }

    [Fact]
    public void MassSetter_WhenCustomInertiaCalculationThrows_ShouldKeepMassInertiaAndSleepState()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var collider = new UnsupportedTestCollider2D { MomentOfInertia = (Fixed64)4 };
        SolidBody2D body = CreateBody(context, collider, (Fixed64)2);
        body.Sleep();
        collider.ThrowOnInertiaCalculation = true;
        Action changeMass = () => body.Mass = (Fixed64)4;

        // A host collider's calculator may reject a proposed mass. The setter
        // must publish neither the mass nor its wake transition before success.
        changeMass.Should().Throw<InvalidOperationException>();

        body.Mass.Should().Be((Fixed64)2);
        body.InverseMass.Should().Be(Fixed64.Half);
        body.MomentOfInertia.Should().Be((Fixed64)4);
        body.InverseMomentOfInertia.Should().Be(Fixed64.FromFraction(1, 4));
        body.IsSleeping.Should().BeTrue();
        foreach (var coordinate in body.Collider.PartitionCoordinates!)
        {
            context.World.TryGetVoxel(coordinate, out var voxel).Should().BeTrue();
            voxel!.TryGetPartition(out PhysicsPartition2D? partition).Should().BeTrue();
            partition!.AwakeDynamicObjectCount.Should().Be(0);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MassSetter_WhenUninitializedOrDeactivated_ShouldStoreConfigurationUntilInitialization(bool deactivated)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var collider = new UnsupportedTestCollider2D { MomentOfInertia = (Fixed64)4 };
        var body = new SolidBody2D(new TestMatterAgent(context), collider) { Mass = (Fixed64)2 };
        if (deactivated)
        {
            body.Initialize(Vector2d.Zero);
            body.Deactivate();
        }
        collider.ThrowOnInertiaCalculation = true;

        Action configureMass = () => body.Mass = (Fixed64)4;

        configureMass.Should().NotThrow();
        body.Mass.Should().Be((Fixed64)4);
        body.Active.Should().BeFalse();
        body.MomentOfInertia.Should().Be(deactivated ? (Fixed64)4 : Fixed64.Zero);
        context.Physics2D.ColliderCount.Should().Be(0);
        collider.ThrowOnInertiaCalculation = false;
        collider.MomentOfInertia = (Fixed64)8;

        body.Initialize(Vector2d.Zero);

        body.MomentOfInertia.Should().Be((Fixed64)8);
        body.InverseMomentOfInertia.Should().Be(Fixed64.FromFraction(1, 8));
        body.Mass.Should().Be((Fixed64)4);
    }

    [Theory]
    [InlineData(BodyMotionType.Static)]
    [InlineData(BodyMotionType.Kinematic)]
    public void MassSetter_ForNonDynamicBody_ShouldRefreshInertiaWithoutChangingPartitionRole(BodyMotionType motionType)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.One), (Fixed64)2,
            motionType: motionType);
        var coordinates = body.Collider.PartitionCoordinates!;
        int colliderId = body.Collider.Id;

        body.Mass = (Fixed64)4;

        body.MotionType.Should().Be(motionType);
        body.Collider.Id.Should().Be(colliderId);
        body.CanTranslate.Should().BeFalse();
        body.CanRotate.Should().BeFalse();
        body.MomentOfInertia.Should().Be((Fixed64)2);
        body.InverseMass.Should().Be(Fixed64.FromFraction(1, 4));
        body.Collider.PartitionCoordinates.Should().BeSameAs(coordinates);
        coordinates.Count.Should().BeGreaterThan(0);
        foreach (var coordinate in coordinates)
        {
            context.World.TryGetVoxel(coordinate, out var voxel).Should().BeTrue();
            voxel!.TryGetPartition(out PhysicsPartition2D? partition).Should().BeTrue();
            partition!.DynamicObjectCount.Should().Be(0);
            partition.AwakeDynamicObjectCount.Should().Be(0);
            partition.StaticObjectCount.Should().Be(motionType == BodyMotionType.Static ? 1 : 0);
            partition.KinematicObjectCount.Should().Be(motionType == BodyMotionType.Kinematic ? 1 : 0);
        }
    }

    [Fact]
    public void MassSetter_RepeatedRuntimeChanges_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.One), (Fixed64)2);

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(() =>
        {
            body.Mass = Fixed64.Zero;
            body.Mass = (Fixed64)(-2);
            body.Mass = (Fixed64)2;
            body.Mass = (Fixed64)4;
            body.Mass = (Fixed64)4;
        });

        allocatedBytes.Should().Be(0);
        body.Mass.Should().Be((Fixed64)4);
        body.IsAwakeForCollision.Should().BeTrue();
    }

    [Fact]
    public void Initialize_WithOffsetCollider_ShouldUseColliderCenterAsDefaultCenterOfMass()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var collider = new LSCircleCollider2D(Fixed64.One)
        {
            LocalOffset = new Vector2d((Fixed64)2, Fixed64.Half)
        };

        SolidBody2D body = CreateBody(context, collider, mass: Fixed64.One);

        body.LocalCenterOfMassOffset.Should().Be(collider.ScaledLocalOffset);
        body.WorldCenterOfMass.Should().Be(collider.ScaledLocalOffset);
    }

    [Fact]
    public void WorldCenterOfMass_ShouldRotateLocalOffsetAroundBodyPosition()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.One), mass: Fixed64.One);

        body.SetPosition(new Vector2d((Fixed64)3, (Fixed64)4));
        body.SetRotation(FixedMath.DegToRad((Fixed64)90));
        body.LocalCenterOfMassOffset = new Vector2d((Fixed64)2, Fixed64.Zero);

        body.WorldCenterOfMass.Should().Be(new Vector2d((Fixed64)3, (Fixed64)6));
    }

    [Fact]
    public void WorldCenterOfMass_WhenAbsolutePointIsUnrepresentable_ShouldExposeTryContract()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D body = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.One),
            mass: Fixed64.One);
        body.SetPosition(new Vector2d(Fixed64.MaxValue, Fixed64.Zero));
        body.LocalCenterOfMassOffset = Vector2d.Right;

        body.TryGetWorldCenterOfMass(out Vector2d center).Should().BeFalse();
        center.Should().Be(Vector2d.Zero);

        Func<Vector2d> readWorldCenter = () => body.WorldCenterOfMass;
        readWorldCenter.Should().Throw<InvalidOperationException>();
    }

    [Fact]
    public void WorldCenterOfMass_WhenRotatedOffsetIsUnrepresentable_ShouldExposeTryContract()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D body = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.One),
            mass: Fixed64.One);
        body.LocalCenterOfMassOffset = new Vector2d(
            Fixed64.MaxValue,
            Fixed64.MaxValue);
        body.SetRotation(Fixed64.PiOver4);

        body.TryGetWorldCenterOfMass(out Vector2d center).Should().BeFalse();
        center.Should().Be(Vector2d.Zero);
        body.TryGetOffsetFromCenterOfMass(
            new ContactAnchor2D(Vector2d.Zero, Vector2d.Zero),
            out Vector2d offset).Should().BeFalse();
        offset.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void CenterOfMassOperations_WhenRotationOverflowsButFinalValuesCancel_ShouldSucceed()
    {
        using GravitasWorldContext context =
            Physics2DTestWorld.CreateContext();
        Vector2d localOffset = new(
            Fixed64.MaxValue,
            Fixed64.MaxValue);
        Vector2d.TryRotate(
            localOffset,
            Fixed64.PiOver4,
            out _).Should().BeFalse();

        SolidBody2D worldBody = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.One),
            mass: Fixed64.One);
        worldBody.SetPosition(new Vector2d(
            Fixed64.Zero,
            -Fixed64.MaxValue));
        worldBody.LocalCenterOfMassOffset = localOffset;
        worldBody.SetRotation(Fixed64.PiOver4);
        worldBody.TryGetWorldCenterOfMass(
            out Vector2d worldCenter).Should().BeTrue();
        worldCenter.Y.Should().BeGreaterThan(Fixed64.Zero);
        worldCenter.Y.Should().BeLessThan(Fixed64.MaxValue);

        SolidBody2D relativeBody = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.One),
            mass: Fixed64.One);
        relativeBody.LocalCenterOfMassOffset = localOffset;
        relativeBody.SetRotation(Fixed64.PiOver4);
        relativeBody.TryGetOffsetFromCenterOfMass(
            new ContactAnchor2D(
                new Vector2d(
                    Fixed64.Zero,
                    Fixed64.MaxValue),
                new Vector2d(
                    Fixed64.Zero,
                    Fixed64.MaxValue)),
            out Vector2d relative).Should().BeTrue();
        relative.Y.Should().BeGreaterThan(Fixed64.Zero);
        relative.Y.Should().BeLessThan(Fixed64.MaxValue);
    }

    [Fact]
    public void TryGetOffsetFromCenterOfMass_WhenAbsolutePointsAreUnrepresentable_ShouldRetainExactLeverArm()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D body = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.One),
            mass: Fixed64.One);
        body.LocalCenterOfMassOffset = Vector2d.Right;
        var anchor = new ContactAnchor2D(
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            Vector2d.Right);

        body.TryGetOffsetFromCenterOfMass(anchor, out Vector2d offset).Should().BeTrue();
        offset.Should().Be(new Vector2d(Fixed64.MaxValue, Fixed64.Zero));
    }

    [Fact]
    public void ResetCenterOfMassFromCollider_ShouldReturnExplicitOverrideToDerivedColliderCenter()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var collider = new LSCircleCollider2D(Fixed64.One)
        {
            LocalOffset = new Vector2d(Fixed64.Half, Fixed64.Zero)
        };
        SolidBody2D body = CreateBody(context, collider, mass: Fixed64.One);
        body.LocalCenterOfMassOffset = new Vector2d(Fixed64.Zero, (Fixed64)3);

        body.ResetCenterOfMassFromCollider();

        body.LocalCenterOfMassOffset.Should().Be(collider.ScaledLocalOffset);
    }

    [Fact]
    public void LocalCenterOfMassOffset_ShouldNoOpForSameExplicitValueAndAllowInactiveSetup()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var inactive = new SolidBody2D(new TestMatterAgent(context), new LSCircleCollider2D(Fixed64.One));

        inactive.LocalCenterOfMassOffset = Vector2d.Right;
        inactive.LocalCenterOfMassOffset.Should().Be(Vector2d.Right);

        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.One), Fixed64.One);
        body.LocalCenterOfMassOffset = Vector2d.Forward;
        body.Sleep();

        body.LocalCenterOfMassOffset = Vector2d.Forward;

        body.IsSleeping.Should().BeTrue();
    }

    private static SolidBody2D CreateBody(
        GravitasWorldContext context,
        LSCollider2D collider,
        Fixed64 mass,
        bool positionFrozen = false,
        BodyMotionType motionType = BodyMotionType.Dynamic)
    {
        var body = new SolidBody2D(new TestMatterAgent(context), collider)
        {
            Mass = mass,
            FreezeAxes = positionFrozen ? BodyFreezeAxes2D.Position : BodyFreezeAxes2D.None
        };

        body.Initialize(Vector2d.Zero, Fixed64.Zero, motionType);
        return body;
    }
}
