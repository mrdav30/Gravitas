using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class SolidBodyMassMutationTests
{
    [Fact]
    public void ChangedMass_ShouldMatchFreshBodyAngularResponseAndPreserveAcceptedMotion()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        SolidBody body = scenario.CreateSphere(Vector3d.Zero).Body;
        SolidBody reference = scenario.CreateSphere(Vector3d.Right * 4, mass: (Fixed64)2).Body;
        body.AddLinearImpulse(Vector3d.Right);
        body.Mass = (Fixed64)2;

        body.LinearVelocity.Should().Be(Vector3d.Right);
        body.InverseMass.Should().Be(Fixed64.Half);
        body.InverseInertiaTensor.Should().Be(reference.InverseInertiaTensor);
        body.AddAngularImpulse(Vector3d.Up * Fixed64.FromFraction(1, 4));
        reference.AddAngularImpulse(Vector3d.Up * Fixed64.FromFraction(1, 4));
        body.AngularVelocity.Should().Be(reference.AngularVelocity);
    }

    [Theory]
    [InlineData(1, 0)]
    [InlineData(1, -1)]
    [InlineData(0, 2)]
    [InlineData(-1, 2)]
    public void ChangedMass_ShouldSynchronizeInertiaAndAwakeMembership(int initialMass, int mass)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var entry = scenario.CreateSphere(Vector3d.Zero, mass: (Fixed64)initialMass);
        int dynamicId = entry.Body.DynamicId;
        var coordinates = entry.Collider.PartitionCoordinates;
        entry.Body.Mass = (Fixed64)mass;

        bool movable = mass > 0;
        entry.Body.CanTranslate.Should().Be(movable);
        entry.Body.CanRotate.Should().Be(movable);
        entry.Body.InverseMass.Should().Be(movable ? Fixed64.Half : Fixed64.Zero);
        entry.Body.IsDynamic.Should().BeTrue();
        entry.Body.DynamicId.Should().Be(dynamicId);
        entry.Collider.PartitionCoordinates.Should().BeSameAs(coordinates);
        foreach (var coordinate in coordinates!)
        {
            scenario.Context.World.TryGetVoxel(coordinate, out var voxel).Should().BeTrue();
            voxel!.TryGetPartition(out PhysicsPartition? partition).Should().BeTrue();
            partition!.AwakeDynamicObjectCount.Should().Be(movable ? 1 : 0);
        }
        if (!movable)
        {
            entry.Body.InverseInertiaTensor.Should().Be(Fixed3x3.Zero);
            entry.Body.AddAngularImpulse(Vector3d.Up);
            entry.Body.AngularVelocity.Should().Be(Vector3d.Zero);
        }
    }

    [Fact]
    public void SameMass_ShouldPreserveSleepAndChangedMass_ShouldWake()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        SolidBody body = scenario.CreateSphere(Vector3d.Zero).Body;
        body.Sleep();
        Fixed64 mass = body.Mass;
        body.Mass = mass;
        body.IsSleeping.Should().BeTrue();
        body.Mass = (Fixed64)2;
        body.IsSleeping.Should().BeFalse();
        body.IsAwakeForCollision.Should().BeTrue();
    }

    [Theory]
    [InlineData(BodyMotionType.Static)]
    [InlineData(BodyMotionType.Kinematic)]
    public void ChangedMass_ShouldPreserveExplicitRole(BodyMotionType role)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        SolidBody body = scenario.CreateBody(new LSSphereCollider(), Vector3d.Zero,
            FixedQuaternion.Identity, isDynamic: role != BodyMotionType.Static,
            isKinematic: role == BodyMotionType.Kinematic).Body;
        int id = body.DynamicId;
        body.Mass = (Fixed64)2;
        body.MotionType.Should().Be(role);
        body.DynamicId.Should().Be(id);
        body.CanTranslate.Should().BeFalse();
        body.CanRotate.Should().BeFalse();
    }

    [Fact]
    public void MassConfiguration_ShouldRemainAvailableBeforeInitializationAndAfterDeactivation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var body = new SolidBody(new TestMatterAgent(scenario.Context), new LSSphereCollider());
        body.Mass = (Fixed64)2;
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
        body.InverseMass.Should().Be(Fixed64.Half);
        body.CanRotate.Should().BeTrue();
        body.Deactivate();
        body.Mass = (Fixed64)4;
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
        body.InverseMass.Should().Be(Fixed64.FromFraction(1, 4));
    }

    [Fact]
    public void RepeatedMassChanges_ShouldAllocateNothing()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        SolidBody body = scenario.CreateSphere(Vector3d.Zero).Body;
        AllocationTestHelper.MeasureSteadyState(() =>
        {
            body.Mass = Fixed64.Zero;
            body.Mass = (Fixed64)2;
        }).Should().Be(0);
    }

    [Fact]
    public void FailedCustomInertiaCalculation_ShouldPreserveMassTensorAndSleep()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var collider = new UnsupportedTestCollider3D { InertiaTensor = Fixed3x3.Identity };
        SolidBody body = scenario.CreateBody(collider, Vector3d.Zero, FixedQuaternion.Identity).Body;
        body.Sleep();
        Fixed3x3 tensor = body.InverseInertiaTensor;
        collider.ThrowOnInertiaCalculation = true;

        Action change = () => body.Mass = (Fixed64)2;
        change.Should().Throw<InvalidOperationException>();

        body.Mass.Should().Be(Fixed64.One);
        body.InverseInertiaTensor.Should().Be(tensor);
        body.IsSleeping.Should().BeTrue();
        foreach (var coordinate in collider.PartitionCoordinates!)
        {
            scenario.Context.World.TryGetVoxel(coordinate, out var voxel).Should().BeTrue();
            voxel!.TryGetPartition(out PhysicsPartition? partition).Should().BeTrue();
            partition!.AwakeDynamicObjectCount.Should().Be(0);
        }
    }

    [Theory]
    [InlineData(BodyFreezeAxes3D.Position)]
    [InlineData(BodyFreezeAxes3D.Rotation)]
    [InlineData(BodyFreezeAxes3D.All)]
    public void ChangedMass_ShouldPreserveFrozenAxes(BodyFreezeAxes3D axes)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        SolidBody body = scenario.CreateSphere(Vector3d.Zero).Body;
        body.FreezeAxes = axes;
        body.Mass = (Fixed64)2;
        body.FreezeAxes.Should().Be(axes);
        body.CanTranslate.Should().Be((axes & BodyFreezeAxes3D.Position) == 0);
        body.CanRotate.Should().Be((axes & BodyFreezeAxes3D.Rotation) == 0);
    }

    [Fact]
    public void ChangedMass_ShouldPreserveQueuedLinearAndAngularAccelerationAcrossZero()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.SetFrameRate(4);
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        // An explicit unit tensor keeps the expected accepted angular acceleration exact.
        var collider = new UnsupportedTestCollider3D { InertiaTensor = Fixed3x3.Identity };
        SolidBody body = scenario.CreateBody(collider, Vector3d.Zero,
            FixedQuaternion.Identity, mass: Fixed64.Two).Body;
        body.UseManualGrounding();
        body.AddLinearImpulse(Vector3d.Right * Fixed64.Two);
        body.AddAngularImpulse(Vector3d.Up * Fixed64.Two);
        body.AddForce(Vector3d.Right * (Fixed64)4);
        body.AddTorque(Vector3d.Up * (Fixed64)3);
        body.Mass = Fixed64.Zero;
        body.HasSolverMobility.Should().BeFalse();
        body.LinearVelocity.Should().Be(Vector3d.Right);
        body.AngularVelocity.Should().Be(Vector3d.Up * Fixed64.Two);
        body.Mass = (Fixed64)4;

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        body.LinearVelocity.Should().Be(Vector3d.Right * Fixed64.FromFraction(3, 2));
        body.AngularVelocity.Should().Be(Vector3d.Up * Fixed64.FromFraction(11, 4));
    }

    [Fact]
    public void RuntimeMassTrace_ShouldReplayIdentically()
    {
        using PhysicsScenarioBuilder first = PhysicsScenarioBuilder.Create();
        using PhysicsScenarioBuilder second = PhysicsScenarioBuilder.Create();
        SolidBody firstBody = first.CreateSphere(Vector3d.Zero).Body;
        SolidBody secondBody = second.CreateSphere(Vector3d.Zero).Body;
        foreach (int mass in new[] { 2, 0, -1, 4, 4, 1 })
        {
            firstBody.Mass = (Fixed64)mass;
            secondBody.Mass = (Fixed64)mass;
            firstBody.AddLinearImpulse(Vector3d.Right);
            secondBody.AddLinearImpulse(Vector3d.Right);
            firstBody.AddAngularImpulse(Vector3d.Up);
            secondBody.AddAngularImpulse(Vector3d.Up);
            first.Context.Simulate();
            first.Context.LateSimulate();
            second.Context.Simulate();
            second.Context.LateSimulate();
            second.Context.ComputeReplayHash().Should().Be(first.Context.ComputeReplayHash());
        }
    }
}
