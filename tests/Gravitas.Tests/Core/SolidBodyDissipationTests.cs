using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class SolidBodyDissipationTests
{
    [Theory]
    [InlineData("linear", PhysicsRuntimeMode.ThreeD)]
    [InlineData("angular", PhysicsRuntimeMode.ThreeD)]
    [InlineData("damping", PhysicsRuntimeMode.ThreeD)]
    [InlineData("linear", PhysicsRuntimeMode.Mixed)]
    [InlineData("angular", PhysicsRuntimeMode.Mixed)]
    [InlineData("damping", PhysicsRuntimeMode.Mixed)]
    public void StrongDissipation_ShouldStopWithoutReversalOrEnergyGain(string term, PhysicsRuntimeMode mode)
    {
        using var scene = CreateScene(mode, term == "damping" ? 64 : 4);
        Fixed64 size = term == "damping" ? Fixed64.FromFraction(1, 8) : Fixed64.One;
        SolidBody body = scene.CreateBody(new LSCuboidCollider { Size = Vector3d.One * size },
            Vector3d.Zero, FixedQuaternion.Identity).Body;
        ConfigureTerm(scene.Context, body, term, strong: true);

        foreach (Fixed64 sign in new[] { Fixed64.One, -Fixed64.One })
        {
            body.ResetPosition(Vector3d.Zero, FixedQuaternion.Identity);
            Vector3d velocity = Vector3d.Up * (Fixed64.FromFraction(3, 8) * sign);
            if (term == "linear") body.ApplyCollisionLinearVelocityDelta(velocity);
            else body.ApplyCollisionAngularVelocityDelta(velocity);
            scene.Context.Simulate();
            scene.Context.LateSimulate();

            body.LinearVelocity.Should().Be(Vector3d.Zero);
            body.AngularVelocity.Should().Be(Vector3d.Zero);
        }
    }

    [Theory]
    [InlineData("linear", -1)]
    [InlineData("linear", 0)]
    [InlineData("linear", 1)]
    [InlineData("angular", -1)]
    [InlineData("angular", 0)]
    [InlineData("angular", 1)]
    [InlineData("damping", -1)]
    [InlineData("damping", 0)]
    [InlineData("damping", 1)]
    public void Dissipation_ShouldOpposeTheDrivenMotionIncludingStartStopAndReversal(string term, int drivenSign)
    {
        using var scene = CreateScene(PhysicsRuntimeMode.ThreeD, 4);
        SolidBody body = scene.CreateCuboid(Vector3d.Zero).Body;
        ConfigureTerm(scene.Context, body, term, strong: false);
        Vector3d initial = drivenSign == 1 ? Vector3d.Zero : Vector3d.Right;
        Vector3d driven = Vector3d.Right * (Fixed64)drivenSign;
        if (term == "linear")
        {
            body.ApplyCollisionLinearVelocityDelta(initial);
            body.AddForce((driven - initial) * (Fixed64)4);
        }
        else
        {
            body.ApplyCollisionAngularVelocityDelta(initial);
            Fixed3x3 inertia = body.Collider.CalculateInertiaTensor(body.Mass, body.LocalCenterOfMassOffset);
            body.AddTorque(inertia * ((driven - initial) * (Fixed64)4));
        }
        scene.Context.Simulate();
        scene.Context.LateSimulate();

        Vector3d after = term == "linear" ? body.LinearVelocity : body.AngularVelocity;
        if (drivenSign == 0) after.Should().Be(Vector3d.Zero);
        else
        {
            (after.X * (Fixed64)drivenSign).Should().BeGreaterThan(Fixed64.Zero);
            after.Magnitude.Should().BeLessThan(Fixed64.One);
            after.Y.Should().Be(Fixed64.Zero);
            after.Z.Should().Be(Fixed64.Zero);
        }
    }

    [Theory]
    [InlineData(10, 1)]
    [InlineData(100, 100)]
    [InlineData(1000, 1000)]
    public void Damping_WithRotatedAnisotropicFrozenAxes_ShouldReducePhysicalEnergy(int length, int thicknessDenominator)
    {
        using var scene = CreateScene(PhysicsRuntimeMode.ThreeD, 64);
        scene.Context.Environment.DampingFactor = (Fixed64)64;
        Vector3d axis = new Vector3d(1, 2, 1).Normalized;
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Cross(Vector3d.Right, axis).Normalized, FixedMath.Acos(axis.X));
        var collider = new LSCuboidCollider
        {
            Size = new Vector3d((Fixed64)length, Fixed64.FromFraction(1, thicknessDenominator), Fixed64.FromFraction(1, thicknessDenominator))
        };
        SolidBody body = scene.CreateBody(collider, Vector3d.Zero, rotation).Body;
        body.FreezeAxes = BodyFreezeAxes3D.RotationZ;
        body.ApplyCollisionAngularVelocityDelta(Vector3d.Right);
        Fixed3x3 inertia = rotation.Conjugate().ToMatrix3x3()
            * collider.CalculateInertiaTensor(body.Mass, body.LocalCenterOfMassOffset)
            * rotation.ToMatrix3x3();
        Fixed64 before = Vector3d.Dot(body.AngularVelocity, inertia * body.AngularVelocity);

        // Inspect the actual integration used to build CCD motion, before separate gyroscopic integration.
        body.EnsureContinuousCollisionFramePrepared(scene.Context.LateSimulateToken).Should().BeTrue();

        Vector3d.Dot(body.AngularVelocity, inertia * body.AngularVelocity).Should().BeLessThan(before);
        body.AngularVelocity.Z.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AngularDrag_ShouldApplyRadialDecayRegardlessOfAnisotropicInertia(bool freezeVertical)
    {
        using var scene = CreateScene(PhysicsRuntimeMode.ThreeD, 64);
        scene.Context.Environment.AirDensity = Fixed64.One;
        var collider = new LSCuboidCollider { Size = new Vector3d(1, 2, 3) };
        SolidBody body = scene.CreateBody(collider, Vector3d.Zero,
            FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)15, (Fixed64)30, (Fixed64)45)).Body;
        body.FreezeAxes = freezeVertical ? BodyFreezeAxes3D.RotationY : BodyFreezeAxes3D.None;
        body.ApplyCollisionAngularVelocityDelta(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Half));
        Vector3d before = body.AngularVelocity;
        Fixed64 fraction = (Fixed64)0.75m * collider.GetFrontalArea(before.Normalized) * scene.Context.DeltaTime;

        body.EnsureContinuousCollisionFramePrepared(scene.Context.LateSimulateToken).Should().BeTrue();

        body.AngularVelocity.Should().Be(before * (Fixed64.One - fraction));
        body.AngularVelocity.Y.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void ZeroLinearDragCoefficient_ShouldPreserveDrivenMotionDespiteDenseAir()
    {
        using var scene = CreateScene(PhysicsRuntimeMode.ThreeD, 4);
        scene.Context.Environment.AirDensity = (Fixed64)1024;
        SolidBody body = scene.CreateCuboid(Vector3d.Zero).Body;
        body.LinearDragCoefficient = Fixed64.Zero;
        body.AddForce(Vector3d.Right * (Fixed64)4);
        scene.Context.Simulate();
        scene.Context.LateSimulate();
        body.LinearVelocity.Should().Be(Vector3d.Right);
        body.Position3d.X.Should().Be(Fixed64.FromFraction(1, 4));
    }

    [Fact]
    public void LinearDrag_AfterDriveChangesDirection_ShouldUseTheNewFrontalArea()
    {
        using var scene = CreateScene(PhysicsRuntimeMode.ThreeD, 64);
        scene.Context.Environment.AirDensity = Fixed64.One;
        var collider = new LSCuboidCollider { Size = new Vector3d(1, 2, 3) };
        SolidBody body = scene.CreateBody(collider, Vector3d.Zero, FixedQuaternion.Identity).Body;
        body.LinearDragCoefficient = Fixed64.One;
        body.AddLinearImpulse(Vector3d.Right);
        body.AddForce((Vector3d.Forward - Vector3d.Right) * (Fixed64)64);
        Vector3d expected = Vector3d.Forward
            * (Fixed64.One - collider.GetFrontalArea(Vector3d.Forward) * scene.Context.DeltaTime);
        scene.Context.Simulate();
        scene.Context.LateSimulate();
        body.LinearVelocity.Should().Be(expected);
        body.Position3d.Should().Be(expected * scene.Context.DeltaTime);
    }

    [Fact]
    public void PreparedMotion_ShouldApplyDissipationOnlyOnceAndPublishTheSameCcdTrajectory()
    {
        using var scene = CreateScene(PhysicsRuntimeMode.Mixed, 64);
        scene.Context.Environment.AirDensity = Fixed64.One;
        scene.Context.Environment.DampingFactor = Fixed64.Half;
        SolidBody body = scene.CreateCuboid(Vector3d.Zero).Body;
        body.ApplyCollisionAngularVelocityDelta(Vector3d.Up);
        body.ApplyCollisionLinearVelocityDelta(Vector3d.Right);
        scene.Context.Physics.BeginLateSimulateBodies(continuousCollisionFramePrepared: false).Should().BeTrue();
        long token = scene.Context.LateSimulateToken;
        Vector3d angular = body.AngularVelocity;
        Vector3d linear = body.LinearVelocity;
        var segment = body.GetContinuousCollisionTrajectorySegment(0);
        segment.AngularVelocity.Should().Be(angular);
        segment.Displacement.Should().Be(linear * scene.Context.DeltaTime);
        body.EnsureContinuousCollisionFramePrepared(token).Should().BeTrue();
        body.ApplyContinuousCollisionHandoff(body.Position3d, Vector3d.Zero,
            scene.Context.DeltaTime * Fixed64.Half).Should().BeTrue();
        scene.Context.Physics.ProcessQueuedContinuousCollisionHandoffs(iterationBudget: 1).Should().Be(1);
        body.AngularVelocity.Should().Be(angular);
        body.LinearVelocity.Should().Be(linear);
    }

    [Fact]
    public void CombinedDragAndConstrainedDamping_ShouldNotAllocateAfterWarmup()
    {
        using var scene = CreateScene(PhysicsRuntimeMode.Mixed, 64);
        scene.Context.Environment.AirDensity = Fixed64.One;
        scene.Context.Environment.DampingFactor = Fixed64.Half;
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)15, (Fixed64)30, (Fixed64)45);
        SolidBody body = scene.CreateBody(new LSCuboidCollider { Size = new Vector3d(1, 2, 3) },
            Vector3d.Zero, rotation).Body;
        body.FreezeAxes = BodyFreezeAxes3D.RotationZ;
        body.SleepEnabled = false;
        long allocated = AllocationTestHelper.MeasureSteadyState(() =>
        {
            body.ResetPosition(Vector3d.Zero, rotation);
            body.AddLinearImpulse(Vector3d.Right);
            body.AddAngularImpulse(Vector3d.Right);
            scene.Context.Simulate();
            scene.Context.LateSimulate();
        }, measurementIterations: 128);
        allocated.Should().Be(0);
        body.AngularSpeed.Should().BeGreaterThan(Fixed64.Zero);
        body.AngularVelocity.Z.Should().Be(Fixed64.Zero);
    }

    private static PhysicsScenarioBuilder CreateScene(PhysicsRuntimeMode mode, int rate)
    {
        var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = mode;
        scene.Context.SetFrameRate(rate);
        scene.Context.Environment.Gravity = Fixed64.Zero;
        scene.Context.Environment.AirDensity = Fixed64.Zero;
        scene.Context.Environment.DampingFactor = Fixed64.Zero;
        scene.Context.Environment.MinSpeed = Fixed64.Zero;
        scene.Context.Environment.MaxSpeed = (Fixed64)100;
        return scene;
    }

    private static void ConfigureTerm(GravitasWorldContext context, SolidBody body, string term, bool strong)
    {
        body.LinearDragCoefficient = Fixed64.One;
        if (term == "damping") context.Environment.DampingFactor = strong ? PhysicsEnvironment.DefaultDampingFactor : Fixed64.FromFraction(1, 4);
        else context.Environment.AirDensity = strong ? (Fixed64)64 : Fixed64.One;
    }
}
