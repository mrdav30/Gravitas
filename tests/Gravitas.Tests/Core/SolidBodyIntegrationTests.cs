using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class SolidBodyIntegrationTests
{
    private static readonly Fixed64 Tolerance = Fixed64.FromFraction(1, 1_000_000);

    public static TheoryData<string, FixedQuaternion> RotationAdmissionCases => new()
    {
        { "zero", FixedQuaternion.Zero },
        { "scaled", new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, (Fixed64)2) },
        { "saturated", new FixedQuaternion(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MaxValue) },
        { "near-unit", new FixedQuaternion(Fixed64.Epsilon, Fixed64.Zero, Fixed64.Zero, Fixed64.One) }
    };

    [Theory]
    [MemberData(nameof(RotationAdmissionCases))]
    public void PublicRotationAdmission_ShouldPublishOneNormalizedOrientation(
        string _,
        FixedQuaternion admittedRotation)
    {
        FixedQuaternion expected = admittedRotation.Normalized;
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero, admittedRotation);

        body.Body.Rotation.Should().Be(expected);
        body.Collider.Rotation.Should().Be(expected);
        body.Body.RotationTransform.WorldRotation.Should().Be(expected);

        body.Body.SetRotation(admittedRotation);
        body.Body.Rotation.Should().Be(expected);

        body.Body.UpdateRotation(admittedRotation, Fixed64.Zero);
        body.Body.Rotation.Should().Be(expected);

        body.Body.ResetPosition(Vector3d.Zero, admittedRotation);
        body.Body.Rotation.Should().Be(expected);
        body.Body.VisualRotation.Should().Be(expected);
        body.Body.RotationTransform.WorldRotation.Should().Be(expected);

        body.Body.SetVisualRotation(admittedRotation);
        body.Body.VisualRotation.Should().Be(expected);
    }

    [Fact]
    public void LateSimulate_ShouldIntegrateForceVelocityAndPosition()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero, mass: (Fixed64)2);

        body.Body.AddForce(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        scenario.Context.LateSimulate();

        body.Body.LinearVelocity.Should().Be(new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero));
        body.Body.LinearAcceleration.Should().Be(Vector3d.Right * (Fixed64)4);
        body.Body.Position3d.Should().Be(new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero));
        body.Body.IsAtRest.Should().BeFalse();
    }

    [Fact]
    public void LateSimulate_ShouldApplyLinearDragWithFixedExpectedValue()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        scenario.Context.Environment.AirDensity = Fixed64.One;
        ScenarioBody<LSCuboidCollider> body = scenario.CreateCuboid(Vector3d.Zero);
        body.Body.LinearDragCoefficient = Fixed64.One;

        body.Body.AddForce(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));
        scenario.Context.LateSimulate();
        scenario.Context.LateSimulate();

        body.Body.LinearVelocity.X.Should().Be(Fixed64.FromFraction(3, 4));
        body.Body.Position3d.X.Should().Be(Fixed64.FromFraction(7, 16));
    }

    [Fact]
    public void LateSimulate_ShouldApplyGroundFrictionWithFixedExpectedValue()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        scenario.Context.Environment.Gravity = Fixed64.One;
        CreateGround(scenario);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        body.Body.AddForce(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));
        scenario.Context.LateSimulate();
        scenario.Context.LateSimulate();

        body.Body.IsGrounded.Should().BeTrue();
        body.Body.LinearVelocity.X.Should().Be(Fixed64.FromFraction(3, 4));
        body.Body.Position3d.X.Should().Be(Fixed64.FromFraction(7, 16));
    }

    [Fact]
    public void GravityScale_WhenChangedWhileGrounded_ShouldRefreshGroundFrictionWeight()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        scenario.Context.Environment.Gravity = Fixed64.One;
        CreateGround(scenario);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        body.Body.GravityScale = Fixed64.Zero;
        body.Body.AddForce(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));
        scenario.Context.LateSimulate();
        scenario.Context.LateSimulate();

        body.Body.IsGrounded.Should().BeTrue();
        body.Body.LinearVelocity.X.Should().Be(Fixed64.One);
        body.Body.Position3d.X.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void LateSimulate_WithDefaultGravityScale_ShouldApplyEnvironmentGravity()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        scenario.Context.Environment.Gravity = (Fixed64)4;
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        scenario.Context.LateSimulate();

        body.Body.LinearVelocity.Y.Should().Be(-Fixed64.One);
        body.Body.Position3d.Y.Should().Be(-Fixed64.FromFraction(1, 4));
    }

    [Fact]
    public void LateSimulate_WithHalfGravityScale_ShouldApplyScaledEnvironmentGravity()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        scenario.Context.Environment.Gravity = (Fixed64)4;
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.GravityScale = Fixed64.Half;

        scenario.Context.LateSimulate();

        body.Body.LinearVelocity.Y.Should().Be(-Fixed64.Half);
        body.Body.Position3d.Y.Should().Be(-Fixed64.FromFraction(1, 8));
    }

    [Fact]
    public void LateSimulate_WithZeroGravityScale_ShouldIgnoreEnvironmentGravity()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        scenario.Context.Environment.Gravity = (Fixed64)4;
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.GravityScale = Fixed64.Zero;

        scenario.Context.LateSimulate();

        body.Body.LinearVelocity.Y.Should().Be(Fixed64.Zero);
        body.Body.Position3d.Y.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void GravityScale_ShouldRejectNegativeValues()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        Action action = () => body.Body.GravityScale = -Fixed64.Epsilon;

        action.Should().Throw<ArgumentException>();
    }

    [Fact]
    public void ContinuousCollisionPrediction_WithGravityScale_ShouldUseScaledEnvironmentGravity()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        scenario.Context.Environment.Gravity = (Fixed64)4;
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.GravityScale = Fixed64.Half;

        body.Body.EnsureContinuousCollisionFramePrepared(123);

        body.Body.ContinuousCollisionFrameDisplacement.Should().Be(new Vector3d(
            Fixed64.Zero,
            -Fixed64.FromFraction(1, 8),
            Fixed64.Zero));
    }

    [Fact]
    public void LateSimulate_ShouldIntegrateTorqueAndAngularDamping()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        ScenarioBody<LSCuboidCollider> body = scenario.CreateCuboid(Vector3d.Zero);

        body.Body.AddTorque(Vector3d.Up);
        scenario.Context.LateSimulate();

        AssertNear(body.Body.AngularVelocity.Y, Fixed64.FromFraction(3, 2));
        AssertNear(body.Body.AngularSpeed, Fixed64.FromFraction(3, 2));
        AssertNear(body.Body.AngularAcceleration, Vector3d.Up * (Fixed64)6);

        scenario.Context.Environment.DampingFactor = Fixed64.Half;
        scenario.Context.LateSimulate();

        AssertNear(body.Body.AngularVelocity.Y, Fixed64.FromFraction(3, 8));
    }

    [Fact]
    public void ResetPosition_ShouldReturnMovingBodyToRest()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        body.Body.AddForce(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));
        scenario.Context.LateSimulate();

        body.Body.IsAtRest.Should().BeFalse();

        body.Body.ResetPosition(Vector3d.Zero, FixedQuaternion.Identity);

        body.Body.IsAtRest.Should().BeTrue();
        body.Body.Position3d.Should().Be(Vector3d.Zero);
        body.Body.Rotation.Should().Be(FixedQuaternion.Identity);
    }

    [Fact]
    public void PointConversion_WithPrimitiveDimensions_ShouldUseCommittedOwnerScale()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        var collider = new LSCuboidCollider
        {
            Size = new Vector3d((Fixed64)2, (Fixed64)4, (Fixed64)6)
        };
        Vector3d position = new((Fixed64)10, (Fixed64)2, (Fixed64)(-3));
        FixedQuaternion rotation = PhysicsScenarioBuilder.Yaw(90);
        Vector3d hostScale = new((Fixed64)3, (Fixed64)2, (Fixed64)4);
        ScenarioBody<LSCuboidCollider> body = scenario.CreateBody(
            collider,
            position,
            rotation);
        body.Body.Agent.Transform.LocalScale = hostScale;
        body.Collider.Simulate();
        Vector3d localPoint = new(Fixed64.Half, Fixed64.One, -Fixed64.Half);
        Vector3d expectedWorldPoint = position + rotation * Vector3d.Multiply(hostScale, localPoint);

        body.Body.TryGetWorldPoint(localPoint, out Vector3d attemptedWorldPoint).Should().BeTrue();
        Vector3d worldPoint = body.Body.GetWorldPoint(localPoint);
        body.Body.TryGetLocalPoint(worldPoint, out Vector3d attemptedLocalPoint).Should().BeTrue();
        Vector3d roundTripped = body.Body.GetLocalPoint(worldPoint);

        attemptedWorldPoint.Should().Be(expectedWorldPoint);
        worldPoint.Should().Be(expectedWorldPoint);
        AssertNear(attemptedLocalPoint, localPoint);
        AssertNear(roundTripped, localPoint);
    }

    [Fact]
    public void PointTransforms_WhenIntermediateProductsExceedTheDomain_ShouldMaterializeRepresentableResults()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        Vector3d position = new((Fixed64)(-2_000_000_000), Fixed64.Zero, Fixed64.Zero);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(position);
        body.Body.Agent.Transform.LocalScale = new Vector3d((Fixed64)3, Fixed64.One, Fixed64.One);
        body.Collider.Simulate();
        Vector3d localPoint = new((Fixed64)1_000_000_000, Fixed64.Zero, Fixed64.Zero);
        Vector3d expectedWorldPoint = new((Fixed64)1_000_000_000, Fixed64.Zero, Fixed64.Zero);

        body.Body.TryGetWorldPoint(localPoint, out Vector3d worldPoint).Should().BeTrue();
        body.Body.TryGetLocalPoint(expectedWorldPoint, out Vector3d roundTripped).Should().BeTrue();

        worldPoint.Should().Be(expectedWorldPoint);
        roundTripped.Should().Be(localPoint);
        body.Body.GetWorldPoint(localPoint).Should().Be(expectedWorldPoint);
        body.Body.GetLocalPoint(expectedWorldPoint).Should().Be(localPoint);
    }

    [Fact]
    public void PointConversion_WithAnisotropicScale_ShouldRoundTrip()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        FixedQuaternion rotation = PhysicsScenarioBuilder.Yaw(90);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(
            new Vector3d((Fixed64)7, (Fixed64)(-3), (Fixed64)11),
            rotation);
        body.Body.Agent.Transform.LocalScale = new Vector3d((Fixed64)3, Fixed64.Half, (Fixed64)4);
        body.Collider.Simulate();
        Vector3d localPoint = new(Fixed64.Half, (Fixed64)(-2), (Fixed64)3);

        body.Body.TryGetWorldPoint(localPoint, out Vector3d worldPoint).Should().BeTrue();
        body.Body.TryGetLocalPoint(worldPoint, out Vector3d roundTripped).Should().BeTrue();

        worldPoint.Should().Be(body.Body.GetWorldPoint(localPoint));
        AssertNear(roundTripped, localPoint);
    }

    [Fact]
    public void PointTransforms_WhenFinalCoordinateIsUnrepresentable_ShouldFailAtomically()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(
            new Vector3d((Fixed64)2_000_000_000, Fixed64.Zero, Fixed64.Zero));
        Vector3d localPoint = new((Fixed64)1_000_000_000, Fixed64.Zero, Fixed64.Zero);

        body.Body.TryGetWorldPoint(localPoint, out Vector3d worldPoint).Should().BeFalse();
        body.Body.TryGetLocalPoint(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            out Vector3d inversePoint).Should().BeFalse();

        worldPoint.Should().Be(Vector3d.Zero);
        inversePoint.Should().Be(Vector3d.Zero);
        ((Action)(() => body.Body.GetWorldPoint(localPoint))).Should().Throw<InvalidOperationException>();
        ((Action)(() => body.Body.GetLocalPoint(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero))))
            .Should().Throw<InvalidOperationException>();
    }

    [Fact]
    public void PointConversion_WhenHostSnapshotChanges_ShouldRetainCommittedSimulationScale()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        Vector3d localPoint = new(Fixed64.One, Fixed64.Two, (Fixed64)3);
        var parent = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.MaxValue, Fixed64.One, Fixed64.One));
        body.Body.Agent.Transform.LocalScale = new Vector3d(Fixed64.Two, Fixed64.One, Fixed64.One);
        body.Body.Agent.Transform.SetParentKeepingLocal(parent);

        body.Body.TryGetWorldPoint(localPoint, out Vector3d worldPoint).Should().BeTrue();
        body.Body.TryGetLocalPoint(worldPoint, out Vector3d roundTrip).Should().BeTrue();
        body.Body.Agent.Transform.TryTransformPoint(localPoint, out _).Should().BeFalse();

        worldPoint.Should().Be(localPoint);
        roundTrip.Should().Be(localPoint);
    }

    [Fact]
    public void PointConversion_WithCompoundBounds_ShouldUseCommittedOwnerScale()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        var collider = new LSCompoundCollider(
            CompoundColliderPart.Cuboid(
                new Vector3d((Fixed64)8, (Fixed64)2, (Fixed64)4),
                new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero)),
            CompoundColliderPart.Sphere(
                Fixed64.Half,
                new Vector3d((Fixed64)(-3), Fixed64.One, Fixed64.Zero)));
        Vector3d position = new((Fixed64)(-4), (Fixed64)3, (Fixed64)7);
        FixedQuaternion rotation = PhysicsScenarioBuilder.Yaw(-90);
        Vector3d hostScale = new(Fixed64.Half, (Fixed64)3, (Fixed64)2);
        ScenarioBody<LSCompoundCollider> body = scenario.CreateBody(collider, position, rotation);
        body.Body.Agent.Transform.LocalScale = hostScale;
        body.Collider.Simulate();
        Vector3d localPoint = new((Fixed64)2, -Fixed64.Half, Fixed64.One);
        Vector3d expectedWorldPoint = position + rotation * Vector3d.Multiply(hostScale, localPoint);

        Vector3d worldPoint = body.Body.GetWorldPoint(localPoint);

        worldPoint.Should().Be(expectedWorldPoint);
        AssertNear(body.Body.GetLocalPoint(worldPoint), localPoint);
    }

    [Fact]
    public void PointConversion_BeforeShapeCommit_ShouldFailAtomically()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        var body = new SolidBody(
            new TestMatterAgent(scenario.Context),
            new LSSphereCollider());

        body.TryGetWorldPoint(Vector3d.One, out Vector3d worldPoint).Should().BeFalse();
        body.TryGetLocalPoint(Vector3d.One, out Vector3d localPoint).Should().BeFalse();
        Action getWorldPoint = () => body.GetWorldPoint(Vector3d.One);
        Action getLocalPoint = () => body.GetLocalPoint(Vector3d.One);

        worldPoint.Should().Be(Vector3d.Zero);
        localPoint.Should().Be(Vector3d.Zero);
        getWorldPoint.Should().Throw<InvalidOperationException>();
        getLocalPoint.Should().Throw<InvalidOperationException>();
    }

    [Fact]
    public void PointTransforms_AfterWarmup_ShouldNotAllocate()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(
            new Vector3d((Fixed64)5, (Fixed64)(-2), (Fixed64)9),
            PhysicsScenarioBuilder.Yaw(30));
        body.Body.Agent.Transform.LocalScale = new Vector3d((Fixed64)3, Fixed64.Half, Fixed64.Two);
        body.Collider.Simulate();
        Vector3d localPoint = new(Fixed64.Half, -Fixed64.One, Fixed64.Two);
        Vector3d worldPoint = body.Body.GetWorldPoint(localPoint);

        void TransformRoundTrip()
        {
            body.Body.TryGetWorldPoint(localPoint, out _);
            body.Body.TryGetLocalPoint(worldPoint, out _);
        }

        AllocationTestHelper.MeasureSteadyState(TransformRoundTrip).Should().Be(0);
    }

    [Fact]
    public void SetHeightAndFreezeAxes_ShouldExposeIndependentLifecycleStateWithoutWakingOnNoOp()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.Sleep();

        body.Body.SetHeight(body.Body.HeightPos);

        body.Body.IsSleeping.Should().BeTrue();
        body.Body.IsRotationFullyFrozen.Should().BeFalse();

        body.Body.SetHeight(Fixed64.One);

        body.Body.HeightPos.Should().Be(Fixed64.One);
        body.Body.IsSleeping.Should().BeFalse();

        body.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        body.Body.IsRotationFullyFrozen.Should().BeTrue();

        body.Body.FreezeAxes = BodyFreezeAxes3D.None;
        body.Body.IsRotationFullyFrozen.Should().BeFalse();

        body.Body.FreezeAxes = BodyFreezeAxes3D.Position;
        body.Body.IsRotationFullyFrozen.Should().BeFalse();
    }

    [Fact]
    public void Visualize_WithResetAccumulation_ShouldPublishAuthoritativeRotationTarget()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.CanSetVisualRotation = true;
        body.Body.DefaultRotationSpeed = Fixed64.One;
        FixedQuaternion target = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)90,
            Fixed64.Zero);

        body.Body.UpdateRotation(target, Fixed64.Zero);
        body.Body.Forward.Should().Be(target.Rotate(Vector3d.Forward));
        body.Body.Up.Should().Be(target.Rotate(Vector3d.Up));
        scenario.Context.LateSimulate();
        scenario.Context.Visualize();

        FixedQuaternion.Angle(body.Body.RotationTransform.LocalRotation, target).Should().BeLessThan(Tolerance);
        body.Body.LastVisualRotation.Should().Be(FixedQuaternion.Identity);
        body.Body.VisualRotation.Should().Be(target);
    }

    [Fact]
    public void SetVisualPosition_ShouldPreservePreviousAndCurrentInterpolationEndpoints()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        body.Body.SetVisualPosition(Vector3d.Right);
        body.Body.SetVisualPosition(Vector3d.Up);

        body.Body.LastVisualPosition.Should().Be(Vector3d.Right);
        body.Body.VisualPosition.Should().Be(Vector3d.Up);
    }

    [Fact]
    public void Visualize_WithSpeedLimitedRotation_ShouldAdvanceAcrossSteadyFrames()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.CanSetVisualRotation = true;
        body.Body.DefaultRotationSpeed = Fixed64.One;
        FixedQuaternion target = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)90,
            Fixed64.Zero);

        body.Body.UpdateRotation(target, Fixed64.One);
        scenario.Context.LateSimulate();
        scenario.Context.Visualize();
        FixedQuaternion firstVisual = body.Body.RotationTransform.LocalRotation;
        scenario.Context.Visualize();
        FixedQuaternion secondVisual = body.Body.RotationTransform.LocalRotation;

        firstVisual.Should().NotBe(FixedQuaternion.Identity);
        firstVisual.Should().NotBe(target);
        secondVisual.Should().NotBe(firstVisual);
        FixedQuaternion.Angle(secondVisual, target).Should().BeLessThan(FixedQuaternion.Angle(firstVisual, target));
    }

    [Fact]
    public void InteractingRotation_ShouldPublishBufferedStateAtInteractionSpeed()
    {
        using PhysicsScenarioBuilder scenario = CreateIntegrationScenario(frameRate: 4);
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(
            scenario.Context,
            transform,
            isParent: true,
            isInteracting: true);
        var body = new SolidBody(agent, new LSSphereCollider())
        {
            CanSetVisualRotation = true,
            DefaultRotationSpeed = (Fixed64)4,
            InteractionRotationSpeed = Fixed64.One,
            Mass = Fixed64.One
        };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
        FixedQuaternion target = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)90,
            Fixed64.Zero);

        body.UpdateRotation(target, Fixed64.One);
        body.RotationChangePending.Should().BeTrue();
        body.CheckChangedValues();
        body.RotationChangePending.Should().BeTrue();
        body.CheckChangedValues();
        body.RotationChangePending.Should().BeFalse();
        scenario.Context.LateSimulate();
        scenario.Context.Visualize();

        FixedQuaternion expected = FixedQuaternion.Slerp(
            FixedQuaternion.Identity,
            target,
            scenario.Context.DeltaTime * body.InteractionRotationSpeed);
        FixedQuaternion.Angle(transform.WorldRotation, expected).Should().BeLessThan(Tolerance);
        transform.WorldRotation.Should().NotBe(FixedQuaternion.Identity);
        transform.WorldRotation.Should().NotBe(target);
    }

    private static PhysicsScenarioBuilder CreateIntegrationScenario(int frameRate)
    {
        PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var settings = new PhysicsSettings(frameRate, null, PhysicsLayerMask.FromLayer(1));

        scenario.Context.ApplySettings(settings);

        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        scenario.Context.Environment.MaxSpeed = (Fixed64)100;
        scenario.Context.Environment.MaxFallSpeed = (Fixed64)100;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;

        return scenario;
    }

    private static void CreateGround(PhysicsScenarioBuilder scenario)
    {
        var transform = new FixedTransform(
            new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);
        var agent = new TestMatterAgent(scenario.Context, transform);
        var collider = new LSCuboidCollider
        {
            Layer = new PhysicsLayer(1),
            Size = new Vector3d((Fixed64)8, Fixed64.One, (Fixed64)8)
        };

        collider.InitializeWithNoBody(agent);
    }

    private static void AssertNear(Fixed64 actual, Fixed64 expected)
    {
        (actual - expected).Abs().Should().BeLessThan(Tolerance);
    }

    private static void AssertNear(Vector3d actual, Vector3d expected)
    {
        AssertNear(actual.X, expected.X);
        AssertNear(actual.Y, expected.Y);
        AssertNear(actual.Z, expected.Z);
    }
}
