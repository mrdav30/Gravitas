using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Response;

public sealed class ContactNormalImpulseResponseTests
{
    [Fact]
    public void ThreeD_UnaccumulatedExtremeRestitution_ShouldNotSaturateBeforeFusedRatio()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> source = scenario.CreateCuboid(
            Vector3d.Zero,
            mass: Fixed64.MaxValue);
        ScenarioBody<LSCuboidCollider> target = scenario.CreateCuboid(
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            mass: Fixed64.MaxValue);

        bool resolved = ContactNormalImpulse3D.TryCalculateVelocityDeltas(
            source.Body,
            Vector3d.Right * Fixed64.MaxValue,
            Vector3d.Zero,
            Vector3d.Zero,
            target.Body,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.One,
            Fixed64.Zero,
            out ContactNormalVelocityDeltaResult3D result);

        resolved.Should().BeTrue();
        result.LinearVelocityDeltaA.Should().Be(Vector3d.Left * Fixed64.MaxValue);
        result.LinearVelocityDeltaB.Should().Be(Vector3d.Right * Fixed64.MaxValue);
    }

    [Fact]
    public void TwoD_UnaccumulatedExtremeRestitution_ShouldNotSaturateBeforeFusedRatio()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = CreateBox2D(context, Vector2d.Zero);
        SolidBody2D target = CreateBox2D(
            context,
            new Vector2d((Fixed64)2, Fixed64.Zero));
        source.Mass = Fixed64.MaxValue;
        target.Mass = Fixed64.MaxValue;

        bool resolved = ContactNormalImpulse2D.TryCalculateVelocityDeltas(
            source,
            Vector2d.Right * Fixed64.MaxValue,
            Fixed64.Zero,
            Vector2d.Zero,
            target,
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Zero,
            Vector2d.Right,
            Fixed64.One,
            Fixed64.Zero,
            out ContactNormalVelocityDeltaResult2D result);

        resolved.Should().BeTrue();
        result.LinearVelocityDeltaA.Should().Be(Vector2d.Left * Fixed64.MaxValue);
        result.LinearVelocityDeltaB.Should().Be(Vector2d.Right * Fixed64.MaxValue);
    }

    [Fact]
    public void ThreeD_UnaccumulatedMaxMassPair_ShouldResolveRepresentableDeltasWithoutImpulseScalar()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> source = scenario.CreateCuboid(Vector3d.Zero, mass: Fixed64.MaxValue);
        ScenarioBody<LSCuboidCollider> target = scenario.CreateCuboid(
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            mass: Fixed64.MaxValue);

        bool resolved = ContactNormalImpulse3D.TryCalculateVelocityDeltas(
            source.Body,
            Vector3d.Right * (Fixed64)8,
            Vector3d.Zero,
            Vector3d.Zero,
            target.Body,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            out ContactNormalVelocityDeltaResult3D result);

        resolved.Should().BeTrue();
        result.NormalVelocity.Should().Be((Fixed64)(-8));
        result.LinearVelocityDeltaA.Should().Be(-Vector3d.Right * (Fixed64)4);
        result.AngularVelocityDeltaA.Should().Be(Vector3d.Zero);
        result.LinearVelocityDeltaB.Should().Be(Vector3d.Right * (Fixed64)4);
        result.AngularVelocityDeltaB.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void TwoD_UnaccumulatedMaxMassPair_ShouldResolveRepresentableDeltasWithoutImpulseScalar()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = CreateBox2D(context, Vector2d.Zero);
        SolidBody2D target = CreateBox2D(context, new Vector2d((Fixed64)2, Fixed64.Zero));
        source.Mass = Fixed64.MaxValue;
        target.Mass = Fixed64.MaxValue;

        bool resolved = ContactNormalImpulse2D.TryCalculateVelocityDeltas(
            source,
            Vector2d.Right * (Fixed64)8,
            Fixed64.Zero,
            Vector2d.Zero,
            target,
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Zero,
            Vector2d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            out ContactNormalVelocityDeltaResult2D result);

        resolved.Should().BeTrue();
        result.NormalVelocity.Should().Be((Fixed64)(-8));
        result.LinearVelocityDeltaA.Should().Be(-Vector2d.Right * (Fixed64)4);
        result.AngularVelocityDeltaA.Should().Be(Fixed64.Zero);
        result.LinearVelocityDeltaB.Should().Be(Vector2d.Right * (Fixed64)4);
        result.AngularVelocityDeltaB.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void UnaccumulatedNormalCalculators_ShouldNotAllocateAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> source3D = scenario.CreateCuboid(Vector3d.Zero, mass: Fixed64.MaxValue);
        ScenarioBody<LSCuboidCollider> target3D = scenario.CreateCuboid(
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            mass: Fixed64.MaxValue);
        using GravitasWorldContext context2D = Physics2DTestWorld.CreateContext();
        SolidBody2D source2D = CreateBox2D(context2D, Vector2d.Zero);
        SolidBody2D target2D = CreateBox2D(context2D, new Vector2d((Fixed64)2, Fixed64.Zero));
        source2D.Mass = Fixed64.MaxValue;
        target2D.Mass = Fixed64.MaxValue;

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            () =>
            {
                _ = ContactNormalImpulse3D.TryCalculateVelocityDeltas(
                    source3D.Body,
                    Vector3d.Right * (Fixed64)8,
                    Vector3d.Zero,
                    Vector3d.Zero,
                    target3D.Body,
                    Vector3d.Zero,
                    Vector3d.Zero,
                    Vector3d.Zero,
                    Vector3d.Right,
                    Fixed64.Zero,
                    Fixed64.Zero,
                    out _);
                _ = ContactNormalImpulse2D.TryCalculateVelocityDeltas(
                    source2D,
                    Vector2d.Right * (Fixed64)8,
                    Fixed64.Zero,
                    Vector2d.Zero,
                    target2D,
                    Vector2d.Zero,
                    Fixed64.Zero,
                    Vector2d.Zero,
                    Vector2d.Right,
                    Fixed64.Zero,
                    Fixed64.Zero,
                    out _);
            },
            warmupIterations: 4,
            stabilizationIterations: 2,
            measurementIterations: 4);

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void ThreeD_UnaccumulatedAuthoredKinematicAngularVelocity_ShouldDriveDynamicTarget()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> source = scenario.CreateCuboid(Vector3d.Zero);
        ScenarioBody<LSCuboidCollider> target = scenario.CreateCuboid(
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));
        source.Body.SetMotionType(BodyMotionType.Kinematic);

        bool resolved = ContactNormalImpulse3D.TryCalculateVelocityDeltas(
            source.Body,
            Vector3d.Zero,
            -Vector3d.Forward,
            Vector3d.Up,
            target.Body,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Up,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            out ContactNormalVelocityDeltaResult3D result);

        resolved.Should().BeTrue();
        result.NormalVelocity.Should().Be(-Fixed64.One);
        result.LinearVelocityDeltaA.Should().Be(Vector3d.Zero);
        result.AngularVelocityDeltaA.Should().Be(Vector3d.Zero);
        result.LinearVelocityDeltaB.X.Should().BeGreaterThan(Fixed64.Zero);
        result.AngularVelocityDeltaB.Should().NotBe(Vector3d.Zero);
    }

    [Fact]
    public void TwoD_UnaccumulatedAuthoredKinematicAngularVelocity_ShouldDriveDynamicTarget()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = CreateBox2D(context, Vector2d.Zero);
        SolidBody2D target = CreateBox2D(context, new Vector2d((Fixed64)2, Fixed64.Zero));
        source.SetMotionType(BodyMotionType.Kinematic);

        bool resolved = ContactNormalImpulse2D.TryCalculateVelocityDeltas(
            source,
            Vector2d.Zero,
            -Fixed64.One,
            Vector2d.Forward,
            target,
            Vector2d.Zero,
            Fixed64.Zero,
            Vector2d.Forward,
            Vector2d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            out ContactNormalVelocityDeltaResult2D result);

        resolved.Should().BeTrue();
        result.NormalVelocity.Should().Be(-Fixed64.One);
        result.LinearVelocityDeltaA.Should().Be(Vector2d.Zero);
        result.AngularVelocityDeltaA.Should().Be(Fixed64.Zero);
        result.LinearVelocityDeltaB.X.Should().BeGreaterThan(Fixed64.Zero);
        result.AngularVelocityDeltaB.Should().NotBe(Fixed64.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ThreeD_SeparatingOrTangentialAngularVelocity_ShouldNotApplyNormalImpulse(bool tangential)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> source = scenario.CreateCuboid(Vector3d.Zero);
        ScenarioBody<LSCuboidCollider> target = scenario.CreateCuboid(
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        source.Collider.Material = PhysicsMaterial.Frictionless;
        target.Collider.Material = PhysicsMaterial.Frictionless;
        source.Body.ApplyCollisionAngularVelocityDelta(Vector3d.Forward);
        Vector3d relativePoint = tangential ? Vector3d.Right : Vector3d.Up;
        CollisionPair pair = CreatePair3D(scenario, source, target, relativePoint);
        Vector3d sourceAngularVelocity = source.Body.AngularVelocity;

        CollisionResponse.CalculateImpulse(pair, applyCachedImpulse: false, applyPositionCorrection: false);

        source.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        source.Body.AngularVelocity.Should().Be(sourceAngularVelocity);
    }

    [Fact]
    public void ThreeD_OffCenterAngularClosing_WithFrozenNormalAxis_ShouldApplyOnlyAngularDelta()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> source = scenario.CreateCuboid(Vector3d.Zero);
        ScenarioBody<LSCuboidCollider> target = scenario.CreateCuboid(
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        source.Collider.Material = PhysicsMaterial.Frictionless;
        target.Collider.Material = PhysicsMaterial.Frictionless;
        source.Body.ApplyCollisionAngularVelocityDelta(-Vector3d.Forward);
        source.Body.FreezeAxes = BodyFreezeAxes3D.PositionX;
        CollisionPair pair = CreatePair3D(scenario, source, target, Vector3d.Up);
        Fixed64 angularVelocityBefore = source.Body.AngularVelocity.Z;

        CollisionResponse.CalculateImpulse(pair, applyCachedImpulse: false, applyPositionCorrection: false);

        source.Body.LinearVelocity.X.Should().Be(Fixed64.Zero);
        source.Body.AngularVelocity.Z.Should().BeGreaterThan(angularVelocityBefore);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void TwoD_SeparatingOrTangentialAngularVelocity_ShouldNotApplyNormalImpulse(bool tangential)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = CreateBox2D(context, Vector2d.Zero);
        SolidBody2D target = CreateBox2D(
            context,
            new Vector2d((Fixed64)2, Fixed64.Zero),
            immovable: true);
        source.Collider.Material = PhysicsMaterial.Frictionless;
        target.Collider.Material = PhysicsMaterial.Frictionless;
        source.ApplyCollisionAngularVelocityDelta(Fixed64.One);
        Vector2d relativePoint = tangential ? Vector2d.Right : Vector2d.Forward;
        CollisionPair2D pair = CreatePair2D(source, target, relativePoint);
        Fixed64 sourceAngularVelocity = source.AngularVelocity;

        CollisionResponse2D.Resolve(pair, applyCachedImpulse: false, applyPositionCorrection: false);

        source.LinearVelocity.Should().Be(Vector2d.Zero);
        source.AngularVelocity.Should().Be(sourceAngularVelocity);
    }

    [Fact]
    public void TwoD_OffCenterAngularClosing_WithFrozenNormalAxis_ShouldApplyOnlyAngularDelta()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = CreateBox2D(context, Vector2d.Zero);
        SolidBody2D target = CreateBox2D(
            context,
            new Vector2d((Fixed64)2, Fixed64.Zero),
            immovable: true);
        source.Collider.Material = PhysicsMaterial.Frictionless;
        target.Collider.Material = PhysicsMaterial.Frictionless;
        source.ApplyCollisionAngularVelocityDelta(-Fixed64.One);
        source.FreezeAxes = BodyFreezeAxes2D.PositionX;
        CollisionPair2D pair = CreatePair2D(source, target, Vector2d.Forward);
        Fixed64 angularVelocityBefore = source.AngularVelocity;

        CollisionResponse2D.Resolve(pair, applyCachedImpulse: false, applyPositionCorrection: false);

        source.LinearVelocity.X.Should().Be(Fixed64.Zero);
        source.AngularVelocity.Should().BeGreaterThan(angularVelocityBefore);
    }

    private static CollisionPair CreatePair3D(
        PhysicsScenarioBuilder scenario,
        ScenarioBody<LSCuboidCollider> source,
        ScenarioBody<LSCuboidCollider> target,
        Vector3d relativePoint)
    {
        CollisionPair pair = scenario.CreatePair(source.Collider, target.Collider);
        pair.Manifold.SetContact(
            source.Body.WorldCenterOfMass + relativePoint,
            target.Body.WorldCenterOfMass + relativePoint,
            Fixed64.Zero,
            Vector3d.Right);
        return pair;
    }

    private static CollisionPair2D CreatePair2D(
        SolidBody2D source,
        SolidBody2D target,
        Vector2d relativePoint)
    {
        var pair = new CollisionPair2D(source.Collider, target.Collider);
        pair.Manifold.SetContact(
            source.WorldCenterOfMass + relativePoint,
            target.WorldCenterOfMass + relativePoint,
            Fixed64.Zero,
            Vector2d.Right);
        return pair;
    }

    private static SolidBody2D CreateBox2D(
        GravitasWorldContext context,
        Vector2d position,
        bool immovable = false)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var body = new SolidBody2D(
            new TestMatterAgent(context, transform),
            new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)))
        {
            Mass = Fixed64.One
        };
        body.Initialize(position, motionType: immovable ? BodyMotionType.Static : BodyMotionType.Dynamic);
        return body;
    }
}
