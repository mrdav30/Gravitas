using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CollisionResponseInvariantTests
{
    private static readonly Fixed64 Tolerance = Fixed64.Fraction(1, 1_000_000);

    [Fact]
    public void ContactManifold_ShouldStoreDetectionDepthWithoutSolverMargin()
    {
        var manifold = new ContactManifold();
        Fixed64 smallDepth = Fixed64.Fraction(1, 1_000);

        manifold.HasContact.Should().BeFalse();

        manifold.SetContact(Vector3d.Zero, Vector3d.Right, smallDepth, Vector3d.Right);

        manifold.HasContact.Should().BeTrue();
        manifold.PrimaryContact.Depth.Should().Be(smallDepth);

        manifold.Reset();

        manifold.HasContact.Should().BeFalse();
    }

    [Fact]
    public void CalculateImpulse_ForEqualMassElasticHeadOnCollision_ShouldSwapNormalVelocities()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.Fraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        left.Body.RestitutionCoefficient = Fixed64.One;
        right.Body.RestitutionCoefficient = Fixed64.One;
        Push(left.Body, 60);
        Push(right.Body, -60);
        CollisionPair pair = CreateDetectedPair(scenario, left.Collider, right.Collider);
        Fixed64 leftVelocityBefore = left.Body.LinearVelocity.x;
        Fixed64 rightVelocityBefore = right.Body.LinearVelocity.x;

        CollisionResponse.CalculateImpulse(pair);

        AssertNear(left.Body.LinearVelocity.x, rightVelocityBefore);
        AssertNear(right.Body.LinearVelocity.x, leftVelocityBefore);
    }

    [Fact]
    public void CalculateImpulse_WithoutContactData_ShouldLeaveBodiesUnchanged()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.Fraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        Push(left.Body, 60);
        Push(right.Body, -60);
        CollisionPair pair = scenario.CreatePair(left.Collider, right.Collider);
        Vector3d leftPositionBefore = left.Body.Position3d;
        Vector3d rightPositionBefore = right.Body.Position3d;
        Vector3d leftVelocityBefore = left.Body.LinearVelocity;
        Vector3d rightVelocityBefore = right.Body.LinearVelocity;

        CollisionResponse.CalculateImpulse(pair);

        left.Body.Position3d.Should().Be(leftPositionBefore);
        right.Body.Position3d.Should().Be(rightPositionBefore);
        left.Body.LinearVelocity.Should().Be(leftVelocityBefore);
        right.Body.LinearVelocity.Should().Be(rightVelocityBefore);
    }

    [Fact]
    public void CalculateImpulse_BelowRestitutionThreshold_ShouldRemoveClosingVelocityWithoutBounce()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.Fraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        left.Body.RestitutionCoefficient = Fixed64.One;
        right.Body.RestitutionCoefficient = Fixed64.One;
        Push(left.Body, 3);
        Push(right.Body, -3);
        CollisionPair pair = CreateDetectedPair(scenario, left.Collider, right.Collider);

        CollisionResponse.CalculateImpulse(pair);

        AssertNear(left.Body.LinearVelocity.x, Fixed64.Zero);
        AssertNear(right.Body.LinearVelocity.x, Fixed64.Zero);
    }

    [Fact]
    public void CalculateImpulse_WithDifferentMasses_ShouldApplySmallerVelocityDeltaToHeavierBody()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> heavy = scenario.CreateSphere(
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            mass: (Fixed64)4);
        ScenarioBody<LSSphereCollider> light = scenario.CreateSphere(
            PhysicsScenarioBuilder.Vector(Fixed64.Fraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            mass: Fixed64.One);
        Push(heavy.Body, 60);
        Push(light.Body, -60);
        CollisionPair pair = CreateDetectedPair(scenario, heavy.Collider, light.Collider);
        Fixed64 heavyVelocityBefore = heavy.Body.LinearVelocity.x;
        Fixed64 lightVelocityBefore = light.Body.LinearVelocity.x;

        CollisionResponse.CalculateImpulse(pair);

        Fixed64 heavyDelta = (heavy.Body.LinearVelocity.x - heavyVelocityBefore).Abs();
        Fixed64 lightDelta = (light.Body.LinearVelocity.x - lightVelocityBefore).Abs();
        heavyDelta.Should().BeLessThan(lightDelta);
    }

    [Fact]
    public void CalculateImpulse_WithKinematicBody_ShouldTreatKinematicBodyAsInfiniteMass()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> kinematic = scenario.CreateSphere(
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            isKinematic: true);
        ScenarioBody<LSSphereCollider> movable = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.Fraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        Push(movable.Body, -60);
        CollisionPair pair = CreateDetectedPair(scenario, kinematic.Collider, movable.Collider);
        Vector3d kinematicPositionBefore = kinematic.Body.Position3d;
        Vector3d kinematicVelocityBefore = kinematic.Body.LinearVelocity;
        Fixed64 movableVelocityBefore = movable.Body.LinearVelocity.x;

        CollisionResponse.CalculateImpulse(pair);

        kinematic.Body.Position3d.Should().Be(kinematicPositionBefore);
        kinematic.Body.LinearVelocity.Should().Be(kinematicVelocityBefore);
        movable.Body.LinearVelocity.x.Should().BeGreaterThan(movableVelocityBefore);
    }

    [Fact]
    public void CalculateImpulse_ShouldApplyPenetrationCorrectionOnlyAboveSlop()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.Fraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        CollisionPair pair = scenario.CreatePair(left.Collider, right.Collider);
        Vector3d leftStart = left.Body.Position3d;
        Vector3d rightStart = right.Body.Position3d;

        pair.Manifold.SetContact(left.Collider.Center, right.Collider.Center, Fixed64.Fraction(1, 1_000), Vector3d.Right);
        CollisionResponse.CalculateImpulse(pair);

        left.Body.Position3d.Should().Be(leftStart);
        right.Body.Position3d.Should().Be(rightStart);

        pair.Manifold.SetContact(left.Collider.Center, right.Collider.Center, Fixed64.Fraction(1, 10), Vector3d.Right);
        CollisionResponse.CalculateImpulse(pair);

        left.Body.Position3d.x.Should().BeLessThan(leftStart.x);
        right.Body.Position3d.x.Should().BeGreaterThan(rightStart.x);
    }

    [Fact]
    public void CalculateImpulse_RepeatedDeterministicSequence_ShouldReplaySameState()
    {
        ResponseState first = RunDeterministicResponseSequence();
        ResponseState second = RunDeterministicResponseSequence();

        second.Should().Be(first);
    }

    [Fact]
    public void CalculateImpulse_ForDynamicBodies_ShouldApplyOpposingLinearImpulse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.Fraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        Push(left.Body, 60);
        Push(right.Body, -60);
        CollisionPair pair = CreateDetectedPair(scenario, left.Collider, right.Collider);
        Fixed64 leftVelocityBefore = left.Body.LinearVelocity.x;
        Fixed64 rightVelocityBefore = right.Body.LinearVelocity.x;

        CollisionResponse.CalculateImpulse(pair);

        left.Body.LinearVelocity.x.Should().BeLessThan(leftVelocityBefore);
        right.Body.LinearVelocity.x.Should().BeGreaterThan(rightVelocityBefore);
    }

    [Fact]
    public void CalculateImpulse_WithImmovableBody_ShouldNotMoveImmovableBody()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> immovable = scenario.CreateSphere(
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            immovable: true);
        ScenarioBody<LSSphereCollider> movable = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.Fraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        Push(movable.Body, -60);
        CollisionPair pair = CreateDetectedPair(scenario, immovable.Collider, movable.Collider);
        Vector3d immovableVelocityBefore = immovable.Body.LinearVelocity;
        Fixed64 movableVelocityBefore = movable.Body.LinearVelocity.x;

        CollisionResponse.CalculateImpulse(pair);

        immovable.Body.LinearVelocity.Should().Be(immovableVelocityBefore);
        movable.Body.LinearVelocity.x.Should().BeGreaterThan(movableVelocityBefore);
    }

    [Fact]
    public void CalculateImpulse_WithTriggerCollider_ShouldNotApplyPhysicalImpulse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> trigger = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> solid = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.Fraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        PhysicsScenarioBuilder.SetTrigger(trigger.Collider);
        Push(trigger.Body, 60);
        Push(solid.Body, -60);
        CollisionPair pair = CreateDetectedPair(scenario, trigger.Collider, solid.Collider);
        Vector3d triggerVelocityBefore = trigger.Body.LinearVelocity;
        Vector3d solidVelocityBefore = solid.Body.LinearVelocity;

        CollisionResponse.CalculateImpulse(pair);

        trigger.Body.LinearVelocity.Should().Be(triggerVelocityBefore);
        solid.Body.LinearVelocity.Should().Be(solidVelocityBefore);
    }

    [Fact]
    public void CalculateImpulse_WithZeroRestitution_ShouldDampenWithoutReversingLinearVelocity()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.Fraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        left.Body.RestitutionCoefficient = Fixed64.Zero;
        right.Body.RestitutionCoefficient = Fixed64.Zero;
        Push(left.Body, 60);
        Push(right.Body, -60);
        CollisionPair pair = CreateDetectedPair(scenario, left.Collider, right.Collider);
        Fixed64 leftVelocityBefore = left.Body.LinearVelocity.x;
        Fixed64 rightVelocityBefore = right.Body.LinearVelocity.x;

        CollisionResponse.CalculateImpulse(pair);

        AssertNear(left.Body.LinearVelocity.x, Fixed64.Zero);
        AssertNear(right.Body.LinearVelocity.x, Fixed64.Zero);
        left.Body.LinearVelocity.x.Should().BeLessThan(leftVelocityBefore);
        right.Body.LinearVelocity.x.Should().BeGreaterThan(rightVelocityBefore);
    }

    [Fact]
    public void CalculateImpulse_WithTangentialVelocity_ShouldApplyDeterministicFrictionImpulse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> wall = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            immovable: true);
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(
            Fixed64.Fraction(3, 4),
            Fixed64.Zero,
            Fixed64.Zero));
        mover.Body.AddLinearImpulse(new Vector3d((Fixed64)(-60), Fixed64.Zero, (Fixed64)30));
        CollisionPair pair = CreateDetectedPair(scenario, wall.Collider, mover.Collider);
        Fixed64 tangentialSpeedBefore = mover.Body.LinearVelocity.z.Abs();

        CollisionResponse.CalculateImpulse(pair);

        mover.Body.LinearVelocity.z.Abs().Should().BeLessThan(tangentialSpeedBefore);
    }

    [Fact]
    public void CalculateImpulse_WithSlopedContactNormal_ShouldResolveAgainstContactPlane()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Vector3d normal = new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero).Normal;
        Vector3d tangent = new Vector3d(-normal.y, normal.x, Fixed64.Zero).Normal;
        ScenarioBody<LSSphereCollider> slope = scenario.CreateSphere(
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            immovable: true);
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(normal * Fixed64.Fraction(3, 4));
        CollisionPair pair = scenario.CreatePair(slope.Collider, mover.Collider);
        mover.Body.AddLinearImpulse((-normal * (Fixed64)60) + (tangent * (Fixed64)30));
        pair.Manifold.SetContact(pair.ColliderA.Center, pair.ColliderB.Center, Fixed64.Fraction(1, 10), normal);
        Fixed64 tangentialSpeedBefore = Vector3d.Dot(mover.Body.LinearVelocity, tangent).Abs();

        CollisionResponse.CalculateImpulse(pair);

        Fixed64 normalSpeedAfter = Vector3d.Dot(mover.Body.LinearVelocity, normal);
        normalSpeedAfter.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        Vector3d.Dot(mover.Body.LinearVelocity, tangent).Abs().Should().BeLessThan(tangentialSpeedBefore);
    }

    [Fact]
    public void CalculateImpulse_WithOffCenterContact_ShouldApplyAngularImpulse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(
            Fixed64.Fraction(3, 4),
            Fixed64.Fraction(1, 4),
            Fixed64.Zero));
        Push(sphere.Body, -60);
        CollisionPair pair = CreateDetectedPair(scenario, cuboid.Collider, sphere.Collider);

        CollisionResponse.CalculateImpulse(pair);

        cuboid.Body.AngularVelocity.Should().NotBe(Vector3d.Zero);
    }

    [Fact]
    public void CalculateImpulse_WithCenteredFaceManifold_ShouldNotIntroduceAngularVelocity()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> floor = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            immovable: true);
        ScenarioBody<LSCuboidCollider> box = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(Fixed64.Zero, Fixed64.Fraction(3, 4), Fixed64.Zero));
        box.Body.AddLinearImpulse(new Vector3d(Fixed64.Zero, (Fixed64)(-60), Fixed64.Zero));
        CollisionPair pair = CreateDetectedPair(scenario, floor.Collider, box.Collider);
        pair.Manifold.Count.Should().Be(ContactManifold.MaxContactCount);

        CollisionResponse.CalculateImpulse(pair);

        box.Body.AngularVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void CalculateImpulse_ShouldNotAllocateForPreparedContactsAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> wall = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            immovable: true);
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(
            Fixed64.Fraction(3, 4),
            Fixed64.Zero,
            Fixed64.Zero));
        CollisionPair singleContactPair = CreateDetectedPair(scenario, wall.Collider, sphere.Collider);
        sphere.Body.AddLinearImpulse(new Vector3d((Fixed64)(-60), Fixed64.Zero, (Fixed64)30));
        CollisionResponse.CalculateImpulse(singleContactPair);

        ScenarioBody<LSCuboidCollider> floor = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(4, 0, 0),
            immovable: true);
        ScenarioBody<LSCuboidCollider> box = scenario.CreateCuboid(new Vector3d((Fixed64)4, Fixed64.Fraction(3, 4), Fixed64.Zero));
        CollisionPair facePair = CreateDetectedPair(scenario, floor.Collider, box.Collider);
        box.Body.AddLinearImpulse(new Vector3d(Fixed64.Zero, (Fixed64)(-60), Fixed64.Zero));
        CollisionResponse.CalculateImpulse(facePair);

        sphere.Body.AddLinearImpulse(new Vector3d((Fixed64)(-60), Fixed64.Zero, (Fixed64)30));
        box.Body.AddLinearImpulse(new Vector3d(Fixed64.Zero, (Fixed64)(-60), Fixed64.Zero));

        long allocatedBytes = MeasureAllocatedBytes(() =>
        {
            CollisionResponse.CalculateImpulse(singleContactPair);
            CollisionResponse.CalculateImpulse(facePair);
        });

        allocatedBytes.Should().Be(0);
    }

    private static CollisionPair CreateDetectedPair(
        PhysicsScenarioBuilder scenario,
        LSCollider colliderA,
        LSCollider colliderB)
    {
        CollisionPair pair = scenario.CreatePair(colliderA, colliderB);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        return pair;
    }

    private static void Push(StiffBody body, int xImpulse)
    {
        body.AddLinearImpulse(new Vector3d((Fixed64)xImpulse, Fixed64.Zero, Fixed64.Zero));
    }

    private static ResponseState RunDeterministicResponseSequence()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.Fraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        left.Body.RestitutionCoefficient = Fixed64.Half;
        right.Body.RestitutionCoefficient = Fixed64.Half;
        Push(left.Body, 60);
        Push(right.Body, -30);

        for (int i = 0; i < 8; i++)
        {
            CollisionPair pair = scenario.CreatePair(left.Collider, right.Collider);
            pair.Manifold.SetContact(left.Collider.Center, right.Collider.Center, Fixed64.Fraction(1, 20), Vector3d.Right);
            CollisionResponse.CalculateImpulse(pair);
        }

        return new ResponseState(
            left.Body.Position3d,
            right.Body.Position3d,
            left.Body.LinearVelocity,
            right.Body.LinearVelocity,
            left.Body.AngularVelocity,
            right.Body.AngularVelocity);
    }

    private static void AssertNear(Fixed64 actual, Fixed64 expected)
    {
        (actual - expected).Abs().Should().BeLessThan(Tolerance);
    }

    private static long MeasureAllocatedBytes(Action action)
    {
        GC.Collect();
        GC.WaitForPendingFinalizers();
        GC.Collect();

        long before = GC.GetAllocatedBytesForCurrentThread();
        action();
        return GC.GetAllocatedBytesForCurrentThread() - before;
    }

    private readonly record struct ResponseState(
        Vector3d LeftPosition,
        Vector3d RightPosition,
        Vector3d LeftVelocity,
        Vector3d RightVelocity,
        Vector3d LeftAngularVelocity,
        Vector3d RightAngularVelocity);
}
