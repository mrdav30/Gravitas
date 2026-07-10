using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CollisionResponseInvariantTests
{
    private static readonly Fixed64 Tolerance = Fixed64.FromFraction(1, 1_000_000);

    [Fact]
    public void ContactManifold_ShouldStoreDetectionDepthWithoutSolverMargin()
    {
        var manifold = new ContactManifold();
        Fixed64 smallDepth = Fixed64.FromFraction(1, 1_000);

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
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        left.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        right.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        Push(left.Body, 60);
        Push(right.Body, -60);
        CollisionPair pair = CreateDetectedPair(scenario, left.Collider, right.Collider);
        Fixed64 leftVelocityBefore = left.Body.LinearVelocity.X;
        Fixed64 rightVelocityBefore = right.Body.LinearVelocity.X;

        CollisionResponse.CalculateImpulse(pair);

        AssertNear(left.Body.LinearVelocity.X, rightVelocityBefore);
        AssertNear(right.Body.LinearVelocity.X, leftVelocityBefore);
    }

    [Fact]
    public void CalculateImpulse_WithoutContactData_ShouldLeaveBodiesUnchanged()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
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
    public void CalculateImpulse_WithZeroContactNormal_ShouldUseColliderCenterFallback()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        Push(left.Body, 60);
        Push(right.Body, -60);
        CollisionPair pair = scenario.CreatePair(left.Collider, right.Collider);
        pair.Manifold.SetContact(left.Collider.Center, right.Collider.Center, Fixed64.FromFraction(1, 4), Vector3d.Zero);
        Fixed64 leftVelocityBefore = left.Body.LinearVelocity.X;
        Fixed64 rightVelocityBefore = right.Body.LinearVelocity.X;

        CollisionResponse.CalculateImpulse(pair);

        left.Body.LinearVelocity.X.Should().BeLessThan(leftVelocityBefore);
        right.Body.LinearVelocity.X.Should().BeGreaterThan(rightVelocityBefore);
    }

    [Fact]
    public void CalculateImpulse_WithOpposedContactNormal_ShouldFlipTowardSecondCollider()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        Push(left.Body, 60);
        Push(right.Body, -60);
        CollisionPair pair = scenario.CreatePair(left.Collider, right.Collider);
        pair.Manifold.SetContact(left.Collider.Center, right.Collider.Center, Fixed64.FromFraction(1, 4), -Vector3d.Right);
        Fixed64 leftVelocityBefore = left.Body.LinearVelocity.X;
        Fixed64 rightVelocityBefore = right.Body.LinearVelocity.X;

        CollisionResponse.CalculateImpulse(pair);

        left.Body.LinearVelocity.X.Should().BeLessThan(leftVelocityBefore);
        right.Body.LinearVelocity.X.Should().BeGreaterThan(rightVelocityBefore);
    }

    [Fact]
    public void CalculateImpulse_WithZeroNormalAndNoFallbackDirection_ShouldIgnoreContact()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        Push(left.Body, 60);
        Push(right.Body, -60);
        CollisionPair pair = scenario.CreatePair(left.Collider, right.Collider);
        pair.Manifold.SetContact(left.Collider.Center, right.Collider.Center, Fixed64.FromFraction(1, 4), Vector3d.Zero);
        Vector3d leftVelocityBefore = left.Body.LinearVelocity;
        Vector3d rightVelocityBefore = right.Body.LinearVelocity;

        CollisionResponse.CalculateImpulse(pair);

        left.Body.LinearVelocity.Should().Be(leftVelocityBefore);
        right.Body.LinearVelocity.Should().Be(rightVelocityBefore);
    }

    [Fact]
    public void CalculateImpulse_BelowRestitutionThreshold_ShouldRemoveClosingVelocityWithoutBounce()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        left.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        right.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        Push(left.Body, 3);
        Push(right.Body, -3);
        CollisionPair pair = CreateDetectedPair(scenario, left.Collider, right.Collider);

        CollisionResponse.CalculateImpulse(pair);

        AssertNear(left.Body.LinearVelocity.X, Fixed64.Zero);
        AssertNear(right.Body.LinearVelocity.X, Fixed64.Zero);
    }

    [Fact]
    public void CalculateImpulse_WithHighConfiguredRestitutionThreshold_ShouldSuppressBounce()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Settings.RestitutionVelocityThreshold = (Fixed64)4;
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        left.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        right.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        Push(left.Body, 60);
        Push(right.Body, -60);
        CollisionPair pair = CreateDetectedPair(scenario, left.Collider, right.Collider);

        CollisionResponse.CalculateImpulse(pair);

        AssertNear(left.Body.LinearVelocity.X, Fixed64.Zero);
        AssertNear(right.Body.LinearVelocity.X, Fixed64.Zero);
    }

    [Fact]
    public void CalculateImpulse_WithZeroConfiguredRestitutionThreshold_ShouldBounceLowSpeedContact()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Settings.RestitutionVelocityThreshold = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        left.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        right.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        Push(left.Body, 3);
        Push(right.Body, -3);
        CollisionPair pair = CreateDetectedPair(scenario, left.Collider, right.Collider);
        Fixed64 leftVelocityBefore = left.Body.LinearVelocity.X;
        Fixed64 rightVelocityBefore = right.Body.LinearVelocity.X;

        CollisionResponse.CalculateImpulse(pair);

        AssertNear(left.Body.LinearVelocity.X, rightVelocityBefore);
        AssertNear(right.Body.LinearVelocity.X, leftVelocityBefore);
    }

    [Fact]
    public void CalculateImpulse_WithDifferentMasses_ShouldApplySmallerVelocityDeltaToHeavierBody()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> heavy = scenario.CreateSphere(
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            mass: (Fixed64)4);
        ScenarioBody<LSSphereCollider> light = scenario.CreateSphere(
            PhysicsScenarioBuilder.Vector(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            mass: Fixed64.One);
        Push(heavy.Body, 60);
        Push(light.Body, -60);
        CollisionPair pair = CreateDetectedPair(scenario, heavy.Collider, light.Collider);
        Fixed64 heavyVelocityBefore = heavy.Body.LinearVelocity.X;
        Fixed64 lightVelocityBefore = light.Body.LinearVelocity.X;

        CollisionResponse.CalculateImpulse(pair);

        Fixed64 heavyDelta = (heavy.Body.LinearVelocity.X - heavyVelocityBefore).Abs();
        Fixed64 lightDelta = (light.Body.LinearVelocity.X - lightVelocityBefore).Abs();
        heavyDelta.Should().BeLessThan(lightDelta);
    }

    [Fact]
    public void CalculateImpulse_WithKinematicBody_ShouldTreatKinematicBodyAsInfiniteMass()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> kinematic = scenario.CreateSphere(
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            isKinematic: true);
        ScenarioBody<LSSphereCollider> movable = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        Push(movable.Body, -60);
        CollisionPair pair = CreateDetectedPair(scenario, kinematic.Collider, movable.Collider);
        Vector3d kinematicPositionBefore = kinematic.Body.Position3d;
        Vector3d kinematicVelocityBefore = kinematic.Body.LinearVelocity;
        Fixed64 movableVelocityBefore = movable.Body.LinearVelocity.X;

        CollisionResponse.CalculateImpulse(pair);

        kinematic.Body.Position3d.Should().Be(kinematicPositionBefore);
        kinematic.Body.LinearVelocity.Should().Be(kinematicVelocityBefore);
        movable.Body.LinearVelocity.X.Should().BeGreaterThan(movableVelocityBefore);
    }

    [Fact]
    public void CalculateImpulse_ShouldApplyPenetrationCorrectionOnlyAboveSlop()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        CollisionPair pair = scenario.CreatePair(left.Collider, right.Collider);
        Vector3d leftStart = left.Body.Position3d;
        Vector3d rightStart = right.Body.Position3d;

        pair.Manifold.SetContact(left.Collider.Center, right.Collider.Center, Fixed64.FromFraction(1, 1_000), Vector3d.Right);
        CollisionResponse.CalculateImpulse(pair);

        left.Body.Position3d.Should().Be(leftStart);
        right.Body.Position3d.Should().Be(rightStart);

        pair.Manifold.SetContact(left.Collider.Center, right.Collider.Center, Fixed64.FromFraction(1, 10), Vector3d.Right);
        CollisionResponse.CalculateImpulse(pair);

        left.Body.Position3d.X.Should().BeLessThan(leftStart.X);
        right.Body.Position3d.X.Should().BeGreaterThan(rightStart.X);
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
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        Push(left.Body, 60);
        Push(right.Body, -60);
        CollisionPair pair = CreateDetectedPair(scenario, left.Collider, right.Collider);
        Fixed64 leftVelocityBefore = left.Body.LinearVelocity.X;
        Fixed64 rightVelocityBefore = right.Body.LinearVelocity.X;

        CollisionResponse.CalculateImpulse(pair);

        left.Body.LinearVelocity.X.Should().BeLessThan(leftVelocityBefore);
        right.Body.LinearVelocity.X.Should().BeGreaterThan(rightVelocityBefore);
    }

    [Fact]
    public void CalculateImpulse_WithImmovableBody_ShouldNotMoveImmovableBody()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> immovable = scenario.CreateSphere(
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            immovable: true);
        ScenarioBody<LSSphereCollider> movable = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        Push(movable.Body, -60);
        CollisionPair pair = CreateDetectedPair(scenario, immovable.Collider, movable.Collider);
        Vector3d immovableVelocityBefore = immovable.Body.LinearVelocity;
        Fixed64 movableVelocityBefore = movable.Body.LinearVelocity.X;

        CollisionResponse.CalculateImpulse(pair);

        immovable.Body.LinearVelocity.Should().Be(immovableVelocityBefore);
        movable.Body.LinearVelocity.X.Should().BeGreaterThan(movableVelocityBefore);
    }

    [Fact]
    public void CalculateImpulse_WithTriggerCollider_ShouldNotApplyPhysicalImpulse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider trigger = scenario.CreateStaticSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> solid = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        trigger.IsTrigger = true;
        Push(solid.Body, -60);
        CollisionPair pair = CreateDetectedPair(scenario, trigger, solid.Collider);
        Vector3d solidVelocityBefore = solid.Body.LinearVelocity;

        CollisionResponse.CalculateImpulse(pair);

        solid.Body.LinearVelocity.Should().Be(solidVelocityBefore);
    }

    [Fact]
    public void CalculateImpulse_WithBodylessOrFrozenPairs_ShouldNotMutateBodies()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider staticCollider = scenario.CreateStaticSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> solid = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        Push(solid.Body, -60);
        CollisionPair bodylessPair = scenario.CreatePair(staticCollider, solid.Collider);
        bodylessPair.Manifold.SetContact(staticCollider.Center, solid.Collider.Center, Fixed64.FromFraction(1, 4), Vector3d.Right);
        Vector3d solidVelocityBefore = solid.Body.LinearVelocity;

        CollisionResponse.CalculateImpulse(bodylessPair);

        solid.Body.LinearVelocity.Should().Be(solidVelocityBefore);

        ScenarioBody<LSSphereCollider> frozenA = scenario.CreateSphere(
            PhysicsScenarioBuilder.Vector(2, 0, 0),
            immovable: true,
            preventAngularForces: true);
        ScenarioBody<LSSphereCollider> frozenB = scenario.CreateSphere(
            PhysicsScenarioBuilder.Vector(Fixed64.FromFraction(11, 4), Fixed64.Zero, Fixed64.Zero),
            immovable: true,
            preventAngularForces: true);
        CollisionPair frozenPair = scenario.CreatePair(frozenA.Collider, frozenB.Collider);
        frozenPair.Manifold.SetContact(frozenA.Collider.Center, frozenB.Collider.Center, Fixed64.FromFraction(1, 4), Vector3d.Right);
        Vector3d frozenAPositionBefore = frozenA.Body.Position3d;
        Vector3d frozenBPositionBefore = frozenB.Body.Position3d;

        CollisionResponse.CalculateImpulse(frozenPair);

        frozenA.Body.Position3d.Should().Be(frozenAPositionBefore);
        frozenB.Body.Position3d.Should().Be(frozenBPositionBefore);
        frozenA.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        frozenB.Body.LinearVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void CalculateImpulse_WithZeroRestitution_ShouldDampenWithoutReversingLinearVelocity()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        left.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero);
        right.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero);
        Push(left.Body, 60);
        Push(right.Body, -60);
        CollisionPair pair = CreateDetectedPair(scenario, left.Collider, right.Collider);
        Fixed64 leftVelocityBefore = left.Body.LinearVelocity.X;
        Fixed64 rightVelocityBefore = right.Body.LinearVelocity.X;

        CollisionResponse.CalculateImpulse(pair);

        AssertNear(left.Body.LinearVelocity.X, Fixed64.Zero);
        AssertNear(right.Body.LinearVelocity.X, Fixed64.Zero);
        left.Body.LinearVelocity.X.Should().BeLessThan(leftVelocityBefore);
        right.Body.LinearVelocity.X.Should().BeGreaterThan(rightVelocityBefore);
    }

    [Fact]
    public void CalculateImpulse_WithCompoundPartMaterial_ShouldUsePartRestitution()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        PhysicsMaterial zeroOwner = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero);
        PhysicsMaterial bouncyPart = new(
            Fixed64.One,
            Fixed64.One,
            Fixed64.One,
            restitutionCombine: PhysicsMaterialCombine.Maximum);
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero, bouncyPart));
        ScenarioBody<LSCompoundCollider> wall = scenario.CreateBody(
            compound,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            immovable: true);
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            preventAngularForces: true);
        wall.Collider.Material = zeroOwner;
        mover.Collider.Material = zeroOwner;
        Push(mover.Body, -60);
        CollisionPair pair = CreateDetectedPair(scenario, wall.Collider, mover.Collider);

        CollisionResponse.CalculateImpulse(pair);

        mover.Body.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void CalculateImpulse_WithTangentialVelocity_ShouldApplyDeterministicFrictionImpulse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> wall = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            immovable: true);
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(
            Fixed64.FromFraction(3, 4),
            Fixed64.Zero,
            Fixed64.Zero));
        mover.Body.AddLinearImpulse(new Vector3d((Fixed64)(-60), Fixed64.Zero, (Fixed64)30));
        CollisionPair pair = CreateDetectedPair(scenario, wall.Collider, mover.Collider);
        Fixed64 tangentialSpeedBefore = mover.Body.LinearVelocity.Z.Abs();

        CollisionResponse.CalculateImpulse(pair);

        mover.Body.LinearVelocity.Z.Abs().Should().BeLessThan(tangentialSpeedBefore);
    }

    [Fact]
    public void CalculateImpulse_WithHighStaticAndZeroDynamicFriction_ShouldHoldTangentialMotion()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> wall = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            immovable: true);
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(
            PhysicsScenarioBuilder.Vector(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            preventAngularForces: true);
        PhysicsMaterial stickyStatic = new((Fixed64)100, Fixed64.Zero, Fixed64.Zero);
        wall.Collider.Material = stickyStatic;
        mover.Collider.Material = stickyStatic;
        mover.Body.AddLinearImpulse(new Vector3d((Fixed64)(-60), Fixed64.Zero, (Fixed64)3));
        CollisionPair pair = CreateDetectedPair(scenario, wall.Collider, mover.Collider);

        CollisionResponse.CalculateImpulse(pair);

        mover.Body.LinearVelocity.Z.Abs().Should().BeLessThan(Tolerance);
    }

    [Fact]
    public void CalculateImpulse_WhenStaticLimitIsExceeded_ShouldUseDynamicFriction()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> wall = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            immovable: true);
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(
            PhysicsScenarioBuilder.Vector(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            preventAngularForces: true);
        PhysicsMaterial sliding = new(Fixed64.Half, Fixed64.Half, Fixed64.Zero);
        wall.Collider.Material = sliding;
        mover.Collider.Material = sliding;
        mover.Body.AddLinearImpulse(new Vector3d((Fixed64)(-60), Fixed64.Zero, (Fixed64)90));
        CollisionPair pair = CreateDetectedPair(scenario, wall.Collider, mover.Collider);
        Fixed64 tangentialSpeedBefore = mover.Body.LinearVelocity.Z.Abs();

        CollisionResponse.CalculateImpulse(pair);

        mover.Body.LinearVelocity.Z.Abs().Should().BeLessThan(tangentialSpeedBefore);
        mover.Body.LinearVelocity.Z.Abs().Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void CalculateImpulse_WithSlopedContactNormal_ShouldResolveAgainstContactPlane()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Vector3d normal = new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero).Normalized;
        Vector3d tangent = new Vector3d(-normal.Y, normal.X, Fixed64.Zero).Normalized;
        ScenarioBody<LSSphereCollider> slope = scenario.CreateSphere(
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            immovable: true);
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(normal * Fixed64.FromFraction(3, 4));
        CollisionPair pair = scenario.CreatePair(slope.Collider, mover.Collider);
        mover.Body.AddLinearImpulse((-normal * (Fixed64)60) + (tangent * (Fixed64)30));
        pair.Manifold.SetContact(pair.ColliderA.Center, pair.ColliderB.Center, Fixed64.FromFraction(1, 10), normal);
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
            Fixed64.FromFraction(3, 4),
            Fixed64.FromFraction(1, 4),
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
            PhysicsScenarioBuilder.Vector(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero));
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
        PhysicsMaterial rough = new((Fixed64)2, Fixed64.One, Fixed64.Half);
        PhysicsMaterial slick = new(Fixed64.Half, Fixed64.FromFraction(1, 4), Fixed64.Zero);
        ScenarioBody<LSCuboidCollider> wall = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            immovable: true);
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(
            Fixed64.FromFraction(3, 4),
            Fixed64.Zero,
            Fixed64.Zero));
        wall.Collider.Material = rough;
        sphere.Collider.Material = slick;
        CollisionPair singleContactPair = CreateDetectedPair(scenario, wall.Collider, sphere.Collider);
        sphere.Body.AddLinearImpulse(new Vector3d((Fixed64)(-60), Fixed64.Zero, (Fixed64)30));
        CollisionResponse.CalculateImpulse(singleContactPair);

        ScenarioBody<LSCuboidCollider> floor = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(4, 0, 0),
            immovable: true);
        ScenarioBody<LSCuboidCollider> box = scenario.CreateCuboid(new Vector3d((Fixed64)4, Fixed64.FromFraction(3, 4), Fixed64.Zero));
        floor.Collider.Material = rough;
        box.Collider.Material = slick;
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

    [Fact]
    public void CalculateImpulse_WithPositionCorrectionCrossing3DPartitions_ShouldNotAllocateAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> moving = scenario.CreateSphere(
            Vector3d.Zero,
            preventAngularForces: true);
        ScenarioBody<LSSphereCollider> wall = scenario.CreateSphere(
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        CollisionPair pair = scenario.CreatePair(moving.Collider, wall.Collider);
        Fixed64 depth = scenario.Context.VoxelSize + CollisionResponse.PenetrationSlop;

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            () =>
            {
                moving.Body.ApplyCollisionLinearVelocityDelta(-moving.Body.LinearVelocity);
                moving.Body.ApplyCollisionAngularVelocityDelta(-moving.Body.AngularVelocity);
                pair.Manifold.SetContact(moving.Collider.Center, moving.Collider.Center, depth, Vector3d.Right);
                CollisionResponse.CalculateImpulse(pair);
                moving.Collider.Simulate();
            },
            warmupIterations: 4,
            stabilizationIterations: 2,
            measurementIterations: 4);

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

    private static void Push(SolidBody body, int xImpulse)
    {
        body.AddLinearImpulse(new Vector3d((Fixed64)xImpulse, Fixed64.Zero, Fixed64.Zero));
    }

    private static ResponseState RunDeterministicResponseSequence()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        left.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Half);
        right.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Half);
        Push(left.Body, 60);
        Push(right.Body, -30);

        for (int i = 0; i < 8; i++)
        {
            CollisionPair pair = scenario.CreatePair(left.Collider, right.Collider);
            pair.Manifold.SetContact(left.Collider.Center, right.Collider.Center, Fixed64.FromFraction(1, 20), Vector3d.Right);
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
        => AllocationTestHelper.MeasureSinglePass(action);

    private readonly record struct ResponseState(
        Vector3d LeftPosition,
        Vector3d RightPosition,
        Vector3d LeftVelocity,
        Vector3d RightVelocity,
        Vector3d LeftAngularVelocity,
        Vector3d RightAngularVelocity);
}
