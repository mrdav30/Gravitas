using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using SwiftCollections.Diagnostics;
using System;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CollisionResponseInvariantTests
{
    private static readonly Fixed64 Tolerance = Fixed64.FromFraction(1, 1_000_000);
    private static readonly Fixed64 StandardSpeed = (Fixed64)2;
    private static readonly Fixed64 LowSpeed = Fixed64.FromFraction(1, 10);

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
        Push(left.Body, StandardSpeed);
        Push(right.Body, -StandardSpeed);
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
        Push(left.Body, StandardSpeed);
        Push(right.Body, -StandardSpeed);
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

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void CalculateImpulse_WithUnrepresentableLeverArm_ShouldRejectContactAtomically(
        bool loggingEnabled)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left =
            scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> right =
            scenario.CreateSphere(Vector3d.Right);
        CollisionPair pair =
            scenario.CreatePair(left.Collider, right.Collider);
        pair.Manifold.SetContact(
            new ContactAnchor(
                new Vector3d(
                    Fixed64.MaxValue,
                    Fixed64.Zero,
                    Fixed64.Zero),
                Vector3d.Right),
            ContactAnchor.FromWorldPoint(right.Collider.Center),
            Fixed64.Half,
            Vector3d.Right);
        Vector3d leftVelocity = left.Body.LinearVelocity;
        Vector3d rightVelocity = right.Body.LinearVelocity;
        string? loggedMessage = null;
        var originalLevel = GravitasLogger.MinimumLevel;
        Action<DiagnosticLevel, string, string> originalHandler =
            GravitasLogger.LogHandler;

        try
        {
            GravitasLogger.MinimumLevel = loggingEnabled
                ? DiagnosticLevel.Error
                : DiagnosticLevel.None;
            GravitasLogger.LogHandler =
                (_, message, _) => loggedMessage = message;

            CollisionResponse.CalculateImpulse(pair);
        }
        finally
        {
            GravitasLogger.MinimumLevel = originalLevel;
            GravitasLogger.LogHandler = originalHandler;
        }

        left.Body.LinearVelocity.Should().Be(leftVelocity);
        right.Body.LinearVelocity.Should().Be(rightVelocity);
        if (loggingEnabled)
        {
            loggedMessage.Should().Contain(
                "cannot be rebased onto its response centers");
        }
        else
        {
            loggedMessage.Should().BeNull();
        }
    }

    [Fact]
    public void CalculateImpulse_WithZeroContactNormal_ShouldUseColliderCenterFallback()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        Push(left.Body, StandardSpeed);
        Push(right.Body, -StandardSpeed);
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
        Push(left.Body, StandardSpeed);
        Push(right.Body, -StandardSpeed);
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
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(-2, 0, 0));
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(2, 0, 0));
        Push(left.Body, StandardSpeed);
        Push(right.Body, -StandardSpeed);
        left.Body.SetPosition(Vector3d.Zero);
        right.Body.SetPosition(Vector3d.Zero);
        left.Collider.RebuildRuntimeShapeOnly();
        right.Collider.RebuildRuntimeShapeOnly();
        CollisionPair pair = scenario.CreatePair(left.Collider, right.Collider);
        pair.ColliderA.Center.Should().Be(pair.ColliderB.Center);
        pair.Manifold.SetContact(left.Collider.Center, right.Collider.Center, Fixed64.FromFraction(1, 4), Vector3d.Zero);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        contact.Normal.Should().Be(Vector3d.Zero);
        Vector3d leftPositionBefore = left.Body.Position3d;
        Vector3d rightPositionBefore = right.Body.Position3d;
        Vector3d leftVelocityBefore = left.Body.LinearVelocity;
        Vector3d rightVelocityBefore = right.Body.LinearVelocity;

        CollisionResponse.CalculateImpulse(pair);

        left.Body.Position3d.Should().Be(leftPositionBefore);
        right.Body.Position3d.Should().Be(rightPositionBefore);
        left.Body.LinearVelocity.Should().Be(leftVelocityBefore);
        right.Body.LinearVelocity.Should().Be(rightVelocityBefore);
        pair.TryGetWarmStartImpulse(contact.ContactId, out _).Should().BeFalse();
    }

    [Fact]
    public void CalculateImpulse_BelowRestitutionThreshold_ShouldRemoveClosingVelocityWithoutBounce()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        left.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        right.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        Push(left.Body, LowSpeed);
        Push(right.Body, -LowSpeed);
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
        Push(left.Body, Fixed64.One);
        Push(right.Body, -Fixed64.One);
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
        Push(left.Body, LowSpeed);
        Push(right.Body, -LowSpeed);
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
        Push(heavy.Body, StandardSpeed);
        Push(light.Body, -StandardSpeed);
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
        Push(movable.Body, -StandardSpeed);
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
    public void CalculateImpulse_WithBodiesConstrainedAlongContactNormal_ShouldSkipCorrectionAndImpulse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        BodyFreezeAxes3D constraints = BodyFreezeAxes3D.PositionX | BodyFreezeAxes3D.Rotation;
        left.Body.FreezeAxes = constraints;
        right.Body.FreezeAxes = constraints;
        CollisionPair pair = scenario.CreatePair(left.Collider, right.Collider);
        pair.Manifold.SetContact(
            left.Collider.Center,
            right.Collider.Center,
            Fixed64.FromFraction(1, 10),
            Vector3d.Right);
        Vector3d leftPosition = left.Body.Position3d;
        Vector3d rightPosition = right.Body.Position3d;

        CollisionResponse.CalculateImpulse(pair);

        left.Body.Position3d.Should().Be(leftPosition);
        right.Body.Position3d.Should().Be(rightPosition);
        left.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        right.Body.LinearVelocity.Should().Be(Vector3d.Zero);
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
        Push(left.Body, StandardSpeed);
        Push(right.Body, -StandardSpeed);
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
        Push(movable.Body, -StandardSpeed);
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
        Push(solid.Body, -StandardSpeed);
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
        Push(solid.Body, -StandardSpeed);
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
        Push(left.Body, StandardSpeed);
        Push(right.Body, -StandardSpeed);
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
        Push(mover.Body, -StandardSpeed);
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
        Push(mover.Body, new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.One));
        CollisionPair pair = CreateDetectedPair(scenario, wall.Collider, mover.Collider);
        Fixed64 tangentialSpeedBefore = mover.Body.LinearVelocity.Z.Abs();

        CollisionResponse.CalculateImpulse(pair);

        mover.Body.LinearVelocity.Z.Abs().Should().BeLessThan(tangentialSpeedBefore);
    }

    [Fact]
    public void CalculateImpulse_WithPositionFrozenDynamicBody_ShouldApplyOffCenterFrictionAsRotationOnly()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> angularOnly = scenario.CreateCuboid(Vector3d.Zero);
        ScenarioBody<LSCuboidCollider> moving = scenario.CreateCuboid(Vector3d.Right * (Fixed64)2);
        angularOnly.Body.FreezeAxes = BodyFreezeAxes3D.Position;
        var material = new PhysicsMaterial(Fixed64.One, Fixed64.One, Fixed64.Zero);
        angularOnly.Collider.Material = material;
        moving.Collider.Material = material;
        moving.Body.ApplyCollisionLinearVelocityDelta(
            new Vector3d((Fixed64)(-4), Fixed64.Zero, (Fixed64)2));
        Fixed64 tangentialSpeedBefore = moving.Body.LinearVelocity.Z.Abs();
        CollisionPair pair = scenario.CreatePair(angularOnly.Collider, moving.Collider);
        pair.Manifold.SetContact(Vector3d.Right, Vector3d.Right, Fixed64.Zero, Vector3d.Right);

        CollisionResponse.CalculateImpulse(pair);

        angularOnly.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        angularOnly.Body.AngularVelocity.Should().NotBe(Vector3d.Zero);
        moving.Body.LinearVelocity.Z.Abs().Should().BeLessThan(tangentialSpeedBefore);
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
        Push(mover.Body, new Vector3d((Fixed64)(-2), Fixed64.Zero, LowSpeed));
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
        Push(mover.Body, new Vector3d((Fixed64)(-2), Fixed64.Zero, (Fixed64)3));
        CollisionPair pair = CreateDetectedPair(scenario, wall.Collider, mover.Collider);
        Fixed64 tangentialSpeedBefore = mover.Body.LinearVelocity.Z.Abs();

        CollisionResponse.CalculateImpulse(pair);

        mover.Body.LinearVelocity.Z.Abs().Should().BeLessThan(tangentialSpeedBefore);
        mover.Body.LinearVelocity.Z.Abs().Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void CalculateImpulse_WithFrictionlessContact_ShouldPreserveTangentialVelocity()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> wall = scenario.CreateCuboid(Vector3d.Zero, immovable: true);
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            preventAngularForces: true);
        wall.Collider.Material = PhysicsMaterial.Frictionless;
        mover.Collider.Material = PhysicsMaterial.Frictionless;
        Push(mover.Body, new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.One));
        CollisionPair pair = CreateDetectedPair(scenario, wall.Collider, mover.Collider);
        Fixed64 tangentialVelocity = mover.Body.LinearVelocity.Z;

        CollisionResponse.CalculateImpulse(pair);

        mover.Body.LinearVelocity.X.Should().Be(Fixed64.Zero);
        mover.Body.LinearVelocity.Z.Should().Be(tangentialVelocity);
    }

    [Fact]
    public void CalculateImpulse_WithKinematicTangentialMotionAndQuantizedTargetMass_ShouldSkipUndefinedFrictionImpulse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> driver = scenario.CreateSphere(
            Vector3d.Zero,
            isKinematic: true);
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            mass: Fixed64.MaxValue,
            preventAngularForces: true);
        driver.Body.Agent.Transform.LocalPosition = new Vector3d(
            Fixed64.FromFraction(1, 10),
            Fixed64.Zero,
            Fixed64.FromFraction(1, 20));
        scenario.Context.AdvanceLateSimulateToken();
        driver.Body.EnsureContinuousCollisionFramePrepared(scenario.Context.LateSimulateToken);
        driver.Body.SampleContinuousCollisionLinearVelocity(Fixed64.One)
            .MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
        CollisionPair pair = CreateDetectedPair(scenario, driver.Collider, target.Collider);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(contact.ContactId, contact.Normal, Fixed64.One, Fixed64.Zero);

        CollisionResponse.CalculateImpulse(pair);

        driver.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        target.Body.LinearVelocity.Should().Be(Vector3d.Zero);
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
        Push(mover.Body, (-normal * (Fixed64)2) + tangent);
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
        Push(sphere.Body, -StandardSpeed);
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
        Push(box.Body, Vector3d.Down * (Fixed64)2);
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
        Push(sphere.Body, new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.One));
        CollisionResponse.CalculateImpulse(singleContactPair);

        ScenarioBody<LSCuboidCollider> floor = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(4, 0, 0),
            immovable: true);
        ScenarioBody<LSCuboidCollider> box = scenario.CreateCuboid(new Vector3d((Fixed64)4, Fixed64.FromFraction(3, 4), Fixed64.Zero));
        floor.Collider.Material = rough;
        box.Collider.Material = slick;
        CollisionPair facePair = CreateDetectedPair(scenario, floor.Collider, box.Collider);
        Push(box.Body, Vector3d.Down * (Fixed64)2);
        CollisionResponse.CalculateImpulse(facePair);

        Push(sphere.Body, new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.One));
        Push(box.Body, Vector3d.Down * (Fixed64)2);

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

    private static void Push(SolidBody body, Fixed64 xVelocity)
    {
        Push(body, new Vector3d(xVelocity, Fixed64.Zero, Fixed64.Zero));
    }

    private static void Push(SolidBody body, Vector3d velocity) =>
        body.AddLinearImpulse(velocity * body.Mass);

    private static ResponseState RunDeterministicResponseSequence()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        left.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Half);
        right.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Half);
        Push(left.Body, StandardSpeed);
        Push(right.Body, -Fixed64.One);

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
