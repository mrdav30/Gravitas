using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Diagnostics;
using Gravitas.Materials;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using System;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedResponseTests
{
    [Fact]
    public void Simulate_WithDynamic3DOnStatic2DSlab_ShouldResolveVerticallyAndNotify()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero));
        LSCollider2D platform = CreateBodylessBox2D(context, Vector2d.Zero, new Vector2d((Fixed64)4, (Fixed64)4));
        int entered3D = 0;
        int stayed3D = 0;
        int entered2D = 0;
        int stayed2D = 0;
        body3D.Collider.OnMixedContactEnter += other =>
        {
            other.Should().BeSameAs(platform);
            entered3D++;
        };
        body3D.Collider.OnMixedContact += _ => stayed3D++;
        platform.OnMixedContactEnter += other =>
        {
            other.Should().BeSameAs(body3D.Collider);
            entered2D++;
        };
        platform.OnMixedContact += other =>
        {
            other.Should().BeSameAs(body3D.Collider);
            stayed2D++;
        };

        Step(context);
        Fixed64 resolvedY = body3D.Body.Position3d.Y;
        Step(context);

        resolvedY.Should().BeGreaterThan(Fixed64.FromFraction(3, 4));
        body3D.Body.Position3d.Y.Should().BeGreaterThanOrEqualTo(resolvedY);
        platform.Center.Should().Be(Vector2d.Zero);
        entered3D.Should().Be(1);
        entered2D.Should().Be(1);
        stayed3D.Should().Be(2);
        stayed2D.Should().Be(2);
        context.MixedCollisions.ActivePairCount.Should().Be(1);
    }

    [Fact]
    public void Simulate_WithDynamic3DPushingDynamic2D_ShouldMoveBothPlanarly()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);

        Step(context);

        body3D.Body.Position3d.X.Should().BeLessThan(-Fixed64.FromFraction(1, 4));
        body2D.Position.X.Should().BeGreaterThan(Fixed64.Zero);
        body2D.Agent.Transform.LocalPosition.Y.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void LateSimulate_ShouldRefreshMovedMixedCollidersAndDistributeContactsAfterIntegration()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.FromFraction(5, 4), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero, immovable: true);
        Vector3d startPosition = body3D.Body.Position3d;
        int entered = 0;
        body3D.Collider.OnMixedContactEnter += other =>
        {
            other.Should().BeSameAs(body2D.Collider);
            entered++;
        };

        body3D.Body.AddForce(new Vector3d((Fixed64)16, Fixed64.Zero, Fixed64.Zero));
        context.Simulate();

        body3D.Body.Position3d.Should().Be(startPosition);
        context.MixedCollisions.ActivePairCount.Should().Be(0);
        entered.Should().Be(0);

        context.LateSimulate();

        body3D.Body.Position3d.X.Should().BeGreaterThan(startPosition.X);
        context.MixedCollisions.ActivePairCount.Should().Be(1);
        entered.Should().Be(1);
    }

    [Fact]
    public void Resolve_WithVerticalOnlyMixedImpulse_ShouldNotTranslateOrSpin2DParticipant()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero));
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        var contact = new MixedContact(
            new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Half),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half),
            Vector3d.Up,
            Fixed64.FromFraction(1, 10));
        body3D.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Up);

        CollisionResponseMixed.Resolve(pair, contact);

        body2D.Position.Should().Be(Vector2d.Zero);
        body2D.LinearVelocity.Should().Be(Vector2d.Zero);
        body2D.AngularVelocity.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithPlanarOffsetMixedImpulse_ShouldSpin2DParticipantAroundCom()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Half));
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        var contact = new MixedContact(
            new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Half),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half),
            Vector3d.Right,
            Fixed64.FromFraction(1, 10));
        body3D.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right);

        CollisionResponseMixed.Resolve(pair, contact);

        body2D.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        body2D.LinearVelocity.Y.Should().Be(Fixed64.Zero);
        body2D.AngularVelocity.Should().BeLessThan(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithConfiguredRestitutionThreshold_ShouldControlMixedBounce()
    {
        Fixed64 highThresholdVelocity = Resolve3DPlanarVelocityAfterMixedResponse(
            threshold: (Fixed64)5,
            initialVelocity: (Fixed64)4);
        Fixed64 zeroThresholdVelocity = Resolve3DPlanarVelocityAfterMixedResponse(
            threshold: Fixed64.Zero,
            initialVelocity: (Fixed64)4);

        highThresholdVelocity.Should().BeGreaterThan(zeroThresholdVelocity);
    }

    [Fact]
    public void Resolve_ShouldCombineColliderMaterialsForMixedRestitution()
    {
        PhysicsMaterial zeroMinimum = new(
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero,
            restitutionCombine: PhysicsMaterialCombine.Minimum);
        PhysicsMaterial oneMinimum = new(
            Fixed64.One,
            Fixed64.One,
            Fixed64.One,
            restitutionCombine: PhysicsMaterialCombine.Minimum);
        PhysicsMaterial oneMaximum = new(
            Fixed64.One,
            Fixed64.One,
            Fixed64.One,
            restitutionCombine: PhysicsMaterialCombine.Maximum);

        Fixed64 minimumPolicyVelocity = Resolve3DPlanarVelocityAfterMixedResponse(
            threshold: Fixed64.Zero,
            initialVelocity: (Fixed64)4,
            material3D: zeroMinimum,
            material2D: oneMinimum);
        Fixed64 maximumPolicyVelocity = Resolve3DPlanarVelocityAfterMixedResponse(
            threshold: Fixed64.Zero,
            initialVelocity: (Fixed64)4,
            material3D: zeroMinimum,
            material2D: oneMaximum);

        maximumPolicyVelocity.Should().BeLessThan(minimumPolicyVelocity);
    }

    [Fact]
    public void Resolve_WithDefaultMixedContact_ShouldSkipResponse()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        body3D.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)4);
        body2D.ApplyCollisionLinearVelocityDelta(-Vector2d.Right * (Fixed64)4);
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);

        bool appliedImpulse = CollisionResponseMixed.Resolve(pair, default);

        appliedImpulse.Should().BeFalse();
        body3D.Body.Position3d.Should().Be(new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        body2D.Position.Should().Be(Vector2d.Zero);
        body3D.Body.LinearVelocity.X.Should().Be((Fixed64)4);
        body2D.LinearVelocity.X.Should().Be(-(Fixed64)4);
    }

    [Fact]
    public void Resolve_WithTriggerPairOrSlopOnlyContact_ShouldSkipResponse()
    {
        using GravitasWorldContext triggerContext = CreateMixedContext();
        LSSphereCollider trigger3D = CreateBodylessSphere3D(
            triggerContext,
            new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        trigger3D.IsTrigger = true;
        SolidBody2D body2D = CreateCircle2D(triggerContext, Vector2d.Zero);
        body2D.ApplyCollisionLinearVelocityDelta(-Vector2d.Right * (Fixed64)4);
        var triggerPair = new CollisionPairMixed(trigger3D, body2D.Collider);
        var triggerContact = new MixedContact(
            trigger3D.Center,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right,
            Fixed64.One);

        CollisionResponseMixed.Resolve(triggerPair, triggerContact).Should().BeFalse();
        body2D.LinearVelocity.X.Should().Be(-(Fixed64)4);

        using GravitasWorldContext slopContext = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            slopContext,
            new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        SolidBody2D slopBody2D = CreateCircle2D(slopContext, Vector2d.Zero);
        var slopPair = new CollisionPairMixed(body3D.Collider, slopBody2D.Collider);
        var slopContact = new MixedContact(
            body3D.Collider.Center,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right,
            CollisionResponseMixed.PenetrationSlop * Fixed64.Half);

        CollisionResponseMixed.Resolve(slopPair, slopContact).Should().BeFalse();
        body3D.Body.Position3d.Should().Be(new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        slopBody2D.Position.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void Resolve_WithMixedFrictionMaterials_ShouldApplyOnlyWhenTangentialMotionExists()
    {
        Fixed64 frictionlessTangentialVelocity = Resolve2DTangentialVelocityAfterMixedResponse(
            PhysicsMaterial.Frictionless,
            PhysicsMaterial.Frictionless);
        Fixed64 staticFrictionTangentialVelocity = Resolve2DTangentialVelocityAfterMixedResponse(
            new PhysicsMaterial((Fixed64)3, Fixed64.One, Fixed64.Zero),
            new PhysicsMaterial((Fixed64)3, Fixed64.One, Fixed64.Zero));
        Fixed64 dynamicFrictionTangentialVelocity = Resolve2DTangentialVelocityAfterMixedResponse(
            new PhysicsMaterial(Fixed64.Half, Fixed64.Half, Fixed64.Zero),
            new PhysicsMaterial(Fixed64.Half, Fixed64.Half, Fixed64.Zero));

        frictionlessTangentialVelocity.Should().Be(Fixed64.Zero);
        staticFrictionTangentialVelocity.Should().BeGreaterThan(Fixed64.Zero);
        dynamicFrictionTangentialVelocity.Should().BeGreaterThan(Fixed64.Zero);
        dynamicFrictionTangentialVelocity.Should().BeLessThan(staticFrictionTangentialVelocity);
    }

    [Fact]
    public void Resolve_WithMixedContactMaterialOverride_ShouldUseOverrideRestitution()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Settings.RestitutionVelocityThreshold = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        body3D.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero);
        body2D.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero);
        body3D.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)4);
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        var contact = new MixedContact(
            body3D.Collider.Center,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right,
            Fixed64.FromFraction(1, 10),
            PhysicsMaterialTestHelper.WithRestitution(Fixed64.One),
            PhysicsMaterialTestHelper.WithRestitution(Fixed64.One));

        bool appliedImpulse = CollisionResponseMixed.Resolve(
            pair,
            contact,
            iteration: 0,
            iterationLimit: 1,
            applyPositionCorrection: false);

        appliedImpulse.Should().BeTrue();
        body3D.Body.LinearVelocity.X.Should().Be(Fixed64.Zero);
        body2D.LinearVelocity.X.Should().Be((Fixed64)4);
    }

    [Fact]
    public void Resolve_WithPlanarVerticalMixedImpulse_ShouldConstrain2DVerticalAndMove3D()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.Half, -Fixed64.FromFraction(1, 4), Fixed64.Half));
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        var contact = new MixedContact(
            new Vector3d(-Fixed64.Half, -Fixed64.FromFraction(1, 4), Fixed64.Half),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half),
            new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero).Normalized,
            Fixed64.FromFraction(1, 5));
        body3D.Body.ApplyCollisionLinearVelocityDelta(new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero));

        CollisionResponseMixed.Resolve(pair, contact);

        body3D.Body.Position3d.Y.Should().BeLessThan(-Fixed64.FromFraction(1, 4));
        body2D.Agent.Transform.LocalPosition.Y.Should().Be(Fixed64.Zero);
        body2D.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        body2D.LinearVelocity.Y.Should().Be(Fixed64.Zero);
        body2D.AngularVelocity.Should().BeLessThan(Fixed64.Zero);
    }

    [Fact]
    public void Simulate_WithMixedResponseDiagnostics_ShouldRecordSinglePairIterationMetadata()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Diagnostics.Enable(eventCapacity: 16, drawCommandCapacity: 0);
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        _ = CreateCircle2D(context, Vector2d.Zero);
        body3D.Body.AddLinearImpulse(Vector3d.Right);

        Step(context);

        GravitasMixedResponseImpulseDiagnosticView impulse = FindFirstMixedImpulse(context);
        impulse.Iteration.Should().Be(0);
        impulse.IterationLimit.Should().Be(1);
    }

    [Fact]
    public void Simulate_WithConnectedMixedPairs_ShouldSolveDedicatedMixedIslandAndReportIterationCap()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Settings.DiscreteSolverIterations = 3;
        context.Diagnostics.Enable(eventCapacity: 64, drawCommandCapacity: 0);
        ScenarioBody<LSSphereCollider> left3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> right3D = CreateSphere3D(
            context,
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        _ = CreateCircle2D(context, Vector2d.Zero);
        left3D.Body.AddLinearImpulse(Vector3d.Right);
        right3D.Body.AddLinearImpulse(Vector3d.Left);

        Step(context);

        GravitasMixedResponseIslandDiagnosticView island = FindFirstMixedIsland(context);
        island.ConstraintCount.Should().Be(2);
        island.IterationCount.Should().Be(3);
        island.ReachedIterationLimit.Should().BeTrue();
        FindMaxMixedImpulseIterationLimit(context).Should().Be(3);
    }

    [Fact]
    public void Simulate_WithOneDynamic3DAndTwoBodyless2DContacts_ShouldUse3DRootForMixedIsland()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Settings.DiscreteSolverIterations = 2;
        context.Diagnostics.Enable(eventCapacity: 64, drawCommandCapacity: 0);
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        _ = CreateBodylessCircle2D(context, new Vector2d(-Fixed64.FromFraction(1, 4), Fixed64.Zero));
        _ = CreateBodylessCircle2D(context, new Vector2d(Fixed64.FromFraction(1, 4), Fixed64.Zero));
        body3D.Body.AddLinearImpulse(Vector3d.Right);

        Step(context);

        GravitasMixedResponseIslandDiagnosticView island = FindFirstMixedIsland(context);
        island.RootKey.Should().Be(body3D.Body.DynamicId << 1);
        island.ConstraintCount.Should().Be(2);
        island.IterationCount.Should().Be(2);
    }

    [Fact]
    public void Simulate_WithTwoBodyless3DContactsAndOneDynamic2D_ShouldUse2DRootForMixedIsland()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Settings.DiscreteSolverIterations = 2;
        context.Diagnostics.Enable(eventCapacity: 64, drawCommandCapacity: 0);
        LSCollider left3D = CreateBodylessSphere3D(context, new Vector3d(-Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero));
        LSCollider right3D = CreateBodylessSphere3D(context, new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        body2D.ApplyCollisionLinearVelocityDelta(Vector2d.Right);

        Step(context);

        GravitasMixedResponseIslandDiagnosticView island = FindFirstMixedIsland(context);
        island.RootKey.Should().Be((body2D.DynamicId << 1) | 1);
        island.ConstraintCount.Should().Be(2);
        island.IterationCount.Should().Be(2);
        left3D.Center.X.Should().Be(-Fixed64.FromFraction(1, 4));
        right3D.Center.X.Should().Be(Fixed64.FromFraction(1, 4));
    }

    [Fact]
    public void Simulate_WithSleepingMixedLinkInAwakeIsland_ShouldWakeConnectedSleepingBodies()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> awake3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> sleeping3D = CreateSphere3D(
            context,
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D bridge2D = CreateCircle2D(context, Vector2d.Zero);

        Step(context);
        context.MixedCollisions.ActivePairCount.Should().Be(2);

        awake3D.Body.SetPosition(new Vector3d(-Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        sleeping3D.Body.SetPosition(new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        bridge2D.SetPosition(Vector2d.Zero);
        sleeping3D.Body.Sleep();
        bridge2D.Sleep();
        awake3D.Body.AddLinearImpulse(Vector3d.Right);

        Step(context);

        bridge2D.IsSleeping.Should().BeFalse();
        sleeping3D.Body.IsSleeping.Should().BeFalse();
    }

    [Fact]
    public void Simulate_WithAllSleepingMixedIsland_ShouldRetainContactsWithoutResponse()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Diagnostics.Enable(eventCapacity: 64, drawCommandCapacity: 0);
        ScenarioBody<LSSphereCollider> left3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> right3D = CreateSphere3D(
            context,
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D bridge2D = CreateCircle2D(context, Vector2d.Zero);

        Step(context);
        context.MixedCollisions.ActivePairCount.Should().Be(2);

        left3D.Body.Sleep();
        right3D.Body.Sleep();
        bridge2D.Sleep();
        context.Diagnostics.Clear();
        Step(context);

        context.MixedCollisions.ActivePairCount.Should().Be(2);
        left3D.Body.IsSleeping.Should().BeTrue();
        right3D.Body.IsSleeping.Should().BeTrue();
        bridge2D.IsSleeping.Should().BeTrue();
        CountMixedEvents(context).Should().Be(0);
    }

    [Fact]
    public void Simulate_WithAwakeAndSleepingMixedIslands_ShouldSolveOnlyAwakeRoot()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Settings.DiscreteSolverIterations = 2;
        context.Diagnostics.Enable(eventCapacity: 64, drawCommandCapacity: 0);
        ScenarioBody<LSSphereCollider> leftAwake3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.FromFraction(15, 4), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> rightAwake3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.FromFraction(9, 4), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D awakeBridge2D = CreateCircle2D(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));
        ScenarioBody<LSSphereCollider> sleeping3D = CreateSphere3D(
            context,
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));
        SolidBody2D sleeping2D = CreateCircle2D(context, new Vector2d((Fixed64)3, Fixed64.Zero));

        Step(context);
        context.MixedCollisions.ActivePairCount.Should().Be(3);

        leftAwake3D.Body.SetPosition(new Vector3d(-Fixed64.FromFraction(15, 4), Fixed64.Zero, Fixed64.Zero));
        rightAwake3D.Body.SetPosition(new Vector3d(-Fixed64.FromFraction(9, 4), Fixed64.Zero, Fixed64.Zero));
        awakeBridge2D.SetPosition(new Vector2d((Fixed64)(-3), Fixed64.Zero));
        sleeping3D.Body.Sleep();
        sleeping2D.Sleep();
        leftAwake3D.Body.AddLinearImpulse(Vector3d.Right);
        rightAwake3D.Body.AddLinearImpulse(Vector3d.Left);
        context.Diagnostics.Clear();
        Step(context);

        sleeping3D.Body.IsSleeping.Should().BeTrue();
        sleeping2D.IsSleeping.Should().BeTrue();
        CountMixedIslandEvents(context).Should().Be(1);
        GravitasMixedResponseIslandDiagnosticView island = FindFirstMixedIsland(context);
        island.RootKey.Should().Be(leftAwake3D.Body.DynamicId << 1);
        island.ConstraintCount.Should().Be(2);
        island.IterationCount.Should().Be(2);
    }

    [Fact]
    public void Simulate_WhenExistingMixedPairFallsAsleep_ShouldRetainRestingContactWithoutExit()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero));
        LSCollider2D platform = CreateBodylessBox2D(context, Vector2d.Zero, new Vector2d((Fixed64)4, (Fixed64)4));
        int exited3D = 0;
        int exited2D = 0;
        body3D.Collider.OnMixedContactExit += other =>
        {
            other.Should().BeSameAs(platform);
            exited3D++;
        };
        platform.OnMixedContactExit += other =>
        {
            other.Should().BeSameAs(body3D.Collider);
            exited2D++;
        };

        Step(context);
        Vector3d restingPosition = body3D.Body.Position3d;
        body3D.Body.Sleep();
        Step(context);

        body3D.Body.IsSleeping.Should().BeTrue();
        body3D.Body.Position3d.Should().Be(restingPosition);
        exited3D.Should().Be(0);
        exited2D.Should().Be(0);
        context.MixedCollisions.ActivePairCount.Should().Be(1);
    }

    [Fact]
    public void Simulate_WithRuntimeModeBoth_ShouldNotCreateMixedPairsOrDiagnostics()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Both;
        context.Diagnostics.Enable(eventCapacity: 16, drawCommandCapacity: 0);
        _ = CreateSphere3D(context, Vector3d.Zero);
        _ = CreateCircle2D(context, Vector2d.Zero);

        Step(context);

        context.MixedCollisions.ActivePairCount.Should().Be(0);
        CountMixedEvents(context).Should().Be(0);
    }

    [Fact]
    public void Simulate_WithKinematic3DAgainstDynamic2D_ShouldOnlyMove2DParticipant()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero),
            isKinematic: true);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);

        Step(context);

        body3D.Body.Position3d.X.Should().Be(-Fixed64.FromFraction(1, 4));
        body2D.Position.X.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithBodyless3DAndDynamic2D_ShouldApplyPlanarImpulseOnlyTo2DParticipant()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSSphereCollider collider3D = CreateBodylessSphere3D(context, new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        PhysicsMaterial sliding = new(Fixed64.Half, Fixed64.Half, Fixed64.Zero);
        collider3D.Material = sliding;
        body2D.Collider.Material = sliding;
        body2D.ApplyCollisionLinearVelocityDelta(new Vector2d((Fixed64)(-4), Fixed64.One));
        var pair = new CollisionPairMixed(collider3D, body2D.Collider);
        var contact = new MixedContact(
            collider3D.Center,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right,
            Fixed64.FromFraction(1, 5));

        bool appliedImpulse = CollisionResponseMixed.Resolve(pair, contact);

        appliedImpulse.Should().BeTrue();
        collider3D.Center.Should().Be(new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        body2D.Position.X.Should().BeGreaterThan(Fixed64.Zero);
        body2D.LinearVelocity.X.Should().BeGreaterThan(-(Fixed64)4);
        body2D.LinearVelocity.Y.Should().BeLessThan(Fixed64.One);
    }

    [Fact]
    public void Resolve_WithDynamic3DAndBodyless2D_ShouldApplyImpulseOnlyTo3DParticipant()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        LSCollider2D collider2D = CreateBodylessCircle2D(context, Vector2d.Zero);
        PhysicsMaterial sliding = new(Fixed64.Half, Fixed64.Half, Fixed64.Zero);
        body3D.Collider.Material = sliding;
        collider2D.Material = sliding;
        body3D.Body.ApplyCollisionLinearVelocityDelta(
            new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.One));
        var pair = new CollisionPairMixed(body3D.Collider, collider2D);
        var contact = new MixedContact(
            body3D.Collider.Center,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right,
            Fixed64.FromFraction(1, 5));

        bool appliedImpulse = CollisionResponseMixed.Resolve(pair, contact);

        appliedImpulse.Should().BeTrue();
        body3D.Body.Position3d.X.Should().BeLessThan(-Fixed64.Half);
        body3D.Body.LinearVelocity.X.Should().BeLessThan((Fixed64)4);
        body3D.Body.LinearVelocity.Z.Should().BeLessThan(Fixed64.One);
        collider2D.Center.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void Resolve_WithMixedTriggerPair_ShouldSkipResponse()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        LSCollider2D trigger2D = CreateBodylessCircle2D(context, Vector2d.Zero, isTrigger: true);
        body3D.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)4);
        var pair = new CollisionPairMixed(body3D.Collider, trigger2D);
        var contact = new MixedContact(
            body3D.Collider.Center,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right,
            Fixed64.FromFraction(1, 5));

        bool appliedImpulse = CollisionResponseMixed.Resolve(pair, contact);

        appliedImpulse.Should().BeFalse();
        body3D.Body.Position3d.Should().Be(new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        body3D.Body.LinearVelocity.X.Should().Be((Fixed64)4);
    }

    [Fact]
    public void Resolve_WithFullyFrozenMixedBodies_ShouldSkipNoEffectiveMass()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero, immovable: true);
        body3D.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)4);
        body2D.ApplyCollisionLinearVelocityDelta(-Vector2d.Right * (Fixed64)4);
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        var contact = new MixedContact(
            body3D.Collider.Center,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right,
            Fixed64.FromFraction(1, 5));

        bool appliedImpulse = CollisionResponseMixed.Resolve(pair, contact);

        appliedImpulse.Should().BeFalse();
        body3D.Body.Position3d.Should().Be(new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        body2D.Position.Should().Be(Vector2d.Zero);
        body3D.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        body2D.LinearVelocity.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void Resolve_With2DRotationFrozen_ShouldApplyPlanarImpulseWithoutYaw()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Half));
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        body2D.FreezeAxes = BodyFreezeAxes2D.Rotation;
        body3D.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)4);
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        var contact = new MixedContact(
            body3D.Collider.Center,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half),
            Vector3d.Right,
            Fixed64.FromFraction(1, 5));

        bool appliedImpulse = CollisionResponseMixed.Resolve(pair, contact);

        appliedImpulse.Should().BeTrue();
        body2D.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        body2D.AngularVelocity.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_With3DRotationFrozen_ShouldApplyLinearImpulseWithoutAngularDelta()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Half),
            preventAngularForces: true);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        body3D.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)4);
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        var contact = new MixedContact(
            body3D.Collider.Center,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half),
            Vector3d.Right,
            Fixed64.FromFraction(1, 5));

        bool appliedImpulse = CollisionResponseMixed.Resolve(pair, contact);

        appliedImpulse.Should().BeTrue();
        body3D.Body.LinearVelocity.X.Should().BeLessThan((Fixed64)4);
        body3D.Body.AngularVelocity.Should().Be(Vector3d.Zero);
        body2D.AngularVelocity.Should().BeLessThan(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithSeparatingMixedVelocity_ShouldSkipImpulseWhenCorrectionIsDisabled()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        body3D.Body.ApplyCollisionLinearVelocityDelta(-Vector3d.Right * (Fixed64)4);
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        var contact = new MixedContact(
            body3D.Collider.Center,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right,
            Fixed64.FromFraction(1, 5));

        bool appliedImpulse = CollisionResponseMixed.Resolve(
            pair,
            contact,
            iteration: 0,
            iterationLimit: 1,
            applyPositionCorrection: false);

        appliedImpulse.Should().BeFalse();
        body3D.Body.LinearVelocity.X.Should().Be(-(Fixed64)4);
        body2D.LinearVelocity.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void Resolve_WithFrictionlessMixedMaterials_ShouldPreserveTangentialVelocity()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        body3D.Collider.Material = PhysicsMaterial.Frictionless;
        body2D.Collider.Material = PhysicsMaterial.Frictionless;
        body3D.Body.ApplyCollisionLinearVelocityDelta(new Vector3d((Fixed64)4, Fixed64.Zero, (Fixed64)3));
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        var contact = new MixedContact(
            body3D.Collider.Center,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right,
            Fixed64.FromFraction(1, 5));

        bool appliedImpulse = CollisionResponseMixed.Resolve(
            pair,
            contact,
            iteration: 0,
            iterationLimit: 1,
            applyPositionCorrection: false);

        appliedImpulse.Should().BeTrue();
        body3D.Body.LinearVelocity.X.Should().BeLessThan((Fixed64)4);
        body3D.Body.LinearVelocity.Z.Should().Be((Fixed64)3);
        body2D.LinearVelocity.Y.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithShallowMixedPenetration_ShouldApplyVelocityImpulseWithoutPositionCorrection()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        body3D.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)4);
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        var contact = new MixedContact(
            body3D.Collider.Center,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right,
            Fixed64.Zero);

        bool appliedImpulse = CollisionResponseMixed.Resolve(pair, contact);

        appliedImpulse.Should().BeTrue();
        body3D.Body.Position3d.Should().Be(new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        body2D.Position.Should().Be(Vector2d.Zero);
        body3D.Body.LinearVelocity.X.Should().BeLessThan((Fixed64)4);
        body2D.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithHighStaticFrictionMixedMaterials_ShouldReduceTangentialVelocityWithoutDynamicClamp()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        PhysicsMaterial highStaticFriction = new((Fixed64)8, Fixed64.One, Fixed64.Zero);
        body3D.Collider.Material = highStaticFriction;
        body2D.Collider.Material = highStaticFriction;
        body3D.Body.ApplyCollisionLinearVelocityDelta(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.FromFraction(1, 4)));
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        var contact = new MixedContact(
            body3D.Collider.Center,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right,
            Fixed64.FromFraction(1, 5));

        bool appliedImpulse = CollisionResponseMixed.Resolve(
            pair,
            contact,
            iteration: 0,
            iterationLimit: 1,
            applyPositionCorrection: false);

        appliedImpulse.Should().BeTrue();
        body3D.Body.LinearVelocity.Z.Should().BeLessThan(Fixed64.FromFraction(1, 4));
    }

    [Fact]
    public void Resolve_WithDynamicFrictionClamp_ShouldReduceTangentialVelocityPartially()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        PhysicsMaterial sliding = new(Fixed64.Half, Fixed64.Half, Fixed64.Zero);
        body3D.Collider.Material = sliding;
        body2D.Collider.Material = sliding;
        body3D.Body.ApplyCollisionLinearVelocityDelta(new Vector3d((Fixed64)4, Fixed64.Zero, (Fixed64)3));
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        var contact = new MixedContact(
            body3D.Collider.Center,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right,
            Fixed64.FromFraction(1, 5));

        bool appliedImpulse = CollisionResponseMixed.Resolve(
            pair,
            contact,
            iteration: 0,
            iterationLimit: 1,
            applyPositionCorrection: false);

        appliedImpulse.Should().BeTrue();
        body3D.Body.LinearVelocity.Z.Should().BeGreaterThan(Fixed64.Zero);
        body3D.Body.LinearVelocity.Z.Should().BeLessThan((Fixed64)3);
    }

    [Fact]
    public void Resolve_WithStaticOnlyFrictionClamp_ShouldPreserveTangentialVelocity()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        PhysicsMaterial staticOnly = new(Fixed64.FromFraction(1, 10), Fixed64.Zero, Fixed64.Zero);
        body3D.Collider.Material = staticOnly;
        body2D.Collider.Material = staticOnly;
        body3D.Body.ApplyCollisionLinearVelocityDelta(new Vector3d((Fixed64)4, Fixed64.Zero, (Fixed64)3));
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        var contact = new MixedContact(
            body3D.Collider.Center,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right,
            Fixed64.FromFraction(1, 5));

        bool appliedImpulse = CollisionResponseMixed.Resolve(
            pair,
            contact,
            iteration: 0,
            iterationLimit: 1,
            applyPositionCorrection: false);

        appliedImpulse.Should().BeTrue();
        body3D.Body.LinearVelocity.X.Should().BeLessThan((Fixed64)4);
        body3D.Body.LinearVelocity.Z.Should().Be((Fixed64)3);
    }

    [Fact]
    public void Resolve_WithOpposedMixedContactNormal_ShouldFlipNormalTowardEmbeddedCollider()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        body3D.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)4);
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        var contact = new MixedContact(
            body3D.Collider.Center,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            -Vector3d.Right,
            Fixed64.FromFraction(1, 5));

        bool appliedImpulse = CollisionResponseMixed.Resolve(
            pair,
            contact,
            iteration: 0,
            iterationLimit: 1,
            applyPositionCorrection: false);

        appliedImpulse.Should().BeTrue();
        body3D.Body.LinearVelocity.X.Should().BeLessThan((Fixed64)4);
        body2D.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithZeroNormalAndCoincidentFallback_ShouldUseVerticalFallback()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        body3D.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Up * (Fixed64)4);
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        var contact = new MixedContact(
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.FromFraction(1, 5));

        bool appliedImpulse = CollisionResponseMixed.Resolve(
            pair,
            contact,
            iteration: 0,
            iterationLimit: 1,
            applyPositionCorrection: false);

        appliedImpulse.Should().BeTrue();
        body3D.Body.LinearVelocity.Y.Should().BeLessThan((Fixed64)4);
        body2D.Position.Should().Be(Vector2d.Zero);
        body2D.LinearVelocity.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void Resolve_WithZeroNormalAndCoincidentCenters_ShouldUseContactPointFallback()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        body3D.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)4);
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        var contact = new MixedContact(
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Zero,
            Fixed64.FromFraction(1, 5));

        bool appliedImpulse = CollisionResponseMixed.Resolve(
            pair,
            contact,
            iteration: 0,
            iterationLimit: 1,
            applyPositionCorrection: false);

        appliedImpulse.Should().BeTrue();
        body3D.Body.LinearVelocity.X.Should().BeLessThan((Fixed64)4);
        body2D.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void Simulate_WithBodylessMixedTrigger_ShouldNotifyTriggerWithoutResponse()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        LSCollider2D trigger = CreateBodylessCircle2D(context, Vector2d.Zero, isTrigger: true);
        int triggerEntered = 0;
        int contactEntered = 0;
        trigger.OnMixedTriggerEnter += other =>
        {
            other.Should().BeSameAs(body3D.Collider);
            triggerEntered++;
        };
        body3D.Collider.OnMixedContactEnter += _ => contactEntered++;

        Step(context);

        body3D.Body.Position3d.Should().Be(Vector3d.Zero);
        triggerEntered.Should().Be(1);
        contactEntered.Should().Be(0);
    }

    [Fact]
    public void Simulate_WithBodylessMixedTrigger_ShouldNotifyStayAndExitOnBothParticipants()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        LSCollider2D trigger = CreateBodylessCircle2D(context, Vector2d.Zero, isTrigger: true);
        int entered3D = 0;
        int stayed3D = 0;
        int exited3D = 0;
        int entered2D = 0;
        int stayed2D = 0;
        int exited2D = 0;
        int contacted = 0;
        body3D.Collider.OnMixedTriggerEnter += other =>
        {
            other.Should().BeSameAs(trigger);
            entered3D++;
        };
        body3D.Collider.OnMixedTriggerStay += other =>
        {
            other.Should().BeSameAs(trigger);
            stayed3D++;
        };
        body3D.Collider.OnMixedTriggerExit += other =>
        {
            other.Should().BeSameAs(trigger);
            exited3D++;
        };
        body3D.Collider.OnMixedContact += _ => contacted++;
        trigger.OnMixedTriggerEnter += other =>
        {
            other.Should().BeSameAs(body3D.Collider);
            entered2D++;
        };
        trigger.OnMixedTriggerStay += other =>
        {
            other.Should().BeSameAs(body3D.Collider);
            stayed2D++;
        };
        trigger.OnMixedTriggerExit += other =>
        {
            other.Should().BeSameAs(body3D.Collider);
            exited2D++;
        };

        Step(context);
        Step(context);
        body3D.Body.SetPosition(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));
        Step(context);

        entered3D.Should().Be(1);
        stayed3D.Should().Be(2);
        exited3D.Should().Be(1);
        entered2D.Should().Be(1);
        stayed2D.Should().Be(2);
        exited2D.Should().Be(1);
        contacted.Should().Be(0);
        context.MixedCollisions.ActivePairCount.Should().Be(0);
    }

    [Fact]
    public void Simulate_WithMixedBoundsOverlapButExactMiss_ShouldSkipPairCreation()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D circle = CreateCircle2D(
            context,
            new Vector2d(Fixed64.FromFraction(9, 10), Fixed64.FromFraction(9, 10)));

        Step(context);

        context.MixedCollisions.LastBroadPhaseCandidateCount.Should().Be(1);
        context.MixedCollisions.ActivePairCount.Should().Be(0);
        circle.Position.Should().Be(new Vector2d(Fixed64.FromFraction(9, 10), Fixed64.FromFraction(9, 10)));
    }

    [Fact]
    public void Simulate_WithAwake3DAgainstSleeping2D_ShouldWakeSleepingParticipant()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateSphere3D(context, new Vector3d(-Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D sleeping2D = CreateCircle2D(context, Vector2d.Zero);
        sleeping2D.Sleep();

        Step(context);

        sleeping2D.IsSleeping.Should().BeFalse();
    }

    [Fact]
    public void Simulate_WithLayerBlockedMixedPair_ShouldNotCreatePairOrRespond()
    {
        using GravitasWorldContext context = CreateMixedContextWithLayerBlock();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero, layer: new PhysicsLayer(1));
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero, layer: new PhysicsLayer(2));
        int entered = 0;
        body3D.Collider.OnMixedContactEnter += _ => entered++;

        Step(context);

        context.MixedCollisions.ActivePairCount.Should().Be(0);
        body3D.Body.Position3d.Should().Be(Vector3d.Zero);
        body2D.Position.Should().Be(Vector2d.Zero);
        entered.Should().Be(0);
    }

    [Fact]
    public void ReplayedMixedResponseScenario_ShouldProduceSameState()
    {
        (Vector3d position3D, Vector3d velocity3D, Vector2d position2D, Vector2d velocity2D) first = RunReplayScenario();
        (Vector3d position3D, Vector3d velocity3D, Vector2d position2D, Vector2d velocity2D) second = RunReplayScenario();

        second.Should().Be(first);
    }

    private static (Vector3d position3D, Vector3d velocity3D, Vector2d position2D, Vector2d velocity2D) RunReplayScenario()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 8);
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);

        for (int i = 0; i < 5; i++)
        {
            body3D.Body.AddForce(Vector3d.Right);
            body2D.AddForce(-Vector2d.Right);
            context.Simulate();
            context.LateSimulate();
        }

        return (body3D.Body.Position3d, body3D.Body.LinearVelocity, body2D.Position, body2D.LinearVelocity);
    }

    private static GravitasWorldContext CreateMixedContext(int frameRate = 4)
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.ApplySettings(new PhysicsSettings(frameRate, null));
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.Environment.Gravity = Fixed64.Zero;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-8), (Fixed64)(-4), (Fixed64)(-8)),
                new Vector3d((Fixed64)8, (Fixed64)4, (Fixed64)8)),
            out _).Should().BeTrue();
        return context;
    }

    private static void Step(GravitasWorldContext context)
    {
        context.Simulate();
        context.LateSimulate();
    }

    private static GravitasMixedResponseImpulseDiagnosticView FindFirstMixedImpulse(GravitasWorldContext context)
    {
        ReadOnlySpan<GravitasDiagnosticEvent> events = context.Diagnostics.Events;
        for (int i = 0; i < events.Length; i++)
            if (events[i].TryAsMixedResponseImpulse(out GravitasMixedResponseImpulseDiagnosticView view))
                return view;

        throw new InvalidOperationException("Expected a mixed response impulse diagnostic event.");
    }

    private static GravitasMixedResponseIslandDiagnosticView FindFirstMixedIsland(GravitasWorldContext context)
    {
        ReadOnlySpan<GravitasDiagnosticEvent> events = context.Diagnostics.Events;
        for (int i = 0; i < events.Length; i++)
            if (events[i].TryAsMixedResponseIsland(out GravitasMixedResponseIslandDiagnosticView view))
                return view;

        throw new InvalidOperationException("Expected a mixed response island diagnostic event.");
    }

    private static int FindMaxMixedImpulseIterationLimit(GravitasWorldContext context)
    {
        int max = 0;
        ReadOnlySpan<GravitasDiagnosticEvent> events = context.Diagnostics.Events;
        for (int i = 0; i < events.Length; i++)
            if (events[i].TryAsMixedResponseImpulse(out GravitasMixedResponseImpulseDiagnosticView view)
                && view.IterationLimit > max)
            {
                max = view.IterationLimit;
            }

        return max;
    }

    private static int CountMixedEvents(GravitasWorldContext context)
    {
        int count = 0;
        ReadOnlySpan<GravitasDiagnosticEvent> events = context.Diagnostics.Events;
        for (int i = 0; i < events.Length; i++)
        {
            if (events[i].Kind == GravitasDiagnosticEventKind.MixedContact
                || events[i].Kind == GravitasDiagnosticEventKind.MixedResponseImpulse
                || events[i].Kind == GravitasDiagnosticEventKind.MixedResponseIsland)
            {
                count++;
            }
        }

        return count;
    }

    private static int CountMixedIslandEvents(GravitasWorldContext context)
    {
        int count = 0;
        ReadOnlySpan<GravitasDiagnosticEvent> events = context.Diagnostics.Events;
        for (int i = 0; i < events.Length; i++)
            if (events[i].Kind == GravitasDiagnosticEventKind.MixedResponseIsland)
                count++;

        return count;
    }

    private static Fixed64 Resolve3DPlanarVelocityAfterMixedResponse(
        Fixed64 threshold,
        Fixed64 initialVelocity,
        PhysicsMaterial? material3D = null,
        PhysicsMaterial? material2D = null)
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Settings.RestitutionVelocityThreshold = threshold;
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Half));
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        body3D.Collider.Material = material3D ?? PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        body2D.Collider.Material = material2D ?? PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        var contact = new MixedContact(
            new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Half),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half),
            Vector3d.Right,
            Fixed64.FromFraction(1, 10));
        body3D.Body.ApplyCollisionLinearVelocityDelta(new Vector3d(initialVelocity, Fixed64.Zero, Fixed64.Zero));

        CollisionResponseMixed.Resolve(pair, contact);

        return body3D.Body.LinearVelocity.X;
    }

    private static Fixed64 Resolve2DTangentialVelocityAfterMixedResponse(
        PhysicsMaterial material3D,
        PhysicsMaterial material2D)
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Settings.RestitutionVelocityThreshold = (Fixed64)100;
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Half));
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        body3D.Collider.Material = material3D;
        body2D.Collider.Material = material2D;
        body3D.Body.ApplyCollisionLinearVelocityDelta(new Vector3d((Fixed64)4, Fixed64.Zero, (Fixed64)2));
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        var contact = new MixedContact(
            new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Half),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half),
            Vector3d.Right,
            Fixed64.FromFraction(1, 10));

        CollisionResponseMixed.Resolve(
            pair,
            contact,
            iteration: 0,
            iterationLimit: 1,
            applyPositionCorrection: false).Should().BeTrue();

        return body2D.LinearVelocity.Y;
    }

    private static GravitasWorldContext CreateMixedContextWithLayerBlock()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.ApplySettings(new PhysicsSettings(
            4,
            new[,]
            {
                { true, true, true },
                { true, true, false },
                { true, false, true }
            }));
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.Environment.Gravity = Fixed64.Zero;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-8), (Fixed64)(-4), (Fixed64)(-8)),
                new Vector3d((Fixed64)8, (Fixed64)4, (Fixed64)8)),
            out _).Should().BeTrue();
        return context;
    }

    private static ScenarioBody<LSSphereCollider> CreateSphere3D(
        GravitasWorldContext context,
        Vector3d position,
        bool immovable = false,
        bool preventAngularForces = false,
        bool isKinematic = false,
        PhysicsLayer? layer = null)
    {
        var collider = new LSSphereCollider();
        if (layer.HasValue)
            collider.Layer = layer.Value;

        var agent = new TestMatterAgent(context, new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One));
        BodyFreezeAxes3D freezeAxes =
            (immovable ? BodyFreezeAxes3D.Position : BodyFreezeAxes3D.None)
            | (preventAngularForces ? BodyFreezeAxes3D.Rotation : BodyFreezeAxes3D.None);
        var body = new SolidBody(agent, collider)
        {
            Mass = Fixed64.One,
            FreezeAxes = freezeAxes,
            IsKinematic = isKinematic
        };
        collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero);
        body.Initialize(position, FixedQuaternion.Identity);
        return new ScenarioBody<LSSphereCollider>(body, collider);
    }

    private static LSSphereCollider CreateBodylessSphere3D(GravitasWorldContext context, Vector3d position)
    {
        var collider = new LSSphereCollider
        {
            Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero)
        };
        var agent = new TestMatterAgent(context, new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One));
        collider.InitializeWithNoBody(agent);
        return collider;
    }

    private static SolidBody2D CreateCircle2D(
        GravitasWorldContext context,
        Vector2d position,
        bool immovable = false,
        PhysicsLayer? layer = null)
    {
        var collider = new LSCircleCollider2D(Fixed64.Half);
        if (layer.HasValue)
            collider.Layer = layer.Value;

        var agent = new TestMatterAgent(
            context,
            new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One));
        var body = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One,
            FreezeAxes = immovable ? BodyFreezeAxes2D.Position : BodyFreezeAxes2D.None
        };
        collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero);
        body.Initialize(position);
        return body;
    }

    private static LSCollider2D CreateBodylessCircle2D(
        GravitasWorldContext context,
        Vector2d position,
        bool isTrigger = false)
    {
        var collider = new LSCircleCollider2D(Fixed64.Half)
        {
            IsTrigger = isTrigger
        };
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One));
        collider.InitializeWithNoBody(agent);
        return collider;
    }

    private static LSCollider2D CreateBodylessBox2D(
        GravitasWorldContext context,
        Vector2d position,
        Vector2d size)
    {
        var collider = new LSAABBoxCollider2D(size);
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One));
        collider.InitializeWithNoBody(agent);
        return collider;
    }
}
