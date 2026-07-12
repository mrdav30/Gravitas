using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Response;

public sealed class CollisionResponse2DManifoldTests
{
    [Fact]
    public void Resolve_WithSymmetricFaceContacts_ShouldNotIntroduceAngularVelocity()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D moving = CreateBox(context, Vector2d.Zero);
        SolidBody2D wall = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        moving.ApplyCollisionLinearVelocityDelta(new Vector2d((Fixed64)4, Fixed64.Zero));
        AddFaceContacts(pair, depth: Fixed64.Half);

        pair.MarkColliding(context.FrameCount);

        moving.LinearVelocity.X.Should().BeLessThan((Fixed64)4);
        moving.AngularVelocity.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithOffCenterSingleContact_ShouldStillIntroduceAngularVelocity()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D moving = CreateBox(context, Vector2d.Zero);
        SolidBody2D wall = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        moving.ApplyCollisionLinearVelocityDelta(new Vector2d((Fixed64)4, Fixed64.Zero));
        pair.Manifold.SetContact(
            new Vector2d(Fixed64.One, Fixed64.One),
            new Vector2d(Fixed64.One, Fixed64.One),
            Fixed64.Half,
            Vector2d.Right);

        pair.MarkColliding(context.FrameCount);

        moving.AngularVelocity.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithConfiguredRestitutionThreshold_ShouldControlBounce()
    {
        Fixed64 highThresholdVelocity = ResolveClosingVelocityAfterResponse(
            threshold: (Fixed64)5,
            initialVelocity: (Fixed64)4);
        Fixed64 zeroThresholdVelocity = ResolveClosingVelocityAfterResponse(
            threshold: Fixed64.Zero,
            initialVelocity: (Fixed64)4);

        highThresholdVelocity.Should().BeGreaterThan(zeroThresholdVelocity);
        highThresholdVelocity.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        zeroThresholdVelocity.Should().BeLessThan(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithZeroContactNormal_ShouldUseColliderCenterFallback()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D moving = CreateBox(context, Vector2d.Zero);
        SolidBody2D wall = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        moving.ApplyCollisionLinearVelocityDelta(new Vector2d((Fixed64)4, Fixed64.Zero));
        pair.Manifold.SetContact(Vector2d.Right, Vector2d.Right, Fixed64.Half, Vector2d.Zero);
        Fixed64 velocityBefore = moving.LinearVelocity.X;

        pair.MarkColliding(context.FrameCount);

        moving.LinearVelocity.X.Should().BeLessThan(velocityBefore);
    }

    [Fact]
    public void Resolve_WithOpposedContactNormal_ShouldFlipTowardSecondCollider()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D moving = CreateBox(context, Vector2d.Zero);
        SolidBody2D wall = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        moving.ApplyCollisionLinearVelocityDelta(new Vector2d((Fixed64)4, Fixed64.Zero));
        pair.Manifold.SetContact(Vector2d.Right, Vector2d.Right, Fixed64.Half, -Vector2d.Right);
        Fixed64 velocityBefore = moving.LinearVelocity.X;

        pair.MarkColliding(context.FrameCount);

        moving.LinearVelocity.X.Should().BeLessThan(velocityBefore);
    }

    [Fact]
    public void Resolve_WithZeroNormalAndNoFallbackDirection_ShouldIgnoreContact()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D first = CreateBox(context, Vector2d.Zero);
        SolidBody2D second = CreateBox(context, Vector2d.Zero);
        var pair = new CollisionPair2D(first.Collider, second.Collider);
        first.ApplyCollisionLinearVelocityDelta(new Vector2d((Fixed64)4, Fixed64.Zero));
        second.ApplyCollisionLinearVelocityDelta(new Vector2d((Fixed64)(-4), Fixed64.Zero));
        pair.Manifold.SetContact(Vector2d.Zero, Vector2d.Zero, Fixed64.Half, Vector2d.Zero);
        Vector2d firstVelocityBefore = first.LinearVelocity;
        Vector2d secondVelocityBefore = second.LinearVelocity;

        pair.MarkColliding(context.FrameCount);

        first.LinearVelocity.Should().Be(firstVelocityBefore);
        second.LinearVelocity.Should().Be(secondVelocityBefore);
    }

    [Fact]
    public void Resolve_WithTriggerEmptyOrInfiniteMassPair_ShouldNotMutateBodies()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D triggerTarget = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero));
        var trigger = new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2))
        {
            IsTrigger = true
        };
        trigger.InitializeWithNoBody(new TestMatterAgent(
            context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        var triggerPair = new CollisionPair2D(trigger, triggerTarget.Collider);
        triggerPair.ColliderB.Should().BeSameAs(trigger);
        triggerPair.Manifold.SetContact(Vector2d.Right, Vector2d.Right, Fixed64.Half, Vector2d.Right);
        triggerTarget.ApplyCollisionLinearVelocityDelta(Vector2d.Right);
        CollisionResponse2D.Resolve(triggerPair);
        triggerTarget.LinearVelocity.Should().Be(Vector2d.Right);

        var leadingTrigger = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero))
        {
            IsTrigger = true
        };
        leadingTrigger.InitializeWithNoBody(new TestMatterAgent(
            context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        SolidBody2D secondTriggerTarget = CreateBox(context, new Vector2d((Fixed64)3, Fixed64.Zero));
        var leadingTriggerPair = new CollisionPair2D(leadingTrigger, secondTriggerTarget.Collider);
        leadingTriggerPair.ColliderA.Should().BeSameAs(leadingTrigger);
        leadingTriggerPair.Manifold.SetContact(Vector2d.Right, Vector2d.Right, Fixed64.Half, Vector2d.Right);
        secondTriggerTarget.ApplyCollisionLinearVelocityDelta(Vector2d.Right);
        Vector2d secondTriggerPosition = secondTriggerTarget.Position;
        CollisionResponse2D.Resolve(leadingTriggerPair);
        secondTriggerTarget.Position.Should().Be(secondTriggerPosition);
        secondTriggerTarget.LinearVelocity.Should().Be(Vector2d.Right);

        SolidBody2D emptyA = CreateBox(context, new Vector2d((Fixed64)4, Fixed64.Zero));
        SolidBody2D emptyB = CreateBox(context, new Vector2d((Fixed64)6, Fixed64.Zero));
        var emptyPair = new CollisionPair2D(emptyA.Collider, emptyB.Collider);
        emptyA.ApplyCollisionLinearVelocityDelta(Vector2d.Right);
        CollisionResponse2D.Resolve(emptyPair);
        emptyA.LinearVelocity.Should().Be(Vector2d.Right);

        SolidBody2D frozenA = CreateBox(context, new Vector2d((Fixed64)8, Fixed64.Zero), immovable: true);
        SolidBody2D frozenB = CreateBox(context, new Vector2d((Fixed64)10, Fixed64.Zero), immovable: true);
        var frozenPair = new CollisionPair2D(frozenA.Collider, frozenB.Collider);
        frozenPair.Manifold.SetContact(Vector2d.Right, Vector2d.Right, Fixed64.Half, Vector2d.Right);
        CollisionResponse2D.Resolve(frozenPair);
        frozenPair.TryGetWarmStartImpulse(
            frozenPair.Manifold.PrimaryContact.ContactId,
            out _).Should().BeFalse();
    }

    [Fact]
    public void Resolve_WhenContactAxisIsFrozen_ShouldSkipUnresolvableCorrectionAndImpulse()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D moving = CreateBox(context, Vector2d.Zero);
        SolidBody2D wall = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        moving.FreezeAxes = BodyFreezeAxes2D.PositionX | BodyFreezeAxes2D.Rotation;
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        pair.Manifold.SetContact(Vector2d.Right, Vector2d.Right, Fixed64.Half, Vector2d.Right);
        Vector2d positionBefore = moving.Position;

        pair.MarkColliding(context.FrameCount);

        moving.Position.Should().Be(positionBefore);
        moving.LinearVelocity.Should().Be(Vector2d.Zero);
        pair.TryGetWarmStartImpulse(
            pair.Manifold.PrimaryContact.ContactId,
            out ContactWarmStartImpulse impulse).Should().BeTrue();
        impulse.NormalImpulse.Should().Be(Fixed64.Zero);
        impulse.TangentImpulse.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithFrictionlessContact_ShouldPreserveTangentialVelocity()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D moving = CreateBox(context, Vector2d.Zero);
        SolidBody2D wall = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        moving.Collider.Material = PhysicsMaterial.Frictionless;
        wall.Collider.Material = PhysicsMaterial.Frictionless;
        moving.FreezeAxes = BodyFreezeAxes2D.Rotation;
        moving.ApplyCollisionLinearVelocityDelta(new Vector2d((Fixed64)4, (Fixed64)2));
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        pair.Manifold.SetContact(Vector2d.Right, Vector2d.Right, Fixed64.Half, Vector2d.Right);
        Fixed64 tangentialVelocity = moving.LinearVelocity.Y;

        pair.MarkColliding(context.FrameCount);

        moving.LinearVelocity.X.Should().BeLessThan((Fixed64)4);
        moving.LinearVelocity.Y.Should().Be(tangentialVelocity);
    }

    [Fact]
    public void Resolve_WithNearZeroTangentMobility_ShouldPreserveTangentialVelocity()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D moving = CreateBox(context, Vector2d.Zero);
        SolidBody2D wall = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        moving.Mass = Fixed64.MaxValue;
        moving.FreezeAxes = BodyFreezeAxes2D.Rotation;
        moving.ApplyCollisionLinearVelocityDelta(new Vector2d(Fixed64.Zero, Fixed64.One));
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        pair.Manifold.SetContact(Vector2d.Right, Vector2d.Right, Fixed64.Zero, Vector2d.Right);
        pair.StoreWarmStartImpulse(
            pair.Manifold.PrimaryContact.ContactId,
            Fixed64.One,
            Fixed64.Zero);
        Fixed64 tangentialVelocity = moving.LinearVelocity.Y;

        pair.MarkColliding(context.FrameCount);

        moving.LinearVelocity.Y.Should().Be(tangentialVelocity);
    }

    [Fact]
    public void Resolve_WithZeroConfiguredRestitutionThreshold_ShouldBounceLowSpeedContact()
    {
        Fixed64 velocity = ResolveClosingVelocityAfterResponse(
            threshold: Fixed64.Zero,
            initialVelocity: Fixed64.FromFraction(1, 8));

        velocity.Should().BeLessThan(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithTwoContacts_ShouldApplyPositionCorrectionOnceForPair()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D moving = CreateBox(context, Vector2d.Zero);
        SolidBody2D wall = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        AddFaceContacts(pair, depth: Fixed64.Half);
        Fixed64 expectedCorrection = Fixed64.Half - CollisionResponse2D.PenetrationSlop;

        pair.MarkColliding(context.FrameCount);

        moving.Position.Should().Be(new Vector2d(-expectedCorrection, Fixed64.Zero));
    }

    [Fact]
    public void Resolve_WithTwoFrictionContacts_ShouldOpposeTangentialMotionAndCacheBothContacts()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D moving = CreateBox(context, Vector2d.Zero);
        SolidBody2D wall = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        moving.ApplyCollisionLinearVelocityDelta(new Vector2d((Fixed64)4, (Fixed64)20));
        Fixed64 tangentialSpeed = moving.LinearVelocity.Y.Abs();
        AddFaceContacts(pair, depth: Fixed64.Half);

        pair.MarkColliding(context.FrameCount);

        moving.LinearVelocity.Y.Abs().Should().BeLessThan(tangentialSpeed);
        pair.TryGetWarmStartImpulse(pair.Manifold[0].ContactId, out ContactWarmStartImpulse first).Should().BeTrue();
        pair.TryGetWarmStartImpulse(pair.Manifold[1].ContactId, out ContactWarmStartImpulse second).Should().BeTrue();
        first.TangentImpulse.Abs().Should().BeGreaterThan(Fixed64.Zero);
        second.TangentImpulse.Abs().Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithCapsuleSideContacts_ShouldOpposeTangentialMotionAndCacheContacts()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D capsule = CreateBody(context, new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)4), Vector2d.Zero);
        capsule.SetRotation(FixedMath.DegToRad((Fixed64)90));
        SolidBody2D floor = CreateBody(
            context,
            new LSAABBoxCollider2D(new Vector2d((Fixed64)5, Fixed64.One)),
            new Vector2d(Fixed64.Zero, -Fixed64.Half),
            immovable: true);
        var pair = new CollisionPair2D(capsule.Collider, floor.Collider);
        CollisionDetection2D.TryCollide(pair, pair.Manifold, context.FrameCount).Should().BeTrue();
        pair.Manifold.Count.Should().Be(2);
        capsule.ApplyCollisionLinearVelocityDelta(new Vector2d((Fixed64)20, (Fixed64)(-4)));
        Fixed64 tangentialSpeed = capsule.LinearVelocity.X.Abs();

        pair.MarkColliding(context.FrameCount);

        capsule.LinearVelocity.X.Abs().Should().BeLessThan(tangentialSpeed);
        for (int i = 0; i < pair.Manifold.Count; i++)
        {
            pair.TryGetWarmStartImpulse(pair.Manifold[i].ContactId, out ContactWarmStartImpulse impulse).Should().BeTrue();
            impulse.TangentImpulse.Abs().Should().BeGreaterThan(Fixed64.Zero);
        }
    }

    [Fact]
    public void Resolve_WithHighStaticAndZeroDynamicFriction_ShouldHoldTangentialMotion()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D moving = CreateBox(context, Vector2d.Zero);
        SolidBody2D wall = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        PhysicsMaterial stickyStatic = new((Fixed64)100, Fixed64.Zero, Fixed64.Zero);
        moving.Collider.Material = stickyStatic;
        wall.Collider.Material = stickyStatic;
        moving.FreezeAxes = BodyFreezeAxes2D.Rotation;
        moving.ApplyCollisionLinearVelocityDelta(new Vector2d((Fixed64)4, Fixed64.Half));
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        pair.Manifold.SetContact(Vector2d.Right, Vector2d.Right, Fixed64.Half, Vector2d.Right);

        pair.MarkColliding(context.FrameCount);

        moving.LinearVelocity.Y.Abs().Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WhenStaticLimitIsExceeded_ShouldUseDynamicFriction()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D moving = CreateBox(context, Vector2d.Zero);
        SolidBody2D wall = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        moving.Collider.Material = new PhysicsMaterial(Fixed64.Half, Fixed64.Half, Fixed64.Zero);
        wall.Collider.Material = new PhysicsMaterial(Fixed64.Half, Fixed64.Half, Fixed64.Zero);
        moving.FreezeAxes = BodyFreezeAxes2D.Rotation;
        moving.ApplyCollisionLinearVelocityDelta(new Vector2d((Fixed64)4, (Fixed64)20));
        Fixed64 tangentialSpeed = moving.LinearVelocity.Y.Abs();
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        pair.Manifold.SetContact(Vector2d.Right, Vector2d.Right, Fixed64.Half, Vector2d.Right);

        pair.MarkColliding(context.FrameCount);

        moving.LinearVelocity.Y.Abs().Should().BeLessThan(tangentialSpeed);
        moving.LinearVelocity.Y.Abs().Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithMaterialManifold_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D moving = CreateBox(context, Vector2d.Zero);
        SolidBody2D wall = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        moving.Collider.Material = new PhysicsMaterial((Fixed64)2, Fixed64.One, Fixed64.Zero);
        wall.Collider.Material = new PhysicsMaterial(Fixed64.Half, Fixed64.FromFraction(1, 4), Fixed64.Half);
        moving.FreezeAxes = BodyFreezeAxes2D.Rotation;
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        pair.Manifold.SetContact(Vector2d.Right, Vector2d.Right, Fixed64.Zero, Vector2d.Right);
        Vector2d resetVelocity = new((Fixed64)4, (Fixed64)20);
        moving.ApplyCollisionLinearVelocityDelta(resetVelocity);
        CollisionResponse2D.Resolve(pair);
        moving.ApplyCollisionLinearVelocityDelta(resetVelocity - moving.LinearVelocity);
        moving.ApplyCollisionAngularVelocityDelta(-moving.AngularVelocity);

        long allocatedBytes = MeasureAllocatedBytes(() => CollisionResponse2D.Resolve(pair));

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void Resolve_WithPositionCorrectionCrossing2DPartitions_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(extent: 128);
        SolidBody2D moving = CreateBox(context, Vector2d.Zero);
        SolidBody2D wall = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        moving.FreezeAxes = BodyFreezeAxes2D.Rotation;
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        Fixed64 correctionStep = context.VoxelSize * (Fixed64)2;
        Fixed64 depth = correctionStep + CollisionResponse2D.PenetrationSlop;

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            () =>
            {
                moving.ApplyCollisionLinearVelocityDelta(-moving.LinearVelocity);
                moving.ApplyCollisionAngularVelocityDelta(-moving.AngularVelocity);
                pair.Manifold.SetContact(moving.Collider.Center, moving.Collider.Center, depth, Vector2d.Right);
                CollisionResponse2D.Resolve(pair);
            },
            warmupIterations: 4,
            stabilizationIterations: 2,
            measurementIterations: 4);

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void Resolve_WithCompoundPartMaterial_ShouldUsePartRestitution()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        PhysicsMaterial zeroOwner = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero);
        PhysicsMaterial bouncyPart = new(
            Fixed64.One,
            Fixed64.One,
            Fixed64.One,
            restitutionCombine: PhysicsMaterialCombine.Maximum);
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero, bouncyPart));
        SolidBody2D wall = CreateBody(context, compound, Vector2d.Zero, immovable: true);
        SolidBody2D moving = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d(-Fixed64.FromFraction(3, 4), Fixed64.Zero));
        wall.Collider.Material = zeroOwner;
        moving.Collider.Material = zeroOwner;
        moving.FreezeAxes = BodyFreezeAxes2D.Rotation;
        moving.ApplyCollisionLinearVelocityDelta(new Vector2d((Fixed64)4, Fixed64.Zero));
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        CollisionDetection2D.TryCollide(pair, pair.Manifold, context.FrameCount).Should().BeTrue();

        pair.MarkColliding(context.FrameCount);

        moving.LinearVelocity.X.Should().BeLessThan(Fixed64.Zero);
    }

    [Fact]
    public void ResponseBody2D_Create_ShouldUseEffectiveMassAndScalarMoment()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D movable = CreateBox(context, Vector2d.Zero);
        ResponseBody2D movableBody = ResponseBody2D.Create(movable.Collider);
        movableBody.CanTranslate.Should().BeTrue();
        movableBody.CanRotate.Should().BeTrue();
        movableBody.InverseMass.Should().BeGreaterThan(Fixed64.Zero);
        movableBody.InverseMoment.Should().BeGreaterThan(Fixed64.Zero);

        movable.FreezeAxes = BodyFreezeAxes2D.Rotation;
        ResponseBody2D angularDisabled = ResponseBody2D.Create(movable.Collider);
        angularDisabled.CanTranslate.Should().BeTrue();
        angularDisabled.CanRotate.Should().BeFalse();
        angularDisabled.InverseMoment.Should().Be(Fixed64.Zero);

        SolidBody2D immovable = CreateBox(context, new Vector2d((Fixed64)4, Fixed64.Zero), immovable: true);
        ResponseBody2D immovableBody = ResponseBody2D.Create(immovable.Collider);
        immovableBody.CanTranslate.Should().BeFalse();
        immovableBody.CanRotate.Should().BeFalse();
        immovableBody.InverseMass.Should().Be(Fixed64.Zero);

        SolidBody2D kinematic = CreateBox(context, new Vector2d((Fixed64)8, Fixed64.Zero), isKinematic: true);
        ResponseBody2D kinematicBody = ResponseBody2D.Create(kinematic.Collider);
        kinematicBody.CanTranslate.Should().BeFalse();
        kinematicBody.CanRotate.Should().BeFalse();

        SolidBody2D zeroMass = CreateBox(context, new Vector2d((Fixed64)12, Fixed64.Zero));
        zeroMass.Mass = Fixed64.Zero;
        ResponseBody2D zeroMassBody = ResponseBody2D.Create(zeroMass.Collider);
        zeroMassBody.CanTranslate.Should().BeFalse();
        zeroMassBody.CanRotate.Should().BeFalse();
        zeroMassBody.InverseMass.Should().Be(Fixed64.Zero);

        SolidBody2D inactive = CreateBox(context, new Vector2d((Fixed64)16, Fixed64.Zero));
        inactive.Deactivate();
        ResponseBody2D inactiveBody = ResponseBody2D.Create(inactive.Collider);
        inactiveBody.CanTranslate.Should().BeFalse();
        inactiveBody.CanRotate.Should().BeFalse();
        inactiveBody.InverseMass.Should().Be(Fixed64.Zero);
        inactiveBody.InverseMoment.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithStaleCachedImpulse_ShouldClampWarmStartCacheAfterFreshSolve()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D moving = CreateBox(context, Vector2d.Zero);
        SolidBody2D wall = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        pair.Manifold.SetContact(Vector2d.Right, Vector2d.Right, Fixed64.Half, Vector2d.Right);
        ulong contactId = pair.Manifold.PrimaryContact.ContactId;
        pair.StoreWarmStartImpulse(contactId, Fixed64.One, Fixed64.One);

        pair.MarkColliding(context.FrameCount);

        moving.LinearVelocity.Should().Be(Vector2d.Zero);
        moving.AngularVelocity.Should().Be(Fixed64.Zero);
        pair.TryGetWarmStartImpulse(contactId, out ContactWarmStartImpulse impulse).Should().BeTrue();
        impulse.NormalImpulse.Should().Be(Fixed64.Zero);
        impulse.TangentImpulse.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithClosingVelocity_ShouldRefreshWarmStartCache()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D moving = CreateBox(context, Vector2d.Zero);
        SolidBody2D wall = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        moving.ApplyCollisionLinearVelocityDelta(new Vector2d((Fixed64)4, Fixed64.Zero));
        pair.Manifold.SetContact(Vector2d.Right, Vector2d.Right, Fixed64.Half, Vector2d.Right);
        ulong contactId = pair.Manifold.PrimaryContact.ContactId;

        pair.MarkColliding(context.FrameCount);

        pair.TryGetWarmStartImpulse(contactId, out ContactWarmStartImpulse impulse).Should().BeTrue();
        impulse.NormalImpulse.Should().BeGreaterThan(Fixed64.Zero);
    }

    private static void AddFaceContacts(CollisionPair2D pair, Fixed64 depth)
    {
        pair.Manifold.SetContact(
            new Vector2d(Fixed64.One, -Fixed64.One),
            new Vector2d(Fixed64.One, -Fixed64.One),
            depth,
            Vector2d.Right);
        pair.Manifold.AddContact(
            new Vector2d(Fixed64.One, Fixed64.One),
            new Vector2d(Fixed64.One, Fixed64.One),
            depth,
            Vector2d.Right);
    }

    private static Fixed64 ResolveClosingVelocityAfterResponse(Fixed64 threshold, Fixed64 initialVelocity)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        context.Settings.RestitutionVelocityThreshold = threshold;
        SolidBody2D moving = CreateBox(context, Vector2d.Zero);
        SolidBody2D wall = CreateBox(context, new Vector2d((Fixed64)2, Fixed64.Zero), immovable: true);
        moving.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        wall.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        moving.ApplyCollisionLinearVelocityDelta(new Vector2d(initialVelocity, Fixed64.Zero));
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        pair.Manifold.SetContact(Vector2d.Right, Vector2d.Right, Fixed64.Half, Vector2d.Right);

        pair.MarkColliding(context.FrameCount);

        return moving.LinearVelocity.X;
    }

    private static SolidBody2D CreateBox(
        GravitasWorldContext context,
        Vector2d position,
        bool immovable = false,
        bool isKinematic = false)
    {
        return CreateBody(
            context,
            new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)),
            position,
            immovable,
            isKinematic);
    }

    private static SolidBody2D CreateBody(
        GravitasWorldContext context,
        LSCollider2D collider,
        Vector2d position,
        bool immovable = false,
        bool isKinematic = false)
    {
        var body = new SolidBody2D(
            new TestMatterAgent(context, new FixedTransform(
                new Vector3d(position.X, Fixed64.Zero, position.Y),
                FixedQuaternion.Identity,
                Vector3d.One)),
            collider)
        {
            Mass = Fixed64.One,
            FreezeAxes = immovable ? BodyFreezeAxes2D.Position : BodyFreezeAxes2D.None,
            IsKinematic = isKinematic
        };
        body.Initialize(position);
        return body;
    }

    private static long MeasureAllocatedBytes(Action action)
        => AllocationTestHelper.MeasureSinglePass(action);
}
