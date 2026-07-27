using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Diagnostics;
using Gravitas.Support;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed partial class SolidBody2DGroundingTests
{
    private static readonly Vector2d Up = Vector2d.Forward;

    [Fact]
    public void Initialize_WithoutSupport_ShouldStartUngrounded()
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateCircle(context, new Vector2d(Fixed64.Zero, (Fixed64)4));

        body.IsGrounded.Should().BeFalse();
        body.WasGrounded.Should().BeFalse();
        body.GroundNormal.Should().Be(Vector2d.Zero);
        body.HasGroundPoint.Should().BeFalse();
    }

    [Fact]
    public void ManualGrounding_ShouldPreserveHostOwnedStateUntilChanged()
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateCircle(context, new Vector2d(Fixed64.Zero, Fixed64.One));
        Vector2d point = new(Fixed64.Zero, Fixed64.Half);

        body.SetManualGrounding(point, Up);
        Step(context);

        body.GroundingMode.Should().Be(GroundingMode.Manual);
        body.IsGrounded.Should().BeTrue();
        body.WasGrounded.Should().BeFalse();
        body.GroundPoint.Should().Be(point);
        body.TryGetGroundPoint(out Vector2d resolvedPoint).Should().BeTrue();
        resolvedPoint.Should().Be(point);
        body.GroundNormal.Should().Be(Up);

        body.ClearManualGrounding();
        Step(context);

        body.GroundingMode.Should().Be(GroundingMode.Manual);
        body.IsGrounded.Should().BeFalse();
        body.WasGrounded.Should().BeTrue();
        body.TryGetGroundPoint(out Vector2d clearedPoint).Should().BeFalse();
        clearedPoint.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void CheckGround_InManualMode_ShouldLeaveHostOwnedStateUnchanged()
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateCircle(context, new Vector2d(Fixed64.Zero, Fixed64.One));
        Vector2d point = new(Fixed64.Half, Fixed64.Half);
        Vector2d normal = new(Fixed64.One, Fixed64.One);
        int groundedChanges = 0;
        body.OnGrounded += _ => groundedChanges++;

        body.SetManualGrounding(point, normal);
        groundedChanges.Should().Be(1);
        body.WasGrounded.Should().BeFalse();

        body.CheckGround();

        body.GroundingMode.Should().Be(GroundingMode.Manual);
        body.IsGrounded.Should().BeTrue();
        body.WasGrounded.Should().BeFalse();
        body.GroundPoint.Should().Be(point);
        body.GroundNormal.Should().Be(normal.Normalized);
        groundedChanges.Should().Be(1);
    }

    [Fact]
    public void UseManualGrounding_ShouldClearStateAndIgnoreAutomaticSupport()
    {
        using GravitasWorldContext context = CreateContext();
        CreateStaticFloor(context);
        SolidBody2D body = CreateCircle(context, new Vector2d(Fixed64.Zero, Fixed64.One));

        body.CheckGround();
        body.IsGrounded.Should().BeTrue();

        body.UseManualGrounding();
        Step(context);

        body.GroundingMode.Should().Be(GroundingMode.Manual);
        body.IsGrounded.Should().BeFalse();
        body.WasGrounded.Should().BeTrue();

        body.UseAutomaticGrounding();

        body.GroundingMode.Should().Be(GroundingMode.Automatic);
        body.IsGrounded.Should().BeTrue();
    }

    [Fact]
    public void GroundingModeTransitions_ShouldRespectNoClearAndNoImmediateRefreshOptions()
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateCircle(context, new Vector2d(Fixed64.Zero, Fixed64.One));
        int groundedChanges = 0;
        body.OnGrounded += _ => groundedChanges++;

        body.SetManualGrounding(Vector2d.Zero, Up);
        body.UseManualGrounding(clearGrounding: false);
        body.UseAutomaticGrounding(checkGroundImmediately: false);

        body.IsGrounded.Should().BeTrue();
        body.GroundingMode.Should().Be(GroundingMode.Automatic);
        groundedChanges.Should().Be(1);
    }

    [Fact]
    public void UseAutomaticGrounding_WhenInactive_ShouldOnlyChangeOwnershipMode()
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);
        body.UseManualGrounding();
        body.Deactivate();

        body.UseAutomaticGrounding();

        body.Active.Should().BeFalse();
        body.GroundingMode.Should().Be(GroundingMode.Automatic);
        body.IsGrounded.Should().BeFalse();
    }

    [Fact]
    public void Initialize_WhenReusingManualBody_ShouldNotProbeAutomaticSupport()
    {
        using GravitasWorldContext context = CreateContext();
        CreateStaticFloor(context);
        SolidBody2D body = CreateCircle(context, new Vector2d(Fixed64.Zero, Fixed64.One));
        body.UseManualGrounding();
        body.Deactivate();

        body.Initialize(new Vector2d(Fixed64.Zero, Fixed64.One));

        body.GroundingMode.Should().Be(GroundingMode.Manual);
        body.IsGrounded.Should().BeFalse();
        body.HasGroundPoint.Should().BeFalse();
        body.GroundNormal.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void AutomaticGrounding_WhenSupportIsLost_ShouldExposeWasGroundedForStep()
    {
        using GravitasWorldContext context = CreateContext();
        CreateStaticFloor(context, layer: new PhysicsLayer(1));
        context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        SolidBody2D body = CreateCircle(context, new Vector2d(Fixed64.Zero, Fixed64.One));

        body.IsGrounded.Should().BeTrue();

        context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(2);
        Step(context);

        body.IsGrounded.Should().BeFalse();
        body.WasGrounded.Should().BeTrue();
    }

    [Fact]
    public void AutomaticGrounding_WhenBodyBecomesKinematic_ShouldClearStaleSupport()
    {
        using GravitasWorldContext context = CreateContext();
        CreateStaticFloor(context, layer: new PhysicsLayer(1));
        context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        SolidBody2D body = CreateCircle(context, new Vector2d(Fixed64.Zero, Fixed64.One));

        body.IsGrounded.Should().BeTrue();

        body.SetMotionType(BodyMotionType.Kinematic);
        Step(context);

        body.IsGrounded.Should().BeFalse();
        body.WasGrounded.Should().BeTrue();
    }

    [Fact]
    public void ContactSupport_ShouldGroundAgainstStaticFloorButRejectWallsAndCeilings()
    {
        using GravitasWorldContext floorContext = CreateContext();
        SolidBody2D floorBody = CreateStaticBox(
            floorContext,
            new Vector2d(Fixed64.Zero, -Fixed64.Half),
            new Vector2d((Fixed64)4, Fixed64.One));
        SolidBody2D floorTouching = CreateCircle(floorContext, Vector2d.Zero);
        DisableProbeFallback(floorTouching);

        Step(floorContext);

        floorTouching.IsGrounded.Should().BeTrue();
        floorTouching.GroundNormal.Y.Should().BeGreaterThan(Fixed64.Zero);
        floorTouching.GroundPoint.Y.Should().BeGreaterThanOrEqualTo(floorBody.Position.Y);
        floorBody.IsGrounded.Should().BeFalse();

        using GravitasWorldContext wallContext = CreateContext();
        SolidBody2D wallBody = CreateStaticBox(
            wallContext,
            new Vector2d(Fixed64.Half, Fixed64.Zero),
            new Vector2d(Fixed64.One, (Fixed64)4));
        SolidBody2D wallTouching = CreateCircle(wallContext, Vector2d.Zero);
        DisableProbeFallback(wallTouching);

        Step(wallContext);

        wallTouching.IsGrounded.Should().BeFalse();
        wallBody.IsGrounded.Should().BeFalse();

        using GravitasWorldContext ceilingContext = CreateContext();
        SolidBody2D ceilingBody = CreateStaticBox(
            ceilingContext,
            new Vector2d(Fixed64.Zero, Fixed64.Half),
            new Vector2d((Fixed64)4, Fixed64.One));
        SolidBody2D ceilingTouching = CreateCircle(ceilingContext, Vector2d.Zero);
        DisableProbeFallback(ceilingTouching);

        Step(ceilingContext);

        ceilingTouching.IsGrounded.Should().BeFalse();
        ceilingBody.IsGrounded.Should().BeFalse();
    }

    [Fact]
    public void ContactSupport_ShouldRejectOrdinaryDynamicBodiesAsGround()
    {
        using GravitasWorldContext context = CreateContext();
        _ = CreateCircle(context, Vector2d.Zero);
        SolidBody2D body = CreateCircle(context, new Vector2d(Fixed64.Zero, Fixed64.One));

        Step(context);

        body.IsGrounded.Should().BeFalse();
    }

    [Fact]
    public void ContactSupport_ShouldIgnoreTriggerPairsDuringDiscreteGroundingRefresh()
    {
        using GravitasWorldContext context = CreateContext();
        LSAABBoxCollider2D triggerFloor = CreateStaticFloor(
            context,
            center: new Vector2d(Fixed64.Zero, -Fixed64.Half),
            size: new Vector2d((Fixed64)4, Fixed64.One));
        triggerFloor.IsTrigger = true;
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);
        DisableProbeFallback(body);

        Step(context);

        body.IsGrounded.Should().BeFalse();
        triggerFloor.IsTrigger.Should().BeTrue();
    }

    [Fact]
    public void DiscreteGroundingPairPolicy_ShouldRejectStaleAndTriggerPairs()
    {
        using GravitasWorldContext context = CreateContext();
        LSAABBoxCollider2D first = CreateStaticFloor(context);
        LSAABBoxCollider2D second = CreateStaticFloor(context, center: Vector2d.Right * (Fixed64)3);
        var pair = new CollisionPair2D(first, second);
        pair.MarkResting(frame: 12);

        GravitasPhysics2DService.ShouldUseDiscreteGroundingPair(pair, frame: 12).Should().BeTrue();
        GravitasPhysics2DService.ShouldUseDiscreteGroundingPair(pair, frame: 13).Should().BeFalse();

        first.IsTrigger = true;
        GravitasPhysics2DService.ShouldUseDiscreteGroundingPair(pair, frame: 12).Should().BeFalse();

        first.IsTrigger = false;
        second.IsTrigger = true;
        GravitasPhysics2DService.ShouldUseDiscreteGroundingPair(pair, frame: 12).Should().BeFalse();
    }

    [Fact]
    public void DiscreteGroundingManifoldPolicy_ShouldRequireCurrentContact()
    {
        var manifold = new ContactManifold2D();

        manifold.BeginUpdate(frame: 12);
        GravitasPhysics2DService.ShouldUseDiscreteGroundingManifold(manifold, frame: 12).Should().BeFalse();

        manifold.SetContact(Vector2d.Zero, Vector2d.Zero, Fixed64.Half, -Up);
        GravitasPhysics2DService.ShouldUseDiscreteGroundingManifold(manifold, frame: 12).Should().BeTrue();
        GravitasPhysics2DService.ShouldUseDiscreteGroundingManifold(manifold, frame: 13).Should().BeFalse();
    }

    [Fact]
    public void RayProbe_ShouldGroundAgainstClosestStaticSupport()
    {
        using GravitasWorldContext context = CreateContext();
        CreateStaticFloor(context, center: new Vector2d(Fixed64.Zero, Fixed64.Zero), layer: new PhysicsLayer(1));
        CreateStaticFloor(context, center: new Vector2d(Fixed64.Zero, -Fixed64.One), layer: new PhysicsLayer(1));
        context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        SolidBody2D body = CreateCircle(context, new Vector2d(Fixed64.Zero, Fixed64.One));
        body.GroundProbeMode = GroundProbeMode2D.Ray;
        body.GroundedDistanceRay = (Fixed64)2;

        body.CheckGround();

        body.IsGrounded.Should().BeTrue();
        body.GroundPoint.Y.Should().Be(Fixed64.Half);
        body.GroundNormal.Should().Be(Up);
    }

    [Fact]
    public void RayProbe_ShouldRejectTriggerSupport()
    {
        using GravitasWorldContext context = CreateContext();
        LSAABBoxCollider2D triggerFloor = CreateStaticFloor(context, layer: new PhysicsLayer(1));
        triggerFloor.IsTrigger = true;
        context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        SolidBody2D body = CreateCircle(context, new Vector2d(Fixed64.Zero, Fixed64.One));
        body.GroundProbeMode = GroundProbeMode2D.Ray;
        body.GroundedDistanceRay = (Fixed64)2;

        body.CheckGround();

        body.IsGrounded.Should().BeFalse();
        body.WasGrounded.Should().BeFalse();
    }

    [Fact]
    public void AutomaticRefresh_ShouldReuseCachedGroundWithinFrameUntilForceProbe()
    {
        using GravitasWorldContext context = CreateContext();
        CreateStaticFloor(context, layer: new PhysicsLayer(1));
        context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        SolidBody2D body = CreateCircle(context, new Vector2d(Fixed64.Zero, Fixed64.One));

        body.CheckGround();
        body.IsGrounded.Should().BeTrue();

        body.GroundedDistanceRay = Fixed64.Zero;
        body.BeginAutomaticGroundingRefresh();
        body.CompleteAutomaticGroundingRefresh();

        body.IsGrounded.Should().BeTrue();

        body.CheckGround();

        body.IsGrounded.Should().BeFalse();
        body.WasGrounded.Should().BeTrue();
    }

    [Fact]
    public void AutomaticRefresh_WhenCachedSupportIsDeactivated_ShouldClearGroundState()
    {
        using GravitasWorldContext context = CreateContext();
        LSAABBoxCollider2D floor = CreateStaticFloor(context, layer: new PhysicsLayer(1));
        context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        SolidBody2D body = CreateCircle(context, new Vector2d(Fixed64.Zero, Fixed64.One));
        body.IsGrounded.Should().BeTrue();

        floor.Deactivate();
        body.BeginAutomaticGroundingRefresh();
        body.CompleteAutomaticGroundingRefresh();

        body.IsGrounded.Should().BeFalse();
        body.WasGrounded.Should().BeTrue();
        body.HasGroundPoint.Should().BeFalse();
        body.GroundNormal.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void ContactGroundCandidate_ShouldRejectLayerMaskedAndLocallyIgnoredSupport()
    {
        using GravitasWorldContext layerContext = CreateContext();
        SolidBody2D layerBody = CreateCircle(layerContext, Vector2d.Zero);
        LSAABBoxCollider2D wrongLayerSupport = CreateStaticFloor(layerContext, layer: new PhysicsLayer(2));
        layerContext.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        layerBody.BeginAutomaticGroundingRefresh();
        layerBody.TryAcceptContactGroundCandidate(
            layerBody.Collider,
            wrongLayerSupport,
            CreateGroundContact(1, Vector2d.Zero, Fixed64.Half, Up),
            ownColliderIsA: true);
        layerBody.CompleteAutomaticGroundingRefresh();

        layerBody.IsGrounded.Should().BeFalse();

        using GravitasWorldContext ignoredContext = CreateContext();
        SolidBody2D ignoredBody = CreateCircle(ignoredContext, Vector2d.Zero);
        LSAABBoxCollider2D ignoredSupport = CreateStaticFloor(ignoredContext, layer: new PhysicsLayer(1));
        ignoredContext.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        ignoredBody.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(ignoredSupport.Layer);

        ignoredBody.BeginAutomaticGroundingRefresh();
        ignoredBody.TryAcceptContactGroundCandidate(
            ignoredBody.Collider,
            ignoredSupport,
            CreateGroundContact(1, Vector2d.Zero, Fixed64.Half, Up),
            ownColliderIsA: true);
        ignoredBody.CompleteAutomaticGroundingRefresh();

        ignoredBody.IsGrounded.Should().BeFalse();
    }

    [Fact]
    public void ContactGroundCandidate_WithZeroNormal_ShouldBeRejected()
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);
        LSAABBoxCollider2D support = CreateStaticFloor(context);
        DisableProbeFallback(body);

        body.BeginAutomaticGroundingRefresh();
        body.TryAcceptContactGroundCandidate(
            body.Collider,
            support,
            CreateGroundContact(1, Vector2d.One, Fixed64.Half, Vector2d.Zero),
            ownColliderIsA: true);
        body.CompleteAutomaticGroundingRefresh();

        body.IsGrounded.Should().BeFalse();
        body.HasGroundPoint.Should().BeFalse();
        body.GroundNormal.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void ContactGroundCandidate_WithoutRepresentableWorldPoint_ShouldStillGround()
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);
        LSAABBoxCollider2D support = CreateStaticFloor(context);
        DisableProbeFallback(body);
        var unavailablePoint = new ContactAnchor2D(
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            Vector2d.Right);
        var contact = new ManifoldContact2D(
            1,
            unavailablePoint,
            unavailablePoint,
            Fixed64.Half,
            -Up);

        contact.TryGetPointA(out _).Should().BeFalse();
        body.BeginAutomaticGroundingRefresh();
        body.TryAcceptContactGroundCandidate(
            body.Collider,
            support,
            contact,
            ownColliderIsA: true);
        body.CompleteAutomaticGroundingRefresh();

        body.IsGrounded.Should().BeTrue();
        body.GroundNormal.Should().Be(Up);
        body.HasGroundPoint.Should().BeFalse();
        body.TryGetGroundPoint(out Vector2d point).Should().BeFalse();
        point.Should().Be(Vector2d.Zero);
        Action getGroundPoint = () => _ = body.GroundPoint;
        getGroundPoint.Should().Throw<InvalidOperationException>()
            .WithMessage("*TryGetGroundPoint*");
    }

    [Fact]
    public void ContactGroundCandidate_WithZeroGravity_ShouldUseExplicitFallbackUpDirection()
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);
        LSAABBoxCollider2D support = CreateStaticFloor(context);
        DisableProbeFallback(body);
        body.Gravity = Vector2d.Zero;
        body.UseGravityDerivedGroundUpDirection = true;
        body.GroundUpDirection = Vector2d.Right;

        body.BeginAutomaticGroundingRefresh();
        body.TryAcceptContactGroundCandidate(
            body.Collider,
            support,
            CreateGroundContact(1, Vector2d.Zero, Fixed64.Half, Vector2d.Right),
            ownColliderIsA: true);
        body.CompleteAutomaticGroundingRefresh();

        body.IsGrounded.Should().BeTrue();
        body.GroundNormal.Should().Be(Vector2d.Right);
    }

    [Fact]
    public void SweptCircleProbe_ShouldGroundWhenCenterRayMissesSupportEdge()
    {
        using GravitasWorldContext context = CreateContext();
        CreateStaticFloor(
            context,
            center: new Vector2d((Fixed64)1.25f, Fixed64.Zero),
            size: new Vector2d((Fixed64)2, Fixed64.One),
            layer: new PhysicsLayer(1));
        context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        SolidBody2D body = CreateCircle(context, new Vector2d(Fixed64.Zero, Fixed64.One));

        body.GroundProbeMode = GroundProbeMode2D.Ray;
        body.CheckGround();
        body.IsGrounded.Should().BeFalse();

        body.GroundProbeMode = GroundProbeMode2D.SweptCircle;
        body.GroundProbeRadius = Fixed64.Half;
        body.CheckGround();

        body.IsGrounded.Should().BeTrue();
        body.GroundNormal.Y.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void GroundedGravity_ShouldRemoveVelocityIntoSupportNormal()
    {
        using GravitasWorldContext context = CreateContext(frameRate: 4);
        CreateStaticFloor(context);
        SolidBody2D body = CreateCircle(context, new Vector2d(Fixed64.Zero, Fixed64.One));
        body.Gravity = new Vector2d(Fixed64.Zero, (Fixed64)(-4));
        body.CheckGround();

        context.LateSimulate();

        body.IsGrounded.Should().BeTrue();
        body.LinearVelocity.Y.Should().Be(Fixed64.Zero);
        body.Position.Y.Should().Be(Fixed64.One);
    }

    [Fact]
    public void ContactSupport_ShouldChooseBestCandidateByNormalDepthColliderAndContactId()
    {
        SubmitCandidatePairAndGetGroundPoint(
                firstNormal: Up,
                firstDepth: Fixed64.Half,
                firstContactId: 10,
                secondNormal: new Vector2d(Fixed64.Half, Fixed64.Half).Normalized,
                secondDepth: (Fixed64)2,
                secondContactId: 1)
            .Should()
            .Be(new Vector2d(Fixed64.Zero, Fixed64.One));

        SubmitCandidatePairAndGetGroundPoint(
                firstNormal: new Vector2d(Fixed64.Half, Fixed64.Half).Normalized,
                firstDepth: (Fixed64)2,
                firstContactId: 10,
                secondNormal: Up,
                secondDepth: Fixed64.Half,
                secondContactId: 1)
            .Should()
            .Be(new Vector2d(Fixed64.One, Fixed64.One));

        SubmitCandidatePairAndGetGroundPoint(
                firstNormal: Up,
                firstDepth: Fixed64.Half,
                firstContactId: 10,
                secondNormal: Up,
                secondDepth: Fixed64.One,
                secondContactId: 1)
            .Should()
            .Be(new Vector2d(Fixed64.One, Fixed64.One));

        SubmitCandidatePairAndGetGroundPoint(
                firstNormal: Up,
                firstDepth: Fixed64.One,
                firstContactId: 20,
                secondNormal: Up,
                secondDepth: Fixed64.One,
                secondContactId: 10,
                reuseFirstSupportForSecondCandidate: true)
            .Should()
            .Be(new Vector2d(Fixed64.One, Fixed64.One));

        SubmitCandidatePairAndGetGroundPoint(
                firstNormal: Up,
                firstDepth: Fixed64.One,
                firstContactId: 10,
                secondNormal: Up,
                secondDepth: Fixed64.One,
                secondContactId: 1,
                submitLowerColliderIdSecond: true)
            .Should()
            .Be(new Vector2d(Fixed64.One, Fixed64.One));

        SubmitCandidatePairAndGetGroundPoint(
                firstNormal: Up,
                firstDepth: Fixed64.One,
                firstContactId: 1,
                secondNormal: new Vector2d(Fixed64.Half, Fixed64.Half).Normalized,
                secondDepth: (Fixed64)2,
                secondContactId: 10)
            .Should()
            .Be(new Vector2d(Fixed64.Zero, Fixed64.One));

        SubmitCandidatePairAndGetGroundPoint(
                firstNormal: Up,
                firstDepth: Fixed64.One,
                firstContactId: 1,
                secondNormal: Up,
                secondDepth: Fixed64.Half,
                secondContactId: 10)
            .Should()
            .Be(new Vector2d(Fixed64.Zero, Fixed64.One));

        SubmitCandidatePairAndGetGroundPoint(
                firstNormal: Up,
                firstDepth: Fixed64.One,
                firstContactId: 1,
                secondNormal: Up,
                secondDepth: Fixed64.One,
                secondContactId: 10)
            .Should()
            .Be(new Vector2d(Fixed64.Zero, Fixed64.One));

        SubmitCandidatePairAndGetGroundPoint(
                firstNormal: Up,
                firstDepth: Fixed64.One,
                firstContactId: 10,
                secondNormal: Up,
                secondDepth: Fixed64.One,
                secondContactId: 10,
                reuseFirstSupportForSecondCandidate: true)
            .Should()
            .Be(new Vector2d(Fixed64.Zero, Fixed64.One));

        SubmitCandidatePairAndGetGroundPoint(
                firstNormal: Up,
                firstDepth: Fixed64.One,
                firstContactId: 1,
                secondNormal: Up,
                secondDepth: Fixed64.One,
                secondContactId: 10,
                reuseFirstSupportForSecondCandidate: true)
            .Should()
            .Be(new Vector2d(Fixed64.Zero, Fixed64.One));
    }

    [Fact]
    public void GroundProbeDiagnostics_ShouldEmit2DProbeShape()
    {
        using GravitasWorldContext context = CreateContext();
        context.Diagnostics.Enable();
        CreateStaticFloor(context, layer: new PhysicsLayer(1));
        context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        SolidBody2D body = CreateCircle(context, new Vector2d(Fixed64.Zero, Fixed64.One));
        body.GroundProbeMode = GroundProbeMode2D.Ray;

        body.CheckGround();

        context.Diagnostics.Events.Should().Contain(e =>
            e.Kind == GravitasDiagnosticEventKind.GroundProbe
            && e.DataA == (int)GroundProbeMode2D.Ray
            && e.DataB == (int)GravitasColliderDimension.TwoD);
    }

    [Fact]
    public void AutomaticProbe_WithUnsupportedColliderShape_ShouldResolveToZeroRadiusRay()
    {
        using GravitasWorldContext context = CreateContext();
        context.Diagnostics.Enable();
        Vector2d position = new(Fixed64.Zero, Fixed64.One);
        var agent = new TestMatterAgent(context, new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One));
        var body = new SolidBody2D(agent, new UnsupportedTestCollider2D())
        {
            Mass = Fixed64.One
        };

        body.Initialize(position);

        body.GroundProbeMode.Should().Be(GroundProbeMode2D.Auto);
        context.Diagnostics.Events.Length.Should().Be(1);
        GravitasDiagnosticEvent probe = context.Diagnostics.Events[0];
        probe.Kind.Should().Be(GravitasDiagnosticEventKind.GroundProbe);
        probe.DataA.Should().Be((int)GroundProbeMode2D.Ray);
        probe.ScalarA.Should().Be(Fixed64.Zero);
        body.IsGrounded.Should().BeFalse();
    }

    [Fact]
    public void CheckGround_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = CreateContext();
        CreateStaticFloor(context, layer: new PhysicsLayer(1));
        context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        SolidBody2D body = CreateCircle(context, new Vector2d(Fixed64.Zero, Fixed64.One));
        body.GroundProbeMode = GroundProbeMode2D.SweptCircle;
        body.GroundProbeRadius = Fixed64.Half;

        for (int i = 0; i < 8; i++)
            body.CheckGround();

        long allocatedBytes = MeasureAllocatedBytes(body.CheckGround);

        allocatedBytes.Should().Be(0);
    }

    private static GravitasWorldContext CreateContext(int frameRate = 4) =>
        Physics2DTestWorld.CreateContext(frameRate);

    private static void Step(GravitasWorldContext context)
    {
        context.Simulate();
        context.LateSimulate();
    }

    private static SolidBody2D CreateCircle(
        GravitasWorldContext context,
        Vector2d position)
    {
        var agent = new TestMatterAgent(context, new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One));
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One
        };
        body.Initialize(position);
        return body;
    }

    private static SolidBody2D CreateStaticBox(
        GravitasWorldContext context,
        Vector2d position,
        Vector2d size)
    {
        var agent = new TestMatterAgent(context, new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One));
        var body = new SolidBody2D(agent, new LSAABBoxCollider2D(size))
        {
            Mass = Fixed64.One
        };
        body.Initialize(position, motionType: BodyMotionType.Static);
        return body;
    }

    private static LSAABBoxCollider2D CreateStaticFloor(
        GravitasWorldContext context,
        Vector2d? center = null,
        Vector2d? size = null,
        PhysicsLayer? layer = null)
    {
        Vector2d position = center ?? Vector2d.Zero;
        var agent = new TestMatterAgent(context, new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One));
        var collider = new LSAABBoxCollider2D(size ?? new Vector2d((Fixed64)8, Fixed64.One))
        {
            Layer = layer ?? new PhysicsLayer(0)
        };
        collider.InitializeWithNoBody(agent);
        return collider;
    }

    private static void ReinitializeStaticFloor(
        GravitasWorldContext context,
        LSAABBoxCollider2D collider,
        Vector2d position)
    {
        var agent = new TestMatterAgent(context, new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One));
        collider.InitializeWithNoBody(agent);
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

    private static void DisableProbeFallback(SolidBody2D body)
    {
        body.GroundedDistanceRay = Fixed64.Zero;
        body.GroundDownDistanceOnAir = Fixed64.Zero;
    }

    private static Vector2d SubmitCandidatePairAndGetGroundPoint(
        Vector2d firstNormal,
        Fixed64 firstDepth,
        ulong firstContactId,
        Vector2d secondNormal,
        Fixed64 secondDepth,
        ulong secondContactId,
        bool submitLowerColliderIdSecond = false,
        bool reuseFirstSupportForSecondCandidate = false)
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);
        DisableProbeFallback(body);
        LSAABBoxCollider2D lowerIdSupport = CreateStaticFloor(context, center: new Vector2d((Fixed64)(-8), Fixed64.Zero));
        LSAABBoxCollider2D higherIdSupport = reuseFirstSupportForSecondCandidate
            ? lowerIdSupport
            : CreateStaticFloor(context, center: new Vector2d((Fixed64)8, Fixed64.Zero));
        LSAABBoxCollider2D firstSupport = submitLowerColliderIdSecond ? higherIdSupport : lowerIdSupport;
        LSAABBoxCollider2D secondSupport = submitLowerColliderIdSecond ? lowerIdSupport : higherIdSupport;

        body.BeginAutomaticGroundingRefresh();
        body.TryAcceptContactGroundCandidate(
            body.Collider,
            firstSupport,
            CreateGroundContact(firstContactId, new Vector2d(Fixed64.Zero, Fixed64.One), firstDepth, firstNormal),
            ownColliderIsA: true);
        body.TryAcceptContactGroundCandidate(
            body.Collider,
            secondSupport,
            CreateGroundContact(secondContactId, new Vector2d(Fixed64.One, Fixed64.One), secondDepth, secondNormal),
            ownColliderIsA: true);
        body.CompleteAutomaticGroundingRefresh();

        return body.GroundPoint;
    }

    private static ManifoldContact2D CreateGroundContact(
        ulong contactId,
        Vector2d point,
        Fixed64 depth,
        Vector2d groundNormal) =>
        new(contactId, point, point, depth, -groundNormal);
}
