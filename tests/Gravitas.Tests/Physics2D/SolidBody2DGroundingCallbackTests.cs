using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed partial class SolidBody2DGroundingTests
{
    [Theory]
    [InlineData(GroundProbeMode2D.Ray)]
    [InlineData(GroundProbeMode2D.SweptCircle)]
    public void QueryProbe_WhenSupportRebindsInGroundedCallback_ShouldRejectOldLifetime(GroundProbeMode2D mode)
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateCircle(context, new Vector2d(Fixed64.Zero, Fixed64.One));
        LSAABBoxCollider2D support = CreateStaticFloor(
            context,
            Vector2d.Zero,
            new Vector2d((Fixed64)2, Fixed64.One));
        body.GroundProbeMode = mode;
        body.GroundProbeRadius = Fixed64.Half;
        body.GroundDownDistanceOnAir = Fixed64.One;
        string events = string.Empty;
        body.OnGrounded += grounded =>
        {
            events += grounded ? "true;" : "false;";
            if (!grounded)
                return;

            support.Deactivate();
            ReinitializeStaticFloor(context, support, new Vector2d((Fixed64)16, Fixed64.Zero));
        };

        body.CheckGround();

        events.Should().Be("true;false;");
        support.IsActive.Should().BeTrue();
        body.GroundingMode.Should().Be(GroundingMode.Automatic);
        body.IsGrounded.Should().BeFalse();
        body.GroundPoint.Should().Be(Vector2d.Zero);
        body.GroundNormal.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void ClearManualGrounding_WhenCallbackReestablishesGround_ShouldPreserveCallbackState()
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);
        Vector2d replacementPoint = new(Fixed64.One, Fixed64.Half);
        Vector2d replacementNormal = new(Fixed64.One, Fixed64.One);
        body.SetManualGrounding(Vector2d.Zero, Up);
        string events = string.Empty;
        body.OnGrounded += grounded =>
        {
            events += grounded ? "true;" : "false;";
            if (!grounded)
                body.SetManualGrounding(replacementPoint, replacementNormal);
        };

        body.ClearManualGrounding();

        events.Should().Be("false;true;");
        body.GroundingMode.Should().Be(GroundingMode.Manual);
        body.IsGrounded.Should().BeTrue();
        body.GroundPoint.Should().Be(replacementPoint);
        body.GroundNormal.Should().Be(replacementNormal.Normalized);
    }

    [Fact]
    public void CheckGround_WhenCallbackReestablishesAutomaticGround_ShouldPreserveCallbackState()
    {
        using GravitasWorldContext context = CreateContext();
        CreateStaticFloor(context, layer: new PhysicsLayer(1));
        context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        SolidBody2D body = CreateCircle(context, new Vector2d(Fixed64.Zero, Fixed64.One));
        Vector2d expectedPoint = body.GroundPoint;
        string events = string.Empty;
        body.OnGrounded += grounded =>
        {
            events += grounded ? "true;" : "false;";
            if (!grounded)
            {
                body.GroundedDistanceRay = (Fixed64)2;
                body.UseAutomaticGrounding();
            }
        };
        body.GroundedDistanceRay = Fixed64.Zero;

        body.CheckGround();

        events.Should().Be("false;true;");
        body.GroundingMode.Should().Be(GroundingMode.Automatic);
        body.IsGrounded.Should().BeTrue();
        body.GroundPoint.Should().Be(expectedPoint);
        body.GroundNormal.Should().Be(Up);
    }

    [Fact]
    public void GroundingCallbacks_WhenBodiesDeactivate_ShouldFinishTheCapturedRefreshSet()
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D first = CreateCircle(context, Vector2d.Zero);
        SolidBody2D survivor = CreateCircle(context, new Vector2d((Fixed64)8, Fixed64.Zero));
        SolidBody2D deactivatedOther = CreateCircle(context, new Vector2d((Fixed64)16, Fixed64.Zero));
        DisableProbeFallback(first);
        DisableProbeFallback(survivor);
        DisableProbeFallback(deactivatedOther);
        CreateStaticFloor(context, new Vector2d(Fixed64.Zero, -Fixed64.Half), new Vector2d((Fixed64)2, Fixed64.One));
        CreateStaticFloor(context, new Vector2d((Fixed64)8, -Fixed64.Half), new Vector2d((Fixed64)2, Fixed64.One));
        CreateStaticFloor(context, new Vector2d((Fixed64)16, -Fixed64.Half), new Vector2d((Fixed64)2, Fixed64.One));
        first.OnGrounded += grounded =>
        {
            if (!grounded)
                return;

            first.Deactivate();
            deactivatedOther.Deactivate();
        };

        Action step = () => Step(context);

        step.Should().NotThrow();
        first.Active.Should().BeFalse();
        deactivatedOther.Active.Should().BeFalse();
        survivor.Active.Should().BeTrue();
        survivor.IsGrounded.Should().BeTrue();
        survivor.GroundNormal.Should().Be(Up);
    }

    [Fact]
    public void GroundingCallback_WhenLaterCandidateSupportDeactivates_ShouldRejectThatCandidate()
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D first = CreateCircle(context, Vector2d.Zero);
        SolidBody2D later = CreateCircle(context, new Vector2d((Fixed64)8, Fixed64.Zero));
        DisableProbeFallback(first);
        DisableProbeFallback(later);
        CreateStaticFloor(
            context,
            new Vector2d(Fixed64.Zero, -Fixed64.Half),
            new Vector2d((Fixed64)2, Fixed64.One));
        LSAABBoxCollider2D laterSupport = CreateStaticFloor(
            context,
            new Vector2d((Fixed64)8, -Fixed64.Half),
            new Vector2d((Fixed64)2, Fixed64.One));
        bool laterWasGroundedWhenSupportDeactivated = false;
        first.OnGrounded += grounded =>
        {
            if (grounded)
            {
                laterWasGroundedWhenSupportDeactivated = later.IsGrounded;
                laterSupport.Deactivate();
            }
        };

        Step(context);

        first.IsGrounded.Should().BeTrue();
        laterWasGroundedWhenSupportDeactivated.Should().BeFalse();
        laterSupport.IsActive.Should().BeFalse();
        later.GroundPoint.Should().Be(Vector2d.Zero);
        later.GroundNormal.Should().Be(Vector2d.Zero);
        later.IsGrounded.Should().BeFalse();
    }

    [Fact]
    public void GroundingCallback_WithRecursiveLateSimulate_ShouldPreserveTheOuterSnapshotRange()
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D first = CreateCircle(context, Vector2d.Zero);
        SolidBody2D outerSurvivor = CreateCircle(context, new Vector2d((Fixed64)8, Fixed64.Zero));
        DisableProbeFallback(first);
        DisableProbeFallback(outerSurvivor);
        CreateStaticFloor(
            context,
            new Vector2d(Fixed64.Zero, -Fixed64.Half),
            new Vector2d((Fixed64)2, Fixed64.One));
        CreateStaticFloor(
            context,
            new Vector2d((Fixed64)8, -Fixed64.Half),
            new Vector2d((Fixed64)2, Fixed64.One));
        SolidBody2D? recursiveBody = null;
        LSAABBoxCollider2D? recursiveSupport = null;
        first.OnGrounded += grounded =>
        {
            if (!grounded || recursiveBody != null)
                return;

            recursiveBody = CreateCircle(context, new Vector2d((Fixed64)16, Fixed64.Zero));
            DisableProbeFallback(recursiveBody);
            recursiveSupport = CreateStaticFloor(
                context,
                new Vector2d((Fixed64)16, -Fixed64.Half),
                new Vector2d((Fixed64)2, Fixed64.One));
            context.LateSimulate();
            recursiveBody.IsGrounded.Should().BeTrue();
            recursiveSupport.Deactivate();
        };

        Step(context);

        outerSurvivor.IsGrounded.Should().BeTrue();
        recursiveBody.Should().NotBeNull();
        recursiveBody!.IsGrounded.Should().BeTrue(
            "the body created by the nested step is outside the outer refresh snapshot");
        recursiveSupport!.IsActive.Should().BeFalse();
    }

    [Fact]
    public void BeginGroundingCallback_WithRecursiveLateSimulate_ShouldPreserveOuterPairCandidates()
    {
        using GravitasWorldContext context = CreateContext();
        LSAABBoxCollider2D firstSupport = CreateStaticFloor(
            context,
            new Vector2d(Fixed64.Zero, -Fixed64.Half),
            new Vector2d((Fixed64)2, Fixed64.One));
        SolidBody2D first = CreateCircle(context, new Vector2d(Fixed64.Zero, Fixed64.One));
        first.CheckGround();
        first.IsGrounded.Should().BeTrue();
        SolidBody2D outerCandidate = CreateCircle(context, new Vector2d((Fixed64)8, Fixed64.Zero));
        DisableProbeFallback(outerCandidate);
        LSAABBoxCollider2D outerSupport = CreateStaticFloor(
            context,
            new Vector2d((Fixed64)8, -Fixed64.Half),
            new Vector2d((Fixed64)2, Fixed64.One));
        bool recursed = false;
        first.OnGrounded += grounded =>
        {
            if (grounded || recursed)
                return;

            recursed = true;
            outerSupport.IsTrigger = true;
            context.LateSimulate();
            outerCandidate.IsGrounded.Should().BeFalse();
            outerSupport.IsTrigger = false;
        };
        first.IsKinematic = true;

        Step(context);

        recursed.Should().BeTrue();
        firstSupport.IsActive.Should().BeTrue();
        outerSupport.IsTrigger.Should().BeFalse();
        outerCandidate.IsGrounded.Should().BeTrue();
        outerCandidate.GroundNormal.Should().Be(Up);
    }

    [Fact]
    public void ContactCandidate_WhenSupportRebindsInGroundedCallback_ShouldRejectOldLifetime()
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);
        DisableProbeFallback(body);
        LSAABBoxCollider2D support = CreateStaticFloor(
            context,
            new Vector2d(Fixed64.Zero, -Fixed64.Half),
            new Vector2d((Fixed64)2, Fixed64.One));
        string events = string.Empty;
        body.OnGrounded += grounded =>
        {
            events += grounded ? "true;" : "false;";
            if (!grounded)
                return;

            support.Deactivate();
            ReinitializeStaticFloor(context, support, new Vector2d((Fixed64)16, -Fixed64.Half));
        };

        Step(context);

        events.Should().Be("true;false;");
        support.IsActive.Should().BeTrue();
        body.GroundingMode.Should().Be(GroundingMode.Automatic);
        body.IsGrounded.Should().BeFalse();
        body.GroundPoint.Should().Be(Vector2d.Zero);
        body.GroundNormal.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void ContactCandidate_WhenCallbackTakesManualOwnership_ShouldPreserveManualState()
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);
        DisableProbeFallback(body);
        LSAABBoxCollider2D support = CreateStaticFloor(
            context,
            new Vector2d(Fixed64.Zero, -Fixed64.Half),
            new Vector2d((Fixed64)2, Fixed64.One));
        Vector2d manualPoint = new((Fixed64)3, (Fixed64)4);
        Vector2d manualNormal = new(Fixed64.One, Fixed64.One);
        string events = string.Empty;
        body.OnGrounded += grounded =>
        {
            events += grounded ? "true;" : "false;";
            if (!grounded)
                return;

            support.Deactivate();
            ReinitializeStaticFloor(context, support, new Vector2d((Fixed64)16, -Fixed64.Half));
            body.SetManualGrounding(manualPoint, manualNormal);
        };

        Step(context);

        events.Should().Be("true;");
        body.GroundingMode.Should().Be(GroundingMode.Manual);
        body.IsGrounded.Should().BeTrue();
        body.GroundPoint.Should().Be(manualPoint);
        body.GroundNormal.Should().Be(manualNormal.Normalized);
    }

    [Fact]
    public void ContactCandidate_WhenCallbackTakesManualOwnershipWithoutRewrite_ShouldPreserveAcceptedState()
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);
        DisableProbeFallback(body);
        CreateStaticFloor(
            context,
            new Vector2d(Fixed64.Zero, -Fixed64.Half),
            new Vector2d((Fixed64)2, Fixed64.One));
        Vector2d acceptedPoint = Vector2d.Zero;
        Vector2d acceptedNormal = Vector2d.Zero;
        string events = string.Empty;
        body.OnGrounded += grounded =>
        {
            events += grounded ? "true;" : "false;";
            if (!grounded)
                return;

            acceptedPoint = body.GroundPoint;
            acceptedNormal = body.GroundNormal;
            body.UseManualGrounding(clearGrounding: false);
        };

        Step(context);

        events.Should().Be("true;");
        body.GroundingMode.Should().Be(GroundingMode.Manual);
        body.IsGrounded.Should().BeTrue();
        body.GroundPoint.Should().Be(acceptedPoint);
        body.GroundNormal.Should().Be(acceptedNormal);
    }

    [Fact]
    public void ContactCandidate_WhenCallbackReprobesDifferentSupport_ShouldPreserveReplacementGround()
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);
        body.GroundProbeMode = GroundProbeMode2D.Ray;
        body.GroundedDistanceRay = (Fixed64)2;
        body.GroundDownDistanceOnAir = (Fixed64)2;
        LSAABBoxCollider2D contactSupport = CreateStaticFloor(
            context,
            new Vector2d(Fixed64.Zero, -Fixed64.Half),
            new Vector2d((Fixed64)2, Fixed64.One));
        LSAABBoxCollider2D replacementSupport = CreateStaticFloor(
            context,
            new Vector2d(Fixed64.Zero, (Fixed64)(-1.5f)),
            new Vector2d((Fixed64)2, Fixed64.One));
        bool reprobed = false;
        Vector2d replacementPoint = Vector2d.Zero;
        Vector2d replacementNormal = Vector2d.Zero;
        string events = string.Empty;
        body.OnGrounded += grounded =>
        {
            events += grounded ? "true;" : "false;";
            if (!grounded || reprobed)
                return;

            reprobed = true;
            contactSupport.Deactivate();
            body.CheckGround();
            body.IsGrounded.Should().BeTrue();
            replacementPoint = body.GroundPoint;
            replacementNormal = body.GroundNormal;
        };

        Step(context);

        reprobed.Should().BeTrue();
        events.Should().Be("true;");
        contactSupport.IsActive.Should().BeFalse();
        replacementSupport.IsActive.Should().BeTrue();
        body.GroundingMode.Should().Be(GroundingMode.Automatic);
        body.IsGrounded.Should().BeTrue();
        body.GroundPoint.Should().Be(replacementPoint);
        body.GroundNormal.Should().Be(replacementNormal);
    }

    [Fact]
    public void ContactCandidate_WhenGroundedCallbackDeactivatesBody_ShouldClearAutomaticGrounding()
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);
        DisableProbeFallback(body);
        CreateStaticFloor(
            context,
            new Vector2d(Fixed64.Zero, -Fixed64.Half),
            new Vector2d((Fixed64)2, Fixed64.One));
        string events = string.Empty;
        body.OnGrounded += grounded =>
        {
            events += grounded ? "true;" : "false;";
            if (grounded)
                body.Deactivate();
        };

        Step(context);

        events.Should().Be("true;false;");
        body.Active.Should().BeFalse();
        body.GroundingMode.Should().Be(GroundingMode.Automatic);
        body.IsGrounded.Should().BeFalse();
        body.GroundPoint.Should().Be(Vector2d.Zero);
        body.GroundNormal.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void ContactCandidate_WhenGroundedCallbackMakesBodyKinematic_ShouldClearAutomaticGrounding()
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);
        DisableProbeFallback(body);
        CreateStaticFloor(
            context,
            new Vector2d(Fixed64.Zero, -Fixed64.Half),
            new Vector2d((Fixed64)2, Fixed64.One));
        string events = string.Empty;
        body.OnGrounded += grounded =>
        {
            events += grounded ? "true;" : "false;";
            if (grounded)
                body.IsKinematic = true;
        };

        Step(context);

        events.Should().Be("true;false;");
        body.Active.Should().BeTrue();
        body.IsKinematic.Should().BeTrue();
        body.GroundingMode.Should().Be(GroundingMode.Automatic);
        body.IsGrounded.Should().BeFalse();
        body.GroundPoint.Should().Be(Vector2d.Zero);
        body.GroundNormal.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void ContactCandidate_WhenGroundedCallbackRebindsBody_ShouldPreserveNewLifetimeState()
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateCircle(context, Vector2d.Zero);
        DisableProbeFallback(body);
        CreateStaticFloor(
            context,
            new Vector2d(Fixed64.Zero, -Fixed64.Half),
            new Vector2d((Fixed64)2, Fixed64.One));
        CreateStaticFloor(
            context,
            new Vector2d((Fixed64)16, Fixed64.Zero),
            new Vector2d((Fixed64)2, Fixed64.One));
        bool rebound = false;
        Vector2d reboundPoint = Vector2d.Zero;
        Vector2d reboundNormal = Vector2d.Zero;
        string events = string.Empty;
        body.OnGrounded += grounded =>
        {
            events += grounded ? "true;" : "false;";
            if (!grounded || rebound)
                return;

            rebound = true;
            body.Deactivate();
            body.GroundedDistanceRay = Fixed64.One;
            body.GroundDownDistanceOnAir = Fixed64.One;
            body.Initialize(new Vector2d((Fixed64)16, Fixed64.One));
            reboundPoint = body.GroundPoint;
            reboundNormal = body.GroundNormal;
        };

        Step(context);

        events.Should().Be("true;true;");
        body.Active.Should().BeTrue();
        body.GroundingMode.Should().Be(GroundingMode.Automatic);
        body.IsGrounded.Should().BeTrue();
        body.GroundPoint.Should().Be(reboundPoint);
        body.GroundNormal.Should().Be(reboundNormal);
    }
}
