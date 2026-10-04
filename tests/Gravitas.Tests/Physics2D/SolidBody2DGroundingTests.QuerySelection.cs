using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Diagnostics;
using Gravitas.Support;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed partial class SolidBody2DGroundingTests
{
    [Theory]
    [InlineData(GroundProbeMode2D.Ray)]
    [InlineData(GroundProbeMode2D.SweptCircle)]
    public void QueryProbe_WhenEarlierNormalRejectsGround_ShouldUseFartherSupport(GroundProbeMode2D mode)
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateSelectionProbe(context, mode, new Vector2d(Fixed64.Zero, Fixed64.Half));
        CreateStaticFloor(context, new Vector2d(-Fixed64.Half, Fixed64.Zero), new Vector2d(1, 4));
        LSAABBoxCollider2D floor = CreateStaticFloor(context, new Vector2d(Fixed64.Zero, -Fixed64.FromFraction(3, 2)));
        context.Diagnostics.Enable();

        body.CheckGround();

        AssertSelectedProbe(context, body, floor, new Vector2d(Fixed64.Zero, -Fixed64.One),
            mode == GroundProbeMode2D.Ray ? Fixed64.FromFraction(3, 2) : Fixed64.One);
    }

    [Theory]
    [InlineData(GroundProbeMode2D.Ray, false)]
    [InlineData(GroundProbeMode2D.Ray, true)]
    [InlineData(GroundProbeMode2D.SweptCircle, false)]
    [InlineData(GroundProbeMode2D.SweptCircle, true)]
    public void QueryProbe_WhenNearSupportIsPhysicallyIgnored_ShouldUseFartherSupport(
        GroundProbeMode2D mode, bool supportIgnoresSource)
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateSelectionProbe(context, mode, Vector2d.Forward);
        context.Settings.GroundCheckLayerMask = PhysicsLayerMask.All;
        LSAABBoxCollider2D near = CreateStaticFloor(context, layer: new PhysicsLayer(1));
        LSAABBoxCollider2D far = CreateStaticFloor(context, -Vector2d.Forward, layer: new PhysicsLayer(2));
        if (supportIgnoresSource)
            near.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(body.Collider.Layer);
        else
            body.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(near.Layer);
        context.Diagnostics.Enable();

        body.CheckGround();

        AssertSelectedProbe(context, body, far, new Vector2d(Fixed64.Zero, -Fixed64.Half),
            mode == GroundProbeMode2D.Ray ? Fixed64.FromFraction(3, 2) : Fixed64.One);
        // Ray collection keeps the source's dynamic candidate; the static-style
        // sweep excludes it. Physical eligibility must not change these counters.
        context.Query2D.LastQueryCandidateCount.Should().Be(mode == GroundProbeMode2D.Ray ? 3 : 2);
    }

    [Theory]
    [InlineData(GroundProbeMode2D.Ray)]
    [InlineData(GroundProbeMode2D.SweptCircle)]
    public void QueryProbe_WhenCompoundFirstWitnessRejectsGround_ShouldUseSeparateSupport(GroundProbeMode2D mode)
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateSelectionProbe(context, mode, new Vector2d(Fixed64.Zero, Fixed64.Half));
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.AABBox(new Vector2d(1, 4), new Vector2d(-Fixed64.Half, Fixed64.Zero)),
            CompoundColliderPart2D.AABBox(new Vector2d(4, 1), new Vector2d(Fixed64.Zero, -Fixed64.Half)));
        compound.InitializeWithNoBody(new TestMatterAgent(context, new FixedTransform(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        LSAABBoxCollider2D floor = CreateStaticFloor(context, new Vector2d(Fixed64.Zero, -Fixed64.FromFraction(3, 2)));
        context.Diagnostics.Enable();

        body.CheckGround();

        // Automatic grounding reduces the compound first, then tests its normal.
        // Selecting the later floor part would change this existing contract.
        AssertSelectedProbe(context, body, floor, new Vector2d(Fixed64.Zero, -Fixed64.One),
            mode == GroundProbeMode2D.Ray ? Fixed64.FromFraction(3, 2) : Fixed64.One);
    }

    [Theory]
    [InlineData(GroundProbeMode2D.Ray)]
    [InlineData(GroundProbeMode2D.SweptCircle)]
    public void QueryProbe_WithEqualDistanceSupports_ShouldKeepLowerOwnerIdAndDiagnosticWitness(GroundProbeMode2D mode)
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateSelectionProbe(context, mode, Vector2d.Forward);
        LSAABBoxCollider2D first = CreateStaticFloor(context);
        LSAABBoxCollider2D second = CreateStaticFloor(context);
        first.Id.Should().BeLessThan(second.Id);
        context.Diagnostics.Enable();

        body.CheckGround();

        AssertSelectedProbe(context, body, first, new Vector2d(Fixed64.Zero, Fixed64.Half),
            mode == GroundProbeMode2D.Ray ? Fixed64.Half : Fixed64.Zero);
    }

    [Theory]
    [InlineData(GroundProbeMode2D.Ray)]
    [InlineData(GroundProbeMode2D.SweptCircle)]
    public void QueryProbe_WhenNearerSupportHasLaterOwnerId_ShouldReplaceFartherAcceptedHit(GroundProbeMode2D mode)
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateSelectionProbe(context, mode, Vector2d.Forward);
        LSAABBoxCollider2D far = CreateStaticFloor(context, -Vector2d.Forward);
        LSAABBoxCollider2D near = CreateStaticFloor(context);
        far.Id.Should().BeLessThan(near.Id);
        context.Diagnostics.Enable();

        body.CheckGround();

        AssertSelectedProbe(context, body, near, new Vector2d(Fixed64.Zero, Fixed64.Half),
            mode == GroundProbeMode2D.Ray ? Fixed64.Half : Fixed64.Zero);
    }

    [Theory]
    [InlineData(GroundProbeMode2D.Ray, 0)]
    [InlineData(GroundProbeMode2D.Ray, 1)]
    [InlineData(GroundProbeMode2D.Ray, 2)]
    [InlineData(GroundProbeMode2D.Ray, 3)]
    [InlineData(GroundProbeMode2D.SweptCircle, 0)]
    [InlineData(GroundProbeMode2D.SweptCircle, 1)]
    [InlineData(GroundProbeMode2D.SweptCircle, 2)]
    [InlineData(GroundProbeMode2D.SweptCircle, 3)]
    public void QueryProbe_WithTinyTravel_ShouldPreserveRayAndSweepAdmission(GroundProbeMode2D mode, int distanceCase)
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateSelectionProbe(context, mode, new Vector2d(Fixed64.Zero, Fixed64.Half));
        CreateStaticFloor(context);
        Fixed64 distance = distanceCase switch
        {
            0 => Fixed64.Zero,
            1 => Fixed64.FromRaw(1),
            2 => Fixed64.Epsilon,
            _ => Fixed64.Epsilon + Fixed64.FromRaw(1)
        };
        body.GroundedDistanceRay = distance;
        body.GroundDownDistanceOnAir = distance;

        body.CheckGround();

        bool expectedGround = mode == GroundProbeMode2D.Ray ? distanceCase != 0 : distanceCase == 3;
        body.IsGrounded.Should().Be(expectedGround);
        body.HasGroundPoint.Should().Be(expectedGround);
        body.GroundNormal.Should().Be(expectedGround ? Up : Vector2d.Zero);
    }

    private static SolidBody2D CreateSelectionProbe(GravitasWorldContext context, GroundProbeMode2D mode, Vector2d position)
    {
        SolidBody2D body = CreateCircle(context, position);
        body.GroundProbeMode = mode;
        body.GroundProbeRadius = Fixed64.Half;
        body.GroundDownDistanceOnAir = (Fixed64)3;
        body.GroundedDistanceRay = (Fixed64)3;
        body.GroundMinNormalDot = Fixed64.One;
        return body;
    }

    private static void AssertSelectedProbe(GravitasWorldContext context, SolidBody2D body,
        LSCollider2D support, Vector2d point, Fixed64 distance)
    {
        body.IsGrounded.Should().BeTrue();
        body.HasGroundPoint.Should().BeTrue();
        body.GroundPoint.Should().Be(point);
        body.GroundNormal.Should().Be(Up);
        context.Diagnostics.Events.Length.Should().Be(1);
        GravitasDiagnosticEvent probe = context.Diagnostics.Events[0];
        probe.Kind.Should().Be(GravitasDiagnosticEventKind.GroundProbe);
        probe.Hit.Should().BeTrue();
        probe.ColliderBId.Should().Be(support.Id);
        probe.PointA.Should().Be(point.ToVector3d(Fixed64.Zero));
        probe.Vector.Should().Be(Up.ToVector3d(Fixed64.Zero));
        probe.ScalarB.Should().Be(distance);
        probe.DataA.Should().Be((int)body.GroundProbeMode);
        probe.DataB.Should().Be((int)GravitasColliderDimension.TwoD);
    }
}
