using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Diagnostics;
using Gravitas.Support;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class SolidBody2DGroundingTests
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
        body.GroundPoint.Should().Be(Vector2d.Zero);
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
        body.GroundNormal.Should().Be(Up);

        body.ClearManualGrounding();
        Step(context);

        body.GroundingMode.Should().Be(GroundingMode.Manual);
        body.IsGrounded.Should().BeFalse();
        body.WasGrounded.Should().BeTrue();
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

        body.IsKinematic = true;
        Step(context);

        body.IsGrounded.Should().BeFalse();
        body.WasGrounded.Should().BeTrue();
    }

    [Fact]
    public void ContactSupport_ShouldGroundAgainstStaticFloorButRejectWallsAndCeilings()
    {
        using GravitasWorldContext floorContext = CreateContext();
        SolidBody2D floorBody = CreateBox(
            floorContext,
            new Vector2d(Fixed64.Zero, -Fixed64.Half),
            new Vector2d((Fixed64)4, Fixed64.One),
            positionFrozen: true);
        SolidBody2D floorTouching = CreateCircle(floorContext, Vector2d.Zero);
        DisableProbeFallback(floorTouching);

        Step(floorContext);

        floorTouching.IsGrounded.Should().BeTrue();
        floorTouching.GroundNormal.Y.Should().BeGreaterThan(Fixed64.Zero);
        floorTouching.GroundPoint.Y.Should().BeGreaterThanOrEqualTo(floorBody.Position.Y);
        floorBody.IsGrounded.Should().BeFalse();

        using GravitasWorldContext wallContext = CreateContext();
        SolidBody2D wallBody = CreateBox(
            wallContext,
            new Vector2d(Fixed64.Half, Fixed64.Zero),
            new Vector2d(Fixed64.One, (Fixed64)4),
            positionFrozen: true);
        SolidBody2D wallTouching = CreateCircle(wallContext, Vector2d.Zero);
        DisableProbeFallback(wallTouching);

        Step(wallContext);

        wallTouching.IsGrounded.Should().BeFalse();
        wallBody.IsGrounded.Should().BeFalse();

        using GravitasWorldContext ceilingContext = CreateContext();
        SolidBody2D ceilingBody = CreateBox(
            ceilingContext,
            new Vector2d(Fixed64.Zero, Fixed64.Half),
            new Vector2d((Fixed64)4, Fixed64.One),
            positionFrozen: true);
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
        Vector2d position,
        bool positionFrozen = false)
    {
        var agent = new TestMatterAgent(context, new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One));
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One,
            FreezeAxes = positionFrozen ? BodyFreezeAxes2D.Position : BodyFreezeAxes2D.None
        };
        body.Initialize(position);
        return body;
    }

    private static SolidBody2D CreateBox(
        GravitasWorldContext context,
        Vector2d position,
        Vector2d size,
        bool positionFrozen)
    {
        var agent = new TestMatterAgent(context, new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One));
        var body = new SolidBody2D(agent, new LSAABBoxCollider2D(size))
        {
            Mass = Fixed64.One,
            FreezeAxes = positionFrozen ? BodyFreezeAxes2D.Position : BodyFreezeAxes2D.None
        };
        body.Initialize(position);
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
}
