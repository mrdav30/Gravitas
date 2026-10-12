using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

/// <content>Checks mixed CCD admission when an independent closing row is not the deepest row.</content>
public sealed partial class MixedQueryCcdTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void MixedCompoundCcd_ShouldAdmitEveryClosingWallAlongsideDeeperTangentFloor(bool source3D, bool twoWalls)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        LSCompoundCollider compound = CreateMixedCcdFloorAndWalls(twoWalls);
        ScenarioBody<LSCompoundCollider> body3D = CreateBody3D(context, compound, Vector3d.Zero,
            immovable: !source3D);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero, immovable: source3D);
        body3D.Body.UseManualGrounding();
        body2D.UseManualGrounding();
        body2D.Collider.Material = PhysicsMaterial.Frictionless;
        var manifold = new ContactManifold();
        CollisionDetectionMixed.TryBuildManifold(compound, body2D.Collider, manifold).Should().BeTrue();
        manifold.Count.Should().Be(twoWalls ? 3 : 2);
        manifold.PrimaryContact.Normal.Should().Be(Vector3d.Up);
        manifold[1].Normal.Should().Be(Vector3d.Left);
        // The floor is deeper but tangent to X motion. The wall is independently closing.
        if (source3D)
        {
            body3D.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            body3D.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Left - (twoWalls ? Vector3d.Forward : Vector3d.Zero));
            body3D.Body.ApplyCollisionAngularVelocityDelta(Vector3d.Up * Fixed64.FromFraction(1, 100));
        }
        else
        {
            body2D.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            body2D.ApplyCollisionLinearVelocityDelta(Vector2d.Right + (twoWalls ? Vector2d.Forward : Vector2d.Zero));
            body2D.ApplyCollisionAngularVelocityDelta(Fixed64.FromFraction(1, 100));
        }
        PrepareMixedContinuousCollisionFrame(context);

        // Observe CCD before a later discrete solve could hide incorrect admission.
        if (source3D)
            body3D.Body.LateSimulate(updateSleepState: false, updateColliderState: false);
        else
            body2D.LateSimulate(updateSleepState: false, updateColliderState: false);

        if (source3D)
        {
            body3D.Body.LinearVelocity.X.Abs().Should().BeLessThan(Fixed64.Half);
            if (twoWalls) body3D.Body.LinearVelocity.Z.Abs().Should().BeLessThan(Fixed64.Half);
            body3D.Body.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
            body3D.Body.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
        }
        else
        {
            body2D.LinearVelocity.X.Abs().Should().BeLessThan(Fixed64.Half);
            if (twoWalls) body2D.LinearVelocity.Y.Abs().Should().BeLessThan(Fixed64.Half);
            body2D.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
            body2D.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
            body2D.AngularSpeed.Should().Be(body2D.AngularVelocity.Abs());
        }
        body2D.Agent.Transform.LocalPosition.Y.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MixedTranslationalCompoundResponse_ShouldSolveBothAdmittedDirections(bool source3D)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        ScenarioBody<LSCompoundCollider> body3D = CreateBody3D(context,
            CreateMixedCcdFloorAndWalls(twoWalls: true), Vector3d.Zero, immovable: !source3D);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero, immovable: source3D);
        body3D.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        body2D.FreezeAxes = BodyFreezeAxes2D.Rotation;
        body2D.Collider.Material = PhysicsMaterial.Frictionless;
        if (source3D) body3D.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Left - Vector3d.Forward);
        else body2D.ApplyCollisionLinearVelocityDelta(Vector2d.Right + Vector2d.Forward);
        PrepareMixedContinuousCollisionFrame(context);

        bool applied = source3D
            ? body3D.Body.TryApplyContinuousCollisionMixed2DResponse(body2D, Vector3d.Right,
                Vector3d.Zero, Vector2d.Zero, Fixed64.Zero, context.DeltaTime)
            : body2D.TryApplyContinuousCollisionMixed3DResponse(body3D.Body, Vector2d.Left,
                Vector2d.Zero, Vector3d.Zero, Fixed64.Zero, context.DeltaTime);

        applied.Should().BeTrue();
        // With rotation locked and a stationary target, each cardinal wall removes its own component.
        body3D.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        body2D.LinearVelocity.Should().Be(Vector2d.Zero);
        body2D.Agent.Transform.LocalPosition.Y.Should().Be(Fixed64.Zero);
    }

    private static LSCompoundCollider CreateMixedCcdFloorAndWalls(bool twoWalls)
    {
        CompoundColliderPart floor = CompoundColliderPart.Cuboid(
            new Vector3d((Fixed64)4, Fixed64.FromFraction(1, 5), (Fixed64)4),
            Vector3d.Down * Fixed64.FromFraction(1, 10), PhysicsMaterial.Frictionless);
        CompoundColliderPart wallX = CompoundColliderPart.Cuboid(
            new Vector3d(Fixed64.FromFraction(1, 5), (Fixed64)2, (Fixed64)4),
            Vector3d.Right * Fixed64.FromFraction(2, 5), PhysicsMaterial.Frictionless);
        CompoundColliderPart wallZ = CompoundColliderPart.Cuboid(
            new Vector3d((Fixed64)4, (Fixed64)2, Fixed64.FromFraction(1, 5)),
            Vector3d.Forward * Fixed64.FromFraction(2, 5), PhysicsMaterial.Frictionless);
        return twoWalls ? new LSCompoundCollider(floor, wallX, wallZ) : new LSCompoundCollider(floor, wallX);
    }

    [Fact]
    public void MixedCcdClosingSelection_ShouldKeepTheGeometricWitnessAndUseExactSpeedAdmission()
    {
        var manifold = new ContactManifold();
        ContactAnchor center = ContactAnchor.FromWorldPoint(Vector3d.Zero);
        bool emptyCloses = MixedContact.TrySelectClosingContact(manifold, center, Vector3d.Right, Vector3d.Zero,
            center, Vector2d.Zero, Fixed64.Zero, out MixedContact missing);
        Assert.False(emptyCloses);
        missing.HasContact.Should().BeFalse();
        // Identical center anchors belong to distinct surfaces. Publishing both
        // into one group would deduplicate their identity and erase the wall row.
        manifold.AddContact(new ContactGroupKey(0, 0, 1),
            new MixedContact(center, center, Vector3d.Up, Fixed64.Half).ToManifoldContact());
        manifold.AddContact(new ContactGroupKey(0, 0, 2),
            new MixedContact(center, center, Vector3d.Right, Fixed64.Quarter).ToManifoldContact());

        bool belowCloses = MixedContact.TrySelectClosingContact(manifold, center, Vector3d.Right * (Fixed64.Epsilon / 2),
            Vector3d.Zero, center, Vector2d.Zero, Fixed64.Zero, out MixedContact belowThreshold);
        Assert.False(belowCloses);
        belowThreshold.Normal3DTo2D.Should().Be(Vector3d.Up);
        bool aboveCloses = MixedContact.TrySelectClosingContact(manifold, center, Vector3d.Right * (Fixed64.Epsilon * 2),
            Vector3d.Zero, center, Vector2d.Zero, Fixed64.Zero, out MixedContact closing);
        Assert.True(aboveCloses);
        closing.Normal3DTo2D.Should().Be(Vector3d.Right);
        bool separatingCloses = MixedContact.TrySelectClosingContact(manifold, center, Vector3d.Left, Vector3d.Zero,
            center, Vector2d.Zero, Fixed64.Zero, out MixedContact separating);
        Assert.False(separatingCloses);
        separating.Normal3DTo2D.Should().Be(Vector3d.Up);

        // Opposed representable velocities close at a speed outside Fixed64; exact sign still admits it.
        bool fullRangeCloses = MixedContact.TrySelectClosingContact(manifold, center, Vector3d.Right * Fixed64.MaxValue,
            Vector3d.Zero, center, Vector2d.Left * Fixed64.MaxValue, Fixed64.Zero,
            out MixedContact fullRange);
        Assert.True(fullRangeCloses);
        fullRange.Normal3DTo2D.Should().Be(Vector3d.Right);
    }

    [Fact]
    public void MixedCcdClosingSelection_ShouldPreserveMinimumYawBeforeSpatialProjection()
    {
        var manifold = new ContactManifold();
        ContactAnchor center = ContactAnchor.FromWorldPoint(Vector3d.Zero);
        ContactAnchor point = ContactAnchor.FromWorldPoint(Vector3d.Right * (Fixed64)1024);
        manifold.AddContact(default, new MixedContact(point, point, Vector3d.Forward, Fixed64.Zero).ToManifoldContact());

        bool closes = MixedContact.TrySelectClosingContact(manifold, center, Vector3d.Zero,
            new Vector3d(Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero), center,
            Vector2d.Zero, Fixed64.MinValue, out MixedContact contact);

        // The angular speeds differ by one raw unit. The 1024-unit arm puts
        // its closing projection beyond the 256-raw-unit speed deadband.
        closes.Should().BeTrue();
        contact.Normal3DTo2D.Should().Be(Vector3d.Forward);
    }

    [Fact]
    public void MixedCcdPreview_WithCenterAnchor_ShouldPreserveMinimumPlanarYawWhenTorqueIsZero()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        SolidBody2D source = CreateCircle2D(context, Vector2d.Zero);
        LSSphereCollider target = CreateBodyless3D(context, new LSSphereCollider(), Vector3d.Left);
        source.ApplyCollisionLinearVelocityDelta(Vector2d.Left);
        source.ApplyCollisionAngularVelocityDelta(Fixed64.MinValue);
        PrepareMixedContinuousCollisionFrame(context);
        var contact = new MixedContact(Vector3d.Zero, Vector3d.Zero, Vector3d.Right, Fixed64.Zero);

        bool applied = source.TryApplyMixedRotationalContinuousCollisionResponse(target, contact,
            Fixed64.Zero, source.Position, Vector2d.Zero, source.Rotation, Fixed64.Zero,
            Fixed64.Zero, context.DeltaTime);

        applied.Should().BeTrue();
        source.LinearVelocity.Should().Be(Vector2d.Zero);
        // The source contact is at its center: no angular impulse exists, even at the signed-domain edge.
        source.AngularVelocity.Should().Be(Fixed64.MinValue);
        source.AngularSpeed.Should().Be(Fixed64.MinValue.Abs());
    }
}
