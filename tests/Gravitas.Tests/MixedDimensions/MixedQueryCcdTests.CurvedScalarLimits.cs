using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using GridForge.Grids.Topology;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedQueryCcdTests
{
    [Theory]
    [InlineData(0, false, false)]
    [InlineData(0, false, true)]
    [InlineData(0, true, false)]
    [InlineData(0, true, true)]
    [InlineData(1, false, false)]
    [InlineData(1, false, true)]
    [InlineData(1, true, false)]
    [InlineData(1, true, true)]
    [InlineData(2, false, false)]
    [InlineData(2, false, true)]
    [InlineData(2, true, false)]
    [InlineData(2, true, true)]
    public void CurvedTargetAtScalarLimit_ContainedReducer_ShouldMatchTranslatedOrigin(
        int shape, bool zAxis, bool upperLimit)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Vector2d axis = zAxis ? Vector2d.Forward : Vector2d.Right;
        Fixed64 sign = upperLimit ? Fixed64.One : -Fixed64.One;
        Fixed64 edge = upperLimit ? Fixed64.MaxValue : Fixed64.MinValue;
        Vector2d center = axis * (edge - sign * Fixed64.FromFraction(1, 100));
        Vector2d start = axis * (edge - sign * Fixed64.FromFraction(1, 20));
        Vector2d localStart = start - center;
        LSCollider target = CreateScalarLimitCurvedTarget(context, shape, zAxis, sign, center);
        LSCollider origin = CreateScalarLimitCurvedTarget(context, shape, zAxis, sign, Vector2d.Zero);
        Vector3d sourceCenter = new(start.X, Fixed64.Zero, start.Y);
        target.GetClosestSurfaceAnchor(sourceCenter, out _, out bool contained);
        contained.Should().BeTrue();
        Fixed64 travel = Fixed64.FromFraction(1, 1000);
        Vector2d displacement = -axis * sign * travel;

        TryScalarLimitSweep(context, origin, localStart, localStart + displacement, out PhysicsMixedHit expected)
            .Should().BeTrue();
        TryScalarLimitSweep(context, target, start, start + displacement, out PhysicsMixedHit hit)
            .Should().BeTrue();

        hit.Distance.Should().Be(Fixed64.Zero);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
        hit.Normal3DTo2D.Should().Be(expected.Normal3DTo2D);
        hit.Anchor3D.TryGetOffsetFrom(new Vector3d(center.X, Fixed64.Zero, center.Y), out Vector3d offset)
            .Should().BeTrue();
        offset.Should().Be(expected.Point3D);
        hit.Anchor2D.TryGetOffsetFrom(sourceCenter, out Vector3d sourceOffset).Should().BeTrue();
        (expected.Point2D - new Vector3d(localStart.X, Fixed64.Zero, localStart.Y))
            .Should().Be(sourceOffset);

        // Strict CCD must use the actual outward normal, not the direction of travel.
        Vector2d outward = expected.NormalFor2DSource;
        foreach (int mode in new[] { -1, 0, 1 })
        {
            Vector2d end = start + outward * travel * (Fixed64)mode;
            TryScalarLimitSweep(context, target, start, end, out PhysicsMixedHit permissive).Should().BeTrue();
            permissive.Normal3DTo2D.Should().Be(expected.Normal3DTo2D);
            TryScalarLimitSweep(context, target, start, end, out _, requireClosing: true)
                .Should().Be(mode < 0);
        }
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(2, false)]
    [InlineData(2, true)]
    public void CurvedTargetAtScalarLimit_EnteringAndLeaving_ShouldMatchTranslatedOrigin(int shape, bool upperLimit)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 sign = upperLimit ? Fixed64.One : -Fixed64.One;
        Fixed64 edge = upperLimit ? Fixed64.MaxValue : Fixed64.MinValue;
        Vector2d center = Vector2d.Right * (edge - sign * Fixed64.FromFraction(1, 100));
        Vector2d contained = Vector2d.Right * (edge - sign * Fixed64.FromFraction(1, 20));
        Vector2d exterior = Vector2d.Right * (edge - sign * Fixed64.FromFraction(3, 5));
        LSCollider target = CreateScalarLimitCurvedTarget(context, shape, false, sign, center);
        LSCollider origin = CreateScalarLimitCurvedTarget(context, shape, false, sign, Vector2d.Zero);

        TryScalarLimitSweep(context, origin, exterior - center, contained - center, out PhysicsMixedHit expected)
            .Should().BeTrue();
        TryScalarLimitSweep(context, target, exterior, contained, out PhysicsMixedHit entry, requireClosing: true)
            .Should().BeTrue();

        entry.Distance.Should().Be(expected.Distance);
        entry.Distance.Should().BeGreaterThan(Fixed64.Zero);
        entry.Normal3DTo2D.Should().Be(expected.Normal3DTo2D);
        TryScalarLimitSweep(context, target, contained, exterior, out PhysicsMixedHit departure).Should().BeTrue();
        departure.Distance.Should().Be(Fixed64.Zero);
        TryScalarLimitSweep(context, target, contained, exterior, out _, requireClosing: true).Should().BeFalse();
        TryScalarLimitSweep(context, target, exterior, exterior - Vector2d.Right * sign, out _).Should().BeFalse();
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(2, false)]
    [InlineData(2, true)]
    public void CurvedTargetAtScalarLimit_PublicQueryAndHostCcd_ShouldAdmitRealCandidates(int shape, bool upperLimit)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.ApplySettings(new PhysicsSettings(1, null));
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.Environment.Gravity = context.Environment.AirDensity = context.Environment.DampingFactor = Fixed64.Zero;
        Fixed64 sign = upperLimit ? Fixed64.One : -Fixed64.One;
        Fixed64 edge = upperLimit ? Fixed64.MaxValue : Fixed64.MinValue;
        Fixed64 minX = upperLimit ? edge - (Fixed64)2 : edge;
        Fixed64 maxX = upperLimit ? edge : edge + (Fixed64)2;
        context.World.TryAddGrid(new GridConfiguration(
            new Vector3d(minX, -Fixed64.One, -Fixed64.One),
            new Vector3d(maxX, Fixed64.One, Fixed64.One),
            topologyMetrics: GridTopologyMetrics.Rectangular((Fixed64)2)), out _).Should().BeTrue();
        Vector2d center = Vector2d.Right * (edge - sign * Fixed64.FromFraction(1, 100));
        Vector2d start = Vector2d.Right * (edge - sign * Fixed64.FromFraction(1, 20));
        LSCollider target = CreateScalarLimitCurvedTarget(context, shape, false, sign, center);
        Fixed64 travel = Fixed64.FromFraction(1, 1000);
        Vector2d end = start + Vector2d.Right * sign * travel;
        var hits = new SwiftList<PhysicsMixedHit>(1);

        context.QueryMixed.SweepCircleAgainst3D(start, end, Fixed64.FromFraction(1, 100),
            Fixed64.Zero, Fixed64.Quarter, IncludeLayerZero, out PhysicsMixedHit hit).Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(target);
        context.QueryMixed.LastQueryCandidateCount.Should().Be(1);
        context.QueryMixed.SweepCircleAgainst3DAll(start, end, Fixed64.FromFraction(1, 100),
            Fixed64.Zero, Fixed64.Quarter, IncludeLayerZero, hits).Should().Be(1);
        hits[0].Anchor3D.Should().Be(hit.Anchor3D);
        context.QueryMixed.SweepCircleAgainst3D(start, end, Fixed64.FromFraction(1, 100),
            Fixed64.Zero, Fixed64.Quarter, PhysicsLayerMask.FromLayer(1), out _).Should().BeFalse();

        var circle = new LSCircleCollider2D(Fixed64.FromFraction(1, 100))
        {
            MixedHalfThicknessOverride = Fixed64.Quarter,
            Material = PhysicsMaterial.Frictionless
        };
        var body = new SolidBody2D(new TestMatterAgent(context, new FixedTransform(
            new Vector3d(start.X, Fixed64.Zero, start.Y), FixedQuaternion.Identity, Vector3d.One)), circle)
        {
            Mass = Fixed64.One,
            FreezeAxes = BodyFreezeAxes2D.Rotation,
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous
        };
        body.Initialize(start);
        body.UseManualGrounding();
        body.ApplyCollisionLinearVelocityDelta(-hit.NormalFor2DSource * Fixed64.FromFraction(1, 100));

        context.Simulate();
        context.LateSimulate();

        body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        Vector2d.Dot(body.LinearVelocity, hit.NormalFor2DSource).Should().BeGreaterThanOrEqualTo(-Fixed64.Epsilon);
        body.Agent.Transform.WorldPosition.Y.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(2, false)]
    [InlineData(2, true)]
    public void CurvedTargetAtScalarLimit_VerticalProjection_ShouldKeepCircleRadiusAndRejectEmptySlab(
        int shape, bool upperLimit)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 sign = upperLimit ? Fixed64.One : -Fixed64.One;
        Fixed64 edge = upperLimit ? Fixed64.MaxValue : Fixed64.MinValue;
        Vector2d center = Vector2d.Right * (edge - sign * Fixed64.FromFraction(1, 100));
        Vector2d start = Vector2d.Right * (edge - sign * Fixed64.FromFraction(1, 20));
        LSCollider target = CreateScalarLimitCurvedTarget(context, shape, false, sign, center);
        LSCollider origin = CreateScalarLimitCurvedTarget(context, shape, false, sign, Vector2d.Zero);
        target.AgentOrNull!.Transform.LocalRotation = FixedQuaternion.Identity;
        origin.AgentOrNull!.Transform.LocalRotation = FixedQuaternion.Identity;
        Fixed64 slabY = (Fixed64)2;
        target.AgentOrNull!.Transform.LocalPosition = new Vector3d(center.X, slabY, center.Y);
        origin.AgentOrNull!.Transform.LocalPosition = Vector3d.Up * slabY;
        target.RebuildRuntimeShapeOnly().Should().BeTrue();
        origin.RebuildRuntimeShapeOnly().Should().BeTrue();
        Vector2d displacement = -Vector2d.Right * sign * Fixed64.FromFraction(1, 1000);

        TryScalarLimitSweep(context, origin, start - center, start - center + displacement,
            out PhysicsMixedHit expected, slabY: slabY).Should().BeTrue();
        TryScalarLimitSweep(context, target, start, start + displacement, out PhysicsMixedHit hit,
            slabY: slabY).Should().BeTrue();

        hit.Distance.Should().Be(Fixed64.Zero);
        hit.Normal3DTo2D.Should().Be(expected.Normal3DTo2D);
        context.QueryMixed.TrySweepCircleAgainst3DCollider(target, start, start + displacement,
            -Vector2d.Right * sign, Fixed64.FromFraction(1, 1000), Fixed64.FromFraction(1, 100),
            (Fixed64)4, Fixed64.Quarter, -Vector3d.Right * sign, null, out _).Should().BeFalse();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapsuleWithRepresentableRigidBounds_RoundedSupportOverflow_ShouldKeepContainedContact(bool upperLimit)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        FixedQuaternion rotation = new FixedQuaternion(
            Fixed64.Zero, Fixed64.Zero, -Fixed64.One, (Fixed64)8).Normalized;
        Fixed64 axisLength = (Fixed64)16;
        Fixed64 radius = Fixed64.Quarter;
        FixedBoundBox localBounds = FixedBoundBox.FromCenteredCapsuleClippedToDomain(
            Vector3d.Zero, rotation, Vector3d.Up, axisLength, radius);
        Fixed64 x = upperLimit
            ? Fixed64.MaxValue - localBounds.Max.X - Fixed64.MinIncrement
            : Fixed64.MinValue - localBounds.Min.X + Fixed64.MinIncrement;
        Vector3d center = new(x, Fixed64.Zero, Fixed64.Zero);
        var capsule = new LSCapsuleCollider
        {
            Radius = radius,
            Size = new Vector3d(Fixed64.One, axisLength + radius * Fixed64.Two, Fixed64.One)
        };
        capsule.InitializeWithNoBody(new TestMatterAgent(context, new FixedTransform(
            center, rotation, Vector3d.One)));
        capsule.BoundsMin.X.Should().BeGreaterThan(Fixed64.MinValue);
        capsule.BoundsMax.X.Should().BeLessThan(Fixed64.MaxValue);
        capsule.GetClosestSurfaceAnchor(center, out Vector3d normal, out bool contained);
        contained.Should().BeTrue();

        // Rounded support axes can cross the domain even when exact rigid bounds fit.
        Vector3d supportAxis = (capsule.Rotation * Vector3d.Up).Normalized;
        FixedSlabProjection.TryGetCapsuleSupport(center, supportAxis, axisLength, radius,
            new FixedRange(-axisLength, axisLength),
            upperLimit ? Vector2d.Right : -Vector2d.Right, out _).Should().BeFalse();
        Vector2d start = new(x, Fixed64.Zero);
        context.QueryMixed.TrySweepCircleAgainst3DCollider(capsule, start, start,
            Vector2d.Zero, Fixed64.Zero, Fixed64.FromFraction(1, 100),
            Fixed64.Zero, axisLength, Vector3d.Zero, null, out PhysicsMixedHit hit).Should().BeTrue();
        hit.Distance.Should().Be(Fixed64.Zero);
        hit.Normal3DTo2D.Should().Be(normal);

        foreach (int mode in new[] { -1, 0, 1 })
        {
            Vector2d movement = hit.NormalFor2DSource * Fixed64.FromFraction(1, 1000) * (Fixed64)mode;
            Fixed64 length = movement.Magnitude;
            Vector2d direction = length > Fixed64.Zero ? movement / length : Vector2d.Zero;
            context.QueryMixed.TrySweepCircleAgainst3DCollider(capsule, start, start + movement,
                direction, length, Fixed64.FromFraction(1, 100), Fixed64.Zero, axisLength,
                new Vector3d(direction.X, Fixed64.Zero, direction.Y), null, out _,
                requireClosingContact: true).Should().Be(mode < 0);
        }
    }

    private static LSCollider CreateScalarLimitCurvedTarget(
        GravitasWorldContext context, int shape, bool zAxis, Fixed64 sign, Vector2d center)
    {
        Fixed64 radius = Fixed64.Quarter;
        Vector3d size = new(Fixed64.One, Fixed64.FromFraction(1, 10), Fixed64.One);
        LSCollider target = shape switch
        {
            0 => new LSCylinderCollider { Radius = radius, Size = size },
            1 => new LSCapsuleCollider
            {
                Radius = radius,
                Size = new Vector3d(Fixed64.One, Fixed64.FromFraction(3, 5), Fixed64.One)
            },
            _ => new LSConeCollider { Radius = radius, Size = size }
        };
        FixedQuaternion rotation = (zAxis
            ? new FixedQuaternion(sign, Fixed64.Zero, Fixed64.Zero, (Fixed64)2)
            : new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, -sign, (Fixed64)2)).Normalized;
        target.Material = PhysicsMaterial.Frictionless;
        target.InitializeWithNoBody(new TestMatterAgent(context, new FixedTransform(
            new Vector3d(center.X, Fixed64.Zero, center.Y), rotation, Vector3d.One)));
        return target;
    }

    private static bool TryScalarLimitSweep(GravitasWorldContext context, LSCollider target,
        Vector2d start, Vector2d end, out PhysicsMixedHit hit, bool requireClosing = false,
        Fixed64 slabY = default)
    {
        Vector2d displacement = end - start;
        Fixed64 length = displacement.Magnitude;
        Vector2d direction = length > Fixed64.Zero ? displacement / length : Vector2d.Zero;
        return context.QueryMixed.TrySweepCircleAgainst3DCollider(target, start, end, direction, length,
            Fixed64.FromFraction(1, 100), slabY, Fixed64.Quarter,
            new Vector3d(direction.X, Fixed64.Zero, direction.Y), null, out hit, requireClosing);
    }
}
