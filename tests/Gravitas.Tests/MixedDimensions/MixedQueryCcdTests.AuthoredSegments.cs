using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

/// <content>Checks that sweep detection retains the host's authored segment rather than a rounded unit ray.</content>
public sealed partial class MixedQueryCcdTests
{
    [Theory]
    [InlineData(3, 4, 5, false)]
    [InlineData(3, 4, 5, true)]
    [InlineData(5, 12, 13, false)]
    [InlineData(5, 12, 13, true)]
    [InlineData(7, 24, 25, false)]
    [InlineData(7, 24, 25, true)]
    public void StaticMeshCircleSweep_ExactCornerTangency_ShouldNotBlockAuthoredTravel(
        int a, int b, int radius, bool reverse)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        var mesh = new LSMeshCollider(new[]
        {
            new Vector3d(0, -2, 0), new Vector3d(0, 2, 0),
            new Vector3d(0, -2, -100), new Vector3d(0, 2, -100)
        }, new[] { 0, 1, 2, 1, 3, 2 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        CreateBodyless3D(context, mesh, Vector3d.Zero);
        Vector2d tangent = new(-b, a), travel = new(a, b);
        Vector2d start = tangent - travel, end = tangent + travel;
        if (reverse) (start, end) = (end, start);
        var hits = new SwiftList<PhysicsMixedHit>();

        context.QueryMixed.SweepCircleAgainst3D(start, end, (Fixed64)radius,
            Fixed64.Zero, Fixed64.Half, IncludeLayerZero, out PhysicsMixedHit hit).Should().BeTrue();
        hit.Distance.Should().Be((Fixed64)radius);
        hit.Point3D.ToVector2d().Should().Be(Vector2d.Zero);
        hit.Point3D.Y.Should().BeInRange(-Fixed64.Half, Fixed64.Half);
        context.QueryMixed.SweepCircleAgainstStatic3DAll(start, end, (Fixed64)radius,
            Fixed64.Zero, Fixed64.Half, IncludeLayerZero, hits, requireClosingContact: true)
            .Should().Be(0);

        context.Environment.Gravity = context.Environment.AirDensity = context.Environment.DampingFactor = Fixed64.Zero;
        var circle = new LSCircleCollider2D((Fixed64)radius)
        {
            MixedHalfThicknessOverride = Fixed64.Half, Material = PhysicsMaterial.Frictionless
        };
        var body = new SolidBody2D(new TestMatterAgent(context, new FixedTransform(
            new Vector3d(start.X, Fixed64.Zero, start.Y), FixedQuaternion.Identity, Vector3d.One)), circle)
        {
            Mass = Fixed64.One, FreezeAxes = BodyFreezeAxes2D.Rotation,
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous
        };
        body.Initialize(start);
        body.UseManualGrounding();
        body.ApplyCollisionLinearVelocityDelta(end - start);
        context.Simulate();
        context.LateSimulate();
        body.Position.Should().Be(end);
        body.LinearVelocity.Should().Be(end - start);
        body.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Theory]
    [InlineData(false, false, -1)]
    [InlineData(false, false, 0)]
    [InlineData(false, false, 1)]
    [InlineData(false, true, -1)]
    [InlineData(false, true, 0)]
    [InlineData(false, true, 1)]
    [InlineData(true, false, -1)]
    [InlineData(true, false, 0)]
    [InlineData(true, false, 1)]
    [InlineData(true, true, -1)]
    [InlineData(true, true, 0)]
    [InlineData(true, true, 1)]
    public void MixedFiniteCorner_OneRawRadiusAroundTangency_ShouldDistinguishMissGrazeAndEntry(
        bool meshTarget, bool reverse, int radiusShift)
    {
        using GravitasWorldContext context = CreateMixedContext();
        if (meshTarget)
            CreateBodyless3D(context, new LSMeshCollider(new[]
            {
                new Vector3d(0, -2, 0), new Vector3d(0, 2, 0),
                new Vector3d(0, -2, -100), new Vector3d(0, 2, -100)
            }, new[] { 0, 1, 2, 1, 3, 2 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation), Vector3d.Zero);
        else
            CreateBodyless3D(context, new LSCuboidCollider { Size = new Vector3d(100, 4, 100) },
                new Vector3d(50, 0, -50));
        Vector2d tangent = new(-4, 3), travel = new(3, 4);
        Vector2d start = tangent - travel, end = tangent + travel;
        if (reverse) (start, end) = (end, start);
        Fixed64 radius = (Fixed64)5 + Fixed64.FromRaw(radiusShift);
        var hits = new SwiftList<PhysicsMixedHit>(1);

        bool found = context.QueryMixed.SweepCircleAgainst3D(start, end, radius,
            Fixed64.Zero, Fixed64.Half, IncludeLayerZero, out PhysicsMixedHit hit);
        found.Should().Be(radiusShift >= 0);
        if (radiusShift == 0) hit.Distance.Should().Be((Fixed64)5);
        if (radiusShift > 0) hit.Distance.Should().BeLessThan((Fixed64)5);
        context.QueryMixed.SweepCircleAgainstStatic3DAll(start, end, radius,
            Fixed64.Zero, Fixed64.Half, IncludeLayerZero, hits, requireClosingContact: true)
            .Should().Be(radiusShift > 0 ? 1 : 0);

        for (int i = 0; i < 4; i++)
            context.QueryMixed.SweepCircleAgainst3DAll(start, end, radius,
                Fixed64.Zero, Fixed64.Half, IncludeLayerZero, hits);
        long before = System.GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 8; i++)
            context.QueryMixed.SweepCircleAgainst3DAll(start, end, radius,
                Fixed64.Zero, Fixed64.Half, IncludeLayerZero, hits);
        long allocated = System.GC.GetAllocatedBytesForCurrentThread() - before;
        allocated.Should().Be(0);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ContinuousBody_ObliqueImpact_ShouldCommitAuthoredSegmentPoint(bool spatial, bool kinematic)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = context.Environment.AirDensity = context.Environment.DampingFactor = Fixed64.Zero;
        context.Settings.ContinuousCollisionMaxToiIterations = 1;
        Vector3d start = new(-1, 0, 0), end = new(2, 0, 4);
        Vector3d expected = new(-Fixed64.Quarter, Fixed64.Zero, Fixed64.One);
        BodyMotionType motion = kinematic ? BodyMotionType.Kinematic : BodyMotionType.Dynamic;
        if (spatial)
        {
            CreateBodyless3D(context, new LSCuboidCollider
            {
                Size = new Vector3d(100, 100, 100), Material = PhysicsMaterial.Frictionless
            }, new Vector3d(50, 0, 0));
            var collider = new LSSphereCollider { Radius = Fixed64.Quarter, Material = PhysicsMaterial.Frictionless };
            var body = new SolidBody(new TestMatterAgent(context,
                new FixedTransform(start, FixedQuaternion.Identity, Vector3d.One)), collider)
            {
                Mass = Fixed64.One, FreezeAxes = BodyFreezeAxes3D.Rotation,
                ContinuousCollisionMode = ContinuousCollisionMode.Continuous
            };
            body.Initialize(start, FixedQuaternion.Identity, motion);
            body.UseManualGrounding();
            if (kinematic) body.Agent.Transform.LocalPosition = end;
            else body.ApplyCollisionLinearVelocityDelta(end - start);
            context.Simulate();
            context.LateSimulate();
            body.Position3d.Should().Be(expected);
            body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        }
        else
        {
            CreateBodylessBox2D(context, new Vector2d(50, 0), new Vector2d(100, 100));
            var collider = new LSCircleCollider2D(Fixed64.Quarter) { Material = PhysicsMaterial.Frictionless };
            var body = new SolidBody2D(new TestMatterAgent(context,
                new FixedTransform(start, FixedQuaternion.Identity, Vector3d.One)), collider)
            {
                Mass = Fixed64.One, FreezeAxes = BodyFreezeAxes2D.Rotation,
                ContinuousCollisionMode = ContinuousCollisionMode.Continuous
            };
            body.Initialize(start.ToVector2d(), motionType: motion);
            body.UseManualGrounding();
            if (kinematic) body.Agent.Transform.LocalPosition = end;
            else body.ApplyCollisionLinearVelocityDelta((end - start).ToVector2d());
            context.Simulate();
            context.LateSimulate();
            body.Position.Should().Be(expected.ToVector2d());
            body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        }
    }
}
