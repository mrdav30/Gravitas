using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedQueryCcdTests
{
    [Theory]
    [InlineData(3, 4, 5, false)]
    [InlineData(3, 4, 5, true)]
    [InlineData(5, 12, 13, false)]
    [InlineData(5, 12, 13, true)]
    [InlineData(7, 24, 25, false)]
    [InlineData(7, 24, 25, true)]
    public void PlanarBoxCircleSweep_ExactCornerTangency_ShouldRetainAuthoredLine(
        int a, int b, int radius, bool reverse)
    {
        using GravitasWorldContext context = CreateMixedContext();
        CreateBodylessBox2D(context, new Vector2d(50, -50), new Vector2d(100, 100));
        Vector2d tangent = new(-b, a), travel = new(a, b);
        Vector2d start = tangent - travel, end = tangent + travel;
        if (reverse) (start, end) = (end, start);

        context.Query2D.SweepCircle(start, end, (Fixed64)radius, out Physics2DHit hit)
            .Should().BeTrue();
        hit.Distance.Should().Be((Fixed64)radius);
        hit.Point.Should().Be(Vector2d.Zero);
    }

    [Theory]
    [InlineData(3, 4, 5, false, false)]
    [InlineData(3, 4, 5, false, true)]
    [InlineData(3, 4, 5, true, false)]
    [InlineData(3, 4, 5, true, true)]
    [InlineData(5, 12, 13, false, false)]
    [InlineData(5, 12, 13, false, true)]
    [InlineData(5, 12, 13, true, false)]
    [InlineData(5, 12, 13, true, true)]
    [InlineData(7, 24, 25, false, false)]
    [InlineData(7, 24, 25, false, true)]
    [InlineData(7, 24, 25, true, false)]
    [InlineData(7, 24, 25, true, true)]
    public void PlanarCornerGrazing_FullHostLoop_ShouldPreserveCircleCapsuleParity(
        int a, int b, int radius, bool capsule, bool reverse)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = context.Environment.AirDensity = context.Environment.DampingFactor = Fixed64.Zero;
        CreateBodylessBox2D(context, new Vector2d(50, -50), new Vector2d(100, 100));
        Vector2d tangent = new(-b, a), travel = new(a, b);
        Vector2d start = tangent - travel, end = tangent + travel;
        if (reverse) (start, end) = (end, start);
        LSCollider2D collider = capsule
            ? new LSCapsuleCollider2D((Fixed64)radius, (Fixed64)(radius * 2))
            : new LSCircleCollider2D((Fixed64)radius);
        collider.Material = PhysicsMaterial.Frictionless;
        var body = new SolidBody2D(new TestMatterAgent(context, new FixedTransform(
            new Vector3d(start.X, Fixed64.Zero, start.Y), FixedQuaternion.Identity, Vector3d.One)), collider)
        {
            Mass = Fixed64.One,
            FreezeAxes = BodyFreezeAxes2D.Rotation,
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
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(2, false)]
    [InlineData(2, true)]
    public void NonzeroCapsuleMover_ExactCapTangency_ShouldRetainChordForEveryTargetFamily(int targetShape, bool reverse)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = targetShape switch
        {
            0 => new LSCircleCollider2D(Fixed64.Quarter),
            1 => new LSCapsuleCollider2D(Fixed64.Quarter, Fixed64.Half),
            _ => new LSAABBoxCollider2D(new Vector2d(100, 100))
        };
        Vector2d targetPosition = targetShape == 2 ? new Vector2d(50, -50) : Vector2d.Zero;
        target.InitializeWithNoBody(new TestMatterAgent(context, new FixedTransform(
            new Vector3d(targetPosition.X, Fixed64.Zero, targetPosition.Y), FixedQuaternion.Identity, Vector3d.One)));
        Fixed64 radius = targetShape == 2 ? (Fixed64)5 : Fixed64.FromFraction(19, 4);
        var capsule = new LSCapsuleCollider2D(radius, radius * Fixed64.Two + Fixed64.Two);
        Vector2d start = reverse ? new Vector2d(-1, 8) : new Vector2d(-7, 0);
        Vector2d displacement = reverse ? new Vector2d(-6, -8) : new Vector2d(6, 8);
        capsule.InitializeWithNoBody(new TestMatterAgent(context, new FixedTransform(
            new Vector3d(start.X, Fixed64.Zero, start.Y), FixedQuaternion.Identity, Vector3d.One)));

        QueryDetection2D.TrySweepMoverShape(capsule, displacement, target, out Physics2DHit hit)
            .Should().BeTrue();

        hit.Distance.Should().Be((Fixed64)5);
        hit.Collider.Should().BeSameAs(target);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConvexMover_ExactRoundedTargetTangency_ShouldRetainWholeShapeChord(bool capsuleTarget)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = capsuleTarget
            ? new LSCapsuleCollider2D((Fixed64)5, (Fixed64)10)
            : new LSCircleCollider2D((Fixed64)5);
        target.InitializeWithNoBody(new TestMatterAgent(context));
        LSCollider2D mover = CreateBodylessBox2D(context, new Vector2d(-8, 0), new Vector2d(2, 2));

        QueryDetection2D.TrySweepMoverShape(mover, new Vector2d(6, 8), target, out Physics2DHit hit)
            .Should().BeTrue();

        hit.Distance.Should().Be((Fixed64)5);
        hit.Collider.Should().BeSameAs(target);
    }

    [Fact]
    public void ConvexMover_ObliqueFaceEntry_ShouldRetainPhysicalDistanceAndSurfaceWitness()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessBox2D(context, new Vector2d(50, 0), new Vector2d(100, 100));
        LSCollider2D mover = CreateBodylessBox2D(context, new Vector2d(-2, 0), new Vector2d(Fixed64.Half, Fixed64.Half));

        QueryDetection2D.TrySweepMoverShape(mover, new Vector2d(5, 12), target, out Physics2DHit hit)
            .Should().BeTrue();

        hit.Distance.Should().Be(Fixed64.FromFraction(91, 20));
        hit.Normal.Should().Be(Vector2d.Left);
        hit.Point.X.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [InlineData(3, 4, 5, false)]
    [InlineData(3, 4, 5, true)]
    [InlineData(5, 12, 13, false)]
    [InlineData(5, 12, 13, true)]
    [InlineData(7, 24, 25, false)]
    [InlineData(7, 24, 25, true)]
    public void MixedSphereConvexSlabSideSweep_ShouldUseAuthoredChordForDistance(
        int a, int b, int length, bool reverse)
    {
        using GravitasWorldContext context = CreateMixedContext();
        int sign = reverse ? -1 : 1;
        CreateBodylessBox2D(context, new Vector2d(sign * 50, 0), new Vector2d(100, 100));
        Vector3d start = new(-sign, 0, 0), end = start + new Vector3d(sign * a, 0, sign * b);

        context.QueryMixed.SweepSphereAgainst2D(start, end, Fixed64.Quarter,
            IncludeLayerZero, out PhysicsMixedHit hit).Should().BeTrue();

        hit.Distance.Should().Be(Fixed64.FromFraction(length * 3, a * 4));
        hit.Point2D.X.Should().Be(Fixed64.Zero);
        hit.NormalFor3DSource.Should().Be(sign > 0 ? Vector3d.Left : Vector3d.Right);
    }

    [Fact]
    public void SpatialSphereMeshFaceSweep_ShouldUseAuthoredChordForDistance()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var mesh = new LSMeshCollider(new[]
        {
            new Vector3d(0, -100, -100), new Vector3d(0, 100, -100),
            new Vector3d(0, 0, 100)
        }, new[] { 0, 1, 2 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        CreateBodyless3D(context, mesh, Vector3d.Zero);
        var worker = new SweptSphereQueryWorker();
        worker.Prepare(new Vector3d(-1, 0, 0), new Vector3d(2, 4, 0), Fixed64.Quarter);

        worker.TrySweep(mesh, out Vector3d impact, out Fixed64 distance).Should().BeTrue();

        distance.Should().Be(Fixed64.FromFraction(5, 4));
        impact.Should().Be(new Vector3d(-Fixed64.Quarter, Fixed64.One, Fixed64.Zero));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SphereFaceSweep_BelowUnitDirectionClosingTolerance_ShouldRemainAGenuineContact(bool mixed)
    {
        using GravitasWorldContext context = CreateMixedContext();
        Fixed64 gap = Fixed64.FromRaw(1);
        Fixed64 planarTravel = Fixed64.One;
        Vector3d start = new(-Fixed64.One - gap, Fixed64.Zero, Fixed64.Zero);
        Vector3d end = new(-Fixed64.One + gap, Fixed64.Zero, planarTravel);
        if (mixed)
        {
            CreateBodylessBox2D(context, new Vector2d(50, 0), new Vector2d(100, 100));
            context.QueryMixed.SweepSphereAgainst2D(start, end, Fixed64.One,
                IncludeLayerZero, out PhysicsMixedHit hit).Should().BeTrue();
            hit.Distance.Should().Be(Fixed64.Half);
            hit.NormalFor3DSource.Should().Be(Vector3d.Left);
        }
        else
        {
            var mesh = new LSMeshCollider(new[]
            {
                new Vector3d(0, -100, -100), new Vector3d(0, 100, -100),
                new Vector3d(0, 0, 100)
            }, new[] { 0, 1, 2 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
            CreateBodyless3D(context, mesh, Vector3d.Zero);
            var worker = new SweptSphereQueryWorker();
            worker.Prepare(start, end, Fixed64.One);
            worker.TrySweep(mesh, out Vector3d impact, out Fixed64 distance).Should().BeTrue();
            distance.Should().Be(Fixed64.Half);
            impact.Should().Be(new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Half));
        }
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(0)]
    [InlineData(1)]
    public void MixedSphereSlabInitialAdmission_ShouldClassifyTheExactOneRawBoundary(int separationRaw)
    {
        using GravitasWorldContext context = CreateMixedContext();
        CreateBodylessBox2D(context, new Vector2d(50, 0), new Vector2d(100, 100));
        Vector3d start = new(-Fixed64.One - Fixed64.FromRaw(separationRaw), Fixed64.Zero, Fixed64.Zero);
        Vector3d end = start + new Vector3d(-Fixed64.FromRaw(2), Fixed64.Zero, Fixed64.One);

        bool found = context.QueryMixed.SweepSphereAgainst2D(start, end, Fixed64.One,
            IncludeLayerZero, out PhysicsMixedHit hit);

        found.Should().Be(separationRaw <= 0);
        if (found) hit.Distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void MixedSphereSlabNearlyParallelLongChord_ShouldRejectUnrepresentableFaceEntry()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessBox2D(context, new Vector2d(50, 0), new Vector2d(100, 100));
        Vector3d start = new(-2, 0, 0);
        Vector3d end = new(-Fixed64.Two + Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.MaxValue - Fixed64.One);
        Fixed64 length = end.Z;

        GravitasQueryMixedService.TrySweepSphereAgainstConvexSlab(start, end,
            (end - start).Normalized, length, Fixed64.One, target, out _).Should().BeFalse();
    }
}
