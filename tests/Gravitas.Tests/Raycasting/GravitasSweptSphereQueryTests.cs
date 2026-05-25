using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Raycasting;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Raycasting;

public sealed class GravitasSweptSphereQueryTests
{
    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);

    [Fact]
    public void SweepSphere_ShouldReportTimeOfImpactAndTargetSurfacePoint()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider target = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);

        bool hit = context.Raycasts.SweepSphere(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            Vector3d.Right,
            (Fixed64)4,
            out LSRaycastHit sweepHit,
            IncludeLayerZero);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target);
        sweepHit.Distance.Should().Be(Fixed64.One);
        sweepHit.Point.Should().Be(new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        sweepHit.Normal.Should().Be(-Vector3d.Right);
        sweepHit.Direction.Should().Be(Vector3d.Right);
    }

    [Fact]
    public void SweepSphere_ShouldReturnStartingOverlapAtZeroDistance()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider target = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);

        bool hit = context.Raycasts.SweepSphere(
            new Vector3d(Fixed64.Fraction(1, 4), Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            Vector3d.Right,
            (Fixed64)2,
            out LSRaycastHit sweepHit,
            IncludeLayerZero);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target);
        sweepHit.Distance.Should().Be(Fixed64.Zero);
        sweepHit.Point.Should().Be(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        sweepHit.Normal.Should().Be(Vector3d.Right);
    }

    [Fact]
    public void SweepSphereAll_ShouldSuppressDuplicatesAndOrderByImpactDistance()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider near = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);
        LSSphereCollider far = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Right * 2);
        var hits = new SwiftList<LSRaycastHit>();

        int count = context.Raycasts.SweepSphereAll(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            hits);

        count.Should().Be(2);
        hits[0].Collider.Should().BeSameAs(near);
        hits[1].Collider.Should().BeSameAs(far);
        hits[0].Distance.Should().BeLessThan(hits[1].Distance);
    }

    [Fact]
    public void SweepSphere_ShouldBreakClosestHitTiesByColliderId()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider first = CreateDynamicCollider(
            context,
            new LSSphereCollider(),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Fraction(1, 4)));
        CreateDynamicCollider(
            context,
            new LSSphereCollider(),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, -Fixed64.Fraction(1, 4)));

        bool hit = context.Raycasts.SweepSphere(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            Fixed64.Fraction(1, 4),
            Vector3d.Right,
            (Fixed64)4,
            out LSRaycastHit sweepHit,
            IncludeLayerZero);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(first);
    }

    [Fact]
    public void SweepSphere_ShouldHonorLayerMaskAndExcludedCollider()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider self = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);
        LSSphereCollider ignoredByMask = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Right);
        ignoredByMask.Layer = new PhysicsLayer(1);
        LSSphereCollider target = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Right * 2);

        bool hit = context.Raycasts.SweepSphere(
            Vector3d.Zero,
            Fixed64.Half,
            Vector3d.Right,
            (Fixed64)4,
            out LSRaycastHit sweepHit,
            IncludeLayerZero,
            self);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(target);
        sweepHit.Collider.Should().NotBeSameAs(self);
        sweepHit.Collider.Should().NotBeSameAs(ignoredByMask);
    }

    [Fact]
    public void SweepSphere_ShouldSupportCapsuleCuboidCylinderAndRotatedTargets()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCapsuleCollider capsule = CreateDynamicCollider(context, new LSCapsuleCollider(), Vector3d.Zero);
        LSCuboidCollider cuboid = CreateDynamicCollider(context, new LSCuboidCollider(), new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));
        LSCylinderCollider cylinder = CreateDynamicCollider(context, new LSCylinderCollider(), new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        LSCuboidCollider rotatedCuboid = CreateDynamicCollider(
            context,
            new LSCuboidCollider(),
            new Vector3d((Fixed64)12, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)45));

        AssertSweepHits(context, new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero), capsule);
        AssertSweepHits(context, new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero), cuboid);
        AssertSweepHits(context, new Vector3d((Fixed64)6, Fixed64.Zero, Fixed64.Zero), cylinder);
        AssertSweepHits(context, new Vector3d((Fixed64)10, Fixed64.Zero, Fixed64.Zero), rotatedCuboid);
    }

    [Fact]
    public void SweptSphereWorker_ShouldDetectCylinderSideImpact()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCylinderCollider cylinder = CreateDynamicCollider(context, new LSCylinderCollider(), new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero));
        var worker = new SweptSphereQueryWorker();
        Vector3d origin = new((Fixed64)6, Fixed64.Zero, Fixed64.Zero);
        worker.Prepare(origin, origin + Vector3d.Right * (Fixed64)4, Fixed64.Fraction(1, 4));

        bool hit = worker.TrySweep(cylinder, out Vector3d centerAtImpact, out Fixed64 distance);

        hit.Should().BeTrue();
        distance.Should().Be(Fixed64.Fraction(5, 4));
        centerAtImpact.Should().Be(new Vector3d((Fixed64)7 + Fixed64.Fraction(1, 4), Fixed64.Zero, Fixed64.Zero));
    }

    [Fact]
    public void SweepSphere_ShouldSupportVerticalAndDiagonalSweeps()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider vertical = CreateDynamicCollider(context, new LSSphereCollider(), Vector3d.Zero);
        LSSphereCollider diagonal = CreateDynamicCollider(context, new LSSphereCollider(), new Vector3d((Fixed64)4, Fixed64.Zero, (Fixed64)4));

        bool verticalHit = context.Raycasts.SweepSphere(
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero),
            Fixed64.Half,
            Vector3d.Down,
            (Fixed64)4,
            out LSRaycastHit verticalHitInfo,
            IncludeLayerZero);
        bool diagonalHit = context.Raycasts.SweepSphere(
            new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)2),
            Fixed64.Half,
            new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.One),
            (Fixed64)4,
            out LSRaycastHit diagonalHitInfo,
            IncludeLayerZero);

        verticalHit.Should().BeTrue();
        verticalHitInfo.Collider.Should().BeSameAs(vertical);
        diagonalHit.Should().BeTrue();
        diagonalHitInfo.Collider.Should().BeSameAs(diagonal);
    }

    private static void AssertSweepHits(GravitasWorldContext context, Vector3d origin, LSCollider expected)
    {
        bool hit = context.Raycasts.SweepSphere(
            origin,
            Fixed64.Fraction(1, 4),
            Vector3d.Right,
            (Fixed64)4,
            out LSRaycastHit sweepHit,
            IncludeLayerZero);

        hit.Should().BeTrue();
        sweepHit.Collider.Should().BeSameAs(expected);
    }

    private static TCollider CreateDynamicCollider<TCollider>(
        GravitasWorldContext context,
        TCollider collider,
        Vector3d position,
        FixedQuaternion? rotation = null)
        where TCollider : LSCollider
    {
        EnsureGrid(context);
        var agent = new TestMatterAgent(context);
        var body = new StiffBody(agent, collider)
        {
            Mass = Fixed64.One
        };

        body.Initialize(position, rotation ?? FixedQuaternion.Identity);
        return collider;
    }

    private static void EnsureGrid(GravitasWorldContext context)
    {
        if (context.World.ActiveGrids.Count > 0)
            return;

        GridConfiguration configuration = new(
            new Vector3d((Fixed64)(-8), (Fixed64)(-8), (Fixed64)(-8)),
            new Vector3d((Fixed64)16, (Fixed64)8, (Fixed64)8));

        context.World.TryAddGrid(configuration, out _).Should().BeTrue();
    }
}
