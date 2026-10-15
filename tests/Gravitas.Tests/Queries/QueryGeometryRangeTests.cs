using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using System;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class QueryGeometryRangeTests
{
    private static readonly Fixed64 Limit = (Fixed64)4096;

    [Fact]
    public void MixedTargetStateFailure_ShouldRetainBatchPrefixesAndSkipUnusedSlabs()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        var circle = new LSCircleCollider2D(Fixed64.One) { MixedHalfThicknessOverride = Limit };
        circle.InitializeWithNoBody(new TestMatterAgent(context));
        var requests = new[] { new PhysicsSweepSphereAgainst2DRequest(Vector3d.Zero, Vector3d.Zero, Fixed64.One),
            new PhysicsSweepSphereAgainst2DRequest(-Vector3d.Right, Vector3d.Right, Fixed64.One) };
        var ranges = new[] { new PhysicsQueryHitRange(7, 9), new PhysicsQueryHitRange(8, 10) };
        var hits = new SwiftList<PhysicsMixedHit>();
        hits.Add(default);
        Action query = () => context.QueryMixed.SweepSphereAgainst2DAllBatch(requests, hits, ranges);

        query.Should().Throw<ArgumentException>();
        ranges[0].Should().Be(new PhysicsQueryHitRange(0, 0));
        ranges[1].Should().Be(new PhysicsQueryHitRange(8, 10));
        hits.Count.Should().Be(0);
        context.QueryMixed.LastBatchRequestCount.Should().Be(2);
        requests[1] = requests[0];
        context.QueryMixed.SweepSphereAgainst2DAllBatch(requests, hits, ranges).Should().Be(0);
    }

    [Fact]
    public void QueryEnvelopes_ShouldRespectPreparedHalfExtentsAndOutwardOddConeHeight()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 raw = Fixed64.MinIncrement;
        context.Query2D.OverlapAabb(Vector2d.Zero, new Vector2d((Fixed64)8192 + raw, raw), out _)
            .Should().BeFalse();
        Action box = () => context.Query2D.OverlapAabb(Vector2d.Zero,
            new Vector2d((Fixed64)8192 + raw + raw, raw), out _);
        box.Should().Throw<ArgumentException>();
        context.Query3D.OverlapCone(Vector3d.Zero, Vector3d.Right, (Fixed64)8192 - raw - raw,
            raw, out _, PhysicsLayerMask.All).Should().BeFalse();
        Action cone = () => context.Query3D.OverlapCone(Vector3d.Zero, Vector3d.Right,
            (Fixed64)8192 - raw, raw, out _, PhysicsLayerMask.All);
        cone.Should().Throw<ArgumentException>();
        context.QueryMixed.SweepCircleAgainst3D(Vector2d.Zero, Vector2d.Right,
            Limit - raw, Fixed64.Zero, raw, PhysicsLayerMask.All, out _).Should().BeFalse();
        context.Query2D.OverlapCircle(Vector2d.Zero, Limit, out _).Should().BeFalse();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void InvalidConeSign_ShouldRetainExistingPerRequestBatchFailure(bool invalidRadius)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var requests = new[] { new PhysicsOverlapCone3DRequest(Vector3d.Zero, Vector3d.Right,
            Fixed64.One, Fixed64.One, PhysicsLayerMask.All),
            new PhysicsOverlapCone3DRequest(Vector3d.Zero, Vector3d.Right,
                invalidRadius ? Fixed64.One : Fixed64.Zero,
                invalidRadius ? Fixed64.Zero : Fixed64.One, PhysicsLayerMask.All) };
        var ranges = new[] { new PhysicsQueryHitRange(7, 9), new PhysicsQueryHitRange(8, 10) };
        Action query = () => context.Query3D.OverlapConeAllBatch(requests, new SwiftList<Physics3DHit>(), ranges);
        query.Should().Throw<ArgumentException>();
        ranges[0].Should().Be(new PhysicsQueryHitRange(0, 0));
        ranges[1].Should().Be(new PhysicsQueryHitRange(8, 10));
    }

    [Fact]
    public void ExplicitMixedQuery_ShouldIgnoreInactiveTargetSlabsAndPreserveDegenerateMisses()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        var circle = new LSCircleCollider2D(Fixed64.One) { MixedHalfThicknessOverride = Limit };
        circle.InitializeWithNoBody(new TestMatterAgent(context));
        var hits = new SwiftList<PhysicsMixedHit>();
        hits.Add(default);
        context.QueryMixed.SweepSphereAgainst2DAll(Vector3d.Zero, Vector3d.Zero, Fixed64.One,
            PhysicsLayerMask.All, hits).Should().Be(0);
        hits.Count.Should().Be(0);
        circle.IsActive = false;
        context.QueryMixed.SweepSphereAgainst2D(-Vector3d.Right, Vector3d.Right, Fixed64.One,
            PhysicsLayerMask.All, out _).Should().BeFalse();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConeBatch_ShouldPreflightEnvelopesBeforeWritingAnyOutput(bool all)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var requests = new[] { new PhysicsOverlapCone3DRequest(Vector3d.Zero, Vector3d.Right,
            Fixed64.One, Fixed64.One, PhysicsLayerMask.All),
            new PhysicsOverlapCone3DRequest(Vector3d.Zero, Vector3d.Right,
                (Fixed64)8192, Fixed64.One, PhysicsLayerMask.All) };
        var hits = new SwiftList<Physics3DHit>();
        hits.Add(default);
        var ranges = new[] { new PhysicsQueryHitRange(7, 9), new PhysicsQueryHitRange(8, 10) };
        var closest = new Physics3DHit[2];
        Action query = all
            ? () => context.Query3D.OverlapConeAllBatch(requests, hits, ranges)
            : () => context.Query3D.OverlapConeBatch(requests, closest);

        query.Should().Throw<ArgumentException>();
        hits.Count.Should().Be(1);
        ranges[0].Start.Should().Be(7);
        context.Query3D.LastBatchRequestCount.Should().Be(0);
    }

    [Theory]
    [InlineData(PhysicsRuntimeMode.TwoD, false)]
    [InlineData(PhysicsRuntimeMode.TwoD, true)]
    [InlineData(PhysicsRuntimeMode.ThreeD, false)]
    [InlineData(PhysicsRuntimeMode.ThreeD, true)]
    [InlineData(PhysicsRuntimeMode.Both, false)]
    [InlineData(PhysicsRuntimeMode.Both, true)]
    public void ExplicitMixedQuery_ShouldValidateTargetSlabsOutsideMixedMode(PhysicsRuntimeMode mode, bool all)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = mode;
        var circle = new LSCircleCollider2D(Fixed64.One) { MixedHalfThicknessOverride = Limit };
        circle.InitializeWithNoBody(new TestMatterAgent(context));
        var results = new SwiftList<PhysicsMixedHit>();
        results.Add(default);
        Action query = all
            ? () => context.QueryMixed.SweepSphereAgainst2DAll(-Vector3d.Right, Vector3d.Right,
                Fixed64.Half, PhysicsLayerMask.All, results)
            : () => context.QueryMixed.SweepSphereAgainst2D(-Vector3d.Right, Vector3d.Right,
                Fixed64.Half, PhysicsLayerMask.All, out _);

        query.Should().Throw<ArgumentException>().WithMessage("*4,096*");
        results.Count.Should().Be(1);
        context.QueryMixed.LastQueryCandidateCount.Should().Be(0);
        circle.IsMixedPartitioned.Should().BeFalse();
    }

    [Fact]
    public void SupportProbe_ShouldRejectOversizedSourceRadiusAtConstruction()
    {
        Action create = () => _ = new Physics2DSupportQuery(new LSCircleCollider2D(Fixed64.One),
            Vector2d.Zero, Limit + Fixed64.MinIncrement, Vector2d.Right, (Fixed64)10000,
            Fixed64.Half, PhysicsLayerMask.All);
        create.Should().Throw<ArgumentException>().WithMessage("*4,096*");
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(0)]
    [InlineData(1)]
    public void AdmittedSources_ShouldRetainLongTravelAndWorldLimitPositioning(int location)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 x = location switch
        {
            -1 => Fixed64.MinValue + Fixed64.MinIncrement,
            1 => Fixed64.MaxValue - (Fixed64)8192,
            _ => Fixed64.Zero
        };
        Vector2d start = new(x, Fixed64.Zero), end = new(x + (Fixed64)8192, Fixed64.Zero);
        Vector3d start3D = new(x, Fixed64.Zero, Fixed64.Zero), end3D = new(end.X, Fixed64.Zero, Fixed64.Zero);

        context.Query2D.Raycast(start, end, out _).Should().BeFalse();
        context.Query2D.SweepCircle(start, end, Fixed64.One, out _).Should().BeFalse();
        context.Query3D.SweepSphere(start3D, Fixed64.One, Vector3d.Right, (Fixed64)8192,
            out _, PhysicsLayerMask.All).Should().BeFalse();
        context.Query3D.OverlapCone(start3D, Vector3d.Right, (Fixed64)8190, Fixed64.One,
            out _, PhysicsLayerMask.All).Should().BeFalse();
        context.QueryMixed.SweepCircleAgainst3D(start, end, Fixed64.One, Fixed64.MaxValue,
            Fixed64.One, PhysicsLayerMask.All, out _).Should().BeFalse();
        context.QueryMixed.SweepSphereAgainst2D(start3D, end3D, Fixed64.One,
            PhysicsLayerMask.All, out _).Should().BeFalse();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void OversizedSpatialSphereBatch_ShouldPreflightBeforeWritingAnyOutput(bool all)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var requests = new[] { new PhysicsSweepSphere3DRequest(Vector3d.Zero, Vector3d.Right, Fixed64.One),
            new PhysicsSweepSphere3DRequest(Vector3d.Zero, Vector3d.Right, Limit + Fixed64.MinIncrement) };
        var hits = new SwiftList<Physics3DHit>();
        var ranges = new[] { new PhysicsQueryHitRange(7, 9), new PhysicsQueryHitRange(8, 10) };
        var closest = new Physics3DHit[2];
        Action query = all
            ? () => context.Query3D.SweepSphereAllBatch(requests, hits, ranges)
            : () => context.Query3D.SweepSphereBatch(requests, closest);

        query.Should().Throw<ArgumentException>();
        context.Query3D.LastBatchRequestCount.Should().Be(0);
        if (all)
        {
            ranges[0].Should().Be(new PhysicsQueryHitRange(7, 9));
            ranges[1].Should().Be(new PhysicsQueryHitRange(8, 10));
        }
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(0)]
    [InlineData(1)]
    public void PolygonAdmission_ShouldUseExactMeanUnderTranslationAndVertexReordering(int translation)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 shift = translation switch
        {
            -1 => Fixed64.MinValue + (Fixed64)8192,
            1 => Fixed64.MaxValue - (Fixed64)8192,
            _ => Fixed64.Zero
        };
        // The exact mean is (0, 1/3 raw): rounding it first hides the excess at the tip.
        Vector2d[] vertices = { new(shift + Limit, shift),
            new(shift - (Fixed64)2048, shift + Fixed64.MinIncrement), new(shift - (Fixed64)2048, shift) };
        for (int order = 0; order < 3; order++)
        {
            Action query = () => context.Query2D.OverlapPolygon(vertices, out _);
            query.Should().Throw<ArgumentException>().WithMessage("*4,096*");
            (vertices[0], vertices[1], vertices[2]) = (vertices[1], vertices[2], vertices[0]);
        }
        Array.Reverse(vertices);
        Action reversed = () => context.Query2D.OverlapPolygon(vertices, out _);
        reversed.Should().Throw<ArgumentException>();
        vertices[2] = new Vector2d(shift + Limit - Fixed64.MinIncrement, shift);
        context.Query2D.OverlapPolygon(vertices, out _).Should().BeFalse();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PlanarBatch_ShouldPreflightEveryRadiusBeforeWritingAnyOutput(bool all)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var requests = new[] { new PhysicsOverlapCircle2DRequest(Vector2d.Zero, Fixed64.One),
            new PhysicsOverlapCircle2DRequest(Vector2d.Zero, Limit + Fixed64.MinIncrement) };
        var hits = new SwiftList<Physics2DHit>();
        hits.Add(default);
        var ranges = new[] { new PhysicsQueryHitRange(7, 9), new PhysicsQueryHitRange(8, 10) };
        var closest = new Physics2DHit[2];
        Action query = all
            ? () => context.Query2D.OverlapCircleAllBatch(requests, hits, ranges)
            : () => context.Query2D.OverlapCircleBatch(requests, closest);

        query.Should().Throw<ArgumentException>();
        hits.Count.Should().Be(1);
        ranges[0].Start.Should().Be(7);
        context.Query2D.LastBatchRequestCount.Should().Be(0);
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(2, false)]
    [InlineData(2, true)]
    [InlineData(3, false)]
    [InlineData(3, true)]
    [InlineData(4, false)]
    [InlineData(4, true)]
    [InlineData(5, false)]
    [InlineData(6, false)]
    [InlineData(6, true)]
    [InlineData(7, false)]
    [InlineData(7, true)]
    [InlineData(8, false)]
    [InlineData(8, true)]
    [InlineData(9, false)]
    [InlineData(9, true)]
    public void OversizedQuery_ShouldRejectBeforeClearingResultsOrChangingQueryState(int family, bool all)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Diagnostics.Enable();
        var planar = new SwiftList<Physics2DHit>();
        var spatial = new SwiftList<Physics3DHit>();
        var mixed = new SwiftList<PhysicsMixedHit>();
        planar.Add(default);
        spatial.Add(default);
        mixed.Add(default);
        context.Query2D.OverlapQueryVersion = 42;
        context.Query3D.CircleVersion = 42;
        Fixed64 oversized = Limit + Fixed64.MinIncrement;
        Vector2d[] vertices = { new(-4096, 0), new(4096, 0), new(4096, 1) };

        Action query = () =>
        {
            switch (family)
            {
                case 0:
                    if (all) context.Query2D.OverlapCircleAll(Vector2d.Zero, oversized, planar);
                    else context.Query2D.OverlapCircle(Vector2d.Zero, oversized, out _);
                    break;
                case 1:
                    if (all) context.Query2D.SweepCircleAll(Vector2d.Zero, Vector2d.Right, oversized, planar);
                    else context.Query2D.SweepCircle(Vector2d.Zero, Vector2d.Right, oversized, out _);
                    break;
                case 2:
                    if (all) context.Query2D.OverlapAabbAll(Vector2d.Zero, new Vector2d(8192, 2), planar);
                    else context.Query2D.OverlapAabb(Vector2d.Zero, new Vector2d(8192, 2), out _);
                    break;
                case 3:
                    if (all) context.Query2D.OverlapPolygonAll(vertices, planar);
                    else context.Query2D.OverlapPolygon(vertices, out _);
                    break;
                case 4:
                    if (all) context.Query3D.OverlapCircleAll(Vector3d.Zero, oversized, PhysicsLayerMask.All, spatial);
                    else context.Query3D.OverlapCircle(Vector3d.Zero, oversized, out _, PhysicsLayerMask.All);
                    break;
                case 5:
                    context.Query3D.OverlapCircleInDirection(Vector3d.Zero, oversized, Vector3d.Right,
                        out _, Fixed64.One, PhysicsLayerMask.All);
                    break;
                case 6:
                    if (all) context.Query3D.SweepSphereAll(Vector3d.Zero, Vector3d.Right, oversized, PhysicsLayerMask.All, spatial);
                    else context.Query3D.SweepSphere(Vector3d.Zero, oversized, Vector3d.Right, Fixed64.One, out _, PhysicsLayerMask.All);
                    break;
                case 7:
                    if (all) context.Query3D.OverlapConeAll(Vector3d.Zero, Vector3d.Right, (Fixed64)8192,
                        Fixed64.One, PhysicsLayerMask.All, spatial);
                    else context.Query3D.OverlapCone(Vector3d.Zero, Vector3d.Right, (Fixed64)8192,
                        Fixed64.One, out _, PhysicsLayerMask.All);
                    break;
                case 8:
                    if (all) context.QueryMixed.SweepSphereAgainst2DAll(Vector3d.Zero, Vector3d.Right,
                        oversized, PhysicsLayerMask.All, mixed);
                    else context.QueryMixed.SweepSphereAgainst2D(Vector3d.Zero, Vector3d.Right,
                        oversized, PhysicsLayerMask.All, out _);
                    break;
                default:
                    if (all) context.QueryMixed.SweepCircleAgainst3DAll(Vector2d.Zero, Vector2d.Right,
                        Limit, Fixed64.Zero, Fixed64.MinIncrement, PhysicsLayerMask.All, mixed);
                    else context.QueryMixed.SweepCircleAgainst3D(Vector2d.Zero, Vector2d.Right,
                        Limit, Fixed64.Zero, Fixed64.MinIncrement, PhysicsLayerMask.All, out _);
                    break;
            }
        };

        query.Should().Throw<ArgumentException>().WithMessage("*4,096*");
        planar.Count.Should().Be(1);
        spatial.Count.Should().Be(1);
        mixed.Count.Should().Be(1);
        context.Query2D.OverlapQueryVersion.Should().Be(42);
        context.Query3D.CircleVersion.Should().Be(42);
        context.Diagnostics.EventCount.Should().Be(0);
    }
}
