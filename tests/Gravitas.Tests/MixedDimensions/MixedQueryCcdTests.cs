using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Diagnostics;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using SwiftCollections;
using System;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed class MixedQueryCcdTests
{
    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);

    [Fact]
    public void SweepSphereAgainst2D_ShouldHitEmbeddedSlabWithoutChangingPure3DQuerySurface()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D platform = CreateBodylessBox2D(context, Vector2d.Zero, new Vector2d((Fixed64)4, (Fixed64)4));
        var pureHits = new SwiftList<Physics3DHit>();

        int pureCount = context.Query3D.SweepSphereAll(
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, (Fixed64)(-3), Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            pureHits);
        bool mixedHit = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, (Fixed64)(-3), Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        pureCount.Should().Be(0);
        mixedHit.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(platform);
        hit.Collider3D.Should().BeNull();
        hit.Distance.Should().Be((Fixed64)2);
        hit.Normal3DTo2D.Should().Be(-Vector3d.Up);
    }

    [Fact]
    public void SweepSphereAgainst2D_ShouldReturnCompound2DOwnerThroughPartGeometry()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D compound = CreateBodylessCompound2D(context, Vector2d.Zero);

        bool mixedHit = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(compound);
        hit.Collider3D.Should().BeNull();
        hit.Distance.Should().Be((Fixed64)2);
        hit.Normal3DTo2D.Should().Be(Vector3d.Right);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepSphereAgainst2D_WithUnsupported2DTarget_ShouldReportConservativeFallback()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessUnsupported2D(context, Vector2d.Zero);

        bool mixedHit = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(target);
        hit.Distance.Should().Be(Fixed64.FromFraction(3, 2));
        hit.Point2D.Should().Be(new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero));
        hit.Point3D.Should().Be(new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero));
        hit.Normal3DTo2D.Should().Be(Vector3d.Right);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.ConservativeFallback);
    }

    [Fact]
    public void SweepCircleAgainst3D_ShouldHit3DPrimitiveWithoutChangingPure2DQuerySurface()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(context, Vector3d.Zero, immovable: true);
        var pureHits = new SwiftList<Physics2DHit>();

        int pureCount = context.Query2D.SweepCircleAll(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            pureHits);
        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        pureCount.Should().Be(0);
        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(target.Collider);
        hit.Collider2D.Should().BeNull();
        hit.Distance.Should().Be((Fixed64)2);
        hit.Normal3DTo2D.Should().Be(-Vector3d.Right);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithSphereCenterOverlap_ShouldUseOppositeSweepSurfacePoint()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(context, Vector3d.Zero, immovable: true);

        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            Vector2d.Zero,
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(target.Collider);
        hit.Collider2D.Should().BeNull();
        hit.Distance.Should().Be(Fixed64.Zero);
        hit.Normal3DTo2D.Should().Be(Vector3d.Right);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithVerticalSphereOverlap_ShouldKeepPlanarPointAtCircleCenter()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(context, Vector3d.Up, immovable: true);

        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            Vector2d.Zero,
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(target.Collider);
        hit.Collider2D.Should().BeNull();
        hit.Distance.Should().Be(Fixed64.Zero);
        hit.Point2D.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Zero));
        hit.Normal3DTo2D.Should().Be(-Vector3d.Up);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithCuboidTarget_ShouldUseExactFiniteSlabReducer()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCuboidCollider> target = CreateBody3D(context, new LSCuboidCollider(), Vector3d.Zero, immovable: true);

        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(target.Collider);
        hit.Distance.Should().Be((Fixed64)2);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithTallSlabCuboidNearMiss_ShouldRejectProxyOnlyHit()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBody3D(context, new LSCuboidCollider(), Vector3d.Zero, immovable: true);
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-3), (Fixed64)2),
            new Vector2d((Fixed64)3, (Fixed64)2),
            Fixed64.Half,
            Fixed64.Zero,
            (Fixed64)2,
            IncludeLayerZero,
            hits);

        count.Should().Be(0);
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithTallSlabCuboidHit_ShouldReportFiniteSlabDistance()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCuboidCollider> target = CreateBody3D(context, new LSCuboidCollider(), Vector3d.Zero, immovable: true);

        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            (Fixed64)2,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(target.Collider);
        hit.Distance.Should().Be((Fixed64)2);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithCapsuleTarget_ShouldUseExactFiniteSlabReducer()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCapsuleCollider> target = CreateBody3D(
            context,
            new LSCapsuleCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true);

        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(target.Collider);
        hit.Distance.Should().Be((Fixed64)2);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithTallSlabCapsuleNearMiss_ShouldRejectProxyOnlyHit()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBody3D(
            context,
            new LSCapsuleCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true);
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-3), (Fixed64)2),
            new Vector2d((Fixed64)3, (Fixed64)2),
            Fixed64.Half,
            Fixed64.Zero,
            (Fixed64)2,
            IncludeLayerZero,
            hits);

        count.Should().Be(0);
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithTallSlabCapsuleHit_ShouldReportFiniteSlabDistance()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCapsuleCollider> target = CreateBody3D(
            context,
            new LSCapsuleCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true);

        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            (Fixed64)2,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(target.Collider);
        hit.Distance.Should().Be((Fixed64)2);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithRotatedCapsuleTarget_ShouldUseExactFiniteSlabReducer()
    {
        using GravitasWorldContext context = CreateMixedContext();
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90);
        ScenarioBody<LSCapsuleCollider> target = CreateBody3D(
            context,
            new LSCapsuleCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true,
            rotation: rotation);

        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            new Vector2d((Fixed64)4, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(target.Collider);
        hit.Distance.Should().Be((Fixed64)2);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithRotatedCapsuleTarget_ShouldRejectProxyOnlyFiniteSlabMiss()
    {
        using GravitasWorldContext context = CreateMixedContext();
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90);
        _ = CreateBody3D(
            context,
            new LSCapsuleCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            new Vector3d(Fixed64.Zero, Fixed64.FromFraction(31, 25), Fixed64.FromFraction(13, 20)),
            immovable: true,
            rotation: rotation);
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            new Vector2d((Fixed64)4, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.FromFraction(3, 4),
            IncludeLayerZero,
            hits);

        count.Should().Be(0);
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithCylinderTarget_ShouldUseExactFiniteSlabReducer()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCylinderCollider> target = CreateBody3D(
            context,
            new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true);

        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(target.Collider);
        hit.Distance.Should().Be((Fixed64)2);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithConeTarget_ShouldUseExactFiniteSlabReducer()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSConeCollider> target = CreateBody3D(
            context,
            new LSConeCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true,
            rotation: FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)90, Fixed64.Zero));

        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.FromFraction(1, 100),
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(target.Collider);
        AssertNear(hit.Distance, Fixed64.FromFraction(1349, 600));
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithConeApexSliceNearMiss_ShouldRejectProxyOnlyHit()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBody3D(
            context,
            new LSConeCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true);
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(3, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(3, 10)),
            Fixed64.FromFraction(1, 10),
            Fixed64.FromFraction(7, 5),
            Fixed64.FromFraction(1, 20),
            IncludeLayerZero,
            hits);

        count.Should().Be(0);
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithRotatedConeTarget_ShouldUseExactCircleSlabSweep()
    {
        using GravitasWorldContext context = CreateMixedContext();
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90);
        ScenarioBody<LSConeCollider> target = CreateBody3D(
            context,
            new LSConeCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true,
            rotation: rotation);

        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            new Vector2d((Fixed64)4, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(target.Collider);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithRotatedConeTarget_ShouldRejectWholeProjectionOnlyHit()
    {
        using GravitasWorldContext context = CreateMixedContext();
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90);
        _ = CreateBody3D(
            context,
            new LSConeCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true,
            rotation: rotation);
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-4), Fixed64.FromFraction(2, 5)),
            new Vector2d((Fixed64)4, Fixed64.FromFraction(2, 5)),
            Fixed64.FromFraction(1, 10),
            Fixed64.FromFraction(9, 20),
            Fixed64.FromFraction(1, 100),
            IncludeLayerZero,
            hits);

        count.Should().Be(0);
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithRotatedCylinderTarget_ShouldUseExactFiniteSlabReducer()
    {
        using GravitasWorldContext context = CreateMixedContext();
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90);
        ScenarioBody<LSCylinderCollider> target = CreateBody3D(
            context,
            new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true,
            rotation: rotation);

        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            new Vector2d((Fixed64)4, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(target.Collider);
        hit.Distance.Should().Be((Fixed64)2);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithRotatedCylinderTarget_ShouldRejectProxyOnlyFiniteSlabMiss()
    {
        using GravitasWorldContext context = CreateMixedContext();
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90);
        _ = CreateBody3D(
            context,
            new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            new Vector3d(Fixed64.Zero, Fixed64.FromFraction(31, 25), Fixed64.FromFraction(13, 20)),
            immovable: true,
            rotation: rotation);
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            new Vector2d((Fixed64)4, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.FromFraction(3, 4),
            IncludeLayerZero,
            hits);

        count.Should().Be(0);
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void SweepCircleAgainst3DAll_WithArbitrarilyRotatedCurvedTargets_ShouldUseExactFiniteSlabReducers()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBody3D(
            context,
            new LSCapsuleCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            new Vector3d((Fixed64)(-1), Fixed64.Zero, Fixed64.Zero),
            immovable: true,
            rotation: FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)25, (Fixed64)35, (Fixed64)50));
        _ = CreateBody3D(
            context,
            new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            immovable: true,
            rotation: FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)(-20), (Fixed64)40, (Fixed64)65));
        _ = CreateBody3D(
            context,
            new LSConeCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero),
            immovable: true,
            rotation: FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)15, (Fixed64)(-25), (Fixed64)70));
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            new Vector2d((Fixed64)7, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            hits);

        count.Should().Be(3);
        hits.Count.Should().Be(3);
        hits[0].ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
        hits[1].ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
        hits[2].ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithTallSlabCylinderNearMiss_ShouldRejectProxyOnlyHit()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBody3D(
            context,
            new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true);
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-3), (Fixed64)2),
            new Vector2d((Fixed64)3, (Fixed64)2),
            Fixed64.Half,
            Fixed64.Zero,
            (Fixed64)2,
            IncludeLayerZero,
            hits);

        count.Should().Be(0);
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithTallSlabCylinderHit_ShouldReportFiniteSlabDistance()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCylinderCollider> target = CreateBody3D(
            context,
            new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true);

        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            (Fixed64)2,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(target.Collider);
        hit.Distance.Should().Be((Fixed64)2);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepCircleAgainst3DAll_PrimitiveFiniteSlabReducers_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = CreateMixedContext();
        FixedQuaternion rotatedCurvedTarget = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90);
        _ = CreateBody3D(context, new LSCuboidCollider(), new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero), immovable: true);
        _ = CreateBody3D(
            context,
            new LSCapsuleCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true);
        _ = CreateBody3D(
            context,
            new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        _ = CreateBody3D(
            context,
            new LSConeCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero),
            immovable: true);
        _ = CreateBody3D(
            context,
            new LSCapsuleCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero),
            immovable: true,
            rotation: rotatedCurvedTarget);
        _ = CreateBody3D(
            context,
            new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            new Vector3d((Fixed64)6, Fixed64.Zero, Fixed64.Zero),
            immovable: true,
            rotation: rotatedCurvedTarget);
        _ = CreateBody3D(
            context,
            new LSConeCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero),
            immovable: true,
            rotation: rotatedCurvedTarget);
        var hits = new SwiftList<PhysicsMixedHit>(8);

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            () => context.QueryMixed.SweepCircleAgainst3DAll(
                new Vector2d((Fixed64)(-8), Fixed64.Zero),
                new Vector2d((Fixed64)10, Fixed64.Zero),
                Fixed64.Half,
                Fixed64.Zero,
                Fixed64.Half,
                IncludeLayerZero,
                hits),
            warmupIterations: 16,
            stabilizationIterations: 4,
            measurementIterations: 16);

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void SweepSphereAgainst2D_WithAabbSlabTarget_ShouldUseExactReducer()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessBox2D(context, Vector2d.Zero, new Vector2d((Fixed64)4, (Fixed64)4));

        bool mixedHit = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, (Fixed64)(-3), Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(target);
        hit.Distance.Should().Be((Fixed64)2);
        hit.Normal3DTo2D.Should().Be(-Vector3d.Up);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepSphereAgainst2D_WithCircleSlabSide_ShouldReportExactHit()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessCircle2D(context, Vector2d.Zero);

        bool mixedHit = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(target);
        hit.Distance.Should().Be((Fixed64)2);
        hit.Normal3DTo2D.Should().Be(Vector3d.Right);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepSphereAgainst2D_WithCircleSlabTopFace_ShouldReportExactHit()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessCircle2D(context, Vector2d.Zero);

        bool mixedHit = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, (Fixed64)(-3), Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(target);
        hit.Distance.Should().Be((Fixed64)2);
        hit.Normal3DTo2D.Should().Be(-Vector3d.Up);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepSphereAgainst2D_WithCircleSlabBoundaryOverlap_ShouldUseEmbeddedCenterFallbackNormal()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessCircle2D(context, Vector2d.Zero);

        bool mixedHit = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero),
            new Vector3d((Fixed64)3, -Fixed64.Half, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(target);
        hit.Distance.Should().Be(Fixed64.Zero);
        hit.Point2D.Should().Be(new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero));
        hit.Normal3DTo2D.Should().Be(Vector3d.Up);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepSphereAgainst2D_WithUnsupportedCenterOverlap_ShouldUseSweepDirectionFallbackNormal()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessUnsupported2D(context, Vector2d.Zero);

        bool mixedHit = context.QueryMixed.SweepSphereAgainst2D(
            Vector3d.Zero,
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(target);
        hit.Distance.Should().Be(Fixed64.Zero);
        hit.Normal3DTo2D.Should().Be(Vector3d.Right);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.ConservativeFallback);
    }

    [Fact]
    public void SweepSphereAgainst2D_WithCapsuleSlabSide_ShouldReportExactHit()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessCapsule2D(context, Vector2d.Zero);

        bool mixedHit = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(target);
        hit.Distance.Should().Be((Fixed64)2);
        hit.Normal3DTo2D.Should().Be(Vector3d.Right);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepSphereAgainst2D_WithCapsuleSlabTopFace_ShouldReportExactHit()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessCapsule2D(context, Vector2d.Zero);

        bool mixedHit = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, (Fixed64)(-3), Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(target);
        hit.Distance.Should().Be((Fixed64)2);
        hit.Normal3DTo2D.Should().Be(-Vector3d.Up);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepSphereAgainst2D_WithCapsuleSlabStartingOverlap_ShouldReportStableExactHit()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessCapsule2D(context, Vector2d.Zero);

        bool mixedHit = context.QueryMixed.SweepSphereAgainst2D(
            Vector3d.Zero,
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(target);
        hit.Distance.Should().Be(Fixed64.Zero);
        hit.Normal3DTo2D.Should().Be(Vector3d.Down);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepSphereAgainst2D_WithAabbSlabBoundsOnlyCornerMiss_ShouldRejectExactMiss()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBodylessBox2D(context, Vector2d.Zero, new Vector2d((Fixed64)2, (Fixed64)2));
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepSphereAgainst2DAll(
            new Vector3d((Fixed64)(-3), Fixed64.FromFraction(9, 10), Fixed64.FromFraction(7, 5)),
            new Vector3d((Fixed64)3, Fixed64.FromFraction(9, 10), Fixed64.FromFraction(7, 5)),
            Fixed64.Half,
            IncludeLayerZero,
            hits);

        count.Should().Be(0);
        context.QueryMixed.LastQueryCandidateCount.Should().Be(1);
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void SweepSphereAgainst2D_WithAabbSlabSideFace_ShouldReportExactHit()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessBox2D(context, Vector2d.Zero, new Vector2d((Fixed64)2, (Fixed64)2));

        bool mixedHit = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(target);
        hit.Distance.Should().Be(Fixed64.FromFraction(3, 2));
        hit.Normal3DTo2D.Should().Be(Vector3d.Right);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepSphereAgainst2D_WithAabbSlabVerticalGrazingTopFace_ShouldReportExactHit()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessBox2D(context, Vector2d.Zero, new Vector2d((Fixed64)2, (Fixed64)2));

        bool mixedHit = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d((Fixed64)(-3), Fixed64.One, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.One, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(target);
        hit.Distance.Should().Be((Fixed64)2);
        hit.Normal3DTo2D.Should().Be(-Vector3d.Up);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepSphereAgainst2D_WithAabbSlabStartingOverlap_ShouldReportStableExactHit()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessBox2D(context, Vector2d.Zero, new Vector2d((Fixed64)2, (Fixed64)2));

        bool mixedHit = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.One, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(target);
        hit.Distance.Should().Be(Fixed64.Zero);
        hit.Normal3DTo2D.Should().Be(-Vector3d.Up);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepSphereAgainst2D_WithPolygonSlabBoundsOnlyCornerMiss_ShouldRejectExactMiss()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBodylessPolygon2D(context, Vector2d.Zero, CreateDiamondVertices());
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepSphereAgainst2DAll(
            new Vector3d(-Fixed64.FromFraction(1, 4), Fixed64.FromFraction(9, 10), Fixed64.FromFraction(7, 5)),
            new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.FromFraction(9, 10), Fixed64.FromFraction(7, 5)),
            Fixed64.Half,
            IncludeLayerZero,
            hits);

        count.Should().Be(0);
        context.QueryMixed.LastQueryCandidateCount.Should().Be(1);
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void SweepSphereAgainst2D_WithPolygonSlabVertexFeature_ShouldReportExactHit()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessPolygon2D(context, Vector2d.Zero, CreateDiamondVertices());

        bool mixedHit = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(target);
        hit.Distance.Should().Be(Fixed64.FromFraction(3, 2));
        hit.Normal3DTo2D.Should().Be(Vector3d.Right);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepSphereAgainst2D_WithCompoundAabbAndPolygonParts_ShouldUseOwnerAndExactPartGeometry()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessCompound2D(
            context,
            Vector2d.Zero,
            CompoundColliderPart2D.AABBox(Vector2d.One, new Vector2d((Fixed64)2, Fixed64.Zero)),
            CompoundColliderPart2D.ConvexPolygon(CreateDiamondVertices(), Vector2d.Zero));

        bool mixedHit = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(target);
        hit.Distance.Should().Be(Fixed64.FromFraction(3, 2));
        hit.Normal3DTo2D.Should().Be(Vector3d.Right);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepSphereAgainst2D_WithCompoundCapsulePart_ShouldUseOwnerAndExactPartGeometry()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessCompound2D(
            context,
            Vector2d.Zero,
            CompoundColliderPart2D.Capsule(Fixed64.Half, (Fixed64)3, Vector2d.Zero));

        bool mixedHit = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(target);
        hit.Distance.Should().Be((Fixed64)2);
        hit.Normal3DTo2D.Should().Be(Vector3d.Right);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepSphereAgainst2D_WithCompoundEqualDistanceParts_ShouldUseAuthoredPartOrder()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessCompound2D(
            context,
            Vector2d.Zero,
            CompoundColliderPart2D.AABBox(Vector2d.One, new Vector2d(Fixed64.Zero, -Fixed64.One)),
            CompoundColliderPart2D.AABBox(Vector2d.One, new Vector2d(Fixed64.Zero, Fixed64.One)));

        bool mixedHit = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(target);
        hit.Distance.Should().Be(Fixed64.FromFraction(5, 2));
        hit.Point2D.Z.Should().Be(-Fixed64.Half);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepSphereAgainst2D_WithRejectedAabbSlabCandidate_ShouldReportExactDiagnostics()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBodylessBox2D(
            context,
            new Vector2d(Fixed64.Zero, (Fixed64)3),
            Vector2d.One);
        var hits = new SwiftList<PhysicsMixedHit>();
        context.Diagnostics.Enable(eventCapacity: 4, drawCommandCapacity: 0);

        int count = context.QueryMixed.SweepSphereAgainst2DAll(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, (Fixed64)(-3)),
            new Vector3d((Fixed64)3, Fixed64.Zero, (Fixed64)3),
            Fixed64.Half,
            IncludeLayerZero,
            hits);

        count.Should().Be(0);
        context.QueryMixed.LastQueryCandidateCount.Should().Be(1);
        ReadOnlySpan<GravitasDiagnosticEvent> events = context.Diagnostics.Events;
        FindDiagnosticEvent(events, GravitasDiagnosticEventKind.MixedQuery)
            .TryAsMixedQuery(out GravitasMixedQueryDiagnosticView queryView)
            .Should()
            .BeTrue();
        queryView.Hit.Should().BeFalse();
        queryView.HitCount.Should().Be(0);

        GravitasDiagnosticEvent summaryEvent = FindDiagnosticEvent(events, GravitasDiagnosticEventKind.QuerySummary);
        summaryEvent.TryAsQuerySummary(out GravitasQuerySummaryDiagnosticView summary).Should().BeTrue();
        summary.SourceDimension.Should().Be(GravitasColliderDimension.ThreeD);
        summary.TargetDimension.Should().Be(GravitasColliderDimension.TwoD);
        summary.ExactReducerAttempts.Should().Be(1);
        summary.AcceptedHits.Should().Be(0);
        summary.FallbackHits.Should().Be(0);
        summary.RejectedConservativeCandidates.Should().Be(0);
        summary.HasConservativeFallback.Should().BeFalse();
    }

    [Fact]
    public void SweepSphereAgainst2D_WithAcceptedAabbSlabCandidate_ShouldReportExactDiagnostics()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessBox2D(
            context,
            Vector2d.Zero,
            new Vector2d((Fixed64)4, (Fixed64)4));
        var hits = new SwiftList<PhysicsMixedHit>();
        context.Diagnostics.Enable(eventCapacity: 4, drawCommandCapacity: 0);

        int count = context.QueryMixed.SweepSphereAgainst2DAll(
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, (Fixed64)(-3), Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            hits);

        count.Should().Be(1);
        hits.Count.Should().Be(1);
        hits[0].Collider2D.Should().BeSameAs(target);
        hits[0].ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);

        ReadOnlySpan<GravitasDiagnosticEvent> events = context.Diagnostics.Events;
        FindDiagnosticEvent(events, GravitasDiagnosticEventKind.MixedQuery)
            .TryAsMixedQuery(out GravitasMixedQueryDiagnosticView queryView)
            .Should()
            .BeTrue();
        queryView.Hit.Should().BeTrue();
        queryView.HitCount.Should().Be(1);

        GravitasDiagnosticEvent summaryEvent = FindDiagnosticEvent(events, GravitasDiagnosticEventKind.QuerySummary);
        summaryEvent.TryAsQuerySummary(out GravitasQuerySummaryDiagnosticView summary).Should().BeTrue();
        summary.SourceDimension.Should().Be(GravitasColliderDimension.ThreeD);
        summary.TargetDimension.Should().Be(GravitasColliderDimension.TwoD);
        summary.ExactReducerAttempts.Should().Be(1);
        summary.AcceptedHits.Should().Be(1);
        summary.FallbackHits.Should().Be(0);
        summary.RejectedConservativeCandidates.Should().Be(0);
        summary.HasConservativeFallback.Should().BeFalse();
    }

    [Fact]
    public void SweepSphereAgainst2DAll_AabbPolygonAndCompoundSlabs_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBodylessBox2D(context, new Vector2d((Fixed64)(-2), Fixed64.Zero), new Vector2d((Fixed64)2, (Fixed64)2));
        _ = CreateBodylessPolygon2D(context, new Vector2d(Fixed64.Zero, Fixed64.Zero), CreateDiamondVertices());
        _ = CreateBodylessCompound2D(
            context,
            new Vector2d((Fixed64)2, Fixed64.Zero),
            CompoundColliderPart2D.AABBox(Vector2d.One, Vector2d.Zero),
            CompoundColliderPart2D.ConvexPolygon(CreateDiamondVertices(), new Vector2d(Fixed64.One, Fixed64.Zero)));
        var hits = new SwiftList<PhysicsMixedHit>(4);

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            () => context.QueryMixed.SweepSphereAgainst2DAll(
                new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero),
                Fixed64.Half,
                IncludeLayerZero,
                hits),
            warmupIterations: 16,
            stabilizationIterations: 4,
            measurementIterations: 16);

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void PublicQuerySurface_ShouldExposeOnlyExplicitConvexMeshSourceSweeps()
    {
        Type[] services =
        {
            typeof(GravitasQuery2DService),
            typeof(GravitasQuery3DService),
            typeof(GravitasQueryMixedService)
        };
        string[] allowedMeshSourceMethods = { "SweepConvexMesh", "SweepConvexMeshAll" };

        foreach (Type service in services)
        {
            foreach (System.Reflection.MethodInfo method in service.GetMethods(
                System.Reflection.BindingFlags.Instance
                | System.Reflection.BindingFlags.Public
                | System.Reflection.BindingFlags.DeclaredOnly))
            {
                method.Name.Should().NotBe("SweepMesh");
                method.Name.Should().NotBe("SweepMeshAll");
                method.Name.Contains("Concave", StringComparison.Ordinal).Should().BeFalse();

                bool acceptsMeshCollider = false;
                bool acceptsRawMesh = false;
                foreach (System.Reflection.ParameterInfo parameter in method.GetParameters())
                {
                    acceptsMeshCollider |= parameter.ParameterType == typeof(LSMeshCollider);
                    acceptsRawMesh |= parameter.ParameterType == typeof(PhysicsMesh)
                        || parameter.ParameterType == typeof(Vector3d[])
                        || parameter.ParameterType == typeof(int[]);
                }

                acceptsRawMesh.Should().BeFalse();
                if (!acceptsMeshCollider)
                    continue;

                service.Should().Be(typeof(GravitasQuery3DService));
                allowedMeshSourceMethods.Should().Contain(method.Name);
            }
        }
    }

    [Fact]
    public void SweepCircleAgainst3D_WithTallSlabAndPlanarSeparation_ShouldRejectProxyOnlySphereHit()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateSphere3D(context, Vector3d.Zero, immovable: true);
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-3), (Fixed64)2),
            new Vector2d((Fixed64)3, (Fixed64)2),
            Fixed64.Half,
            Fixed64.Zero,
            (Fixed64)2,
            IncludeLayerZero,
            hits);

        count.Should().Be(0);
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void SweepCircleAgainst3D_NearSlabCorner_ShouldUseVerticalOverlapToReducePlanarSphereReach()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.FromFraction(9, 10), Fixed64.Zero),
            immovable: true);
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(9, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(9, 10)),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            hits);

        count.Should().Be(0);
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void SweepCircleAgainst3D_ShouldHitMeshTargetThroughFiniteSlabProjection()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSMeshCollider> mesh = CreateMesh3D(
            context,
            MeshTestFixtures.CreateVerticalQuad(Fixed64.Zero, -Fixed64.One, Fixed64.One),
            Vector3d.Zero,
            immovable: true);

        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.One,
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(mesh.Collider);
        hit.Distance.Should().Be(Fixed64.FromFraction(5, 2));
        hit.Point3D.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));
        hit.Normal3DTo2D.Should().Be(-Vector3d.Right);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithMeshTarget_ShouldReportTriangleCandidateCount()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateMesh3D(
            context,
            MeshTestFixtures.CreateVerticalQuad(Fixed64.Zero, -Fixed64.One, Fixed64.One),
            Vector3d.Zero,
            immovable: true);
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.One,
            Fixed64.Half,
            IncludeLayerZero,
            hits);

        count.Should().Be(1);
        context.QueryMixed.LastQueryCandidateCount.Should().Be(1);
        context.QueryMixed.LastMeshTriangleCandidateCount.Should().Be(2);

        _ = context.QueryMixed.SweepSphereAgainst2DAll(
            new Vector3d((Fixed64)(-3), Fixed64.One, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.One, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            hits);

        context.QueryMixed.LastMeshTriangleCandidateCount.Should().Be(0);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithMeshTriangleProxyOnlyHit_ShouldRejectFiniteSlabMiss()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateMesh3D(
            context,
            CreateSlabClippedProxyOnlyTriangle(),
            Vector3d.Zero,
            immovable: true);
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.One,
            Fixed64.Half,
            IncludeLayerZero,
            hits);

        count.Should().Be(0);
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithMeshTriangleOnSlabBoundary_ShouldUseExactFiniteSlabReducer()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSMeshCollider> mesh = CreateMesh3D(
            context,
            CreateSlabBoundaryTriangle(),
            Vector3d.Zero,
            immovable: true);

        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.One,
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(mesh.Collider);
        hit.Distance.Should().Be(Fixed64.FromFraction(5, 2));
        hit.Point3D.Y.Should().Be(Fixed64.Half);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithMeshTrianglePointProjection_ShouldUseExactFiniteSlabReducer()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSMeshCollider> mesh = CreateMesh3D(
            context,
            CreatePointProjectionTriangle(),
            Vector3d.Zero,
            immovable: true);

        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Half,
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(mesh.Collider);
        hit.Distance.Should().Be(Fixed64.FromFraction(5, 2));
        hit.Point3D.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithMeshTriangleSegmentProjection_ShouldUseExactFiniteSlabReducer()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSMeshCollider> mesh = CreateMesh3D(
            context,
            CreateSegmentProjectionTriangle(),
            Vector3d.Zero,
            immovable: true);

        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Half,
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(mesh.Collider);
        hit.Distance.Should().Be(Fixed64.FromFraction(5, 2));
        hit.Point3D.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithStartingOverlapInsideMeshProjection_ShouldReturnStableExactHit()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSMeshCollider> mesh = CreateMesh3D(
            context,
            CreateSlabBoundaryTriangle(),
            Vector3d.Zero,
            immovable: true);

        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d(Fixed64.Half, Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.One,
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(mesh.Collider);
        hit.Distance.Should().Be(Fixed64.Zero);
        hit.Point3D.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
        hit.Normal3DTo2D.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithEqualDistanceMeshTriangles_ShouldUseAuthoredTriangleOrder()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSMeshCollider> mesh = CreateMesh3D(
            context,
            CreateEqualDistanceTriangleMesh(),
            Vector3d.Zero,
            immovable: true);

        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.One,
            Fixed64.One,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(mesh.Collider);
        hit.Distance.Should().Be(Fixed64.FromFraction(5, 2));
        hit.Point3D.Y.Should().Be(Fixed64.Half);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepCircleAgainst3D_ShouldHitCompoundTargetThroughEarliestPartGeometry()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCompoundCollider> compound = CreateCompound3D(context, Vector3d.Zero, immovable: true);

        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(compound.Collider);
        hit.Distance.Should().Be(Fixed64.One);
        hit.Normal3DTo2D.Should().Be(-Vector3d.Right);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithCompoundMeshParts_ShouldUseAuthoredPartOrderAndOwnerIdentity()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCompoundCollider> compound = CreateBody3D(
            context,
            CreateEqualDistanceMeshPartCompound(),
            Vector3d.Zero,
            immovable: true);

        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.One,
            Fixed64.One,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(compound.Collider);
        hit.Distance.Should().Be(Fixed64.FromFraction(5, 2));
        hit.Point3D.Y.Should().Be(Fixed64.Half);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }


    [Fact]
    public void MixedQueryDiagnostics_ShouldRecordReducerQualityCounters()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateSphere3D(context, Vector3d.Zero, immovable: true);
        _ = CreateMesh3D(
            context,
            MeshTestFixtures.CreateVerticalQuad((Fixed64)2, -Fixed64.One, Fixed64.One),
            Vector3d.Zero,
            immovable: true);
        var hits = new SwiftList<PhysicsMixedHit>();
        context.Diagnostics.Enable(eventCapacity: 4, drawCommandCapacity: 0);

        int count = context.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)4, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.One,
            Fixed64.Half,
            IncludeLayerZero,
            hits);

        count.Should().Be(2);
        ReadOnlySpan<GravitasDiagnosticEvent> events = context.Diagnostics.Events;
        FindDiagnosticEvent(events, GravitasDiagnosticEventKind.MixedQuery)
            .TryAsMixedQuery(out GravitasMixedQueryDiagnosticView queryView)
            .Should()
            .BeTrue();
        queryView.HitCount.Should().Be(2);

        GravitasDiagnosticEvent summaryEvent = FindDiagnosticEvent(events, GravitasDiagnosticEventKind.QuerySummary);
        summaryEvent.TryAsQuerySummary(out GravitasQuerySummaryDiagnosticView summary).Should().BeTrue();
        summary.SourceDimension.Should().Be(GravitasColliderDimension.TwoD);
        summary.TargetDimension.Should().Be(GravitasColliderDimension.ThreeD);
        summary.ExactReducerAttempts.Should().Be(2);
        summary.AcceptedHits.Should().Be(2);
        summary.FallbackHits.Should().Be(0);
        summary.RejectedConservativeCandidates.Should().Be(0);
        summary.HasConservativeFallback.Should().BeFalse();
    }

    [Fact]
    public void SweepCircleAgainst3D_WithStartingOverlapInsideCompoundPart_ShouldReturnStableHit()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCompoundCollider> compound = CreateCompound3D(context, Vector3d.Zero, immovable: true);

        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d((Fixed64)(-1), Fixed64.Zero),
            new Vector2d(Fixed64.One, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(compound.Collider);
        hit.Distance.Should().Be(Fixed64.Zero);
        hit.Point3D.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
        hit.Normal3DTo2D.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void LateSimulate_WithMixed3DContinuousCollision_ShouldClampBeforeCrossing2DSlab()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        ScenarioBody<LSSphereCollider> falling = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero));
        _ = CreateBodylessBox2D(context, Vector2d.Zero, new Vector2d((Fixed64)4, (Fixed64)4));

        falling.Body.AddForce(Vector3d.Down * (Fixed64)10);
        context.Simulate();
        context.LateSimulate();

        falling.Body.Position3d.Y.Should().BeGreaterThanOrEqualTo(Fixed64.One);
        falling.Body.LinearVelocity.Y.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void LateSimulate_WithMixed2DContinuousCollision_ShouldClampBeforeCrossing3DPrimitive()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        _ = CreateSphere3D(context, Vector3d.Zero, immovable: true);
        SolidBody2D moving2D = CreateCircle2D(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));

        moving2D.AddForce(Vector2d.Right * (Fixed64)10);
        context.Simulate();
        context.LateSimulate();

        moving2D.Position.X.Should().BeLessThanOrEqualTo((Fixed64)(-1));
        moving2D.LinearVelocity.X.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void LateSimulate_WithMixed2DContinuousCollision_ShouldClampBeforeCrossing3DMesh()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        _ = CreateMesh3D(
            context,
            MeshTestFixtures.CreateVerticalQuad(Fixed64.Zero, -Fixed64.One, Fixed64.One),
            Vector3d.Zero,
            immovable: true);
        SolidBody2D moving2D = CreateCircle2D(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));

        moving2D.AddForce(Vector2d.Right * (Fixed64)10);
        context.Simulate();
        context.LateSimulate();

        moving2D.Position.X.Should().BeLessThanOrEqualTo(-Fixed64.Half);
        moving2D.LinearVelocity.X.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void LateSimulate_WithMixed2DContinuousCollision_ShouldClampBeforeCrossing3DCompound()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        _ = CreateCompound3D(context, Vector3d.Zero, immovable: true);
        SolidBody2D moving2D = CreateCircle2D(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));

        moving2D.AddForce(Vector2d.Right * (Fixed64)10);
        context.Simulate();
        context.LateSimulate();

        moving2D.Position.X.Should().BeLessThanOrEqualTo((Fixed64)(-2));
        moving2D.LinearVelocity.X.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void LateSimulate_WithMixedDynamicContinuousCollision_ShouldClampBothAtSharedTimeOfImpact()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D body2D = CreateCircle2D(context, new Vector2d((Fixed64)5, Fixed64.Zero));
        body3D.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        body2D.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        body3D.Body.AddForce(Vector3d.Right * (Fixed64)5);
        body2D.AddForce(-Vector2d.Right * (Fixed64)5);
        context.LateSimulate();

        body3D.Body.Position3d.X.Should().BeLessThanOrEqualTo(-Fixed64.Half);
        body2D.Position.X.Should().BeGreaterThanOrEqualTo(Fixed64.Half);
        (body2D.Position.X - body3D.Body.Position3d.X).Should().BeGreaterThanOrEqualTo(Fixed64.One);
        body3D.Body.LinearVelocity.X.Should().BeLessThanOrEqualTo(Fixed64.Zero);
        body2D.LinearVelocity.X.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
    }

    [Fact]
    public void LateSimulate_WithMixed3DSourceCcdRestitutionThreshold_ShouldSuppressDynamicBounce()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Settings.RestitutionVelocityThreshold = (Fixed64)5;
        ScenarioBody<LSSphereCollider> source3D = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D target2D = CreateCircle2D(context, Vector2d.Zero);
        source3D.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target2D.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source3D.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        target2D.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);

        source3D.Body.AddForce(Vector3d.Right * (Fixed64)4);
        context.LateSimulate();

        source3D.Body.LinearVelocity.X.Should().Be((Fixed64)2);
        target2D.LinearVelocity.X.Should().Be((Fixed64)2);
    }

    [Fact]
    public void LateSimulate_WithMixed3DSourceZeroRestitutionThreshold_ShouldBounceDynamicContact()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Settings.RestitutionVelocityThreshold = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> source3D = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D target2D = CreateCircle2D(context, Vector2d.Zero);
        source3D.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target2D.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source3D.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        target2D.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);

        source3D.Body.AddForce(Vector3d.Right * (Fixed64)4);
        context.LateSimulate();

        source3D.Body.LinearVelocity.X.Should().Be(Fixed64.Zero);
        target2D.LinearVelocity.X.Should().Be((Fixed64)4);
    }

    [Fact]
    public void LateSimulate_WithMixed2DSourceCcdRestitutionThreshold_ShouldSuppressDynamicBounce()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Settings.RestitutionVelocityThreshold = (Fixed64)5;
        SolidBody2D source2D = CreateCircle2D(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));
        ScenarioBody<LSSphereCollider> target3D = CreateSphere3D(context, Vector3d.Zero);
        source2D.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target3D.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source2D.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        target3D.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);

        source2D.AddForce(Vector2d.Right * (Fixed64)4);
        context.LateSimulate();

        source2D.LinearVelocity.X.Should().Be((Fixed64)2);
        target3D.Body.LinearVelocity.X.Should().Be((Fixed64)2);
    }

    [Fact]
    public void LateSimulate_WithMixed2DSourceZeroRestitutionThreshold_ShouldBounceDynamicContact()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Settings.RestitutionVelocityThreshold = Fixed64.Zero;
        SolidBody2D source2D = CreateCircle2D(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));
        ScenarioBody<LSSphereCollider> target3D = CreateSphere3D(context, Vector3d.Zero);
        source2D.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target3D.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source2D.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        target3D.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);

        source2D.AddForce(Vector2d.Right * (Fixed64)4);
        context.LateSimulate();

        source2D.LinearVelocity.X.Should().Be(Fixed64.Zero);
        target3D.Body.LinearVelocity.X.Should().Be((Fixed64)4);
    }

    [Fact]
    public void LateSimulate_WithMixedDynamicChain_ShouldRelayHandoffAcrossServicesDeterministically()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> receiver = CreateSphere3D(
            context,
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));
        SolidBody2D middle = CreateCircle2D(context, Vector2d.Zero);
        ScenarioBody<LSSphereCollider> driver = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero));
        driver.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        middle.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        receiver.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        middle.Sleep();
        receiver.Body.Sleep();

        driver.Body.AddForce(Vector3d.Right * (Fixed64)10);
        context.LateSimulate();

        middle.IsSleeping.Should().BeFalse();
        receiver.Body.IsSleeping.Should().BeFalse();
        receiver.Body.Position3d.X.Should().BeGreaterThan((Fixed64)2);
        (receiver.Body.Position3d.X - middle.Position.X).Should().BeGreaterThan(Fixed64.FromFraction(19, 20));
        (middle.Position.X - driver.Body.Position3d.X).Should().BeGreaterThan(Fixed64.FromFraction(19, 20));
        context.Physics.LastContinuousCollisionIslandCount.Should().Be(1);
        context.Physics.LastContinuousCollisionIslandIterationCount.Should().BeGreaterThanOrEqualTo(1);
        context.Physics.LastContinuousCollisionIslandLimitReached.Should().BeFalse();
    }

    [Fact]
    public void LateSimulate_WithKinematic3DSourceCrossingDynamic2DSlab_ShouldTransferVelocityAtSweptToi()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        SolidBody2D target = CreateCircle2D(context, Vector2d.Zero);
        target.Sleep();
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Half);
        target.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Half);

        source.Body.Agent.Transform.Position = new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        context.LateSimulate();

        source.Body.Position3d.X.Should().Be((Fixed64)5);
        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        target.Position.X.Should().BeGreaterThan(Fixed64.Zero);
        target.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        target.IsSleeping.Should().BeFalse();
    }

    [Fact]
    public void LateSimulate_WithKinematic3DSourceCrossingDynamic2DSlab_ShouldNotTransferVelocityAcrossFrozenTargetAxis()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        SolidBody2D target = CreateCircle2D(context, Vector2d.Zero);
        target.FreezeAxes = BodyFreezeAxes2D.PositionX;
        target.Sleep();
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.Body.Agent.Transform.Position = new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        context.LateSimulate();

        source.Body.Position3d.X.Should().Be((Fixed64)5);
        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(0);
        target.Position.Should().Be(Vector2d.Zero);
        target.LinearVelocity.Should().Be(Vector2d.Zero);
        target.IsSleeping.Should().BeTrue();
    }

    [Fact]
    public void LateSimulate_WithKinematic3DSource_ShouldNotPushDynamic2DTargetBehindEarlierStaticSlab()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        _ = CreateCircle2D(context, Vector2d.Zero, immovable: true);
        SolidBody2D target = CreateCircle2D(context, new Vector2d((Fixed64)3, Fixed64.Zero));
        target.Sleep();
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.Body.Agent.Transform.Position = new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        context.LateSimulate();

        Fixed64 expectedFirstHitX = -Fixed64.One;
        Fixed64 tolerance = Fixed64.FromFraction(1, 1024);
        source.Body.Position3d.X.Should().BeGreaterThanOrEqualTo(expectedFirstHitX - tolerance);
        source.Body.Position3d.X.Should().BeLessThanOrEqualTo(expectedFirstHitX + tolerance);
        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        target.Position.X.Should().Be((Fixed64)3);
        target.LinearVelocity.Should().Be(Vector2d.Zero);
        target.IsSleeping.Should().BeTrue();
    }

    [Fact]
    public void LateSimulate_WithKinematic2DSourceCrossingDynamic3DTarget_ShouldTransferVelocityAtSweptToi()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(context, Vector3d.Zero);
        target.Body.Sleep();
        SolidBody2D source = CreateCircle2D(
            context,
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Half);
        target.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Half);

        source.Agent.Transform.Position = new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        context.LateSimulate();

        source.Position.X.Should().Be((Fixed64)5);
        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
        target.Body.Position3d.X.Should().BeGreaterThan(Fixed64.Zero);
        target.Body.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        target.Body.IsSleeping.Should().BeFalse();
        context.Physics.LastContinuousCollisionIslandIterationCount.Should().Be(1);
    }

    [Fact]
    public void LateSimulate_WithKinematic2DSourceCrossingDynamic3DTarget_ShouldNotTransferVelocityAcrossFrozenTargetAxis()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(context, Vector3d.Zero);
        target.Body.FreezeAxes = BodyFreezeAxes3D.PositionX;
        target.Body.Sleep();
        SolidBody2D source = CreateCircle2D(
            context,
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.Agent.Transform.Position = new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        context.LateSimulate();

        source.Position.X.Should().Be((Fixed64)5);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        target.Body.Position3d.Should().Be(Vector3d.Zero);
        target.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        target.Body.IsSleeping.Should().BeTrue();
    }

    [Fact]
    public void LateSimulate_WithKinematic2DSource_ShouldNotPushDynamic3DTargetBehindEarlierStaticPrimitive()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        _ = CreateSphere3D(context, Vector3d.Zero, immovable: true);
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(context, new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero));
        target.Body.Sleep();
        SolidBody2D source = CreateCircle2D(
            context,
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.Agent.Transform.Position = new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        context.LateSimulate();

        Fixed64 expectedFirstHitX = -Fixed64.One;
        Fixed64 tolerance = Fixed64.FromFraction(1, 1024);
        source.Position.X.Should().BeGreaterThanOrEqualTo(expectedFirstHitX - tolerance);
        source.Position.X.Should().BeLessThanOrEqualTo(expectedFirstHitX + tolerance);
        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
        target.Body.Position3d.X.Should().Be((Fixed64)3);
        target.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        target.Body.IsSleeping.Should().BeTrue();
    }

    [Fact]
    public void LateSimulate_WithKinematic2DSourceCrossingDynamic3DTarget_ShouldPreserveHandoffVelocityAfterMixedResponse()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(context, Vector3d.Zero);
        target.Body.Sleep();
        SolidBody2D source = CreateCircle2D(
            context,
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        target.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);

        source.Agent.Transform.Position = new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        context.LateSimulate();

        source.Position.X.Should().Be((Fixed64)5);
        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
        target.Body.Position3d.X.Should().BeGreaterThan(Fixed64.Zero);
        target.Body.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        target.Body.IsSleeping.Should().BeFalse();
    }

    [Fact]
    public void LateSimulate_With3DTo2DMixedHandoffChain_ShouldHonorSingleContextHandoffBudget()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Settings.ContinuousCollisionMaxToiIterations = 1;
        ScenarioBody<LSSphereCollider> receiver = CreateSphere3D(
            context,
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));
        SolidBody2D middle = CreateCircle2D(context, Vector2d.Zero);
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        middle.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        receiver.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        middle.Sleep();
        receiver.Body.Sleep();

        source.Body.Agent.Transform.Position = new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        context.LateSimulate();

        middle.IsSleeping.Should().BeFalse();
        receiver.Body.IsSleeping.Should().BeFalse();
        middle.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        receiver.Body.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        (context.Physics.LastContinuousCollisionIslandIterationCount
            + context.Physics2D.LastContinuousCollisionIslandIterationCount)
            .Should()
            .Be(1);
    }

    [Fact]
    public void LateSimulate_With2DTo3DMixedHandoffChain_ShouldHonorSingleContextHandoffBudget()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Settings.ContinuousCollisionMaxToiIterations = 1;
        SolidBody2D receiver = CreateCircle2D(context, new Vector2d((Fixed64)2, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> middle = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D source = CreateCircle2D(
            context,
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        middle.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        receiver.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        middle.Body.Sleep();
        receiver.Sleep();

        source.Agent.Transform.Position = new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        context.LateSimulate();

        middle.Body.IsSleeping.Should().BeFalse();
        receiver.IsSleeping.Should().BeFalse();
        middle.Body.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        receiver.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        (context.Physics.LastContinuousCollisionIslandIterationCount
            + context.Physics2D.LastContinuousCollisionIslandIterationCount)
            .Should()
            .Be(1);
    }

    [Fact]
    public void LateSimulate_WithIndependent3DAnd2DHandoffQueues_ShouldHonorSingleContextBudget()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Settings.ContinuousCollisionMaxToiIterations = 1;
        Fixed64 lane3D = (Fixed64)(-4);
        Fixed64 lane2D = (Fixed64)4;

        ScenarioBody<LSSphereCollider> middle3D = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, lane3D));
        ScenarioBody<LSSphereCollider> receiver3D = CreateSphere3D(
            context,
            new Vector3d((Fixed64)2, Fixed64.Zero, lane3D));
        ScenarioBody<LSSphereCollider> driver3D = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-5), Fixed64.Zero, lane3D));
        SolidBody2D middle2D = CreateCircle2D(context, new Vector2d(Fixed64.Zero, lane2D));
        SolidBody2D receiver2D = CreateCircle2D(context, new Vector2d((Fixed64)2, lane2D));
        SolidBody2D driver2D = CreateCircle2D(context, new Vector2d((Fixed64)(-5), lane2D));

        middle3D.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        receiver3D.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        driver3D.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        middle2D.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        receiver2D.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        driver2D.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        middle3D.Body.Sleep();
        receiver3D.Body.Sleep();
        middle2D.Sleep();
        receiver2D.Sleep();

        driver3D.Body.AddForce(Vector3d.Right * (Fixed64)10);
        driver2D.AddForce(Vector2d.Right * (Fixed64)10);
        context.LateSimulate();

        middle3D.Body.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        middle2D.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        context.Physics.LastContinuousCollisionIslandIterationCount.Should().Be(1);
        context.Physics.LastContinuousCollisionIslandLimitReached.Should().BeTrue();
        context.Physics2D.LastContinuousCollisionIslandIterationCount.Should().Be(0);
        context.Physics2D.LastContinuousCollisionIslandLimitReached.Should().BeTrue();
    }

    [Fact]
    public void SweepSphereAgainstStatic2DAll_ShouldCollectOnlyStaticStyle2DTargets()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D movable = CreateCircle2D(context, new Vector2d(Fixed64.Zero, Fixed64.Zero));
        SolidBody2D kinematic = CreateCircle2D(
            context,
            new Vector2d(Fixed64.Zero, Fixed64.One),
            immovable: false,
            isKinematic: true);
        SolidBody2D immovable = CreateCircle2D(
            context,
            new Vector2d(Fixed64.Zero, -Fixed64.One),
            immovable: true);
        SolidBody2D nonDynamic = CreateCircle2D(
            context,
            new Vector2d((Fixed64)2, Fixed64.Zero),
            isDynamic: false);
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepSphereAgainstStatic2DAll(
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            hits,
            source.Collider,
            includeTriggers: false);

        count.Should().Be(3);
        context.QueryMixed.LastQueryCandidateCount.Should().Be(3);
        hits.Should().OnlyContain(hit => !ReferenceEquals(hit.Collider2D, movable.Collider));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider2D, kinematic.Collider));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider2D, immovable.Collider));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider2D, nonDynamic.Collider));
    }

    [Fact]
    public void SweepCircleAgainstStatic3DAll_ShouldCollectOnlyStaticStyle3DTargets()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        SolidBody2D source = CreateCircle2D(context, new Vector2d((Fixed64)(-5), Fixed64.Zero));
        ScenarioBody<LSSphereCollider> movable = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> kinematic = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.One),
            isKinematic: true);
        ScenarioBody<LSSphereCollider> immovable = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, -Fixed64.One),
            immovable: true);
        ScenarioBody<LSSphereCollider> nonDynamic = CreateSphere3D(
            context,
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            isDynamic: false);
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepCircleAgainstStatic3DAll(
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            new Vector2d((Fixed64)5, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            hits,
            source.Collider,
            includeTriggers: false);

        count.Should().Be(3);
        context.QueryMixed.LastQueryCandidateCount.Should().Be(3);
        hits.Should().OnlyContain(hit => !ReferenceEquals(hit.Collider3D, movable.Collider));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider3D, kinematic.Collider));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider3D, immovable.Collider));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider3D, nonDynamic.Collider));
    }

    [Fact]
    public void SweepSphereAgainstStatic2DAll_WithCachedTargetRefresh_ShouldRefreshOnNextLateToken()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D target = CreateCircle2D(context, Vector2d.Zero, immovable: true);
        var hits = new SwiftList<PhysicsMixedHit>();

        context.AdvanceLateSimulateToken();
        int firstCount = context.QueryMixed.SweepSphereAgainstStatic2DAll(
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            hits,
            source.Collider,
            includeTriggers: false,
            cacheTargetPartitions: true);

        target.SetPosition(new Vector2d((Fixed64)20, Fixed64.Zero));
        context.AdvanceLateSimulateToken();
        int secondCount = context.QueryMixed.SweepSphereAgainstStatic2DAll(
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            hits,
            source.Collider,
            includeTriggers: false,
            cacheTargetPartitions: true);

        firstCount.Should().Be(1);
        secondCount.Should().Be(0);
    }

    [Fact]
    public void SweepCircleAgainstStatic3DAll_WithCachedTargetRefresh_ShouldRefreshOnNextLateToken()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        SolidBody2D source = CreateCircle2D(context, new Vector2d((Fixed64)(-5), Fixed64.Zero));
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(
            context,
            Vector3d.Zero,
            immovable: true);
        var hits = new SwiftList<PhysicsMixedHit>();

        context.AdvanceLateSimulateToken();
        int firstCount = context.QueryMixed.SweepCircleAgainstStatic3DAll(
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            new Vector2d((Fixed64)5, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            hits,
            source.Collider,
            includeTriggers: false,
            cacheTargetPartitions: true);

        target.Body.SetPosition(new Vector3d((Fixed64)20, Fixed64.Zero, Fixed64.Zero));
        context.AdvanceLateSimulateToken();
        int secondCount = context.QueryMixed.SweepCircleAgainstStatic3DAll(
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            new Vector2d((Fixed64)5, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            hits,
            source.Collider,
            includeTriggers: false,
            cacheTargetPartitions: true);

        firstCount.Should().Be(1);
        secondCount.Should().Be(0);
    }

    [Fact]
    public void MixedDiagnostics_ShouldRecordContactResponseAndDimensionTaggedPayloads()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        body3D.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Down);
        context.Diagnostics.Enable(eventCapacity: 16, drawCommandCapacity: 0);

        context.Simulate();
        context.LateSimulate();

        ReadOnlySpan<GravitasDiagnosticEvent> events = context.Diagnostics.Events;
        GravitasDiagnosticEvent mixedContact = FindDiagnosticEvent(events, GravitasDiagnosticEventKind.MixedContact);
        mixedContact.ColliderADimension.Should().Be(GravitasColliderDimension.ThreeD);
        mixedContact.ColliderBDimension.Should().Be(GravitasColliderDimension.TwoD);
        mixedContact.ColliderAId.Should().Be(body3D.Collider.Id);
        mixedContact.ColliderBId.Should().Be(body2D.Collider.Id);
        mixedContact.ColliderAType.Should().Be(ColliderType.Sphere);
        mixedContact.ColliderB2DType.Should().Be(ColliderType2D.Circle);
        mixedContact.ScalarA.Should().BeGreaterThan(Fixed64.Zero);
        mixedContact.Hit.Should().BeTrue();

        GravitasDiagnosticEvent mixedImpulse = FindDiagnosticEvent(events, GravitasDiagnosticEventKind.MixedResponseImpulse);
        mixedImpulse.ColliderADimension.Should().Be(GravitasColliderDimension.ThreeD);
        mixedImpulse.ColliderBDimension.Should().Be(GravitasColliderDimension.TwoD);
        mixedImpulse.Vector.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
        mixedImpulse.ScalarA.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void CaptureMixedCollider_ShouldDrawEmbeddedSlabGeometry()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D circle = CreateBodylessCircle2D(context, new Vector2d((Fixed64)2, (Fixed64)3));
        LSCollider2D capsule = CreateBodylessCapsule2D(context, Vector2d.Zero);
        LSCollider2D box = CreateBodylessBox2D(
            context,
            new Vector2d((Fixed64)(-2), (Fixed64)(-3)),
            new Vector2d((Fixed64)4, (Fixed64)2));
        context.Diagnostics.Enable(eventCapacity: 0, drawCommandCapacity: 6);

        context.Diagnostics.CaptureMixedCollider(circle, GravitasDiagnosticColor.Cyan);
        context.Diagnostics.CaptureMixedCollider(capsule, GravitasDiagnosticColor.Green);
        context.Diagnostics.CaptureMixedCollider(box, GravitasDiagnosticColor.Yellow);

        ReadOnlySpan<GravitasDebugDrawCommand> commands = context.Diagnostics.DrawCommands;
        commands.Length.Should().Be(5);
        commands[0].Kind.Should().Be(GravitasDebugDrawKind.WireCylinder);
        commands[0].ColliderDimension.Should().Be(GravitasColliderDimension.TwoD);
        commands[0].Collider2DType.Should().Be(ColliderType2D.Circle);
        commands[0].Center.Should().Be(new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)3));
        commands[0].Height.Should().Be(context.Settings.Mixed2DHalfThickness * 2);
        commands[1].Kind.Should().Be(GravitasDebugDrawKind.WireCylinder);
        commands[1].ColliderDimension.Should().Be(GravitasColliderDimension.TwoD);
        commands[1].Collider2DType.Should().Be(ColliderType2D.Capsule);
        commands[1].Radius.Should().Be(Fixed64.Half);
        commands[1].Height.Should().Be(context.Settings.Mixed2DHalfThickness * 2);
        commands[2].Kind.Should().Be(GravitasDebugDrawKind.WireCylinder);
        commands[2].Collider2DType.Should().Be(ColliderType2D.Capsule);
        commands[3].Kind.Should().Be(GravitasDebugDrawKind.WireBox);
        commands[3].Collider2DType.Should().Be(ColliderType2D.Capsule);
        commands[3].Size.Should().Be(new Vector3d(Fixed64.One, context.Settings.Mixed2DHalfThickness * 2, (Fixed64)2));
        commands[4].Kind.Should().Be(GravitasDebugDrawKind.WireBox);
        commands[4].ColliderDimension.Should().Be(GravitasColliderDimension.TwoD);
        commands[4].Collider2DType.Should().Be(ColliderType2D.AABox);
        commands[4].Center.Should().Be(new Vector3d((Fixed64)(-2), Fixed64.Zero, (Fixed64)(-3)));
        commands[4].Size.Should().Be(new Vector3d((Fixed64)4, context.Settings.Mixed2DHalfThickness * 2, (Fixed64)2));
    }

    private static GravitasWorldContext CreateMixedContext(int frameRate = 4)
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.ApplySettings(new PhysicsSettings(frameRate, null));
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-8), (Fixed64)(-4), (Fixed64)(-8)),
                new Vector3d((Fixed64)8, (Fixed64)4, (Fixed64)8)),
            out _).Should().BeTrue();
        return context;
    }

    private static ScenarioBody<LSSphereCollider> CreateSphere3D(
        GravitasWorldContext context,
        Vector3d position,
        bool immovable = false,
        bool isKinematic = false,
        bool isDynamic = true)
    {
        return CreateBody3D(
            context,
            new LSSphereCollider(),
            position,
            immovable: immovable,
            isKinematic: isKinematic,
            isDynamic: isDynamic);
    }

    private static ScenarioBody<LSMeshCollider> CreateMesh3D(
        GravitasWorldContext context,
        LSMeshCollider collider,
        Vector3d position,
        bool immovable = false)
    {
        return CreateBody3D(context, collider, position, immovable: immovable);
    }

    private static ScenarioBody<LSCompoundCollider> CreateCompound3D(
        GravitasWorldContext context,
        Vector3d position,
        bool immovable = false)
    {
        var collider = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d((Fixed64)(-1), Fixed64.Zero, Fixed64.Zero)),
            CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero)));
        return CreateBody3D(context, collider, position, immovable: immovable);
    }

    private static LSMeshCollider CreateSlabClippedProxyOnlyTriangle()
    {
        Fixed64 zNearBroadPhase = Fixed64.FromFraction(49, 100);
        Fixed64 zClippedMiss = Fixed64.FromFraction(71, 100);
        return new LSMeshCollider(
            new[]
            {
                new Vector3d(Fixed64.Zero, Fixed64.Zero, zNearBroadPhase),
                new Vector3d(Fixed64.Zero, (Fixed64)2, zClippedMiss),
                new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.One)
            },
            new[] { 0, 1, 2 },
            MeshColliderMode.Convex,
            MeshInertiaPolicy.SurfaceApproximation);
    }

    private static LSMeshCollider CreateSlabBoundaryTriangle()
    {
        return CreateOpenTriangleMesh(
            new Vector3d(Fixed64.Zero, Fixed64.Half, -Fixed64.One),
            new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.One),
            new Vector3d(Fixed64.One, Fixed64.Half, Fixed64.Zero));
    }

    private static LSMeshCollider CreatePointProjectionTriangle()
    {
        return CreateOpenTriangleMesh(
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero),
            new Vector3d(-Fixed64.One, (Fixed64)2, Fixed64.One),
            new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One));
    }

    private static LSMeshCollider CreateSegmentProjectionTriangle()
    {
        return CreateOpenTriangleMesh(
            new Vector3d(Fixed64.Zero, Fixed64.One, -Fixed64.One),
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.One),
            new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.Zero));
    }

    private static LSMeshCollider CreateEqualDistanceTriangleMesh()
    {
        return new LSMeshCollider(
            new[]
            {
                new Vector3d(Fixed64.Zero, Fixed64.Half, -Fixed64.One),
                new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.One),
                new Vector3d(Fixed64.One, Fixed64.Half, Fixed64.Zero),
                new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 2), -Fixed64.One),
                new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 2), Fixed64.One),
                new Vector3d(Fixed64.One, Fixed64.FromFraction(3, 2), Fixed64.Zero)
            },
            new[] { 0, 1, 2, 3, 4, 5 },
            MeshColliderMode.Convex,
            MeshInertiaPolicy.SurfaceApproximation);
    }

    private static LSMeshCollider CreateOpenTriangleMesh(Vector3d first, Vector3d second, Vector3d third)
    {
        return new LSMeshCollider(
            new[] { first, second, third },
            new[] { 0, 1, 2 },
            MeshColliderMode.Convex,
            MeshInertiaPolicy.SurfaceApproximation);
    }

    private static LSCompoundCollider CreateEqualDistanceMeshPartCompound()
    {
        Vector3d[] vertices =
        {
            new(Fixed64.Zero, Fixed64.Zero, -Fixed64.One),
            new(Fixed64.Zero, Fixed64.Zero, Fixed64.One),
            new(Fixed64.One, Fixed64.Zero, Fixed64.Zero)
        };
        int[] triangles = { 0, 1, 2 };
        return new LSCompoundCollider(
            CompoundColliderPart.ConvexMesh(
                vertices,
                triangles,
                new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Zero),
                MeshInertiaPolicy.SurfaceApproximation),
            CompoundColliderPart.ConvexMesh(
                vertices,
                triangles,
                new Vector3d(Fixed64.Half, Fixed64.FromFraction(3, 2), Fixed64.Zero),
                MeshInertiaPolicy.SurfaceApproximation));
    }

    private static ScenarioBody<TCollider> CreateBody3D<TCollider>(
        GravitasWorldContext context,
        TCollider collider,
        Vector3d position,
        bool immovable = false,
        bool isKinematic = false,
        FixedQuaternion? rotation = null,
        bool isDynamic = true)
        where TCollider : LSCollider
    {
        FixedQuaternion startRotation = rotation ?? FixedQuaternion.Identity;
        var agent = new TestMatterAgent(context, new FixedTransform(position, startRotation, Vector3d.One));
        var body = new SolidBody(agent, collider)
        {
            Mass = Fixed64.One,
            FreezeAxes = immovable ? BodyFreezeAxes3D.Position : BodyFreezeAxes3D.None,
            IsKinematic = isKinematic
        };
        collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero);
        body.Initialize(position, startRotation, isDynamic);
        return new ScenarioBody<TCollider>(body, collider);
    }

    private static SolidBody2D CreateCircle2D(
        GravitasWorldContext context,
        Vector2d position,
        bool immovable = false,
        bool isKinematic = false,
        bool isDynamic = true)
    {
        var collider = new LSCircleCollider2D(Fixed64.Half);
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One));
        var body = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One,
            FreezeAxes = immovable ? BodyFreezeAxes2D.Position : BodyFreezeAxes2D.None,
            IsKinematic = isKinematic
        };
        collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero);
        body.Initialize(position, isDynamic: isDynamic);
        return body;
    }

    private static LSCollider2D CreateBodylessCircle2D(GravitasWorldContext context, Vector2d position)
    {
        var collider = new LSCircleCollider2D(Fixed64.Half);
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One));
        collider.InitializeWithNoBody(agent);
        return collider;
    }

    private static LSCollider2D CreateBodylessUnsupported2D(GravitasWorldContext context, Vector2d position)
    {
        var collider = new UnsupportedTestCollider2D();
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One));
        collider.InitializeWithNoBody(agent);
        return collider;
    }

    private static LSCollider2D CreateBodylessCapsule2D(GravitasWorldContext context, Vector2d position)
    {
        var collider = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3);
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One));
        collider.InitializeWithNoBody(agent);
        return collider;
    }

    private static LSCollider2D CreateBodylessBox2D(
        GravitasWorldContext context,
        Vector2d position,
        Vector2d size)
    {
        var collider = new LSAABBoxCollider2D(size);
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One));
        collider.InitializeWithNoBody(agent);
        return collider;
    }

    private static LSCollider2D CreateBodylessPolygon2D(
        GravitasWorldContext context,
        Vector2d position,
        Vector2d[] vertices)
    {
        var collider = new LSPolygonCollider2D(vertices);
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One));
        collider.InitializeWithNoBody(agent);
        return collider;
    }

    private static LSCollider2D CreateBodylessCompound2D(GravitasWorldContext context, Vector2d position)
    {
        var collider = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero),
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)4, Fixed64.Zero)));
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One));
        collider.InitializeWithNoBody(agent);
        return collider;
    }

    private static LSCollider2D CreateBodylessCompound2D(
        GravitasWorldContext context,
        Vector2d position,
        params CompoundColliderPart2D[] parts)
    {
        var collider = new LSCompoundCollider2D(parts);
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One));
        collider.InitializeWithNoBody(agent);
        return collider;
    }

    private static Vector2d[] CreateDiamondVertices() =>
        new[]
        {
            new Vector2d(Fixed64.Zero, Fixed64.One),
            new Vector2d(Fixed64.One, Fixed64.Zero),
            new Vector2d(Fixed64.Zero, -Fixed64.One),
            new Vector2d(-Fixed64.One, Fixed64.Zero)
        };

    private static GravitasDiagnosticEvent FindDiagnosticEvent(
        ReadOnlySpan<GravitasDiagnosticEvent> events,
        GravitasDiagnosticEventKind kind)
    {
        for (int i = 0; i < events.Length; i++)
        {
            if (events[i].Kind == kind)
                return events[i];
        }

        throw new InvalidOperationException($"Expected diagnostic event kind {kind}.");
    }

    private static void AssertNear(Fixed64 actual, Fixed64 expected) =>
        (actual - expected).Abs().Should().BeLessThanOrEqualTo(Fixed64.FromFraction(1, 1000));
}
