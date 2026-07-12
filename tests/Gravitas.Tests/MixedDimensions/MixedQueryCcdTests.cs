using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Diagnostics;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using SwiftCollections;
using SwiftCollections.Query;
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
    public void MixedSweeps_ShouldHonorTriggerFilterInBothDirections()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D trigger2D = CreateBodylessCircle2D(context, Vector2d.Zero);
        trigger2D.IsTrigger = true;
        LSCollider trigger3D = CreateBodyless3D(
            context,
            new LSSphereCollider(),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)4));
        trigger3D.IsTrigger = true;

        bool excluded2DTrigger = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out _,
            includeTriggers: false);
        bool included2DTrigger = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit2D,
            includeTriggers: true);
        bool excluded3DTrigger = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d((Fixed64)(-3), (Fixed64)4),
            new Vector2d((Fixed64)3, (Fixed64)4),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            out _,
            includeTriggers: false);
        bool included3DTrigger = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d((Fixed64)(-3), (Fixed64)4),
            new Vector2d((Fixed64)3, (Fixed64)4),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit3D,
            includeTriggers: true);

        excluded2DTrigger.Should().BeFalse();
        included2DTrigger.Should().BeTrue();
        hit2D.Collider2D.Should().BeSameAs(trigger2D);
        excluded3DTrigger.Should().BeFalse();
        included3DTrigger.Should().BeTrue();
        hit3D.Collider3D.Should().BeSameAs(trigger3D);
    }

    [Fact]
    public void MixedSweeps_ShouldFilterTargetsOwnedByExcludedColliderAgent()
    {
        using GravitasWorldContext sphereAgainst2DContext = CreateMixedContext();
        var shared2DAgent = new TestMatterAgent(sphereAgainst2DContext);
        var excluded3D = new LSSphereCollider();
        excluded3D.InitializeWithNoBody(shared2DAgent);
        var target2D = new LSCircleCollider2D(Fixed64.Half);
        target2D.InitializeWithNoBody(shared2DAgent);

        bool hitShared2DTarget = sphereAgainst2DContext.QueryMixed.SweepSphereAgainst2D(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out _,
            excluded3D);

        using GravitasWorldContext circleAgainst3DContext = CreateMixedContext();
        var shared3DAgent = new TestMatterAgent(circleAgainst3DContext);
        var excluded2D = new LSCircleCollider2D(Fixed64.Half);
        excluded2D.InitializeWithNoBody(shared3DAgent);
        var target3D = new LSSphereCollider();
        target3D.InitializeWithNoBody(shared3DAgent);

        bool hitShared3DTarget = circleAgainst3DContext.QueryMixed.SweepCircleAgainst3D(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            out _,
            excluded2D);

        hitShared2DTarget.Should().BeFalse();
        hitShared3DTarget.Should().BeFalse();
        target2D.IsActive.Should().BeTrue();
        target3D.IsActive.Should().BeTrue();
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
    public void SweepCircleAgainst3D_WithSpherePointAboveSlab_ShouldClamp2DPointToUpperFace()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(context, Vector3d.Up, immovable: true);

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
        hit.Body3D.Should().BeSameAs(target.Body);
        hit.Collider2D.Should().BeNull();
        hit.Distance.Should().Be(Fixed64.FromFraction(5, 2));
        AssertNear(hit.Point3D.X, -FixedMath.Sqrt(Fixed64.FromFraction(1, 20)));
        AssertNear(hit.Point3D.Y, Fixed64.One - FixedMath.Sqrt(Fixed64.FromFraction(1, 5)));
        hit.Point3D.Z.Should().Be(Fixed64.Zero);
        hit.Point2D.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Zero));
        AssertNear(hit.Normal3DTo2D.X, -FixedMath.Sqrt(Fixed64.FromFraction(1, 5)));
        AssertNear(hit.Normal3DTo2D.Y, -FixedMath.Sqrt(Fixed64.FromFraction(4, 5)));
        hit.Normal3DTo2D.Z.Should().Be(Fixed64.Zero);
        hit.Direction3D.Should().Be(Vector3d.Right);
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
    public void FiniteSlabProjectionSweep_WithCylinderOutsideSlab_ShouldRejectWithoutProjection()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCylinderCollider> target = CreateBody3D(
            context,
            new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero),
            immovable: true);

        bool found = FiniteSlabProjectionSweep.TrySweepCircleAgainstCylinder(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)8,
            Fixed64.Half,
            -Fixed64.Half,
            Fixed64.Half,
            target.Collider,
            out Fixed64 distance);

        found.Should().BeFalse();
        distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void FiniteSlabProjectionSweep_WithCylinderBehindSweepDirection_ShouldRejectMovingAway()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCylinderCollider> target = CreateBody3D(
            context,
            new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true);

        bool found = FiniteSlabProjectionSweep.TrySweepCircleAgainstCylinder(
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)8,
            Fixed64.Half,
            -Fixed64.Half,
            Fixed64.Half,
            target.Collider,
            out Fixed64 distance);

        found.Should().BeFalse();
        distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void FiniteSlabProjectionSweep_WithCylinderPastSweepLength_ShouldRejectOutOfRangeHit()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCylinderCollider> target = CreateBody3D(
            context,
            new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true);

        bool found = FiniteSlabProjectionSweep.TrySweepCircleAgainstCylinder(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            Vector2d.Right,
            Fixed64.One,
            Fixed64.Half,
            -Fixed64.Half,
            Fixed64.Half,
            target.Collider,
            out Fixed64 distance);

        found.Should().BeFalse();
        distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void FiniteSlabProjectionSweep_WithCylinderStartingInsideProjection_ShouldReturnZeroDistance()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCylinderCollider> target = CreateBody3D(
            context,
            new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true);

        bool found = FiniteSlabProjectionSweep.TrySweepCircleAgainstCylinder(
            Vector2d.Zero,
            Vector2d.Right,
            (Fixed64)4,
            Fixed64.Half,
            -Fixed64.Half,
            Fixed64.Half,
            target.Collider,
            out Fixed64 distance);

        found.Should().BeTrue();
        distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void FiniteSlabProjectionSweep_WithCapsuleOutsideSlab_ShouldRejectWithoutProjection()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCapsuleCollider> target = CreateBody3D(
            context,
            new LSCapsuleCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            new Vector3d(Fixed64.Zero, (Fixed64)4, Fixed64.Zero),
            immovable: true);

        bool found = FiniteSlabProjectionSweep.TrySweepCircleAgainstCapsule(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)8,
            Fixed64.Half,
            -Fixed64.Half,
            Fixed64.Half,
            target.Collider,
            out Fixed64 distance);

        found.Should().BeFalse();
        distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void FiniteSlabProjectionSweep_WithConeOutsideSlab_ShouldRejectWithoutProjection()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSConeCollider> target = CreateBody3D(
            context,
            new LSConeCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            new Vector3d(Fixed64.Zero, (Fixed64)4, Fixed64.Zero),
            immovable: true);

        bool found = FiniteSlabProjectionSweep.TrySweepCircleAgainstCone(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)8,
            Fixed64.Half,
            -Fixed64.Half,
            Fixed64.Half,
            target.Collider,
            out Fixed64 distance);

        found.Should().BeFalse();
        distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void FiniteSlabProjectionSweep_WithConeBaseAndApexBands_ShouldRetainSupportExtrema()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSConeCollider> target = CreateBody3D(
            context,
            new LSConeCollider { Size = new Vector3d((Fixed64)2, (Fixed64)4, (Fixed64)2) },
            Vector3d.Zero,
            immovable: true);
        Fixed64 bandHalfHeight = Fixed64.FromFraction(1, 10);
        Fixed64 baseY = target.Collider.WorldBaseCenter.Y;
        Fixed64 apexY = target.Collider.WorldApex.Y;

        bool baseFound = FiniteSlabProjectionSweep.TrySweepCircleAgainstCone(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)8,
            Fixed64.Half,
            baseY - bandHalfHeight,
            baseY + bandHalfHeight,
            target.Collider,
            out Fixed64 baseDistance);
        bool apexFound = FiniteSlabProjectionSweep.TrySweepCircleAgainstCone(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)8,
            Fixed64.Half,
            apexY,
            apexY + bandHalfHeight,
            target.Collider,
            out Fixed64 apexDistance);

        baseFound.Should().BeTrue();
        baseDistance.Should().Be((Fixed64)3);
        apexFound.Should().BeTrue();
        apexDistance.Should().Be(Fixed64.FromFraction(7, 2));
    }

    [Fact]
    public void FiniteSlabProjectionSweep_WithCapsuleAndConeBelowSlab_ShouldRejectWithoutProjection()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCapsuleCollider> capsule = CreateBody3D(
            context,
            new LSCapsuleCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            new Vector3d(Fixed64.Zero, (Fixed64)(-4), Fixed64.Zero),
            immovable: true);
        ScenarioBody<LSConeCollider> cone = CreateBody3D(
            context,
            new LSConeCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            new Vector3d(Fixed64.Zero, (Fixed64)(-4), Fixed64.Zero),
            immovable: true);

        bool capsuleFound = FiniteSlabProjectionSweep.TrySweepCircleAgainstCapsule(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)8,
            Fixed64.Half,
            -Fixed64.Half,
            Fixed64.Half,
            capsule.Collider,
            out Fixed64 capsuleDistance);
        bool coneFound = FiniteSlabProjectionSweep.TrySweepCircleAgainstCone(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)8,
            Fixed64.Half,
            -Fixed64.Half,
            Fixed64.Half,
            cone.Collider,
            out Fixed64 coneDistance);

        capsuleFound.Should().BeFalse();
        capsuleDistance.Should().Be(Fixed64.Zero);
        coneFound.Should().BeFalse();
        coneDistance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void FiniteSlabProjectionSweep_WithHorizontalConeAxisSupport_ShouldUseStablePlanarFallback()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSConeCollider> target = CreateBody3D(
            context,
            new LSConeCollider { Size = new Vector3d((Fixed64)2, (Fixed64)4, (Fixed64)2) },
            Vector3d.Zero,
            immovable: true,
            rotation: FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90));

        bool firstFound = FiniteSlabProjectionSweep.TrySweepCircleAgainstCone(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)8,
            Fixed64.Half,
            -Fixed64.FromFraction(1, 10),
            Fixed64.FromFraction(1, 10),
            target.Collider,
            out Fixed64 firstDistance);
        bool secondFound = FiniteSlabProjectionSweep.TrySweepCircleAgainstCone(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)8,
            Fixed64.Half,
            -Fixed64.FromFraction(1, 10),
            Fixed64.FromFraction(1, 10),
            target.Collider,
            out Fixed64 secondDistance);

        firstFound.Should().BeTrue();
        secondFound.Should().BeTrue();
        firstDistance.Should().BeGreaterThan(Fixed64.Zero);
        firstDistance.Should().BeLessThan((Fixed64)8);
        secondDistance.Should().Be(firstDistance);
    }

    [Fact]
    public void FiniteSlabProjectionSweep_WithHorizontalCylinderZeroRadialSupport_ShouldUseRemainingSupports()
    {
        using GravitasWorldContext context = CreateMixedContext();
        Fixed64 almostHorizontal = (Fixed64)90 - Fixed64.FromFraction(1, 500_000);
        ScenarioBody<LSCylinderCollider> target = CreateBody3D(
            context,
            new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true,
            rotation: FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, almostHorizontal));
        Vector3d radialDirection = Vector3d.Right
            - target.Collider.LineDirection * Vector3d.Dot(Vector3d.Right, target.Collider.LineDirection);

        bool found = FiniteSlabProjectionSweep.TrySweepCircleAgainstCylinder(
            Vector2d.Zero,
            Vector2d.Right,
            (Fixed64)4,
            Fixed64.Half,
            -Fixed64.FromFraction(1, 10),
            Fixed64.FromFraction(1, 10),
            target.Collider,
            out Fixed64 distance);

        (target.Collider.LineSegmentEnd.Y - target.Collider.LineSegmentStart.Y).Abs().Should().BeGreaterThan(Fixed64.Epsilon);
        radialDirection.Magnitude.Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        found.Should().BeTrue();
        distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void FiniteSlabProjectionSweep_WithTiltedCapsuleNarrowSlab_ShouldHitClippedProjection()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCapsuleCollider> target = CreateBody3D(
            context,
            new LSCapsuleCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true,
            rotation: FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)45));

        bool found = FiniteSlabProjectionSweep.TrySweepCircleAgainstCapsule(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)8,
            Fixed64.Half,
            -Fixed64.Half,
            Fixed64.Half,
            target.Collider,
            out Fixed64 distance);

        found.Should().BeTrue();
        distance.Should().BeGreaterThan(Fixed64.Zero);
        distance.Should().BeLessThan((Fixed64)4);
    }

    [Fact]
    public void FiniteSlabProjectionSweep_WithTiltedCylinderNarrowSlab_ShouldHitClippedProjection()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCylinderCollider> target = CreateBody3D(
            context,
            new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true,
            rotation: FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)45));

        bool found = FiniteSlabProjectionSweep.TrySweepCircleAgainstCylinder(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)8,
            Fixed64.Half,
            -Fixed64.Half,
            Fixed64.Half,
            target.Collider,
            out Fixed64 distance);

        found.Should().BeTrue();
        distance.Should().BeGreaterThan(Fixed64.Zero);
        distance.Should().BeLessThan((Fixed64)4);
    }

    [Fact]
    public void FiniteSlabProjectionSweep_WithVerticalConeNarrowSlab_ShouldHitSliceProjection()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSConeCollider> target = CreateBody3D(
            context,
            new LSConeCollider { Size = new Vector3d((Fixed64)2, (Fixed64)4, (Fixed64)2) },
            Vector3d.Zero,
            immovable: true);

        bool found = FiniteSlabProjectionSweep.TrySweepCircleAgainstCone(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)8,
            Fixed64.Half,
            Fixed64.FromFraction(1, 2),
            Fixed64.FromFraction(3, 5),
            target.Collider,
            out Fixed64 distance);

        found.Should().BeTrue();
        distance.Should().BeGreaterThan(Fixed64.Zero);
        distance.Should().BeLessThan((Fixed64)4);
    }

    [Fact]
    public void FiniteSlabProjectionSweep_WithClippedCurvedSupportMatrix_ShouldClassifyEdgeRows()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCapsuleCollider> capsule = CreateBody3D(
            context,
            new LSCapsuleCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            immovable: true,
            rotation: FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)30, Fixed64.Zero, (Fixed64)55));
        ScenarioBody<LSCylinderCollider> cylinder = CreateBody3D(
            context,
            new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true,
            rotation: FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)45));
        ScenarioBody<LSConeCollider> cone = CreateBody3D(
            context,
            new LSConeCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true);

        bool capsuleHit = FiniteSlabProjectionSweep.TrySweepCircleAgainstCapsule(
            new Vector2d((Fixed64)(-7), Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)8,
            Fixed64.Half,
            -Fixed64.Half,
            Fixed64.Half,
            capsule.Collider,
            out Fixed64 capsuleDistance);
        bool cylinderHitFromLeft = FiniteSlabProjectionSweep.TrySweepCircleAgainstCylinder(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)8,
            Fixed64.Half,
            -Fixed64.Half,
            Fixed64.Half,
            cylinder.Collider,
            out Fixed64 cylinderLeftDistance);
        bool cylinderHitFromRight = FiniteSlabProjectionSweep.TrySweepCircleAgainstCylinder(
            new Vector2d((Fixed64)4, Fixed64.Zero),
            -Vector2d.Right,
            (Fixed64)8,
            Fixed64.Half,
            -Fixed64.Half,
            Fixed64.Half,
            cylinder.Collider,
            out Fixed64 cylinderRightDistance);
        bool coneHit = FiniteSlabProjectionSweep.TrySweepCircleAgainstCone(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)8,
            Fixed64.Half,
            -Fixed64.Half,
            Fixed64.Half,
            cone.Collider,
            out Fixed64 coneDistance);
        bool coneMiss = FiniteSlabProjectionSweep.TrySweepCircleAgainstCone(
            new Vector2d((Fixed64)(-4), (Fixed64)4),
            Vector2d.Right,
            (Fixed64)8,
            Fixed64.Half,
            -Fixed64.Half,
            Fixed64.Half,
            cone.Collider,
            out Fixed64 coneMissDistance);

        capsuleHit.Should().BeTrue(nameof(capsuleHit));
        capsuleDistance.Should().BeGreaterThan(Fixed64.Zero);
        cylinderHitFromLeft.Should().BeTrue(nameof(cylinderHitFromLeft));
        cylinderHitFromRight.Should().BeTrue(nameof(cylinderHitFromRight));
        cylinderLeftDistance.Should().BeGreaterThan(Fixed64.Zero);
        cylinderRightDistance.Should().BeGreaterThan(Fixed64.Zero);
        coneHit.Should().BeTrue(nameof(coneHit));
        coneDistance.Should().BeGreaterThan(Fixed64.Zero);
        coneMiss.Should().BeFalse();
        coneMissDistance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void FiniteSlabProjectionSweep_WithBoundarySupportMatrix_ShouldClassifyDegenerateAndClippedRows()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCylinderCollider> verticalCylinder = CreateBody3D(
            context,
            new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true);
        ScenarioBody<LSCylinderCollider> tiltedCylinder = CreateBody3D(
            context,
            new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true,
            rotation: FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)45));
        ScenarioBody<LSCapsuleCollider> horizontalCapsule = CreateBody3D(
            context,
            new LSCapsuleCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true,
            rotation: FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90));
        ScenarioBody<LSConeCollider> verticalCone = CreateBody3D(
            context,
            new LSConeCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true);

        bool cylinderInsideZeroDirection = FiniteSlabProjectionSweep.TrySweepCircleAgainstCylinder(
            Vector2d.Zero,
            Vector2d.Zero,
            (Fixed64)4,
            Fixed64.Half,
            -Fixed64.Half,
            Fixed64.Half,
            verticalCylinder.Collider,
            out Fixed64 cylinderInsideDistance);
        bool cylinderOutsideZeroDirection = FiniteSlabProjectionSweep.TrySweepCircleAgainstCylinder(
            new Vector2d((Fixed64)4, Fixed64.Zero),
            Vector2d.Zero,
            (Fixed64)4,
            Fixed64.Half,
            -Fixed64.Half,
            Fixed64.Half,
            verticalCylinder.Collider,
            out Fixed64 cylinderOutsideDistance);
        bool cylinderOutsideSlab = FiniteSlabProjectionSweep.TrySweepCircleAgainstCylinder(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)8,
            Fixed64.Half,
            (Fixed64)3,
            (Fixed64)4,
            verticalCylinder.Collider,
            out Fixed64 cylinderOutsideSlabDistance);
        bool cylinderTopBand = FiniteSlabProjectionSweep.TrySweepCircleAgainstCylinder(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)8,
            Fixed64.Half,
            Fixed64.FromFraction(7, 5),
            Fixed64.FromFraction(8, 5),
            verticalCylinder.Collider,
            out Fixed64 cylinderTopBandDistance);
        bool tiltedCylinderForward = FiniteSlabProjectionSweep.TrySweepCircleAgainstCylinder(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)8,
            Fixed64.Half,
            -Fixed64.Half,
            Fixed64.Half,
            tiltedCylinder.Collider,
            out Fixed64 tiltedCylinderForwardDistance);
        bool tiltedCylinderReverse = FiniteSlabProjectionSweep.TrySweepCircleAgainstCylinder(
            new Vector2d((Fixed64)4, Fixed64.Zero),
            -Vector2d.Right,
            (Fixed64)8,
            Fixed64.Half,
            -Fixed64.Half,
            Fixed64.Half,
            tiltedCylinder.Collider,
            out Fixed64 tiltedCylinderReverseDistance);
        bool horizontalCapsuleHit = FiniteSlabProjectionSweep.TrySweepCircleAgainstCapsule(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)8,
            Fixed64.Half,
            -Fixed64.Half,
            Fixed64.Half,
            horizontalCapsule.Collider,
            out Fixed64 horizontalCapsuleDistance);
        bool coneInsideZeroDirection = FiniteSlabProjectionSweep.TrySweepCircleAgainstCone(
            Vector2d.Zero,
            Vector2d.Zero,
            (Fixed64)4,
            Fixed64.Half,
            -Fixed64.Half,
            Fixed64.Half,
            verticalCone.Collider,
            out Fixed64 coneInsideDistance);
        bool coneOutsideZeroDirection = FiniteSlabProjectionSweep.TrySweepCircleAgainstCone(
            new Vector2d((Fixed64)4, Fixed64.Zero),
            Vector2d.Zero,
            (Fixed64)4,
            Fixed64.Half,
            -Fixed64.Half,
            Fixed64.Half,
            verticalCone.Collider,
            out Fixed64 coneOutsideDistance);

        cylinderInsideZeroDirection.Should().BeTrue(nameof(cylinderInsideZeroDirection));
        cylinderInsideDistance.Should().Be(Fixed64.Zero);
        cylinderOutsideZeroDirection.Should().BeFalse();
        cylinderOutsideDistance.Should().Be(Fixed64.Zero);
        cylinderOutsideSlab.Should().BeFalse();
        cylinderOutsideSlabDistance.Should().Be(Fixed64.Zero);
        cylinderTopBand.Should().BeTrue(nameof(cylinderTopBand));
        cylinderTopBandDistance.Should().BeGreaterThan(Fixed64.Zero);
        tiltedCylinderForward.Should().BeTrue(nameof(tiltedCylinderForward));
        tiltedCylinderForwardDistance.Should().BeGreaterThan(Fixed64.Zero);
        tiltedCylinderReverse.Should().BeTrue(nameof(tiltedCylinderReverse));
        tiltedCylinderReverseDistance.Should().BeGreaterThan(Fixed64.Zero);
        horizontalCapsuleHit.Should().BeTrue(nameof(horizontalCapsuleHit));
        horizontalCapsuleDistance.Should().BeGreaterThan(Fixed64.Zero);
        coneInsideZeroDirection.Should().BeTrue(nameof(coneInsideZeroDirection));
        coneInsideDistance.Should().Be(Fixed64.Zero);
        coneOutsideZeroDirection.Should().BeFalse();
        coneOutsideDistance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void FiniteSlabProjectionSweep_WithHorizontalCylinderAndBoundaryBands_ShouldClassifySupportRows()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCylinderCollider> horizontalCylinder = CreateBody3D(
            context,
            new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true,
            rotation: FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90));
        ScenarioBody<LSCylinderCollider> tiltedCylinder = CreateBody3D(
            context,
            new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true,
            rotation: FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)20, Fixed64.Zero, (Fixed64)55));
        ScenarioBody<LSCapsuleCollider> tiltedCapsule = CreateBody3D(
            context,
            new LSCapsuleCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            Vector3d.Zero,
            immovable: true,
            rotation: FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)35, Fixed64.Zero, (Fixed64)65));
        ScenarioBody<LSConeCollider> tiltedCone = CreateBody3D(
            context,
            new LSConeCollider { Size = new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)2) },
            Vector3d.Zero,
            immovable: true,
            rotation: FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)35, (Fixed64)35));

        bool horizontalHit = FiniteSlabProjectionSweep.TrySweepCircleAgainstCylinder(
            new Vector2d(Fixed64.Zero, (Fixed64)(-4)),
            Vector2d.Forward,
            (Fixed64)8,
            Fixed64.Half,
            -Fixed64.FromFraction(1, 10),
            Fixed64.FromFraction(1, 10),
            horizontalCylinder.Collider,
            out Fixed64 horizontalDistance);
        bool tiltedLeftHit = FiniteSlabProjectionSweep.TrySweepCircleAgainstCylinder(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)8,
            Fixed64.Half,
            -Fixed64.FromFraction(1, 10),
            Fixed64.FromFraction(1, 10),
            tiltedCylinder.Collider,
            out Fixed64 tiltedLeftDistance);
        bool tiltedRightHit = FiniteSlabProjectionSweep.TrySweepCircleAgainstCylinder(
            new Vector2d((Fixed64)4, Fixed64.Zero),
            -Vector2d.Right,
            (Fixed64)8,
            Fixed64.Half,
            -Fixed64.FromFraction(1, 10),
            Fixed64.FromFraction(1, 10),
            tiltedCylinder.Collider,
            out Fixed64 tiltedRightDistance);
        bool capsuleBoundaryHit = FiniteSlabProjectionSweep.TrySweepCircleAgainstCapsule(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)8,
            Fixed64.Half,
            Fixed64.FromFraction(1, 4),
            Fixed64.FromFraction(3, 10),
            tiltedCapsule.Collider,
            out Fixed64 capsuleBoundaryDistance);
        bool coneTiltedHit = FiniteSlabProjectionSweep.TrySweepCircleAgainstCone(
            new Vector2d((Fixed64)(-4), Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)8,
            Fixed64.Half,
            -Fixed64.FromFraction(1, 5),
            Fixed64.FromFraction(1, 5),
            tiltedCone.Collider,
            out Fixed64 coneTiltedDistance);

        horizontalHit.Should().BeTrue(nameof(horizontalHit));
        horizontalDistance.Should().BeGreaterThan(Fixed64.Zero);
        tiltedLeftHit.Should().BeFalse(nameof(tiltedLeftHit));
        tiltedRightHit.Should().BeFalse(nameof(tiltedRightHit));
        tiltedLeftDistance.Should().Be(Fixed64.Zero);
        tiltedRightDistance.Should().Be(Fixed64.Zero);
        capsuleBoundaryHit.Should().BeFalse(nameof(capsuleBoundaryHit));
        capsuleBoundaryDistance.Should().Be(Fixed64.Zero);
        coneTiltedHit.Should().BeFalse(nameof(coneTiltedHit));
        coneTiltedDistance.Should().Be(Fixed64.Zero);
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
    public void SweepSphereAgainst2D_WithAabbSlabVerticalCornerMiss_ShouldRejectCapAndEdgeMiss()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBodylessBox2D(context, Vector2d.Zero, new Vector2d((Fixed64)2, (Fixed64)2));
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepSphereAgainst2DAll(
            new Vector3d(Fixed64.FromFraction(7, 5), (Fixed64)3, Fixed64.FromFraction(7, 5)),
            new Vector3d(Fixed64.FromFraction(7, 5), (Fixed64)(-3), Fixed64.FromFraction(7, 5)),
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
    public void SweepSphereAgainst2D_WithAabbSlabBottomFace_ShouldReportExactHit()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessBox2D(context, Vector2d.Zero, new Vector2d((Fixed64)2, (Fixed64)2));

        bool mixedHit = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d(Fixed64.Zero, (Fixed64)(-3), Fixed64.Zero),
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(target);
        hit.Distance.Should().Be((Fixed64)2);
        hit.Normal3DTo2D.Should().Be(Vector3d.Up);
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
    public void SweepSphereAgainst2D_WithCapsuleSlabBottomFace_ShouldReportExactHit()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessCapsule2D(context, Vector2d.Zero);

        bool mixedHit = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d(Fixed64.Zero, (Fixed64)(-3), Fixed64.Zero),
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(target);
        hit.Distance.Should().Be((Fixed64)2);
        hit.Normal3DTo2D.Should().Be(Vector3d.Up);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepSphereAgainst2D_WithCapsuleSlabVerticalEndMiss_ShouldRejectCapAndBoundaryMiss()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBodylessCapsule2D(context, Vector2d.Zero);
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepSphereAgainst2DAll(
            new Vector3d(Fixed64.FromFraction(4, 5), (Fixed64)3, (Fixed64)2),
            new Vector3d(Fixed64.FromFraction(4, 5), (Fixed64)(-3), (Fixed64)2),
            Fixed64.Half,
            IncludeLayerZero,
            hits);

        count.Should().Be(0);
        context.QueryMixed.LastQueryCandidateCount.Should().Be(1);
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void SweepSphereAgainst2D_WithAabbCapPlaneJustBeyondLength_ShouldRejectBroadCandidate()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBodylessBox2D(context, Vector2d.Zero, new Vector2d((Fixed64)2, (Fixed64)2));
        Vector3d start = new(Fixed64.FromFraction(7, 5), (Fixed64)3, Fixed64.FromFraction(7, 5));
        Vector3d end = new(Fixed64.FromFraction(73, 50), Fixed64.One, Fixed64.FromFraction(7, 5));
        Vector3d segment = end - start;
        Fixed64 length = segment.Magnitude;
        Fixed64 capDistance = (Fixed64.One - start.Y) / (segment.Y / length);

        bool mixedHit = context.QueryMixed.SweepSphereAgainst2D(
            start,
            end,
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        capDistance.Should().BeGreaterThan(length);
        mixedHit.Should().BeFalse();
        hit.Should().Be(default(PhysicsMixedHit));
        context.QueryMixed.LastQueryCandidateCount.Should().Be(1);
    }

    [Fact]
    public void SweepSphereAgainst2D_WithPolygonSideFaceBeyondLength_ShouldRejectBroadCandidate()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBodylessPolygon2D(context, Vector2d.Zero, CreateDiamondVertices());

        bool mixedHit = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.FromFraction(7, 5)),
            new Vector3d(Fixed64.FromFraction(-7, 5), Fixed64.Zero, Fixed64.FromFraction(7, 5)),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeFalse();
        hit.Should().Be(default(PhysicsMixedHit));
        context.QueryMixed.LastQueryCandidateCount.Should().Be(1);
    }

    [Fact]
    public void SweepSphereAgainst2D_WithAabbSideFaceBelowSlab_ShouldRejectBroadCandidate()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBodylessBox2D(context, Vector2d.Zero, new Vector2d((Fixed64)2, (Fixed64)2));
        Fixed64 belowSlab = Fixed64.FromFraction(-51, 100);

        bool mixedHit = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d((Fixed64)(-3), belowSlab, Fixed64.Zero),
            new Vector3d(Fixed64.FromFraction(-3, 2), belowSlab, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeFalse();
        hit.Should().Be(default(PhysicsMixedHit));
        context.QueryMixed.LastQueryCandidateCount.Should().Be(1);
    }

    [Fact]
    public void SweepSphereAgainst2D_WithCapsuleCapPlaneJustBeyondLength_ShouldRejectBroadCandidate()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBodylessCapsule2D(context, Vector2d.Zero);
        Vector3d start = new(Fixed64.FromFraction(4, 5), (Fixed64)3, (Fixed64)2);
        Vector3d end = new(Fixed64.FromFraction(43, 50), Fixed64.One, (Fixed64)2);
        Vector3d segment = end - start;
        Fixed64 length = segment.Magnitude;
        Fixed64 capDistance = (Fixed64.One - start.Y) / (segment.Y / length);

        bool mixedHit = context.QueryMixed.SweepSphereAgainst2D(
            start,
            end,
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        capDistance.Should().BeGreaterThan(length);
        mixedHit.Should().BeFalse();
        hit.Should().Be(default(PhysicsMixedHit));
        context.QueryMixed.LastQueryCandidateCount.Should().Be(1);
    }

    [Fact]
    public void SweepSphereAgainst2D_WithCapsuleEndpointJustBeyondLength_ShouldRejectBroadCandidate()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBodylessCapsule2D(context, Vector2d.Zero);

        bool mixedHit = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d(Fixed64.FromFraction(-6, 5), Fixed64.Zero, Fixed64.FromFraction(11, 5)),
            new Vector3d(Fixed64.FromFraction(-19, 20), Fixed64.Zero, Fixed64.FromFraction(39, 20)),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeFalse();
        hit.Should().Be(default(PhysicsMixedHit));
        context.QueryMixed.LastQueryCandidateCount.Should().Be(1);
    }

    [Fact]
    public void SweepSphereAgainst2D_WithCapsuleSideRootsOutsideBothSlabFaces_ShouldRejectBroadCandidates()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBodylessCapsule2D(context, Vector2d.Zero);
        Fixed64 belowSlab = Fixed64.FromFraction(-51, 100);
        Fixed64 aboveSlab = Fixed64.FromFraction(51, 100);

        bool belowHit = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d((Fixed64)(-3), belowSlab, Fixed64.Zero),
            new Vector3d(-Fixed64.One, belowSlab, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit belowResult);

        belowHit.Should().BeFalse();
        belowResult.Should().Be(default(PhysicsMixedHit));
        context.QueryMixed.LastQueryCandidateCount.Should().Be(1);

        bool aboveHit = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d((Fixed64)(-3), aboveSlab, Fixed64.Zero),
            new Vector3d(-Fixed64.One, aboveSlab, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit aboveResult);

        aboveHit.Should().BeFalse();
        aboveResult.Should().Be(default(PhysicsMixedHit));
        context.QueryMixed.LastQueryCandidateCount.Should().Be(1);
    }

    [Fact]
    public void SweepSphereAgainst2D_WithCircleSupportCandidatesOutsideFiniteSlab_ShouldRejectBroadCandidates()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBodylessCircle2D(context, Vector2d.Zero);

        bool shortCapHit = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d(Fixed64.FromFraction(4, 5), Fixed64.Zero, Fixed64.FromFraction(4, 5)),
            new Vector3d(Fixed64.FromFraction(4, 5), Fixed64.FromFraction(99, 100), Fixed64.FromFraction(4, 5)),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit shortCapResult);

        shortCapHit.Should().BeFalse();
        shortCapResult.Should().Be(default(PhysicsMixedHit));
        context.QueryMixed.LastQueryCandidateCount.Should().Be(1);

        bool sideMiss = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero),
            new Vector3d((Fixed64)2, Fixed64.FromFraction(1, 5), Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit sideResult);

        sideMiss.Should().BeFalse();
        sideResult.Should().Be(default(PhysicsMixedHit));
        context.QueryMixed.LastQueryCandidateCount.Should().Be(1);

        bool mirroredSideMiss = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d(Fixed64.Zero, (Fixed64)(-2), Fixed64.Zero),
            new Vector3d((Fixed64)2, Fixed64.FromFraction(-1, 5), Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit mirroredSideResult);

        mirroredSideMiss.Should().BeFalse();
        mirroredSideResult.Should().Be(default(PhysicsMixedHit));
        context.QueryMixed.LastQueryCandidateCount.Should().Be(1);
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
    public void SweepSphereAgainst2D_WithCompoundPartsOutsidePath_ShouldRejectExactMiss()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBodylessCompound2D(
            context,
            Vector2d.Zero,
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.Zero, (Fixed64)2)),
            CompoundColliderPart2D.AABBox(Vector2d.One, new Vector2d(Fixed64.Zero, (Fixed64)(-2))));
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepSphereAgainst2DAll(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            hits);

        count.Should().Be(0);
        context.QueryMixed.LastQueryCandidateCount.Should().Be(1);
        hits.Count.Should().Be(0);
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
    public void SweepCircleAgainst3D_WithCompoundSpherePartOutsideFiniteSlab_ShouldRejectExactPart()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBody3D(
            context,
            new LSCompoundCollider(
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero)),
                CompoundColliderPart.Cuboid(Vector3d.One, new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)2))),
            Vector3d.Zero,
            immovable: true);
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.FromFraction(1, 4),
            IncludeLayerZero,
            hits);

        count.Should().Be(0);
        context.QueryMixed.LastQueryCandidateCount.Should().Be(1);
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithCompoundCuboidAndVerticalCylinderOutsideSlab_ShouldRejectParts()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBody3D(
            context,
            new LSCompoundCollider(
                CompoundColliderPart.Cuboid(
                    Vector3d.One,
                    new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero)),
                CompoundColliderPart.Cylinder(
                    Fixed64.Half,
                    Fixed64.One,
                    new Vector3d(Fixed64.Zero, (Fixed64)(-2), Fixed64.Zero)),
                CompoundColliderPart.Cylinder(
                    Fixed64.Half,
                    Fixed64.One,
                    new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero))),
            Vector3d.Zero,
            immovable: true);
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.FromFraction(1, 4),
            IncludeLayerZero,
            hits);

        count.Should().Be(0);
        context.QueryMixed.LastQueryCandidateCount.Should().Be(1);
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithCompoundSpherePartBehindAndMovingAway_ShouldRejectExactPart()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBody3D(
            context,
            new LSCompoundCollider(
                CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero),
                CompoundColliderPart.Cuboid(Vector3d.One, new Vector3d((Fixed64)3, Fixed64.Zero, (Fixed64)2))),
            Vector3d.Zero,
            immovable: true);
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)2, Fixed64.Zero),
            new Vector2d((Fixed64)5, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            hits);

        count.Should().Be(0);
        context.QueryMixed.LastQueryCandidateCount.Should().Be(1);
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
    public void SweepCircleAgainst3D_WithRotatedMeshTriangleBelowSlab_ShouldRejectConservativeLocalCandidate()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var meshCollider = new LSMeshCollider(
            new[]
            {
                new Vector3d(-Fixed64.FromFraction(1, 4), (Fixed64)(-2), -Fixed64.FromFraction(1, 4)),
                new Vector3d(Fixed64.FromFraction(1, 4), (Fixed64)(-2), -Fixed64.FromFraction(1, 4)),
                new Vector3d(Fixed64.Zero, (Fixed64)(-2), Fixed64.FromFraction(1, 4)),
                new Vector3d(-Fixed64.FromFraction(1, 4), Fixed64.Zero, (Fixed64)2),
                new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.Zero, (Fixed64)2),
                new Vector3d(Fixed64.Zero, Fixed64.FromFraction(1, 4), (Fixed64)2)
            },
            new[] { 0, 1, 2, 3, 4, 5 },
            MeshColliderMode.Convex,
            MeshInertiaPolicy.SurfaceApproximation);
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            Fixed64.Zero,
            (Fixed64)45);
        _ = CreateBody3D(
            context,
            meshCollider,
            Vector3d.Zero,
            immovable: true,
            rotation: rotation);
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            hits);

        count.Should().Be(0);
        hits.Count.Should().Be(0);
        context.QueryMixed.LastQueryCandidateCount.Should().Be(1);
        context.QueryMixed.LastMeshTriangleCandidateCount.Should().Be(1);
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
    public void SweepCircleAgainst3D_WithMeshPointAtSweepCenter_ShouldUseOppositeDirectionFallback()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSMeshCollider> mesh = CreateMesh3D(
            context,
            CreatePointProjectionTriangle(),
            Vector3d.Zero,
            immovable: true);

        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            Vector2d.Zero,
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.One,
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(mesh.Collider);
        hit.Body3D.Should().BeSameAs(mesh.Body);
        hit.Collider2D.Should().BeNull();
        hit.Distance.Should().Be(Fixed64.Zero);
        hit.Point3D.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));
        hit.Point2D.Should().Be(new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.Zero));
        hit.Normal3DTo2D.Should().Be(-Vector3d.Right);
        hit.Direction3D.Should().Be(Vector3d.Right);
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
    public void SweepCircleAgainst3D_WithStartingOverlapInsideMeshSegmentProjection_ShouldReturnStableExactHit()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSMeshCollider> mesh = CreateMesh3D(
            context,
            CreateSegmentProjectionTriangle(),
            Vector3d.Zero,
            immovable: true);

        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d(Fixed64.Zero, Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Half,
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(mesh.Collider);
        hit.Distance.Should().Be(Fixed64.Zero);
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
    public void SweepCircleAgainst3D_WithCompoundPartsOutsidePath_ShouldRejectExactMiss()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBody3D(
            context,
            new LSCompoundCollider(
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)2)),
                CompoundColliderPart.Cuboid(Vector3d.One, new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)(-2)))),
            Vector3d.Zero,
            immovable: true);
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            hits);

        count.Should().Be(0);
        context.QueryMixed.LastQueryCandidateCount.Should().Be(1);
        hits.Count.Should().Be(0);
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
    public void MixedQueryDiagnostics_WithUnsupported3DTarget_ShouldRecordRejectedConservativeCandidate()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBodyless3D(context, new UnsupportedTestCollider3D(), Vector3d.Zero);
        var hits = new SwiftList<PhysicsMixedHit>();
        context.Diagnostics.Enable(eventCapacity: 4, drawCommandCapacity: 0);

        int count = context.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            hits);

        count.Should().Be(0);
        context.QueryMixed.LastQueryCandidateCount.Should().Be(1);
        hits.Count.Should().Be(0);

        GravitasDiagnosticEvent summaryEvent = FindDiagnosticEvent(context.Diagnostics.Events, GravitasDiagnosticEventKind.QuerySummary);
        summaryEvent.TryAsQuerySummary(out GravitasQuerySummaryDiagnosticView summary).Should().BeTrue();
        summary.SourceDimension.Should().Be(GravitasColliderDimension.TwoD);
        summary.TargetDimension.Should().Be(GravitasColliderDimension.ThreeD);
        summary.ExactReducerAttempts.Should().Be(0);
        summary.AcceptedHits.Should().Be(0);
        summary.FallbackHits.Should().Be(0);
        summary.RejectedConservativeCandidates.Should().Be(1);
        summary.HasConservativeFallback.Should().BeTrue();
    }

    [Fact]
    public void MixedQueryDiagnostics_WithUnsupported2DTarget_ShouldRecordAcceptedConservativeFallback()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessUnsupported2D(context, Vector2d.Zero);
        var hits = new SwiftList<PhysicsMixedHit>();
        context.Diagnostics.Enable(eventCapacity: 4, drawCommandCapacity: 0);

        int count = context.QueryMixed.SweepSphereAgainst2DAll(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            hits);

        count.Should().Be(1);
        hits.Count.Should().Be(1);
        hits[0].Collider2D.Should().BeSameAs(target);
        hits[0].ReducerKind.Should().Be(PhysicsQueryReducerKind.ConservativeFallback);

        GravitasDiagnosticEvent summaryEvent = FindDiagnosticEvent(context.Diagnostics.Events, GravitasDiagnosticEventKind.QuerySummary);
        summaryEvent.TryAsQuerySummary(out GravitasQuerySummaryDiagnosticView summary).Should().BeTrue();
        summary.SourceDimension.Should().Be(GravitasColliderDimension.ThreeD);
        summary.TargetDimension.Should().Be(GravitasColliderDimension.TwoD);
        summary.ExactReducerAttempts.Should().Be(0);
        summary.AcceptedHits.Should().Be(1);
        summary.FallbackHits.Should().Be(1);
        summary.RejectedConservativeCandidates.Should().Be(0);
        summary.HasConservativeFallback.Should().BeTrue();
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
    public void LateSimulate_WithMixed2DContinuousCollision_ShouldClampAgainstBodyless3DTarget()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        _ = CreateBodyless3D(context, new LSSphereCollider(), Vector3d.Zero);
        SolidBody2D moving2D = CreateCircle2D(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));

        moving2D.AddForce(Vector2d.Right * (Fixed64)10);
        context.Simulate();
        context.LateSimulate();

        moving2D.Position.X.Should().Be(-Fixed64.One);
        moving2D.LinearVelocity.Should().Be(Vector2d.Zero);
        moving2D.LastContinuousCollisionToiIterationCount.Should().Be(1);
    }

    [Fact]
    public void LateSimulate_WithMixed2DContinuousCollision_ShouldClampAgainstKinematic3DTarget()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        _ = CreateSphere3D(context, Vector3d.Zero, isKinematic: true);
        SolidBody2D moving2D = CreateCircle2D(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));

        moving2D.AddForce(Vector2d.Right * (Fixed64)10);
        context.Simulate();
        context.LateSimulate();

        moving2D.Position.X.Should().Be(-Fixed64.One);
        moving2D.LinearVelocity.Should().Be(Vector2d.Zero);
        moving2D.LastContinuousCollisionToiIterationCount.Should().Be(1);
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
    public void LateSimulate_WithMixed3DSourceAndSleeping2DTarget_ShouldWakeAndApplyDynamicHandoff()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        SolidBody2D target = CreateCircle2D(context, Vector2d.Zero);
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero));
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Sleep();

        source.Body.AddForce(Vector3d.Right * (Fixed64)10);
        context.LateSimulate();

        target.IsSleeping.Should().BeFalse();
        target.Position.X.Should().BeGreaterThan(Fixed64.Zero);
        target.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        source.Body.Position3d.X.Should().BeLessThan(target.Position.X);
    }

    [Fact]
    public void LateSimulate_WithMixed3DSourceAndStaleDynamic2DCandidate_ShouldReachRequestedEndPose()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.MaxSpeed = (Fixed64)16;
        ScenarioBody<LSSphereCollider> deactivator = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)3));
        SolidBody2D target = CreateCircle2D(context, Vector2d.Zero);
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero));
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        deactivator.Body.OnMoved += target.Deactivate;

        deactivator.Body.AddForce(Vector3d.Right);
        source.Body.AddForce(Vector3d.Right * (Fixed64)10);
        context.LateSimulate();

        deactivator.Body.Position3d.Should().Be(new Vector3d(Fixed64.One, Fixed64.Zero, (Fixed64)3));
        target.Active.Should().BeFalse();
        target.Position.Should().Be(Vector2d.Zero);
        target.LinearVelocity.Should().Be(Vector2d.Zero);
        source.Body.Position3d.Should().Be(new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero));
        source.Body.LinearVelocity.Should().Be(Vector3d.Right * (Fixed64)10);
    }

    [Fact]
    public void LateSimulate_WithMixed3DSourceAndTwo2DTargets_ShouldHandoffOnlyToNearestTarget()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        SolidBody2D nearest = CreateCircle2D(context, Vector2d.Zero);
        SolidBody2D farther = CreateCircle2D(context, new Vector2d((Fixed64)3, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero));
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        nearest.Mass = (Fixed64)100;
        nearest.Sleep();
        farther.Sleep();

        source.Body.AddForce(Vector3d.Right * (Fixed64)10);
        context.LateSimulate();

        nearest.IsSleeping.Should().BeFalse();
        nearest.Position.X.Should().BeGreaterThan(Fixed64.Zero);
        nearest.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        farther.IsSleeping.Should().BeTrue();
        farther.Position.Should().Be(new Vector2d((Fixed64)3, Fixed64.Zero));
        farther.LinearVelocity.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void LateSimulate_WithMixed2DSourceAndSleeping3DTarget_ShouldWakeAndApplyDynamicHandoff()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D source = CreateCircle2D(context, new Vector2d((Fixed64)(-5), Fixed64.Zero));
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.Sleep();

        source.AddForce(Vector2d.Right * (Fixed64)10);
        context.LateSimulate();

        target.Body.IsSleeping.Should().BeFalse();
        target.Body.Position3d.X.Should().BeGreaterThan(Fixed64.Zero);
        target.Body.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        source.Position.X.Should().BeLessThan(target.Body.Position3d.X);
    }

    [Fact]
    public void LateSimulate_WithVerticalMixed3DDynamicHit_ShouldNotInventPlanar2DHandoff()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        SolidBody2D target = CreateCircle2D(context, Vector2d.Zero);
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero));
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Sleep();

        source.Body.AddForce(Vector3d.Down * (Fixed64)10);
        context.LateSimulate();

        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        source.Body.LinearVelocity.Y.Should().Be(Fixed64.Zero);
        target.Position.Should().Be(Vector2d.Zero);
        target.LinearVelocity.Should().Be(Vector2d.Zero);
        target.IsSleeping.Should().BeTrue();
    }

    [Fact]
    public void LateSimulate_WithDynamic2DSourceAndVerticalMixedHit_ShouldRejectZeroPlanarNormalDeterministically()
    {
        var first = RunVerticalMixedDynamic2DSourceScenario();
        var second = RunVerticalMixedDynamic2DSourceScenario();

        second.Should().Be(first);
        first.SourcePosition.Should().Be(new Vector2d(-Fixed64.Half, Fixed64.Zero));
        first.SourceVelocity.Should().Be(Vector2d.Right * (Fixed64)3);
        first.TargetPosition.Should().Be(new Vector3d(-Fixed64.Half, Fixed64.FromFraction(-3, 2), Fixed64.Zero));
        first.TargetVelocity.Should().Be(Vector3d.Down * (Fixed64)5);
        first.SourceToiIterations.Should().Be(2);
    }

    [Fact]
    public void LateSimulate_WithDynamic2DSourceAndVerticallyDominatedMixedHit_ShouldNotInventPlanarImpulse()
    {
        var first = RunPlanarSeparatingMixed2DSourceScenario(isKinematic: false);
        var second = RunPlanarSeparatingMixed2DSourceScenario(isKinematic: false);

        second.Should().Be(first);
        first.SourcePosition.X.Should().BeGreaterThan((Fixed64)(-2));
        first.SourcePosition.X.Should().BeLessThan(Fixed64.Zero);
        first.SourceVelocity.Should().Be(Vector2d.Right * (Fixed64)2);
        first.TargetPosition.Should().Be(new Vector3d(-Fixed64.Half, Fixed64.FromFraction(-13, 10), Fixed64.Zero));
        first.TargetVelocity.Should().Be(new Vector3d(Fixed64.One, Fixed64.FromFraction(-24, 5), Fixed64.Zero));
        first.SourceToiIterations.Should().Be(2);
    }

    [Fact]
    public void LateSimulate_WithHugeMassDynamic2DAnd3DPair_ShouldResolveFiniteResponseMass()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        Fixed64 hugeMass = (Fixed64)33_554_432;
        SolidBody2D source = CreateCircle2D(context, new Vector2d((Fixed64)(-2), Fixed64.Zero));
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(context, Vector3d.Zero);
        source.Mass = hugeMass;
        target.Body.Mass = hugeMass;
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.Sleep();

        source.InverseMass.Should().BeGreaterThan(Fixed64.Zero);
        (source.InverseMass + target.Body.InverseMass).Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        source.ApplyCollisionLinearVelocityDelta(Vector2d.Right * (Fixed64)2);
        context.LateSimulate();

        source.Position.Should().Be(-Vector2d.Right * Fixed64.Half);
        source.LinearVelocity.Should().Be(Vector2d.Right);
        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
        source.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
        target.Body.Position3d.Should().Be(Vector3d.Right * Fixed64.Half);
        target.Body.LinearVelocity.Should().Be(Vector3d.Right);
    }

    [Fact]
    public void LateSimulate_WithNearSingularFrozen2DSourceMobility_ShouldRejectResponseBeforeDivision()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        Fixed64 smallOffset = Fixed64.Epsilon;
        Vector2d sourceStart = new(-Fixed64.One + Fixed64.Epsilon, smallOffset);
        SolidBody2D source = CreateCircle2D(context, sourceStart);
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(context, Vector3d.Zero);
        source.FreezeAxes = BodyFreezeAxes2D.PositionX;
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.ApplyCollisionLinearVelocityDelta(Vector2d.Forward * Fixed64.Half);
        target.Body.ApplyCollisionLinearVelocityDelta(-Vector3d.Right * (Fixed64)2);

        context.LateSimulate();

        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
        source.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
        source.LinearVelocity.X.Should().Be(Fixed64.Zero);
        source.LinearVelocity.Y.Should().BeGreaterThan(Fixed64.Zero);
        source.LinearVelocity.Y.Should().BeLessThan(Fixed64.Half);
        // The 3D phase has already advanced the target; rejection must not queue a second handoff.
        target.Body.Position3d.Should().Be(new Vector3d(
            Fixed64.FromRaw(-512_673_560),
            Fixed64.Zero,
            Fixed64.FromRaw(-254)));
        target.Body.LinearVelocity.Should().Be(new Vector3d(
            Fixed64.FromRaw(45),
            Fixed64.Zero,
            Fixed64.FromRaw(330_382_150)));
    }

    [Fact]
    public void LateSimulate_WithNearSingularFrozen2DTargetMobility_ShouldUse3DSourceFallback()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        Fixed64 smallOffset = Fixed64.FromFraction(1, 4096);
        SolidBody2D target = CreateCircle2D(context, new Vector2d(Fixed64.Zero, smallOffset));
        target.FreezeAxes = BodyFreezeAxes2D.PositionX;
        target.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Sleep();
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero));
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.Body.AddForce(Vector3d.Right * (Fixed64)10);
        context.LateSimulate();

        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        source.Body.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
        target.Position.Should().Be(new Vector2d(Fixed64.Zero, smallOffset));
        target.LinearVelocity.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void LateSimulate_WithNearSingularFrozen3DSourceMobility_ShouldStopZeroTimeIteration()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        Fixed64 smallOffset = Fixed64.FromFraction(1, 65536);
        Vector3d sourceStart = new(-Fixed64.One, Fixed64.Zero, smallOffset);
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(context, sourceStart);
        SolidBody2D target = CreateCircle2D(context, Vector2d.Zero);
        source.Body.FreezeAxes = BodyFreezeAxes3D.PositionX;
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Forward * Fixed64.Half);
        target.ApplyCollisionLinearVelocityDelta(-Vector2d.Right * (Fixed64)2);

        context.LateSimulate();

        source.Body.Position3d.Should().Be(sourceStart);
        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        source.Body.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
    }

    [Fact]
    public void LateSimulate_WithKinematic2DSourceAndVerticallyDominatedMixedHit_ShouldRejectPlanarSeparation()
    {
        var first = RunPlanarSeparatingMixed2DSourceScenario(isKinematic: true);
        var second = RunPlanarSeparatingMixed2DSourceScenario(isKinematic: true);

        second.Should().Be(first);
        first.SourcePosition.Should().Be(Vector2d.Zero);
        first.SourceVelocity.Should().Be(Vector2d.Zero);
        first.TargetPosition.Should().Be(new Vector3d(-Fixed64.Half, Fixed64.FromFraction(-13, 10), Fixed64.Zero));
        first.TargetVelocity.Should().Be(new Vector3d(Fixed64.One, Fixed64.FromFraction(-24, 5), Fixed64.Zero));
        first.SourceToiIterations.Should().Be(0);
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
        Fixed64 mixedContactSpan = driver.Collider.ScaledRadius + ((LSCircleCollider2D)middle.Collider).ScaledRadius;
        Fixed64 maxResidualPenetration = Fixed64.FromFraction(1, 10);
        (receiver.Body.Position3d.X - middle.Position.X).Should().BeGreaterThan(mixedContactSpan - maxResidualPenetration);
        (middle.Position.X - driver.Body.Position3d.X).Should().BeGreaterThan(mixedContactSpan - maxResidualPenetration);
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
    public void LateSimulate_WithKinematic3DSourceAndStaleDynamic2DCandidate_ShouldReachHostTargetPose()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> deactivator = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)3));
        SolidBody2D target = CreateCircle2D(context, Vector2d.Zero);
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        deactivator.Body.OnMoved += target.Deactivate;
        var hostTarget = new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero);

        deactivator.Body.AddForce(Vector3d.Right);
        source.Body.Agent.Transform.Position = hostTarget;
        context.LateSimulate();

        deactivator.Body.Position3d.Should().Be(new Vector3d(Fixed64.One, Fixed64.Zero, (Fixed64)3));
        target.Active.Should().BeFalse();
        target.Position.Should().Be(Vector2d.Zero);
        target.LinearVelocity.Should().Be(Vector2d.Zero);
        source.Body.Position3d.Should().Be(hostTarget);
        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Fact]
    public void LateSimulate_WithKinematic3DSourceAndNewlyFilteredDynamic2DCandidate_ShouldReachHostTargetPose()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> deactivator = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)3));
        SolidBody2D target = CreateCircle2D(context, Vector2d.Zero);
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        deactivator.Body.OnMoved += () =>
            target.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(source.Collider.Layer);
        Vector3d hostTarget = Vector3d.Right * (Fixed64)5;

        deactivator.Body.AddForce(Vector3d.Right);
        source.Body.Agent.Transform.Position = hostTarget;
        context.LateSimulate();

        target.Active.Should().BeTrue();
        target.Collider.IgnoredCollisionLayers.Includes(source.Collider.Layer).Should().BeTrue();
        target.Position.Should().Be(Vector2d.Zero);
        target.LinearVelocity.Should().Be(Vector2d.Zero);
        source.Body.Position3d.Should().Be(hostTarget);
        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Fact]
    public void LateSimulate_WithKinematic3DSourceAndBroadCorner2DCandidate_ShouldRejectRelativeSphereMiss()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        Vector3d sourceStart = new((Fixed64)(-5), Fixed64.Zero, (Fixed64)(-5));
        Vector3d hostTarget = new((Fixed64)5, Fixed64.Zero, (Fixed64)5);
        SolidBody2D target = CreateCircle2D(context, new Vector2d(Fixed64.Zero, (Fixed64)2));
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(context, sourceStart, isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Sleep();

        var sourceBounds = DynamicCcdCandidateIndex.CreateSweptSphereBounds(
            sourceStart,
            hostTarget - sourceStart,
            source.Body.ResolveContinuousCollisionProxyRadius());
        Fixed64 targetRadius = FixedMath.Max(
            target.ResolveContinuousCollisionProxyRadius(),
            target.Collider.MixedHalfThickness);
        var targetBounds = DynamicCcdCandidateIndex.CreateSweptSphereBounds(
            new Vector3d(target.Position.X, target.Collider.MixedSlabCenterY, target.Position.Y),
            Vector3d.Zero,
            targetRadius);
        bool broadBoundsOverlap = !(sourceBounds.Min.X > targetBounds.Max.X
            || sourceBounds.Max.X < targetBounds.Min.X
            || sourceBounds.Min.Y > targetBounds.Max.Y
            || sourceBounds.Max.Y < targetBounds.Min.Y
            || sourceBounds.Min.Z > targetBounds.Max.Z
            || sourceBounds.Max.Z < targetBounds.Min.Z);
        broadBoundsOverlap.Should().BeTrue();

        source.Body.Agent.Transform.Position = hostTarget;
        context.LateSimulate();

        source.Body.Position3d.Should().Be(hostTarget);
        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(0);
        target.Position.Should().Be(new Vector2d(Fixed64.Zero, (Fixed64)2));
        target.LinearVelocity.Should().Be(Vector2d.Zero);
        target.IsSleeping.Should().BeTrue();
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
    public void LateSimulate_WithKinematic3DSourceAndNearSingularFrozen2DTargetMobility_ShouldNotPushTarget()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        Fixed64 smallOffset = Fixed64.FromFraction(1, 65536);
        SolidBody2D target = CreateCircle2D(context, new Vector2d(Fixed64.Zero, smallOffset));
        target.FreezeAxes = BodyFreezeAxes2D.PositionX;
        target.Sleep();
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector3d hostTarget = Vector3d.Right * (Fixed64)5;

        source.Body.Agent.Transform.Position = hostTarget;
        context.LateSimulate();

        source.Body.Position3d.Should().Be(hostTarget);
        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(0);
        target.Position.Should().Be(new Vector2d(Fixed64.Zero, smallOffset));
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
    public void LateSimulate_WithKinematic3DSource_ShouldClampToCloserStatic3DBeforeStatic2DSlab()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        _ = CreateSphere3D(context, Vector3d.Zero, immovable: true);
        _ = CreateCircle2D(context, new Vector2d((Fixed64)3, Fixed64.Zero), immovable: true);
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
    }

    [Fact]
    public void LateSimulate_WithKinematic2DSource_ShouldClampToCloserStatic3DBeforeStatic2D()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        _ = CreateSphere3D(context, Vector3d.Zero, immovable: true);
        _ = CreateCircle2D(context, new Vector2d((Fixed64)3, Fixed64.Zero), immovable: true);
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
    }

    [Fact]
    public void LateSimulate_WithKinematic2DSourceAndStaleDynamic2DCandidate_ShouldReachHostTargetPose()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Both;
        context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> deactivator = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)3));
        SolidBody2D target = CreateCircle2D(context, Vector2d.Zero);
        SolidBody2D source = CreateCircle2D(
            context,
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        deactivator.Body.OnMoved += target.Deactivate;

        deactivator.Body.AddForce(Vector3d.Right);
        source.Agent.Transform.Position = new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        context.LateSimulate();

        deactivator.Body.Position3d.Should().Be(new Vector3d(Fixed64.One, Fixed64.Zero, (Fixed64)3));
        target.Active.Should().BeFalse();
        target.Position.Should().Be(Vector2d.Zero);
        target.LinearVelocity.Should().Be(Vector2d.Zero);
        source.Position.Should().Be(Vector2d.Right * (Fixed64)5);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        source.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
    }

    [Fact]
    public void LateSimulate_WithKinematic2DSourceAndNewlyFilteredDynamic2DCandidate_ShouldReachHostTargetPose()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Both;
        context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> deactivator = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)3));
        SolidBody2D target = CreateCircle2D(context, Vector2d.Zero);
        SolidBody2D source = CreateCircle2D(
            context,
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        deactivator.Body.OnMoved += () =>
            target.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(source.Collider.Layer);

        deactivator.Body.AddForce(Vector3d.Right);
        source.Agent.Transform.Position = new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        context.LateSimulate();

        deactivator.Body.Position3d.Should().Be(new Vector3d(Fixed64.One, Fixed64.Zero, (Fixed64)3));
        target.Active.Should().BeTrue();
        target.Collider.IgnoredCollisionLayers.Includes(source.Collider.Layer).Should().BeTrue();
        target.Position.Should().Be(Vector2d.Zero);
        target.LinearVelocity.Should().Be(Vector2d.Zero);
        source.Position.Should().Be(Vector2d.Right * (Fixed64)5);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        source.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
    }

    [Fact]
    public void LateSimulate_WithKinematic2DSourceAndStaleDynamic3DCandidate_ShouldReachHostTargetPose()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> deactivator = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)3));
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D source = CreateCircle2D(
            context,
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        deactivator.Body.OnMoved += target.Body.Deactivate;

        deactivator.Body.AddForce(Vector3d.Right);
        source.Agent.Transform.Position = new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        context.LateSimulate();

        deactivator.Body.Position3d.Should().Be(new Vector3d(Fixed64.One, Fixed64.Zero, (Fixed64)3));
        target.Body.Active.Should().BeFalse();
        target.Body.Position3d.Should().Be(Vector3d.Zero);
        target.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        source.Position.Should().Be(Vector2d.Right * (Fixed64)5);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        source.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
    }

    [Fact]
    public void LateSimulate_WithKinematic2DSourceAndNewlyFilteredDynamic3DCandidate_ShouldReachHostTargetPose()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> deactivator = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)3));
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D source = CreateCircle2D(
            context,
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        deactivator.Body.OnMoved += () =>
            target.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(source.Collider.Layer);

        deactivator.Body.AddForce(Vector3d.Right);
        source.Agent.Transform.Position = new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        context.LateSimulate();

        deactivator.Body.Position3d.Should().Be(new Vector3d(Fixed64.One, Fixed64.Zero, (Fixed64)3));
        target.Body.Active.Should().BeTrue();
        target.Collider.IgnoredCollisionLayers.Includes(source.Collider.Layer).Should().BeTrue();
        target.Body.Position3d.Should().Be(Vector3d.Zero);
        target.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        source.Position.Should().Be(Vector2d.Right * (Fixed64)5);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        source.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
    }

    [Fact]
    public void LateSimulate_WithKinematic2DSourceAndBroadCorner3DCandidate_ShouldRejectRelativeSphereMiss()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        Vector2d sourceStart = new((Fixed64)(-5), (Fixed64)(-5));
        Vector2d hostTarget = new((Fixed64)5, (Fixed64)5);
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)2));
        SolidBody2D source = CreateCircle2D(context, sourceStart, isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.Sleep();

        Fixed64 sourceRadius = FixedMath.Max(
            source.ResolveContinuousCollisionProxyRadius(),
            source.Collider.MixedHalfThickness);
        FixedBoundVolume sourceBounds = DynamicCcdCandidateIndex.CreateSweptSphereBounds(
            new Vector3d(sourceStart.X, source.Collider.MixedSlabCenterY, sourceStart.Y),
            new Vector3d(hostTarget.X - sourceStart.X, Fixed64.Zero, hostTarget.Y - sourceStart.Y),
            sourceRadius);
        FixedBoundVolume targetBounds = DynamicCcdCandidateIndex.CreateSweptSphereBounds(
            target.Body.Position3d,
            Vector3d.Zero,
            target.Body.ResolveContinuousCollisionProxyRadius());
        bool broadBoundsOverlap = !(sourceBounds.Min.X > targetBounds.Max.X
            || sourceBounds.Max.X < targetBounds.Min.X
            || sourceBounds.Min.Y > targetBounds.Max.Y
            || sourceBounds.Max.Y < targetBounds.Min.Y
            || sourceBounds.Min.Z > targetBounds.Max.Z
            || sourceBounds.Max.Z < targetBounds.Min.Z);
        broadBoundsOverlap.Should().BeTrue();

        source.Agent.Transform.Position = new Vector3d(hostTarget.X, Fixed64.Zero, hostTarget.Y);
        context.LateSimulate();

        source.Position.Should().Be(hostTarget);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        source.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
        target.Body.Position3d.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)2));
        target.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        target.Body.IsSleeping.Should().BeTrue();
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
    public void LateSimulate_WithBothRuntimeMode_ShouldNotTransferKinematic3DSourceIntoDynamic2DTarget()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Both;
        context.Environment.Gravity = Fixed64.Zero;
        SolidBody2D target = CreateCircle2D(context, Vector2d.Zero);
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
    public void LateSimulate_WithBothRuntimeMode_ShouldNotTransferKinematic2DSourceIntoDynamic3DTarget()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Both;
        context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(context, Vector3d.Zero);
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
    public void SweepSphereAgainst2DAll_WithInactiveWrongLayerAndMultiVoxelTargets_ShouldFilterCandidatesOnce()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        LSCollider2D included = CreateBodylessBox2D(
            context,
            Vector2d.Zero,
            new Vector2d((Fixed64)8, (Fixed64)8));
        LSCollider2D inactive = CreateBodylessCircle2D(context, new Vector2d(Fixed64.Zero, Fixed64.One));
        LSCollider2D wrongLayer = CreateBodylessCircle2D(context, new Vector2d(Fixed64.Zero, -Fixed64.One));
        inactive.IsActive = false;
        wrongLayer.Layer = new PhysicsLayer(1);
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepSphereAgainst2DAll(
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            hits);

        count.Should().Be(1);
        context.QueryMixed.LastQueryCandidateCount.Should().Be(1);
        hits[0].Collider2D.Should().BeSameAs(included);
        hits.Should().NotContain(hit => ReferenceEquals(hit.Collider2D, inactive));
        hits.Should().NotContain(hit => ReferenceEquals(hit.Collider2D, wrongLayer));
    }

    [Fact]
    public void SweepCircleAgainst3DAll_WithInactiveWrongLayerAndMultiVoxelTargets_ShouldFilterCandidatesOnce()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        ScenarioBody<LSCuboidCollider> included = CreateBody3D(
            context,
            new LSCuboidCollider { Size = new Vector3d((Fixed64)8, Fixed64.One, (Fixed64)8) },
            Vector3d.Zero,
            immovable: true);
        ScenarioBody<LSSphereCollider> inactive = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.One),
            immovable: true);
        ScenarioBody<LSSphereCollider> wrongLayer = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, -Fixed64.One),
            immovable: true);
        inactive.Collider.Deactivate();
        wrongLayer.Collider.Layer = new PhysicsLayer(1);
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            new Vector2d((Fixed64)5, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            hits);

        count.Should().Be(1);
        context.QueryMixed.LastQueryCandidateCount.Should().Be(1);
        hits[0].Collider3D.Should().BeSameAs(included.Collider);
        hits.Should().NotContain(hit => ReferenceEquals(hit.Collider3D, inactive.Collider));
        hits.Should().NotContain(hit => ReferenceEquals(hit.Collider3D, wrongLayer.Collider));
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

    private static TCollider CreateBodyless3D<TCollider>(
        GravitasWorldContext context,
        TCollider collider,
        Vector3d position)
        where TCollider : LSCollider
    {
        var agent = new TestMatterAgent(context, new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One));
        collider.InitializeWithNoBody(agent);
        return collider;
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

    private static (
        Vector2d SourcePosition,
        Vector2d SourceVelocity,
        Vector3d TargetPosition,
        Vector3d TargetVelocity,
        int SourceToiIterations) RunVerticalMixedDynamic2DSourceScenario()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        SolidBody2D source = CreateCircle2D(context, new Vector2d((Fixed64)(-2), Fixed64.Zero));
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.Half, Fixed64.FromFraction(7, 2), Fixed64.Zero));
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.AddForce(Vector2d.Right * (Fixed64)3);
        target.Body.AddForce(Vector3d.Down * (Fixed64)5);
        context.LateSimulate();

        return (
            source.Position,
            source.LinearVelocity,
            target.Body.Position3d,
            target.Body.LinearVelocity,
            source.LastContinuousCollisionToiIterationCount);
    }

    private static (
        Vector2d SourcePosition,
        Vector2d SourceVelocity,
        Vector3d TargetPosition,
        Vector3d TargetVelocity,
        int SourceToiIterations) RunPlanarSeparatingMixed2DSourceScenario(bool isKinematic)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        SolidBody2D source = CreateCircle2D(
            context,
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            isKinematic: isKinematic);
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(
            context,
            new Vector3d(Fixed64.FromFraction(-3, 2), Fixed64.FromFraction(7, 2), Fixed64.Zero));
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        if (isKinematic)
            source.Agent.Transform.Position = Vector3d.Zero;
        else
            source.AddForce(Vector2d.Right * (Fixed64)2);

        target.Body.AddForce(new Vector3d(Fixed64.One, Fixed64.FromFraction(-24, 5), Fixed64.Zero));
        context.LateSimulate();

        return (
            source.Position,
            source.LinearVelocity,
            target.Body.Position3d,
            target.Body.LinearVelocity,
            source.LastContinuousCollisionToiIterationCount);
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
