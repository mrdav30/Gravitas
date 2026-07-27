using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Diagnostics;
using Gravitas.Tests.Support;
using System;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Diagnostics;

public sealed class GravitasDiagnosticEventViewTests
{
    [Fact]
    public void TryAsBodyDeltaViews_ShouldMapBodyPayloadsAndRejectWrongKinds()
    {
        Vector3d force = new((Fixed64)4, Fixed64.One, Fixed64.Zero);
        Vector3d accelerationDelta = new(Fixed64.Half, Fixed64.Zero, Fixed64.Zero);
        GravitasDiagnosticEvent forceEvent = CreateEvent(
            GravitasDiagnosticEventKind.ForceDelta,
            bodyId: 11,
            colliderAId: 21,
            colliderAType: ColliderType.Sphere,
            vector: force,
            pointA: accelerationDelta,
            scalarA: force.Magnitude);

        forceEvent.TryAsForceDelta(out GravitasForceDeltaDiagnosticView forceView).Should().BeTrue();
        forceView.Frame.Should().Be(7);
        forceView.Sequence.Should().Be(3);
        forceView.BodyId.Should().Be(11);
        forceView.ColliderId.Should().Be(21);
        forceView.ColliderType.Should().Be(ColliderType.Sphere);
        forceView.Force.Should().Be(force);
        forceView.AccelerationDelta.Should().Be(accelerationDelta);
        forceView.ForceMagnitude.Should().Be(force.Magnitude);
        forceEvent.TryAsTorqueDelta(out _).Should().BeFalse();

        Vector3d torque = new(Fixed64.Zero, (Fixed64)5, Fixed64.One);
        GravitasDiagnosticEvent torqueEvent = CreateEvent(
            GravitasDiagnosticEventKind.TorqueDelta,
            bodyId: 12,
            colliderAId: 22,
            colliderAType: ColliderType.Capsule,
            vector: torque,
            scalarA: torque.Magnitude);

        torqueEvent.TryAsTorqueDelta(out GravitasTorqueDeltaDiagnosticView torqueView).Should().BeTrue();
        torqueView.Frame.Should().Be(7);
        torqueView.BodyId.Should().Be(12);
        torqueView.ColliderId.Should().Be(22);
        torqueView.ColliderType.Should().Be(ColliderType.Capsule);
        torqueView.Torque.Should().Be(torque);
        torqueView.TorqueMagnitude.Should().Be(torque.Magnitude);

        Vector3d before = new(Fixed64.One, Fixed64.Zero, Fixed64.Zero);
        Vector3d after = new((Fixed64)2, Fixed64.Zero, Fixed64.One);
        GravitasDiagnosticEvent linearEvent = CreateEvent(
            GravitasDiagnosticEventKind.LinearVelocityDelta,
            bodyId: 13,
            colliderAId: 23,
            colliderAType: ColliderType.OBBox,
            start: before,
            end: after,
            vector: after - before,
            scalarA: after.Magnitude);

        linearEvent.TryAsLinearVelocityDelta(out GravitasVelocityDeltaDiagnosticView linearView).Should().BeTrue();
        linearView.Frame.Should().Be(7);
        linearView.Sequence.Should().Be(3);
        linearView.Kind.Should().Be(GravitasDiagnosticEventKind.LinearVelocityDelta);
        linearView.BodyId.Should().Be(13);
        linearView.ColliderId.Should().Be(23);
        linearView.ColliderType.Should().Be(ColliderType.OBBox);
        linearView.Before.Should().Be(before);
        linearView.After.Should().Be(after);
        linearView.Delta.Should().Be(after - before);
        linearView.ResultSpeed.Should().Be(after.Magnitude);

        GravitasDiagnosticEvent angularEvent = CreateEvent(
            GravitasDiagnosticEventKind.AngularVelocityDelta,
            bodyId: 14,
            colliderAId: 24,
            colliderAType: ColliderType.Cylinder,
            start: before,
            end: after,
            vector: after - before,
            scalarA: after.Magnitude);

        angularEvent.TryAsAngularVelocityDelta(out GravitasVelocityDeltaDiagnosticView angularView).Should().BeTrue();
        angularView.Kind.Should().Be(GravitasDiagnosticEventKind.AngularVelocityDelta);
        angularView.BodyId.Should().Be(14);
        angularView.ColliderId.Should().Be(24);
        angularView.ColliderType.Should().Be(ColliderType.Cylinder);
        angularView.Before.Should().Be(before);
        angularView.After.Should().Be(after);
        angularView.Delta.Should().Be(after - before);
        angularView.ResultSpeed.Should().Be(after.Magnitude);
    }

    [Fact]
    public void TryAsViews_ShouldRejectNonMatchingKinds()
    {
        GravitasDiagnosticEvent forceEvent = CreateEvent(GravitasDiagnosticEventKind.ForceDelta);
        GravitasDiagnosticEvent torqueEvent = CreateEvent(GravitasDiagnosticEventKind.TorqueDelta);

        forceEvent.TryAsTorqueDelta(out _).Should().BeFalse();
        forceEvent.TryAsLinearVelocityDelta(out _).Should().BeFalse();
        forceEvent.TryAsAngularVelocityDelta(out _).Should().BeFalse();
        forceEvent.TryAsGroundProbe(out _).Should().BeFalse();
        forceEvent.TryAsRayQuery(out _).Should().BeFalse();
        forceEvent.TryAsCircleQuery(out _).Should().BeFalse();
        forceEvent.TryAsQuerySummary(out _).Should().BeFalse();
        forceEvent.TryAsContact(out _).Should().BeFalse();
        forceEvent.TryAsResponseImpulse(out _).Should().BeFalse();
        forceEvent.TryAsMixedQuery(out _).Should().BeFalse();
        forceEvent.TryAsMixedContact(out _).Should().BeFalse();
        forceEvent.TryAsMixedResponseImpulse(out _).Should().BeFalse();
        forceEvent.TryAsMixedResponseIsland(out _).Should().BeFalse();
        forceEvent.TryAsJoint(out _).Should().BeFalse();
        forceEvent.TryAsRagdoll(out _).Should().BeFalse();
        torqueEvent.TryAsForceDelta(out _).Should().BeFalse();
    }

    [Fact]
    public void TryAsQueryViews_ShouldMapQueryPayloads()
    {
        Vector3d start = new(Fixed64.One, (Fixed64)2, (Fixed64)3);
        Vector3d end = new((Fixed64)4, (Fixed64)5, (Fixed64)6);
        Vector3d hitPoint = new((Fixed64)7, (Fixed64)8, (Fixed64)9);
        Vector3d normal = new(Fixed64.Zero, Fixed64.One, Fixed64.Zero);
        GravitasDiagnosticEvent groundProbeEvent = CreateEvent(
            GravitasDiagnosticEventKind.GroundProbe,
            bodyId: 31,
            colliderAId: 41,
            colliderBId: 42,
            colliderAType: ColliderType.Sphere,
            colliderBType: ColliderType.Mesh,
            start: start,
            end: end,
            pointA: hitPoint,
            vector: normal,
            scalarA: Fixed64.Half,
            scalarB: (Fixed64)10,
            dataA: (int)GroundProbeMode.SweptSphere,
            hit: true);

        groundProbeEvent.TryAsGroundProbe(out GravitasGroundProbeDiagnosticView groundView).Should().BeTrue();
        groundView.Frame.Should().Be(7);
        groundView.Sequence.Should().Be(3);
        groundView.BodyId.Should().Be(31);
        groundView.ColliderId.Should().Be(41);
        groundView.HitColliderId.Should().Be(42);
        groundView.ColliderType.Should().Be(ColliderType.Sphere);
        groundView.HitColliderType.Should().Be(ColliderType.Mesh);
        groundView.Start.Should().Be(start);
        groundView.End.Should().Be(end);
        groundView.HitPoint.Should().Be(hitPoint);
        groundView.Normal.Should().Be(normal);
        groundView.Radius.Should().Be(Fixed64.Half);
        groundView.Distance.Should().Be((Fixed64)10);
        groundView.Mode.Should().Be(GroundProbeMode.SweptSphere);
        groundView.Hit.Should().BeTrue();

        GravitasDiagnosticEvent groundProbe2DEvent = CreateEvent(
            GravitasDiagnosticEventKind.GroundProbe,
            bodyId: 32,
            colliderAId: 43,
            colliderBId: 44,
            colliderADimension: GravitasColliderDimension.TwoD,
            colliderBDimension: GravitasColliderDimension.TwoD,
            colliderA2DType: ColliderType2D.Circle,
            colliderB2DType: ColliderType2D.AABox,
            start: start,
            end: end,
            pointA: hitPoint,
            vector: normal,
            scalarA: Fixed64.Half,
            scalarB: (Fixed64)3,
            dataA: (int)GroundProbeMode2D.SweptCircle,
            dataB: (int)GravitasColliderDimension.TwoD,
            hit: true);

        groundProbe2DEvent.TryAsGroundProbe(out GravitasGroundProbeDiagnosticView ground2DView).Should().BeTrue();
        ground2DView.ColliderDimension.Should().Be(GravitasColliderDimension.TwoD);
        ground2DView.HitColliderDimension.Should().Be(GravitasColliderDimension.TwoD);
        ground2DView.Collider2DType.Should().Be(ColliderType2D.Circle);
        ground2DView.HitCollider2DType.Should().Be(ColliderType2D.AABox);
        ground2DView.Mode2D.Should().Be(GroundProbeMode2D.SweptCircle);

        GravitasDiagnosticEvent rayEvent = CreateEvent(
            GravitasDiagnosticEventKind.RayQuery,
            colliderAId: 51,
            colliderAType: ColliderType.Compound,
            start: start,
            end: end,
            pointA: hitPoint,
            vector: normal,
            scalarA: Fixed64.Zero,
            scalarB: (Fixed64)11,
            dataA: 0x12,
            dataB: 2,
            hit: true);

        rayEvent.TryAsRayQuery(out GravitasRayQueryDiagnosticView rayView).Should().BeTrue();
        rayView.Frame.Should().Be(7);
        rayView.Sequence.Should().Be(3);
        rayView.HitColliderId.Should().Be(51);
        rayView.HitColliderType.Should().Be(ColliderType.Compound);
        rayView.Start.Should().Be(start);
        rayView.End.Should().Be(end);
        rayView.HitPoint.Should().Be(hitPoint);
        rayView.Normal.Should().Be(normal);
        rayView.SweepRadius.Should().Be(Fixed64.Zero);
        rayView.Distance.Should().Be((Fixed64)11);
        rayView.LayerMaskBits.Should().Be(0x12);
        rayView.HitCount.Should().Be(2);
        rayView.Hit.Should().BeTrue();

        GravitasDiagnosticEvent circleEvent = CreateEvent(
            GravitasDiagnosticEventKind.CircleQuery,
            colliderAId: 61,
            colliderAType: ColliderType.Cylinder,
            start: start,
            end: end,
            pointA: hitPoint,
            vector: Vector3d.Forward,
            scalarA: (Fixed64)3,
            scalarB: (Fixed64)12,
            dataA: 0x24,
            dataB: 4,
            hit: true);

        circleEvent.TryAsCircleQuery(out GravitasCircleQueryDiagnosticView circleView).Should().BeTrue();
        circleView.Frame.Should().Be(7);
        circleView.Sequence.Should().Be(3);
        circleView.HitColliderId.Should().Be(61);
        circleView.HitColliderType.Should().Be(ColliderType.Cylinder);
        circleView.Center.Should().Be(start);
        circleView.End.Should().Be(end);
        circleView.HitPoint.Should().Be(hitPoint);
        circleView.Direction.Should().Be(Vector3d.Forward);
        circleView.Radius.Should().Be((Fixed64)3);
        circleView.Distance.Should().Be((Fixed64)12);
        circleView.LayerMaskBits.Should().Be(0x24);
        circleView.HitCount.Should().Be(4);
        circleView.Hit.Should().BeTrue();

        GravitasDiagnosticEvent querySummaryEvent = CreateEvent(
            GravitasDiagnosticEventKind.QuerySummary,
            colliderADimension: GravitasColliderDimension.TwoD,
            colliderBDimension: GravitasColliderDimension.ThreeD,
            start: start,
            end: end,
            scalarA: (Fixed64)2,
            scalarB: (Fixed64)1,
            dataA: 5,
            dataB: 3,
            hit: true);

        querySummaryEvent.TryAsQuerySummary(out GravitasQuerySummaryDiagnosticView summaryView).Should().BeTrue();
        summaryView.Frame.Should().Be(7);
        summaryView.Sequence.Should().Be(3);
        summaryView.SourceDimension.Should().Be(GravitasColliderDimension.TwoD);
        summaryView.TargetDimension.Should().Be(GravitasColliderDimension.ThreeD);
        summaryView.Start.Should().Be(start);
        summaryView.End.Should().Be(end);
        summaryView.ExactReducerAttempts.Should().Be(5);
        summaryView.AcceptedHits.Should().Be(3);
        summaryView.FallbackHits.Should().Be(2);
        summaryView.RejectedConservativeCandidates.Should().Be(1);
        summaryView.HasConservativeFallback.Should().BeTrue();

        CreateEvent(GravitasDiagnosticEventKind.QuerySummary)
            .TryAsQuerySummary(out GravitasQuerySummaryDiagnosticView exactSummaryView)
            .Should()
            .BeTrue();
        exactSummaryView.HasConservativeFallback.Should().BeFalse();
    }

    [Fact]
    public void TryAsContactViews_ShouldMapContactAndImpulsePayloads()
    {
        Vector3d pointA = new(Fixed64.One, Fixed64.Zero, Fixed64.Zero);
        Vector3d pointB = new(Fixed64.Zero, Fixed64.One, Fixed64.Zero);
        Vector3d normal = Vector3d.Right;
        GravitasDiagnosticEvent contactEvent = CreateEvent(
            GravitasDiagnosticEventKind.Contact,
            colliderAId: 71,
            colliderBId: 72,
            colliderAType: ColliderType.Sphere,
            colliderBType: ColliderType.OBBox,
            pointA: pointA,
            pointB: pointB,
            hasPointA: true,
            hasPointB: true,
            vector: normal,
            scalarA: Fixed64.Half,
            dataA: 2,
            dataB: 1,
            hit: true);

        contactEvent.TryAsContact(out GravitasContactDiagnosticView contactView).Should().BeTrue();
        contactView.Frame.Should().Be(7);
        contactView.Sequence.Should().Be(3);
        contactView.ColliderAId.Should().Be(71);
        contactView.ColliderBId.Should().Be(72);
        contactView.ColliderAType.Should().Be(ColliderType.Sphere);
        contactView.ColliderBType.Should().Be(ColliderType.OBBox);
        contactView.PointA.Should().Be(pointA);
        contactView.PointB.Should().Be(pointB);
        contactView.HasPointA.Should().BeTrue();
        contactView.HasPointB.Should().BeTrue();
        contactView.Normal.Should().Be(normal);
        contactView.Depth.Should().Be(Fixed64.Half);
        contactView.ContactCount.Should().Be(2);
        contactView.DepthIsClamped.Should().BeTrue();
        contactView.HasContact.Should().BeTrue();

        Vector3d impulse = Vector3d.Left * (Fixed64)5;
        GravitasDiagnosticEvent impulseEvent = CreateEvent(
            GravitasDiagnosticEventKind.ResponseImpulse,
            colliderAId: 73,
            colliderBId: 74,
            colliderAType: ColliderType.Capsule,
            colliderBType: ColliderType.Mesh,
            pointA: pointA,
            pointB: pointB,
            hasPointA: true,
            hasPointB: true,
            vector: impulse,
            scalarA: impulse.Magnitude,
            scalarB: (Fixed64)(-2),
            hit: true);

        impulseEvent.TryAsResponseImpulse(out GravitasResponseImpulseDiagnosticView impulseView).Should().BeTrue();
        impulseView.Frame.Should().Be(7);
        impulseView.Sequence.Should().Be(3);
        impulseView.ColliderAId.Should().Be(73);
        impulseView.ColliderBId.Should().Be(74);
        impulseView.ColliderAType.Should().Be(ColliderType.Capsule);
        impulseView.ColliderBType.Should().Be(ColliderType.Mesh);
        impulseView.PointA.Should().Be(pointA);
        impulseView.PointB.Should().Be(pointB);
        impulseView.HasPointA.Should().BeTrue();
        impulseView.HasPointB.Should().BeTrue();
        impulseView.Impulse.Should().Be(impulse);
        impulseView.ImpulseMagnitude.Should().Be(impulse.Magnitude);
        impulseView.NormalVelocity.Should().Be((Fixed64)(-2));

        GravitasDiagnosticEvent relativeOnlyEvent = CreateEvent(
            GravitasDiagnosticEventKind.Contact,
            hit: true);
        relativeOnlyEvent.TryAsContact(out GravitasContactDiagnosticView relativeOnlyView)
            .Should()
            .BeTrue();
        relativeOnlyView.HasContact.Should().BeTrue();
        relativeOnlyView.HasPointA.Should().BeFalse();
        relativeOnlyView.HasPointB.Should().BeFalse();
        relativeOnlyView.DepthIsClamped.Should().BeFalse();
    }

    [Fact]
    public void TryAsMixedViews_ShouldMapDimensionTaggedPayloads()
    {
        Vector3d point3D = new(Fixed64.One, Fixed64.Zero, Fixed64.Zero);
        Vector3d point2D = new(Fixed64.Zero, Fixed64.Zero, Fixed64.One);
        Vector3d normal = new(Fixed64.Zero, Fixed64.One, Fixed64.Zero);
        Vector3d mixedQueryStart = new((Fixed64)(-2), Fixed64.One, (Fixed64)3);
        GravitasDiagnosticEvent mixedQueryEvent = CreateEvent(
            GravitasDiagnosticEventKind.MixedQuery,
            colliderAId: 81,
            colliderBId: 82,
            colliderADimension: GravitasColliderDimension.ThreeD,
            colliderBDimension: GravitasColliderDimension.TwoD,
            colliderAType: ColliderType.Sphere,
            colliderB2DType: ColliderType2D.Compound,
            start: mixedQueryStart,
            end: Vector3d.Right,
            pointA: point3D,
            pointB: point2D,
            vector: normal,
            scalarA: Fixed64.Half,
            scalarB: (Fixed64)8,
            dataA: 0x44,
            dataB: 3,
            hit: true);

        mixedQueryEvent.TryAsMixedQuery(out GravitasMixedQueryDiagnosticView queryView).Should().BeTrue();
        queryView.Frame.Should().Be(7);
        queryView.Sequence.Should().Be(3);
        queryView.Collider3DId.Should().Be(81);
        queryView.Collider2DId.Should().Be(82);
        queryView.Collider3DType.Should().Be(ColliderType.Sphere);
        queryView.Collider2DType.Should().Be(ColliderType2D.Compound);
        queryView.Collider3DDimension.Should().Be(GravitasColliderDimension.ThreeD);
        queryView.Collider2DDimension.Should().Be(GravitasColliderDimension.TwoD);
        queryView.Start.Should().Be(mixedQueryStart);
        queryView.End.Should().Be(Vector3d.Right);
        queryView.Point3D.Should().Be(point3D);
        queryView.Point2D.Should().Be(point2D);
        queryView.Normal3DTo2D.Should().Be(normal);
        queryView.Radius.Should().Be(Fixed64.Half);
        queryView.Distance.Should().Be((Fixed64)8);
        queryView.LayerMaskBits.Should().Be(0x44);
        queryView.HitCount.Should().Be(3);
        queryView.Hit.Should().BeTrue();

        GravitasDiagnosticEvent mixedContactEvent = CreateEvent(
            GravitasDiagnosticEventKind.MixedContact,
            colliderAId: 83,
            colliderBId: 84,
            colliderADimension: GravitasColliderDimension.ThreeD,
            colliderBDimension: GravitasColliderDimension.TwoD,
            colliderAType: ColliderType.OBBox,
            colliderB2DType: ColliderType2D.Circle,
            pointA: point3D,
            pointB: point2D,
            hasPointA: true,
            hasPointB: true,
            vector: normal,
            scalarA: (Fixed64)2,
            dataA: 1,
            hit: true);

        mixedContactEvent.TryAsMixedContact(out GravitasMixedContactDiagnosticView contactView).Should().BeTrue();
        contactView.Frame.Should().Be(7);
        contactView.Sequence.Should().Be(3);
        contactView.Collider3DId.Should().Be(83);
        contactView.Collider2DId.Should().Be(84);
        contactView.Collider3DType.Should().Be(ColliderType.OBBox);
        contactView.Collider2DType.Should().Be(ColliderType2D.Circle);
        contactView.Collider3DDimension.Should().Be(GravitasColliderDimension.ThreeD);
        contactView.Collider2DDimension.Should().Be(GravitasColliderDimension.TwoD);
        contactView.Point3D.Should().Be(point3D);
        contactView.Point2D.Should().Be(point2D);
        contactView.HasPoint3D.Should().BeTrue();
        contactView.HasPoint2D.Should().BeTrue();
        contactView.Normal3DTo2D.Should().Be(normal);
        contactView.Depth.Should().Be((Fixed64)2);
        contactView.DepthIsClamped.Should().BeTrue();
        contactView.HasContact.Should().BeTrue();

        Vector3d impulse = normal * (Fixed64)6;
        GravitasDiagnosticEvent mixedImpulseEvent = CreateEvent(
            GravitasDiagnosticEventKind.MixedResponseImpulse,
            colliderAId: 85,
            colliderBId: 86,
            colliderADimension: GravitasColliderDimension.ThreeD,
            colliderBDimension: GravitasColliderDimension.TwoD,
            colliderAType: ColliderType.Mesh,
            colliderB2DType: ColliderType2D.AABox,
            pointA: point3D,
            pointB: point2D,
            hasPointA: true,
            hasPointB: true,
            vector: impulse,
            scalarA: impulse.Magnitude,
            scalarB: (Fixed64)(-3),
            dataA: 2,
            dataB: 4,
            hit: true);

        mixedImpulseEvent.TryAsMixedResponseImpulse(out GravitasMixedResponseImpulseDiagnosticView impulseView).Should().BeTrue();
        impulseView.Frame.Should().Be(7);
        impulseView.Sequence.Should().Be(3);
        impulseView.Collider3DId.Should().Be(85);
        impulseView.Collider2DId.Should().Be(86);
        impulseView.Collider3DType.Should().Be(ColliderType.Mesh);
        impulseView.Collider2DType.Should().Be(ColliderType2D.AABox);
        impulseView.Collider3DDimension.Should().Be(GravitasColliderDimension.ThreeD);
        impulseView.Collider2DDimension.Should().Be(GravitasColliderDimension.TwoD);
        impulseView.Point3D.Should().Be(point3D);
        impulseView.Point2D.Should().Be(point2D);
        impulseView.HasPoint3D.Should().BeTrue();
        impulseView.HasPoint2D.Should().BeTrue();
        impulseView.Impulse.Should().Be(impulse);
        impulseView.ImpulseMagnitude.Should().Be(impulse.Magnitude);
        impulseView.NormalVelocity.Should().Be((Fixed64)(-3));
        impulseView.Iteration.Should().Be(2);
        impulseView.IterationLimit.Should().Be(4);

        GravitasDiagnosticEvent mixedIslandEvent = CreateEvent(
            GravitasDiagnosticEventKind.MixedResponseIsland,
            bodyId: 123,
            dataA: 3,
            dataB: 4,
            hit: true);

        mixedIslandEvent.TryAsMixedResponseIsland(out GravitasMixedResponseIslandDiagnosticView islandView).Should().BeTrue();
        islandView.Frame.Should().Be(7);
        islandView.Sequence.Should().Be(3);
        islandView.RootKey.Should().Be(123);
        islandView.ConstraintCount.Should().Be(3);
        islandView.IterationCount.Should().Be(4);
        islandView.ReachedIterationLimit.Should().BeTrue();
    }

    [Fact]
    public void TryAsJointAndRagdollViews_ShouldMapConstraintPayloads()
    {
        Vector3d jointVector = new((Fixed64)5, (Fixed64)6, (Fixed64)7);
        GravitasDiagnosticEvent jointEvent = CreateEvent(
            GravitasDiagnosticEventKind.JointImpulse,
            jointId: 91,
            colliderAId: 92,
            colliderBId: 93,
            colliderADimension: GravitasColliderDimension.ThreeD,
            colliderBDimension: GravitasColliderDimension.TwoD,
            colliderAType: ColliderType.Capsule,
            colliderB2DType: ColliderType2D.Capsule,
            vector: jointVector,
            scalarA: (Fixed64)4,
            scalarB: Fixed64.Half,
            dataA: 8,
            dataB: 2);

        jointEvent.TryAsJoint(out GravitasJointDiagnosticView jointView).Should().BeTrue();
        jointView.Frame.Should().Be(7);
        jointView.Sequence.Should().Be(3);
        jointView.Kind.Should().Be(GravitasDiagnosticEventKind.JointImpulse);
        jointView.JointId.Should().Be(91);
        jointView.ColliderAId.Should().Be(92);
        jointView.ColliderBId.Should().Be(93);
        jointView.ColliderADimension.Should().Be(GravitasColliderDimension.ThreeD);
        jointView.ColliderBDimension.Should().Be(GravitasColliderDimension.TwoD);
        jointView.ColliderAType.Should().Be(ColliderType.Capsule);
        jointView.ColliderBType.Should().Be(ColliderType.None);
        jointView.ColliderA2DType.Should().Be(ColliderType2D.None);
        jointView.ColliderB2DType.Should().Be(ColliderType2D.Capsule);
        jointView.ImpulseMagnitude.Should().Be((Fixed64)4);
        jointView.LinearAnchorErrorMagnitude.Should().Be(Fixed64.Half);
        jointView.MotorImpulseMagnitude.Should().Be((Fixed64)5);
        jointView.MotorErrorMagnitude.Should().Be((Fixed64)6);
        jointView.LimitErrorMagnitude.Should().Be((Fixed64)7);
        jointView.LimitError.Should().Be((Fixed64)7);
        jointView.RowCount.Should().Be(8);
        jointView.ClampedRowCount.Should().Be(2);
        jointEvent.TryAsRagdoll(out _).Should().BeFalse();

        GravitasDiagnosticEvent limitEvent = CreateEvent(
            GravitasDiagnosticEventKind.JointLimitReached,
            vector: jointVector,
            scalarB: (Fixed64)9);

        limitEvent.TryAsJoint(out GravitasJointDiagnosticView limitView).Should().BeTrue();
        limitView.LimitError.Should().Be((Fixed64)9);

        CreateEvent(GravitasDiagnosticEventKind.JointRegistered, jointId: 94)
            .TryAsJoint(out GravitasJointDiagnosticView registeredView)
            .Should()
            .BeTrue();
        registeredView.JointId.Should().Be(94);
        CreateEvent(GravitasDiagnosticEventKind.JointRemoved, jointId: 95)
            .TryAsJoint(out GravitasJointDiagnosticView removedView)
            .Should()
            .BeTrue();
        removedView.JointId.Should().Be(95);

        GravitasDiagnosticEvent ragdollEvent = CreateEvent(
            GravitasDiagnosticEventKind.RagdollActivated,
            bodyId: 101,
            dataA: 12,
            dataB: 13,
            hit: true);

        ragdollEvent.TryAsRagdoll(out GravitasRagdollDiagnosticView ragdollView).Should().BeTrue();
        ragdollView.Frame.Should().Be(7);
        ragdollView.Sequence.Should().Be(3);
        ragdollView.RagdollId.Should().Be(101);
        ragdollView.LinkCount.Should().Be(12);
        ragdollView.JointCount.Should().Be(13);
        ragdollView.IsActive.Should().BeTrue();
        ragdollEvent.TryAsJoint(out _).Should().BeFalse();
    }

    [Fact]
    public void DispatchTo_ShouldRouteEveryEventKindToTypedVisitorMethod()
    {
        var visitor = new RecordingDiagnosticEventVisitor();

        CreateEvent(GravitasDiagnosticEventKind.ForceDelta, vector: Vector3d.Right).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.TorqueDelta, vector: Vector3d.Up).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.LinearVelocityDelta, start: Vector3d.Zero, end: Vector3d.Right).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.AngularVelocityDelta, start: Vector3d.Zero, end: Vector3d.Up).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.GroundProbe, dataA: (int)GroundProbeMode.Ray).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.RayQuery, scalarA: Fixed64.Half).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.CircleQuery, scalarA: Fixed64.One).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.QuerySummary, dataA: 3, dataB: 2).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.Contact, dataA: 2, hit: true).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.ResponseImpulse, vector: Vector3d.Left).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.MixedQuery, colliderADimension: GravitasColliderDimension.ThreeD, colliderBDimension: GravitasColliderDimension.TwoD).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.MixedContact, scalarA: Fixed64.Half).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.MixedResponseImpulse, vector: Vector3d.Down).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.MixedResponseIsland, bodyId: 7).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.JointRegistered, jointId: 8).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.JointRemoved, jointId: 9).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.JointImpulse, jointId: 10).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.JointLimitReached, jointId: 11).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.RagdollActivated, bodyId: 12).DispatchTo(visitor);
        CreateEvent((GravitasDiagnosticEventKind)250).DispatchTo(visitor);

        visitor.Route.Should().Equal(
            nameof(RecordingDiagnosticEventVisitor.VisitForceDelta),
            nameof(RecordingDiagnosticEventVisitor.VisitTorqueDelta),
            nameof(RecordingDiagnosticEventVisitor.VisitLinearVelocityDelta),
            nameof(RecordingDiagnosticEventVisitor.VisitAngularVelocityDelta),
            nameof(RecordingDiagnosticEventVisitor.VisitGroundProbe),
            nameof(RecordingDiagnosticEventVisitor.VisitRayQuery),
            nameof(RecordingDiagnosticEventVisitor.VisitCircleQuery),
            nameof(RecordingDiagnosticEventVisitor.VisitQuerySummary),
            nameof(RecordingDiagnosticEventVisitor.VisitContact),
            nameof(RecordingDiagnosticEventVisitor.VisitResponseImpulse),
            nameof(RecordingDiagnosticEventVisitor.VisitMixedQuery),
            nameof(RecordingDiagnosticEventVisitor.VisitMixedContact),
            nameof(RecordingDiagnosticEventVisitor.VisitMixedResponseImpulse),
            nameof(RecordingDiagnosticEventVisitor.VisitMixedResponseIsland),
            nameof(RecordingDiagnosticEventVisitor.VisitJoint),
            nameof(RecordingDiagnosticEventVisitor.VisitJoint),
            nameof(RecordingDiagnosticEventVisitor.VisitJoint),
            nameof(RecordingDiagnosticEventVisitor.VisitJoint),
            nameof(RecordingDiagnosticEventVisitor.VisitRagdoll),
            nameof(RecordingDiagnosticEventVisitor.VisitUnknown));
        visitor.LastForce.Force.Should().Be(Vector3d.Right);
        visitor.LastMixedContact.Depth.Should().Be(Fixed64.Half);
        visitor.LastJoint.JointId.Should().Be(11);
        visitor.LastRagdoll.RagdollId.Should().Be(12);
        visitor.LastUnknown.Kind.Should().Be((GravitasDiagnosticEventKind)250);
    }

    [Fact]
    public void PartialDiagnosticEventVisitor_ShouldIgnoreUnimplementedKindsAndPreserveOverriddenRoute()
    {
        var visitor = new PartialDiagnosticEventVisitor();

        CreateEvent(GravitasDiagnosticEventKind.ForceDelta, vector: Vector3d.Right).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.TorqueDelta).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.LinearVelocityDelta).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.AngularVelocityDelta).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.GroundProbe).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.RayQuery).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.CircleQuery).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.QuerySummary).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.Contact).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.ResponseImpulse).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.MixedQuery).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.MixedContact).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.MixedResponseImpulse).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.MixedResponseIsland).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.JointRegistered).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.JointRemoved).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.JointImpulse).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.JointLimitReached).DispatchTo(visitor);
        CreateEvent(GravitasDiagnosticEventKind.RagdollActivated).DispatchTo(visitor);
        CreateEvent((GravitasDiagnosticEventKind)250).DispatchTo(visitor);

        visitor.ForceVisitCount.Should().Be(1);
        visitor.LastForce.Force.Should().Be(Vector3d.Right);
    }

    [Fact]
    public void DispatchEventsTo_ShouldVisitCapturedEventsInBufferOrder()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(Vector3d.Zero);
        var visitor = new RecordingDiagnosticEventVisitor();

        scenario.Context.Diagnostics.Enable(eventCapacity: 4, drawCommandCapacity: 0);
        sphere.Body.AddForce(Vector3d.Right);
        sphere.Body.AddTorque(Vector3d.Up);

        scenario.Context.Diagnostics.DispatchEventsTo(visitor);

        visitor.Route.Should().Equal(
            nameof(RecordingDiagnosticEventVisitor.VisitForceDelta),
            nameof(RecordingDiagnosticEventVisitor.VisitTorqueDelta));
        visitor.LastForce.Sequence.Should().Be(0);
        visitor.LastTorque.Sequence.Should().Be(1);
    }

    [Fact]
    public void DispatchTo_ShouldRejectNullDiagnosticVisitors()
    {
        GravitasDiagnosticEvent diagnosticEvent = CreateEvent(GravitasDiagnosticEventKind.ForceDelta);

        Action dispatchEvent = () => diagnosticEvent.DispatchTo(null!);

        dispatchEvent.Should().Throw<ArgumentNullException>().WithParameterName("visitor");
    }

    [Fact]
    public void DispatchEventsTo_ShouldRejectNullVisitor()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();

        Action dispatch = () => scenario.Context.Diagnostics.DispatchEventsTo(null!);

        dispatch.Should().Throw<ArgumentNullException>().WithParameterName("visitor");
    }

    private static GravitasDiagnosticEvent CreateEvent(
        GravitasDiagnosticEventKind kind,
        int bodyId = -1,
        int jointId = -1,
        int colliderAId = -1,
        int colliderBId = -1,
        GravitasColliderDimension colliderADimension = GravitasColliderDimension.None,
        GravitasColliderDimension colliderBDimension = GravitasColliderDimension.None,
        ColliderType colliderAType = ColliderType.None,
        ColliderType colliderBType = ColliderType.None,
        ColliderType2D colliderA2DType = ColliderType2D.None,
        ColliderType2D colliderB2DType = ColliderType2D.None,
        Vector3d start = default,
        Vector3d end = default,
        Vector3d pointA = default,
        Vector3d pointB = default,
        bool hasPointA = false,
        bool hasPointB = false,
        Vector3d vector = default,
        Fixed64 scalarA = default,
        Fixed64 scalarB = default,
        int dataA = 0,
        int dataB = 0,
        bool hit = false)
    {
        return new GravitasDiagnosticEvent(
            frame: 7,
            sequence: 3,
            kind: kind,
            bodyId: bodyId,
            jointId: jointId,
            colliderAId: colliderAId,
            colliderBId: colliderBId,
            colliderADimension: colliderADimension,
            colliderBDimension: colliderBDimension,
            colliderAType: colliderAType,
            colliderBType: colliderBType,
            colliderA2DType: colliderA2DType,
            colliderB2DType: colliderB2DType,
            start: start,
            end: end,
            pointA: pointA,
            pointB: pointB,
            hasPointA: hasPointA,
            hasPointB: hasPointB,
            vector: vector,
            scalarA: scalarA,
            scalarB: scalarB,
            dataA: dataA,
            dataB: dataB,
            hit: hit);
    }

    private sealed class RecordingDiagnosticEventVisitor : GravitasDiagnosticEventVisitor
    {
        public readonly List<string> Route = new();
        public GravitasForceDeltaDiagnosticView LastForce;
        public GravitasTorqueDeltaDiagnosticView LastTorque;
        public GravitasMixedContactDiagnosticView LastMixedContact;
        public GravitasJointDiagnosticView LastJoint;
        public GravitasRagdollDiagnosticView LastRagdoll;
        public GravitasDiagnosticEvent LastUnknown;

        public override void VisitForceDelta(in GravitasForceDeltaDiagnosticView view)
        {
            Route.Add(nameof(VisitForceDelta));
            LastForce = view;
        }

        public override void VisitTorqueDelta(in GravitasTorqueDeltaDiagnosticView view)
        {
            Route.Add(nameof(VisitTorqueDelta));
            LastTorque = view;
        }

        public override void VisitLinearVelocityDelta(in GravitasVelocityDeltaDiagnosticView view) =>
            Route.Add(nameof(VisitLinearVelocityDelta));

        public override void VisitAngularVelocityDelta(in GravitasVelocityDeltaDiagnosticView view) =>
            Route.Add(nameof(VisitAngularVelocityDelta));

        public override void VisitGroundProbe(in GravitasGroundProbeDiagnosticView view) =>
            Route.Add(nameof(VisitGroundProbe));

        public override void VisitRayQuery(in GravitasRayQueryDiagnosticView view) =>
            Route.Add(nameof(VisitRayQuery));

        public override void VisitCircleQuery(in GravitasCircleQueryDiagnosticView view) =>
            Route.Add(nameof(VisitCircleQuery));

        public override void VisitQuerySummary(in GravitasQuerySummaryDiagnosticView view) =>
            Route.Add(nameof(VisitQuerySummary));

        public override void VisitContact(in GravitasContactDiagnosticView view) =>
            Route.Add(nameof(VisitContact));

        public override void VisitResponseImpulse(in GravitasResponseImpulseDiagnosticView view) =>
            Route.Add(nameof(VisitResponseImpulse));

        public override void VisitMixedQuery(in GravitasMixedQueryDiagnosticView view) =>
            Route.Add(nameof(VisitMixedQuery));

        public override void VisitMixedContact(in GravitasMixedContactDiagnosticView view)
        {
            Route.Add(nameof(VisitMixedContact));
            LastMixedContact = view;
        }

        public override void VisitMixedResponseImpulse(in GravitasMixedResponseImpulseDiagnosticView view) =>
            Route.Add(nameof(VisitMixedResponseImpulse));

        public override void VisitMixedResponseIsland(in GravitasMixedResponseIslandDiagnosticView view) =>
            Route.Add(nameof(VisitMixedResponseIsland));

        public override void VisitJoint(in GravitasJointDiagnosticView view)
        {
            Route.Add(nameof(VisitJoint));
            LastJoint = view;
        }

        public override void VisitRagdoll(in GravitasRagdollDiagnosticView view)
        {
            Route.Add(nameof(VisitRagdoll));
            LastRagdoll = view;
        }

        public override void VisitUnknown(in GravitasDiagnosticEvent diagnosticEvent)
        {
            Route.Add(nameof(VisitUnknown));
            LastUnknown = diagnosticEvent;
        }
    }

    private sealed class PartialDiagnosticEventVisitor : GravitasDiagnosticEventVisitor
    {
        public int ForceVisitCount;
        public GravitasForceDeltaDiagnosticView LastForce;

        public override void VisitForceDelta(in GravitasForceDeltaDiagnosticView view)
        {
            base.VisitForceDelta(view);
            ForceVisitCount++;
            LastForce = view;
        }
    }
}
