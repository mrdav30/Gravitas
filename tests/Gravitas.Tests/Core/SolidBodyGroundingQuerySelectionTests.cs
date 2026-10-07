using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Diagnostics;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class SolidBodyGroundingQuerySelectionTests
{
    [Theory]
    [InlineData(GroundProbeMode.Ray)]
    [InlineData(GroundProbeMode.SweptSphere)]
    public void CheckGround_WithoutRawHits_ShouldEmitQueryMissBeforeGroundProbeMiss(GroundProbeMode mode)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        SolidBody body = CreateProbe(scenario, mode);
        scenario.Context.Diagnostics.Enable();

        body.CheckGround();

        body.IsGrounded.Should().BeFalse();
        body.HasHitPoint.Should().BeFalse();
        scenario.Context.Diagnostics.Events.Length.Should().Be(2);
        GravitasDiagnosticEvent raw = scenario.Context.Diagnostics.Events[0];
        raw.Kind.Should().Be(GravitasDiagnosticEventKind.RayQuery);
        raw.Hit.Should().BeFalse();
        raw.DataB.Should().Be(0);
        raw.ColliderAId.Should().Be(-1);
        GravitasDiagnosticEvent probe = scenario.Context.Diagnostics.Events[1];
        probe.Kind.Should().Be(GravitasDiagnosticEventKind.GroundProbe);
        probe.Hit.Should().BeFalse();
        probe.ColliderBId.Should().Be(-1);
    }

    [Fact]
    public void CheckGround_WithRawRaySelfHit_ShouldSelectSeparateSupportAndKeepSelfInRawDiagnostics()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        SolidBody body = CreateProbe(scenario, GroundProbeMode.Ray);
        scenario.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.All;
        LSCuboidCollider support = AddBox(scenario, Fixed64.Zero);

        AssertSelectionMatchesAllHits(scenario, body, body.Collider, support, 2);
    }

    [Theory]
    [InlineData(GroundProbeMode.Ray)]
    [InlineData(GroundProbeMode.SweptSphere)]
    public void CheckGround_WithLaterKinematicTieVisitedBeforeStatic_ShouldSelectLowerStaticOwnerId(GroundProbeMode mode)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        SolidBody body = CreateProbe(scenario, mode);
        LSCuboidCollider first = AddBox(scenario, Fixed64.Zero);
        var later = new LSCuboidCollider { Layer = new PhysicsLayer(1), Size = Vector3d.One };
        scenario.CreateBody(later, Vector3d.Zero, FixedQuaternion.Identity, isKinematic: true);
        first.Id.Should().BeLessThan(later.Id);

        // Equal geometry occupies the same voxels, whose kinematic role is
        // visited before the static role. The lower static ID must replace it.
        AssertSelectionMatchesAllHits(scenario, body, first, first, 2);
    }

    [Theory]
    [InlineData(GroundProbeMode.Ray)]
    [InlineData(GroundProbeMode.SweptSphere)]
    public void CheckGround_WhenDiagnosticsAreDisabledThenEnabled_ShouldPreserveSelectionAndQueryCounters(GroundProbeMode mode)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        SolidBody body = CreateProbe(scenario, mode);
        LSCuboidCollider near = AddBox(scenario, (Fixed64)3);
        body.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(near.Layer);
        LSMeshCollider far = MeshTestFixtures.CreateConvexQuadFloor();
        far.Layer = new PhysicsLayer(2);
        scenario.InitializeStaticCollider(far, Vector3d.Zero);
        GravitasWorldContext context = scenario.Context;
        Physics3DHit expected = AssertSelectionMatchesAllHits(scenario, body, near, far, 2);
        expected.TryGetPoint(out Vector3d point).Should().BeTrue();
        GravitasDiagnosticEvent expectedRawEvent = context.Diagnostics.Events[0];
        int candidateCount = context.Query3D.LastQueryCandidateCount;
        int triangleCount = context.Query3D.LastMeshTriangleCandidateCount;
        if (mode == GroundProbeMode.SweptSphere)
        {
            candidateCount.Should().Be(2);
            triangleCount.Should().BeGreaterThan(0);
        }

        for (int pass = 0; pass < 2; pass++)
        {
            if (pass == 0)
                context.Diagnostics.Disable();
            else
            {
                context.Diagnostics.Enable();
                context.Diagnostics.Clear();
            }

            body.CheckGround();

            body.IsGrounded.Should().BeTrue();
            body.HitPlatform.Should().BeSameAs(far.Transform);
            body.HitPoint.Should().Be(point);
            body.GroundNormal.Should().Be(expected.Normal);
            context.Query3D.LastQueryCandidateCount.Should().Be(candidateCount);
            context.Query3D.LastMeshTriangleCandidateCount.Should().Be(triangleCount);
            context.Diagnostics.Events.Length.Should().Be(pass == 0 ? 0 : 2);
            if (pass == 1)
            {
                // Re-enabling capture must restore the complete raw all-hit
                // diagnostic, including its ignored nearest owner and count.
                context.Diagnostics.Events[0].Should().BeEquivalentTo(expectedRawEvent);
                GravitasDiagnosticEvent probe = context.Diagnostics.Events[1];
                probe.ColliderBId.Should().Be(far.Id);
                probe.PointA.Should().Be(point);
                probe.Vector.Should().Be(expected.Normal);
                probe.ScalarB.Should().Be(expected.Distance);
            }
        }
    }

    [Theory]
    [InlineData(GroundProbeMode.Ray, 0)]
    [InlineData(GroundProbeMode.SweptSphere, 0)]
    [InlineData(GroundProbeMode.Ray, 1)]
    [InlineData(GroundProbeMode.SweptSphere, 1)]
    [InlineData(GroundProbeMode.Ray, 2)]
    [InlineData(GroundProbeMode.SweptSphere, 2)]
    public void CheckGround_WhenNearestRawHitIsIneligible_ShouldPreserveRawDiagnosticsAndSelectFartherSupport(
        GroundProbeMode mode, int rejection)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        SolidBody body = CreateProbe(scenario, mode);
        var near = new LSCuboidCollider { Layer = new PhysicsLayer(1), Size = Vector3d.One };
        if (rejection == 0)
            scenario.CreateBody(near, new Vector3d(0, 3, 0), FixedQuaternion.Identity);
        else
            scenario.InitializeStaticCollider(near, new Vector3d(0, 3, 0));
        LSCuboidCollider far = AddBox(scenario, Fixed64.Zero, 2);
        if (rejection == 1)
            body.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(near.Layer);
        else if (rejection == 2)
            near.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(body.Collider.Layer);

        AssertSelectionMatchesAllHits(scenario, body, near, far, 2);
    }

    [Theory]
    [InlineData(GroundProbeMode.Ray, true)]
    [InlineData(GroundProbeMode.SweptSphere, true)]
    [InlineData(GroundProbeMode.Ray, false)]
    [InlineData(GroundProbeMode.SweptSphere, false)]
    public void CheckGround_ShouldSelectByDistanceThenOwnerId(GroundProbeMode mode, bool equalDistance)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        SolidBody body = CreateProbe(scenario, mode);
        LSCuboidCollider first = AddBox(scenario, Fixed64.Zero);
        LSCuboidCollider later = AddBox(scenario, equalDistance ? Fixed64.Zero : (Fixed64)2);
        first.Id.Should().BeLessThan(later.Id);
        LSCollider expected = equalDistance ? first : later;

        AssertSelectionMatchesAllHits(scenario, body, expected, expected, 2);
    }

    [Theory]
    [InlineData(GroundProbeMode.Ray, false)]
    [InlineData(GroundProbeMode.SweptSphere, false)]
    [InlineData(GroundProbeMode.Ray, true)]
    [InlineData(GroundProbeMode.SweptSphere, true)]
    public void CheckGround_ShouldAcceptTriggersAndKinematicSupports(GroundProbeMode mode, bool kinematic)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        SolidBody body = CreateProbe(scenario, mode);
        var support = new LSCuboidCollider { Layer = new PhysicsLayer(1), IsTrigger = !kinematic };
        if (kinematic)
            scenario.CreateBody(support, Vector3d.Zero, FixedQuaternion.Identity, isKinematic: true);
        else
            scenario.InitializeStaticCollider(support, Vector3d.Zero);

        AssertSelectionMatchesAllHits(scenario, body, support, support, 1);
    }

    [Theory]
    [InlineData(GroundProbeMode.Ray)]
    [InlineData(GroundProbeMode.SweptSphere)]
    public void CheckGround_WhenProbeStartsInsideSupport_ShouldRejectNonUpwardWitness(GroundProbeMode mode)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        SolidBody body = CreateProbe(scenario, mode);
        LSCuboidCollider support = AddBox(scenario, (Fixed64)4);

        body.GroundMinNormalDot = Fixed64.Zero;
        scenario.Context.Diagnostics.Enable();

        body.CheckGround();

        body.IsGrounded.Should().BeFalse();
        body.HasHitPoint.Should().BeFalse();
        scenario.Context.Diagnostics.Events[0].Hit.Should().BeTrue();
        scenario.Context.Diagnostics.Events[0].Vector.Y.Should().BeLessThanOrEqualTo(Fixed64.Zero);
        scenario.Context.Diagnostics.Events[1].Hit.Should().BeFalse();
    }

    [Theory]
    [InlineData(GroundProbeMode.Ray)]
    [InlineData(GroundProbeMode.SweptSphere)]
    public void CheckGround_WhenNearestWitnessHasNoUpwardSupport_ShouldSelectFartherFloor(GroundProbeMode mode)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        SolidBody body = CreateProbe(scenario, mode);
        LSCuboidCollider wall = AddBox(scenario, (Fixed64)4);
        LSCuboidCollider floor = AddBox(scenario, Fixed64.Zero);

        AssertSelectionMatchesAllHits(scenario, body, wall, floor, 2);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void GroundMinNormalDot_RuntimeChange_ShouldRefreshCachedSupportOnNextStep(bool sleeping)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        SolidBody body = CreateProbe(scenario, GroundProbeMode.Ray);
        var slope = new LSCuboidCollider { Layer = new PhysicsLayer(1), Size = new Vector3d(8, 1, 8) };
        scenario.CreateBody(slope, Vector3d.Zero,
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)45), immovable: true);
        body.CheckGround();
        body.IsGrounded.Should().BeTrue();
        if (sleeping)
            body.Sleep();
        var before = scenario.Context.ComputeReplayHash();
        body.GroundMinNormalDot = Fixed64.Half;
        scenario.Context.ComputeReplayHash().Should().Be(before);
        body.IsSleeping.Should().Be(sleeping);

        body.GroundMinNormalDot = Fixed64.One;

        body.IsSleeping.Should().BeFalse();
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();
        body.IsGrounded.Should().BeFalse();
        body.HasHitPoint.Should().BeFalse();
    }

    [Fact]
    public void GroundMinNormalDot_RuntimeChange_ShouldPreserveSleepingManualSupport()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        SolidBody body = scenario.CreateSphere(Vector3d.Zero).Body;
        body.SetManualGrounding(Vector3d.Zero, Vector3d.Right);
        body.Sleep();

        body.GroundMinNormalDot = Fixed64.One;
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        body.IsSleeping.Should().BeTrue();
        body.IsGrounded.Should().BeTrue();
        body.GroundNormal.Should().Be(Vector3d.Right);
    }

    [Theory]
    [InlineData(GroundProbeMode.Ray)]
    [InlineData(GroundProbeMode.SweptSphere)]
    public void CheckGround_ShouldApplyInclusiveConfigurableSlopeThreshold(GroundProbeMode mode)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        SolidBody body = CreateProbe(scenario, mode);
        var slope = new LSCuboidCollider { Layer = new PhysicsLayer(1), Size = new Vector3d(8, 1, 8) };
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)45);
        scenario.CreateBody(slope, Vector3d.Zero, rotation, immovable: true);
        Physics3DHit selected = AssertSelectionMatchesAllHits(scenario, body, slope, slope, 1);
        Fixed64 upDot = selected.Normal.Normalized.Y;
        upDot.Should().BeGreaterThan(Fixed64.Half).And.BeLessThan(Fixed64.One);
        body.GroundMinNormalDot = upDot;
        body.CheckGround();
        body.IsGrounded.Should().BeTrue();
        body.GroundMinNormalDot = upDot + Fixed64.FromRaw(1);
        body.CheckGround();
        body.IsGrounded.Should().BeFalse();
        body.WasGrounded.Should().BeTrue();
        body.HasHitPoint.Should().BeFalse();
        body.HitPlatform.Should().BeNull();
        body.GroundMinNormalDot = Fixed64.Zero;
        body.CheckGround();
        body.IsGrounded.Should().BeTrue();
    }

    [Theory]
    [InlineData(GroundProbeMode.Ray)]
    [InlineData(GroundProbeMode.SweptSphere)]
    public void CheckGround_ShouldReduceCompoundToOneOwnerAndPreserveMeshCounters(GroundProbeMode mode)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        SolidBody body = CreateProbe(scenario, mode);
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero),
            CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Down))
        { Layer = new PhysicsLayer(1) };
        scenario.InitializeStaticCollider(compound, new Vector3d(0, 2, 0));
        LSMeshCollider mesh = MeshTestFixtures.CreateConvexQuadFloor();
        mesh.Layer = new PhysicsLayer(2);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);

        AssertSelectionMatchesAllHits(scenario, body, compound, compound, 2);
        if (mode == GroundProbeMode.SweptSphere)
            scenario.Context.Query3D.LastMeshTriangleCandidateCount.Should().BeGreaterThan(0);
    }

    [Theory]
    [InlineData(GroundProbeMode.Ray, 0)]
    [InlineData(GroundProbeMode.Ray, 1)]
    [InlineData(GroundProbeMode.Ray, 2)]
    [InlineData(GroundProbeMode.Ray, 3)]
    [InlineData(GroundProbeMode.SweptSphere, 0)]
    [InlineData(GroundProbeMode.SweptSphere, 1)]
    [InlineData(GroundProbeMode.SweptSphere, 2)]
    [InlineData(GroundProbeMode.SweptSphere, 3)]
    public void CheckGround_WithTinyTravel_ShouldPreserveQueryAdmissionAndEventOrder(GroundProbeMode mode, int distanceCase)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        SolidBody body = CreateProbe(scenario, mode, Vector3d.Zero);
        LSCuboidCollider support = AddBox(scenario, -Fixed64.Half);
        Fixed64 distance = distanceCase switch
        {
            0 => Fixed64.Zero,
            1 => Fixed64.FromRaw(1),
            2 => Fixed64.Epsilon,
            _ => Fixed64.Epsilon + Fixed64.FromRaw(1)
        };
        body.GroundedDistanceRay = body.GroundDownDistanceOnAir = distance;
        bool admitted = mode == GroundProbeMode.Ray ? distanceCase != 0 : distanceCase == 3;
        scenario.Context.Diagnostics.Enable();

        body.CheckGround();

        body.IsGrounded.Should().Be(admitted);
        body.HasHitPoint.Should().Be(admitted);
        scenario.Context.Diagnostics.Events.Length.Should().Be(admitted ? 2 : 1);
        GravitasDiagnosticEvent probe = scenario.Context.Diagnostics.Events[admitted ? 1 : 0];
        probe.Kind.Should().Be(GravitasDiagnosticEventKind.GroundProbe);
        probe.Hit.Should().Be(admitted);
        probe.ColliderBId.Should().Be(admitted ? support.Id : -1);
        if (admitted)
            scenario.Context.Diagnostics.Events[0].Kind.Should().Be(GravitasDiagnosticEventKind.RayQuery);
        scenario.Context.Query3D.LastQueryCandidateCount.Should().Be(
            admitted && mode == GroundProbeMode.SweptSphere ? 1 : 0);
    }

    [Theory]
    [InlineData(GroundProbeMode.Ray)]
    [InlineData(GroundProbeMode.SweptSphere)]
    public void CheckGround_ShouldPublishBothDiagnosticEventsAndWitnessBeforeGroundedCallback(GroundProbeMode mode)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        SolidBody body = CreateProbe(scenario, mode);
        LSCuboidCollider support = AddBox(scenario, Fixed64.Zero);
        int callbacks = 0;
        scenario.Context.Diagnostics.Enable();
        body.OnGrounded = grounded =>
        {
            callbacks++;
            grounded.Should().BeTrue();
            body.HitPlatform.Should().BeSameAs(support.Transform);
            body.HasHitPoint.Should().BeTrue();
            scenario.Context.Diagnostics.Events.Length.Should().Be(2);
            scenario.Context.Diagnostics.Events[0].Kind.Should().Be(GravitasDiagnosticEventKind.RayQuery);
            scenario.Context.Diagnostics.Events[1].Kind.Should().Be(GravitasDiagnosticEventKind.GroundProbe);
            scenario.Context.Diagnostics.Events[1].PointA.Should().Be(body.HitPoint);
        };

        body.CheckGround();

        callbacks.Should().Be(1);
        body.OnGrounded = null;
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CheckGround_WithTinySphereRadius_ShouldUseRayAndKeepRequestedProbeDiagnostics(bool atThreshold)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        SolidBody body = CreateProbe(scenario, GroundProbeMode.SweptSphere);
        body.GroundProbeRadius = atThreshold ? Fixed64.Epsilon : Fixed64.FromRaw(1);
        LSCuboidCollider support = AddBox(scenario, Fixed64.Zero);

        AssertSelectionMatchesAllHits(scenario, body, support, support, 1);

        scenario.Context.Diagnostics.Events[0].ScalarA.Should().Be(Fixed64.Zero);
        GravitasDiagnosticEvent probe = scenario.Context.Diagnostics.Events[1];
        probe.ScalarA.Should().Be(body.GroundProbeRadius);
        probe.DataA.Should().Be((int)GroundProbeMode.SweptSphere);
        scenario.Context.Query3D.LastQueryCandidateCount.Should().Be(0);
    }

    private static SolidBody CreateProbe(PhysicsScenarioBuilder scenario, GroundProbeMode mode, Vector3d? position = null)
    {
        scenario.Context.Settings.GroundCheckLayerMask = new PhysicsLayerMask((1 << 1) | (1 << 2));
        SolidBody body = scenario.CreateBody(new LSSphereCollider { Layer = new PhysicsLayer(0) },
            position ?? new Vector3d(0, 4, 0), FixedQuaternion.Identity).Body;
        body.GroundProbeMode = mode;
        body.GroundProbeRadius = Fixed64.Half;
        body.GroundOriginOffset = Fixed64.Zero;
        body.GroundedDistanceRay = body.GroundDownDistanceOnAir = (Fixed64)4;
        return body;
    }

    private static LSCuboidCollider AddBox(PhysicsScenarioBuilder scenario, Fixed64 y, int layer = 1)
    {
        var collider = new LSCuboidCollider { Layer = new PhysicsLayer(layer), Size = Vector3d.One };
        scenario.InitializeStaticCollider(collider, new Vector3d(Fixed64.Zero, y, Fixed64.Zero));
        return collider;
    }

    private static Physics3DHit AssertSelectionMatchesAllHits(PhysicsScenarioBuilder scenario, SolidBody body,
        LSCollider rawNearest, LSCollider support, int expectedRawCount)
    {
        var hits = new SwiftList<Physics3DHit>();
        GravitasWorldContext context = scenario.Context;
        Vector3d start = body.Position3d;
        Vector3d end = start + Vector3d.Down * body.GroundDownDistanceOnAir;
        context.Diagnostics.Enable();
        if (body.GroundProbeMode == GroundProbeMode.Ray || body.GroundProbeRadius <= Fixed64.Epsilon)
            context.Query3D.RaycastAll(start, end, context.Settings.GroundCheckLayerMask, hits);
        else
            context.Query3D.SweepSphereAll(start, end, body.GroundProbeRadius,
                context.Settings.GroundCheckLayerMask, hits, body.Collider);
        hits.Count.Should().Be(expectedRawCount);
        hits[0].Collider.Should().BeSameAs(rawNearest);
        Physics3DHit expected = default;
        for (int i = 0; i < hits.Count; i++)
            if (ReferenceEquals(hits[i].Collider, support))
                expected = hits[i];
        expected.Collider.Should().BeSameAs(support);
        expected.TryGetPoint(out Vector3d expectedPoint).Should().BeTrue();
        GravitasDiagnosticEvent rawEvent = context.Diagnostics.Events[0];
        int candidateCount = context.Query3D.LastQueryCandidateCount;
        int triangleCount = context.Query3D.LastMeshTriangleCandidateCount;
        context.Diagnostics.Clear();

        body.CheckGround();

        body.IsGrounded.Should().BeTrue();
        body.HitPlatform.Should().BeSameAs(support.Transform);
        body.HitPoint.Should().Be(expectedPoint);
        body.GroundNormal.Should().Be(expected.Normal);
        context.Diagnostics.Events.Length.Should().Be(2);
        // Raw query diagnostics intentionally describe all intersections, not
        // only physically eligible supports or the accepted ground witness.
        context.Diagnostics.Events[0].Should().BeEquivalentTo(rawEvent);
        GravitasDiagnosticEvent probe = context.Diagnostics.Events[1];
        probe.Kind.Should().Be(GravitasDiagnosticEventKind.GroundProbe);
        probe.ColliderBId.Should().Be(support.Id);
        probe.PointA.Should().Be(expectedPoint);
        probe.Vector.Should().Be(expected.Normal);
        probe.ScalarB.Should().Be(expected.Distance);
        context.Query3D.LastQueryCandidateCount.Should().Be(candidateCount);
        context.Query3D.LastMeshTriangleCandidateCount.Should().Be(triangleCount);
        return expected;
    }
}
