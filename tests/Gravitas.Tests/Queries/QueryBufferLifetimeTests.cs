using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using System;
using System.Runtime.CompilerServices;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class QueryBufferLifetimeTests
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
    [InlineData(3, false, false)]
    [InlineData(3, false, true)]
    [InlineData(3, true, false)]
    [InlineData(3, true, true)]
    public void ReplacingCallerResults_ShouldReleaseOldTailAndPreserveOnlyReportedHits(int path, bool batch, bool empty)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        var retired = RetireReplacedCallerResults(scene.Context, path, batch, empty);
        Collect();
        for (int index = 0; index < retired.Targets.Length; index++)
            Assert.Equal(!empty && index == 0, retired.Targets[index].IsAlive);
        retired.ClearCallerHits();
        Collect();
        foreach (WeakReference target in retired.Targets) Assert.False(target.IsAlive);
        GC.KeepAlive(retired.ClearCallerHits);
        GC.KeepAlive(scene.Context);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static (WeakReference[] Targets, Action ClearCallerHits) RetireReplacedCallerResults(
        GravitasWorldContext context, int path, bool batch, bool empty)
    {
        object[] targets = Targets(context, twoD: path is 0 or 2);
        var ranges = new PhysicsQueryHitRange[1];
        int expected = empty ? 0 : 1, endpoint = empty ? 1 : 3;
        Action clear;
        if (path == 0)
        {
            var hits = new SwiftList<Physics2DHit>();
            int Fill(int end) => batch
                ? context.Query2D.RaycastAllBatch(new[] { new PhysicsRaycast2DRequest(Vector2d.Zero, new(0, end)) }, hits, ranges)
                : context.Query2D.RaycastAll(Vector2d.Zero, new(0, end), hits);
            Assert.Equal(3, Fill(8)); Assert.Equal(expected, Fill(endpoint)); Assert.Equal(expected, hits.Count);
            if (!empty) Assert.Same(targets[0], hits[0].Collider);
            clear = hits.Clear;
        }
        else if (path == 1)
        {
            var hits = new SwiftList<Physics3DHit>();
            int Fill(int end) => batch
                ? context.Query3D.RaycastAllBatch(new[] { new PhysicsRaycast3DRequest(Vector3d.Zero, Vector3d.Forward * end) }, hits, ranges)
                : context.Query3D.RaycastAll(Vector3d.Zero, Vector3d.Forward * end, PhysicsLayerMask.All, hits);
            Assert.Equal(3, Fill(8)); Assert.Equal(expected, Fill(endpoint)); Assert.Equal(expected, hits.Count);
            if (!empty) Assert.Same(targets[0], hits[0].Collider);
            clear = hits.Clear;
        }
        else
        {
            var hits = new SwiftList<PhysicsMixedHit>();
            int Fill(int end)
            {
                if (path == 2)
                    return batch ? context.QueryMixed.SweepSphereAgainst2DAllBatch(new[] {
                        new PhysicsSweepSphereAgainst2DRequest(Vector3d.Zero, Vector3d.Forward * end, Fixed64.Quarter) }, hits, ranges)
                        : context.QueryMixed.SweepSphereAgainst2DAll(Vector3d.Zero, Vector3d.Forward * end,
                            Fixed64.Quarter, PhysicsLayerMask.All, hits);
                return batch ? context.QueryMixed.SweepCircleAgainst3DAllBatch(new[] {
                    new PhysicsSweepCircleAgainst3DRequest(Vector2d.Zero, new(0, end), Fixed64.Quarter, Fixed64.Zero, Fixed64.Half) }, hits, ranges)
                    : context.QueryMixed.SweepCircleAgainst3DAll(Vector2d.Zero, new(0, end), Fixed64.Quarter,
                        Fixed64.Zero, Fixed64.Half, PhysicsLayerMask.All, hits);
            }
            Assert.Equal(3, Fill(8)); Assert.Equal(expected, Fill(endpoint)); Assert.Equal(expected, hits.Count);
            if (!empty) Assert.Same(targets[0], path == 2 ? hits[0].Collider2D : (object?)hits[0].Collider3D);
            clear = hits.Clear;
        }
        if (batch) { Assert.Equal(0, ranges[0].Start); Assert.Equal(expected, ranges[0].Count); }
        // The live caller buffer owns the one reported target, if any. Its
        // overwritten tail must not keep the other retired host objects alive.
        return (Retire(targets), clear);
    }

    [Theory]
    [InlineData(0, 0)]
    [InlineData(0, 1)]
    [InlineData(0, 2)]
    [InlineData(0, 3)]
    [InlineData(1, 0)]
    [InlineData(1, 1)]
    [InlineData(1, 2)]
    [InlineData(1, 3)]
    [InlineData(2, 0)]
    [InlineData(2, 1)]
    [InlineData(2, 2)]
    [InlineData(2, 3)]
    public void CandidateQueries_ShouldReleaseRetiredTargetsWhileTheirServiceRemainsAlive(int path, int completion)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        WeakReference[] retired = RetireCandidateTargets(scene.Context, path, completion);
        Collect();
        foreach (WeakReference target in retired) Assert.False(target.IsAlive);
        GC.KeepAlive(scene.Context);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void AllHitBatchAfterShrinkingRequests_ShouldRetainOnlyCallerOwnedHits(int path)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        var retired = RetireBatchTargets(scene.Context, path, failSecondRequest: false);
        Collect();
        foreach (WeakReference target in retired.Targets) Assert.True(target.IsAlive);
        retired.ClearCallerHits();
        Collect();
        foreach (WeakReference target in retired.Targets) Assert.False(target.IsAlive);
        GC.KeepAlive(retired.ClearCallerHits);
        GC.KeepAlive(scene.Context);
    }

    [Fact]
    public void FailedSecondBatchRequest_ShouldReleaseInternalHitsAndPreserveCompletedCallerResults()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var retired = RetireBatchTargets(scene.Context, 1, failSecondRequest: true);
        Collect();
        foreach (WeakReference target in retired.Targets) Assert.True(target.IsAlive);
        retired.ClearCallerHits();
        Collect();
        foreach (WeakReference target in retired.Targets) Assert.False(target.IsAlive);
        GC.KeepAlive(retired.ClearCallerHits);
        GC.KeepAlive(scene.Context);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static WeakReference[] RetireCandidateTargets(GravitasWorldContext context, int path, int completion)
    {
        object[] targets = Targets(context, twoD: path != 2);
        int steps = completion == 0 ? 1 : 3;
        for (int step = 0; step < steps; step++)
        {
            int x = step == 2 ? 10 : 0, end = step == 1 ? 3 : 8;
            Vector2d start2D = new((Fixed64)x, Fixed64.Zero), end2D = new((Fixed64)x, (Fixed64)end);
            bool found;
            int candidates;
            if (path == 0)
            {
                found = context.Query2D.Raycast(start2D, end2D, out _);
                candidates = context.Query2D.LastQueryCandidateCount;
            }
            else if (path == 1)
            {
                found = context.QueryMixed.SweepSphereAgainst2D(new(x, 0, 0), new(x, 0, end),
                    Fixed64.Quarter, PhysicsLayerMask.All, out _);
                candidates = context.QueryMixed.LastQueryCandidateCount;
            }
            else
            {
                found = context.QueryMixed.SweepCircleAgainst3D(start2D, end2D, Fixed64.Quarter,
                    Fixed64.Zero, Fixed64.Half, PhysicsLayerMask.All, out _);
                candidates = context.QueryMixed.LastQueryCandidateCount;
            }
            Assert.Equal(step != 2, found);
            Assert.Equal(step == 0 ? 3 : step == 1 ? 1 : 0, candidates);
        }
        WeakReference[] retired = Retire(targets);
        // Completion 0 checks the last query itself; 1 checks a smaller then
        // empty query; 2/3 additionally check explicit reset/disposal lifetimes.
        if (completion == 2)
        {
            context.Query2D.Reset(); context.QueryMixed.Reset();
        }
        if (completion == 3) context.Dispose();
        return retired;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static (WeakReference[] Targets, Action ClearCallerHits) RetireBatchTargets(
        GravitasWorldContext context, int path, bool failSecondRequest)
    {
        object[] targets = Targets(context, twoD: path is 0 or 2);
        var ranges = new PhysicsQueryHitRange[3];
        Action clear;
        int count, requests, reportedHits;
        if (path == 0)
        {
            var hits = new SwiftList<Physics2DHit>();
            PhysicsRaycast2DRequest[] batch = { new(new(0, 0), new(0, 8)),
                new(new(0, 0), new(0, 3)), new(new(10, 0), new(10, 8)) };
            count = context.Query2D.RaycastAllBatch(batch, hits, ranges);
            requests = context.Query2D.LastBatchRequestCount; reportedHits = context.Query2D.LastBatchHitCount;
            Assert.True(context.Query2D.LastBatchCandidateCount >= 4);
            AssertCallerHits(hits.Count, index => hits[index].Collider, targets);
            clear = hits.Clear;
        }
        else if (path == 1)
        {
            var hits = new SwiftList<Physics3DHit>();
            if (failSecondRequest)
            {
                PhysicsOverlapCone3DRequest[] batch = { new(Vector3d.Zero, Vector3d.Forward, (Fixed64)8, Fixed64.Two),
                    new(Vector3d.Zero, Vector3d.Zero, (Fixed64)8, Fixed64.Two) };
                Assert.Throws<ArgumentException>(() => context.Query3D.OverlapConeAllBatch(batch, hits, ranges));
                count = hits.Count;
            }
            else
            {
                PhysicsRaycast3DRequest[] batch = { new(Vector3d.Zero, Vector3d.Forward * 8),
                    new(Vector3d.Zero, Vector3d.Forward * 3), new(new(10, 0, 0), new(10, 0, 8)) };
                count = context.Query3D.RaycastAllBatch(batch, hits, ranges);
            }
            requests = context.Query3D.LastBatchRequestCount; reportedHits = context.Query3D.LastBatchHitCount;
            // Ray queries do not count candidates; cone reducers count each
            // eligible target once. Cleanup must preserve that existing metric.
            Assert.Equal(failSecondRequest ? 3 : 0, context.Query3D.LastBatchCandidateCount);
            AssertCallerHits(hits.Count, index => hits[index].Collider, targets);
            clear = hits.Clear;
        }
        else
        {
            var hits = new SwiftList<PhysicsMixedHit>();
            if (path == 2)
            {
                PhysicsSweepSphereAgainst2DRequest[] batch = {
                    new(Vector3d.Zero, Vector3d.Forward * 8, Fixed64.Quarter),
                    new(Vector3d.Zero, Vector3d.Forward * 3, Fixed64.Quarter),
                    new(new(10, 0, 0), new(10, 0, 8), Fixed64.Quarter) };
                count = context.QueryMixed.SweepSphereAgainst2DAllBatch(batch, hits, ranges);
            }
            else
            {
                PhysicsSweepCircleAgainst3DRequest[] batch = {
                    new(new(0, 0), new(0, 8), Fixed64.Quarter, Fixed64.Zero, Fixed64.Half),
                    new(new(0, 0), new(0, 3), Fixed64.Quarter, Fixed64.Zero, Fixed64.Half),
                    new(new(10, 0), new(10, 8), Fixed64.Quarter, Fixed64.Zero, Fixed64.Half) };
                count = context.QueryMixed.SweepCircleAgainst3DAllBatch(batch, hits, ranges);
            }
            requests = context.QueryMixed.LastBatchRequestCount; reportedHits = context.QueryMixed.LastBatchHitCount;
            Assert.True(context.QueryMixed.LastBatchCandidateCount >= 4);
            AssertCallerHits(hits.Count, index => path == 2 ? hits[index].Collider2D : (object?)hits[index].Collider3D, targets);
            clear = hits.Clear;
        }
        Assert.Equal(failSecondRequest ? 2 : 3, requests);
        Assert.Equal(failSecondRequest ? 3 : 4, count); Assert.Equal(count, reportedHits);
        Assert.Equal(0, ranges[0].Start); Assert.Equal(3, ranges[0].Count);
        if (!failSecondRequest)
        {
            Assert.Equal(3, ranges[1].Start); Assert.Equal(1, ranges[1].Count);
            Assert.Equal(4, ranges[2].Start); Assert.Equal(0, ranges[2].Count);
        }
        return (Retire(targets), clear);
    }

    private static void AssertCallerHits(int count, Func<int, object?> getCollider, object[] targets)
    {
        Assert.InRange(count, 3, 4);
        for (int index = 0; index < count; index++) Assert.Same(targets[index % 3], getCollider(index));
    }

    private static object[] Targets(GravitasWorldContext context, bool twoD)
    {
        var targets = new object[3];
        for (int index = 0; index < targets.Length; index++)
        {
            var agent = new TestMatterAgent(context, new FixedTransform(Vector3d.Forward * (2 * index + 2),
                FixedQuaternion.Identity, Vector3d.One));
            if (twoD)
            {
                var collider = new LSAABBoxCollider2D(Vector2d.One) { MixedHalfThicknessOverride = Fixed64.Half };
                collider.InitializeWithNoBody(agent); targets[index] = collider;
            }
            else
            {
                var collider = new LSCuboidCollider { Size = Vector3d.One };
                collider.InitializeWithNoBody(agent); targets[index] = collider;
            }
        }
        return targets;
    }

    private static WeakReference[] Retire(object[] targets)
    {
        var retired = new WeakReference[targets.Length];
        for (int index = 0; index < targets.Length; index++)
        {
            retired[index] = new(targets[index]);
            if (targets[index] is LSCollider2D planar) planar.Deactivate();
            else ((LSCollider)targets[index]).Deactivate();
        }
        return retired;
    }

    private static void Collect()
    {
        GC.Collect(); GC.WaitForPendingFinalizers(); GC.Collect();
    }
}
