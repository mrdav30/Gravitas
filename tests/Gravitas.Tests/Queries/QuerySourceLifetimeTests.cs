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

public sealed class QuerySourceLifetimeTests
{
    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    public void CompletedSphereQuery_ShouldReleaseItsExcludedCollider(int kind)
    {
        var retired = RetireSphereExclusion(kind);
        Collect();
        Assert.False(retired.Excluded.IsAlive);
        GC.KeepAlive(retired.Query);
    }

    [Theory]
    [InlineData(false, false, false)]
    [InlineData(false, false, true)]
    [InlineData(true, false, false)]
    [InlineData(true, false, true)]
    [InlineData(false, true, false)]
    [InlineData(false, true, true)]
    [InlineData(true, true, false)]
    [InlineData(true, true, true)]
    public void CompletedSourceSweep_ShouldReleaseSourceAndExclusion(bool all, bool batch, bool stationary)
    {
        var retired = RetireSourceSweep(all, batch, stationary, failSecondRequest: false);
        Collect();
        Assert.False(retired.Source.IsAlive);
        Assert.False(retired.Excluded.IsAlive);
        GC.KeepAlive(retired.Query);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FailedBatch_ShouldReleaseThePreviouslyPreparedSourceAndExclusion(bool all)
    {
        var retired = RetireSourceSweep(all, batch: true, stationary: false, failSecondRequest: true);
        Collect();
        Assert.False(retired.Source.IsAlive);
        Assert.False(retired.Excluded.IsAlive);
        GC.KeepAlive(retired.Query);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static (GravitasQuery3DService Query, WeakReference Excluded) RetireSphereExclusion(int kind)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        LSCuboidCollider excluded = Collider(scene.Context, 2);
        GravitasQuery3DService query = scene.Context.Query3D;
        if (kind == 0)
            Assert.Equal(0, query.SweepSphereAll(Vector3d.Zero, Vector3d.Forward * 4, Fixed64.Quarter,
                PhysicsLayerMask.All, new SwiftList<Physics3DHit>(), excluded));
        else if (kind == 1)
            Assert.False(query.SweepSphere(Vector3d.Zero, Fixed64.Quarter, Vector3d.Forward, (Fixed64)4,
                out _, PhysicsLayerMask.All, excluded));
        else if (kind == 2)
            Assert.Equal(0, query.OverlapSphereAgainstStaticAll(excluded.Center, Fixed64.Quarter,
                PhysicsLayerMask.All, new SwiftList<Physics3DHit>(), excluded));
        else
        {
            PhysicsSweepSphere3DRequest[] requests = { new(Vector3d.Zero, Vector3d.Forward * 4,
                Fixed64.Quarter, PhysicsLayerMask.All, excluded) };
            if (kind == 3) Assert.Equal(0, query.SweepSphereBatch(requests, new Physics3DHit[1]));
            else Assert.Equal(0, query.SweepSphereAllBatch(requests, new SwiftList<Physics3DHit>(), new PhysicsQueryHitRange[1]));
        }
        excluded.Deactivate();
        return (query, new(excluded));
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static (GravitasQuery3DService Query, WeakReference Source, WeakReference Excluded) RetireSourceSweep(
        bool all, bool batch, bool stationary, bool failSecondRequest)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        LSCuboidCollider source = Collider(scene.Context, 0), excluded = Collider(scene.Context, 2), target = Collider(scene.Context, 3);
        GravitasQuery3DService query = scene.Context.Query3D;
        Vector3d displacement = stationary ? Vector3d.Zero : Vector3d.Forward * 4;
        var hits = new SwiftList<Physics3DHit>();
        if (batch)
        {
            var request = new PhysicsSweepCuboid3DRequest(source, displacement, PhysicsLayerMask.All, excluded);
            PhysicsSweepCuboid3DRequest[] requests = { request, failSecondRequest ? default : request };
            var closest = new Physics3DHit[requests.Length];
            var ranges = new PhysicsQueryHitRange[requests.Length];
            if (failSecondRequest)
            {
                if (all) Assert.Throws<ArgumentNullException>(() => query.SweepCuboidAllBatch(requests, hits, ranges));
                else Assert.Throws<ArgumentNullException>(() => query.SweepCuboidBatch(requests, closest));
            }
            else if (all)
            {
                Assert.Equal(stationary ? 0 : 2, query.SweepCuboidAllBatch(requests, hits, ranges));
                if (!stationary) { Assert.Same(target, hits[0].Collider); Assert.Same(target, hits[1].Collider); }
            }
            else
            {
                Assert.Equal(stationary ? 0 : 2, query.SweepCuboidBatch(requests, closest));
                if (!stationary) { Assert.Same(target, closest[0].Collider); Assert.Same(target, closest[1].Collider); }
            }
        }
        else if (all)
        {
            Assert.Equal(stationary ? 0 : 1, query.SweepCuboidAll(source, displacement, PhysicsLayerMask.All, hits, excluded));
            if (!stationary) Assert.Same(target, hits[0].Collider);
        }
        else
        {
            Assert.Equal(!stationary, query.SweepCuboid(source, displacement, PhysicsLayerMask.All, out Physics3DHit hit, excluded));
            if (!stationary) Assert.Same(target, hit.Collider);
        }
        source.Deactivate(); excluded.Deactivate(); target.Deactivate();
        return (query, new(source), new(excluded));
    }

    private static LSCuboidCollider Collider(GravitasWorldContext context, int z)
    {
        var collider = new LSCuboidCollider { Size = Vector3d.One };
        collider.InitializeWithNoBody(new TestMatterAgent(context, new FixedTransform(Vector3d.Forward * z,
            FixedQuaternion.Identity, Vector3d.One)));
        return collider;
    }

    private static void Collect()
    {
        GC.Collect(); GC.WaitForPendingFinalizers(); GC.Collect();
    }
}
