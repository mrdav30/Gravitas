using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Xunit;

namespace Gravitas.Tests.Runtime;

public sealed class ContextResetLifetimeTests
{
    [Theory]
    [InlineData(PhysicsRuntimeMode.ThreeD, 0)]
    [InlineData(PhysicsRuntimeMode.TwoD, 0)]
    [InlineData(PhysicsRuntimeMode.Both, 0)]
    [InlineData(PhysicsRuntimeMode.Mixed, 0)]
    [InlineData(PhysicsRuntimeMode.ThreeD, 1)]
    [InlineData(PhysicsRuntimeMode.TwoD, 1)]
    [InlineData(PhysicsRuntimeMode.Both, 1)]
    [InlineData(PhysicsRuntimeMode.Mixed, 1)]
    [InlineData(PhysicsRuntimeMode.ThreeD, 2)]
    [InlineData(PhysicsRuntimeMode.TwoD, 2)]
    [InlineData(PhysicsRuntimeMode.Both, 2)]
    [InlineData(PhysicsRuntimeMode.Mixed, 2)]
    public void Reset_ReleasesEndedGraphsAndSupportsReuse(PhysicsRuntimeMode mode, int exercise)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, mode);
        scene.Context.Reset();
        long emptyStart = GC.GetAllocatedBytesForCurrentThread();
        scene.Context.Reset();
        long emptyResetBytes = GC.GetAllocatedBytesForCurrentThread() - emptyStart;
        var emptyHash = scene.Context.ComputeReplayHash();
        WeakReference[] retired = Populate(scene.Context, exercise);
        Collect();
        Assert.True(retired[0].IsAlive);

        scene.Context.Reset();
        Assert.Equal(0, scene.Context.Physics.BodyCount);
        Assert.Equal(0, scene.Context.Physics.ColliderCount);
        Assert.Equal(0, scene.Context.Physics2D.BodyCount);
        Assert.Equal(0, scene.Context.Physics2D.ColliderCount);
        Assert.Equal(0, scene.Context.MixedCollisions.ActivePairCount);
        Assert.Equal(emptyHash, scene.Context.ComputeReplayHash());
        Collect();
        foreach (WeakReference target in retired) Assert.False(target.IsAlive);

        WeakReference[] reused = Populate(scene.Context, exercise);
        long before = GC.GetAllocatedBytesForCurrentThread();
        scene.Context.Reset();
        long resetBytes = GC.GetAllocatedBytesForCurrentThread() - before;
        // Reset creates a clock lifetime token; releasing populated buffers adds no allocation.
        Assert.Equal(emptyResetBytes, resetBytes);
        Assert.Equal(emptyHash, scene.Context.ComputeReplayHash());
        Collect();
        foreach (WeakReference target in reused) Assert.False(target.IsAlive);
        GC.KeepAlive(scene.Context);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void Reset_ReleasesMixedGraphsWhileSurfaceHandleIsRetained(bool capsule, bool disposeSurface)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, PhysicsRuntimeMode.Mixed);
        var retired = PopulateSurface(scene.Context, capsule);
        if (disposeSurface) retired.Surface.Dispose();
        scene.Context.Reset();
        Collect();
        foreach (WeakReference target in retired.Targets) Assert.False(target.IsAlive);
        GC.KeepAlive(retired.Surface);
        GC.KeepAlive(scene.Context);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static (StaticSurface2D Surface, WeakReference[] Targets) PopulateSurface(GravitasWorldContext context, bool capsule)
    {
        var left = new LSAABBoxCollider2D(new Vector2d(2, 2));
        var right = new LSAABBoxCollider2D(new Vector2d(2, 2));
        left.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(new Vector3d(-1, 0, -1), FixedQuaternion.Identity, Vector3d.One)));
        right.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(new Vector3d(1, 0, -1), FixedQuaternion.Identity, Vector3d.One)));
        StaticSurface2D surface = context.Physics2D.CreateStaticSurface(new LSCollider2D[] { left, right });
        LSCollider collider = capsule ? new LSCapsuleCollider { Radius = Fixed64.Half, Size = Vector3d.One }
            : new LSSphereCollider();
        var agent = new TestMatterAgent(context);
        var body = new SolidBody(agent, collider) { Mass = Fixed64.One, FreezeAxes = BodyFreezeAxes3D.Rotation, SleepEnabled = false };
        body.Initialize(new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Quarter), FixedQuaternion.Identity);
        body.UseManualGrounding();
        context.Simulate(); context.LateSimulate();
        Assert.True(context.MixedCollisions.ActivePairCount > 0);
        return (surface, new[] { new WeakReference(left), new WeakReference(right), new WeakReference(collider),
            new WeakReference(body), new WeakReference(agent) });
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Reset_ReleasesCompletedContinuousHandoffGraphs(bool twoD)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, twoD ? PhysicsRuntimeMode.TwoD : PhysicsRuntimeMode.ThreeD);
        scene.Context.SetFrameRate(1);
        WeakReference[] retired = PopulateContinuous(scene.Context, twoD);
        scene.Context.Reset();
        Collect();
        foreach (WeakReference target in retired) Assert.False(target.IsAlive);
        GC.KeepAlive(scene.Context);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static WeakReference[] PopulateContinuous(GravitasWorldContext context, bool twoD)
    {
        var targets = new List<WeakReference>();
        foreach (int x in new[] { 0, 2, -5 })
        {
            var agent = new TestMatterAgent(context);
            if (twoD)
            {
                var collider = new LSCircleCollider2D(Fixed64.Half);
                var body = new SolidBody2D(agent, collider)
                { Mass = Fixed64.One, ContinuousCollisionMode = ContinuousCollisionMode.Continuous };
                body.Initialize(new Vector2d(x, 0));
                if (x == -5) body.AddLinearImpulse(Vector2d.Right * (Fixed64)10);
                targets.Add(new WeakReference(body)); targets.Add(new WeakReference(collider));
            }
            else
            {
                var collider = new LSSphereCollider();
                var body = new SolidBody(agent, collider)
                { Mass = Fixed64.One, ContinuousCollisionMode = ContinuousCollisionMode.Continuous };
                body.Initialize(new Vector3d(x, 0, 0), FixedQuaternion.Identity);
                body.UseManualGrounding();
                if (x == -5) body.AddLinearImpulse(Vector3d.Right * (Fixed64)10);
                targets.Add(new WeakReference(body)); targets.Add(new WeakReference(collider));
            }
            targets.Add(new WeakReference(agent));
        }
        context.Simulate(); context.LateSimulate();
        Assert.True((twoD ? context.Physics2D.LastContinuousCollisionIslandCount
            : context.Physics.LastContinuousCollisionIslandCount) > 0);
        // A later empty queue must not hide references left by completed handoffs.
        context.Simulate(); context.LateSimulate();
        return targets.ToArray();
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static WeakReference[] Populate(GravitasWorldContext context, int exercise)
    {
        var targets = new List<WeakReference>();
        var deactivate = new List<Action>();
        int entered = 0;
        int count = exercise == 2 ? 5 : 1;
        for (int i = 0; i < count; i++)
        {
            Fixed64 x = (Fixed64)(i * 3 - 6);
            if (context.Settings.RuntimeMode != PhysicsRuntimeMode.TwoD)
            {
                var agent = new TestMatterAgent(context);
                var collider = new LSSphereCollider { Material = PhysicsMaterial.Frictionless };
                var body = new SolidBody(agent, collider)
                { Mass = Fixed64.One, FreezeAxes = BodyFreezeAxes3D.Rotation, SleepEnabled = false,
                    ContinuousCollisionMode = ContinuousCollisionMode.Continuous };
                body.Initialize(new Vector3d(x, Fixed64.Zero, Fixed64.Quarter), FixedQuaternion.Identity);
                body.UseManualGrounding();
                Track(agent, collider, body);
                deactivate.Add(body.Deactivate);
                var wallAgent = new TestMatterAgent(context,
                    new FixedTransform(new Vector3d(x, Fixed64.Zero, -Fixed64.One), FixedQuaternion.Identity, Vector3d.One));
                var wall = new LSCuboidCollider { Size = new(2, 4, 2), Material = PhysicsMaterial.Frictionless };
                wall.InitializeWithNoBody(wallAgent);
                wall.OnContactEnter += _ => entered++;
                Track(wallAgent, wall);
                deactivate.Add(wall.Deactivate);
            }
            if (context.Settings.RuntimeMode != PhysicsRuntimeMode.ThreeD)
            {
                var agent = new TestMatterAgent(context);
                var collider = new LSCircleCollider2D(Fixed64.Half) { Material = PhysicsMaterial.Frictionless };
                var body = new SolidBody2D(agent, collider)
                { Mass = Fixed64.One, FreezeAxes = BodyFreezeAxes2D.Rotation, SleepEnabled = false,
                    ContinuousCollisionMode = ContinuousCollisionMode.Continuous };
                body.Initialize(new Vector2d(x, (Fixed64)4.25m));
                Track(agent, collider, body);
                deactivate.Add(body.Deactivate);
                var wallAgent = new TestMatterAgent(context,
                    new FixedTransform(new Vector3d(x, Fixed64.Zero, (Fixed64)3), FixedQuaternion.Identity, Vector3d.One));
                var wall = new LSAABBoxCollider2D(new Vector2d(2, 2)) { Material = PhysicsMaterial.Frictionless };
                wall.InitializeWithNoBody(wallAgent);
                wall.OnContactEnter += _ => entered++;
                Track(wallAgent, wall);
                deactivate.Add(wall.Deactivate);
                if (context.Settings.RuntimeMode == PhysicsRuntimeMode.Mixed)
                {
                    var mixedWall = new LSAABBoxCollider2D(new Vector2d(2, 2)) { Material = PhysicsMaterial.Frictionless };
                    mixedWall.InitializeWithNoBody(new TestMatterAgent(context,
                        new FixedTransform(new Vector3d(x, Fixed64.Zero, -Fixed64.One), FixedQuaternion.Identity, Vector3d.One)));
                    mixedWall.OnContactEnter += _ => entered++;
                    Track(mixedWall);
                    deactivate.Add(mixedWall.Deactivate);
                }
            }
        }
        context.ComputeReplayHash();
        if (exercise != 0)
        {
            context.Simulate(); context.LateSimulate();
            Assert.True(entered > 0);
            if (context.Settings.RuntimeMode == PhysicsRuntimeMode.Mixed)
                Assert.True(context.MixedCollisions.ActivePairCount > 0);
        }
        if (exercise == 2)
        {
            int perGroup = deactivate.Count / count;
            for (int i = perGroup; i < deactivate.Count; i++) deactivate[i]();
            context.Simulate(); context.LateSimulate();
            context.ComputeReplayHash();
        }
        return targets.ToArray();

        void Track(params object[] objects)
        {
            foreach (object target in objects) targets.Add(new WeakReference(target));
        }
    }

    private static void Configure(GravitasWorldContext context, PhysicsRuntimeMode mode)
    {
        context.Settings.RuntimeMode = mode;
        context.Settings.Mixed2DHalfThickness = (Fixed64)2;
        context.SetFrameRate(16);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        context.Environment.MinSpeed = Fixed64.Zero;
    }

    private static void Collect()
    {
        GC.Collect(); GC.WaitForPendingFinalizers(); GC.Collect();
    }
}
