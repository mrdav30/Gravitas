using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;
using Gravitas.Support;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests;

public sealed class StaticSurfaceContinuationTests
{
    [Theory]
    [InlineData("Material")]
    [InlineData("Move")]
    [InlineData("Scale")]
    [InlineData("Filter")]
    [InlineData("Trigger")]
    [InlineData("Remove")]
    [InlineData("Replace")]
    [InlineData("SameMembers")]
    [InlineData("Add")]
    public void DisconnectedMeshEdit_ShouldNotWakeAnUnrelatedSleepingContact(string change)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, PhysicsRuntimeMode.ThreeD);
        LSMeshCollider left = Floor(-8, 0), right = Floor(0, 8), remote = Floor(16, 24);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right, remote });
        var sphere = new LSSphereCollider { Radius = Fixed64.Half, Material = PhysicsMaterial.Frictionless };
        SolidBody body = scene.CreateBody(sphere, new(-(Fixed64)2, Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity).Body;
        body.UseManualGrounding(); body.GravityScale = Fixed64.Zero;
        Step(scene.Context); body.Sleep(); Step(scene.Context);
        Assert.True(body.IsSleeping);
        Vector3d position = body.Position3d;
        switch (change)
        {
            case "Material": remote.Material = PhysicsMaterial.Bouncy; break;
            case "Move": remote.Position += Vector3d.Right; break;
            case "Scale": remote.Transform.LocalScale = Vector3d.One * Fixed64.Half; break;
            case "Filter": remote.IgnoredCollisionLayers = PhysicsLayerMask.All; break;
            case "Trigger": remote.IsTrigger = true; break;
            case "Remove": remote.Deactivate(); break;
            case "Replace": surface.ReplaceMembers(new[] { left, right }); break;
            case "SameMembers": surface.ReplaceMembers(new[] { remote, right, left }); break;
            default: surface.ReplaceMembers(new[] { left, right, remote, Floor(24, 32) }); break;
        }
        Step(scene.Context);
        Assert.True(body.IsSleeping);
        Assert.Equal(position, body.Position3d);

        LSMeshCollider Floor(int from, int to)
        {
            var member = new LSMeshCollider(new[] { new Vector3d(from, 0, -2), new(from, 0, 2),
                new(to, 0, -2), new(to, 0, 2) }, new[] { 0, 1, 2, 2, 1, 3 },
                MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
            scene.InitializeStaticCollider(member, Vector3d.Zero);
            return member;
        }
    }

    [Theory]
    [InlineData("Material")]
    [InlineData("Move")]
    [InlineData("Scale")]
    [InlineData("Filter")]
    [InlineData("Trigger")]
    [InlineData("Remove")]
    [InlineData("Replace")]
    [InlineData("SameMembers")]
    [InlineData("Add")]
    public void DisconnectedPlanarEdit_ShouldNotWakeAnUnrelatedSleepingContact(string change)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, PhysicsRuntimeMode.TwoD);
        LSCollider2D left = Rectangle(-8, 0), right = Rectangle(0, 8), remote = Rectangle(16, 24);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right, remote });
        var circle = new LSCircleCollider2D(Fixed64.Half) { Material = PhysicsMaterial.Frictionless };
        var body = new SolidBody2D(new TestMatterAgent(scene.Context), circle) { Mass = Fixed64.One };
        body.Initialize(new(-(Fixed64)2, Fixed64.Quarter)); body.UseManualGrounding(); body.GravityScale = Fixed64.Zero;
        Step(scene.Context); body.Sleep(); Step(scene.Context);
        Assert.True(body.IsSleeping);
        Vector2d position = body.Position;
        switch (change)
        {
            case "Material": remote.Material = PhysicsMaterial.Bouncy; break;
            case "Move": remote.Agent.Transform.LocalPosition += Vector3d.Right; break;
            case "Scale": remote.Agent.Transform.LocalScale = Vector3d.One * Fixed64.Half; break;
            case "Filter": remote.IgnoredCollisionLayers = PhysicsLayerMask.All; break;
            case "Trigger": remote.IsTrigger = true; break;
            case "Remove": remote.Deactivate(); break;
            case "Replace": surface.ReplaceMembers(new[] { left, right }); break;
            case "SameMembers": surface.ReplaceMembers(new[] { remote, right, left }); break;
            default: surface.ReplaceMembers(new[] { left, right, remote, Rectangle(24, 32) }); break;
        }
        Step(scene.Context);
        Assert.True(body.IsSleeping);
        Assert.Equal(position, body.Position);

        LSCollider2D Rectangle(int from, int to)
        {
            var member = new LSAABBoxCollider2D(new Vector2d(to - from, 2));
            member.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(
                new Vector3d((Fixed64)(from + to) * Fixed64.Half, Fixed64.Zero, -Fixed64.One),
                FixedQuaternion.Identity, Vector3d.One)));
            return member;
        }
    }

    private static void Configure(GravitasWorldContext context, PhysicsRuntimeMode mode)
    {
        context.Settings.RuntimeMode = mode; context.SetFrameRate(16);
        context.Environment.Gravity = Fixed64.Zero; context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero; context.Environment.MinSpeed = Fixed64.Zero;
    }

    [Theory]
    [InlineData(false, 0)] [InlineData(true, 0)]
    [InlineData(false, 1)] [InlineData(true, 1)]
    [InlineData(false, 2)] [InlineData(true, 2)]
    public void JoinedQuerySupport_ShouldRevalidateItsMemberWithoutWakingForAnUnchangedWitness(bool planar, int change)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, planar ? PhysicsRuntimeMode.TwoD : PhysicsRuntimeMode.ThreeD);
        scene.Context.Environment.Gravity = Fixed64.One;
        scene.Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(0);
        SolidBody? spatial = null;
        SolidBody2D? flat = null;
        System.IDisposable surface;
        System.Action mutate;
        if (planar)
        {
            var left = Rectangle(-2); var right = Rectangle(2); var remote = Rectangle(18);
            surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right, remote });
            var circle = new LSCircleCollider2D(Fixed64.Half) { LocalOffset = Vector2d.Forward };
            flat = new SolidBody2D(new TestMatterAgent(scene.Context), circle)
                { Mass = Fixed64.One, Gravity = -Vector2d.Forward, SleepEnabled = false };
            flat.Initialize(new(-2, 0));
            mutate = change == 0 ? () => remote.Material = PhysicsMaterial.Bouncy
                : change == 1 ? () => right.Agent.Transform.LocalPosition += Vector3d.Right * (Fixed64)12
                : () => left.IgnoredCollisionLayers = PhysicsLayerMask.All;

            LSCollider2D Rectangle(int x)
            {
                var member = new LSAABBoxCollider2D(new Vector2d(4, 1));
                member.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(
                    new Vector3d((Fixed64)x, Fixed64.Zero, -Fixed64.Half), FixedQuaternion.Identity, Vector3d.One)));
                return member;
            }
        }
        else
        {
            var left = Floor(-4, 0); var right = Floor(0, 4); var remote = Floor(16, 20);
            surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right, remote });
            spatial = scene.CreateBody(new LSSphereCollider { Radius = Fixed64.Half, LocalOffset = Vector3d.Up },
                new(-2, 0, 0), FixedQuaternion.Identity, preventAngularForces: true).Body;
            spatial.SleepEnabled = false;
            mutate = change == 0 ? () => remote.Material = PhysicsMaterial.Bouncy
                : change == 1 ? () => right.Position += Vector3d.Right * (Fixed64)12
                : () => left.IgnoredCollisionLayers = PhysicsLayerMask.All;

            LSMeshCollider Floor(int from, int to)
            {
                var member = new LSMeshCollider(new[] { new Vector3d(from, 0, -2), new(from, 0, 2),
                    new(to, 0, -2), new(to, 0, 2) }, new[] { 0, 1, 2, 2, 1, 3 },
                    MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
                scene.InitializeStaticCollider(member, Vector3d.Zero);
                return member;
            }
        }
        using (surface)
        {
            for (int frame = 0; frame < 16; frame++) Step(scene.Context);
            Assert.True(planar ? flat!.IsGrounded : spatial!.IsGrounded);
            Assert.True(planar ? flat!.Collider.Bounds.Min.Y > Fixed64.Zero : spatial!.Collider.BoundsMin.Y > Fixed64.Zero);
            if (planar) { flat!.SleepEnabled = true; flat.Sleep(); }
            else { spatial!.SleepEnabled = true; spatial.Sleep(); }
            Step(scene.Context);
            Assert.True(planar ? flat!.IsSleeping : spatial!.IsSleeping);
            mutate();
            for (int frame = 0; frame < 4; frame++) Step(scene.Context);
            Assert.Equal(change != 2, planar ? flat!.IsSleeping : spatial!.IsSleeping);
            Assert.Equal(change != 2, planar ? flat!.IsGrounded : spatial!.IsGrounded);
            if (change == 2) Assert.True(planar ? flat!.Position.Y < Fixed64.Zero : spatial!.Position3d.Y < Fixed64.Zero);
        }
    }

    private static void Step(GravitasWorldContext context) { context.Simulate(); context.LateSimulate(); }
}
