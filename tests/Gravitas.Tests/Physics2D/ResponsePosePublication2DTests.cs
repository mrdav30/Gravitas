using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using System.Linq;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class ResponsePosePublication2DTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Corrections_KeepGeometryCurrentAndCoalesceMembershipUntilQuery(bool mixed)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, mixed ? PhysicsRuntimeMode.Mixed : PhysicsRuntimeMode.TwoD);
        var circle = new LSCircleCollider2D(Fixed64.Half);
        SolidBody2D body = CreateBody(scene.Context, circle, Vector2d.Zero);
        scene.Context.Simulate(); scene.Context.LateSimulate();
        var original = circle.PartitionCoordinates!.ToArray();

        body.ApplyCollisionPositionState(body.PrepareCollisionPositionCorrection(Vector2d.Right * (Fixed64)4));
        body.ApplyCollisionPositionState(body.PrepareCollisionPositionCorrection(Vector2d.Right * (Fixed64)4));

        Assert.Equal(new Vector2d(8, 0), circle.Center);
        Assert.Equal(original, circle.PartitionCoordinates!.ToArray());
        Assert.True(scene.Context.Query2D.Raycast(new Vector2d(8, 2), new Vector2d(8, -2), out Physics2DHit hit));
        Assert.Same(circle, hit.Collider);
        Assert.Equal((Fixed64)1.5m, hit.Distance);
        Assert.NotEqual(original, circle.PartitionCoordinates!.ToArray());
        Assert.Equal(new Vector2d(8, 0), body.Position);
        if (mixed) Assert.False(scene.Context.MixedCollisions.ClearPartitioned2DCollider(circle));
    }

    [Fact]
    public void CorrectionBeforeDistribution_StillDiscoversContactsAtTheCorrectedPose()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, PhysicsRuntimeMode.TwoD);
        var circle = new LSCircleCollider2D(Fixed64.Half);
        SolidBody2D body = CreateBody(scene.Context, circle, Vector2d.Zero);
        var wall = new LSAABBoxCollider2D(Vector2d.One);
        wall.InitializeWithNoBody(new TestMatterAgent(scene.Context,
            new FixedTransform(new Vector3d((Fixed64)8.75m, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One)));
        int entered = 0;
        wall.OnContactEnter += _ => entered++;
        body.ApplyCollisionPositionState(body.PrepareCollisionPositionCorrection(Vector2d.Right * (Fixed64)8));

        scene.Context.Simulate(); scene.Context.LateSimulate();

        Assert.Equal(1, entered);
        Assert.NotEqual(new Vector2d(8, 0), body.Position);
        Assert.Equal(body.Position, circle.Center);
    }

    [Theory]
    [InlineData(PhysicsRuntimeMode.TwoD, false)]
    [InlineData(PhysicsRuntimeMode.Both, false)]
    [InlineData(PhysicsRuntimeMode.Mixed, false)]
    [InlineData(PhysicsRuntimeMode.TwoD, true)]
    public void CompletedResponse_PublishesMembershipAcrossVoxelBoundaries(PhysicsRuntimeMode mode, bool serviceStep)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, mode);
        AddWall(scene, mode == PhysicsRuntimeMode.Mixed);
        var circle = new LSCircleCollider2D((Fixed64)2);
        SolidBody2D body = CreateBody(scene.Context, circle, Vector2d.Zero);
        if (serviceStep)
        {
            scene.Context.Physics2D.Simulate(); scene.Context.Physics2D.LateSimulate();
        }
        else
        {
            scene.Context.Simulate(); scene.Context.LateSimulate();
        }

        Assert.True(body.Position.Y > Fixed64.One);
        Assert.Equal(body.Position, circle.Center);
        Assert.False(scene.Context.Collisions2D.ClearPartitionedCollider(circle));
        if (mode == PhysicsRuntimeMode.Mixed)
            Assert.False(scene.Context.MixedCollisions.ClearPartitioned2DCollider(circle));
        Assert.True(scene.Context.Query2D.Raycast(new Vector2d((Fixed64)(-3), (Fixed64)3.5m),
            new Vector2d((Fixed64)3, (Fixed64)3.5m), out Physics2DHit hit));
        Assert.Same(circle, hit.Collider);
    }

    [Fact]
    public void CorrectionWithPendingGeometry_RefreshesMassProperties()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, PhysicsRuntimeMode.TwoD);
        var circle = new LSCircleCollider2D(Fixed64.Half);
        SolidBody2D body = CreateBody(scene.Context, circle, Vector2d.Zero);
        circle.Radius = Fixed64.One;
        circle.LocalOffset = new Vector2d(Fixed64.Zero, Fixed64.Quarter);

        body.ApplyCollisionPositionState(body.PrepareCollisionPositionCorrection(Vector2d.Right));

        Assert.Equal(Fixed64.Half, body.MomentOfInertia);
        Assert.Equal(circle.LocalOffset, body.LocalCenterOfMassOffset);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CorrectionPublicationAndQuery_AreWarmAllocationFree(bool mixed)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, mixed ? PhysicsRuntimeMode.Mixed : PhysicsRuntimeMode.TwoD);
        AddWall(scene, mixed);
        var circle = new LSCircleCollider2D(Fixed64.Half);
        Vector2d start = new(Fixed64.Zero, Fixed64.Quarter);
        SolidBody2D body = CreateBody(scene.Context, circle, start);
        Assert.Equal(0, AllocationTestHelper.MeasureSteadyState(() =>
        {
            body.ResetPosition(start);
            scene.Context.Simulate(); scene.Context.LateSimulate();
            scene.Context.Query2D.Raycast(new Vector2d(0, 2), new Vector2d(0, -2), out _);
        }));
    }

    private static SolidBody2D CreateBody(GravitasWorldContext context, LSCircleCollider2D circle, Vector2d position)
    {
        circle.Material = PhysicsMaterial.Frictionless;
        var body = new SolidBody2D(new TestMatterAgent(context), circle)
        { Mass = Fixed64.One, FreezeAxes = BodyFreezeAxes2D.Rotation, SleepEnabled = false };
        body.Initialize(position);
        body.UseManualGrounding();
        return body;
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

    private static void AddWall(PhysicsScenarioBuilder scene, bool mixed)
    {
        if (mixed)
            scene.InitializeStaticCollider(new LSCuboidCollider
                { Size = new(16, 4, 2), Material = PhysicsMaterial.Frictionless }, new Vector3d(0, 0, -1));
        else
            new LSAABBoxCollider2D(new Vector2d(16, 2)) { Material = PhysicsMaterial.Frictionless }
                .InitializeWithNoBody(new TestMatterAgent(scene.Context,
                    new FixedTransform(new Vector3d(0, 0, -1), FixedQuaternion.Identity, Vector3d.One)));
    }
}
