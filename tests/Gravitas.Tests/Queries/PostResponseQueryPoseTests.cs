using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class PostResponseQueryPoseTests
{
    [Theory]
    [InlineData(PhysicsRuntimeMode.ThreeD, false)]
    [InlineData(PhysicsRuntimeMode.Mixed, false)]
    [InlineData(PhysicsRuntimeMode.Both, false)]
    [InlineData(PhysicsRuntimeMode.ThreeD, true)]
    public void Raycast_AfterResponse_UsesFinalBodyPose(PhysicsRuntimeMode mode, bool serviceStep)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        bool mixed = mode == PhysicsRuntimeMode.Mixed;
        Configure(scene.Context, mixed);
        scene.Context.Settings.RuntimeMode = mode;
        AddWall(scene, mixed);
        var capsule = new LSCapsuleCollider
        {
            Radius = Fixed64.Half, Size = new(1, 3, 1), Material = PhysicsMaterial.Frictionless
        };
        var body = scene.CreateBody(capsule, new((Fixed64)(-2), Fixed64.Zero, Fixed64.Quarter), FixedQuaternion.Identity).Body;
        body.FreezeAxes = BodyFreezeAxes3D.PositionY | BodyFreezeAxes3D.Rotation;
        body.SleepEnabled = false;
        body.UseManualGrounding();

        if (serviceStep)
        {
            scene.Context.Physics.Simulate();
            scene.Context.Physics.LateSimulate();
        }
        else
        {
            scene.Context.Simulate();
            scene.Context.LateSimulate();
        }

        Assert.True(body.Position3d.Z > Fixed64.Quarter);
        AssertRay(scene.Context, body, capsule);
        AssertOtherQueries(scene.Context, body, capsule);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ResponseAcrossVoxelBoundary_PublishesNewQueryMembership(bool mixed)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, mixed);
        AddWall(scene, mixed);
        var capsule = new LSCapsuleCollider
        {
            Radius = (Fixed64)2, Size = new(4, 6, 4), Material = PhysicsMaterial.Frictionless
        };
        var body = scene.CreateBody(capsule, new(-2, 0, 0), FixedQuaternion.Identity).Body;
        body.FreezeAxes = BodyFreezeAxes3D.PositionY | BodyFreezeAxes3D.Rotation;
        body.SleepEnabled = false;
        body.UseManualGrounding();
        scene.Context.Simulate();
        scene.Context.LateSimulate();

        Assert.True(scene.Context.Query3D.Raycast(new Vector3d((Fixed64)(-5), Fixed64.Zero, (Fixed64)3.5m), Vector3d.Right,
            (Fixed64)6, out Physics3DHit hit, PhysicsLayerMask.All));
        Assert.Same(capsule, hit.Collider);
        Assert.Equal(body.Position3d, capsule.Center);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RepeatedResponseAndQuery_IsAllocationFreeAfterWarmup(bool mixed)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, mixed);
        AddWall(scene, mixed);
        var capsule = new LSCapsuleCollider { Material = PhysicsMaterial.Frictionless };
        Vector3d start = new((Fixed64)(-2), Fixed64.Zero, Fixed64.Quarter);
        var body = scene.CreateBody(capsule, start, FixedQuaternion.Identity).Body;
        body.FreezeAxes = BodyFreezeAxes3D.PositionY | BodyFreezeAxes3D.Rotation;
        body.SleepEnabled = false;
        body.UseManualGrounding();
        Assert.Equal(0, AllocationTestHelper.MeasureSteadyState(() =>
        {
            body.SetPosition(start);
            scene.Context.Simulate();
            scene.Context.LateSimulate();
            scene.Context.Query3D.Raycast(new Vector3d(-2, 0, 2), Vector3d.Backward,
                (Fixed64)4, out _, PhysicsLayerMask.All);
        }));
    }

    [Fact]
    public void ResponsePublishingCallbackGeometry_AlsoRefreshesMassProperties()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, mixed: true);
        AddWall(scene, mixed: true);
        var capsule = new LSCapsuleCollider
        {
            Radius = Fixed64.Half, Size = new(1, 3, 1), Material = PhysicsMaterial.Frictionless
        };
        var body = scene.CreateBody(capsule,
            new((Fixed64)(-2), Fixed64.Zero, Fixed64.Quarter), FixedQuaternion.Identity).Body;
        body.SleepEnabled = false;
        body.UseManualGrounding();
        capsule.OnMixedContactEnter += _ =>
        {
            capsule.Radius = Fixed64.Quarter;
            capsule.LocalOffset = Vector3d.Up * Fixed64.Quarter;
        };
        scene.Context.Simulate();
        scene.Context.LateSimulate();

        Assert.Equal(Fixed64.Quarter, capsule.ScaledRadius);
        var reference = scene.CreateBody(new LSCapsuleCollider
        {
            Radius = Fixed64.Quarter, Size = new(1, 3, 1), LocalOffset = Vector3d.Up * Fixed64.Quarter
        }, new(5, 0, 5), body.Rotation).Body;
        Assert.Equal(reference.LocalCenterOfMassOffset, body.LocalCenterOfMassOffset);
        Assert.Equal(reference.InverseInertiaTensor, body.InverseInertiaTensor);
    }

    [Fact]
    public void Raycast_AfterRepeatedMixedTraversals_UsesFinalBodyPose()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, mixed: true);
        AddWall(scene, mixed: true);
        var capsule = new LSCapsuleCollider
        {
            Radius = Fixed64.Half, Size = new(1, 3, 1), Material = PhysicsMaterial.Frictionless
        };
        var body = scene.CreateBody(capsule, new((Fixed64)(-2), Fixed64.Zero, Fixed64.Half), FixedQuaternion.Identity).Body;
        body.FreezeAxes = BodyFreezeAxes3D.PositionY | BodyFreezeAxes3D.Rotation;
        body.SleepEnabled = false;
        body.UseManualGrounding();
        for (int i = 0; i < 32; i++) Step();
        for (int traversal = 0; traversal < 4; traversal++)
        {
            body.AddLinearImpulse(new Vector3d(traversal % 2 == 0 ? Fixed64.One : -Fixed64.One,
                Fixed64.Zero, Fixed64.Zero) - body.LinearVelocity);
            for (int frame = 0; frame < 64; frame++)
            {
                body.AddLinearImpulse(new(Fixed64.Zero, Fixed64.Zero, -Fixed64.FromFraction(1, 16)));
                Step();
            }
        }
        Assert.Equal(2100339672, body.Position3d.Z.m_rawValue);
        AssertRay(scene.Context, body, capsule);

        void Step()
        {
            scene.Context.Simulate();
            scene.Context.LateSimulate();
        }
    }

    private static void AssertRay(GravitasWorldContext context, SolidBody body, LSCapsuleCollider capsule)
    {
        Vector3d position = body.Position3d;
        Assert.True(context.Query3D.Raycast(new(position.X, position.Y, (Fixed64)2), Vector3d.Backward,
            (Fixed64)4, out Physics3DHit hit, PhysicsLayerMask.All));
        Assert.Same(capsule, hit.Collider);
        Assert.Equal((Fixed64)2 - (position.Z + capsule.ScaledRadius), hit.Distance);
        Assert.Equal(position, body.Position3d);
    }

    private static void AssertOtherQueries(GravitasWorldContext context, SolidBody body, LSCapsuleCollider capsule)
    {
        var hash = context.ComputeReplayHash();
        Vector3d position = body.Position3d;
        Vector3d start = new(position.X, position.Y, (Fixed64)2);
        Vector3d end = start - Vector3d.Forward * (Fixed64)4;
        var hits = new SwiftList<Physics3DHit>();
        context.Query3D.RaycastAll(start, end, PhysicsLayerMask.All, hits);
        Assert.Equal((Fixed64)2 - (position.Z + capsule.ScaledRadius), hits[0].Distance);
        Assert.True(context.Query3D.SweepSphere(start, Fixed64.Quarter, Vector3d.Backward,
            (Fixed64)4, out Physics3DHit sweep, PhysicsLayerMask.All));
        Assert.Same(capsule, sweep.Collider);
        Assert.Equal((Fixed64)2 - (position.Z + capsule.ScaledRadius + Fixed64.Quarter), sweep.Distance);

        Vector3d cap = position + Vector3d.Forward * (capsule.ScaledRadius + Fixed64.FromFraction(1, 64));
        Assert.True(context.Query3D.OverlapCircle(cap, Fixed64.FromFraction(1, 32), out Physics3DHit overlap,
            PhysicsLayerMask.All));
        Assert.Same(capsule, overlap.Collider);
        var requests = new[] { new PhysicsRaycast3DRequest(start, end, PhysicsLayerMask.All) };
        var batchHits = new Physics3DHit[1];
        Assert.Equal(1, context.Query3D.RaycastBatch(requests, batchHits));
        Assert.Equal(hits[0].Distance, batchHits[0].Distance);
        Assert.True(context.QueryMixed.SweepCircleAgainst3D(start.ToVector2d(), end.ToVector2d(),
            Fixed64.Quarter, Fixed64.Zero, Fixed64.Quarter, PhysicsLayerMask.All, out PhysicsMixedHit mixed));
        Assert.Same(capsule, mixed.Collider3D);
        Assert.Equal(sweep.Distance, mixed.Distance);
        Assert.Equal(hash, context.ComputeReplayHash());
    }

    private static void AddWall(PhysicsScenarioBuilder scene, bool mixed)
    {
        if (mixed)
        {
            var wall = new LSAABBoxCollider2D(new Vector2d(16, 2)) { Material = PhysicsMaterial.Frictionless };
            wall.InitializeWithNoBody(new TestMatterAgent(scene.Context,
                new FixedTransform(new Vector3d(0, 0, -1), FixedQuaternion.Identity, Vector3d.One)));
        }
        else
            scene.InitializeStaticCollider(new LSCuboidCollider
                { Size = new(16, 4, 2), Material = PhysicsMaterial.Frictionless }, new(0, 0, -1));
    }

    private static void Configure(GravitasWorldContext context, bool mixed)
    {
        context.Settings.RuntimeMode = mixed ? PhysicsRuntimeMode.Mixed : PhysicsRuntimeMode.ThreeD;
        context.Settings.Mixed2DHalfThickness = (Fixed64)2;
        context.SetFrameRate(16);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        context.Environment.MinSpeed = Fixed64.Zero;
    }
}
