using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using System;
using System.Numerics;
using Xunit;

namespace Gravitas.Tests;

/// <summary>Checks complete continuous steps across declared static seams.</summary>
public sealed class StaticSurfaceContinuousCollisionTests
{
    [Fact]
    public void ScalarRoundedRimCandidateAboveTheJoinedSurface_ShouldNotDeflectTangentialMotion()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, PhysicsRuntimeMode.ThreeD);
        const long normalRaw = 1518500250;
        Fixed64 normal = Fixed64.FromRaw(normalRaw);
        Vector3d start = new(Fixed64.FromRaw(32768), normal, normal);
        BigInteger exactGap = 2 * (BigInteger)normalRaw * normalRaw
            - (BigInteger)Fixed64.Half.m_rawValue * Fixed64.Half.m_rawValue;
        Assert.Equal(new BigInteger(72737096), exactGap);
        // The exact floor gap is positive, but its scalar square loses
        // that gap and the small tangent component. The public cast is allowed
        // to supply this conservative candidate; it must not invent response.
        Assert.Equal(Fixed64.Half * Fixed64.Half, new Vector3d(start.X, normal, normal).MagnitudeSquared);
        var mesh = ObliqueSheet(-8, 0); var right = ObliqueSheet(0, 8);
        using var declaration = scene.Context.Physics.CreateStaticSurface(new[] { mesh, right });
        Assert.True(scene.Context.Query3D.SweepSphere(start, Fixed64.Half, Vector3d.Left, Fixed64.FromFraction(1, 16),
            out Physics3DHit hit, PhysicsLayerMask.All, excludedCollider: right));
        Assert.Same(mesh, hit.Collider); Assert.Equal(Fixed64.Zero, hit.Distance);
        var sphere = new LSSphereCollider { Radius = Fixed64.Half, Material = PhysicsMaterial.Frictionless };
        SolidBody body = scene.CreateBody(sphere, start, FixedQuaternion.Identity).Body;
        var initial = new ContactManifold(); initial.BeginUpdate(scene.Context.FrameCount);
        Assert.False(CollisionDetection.DoCollisionCheck(new CollisionWorkItem(scene.Context, mesh, sphere, CollisionType.Mesh_Sphere, initial)));
        body.FreezeAxes = BodyFreezeAxes3D.Rotation; body.SleepEnabled = false; body.UseManualGrounding();
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        body.AddLinearImpulse(Vector3d.Left);
        scene.Context.Simulate(); scene.Context.LateSimulate();
        Assert.Equal(Vector3d.Left, body.LinearVelocity);
        Assert.True(body.Position3d.X < Fixed64.Zero);
        Assert.Equal(start.Y, body.Position3d.Y); Assert.Equal(start.Z, body.Position3d.Z);

        LSMeshCollider ObliqueSheet(int left, int right)
        {
            var member = new LSMeshCollider(new[] { new Vector3d(left, -2, 2), new(left, 2, -2),
                new(right, -2, 2), new(right, 2, -2) }, new[] { 0, 1, 2, 2, 1, 3 },
                MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation) { Material = PhysicsMaterial.Frictionless };
            scene.InitializeStaticCollider(member, Vector3d.Zero);
            return member;
        }
    }

    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void OffsetSphere_InsideTheProxySeam_ShouldStillHitTheLaterPhysicalSurface(bool joined)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context, PhysicsRuntimeMode.Mixed);
        var left = Rectangle(scene, -8, 0); var right = Rectangle(scene, 0, 8);
        using var surface = joined ? scene.Context.Physics2D.CreateStaticSurface(new[] { left, right }) : null;
        var sphere = new LSSphereCollider { Radius = Fixed64.Half, LocalOffset = Vector3d.Forward * Fixed64.Two,
            Material = PhysicsMaterial.Frictionless };
        SolidBody body = scene.CreateBody(sphere, new(-Fixed64.Quarter, Fixed64.Zero, -Fixed64.Half), FixedQuaternion.Identity).Body;
        body.FreezeAxes = BodyFreezeAxes3D.Rotation; body.SleepEnabled = false; body.UseManualGrounding();
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        body.AddLinearImpulse(Vector3d.Backward * (Fixed64)128);
        scene.Context.Simulate(); scene.Context.LateSimulate();
        Assert.True(sphere.Center.Z > Fixed64.Zero, $"The offset sphere crossed the complete solid: {sphere.Center}.");
        Assert.True(body.LastContinuousCollisionToiIterationCount > 0);
    }

    [Fact]
    public void ConcaveMeshSource_ShouldKeepItsConservativeFallbackAgainstDeclaredSurfaces()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context, PhysicsRuntimeMode.ThreeD);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { Sheet(scene, -8, 0, true), Sheet(scene, 0, 8, true) });
        var mesh = MeshTestFixtures.CreateConvexCube(MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        SolidBody body = scene.CreateBody(mesh, new(-2, 2, 0), FixedQuaternion.Identity).Body;
        body.FreezeAxes = BodyFreezeAxes3D.Rotation; body.SleepEnabled = false; body.UseManualGrounding();
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        body.AddLinearImpulse(Vector3d.Down * (Fixed64)64);
        scene.Context.Simulate(); scene.Context.LateSimulate();
        Assert.True(body.Position3d.Y > Fixed64.Zero);
        Assert.True(body.LinearVelocity.Y > (Fixed64)(-64));
    }

    [Theory]
    [InlineData(0)] [InlineData(1)] [InlineData(2)] [InlineData(3)]
    public void PublicInitialOverlap_ShouldKeepBothMemberIdentitiesAndZeroDistance(int dimension)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, dimension < 2 ? dimension == 0 ? PhysicsRuntimeMode.TwoD : PhysicsRuntimeMode.ThreeD
            : PhysicsRuntimeMode.Mixed);
        if (dimension == 0 || dimension == 3)
        {
            var left = Rectangle(scene, -8, 0); var right = Rectangle(scene, 0, 8);
            using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right });
            if (dimension == 0)
            {
                var hits = new SwiftList<Physics2DHit>();
                Assert.Equal(2, scene.Context.Query2D.SweepCircleAll(new(Fixed64.Zero, Fixed64.Quarter),
                    new(Fixed64.One, Fixed64.Quarter), Fixed64.Half, hits));
                Assert.Same(left, hits[0].Collider); Assert.Same(right, hits[1].Collider);
                Assert.Equal(Fixed64.Zero, hits[0].Distance); Assert.Equal(Fixed64.Zero, hits[1].Distance);
            }
            else
            {
                var hits = new SwiftList<PhysicsMixedHit>();
                Assert.Equal(2, scene.Context.QueryMixed.SweepSphereAgainst2DAll(new(Fixed64.Zero, Fixed64.Zero, Fixed64.Quarter),
                    new(Fixed64.One, Fixed64.Zero, Fixed64.Quarter), Fixed64.Half, PhysicsLayerMask.All, hits));
                Assert.Same(left, hits[0].Collider2D); Assert.Same(right, hits[1].Collider2D);
                Assert.Equal(Fixed64.Zero, hits[0].Distance); Assert.Equal(Fixed64.Zero, hits[1].Distance);
            }
        }
        else
        {
            var left = Sheet(scene, -8, 0, dimension == 1); var right = Sheet(scene, 0, 8, dimension == 1);
            using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
            if (dimension == 1)
            {
                var hits = new SwiftList<Physics3DHit>();
                Assert.Equal(2, scene.Context.Query3D.SweepSphereAll(new(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero),
                    new(Fixed64.One, Fixed64.Quarter, Fixed64.Zero), Fixed64.Half, PhysicsLayerMask.All, hits));
                Assert.Same(left, hits[0].Collider); Assert.Same(right, hits[1].Collider);
                Assert.Equal(Fixed64.Zero, hits[0].Distance); Assert.Equal(Fixed64.Zero, hits[1].Distance);
            }
            else
            {
                var hits = new SwiftList<PhysicsMixedHit>();
                Assert.Equal(2, scene.Context.QueryMixed.SweepCircleAgainst3DAll(new(Fixed64.Zero, Fixed64.Quarter),
                    new(Fixed64.One, Fixed64.Quarter), Fixed64.Half, Fixed64.Zero, Fixed64.Half, PhysicsLayerMask.All, hits));
                Assert.Same(left, hits[0].Collider3D); Assert.Same(right, hits[1].Collider3D);
                Assert.Equal(Fixed64.Zero, hits[0].Distance); Assert.Equal(Fixed64.Zero, hits[1].Distance);
            }
        }
    }

    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void RotatingPlanarCapsule_ShouldKeepTangentialVelocityAcrossTheJoin(bool mixed)
    {
        var monolithic = Run(joined: false);
        var joined = Run(joined: true);
        // A planar endpoint can exchange yaw through floor-normal torque. Its
        // invariant is agreement with the same finite unsplit surface, including
        // any conservative interval-limit tail, rather than positive yaw forever.
        Assert.Equal(monolithic, joined);
        Assert.True(joined[^1].Position.X > Fixed64.Zero, $"The capsule never crossed the join: {joined[^1]}.");
        Assert.NotEqual(Fixed64.Zero, joined[^1].Rotation);

        (Vector2d Position, Fixed64 Rotation, Vector2d Velocity, Fixed64 AngularVelocity, bool Limited)[] Run(bool joined)
        {
            using var scene = PhysicsScenarioBuilder.Create();
            Configure(scene.Context, mixed ? PhysicsRuntimeMode.Mixed : PhysicsRuntimeMode.TwoD);
            using IDisposable? declaration = joined ? mixed
                ? scene.Context.Physics.CreateStaticSurface(new[] { Sheet(scene, -8, 0, false), Sheet(scene, 0, 8, false) })
                : scene.Context.Physics2D.CreateStaticSurface(new[] { Rectangle(scene, -8, 0), Rectangle(scene, 0, 8) }) : null;
            if (!joined)
            {
                if (mixed) Sheet(scene, -8, 8, false);
                else Rectangle(scene, -8, 8);
            }
            var capsule = new LSCapsuleCollider2D(Fixed64.Half, Fixed64.Two)
            { Material = PhysicsMaterial.Frictionless, MixedHalfThicknessOverride = Fixed64.Half };
            var body = new SolidBody2D(new TestMatterAgent(scene.Context), capsule) { Mass = Fixed64.One, SleepEnabled = false };
            body.Initialize(new(-Fixed64.Quarter, Fixed64.One)); body.UseManualGrounding();
            body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            body.AddLinearImpulse(Vector2d.Right); body.AddAngularImpulse(Fixed64.FromFraction(1, 64));
            var frames = new (Vector2d Position, Fixed64 Rotation, Vector2d Velocity, Fixed64 AngularVelocity, bool Limited)[12];
            for (int frame = 0; frame < frames.Length; frame++)
            {
                body.AddLinearImpulse(new(Fixed64.Zero, -Fixed64.FromFraction(1, 16)));
                scene.Context.Simulate(); scene.Context.LateSimulate();
                Assert.Equal(Fixed64.One, body.LinearVelocity.X);
                Assert.True(body.Position.Y > Fixed64.Quarter);
                frames[frame] = (body.Position, body.Rotation, body.LinearVelocity, body.AngularVelocity, body.LastContinuousCollisionToiIterationLimitReached);
            }
            return frames;
        }
    }

    [Fact]
    public void SpatialCapsuleTurningParallelToTheWall_ShouldKeepMotionAcrossTheJoinedSolid()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context, PhysicsRuntimeMode.Mixed);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { Rectangle(scene, -8, 0), Rectangle(scene, 0, 8) });
        var capsule = new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1), Material = PhysicsMaterial.Frictionless };
        SolidBody body = scene.CreateBody(capsule, new(-Fixed64.Quarter, Fixed64.Zero, Fixed64.Half), FixedQuaternion.Identity).Body;
        body.FreezeAxes = BodyFreezeAxes3D.PositionY | BodyFreezeAxes3D.RotationX | BodyFreezeAxes3D.RotationY;
        body.SleepEnabled = false; body.UseManualGrounding();
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        body.AddLinearImpulse(Vector3d.Right); body.AddAngularImpulse(Vector3d.Forward * Fixed64.FromFraction(1, 64));
        for (int frame = 0; frame < 12; frame++)
        {
            body.AddLinearImpulse(Vector3d.Backward * Fixed64.FromFraction(1, 16));
            scene.Context.Simulate(); scene.Context.LateSimulate();
            Assert.Equal(Fixed64.One, body.LinearVelocity.X);
            Assert.True(body.AngularVelocity.Z > Fixed64.Zero, $"Frame {frame}: a Z-normal impulse removed Z angular momentum.");
            Assert.True(body.Position3d.Z > Fixed64.Quarter);
        }
        Assert.True(body.Position3d.X > Fixed64.Quarter, $"The capsule stopped before the join: {body.Position3d}.");
        Assert.NotEqual(FixedQuaternion.Identity, body.Rotation);
    }

    [Fact]
    public void SpatialCapsuleWithFreeTransverseRotation_ShouldRetainMomentumAtConservativeFrontiers()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context, PhysicsRuntimeMode.Mixed);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { Rectangle(scene, -8, 0), Rectangle(scene, 0, 8) });
        var capsule = new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1), Material = PhysicsMaterial.Frictionless };
        SolidBody body = scene.CreateBody(capsule, new(-Fixed64.Quarter, Fixed64.Zero, Fixed64.Half), FixedQuaternion.Identity).Body;
        body.FreezeAxes = BodyFreezeAxes3D.PositionY; body.SleepEnabled = false; body.UseManualGrounding();
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        body.AddLinearImpulse(Vector3d.Right); body.AddAngularImpulse(Vector3d.Forward * Fixed64.FromFraction(1, 64));
        bool limited = false;
        for (int frame = 0; frame < 12; frame++)
        {
            body.AddLinearImpulse(Vector3d.Backward * Fixed64.FromFraction(1, 16));
            scene.Context.Simulate(); scene.Context.LateSimulate();
            Assert.Equal(Fixed64.One, body.LinearVelocity.X);
            Assert.True(body.AngularVelocity.Z > Fixed64.Zero, $"Frame {frame}: a nonclosing witness removed Z angular momentum.");
            Assert.True(body.Position3d.Z > Fixed64.Quarter);
            limited |= body.LastContinuousCollisionToiIterationLimitReached;
        }
        // The general rotated contact path can retain a stationary tail when
        // geometry advances below the response closing-speed deadband. It must
        // retain momentum at that frontier rather than invent a stopping impulse.
        Assert.True(limited, "This free-axis fixture must exercise its conservative stationary tail.");
    }

    [Fact]
    public void SolvedJoinedSolidPlane_ShouldNotHideARaisedMemberLaterInTheMotion()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context, PhysicsRuntimeMode.Mixed);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[]
        { Rectangle(scene, -8, 0), Rectangle(scene, 0, 1), Rectangle(scene, 1, 2, top: 2) });
        var capsule = new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1), Material = PhysicsMaterial.Frictionless };
        SolidBody body = scene.CreateBody(capsule, new(-Fixed64.Quarter, Fixed64.Zero, Fixed64.Half), FixedQuaternion.Identity).Body;
        body.FreezeAxes = BodyFreezeAxes3D.PositionY | BodyFreezeAxes3D.RotationX | BodyFreezeAxes3D.RotationY;
        body.SleepEnabled = false; body.UseManualGrounding();
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        body.AddLinearImpulse(Vector3d.Right * (Fixed64)32);
        body.AddAngularImpulse(Vector3d.Forward * Fixed64.FromFraction(1, 64));
        body.AddLinearImpulse(Vector3d.Backward * Fixed64.FromFraction(1, 16));
        scene.Context.Simulate(); scene.Context.LateSimulate();
        Assert.True(body.Position3d.X < Fixed64.One, $"The capsule crossed the raised member: {body.Position3d}.");
        Assert.True(body.Position3d.X > Fixed64.Quarter, $"The handled plane stopped motion before the raised member: {body.Position3d}.");
        Assert.True(body.LinearVelocity.X < (Fixed64)32, "The raised wall needs an actual closing response, not an unresolved interval clamp.");
        Assert.True(body.LastContinuousCollisionToiIterationCount > 0);
    }

    [Theory]
    [InlineData(false, false, false)] [InlineData(true, false, false)]
    [InlineData(false, true, false)] [InlineData(true, true, false)]
    [InlineData(false, false, true)] [InlineData(true, false, true)]
    [InlineData(false, true, true)] [InlineData(true, true, true)]
    public void SolvedPlane_ShouldNotHideADisconnectedLowerCornerWithinTheRetainedOverlap(bool joined, bool bridged, bool sameHeight)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context, PhysicsRuntimeMode.Mixed);
        var floor = new LSPolygonCollider2D(new Vector2d(-8, -2), new(Fixed64.Half, -(Fixed64)2),
            new(Fixed64.Half, Fixed64.Zero), new(-8, 0))
        { Material = PhysicsMaterial.Frictionless, MixedHalfThicknessOverride = Fixed64.Two };
        Fixed64 lowerTop = sameHeight ? Fixed64.Zero : -Fixed64.FromRaw(1);
        var block = new LSPolygonCollider2D(new Vector2d(1, -2), new(2, -2), new((Fixed64)2, lowerTop), new(Fixed64.One, lowerTop))
        { Material = PhysicsMaterial.Frictionless, MixedHalfThicknessOverride = Fixed64.Two };
        floor.InitializeWithNoBody(new TestMatterAgent(scene.Context));
        block.InitializeWithNoBody(new TestMatterAgent(scene.Context));
        var bridge = new LSPolygonCollider2D(new Vector2d(Fixed64.Half, -(Fixed64)2), new(1, -2), new(1, -1), new(Fixed64.Half, -Fixed64.One))
        { Material = PhysicsMaterial.Frictionless, MixedHalfThicknessOverride = Fixed64.Two };
        if (bridged) bridge.InitializeWithNoBody(new TestMatterAgent(scene.Context));
        using IDisposable? declaration = joined ? scene.Context.Physics2D.CreateStaticSurface(bridged
            ? new LSCollider2D[] { floor, block, bridge } : new LSCollider2D[] { floor, block }) : null;
        var capsule = new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1), Material = PhysicsMaterial.Frictionless };
        SolidBody body = scene.CreateBody(capsule, new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromFraction(49, 100)), FixedQuaternion.Identity).Body;
        body.FreezeAxes = BodyFreezeAxes3D.PositionY | BodyFreezeAxes3D.RotationX | BodyFreezeAxes3D.RotationY;
        body.SleepEnabled = false; body.UseManualGrounding(); body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        body.AddLinearImpulse(Vector3d.Right * (Fixed64)64 + Vector3d.Backward);
        body.AddAngularImpulse(Vector3d.Forward * Fixed64.FromFraction(1, 64));
        scene.Context.Simulate(); scene.Context.LateSimulate();
        // The initial floor penetration also reaches the separate lower block's
        // exposed corner. A solved plane on another component cannot hide it.
        AssertSphereSubsetOverlaps(scene.Context, block, new(Fixed64.One + Fixed64.Half, Fixed64.Zero, Fixed64.FromFraction(49, 100)));
        Assert.True(body.LinearVelocity.X < (Fixed64)64
                || body.LastContinuousCollisionToiIterationLimitReached && body.Position3d.X < Fixed64.One,
            $"joined={joined}: the separate corner supplied no closing response; position {body.Position3d}, velocity {body.LinearVelocity}, TOI {body.LastContinuousCollisionToiIterationCount}, limit {body.LastContinuousCollisionToiIterationLimitReached}.");
        Assert.True(body.LastContinuousCollisionToiIterationCount > 1);
    }

    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void SolvedCap_ShouldKeepTheRealRimAcrossAHoleInAConnectedCoplanarSurface(bool joined)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context, PhysicsRuntimeMode.Mixed);
        var left = new LSPolygonCollider2D(new Vector2d(-8, -2), new(Fixed64.Half, -(Fixed64)2),
            new(Fixed64.Half, (Fixed64)2), new(-8, 2));
        var right = new LSPolygonCollider2D(new Vector2d(1, -2), new(2, -2), new(2, 2), new(1, 2));
        var bridge = new LSPolygonCollider2D(new Vector2d(Fixed64.Half, Fixed64.One), new(1, 1), new(1, 2), new(Fixed64.Half, (Fixed64)2));
        LSCollider2D[] members = { left, right, bridge };
        foreach (LSCollider2D member in members)
        {
            member.Material = PhysicsMaterial.Frictionless; member.MixedHalfThicknessOverride = Fixed64.Half;
            member.InitializeWithNoBody(new TestMatterAgent(scene.Context));
        }
        using IDisposable? declaration = joined ? scene.Context.Physics2D.CreateStaticSurface(members) : null;
        var capsule = new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1), Material = PhysicsMaterial.Frictionless };
        SolidBody body = scene.CreateBody(capsule, new(Fixed64.Zero, Fixed64.FromFraction(149, 100), Fixed64.Zero), FixedQuaternion.Identity).Body;
        body.FreezeAxes = BodyFreezeAxes3D.PositionZ | BodyFreezeAxes3D.RotationX | BodyFreezeAxes3D.RotationZ;
        body.SleepEnabled = false; body.UseManualGrounding(); body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        body.AddLinearImpulse(Vector3d.Right * (Fixed64)64 + Vector3d.Down);
        body.AddAngularImpulse(Vector3d.Up * Fixed64.FromFraction(1, 64));
        scene.Context.Simulate(); scene.Context.LateSimulate();
        // The lower endpoint sphere is a subset of this upright capsule. At the
        // route midpoint it intersects the far rim despite the remote bridge.
        AssertSphereSubsetOverlaps(scene.Context, right, new(Fixed64.One + Fixed64.Half, Fixed64.FromFraction(99, 100), Fixed64.Zero));
        Assert.True(body.LinearVelocity.X < (Fixed64)64
                || body.LastContinuousCollisionToiIterationLimitReached && body.Position3d.X < Fixed64.One,
            $"joined={joined}: the capsule crossed the hole's true rim; position {body.Position3d}, velocity {body.LinearVelocity}, TOI {body.LastContinuousCollisionToiIterationCount}, limit {body.LastContinuousCollisionToiIterationLimitReached}.");
    }

    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void MixedRoundedFiniteRimEntry_ShouldKeepTheNonclosingFilledFace(bool planar)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context, PhysicsRuntimeMode.Mixed);
        Fixed64 height = Fixed64.FromRaw(Fixed64.Half.m_rawValue - 1);
        Fixed64 entry = Fixed64.Quarter - Fixed64.FromRaw(65536);
        // The exact left-rim entry is sqrt(2*rRaw-1). Nearest TOI sampling
        // leaves x=65536raw, whose squared rim gap is positive by one raw².
        Assert.Equal(BigInteger.One, (BigInteger)65536 * 65536 - (2 * (BigInteger)Fixed64.Half.m_rawValue - 1));
        var hits = new SwiftList<PhysicsMixedHit>();
        IDisposable declaration;
        if (planar)
        {
            var left = Sheet(scene, -8, 0, false); var right = Sheet(scene, 0, 8, false);
            declaration = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
            scene.Context.QueryMixed.SweepCircleAgainst3DAll(new(Fixed64.Quarter, height), new(-Fixed64.Quarter, height),
                Fixed64.Half, Fixed64.Zero, Fixed64.Half, PhysicsLayerMask.All, hits);
            Assert.Contains(hits, hit => hit.Collider3D == left && hit.Distance == entry);
        }
        else
        {
            var left = Rectangle(scene, -8, 0); var right = Rectangle(scene, 0, 8);
            declaration = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right });
            scene.Context.QueryMixed.SweepSphereAgainst2DAll(new(Fixed64.Quarter, Fixed64.Zero, height),
                new(-Fixed64.Quarter, Fixed64.Zero, height), Fixed64.Half, PhysicsLayerMask.All, hits);
            Assert.Contains(hits, hit => hit.Collider2D == left && hit.Distance == entry);
        }
        using (declaration)
        if (planar)
        {
            var circle = new LSCircleCollider2D(Fixed64.Half)
            { Material = PhysicsMaterial.Frictionless, MixedHalfThicknessOverride = Fixed64.Half };
            var body = new SolidBody2D(new TestMatterAgent(scene.Context), circle)
            { Mass = Fixed64.One, FreezeAxes = BodyFreezeAxes2D.Rotation, SleepEnabled = false };
            body.Initialize(new(Fixed64.Quarter, height)); body.UseManualGrounding();
            body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            body.AddLinearImpulse(new((Fixed64)(-8), Fixed64.Zero));
            scene.Context.Simulate(); scene.Context.LateSimulate();
            Assert.Equal((Fixed64)(-8), body.LinearVelocity.X);
            Assert.Equal(Fixed64.Zero, body.LinearVelocity.Y);
            Assert.True(body.Position.X < Fixed64.Zero);
            Assert.Equal(height, body.Position.Y);
        }
        else
        {
            SolidBody body = scene.CreateBody(new LSSphereCollider { Radius = Fixed64.Half, Material = PhysicsMaterial.Frictionless },
                new(Fixed64.Quarter, Fixed64.Zero, height), FixedQuaternion.Identity).Body;
            body.FreezeAxes = BodyFreezeAxes3D.Rotation | BodyFreezeAxes3D.PositionY;
            body.SleepEnabled = false; body.UseManualGrounding();
            body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            body.AddLinearImpulse(Vector3d.Left * (Fixed64)8);
            scene.Context.Simulate(); scene.Context.LateSimulate();
            Assert.Equal((Fixed64)(-8), body.LinearVelocity.X);
            Assert.Equal(Fixed64.Zero, body.LinearVelocity.Z);
            Assert.True(body.Position3d.X < Fixed64.Zero);
            Assert.Equal(height, body.Position3d.Z);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PlanarCircleCrossing_ShouldRetainTangentialMotionAndBlocking(bool mixed)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, mixed ? PhysicsRuntimeMode.Mixed : PhysicsRuntimeMode.TwoD);
        IDisposable declaration = mixed
            ? scene.Context.Physics.CreateStaticSurface(new[] { Sheet(scene, -8, 0, false), Sheet(scene, 0, 8, false) })
            : scene.Context.Physics2D.CreateStaticSurface(new[] { Rectangle(scene, -8, 0), Rectangle(scene, 0, 8) });
        using (declaration)
        {
            var circle = new LSCircleCollider2D(Fixed64.Half)
            { Material = PhysicsMaterial.Frictionless, MixedHalfThicknessOverride = Fixed64.Half };
            var body = new SolidBody2D(new TestMatterAgent(scene.Context), circle)
            {
                Mass = Fixed64.One, FreezeAxes = BodyFreezeAxes2D.Rotation, SleepEnabled = false
            };
            body.Initialize(new(-Fixed64.Two, Fixed64.Half)); body.UseManualGrounding();
            for (int frame = 0; frame < 32; frame++) Step();
            // CCD must preserve an existing resting contact, including its small
            // discrete penetration, when a host enables continuous motion.
            body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            body.AddLinearImpulse(Vector2d.Right);
            for (int frame = 0; frame < 64; frame++)
            {
                Step();
                Assert.True(body.LinearVelocity.X == Fixed64.One,
                    $"Frame {frame}, mixed={mixed}: position {body.Position}, velocity {body.LinearVelocity}.");
                Assert.InRange(body.Position.Y.m_rawValue, Fixed64.Quarter.m_rawValue, Fixed64.Half.m_rawValue);
            }
            Assert.Equal(Fixed64.Two, body.Position.X);

            void Step()
            {
                body.AddLinearImpulse(new(Fixed64.Zero, -Fixed64.FromFraction(1, 16)));
                scene.Context.Simulate(); scene.Context.LateSimulate();
            }
        }
    }

    [Theory]
    [InlineData(false, true, false)]
    [InlineData(true, true, false)]
    [InlineData(false, false, false)]
    [InlineData(false, true, true)]
    [InlineData(true, true, true)]
    public void SpatialRoundedCrossing_ShouldRetainTangentialMotionAndBlocking(bool mixed, bool split, bool capsule)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, mixed ? PhysicsRuntimeMode.Mixed : PhysicsRuntimeMode.ThreeD);
        IDisposable declaration = mixed
            ? scene.Context.Physics2D.CreateStaticSurface(new[] { Rectangle(scene, -8, 0), Rectangle(scene, 0, 8) })
            : scene.Context.Physics.CreateStaticSurface(split
                ? new[] { Sheet(scene, -8, 0, true), Sheet(scene, 0, 8, true) }
                : new[] { Sheet(scene, -8, 8, true) });
        using (declaration)
        {
            LSCollider mover = capsule ? new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) }
                : new LSSphereCollider { Radius = Fixed64.Half };
            mover.Material = PhysicsMaterial.Frictionless;
            Vector3d start = mixed ? new(-Fixed64.Two, Fixed64.Zero, Fixed64.Half)
                : new(-Fixed64.Two, capsule ? Fixed64.One : Fixed64.Half, Fixed64.Zero);
            SolidBody body = scene.CreateBody(mover, start, FixedQuaternion.Identity).Body;
            body.FreezeAxes = BodyFreezeAxes3D.Rotation | (mixed ? BodyFreezeAxes3D.PositionY : BodyFreezeAxes3D.None);
            body.SleepEnabled = false; body.UseManualGrounding();
            for (int frame = 0; frame < 32; frame++) Step();
            body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            body.AddLinearImpulse(Vector3d.Right);
            for (int frame = 0; frame < 64; frame++)
            {
                Step();
                Assert.True(body.LinearVelocity.X == Fixed64.One,
                    $"Frame {frame}, mixed={mixed}: position {body.Position3d}, velocity {body.LinearVelocity}.");
                Fixed64 height = mixed ? body.Position3d.Z : body.Position3d.Y;
                Assert.InRange(height.m_rawValue, (!mixed && capsule ? Fixed64.Half + Fixed64.Quarter : Fixed64.Quarter).m_rawValue,
                    (!mixed && capsule ? Fixed64.One : Fixed64.Half).m_rawValue);
            }
            Assert.Equal(Fixed64.Two, body.Position3d.X);

            void Step()
            {
                body.AddLinearImpulse((mixed ? Vector3d.Backward : Vector3d.Down) * Fixed64.FromFraction(1, 16));
                scene.Context.Simulate(); scene.Context.LateSimulate();
            }
        }
    }

    [Fact]
    public void NonclosingSeamBeforeLaterWallInTheSameMesh_ShouldNotHideTheWall()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context, PhysicsRuntimeMode.ThreeD);
        var left = Sheet(scene, -8, 0, true);
        var floorAndWall = new LSMeshCollider(new[] { new Vector3d(0, 0, -2), new(0, 0, 2),
            new(1, 0, -2), new(1, 0, 2), new(1, 2, -2), new(1, 2, 2) },
            new[] { 0, 1, 2, 2, 1, 3, 2, 3, 4, 4, 3, 5 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
        { Material = PhysicsMaterial.Frictionless };
        scene.InitializeStaticCollider(floorAndWall, Vector3d.Zero);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, floorAndWall });
        var sphere = new LSSphereCollider { Radius = Fixed64.Half, Material = PhysicsMaterial.Frictionless };
        SolidBody body = scene.CreateBody(sphere, new(-Fixed64.Quarter, Fixed64.Half, Fixed64.Zero), FixedQuaternion.Identity).Body;
        body.FreezeAxes = BodyFreezeAxes3D.Rotation; body.SleepEnabled = false; body.UseManualGrounding();
        for (int frame = 0; frame < 32; frame++)
        {
            body.AddLinearImpulse(Vector3d.Down * Fixed64.FromFraction(1, 16));
            scene.Context.Simulate(); scene.Context.LateSimulate();
        }
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        body.AddLinearImpulse(Vector3d.Right * (Fixed64)32);
        scene.Context.Simulate(); scene.Context.LateSimulate();
        // The genuine vertical wall is X=1. The initial floor contact and
        // covered seam must not remove that later feature from CCD searching.
        Assert.True(body.Position3d.X < Fixed64.One, $"The sphere crossed the wall: {body.Position3d}.");
        Assert.True(body.Position3d.X > Fixed64.Quarter, $"The floor stopped the sphere before the real wall: {body.Position3d}.");
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PlanarCircle_TrueOuterEdge_ShouldStillBlockIncomingMotion(bool mixed)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, mixed ? PhysicsRuntimeMode.Mixed : PhysicsRuntimeMode.TwoD);
        IDisposable declaration = mixed
            ? scene.Context.Physics.CreateStaticSurface(new[] { Sheet(scene, -8, 0, false), Sheet(scene, 0, 8, false) })
            : scene.Context.Physics2D.CreateStaticSurface(new[] { Rectangle(scene, -8, 0), Rectangle(scene, 0, 8) });
        using (declaration)
        {
            var circle = new LSCircleCollider2D(Fixed64.Half)
            { Material = PhysicsMaterial.Frictionless, MixedHalfThicknessOverride = Fixed64.Half };
            var body = new SolidBody2D(new TestMatterAgent(scene.Context), circle)
            { Mass = Fixed64.One, FreezeAxes = BodyFreezeAxes2D.Rotation, SleepEnabled = false,
                ContinuousCollisionMode = ContinuousCollisionMode.Continuous };
            body.Initialize(new((Fixed64)(-9), Fixed64.Zero)); body.UseManualGrounding();
            body.AddLinearImpulse(Vector2d.Right * (Fixed64)512);
            scene.Context.Simulate(); scene.Context.LateSimulate();
            Assert.True(body.Position.X < (Fixed64)(-8), $"The circle crossed the outer edge: {body.Position}.");
            Assert.True(body.LinearVelocity.X < (Fixed64)512);
            Assert.True(body.LastContinuousCollisionToiIterationCount > 0);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SpatialSphere_TrueOuterEdge_ShouldStillBlockIncomingMotion(bool mixed)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context, mixed ? PhysicsRuntimeMode.Mixed : PhysicsRuntimeMode.ThreeD);
        IDisposable declaration = mixed
            ? scene.Context.Physics2D.CreateStaticSurface(new[] { Rectangle(scene, -8, 0), Rectangle(scene, 0, 8) })
            : scene.Context.Physics.CreateStaticSurface(new[] { Sheet(scene, -8, 0, true), Sheet(scene, 0, 8, true) });
        using (declaration)
        {
            var sphere = new LSSphereCollider { Radius = Fixed64.Half, Material = PhysicsMaterial.Frictionless };
            Vector3d start = mixed ? new((Fixed64)(-9), Fixed64.Zero, Fixed64.Zero)
                : new((Fixed64)(-9), Fixed64.Zero, Fixed64.Zero);
            SolidBody body = scene.CreateBody(sphere, start, FixedQuaternion.Identity).Body;
            body.FreezeAxes = BodyFreezeAxes3D.Rotation | (mixed ? BodyFreezeAxes3D.PositionY : BodyFreezeAxes3D.None);
            body.SleepEnabled = false; body.UseManualGrounding();
            body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            body.AddLinearImpulse(Vector3d.Right * (Fixed64)512);
            scene.Context.Simulate(); scene.Context.LateSimulate();
            Assert.True(body.Position3d.X < (Fixed64)(-8), $"The sphere crossed the outer edge: {body.Position3d}.");
            Assert.True(body.LinearVelocity.X < (Fixed64)512);
            Assert.True(body.LastContinuousCollisionToiIterationCount > 0);
        }
    }

    private static void Configure(GravitasWorldContext context, PhysicsRuntimeMode mode)
    {
        context.Settings.RuntimeMode = mode; context.SetFrameRate(16);
        context.Environment.Gravity = Fixed64.Zero; context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero; context.Environment.MinSpeed = Fixed64.Zero;
        context.Environment.MaxSpeed = context.Environment.MaxFallSpeed = (Fixed64)1024;
    }

    private static void AssertSphereSubsetOverlaps(GravitasWorldContext context, LSCollider2D target, Vector3d center)
    {
        var hits = new SwiftList<PhysicsMixedHit>();
        context.QueryMixed.SweepSphereAgainst2DAll(center, center + Vector3d.Right,
            Fixed64.Half, PhysicsLayerMask.All, hits);
        bool found = false;
        foreach (PhysicsMixedHit hit in hits) found |= hit.Collider2D == target && hit.Distance == Fixed64.Zero;
        Assert.True(found, "The independent radius-half sphere subset must overlap the obstacle at the route midpoint.");
    }

    private static LSCollider2D Rectangle(PhysicsScenarioBuilder scene, int left, int right, int top = 0)
    {
        var member = new LSAABBoxCollider2D(new Vector2d(right - left, top + 2))
        { Material = PhysicsMaterial.Frictionless, MixedHalfThicknessOverride = Fixed64.Two };
        member.InitializeWithNoBody(new TestMatterAgent(scene.Context,
            new FixedTransform(new Vector3d((Fixed64)(left + right) * Fixed64.Half, Fixed64.Zero, (Fixed64)(top - 2) * Fixed64.Half),
                FixedQuaternion.Identity, Vector3d.One)));
        return member;
    }

    private static LSMeshCollider Sheet(PhysicsScenarioBuilder scene, int left, int right, bool horizontal)
    {
        Vector3d[] points = horizontal
            ? new[] { new Vector3d(left, 0, -2), new(left, 0, 2), new(right, 0, -2), new(right, 0, 2) }
            : new[] { new Vector3d(left, -2, 0), new(left, 2, 0), new(right, -2, 0), new(right, 2, 0) };
        var member = new LSMeshCollider(points, new[] { 0, 1, 2, 2, 1, 3 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
        { Material = PhysicsMaterial.Frictionless };
        scene.InitializeStaticCollider(member, Vector3d.Zero);
        return member;
    }
}
