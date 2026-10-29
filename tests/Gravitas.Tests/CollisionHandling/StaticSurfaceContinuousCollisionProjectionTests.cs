using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using Gravitas.Support;
using GridForge.Configuration;
using SwiftCollections;
using System.Numerics;
using Xunit;

namespace Gravitas.Tests;

public sealed class StaticSurfaceContinuousCollisionProjectionTests
{
    [Fact]
    public void CircleShrunkByGroundingCallback_ShouldRefineTheEarlierLargerProxy()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context, PhysicsRuntimeMode.TwoD);
        var wall = new LSAABBoxCollider2D(new Vector2d(Fixed64.Half, (Fixed64)8))
        { Material = PhysicsMaterial.Frictionless };
        wall.InitializeWithNoBody(new TestMatterAgent(scene.Context,
            new FixedTransform(new(1, 0, 0), FixedQuaternion.Identity, Vector3d.One)));
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { wall });
        var circle = new LSCircleCollider2D(Fixed64.One) { Material = PhysicsMaterial.Frictionless };
        var body = new SolidBody2D(new TestMatterAgent(scene.Context), circle)
        { Mass = Fixed64.One, FreezeAxes = BodyFreezeAxes2D.Rotation, SleepEnabled = false,
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous };
        body.Initialize(Vector2d.Zero);
        body.SetManualGrounding(-Vector2d.Forward, Vector2d.Forward);
        body.UseAutomaticGrounding(checkGroundImmediately: false);
        int changes = 0;
        body.OnGrounded = grounded =>
        {
            Assert.False(grounded);
            changes++;
            circle.Radius = Fixed64.Quarter;
        };
        body.AddLinearImpulse(Vector2d.Right);
        scene.Context.Simulate(); scene.Context.LateSimulate();
        Assert.Equal(1, changes);
        Assert.Equal(Fixed64.Quarter, circle.ScaledRadius);
        Assert.Equal(Fixed64.One, body.LinearVelocity.X);
        Assert.Equal(Fixed64.FromFraction(1, 16), body.Position.X);
    }

    [Fact]
    public void OffsetCircle_ApproachingAnIsolatedFace_ShouldSampleItsPhysicalCenter()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context, PhysicsRuntimeMode.TwoD);
        var floor = new LSAABBoxCollider2D(new Vector2d(4, 2)) { Material = PhysicsMaterial.Frictionless };
        floor.InitializeWithNoBody(new TestMatterAgent(scene.Context,
            new FixedTransform(new(2, 0, -1), FixedQuaternion.Identity, Vector3d.One)));
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { floor });
        var circle = new LSCircleCollider2D(Fixed64.Half)
        { LocalOffset = Vector2d.Right * Fixed64.Two, Material = PhysicsMaterial.Frictionless };
        var body = new SolidBody2D(new TestMatterAgent(scene.Context), circle)
        { Mass = Fixed64.One, FreezeAxes = BodyFreezeAxes2D.Rotation, SleepEnabled = false };
        body.Initialize(new(-Fixed64.One, Fixed64.One)); body.UseManualGrounding();
        Vector2d displacement = new(Fixed64.FromFraction(1, 16), -Fixed64.Half);
        Assert.True(QueryDetection2D.TrySweepMoverShape(circle, displacement, floor, out var hit));
        Assert.True(hit.Distance > Fixed64.Zero);
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        body.AddLinearImpulse(new(Fixed64.One, (Fixed64)(-8)));
        scene.Context.Simulate(); scene.Context.LateSimulate();
        Assert.Equal(Fixed64.One, body.LinearVelocity.X); Assert.Equal(Fixed64.Zero, body.LinearVelocity.Y);
        Assert.Equal(body.Position.X + Fixed64.Two, circle.Center.X);
        Assert.True(circle.Center.Y >= Fixed64.Half);
    }

    [Fact]
    public void RoundedMixedSeamCandidate_ShouldKeepTheLaterPhysicalFloorHit()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context, PhysicsRuntimeMode.Mixed);
        var left = new LSPolygonCollider2D(new Vector2d(-6, 2), new(-2, -2), new(0, 0), new(-4, 4));
        var right = new LSPolygonCollider2D(new Vector2d(-2, -2), new(2, -6), new(4, -4), new(0, 0));
        foreach (LSPolygonCollider2D member in new[] { left, right })
        {
            member.MixedHalfThicknessOverride = Fixed64.Half; member.Material = PhysicsMaterial.Frictionless;
            member.InitializeWithNoBody(new TestMatterAgent(scene.Context,
                new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        }
        using var declaration = scene.Context.Physics2D.CreateStaticSurface(new LSCollider2D[] { left, right });

        const long a = 1_518_500_250, delta = 32_000;
        Vector3d center = new(Fixed64.FromRaw(a + delta), Fixed64.Zero, Fixed64.FromRaw(a - delta));
        Vector3d normal = new(1, 0, 1), start = center + normal, end = center - normal;
        // The rounded rim TOI samples this center, but the exact sphere remains
        // outside the common supporting plane. Its true later floor hit must
        // survive rejection of the covered left corner.
        BigInteger radius = Fixed64.Half.m_rawValue;
        Assert.Equal(new BigInteger(72_737_096), (BigInteger)2 * a * a - radius * radius);
        Assert.True((BigInteger)2 * a * a + (BigInteger)2 * delta * delta > radius * radius);
        var hits = new SwiftList<PhysicsMixedHit>();
        Assert.Equal(2, scene.Context.QueryMixed.SweepSphereAgainst2DAll(start, end, Fixed64.Half,
            PhysicsLayerMask.All, hits, includeTriggers: false));
        Assert.Same(left, hits[0].Collider2D);
        Assert.Equal(center, new FixedSegment(start, end).GetPointAtDistance(hits[0].Distance, (end - start).Magnitude));
        SolidBody probe = scene.CreateBody(new LSSphereCollider { Radius = Fixed64.Half }, center,
            FixedQuaternion.Identity).Body;
        var contacts = new ContactManifold();
        Assert.False(CollisionDetectionMixed.TryBuildDiscreteManifold(probe.Collider, left, contacts));
        Assert.False(CollisionDetectionMixed.TryBuildDiscreteManifold(probe.Collider, right, contacts));
        probe.Deactivate();

        SolidBody body = scene.CreateBody(new LSSphereCollider
            { Radius = Fixed64.Half, Material = PhysicsMaterial.Frictionless }, start, FixedQuaternion.Identity).Body;
        body.FreezeAxes = BodyFreezeAxes3D.Rotation | BodyFreezeAxes3D.PositionY;
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        body.SleepEnabled = false; body.UseManualGrounding(); body.AddLinearImpulse(-normal * (Fixed64)32);
        scene.Context.Simulate(); scene.Context.LateSimulate();
        Assert.True(body.LastContinuousCollisionToiIterationCount > 0);
        Assert.True(body.Position3d.X + body.Position3d.Z < start.X + start.Z);
        Assert.True(body.LinearVelocity.X + body.LinearVelocity.Z >= -Fixed64.Epsilon);
        Assert.Equal(Fixed64.Zero, body.LinearVelocity.X - body.LinearVelocity.Z);
        Assert.True((BigInteger)body.Position3d.X.m_rawValue + body.Position3d.Z.m_rawValue >= (BigInteger)2 * (a - 1),
            $"The real floor did not retain the incoming sphere: {body.Position3d}.");
    }

    [Fact]
    public void CompensatedRigidMesh_ShouldKeepThePhysicalSphereHitInARepresentableChart()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context, PhysicsRuntimeMode.ThreeD);
        Vector3d physicalGridCenter = new(1_224_000_000, -232_000_000, 0);
        Assert.True(scene.Context.World.TryAddGrid(new GridConfiguration(physicalGridCenter - Vector3d.One * (Fixed64)64,
            physicalGridCenter + Vector3d.One * (Fixed64)64), out _));
        Fixed64 b = (Fixed64)(-1_800_000_000);
        var target = new LSMeshCollider(new[]
        {
            new Vector3d(b - Fixed64.Two, b - Fixed64.Two, Fixed64.Zero),
            new Vector3d(b + Fixed64.Two, b - Fixed64.Two, Fixed64.Zero),
            new Vector3d(b - Fixed64.Two, b + Fixed64.Two, Fixed64.Zero),
            new Vector3d(b + Fixed64.Two, b + Fixed64.Two, Fixed64.Zero)
        }, new[] { 0, 1, 2, 2, 1, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
        { Material = PhysicsMaterial.Frictionless };
        Vector3d host = new(Fixed64.Zero, (Fixed64)2_000_000_000, Fixed64.Zero);
        var rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero,
            Fixed64.FromRaw(2576980377), Fixed64.FromRaw(3435973837));
        target.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(host, rotation, Vector3d.One)));
        // Establish the rigid authored-chart prerequisite before testing CCD.
        Assert.Equal(Vector3d.One, target.Mesh.OwnerScale);
        Assert.Equal(host, target.Mesh.Origin);
        Assert.True((BigInteger)target.Center.Y.m_rawValue - host.Y.m_rawValue < long.MinValue);
        using var declaration = scene.Context.Physics.CreateStaticSurface(new[] { target });
        Vector3d start = target.Center + Vector3d.Forward * Fixed64.Two;
        Assert.True(scene.Context.Query3D.SweepSphere(start, Fixed64.Half, Vector3d.Backward, (Fixed64)4,
            out var hit, PhysicsLayerMask.All));
        Assert.Same(target, hit.Collider);
        Assert.Equal(Fixed64.One + Fixed64.Half, hit.Distance);
        SolidBody body = scene.CreateBody(new LSSphereCollider { Radius = Fixed64.Half, Material = PhysicsMaterial.Frictionless },
            start, FixedQuaternion.Identity).Body;
        body.FreezeAxes = BodyFreezeAxes3D.Rotation; body.UseManualGrounding(); body.SleepEnabled = false;
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        body.AddLinearImpulse(Vector3d.Backward * (Fixed64)64);
        scene.Context.Simulate(); scene.Context.LateSimulate();
        Assert.True(body.Position3d.Z > Fixed64.Zero, $"The admitted mesh disappeared from the physical sweep: {body.Position3d}.");
    }

    [Theory]
    [InlineData(-1, false)] [InlineData(1, false)]
    [InlineData(-1, true)] [InlineData(1, true)]
    public void RoundedSourceStraddlingDeclaredPlane_ShouldFollowItsNativeMinimumExit(int sign, bool capsule)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context, PhysicsRuntimeMode.ThreeD);
        var sheet = new LSMeshCollider(new[] { new Vector3d(-8, 0, -2), new(-8, 0, 2), new(8, 0, -2), new(8, 0, 2) },
            new[] { 0, 1, 2, 2, 1, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
        { Material = PhysicsMaterial.Frictionless };
        scene.InitializeStaticCollider(sheet, Vector3d.Zero);
        using var declaration = scene.Context.Physics.CreateStaticSurface(new[] { sheet });
        LSCollider source = capsule
            ? new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1), Material = PhysicsMaterial.Frictionless }
            : new LSSphereCollider { Radius = Fixed64.Half, Material = PhysicsMaterial.Frictionless };
        SolidBody body = scene.CreateBody(source,
            new(-Fixed64.Quarter, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity).Body;
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous; body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        body.SleepEnabled = false; body.UseManualGrounding();
        // The initially straddled sheet has tied up/down exits. The native
        // deterministic tie selects Down; CCD must preserve that contract.
        bool sourceIsA = source.Priority >= sheet.Priority;
        LSCollider colliderA = sourceIsA ? source : sheet, colliderB = sourceIsA ? sheet : source;
        var initial = new ContactManifold(); initial.BeginUpdate(scene.Context.FrameCount);
        Assert.True(CollisionDetection.DoCollisionCheck(new CollisionWorkItem(scene.Context, colliderA, colliderB,
            ColliderSettings.GetCollisionType(colliderA.Shape, colliderB.Shape), initial)));
        Assert.True(initial.HasContact);
        Assert.Equal(Vector3d.Down, sourceIsA ? -initial.PrimaryContact.Normal : initial.PrimaryContact.Normal);
        body.AddLinearImpulse(Vector3d.Up * (Fixed64)(sign * 32));
        scene.Context.Simulate(); scene.Context.LateSimulate();
        if (sign > 0)
        {
            Assert.Equal(Fixed64.Zero, body.LinearVelocity.Y);
            Assert.True(body.Position3d.Y.Abs() < Fixed64.One);
            Assert.True(body.LastContinuousCollisionToiIterationCount > 0);
        }
        else
        {
            Assert.Equal((Fixed64)(sign * 32), body.LinearVelocity.Y);
            Assert.True(body.Position3d.Y < -Fixed64.One);
        }
    }

    [Theory]
    [InlineData("Vertical", true)] [InlineData("ZeroCore", true)]
    [InlineData("TiltX", false)] [InlineData("TiltZ", false)]
    [InlineData("OutsideStart", false)] [InlineData("OutsideEnd", false)]
    [InlineData("Box", false)] [InlineData("UnrepresentableEnd", false)]
    public void ExactSphereSourceProjection_ShouldRequireTheWholePhysicalCapsuleChord(string shape, bool expected)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context, PhysicsRuntimeMode.Mixed);
        LSCollider2D target = Rectangle(scene);
        LSCollider source = shape == "Box" ? new LSCuboidCollider { Size = Vector3d.One }
            : new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, shape == "ZeroCore" ? 1 : 2, 1) };
        FixedQuaternion rotation = shape == "TiltX" ? FixedQuaternion.FromAxisAngle(Vector3d.Right, Fixed64.HalfPi)
            : shape == "TiltZ" ? FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.HalfPi) : FixedQuaternion.Identity;
        Vector3d position = shape is "OutsideStart" or "ZeroCore" ? Vector3d.Up * Fixed64.Two
            : shape == "UnrepresentableEnd" ? Vector3d.Right * Fixed64.Two : Vector3d.Zero;
        scene.CreateBody(source, position, rotation);
        Vector3d displacement = shape == "OutsideEnd" ? Vector3d.Up * Fixed64.Two
            : shape == "UnrepresentableEnd" ? Vector3d.Right * Fixed64.MaxValue : Vector3d.Right;
        bool actual = GravitasQueryMixedService.TryResolveExactSphereSourceFor2D(source, target, displacement,
            out Vector3d start, out Vector3d end, out Fixed64 radius);
        Assert.Equal(expected, actual);
        if (expected)
        {
            Assert.Equal(source.Center, start); Assert.Equal(start + displacement, end); Assert.Equal(Fixed64.Half, radius);
        }
    }

    [Theory]
    [InlineData("CurrentAngular", false)] [InlineData("PreparedAngular", false)]
    [InlineData("KinematicGoal", false)] [InlineData("KinematicTranslation", true)]
    public void ExactSphereSourceProjection_ShouldRejectAnyRetainedOrAuthoredAngularArc(string motion, bool expected)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context, PhysicsRuntimeMode.Mixed);
        LSCollider2D target = Rectangle(scene);
        var capsule = new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) };
        SolidBody body = scene.CreateBody(capsule, Vector3d.Zero, FixedQuaternion.Identity,
            isKinematic: motion.StartsWith("Kinematic")).Body;
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        if (motion is "CurrentAngular" or "PreparedAngular") body.AddAngularImpulse(Vector3d.Up * Fixed64.FromFraction(1, 64));
        if (motion == "PreparedAngular")
        {
            scene.Context.AdvanceLateSimulateToken(); scene.Context.Physics.PrepareContinuousCollisionFrame();
            Assert.NotEqual(Vector3d.Zero, body.SampleContinuousCollisionAngularVelocity(Fixed64.Zero));
            body.ApplyCollisionVelocityState(Vector3d.Zero, Vector3d.Zero);
        }
        if (motion == "KinematicGoal") body.Agent.Transform.LocalRotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.FromFraction(1, 100));
        Assert.Equal(expected, GravitasQueryMixedService.TryResolveExactSphereSourceFor2D(capsule, target, Vector3d.Right,
            out _, out _, out _));
    }

    [Fact]
    public void CapsuleTurningUnderJoinedSlabCaps_ShouldKeepTangentialMotion()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context, PhysicsRuntimeMode.Mixed);
        var left = Rectangle(scene, -8, 0); var right = Rectangle(scene, 0, 8);
        using var declaration = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right });
        var capsule = new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1), Material = PhysicsMaterial.Frictionless };
        SolidBody body = scene.CreateBody(capsule, new(-Fixed64.Quarter, -(Fixed64)2, -Fixed64.One), FixedQuaternion.Identity).Body;
        body.FreezeAxes = BodyFreezeAxes3D.PositionZ | BodyFreezeAxes3D.RotationX | BodyFreezeAxes3D.RotationZ;
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous; body.SleepEnabled = false; body.UseManualGrounding();
        body.AddLinearImpulse(Vector3d.Right); body.AddAngularImpulse(Vector3d.Up * Fixed64.FromFraction(1, 64));
        for (int frame = 0; frame < 12; frame++)
        {
            body.AddLinearImpulse(Vector3d.Up * Fixed64.FromFraction(1, 16));
            scene.Context.Simulate(); scene.Context.LateSimulate();
            Assert.Equal(Fixed64.One, body.LinearVelocity.X);
            Assert.True(body.Position3d.Y <= -(Fixed64)2);
        }
        Assert.True(body.Position3d.X > Fixed64.Quarter);
    }

    [Theory]
    [InlineData(false, false)] [InlineData(true, false)] [InlineData(false, true)] [InlineData(true, true)]
    public void HullCrossingDeclaredConcaveFaces_ShouldKeepItsExactTriangleAdmission(bool meshSource, bool fromBelow)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context, PhysicsRuntimeMode.ThreeD);
        var sheet = new LSMeshCollider(new[] { new Vector3d(-8, 0, -2), new(-8, 0, 2), new(8, 0, -2), new(8, 0, 2) },
            new[] { 0, 1, 2, 2, 1, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
        { Material = PhysicsMaterial.Frictionless };
        scene.InitializeStaticCollider(sheet, Vector3d.Zero);
        using var declaration = scene.Context.Physics.CreateStaticSurface(new[] { sheet });
        LSCollider source = meshSource ? MeshTestFixtures.CreateConvexCube()
            : new LSCuboidCollider { Size = Vector3d.One };
        source.Material = PhysicsMaterial.Frictionless;
        SolidBody body = scene.CreateBody(source,
            new(-Fixed64.Quarter, fromBelow ? -Fixed64.Two : Fixed64.Half, Fixed64.Zero), FixedQuaternion.Identity).Body;
        body.FreezeAxes = BodyFreezeAxes3D.Rotation; body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        body.UseManualGrounding(); body.SleepEnabled = false;
        body.AddLinearImpulse(Vector3d.Right * (Fixed64)64 + (fromBelow ? Vector3d.Up * (Fixed64)64 : Vector3d.Down));
        scene.Context.Simulate(); scene.Context.LateSimulate();
        Assert.Equal((Fixed64)64, body.LinearVelocity.X);
        Assert.True(body.Position3d.X > Fixed64.One);
        Assert.True(fromBelow ? body.Position3d.Y < Fixed64.Zero : body.Position3d.Y > Fixed64.Zero);
    }

    [Fact]
    public void RotatingHullOnJoinedSlabs_ShouldKeepMomentumAtAnUnprovedFrontier()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context, PhysicsRuntimeMode.Mixed);
        using var declaration = scene.Context.Physics2D.CreateStaticSurface(new[] { Rectangle(scene, -4, 0), Rectangle(scene, 0, 4) });
        var source = new LSCuboidCollider { Size = Vector3d.One, Material = PhysicsMaterial.Frictionless };
        SolidBody body = scene.CreateBody(source, new(-Fixed64.Quarter, Fixed64.Zero, Fixed64.Half), FixedQuaternion.Identity).Body;
        body.FreezeAxes = BodyFreezeAxes3D.PositionY | BodyFreezeAxes3D.RotationX | BodyFreezeAxes3D.RotationY;
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous; body.SleepEnabled = false; body.UseManualGrounding();
        body.AddLinearImpulse(Vector3d.Right + Vector3d.Backward * Fixed64.FromFraction(1, 16));
        body.AddAngularImpulse(Vector3d.Forward * Fixed64.FromFraction(1, 64));
        scene.Context.Simulate(); scene.Context.LateSimulate();
        Assert.Equal(Fixed64.One, body.LinearVelocity.X);
        Assert.True(body.AngularVelocity.Z > Fixed64.Zero);
        Assert.True(body.Position3d.Z > Fixed64.Quarter);
    }

    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void RotationalSearchPaddingAtScalarFaces_ShouldRemainConservative(bool maximum)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context, PhysicsRuntimeMode.Mixed);
        Fixed64 x = maximum ? Fixed64.MaxValue - Fixed64.Two : Fixed64.MinValue + Fixed64.Two;
        var center = new Vector3d(x, Fixed64.Zero, Fixed64.Zero);
        FixedBoundBox grid = FixedBoundBox.FromCenterAndScopeClippedToDomain(center, new(16, 8, 8));
        Assert.True(scene.Context.World.TryAddGrid(new GridConfiguration(grid.Min, grid.Max), out _));
        var target = new LSAABBoxCollider2D(new Vector2d(2, 2))
        { MixedHalfThicknessOverride = Fixed64.One, Material = PhysicsMaterial.Frictionless };
        target.InitializeWithNoBody(new TestMatterAgent(scene.Context,
            new FixedTransform(new(x, Fixed64.Zero, -Fixed64.One), FixedQuaternion.Identity, Vector3d.One)));
        using var declaration = scene.Context.Physics2D.CreateStaticSurface(new LSCollider2D[] { target });
        var capsule = new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1), Material = PhysicsMaterial.Frictionless };
        SolidBody body = scene.CreateBody(capsule, center + Vector3d.Forward * Fixed64.Half, FixedQuaternion.Identity).Body;
        Assert.True(declaration.IntersectsMixedContinuous(capsule, (Fixed64)4),
            "An unrepresentable virtual search expansion must retain the unresolved interval.");
        body.FreezeAxes = BodyFreezeAxes3D.RotationX | BodyFreezeAxes3D.RotationY;
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous; body.SleepEnabled = false; body.UseManualGrounding();
        body.AddAngularImpulse(Vector3d.Forward * ((Fixed64)100 / body.EffectiveInverseInertiaTensor.M33));
        body.AddLinearImpulse(Vector3d.Backward * Fixed64.FromFraction(1, 16));
        scene.Context.Simulate(); scene.Context.LateSimulate();
        Assert.Equal(x, body.Position3d.X);
        Assert.NotEqual(FixedQuaternion.Identity, body.Rotation);
        Assert.True(body.AngularVelocity.Z > Fixed64.Zero);
    }

    private static LSCollider2D Rectangle(PhysicsScenarioBuilder scene, int left = -2, int right = 2)
    {
        var target = new LSAABBoxCollider2D(new Vector2d(right - left, 2))
        { MixedHalfThicknessOverride = Fixed64.One, Material = PhysicsMaterial.Frictionless };
        target.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(
            new Vector3d((Fixed64)(left + right) * Fixed64.Half, Fixed64.Zero, -Fixed64.One), FixedQuaternion.Identity, Vector3d.One)));
        return target;
    }

    private static void Configure(GravitasWorldContext context, PhysicsRuntimeMode mode)
    {
        context.Settings.RuntimeMode = mode; context.SetFrameRate(16);
        context.Environment.Gravity = Fixed64.Zero; context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero; context.Environment.MinSpeed = Fixed64.Zero;
        context.Environment.MaxSpeed = context.Environment.MaxFallSpeed = (Fixed64)1024;
    }
}
