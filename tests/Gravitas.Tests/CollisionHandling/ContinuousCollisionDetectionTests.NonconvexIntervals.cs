using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed partial class ContinuousCollisionDetectionTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContinuousMode_EmptyTriangleBoundsCorner_ShouldAdvanceThroughSeparatedMotion(bool meshSource)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        var mesh = new LSMeshCollider(new[] { Vector3d.Zero, Vector3d.Right * (Fixed64)10,
            Vector3d.Up * (Fixed64)10 }, new[] { 0,1,2 }, MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);
        var cuboid = new LSCuboidCollider { Size = new Vector3d((Fixed64)12,
            Fixed64.FromFraction(1,10), Fixed64.FromFraction(1,10)) };
        SolidBody meshBody = scenario.CreateBody(mesh, Vector3d.Zero, FixedQuaternion.Identity,
            immovable: !meshSource, isKinematic: meshSource).Body;
        SolidBody boxBody = scenario.CreateBody(cuboid, new Vector3d(9,9,0), FixedQuaternion.Identity,
            immovable: meshSource, isKinematic: !meshSource).Body;
        SolidBody source = meshSource ? meshBody : boxBody;
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.UseManualGrounding();
        mesh.Bounds.Intersects(cuboid.Bounds).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(scenario.CreatePair(mesh, cuboid)).Should().BeFalse();
        if (meshSource)
        {
            source.SetRotation(FixedQuaternion.FromAxisAngle(Vector3d.Forward, FixedMath.DegToRad((Fixed64)64)));
            CollisionDetection.DoCollisionCheck(scenario.CreatePair(mesh, cuboid)).Should().BeTrue();
            source.SetRotation(FixedQuaternion.Identity);
        }
        FixedQuaternion target = FixedQuaternion.FromAxisAngle(Vector3d.Forward,
            (meshSource ? Fixed64.One : -Fixed64.One) * Fixed64.Pi * Fixed64.Half);
        source.Agent.Transform.LocalRotation = target;

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        // The clockwise blade stays beyond x+y=10; the rotating triangle's
        // tip reaches the stationary blade later, near 64 degrees.
        if (meshSource)
        {
            Vector3d right = source.Rotation * Vector3d.Right;
            FixedMath.RadToDeg(FixedMath.Atan2(right.Y, right.X)).Should().BeInRange((Fixed64)63, (Fixed64)65);
            source.LastContinuousCollisionToiIterationCount.Should().Be(1);
        }
        else
        {
            source.Rotation.Should().Be(target);
            source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        }
    }

    [Theory]
    [InlineData(false, MeshColliderMode.Concave, false)]
    [InlineData(true, MeshColliderMode.Concave, false)]
    [InlineData(false, MeshColliderMode.Convex, false)]
    [InlineData(false, MeshColliderMode.Concave, true)]
    [InlineData(true, MeshColliderMode.Concave, true)]
    public void ContinuousMode_JoinedMeshBounds_ShouldAdvanceToTheFirstFiniteFace(
        bool joinedWall, MeshColliderMode mode, bool declaredSurface)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        var vertices = new[]
        {
            new Vector3d(0,0,-2), new Vector3d(2,0,-2),
            new Vector3d(2,0,2), new Vector3d(0,0,2),
            new Vector3d(0,2,-2), new Vector3d(0,2,2)
        };
        int[] triangles = joinedWall
            ? new[] { 0,3,1, 1,3,2, 0,4,3, 3,4,5 }
            : new[] { 0,3,1, 1,3,2 };
        if (!joinedWall) System.Array.Resize(ref vertices, 4);
        var mesh = new LSMeshCollider(vertices, triangles, mode,
            MeshInertiaPolicy.SurfaceApproximation) { Material = PhysicsMaterial.Frictionless };
        scenario.CreateBody(mesh, Vector3d.Zero, FixedQuaternion.Identity, immovable: true);
        using var surface = declaredSurface ? scenario.Context.Physics.CreateStaticSurface(new[] { mesh }) : null;
        var cone = new LSConeCollider { Radius = Fixed64.Half, Size = Vector3d.One,
            Material = PhysicsMaterial.Frictionless };
        SolidBody body = scenario.CreateBody(cone,
            new Vector3d(Fixed64.FromFraction(3,5), Fixed64.FromFraction(11,20), Fixed64.Zero),
            FixedQuaternion.Identity, isKinematic: true).Body;
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        body.UseManualGrounding();

        CollisionPair pair = scenario.CreatePair(mesh, cone);
        CollisionDetection.DoCollisionCheck(pair).Should().BeFalse();
        body.SetRotation(FixedQuaternion.FromAxisAngle(Vector3d.Forward, FixedMath.DegToRad((Fixed64)(-8))));
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        body.SetRotation(FixedQuaternion.Identity);
        body.Agent.Transform.LocalRotation = FixedQuaternion.FromAxisAngle(Vector3d.Forward,
            FixedMath.DegToRad((Fixed64)(-90)));

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        // The lowest support is -1/2*(cos(theta)+sin(theta)); the floor is
        // 0.55 below the center, so its first contact lies between 5 and 8 degrees.
        Fixed64 clockwiseAngle = -FixedMath.RadToDeg(FixedMath.Atan2(
            body.Rotation.Rotate(Vector3d.Right).Y, body.Rotation.Rotate(Vector3d.Right).X));
        clockwiseAngle.Should().BeInRange((Fixed64)5, (Fixed64)8);
        body.LastContinuousCollisionToiIterationCount.Should().Be(1);
    }
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContinuousMode_MeshConeRotation_ShouldTransferMomentumAtTheRetainedWitness(bool meshSource)
    {
        using PhysicsScenarioBuilder scene = CreateCcdScenario();
        var mesh = new LSMeshCollider(new[] { new Vector3d(0, 0, -2), new Vector3d(2, 0, -2),
            new Vector3d(2, 0, 2), new Vector3d(0, 0, 2) }, new[] { 0, 3, 1, 1, 3, 2 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
            { Material = PhysicsMaterial.Frictionless };
        SolidBody meshBody = scene.CreateBody(mesh, Vector3d.Zero, FixedQuaternion.Identity,
            isKinematic: meshSource).Body;
        var cone = new LSConeCollider { Radius = Fixed64.Half, Size = Vector3d.One,
            Material = PhysicsMaterial.Frictionless };
        SolidBody coneBody = scene.CreateBody(cone,
            new Vector3d(Fixed64.FromFraction(3, 5), Fixed64.Half, Fixed64.Zero),
            FixedQuaternion.Identity, isKinematic: !meshSource).Body;
        meshBody.ContinuousCollisionMode = coneBody.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        meshBody.UseManualGrounding(); coneBody.UseManualGrounding();
        SolidBody source = meshSource ? meshBody : coneBody;
        SolidBody target = meshSource ? coneBody : meshBody;
        FixedQuaternion authored = FixedQuaternion.FromAxisAngle(Vector3d.Forward,
            FixedMath.DegToRad((Fixed64)(meshSource ? 90 : -90)));
        source.Agent.Transform.LocalRotation = authored;
        target.Sleep();

        scene.Context.Simulate();
        scene.Context.LateSimulate();

        source.Rotation.Should().Be(authored);
        target.IsSleeping.Should().BeFalse();
        (target.LinearVelocity.MagnitudeSquared + target.AngularVelocity.MagnitudeSquared)
            .Should().BeGreaterThan(Fixed64.Zero);
        source.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void ContinuousMode_ConeInsideClosedConvexMesh_ShouldRetainVolumeAdmission()
    {
        using PhysicsScenarioBuilder scene = CreateCcdScenario();
        LSMeshCollider mesh = MeshTestFixtures.CreateConvexCube();
        var meshTransform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One * (Fixed64)4);
        mesh.InitializeWithNoBody(new TestMatterAgent(scene.Context, meshTransform));
        var cone = new LSConeCollider { Radius = Fixed64.Half, Size = Vector3d.One };
        SolidBody body = scene.CreateBody(cone, Vector3d.Zero, FixedQuaternion.Identity, isKinematic: true).Body;
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        body.UseManualGrounding();
        CollisionDetection.IntersectsMeshConeSurface(mesh, cone, scene.Context.CollisionScratch).Should().BeFalse();
        CollisionDetection.DoCollisionCheck(scene.CreatePair(mesh, cone)).Should().BeTrue();
        body.ResetPosition(new Vector3d(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(51, 20), Fixed64.Zero),
            FixedQuaternion.Identity);
        FixedQuaternion authored = FixedQuaternion.FromAxisAngle(Vector3d.Forward,
            FixedMath.DegToRad((Fixed64)(-90)));
        body.Agent.Transform.LocalRotation = authored;

        scene.Context.Simulate();
        scene.Context.LateSimulate();

        body.Rotation.Should().NotBe(authored);
        body.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void ContinuousMode_DynamicConeRotation_ShouldStopAtTheRetainedFiniteWitness()
    {
        using PhysicsScenarioBuilder scene = CreateCcdScenario();
        LSMeshCollider mesh = MeshTestFixtures.CreateConvexQuadFloor(MeshColliderMode.Concave);
        mesh.Material = PhysicsMaterial.Frictionless;
        scene.InitializeStaticCollider(mesh, Vector3d.Zero);
        var cone = new LSConeCollider { Radius = Fixed64.Half, Size = Vector3d.One,
            Material = PhysicsMaterial.Frictionless };
        SolidBody body = scene.CreateBody(cone, new Vector3d(Fixed64.FromFraction(3, 5),
            Fixed64.Half, Fixed64.Zero), FixedQuaternion.Identity).Body;
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        body.UseManualGrounding();
        body.ApplyCollisionAngularVelocityDelta(Vector3d.Forward * (Fixed64)(-2));

        scene.Context.Simulate();
        scene.Context.LateSimulate();

        // The same-dimensional static-target contract stops rotation at a closing witness.
        body.AngularVelocity.Should().Be(Vector3d.Zero);
        body.LinearVelocity.Should().Be(Vector3d.Zero);
        body.Position3d.Y.Should().Be(Fixed64.Half);
        body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        body.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
        CollisionDetection.DoCollisionCheck(scene.CreatePair(mesh, cone)).Should().BeTrue();
    }

}
