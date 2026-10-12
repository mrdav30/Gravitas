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
    [InlineData(false)]
    [InlineData(true)]
    public void ContinuousMode_JoinedMeshBounds_ShouldAdvanceToTheFirstFiniteFace(bool joinedWall)
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
        var mesh = new LSMeshCollider(vertices, triangles, MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation) { Material = PhysicsMaterial.Frictionless };
        scenario.CreateBody(mesh, Vector3d.Zero, FixedQuaternion.Identity, immovable: true);
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
}
