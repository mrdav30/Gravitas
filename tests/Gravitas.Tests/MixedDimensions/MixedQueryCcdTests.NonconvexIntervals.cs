using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedQueryCcdTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FullLoop_NonconvexIntervalsMixed_EmptyTriangleCorner_ShouldPreserveFiniteFeatureTiming(
        bool meshSource)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.SetFrameRate(1);
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.World.TryAddGrid(new GridConfiguration(new Vector3d(-16, -4, -16),
            new Vector3d(16, 4, 16)), out _).Should().BeTrue();
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        var mesh = new LSMeshCollider(new[]
        {
            Vector3d.Zero, Vector3d.Right * (Fixed64)10, Vector3d.Forward * (Fixed64)10
        }, new[] { 0, 1, 2 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        SolidBody spatial = CreateBody3D(context, mesh, Vector3d.Zero,
            immovable: !meshSource, isKinematic: meshSource).Body;
        spatial.UseManualGrounding();
        Vector2d[] vertices =
        {
            new((Fixed64)(-6), Fixed64.FromFraction(-1, 20)),
            new((Fixed64)6, Fixed64.FromFraction(-1, 20)),
            new((Fixed64)6, Fixed64.FromFraction(1, 20)),
            new((Fixed64)(-6), Fixed64.FromFraction(1, 20))
        };
        var planarCollider = new LSPolygonCollider2D(vertices) { Material = PhysicsMaterial.Frictionless };
        Vector2d planarPosition = new(9, 9);
        var planar = new SolidBody2D(new TestMatterAgent(context,
            new FixedTransform(planarPosition.ToVector3d(Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One)),
            planarCollider) { Mass = Fixed64.One };
        planar.Initialize(planarPosition, motionType: meshSource ? BodyMotionType.Static : BodyMotionType.Kinematic);
        planar.UseManualGrounding();
        mesh.Bounds.Intersects(planarCollider.MixedBounds3D).Should().BeTrue();
        CollisionDetectionMixed.TryCollide(mesh, planarCollider, out _).Should().BeFalse();
        FixedQuaternion meshTarget = FixedQuaternion.FromAxisAngle(Vector3d.Up, -RotationalMixedQuarterTurn);
        if (meshSource)
        {
            spatial.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            spatial.SetRotation(FixedQuaternion.FromAxisAngle(Vector3d.Up,
                FixedMath.DegToRad((Fixed64)(-64))));
            CollisionDetectionMixed.TryCollide(mesh, planarCollider, out _).Should().BeTrue();
            spatial.SetRotation(FixedQuaternion.Identity);
            spatial.Agent.Transform.LocalRotation = meshTarget;
        }
        else
        {
            planar.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            planar.Agent.Transform.LocalRotationXZRadians = -RotationalMixedQuarterTurn;
        }

        context.Simulate();
        context.LateSimulate();

        if (meshSource)
        {
            // The rotating finite triangle's tip reaches the stationary blade near 64 degrees.
            Vector3d right = spatial.Rotation * Vector3d.Right;
            FixedMath.RadToDeg(FixedMath.Atan2(right.Z, right.X)).Should().BeInRange((Fixed64)63, (Fixed64)65);
            spatial.LastContinuousCollisionToiIterationCount.Should().Be(1);
        }
        else
        {
            // Clockwise blade rotation remains outside x+z=10 throughout the authored quarter-turn.
            planar.Rotation.Should().Be(-RotationalMixedQuarterTurn);
            planar.LastContinuousCollisionToiIterationCount.Should().Be(0);
        }
        spatial.Position3d.Should().Be(Vector3d.Zero);
        planar.Position.Should().Be(planarPosition);
    }

    [Theory]
    [InlineData(false, false, 2)]
    [InlineData(false, true, 2)]
    [InlineData(true, false, 2)]
    [InlineData(true, true, 2)]
    [InlineData(false, false, 4)]
    [InlineData(false, true, 4)]
    [InlineData(true, false, 4)]
    [InlineData(true, true, 4)]
    public void FullLoop_NonconvexIntervalsMixed_ShouldAdvanceThroughEmptyCompoundBounds(
        bool source3D, bool compoundSource, int wallCenter)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        SolidBody? mover3D = null;
        SolidBody2D? mover2D = null;
        LSCollider spatial;
        LSCollider2D planar;
        Vector3d spatialWallSize = new((Fixed64)8, Fixed64.One, Fixed64.FromFraction(1, 5));
        Vector2d planarWallSize = new((Fixed64)8, Fixed64.FromFraction(1, 5));
        if (source3D)
        {
            Vector3d bladeSize = new((Fixed64)6, Fixed64.One, Fixed64.FromFraction(1, 5));
            spatial = compoundSource
                ? new LSCompoundCollider(CompoundColliderPart.Cuboid(bladeSize, Vector3d.Zero))
                : new LSCuboidCollider { Size = bladeSize };
            mover3D = CreateBody3D(context, spatial, Vector3d.Zero, isKinematic: true).Body;
            mover3D.Collider.Material = PhysicsMaterial.Frictionless;
            mover3D.UseManualGrounding();
            mover3D.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            planar = CreateBodylessCompound2D(context, Vector2d.Zero,
                CompoundColliderPart2D.AABBox(planarWallSize,
                    new Vector2d(Fixed64.Zero, (Fixed64)wallCenter), PhysicsMaterial.Frictionless),
                CompoundColliderPart2D.AABBox(planarWallSize,
                    new Vector2d(Fixed64.Zero, (Fixed64)(-wallCenter)), PhysicsMaterial.Frictionless));
        }
        else
        {
            planar = CreateNonconvexIntervalBlade2D(compoundSource);
            mover2D = CreateNonconvexIntervalBody2D(context, planar);
            mover2D.UseManualGrounding();
            mover2D.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            spatial = CreateBodyless3D(context, new LSCompoundCollider(
                CompoundColliderPart.Cuboid(spatialWallSize,
                    new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)wallCenter), PhysicsMaterial.Frictionless),
                CompoundColliderPart.Cuboid(spatialWallSize,
                    new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)(-wallCenter)), PhysicsMaterial.Frictionless)),
                Vector3d.Zero);
        }
        spatial.Bounds.Intersects(planar.MixedBounds3D).Should().BeTrue();
        CollisionDetectionMixed.TryCollide(spatial, planar, out _).Should().BeFalse();
        Fixed64 lowAngle = FixedMath.DegToRad((Fixed64)30);
        Fixed64 highAngle = FixedMath.DegToRad((Fixed64)45);
        if (source3D) mover3D!.SetRotation(FixedQuaternion.FromAxisAngle(Vector3d.Up, lowAngle));
        else mover2D!.SetRotation(lowAngle);
        CollisionDetectionMixed.TryCollide(spatial, planar, out _).Should().BeFalse();
        if (source3D) mover3D!.SetRotation(FixedQuaternion.FromAxisAngle(Vector3d.Up, highAngle));
        else mover2D!.SetRotation(highAngle);
        CollisionDetectionMixed.TryCollide(spatial, planar, out _).Should().Be(wallCenter == 2);
        if (source3D)
        {
            mover3D!.SetRotation(FixedQuaternion.Identity);
            mover3D.Agent.Transform.LocalRotation = RotationalMixedQuarterTurn3D;
        }
        else
        {
            mover2D!.SetRotation(Fixed64.Zero);
            mover2D.Agent.Transform.LocalRotationXZRadians = RotationalMixedQuarterTurn;
        }

        context.Simulate();
        context.LateSimulate();

        Fixed64 angle = source3D
            ? FixedQuaternion.Angle(FixedQuaternion.Identity, mover3D!.Rotation)
            : FixedMath.RadToDeg(mover2D!.Rotation);
        int iterations = source3D
            ? mover3D!.LastContinuousCollisionToiIterationCount
            : mover2D!.LastContinuousCollisionToiIterationCount;
        if (wallCenter == 2)
        {
            angle.Should().BeInRange((Fixed64)30, (Fixed64)45);
            iterations.Should().BeGreaterThan(0);
        }
        else
        {
            if (source3D) mover3D!.Rotation.Should().Be(RotationalMixedQuarterTurn3D);
            else mover2D!.Rotation.Should().Be(RotationalMixedQuarterTurn);
            iterations.Should().Be(0);
        }
        if (source3D) mover3D!.Position3d.Should().Be(Vector3d.Zero);
        else mover2D!.Position.Should().Be(Vector2d.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FullLoop_NonconvexIntervalsMixed_ShouldUseMovingTargetPose(bool approaching)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        SolidBody2D source = CreateNonconvexIntervalBody2D(context,
            CreateNonconvexIntervalBlade2D(compoundSource: false));
        source.UseManualGrounding();
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector3d wallSize = new((Fixed64)8, Fixed64.One, Fixed64.FromFraction(1, 5));
        SolidBody target = CreateBody3D(context, new LSCompoundCollider(
            CompoundColliderPart.Cuboid(wallSize, Vector3d.Forward * Fixed64.Two),
            CompoundColliderPart.Cuboid(wallSize, -Vector3d.Forward * Fixed64.Two)),
            approaching ? Vector3d.Up * (Fixed64)4 : Vector3d.Zero,
            isKinematic: true).Body;
        target.Collider.Material = PhysicsMaterial.Frictionless;
        target.UseManualGrounding();
        CollisionDetectionMixed.TryCollide(target.Collider, source.Collider, out _).Should().BeFalse();
        target.Agent.Transform.LocalPosition = approaching ? Vector3d.Zero : Vector3d.Up * (Fixed64)4;
        source.Agent.Transform.LocalRotationXZRadians = RotationalMixedQuarterTurn;

        context.Simulate();
        context.LateSimulate();

        if (approaching)
        {
            source.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
        }
        else
        {
            // The departing walls clear the finite slab before the blade can reach either wall in X/Z.
            source.Rotation.Should().Be(RotationalMixedQuarterTurn);
            source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        }
    }

    private static LSCollider2D CreateNonconvexIntervalBlade2D(bool compoundSource)
    {
        Vector2d[] vertices =
        {
            new((Fixed64)(-3), Fixed64.FromFraction(-1, 10)),
            new((Fixed64)3, Fixed64.FromFraction(-1, 10)),
            new((Fixed64)3, Fixed64.FromFraction(1, 10)),
            new((Fixed64)(-3), Fixed64.FromFraction(1, 10))
        };
        return compoundSource
            ? new LSCompoundCollider2D(CompoundColliderPart2D.ConvexPolygon(vertices, Vector2d.Zero))
            : new LSPolygonCollider2D(vertices);
    }

    private static SolidBody2D CreateNonconvexIntervalBody2D(GravitasWorldContext context, LSCollider2D collider)
    {
        collider.Material = PhysicsMaterial.Frictionless;
        var body = new SolidBody2D(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)), collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(Vector2d.Zero, motionType: BodyMotionType.Kinematic);
        return body;
    }
}
