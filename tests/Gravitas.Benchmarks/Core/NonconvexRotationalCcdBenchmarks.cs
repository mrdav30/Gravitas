using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;
using System;
using static Gravitas.Benchmarks.ContinuousCollisionBenchmarkSupport;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class NonconvexRotationalCcdBenchmarks
{
    private static readonly Fixed64 QuarterTurn = FixedMath.DegToRad((Fixed64)90);
    private static readonly FixedQuaternion ClockwiseQuarterTurn =
        FixedQuaternion.FromAxisAngle(Vector3d.Forward, -QuarterTurn);
    private static readonly FixedQuaternion MixedMeshQuarterTurn =
        FixedQuaternion.FromAxisAngle(Vector3d.Up, -QuarterTurn);
    private static readonly Vector3d ConePosition =
        new(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(11, 20), Fixed64.Zero);
    private static readonly Vector3d TriangleCornerPosition = new(9, 9, 0);

    private (GravitasWorldContext Context, SolidBody Source) _joinedMesh;
    private (GravitasWorldContext Context, SolidBody Source) _floorOnly;
    private (GravitasWorldContext Context, SolidBody Source) _emptyTriangleCorner;
    private (GravitasWorldContext Context, SolidBody2D Source) _compoundContact;
    private (GravitasWorldContext Context, SolidBody2D Source) _emptyCompound;
    private (GravitasWorldContext Context, SolidBody Spatial, SolidBody2D Planar) _mixedTriangleContact;
    private (GravitasWorldContext Context, SolidBody Spatial, SolidBody2D Planar) _mixedEmptyTriangle;

    [GlobalSetup]
    public void Setup()
    {
        _joinedMesh = CreateConeMeshScenario(joinedWall: true);
        _floorOnly = CreateConeMeshScenario(joinedWall: false);
        _emptyTriangleCorner = CreateTriangleCornerScenario();
        _compoundContact = CreateCompoundScenario(wallCenter: 2);
        _emptyCompound = CreateCompoundScenario(wallCenter: 4);
        _mixedTriangleContact = CreateMixedTriangleScenario(meshSource: true);
        _mixedEmptyTriangle = CreateMixedTriangleScenario(meshSource: false);
        for (int i = 0; i < 3; i++)
        {
            JoinedFloorAndWallFirstImpact();
            FloorOnlyFirstImpact();
            EmptyFiniteTriangleCorner();
            CompoundWallsFirstImpact2D();
            EmptyCompoundBounds2D();
            MixedFiniteTriangleFirstImpact3D();
            MixedEmptyFiniteTriangleCorner2D();
        }
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _joinedMesh.Context.Dispose();
        _floorOnly.Context.Dispose();
        _emptyTriangleCorner.Context.Dispose();
        _compoundContact.Context.Dispose();
        _emptyCompound.Context.Dispose();
        _mixedTriangleContact.Context.Dispose();
        _mixedEmptyTriangle.Context.Dispose();
    }

    [Benchmark]
    public long JoinedFloorAndWallFirstImpact() => Run3D(_joinedMesh, ConePosition).Z.m_rawValue;

    [Benchmark]
    public long FloorOnlyFirstImpact() => Run3D(_floorOnly, ConePosition).Z.m_rawValue;

    [Benchmark]
    public long EmptyFiniteTriangleCorner() => Run3D(_emptyTriangleCorner, TriangleCornerPosition).Z.m_rawValue;

    [Benchmark]
    public long CompoundWallsFirstImpact2D() => Run2D(_compoundContact).m_rawValue;

    [Benchmark]
    public long EmptyCompoundBounds2D() => Run2D(_emptyCompound).m_rawValue;

    [Benchmark]
    public long MixedFiniteTriangleFirstImpact3D()
    {
        ResetMixedTriangleScenario(_mixedTriangleContact);
        _mixedTriangleContact.Spatial.Agent.Transform.LocalRotation = MixedMeshQuarterTurn;
        _mixedTriangleContact.Context.Simulate();
        _mixedTriangleContact.Context.LateSimulate();
        return _mixedTriangleContact.Spatial.Rotation.Y.m_rawValue;
    }

    [Benchmark]
    public long MixedEmptyFiniteTriangleCorner2D()
    {
        ResetMixedTriangleScenario(_mixedEmptyTriangle);
        _mixedEmptyTriangle.Planar.Agent.Transform.LocalRotationXZRadians = -QuarterTurn;
        _mixedEmptyTriangle.Context.Simulate();
        _mixedEmptyTriangle.Context.LateSimulate();
        return _mixedEmptyTriangle.Planar.Rotation.m_rawValue;
    }

    private static FixedQuaternion Run3D(
        (GravitasWorldContext Context, SolidBody Source) scenario, Vector3d position)
    {
        scenario.Source.ResetPosition(position, FixedQuaternion.Identity);
        scenario.Source.Agent.Transform.LocalRotation = ClockwiseQuarterTurn;
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();
        return scenario.Source.Rotation;
    }

    private static Fixed64 Run2D((GravitasWorldContext Context, SolidBody2D Source) scenario)
    {
        scenario.Source.ResetPosition(Vector2d.Zero);
        scenario.Source.Agent.Transform.LocalRotationXZRadians = QuarterTurn;
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();
        return scenario.Source.Rotation;
    }

    private static (GravitasWorldContext Context, SolidBody Source) CreateConeMeshScenario(bool joinedWall)
    {
        GravitasWorldContext context = CreateContext3D(16, 16);
        context.Environment.DampingFactor = Fixed64.Zero;
        Vector3d[] vertices =
        {
            new(0, 0, -2), new(2, 0, -2), new(2, 0, 2),
            new(0, 0, 2), new(0, 2, -2), new(0, 2, 2)
        };
        int[] triangles = joinedWall
            ? new[] { 0, 3, 1, 1, 3, 2, 0, 4, 3, 3, 4, 5 }
            : new[] { 0, 3, 1, 1, 3, 2 };
        if (!joinedWall) Array.Resize(ref vertices, 4);
        var mesh = new LSMeshCollider(vertices, triangles, MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation) { Material = PhysicsMaterial.Frictionless };
        CreateBody3D(context, mesh, Vector3d.Zero, BodyMotionType.Static);
        SolidBody source = CreateBody3D(context, new LSConeCollider
        {
            Radius = Fixed64.Half,
            Size = Vector3d.One,
            Material = PhysicsMaterial.Frictionless
        }, ConePosition, BodyMotionType.Kinematic);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        return (context, source);
    }

    private static (GravitasWorldContext Context, SolidBody Source) CreateTriangleCornerScenario()
    {
        GravitasWorldContext context = CreateContext3D(16, 16);
        context.Environment.DampingFactor = Fixed64.Zero;
        var mesh = new LSMeshCollider(new[]
        {
            Vector3d.Zero, Vector3d.Right * (Fixed64)10, Vector3d.Up * (Fixed64)10
        }, new[] { 0, 1, 2 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        CreateBody3D(context, mesh, Vector3d.Zero, BodyMotionType.Static);
        SolidBody source = CreateBody3D(context, new LSCuboidCollider
        {
            Size = new Vector3d((Fixed64)12, Fixed64.FromFraction(1, 10), Fixed64.FromFraction(1, 10))
        }, TriangleCornerPosition, BodyMotionType.Kinematic);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        return (context, source);
    }

    private static (GravitasWorldContext Context, SolidBody2D Source) CreateCompoundScenario(int wallCenter)
    {
        GravitasWorldContext context = CreateContext2D(16, 16);
        context.Environment.DampingFactor = Fixed64.Zero;
        SolidBody2D source = CreateThinPolygon2D(context, Vector2d.Zero);
        source.SetMotionType(BodyMotionType.Kinematic);
        source.UseManualGrounding();
        source.Collider.Material = PhysicsMaterial.Frictionless;
        Vector2d wallSize = new((Fixed64)8, Fixed64.FromFraction(1, 5));
        var target = new SolidBody2D(new BenchmarkMatterAgent(context, Vector3d.Zero),
            new LSCompoundCollider2D(
                CompoundColliderPart2D.AABBox(wallSize, new Vector2d(Fixed64.Zero, (Fixed64)wallCenter)),
                CompoundColliderPart2D.AABBox(wallSize, new Vector2d(Fixed64.Zero, (Fixed64)(-wallCenter)))));
        target.Collider.Material = PhysicsMaterial.Frictionless;
        target.Initialize(Vector2d.Zero, motionType: BodyMotionType.Static);
        return (context, source);
    }

    private static (GravitasWorldContext Context, SolidBody Spatial, SolidBody2D Planar)
        CreateMixedTriangleScenario(bool meshSource)
    {
        GravitasWorldContext context = CreateMixedContext(16, 16);
        context.Environment.DampingFactor = Fixed64.Zero;
        var mesh = new LSMeshCollider(new[]
        {
            Vector3d.Zero, Vector3d.Right * (Fixed64)10, Vector3d.Forward * (Fixed64)10
        }, new[] { 0, 1, 2 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        SolidBody spatial = CreateBody3D(context, mesh, Vector3d.Zero,
            meshSource ? BodyMotionType.Kinematic : BodyMotionType.Static);
        spatial.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        var polygon = new LSPolygonCollider2D(
            new Vector2d((Fixed64)(-6), Fixed64.FromFraction(-1, 20)),
            new Vector2d((Fixed64)6, Fixed64.FromFraction(-1, 20)),
            new Vector2d((Fixed64)6, Fixed64.FromFraction(1, 20)),
            new Vector2d((Fixed64)(-6), Fixed64.FromFraction(1, 20)))
        {
            Material = PhysicsMaterial.Frictionless
        };
        var planar = new SolidBody2D(new BenchmarkMatterAgent(context, new Vector3d(9, 0, 9)), polygon)
        {
            Mass = Fixed64.One,
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous
        };
        planar.Initialize(new Vector2d(9, 9),
            motionType: meshSource ? BodyMotionType.Static : BodyMotionType.Kinematic);
        planar.UseManualGrounding();
        return (context, spatial, planar);
    }

    private static void ResetMixedTriangleScenario(
        (GravitasWorldContext Context, SolidBody Spatial, SolidBody2D Planar) scenario)
    {
        scenario.Spatial.ResetPosition(Vector3d.Zero, FixedQuaternion.Identity);
        scenario.Planar.ResetPosition(new Vector2d(9, 9));
    }

    private static SolidBody CreateBody3D(GravitasWorldContext context, LSCollider collider,
        Vector3d position, BodyMotionType motionType)
    {
        var body = new SolidBody(new BenchmarkMatterAgent(context, position), collider) { Mass = Fixed64.One };
        body.Initialize(position, FixedQuaternion.Identity, motionType);
        body.UseManualGrounding();
        return body;
    }
}
