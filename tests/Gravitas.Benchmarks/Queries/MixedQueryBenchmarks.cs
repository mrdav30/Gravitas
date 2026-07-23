using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using SwiftCollections;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class MixedQueryBenchmarks
{
    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);

    private GravitasWorldContext _sparseContext;
    private GravitasWorldContext _denseContext;
    private GravitasWorldContext _cornerMissContext;
    private GravitasWorldContext _denseCuboidContext;
    private GravitasWorldContext _denseCapsuleContext;
    private GravitasWorldContext _denseCylinderContext;
    private GravitasWorldContext _denseRotatedCapsuleContext;
    private GravitasWorldContext _denseRotatedCylinderContext;
    private GravitasWorldContext _denseRotatedConeContext;
    private GravitasWorldContext _densePartiallyClippedCapsuleContext;
    private GravitasWorldContext _densePartiallyClippedCylinderContext;
    private GravitasWorldContext _densePartiallyClippedConeContext;
    private GravitasWorldContext _denseLongRotatedConeContext;
    private GravitasWorldContext _denseWideRotatedConeContext;
    private GravitasWorldContext _sparseMeshContext;
    private GravitasWorldContext _denseMeshContext;
    private GravitasWorldContext _falsePositiveMeshContext;
    private GravitasWorldContext _sparseCompoundContext;
    private GravitasWorldContext _denseCompoundContext;
    private GravitasWorldContext _falsePositiveCompoundContext;
    private SwiftList<PhysicsMixedHit> _hits;
    private Vector2d _sparseEnd;
    private Vector2d _denseEnd;
    private Vector2d _cornerMissEnd;

    [Params(64, 1024)]
    public int ColliderCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        int sparseExtentX = 16 + (ColliderCount * 3);
        int denseExtentX = 16 + ColliderCount;

        _sparseContext = BenchmarkPhysicsScene.CreateMixedContext(sparseExtentX, 16, clearAllPools: true);
        _denseContext = BenchmarkPhysicsScene.CreateMixedContext(denseExtentX, 16);
        _cornerMissContext = BenchmarkPhysicsScene.CreateMixedContext(denseExtentX, 16);
        _denseCuboidContext = BenchmarkPhysicsScene.CreateMixedContext(denseExtentX, 16);
        _denseCapsuleContext = BenchmarkPhysicsScene.CreateMixedContext(denseExtentX, 16);
        _denseCylinderContext = BenchmarkPhysicsScene.CreateMixedContext(denseExtentX, 16);
        _denseRotatedCapsuleContext = BenchmarkPhysicsScene.CreateMixedContext(denseExtentX, 16);
        _denseRotatedCylinderContext = BenchmarkPhysicsScene.CreateMixedContext(denseExtentX, 16);
        _denseRotatedConeContext = BenchmarkPhysicsScene.CreateMixedContext(denseExtentX, 16);
        _densePartiallyClippedCapsuleContext = BenchmarkPhysicsScene.CreateMixedContext(denseExtentX, 16);
        _densePartiallyClippedCylinderContext = BenchmarkPhysicsScene.CreateMixedContext(denseExtentX, 16);
        _densePartiallyClippedConeContext = BenchmarkPhysicsScene.CreateMixedContext(denseExtentX, 16);
        _denseLongRotatedConeContext = BenchmarkPhysicsScene.CreateMixedContext(denseExtentX, 16);
        _denseWideRotatedConeContext = BenchmarkPhysicsScene.CreateMixedContext(denseExtentX, 16);
        _sparseMeshContext = BenchmarkPhysicsScene.CreateMixedContext(sparseExtentX, 16);
        _denseMeshContext = BenchmarkPhysicsScene.CreateMixedContext(denseExtentX, 16);
        _falsePositiveMeshContext = BenchmarkPhysicsScene.CreateMixedContext(denseExtentX, 16);
        _sparseCompoundContext = BenchmarkPhysicsScene.CreateMixedContext(sparseExtentX, 16);
        _denseCompoundContext = BenchmarkPhysicsScene.CreateMixedContext(denseExtentX, 16);
        _falsePositiveCompoundContext = BenchmarkPhysicsScene.CreateMixedContext(denseExtentX, 16);
        _hits = new SwiftList<PhysicsMixedHit>(ColliderCount);
        FixedQuaternion rotatedCurvedTarget = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90);
        FixedQuaternion partiallyClippedCurvedTarget = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)45);

        for (int i = 0; i < ColliderCount; i++)
        {
            Vector3d densePosition = new((Fixed64)i, Fixed64.Zero, DenseZForIndex(i));
            Vector3d sparsePosition = new((Fixed64)(i * 3), Fixed64.Zero, Fixed64.Zero);
            _ = CreateSphere3D(_sparseContext, new Vector3d((Fixed64)(i * 3), Fixed64.Zero, Fixed64.Zero));
            _ = CreateSphere3D(_denseContext, densePosition);
            _ = CreateSphere3D(
                _cornerMissContext,
                new Vector3d((Fixed64)i, Fixed64.FromFraction(9, 10), Fixed64.Zero));
            _ = CreateStatic3D(_denseCuboidContext, new LSCuboidCollider(), densePosition);
            _ = CreateStatic3D(
                _denseCapsuleContext,
                new LSCapsuleCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
                densePosition);
            _ = CreateStatic3D(
                _denseCylinderContext,
                new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
                densePosition);
            _ = CreateStatic3D(
                _denseRotatedCapsuleContext,
                new LSCapsuleCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
                densePosition,
                rotatedCurvedTarget);
            _ = CreateStatic3D(
                _denseRotatedCylinderContext,
                new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
                densePosition,
                rotatedCurvedTarget);
            _ = CreateStatic3D(
                _denseRotatedConeContext,
                new LSConeCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
                densePosition,
                rotatedCurvedTarget);
            _ = CreateStatic3D(
                _densePartiallyClippedCapsuleContext,
                new LSCapsuleCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
                densePosition,
                partiallyClippedCurvedTarget);
            _ = CreateStatic3D(
                _densePartiallyClippedCylinderContext,
                new LSCylinderCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
                densePosition,
                partiallyClippedCurvedTarget);
            _ = CreateStatic3D(
                _densePartiallyClippedConeContext,
                new LSConeCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
                densePosition,
                partiallyClippedCurvedTarget);
            _ = CreateStatic3D(
                _denseLongRotatedConeContext,
                CreateCone(Fixed64.FromFraction(1, 4), (Fixed64)6),
                densePosition,
                rotatedCurvedTarget);
            _ = CreateStatic3D(
                _denseWideRotatedConeContext,
                CreateCone(Fixed64.One, (Fixed64)2),
                densePosition,
                rotatedCurvedTarget);
            _ = CreateStatic3D(_sparseMeshContext, CreateVerticalQuadMesh(), sparsePosition);
            _ = CreateStatic3D(_denseMeshContext, CreateVerticalQuadMesh(), densePosition);
            _ = CreateStatic3D(_falsePositiveMeshContext, CreateSlabClippedProxyOnlyTriangleMesh(), densePosition);
            _ = CreateStatic3D(_sparseCompoundContext, CreateCompoundTarget(), sparsePosition);
            _ = CreateStatic3D(_denseCompoundContext, CreateCompoundTarget(), densePosition);
            _ = CreateStatic3D(_falsePositiveCompoundContext, CreateProxyOnlyCompoundTarget(), densePosition);
        }

        _sparseContext.Simulate();
        _denseContext.Simulate();
        _cornerMissContext.Simulate();
        _denseCuboidContext.Simulate();
        _denseCapsuleContext.Simulate();
        _denseCylinderContext.Simulate();
        _denseRotatedCapsuleContext.Simulate();
        _denseRotatedCylinderContext.Simulate();
        _denseRotatedConeContext.Simulate();
        _densePartiallyClippedCapsuleContext.Simulate();
        _densePartiallyClippedCylinderContext.Simulate();
        _densePartiallyClippedConeContext.Simulate();
        _denseLongRotatedConeContext.Simulate();
        _denseWideRotatedConeContext.Simulate();
        _sparseMeshContext.Simulate();
        _denseMeshContext.Simulate();
        _falsePositiveMeshContext.Simulate();
        _sparseCompoundContext.Simulate();
        _denseCompoundContext.Simulate();
        _falsePositiveCompoundContext.Simulate();

        _sparseEnd = new Vector2d((Fixed64)(ColliderCount * 3), Fixed64.Zero);
        _denseEnd = new Vector2d((Fixed64)ColliderCount, Fixed64.Zero);
        _cornerMissEnd = new Vector2d((Fixed64)ColliderCount, Fixed64.FromFraction(9, 10));
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _sparseContext?.Dispose();
        _denseContext?.Dispose();
        _cornerMissContext?.Dispose();
        _denseCuboidContext?.Dispose();
        _denseCapsuleContext?.Dispose();
        _denseCylinderContext?.Dispose();
        _denseRotatedCapsuleContext?.Dispose();
        _denseRotatedCylinderContext?.Dispose();
        _denseRotatedConeContext?.Dispose();
        _densePartiallyClippedCapsuleContext?.Dispose();
        _densePartiallyClippedCylinderContext?.Dispose();
        _densePartiallyClippedConeContext?.Dispose();
        _denseLongRotatedConeContext?.Dispose();
        _denseWideRotatedConeContext?.Dispose();
        _sparseMeshContext?.Dispose();
        _denseMeshContext?.Dispose();
        _falsePositiveMeshContext?.Dispose();
        _sparseCompoundContext?.Dispose();
        _denseCompoundContext?.Dispose();
        _falsePositiveCompoundContext?.Dispose();
        _sparseContext = null;
        _denseContext = null;
        _cornerMissContext = null;
        _denseCuboidContext = null;
        _denseCapsuleContext = null;
        _denseCylinderContext = null;
        _denseRotatedCapsuleContext = null;
        _denseRotatedCylinderContext = null;
        _denseRotatedConeContext = null;
        _densePartiallyClippedCapsuleContext = null;
        _densePartiallyClippedCylinderContext = null;
        _densePartiallyClippedConeContext = null;
        _denseLongRotatedConeContext = null;
        _denseWideRotatedConeContext = null;
        _sparseMeshContext = null;
        _denseMeshContext = null;
        _falsePositiveMeshContext = null;
        _sparseCompoundContext = null;
        _denseCompoundContext = null;
        _falsePositiveCompoundContext = null;
        _hits = null;
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_SparseSphereTargets()
    {
        return _sparseContext.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            _sparseEnd,
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            _hits);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseSphereTargets()
    {
        return _denseContext.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            _denseEnd,
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            _hits);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_CornerProxyMissSphereTargets()
    {
        return _cornerMissContext.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-2), Fixed64.FromFraction(9, 10)),
            _cornerMissEnd,
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            _hits);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseCuboidTargets()
    {
        return SweepCircleAgainstDensePrimitive3D(_denseCuboidContext);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseCuboidTargets_CandidateCount()
    {
        _ = SweepCircleAgainstDensePrimitive3D(_denseCuboidContext);
        return _denseCuboidContext.QueryMixed.LastQueryCandidateCount;
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseCapsuleTargets()
    {
        return SweepCircleAgainstDensePrimitive3D(_denseCapsuleContext);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseCapsuleTargets_CandidateCount()
    {
        _ = SweepCircleAgainstDensePrimitive3D(_denseCapsuleContext);
        return _denseCapsuleContext.QueryMixed.LastQueryCandidateCount;
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseCylinderTargets()
    {
        return SweepCircleAgainstDensePrimitive3D(_denseCylinderContext);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseCylinderTargets_CandidateCount()
    {
        _ = SweepCircleAgainstDensePrimitive3D(_denseCylinderContext);
        return _denseCylinderContext.QueryMixed.LastQueryCandidateCount;
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseRotatedCapsuleTargets()
    {
        return SweepCircleAgainstDensePrimitive3D(_denseRotatedCapsuleContext);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseRotatedCapsuleTargets_CandidateCount()
    {
        _ = SweepCircleAgainstDensePrimitive3D(_denseRotatedCapsuleContext);
        return _denseRotatedCapsuleContext.QueryMixed.LastQueryCandidateCount;
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseRotatedCylinderTargets()
    {
        return SweepCircleAgainstDensePrimitive3D(_denseRotatedCylinderContext);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseRotatedCylinderTargets_CandidateCount()
    {
        _ = SweepCircleAgainstDensePrimitive3D(_denseRotatedCylinderContext);
        return _denseRotatedCylinderContext.QueryMixed.LastQueryCandidateCount;
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseRotatedConeTargets()
    {
        return SweepCircleAgainstDensePrimitive3D(_denseRotatedConeContext);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseRotatedConeTargets_CandidateCount()
    {
        _ = SweepCircleAgainstDensePrimitive3D(_denseRotatedConeContext);
        return _denseRotatedConeContext.QueryMixed.LastQueryCandidateCount;
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DensePartiallyClippedCapsuleTargets()
    {
        return SweepCircleAgainstDensePrimitive3D(_densePartiallyClippedCapsuleContext);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DensePartiallyClippedCylinderTargets()
    {
        return SweepCircleAgainstDensePrimitive3D(_densePartiallyClippedCylinderContext);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DensePartiallyClippedConeTargets()
    {
        return SweepCircleAgainstDensePrimitive3D(_densePartiallyClippedConeContext);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseLongRotatedConeTargets()
    {
        return SweepCircleAgainstDensePrimitive3D(_denseLongRotatedConeContext);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseWideRotatedConeTargets()
    {
        return SweepCircleAgainstDensePrimitive3D(_denseWideRotatedConeContext);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_SparseMeshTargets()
    {
        return SweepCircleAgainst3D(_sparseMeshContext, _sparseEnd);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_SparseMeshTargets_CandidateCount()
    {
        _ = SweepCircleAgainst3D(_sparseMeshContext, _sparseEnd);
        return _sparseMeshContext.QueryMixed.LastQueryCandidateCount;
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseMeshTargets()
    {
        return SweepCircleAgainst3D(_denseMeshContext, _denseEnd);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseMeshTargets_CandidateCount()
    {
        _ = SweepCircleAgainst3D(_denseMeshContext, _denseEnd);
        return _denseMeshContext.QueryMixed.LastQueryCandidateCount;
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_FalsePositiveMeshTargets()
    {
        return SweepCircleAgainst3D(_falsePositiveMeshContext, _denseEnd);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_FalsePositiveMeshTargets_CandidateCount()
    {
        _ = SweepCircleAgainst3D(_falsePositiveMeshContext, _denseEnd);
        return _falsePositiveMeshContext.QueryMixed.LastQueryCandidateCount;
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_SparseCompoundTargets()
    {
        return SweepCircleAgainst3D(_sparseCompoundContext, _sparseEnd);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_SparseCompoundTargets_CandidateCount()
    {
        _ = SweepCircleAgainst3D(_sparseCompoundContext, _sparseEnd);
        return _sparseCompoundContext.QueryMixed.LastQueryCandidateCount;
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseCompoundTargets()
    {
        return SweepCircleAgainst3D(_denseCompoundContext, _denseEnd);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_DenseCompoundTargets_CandidateCount()
    {
        _ = SweepCircleAgainst3D(_denseCompoundContext, _denseEnd);
        return _denseCompoundContext.QueryMixed.LastQueryCandidateCount;
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_FalsePositiveCompoundTargets()
    {
        return SweepCircleAgainst3D(_falsePositiveCompoundContext, _denseEnd);
    }

    [Benchmark]
    public int SweepCircleAgainst3DAll_FalsePositiveCompoundTargets_CandidateCount()
    {
        _ = SweepCircleAgainst3D(_falsePositiveCompoundContext, _denseEnd);
        return _falsePositiveCompoundContext.QueryMixed.LastQueryCandidateCount;
    }

    private static SolidBody CreateSphere3D(GravitasWorldContext context, Vector3d position)
    {
        return CreateStatic3D(context, new LSSphereCollider(), position);
    }

    private static SolidBody CreateStatic3D(
        GravitasWorldContext context,
        LSCollider collider,
        Vector3d position,
        FixedQuaternion? rotation = null)
    {
        var agent = new BenchmarkMatterAgent(context, position);
        FixedQuaternion startRotation = rotation ?? FixedQuaternion.Identity;
        agent.Transform.LocalRotation = startRotation;
        var body = new SolidBody(agent, collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(position, startRotation, BodyMotionType.Static);
        return body;
    }

    private int SweepCircleAgainstDensePrimitive3D(GravitasWorldContext context)
    {
        return SweepCircleAgainst3D(context, _denseEnd);
    }

    private int SweepCircleAgainst3D(GravitasWorldContext context, Vector2d end)
    {
        return context.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            end,
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            _hits);
    }

    private static Fixed64 DenseZForIndex(int index)
    {
        int lane = index % 4;
        return lane switch
        {
            0 => Fixed64.Zero,
            1 => Fixed64.FromFraction(1, 4),
            2 => -Fixed64.FromFraction(1, 4),
            _ => Fixed64.FromFraction(3, 4)
        };
    }

    private static LSConeCollider CreateCone(Fixed64 radius, Fixed64 height)
    {
        return new LSConeCollider
        {
            Radius = radius,
            Size = new Vector3d(radius * (Fixed64)2, height, radius * (Fixed64)2)
        };
    }

    private static LSMeshCollider CreateVerticalQuadMesh()
    {
        Fixed64 minY = -Fixed64.One;
        Fixed64 maxY = Fixed64.One;
        Fixed64 zMin = -Fixed64.Half;
        Fixed64 zMax = Fixed64.Half;
        return new LSMeshCollider(
            new[]
            {
                new Vector3d(Fixed64.Zero, minY, zMin),
                new Vector3d(Fixed64.Zero, maxY, zMin),
                new Vector3d(Fixed64.Zero, minY, zMax),
                new Vector3d(Fixed64.Zero, maxY, zMax)
            },
            new[] { 0, 1, 2, 2, 1, 3 },
            MeshColliderMode.Convex,
            MeshInertiaPolicy.SurfaceApproximation);
    }

    private static LSMeshCollider CreateSlabClippedProxyOnlyTriangleMesh()
    {
        GetSlabClippedProxyOnlyTriangle(out Vector3d[] vertices, out int[] triangles);
        return new LSMeshCollider(vertices, triangles, MeshColliderMode.Convex, MeshInertiaPolicy.SurfaceApproximation);
    }

    private static LSCompoundCollider CreateCompoundTarget()
    {
        return new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero),
            CompoundColliderPart.Cuboid(
                Vector3d.One,
                new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero)));
    }

    private static LSCompoundCollider CreateProxyOnlyCompoundTarget()
    {
        GetSlabClippedProxyOnlyTriangle(out Vector3d[] vertices, out int[] triangles);
        return new LSCompoundCollider(
            CompoundColliderPart.ConvexMesh(
                vertices,
                triangles,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.One,
                MeshInertiaPolicy.SurfaceApproximation));
    }

    private static void GetSlabClippedProxyOnlyTriangle(out Vector3d[] vertices, out int[] triangles)
    {
        vertices = new[]
        {
            new Vector3d(Fixed64.Zero, -Fixed64.One, Fixed64.FromFraction(49, 100)),
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.FromFraction(71, 100)),
            new Vector3d(Fixed64.Zero, -Fixed64.One, Fixed64.One)
        };
        triangles = new[] { 0, 1, 2 };
    }
}
