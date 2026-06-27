using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Queries;
using Gravitas.Support;
using SwiftCollections;
using System;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class QueryBatchBenchmarks
{
    private const int RequestCount = 16;

    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);

    private GravitasWorldContext _context3D;
    private GravitasWorldContext _context2D;
    private GravitasWorldContext _mixed3DSourceContext;
    private GravitasWorldContext _mixed2DSourceContext;
    private PhysicsRaycast3DRequest[] _ray3DRequests;
    private PhysicsSweepSphere3DRequest[] _sweepSphere3DRequests;
    private PhysicsRaycast2DRequest[] _ray2DRequests;
    private PhysicsOverlapCircle2DRequest[] _overlapCircle2DRequests;
    private PhysicsOverlapAabb2DRequest[] _overlapAabb2DRequests;
    private PhysicsOverlapPolygon2DRequest[] _overlapPolygon2DRequests;
    private Vector2d[] _polygon2DVertices;
    private PhysicsSweepCircle2DRequest[] _sweepCircle2DRequests;
    private PhysicsSweepSphereAgainst2DRequest[] _mixedSphereRequests;
    private PhysicsSweepCircleAgainst3DRequest[] _mixedCircleRequests;
    private Physics3DHit[] _closest3D;
    private Physics2DHit[] _closest2D;
    private PhysicsMixedHit[] _closestMixed;
    private PhysicsQueryHitRange[] _ranges;
    private SwiftList<Physics3DHit> _hits3D;
    private SwiftList<Physics2DHit> _hits2D;
    private SwiftList<PhysicsMixedHit> _hitsMixed;

    [Params(64)]
    public int ColliderCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        int extent = BenchmarkPhysicsScene.GridExtentForLine(ColliderCount);
        _context3D = BenchmarkPhysicsScene.CreateContext(extent, clearAllPools: true);
        BenchmarkPhysicsScene.CreateDynamicSphereLine(_context3D, ColliderCount);

        _context2D = ContinuousCollisionBenchmarkSupport.CreateContext2D(extent, 8);
        _mixed3DSourceContext = ContinuousCollisionBenchmarkSupport.CreateMixedContext(extent, 8);
        _mixed2DSourceContext = ContinuousCollisionBenchmarkSupport.CreateMixedContext(extent, 8);

        for (int i = 0; i < ColliderCount; i++)
        {
            Vector2d position2D = new((Fixed64)(i * 2), Fixed64.Zero);
            Vector3d position3D = new((Fixed64)(i * 2), Fixed64.Zero, Fixed64.Zero);
            ContinuousCollisionBenchmarkSupport.CreateStaticCircle2D(_context2D, position2D);
            ContinuousCollisionBenchmarkSupport.CreateStaticCircle2D(_mixed3DSourceContext, position2D);
            ContinuousCollisionBenchmarkSupport.CreateStaticSphere3D(_mixed2DSourceContext, position3D);
        }

        _ray3DRequests = new PhysicsRaycast3DRequest[RequestCount];
        _sweepSphere3DRequests = new PhysicsSweepSphere3DRequest[RequestCount];
        _ray2DRequests = new PhysicsRaycast2DRequest[RequestCount];
        _overlapCircle2DRequests = new PhysicsOverlapCircle2DRequest[RequestCount];
        _overlapAabb2DRequests = new PhysicsOverlapAabb2DRequest[RequestCount];
        _overlapPolygon2DRequests = new PhysicsOverlapPolygon2DRequest[RequestCount];
        _polygon2DVertices = new Vector2d[RequestCount * 4];
        _sweepCircle2DRequests = new PhysicsSweepCircle2DRequest[RequestCount];
        _mixedSphereRequests = new PhysicsSweepSphereAgainst2DRequest[RequestCount];
        _mixedCircleRequests = new PhysicsSweepCircleAgainst3DRequest[RequestCount];
        _closest3D = new Physics3DHit[RequestCount];
        _closest2D = new Physics2DHit[RequestCount];
        _closestMixed = new PhysicsMixedHit[RequestCount];
        _ranges = new PhysicsQueryHitRange[RequestCount];
        _hits3D = new SwiftList<Physics3DHit>(ColliderCount * RequestCount);
        _hits2D = new SwiftList<Physics2DHit>(ColliderCount * RequestCount);
        _hitsMixed = new SwiftList<PhysicsMixedHit>(ColliderCount * RequestCount);

        for (int i = 0; i < RequestCount; i++)
        {
            Fixed64 yOrZ = Fixed64.FromFraction(i % 4, 4);
            Vector3d start3D = new((Fixed64)(-2), yOrZ, Fixed64.Zero);
            Vector3d end3D = new((Fixed64)(ColliderCount * 2), yOrZ, Fixed64.Zero);
            Vector2d start2D = new((Fixed64)(-2), yOrZ);
            Vector2d end2D = new((Fixed64)(ColliderCount * 2), yOrZ);
            Vector2d areaCenter = new((Fixed64)(i * 2), Fixed64.Zero);

            _ray3DRequests[i] = new PhysicsRaycast3DRequest(start3D, end3D, IncludeLayerZero);
            _sweepSphere3DRequests[i] = new PhysicsSweepSphere3DRequest(start3D, end3D, Fixed64.Half, IncludeLayerZero);
            _ray2DRequests[i] = new PhysicsRaycast2DRequest(start2D, end2D, IncludeLayerZero);
            _overlapCircle2DRequests[i] = new PhysicsOverlapCircle2DRequest(areaCenter, (Fixed64)4, IncludeLayerZero);
            _overlapAabb2DRequests[i] = new PhysicsOverlapAabb2DRequest(areaCenter, new Vector2d((Fixed64)8, (Fixed64)2), IncludeLayerZero);
            int vertexStart = i * 4;
            _overlapPolygon2DRequests[i] = new PhysicsOverlapPolygon2DRequest(vertexStart, 4, IncludeLayerZero);
            WriteBoxVertices(areaCenter, new Vector2d((Fixed64)4, Fixed64.One), _polygon2DVertices, vertexStart);
            _sweepCircle2DRequests[i] = new PhysicsSweepCircle2DRequest(start2D, end2D, Fixed64.Half, IncludeLayerZero);
            _mixedSphereRequests[i] = new PhysicsSweepSphereAgainst2DRequest(
                new Vector3d(start2D.X, Fixed64.One, start2D.Y),
                new Vector3d(end2D.X, Fixed64.One, end2D.Y),
                Fixed64.Half,
                IncludeLayerZero);
            _mixedCircleRequests[i] = new PhysicsSweepCircleAgainst3DRequest(
                start2D,
                end2D,
                Fixed64.Half,
                Fixed64.Zero,
                Fixed64.Half,
                IncludeLayerZero);
        }
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _context3D?.Dispose();
        _context2D?.Dispose();
        _mixed3DSourceContext?.Dispose();
        _mixed2DSourceContext?.Dispose();
    }

    [Benchmark]
    public int QueryBatch3DRaycast_Individual()
    {
        int total = 0;
        for (int i = 0; i < _ray3DRequests.Length; i++)
        {
            PhysicsRaycast3DRequest request = _ray3DRequests[i];
            total += _context3D.Query3D.RaycastAll(request.Start, request.End, request.LayerMask, _hits3D);
        }

        return total;
    }

    [Benchmark]
    public int QueryBatch3DRaycast_Batch() =>
        _context3D.Query3D.RaycastAllBatch(_ray3DRequests, _hits3D, _ranges);

    [Benchmark]
    public int QueryBatch3DSweepSphere_Individual()
    {
        int total = 0;
        for (int i = 0; i < _sweepSphere3DRequests.Length; i++)
        {
            PhysicsSweepSphere3DRequest request = _sweepSphere3DRequests[i];
            total += _context3D.Query3D.SweepSphereAll(
                request.Start,
                request.End,
                request.Radius,
                request.LayerMask,
                _hits3D,
                request.ExcludedCollider);
        }

        return total;
    }

    [Benchmark]
    public int QueryBatch3DSweepSphere_Batch() =>
        _context3D.Query3D.SweepSphereAllBatch(_sweepSphere3DRequests, _hits3D, _ranges);

    [Benchmark]
    public int QueryBatch2DRaycast_Individual()
    {
        int total = 0;
        for (int i = 0; i < _ray2DRequests.Length; i++)
        {
            PhysicsRaycast2DRequest request = _ray2DRequests[i];
            total += _context2D.Query2D.RaycastAll(request.Start, request.End, request.LayerMask, _hits2D);
        }

        return total;
    }

    [Benchmark]
    public int QueryBatch2DRaycast_Batch() =>
        _context2D.Query2D.RaycastAllBatch(_ray2DRequests, _hits2D, _ranges);

    [Benchmark]
    public int QueryBatch2DArea_Individual()
    {
        int total = 0;
        for (int i = 0; i < RequestCount; i++)
        {
            PhysicsOverlapCircle2DRequest circle = _overlapCircle2DRequests[i];
            PhysicsOverlapAabb2DRequest aabb = _overlapAabb2DRequests[i];
            PhysicsOverlapPolygon2DRequest polygon = _overlapPolygon2DRequests[i];
            total += _context2D.Query2D.OverlapCircleAll(circle.Center, circle.Radius, circle.LayerMask, _hits2D);
            total += _context2D.Query2D.OverlapAabbAll(aabb.Center, aabb.Size, aabb.LayerMask, _hits2D);
            total += _context2D.Query2D.OverlapPolygonAll(
                _polygon2DVertices.AsSpan(polygon.VertexStart, polygon.VertexCount),
                polygon.LayerMask,
                _hits2D);
        }

        return total;
    }

    [Benchmark]
    public int QueryBatch2DArea_Batch()
    {
        return _context2D.Query2D.OverlapCircleAllBatch(_overlapCircle2DRequests, _hits2D, _ranges)
            + _context2D.Query2D.OverlapAabbAllBatch(_overlapAabb2DRequests, _hits2D, _ranges)
            + _context2D.Query2D.OverlapPolygonAllBatch(_overlapPolygon2DRequests, _polygon2DVertices, _hits2D, _ranges);
    }

    [Benchmark]
    public int QueryBatch2DSweepCircle_Individual()
    {
        int total = 0;
        for (int i = 0; i < _sweepCircle2DRequests.Length; i++)
        {
            PhysicsSweepCircle2DRequest request = _sweepCircle2DRequests[i];
            total += _context2D.Query2D.SweepCircleAll(
                request.Start,
                request.End,
                request.Radius,
                request.LayerMask,
                _hits2D,
                request.ExcludedCollider,
                request.IncludeTriggers);
        }

        return total;
    }

    [Benchmark]
    public int QueryBatch2DSweepCircle_Batch() =>
        _context2D.Query2D.SweepCircleAllBatch(_sweepCircle2DRequests, _hits2D, _ranges);

    [Benchmark]
    public int QueryBatchMixedSweeps_Individual()
    {
        int total = 0;
        for (int i = 0; i < RequestCount; i++)
        {
            PhysicsSweepSphereAgainst2DRequest sphere = _mixedSphereRequests[i];
            PhysicsSweepCircleAgainst3DRequest circle = _mixedCircleRequests[i];
            total += _mixed3DSourceContext.QueryMixed.SweepSphereAgainst2DAll(
                sphere.Start,
                sphere.End,
                sphere.Radius,
                sphere.LayerMask,
                _hitsMixed,
                sphere.ExcludedCollider,
                sphere.IncludeTriggers);
            total += _mixed2DSourceContext.QueryMixed.SweepCircleAgainst3DAll(
                circle.Start,
                circle.End,
                circle.Radius,
                circle.SlabCenterY,
                circle.HalfThickness,
                circle.LayerMask,
                _hitsMixed,
                circle.ExcludedCollider,
                circle.IncludeTriggers);
        }

        return total;
    }

    [Benchmark]
    public int QueryBatchMixedSweeps_Batch()
    {
        return _mixed3DSourceContext.QueryMixed.SweepSphereAgainst2DAllBatch(_mixedSphereRequests, _hitsMixed, _ranges)
            + _mixed2DSourceContext.QueryMixed.SweepCircleAgainst3DAllBatch(_mixedCircleRequests, _hitsMixed, _ranges);
    }

    private static void WriteBoxVertices(Vector2d center, Vector2d halfExtents, Vector2d[] vertices, int start)
    {
        vertices[start] = center - halfExtents;
        vertices[start + 1] = new Vector2d(center.X + halfExtents.X, center.Y - halfExtents.Y);
        vertices[start + 2] = center + halfExtents;
        vertices[start + 3] = new Vector2d(center.X - halfExtents.X, center.Y + halfExtents.Y);
    }
}
