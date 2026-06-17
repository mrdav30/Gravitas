using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Queries;
using GridForge.Configuration;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class Physics2DBenchmarks
{
    private GravitasWorldContext _integrationContext;
    private GravitasWorldContext _collisionContext;
    private GravitasWorldContext _queryContext;
    private GravitasWorldContext _detectionContext;
    private GravitasWorldContext _pairCleanupContext;
    private SwiftList<StiffBody2D> _integrationBodies;
    private SwiftList<StiffBody2D> _collisionBodies;
    private SwiftList<StiffBody2D> _pairCleanupOwners;
    private SwiftList<LSCollider2D> _sweepCollisionColliders;
    private SwiftList<LSCollider2D> _sweepQueryColliders;
    private SwiftList<LSCollider2D> _sweepSortedColliders;
    private SwiftList<Physics2DHit> _queryHits;
    private PreparedPair2D[] _shapePairs;

    [Params(64, 1024)]
    public int BodyCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _integrationContext = GravitasWorldContext.CreateOwned();
        _collisionContext = GravitasWorldContext.CreateOwned();
        _queryContext = GravitasWorldContext.CreateOwned();
        _detectionContext = GravitasWorldContext.CreateOwned();
        _integrationContext.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        Configure2DContext(_collisionContext, BodyCount);
        Configure2DContext(_queryContext, BodyCount);
        Configure2DContext(_detectionContext, BodyCount);
        _integrationBodies = new SwiftList<StiffBody2D>(BodyCount);
        _collisionBodies = new SwiftList<StiffBody2D>(BodyCount);
        _sweepCollisionColliders = new SwiftList<LSCollider2D>(BodyCount * 2);
        _sweepQueryColliders = new SwiftList<LSCollider2D>(BodyCount);
        _sweepSortedColliders = new SwiftList<LSCollider2D>(BodyCount * 2);
        _queryHits = new SwiftList<Physics2DHit>(BodyCount);
        _shapePairs = new PreparedPair2D[BodyCount];

        for (int i = 0; i < BodyCount; i++)
        {
            Vector2d position = PositionForIndex(i, spacing: (Fixed64)3);
            StiffBody2D body = CreateBody(_integrationContext, new LSCircleCollider2D(Fixed64.Half), position, immovable: false);
            StiffBody2D queryBody = CreateBody(_queryContext, CreateShape(i), position, immovable: true);
            _integrationBodies.Add(body);
            _sweepQueryColliders.Add(queryBody.Collider);
            _shapePairs[i] = CreatePreparedPair(i);
        }

        for (int i = 0; i < BodyCount; i++)
        {
            Vector2d position = PositionForIndex(i, spacing: (Fixed64)2);
            StiffBody2D dynamicBody = CreateBody(_collisionContext, new LSCircleCollider2D(Fixed64.Half), position, immovable: false);
            StiffBody2D staticBody = CreateBody(_collisionContext, new LSCircleCollider2D(Fixed64.Half), position + new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: true);
            _collisionBodies.Add(dynamicBody);
            _sweepCollisionColliders.Add(dynamicBody.Collider);
            _sweepCollisionColliders.Add(staticBody.Collider);
        }
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _integrationContext.Dispose();
        _collisionContext.Dispose();
        _queryContext.Dispose();
        _detectionContext.Dispose();
        _pairCleanupContext?.Dispose();

        _integrationContext = null;
        _collisionContext = null;
        _queryContext = null;
        _detectionContext = null;
        _pairCleanupContext = null;
        _integrationBodies = null;
        _collisionBodies = null;
        _pairCleanupOwners = null;
        _sweepCollisionColliders = null;
        _sweepQueryColliders = null;
        _sweepSortedColliders = null;
        _queryHits = null;
        _shapePairs = null;
    }

    [Benchmark]
    public int IntegrateDynamicBodies()
    {
        for (int i = 0; i < _integrationBodies.Count; i++)
            _integrationBodies[i].AddForce(Vector2d.Right);

        _integrationContext.Physics2D.LateSimulate();
        return _integrationContext.Physics2D.BodyCount;
    }

    [Benchmark]
    public int ResolveOverlappingCirclePairs_SweepBaseline()
    {
        for (int i = 0; i < _collisionBodies.Count; i++)
            _collisionBodies[i].SetPosition(PositionForIndex(i, spacing: (Fixed64)2));

        PrepareSweep(_sweepCollisionColliders, _sweepSortedColliders);
        int collisionCount = 0;
        for (int i = 0; i < _sweepSortedColliders.Count; i++)
        {
            LSCollider2D first = _sweepSortedColliders[i];
            for (int j = i + 1; j < _sweepSortedColliders.Count; j++)
            {
                LSCollider2D second = _sweepSortedColliders[j];
                if (second.MinX > first.MaxX)
                    break;
                if (second.MinY > first.MaxY || second.MaxY < first.MinY)
                    continue;

                if (CollisionDetection2D.TryCollide(first, second, out _))
                    collisionCount++;
            }
        }

        return collisionCount;
    }

    [Benchmark]
    public int ResolveOverlappingCirclePairs()
    {
        for (int i = 0; i < _collisionBodies.Count; i++)
            _collisionBodies[i].SetPosition(PositionForIndex(i, spacing: (Fixed64)2));

        _collisionContext.Physics2D.Simulate();
        return _collisionContext.Physics2D.BodyCount;
    }

    [Benchmark]
    public uint SimulateUnchangedColliders()
    {
        uint versionTotal = 0;
        for (int i = 0; i < _sweepQueryColliders.Count; i++)
        {
            LSCollider2D collider = _sweepQueryColliders[i];
            collider.Simulate();
            versionTotal += collider.RuntimeShapeVersion;
        }

        return versionTotal;
    }

    [IterationSetup(Target = nameof(DeactivateOverlappingPairOwners))]
    public void SetupPairCleanup()
    {
        _pairCleanupContext = GravitasWorldContext.CreateOwned();
        Configure2DContext(_pairCleanupContext, BodyCount);
        _pairCleanupOwners = new SwiftList<StiffBody2D>(BodyCount);
        for (int i = 0; i < BodyCount; i++)
        {
            Vector2d position = PositionForIndex(i, spacing: (Fixed64)2);
            StiffBody2D owner = CreateBody(_pairCleanupContext, new LSCircleCollider2D(Fixed64.Half), position, immovable: false);
            _ = CreateBody(_pairCleanupContext, new LSCircleCollider2D(Fixed64.Half), position + new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: true);
            _pairCleanupOwners.Add(owner);
        }

        _pairCleanupContext.Simulate();
    }

    [IterationCleanup(Target = nameof(DeactivateOverlappingPairOwners))]
    public void CleanupPairCleanup()
    {
        _pairCleanupContext.Dispose();
        _pairCleanupContext = null;
        _pairCleanupOwners = null;
    }

    [Benchmark]
    public int DeactivateOverlappingPairOwners()
    {
        for (int i = 0; i < _pairCleanupOwners.Count; i++)
            _pairCleanupOwners[i].Collider.Deactivate();

        return _pairCleanupContext.Physics2D.ColliderCount;
    }

    [Benchmark]
    public int CheckRequiredShapePairs()
    {
        int collisionCount = 0;
        for (int i = 0; i < _shapePairs.Length; i++)
        {
            PreparedPair2D pair = _shapePairs[i];
            if (CollisionDetection2D.TryCollide(pair.ColliderA, pair.ColliderB, out _))
                collisionCount++;
        }

        return collisionCount;
    }

    [Benchmark]
    public int OverlapCircleAll_SweepBaseline()
    {
        Vector2d center = new((Fixed64)12, (Fixed64)12);
        Fixed64 radius = (Fixed64)18;
        Fixed64 minX = center.X - radius;
        Fixed64 maxX = center.X + radius;
        Fixed64 minY = center.Y - radius;
        Fixed64 maxY = center.Y + radius;
        PrepareSweep(_sweepQueryColliders, _sweepSortedColliders);

        int count = 0;
        for (int i = 0; i < _sweepSortedColliders.Count; i++)
        {
            LSCollider2D collider = _sweepSortedColliders[i];
            if (collider.MinX > maxX)
                break;
            if (collider.MaxX < minX
                || collider.MinY > maxY
                || collider.MaxY < minY)
            {
                continue;
            }

            if (QueryDetection2D.TryOverlapCircle(center, radius, collider, out _))
                count++;
        }

        return count;
    }

    [Benchmark]
    public int OverlapCircleAll()
    {
        return _queryContext.Query2D.OverlapCircleAll(
            new Vector2d((Fixed64)12, (Fixed64)12),
            (Fixed64)18,
            _queryHits);
    }

    [Benchmark]
    public int RaycastAll_SweepBaseline()
    {
        Vector2d start = new((Fixed64)(-8), (Fixed64)12);
        Vector2d end = new((Fixed64)64, (Fixed64)12);
        Fixed64 minX = FixedMath.Min(start.X, end.X);
        Fixed64 maxX = FixedMath.Max(start.X, end.X);
        Fixed64 minY = FixedMath.Min(start.Y, end.Y);
        Fixed64 maxY = FixedMath.Max(start.Y, end.Y);
        PrepareSweep(_sweepQueryColliders, _sweepSortedColliders);

        int count = 0;
        for (int i = 0; i < _sweepSortedColliders.Count; i++)
        {
            LSCollider2D collider = _sweepSortedColliders[i];
            if (collider.MinX > maxX)
                break;
            if (collider.MaxX < minX
                || collider.MinY > maxY
                || collider.MaxY < minY)
            {
                continue;
            }

            if (QueryDetection2D.TryRaycast(start, end, collider, out _))
                count++;
        }

        return count;
    }

    [Benchmark]
    public int RaycastAll()
    {
        return _queryContext.Query2D.RaycastAll(
            new Vector2d((Fixed64)(-8), (Fixed64)12),
            new Vector2d((Fixed64)64, (Fixed64)12),
            _queryHits);
    }

    [Benchmark]
    public int SweepCircleAll_NoHit()
    {
        return _queryContext.Query2D.SweepCircleAll(
            new Vector2d((Fixed64)(-8), (Fixed64)(-8)),
            new Vector2d((Fixed64)(-4), (Fixed64)(-8)),
            Fixed64.Half,
            _queryHits);
    }

    [Benchmark]
    public int SweepCircleAll_SparseHit()
    {
        return _queryContext.Query2D.SweepCircleAll(
            new Vector2d((Fixed64)(-8), (Fixed64)12),
            new Vector2d((Fixed64)64, (Fixed64)12),
            Fixed64.Half,
            _queryHits);
    }

    [Benchmark]
    public int SweepCircleAll_DenseHit()
    {
        return _queryContext.Query2D.SweepCircleAll(
            new Vector2d((Fixed64)(-8), (Fixed64)12),
            new Vector2d((Fixed64)64, (Fixed64)12),
            (Fixed64)18,
            _queryHits);
    }

    private static void Configure2DContext(GravitasWorldContext context, int bodyCount)
    {
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        int extent = bodyCount <= 64 ? 64 : 512;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-16), Fixed64.Zero, (Fixed64)(-16)),
                new Vector3d((Fixed64)extent, Fixed64.Zero, (Fixed64)extent)),
            out _);
    }

    private static void PrepareSweep(SwiftList<LSCollider2D> source, SwiftList<LSCollider2D> sorted)
    {
        sorted.FastClear();
        for (int i = 0; i < source.Count; i++)
        {
            LSCollider2D collider = source[i];
            if (!collider.IsActive)
                continue;

            collider.Rebuild();
            sorted.Add(collider);
        }

        SortCollidersByMinX(sorted);
    }

    private PreparedPair2D CreatePreparedPair(int index)
    {
        LSCollider2D colliderA = CreateShape(index);
        LSCollider2D colliderB = CreateShape(index + 1);
        Vector2d position = PositionForIndex(index, spacing: (Fixed64)4);
        _ = CreateBody(_detectionContext, colliderA, position, immovable: true);
        _ = CreateBody(_detectionContext, colliderB, position + new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: true);
        return new PreparedPair2D(colliderA, colliderB);
    }

    private static StiffBody2D CreateBody(
        GravitasWorldContext context,
        LSCollider2D collider,
        Vector2d position,
        bool immovable)
    {
        var agent = new BenchmarkMatterAgent(context, new Vector3d(position.X, Fixed64.Zero, position.Y));
        var body = new StiffBody2D(agent, collider)
        {
            Mass = Fixed64.One,
            Immovable = immovable
        };
        body.Initialize(position);
        return body;
    }

    private static LSCollider2D CreateShape(int index)
    {
        return (index % 3) switch
        {
            0 => new LSCircleCollider2D(Fixed64.One),
            1 => new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)),
            _ => new LSPolygonCollider2D(
                new Vector2d(-Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.One, Fixed64.One),
                new Vector2d(-Fixed64.One, Fixed64.One))
        };
    }

    private static Vector2d PositionForIndex(int index, Fixed64 spacing)
    {
        int width = 8;
        int x = index % width;
        int y = index / width;
        return new Vector2d((Fixed64)x * spacing, (Fixed64)y * spacing);
    }

    private static void SortCollidersByMinX(SwiftList<LSCollider2D> colliders)
    {
        if (colliders.Count < 2)
            return;

        QuickSortColliders(colliders, 0, colliders.Count - 1);
    }

    private static void QuickSortColliders(SwiftList<LSCollider2D> colliders, int left, int right)
    {
        while (left < right)
        {
            if (right - left <= 16)
            {
                InsertionSortColliders(colliders, left, right);
                return;
            }

            int i = left;
            int j = right;
            LSCollider2D pivot = colliders[left + ((right - left) / 2)];
            while (i <= j)
            {
                while (CompareByMinX(colliders[i], pivot) < 0)
                    i++;
                while (CompareByMinX(colliders[j], pivot) > 0)
                    j--;

                if (i > j)
                    continue;

                if (i != j)
                    (colliders[i], colliders[j]) = (colliders[j], colliders[i]);

                i++;
                j--;
            }

            if (j - left < right - i)
            {
                if (left < j)
                    QuickSortColliders(colliders, left, j);

                left = i;
            }
            else
            {
                if (i < right)
                    QuickSortColliders(colliders, i, right);

                right = j;
            }
        }
    }

    private static void InsertionSortColliders(SwiftList<LSCollider2D> colliders, int left, int right)
    {
        for (int i = left + 1; i <= right; i++)
        {
            LSCollider2D value = colliders[i];
            int index = i - 1;
            while (index >= left && CompareByMinX(colliders[index], value) > 0)
            {
                colliders[index + 1] = colliders[index];
                index--;
            }

            colliders[index + 1] = value;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int CompareByMinX(LSCollider2D left, LSCollider2D right)
    {
        int min = left.MinX.CompareTo(right.MinX);
        return min != 0 ? min : left.Id.CompareTo(right.Id);
    }

    private readonly struct PreparedPair2D
    {
        public PreparedPair2D(LSCollider2D colliderA, LSCollider2D colliderB)
        {
            ColliderA = colliderA;
            ColliderB = colliderB;
        }

        public LSCollider2D ColliderA { get; }

        public LSCollider2D ColliderB { get; }
    }
}
