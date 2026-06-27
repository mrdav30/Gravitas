using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Queries;
using GridForge.Configuration;
using SwiftCollections;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class Physics2DBenchmarks
{
    private GravitasWorldContext _integrationContext;
    private GravitasWorldContext _collisionContext;
    private GravitasWorldContext _queryContext;
    private GravitasWorldContext _detectionContext;
    private GravitasWorldContext _responseContext;
    private GravitasWorldContext _pairCleanupContext;
    private SwiftList<SolidBody2D> _integrationBodies;
    private SwiftList<SolidBody2D> _collisionBodies;
    private SwiftList<SolidBody2D> _pairCleanupOwners;
    private SwiftList<LSCollider2D> _sweepCollisionColliders;
    private SwiftList<LSCollider2D> _sweepQueryColliders;
    private SwiftList<LSCollider2D> _sweepSortedColliders;
    private SwiftList<Physics2DHit> _queryHits;
    private ContactManifold2D _sweepCollisionManifold;
    private PreparedPair2D[] _shapePairs;
    private CollisionPair2D[] _angularResponsePairs;
    private SolidBody2D[] _angularResponseBodies;
    private Vector2d[] _angularResponseVelocities;
    private CollisionWorkItem2D[] _twoContactDetectionItems;
    private ContactManifold2D[] _twoContactDetectionManifolds;
    private CollisionPair2D[] _twoContactResponsePairs;
    private SolidBody2D[] _twoContactResponseBodies;
    private Vector2d[] _twoContactResponseVelocities;
    private CollisionPair2D[] _materialResponsePairs;
    private SolidBody2D[] _materialResponseBodies;
    private Vector2d[] _materialResponseVelocities;

    [Params(64, 1024)]
    public int BodyCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _integrationContext = GravitasWorldContext.CreateOwned();
        _collisionContext = GravitasWorldContext.CreateOwned();
        _queryContext = GravitasWorldContext.CreateOwned();
        _detectionContext = GravitasWorldContext.CreateOwned();
        _responseContext = GravitasWorldContext.CreateOwned();
        _integrationContext.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        Configure2DContext(_collisionContext, BodyCount);
        Configure2DContext(_queryContext, BodyCount);
        Configure2DContext(_detectionContext, BodyCount);
        Configure2DContext(_responseContext, BodyCount);
        _integrationBodies = new SwiftList<SolidBody2D>(BodyCount);
        _collisionBodies = new SwiftList<SolidBody2D>(BodyCount);
        _sweepCollisionColliders = new SwiftList<LSCollider2D>(BodyCount * 2);
        _sweepQueryColliders = new SwiftList<LSCollider2D>(BodyCount);
        _sweepSortedColliders = new SwiftList<LSCollider2D>(BodyCount * 2);
        _queryHits = new SwiftList<Physics2DHit>(BodyCount);
        _sweepCollisionManifold = new ContactManifold2D();
        _shapePairs = new PreparedPair2D[BodyCount];
        _angularResponsePairs = new CollisionPair2D[BodyCount];
        _angularResponseBodies = new SolidBody2D[BodyCount];
        _angularResponseVelocities = new Vector2d[BodyCount];
        _twoContactDetectionItems = new CollisionWorkItem2D[BodyCount];
        _twoContactDetectionManifolds = new ContactManifold2D[BodyCount];
        _twoContactResponsePairs = new CollisionPair2D[BodyCount];
        _twoContactResponseBodies = new SolidBody2D[BodyCount];
        _twoContactResponseVelocities = new Vector2d[BodyCount];
        _materialResponsePairs = new CollisionPair2D[BodyCount];
        _materialResponseBodies = new SolidBody2D[BodyCount];
        _materialResponseVelocities = new Vector2d[BodyCount];

        for (int i = 0; i < BodyCount; i++)
        {
            Vector2d position = PositionForIndex(i, spacing: (Fixed64)3);
            SolidBody2D body = CreateBody(_integrationContext, new LSCircleCollider2D(Fixed64.Half), position, immovable: false);
            SolidBody2D queryBody = CreateBody(_queryContext, CreateShape(i), position, immovable: true);
            _integrationBodies.Add(body);
            _sweepQueryColliders.Add(queryBody.Collider);
            _shapePairs[i] = CreatePreparedPair(i);

            _angularResponsePairs[i] = CreateAngularResponsePair(
                i,
                out SolidBody2D angularBody,
                out Vector2d angularVelocity);
            _angularResponseBodies[i] = angularBody;
            _angularResponseVelocities[i] = angularVelocity;
            _twoContactDetectionItems[i] = CreateTwoContactDetectionItem(i);
            _twoContactDetectionManifolds[i] = new ContactManifold2D();
            _twoContactResponsePairs[i] = CreateTwoContactResponsePair(
                i,
                out SolidBody2D responseBody,
                out Vector2d responseVelocity);
            _twoContactResponseBodies[i] = responseBody;
            _twoContactResponseVelocities[i] = responseVelocity;
            _materialResponsePairs[i] = CreateMaterialTwoContactResponsePair(
                i,
                out SolidBody2D materialBody,
                out Vector2d materialVelocity);
            _materialResponseBodies[i] = materialBody;
            _materialResponseVelocities[i] = materialVelocity;
        }

        for (int i = 0; i < BodyCount; i++)
        {
            Vector2d position = PositionForIndex(i, spacing: (Fixed64)2);
            SolidBody2D dynamicBody = CreateBody(_collisionContext, new LSCircleCollider2D(Fixed64.Half), position, immovable: false);
            SolidBody2D staticBody = CreateBody(_collisionContext, new LSCircleCollider2D(Fixed64.Half), position + new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: true);
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
        _responseContext.Dispose();
        _pairCleanupContext?.Dispose();

        _integrationContext = null;
        _collisionContext = null;
        _queryContext = null;
        _detectionContext = null;
        _responseContext = null;
        _pairCleanupContext = null;
        _integrationBodies = null;
        _collisionBodies = null;
        _pairCleanupOwners = null;
        _sweepCollisionColliders = null;
        _sweepQueryColliders = null;
        _sweepSortedColliders = null;
        _queryHits = null;
        _sweepCollisionManifold = null;
        _shapePairs = null;
        _angularResponsePairs = null;
        _angularResponseBodies = null;
        _angularResponseVelocities = null;
        _twoContactDetectionItems = null;
        _twoContactDetectionManifolds = null;
        _twoContactResponsePairs = null;
        _twoContactResponseBodies = null;
        _twoContactResponseVelocities = null;
        _materialResponsePairs = null;
        _materialResponseBodies = null;
        _materialResponseVelocities = null;
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

                CollisionType2D collisionType = ColliderSettings2D.GetCollisionType(first.Shape, second.Shape);
                if (CollisionDetection2D.TryCollide(
                    new CollisionWorkItem2D(first, second, collisionType),
                    _sweepCollisionManifold,
                    collisionCount))
                {
                    collisionCount++;
                }
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
    public long ResolveAngularContactPairs()
    {
        long checksum = 0;
        for (int i = 0; i < _angularResponsePairs.Length; i++)
        {
            SolidBody2D body = _angularResponseBodies[i];
            body.ApplyCollisionAngularVelocityDelta(-body.AngularVelocity);
            body.ApplyCollisionLinearVelocityDelta(_angularResponseVelocities[i] - body.LinearVelocity);

            CollisionResponse2D.Resolve(_angularResponsePairs[i]);
            checksum += body.AngularVelocity.m_rawValue;
        }

        return checksum;
    }

    [Benchmark]
    public long DetectConvexConvexTwoContactManifolds()
    {
        long checksum = 0;
        for (int i = 0; i < _twoContactDetectionItems.Length; i++)
        {
            ContactManifold2D manifold = _twoContactDetectionManifolds[i];
            if (CollisionDetection2D.TryCollide(_twoContactDetectionItems[i], manifold, i))
                checksum += manifold.Count;
        }

        return checksum;
    }

    [Benchmark]
    public long ResolveTwoContactManifoldPairs()
    {
        long checksum = 0;
        for (int i = 0; i < _twoContactResponsePairs.Length; i++)
        {
            SolidBody2D body = _twoContactResponseBodies[i];
            body.ApplyCollisionAngularVelocityDelta(-body.AngularVelocity);
            body.ApplyCollisionLinearVelocityDelta(_twoContactResponseVelocities[i] - body.LinearVelocity);

            CollisionResponse2D.Resolve(_twoContactResponsePairs[i]);
            checksum += body.LinearVelocity.X.m_rawValue;
            checksum += body.AngularVelocity.m_rawValue;
        }

        return checksum;
    }

    [Benchmark]
    public long ResolveTwoContactMaterialManifoldPairs()
    {
        long checksum = 0;
        for (int i = 0; i < _materialResponsePairs.Length; i++)
        {
            SolidBody2D body = _materialResponseBodies[i];
            body.ApplyCollisionAngularVelocityDelta(-body.AngularVelocity);
            body.ApplyCollisionLinearVelocityDelta(_materialResponseVelocities[i] - body.LinearVelocity);

            CollisionResponse2D.Resolve(_materialResponsePairs[i]);
            checksum += body.LinearVelocity.X.m_rawValue;
            checksum += body.AngularVelocity.m_rawValue;
        }

        return checksum;
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
        _pairCleanupOwners = new SwiftList<SolidBody2D>(BodyCount);
        for (int i = 0; i < BodyCount; i++)
        {
            Vector2d position = PositionForIndex(i, spacing: (Fixed64)2);
            SolidBody2D owner = CreateBody(_pairCleanupContext, new LSCircleCollider2D(Fixed64.Half), position, immovable: false);
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
            if (CollisionDetection2D.TryCollide(pair.WorkItem, pair.Manifold, i))
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
    public int OverlapAabbAll()
    {
        return _queryContext.Query2D.OverlapAabbAll(
            new Vector2d((Fixed64)12, (Fixed64)12),
            new Vector2d((Fixed64)36, (Fixed64)36),
            _queryHits);
    }

    [Benchmark]
    public int OverlapPolygonAll()
    {
        ReadOnlySpan<Vector2d> vertices = stackalloc Vector2d[]
        {
            new Vector2d((Fixed64)(-6), (Fixed64)(-4)),
            new Vector2d((Fixed64)30, (Fixed64)(-4)),
            new Vector2d((Fixed64)34, (Fixed64)24),
            new Vector2d((Fixed64)(-2), (Fixed64)28)
        };
        return _queryContext.Query2D.OverlapPolygonAll(vertices, _queryHits);
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
        CollisionType2D collisionType = ColliderSettings2D.GetCollisionType(colliderA.Shape, colliderB.Shape);
        return new PreparedPair2D(new CollisionWorkItem2D(colliderA, colliderB, collisionType));
    }

    private CollisionWorkItem2D CreateTwoContactDetectionItem(int index)
    {
        var first = new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2));
        var second = new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2));
        Vector2d position = PositionForIndex(index + BodyCount, spacing: (Fixed64)4);
        _ = CreateBody(_detectionContext, first, position, immovable: true);
        _ = CreateBody(
            _detectionContext,
            second,
            position + new Vector2d(Fixed64.FromFraction(3, 2), Fixed64.Zero),
            immovable: true);
        CollisionType2D collisionType = ColliderSettings2D.GetCollisionType(first.Shape, second.Shape);
        return new CollisionWorkItem2D(first, second, collisionType);
    }

    private CollisionPair2D CreateAngularResponsePair(
        int index,
        out SolidBody2D dynamicBody,
        out Vector2d resetVelocity)
    {
        Vector2d position = PositionForIndex(index, spacing: (Fixed64)4);
        dynamicBody = CreateBody(
            _responseContext,
            new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)),
            position,
            immovable: false);
        SolidBody2D staticBody = CreateBody(
            _responseContext,
            new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)),
            position + new Vector2d(Fixed64.Half, Fixed64.Zero),
            immovable: true);
        var pair = new CollisionPair2D(dynamicBody.Collider, staticBody.Collider);

        Vector2d normal = pair.ColliderB.Center - pair.ColliderA.Center;
        normal = normal.MagnitudeSquared > Fixed64.Epsilon ? normal.Normalized : Vector2d.Right;
        Vector2d tangent = new(-normal.Y, normal.X);
        bool dynamicIsColliderA = ReferenceEquals(pair.ColliderA.Body, dynamicBody);
        Fixed64 contactOffset = Fixed64.Half;
        Vector2d contactPoint = pair.ColliderA.Center + tangent * contactOffset;
        pair.Manifold.SetContact(contactPoint, contactPoint, Fixed64.Zero, normal);
        resetVelocity = dynamicIsColliderA
            ? normal * (Fixed64)8 + tangent * (Fixed64)2
            : -normal * (Fixed64)8 + tangent * (Fixed64)2;
        return pair;
    }

    private CollisionPair2D CreateTwoContactResponsePair(
        int index,
        out SolidBody2D dynamicBody,
        out Vector2d resetVelocity)
    {
        Vector2d position = PositionForIndex(index + BodyCount, spacing: (Fixed64)4);
        dynamicBody = CreateBody(
            _responseContext,
            new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)),
            position,
            immovable: false);
        SolidBody2D staticBody = CreateBody(
            _responseContext,
            new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)),
            position + new Vector2d(Fixed64.FromFraction(3, 2), Fixed64.Zero),
            immovable: true);
        var pair = new CollisionPair2D(dynamicBody.Collider, staticBody.Collider);

        Vector2d normal = pair.ColliderB.Center - pair.ColliderA.Center;
        normal = normal.MagnitudeSquared > Fixed64.Epsilon ? normal.Normalized : Vector2d.Right;
        Vector2d tangent = new(-normal.Y, normal.X);
        Vector2d pointA0 = pair.ColliderA.Center + normal + tangent;
        Vector2d pointA1 = pair.ColliderA.Center + normal - tangent;
        Vector2d pointB0 = pair.ColliderB.Center - normal + tangent;
        Vector2d pointB1 = pair.ColliderB.Center - normal - tangent;
        pair.Manifold.SetContact(pointA0, pointB0, Fixed64.Zero, normal);
        pair.Manifold.AddContact(pointA1, pointB1, Fixed64.Zero, normal);

        bool dynamicIsColliderA = ReferenceEquals(pair.ColliderA.Body, dynamicBody);
        resetVelocity = dynamicIsColliderA
            ? normal * (Fixed64)8 + tangent * (Fixed64)20
            : -normal * (Fixed64)8 + tangent * (Fixed64)20;
        return pair;
    }

    private CollisionPair2D CreateMaterialTwoContactResponsePair(
        int index,
        out SolidBody2D dynamicBody,
        out Vector2d resetVelocity)
    {
        CollisionPair2D pair = CreateTwoContactResponsePair(index + BodyCount, out dynamicBody, out resetVelocity);
        pair.ColliderA.Material = RoughSurface;
        pair.ColliderB.Material = SlickSurface;
        return pair;
    }

    private static SolidBody2D CreateBody(
        GravitasWorldContext context,
        LSCollider2D collider,
        Vector2d position,
        bool immovable)
    {
        var agent = new BenchmarkMatterAgent(context, new Vector3d(position.X, Fixed64.Zero, position.Y));
        var body = new SolidBody2D(agent, collider)
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
        public PreparedPair2D(CollisionWorkItem2D workItem)
        {
            WorkItem = workItem;
            Manifold = new ContactManifold2D();
        }

        public CollisionWorkItem2D WorkItem { get; }

        public ContactManifold2D Manifold { get; }
    }

    private static readonly PhysicsMaterial RoughSurface = new(
        (Fixed64)2,
        Fixed64.One,
        Fixed64.FromFraction(1, 4),
        PhysicsMaterialCombine.Maximum,
        PhysicsMaterialCombine.Minimum);

    private static readonly PhysicsMaterial SlickSurface = new(
        Fixed64.Half,
        Fixed64.FromFraction(1, 4),
        Fixed64.FromFraction(3, 4),
        PhysicsMaterialCombine.Minimum,
        PhysicsMaterialCombine.Maximum);
}
