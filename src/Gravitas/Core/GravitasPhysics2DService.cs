using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// Context-owned pure 2D body, broad-phase, narrow-phase, response, and query service.
/// </summary>
public sealed class GravitasPhysics2DService
{
    private readonly GravitasWorldContext _context;
    private readonly SwiftBucket<StiffBody2D> _dynamicBodies = new();
    private readonly SwiftList<LSCollider2D> _colliders = new();
    private readonly SwiftList<LSCollider2D> _sortedColliders = new();
    private readonly SwiftDictionary<ulong, CollisionPair2D> _pairs = new();
    private readonly SwiftList<ulong> _pairsToRemove = new();
    private readonly SwiftStack<CollisionPair2D> _cachedPairs = new();
    private int _nextColliderId = 1;

    public GravitasPhysics2DService(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
    }

    public int BodyCount { get; private set; }

    public int ColliderCount => _colliders.Count;

    public bool SimulatePhysics { get; set; } = true;

    internal void AssimilateBody(StiffBody2D body, bool isDynamic)
    {
        SwiftThrowHelper.ThrowIfNull(body, nameof(body));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(body.Context, _context),
            nameof(body),
            "2D body must belong to this physics service context.");

        if (isDynamic)
        {
            body.DynamicId = _dynamicBodies.Add(body);
            BodyCount++;
        }

        AssimilateCollider(body.Collider);
    }

    internal void AssimilateCollider(LSCollider2D collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(collider.Context, _context),
            nameof(collider),
            "2D collider must belong to this physics service context.");

        collider.SetPhysicsState(_nextColliderId++, _colliders.Count);
        _colliders.Add(collider);
    }

    internal void DessimilateBody(StiffBody2D body)
    {
        SwiftThrowHelper.ThrowIfNull(body, nameof(body));
        if (body.DynamicId >= 0 && _dynamicBodies.TryRemoveAt(body.DynamicId) && BodyCount > 0)
            BodyCount--;

        LSCollider2D collider = body.Collider;
        DessimilateCollider(collider);
    }

    internal void DessimilateCollider(LSCollider2D collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        RemovePairsForCollider(collider);
        RemoveCollider(collider);
        collider.ClearPhysicsState();
    }

    public void Simulate()
    {
        if (!SimulatePhysics)
            return;

        PrepareBroadPhase();
        int frame = _context.FrameCount;

        for (int i = 0; i < _sortedColliders.Count; i++)
        {
            LSCollider2D first = _sortedColliders[i];
            if (!first.IsActive)
                continue;

            for (int j = i + 1; j < _sortedColliders.Count; j++)
            {
                LSCollider2D second = _sortedColliders[j];
                if (second.MinX > first.MaxX)
                    break;
                if (!second.IsActive || !BoundsOverlapY(first, second))
                    continue;

                ProcessCandidate(first, second, frame);
            }
        }

        CleanupUntouchedPairs(frame);
    }

    public void LateSimulate()
    {
        if (!SimulatePhysics)
            return;

        foreach (StiffBody2D body in _dynamicBodies)
            body.LateSimulate();
    }

    public void Reset()
    {
        _dynamicBodies.Clear();
        _colliders.FastClear();
        _sortedColliders.FastClear();
        _pairs.Clear();
        _pairsToRemove.FastClear();
        _cachedPairs.Clear();
        _nextColliderId = 1;
        BodyCount = 0;
    }

    /// <summary>
    /// Writes all active pure 2D colliders overlapping the query circle into <paramref name="results"/>.
    /// </summary>
    /// <returns>The number of hits written to <paramref name="results"/>.</returns>
    public int OverlapCircleAll(Vector2d center, Fixed64 radius, SwiftList<Physics2DHit> results)
    {
        return OverlapCircleAll(center, radius, PhysicsLayerMask.All, results);
    }

    /// <summary>
    /// Writes all active pure 2D colliders on included layers that overlap the query circle.
    /// </summary>
    /// <returns>The number of hits written to <paramref name="results"/>.</returns>
    public int OverlapCircleAll(
        Vector2d center,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        SwiftList<Physics2DHit> results)
    {
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));
        SwiftThrowHelper.ThrowIfArgument(radius < Fixed64.Zero, nameof(radius), "2D query radius cannot be negative.");

        results.FastClear();
        Fixed64 minX = center.x - radius;
        Fixed64 maxX = center.x + radius;
        Fixed64 minY = center.y - radius;
        Fixed64 maxY = center.y + radius;

        PrepareBroadPhase();
        for (int i = 0; i < _sortedColliders.Count; i++)
        {
            LSCollider2D collider = _sortedColliders[i];
            if (collider.MinX > maxX)
                break;
            if (!collider.IsActive
                || !layerMask.Includes(collider.Layer)
                || collider.MaxX < minX
                || collider.MinY > maxY
                || collider.MaxY < minY)
                continue;

            if (CollisionDetection2D.TryOverlapCircle(center, radius, collider, out Physics2DHit hit))
                results.Add(hit);
        }

        Physics2DHitSorter.SortByDistance(results);
        return results.Count;
    }

    private void PrepareBroadPhase()
    {
        _sortedColliders.FastClear();
        for (int i = 0; i < _colliders.Count; i++)
        {
            LSCollider2D collider = _colliders[i];
            if (!collider.IsActive)
                continue;

            collider.Rebuild();
            _sortedColliders.Add(collider);
        }

        SortCollidersByMinX(_sortedColliders);
    }

    private void ProcessCandidate(LSCollider2D first, LSCollider2D second, int frame)
    {
        if (ReferenceEquals(first.AgentOrNull, second.AgentOrNull))
            return;

        if (IsLayerCollisionDisabled(first.Layer, second.Layer))
            return;

        ulong key = CreatePairKey(first.Id, second.Id);
        if (!CollisionDetection2D.TryCollide(first, second, out Contact2D contact))
            return;

        bool hasPair = _pairs.TryGetValue(key, out CollisionPair2D pair);
        if (!HasAwakeMovableParticipant(first, second) && !first.IsTrigger && !second.IsTrigger)
        {
            if (hasPair)
                pair!.MarkResting(frame);

            return;
        }

        if (!hasPair)
        {
            pair = CreatePair(first, second);
            _pairs.Add(key, pair);
        }

        pair!.MarkColliding(frame, contact);
    }

    private void CleanupUntouchedPairs(int frame)
    {
        _pairsToRemove.FastClear();
        foreach (var pairEntry in _pairs)
        {
            CollisionPair2D pair = pairEntry.Value;
            if (pair.LastFrame == frame)
                continue;

            pair.MarkSeparated();
            _pairsToRemove.Add(pairEntry.Key);
        }

        for (int i = 0; i < _pairsToRemove.Count; i++)
        {
            ulong key = _pairsToRemove[i];
            if (_pairs.TryGetValue(key, out CollisionPair2D pair))
                RecyclePair(pair);

            _pairs.Remove(key);
        }
    }

    private CollisionPair2D CreatePair(LSCollider2D first, LSCollider2D second)
    {
        if (_context.Settings.PoolingEnabled && _cachedPairs.Count > 0)
        {
            CollisionPair2D pair = _cachedPairs.Pop();
            pair.Initialize(first, second);
            return pair;
        }

        return new CollisionPair2D(first, second);
    }

    private void RecyclePair(CollisionPair2D pair)
    {
        if (_context.Settings.PoolingEnabled)
            _cachedPairs.Push(pair);
    }

    private void RemovePairsForCollider(LSCollider2D collider)
    {
        _pairsToRemove.FastClear();
        foreach (var pairEntry in _pairs)
        {
            CollisionPair2D pair = pairEntry.Value;
            if (!ReferenceEquals(pair.ColliderA, collider) && !ReferenceEquals(pair.ColliderB, collider))
                continue;

            pair.MarkSeparated();
            _pairsToRemove.Add(pairEntry.Key);
        }

        for (int i = 0; i < _pairsToRemove.Count; i++)
        {
            ulong key = _pairsToRemove[i];
            if (_pairs.TryGetValue(key, out CollisionPair2D pair))
                RecyclePair(pair);

            _pairs.Remove(key);
        }
    }

    private void RemoveCollider(LSCollider2D collider)
    {
        int index = collider.ServiceIndex;
        if (index < 0 || index >= _colliders.Count || !ReferenceEquals(_colliders[index], collider))
            return;

        int lastIndex = _colliders.Count - 1;
        if (index != lastIndex)
        {
            LSCollider2D moved = _colliders[lastIndex];
            _colliders[index] = moved;
            moved.SetServiceIndex(index);
        }

        _colliders.RemoveAt(lastIndex);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HasAwakeMovableParticipant(LSCollider2D first, LSCollider2D second) =>
        IsAwakeMovable(first.Body) || IsAwakeMovable(second.Body);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsAwakeMovable(StiffBody2D? body) =>
        body != null && body.CanMove && !body.IsSleeping;

    private bool IsLayerCollisionDisabled(PhysicsLayer layer1, PhysicsLayer layer2)
    {
        bool[,] matrix = _context.Settings.CollisionMatrix;
        int layerIndex1 = layer1.Index;
        int layerIndex2 = layer2.Index;
        if (layerIndex1 >= matrix.GetLength(0) || layerIndex2 >= matrix.GetLength(1))
            return false;

        return !matrix[layerIndex1, layerIndex2];
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool BoundsOverlapY(LSCollider2D first, LSCollider2D second) =>
        first.MinY <= second.MaxY && first.MaxY >= second.MinY;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ulong CreatePairKey(int firstId, int secondId)
    {
        if (firstId > secondId)
            (firstId, secondId) = (secondId, firstId);

        return ((ulong)(uint)firstId << 32) | (uint)secondId;
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
}
