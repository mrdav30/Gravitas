using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;
using GridForge.Spatial;
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
    private readonly SwiftDictionary<int, LSCollider2D> _collidersById = new();
    private readonly SwiftHashSet<ulong> _processedPairKeys = new();
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

    internal int LastBroadPhaseCandidateCount { get; private set; }

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
        _collidersById.Add(collider.Id, collider);
        _context.Collisions2D.PartitionCollider(collider);
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
        _context.Collisions2D.ClearPartitionedCollider(collider, force: true);
        RemoveCollider(collider);
        _collidersById.Remove(collider.Id);
        collider.ClearPhysicsState();
    }

    public void Simulate()
    {
        if (!SimulatePhysics)
            return;

        LastBroadPhaseCandidateCount = 0;
        EnsureFrameCapacity();
        _processedPairKeys.Clear();
        int frame = _context.FrameCount;
        _context.Collisions2D.CheckAndDistributeCollisions();
        CleanupUntouchedPairs(frame);
    }

    public void LateSimulate()
    {
        if (!SimulatePhysics)
            return;

        foreach (StiffBody2D body in _dynamicBodies)
            body.LateSimulate();
    }

    public void Visualize()
    {
        foreach (StiffBody2D body in _dynamicBodies)
            body.OnVisualize();
    }

    public void LateVisualize()
    {
        foreach (StiffBody2D body in _dynamicBodies)
            body.LateVisualize();
    }

    public void Reset()
    {
        _dynamicBodies.Clear();
        _colliders.FastClear();
        _collidersById.Clear();
        _processedPairKeys.Clear();
        _pairs.Clear();
        _pairsToRemove.FastClear();
        _cachedPairs.Clear();
        _nextColliderId = 1;
        BodyCount = 0;
        LastBroadPhaseCandidateCount = 0;
    }

    internal bool TryGetColliderById(int colliderId, out LSCollider2D? collider)
    {
        return _collidersById.TryGetValue(colliderId, out collider);
    }

    internal void ProcessPartitionCandidate(int firstId, int secondId, WorldVoxelIndex partitionIndex)
    {
        if (!_collidersById.TryGetValue(firstId, out LSCollider2D? first)
            || !_collidersById.TryGetValue(secondId, out LSCollider2D? second))
        {
            return;
        }

        if (ReferenceEquals(first!.AgentOrNull, second!.AgentOrNull)
            || IsLayerCollisionDisabled(first.Layer, second.Layer)
            || !CollisionDetection2D.BoundsOverlap(first, second)
            || !IsCanonicalSharedPartition(first, second, partitionIndex))
        {
            return;
        }

        ulong key = CreatePairKey(firstId, secondId);
        if (!_processedPairKeys.Add(key))
            return;

        LastBroadPhaseCandidateCount++;
        ProcessCandidate(first, second, _context.FrameCount);
    }

    private static bool IsCanonicalSharedPartition(LSCollider2D first, LSCollider2D second, WorldVoxelIndex currentIndex)
    {
        SwiftList<WorldVoxelIndex>? firstCoordinates = first.PartitionCoordinates;
        SwiftList<WorldVoxelIndex>? secondCoordinates = second.PartitionCoordinates;
        if (firstCoordinates == null || secondCoordinates == null)
            return true;

        if (!TryGetMinimumVoxelIndexForGrid(firstCoordinates, currentIndex, out VoxelIndex firstMin)
            || !TryGetMinimumVoxelIndexForGrid(secondCoordinates, currentIndex, out VoxelIndex secondMin))
        {
            return true;
        }

        VoxelIndex current = currentIndex.VoxelIndex;
        return current.x == Max(firstMin.x, secondMin.x)
            && current.z == Max(firstMin.z, secondMin.z)
            && current.y == Max(firstMin.y, secondMin.y);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int Max(int first, int second) => first >= second ? first : second;

    private static bool TryGetMinimumVoxelIndexForGrid(
        SwiftList<WorldVoxelIndex> coordinates,
        WorldVoxelIndex gridIdentity,
        out VoxelIndex minimum)
    {
        minimum = default;
        bool found = false;
        for (int i = 0; i < coordinates.Count; i++)
        {
            WorldVoxelIndex candidate = coordinates[i];
            if (candidate.GridIndex != gridIdentity.GridIndex || candidate.GridSpawnToken != gridIdentity.GridSpawnToken)
                continue;

            VoxelIndex voxel = candidate.VoxelIndex;
            if (!found)
            {
                minimum = voxel;
                found = true;
                continue;
            }

            if (voxel.x < minimum.x)
                minimum.x = voxel.x;
            if (voxel.z < minimum.z)
                minimum.z = voxel.z;
            if (voxel.y < minimum.y)
                minimum.y = voxel.y;
        }

        return found;
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

            if (TryKeepRestingPair(pair, frame))
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

    private bool TryKeepRestingPair(CollisionPair2D pair, int frame)
    {
        LSCollider2D first = pair.ColliderA;
        LSCollider2D second = pair.ColliderB;
        if (!first.IsActive
            || !second.IsActive
            || first.IsTrigger
            || second.IsTrigger
            || HasAwakeMovableParticipant(first, second)
            || IsLayerCollisionDisabled(first.Layer, second.Layer)
            || !CollisionDetection2D.TryCollide(first, second, out _))
        {
            return false;
        }

        pair.MarkResting(frame);
        return true;
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

    private void EnsureFrameCapacity()
    {
        int colliderCount = _colliders.Count;
        if (colliderCount <= 0)
            return;

        int expectedPairKeyCapacity = colliderCount * 4;
        _processedPairKeys.EnsureCapacity(expectedPairKeyCapacity);
        _pairsToRemove.EnsureCapacity(colliderCount);
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
    private static ulong CreatePairKey(int firstId, int secondId)
    {
        if (firstId > secondId)
            (firstId, secondId) = (secondId, firstId);

        return ((ulong)(uint)firstId << 32) | (uint)secondId;
    }
}
