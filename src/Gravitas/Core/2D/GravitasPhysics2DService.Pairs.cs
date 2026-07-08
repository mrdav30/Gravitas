//=======================================================================
// GravitasPhysics2DService.Pairs.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using GridForge.Spatial;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas;

public sealed partial class GravitasPhysics2DService
{
    internal void ProcessPartitionCandidate(int firstId, int secondId, WorldVoxelIndex partitionIndex)
    {
        if (!TryGetColliderById(firstId, out LSCollider2D? first)
            || !TryGetColliderById(secondId, out LSCollider2D? second))
        {
            return;
        }

        if (!RequireCollisionPair(first!, second!)
            || !CollisionDetection2D.BoundsOverlap(first!, second!)
            || !IsCanonicalSharedPartition(first!, second!, partitionIndex))
        {
            return;
        }

        ulong key = CreatePairKey(firstId, secondId);
        if (!_processedPairKeys.Add(key))
            return;

        LastBroadPhaseCandidateCount++;
        ProcessCandidate(first!, second!, _context.FrameCount);
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
        ulong key = CreatePairKey(first.Id, second.Id);
        bool hasPair = _pairs.TryGetValue(key, out CollisionPair2D pair);
        bool hasAwakeMovableParticipant = HasAwakeMovableParticipant(first, second);
        bool triggerPair = first.IsTrigger || second.IsTrigger;
        if (!hasPair && !hasAwakeMovableParticipant && !triggerPair)
            return;

        bool createdPair = false;
        if (!hasPair)
        {
            pair = CreatePair(first, second);
            createdPair = true;
        }

        if (!CollisionDetection2D.TryCollide(pair!, pair!.Manifold, frame))
        {
            if (createdPair)
                RecyclePair(pair);
            return;
        }

        if (triggerPair)
        {
            if (createdPair)
                RegisterPair(key, pair);

            pair.MarkColliding(frame);
            return;
        }

        if (!hasAwakeMovableParticipant)
        {
            pair.MarkResting(frame);
            _discreteResponsePairs.Add(pair);
            return;
        }

        if (createdPair)
            RegisterPair(key, pair);

        pair.MarkCollidingDeferred(frame);
        _discreteResponsePairs.Add(pair);
    }

    private void RegisterPair(ulong key, CollisionPair2D pair)
    {
        _pairs.Add(key, pair);
        pair.ColliderA.TryAddCollisionPair(pair.Id2, pair);
        pair.ColliderB.TryAddCollisionPairHolder(pair.Id1);
    }

    private void ExpandDiscreteResponsePairs(int frame)
    {
        if (_discreteResponsePairs.Count == 0)
            return;

        _discreteResponseBodyKeys.Clear();
        _discreteResponseBodyQueue.FastClear();

        for (int i = 0; i < _discreteResponsePairs.Count; i++)
        {
            CollisionPair2D pair = _discreteResponsePairs[i];
            AddDiscreteResponseBody(pair.ColliderA.Body);
            AddDiscreteResponseBody(pair.ColliderB.Body);
        }

        for (int readIndex = 0; readIndex < _discreteResponseBodyQueue.Count; readIndex++)
        {
            int dynamicId = _discreteResponseBodyQueue[readIndex];
            if (!TryGetDynamicBody(dynamicId, out SolidBody2D body))
                continue;

            AddExistingResponsePairs(body.Collider, frame);
        }
    }

    private void AddExistingResponsePairs(LSCollider2D collider, int frame)
    {
        SwiftDictionary<int, CollisionPair2D>? ownedPairs = collider.CollisionPairs;
        if (ownedPairs != null)
        {
            foreach (var pairEntry in ownedPairs)
                TryAddExistingResponsePair(pairEntry.Value, frame);
        }

        SwiftHashSet<int>? pairHolders = collider.CollisionPairHolders;
        if (pairHolders == null)
            return;

        foreach (int holderId in pairHolders)
        {
            if (!TryGetColliderById(holderId, out LSCollider2D? holder)
                || holder!.TryGetCollisionPair(collider.Id, out CollisionPair2D? pair) != true
                || pair == null)
            {
                continue;
            }

            TryAddExistingResponsePair(pair, frame);
        }
    }

    private void TryAddExistingResponsePair(CollisionPair2D pair, int frame)
    {
        ulong key = CreatePairKey(pair.Id1, pair.Id2);
        if (!_processedPairKeys.Add(key))
            return;

        LSCollider2D first = pair.ColliderA;
        LSCollider2D second = pair.ColliderB;
        if (first.IsTrigger
            || second.IsTrigger
            || !RequireCollisionPair(first, second)
            || !CollisionDetection2D.BoundsOverlap(first, second)
            || !CollisionDetection2D.TryCollide(pair, pair.Manifold, frame))
        {
            return;
        }

        if (HasAwakeMovableParticipant(first, second))
            pair.MarkCollidingDeferred(frame);
        else
            pair.MarkResting(frame);

        _discreteResponsePairs.Add(pair);
        AddDiscreteResponseBody(first.Body);
        AddDiscreteResponseBody(second.Body);
    }

    private void AddDiscreteResponseBody(SolidBody2D? body)
    {
        if (!IsMovableIslandBody(body) || !_discreteResponseBodyKeys.Add(body!.DynamicId))
            return;

        _discreteResponseBodyQueue.Add(body.DynamicId);
    }

    internal bool RequireCollisionPair(LSCollider2D first, LSCollider2D second)
    {
        return first.IsActive && second.IsActive
            && first.Shape != ColliderType2D.None && second.Shape != ColliderType2D.None
            && (first.Body != null || second.Body != null)
            && !ReferenceEquals(first.AgentOrNull, second.AgentOrNull)
            && !IsLayerCollisionDisabled(first.Layer, second.Layer)
            && ColliderCollisionFilter.AllowsPhysicalPair(first, second)
            && !_context.Constraints2D.ShouldExcludeLinkedCollision(first, second)
            && !first.IsSibling(second);
    }

    private void CleanupUntouchedPairs(int frame)
    {
        _pairsToRemove.FastClear();
        foreach (var pairEntry in _pairs)
        {
            CollisionPair2D pair = pairEntry.Value;
            if (pair.LastFrame == frame)
                continue;

            if (TryKeepUntouchedPair(pair, frame))
                continue;

            pair.MarkSeparated();
            RemovePairReferences(pair);
            _pairsToRemove.Add(pairEntry.Key);
        }

        for (int i = 0; i < _pairsToRemove.Count; i++)
        {
            ulong key = _pairsToRemove[i];
            RecyclePair(_pairs[key]);
            _pairs.Remove(key);
        }
    }

    private bool TryKeepUntouchedPair(CollisionPair2D pair, int frame)
    {
        LSCollider2D first = pair.ColliderA;
        LSCollider2D second = pair.ColliderB;
        bool triggerPair = first.IsTrigger || second.IsTrigger;
        if ((!triggerPair && HasAwakeMovableParticipant(first, second))
            || !RequireCollisionPair(first, second)
            || !CollisionDetection2D.TryCollide(pair, pair.Manifold, frame))
        {
            return false;
        }

        if (triggerPair)
        {
            pair.MarkColliding(frame);
            return true;
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
        SwiftDictionary<int, CollisionPair2D>? collisionPairs = collider.CollisionPairs;
        if (collisionPairs != null)
        {
            foreach (var pairEntry in collisionPairs)
            {
                CollisionPair2D pair = pairEntry.Value;
                pair.MarkSeparated();
                pair.ColliderB.TryRemoveCollisionPairHolder(pair.Id1);
                _pairs.Remove(CreatePairKey(pair.Id1, pair.Id2));
                RecyclePair(pair);
            }
        }

        SwiftHashSet<int>? collisionPairHolders = collider.CollisionPairHolders;
        if (collisionPairHolders != null)
        {
            foreach (int holderId in collisionPairHolders)
            {
                if (!TryGetColliderById(holderId, out LSCollider2D? holder)
                    || !holder!.TryRemoveCollisionPair(collider.Id, out CollisionPair2D? pair)
                    || pair == null)
                {
                    continue;
                }

                pair.MarkSeparated();
                _pairs.Remove(CreatePairKey(pair.Id1, pair.Id2));
                RecyclePair(pair);
            }
        }

        collider.ClearCollisionPairState();
        collider.ClearRuntimeRelationships();
    }
}
