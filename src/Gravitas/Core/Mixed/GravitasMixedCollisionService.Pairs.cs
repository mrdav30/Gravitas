//=======================================================================
// GravitasMixedCollisionService.Pairs.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System.Runtime.CompilerServices;

namespace Gravitas;

internal sealed partial class GravitasMixedCollisionService
{
    internal void RemovePairsFor3DCollider(LSCollider collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        _pairsToRemove.FastClear();
        foreach (var pairEntry in _pairs)
        {
            CollisionPairMixed pair = pairEntry.Value;
            if (!ReferenceEquals(pair.Collider3D, collider))
                continue;

            pair.MarkSeparated();
            _pairsToRemove.Add(pairEntry.Key);
        }

        RemoveQueuedPairs();
    }

    internal void RemovePairsFor2DCollider(LSCollider2D collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        _pairsToRemove.FastClear();
        foreach (var pairEntry in _pairs)
        {
            CollisionPairMixed pair = pairEntry.Value;
            if (!ReferenceEquals(pair.Collider2D, collider))
                continue;

            pair.MarkSeparated();
            _pairsToRemove.Add(pairEntry.Key);
        }

        RemoveQueuedPairs();
    }

    internal bool RequireCollisionPair(LSCollider collider3D, LSCollider2D collider2D)
    {
        return collider3D.IsActive && collider2D.IsActive
            && collider3D.Shape != ColliderType.None && collider2D.Shape != ColliderType2D.None
            && (collider3D.Body != null || collider2D.Body != null)
            && !ReferenceEquals(collider3D.AgentOrNull, collider2D.AgentOrNull)
            && !collider3D.ExcludesMixedCollisionWith(collider2D)
            && !_context.Physics.IsLayerCollisionDisabled(collider3D.Layer, collider2D.Layer);
    }

    private void ProcessCandidate(MixedColliderKey candidate, int frame)
    {
        if (!_context.Physics.TryGetColliderById(candidate.Collider3DId, out LSCollider? collider3D)
            || !_context.Physics2D.TryGetColliderById(candidate.Collider2DId, out LSCollider2D? collider2D)
            || !RequireCollisionPair(collider3D!, collider2D!)
            || !MixedBoundsOverlap(collider3D!, collider2D!))
        {
            return;
        }

        LSCollider resolved3D = collider3D!;
        LSCollider2D resolved2D = collider2D!;
        bool hasPair = _pairs.TryGetValue(candidate.Key, out CollisionPairMixed pair);
        bool triggerPair = resolved3D.IsTrigger || resolved2D.IsTrigger;
        bool hasAwakeMovableParticipant = HasAwakeMovableParticipant(resolved3D, resolved2D);
        if (!hasPair && !triggerPair && !hasAwakeMovableParticipant)
            return;

        if (!CollisionDetectionMixed.TryCollide(resolved3D, resolved2D, out MixedContact contact))
            return;

        if (!triggerPair && !hasAwakeMovableParticipant)
        {
            if (hasPair)
            {
                pair!.MarkResting(frame, contact);
                _mixedResponsePairs.Add(pair);
            }

            return;
        }

        if (!hasPair)
        {
            pair = CreatePair(resolved3D, resolved2D);
            _pairs.Add(candidate.Key, pair);
        }

        pair!.MarkColliding(frame, contact);
        if (!triggerPair)
            _mixedResponsePairs.Add(pair);
    }

    private void CleanupUntouchedPairs(int frame)
    {
        _pairsToRemove.FastClear();
        foreach (var pairEntry in _pairs)
        {
            CollisionPairMixed pair = pairEntry.Value;
            if (pair.LastFrame == frame)
                continue;

            if (TryKeepRestingPair(pair, frame))
                continue;

            pair.MarkSeparated();
            _pairsToRemove.Add(pairEntry.Key);
        }

        RemoveQueuedPairs();
    }

    private bool TryKeepRestingPair(CollisionPairMixed pair, int frame)
    {
        LSCollider collider3D = pair.Collider3D;
        LSCollider2D collider2D = pair.Collider2D;
        if (collider3D.IsTrigger
            || collider2D.IsTrigger
            || HasAwakeMovableParticipant(collider3D, collider2D)
            || !RequireCollisionPair(collider3D, collider2D)
            || !MixedBoundsOverlap(collider3D, collider2D)
            || !CollisionDetectionMixed.TryCollide(collider3D, collider2D, out _))
        {
            return false;
        }

        pair.MarkResting(frame);
        return true;
    }

    private CollisionPairMixed CreatePair(LSCollider collider3D, LSCollider2D collider2D)
    {
        if (_context.Settings.PoolingEnabled && _cachedPairs.Count > 0)
        {
            CollisionPairMixed pair = _cachedPairs.Pop();
            pair.Initialize(collider3D, collider2D);
            return pair;
        }

        return new CollisionPairMixed(collider3D, collider2D);
    }

    private void RemoveQueuedPairs()
    {
        for (int i = 0; i < _pairsToRemove.Count; i++)
        {
            ulong key = _pairsToRemove[i];
            if (_pairs.TryGetValue(key, out CollisionPairMixed pair))
                RecyclePair(pair);

            _pairs.Remove(key);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void RecyclePair(CollisionPairMixed pair)
    {
        if (_context.Settings.PoolingEnabled)
            _cachedPairs.Push(pair);
    }

}
