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

internal readonly struct MixedPairLifetimeToken
{
    internal MixedPairLifetimeToken(CollisionPairMixed pair)
    {
        Pair = pair;
        LifetimeVersion = pair.LifetimeVersion;
        Collider3D = new ColliderLifetimeToken(pair.Collider3D);
        Collider2D = new ColliderLifetimeToken2D(pair.Collider2D);
    }

    internal CollisionPairMixed Pair { get; }

    internal long LifetimeVersion { get; }

    private ColliderLifetimeToken Collider3D { get; }

    private ColliderLifetimeToken2D Collider2D { get; }

    internal bool IsCurrentLifetime => Pair.LifetimeVersion == LifetimeVersion
        && Collider3D.IsCurrentLifetime
        && Collider2D.IsCurrentLifetime;
}

internal sealed partial class GravitasMixedCollisionService
{
    internal void RemovePairsFor3DCollider(LSCollider collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        int removalStart = _pairsToRemove.Count;
        foreach (var pairEntry in _pairs)
        {
            CollisionPairMixed pair = pairEntry.Value;
            if (!ReferenceEquals(pair.Collider3D, collider))
                continue;

            _pairsToRemove.Add(new MixedPairLifetimeToken(pair));
        }

        RemoveQueuedPairs(removalStart, _pairsToRemove.Count);
    }

    internal void RemovePairsFor2DCollider(LSCollider2D collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        int removalStart = _pairsToRemove.Count;
        foreach (var pairEntry in _pairs)
        {
            CollisionPairMixed pair = pairEntry.Value;
            if (!ReferenceEquals(pair.Collider2D, collider))
                continue;

            _pairsToRemove.Add(new MixedPairLifetimeToken(pair));
        }

        RemoveQueuedPairs(removalStart, _pairsToRemove.Count);
    }

    internal bool RequireCollisionPair(LSCollider collider3D, LSCollider2D collider2D)
    {
        return collider3D.IsActive && !collider3D.IsDeactivationInProgress && collider2D.IsActive
            && collider3D.Shape != ColliderType.None && collider2D.Shape != ColliderType2D.None
            && (collider3D.Body != null || collider2D.Body != null)
            && !ReferenceEquals(collider3D.AgentOrNull, collider2D.AgentOrNull)
            && !collider3D.ExcludesMixedCollisionWith(collider2D)
            && ColliderCollisionFilter.AllowsPhysicalPair(collider3D, collider2D)
            && !_context.Physics.IsLayerCollisionDisabled(collider3D.Layer, collider2D.Layer);
    }

    private void ProcessCandidate(MixedColliderKey candidate, int frame)
    {
        if (!_context.Physics.TryGetColliderById(candidate.Collider3DId, out LSCollider? collider3D)
            || !_context.Physics2D.TryGetColliderById(candidate.Collider2DId, out LSCollider2D? collider2D))
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
            pair!.MarkResting(frame, contact);
            _mixedResponsePairs.Add(pair);
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
        int removalStart = _pairsToRemove.Count;
        foreach (var pairEntry in _pairs)
            _pairsToRemove.Add(new MixedPairLifetimeToken(pairEntry.Value));

        int removalEnd = _pairsToRemove.Count;
        try
        {
            for (int i = removalStart; i < removalEnd; i++)
            {
                MixedPairLifetimeToken token = _pairsToRemove[i];
                CollisionPairMixed pair = token.Pair;
                if (!IsCurrentPair(token))
                    continue;

                if (pair.LastFrame == frame)
                    continue;

                if (TryKeepUntouchedPair(pair, frame))
                    continue;

                RemoveCurrentPair(token);
            }
        }
        finally
        {
            if (removalStart == 0)
                _pairsToRemove.FastClear();
        }
    }

    private bool TryKeepUntouchedPair(CollisionPairMixed pair, int frame)
    {
        LSCollider collider3D = pair.Collider3D;
        LSCollider2D collider2D = pair.Collider2D;
        bool triggerPair = collider3D.IsTrigger || collider2D.IsTrigger;
        if ((!triggerPair && HasAwakeMovableParticipant(collider3D, collider2D))
            || !RequireCollisionPair(collider3D, collider2D)
            || !MixedBoundsOverlap(collider3D, collider2D)
            || !CollisionDetectionMixed.TryCollide(collider3D, collider2D, out MixedContact contact))
        {
            return false;
        }

        if (triggerPair)
        {
            pair.MarkColliding(frame, contact);
            return true;
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

    private void RemoveQueuedPairs(int removalStart, int removalEnd)
    {
        try
        {
            for (int i = removalStart; i < removalEnd; i++)
                RemoveCurrentPair(_pairsToRemove[i]);
        }
        finally
        {
            if (removalStart == 0)
                _pairsToRemove.FastClear();
        }
    }

    private bool IsCurrentPair(in MixedPairLifetimeToken token) =>
        token.IsCurrentLifetime
        && _pairs.TryGetValue(token.Pair.Key, out CollisionPairMixed currentPair)
        && ReferenceEquals(currentPair, token.Pair);

    private void RemoveCurrentPair(in MixedPairLifetimeToken token)
    {
        if (!IsCurrentPair(token))
            return;

        CollisionPairMixed pair = token.Pair;
        _pairs.Remove(pair.Key);
        pair.MarkSeparated();
        RecyclePair(pair);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void RecyclePair(CollisionPairMixed pair)
    {
        if (_context.Settings.PoolingEnabled && !pair.IsNotificationInProgress)
            _cachedPairs.Push(pair);
    }

}
