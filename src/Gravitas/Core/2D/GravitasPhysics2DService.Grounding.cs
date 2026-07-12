//=======================================================================
// GravitasPhysics2DService.Grounding.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.Colliders;
using Gravitas.CollisionHandling;

namespace Gravitas;

public sealed partial class GravitasPhysics2DService
{
    private readonly SwiftCollections.SwiftList<GroundingBodySnapshotEntry> _groundingBodySnapshot = new();
    private readonly SwiftCollections.SwiftList<GroundingPairSnapshotEntry> _groundingPairSnapshot = new();

    private readonly struct GroundingBodySnapshotEntry
    {
        private readonly ColliderLifetimeToken2D _registration;

        internal GroundingBodySnapshotEntry(SolidBody2D body)
        {
            Body = body;
            _registration = new ColliderLifetimeToken2D(body.Collider);
        }

        internal SolidBody2D Body { get; }

        internal bool IsActive => _registration.IsActive;
    }

    private readonly struct GroundingPairSnapshotEntry
    {
        private readonly long _lifetimeVersion;

        internal GroundingPairSnapshotEntry(CollisionPair2D pair)
        {
            Pair = pair;
            _lifetimeVersion = pair.LifetimeVersion;
        }

        internal CollisionPair2D Pair { get; }

        internal bool IsCurrentLifetime => Pair.LifetimeVersion == _lifetimeVersion;
    }

    private void RefreshGroundingFromDiscreteResponse(int frame)
    {
        int snapshotStart = _groundingBodySnapshot.Count;
        _groundingBodySnapshot.EnsureCapacity(snapshotStart + _dynamicBodies.Count);
        foreach (SolidBody2D body in _dynamicBodies)
            _groundingBodySnapshot.Add(new GroundingBodySnapshotEntry(body));
        int snapshotEnd = _groundingBodySnapshot.Count;

        int pairSnapshotStart = _groundingPairSnapshot.Count;
        _groundingPairSnapshot.EnsureCapacity(pairSnapshotStart + _discreteResponsePairs.Count);
        for (int i = 0; i < _discreteResponsePairs.Count; i++)
            _groundingPairSnapshot.Add(new GroundingPairSnapshotEntry(_discreteResponsePairs[i]));
        int pairSnapshotEnd = _groundingPairSnapshot.Count;

        try
        {
            for (int i = snapshotStart; i < snapshotEnd; i++)
            {
                GroundingBodySnapshotEntry registration = _groundingBodySnapshot[i];
                if (!registration.IsActive)
                    continue;

                registration.Body.BeginAutomaticGroundingRefresh();
            }

            for (int i = pairSnapshotStart; i < pairSnapshotEnd; i++)
            {
                GroundingPairSnapshotEntry registration = _groundingPairSnapshot[i];
                if (!registration.IsCurrentLifetime)
                    continue;

                CollisionPair2D pair = registration.Pair;
                if (!ShouldUseDiscreteGroundingPair(pair, frame))
                    continue;

                ContactManifold2D manifold = pair.Manifold;
                if (!ShouldUseDiscreteGroundingManifold(manifold, frame))
                    continue;

                SolidBody2D? bodyA = pair.ColliderA.Body;
                SolidBody2D? bodyB = pair.ColliderB.Body;
                for (int contactIndex = 0; contactIndex < manifold.Count; contactIndex++)
                {
                    ManifoldContact2D contact = manifold[contactIndex];
                    bodyA?.TryAcceptContactGroundCandidate(pair.ColliderA, pair.ColliderB, contact, ownColliderIsA: true);
                    bodyB?.TryAcceptContactGroundCandidate(pair.ColliderB, pair.ColliderA, contact, ownColliderIsA: false);
                }
            }

            for (int i = snapshotStart; i < snapshotEnd; i++)
            {
                GroundingBodySnapshotEntry registration = _groundingBodySnapshot[i];
                if (!registration.IsActive)
                    continue;

                registration.Body.CompleteAutomaticGroundingRefresh();
            }
        }
        finally
        {
            if (pairSnapshotStart == 0)
            {
                _groundingPairSnapshot.FastClear();
            }
            else
            {
                while (_groundingPairSnapshot.Count > pairSnapshotStart)
                    _groundingPairSnapshot.RemoveAt(_groundingPairSnapshot.Count - 1);
            }

            if (snapshotStart == 0)
            {
                _groundingBodySnapshot.FastClear();
            }
            else
            {
                while (_groundingBodySnapshot.Count > snapshotStart)
                    _groundingBodySnapshot.RemoveAt(_groundingBodySnapshot.Count - 1);
            }
        }
    }

    internal static bool ShouldUseDiscreteGroundingPair(CollisionPair2D pair, int frame) =>
        pair.LastFrame == frame && !pair.ColliderA.IsTrigger && !pair.ColliderB.IsTrigger;

    internal static bool ShouldUseDiscreteGroundingManifold(ContactManifold2D manifold, int frame) =>
        manifold.HasContact && manifold.LastUpdatedFrame == frame;
}
