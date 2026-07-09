//=======================================================================
// GravitasPhysics2DService.Grounding.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.CollisionHandling;

namespace Gravitas;

public sealed partial class GravitasPhysics2DService
{
    private void RefreshGroundingFromDiscreteResponse(int frame)
    {
        foreach (SolidBody2D body in _dynamicBodies)
            body.BeginAutomaticGroundingRefresh();

        for (int i = 0; i < _discreteResponsePairs.Count; i++)
        {
            CollisionPair2D pair = _discreteResponsePairs[i];
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

        foreach (SolidBody2D body in _dynamicBodies)
            body.CompleteAutomaticGroundingRefresh();
    }

    internal static bool ShouldUseDiscreteGroundingPair(CollisionPair2D pair, int frame) =>
        pair.LastFrame == frame && !pair.ColliderA.IsTrigger && !pair.ColliderB.IsTrigger;

    internal static bool ShouldUseDiscreteGroundingManifold(ContactManifold2D manifold, int frame) =>
        manifold.HasContact && manifold.LastUpdatedFrame == frame;
}
