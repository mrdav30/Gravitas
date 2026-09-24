using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Queries;

internal static partial class QueryDetection2D
{
    internal static bool TryFindSupport(in Physics2DSupportQuery request, Vector2d end,
        LSCollider2D collider, out Physics2DHit hit)
    {
        if (collider is not LSCompoundCollider2D compound)
            return TryFindLeafSupport(request, end, collider, out hit);

        bool found = false;
        Physics2DHit best = default;
        for (int i = 0; i < compound.PartCount; i++)
        {
            if (TryFindLeafSupport(request, end, compound.GetPartCollider(i), out Physics2DHit candidate))
                TryKeepEarlierHit(candidate, ref found, ref best);
        }
        hit = found ? new Physics2DHit(compound, best.Anchor, best.Normal, best.Distance) : default;
        return found;
    }

    private static bool TryFindLeafSupport(in Physics2DSupportQuery request, Vector2d end,
        LSCollider2D collider, out Physics2DHit hit)
    {
        // The public generic sweep intentionally rejects tiny segments. Support has
        // an authored direction/distance and must also recognize stationary contact.
        bool found = TryOverlapCircle(request.Center, request.Radius, collider, out hit);
        if (found)
            hit = new Physics2DHit(collider, hit.Anchor, hit.Normal, Fixed64.Zero);
        else
        {
            found = collider switch
            {
                LSCircleCollider2D circle => TrySweepCircleCircle(request.Center, end, request.Distance,
                    request.Radius, circle, out hit),
                LSCapsuleCollider2D capsule => TrySweepCircleCapsule(request.Center, end, request.Distance,
                    request.Radius, capsule, out hit),
                _ => TrySweepCircleConvex(request.Center, -request.Up, request.Distance,
                    request.Radius, collider, out hit)
            };
        }
        if (found && Vector2d.Dot(hit.Normal, request.Up) >= request.MinimumNormalDot)
            return true;
        hit = default;
        return false;
    }
}
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
