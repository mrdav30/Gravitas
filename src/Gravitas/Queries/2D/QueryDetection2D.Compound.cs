//=======================================================================
// QueryDetection2D.Compound.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Queries;

internal static partial class QueryDetection2D
{
    private static bool TryRaycastCompound(
        Vector2d start,
        Vector2d end,
        LSCompoundCollider2D compound,
        out Physics2DHit hit)
    {
        bool found = false;
        Physics2DHit best = default;

        for (int i = 0; i < compound.PartCount; i++)
        {
            LSCollider2D part = compound.GetPartCollider(i);
            if (!TryRaycast(start, end, part, out Physics2DHit candidate))
                continue;

            TryKeepEarlierHit(candidate, ref found, ref best);
        }

        if (!found)
        {
            hit = default;
            return false;
        }

        hit = new Physics2DHit(compound, best.Anchor, best.Normal, best.Distance);
        return true;
    }

    private static bool TrySweepCircleCompound(
        Vector2d start,
        Vector2d end,
        Fixed64 radius,
        LSCompoundCollider2D compound,
        out Physics2DHit hit)
    {
        bool found = false;
        Physics2DHit best = default;

        for (int i = 0; i < compound.PartCount; i++)
        {
            LSCollider2D part = compound.GetPartCollider(i);
            if (!TrySweepCircle(start, end, radius, part, out Physics2DHit candidate))
                continue;

            TryKeepEarlierHit(candidate, ref found, ref best);
        }

        if (!found)
        {
            hit = default;
            return false;
        }

        hit = new Physics2DHit(compound, best.Anchor, best.Normal, best.Distance);
        return true;
    }

    private static bool TrySweepMoverCompound(
        LSCompoundCollider2D mover,
        Vector2d displacement,
        LSCollider2D target,
        out Physics2DHit hit)
    {
        bool found = false;
        Physics2DHit best = default;

        for (int i = 0; i < mover.PartCount; i++)
        {
            LSCollider2D part = mover.GetPartCollider(i);
            if (!TrySweepMoverShape(part, displacement, target, out Physics2DHit candidate))
                continue;

            TryKeepEarlierHit(candidate, ref found, ref best);
        }

        hit = best;
        return found;
    }

    private static bool TrySweepMoverAgainstCompound(
        LSCollider2D mover,
        Vector2d displacement,
        LSCompoundCollider2D target,
        out Physics2DHit hit)
    {
        bool found = false;
        Physics2DHit best = default;

        for (int i = 0; i < target.PartCount; i++)
        {
            LSCollider2D part = target.GetPartCollider(i);
            if (!TrySweepMoverShape(mover, displacement, part, out Physics2DHit candidate))
                continue;

            TryKeepCloserHit(candidate, ref found, ref best);
        }

        if (!found)
        {
            hit = default;
            return false;
        }

        hit = new Physics2DHit(target, best.Anchor, best.Normal, best.Distance);
        return true;
    }
}
