//=======================================================================
// QueryDetection2D.SupportQuery.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Queries;

/// <content>Finds eligible support hits on planar collider shapes and compound parts.</content>
internal static partial class QueryDetection2D
{
    internal static PhysicsQueryExecutionStatus ReserveSupportGeometry(LSCollider2D collider, ref PhysicsQueryMeter meter)
    {
        if (collider is not LSCompoundCollider2D compound)
            return ReserveLeafSupportGeometry(collider, ref meter);

        int partCount = compound.PartCount;
        // Authored compounds are flat. Reserve one visit here and one in TryFindSupport for each part.
        if (!meter.ChargeGeometry(2UL * (ulong)partCount))
            return PhysicsQueryExecutionStatus.WorkBudgetExceeded;

        for (int i = 0; i < partCount; i++)
        {
            var status = ReserveLeafSupportGeometry(compound.GetPartCollider(i), ref meter);
            if (status != PhysicsQueryExecutionStatus.Completed)
                return status;
        }
        return PhysicsQueryExecutionStatus.Completed;
    }

    private static PhysicsQueryExecutionStatus ReserveLeafSupportGeometry(LSCollider2D collider, ref PhysicsQueryMeter meter)
    {
        // One fixed centered-axis overlap kernel, then the circle quadratic or
        // capsule's finite side and two cap intervals. Allow a fixed witness step.
        ulong cost = collider switch
        {
            LSCircleCollider2D => 3,
            LSCapsuleCollider2D => 5,
            LSAABBoxCollider2D => ConvexSupportGeometry(4) + 8,
            // Compounds contain only the four native planar leaves.
            _ => ConvexSupportGeometry(((LSPolygonCollider2D)collider).Count)
        };
        return meter.ChargeGeometry(cost)
            ? PhysicsQueryExecutionStatus.Completed
            : PhysicsQueryExecutionStatus.WorkBudgetExceeded;
    }

    private static ulong ConvexSupportGeometry(int vertexCount)
    {
        // Circle contact tests up to N edge axes with N projections each, visits
        // N closest-vertex candidates, and projects N vertices on up to two more
        // axes. Depth refinement is at most 63 steps in the nonnegative raw domain.
        // On contact, support reduction takes at most three passes and containment
        // at most three (including degenerate bounds). Otherwise, the sweep tests
        // N edges: four endpoint sweeps with three features each, plus an axis
        // intersection per edge. Its witness reduction takes at most three passes.
        // Reserve the larger continuation: 17N instead of 6N, plus fixed setup
        // and witness work. The box also writes four offsets per kernel call.
        // N originates in an int-sized span, so N² + 21N + 67 fits in ulong.
        ulong vertices = (ulong)vertexCount;
        return vertices * vertices + 21UL * vertices + 63UL + 4UL;
    }

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
