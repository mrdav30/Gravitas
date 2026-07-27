//=======================================================================
// ConvexSweepHitPolicy.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Queries;

internal static class ConvexSweepHitPolicy
{
    internal static Vector3d ResolveHitNormal(
        LSCollider targetCollider,
        Vector3d point,
        Vector3d resultNormal,
        Vector3d fallbackNormal,
        Vector3d sweepDirection,
        Vector3d planarNormal,
        bool hasRefinedSurfaceNormal = false,
        bool hasMaterializedPoint = true)
    {
        Vector3d surfaceNormal = hasRefinedSurfaceNormal
            ? targetCollider.GetNormalAtPoint(point)
            : Vector3d.Zero;
        Vector3d normal = surfaceNormal.MagnitudeSquared > Fixed64.Epsilon
            ? surfaceNormal
            : planarNormal.MagnitudeSquared > Fixed64.Epsilon
                ? planarNormal
            : resultNormal.MagnitudeSquared > Fixed64.Epsilon
                ? resultNormal
                : fallbackNormal.MagnitudeSquared > Fixed64.Epsilon
                    ? fallbackNormal
                    : hasMaterializedPoint
                        ? targetCollider.GetNormalAtPoint(point)
                        : Vector3d.Zero;

        if (normal.MagnitudeSquared <= Fixed64.Epsilon)
        {
            return sweepDirection.MagnitudeSquared > Fixed64.Epsilon
                ? -sweepDirection.Normalized
                : Vector3d.Zero;
        }

        normal = normal.Normalized;
        return Vector3d.CompareProjection(
                normal,
                Vector3d.Zero,
                sweepDirection) > 0
            ? -normal
            : normal;
    }
}
