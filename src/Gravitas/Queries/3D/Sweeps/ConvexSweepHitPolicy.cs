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
        Vector3d sweepDirection)
    {
        Vector3d normal = targetCollider.GetNormalAtPoint(point);
        if (normal.MagnitudeSquared <= Fixed64.Epsilon)
            normal = resultNormal.MagnitudeSquared > Fixed64.Epsilon ? resultNormal : fallbackNormal;

        if (normal.MagnitudeSquared <= Fixed64.Epsilon)
            return sweepDirection.MagnitudeSquared > Fixed64.Epsilon ? -sweepDirection : Vector3d.Zero;

        normal = normal.Normalized;
        return Vector3d.Dot(normal, sweepDirection) > Fixed64.Zero ? -normal : normal;
    }
}
