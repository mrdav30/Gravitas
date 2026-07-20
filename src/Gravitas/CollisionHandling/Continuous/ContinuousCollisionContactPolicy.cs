//=======================================================================
// ContinuousCollisionContactPolicy.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Bounds;
using Gravitas.Colliders;

namespace Gravitas.CollisionHandling;

internal static class ContinuousCollisionContactPolicy
{
    internal static Vector3d ResolveSweptSpherePoint(
        LSCollider target,
        Vector3d sphereCenterAtImpact,
        Vector3d direction,
        Fixed64 sphereRadius)
    {
        if (target is LSCapsuleCollider capsule)
        {
            Vector3d normal = capsule.GetNormalAtPoint(sphereCenterAtImpact);
            if (FixedSegment.TryGetSurfacePointOnCenteredCapsule(
                    sphereCenterAtImpact,
                    capsule.Center,
                    capsule.WorldAxis,
                    capsule.AxisHalfLength,
                    capsule.ScaledRadius,
                    normal,
                    out Vector3d targetSurfacePoint))
            {
                return targetSurfacePoint;
            }

            return sphereCenterAtImpact - normal * sphereRadius;
        }

        Vector3d centerDelta = sphereCenterAtImpact - target.Center;
        if (centerDelta.MagnitudeSquared <= Fixed64.Epsilon)
            return target.Center - direction * target.ScaledRadius;

        return target.ClosestPointOnSurface(sphereCenterAtImpact);
    }

    internal static Vector3d ResolveSweptSphereNormal(
        LSCollider target,
        Vector3d point,
        Vector3d sphereCenterAtImpact,
        Vector3d direction)
    {
        if (target is LSCapsuleCollider capsule)
            return capsule.GetNormalAtPoint(sphereCenterAtImpact);

        Vector3d fromPointToSphereCenter = sphereCenterAtImpact - point;
        if ((target is LSCuboidCollider || target is LSCylinderCollider || target is LSConeCollider)
            && fromPointToSphereCenter.MagnitudeSquared > Fixed64.Epsilon)
        {
            return fromPointToSphereCenter.Normalized;
        }

        Vector3d normal = target.GetNormalAtPoint(point);
        if (normal.MagnitudeSquared > Fixed64.Epsilon)
        {
            normal = normal.Normalized;
            if (target is LSMeshCollider && Vector3d.Dot(normal, direction) > Fixed64.Zero)
                return -normal;

            return normal;
        }

        if (fromPointToSphereCenter.MagnitudeSquared > Fixed64.Epsilon)
            return fromPointToSphereCenter.Normalized;

        return direction.MagnitudeSquared > Fixed64.Epsilon ? -direction.Normalized : Vector3d.Zero;
    }
}
