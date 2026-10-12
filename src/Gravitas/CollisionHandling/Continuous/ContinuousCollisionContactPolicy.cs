//=======================================================================
// ContinuousCollisionContactPolicy.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;

namespace Gravitas.CollisionHandling;

internal static class ContinuousCollisionContactPolicy
{
    internal static bool TryResolveSweptSphereContact(
        LSCollider target,
        Vector3d sphereCenterAtImpact,
        Vector3d direction,
        out ContactAnchor targetAnchor,
        out Vector3d normal)
    {
        if (target is LSSphereCollider or LSCapsuleCollider)
        {
            // Keep the same reducer's witness; an inverse-normal round trip can shift it.
            FixedPointAnchor surface = target.GetClosestSurfaceAnchor(
                sphereCenterAtImpact, out normal);
            targetAnchor = new ContactAnchor(surface);
            return true;
        }

        // Solid-volume normals remain outward for contained starts: CCD must
        // permit escape and block deeper motion. Signed distance is not a flip.
        if (target is LSCylinderCollider cylinder)
        {
            if (!FixedSegment.TryGetClosestCenteredFiniteCylinderSurfaceAnchor(
                    sphereCenterAtImpact,
                    cylinder.Center,
                    cylinder.Rotation,
                    Vector3d.Up,
                    cylinder.Height,
                    cylinder.ScaledRadius,
                    Vector3d.Right,
                    out FixedPointAnchor targetPoint,
                    out normal,
                    out _))
            {
                targetAnchor = default;
                normal = default;
                return false;
            }

            targetAnchor = new ContactAnchor(targetPoint);
            return true;
        }

        if (target is LSConeCollider cone)
        {
            if (!FixedSegment.TryGetClosestCenteredFiniteConeSurfaceAnchor(
                    sphereCenterAtImpact,
                    cone.Center,
                    cone.Rotation,
                    Vector3d.Up,
                    cone.Height,
                    cone.ScaledRadius,
                    Vector3d.Right,
                    out FixedPointAnchor targetPoint,
                    out normal,
                    out _))
            {
                targetAnchor = default;
                normal = default;
                return false;
            }

            targetAnchor = new ContactAnchor(targetPoint);
            return true;
        }

        if (target is LSCuboidCollider cuboid)
        {
            FixedPointAnchor targetPoint = cuboid.GetClosestSurfaceAnchor(
                sphereCenterAtImpact, out normal, out _);
            targetAnchor = new ContactAnchor(targetPoint);
            return true;
        }

        if (target is LSMeshCollider mesh)
        {
            mesh.FindClosestPointAnchor(
                sphereCenterAtImpact,
                out FixedPointAnchor meshPoint,
                out normal);
            normal = ResolveSurfaceNormal(meshPoint, sphereCenterAtImpact, normal);
            targetAnchor = new ContactAnchor(meshPoint);
            return true;
        }

        if (target is LSCompoundCollider compound)
        {
            FixedPointAnchor surfacePoint = compound.GetClosestSurfaceAnchor(
                sphereCenterAtImpact, out normal, out LSCollider surfaceOwner);
            if (surfaceOwner is LSMeshCollider)
                normal = ResolveSurfaceNormal(surfacePoint, sphereCenterAtImpact, normal);
            // Keep the selected part's witness and outward primitive normal.
            // Re-querying a materialized point can select another part at a
            // shared boundary or lose an otherwise valid far-domain anchor.
            targetAnchor = new ContactAnchor(surfacePoint);
            return true;
        }

        Vector3d point = target.ClosestPointOnSurface(sphereCenterAtImpact);
        normal = ResolveLegacyNormal(target, point, sphereCenterAtImpact, direction);
        var worldPoint = new FixedPointAnchor(
            point,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        if (worldPoint.TryGetLocalPointIn(
                target.Center,
                target.Rotation,
                out Vector3d localPoint))
        {
            targetAnchor = new ContactAnchor(
                target.Center,
                target.Rotation,
                localPoint);
            return true;
        }

        targetAnchor = default;
        normal = default;
        return false;
    }

    internal static Vector3d ResolveSurfaceNormal(
        in FixedPointAnchor surfacePoint, Vector3d sourceCenter, Vector3d outwardNormal)
    {
        // Exterior apices and two-sided meshes need the exact closest-feature
        // direction; only coincidence retains the selected feature's normal.
        var center = new FixedPointAnchor(sourceCenter, FixedQuaternion.Identity, Vector3d.Zero);
        Vector3d normal = WidePointAnchor3d.GetDirection(center, surfacePoint);
        return normal == Vector3d.Zero ? outwardNormal : normal;
    }

    private static Vector3d ResolveLegacyNormal(
        LSCollider target,
        Vector3d point,
        Vector3d sphereCenterAtImpact,
        Vector3d direction)
    {
        Vector3d fromPointToSphereCenter = sphereCenterAtImpact - point;
        Vector3d normal = target.GetNormalAtPoint(point);
        if (normal.MagnitudeSquared > Fixed64.Epsilon)
            return normal.Normalized;

        if (fromPointToSphereCenter.MagnitudeSquared > Fixed64.Epsilon)
            return fromPointToSphereCenter.Normalized;

        return direction.MagnitudeSquared > Fixed64.Epsilon
            ? -direction.Normalized
            : Vector3d.Zero;
    }
}
