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
        Vector3d fallbackNormal = direction.MagnitudeSquared > Fixed64.Epsilon
            ? -direction.Normalized
            : Vector3d.Right;

        if (target is LSCapsuleCollider capsule)
        {
            normal = capsule.GetNormalAtPoint(sphereCenterAtImpact);
            Vector3d localNormal =
                capsule.Rotation.Inverse().Rotate(normal).Normalized;
            targetAnchor = new ContactAnchor(
                FixedSegment.GetSurfaceAnchorOnCenteredCapsule(
                    sphereCenterAtImpact,
                    capsule.Center,
                    capsule.Rotation,
                    Vector3d.Up,
                    capsule.AxisLength,
                    capsule.ScaledRadius,
                    localNormal));
            return true;
        }

        if (target is LSSphereCollider sphere)
        {
            normal = Vector3d.GetDirection(sphere.Center, sphereCenterAtImpact);
            if (normal == Vector3d.Zero)
                normal = fallbackNormal;
            targetAnchor = new ContactAnchor(
                FixedSegment.GetCenteredCapsuleSupportAnchor(
                    sphere.Center,
                    sphere.Rotation,
                    Fixed64.Zero,
                    sphere.ScaledRadius,
                    normal));
            return true;
        }

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
                    out Vector3d outwardNormal,
                    out Fixed64 signedDistance))
            {
                targetAnchor = default;
                normal = default;
                return false;
            }

            targetAnchor = new ContactAnchor(targetPoint);
            normal = signedDistance < Fixed64.Zero ? -outwardNormal : outwardNormal;
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
                    out Vector3d outwardNormal,
                    out Fixed64 signedDistance))
            {
                targetAnchor = default;
                normal = default;
                return false;
            }

            targetAnchor = new ContactAnchor(targetPoint);
            normal = signedDistance < Fixed64.Zero ? -outwardNormal : outwardNormal;
            return true;
        }

        if (target is LSCuboidCollider cuboid)
        {
            FixedPointAnchor targetPoint =
                cuboid.OrientedBox.GetClosestPointAnchor(sphereCenterAtImpact);
            var sphereCenter = new FixedPointAnchor(
                sphereCenterAtImpact,
                FixedQuaternion.Identity,
                Vector3d.Zero);
            normal = sphereCenter.TryGetOffsetFrom(
                    targetPoint,
                    out Vector3d fromSurface)
                && fromSurface.MagnitudeSquared > Fixed64.Epsilon
                    ? fromSurface.Normalized
                    : cuboid.GetNormalAtPoint(sphereCenterAtImpact);
            targetAnchor = new ContactAnchor(targetPoint);
            return true;
        }

        if (target is LSMeshCollider mesh)
        {
            mesh.FindClosestPointAnchor(
                sphereCenterAtImpact,
                out FixedPointAnchor meshPoint,
                out normal);
            normal = ResolveMeshSphereNormal(meshPoint, sphereCenterAtImpact, normal);
            targetAnchor = new ContactAnchor(meshPoint);
            return true;
        }

        if (target is LSCompoundCollider compound)
        {
            FixedPointAnchor surfacePoint = compound.GetClosestSurfaceAnchor(
                sphereCenterAtImpact, out normal, out LSCollider surfaceOwner);
            if (surfaceOwner is LSMeshCollider)
                normal = ResolveMeshSphereNormal(surfacePoint, sphereCenterAtImpact, normal);
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

    private static Vector3d ResolveMeshSphereNormal(
        in FixedPointAnchor surfacePoint,
        Vector3d sphereCenter,
        Vector3d faceNormal)
    {
        // Mesh sweeps are two-sided surface queries. Use the closest feature's
        // direction even at edges, vertices and sub-epsilon separations; travel
        // cannot turn an overhead contact into support. Only exact coincidence
        // retains the selected triangle's authored normal and stable tie order.
        var center = new FixedPointAnchor(sphereCenter, FixedQuaternion.Identity, Vector3d.Zero);
        Vector3d normal = WidePointAnchor3d.GetDirection(center, surfacePoint);
        return normal == Vector3d.Zero ? faceNormal : normal;
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
